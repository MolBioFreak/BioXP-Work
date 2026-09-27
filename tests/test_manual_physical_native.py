"""Ordinary native -> canonical SQLite finite owner -> real source/transport."""
from contextlib import nullcontext
from types import SimpleNamespace
import time
import socket
_SOCKET = socket.socket
import pytest
from tests.test_oem_pipette_calibration import rig
from bioxp.manual_pipetting import compile_manual_pipetting, bind_manual_physical_handler, manual_physical_plan
from bioxp.protocols.models import ProtocolActionKind, ProtocolDocument
from bioxp.protocols.executor import ProtocolExecutor
from bioxp.oem_deck_movement import make_wp8_operation_executor


@pytest.mark.parametrize("step", [
    {"operation": "load_tip", "tray": 0, "well": "A1"},
    {"operation": "load_tip", "tray": 1, "well": "C1"},
    {"operation": "load_tip", "tray": 1, "well": "A1", "overpress": 1},
    {"operation": "measure_fluid_height", "speed": True},
    {"operation": "measure_fluid_height", "position": 90000},
    {"operation": "calibrate"},
])
def test_invalid_manual_physical_params(step):
    with pytest.raises(ValueError):
        compile_manual_pipetting({"protocol_id": "invalid", "steps": [step]})
    with pytest.raises(ValueError):
        manual_physical_plan(step)


@pytest.mark.parametrize("operation", ["load_tip", "measure_fluid_height"])
@pytest.mark.parametrize("failure", [None, "native", "timestamps"])
def test_route_ready_finite_owner(rig, monkeypatch, tmp_path, operation, failure):
    def local_socket(family=socket.AF_INET, *args, **kwargs):
        if family != socket.AF_UNIX:
            pytest.fail("network/hardware socket attempted")
        return _SOCKET(family, *args, **kwargs)
    monkeypatch.setattr(socket, "socket", local_socket)
    p, store = rig.provider, rig.store
    stamps = p.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    store.bind_workflow_dispatcher(lambda command: None)
    # Captured persisted source location deliberately differs from tray ordinal.
    monkeypatch.setattr(p, "_load_state", lambda: {"machine_status": {"constructed_tip_trays": [{"location": 10}]}})
    monkeypatch.setattr("bioxp.oem_machine_bundle.get_active_oem_machine_snapshot", lambda:
        SimpleNamespace(config_sections={"offsets": {"m_Z_MOTOR_MAX_CURRENT_DOWN": 17}}))
    for driver in rig.drivers:
        driver.stamp = None if failure == "timestamps" else time.monotonic() + .1
    if failure == "native":
        if operation == "load_tip":
            rig.native.exception = "x"
        else:
            rig.drivers[0].failed = True
    store.admit_workflow(command_id="parent", idempotency_key="parent", plan_fingerprint="fixture",
        requested_inputs={"bundle": {}}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
    assert store.claim_next()["command_id"] == "parent"
    wp8 = make_wp8_operation_executor(provider_getter=lambda: p, command_store=store)
    admission = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1}, "board4_authority": {"active_board_epoch": 1}}}
    children, results = [], []
    def execute(plan, action, state):
        admitted = store.admit_internal_wp8_operation(plan["operation"], inputs={}, state=admission,
            idempotency_key=action.action_id, prepared_plan=plan)
        cid = admitted["command_id"]; children.append(cid)
        claimed = store.claim_next()
        assert claimed["command_id"] == cid
        from bioxp.operator_controls import _DISPATCH_CONTEXT
        token = _DISPATCH_CONTEXT.set({"operator_command_id": cid, "action_id": "oem.deck._finite_operation"})
        try:
            result = wp8(command_id=cid, plan=plan)
        finally:
            _DISPATCH_CONTEXT.reset(token)
        results.append(result)
        store.finish(cid, status="completed", payload={"response": result}, claimed=claimed)
        return result
    from bioxp.pipette.receipts import PipetteReceiptStore
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    bind_collection_test_identity(monkeypatch)
    receipts_store = PipetteReceiptStore(tmp_path)
    handler = bind_manual_physical_handler(command_store=store, execute_plan=execute,
        require_motion_ready=lambda: None, provider_getter=lambda: p, receipt_store_getter=lambda: receipts_store)
    step = ({"operation": "load_tip", "tray": 1, "well": "B2", "overpress": True, "lift_z": True}
            if operation == "load_tip" else {"operation": operation})
    doc = compile_manual_pipetting({"protocol_id": "native-manual", "steps": [step, step]})
    assert ProtocolDocument.from_payload(doc.to_payload()) == doc
    assert all(a.oem_opcode is None for a in doc.stages[0].actions)
    from bioxp.services.protocol_service import _workflow_resources
    assert {"axis:x", "axis:y", "axis:z", "pipette"} <= set(_workflow_resources(doc))
    state = ProtocolExecutor(dry_run=False, job_id="parent",
        before_native_entry=lambda identity, state: store.assert_workflow_current("parent"), handlers={
        ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL: handler}).execute(doc)
    if failure == "native":
        assert not state.completed
        assert len(children) == 1
        evidence = store.wp8_operation_evidence(children[0])
        assert "failed" in repr(evidence) or "ambiguous" in repr(evidence)
        if operation == "measure_fluid_height":
            assert any(e[0] == "TR" for e in rig.events)
            assert not any(e[0] == "dispense" for e in rig.events)
        else:
            assert not any(e[0] == "home_z" for e in rig.events)
        return
    import json
    assert state.completed, [json.loads(store.wp8_operation_evidence(cid)["children"][0]["terminal_evidence_json"]).get("result", {}).get("error") for cid in children]
    assert len(children) == 2
    journal = receipts_store.read(limit=100)
    assert len(journal) == (2 if operation == "load_tip" else 6)
    claims = receipts_store.connection.execute("SELECT command_id, source_identity_json FROM pipette_operations").fetchall()
    assert len({row["command_id"] for row in claims}) == len(journal)
    assert not set(children).intersection(row["command_id"] for row in claims)
    assert all(json.loads(row["source_identity_json"])["parent_operator_command_id"] in children for row in claims)
    assert all("oem_opcode" not in repr(row["requested_inputs"]) for row in journal)
    receipts = [store.wp8_operation_evidence(cid) for cid in children]
    assert all(len(r["children"]) == 1 for r in receipts)
    source = results[0]["completed_children"][0]["result"]
    assert source["ok"] and source["calibration_persisted"] is False
    if operation == "load_tip":
        assert source["location"] == 10
        assert source["lost_steps"] == -101
        assert results[0]["lost_steps"] == -101
        assert store.deck_semantic_state()["tip_dirty"] is False
        assert store.deck_semantic_state()["tip_loaded"] is False
    else:
        assert source["position_steps"] == 88000
        assert results[0]["source_return"] == results[0]["position_steps"] == 88000
        assert source["timing"] is None if failure == "timestamps" else source["timing"] is not None
        br = [i for i,e in enumerate(rig.events) if e[0] == "BR"][:4]
        z = next(i for i,e in enumerate(rig.events) if e[0] == "move" and e[1:3] == ("z", 92015))
        wait = next(i for i,e in enumerate(rig.events) if e[0] == "wait" and e[2].startswith("BR"))
        assert max(br) < z < wait
