"""Connected finite zOffset scan: real provider, SQLite owners, transport-replaced IO."""
import asyncio
import socket
from types import SimpleNamespace

import pytest

from tests.test_oem_pipette_calibration import rig
from bioxp.oem_deck_movement import compile_finite_plate_operation
from bioxp.manual_pipetting import compile_manual_pipetting, manual_physical_plan
from bioxp.protocols.models import ProtocolActionKind
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.services.pipette_service import run_pipette_operation

_SOCKET = socket.socket


def test_typed_manual_scan_compiles_one_finite_child_without_execution():
    step = {"operation": "source_fluid_offset", "plate": "RC", "speed": 300,
            "transfer_fluid": False, "skip_steps": 1}
    doc = compile_manual_pipetting({"protocol_id": "manual-fluid", "steps": [step]})
    action = doc.stages[0].actions[0]
    assert action.kind == ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL
    plan = manual_physical_plan(action.params)
    assert [(child["operation"], child["arguments"]) for child in plan["children"]] == [
        ("sourceFluidOffset", {key: value for key, value in step.items() if key != "operation"})]
    with pytest.raises(ValueError):
        manual_physical_plan({**step, "tip_loaded": True})


def test_manual_scan_handler_supplies_oem_constructor_logical_wells(monkeypatch):
    from contextlib import nullcontext
    from bioxp.manual_pipetting import bind_manual_physical_handler
    from bioxp.protocols.runtime_state import ProtocolRuntimeState
    doc = compile_manual_pipetting({"protocol_id": "manual-rc", "steps": [{
        "operation": "source_fluid_offset", "plate": "RC", "speed": 300,
        "transfer_fluid": True, "skip_steps": 12}]})
    state = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id="manual-rc-owner")
    assert not state.source_model.trays
    action = doc.stages[0].actions[0]
    provider = SimpleNamespace()
    store = SimpleNamespace(workflow_context=lambda *a, **kw: nullcontext(),
                            assert_workflow_current=lambda *a: None)
    monkeypatch.setattr("bioxp.runtime_state.get_active_oem_runtime_state_store", lambda: object())
    monkeypatch.setattr("bioxp.pipette.manual_settings.read_pipette_operation_settings",
                        lambda _: {"runtime_values": {"LogPressure": False,
                                                       "CheckForStaticTipLoss": False}})
    def execute(plan, action, runtime):
        assert runtime is state
        assert provider._manual_pipette_source_state is state
        assert provider._manual_pipette_source_settings["LogPressure"] is False
        wells = state.source_model.trays["REAGENT_PLATE"].wells
        assert len(wells) == 96 and wells[0].volume == 0.0
        assert wells[0].capacity == 1081.0
        assert state.source_model.trays["TROUGH"].location == 16
        assert plan["children"][0]["operation"] == "sourceFluidOffset"
        return {"ok": True, "source_return": 88000}
    handle = bind_manual_physical_handler(command_store=store, execute_plan=execute,
        require_motion_ready=lambda: None, provider_getter=lambda: provider,
        receipt_store_getter=lambda: None)
    assert handle(action, state)["source_return"] == 88000


@pytest.mark.parametrize("plate,transfer_fluid", [("STRIP", False), ("RC", True)])
def test_inline_scan_under_one_owner_with_distinct_source_occurrences(rig, monkeypatch, tmp_path,
                                                                      plate, transfer_fluid):
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    bind_collection_test_identity(monkeypatch)
    monkeypatch.setattr(socket, "socket", lambda family=socket.AF_INET, *a, **kw:
        _SOCKET(family, *a, **kw) if family == socket.AF_UNIX else pytest.fail("network socket"))
    monkeypatch.setattr("bioxp.oem_machine_bundle.get_active_oem_machine_snapshot", lambda:
        SimpleNamespace(operation_parameters={"Mode": "WebMode"}, config_sections={"offsets": {
            "m_Z_MOTOR_MAX_CURRENT_DOWN": 17}},
            fields={"machine.camera_installed": SimpleNamespace(value=False)}, camera_calibrated=False))
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_oem_parity_config",
                        lambda _: SimpleNamespace(blockers=[], values={}))
    p = rig.provider
    p._wp8_execution_fence_checker = lambda *a, **kw: None
    p._oem_pipette_rgb_writer = lambda *a: {"ok": True}
    from bioxp.protocols.runtime_state import ProtocolSourceModel, SourceTray, SourceWell
    model = ProtocolSourceModel()
    if transfer_fluid:
        model.trays["REAGENT_PLATE"] = SourceTray("REAGENT_PLATE", 3,
            [SourceWell(None, 0.0, 200.0) for _ in range(96)])
    p._manual_pipette_source_state = SimpleNamespace(source_model=model)
    p._manual_pipette_source_settings = {"LogPressure": False, "CheckForStaticTipLoss": False}
    p._read_constructed_tip_tray = lambda i: {"tip_type": "T50" if i == 0 else "T200",
        "location": 7 + i, "construction_id": "fixture", "tip_available": True,
        "occupancy": [True] * 96}
    p.bind_tip_tray_state_reader(rig.store.tip_tray_state)
    p.bind_tip_tray_state_publisher(rig.store.publish_tip_tray_transition)
    receipts = PipetteReceiptStore(tmp_path)
    async def inline(label, body, *, timeout_s):
        return body()
    def receipt(name, call, command_id, identity, inputs):
        return asyncio.run(run_pipette_operation(name, call, get_transport=lambda: rig.group,
            run_blocking=inline, receipt_store=receipts,
            requested_inputs={"source_occurrence_id": identity["source_identity"]},
            runtime_binding={"idempotency_key": identity["source_identity"],
                "entrypoint_id": "protocol.pipette_manual_physical", "caller_class": "protocol_manual",
                "parent_operator_command_id": command_id}))
    p._manual_pipette_receipt_runner = receipt
    rig.store.admit_workflow(command_id="offset-parent", idempotency_key="offset-parent",
        plan_fingerprint="offset", requested_inputs={}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
    rig.store.claim_next()
    p.publish_tip_tray_transition(tray_id=0, transition="construct",
        operation_id="offset-parent:construct", command_id="offset-parent",
        provenance={"source_operation": "ClassMachineStatus.constructor"})
    for transport in rig.group._transports:
        transport._tip_loaded = False
    rig.group._tip_type = 201
    calls = []
    ejected_at = {}
    for driver in rig.drivers:
        def query(d=driver):
            calls.append(d.channel)
            loaded = len(calls) > 4 and len(rig.native.moves) > ejected_at.get(d.channel, -1)
            return {"ok": True, "semantic_ok": True, "tip_loaded": loaded,
                    "source_return_completed": True, "source_return": int(loaded),
                    "source_tip_loaded": loaded, "hardware_truth_level": "hardware_query"}
        def eject(d=driver, **kwargs):
            ejected_at[d.channel] = len(rig.native.moves)
            return d.issue("eject", **kwargs)
        monkeypatch.setattr(driver, "query_tip_status", query)
        monkeypatch.setattr(driver, "pipette_eject_tip", eject, raising=False)
        monkeypatch.setattr(driver, "aspirate", lambda volume, d=driver, **kw:
            d.issue("aspirate", volume=volume, **kw), raising=False)
        monkeypatch.setattr(driver, "dispense", lambda volume, d=driver, **kw:
            d.issue("dispense_liquid", volume=volume, **kw), raising=False)
    plan = compile_finite_plate_operation("source_fluid_offset", source_leaf_available=True,
        plate=plate, speed=300, transfer_fluid=transfer_fluid, skip_steps=12)
    try:
        result = p._wp8_execute_nested_plan(plan=plan, command_id="offset-parent",
            owner_identity={"source_identity": f"offset-parent:scan:{plate}"})
    except Exception as exc:
        evidence = getattr(exc, "evidence", {})
        body = (evidence.get("failure_evidence") or [{}])[0].get("result") or {}
        pytest.fail(str({"tip_type": rig.group._tip_type, "queries": len(calls), "ejected_at": ejected_at,
            "loads": [(s.get("result") or {}).get("source_return") for s in body.get("steps", []) if s["operation"] == "loadTips"],
            "load_failures": [(s.get("result") or {}).get("error") for s in body.get("steps", []) if s["operation"] == "loadTips"],
            "ops": [s["operation"] for s in body.get("steps", [])],
            "failures": [(step["operation"], step.get("error"),
                str(step.get("evidence"))[:500]) for step in body.get("steps", [])
                         if step.get("error")]}))
    assert result["ok"], result
    body = result["source_children"][0]["result"]
    assert [s["well"] for s in body["samples"]] == ["A1", "B1"]
    assert [s["operation"] for s in body["steps"]].count("loadTips") == (3 if transfer_fluid else 2)
    assert [s["operation"] for s in body["steps"]].count("ejectAllTips") == (3 if transfer_fluid else 2)
    assert body["source_return"] == 88000
    assert len(receipts.read(limit=100)) > 4
    keys = [row[0] for row in receipts.connection.execute(
        "SELECT command_id FROM pipette_operations")]
    assert len(keys) == len(set(keys))
    assert rig.store.connection.execute("SELECT count(*) FROM operator_commands WHERE command_id=?",
        ("offset-parent",)).fetchone()[0] == 1
