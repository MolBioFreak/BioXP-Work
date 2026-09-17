"""Cleanup source predicates; physical leaves doubled, no hardware routes."""
from types import SimpleNamespace
from contextlib import nullcontext
from tests.test_protocol_oem_provider_bindings import rig
import pytest
from bioxp.oem_deck_movement import compile_finite_plate_operation, execute_finite_plate_operation
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider


def plan(**inputs):
    return compile_finite_plate_operation("cleanup", source_leaf_available=True,
        **{"cover_locations": {4: 18, 5: 20}, **inputs})


def run_cleanup(*, cached, queried, door=True, covers=None, fail=None):
    provider = object.__new__(Provider)
    trace = []
    # Exercise actual provider query validation/result, with transport/audit leaf doubled.
    provider._wp8_run_pipette = lambda **kwargs: {"channels": [
        {"tip_loaded": queried if index == 0 else False} for index in range(4)]}
    def invoke(child):
        name = child["operation"]
        trace.append(name)
        if name == fail:
            raise RuntimeError("physical_leaf_failed")
        if name == "queryTipStatus":
            return provider.wp8_query_tip_status(name, {}, command_id="cleanup",
                child_order=child["order"], plan_digest="test")
        return {"ok": True, "door_ok": door}
    result = execute_finite_plate_operation(plan(tip_exists=cached,
        **({"cover_locations": covers} if covers is not None else {})), invoke)
    return trace, result


@pytest.mark.parametrize("cached,queried", [(False, True), (True, False), (False, False), (True, True)])
def test_cleanup_uses_query_not_cached_tiploaded_and_always_clears(cached, queried):
    trace, result = run_cleanup(cached=cached, queried=queried)
    assert ("ejectAllTipsCleanup" in trace) is queried
    assert trace.count("clearTipLoaded") == 1
    assert trace.index("clearTipLoaded") < trace.index("sendGripperHome")
    assert result["residual_state"]["tip_loaded"] is False
    if queried:
        assert trace[:10] == ["waitStop", "checkDoorStatus", "queryTipStatus",
            "scriptmoveToWaste", "updateLocation", "ejectAllTipsCleanup",
            "moveZ80000", "moveX79000", "clearTipLoaded", "sendGripperHome"]


@pytest.mark.parametrize("cached", [False, True])
def test_closed_door_skips_query_and_every_cleanup_effect(cached):
    trace, result = run_cleanup(cached=cached, queried=True, door=False)
    assert trace == ["waitStop", "checkDoorStatus"]
    assert result["residual_state"] == {}


@pytest.mark.parametrize("covers", [{}, {4: 18}, {5: 20}, {4: None, 5: 20},
    {4: -1, 5: 20}, {4: True, 5: 20}, {4: "18", 5: 20}])
def test_unknown_plate_custody_never_compiles_default_motion(covers):
    with pytest.raises((ValueError, RuntimeError), match="source_authority_missing:cleanup_plate_location"):
        plan(cover_locations=covers)


def test_carried_cover_skips_catch_preserves_custody_transitions():
    trace, result = run_cleanup(cached=False, queried=False, covers={4: 29, 5: 3})
    assert trace.count("catchPlate") == 1
    assert trace.count("releasePlate") == 2
    assert result["residual_state"]["plate_locations"] == {"4": 18, "5": 20}
    children = plan(cover_locations={4: 29, 5: 3})["children"]
    assert [c["state_mutation"] for c in children if c["operation"] == "updatePlateLocation"] == [
        {"plate": 4, "location": 18}, {"plate": 5, "location": 29}, {"plate": 5, "location": 20}]


def test_query_failure_does_not_clear_or_move():
    with pytest.raises(RuntimeError, match="wp8_tip_status_invalid"):
        run_cleanup(cached=False, queried=None)


def test_ejection_failure_propagates_without_running_remainder():
    with pytest.raises(RuntimeError, match="physical_leaf_failed"):
        run_cleanup(cached=False, queried=True, fail="ejectAllTipsCleanup")


@pytest.mark.parametrize("door_ok", [False, True])
def test_cleanup_prefix_canonical_parent_child_store_and_reopen(rig, tmp_path, door_ok, monkeypatch):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.oem_deck_movement import make_wp8_operation_executor
    p, _, trace, _, _, machine = rig
    initial = OEMRuntimeStore(tmp_path); initial.close()
    store = OperatorCommandStore(tmp_path)
    stamps = p.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    store.bind_workflow_dispatcher(lambda command: None)
    p._deck_semantic_state_publisher = store.publish_deck_owner_state
    store.publish_deck_owner_state(source_operation="pipette_owner", source_command_id="observed",
        updates={"tip_loaded": True, "tip_dirty": False, "tip_location": -1}, **stamps)
    # Only physical leaves doubled. Actual door/query/state/gripper wrappers run.
    p.primitives.deck_io_query_type = lambda kind: {"value": (0 if door_ok else 1) if kind == 0 else 0}
    from tests.protocol_v1_integration_fixture import NativePhysicalRecorder
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    from bioxp import oem_machine_bundle
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot',
        oem_machine_bundle.load_oem_machine_snapshot(
            snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json',
            operator_label_serial=206, require_operator_label=True))
    native = NativePhysicalRecorder(monkeypatch)
    native.positions = {(4, 2): 0}
    exchange = native.exchange
    def wire(board, command, typ, motor, value, **kwargs):
        if (board, command, typ, motor, value) == (4, 4, 0, 2, 10000):
            from tests.test_motor_receive_identity import receive
            native.trace.append((board, command, typ, motor, value))
            native.positions[board, motor] = value
            receive(native.tester, board=board, motor=motor)
            return {'status': 100, 'value': value}
        return exchange(board, command, typ, motor, value, **kwargs)
    native.tester._send_motor = wire
    native.tester.send_tmcl_retry = wire
    monkeypatch.setattr(native.tester, '_motion_oem_gripper_version', lambda: 1)
    p.primitives.tester.motor_oem_home_axis = native.tester.motor_oem_home_axis
    p._wp8_calibration = lambda: (1, {})  # explicit fixture gripper version
    p.primitives.pipette_audit_runner = lambda name, call, **kw: call(SimpleNamespace(
        query_tip_status_all=lambda: {"channels": [{"tip_loaded": False} for _ in range(4)]}))
    execute = make_wp8_operation_executor(provider_getter=lambda: p, command_store=store)
    admission = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1}, "board4_authority": {"active_board_epoch": 1}}}
    children = []
    # Existing waste_sequence provider must reuse the exact cleanup prefix,
    # without demanding unrelated cover custody or rebuilding cached predicates.
    def canonical_prefix(*, plan, command_id, owner_identity):
        with store.workflow_context("parent", source_occurrence_id="cleanup-prefix"):
            child = store.admit_internal_wp8_operation("cleanup", inputs={}, state=admission,
                idempotency_key="cleanup-prefix", prepared_plan=plan)
        cid = child["command_id"]; children.append(cid)
        claimed = store.claim_next(); assert claimed["command_id"] == cid
        result = execute(command_id=cid, plan=plan)
        store.finish(cid, status="completed", payload={"response": result}, claimed=claimed)
        return result
    p._wp8_execute_nested_plan = canonical_prefix
    # Plan construction should need no pre-query machine snapshot at all.
    p.wp8_operation_machine_state = lambda *a: pytest.fail("unrelated machine/custody read")
    try:
        store.admit_workflow(command_id="parent", idempotency_key="parent", plan_fingerprint="fixture",
            requested_inputs={"bundle": {"execution": {"runtime_state": {}}}}, ownership_generation=1,
            resources=("axis:x", "axis:y", "axis:z", "gripper"), board_epochs={})
        assert store.claim_next()["command_id"] == "parent"
        # Explicit test source-body settlement. NOT API event-lifetime qualification.
        p._wp8_stop_event.clear(); p._wp8_stop_event.set()
        result = p.wp8_cleanup_waste_prelude("cleanupWastePrelude", {}, command_id="parent", owner_identity={})
        assert result["ok"] is True
        assert p._wp8_stop_event.is_set() is False
        assert store.deck_semantic_state()["tip_loaded"] is (not door_ok)
        evidence = store.wp8_operation_evidence(children[0])
        query = next(row for row in evidence["children"] if row["operation"] == "queryTipStatus")
        assert query["terminal_state"] == "completed"
        import json
        assert bool(json.loads(query["terminal_evidence_json"])["result"].get("source_branch_skipped")) is (not door_ok)
        home = next(row for row in evidence['children'] if row['operation'] == 'sendGripperHome')
        result = json.loads(home['terminal_evidence_json'])['result']
        if door_ok:
            assert result['source_call_completed'] is True
            assert result['primitive_result']['home']['source_return_code'] == 0
            assert native.trace[0] == (4, 5, 6, 2, 31)
            assert native.trace[-1] == (4, 5, 6, 2, 10)
        else:
            assert result['source_branch_skipped'] is True
            assert native.trace == []
        assert store.finish_workflow("parent", status="completed", payload={}, lifecycle_settled=True)["command"]["status"] == "completed"
    finally:
        store.connection.close()
    reopened = OperatorCommandStore(tmp_path)
    try:
        assert reopened.deck_semantic_state()["tip_loaded"] is (not door_ok)
        assert reopened.get_command("parent")["status"] == "completed"
        assert reopened.get_command(children[0])["status"] == "completed"
    finally:
        reopened.connection.close()
