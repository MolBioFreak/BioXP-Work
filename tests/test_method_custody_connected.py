"""Explicit custody actions through actual finite worker and source publications."""
import threading
import pytest
from tests.test_deck_scoped_authority import retained_rig
from tests.test_cover_carry_release_connected import connected
from tests.test_native_completion_handoff import handoff


@pytest.fixture(autouse=True)
def empty_inspection_predecessor(retained_rig):
    # This test deliberately reuses the empty-inventory inspection fixture.
    # Reset only the scratch copy via its actual semantic publisher; retained
    # source captures and historical records are never edited.
    provider, _, _, _, store, _ = retained_rig
    store.publish_deck_owner_state(source_operation="updatePlateLocation",
        source_command_id="method-fixture-empty-inventory", updates={"movable_plate_locations": {}},
        **provider.deck_owner_authority_stamps())


@pytest.mark.parametrize("failure", [False, True])
def test_explicit_catch_release_native_custody(handoff, monkeypatch, failure):
    from bioxp import api
    from bioxp.services.protocol_service import bind_protocol_dispatcher, create_protocol_job, ProtocolOperatorBundleStore
    rig = handoff
    store = rig.store
    original = rig.native.motor_oem_move_absolute
    def move(board, target, **kw):
        if failure and len(rig.admitted) == 2 and store.deck_semantic_state()["plate_on_gantry"] == 4:
            raise RuntimeError("method_release_physical_leaf_failure")
        return original(board, target, **kw)
    monkeypatch.setattr(rig.native, "motor_oem_move_absolute", move)
    actions = [{"action_id": "catch", "kind": "plate_catch", "params": {"plate": 4},
                "metadata": {"step_id": "custody", "child_index": 0}},
               {"action_id": "release", "kind": "plate_release", "params": {"destination": 18},
                "metadata": {"step_id": "custody", "child_index": 1}}]
    payload = {"idempotency_key": "explicit-custody", "live_execution": {"live_execution_ack": True},
               "document": {"protocol_id": "custody", "stages": [{"stage_id": "s", "actions": actions}]}}
    handlers = {"plate_catch": api._protocol_live_custody_handler, "plate_release": api._protocol_live_custody_handler}
    artifacts = ProtocolOperatorBundleStore(rig.root / "method-artifacts")
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k: (handlers, {}, {}), artifact_store=artifacts)
    done = threading.Event()
    finish = store.finish_workflow
    def finished(*args, **kwargs):
        result = finish(*args, **kwargs)
        done.set()
        return result
    monkeypatch.setattr(store, "finish_workflow", finished)
    job = create_protocol_job(payload, dry_run=False, command_store=store, store=artifacts,
        handlers=handlers, ownership_generation=rig.provider.deck_owner_authority_stamps()["ownership_generation"], board_epochs={})
    dispatch_errors = []
    def dispatch(claimed):
        try:
            return rig.plane._dispatch_one(claimed)
        except Exception as exc:
            import traceback
            dispatch_errors.append(traceback.format_exc())
            raise
    store.start(dispatch)
    assert done.wait(20), store.get_workflow(job["job_id"])
    result = store.get_workflow(job["job_id"])
    rows = result["execution"]["runtime_state"]["action_results"]
    import json
    (rig.root / "method-result.json").write_text(json.dumps(result, indent=2))
    assert len(rows) == 2, str(rig.root / "method-result.json")
    assert rows[0]["ok"] is True and rows[0]["metadata"] == actions[0]["metadata"]
    assert [r[0] for r in rig.admitted] == ["catch_plate", "release_plate"]
    for row, (_, _, cid) in zip(rows, rig.admitted):
        durable = store.wp8_operation_evidence(cid)["children"]
        assert [(x["child_order"], x["operation"], x["status"]) for x in row["child_outcomes"]] == [
            (x["child_order"], x["operation"], x["terminal_state"]) for x in durable]
    semantic = store.deck_semantic_state()
    if failure:
        assert rows[1]["ok"] is False and semantic["plate_on_gantry"] == 4
        assert result["command"]["status"] == "ambiguous"
        child = store.wp8_operation_evidence(rig.admitted[1][2])
        assert "method_release_physical_leaf_failure" in repr(child)
    else:
        assert result["command"]["status"] == "completed", result
        assert semantic["plate_on_gantry"] is None
        assert semantic["movable_plate_locations"]["OUTPUT_COVER"] == "LOC_OC_COVER_STORAGE"


@pytest.mark.parametrize("kind,params,leaf_operation", [
    ("plate_press", {"plate": 0, "run_in_parallel": False}, "moveZPress"),
    ("cut_seal", {"count": 1, "cut_z_offset_steps": 0}, "getG"),
])
def test_explicit_press_cut_real_finite_owner(handoff, monkeypatch, kind, params, leaf_operation):
    from bioxp import api
    from bioxp.services.protocol_service import bind_protocol_dispatcher, create_protocol_job, ProtocolOperatorBundleStore
    rig, done = handoff, threading.Event()
    store = rig.store
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", rig.provider)
    # Authored simulated predecessor: door already open avoids adding a
    # composed door/Park test to this cutting qualification.
    rig.provider.wp8_update_thermal_door_open("updateThermalDoorOpen", {"value": True},
        command_id="method-cut-predecessor", child_order=0, plan_digest="fixture")
    rig.native.positions[6, 0] = 1
    # This simulator commits target positions synchronously at the physical
    # move leaf; expose its stopped observation, not a planner success stub.
    rig.native.motor_wait_stopped = lambda *a, **k: {"ok": True, "stopped": True}
    profile = rig.native._motion_oem_axis_profile
    rig.native._motion_oem_axis_profile = lambda axis, startup=False: profile(axis, startup=startup)
    rig.provider.primitives._z_profile_overrides = {}
    rig.provider.primitives._x_profile_overrides = {}
    from bioxp.usb_driver import BioXpTester
    rig.native._tmcl_success = BioXpTester._tmcl_success
    rig.native.oem_current_board_lifecycle_generation = lambda: 3
    rig.native.motor_oem_require_no_motion_profile = BioXpTester.motor_oem_require_no_motion_profile.__get__(rig.native)
    for axis in ("x", "z"):
        preset = rig.native._motion_oem_axis_profile(axis)
        for param, value in ((4, preset["speed"]), (5, preset["acc"]), (6, preset["run_current"]), (205, preset["stall_guard"]), (12, 1)):
            rig.native.parameters[preset["board"], preset["motor"], param] = value
    artifacts = ProtocolOperatorBundleStore(rig.root / "method-artifacts")
    handlers = {kind: api._protocol_live_custody_handler}
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k: (handlers, {}, {}), artifact_store=artifacts)
    finish = store.finish_workflow
    def finished(*a, **k):
        result = finish(*a, **k)
        done.set()
        return result
    monkeypatch.setattr(store, "finish_workflow", finished)
    payload = {"idempotency_key": "explicit-" + kind, "live_execution": {"live_execution_ack": True},
        "document": {"protocol_id": kind, "stages": [{"stage_id": "s", "actions": [
            {"action_id": kind, "kind": kind, "params": params}]}]}}
    job = create_protocol_job(payload, dry_run=False, command_store=store, store=artifacts,
        handlers=handlers, ownership_generation=rig.provider.deck_owner_authority_stamps()["ownership_generation"], board_epochs={})
    dispatch_errors = []
    def dispatch(claimed):
        try:
            return rig.plane._dispatch_one(claimed)
        except Exception as exc:
            import traceback
            dispatch_errors.append(traceback.format_exc())
            raise
    store.start(dispatch)
    assert done.wait(20), store.get_workflow(job["job_id"])
    result = store.get_workflow(job["job_id"])
    rows = result["execution"]["runtime_state"]["action_results"]
    import json, os
    if os.environ.get("CAVRO_EVIDENCE_ROOT"):
        from pathlib import Path
        Path(os.environ["CAVRO_EVIDENCE_ROOT"], "native-cut-result.json").write_text(json.dumps(result, indent=2))
    assert not dispatch_errors, dispatch_errors
    assert result["command"]["status"] == "completed", [(r.get("command", {}).get("terminal_evidence"), r.get("message"), r.get("child_outcomes")) for r in rows]
    outcomes = rows[0]["child_outcomes"]
    assert any(c["operation"] == leaf_operation for c in outcomes)
    command = rows[0]["command"]
    durable = store.wp8_operation_evidence(command["command_id"])["children"]
    assert len(outcomes) == len(durable)
    assert command["terminal_evidence"]["response"]["child_outcomes"] == outcomes
    assert '"omitted"' not in json.dumps(outcomes)
    if kind == "cut_seal":
        assert len(outcomes) > 10
        acquired = json.loads(durable[0]["terminal_evidence_json"])["result"]["lock_token"]
        assert durable[0]["operation"] == "LockGripperOperation"
        gripper_tasks = [task for task in rig.provider._wp8_tasks.values() if task["kind"] == "gripper_home_and_unlock"]
        assert gripper_tasks
        for task in gripper_tasks:
            task["thread"].join(timeout=5)
            assert task["state"] == "completed", task["error"]
            assert task["result"]["released"]["released_lock_token"] == acquired
        assert rig.provider._wp8_gripper_lock_owner is None
