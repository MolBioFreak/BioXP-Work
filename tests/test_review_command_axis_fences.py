"""Shared/legacy plane action footprints; not a test of live STOP delivery."""
import threading

import pytest

from bioxp.operator_command_plane import AXIS_BY_ACTION, OperatorCommandStore, _active_board_epochs


Y_ACTIONS = ("oem.y.manual_panel_home", "oem.y.move_steps", "oem.y.move_absolute")


@pytest.mark.parametrize("action", Y_ACTIONS)
def test_normal_y_actions_have_y_footprint(action):
    assert AXIS_BY_ACTION.get(action) == "y"
    assert OperatorCommandStore._axes_for_action(action) == {"y"}


@pytest.mark.parametrize("action", (*Y_ACTIONS, "oem.z.scriptmove_to"))
@pytest.mark.parametrize("axis", ("x", "y", "z"))
def test_each_relevant_axis_fence_blocks_shared_dispatch(action, axis):
    # No database or controller is needed for the real fence predicate.
    store = object.__new__(OperatorCommandStore)
    store._priority_fence = threading.Event()
    store._axis_priority_fences = {name: threading.Event() for name in ("x", "y", "z")}
    store._axis_priority_fences[axis].set()
    expected = {"x", "y", "z"} if action == "oem.z.scriptmove_to" else {"y"}
    assert store.action_fenced(action) is (axis in expected)
    store._axis_priority_fences[axis].clear()
    assert store.action_fenced(action) is False
    store._priority_fence.set()
    assert store.action_fenced(action) is True


@pytest.mark.parametrize("action", Y_ACTIONS)
def test_y_board_authority_is_board4(action):
    state = {"serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 20, "active_board_epoch": 20},
        "board4_authority": {"active_board_epoch": 10}}}
    assert _active_board_epochs(state, action) == {"4": 10}


def test_scriptmove_fences_xyz_and_reads_both_board_epochs():
    assert OperatorCommandStore._axes_for_action("oem.z.scriptmove_to") == {"x", "y", "z"}
    state = {"serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 20},
        "board4_authority": {"active_board_epoch": 10}}}
    assert _active_board_epochs(state, "oem.z.scriptmove_to") == {"4": 10, "5": 20}


@pytest.mark.parametrize("action,inputs,axis", [
    ("oem.y.move_steps", {"steps": 10}, "y"),
    ("oem.z.scriptmove_to", {"location_id": "LOC_OC"}, "x"),
    ("oem.z.scriptmove_to", {"location_id": "LOC_OC"}, "y"),
])
def test_method_control_checks_each_composite_or_y_epoch(tmp_path, action, inputs, axis):
    from fastapi import HTTPException
    from bioxp.oem_runtime_store import OEMRuntimeStore
    runtime = OEMRuntimeStore(tmp_path)
    store = OperatorCommandStore(tmp_path)
    state = {"ownership_generation": 7, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 20},
        "board4_authority": {"active_board_epoch": 10}}}
    try:
        method = store.admit_method({"name": "offline-epoch-method", "idempotency_key": "method-review-key",
            "expected_ownership_generation": 7,
            "steps": [{"action_id": action, "inputs": inputs}]}, state=state)
        recovery = store.recovery()
        with store._transaction() as conn:
            conn.execute(f"UPDATE operator_plane_safety SET {axis}_epoch={axis}_epoch+1 WHERE singleton=1")
        request = {"idempotency_key": "pause-review-key", "expected_version": method["version"],
            "expected_recovery_epoch": recovery["recovery_epoch"],
            "expected_global_safety_epoch": recovery["global_safety_epoch"],
            "expected_axis_safety_epoch": 0}
        with pytest.raises(HTTPException) as exc:
            store.method_mutation(method["method_id"], "pause", request)
        assert exc.value.detail["error"] == "method_safety_epoch_conflict"
        result = store.method_mutation(method["method_id"], "pause", {**request, "expected_axis_safety_epoch": 1})
        assert result["status"] == "pause_requested"
    finally:
        store.connection.close()
        runtime.close()
