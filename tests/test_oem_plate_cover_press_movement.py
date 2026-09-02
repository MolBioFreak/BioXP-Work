from __future__ import annotations

from contextlib import nullcontext
import json
import sqlite3
import threading
from types import SimpleNamespace

import pytest

from bioxp import oem_deck_movement
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandPlane, OperatorCommandStore


def ops(plan):
    return [row["operation"] for row in plan["children"]]


def test_wp8_allowlist_has_no_retired_parallel_authority_aliases() -> None:
    assert {
        "doorOpenClose", "storeOutputCover4To18", "storeReagentCover5To20",
    }.isdisjoint(oem_deck_movement.WP8_COMPILED_CHILD_OPERATIONS)


def test_oem_mp_translation_matches_classglobals_duplicate_location_rules() -> None:
    assert oem_deck_movement.translate_oem_plate_move("PL_OUTPUT", "LOC_TC", "PRESS") == {
        "plate": 1, "destination": 23, "press_plate": True,
    }
    assert oem_deck_movement.translate_oem_plate_move("CV_BIOSECURITY", "LOC_TC", None) == {
        "plate": 3, "destination": 5, "press_plate": False,
    }
    assert oem_deck_movement.translate_oem_plate_move("CV_OUTPUT", "LOC_OC", None)["destination"] == 17
    assert oem_deck_movement.translate_oem_plate_move("PL_OUTPUT", "LOC_OC", None)["destination"] == 21
    assert oem_deck_movement.translate_oem_plate_move("PL_OUTPUT", "LOC_OC", "press")["press_plate"] is False


def test_every_compiled_wp8_target_is_bounded_by_position_table_rows() -> None:
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    from bioxp.oem_compat.position_table import PositionTable

    plans = [
        oem_deck_movement.compile_finite_plate_operation(
            "move_plate", source_leaf_available=True, plate=1, destination=24,
            press_plate=True, thermal_door_open=False,
        ),
        oem_deck_movement.compile_finite_plate_operation(
            "catch_plate", source_leaf_available=True, plate=3, plate_location=5,
            thermal_door_open=True, gripper_version=1,
        ),
        oem_deck_movement.compile_finite_plate_operation(
            "release_plate", source_leaf_available=True, destination=25,
            plate_on_gantry=1, output_plate_location=29, current_tray=2,
            press_plate=True, gripper_version=1,
        ),
        oem_deck_movement.compile_finite_plate_operation(
            "press_plates", source_leaf_available=True, plates=[2, 0, 1],
            plate_locations={1: 21},
        ),
        oem_deck_movement.compile_finite_plate_operation(
            "cleanup", source_leaf_available=True, tip_exists=True,
            cover_locations={4: 17, 5: 19},
        ),
        oem_deck_movement.compile_finite_plate_operation(
            "park_gantry", source_leaf_available=True,
        ),
        {"children": [{"operation": "scriptmoveTo", "arguments": {"destination": 2}}]},
    ]
    targets = frozenset().union(
        *(oem_deck_movement.compiled_wp8_machine_targets(plan) for plan in plans)
    )
    table = PositionTable.from_rows([
        {"location_id": LOCATION_ID_TO_NAME[target], "x": target, "y": target,
         "zLow": 60000, "zDelta": 10000}
        for target in sorted(targets)
    ])
    assert oem_deck_movement.serial206_position_table_ordinals(table) == targets
    for plan in plans:
        oem_deck_movement.validate_compiled_wp8_machine_targets(plan, table)
        for target in oem_deck_movement.compiled_wp8_machine_targets(plan):
            rows_without_target = {
                str(index): row for index, row in enumerate(table.rows())
                if row["location_id"] != LOCATION_ID_TO_NAME[target]
            }
            with pytest.raises(
                RuntimeError,
                match=rf"machine_target_absent_from_serial206_position_table:{target}",
            ):
                oem_deck_movement.validate_compiled_wp8_machine_targets(
                    plan, rows_without_target,
                )


def test_plate_move_composite_keeps_catch_release_atomic_source_order() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "move_plate", source_leaf_available=True, plate=1, destination=24, press_plate=True,
        thermal_door_open=False,
    )
    assert ops(plan) == ["catchPlate", "releasePlate"]
    assert plan["children"][0]["arguments"] == {"plate": 1, "run_in_parallel": True}
    assert plan["children"][1]["arguments"] == {
        "destination": 24, "press_plate": True, "run_in_parallel": True,
    }
    assert all(child["ignored_return"] is True for child in plan["children"])
    with pytest.raises(RuntimeError, match="thermal_door_must_be_open"):
        oem_deck_movement.compile_finite_plate_operation(
            "move_plate", source_leaf_available=True,
            plate=0, destination=25, press_plate=False, thermal_door_open=False,
        )
    with pytest.raises(RuntimeError, match="thermal_door_must_be_closed"):
        oem_deck_movement.compile_finite_plate_operation(
            "move_plate", source_leaf_available=True,
            plate=1, destination=18, press_plate=False, thermal_door_open=True,
        )


def test_catch_plate_compiles_literal_order_remap_and_residual_policies() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "catch_plate", source_leaf_available=True, plate=1, plate_location=1,
        thermal_door_open=False, gripper_version=0, run_in_parallel=True,
        output_plate_location=1,
    )
    assert plan["resolved_location"] == 21
    assert ops(plan) == [
        "doorOpen", "scriptmoveTo", "updateLocation", "LockGripperOperation",
        "setGripperCurrent", "setGripperVMax", "StopCloseGripper", "OpenGripperWide",
        "led2On", "Sleep", "SnapshotImage", "Sleep", "led2Off", "moveZ", "Sleep",
        "CloseGripper", "LoadGantry", "moveZPseudoHome", "updatePlateLocation",
        "ReleaseLockGripperOperation",
    ]
    assert plan["children"][0]["ignored_return"] is True
    assert plan["children"][2]["state_mutation"] == {"current_location": 21, "current_well": 0}
    assert plan["children"][-2]["state_mutation"] == {"plate": 1, "location": 29}
    assert plan["exception_policy"] == "log_and_suppress"
    assert plan["finally_children"] == ["ReleaseLockGripperOperation"]


def test_catch_pool_and_biosecurity_literal_special_children() -> None:
    pool = oem_deck_movement.compile_finite_plate_operation(
        "catch_plate", source_leaf_available=True, plate=0, plate_location=0,
        thermal_door_open=True, gripper_version=1,
    )
    assert pool["resolved_location"] == 25
    assert "moveGClosedPlus3000" in ops(pool)
    bio = oem_deck_movement.compile_finite_plate_operation(
        "catch_plate", source_leaf_available=True, plate=3, plate_location=5,
        thermal_door_open=True, gripper_version=1,
    )
    sequence = ops(bio)
    start = sequence.index("moveStepsZMinus6000")
    assert sequence[start:start + 3] == ["moveStepsZMinus6000", "moveStepsYMinus800", "moveStepsYPlus1600"]


def test_release_exact_order_offset_press_and_no_cleanup() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "release_plate", source_leaf_available=True, destination=25,
        plate_on_gantry=1, output_plate_location=29, current_tray=2,
        press_plate=True, gripper_version=1, run_in_parallel=True,
    )
    assert plan["z_offset"] == -30236
    sequence = ops(plan)
    assert sequence[:8] == [
        "scriptmoveTo", "moveZLow", "Sleep", "LockGripperOperation",
        "setGripperCurrent", "OpenGripperWide", "Sleep", "setGripperVMax",
    ]
    assert sequence[8:13] == ["Sleep", "moveZPressApproach", "CloseGripper", "setZCurrent31", "moveZPress"]
    assert sequence[13] == "setZaxisCurrentmax100"
    assert sequence.index("LoadGantryNull") < sequence.index("sendZandGripperHome") < sequence.index("updatePlateLocation") < sequence.index("updateLocation")
    plate_update = next(row for row in plan["children"] if row["operation"] == "updatePlateLocation")
    assert plate_update["arguments"] == {"plate": 2, "location": 25}
    assert plate_update["state_mutation"] == {"plate": 2, "location": 25}
    assert plan["exception_policy"] == "log_and_suppress"
    assert plan["finally_children"] == []
    assert plan["residual_policy"] == "retain_completed_children_without_rollback"


def test_pressplates_internal_targets_array_order_and_background_home() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "press_plates", source_leaf_available=True, plates=[2, 99, 0, 1],
        plate_locations={0: 23, 1: 21, 2: 3}, run_in_parallel=True,
    )
    assert "pressPlate" not in ops(plan)
    script_moves = [row for row in plan["children"] if row["operation"] == "scriptmoveTo"]
    assert [row["arguments"]["destination"] for row in script_moves] == [27, 24, 21]
    assert all("plate" not in row["arguments"] for row in script_moves)
    assert sum(row["operation"] == "moveZPressApproach" for row in plan["children"]) == 3
    assert plan["children"][-1]["operation"] == "backgroundGripperHomeAndUnlock"
    assert plan["children"][-1]["awaited"] is False
    assert plan["parent_return_allows_background_pending"] is True


def test_send_z_gripper_home_parallel_and_sequential_are_literal() -> None:
    parallel = oem_deck_movement.compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True, run_in_parallel=True,
        gripper_position=4000, closed_position=3000,
    )
    assert ops(parallel) == ["getG", "startMoveZPseudoHome", "Sleep", "startGripperHomeAndUnlock", "waitMoveZOnly"]
    assert parallel["children"][2]["arguments"] == {"milliseconds": 1000}
    sequential = oem_deck_movement.compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True, run_in_parallel=False,
        gripper_position=0, closed_position=3000,
    )
    assert ops(sequential) == ["getG", "moveZPseudoHome", "sendGripperHome", "ReleaseLockGripperOperation"]


def test_door_profiles_returns_and_retry_semantics() -> None:
    noop = oem_deck_movement.compile_finite_plate_operation(
        "thermal_door", source_leaf_available=True, open=True, door_is_open=True,
        board_test_mode=False,
    )
    assert noop["children"] == []
    low = oem_deck_movement.compile_finite_plate_operation(
        "thermal_door", source_leaf_available=True, open=True, door_is_open=False,
        board_test_mode=False,
    )
    assert ops(low) == [
        "parkGantry", "setDoorStallThresholdPlus2", "setDoorMaxCurrent", "moveDoorOpen",
        "readDoorSensors", "HomeAxisD", "setDoorStallThreshold", "setDoorMaxCurrent",
        "moveDoorOpen", "readDoorSensors", "updateThermalDoorOpen",
    ]
    assert set(ops(low)) <= oem_deck_movement.WP8_COMPILED_CHILD_OPERATIONS
    for child in low["children"][5:10]:
        assert child["source_condition"] == {
            "child_order": 4, "result_field": "door_open", "equals": False,
        }
    calls = []
    def already_open(child):
        calls.append(child["operation"])
        if child["operation"] == "readDoorSensors":
            return {"ok": True, "door_open": True}
        return {"ok": True}
    result = oem_deck_movement.execute_finite_plate_operation(low, already_open)
    assert result["ok"] is True
    assert calls == [
        "parkGantry", "setDoorStallThresholdPlus2", "setDoorMaxCurrent",
        "moveDoorOpen", "readDoorSensors", "updateThermalDoorOpen",
    ]
    assert low["normal_state_update_unconditional"] is True
    board = oem_deck_movement.compile_finite_plate_operation(
        "thermal_door", source_leaf_available=True, open=False, door_is_open=True,
        board_test_mode=True,
    )
    assert board["success_return"] is False
    assert board["failure_policy"] == "throw"
    def closed_sensor_failure(child):
        if child["operation"] == "readDoorSensors":
            return {"ok": True, "door_closed": False}
        return {"ok": True}
    with pytest.raises(RuntimeError, match="thermal_door_close_failed"):
        oem_deck_movement.execute_finite_plate_operation(board, closed_sensor_failure)


def test_cleanup_keeps_distinct_waste_and_cover_storage_order() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "cleanup", source_leaf_available=True, tip_exists=True, door_status_ok=True,
        current_location=1, cover_locations={4: 17, 5: 19},
    )
    sequence = ops(plan)
    assert set(sequence) <= oem_deck_movement.WP8_COMPILED_CHILD_OPERATIONS
    assert sequence[:8] == ["waitStop", "checkDoorStatus", "queryTipStatus", "scriptmoveToWaste", "updateLocation", "ejectAllTipsCleanup", "moveZ80000", "moveX79000"]
    assert "storeOutputCover4To18" not in sequence
    assert "storeReagentCover5To20" not in sequence
    cover_start = sequence.index("sendGripperHome") + 1
    assert sequence[cover_start:] == [
        "doorOpen", "catchPlate", "updatePlateLocation", "releasePlate", "updatePlateLocation",
        "doorOpen", "catchPlate", "updatePlateLocation", "releasePlate", "updatePlateLocation",
        "doorOpen", "parkGantry",
    ]
    door_children = [row for row in plan["children"] if row["operation"] == "doorOpen"]
    assert [row["arguments"]["open"] for row in door_children] == [False, False, True]
    assert "destination != 20 || destination != 18" in plan["source_hazards"]
    calls = []
    def blocked(child):
        calls.append(child["operation"])
        if child["operation"] == "checkDoorStatus":
            return {"ok": True, "door_ok": False}
        return {"ok": True}
    result = oem_deck_movement.execute_finite_plate_operation(plan, blocked)
    assert result["ok"] is True
    assert calls == ["waitStop", "checkDoorStatus"]


def test_existing_park_and_distinct_waste_plan_are_reused() -> None:
    park = oem_deck_movement.compile_finite_plate_operation("park_gantry", source_leaf_available=True)
    assert park["provider_method"] == "parkGantry"
    assert park["state_update"] == "source_ordered_partial_residuals"
    waste = oem_deck_movement.compile_finite_plate_operation("waste_sequence", source_leaf_available=True)
    assert waste["provider_method"] == "cleanupWastePrelude"


def test_executor_suppresses_catch_failure_runs_finally_and_retains_completed() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "catch_plate", source_leaf_available=True, plate=1, plate_location=1,
        thermal_door_open=False, gripper_version=1, run_in_parallel=True,
    )
    calls = []

    def invoke(child):
        calls.append(child["operation"])
        if child["operation"] == "moveZ":
            raise RuntimeError("boom")
        return {"ok": True}

    result = oem_deck_movement.execute_finite_plate_operation(plan, invoke)
    assert result["exception_suppressed"] is True
    assert result["failed_child"] == "moveZ"
    assert calls[-1] == "ReleaseLockGripperOperation"
    assert result["residual_state"]["current_location"] == 21
    assert "plate_locations" not in result["residual_state"]


def test_absent_controller_leaf_fails_closed_explicitly() -> None:
    with pytest.raises(RuntimeError, match="source_authority_missing:press_plate"):
        oem_deck_movement.compile_finite_plate_operation("press_plate", source_leaf_available=False)


class _ProductionWp8Fake:
    def __init__(self, fail_at=None, false_at=None, background_receipts=True):
        self.calls = []
        self.fail_at = fail_at
        self.false_at = false_at
        self.background_receipts = background_receipts

    def movement_lease(self):
        return nullcontext()

    def deck_owner_authority_stamps(self):
        return {"ownership_generation": 7, "board_epoch_4": 11, "board_epoch_5": 12}

    def execute_wp8_child(self, child, *, command_id, child_order, plan_digest):
        self.calls.append(child["operation"])
        if child["operation"] == self.fail_at:
            raise oem_deck_movement.DeckExecutionFailure(
                "injected provider failure", delivery_attempted=True,
            )
        if child["operation"] == self.false_at:
            return {"ok": False, "delivery_attempted": True, "provider_call": child["operation"]}
        if not child.get("awaited", True) and self.background_receipts:
            return {
                "ok": True,
                "provider_call": child["operation"],
                "background_task_id": f"task-{child['order']}",
                "background_task_state": "running_unawaited",
            }
        return {"ok": True, "provider_call": child["operation"]}


def _arm_wp8_test_command(store, operation: str, key: str) -> str:
    from bioxp.oem_deck_movement import OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS

    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None,
        "movable_plate_locations": dict(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS),
        "pseudo_z_home": 65000, "ownership_generation": 7,
        "board_epoch_4": 11, "board_epoch_5": 12,
        "latch_status": True, "machine_latch_closed": True,
        "latch_observation_id": f"latch-{key}",
        "source_operation": "test_source_snapshot",
        "source_command_id": f"source-{key}",
    })
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 12},
            "board4_authority": {"active_board_epoch": 11},
        },
    }
    admitted = store.admit_internal_wp8_operation(
        operation, inputs={}, state=state, idempotency_key=key,
    )
    claimed = store.claim_next()
    assert claimed is not None and claimed["command_id"] == admitted["command_id"]
    return str(admitted["command_id"])


def test_production_executor_calls_provider_and_terminalizes_every_child(tmp_path, monkeypatch) -> None:
    monkeypatch.delenv("BIOXP_OEM_RUNTIME_STATE_ROOT", raising=False)
    monkeypatch.delenv("BIOXP_OPERATOR_STATE_ROOT", raising=False)
    monkeypatch.delenv("BIOXP_RUNTIME_STATE_ROOT", raising=False)
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    monkeypatch.setattr(store, "assert_deck_execution_current", lambda *_args, **_kwargs: None)
    provider = _ProductionWp8Fake()
    plan = oem_deck_movement.compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True, run_in_parallel=False,
        gripper_position=0, closed_position=3000,
    )
    command_id = _arm_wp8_test_command(store, str(plan["operation"]), "wp8-production-success")
    result = oem_deck_movement.make_wp8_operation_executor(
        provider_getter=lambda: provider, command_store=store,
    )(command_id=command_id, plan=plan)
    evidence = store.wp8_operation_evidence(command_id)
    assert result["ok"] is True
    assert provider.calls == ops(plan)
    assert [row["terminal_state"] for row in evidence["children"]] == ["completed"] * len(plan["children"])
    assert all(json.loads(row["terminal_evidence_json"])["result"]["provider_call"] == row["operation"] for row in evidence["children"])
    store.connection.close()


def test_production_executor_retains_terminal_rows_and_residual_after_failure(tmp_path, monkeypatch) -> None:
    monkeypatch.delenv("BIOXP_OEM_RUNTIME_STATE_ROOT", raising=False)
    monkeypatch.delenv("BIOXP_OPERATOR_STATE_ROOT", raising=False)
    monkeypatch.delenv("BIOXP_RUNTIME_STATE_ROOT", raising=False)
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    monkeypatch.setattr(store, "assert_deck_execution_current", lambda *_args, **_kwargs: None)
    provider = _ProductionWp8Fake(fail_at="moveZ")
    plan = oem_deck_movement.compile_finite_plate_operation(
        "catch_plate", source_leaf_available=True, plate=1, plate_location=1,
        thermal_door_open=False, gripper_version=1, run_in_parallel=True,
    )
    command_id = _arm_wp8_test_command(store, str(plan["operation"]), "wp8-production-failure")
    result = oem_deck_movement.make_wp8_operation_executor(
        provider_getter=lambda: provider, command_store=store,
    )(command_id=command_id, plan=plan)
    evidence = store.wp8_operation_evidence(command_id)
    by_op = {row["operation"]: row for row in evidence["children"]}
    assert result["exception_suppressed"] is True
    assert by_op["updateLocation"]["terminal_state"] == "completed"
    assert by_op["moveZ"]["terminal_state"] == "ambiguous"
    assert by_op["updatePlateLocation"]["terminal_state"] == "planned"
    assert json.loads(evidence["state_transitions"][0]["transition_json"])["current_location"] == 21
    with pytest.raises(sqlite3.DatabaseError):
        store.connection.execute("DELETE FROM operator_plane_wp8_children WHERE command_id=?", (command_id,))
    store.connection.close()


def test_production_executor_does_not_complete_false_nonignored_child(tmp_path, monkeypatch) -> None:
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    monkeypatch.setattr(store, "assert_deck_execution_current", lambda *_args, **_kwargs: None)
    provider = _ProductionWp8Fake(false_at="moveZ")
    plan = oem_deck_movement.compile_finite_plate_operation(
        "catch_plate", source_leaf_available=True, plate=1, plate_location=1,
        thermal_door_open=False, gripper_version=1, run_in_parallel=True,
    )
    command_id = _arm_wp8_test_command(store, str(plan["operation"]), "wp8-production-false")
    result = oem_deck_movement.make_wp8_operation_executor(
        provider_getter=lambda: provider, command_store=store,
    )(command_id=command_id, plan=plan)
    evidence = store.wp8_operation_evidence(command_id)
    by_op = {row["operation"]: row for row in evidence["children"]}
    assert result["ok"] is False
    assert result["delivery_attempted"] is True
    assert by_op["moveZ"]["terminal_state"] == "ambiguous"
    assert by_op["updatePlateLocation"]["terminal_state"] == "planned"
    store.connection.close()


def test_production_executor_does_not_complete_false_ignored_transport_child(tmp_path, monkeypatch) -> None:
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    monkeypatch.setattr(store, "assert_deck_execution_current", lambda *_args, **_kwargs: None)
    provider = _ProductionWp8Fake(false_at="catchPlate")
    plan = oem_deck_movement.compile_finite_plate_operation(
        "move_plate", source_leaf_available=True,
        plate=1, destination=24, press_plate=True,
        thermal_door_open=True, run_in_parallel=True,
    )
    command_id = _arm_wp8_test_command(store, str(plan["operation"]), "wp8-production-false-ignored")
    with pytest.raises(oem_deck_movement.DeckExecutionFailure, match="wp8 child failed: catchPlate"):
        oem_deck_movement.make_wp8_operation_executor(
            provider_getter=lambda: provider, command_store=store,
        )(command_id=command_id, plan=plan)
    evidence = store.wp8_operation_evidence(command_id)
    by_op = {row["operation"]: row for row in evidence["children"]}
    assert by_op["catchPlate"]["terminal_state"] == "ambiguous"
    assert by_op["releasePlate"]["terminal_state"] == "planned"
    store.connection.close()


def test_production_executor_rejects_fabricated_unawaited_task_evidence(tmp_path, monkeypatch) -> None:
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    monkeypatch.setattr(store, "assert_deck_execution_current", lambda *_args, **_kwargs: None)
    provider = _ProductionWp8Fake(background_receipts=False)
    plan = oem_deck_movement.compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True, run_in_parallel=True,
        gripper_position=4000, closed_position=3000,
    )
    command_id = _arm_wp8_test_command(store, str(plan["operation"]), "wp8-background-missing")
    with pytest.raises(oem_deck_movement.DeckExecutionFailure, match="background task evidence missing"):
        oem_deck_movement.make_wp8_operation_executor(
            provider_getter=lambda: provider, command_store=store,
        )(command_id=command_id, plan=plan)
    evidence = store.wp8_operation_evidence(command_id)
    start = next(row for row in evidence["children"] if row["operation"] == "startMoveZPseudoHome")
    assert start["terminal_state"] == "failed"
    assert "background_task_id" not in json.loads(start["terminal_evidence_json"])["result"]
    store.connection.close()


def _bare_wp8_provider():
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    provider = object.__new__(Serial206OemInitializationProvider)
    provider._wp8_task_lock = threading.Lock()
    provider._wp8_tasks = {}
    provider._wp8_gripper_lock = threading.Lock()
    provider._wp8_gripper_lock_owner = None
    provider.deck_owner_authority_stamps = lambda: {
        "ownership_generation": 7, "board_epoch_4": 11, "board_epoch_5": 12,
    }
    return provider


def test_production_wp8_snapshot_carries_authoritative_position_table(monkeypatch) -> None:
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    provider = object.__new__(Serial206OemInitializationProvider)
    table_rows = {"23": {"location_id": "LOC_P_TC", "x": 1, "y": 2, "zLow": 3}}
    provider.mov_execution_machine_state = lambda: {
        "plate_locations": {0: 23}, "plate_on_gantry": None, "tip_loaded": False,
        "thermal_door_open": False, "gripper_version": 1, "board_test_mode": False,
        "position_table_by_location": table_rows,
    }
    provider.primitives = SimpleNamespace(
        tester=SimpleNamespace(motor_get_position=lambda *_args, **_kwargs: {"position": 4000})
    )
    monkeypatch.setattr(
        "bioxp.oem_initialization.build_machine_calibration_manifest",
        lambda **_kwargs: {
            "ok": True,
            "gripper": {"GripperClosePOS": {"value": 3000}},
            "thermal_door": {
                "TCDoorOpen": {"value": 1},
                "TCDoorStallGuardThreshold": {"value": 2},
                "TC_DOOR_MAX_CURRENT": {"value": 3},
            },
        },
    )

    snapshot = provider.wp8_operation_machine_state("park_gantry", {})

    assert snapshot["position_table_by_location"] is table_rows


def test_wait_move_z_binds_exact_command_plan_task_not_global_last_pointer() -> None:
    provider = _bare_wp8_provider()
    provider.wp8_move_z_pseudo_home = lambda operation, _arguments: {
        "ok": True, "operation": operation,
    }
    first = provider.wp8_start_move_z_pseudo_home(
        "first", {}, command_id="cmd-first", child_order=1, plan_digest="a" * 64,
    )
    provider.wp8_start_move_z_pseudo_home(
        "second", {}, command_id="cmd-second", child_order=1, plan_digest="b" * 64,
    )

    waited = provider.wp8_wait_move_z_only(
        "waitMoveZOnly", {"timeout_ms": 1000}, command_id="cmd-first",
        child_order=4, plan_digest="a" * 64,
    )

    assert waited["background_task_id"] == first["background_task_id"]
    assert waited["result"]["operation"] == "first"
    row = provider._wp8_tasks[first["background_task_id"]]
    assert row["command_id"] == "cmd-first"
    assert row["child_order"] == 1
    assert row["plan_digest"] == "a" * 64


def test_background_gripper_home_releases_only_exact_owner_in_finally() -> None:
    provider = _bare_wp8_provider()
    provider.wp8_lock_gripper(
        "LockGripperOperation", {}, command_id="cmd-owner", child_order=2,
        plan_digest="c" * 64, dispatch_attempt_id="dispatch-owner",
        ownership_generation=7, board_epoch_4=11, board_epoch_5=12,
    )
    provider.wp8_send_gripper_home = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        RuntimeError("home failed")
    )

    started = provider.wp8_start_gripper_home_and_unlock(
        "startGripperHomeAndUnlock", {}, command_id="cmd-owner", child_order=3,
        plan_digest="c" * 64,
    )
    row = provider._wp8_tasks[started["background_task_id"]]
    row["thread"].join(timeout=1.0)

    assert row["state"] == "failed"
    assert provider._wp8_gripper_lock_owner is None
    assert provider._wp8_gripper_lock.acquire(blocking=False) is True
    provider._wp8_gripper_lock.release()


def test_gripper_lock_release_requires_complete_immutable_token() -> None:
    provider = _bare_wp8_provider()
    identity = {
        "command_id": "cmd-token", "child_order": 4, "plan_digest": "f" * 64,
        "dispatch_attempt_id": "dispatch-1", "ownership_generation": 7,
        "board_epoch_4": 11, "board_epoch_5": 12,
    }
    acquired = provider.wp8_lock_gripper("LockGripperOperation", {}, **identity)
    token = acquired["lock_token"]

    with pytest.raises(RuntimeError, match="wrong_owner"):
        provider.wp8_release_gripper_lock(
            "ReleaseLockGripperOperation", {},
            lock_token={**token, "dispatch_attempt_id": "dispatch-stale"},
            **identity,
        )
    assert provider._wp8_gripper_lock.locked() is True
    assert provider._wp8_gripper_lock_owner.receipt() == token

    released = provider.wp8_release_gripper_lock(
        "ReleaseLockGripperOperation", {}, lock_token=token, **identity,
    )
    assert released["released_lock_token"] == token
    assert provider._wp8_gripper_lock_owner is None
    assert provider._wp8_gripper_lock.locked() is False


def test_nested_wrapper_gripper_lock_uses_outer_durable_child_owner(monkeypatch) -> None:
    provider = _bare_wp8_provider()
    monkeypatch.setattr(
        oem_deck_movement, "validate_compiled_wp8_machine_targets",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        "bioxp.oem_serial206_initialization.load_bound_oem_position_table",
        lambda: object(),
    )
    provider.wp8_operation_machine_state = lambda _operation, _inputs: {}
    provider.wp8_scriptmove_to = lambda *_args, **_kwargs: {"ok": True}
    provider.wp8_move_z_to_location = lambda *_args, **_kwargs: {"ok": True}
    provider.wp8_sleep = lambda *_args, **_kwargs: {"ok": True}
    observed: dict = {}

    def intercept_after_lock(*_args, **_kwargs):
        assert provider._wp8_gripper_lock_owner is not None
        observed.update(provider._wp8_gripper_lock_owner.receipt())
        raise RuntimeError("intercepted_after_lock")

    provider.wp8_set_gripper_current = intercept_after_lock
    outer_digest = "a" * 64
    outer_child = {
        "order": 17,
        "operation": "releasePlate",
        "arguments": {
            "destination": 25,
            "plate_on_gantry": 1,
            "output_plate_location": 29,
            "current_tray": 2,
            "press_plate": False,
            "gripper_version": 1,
            "run_in_parallel": True,
        },
        "_delivery_identity": {
            "dispatch_attempt_id": "dispatch-outer",
            "ownership_generation": 23,
            "board_epoch_4": 41,
            "board_epoch_5": 59,
        },
    }

    result = provider.execute_wp8_child(
        outer_child,
        command_id="command-outer",
        child_order=17,
        plan_digest=outer_digest,
    )
    assert result["failed_child"] == "setGripperCurrent"
    assert observed == {
        "command_id": "command-outer",
        "acquiring_identity": "child:17:releasePlate",
        "plan_digest": outer_digest,
        "dispatch_attempt_id": "dispatch-outer",
        "ownership_generation": 23,
        "board_epoch_4": 41,
        "board_epoch_5": 59,
    }

    nested_release = {
        "order": 99,
        "operation": "ReleaseLockGripperOperation",
        "arguments": {},
        "_delivery_identity": {
            "work_identity": "child:17:releasePlate",
            "plan_digest": outer_digest,
            "dispatch_attempt_id": "dispatch-stale",
            "ownership_generation": 23,
            "board_epoch_4": 41,
            "board_epoch_5": 59,
        },
    }
    with pytest.raises(RuntimeError, match="wp8_gripper_lock_wrong_owner"):
        provider.execute_wp8_child(
            nested_release,
            command_id="command-outer",
            child_order=99,
            plan_digest="nested-plan-digest",
        )
    assert provider._wp8_gripper_lock.locked() is True
    assert provider._wp8_gripper_lock_owner.receipt() == observed

    nested_release["_delivery_identity"]["dispatch_attempt_id"] = "dispatch-outer"
    nested_release["_delivery_identity"]["work_identity"] = "child:18:releasePlate"
    with pytest.raises(RuntimeError, match="wp8_gripper_lock_wrong_owner"):
        provider.execute_wp8_child(
            nested_release,
            command_id="command-outer",
            child_order=99,
            plan_digest="nested-plan-digest",
        )
    assert provider._wp8_gripper_lock.locked() is True
    assert provider._wp8_gripper_lock_owner.receipt() == observed

    nested_release["_delivery_identity"]["work_identity"] = "child:17:releasePlate"
    released = provider.execute_wp8_child(
        nested_release,
        command_id="command-outer",
        child_order=99,
        plan_digest="nested-plan-digest",
    )
    assert released["released_lock_token"] == observed
    assert provider._wp8_gripper_lock_owner is None
    assert provider._wp8_gripper_lock.locked() is False


def test_background_gripper_home_durably_settles_after_unawaited_return() -> None:
    provider = _bare_wp8_provider()
    release = threading.Event()
    settlements: list[tuple[str, str, dict]] = []
    provider.wp8_send_gripper_home = lambda *_args, **_kwargs: (
        release.wait(1.0) and {"ok": True, "delivery_attempted": True}
    )
    provider.bind_wp8_background_task_settler(
        lambda task_id, *, state, evidence: settlements.append(
            (task_id, state, dict(evidence))
        )
    )
    provider.wp8_lock_gripper(
        "LockGripperOperation", {}, command_id="cmd-durable", child_order=2,
        plan_digest="d" * 64, dispatch_attempt_id="dispatch-durable",
        ownership_generation=7, board_epoch_4=11, board_epoch_5=12,
    )

    started = provider.wp8_start_gripper_home_and_unlock(
        "startGripperHomeAndUnlock", {}, command_id="cmd-durable", child_order=3,
        plan_digest="d" * 64,
    )
    assert settlements == []
    release.set()
    provider._wp8_tasks[started["background_task_id"]]["thread"].join(timeout=1.0)

    assert settlements == [(
        started["background_task_id"], "completed",
        {"result": provider._wp8_tasks[started["background_task_id"]]["result"], "error": None},
    )]


def test_parallel_home_plan_allows_parent_return_for_final_unawaited_gripper() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True,
        run_in_parallel=True, gripper_position=4000, closed_position=3000,
    )
    assert plan["parent_return_allows_background_pending"] is True
    assert [child["operation"] for child in plan["children"] if not child["awaited"]] == [
        "startMoveZPseudoHome", "startGripperHomeAndUnlock",
    ]


def test_wp8_delivery_marker_must_commit_before_provider_call() -> None:
    calls: list[str] = []

    class Store:
        def assert_deck_execution_current(self, *_args, **_kwargs):
            return None
        def persist_wp8_plan(self, *_args, **_kwargs):
            return None
        def create_wp8_background_task(self, *_args, **_kwargs):
            calls.append("marker")
            raise RuntimeError("marker commit failed")
        def record_delivery_attempt(self, *_args, **_kwargs):
            calls.append("marker")
            raise RuntimeError("marker commit failed")
        def terminalize_wp8_child(self, *_args, **_kwargs):
            calls.append("terminal")
        def persist_wp8_state_mutation(self, *_args, **_kwargs):
            return None
        def finalize_wp8_operation(self, *_args, **_kwargs):
            return None

    class Provider:
        def movement_lease(self):
            return nullcontext()
        def deck_owner_authority_stamps(self):
            return {"ownership_generation": 7, "board_epoch_4": 11, "board_epoch_5": 12}
        def execute_wp8_child(self, *_args, **_kwargs):
            calls.append("provider")
            return {"ok": True, "delivery_attempted": True}

    plan = oem_deck_movement.compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True,
        run_in_parallel=True, gripper_position=4000, closed_position=3000,
    )
    with pytest.raises(RuntimeError, match="marker commit failed"):
        oem_deck_movement.make_wp8_operation_executor(
            provider_getter=Provider, command_store=Store(),
        )(command_id="durable-command", plan=plan)
    assert calls == ["marker"]


def test_wp8_outer_completion_uses_committed_marker_after_stale_provider_return(monkeypatch) -> None:
    class Store:
        def action_fenced(self, _action_id):
            return False
        def wp8_operation_evidence(self, _command_id):
            return {"children": [{"terminal_state": "planned"}]}
        def has_delivery_attempt(self, _command_id):
            return True
        def mark_deck_recovery_required(self, *args, **kwargs):
            self.recovery = (args, kwargs)
        def finish(self, command_id, **kwargs):
            self.finished = (command_id, kwargs)

    provider = SimpleNamespace(
        wp8_operation_machine_state=lambda _operation, _inputs: {},
        execute_wp8_child=lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        "bioxp.operator_command_plane.compile_finite_plate_operation",
        lambda *_args, **_kwargs: {"plan_digest": "a" * 64},
    )
    app = SimpleNamespace(state=SimpleNamespace(
        oem_deck_provider=provider,
        oem_wp8_operation_executor=lambda **_kwargs: (_ for _ in ()).throw(
            RuntimeError("deck_execution_dispatch_authority_stale")
        ),
    ))
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = app
    plane.store = Store()
    plane.machine_state_provider = lambda: {}

    plane._dispatch_one({
        "command_id": "wp8-stale-marker", "action_id": "oem.deck._finite_operation",
        "requested_inputs": {},
        "effective_inputs": {"operation": "park_gantry", "operation_inputs": {}},
    })

    assert plane.store.finished[1]["status"] == "ambiguous"
    assert plane.store.finished[1]["payload"]["delivery_attempted"] is True
    assert plane.store.recovery[0] == ("wp8-stale-marker",)


def test_wp8_parent_can_return_with_durably_owned_background_task_when_plan_allows_it() -> None:
    class Store:
        def __init__(self):
            self.tasks = {}
            self.finalized = None
        def assert_deck_execution_current(self, *_args, **_kwargs):
            return None
        def persist_wp8_plan(self, *_args, **_kwargs):
            return None
        def create_wp8_background_task(self, command_id, child_order, **kwargs):
            self.tasks[(command_id, child_order)] = {"state": "created", **kwargs}
            return {"attempt_sequence": 1, "dispatch_attempt_id": "dispatch-1"}
        def record_delivery_attempt(self, *_args, **_kwargs):
            return {"attempt_sequence": 1, "dispatch_attempt_id": "dispatch-1"}
        def mark_wp8_background_task(self, command_id, child_order, *, state, evidence):
            self.tasks[(command_id, child_order)].update(state=state, evidence=evidence)
        def assert_wp8_background_tasks_settled(self, command_id):
            if any(key[0] == command_id and row["state"] not in {"completed", "failed"}
                   for key, row in self.tasks.items()):
                raise RuntimeError("wp8_background_task_unsettled")
        def terminalize_wp8_child(self, *_args, **_kwargs):
            return None
        def persist_wp8_state_mutation(self, *_args, **_kwargs):
            return None
        def finalize_wp8_operation(self, _command_id, result):
            self.finalized = result

    class Provider:
        def movement_lease(self):
            return nullcontext()
        def deck_owner_authority_stamps(self):
            return {"ownership_generation": 7, "board_epoch_4": 11, "board_epoch_5": 12}
        def execute_wp8_child(self, child, *, command_id, child_order, plan_digest):
            return {
                "ok": True,
                "delivery_attempted": True,
                "background_task_id": f"{command_id}:{child_order}:{plan_digest}:running",
                "background_task_state": "running_unawaited",
            }

    plan = oem_deck_movement.compile_finite_plate_operation(
        "press_plates", source_leaf_available=True,
        plates=[2, 0], plate_locations={2: 3, 0: 23}, run_in_parallel=True,
    )
    store = Store()
    result = oem_deck_movement.make_wp8_operation_executor(
        provider_getter=Provider, command_store=store,
    )(command_id="durable-command", plan=plan)
    assert result["ok"] is True
    assert result["background_pending"] is True
    assert any(row["state"] == "running" for row in store.tasks.values())
    assert store.finalized is None
