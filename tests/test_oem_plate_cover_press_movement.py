from __future__ import annotations

import pytest

from bioxp import oem_deck_movement


def ops(plan):
    return [row["operation"] for row in plan["children"]]


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
    assert plan["exception_policy"] == "log_and_suppress"
    assert plan["finally_children"] == []
    assert plan["residual_policy"] == "retain_completed_children_without_rollback"


def test_pressplates_internal_targets_array_order_and_background_home() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "press_plates", source_leaf_available=True, plates=[2, 99, 0, 1],
        plate_locations={0: 23, 1: 21, 2: 3}, run_in_parallel=True,
    )
    presses = [row for row in plan["children"] if row["operation"] == "pressPlate"]
    assert [(row["arguments"]["plate"], row["arguments"]["pressure_target"]) for row in presses] == [(2, 27), (0, 24), (1, 22)]
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
    low = oem_deck_movement.compile_finite_plate_operation(
        "thermal_door", source_leaf_available=True, open=True, serial=10,
        board_test_mode=False, current_location=1,
    )
    assert ops(low) == ["parkGantry", "setDoorStallThresholdPlus2", "setDoorMaxCurrent", "moveDoorOpen", "readDoorSensors", "HomeAxisD", "setDoorStallThresholdPlus2", "setDoorMaxCurrent", "moveDoorOpen", "readDoorSensors", "updateThermalDoorOpen"]
    assert low["normal_state_update_unconditional"] is True
    board = oem_deck_movement.compile_finite_plate_operation(
        "thermal_door", source_leaf_available=True, open=False, serial=10,
        board_test_mode=True, current_location=28,
    )
    assert board["success_return"] is False
    assert board["failure_policy"] == "throw"


def test_cleanup_keeps_distinct_waste_and_cover_storage_order() -> None:
    plan = oem_deck_movement.compile_finite_plate_operation(
        "cleanup", source_leaf_available=True, tip_exists=True, door_status_ok=True,
        current_location=1, cover_locations={4: 17, 5: 19},
    )
    sequence = ops(plan)
    assert sequence[:8] == ["waitStop", "checkDoorStatus", "queryTipStatus", "scriptmoveToWaste", "updateLocation", "ejectAllTipsCleanup", "moveZ80000", "moveX79000"]
    assert sequence.index("sendGripperHome") < sequence.index("storeOutputCover4To18") < sequence.index("storeReagentCover5To20") < sequence.index("doorOpenClose") < sequence.index("parkGantry")
    assert "destination != 20 || destination != 18" in plan["source_hazards"]


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
