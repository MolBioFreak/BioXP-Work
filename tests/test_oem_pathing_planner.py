from src.bioxp.oem_compat.machine_state import OemMachineState, OemDefaultParameters
from src.bioxp.oem_compat.pathing import OemPathPlanner
from src.bioxp.oem_compat.position_table import PositionTable


def _planner():
    rows = [
        {"name": "LOC_MS", "x": 1000, "y": 1000, "zLow": 80000, "zDelta": 30000, "inc_factor": 1},
        {"name": "LOC_OC", "x": 5000, "y": 5000, "zLow": 80000, "zDelta": 30000, "inc_factor": 1},
        {"name": "LOC_TC", "x": 20000, "y": 10000, "zLow": 80000, "zDelta": 30000, "inc_factor": 1},
        {"name": "LOC_RC", "x": 30000, "y": 20000, "zLow": 80000, "zDelta": 30000, "inc_factor": 1},
        {"name": "WASTE_BIN", "x": 90000, "y": 93000, "zLow": 0, "zDelta": 0, "inc_factor": 0},
        {"name": "TECANRACK1", "x": 2000, "y": 90000, "zLow": 75000, "zDelta": 20000, "inc_factor": 1},
        {"name": "TECANRACK2", "x": 3000, "y": 90000, "zLow": 75000, "zDelta": 20000, "inc_factor": 1},
        {"name": "TECANRACK3", "x": 4000, "y": 90000, "zLow": 75000, "zDelta": 20000, "inc_factor": 1},
        {"name": "TECANRACK4", "x": 5000, "y": 90000, "zLow": 75000, "zDelta": 20000, "inc_factor": 1},
        {"name": "LOC_STRIP1", "x": 40000, "y": 50000, "zLow": 70000, "zDelta": 10000, "inc_factor": 1},
        {"name": "LOC_STRIP2", "x": 42000, "y": 52000, "zLow": 70000, "zDelta": 10000, "inc_factor": 1},
        {"name": "LOC_STRIP3", "x": 44000, "y": 54000, "zLow": 70000, "zDelta": 10000, "inc_factor": 1},
        {"name": "LOC_STRIP4", "x": 46000, "y": 56000, "zLow": 70000, "zDelta": 10000, "inc_factor": 1},
        {"name": "LOC_TROUGH", "x": 60000, "y": 60000, "zLow": 70000, "zDelta": 10000, "inc_factor": 1},
        {"name": "LOC_RC_COVER", "x": 65000, "y": 65000, "zLow": 70000, "zDelta": 10000, "inc_factor": 1},
    ]
    return OemPathPlanner(PositionTable.from_rows(rows), x_high_limit=90263, y_high_limit=102956)


def test_gripper_confirmed_no_tip_direct_branch_is_read_only():
    planner = _planner()
    state = OemMachineState(current_x=0, current_y=0, current_z=0, tip_loaded=False, axis_confirmed={"gripper": True})
    plan = planner.plan_script_move_to(current_loc="LOC_MS", location_id="LOC_MS", column=2, row=3, positionflag=1, state=state)
    assert plan["branch"] == "gripper_confirmed_no_tip_direct_moveTo"
    assert plan["target_coordinates"] == {"x": 1000 - 2132 * 2, "y": 1000 + 2132 * 3, "z": 50000}
    assert plan["steps"][0]["op"] == "moveTo"
    assert plan["motion_commanded"] is False
    assert plan["steps"][0]["physical_motion"] is False


def test_same_xy_branch_moves_z_only():
    planner = _planner()
    state = OemMachineState(current_x=1000, current_y=1000, current_z=1)
    plan = planner.plan_script_move_to(current_loc="LOC_MS", location_id="LOC_MS", positionflag=2, state=state)
    assert plan["branch"] == "same_xy_move_z"
    assert [s["op"] for s in plan["steps"]] == ["moveZ"]
    assert plan["steps"][0]["z"] == 80000


def test_tip_loaded_near_axis_parallel_branch_uses_pseudo_home_guard():
    planner = _planner()
    state = OemMachineState(current_x=900, current_y=50000, current_z=70000, tip_loaded=True, default_parameters=OemDefaultParameters(65000))
    plan = planner.plan_script_move_to(current_loc="LOC_MS", location_id="LOC_MS", positionflag=1, state=state)
    assert plan["steps"][0] == {"op": "moveZ", "motion_commanded": False, "opened_usb": False, "physical_motion": False, "z": 65000, "reason": "current_z_above_pseudo_home", "source_lines": "ClassControlInterface.cs:3864-3867"}
    assert plan["branch"] in {"tip_loaded_near_axis", "tip_loaded_midpoint_non_waste"}


def test_waste_bin_special_branch_emits_oem_bin_sequence():
    planner = _planner()
    state = OemMachineState(current_location_id="LOC_MS", current_x=30000, current_y=30000, current_z=80000, tip_loaded=True)
    plan = planner.plan_script_move_to(current_loc="LOC_MS", location_id="WASTE_BIN", state=state)
    ops = [s["op"] for s in plan["steps"]]
    assert plan["branch"] == "tip_loaded_waste_bin_special"
    assert "moveSteps" in ops
    assert any(s.get("z") == 145000 for s in plan["steps"])
    assert plan["steps"][-1]["z"] == 65000


def test_dirty_tip_waste_bin_branch_matches_dirty_fallback():
    planner = _planner()
    state = OemMachineState(current_x=1, current_y=100, current_z=1, tip_loaded=False, tip_dirty=True)
    plan = planner.plan_script_move_to(current_loc="LOC_MS", location_id="WASTE_BIN", state=state)
    assert plan["branch"] == "dirty_tip_waste_bin_special"
    assert [s["op"] for s in plan["steps"][:3]] == ["moveZ", "moveY", "moveX"]
