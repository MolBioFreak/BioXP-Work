from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

from .machine_state import OemMachineState
from .position_table import PositionTable, OEM_X_INCREMENT, OEM_Y_INCREMENT

LOCATION_ID_TO_NAME = {
    0: "LOC_MS", 1: "LOC_OC", 2: "LOC_TC", 3: "LOC_RC", 4: "LOC_BSCS", 5: "LOC_BSC", 6: "WASTE_BIN",
    7: "TECANRACK1", 8: "TECANRACK2", 9: "TECANRACK3", 10: "TECANRACK4", 11: "LOC_STRIP1", 12: "LOC_STRIP2",
    13: "LOC_STRIP3", 14: "LOC_STRIP4", 15: "LOC_TIP_HOTEL", 16: "LOC_TROUGH", 17: "LOC_OC_COVER",
    18: "LOC_OC_COVER_STORAGE", 19: "LOC_RC_COVER", 20: "LOC_RC_COVER_STORAGE", 21: "LOC_P_OC", 22: "LOC_P_OC_PRESS",
    23: "LOC_P_TC", 24: "LOC_P_TC_PRESS", 25: "LOC_P_MS", 26: "LOC_P_MS_PRESS", 27: "LOC_P_RC_PRESS",
    28: "LOC_PARK", 29: "LOC_GANTRY", 30: "LOC_CHECK_POINT", 31: "CAMERA_OFFSET", 32: "UNKNOWN",
}
LOCATION_NAME_TO_ID = {v: k for k, v in LOCATION_ID_TO_NAME.items()}
TECAN_RACK_IDS = {7, 8, 9, 10}
TECAN_RACK_OR_STRIP4_IDS = {7, 8, 9, 10, 15}
WASTE_BIN_ID = 6
SOURCE_LINES = "ClassControlInterface.cs:3734-4014"
MIDPOINT_LINES = "ClassControlInterface.cs:5254-5366"
XY_MM_TO_STEPS = 236.94


def _loc_name(value: str | int | None) -> str:
    if value is None:
        return "UNKNOWN"
    if isinstance(value, int) or (isinstance(value, str) and value.strip().lstrip('-').isdigit()):
        return LOCATION_ID_TO_NAME.get(int(value), f"UNKNOWN_{value}")
    return str(value).strip().upper()


def _loc_id(value: str | int | None) -> int:
    if value is None:
        return LOCATION_NAME_TO_ID.get("UNKNOWN", 32)
    if isinstance(value, int) or (isinstance(value, str) and value.strip().lstrip('-').isdigit()):
        return int(value)
    return LOCATION_NAME_TO_ID.get(str(value).strip().upper(), 32)


def _step(kind: str, **kwargs: Any) -> dict[str, Any]:
    payload = {"op": kind, "motion_commanded": False, "opened_usb": False, "physical_motion": False}
    payload.update(kwargs)
    return payload


@dataclass(frozen=True)
class OemPathPlanner:
    table: PositionTable
    x_high_limit: int = 90263
    y_high_limit: int = 102956

    def _target(self, location_id: str | int):
        return self.table.resolve(location_id=_loc_name(location_id))

    def _xy_z_for_script_target(self, location_id: str | int, *, column: int, row: int, positionflag: int, state: OemMachineState) -> dict[str, int]:
        target = self._target(location_id)
        loc_id = _loc_id(location_id)
        effective_row = int(row)
        if loc_id not in {7, 8, 9, 10, 6, 16}:  # exact OEM numeric-exempt set, line 3834
            effective_row = int(row) - (state.tip_location if state.tip_location != -1 else 0) * 2
        if int(positionflag) == 0:
            z = state.default_parameters.pseudo_z_home
        elif int(positionflag) == 1:
            z = target.z_high
        else:
            z = target.z_low
        if z is None:
            z = state.default_parameters.pseudo_z_home
        return {
            "x": int(target.base_coordinates.get("x", 0)) + int(target.inc_factor) * OEM_X_INCREMENT * int(column),
            "y": int(target.base_coordinates.get("y", 0)) + int(target.inc_factor) * OEM_Y_INCREMENT * effective_row,
            "z": int(z),
        }

    def get_midpoints(self, *, current_x: int, current_y: int, target_x: int, target_y: int) -> list[dict[str, int]]:
        """Port of ClassControlInterface.getMidPoint(x,y), source lines 5254-5366."""
        p1 = self._target(1)   # LOC_OC
        p2 = self._target(2)   # LOC_TC
        p3 = self._target(3)   # LOC_RC
        p11 = self._target(11) # LOC_STRIP1
        p14 = self._target(14) # LOC_STRIP4
        x2, y2, x, y = int(current_x), int(current_y), int(target_x), int(target_y)
        points: list[dict[str, int]] = []
        if x2 < int(p2.base_coordinates["x"]) + 2369.4 and y2 < int(p3.base_coordinates["y"]) + 3554.1:
            if x < int(p2.base_coordinates["x"]) + 2369.4 and y < int(p3.base_coordinates["y"]) + 3554.1:
                points.append({"x": x, "y": y})
            elif y2 > int(p1.base_coordinates["y"]) - 1184.7:
                points += [{"x": x, "y": y2}, {"x": x, "y": y}]
            elif x2 > int(p14.base_coordinates["x"]) - 1184.7:
                points += [{"x": x2, "y": y}, {"x": x, "y": y}]
            else:
                num = int(p11.base_coordinates["x"]); num2 = int(p1.base_coordinates["y"])
                if num > x: num = x
                if num < x2: num = x2
                if num2 < y2: num2 = y2
                if num2 > y: num2 = y
                points += [{"x": num, "y": num2}, {"x": x, "y": y}]
        elif x2 > int(p14.base_coordinates["x"]) - 1184.7 and y2 > int(p1.base_coordinates["y"]) - 1184.7:
            if x > int(p14.base_coordinates["x"]) - 1184.7 and y > int(p1.base_coordinates["y"]) - 1184.7:
                points.append({"x": x, "y": y})
            elif x2 < int(p2.base_coordinates["x"]) + 2369.4:
                points += [{"x": x2, "y": y}, {"x": x, "y": y}]
            elif y2 < int(p3.base_coordinates["y"]) + 3554.1:
                points += [{"x": x, "y": y2}, {"x": x, "y": y}]
            elif y > int(p1.base_coordinates["y"]) - 1184.7:
                points += [{"x": x2, "y": y}, {"x": x, "y": y}]
            else:
                num = int(p11.base_coordinates["x"]) - 5500
                num2 = int(p1.base_coordinates["y"])
                if num2 < y: num2 = y
                points += [{"x": num, "y": num2}, {"x": x, "y": y}]
        return points

    @staticmethod
    def _move_to_authority(state: OemMachineState) -> dict[str, Any]:
        """Carry the exact dynamic predicates required by the OEM moveTo branch."""
        return {
            "gripper_confirmed": bool(state.confirm_axis("gripper")),
            "tip_loaded": bool(state.tip_loaded),
            "plate_on_gantry": state.plate_on_gantry,
            "location19_y": state.location19_y,
        }

    def plan_script_move_to(self, *, current_loc: str | int | None, location_id: str | int, column: int = 0, row: int = 0, positionflag: int = 0, state: OemMachineState, run_in_parallel: bool = True) -> dict[str, Any]:
        loc_id = _loc_id(location_id); current_id = _loc_id(current_loc)
        target = self._target(location_id)
        target_xyz = self._xy_z_for_script_target(location_id, column=column, row=row, positionflag=positionflag, state=state)
        x, y, num = target_xyz["x"], target_xyz["y"], target_xyz["z"]
        x2, y2, z = int(state.current_x), int(state.current_y), int(state.current_z)
        pseudo = int(state.default_parameters.pseudo_z_home)
        steps: list[dict[str, Any]] = []
        branch = "unassigned"

        if x == x2 and y == y2:
            branch = "same_xy_move_z"
            steps.append(_step("moveZ", z=num, source_lines="ClassControlInterface.cs:3853-3856"))
        elif state.confirm_axis("gripper") and not state.tip_loaded:
            branch = "gripper_confirmed_no_tip_direct_moveTo"
            steps.append(_step("moveTo", x=x, y=y, z=num, run_in_parallel=bool(run_in_parallel), move_to_authority=self._move_to_authority(state), source_lines="ClassControlInterface.cs:3858-3860"))
        elif state.tip_loaded:
            branch = "tip_loaded"
            if z > pseudo:
                steps.append(_step("moveZ", z=pseudo, reason="current_z_above_pseudo_home", source_lines="ClassControlInterface.cs:3864-3867"))
            mid = self.get_midpoints(current_x=x2, current_y=y2, target_x=x, target_y=y)
            if current_id in TECAN_RACK_OR_STRIP4_IDS and (loc_id != 20 or loc_id != 18) and loc_id != WASTE_BIN_ID:
                branch = "tip_loaded_from_rack_or_strip4_direct_xy_then_z"
                steps += [_step("moveXY", x=x, y=y, source_lines="ClassControlInterface.cs:3869-3872"), _step("sleep", milliseconds=50), _step("moveZ", z=num, source_lines="ClassControlInterface.cs:3871-3873")]
            elif len(mid) != 0 and loc_id != WASTE_BIN_ID:
                branch = "tip_loaded_midpoint_non_waste"
                if state.clean_path and (loc_id != 20 or loc_id != 18):
                    steps.append(_step("moveXY", x=x, y=y, source_lines="ClassControlInterface.cs:3875-3880"))
                else:
                    steps += [_step("moveXY", **pt, source_lines="ClassControlInterface.cs:3882-3887") for pt in mid]
                steps.append(_step("moveZ", z=num, source_lines="ClassControlInterface.cs:3888"))
            elif len(mid) != 0 and loc_id == WASTE_BIN_ID:
                branch = "tip_loaded_midpoint_waste_bin"
                if state.clean_path:
                    steps.append(_step("moveXY", x=82450, y=y, source_lines="ClassControlInterface.cs:3890-3895"))
                else:
                    mid2 = self.get_midpoints(current_x=x2, current_y=y2, target_x=82450, target_y=y)
                    steps += [_step("moveXY", **pt, source_lines="ClassControlInterface.cs:3896-3903") for pt in mid2]
                if str(state.device_type).upper() == "DBC":
                    steps.append(_step("moveZ", z=75000, source_lines="ClassControlInterface.cs:3904-3907"))
                else:
                    steps += [_step("moveZ", z=145000, source_lines="ClassControlInterface.cs:3908-3910"), _step("moveX", x=self.x_high_limit - 500, source_lines="ClassControlInterface.cs:3911"), _step("moveSteps", axis="x", delta=-1000, source_lines="ClassControlInterface.cs:3912"), _step("moveZ", z=pseudo, source_lines="ClassControlInterface.cs:3913")]
            elif abs(x - x2) < 3000 or (abs(y - y2) < 3000 and loc_id != WASTE_BIN_ID):
                branch = "tip_loaded_near_axis"
                if run_in_parallel:
                    steps.append(_step("parallel", steps=[_step("moveX", x=x), _step("moveY", y=y)], source_lines="ClassControlInterface.cs:3916-3928"))
                    if num > pseudo:
                        steps.append(_step("moveZ", z=num, source_lines="ClassControlInterface.cs:3929-3932"))
                else:
                    steps += [_step("moveXY", x=x, y=y, source_lines="ClassControlInterface.cs:3934-3937"), _step("moveZ", z=num)]
            elif current_id in TECAN_RACK_IDS and loc_id != WASTE_BIN_ID:
                branch = "tip_loaded_from_tecan_rack_to_non_waste"
                if loc_id in {11, 12, 13, 14, 16, 6}:
                    steps += [_step("moveX", x=x, source_lines="ClassControlInterface.cs:3940-3945"), _step("moveY", y=y)]
                else:
                    steps += [_step("moveX", x=27750, source_lines="ClassControlInterface.cs:3948-3951"), _step("moveY", y=y), _step("moveX", x=x)]
                if num > pseudo:
                    steps.append(_step("moveZ", z=num, source_lines="ClassControlInterface.cs:3953-3956"))
            elif loc_id == WASTE_BIN_ID:
                branch = "tip_loaded_waste_bin_special"
                if current_id in {2, 0, 3, 1}:
                    p3 = self._target(3)
                    steps += [_step("moveY", y=int(p3.base_coordinates["y"]) + OEM_Y_INCREMENT // 2, source_lines="ClassControlInterface.cs:3960-3968"), _step("moveX", x=82450), _step("moveY", y=y), _step("moveZ", z=145000), _step("moveX", x=self.x_high_limit - 500), _step("moveSteps", axis="x", delta=-1000), _step("moveZ", z=pseudo)]
                else:
                    p19 = self._target(19)
                    if y2 < int(p19.base_coordinates["y"]):
                        steps += [_step("moveY", y=int(p19.base_coordinates["y"]), source_lines="ClassControlInterface.cs:3971-3982"), _step("moveX", x=76050)]
                    steps += [_step("moveY", y=y), _step("moveX", x=82450), _step("moveZ", z=145000), _step("moveX", x=self.x_high_limit - 500), _step("moveSteps", axis="x", delta=-1000), _step("moveZ", z=pseudo)]
            else:
                branch = "tip_loaded_default_moveTo"
                steps.append(_step("moveTo", x=x, y=y, z=num, run_in_parallel=bool(run_in_parallel), move_to_authority=self._move_to_authority(state), source_lines="ClassControlInterface.cs:3985-3988"))
        elif not state.tip_dirty:
            branch = "no_tip_not_dirty_default_moveTo"
            steps.append(_step("moveTo", x=x, y=y, z=num, run_in_parallel=bool(run_in_parallel), move_to_authority=self._move_to_authority(state), source_lines="ClassControlInterface.cs:3990-3992"))
        elif loc_id == WASTE_BIN_ID:
            branch = "dirty_tip_waste_bin_special"
            steps.append(_step("moveZ", z=pseudo, source_lines="ClassControlInterface.cs:3994-4007"))
            if y2 < 46600:
                steps += [_step("moveY", y=46600), _step("moveX", x=76050)]
            steps += [_step("moveY", y=y), _step("moveX", x=82450), _step("moveZ", z=145000), _step("moveX", x=self.x_high_limit - 500), _step("moveSteps", axis="x", delta=-1000), _step("moveZ", z=pseudo)]
        else:
            branch = "dirty_tip_default_moveTo"
            steps.append(_step("moveTo", x=x, y=y, z=num, run_in_parallel=bool(run_in_parallel), move_to_authority=self._move_to_authority(state), source_lines="ClassControlInterface.cs:4009-4011"))

        return {
            "ok": True,
            "schema_version": "bioxp.oem_scriptmove_path_plan.v1",
            "semantic_action": "scriptmoveTo",
            "branch": branch,
            "source_formula": SOURCE_LINES,
            "midpoint_source_formula": MIDPOINT_LINES,
            "current_location_id": _loc_name(current_loc),
            "target_location_id": _loc_name(location_id),
            "column": int(column),
            "row": int(row),
            "positionflag": int(positionflag),
            "target": target.to_payload(),
            "target_coordinates": target_xyz,
            "state": state.to_payload(),
            "steps": steps,
            "step_count": len(steps),
            "show_gantry": {"x_mm": x / XY_MM_TO_STEPS, "y_mm": y / XY_MM_TO_STEPS, "source_lines": "ClassControlInterface.cs:4013"},
            "opened_usb": False,
            "physical_motion": False,
            "motion_commanded": False,
            "controller_command_planned": False,
        }
