from __future__ import annotations

from dataclasses import dataclass
from typing import Any

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
# Fixed deck stations of tip trays 1-5 (ControlLib constructor). Hardware facts, not occupancy.
TIP_TRAY_LOCATIONS = (7, 8, 9, 10, 15)
TECAN_DIRECT_DESTINATION_IDS = frozenset({11, 12, 13, 14, 16})
WASTE_BIN_ID = 6
SOURCE_LINES = "ClassControlInterface.cs:3734-4014"
MIDPOINT_LINES = "ClassControlInterface.cs:5254-5366"
SCRIPTMOVE_TO_METHOD_TOKEN = "0x06000120"
SCRIPTMOVE_TO_IL_SHA256 = "3e94ea68ac09edbbeb021907285e9e47206c07d118e5f3a9f5611cd03c491ce3"
XY_MM_TO_STEPS = 236.94
CLOSED_DOOR_EXEMPT_TIP_TYPES = frozenset({10})
# Closed thermal door bezel clearance around LOC_TC wells (matches approved deck map: ~21 mm X, ~19 mm Y).
CLOSED_DOOR_MARGIN_X_STEPS = int(round(21.0 * XY_MM_TO_STEPS))
CLOSED_DOOR_MARGIN_Y_STEPS = int(round(19.0 * XY_MM_TO_STEPS))


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


def _point_in_box(x: int, y: int, *, x_min: int, x_max: int, y_min: int, y_max: int) -> bool:
    return x_min <= x <= x_max and y_min <= y <= y_max


def _segment_intersects_box(
    x1: int, y1: int, x2: int, y2: int, *, x_min: int, x_max: int, y_min: int, y_max: int
) -> bool:
    """Exact Liang-Barsky segment vs axis-aligned box intersection."""
    dx = x2 - x1
    dy = y2 - y1
    t0, t1 = 0.0, 1.0
    for p_val, q_val in ((-dx, x1 - x_min), (dx, x_max - x1), (-dy, y1 - y_min), (dy, y_max - y1)):
        if p_val == 0:
            if q_val < 0:
                return False
        else:
            r = q_val / p_val
            if p_val < 0:
                if r > t1:
                    return False
                if r > t0:
                    t0 = r
            else:
                if r < t0:
                    return False
                if r < t1:
                    t1 = r
    return t0 <= t1


def _leg_rect_overlaps_box(
    x1: int, y1: int, x2: int, y2: int, *, x_min: int, x_max: int, y_min: int, y_max: int
) -> bool:
    """True when the leg's bounding rectangle touches the box.

    X and Y run on independent motors, so the head does not follow the straight
    line between two points; it stays somewhere inside their bounding rectangle.
    """
    return not (max(x1, x2) < x_min or min(x1, x2) > x_max or max(y1, y2) < y_min or min(y1, y2) > y_max)


def xy_travel_legs(start_x: int, start_y: int, steps: list[dict[str, Any]]) -> list[tuple[int, int, int, int]]:
    """Expand planned steps into XY legs (start and end of each horizontal travel)."""
    legs: list[tuple[int, int, int, int]] = []
    cx, cy = int(start_x), int(start_y)

    def go(nx: int, ny: int) -> None:
        nonlocal cx, cy
        if (nx, ny) != (cx, cy):
            legs.append((cx, cy, nx, ny))
        cx, cy = nx, ny

    for step in steps:
        op = step.get("op")
        if op in ("moveXY", "moveTo"):
            go(int(step.get("x", cx)), int(step.get("y", cy)))
        elif op == "moveX":
            go(int(step["x"]), cy)
        elif op == "moveY":
            go(cx, int(step["y"]))
        elif op == "moveSteps":
            if step.get("axis") == "x":
                go(cx + int(step["delta"]), cy)
            elif step.get("axis") == "y":
                go(cx, cy + int(step["delta"]))
        elif op == "parallel":
            nx, ny = cx, cy
            for sub in step.get("steps") or ():
                if sub.get("op") == "moveX":
                    nx = int(sub["x"])
                elif sub.get("op") == "moveY":
                    ny = int(sub["y"])
            go(nx, ny)
    return legs


def workflow_thermal_door_policy(document: Any) -> dict[str, Any]:
    """Decide whether a pipetting workflow should open the thermal door at launch.

    Matches OEM default (open door before pipetting) unless:
    - The workflow explicitly opts to keep the thermal door closed, or
    - The workflow only uses 10 uL tips (which clear the closed door at Z=500), or
    - The workflow contains no pipetting actions.
    """
    doc = document if isinstance(document, dict) else {}
    metadata = doc.get("metadata") if isinstance(doc.get("metadata"), dict) else {}
    bms_meta = metadata.get("bms_method") if isinstance(metadata.get("bms_method"), dict) else {}
    bms_method = bms_meta.get("method") if isinstance(bms_meta.get("method"), dict) else {}
    for container in (metadata, bms_meta, bms_method):
        if container.get("keep_thermal_door_closed") is True or container.get("thermal_door_policy") == "keep_closed":
            return {"should_open": False, "reason": "explicit_keep_closed", "tip_types": []}
    has_pipetting = False
    tip_types: set[int] = set()
    untyped_tip_load = False
    pipette_kinds = {
        "pipette_manual_physical", "pipette_tip", "pipette_aspirate",
        "pipette_dispense", "pipette_mix", "pipette_position",
    }
    for stage in doc.get("stages") or ():
        if not isinstance(stage, dict):
            continue
        for action in stage.get("actions") or ():
            if not isinstance(action, dict):
                continue
            kind = str(action.get("kind") or "")
            params = action.get("params") if isinstance(action.get("params"), dict) else {}
            if kind in pipette_kinds:
                has_pipetting = True
            op = str(params.get("operation") or "")
            if (kind == "pipette_manual_physical" and op in {"load_tip", "source_load_tips"}) or kind == "pipette_tip":
                raw_tip = params.get("tip_type")
                if isinstance(raw_tip, int) and not isinstance(raw_tip, bool):
                    tip_types.add(raw_tip)
                elif op in {"load_tip", "source_load_tips"}:
                    untyped_tip_load = True
    if not has_pipetting:
        return {"should_open": False, "reason": "no_pipetting_actions", "tip_types": []}
    if tip_types and not untyped_tip_load and tip_types.issubset(CLOSED_DOOR_EXEMPT_TIP_TYPES):
        return {"should_open": False, "reason": "exempt_10ul_only", "tip_types": sorted(tip_types)}
    return {"should_open": True, "reason": "default_open_for_non_10ul_pipetting", "tip_types": sorted(tip_types)}


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
        elif int(positionflag) in (1, 2) and state.tip_loaded:
            # Loaded-tip length correction (0 unless measured); clearance height is untouched.
            z = int(z) + int(state.tip_z_offset_steps)
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

    def closed_thermal_door_no_go_box(self) -> dict[str, int]:
        """No-go box over LOC_TC when the thermal door is closed, plus chiller bypass Y."""
        p1 = self._target(1)   # LOC_OC
        p2 = self._target(2)   # LOC_TC
        p3 = self._target(3)   # LOC_RC
        inc = int(p2.inc_factor) or 1
        tc_a1_x = int(p2.base_coordinates["x"])
        tc_a1_y = int(p2.base_coordinates["y"])
        tc_h12_x = tc_a1_x + inc * OEM_X_INCREMENT * 11
        tc_h12_y = tc_a1_y + inc * OEM_Y_INCREMENT * 7
        x_min = min(tc_a1_x, tc_h12_x) - CLOSED_DOOR_MARGIN_X_STEPS
        x_max = max(tc_a1_x, tc_h12_x) + CLOSED_DOOR_MARGIN_X_STEPS
        y_min = 0
        y_max = max(tc_a1_y, tc_h12_y) + CLOSED_DOOR_MARGIN_Y_STEPS
        bypass_y = max(int(p1.base_coordinates["y"]), int(p3.base_coordinates["y"]), y_max + 2000)
        return {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "bypass_y": bypass_y,
        }

    def get_closed_door_waypoints(
        self, *, current_x: int, current_y: int, target_x: int, target_y: int
    ) -> list[dict[str, int]]:
        """Chiller-corridor waypoints when a direct move would cross the closed thermal door box."""
        x1, y1, x2, y2 = int(current_x), int(current_y), int(target_x), int(target_y)
        if x1 == x2 and y1 == y2:
            return []
        box = self.closed_thermal_door_no_go_box()
        bounds = {k: box[k] for k in ("x_min", "x_max", "y_min", "y_max")}
        if _point_in_box(x1, y1, **bounds) or _point_in_box(x2, y2, **bounds):
            return []
        if not _leg_rect_overlaps_box(x1, y1, x2, y2, **bounds):
            return []
        return self.closed_door_corridor_route(current_x=x1, current_y=y1, target_x=x2, target_y=y2)

    def closed_door_corridor_route(
        self, *, current_x: int, current_y: int, target_x: int, target_y: int
    ) -> list[dict[str, int]]:
        """Up to the chiller corridor (Y = bypass), across, then to the target.

        Legs below the corridor are single-axis, so for endpoints outside the box
        none of them can enter it.
        """
        x1, y1, x2, y2 = int(current_x), int(current_y), int(target_x), int(target_y)
        bypass_y = self.closed_thermal_door_no_go_box()["bypass_y"]
        points: list[dict[str, int]] = []
        if y1 < bypass_y:
            points.append({"x": x1, "y": bypass_y})
        if y2 < bypass_y:
            points.append({"x": x2, "y": bypass_y})
        points.append({"x": x2, "y": y2})
        out: list[dict[str, int]] = []
        prev = (x1, y1)
        for pt in points:
            if (pt["x"], pt["y"]) != prev:
                out.append(pt)
                prev = (pt["x"], pt["y"])
        return out

    def closed_door_route_check(self, *, start_x: int, start_y: int, steps: list[dict[str, Any]]) -> dict[str, Any]:
        """Check every planned XY leg against the closed-door box."""
        box = self.closed_thermal_door_no_go_box()
        bounds = {k: box[k] for k in ("x_min", "x_max", "y_min", "y_max")}
        legs = xy_travel_legs(start_x, start_y, steps)
        crossing = [list(leg) for leg in legs if _leg_rect_overlaps_box(*leg, **bounds)]
        return {"box": box, "legs": [list(leg) for leg in legs], "crossing_legs": crossing, "clear": not crossing}

    @staticmethod
    def _move_to_authority(state: OemMachineState) -> dict[str, Any]:
        """Carry the exact dynamic predicates required by the OEM moveTo branch."""
        return {
            "gripper_confirmed": bool(state.confirm_axis("gripper")),
            "tip_loaded": bool(state.tip_loaded),
            "plate_on_gantry": state.plate_on_gantry,
            "location19_y": state.location19_y,
        }

    def plan_script_move_to(self, *, current_loc: str | int | None, location_id: str | int, column: int = 0, row: int = 0, well: int | None = None, positionflag: int = 0, state: OemMachineState, run_in_parallel: bool = True) -> dict[str, Any]:
        loc_id = _loc_id(location_id); current_id = _loc_id(current_loc)
        if loc_id == 32:
            raise ValueError("Wrong destination!")
        well_overload: dict[str, Any] | None = None
        if well is not None:
            signed_well = int(well)
            row = abs(signed_well) // 12 * (-1 if signed_well < 0 else 1)
            column = signed_well - row * 12
            run_in_parallel = True
            well_overload = {
                "well": signed_well,
                "signed_divisor": 12,
                "column_remainder": column,
                "row_quotient": row,
                "run_in_parallel": True,
            }
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
            closed_mid: list[dict[str, int]] = []
            if state.needs_closed_door_avoidance():
                tx = 82450 if loc_id == WASTE_BIN_ID else x
                closed_mid = self.get_closed_door_waypoints(current_x=x2, current_y=y2, target_x=tx, target_y=y)
            mid = self.get_midpoints(current_x=x2, current_y=y2, target_x=x, target_y=y)
            if closed_mid and loc_id == WASTE_BIN_ID:
                branch = "tip_loaded_closed_door_avoidance_waste_bin"
                steps += [_step("moveXY", **pt, reason="closed_thermal_door_bypass") for pt in closed_mid]
                if str(state.device_type).upper() == "DBC":
                    steps.append(_step("moveZ", z=75000))
                else:
                    steps += [_step("moveZ", z=145000), _step("moveX", x=self.x_high_limit - 500),
                              _step("moveSteps", axis="x", delta=-1000), _step("moveZ", z=pseudo)]
            elif closed_mid:
                branch = "tip_loaded_closed_door_avoidance"
                steps += [_step("moveXY", **pt, reason="closed_thermal_door_bypass") for pt in closed_mid]
                steps.append(_step("moveZ", z=num))
            elif loc_id == WASTE_BIN_ID and not state.tip_dirty:
                # Clean tip (picked up, never in liquid or a well): straight to the bin.
                # Used or unknown tips keep the OEM legs around open plates below.
                branch = "clean_tip_direct_to_waste"
                steps.append(_step("moveXY", x=82450, y=y, reason="clean_tip_direct_to_waste"))
                if str(state.device_type).upper() == "DBC":
                    steps.append(_step("moveZ", z=75000))
                else:
                    steps += [_step("moveZ", z=145000), _step("moveX", x=self.x_high_limit - 500),
                              _step("moveSteps", axis="x", delta=-1000), _step("moveZ", z=pseudo)]
            elif current_id in TECAN_RACK_OR_STRIP4_IDS and (loc_id != 20 or loc_id != 18) and loc_id != WASTE_BIN_ID:
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
                    steps.append(_step("parallel", steps=[_step("moveX", x=x), _step("moveY", y=y)], parallel_semantics="source_task_wait_all_move_x_move_y", join="Task.WaitAll", source_lines="ClassControlInterface.cs:3916-3928"))
                    if num > pseudo:
                        steps.append(_step("moveZ", z=num, source_lines="ClassControlInterface.cs:3929-3932"))
                else:
                    steps += [_step("moveXY", x=x, y=y, source_lines="ClassControlInterface.cs:3934-3937"), _step("moveZ", z=num)]
            elif current_id in TECAN_RACK_IDS and loc_id != WASTE_BIN_ID:
                branch = "tip_loaded_from_tecan_rack_to_non_waste"
                if loc_id in TECAN_DIRECT_DESTINATION_IDS:
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

        closed_door_route: dict[str, Any] | None = None
        if state.tip_loaded and state.needs_closed_door_avoidance():
            box = self.closed_thermal_door_no_go_box()
            bounds = {k: box[k] for k in ("x_min", "x_max", "y_min", "y_max")}
            travel_x = 82450 if loc_id == WASTE_BIN_ID else x
            endpoint_in_box = _point_in_box(x2, y2, **bounds) or _point_in_box(travel_x, y, **bounds)
            check = self.closed_door_route_check(start_x=x2, start_y=y2, steps=steps)
            closed_door_route = {"oem_branch": branch, "endpoint_in_box": endpoint_in_box,
                                 "oem_route_check": check, "rerouted": False}
            if not check["clear"] and not endpoint_in_box:
                xy_ops = ("moveXY", "moveTo", "moveX", "moveY", "parallel", "moveSteps")
                first_xy = next((i for i, st in enumerate(steps) if st["op"] in xy_ops), len(steps))
                rebuilt = list(steps[:first_xy])
                rebuilt += [_step("moveXY", **pt, reason="closed_thermal_door_bypass")
                            for pt in self.closed_door_corridor_route(
                                current_x=x2, current_y=y2, target_x=travel_x, target_y=y)]
                if loc_id == WASTE_BIN_ID:
                    if str(state.device_type).upper() == "DBC":
                        rebuilt.append(_step("moveZ", z=75000))
                    else:
                        rebuilt += [_step("moveZ", z=145000), _step("moveX", x=self.x_high_limit - 500),
                                    _step("moveSteps", axis="x", delta=-1000), _step("moveZ", z=pseudo)]
                    branch = "tip_loaded_closed_door_avoidance_waste_bin"
                else:
                    rebuilt.append(_step("moveZ", z=num))
                    branch = "tip_loaded_closed_door_avoidance"
                steps = rebuilt
                closed_door_route.update(
                    rerouted=True,
                    final_route_check=self.closed_door_route_check(start_x=x2, start_y=y2, steps=steps))

        plan = {
            "ok": True,
            "schema_version": "bioxp.oem_scriptmove_path_plan.v1",
            "semantic_action": "scriptmoveTo",
            "branch": branch,
            "source_formula": SOURCE_LINES,
            "midpoint_source_formula": MIDPOINT_LINES,
            "source_method_token": SCRIPTMOVE_TO_METHOD_TOKEN,
            "source_il_sha256": SCRIPTMOVE_TO_IL_SHA256,
            "current_location_id": _loc_name(current_loc),
            "target_location_id": _loc_name(location_id),
            "column": int(column),
            "row": int(row),
            "well_overload": well_overload,
            "positionflag": int(positionflag),
            "target": target.to_payload(),
            "target_coordinates": target_xyz,
            "state": state.to_payload(),
            "steps": steps,
            "closed_door_route": closed_door_route,
            "step_count": len(steps),
            "source_hazards": (
                ["confirmed_oem_cover_storage_tautology"]
                if state.tip_loaded and loc_id in {18, 20}
                else []
            ),
            "opened_usb": False,
            "physical_motion": False,
            "motion_commanded": False,
            "controller_command_planned": False,
        }
        if branch != "same_xy_move_z":
            plan["show_gantry"] = {
                "x_mm": x / XY_MM_TO_STEPS,
                "y_mm": y / XY_MM_TO_STEPS,
                "source_lines": "ClassControlInterface.cs:4013",
            }
        return plan
