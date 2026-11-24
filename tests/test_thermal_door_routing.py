"""Closed thermal door avoidance, 10 uL exemption, and default-open workflow policy (offline)."""
from bioxp.oem_compat.machine_state import OemMachineState
from bioxp.oem_compat.pathing import (
    OemPathPlanner,
    _segment_intersects_box,
    workflow_thermal_door_policy,
)


class _Row:
    def __init__(self, location_id: str, x: int, y: int, *, z_high: int = 50000, z_low: int = 80000, inc_factor: int = 1):
        self.location_id = location_id
        self.base_coordinates = {"x": x, "y": y}
        self.z_high = z_high
        self.z_low = z_low
        self.inc_factor = inc_factor

    def to_payload(self):
        return {"location_id": self.location_id, "base_coordinates": dict(self.base_coordinates)}


class _CalibratedTable:
    _ROWS = {
        "LOC_MS": _Row("LOC_MS", 26213, 9241),
        "LOC_OC": _Row("LOC_OC", 26213, 42413),
        "LOC_TC": _Row("LOC_TC", 67606, 9241),
        "LOC_RC": _Row("LOC_RC", 67677, 45256),
        "WASTE_BIN": _Row("WASTE_BIN", 82450, 93000, inc_factor=0),
        "TECANRACK1": _Row("TECANRACK1", 60571, 71755),
        "LOC_STRIP1": _Row("LOC_STRIP1", 75022, 83403),
        "LOC_STRIP4": _Row("LOC_STRIP4", 68625, 83403),
        "LOC_TROUGH": _Row("LOC_TROUGH", 85555, 63278, inc_factor=0),
        "LOC_OC_COVER_STORAGE": _Row("LOC_OC_COVER_STORAGE", 90000, 10000, inc_factor=0),
        "LOC_PARK": _Row("LOC_PARK", 1506, 71, inc_factor=0),
    }

    def resolve(self, location_id: str):
        return self._ROWS[location_id]


def _planner():
    return OemPathPlanner(_CalibratedTable())


def _assert_xy_legs_clear_box(start_xy, steps, box):
    bounds = {k: box[k] for k in ("x_min", "x_max", "y_min", "y_max")}
    cx, cy = start_xy
    for step in steps:
        if step["op"] == "moveXY":
            nx, ny = step["x"], step["y"]
            assert not _segment_intersects_box(cx, cy, nx, ny, **bounds), (
                f"leg ({cx},{cy})->({nx},{ny}) intersects closed-door box {bounds}"
            )
            cx, cy = nx, ny


def test_closed_door_no_go_box_matches_calibrated_deck_map():
    box = _planner().closed_thermal_door_no_go_box()
    assert box["x_min"] == 39178
    assert box["x_max"] == 72582
    assert box["y_min"] == 0
    assert box["y_max"] == 28667
    assert box["bypass_y"] == 45256


def test_50ul_and_200ul_and_unknown_tips_route_around_closed_door():
    planner = _planner()
    box = planner.closed_thermal_door_no_go_box()
    for tip_type in (50, 200, 201, None):
        state = OemMachineState.from_query(
            current_location_id="0", current_x=26213, current_y=9241, current_z=500,
            tip_loaded=True, tip_dirty=True, tip_type=tip_type, thermal_door_open=False,
            pseudo_z_home=500,
        )
        plan = planner.plan_script_move_to(current_loc=0, location_id=16, positionflag=1, state=state)
        assert plan["branch"] == "tip_loaded_closed_door_avoidance"
        xy_steps = [(s["x"], s["y"]) for s in plan["steps"] if s["op"] == "moveXY"]
        assert xy_steps == [(26213, 45256), (85555, 63278)]
        _assert_xy_legs_clear_box((26213, 9241), plan["steps"], box)


def test_return_from_trough_to_magsep_routes_via_chiller_corridor():
    planner = _planner()
    box = planner.closed_thermal_door_no_go_box()
    state = OemMachineState.from_query(
        current_location_id="16", current_x=85555, current_y=63278, current_z=500,
        tip_loaded=True, tip_dirty=True, tip_type=50, thermal_door_open=False,
        pseudo_z_home=500,
    )
    plan = planner.plan_script_move_to(current_loc=16, location_id=0, positionflag=1, state=state)
    assert plan["branch"] == "tip_loaded_closed_door_avoidance"
    xy_steps = [(s["x"], s["y"]) for s in plan["steps"] if s["op"] == "moveXY"]
    assert xy_steps == [(26213, 45256), (26213, 9241)]
    _assert_xy_legs_clear_box((85555, 63278), plan["steps"], box)


def test_both_north_endpoints_use_three_leg_chiller_bypass():
    planner = _planner()
    box = planner.closed_thermal_door_no_go_box()
    state = OemMachineState.from_query(
        current_location_id="0", current_x=26213, current_y=9241, current_z=500,
        tip_loaded=True, tip_dirty=True, tip_type=200, thermal_door_open=False,
        pseudo_z_home=500,
    )
    plan = planner.plan_script_move_to(current_loc=0, location_id=18, positionflag=0, state=state)
    assert plan["branch"] == "tip_loaded_closed_door_avoidance"
    xy_steps = [(s["x"], s["y"]) for s in plan["steps"] if s["op"] == "moveXY"]
    assert xy_steps == [(26213, 45256), (90000, 45256), (90000, 10000)]
    _assert_xy_legs_clear_box((26213, 9241), plan["steps"], box)


def test_10ul_tip_is_exempt_from_closed_door_avoidance():
    planner = _planner()
    state = OemMachineState.from_query(
        current_location_id="0", current_x=26213, current_y=9241, current_z=500,
        tip_loaded=True, tip_dirty=True, tip_type=10, thermal_door_open=False,
        pseudo_z_home=500,
    )
    assert state.needs_closed_door_avoidance() is False
    plan = planner.plan_script_move_to(current_loc=0, location_id=16, positionflag=1, state=state)
    assert plan["branch"] != "tip_loaded_closed_door_avoidance"


def test_unknown_door_state_is_avoided_like_closed():
    planner = _planner()
    state = OemMachineState.from_query(
        current_location_id="0", current_x=26213, current_y=9241, current_z=500,
        tip_loaded=True, tip_dirty=True, tip_type=50, thermal_door_open=None,
        pseudo_z_home=500,
    )
    assert state.needs_closed_door_avoidance() is True


def test_door_open_or_no_tip_keeps_oem_behavior():
    planner = _planner()
    for loaded, door in ((False, False), (True, True)):
        state = OemMachineState.from_query(
            current_location_id="0", current_x=26213, current_y=9241, current_z=500,
            tip_loaded=loaded, tip_dirty=True, tip_type=50, thermal_door_open=door,
            pseudo_z_home=500,
        )
        assert state.needs_closed_door_avoidance() is False
        plan = planner.plan_script_move_to(current_loc=0, location_id=16, positionflag=1, state=state)
        assert plan["branch"] != "tip_loaded_closed_door_avoidance"


def test_waste_bin_route_avoids_closed_door_and_keeps_oem_eject():
    planner = _planner()
    box = planner.closed_thermal_door_no_go_box()
    state = OemMachineState.from_query(
        current_location_id="0", current_x=26213, current_y=9241, current_z=500,
        tip_loaded=True, tip_dirty=False, tip_type=50, thermal_door_open=False,
        pseudo_z_home=500, device_type="BIOXP",
    )
    plan = planner.plan_script_move_to(current_loc=0, location_id=6, positionflag=0, state=state)
    assert plan["branch"] == "tip_loaded_closed_door_avoidance_waste_bin"
    xy_steps = [(s["x"], s["y"]) for s in plan["steps"] if s["op"] == "moveXY"]
    assert xy_steps == [(26213, 45256), (82450, 93000)]
    _assert_xy_legs_clear_box((26213, 9241), plan["steps"], box)
    assert any(s["op"] == "moveZ" and s["z"] == 145000 for s in plan["steps"])


def test_non_crossing_move_and_target_inside_tc_do_not_detour():
    planner = _planner()
    state_chiller = OemMachineState.from_query(
        current_location_id="1", current_x=26213, current_y=42413, current_z=500,
        tip_loaded=True, tip_dirty=True, tip_type=50, thermal_door_open=False,
        pseudo_z_home=500,
    )
    plan_clear = planner.plan_script_move_to(current_loc=1, location_id=3, positionflag=1, state=state_chiller)
    assert plan_clear["branch"] != "tip_loaded_closed_door_avoidance"
    plan_tc = planner.plan_script_move_to(current_loc=1, location_id=2, positionflag=1, state=state_chiller)
    assert plan_tc["branch"] != "tip_loaded_closed_door_avoidance"


def test_workflow_thermal_door_policy_defaults_and_exemptions():
    doc_50 = {
        "metadata": {},
        "stages": [{"actions": [{"kind": "pipette_manual_physical", "params": {"operation": "load_tip", "tip_type": 50}}]}],
    }
    assert workflow_thermal_door_policy(doc_50)["should_open"] is True

    doc_10_only = {
        "metadata": {},
        "stages": [{"actions": [
            {"kind": "pipette_manual_physical", "params": {"operation": "load_tip", "tip_type": 10}},
            {"kind": "pipette_aspirate", "params": {"volume_ul": 5}},
        ]}],
    }
    res_10 = workflow_thermal_door_policy(doc_10_only)
    assert res_10["should_open"] is False and res_10["reason"] == "exempt_10ul_only"

    doc_mixed = {
        "metadata": {},
        "stages": [{"actions": [
            {"kind": "pipette_manual_physical", "params": {"operation": "load_tip", "tip_type": 10}},
            {"kind": "pipette_manual_physical", "params": {"operation": "load_tip", "tip_type": 200}},
        ]}],
    }
    assert workflow_thermal_door_policy(doc_mixed)["should_open"] is True

    doc_keep_closed = {
        "metadata": {"bms_method": {"method": {"keep_thermal_door_closed": True}}},
        "stages": doc_50["stages"],
    }
    res_closed = workflow_thermal_door_policy(doc_keep_closed)
    assert res_closed["should_open"] is False and res_closed["reason"] == "explicit_keep_closed"

    doc_no_pipette = {"metadata": {}, "stages": [{"actions": [{"kind": "thermal_cycle", "params": {}}]}]}
    res_none = workflow_thermal_door_policy(doc_no_pipette)
    assert res_none["should_open"] is False and res_none["reason"] == "no_pipetting_actions"
