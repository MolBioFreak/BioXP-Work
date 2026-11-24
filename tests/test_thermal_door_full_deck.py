"""Closed thermal door: every location pair on the real serial-206 calibration (offline)."""
import itertools

import pytest

from bioxp.oem_compat.machine_state import OemMachineState
from bioxp.oem_compat.pathing import OemPathPlanner, _leg_rect_overlaps_box, _loc_name, _point_in_box, xy_travel_legs
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


@pytest.fixture()
def planner(monkeypatch):
    bind_serial206_oem_snapshot(monkeypatch)
    return OemPathPlanner(load_bound_oem_position_table())


def _locations(planner):
    out = {}
    for lid in range(40):
        try:
            row = planner._target(lid)
        except Exception:
            continue
        out[lid] = (int(row.base_coordinates["x"]), int(row.base_coordinates["y"]))
    return out


def _state(loc, xy, *, tip_type, door_open):
    return OemMachineState.from_query(
        current_location_id=str(loc), current_x=xy[0], current_y=xy[1], current_z=500,
        tip_loaded=True, tip_dirty=True, tip_type=tip_type, thermal_door_open=door_open,
        pseudo_z_home=500,
    )


def _pairs(planner):
    locs = _locations(planner)
    box = planner.closed_thermal_door_no_go_box()
    bounds = {k: box[k] for k in ("x_min", "x_max", "y_min", "y_max")}
    outside = {k: v for k, v in locs.items() if k not in (32,) and not _point_in_box(*v, **bounds)}
    return locs, bounds, list(itertools.permutations(outside, 2))


@pytest.mark.parametrize("tip_type", [50, 200, None])
def test_no_leg_enters_closed_door_box_for_long_tips(planner, tip_type):
    locs, bounds, pairs = _pairs(planner)
    crossings = []
    for a, b in pairs:
        plan = planner.plan_script_move_to(current_loc=a, location_id=b, positionflag=0,
                                           state=_state(a, locs[a], tip_type=tip_type, door_open=False))
        for leg in xy_travel_legs(*locs[a], plan["steps"]):
            if _leg_rect_overlaps_box(*leg, **bounds):
                crossings.append((_loc_name(a), _loc_name(b), plan["branch"], leg))
    assert crossings == []
    assert len(pairs) > 500


def test_ten_ul_and_open_door_keep_exact_oem_plans(planner):
    locs, _bounds, pairs = _pairs(planner)
    for a, b in pairs:
        for tip_type, door in ((10, False), (50, True), (50, None)):
            plan = planner.plan_script_move_to(current_loc=a, location_id=b, positionflag=0,
                                               state=_state(a, locs[a], tip_type=tip_type, door_open=door))
            assert plan["closed_door_route"] is None
            assert "closed_door" not in plan["branch"]


def test_strip4_to_magsep_now_goes_over_chillers(planner):
    locs = _locations(planner)
    plan = planner.plan_script_move_to(current_loc=14, location_id=0, positionflag=0,
                                       state=_state(14, locs[14], tip_type=50, door_open=False))
    assert plan["branch"] == "tip_loaded_closed_door_avoidance"
    assert plan["closed_door_route"]["oem_route_clear"] is True
    bypass_y = planner.closed_thermal_door_no_go_box()["bypass_y"]
    xy = [(s["x"], s["y"]) for s in plan["steps"] if s["op"] == "moveXY"]
    assert xy == [(locs[0][0], bypass_y), locs[0]]


@pytest.mark.parametrize("tip_type", [10, 50, 200, None])
@pytest.mark.parametrize("dirty", [False, True])
def test_executed_plan_projection_keeps_every_step(planner, tip_type, dirty):
    """The plan is stored and executed through _json_safe; nothing may be cut."""
    from bioxp.oem_serial206_initialization import _json_safe
    locs, _bounds, pairs = _pairs(planner)
    cut = []
    for a, b in pairs:
        state = OemMachineState.from_query(
            current_location_id=str(a), current_x=locs[a][0], current_y=locs[a][1], current_z=500,
            tip_loaded=True, tip_dirty=dirty, tip_type=tip_type, thermal_door_open=False,
            pseudo_z_home=500, device_type="BIOXP",
        )
        plan = planner.plan_script_move_to(current_loc=a, location_id=b, positionflag=0, state=state)
        if _json_safe(plan)["steps"] != plan["steps"]:
            cut.append((_loc_name(a), _loc_name(b), plan["branch"]))
    assert cut == []
