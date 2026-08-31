from __future__ import annotations

import pytest

from src.bioxp.oem_compat.machine_state import OemDefaultParameters, OemMachineState
from src.bioxp.oem_compat import pathing
from src.bioxp.oem_compat.pathing import OemPathPlanner
from src.bioxp.oem_compat.position_table import OEM_Y_INCREMENT, PositionTable


def _planner() -> OemPathPlanner:
    rows = []
    for location_id, name in {
        0: "LOC_MS",
        1: "LOC_OC",
        2: "LOC_TC",
        3: "LOC_RC",
        6: "WASTE_BIN",
        7: "TECANRACK1",
        8: "TECANRACK2",
        9: "TECANRACK3",
        10: "TECANRACK4",
        11: "LOC_STRIP1",
        12: "LOC_STRIP2",
        13: "LOC_STRIP3",
        14: "LOC_STRIP4",
        15: "LOC_TIP_HOTEL",
        16: "LOC_TROUGH",
        18: "LOC_OC_COVER_STORAGE",
        19: "LOC_RC_COVER",
        20: "LOC_RC_COVER_STORAGE",
        32: "UNKNOWN",
    }.items():
        rows.append(
            {
                "name": name,
                "x": 1000 + location_id * 4000,
                "y": 1000 + location_id * 3000,
                "zLow": 80000 + location_id,
                "zDelta": 30000,
                "inc_factor": 1,
            }
        )
    return OemPathPlanner(PositionTable.from_rows(rows))


def _state(**changes: object) -> OemMachineState:
    values = {
        "current_x": -50000,
        "current_y": 120000,
        "current_z": 0,
        "default_parameters": OemDefaultParameters(65000),
    }
    values.update(changes)
    return OemMachineState(**values)


def test_scriptmove_destination_32_raises_exact_wrong_destination_text() -> None:
    with pytest.raises(ValueError, match=r"^Wrong destination!$"):
        _planner().plan_script_move_to(
            current_loc=0, location_id=32, state=_state()
        )


def test_scriptmove_well_overload_uses_signed_divide_modulo_by_12_and_parallel_true() -> None:
    plan = _planner().plan_script_move_to(
        current_loc=0,
        location_id=0,
        well=-13,
        state=_state(axis_confirmed={"gripper": True}),
        run_in_parallel=False,
    )

    assert (plan["column"], plan["row"]) == (-1, -1)
    assert plan["well_overload"] == {
        "well": -13,
        "signed_divisor": 12,
        "column_remainder": -1,
        "row_quotient": -1,
        "run_in_parallel": True,
    }
    assert plan["steps"][0]["run_in_parallel"] is True


@pytest.mark.parametrize("location_id", [7, 8, 9, 10, 6, 16])
def test_scriptmove_tip_offset_exemption_set_is_exact(location_id: int) -> None:
    planner = _planner()
    target = planner._target(location_id)
    plan = planner.plan_script_move_to(
        current_loc=0,
        location_id=location_id,
        row=4,
        state=_state(tip_location=3, axis_confirmed={"gripper": True}),
    )
    assert plan["target_coordinates"]["y"] == int(target.base_coordinates["y"]) + 4 * OEM_Y_INCREMENT


def test_scriptmove_non_exempt_destination_applies_effective_tip_offset() -> None:
    planner = _planner()
    target = planner._target(11)
    plan = planner.plan_script_move_to(
        current_loc=0,
        location_id=11,
        row=4,
        state=_state(tip_location=3, axis_confirmed={"gripper": True}),
    )
    assert plan["target_coordinates"]["y"] == int(target.base_coordinates["y"]) - 2 * OEM_Y_INCREMENT


@pytest.mark.parametrize(
    ("positionflag", "expected"),
    [(0, 65000), (1, 50000), (2, 80000), (-1, 80000), (999, 80000)],
)
def test_scriptmove_positionflag_zero_one_and_all_other_values_select_exact_z(
    positionflag: int, expected: int
) -> None:
    plan = _planner().plan_script_move_to(
        current_loc=0,
        location_id=0,
        positionflag=positionflag,
        state=_state(axis_confirmed={"gripper": True}),
    )
    assert plan["target_coordinates"]["z"] == expected


def test_scriptmove_same_xy_returns_after_move_z_without_show_gantry() -> None:
    plan = _planner().plan_script_move_to(
        current_loc=0,
        location_id=0,
        state=_state(current_x=1000, current_y=1000),
    )
    assert [step["op"] for step in plan["steps"]] == ["moveZ"]
    assert "show_gantry" not in plan


@pytest.mark.parametrize("destination", [18, 20])
def test_cover_storage_clean_path_uses_direct_xy_under_confirmed_tautology(
    destination: int,
) -> None:
    planner = _planner()
    plan = planner.plan_script_move_to(
        current_loc=0,
        location_id=destination,
        state=_state(
            current_x=1000,
            current_y=1000,
            current_z=0,
            tip_loaded=True,
            clean_path=True,
        ),
    )
    direct_xy = next(step for step in plan["steps"] if step["op"] == "moveXY")
    assert (direct_xy["x"], direct_xy["y"]) == (
        plan["target_coordinates"]["x"],
        plan["target_coordinates"]["y"],
    )
    assert plan["source_hazards"] == ["confirmed_oem_cover_storage_tautology"]


def test_reachable_tecan_destination_set_excludes_unreachable_waste_branch() -> None:
    assert pathing.TECAN_DIRECT_DESTINATION_IDS == frozenset({11, 12, 13, 14, 16})


def test_near_axis_parallel_plan_contains_one_xy_sibling_group_before_conditional_z() -> None:
    plan = _planner().plan_script_move_to(
        current_loc=0,
        location_id=0,
        positionflag=2,
        state=_state(current_x=900, current_y=120000, current_z=0, tip_loaded=True),
    )
    aggregate = plan["steps"][0]
    assert aggregate["op"] == "parallel"
    assert [child["op"] for child in aggregate["steps"]] == ["moveX", "moveY"]
    assert aggregate["parallel_semantics"] == "source_task_wait_all_move_x_move_y"
    assert aggregate["join"] == "Task.WaitAll"
    assert plan["steps"][1]["op"] == "moveZ"


def test_scriptmove_plan_is_bound_to_pinned_raw_il_authority() -> None:
    plan = _planner().plan_script_move_to(
        current_loc=0,
        location_id=0,
        state=_state(axis_confirmed={"gripper": True}),
    )
    assert plan["source_method_token"] == "0x06000120"
    assert plan["source_il_sha256"] == (
        "3e94ea68ac09edbbeb021907285e9e47206c07d118e5f3a9f5611cd03c491ce3"
    )


def test_scriptmove_non_early_branches_end_with_show_gantry_projection() -> None:
    plan = _planner().plan_script_move_to(
        current_loc=0,
        location_id=0,
        state=_state(axis_confirmed={"gripper": True}),
    )
    assert plan["show_gantry"] == {
        "x_mm": plan["target_coordinates"]["x"] / 236.94,
        "y_mm": plan["target_coordinates"]["y"] / 236.94,
        "source_lines": "ClassControlInterface.cs:4013",
    }
