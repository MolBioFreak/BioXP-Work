from __future__ import annotations

import pytest

from src.bioxp.pipette.application import (
    PipetteApplicationPlanner,
    PipettePhysicalMutationBlocked,
)


def test_load_tip_plan_preserves_oem_control_lib_sequence_without_motion():
    planner = PipetteApplicationPlanner()

    plan = planner.plan_load_tip(
        tip_tray="tray-1",
        tip_well="A1",
        tip_type=200,
        tip_location=2,
        home_z_after=True,
    )

    assert plan["ok"] is True
    assert plan["execution_admitted"] is False
    assert plan["motion_commanded"] is False
    assert [step["action"] for step in plan["steps"]] == [
        "resolve_tip_tray_well",
        "move_gantry_to_tip",
        "update_machine_location",
        "lower_pipette_z",
        "apply_overpressure",
        "query_tip_status",
        "home_z",
        "clear_tip_dirty",
    ]
    assert plan["requested_inputs"]["tip_location"] == 2
    assert plan["required_completion_evidence"] == [
        "gantry_at_tip_location",
        "z_at_load_depth",
        "overpressure_controller_ack",
        "hardware_tip_loaded_readback",
        "z_home_readback",
        "machine_state_reconciled",
    ]


def test_waste_fluid_and_plunger_plans_keep_oem_constants_and_dependencies():
    planner = PipetteApplicationPlanner()

    waste = planner.plan_move_to_waste()
    fluid = planner.plan_detect_fluid(fluid_class="MS")
    up = planner.plan_plunger(direction="up")
    down = planner.plan_plunger(direction="down")

    assert waste["constants"]["waste_location_id"] == 6
    assert fluid["constants"]["supported_offset_classes"] == ["TC", "MS", "OC", "RC", "STRIP"]
    assert fluid["requested_inputs"]["fluid_class"] == "MS"
    assert [step["action"] for step in fluid["steps"]] == [
        "resolve_fluid_target",
        "verify_plate_strip_state",
        "move_gantry_to_fluid_target",
        "lower_z_to_detection_start",
        "enable_pressure_logging",
        "send_fluid_detection",
        "wait_for_correlated_fluid_completion",
        "reconcile_detected_fluid_height",
        "park_z_and_gantry",
    ]
    assert up["constants"]["z_current"] == 31
    assert down["constants"]["z_current"] == 31


def test_application_planner_rejects_execution_in_no_motion_tranche():
    planner = PipetteApplicationPlanner()

    with pytest.raises(PipettePhysicalMutationBlocked):
        planner.execute("move_to_waste", {})


def test_application_plan_api_exposes_typed_no_motion_contract():
    from fastapi.testclient import TestClient

    from src.bioxp.api import app

    client = TestClient(app)
    status = client.get("/liquid/application/status")
    plan = client.post(
        "/liquid/application/plan",
        json={"operation": "detect_fluid", "fluid_class": "RC"},
    )

    assert status.status_code == 200
    assert status.json()["execution_admitted"] is False
    assert plan.status_code == 200
    assert plan.json()["requested_inputs"] == {"fluid_class": "RC"}
    assert plan.json()["motion_commanded"] is False
