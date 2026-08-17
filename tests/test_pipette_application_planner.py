from __future__ import annotations

import pytest

from src.bioxp.pipette.application import (
    PipetteApplicationPlanner,
    PipettePhysicalMutationBlocked,
)
from src.bioxp.pipette.receipts import PipetteReceiptStore


def _bound_dependencies():
    return {
        name: {
            "bound": True,
            "authority": f"test.{name}",
            "generation": 7,
            "state": {"ready": True, "identity": f"{name}-identity"},
            "blockers": [],
        }
        for name in ("deck", "gantry", "z", "pressure", "pipette", "machine_state")
    }


def test_load_tip_plan_preserves_oem_control_lib_sequence_without_motion():
    planner = PipetteApplicationPlanner(dependency_resolver=_bound_dependencies)

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
    assert plan["dependencies_satisfied"] is True
    assert all(step.get("owner") for step in plan["steps"])
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
    planner = PipetteApplicationPlanner(dependency_resolver=_bound_dependencies)

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
    planner = PipetteApplicationPlanner(dependency_resolver=_bound_dependencies)

    with pytest.raises(PipettePhysicalMutationBlocked):
        planner.execute("move_to_waste", {})


def test_application_plan_fails_closed_when_required_provider_is_unbound():
    dependencies = _bound_dependencies()
    dependencies["gantry"] = {"bound": False, "authority": None, "error": "provider_unavailable"}
    planner = PipetteApplicationPlanner(dependency_resolver=lambda: dependencies)

    plan = planner.plan_move_to_waste()

    assert plan["ok"] is False
    assert plan["dependencies_satisfied"] is False
    assert plan["blocker"] == "application_dependencies_unbound"
    assert plan["missing_dependencies"] == ["gantry"]


def test_application_plan_fails_closed_without_dependency_generation_and_state():
    dependencies = _bound_dependencies()
    dependencies["gantry"] = {"bound": True, "authority": "test.gantry", "blockers": []}
    planner = PipetteApplicationPlanner(dependency_resolver=lambda: dependencies)

    plan = planner.plan_move_to_waste()
    status = planner.status()

    assert plan["ok"] is False
    assert plan["dependencies_satisfied"] is False
    assert any("gantry" in blocker for blocker in plan["dependency_blockers"])
    assert status["dependencies_satisfied"] is False
    assert any("gantry" in blocker for blocker in status["dependency_blockers"])


def test_receipt_store_uses_durable_oem_root_and_never_infers_completion(monkeypatch, tmp_path):
    root = tmp_path / "oem-runtime"
    monkeypatch.delenv("BIOXP_PIPETTE_RECEIPT_ROOT", raising=False)
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(root))

    store = PipetteReceiptStore()

    assert store.root == root / "pipette"
    assert store._truth({"ok": True, "outcome": "completion"})["completion_verified"] is False


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
    assert isinstance(plan.json()["receipt_id"], str)
    assert plan.json()["receipt_truth"]["physical_effect_verified"] is False
