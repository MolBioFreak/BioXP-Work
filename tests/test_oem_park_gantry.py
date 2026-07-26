from __future__ import annotations

import pytest

from src.bioxp.oem_park_gantry import OemParkReceiptError, evaluate_park_gantry_receipt


def base_receipt():
    return {
        "current_location_before": 6,
        "current_well_before": 0,
        "tip_present_before": False,
        "tip_cleanup": None,
        "rehome": False,
        "preliminary_route_without_z_verified": False,
        "xy_home_residual_steps": None,
        "final_route_profile": 2,
        "final_route_verified": True,
        "location_28_persisted": True,
    }


def test_already_parked_is_exact_noop():
    value = base_receipt()
    value.update(
        current_location_before=28,
        final_route_profile=None,
        final_route_verified=False,
        location_28_persisted=True,
    )
    result = evaluate_park_gantry_receipt(value)
    assert result["ok"] is True
    assert result["receipt_validation_pass"] is True
    assert result["outcome"] == "already_parked"
    assert result["production_admission_pass"] is False
    assert result["physical_effect_verified"] is False


def test_normal_park_requires_profile_two_and_persisted_location():
    result = evaluate_park_gantry_receipt(base_receipt())
    assert result["ok"] is True
    assert result["receipt_validation_pass"] is True
    assert result["outcome"] == "park_verified"
    assert result["production_admission_pass"] is False
    assert result["physical_effect_verified"] is False


def test_rehome_branch_requires_preliminary_route_and_exact_residual_limits():
    value = base_receipt()
    value.update(
        rehome=True,
        preliminary_route_without_z_verified=True,
        xy_home_residual_steps=[-100, 100],
    )
    assert evaluate_park_gantry_receipt(value)["ok"] is True
    value["xy_home_residual_steps"] = [101, 0]
    result = evaluate_park_gantry_receipt(value)
    assert result["ok"] is False
    assert "x_home_residual_exceeded" in result["failures"]


def test_tip_branch_requires_verified_empty_and_clearance_moves():
    value = base_receipt()
    value.update(
        tip_present_before=True,
        tip_cleanup={
            "waste_route_verified": True,
            "ejection_postcondition_empty": True,
            "z_80000_verified": True,
            "x_79000_verified": True,
            "machine_tip_state_cleared": True,
        },
    )
    assert evaluate_park_gantry_receipt(value)["ok"] is True
    value["tip_cleanup"]["ejection_postcondition_empty"] = False
    result = evaluate_park_gantry_receipt(value)
    assert result["ok"] is False
    assert "tip_ejection_not_verified_empty" in result["failures"]


def test_park_rejects_truthy_or_unexpected_receipts():
    value = base_receipt()
    value["final_route_verified"] = 1
    with pytest.raises(OemParkReceiptError, match="exact bool"):
        evaluate_park_gantry_receipt(value)
    value = base_receipt()
    value["axis"] = "x"
    with pytest.raises(OemParkReceiptError, match="unexpected"):
        evaluate_park_gantry_receipt(value)
