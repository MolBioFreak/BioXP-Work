from __future__ import annotations

import pytest

from src.bioxp.oem_self_test import OemSelfTestReceiptError, evaluate_oem_self_test_receipt, self_test_contract


def passing_receipt():
    return {
        "tc": {
            "high_leg_s": 149.0,
            "low_leg_s": 149.0,
            "lid_leg_s": 149.0,
            "nest_a_c": 25.0,
            "nest_b_c": 26.9,
            "pedestal_c": 5.0,
            "lid_reached_target": True,
            "heater_off_acknowledged": True,
        },
        "rc": {
            "initial_c": 20.8,
            "target_c": 16,
            "completed_within_s": 179.0,
            "final_c": 17.9,
            "default_cool_rate_restored": True,
        },
        "oc": {
            "initial_c": 14.9,
            "target_c": None,
            "completed_within_s": 0.0,
            "final_c": 14.9,
            "default_cool_rate_restored": True,
        },
        "motion": {
            "xy_home_residual_steps": [100, -100],
            "z_home_residual_steps": 100,
            "gripper_home_residual_steps": -500,
            "gantry_park_verified": True,
            "door_open_verified": True,
        },
        "concurrency": {
            "branch_tasks": {
                "tc": {"task_id": "tc-task-1", "submitted_at_ms": 0, "started_at_ms": 10, "completed_at_ms": 800},
                "rc": {"task_id": "rc-task-1", "submitted_at_ms": 1, "started_at_ms": 11, "completed_at_ms": 700},
                "oc": {"task_id": "oc-task-1", "submitted_at_ms": 2, "started_at_ms": 12, "completed_at_ms": 900},
            },
            "motion_task_id": "motion-task-1",
            "motion_started_at_ms": 20,
            "motion_completed_at_ms": 600,
            "join_started_at_ms": 601,
            "join_completed_at_ms": 900,
            "thermal_wait_completed_within_ms": 299,
        },
        "provider_identity": {
            "provider_id": "self-test-provider-1",
            "provider_run_id": "self-test-run-1",
            "binding_generation": 3,
        },
        "launched_branch_results": {"tc": True, "rc": True, "oc": True},
        "final_chiller_pwm_reset_acknowledged": True,
        "inspection_log_only": False,
    }


def test_self_test_contract_locks_exact_oem_thresholds_and_cleanup():
    contract = self_test_contract()
    assert contract["tc"] == {
        "heat_delta_c": 5.0,
        "cool_delta_from_lid_target_c": -10.0,
        "temperature_leg_timeout_s": 150.0,
        "max_nest_delta_c": 2.0,
        "pedestal_range_c_inclusive": [5.0, 40.0],
        "lid_target_tolerance_c": 1.0,
        "finally": "turn_off_heater",
    }
    assert contract["rc_oc"]["cool_rate_self_test_c_per_s"] == -0.05
    assert contract["rc_oc"]["cool_rate_default_c_per_s"] == -0.025
    assert contract["rc_oc"]["skip_active_cooling_at_or_below_c"] == 15.0
    assert contract["rc_oc"]["active_target"] == "int(initial_c)-4"
    assert contract["motion"]["xy_z_home_max_abs_error_steps"] == 100
    assert contract["motion"]["gripper_home_max_abs_error_steps"] == 500
    assert contract["parallel_completion_timeout_ms"] == 100_000


def test_passing_self_test_receipt_requires_all_hardware_postconditions():
    result = evaluate_oem_self_test_receipt(passing_receipt())
    assert result["ok"] is True
    assert result["receipt_validation_pass"] is True
    assert result["oem_effective_pass"] is True
    assert result["production_admission_pass"] is False
    assert result["physical_effect_verified"] is False
    assert result["provider_live_bound"] is False
    assert result["failures"] == []


@pytest.mark.parametrize(
    ("mutate", "failure"),
    [
        (lambda r: r["tc"].update(nest_b_c=27.01), "tc_nest_delta_exceeded"),
        (lambda r: r["tc"].update(pedestal_c=4.99), "tc_pedestal_out_of_range"),
        (lambda r: r["tc"].update(high_leg_s=150.01), "tc_high_temperature_timeout"),
        (lambda r: r["rc"].update(final_c=18.0), "rc_target_not_reached"),
        (lambda r: r["oc"].update(default_cool_rate_restored=False), "oc_default_cool_rate_not_restored"),
        (lambda r: r["motion"].update(xy_home_residual_steps=[101, 0]), "x_home_residual_exceeded"),
        (lambda r: r["motion"].update(z_home_residual_steps=-101), "z_home_residual_exceeded"),
        (lambda r: r["motion"].update(gripper_home_residual_steps=501), "gripper_home_residual_exceeded"),
        (lambda r: r["concurrency"].update(thermal_wait_completed_within_ms=100_001), "parallel_completion_timeout"),
        (lambda r: r["concurrency"]["branch_tasks"]["rc"].update(started_at_ms=601), "rc_did_not_overlap_motion"),
    ],
)
def test_self_test_threshold_failures_are_exact(mutate, failure):
    receipt = passing_receipt()
    mutate(receipt)
    result = evaluate_oem_self_test_receipt(receipt)
    assert result["ok"] is False
    assert failure in result["failures"]
    assert result["physical_effect_verified"] is False


def test_inspection_log_only_preserves_oem_override_but_never_production_admission():
    receipt = passing_receipt()
    receipt["inspection_log_only"] = True
    receipt["tc"]["lid_reached_target"] = False
    result = evaluate_oem_self_test_receipt(receipt)
    assert result["oem_effective_pass"] is True
    assert result["production_admission_pass"] is False
    assert result["ok"] is False
    assert "tc_lid_target_not_reached" in result["failures"]


def test_false_launched_task_result_fails_even_when_derived_metrics_pass():
    receipt = passing_receipt()
    receipt["launched_branch_results"]["rc"] = False
    result = evaluate_oem_self_test_receipt(receipt)
    assert result["ok"] is False
    assert "rc_launched_result_false" in result["failures"]
    assert result["physical_effect_verified"] is False


def test_self_test_concurrency_requires_unique_tasks_and_source_ordered_join():
    duplicate = passing_receipt()
    duplicate["concurrency"]["branch_tasks"]["oc"]["task_id"] = "rc-task-1"
    with pytest.raises(OemSelfTestReceiptError, match="task identities"):
        evaluate_oem_self_test_receipt(duplicate)

    early_join = passing_receipt()
    early_join["concurrency"]["join_started_at_ms"] = 599
    result = evaluate_oem_self_test_receipt(early_join)
    assert result["receipt_validation_pass"] is False
    assert "join_started_before_motion_completed" in result["failures"]
    assert result["production_admission_pass"] is False


def test_self_test_receipt_rejects_truthy_and_missing_values():
    receipt = passing_receipt()
    receipt["tc"]["lid_reached_target"] = 1
    with pytest.raises(OemSelfTestReceiptError, match="exact bool"):
        evaluate_oem_self_test_receipt(receipt)

    receipt = passing_receipt()
    del receipt["motion"]["door_open_verified"]
    with pytest.raises(OemSelfTestReceiptError, match="missing"):
        evaluate_oem_self_test_receipt(receipt)
