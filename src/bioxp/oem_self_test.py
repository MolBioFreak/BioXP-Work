"""Strict semantic validation of source-shaped BioXP OEM self-test receipts.

A supplied receipt can validate the closed schema, OEM thresholds, ordering, and
postconditions.  It cannot bind a live provider or establish a physical effect.
"""
from __future__ import annotations

import math
from typing import Any


class OemSelfTestReceiptError(ValueError):
    pass


SOURCE_ANCHORS = {
    "selftest": "ControlLib.selftest:10688-10785",
    "tc": "ControlLib.TCSelfTest:10788-10865",
    "rc": "ControlLib.RCSelfTest:10867-10932",
    "oc": "ControlLib.OCSelfTest:10934-10999",
    "wait": "ControlLib.waitforcompletion:11001-11020",
}


def self_test_contract() -> dict[str, Any]:
    return {
        "tc": {
            "heat_delta_c": 5.0,
            "cool_delta_from_lid_target_c": -10.0,
            "temperature_leg_timeout_s": 150.0,
            "max_nest_delta_c": 2.0,
            "pedestal_range_c_inclusive": [5.0, 40.0],
            "lid_target_tolerance_c": 1.0,
            "finally": "turn_off_heater",
        },
        "rc_oc": {
            "cool_rate_self_test_c_per_s": -0.05,
            "cool_rate_default_c_per_s": -0.025,
            "skip_active_cooling_at_or_below_c": 15.0,
            "active_target": "int(initial_c)-4",
            "active_cooling_timeout_s": 180.0,
            "success": "int(final_c)-target_c<=1",
            "finally": "restore_default_cool_rate",
        },
        "motion": {
            "close_thermal_door_first": True,
            "home_order": ["z", "x", "y"],
            "xy_max_moves_concurrent": True,
            "xy_z_home_max_abs_error_steps": 100,
            "gripper_home_max_abs_error_steps": 500,
            "park_then_open_door": True,
        },
        "parallel_branches": ["tc", "rc", "oc", "motion"],
        "parallel_completion_timeout_ms": 100_000,
        "final_chiller_pwm": 0,
        "source_anchors": dict(SOURCE_ANCHORS),
    }


def _mapping(value: Any, name: str, keys: set[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise OemSelfTestReceiptError(f"{name} must be an object")
    missing = sorted(keys - set(value))
    extra = sorted(set(value) - keys)
    if missing:
        raise OemSelfTestReceiptError(f"{name} missing fields: {missing}")
    if extra:
        raise OemSelfTestReceiptError(f"{name} has unexpected fields: {extra}")
    return value


def _bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise OemSelfTestReceiptError(f"{name} must be an exact bool")
    return value


def _number(value: Any, name: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise OemSelfTestReceiptError(f"{name} must be a finite number")
    return float(value)


def _integer(value: Any, name: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise OemSelfTestReceiptError(f"{name} must be an exact integer >= {minimum}")
    return value


def _integer_or_none(value: Any, name: str) -> int | None:
    if value is None:
        return None
    if type(value) is not int:
        raise OemSelfTestReceiptError(f"{name} must be an exact integer or null")
    return value


def _identifier(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise OemSelfTestReceiptError(f"{name} must be a bounded nonblank identifier")
    return value


def evaluate_oem_self_test_receipt(receipt: dict[str, Any]) -> dict[str, Any]:
    top = _mapping(
        receipt,
        "receipt",
        {
            "tc", "rc", "oc", "motion", "concurrency", "provider_identity",
            "launched_branch_results", "final_chiller_pwm_reset_acknowledged",
            "inspection_log_only",
        },
    )
    tc = _mapping(
        top["tc"],
        "tc",
        {"high_leg_s", "low_leg_s", "lid_leg_s", "nest_a_c", "nest_b_c", "pedestal_c", "lid_reached_target", "heater_off_acknowledged"},
    )
    chiller_keys = {"initial_c", "target_c", "completed_within_s", "final_c", "default_cool_rate_restored"}
    rc = _mapping(top["rc"], "rc", chiller_keys)
    oc = _mapping(top["oc"], "oc", chiller_keys)
    motion = _mapping(
        top["motion"],
        "motion",
        {"xy_home_residual_steps", "z_home_residual_steps", "gripper_home_residual_steps", "gantry_park_verified", "door_open_verified"},
    )
    concurrency = _mapping(
        top["concurrency"],
        "concurrency",
        {
            "branch_tasks", "motion_task_id", "motion_started_at_ms",
            "motion_completed_at_ms", "join_started_at_ms", "join_completed_at_ms",
            "thermal_wait_completed_within_ms",
        },
    )
    branch_tasks = _mapping(concurrency["branch_tasks"], "concurrency.branch_tasks", {"tc", "rc", "oc"})
    task_rows: dict[str, dict[str, Any]] = {}
    task_ids: list[str] = []
    for branch in ("tc", "rc", "oc"):
        task = _mapping(
            branch_tasks[branch],
            f"concurrency.branch_tasks.{branch}",
            {"task_id", "submitted_at_ms", "started_at_ms", "completed_at_ms"},
        )
        task_ids.append(_identifier(task["task_id"], f"concurrency.branch_tasks.{branch}.task_id"))
        for field in ("submitted_at_ms", "started_at_ms", "completed_at_ms"):
            _integer(task[field], f"concurrency.branch_tasks.{branch}.{field}")
        task_rows[branch] = task
    motion_task_id = _identifier(concurrency["motion_task_id"], "concurrency.motion_task_id")
    if len(set([*task_ids, motion_task_id])) != 4:
        raise OemSelfTestReceiptError("self-test task identities must be unique")
    for field in (
        "motion_started_at_ms", "motion_completed_at_ms", "join_started_at_ms",
        "join_completed_at_ms", "thermal_wait_completed_within_ms",
    ):
        _integer(concurrency[field], f"concurrency.{field}")

    provider = _mapping(
        top["provider_identity"],
        "provider_identity",
        {"provider_id", "provider_run_id", "binding_generation"},
    )
    _identifier(provider["provider_id"], "provider_identity.provider_id")
    _identifier(provider["provider_run_id"], "provider_identity.provider_run_id")
    _integer(provider["binding_generation"], "provider_identity.binding_generation")

    launched_results = _mapping(top["launched_branch_results"], "launched_branch_results", {"tc", "rc", "oc"})
    inspection_log_only = _bool(top["inspection_log_only"], "inspection_log_only")
    final_pwm = _bool(top["final_chiller_pwm_reset_acknowledged"], "final_chiller_pwm_reset_acknowledged")

    failures: list[str] = []
    thermal_failures: list[str] = []
    for branch in ("tc", "rc", "oc"):
        if not _bool(launched_results[branch], f"launched_branch_results.{branch}"):
            failures.append(f"{branch}_launched_result_false")

    for field, label in (
        ("high_leg_s", "tc_high_temperature_timeout"),
        ("low_leg_s", "tc_low_temperature_timeout"),
        ("lid_leg_s", "tc_lid_temperature_timeout"),
    ):
        if _number(tc[field], f"tc.{field}") > 150.0:
            thermal_failures.append(label)
    if abs(_number(tc["nest_a_c"], "tc.nest_a_c") - _number(tc["nest_b_c"], "tc.nest_b_c")) > 2.0:
        thermal_failures.append("tc_nest_delta_exceeded")
    pedestal = _number(tc["pedestal_c"], "tc.pedestal_c")
    if pedestal < 5.0 or pedestal > 40.0:
        thermal_failures.append("tc_pedestal_out_of_range")
    if not _bool(tc["lid_reached_target"], "tc.lid_reached_target"):
        thermal_failures.append("tc_lid_target_not_reached")
    if not _bool(tc["heater_off_acknowledged"], "tc.heater_off_acknowledged"):
        thermal_failures.append("tc_heater_not_turned_off")

    for name, row in (("rc", rc), ("oc", oc)):
        initial = _number(row["initial_c"], f"{name}.initial_c")
        target = _integer_or_none(row["target_c"], f"{name}.target_c")
        elapsed = _number(row["completed_within_s"], f"{name}.completed_within_s")
        final = _number(row["final_c"], f"{name}.final_c")
        if initial <= 15.0:
            if target is not None:
                thermal_failures.append(f"{name}_unexpected_active_target")
        else:
            expected_target = int(initial) - 4
            if target != expected_target:
                thermal_failures.append(f"{name}_target_mismatch")
            if elapsed > 180.0:
                thermal_failures.append(f"{name}_cooling_timeout")
            if target is not None and int(final) - target > 1:
                thermal_failures.append(f"{name}_target_not_reached")
        if not _bool(row["default_cool_rate_restored"], f"{name}.default_cool_rate_restored"):
            thermal_failures.append(f"{name}_default_cool_rate_not_restored")

    xy = motion["xy_home_residual_steps"]
    if not isinstance(xy, list) or len(xy) != 2:
        raise OemSelfTestReceiptError("motion.xy_home_residual_steps must contain exactly two values")
    x_error = _number(xy[0], "motion.x_home_residual_steps")
    y_error = _number(xy[1], "motion.y_home_residual_steps")
    if abs(x_error) > 100:
        failures.append("x_home_residual_exceeded")
    if abs(y_error) > 100:
        failures.append("y_home_residual_exceeded")
    if abs(_number(motion["z_home_residual_steps"], "motion.z_home_residual_steps")) > 100:
        failures.append("z_home_residual_exceeded")
    if abs(_number(motion["gripper_home_residual_steps"], "motion.gripper_home_residual_steps")) > 500:
        failures.append("gripper_home_residual_exceeded")
    if not _bool(motion["gantry_park_verified"], "motion.gantry_park_verified"):
        failures.append("gantry_park_not_verified")
    if not _bool(motion["door_open_verified"], "motion.door_open_verified"):
        failures.append("door_open_not_verified")

    motion_start = concurrency["motion_started_at_ms"]
    motion_complete = concurrency["motion_completed_at_ms"]
    if motion_complete < motion_start:
        failures.append("motion_completion_precedes_start")
    for branch, task in task_rows.items():
        if task["started_at_ms"] < task["submitted_at_ms"] or task["completed_at_ms"] < task["started_at_ms"]:
            failures.append(f"{branch}_task_timestamp_order_invalid")
        if not (task["started_at_ms"] < motion_complete and task["completed_at_ms"] > motion_start):
            failures.append(f"{branch}_did_not_overlap_motion")
    if concurrency["join_started_at_ms"] < motion_complete:
        failures.append("join_started_before_motion_completed")
    if concurrency["join_completed_at_ms"] < concurrency["join_started_at_ms"]:
        failures.append("join_completion_precedes_join_start")
    if concurrency["join_completed_at_ms"] < max(task["completed_at_ms"] for task in task_rows.values()):
        failures.append("join_completed_before_thermal_tasks")
    if concurrency["thermal_wait_completed_within_ms"] > 100_000:
        failures.append("parallel_completion_timeout")
    if not final_pwm:
        failures.append("final_chiller_pwm_not_reset")

    failures = thermal_failures + failures
    receipt_validation_pass = not failures
    nonthermal_failures = [failure for failure in failures if failure not in thermal_failures]
    oem_effective_pass = receipt_validation_pass or bool(inspection_log_only and thermal_failures and not nonthermal_failures)
    return {
        "ok": receipt_validation_pass,
        "status": "receipt_valid" if receipt_validation_pass else "receipt_rejected",
        "receipt_validation_pass": receipt_validation_pass,
        "oem_effective_pass": oem_effective_pass,
        "production_admission_pass": False,
        "provider_live_bound": False,
        "physical_motion_commanded": False,
        "physical_effect_verified": False,
        "failures": failures,
        "thermal_failures": thermal_failures,
        "inspection_log_only": inspection_log_only,
        "contract": self_test_contract(),
        "source_anchors": dict(SOURCE_ANCHORS),
    }
