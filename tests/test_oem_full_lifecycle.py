from __future__ import annotations

import json

import pytest

from src.bioxp.oem_full_lifecycle import (
    OEM_ACQUISITION_ID,
    OEM_MACHINE_SERIAL,
    OemFullLifecycleError,
    OemFullLifecycleRuns,
    current_registry_sha256,
)
from src.bioxp.oem_movement_ledger import OEM_INITIALIZE_MOTORS_STAGES
from src.bioxp.oem_runtime_store import OEMRuntimeStore


def _request(**overrides):
    payload = {
        "command": "initialize_oem_movement_lifecycle",
        "operator_ack": "INITIALIZE",
        "expected_machine_serial": OEM_MACHINE_SERIAL,
        "expected_registry_sha256": current_registry_sha256(),
        "idempotency_key": "full-oem-dry-run-1",
        "mode": "dry_run",
        "inputs": {
            "ownership_generation": 41,
            "saved_status": 1,
            "ship_mode": "",
            "start_mode": "WebMode",
            "tip_present": False,
            "self_test_due": True,
            "camera_required": True,
            "deck_inspection": True,
        },
    }
    payload.update(overrides)
    return payload


def test_full_happy_path_dry_run_is_robot_owned_and_emits_no_physical_frames(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))

    created = runs.create(_request())
    finished = runs.execute_dry_run(created["run_id"])

    assert finished["terminal_state"] == "oem_movement_ready_job_admission"
    assert finished["source_authority_verified"] is True
    assert finished["configuration_verified"] is True
    assert finished["machine_serial"] == 206
    assert finished["acquisition_id"] == OEM_ACQUISITION_ID
    assert finished["ownership_generation"] == 41
    assert finished["physical_motion_commanded"] is False
    assert finished["physical_effect_verified"] is False
    assert finished["transport_frames"] == []
    assert finished["expected_next_stage"] is None
    expected_movement_stages = [
        f"initialize_motors_m{number:02d}_{row['key'].replace('-', '_')}"
        for number, row in enumerate(OEM_INITIALIZE_MOTORS_STAGES, start=1)
    ]
    assert [row["stage_id"] for row in finished["stages"]] == [
        "initialize_environment",
        "enqueue_initialize_system",
        "worker_claim_initialize_system",
        "initialize_system_reentry_guard",
        "initialize_system_initial_check",
        "configure_motors_without_motion",
        "initialize_motion_flags",
        *expected_movement_stages,
        "initialize_motion_tip_query",
        "initialize_motion_no_tip",
        "self_test_gate",
        "self_test_tc_rc_oc",
        "self_test_motion",
        "camera_gate",
        "camera_check",
        "cover_inspection",
        "gantry_park",
        "start_mode_web_job_admission",
    ]
    assert all(row["status"] == "completed" for row in finished["stages"])
    assert all(row["physical_motion_commanded"] is False for row in finished["stages"])
    assert all(row["controller_acknowledged"] is False for row in finished["stages"])
    assert all(row["postcondition_verified"] is False for row in finished["stages"])


def test_stale_tip_branch_and_not_due_self_test_preserve_exact_branch_order(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    request = _request(
        idempotency_key="tip-branch",
        inputs={
            **_request()["inputs"],
            "tip_present": True,
            "self_test_due": False,
            "camera_required": False,
            "deck_inspection": False,
            "start_mode": "DevMode",
        },
    )

    finished = runs.execute_dry_run(runs.create(request)["run_id"])
    stage_ids = [row["stage_id"] for row in finished["stages"]]

    assert stage_ids[stage_ids.index("initialize_motion_tip_query") + 1] == "tip_open_thermal_door"
    assert stage_ids[stage_ids.index("tip_open_thermal_door"):stage_ids.index("self_test_gate")] == [
        "tip_open_thermal_door",
        "tip_route_park_to_waste",
        "tip_eject_all",
        "tip_move_z_80000",
        "tip_move_x_79000",
        "tip_verify_empty",
        "pipette_reinitialize_retry_once",
    ]
    assert "self_test_tc_rc_oc" not in stage_ids
    assert "camera_check" not in stage_ids
    assert "cover_inspection" not in stage_ids
    assert finished["terminal_state"] == "oem_movement_ready_manual"


def test_saved_status_recovery_returns_before_normal_camera_cover_park_and_start_mode(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    request = _request(
        idempotency_key="saved-status",
        inputs={**_request()["inputs"], "saved_status": 3},
    )

    finished = runs.execute_dry_run(runs.create(request)["run_id"])
    stage_ids = [row["stage_id"] for row in finished["stages"]]

    assert stage_ids[-3:] == [
        "saved_status_initialize_motion",
        "saved_status_inspect_cover",
        "saved_status_unlock_warning_return",
    ]
    assert "camera_gate" not in stage_ids
    assert "gantry_park" not in stage_ids
    assert finished["terminal_state"] == "oem_movement_blocked_saved_status_recovery"


def test_creation_is_idempotent_and_rejects_authority_or_arbitrary_input(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    first = runs.create(_request())
    second = runs.create(_request())
    assert first["run_id"] == second["run_id"]

    with pytest.raises(OemFullLifecycleError, match="machine serial"):
        runs.create(_request(idempotency_key="wrong-serial", expected_machine_serial=3250))
    with pytest.raises(OemFullLifecycleError, match="registry"):
        runs.create(_request(idempotency_key="wrong-registry", expected_registry_sha256="0" * 64))
    with pytest.raises(OemFullLifecycleError, match="unknown lifecycle input"):
        runs.create(_request(idempotency_key="raw-motion", inputs={**_request()["inputs"], "axis": "z"}))


def test_restart_during_running_stage_blocks_and_never_auto_resumes(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    runs = OemFullLifecycleRuns(store)
    created = runs.create(_request())
    payload = runs.get(created["run_id"])
    payload["run_state"] = "running"
    payload["current_stage"] = "initialize_motors_m01_m19"
    payload["stages"][0]["status"] = "running"
    store.write_oem_full_lifecycle_run(payload)

    recovered = OemFullLifecycleRuns(store).recover(created["run_id"])

    assert recovered["run_state"] == "blocked"
    assert recovered["terminal_state"] == "oem_movement_blocked"
    assert recovered["blocked_reason"] == "restart_during_physical_or_unresolved_stage_requires_operator_inspection"
    assert recovered["physical_effect_verified"] is False


def test_cancel_only_at_safe_boundary(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    created = runs.create(_request())
    cancelled = runs.cancel(created["run_id"])
    assert cancelled["run_state"] == "cancelled"

    other = runs.create(_request(idempotency_key="unsafe-cancel"))
    payload = runs.get(other["run_id"])
    payload["run_state"] = "running"
    payload["current_stage"] = "initialize_motion_tip_query"
    runs.store.write_oem_full_lifecycle_run(payload)
    with pytest.raises(OemFullLifecycleError, match="safe cancellation boundary"):
        runs.cancel(other["run_id"])


def test_persisted_json_contains_no_fake_completion_claims(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    runs = OemFullLifecycleRuns(store)
    result = runs.execute_dry_run(runs.create(_request())["run_id"])
    raw = json.dumps(result, sort_keys=True)

    assert '"controller_acknowledged": true' not in raw
    assert '"physical_effect_verified": true' not in raw
    assert '"postcondition_verified": true' not in raw
