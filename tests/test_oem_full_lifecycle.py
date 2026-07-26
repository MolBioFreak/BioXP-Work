from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor

import pytest

from src.bioxp.oem_full_lifecycle import (
    OEM_ACQUISITION_ID,
    OEM_MACHINE_SERIAL,
    OemFullLifecycleError,
    OemFullLifecycleRuns,
    current_authority_identity,
    current_registry_sha256,
)
from src.bioxp.oem_movement_ledger import OEM_INITIALIZE_MOTORS_STAGES
from src.bioxp.oem_runtime_store import OEMRuntimeStore


def _request(**overrides):
    payload = {
        "command": "initialize_oem_movement_lifecycle",
        "operator_ack": "INITIALIZE",
        "expected_generation": 41,
        "bms_connection_generation": 77,
        "expected_machine_serial": OEM_MACHINE_SERIAL,
        "expected_registry_sha256": current_registry_sha256(),
        "expected_evidence_lock_sha256": current_authority_identity()["evidence_lock_sha256"],
        "idempotency_key": "full-oem-dry-run-1",
        "mode": "dry_run",
        "inputs": {
            "ownership_generation": 41,
            "can_ready": True,
            "board_test_mode": False,
            "pipette_exists": None,
            "initialize_system_producer": "initializeEnvironment",
            "update_check_suppresses_initialize_system": False,
            "system_in_motion_at_entry": False,
            "enclosure_door_closed": True,
            "latch_closed": True,
            "saved_status": 1,
            "ship_mode": "",
            "start_mode": "WebMode",
            "tip_present": False,
            "self_test_due": True,
            "check_camera": True,
            "camera_installed": True,
            "is_development_machine": False,
            "deck_inspection": True,
        },
    }
    payload.update(overrides)
    return payload


def _cancel(runs, run_id):
    request = _request()
    return runs.cancel(run_id, {
        field: request[field]
        for field in (
            "expected_generation",
            "bms_connection_generation",
            "expected_machine_serial",
            "expected_registry_sha256",
            "expected_evidence_lock_sha256",
        )
    })


def test_full_happy_path_dry_run_is_robot_owned_and_emits_no_physical_frames(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))

    created = runs.create(_request())
    finished = runs.execute_dry_run(created["run_id"])

    assert finished["run_state"] == "dry_run_non_ready"
    assert finished["terminal_state"] == "dry_run_non_readiness_terminal"
    assert finished["planned_terminal_state"] == "oem_movement_ready_job_admission"
    assert finished["evidence_lock_identity_verified"] is True
    assert finished["source_authority_verified"] is False
    assert finished["configuration_verified"] is False
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
        "construct_control_lib",
        "constructor_pipette_initiate_status_retry",
        "configure_motors_without_motion",
        "initialize_environment",
        "initialize_environment_initial_check",
        "enqueue_initialize_system",
        "worker_update_check_gate",
        "initialize_system_reentry_guard",
        "initialize_system_latch_set",
        "initialize_system_initial_check",
        "initialize_motion_flags",
        *expected_movement_stages,
        "initialize_motion_tip_query",
        "initialize_motion_no_tip",
        "self_test_gate",
        "self_test_launch_tc_rc_oc",
        "self_test_motion_while_thermal_running",
        "self_test_join_and_reset_chillers",
        "camera_gate",
        "camera_check",
        "cover_force_high_home",
        "cover_inspection",
        "gantry_park",
        "start_mode_webmode_terminal",
        "initialize_system_latch_cleared_finally",
    ]
    assert all(row["status"] == "dry_run_simulated" for row in finished["stages"])
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
            "check_camera": False,
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
    assert "self_test_launch_tc_rc_oc" not in stage_ids
    assert "camera_check" not in stage_ids
    assert "cover_force_high_home" in stage_ids
    assert "cover_inspection" not in stage_ids
    assert "cover_inspection_disabled_return_ok" in stage_ids
    assert finished["planned_terminal_state"] == "oem_movement_ready_development"
    assert finished["terminal_state"] == "dry_run_non_readiness_terminal"


def test_saved_status_recovery_returns_before_normal_camera_cover_park_and_start_mode(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    request = _request(
        idempotency_key="saved-status",
        inputs={**_request()["inputs"], "saved_status": 3},
    )

    finished = runs.execute_dry_run(runs.create(request)["run_id"])
    stage_ids = [row["stage_id"] for row in finished["stages"]]

    assert stage_ids[-4:] == [
        "saved_status_initialize_motion",
        "saved_status_inspect_cover",
        "saved_status_unlock_warning_return",
        "initialize_system_latch_cleared_finally",
    ]
    assert "camera_gate" not in stage_ids
    assert "gantry_park" not in stage_ids
    assert stage_ids[-1] == "initialize_system_latch_cleared_finally"
    assert finished["planned_terminal_state"] == "oem_movement_blocked_saved_status_recovery"
    assert finished["terminal_state"] == "dry_run_non_readiness_terminal"


def test_application_admission_branches_before_enqueue_and_motor_config_is_constructor_ordered(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    open_request = _request(
        idempotency_key="door-open-admission",
        inputs={
            **_request()["inputs"],
            "enclosure_door_closed": False,
            "latch_closed": False,
        },
    )
    payload = runs.create(open_request)
    stage_ids = [row["stage_id"] for row in payload["stages"]]
    assert stage_ids == [
        "construct_control_lib",
        "constructor_pipette_initiate_status_retry",
        "configure_motors_without_motion",
        "initialize_environment",
        "initialize_environment_initial_check",
        "admission_warning_enclosure_open",
    ]
    assert "enqueue_initialize_system" not in stage_ids
    assert payload["planned_terminal_state"] == "oem_waiting_enclosure_open"


def test_initial_check_and_self_test_metadata_preserve_hardware_and_parallel_truth(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    payload = runs.create(_request(idempotency_key="source-semantics"))
    by_id = {row["stage_id"]: row for row in payload["stages"]}
    assert by_id["initialize_environment_initial_check"]["would_command_hardware"] is True
    assert by_id["initialize_environment_initial_check"]["result_semantics"] == "return_value_ignored_by_oem_caller"
    assert by_id["initialize_system_initial_check"]["would_command_hardware"] is True
    assert by_id["self_test_launch_tc_rc_oc"]["parallel_branches"] == ["TC", "RC", "OC"]
    assert by_id["self_test_motion_while_thermal_running"]["would_command_physical_motion"] is True
    assert by_id["self_test_join_and_reset_chillers"]["join_timeout_ms"] == 100000
    assert by_id["initialize_motion_flags"]["source_anchor"] == "ControlLib.initializeMotion:8797-8804"
    assert by_id["initialize_motion_tip_query"]["source_anchor"] == "ControlLib.initializeMotion:8805-8807"
    assert by_id["camera_check"]["source_anchor"].endswith("ControlLib.CheckCamera:1929-1960")
    movement_rows = [row for row in payload["stages"] if row["stage_id"].startswith("initialize_motors_m")]
    assert len(movement_rows) == 19
    assert [row["movement_ledger_stage_id"] for row in movement_rows] == [f"M{number:02d}" for number in range(1, 20)]
    assert [row["movement_ledger_stage"] for row in movement_rows] == [row["key"] for row in OEM_INITIALIZE_MOTORS_STAGES]
    assert all(row["movement_ledger_schema"] == "bioxp.oem_initialize_motors_ledger.v1" for row in movement_rows)


def test_constructor_branch_nesting_matches_can_and_board_test_mode(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    normal = runs.create(_request(idempotency_key="constructor-normal"))
    normal_ids = [row["stage_id"] for row in normal["stages"]]
    assert normal_ids[1:3] == [
        "constructor_pipette_initiate_status_retry",
        "configure_motors_without_motion",
    ]
    _cancel(runs, normal["run_id"])

    board = runs.create(_request(
        idempotency_key="constructor-board-test",
        inputs={**_request()["inputs"], "board_test_mode": True, "pipette_exists": True},
    ))
    board_ids = [row["stage_id"] for row in board["stages"]]
    assert board_ids[1:3] == [
        "constructor_board_test_pipette_initiate_pressure",
        "configure_motors_without_motion",
    ]
    _cancel(runs, board["run_id"])

    no_can = runs.create(_request(
        idempotency_key="constructor-no-can",
        inputs={**_request()["inputs"], "can_ready": False},
    ))
    no_can_ids = [row["stage_id"] for row in no_can["stages"]]
    assert not [stage for stage in no_can_ids if stage.startswith("constructor_") and stage != "construct_control_lib"]


def test_ship_mode_models_literal_ignored_door_result_and_separate_linux_safety_block(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    payload = runs.create(_request(
        idempotency_key="ship-mode",
        inputs={**_request()["inputs"], "ship_mode": "PARK"},
    ))
    assert [row["stage_id"] for row in payload["stages"][-2:]] == [
        "ship_mode_close_door_result_ignored",
        "ship_mode_shutdown_requested",
    ]
    close = payload["stages"][-2]
    assert close["result_semantics"] == "boolean_result_ignored_by_oem_caller"
    assert payload["planned_terminal_state"] == "oem_shipping_shutdown_requested_latch_remains_set"
    assert payload["safety_deviation"] == [{
        "deviation_id": "ship_mode_shutdown_interlock",
        "oem_semantics": "doorOpen(false,false) result ignored; shutdown requested; m_systemInmotion remains true",
        "linux_safety_policy": "never request OS shutdown or infer door closure from an unbound dry-run provider",
        "live_execution_blocked": True,
    }]


def test_plan_binds_current_evidence_lock_and_selected_producer(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    payload = runs.create(_request(idempotency_key="authority-producer"))
    assert payload["evidence_lock_sha256"] == "a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c"
    enqueue = next(row for row in payload["stages"] if row["stage_id"] == "enqueue_initialize_system")
    assert enqueue["producer"] == "initializeEnvironment"


def test_can_unavailable_branch_never_enqueues_or_configures_motors(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    request = _request(
        idempotency_key="can-unavailable",
        inputs={**_request()["inputs"], "can_ready": False, "start_mode": "WebMode"},
    )
    payload = runs.create(request)
    stage_ids = [row["stage_id"] for row in payload["stages"]]
    assert "configure_motors_without_motion" not in stage_ids
    assert "initialize_environment_initial_check" not in stage_ids
    assert "enqueue_initialize_system" not in stage_ids
    assert payload["planned_terminal_state"] == "oem_blocked_can_unavailable"


def test_update_check_and_reentry_predicates_terminate_before_latch_or_motion(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    update = runs.create(_request(
        idempotency_key="update-suppressed",
        inputs={**_request()["inputs"], "update_check_suppresses_initialize_system": True},
    ))
    update_ids = [row["stage_id"] for row in update["stages"]]
    assert update_ids[-1] == "worker_update_check_suppressed_initialize_system"
    assert "initialize_system_latch_set" not in update_ids
    assert update["planned_terminal_state"] == "oem_initialize_system_suppressed_by_update_check"

    _cancel(runs, update["run_id"])
    reentry = runs.create(_request(
        idempotency_key="reentry-suppressed",
        inputs={**_request()["inputs"], "system_in_motion_at_entry": True},
    ))
    reentry_ids = [row["stage_id"] for row in reentry["stages"]]
    assert reentry_ids[-1] == "initialize_system_reentry_return"
    assert "initialize_system_latch_set" not in reentry_ids
    assert reentry["planned_terminal_state"] == "oem_initialize_system_reentry_suppressed"


def test_is_development_machine_is_distinct_from_start_mode_and_suppresses_only_camera(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    dev_host = runs.create(_request(
        idempotency_key="dev-machine-web-mode",
        inputs={**_request()["inputs"], "start_mode": "WebMode", "is_development_machine": True},
    ))
    ids = [row["stage_id"] for row in dev_host["stages"]]
    assert "camera_check" not in ids
    camera_gate = next(row for row in dev_host["stages"] if row["stage_id"] == "camera_gate")
    assert camera_gate["source_predicate"] == "CheckCamera && CameraInstalled && !IsDevelopmentMachine()"
    assert dev_host["planned_terminal_state"] == "oem_movement_ready_job_admission"

    _cancel(runs, dev_host["run_id"])
    dev_mode = runs.create(_request(
        idempotency_key="dev-mode-production-host",
        inputs={**_request()["inputs"], "start_mode": "DevMode", "is_development_machine": False},
    ))
    assert "camera_check" in [row["stage_id"] for row in dev_mode["stages"]]
    terminal = next(row for row in dev_mode["stages"] if row["stage_id"] == "start_mode_devmode_terminal")
    assert terminal["would_command_hardware"] is True


def test_creation_is_idempotent_and_rejects_authority_or_arbitrary_input(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    first = runs.create(_request())
    second = runs.create(_request())
    assert first["run_id"] == second["run_id"]

    with ThreadPoolExecutor(max_workers=8) as pool:
        raced = list(pool.map(lambda _index: runs.create(_request()), range(32)))
    assert {row["run_id"] for row in raced} == {first["run_id"]}
    assert len(runs.store.list_oem_full_lifecycle_runs()) == 1

    with pytest.raises(OemFullLifecycleError, match="machine serial"):
        runs.create(_request(idempotency_key="wrong-serial", expected_machine_serial=3250))
    with pytest.raises(OemFullLifecycleError, match="registry"):
        runs.create(_request(idempotency_key="wrong-registry", expected_registry_sha256="0" * 64))
    with pytest.raises(OemFullLifecycleError, match="unknown lifecycle input"):
        runs.create(_request(idempotency_key="raw-motion", inputs={**_request()["inputs"], "axis": "z"}))


def test_atomic_idempotency_reservation_converges_first_concurrent_creators(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    request = _request(idempotency_key="first-concurrent-create")
    with ThreadPoolExecutor(max_workers=8) as pool:
        raced = list(pool.map(lambda _index: runs.create(request), range(32)))
    assert len({row["run_id"] for row in raced}) == 1
    assert len(runs.store.list_oem_full_lifecycle_runs()) == 1


def test_atomic_active_run_exclusion_rejects_distinct_first_keys(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    requests = [_request(idempotency_key=f"distinct-{index}") for index in range(16)]

    def create(request):
        try:
            return ("created", runs.create(request)["run_id"])
        except OemFullLifecycleError as exc:
            return ("blocked", str(exc))

    with ThreadPoolExecutor(max_workers=16) as pool:
        results = list(pool.map(create, requests))
    assert len([row for row in results if row[0] == "created"]) == 1
    assert all("active OEM lifecycle run" in row[1] for row in results if row[0] == "blocked")
    assert len(runs.store.list_oem_full_lifecycle_runs()) == 1


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
    cancelled = _cancel(runs, created["run_id"])
    assert cancelled["run_state"] == "cancelled"

    other = runs.create(_request(idempotency_key="unsafe-cancel"))
    payload = runs.get(other["run_id"])
    payload["run_state"] = "running"
    payload["current_stage"] = "initialize_motion_tip_query"
    runs.store.write_oem_full_lifecycle_run(payload)
    with pytest.raises(OemFullLifecycleError, match="safe cancellation boundary"):
        _cancel(runs, other["run_id"])


def test_persisted_json_contains_no_fake_completion_claims(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    runs = OemFullLifecycleRuns(store)
    result = runs.execute_dry_run(runs.create(_request())["run_id"])
    raw = json.dumps(result, sort_keys=True)

    assert '"controller_acknowledged": true' not in raw
    assert '"physical_effect_verified": true' not in raw
    assert '"postcondition_verified": true' not in raw
