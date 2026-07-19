from pathlib import Path
import pytest


@pytest.fixture(autouse=True)
def _reset_canonical_startup_lifecycle():
    from src.bioxp.lifecycle_state import lifecycle_state

    lifecycle_state.transport_changed(None, reason="startup_test_setup")
    yield
    lifecycle_state.transport_changed(None, reason="startup_test_teardown")


def test_dry_run_binds_session_and_waits_for_explicit_constructor_stage(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hardware = DryRunStartupHardware(door_closed=False, latch_closed=False)
    program = OEMStartupProgram(hardware=hardware, artifact_base=tmp_path)

    status = program.request_startup(
        {"mode": "dry_run", "require_config": False, "door_policy": "wait_for_closed"}
    )

    assert status["state"] == "waiting_for_constructor_pipette_stage"
    assert status["next_action"] == "POST /oem/startup/constructor_pipettes"
    assert status["ready"] is False
    assert status["queued"] is False
    assert status["failed_closed"] is False
    assert hardware.motion_calls == []
    assert hardware.initial_check_calls == 0
    assert program.worker_status()["queue_depth"] == 0
    assert (Path(status["artifact_root"]) / "startup_request.json").exists()


def test_generic_startup_does_not_read_config_or_advance_door_policy(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hardware = DryRunStartupHardware(
        door_closed=True,
        latch_closed=True,
        config_status={"status": "missing", "searched_roots": []},
    )
    program = OEMStartupProgram(hardware=hardware, artifact_base=tmp_path)

    status = program.request_startup(
        {"mode": "dry_run", "require_config": True, "door_policy": "already_closed"}
    )

    assert status["state"] == "waiting_for_constructor_pipette_stage"
    assert status["completed_stages"] == []
    assert hardware.initial_check_calls == 0
    assert hardware.motion_calls == []


def test_door_event_records_canonical_door_truth_without_advancing_startup(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hardware = DryRunStartupHardware()
    program = OEMStartupProgram(hardware=hardware, artifact_base=tmp_path)
    startup = program.request_startup({"mode": "dry_run", "require_config": False})

    event = program.door_event(startup["session_id"], door_closed=True, latch_closed=True)

    assert event["ok"] is True
    assert event["session_id"] == startup["session_id"]
    assert event["state"] == "stopped"
    assert event["door"]["door_closed"] is True
    assert event["door"]["latch_closed"] is True
    assert event["lifecycle"]["startup"]["state"] == "not_run"
    assert event["lifecycle"]["startup"]["stages"]["constructor_pipette_stage"]["state"] == "not_run"
    assert hardware.initial_check_calls == 0
    assert hardware.motion_calls == []
    assert program.worker_status()["queue_depth"] == 0


def test_live_startup_requires_ack_and_artifact_root(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    program = OEMStartupProgram(hardware=DryRunStartupHardware(), artifact_base=tmp_path)

    missing_ack = program.request_startup(
        {"mode": "live", "artifact_root": str(tmp_path / "live")}
    )
    assert missing_ack["state"] == "failed_closed"
    assert "operator_ack" in missing_ack["failure_reason"]

    missing_artifact = program.request_startup(
        {"mode": "live", "operator_ack": "INITIALIZE"}
    )
    assert missing_artifact["state"] == "failed_closed"
    assert "artifact_root" in missing_artifact["failure_reason"]


def test_live_artifact_root_must_be_absolute_and_allowed(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    program = OEMStartupProgram(hardware=DryRunStartupHardware(), artifact_base=tmp_path)

    relative = program.request_startup(
        {
            "mode": "live",
            "operator_ack": "INITIALIZE",
            "artifact_root": "relative/live",
        }
    )
    assert relative["state"] == "failed_closed"
    assert "absolute" in relative["failure_reason"]

    outside = program.request_startup(
        {
            "mode": "live",
            "operator_ack": "INITIALIZE",
            "artifact_root": "/home/dalab/not-allowed-bioxp-live",
        }
    )
    assert outside["state"] == "failed_closed"
    assert "outside allowed" in outside["failure_reason"]


def test_live_request_does_not_homing_or_queue_worker_before_explicit_stages(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hardware = DryRunStartupHardware(door_closed=True, latch_closed=True)
    program = OEMStartupProgram(
        hardware=hardware,
        artifact_base=tmp_path,
        allowlist_roots=[tmp_path],
    )

    status = program.request_startup(
        {
            "mode": "live",
            "operator_ack": "INITIALIZE",
            "artifact_root": str(tmp_path / "live"),
            "require_config": True,
            "run_homing": True,
        }
    )

    assert status["state"] == "waiting_for_constructor_pipette_stage"
    assert status["queued"] is False
    assert program.run_next_worker_command() is None
    assert hardware.motion_calls == []
    assert hardware.initial_check_calls == 0


def test_live_startup_normalizes_legacy_raw_axis_prep_shape():
    from src.bioxp.oem_startup_program import BioXpStartupHardware

    class LegacyTester:
        def motor_oem_initialize_without_motion(self):
            return {"x": {"ops": []}, "y": {"ops": []}}

    hardware = BioXpStartupHardware(lambda: LegacyTester())
    result = hardware.configure_without_motion(mode="live")

    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert result["normalized_from_raw_axis_prep"] is True
    assert "axes" in result
