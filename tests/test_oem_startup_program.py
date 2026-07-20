import json
from pathlib import Path


def test_dry_run_waits_for_door_close_without_motion(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hw = DryRunStartupHardware(door_closed=False, latch_closed=False)
    program = OEMStartupProgram(hardware=hw, artifact_base=tmp_path)

    status = program.request_startup({"mode": "dry_run", "require_config": False, "door_policy": "wait_for_closed"})

    assert status["state"] == "waiting_for_door_close"
    assert status["ready"] is False
    assert status["failed_closed"] is False
    assert hw.motion_calls == []
    artifact_root = Path(status["artifact_root"])
    assert (artifact_root / "startup_request.json").exists()
    assert (artifact_root / "door_wait.json").exists()


def test_door_event_queues_initialize_system_after_initial_check(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hw = DryRunStartupHardware(door_closed=False, latch_closed=False)
    program = OEMStartupProgram(hardware=hw, artifact_base=tmp_path)
    first = program.request_startup({"mode": "dry_run", "require_config": False, "door_policy": "wait_for_closed"})

    hw.door_closed = True
    hw.latch_closed = True
    status = program.door_event(first["session_id"], door_closed=True, latch_closed=True)

    assert status["state"] == "initialize_system_queued"
    assert "initial_check_after_door" in status["completed_stages"]
    assert hw.motion_calls == []
    events = (Path(status["artifact_root"]) / "motion_queue_events.jsonl").read_text().splitlines()
    assert any(json.loads(line)["event"] == "queued" for line in events)


def test_live_startup_requires_ack_and_artifact_root(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    program = OEMStartupProgram(hardware=DryRunStartupHardware(), artifact_base=tmp_path)

    missing_ack = program.request_startup({"mode": "live", "artifact_root": str(tmp_path / "live")})
    assert missing_ack["state"] == "failed_closed"
    assert "operator_ack" in missing_ack["failure_reason"]

    missing_artifact = program.request_startup({"mode": "live", "operator_ack": "INITIALIZE"})
    assert missing_artifact["state"] == "failed_closed"
    assert "artifact_root" in missing_artifact["failure_reason"]


def test_missing_required_config_fails_before_hardware(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hw = DryRunStartupHardware(config_status={"status": "missing", "searched_roots": []})
    program = OEMStartupProgram(hardware=hw, artifact_base=tmp_path)

    status = program.request_startup({"mode": "dry_run", "require_config": True})

    assert status["state"] == "failed_closed"
    assert "config" in status["failure_reason"]
    assert hw.initial_check_calls == 0


def test_already_closed_runs_initial_check_twice_before_queue(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hw = DryRunStartupHardware(door_closed=True, latch_closed=True)
    program = OEMStartupProgram(hardware=hw, artifact_base=tmp_path)

    status = program.request_startup({"mode": "dry_run", "require_config": False, "door_policy": "already_closed"})

    assert status["state"] == "initialize_system_queued"
    assert hw.initial_check_calls == 2
    assert (Path(status["artifact_root"]) / "initial_check_after_door.json").exists()


def test_live_artifact_root_must_be_absolute_and_allowed(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    program = OEMStartupProgram(hardware=DryRunStartupHardware(), artifact_base=tmp_path)

    relative = program.request_startup({"mode": "live", "operator_ack": "INITIALIZE", "artifact_root": "relative/live"})
    assert relative["state"] == "failed_closed"
    assert "absolute" in relative["failure_reason"]

    outside = program.request_startup({"mode": "live", "operator_ack": "INITIALIZE", "artifact_root": "/home/dalab/not-allowed-bioxp-live"})
    assert outside["state"] == "failed_closed"
    assert "outside allowed" in outside["failure_reason"]


def test_door_event_only_valid_while_waiting(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    program = OEMStartupProgram(hardware=DryRunStartupHardware(door_closed=True, latch_closed=True), artifact_base=tmp_path)
    status = program.request_startup({"mode": "dry_run", "require_config": False})

    try:
        program.door_event(status["session_id"], door_closed=True, latch_closed=True)
    except ValueError as exc:
        assert "waiting_for_door_close" in str(exc)
    else:
        raise AssertionError("door_event unexpectedly accepted outside waiting state")


def test_live_homing_blocks_when_switch_predicates_unknown(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    cfg = {"status": "loaded", "live_ready": True, "fields": {"StartMode": "0", "GripperVersion": "1"}, "missing_fields": []}
    hw = DryRunStartupHardware(door_closed=True, latch_closed=True, config_status=cfg)
    program = OEMStartupProgram(hardware=hw, artifact_base=tmp_path, allowlist_roots=[tmp_path])

    status = program.request_startup({"mode": "live", "operator_ack": "INITIALIZE", "artifact_root": str(tmp_path / "live"), "require_config": True, "run_homing": True})

    assert status["state"] == "failed_closed"
    assert "predicates" in status["failure_reason"]
    assert status["axis_reference"]["ok"] is False


def test_worker_run_next_reaches_diagnostic_not_ready(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    hw = DryRunStartupHardware(door_closed=True, latch_closed=True)
    program = OEMStartupProgram(hardware=hw, artifact_base=tmp_path)
    queued = program.request_startup({"mode": "dry_run", "require_config": False})

    result = program.run_next_worker_command()
    status = program.status(queued["session_id"])

    assert result["ok"] is True
    assert status["state"] == "diagnostic_complete"
    assert status["ready"] is False
    assert "pipette" in status["failure_reason"]
    assert (Path(status["artifact_root"]) / "initialize_motors_trace.jsonl").exists()
    assert (Path(status["artifact_root"]) / "vision_inspection.json").exists()



def test_live_startup_normalizes_legacy_raw_axis_prep_shape(tmp_path):
    from src.bioxp.oem_startup_program import BioXpStartupHardware

    class LegacyTester:
        def motor_oem_initialize_without_motion(self):
            return {"x": {"ops": []}, "y": {"ops": []}}

    hw = BioXpStartupHardware(lambda: LegacyTester())
    result = hw.configure_without_motion(mode="live")

    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert result["normalized_from_raw_axis_prep"] is True
    assert "axes" in result
