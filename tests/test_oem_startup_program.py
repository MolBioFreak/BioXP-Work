import json
from pathlib import Path


def test_dry_run_waits_for_door_close_without_motion(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, FakeStartupHardware

    hw = FakeStartupHardware(door_closed=False, latch_closed=False)
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
    from src.bioxp.oem_startup_program import OEMStartupProgram, FakeStartupHardware

    hw = FakeStartupHardware(door_closed=False, latch_closed=False)
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
    from src.bioxp.oem_startup_program import OEMStartupProgram, FakeStartupHardware

    program = OEMStartupProgram(hardware=FakeStartupHardware(), artifact_base=tmp_path)

    missing_ack = program.request_startup({"mode": "live", "artifact_root": str(tmp_path / "live")})
    assert missing_ack["state"] == "failed_closed"
    assert "operator_ack" in missing_ack["failure_reason"]

    missing_artifact = program.request_startup({"mode": "live", "operator_ack": "INITIALIZE"})
    assert missing_artifact["state"] == "failed_closed"
    assert "artifact_root" in missing_artifact["failure_reason"]


def test_missing_required_config_fails_before_hardware(tmp_path):
    from src.bioxp.oem_startup_program import OEMStartupProgram, FakeStartupHardware

    hw = FakeStartupHardware(config_status={"status": "missing", "searched_roots": []})
    program = OEMStartupProgram(hardware=hw, artifact_base=tmp_path)

    status = program.request_startup({"mode": "dry_run", "require_config": True})

    assert status["state"] == "failed_closed"
    assert "config" in status["failure_reason"]
    assert hw.initial_check_calls == 0
