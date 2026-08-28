from pathlib import Path
import pytest


@pytest.fixture(autouse=True)
def _reset_canonical_startup_lifecycle():
    from src.bioxp.lifecycle_state import lifecycle_state

    lifecycle_state.transport_changed(None, reason="startup_test_setup")
    yield
    lifecycle_state.transport_changed(None, reason="startup_test_teardown")


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
