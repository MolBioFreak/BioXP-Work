from __future__ import annotations

import pytest

from src.bioxp.lifecycle_state import CanonicalLifecycleOwner, LifecycleStateError


class _InitialCheckHardware:
    def __init__(self):
        self.calls = []

    def set_led_rgb(self, r, g, b):
        self.calls.append(("led", r, g, b))
        return {"ok": True}

    def query_door(self):
        self.calls.append(("door",))
        return {"value": 1, "ack": {"status": 100}}

    def query_latch(self):
        self.calls.append(("latch",))
        return {"value": 0, "ack": {"status": 100}}

    def set_solenoid(self, value):
        self.calls.append(("solenoid", value))
        return {"ok": True}

    def query_voltage(self):
        self.calls.append(("voltage",))
        return {
            "payload_raw": 0,
            "reply_present": True,
            "transport_outcome": "reply",
            "oem_status": 100,
        }

    def deactivate_boards(self):
        self.calls.append(("deactivate",))
        return {"ok": True}

    def activate_boards(self):
        self.calls.append(("activate",))
        return {"ok": True}


def _owner_ready_for_initial_check():
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason="test_transport_ready")
    owner.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    owner.run_stage("initialization_without_motion", lambda: {"ok": True})
    return owner


def test_initial_check_is_repeatable_and_does_not_consume_camera_dependency():
    owner = _owner_ready_for_initial_check()
    owner.bind_configuration(start_mode="API", board_test_mode=False, check_camera=True)
    hardware = _InitialCheckHardware()
    sleeps = []

    first = owner.run_initial_check(
        hardware,
        can_ready=lambda: True,
        sleep=lambda seconds: sleeps.append(seconds),
    )
    second = owner.run_initial_check(
        hardware,
        can_ready=lambda: True,
        sleep=lambda seconds: sleeps.append(seconds),
    )

    first_evidence = first["startup"]["stages"]["initial_check"]["evidence"]
    row = second["startup"]["stages"]["initial_check"]
    assert first_evidence["ok"] is True
    assert "camera_dependency" not in first_evidence
    assert not [step for step in first_evidence["trace"] if "Camera" in step["step"]]
    assert row["state"] == "passed"
    assert row["attempt_count"] == 2
    assert len(row["history"]) == 1
    assert row["history"][0]["state"] == "passed"
    assert sleeps == [0.05, 0.5, 0.05, 0.5]
    assert [call[0] for call in hardware.calls].count("deactivate") == 2
    assert [call[0] for call in hardware.calls].count("activate") == 2


def test_camera_gate_is_evaluated_only_at_initialize_system_boundary():
    owner = _owner_ready_for_initial_check()
    owner.bind_configuration(start_mode="API", board_test_mode=False, check_camera=True)

    missing = owner.initialize_system_camera_dependency()
    assert missing["ok"] is False
    assert missing["stage"] == "initializeSystem_after_initializeMotion_before_inspectCover"

    owner.record_camera_evidence({"available": True, "ok": True, "probe_id": "probe-1"})
    ready = owner.initialize_system_camera_dependency()
    assert ready["ok"] is True
    assert ready["evidence"]["probe_id"] == "probe-1"


def test_nonrepeatable_predecessor_stages_remain_single_owner_operations():
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason="test_transport_ready")
    owner.run_stage("constructor_pipette_stage", lambda: {"ok": True})

    with pytest.raises(LifecycleStateError, match="already passed"):
        owner.run_stage("constructor_pipette_stage", lambda: {"ok": True})
