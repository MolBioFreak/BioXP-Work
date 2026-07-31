from __future__ import annotations

from typing import Any, cast

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


def test_initial_check_stops_immediately_after_failed_led_write():
    class FailedLedHardware(_InitialCheckHardware):
        def set_led_rgb(self, r, g, b):
            self.calls.append(("led_failed", r, g, b))
            return {"ok": False, "acks": {"r": {"status": 2}}, "sent": 1}

    owner = _owner_ready_for_initial_check()
    owner.bind_configuration(start_mode="API", board_test_mode=False, check_camera=False)
    hardware = FailedLedHardware()

    result = owner.run_initial_check(
        hardware, can_ready=lambda: True, sleep=lambda _seconds: None
    )

    evidence = result["startup"]["stages"]["initial_check"]["evidence"]
    assert evidence["error"] == "LED_white_failed"
    assert hardware.calls == [("led_failed", 255, 255, 255)]


def test_initial_check_does_not_activate_after_deactivation_failure():
    class FailedDeactivateHardware(_InitialCheckHardware):
        def deactivate_boards(self):
            self.calls.append(("deactivate_failed",))
            return {"ok": False, "acks": {4: None}}

    owner = _owner_ready_for_initial_check()
    owner.bind_configuration(start_mode="API", board_test_mode=False, check_camera=False)
    hardware = FailedDeactivateHardware()

    result = owner.run_initial_check(
        hardware, can_ready=lambda: True, sleep=lambda _seconds: None
    )

    evidence = result["startup"]["stages"]["initial_check"]["evidence"]
    assert evidence["error"] == "deactivate_boards_failed"
    assert ("deactivate_failed",) in hardware.calls
    assert not [call for call in hardware.calls if call[0] == "activate"]


def test_initial_check_stops_after_failed_production_door_ack():
    class FailedDoorHardware(_InitialCheckHardware):
        def query_door(self):
            self.calls.append(("door_failed",))
            return {"ok": False, "value": None, "ack": {"status": 2}, "query": "door"}

    owner = _owner_ready_for_initial_check()
    owner.bind_configuration(start_mode="API", board_test_mode=False, check_camera=False)
    hardware = FailedDoorHardware()

    result = owner.run_initial_check(
        hardware, can_ready=lambda: True, sleep=lambda _seconds: None
    )

    evidence = result["startup"]["stages"]["initial_check"]["evidence"]
    assert evidence["error"] == "query_door_failed"
    assert hardware.calls == [("led", 255, 255, 255), ("door_failed",)]


def test_initial_check_failed_voltage_status_does_not_release_solenoid_or_cycle_boards():
    class FailedVoltageHardware(_InitialCheckHardware):
        def query_voltage(self):
            self.calls.append(("voltage_failed",))
            return {
                "ok": False,
                "payload_raw": None,
                "reply_present": True,
                "transport_outcome": "reply",
                "oem_status": 2,
                "ack": {"status": 2},
            }

    owner = _owner_ready_for_initial_check()
    owner.bind_configuration(start_mode="API", board_test_mode=False, check_camera=False)
    hardware = FailedVoltageHardware()

    result = owner.run_initial_check(
        hardware, can_ready=lambda: True, sleep=lambda _seconds: None
    )

    evidence = result["startup"]["stages"]["initial_check"]["evidence"]
    assert evidence["error"] == "query_24V_failed"
    assert ("voltage_failed",) in hardware.calls
    assert not [call for call in hardware.calls if call[0] in {"solenoid", "deactivate", "activate"}]


def test_production_lifecycle_adapter_is_strict_fail_fast_and_returns_aggregate_success():
    from src.bioxp import api

    class Tester:
        BOARD_DECK = 5

        def __init__(self):
            self.calls = []
            self.activation_status = 100

        @staticmethod
        def _tmcl_success(ack):
            return isinstance(ack, dict) and ack.get("status") == 100

        @staticmethod
        def _oem_board_activation_map_success(rows):
            return bool(rows) and all(
                isinstance(row, dict) and (row.get("ack") or {}).get("status") == 100
                for row in rows.values()
            )

        def strip_set_rgb(self, *args, **kwargs):
            self.calls.append(("led", args, kwargs))
            return {"ok": True}

        def query_only_tmcl(self, *args):
            self.calls.append(("query", args))
            return {"status": 100, "value": 1}

        def deck_io_set_type(self, *args):
            self.calls.append(("solenoid", args))
            return {"ok": True}

        def deactivate_boards(self, **kwargs):
            self.calls.append(("deactivate", kwargs))
            return {4: {"ack": {"status": self.activation_status}}}

        def activate_boards(self, **kwargs):
            self.calls.append(("activate", kwargs))
            return {4: {"ack": {"status": self.activation_status}}}

    tester = Tester()
    adapter = api._LifecycleHardware(cast(Any, tester))

    assert adapter.set_led_rgb(255, 255, 255)["ok"] is True
    assert tester.calls[-1][2] == {
        "reconnect_first": False,
        "activate_first": False,
        "fail_fast": True,
    }
    assert adapter.query_door()["ok"] is True
    assert adapter.query_latch()["ok"] is True
    assert adapter.query_voltage()["ok"] is True
    assert adapter.deactivate_boards()["ok"] is True
    assert adapter.activate_boards()["ok"] is True
    assert ("deactivate", {"expect_reply": True, "fail_fast": True}) in tester.calls
    assert ("activate", {"expect_reply": True, "fail_fast": True}) in tester.calls

    tester.activation_status = 2
    failed = adapter.activate_boards()
    assert failed["ok"] is False
    assert failed["acks"] == {4: {"ack": {"status": 2}}}


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
