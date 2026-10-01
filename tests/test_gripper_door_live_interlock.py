"""Gripper/thermal-door controls do not grey out on a missed snapshot reply.

Their routes read 24 V, door and latch live at motion time, so readiness no
longer depends on the shared hardware snapshot.
"""
import pytest
from fastapi import HTTPException

from bioxp import api
from bioxp.operator_controls import _assess_action

READY = {
    "ownership": {"transport": "owned", "usb": "service", "router": "running", "CAN_READY": True},
    "maintenance": {"motion_blocked": False, "recovery_required": False},
    "lifecycle": {"operation_state": "idle"},
    "domains": {},  # snapshot blanked: no power/latch observation
}


def _action(path):
    return {"action_id": "route.test", "informational_method": "POST",
            "informational_path": path, "safety_class": "motion"}


@pytest.mark.parametrize("path", [
    "/motion/gripper/clear", "/motion/gripper/home", "/motion/gripper/open",
    "/motion/gripper/open_wide", "/motion/gripper/close",
    "/motion/thermal_door/home", "/motion/thermal_door/open", "/motion/thermal_door/close",
])
def test_blank_snapshot_keeps_gripper_and_door_enabled(path):
    assert _assess_action(_action(path), READY)["enabled"] is True


def test_motion_still_blocked_when_not_activated():
    state = {**READY, "maintenance": {"motion_blocked": True, "recovery_required": False}}
    assert _assess_action(_action("/motion/gripper/home"), state)["enabled"] is False


class _Tester:
    def __init__(self, ok):
        self.ok = ok

    def motor_oem_verify_motion_interlock(self):
        return {"ok": self.ok, "rail_24v": {"safety_valid": self.ok}, "door": {"value": 1}, "latch": {"value": 1 if self.ok else 0}}


def test_live_interlock_runs_the_motion_when_sensors_read_ok():
    assert api._with_live_interlock(_Tester(True), lambda: "moved") == "moved"


def test_live_interlock_refuses_without_commanding_motion():
    calls = []
    with pytest.raises(HTTPException) as exc:
        api._with_live_interlock(_Tester(False), lambda: calls.append("moved"))
    assert exc.value.status_code == 409
    assert exc.value.detail["error"] == "motion_interlock_not_ready"
    assert exc.value.detail["physical_motion_commanded"] is False
    assert calls == []
