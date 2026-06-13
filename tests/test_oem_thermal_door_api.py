import asyncio

import pytest
from fastapi import HTTPException

from src.bioxp import api


class FakeThermalDoorTester:
    def motion_gate_live_snapshot(self):
        return {"ok": True}

    def motion_arm_state(self):
        return {"armed": True, "reason": "strict_init_pass"}

    def motor_oem_door_search_home(self, *, timeout_s=20.0, startup=False):
        return {"ok": True, "operation": "doorSearchHome", "closed_after": True, "timeout_s": timeout_s, "startup": startup}

    def motor_oem_open_thermal_door(self, *, timeout_s=20.0):
        return {"ok": True, "operation": "openThermalDoor", "target": 16000, "after": {"opened": True}, "timeout_s": timeout_s}

    def motor_oem_close_thermal_door(self, *, timeout_s=20.0):
        return {"ok": True, "operation": "closeThermalDoor", "target": 0, "after": {"closed": True}, "timeout_s": timeout_s}


class FailingOpenTester(FakeThermalDoorTester):
    def motor_oem_open_thermal_door(self, *, timeout_s=20.0):
        return {"ok": False, "failure": "door_open_predicate_not_confirmed", "target": 16000, "after": {"opened": False}}


def _patch_runtime(monkeypatch, tester):
    monkeypatch.setattr(api, "_get_tester", lambda: tester)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda req=None: None)


def _run(coro):
    return asyncio.run(coro)


def test_thermal_door_routes_exist_in_fastapi_app():
    paths = {route.path for route in api.app.routes}

    assert "/motion/thermal_door/home" in paths
    assert "/motion/thermal_door/open" in paths
    assert "/motion/thermal_door/close" in paths


def test_thermal_door_open_requires_ack(monkeypatch):
    _patch_runtime(monkeypatch, FakeThermalDoorTester())
    req = api.ThermalDoorActionRequest(operator_ack="WRONG", reason="test")

    with pytest.raises(HTTPException) as exc:
        _run(api.motion_thermal_door_open(req))

    assert exc.value.status_code == 422
    assert "OPEN_THERMAL_DOOR" in str(exc.value.detail)


def test_thermal_door_open_success_returns_predicate_backed_payload(monkeypatch):
    _patch_runtime(monkeypatch, FakeThermalDoorTester())
    req = api.ThermalDoorActionRequest(operator_ack="OPEN_THERMAL_DOOR", reason="test", timeout_s=5)

    payload = _run(api.motion_thermal_door_open(req))

    assert payload["ok"] is True
    assert payload["target"] == 16000
    assert payload["after"]["opened"] is True


def test_thermal_door_open_failure_preserves_json_detail(monkeypatch):
    _patch_runtime(monkeypatch, FailingOpenTester())
    req = api.ThermalDoorActionRequest(operator_ack="OPEN_THERMAL_DOOR", reason="test")

    with pytest.raises(HTTPException) as exc:
        _run(api.motion_thermal_door_open(req))

    assert exc.value.status_code == 409
    assert exc.value.detail["failure"] == "door_open_predicate_not_confirmed"
    assert exc.value.detail["target"] == 16000


def test_thermal_door_home_and_close_routes_call_oem_methods(monkeypatch):
    _patch_runtime(monkeypatch, FakeThermalDoorTester())

    home = _run(api.motion_thermal_door_home(api.ThermalDoorActionRequest(operator_ack="HOME_THERMAL_DOOR", reason="test")))
    close = _run(api.motion_thermal_door_close(api.ThermalDoorActionRequest(operator_ack="CLOSE_THERMAL_DOOR", reason="test")))

    assert home["closed_after"] is True
    assert close["after"]["closed"] is True
