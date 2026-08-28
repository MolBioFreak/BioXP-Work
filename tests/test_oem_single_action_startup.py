from __future__ import annotations

import importlib
import sys

import pytest

from src.bioxp.lifecycle_state import CanonicalLifecycleOwner, LifecycleStateError


def _load_api(monkeypatch):
    monkeypatch.setenv("BIOXP_SKIP_USB_CONNECT", "1")
    sys.modules.pop("src.bioxp.api", None)
    return importlib.import_module("src.bioxp.api")


class _InitialCheckHardware:
    def __init__(self, calls: list[str]):
        self.calls = calls

    def set_led_rgb(self, _r, _g, _b):
        self.calls.append("initial_check")
        return {"ok": True}

    def query_door(self):
        return {"value": 1, "ack": {"status": 100}}

    def query_latch(self):
        return {"value": 0, "ack": {"status": 100}}

    def set_solenoid(self, _value):
        return {"ok": True}

    def query_voltage(self):
        return {
            "payload_raw": 0,
            "reply_present": True,
            "transport_outcome": "reply",
            "oem_status": 100,
        }

    def deactivate_boards(self):
        return {"ok": True}

    def activate_boards(self):
        return {"ok": True}


def _fresh_owner() -> CanonicalLifecycleOwner:
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason="test_transport_ready")
    owner.bind_configuration(start_mode="WebMode", board_test_mode=False, check_camera=True)
    return owner


def test_single_oem_non_motion_startup_stops_after_failed_stage(monkeypatch):
    api = _load_api(monkeypatch)
    owner = _fresh_owner()
    calls: list[str] = []

    monkeypatch.setattr(api, "lifecycle_state", owner)
    monkeypatch.setattr(api, "_constructor_pipette_action", lambda: calls.append("constructor") or {"ok": False, "error": "pipette_failed"})
    monkeypatch.setattr(api, "_initialize_without_motion_action", lambda: calls.append("forbidden") or {"ok": True})
    monkeypatch.setattr(api, "_LifecycleHardware", lambda _tester: _InitialCheckHardware(calls))
    monkeypatch.setattr(api, "_get_tester", lambda: object())
    monkeypatch.setattr(api, "_can_ready_observation", lambda: True)

    result = api._run_oem_non_motion_startup_sequence(
        sleep=lambda _seconds: None,
        clock=lambda: 1.0,
    )

    assert result["ok"] is False
    assert result["failed_stage"] == "constructor_pipette_stage"
    assert calls == ["constructor"]
    assert owner.projection()["startup"]["stages"]["initialization_without_motion"]["state"] == "blocked"
    assert result["initialize_system_started"] is False


def test_openapi_exposes_one_live_non_motion_startup_route(monkeypatch):
    api = _load_api(monkeypatch)
    paths = api.app.openapi()["paths"]

    assert "/oem/startup/initialize_environment" in paths
    assert "post" in paths["/oem/startup/initialize_environment"]


def test_single_startup_http_route_requires_ack_and_returns_non_motion_boundary(monkeypatch):
    from fastapi.testclient import TestClient

    api = _load_api(monkeypatch)
    client = TestClient(api.app)

    denied = client.post("/oem/startup/initialize_environment", json={"mode": "live"})
    assert denied.status_code == 409
    assert "operator_ack INITIALIZE" in denied.json()["detail"]

    expected = {
        "ok": True,
        "failed_stage": None,
        "sequence": [
            "constructor_pipette_stage",
            "initialization_without_motion",
            "initial_check",
        ],
        "lifecycle": _fresh_owner().projection(),
        "physical_motion": False,
        "homing_performed": False,
        "initialize_system_started": False,
        "next_oem_boundary": "initializeSystem",
        "source_anchors": {
            "constructor": "ControlLib.cs:700,963-984",
            "environment": "BioXPMainWindow.cs:821,973-997",
        },
    }

    async def fake_run_blocking(_label, func, timeout_s):
        assert func is api._run_oem_non_motion_startup_sequence
        assert timeout_s == 460.0
        return expected

    monkeypatch.setattr(api, "_can_ready_observation", lambda: True)
    monkeypatch.setattr(api, "_run_blocking", fake_run_blocking)
    response = client.post(
        "/oem/startup/initialize_environment",
        json={"mode": "live", "operator_ack": "INITIALIZE"},
    )

    assert response.status_code == 200
    assert response.json()["initialize_system_started"] is False
    assert response.json()["next_oem_boundary"] == "initializeSystem"