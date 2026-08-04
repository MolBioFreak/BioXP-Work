import asyncio

import pytest
from fastapi import HTTPException


def test_live_right_reference_route_is_retired():
    import src.bioxp.api as api

    with pytest.raises(HTTPException) as exc:
        asyncio.run(api.motion_oem_z_live_right_reference())

    assert exc.value.status_code == 410
    assert exc.value.detail["error"] == "z_live_right_reference_retired"


def test_diagnostic_home_requires_named_597_confirmation_and_stays_separate(monkeypatch):
    import src.bioxp.api as api

    calls = []

    async def run_blocking(_label, fn, *, timeout_s):
        calls.append(("timeout", timeout_s))
        return fn()

    def execute(intent, values=None):
        calls.append((intent, dict(values or {})))
        return {"ok": True, "diagnostic_only": True}

    monkeypatch.setattr(api, "_run_blocking", run_blocking)
    monkeypatch.setattr(api, "_execute_provider_z_intent", execute)

    request = api.OemZDiagnosticHomeRequest(confirm="DIAGNOSTIC_Z_HOME_597")
    result = asyncio.run(api.motion_oem_z_diagnostic_home_axis(request))

    assert result["ok"] is True
    assert ("diagnostic_home_axis", {"timeout_s": 30.0, "confirm": "DIAGNOSTIC_Z_HOME_597"}) in calls


def test_manual_z_home_dispatches_only_provider_move_z_home_path(monkeypatch):
    import src.bioxp.api as api

    calls = []

    async def run_blocking(_label, fn, *, timeout_s):
        calls.append(("timeout", timeout_s))
        return fn()

    def execute(intent, values=None):
        calls.append((intent, dict(values or {})))
        return {"ok": True, "source_method": "ClassControlInterface.MoveZHome -> goHome(true,1791)"}

    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    monkeypatch.setattr(api, "_run_blocking", run_blocking)
    monkeypatch.setattr(api, "_execute_provider_z_intent", execute)

    result = asyncio.run(api.motion_oem_manual_home(api.OemManualHomeRequest(axis="z")))

    assert result["ok"] is True
    assert ("manual_home", {"timeout_s": 30.0}) in calls
    assert not any(call[0] == "diagnostic_home_axis" for call in calls if isinstance(call, tuple))


def test_z_stop_route_uses_the_safety_interrupt_lane(monkeypatch):
    import src.bioxp.api as api

    calls = []
    tester_sentinel = object()

    async def run_safety_interrupt(label, fn, *, timeout_s):
        calls.append(("lane", label, timeout_s))
        return fn(tester_sentinel)

    def execute(intent, values=None):
        calls.append((intent, dict(values or {})))
        return {"ok": True, "interrupt": True}

    monkeypatch.setattr(api, "_run_safety_interrupt_blocking", run_safety_interrupt)
    monkeypatch.setattr(api, "_execute_provider_z_intent", execute)

    result = asyncio.run(api.motion_oem_z_stop())

    assert result == {"ok": True, "interrupt": True}
    assert calls == [
        ("lane", "serial-206 Z stop", 10.0),
        ("stop", {"timeout_s": 3.0}),
    ]
