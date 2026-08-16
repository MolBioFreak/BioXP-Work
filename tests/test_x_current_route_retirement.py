from __future__ import annotations

import asyncio
from contextlib import contextmanager

import pytest
from fastapi import HTTPException


@contextmanager
def _operator_context(api):
    from bioxp import operator_controls

    token = operator_controls._DISPATCH_CONTEXT.set(
        {
            "operator_command_id": "op-x-route-proof",
            "idempotency_key": "idem-x-route-proof",
            "expected_ownership_generation": 17,
        }
    )
    try:
        yield
    finally:
        operator_controls._DISPATCH_CONTEXT.reset(token)


def test_generic_x_current_route_rejects_before_tester_access(monkeypatch, tmp_path):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    from bioxp import api

    monkeypatch.setattr(
        api,
        "_get_tester",
        lambda: (_ for _ in ()).throw(AssertionError("tester access before provider retirement")),
    )
    request = api.MotionAxisCurrentRequest(axes=[api.AxisName.X], run_current=31)

    with pytest.raises(HTTPException) as raised:
        asyncio.run(api.motion_axes_current(request))

    assert raised.value.status_code == 410
    assert raised.value.detail["error"] == "generic_x_current_mutation_retired"
    assert raised.value.detail["io_performed"] is False


def test_generic_mixed_xy_current_route_rejects_atomically_before_tester_access(monkeypatch, tmp_path):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    from bioxp import api

    monkeypatch.setattr(
        api,
        "_get_tester",
        lambda: (_ for _ in ()).throw(AssertionError("mixed-axis current request performed I/O")),
    )
    request = api.MotionAxisCurrentRequest(axes=[api.AxisName.X, api.AxisName.Y], run_current=31)

    with pytest.raises(HTTPException) as raised:
        asyncio.run(api.motion_axes_current(request))

    assert raised.value.status_code == 410
    assert raised.value.detail["replacements"] == ["oem.xy.enable", "oem.xyz.enable"]
    assert raised.value.detail["physical_motion_commanded"] is False


def test_typed_x_routes_dispatch_exact_provider_intents(monkeypatch, tmp_path):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    from bioxp import api

    calls = []

    class Provider:
        def capability_status(self):
            return {"initialize_motors_live_available": True}

        def execute_x_intent(self, intent, values):
            calls.append((intent, dict(values)))
            return {"ok": True, "intent": intent}

    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", Provider())
    monkeypatch.setattr(api, "_run_blocking", lambda _label, fn, **_kwargs: asyncio.sleep(0, result=fn()))

    with _operator_context(api):
        asyncio.run(api.motion_oem_x_diagnostic_home_axis())
        asyncio.run(api.motion_oem_x_set_home(api.OemXSetHomeRequest(operator_ack="SET_HOME_CURRENT_POSITION")))
        asyncio.run(api.motion_oem_x_internal_enable_xy(api.OemXCurrentModeRequest(enabled=True)))
        asyncio.run(api.motion_oem_x_internal_enable_xyz(api.OemXCurrentModeRequest(enabled=False, z_current_up=27)))
        asyncio.run(api.motion_oem_x_observation(api.OemXObservationRequest(
            command_id="movement-command",
            verdict="pass",
            physical_motion_observed=True,
            expected_direction_observed=True,
            home_endpoint_observed=False,
            stopped_observed=True,
            note="observed",
        )))

    assert [row[0] for row in calls] == [
        "home_axis",
        "set_home",
        "enable_xy_current",
        "enable_xyz_current",
        "observe",
    ]
    assert calls[1][1]["operator_ack"] == "SET_HOME_CURRENT_POSITION"
    assert calls[2][1]["enabled"] is True
    assert calls[3][1]["z_current_up"] == 27
    assert calls[4][1]["observed_command_id"] == "movement-command"
    assert all(row[1]["command_id"] == "op-x-route-proof" for row in calls)
    assert all(row[1]["idempotency_key"] == "idem-x-route-proof" for row in calls)
    assert all(row[1]["expected_generation"] == 17 for row in calls)


def test_homexy_fails_closed_without_provider_before_tester_access(monkeypatch, tmp_path):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    from bioxp import api

    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", None)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    monkeypatch.setattr(
        api,
        "_get_tester",
        lambda: (_ for _ in ()).throw(AssertionError("HomeXY direct tester fallback used")),
    )
    request = api.OemHomeXYRequest(operator_ack="HOMEXY")

    with pytest.raises(HTTPException) as raised:
        asyncio.run(api.motion_oem_home_xy(request))

    assert raised.value.status_code == 503
    assert raised.value.detail["error"] == "serial206_homexy_provider_unavailable"
    assert raised.value.detail["physical_motion_commanded"] is False
