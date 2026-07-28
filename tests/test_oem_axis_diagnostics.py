from __future__ import annotations

import asyncio

import pytest
from fastapi import HTTPException
from pydantic import ValidationError


def test_catalog_exposes_complete_finite_robot_owned_actions():
    from src.bioxp.oem_axis_diagnostics import AXIS_DIAGNOSTIC_CATALOG, diagnostic_catalog

    assert AXIS_DIAGNOSTIC_CATALOG == {
        "x": ("status", "move-negative", "move-positive", "home", "park-6000", "stop"),
        "y": ("status", "move-negative", "move-positive", "home", "stop"),
        "z": ("status", "move-negative", "move-positive", "home", "stop"),
        "g": ("status", "commission-home", "close", "open", "open-wide", "stop"),
        "door": ("status", "home", "open", "close", "stop"),
    }
    projection = diagnostic_catalog()
    assert projection["schema"] == "bioxp.oem_axis_diagnostics.v1"
    assert projection["caller_supplied_motion_values"] is False
    assert projection["axes"]["x"]["move-positive"]["steps"] == 100
    assert projection["axes"]["x"]["park-6000"]["position_steps"] == 6000
    assert projection["axes"]["g"]["commission-home"]["required_idle_readback"] == {
        "run": 10,
        "standby": 10,
    }


def test_resolver_rejects_unknown_and_cross_axis_actions():
    from src.bioxp.oem_axis_diagnostics import AxisDiagnosticContractError, resolve_axis_diagnostic

    for axis, operation in (
        ("x", "open"),
        ("door", "move-positive"),
        ("g", "move-zero"),
        ("y", "park-6000"),
        ("raw-board-4", "home"),
        ("x", "set-current-31"),
    ):
        with pytest.raises(AxisDiagnosticContractError):
            resolve_axis_diagnostic(axis, operation)


def test_robot_api_publishes_only_finite_diagnostic_models_and_routes():
    from src.bioxp import api

    paths = {route.path for route in api.app.routes if hasattr(route, "path")}
    assert {
        "/motion/diagnostics/catalog",
        "/motion/diagnostics/status",
        "/motion/diagnostics/execute",
        "/motion/diagnostics/stop",
    } <= paths
    assert {
        "collect_axis_diagnostics",
        "run_axis_diagnostic",
        "stop_axis_diagnostic",
    } <= set(api.BMS_COMMISSIONING_CAPABILITIES)

    with pytest.raises(ValidationError):
        api.AxisDiagnosticExecuteRequest(
            axis="x",
            operation="move-positive",
            operator_ack="RUN_AXIS_DIAGNOSTIC",
            reason="test",
            steps=999999,
        )
    with pytest.raises(ValidationError):
        api.AxisDiagnosticExecuteRequest(
            axis="x",
            operation="move-positive",
            operator_ack="wrong",
            reason="test",
        )
    with pytest.raises(ValidationError):
        api.AxisDiagnosticStopRequest(axis="x", operator_ack="wrong", reason="test")


def test_x_positive_diagnostic_dispatches_fixed_relative_steps(monkeypatch):
    from src.bioxp import api

    class Tester:
        def _motion_oem_axis_profile(self, axis, startup=False):
            assert axis == "x"
            return {"board": 3, "motor": 0}

        def motor_axis_status(self, board, motor):
            return {"ok": True, "board": board, "motor": motor, "speed": {"speed": 0}}

        def motor_query_home_switch(self, board, motor):
            return {"ok": True, "board": board, "motor": motor, "active": False}

    captured: dict[str, object] = {}

    async def fake_relative(req):
        captured.update(req.model_dump())
        return {"ok": True, "motion": "relative"}

    async def immediate(_label, fn, **_kwargs):
        return fn()

    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    monkeypatch.setattr(api, "_get_tester", lambda: Tester())
    monkeypatch.setattr(api, "move_axis_relative", fake_relative)
    monkeypatch.setattr(api, "_run_blocking", immediate)

    response = asyncio.run(
        api.motion_diagnostics_execute(
            api.AxisDiagnosticExecuteRequest(
                axis="x",
                operation="move-positive",
                operator_ack="RUN_AXIS_DIAGNOSTIC",
                reason="supervised X positive relative test",
            )
        )
    )

    assert getattr(captured["axis"], "value", captured["axis"]) == "x"
    assert captured["steps"] == 100
    assert response["operation"] == "move-positive"
    assert response["terminal_status"]["rows"]["x"]["status"]["speed"]["speed"] == 0


def test_cross_axis_diagnostic_fails_before_hardware_access(monkeypatch):
    from src.bioxp import api

    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: (_ for _ in ()).throw(AssertionError("readiness accessed")))
    monkeypatch.setattr(api, "_get_tester", lambda: (_ for _ in ()).throw(AssertionError("hardware accessed")))

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            api.motion_diagnostics_execute(
                api.AxisDiagnosticExecuteRequest(
                    axis="x",
                    operation="open",
                    operator_ack="RUN_AXIS_DIAGNOSTIC",
                    reason="invalid contract probe",
                )
            )
        )
    assert exc_info.value.status_code == 422


def test_diagnostic_missing_literal_success_is_rejected(monkeypatch):
    from src.bioxp import api

    class Tester:
        def _motion_oem_axis_profile(self, axis, startup=False):
            return {"board": 3, "motor": 0}

        def motor_axis_status(self, board, motor):
            return {"ok": True, "speed": {"speed": 0}}

        def motor_query_home_switch(self, board, motor):
            return {"ok": True, "active": False}

    async def fake_relative(_req):
        return {"detail": "missing literal success"}

    async def immediate(_label, fn, **_kwargs):
        return fn()

    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    monkeypatch.setattr(api, "_get_tester", lambda: Tester())
    monkeypatch.setattr(api, "move_axis_relative", fake_relative)
    monkeypatch.setattr(api, "_run_blocking", immediate)

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            api.motion_diagnostics_execute(
                api.AxisDiagnosticExecuteRequest(
                    axis="x",
                    operation="move-positive",
                    operator_ack="RUN_AXIS_DIAGNOSTIC",
                    reason="fail-closed result probe",
                )
            )
        )

    assert exc_info.value.status_code == 409
    assert exc_info.value.detail["ok"] is False
    assert exc_info.value.detail["physical_effect_verified"] is False
