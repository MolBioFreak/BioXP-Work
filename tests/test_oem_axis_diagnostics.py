from __future__ import annotations

import pytest

from bioxp.oem_axis_diagnostics import (
    AXIS_DIAGNOSTIC_CATALOG,
    AxisDiagnosticContractError,
    diagnostic_catalog,
    resolve_axis_diagnostic,
)


def test_catalog_exposes_complete_finite_core_function_blocks() -> None:
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
    assert projection["axes"]["y"]["move-negative"]["steps"] == -100
    assert projection["axes"]["z"]["move-positive"]["steps"] == 100
    assert projection["axes"]["x"]["park-6000"]["position_steps"] == 6000
    assert projection["axes"]["g"]["commission-home"]["temporary_action_current_internal"] is True
    assert projection["axes"]["g"]["commission-home"]["required_idle_readback"] == {"run": 10, "standby": 10}


@pytest.mark.parametrize(
    ("axis", "operation", "executor", "value"),
    [
        ("x", "move-negative", "relative", -100),
        ("x", "move-positive", "relative", 100),
        ("y", "move-positive", "relative", 100),
        ("z", "move-negative", "relative", -100),
        ("x", "park-6000", "absolute", 6000),
        ("y", "home", "home", None),
        ("g", "commission-home", "gripper-commission-home", None),
        ("g", "open-wide", "gripper-open-wide", None),
        ("door", "close", "door-close", None),
        ("door", "stop", "stop", None),
    ],
)
def test_resolver_maps_only_robot_owned_oem_actions(axis: str, operation: str, executor: str, value: int | None) -> None:
    action = resolve_axis_diagnostic(axis, operation)
    assert action.axis == axis
    assert action.operation == operation
    assert action.executor == executor
    assert action.value == value


@pytest.mark.parametrize(
    ("axis", "operation"),
    [
        ("x", "open"),
        ("door", "move-positive"),
        ("g", "move-zero"),
        ("y", "park-6000"),
        ("raw-board-4", "home"),
        ("x", "set-current-31"),
    ],
)
def test_resolver_rejects_unknown_or_cross_axis_operations(axis: str, operation: str) -> None:
    with pytest.raises(AxisDiagnosticContractError):
        resolve_axis_diagnostic(axis, operation)


def test_robot_api_publishes_only_semantic_diagnostic_routes() -> None:
    from bioxp import api

    paths = {route.path for route in api.app.routes}
    assert "/motion/diagnostics/catalog" in paths
    assert "/motion/diagnostics/status" in paths
    assert "/motion/diagnostics/execute" in paths
    assert "/motion/diagnostics/stop" in paths


def test_robot_diagnostic_models_reject_raw_values_and_wrong_ack() -> None:
    from pydantic import ValidationError
    from bioxp.api import AxisDiagnosticExecuteRequest, AxisDiagnosticStopRequest

    with pytest.raises(ValidationError):
        AxisDiagnosticExecuteRequest(
            axis="x",
            operation="move-positive",
            operator_ack="RUN_AXIS_DIAGNOSTIC",
            reason="test",
            steps=999999,
        )
    with pytest.raises(ValidationError):
        AxisDiagnosticExecuteRequest(
            axis="x",
            operation="move-positive",
            operator_ack="wrong",
            reason="test",
        )
    with pytest.raises(ValidationError):
        AxisDiagnosticStopRequest(axis="x", operator_ack="wrong", reason="test")


def test_x_positive_route_dispatches_fixed_relative_steps(monkeypatch) -> None:
    import asyncio
    from bioxp import api

    class Tester:
        def _motion_oem_axis_profile(self, axis, startup=True):
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

    monkeypatch.setattr(api, "_require_motion_route_ready", lambda _req: None)
    monkeypatch.setattr(api, "_get_tester", lambda: Tester())
    monkeypatch.setattr(api, "move_axis_relative", fake_relative)
    monkeypatch.setattr(api, "_run_blocking", immediate)

    response = asyncio.run(api.motion_diagnostics_execute(api.AxisDiagnosticExecuteRequest(
        axis="x",
        operation="move-positive",
        operator_ack="RUN_AXIS_DIAGNOSTIC",
        reason="supervised X positive relative test",
    )))

    assert getattr(captured["axis"], "value", captured["axis"]) == "x"
    assert captured["steps"] == 100
    assert response["operation"] == "move-positive"
    assert response["terminal_status"]["rows"]["x"]["status"]["speed"]["speed"] == 0


def test_incompatible_route_fails_before_tester_access(monkeypatch) -> None:
    import asyncio
    from fastapi import HTTPException
    from bioxp import api

    monkeypatch.setattr(api, "_require_motion_route_ready", lambda _req: None)
    monkeypatch.setattr(api, "_get_tester", lambda: (_ for _ in ()).throw(AssertionError("hardware accessed")))

    with pytest.raises(HTTPException) as exc:
        asyncio.run(api.motion_diagnostics_execute(api.AxisDiagnosticExecuteRequest(
            axis="x",
            operation="open",
            operator_ack="RUN_AXIS_DIAGNOSTIC",
            reason="invalid contract probe",
        )))
    assert exc.value.status_code == 422


def test_gripper_stop_restores_idle_and_verifies_stop(monkeypatch) -> None:
    import asyncio
    from bioxp import api

    events: list[str] = []

    class Tester:
        def _motion_oem_axis_profile(self, axis, startup=True):
            assert axis == "g"
            return {"board": 4, "motor": 2}

        def motor_stop(self, board, motor):
            assert (board, motor) == (4, 2)
            events.append("stop")
            return {"ok": True}

    async def immediate(_label, fn, **_kwargs):
        return fn()

    monkeypatch.setattr(api, "_get_tester", lambda: Tester())
    monkeypatch.setattr(api, "_run_blocking", immediate)
    monkeypatch.setattr(api, "restore_gripper_idle_current", lambda _tester, **_kwargs: events.append("restore") or {"ok": True})
    monkeypatch.setattr(api, "gripper_status", lambda _tester: {
        "ok": True,
        "speed": {"speed": 0},
        "current": {"run_current_param6": 10, "standby_current_param7": 10},
    })

    response = asyncio.run(api.motion_diagnostics_stop(api.AxisDiagnosticStopRequest(
        axis="g",
        operator_ack="STOP_AXIS",
        reason="operator stop",
    )))

    assert events == ["stop", "restore"]
    assert response["ok"] is True
    assert response["terminal_status"]["rows"]["g"]["current"] == {"run_current_param6": 10, "standby_current_param7": 10}
