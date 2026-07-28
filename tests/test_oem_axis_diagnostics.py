from __future__ import annotations

import asyncio
import threading
from types import SimpleNamespace
from typing import Any, cast

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


def test_diagnostic_stop_preempts_inflight_normal_tester_lock(monkeypatch):
    from src.bioxp import api

    stop_called = threading.Event()

    class Tester:
        def _motion_oem_axis_profile(self, axis, startup=False):
            assert axis == "x"
            return {"board": 3, "motor": 0}

        def motor_stop(self, board, motor):
            stop_called.set()
            return {"ok": True, "board": board, "motor": motor}

        def motor_axis_status(self, board, motor):
            return {"ok": True, "speed": {"speed": 0}}

        def motor_query_home_switch(self, board, motor):
            return {"ok": True, "active": False}

    monkeypatch.setattr(api, "_get_tester", lambda: Tester())

    async def scenario():
        lock_held = asyncio.Event()
        release_normal_lane = asyncio.Event()

        async def hold_normal_lane():
            async with api._tester_lock:
                lock_held.set()
                await release_normal_lane.wait()

        holder = asyncio.create_task(hold_normal_lane())
        await lock_held.wait()
        stop_task = asyncio.create_task(
            api.motion_diagnostics_stop(
                api.AxisDiagnosticStopRequest(
                    axis="x",
                    operator_ack="STOP_AXIS",
                    reason="preempt blocked diagnostic",
                    operator="independent safety regression",
                )
            )
        )
        called_before_release = await asyncio.to_thread(stop_called.wait, 0.15)
        release_normal_lane.set()
        await holder
        result = await stop_task
        return called_before_release, result

    called_before_release, result = asyncio.run(scenario())
    assert called_before_release is True
    assert result["verified_stopped"] is True


def test_diagnostic_stop_holds_connection_lease_against_release(monkeypatch):
    from src.bioxp import api

    stop_entered = threading.Event()
    allow_stop = threading.Event()

    class Tester:
        disconnected = False

        def _motion_oem_axis_profile(self, axis, startup=False):
            return {"board": 3, "motor": 0}

        def motor_stop(self, board, motor):
            stop_entered.set()
            assert allow_stop.wait(1.0)
            assert self.disconnected is False
            return {"ok": True, "board": board, "motor": motor}

        def motor_axis_status(self, board, motor):
            return {"ok": True, "speed": {"speed": 0}}

        def motor_query_home_switch(self, board, motor):
            return {"ok": True, "active": False}

        def _disconnect(self):
            self.disconnected = True

    tester = Tester()
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_pipette_transport", None)

    async def scenario():
        stop_task = asyncio.create_task(
            api.motion_diagnostics_stop(
                api.AxisDiagnosticStopRequest(
                    axis="x",
                    operator_ack="STOP_AXIS",
                    reason="connection lease regression",
                    operator="independent safety regression",
                )
            )
        )
        assert await asyncio.to_thread(stop_entered.wait, 1.0)
        release_task = asyncio.create_task(
            api.maintenance_usb_release(cast(Any, SimpleNamespace(client=SimpleNamespace(host="127.0.0.1"))))
        )
        await asyncio.sleep(0.05)
        release_completed_while_stop_blocked = release_task.done()
        allow_stop.set()
        stop_result = await stop_task
        release_result = await release_task
        return release_completed_while_stop_blocked, stop_result, release_result

    release_completed, stop_result, release_result = asyncio.run(scenario())
    assert release_completed is False
    assert stop_result["verified_stopped"] is True
    assert release_result["usb_owner"] == "released"
    assert tester.disconnected is True


def test_interrupt_timeout_keeps_connection_lease_until_worker_exits(monkeypatch):
    from src.bioxp import api

    worker_entered = threading.Event()
    allow_worker_exit = threading.Event()

    class Tester:
        disconnected = False

        def _disconnect(self):
            self.disconnected = True

    tester = Tester()
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_pipette_transport", None)

    def blocked_interrupt(active_tester):
        assert active_tester is tester
        worker_entered.set()
        assert allow_worker_exit.wait(1.0)
        assert tester.disconnected is False
        return {"ok": True}

    async def scenario():
        with pytest.raises(HTTPException) as exc_info:
            await api._run_safety_interrupt_blocking(
                "timeout lease regression",
                blocked_interrupt,
                timeout_s=0.05,
            )
        assert exc_info.value.status_code == 504
        assert await asyncio.to_thread(worker_entered.wait, 1.0)
        release_task = asyncio.create_task(
            api.maintenance_usb_release(cast(Any, SimpleNamespace(client=SimpleNamespace(host="127.0.0.1"))))
        )
        await asyncio.sleep(0.05)
        release_completed_before_worker_exit = release_task.done()
        allow_worker_exit.set()
        release_result = await release_task
        return release_completed_before_worker_exit, release_result

    release_completed, release_result = asyncio.run(scenario())
    assert release_completed is False
    assert release_result["usb_owner"] == "released"
    assert tester.disconnected is True


def test_cancelled_maintenance_release_retains_leases_until_disconnect_exits(monkeypatch):
    from src.bioxp import api

    disconnect_entered = threading.Event()
    allow_disconnect_exit = threading.Event()
    stop_called = threading.Event()

    class Tester:
        def _disconnect(self):
            disconnect_entered.set()
            assert allow_disconnect_exit.wait(1.0)

        def _motion_oem_axis_profile(self, axis, startup=False):
            return {"board": 3, "motor": 0}

        def motor_stop(self, board, motor):
            stop_called.set()
            return {"ok": True, "board": board, "motor": motor}

        def motor_axis_status(self, board, motor):
            return {"ok": True, "speed": {"speed": 0}}

        def motor_query_home_switch(self, board, motor):
            return {"ok": True, "active": False}

    tester = Tester()
    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_pipette_transport", None)

    async def scenario():
        release_task = asyncio.create_task(
            api.maintenance_usb_release(cast(Any, SimpleNamespace(client=SimpleNamespace(host="127.0.0.1"))))
        )
        assert await asyncio.to_thread(disconnect_entered.wait, 1.0)
        release_task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await release_task
        assert api._tester_lock.locked() is True
        assert api._tester_transition_lock.locked() is True

        stop_task = asyncio.create_task(
            api.motion_diagnostics_stop(
                api.AxisDiagnosticStopRequest(
                    axis="x",
                    operator_ack="STOP_AXIS",
                    reason="disconnect cancellation lease regression",
                    operator="independent safety regression",
                )
            )
        )
        await asyncio.sleep(0.05)
        assert stop_called.is_set() is False
        assert stop_task.done() is False

        allow_disconnect_exit.set()
        with pytest.raises(HTTPException) as exc_info:
            await stop_task
        assert exc_info.value.status_code == 503
        for _ in range(50):
            if not api._tester_lock.locked() and not api._tester_transition_lock.locked():
                break
            await asyncio.sleep(0.01)
        assert api._tester is None

    asyncio.run(scenario())


def test_cancelled_maintenance_reconnect_cannot_construct_a_second_owner(monkeypatch):
    from src.bioxp import api

    first_constructor_entered = threading.Event()
    allow_first_constructor_exit = threading.Event()
    constructor_calls: list[object] = []

    class Tester:
        pass

    def construct(*, alt):
        candidate = Tester()
        constructor_calls.append(candidate)
        if len(constructor_calls) == 1:
            first_constructor_entered.set()
            assert allow_first_constructor_exit.wait(1.0)
        return candidate

    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "BioXpTester", construct)
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda *, shared_usb: {"tester": shared_usb})

    request = cast(Any, SimpleNamespace(client=SimpleNamespace(host="127.0.0.1")))

    async def scenario():
        first = asyncio.create_task(api.maintenance_usb_reconnect(request))
        assert await asyncio.to_thread(first_constructor_entered.wait, 1.0)
        first.cancel()
        with pytest.raises(asyncio.CancelledError):
            await first
        assert api._tester_lock.locked() is True
        assert api._tester_transition_lock.locked() is True

        second = asyncio.create_task(api.maintenance_usb_reconnect(request))
        await asyncio.sleep(0.05)
        assert len(constructor_calls) == 1
        assert second.done() is False

        allow_first_constructor_exit.set()
        second_result = await second
        assert len(constructor_calls) == 1
        assert api._tester is constructor_calls[0]
        assert second_result["usb_owner"] == "service"

    asyncio.run(scenario())


def test_public_reconnect_holds_transition_lease_against_stop(monkeypatch):
    from src.bioxp import api

    reconnect_entered = threading.Event()
    allow_reconnect_exit = threading.Event()
    stop_called = threading.Event()

    class Tester:
        def reconnect(self):
            reconnect_entered.set()
            assert allow_reconnect_exit.wait(1.0)
            return {"ok": True}

        def _motion_oem_axis_profile(self, axis, startup=False):
            return {"board": 3, "motor": 0}

        def motor_stop(self, board, motor):
            stop_called.set()
            return {"ok": True, "board": board, "motor": motor}

        def motor_axis_status(self, board, motor):
            return {"ok": True, "speed": {"speed": 0}}

        def motor_query_home_switch(self, board, motor):
            return {"ok": True, "active": False}

    tester = Tester()
    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda *, shared_usb: {"tester": shared_usb})
    monkeypatch.setattr(api, "_status_payload", lambda: {"available": True})

    async def scenario():
        reconnect_task = asyncio.create_task(api.reconnect_runtime())
        assert await asyncio.to_thread(reconnect_entered.wait, 1.0)
        stop_task = asyncio.create_task(
            api.motion_diagnostics_stop(
                api.AxisDiagnosticStopRequest(
                    axis="x",
                    operator_ack="STOP_AXIS",
                    reason="public reconnect transition lease regression",
                    operator="independent safety regression",
                )
            )
        )
        await asyncio.sleep(0.05)
        assert stop_called.is_set() is False
        assert stop_task.done() is False
        allow_reconnect_exit.set()
        reconnect_result = await reconnect_task
        stop_result = await stop_task
        assert reconnect_result["ok"] is True
        assert stop_result["verified_stopped"] is True

    asyncio.run(scenario())


@pytest.mark.parametrize("route_name", ["motion", "thermal", "chiller"])
def test_transport_replacing_hard_reset_holds_transition_lease_against_stop(monkeypatch, route_name):
    from src.bioxp import api

    reset_entered = threading.Event()
    allow_reset_exit = threading.Event()
    stop_called = threading.Event()

    class Tester:
        def _hard_reset(self):
            reset_entered.set()
            assert allow_reset_exit.wait(1.0)
            return {"ok": True}

        def motor_hard_reset(self, *, rounds):
            return self._hard_reset()

        def thermal_hard_reset(self):
            return self._hard_reset()

        def chiller_hard_reset(self):
            return self._hard_reset()

        def _motion_oem_axis_profile(self, axis, startup=False):
            return {"board": 3, "motor": 0}

        def motor_stop(self, board, motor):
            stop_called.set()
            return {"ok": True, "board": board, "motor": motor}

        def motor_axis_status(self, board, motor):
            return {"ok": True, "speed": {"speed": 0}}

        def motor_query_home_switch(self, board, motor):
            return {"ok": True, "active": False}

    class ReferenceStore:
        def mark_desynced(self, command):
            return {"ok": True}

        def snapshot(self, axes):
            return {"axes": [axis.value for axis in axes]}

    tester = Tester()
    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "_reference_state_store", ReferenceStore())
    monkeypatch.setattr(api, "_ownership_changed", lambda **kwargs: None)
    monkeypatch.setattr(api, "_mark_post_maintenance_motion_block", lambda **kwargs: {})

    async def scenario():
        if route_name == "motion":
            reset_task = asyncio.create_task(api.motion_hard_reset(api.MotionHardResetRequest(rounds=1)))
        elif route_name == "thermal":
            reset_task = asyncio.create_task(api.thermal_hard_reset())
        else:
            reset_task = asyncio.create_task(api.chiller_hard_reset())

        assert await asyncio.to_thread(reset_entered.wait, 1.0)
        stop_task = asyncio.create_task(
            api.motion_diagnostics_stop(
                api.AxisDiagnosticStopRequest(
                    axis="x",
                    operator_ack="STOP_AXIS",
                    reason=f"{route_name} hard reset transition lease regression",
                    operator="independent safety regression",
                )
            )
        )
        await asyncio.sleep(0.05)
        stop_completed_during_reset = stop_task.done()
        allow_reset_exit.set()
        reset_result = await reset_task
        stop_result = await stop_task
        return stop_completed_during_reset, reset_result, stop_result

    stop_completed, reset_result, stop_result = asyncio.run(scenario())
    assert stop_completed is False
    assert reset_result["ok"] is True
    assert stop_result["verified_stopped"] is True
    assert stop_called.is_set() is True


def test_maintenance_reconnect_disconnects_candidate_when_wrapper_build_fails(monkeypatch):
    from src.bioxp import api

    candidates = []

    class Tester:
        def __init__(self, *, alt):
            self.disconnected = False
            candidates.append(self)

        def _disconnect(self):
            self.disconnected = True

    def fail_wrapper_build(*, shared_usb):
        raise RuntimeError("wrapper construction failed")

    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "BioXpTester", Tester)
    monkeypatch.setattr(api, "build_default_pipette_transport", fail_wrapper_build)
    request = cast(Any, SimpleNamespace(client=SimpleNamespace(host="127.0.0.1")))

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(api.maintenance_usb_reconnect(request))

    assert exc_info.value.status_code == 503
    assert len(candidates) == 1
    assert candidates[0].disconnected is True
    assert api._tester is None
    assert api._pipette_transport is None


def test_failed_reconnect_cleanup_quarantines_owner_and_blocks_retry(monkeypatch):
    from src.bioxp import api

    candidates = []

    class Tester:
        def __init__(self, *, alt):
            candidates.append(self)

        def _disconnect(self):
            raise RuntimeError("USB release failed")

    def fail_wrapper_build(*, shared_usb):
        raise RuntimeError("wrapper construction failed")

    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None, raising=False)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "BioXpTester", Tester)
    monkeypatch.setattr(api, "build_default_pipette_transport", fail_wrapper_build)
    request = cast(Any, SimpleNamespace(client=SimpleNamespace(host="127.0.0.1")))

    with pytest.raises(HTTPException) as first_error:
        asyncio.run(api.maintenance_usb_reconnect(request))
    assert first_error.value.status_code == 503
    assert len(candidates) == 1
    assert api._tester is None
    assert api._tester_quarantine is candidates[0]

    with pytest.raises(HTTPException) as getter_error:
        api._get_tester()
    assert getter_error.value.status_code == 503

    with pytest.raises(HTTPException) as retry_error:
        asyncio.run(api.maintenance_usb_reconnect(request))
    assert retry_error.value.status_code == 503
    assert len(candidates) == 1
    assert api._tester_quarantine is candidates[0]
