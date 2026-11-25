"""Regressions for the second-review follow-ups on the handler repairs.

1. A fan tick retired by reconnect must not publish its chiller sample.
2. A shutdown refused because command workers are still active must not close
   the stores or readers those workers use.
3. A Stop that never reached the wire is not reported or journaled as delivered.
"""
from __future__ import annotations

import asyncio
import threading
from types import SimpleNamespace

import pytest
from fastapi import FastAPI

from bioxp import api
from bioxp.usb_driver import BioXpTester
from tests.test_handler_repair_lifespan import _prep_lifespan_env
from tests.test_handler_repair_tips_and_fans import _make_fake_tester


# ---------------------------------------------------------------------------
# 1. Chiller sample from a retired tick
# ---------------------------------------------------------------------------


def _chiller_tester(monkeypatch):
    class FakeTimer:
        def __init__(self, interval, fn, args=()):
            self.fn, self.args, self.daemon = fn, args, False

        def start(self):
            pass

        def cancel(self):
            pass

    monkeypatch.setattr(threading, "Timer", FakeTimer)
    tester = _make_fake_tester()
    monkeypatch.setattr(tester, "send_tmcl_retry", lambda *a, **k: {"status": 100, "value": 30000})
    tester._oem_fan_service_start(tester.BOARD_CHILLER, 0)
    service = tester._oem_fan_services()[(tester.BOARD_CHILLER, 0)]
    guard = ((tester.BOARD_CHILLER, 0), service["generation"], service["token"])
    return tester, guard


def test_retired_chiller_tick_does_not_overwrite_replacement_chiller_state(monkeypatch):
    tester, old_guard = _chiller_tester(monkeypatch)

    def send_then_reconnect(*args, **kwargs):
        # The old tick has its reading; a reconnect lands before it applies it.
        tester._oem_transport_generation += 1
        replacement = tester._oem_thermal_state(tester.BOARD_CHILLER, 0)
        replacement.update(target=25.0, reached=False, chiller_current=30.0, start=None, elapsed=0)
        return {"status": 100, "value": 24000}

    monkeypatch.setattr(tester, "send_tmcl_retry", send_then_reconnect)
    tester._oem_fan_read(tester.BOARD_CHILLER, 143, 0, 0, 30.0, tick_guard=old_guard)

    state = tester._oem_thermal_state(tester.BOARD_CHILLER, 0)
    assert state["chiller_current"] == 30.0
    assert state["reached"] is False


def test_active_chiller_tick_still_publishes_its_sample(monkeypatch):
    tester, guard = _chiller_tester(monkeypatch)
    state = tester._oem_thermal_state(tester.BOARD_CHILLER, 0)
    state.update(target=25.0, reached=False, chiller_current=30.0, start=None, elapsed=0)
    monkeypatch.setattr(tester, "send_tmcl_retry", lambda *a, **k: {"status": 100, "value": 24000})

    tester._oem_fan_read(tester.BOARD_CHILLER, 143, 0, 0, 30.0, tick_guard=guard)

    assert state["chiller_current"] == 24.0
    assert state["reached"] is True


# ---------------------------------------------------------------------------
# 2. Refused shutdown keeps worker dependencies open
# ---------------------------------------------------------------------------


def test_refused_shutdown_keeps_history_reader_operator_resources_and_calibration_store(tmp_path, monkeypatch):
    _root, events, cal_stores = _prep_lifespan_env(tmp_path, monkeypatch, verified_release=False)
    test_app = FastAPI(lifespan=api.lifespan)
    monkeypatch.setattr(api, "app", test_app)
    restore = {}

    async def run_scenario():
        with pytest.raises(RuntimeError, match="command workers remain active"):
            async with api.lifespan(test_app):
                plane = test_app.state.operator_command_plane
                history = test_app.state.operator_history_reader
                lifecycle = test_app.state.operator_control_plane_lifecycle
                restore.update(plane=plane, history=history, lifecycle=lifecycle,
                               stop=plane.stop, history_close=history.close,
                               lifecycle_close=lifecycle.close)

                def refuse_stop():
                    events.append("command_plane_stop_refused")
                    raise RuntimeError("worker still active")

                def recorded(name, original):
                    def close():
                        events.append(name)
                        return original()
                    return close

                monkeypatch.setattr(plane, "stop", refuse_stop)
                monkeypatch.setattr(history, "close", recorded("history_reader_close", history.close))
                monkeypatch.setattr(lifecycle, "close", recorded("operator_lifecycle_close", lifecycle.close))

    try:
        asyncio.run(run_scenario())
        assert "command_plane_stop_refused" in events
        assert "history_reader_close" not in events
        assert "operator_lifecycle_close" not in events
        assert "calibration_store_close" not in events
        assert "usb_disconnect" not in events
        assert cal_stores[0].closed == 0
        assert test_app.state.receiver_audit_shutdown["reason"] == "command_workers_active"
    finally:
        if restore:
            restore["stop"]()
            restore["lifecycle_close"]()
            restore["history_close"]()


# ---------------------------------------------------------------------------
# 3. Stop delivered only when a frame was written
# ---------------------------------------------------------------------------


def _deferred_stop_without_wire_write(delivery_ok: bool, release: threading.Event):
    def interrupt(_tester):
        # Absent board (ok, nothing to send) or failure before the first write.
        def reconcile():
            release.wait(timeout=5.0)
            return {"ok": True, "reconciled": True}

        reconcile.delivery_result = {
            "ok": delivery_ok,
            "axis": "z",
            "intent": "stop",
            "delivery_completed": True,
            "physical_effect_verified": False,
        }
        return reconcile

    return interrupt


@pytest.mark.parametrize(
    ("delivery_ok", "expected_phase"),
    [(True, "delivery_attempted"), (False, "failed")],
    ids=["absent_board_noop", "failed_before_first_write"],
)
def test_stop_without_wire_write_is_not_reported_or_journaled_as_delivered(monkeypatch, delivery_ok, expected_phase):
    monkeypatch.setattr(api, "_tester", SimpleNamespace())
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_INTERRUPT_RECONCILIATION_DEADLINE_S", 0.1)
    phases = []
    monkeypatch.setattr(api, "_journal_interrupt_phase", lambda phase, **kw: phases.append((phase, kw)))
    release = threading.Event()

    async def run():
        try:
            return await api._run_safety_interrupt_blocking(
                "test stop no wire write",
                _deferred_stop_without_wire_write(delivery_ok, release),
                timeout_s=5.0,
                surface=f"z-nowire-{expected_phase}",
            )
        finally:
            release.set()

    try:
        out = asyncio.run(run())
    finally:
        release.set()

    assert out["delivered"] is False
    assert out["wire_write_observed"] is False
    assert out["reconciliation_pending"] is True
    assert out["connection_lease_held"] is False
    assert out["ok"] is delivery_ok
    names = [phase for phase, _ in phases]
    assert "delivered" not in names
    assert expected_phase in names
    assert "reconciliation_pending" in names
