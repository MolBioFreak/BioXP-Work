"""Regression tests for the 2026-09-20 warm-lock / reconnect incident.

Covers:
- a served operator view is never turned cold by a later failed refresh;
- passive provider reads report busy instead of stalling on the authority lock;
- the deck-readiness tip probe fails fast and aborts the sweep on the first
  shared-bus ack timeout.
"""
from __future__ import annotations

import asyncio
import threading
import time

import pytest

from bioxp.operator_controls import _OperatorPollCache, _PASSIVE_OPERATOR_POLL
from bioxp.oem_serial206_initialization import (
    ProviderAuthorityBusy,
    Serial206OemInitializationProvider,
    _MutationPriorityRLock,
)
from bioxp.pipette.models import PipetteCommandError
from bioxp.pipette.transport import FourPipetteTransport, _result_has_ack_timeout


def test_served_view_stays_served_after_refresh_failure():
    cache = _OperatorPollCache()
    attempts = {"count": 0}

    async def dashboard_view():
        attempts["count"] += 1
        if attempts["count"] == 1:
            return {"ownership_generation": 1, "actions": []}
        raise RuntimeError("refresh exploded")

    poll = cache.wrap(dashboard_view)
    try:
        first = asyncio.run(poll())
        assert first.get("ownership_generation") == 1

        deadline = time.monotonic() + 5.0
        while attempts["count"] < 2 and time.monotonic() < deadline:
            served = asyncio.run(poll())
            assert served.get("ownership_generation") == 1
            time.sleep(0.05)
        assert attempts["count"] >= 2, "the failing refresh never ran"

        # The cached body must keep being served; a failed refresh must never
        # 503 a view that was served before.
        served = asyncio.run(poll())
        assert served.get("ownership_generation") == 1
    finally:
        cache.close()


def test_result_has_ack_timeout_nested():
    assert _result_has_ack_timeout({"ack": {"error": "ack_timeout"}, "ok": False})
    assert not _result_has_ack_timeout({"ack": {"error": "completion_not_verified"}})
    assert _result_has_ack_timeout({"a": [{"b": {"error": "ack_timeout"}}]})


def test_passive_projection_scope_reports_busy_instead_of_waiting():
    provider = object.__new__(Serial206OemInitializationProvider)
    provider._lock = _MutationPriorityRLock()
    provider.state_store = None

    held = threading.Event()
    release = threading.Event()

    def holder():
        provider._lock.acquire()
        held.set()
        release.wait(5.0)
        provider._lock.release()

    thread = threading.Thread(target=holder)
    thread.start()
    try:
        assert held.wait(5.0)
        token = _PASSIVE_OPERATOR_POLL.set(True)
        try:
            with pytest.raises(ProviderAuthorityBusy):
                with provider.projection_scope():
                    pass
        finally:
            _PASSIVE_OPERATOR_POLL.reset(token)
    finally:
        release.set()
        thread.join(5.0)


def test_query_tip_status_all_fails_fast_on_first_ack_timeout():
    class FakeChannel:
        def __init__(self, result):
            self._result = result
            self._tip_loaded = False
            self.calls = []

        def _get_driver(self):
            return object()

        def _safe_query_tip_status(self, driver, **kwargs):
            self.calls.append(kwargs)
            return self._result

    timeout_channel = FakeChannel({"ok": False, "error": "ack_timeout", "ascii_command": "?31"})
    healthy_channel = FakeChannel({
        "ok": True, "semantic_ok": True, "hardware_truth_level": "hardware_query",
        "tip_loaded": False, "source_return": 2,
    })

    transport = object.__new__(FourPipetteTransport)
    transport._transports = [timeout_channel, healthy_channel]
    transport._transaction_lock = threading.RLock()

    with pytest.raises(PipetteCommandError) as exc_info:
        transport.query_tip_status_all(response_timeout_s=3.0, abort_after_first_ack_timeout=True)

    details = exc_info.value.details
    assert details["sweep_aborted_after_first_ack_timeout"] is True
    assert "ack_timeout" in repr(details["observed_channels"])
    # The sweep must stop at the first timed-out channel: no second query.
    assert healthy_channel.calls == []
    assert timeout_channel.calls and timeout_channel.calls[0]["response_timeout_s"] == 3.0
