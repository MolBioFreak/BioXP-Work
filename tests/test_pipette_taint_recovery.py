"""Regression tests for the 2026-09-21 pipette wake / completion-taint recovery fix.

Operator directive: the OEM wake-carrying init is the correct behavior, and the
late hardening had placed a permanent blocker on top of it.  This suite pins the
removal contract:

- a completion taint is RECORD-ONLY: it never refuses a retry, and the record
  clears itself on the next valid completion;
- a dead (past-deadline) completion registration is evicted instead of blocking
  a fresh attempt; a genuinely live registration still excludes a concurrent one;
- the reinitialize recovery path carries the OEM wake frame for channels the
  software does not believe initialized (and for every channel when forced by
  the readiness self-heal);
- a silent tip readback resets the software-initialized belief so the next
  init attempt wakes instead of sending WR alone;
- the bounded readiness self-heal escalates on a streak of all-silent sweeps,
  exactly once per cooldown window, and never on a replied sweep.
"""
from __future__ import annotations

import time

import pytest

from bioxp.novo_router import NovoFrame, NovoRouter, NovoRouterError
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport


def _make_router() -> NovoRouter:
    return NovoRouter(
        ep_in=object(),
        ep_out=object(),
        decode=lambda payload: payload,
        clock=time.monotonic,
    )


def _rx_id(channel: int, family: int = 0) -> int:
    return 0x400 | 0x100 | family | (channel << 3)


def _terminal_frame(channel: int, family: int = 0, *, received_at: float) -> NovoFrame:
    return NovoFrame(
        arbitration_id=_rx_id(channel, family),
        dlc=2,
        data=bytes((0x20, 0x20)),
        raw=bytes((0x20, 0x20)),
        received_at=received_at,
        classification="pipette",
    )


def _ack_frame(channel: int, family: int = 0, *, received_at: float) -> NovoFrame:
    return NovoFrame(
        arbitration_id=_rx_id(channel, family),
        dlc=0,
        data=b"",
        raw=b"",
        received_at=received_at,
        classification="pipette",
    )


def test_completion_timeout_records_taint_but_never_refuses_a_retry():
    router = _make_router()
    token = router.prepare_pipette_completion(
        0, 0.05, command_family=0, command_name="pipette_initialize", expected_rx_id=_rx_id(0)
    )
    router.bind_pipette_completion(0, owner_token=token, transaction_id="tx-1", tx_started_at=time.monotonic())
    outcome = router.wait_pipette_completion(0, 0.10, owner_token=token)
    assert outcome["ok"] is False and outcome["outcome"] == "timeout"
    taint = router.pipette_completion_taint(0)
    assert isinstance(taint, dict) and taint.get("reason") == "completion_timeout"

    # The blocker removal: with the taint recorded, a fresh attempt is admitted.
    retry_token = router.prepare_pipette_completion(
        0, 0.05, command_family=0, command_name="pipette_initialize", expected_rx_id=_rx_id(0)
    )
    assert isinstance(retry_token, str) and retry_token and retry_token != token


def test_valid_completion_clears_the_taint_record():
    router = _make_router()
    token = router.prepare_pipette_completion(
        1, 0.02, command_family=0, command_name="pipette_initialize", expected_rx_id=_rx_id(1)
    )
    router.bind_pipette_completion(1, owner_token=token, transaction_id="tx-1", tx_started_at=time.monotonic())
    assert router.wait_pipette_completion(1, 0.05, owner_token=token)["ok"] is False
    assert router.pipette_completion_taint(1) is not None

    started = time.monotonic()
    token2 = router.prepare_pipette_completion(
        1, 0.5, command_family=0, command_name="pipette_initialize", expected_rx_id=_rx_id(1)
    )
    router.bind_pipette_completion(1, owner_token=token2, transaction_id="tx-2", tx_started_at=started)
    router._dispatch(_ack_frame(1, received_at=started + 0.1))
    router._dispatch(_terminal_frame(1, received_at=started + 0.5))
    outcome = router.wait_pipette_completion(1, 0.5, owner_token=token2)
    assert outcome["ok"] is True, outcome
    assert router.pipette_completion_taint(1) is None


def test_dead_registration_is_evicted_not_blocking():
    router = _make_router()
    first = router.prepare_pipette_completion(
        2, 0.0, command_family=0, command_name="pipette_initialize", expected_rx_id=_rx_id(2)
    )
    time.sleep(0.01)
    second = router.prepare_pipette_completion(
        2, 5.0, command_family=0, command_name="pipette_initialize", expected_rx_id=_rx_id(2)
    )
    assert second and second != first


def test_live_registration_still_excludes_a_second_concurrent_op():
    router = _make_router()
    router.prepare_pipette_completion(
        3, 60.0, command_family=0, command_name="pipette_initialize", expected_rx_id=_rx_id(3)
    )
    with pytest.raises(NovoRouterError, match="already registered"):
        router.prepare_pipette_completion(
            3, 60.0, command_family=0, command_name="pipette_initialize", expected_rx_id=_rx_id(3)
        )


class _FakePipetteDriver:
    def __init__(self, channel: int) -> None:
        self.pipette_id = channel
        self.calls: list[str] = []

    def pipette_initialize(self, pressure_profile: str = "1R"):
        self.calls.append("pipette_initialize")
        return {"ok": True, "immediate_ack_received": True, "requested_pressure_profile": pressure_profile}

    def pipette_initiate_group(self):
        self.calls.append("pipette_initiate_group")
        return {"ok": True, "immediate_ack_received": True}

    def wait_pipette_initialization_completion(self, timeout_s):
        return {"ok": True, "channel": self.pipette_id}

    def query_status(self):
        return {"ok": True, "oem_process_error_code": 32, "oem_error_code": 0}


def _build_transport(initialized: list[bool]):
    drivers = [_FakePipetteDriver(channel) for channel in range(4)]
    transports = []
    for channel in range(4):
        transport = CanPipetteTransport(driver_factory=lambda d=drivers[channel]: d, pipette_id=channel)
        transport._initialized = bool(initialized[channel])
        transports.append(transport)
    four = FourPipetteTransport(transports, sleep=lambda _s: None)
    return four, drivers


def test_reinitialize_carries_wake_for_uninitialized_channels():
    transport, drivers = _build_transport([False, True, True, True])
    outcome = transport.reinitialize_pipette()
    assert outcome["ok"] is True
    assert drivers[0].calls == ["pipette_initialize"]
    for driver in drivers[1:]:
        assert driver.calls == ["pipette_initiate_group"]


def test_reinitialize_force_wake_sends_wake_for_every_channel():
    transport, drivers = _build_transport([True, True, True, True])
    outcome = transport.reinitialize_pipette(force_wake=True)
    assert outcome["ok"] is True
    for driver in drivers:
        assert driver.calls == ["pipette_initialize"]


def test_silent_tip_readback_resets_software_initialized_belief():
    class _Driver:
        def __init__(self, result):
            self._result = result

        def query_tip_status(self, **kwargs):
            return dict(self._result)

    silent = CanPipetteTransport(
        driver_factory=lambda: _Driver({"ok": False, "ack": {"received": False, "error": "ack_timeout"}}),
        pipette_id=0,
    )
    silent._initialized = True
    silent._safe_query_tip_status(silent._get_driver(), required=False)
    assert silent._initialized is False

    answering = CanPipetteTransport(
        driver_factory=lambda: _Driver({"ok": True, "tip_loaded": True, "ack": {"received": True}}),
        pipette_id=0,
    )
    answering._initialized = True
    answering._safe_query_tip_status(answering._get_driver(), required=False)
    assert answering._initialized is True


def test_readiness_silence_streak_escalates_once_per_cooldown(monkeypatch):
    from bioxp import api as api_module

    seen = []

    class _Transport:
        def reinitialize_pipette(self, *, force_wake: bool = False):
            seen.append(("reinitialize_pipette", force_wake))
            return {"ok": True, "outcome": "reinitialized"}

    async def _fake_run_pipette_operation(name, operation, *, get_transport, run_blocking, timeout_s=600.0,
                                          receipt_store=None, requested_inputs=None, preflight=None,
                                          runtime_binding=None):
        seen.append(("operation", name, requested_inputs))
        return operation(get_transport())

    monkeypatch.setattr(api_module, "run_pipette_operation", _fake_run_pipette_operation)
    monkeypatch.setattr(api_module, "_get_pipette_transport", lambda: _Transport())
    state = api_module._PIPETTE_READINESS_RECOVERY
    state.update({"silent_streak": 0, "last_attempt_at": None, "last_outcome": None})
    try:
        silent = {"channels": [{"channel": 0, "result": {"ok": False, "ack": {"received": False}}}]}
        replied = {"channels": [{"channel": 0, "result": {"ok": True, "ack": {"received": True}}}]}

        api_module._pipette_readiness_silent_sweep(replied)
        assert state["silent_streak"] == 0

        api_module._pipette_readiness_silent_sweep(silent)
        api_module._pipette_readiness_silent_sweep(silent)
        assert not any(entry[0] == "reinitialize_pipette" for entry in seen)
        assert state["silent_streak"] == 2

        api_module._pipette_readiness_silent_sweep(silent)  # third silent sweep -> one recovery
        assert ("reinitialize_pipette", True) in seen
        assert state["silent_streak"] == 0
        assert state["last_outcome"] == "reinitialized"

        api_module._pipette_readiness_silent_sweep(silent)
        api_module._pipette_readiness_silent_sweep(silent)
        api_module._pipette_readiness_silent_sweep(silent)  # cooldown holds the second attempt
        attempts = [entry for entry in seen if entry[0] == "reinitialize_pipette"]
        assert len(attempts) == 1
    finally:
        state.update({"silent_streak": 0, "last_attempt_at": None, "last_outcome": None})
