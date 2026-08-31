from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from typing import Any, cast

import pytest

from src.bioxp.api import PipetteAspirateRequest, PipetteDispenseRequest, _pipette_forceabort_latched
from src.bioxp.can_driver import BioXpCanDriver
from src.bioxp.pipette.models import (
    PipetteAspirateCommand,
    PipetteDispenseCommand,
    PipetteMixCommand,
)
from src.bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport


class Child:
    def __init__(self, channel: int, events: list[tuple[Any, ...]], *, level: float = 20.0, speed: float = 10.0):
        self.channel = channel
        self.events = events
        self._initialized = True
        self._top_speed = speed
        self._liquid_level_ul = level
        self._last_tip_status = None
        self._reader_generation = 1
        self._driver = SimpleNamespace()

    def _get_driver(self):
        return self._driver

    def aspirate(self, command, *, wait_for_completion, verify_tip):
        self.events.append(("aspirate", self.channel, wait_for_completion, verify_tip))
        return {"ok": True, "delivery_verified": True, "controller_acknowledged": True, "completion_verified": False, "volume_ul": command.volume_ul}

    def dispense(self, command, *, wait_for_completion, verify_tip):
        self.events.append(("dispense", self.channel, wait_for_completion, verify_tip))
        return {"ok": True, "delivery_verified": True, "controller_acknowledged": True, "completion_verified": False, "volume_ul": command.volume_ul, "dispense_type": command.dispense_type}

    def mix(self, command, *, wait_for_completion, verify_tip):
        self.events.append(("mix", self.channel, wait_for_completion, verify_tip))
        return {"ok": True, "delivery_verified": True, "controller_acknowledged": True, "completion_verified": True, "cycles": []}

    def dispense_all(self, *, wait_for_completion, verify_tip):
        self.events.append(("dispense_all", self.channel, wait_for_completion, verify_tip))
        return {"ok": True, "delivery_verified": True, "controller_acknowledged": True, "completion_verified": False, "previous_liquid_level_ul": self._liquid_level_ul}

    def set_top_speed(self, value):
        self.events.append(("speed", self.channel, float(value)))
        self._top_speed = float(value)
        return {"ok": True}

    def wait_for_completion(self, timeout_s):
        self.events.append(("wait", self.channel, timeout_s))
        return {"ok": True, "outcome": "completion"}

    def apply_completed_effect(self, operation, result):
        if operation in {"dispense", "dispense_all"}:
            self._liquid_level_ul = 0.0 if operation == "dispense_all" else max(0.0, self._liquid_level_ul - float(result.get("volume_ul", 0.0)))
        elif operation == "aspirate":
            self._liquid_level_ul += float(result.get("volume_ul", 0.0))
        return {"state_reconciled": True, "state_reconciliation_source": "completion"}

    def terminate(self, command=None, *, wait_for_completion=False):
        self.events.append(("terminate", self.channel, wait_for_completion))
        return {"ok": True, "delivery_verified": True, "controller_acknowledged": True, "completion_verified": False}


def owner(*, tip_location: int = -1, levels=(20.0, 20.0, 20.0, 20.0), speeds=(10.0, 10.0, 10.0, 10.0), forceabort=None):
    events: list[tuple[Any, ...]] = []
    sleeps: list[float] = []
    children = [Child(i, events, level=levels[i], speed=speeds[i]) for i in range(4)]
    subject = FourPipetteTransport(
        cast(Any, children),
        sleep=sleeps.append,
        liquid_mutation_enabled=True,
        forceabort=forceabort,
    )
    subject.loadTip(200, tip_location)
    query_count = 0

    def query_all():
        nonlocal query_count
        query_count += 1
        return {
            "ok": True,
            "tip_count": 4,
            "channels_with_tips": [0, 1, 2, 3],
            "channels": [{"channel": i, "tip_loaded": True} for i in range(4)],
        }

    subject.query_tip_status_all = query_all  # type: ignore[method-assign]
    return subject, children, events, sleeps, lambda: query_count


def test_standard_aspirate_uses_tip_location_queries_once_and_sleeps_after_every_send():
    subject, _children, events, sleeps, query_count = owner(tip_location=-1, speeds=(4.0, 20.0, 20.0, 20.0))

    result = subject.aspirate(PipetteAspirateCommand(volume_ul=8.0))

    assert query_count() == 1
    assert [row[1] for row in events if row[0] == "aspirate"] == [0, 1, 2, 3]
    assert all(row[3] is False for row in events if row[0] == "aspirate")
    assert sleeps.count(0.010) == 4
    assert result["timeout_ms"] == 14_000  # channel-0 speed when TipLocation == -1


def test_explicit_aspirate_overload_is_selected_only_when_speed_is_supplied():
    subject, _children, events, sleeps, query_count = owner(tip_location=2)

    result = subject.aspirate(PipetteAspirateCommand(volume_ul=5.0, channels=(1, 3), speed=5))

    assert query_count() == 0
    assert [row[:3] for row in events if row[0] == "speed"] == [("speed", 1, 5.0), ("speed", 3, 5.0)]
    assert [row[1] for row in events if row[0] == "aspirate"] == [1, 3]
    assert result["timeout_ms"] == 9_000
    assert result["speed_phase"]["ok"] is True
    assert result["speed_phase"]["timeout_ms"] == 7_000
    assert result["speed_phase"]["post_wait_delay_ms"] == 10
    assert sleeps.count(0.010) == 5


def test_standard_dispense_has_oem_presend_delay_postsend_delay_timeout_and_source_return():
    subject, _children, events, sleeps, query_count = owner(tip_location=1, speeds=(20.0, 2.0, 20.0, 20.0))

    result = subject.dispense(PipetteDispenseCommand(volume_ul=5.0))

    assert query_count() == 1
    assert [row[1] for row in events if row[0] == "dispense"] == [1]
    assert all(row[3] is False for row in events if row[0] == "dispense")
    assert sleeps[:2] == [0.005, 0.010]
    assert result["timeout_ms"] == 17_500
    assert result["source_return"] == 0
    assert result["channels"][0]["result"]["dispense_type"] == 0


def test_dispense_uses_distinct_exact_standard_and_explicit_oem_timeout_formulas():
    standard, _children, _events, _sleeps, _queries = owner(tip_location=0, speeds=(3.0, 3.0, 3.0, 3.0))
    standard_result = standard.dispense(PipetteDispenseCommand(volume_ul=1.0))
    assert standard_result["timeout_ms"] == 6_666

    explicit, _children, _events, _sleeps, _queries = owner(tip_location=0, speeds=(3.0, 3.0, 3.0, 3.0))
    explicit_result = explicit.dispense(
        PipetteDispenseCommand(volume_ul=1.0, channels=(0,), speed=3.0)
    )
    assert explicit_result["timeout_ms"] == 5_665


def test_driver_dispense_wire_type_is_always_literal_one_but_accounting_type_is_retained():
    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    captured = {}

    def send(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        return {"ok": True}

    driver.__dict__["_send_pipette_command"] = send
    result = driver.dispense(4.5, dispense_type=2, wait_for_completion=False)

    assert captured["command"] == "D4.5,1R"
    assert result["dispense_type"] == 2


def test_one_channel_mix_has_no_tip_query_and_uses_sequence_covering_timeout():
    subject, _children, events, _sleeps, query_count = owner(tip_location=3)

    result = subject.mix(PipetteMixCommand(volume_ul=2.0, cycles=3))

    assert query_count() == 0
    assert [row[1] for row in events if row[0] == "mix"] == [3]
    assert all(row[3] is False for row in events if row[0] == "mix")
    assert result["timeout_ms"] >= 7_500
    assert result["completion_verified"] is False
    assert result["state_reconciled"] is False


def test_driver_mix_uses_accounting_type_zero_and_no_constituent_completion_waits():
    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver._sleep = lambda seconds: sleeps.append(seconds)
    calls = []
    sleeps = []
    driver.__dict__["aspirate"] = lambda volume, **kwargs: calls.append(("P", volume, kwargs)) or {"ok": True}
    driver.__dict__["dispense"] = lambda volume, **kwargs: calls.append(("D", volume, kwargs)) or {"ok": True}

    result = driver.mix(2.0, 2, wait_for_completion=True)

    assert [(row[0], row[2]["wait_for_completion"]) for row in calls] == [("P", False), ("D", False), ("P", False)]
    assert calls[1][2]["dispense_type"] == 0
    assert sleeps == [1.5, 1.5, 1.5]
    assert result["ok"] is True


def test_dispense_all_noarg_queries_once_explicit_does_not_and_returns_channel_zero_level():
    subject, _children, events, sleeps, query_count = owner(tip_location=-1, levels=(33.0, 40.0, 50.0, 60.0), speeds=(10.0, 1.0, 1.0, 1.0))

    standard = subject.dispense_all()

    assert query_count() == 1
    assert sleeps.count(0.030) == 4
    assert standard["source_return"] == 33.0
    assert standard["timeout_ms"] == 20_500

    events.clear()
    sleeps.clear()
    explicit = subject.dispense_all(channels=[2])
    assert query_count() == 1
    assert [row[1] for row in events if row[0] == "dispense_all"] == [2]
    assert sleeps == [0.030]
    assert explicit["source_return"] == 0.0


def test_terminate_can_interrupt_group_wait_sends_all_tr_and_uses_shared_eight_second_deadline():
    subject, children, events, _sleeps, _query_count = owner(tip_location=0)
    wait_entered = threading.Event()
    release_wait = threading.Event()

    def blocking_wait(timeout_s):
        wait_entered.set()
        release_wait.wait(timeout=1.0)
        return {"ok": True, "outcome": "completion", "timeout_seen": timeout_s}

    children[0].wait_for_completion = blocking_wait  # type: ignore[method-assign]
    aspirate_box = {}
    worker = threading.Thread(
        target=lambda: aspirate_box.setdefault(
            "result",
            subject.aspirate(PipetteAspirateCommand(volume_ul=1.0)),
        )
    )
    worker.start()
    assert wait_entered.wait(timeout=0.5)

    terminated_box = {}
    terminate_worker = threading.Thread(
        target=lambda: terminated_box.setdefault("result", subject.terminate())
    )
    terminate_worker.start()
    deadline = time.monotonic() + 0.5
    while len([row for row in events if row[0] == "terminate"]) < 4 and time.monotonic() < deadline:
        time.sleep(0.005)
    assert [row[1] for row in events if row[0] == "terminate"] == [0, 1, 2, 3]
    release_wait.set()
    worker.join(timeout=1.0)
    terminate_worker.join(timeout=1.0)
    terminated = terminated_box["result"]

    assert all(row[2] is False for row in events if row[0] == "terminate")
    assert terminated["timeout_ms"] == 8_000
    assert terminated["ok"] is True
    interrupted = aspirate_box["result"]
    assert interrupted["ok"] is False
    assert interrupted["completion_verified"] is False
    assert interrupted["state_reconciled"] is False
    assert interrupted["interrupted_by_terminate"] is True
    assert children[0]._liquid_level_ul == 20.0


def test_aspirate_timeout_failure_retains_oem_one_second_sleep():
    subject, children, _events, sleeps, _query_count = owner(tip_location=0)
    children[0].wait_for_completion = lambda _timeout: {"ok": False, "outcome": "timeout"}  # type: ignore[method-assign]

    result = subject.aspirate(PipetteAspirateCommand(volume_ul=1.0))

    assert result["ok"] is False
    assert sleeps[-1] == 1.0


def test_dispense_all_checks_forceabort_before_send_and_after_wait():
    latch = {"value": True}
    subject, _children, events, _sleeps, _query_count = owner(
        tip_location=0,
        forceabort=lambda: latch["value"],
    )

    from src.bioxp.pipette.transport import PipetteCommandError

    try:
        subject.dispense_all()
    except PipetteCommandError as exc:
        assert "force abort" in str(exc)
    else:  # pragma: no cover - fail-closed assertion
        raise AssertionError("forceabort must reject DispenseAll")
    assert not [row for row in events if row[0] == "dispense_all"]

    latch["value"] = False

    def complete_and_abort(_timeout):
        latch["value"] = True
        return {"ok": True, "outcome": "completion"}

    _children[0].wait_for_completion = complete_and_abort  # type: ignore[method-assign]
    try:
        subject.dispense_all()
    except PipetteCommandError as exc:
        assert "force abort" in str(exc)
    else:  # pragma: no cover - fail-closed assertion
        raise AssertionError("post-wait forceabort must reject DispenseAll")
    assert [row[1] for row in events if row[0] == "dispense_all"] == [0]


def test_api_exposes_only_paired_typed_aspirate_explicit_overload() -> None:
    standard = PipetteAspirateRequest(volume_ul=5.0)
    assert standard.channels is None
    assert standard.speed is None
    explicit = PipetteAspirateRequest(volume_ul=5.0, channels=[0, 2], speed=125.0)
    assert explicit.channels == [0, 2]
    assert explicit.speed == 125.0
    with pytest.raises(ValueError, match="both channels and speed"):
        PipetteAspirateRequest(volume_ul=5.0, channels=[0])


def test_api_exposes_paired_dispense_overload_and_fixes_accounting_type_zero() -> None:
    explicit = PipetteDispenseRequest(
        volume_ul=5.0,
        channels=[1, 3],
        speed=90.0,
        dispense_type=0,
    )
    assert explicit.channels == [1, 3]
    assert explicit.speed == 90.0
    assert explicit.dispense_type == 0
    with pytest.raises(ValueError, match="both channels and speed"):
        PipetteDispenseRequest(volume_ul=5.0, speed=90.0)
    with pytest.raises(ValueError):
        PipetteDispenseRequest(volume_ul=5.0, dispense_type=2)


def test_api_pipette_forceabort_uses_retained_oem_latch_without_hardware_query() -> None:
    calls = 0

    class Tester:
        def oem_no24v_state(self) -> bool:
            nonlocal calls
            calls += 1
            return True

    assert _pipette_forceabort_latched(Tester()) is True
    assert calls == 1
