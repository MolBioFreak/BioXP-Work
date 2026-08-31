from __future__ import annotations

from typing import Any, cast
import time

import pytest

from src.bioxp.pipette.models import PipetteAspirateCommand, PipetteDiagnosticCommand
from src.bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport, PipetteCommandError


class FakeTransport:
    def __init__(self, channel: int, *, tip_loaded: bool = False, speed: float = 10.0):
        self.channel = channel
        self._tip_loaded = tip_loaded
        self._last_tip_status = {
            "ok": True,
            "semantic_ok": True,
            "hardware_truth_level": "hardware_query",
            "tip_loaded": tip_loaded,
            "observed_at": time.time(),
            "reader_generation": 1,
        }
        self._top_speed = speed
        self._reader_generation = 1
        self._error_callback = None
        self.calls: list[tuple[str, object]] = []

    def set_top_speed(self, value: float):
        self.calls.append(("set_top_speed", value))
        self._top_speed = value
        return {"ok": True, "completion_verified": True}

    def query_pressure(self):
        self.calls.append(("query_pressure", None))
        return {"ok": True, "semantic_ok": True, "pressure": float(self.channel)}

    def execute_diagnoses(self, command, *, wait_for_completion: bool):
        self.calls.append(("execute_diagnoses", (command.number, wait_for_completion)))
        return {"ok": True}

    def wait_for_completion(self, timeout_s: float):
        self.calls.append(("wait_for_completion", timeout_s))
        return {"ok": True, "pipette_message_state": {"diagnosis": "ok"}}

    def get_data(self, query: str):
        self.calls.append(("get_data", query))
        return {"ok": True, "semantic_ok": True, "query": query, "value": f"{self.channel}:{query}"}


def collection(*, tips=(False, False, False, False), speeds=(10.0, 10.0, 10.0, 10.0)):
    transports = [
        FakeTransport(channel, tip_loaded=tips[channel], speed=speeds[channel])
        for channel in range(4)
    ]
    return FourPipetteTransport(
        cast(Any, transports),
        sleep=lambda _seconds: None,
        liquid_mutation_enabled=True,
    ), transports


def test_standard_liquid_selection_uses_oem_tip_location_not_metadata():
    subject, _ = collection()
    subject.loadTip(200, 2)

    assert subject._tip_location_channels() == [2]

    subject.loadTip(200, -1)
    assert subject._tip_location_channels() == [0, 1, 2, 3]

    command = PipetteAspirateCommand(volume_ul=1.0, metadata={"channels": [3], "speed": 2})
    assert command.channels is None
    assert command.speed is None


def test_set_top_speed_skips_channels_without_tips_and_honors_oem_early_return():
    subject, transports = collection(tips=(False, True, False, True), speeds=(20.0, 10.0, 10.0, 10.0))

    early = subject.set_top_speed(20.0)
    assert early["ok"] is True
    assert early["outcome"] == "unchanged_channel_zero_speed"
    assert all(not transport.calls for transport in transports)

    result = subject.set_top_speed(12.5)
    assert result["ok"] is True
    assert [row["channel"] for row in result["channels"]] == [1, 3]
    assert transports[0].calls == []
    assert transports[1].calls == [("set_top_speed", 12.5)]
    assert transports[2].calls == []
    assert transports[3].calls == [("set_top_speed", 12.5)]


def test_pressure_and_diagnostics_default_to_cached_tip_channels():
    subject, transports = collection(tips=(True, False, True, False))

    pressure = subject.read_pressure()
    diagnosis = subject.execute_diagnoses(PipetteDiagnosticCommand(number=1))

    assert [row["channel"] for row in pressure["channels"]] == [0, 2]
    assert [row["channel"] for row in diagnosis["channels"]] == [0, 2]
    assert all(not transports[channel].calls for channel in (1, 3))


def test_get_data_without_query_runs_exact_oem_sweep_for_each_selected_channel():
    subject, transports = collection()
    expected = ["?40", "?41", "?42", "?44", "?45", "?47", "?51", "?52", "?53", "?54", "?55", "?58", "?59"]

    result = subject.get_data(cast(Any, None), channels=[1, 3])

    assert result["ok"] is True
    assert result["queries"] == expected
    assert transports[0].calls == []
    assert transports[2].calls == []
    assert transports[1].calls == [("get_data", query) for query in expected]
    assert transports[3].calls == [("get_data", query) for query in expected]


def test_group_host_accounting_waits_for_correlated_channel_completion():
    class Driver:
        _pipette_message_state = {}

        @staticmethod
        def query_tip_status():
            return {"ok": True, "semantic_ok": True, "tip_loaded": True, "hardware_truth_level": "hardware_query"}

        @staticmethod
        def aspirate(*args, **kwargs):
            del args, kwargs
            return {
                "ok": True,
                "tx_ok": True,
                "delivery_verified": True,
                "controller_acknowledged": True,
                "completion_verified": False,
                "provenance": {"outcome": "ack"},
            }

        @staticmethod
        def wait_pipette_command_completion(timeout_s):
            del timeout_s
            return {"ok": True, "outcome": "completion"}

    transports = [
        CanPipetteTransport(driver_factory=Driver, pipette_id=channel)
        for channel in range(4)
    ]
    for transport in transports:
        transport._initialized = True
    subject = FourPipetteTransport(
        transports,
        sleep=lambda _seconds: None,
        liquid_mutation_enabled=True,
    )

    result = subject.aspirate(
        PipetteAspirateCommand(volume_ul=5.0, metadata={"channels": [0, 1, 2, 3]})
    )

    assert result["ok"] is True
    assert result["delivery_verified"] is True
    assert result["controller_acknowledged"] is True
    assert result["completion_verified"] is True
    assert result["state_reconciled"] is True
    assert result["physical_effect_verified"] is False
    assert [transport._liquid_level_ul for transport in transports] == [5.0, 5.0, 5.0, 5.0]
