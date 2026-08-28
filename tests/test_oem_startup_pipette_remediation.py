from __future__ import annotations

from dataclasses import dataclass

import pytest

from src.bioxp.pipette.models import PipetteCommandError
from src.bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport


class FakeDriver:
    def __init__(self, tip_states: list[bool], *, eject_ok: bool = True):
        self.tip_states = list(tip_states)
        self.eject_ok = eject_ok
        self.commands: list[str] = []

    def query_tip_status(self):
        self.commands.append("?31")
        tip = self.tip_states.pop(0) if len(self.tip_states) > 1 else self.tip_states[0]
        return {"ok": True, "tip_loaded": tip, "semantic_ok": True, "hardware_truth_level": "hardware_query"}

    def pipette_eject_tip(self):
        self.commands.append("E1R")
        return {"ok": self.eject_ok, "ack": {"outcome": "ack"}, "error": None if self.eject_ok else "ack_failed"}


@dataclass
class CollectionFixture:
    collection: FourPipetteTransport
    drivers: list[FakeDriver]


def build_collection(states: list[list[bool]], *, eject_ok_channel: int | None = None) -> CollectionFixture:
    drivers = [
        FakeDriver(rows, eject_ok=index != eject_ok_channel)
        for index, rows in enumerate(states)
    ]
    transports = [
        CanPipetteTransport(driver_factory=lambda driver=driver: driver, pipette_id=index)
        for index, driver in enumerate(drivers)
    ]
    return CollectionFixture(FourPipetteTransport(transports, sleep=lambda _seconds: None), drivers)


def test_query_tip_status_all_requires_exact_hardware_readback():
    fixture = build_collection([[True], [False], [True], [False]])
    result = fixture.collection.query_tip_status_all()
    assert result["ok"] is True
    assert result["tip_count"] == 2
    assert result["channels_with_tips"] == [0, 2]
    assert result["physical_effect_verified"] is False
    assert result["hardware_query_verified"] is True


def test_query_tip_status_rejects_software_shadow_or_semantic_failure():
    fixture = build_collection([[True], [False], [False], [False]])
    fixture.drivers[0].query_tip_status = lambda: {
        "ok": True,
        "tip_loaded": True,
        "semantic_ok": False,
        "hardware_truth_level": "software_shadow",
    }
    with pytest.raises(PipetteCommandError, match="hardware tip-status"):
        fixture.collection.query_tip_status_all()
