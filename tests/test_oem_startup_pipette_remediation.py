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


def test_startup_eject_targets_loaded_channels_and_verifies_empty():
    fixture = build_collection([[True, False], [False], [True, False], [False]])
    result = fixture.collection.eject_all_tips_for_oem_startup(
        operator_ack="EJECT_STALE_STARTUP_TIPS",
        expected_channels_with_tips=[0, 2],
    )
    assert result["ok"] is True
    assert result["outcome"] == "verified_empty"
    assert result["physical_effect_verified"] is True
    assert result["channels_targeted"] == [0, 2]
    assert fixture.drivers[0].commands == ["?31", "E1R", "?31"]
    assert fixture.drivers[1].commands == ["?31", "?31"]
    assert fixture.drivers[2].commands == ["?31", "E1R", "?31"]
    assert fixture.drivers[3].commands == ["?31", "?31"]


def test_startup_eject_rejects_stale_expected_tip_set_without_transmit():
    fixture = build_collection([[True], [False], [False], [False]])
    with pytest.raises(PipetteCommandError, match="changed since authorization"):
        fixture.collection.eject_all_tips_for_oem_startup(
            operator_ack="EJECT_STALE_STARTUP_TIPS",
            expected_channels_with_tips=[0, 2],
        )
    assert all("E1R" not in driver.commands for driver in fixture.drivers)


def test_startup_eject_requires_literal_operator_ack_without_transmit():
    fixture = build_collection([[True], [False], [False], [False]])
    with pytest.raises(PipetteCommandError, match="operator acknowledgement"):
        fixture.collection.eject_all_tips_for_oem_startup(
            operator_ack=True,
            expected_channels_with_tips=[0],
        )
    assert all(driver.commands == [] for driver in fixture.drivers)


def test_partial_ejection_failure_reports_prior_mutations_and_fresh_post_attempt_readback():
    fixture = build_collection(
        [[True, False], [False], [True, True], [False]],
        eject_ok_channel=2,
    )
    with pytest.raises(PipetteCommandError, match="immediate acknowledgement") as captured:
        fixture.collection.eject_all_tips_for_oem_startup(
            operator_ack="EJECT_STALE_STARTUP_TIPS",
            expected_channels_with_tips=[0, 2],
        )
    details = captured.value.details
    assert [row["channel"] for row in details["sends"]] == [0]
    assert details["failed_channel"] == 2
    assert details["post_attempt_readback"]["channels_with_tips"] == [2]
    assert details["physical_effect_verified"] is False
    assert fixture.drivers[0].commands == ["?31", "E1R", "?31"]
    assert fixture.drivers[2].commands == ["?31", "E1R", "?31"]


def test_startup_eject_fails_closed_on_missing_ack_or_nonempty_postcondition():
    ack_failure = build_collection([[True, True], [False], [False], [False]], eject_ok_channel=0)
    with pytest.raises(PipetteCommandError, match="immediate acknowledgement"):
        ack_failure.collection.eject_all_tips_for_oem_startup(
            operator_ack="EJECT_STALE_STARTUP_TIPS",
            expected_channels_with_tips=[0],
        )

    still_loaded = build_collection([[True, True], [False], [False], [False]])
    with pytest.raises(PipetteCommandError, match="still reports loaded tips"):
        still_loaded.collection.eject_all_tips_for_oem_startup(
            operator_ack="EJECT_STALE_STARTUP_TIPS",
            expected_channels_with_tips=[0],
        )
