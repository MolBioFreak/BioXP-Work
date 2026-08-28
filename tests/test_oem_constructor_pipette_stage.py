from __future__ import annotations

from dataclasses import dataclass, field
from types import MethodType
from typing import Any

from src.bioxp.can_driver import BioXpCanDriver
from src.bioxp.pipette.models import PipetteInitCommand
from src.bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport


@dataclass
class RecordingRouter:
    events: list[tuple[Any, ...]]
    epoch: int = 0

    def begin_pressure_epoch(self) -> dict[str, int]:
        self.epoch += 1
        self.events.append(("pressure_epoch", self.epoch))
        return {"epoch": self.epoch}

    def calculate_pressure_offsets(self) -> dict[int, float]:
        self.events.append(("calculate_pressure_offsets", self.epoch))
        return {channel: float(channel) for channel in range(4)}


@dataclass
class RecordingDriver:
    channel: int
    events: list[tuple[Any, ...]]
    router: RecordingRouter
    status_results: list[dict[str, Any]] = field(default_factory=lambda: [{"ok": True, "oem_error_code": 0}])
    completion_timeouts: list[float] = field(default_factory=list)
    initialize_calls: int = 0

    def __post_init__(self) -> None:
        self.bus = type("Bus", (), {"router": self.router})()

    def pipette_initialize(self, pressure_profile: str = "1R") -> dict[str, Any]:
        self.initialize_calls += 1
        self.events.append(("initiate_false", self.channel, pressure_profile, self.initialize_calls))
        return {
            "ok": True,
            "immediate_ack_received": True,
            "initialized_after_valid_completion": False,
        }

    def pipette_initiate_group(self) -> dict[str, Any]:
        self.events.append(("wrong_group_wr", self.channel))
        return {"ok": True, "immediate_ack_received": True}

    def wait_pipette_initialization_completion(self, timeout_s: float) -> dict[str, Any]:
        self.completion_timeouts.append(float(timeout_s))
        self.events.append(("wait_completion", self.channel, float(timeout_s)))
        return {"ok": True, "channel": self.channel, "outcome": "completion"}

    def enable_pressure_stream(self, enabled: bool) -> dict[str, Any]:
        self.events.append(("pressure_stream", self.channel, bool(enabled), self.router.epoch))
        return {"ok": True}

    def query_firmware(self, number: int = 1) -> dict[str, Any]:
        self.events.append(("firmware", self.channel, number))
        return {"ok": True, "firmware": "OEM"}

    def query_status(self) -> dict[str, Any]:
        self.events.append(("status", self.channel))
        if len(self.status_results) > 1:
            return self.status_results.pop(0)
        return dict(self.status_results[0])

    def close(self) -> None:
        return None


def _collection(*, status_results: dict[int, list[dict[str, Any]]] | None = None):
    events: list[tuple[Any, ...]] = []
    router = RecordingRouter(events)
    drivers: list[RecordingDriver] = []
    transports: list[CanPipetteTransport] = []
    for channel in range(4):
        driver = RecordingDriver(
            channel,
            events,
            router,
            status_results=list((status_results or {}).get(channel, [{"ok": True, "oem_error_code": 0}])),
        )
        drivers.append(driver)
        transports.append(
            CanPipetteTransport(
                driver_factory=lambda value=driver: value,
                pipette_id=channel,
                transport_name="novo_usb_can",
                transport_details={"shared_bioxp_usb_runtime": True},
            )
        )
    sleeps: list[float] = []
    return FourPipetteTransport(transports, sleep=sleeps.append), drivers, router, events, sleeps


def test_query_status_treats_captured_three_byte_q1_reply_as_empty_oem_error_collection():
    driver = object.__new__(BioXpCanDriver)

    def fake_send(self, *args, **kwargs):
        return {
            "ok": True,
            "ack": {"data": [0x20, 0x60, 0x40]},
            "provenance": {"outcome": "completion"},
        }

    driver._send_pipette_command = MethodType(fake_send, driver)

    result = driver.query_status()

    assert result["reply_received"] is True
    assert result["oem_raw_status"] == 0x40
    assert result["oem_error_code"] == 0
    assert result["oem_error_codes"] == []
    assert result["oem_error_free"] is True
    assert result["ok"] is True
    assert result["error"] is None


def test_query_status_applies_oem_error_collection_from_index_three():
    driver = object.__new__(BioXpCanDriver)

    def fake_send(self, *args, **kwargs):
        return {
            "ok": True,
            "ack": {"data": [0x20, 0x60, 0x40, 0x45]},
            "provenance": {"outcome": "completion"},
        }

    driver._send_pipette_command = MethodType(fake_send, driver)

    result = driver.query_status()

    assert result["reply_received"] is True
    assert result["oem_error_code"] == 0x45
    assert result["oem_error_codes"] == [0x45]
    assert result["oem_error_free"] is False
    assert result["ok"] is False
    assert result["error"] == "pipette_reported_error_0x45"