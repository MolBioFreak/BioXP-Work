from __future__ import annotations

import threading

import pytest

from src.bioxp.novo_usb_can import BioXpNovoUsbDriver, NovoUsbCanBus, novo_decode, novo_encode
from src.bioxp.novo_router import NovoRouter


class FakeEndpointOut:
    def __init__(self, write_event):
        self.writes = []
        self.write_event = write_event

    def write(self, frame, timeout=None):
        self.writes.append((bytes(frame), timeout))
        self.write_event.set()
        return len(frame)


class FakeEndpointIn:
    def __init__(self, frames, write_event):
        self.frames = list(frames)
        self.reads = []
        self.write_event = write_event

    def read(self, size, timeout=None):
        self.reads.append((size, timeout))
        if not self.write_event.wait((timeout or 100) / 1000.0):
            raise TimeoutError("timeout")
        if not self.frames:
            raise TimeoutError("timeout")
        return list(self.frames.pop(0))


class FakeSharedUsb:
    def __init__(self, replies):
        write_event = threading.Event()
        self.dev = object()
        self.ep_out = FakeEndpointOut(write_event)
        self.ep_in = FakeEndpointIn(replies, write_event)
        self.novo_router = NovoRouter(
            ep_in=self.ep_in,
            ep_out=self.ep_out,
            decode=novo_decode,
            read_timeout_ms=10,
        )
        self.novo_router.start()

    def close(self):
        self.novo_router.shutdown()


def _novo_frame(arbitration_id: int, data: list[int] | bytes | bytearray) -> bytes:
    return novo_encode(NovoUsbCanBus.build_payload(arbitration_id, data, len(data)))


def test_group_initialization_defers_completion_until_collection_waits():
    shared = FakeSharedUsb([
        _novo_frame(0x501, []),
        _novo_frame(0x501, [0x20, 0x20]),
    ])
    try:
        driver = BioXpNovoUsbDriver(shared_usb=shared, pipette_id=0)

        sent = driver.pipette_initiate_group()

        assert sent["ok"] is True
        assert sent["immediate_ack_received"] is True
        assert sent["completion_deferred"] is True
        assert isinstance(sent["completion_owner_token"], str)
        completion = driver.wait_pipette_initialization_completion(1.0)
        assert completion["ok"] is True
        assert completion["outcome"] == "completion"
    finally:
        shared.close()


def test_pipette_transactions_reject_rx_domain_and_wrong_multipart_tx_ids():
    shared = FakeSharedUsb([])
    try:
        bus = NovoUsbCanBus(shared_usb=shared)
        rx_domain = type("Msg", (), {"arbitration_id": 0x501, "data": [ord("W"), ord("R")], "dlc": 2})()
        with pytest.raises(Exception, match="TX tuple"):
            bus.transact_can(
                rx_domain,
                channel=0,
                expected_function=1,
                timeout_s=0.0,
                matcher_name="invalid_rx_domain",
            )

        wrong_first = type("Msg", (), {"arbitration_id": 0x104, "data": list(b"12345678"), "dlc": 8})()
        valid_final = type("Msg", (), {"arbitration_id": 0x101, "data": list(b"R"), "dlc": 1})()
        with pytest.raises(Exception, match="multipart TX tuple"):
            bus.transact_can_many(
                [wrong_first, valid_final],
                channel=0,
                expected_function=1,
                timeout_s=0.0,
                matcher_name="invalid_first_fragment",
            )
    finally:
        shared.close()


def test_novo_encoding_round_trips_and_escapes_frame_bytes():
    payload = bytes([0x00, 0x00, 0x01, 0x06, 0x04, 0x7E, 0x7D, 0x31, 0x00])
    frame = novo_encode(payload)

    assert frame[0] == 0x7E
    assert frame[-1] == 0x7E
    assert bytes([0x7D, 0x5E]) in frame
    assert bytes([0x7D, 0x5D]) in frame
    assert novo_decode(frame) == payload


def test_novo_usb_bus_writes_oem_caninterfaceboard_record_shape():
    shared = FakeSharedUsb([])
    try:
        bus = NovoUsbCanBus(shared_usb=shared)
        msg = type("Msg", (), {"arbitration_id": 0x106, "data": [ord("?"), ord("3"), ord("1"), 0, 0, 0, 0, 0], "dlc": 8})()

        bus.send(msg)

        written, timeout = shared.ep_out.writes[0]
        decoded = novo_decode(written)
        assert timeout == 2000
        assert decoded[:5] == bytes([0x00, 0x00, 0x01, 0x06, 0x08])
        assert decoded[5:13] == b"?31\x00\x00\x00\x00\x00"
    finally:
        shared.close()


def test_novo_usb_driver_reuses_bioxp_tester_endpoint_and_queries_tip_status():
    shared = FakeSharedUsb([
        _novo_frame(0x100, b"EVTdoor"),
        _novo_frame(0x506, [0x20, 0x60, ord("1")]),
    ])
    try:
        driver = BioXpNovoUsbDriver(shared_usb=shared, response_timeout_s=0.05)

        result = driver.query_tip_status()

        assert result["ok"] is True
        assert result["tip_loaded"] is True
        assert result["ack"]["arbitration_id"] == 0x506
        assert result["provenance"]["matcher"] == "query_tip_status"
        assert result["provenance"]["outcome"] == "completion"
        assert result["provenance"]["skipped_count"] == 1
        assert result["provenance"]["skipped_frames"][0]["arbitration_id"] == 0x100
        written, _timeout = shared.ep_out.writes[0]
        decoded = novo_decode(written)
        assert decoded[:5] == bytes([0x00, 0x00, 0x01, 0x06, 0x03])
        assert decoded[5:8] == b"?31"
    finally:
        shared.close()


def test_mutating_transaction_requires_dlc0_ack_before_deferred_token():
    shared = FakeSharedUsb([_novo_frame(0x501, b"")])
    try:
        bus = NovoUsbCanBus(shared_usb=shared)
        msg = type("Msg", (), {"arbitration_id": 0x101, "data": list(b"P1,1R"), "dlc": 5})()

        result = bus.transact_can(
            msg,
            channel=0,
            expected_function=1,
            timeout_s=0.1,
            matcher_name="pipette_aspirate",
            wait_for_completion=False,
        )

        assert result["ok"] is True
        assert result["outcome"] == "ack"
        assert result["immediate_ack_received"] is True
        assert result["completion_received"] is False
        assert result["completion_deferred"] is True
        assert isinstance(result["completion_owner_token"], str)
    finally:
        shared.close()


def test_mutating_transaction_waits_for_dlc0_ack_then_exact_dlc2_completion():
    shared = FakeSharedUsb([
        _novo_frame(0x501, b""),
        _novo_frame(0x501, bytes([0x20, 0xA5])),
    ])
    try:
        bus = NovoUsbCanBus(shared_usb=shared)
        msg = type("Msg", (), {"arbitration_id": 0x101, "data": list(b"P1,1R"), "dlc": 5})()

        result = bus.transact_can(
            msg,
            channel=0,
            expected_function=1,
            timeout_s=0.1,
            matcher_name="pipette_aspirate",
            completion_timeout_s=0.2,
            wait_for_completion=True,
        )

        assert result["ok"] is True
        assert result["immediate_ack_received"] is True
        assert result["completion_received"] is True
        assert result["completion"]["observed_rx_dlc"] == 2
        assert result["completion"]["data"] == [0x20, 0xA5]
    finally:
        shared.close()


def test_novo_usb_driver_keeps_base_pipette_delay_hook_without_claiming_socketcan():
    shared = FakeSharedUsb([])
    try:
        driver = BioXpNovoUsbDriver(shared_usb=shared)

        assert callable(driver._sleep)
    finally:
        shared.close()
