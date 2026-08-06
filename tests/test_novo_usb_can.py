from __future__ import annotations

import threading

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


def test_novo_usb_driver_keeps_base_pipette_delay_hook_without_claiming_socketcan():
    shared = FakeSharedUsb([])
    try:
        driver = BioXpNovoUsbDriver(shared_usb=shared)

        assert callable(driver._sleep)
    finally:
        shared.close()
