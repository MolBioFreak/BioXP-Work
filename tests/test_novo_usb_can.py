from __future__ import annotations

from src.bioxp.novo_usb_can import BioXpNovoUsbDriver, NovoUsbCanBus, novo_decode, novo_encode


class FakeEndpointOut:
    def __init__(self):
        self.writes = []

    def write(self, frame, timeout=None):
        self.writes.append((bytes(frame), timeout))
        return len(frame)


class FakeEndpointIn:
    def __init__(self, frames):
        self.frames = list(frames)
        self.reads = []

    def read(self, size, timeout=None):
        self.reads.append((size, timeout))
        if not self.frames:
            raise TimeoutError("timeout")
        return list(self.frames.pop(0))


class FakeSharedUsb:
    def __init__(self, replies):
        self.dev = object()
        self.ep_out = FakeEndpointOut()
        self.ep_in = FakeEndpointIn(replies)


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
    bus = NovoUsbCanBus(shared_usb=shared)
    msg = type("Msg", (), {"arbitration_id": 0x106, "data": [ord("?"), ord("3"), ord("1"), 0, 0, 0, 0, 0], "dlc": 8})()

    bus.send(msg)

    written, timeout = shared.ep_out.writes[0]
    decoded = novo_decode(written)
    assert timeout == 2000
    assert decoded[:5] == bytes([0x00, 0x00, 0x01, 0x06, 0x08])
    assert decoded[5:13] == b"?31\x00\x00\x00\x00\x00"


def test_novo_usb_driver_reuses_bioxp_tester_endpoint_and_queries_tip_status():
    shared = FakeSharedUsb([
        _novo_frame(0x100, b"EVTdoor"),
        _novo_frame(0x106, [0x00, 0x00, ord("1"), 0x00, 0x00, 0x00, 0x00, 0x00]),
    ])
    driver = BioXpNovoUsbDriver(shared_usb=shared, response_timeout_s=0.05)

    result = driver.query_tip_status()

    assert result["ok"] is True
    assert result["tip_loaded"] is True
    assert result["ack"]["demux"]["matched_address"] == "report"
    assert result["ack"]["demux"]["skipped_count"] == 1
    written, _timeout = shared.ep_out.writes[0]
    decoded = novo_decode(written)
    assert decoded[:5] == bytes([0x00, 0x00, 0x01, 0x06, 0x08])
    assert decoded[5:8] == b"?31"
