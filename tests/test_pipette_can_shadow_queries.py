from __future__ import annotations

from types import SimpleNamespace

from src.bioxp.can_driver import BioXpCanDriver


class FakeBus:
    def __init__(self, replies):
        self.replies = list(replies)
        self.sent = []
        self.recv_timeouts = []

    def send(self, msg):
        self.sent.append(msg)

    def recv(self, timeout=None):
        self.recv_timeouts.append(timeout)
        if not self.replies:
            return None
        return self.replies.pop(0)


def _driver_with_replies(*replies, pipette_id=0):
    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver.bus = FakeBus(replies)
    driver.channel = "vcan-test"
    driver.bitrate = 1_000_000
    driver.pipette_id = int(pipette_id)
    driver.response_timeout_s = 0.01
    return driver


def _frame(arbitration_id: int, data: list[int] | bytes | bytearray):
    return SimpleNamespace(
        arbitration_id=int(arbitration_id),
        data=list(data),
        dlc=len(data),
        is_extended_id=False,
    )


def test_query_tip_status_demuxes_async_frame_until_pipette_report_reply():
    driver = _driver_with_replies(
        _frame(0x100, b"EVTdoor"),
        _frame(0x506, [0x20, 0x60, ord("1")]),
    )

    result = driver.query_tip_status()

    assert driver.bus.sent[0].arbitration_id == 0x106
    assert bytes(driver.bus.sent[0].data[:3]).decode("ascii") == "?31"
    assert result["ok"] is True
    assert result["tip_loaded"] is True
    assert result["hardware_truth_level"] == "hardware_query"
    assert result["ack"]["arbitration_id"] == 0x506
    assert result["ack"]["demux"]["matched_address"] == "report_rx"
    assert result["ack"]["demux"]["skipped_count"] == 1
    assert result["ack"]["demux"]["skipped_frames"][0]["arbitration_id"] == 0x100
    assert result["oem_source_anchor"] == "ClassPipette.QueryTipStatus: ?31"


def test_query_pressure_demuxes_report_reply_and_parses_ascii_pressure_value():
    driver = _driver_with_replies(
        _frame(0x101, b"WR\x20\x00\x00\x00\x00\x00"),
        _frame(0x506, bytes([0x20, 0x60]) + b"123.4"),
    )

    result = driver.query_pressure()

    assert driver.bus.sent[0].arbitration_id == 0x106
    assert bytes(driver.bus.sent[0].data[:3]).decode("ascii") == "?57"
    assert result["ok"] is True
    assert result["pressure"] == 123.4
    assert result["ack"]["arbitration_id"] == 0x506
    assert result["ack"]["demux"]["matched_address"] == "report_rx"
    assert result["ack"]["demux"]["skipped_count"] == 1
    assert result["oem_source_anchor"] == "ClassPipette.QueryPressure: ?57"


def test_query_tip_status_keeps_transport_and_semantic_truth_separate_for_unparsed_reply():
    driver = _driver_with_replies(_frame(0x506, b"????...."))

    result = driver.query_tip_status()

    assert result["ok"] is False
    assert result["reply_received"] is True
    assert result["semantic_ok"] is False
    assert result["tip_loaded"] is None
    assert result["hardware_truth_level"] == "unparsed_hardware_reply"


def test_query_pressure_does_not_parse_command_echo_digits_as_pressure():
    driver = _driver_with_replies(_frame(0x506, b"?57....."))

    result = driver.query_pressure()

    assert result["ok"] is False
    assert result["reply_received"] is True
    assert result["semantic_ok"] is False
    assert result["pressure"] is None
    assert result["hardware_truth_level"] == "unparsed_hardware_reply"


def test_query_demux_uses_single_absolute_timeout_after_skipped_frames():
    driver = _driver_with_replies(
        *[_frame(0x180 + i, b"noise") for i in range(20)],
    )

    result = driver.query_tip_status()

    assert result["ok"] is False
    assert result["ack"]["error"] == "ack_timeout"
    assert result["ack"]["demux"]["skipped_count"] == 20
    assert len(result["ack"]["demux"]["skipped_frames"]) == 12
    assert result["ack"]["demux"]["skipped_frames_truncated"] is True
    assert max(driver.bus.recv_timeouts) <= driver.response_timeout_s
