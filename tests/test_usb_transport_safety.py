import queue
import threading
import time

import pytest
import usb.core

from src.bioxp.novo_router import NovoFrame, NovoRouter
from src.bioxp.novo_usb_can import novo_decode, novo_encode
from src.bioxp.usb_driver import BioXpTester


def _bare_tester():
    tester = BioXpTester.__new__(BioXpTester)
    tester._bus_event_buffer = []
    tester._bus_event_buffer_max = 256
    tester._usb_sniff_ledger_path = None
    tester._usb_sniff_ledger_run_id = None
    tester._usb_sniff_ledger_seq = 0
    return tester


def _event_frame(*, status: int, cmd_or_axis: int, value_bytes: tuple[int, int, int, int], checksum: int = 0xEE):
    return [
        0x7E,
        0x00,
        0x00,
        0x00,
        0x00,
        0x08,
        BioXpTester.BOARD_HEAD,
        status,
        cmd_or_axis,
        *value_bytes,
        checksum,
        0x7E,
    ]


@pytest.mark.parametrize("motor", [0, 1, 2])
def test_target_position_event_decodes_motor_from_oem_msg8(motor):
    tester = _bare_tester()
    frame = _event_frame(
        status=128,
        cmd_or_axis=0,
        value_bytes=(0, 0, 0, motor),
        checksum=0xA0 + motor,
    )

    event = tester._decode_bus_event_frame(frame, source="test")

    assert event["status"] == 128
    assert event["motor"] == motor
    assert event["value"] == motor
    assert event["raw"][13] != motor


def test_stall_event_decodes_axis_from_oem_msg4_and_unknown_layout_is_unqualified():
    tester = _bare_tester()
    stall = tester._decode_bus_event_frame(
        _event_frame(status=130, cmd_or_axis=2, value_bytes=(0, 0, 0, 77)),
        source="test",
    )
    unknown = tester._decode_bus_event_frame(
        _event_frame(status=132, cmd_or_axis=2, value_bytes=(0, 0, 0, 1)),
        source="test",
    )

    assert stall["motor"] == 2
    assert unknown["motor"] is None


def test_send_tmcl_serializes_complete_write_reply_transactions():
    tester = _bare_tester()
    tester._transport_lock = threading.RLock()
    tester.KNOWN_HEARTBEATS = set()
    state_lock = threading.Lock()
    start = threading.Barrier(3)
    operations = []
    pending_motors: queue.SimpleQueue[int] = queue.SimpleQueue()

    class FakeOut:
        def write(self, frame, timeout):
            del timeout
            # NovoRouter owns endpoint reads; correlate the fake reply through
            # the router's single reader rather than the caller thread.
            raw = bytes(frame)
            motor = int(raw[8])
            with state_lock:
                operations.append(("write", motor))
            pending_motors.put(motor)
            return len(frame)

    class FakeIn:
        def read(self, size, timeout):
            del size, timeout
            try:
                motor = pending_motors.get_nowait()
            except queue.Empty as exc:
                raise usb.core.USBTimeoutError("router idle") from exc
            time.sleep(0.03)
            with state_lock:
                operations.append(("read", motor))
            return novo_encode(
                bytes((0, 0, 0, 0, 8, BioXpTester.BOARD_HEAD, 100, 5, 0, 0, 0, motor, 0))
            )

    tester.ep_out = FakeOut()
    tester.ep_in = FakeIn()
    tester.novo_router = NovoRouter(
        ep_in=tester.ep_in,
        ep_out=tester.ep_out,
        decode=novo_decode,
        read_timeout_ms=1,
    )
    tester.novo_router.start()
    results = {}

    def worker(motor):
        start.wait()
        results[motor] = tester.send_tmcl(
            BioXpTester.BOARD_HEAD,
            5,
            4,
            motor,
            123,
            read_timeout_ms=10,
            max_reads=2,
        )

    threads = [threading.Thread(target=worker, args=(motor,)) for motor in (0, 1)]
    for thread in threads:
        thread.start()
    start.wait()
    for thread in threads:
        thread.join(timeout=2.0)

    assert all(not thread.is_alive() for thread in threads)
    assert set(results) == {0, 1}
    assert all(result is not None for result in results.values())
    assert operations in (
        [("write", 0), ("read", 0), ("write", 1), ("read", 1)],
        [("write", 1), ("read", 1), ("write", 0), ("read", 0)],
    )


def _tmcl_frame(*, board: int, status: int, command: int) -> NovoFrame:
    data = bytes((board, status, command, 0, 0, 0, 0, 0))
    return NovoFrame(
        arbitration_id=0,
        dlc=8,
        data=data,
        raw=data,
        received_at=1.0,
        classification="tmcl",
    )


def test_chiller_activation_matcher_preserves_board_identity_without_command_echo():
    matcher = NovoRouter.tmcl_matcher(
        board_id=BioXpTester.BOARD_CHILLER,
        command=64,
        strict=True,
        require_command_echo=False,
    )

    assert matcher(_tmcl_frame(board=7, status=2, command=0)).matched is True
    assert matcher(_tmcl_frame(board=6, status=100, command=64)).matched is False


def test_default_tmcl_matcher_still_requires_command_echo():
    matcher = NovoRouter.tmcl_matcher(board_id=7, command=64, strict=True)

    assert matcher(_tmcl_frame(board=7, status=2, command=0)).matched is False
    assert matcher(_tmcl_frame(board=7, status=2, command=64)).matched is True


def test_oem_board_activation_relaxes_command_echo_only_for_chiller(monkeypatch):
    tester = _bare_tester()
    tester.BOARDS = [4, 5, 6, 7]
    calls = []

    def send_tmcl_retry(board_id, command, cmd_type, motor, value, **kwargs):
        calls.append((board_id, command, kwargs["require_command_echo"]))
        return {"status": 2 if board_id == 7 else 100}

    monkeypatch.setattr(tester, "enable_motor_power", lambda: None)
    monkeypatch.setattr(tester, "send_tmcl_retry", send_tmcl_retry)
    monkeypatch.setattr(time, "sleep", lambda _: None)

    result = tester.activate_boards(expect_reply=True)

    assert result[7]["status"] == 2
    assert calls == [(4, 64, True), (5, 64, True), (6, 64, True), (7, 64, False)]
    assert tester._oem_initialized_boards == {4, 5, 6, 7}
