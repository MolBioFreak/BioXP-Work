import threading
import time

import pytest
import usb.core

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
    pending_motor_by_thread = {}

    class FakeOut:
        def write(self, frame, timeout):
            motor = int(frame[8])
            ident = threading.get_ident()
            with state_lock:
                pending_motor_by_thread[ident] = motor
                operations.append(("write", motor))
            time.sleep(0.03)
            return len(frame)

    class FakeIn:
        def read(self, size, timeout):
            del size, timeout
            ident = threading.get_ident()
            with state_lock:
                motor = pending_motor_by_thread.get(ident)
            if motor is None:
                raise usb.core.USBTimeoutError("drain empty")
            time.sleep(0.03)
            with state_lock:
                operations.append(("read", motor))
                pending_motor_by_thread.pop(ident, None)
            return [0x7E, 0, 0, 0, 0, 8, BioXpTester.BOARD_HEAD, 100, 5, 0, 0, 0, motor, 0, 0x7E]

    tester.ep_out = FakeOut()
    tester.ep_in = FakeIn()
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
