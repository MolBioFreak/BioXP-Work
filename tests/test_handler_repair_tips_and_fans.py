"""Offline regression tests for handler repairs 1 (tip state) and 2 (fan timers)."""
from __future__ import annotations

import asyncio
import threading
import time
from types import SimpleNamespace

import pytest

from bioxp.pipette.transport import PipetteCommandError, PipetteTipStateError
from bioxp.usb_driver import BioXpTester
from tests.pressure_source_v1_support import pressure_source_transport


def _tip_head(monkeypatch, mounted, *, stuck_channels=(), fail_eject_channels=(), invalid_query_channels=()):
    group = pressure_source_transport(loaded_channel=None)
    present = set(mounted)
    stuck = set(stuck_channels)
    fail_eject = set(fail_eject_channels)
    invalid_query = set(invalid_query_channels)
    wire = []

    for channel, transport in enumerate(group._transports):
        driver = transport._get_driver()
        transport._initialized = True

        def exchange(command, *, channel=channel, original=driver._send_pipette_command, **kwargs):
            wire.append((command, channel))
            if command == "?31":
                if channel in invalid_query:
                    return {"ok": False, "semantic_ok": False, "hardware_truth_level": "error"}
                packet = original(command, **kwargs)
                packet["ack"]["data"][-1] = ord("1" if channel in present else "0")
                return packet
            if command == "E1R":
                if channel in fail_eject:
                    return {"ok": False, "outcome": "timeout", "ack": {"outcome": "timeout"}}
                if channel not in stuck:
                    present.discard(channel)
                return {"ok": True, "outcome": "tx_only", "ack": {"outcome": "ack"}}
            return original(command, **kwargs)

        def wait(timeout, channel=channel):
            wire.append(("wait", channel))
            return {"ok": True}

        def eject_tip(initialized=True, wait_for_completion=True, channel=channel):
            wire.append(("E1R_startup", channel))
            if channel in fail_eject:
                return {"ok": False, "ack": {"outcome": "timeout"}}
            if channel not in stuck:
                present.discard(channel)
            return {"ok": True, "ack": {"outcome": "ack"}}

        monkeypatch.setattr(driver, "_send_pipette_command", exchange)
        monkeypatch.setattr(driver, "wait_pipette_command_completion", wait)
        monkeypatch.setattr(driver, "pipette_eject_tip", eject_tip)
    return group, present, stuck, invalid_query, wire


def test_partial_eject_clears_verified_empty_and_keeps_stuck_channel(monkeypatch):
    group, _present, _stuck, _invalid, _wire = _tip_head(monkeypatch, [0, 1, 2, 3], stuck_channels=[3])
    group.query_tip_status_all()
    assert group._tip_location_channels() == [0, 1, 2, 3]

    with pytest.raises(PipetteTipStateError) as exc:
        group.eject_all_tips(check_missing_tip=True, wait=True, channels=None)
    assert exc.value.details["remaining_channels"] == [3]
    assert group._tip_location_channels() == [3]
    assert group._expected_tip_channels == [3]
    assert [ch for ch, t in enumerate(group._transports) if t._tip_loaded] == [3]
    verified = group._verify_standard_tip_selection(group._tip_location_channels())
    assert verified["channels_with_tips"] == [3]


def test_partial_eject_preserves_unselected_loaded_channels(monkeypatch):
    group, _present, _stuck, _invalid, _wire = _tip_head(monkeypatch, [0, 1, 2, 3], stuck_channels=[1])
    group.query_tip_status_all()

    with pytest.raises(PipetteTipStateError) as exc:
        group.eject_all_tips(check_missing_tip=True, wait=True, channels=[0, 1])
    assert exc.value.details["remaining_channels"] == [1]
    assert group._tip_location_channels() == [1, 2, 3]
    assert [ch for ch, t in enumerate(group._transports) if t._tip_loaded] == [1, 2, 3]


def test_startup_eject_clears_verified_empty_on_success_and_partial_failure(monkeypatch):
    # Full success
    group_ok, _p1, _s1, _i1, _w1 = _tip_head(monkeypatch, [0, 1, 2, 3])
    group_ok.query_tip_status_all()
    out = group_ok.eject_all_tips_for_oem_startup(
        operator_ack="EJECT_STALE_STARTUP_TIPS",
        expected_channels_with_tips=[0, 1, 2, 3],
    )
    assert out["ok"] is True
    assert group_ok._expected_tip_channels is None
    assert [ch for ch, t in enumerate(group_ok._transports) if t._tip_loaded] == []

    # Partial failure: channel 3 stuck
    group_partial, _p2, _s2, _i2, _w2 = _tip_head(monkeypatch, [0, 1, 2, 3], stuck_channels=[3])
    group_partial.query_tip_status_all()
    with pytest.raises(PipetteCommandError):
        group_partial.eject_all_tips_for_oem_startup(
            operator_ack="EJECT_STALE_STARTUP_TIPS",
            expected_channels_with_tips=[0, 1, 2, 3],
        )
    assert group_partial._tip_location_channels() == [3]
    assert group_partial._expected_tip_channels == [3]
    assert [ch for ch, t in enumerate(group_partial._transports) if t._tip_loaded] == [3]


def test_startup_eject_ack_failure_invalid_readback_does_not_clear_state(monkeypatch):
    group, _present, _stuck, invalid_query, _wire = _tip_head(
        monkeypatch, [0, 1], fail_eject_channels=[1]
    )
    group.query_tip_status_all()
    assert group._expected_tip_channels == [0, 1]

    # Make post-attempt readback fail on channel 0 after channel 1 fails ack.
    orig_query = group.query_tip_status_all
    calls = {"n": 0}

    def flaky_query(*args, **kwargs):
        calls["n"] += 1
        if calls["n"] >= 2:
            invalid_query.add(0)
        return orig_query(*args, **kwargs)

    monkeypatch.setattr(group, "query_tip_status_all", flaky_query)
    with pytest.raises(PipetteCommandError):
        group.eject_all_tips_for_oem_startup(
            operator_ack="EJECT_STALE_STARTUP_TIPS",
            expected_channels_with_tips=[0, 1],
        )
    assert group._expected_tip_channels == [0, 1]


def test_reconcile_intentional_tip_removal_prevents_false_loss_while_unexplained_remains_reportable(monkeypatch):
    group, present, _stuck, _invalid, _wire = _tip_head(monkeypatch, [0, 1, 2, 3])
    group.query_tip_status_all()
    assert group._expected_tip_channels == [0, 1, 2, 3]

    # Operator intentionally removes tip on channel 1 and invokes the explicit completion hook.
    present.discard(1)
    reconciled = group.reconcile_intentional_tip_removal([1])
    assert reconciled["ok"] is True
    assert reconciled["reconciled_empty_channels"] == [1]
    assert group._expected_tip_channels == [0, 2, 3]
    assert group._tip_location_channels() == [0, 2, 3]

    # Channel 2 disappears WITHOUT intentional reconciliation -> remains reportable evidence.
    present.discard(2)
    out = group.eject_all_tips(check_missing_tip=True, wait=True, channels=None)
    assert out["ok"] is True
    assert out["lost_tip_channels"] == [2]
    assert 1 not in out["lost_tip_channels"]


def test_reconcile_intentional_tip_removal_invalid_readback_does_not_clear_state(monkeypatch):
    group, present, _stuck, invalid_query, _wire = _tip_head(monkeypatch, [0, 1])
    group.query_tip_status_all()
    assert group._expected_tip_channels == [0, 1]

    present.discard(0)
    invalid_query.add(0)
    with pytest.raises(PipetteCommandError):
        group.reconcile_intentional_tip_removal([0])
    assert group._expected_tip_channels == [0, 1]
    assert group._transports[0]._tip_loaded is True


# ---------------------------------------------------------------------------
# Item 2: Fan timer retirement, reconnect races, synchronization & re-arming
# ---------------------------------------------------------------------------


def _make_fake_tester():
    tester = BioXpTester.__new__(BioXpTester)
    tester._transport_lock = threading.RLock()
    tester.dev = SimpleNamespace(label="dev-1", reset=lambda: None)
    tester.ep_out = object()
    tester.ep_in = object()
    tester.novo_router = SimpleNamespace(shutdown=lambda: None, running=True, reader_generation=1)
    tester._receiver_audit = None
    tester._oem_transport_generation = 1
    tester._oem_no_motion_profiles = {}
    tester._oem_board_initialized = {int(b): True for b in BioXpTester.BOARDS}
    tester._oem_motor_home_cache = {}
    tester._motor_last_tx_ts = {int(b): 0.0 for b in BioXpTester.MOTOR_BOARDS}
    tester._motor_noresp_streak = {int(b): 0 for b in BioXpTester.MOTOR_BOARDS}
    tester._chiller_last_tx_ts = 0.0
    tester._chiller_noresp_streak = 0
    tester._thermal_last_tx_ts = 0.0
    tester._thermal_noresp_streak = 0
    tester._oem_thermal_states = {}
    return tester


def test_fan_timer_retired_on_disconnect_does_not_write_or_reschedule_or_turn_off_cooling(monkeypatch):
    import usb.util

    monkeypatch.setattr(usb.util, "release_interface", lambda dev, iface: None)
    monkeypatch.setattr(usb.util, "dispose_resources", lambda dev: None)
    tester = _make_fake_tester()

    scheduled = []

    class FakeTimer:
        def __init__(self, interval, fn, args=()):
            self.interval = interval
            self.fn = fn
            self.args = args
            self.daemon = False
            self.cancelled = False
            self.started = False
            scheduled.append(self)

        def start(self):
            self.started = True

        def cancel(self):
            self.cancelled = True

    monkeypatch.setattr(threading, "Timer", FakeTimer)
    writes = []

    def fake_tmcl(board, cmd, typ, motor, val, **kwargs):
        writes.append((board, cmd, typ, motor, val))
        return {"status": 100, "value": 25000}

    monkeypatch.setattr(tester, "send_tmcl_retry", fake_tmcl)

    tester._oem_fan_service_start(tester.BOARD_THERMAL, 0)
    assert len(scheduled) == 1
    first_timer = scheduled[0]
    writes.clear()

    report = tester._disconnect()
    assert report["ok"] is True
    assert first_timer.cancelled is True
    # Retiring host timers must NOT send cooling/fan-off writes to hardware.
    assert writes == []

    # Invoking the retained callback after disconnect must neither write nor reschedule.
    first_timer.fn(*first_timer.args)
    assert writes == []
    assert len(scheduled) == 1


def test_fan_tick_in_flight_across_reconnect_does_not_write_or_pollute_replacement(monkeypatch):
    import usb.util

    monkeypatch.setattr(usb.util, "release_interface", lambda dev, iface: None)
    monkeypatch.setattr(usb.util, "dispose_resources", lambda dev: None)
    tester = _make_fake_tester()

    scheduled = []

    class FakeTimer:
        def __init__(self, interval, fn, args=()):
            self.interval = interval
            self.fn = fn
            self.args = args
            self.daemon = False
            self.cancelled = False
            scheduled.append(self)

        def start(self):
            pass

        def cancel(self):
            self.cancelled = True

    monkeypatch.setattr(threading, "Timer", FakeTimer)

    connection_id = {"current": 1}
    old_tick_in_read = threading.Event()
    release_old_tick = threading.Event()
    tx_log = []

    def fake_connect():
        connection_id["current"] += 1
        tester.dev = SimpleNamespace(label=f"dev-{connection_id['current']}", reset=lambda: None)
        tester.ep_out = object()
        tester.ep_in = object()
        tester.novo_router = SimpleNamespace(shutdown=lambda: None, running=True, reader_generation=connection_id["current"])

    monkeypatch.setattr(tester, "_connect", fake_connect)

    old_thread_id = {"tid": None}

    def fake_tmcl(board, cmd, typ, motor, val, **kwargs):
        conn = connection_id["current"]
        is_old = threading.get_ident() == old_thread_id["tid"]
        tx_log.append((conn, is_old, board, cmd, motor, val))
        if is_old and cmd == 143:
            # Return an extreme pedestal temp on connection 1 and pause before the write.
            return {"status": 100, "value": 99000}
        return {"status": 100, "value": 26000}

    monkeypatch.setattr(tester, "send_tmcl_retry", fake_tmcl)

    tester._oem_fan_service_start(tester.BOARD_THERMAL, 0)
    old_timer = scheduled[0]

    # Pause old tick AFTER it starts and completes its pedestal read (outside transport guard),
    # right before its next read/write.
    orig_fan_read = tester._oem_fan_read
    read_calls = {"n": 0}

    def paused_fan_read(*args, **kwargs):
        res = orig_fan_read(*args, **kwargs)
        if threading.get_ident() == old_thread_id["tid"]:
            read_calls["n"] += 1
            if read_calls["n"] == 1:
                old_tick_in_read.set()
                assert release_old_tick.wait(timeout=5.0)
        return res

    monkeypatch.setattr(tester, "_oem_fan_read", paused_fan_read)

    def run_old_tick():
        old_thread_id["tid"] = threading.get_ident()
        old_timer.fn(*old_timer.args)

    t = threading.Thread(target=run_old_tick, daemon=True)
    t.start()
    assert old_tick_in_read.wait(timeout=5.0)

    # Reconnect while old tick is mid-flight!
    rec = tester.reconnect()
    assert rec["ok"] is True
    assert connection_id["current"] == 2
    # Reconnect must have re-armed the thermal fan service on connection 2.
    assert len(scheduled) == 2
    new_timer = scheduled[1]
    assert new_timer.cancelled is False

    # Release old tick: it must NOT write to connection 2, NOT reschedule, and NOT pollute state.
    release_old_tick.set()
    t.join(timeout=5.0)
    assert not t.is_alive()

    assert [row for row in tx_log if row[0] == 2 and row[1] is True] == []
    assert len(scheduled) == 2

    # Replacement service runs once cleanly and reports its own state.
    new_timer.fn(*new_timer.args)
    assert len(scheduled) == 3
    status = tester.oem_fan_service_status()["services"]
    thermal_row = next(r for r in status if r["board"] == tester.BOARD_THERMAL and r["axis"] == 0)
    assert thermal_row["running"] is True
    assert thermal_row["error"] is False
    assert thermal_row["pedestal_c"] < 50.0
    assert thermal_row["last_error"] is None


def test_api_claim_and_reconnect_rearm_fan_services(monkeypatch):
    from bioxp import api

    started_boards = []

    class FakeTester:
        BOARD_THERMAL = 8
        BOARD_CHILLER = 10

        def __init__(self, alt=1):
            self.reconnected = 0

        def _oem_ensure_fan_service(self, board):
            started_boards.append((id(self), int(board)))
            return {"board": int(board), "started": True}

        def reconnect(self):
            self.reconnected += 1
            return {"ok": True}

        def _disconnect(self):
            return {"ok": True, "release_interface_ok": True, "dispose_resources_ok": True}

    monkeypatch.setattr(api, "BioXpTester", FakeTester)
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda **kwargs: SimpleNamespace(close=lambda: None))
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "_ownership_changed", lambda **kwargs: None)
    monkeypatch.setattr(api, "_mark_post_maintenance_motion_block", lambda **kwargs: {"ok": True, **kwargs})
    monkeypatch.setattr(api, "_maintenance_state_payload", lambda: {"ok": True})

    asyncio.run(api._claim_service_usb_runtime(source="test_claim", block_motion=False))
    first_owner = api._tester
    assert (id(first_owner), 8) in started_boards and (id(first_owner), 10) in started_boards

    started_boards.clear()
    res = asyncio.run(api.reconnect_runtime())
    assert res["ok"] is True
    assert first_owner.reconnected == 1
    assert (id(first_owner), 8) in started_boards and (id(first_owner), 10) in started_boards
