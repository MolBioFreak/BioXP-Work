from __future__ import annotations

import itertools

import pytest

from bioxp.usb_driver import BioXpTester


ACK = {"status": 100, "value": 0}


def _driver():
    driver = object.__new__(BioXpTester)
    driver._oem_board_state = lambda: {5: True}
    driver.oem_no24v_state = lambda: False
    driver._motion_oem_axis_profile = lambda axis, startup=False: {
        "axis_min_steps": 60,
        "axis_max_steps": 90263,
    }
    driver.motor_axis_key_for_channel = lambda board, motor=0: "x"
    driver.motor_query_motor_stop = lambda board, motor=0: {"ok": True, "ack": ACK}
    driver.begin_bus_event_window = lambda: {"after_sequence": 10}
    driver.motor_get_position = lambda board, motor=0: {"ok": True, "ack": ACK, "position": 1000}
    driver.motor_get_speed = lambda board, motor=0: {"ok": True, "ack": ACK, "speed": 0}
    driver.motor_oem_wait_target_reached = lambda board, motor=0, **kwargs: {
        "ok": True,
        "target_reached": True,
        "event": {"status": 128, "board": 5, "motor": 0, "event_sequence": 11},
        "events": [{"status": 128, "board": 5, "motor": 0, "event_sequence": 11}],
    }
    return driver


def test_x_absolute_rejects_outside_release_envelope_before_dispatch(monkeypatch):
    driver = _driver()
    monkeypatch.setattr(driver, "_send_motor", lambda *a, **k: (_ for _ in ()).throw(AssertionError("dispatch")))

    low = driver.motor_oem_move_absolute(5, -1, motor=0)
    high = driver.motor_oem_move_absolute(5, 90264, motor=0)

    assert low["ok"] is False and low["command_sent"] is False
    assert high["ok"] is False and high["command_sent"] is False


def test_x_absolute_clamps_release_low_coordinates_to_effective_60(monkeypatch):
    driver = _driver()
    sent = []
    monkeypatch.setattr(driver, "_send_motor", lambda *args, **kwargs: sent.append((args, kwargs)) or ACK)
    monkeypatch.setattr(driver, "motor_get_position", lambda board, motor=0: {"ok": True, "ack": ACK, "position": 1000 if not sent else 60})

    result = driver.motor_oem_move_absolute(5, 0, motor=0, wait_for_stop=True)

    assert result["ok"] is True
    assert result["wire_position"] == 60
    assert len(sent) == 1
    assert result["proof"] == {
        "direct_ack": True,
        "addressed_event_128": True,
        "target_position": True,
        "speed_zero": True,
    }


def test_x_absolute_missing_ack_never_replays(monkeypatch):
    driver = _driver()
    sent = []
    monkeypatch.setattr(driver, "_send_motor", lambda *args, **kwargs: sent.append((args, kwargs)))

    result = driver.motor_oem_move_absolute(5, 2000, motor=0)

    assert result["ok"] is False
    assert result["failure"] == "x_direct_movement_ack_required"
    assert result["uncertain_delivery"] is True
    assert len(sent) == 1


def test_x_launch_ticket_never_claims_completion(monkeypatch):
    driver = _driver()
    sent = []
    monkeypatch.setattr(driver, "_send_motor", lambda *args, **kwargs: sent.append((args, kwargs)) or ACK)

    result = driver.motor_oem_move_absolute(5, 2000, motor=0, wait_for_stop=False)

    assert result["ok"] is True
    assert result["pending_motion"] is True
    assert result["completion_verified"] is False
    assert result["proof"]["direct_ack"] is True
    assert result["proof"]["addressed_event_128"] is False
    assert len(sent) == 1


def test_x_wait_rejects_fresh_board_level_fault_without_motor(monkeypatch):
    driver = _driver()
    monkeypatch.setattr(driver, "collect_bus_events", lambda **kwargs: [
        {"event_sequence": 11, "board": 5, "status": 13, "motor": None},
    ])

    result = BioXpTester.motor_oem_wait_target_reached(
        driver,
        5,
        motor=0,
        timeout_s=0.01,
        event_window={"after_sequence": 10},
    )

    assert result["ok"] is False
    assert result["failure"] == "controller_async_error_13"


@pytest.mark.parametrize(
    "events",
    [
        [{"event_sequence": 10, "board": 5, "status": 128, "motor": 0}],
        [{"board": 5, "status": 128, "motor": 0}],
        [{"event_sequence": 11, "board": 4, "status": 128, "motor": 0}],
        [{"event_sequence": 11, "board": 5, "status": 128, "motor": 1}],
    ],
)
def test_x_wait_rejects_stale_missing_sequence_wrong_board_and_wrong_motor(monkeypatch, events):
    driver = _driver()
    batches = itertools.chain([events], itertools.repeat([]))
    monkeypatch.setattr(driver, "collect_bus_events", lambda **kwargs: next(batches))

    result = BioXpTester.motor_oem_wait_target_reached(
        driver,
        5,
        motor=0,
        timeout_s=0.001,
        event_window={"after_sequence": 10},
    )

    assert result["ok"] is False
    assert result["target_reached"] is False
    assert result["failure"] == "oem_moveToAbs_target_event_timeout"


def test_x_wait_rejects_late_decoded_status_128_received_before_command_dispatch(monkeypatch):
    driver = _driver()
    batches = itertools.chain([
        [{"event_sequence": 11, "board": 5, "status": 128, "motor": 0, "received_at": 99.9}],
    ], itertools.repeat([]))
    monkeypatch.setattr(driver, "collect_bus_events", lambda **kwargs: next(batches))

    result = BioXpTester.motor_oem_wait_target_reached(
        driver,
        5,
        motor=0,
        timeout_s=0.001,
        event_window={"after_sequence": 10, "dispatch_cursors": {"5:0": 100.0}},
    )

    assert result["ok"] is False
    assert result["target_reached"] is False
    assert result["failure"] == "oem_moveToAbs_target_event_timeout"


def test_x_wait_accepts_only_fresh_correctly_addressed_128(monkeypatch):
    driver = _driver()
    monkeypatch.setattr(driver, "collect_bus_events", lambda **kwargs: [
        {"event_sequence": 11, "board": 5, "status": 128, "motor": 0},
    ])

    result = BioXpTester.motor_oem_wait_target_reached(
        driver,
        5,
        motor=0,
        timeout_s=0.01,
        event_window={"after_sequence": 10},
    )

    assert result["ok"] is True
    assert result["target_reached"] is True
    assert result["event"]["event_sequence"] == 11


def test_x_switch_mask_recovery_writes_both_masks_and_verifies_both(monkeypatch):
    driver = _driver()
    writes = []
    values = {12: 0, 13: 0}

    def get_param(board, param, motor=0):
        return {"ok": True, "ack": ACK, "value": values[param]}

    def set_param(board, param, value, motor=0):
        writes.append((board, motor, param, value))
        values[param] = value
        return {"ok": True, "ack": ACK, "readback": {"ok": True, "ack": ACK, "value": value}}

    monkeypatch.setattr(driver, "motor_get_axis_param", get_param)
    monkeypatch.setattr(driver, "motor_set_axis_param", set_param)

    result = driver.motor_x_reconcile_switch_masks()

    assert result["ok"] is True
    assert writes == [(5, 0, 12, 0), (5, 0, 13, 0)]
    assert result["switch_masks"] == {12: 0, 13: 0}


def _current_driver(monkeypatch):
    driver = _driver()
    driver._oem_board_presence = {4: True, 5: True, 6: True, 7: True}
    writes = []

    def set_param(board, param, value, motor=0):
        writes.append((int(board), int(motor), int(param), int(value)))
        return {
            "ok": True,
            "ack": ACK,
            "readback": {"ok": True, "ack": ACK, "value": int(value)},
        }

    monkeypatch.setattr(driver, "motor_set_axis_param", set_param)
    return driver, writes


def test_enable_xy_exact_current_order_and_wait(monkeypatch):
    driver, writes = _current_driver(monkeypatch)
    waits = []

    result = driver.motor_oem_set_xy_current_mode(True, sleep_fn=waits.append)

    assert result["ok"] is True
    assert writes == [(5, 0, 6, 10), (4, 0, 6, 10), (5, 0, 6, 31), (4, 0, 6, 31)]
    assert waits == [0.500]
    assert result["gripper_current_written"] is False
    assert result["physical_motion_commanded"] is False


def test_disable_xy_sets_only_x_y_current_one(monkeypatch):
    driver, writes = _current_driver(monkeypatch)

    result = driver.motor_oem_set_xy_current_mode(False, sleep_fn=lambda _value: None)

    assert result["ok"] is True
    assert writes == [(5, 0, 6, 1), (4, 0, 6, 1)]


def test_enable_xyz_has_two_waits_then_z_and_never_gripper(monkeypatch):
    driver, writes = _current_driver(monkeypatch)
    waits = []

    result = driver.motor_oem_set_xyz_current_mode(True, z_current_up=29, sleep_fn=waits.append)

    assert result["ok"] is True
    assert writes == [
        (5, 0, 6, 10), (4, 0, 6, 10),
        (5, 0, 6, 31), (4, 0, 6, 31),
        (4, 1, 6, 29),
    ]
    assert waits == [0.500, 0.500]
    assert all(not (board == 5 and motor == 1) for board, motor, _param, _value in writes)
    assert result["gripper_current_written"] is False


def test_disable_xyz_sets_x_y_z_one(monkeypatch):
    driver, writes = _current_driver(monkeypatch)

    result = driver.motor_oem_set_xyz_current_mode(False, sleep_fn=lambda _value: None)

    assert result["ok"] is True
    assert writes == [(5, 0, 6, 1), (4, 0, 6, 1), (4, 1, 6, 1)]


def test_enable_xy_skips_absent_board_and_stops_after_first_failed_readback(monkeypatch):
    driver, writes = _current_driver(monkeypatch)
    driver._oem_board_presence[4] = False

    def fail_first(board, param, value, motor=0):
        writes.append((int(board), int(motor), int(param), int(value)))
        return {"ok": False, "ack": {"status": 2}, "readback": {"ok": False, "ack": {"status": 2}, "value": 0}}

    monkeypatch.setattr(driver, "motor_set_axis_param", fail_first)
    result = driver.motor_oem_set_xy_current_mode(True, sleep_fn=lambda _value: pytest.fail("wait after failed write"))

    assert result["ok"] is False
    assert writes == [(5, 0, 6, 10)]


def test_enable_xyz_stops_before_later_xy_and_z_writes_after_failed_ack(monkeypatch):
    driver, writes = _current_driver(monkeypatch)

    def fail_first(board, param, value, motor=0):
        writes.append((int(board), int(motor), int(param), int(value)))
        return {"ok": False, "ack": {"status": 2}, "readback": {"ok": False, "ack": {"status": 2}, "value": int(value)}}

    monkeypatch.setattr(driver, "motor_set_axis_param", fail_first)
    result = driver.motor_oem_set_xyz_current_mode(True, z_current_up=29, sleep_fn=lambda _value: pytest.fail("wait after failed write"))

    assert result["ok"] is False
    assert writes == [(5, 0, 6, 10)]
