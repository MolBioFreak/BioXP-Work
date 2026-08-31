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
    driver.begin_bus_event_window = lambda *, reset_wait_latch=True: {
        "after_sequence": 10 if reset_wait_latch else None,
        "oem_wait_latch_reset": bool(reset_wait_latch),
    }
    driver.motor_get_position = lambda board, motor=0: {"ok": True, "ack": ACK, "position": 1000}
    driver.motor_get_speed = lambda board, motor=0: {"ok": True, "ack": ACK, "speed": 0}
    driver.motor_oem_wait_target_reached = lambda board, motor=0, **kwargs: {
        "ok": True,
        "target_reached": True,
        "event": {"status": 128, "board": 5, "motor": 0, "event_sequence": 11},
        "events": [{"status": 128, "board": 5, "motor": 0, "event_sequence": 11}],
    }
    return driver


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


def test_x_wait_uses_oem_wait_latch_sequence_instead_of_custom_dispatch_timestamp(monkeypatch):
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
        event_window={"after_sequence": 10, "oem_wait_latch_reset": True},
    )

    assert result["ok"] is True
    assert result["target_reached"] is True
    assert result["event"]["event_sequence"] == 11


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
    monkeypatch.setattr(
        driver,
        "motor_get_axis_param",
        lambda board, param, motor=0: {
            "ok": True,
            "ack": ACK,
            "value": 0,
        },
    )
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


def test_enable_xy_skips_absent_board_and_preserves_void_source_sequence_after_failed_ack(monkeypatch):
    driver, writes = _current_driver(monkeypatch)
    driver._oem_board_presence[4] = False

    def fail_first(board, param, value, motor=0):
        writes.append((int(board), int(motor), int(param), int(value)))
        return {"ok": False, "ack": {"status": 2}, "readback": {"ok": False, "ack": {"status": 2}, "value": 0}}

    monkeypatch.setattr(driver, "motor_set_axis_param", fail_first)
    waits = []
    result = driver.motor_oem_set_xy_current_mode(True, sleep_fn=waits.append)

    assert result["ok"] is True
    assert result["source_call_completed"] is True
    assert result["controller_command_acknowledged"] is False
    assert writes == [(5, 0, 6, 10), (5, 0, 6, 31)]
    assert waits == [0.500]


def test_enable_xyz_preserves_void_source_sequence_after_failed_ack(monkeypatch):
    driver, writes = _current_driver(monkeypatch)

    def fail_first(board, param, value, motor=0):
        writes.append((int(board), int(motor), int(param), int(value)))
        return {"ok": False, "ack": {"status": 2}, "readback": {"ok": False, "ack": {"status": 2}, "value": int(value)}}

    monkeypatch.setattr(driver, "motor_set_axis_param", fail_first)
    waits = []
    result = driver.motor_oem_set_xyz_current_mode(True, z_current_up=29, sleep_fn=waits.append)

    assert result["ok"] is True
    assert result["source_call_completed"] is True
    assert result["controller_command_acknowledged"] is False
    assert writes == [
        (5, 0, 6, 10), (4, 0, 6, 10),
        (5, 0, 6, 31), (4, 0, 6, 31),
        (4, 1, 6, 29),
    ]
    assert waits == [0.500, 0.500]