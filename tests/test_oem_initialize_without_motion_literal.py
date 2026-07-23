from __future__ import annotations

from src.bioxp import usb_driver
from src.bioxp.usb_driver import BioXpTester


def _literal_tester(monkeypatch, *, gripper_version: int = 1):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "motor_oem_wait_for_board", lambda: {"ok": True, "polls": 1})
    monkeypatch.setattr(
        tester,
        "motor_function_preset",
        lambda axis: {
            "x": {"board": tester.BOARD_DECK, "motor": 0},
            "y": {"board": tester.BOARD_HEAD, "motor": 0},
            "z": {"board": tester.BOARD_HEAD, "motor": 1},
            "g": {"board": tester.BOARD_HEAD, "motor": 2},
            "door": {"board": tester.BOARD_THERMAL, "motor": 0},
        }[axis],
    )
    values = {
        "m_Z_MOTOR_MAX_CURRENT_UP": 31,
        "m_Z_MOTOR_STALL_GUARD_THRESHOLD": 3,
        "m_TC_DOOR_VELOCITY": 50,
        "m_TC_DOOR_ACCELERATION": 20,
        "m_TC_DOOR_MAX_CURRENT": 31,
        "m_TCDoorStallGuardThreshold": 6,
    }
    monkeypatch.setattr(
        tester,
        "_machine_config_offset_int",
        lambda key, fallback: (values[key], "immutable_oem_machine_snapshot"),
    )
    monkeypatch.setattr(tester, "_motion_oem_gripper_version", lambda: gripper_version)
    wire = []

    def tx(**row):
        wire.append(row)
        return {**row, "ack": {"status": 100}, "ok": True}

    monkeypatch.setattr(tester, "_oem_no_motion_tmcl", tx)
    sleeps = []
    monkeypatch.setattr(usb_driver.time, "sleep", lambda seconds: sleeps.append(seconds))
    return tester, wire, sleeps


def test_initialize_without_motion_emits_literal_oem_wire_order(monkeypatch):
    tester, wire, sleeps = _literal_tester(monkeypatch, gripper_version=1)

    result = tester.motor_oem_initialize_without_motion()

    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert result["wire_contract"] == "literal_oem_order_no_generic_axis_helper"
    assert result["machine_values"]["z_stall_guard"]["value"] == 3
    assert [(row["name"], row["board"], row["command"], row["cmd_type"], row["motor"], row["value"]) for row in wire] == [
        ("turnOffHeater.tc_pwm_0.first", 0x06, 144, 0, 0, 0),
        ("turnOffHeater.tc_pwm_0.second", 0x06, 144, 0, 0, 0),
        ("setChillerPWM.OC", 0x07, 144, 0, 1, 0),
        ("setChillerPWM.RC", 0x07, 144, 0, 0, 0),
        ("x.setMaxSpeed", 0x05, 5, 4, 0, 1700),
        ("x.setMaxAcc", 0x05, 5, 5, 0, 350),
        ("x.setMaxCurrent", 0x05, 5, 6, 0, 31),
        ("x.setStallGuardThreshold", 0x05, 5, 205, 0, 16),
        ("y.setMaxSpeed", 0x04, 5, 4, 0, 1800),
        ("y.setMaxAcc", 0x04, 5, 5, 0, 400),
        ("y.setMaxCurrent", 0x04, 5, 6, 0, 31),
        ("y.setStallGuardThreshold", 0x04, 5, 205, 0, 16),
        ("y.disableRightSwitch", 0x04, 5, 12, 0, 1),
        ("z.setMaxSpeed", 0x04, 5, 4, 1, 1791),
        ("z.setMaxAcc", 0x04, 5, 5, 1, 576),
        ("z.setMaxCurrent", 0x04, 5, 6, 1, 31),
        ("z.readMaxCurrent", 0x04, 6, 6, 1, 0),
        ("z.setStallGuardThreshold", 0x04, 5, 205, 1, 3),
        ("g.setMaxSpeed", 0x04, 5, 4, 2, 1500),
        ("g.setMaxAcc", 0x04, 5, 5, 2, 20),
        ("g.setMaxCurrent", 0x04, 5, 6, 2, 10),
        ("g.setStallGuardThreshold", 0x04, 5, 205, 2, 20),
        ("g.setRdiv", 0x04, 5, 153, 2, 6),
        ("g.setPdiv", 0x04, 5, 154, 2, 2),
        ("door.setMaxSpeed", 0x06, 5, 4, 0, 50),
        ("door.setMaxAcc", 0x06, 5, 5, 0, 20),
        ("door.setMaxCurrent", 0x06, 5, 6, 0, 31),
        ("door.setStallGuardThreshold", 0x06, 5, 205, 0, 6),
        ("door.disableRightSwitch", 0x06, 5, 12, 0, 1),
        ("door.disableLeftSwitch", 0x06, 5, 13, 0, 1),
        ("setChillerCoolRate.OC", 0x07, 9, 8, 1, -25),
        ("setChillerCoolRate.RC", 0x07, 9, 8, 0, -25),
        ("thermal.setTCHeatRate", 0x06, 9, 7, 0, 2500),
        ("thermal.setTCCoolRate", 0x06, 9, 8, 0, -2000),
        ("setColor.white.r", 0x05, 50, 0, 0, 1024),
        ("setColor.white.g", 0x05, 50, 0, 1, 1024),
        ("setColor.white.b", 0x05, 50, 0, 2, 1024),
    ]
    assert not [
        row for row in wire
        if row["command"] == 5 and row["cmd_type"] in (7, 212)
    ]
    assert sleeps == [0.001] + [0.002] * 19


def test_initialize_without_motion_issues_no_write_after_first_failed_ack(monkeypatch):
    tester, wire, _ = _literal_tester(monkeypatch, gripper_version=1)

    def fail_first(**row):
        wire.append(row)
        return {**row, "ack": {"status": 2}, "ok": False}

    monkeypatch.setattr(tester, "_oem_no_motion_tmcl", fail_first)

    result = tester.motor_oem_initialize_without_motion()

    assert result["ok"] is False
    assert [row["name"] for row in wire] == ["turnOffHeater.tc_pwm_0.first"]
    assert result["failures"] == [
        {"name": "turnOffHeater.tc_pwm_0.first", "ack": {"status": 2}}
    ]


def test_strict_startup_led_write_stops_without_fallback_or_later_channels(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    writes = []

    def failed_ack(board, command, cmd_type, motor, value, **_kwargs):
        writes.append((board, command, cmd_type, motor, value))
        return {"status": 2}

    monkeypatch.setattr(tester, "send_tmcl_retry", failed_ack)
    monkeypatch.setattr(
        tester,
        "send_tmcl",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("strict startup LED must not issue a fallback write")
        ),
    )
    monkeypatch.setattr(
        tester,
        "activate_boards",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("initialCheck LED write must not activate boards implicitly")
        ),
    )

    result = tester.strip_set_rgb(
        255, 255, 255, reconnect_first=False, activate_first=False, fail_fast=True
    )

    assert result["ok"] is False
    assert len(writes) == 1
    assert list(result["acks"]) == ["r"]
    assert result["sent"] == 1


def test_wait_for_board_activation_stops_after_first_failed_board(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    tester._oem_initialized_boards = set()
    writes = []
    clock, sleep = _fake_clock()

    monkeypatch.setattr(tester, "enable_motor_power", lambda: None)

    def failed_ack(board, *_args, **_kwargs):
        writes.append(board)
        return None

    monkeypatch.setattr(tester, "send_tmcl_retry", failed_ack)

    result = tester.motor_oem_wait_for_board(
        timeout_s=2.0,
        poll_interval_s=0.1,
        activation_every_polls=1,
        clock=clock,
        sleep=sleep,
    )

    assert result["ok"] is False
    assert result["error"] == "waitForBoard_activation_failed"
    assert writes == [tester.BOARDS[0]]
    assert list(result["activations"][0]) == [tester.BOARDS[0]]


def test_non_chiller_thermal_activation_rejects_status_two(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "_send_thermal", lambda *_args, **_kwargs: {"status": 2})

    result = tester.thermal_activate()

    assert result == {"ack": {"status": 2}, "ok": False}


def test_initialize_without_motion_preserves_gripper_version_zero_literals(monkeypatch):
    tester, wire, _ = _literal_tester(monkeypatch, gripper_version=0)

    result = tester.motor_oem_initialize_without_motion()

    assert result["ok"] is True
    gripper = {row["name"]: row["value"] for row in wire if row["name"].startswith("g.")}
    assert gripper == {
        "g.setMaxSpeed": 600,
        "g.setMaxAcc": 5,
        "g.setMaxCurrent": 31,
        "g.setStallGuardThreshold": 5,
        "g.setRdiv": 6,
        "g.setPdiv": 2,
    }


def _fake_clock():
    state = {"now": 0.0}

    def clock():
        return state["now"]

    def sleep(seconds):
        state["now"] += seconds

    return clock, sleep


def test_wait_for_board_preserves_oem_100ms_poll_and_31_poll_reactivation(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    tester._oem_initialized_boards = set()
    activations = []
    clock, sleep = _fake_clock()

    def activate(expect_reply=True, fail_fast=False):
        activations.append((expect_reply, fail_fast))
        tester._oem_initialized_boards = set(tester.BOARDS)
        return {board: {"status": 100} for board in tester.BOARDS}

    monkeypatch.setattr(tester, "activate_boards", activate)

    result = tester.motor_oem_wait_for_board(clock=clock, sleep=sleep)

    assert result["ok"] is True
    assert result["polls"] == 31
    assert result["missing_boards"] == []
    assert activations == [(True, True)]


def test_wait_for_board_returns_immediately_when_already_ready(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    tester._oem_initialized_boards = set(tester.BOARDS)
    clock, sleep = _fake_clock()

    result = tester.motor_oem_wait_for_board(clock=clock, sleep=sleep)

    assert result["ok"] is True
    assert result["polls"] == 0
    assert result["elapsed_ms"] == 0


def test_wait_for_board_times_out_with_exact_missing_board_evidence(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    tester.BOARDS = [4, 5, 6, 7]
    tester._oem_initialized_boards = {4, 6}
    activations = []
    clock, sleep = _fake_clock()
    monkeypatch.setattr(
        tester,
        "activate_boards",
        lambda expect_reply=True, fail_fast=False: activations.append(
            (expect_reply, fail_fast)
        )
        or {board: {"status": 100} for board in tester.BOARDS},
    )

    result = tester.motor_oem_wait_for_board(
        timeout_s=0.35,
        poll_interval_s=0.1,
        activation_every_polls=2,
        clock=clock,
        sleep=sleep,
    )

    assert result["ok"] is False
    assert result["error"] == "waitForBoard_timeout"
    assert result["initialized_boards"] == [4, 6]
    assert result["missing_boards"] == [5, 7]
    assert result["physical_motion"] is False
    assert result["motion_commanded"] is False
    assert activations == [(True, True), (True, True)]


def test_initialize_without_motion_stops_before_writes_when_board_wait_fails(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    wait = {
        "ok": False,
        "error": "waitForBoard_timeout",
        "initialized_boards": [4, 5, 6],
        "missing_boards": [7],
        "physical_motion": False,
        "motion_commanded": False,
    }
    monkeypatch.setattr(tester, "motor_oem_wait_for_board", lambda: wait)
    monkeypatch.setattr(
        tester,
        "_oem_no_motion_tmcl",
        lambda **_: (_ for _ in ()).throw(AssertionError("configuration write attempted")),
    )

    result = tester.motor_oem_initialize_without_motion()

    assert result["ok"] is False
    assert result["error"] == "waitForBoard_timeout"
    assert result["blocked_before_configuration_writes"] is True
    assert result["wait_for_board"] == wait
    assert result["physical_motion"] is False
    assert result["motion_commanded"] is False
