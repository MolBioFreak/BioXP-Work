import pytest

from src.bioxp.usb_driver import BioXpTester
from support_oem_machine_bundle import serial_206_immutable_machine_bundle


class FakeTester(BioXpTester):
    BOARD_HEAD = 0x04
    BOARD_DECK = 0x05
    BOARD_THERMAL = 0x06
    MOTOR_SWITCH_ACTIVE_VALUE = 1

    def __init__(self, *, position=-1, speed=0, home=1):
        self.position = position
        self.speed = speed
        self.home = home
        self.calls = []

    def _machine_config_bundle(self):
        return serial_206_immutable_machine_bundle()

    def motor_get_position(self, board, motor=0):
        self.calls.append(("pos", board, motor))
        return {"ack": {"status": 100}, "position": self.position}

    def motor_get_speed(self, board, motor=0):
        self.calls.append(("speed", board, motor))
        return {"ack": {"status": 100}, "speed": self.speed}

    def motor_query_home_switch(self, board, motor=0):
        self.calls.append(("home", board, motor))
        return {"ack": {"status": 100}, "value": self.home, "ok": True}

    def motor_get_switch_activity(self, board, motor=0):
        self.calls.append(("switches", board, motor))
        return {"left_raw_active": self.home == 1, "right_raw_active": True, "left_state": self.home, "right_state": 1}


def test_z_already_home_accepts_active_home_top_near_zero_without_motion():
    tester = FakeTester(position=-1, speed=0, home=1)

    result = tester.motor_oem_axis_already_home("z", tolerance_steps=2)

    assert result["ok"] is True
    assert result["already_home"] is True
    assert result["physical_motion_commanded"] is False
    assert result["predicate_active"] is True
    assert result["stopped"] is True
    assert result["near_reference"] is True


def test_z_already_home_fails_closed_when_position_is_not_near_reference():
    tester = FakeTester(position=-100, speed=0, home=1)

    result = tester.motor_oem_axis_already_home("z", tolerance_steps=2)

    assert result["ok"] is False
    assert result["predicate_active"] is True
    assert result["near_reference"] is False


def test_duplicate_startup_homing_authority_is_absent():
    tester = BioXpTester.__new__(BioXpTester)
    assert not hasattr(tester, "motor_startup_homing_mimic")
