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


def test_startup_homing_mimic_does_not_skip_literal_z_axis_search_when_near_reference():
    class StartupTester(FakeTester):
        def reconnect(self, *, hard_reset=False):
            self.calls.append(("reconnect", hard_reset))

        def activate_boards(self, expect_reply=False):
            self.calls.append(("activate", expect_reply))
            return {"ok": True}

        def motor_oem_initialize_without_motion(self):
            self.calls.append(("prep",))
            return {"ok": True}

        def motor_oem_home_axis(self, axis_key, *, speed=None, timeout_s=20.0, startup=False):
            self.calls.append(("searched_home", axis_key, startup))
            return {"home": {"ok": False}}

        def motor_oem_verify_z_clearance_for_xy(self, **kwargs):
            self.calls.append(("z_clear", kwargs))
            return {"ok": False, "forced_abort_after_z_for_test": True}

        def motor_query_24v_sensor(self):
            return {"no24v": False}

    tester = StartupTester(position=-1, speed=0, home=1)

    with pytest.raises(RuntimeError, match="retired duplicate startup authority"):
        tester.motor_startup_homing_mimic()

    assert tester.calls == []


def test_startup_homing_mimic_uses_literal_z_axis_search_not_coordinate_recovery():
    class StartupTester(FakeTester):
        def reconnect(self, *, hard_reset=False):
            self.calls.append(("reconnect", hard_reset))

        def activate_boards(self, expect_reply=False):
            self.calls.append(("activate", expect_reply))
            return {"ok": True}

        def motor_oem_initialize_without_motion(self):
            self.calls.append(("prep",))
            return {"ok": True}

        def motor_oem_move_z_to_reference(self, **kwargs):
            raise AssertionError(f"coordinate recovery must not substitute for OEM Z startup: {kwargs}")

        def motor_oem_home_axis(self, axis_key, *, speed=None, timeout_s=20.0, startup=False):
            self.calls.append(("searched_home", axis_key, startup, speed))
            return {"home": {"ok": True}}

        def motor_oem_verify_z_clearance_for_xy(self, **kwargs):
            self.calls.append(("z_clear", kwargs))
            return {"ok": False, "forced_abort_after_z_for_test": True}

        def motor_query_24v_sensor(self):
            return {"no24v": False}

    tester = StartupTester(position=-100, speed=0, home=0)

    with pytest.raises(RuntimeError, match="retired duplicate startup authority"):
        tester.motor_startup_homing_mimic()

    assert tester.calls == []
