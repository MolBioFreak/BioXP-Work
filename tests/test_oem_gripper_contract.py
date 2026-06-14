import pytest
from fastapi import HTTPException

from src.bioxp import api


class FakeTester:
    MOTOR_SWITCH_ACTIVE_VALUE = 1

    def __init__(self, *, left=1, right=1, left_disabled=False, right_disabled=False, position=0, speed=0, run=10, standby=10, move_ok=True, wait_stopped=True):
        self.left = left
        self.right = right
        self.left_disabled = left_disabled
        self.right_disabled = right_disabled
        self.position = position
        self.speed = speed
        self.current = {(4, 6, 2): run, (4, 7, 2): standby}
        self.calls = []
        self.move_ok = move_ok
        self.wait_stopped = wait_stopped

    def _motion_oem_axis_profile(self, axis, startup=False):
        assert axis == "g"
        return {"board": 4, "motor": 2, "speed": 600, "acc": 5, "stall_guard": 5, "run_current": 31, "standby_current": 10, "restore_current": 10, "rdiv": 6, "pdiv": 2, "home_speed": 600, "gripper_version": 0}

    def motor_function_preset(self, axis):
        return self._motion_oem_axis_profile(axis)

    def motor_get_switches(self, board, motor=0):
        return {"left_state": self.left, "right_state": self.right, "left_disabled": self.left_disabled, "right_disabled": self.right_disabled}

    def motor_get_switch_activity(self, board, motor=0):
        return api._switch_activity_from_switches(self, board, motor, self.motor_get_switches(board, motor=motor))

    def motor_query_home_switch(self, board, motor=0):
        return {"ok": True, "value": self.left}

    def motor_get_position(self, board, motor=0):
        return {"ok": True, "position": self.position}

    def motor_get_speed(self, board, motor=0):
        return {"ok": True, "speed": self.speed}

    def motor_get_axis_param(self, board, param, motor=0):
        return {"ok": True, "value": self.current.get((board, param, motor), 0)}

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.calls.append(("set", board, param, value, motor))
        self.current[(board, param, motor)] = value
        return {"ok": True, "param": param, "value": value}

    def motor_prepare_axis(self, board, **kwargs):
        self.calls.append(("prepare", board, kwargs))
        return {"ok": True, "board": board, **kwargs}

    def motor_move_relative(self, board, steps, motor=0):
        self.calls.append(("move_relative", board, steps, motor))
        if self.move_ok:
            self.position += int(steps)
        return {"ok": self.move_ok, "steps": steps}

    def motor_wait_stopped(self, board, motor=0, timeout_s=12.0, require_seen_nonzero=False):
        self.calls.append(("wait", board, motor, timeout_s, require_seen_nonzero))
        return {"ok": self.wait_stopped, "stopped": self.wait_stopped, "seen_nonzero": self.wait_stopped}

    def motor_oem_home_axis(self, axis, startup=False, timeout_s=15.0):
        self.calls.append(("home", axis, startup, timeout_s))
        return {"ok": True, "home": {"ok": True, "position_after": {"position": 0}}}

    def motor_restore_gripper_idle_current(self, reason=""):
        self.calls.append(("restore_idle", reason))
        self.current[(4, 7, 2)] = 10
        self.current[(4, 6, 2)] = 10
        return {"ok": True, "reason": reason, "run_current_param6": {"value": 10}, "standby_current_param7": {"value": 10}}


def test_gripper_status_reports_gap10_diagnostic_and_oem_near_zero_predicate():
    from src.bioxp.oem_gripper import gripper_status

    payload = gripper_status(FakeTester(position=49))

    assert payload["physical_motion"] is False
    assert payload["switches"]["both_effective_limits_active"] is True
    assert "both_effective_limits_active" not in payload["blockers"]
    assert payload["oem_home_predicate"]["position_lt_50"] is True
    assert payload["oem_home_predicate"]["oem_confirmed_home"] is True
    assert payload["current"]["idle_safe"] is True


def test_gripper_clear_does_not_hard_block_on_generic_both_effective_limits_active():
    from src.bioxp.oem_gripper import gripper_clear

    tester = FakeTester(left=1, right=1)

    payload = gripper_clear(tester, operator_ack="GRIPPER_CLEAR", reason="supervised test")

    assert payload["ok"] is True
    assert payload["before"]["switches"]["both_effective_limits_active"] is True
    assert payload["before"]["gap10_motion_gate"]["hard_blocker_removed"] is True
    assert any(call[0] == "move_relative" for call in tester.calls)


def test_gripper_clear_requires_ack_and_reason_before_motion():
    from src.bioxp.oem_gripper import gripper_clear

    tester = FakeTester(left=0, right=0)

    with pytest.raises(HTTPException):
        gripper_clear(tester, operator_ack="", reason="supervised test")
    with pytest.raises(HTTPException):
        gripper_clear(tester, operator_ack="GRIPPER_CLEAR", reason="")
    assert tester.calls == []


def test_gripper_clear_asserts_profile_moves_and_restores_idle_current():
    from src.bioxp.oem_gripper import gripper_clear

    tester = FakeTester(left=0, right=0, run=31, standby=31)

    payload = gripper_clear(tester, operator_ack="GRIPPER_CLEAR", reason="supervised test")

    assert payload["ok"] is True
    assert any(call[0] == "prepare" for call in tester.calls)
    assert any(call[0] == "move_relative" and call[2] == 10000 for call in tester.calls)
    assert tester.current[(4, 6, 2)] == 10
    assert tester.current[(4, 7, 2)] == 10


def test_gripper_home_requires_ack_and_routes_through_oem_home_with_restore():
    from src.bioxp.oem_gripper import gripper_home

    tester = FakeTester(left=0, right=0, run=31, standby=31)

    payload = gripper_home(tester, operator_ack="GRIPPER_HOME", reason="supervised test")

    assert payload["ok"] is True
    assert any(call[0] == "home" and call[1] == "g" for call in tester.calls)
    assert tester.current[(4, 6, 2)] == 10
    assert tester.current[(4, 7, 2)] == 10



def test_gripper_profile_reports_machine_config_positions_and_source_anchor():
    from src.bioxp.oem_gripper import gripper_status

    payload = gripper_status(FakeTester(position=49))
    profile = payload["profile"]

    assert profile["machine_positions"]["originOffsetG"] == 4450
    assert profile["machine_positions"]["close"] == 27350
    assert profile["machine_positions"]["open"] == 31400
    assert profile["machine_positions"]["open_wide"] == 32400
    assert profile["machine_positions"]["source"] == "original_ssd_machine_config"
    assert profile["provenance"]["source_anchor"]["name"] == "MotorGrip home/confirm/machine positions"



def test_gripper_home_accepts_final_query_home_even_with_both_limits_active():
    from src.bioxp.oem_gripper import gripper_home

    class FinalQueryHomeTester(FakeTester):
        def __init__(self):
            super().__init__(left=0, right=1, position=0, run=31, standby=31)
            self.home_called = False

        def motor_oem_home_axis(self, axis, startup=False, timeout_s=15.0):
            self.calls.append(("home", axis, startup, timeout_s))
            self.home_called = True
            # Simulate old nested payload reporting failure/ambiguous, while the
            # final OEM queryHome status proves home after motion.
            self.left = 1
            self.right = 1
            self.position = -17925
            return {"ok": False, "home": {"ok": False, "false_home_guard": "both_effective_limits_active_diagnostic"}}

    tester = FinalQueryHomeTester()

    payload = gripper_home(tester, operator_ack="GRIPPER_HOME", reason="operator watched physical home")

    assert payload["ok"] is True
    assert payload["acceptance"]["query_home_active"] is True
    assert payload["acceptance"]["accepted_by"] == "queryHome(MotorGrip)"
    assert payload["acceptance"]["both_effective_limits_active_is_diagnostic"] is True
    assert payload["after_status"]["switches"]["both_effective_limits_active"] is True
    assert tester.current[(4, 6, 2)] == 10
    assert tester.current[(4, 7, 2)] == 10



def test_gripper_clear_masks_both_g_limits_while_moving_off_home():
    from src.bioxp.oem_gripper import gripper_clear

    tester = FakeTester(left=1, right=1, left_disabled=False, right_disabled=False, position=-17925)

    payload = gripper_clear(tester, operator_ack="GRIPPER_CLEAR", reason="operator move down")

    assert payload["ok"] is True
    assert payload["position_delta"] == 10000
    assert any(call == ("set", 4, 12, 1, 2) for call in tester.calls)
    assert any(call == ("set", 4, 13, 1, 2) for call in tester.calls)
    assert payload["limit_mask"]["disable_right_restore"]["value"] == 0
    assert payload["limit_mask"]["disable_left_restore"]["value"] == 0
