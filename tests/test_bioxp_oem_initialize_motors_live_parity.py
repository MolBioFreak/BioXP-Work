from src.bioxp.oem_startup_program import BioXpStartupHardware
from src.bioxp.usb_driver import BioXpTester


class _FakeTester:
    BOARD_HEAD = 0x04
    BOARD_DECK = 0x05
    BOARD_THERMAL = 0x06
    MOTOR_HEAD_CLEARANCE_LIFT_ABS = 12500

    def __init__(self):
        self.calls = []

    def motor_get_position(self, board, motor=0):
        return {"position": 0}

    def motor_get_speed(self, board, motor=0):
        return {"speed": 0}

    def motor_get_switch_activity(self, board, motor=0):
        return {"left_state": 0, "right_state": 1}

    def motor_query_24v_sensor(self):
        return {"no24v": False}

    def io_snapshot(self, board):
        return {1: 1, 3: 1}


def test_stepwise_plan_is_oem_initialize_motors_order_not_head_clearance_replacement():
    hw = BioXpStartupHardware(lambda: _FakeTester())

    result = hw.startup_homing_stepwise(mode="shadow", step="plan", execute=False)

    assert result["ok"] is True
    assert [row["step"] for row in result["steps"]] == [
        "z-home",
        "gripper-clear",
        "gripper-home",
        "x-home",
        "x-park-6000",
        "y-home",
        "door-home",
        "y-set-home",
    ]
    assert "head-clearance-z-up" not in [row["step"] for row in result["steps"]]
    assert result["oem_sequence_anchor"].endswith("ClassControlInterface.initializeMotors lines 3348-3421")


def test_stepwise_plan_steps_are_accepted_by_direct_supervised_api_schema():
    from src.bioxp.api import OemStartupStepRequest

    hw = BioXpStartupHardware(lambda: _FakeTester())
    plan = hw.startup_homing_stepwise(mode="shadow", step="plan", execute=False)

    for row in plan["steps"]:
        parsed = OemStartupStepRequest(step=row["step"])
        assert parsed.step == row["step"]


def test_live_commissioning_profiles_are_soft_but_preserve_oem_home_order():
    tester = BioXpTester.__new__(BioXpTester)

    x = tester._motion_oem_axis_profile("x", startup=True)
    y = tester._motion_oem_axis_profile("y", startup=True)
    z = tester._motion_oem_axis_profile("z", startup=True)

    assert x["speed"] == 300
    assert x["acc"] == 120
    assert x["standby_current"] == 20
    assert x["home_speed"] == 250
    assert x["oem_home_step"] == "MotorX.axisSearchHome(speed=250)"

    assert y["speed"] == 300
    assert y["acc"] == 120
    assert y["standby_current"] == 20
    assert y["home_speed"] == 250
    assert y["oem_home_step"] == "MotorY.axisSearchHome(speed=250)"

    assert z["speed"] == 250
    assert z["acc"] == 60
    assert z["standby_current"] == 20
    assert z["home_speed"] == 250
    assert z["positive_down_requires_right_mask"] is True
    assert z["oem_home_step"] == "MotorZ.axisSearchHome(speed=1791) via live Z reference recovery"


def test_z_home_step_skips_reference_return_when_home_fails():
    import pytest
    from fastapi import HTTPException
    from src.bioxp.api import _execute_oem_startup_step

    calls = []

    class Tester:
        def motion_arm_state(self):
            return {"armed": True}

        def motion_gate_live_snapshot(self):
            return {"ok": True}

        def activate_boards(self, expect_reply=False):
            calls.append(("activate", expect_reply))
            return {"ok": True}

        def motor_prepare_motion_interlock(self, force_lock=False):
            calls.append(("interlock", force_lock))
            return {"ok": True}

        def motor_oem_home_axis(self, axis, startup=False, timeout_s=0):
            calls.append(("home", axis, startup, timeout_s))
            return {"home": {"ok": False, "error": "ambiguous switch state"}}

        def motor_oem_move_z_to_reference(self, *args, **kwargs):  # pragma: no cover - must not run on failed home
            calls.append(("z_reference", args, kwargs))
            return {"ok": True}

    with pytest.raises(HTTPException) as exc_info:
        _execute_oem_startup_step(Tester(), "z-home", timeout_s=10)

    assert exc_info.value.status_code == 409
    assert "did not confirm" in str(exc_info.value.detail)
    assert not any(call[0] == "z_reference" for call in calls)


def test_z_reference_return_masks_and_restores_right_limit_without_changing_x_y():
    calls = []

    class Tester(BioXpTester):
        BOARD_HEAD = 0x04
        BOARD_DECK = 0x05
        BOARD_THERMAL = 0x06
        MOTOR_SWITCH_ACTIVE_VALUE = 1

        def motor_get_position(self, board, motor=0):
            calls.append(("get_position", board, motor))
            return {"position": -10000}

        def motor_axis_status(self, board, motor=0):
            calls.append(("status", board, motor))
            return {"position": {"position": 0 if calls.count(("status", board, motor)) else -10000}, "speed": {"speed": 0}, "max_current": {"value": 20}, "switches": {"right_state": 1, "left_state": 0}}

        def motor_stop(self, board, motor=0):
            calls.append(("stop", board, motor))
            return {"ok": True}

        def motor_get_axis_param(self, board, param, motor=0):
            calls.append(("gap", board, motor, param))
            if param == 12:
                return {"value": 0}
            if param == 13:
                return {"value": 0}
            return {"value": 0}

        def motor_set_axis_param(self, board, param, value, motor=0):
            calls.append(("sap", board, motor, param, value))
            return {"ok": True, "param": param, "value": value}

        def motor_move_relative(self, board, steps, motor=0):
            calls.append(("move_relative", board, motor, steps))
            return {"ok": True}

        def motor_wait_stopped(self, board, motor=0, **kwargs):
            calls.append(("wait", board, motor, kwargs))
            return {"stopped": True, "seen_nonzero": True}

        def motor_query_24v_sensor(self):
            return {"no24v": False}

    tester = Tester.__new__(Tester)

    result = tester.motor_oem_move_z_to_reference(target_position=0, timeout_s=10)

    assert result["ok"] is True
    assert result["steps"] == 10000
    assert ("sap", 0x04, 1, 12, 1) in calls  # live Z reference recovery masks GAP10/right during positive-to-0 motion
    assert ("sap", 0x04, 1, 13, 0) in calls
    assert result["right_mask_applied_for_positive_down"] is True
    assert result["disable_right_restore"]["value"] == 0
    assert result["disable_left_restore"]["value"] == 0
    assert not any(call[:3] == ("sap", 0x05, 0) for call in calls)  # no X mutation





def test_full_init_uses_oem_xy_axis_search_not_controller_zero(monkeypatch):
    from src.bioxp.usb_driver import BioXpTester

    calls = []

    class Tester(BioXpTester):
        def _motion_oem_axis_profile(self, axis, startup=False):
            return {
                'x': {'board':5,'motor':0},
                'y': {'board':4,'motor':0,'disable_right': True},
                'z': {'board':4,'motor':1},
                'g': {'board':4,'motor':2,'run_current':31,'standby_current':10,'restore_current':10},
                'door': {'board':6,'motor':0},
            }[axis]

        def motor_oem_axis_already_home(self, axis, tolerance_steps=2):
            return {'ok': True, 'axis': axis}

        def motor_oem_verify_z_clearance_for_xy(self, **kwargs):
            return {'ok': True}

        def motor_oem_axis_search_home(self, axis, **kwargs):
            calls.append(('axis_search_home', axis, kwargs))
            return {'ok': True, 'axis': axis, 'switch_transition': True, 'set_home': {'ok': True}}

        def motor_set_home(self, board, motor=0):
            calls.append(('set_home', board, motor))
            return {'ok': True}

        def motor_set_axis_param(self, board, param, value, motor=0):
            calls.append(('set_axis_param', board, motor, param, value))
            return {'ok': True}

        def motor_move_absolute(self, board, position, motor=0):
            calls.append(('move_absolute', board, motor, position))
            return {'ok': True}

        def motor_wait_stopped(self, board, motor=0, **kwargs):
            calls.append(('wait_stopped', board, motor, kwargs))
            return {'stopped': True}

        def motor_oem_door_search_home(self, **kwargs):
            calls.append(('door_home', kwargs))
            return {'ok': True}

    import src.bioxp.oem_gripper as og
    monkeypatch.setattr(og, 'gripper_status', lambda tester: {'ok': True, 'oem_home_predicate': {'oem_confirmed_home': True}})

    tester = Tester.__new__(Tester)
    result = tester.motor_oem_initialize_motors_full_sequence(timeout_s=30)

    assert result['ok'] is True
    assert result['failed_at'] is None
    assert result['steps'][-6:] == ['x_axisSearchHome_250', 'x_setHome', 'x_setSpeed_1700', 'x_moveX_6000', 'y_axisSearchHome_250', 'thermal_door_doorSearchHome', 'y_setHome'][-6:]
    axis_searches = [call for call in calls if call[0] == 'axis_search_home']
    assert axis_searches[0][1:] == ('x', {'speed': 250, 'timeout_s': 30.0, 'max_search_abs_delta': None})
    assert axis_searches[1][1:] == ('y', {'speed': 250, 'timeout_s': 30.0, 'max_search_abs_delta': None})
    # The only absolute move allowed in this phase is the OEM X park to 6000.
    assert ('move_absolute', 5, 0, 0) not in calls
    assert ('move_absolute', 4, 0, 0) not in calls
    assert ('move_absolute', 5, 0, 6000) in calls
