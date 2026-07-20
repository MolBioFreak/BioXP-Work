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


def test_oem_startup_profiles_preserve_literal_xy_constants_and_masks():
    tester = BioXpTester.__new__(BioXpTester)

    x = tester._motion_oem_axis_profile("x", startup=True)
    y = tester._motion_oem_axis_profile("y", startup=True)

    assert x["speed"] == 1700
    assert x["acc"] == 350
    assert x["home_speed"] == 250
    assert x["oem_home_step"] == "MotorX.axisSearchHome(speed=250)"
    assert "disable_right" not in x
    assert "disable_left" not in x

    assert y["speed"] == 1800
    assert y["acc"] == 400
    assert y["home_speed"] == 250
    assert y["oem_home_step"] == "MotorY.axisSearchHome(speed=250)"
    assert y["disable_right"] is True
    assert "disable_left" not in y


def test_initialize_without_motion_dispatches_literal_xy_profiles_without_x_mask_writes(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    captured = {}
    original_profile = tester._motion_oem_axis_profile

    def profile(axis):
        if axis in {"x", "y"}:
            return original_profile(axis, startup=True)
        return {"board": 4, "motor": 2, "run_current": 10}

    def prepare(board, *, motor=0, **kwargs):
        axis = "x" if board == tester.BOARD_DECK else ("y" if motor == 0 else f"other-{motor}")
        captured[axis] = kwargs
        return {"ok": True, "ops": []}

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", profile)
    monkeypatch.setattr(tester, "motor_prepare_axis", prepare)

    result = tester.motor_oem_initialize_without_motion()

    assert result["ok"] is True
    assert captured["x"]["speed"] == 1700
    assert captured["x"]["acc"] == 350
    assert captured["x"]["disable_right"] is None
    assert captured["x"]["disable_left"] is None
    assert captured["y"]["speed"] == 1800
    assert captured["y"]["acc"] == 400
    assert captured["y"]["disable_right"] is True
    assert captured["y"]["disable_left"] is None


def test_z_home_step_uses_full_init_z_reference_contract_when_probe_fails():
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

        def motor_oem_axis_already_home(self, axis, tolerance_steps=0):
            calls.append(("probe", axis, tolerance_steps))
            if sum(1 for c in calls if c[0] == "probe") == 1:
                return {"ok": False, "position": {"position": -12345}, "home": {"value": 0}}
            return {"ok": True, "position": {"position": 0}, "home": {"value": 1}}

        def motor_oem_move_z_to_reference(self, *args, **kwargs):
            calls.append(("z_reference", args, kwargs))
            return {"ok": True, "target_position": 0}

        def motor_oem_home_axis(self, axis, startup=False, timeout_s=0):  # pragma: no cover - must not run
            raise AssertionError(f"generic startup home route must not run for z-home: {axis} {startup} {timeout_s}")

    result = _execute_oem_startup_step(Tester(), "z-home", timeout_s=10)

    assert result["result"]["ok"] is True
    assert result["result"]["z_axisSearchHome_1791_probe"]["ok"] is False
    assert result["result"]["z_reference_return"]["ok"] is True
    assert result["result"]["z_reference_verify_after_return"]["ok"] is True
    assert any(call[0] == "z_reference" for call in calls)


def test_z_home_step_fails_closed_when_reference_return_does_not_verify_home():
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

        def motor_oem_axis_already_home(self, axis, tolerance_steps=0):
            calls.append(("probe", axis, tolerance_steps))
            return {"ok": False, "position": {"position": -12345}, "home": {"value": 0}}

        def motor_oem_move_z_to_reference(self, *args, **kwargs):
            calls.append(("z_reference", args, kwargs))
            return {"ok": True, "target_position": 0}

        def motor_oem_home_axis(self, axis, startup=False, timeout_s=0):  # pragma: no cover - must not run
            raise AssertionError(f"generic startup home route must not run for z-home: {axis} {startup} {timeout_s}")

    with pytest.raises(HTTPException) as exc_info:
        _execute_oem_startup_step(Tester(), "z-home", timeout_s=10)

    assert exc_info.value.status_code == 409
    assert "did not confirm" in str(exc_info.value.detail)
    assert any(call[0] == "z_reference" for call in calls)


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


def test_z_reference_return_rejects_untrusted_coordinate_before_any_command(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    calls = []
    monkeypatch.setattr(
        tester,
        "_motion_oem_axis_profile",
        lambda axis, startup=False: {
            "board": 0x04,
            "motor": 1,
            "axis_min_steps": -160000,
            "axis_max_steps": 0,
            "home_search_max_abs_delta": 160000,
        },
    )
    monkeypatch.setattr(tester, "motor_get_position", lambda board, motor=0: {"position": 2_000_000})
    monkeypatch.setattr(tester, "motor_stop", lambda *args, **kwargs: calls.append(("stop", args, kwargs)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_set_axis_param", lambda *args, **kwargs: calls.append(("sap", args, kwargs)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_move_relative", lambda *args, **kwargs: calls.append(("move", args, kwargs)) or {"ok": True})

    result = tester.motor_oem_move_z_to_reference(target_position=0, timeout_s=10)

    assert result["ok"] is False
    assert result["error"] == "z_position_outside_machine_envelope"
    assert result["physical_motion_commanded"] is False
    assert calls == []


def test_z_reference_return_requires_observed_motion_and_verified_post_target(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(
        tester,
        "_motion_supervised_signed_z_profile",
        lambda: {
            "board": 0x04,
            "motor": 1,
            "axis_min_steps": -160000,
            "axis_max_steps": 0,
            "home_search_max_abs_delta": 160000,
            "speed": 250,
            "acc": 60,
            "run_current": 31,
            "standby_current": 20,
            "stall_guard": 16,
            "positive_down_requires_right_mask": False,
        },
    )
    monkeypatch.setattr(tester, "motor_get_position", lambda board, motor=0: {"position": -10000})
    monkeypatch.setattr(tester, "motor_stop", lambda board, motor=0: {"ok": True})
    monkeypatch.setattr(tester, "motor_set_axis_param", lambda board, param, value, motor=0: {"ok": True, "value": value})
    monkeypatch.setattr(tester, "motor_move_relative", lambda board, steps, motor=0: {"ok": True, "steps": steps})
    monkeypatch.setattr(
        tester,
        "motor_wait_stopped",
        lambda board, motor=0, **kwargs: {"stopped": True, "seen_nonzero": False},
    )
    monkeypatch.setattr(
        tester,
        "motor_axis_status",
        lambda board, motor=0: {
            "position": {"position": -10000},
            "speed": {"speed": 0},
            "switches": {"right_state": 1, "left_state": 0},
        },
    )

    result = tester.motor_oem_move_z_to_reference(target_position=0, timeout_s=10)

    assert result["ok"] is False
    assert result["failure"] == "z_reference_completion_not_proven"
    assert result["completion_evidence"]["motion_observed"] is False
    assert result["completion_evidence"]["target_position_confirmed"] is False


def test_initialize_without_motion_keeps_gripper_v1_at_idle_current(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    prepared = []

    def fake_prepare(board, *, motor=0, **kwargs):
        prepared.append({"board": board, "motor": motor, **kwargs})
        return {"ops": [{"op": "prepare", "ack": {"status": 100}}]}

    monkeypatch.setattr(tester, "motor_prepare_axis", fake_prepare)

    result = tester.motor_oem_initialize_without_motion()

    assert result["ok"] is True
    gripper = next(row for row in prepared if row["board"] == tester.BOARD_HEAD and row["motor"] == 2)
    assert gripper["run_current"] == 10
    assert gripper["standby_current"] == 10





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
    assert axis_searches[0][1:] == ('z', {'speed': 1791, 'timeout_s': 30.0, 'max_search_abs_delta': None})
    assert axis_searches[1][1:] == ('x', {'speed': 250, 'timeout_s': 30.0, 'max_search_abs_delta': None})
    assert axis_searches[2][1:] == ('y', {'speed': 250, 'timeout_s': 30.0, 'max_search_abs_delta': None})
    # The only absolute move allowed in this phase is the OEM X park to 6000.
    assert ('move_absolute', 5, 0, 0) not in calls
    assert ('move_absolute', 4, 0, 0) not in calls
    assert ('move_absolute', 5, 0, 6000) in calls



def test_axis_search_home_uses_oem_sethome_and_queryhome_contract_for_x(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1

    recorded = {}

    def fake_profile(axis, startup=False):
        assert axis == "x"
        return {"board": 5, "motor": 0, "home_search_max_abs_delta": 91919}

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", fake_profile)
    monkeypatch.setattr(tester, "motor_set_home", lambda board, motor=0: {"ok": True, "board": board, "motor": motor})
    monkeypatch.setattr(tester, "motor_query_home_switch", lambda board, motor=0: {"value": 0})
    monkeypatch.setattr(tester, "motor_get_switch_activity", lambda board, motor=0: {"left_state": 0, "right_state": 1})

    def fake_go_home(axis, **kwargs):
        recorded.update(kwargs)
        return {"ok": True, "home_after": {"value": 1}, "set_home": {"ok": True}, "switch_transition": True}

    monkeypatch.setattr(tester, "motor_oem_go_home", fake_go_home)

    result = tester.motor_oem_axis_search_home("x", speed=250, timeout_s=30, max_search_abs_delta=91919)

    assert result["sethome_init"]["ok"] is True
    assert recorded["require_switch_transition"] is True
    assert recorded["max_search_abs_delta"] == 91919
    assert result["ok"] is True


def test_full_initialize_motors_passes_bounded_x_and_y_search(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    calls = []

    monkeypatch.setattr(tester, "motor_oem_axis_already_home", lambda axis, tolerance_steps=2: {"ok": True})
    monkeypatch.setattr(tester, "motor_oem_verify_z_clearance_for_xy", lambda **kwargs: {"ok": True})

    import src.bioxp.oem_gripper as oem_gripper
    monkeypatch.setattr(oem_gripper, "gripper_status", lambda tester_arg: {"ok": True, "oem_home_predicate": {"oem_confirmed_home": True}})

    def fake_profile(axis, startup=True):
        limits = {"x": 91919, "y": 95247, "z": 160000, "g": 15000, "door": 18500}
        return {"board": 5 if axis == "x" else 4, "motor": 0, "home_search_max_abs_delta": limits[axis]}

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", fake_profile)

    def fake_axis_search(axis, **kwargs):
        calls.append((axis, kwargs))
        return {"ok": True, "home_after": {"value": 1}, "set_home": {"ok": True}}

    monkeypatch.setattr(tester, "motor_oem_axis_search_home", fake_axis_search)
    monkeypatch.setattr(tester, "motor_set_home", lambda board, motor=0: {"ok": True})
    monkeypatch.setattr(tester, "motor_set_axis_param", lambda board, param, value, motor=0: {"ok": True})
    monkeypatch.setattr(tester, "motor_move_absolute", lambda board, position, motor=0: {"ok": True})
    monkeypatch.setattr(tester, "motor_wait_stopped", lambda board, motor=0, **kwargs: {"stopped": True})
    monkeypatch.setattr(tester, "motor_oem_door_search_home", lambda **kwargs: {"ok": True})

    result = tester.motor_oem_initialize_motors_full_sequence(timeout_s=90)

    assert result["ok"] is True
    assert ("x", {"speed": 250, "timeout_s": 45.0, "max_search_abs_delta": 91919}) in calls
    assert ("y", {"speed": 250, "timeout_s": 45.0, "max_search_abs_delta": 95247}) in calls


def test_initialize_without_motion_has_top_level_ok(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)

    profiles = {
        "x": {"board": 5, "motor": 0},
        "y": {"board": 4, "motor": 0},
        "z": {"board": 4, "motor": 1},
        "g": {"board": 4, "motor": 2},
        "door": {"board": 6, "motor": 0},
    }
    monkeypatch.setattr(tester, "_motion_oem_axis_profile", lambda axis: profiles[axis])
    monkeypatch.setattr(tester, "motor_prepare_axis", lambda board, **kwargs: {"board": board, "ops": [{"op": "sap6", "ack": {"status": 100}}]})
    monkeypatch.setattr(tester, "_tmcl_success", lambda ack: ack.get("status") == 100)

    result = tester.motor_oem_initialize_without_motion()

    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert set(result["axes"]) == {"x", "y", "z", "g", "door"}
