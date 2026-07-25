from typing import cast

from src.bioxp.oem_startup_program import BioXpStartupHardware
from src.bioxp.usb_driver import BioXpTester


def _bound_oem_machine_config():
    return {
        "ok": True,
        "config": {
            "axis_limits": {
                "x": {"max_steps": 91919},
                "y": {"max_steps": 95247},
                "z": {"max_steps": 160000},
            }
        },
    }


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
        "gripper-current-31",
        "gripper-clear-10000",
        "gripper-home",
        "x-home",
        "x-park-6000",
        "y-home",
        "door-home",
        "door-closed-predicate",
        "y-set-home",
        "ui-zero-calibrated",
        "chiller-oc-cool-rate",
        "chiller-rc-cool-rate",
        "system-status-initialized",
        "gripper-idle-current-10",
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


def test_oem_startup_profiles_preserve_literal_xy_constants_and_masks(monkeypatch):
    monkeypatch.setattr("src.bioxp.usb_driver.find_oem_machine_config_bundle", _bound_oem_machine_config)
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



def test_stepwise_z_home_executes_only_the_oem_axis_search_home_primitive():
    calls = []

    class Tester(_FakeTester):
        def motor_oem_home_axis(self, axis, *, startup=False, timeout_s=0):
            calls.append(("motor_oem_home_axis", axis, startup, timeout_s))
            return {"axis": axis, "startup": startup, "home": {"ok": True}}

        def motor_oem_move_z_to_reference(self, *args, **kwargs):  # pragma: no cover - forbidden by OEM stage contract
            raise AssertionError("Z reference-return is not part of ClassControlInterface.initializeMotors")

    result = BioXpStartupHardware(lambda: Tester()).startup_homing_stepwise(
        mode="live",
        step="z-home",
        execute=True,
    )

    assert result["ok"] is True
    assert result["step"]["oem_anchor"] == "initializeMotors: MotorZ.axisSearchHome(speed=1791)"
    assert calls == [("motor_oem_home_axis", "z", True, 30.0)]
    assert "z_reference_return" not in result


def test_stepwise_z_plan_does_not_describe_a_non_oem_right_limit_correction():
    plan = BioXpStartupHardware(lambda: _FakeTester()).startup_homing_stepwise(
        mode="shadow",
        step="plan",
        execute=False,
    )

    z_step = next(row for row in plan["steps"] if row["step"] == "z-home")
    assert "live_semantic_correction" not in z_step


def test_stepwise_gripper_current_and_clear_are_distinct_oem_stages():
    calls = []

    class Tester(_FakeTester):
        def _motion_oem_axis_profile(self, axis, *, startup=False):
            assert (axis, startup) == ("g", True)
            return {"board": 4, "motor": 2}

        def motor_set_axis_param(self, board, param, value, motor=0):
            calls.append(("set_param", board, param, value, motor))
            return {"ok": True}

        def motor_move_relative(self, board, steps, motor=0):
            calls.append(("move_relative", board, steps, motor))
            return {"ok": True}

        def motor_wait_stopped(self, board, motor=0, **kwargs):
            calls.append(("wait", board, motor, kwargs))
            return {"stopped": True}

    hardware = BioXpStartupHardware(lambda: Tester())
    current = hardware.startup_homing_stepwise(mode="live", step="gripper-current-31", execute=True)
    clear = hardware.startup_homing_stepwise(mode="live", step="gripper-clear-10000", execute=True)

    assert current["ok"] is True
    assert clear["ok"] is True
    assert calls == [
        ("set_param", 4, 6, 31, 2),
        ("move_relative", 4, 10000, 2),
        ("wait", 4, 2, {"timeout_s": 20.0, "require_seen_nonzero": False}),
    ]


def test_terminal_ui_zero_is_only_a_calibrated_source_branch_and_never_a_durable_claim():
    class Tester(_FakeTester):
        def _machine_config_bundle(self):
            return {"ok": True, "config": {"calibration": {"Calibrated": 0}}}

    uncalibrated = BioXpStartupHardware(lambda: Tester()).startup_homing_stepwise(
        mode="live", step="ui-zero-calibrated", execute=True
    )

    assert uncalibrated["ok"] is True
    assert uncalibrated["source_branch_taken"] is False
    assert uncalibrated["ui_writes"] == []
    assert "durable_robot_state" not in uncalibrated

    class CalibratedTester(Tester):
        def _machine_config_bundle(self):
            return {"ok": True, "config": {"calibration": {"Calibrated": 1}}}

    blocked = BioXpStartupHardware(lambda: CalibratedTester()).startup_homing_stepwise(
        mode="live", step="ui-zero-calibrated", execute=True
    )

    assert blocked["ok"] is False
    assert blocked["source_branch_taken"] is True
    assert blocked["expected_ui_writes"] == [("x", "0"), ("y", "0"), ("z", "0"), ("z", "0")]
    assert blocked["physical_motion"] is False
    assert "calibrated_ui_position_sink_not_bound" in blocked["blockers"]


def test_terminal_chiller_stages_fail_closed_with_distinct_literal_gp8_minus_25_contracts():
    opened = []

    class Tester(_FakeTester):
        pass

    def factory():
        opened.append(True)
        return Tester()

    hardware = BioXpStartupHardware(factory)
    oc = hardware.startup_homing_stepwise(mode="live", step="chiller-oc-cool-rate", execute=True)
    rc = hardware.startup_homing_stepwise(mode="live", step="chiller-rc-cool-rate", execute=True)

    assert opened == []
    assert oc["ok"] is False
    assert rc["ok"] is False
    assert oc["literal_oem_gp8_write"] == {"board": "BOARD_CHILLER", "command": 9, "type": 8, "bank": 1, "value": -25}
    assert rc["literal_oem_gp8_write"] == {"board": "BOARD_CHILLER", "command": 9, "type": 8, "bank": 0, "value": -25}
    assert oc["blockers"] == ["single_stage_chiller_transport_not_source_bound"]
    assert rc["blockers"] == ["single_stage_chiller_transport_not_source_bound"]


def test_terminal_gripper_idle_current_uses_10_only_for_gripper_version_one():
    calls = []

    class VersionOneTester(_FakeTester):
        def _motion_oem_axis_profile(self, axis, *, startup=False):
            assert (axis, startup) == ("g", True)
            return {"board": 4, "motor": 2, "gripper_version": 1}

        def motor_set_axis_param(self, board, param, value, motor=0):
            calls.append((board, param, value, motor))
            return {"ok": True}

    current = BioXpStartupHardware(lambda: VersionOneTester()).startup_homing_stepwise(
        mode="live", step="gripper-idle-current-10", execute=True
    )

    assert current["ok"] is True
    assert current["source_branch_taken"] is True
    assert calls == [(4, 6, 10, 2)]

    class VersionZeroTester(VersionOneTester):
        def _motion_oem_axis_profile(self, axis, *, startup=False):
            return {"board": 4, "motor": 2, "gripper_version": 0}

    skipped = BioXpStartupHardware(lambda: VersionZeroTester()).startup_homing_stepwise(
        mode="live", step="gripper-idle-current-10", execute=True
    )

    assert skipped["ok"] is True
    assert skipped["source_branch_taken"] is False
    assert calls == [(4, 6, 10, 2)]


def test_stepwise_gripper_home_uses_serial_206_version_1_oem_search_speed():
    calls = []

    class Tester(_FakeTester):
        def motor_oem_home_axis(self, axis, *, startup=False, speed=None, timeout_s=0):
            calls.append((axis, startup, speed, timeout_s))
            return {"axis": axis, "startup": startup, "home": {"ok": True}}

    result = BioXpStartupHardware(lambda: Tester()).startup_homing_stepwise(
        mode="live",
        step="gripper-home",
        execute=True,
    )

    assert result["ok"] is True
    assert result["step"]["oem_anchor"].endswith("speed=600|200)")
    assert calls == [("g", True, 200, 30.0)]


def test_stepwise_x_home_and_park_preserve_literal_oem_calls_and_waits(monkeypatch):
    from src.bioxp import oem_startup_program

    calls = []
    sleeps = []
    monkeypatch.setattr(oem_startup_program.time, "sleep", lambda seconds: sleeps.append(seconds))

    class Tester(_FakeTester):
        def motor_oem_home_axis(self, axis, *, startup=False, speed=None, timeout_s=0):
            calls.append(("home", axis, startup, speed, timeout_s))
            return {"axis": axis, "home": {"ok": True}}

        def _motion_oem_axis_profile(self, axis, *, startup=False):
            assert (axis, startup) == ("x", True)
            return {"board": 5, "motor": 0}

        def motor_set_home(self, board, motor=0):
            calls.append(("set_home", board, motor))
            return {"ok": True}

        def motor_set_axis_param(self, board, param, value, motor=0):
            calls.append(("set_param", board, param, value, motor))
            return {"ok": True}

        def motor_move_absolute(self, board, value, motor=0):
            calls.append(("move_absolute", board, value, motor))
            return {"ok": True}

        def motor_wait_stopped(self, board, motor=0, **kwargs):
            calls.append(("wait", board, motor, kwargs))
            return {"stopped": True, "seen_nonzero": True}

    hardware = BioXpStartupHardware(lambda: Tester())
    x_home = hardware.startup_homing_stepwise(mode="live", step="x-home", execute=True)
    x_park = hardware.startup_homing_stepwise(mode="live", step="x-park-6000", execute=True)

    assert x_home["ok"] is True
    assert x_park["ok"] is True
    assert calls == [
        ("home", "x", True, 250, 30.0),
        ("set_home", 5, 0),
        ("set_param", 5, 4, 1700, 0),
        ("move_absolute", 5, 6000, 0),
        ("wait", 5, 0, {"timeout_s": 12.0, "require_seen_nonzero": True}),
    ]
    assert sleeps == [0.02, 0.04]


def test_stepwise_y_door_and_y_set_home_use_their_distinct_oem_primitives():
    calls = []

    class Tester(_FakeTester):
        def motor_oem_home_axis(self, axis, *, startup=False, speed=None, timeout_s=0):
            calls.append(("axis_home", axis, startup, speed, timeout_s))
            return {"axis": axis, "home": {"ok": True}}

        def motor_oem_door_search_home(self, *, startup=False, timeout_s=0):
            calls.append(("door_search", startup, timeout_s))
            return {"ok": True, "home": {"ok": True}}

        def _motion_oem_axis_profile(self, axis, *, startup=False):
            assert (axis, startup) == ("y", True)
            return {"board": 4, "motor": 0}

        def motor_set_home(self, board, motor=0):
            calls.append(("set_home", board, motor))
            return {"ok": True}

    hardware = BioXpStartupHardware(lambda: Tester())
    y_home = hardware.startup_homing_stepwise(mode="live", step="y-home", execute=True)
    door = hardware.startup_homing_stepwise(mode="live", step="door-home", execute=True)
    y_set = hardware.startup_homing_stepwise(mode="live", step="y-set-home", execute=True)

    assert y_home["ok"] is True
    assert door["ok"] is True
    assert y_set["ok"] is True
    assert calls == [
        ("axis_home", "y", True, 250, 30.0),
        ("door_search", True, 30.0),
        ("set_home", 4, 0),
    ]


def test_raw_z_home_step_executes_only_oem_axis_search_home(monkeypatch):
    from src.bioxp import api

    monkeypatch.setattr(api, "_require_motion_not_blocked_by_maintenance", lambda: None)
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

        def motor_oem_home_axis(self, axis, *, startup=False, timeout_s=0):
            calls.append(("motor_oem_home_axis", axis, startup, timeout_s))
            return {"axis": axis, "home": {"ok": True}}

        def motor_oem_axis_already_home(self, axis, tolerance_steps=0):
            assert any(call[0] == "motor_oem_home_axis" for call in calls), "pre-home probe is not part of OEM initializeMotors Z stage"
            return {"ok": True}

        def motor_oem_move_z_to_reference(self, *args, **kwargs):  # pragma: no cover - forbidden by source contract
            raise AssertionError("Z reference-return is not part of ClassControlInterface.initializeMotors")

    result = api._execute_oem_startup_step(cast(api.BioXpTester, Tester()), "z-home", timeout_s=30.0)

    assert result["result"]["home"]["ok"] is True
    assert calls == [
        ("activate", True),
        ("interlock", True),
        ("motor_oem_home_axis", "z", True, 30.0),
    ]


def test_raw_x_park_preserves_oem_pre_sethome_and_pre_move_delays(monkeypatch):
    from src.bioxp import api

    monkeypatch.setattr(api, "_require_motion_not_blocked_by_maintenance", lambda: None)
    sleeps = []
    monkeypatch.setattr(api.time, "sleep", lambda seconds: sleeps.append(seconds))

    class Tester:
        def motion_arm_state(self):
            return {"armed": True}

        def motion_gate_live_snapshot(self):
            return {"ok": True}

        def activate_boards(self, expect_reply=False):
            return {"ok": True}

        def motor_prepare_motion_interlock(self, force_lock=False):
            return {"ok": True}

        def _motion_oem_axis_profile(self, axis, startup=False):
            assert (axis, startup) == ("x", True)
            return {"board": 5, "motor": 0}

        def motor_set_home(self, board, motor=0):
            assert (board, motor) == (5, 0)
            return {"ok": True}

        def motor_set_axis_param(self, board, param, value, motor=0):
            assert (board, param, value, motor) == (5, 4, 1700, 0)
            return {"ok": True}

        def motor_move_absolute(self, board, value, motor=0):
            assert (board, value, motor) == (5, 6000, 0)
            return {"ok": True}

        def motor_wait_stopped(self, board, motor=0, **kwargs):
            return {"stopped": True}

    result = api._execute_oem_startup_step(cast(api.BioXpTester, Tester()), "x-park-6000", timeout_s=30.0)

    assert result["result"]["move_x_6000"]["ok"] is True
    assert sleeps == [0.02, 0.04]


def test_monolithic_full_initialize_motors_is_fail_closed_pending_literal_oem_rewrite(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)

    def forbidden_profile(*_args, **_kwargs):  # pragma: no cover - must never be reached by a blocked surface
        raise AssertionError("blocked full sequence must not inspect/configure an axis or command motion")

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", forbidden_profile)

    result = tester.motor_oem_initialize_motors_full_sequence(timeout_s=30.0)

    assert result == {
        "ok": False,
        "source_mode": "ClassControlInterface.initializeMotors",
        "physical_motion_commanded": False,
        "blocked": True,
        "blocked_reason": "literal_direct_oem_stage_rewrite_pending",
    }


def test_oem_rehome_is_fail_closed_pending_literal_oem_rewrite(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "motor_startup_homing_mimic", lambda: (_ for _ in ()).throw(AssertionError("must not call legacy mimic")))

    result = tester.motor_oem_rehome(timeout_s=30.0)

    assert result == {
        "ok": False,
        "source_mode": "ControlLib.rehome",
        "physical_motion_commanded": False,
        "blocked": True,
        "blocked_reason": "literal_direct_oem_stage_rewrite_pending",
    }


def test_startup_door_search_home_preserves_oem_active_home_preclear(monkeypatch):
    """ClassThermalBoard.doorSearchHome pre-clears +2000 when queryHome is active."""
    tester = BioXpTester.__new__(BioXpTester)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1
    calls = []

    monkeypatch.setattr(
        tester,
        "_motion_oem_axis_profile",
        lambda axis: {"board": 6, "motor": 0, "stall_guard": 9, "stall_guard_param": 205, "home_speed": 600},
    )
    monkeypatch.setattr(tester, "motor_thermal_door_status", lambda: {"home": {"value": 1}, "closed": True, "opened": False, "switches": {}})
    monkeypatch.setattr(tester, "motor_set_axis_param", lambda board, param, value, motor=0: calls.append(("stall", board, param, value, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_move_relative", lambda board, steps, motor=0: calls.append(("relative", board, steps, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_move_left", lambda board, speed, motor=0: calls.append(("left", board, speed, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_wait_stopped", lambda *args, **kwargs: {"stopped": True, "ambiguous_no_motion": False})
    monkeypatch.setattr(tester, "motor_stop", lambda board, motor=0: calls.append(("stop", board, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_set_home", lambda board, motor=0: calls.append(("set_home", board, motor)) or {"ok": True})

    result = tester.motor_oem_door_search_home(startup=True)

    assert result["ok"] is True
    assert calls[:4] == [
        ("stall", 6, 205, 11, 0),
        ("relative", 6, 2000, 0),
        ("stall", 6, 205, 9, 0),
        ("left", 6, 600, 0),
    ]


def test_z_reference_return_masks_and_restores_right_limit_without_changing_x_y(monkeypatch):
    monkeypatch.setattr("src.bioxp.usb_driver.find_oem_machine_config_bundle", _bound_oem_machine_config)
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
    monkeypatch.setattr("src.bioxp.usb_driver.find_oem_machine_config_bundle", _bound_oem_machine_config)
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
