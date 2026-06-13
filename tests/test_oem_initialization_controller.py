from src.bioxp.oem_initialization import run_oem_initialization_controller


class FakeInitTester:
    def __init__(self):
        self.calls = []

    def motion_gate_live_snapshot(self):
        self.calls.append("gate")
        return {"ok": True}

    def motor_oem_initialize_without_motion(self):
        self.calls.append("prep")
        return {"ok": True}

    def motor_plan_thermal_door_restore(self, restore_requested=False):
        self.calls.append(("door_plan", restore_requested))
        return {"ok": True, "capture": {"classified": {"state": "closed"}}, "restore_plan": {"ok": True}}

    def motor_oem_axis_already_home(self, axis, tolerance_steps=2):
        self.calls.append(("already_home", axis, tolerance_steps))
        return {"ok": True, "already_home": True, "physical_motion_commanded": False}

    def motor_oem_verify_z_clearance_for_xy(self, **kwargs):
        self.calls.append(("z_clear", kwargs))
        return {"ok": True}

    def _motion_oem_axis_profile(self, axis, startup=True):
        return {"board": 1, "motor": 0}

    def motor_get_switches(self, board, motor=0):
        return {"left_state": 1, "right_state": 1, "left_disabled": False, "right_disabled": False}

    def motor_get_position(self, board, motor=0):
        return {"position": 0}

    def motor_get_speed(self, board, motor=0):
        return {"speed": 0}

    def motor_get_axis_param(self, board, param, motor=0):
        return {"value": 10}

    def motor_query_home_switch(self, board, motor=0):
        return {"value": 1}

    def motor_prepare_axis(self, board, **kwargs):
        self.calls.append(("prepare_axis", board, kwargs))
        return {"ok": True}

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.calls.append(("set", param, value))
        return {"ok": True}

    def motor_move_relative(self, board, steps, motor=0):
        self.calls.append(("move_relative", steps))
        return {"ok": True}

    def motor_wait_stopped(self, board, motor=0, **kwargs):
        self.calls.append(("wait", kwargs))
        return {"ok": True, "stopped": True, "seen_nonzero": True}

    def motor_restore_gripper_idle_current(self, reason=""):
        self.calls.append(("restore_g", reason))
        return {"ok": True}

    def motor_oem_home_axis(self, axis, startup=True, timeout_s=0):
        self.calls.append(("home_axis", axis))
        return {"ok": True, "home": {"ok": True}}

    def motor_oem_home_xy(self, timeout_s=60.0):
        self.calls.append(("homexy", timeout_s))
        return {"ok": True, "home_rebase": {"x": {"home_rebased": True}, "y": {"home_rebased": True}}}

    def motor_oem_door_search_home(self, timeout_s=45.0, startup=False):
        self.calls.append(("door_home", timeout_s, startup))
        return {"ok": True}

    def motor_oem_initialize_motors_full_sequence(self, timeout_s=120.0):
        self.calls.append(("full_initialize_motors", timeout_s))
        return {
            "ok": True,
            "source_mode": "ClassControlInterface.initializeMotors",
            "steps": [
                "z_axisSearchHome_1791",
                "g_moveSteps_plus10000_then_axisSearchHome",
                "x_axisSearchHome_250_setHome_setSpeed1700_moveX6000",
                "y_axisSearchHome_250",
                "thermal_door_doorSearchHome",
            ],
        }

    def motion_arm_state(self):
        return {"armed": True, "reason": "strict_init_pass"}

    def motor_axis_status(self, board, motor=0):
        return {"speed": {"speed": 0}, "position": {"position": 0}}


def test_controller_no_homing_builds_all_phases_without_motion():
    tester = FakeInitTester()

    result = run_oem_initialization_controller(tester, run_homing=False)

    assert result["ok"] is True
    assert result["ready"] is True
    assert "initialize_without_motion" in result["phase_names"]
    assert result["machine_calibration_manifest"]["thermal_door"]["TCDoorOpen"]["value"] == 18500
    assert not any(call == ("homexy", 60.0) for call in tester.calls)


def test_controller_with_homing_runs_full_initialize_motors_phase():
    tester = FakeInitTester()

    result = run_oem_initialization_controller(tester, run_homing=True, timeout_s=90)

    assert result["ok"] is True
    assert result["ready"] is True
    assert "initialize_motors_full_sequence" in result["phase_names"]
    assert ("full_initialize_motors", 90) in tester.calls
    assert not any(isinstance(call, tuple) and call[0] == "homexy" for call in tester.calls)


def test_controller_fails_closed_on_full_initialize_motors_failure():
    class BadFullInit(FakeInitTester):
        def motor_oem_initialize_motors_full_sequence(self, timeout_s=120.0):
            return {"ok": False, "error": "x_not_rebased"}

    result = run_oem_initialization_controller(BadFullInit(), run_homing=True)

    assert result["ok"] is False
    assert result["ready"] is False
    assert result["failed_at"] == "initialize_motors_full_sequence"


class FakeFullInitSequenceTester(FakeInitTester):
    def __init__(self):
        super().__init__()
        self.calls = []

    def motor_oem_home_xy(self, timeout_s=60.0):
        self.calls.append(("homexy",))
        return {"ok": False, "should_not_be_called": True}

    def motor_oem_initialize_motors_full_sequence(self, timeout_s=120.0):
        self.calls.append(("full_initialize_motors", timeout_s))
        return {
            "ok": True,
            "source_mode": "ClassControlInterface.initializeMotors",
            "steps": [
                "z_axisSearchHome_1791",
                "g_moveSteps_plus10000_then_axisSearchHome",
                "x_axisSearchHome_250_setHome_setSpeed1700_moveX6000",
                "y_axisSearchHome_250",
                "thermal_door_doorSearchHome",
            ],
        }


def test_controller_uses_full_initialize_motors_not_homexy_for_run_homing():
    tester = FakeFullInitSequenceTester()

    result = run_oem_initialization_controller(tester, run_homing=True, timeout_s=123)

    assert result["ok"] is True
    assert ("full_initialize_motors", 123) in tester.calls
    assert not any(call[0] == "homexy" for call in tester.calls)
    assert any(
        p.get("phase") == "initialize_motors_full_sequence"
        and p.get("result", {}).get("source_mode") == "ClassControlInterface.initializeMotors"
        for p in result["phases"]
    )
