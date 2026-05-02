import importlib
import sys
import types


class FakeTester:
    def __init__(self, *, armed: bool = True, live: bool = True):
        self.armed = armed
        self.live = live
        self.activate_boards_calls = 0
        self.prepare_axis_calls = 0
        self.prepare_interlock_calls = 0
        self.current_values = {}
        self.current_ops = []

    def motor_function_preset(self, axis: str):
        return {
            "label": axis.upper(),
            "board": 5,
            "motor": 0,
            "speed": 1700,
            "acc": 350,
            "run_current": 31,
            "standby_current": 10,
            "stall_guard": 16,
            "disable_right": False,
            "disable_left": False,
            "warm_enable": True,
        }

    def motion_arm_state(self):
        return {"armed": self.armed, "reason": "cached-strict-startup"}

    def motion_gate_live_snapshot(self):
        return {"ok": self.live, "door_closed": True}

    def activate_boards(self, expect_reply: bool = True):
        self.activate_boards_calls += 1
        return {"5": {"ok": True, "expect_reply": expect_reply}}

    def motor_prepare_motion_interlock(self, force_lock: bool = True):
        self.prepare_interlock_calls += 1
        return {"ok": True, "force_lock": force_lock}

    def motor_prepare_axis(self, board: int, **kwargs):
        self.prepare_axis_calls += 1
        return {"ok": True, "board": board, **kwargs}

    def motor_get_position(self, board: int, motor: int = 0):
        return {"ok": True, "position": 100}

    def motor_get_switch_activity(self, board: int, motor: int = 0):
        return {"left_active": False, "right_active": False}

    def motor_move_relative(self, board: int, steps: int, motor: int = 0):
        return {"ok": True, "board": board, "motor": motor, "steps": steps}

    def motor_get_axis_param(self, board: int, param: int, motor: int = 0):
        self.current_ops.append(("get", board, param, motor))
        return {
            "ok": True,
            "board": board,
            "param": param,
            "motor": motor,
            "value": self.current_values.get((board, param, motor), 0),
            "debug_extra": "should be trimmed from operator payload",
        }

    def motor_set_axis_param(self, board: int, param: int, value: int, motor: int = 0):
        self.current_ops.append(("set", board, param, motor, value))
        self.current_values[(board, param, motor)] = value
        return {
            "ok": True,
            "board": board,
            "param": param,
            "motor": motor,
            "set_value": value,
            "ack": {"board": board, "cmd": 5, "status": 100, "status_str": "OK", "value": value, "extra": "trim"},
        }

    def motor_get_speed(self, board: int, motor: int = 0):
        self.current_ops.append(("speed", board, motor))
        return {"ok": True, "board": board, "motor": motor, "speed": 0}


def load_api(monkeypatch):
    usb_pkg = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_pkg.core = usb_core
    usb_pkg.util = usb_util
    monkeypatch.setitem(sys.modules, "usb", usb_pkg)
    monkeypatch.setitem(sys.modules, "usb.core", usb_core)
    monkeypatch.setitem(sys.modules, "usb.util", usb_util)
    for name in ["src.bioxp.api", "src.bioxp.usb_driver", "src.bioxp"]:
        sys.modules.pop(name, None)
    return importlib.import_module("src.bioxp.api")


def test_prepare_motion_axis_ignores_reuse_without_debug_flag(monkeypatch):
    monkeypatch.delenv("BIOXP_ENABLE_PREP_REUSE_DEBUG", raising=False)
    api = load_api(monkeypatch)
    tester = FakeTester(armed=True, live=True)

    _, board_status, interlock, prep, policy = api._prepare_motion_axis(
        tester,
        api.AxisName.X,
        reuse_prepared=True,
    )

    assert tester.activate_boards_calls == 1
    assert tester.prepare_axis_calls == 1
    assert tester.prepare_interlock_calls == 1
    assert board_status["5"]["ok"] is True
    assert interlock["ok"] is True
    assert interlock["force_lock"] is True
    assert prep["ok"] is True
    assert policy["reuse_requested"] is True
    assert policy["reuse_allowed"] is False
    assert policy["reuse_used"] is False
    assert policy["interlock_reused"] is False
    assert policy["board_activation_skipped"] is False
    assert policy["axis_prep_skipped"] is False


def test_prepare_motion_axis_allows_debug_reuse_but_still_runs_fresh_interlock_wake(monkeypatch):
    monkeypatch.setenv("BIOXP_ENABLE_PREP_REUSE_DEBUG", "1")
    api = load_api(monkeypatch)
    tester = FakeTester(armed=True, live=True)

    _, board_status, interlock, prep, policy = api._prepare_motion_axis(
        tester,
        api.AxisName.X,
        reuse_prepared=True,
    )

    assert tester.activate_boards_calls == 1
    assert tester.prepare_axis_calls == 0
    assert tester.prepare_interlock_calls == 1
    assert board_status["5"]["ok"] is True
    assert interlock["ok"] is True
    assert interlock["force_lock"] is True
    assert prep["reused"] is True
    assert policy["reuse_requested"] is True
    assert policy["reuse_allowed"] is True
    assert policy["reuse_used"] is True
    assert policy["interlock_reused"] is False
    assert policy["board_activation_skipped"] is False
    assert policy["axis_prep_skipped"] is True


def test_relative_move_response_exposes_truth_metadata(monkeypatch):
    monkeypatch.delenv("BIOXP_ENABLE_PREP_REUSE_DEBUG", raising=False)
    api = load_api(monkeypatch)
    tester = FakeTester(armed=True, live=True)

    monkeypatch.setattr(
        api,
        "_wait_for_motion_with_guardrails",
        lambda *args, **kwargs: {
            "ok": True,
            "elapsed_ms": 125,
            "position_after": {"position": 112},
            "switch_activity_after": {"left_active": False, "right_active": False},
        },
    )

    result = api._execute_relative_move(
        tester,
        api.AxisName.X,
        steps=12,
        wait_timeout_s=5.0,
        reuse_prepared=True,
    )

    assert result["prep_policy"]["reuse_requested"] is True
    assert result["prep_policy"]["reuse_allowed"] is False
    assert result["motion_truth"]["evidence_level"] == "controller_only"
    assert result["motion_truth"]["independent_evidence_required"] is True
    assert result["motion_truth"]["physical_motion_confirmed"] is False


def test_set_motion_axis_currents_caps_standby_and_does_not_move(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeTester()

    result = api._set_motion_axis_currents(
        tester,
        [api.AxisName.X],
        run_current=20,
        standby_current=31,
    )

    assert result["ok"] is True
    assert result["motion_commanded"] is False
    assert result["axes"]["x"]["requested"] == {"run_current_param6": 20, "standby_current_param7": 31}
    assert result["axes"]["x"]["applied"] == {"run_current_param6": 20, "standby_current_param7": 20}
    assert result["axes"]["x"]["after"]["param6"]["value"] == 20
    assert result["axes"]["x"]["after"]["param7"]["value"] == 20
    assert "debug_extra" not in result["axes"]["x"]["before"]["param6"]
    assert "extra" not in result["axes"]["x"]["writes"]["param6"]["ack"]
    assert tester.current_ops == [
        ("get", 5, 6, 0),
        ("get", 5, 7, 0),
        ("set", 5, 7, 0, 20),
        ("set", 5, 6, 0, 20),
        ("get", 5, 6, 0),
        ("get", 5, 7, 0),
        ("speed", 5, 0),
    ]


def test_set_motion_axis_currents_rejects_non_gantry_axes(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeTester()

    try:
        api._set_motion_axis_currents(tester, [api.AxisName.GRIPPER], run_current=20, standby_current=20)
    except api.HTTPException as exc:
        assert exc.status_code == 400
        assert "gantry x/y/z" in str(exc.detail)
    else:
        raise AssertionError("expected non-gantry axis rejection")
