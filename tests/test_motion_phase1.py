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
    assert tester.prepare_interlock_calls == 0
    assert board_status["5"]["ok"] is True
    assert interlock["reused"] is True
    assert prep["ok"] is True
    assert policy["reuse_requested"] is True
    assert policy["reuse_allowed"] is False
    assert policy["reuse_used"] is False
    assert policy["board_activation_skipped"] is False
    assert policy["axis_prep_skipped"] is False


def test_prepare_motion_axis_allows_debug_reuse_but_still_activates_boards(monkeypatch):
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
    assert tester.prepare_interlock_calls == 0
    assert board_status["5"]["ok"] is True
    assert interlock["reused"] is True
    assert prep["reused"] is True
    assert policy["reuse_requested"] is True
    assert policy["reuse_allowed"] is True
    assert policy["reuse_used"] is True
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
