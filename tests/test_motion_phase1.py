import importlib
import sys
import types


class FakeTester:
    MOTOR_SWITCH_ACTIVE_VALUE = 1

    def __init__(self, *, armed: bool = True, live: bool = True):
        self.armed = armed
        self.live = live
        self.activate_boards_calls = 0
        self.prepare_axis_calls = 0
        self.prepare_interlock_calls = 0
        self.current_values = {}
        self.current_ops = []
        self.bus_events = []
        self.tail_bus_events = []
        self.clear_bus_event_calls = 0
        self.pop_bus_event_calls = 0
        self.collect_bus_event_calls = 0

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
        motor = kwargs.get("motor", 0)
        return {
            "ok": True,
            "board": board,
            **kwargs,
            "ops": [
                {"op": "gap6-current_before", "set": 7, "rb": {"value": 7}},
                {"op": "sap6-run_current", "set": kwargs.get("run_current"), "rb": {"value": kwargs.get("run_current")}, "ack": {"status": 100}},
                {"op": "sap7-standby_current", "set": kwargs.get("standby_current"), "rb": {"value": kwargs.get("standby_current")}, "ack": {"status": 100}},
                {"op": "sap4-max_speed", "set": kwargs.get("speed"), "rb": {"value": kwargs.get("speed")}, "ack": {"status": 100}},
                {"op": "sap5-max_acc", "set": kwargs.get("acc"), "rb": {"value": kwargs.get("acc")}, "ack": {"status": 100}},
                {"op": "sap205-stall_guard", "set": kwargs.get("stall_guard"), "rb": {"value": kwargs.get("stall_guard")}, "ack": {"status": 100}},
                {"op": "sap12-disable_right", "set": int(bool(kwargs.get("disable_right"))), "rb": {"value": int(bool(kwargs.get("disable_right")))}, "ack": {"status": 100}},
                {"op": "sap13-disable_left", "set": int(bool(kwargs.get("disable_left"))), "rb": {"value": int(bool(kwargs.get("disable_left")))}, "ack": {"status": 100}},
            ],
        }

    def motor_get_position(self, board: int, motor: int = 0):
        return {"ok": True, "position": 100}

    def motor_get_switch_activity(self, board: int, motor: int = 0):
        return {"left_active": False, "right_active": False}

    def motor_move_relative(self, board: int, steps: int, motor: int = 0):
        self.bus_events.append({"board": board, "status": 128, "motor": motor, "cmd": 4, "value": steps, "source": "fake_move_ack_window"})
        return {"ok": True, "board": board, "motor": motor, "steps": steps}

    def clear_bus_event_buffer(self):
        self.clear_bus_event_calls += 1
        self.bus_events = []
        return {"cleared": True}

    def pop_bus_event_buffer(self):
        self.pop_bus_event_calls += 1
        events = list(self.bus_events)
        self.bus_events = []
        return events

    def collect_bus_events(self, duration_s=0.25, timeout_ms=12, max_events=96):
        self.collect_bus_event_calls += 1
        return list(self.tail_bus_events)

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

    def motor_get_switches(self, board: int, motor: int = 0):
        return {"left_state": 0, "right_state": 0, "left_disabled": False, "right_disabled": False}


def mark_x_referenced(api, monkeypatch):
    monkeypatch.setattr(
        api._reference_state_store,
        "snapshot",
        lambda axes=None: {"rows": {"x": {"state": "referenced"}}},
    )


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
    mark_x_referenced(api, monkeypatch)
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
    mark_x_referenced(api, monkeypatch)
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
    mark_x_referenced(api, monkeypatch)
    monkeypatch.setattr(api._reference_state_store, "snapshot", lambda axes=None: {"rows": {"x": {"state": "referenced"}}})
    tester = FakeTester(armed=True, live=True)
    tester.tail_bus_events = [
        {"board": 5, "status": 130, "cmd": 0, "value": 0, "source": "fake_tail_collect"}
    ]

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


def test_set_motion_axis_currents_rejects_hot_standby_without_commissioning_override(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeTester()

    try:
        api._set_motion_axis_currents(
            tester,
            [api.AxisName.X],
            run_current=20,
            standby_current=31,
        )
    except api.HTTPException as exc:
        assert exc.status_code == 409
        assert "standby_current above OEM idle" in str(exc.detail)
    else:
        raise AssertionError("expected hot standby current to require commissioning override")


def test_set_motion_axis_currents_caps_standby_with_commissioning_override_and_does_not_move(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeTester()

    result = api._set_motion_axis_currents(
        tester,
        [api.AxisName.X],
        run_current=20,
        standby_current=31,
        operator_ack=True,
        commissioning_override=True,
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


def test_set_motion_axis_currents_rejects_unsupported_axes(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeTester()

    try:
        api._set_motion_axis_currents(tester, [api.AxisName.THERMAL_DOOR], run_current=10, standby_current=10)
    except api.HTTPException as exc:
        assert exc.status_code == 400
        assert "gantry/gripper x/y/z/g" in str(exc.detail)
    else:
        raise AssertionError("expected unsupported axis rejection")


def test_relative_move_response_exposes_and_rejects_stall_evidence(monkeypatch):
    monkeypatch.delenv("BIOXP_ENABLE_PREP_REUSE_DEBUG", raising=False)
    api = load_api(monkeypatch)
    mark_x_referenced(api, monkeypatch)
    monkeypatch.setattr(api._reference_state_store, "snapshot", lambda axes=None: {"rows": {"x": {"state": "referenced"}}})
    tester = FakeTester(armed=True, live=True)
    tester.tail_bus_events = [
        {"board": 5, "status": 130, "cmd": 0, "value": 0, "source": "fake_tail_collect"}
    ]

    monkeypatch.setattr(
        api,
        "_wait_for_motion_with_guardrails",
        lambda *args, **kwargs: {
            "ok": True,
            "elapsed_ms": 125,
            "last_speed": 0,
            "seen_nonzero": True,
            "position_after": {"ok": True, "position": 112},
            "switch_activity_after": {"left_active": False, "right_active": False},
            "log_tail": [
                {"elapsed_ms": 25, "speed": 30, "position": 104},
                {"elapsed_ms": 125, "speed": 0, "position": 112},
            ],
        },
    )

    result = api._execute_relative_move(
        tester,
        api.AxisName.X,
        steps=12,
        wait_timeout_s=5.0,
        reuse_prepared=False,
    )

    evidence = result["motion_evidence"]
    assert result["ok"] is False
    assert result["motion_failure"]["category"] == "controller_motion_error"
    assert evidence["prep_params"]["run_current_param6"]["set"] == 31
    assert evidence["prep_params"]["standby_current_param7"]["readback"] == 10
    assert evidence["prep_params"]["speed_param4"]["set"] == 1700
    assert evidence["prep_params"]["acc_param5"]["set"] == 350
    assert evidence["prep_params"]["stallguard_param205"]["set"] == 16
    assert evidence["prep_params"]["switch_masks"]["right_param12"]["set"] is False
    assert evidence["telemetry"]["before"]["gap1_position"]["position"] == 100
    assert evidence["telemetry"]["during"] == result["wait"]["log_tail"]
    assert evidence["telemetry"]["after"]["gap1_position"]["position"] == 112
    assert tester.clear_bus_event_calls == 1
    assert tester.pop_bus_event_calls == 1
    assert tester.collect_bus_event_calls == 1
    assert evidence["events"]["capture_attempted"] is True
    assert evidence["events"]["captured_count"] == 2
    assert evidence["events"]["target_reached_128"][0]["source"] == "fake_move_ack_window"
    assert evidence["events"]["stallguard_130"][0]["source"] == "fake_tail_collect"
    assert evidence["classification"]["controller_motion_evidence"] is False
    assert evidence["classification"]["target_reached_event_seen"] is True
    assert evidence["classification"]["stall_event_seen"] is True
    assert evidence["classification"]["position_delta"] == 12
    assert evidence["classification"]["nonzero_speed_seen"] is True
    assert evidence["classification"]["physical_motion_confirmed"] is False


def test_interlock_override_reason_alias_requires_non_empty_reason_when_enabling(monkeypatch):
    api = load_api(monkeypatch)
    req = api.MotionInterlockOverrideRequest(
        enabled=True,
        override_latch_sensor=True,
        override_rail_24v=True,
        operator_ack="INTERLOCK_OVERRIDE",
        reason="commissioning latch sensor diagnosis",
    )
    assert api._motion_interlock_override_reason(req) == "api_interlock_override | commissioning latch sensor diagnosis"

    blank = api.MotionInterlockOverrideRequest(
        enabled=True,
        override_latch_sensor=True,
        override_rail_24v=True,
        operator_ack="INTERLOCK_OVERRIDE",
        reason="   ",
    )
    try:
        api._motion_interlock_override_reason(blank)
    except api.HTTPException as exc:
        assert exc.status_code == 409
        assert "non-empty reason" in str(exc.detail)
    else:
        raise AssertionError("enabled commissioning override must require a non-empty reason/operator_note")


def test_gripper_axis_status_classifies_hot_idle_current(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeTester()
    tester.current_values[(5, 6, 0)] = 31
    tester.current_values[(5, 7, 0)] = 31

    result = api._axis_status_payload(tester, api.AxisName.GRIPPER, include_current=True)

    assert result["current_safety"]["classification"] == "G_CURRENT_UNSAFE_HOT_IDLE"
    assert result["current_safety"]["speed"] == 0
    assert result["current_safety"]["run_current_param6"] == 31
    assert result["current_safety"]["standby_current_param7"] == 31
    assert result["current_safety"]["motion_commanded"] is False
