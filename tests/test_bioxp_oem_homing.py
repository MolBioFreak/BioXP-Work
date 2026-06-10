import importlib
import sys
import types
import threading
import time

import pytest
from fastapi import HTTPException


def _load_usb_driver(monkeypatch):
    usb_pkg = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_pkg.core = usb_core
    usb_pkg.util = usb_util
    monkeypatch.setitem(sys.modules, "usb", usb_pkg)
    monkeypatch.setitem(sys.modules, "usb.core", usb_core)
    monkeypatch.setitem(sys.modules, "usb.util", usb_util)
    sys.modules.pop("src.bioxp.usb_driver", None)
    return importlib.import_module("src.bioxp.usb_driver")


def _make_tester(monkeypatch):
    usb_driver = _load_usb_driver(monkeypatch)
    tester = usb_driver.BioXpTester.__new__(usb_driver.BioXpTester)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1
    return tester, usb_driver


def test_motor_oem_home_axis_uses_vendor_button_speed_for_x(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    observed = {}

    def fake_prepare_axis(board_id, motor=0, **kwargs):
        observed["prepare"] = {"board_id": board_id, "motor": motor, **kwargs}
        return {"ok": True}

    def fake_go_home(axis_key, *, speed, rehome, timeout_s=None):
        observed["go_home"] = {"axis_key": axis_key, "speed": speed, "rehome": rehome, "timeout_s": timeout_s}
        return {"ok": True, "speed": speed, "rehome": rehome}

    monkeypatch.setattr(tester, "motor_prepare_axis", fake_prepare_axis)
    monkeypatch.setattr(tester, "motor_oem_go_home", fake_go_home)

    result = tester.motor_oem_home_axis("x")

    assert result["axis"] == "x"
    assert observed["prepare"]["run_current"] == 31
    assert observed["prepare"]["stall_guard"] == 16
    assert observed["go_home"] == {"axis_key": "x", "speed": 500, "rehome": True, "timeout_s": 20.0}



def test_motor_oem_home_axis_restores_gripper_current_for_version_one(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    observed = {}

    def fake_prepare_axis(board_id, motor=0, **kwargs):
        observed["prepare"] = {"board_id": board_id, "motor": motor, **kwargs}
        return {"ok": True}

    def fake_go_home(axis_key, *, speed, rehome, timeout_s=None):
        observed["go_home"] = {"axis_key": axis_key, "speed": speed, "rehome": rehome, "timeout_s": timeout_s}
        return {"ok": True, "speed": speed, "rehome": rehome}

    def fake_set_axis_param(board_id, param, value, motor=0):
        observed.setdefault("restores", []).append(
            {"board_id": board_id, "param": param, "value": value, "motor": motor}
        )
        return {"ack": {"status": 100}, "readback": {"value": value}}

    monkeypatch.setattr(tester, "motor_prepare_axis", fake_prepare_axis)
    monkeypatch.setattr(tester, "motor_oem_go_home", fake_go_home)
    monkeypatch.setattr(tester, "motor_set_axis_param", fake_set_axis_param)
    monkeypatch.setattr(tester, "_motion_oem_gripper_version", lambda: 1)

    result = tester.motor_oem_home_axis("g")

    assert result["axis"] == "g"
    assert observed["prepare"]["run_current"] == 31
    assert observed["go_home"] == {"axis_key": "g", "speed": 200, "rehome": True, "timeout_s": 20.0}
    assert observed["restores"] == [
        {
            "board_id": tester.BOARD_HEAD,
            "param": 6,
            "value": 10,
            "motor": 2,
        }
    ]



def test_motor_oem_home_axis_z_startup_uses_live_gap10_reference_at_zero(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    observed = {}

    def fake_prepare_axis(board_id, motor=0, **kwargs):
        observed["prepare"] = {"board_id": board_id, "motor": motor, **kwargs}
        return {"ok": True}

    def fail_axis_search(*args, **kwargs):
        raise AssertionError("Z startup should not run MoveLeft/GAP9 search when GAP10 reference is already active at controller zero")

    monkeypatch.setattr(tester, "motor_prepare_axis", fake_prepare_axis)
    monkeypatch.setattr(tester, "motor_get_position", lambda board_id, motor=0: {"ok": True, "position": 0})
    monkeypatch.setattr(
        tester,
        "motor_get_switch_activity",
        lambda board_id, motor=0: {"left_state": 0, "right_state": 1, "left_active": False, "right_active": True},
    )
    monkeypatch.setattr(tester, "motor_query_home_switch", lambda board_id, motor=0: {"ok": True, "value": 0})
    monkeypatch.setattr(tester, "motor_oem_axis_search_home", fail_axis_search)

    result = tester.motor_oem_home_axis("z", startup=True)

    home = result["home"]
    assert result["axis"] == "z"
    assert observed["prepare"]["speed"] == 250
    assert observed["prepare"]["acc"] == 60
    assert home["ok"] is True
    assert home["already_at_z_reference"] is True
    assert home["physical_motion_commanded"] is False
    assert home["set_home"] is None
    assert home["sethome_init"]["skipped"] is True
    assert home["live_z_reference_predicate"] == {
        "channel": "right/GAP10",
        "active_value": 1,
        "observed_value": 1,
        "controller_position": 0,
        "reason": "2026-05-07 live Z MoveLeft/GAP9 search ran negative without transition; GAP10/right is active at controller reference 0",
    }



def test_startup_profiles_use_soft_live_commissioning_values_with_oem_anchors(monkeypatch):
    tester, _ = _make_tester(monkeypatch)

    x = tester._motion_oem_axis_profile("x", startup=True)
    y = tester._motion_oem_axis_profile("y", startup=True)
    z = tester._motion_oem_axis_profile("z", startup=True)

    assert (x["speed"], x["acc"], x["home_speed"], x["standby_current"]) == (300, 120, 250, 20)
    assert (y["speed"], y["acc"], y["home_speed"], y["standby_current"]) == (300, 120, 250, 20)
    assert (z["speed"], z["acc"], z["home_speed"], z["standby_current"]) == (250, 60, 250, 20)
    assert z["oem_home_speed"] == 1791
    assert z["positive_down_requires_right_mask"] is True
    assert tester.MOTOR_SWITCH_ACTIVE_VALUE == 1



def test_head_clearance_default_is_operator_requested_15k(monkeypatch):
    tester, _ = _make_tester(monkeypatch)

    assert tester.MOTOR_HEAD_CLEARANCE_LIFT_ABS == 15000



def test_motor_oem_go_home_z_uses_oem_move_left_not_linux_reversal(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1
    calls = []

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", lambda axis_key: {"board": 4, "motor": 1})
    monkeypatch.setattr(tester, "motor_get_position", lambda board_id, motor=0: {"position": -5000, "board": board_id, "motor": motor})
    home_values = iter([0, 0, 1, 1])
    monkeypatch.setattr(
        tester,
        "motor_query_home_switch",
        lambda board_id, motor=0: {"value": next(home_values), "board": board_id, "motor": motor},
    )
    monkeypatch.setattr(
        tester,
        "motor_get_switch_activity",
        lambda board_id, motor=0: {"left_state": 0, "right_state": 1, "left_active": False, "right_active": True},
    )
    monkeypatch.setattr(tester, "motor_move_left", lambda board_id, speed=250, motor=0: calls.append(("move_left", board_id, motor, speed)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_get_speed", lambda board_id, motor=0: {"speed": 1791, "board": board_id, "motor": motor})
    monkeypatch.setattr(tester, "motor_get_switches", lambda board_id, motor=0: {"left_state": 1, "right_state": 1, "board": board_id, "motor": motor})
    monkeypatch.setattr(tester, "motor_move_right", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("OEM goHome must not use Linux-only Z reversal")))
    monkeypatch.setattr(tester, "motor_wait_stopped", lambda board_id, motor=0, timeout_s=30.0, **kwargs: {"stopped": True})
    monkeypatch.setattr(tester, "motor_stop", lambda board_id, motor=0: calls.append(("stop", board_id, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_set_home", lambda board_id, motor=0: calls.append(("set_home", board_id, motor)) or {"ok": True})

    result = tester.motor_oem_go_home("z", speed=1791, rehome=False, timeout_s=1.0)

    assert result["ok"] is True
    assert result["move_direction"] == "move_left"
    assert calls[0] == ("move_left", 4, 1, 1791)
    assert ("set_home", 4, 1) in calls



def test_motor_oem_axis_search_home_skips_initial_fake_sethome(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1
    calls = []

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", lambda axis_key, startup=False: {"board": 4, "motor": 1})
    monkeypatch.setattr(tester, "motor_query_home_switch", lambda board_id, motor=0: {"value": 0, "board": board_id, "motor": motor})
    monkeypatch.setattr(
        tester,
        "motor_get_switch_activity",
        lambda board_id, motor=0: {"left_state": 0, "right_state": 1, "left_active": False, "right_active": True},
    )
    monkeypatch.setattr(tester, "motor_set_home", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("initial setHome must be skipped until live switch transition")))
    monkeypatch.setattr(
        tester,
        "motor_oem_go_home",
        lambda axis_key, **kwargs: calls.append((axis_key, kwargs)) or {"ok": False, "false_home_guard": "unit_test"},
    )

    result = tester.motor_oem_axis_search_home("z", speed=250, timeout_s=1.0, max_search_abs_delta=1000)

    assert result["sethome_init"]["skipped"] is True
    assert result["false_home_guard"] == "unit_test"
    assert calls == [("z", {"speed": 250, "rehome": False, "timeout_s": 1.0, "require_switch_transition": True, "max_search_abs_delta": 1000})]



def test_motor_startup_homing_mimic_uses_oem_initialize_sequence(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    sequence = []

    monkeypatch.setattr(tester, "reconnect", lambda: sequence.append("reconnect"))
    monkeypatch.setattr(tester, "activate_boards", lambda expect_reply=True: {"ok": True, "expect_reply": expect_reply})
    monkeypatch.setattr(
        tester,
        "motor_oem_initialize_without_motion",
        lambda: sequence.append("init_without_motion") or {"ok": True},
    )
    monkeypatch.setattr(
        tester,
        "motor_move_relative",
        lambda board_id, steps, motor=0: sequence.append(("g_pre_move", board_id, motor, steps)) or {"ok": True},
    )
    monkeypatch.setattr(
        tester,
        "motor_wait_stopped",
        lambda board_id, motor=0, timeout_s=4.0, poll_s=0.06, require_seen_nonzero=False: sequence.append(("wait", board_id, motor, timeout_s, require_seen_nonzero)) or {"stopped": True},
    )
    monkeypatch.setattr(
        tester,
        "motor_oem_home_axis",
        lambda axis_key, **kwargs: sequence.append(("home", axis_key, kwargs)) or {"axis": axis_key, "home": {"ok": True}},
    )
    monkeypatch.setattr(tester, "motor_set_home", lambda board_id, motor=0: sequence.append(("set_home", board_id, motor)) or {"ok": True})
    monkeypatch.setattr(
        tester,
        "motor_set_axis_param",
        lambda board_id, param, value, motor=0: sequence.append(("sap", board_id, param, value, motor)) or {"ack": {"status": 100}, "readback": {"value": value}},
    )
    monkeypatch.setattr(
        tester,
        "motor_move_absolute",
        lambda board_id, position, motor=0: sequence.append(("move_abs", board_id, position, motor)) or {"ok": True},
    )
    monkeypatch.setattr(tester, "motor_query_24v_sensor", lambda: {"ok": True})

    tester.motor_startup_homing_mimic()

    expected = [
        "reconnect",
        "init_without_motion",
        ("home", "z", {"startup": True}),
        ("g_pre_move", tester.BOARD_HEAD, 2, 10000),
        ("wait", tester.BOARD_HEAD, 2, 10.0, True),
        ("home", "g", {"startup": True}),
        ("home", "x", {"startup": True}),
        ("set_home", tester.BOARD_DECK, 0),
        ("sap", tester.BOARD_DECK, 4, 1700, 0),
        ("move_abs", tester.BOARD_DECK, 6000, 0),
        ("wait", tester.BOARD_DECK, 0, 8.0, True),
        ("home", "y", {"startup": True}),
        ("home", "door", {"startup": True}),
        ("set_home", tester.BOARD_HEAD, 0),
    ]
    assert sequence == expected



def test_execute_home_axis_blocks_unverified_home_predicate_by_default(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)

    class FakeTester:
        def _motion_oem_axis_profile(self, axis_key, startup=False):
            assert axis_key == "x"
            return {"home_speed": 500, "speed": 500}

        def motor_oem_switch_search_home_axis(self, axis_key, *, speed=None, timeout_s=None):
            raise AssertionError("unverified manual homing must fail before dispatch")

    with pytest.raises(HTTPException) as exc_info:
        api._execute_home_axis(FakeTester(), api.AxisName.X, speed=444, timeout_s=9.0)

    assert exc_info.value.status_code == 409
    assert exc_info.value.detail["axis"] == "x"
    assert exc_info.value.detail["blocked_route"] == "/motion/axis/home"
    assert "live/source verified" in exc_info.value.detail["message"]



def test_execute_home_axis_z_override_still_blocks_implementation_mapped_predicate(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)

    class FakeTester:
        def _motion_oem_axis_profile(self, axis_key, startup=False):
            assert axis_key == "z"
            return {"board": 4, "motor": 1, "home_speed": 250, "speed": 250}

        def motor_oem_switch_search_home_axis(self, axis_key, *, speed=None, timeout_s=None):
            raise AssertionError("Z implementation-mapped manual home override must fail before dispatch")

    monkeypatch.setattr(
        api,
        "_home_predicate_snapshot",
        lambda tester, axis: {
            "axis": axis.value,
            "interpreted": {
                "axis": axis.value,
                "confidence": "implementation_mapped",
                "home_switch": "gap9_left",
                "active_value": 1,
                "gap9_left_value": 0,
                "gap10_right_value": 1,
            },
        },
    )

    with pytest.raises(HTTPException) as exc_info:
        api._execute_home_axis(
            FakeTester(),
            api.AxisName.Z,
            speed=250,
            timeout_s=9.0,
            allow_implementation_mapped_predicate=True,
        )

    assert exc_info.value.status_code == 409
    detail = exc_info.value.detail
    assert detail["axis"] == "z"
    assert detail["blocked_route"] == "/motion/axis/home"
    assert "implementation-mapped override is deliberately disabled for Z" in detail["message"]
    assert detail["incident_guard"] == "manual_z_home_gap9_search_can_ignore_physical_gap10_bottom_reference"


def test_execute_home_axis_override_routes_through_transition_home(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)
    observed = {}

    class FakeTester:
        def _motion_oem_axis_profile(self, axis_key, startup=False):
            assert axis_key == "x"
            return {"home_speed": 500, "speed": 500}

        def motor_oem_switch_search_home_axis(self, axis_key, *, speed=None, timeout_s=None):
            observed["call"] = {"axis_key": axis_key, "speed": speed, "timeout_s": timeout_s}
            return {"ok": True, "axis_key": axis_key, "speed": speed}

    fake_tester = FakeTester()
    monkeypatch.setattr(
        api,
        "_prepare_motion_axis",
        lambda tester, axis: (
            {"board": 5, "motor": 0, "speed": 1700, "acc": 350},
            {"ok": True},
            {"ok": True},
            {"ok": True},
            {"reuse_requested": False},
        ),
    )

    result = api._execute_home_axis(
        fake_tester,
        api.AxisName.X,
        speed=444,
        timeout_s=9.0,
        allow_implementation_mapped_predicate=True,
    )

    assert observed["call"] == {"axis_key": "x", "speed": 444, "timeout_s": 9.0}
    assert result["home"]["ok"] is True
    assert result["home"]["axis_key"] == "x"
    assert result["motion_profile"]["vendor_path"] == "oem_axis_search_home_transition_guarded"



def test_execute_home_axis_rejects_speed_above_oem_profile(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)

    class FakeTester:
        def _motion_oem_axis_profile(self, axis_key, startup=False):
            assert axis_key == "x"
            return {"home_speed": 500, "speed": 500}

        def motor_oem_home_axis(self, axis_key, *, speed=None, timeout_s=None):
            raise AssertionError("oversized home-speed requests must be rejected before dispatch")

    monkeypatch.setattr(
        api,
        "_prepare_motion_axis",
        lambda tester, axis: (
            {"board": 5, "motor": 0, "speed": 1700, "acc": 350},
            {"ok": True},
            {"ok": True},
            {"ok": True},
            {"reuse_requested": False},
        ),
    )

    with pytest.raises(HTTPException) as exc_info:
        api._execute_home_axis(FakeTester(), api.AxisName.X, speed=501, timeout_s=9.0)

    assert exc_info.value.status_code == 422
    assert "between 1 and 500" in str(exc_info.value.detail)



def test_execute_home_axis_returns_structured_failure_when_transition_home_does_not_confirm_switch(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)

    class FakeTester:
        def _motion_oem_axis_profile(self, axis_key, startup=False):
            return {"home_speed": 500, "speed": 500}

        def motor_oem_switch_search_home_axis(self, axis_key, *, speed=None, timeout_s=None):
            return {
                "axis": axis_key,
                "home": {
                    "ok": False,
                    "home_after": {"value": 1},
                    "wait": {"stopped": True},
                    "set_home": {"ack": {"status": 100}},
                },
            }

    monkeypatch.setattr(
        api,
        "_prepare_motion_axis",
        lambda tester, axis: (
            {"board": 5, "motor": 0, "speed": 1700, "acc": 350},
            {"ok": True},
            {"ok": True},
            {"ok": True},
            {"reuse_requested": False},
        ),
    )

    result = api._execute_home_axis(
        FakeTester(),
        api.AxisName.X,
        speed=444,
        timeout_s=9.0,
        allow_implementation_mapped_predicate=True,
    )

    assert result["ok"] is False
    assert result["motion_failure"]["category"] == "home_not_confirmed"
    assert result["motion_failure"]["http_status_previously"] == 409
    assert "did not confirm the home switch" in result["motion_failure"]["message"]



def test_motion_arm_strict_startup_accepts_nested_oem_home_payload(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    tester.BOARD_HEAD = 0
    tester.BOARD_DECK = 1
    tester.BOARD_THERMAL = 2
    tester.MOTION_ERROR_CODES = {"STRICT_HOMING_FAILED": 91, "STRICT_INIT_FAILED": 92}
    tester._motion_arm = {}

    def ack():
        return {"status": 100}

    def nested_home(axis_key):
        return {
            "axis": axis_key,
            "startup": True,
            "prepare": {"ok": True},
            "home": {
                "move_left": {"ack": ack()},
                "wait": {"stopped": True},
                "stop": {"ack": ack()},
                "set_home": {"ack": ack()},
                "home_after": {"value": 1},
            },
        }

    monkeypatch.setattr(tester, "reconnect", lambda: None)
    monkeypatch.setattr(tester, "activate_boards", lambda expect_reply=True: {0: ack(), 1: ack(), 2: ack()})
    monkeypatch.setattr(tester, "motion_gate_live_snapshot", lambda: {"ok": True, "error_keys": []})
    monkeypatch.setattr(tester, "latch_oem", lambda enabled: {"ack": ack()})
    monkeypatch.setattr(
        tester,
        "motor_prepare_motion_interlock",
        lambda force_lock=True: {"rail_24v": {"no24v": False, "raw": 0}, "latch": {"ack": ack()}},
    )
    monkeypatch.setattr(
        tester,
        "motor_startup_homing_mimic",
        lambda: {
            "z_home": nested_home("z"),
            "g_home": nested_home("g"),
            "x_home": nested_home("x"),
            "y_home": nested_home("y"),
            "door_home": nested_home("door"),
            "x_move_6000": {"ack": ack()},
            "x_wait": {"stopped": True},
        },
    )

    result = tester.motion_arm_strict_startup(run_homing=True)

    assert result["ok"] is True
    assert result["arm_state"]["armed"] is True
    check_map = {row["name"]: row for row in result["checks"]}
    assert check_map["home_X"]["ok"] is True
    assert check_map["home_Y"]["ok"] is True
    assert check_map["home_Z"]["ok"] is True
    assert check_map["home_G"]["ok"] is True
    assert check_map["x_after_home_move6000"]["ok"] is True



def test_motor_oem_home_xy_matches_oem_parallel_task_run_waitall(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    calls = []
    lock = threading.Lock()
    both_started = threading.Event()
    started = []
    ended = []

    def fake_profile(axis_key, startup=False):
        assert startup is False
        if axis_key == "x":
            return {"board": 5, "motor": 0, "speed": 1700, "acc": 350}
        if axis_key == "y":
            return {"board": 4, "motor": 0, "speed": 1800, "acc": 400}
        raise AssertionError(axis_key)

    def fake_set_axis_param(board, param, value, motor=0):
        with lock:
            calls.append(("sap", board, motor, param, value))
        return {"ok": True, "value": value}

    def fake_go_home(axis_key, *, speed, rehome, timeout_s=30.0, require_switch_transition=True, max_search_abs_delta=None):
        assert speed == 200
        assert rehome is False
        assert timeout_s == 12.0
        assert require_switch_transition is False
        assert max_search_abs_delta is None
        with lock:
            started.append(axis_key)
            calls.append(("home_start", axis_key, time.monotonic()))
            if set(started) == {"x", "y"}:
                both_started.set()
        assert both_started.wait(1.0), "HomeXY must start both X and Y homes before either returns"
        time.sleep(0.02)
        with lock:
            ended.append(axis_key)
            calls.append(("home_end", axis_key, time.monotonic()))
        return {"axis": axis_key, "ok": True}

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", fake_profile)
    monkeypatch.setattr(tester, "motor_set_axis_param", fake_set_axis_param)
    monkeypatch.setattr(tester, "motor_oem_go_home", fake_go_home)

    result = tester.motor_oem_home_xy(timeout_s=12.0)

    assert result["ok"] is True
    assert result["source_mode"] == "HomeXY"
    assert result["parallel_oem_semantics"] is True
    assert result["live_parallel_execution"] is True
    assert result["implementation_note"] == "oem_task_run_waitall_parallel_goHome_false_speed200"
    assert set(started) == {"x", "y"}
    assert set(ended) == {"x", "y"}
    first_end_index = min(i for i, row in enumerate(calls) if row[0] == "home_end")
    starts_before_first_end = [row[1] for row in calls[:first_end_index] if row[0] == "home_start"]
    assert set(starts_before_first_end) == {"x", "y"}
    assert calls[:4] == [
        ("sap", 5, 0, 4, 200),
        ("sap", 5, 0, 5, 200),
        ("sap", 4, 0, 4, 200),
        ("sap", 4, 0, 5, 200),
    ]
    assert calls[-4:] == [
        ("sap", 5, 0, 4, 1700),
        ("sap", 5, 0, 5, 350),
        ("sap", 4, 0, 4, 1800),
        ("sap", 4, 0, 5, 400),
    ]


def test_oem_rehome_and_initialize_motion_wrappers_keep_modes_separate(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    sequence = []
    monkeypatch.setattr(tester, "motor_startup_homing_mimic", lambda: sequence.append("initializeMotors") or {"aborted_at": None})
    monkeypatch.setattr(tester, "motor_oem_initialize_without_motion", lambda: sequence.append("initializeMotorsWithoutMotion") or {"ok": True})

    rehome = tester.motor_oem_rehome(timeout_s=99.0)
    init_no_motion = tester.motor_oem_initialize_motion(run_homing=False, timeout_s=55.0)
    init_with_homing = tester.motor_oem_initialize_motion(run_homing=True, timeout_s=77.0)

    assert rehome["source_mode"] == "ControlLib.rehome"
    assert rehome["initialize_motors"]["aborted_at"] is None
    assert rehome["door_state_save"]["implemented"] is False
    assert init_no_motion["source_mode"] == "ControlLib.initializeMotion"
    assert init_no_motion["physical_motion_commanded"] is False
    assert init_no_motion["rehome"] is None
    assert init_with_homing["physical_motion_commanded"] is True
    assert init_with_homing["rehome"]["source_mode"] == "ControlLib.rehome"
    assert sequence == ["initializeMotors", "initializeMotorsWithoutMotion", "initializeMotorsWithoutMotion", "initializeMotors"]


def test_execute_oem_home_xy_uses_predicate_guard_and_direct_mode(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)
    observed = {}

    class FakeTester:
        def motor_oem_home_xy(self, *, timeout_s=30.0):
            observed["timeout_s"] = timeout_s
            return {"ok": True, "source_mode": "HomeXY"}

    monkeypatch.setattr(
        api,
        "_home_predicate_snapshot",
        lambda tester, axis: {"axis": axis.value, "interpreted": {"confidence": "implementation_mapped"}},
    )

    result = api._execute_oem_home_xy(FakeTester(), timeout_s=22.0, allow_implementation_mapped_predicate=True)

    assert result["ok"] is True
    assert result["source_mode"] == "HomeXY"
    assert observed["timeout_s"] == 22.0
    assert result["route_semantics"]["raw_fastapi_route"] == "/motion/oem/home_xy"
    assert result["route_semantics"]["not_equivalent_to"] == ["/motion/axis/zero", "/motion/axis/home single-axis manual route"]


def test_execute_oem_initialize_motion_dispatches_no_homing_diagnostic(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)
    observed = {}

    class FakeTester:
        def motor_oem_initialize_motion(self, *, run_homing=False, timeout_s=120.0, include_tip_pipette_cleanup=False):
            observed.update({"run_homing": run_homing, "timeout_s": timeout_s, "cleanup": include_tip_pipette_cleanup})
            return {"ok": True, "source_mode": "ControlLib.initializeMotion", "physical_motion_commanded": bool(run_homing)}

    result = api._execute_oem_initialize_motion(
        FakeTester(),
        run_homing=False,
        timeout_s=44.0,
        include_tip_pipette_cleanup=True,
    )

    assert result["ok"] is True
    assert result["source_mode"] == "ControlLib.initializeMotion"
    assert observed == {"run_homing": False, "timeout_s": 44.0, "cleanup": True}
    assert result["route_semantics"]["raw_fastapi_route"] == "/motion/oem/initialize_motion"
