import importlib
import asyncio
import json
import sys
import types
import threading
import time
from pathlib import Path

import pytest
from fastapi import HTTPException


def _load_usb_driver(monkeypatch):
    usb_pkg = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_pkg.core = usb_core
    usb_pkg.util = usb_util
    setattr(usb_util, "release_interface", lambda dev, interface: None)
    setattr(usb_util, "dispose_resources", lambda dev: None)
    monkeypatch.setitem(sys.modules, "usb", usb_pkg)
    monkeypatch.setitem(sys.modules, "usb.core", usb_core)
    monkeypatch.setitem(sys.modules, "usb.util", usb_util)
    return importlib.import_module("src.bioxp.usb_driver")


def _make_tester(monkeypatch):
    usb_driver = _load_usb_driver(monkeypatch)
    tester = usb_driver.BioXpTester.__new__(usb_driver.BioXpTester)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1
    axis_limits = {"x": 91919, "y": 95247, "z": 160000, "g": 15000}
    offsets = {
        "m_TCDoorOpen": 16000,
        "m_TCDoorClose": 0,
        "m_TCDoorStallGuardThreshold": 6,
        "m_TC_DOOR_VELOCITY": 50,
        "m_TC_DOOR_ACCELERATION": 20,
        "m_TC_DOOR_MAX_CURRENT": 31,
    }
    monkeypatch.setattr(
        tester,
        "_machine_config_axis_max",
        lambda axis, fallback: (axis_limits[str(axis)], "immutable_oem_machine_snapshot"),
    )
    monkeypatch.setattr(
        tester,
        "_machine_config_offset_int",
        lambda key, fallback: (offsets[str(key)], "immutable_oem_machine_snapshot"),
    )
    monkeypatch.setattr(tester, "_motion_oem_gripper_version", lambda: 1)
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

    def fake_restore_gripper_idle_current(reason=None):
        write = fake_set_axis_param(tester.BOARD_HEAD, 6, 10, motor=2)
        return {"ok": True, "reason": reason, "writes": {"param6_run_idle": write}}

    monkeypatch.setattr(tester, "motor_prepare_axis", fake_prepare_axis)
    monkeypatch.setattr(tester, "motor_oem_go_home", fake_go_home)
    monkeypatch.setattr(tester, "motor_set_axis_param", fake_set_axis_param)
    monkeypatch.setattr(tester, "motor_restore_gripper_idle_current", fake_restore_gripper_idle_current)
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



def test_motor_oem_home_axis_z_startup_dispatches_literal_axis_search(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    observed = {}

    def fake_prepare_axis(board_id, motor=0, **kwargs):
        observed["prepare"] = {"board_id": board_id, "motor": motor, **kwargs}
        return {"ok": True}

    def axis_search(axis, **kwargs):
        observed["search"] = {"axis": axis, **kwargs}
        return {"ok": False, "false_home_guard": "test_fail_closed"}

    monkeypatch.setattr(tester, "motor_prepare_axis", fake_prepare_axis)
    monkeypatch.setattr(tester, "motor_oem_axis_search_home", axis_search)

    result = tester.motor_oem_home_axis("z", startup=True)

    assert result["axis"] == "z"
    assert observed["prepare"]["speed"] == 1791
    assert observed["prepare"]["acc"] == 576
    assert observed["search"] == {
        "axis": "z",
        "speed": 1791,
        "timeout_s": 20.0,
        "max_search_abs_delta": 160000,
    }
    assert result["home"]["ok"] is False
    assert result["home"]["false_home_guard"] == "test_fail_closed"



def test_startup_profiles_keep_literal_oem_xy_values_and_separate_z_recovery(monkeypatch):
    tester, _ = _make_tester(monkeypatch)

    x = tester._motion_oem_axis_profile("x", startup=True)
    y = tester._motion_oem_axis_profile("y", startup=True)
    z = tester._motion_oem_axis_profile("z", startup=True)

    assert (x["speed"], x["acc"], x["home_speed"], x["standby_current"]) == (1700, 350, 250, 20)
    assert (y["speed"], y["acc"], y["home_speed"], y["standby_current"]) == (1800, 400, 250, 20)
    assert (z["speed"], z["acc"], z["home_speed"], z["standby_current"]) == (1791, 576, 1791, 20)
    assert "disable_right" not in x
    assert "disable_left" not in x
    assert y["disable_right"] is True
    assert "disable_left" not in y
    assert x["oem_home_step"] == "MotorX.axisSearchHome(speed=250)"
    assert y["oem_home_step"] == "MotorY.axisSearchHome(speed=250)"
    assert z["oem_home_step"] == "MotorZ.axisSearchHome(speed=1791)"
    assert (z["axis_min_steps"], z["axis_max_steps"]) == (0, 160000)
    assert "positive_down_requires_right_mask" not in z
    assert tester.MOTOR_SWITCH_ACTIVE_VALUE == 1



def test_head_clearance_default_is_operator_requested_15k(monkeypatch):
    tester, _ = _make_tester(monkeypatch)

    assert tester.MOTOR_HEAD_CLEARANCE_LIFT_ABS == 15000



def test_motor_oem_go_home_z_uses_oem_move_left_not_linux_reversal(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1
    calls = []

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", lambda axis_key: {"board": 4, "motor": 1})
    position_values = iter([-5000, -5000, -4990, -4980, -4980, 0])
    monkeypatch.setattr(
        tester,
        "motor_get_position",
        lambda board_id, motor=0: {"position": next(position_values), "board": board_id, "motor": motor},
    )
    home_values = iter([0, 0, 0, 1, 1, 1])
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
    speed_values = iter([1791, 1791])
    monkeypatch.setattr(
        tester,
        "motor_get_speed",
        lambda board_id, motor=0: {"speed": next(speed_values), "board": board_id, "motor": motor},
    )
    monkeypatch.setattr(tester, "motor_get_switches", lambda board_id, motor=0: {"left_state": 1, "right_state": 1, "board": board_id, "motor": motor})
    monkeypatch.setattr(tester, "motor_move_right", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("OEM goHome must not use Linux-only Z reversal")))
    monkeypatch.setattr(tester, "motor_wait_stopped", lambda board_id, motor=0, timeout_s=30.0, **kwargs: {"stopped": True, "last_speed": 0})
    monkeypatch.setattr(tester, "motor_stop", lambda board_id, motor=0: calls.append(("stop", board_id, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_set_home", lambda board_id, motor=0: calls.append(("set_home", board_id, motor)) or {"ok": True, "readback": {"value": 0}})

    result = tester.motor_oem_go_home("z", speed=1791, rehome=False, timeout_s=1.0)

    assert result["ok"] is True
    assert result["move_direction"] == "move_left"
    assert calls[0] == ("move_left", 4, 1, 1791)
    assert ("set_home", 4, 1) in calls



def test_motor_oem_axis_search_home_preserves_source_initial_sethome(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1
    calls = []

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", lambda axis_key, startup=False: {"board": 5, "motor": 0})
    monkeypatch.setattr(tester, "motor_query_home_switch", lambda board_id, motor=0: {"value": 0, "board": board_id, "motor": motor})
    monkeypatch.setattr(
        tester,
        "motor_get_switch_activity",
        lambda board_id, motor=0: {"left_state": 0, "right_state": 1, "left_active": False, "right_active": True},
    )
    monkeypatch.setattr(tester, "motor_set_home", lambda board_id, motor=0: calls.append(("set_home", board_id, motor)) or {"ok": True})
    monkeypatch.setattr(
        tester,
        "motor_oem_go_home",
        lambda axis_key, **kwargs: calls.append(("go_home", axis_key, kwargs)) or {"ok": True, "home_after": {"value": 1}, "set_home": {"ok": True}, "switch_transition": True},
    )

    result = tester.motor_oem_axis_search_home("x", speed=250, timeout_s=1.0, max_search_abs_delta=1000)

    assert result["sethome_init"]["ok"] is True
    assert result["ok"] is True
    assert calls == [
        ("set_home", 5, 0),
        ("go_home", "x", {"speed": 250, "rehome": False, "timeout_s": 1.0, "require_switch_transition": True, "max_search_abs_delta": 1000}),
    ]




def test_startup_door_home_preserves_board_level_oem_active_home_preclear(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    calls = []

    monkeypatch.setattr(tester, "motor_prepare_axis", lambda board_id, motor=0, **kwargs: calls.append(("prepare", board_id, motor, kwargs)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_query_home_switch", lambda board_id, motor=0: {"ok": True, "value": 1})
    monkeypatch.setattr(tester, "motor_set_axis_param", lambda board_id, param, value, motor=0: calls.append(("sap", board_id, param, value, motor)) or {"ok": True, "value": value})
    monkeypatch.setattr(tester, "motor_move_relative", lambda board_id, steps, motor=0: calls.append(("relative", board_id, steps, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_move_left", lambda board_id, speed=600, motor=0: calls.append(("move_left", board_id, speed, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_wait_stopped", lambda board_id, motor=0, timeout_s=20.0, **kwargs: calls.append(("wait", board_id, motor, timeout_s, kwargs)) or {"stopped": True, "seen_nonzero": True})
    monkeypatch.setattr(tester, "motor_stop", lambda board_id, motor=0: calls.append(("stop", board_id, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_set_home", lambda board_id, motor=0: calls.append(("set_home", board_id, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_get_switch_activity", lambda board_id, motor=0: {"left_state": 1, "right_state": 0})
    monkeypatch.setattr(
        tester,
        "motor_thermal_door_status",
        lambda: {"closed": True, "opened": False, "home": {"value": 1}, "switches": {"left_state": 1, "right_state": 0}},
    )

    result = tester.motor_oem_home_axis("door", startup=True, timeout_s=90.0)
    home = result["home"]

    assert home["oem_mode"] == "initializeMotors.doorSearchHome"
    assert home["startup"] is True
    assert home["preclear_move"]["ok"] is True
    assert calls[:4] == [
        ("prepare", tester.BOARD_THERMAL, 0, {
            "run_current": 31,
            "standby_current": 10,
            "speed": 50,
            "acc": 20,
            "stall_guard": 6,
            "ramp_mode": None,
            "disable_right": True,
            "disable_left": True,
            "rdiv": None,
            "pdiv": None,
            "warm_enable": False,
        }),
        ("sap", tester.BOARD_THERMAL, 205, 8, 0),
        ("relative", tester.BOARD_THERMAL, 2000, 0),
        ("wait", tester.BOARD_THERMAL, 0, 8.0, {}),
    ]
    assert home["wait"]["stopped"] is True
    assert home["ok"] is True
    assert ("wait", tester.BOARD_THERMAL, 0, 20.0, {"require_seen_nonzero": False}) in calls



def test_startup_door_home_accepts_oem_closed_predicate_after_wait_timeout(monkeypatch):
    tester, _ = _make_tester(monkeypatch)

    monkeypatch.setattr(tester, "motor_query_home_switch", lambda board_id, motor=0: {"ok": True, "value": 1})
    monkeypatch.setattr(tester, "motor_set_axis_param", lambda board_id, param, value, motor=0: {"ok": True, "value": value})
    monkeypatch.setattr(tester, "motor_move_relative", lambda board_id, steps, motor=0: {"ok": True, "steps": steps})
    monkeypatch.setattr(tester, "motor_move_left", lambda board_id, speed=600, motor=0: {"ok": True})
    monkeypatch.setattr(tester, "motor_wait_stopped", lambda board_id, motor=0, timeout_s=20.0, **kwargs: {"stopped": False, "seen_nonzero": True, "elapsed_ms": 20000})
    monkeypatch.setattr(tester, "motor_stop", lambda board_id, motor=0: {"ok": True})
    monkeypatch.setattr(tester, "motor_set_home", lambda board_id, motor=0: {"ok": True, "board": board_id, "motor": motor})
    monkeypatch.setattr(tester, "motor_get_switch_activity", lambda board_id, motor=0: {"left_state": 1, "right_state": 0})
    monkeypatch.setattr(
        tester,
        "motor_thermal_door_status",
        lambda: {"closed": True, "opened": False, "home": {"value": 1}, "switches": {"left_state": 1, "right_state": 0}},
    )

    home = tester.motor_oem_door_search_home(timeout_s=90.0, startup=True)

    assert home["ok"] is True
    assert home["partial"] is False
    assert home["failure"] is None
    assert home["closed_confirmed"] is True
    assert home["set_home"]["ok"] is True
    assert home["wait_warning"] == "wait_not_stopped_but_closed_predicate_confirmed"


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
        "motor_oem_axis_already_home",
        lambda axis_key, tolerance_steps=0: sequence.append(("z_probe", axis_key, tolerance_steps)) or {"ok": True, "already_home": True},
    )
    monkeypatch.setattr(
        tester,
        "motor_oem_verify_z_clearance_for_xy",
        lambda **kwargs: sequence.append(("z_clear", kwargs)) or {"ok": True},
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
        ("z_clear", {"target": -15000, "min_clearance": -10000, "timeout_s": 20.0}),
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

        def motor_get_position(self, board, motor=0):
            return {"position": 1234, "board": board, "motor": motor}

        def motor_oem_switch_search_home_axis(self, axis_key, *, speed=None, timeout_s=None):
            observed["call"] = {"axis_key": axis_key, "speed": speed, "timeout_s": timeout_s}
            return {"ok": True, "axis_key": axis_key, "speed": speed}

    fake_tester = FakeTester()
    monkeypatch.setattr(
        api,
        "_prepare_motion_axis",
        lambda tester, axis, **kwargs: (
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
    assert result["motion_profile"]["vendor_path"] == "oem_axis_search_home_queryHome_setHome"



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
        lambda tester, axis, **kwargs: (
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

        def motor_get_position(self, board, motor=0):
            return {"position": 1234, "board": board, "motor": motor}

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
        lambda tester, axis, **kwargs: (
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
    tester._motion_latch_override = {"enabled": False, "override_latch_sensor": False, "override_rail_24v": False}

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



def test_motor_oem_home_xy_preserves_oem_parallel_semantics_with_serial_transactions(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    calls = []

    def fake_profile(axis_key, startup=False):
        assert startup is False
        if axis_key == "x":
            return {"board": 5, "motor": 0, "speed": 1700, "acc": 350}
        if axis_key == "y":
            return {"board": 4, "motor": 0, "speed": 1800, "acc": 400}
        raise AssertionError(axis_key)

    def fake_set_axis_param(board, param, value, motor=0):
        calls.append(("sap", board, motor, param, value))
        return {"ok": True, "value": value}

    def fake_go_home(axis_key, *, speed, rehome, timeout_s=30.0, require_switch_transition=True, max_search_abs_delta=None):
        assert speed == 200
        assert rehome is False
        assert timeout_s == 12.0
        assert require_switch_transition is True
        assert max_search_abs_delta is None
        calls.append(("home", axis_key))
        return {"axis": axis_key, "ok": True}

    def fake_status(board, motor=0):
        calls.append(("status", board, motor))
        return {"switches": {"left_raw_active": True}, "speed": {"speed": 0}}

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", fake_profile)
    monkeypatch.setattr(tester, "motor_set_axis_param", fake_set_axis_param)
    monkeypatch.setattr(tester, "motor_oem_go_home", fake_go_home)
    monkeypatch.setattr(tester, "motor_axis_status", fake_status)
    monkeypatch.setattr(tester, "motor_set_home", lambda board, motor=0: calls.append(("set_home", board, motor)) or {"ok": True})
    monkeypatch.setattr(tester, "motor_get_position", lambda board, motor=0: calls.append(("position", board, motor)) or {"position": 0})

    result = tester.motor_oem_home_xy(timeout_s=12.0)

    assert result["ok"] is True
    assert result["source_mode"] == "HomeXY"
    assert result["parallel_oem_semantics"] is True
    assert result["live_parallel_execution"] is True
    assert result["implementation_note"] == "oem_task_run_waitall_with_transaction_serialized_usb"
    assert {row for row in calls if row[0] == "home"} == {("home", "x"), ("home", "y")}
    assert calls[:4] == [
        ("sap", 5, 0, 4, 200),
        ("sap", 5, 0, 5, 200),
        ("sap", 4, 0, 4, 200),
        ("sap", 4, 0, 5, 200),
    ]
    assert result["home_rebase"]["x"]["home_rebased"] is True
    assert result["home_rebase"]["y"]["home_rebased"] is True


def test_oem_rehome_and_initialize_motion_wrappers_keep_modes_separate(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    sequence = []
    monkeypatch.setattr(tester, "motor_startup_homing_mimic", lambda: sequence.append("initializeMotors") or {"aborted_at": None})
    monkeypatch.setattr(tester, "motor_oem_initialize_without_motion", lambda: sequence.append("initializeMotorsWithoutMotion") or {"ok": True})
    monkeypatch.setattr(
        tester,
        "motor_plan_thermal_door_restore",
        lambda *, restore_requested=False: {
            "capture": {"implemented": False, "restore_requested": restore_requested},
            "restore_plan": {"implemented": False, "restore_requested": restore_requested},
        },
    )

    rehome = tester.motor_oem_rehome(timeout_s=99.0)
    init_no_motion = tester.motor_oem_initialize_motion(run_homing=False, timeout_s=55.0)
    init_with_homing = tester.motor_oem_initialize_motion(run_homing=True, timeout_s=77.0)

    assert rehome["source_mode"] == "ControlLib.rehome"
    assert rehome["ok"] is False
    assert rehome["blocked"] is True
    assert rehome["blocked_reason"] == "literal_direct_oem_stage_rewrite_pending"
    assert rehome["physical_motion_commanded"] is False
    assert "initialize_motors" not in rehome
    assert init_no_motion["source_mode"] == "ControlLib.initializeMotion"
    assert init_no_motion["physical_motion_commanded"] is False
    assert init_no_motion["rehome"] is None
    assert init_with_homing["ok"] is False
    assert init_with_homing["physical_motion_commanded"] is False
    assert init_with_homing["rehome"]["source_mode"] == "ControlLib.rehome"
    assert sequence == ["initializeMotorsWithoutMotion", "initializeMotorsWithoutMotion"]


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


def test_execute_oem_rehome_remains_publicly_blocked_without_motion(monkeypatch):
    from src.bioxp.oem_homing_model import source_matrix
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)

    class FakeTester:
        def motor_oem_rehome(self, *, timeout_s=120.0):
            return {
                "ok": False,
                "source_mode": "ControlLib.rehome",
                "blocked": True,
                "blocked_reason": "literal_direct_oem_stage_rewrite_pending",
                "physical_motion_commanded": False,
            }

    result = api._execute_oem_rehome(FakeTester(), timeout_s=44.0)

    assert result["ok"] is False
    assert result["physical_motion_commanded"] is False
    assert result["result"]["blocked"] is True
    assert result["route_semantics"]["home_semantics"] == "rehome_intentionally_blocked_no_monolithic_live_homing"
    rehome_mapping = next(
        mapping
        for mapping in source_matrix()["live_target_mappings"]
        if mapping["source_mode"] == "ControlLib.rehome"
    )
    assert rehome_mapping["target_status"] == "intentionally_blocked_no_monolithic_live_homing"


def test_initialize_motion_legacy_wrapper_stays_no_motion_but_openapi_exposes_typed_provider(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)

    class FakeTester:
        def motor_oem_initialize_motion(self, *, run_homing=False, timeout_s=120.0, include_tip_pipette_cleanup=False):
            assert run_homing is True
            return {
                "ok": False,
                "source_mode": "ControlLib.initializeMotion",
                "run_homing": True,
                "physical_motion_commanded": False,
                "rehome": {"blocked": True, "physical_motion_commanded": False},
            }

    result = api._execute_oem_initialize_motion(FakeTester(), run_homing=True, timeout_s=44.0)

    assert result["ok"] is False
    assert result["physical_motion_commanded"] is False
    assert result["route_semantics"]["home_semantics"] == "initializeMotion_homing_request_delegates_to_blocked_rehome_no_motion"
    schemas = api.app.openapi()["components"]["schemas"]
    assert "does not enable physical homing" in schemas["OemRehomeRequest"]["properties"]["run_homing"]["description"]
    assert "typed serial-206 provider" in schemas["OemInitializeMotionRequest"]["properties"]["run_homing"]["description"]
    assert "typed serial-206 provider" in schemas["OemInitializationRunRequest"]["properties"]["run_homing"]["description"]


def test_initialization_run_homing_fails_closed_when_typed_provider_is_unbound(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    monkeypatch.setattr(api, "_get_tester", lambda: object())
    monkeypatch.setattr(
        api,
        "run_oem_initialization_controller",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("blocked request must not invoke initialization controller")),
    )

    async def run_blocking(_label, callback, **_kwargs):
        return callback()

    monkeypatch.setattr(api, "_run_blocking", run_blocking)
    with pytest.raises(HTTPException) as raised:
        asyncio.run(
            api.motion_oem_initialization_run(
                api.OemInitializationRunRequest(
                    operator_ack="OEM_INITIALIZATION_RUN_WITH_HOMING",
                    run_homing=True,
                )
            )
        )

    assert raised.value.status_code == 503
    assert raised.value.detail["error"] == "serial206_oem_initialization_provider_unavailable"
    assert raised.value.detail["physical_motion_commanded"] is False

def test_initialization_controller_homing_request_is_directly_blocked_without_tester_call():
    from src.bioxp.oem_initialization import run_oem_initialization_controller

    class Tester:
        def __getattr__(self, name):
            raise AssertionError(f"blocked controller must not access tester.{name}")

    result = run_oem_initialization_controller(Tester(), run_homing=True)

    assert result["ok"] is False
    assert result["ready"] is False
    assert result["failed_at"] == "initialize_motors_full_sequence"
    assert result["phases"][-1]["blocked"] is True
    assert result["phases"][-1]["physical_motion_commanded"] is False


def test_checked_in_oem_homing_matrix_json_matches_source_model():
    from src.bioxp.oem_homing_model import source_matrix

    root = Path(__file__).resolve().parents[1]
    actual = json.loads((root / "docs/oem/oem_homing_source_matrix.json").read_text())
    expected = json.loads(json.dumps(source_matrix(), sort_keys=True))
    assert actual == expected


def test_source_model_covers_public_initialization_run_route():
    from src.bioxp.oem_homing_model import source_matrix

    raw_route_paths = {row["path"] for row in source_matrix()["raw_fastapi_route_table"]}
    mapped_route_paths = {
        row["route"]
        for row in source_matrix()["routes"]
        if row["surface"] == "raw-fastapi"
    }
    assert "/motion/oem/initialization/run" in raw_route_paths
    assert "/motion/oem/initialization/run" in mapped_route_paths
