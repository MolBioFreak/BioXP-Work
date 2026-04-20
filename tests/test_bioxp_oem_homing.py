import importlib
import sys
import types

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
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 0
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
        lambda board_id, motor=0, timeout_s=4.0, poll_s=0.06: sequence.append(("wait", board_id, motor, timeout_s)) or {"stopped": True},
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

    assert sequence[:2] == ["reconnect", "init_without_motion"]
    assert ("g_pre_move", tester.BOARD_HEAD, 2, 10000) in sequence
    assert ("home", "z", {"startup": True}) in sequence
    assert ("home", "g", {"startup": True}) in sequence
    assert ("home", "x", {"startup": True}) in sequence
    assert ("move_abs", tester.BOARD_DECK, 6000, 0) in sequence
    assert ("home", "y", {"startup": True}) in sequence
    assert ("home", "door", {"startup": True}) in sequence



def test_execute_home_axis_routes_through_tester_oem_home(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)
    observed = {}

    class FakeTester:
        def _motion_oem_axis_profile(self, axis_key, startup=False):
            assert axis_key == "x"
            return {"home_speed": 500, "speed": 500}

        def motor_oem_home_axis(self, axis_key, *, speed=None, timeout_s=None):
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

    result = api._execute_home_axis(fake_tester, api.AxisName.X, speed=444, timeout_s=9.0)

    assert observed["call"] == {"axis_key": "x", "speed": 444, "timeout_s": 9.0}
    assert result["home"]["ok"] is True
    assert result["home"]["axis_key"] == "x"



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



def test_execute_home_axis_raises_when_oem_home_does_not_confirm_switch(monkeypatch):
    from tests.test_motion_service import load_api

    api = load_api(monkeypatch)

    class FakeTester:
        def _motion_oem_axis_profile(self, axis_key, startup=False):
            return {"home_speed": 500, "speed": 500}

        def motor_oem_home_axis(self, axis_key, *, speed=None, timeout_s=None):
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

    with pytest.raises(HTTPException) as exc_info:
        api._execute_home_axis(FakeTester(), api.AxisName.X, speed=444, timeout_s=9.0)

    assert exc_info.value.status_code == 409
    assert "did not confirm the home switch" in str(exc_info.value.detail)



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
                "home_after": {"value": 0},
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
