import importlib
import asyncio
import inspect
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
        "m_Z_MOTOR_STALL_GUARD_THRESHOLD": 16,
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



def test_startup_profiles_keep_literal_oem_xy_values_and_separate_z_recovery(monkeypatch):
    tester, _ = _make_tester(monkeypatch)

    x = tester._motion_oem_axis_profile("x", startup=True)
    y = tester._motion_oem_axis_profile("y", startup=True)
    z = tester._motion_oem_axis_profile("z", startup=True)

    assert (x["speed"], x["acc"], x["home_speed"], x["standby_current"]) == (1700, 350, 250, 20)
    assert (y["speed"], y["acc"], y["home_speed"], y["standby_current"]) == (1800, 400, 250, 20)
    assert (z["speed"], z["acc"], z["home_speed"], z["standby_current"]) == (1791, 576, 1791, 10)
    assert z["standby_current_written"] is False
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
            "write_standby": True,
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

def test_duplicate_and_signed_z_authorities_are_absent(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    retired = (
        "motor_startup_homing_mimic",
        "motor_oem_initialize_without_motion",
        "motor_oem_move_z_to_reference",
        "motor_oem_verify_z_clearance_for_xy",
        "_motion_supervised_signed_z_profile",
        "_oem_no_motion_tmcl",
    )
    for name in retired:
        assert not hasattr(tester, name)


def test_source_homing_defaults_do_not_require_linux_switch_transition(monkeypatch):
    usb_driver = _load_usb_driver(monkeypatch)
    for name in ("motor_oem_axis_search_home", "motor_oem_go_home", "_motor_oem_go_home_adapted", "motor_oem_home_axis"):
        parameter = inspect.signature(getattr(usb_driver.BioXpTester, name)).parameters["require_switch_transition"]
        assert parameter.default is False


def test_legacy_non_oem_hardware_scripts_are_absent():
    root = Path(__file__).resolve().parents[1]
    for relative in (
        "scripts/bioxp_oem_reference_challenge.py",
        "scripts/bioxp_oem_z_home_probe.py",
    ):
        assert not (root / relative).exists()


def test_z_source_contract_keeps_three_distinct_home_intents(monkeypatch):
    tester, _ = _make_tester(monkeypatch)
    assert tester._motion_oem_axis_profile("z", startup=True)["home_speed"] == 1791
    assert tester._motion_oem_axis_profile("z", startup=False)["home_speed"] == 1791
    assert tester._motion_oem_axis_profile("z", startup=False)["board_test_home_speed"] == 597