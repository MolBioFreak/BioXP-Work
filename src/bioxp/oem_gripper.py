"""OEM gripper/G-axis status and action contract.

This module keeps BioXP gripper logic out of generic axis controls.  It is
source-shaped from ClassControlInterface gripper paths: scoped action current,
version/profile evidence, explicit clear/home operations, and idle-current
restore in every exit path.
"""

from __future__ import annotations

from typing import Any

from fastapi import HTTPException

OEM_IDLE_CURRENT = 10
GRIPPER_ACTION_CURRENT = 31
GRIPPER_CLEAR_STEPS = 10000
GRIPPER_CLEAR_ACK = "GRIPPER_CLEAR"
GRIPPER_HOME_ACK = "GRIPPER_HOME"


def _value(payload: Any, *keys: str) -> Any:
    cur = payload
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _int_or_none(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def _profile(tester: Any) -> dict[str, Any]:
    raw = tester._motion_oem_axis_profile("g", startup=True)
    if not isinstance(raw, dict):
        raw = tester.motor_function_preset("g")
    if not isinstance(raw, dict):
        raise HTTPException(status_code=503, detail="OEM gripper profile unavailable")
    out = dict(raw)
    out.setdefault("board", 4)
    out.setdefault("motor", 2)
    out.setdefault("run_current", GRIPPER_ACTION_CURRENT)
    out.setdefault("standby_current", OEM_IDLE_CURRENT)
    out.setdefault("restore_current", OEM_IDLE_CURRENT)
    out.setdefault("speed", 600)
    out.setdefault("acc", 5)
    out.setdefault("stall_guard", 5)
    out.setdefault("rdiv", 6)
    out.setdefault("pdiv", 2)
    out["provenance"] = {
        "source": "ClassControlInterface initializeMotorsWithoutMotion/initializeMotors gripper profile",
        "gripper_version": out.get("gripper_version", out.get("version", "runtime_profile_or_fallback")),
        "fallback_fields_possible": True,
    }
    return out


def _switch_activity(tester: Any, board: int, motor: int) -> dict[str, Any]:
    switches = tester.motor_get_switches(board, motor=motor)
    active_val = int(getattr(tester, "MOTOR_SWITCH_ACTIVE_VALUE", 1))
    left = switches.get("left_state") if isinstance(switches, dict) else None
    right = switches.get("right_state") if isinstance(switches, dict) else None
    left_disabled = bool(switches.get("left_disabled", False)) if isinstance(switches, dict) else False
    right_disabled = bool(switches.get("right_disabled", False)) if isinstance(switches, dict) else False
    left_raw_active = switches.get("left_raw_active") if isinstance(switches, dict) else None
    right_raw_active = switches.get("right_raw_active") if isinstance(switches, dict) else None
    if left_raw_active is None and left is not None:
        left_raw_active = int(left) == active_val
    if right_raw_active is None and right is not None:
        right_raw_active = int(right) == active_val
    left_active = None if left_raw_active is None else bool(left_raw_active) and not left_disabled
    right_active = None if right_raw_active is None else bool(right_raw_active) and not right_disabled
    return {
        "board": board,
        "motor": motor,
        "active_raw_value": active_val,
        "left_state": left,
        "right_state": right,
        "left_disabled": left_disabled,
        "right_disabled": right_disabled,
        "left_raw_active": left_raw_active,
        "right_raw_active": right_raw_active,
        "left_active": left_active,
        "right_active": right_active,
        "both_effective_limits_active": bool(left_active is True and right_active is True),
        "raw": switches,
    }


def gripper_status(tester: Any) -> dict[str, Any]:
    profile = _profile(tester)
    board = int(profile["board"])
    motor = int(profile["motor"])
    switches = _switch_activity(tester, board, motor)
    position = tester.motor_get_position(board, motor=motor)
    speed = tester.motor_get_speed(board, motor=motor)
    run = tester.motor_get_axis_param(board, 6, motor=motor)
    standby = tester.motor_get_axis_param(board, 7, motor=motor)
    query_home = None
    if hasattr(tester, "motor_query_home_switch"):
        query_home = tester.motor_query_home_switch(board, motor=motor)
    pos_value = _int_or_none(_value(position, "position"))
    query_value = _int_or_none(_value(query_home, "value"))
    query_home_active = bool(query_value == int(getattr(tester, "MOTOR_SWITCH_ACTIVE_VALUE", 1))) if query_value is not None else None
    position_lt_50 = bool(pos_value is not None and pos_value < 50)
    run_value = _int_or_none(_value(run, "value"))
    standby_value = _int_or_none(_value(standby, "value"))
    speed_value = _int_or_none(_value(speed, "speed"))
    idle_safe = bool((run_value is None or run_value <= OEM_IDLE_CURRENT) and (standby_value is None or standby_value <= OEM_IDLE_CURRENT))
    blockers: list[str] = []
    if switches["both_effective_limits_active"]:
        blockers.append("both_effective_limits_active")
    if speed_value == 0 and not idle_safe:
        blockers.append("g_current_hot_while_idle")
    return {
        "ok": True,
        "schema": "bioxp.oem_gripper_status.v1",
        "opened_usb": True,
        "physical_motion": False,
        "motion_commanded": False,
        "axis": "g",
        "board": board,
        "motor": motor,
        "position": position,
        "speed": speed,
        "switches": switches,
        "current": {
            "run_current_param6": run_value,
            "standby_current_param7": standby_value,
            "safe_idle_max": OEM_IDLE_CURRENT,
            "idle_safe": idle_safe,
            "raw": {"run": run, "standby": standby},
        },
        "profile": profile,
        "oem_home_predicate": {
            "query_home": query_home,
            "query_home_active": query_home_active,
            "position_lt_50": position_lt_50,
            "oem_confirmed_home": bool(query_home_active is True or position_lt_50),
            "source": "ClassControlInterface.confirmAxis(g): queryHome OR getG()<50",
        },
        "blockers": blockers,
    }


def _require_action(operator_ack: str | None, expected: str, reason: str | None) -> None:
    if operator_ack != expected:
        raise HTTPException(status_code=409, detail={"error": "operator_ack_required", "expected_operator_ack": expected, "motion_commanded": False})
    if not str(reason or "").strip():
        raise HTTPException(status_code=409, detail={"error": "reason_required", "motion_commanded": False})


def _restore_idle(tester: Any, reason: str) -> dict[str, Any]:
    if hasattr(tester, "motor_restore_gripper_idle_current"):
        return tester.motor_restore_gripper_idle_current(reason=reason)
    profile = _profile(tester)
    board = int(profile["board"])
    motor = int(profile["motor"])
    standby = tester.motor_set_axis_param(board, 7, OEM_IDLE_CURRENT, motor=motor)
    run = tester.motor_set_axis_param(board, 6, OEM_IDLE_CURRENT, motor=motor)
    return {"ok": True, "reason": reason, "standby": standby, "run": run}


def restore_gripper_idle_current(tester: Any, *, reason: str = "operator_restore_idle_current") -> dict[str, Any]:
    before = gripper_status(tester)
    restore = _restore_idle(tester, reason)
    after = gripper_status(tester)
    return {"ok": True, "motion_commanded": False, "physical_motion": False, "before": before, "restore": restore, "after": after}


def _apply_profile(tester: Any, profile: dict[str, Any]) -> dict[str, Any]:
    return tester.motor_prepare_axis(
        int(profile["board"]),
        motor=int(profile["motor"]),
        run_current=int(profile.get("run_current", GRIPPER_ACTION_CURRENT)),
        standby_current=int(profile.get("standby_current", OEM_IDLE_CURRENT)),
        speed=int(profile.get("speed", 600)),
        acc=int(profile.get("acc", 5)),
        stall_guard=profile.get("stall_guard"),
        rdiv=profile.get("rdiv", 6),
        pdiv=profile.get("pdiv", 2),
        disable_right=bool(profile.get("disable_right", False)),
        disable_left=bool(profile.get("disable_left", False)),
        warm_enable=bool(profile.get("warm_enable", False)),
    )


def _preflight_for_motion(tester: Any) -> dict[str, Any]:
    status = gripper_status(tester)
    if "both_effective_limits_active" in status.get("blockers", []):
        raise HTTPException(status_code=409, detail={"error": "gripper_switch_conflict", "blocker": "both_effective_limits_active", "motion_commanded": False, "status": status})
    return status


def gripper_clear(tester: Any, *, operator_ack: str | None, reason: str | None, timeout_s: float = 12.0) -> dict[str, Any]:
    _require_action(operator_ack, GRIPPER_CLEAR_ACK, reason)
    before = _preflight_for_motion(tester)
    profile = _profile(tester)
    board = int(profile["board"])
    motor = int(profile["motor"])
    prepare = None
    set_action_current = None
    move = None
    wait = None
    restore = None
    try:
        prepare = _apply_profile(tester, profile)
        set_action_current = tester.motor_set_axis_param(board, 6, GRIPPER_ACTION_CURRENT, motor=motor)
        move = tester.motor_move_relative(board, GRIPPER_CLEAR_STEPS, motor=motor)
        wait = tester.motor_wait_stopped(board, motor=motor, timeout_s=min(float(timeout_s), 12.0), require_seen_nonzero=True)
        ok = bool(isinstance(move, dict) and move.get("ok") and isinstance(wait, dict) and wait.get("stopped") is True and wait.get("seen_nonzero") is not False)
        if not ok:
            raise HTTPException(status_code=409, detail={"error": "OEM gripper clear failed/ambiguous", "motion_commanded": True, "move": move, "wait": wait, "before": before})
        return {
            "ok": True,
            "schema": "bioxp.oem_gripper_clear.v1",
            "motion_commanded": True,
            "physical_motion": False,
            "oem_source": "initializeMotors: setGripperCurrent(31); moveSteps(MotorGrip,+10000,true)",
            "before": before,
            "profile": profile,
            "prepare": prepare,
            "set_action_current": set_action_current,
            "move_steps_10000": move,
            "wait": wait,
            "restore": None,  # filled by finally side effect is returned via after_status below in caller-visible state
            "after_status": None,
        }
    finally:
        restore = _restore_idle(tester, "gripper_clear_finally")
        # Mutate local return if possible is intentionally skipped; always verify with status endpoint.


def gripper_home(tester: Any, *, operator_ack: str | None, reason: str | None, timeout_s: float = 15.0) -> dict[str, Any]:
    _require_action(operator_ack, GRIPPER_HOME_ACK, reason)
    before = _preflight_for_motion(tester)
    profile = _profile(tester)
    prepare = None
    home = None
    try:
        prepare = _apply_profile(tester, profile)
        home = tester.motor_oem_home_axis("g", startup=True, timeout_s=timeout_s)
        home_payload = home.get("home") if isinstance(home, dict) else home
        ok = bool((home_payload or {}).get("ok") if isinstance(home_payload, dict) else home)
        if not ok:
            raise HTTPException(status_code=409, detail={"error": "OEM gripper home failed", "motion_commanded": True, "home": home, "before": before})
        return {
            "ok": True,
            "schema": "bioxp.oem_gripper_home.v1",
            "motion_commanded": True,
            "physical_motion": False,
            "oem_source": "btnGripperHome/initializeMotors: gripper-version-specific goHome/axisSearchHome with current restore",
            "before": before,
            "profile": profile,
            "prepare": prepare,
            "home": home,
        }
    finally:
        _restore_idle(tester, "gripper_home_finally")
