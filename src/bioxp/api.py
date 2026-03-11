import asyncio
import base64
import os
import signal
import subprocess
import tempfile
import time
from contextlib import asynccontextmanager
from enum import Enum
from typing import Optional

from fastapi import FastAPI, HTTPException, Query
from pydantic import BaseModel, Field
from starlette.background import BackgroundTask
from starlette.concurrency import run_in_threadpool
from starlette.responses import StreamingResponse

from .usb_driver import BioXpTester

_tester: Optional[BioXpTester] = None
_startup_error: Optional[str] = None
_tester_lock = asyncio.Lock()
_camera_stream_lock = asyncio.Lock()
_camera_stream_state = {
    "active": False,
    "device": None,
    "fps": None,
    "quality": None,
    "width": None,
    "height": None,
    "frames_emitted": 0,
    "started_at": None,
    "last_frame_at": None,
    "last_error": None,
}


@asynccontextmanager
async def lifespan(app: FastAPI):
    del app
    global _tester, _startup_error
    try:
        alt = int(os.environ.get("BIOXP_USB_ALT", "1"))
        _tester = BioXpTester(alt=alt)
        _startup_error = None
    except Exception as exc:
        _tester = None
        _startup_error = str(exc)
        print(f"[WARN] BioXP USB runtime unavailable: {_startup_error}")
    try:
        yield
    finally:
        _tester = None


app = FastAPI(
    title="BioXP 3200 Control API",
    description="REST interface backed by the canonical USB runtime (usb_driver.py).",
    version="0.3.0",
    lifespan=lifespan,
)


def _get_tester() -> BioXpTester:
    if _tester is None:
        raise HTTPException(
            status_code=503,
            detail=_startup_error or "BioXP USB runtime not available.",
        )
    return _tester


class AxisName(str, Enum):
    X = "x"
    Y = "y"
    Z = "z"
    GRIPPER = "g"
    THERMAL_DOOR = "door"


class ThermalBankName(str, Enum):
    NEST = "nest"
    LID = "lid"
    PEDESTAL = "pedestal"


class ChillerBankName(str, Enum):
    RC = "rc"
    OC = "oc"


class MoveRelativeRequest(BaseModel):
    axis: AxisName
    steps: int = Field(..., description="Relative target in motor steps")
    wait_timeout_s: float = Field(12.0, gt=0.1, le=60.0)


class MoveAbsoluteRequest(BaseModel):
    axis: AxisName
    position_steps: int = Field(..., description="Absolute target in motor steps")
    wait_timeout_s: float = Field(12.0, gt=0.1, le=60.0)


class HomeAxisRequest(BaseModel):
    axis: AxisName
    speed: Optional[int] = Field(None, gt=0)
    timeout_s: float = Field(15.0, gt=0.1, le=90.0)


class MotionHardResetRequest(BaseModel):
    rounds: int = Field(2, ge=1, le=5)


class ThermalRequest(BaseModel):
    bank: ThermalBankName = Field(..., description="nest, lid, or pedestal")
    target_temp_c: float = Field(..., ge=0.0, le=120.0)


class ChillerRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    target_temp_c: float = Field(..., ge=-20.0, le=40.0)


class CameraSnapshotRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")


class CameraHealthRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")
    seconds: int = Field(5, ge=1, le=30)


class CameraRecoverRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")
    max_resets: int = Field(2, ge=0, le=5)


class CameraControlRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")
    cid: int = Field(..., ge=0)
    value: int = Field(..., description="Raw V4L2 control value")


class LedRgbRequest(BaseModel):
    r: int = Field(..., ge=0, le=255)
    g: int = Field(..., ge=0, le=255)
    b: int = Field(..., ge=0, le=255)
    reconnect_first: bool = True


class LedIntensityRequest(BaseModel):
    pct: int = Field(..., ge=0, le=100)


class ThermalFanRequest(BaseModel):
    speed: int = Field(..., ge=0, le=255)


class ThermalPwmRequest(BaseModel):
    bank: ThermalBankName = Field(..., description="nest or lid")
    pwm: int = Field(..., ge=0, le=100)


class ThermalRatesRequest(BaseModel):
    bank: ThermalBankName = Field(..., description="nest or lid")
    cool_rate_c_s: float = Field(..., ge=-2.0, le=0.0)
    heat_rate_c_s: float = Field(..., ge=0.0, le=2.0)


class ChillerFanRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    speed: int = Field(..., ge=0, le=255)


class ChillerPwmRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    pwm: int = Field(..., ge=0, le=100)


class ChillerRatesRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    cool_rate_c_s: float = Field(..., ge=-2.0, le=0.0)
    heat_rate_c_s: float = Field(..., ge=0.0, le=2.0)


_THERMAL_BANK_MAP = {
    ThermalBankName.NEST: BioXpTester.THERMAL_BANK_NEST,
    ThermalBankName.LID: BioXpTester.THERMAL_BANK_LID,
}

_CHILLER_BANK_MAP = {
    ChillerBankName.RC: BioXpTester.CHILLER_BANK_RC,
    ChillerBankName.OC: BioXpTester.CHILLER_BANK_OC,
}

_DEFAULT_MOTION_SPEED = 100
_DEFAULT_MOTION_ACC = 50
_MOTION_NO_DELTA_TIMEOUT_S = 2.0
_MOTION_HOME_PRECLEAR_STEPS = 500


def _axis_preset(tester: BioXpTester, axis: AxisName):
    preset = tester.motor_function_preset(axis.value)
    if not isinstance(preset, dict):
        raise HTTPException(status_code=404, detail=f"Unknown axis preset for {axis.value}")
    out = dict(preset)
    out["speed"] = _DEFAULT_MOTION_SPEED
    out["acc"] = _DEFAULT_MOTION_ACC
    return out


def _position_value(row: Optional[dict]) -> Optional[int]:
    if not isinstance(row, dict):
        return None
    value = row.get("position")
    return int(value) if isinstance(value, int) else None


def _speed_value(row: Optional[dict]) -> Optional[int]:
    if not isinstance(row, dict):
        return None
    value = row.get("speed")
    return int(value) if isinstance(value, int) else None


def _position_delta(before: Optional[dict], after: Optional[dict]) -> Optional[int]:
    left = _position_value(before)
    right = _position_value(after)
    if left is None or right is None:
        return None
    return int(right) - int(left)


def _guard_direction(axis: AxisName, steps: int, switch_activity: Optional[dict], preset: Optional[dict] = None) -> None:
    if not isinstance(switch_activity, dict) or steps == 0:
        return
    # Simultaneous left+right assertions are not physically credible as hard-stop
    # data on this instrument; treat that as unreliable raw switch state.
    if switch_activity.get("left_active") is True and switch_activity.get("right_active") is True:
        return
    left_masked = bool((preset or {}).get("disable_left", False))
    right_masked = bool((preset or {}).get("disable_right", False))
    if steps < 0 and switch_activity.get("left_active") is True and not left_masked:
        raise HTTPException(status_code=409, detail=f"Axis {axis.value} negative travel blocked by active left limit.")
    if steps > 0 and switch_activity.get("right_active") is True and not right_masked:
        raise HTTPException(status_code=409, detail=f"Axis {axis.value} positive travel blocked by active right limit.")


def _guard_absolute_target(
    axis: AxisName,
    current_position: Optional[dict],
    target_steps: int,
    switch_activity: Optional[dict],
    preset: Optional[dict] = None,
) -> None:
    current = _position_value(current_position)
    if current is None:
        return
    _guard_direction(axis, int(target_steps) - int(current), switch_activity, preset)


def _wait_for_motion_with_guardrails(
    tester: BioXpTester,
    board: int,
    motor: int,
    timeout_s: float,
    *,
    no_delta_timeout_s: float = _MOTION_NO_DELTA_TIMEOUT_S,
    poll_s: float = 0.10,
) -> dict:
    started = time.monotonic()
    deadline = started + max(0.5, float(timeout_s))
    position_row = tester.motor_get_position(board, motor=motor)
    last_position = _position_value(position_row)
    last_progress_at = started
    log_tail = []

    while True:
        speed_row = tester.motor_get_speed(board, motor=motor)
        position_row = tester.motor_get_position(board, motor=motor)
        switch_row = tester.motor_get_switch_activity(board, motor=motor)
        now = time.monotonic()
        speed = _speed_value(speed_row)
        position = _position_value(position_row)

        if position is not None and last_position is None:
            last_position = position
            last_progress_at = now
        elif position is not None and last_position is not None and position != last_position:
            last_position = position
            last_progress_at = now

        log_tail.append({"elapsed_ms": int((now - started) * 1000), "speed": speed, "position": position})
        if len(log_tail) > 20:
            log_tail = log_tail[-20:]

        if speed == 0:
            return {
                "ok": True,
                "stopped": True,
                "elapsed_ms": int((now - started) * 1000),
                "last_speed": speed,
                "position_after": position_row,
                "switch_activity_after": switch_row,
                "log_tail": log_tail,
            }

        if now >= deadline or now - last_progress_at >= max(0.5, float(no_delta_timeout_s)):
            stop = tester.motor_stop(board, motor=motor)
            settle = tester.motor_wait_stopped(board, motor=motor, timeout_s=2.0, poll_s=0.06)
            return {
                "ok": False,
                "stopped": False,
                "error": "no position delta detected for 2.0s; motion aborted."
                if now - last_progress_at >= max(0.5, float(no_delta_timeout_s))
                else "motion timed out before the motor reported stop; motion aborted.",
                "elapsed_ms": int((now - started) * 1000),
                "last_speed": speed,
                "position_after": position_row,
                "switch_activity_after": switch_row,
                "stop": stop,
                "settle": settle,
                "log_tail": log_tail,
            }

        time.sleep(max(0.02, float(poll_s)))


def _guarded_home_search(
    tester: BioXpTester,
    preset: dict,
    *,
    speed: int,
    timeout_s: float,
) -> dict:
    board = int(preset["board"])
    motor = int(preset["motor"])
    active_value = int(tester.MOTOR_SWITCH_ACTIVE_VALUE)
    effective_speed = max(1, min(int(speed), _DEFAULT_MOTION_SPEED))
    started = time.monotonic()
    deadline = started + max(2.0, float(timeout_s))
    position_before = tester.motor_get_position(board, motor=motor)
    home_before = tester.motor_query_home_switch(board, motor=motor)
    switch_before = tester.motor_get_switch_activity(board, motor=motor)

    preclear = None
    preclear_wait = None
    home_cleared = None
    if home_before.get("value") == active_value:
        preclear = tester.motor_move_relative(board, _MOTION_HOME_PRECLEAR_STEPS, motor=motor)
        if not preclear.get("ok"):
            raise HTTPException(status_code=409, detail=f"Axis {preset['label']} home preclear command failed.")
        preclear_wait = _wait_for_motion_with_guardrails(tester, board, motor, timeout_s=6.0)
        if not preclear_wait.get("ok"):
            raise HTTPException(status_code=409, detail=preclear_wait.get("error"))
        home_cleared = tester.motor_query_home_switch(board, motor=motor)
        if home_cleared.get("value") == active_value:
            raise HTTPException(status_code=409, detail=f"Axis {preset['label']} home switch stayed active after preclear; refusing to home.")

    sethome_init = tester.motor_set_home(board, motor=motor)
    move_left = tester.motor_move_left(board, speed=effective_speed, motor=motor)
    if not move_left.get("ok"):
        raise HTTPException(status_code=409, detail=f"Axis {preset['label']} homing command failed.")

    last_position = _position_value(position_before)
    last_progress_at = time.monotonic()
    polls = []
    home_hit = None
    while time.monotonic() < deadline:
        home_row = tester.motor_query_home_switch(board, motor=motor)
        speed_row = tester.motor_get_speed(board, motor=motor)
        position_row = tester.motor_get_position(board, motor=motor)
        switch_row = tester.motor_get_switch_activity(board, motor=motor)
        now = time.monotonic()
        position = _position_value(position_row)
        speed_now = _speed_value(speed_row)
        if position is not None and last_position is None:
            last_position = position
            last_progress_at = now
        elif position is not None and last_position is not None and position != last_position:
            last_position = position
            last_progress_at = now
        polls.append(
            {
                "elapsed_ms": int((now - started) * 1000),
                "home": home_row.get("value"),
                "speed": speed_now,
                "position": position,
                "left_active": switch_row.get("left_active"),
                "right_active": switch_row.get("right_active"),
            }
        )
        if len(polls) > 20:
            polls = polls[-20:]
        if home_row.get("value") == active_value:
            home_hit = home_row
            break
        if now - last_progress_at >= _MOTION_NO_DELTA_TIMEOUT_S:
            tester.motor_stop(board, motor=motor)
            tester.motor_wait_stopped(board, motor=motor, timeout_s=2.0, poll_s=0.06)
            raise HTTPException(status_code=409, detail=f"Axis {preset['label']} homing aborted after 2.0s with no position change.")
        time.sleep(0.08)

    stop = tester.motor_stop(board, motor=motor)
    wait = tester.motor_wait_stopped(board, motor=motor, timeout_s=2.0, poll_s=0.06)
    if home_hit is None:
        raise HTTPException(status_code=409, detail=f"Axis {preset['label']} homing timed out before the home switch triggered.")

    sethome_final = tester.motor_set_home(board, motor=motor)
    home_after = tester.motor_query_home_switch(board, motor=motor)
    position_after = tester.motor_get_position(board, motor=motor)
    return {
        "board": board,
        "motor": motor,
        "speed": effective_speed,
        "acc": int(preset["acc"]),
        "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
        "position_before": position_before,
        "position_after": position_after,
        "position_delta": _position_delta(position_before, position_after),
        "switch_activity_before": switch_before,
        "switch_activity_after": tester.motor_get_switch_activity(board, motor=motor),
        "home_before": home_before,
        "home_after": home_after,
        "preclear": preclear,
        "preclear_wait": preclear_wait,
        "home_cleared": home_cleared,
        "sethome_init": sethome_init,
        "move_left": move_left,
        "home_hit": home_hit,
        "stop": stop,
        "wait": wait,
        "sethome_final": sethome_final,
        "elapsed_ms": int((time.monotonic() - started) * 1000),
        "log_tail": polls,
    }


def _execute_relative_move(tester: BioXpTester, axis: AxisName, steps: int, wait_timeout_s: float) -> dict:
    preset, board_status, interlock, prep = _prepare_motion_axis(tester, axis)
    position_before = tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_before = tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    _guard_direction(axis, steps, switch_before, preset)
    move = tester.motor_move_relative(preset["board"], steps, motor=preset["motor"])
    if not move.get("ok"):
        raise HTTPException(status_code=409, detail=f"Axis {axis.value} relative move command failed.")
    wait = _wait_for_motion_with_guardrails(tester, preset["board"], preset["motor"], timeout_s=wait_timeout_s)
    if not wait.get("ok"):
        raise HTTPException(status_code=409, detail=wait.get("error"))
    position_after = wait.get("position_after") or tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_after = wait.get("switch_activity_after") or tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    return {
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "motion_profile": {
            "speed": int(preset["speed"]),
            "acc": int(preset["acc"]),
            "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
        },
        "position_before": position_before,
        "position_after": position_after,
        "position_delta": _position_delta(position_before, position_after),
        "switch_activity_before": switch_before,
        "switch_activity_after": switch_after,
        "move": move,
        "wait": wait,
    }


def _execute_absolute_move(tester: BioXpTester, axis: AxisName, position_steps: int, wait_timeout_s: float) -> dict:
    preset, board_status, interlock, prep = _prepare_motion_axis(tester, axis)
    position_before = tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_before = tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    _guard_absolute_target(axis, position_before, position_steps, switch_before, preset)
    move = tester.motor_move_absolute(preset["board"], position_steps, motor=preset["motor"])
    if not move.get("ok"):
        raise HTTPException(status_code=409, detail=f"Axis {axis.value} absolute move command failed.")
    wait = _wait_for_motion_with_guardrails(tester, preset["board"], preset["motor"], timeout_s=wait_timeout_s)
    if not wait.get("ok"):
        raise HTTPException(status_code=409, detail=wait.get("error"))
    position_after = wait.get("position_after") or tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_after = wait.get("switch_activity_after") or tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    return {
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "motion_profile": {
            "speed": int(preset["speed"]),
            "acc": int(preset["acc"]),
            "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
        },
        "position_before": position_before,
        "position_after": position_after,
        "position_delta": _position_delta(position_before, position_after),
        "target_position": int(position_steps),
        "switch_activity_before": switch_before,
        "switch_activity_after": switch_after,
        "move": move,
        "wait": wait,
    }


def _execute_home_axis(tester: BioXpTester, axis: AxisName, speed: Optional[int], timeout_s: float) -> dict:
    preset, board_status, interlock, prep = _prepare_motion_axis(tester, axis)
    home = _guarded_home_search(
        tester,
        preset,
        speed=preset["speed"] if speed is None else min(int(speed), _DEFAULT_MOTION_SPEED),
        timeout_s=min(float(timeout_s), 20.0),
    )
    return {
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "motion_profile": {
            "speed": int(preset["speed"]),
            "acc": int(preset["acc"]),
            "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
        },
        "home": home,
    }


def _prepare_motion_axis(tester: BioXpTester, axis: AxisName):
    preset = _axis_preset(tester, axis)
    board_status = tester.activate_boards(expect_reply=True)
    interlock = None
    if axis is not AxisName.THERMAL_DOOR:
        interlock = tester.motor_prepare_motion_interlock(force_lock=True)
    prep = tester.motor_prepare_axis(
        preset["board"],
        motor=preset["motor"],
        run_current=preset["run_current"],
        standby_current=preset["standby_current"],
        speed=preset["speed"],
        acc=preset["acc"],
        stall_guard=preset.get("stall_guard"),
        ramp_mode=preset.get("ramp_mode"),
        disable_right=bool(preset.get("disable_right", False)),
        disable_left=bool(preset.get("disable_left", False)),
        rdiv=preset.get("rdiv"),
        pdiv=preset.get("pdiv"),
        warm_enable=bool(preset.get("warm_enable", False)),
    )
    return preset, board_status, interlock, prep


def _status_payload() -> dict:
    runtime_available = _tester is not None
    board_status = None
    deck_io_snapshot = None
    status_error = None
    hardware_connected = False
    status = "degraded"

    if runtime_available:
        try:
            board_status = _tester.activate_boards(expect_reply=True)
            if all(reply is None for reply in board_status.values()):
                _tester.reconnect()
                board_status = _tester.activate_boards(expect_reply=True)
            hardware_connected = any(reply is not None for reply in board_status.values())
            status = "ok" if hardware_connected else "degraded"
            if hardware_connected:
                try:
                    deck_io_snapshot = _tester.io_snapshot(_tester.BOARD_DECK)
                except Exception as exc:
                    status_error = f"deck IO snapshot unavailable: {exc}"
        except Exception as exc:
            status_error = str(exc)

    return {
        "status": status if runtime_available else "degraded",
        "transport": "usb",
        "runtime_available": runtime_available,
        "hardware_connected": hardware_connected,
        "startup_error": _startup_error,
        "status_error": status_error,
        "board_status": board_status,
        "deck_io_snapshot": deck_io_snapshot,
    }


def _motion_power_status_payload(tester: BioXpTester) -> dict:
    board_status = tester.activate_boards(expect_reply=True)
    if all(reply is None for reply in board_status.values()):
        tester.reconnect()
        board_status = tester.activate_boards(expect_reply=True)
    hardware_connected = any(reply is not None for reply in board_status.values())
    deck_io_snapshot = tester.io_snapshot(tester.BOARD_DECK) if hardware_connected else None
    return {
        "hardware_connected": hardware_connected,
        "board_status": board_status,
        "deck_io_snapshot": deck_io_snapshot,
        "rail_24v": tester.motor_query_24v_sensor(),
        "motion_arm": tester.motion_arm_state(),
        "latch_override": tester.motion_latch_override_state(),
    }


def _camera_snapshot_with_data(tester: BioXpTester, preferred: str) -> dict:
    result = _camera_capture_snapshot_direct(tester, preferred)
    image_b64 = None
    image_error = None
    path = result.get("path")
    if result.get("ok") and path and os.path.exists(path):
        try:
            with open(path, "rb") as handle:
                image_b64 = base64.b64encode(handle.read()).decode("ascii")
        except Exception as exc:
            image_error = str(exc)
    result["image_b64"] = image_b64
    result["image_error"] = image_error
    return result


async def _run_blocking(label: str, func, timeout_s: float = 30.0):
    async with _tester_lock:
        try:
            return await asyncio.wait_for(run_in_threadpool(func), timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise HTTPException(status_code=504, detail=f"{label} timed out after {timeout_s:.0f}s") from exc


def _pick_capture_device(tester: BioXpTester, preferred: str) -> dict:
    pick = tester.camera_wait_pick_device(
        preferred=preferred,
        timeout_s=4.0,
        poll_s=0.25,
        require_controls=True,
    )
    if not pick.get("ok"):
        return {
            "ok": False,
            "device": None,
            "error": pick.get("error") or "no capture-capable camera device found",
            "rows": pick.get("rows", []),
        }
    return pick


def _camera_device_rows_local() -> list[dict]:
    rows = []
    for name in sorted(os.listdir("/dev")):
        if not name.startswith("video"):
            continue
        device = f"/dev/{name}"
        label = ""
        try:
            with open(f"/sys/class/video4linux/{name}/name", "r", encoding="utf-8", errors="ignore") as handle:
                label = handle.read().strip()
        except OSError:
            label = ""
        rows.append({"device": device, "name": label})
    return rows


def _pick_stream_device(preferred: str) -> dict:
    rows = _camera_device_rows_local()
    if preferred and os.path.exists(preferred):
        return {"ok": True, "device": preferred, "rows": rows}
    for row in rows:
        if row["device"] == "/dev/video0":
            return {"ok": True, "device": row["device"], "rows": rows}
    if rows:
        return {"ok": True, "device": rows[0]["device"], "rows": rows}
    return {"ok": False, "device": None, "rows": [], "error": "no /dev/video* devices found"}


def _camera_reset_local(preferred: str) -> dict:
    device = preferred if preferred and os.path.exists(preferred) else "/dev/video0"
    ps = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    killed: list[int] = []
    errors: list[dict] = []
    for raw in (ps.stdout or "").splitlines():
        line = raw.strip()
        if not line or "ffmpeg" not in line or device not in line:
            continue
        parts = line.split(None, 1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            killed.append(pid)
        except ProcessLookupError:
            continue
        except Exception as exc:
            errors.append({"pid": pid, "error": str(exc)})

    if killed:
        time.sleep(0.5)
        for pid in killed:
            try:
                os.kill(pid, 0)
            except OSError:
                continue
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                continue
            except Exception as exc:
                errors.append({"pid": pid, "error": str(exc)})

    lock_released = False
    if _camera_stream_lock.locked():
        try:
            _camera_stream_lock.release()
            lock_released = True
        except RuntimeError:
            lock_released = False

    _camera_stream_state.update(
        {
            "active": False,
            "device": device,
            "last_error": "stream stopped",
            "last_frame_at": None,
        }
    )

    remaining = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    survivors = []
    for raw in (remaining.stdout or "").splitlines():
        line = raw.strip()
        if not line or "ffmpeg" not in line or device not in line:
            continue
        parts = line.split(None, 1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        survivors.append({"pid": pid, "cmd": parts[1] if len(parts) > 1 else ""})

    return {
        "ok": not survivors,
        "device": device,
        "killed_pids": killed,
        "lock_released": lock_released,
        "survivors": survivors,
        "errors": errors,
    }


def _camera_stream_state_payload() -> dict:
    last_frame_at = _camera_stream_state.get("last_frame_at")
    started_at = _camera_stream_state.get("started_at")
    return {
        **_camera_stream_state,
        "last_frame_age_s": None if not last_frame_at else round(max(0.0, time.time() - float(last_frame_at)), 2),
        "stream_age_s": None if not started_at else round(max(0.0, time.time() - float(started_at)), 2),
    }


def _camera_capture_snapshot_direct(tester: BioXpTester, preferred: str) -> dict:
    pick = _pick_capture_device(tester, preferred)
    if not pick.get("ok"):
        return {"ok": False, "error": pick.get("error"), "device": None, "path": None}

    device = pick["device"]
    out_dir = os.path.join(tempfile.gettempdir(), "bioxp-api-camera")
    os.makedirs(out_dir, exist_ok=True)
    ts = __import__("time").strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"snapshot_{ts}.jpg")
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "v4l2",
        "-i",
        device,
        "-frames:v",
        "1",
        out_path,
    ]
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "ffmpeg snapshot timeout", "device": device, "path": out_path}

    output = (proc.stdout or "").strip()
    exists = os.path.exists(out_path)
    size = os.path.getsize(out_path) if exists else 0
    ok = proc.returncode == 0 and exists and size > 0
    return {
        "ok": ok,
        "device": device,
        "path": out_path,
        "size": size,
        "rc": int(proc.returncode),
        "output": output,
        "error": None if ok else (output or "snapshot failed"),
        "pick": pick,
    }


def _camera_stream_health_direct(tester: BioXpTester, preferred: str, seconds: int) -> dict:
    if _camera_stream_lock.locked():
        return {
            "ok": False,
            "busy": True,
            "stream_active": True,
            "device": preferred,
            "error": "live stream is active; stop the stream before running stream health",
            "stream_state": _camera_stream_state_payload(),
        }

    pick = _pick_capture_device(tester, preferred)
    if not pick.get("ok"):
        return {"ok": False, "error": pick.get("error"), "device": None}

    device = pick["device"]
    dur = max(1, int(seconds))
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "info",
        "-f",
        "v4l2",
        "-i",
        device,
        "-t",
        str(dur),
        "-f",
        "null",
        "-",
    ]
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=float(dur) + 8.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "ffmpeg stream timeout", "device": device}

    output = proc.stdout or ""
    import re
    frames = [int(x) for x in re.findall(r"frame=\s*([0-9]+)", output)]
    fps_vals = [float(x) for x in re.findall(r"fps=\s*([0-9.]+)", output)]
    frame_max = max(frames) if frames else 0
    fps_last = fps_vals[-1] if fps_vals else None
    ok = proc.returncode == 0 and frame_max > 0
    return {
        "ok": ok,
        "device": device,
        "seconds": dur,
        "frames": frame_max,
        "fps_last": fps_last,
        "rc": int(proc.returncode),
        "error": None if ok else "stream health failed",
        "output_tail": output[-600:],
        "pick": pick,
    }


def _camera_devices_payload(tester: BioXpTester) -> dict:
    rows = tester.camera_device_rows()
    preferred = None
    pick = _pick_capture_device(tester, "/dev/video0")
    if pick.get("ok"):
        preferred = pick.get("device")
    for row in rows:
        row["capture_candidate"] = row.get("device") == preferred
    return {"ok": True, "rows": rows, "preferred_device": preferred}


async def _camera_mjpeg_response(
    tester: BioXpTester,
    preferred: str,
    fps: int,
    quality: int,
    width: int,
    height: int,
):
    fps = max(1, min(int(fps), 30))
    quality = max(2, min(int(quality), 15))
    width = max(160, min(int(width), 1920))
    height = max(120, min(int(height), 1080))
    if _camera_stream_lock.locked():
        await run_in_threadpool(_camera_reset_local, preferred)
        await asyncio.sleep(0.15)
    await _camera_stream_lock.acquire()
    proc = None
    try:
        pick = _pick_stream_device(preferred)
        if not pick.get("ok"):
            raise HTTPException(status_code=503, detail=pick.get("error") or "No capture-capable camera device found")

        device = pick["device"]
        _camera_stream_state.update(
            {
                "active": True,
                "device": device,
                "fps": fps,
                "quality": quality,
                "width": width,
                "height": height,
                "frames_emitted": 0,
                "started_at": time.time(),
                "last_frame_at": None,
                "last_error": None,
            }
        )
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-avioflags",
            "direct",
            "-f",
            "v4l2",
            "-input_format",
            "mjpeg",
            "-framerate",
            str(fps),
            "-video_size",
            f"{width}x{height}",
            "-i",
            device,
            "-an",
            "-vf",
            f"fps={fps}",
            "-q:v",
            str(quality),
            "-vcodec",
            "mjpeg",
            "-f",
            "image2pipe",
            "pipe:1",
        ]
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        if proc.stdout is None:
            raise HTTPException(status_code=500, detail="ffmpeg stream stdout unavailable")
        await asyncio.sleep(0.25)
        if proc.returncode is not None:
            stderr = b""
            if proc.stderr is not None:
                try:
                    stderr = await asyncio.wait_for(proc.stderr.read(), timeout=0.5)
                except asyncio.TimeoutError:
                    stderr = b""
            detail = stderr.decode("utf-8", errors="replace").strip() or "camera stream exited before producing frames"
            raise HTTPException(status_code=503, detail=detail)
    except Exception as exc:
        detail = exc.detail if isinstance(exc, HTTPException) else str(exc)
        _camera_stream_state.update({"active": False, "last_error": detail})
        if proc is not None and proc.returncode is None:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), timeout=3.0)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
        if _camera_stream_lock.locked():
            _camera_stream_lock.release()
        raise

    cleanup_started = False
    cleanup_guard = asyncio.Lock()

    async def cleanup():
        nonlocal cleanup_started
        async with cleanup_guard:
            if cleanup_started:
                return
            cleanup_started = True
            if proc is not None and proc.returncode is None:
                proc.terminate()
                try:
                    await asyncio.wait_for(proc.wait(), timeout=3.0)
                except asyncio.TimeoutError:
                    proc.kill()
                    await proc.wait()
            _camera_stream_state["active"] = False
            if _camera_stream_lock.locked():
                _camera_stream_lock.release()

    async def iterator():
        buffer = bytearray()
        soi = b"\xff\xd8"
        eoi = b"\xff\xd9"
        try:
            while True:
                if proc.stdout is None:
                    break
                chunk = await proc.stdout.read(16384)
                if not chunk:
                    break
                buffer.extend(chunk)
                while True:
                    start = buffer.find(soi)
                    if start == -1:
                        if len(buffer) > 65536:
                            buffer.clear()
                        break
                    if start > 0:
                        del buffer[:start]
                    end = buffer.find(eoi, 2)
                    if end == -1:
                        break
                    frame = bytes(buffer[: end + 2])
                    del buffer[: end + 2]
                    _camera_stream_state["frames_emitted"] = int(_camera_stream_state.get("frames_emitted") or 0) + 1
                    _camera_stream_state["last_frame_at"] = time.time()
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                    )
                    yield header + frame + b"\r\n"
        finally:
            await cleanup()

    headers = {
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
        "Pragma": "no-cache",
        "X-BioXp-Camera-Device": device,
    }
    return StreamingResponse(
        iterator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers=headers,
        background=BackgroundTask(cleanup),
    )


@app.get("/status")
async def get_status():
    return await _run_blocking("BioXP status", _status_payload, timeout_s=20.0)


@app.post("/reconnect")
async def reconnect_runtime():
    tester = _get_tester()
    await _run_blocking("BioXP reconnect", tester.reconnect, timeout_s=20.0)
    return {
        "ok": True,
        "message": "USB runtime reconnect requested.",
        **(await _run_blocking("BioXP status", _status_payload, timeout_s=20.0)),
    }


@app.get("/motion/axis/{axis}/status")
async def axis_status(axis: AxisName):
    tester = _get_tester()
    preset = _axis_preset(tester, axis)
    return await _run_blocking(
        f"Axis {axis.value} status",
        lambda: {
        "axis": axis.value,
        "preset": preset,
        "status": tester.motor_axis_status(preset["board"], motor=preset["motor"]),
        "switch_activity": tester.motor_get_switch_activity(preset["board"], motor=preset["motor"]),
        },
        timeout_s=20.0,
    )


@app.post("/motion/interlock/prepare")
async def prepare_interlock():
    tester = _get_tester()
    return await _run_blocking(
        "Motion interlock prep",
        lambda: (
            tester.activate_boards(expect_reply=True),
            tester.motor_prepare_motion_interlock(force_lock=True),
        )[1],
        timeout_s=25.0,
    )


@app.get("/motion/power/status")
async def motion_power_status():
    tester = _get_tester()
    return await _run_blocking(
        "Motion power status",
        lambda: _motion_power_status_payload(tester),
        timeout_s=25.0,
    )


@app.post("/motion/power/enable")
async def motion_power_enable():
    tester = _get_tester()
    return await _run_blocking(
        "Motion power enable",
        lambda: tester.motor_enable_sequence(conservative=True),
        timeout_s=35.0,
    )


@app.post("/motion/power/diag")
async def motion_power_diag():
    tester = _get_tester()
    return await _run_blocking(
        "Motion driver power diagnostic",
        lambda: tester.motor_driver_power_diag(axis_keys=("x", "y", "z", "g", "door")),
        timeout_s=45.0,
    )


@app.post("/motion/hard_reset")
async def motion_hard_reset(req: MotionHardResetRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Motion hard reset",
        lambda: tester.motor_hard_reset(rounds=req.rounds),
        timeout_s=max(45.0, 20.0 * float(req.rounds)),
    )


@app.post("/motion/clear_lock")
async def clear_lock():
    tester = _get_tester()
    return await _run_blocking(
        "Head lock clear",
        lambda: (
            tester.activate_boards(expect_reply=True),
            tester.motor_ensure_head_clearance(
                force_rehome=True,
                ensure_interlock=True,
                preclear_abs=tester.MOTOR_HEAD_CLEARANCE_LIFT_ABS,
            ),
        )[1],
        timeout_s=45.0,
    )


@app.get("/latch/status")
async def latch_status():
    tester = _get_tester()
    snapshot = await _run_blocking(
        "Latch status",
        lambda: tester.io_snapshot(tester.BOARD_DECK),
        timeout_s=20.0,
    )
    return {
        "snapshot": snapshot,
        "rail_24v": snapshot.get(0),
        "door_sensor": snapshot.get(1),
        "solenoid_state": snapshot.get(2),
        "latch_sensor": snapshot.get(3),
    }


@app.post("/latch/lock")
async def latch_lock():
    tester = _get_tester()
    return await _run_blocking("Latch lock", lambda: tester.latch_oem(True), timeout_s=20.0)


@app.post("/latch/unlock")
async def latch_unlock():
    tester = _get_tester()
    return await _run_blocking("Latch unlock", lambda: tester.latch_oem(False), timeout_s=20.0)


@app.post("/led/off")
async def led_off():
    tester = _get_tester()
    return await _run_blocking("LED off", tester.strip_off, timeout_s=20.0)


@app.post("/led/on")
async def led_on():
    tester = _get_tester()
    return await _run_blocking("LED on", tester.strip_on, timeout_s=20.0)


@app.post("/led/pct")
async def led_pct(req: LedIntensityRequest):
    tester = _get_tester()
    return await _run_blocking("LED percent", lambda: tester.strip_set_pct(req.pct), timeout_s=20.0)


@app.post("/led/rgb")
async def led_rgb(req: LedRgbRequest):
    tester = _get_tester()
    return await _run_blocking(
        "LED RGB",
        lambda: tester.strip_set_rgb(
            req.r,
            req.g,
            req.b,
            reconnect_first=bool(req.reconnect_first),
        ),
        timeout_s=20.0,
    )


@app.post("/motion/axis/relative")
async def move_axis_relative(req: MoveRelativeRequest):
    tester = _get_tester()
    return await _run_blocking(
        f"Axis {req.axis.value} relative move",
        lambda: _execute_relative_move(tester, req.axis, req.steps, req.wait_timeout_s),
        timeout_s=max(25.0, req.wait_timeout_s + 10.0),
    )


@app.post("/motion/axis/absolute")
async def move_axis_absolute(req: MoveAbsoluteRequest):
    tester = _get_tester()
    return await _run_blocking(
        f"Axis {req.axis.value} absolute move",
        lambda: _execute_absolute_move(tester, req.axis, req.position_steps, req.wait_timeout_s),
        timeout_s=max(35.0, req.wait_timeout_s + 10.0),
    )


@app.post("/motion/axis/home")
async def home_axis(req: HomeAxisRequest):
    tester = _get_tester()
    return await _run_blocking(
        f"Axis {req.axis.value} home",
        lambda: _execute_home_axis(tester, req.axis, req.speed, req.timeout_s),
        timeout_s=max(35.0, req.timeout_s + 10.0),
    )


@app.post("/thermal/baseline")
async def thermal_baseline():
    tester = _get_tester()
    return await _run_blocking("Thermal baseline", lambda: tester.thermal_apply_vendor_baseline(verify=True), timeout_s=30.0)


@app.get("/thermal/snapshot")
async def thermal_snapshot():
    tester = _get_tester()
    return await _run_blocking("Thermal snapshot", tester.thermal_snapshot, timeout_s=25.0)


@app.post("/thermal/set_temp")
async def set_thermal_temp(req: ThermalRequest):
    tester = _get_tester()
    if req.bank is ThermalBankName.PEDESTAL:
        return await _run_blocking("Thermal pedestal setpoint", lambda: tester.thermal_set_ped_temp(req.target_temp_c, verify=True), timeout_s=25.0)
    bank = _THERMAL_BANK_MAP[req.bank]
    return await _run_blocking("Thermal setpoint", lambda: tester.thermal_set_target_temp(bank, req.target_temp_c, verify=True), timeout_s=25.0)


@app.post("/thermal/fan")
async def set_thermal_fan(req: ThermalFanRequest):
    tester = _get_tester()
    return await _run_blocking("Thermal fan", lambda: tester.thermal_set_fan(req.speed, verify=True), timeout_s=25.0)


@app.post("/thermal/pwm")
async def set_thermal_pwm(req: ThermalPwmRequest):
    tester = _get_tester()
    if req.bank not in _THERMAL_BANK_MAP:
        raise HTTPException(status_code=400, detail="PWM is only supported for nest or lid.")
    return await _run_blocking("Thermal PWM", lambda: tester.thermal_set_pwm(_THERMAL_BANK_MAP[req.bank], req.pwm, verify=True), timeout_s=25.0)


@app.post("/thermal/rates")
async def set_thermal_rates(req: ThermalRatesRequest):
    tester = _get_tester()
    if req.bank not in _THERMAL_BANK_MAP:
        raise HTTPException(status_code=400, detail="Rate control is only supported for nest or lid.")
    return await _run_blocking(
        "Thermal rates",
        lambda: tester.thermal_set_rates(
            _THERMAL_BANK_MAP[req.bank],
            cool_rate_c_s=req.cool_rate_c_s,
            heat_rate_c_s=req.heat_rate_c_s,
            verify=True,
        ),
        timeout_s=25.0,
    )


@app.post("/thermal/fast_profile")
async def thermal_fast_profile():
    tester = _get_tester()
    return await _run_blocking("Thermal fast profile", lambda: tester.thermal_apply_fast_profile(verify=True), timeout_s=30.0)


@app.post("/thermal/hard_reset")
async def thermal_hard_reset():
    tester = _get_tester()
    return await _run_blocking("Thermal hard reset", tester.thermal_hard_reset, timeout_s=40.0)


@app.post("/chiller/baseline")
async def chiller_baseline():
    tester = _get_tester()
    return await _run_blocking("Chiller baseline", lambda: tester.chiller_apply_vendor_baseline(verify=True), timeout_s=30.0)


@app.get("/chiller/snapshot")
async def chiller_snapshot():
    tester = _get_tester()
    return await _run_blocking("Chiller snapshot", tester.chiller_snapshot, timeout_s=25.0)


@app.post("/chiller/set_temp")
async def set_chiller_temp(req: ChillerRequest):
    tester = _get_tester()
    bank = _CHILLER_BANK_MAP[req.bank]
    return await _run_blocking(
        "Chiller setpoint",
        lambda: (
            tester.chiller_activate(),
            tester.chiller_set_target_temp(bank, req.target_temp_c, verify=True),
        )[1],
        timeout_s=25.0,
    )


@app.post("/chiller/fan")
async def set_chiller_fan(req: ChillerFanRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Chiller fan",
        lambda: (
            tester.chiller_activate(),
            tester.chiller_set_fan(_CHILLER_BANK_MAP[req.bank], req.speed, verify=True),
        )[1],
        timeout_s=25.0,
    )


@app.post("/chiller/pwm")
async def set_chiller_pwm(req: ChillerPwmRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Chiller PWM",
        lambda: (
            tester.chiller_activate(),
            tester.chiller_set_pwm(_CHILLER_BANK_MAP[req.bank], req.pwm, verify=True),
        )[1],
        timeout_s=25.0,
    )


@app.post("/chiller/rates")
async def set_chiller_rates(req: ChillerRatesRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Chiller rates",
        lambda: (
            tester.chiller_activate(),
            tester.chiller_set_rates(
                _CHILLER_BANK_MAP[req.bank],
                cool_rate_c_s=req.cool_rate_c_s,
                heat_rate_c_s=req.heat_rate_c_s,
                verify=True,
            ),
        )[1],
        timeout_s=25.0,
    )


@app.post("/chiller/hard_reset")
async def chiller_hard_reset():
    tester = _get_tester()
    return await _run_blocking("Chiller hard reset", tester.chiller_hard_reset, timeout_s=40.0)


@app.get("/camera/devices")
async def camera_devices():
    tester = _get_tester()
    return await _run_blocking("Camera devices", lambda: _camera_devices_payload(tester), timeout_s=15.0)


@app.get("/camera/controls")
async def camera_controls(device: str = "/dev/video0"):
    tester = _get_tester()
    return await _run_blocking("Camera controls", lambda: tester.camera_enumerate_controls(device=device), timeout_s=15.0)


@app.post("/camera/control")
async def camera_control(req: CameraControlRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Camera control",
        lambda: {
            **tester.v4l2_set_ctrl(req.cid, req.value, device=req.device),
            "stream_active": bool(_camera_stream_state.get("active")),
            "stream_state": _camera_stream_state_payload(),
        },
        timeout_s=15.0,
    )


@app.post("/camera/snapshot")
async def camera_snapshot(req: CameraSnapshotRequest):
    tester = _get_tester()
    return await _run_blocking("Camera snapshot", lambda: _camera_snapshot_with_data(tester, req.device), timeout_s=20.0)


@app.post("/camera/stream_health")
async def camera_stream_health(req: CameraHealthRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Camera stream health",
        lambda: _camera_stream_health_direct(tester, req.device, req.seconds),
        timeout_s=max(20.0, req.seconds + 10.0),
    )


@app.post("/camera/auto_recover")
async def camera_auto_recover(req: CameraRecoverRequest):
    if _camera_stream_lock.locked():
        return {
            "ok": False,
            "busy": True,
            "stream_active": True,
            "device": req.device,
            "error": "live stream is active; stop the stream before running auto recover",
            "stream_state": _camera_stream_state_payload(),
        }
    tester = _get_tester()
    return await _run_blocking(
        "Camera auto recover",
        lambda: tester.camera_auto_oneclick(device=req.device, max_resets=req.max_resets),
        timeout_s=75.0,
    )


@app.post("/camera/reset")
async def camera_reset(req: CameraSnapshotRequest):
    return _camera_reset_local(req.device)


@app.post("/camera/stop")
async def camera_stop(req: CameraSnapshotRequest):
    return await run_in_threadpool(_camera_reset_local, req.device)


@app.get("/camera/stream_state")
async def camera_stream_state():
    return _camera_stream_state_payload()


@app.get("/camera/mjpeg")
async def camera_mjpeg(
    device: str = "/dev/video0",
    fps: int = Query(8, ge=1, le=30),
    quality: int = Query(7, ge=2, le=15),
    width: int = Query(640, ge=160, le=1920),
    height: int = Query(480, ge=120, le=1080),
):
    tester = _get_tester()
    return await _camera_mjpeg_response(
        tester,
        preferred=device,
        fps=fps,
        quality=quality,
        width=width,
        height=height,
    )


@app.post("/liquid/aspirate")
async def aspirate_not_available():
    raise HTTPException(
        status_code=501,
        detail="Liquid handling is not exported through the canonical USB runtime API yet.",
    )


@app.post("/liquid/dispense")
async def dispense_not_available():
    raise HTTPException(
        status_code=501,
        detail="Liquid handling is not exported through the canonical USB runtime API yet.",
    )
