import os
from contextlib import asynccontextmanager
from enum import Enum
from typing import Optional

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from .usb_driver import BioXpTester

_tester: Optional[BioXpTester] = None
_startup_error: Optional[str] = None


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
    version="0.2.0",
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


class ThermalRequest(BaseModel):
    bank: ThermalBankName = Field(..., description="nest, lid, or pedestal")
    target_temp_c: float = Field(..., ge=0.0, le=120.0)


class ChillerRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    target_temp_c: float = Field(..., ge=-20.0, le=40.0)


class CameraSnapshotRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")


_THERMAL_BANK_MAP = {
    ThermalBankName.NEST: BioXpTester.THERMAL_BANK_NEST,
    ThermalBankName.LID: BioXpTester.THERMAL_BANK_LID,
}

_CHILLER_BANK_MAP = {
    ChillerBankName.RC: BioXpTester.CHILLER_BANK_RC,
    ChillerBankName.OC: BioXpTester.CHILLER_BANK_OC,
}


def _axis_preset(tester: BioXpTester, axis: AxisName):
    preset = tester.motor_function_preset(axis.value)
    if not isinstance(preset, dict):
        raise HTTPException(status_code=404, detail=f"Unknown axis preset for {axis.value}")
    return preset


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


@app.get("/status")
async def get_status():
    return {
        "status": "ok" if _tester is not None else "degraded",
        "transport": "usb",
        "hardware_connected": _tester is not None,
        "startup_error": _startup_error,
    }


@app.get("/motion/axis/{axis}/status")
async def axis_status(axis: AxisName):
    tester = _get_tester()
    preset = _axis_preset(tester, axis)
    return {
        "axis": axis.value,
        "preset": preset,
        "status": tester.motor_axis_status(preset["board"], motor=preset["motor"]),
        "switch_activity": tester.motor_get_switch_activity(preset["board"], motor=preset["motor"]),
    }


@app.post("/motion/interlock/prepare")
async def prepare_interlock():
    tester = _get_tester()
    tester.activate_boards(expect_reply=True)
    return tester.motor_prepare_motion_interlock(force_lock=True)


@app.post("/motion/clear_lock")
async def clear_lock():
    tester = _get_tester()
    tester.activate_boards(expect_reply=True)
    return tester.motor_ensure_head_clearance(
        force_rehome=True,
        ensure_interlock=True,
        preclear_abs=tester.MOTOR_HEAD_CLEARANCE_LIFT_ABS,
    )


@app.post("/motion/axis/relative")
async def move_axis_relative(req: MoveRelativeRequest):
    tester = _get_tester()
    preset, board_status, interlock, prep = _prepare_motion_axis(tester, req.axis)
    move = tester.motor_move_relative(preset["board"], req.steps, motor=preset["motor"])
    wait = tester.motor_wait_stopped(preset["board"], motor=preset["motor"], timeout_s=req.wait_timeout_s)
    pos = tester.motor_get_position(preset["board"], motor=preset["motor"])
    sw = tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    return {
        "axis": req.axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "move": move,
        "wait": wait,
        "position_after": pos,
        "switch_activity_after": sw,
    }


@app.post("/motion/axis/absolute")
async def move_axis_absolute(req: MoveAbsoluteRequest):
    tester = _get_tester()
    preset, board_status, interlock, prep = _prepare_motion_axis(tester, req.axis)
    move = tester.motor_move_absolute(preset["board"], req.position_steps, motor=preset["motor"])
    wait = tester.motor_wait_stopped(preset["board"], motor=preset["motor"], timeout_s=req.wait_timeout_s)
    pos = tester.motor_get_position(preset["board"], motor=preset["motor"])
    sw = tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    return {
        "axis": req.axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "move": move,
        "wait": wait,
        "position_after": pos,
        "switch_activity_after": sw,
    }


@app.post("/motion/axis/home")
async def home_axis(req: HomeAxisRequest):
    tester = _get_tester()
    preset, board_status, interlock, prep = _prepare_motion_axis(tester, req.axis)
    speed = preset["speed"] if req.speed is None else int(req.speed)
    result = tester.motor_axis_search_home(
        preset["board"],
        motor=preset["motor"],
        speed=speed,
        timeout_s=req.timeout_s,
        home_active_value=int(tester.MOTOR_SWITCH_ACTIVE_VALUE),
    )
    return {
        "axis": req.axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "home": result,
    }


@app.post("/thermal/baseline")
async def thermal_baseline():
    tester = _get_tester()
    return tester.thermal_apply_vendor_baseline(verify=True)


@app.post("/thermal/set_temp")
async def set_thermal_temp(req: ThermalRequest):
    tester = _get_tester()
    if req.bank is ThermalBankName.PEDESTAL:
        return tester.thermal_set_ped_temp(req.target_temp_c, verify=True)
    bank = _THERMAL_BANK_MAP[req.bank]
    return tester.thermal_set_target_temp(bank, req.target_temp_c, verify=True)


@app.post("/chiller/baseline")
async def chiller_baseline():
    tester = _get_tester()
    return tester.chiller_apply_vendor_baseline(verify=True)


@app.post("/chiller/set_temp")
async def set_chiller_temp(req: ChillerRequest):
    tester = _get_tester()
    bank = _CHILLER_BANK_MAP[req.bank]
    tester.chiller_activate()
    return tester.chiller_set_target_temp(bank, req.target_temp_c, verify=True)


@app.post("/camera/snapshot")
async def camera_snapshot(req: CameraSnapshotRequest):
    tester = _get_tester()
    return tester.camera_capture_snapshot(preferred=req.device)


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
