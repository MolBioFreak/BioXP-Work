from contextlib import asynccontextmanager
from enum import Enum
from typing import Optional

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from .can_driver import BioXpCanDriver, BoardAssy, MotorAxis

_robot: Optional[BioXpCanDriver] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _robot
    try:
        _robot = BioXpCanDriver(channel="can0")
    except OSError:
        _robot = None
        print("[WARN] SocketCAN interface 'can0' not found. "
              "Running in dry-run mode (no hardware).")
    yield
    if _robot and _robot.bus:
        _robot.bus.shutdown()


app = FastAPI(
    title="BioXP 3200 Control API",
    description="REST interface for the BioXP 3200 liquid handling robot, "
                "driven by a reverse-engineered Python CAN driver over SocketCAN.",
    version="0.1.0",
    lifespan=lifespan,
)


def _get_robot() -> BioXpCanDriver:
    if _robot is None:
        raise HTTPException(
            status_code=503,
            detail="CAN bus not available. Is the SocketCAN interface up?",
        )
    return _robot


class AxisName(str, Enum):
    X = "X"
    Y = "Y"
    Z = "Z"
    GRIPPER = "GRIPPER"

_axis_map = {
    AxisName.X: MotorAxis.X,
    AxisName.Y: MotorAxis.Y,
    AxisName.Z: MotorAxis.Z,
    AxisName.GRIPPER: MotorAxis.GRIPPER,
}


class MoveRequest(BaseModel):
    axis: AxisName
    position_steps: int = Field(..., description="Absolute target in stepper steps")


class AspirateRequest(BaseModel):
    volume_ul: float = Field(..., gt=0, le=100, description="Volume in microlitres (0.1-100)")
    tip_pressure_profile: str = Field("1R", description="Firmware pressure profile code")


class DispenseRequest(BaseModel):
    volume_ul: float = Field(..., gt=0, le=100, description="Volume in microlitres (0.1-100)")
    tip_pressure_profile: str = Field("1R", description="Firmware pressure profile code")


class ThermalRequest(BaseModel):
    target_temp_c: float = Field(..., ge=4, le=100, description="Target temperature in Celsius")


class StatusResponse(BaseModel):
    status: str
    can_interface: str
    hardware_connected: bool


@app.get("/status", response_model=StatusResponse)
async def get_status():
    return StatusResponse(
        status="ok",
        can_interface="can0",
        hardware_connected=_robot is not None,
    )


@app.post("/motor/move")
async def move_motor(req: MoveRequest):
    robot = _get_robot()
    robot.move_axis(_axis_map[req.axis], req.position_steps)
    return {
        "result": "ok",
        "axis": req.axis.value,
        "position_steps": req.position_steps,
    }


@app.post("/liquid/aspirate")
async def aspirate(req: AspirateRequest):
    robot = _get_robot()
    robot.aspirate(req.volume_ul, req.tip_pressure_profile)
    return {
        "result": "ok",
        "volume_ul": req.volume_ul,
        "profile": req.tip_pressure_profile,
    }


@app.post("/liquid/dispense")
async def dispense(req: DispenseRequest):
    robot = _get_robot()
    robot.aspirate(req.volume_ul, req.tip_pressure_profile)
    return {
        "result": "ok",
        "volume_ul": req.volume_ul,
        "profile": req.tip_pressure_profile,
    }


@app.post("/thermal/set_temp")
async def set_thermal_temp(req: ThermalRequest):
    robot = _get_robot()
    robot.set_thermal_temperature(req.target_temp_c)
    return {
        "result": "ok",
        "target_temp_c": req.target_temp_c,
    }
