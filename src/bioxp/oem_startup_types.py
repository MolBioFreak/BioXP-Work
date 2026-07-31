from __future__ import annotations

from enum import Enum
from typing import Literal, Optional
from pydantic import BaseModel, Field, field_validator


class OemStartupState(str, Enum):
    CREATED = "created"
    CONFIG_LOADING = "config_loading"
    CONFIG_LOADED = "config_loaded"
    CONFIG_MISSING = "config_missing"
    BACKEND_READY = "backend_ready"
    CONTROL_LIB_CONSTRUCTED = "control_lib_constructed"
    PIPETTE_CHECKED = "pipette_checked"
    MOTORS_CONFIGURED_WITHOUT_MOTION = "motors_configured_without_motion"
    INITIALIZE_ENVIRONMENT = "initialize_environment"
    INITIAL_CHECK_BEFORE_DOOR = "initial_check_before_door"
    WAITING_FOR_DOOR_CLOSE = "waiting_for_door_close"
    DOOR_CLOSE_OBSERVED = "door_close_observed"
    INITIAL_CHECK_AFTER_DOOR = "initial_check_after_door"
    INITIALIZE_SYSTEM_QUEUED = "initialize_system_queued"
    INITIALIZE_SYSTEM_RUNNING = "initialize_system_running"
    INITIALIZE_MOTION_RUNNING = "initialize_motion_running"
    INITIALIZE_MOTORS_RUNNING = "initialize_motors_running"
    HOMING_Z = "homing_z"
    HOMING_GRIPPER_CLEAR = "homing_gripper_clear"
    HOMING_GRIPPER = "homing_gripper"
    HOMING_X = "homing_x"
    PARKING_X_6000 = "parking_x_6000"
    HOMING_Y = "homing_y"
    HOMING_DOOR = "homing_door"
    SETTING_Y_HOME = "setting_y_home"
    POST_HOME_PIPETTE = "post_home_pipette"
    VISION_INSPECTION = "vision_inspection"
    PARKING_GANTRY = "parking_gantry"
    DOOR_READY = "door_ready"
    READY = "ready"
    DIAGNOSTIC_COMPLETE = "diagnostic_complete"
    FAILED_CLOSED = "failed_closed"
    ABORTED = "aborted"


STARTUP_STAGE_ORDER = [
    OemStartupState.CONFIG_LOADED.value,
    OemStartupState.BACKEND_READY.value,
    OemStartupState.CONTROL_LIB_CONSTRUCTED.value,
    OemStartupState.PIPETTE_CHECKED.value,
    OemStartupState.MOTORS_CONFIGURED_WITHOUT_MOTION.value,
    OemStartupState.INITIALIZE_ENVIRONMENT.value,
    OemStartupState.INITIAL_CHECK_BEFORE_DOOR.value,
    OemStartupState.DOOR_CLOSE_OBSERVED.value,
    OemStartupState.INITIAL_CHECK_AFTER_DOOR.value,
    OemStartupState.INITIALIZE_SYSTEM_QUEUED.value,
    OemStartupState.INITIALIZE_SYSTEM_RUNNING.value,
    OemStartupState.INITIALIZE_MOTION_RUNNING.value,
    OemStartupState.INITIALIZE_MOTORS_RUNNING.value,
    OemStartupState.HOMING_Z.value,
    OemStartupState.HOMING_GRIPPER_CLEAR.value,
    OemStartupState.HOMING_GRIPPER.value,
    OemStartupState.HOMING_X.value,
    OemStartupState.PARKING_X_6000.value,
    OemStartupState.HOMING_Y.value,
    OemStartupState.HOMING_DOOR.value,
    OemStartupState.SETTING_Y_HOME.value,
    OemStartupState.POST_HOME_PIPETTE.value,
    OemStartupState.VISION_INSPECTION.value,
    OemStartupState.PARKING_GANTRY.value,
    OemStartupState.DOOR_READY.value,
    OemStartupState.READY.value,
]


class OemStartupRequest(BaseModel):
    mode: Literal["dry_run", "shadow", "live"] = "dry_run"
    operator_ack: Optional[str] = None
    artifact_root: Optional[str] = None
    require_config: bool = True
    door_policy: Literal["wait_for_closed", "fail_if_open", "already_closed"] = "wait_for_closed"
    run_homing: bool = True
    run_post_home: bool = True
    timeout_s: float = Field(default=300.0, gt=0.1, le=900.0)


class OemDoorEventRequest(BaseModel):
    door_closed: bool = True
    latch_closed: bool = True


class OemInitialCheckRequest(BaseModel):
    mode: Literal["dry_run", "shadow", "live"] = "shadow"
    operator_ack: Optional[str] = None


class OemSwitchAuditRequest(BaseModel):
    axes: list[Literal["x", "y", "z", "g", "door"]] = Field(default_factory=lambda: ["x", "y", "z", "g", "door"])
    mode: Literal["status", "live_probe"] = "status"
    artifact_root: Optional[str] = None

    @field_validator("axes")
    @classmethod
    def axes_not_empty(cls, value):
        if not value:
            raise ValueError("at least one axis is required")
        return value
