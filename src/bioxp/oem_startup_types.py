from __future__ import annotations

from enum import Enum
from typing import Optional
from pydantic import BaseModel, Field


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
    READY = "ready"
    FAILED_CLOSED = "failed_closed"
    ABORTED = "aborted"


class OemStartupRequest(BaseModel):
    mode: str = Field("dry_run", pattern=r"^(dry_run|shadow|live)$")
    operator_ack: Optional[str] = None
    artifact_root: Optional[str] = None
    require_config: bool = True
    door_policy: str = Field("wait_for_closed", pattern=r"^(wait_for_closed|fail_if_open|already_closed)$")
    run_homing: bool = True
    run_post_home: bool = True
    timeout_s: float = 300.0


class OemDoorEventRequest(BaseModel):
    session_id: Optional[str] = None
    door_closed: bool = True
    latch_closed: bool = True


class OemSwitchAuditRequest(BaseModel):
    axes: list[str] = Field(default_factory=lambda: ["x", "y", "z", "g", "door"])
    mode: str = Field("status", pattern=r"^(status|live_probe)$")
    artifact_root: Optional[str] = None
