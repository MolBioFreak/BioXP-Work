from __future__ import annotations

import time
import uuid
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class OEMCommandName(str, Enum):
    INITIALIZE_SYSTEM = "initializeSystem"
    UNLOCK_PROCESS = "unlockProcess"
    PREPARE_TO_RUN_JOB = "PrepareToRunJob"
    ABORT_JOB = "abortjob"
    VALIDATE_JOB = "validateJob"
    WAKE_FROM_PAUSE = "wakefrompause"

    @classmethod
    def validate(cls, value: str) -> "OEMCommandName":
        try:
            return cls(value)
        except ValueError as exc:
            allowed = ",".join(item.value for item in cls)
            raise ValueError(f"unknown OEM runtime command {value!r}; allowed={allowed}") from exc


class OEMRuntimeMode(str, Enum):
    DRY_RUN = "dry_run"
    SHADOW = "shadow"
    LIVE = "live"


class OEMRuntimeStateName(str, Enum):
    BOOTING = "booting"
    BACKEND_UNAVAILABLE = "backend_unavailable"
    IDLE_NOT_READY = "idle_not_ready"
    WAITING_FOR_DOOR_CLOSE = "waiting_for_door_close"
    INITIALIZING = "initializing"
    READY_FOR_JOB = "ready_for_job"
    USER_PAUSED = "user_paused"
    RESUMING_FROM_PAUSE = "resuming_from_pause"
    ABORTING_JOB = "aborting_job"
    ERROR_OPERATOR_ACTION_REQUIRED = "error_operator_action_required"
    EMERGENCY_STOPPED = "emergency_stopped"
    RECOVERY_REQUIRED = "recovery_required"
    SHUTDOWN = "shutdown"


class OEMWorkerStateName(str, Enum):
    NOT_STARTED = "not_started"
    IDLE = "idle"
    QUEUED = "queued"
    RUNNING = "running"
    TIMEOUT = "timeout"
    ABORTING = "aborting"
    FAILED = "failed"
    RECOVERING = "recovering"
    STOPPED = "stopped"


class OEMErrorSituation(str, Enum):
    NONE = "none"
    BOARD_ERROR = "board_error"
    MOTION_INITIALIZATION_ERROR = "motion_initialization_error"
    DOOR_MALFUNCTION = "door_malfunction"
    INITIALIZATION_FAILURE = "initialization_failure"
    JOB_LOAD_ERROR = "job_load_error"
    SERVER_COMMUNICATION_ERROR = "server_communication_error"
    PIPETTE_ERROR = "pipette_error"
    CAMERA_INITIALIZATION_FAILURE = "camera_initialization_failure"
    COVER_INSPECTION_FAILURE = "cover_inspection_failure"
    EMERGENCY_STOP = "emergency_stop"
    COMMAND_TIMEOUT = "command_timeout"
    TRANSPORT_LOSS = "usb_can_transport_loss"


def utc_ts() -> float:
    return time.time()


def new_id(prefix: str) -> str:
    return f"{prefix}_{int(time.time()*1000)}_{uuid.uuid4().hex[:10]}"


@dataclass
class OEMRuntimeCommand:
    name: str
    mode: str = OEMRuntimeMode.DRY_RUN.value
    source: str = "api"
    params: dict[str, Any] = field(default_factory=dict)
    command_id: str = field(default_factory=lambda: new_id("cmd"))
    session_id: str | None = None
    operator_ack: str | None = None
    artifact_root: str | None = None
    created_at: float = field(default_factory=utc_ts)
    timeout_s: float = 30.0

    def __post_init__(self) -> None:
        self.name = OEMCommandName.validate(self.name).value
        if self.mode not in {m.value for m in OEMRuntimeMode}:
            raise ValueError(f"invalid OEM runtime mode {self.mode!r}")
        if self.mode == OEMRuntimeMode.LIVE.value:
            if not self.operator_ack:
                raise ValueError("operator_ack required for live OEM runtime command")
            if not self.artifact_root:
                raise ValueError("artifact_root required for live OEM runtime command")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class OEMRuntimeEvent:
    event_type: str
    source: str = "api"
    payload: dict[str, Any] = field(default_factory=dict)
    event_id: str = field(default_factory=lambda: new_id("evt"))
    created_at: float = field(default_factory=utc_ts)
    state_before: dict[str, Any] | None = None
    state_after: dict[str, Any] | None = None
    actions_taken: list[str] = field(default_factory=list)
    artifact_path: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class OEMMachineStatus:
    enclosure_door_closed: bool | None = None
    latch_closed: bool | None = None
    thermal_door_open: bool | None = None
    tip_loaded: bool | None = None
    tip_dirty: bool | None = None
    location: str | None = None
    running_job: bool = False
    user_paused: bool = False
    system_in_motion: bool = False
    abort_job: bool = False
    stop_scripts: bool = False
    pause_scripts: bool = False
    force_abort: bool = False
    latest_status: str = "unknown"
    saved_status: str = "unknown"
    door_close_retry: int = 0
    start_mode: str | None = None
    ship_mode: str | None = None
    self_test_due: bool = False
    camera_required: bool = False
    camera_ready: bool = False
    cover_inspection_status: str = "unproven"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class OEMWorkerSnapshot:
    state: str = OEMWorkerStateName.NOT_STARTED.value
    gantry_available: bool = True
    queue_depth: int = 0
    active_command: dict[str, Any] | None = None
    last_heartbeat_at: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class OEMRuntimeSnapshot:
    runtime_state: str = OEMRuntimeStateName.IDLE_NOT_READY.value
    machine_status: OEMMachineStatus = field(default_factory=OEMMachineStatus)
    worker: OEMWorkerSnapshot = field(default_factory=OEMWorkerSnapshot)
    hardware: dict[str, Any] = field(default_factory=lambda: {"backend_ready": None, "can_ready": None, "usb_connected": None, "board_status": None, "truth_state": "unknown"})
    operator: dict[str, Any] = field(default_factory=lambda: {"action_required": False, "message_id": None, "error_situation": OEMErrorSituation.NONE.value})
    confidence: dict[str, str] = field(default_factory=lambda: {"door_latch": "unknown", "motion_reference": "unknown", "pipette": "unproven", "vision": "unproven"})
    updated_at: float = field(default_factory=utc_ts)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        from .lifecycle_state import lifecycle_state

        lifecycle = lifecycle_state.projection()
        payload["runtime_state_legacy"] = payload.pop("runtime_state")
        payload["runtime_state"] = lifecycle["operation_state"]
        payload["operation_state"] = lifecycle["operation_state"]
        payload["startup"] = lifecycle["startup"]
        payload["lifecycle"] = lifecycle
        payload["machine_status"]["enclosure_door_closed"] = lifecycle["door"]["door_closed"]
        payload["machine_status"]["latch_closed"] = lifecycle["door"]["latch_closed"]
        payload["machine_status"]["user_paused"] = lifecycle["operation_state"] == "paused"
        payload["machine_status"]["running_job"] = lifecycle["operation_state"] == "running"
        payload["machine_status"]["latest_status"] = lifecycle["operation_state"]
        payload["ok"] = lifecycle["operation_state"] not in {"error", "emergency"}
        return payload
