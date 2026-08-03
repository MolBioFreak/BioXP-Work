from __future__ import annotations

from enum import Enum
from typing import Literal, Optional
from pydantic import BaseModel, Field, field_validator


class OemStartupState(str, Enum):
    CREATED = "created"
    FAILED_CLOSED = "failed_closed"


class OemStartupRequest(BaseModel):
    mode: Literal["dry_run", "shadow", "live"] = "dry_run"
    operator_ack: Optional[str] = None
    artifact_root: Optional[str] = None
    require_config: bool = True
    door_policy: Literal["wait_for_closed", "fail_if_open", "already_closed"] = "wait_for_closed"
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
