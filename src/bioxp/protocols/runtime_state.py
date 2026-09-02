from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from .models import ProtocolDocument


class StageExecutionStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    FAILED = "failed"
    COMPLETED = "completed"


@dataclass
class ProtocolExecutionEvent:
    sequence: int
    event: str
    stage_id: str | None = None
    action_id: str | None = None
    detail: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ProtocolExecutionEvent":
        return cls(
            sequence=int(payload.get("sequence", 0)),
            event=str(payload.get("event", "unknown")),
            stage_id=payload.get("stage_id"),
            action_id=payload.get("action_id"),
            detail=dict(payload.get("detail") or {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "sequence": int(self.sequence),
            "event": self.event,
            "stage_id": self.stage_id,
            "action_id": self.action_id,
            "detail": dict(self.detail),
        }


@dataclass
class ProtocolStageState:
    stage_id: str
    title: str | None = None
    status: StageExecutionStatus = StageExecutionStatus.PENDING
    review_required: bool = False
    current_action_id: str | None = None
    completed_actions: list[str] = field(default_factory=list)
    pause_marker_action_id: str | None = None

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ProtocolStageState":
        return cls(
            stage_id=str(payload["stage_id"]),
            title=payload.get("title"),
            status=StageExecutionStatus(str(payload.get("status", StageExecutionStatus.PENDING.value))),
            review_required=bool(payload.get("review_required", False)),
            current_action_id=payload.get("current_action_id"),
            completed_actions=[str(value) for value in payload.get("completed_actions") or []],
            pause_marker_action_id=payload.get("pause_marker_action_id"),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "stage_id": self.stage_id,
            "title": self.title,
            "status": self.status.value,
            "review_required": bool(self.review_required),
            "current_action_id": self.current_action_id,
            "completed_actions": list(self.completed_actions),
            "pause_marker_action_id": self.pause_marker_action_id,
        }


@dataclass
class ProtocolRuntimeState:
    protocol_id: str
    dry_run: bool
    job_id: str | None = None
    current_stage_id: str | None = None
    paused: bool = False
    awaiting_review: bool = False
    completed: bool = False
    pause_reason: str | None = None
    stage_states: dict[str, ProtocolStageState] = field(default_factory=dict)
    events: list[ProtocolExecutionEvent] = field(default_factory=list)
    action_results: list[dict[str, Any]] = field(default_factory=list)

    @classmethod
    def from_document(
        cls,
        document: ProtocolDocument,
        *,
        dry_run: bool,
        job_id: str | None = None,
    ) -> "ProtocolRuntimeState":
        return cls(
            protocol_id=document.protocol_id,
            dry_run=bool(dry_run),
            job_id=job_id,
            stage_states={
                stage.stage_id: ProtocolStageState(
                    stage_id=stage.stage_id,
                    title=stage.title,
                    review_required=bool(stage.review_required),
                )
                for stage in document.stages
            },
        )

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ProtocolRuntimeState":
        return cls(
            protocol_id=str(payload["protocol_id"]),
            dry_run=bool(payload.get("dry_run", False)),
            job_id=payload.get("job_id"),
            current_stage_id=payload.get("current_stage_id"),
            paused=bool(payload.get("paused", False)),
            awaiting_review=bool(payload.get("awaiting_review", False)),
            completed=bool(payload.get("completed", False)),
            pause_reason=payload.get("pause_reason"),
            stage_states={
                str(stage_id): ProtocolStageState.from_payload(stage_payload)
                for stage_id, stage_payload in (payload.get("stage_states") or {}).items()
            },
            events=[
                ProtocolExecutionEvent.from_payload(event_payload)
                for event_payload in payload.get("events") or []
            ],
            action_results=[dict(entry) for entry in payload.get("action_results") or []],
        )

    def record_event(
        self,
        event: str,
        *,
        stage_id: str | None = None,
        action_id: str | None = None,
        detail: dict[str, Any] | None = None,
    ) -> None:
        self.events.append(
            ProtocolExecutionEvent(
                sequence=len(self.events) + 1,
                event=event,
                stage_id=stage_id,
                action_id=action_id,
                detail=dict(detail or {}),
            )
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "protocol_id": self.protocol_id,
            "dry_run": bool(self.dry_run),
            "job_id": self.job_id,
            "current_stage_id": self.current_stage_id,
            "paused": bool(self.paused),
            "awaiting_review": bool(self.awaiting_review),
            "completed": bool(self.completed),
            "pause_reason": self.pause_reason,
            "stage_states": {
                stage_id: state.to_payload()
                for stage_id, state in self.stage_states.items()
            },
            "action_results": [dict(entry) for entry in self.action_results],
            "events": [event.to_payload() for event in self.events],
        }
