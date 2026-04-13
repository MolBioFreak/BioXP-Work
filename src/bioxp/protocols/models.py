from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from ..domain.capabilities import CapabilityName


class ProtocolActionKind(str, Enum):
    MOVE = "move"
    HOME = "home"
    PIPETTE_INIT = "pipette_init"
    PIPETTE_TIP = "pipette_tip"
    PIPETTE_ASPIRATE = "pipette_aspirate"
    PIPETTE_DISPENSE = "pipette_dispense"
    PIPETTE_MIX = "pipette_mix"
    INSPECT = "inspect"
    BARCODE_READ = "barcode_read"
    PAUSE_REVIEW = "pause_review"
    NOTE = "note"
    LED = "led"
    WAIT = "wait"
    PLATE_PREPARE = "plate_prepare"
    PLATE_MOVE = "plate_move"
    THERMAL_DOOR = "thermal_door"
    MOVE_COVER = "move_cover"
    CHILLER_SETPOINT = "chiller_setpoint"
    THERMAL_SETPOINT = "thermal_setpoint"
    LOOP_MARKER = "loop_marker"
    SEAL_SEPARATE = "seal_separate"
    LIQUID_ADJUST = "liquid_adjust"
    TIP_EJECT = "tip_eject"


def normalize_action_kind(value: ProtocolActionKind | str) -> ProtocolActionKind:
    if isinstance(value, ProtocolActionKind):
        return value
    normalized = str(value).strip().lower().replace("-", "_").replace(" ", "_")
    return ProtocolActionKind(normalized)


@dataclass(frozen=True)
class ProtocolAction:
    action_id: str
    stage_id: str
    kind: ProtocolActionKind
    params: Mapping[str, Any] = field(default_factory=dict)
    description: str | None = None
    required_capability: CapabilityName | None = None
    review_required: bool = False
    pause_message: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "ProtocolAction":
        raw_capability = payload.get("required_capability")
        capability = CapabilityName(str(raw_capability)) if raw_capability else None
        return cls(
            action_id=str(payload["action_id"]),
            stage_id=str(payload["stage_id"]),
            kind=normalize_action_kind(payload["kind"]),
            params=dict(payload.get("params") or {}),
            description=payload.get("description"),
            required_capability=capability,
            review_required=bool(payload.get("review_required", False)),
            pause_message=payload.get("pause_message"),
            metadata=dict(payload.get("metadata") or {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "action_id": self.action_id,
            "stage_id": self.stage_id,
            "kind": self.kind.value,
            "params": dict(self.params),
            "description": self.description,
            "required_capability": None if self.required_capability is None else self.required_capability.value,
            "review_required": bool(self.review_required),
            "pause_message": self.pause_message,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class ProtocolStage:
    stage_id: str
    title: str | None = None
    actions: tuple[ProtocolAction, ...] = ()
    review_required: bool = False
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "ProtocolStage":
        return cls(
            stage_id=str(payload["stage_id"]),
            title=payload.get("title"),
            actions=tuple(ProtocolAction.from_payload(action) for action in payload.get("actions") or ()),
            review_required=bool(payload.get("review_required", False)),
            metadata=dict(payload.get("metadata") or {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "stage_id": self.stage_id,
            "title": self.title,
            "review_required": bool(self.review_required),
            "actions": [action.to_payload() for action in self.actions],
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class ProtocolDocument:
    protocol_id: str
    version: int = 1
    stages: tuple[ProtocolStage, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "ProtocolDocument":
        return cls(
            protocol_id=str(payload["protocol_id"]),
            version=int(payload.get("version", 1)),
            stages=tuple(ProtocolStage.from_payload(stage) for stage in payload.get("stages") or ()),
            metadata=dict(payload.get("metadata") or {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "protocol_id": self.protocol_id,
            "version": int(self.version),
            "stages": [stage.to_payload() for stage in self.stages],
            "metadata": dict(self.metadata),
        }
