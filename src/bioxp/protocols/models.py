from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from types import MappingProxyType
from typing import Any, Mapping


def _capture(value: Any) -> Any:
    """Detach and freeze JSON-shaped plan data, never a live Python object."""
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise ValueError("Protocol data keys must be strings")
        return MappingProxyType({key: _capture(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_capture(item) for item in value)
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float and isfinite(value):
        return value
    raise ValueError("Protocol data must contain only finite JSON values")


def _payload(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _payload(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_payload(item) for item in value]
    return value


# ControlLib.scriptInterpretor's exact labels; default is not executable support.
OEM_OPERATION_FORMS = MappingProxyType({
    **dict.fromkeys((
        "aa", "catchPlate", "catch", "cc", "cutseal", "da", "delaypoint",
        "dopen", "dclose", "ejt", "iniPipette", "la", "ldtip", "led", "ms",
        "park", "pressp", "releasePlate", "release", "retip", "snapshot", "so",
        "sp", "splid", "step", "sweep", "wait",
    ), "raw"),
    "ampmix": "ClassAmpMix", "dsa": "ClassDispenseAll",
    "masp": "ClassAspirate", "mmix": "ClassMix", "mov": "ClassMoveTo",
    "rmb": "ClassDebubble",
})

# Only source fields are admitted; omitted and explicit null remain distinct.
OEM_TYPED_FIELDS = MappingProxyType({
    "ClassAmpMix": frozenset({"m_repeat"}),
    "ClassAspirate": frozenset({"m_volume", "m_air", "m_speed", "m_delay", "m_aspiratecushion", "m_aspirateheight", "m_overaspirate"}),
    "ClassDispenseAll": frozenset({"m_speed", "m_delay", "m_ntd", "m_purge", "m_purgespeed", "m_dispensehigh", "m_dispensecushion", "m_shakeoffcount", "m_dispenseheight"}),
    "ClassMix": frozenset({"m_aspirateOptions", "m_dispenseAllOptions", "m_repeat", "m_tipDip", "m_mixType"}),
    "ClassMoveTo": frozenset({"m_destination", "m_well", "m_piersOption", "m_oldWell", "m_material"}),
    "ClassDebubble": frozenset({"m_aspirateOptions", "m_dispenseOptions", "m_orbit", "m_repeat", "m_tipDip"}),
})
# Minimum token membership used by the source, not scientific value/range policy.
OEM_RAW_MIN_ARGUMENTS = MappingProxyType({
    "aa": 1, "catchPlate": 1, "catch": 1, "cc": 2, "da": 1,
    "ldtip": 1, "led": 3, "ms": 2, "releasePlate": 1, "release": 1,
    "so": 1, "sp": 3, "splid": 3, "wait": 1,
})

from ..domain.capabilities import CapabilityName


class ProtocolActionKind(str, Enum):
    OEM_OPERATION = "oem_operation"
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
    oem_opcode: str | None = None
    source_occurrence_id: str | None = None
    source_key: str | int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", normalize_action_kind(self.kind))
        object.__setattr__(self, "params", _capture(self.params))
        object.__setattr__(self, "metadata", _capture(self.metadata))

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
            oem_opcode=payload.get("oem_opcode"),
            source_occurrence_id=payload.get("source_occurrence_id"),
            source_key=payload.get("source_key"),
            metadata=dict(payload.get("metadata") or {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "action_id": self.action_id,
            "stage_id": self.stage_id,
            "kind": self.kind.value,
            "params": _payload(self.params),
            "description": self.description,
            "required_capability": None if self.required_capability is None else self.required_capability.value,
            "review_required": bool(self.review_required),
            "pause_message": self.pause_message,
            "oem_opcode": self.oem_opcode,
            "source_occurrence_id": self.source_occurrence_id,
            "source_key": self.source_key,
            "metadata": _payload(self.metadata),
        }


@dataclass(frozen=True)
class ProtocolStage:
    stage_id: str
    title: str | None = None
    actions: tuple[ProtocolAction, ...] = ()
    review_required: bool = False
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "actions", tuple(self.actions))
        object.__setattr__(self, "metadata", _capture(self.metadata))

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
            "metadata": _payload(self.metadata),
        }


@dataclass(frozen=True)
class ProtocolDocument:
    protocol_id: str
    version: int = 1
    stages: tuple[ProtocolStage, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "stages", tuple(self.stages))
        object.__setattr__(self, "metadata", _capture(self.metadata))

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "ProtocolDocument":
        from .validators import validate_protocol_document

        document = cls(
            protocol_id=str(payload["protocol_id"]),
            version=int(payload.get("version", 1)),
            stages=tuple(ProtocolStage.from_payload(stage) for stage in payload.get("stages") or ()),
            metadata=dict(payload.get("metadata") or {}),
        )
        return validate_protocol_document(document)

    def to_payload(self) -> dict[str, Any]:
        return {
            "protocol_id": self.protocol_id,
            "version": int(self.version),
            "stages": [stage.to_payload() for stage in self.stages],
            "metadata": _payload(self.metadata),
        }
