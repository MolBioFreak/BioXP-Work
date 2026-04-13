from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from threading import Lock
from typing import Any, Iterable


class ReferenceState(str, Enum):
    UNKNOWN = "unknown"
    REFERENCED = "referenced"
    DESYNCED = "desynced"


@dataclass(frozen=True)
class AxisReferenceRecord:
    axis: str
    state: ReferenceState
    origin_position_steps: int | None = None
    source: str | None = None
    note: str | None = None
    updated_at: str | None = None
    last_motion_kind: str | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            "axis": self.axis,
            "state": self.state.value,
            "origin_position_steps": self.origin_position_steps,
            "source": self.source,
            "note": self.note,
            "updated_at": self.updated_at,
            "last_motion_kind": self.last_motion_kind,
        }


@dataclass(frozen=True)
class MarkAxisReferencedCommand:
    axis: Any
    position_steps: int = 0
    source: str = "manual"
    note: str | None = None
    motion_kind: str | None = None


@dataclass(frozen=True)
class MarkAxisDesyncedCommand:
    axis: Any
    reason: str
    source: str = "manual"
    motion_kind: str | None = None


class ReferenceStateStore:
    def __init__(self) -> None:
        self._rows: dict[str, AxisReferenceRecord] = {}
        self._lock = Lock()

    def reset(self) -> None:
        with self._lock:
            self._rows = {}

    def snapshot(self, axes: Iterable[Any]) -> dict[str, Any]:
        axis_keys = [_axis_value(axis) for axis in axes]
        with self._lock:
            rows = {
                axis: self._rows.get(axis, AxisReferenceRecord(axis=axis, state=ReferenceState.UNKNOWN)).to_payload()
                for axis in axis_keys
            }
        return {
            "axes": axis_keys,
            "rows": rows,
        }

    def mark_referenced(self, command: MarkAxisReferencedCommand) -> dict[str, Any]:
        axis = _axis_value(command.axis)
        record = AxisReferenceRecord(
            axis=axis,
            state=ReferenceState.REFERENCED,
            origin_position_steps=int(command.position_steps),
            source=_normalize_text(command.source, fallback="manual"),
            note=_normalize_optional_text(command.note),
            updated_at=_utc_now(),
            last_motion_kind=_normalize_optional_text(command.motion_kind),
        )
        with self._lock:
            self._rows[axis] = record
        return record.to_payload()

    def mark_desynced(self, command: MarkAxisDesyncedCommand) -> dict[str, Any]:
        axis = _axis_value(command.axis)
        with self._lock:
            previous = self._rows.get(axis)
            record = AxisReferenceRecord(
                axis=axis,
                state=ReferenceState.DESYNCED,
                origin_position_steps=None if previous is None else previous.origin_position_steps,
                source=_normalize_text(command.source, fallback="manual"),
                note=_normalize_text(command.reason, fallback="desynced"),
                updated_at=_utc_now(),
                last_motion_kind=_normalize_optional_text(command.motion_kind)
                or (None if previous is None else previous.last_motion_kind),
            )
            self._rows[axis] = record
        return record.to_payload()

    def record_motion(self, axis: Any, motion_kind: str) -> dict[str, Any]:
        axis_key = _axis_value(axis)
        normalized_kind = _normalize_text(motion_kind, fallback="motion")
        with self._lock:
            previous = self._rows.get(axis_key)
            if previous is None:
                record = AxisReferenceRecord(
                    axis=axis_key,
                    state=ReferenceState.UNKNOWN,
                    updated_at=_utc_now(),
                    last_motion_kind=normalized_kind,
                )
            else:
                record = AxisReferenceRecord(
                    axis=previous.axis,
                    state=previous.state,
                    origin_position_steps=previous.origin_position_steps,
                    source=previous.source,
                    note=previous.note,
                    updated_at=_utc_now(),
                    last_motion_kind=normalized_kind,
                )
            self._rows[axis_key] = record
        return record.to_payload()


def _axis_value(axis: Any) -> str:
    return str(getattr(axis, "value", axis))


def _normalize_optional_text(value: str | None) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _normalize_text(value: str | None, *, fallback: str) -> str:
    return _normalize_optional_text(value) or fallback


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()
