from __future__ import annotations

import fcntl
import json
import logging
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from threading import Lock
from typing import Any, Iterable


logger = logging.getLogger(__name__)


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
    def __init__(self, state_path: str | Path | None = None) -> None:
        self._state_path = None if state_path is None else Path(state_path)
        self._state_lock_path = None if self._state_path is None else self._state_path.with_suffix(self._state_path.suffix + ".lock")
        self._rows: dict[str, AxisReferenceRecord] = {}
        self._disk_state_dirty = False
        self._lock = Lock()
        self._load_from_disk()

    def reset(self) -> bool:
        with self._lock:
            with self._state_file_lock(fcntl.LOCK_EX):
                self._rows = {}
                return self._persist_locked()

    def snapshot(self, axes: Iterable[Any]) -> dict[str, Any]:
        axis_keys = [_axis_value(axis) for axis in axes]
        with self._lock:
            with self._state_file_lock(fcntl.LOCK_SH):
                if not self._disk_state_dirty:
                    self._load_from_disk_locked()
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
            with self._state_file_lock(fcntl.LOCK_EX):
                if self._disk_state_dirty:
                    self._reconcile_dirty_rows_locked()
                else:
                    self._load_from_disk_locked()
                self._rows[axis] = record
                persisted = self._persist_locked()
        payload = record.to_payload()
        payload["persisted"] = persisted
        return payload

    def mark_desynced(self, command: MarkAxisDesyncedCommand) -> dict[str, Any]:
        axis = _axis_value(command.axis)
        with self._lock:
            with self._state_file_lock(fcntl.LOCK_EX):
                if self._disk_state_dirty:
                    self._reconcile_dirty_rows_locked()
                else:
                    self._load_from_disk_locked()
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
                persisted = self._persist_locked()
        payload = record.to_payload()
        payload["persisted"] = persisted
        return payload

    def record_motion(self, axis: Any, motion_kind: str) -> dict[str, Any]:
        axis_key = _axis_value(axis)
        normalized_kind = _normalize_text(motion_kind, fallback="motion")
        with self._lock:
            with self._state_file_lock(fcntl.LOCK_EX):
                if self._disk_state_dirty:
                    self._reconcile_dirty_rows_locked()
                else:
                    self._load_from_disk_locked()
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
                persisted = self._persist_locked()
        payload = record.to_payload()
        payload["persisted"] = persisted
        return payload

    def _load_from_disk(self) -> None:
        with self._state_file_lock(fcntl.LOCK_SH):
            self._load_from_disk_locked()

    def _load_from_disk_locked(self) -> None:
        loaded = self._read_rows_from_disk_locked()
        if loaded is None:
            return
        self._rows = loaded
        self._disk_state_dirty = False

    def _read_rows_from_disk_locked(self) -> dict[str, AxisReferenceRecord] | None:
        if self._state_path is None:
            return dict(self._rows)
        if not self._state_path.exists():
            return {}
        try:
            payload = json.loads(self._state_path.read_text())
        except Exception:
            logger.warning("Failed to load reference state from %s", self._state_path, exc_info=True)
            return None
        rows = payload.get("rows", {}) if isinstance(payload, dict) else {}
        if not isinstance(rows, dict):
            return {}
        loaded: dict[str, AxisReferenceRecord] = {}
        for axis, row in rows.items():
            if not isinstance(row, dict):
                continue
            state = _normalize_reference_state(row.get("state"))
            if state is None:
                continue
            loaded[str(axis)] = AxisReferenceRecord(
                axis=str(row.get("axis", axis)),
                state=state,
                origin_position_steps=_normalize_optional_int(row.get("origin_position_steps")),
                source=_normalize_optional_text(row.get("source")),
                note=_normalize_optional_text(row.get("note")),
                updated_at=_normalize_optional_text(row.get("updated_at")),
                last_motion_kind=_normalize_optional_text(row.get("last_motion_kind")),
            )
        return loaded

    def _reconcile_dirty_rows_locked(self) -> None:
        loaded = self._read_rows_from_disk_locked()
        if loaded is None:
            return
        for axis, local_record in self._rows.items():
            disk_record = loaded.get(axis)
            if _record_is_newer_or_equal(local_record, disk_record):
                loaded[axis] = local_record
        self._rows = loaded

    def _persist_locked(self) -> bool:
        if self._state_path is None:
            self._disk_state_dirty = False
            return True
        try:
            self._state_path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                "version": 1,
                "rows": {axis: record.to_payload() for axis, record in self._rows.items()},
            }
            tmp_path = self._state_path.with_suffix(self._state_path.suffix + ".tmp")
            tmp_path.write_text(json.dumps(payload, sort_keys=True, indent=2))
            tmp_path.replace(self._state_path)
            self._disk_state_dirty = False
            return True
        except Exception:
            self._disk_state_dirty = True
            logger.warning("Failed to persist reference state to %s", self._state_path, exc_info=True)
            return False

    @contextmanager
    def _state_file_lock(self, mode: int):
        if self._state_lock_path is None:
            yield
            return
        try:
            self._state_lock_path.parent.mkdir(parents=True, exist_ok=True)
            lock_file = self._state_lock_path.open("a+")
        except Exception:
            logger.warning("Failed to lock reference state file %s", self._state_lock_path, exc_info=True)
            yield
            return
        try:
            try:
                fcntl.flock(lock_file.fileno(), mode)
            except Exception:
                logger.warning("Failed to lock reference state file %s", self._state_lock_path, exc_info=True)
                yield
                return
            try:
                yield
            finally:
                try:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
                except Exception:
                    logger.warning("Failed to unlock reference state file %s", self._state_lock_path, exc_info=True)
        finally:
            lock_file.close()


def _axis_value(axis: Any) -> str:
    return str(getattr(axis, "value", axis))


def _normalize_optional_text(value: str | None) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _normalize_text(value: str | None, *, fallback: str) -> str:
    return _normalize_optional_text(value) or fallback


def _normalize_optional_int(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _normalize_reference_state(value: Any) -> ReferenceState | None:
    if value is None:
        return ReferenceState.UNKNOWN
    try:
        return ReferenceState(str(value).strip())
    except Exception:
        return None


def _record_is_newer_or_equal(local_record: AxisReferenceRecord, disk_record: AxisReferenceRecord | None) -> bool:
    if disk_record is None:
        return True
    local_updated_at = _normalize_optional_text(local_record.updated_at) or ""
    disk_updated_at = _normalize_optional_text(disk_record.updated_at) or ""
    return local_updated_at >= disk_updated_at


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()
