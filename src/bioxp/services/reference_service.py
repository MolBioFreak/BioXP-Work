from __future__ import annotations

import fcntl
import hashlib
import json
import logging
import sqlite3
import time
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from threading import Lock
from typing import Any, Callable, Iterable


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


class ReferenceStateAuthorityError(RuntimeError):
    """Reference-state storage is unavailable or cannot be verified."""


class ReferenceStateStore:
    def __init__(self, state_path: str | Path | None = None) -> None:
        self._state_path = None if state_path is None else Path(state_path)
        self._database_path = (
            None
            if self._state_path is None
            else self._state_path
            if self._state_path.suffix == ".db"
            else self._state_path.parent / "bioxp_runtime.db"
        )
        self._state_lock_path = None if self._database_path is None else self._database_path.with_suffix(".db.lock")
        self._rows: dict[str, AxisReferenceRecord] = {}
        self._disk_state_dirty = False
        self._authority_untrusted = self._state_path is None
        self._lock = Lock()
        self._authority_write_depth = 0
        self._load_from_disk()

    def _open_database(self) -> sqlite3.Connection:
        if self._database_path is None:
            raise ReferenceStateAuthorityError("durable reference database path required")
        connection = sqlite3.connect(self._database_path, timeout=2.0)
        connection.create_function(
            "reference_write_allowed", 0, lambda: 1 if self._authority_write_depth > 0 else 0
        )
        connection.create_function(
            "sha256_utf8", 1, lambda value: hashlib.sha256(str(value).encode("utf-8")).hexdigest(), deterministic=True
        )
        connection.create_function(
            "canonical_json", 1, lambda value: json.dumps(json.loads(str(value)), sort_keys=True, separators=(",", ":")), deterministic=True
        )
        connection.execute("PRAGMA busy_timeout=2000")
        connection.executescript(
            """
            CREATE TABLE IF NOT EXISTS reference_state_authority (
                authority_key TEXT PRIMARY KEY CHECK(authority_key='reference_state'),
                payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
                payload_sha256 TEXT NOT NULL CHECK(length(payload_sha256)=64),
                updated_at REAL NOT NULL
            ) WITHOUT ROWID;
            DROP TRIGGER IF EXISTS reference_state_authority_coherence_v1;
            DROP TRIGGER IF EXISTS reference_state_authority_authorized_insert_v1;
            DROP TRIGGER IF EXISTS reference_state_authority_authorized_update_v1;
            DROP TRIGGER IF EXISTS reference_state_authority_no_delete_v1;
            CREATE TRIGGER reference_state_authority_coherence_v1
            BEFORE INSERT ON reference_state_authority
            WHEN NEW.payload_json<>canonical_json(NEW.payload_json)
              OR NEW.payload_sha256<>sha256_utf8(NEW.payload_json)
            BEGIN SELECT RAISE(ABORT,'reference authority bytes are incoherent'); END;
            CREATE TRIGGER reference_state_authority_authorized_insert_v1
            BEFORE INSERT ON reference_state_authority
            WHEN reference_write_allowed()<>1
            BEGIN SELECT RAISE(ABORT,'reference authority writer is not authoritative'); END;
            CREATE TRIGGER reference_state_authority_authorized_update_v1
            BEFORE UPDATE ON reference_state_authority
            WHEN reference_write_allowed()<>1
              OR NEW.authority_key IS NOT OLD.authority_key
              OR NEW.payload_json<>canonical_json(NEW.payload_json)
              OR NEW.payload_sha256<>sha256_utf8(NEW.payload_json)
            BEGIN SELECT RAISE(ABORT,'reference authority update is not authoritative'); END;
            CREATE TRIGGER reference_state_authority_no_delete_v1
            BEFORE DELETE ON reference_state_authority
            BEGIN SELECT RAISE(ABORT,'reference authority cannot be deleted'); END;
            """
        )
        expected_columns = (
            ("authority_key", "TEXT", 1, 1),
            ("payload_json", "TEXT", 1, 0),
            ("payload_sha256", "TEXT", 1, 0),
            ("updated_at", "REAL", 1, 0),
        )
        actual_columns = tuple(
            (str(row[1]), str(row[2]).upper(), int(row[3]), int(row[5]))
            for row in connection.execute("PRAGMA table_info(reference_state_authority)").fetchall()
        )
        table_row = connection.execute(
            "SELECT sql FROM sqlite_master WHERE type='table' AND name='reference_state_authority'"
        ).fetchone()
        table_sql = "".join(str(table_row[0] if table_row else "").upper().split())
        if actual_columns != expected_columns or "CHECK(AUTHORITY_KEY='REFERENCE_STATE')" not in table_sql or "WITHOUTROWID" not in table_sql:
            connection.close()
            raise ReferenceStateAuthorityError("reference authority table shape is not exact")
        expected = {
            "reference_state_authority_coherence_v1",
            "reference_state_authority_authorized_insert_v1",
            "reference_state_authority_authorized_update_v1",
            "reference_state_authority_no_delete_v1",
        }
        actual = {
            str(row[0])
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type='trigger' AND tbl_name='reference_state_authority'"
            ).fetchall()
        }
        if actual != expected:
            connection.close()
            raise ReferenceStateAuthorityError("reference authority trigger set is not exact")
        return connection

    @staticmethod
    def _unknown_row(axis: str) -> dict[str, Any]:
        return AxisReferenceRecord(axis=axis, state=ReferenceState.UNKNOWN).to_payload()

    def _failure(self, axes: Iterable[Any], error: str, *, proposed: Iterable[AxisReferenceRecord] | None = None) -> dict[str, Any]:
        axis_keys = [_axis_value(axis) for axis in axes]
        proposed_rows = [record.to_payload() for record in (proposed or [])]
        if not proposed_rows:
            proposed_rows = [self._unknown_row(axis) for axis in axis_keys]
        return {
            "ok": False,
            "persisted": False,
            "verified": False,
            "durable_clean": False,
            "error": str(error),
            "axes": axis_keys,
            "rows": proposed_rows,
        }

    def _clean_commit_result(self) -> dict[str, Any]:
        return {"ok": True, "persisted": True, "verified": True, "durable_clean": True}

    def reset(self) -> dict[str, Any]:
        with self._lock:
            if self._state_path is None:
                return self._failure([], "durable_reference_path_required")
            if self._disk_state_dirty or self._authority_untrusted:
                return self._failure([], "reference_state_authority_untrusted")
            try:
                with self._state_file_lock(fcntl.LOCK_EX):
                    loaded = self._read_rows_from_disk_locked()
                    if loaded is None:
                        self._authority_untrusted = True
                        return self._failure([], "reference_state_disk_unreadable")
                    previous = dict(self._rows)
                    self._rows = dict(loaded)
                    return self._commit_candidate_locked({}, previous)
            except ReferenceStateAuthorityError as exc:
                self._authority_untrusted = True
                return self._failure([], str(exc))

    def snapshot(self, axes: Iterable[Any]) -> dict[str, Any]:
        axis_keys = [_axis_value(axis) for axis in axes]
        with self._lock:
            if self._state_path is None:
                return {
                    **self._failure(axis_keys, "durable_reference_path_required"),
                    "axes": axis_keys,
                    "rows": {axis: self._unknown_row(axis) for axis in axis_keys},
                }
            if self._disk_state_dirty or self._authority_untrusted:
                return {
                    **self._failure(axis_keys, "reference_state_authority_untrusted"),
                    "axes": axis_keys,
                    "rows": {axis: self._unknown_row(axis) for axis in axis_keys},
                }
            try:
                with self._state_file_lock(fcntl.LOCK_SH):
                    loaded = self._read_rows_from_disk_locked()
                    if loaded is None:
                        self._authority_untrusted = True
                        return {
                            **self._failure(axis_keys, "reference_state_disk_unreadable"),
                            "axes": axis_keys,
                            "rows": {axis: self._unknown_row(axis) for axis in axis_keys},
                        }
                    self._rows = loaded
                    self._disk_state_dirty = False
                    self._authority_untrusted = False
                    rows = {
                        axis: self._rows.get(axis, AxisReferenceRecord(axis=axis, state=ReferenceState.UNKNOWN)).to_payload()
                        for axis in axis_keys
                    }
            except ReferenceStateAuthorityError as exc:
                self._authority_untrusted = True
                return {
                    **self._failure(axis_keys, str(exc)),
                    "axes": axis_keys,
                    "rows": {axis: self._unknown_row(axis) for axis in axis_keys},
                }
        return {
            "ok": True,
            "persisted": True,
            "verified": True,
            "durable_clean": True,
            "axes": axis_keys,
            "rows": rows,
        }

    def mark_referenced(self, command: MarkAxisReferencedCommand) -> dict[str, Any]:
        result = self.mark_referenced_many([command])
        return self._single_row_result(result, command.axis)

    def mark_referenced_many(self, commands: Iterable[MarkAxisReferencedCommand]) -> dict[str, Any]:
        commands = list(commands)

        def apply(candidate: dict[str, AxisReferenceRecord]) -> list[AxisReferenceRecord]:
            proposed: list[AxisReferenceRecord] = []
            for command in commands:
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
                candidate[axis] = record
                proposed.append(record)
            return proposed

        return self._run_mutation([command.axis for command in commands], apply)

    def mark_desynced(self, command: MarkAxisDesyncedCommand) -> dict[str, Any]:
        result = self.mark_desynced_many([command])
        return self._single_row_result(result, command.axis)

    def mark_desynced_many(self, commands: Iterable[MarkAxisDesyncedCommand]) -> dict[str, Any]:
        commands = list(commands)

        def apply(candidate: dict[str, AxisReferenceRecord]) -> list[AxisReferenceRecord]:
            proposed: list[AxisReferenceRecord] = []
            for command in commands:
                axis = _axis_value(command.axis)
                previous = candidate.get(axis)
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
                candidate[axis] = record
                proposed.append(record)
            return proposed

        return self._run_mutation([command.axis for command in commands], apply)

    def record_motion(self, axis: Any, motion_kind: str) -> dict[str, Any]:
        result = self.record_motion_many([(axis, motion_kind)])
        return self._single_row_result(result, axis)

    def record_motion_many(self, motions: Iterable[tuple[Any, str]]) -> dict[str, Any]:
        motions = list(motions)

        def apply(candidate: dict[str, AxisReferenceRecord]) -> list[AxisReferenceRecord]:
            proposed: list[AxisReferenceRecord] = []
            for axis, motion_kind in motions:
                axis_key = _axis_value(axis)
                normalized_kind = _normalize_text(motion_kind, fallback="motion")
                previous = candidate.get(axis_key)
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
                candidate[axis_key] = record
                proposed.append(record)
            return proposed

        return self._run_mutation([axis for axis, _ in motions], apply)

    def recover_untrusted_authority(self, reason: str) -> dict[str, Any]:
        """Convert all persisted references to desynced before restoring trust."""
        with self._lock:
            if self._state_path is None:
                return self._failure([], "durable_reference_path_required")
            try:
                with self._state_file_lock(fcntl.LOCK_EX):
                    loaded = self._read_rows_from_disk_locked()
                    if loaded is None:
                        self._authority_untrusted = True
                        return self._failure([], "reference_state_disk_unreadable")
                    previous = dict(self._rows)
                    self._rows = dict(loaded)
                    candidate = dict(loaded)
                    proposed: list[AxisReferenceRecord] = []
                    for axis, record in loaded.items():
                        if record.state is not ReferenceState.REFERENCED:
                            continue
                        replacement = AxisReferenceRecord(
                            axis=record.axis,
                            state=ReferenceState.DESYNCED,
                            origin_position_steps=record.origin_position_steps,
                            source="reference_recovery",
                            note=_normalize_text(reason, fallback="reference authority recovery"),
                            updated_at=_utc_now(),
                            last_motion_kind=record.last_motion_kind,
                        )
                        candidate[axis] = replacement
                        proposed.append(replacement)
                    result = self._commit_candidate_locked(candidate, previous)
                    result.update({"axes": list(loaded), "rows": [row.to_payload() for row in proposed]})
                    return result
            except ReferenceStateAuthorityError as exc:
                self._authority_untrusted = True
                return self._failure([], str(exc))

    @staticmethod
    def _single_row_result(result: dict[str, Any], axis: Any) -> dict[str, Any]:
        axis_key = _axis_value(axis)
        rows = result.get("rows") if isinstance(result, dict) else None
        row = next((dict(item) for item in rows or [] if item.get("axis") == axis_key), None)
        if row is None:
            row = ReferenceStateStore._unknown_row(axis_key)
        for key in ("ok", "persisted", "verified", "durable_clean", "error"):
            if key in result:
                row[key] = result[key]
        return row

    def _run_mutation(
        self,
        axes: Iterable[Any],
        mutator: Callable[[dict[str, AxisReferenceRecord]], list[AxisReferenceRecord]],
    ) -> dict[str, Any]:
        axis_keys = [_axis_value(axis) for axis in axes]
        with self._lock:
            if self._state_path is None:
                return self._failure(axis_keys, "durable_reference_path_required")
            if self._disk_state_dirty or self._authority_untrusted:
                return self._failure(axis_keys, "reference_state_authority_untrusted")
            try:
                with self._state_file_lock(fcntl.LOCK_EX):
                    loaded = self._read_rows_from_disk_locked()
                    if loaded is None:
                        self._authority_untrusted = True
                        return self._failure(axis_keys, "reference_state_disk_unreadable")
                    previous = dict(self._rows)
                    self._rows = dict(loaded)
                    candidate = dict(loaded)
                    proposed = mutator(candidate)
                    result = self._commit_candidate_locked(candidate, previous)
                    result.update({"axes": axis_keys, "rows": [row.to_payload() for row in proposed]})
                    return result
            except ReferenceStateAuthorityError as exc:
                self._authority_untrusted = True
                return self._failure(axis_keys, str(exc))
            except Exception as exc:
                self._authority_untrusted = True
                self._disk_state_dirty = True
                return self._failure(axis_keys, f"reference_state_mutation_failed:{type(exc).__name__}:{exc}")

    def _commit_candidate_locked(
        self,
        candidate: dict[str, AxisReferenceRecord],
        previous: dict[str, AxisReferenceRecord],
    ) -> dict[str, Any]:
        self._rows = dict(candidate)
        if not self._persist_locked():
            self._rows = dict(previous)
            self._disk_state_dirty = True
            self._authority_untrusted = True
            return self._failure([], "reference_state_persist_failed")
        verified = self._read_rows_from_disk_locked()
        if verified is None or not self._records_equal(candidate, verified):
            self._rows = dict(previous)
            self._disk_state_dirty = True
            self._authority_untrusted = True
            return self._failure([], "reference_state_durable_reread_mismatch")
        self._rows = verified
        self._disk_state_dirty = False
        self._authority_untrusted = False
        return self._clean_commit_result()

    @staticmethod
    def _records_equal(
        left: dict[str, AxisReferenceRecord],
        right: dict[str, AxisReferenceRecord],
    ) -> bool:
        return {
            axis: record.to_payload() for axis, record in left.items()
        } == {
            axis: record.to_payload() for axis, record in right.items()
        }

    def _load_from_disk(self) -> None:
        if self._state_path is None:
            self._authority_untrusted = True
            return
        try:
            with self._state_file_lock(fcntl.LOCK_SH):
                self._load_from_disk_locked()
        except ReferenceStateAuthorityError:
            self._authority_untrusted = True
            self._disk_state_dirty = True

    def _load_from_disk_locked(self) -> None:
        loaded = self._read_rows_from_disk_locked()
        if loaded is None:
            self._authority_untrusted = True
            self._disk_state_dirty = True
            return
        self._rows = loaded
        self._disk_state_dirty = False
        self._authority_untrusted = False

    def _read_rows_from_disk_locked(self) -> dict[str, AxisReferenceRecord] | None:
        if self._database_path is None:
            return None
        try:
            self._database_path.parent.mkdir(parents=True, exist_ok=True)
            connection = self._open_database()
            try:
                selected = connection.execute(
                    "SELECT payload_json,payload_sha256 FROM reference_state_authority WHERE authority_key=?",
                    ("reference_state",),
                ).fetchone()
                if selected is None:
                    payload = {"version": 1, "rows": {}}
                else:
                    encoded = str(selected[0])
                    if encoded != json.dumps(json.loads(encoded), sort_keys=True, separators=(",", ":")):
                        raise ReferenceStateAuthorityError("reference authority JSON is not canonical")
                    if str(selected[1]) != hashlib.sha256(encoded.encode("utf-8")).hexdigest():
                        raise ReferenceStateAuthorityError("reference authority digest mismatch")
                    payload = json.loads(encoded)
            finally:
                connection.close()
        except Exception:
            logger.warning("Failed to load reference state from SQLite %s", self._database_path, exc_info=True)
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

    def _persist_locked(self) -> bool:
        if self._database_path is None:
            self._disk_state_dirty = True
            return False
        try:
            self._database_path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                "version": 1,
                "rows": {axis: record.to_payload() for axis, record in self._rows.items()},
            }
            encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
            connection = self._open_database()
            try:
                connection.execute("BEGIN IMMEDIATE")
                self._authority_write_depth += 1
                connection.execute(
                    "INSERT INTO reference_state_authority(authority_key,payload_json,payload_sha256,updated_at) VALUES(?,?,?,?) ON CONFLICT(authority_key) DO UPDATE SET payload_json=excluded.payload_json,payload_sha256=excluded.payload_sha256,updated_at=excluded.updated_at",
                    ("reference_state", encoded, hashlib.sha256(encoded.encode("utf-8")).hexdigest(), time.time()),
                )
                connection.execute("COMMIT")
            finally:
                self._authority_write_depth = max(0, self._authority_write_depth - 1)
                connection.close()
            return True
        except Exception:
            self._disk_state_dirty = True
            logger.warning("Failed to persist reference state to SQLite %s", self._database_path, exc_info=True)
            return False

    @contextmanager
    def _state_file_lock(self, mode: int):
        if self._state_lock_path is None:
            yield
            return
        try:
            self._state_lock_path.parent.mkdir(parents=True, exist_ok=True)
            lock_file = self._state_lock_path.open("a+")
        except Exception as exc:
            logger.warning("Failed to lock reference state file %s", self._state_lock_path, exc_info=True)
            raise ReferenceStateAuthorityError(f"reference state lock unavailable: {exc}") from exc
        try:
            try:
                fcntl.flock(lock_file.fileno(), mode)
            except Exception as exc:
                logger.warning("Failed to lock reference state file %s", self._state_lock_path, exc_info=True)
                raise ReferenceStateAuthorityError(f"reference state lock unavailable: {exc}") from exc
            try:
                yield
            finally:
                try:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
                except Exception as exc:
                    logger.warning("Failed to unlock reference state file %s", self._state_lock_path, exc_info=True)
                    raise ReferenceStateAuthorityError(f"reference state unlock unavailable: {exc}") from exc
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
