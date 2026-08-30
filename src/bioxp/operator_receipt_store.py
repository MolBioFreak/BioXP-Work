"""Low-overhead robot-local persistence for operator commands.

Compact command and transition data belongs in SQLite. Large route responses and
stage transcripts are stored as robot-local evidence artifacts and loaded only
for an explicit receipt-detail request.
"""
from __future__ import annotations

import errno
import hashlib
import fcntl
import json
import os
import sqlite3
import stat
import tempfile
import time
import uuid
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .runtime_audit_store import (
    RuntimeAuditDatabase,
    assert_migration_slot,
    runtime_audit_migration_identity,
    runtime_state_root as _canonical_runtime_state_root,
)

TERMINAL_STATES = frozenset({
    "completed",
    "cleared",
    "failed",
    "rejected",
    "ambiguous",
    "outcome_unknown",
    "cancelled",
    "reconciliation_required",
})
NONREPLAYABLE_INTERRUPT_ACTIONS = frozenset({
    "meta.emergency_stop",
    "oem.y.stop",
    "oem.z.stop",
    "oem.z.abort",
    "oem.x.stop",
    "oem.abort_all",
})
_LINKED_FINALIZATION_KEY = "_bioxp_linked_pipette_finalization"
MAX_INTERRUPT_FALLBACK_ARCHIVES = 8
_SUMMARY_FIELDS = frozenset({
    "ok",
    "status",
    "state",
    "error",
    "failure",
    "failure_reason",
    "reason",
    "detail",
    "command_id",
    "action_id",
    "started_at",
    "finished_at",
    "duration_ms",
    "delivery_verified",
    "controller_acknowledged",
    "completion_verified",
    "semantic_query_response_verified",
    "hardware_precondition_verified",
    "hardware_postcondition_verified",
    "remote_acknowledged",
    "physical_effect_verified",
    "receipt_truth",
    "truth",
    "automatic_retry",
    "physical_outcome",
    "command_issued",
    "position_steps",
    "speed_steps_s",
    "left_switch_state",
    "right_switch_state",
    "home_switch_active",
    "reference_state",
    "lifecycle_state",
    "result_summary",
    "terminal_state",
    "terminal_z_state",
    "z_authority",
})


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _evidence_parts(relpath: str) -> tuple[str, ...]:
    selected = Path(str(relpath))
    if (
        selected.is_absolute()
        or len(selected.parts) < 2
        or selected.parts[0] != "operator_evidence"
        or any(part in {"", ".", ".."} for part in selected.parts)
    ):
        raise RuntimeError("operator evidence path is outside the governed root")
    return tuple(selected.parts)


def _open_evidence_parent(root: Path, relpath: str) -> tuple[int, str]:
    parts = _evidence_parts(relpath)
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(root, os.O_RDONLY | os.O_DIRECTORY | nofollow)
    try:
        for part in parts[:-1]:
            child = os.open(
                part,
                os.O_RDONLY | os.O_DIRECTORY | nofollow,
                dir_fd=descriptor,
            )
            os.close(descriptor)
            descriptor = child
        return descriptor, parts[-1]
    except Exception:
        os.close(descriptor)
        raise


def _read_confined_evidence(
    root: Path,
    relpath: str,
    *,
    expected_sha256: str | None = None,
    expected_bytes: int | None = None,
) -> tuple[bytes, str, int]:
    parent_descriptor, leaf = _open_evidence_parent(root, relpath)
    descriptor: int | None = None
    try:
        descriptor = os.open(
            leaf,
            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=parent_descriptor,
        )
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise RuntimeError("operator evidence is not a regular file")
        digest = hashlib.sha256()
        chunks: list[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            chunks.append(chunk)
        after = os.fstat(descriptor)
        if (before.st_dev, before.st_ino, before.st_size) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
        ):
            raise RuntimeError("operator evidence changed while it was read")
        actual_digest = digest.hexdigest()
        if expected_bytes is not None and before.st_size != int(expected_bytes):
            raise RuntimeError("operator evidence size mismatch")
        if expected_sha256 is not None and actual_digest != str(expected_sha256):
            raise RuntimeError("operator evidence digest mismatch")
        return b"".join(chunks), actual_digest, int(before.st_size)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent_descriptor)


def _unlink_confined_evidence(
    root: Path,
    relpath: str,
    *,
    expected_sha256: str,
    expected_bytes: int,
) -> None:
    parts = _evidence_parts(relpath)
    parent_descriptor, leaf = _open_evidence_parent(root, relpath)
    tombstone = f".{leaf}.expiry-{uuid.uuid4().hex}"
    moved = False
    try:
        os.rename(
            leaf,
            tombstone,
            src_dir_fd=parent_descriptor,
            dst_dir_fd=parent_descriptor,
        )
        moved = True
        os.fsync(parent_descriptor)
        tombstone_relpath = Path(*parts[:-1], tombstone).as_posix()
        try:
            _read_confined_evidence(
                root,
                tombstone_relpath,
                expected_sha256=expected_sha256,
                expected_bytes=expected_bytes,
            )
        except Exception:
            try:
                os.stat(leaf, dir_fd=parent_descriptor, follow_symlinks=False)
            except FileNotFoundError:
                os.rename(
                    tombstone,
                    leaf,
                    src_dir_fd=parent_descriptor,
                    dst_dir_fd=parent_descriptor,
                )
                moved = False
                os.fsync(parent_descriptor)
            raise
        os.unlink(tombstone, dir_fd=parent_descriptor)
        moved = False
        os.fsync(parent_descriptor)
    finally:
        if moved:
            # Leave the confined tombstone in place for startup classification;
            # never guess which concurrently recreated leaf should be removed.
            os.fsync(parent_descriptor)
        os.close(parent_descriptor)


def _ensure_durable_directory(path: Path) -> None:
    missing: list[Path] = []
    current = path
    while not current.exists():
        missing.append(current)
        if current == current.parent:
            break
        current = current.parent
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    for created in reversed(missing):
        _fsync_directory(created.parent)


def runtime_state_root(root: str | Path | None = None) -> Path:
    return _canonical_runtime_state_root(root)


def _json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        default=str,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _json_text(value: Any) -> str:
    return _json_bytes(value).decode("utf-8")


def _bounded_scalar(value: Any, *, max_string: int = 2000) -> Any:
    if value is None or type(value) in {bool, int}:
        return value
    if type(value) is float:
        return value if value == value and value not in {float("inf"), float("-inf")} else None
    if isinstance(value, str):
        return value if len(value) <= max_string else value[:max_string] + "...[truncated]"
    return str(value)[:max_string]


def compact_response_summary(value: Any, *, max_depth: int = 6, max_items: int = 96) -> Any:
    """Keep decision fields while excluding route transcripts and broad snapshots."""
    remaining = [max_items]

    def visit(item: Any, depth: int, *, selected: bool = False) -> Any:
        if remaining[0] <= 0:
            return {"omitted": "summary_item_limit"}
        remaining[0] -= 1
        if item is None or type(item) in {bool, int, float, str}:
            return _bounded_scalar(item)
        if depth >= max_depth:
            return {"omitted": "summary_depth_limit"}
        if isinstance(item, Mapping):
            output: dict[str, Any] = {}
            for raw_key, raw_value in item.items():
                key = str(raw_key)[:96]
                keep = selected or key in _SUMMARY_FIELDS
                if keep:
                    output[key] = visit(raw_value, depth + 1, selected=key in {
                        "detail",
                        "result_summary",
                        "receipt_truth",
                        "truth",
                        "terminal_state",
                        "terminal_z_state",
                        "z_authority",
                    })
                elif isinstance(raw_value, Mapping):
                    nested = visit(raw_value, depth + 1)
                    if isinstance(nested, Mapping) and nested:
                        output[key] = nested
                if remaining[0] <= 0:
                    break
            return output
        if isinstance(item, (list, tuple)):
            if not selected:
                return []
            return [visit(entry, depth + 1, selected=True) for entry in list(item)[:16]]
        return _bounded_scalar(item)

    projected = visit(value, 0, selected=True)
    raw = _json_bytes(projected)
    if len(raw) <= 16_384:
        return projected
    return {
        "bounded": True,
        "original_summary_bytes": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest(),
    }


class OperatorReceiptStore:
    """SQLite command index with on-demand file-backed detailed evidence."""

    def __init__(
        self,
        root: str | Path | None = None,
        *,
        initialize_schema: bool = False,
    ) -> None:
        self.root = runtime_state_root(root)
        self.path = self.root / "bioxp_runtime.db"
        self.evidence_root = self.root / "operator_evidence"
        self.interrupt_fallback_path = self.root / "operator_interrupt_fallback.jsonl"
        self.interrupt_fallback_lock_path = self.root / "operator_interrupt_fallback.lock"
        _ensure_durable_directory(self.evidence_root)
        if self.evidence_root.is_symlink() or not self.evidence_root.is_dir():
            raise RuntimeError("operator evidence root must be a non-symlink directory")
        os.chmod(self.evidence_root, 0o700)
        self.legacy_path = self.root / "operator_action_receipts.json"
        fallback_legacy = Path("/tmp/bioxp-oem-runtime/operator_action_receipts.json")
        if not self.legacy_path.exists() and fallback_legacy.exists():
            self.legacy_path = fallback_legacy
        self._audit_database = RuntimeAuditDatabase(root=self.root, initialize_schema=False)
        self.connection = self._audit_database.connection
        self.lock = self._audit_database.writer_lock
        with self.lock:
            self._configure()
            if initialize_schema:
                raise RuntimeError("constructor-owned migration is retired; prepare the canonical database explicitly")
            from .oem_runtime_store import verify_canonical_runtime_database

            verify_canonical_runtime_database(self.connection)
            if not assert_migration_slot(
                self.connection,
                runtime_audit_migration_identity(),
            ):
                raise RuntimeError(
                    "runtime audit schema must be migrated by the API lifespan owner before store construction"
                )
            _fsync_directory(self.root)

    def converge_startup_state(self) -> None:
        """Run legacy import and evidence recovery under the API lifespan owner."""
        with self.lock:
            self._import_legacy_once()
            self._import_interrupt_fallback()
            self._remove_orphan_evidence()
            self.sweep_expired_evidence()
            self.reconcile_nonterminal_receipts()

    def _configure(self) -> None:
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=FULL")
        self.connection.execute("PRAGMA foreign_keys=ON")
        self.connection.execute("PRAGMA busy_timeout=2000")
        self.connection.execute("PRAGMA wal_autocheckpoint=256")
        self.connection.execute("PRAGMA journal_size_limit=4194304")
        self.connection.execute("PRAGMA temp_store=MEMORY")

    def _evidence_path(self, command_id: str, started_at: Any, digest: str) -> tuple[Path, str]:
        try:
            day = time.strftime("%Y-%m-%d", time.localtime(float(started_at)))
        except (TypeError, ValueError, OverflowError):
            day = "unknown-date"
        safe_id = "".join(character for character in command_id if character.isalnum() or character in "._-")[:160]
        relpath = Path("operator_evidence") / day / f"{safe_id or 'command'}.{digest}.json"
        return self.root / relpath, relpath.as_posix()

    def _persist_evidence(self, receipt: Mapping[str, Any]) -> tuple[str, str, int] | tuple[None, None, None]:
        response = receipt.get("response")
        stages = receipt.get("stage_receipts")
        if response is None and not stages:
            return None, None, None
        command_id = str(receipt.get("command_id") or "")
        raw = _json_bytes({
            "schema_version": "bioxp.operator_action_evidence.v1",
            "command_id": command_id,
            "action_id": receipt.get("action_id"),
            "response": response,
            "stage_receipts": stages,
        })
        digest = hashlib.sha256(raw).hexdigest()
        path, relpath = self._evidence_path(command_id, receipt.get("started_at"), digest)
        _ensure_durable_directory(path.parent)
        fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
        temporary = Path(temporary_name)
        try:
            os.fchmod(fd, 0o600)
            with os.fdopen(fd, "wb", closefd=True) as handle:
                handle.write(raw)
                handle.flush()
                os.fsync(handle.fileno())
            try:
                os.link(temporary, path)
            except FileExistsError:
                existing, existing_digest, existing_size = _read_confined_evidence(
                    self.root,
                    relpath,
                )
                if existing_digest != digest or existing_size != len(raw):
                    raise RuntimeError("immutable operator evidence path identity mismatch")
                if existing != raw:
                    raise RuntimeError("immutable operator evidence path contains different bytes")
            _fsync_directory(path.parent)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        return relpath, digest, len(raw)

    @staticmethod
    def _five_calendar_year_deadline(now: float) -> float:
        current = datetime.fromtimestamp(float(now), tz=timezone.utc)
        try:
            future = current.replace(year=current.year + 5)
        except ValueError:
            future = current.replace(year=current.year + 5, day=28)
        return future.timestamp()

    def _register_evidence(
        self,
        receipt: Mapping[str, Any],
        evidence: tuple[str, str, int] | tuple[None, None, None],
    ) -> str | None:
        relpath, digest, size = evidence
        if relpath is None or digest is None or size is None:
            return None
        artifact_id = f"evidence:{digest}"
        now = time.time()
        deadline = self._five_calendar_year_deadline(now)
        existing = self.connection.execute(
            "SELECT original_relpath,sha256,byte_count FROM runtime_evidence_objects WHERE evidence_artifact_id=?",
            (artifact_id,),
        ).fetchone()
        if existing is not None:
            if (
                str(existing["original_relpath"]) != str(relpath)
                or str(existing["sha256"]) != str(digest)
                or int(existing["byte_count"]) != int(size)
            ):
                raise RuntimeError("evidence artifact identity collision")
        else:
            self.connection.execute(
                """
                INSERT INTO runtime_evidence_objects(
                    evidence_artifact_id,command_id,pipette_operation_id,original_relpath,active_relpath,
                    sha256,byte_count,created_at,retention_deadline,legal_hold,expiry_state,updated_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    artifact_id,
                    receipt.get("command_id"),
                    receipt.get("pipette_operation_id"),
                    str(relpath),
                    str(relpath),
                    str(digest),
                    int(size),
                    now,
                    float(deadline),
                    0,
                    "active",
                    now,
                ),
            )
            self.connection.execute(
                "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                (artifact_id, "published", now, _json_text({"relpath": relpath, "sha256": digest, "byte_count": size})),
            )
        command_id = receipt.get("command_id")
        if command_id:
            link_columns = {
                str(row["name"])
                for row in self.connection.execute("PRAGMA table_info(runtime_evidence_links)").fetchall()
            }
            if {"target_kind", "target_identity"}.issubset(link_columns):
                self.connection.execute(
                    """
                    INSERT INTO runtime_evidence_links(
                        evidence_artifact_id,target_kind,target_identity,command_id,link_kind,created_at
                    ) VALUES(?,?,?,?,?,?)
                    """,
                    (artifact_id, "command", str(command_id), str(command_id), "command_evidence", now),
                )
            else:
                self.connection.execute(
                    "INSERT INTO runtime_evidence_links(evidence_artifact_id,command_id,link_kind,created_at) VALUES(?,?,?,?)",
                    (artifact_id, str(command_id), "command_evidence", now),
                )
        return artifact_id

    def _apply_legal_hold_assessment_locked(
        self,
        command_id: str,
        *,
        legal_hold: bool,
        actor: str,
        assessment: Mapping[str, Any] | None,
        observed_at: float,
    ) -> dict[str, Any]:
        if not self.connection.in_transaction:
            raise RuntimeError("legal-hold assessment requires the caller transaction")
        row = self.connection.execute(
            """
            SELECT o.evidence_artifact_id,o.expiry_state
            FROM runtime_evidence_objects o
            JOIN runtime_evidence_links l
              ON l.evidence_artifact_id=o.evidence_artifact_id
            WHERE l.target_kind='command' AND l.target_identity=?
            ORDER BY o.created_at DESC,o.evidence_artifact_id DESC LIMIT 1
            """,
            (str(command_id),),
        ).fetchone()
        if row is None:
            raise KeyError("command evidence is unavailable for legal hold")
        if str(row["expiry_state"]) not in {"active", "retained"}:
            raise RuntimeError("legal hold cannot change after evidence expiry begins")
        artifact_id = str(row["evidence_artifact_id"])
        detail = {
            "legal_hold": legal_hold,
            "actor": actor,
            "assessment": dict(assessment or {}),
        }
        updated = self.connection.execute(
            "UPDATE runtime_evidence_objects SET legal_hold=?,updated_at=? "
            "WHERE evidence_artifact_id=? AND expiry_state IN ('active','retained')",
            (int(legal_hold), observed_at, artifact_id),
        )
        if updated.rowcount != 1:
            raise RuntimeError("legal-hold projection compare-and-swap failed")
        cursor = self.connection.execute(
            "INSERT INTO runtime_evidence_events("
            "evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
            (artifact_id, "legal_hold_assessment", observed_at, _json_text(detail)),
        )
        event_id = cursor.lastrowid
        if type(event_id) is not int:
            raise RuntimeError("legal-hold assessment event identity is unavailable")
        self.connection.execute(
            "INSERT INTO runtime_evidence_links("
            "evidence_artifact_id,target_kind,target_identity,command_id,link_kind,created_at"
            ") VALUES(?,?,?,?,?,?)",
            (
                artifact_id,
                "assessment",
                f"legal_hold:{event_id}",
                None,
                "legal_hold_assessment",
                observed_at,
            ),
        )
        return {
            "state": "legal_hold" if legal_hold else "released",
            "command_id": str(command_id),
            "evidence_artifact_id": artifact_id,
            "assessment_event_id": event_id,
            "actor": actor,
        }

    def assess_evidence_legal_hold(
        self,
        command_id: str,
        *,
        legal_hold: bool,
        actor: str,
        assessment: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Append a governed legal-hold assessment and update its projection."""
        if type(legal_hold) is not bool:
            raise TypeError("legal_hold must be an exact boolean")
        named_actor = str(actor).strip()
        if not named_actor:
            raise ValueError("legal-hold assessment requires a named actor")
        with self.lock:
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                result = self._apply_legal_hold_assessment_locked(
                    str(command_id),
                    legal_hold=legal_hold,
                    actor=named_actor,
                    assessment=assessment,
                    observed_at=time.time(),
                )
                self.connection.execute("COMMIT")
                return result
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise

    def expire_evidence(
        self,
        command_id: str | None = None,
        *,
        evidence_artifact_id: str | None = None,
    ) -> dict[str, Any]:
        if (command_id is None) == (evidence_artifact_id is None):
            raise ValueError("select exactly one evidence expiry identity")
        current_time = time.time()
        with self.lock:
            if evidence_artifact_id is None:
                row = self.connection.execute(
                    """
                    SELECT o.*
                    FROM runtime_evidence_objects o
                    JOIN runtime_evidence_links l
                      ON l.evidence_artifact_id=o.evidence_artifact_id
                    WHERE l.target_kind='command' AND l.target_identity=?
                    ORDER BY o.created_at DESC,o.evidence_artifact_id DESC LIMIT 1
                    """,
                    (str(command_id),),
                ).fetchone()
            else:
                row = self.connection.execute(
                    "SELECT * FROM runtime_evidence_objects WHERE evidence_artifact_id=?",
                    (str(evidence_artifact_id),),
                ).fetchone()
            if row is None:
                return {
                    "state": "no_evidence",
                    "command_id": None if command_id is None else str(command_id),
                    "evidence_artifact_id": evidence_artifact_id,
                }
            artifact_id = str(row["evidence_artifact_id"])
            state = str(row["expiry_state"])
            if state in {"expired", "missing", "integrity_failed", "orphan_cleaned"}:
                return {
                    "state": state,
                    "command_id": None if command_id is None else str(command_id),
                    "evidence_artifact_id": artifact_id,
                    "expiry_receipt_id": row["expiry_receipt_id"],
                }
            if row["retention_deadline"] is None:
                raise RuntimeError("evidence retention authority is missing")
            deadline = float(row["retention_deadline"])
            if bool(row["legal_hold"]):
                return {
                    "state": "legal_hold",
                    "command_id": None if command_id is None else str(command_id),
                    "evidence_artifact_id": artifact_id,
                }
            if state != "expiry_pending" and current_time < deadline:
                return {
                    "state": "retained",
                    "command_id": None if command_id is None else str(command_id),
                    "evidence_artifact_id": artifact_id,
                }
            relpath = row["active_relpath"]
            if state != "expiry_pending":
                if not relpath:
                    raise RuntimeError("active evidence is missing its retained path")
                self.connection.execute("BEGIN IMMEDIATE")
                try:
                    updated = self.connection.execute(
                        "UPDATE runtime_evidence_objects SET expiry_state='expiry_pending',updated_at=? "
                        "WHERE evidence_artifact_id=? AND expiry_state IN ('active','retained') AND legal_hold=0",
                        (current_time, artifact_id),
                    )
                    if updated.rowcount != 1:
                        raise RuntimeError("evidence expiry-pending compare-and-swap failed")
                    self.connection.execute(
                        "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                        (artifact_id, "expiry_pending", current_time, _json_text({"command_id": command_id})),
                    )
                    self.connection.execute("COMMIT")
                except Exception:
                    if self.connection.in_transaction:
                        self.connection.execute("ROLLBACK")
                    raise

        with self.lock:
            current = self.connection.execute(
                "SELECT active_relpath,sha256,byte_count,legal_hold,expiry_state "
                "FROM runtime_evidence_objects WHERE evidence_artifact_id=?",
                (artifact_id,),
            ).fetchone()
            if current is None:
                raise RuntimeError("evidence authority disappeared during expiry")
            if bool(current["legal_hold"]):
                return {
                    "state": "legal_hold",
                    "command_id": None if command_id is None else str(command_id),
                    "evidence_artifact_id": artifact_id,
                }
            if str(current["expiry_state"]) != "expiry_pending":
                raise RuntimeError("evidence expiry state changed before deletion")
            active_relpath = current["active_relpath"]
            already_absent = False
            if active_relpath:
                try:
                    _unlink_confined_evidence(
                        self.root,
                        str(active_relpath),
                        expected_sha256=str(current["sha256"]),
                        expected_bytes=int(current["byte_count"]),
                    )
                except FileNotFoundError:
                    already_absent = True
                except Exception as exc:
                    failure_time = time.time()
                    self.connection.execute("BEGIN IMMEDIATE")
                    try:
                        self.connection.execute(
                            "UPDATE runtime_evidence_objects SET expiry_state='integrity_failed',updated_at=? "
                            "WHERE evidence_artifact_id=? AND expiry_state='expiry_pending'",
                            (failure_time, artifact_id),
                        )
                        self.connection.execute(
                            "UPDATE operator_commands SET evidence_state='integrity_failed',updated_at=? "
                            "WHERE command_id IN (SELECT target_identity FROM runtime_evidence_links "
                            "WHERE evidence_artifact_id=? AND target_kind='command')",
                            (failure_time, artifact_id),
                        )
                        self.connection.execute(
                            "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                            (artifact_id, "integrity_failure", failure_time, _json_text({"reason_code": "expiry_delete_integrity_failure"})),
                        )
                        self.connection.execute("COMMIT")
                    except Exception:
                        if self.connection.in_transaction:
                            self.connection.execute("ROLLBACK")
                    raise RuntimeError("evidence expiry integrity failure") from exc

            expiry_receipt_id = uuid.uuid4().hex
            completed_at = time.time()
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                updated = self.connection.execute(
                    "UPDATE runtime_evidence_objects SET active_relpath=NULL,expiry_state='expired',"
                    "expiry_receipt_id=?,updated_at=? WHERE evidence_artifact_id=? "
                    "AND expiry_state='expiry_pending' AND legal_hold=0",
                    (expiry_receipt_id, completed_at, artifact_id),
                )
                if updated.rowcount != 1:
                    raise RuntimeError("evidence expiry final compare-and-swap failed")
                self.connection.execute(
                    "UPDATE operator_commands SET evidence_relpath=NULL,evidence_state='expired',updated_at=? "
                    "WHERE command_id IN (SELECT target_identity FROM runtime_evidence_links "
                    "WHERE evidence_artifact_id=? AND target_kind='command')",
                    (completed_at, artifact_id),
                )
                for event_kind in ("deleted", "expired"):
                    self.connection.execute(
                        "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                        (artifact_id, event_kind, completed_at, _json_text({"expiry_receipt_id": expiry_receipt_id, "already_absent": already_absent})),
                    )
                self.connection.execute("COMMIT")
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise
        return {
            "state": "expired",
            "command_id": None if command_id is None else str(command_id),
            "evidence_artifact_id": artifact_id,
            "expiry_receipt_id": expiry_receipt_id,
        }

    def sweep_expired_evidence(self, *, limit: int = 1000) -> dict[str, Any]:
        selected_limit = int(limit)
        if selected_limit < 1 or selected_limit > 1000:
            raise ValueError("retention sweep limit must be between 1 and 1000")
        with self.lock:
            rows = self.connection.execute(
                "SELECT evidence_artifact_id FROM runtime_evidence_objects "
                "WHERE expiry_state='expiry_pending' OR "
                "(expiry_state IN ('active','retained') AND legal_hold=0 "
                "AND retention_deadline IS NOT NULL AND retention_deadline<=?) "
                "ORDER BY COALESCE(retention_deadline,0),evidence_artifact_id LIMIT ?",
                (time.time(), selected_limit),
            ).fetchall()
        outcomes = [
            self.expire_evidence(evidence_artifact_id=str(row["evidence_artifact_id"]))
            for row in rows
        ]
        return {
            "selected": len(rows),
            "expired": sum(item.get("state") == "expired" for item in outcomes),
            "retained": sum(item.get("state") == "retained" for item in outcomes),
            "legal_hold": sum(item.get("state") == "legal_hold" for item in outcomes),
        }

    @staticmethod
    def _compact_receipt(receipt: Mapping[str, Any]) -> tuple[dict[str, Any], Any]:
        compact = dict(receipt)
        response = compact.pop("response", None)
        compact.pop("stage_receipts", None)
        response_summary = compact_response_summary(response) if response is not None else None
        compact["response"] = response_summary
        compact["stage_receipts"] = []
        return compact, response_summary

    def _upsert(
        self,
        receipt: Mapping[str, Any],
        *,
        evidence: tuple[str, str, int] | tuple[None, None, None],
        expected_status: str | None = None,
        reconciliation_transition: bool = False,
    ) -> dict[str, Any]:
        command_id = str(receipt.get("command_id") or "")
        idempotency_key = str(receipt.get("idempotency_key") or "")
        action_id = str(receipt.get("action_id") or "")
        replay_enabled = (
            action_id not in NONREPLAYABLE_INTERRUPT_ACTIONS
            and receipt.get("idempotency_replay_enabled") is not False
        )
        selected_receipt = dict(receipt)
        selected_receipt["idempotency_replay_enabled"] = replay_enabled
        compact, response_summary = self._compact_receipt(selected_receipt)
        status = str(receipt.get("status") or "unknown")
        if not command_id or not idempotency_key or not action_id:
            raise ValueError("operator receipt requires command_id, idempotency_key, and action_id")
        previous = self.connection.execute(
            "SELECT status FROM operator_commands WHERE command_id=?",
            (command_id,),
        ).fetchone()
        previous_status = str(previous["status"]) if previous is not None else None
        if reconciliation_transition:
            if status not in TERMINAL_STATES:
                raise RuntimeError("operator receipt reconciliation target must be terminal")
            if (
                expected_status not in TERMINAL_STATES
                or previous_status != expected_status
            ):
                raise RuntimeError("stale operator receipt reconciliation expected state")
        elif previous is not None:
            if expected_status is None or previous_status != expected_status:
                raise RuntimeError("stale operator receipt expected state")
            if previous_status in TERMINAL_STATES:
                raise RuntimeError(
                    f"ordinary receipt transition cannot mutate terminal state {previous_status}"
                )
        elif expected_status is not None:
            raise RuntimeError("operator receipt expected state has no matching claim")
        relpath, digest, size = evidence
        if relpath is None and previous is not None:
            existing = self.connection.execute(
                "SELECT evidence_relpath,evidence_sha256,evidence_bytes FROM operator_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if existing is not None:
                relpath, digest, size = existing
        now = time.time()
        upsert = self.connection.execute(
            """
            INSERT INTO operator_commands(
                command_id,idempotency_key,idempotency_replay_enabled,action_id,status,safety_class,
                ownership_generation,started_at,finished_at,duration_ms,
                delivery_verified,controller_acknowledged,completion_verified,
                hardware_precondition_verified,hardware_postcondition_verified,
                physical_effect_verified,receipt_json,
                response_summary_json,evidence_relpath,evidence_sha256,evidence_bytes,updated_at
            ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            ON CONFLICT(command_id) DO UPDATE SET
                idempotency_replay_enabled=excluded.idempotency_replay_enabled,
                status=excluded.status,
                finished_at=excluded.finished_at,
                duration_ms=excluded.duration_ms,
                delivery_verified=excluded.delivery_verified,
                controller_acknowledged=excluded.controller_acknowledged,
                completion_verified=excluded.completion_verified,
                hardware_precondition_verified=excluded.hardware_precondition_verified,
                hardware_postcondition_verified=excluded.hardware_postcondition_verified,
                physical_effect_verified=excluded.physical_effect_verified,
                receipt_json=excluded.receipt_json,
                response_summary_json=excluded.response_summary_json,
                evidence_relpath=COALESCE(excluded.evidence_relpath,operator_commands.evidence_relpath),
                evidence_sha256=COALESCE(excluded.evidence_sha256,operator_commands.evidence_sha256),
                evidence_bytes=COALESCE(excluded.evidence_bytes,operator_commands.evidence_bytes),
                updated_at=excluded.updated_at
            WHERE operator_commands.status=?
            """,
            (
                command_id,
                idempotency_key,
                int(replay_enabled),
                action_id,
                status,
                receipt.get("safety_class"),
                int(receipt.get("ownership_generation", -1)),
                str(receipt.get("started_at") or now),
                None if receipt.get("finished_at") is None else str(receipt.get("finished_at")),
                receipt.get("duration_ms"),
                int(receipt.get("delivery_verified") is True),
                int(receipt.get("controller_acknowledged") is True),
                int(receipt.get("completion_verified") is True),
                int(receipt.get("hardware_precondition_verified") is True),
                int(receipt.get("hardware_postcondition_verified") is True),
                int(receipt.get("physical_effect_verified") is True),
                _json_text(compact),
                None if response_summary is None else _json_text(response_summary),
                relpath,
                digest,
                size,
                now,
                previous_status if previous_status is not None else status,
            ),
        )
        if upsert.rowcount != 1:
            raise RuntimeError("operator receipt state changed during persistence")
        if previous is None or str(previous["status"]) != status:
            transition_detail = {
                "machine_assessment": receipt.get("machine_assessment"),
                "error": receipt.get("error"),
                "controller_acknowledged": receipt.get("controller_acknowledged") is True,
                "physical_effect_verified": receipt.get("physical_effect_verified") is True,
            }
            self.connection.execute(
                "INSERT INTO operator_transitions(command_id,state,observed_at,detail_json) VALUES(?,?,?,?)",
                (command_id, status, now, _json_text(transition_detail)),
            )
        return compact

    def _prune_locked(self) -> list[str]:
        # Compact command metadata is retained indefinitely. Evidence lifecycle
        # owns byte expiry and never deletes the command projection here.
        return []

    def _remove_evidence_files_locked(self, relpaths: list[str]) -> None:
        """Remove only objects whose durable lifecycle state authorizes cleanup."""
        changed_directories: set[Path] = set()
        for relpath in relpaths:
            authority = self.connection.execute(
                """
                SELECT sha256,byte_count,expiry_state FROM runtime_evidence_objects
                WHERE active_relpath=?
                """,
                (relpath,),
            ).fetchone()
            if authority is None or str(authority["expiry_state"]) not in {
                "expiry_pending",
                "orphan_quarantined",
            }:
                continue
            try:
                _unlink_confined_evidence(
                    self.root,
                    str(relpath),
                    expected_sha256=str(authority["sha256"]),
                    expected_bytes=int(authority["byte_count"]),
                )
            except FileNotFoundError:
                continue
            changed_directories.add((self.root / relpath).parent)
        for directory in sorted(changed_directories, key=lambda path: len(path.parts), reverse=True):
            current = directory
            while current != self.evidence_root:
                try:
                    current.rmdir()
                except OSError as exc:
                    if exc.errno in {errno.ENOENT, errno.ENOTEMPTY, errno.EEXIST}:
                        break
                    raise
                parent = current.parent
                _fsync_directory(parent)
                current = parent

    def _remove_pruned_evidence(self, relpaths: list[str], *, nonblocking: bool = False) -> None:
        if not relpaths:
            return
        with self.lock:
            if nonblocking:
                self.connection.execute("PRAGMA busy_timeout=0")
            try:
                try:
                    self.connection.execute("BEGIN IMMEDIATE")
                except sqlite3.Error:
                    return
                try:
                    self._remove_evidence_files_locked(relpaths)
                    self.connection.execute("COMMIT")
                except Exception:
                    if self.connection.in_transaction:
                        self.connection.execute("ROLLBACK")
                    raise
            finally:
                if nonblocking:
                    self.connection.execute("PRAGMA busy_timeout=2000")

    def _remove_orphan_evidence(self) -> None:
        """Classify every unbound object durably before confined cleanup."""
        interrupted = self.connection.execute(
            "SELECT evidence_artifact_id,active_relpath,sha256,byte_count FROM runtime_evidence_objects WHERE expiry_state='orphan_quarantined' AND legal_hold=0 ORDER BY evidence_artifact_id"
        ).fetchall()
        for row in interrupted:
            relpath = str(row["active_relpath"] or "")
            if not relpath:
                raise RuntimeError("orphan quarantine row is missing its active relpath")
            try:
                _unlink_confined_evidence(
                    self.root,
                    relpath,
                    expected_sha256=str(row["sha256"]),
                    expected_bytes=int(row["byte_count"]),
                )
            except FileNotFoundError:
                pass
            receipt_id = str(uuid.uuid4())
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                updated = self.connection.execute(
                    "UPDATE runtime_evidence_objects SET active_relpath=NULL,expiry_state='orphan_cleaned',expiry_receipt_id=?,updated_at=? WHERE evidence_artifact_id=? AND active_relpath=? AND expiry_state='orphan_quarantined' AND legal_hold=0",
                    (receipt_id, time.time(), str(row["evidence_artifact_id"]), relpath),
                )
                if updated.rowcount != 1:
                    raise RuntimeError("orphan cleanup compare-and-swap failed")
                self.connection.execute(
                    "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                    (
                        str(row["evidence_artifact_id"]), "orphan_deleted", time.time(),
                        _json_text({"receipt_id": receipt_id, "relpath": relpath, "reason": "orphan_cleanup"}),
                    ),
                )
                self.connection.execute(
                    "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                    (
                        str(row["evidence_artifact_id"]), "orphan_cleaned", time.time(),
                        _json_text({
                            "receipt_id": receipt_id,
                            "sha256": str(row["sha256"]),
                            "byte_count": int(row["byte_count"]),
                            "reason": "orphan_cleanup",
                        }),
                    ),
                )
                self.connection.execute("COMMIT")
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise
        referenced = {
            str(row["active_relpath"])
            for row in self.connection.execute(
                "SELECT active_relpath FROM runtime_evidence_objects WHERE active_relpath IS NOT NULL"
            ).fetchall()
        }
        regular: list[tuple[str, str, int]] = []
        special: list[tuple[str, str]] = []
        for directory, directory_names, file_names in os.walk(
            self.evidence_root,
            topdown=True,
            followlinks=False,
        ):
            selected_directory = Path(directory)
            for name in list(directory_names):
                candidate = selected_directory / name
                mode = candidate.lstat().st_mode
                if stat.S_ISLNK(mode):
                    directory_names.remove(name)
                    relpath = candidate.relative_to(self.root).as_posix()
                    if relpath not in referenced:
                        special.append((relpath, "symlink"))
            for name in file_names:
                candidate = selected_directory / name
                relpath = candidate.relative_to(self.root).as_posix()
                if relpath in referenced:
                    continue
                mode = candidate.lstat().st_mode
                if not stat.S_ISREG(mode):
                    special.append((relpath, "symlink" if stat.S_ISLNK(mode) else "non_regular"))
                    continue
                _, digest, size = _read_confined_evidence(self.root, relpath)
                regular.append((relpath, digest, size))

        classified_at = time.time()
        classified: list[tuple[str, str, str, int]] = []
        self.connection.execute("BEGIN IMMEDIATE")
        try:
            for relpath, digest, size in regular:
                artifact_id = "orphan:" + hashlib.sha256(
                    f"{relpath}\0{digest}\0{size}".encode("utf-8")
                ).hexdigest()
                self.connection.execute(
                    """
                    INSERT OR IGNORE INTO runtime_evidence_objects(
                        evidence_artifact_id,command_id,pipette_operation_id,original_relpath,
                        active_relpath,sha256,byte_count,created_at,retention_deadline,
                        legal_hold,expiry_state,updated_at
                    ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)
                    """,
                    (
                        artifact_id,
                        None,
                        None,
                        relpath,
                        relpath,
                        digest,
                        size,
                        classified_at,
                        self._five_calendar_year_deadline(classified_at),
                        0,
                        "orphan_quarantined",
                        classified_at,
                    ),
                )
                authority = self.connection.execute(
                    """
                    SELECT active_relpath,sha256,byte_count,expiry_state
                    FROM runtime_evidence_objects WHERE evidence_artifact_id=?
                    """,
                    (artifact_id,),
                ).fetchone()
                if (
                    authority is None
                    or str(authority["active_relpath"]) != relpath
                    or str(authority["sha256"]) != digest
                    or int(authority["byte_count"]) != size
                    or str(authority["expiry_state"]) != "orphan_quarantined"
                ):
                    raise RuntimeError("orphan evidence classification identity conflict")
                self.connection.execute(
                    """
                    INSERT INTO runtime_evidence_events(
                        evidence_artifact_id,event_kind,observed_at,detail_json
                    ) VALUES(?,?,?,?)
                    """,
                    (
                        artifact_id,
                        "orphan_classified",
                        classified_at,
                        _json_text({"relpath": relpath, "sha256": digest, "byte_count": size}),
                    ),
                )
                classified.append((artifact_id, relpath, digest, size))
            for relpath, object_kind in special:
                digest = hashlib.sha256(
                    f"{relpath}\0{object_kind}".encode("utf-8")
                ).hexdigest()
                artifact_id = f"orphan-integrity:{digest}"
                self.connection.execute(
                    """
                    INSERT OR IGNORE INTO runtime_evidence_objects(
                        evidence_artifact_id,command_id,pipette_operation_id,original_relpath,
                        active_relpath,sha256,byte_count,created_at,retention_deadline,
                        legal_hold,expiry_state,updated_at
                    ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)
                    """,
                    (
                        artifact_id,
                        None,
                        None,
                        relpath,
                        relpath,
                        digest,
                        0,
                        classified_at,
                        self._five_calendar_year_deadline(classified_at),
                        0,
                        "integrity_failed",
                        classified_at,
                    ),
                )
                self.connection.execute(
                    """
                    INSERT INTO runtime_evidence_events(
                        evidence_artifact_id,event_kind,observed_at,detail_json
                    ) VALUES(?,?,?,?)
                    """,
                    (
                        artifact_id,
                        "integrity_failure",
                        classified_at,
                        _json_text({"relpath": relpath, "object_kind": object_kind}),
                    ),
                )
            self.connection.execute("COMMIT")
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

        for artifact_id, relpath, digest, size in classified:
            try:
                _unlink_confined_evidence(
                    self.root,
                    relpath,
                    expected_sha256=digest,
                    expected_bytes=size,
                )
            except Exception as exc:
                failed_at = time.time()
                self.connection.execute("BEGIN IMMEDIATE")
                try:
                    self.connection.execute(
                        "UPDATE runtime_evidence_objects SET expiry_state='integrity_failed',updated_at=? WHERE evidence_artifact_id=?",
                        (failed_at, artifact_id),
                    )
                    self.connection.execute(
                        "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                        (
                            artifact_id,
                            "integrity_failure",
                            failed_at,
                            _json_text({"relpath": relpath, "error": str(exc)[:500]}),
                        ),
                    )
                    self.connection.execute("COMMIT")
                except Exception:
                    if self.connection.in_transaction:
                        self.connection.execute("ROLLBACK")
                raise
            cleaned_at = time.time()
            cleanup_receipt = uuid.uuid4().hex
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                cleaned = self.connection.execute(
                    """
                    UPDATE runtime_evidence_objects
                    SET active_relpath=NULL,expiry_state='orphan_cleaned',
                        expiry_receipt_id=?,updated_at=?
                    WHERE evidence_artifact_id=? AND active_relpath=? AND expiry_state='orphan_quarantined'
                    """,
                    (cleanup_receipt, cleaned_at, artifact_id, relpath),
                )
                if cleaned.rowcount != 1:
                    raise RuntimeError("orphan cleanup compare-and-swap failed")
                for event_kind in ("orphan_deleted", "orphan_cleaned"):
                    self.connection.execute(
                        "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                        (
                            artifact_id,
                            event_kind,
                            cleaned_at,
                            _json_text({"cleanup_receipt_id": cleanup_receipt}),
                        ),
                    )
                self.connection.execute("COMMIT")
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise

        directories = sorted(
            (path for path in self.evidence_root.rglob("*") if path.is_dir() and not path.is_symlink()),
            key=lambda path: len(path.parts),
            reverse=True,
        )
        for directory in directories:
            try:
                directory.rmdir()
            except OSError:
                continue
            _fsync_directory(directory.parent)

    def _import_legacy_once(self) -> None:
        self.connection.execute("BEGIN IMMEDIATE")
        pruned: list[str] = []
        try:
            marker = self.connection.execute(
                "SELECT value FROM runtime_metadata WHERE key='operator_receipt_legacy_import_v1'"
            ).fetchone()
            if marker is not None:
                self.connection.execute("COMMIT")
                return
            imported = 0
            if self.legacy_path.exists():
                payload = json.loads(self.legacy_path.read_text(encoding="utf-8"))
                rows = payload.get("receipts") if isinstance(payload, Mapping) else None
                if not isinstance(rows, list):
                    raise ValueError("legacy operator receipt ledger has no receipts array")
                for raw_row in rows:
                    if not isinstance(raw_row, Mapping):
                        raise ValueError("legacy operator receipt ledger contains a non-object row")
                    row = dict(raw_row)
                    evidence = self._persist_evidence(row)
                    self._upsert(row, evidence=evidence)
                    self._register_evidence(row, evidence)
                    imported += 1
                pruned = self._prune_locked()
            self.connection.execute(
                "INSERT OR REPLACE INTO runtime_metadata(key,value,updated_at) VALUES(?,?,?)",
                (
                    "operator_receipt_legacy_import_v1",
                    _json_text({
                        "source": str(self.legacy_path),
                        "source_retained": True,
                        "imported_receipts": imported,
                    }),
                    time.time(),
                ),
            )
            self.connection.execute("COMMIT")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise RuntimeError(
                f"legacy operator receipt import failed for {self.legacy_path}"
            ) from exc
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise
        self._remove_pruned_evidence(pruned)

    def append_interrupt_fallback(
        self,
        receipt: Mapping[str, Any],
        *,
        reason: str,
    ) -> dict[str, Any]:
        """Fsync one safety receipt without waiting for SQLite."""
        row = dict(receipt)
        row["persistence_fallback"] = {
            "kind": "operator_interrupt_jsonl",
            "reason": str(reason)[:500],
            "recorded_at": time.time(),
        }
        compact, _ = self._compact_receipt(row)
        raw = _json_bytes(compact) + b"\n"
        lock_descriptor = os.open(self.interrupt_fallback_lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            os.fchmod(lock_descriptor, 0o600)
            fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
            descriptor = os.open(
                self.interrupt_fallback_path,
                os.O_APPEND | os.O_CREAT | os.O_WRONLY,
                0o600,
            )
            try:
                os.fchmod(descriptor, 0o600)
                written = os.write(descriptor, raw)
                if written != len(raw):
                    raise OSError(f"short operator interrupt fallback write: {written}/{len(raw)} bytes")
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            _fsync_directory(self.root)
        finally:
            fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
            os.close(lock_descriptor)
        return compact

    def _import_interrupt_fallback(self) -> None:
        lock_descriptor = os.open(self.interrupt_fallback_lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            os.fchmod(lock_descriptor, 0o600)
            fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
            if self.interrupt_fallback_path.exists():
                pending = self.root / f"operator_interrupt_fallback.pending.{time.time_ns()}.jsonl"
                os.replace(self.interrupt_fallback_path, pending)
                _fsync_directory(self.root)
        finally:
            fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
            os.close(lock_descriptor)

        pruned: list[str] = []
        pending_paths = sorted(self.root.glob("operator_interrupt_fallback.pending.*.jsonl"))
        for pending in pending_paths:
            try:
                receipts = [
                    json.loads(line)
                    for line in pending.read_text(encoding="utf-8").splitlines()
                    if line.strip()
                ]
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"operator interrupt fallback import failed for {pending}") from exc
            if not all(isinstance(row, Mapping) for row in receipts):
                raise RuntimeError("operator interrupt fallback contains a non-object row")
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                prepared = [(dict(row), self._persist_evidence(row)) for row in receipts]
                for row, evidence in prepared:
                    self._upsert(row, evidence=evidence)
                    self._register_evidence(row, evidence)
                pruned.extend(self._prune_locked())
                self.connection.execute("COMMIT")
            except Exception:
                self.connection.execute("ROLLBACK")
                raise
            archive = self.root / pending.name.replace(".pending.", ".imported.")
            os.replace(pending, archive)
            _fsync_directory(self.root)

        archives = sorted(
            self.root.glob("operator_interrupt_fallback.imported.*.jsonl"),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
        removed_archive = False
        for stale in archives[MAX_INTERRUPT_FALLBACK_ARCHIVES:]:
            stale.unlink()
            removed_archive = True
        if removed_archive:
            _fsync_directory(self.root)
        self._remove_pruned_evidence(pruned)

    def reconcile_nonterminal_receipts(self) -> int:
        """Delegate startup repair to the linked-projection CAS owner."""
        return self._audit_database.reconcile_nonterminal_claims()

    def claim(self, receipt: Mapping[str, Any]) -> tuple[dict[str, Any], bool]:
        """Atomically claim an idempotency key before normal dispatch."""
        row = dict(receipt)
        action_id = str(row.get("action_id") or "operator.action")
        payload = {
            "command_id": str(row.get("command_id") or ""),
            "idempotency_key": str(row.get("idempotency_key") or ""),
            "action_id": action_id,
            "operation": str(row.get("operation") or action_id),
            "entrypoint_id": str(row.get("entrypoint_id") or f"operator.{action_id}"),
            "caller_class": str(row.get("caller_class") or "operator"),
            "control_class": str(row.get("control_class") or row.get("safety_class") or "service"),
            "ownership_generation": int(row.get("ownership_generation") or 0),
            "source_identity": row.get("source_identity") or {"authority": "robot_runtime"},
            "requested_inputs": row.get("requested_inputs", row.get("inputs", {})) or {},
            "effective_inputs": row.get("effective_inputs", {}) or {},
            "safety_class": row.get("safety_class"),
            "idempotency_replay_enabled": row.get("idempotency_replay_enabled", True),
            "started_at": row.get("started_at"),
        }
        with self.lock:
            claimed, created = self._audit_database.claim(payload)
            stored = self.connection.execute(
                "SELECT * FROM operator_commands WHERE command_id=?",
                (claimed["command_id"],),
            ).fetchone()
            if stored is None:
                raise RuntimeError("operator claim missing after durable commit")
            return self._row_receipt(stored, include_evidence=False), created

    def put_interrupt(self, receipt: Mapping[str, Any]) -> dict[str, Any]:
        """Persist a delivered safety interrupt without waiting on normal DB work."""
        if not self.lock.acquire(blocking=False):
            return self.append_interrupt_fallback(receipt, reason="sqlite_connection_busy")
        try:
            self.connection.execute("PRAGMA busy_timeout=0")
            artifact_id = None
            try:
                self.connection.execute("BEGIN IMMEDIATE")
                evidence = self._persist_evidence(receipt)
                previous = self.connection.execute(
                    "SELECT evidence_relpath FROM operator_commands WHERE command_id=?",
                    (str(receipt.get("command_id") or ""),),
                ).fetchone()
                compact = self._upsert(receipt, evidence=evidence)
                artifact_id = self._register_evidence(receipt, evidence)
                if artifact_id is not None:
                    compact.update({"evidence_artifact_id": artifact_id, "evidence_relpath": evidence[0], "evidence_sha256": evidence[1], "evidence_bytes": evidence[2]})
                pruned = self._prune_locked()
                self.connection.execute("COMMIT")
                if (
                    previous is not None
                    and previous["evidence_relpath"]
                    and evidence[0]
                    and previous["evidence_relpath"] != evidence[0]
                ):
                    pruned.append(str(previous["evidence_relpath"]))
            except (OSError, RuntimeError, sqlite3.Error) as exc:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                return self.append_interrupt_fallback(
                    receipt,
                    reason=f"{type(exc).__name__}: {exc}",
                )
            finally:
                self.connection.execute("PRAGMA busy_timeout=2000")
        finally:
            self.lock.release()
        self._remove_pruned_evidence(pruned, nonblocking=True)
        return compact

    def _finalize_linked_pipette(
        self,
        *,
        receipt: Mapping[str, Any],
        expected_status: str | None,
        linked_finalization: Mapping[str, Any] | None,
    ) -> None:
        command_id = str(receipt.get("command_id") or "")
        operation = self.connection.execute(
            "SELECT pipette_operation_id,status FROM pipette_operations WHERE command_id=?",
            (command_id,),
        ).fetchone()
        if operation is None:
            if linked_finalization is not None:
                raise RuntimeError("linked pipette finalization has no child projection")
            return
        if expected_status is None or str(operation["status"]) != str(expected_status):
            raise RuntimeError("linked pipette child expected state is stale")

        if linked_finalization is not None:
            required = {
                "command_id",
                "pipette_operation_id",
                "expected_status",
                "status",
                "outcome",
                "failure_code",
                "result",
                "effective_inputs",
                "receipt",
                "normalized",
            }
            if set(linked_finalization) != required:
                raise RuntimeError("linked pipette finalization shape is invalid")
            if (
                str(linked_finalization["command_id"]) != command_id
                or str(linked_finalization["pipette_operation_id"])
                != str(operation["pipette_operation_id"])
                or str(linked_finalization["expected_status"]) != str(expected_status)
            ):
                raise RuntimeError("linked pipette finalization identity is invalid")
            child_result = linked_finalization["result"]
            child_receipt = linked_finalization["receipt"]
            normalized = linked_finalization["normalized"]
            if not all(
                isinstance(value, Mapping)
                for value in (child_result, child_receipt, normalized)
            ):
                raise RuntimeError("linked pipette finalization payload is invalid")
            child_status = str(linked_finalization["status"])
            outcome = str(linked_finalization["outcome"])
            failure_code = linked_finalization["failure_code"]
            effective_inputs = linked_finalization["effective_inputs"]
            if not isinstance(effective_inputs, Mapping):
                raise RuntimeError("linked pipette effective inputs are invalid")
        else:
            child_status = str(receipt.get("status") or "outcome_unknown")
            if child_status not in TERMINAL_STATES and child_status not in {
                "dispatched",
                "acknowledged",
            }:
                child_status = "outcome_unknown"
            outcome = str(
                receipt.get("physical_outcome")
                or receipt.get("outcome")
                or ("completed" if child_status in {"completed", "observed"} else child_status)
            )
            failure_code = receipt.get("failure_code") or receipt.get("error")
            child_result = {
                "ok": child_status in {"completed", "observed"}
                and receipt.get("machine_assessment") == "pass",
                "outcome": outcome,
                "error": failure_code,
                "delivery_verified": receipt.get("delivery_verified") is True,
                "controller_acknowledged": receipt.get("controller_acknowledged") is True,
                "completion_verified": receipt.get("completion_verified") is True,
                "hardware_precondition_verified": receipt.get("hardware_precondition_verified") is True,
                "hardware_postcondition_verified": receipt.get("hardware_postcondition_verified") is True,
                "physical_effect_verified": False,
                "retry_forbidden": child_status in {
                    "ambiguous",
                    "outcome_unknown",
                    "reconciliation_required",
                },
            }
            child_receipt = child_result
            normalized = None
            effective_inputs = {}

        self._audit_database.finalize_claim(
            command_id=command_id,
            pipette_operation_id=str(operation["pipette_operation_id"]),
            expected_status=str(expected_status),
            status=child_status,
            outcome=outcome,
            failure_code=None if failure_code is None else str(failure_code),
            result=dict(child_result),
            effective_inputs=dict(effective_inputs),
            receipt_json=_json_text(child_receipt),
        )
        if normalized is not None:
            self._audit_database.persist_normalized_pipette_result(
                command_id=command_id,
                pipette_operation_id=str(operation["pipette_operation_id"]),
                result=dict(child_result),
                normalized=dict(normalized),
            )

    def put(
        self,
        receipt: Mapping[str, Any],
        *,
        _expected_status: str | None = None,
        _reconciliation_transition: bool = False,
        _linked_pipette_finalization: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        row = dict(receipt)
        with self.lock:
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                evidence = self._persist_evidence(row) if row.get("status") in TERMINAL_STATES else (None, None, None)
                artifact_id = None
                previous = self.connection.execute(
                    "SELECT evidence_relpath FROM operator_commands WHERE command_id=?",
                    (str(row.get("command_id") or ""),),
                ).fetchone()
                if not _reconciliation_transition:
                    self._finalize_linked_pipette(
                        receipt=row,
                        expected_status=_expected_status,
                        linked_finalization=_linked_pipette_finalization,
                    )
                compact = self._upsert(
                    row,
                    evidence=evidence,
                    expected_status=_expected_status,
                    reconciliation_transition=_reconciliation_transition,
                )
                artifact_id = self._register_evidence(row, evidence)
                if artifact_id is not None:
                    compact.update({"evidence_artifact_id": artifact_id, "evidence_relpath": evidence[0], "evidence_sha256": evidence[1], "evidence_bytes": evidence[2]})
                pruned = self._prune_locked()
                self.connection.execute("COMMIT")
                if (
                    previous is not None
                    and previous["evidence_relpath"]
                    and evidence[0]
                    and previous["evidence_relpath"] != evidence[0]
                ):
                    pruned.append(str(previous["evidence_relpath"]))
            except Exception:
                self.connection.execute("ROLLBACK")
                raise
        self._remove_pruned_evidence(pruned)
        return compact

    def reconcile(self, receipt: Mapping[str, Any], *, expected_status: str) -> dict[str, Any]:
        """Persist an explicit reconciliation transition from an exact protected state."""
        return self.put(
            receipt,
            _expected_status=expected_status,
            _reconciliation_transition=True,
        )

    def assess(
        self,
        command_id: str,
        *,
        expected_generation: int,
        verdict: str,
        note: str,
        idempotency_key: str,
        legal_hold: bool | None = None,
        actor: str | None = None,
    ) -> dict[str, Any]:
        """Append an operator assessment without republishing command status."""
        if verdict not in {"pass", "fail"}:
            raise ValueError("operator assessment verdict must be pass or fail")
        named_actor = None if actor is None else str(actor).strip()
        if named_actor is not None and not named_actor:
            raise ValueError("legal-hold assessment requires a named actor")
        if (legal_hold is None) != (named_actor is None):
            raise ValueError("legal hold and named actor must be supplied together")
        if legal_hold is not None and type(legal_hold) is not bool:
            raise TypeError("legal_hold must be an exact boolean")
        assessment = {
            "operator_assessment": verdict,
            "operator_note": str(note).strip(),
            "operator_assessment_idempotency_key": str(idempotency_key),
            "operator_assessment_generation": int(expected_generation),
            "legal_hold": legal_hold,
            "legal_hold_actor": named_actor,
        }
        with self.lock:
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                row = self.connection.execute(
                    "SELECT * FROM operator_commands WHERE command_id=?",
                    (str(command_id),),
                ).fetchone()
                if row is None:
                    raise KeyError("operator action receipt not found")
                if int(row["ownership_generation"] or 0) != int(expected_generation):
                    raise RuntimeError("operator assessment ownership generation mismatch")
                existing = self.connection.execute(
                    """
                    SELECT detail_json FROM operator_transitions
                    WHERE command_id=?
                      AND json_extract(detail_json,'$.operator_assessment_idempotency_key')=?
                    ORDER BY transition_id DESC LIMIT 1
                    """,
                    (str(command_id), str(idempotency_key)),
                ).fetchone()
                if existing is not None:
                    prior = json.loads(str(existing["detail_json"]))
                    for key, value in assessment.items():
                        if prior.get(key) != value:
                            raise RuntimeError("operator assessment idempotency conflict")
                    self.connection.execute("COMMIT")
                    return self._row_receipt(row, include_evidence=False)
                now = time.time()
                detail = {**assessment, "operator_assessed_at": now}
                self.connection.execute(
                    "INSERT INTO operator_transitions(command_id,state,observed_at,detail_json) VALUES(?,?,?,?)",
                    (str(command_id), str(row["status"]), now, _json_text(detail)),
                )
                hold_result = None
                if legal_hold is not None and named_actor is not None:
                    hold_result = self._apply_legal_hold_assessment_locked(
                        str(command_id),
                        legal_hold=legal_hold,
                        actor=named_actor,
                        assessment=detail,
                        observed_at=now,
                    )
                verified = int(row["physical_effect_verified"] or 0)
                if verdict == "fail":
                    verified = 0
                elif str(row["safety_class"] or "") == "motion":
                    verified = 1
                updated = self.connection.execute(
                    """
                    UPDATE operator_commands
                    SET physical_effect_verified=?,updated_at=?
                    WHERE command_id=? AND status=? AND ownership_generation=?
                    """,
                    (
                        verified,
                        now,
                        str(command_id),
                        str(row["status"]),
                        int(expected_generation),
                    ),
                )
                if updated.rowcount != 1:
                    raise RuntimeError("operator assessment state changed during persistence")
                stored = self.connection.execute(
                    "SELECT * FROM operator_commands WHERE command_id=?",
                    (str(command_id),),
                ).fetchone()
                self.connection.execute("COMMIT")
                if stored is None:
                    raise RuntimeError("operator assessment projection disappeared")
                response = self._row_receipt(stored, include_evidence=False)
                if hold_result is not None:
                    response["legal_hold_assessment"] = hold_result
                return response
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise

    def _row_receipt(self, row: sqlite3.Row, *, include_evidence: bool) -> dict[str, Any]:
        receipt = json.loads(row["receipt_json"])
        receipt["status"] = str(row["status"])
        receipt["controller_acknowledged"] = bool(row["controller_acknowledged"])
        receipt["physical_effect_verified"] = bool(row["physical_effect_verified"])
        if receipt["status"] not in TERMINAL_STATES:
            receipt["retry_forbidden"] = True
            receipt.setdefault("outcome", "in_progress")
        if row["outcome"] is not None:
            receipt["outcome"] = row["outcome"]
        if row["failure_code"] is not None:
            receipt["failure_code"] = row["failure_code"]
        summary = None if row["response_summary_json"] is None else json.loads(row["response_summary_json"])
        receipt["response"] = summary
        receipt["stage_receipts"] = []
        if include_evidence and row["evidence_relpath"]:
            try:
                raw, _, _ = _read_confined_evidence(
                    self.root,
                    str(row["evidence_relpath"]),
                    expected_sha256=str(row["evidence_sha256"]),
                    expected_bytes=int(row["evidence_bytes"]),
                )
                evidence = json.loads(raw)
                receipt["response"] = evidence.get("response")
                receipt["stage_receipts"] = evidence.get("stage_receipts") or []
            except (OSError, RuntimeError, ValueError, json.JSONDecodeError):
                receipt["response"] = {
                    "summary": summary,
                    "evidence_unavailable": True,
                    "evidence_state": "integrity_failed",
                    "error_code": "operator_evidence_integrity_failure",
                }
        response = receipt.get("response")
        body = response.get("body") if isinstance(response, Mapping) else None
        detail = body.get("detail") if isinstance(body, Mapping) else None
        authority = None
        if isinstance(body, Mapping):
            authority = body.get("authority_receipt")
        if not isinstance(authority, Mapping) and isinstance(detail, Mapping):
            authority = detail.get("authority_receipt")
        if (
            isinstance(authority, Mapping)
            and authority.get("command_id") == receipt.get("command_id")
            and str(receipt.get("action_id") or "").startswith("oem.x.")
        ):
            receipt["authority_receipt_id"] = receipt.get("authority_receipt_id") or receipt["command_id"]
            if not isinstance(receipt.get("authority_receipt_status"), str):
                receipt["authority_receipt_status"] = receipt.get("status")
        assessment = self.connection.execute(
            """
            SELECT detail_json FROM operator_transitions
            WHERE command_id=? AND json_type(detail_json,'$.operator_assessment')='text'
            ORDER BY transition_id DESC LIMIT 1
            """,
            (str(row["command_id"]),),
        ).fetchone()
        if assessment is not None:
            detail = json.loads(str(assessment["detail_json"]))
            for key in (
                "operator_assessment",
                "operator_note",
                "operator_assessment_idempotency_key",
                "operator_assessed_at",
            ):
                if key in detail:
                    receipt[key] = detail[key]
        return receipt

    def list(self, limit: int = 100, *, include_evidence: bool = False) -> list[dict[str, Any]]:
        selected_limit = max(1, min(int(limit), 200))
        with self.lock:
            rows = self.connection.execute(
                "SELECT * FROM operator_commands ORDER BY updated_at DESC, sequence DESC LIMIT ?",
                (selected_limit,),
            ).fetchall()
            return [self._row_receipt(row, include_evidence=include_evidence) for row in rows]

    def by_command(self, command_id: str, *, include_evidence: bool = True) -> dict[str, Any] | None:
        with self.lock:
            row = self.connection.execute(
                "SELECT * FROM operator_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            return None if row is None else self._row_receipt(row, include_evidence=include_evidence)

    def by_idempotency(self, key: str, *, include_evidence: bool = True) -> dict[str, Any] | None:
        with self.lock:
            row = self.connection.execute(
                """
                SELECT * FROM operator_commands
                WHERE idempotency_key=? AND idempotency_replay_enabled=1
                """,
                (key,),
            ).fetchone()
            return None if row is None else self._row_receipt(row, include_evidence=include_evidence)
