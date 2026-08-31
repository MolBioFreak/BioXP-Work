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
import tempfile
import threading
import time
import uuid
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .runtime_audit_store import RuntimeAuditDatabase, runtime_state_root as _canonical_runtime_state_root

TERMINAL_STATES = frozenset({
    "completed",
    "failed",
    "rejected",
    "ambiguous",
    "cancelled",
    "reconciliation_required",
})
NONREPLAYABLE_INTERRUPT_ACTIONS = frozenset({
    "meta.emergency_stop",
    "oem.y.stop",
    "oem.z.stop",
    "oem.x.stop",
    "oem.abort_all",
})
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
    "controller_acknowledged",
    "remote_acknowledged",
    "physical_effect_verified",
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

    def __init__(self, root: str | Path | None = None) -> None:
        self.root = runtime_state_root(root)
        self.path = self.root / "bioxp_runtime.db"
        self.evidence_root = self.root / "operator_evidence"
        self.interrupt_fallback_path = self.root / "operator_interrupt_fallback.jsonl"
        self.interrupt_fallback_lock_path = self.root / "operator_interrupt_fallback.lock"
        _ensure_durable_directory(self.evidence_root)
        os.chmod(self.evidence_root, 0o700)
        self.legacy_path = self.root / "operator_action_receipts.json"
        fallback_legacy = Path("/tmp/bioxp-oem-runtime/operator_action_receipts.json")
        if not self.legacy_path.exists() and fallback_legacy.exists():
            self.legacy_path = fallback_legacy
        self.lock = threading.RLock()
        self._audit_database = RuntimeAuditDatabase(root=self.root)
        self.connection = self._audit_database.connection
        with self.lock:
            self._configure()
            self._create_schema()
            _fsync_directory(self.root)
            self._import_legacy_once()
            self._import_interrupt_fallback()
            self._remove_orphan_evidence()

    def _configure(self) -> None:
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=FULL")
        self.connection.execute("PRAGMA foreign_keys=ON")
        self.connection.execute("PRAGMA busy_timeout=2000")
        self.connection.execute("PRAGMA wal_autocheckpoint=256")
        self.connection.execute("PRAGMA journal_size_limit=4194304")
        self.connection.execute("PRAGMA temp_store=MEMORY")

    def _create_schema(self) -> None:
        try:
            self.connection.executescript(
                """
                BEGIN IMMEDIATE;
                CREATE TABLE IF NOT EXISTS runtime_metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL,
                    updated_at REAL NOT NULL
                ) WITHOUT ROWID;
                CREATE TABLE IF NOT EXISTS operator_commands (
                    sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                    command_id TEXT NOT NULL UNIQUE,
                    idempotency_key TEXT NOT NULL,
                    idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1
                        CHECK(idempotency_replay_enabled IN (0,1)),
                    action_id TEXT NOT NULL,
                    status TEXT NOT NULL,
                    safety_class TEXT,
                    ownership_generation INTEGER NOT NULL,
                    started_at TEXT NOT NULL,
                    finished_at TEXT,
                    duration_ms REAL,
                    controller_acknowledged INTEGER NOT NULL DEFAULT 0 CHECK(controller_acknowledged IN (0,1)),
                    physical_effect_verified INTEGER NOT NULL DEFAULT 0 CHECK(physical_effect_verified IN (0,1)),
                    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
                    response_summary_json TEXT CHECK(response_summary_json IS NULL OR json_valid(response_summary_json)),
                    evidence_relpath TEXT,
                    evidence_sha256 TEXT,
                    evidence_bytes INTEGER,
                    updated_at REAL NOT NULL
                );
                CREATE INDEX IF NOT EXISTS operator_commands_history_idx
                    ON operator_commands(sequence DESC);
                CREATE INDEX IF NOT EXISTS operator_commands_updated_idx
                    ON operator_commands(updated_at DESC, sequence DESC);
                CREATE INDEX IF NOT EXISTS operator_commands_action_status_idx
                    ON operator_commands(action_id, status, sequence DESC);
                CREATE TABLE IF NOT EXISTS operator_transitions (
                    transition_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    command_id TEXT NOT NULL REFERENCES operator_commands(command_id) ON DELETE CASCADE,
                    state TEXT NOT NULL,
                    observed_at REAL NOT NULL,
                    detail_json TEXT CHECK(detail_json IS NULL OR json_valid(detail_json))
                );
                CREATE INDEX IF NOT EXISTS operator_transitions_command_idx
                    ON operator_transitions(command_id, transition_id);
                """
            )
            columns = {
                str(row["name"])
                for row in self.connection.execute("PRAGMA table_info(operator_commands)")
            }
            if "idempotency_replay_enabled" not in columns:
                self.connection.execute(
                    """
                    ALTER TABLE operator_commands
                    ADD COLUMN idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1
                        CHECK(idempotency_replay_enabled IN (0,1))
                    """
                )
            placeholders = ",".join("?" for _ in NONREPLAYABLE_INTERRUPT_ACTIONS)
            self.connection.execute(
                f"""
                UPDATE operator_commands
                SET idempotency_replay_enabled=0
                WHERE action_id IN ({placeholders})
                """,
                tuple(NONREPLAYABLE_INTERRUPT_ACTIONS),
            )
            for index in self.connection.execute("PRAGMA index_list(operator_commands)").fetchall():
                if not index["unique"] or index["origin"] != "c" or index["partial"]:
                    continue
                name = str(index["name"])
                indexed_columns = [
                    str(row["name"])
                    for row in self.connection.execute(
                        "SELECT name FROM pragma_index_info(?)",
                        (name,),
                    )
                ]
                if indexed_columns == ["idempotency_key"]:
                    drop_row = self.connection.execute(
                        "SELECT printf('DROP INDEX \"%w\"', ?)",
                        (name,),
                    ).fetchone()
                    if drop_row is None or not isinstance(drop_row[0], str):
                        raise RuntimeError("operator_index_drop_statement_missing")
                    self.connection.execute(drop_row[0])
            self.connection.execute(
                """
                CREATE UNIQUE INDEX IF NOT EXISTS operator_commands_replay_key_idx
                    ON operator_commands(idempotency_key)
                    WHERE idempotency_replay_enabled=1
                """
            )
            self.connection.execute("PRAGMA user_version=2")
            self.connection.execute("COMMIT")
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

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
                existing = path.read_bytes()
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
        deadline = receipt.get("evidence_retention_deadline")
        if deadline is None:
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
                    int(bool(receipt.get("legal_hold", False))),
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

    def expire_evidence(
        self,
        command_id: str,
        *,
        retention_deadline: float | None = None,
        now: float | None = None,
        legal_hold: bool | None = None,
    ) -> dict[str, Any]:
        current_time = time.time() if now is None else float(now)
        with self.lock:
            row = self.connection.execute(
                """
                SELECT o.* FROM runtime_evidence_objects o
                WHERE o.command_id=? ORDER BY o.created_at DESC LIMIT 1
                """,
                (str(command_id),),
            ).fetchone()
            if row is None:
                return {"state": "no_evidence", "command_id": str(command_id)}
            deadline = float(row["retention_deadline"] if retention_deadline is None else retention_deadline)
            held = bool(row["legal_hold"] if legal_hold is None else legal_hold)
            if held:
                return {"state": "legal_hold", "command_id": str(command_id), "evidence_artifact_id": row["evidence_artifact_id"]}
            if current_time < deadline:
                return {"state": "retained", "command_id": str(command_id), "evidence_artifact_id": row["evidence_artifact_id"]}
            artifact_id = str(row["evidence_artifact_id"])
            relpath = row["active_relpath"]
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                self.connection.execute(
                    "UPDATE runtime_evidence_objects SET expiry_state='expiry_pending',updated_at=? WHERE evidence_artifact_id=?",
                    (current_time, artifact_id),
                )
                self.connection.execute(
                    "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                    (artifact_id, "expiry_pending", current_time, _json_text({"command_id": command_id})),
                )
                self.connection.execute("COMMIT")
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise
        if relpath:
            path = self.root / str(relpath)
            try:
                path.unlink()
                _fsync_directory(path.parent)
            except FileNotFoundError:
                pass
            except Exception as exc:
                with self.lock:
                    self.connection.execute("BEGIN IMMEDIATE")
                    try:
                        self.connection.execute(
                            "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                            (artifact_id, "integrity_failure", current_time, _json_text({"error": str(exc)[:500]})),
                        )
                        self.connection.execute("COMMIT")
                    except Exception:
                        if self.connection.in_transaction:
                            self.connection.execute("ROLLBACK")
                    raise
        expiry_receipt_id = uuid.uuid4().hex
        with self.lock:
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                self.connection.execute(
                    "UPDATE runtime_evidence_objects SET active_relpath=NULL,expiry_state='expired',expiry_receipt_id=?,updated_at=? WHERE evidence_artifact_id=?",
                    (expiry_receipt_id, current_time, artifact_id),
                )
                self.connection.execute(
                    "UPDATE operator_commands SET evidence_relpath=NULL,evidence_state='expired',updated_at=? WHERE command_id=?",
                    (current_time, str(command_id)),
                )
                for event_kind in ("deleted", "expired"):
                    self.connection.execute(
                        "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                        (artifact_id, event_kind, current_time, _json_text({"expiry_receipt_id": expiry_receipt_id})),
                    )
                self.connection.execute("COMMIT")
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise
        return {"state": "expired", "command_id": str(command_id), "evidence_artifact_id": artifact_id, "expiry_receipt_id": expiry_receipt_id}

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
        relpath, digest, size = evidence
        if relpath is None and previous is not None:
            existing = self.connection.execute(
                "SELECT evidence_relpath,evidence_sha256,evidence_bytes FROM operator_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if existing is not None:
                relpath, digest, size = existing
        now = time.time()
        self.connection.execute(
            """
            INSERT INTO operator_commands(
                command_id,idempotency_key,idempotency_replay_enabled,action_id,status,safety_class,
                ownership_generation,started_at,finished_at,duration_ms,
                controller_acknowledged,physical_effect_verified,receipt_json,
                response_summary_json,evidence_relpath,evidence_sha256,evidence_bytes,updated_at
            ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            ON CONFLICT(command_id) DO UPDATE SET
                sequence=(SELECT COALESCE(MAX(sequence),0)+1 FROM operator_commands),
                idempotency_replay_enabled=excluded.idempotency_replay_enabled,
                status=excluded.status,
                finished_at=excluded.finished_at,
                duration_ms=excluded.duration_ms,
                controller_acknowledged=excluded.controller_acknowledged,
                physical_effect_verified=excluded.physical_effect_verified,
                receipt_json=excluded.receipt_json,
                response_summary_json=excluded.response_summary_json,
                evidence_relpath=COALESCE(excluded.evidence_relpath,operator_commands.evidence_relpath),
                evidence_sha256=COALESCE(excluded.evidence_sha256,operator_commands.evidence_sha256),
                evidence_bytes=COALESCE(excluded.evidence_bytes,operator_commands.evidence_bytes),
                updated_at=excluded.updated_at
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
                int(receipt.get("controller_acknowledged") is True),
                int(receipt.get("physical_effect_verified") is True),
                _json_text(compact),
                None if response_summary is None else _json_text(response_summary),
                relpath,
                digest,
                size,
                now,
            ),
        )
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
        changed_directories: set[Path] = set()
        for relpath in relpaths:
            referenced = self.connection.execute(
                "SELECT 1 FROM operator_commands WHERE evidence_relpath=? LIMIT 1",
                (relpath,),
            ).fetchone()
            if referenced is not None:
                continue
            path = self.root / relpath
            try:
                path.unlink()
            except FileNotFoundError:
                continue
            changed_directories.add(path.parent)
        for directory in changed_directories:
            _fsync_directory(directory)
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
        self.connection.execute("BEGIN IMMEDIATE")
        try:
            referenced = {
                str(row["evidence_relpath"])
                for row in self.connection.execute(
                    "SELECT evidence_relpath FROM operator_commands WHERE evidence_relpath IS NOT NULL"
                ).fetchall()
            }
            stale: list[str] = []
            for path in self.evidence_root.rglob("*"):
                if not path.is_file():
                    continue
                relpath = path.relative_to(self.root).as_posix()
                if relpath not in referenced:
                    stale.append(relpath)
            self._remove_evidence_files_locked(stale)
            directories = sorted(
                (path for path in self.evidence_root.rglob("*") if path.is_dir()),
                key=lambda path: len(path.parts),
                reverse=True,
            )
            for directory in directories:
                try:
                    directory.rmdir()
                except OSError:
                    continue
                try:
                    _fsync_directory(directory.parent)
                except OSError:
                    continue
            self.connection.execute("COMMIT")
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

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
        """Fail stale claimed commands closed after process startup."""
        with self.lock:
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                rows = self.connection.execute(
                    "SELECT receipt_json FROM operator_commands WHERE status IN ('reserved','queued','executing')"
                ).fetchall()
                if not rows:
                    self.connection.execute("COMMIT")
                    return 0
                now = time.time()
                for selected in rows:
                    receipt = json.loads(selected["receipt_json"])
                    try:
                        started_at = float(receipt.get("started_at") or now)
                    except (TypeError, ValueError, OverflowError):
                        started_at = now
                    receipt.update({
                        "status": "reconciliation_required",
                        "automatic_retry": False,
                        "physical_outcome": "ambiguous",
                        "finished_at": str(now),
                        "duration_ms": max(0.0, (now - started_at) * 1000.0),
                        "response": {
                            "status_code": 409,
                            "body": {
                                "error": "handler_restarted_after_command_claim",
                                "automatic_retry": False,
                                "physical_outcome": "ambiguous",
                            },
                        },
                    })
                    evidence = self._persist_evidence(receipt)
                    self._upsert(receipt, evidence=evidence)
                    self._register_evidence(receipt, evidence)
                self.connection.execute("COMMIT")
            except Exception:
                self.connection.execute("ROLLBACK")
                raise
        return len(rows)

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

    def put(self, receipt: Mapping[str, Any]) -> dict[str, Any]:
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
                compact = self._upsert(row, evidence=evidence)
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

    def _row_receipt(self, row: sqlite3.Row, *, include_evidence: bool) -> dict[str, Any]:
        receipt = json.loads(row["receipt_json"])
        summary = None if row["response_summary_json"] is None else json.loads(row["response_summary_json"])
        receipt["response"] = summary
        receipt["stage_receipts"] = []
        if include_evidence and row["evidence_relpath"]:
            path = self.root / str(row["evidence_relpath"])
            try:
                raw = path.read_bytes()
                if hashlib.sha256(raw).hexdigest() != row["evidence_sha256"]:
                    raise ValueError("operator evidence digest mismatch")
                evidence = json.loads(raw)
                receipt["response"] = evidence.get("response")
                receipt["stage_receipts"] = evidence.get("stage_receipts") or []
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                receipt["response"] = {
                    "summary": summary,
                    "evidence_unavailable": f"{type(exc).__name__}: {exc}"[:500],
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
        return receipt

    def list(
        self,
        limit: int = 100,
        *,
        include_evidence: bool = False,
        before_sequence: int | None = None,
    ) -> list[dict[str, Any]]:
        selected_limit = max(1, min(int(limit), 200))
        with self.lock:
            if before_sequence is None:
                rows = self.connection.execute(
                    "SELECT * FROM operator_commands ORDER BY sequence DESC LIMIT ?",
                    (selected_limit,),
                ).fetchall()
            else:
                rows = self.connection.execute(
                    "SELECT * FROM operator_commands WHERE sequence<? ORDER BY sequence DESC LIMIT ?",
                    (int(before_sequence), selected_limit),
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


_LEGACY_COMMAND_COLUMNS = frozenset({
    "command_id", "stream_sequence", "method_id", "method_sequence", "action_id",
    "requested_json", "effective_json", "status", "version", "ownership_generation",
    "queued_at", "dispatched_at", "finished_at", "source_noop", "source_noop_reason",
    "remote_acknowledged", "controller_acknowledged", "physical_effect_verified", "terminal_json",
})
_LEGACY_METHOD_COLUMNS = frozenset({
    "method_id", "name", "digest", "failure_policy", "status", "version",
    "ownership_generation", "expanded_count", "first_stream_sequence",
    "last_stream_sequence", "queued_at", "updated_at",
})
_LEGACY_NONTERMINAL_COMMAND_STATES = frozenset({
    "queued", "dispatched", "issued_pending", "stop_requested", "abort_requested",
})
_LEGACY_NONTERMINAL_METHOD_STATES = frozenset({
    "queued", "running", "pause_requested", "paused", "cancel_requested",
    "stopping", "aborting", "recovery_required",
})


class OperatorHistoryReader:
    """Read-only projection over the retired operator-plane tables."""

    def __init__(self, root: str | Path | None = None) -> None:
        self.root = runtime_state_root(root)
        self.path = self.root / "bioxp_runtime.db"
        self.lock = threading.RLock()
        self.connection: sqlite3.Connection | None = None
        if self.path.is_file():
            connection = sqlite3.connect(
                f"{self.path.as_uri()}?mode=ro", uri=True, timeout=2.0,
                isolation_level=None, check_same_thread=False,
            )
            connection.row_factory = sqlite3.Row
            connection.execute("PRAGMA query_only=ON")
            self.connection = connection

    def _table_columns(self, table: str) -> set[str] | None:
        if self.connection is None:
            return None
        exists = self.connection.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (table,)
        ).fetchone()
        if exists is None:
            return None
        return {str(row["name"]) for row in self.connection.execute(f'PRAGMA table_info("{table}")')}

    def _require_schema(self, table: str, required: frozenset[str]) -> bool:
        columns = self._table_columns(table)
        if columns is None:
            return False
        missing = sorted(required - columns)
        if missing:
            raise RuntimeError(
                f"unsupported legacy operator history schema: {table} missing {','.join(missing)}"
            )
        return True

    @staticmethod
    def _load_json(value: Any, default: Any) -> Any:
        if value is None:
            return default
        try:
            return json.loads(str(value))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError("unsupported legacy operator history schema: invalid JSON") from exc

    def _canonical_command(self, command_id: str) -> sqlite3.Row | None:
        if self.connection is None:
            return None
        columns = self._table_columns("serial206_movement_commands")
        if columns is None:
            return None
        required = {
            "command_id", "sequence", "state", "state_version",
            "expected_board_epochs_json", "terminal_receipt_id",
        }
        if not required.issubset(columns):
            raise RuntimeError("unsupported legacy operator history schema: serial206_movement_commands")
        return self.connection.execute(
            "SELECT sequence,state,state_version,expected_board_epochs_json,terminal_receipt_id "
            "FROM serial206_movement_commands WHERE command_id=?", (command_id,),
        ).fetchone()

    def _command_projection(self, row: sqlite3.Row) -> dict[str, Any]:
        command_id = str(row["command_id"])
        canonical = self._canonical_command(command_id)
        stored_status = str(canonical["state"]) if canonical is not None else str(row["status"])
        nonterminal = stored_status in _LEGACY_NONTERMINAL_COMMAND_STATES
        terminal = self._load_json(row["terminal_json"], None)
        transition_sequence = None
        transition_columns = self._table_columns("operator_plane_transitions")
        if transition_columns is not None:
            if not {"transition_sequence", "command_id"}.issubset(transition_columns):
                raise RuntimeError("unsupported legacy operator history schema: operator_plane_transitions")
            selected = self.connection.execute(
                "SELECT MAX(transition_sequence) FROM operator_plane_transitions WHERE command_id=?",
                (command_id,),
            ).fetchone()
            transition_sequence = selected[0] if selected and selected[0] is not None else None
        return {
            "schema_version": "bioxp.operator_command_receipt.v1",
            "source": "legacy_operator_plane",
            "command_id": command_id,
            "method_id": row["method_id"],
            "method_sequence": row["method_sequence"],
            "stream_sequence": int(row["stream_sequence"]),
            "action_id": str(row["action_id"]),
            "status": "ambiguous" if nonterminal else stored_status,
            "stored_status": stored_status,
            "recovery_required": nonterminal,
            "automatic_retry": False,
            "physical_outcome": "ambiguous" if nonterminal else None,
            "ownership_generation": int(row["ownership_generation"]),
            "requested_inputs": self._load_json(row["requested_json"], {}),
            "effective_inputs": self._load_json(row["effective_json"], {}),
            "accepted_at": float(row["queued_at"]),
            "queued_at": float(row["queued_at"]),
            "dispatched_at": row["dispatched_at"],
            "finished_at": row["finished_at"],
            "source_noop": bool(row["source_noop"]),
            "source_noop_reason": row["source_noop_reason"],
            "remote_acknowledged": bool(row["remote_acknowledged"]),
            "controller_acknowledged": bool(row["controller_acknowledged"]),
            "physical_effect_verified": bool(row["physical_effect_verified"]),
            "terminal_evidence": terminal,
            "sequence": int(canonical["sequence"]) if canonical is not None else int(row["stream_sequence"]),
            "state_version": int(canonical["state_version"]) if canonical is not None else int(row["version"]),
            "expected_board_epoch_by_board": self._load_json(canonical["expected_board_epochs_json"], {}) if canonical is not None else {},
            "terminal_receipt_id": canonical["terminal_receipt_id"] if canonical is not None else None,
            "completion_class": terminal.get("completion_class") if isinstance(terminal, Mapping) else None,
            "transition_sequence": transition_sequence,
        }

    def get_command(self, command_id: str) -> dict[str, Any] | None:
        with self.lock:
            if self.connection is None or not self._require_schema("operator_plane_commands", _LEGACY_COMMAND_COLUMNS):
                return None
            row = self.connection.execute(
                "SELECT * FROM operator_plane_commands WHERE command_id=?", (str(command_id),)
            ).fetchone()
            return None if row is None else self._command_projection(row)

    def list_commands(
        self, *, limit: int = 100, before_sequence: int | None = None,
        exclude_command_ids: set[str] | frozenset[str] | None = None,
    ) -> list[dict[str, Any]]:
        bounded = min(max(int(limit), 1), 200)
        excluded = {str(value) for value in (exclude_command_ids or set())}
        with self.lock:
            if self.connection is None or not self._require_schema("operator_plane_commands", _LEGACY_COMMAND_COLUMNS):
                return []
            clauses: list[str] = []
            parameters: list[Any] = []
            if before_sequence is not None:
                clauses.append("stream_sequence<?")
                parameters.append(int(before_sequence))
            if excluded:
                clauses.append(f"command_id NOT IN ({','.join('?' for _ in excluded)})")
                parameters.extend(sorted(excluded))
            where = f" WHERE {' AND '.join(clauses)}" if clauses else ""
            parameters.append(bounded)
            rows = self.connection.execute(
                f"SELECT * FROM operator_plane_commands{where} ORDER BY stream_sequence DESC LIMIT ?",
                tuple(parameters),
            ).fetchall()
            return [self._command_projection(row) for row in rows]

    def command_detail_v2(self, command_id: str) -> dict[str, Any] | None:
        projection = self.get_command(command_id)
        if projection is None or self.connection is None:
            return projection
        with self.lock:
            transitions: list[dict[str, Any]] = []
            columns = self._table_columns("operator_plane_transitions")
            if columns is not None:
                required = {"transition_sequence", "command_id", "state", "payload_json", "created_at"}
                if not required.issubset(columns):
                    raise RuntimeError("unsupported legacy operator history schema: operator_plane_transitions")
                rows = self.connection.execute(
                    "SELECT transition_sequence,state,payload_json,created_at FROM operator_plane_transitions "
                    "WHERE command_id=? ORDER BY transition_sequence LIMIT 200", (str(command_id),),
                ).fetchall()
                previous = None
                for row in rows:
                    transitions.append({
                        "transition_id": str(row["transition_sequence"]), "from_status": previous,
                        "to_status": str(row["state"]), "at": float(row["created_at"]),
                        "reason": (self._load_json(row["payload_json"], {}) or {}).get("reason"),
                    })
                    previous = str(row["state"])
            terminal = projection.get("terminal_evidence")
            response = terminal.get("response") if isinstance(terminal, Mapping) else None
            response = dict(response) if isinstance(response, Mapping) else (dict(terminal) if isinstance(terminal, Mapping) else {})
            projection.update({
                "canonical_inputs": dict(projection.get("requested_inputs") or {}),
                "requested_values": dict(projection.get("requested_inputs") or {}),
                "effective_values": dict(projection.get("effective_inputs") or {}),
                "observed_values": dict(terminal.get("observed_values") or {}) if isinstance(terminal, Mapping) else {},
                "raw_return_layers": dict(response.get("raw_return_layers") or {}),
                "controller_evidence": dict(response.get("controller_evidence") or {}),
                "transport_artifacts": list(terminal.get("transport_artifacts") or []) if isinstance(terminal, Mapping) else [],
                "child_receipts": list(terminal.get("child_receipts") or []) if isinstance(terminal, Mapping) else [],
                "transitions": transitions,
            })
            return projection

    def get_method(self, method_id: str) -> dict[str, Any] | None:
        with self.lock:
            if self.connection is None or not self._require_schema("operator_plane_methods", _LEGACY_METHOD_COLUMNS):
                return None
            row = self.connection.execute(
                "SELECT * FROM operator_plane_methods WHERE method_id=?", (str(method_id),)
            ).fetchone()
            if row is None:
                return None
            stored_status = str(row["status"])
            nonterminal = stored_status in _LEGACY_NONTERMINAL_METHOD_STATES
            return {
                "schema_version": "bioxp.operator_method_receipt.v1",
                "source": "legacy_operator_plane",
                "method_id": str(row["method_id"]), "name": str(row["name"]),
                "method_digest": str(row["digest"]),
                "status": "ambiguous" if nonterminal else stored_status,
                "stored_status": stored_status, "recovery_required": nonterminal,
                "automatic_retry": False, "version": int(row["version"]),
                "ownership_generation": int(row["ownership_generation"]),
                "expanded_count": int(row["expanded_count"]),
                "first_stream_sequence": row["first_stream_sequence"],
                "last_stream_sequence": row["last_stream_sequence"],
                "queued_at": float(row["queued_at"]), "updated_at": float(row["updated_at"]),
                "failure_policy": str(row["failure_policy"]),
            }

    def list_method_commands(self, method_id: str) -> list[dict[str, Any]]:
        with self.lock:
            if self.connection is None or not self._require_schema("operator_plane_commands", _LEGACY_COMMAND_COLUMNS):
                return []
            rows = self.connection.execute(
                "SELECT * FROM operator_plane_commands WHERE method_id=? "
                "ORDER BY method_sequence,stream_sequence", (str(method_id),),
            ).fetchall()
            return [self._command_projection(row) for row in rows]

    def close(self) -> None:
        with self.lock:
            if self.connection is not None:
                self.connection.close()
                self.connection = None
