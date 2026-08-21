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
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from .oem_runtime_store import migrate_runtime_database_v2

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
    "oem.z.stop",
    "oem.y.stop",
    "oem.z.abort",
    "oem.x.stop",
    "oem.abort_all",
})
MAX_OPERATOR_COMMANDS = 512
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
    selected = (
        root
        or os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT")
        or os.environ.get("BIOXP_OEM_RUNTIME_ROOT")
        or "/tmp/bioxp-oem-runtime"
    )
    path = Path(selected)
    _ensure_durable_directory(path)
    os.chmod(path, 0o700)
    return path


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
        self.connection = sqlite3.connect(
            self.path,
            timeout=2.0,
            isolation_level=None,
            check_same_thread=False,
        )
        self.connection.row_factory = sqlite3.Row
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
        migrate_runtime_database_v2(self.connection, self.root)

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
        total = int(self.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0])
        excess = max(0, total - MAX_OPERATOR_COMMANDS)
        if excess == 0:
            return []
        stale = self.connection.execute(
            """
            SELECT sequence,evidence_relpath
            FROM operator_commands
            WHERE status NOT IN ('queued','executing')
            ORDER BY updated_at ASC, sequence ASC
            LIMIT ?
            """,
            (excess,),
        ).fetchall()
        if not stale:
            return []
        self.connection.executemany(
            "DELETE FROM operator_commands WHERE sequence=?",
            [(int(row["sequence"]),) for row in stale],
        )
        return [str(row["evidence_relpath"]) for row in stale if row["evidence_relpath"]]

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
                    "SELECT receipt_json FROM operator_commands WHERE status IN ('queued','executing')"
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
                    self._upsert(receipt, evidence=(None, None, None))
                self.connection.execute("COMMIT")
            except Exception:
                self.connection.execute("ROLLBACK")
                raise
        return len(rows)

    def claim(self, receipt: Mapping[str, Any]) -> tuple[dict[str, Any], bool]:
        """Atomically claim an idempotency key before normal dispatch."""
        row = dict(receipt)
        with self.lock:
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                existing = self.connection.execute(
                    "SELECT * FROM operator_commands WHERE idempotency_key=?",
                    (str(row.get("idempotency_key") or ""),),
                ).fetchone()
                if existing is not None:
                    self.connection.execute("COMMIT")
                    return self._row_receipt(existing, include_evidence=False), False
                compact = self._upsert(row, evidence=(None, None, None))
                pruned = self._prune_locked()
                self.connection.execute("COMMIT")
            except Exception:
                self.connection.execute("ROLLBACK")
                raise
        self._remove_pruned_evidence(pruned)
        return compact, True

    def put_interrupt(self, receipt: Mapping[str, Any]) -> dict[str, Any]:
        """Persist a delivered safety interrupt without waiting on normal DB work."""
        if not self.lock.acquire(blocking=False):
            return self.append_interrupt_fallback(receipt, reason="sqlite_connection_busy")
        try:
            self.connection.execute("PRAGMA busy_timeout=0")
            try:
                self.connection.execute("BEGIN IMMEDIATE")
                evidence = self._persist_evidence(receipt)
                previous = self.connection.execute(
                    "SELECT evidence_relpath FROM operator_commands WHERE command_id=?",
                    (str(receipt.get("command_id") or ""),),
                ).fetchone()
                compact = self._upsert(receipt, evidence=evidence)
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
                previous = self.connection.execute(
                    "SELECT evidence_relpath FROM operator_commands WHERE command_id=?",
                    (str(row.get("command_id") or ""),),
                ).fetchone()
                compact = self._upsert(row, evidence=evidence)
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
