from __future__ import annotations

import hashlib
import fcntl
import json
import os
import sqlite3
import threading
import tempfile
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

from .oem_runtime_types import OEMRuntimeSnapshot, utc_ts


MAX_SERIAL206_RECEIPTS_PER_STREAM = 128
MAX_SERIAL206_INTERRUPT_FALLBACK_ARCHIVES = 8


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    """Durably replace one private JSON authority file.

    The temporary file and containing state directory are deliberately private.
    ``fsync`` is applied to both file contents and the parent directory so an ACK
    cannot be returned for a transition that only existed in page cache.
    """
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.chmod(path.parent, 0o700)
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    tmp = Path(tmp_name)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "wb", closefd=True) as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
        os.chmod(path, 0o600)
        directory_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            tmp.unlink()
        except FileNotFoundError:
            pass


class OEMRuntimeStore:
    def __init__(self, root: str | Path | None = None):
        self.root = Path(
            root
            or os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT")
            or os.environ.get("BIOXP_OEM_RUNTIME_ROOT")
            or "/tmp/bioxp-oem-runtime"
        )
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.root, 0o700)
        self.serial206_interrupt_fallback_path = self.root / "serial206_interrupt_fallback.jsonl"
        self.serial206_interrupt_fallback_lock_path = self.root / "serial206_interrupt_fallback.lock"
        self._lock = threading.RLock()
        self._db = sqlite3.connect(
            self.root / "bioxp_runtime.db",
            timeout=2.0,
            isolation_level=None,
            check_same_thread=False,
        )
        self._db.row_factory = sqlite3.Row
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=FULL")
        self._db.execute("PRAGMA foreign_keys=ON")
        self._db.execute("PRAGMA busy_timeout=2000")
        self._db.execute("PRAGMA wal_autocheckpoint=256")
        self._db.execute("PRAGMA journal_size_limit=4194304")
        self._db.executescript(
            """
            CREATE TABLE IF NOT EXISTS runtime_metadata (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL,
                updated_at REAL NOT NULL
            ) WITHOUT ROWID;
            CREATE TABLE IF NOT EXISTS serial206_receipts (
                stream TEXT NOT NULL,
                receipt_id TEXT NOT NULL,
                command_id TEXT,
                idempotency_key TEXT,
                idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1
                    CHECK(idempotency_replay_enabled IN (0, 1)),
                status TEXT,
                observed_at REAL NOT NULL,
                receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
                PRIMARY KEY(stream, receipt_id)
            ) WITHOUT ROWID;
            CREATE INDEX IF NOT EXISTS serial206_receipts_command_idx
                ON serial206_receipts(stream, command_id);
            CREATE UNIQUE INDEX IF NOT EXISTS serial206_receipts_idempotency_idx
                ON serial206_receipts(stream, idempotency_key)
                WHERE idempotency_key IS NOT NULL AND idempotency_replay_enabled = 1;
            CREATE INDEX IF NOT EXISTS serial206_receipts_time_idx
                ON serial206_receipts(stream, observed_at DESC);
            """
        )
        self._seq = self._load_seq()
        self._import_embedded_serial206_receipts_once()
        self._import_serial206_interrupt_fallback()

    def _import_embedded_serial206_receipts_once(self) -> None:
        marker_key = "serial206_embedded_receipt_import_v1"
        if self._db.execute(
            "SELECT 1 FROM runtime_metadata WHERE key=?",
            (marker_key,),
        ).fetchone() is not None:
            return
        path = self.root / "serial206_oem_initialization_state.json"
        imported = 0
        payload: Any = None
        if path.exists():
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError(
                    f"embedded serial-206 receipt import failed for {path}"
                ) from exc
            if not isinstance(payload, dict):
                raise RuntimeError(
                    f"embedded serial-206 receipt import failed for {path}: state is not an object"
                )
        self._db.execute("BEGIN IMMEDIATE")
        try:
            if isinstance(payload, dict):
                for stream, lifecycle_key in (("z", "z_lifecycle"), ("x", "x_lifecycle")):
                    lifecycle = payload.get(lifecycle_key)
                    receipts = lifecycle.get("receipts") if isinstance(lifecycle, dict) else None
                    if receipts is None:
                        continue
                    if not isinstance(receipts, list):
                        raise ValueError(f"{lifecycle_key}.receipts must be a list")
                    for receipt_index, raw_receipt in enumerate(receipts):
                        if not isinstance(raw_receipt, dict):
                            raise ValueError(f"{lifecycle_key}.receipts contains a non-object row")
                        imported_receipt = dict(raw_receipt)
                        replay_enabled = imported_receipt.get("idempotency_replay_enabled")
                        if replay_enabled is None:
                            replay_enabled = imported_receipt.get("intent") not in {"stop", "abort"}
                        replay_enabled = bool(replay_enabled)
                        imported_receipt["idempotency_replay_enabled"] = replay_enabled
                        command_id = imported_receipt.get("command_id")
                        command_text = str(command_id) if isinstance(command_id, str) and command_id else None
                        idempotency_key = imported_receipt.get("idempotency_key")
                        idempotency_text = (
                            idempotency_key
                            if isinstance(idempotency_key, str) and idempotency_key
                            else None
                        )
                        status = imported_receipt.get("status")
                        status_text = status if isinstance(status, str) and status else None
                        identity_payload = json.dumps(
                            imported_receipt,
                            sort_keys=True,
                            separators=(",", ":"),
                            allow_nan=False,
                        )
                        supplied_receipt_id = imported_receipt.get("receipt_id")
                        if isinstance(supplied_receipt_id, str) and supplied_receipt_id:
                            receipt_id = supplied_receipt_id
                        elif replay_enabled:
                            receipt_id = command_text or hashlib.sha256(identity_payload.encode("utf-8")).hexdigest()
                        else:
                            receipt_id = hashlib.sha256(
                                f"{stream}:{receipt_index}:{identity_payload}".encode("utf-8")
                            ).hexdigest()
                        imported_receipt["receipt_id"] = receipt_id
                        encoded = json.dumps(
                            imported_receipt,
                            sort_keys=True,
                            separators=(",", ":"),
                            allow_nan=False,
                        )
                        try:
                            observed_at = float(
                                imported_receipt.get("finished_at")
                                or imported_receipt.get("started_at")
                                or imported_receipt.get("observed_at")
                                or utc_ts()
                            )
                        except (TypeError, ValueError, OverflowError):
                            observed_at = float(utc_ts())
                        self._db.execute(
                            """
                            INSERT INTO serial206_receipts(
                                stream,receipt_id,command_id,idempotency_key,
                                idempotency_replay_enabled,status,observed_at,receipt_json
                            ) VALUES(?,?,?,?,?,?,?,?)
                            ON CONFLICT(stream,receipt_id) DO UPDATE SET
                                command_id=excluded.command_id,
                                idempotency_key=excluded.idempotency_key,
                                idempotency_replay_enabled=excluded.idempotency_replay_enabled,
                                status=excluded.status,
                                observed_at=excluded.observed_at,
                                receipt_json=excluded.receipt_json
                            """,
                            (
                                stream,
                                receipt_id,
                                command_text,
                                idempotency_text,
                                int(replay_enabled),
                                status_text,
                                observed_at,
                                encoded,
                            ),
                        )
                        imported += 1
            self._db.execute(
                "INSERT INTO runtime_metadata(key,value,updated_at) VALUES(?,?,?)",
                (
                    marker_key,
                    json.dumps({"source": str(path), "source_retained": True, "imported": imported}),
                    time.time(),
                ),
            )
            self._db.execute("COMMIT")
        except Exception:
            self._db.execute("ROLLBACK")
            raise

    def append_serial206_interrupt_fallback(
        self,
        stream: str,
        receipt: Mapping[str, Any],
        *,
        reason: str,
    ) -> dict[str, Any]:
        row = dict(receipt)
        row["persistence_fallback"] = {
            "kind": "serial206_interrupt_jsonl",
            "reason": str(reason)[:500],
            "recorded_at": time.time(),
        }
        wrapper = {"stream": str(stream).strip().lower(), "receipt": row}
        raw = json.dumps(
            wrapper,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8") + b"\n"
        lock_descriptor = os.open(
            self.serial206_interrupt_fallback_lock_path,
            os.O_CREAT | os.O_RDWR,
            0o600,
        )
        try:
            os.fchmod(lock_descriptor, 0o600)
            fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
            descriptor = os.open(
                self.serial206_interrupt_fallback_path,
                os.O_APPEND | os.O_CREAT | os.O_WRONLY,
                0o600,
            )
            try:
                os.fchmod(descriptor, 0o600)
                written = os.write(descriptor, raw)
                if written != len(raw):
                    raise OSError(f"short serial-206 interrupt fallback write: {written}/{len(raw)} bytes")
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            directory_descriptor = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)
        finally:
            fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
            os.close(lock_descriptor)
        return row

    def _import_serial206_interrupt_fallback(self) -> None:
        lock_descriptor = os.open(
            self.serial206_interrupt_fallback_lock_path,
            os.O_CREAT | os.O_RDWR,
            0o600,
        )
        try:
            os.fchmod(lock_descriptor, 0o600)
            fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
            if self.serial206_interrupt_fallback_path.exists():
                pending = self.root / f"serial206_interrupt_fallback.pending.{time.time_ns()}.jsonl"
                os.replace(self.serial206_interrupt_fallback_path, pending)
                directory_descriptor = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory_descriptor)
                finally:
                    os.close(directory_descriptor)
        finally:
            fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
            os.close(lock_descriptor)

        pending_paths = sorted(self.root.glob("serial206_interrupt_fallback.pending.*.jsonl"))
        for pending in pending_paths:
            try:
                rows = [
                    json.loads(line)
                    for line in pending.read_text(encoding="utf-8").splitlines()
                    if line.strip()
                ]
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"serial-206 interrupt fallback import failed for {pending}") from exc
            for wrapper in rows:
                if not isinstance(wrapper, Mapping) or not isinstance(wrapper.get("receipt"), Mapping):
                    raise RuntimeError("serial-206 interrupt fallback contains an invalid row")
                self.append_serial206_receipt(str(wrapper.get("stream") or ""), wrapper["receipt"])
            archive = self.root / pending.name.replace(".pending.", ".imported.")
            os.replace(pending, archive)
            directory_descriptor = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)

        archives = sorted(
            self.root.glob("serial206_interrupt_fallback.imported.*.jsonl"),
            key=lambda selected: selected.stat().st_mtime_ns,
            reverse=True,
        )
        removed_archive = False
        for stale in archives[MAX_SERIAL206_INTERRUPT_FALLBACK_ARCHIVES:]:
            stale.unlink()
            removed_archive = True
        if removed_archive:
            directory_descriptor = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)

    def append_serial206_interrupt_receipt(
        self,
        stream: str,
        receipt: Mapping[str, Any],
    ) -> dict[str, Any]:
        if not self._lock.acquire(blocking=False):
            return self.append_serial206_interrupt_fallback(
                stream,
                receipt,
                reason="sqlite_connection_busy",
            )
        try:
            self._db.execute("PRAGMA busy_timeout=0")
            try:
                return self.append_serial206_receipt(stream, dict(receipt))
            except sqlite3.Error as exc:
                return self.append_serial206_interrupt_fallback(
                    stream,
                    receipt,
                    reason=f"{type(exc).__name__}: {exc}",
                )
            finally:
                self._db.execute("PRAGMA busy_timeout=2000")
        finally:
            self._lock.release()

    @property
    def serial206_initialization_state_path(self) -> Path:
        return self.root / "serial206_oem_initialization_state.json"

    def read_oem_serial206_initialization_state(self) -> dict[str, Any] | None:
        """Read the single atomic serial-206 lifecycle authority.

        JSON/schema errors intentionally propagate.  Callers must fail closed;
        treating corruption as a fresh state could replay acknowledged motion.
        """
        path = self.serial206_initialization_state_path
        if not path.exists():
            return None
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("serial-206 initialization state must be an object")
        return payload

    def write_oem_serial206_initialization_state(self, state: dict[str, Any]) -> dict[str, Any]:
        """Persist compact current authority; detailed receipts remain in SQLite."""
        payload = dict(state)
        required = {"movement_ledger", "used_approvals", "initialize_motion_ledger"}
        if not required.issubset(payload):
            raise ValueError("serial-206 state must contain all lifecycle ledgers")
        stored_payload = dict(payload)
        for lifecycle_key in ("z_lifecycle", "x_lifecycle"):
            lifecycle = stored_payload.get(lifecycle_key)
            if isinstance(lifecycle, dict):
                compact_lifecycle = dict(lifecycle)
                receipts = compact_lifecycle.get("receipts")
                if isinstance(receipts, list):
                    compact_lifecycle["receipts"] = receipts[-1:]
                    compact_lifecycle["receipts_omitted_to_sqlite"] = max(0, len(receipts) - 1)
                stored_payload[lifecycle_key] = compact_lifecycle
        with self._lock:
            _atomic_json(self.serial206_initialization_state_path, stored_payload)
        return payload

    def append_serial206_receipt(self, stream: str, receipt: dict[str, Any]) -> dict[str, Any]:
        """Persist one provider receipt without expanding the current-state file."""
        selected_stream = str(stream).strip().lower()
        if selected_stream not in {"x", "z", "initialize_motion"}:
            raise ValueError("unsupported serial-206 receipt stream")
        payload = dict(receipt)
        replay_enabled = payload.get("idempotency_replay_enabled")
        if replay_enabled is None:
            replay_enabled = payload.get("intent") not in {"stop", "abort"}
        replay_enabled = bool(replay_enabled)
        payload["idempotency_replay_enabled"] = replay_enabled
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
        command_id = payload.get("command_id")
        command_text = str(command_id) if isinstance(command_id, str) and command_id else None
        idempotency_key = payload.get("idempotency_key")
        idempotency_text = (
            idempotency_key
            if isinstance(idempotency_key, str) and idempotency_key
            else None
        )
        status = payload.get("status")
        status_text = status if isinstance(status, str) and status else None
        supplied_receipt_id = payload.get("receipt_id")
        receipt_id = (
            str(supplied_receipt_id)
            if isinstance(supplied_receipt_id, str) and supplied_receipt_id
            else command_text or hashlib.sha256(encoded.encode("utf-8")).hexdigest()
        )
        try:
            observed_at = float(
                payload.get("finished_at")
                or payload.get("started_at")
                or payload.get("observed_at")
                or utc_ts()
            )
        except (TypeError, ValueError, OverflowError):
            observed_at = float(utc_ts())
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                self._db.execute(
                    """
                    INSERT INTO serial206_receipts(
                        stream,receipt_id,command_id,idempotency_key,
                        idempotency_replay_enabled,status,observed_at,receipt_json
                    ) VALUES(?,?,?,?,?,?,?,?)
                    ON CONFLICT(stream,receipt_id) DO UPDATE SET
                        command_id=excluded.command_id,
                        idempotency_key=excluded.idempotency_key,
                        idempotency_replay_enabled=excluded.idempotency_replay_enabled,
                        status=excluded.status,
                        observed_at=excluded.observed_at,
                        receipt_json=excluded.receipt_json
                    """,
                    (
                        selected_stream,
                        receipt_id,
                        command_text,
                        idempotency_text,
                        int(replay_enabled),
                        status_text,
                        observed_at,
                        encoded,
                    ),
                )
                self._db.execute(
                    """
                    DELETE FROM serial206_receipts
                    WHERE stream=? AND receipt_id IN (
                        SELECT receipt_id
                        FROM serial206_receipts
                        WHERE stream=?
                        ORDER BY observed_at DESC, receipt_id DESC
                        LIMIT -1 OFFSET ?
                    )
                    """,
                    (
                        selected_stream,
                        selected_stream,
                        MAX_SERIAL206_RECEIPTS_PER_STREAM,
                    ),
                )
                self._db.execute("COMMIT")
            except Exception:
                self._db.execute("ROLLBACK")
                raise
        return payload

    def read_serial206_receipt(self, stream: str, command_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute(
                "SELECT receipt_json FROM serial206_receipts WHERE stream=? AND command_id=? LIMIT 1",
                (str(stream).strip().lower(), str(command_id)),
            ).fetchone()
        return None if row is None else json.loads(row["receipt_json"])

    def read_serial206_receipt_by_idempotency(self, stream: str, key: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute(
                """
                SELECT receipt_json FROM serial206_receipts
                WHERE stream=? AND idempotency_key=?
                  AND idempotency_replay_enabled=1
                LIMIT 1
                """,
                (str(stream).strip().lower(), str(key)),
            ).fetchone()
        return None if row is None else json.loads(row["receipt_json"])

    def list_serial206_receipts(self, stream: str, limit: int = 50) -> list[dict[str, Any]]:
        selected_limit = max(1, min(int(limit), 200))
        with self._lock:
            rows = self._db.execute(
                """
                SELECT receipt_json FROM serial206_receipts
                WHERE stream=? ORDER BY observed_at DESC, receipt_id DESC LIMIT ?
                """,
                (str(stream).strip().lower(), selected_limit),
            ).fetchall()
        return [json.loads(row["receipt_json"]) for row in reversed(rows)]

    def _load_seq(self) -> int:
        p = self.root / "sequence.txt"
        try:
            return int(p.read_text().strip())
        except Exception:
            return 0

    def next_seq(self) -> int:
        with self._lock:
            self._seq += 1
            (self.root / "sequence.txt").write_text(str(self._seq))
            return self._seq

    def write_state(self, snapshot: OEMRuntimeSnapshot | dict[str, Any]) -> dict[str, Any]:
        payload = snapshot.to_dict() if hasattr(snapshot, "to_dict") else dict(snapshot)
        from .hardware_status import hardware_state
        from .lifecycle_state import lifecycle_state

        canonical = hardware_state.completed_snapshot()
        payload["canonical_hardware_snapshot"] = {
            "snapshot_id": None if canonical is None else canonical.get("snapshot_id"),
            "ownership_epoch": hardware_state.ownership_epoch,
            "available": canonical is not None,
            "reference_only": True,
        }
        lifecycle = lifecycle_state.projection()
        payload["runtime_state"] = lifecycle["operation_state"]
        payload["operation_state"] = lifecycle["operation_state"]
        payload["startup"] = lifecycle["startup"]
        payload["lifecycle_revision"] = lifecycle["revision"]
        payload["sequence"] = self.next_seq()
        payload["updated_at"] = utc_ts()
        _atomic_json(self.root / "runtime_state.json", payload)
        return payload

    def read_state(self) -> dict[str, Any] | None:
        p = self.root / "runtime_state.json"
        if not p.exists():
            return None
        return json.loads(p.read_text())


    def create_oem_full_lifecycle_run_once(
        self,
        run: dict[str, Any],
        *,
        current_ownership_generation: Callable[[], int] | None = None,
    ) -> dict[str, Any]:
        """Atomically converge same-key creators within the single runtime owner."""
        payload = dict(run)
        key = payload.get("idempotency_key")
        request = payload.get("request")
        if not isinstance(key, str) or not key or len(key) > 128:
            raise ValueError("valid bounded idempotency_key required")
        with self._lock:
            if current_ownership_generation is not None:
                expected = request.get("expected_generation") if isinstance(request, dict) else None
                if expected != current_ownership_generation():
                    raise ValueError("expected_generation no longer matches current robot ownership generation")
            existing_runs = self.list_oem_full_lifecycle_runs()
            for existing in existing_runs:
                if existing.get("idempotency_key") != key:
                    continue
                if existing.get("request") != request:
                    raise ValueError("idempotency_key is already bound to a different request")
                return existing
            active_states = {"planned", "running", "admitted", "acknowledged", "blocked", "reconciliation_required"}
            if any(existing.get("run_state") in active_states for existing in existing_runs):
                raise ValueError("another active OEM lifecycle run already owns the robot lifecycle")
            return self.write_oem_full_lifecycle_run(payload)

    def write_oem_full_lifecycle_run(self, run: dict[str, Any]) -> dict[str, Any]:
        """Persist one full OEM movement-lifecycle run atomically.

        Run files are immutable by identity but replaceable by monotonic state
        updates.  The robot owns the directory and run identifier; callers do
        not supply paths.
        """
        payload = dict(run)
        run_id = str(payload.get("run_id") or "").strip()
        if not run_id or "/" in run_id or "\\" in run_id or run_id in {".", ".."}:
            raise ValueError("valid robot-owned run_id required")
        with self._lock:
            payload["sequence"] = self.next_seq()
            _atomic_json(self.root / "movement_runs" / f"{run_id}.json", payload)
        return payload

    def mutate_oem_full_lifecycle_run(
        self,
        run_id: str,
        mutation: Callable[[dict[str, Any]], dict[str, Any]],
    ) -> dict[str, Any]:
        """Read, validate, and replace one run under the runtime-owner lock."""
        with self._lock:
            payload = self.read_oem_full_lifecycle_run(run_id)
            if payload is None:
                raise ValueError(f"full OEM lifecycle run {run_id!r} not found")
            return self.write_oem_full_lifecycle_run(mutation(payload))

    def read_oem_full_lifecycle_run(self, run_id: str) -> dict[str, Any] | None:
        selected = str(run_id).strip()
        if not selected or "/" in selected or "\\" in selected or selected in {".", ".."}:
            raise ValueError("valid robot-owned run_id required")
        path = self.root / "movement_runs" / f"{selected}.json"
        if not path.exists():
            return None
        return json.loads(path.read_text())

    def list_oem_full_lifecycle_runs(self) -> list[dict[str, Any]]:
        root = self.root / "movement_runs"
        if not root.exists():
            return []
        rows: list[dict[str, Any]] = []
        for path in sorted(root.glob("*.json")):
            rows.append(json.loads(path.read_text()))
        return rows

    def append_journal(self, name: str, payload: dict[str, Any]) -> dict[str, Any]:
        row = dict(payload)
        row.setdefault("created_at", utc_ts())
        row["sequence"] = self.next_seq()
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            with path.open("a") as fh:
                fh.write(json.dumps(row, sort_keys=True) + "\n")
        return row

    def append_command_queue(self, command: dict[str, Any]) -> dict[str, Any]:
        return self.append_journal("command_queue.jsonl", command)

    def append_command_history(self, row: dict[str, Any]) -> dict[str, Any]:
        return self.append_journal("command_history.jsonl", row)

    def append_event(self, event: dict[str, Any]) -> dict[str, Any]:
        return self.append_journal("event_journal.jsonl", event)

    def append_error(self, error: dict[str, Any]) -> dict[str, Any]:
        return self.append_journal("runtime_errors.jsonl", error)

    def read_journal(self, name: str, limit: int = 50) -> list[dict[str, Any]]:
        path = self.root / name
        if not path.exists():
            return []
        rows = []
        for line in path.read_text().splitlines():
            if line.strip():
                rows.append(json.loads(line))
        return rows[-limit:]

    def recover_state(self) -> dict[str, Any]:
        state = self.read_state()
        if state is None:
            return {"recovery": "fresh", "state": None, "recovery_required": False}
        worker = state.get("worker") or {}
        active = worker.get("active_command")
        running = worker.get("state") == "running" or active is not None
        return {"recovery": "active_command" if running else "idle", "state": state, "recovery_required": bool(running)}
