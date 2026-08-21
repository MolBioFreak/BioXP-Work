from __future__ import annotations

import hashlib
import fcntl
import json
import os
import sqlite3
import threading
import tempfile
import time
from collections.abc import Callable, Iterable, Mapping
from pathlib import Path
from typing import Any

from .oem_runtime_types import OEMRuntimeSnapshot, utc_ts


MAX_SERIAL206_RECEIPTS_PER_STREAM = 128
MAX_SERIAL206_INTERRUPT_FALLBACK_ARCHIVES = 8

SERIAL206_SCHEMA_VERSION = 2
SERIAL206_BOARD4_MEMBERS = {"y": 0, "z": 1, "gripper": 2}
_NONREPLAYABLE_ACTIONS = frozenset({
    "meta.emergency_stop",
    "oem.x.stop",
    "oem.y.stop",
    "oem.z.stop",
    "oem.z.abort",
    "oem.abort_all",
})


def _schema_source_digests(root: Path) -> dict[str, str | None]:
    digests: dict[str, str | None] = {}
    for name in ("reference-state.json", "serial206_oem_initialization_state.json"):
        path = root / name
        digests[name] = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
    return digests


def _drop_unconditional_idempotency_indexes(connection: sqlite3.Connection) -> None:
    for row in connection.execute("PRAGMA index_list(operator_commands)").fetchall():
        if not row[1] or row[3] != "c" or row[4]:
            continue
        name = str(row[1])
        columns = [
            str(info[0])
            for info in connection.execute(
                "SELECT name FROM pragma_index_info(?)", (name,)
            ).fetchall()
        ]
        if columns == ["idempotency_key"]:
            connection.execute(f'DROP INDEX "{name.replace(chr(34), chr(34) * 2)}"')


def _execute_schema_batch(connection: sqlite3.Connection, script: str) -> None:
    for statement in script.split(";"):
        statement = statement.strip()
        if statement:
            connection.execute(statement)


def _create_v1_runtime_schema(connection: sqlite3.Connection) -> None:
    _execute_schema_batch(connection,
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
            controller_acknowledged INTEGER NOT NULL DEFAULT 0
                CHECK(controller_acknowledged IN (0,1)),
            physical_effect_verified INTEGER NOT NULL DEFAULT 0
                CHECK(physical_effect_verified IN (0,1)),
            receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
            response_summary_json TEXT
                CHECK(response_summary_json IS NULL OR json_valid(response_summary_json)),
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
        str(row[1]) for row in connection.execute("PRAGMA table_info(operator_commands)").fetchall()
    }
    if "idempotency_replay_enabled" not in columns:
        connection.execute(
            """
            ALTER TABLE operator_commands
            ADD COLUMN idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1
                CHECK(idempotency_replay_enabled IN (0,1))
            """
        )
    _drop_unconditional_idempotency_indexes(connection)
    placeholders = ",".join("?" for _ in _NONREPLAYABLE_ACTIONS)
    connection.execute(
        f"UPDATE operator_commands SET idempotency_replay_enabled=0 WHERE action_id IN ({placeholders})",
        tuple(sorted(_NONREPLAYABLE_ACTIONS)),
    )
    connection.execute(
        """
        CREATE UNIQUE INDEX IF NOT EXISTS operator_commands_replay_key_idx
            ON operator_commands(idempotency_key)
            WHERE idempotency_replay_enabled=1
        """
    )


def _create_v2_authority_schema(connection: sqlite3.Connection) -> None:
    _execute_schema_batch(connection,
        """
        CREATE TABLE IF NOT EXISTS runtime_schema_migrations (
            version INTEGER PRIMARY KEY CHECK(version=2),
            backup_sha256 TEXT NOT NULL CHECK(length(backup_sha256)=64),
            source_json_digests_json TEXT NOT NULL CHECK(json_valid(source_json_digests_json)),
            started_at REAL NOT NULL,
            finished_at REAL NOT NULL,
            result TEXT NOT NULL CHECK(result='committed')
        ) WITHOUT ROWID;
        CREATE TABLE IF NOT EXISTS serial206_board_authority (
            board_id INTEGER PRIMARY KEY CHECK(board_id=4),
            state TEXT NOT NULL CHECK(state IN ('inactive','transitioning','active','faulted')),
            prior_board_epoch INTEGER CHECK(prior_board_epoch IS NULL OR prior_board_epoch>=0),
            active_board_epoch INTEGER CHECK(active_board_epoch IS NULL OR active_board_epoch>=0),
            transition_id TEXT,
            deactivation_attempt_id TEXT,
            deactivation_delivery INTEGER CHECK(deactivation_delivery IS NULL OR deactivation_delivery IN (0,1)),
            deactivation_reply_valid INTEGER CHECK(deactivation_reply_valid IS NULL OR deactivation_reply_valid IN (0,1)),
            deactivation_status_code INTEGER,
            activation_attempt_id TEXT,
            activation_delivery INTEGER CHECK(activation_delivery IS NULL OR activation_delivery IN (0,1)),
            activation_reply_valid INTEGER CHECK(activation_reply_valid IS NULL OR activation_reply_valid IN (0,1)),
            activation_status_code INTEGER,
            member_motors_json TEXT NOT NULL CHECK(json_valid(member_motors_json)),
            state_version INTEGER NOT NULL CHECK(state_version>=1),
            updated_at REAL NOT NULL
        );
        CREATE TABLE IF NOT EXISTS serial206_axis_authority (
            axis TEXT PRIMARY KEY CHECK(axis IN ('y','z','gripper')),
            board_id INTEGER NOT NULL REFERENCES serial206_board_authority(board_id),
            motor_id INTEGER NOT NULL CHECK(motor_id BETWEEN 0 AND 2),
            ownership_generation INTEGER NOT NULL CHECK(ownership_generation>=0),
            prepared_board_epoch INTEGER CHECK(prepared_board_epoch IS NULL OR prepared_board_epoch>=0),
            profile_fingerprint TEXT,
            lifecycle_state TEXT NOT NULL CHECK(lifecycle_state IN ('unprepared','prepared_unreferenced','referenced_ready','generation_stale','reconciliation_required','faulted')),
            reference_state TEXT NOT NULL CHECK(reference_state IN ('unreferenced','referenced','generation_stale','reconciliation_required')),
            origin_position_steps INTEGER,
            observed_position_steps INTEGER,
            last_discrepancy_steps INTEGER,
            last_command_id TEXT,
            last_receipt_id TEXT,
            interrupt_epoch INTEGER NOT NULL DEFAULT 0 CHECK(interrupt_epoch>=0),
            state_version INTEGER NOT NULL CHECK(state_version>=1),
            updated_at REAL NOT NULL
        ) WITHOUT ROWID;
        CREATE TABLE IF NOT EXISTS serial206_movement_methods (
            method_id TEXT PRIMARY KEY,
            idempotency_key TEXT NOT NULL UNIQUE,
            action_id TEXT NOT NULL,
            canonical_inputs_sha256 TEXT NOT NULL CHECK(length(canonical_inputs_sha256)=64),
            state TEXT NOT NULL CHECK(state IN ('queued','active','completed','completed_partial','failed','cleared','interrupted','ambiguous')),
            state_version INTEGER NOT NULL CHECK(state_version>=1),
            failure_policy TEXT NOT NULL CHECK(failure_policy='require_completed'),
            child_count INTEGER NOT NULL CHECK(child_count>=1),
            accepted_at REAL NOT NULL,
            started_at REAL,
            finished_at REAL
        ) WITHOUT ROWID;
        CREATE TABLE IF NOT EXISTS serial206_movement_commands (
            sequence INTEGER PRIMARY KEY AUTOINCREMENT,
            command_id TEXT NOT NULL UNIQUE,
            idempotency_key TEXT NOT NULL,
            action_id TEXT NOT NULL,
            method_id TEXT REFERENCES serial206_movement_methods(method_id) ON DELETE CASCADE,
            method_order INTEGER NOT NULL DEFAULT 0 CHECK(method_order>=0),
            parallel_group INTEGER NOT NULL DEFAULT 0 CHECK(parallel_group>=0),
            axis_scope TEXT,
            board_scope_json TEXT NOT NULL CHECK(json_valid(board_scope_json)),
            ownership_generation INTEGER NOT NULL CHECK(ownership_generation>=0),
            expected_board_epochs_json TEXT NOT NULL CHECK(json_valid(expected_board_epochs_json)),
            canonical_inputs_sha256 TEXT NOT NULL CHECK(length(canonical_inputs_sha256)=64),
            state TEXT NOT NULL CHECK(state IN ('queued','dispatched','issued_pending','interrupting','completed','failed','cleared','interrupted','ambiguous','rejected')),
            state_version INTEGER NOT NULL CHECK(state_version>=1),
            admitted_interrupt_epochs_json TEXT NOT NULL CHECK(json_valid(admitted_interrupt_epochs_json)),
            accepted_at REAL NOT NULL,
            queued_at REAL NOT NULL,
            dispatched_at REAL,
            finished_at REAL,
            terminal_receipt_id TEXT
        );
        CREATE TABLE IF NOT EXISTS serial206_command_resources (
            command_id TEXT NOT NULL REFERENCES serial206_movement_commands(command_id) ON DELETE CASCADE,
            resource_key TEXT NOT NULL,
            PRIMARY KEY(command_id,resource_key)
        ) WITHOUT ROWID;
        CREATE TABLE IF NOT EXISTS serial206_command_dependencies (
            command_id TEXT NOT NULL REFERENCES serial206_movement_commands(command_id) ON DELETE CASCADE,
            depends_on_command_id TEXT NOT NULL REFERENCES serial206_movement_commands(command_id) ON DELETE CASCADE,
            required_terminal TEXT NOT NULL CHECK(required_terminal='completed'),
            PRIMARY KEY(command_id,depends_on_command_id),
            CHECK(command_id<>depends_on_command_id)
        ) WITHOUT ROWID;
        CREATE TABLE IF NOT EXISTS serial206_interrupt_imports (
            record_sha256 TEXT PRIMARY KEY CHECK(length(record_sha256)=64),
            interrupt_attempt_id TEXT NOT NULL UNIQUE,
            axis TEXT NOT NULL,
            interrupt_epoch INTEGER NOT NULL CHECK(interrupt_epoch>=0),
            imported_at REAL NOT NULL,
            receipt_id TEXT NOT NULL
        ) WITHOUT ROWID;
        CREATE UNIQUE INDEX IF NOT EXISTS serial206_movement_commands_idempotency_idx
            ON serial206_movement_commands(idempotency_key);
        CREATE INDEX IF NOT EXISTS serial206_movement_commands_ready_idx
            ON serial206_movement_commands(state,sequence);
        CREATE INDEX IF NOT EXISTS serial206_movement_commands_method_idx
            ON serial206_movement_commands(method_id,method_order,parallel_group,sequence);
        CREATE INDEX IF NOT EXISTS serial206_command_resources_lookup_idx
            ON serial206_command_resources(resource_key,command_id);
        CREATE INDEX IF NOT EXISTS serial206_command_dependencies_reverse_idx
            ON serial206_command_dependencies(depends_on_command_id,command_id);
        """
    )


def _verify_v2_schema(connection: sqlite3.Connection) -> None:
    required = {
        "runtime_schema_migrations",
        "serial206_board_authority",
        "serial206_axis_authority",
        "serial206_movement_methods",
        "serial206_movement_commands",
        "serial206_command_resources",
        "serial206_command_dependencies",
        "serial206_interrupt_imports",
    }
    found = {
        str(row[0])
        for row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type='table'"
        ).fetchall()
    }
    missing = sorted(required - found)
    if missing:
        raise RuntimeError(f"serial-206 v2 schema missing tables: {','.join(missing)}")
    required_indexes = {
        "serial206_movement_commands_idempotency_idx",
        "serial206_movement_commands_ready_idx",
        "serial206_movement_commands_method_idx",
        "serial206_command_resources_lookup_idx",
        "serial206_command_dependencies_reverse_idx",
    }
    found_indexes = {
        str(row[1])
        for row in connection.execute(
            "SELECT type,name FROM sqlite_master WHERE type='index'"
        ).fetchall()
    }
    missing_indexes = sorted(required_indexes - found_indexes)
    if missing_indexes:
        raise RuntimeError(f"serial-206 v2 schema missing indexes: {','.join(missing_indexes)}")


def migrate_runtime_database_v2(connection: sqlite3.Connection, root: str | Path) -> None:
    """Own the one-time v1-to-v2 runtime schema transition.

    Both runtime stores call this function. It is idempotent and refuses a
    future schema without creating or mutating application tables.
    """
    selected_root = Path(root)
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    if version > SERIAL206_SCHEMA_VERSION:
        raise RuntimeError(f"unsupported runtime schema version {version}")
    if version == SERIAL206_SCHEMA_VERSION:
        _verify_v2_schema(connection)
        return
    backup_sha256 = "0" * 64
    source_digests = _schema_source_digests(selected_root)
    started_at = time.time()
    connection.execute("BEGIN IMMEDIATE")
    try:
        _create_v1_runtime_schema(connection)
        _create_v2_authority_schema(connection)
        connection.execute(
            """
            INSERT OR IGNORE INTO serial206_board_authority(
                board_id,state,member_motors_json,state_version,updated_at
            ) VALUES(4,'inactive',?,1,?)
            """,
            (json.dumps(SERIAL206_BOARD4_MEMBERS, sort_keys=True, separators=(",", ":")), time.time()),
        )
        for axis, motor_id in SERIAL206_BOARD4_MEMBERS.items():
            connection.execute(
                """
                INSERT OR IGNORE INTO serial206_axis_authority(
                    axis,board_id,motor_id,ownership_generation,lifecycle_state,
                    reference_state,state_version,updated_at
                ) VALUES(?,4,?,?, 'unprepared','unreferenced',1,?)
                """,
                (axis, motor_id, 0, time.time()),
            )
        finished_at = time.time()
        connection.execute(
            """
            INSERT OR REPLACE INTO runtime_schema_migrations(
                version,backup_sha256,source_json_digests_json,started_at,finished_at,result
            ) VALUES(2,?,?,?,?, 'committed')
            """,
            (backup_sha256, json.dumps(source_digests, sort_keys=True, separators=(",", ":")), started_at, finished_at),
        )
        connection.execute("PRAGMA user_version=2")
        _verify_v2_schema(connection)
        connection.execute("PRAGMA foreign_key_check")
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise


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
        self._closed = False
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=FULL")
        self._db.execute("PRAGMA foreign_keys=ON")
        self._db.execute("PRAGMA busy_timeout=2000")
        self._db.execute("PRAGMA wal_autocheckpoint=256")
        self._db.execute("PRAGMA journal_size_limit=4194304")
        migrate_runtime_database_v2(self._db, self.root)
        self._seq = self._load_seq()
        self._import_embedded_serial206_receipts_once()
        self._import_serial206_interrupt_fallback()

    def close(self) -> None:
        with self._lock:
            if not self._closed:
                self._db.close()
                self._closed = True

    @staticmethod
    def _authority_row(row: sqlite3.Row | None) -> dict[str, Any]:
        return {} if row is None else {str(key): row[key] for key in row.keys()}

    def _board4_row_locked(self) -> dict[str, Any]:
        row = self._db.execute(
            "SELECT * FROM serial206_board_authority WHERE board_id=4"
        ).fetchone()
        board = self._authority_row(row)
        if board:
            board["member_motors"] = json.loads(board.pop("member_motors_json"))
        return board

    def _axis_rows_locked(self) -> dict[str, dict[str, Any]]:
        rows = self._db.execute(
            "SELECT * FROM serial206_axis_authority ORDER BY axis"
        ).fetchall()
        output: dict[str, dict[str, Any]] = {}
        for row in rows:
            axis = str(row["axis"])
            output[axis] = self._authority_row(row)
        return output

    def board4_authority_projection(self) -> dict[str, Any]:
        with self._lock:
            return {"board": self._board4_row_locked(), "axes": self._axis_rows_locked()}

    def record_board4_transition(
        self,
        *,
        active: bool,
        ack: Any,
        transition_id: str,
        ownership_generation: int,
        invalidate_axes: bool = True,
        continuity_proven: bool = True,
    ) -> dict[str, Any]:
        """Persist one board-4 command-64 boundary and its member-axis effect."""
        if type(active) is not bool:
            raise ValueError("board-4 active must be bool")
        if not transition_id:
            raise ValueError("board-4 transition_id is required")
        status_code = int(ack.get("status", -1)) if isinstance(ack, Mapping) else -1
        reply_valid = isinstance(ack, Mapping)
        accepted = reply_valid and status_code == 100
        now = time.time()
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                current = self._db.execute(
                    "SELECT * FROM serial206_board_authority WHERE board_id=4"
                ).fetchone()
                if current is None:
                    raise RuntimeError("serial-206 board-4 authority row is missing")
                current_active_epoch = current["active_board_epoch"]
                current_prior_epoch = current["prior_board_epoch"]
                if active:
                    board_epoch = (
                        int(current_active_epoch)
                        if accepted and current["state"] == "active" and current_active_epoch is not None
                        else (int(current_active_epoch or current_prior_epoch or 0) + 1 if accepted else current_active_epoch)
                    )
                    state = "active" if accepted and continuity_proven else "faulted"
                    prior_epoch = current_active_epoch if current_active_epoch is not None else current_prior_epoch
                    active_epoch = board_epoch if accepted and continuity_proven else None
                    attempt_column = "activation_attempt_id"
                    delivery_column = "activation_delivery"
                    reply_column = "activation_reply_valid"
                    status_column = "activation_status_code"
                else:
                    prior_epoch = current_active_epoch if current_active_epoch is not None else current_prior_epoch
                    active_epoch = None
                    state = "inactive" if accepted and continuity_proven else "faulted"
                    attempt_column = "deactivation_attempt_id"
                    delivery_column = "deactivation_delivery"
                    reply_column = "deactivation_reply_valid"
                    status_column = "deactivation_status_code"
                self._db.execute(
                    f"""
                    UPDATE serial206_board_authority
                    SET state=?, prior_board_epoch=?, active_board_epoch=?, transition_id=?,
                        {attempt_column}=?, {delivery_column}=?, {reply_column}=?,
                        {status_column}=?, state_version=state_version+1, updated_at=?
                    WHERE board_id=4
                    """,
                    (
                        state,
                        prior_epoch,
                        active_epoch,
                        transition_id,
                        transition_id,
                        1,
                        int(reply_valid),
                        status_code,
                        now,
                    ),
                )
                if invalidate_axes:
                    for axis, row in self._axis_rows_locked().items():
                        has_authority = bool(
                            row.get("lifecycle_state") != "unprepared"
                            or row.get("reference_state") != "unreferenced"
                            or row.get("prepared_board_epoch") is not None
                        )
                        lifecycle = "generation_stale" if has_authority else "unprepared"
                        reference = "generation_stale" if has_authority else "unreferenced"
                        if state == "faulted" and has_authority:
                            lifecycle = "generation_stale"
                        self._db.execute(
                            """
                            UPDATE serial206_axis_authority
                            SET prepared_board_epoch=NULL, lifecycle_state=?, reference_state=?,
                                state_version=state_version+1, updated_at=?
                            WHERE axis=?
                            """,
                            (lifecycle, reference, now, axis),
                        )
                self._db.execute("COMMIT")
            except Exception:
                if self._db.in_transaction:
                    self._db.execute("ROLLBACK")
                raise
            return self.board4_authority_projection() | {
                "transition": {
                    "transition_id": transition_id,
                    "active": active,
                    "accepted": accepted,
                    "status_code": status_code,
                    "delivery": True,
                    "reply_valid": reply_valid,
                    "continuity_proven": continuity_proven,
                    "ownership_generation": int(ownership_generation),
                }
            }

    def prepare_axis_authority(
        self,
        axis: str,
        *,
        ownership_generation: int,
        profile_fingerprint: str,
    ) -> dict[str, Any]:
        axis = str(axis)
        if axis not in SERIAL206_BOARD4_MEMBERS:
            return {"ok": False, "failure": "unsupported_board4_axis", "axis": axis}
        with self._lock:
            board = self._board4_row_locked()
            if board.get("state") != "active" or board.get("active_board_epoch") is None:
                return {"ok": False, "failure": "board4_not_active", "axis": axis, "board": board}
            now = time.time()
            self._db.execute("BEGIN IMMEDIATE")
            try:
                self._db.execute(
                    """
                    UPDATE serial206_axis_authority
                    SET ownership_generation=?, prepared_board_epoch=?, profile_fingerprint=?,
                        lifecycle_state='prepared_unreferenced', reference_state='unreferenced',
                        state_version=state_version+1, updated_at=?
                    WHERE axis=?
                    """,
                    (int(ownership_generation), int(board["active_board_epoch"]), str(profile_fingerprint), now, axis),
                )
                self._db.execute("COMMIT")
            except Exception:
                if self._db.in_transaction:
                    self._db.execute("ROLLBACK")
                raise
            return {"ok": True, "axis": self._axis_rows_locked()[axis], "board": self._board4_row_locked()}

    def publish_axis_reference(
        self,
        axis: str,
        *,
        position_steps: int,
        ownership_generation: int,
        receipt_id: str | None = None,
    ) -> dict[str, Any]:
        axis = str(axis)
        if axis not in SERIAL206_BOARD4_MEMBERS:
            return {"ok": False, "failure": "unsupported_board4_axis", "axis": axis}
        with self._lock:
            board = self._board4_row_locked()
            row = self._axis_rows_locked().get(axis, {})
            if board.get("state") != "active" or row.get("prepared_board_epoch") != board.get("active_board_epoch"):
                return {"ok": False, "failure": "axis_board_epoch_not_current", "axis": axis, "board": board, "axis_state": row}
            if int(row.get("ownership_generation", -1)) != int(ownership_generation):
                return {"ok": False, "failure": "axis_generation_mismatch", "axis": axis, "axis_state": row}
            now = time.time()
            self._db.execute("BEGIN IMMEDIATE")
            try:
                self._db.execute(
                    """
                    UPDATE serial206_axis_authority
                    SET lifecycle_state='referenced_ready', reference_state='referenced',
                        origin_position_steps=?, observed_position_steps=?, last_receipt_id=?,
                        state_version=state_version+1, updated_at=?
                    WHERE axis=?
                    """,
                    (int(position_steps), int(position_steps), receipt_id, now, axis),
                )
                self._db.execute("COMMIT")
            except Exception:
                if self._db.in_transaction:
                    self._db.execute("ROLLBACK")
                raise
            return {"ok": True, "axis": self._axis_rows_locked()[axis], "board": self._board4_row_locked()}

    def record_axis_observation(
        self,
        axis: str,
        *,
        requested_position_steps: int,
        observed_position_steps: int,
        receipt_id: str | None = None,
    ) -> dict[str, Any]:
        axis = str(axis)
        if axis not in SERIAL206_BOARD4_MEMBERS:
            return {"ok": False, "failure": "unsupported_board4_axis", "axis": axis}
        discrepancy = int(observed_position_steps) - int(requested_position_steps)
        with self._lock:
            now = time.time()
            self._db.execute("BEGIN IMMEDIATE")
            try:
                self._db.execute(
                    """
                    UPDATE serial206_axis_authority
                    SET observed_position_steps=?, last_discrepancy_steps=?, last_receipt_id=?, updated_at=?
                    WHERE axis=?
                    """,
                    (int(observed_position_steps), discrepancy, receipt_id, now, axis),
                )
                self._db.execute("COMMIT")
            except Exception:
                if self._db.in_transaction:
                    self._db.execute("ROLLBACK")
                raise
            return {
                "ok": True,
                "axis": self._axis_rows_locked()[axis],
                "discrepancy_steps": discrepancy,
                "reconciled_to_observed": True,
            }

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

    def append_serial206_receipts_atomic(
        self,
        receipts: Iterable[tuple[str, dict[str, Any]]],
    ) -> list[dict[str, Any]]:
        """Persist several stream receipts in one SQLite transaction."""
        normalized: list[tuple[str, dict[str, Any], tuple[Any, ...]]] = []
        for stream, receipt in receipts:
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
            idempotency_text = idempotency_key if isinstance(idempotency_key, str) and idempotency_key else None
            status = payload.get("status")
            status_text = status if isinstance(status, str) and status else None
            supplied_receipt_id = payload.get("receipt_id")
            receipt_id = str(supplied_receipt_id) if isinstance(supplied_receipt_id, str) and supplied_receipt_id else command_text or hashlib.sha256(encoded.encode("utf-8")).hexdigest()
            try:
                observed_at = float(payload.get("finished_at") or payload.get("started_at") or payload.get("observed_at") or utc_ts())
            except (TypeError, ValueError, OverflowError):
                observed_at = float(utc_ts())
            normalized.append((selected_stream, payload, (selected_stream, receipt_id, command_text, idempotency_text, int(replay_enabled), status_text, observed_at, encoded)))
        if not normalized:
            raise ValueError("at least one serial-206 receipt required")
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                for selected_stream, _payload, row in normalized:
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
                        row,
                    )
                    self._db.execute(
                        """
                        DELETE FROM serial206_receipts
                        WHERE stream=? AND receipt_id IN (
                            SELECT receipt_id FROM serial206_receipts
                            WHERE stream=? ORDER BY observed_at DESC, receipt_id DESC
                            LIMIT -1 OFFSET ?
                        )
                        """,
                        (selected_stream, selected_stream, MAX_SERIAL206_RECEIPTS_PER_STREAM),
                    )
                self._db.execute("COMMIT")
            except Exception:
                self._db.execute("ROLLBACK")
                raise
        return [payload for _stream, payload, _row in normalized]

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
