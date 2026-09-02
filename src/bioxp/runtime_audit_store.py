from __future__ import annotations

import hashlib
import inspect
import json
import os
import socket
import sqlite3
import threading
import time
import uuid
import fcntl
from contextlib import contextmanager
from dataclasses import dataclass, field
from functools import wraps
from pathlib import Path
from typing import Any, Callable, Iterator, Mapping, cast


CANONICAL_RUNTIME_ROOT = Path("/var/lib/bioxp-oem-runtime")
RUNTIME_ROOT_ENV_NAMES = (
    "BIOXP_OEM_RUNTIME_STATE_ROOT",
    "BIOXP_OEM_RUNTIME_ROOT",
)
SCHEMA_VERSION = 5
RUNTIME_AUDIT_MIGRATION_VERSION = 1

RUNTIME_RELEASE_RECEIPTS_DDL = """
CREATE TABLE IF NOT EXISTS runtime_release_receipts (
    receipt_id TEXT PRIMARY KEY,
    release_id TEXT NOT NULL,
    deployment_receipt_id TEXT NOT NULL,
    systemd_invocation_id TEXT NOT NULL,
    application_pid INTEGER NOT NULL CHECK(application_pid > 1),
    application_cgroup TEXT NOT NULL,
    application_cgroup_sha256 TEXT NOT NULL,
    application_start_time_ticks INTEGER NOT NULL CHECK(application_start_time_ticks > 0),
    application_started_at REAL NOT NULL,
    canonical_receipt_sha256 TEXT NOT NULL,
    source_manifest_sha256 TEXT NOT NULL,
    source_aggregate_sha256 TEXT NOT NULL,
    image_id TEXT NOT NULL,
    image_inspection_receipt_sha256 TEXT NOT NULL,
    udocker_path TEXT NOT NULL,
    udocker_sha256 TEXT NOT NULL,
    udocker_tree_sha256 TEXT NOT NULL,
    unit_sha256 TEXT NOT NULL,
    launcher_sha256 TEXT NOT NULL,
    configuration_sha256 TEXT NOT NULL,
    oem_lock_sha256 TEXT NOT NULL,
    declared_listener_json TEXT NOT NULL CHECK(json_valid(declared_listener_json)),
    observed_listener_json TEXT NOT NULL CHECK(json_valid(observed_listener_json)),
    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
    receipt_sha256 TEXT NOT NULL,
    recorded_at REAL NOT NULL
);
"""
RUNTIME_RELEASE_RECEIPTS_INDEX_DDL = """
CREATE INDEX IF NOT EXISTS runtime_release_receipts_time_idx
    ON runtime_release_receipts(recorded_at DESC, receipt_id);
"""
_V4_APPEND_ONLY_TABLES = (
    "runtime_release_receipts",
)
RUNTIME_RELEASE_RECEIPTS_TRIGGER_DDL = (
    """
    CREATE TRIGGER IF NOT EXISTS runtime_release_receipts_append_only_update
    BEFORE UPDATE ON runtime_release_receipts
    BEGIN
        SELECT RAISE(ABORT, 'append-only table cannot be updated');
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS runtime_release_receipts_append_only_delete
    BEFORE DELETE ON runtime_release_receipts
    BEGIN
        SELECT RAISE(ABORT, 'append-only table cannot be deleted');
    END;
    """,
)

NONTERMINAL_COMMAND_STATES = frozenset(
    {
        "reserved",
        "queued",
        "admitted",
        "dispatched",
        "acknowledged",
        "executing",
        "running",
        "blocked",
    }
)
TERMINAL_COMMAND_STATES = frozenset(
    {
        "completed",
        "cleared",
        "failed",
        "rejected",
        "ambiguous",
        "cancelled",
        "outcome_unknown",
        "reconciliation_required",
        "observed",
    }
)
STARTUP_RECONCILIATION_STATES = frozenset(
    {*NONTERMINAL_COMMAND_STATES, "reconciliation_required"}
)
NONREPLAYABLE_INTERRUPT_ACTIONS = frozenset(
    {
        "meta.emergency_stop",
        "oem.x.stop",
        "oem.y.stop",
        "oem.z.stop",
        "oem.z.abort",
        "oem.abort_all",
    }
)
TERMINAL_STATES = TERMINAL_COMMAND_STATES
RUNTIME_LIFECYCLE_LOCK_NAME = "runtime-storage.lifecycle.lock"


class RuntimeAuditStoreError(RuntimeError):
    """The robot audit authority cannot guarantee a durable operation claim."""


@dataclass(frozen=True)
class RuntimeMigrationIdentity:
    version: int
    name: str
    ddl_sha256: str


@dataclass
class RuntimeWriteCoordinator:
    """One process-wide serialization boundary for a canonical runtime DB."""

    root: Path
    _raw_lock: threading.RLock = field(default_factory=threading.RLock)
    _state_lock: threading.Lock = field(default_factory=threading.Lock)
    _owner_thread_id: int | None = None
    _active_depth: int = 0
    _waiting_writers: int = 0

    @property
    def lock(self) -> "RuntimeWriteCoordinator":
        """Expose this instrumented coordinator as the existing writer lock."""
        return self

    def acquire(self, blocking: bool = True, timeout: float = -1) -> bool:
        thread_id = threading.get_ident()
        with self._state_lock:
            reentrant = self._owner_thread_id == thread_id
            if not reentrant:
                self._waiting_writers += 1
        try:
            if timeout == -1:
                acquired = self._raw_lock.acquire(blocking)
            else:
                acquired = self._raw_lock.acquire(blocking, timeout)
        finally:
            if not reentrant:
                with self._state_lock:
                    self._waiting_writers -= 1
        if acquired:
            with self._state_lock:
                if self._owner_thread_id not in {None, thread_id}:
                    self._raw_lock.release()
                    raise RuntimeError("runtime writer lock owner accounting conflict")
                self._owner_thread_id = thread_id
                self._active_depth += 1
        return acquired

    def release(self) -> None:
        thread_id = threading.get_ident()
        with self._state_lock:
            if self._owner_thread_id != thread_id or self._active_depth <= 0:
                raise RuntimeError("runtime writer lock release by non-owner")
            self._active_depth -= 1
            if self._active_depth == 0:
                self._owner_thread_id = None
        self._raw_lock.release()

    def __enter__(self) -> "RuntimeWriteCoordinator":
        self.acquire()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.release()

    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "status": "ok",
                "queue_depth": max(0, int(self._waiting_writers)),
                "active": self._active_depth > 0,
                "active_depth": max(0, int(self._active_depth)),
                "owner_thread_id": self._owner_thread_id,
            }


_COORDINATORS_LOCK = threading.Lock()
_COORDINATORS: dict[Path, RuntimeWriteCoordinator] = {}


def runtime_write_coordinator(root: str | Path) -> RuntimeWriteCoordinator:
    selected = Path(root).expanduser().resolve(strict=False)
    with _COORDINATORS_LOCK:
        coordinator = _COORDINATORS.get(selected)
        if coordinator is None:
            coordinator = RuntimeWriteCoordinator(selected)
            _COORDINATORS[selected] = coordinator
        return coordinator


def serialized_runtime_write(method: Callable[..., Any]) -> Callable[..., Any]:
    @wraps(method)
    def wrapped(self: "RuntimeAuditDatabase", *args: Any, **kwargs: Any) -> Any:
        with self.writer_lock:
            return method(self, *args, **kwargs)

    return wrapped

def _state_changing_pragma(operation: str) -> bool:
    """Classify PRAGMAs that can change connection or database state."""
    selected = str(operation).lstrip().upper()
    if not selected.startswith("PRAGMA "):
        return False
    body = selected[7:].strip()
    name = body.split("=", 1)[0].split("(", 1)[0].strip().split(".")[-1]
    return "=" in body or name in {
        "INCREMENTAL_VACUUM",
        "OPTIMIZE",
        "SHRINK_MEMORY",
        "WAL_CHECKPOINT",
    }


@contextmanager
def runtime_lifecycle_lock(
    root: str | Path,
    *,
    exclusive: bool,
    deadline: float | None = None,
) -> Iterator[None]:
    """Join the process-wide mutation/storage lifecycle barrier."""
    selected_root = Path(root)
    selected_root.mkdir(parents=True, exist_ok=True, mode=0o700)
    descriptor = os.open(
        selected_root / RUNTIME_LIFECYCLE_LOCK_NAME,
        os.O_CREAT | os.O_RDWR | getattr(os, "O_CLOEXEC", 0),
        0o600,
    )
    acquired = False
    try:
        os.fchmod(descriptor, 0o600)
        mode = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
        if deadline is None:
            fcntl.flock(descriptor, mode)
            acquired = True
        else:
            while True:
                try:
                    fcntl.flock(descriptor, mode | fcntl.LOCK_NB)
                    acquired = True
                    break
                except BlockingIOError as exc:
                    remaining = float(deadline) - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("runtime lifecycle lock deadline exceeded") from exc
                    time.sleep(min(0.01, remaining))
        if deadline is not None and time.monotonic() >= float(deadline):
            raise TimeoutError("runtime lifecycle lock deadline exceeded")
        yield
    finally:
        if acquired:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


class RuntimeLifecycleConnection(sqlite3.Connection):
    """SQLite connection that holds the shared lifecycle lease per transaction."""

    _lifecycle_root: Path | None = None
    _lifecycle_descriptor: int | None = None
    _exclusive_lifecycle_descriptor: int | None = None

    def bind_lifecycle_root(self, root: Path) -> None:
        self._lifecycle_root = Path(root)

    @contextmanager
    def exclusive_lifecycle(self) -> Iterator[None]:
        if self._lifecycle_descriptor is not None or self._exclusive_lifecycle_descriptor is not None:
            raise RuntimeAuditStoreError("runtime lifecycle lock is already held")
        if self._lifecycle_root is None:
            raise RuntimeAuditStoreError("runtime lifecycle root is not bound")
        descriptor = os.open(
            self._lifecycle_root / RUNTIME_LIFECYCLE_LOCK_NAME,
            os.O_CREAT | os.O_RDWR | getattr(os, "O_CLOEXEC", 0),
            0o600,
        )
        try:
            os.fchmod(descriptor, 0o600)
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            self._exclusive_lifecycle_descriptor = descriptor
            yield
        finally:
            self._exclusive_lifecycle_descriptor = None
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)

    def _acquire_lifecycle(self) -> None:
        if self._exclusive_lifecycle_descriptor is not None:
            return
        if self._lifecycle_descriptor is not None:
            return
        if self._lifecycle_root is None:
            raise RuntimeAuditStoreError("runtime lifecycle root is not bound")
        descriptor = os.open(
            self._lifecycle_root / RUNTIME_LIFECYCLE_LOCK_NAME,
            os.O_CREAT | os.O_RDWR | getattr(os, "O_CLOEXEC", 0),
            0o600,
        )
        try:
            os.fchmod(descriptor, 0o600)
            fcntl.flock(descriptor, fcntl.LOCK_SH)
        except Exception:
            os.close(descriptor)
            raise
        self._lifecycle_descriptor = descriptor

    def _release_lifecycle(self) -> None:
        descriptor = self._lifecycle_descriptor
        self._lifecycle_descriptor = None
        if descriptor is not None:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)

    def execute(self, sql: str, parameters: Any = (), /) -> sqlite3.Cursor:
        operation = str(sql).lstrip().upper()
        begins = operation.startswith("BEGIN")
        ends = operation.startswith("COMMIT") or operation.startswith("ROLLBACK")
        mutates = begins or _state_changing_pragma(operation) or operation.startswith(
            ("INSERT ", "UPDATE ", "DELETE ", "REPLACE ", "CREATE ", "ALTER ", "DROP ")
        )
        acquired = mutates and self._lifecycle_descriptor is None
        if mutates:
            self._acquire_lifecycle()
        try:
            cursor = super().execute(sql, parameters)
        except Exception:
            if acquired or ends or not self.in_transaction:
                self._release_lifecycle()
            raise
        if ends or (mutates and not self.in_transaction):
            self._release_lifecycle()
        return cursor

    def executemany(self, sql: str, seq_of_parameters: Any, /) -> sqlite3.Cursor:
        operation = str(sql).lstrip().upper()
        mutates = operation.startswith(("INSERT ", "UPDATE ", "DELETE ", "REPLACE "))
        acquired = mutates and self._lifecycle_descriptor is None
        if mutates:
            self._acquire_lifecycle()
        try:
            cursor = super().executemany(sql, seq_of_parameters)
        except Exception:
            if acquired or not self.in_transaction:
                self._release_lifecycle()
            raise
        if mutates and not self.in_transaction:
            self._release_lifecycle()
        return cursor

    def executescript(self, sql_script: str, /) -> sqlite3.Cursor:
        upper_script = sql_script.upper()
        owns_lifecycle = any(
            token in upper_script
            for token in (
                "BEGIN",
                "CREATE ",
                "ALTER ",
                "DROP ",
                "INSERT ",
                "UPDATE ",
                "DELETE ",
                "REPLACE ",
            )
        ) or any(
            _state_changing_pragma(statement)
            for statement in upper_script.split(";")
        )
        acquired = owns_lifecycle and self._lifecycle_descriptor is None
        if owns_lifecycle:
            self._acquire_lifecycle()
        try:
            cursor = super().executescript(sql_script)
        except Exception:
            if acquired or not self.in_transaction:
                self._release_lifecycle()
            raise
        if owns_lifecycle and not self.in_transaction:
            self._release_lifecycle()
        return cursor

    def commit(self) -> None:
        try:
            super().commit()
        finally:
            if not self.in_transaction:
                self._release_lifecycle()

    def rollback(self) -> None:
        try:
            super().rollback()
        finally:
            if not self.in_transaction:
                self._release_lifecycle()

    def close(self) -> None:
        try:
            super().close()
        finally:
            self._release_lifecycle()


def _ensure_directory(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.chmod(path, 0o700)


def _normalized_path(value: str | Path) -> Path:
    return Path(value).expanduser().resolve(strict=False)


def resolve_runtime_state_root(root: str | Path | None = None) -> Path:
    candidates: list[tuple[str, Path]] = []
    if root is not None and str(root).strip():
        candidates.append(("argument", _normalized_path(root)))
    for name in RUNTIME_ROOT_ENV_NAMES:
        raw = os.environ.get(name)
        if raw and raw.strip():
            candidates.append((name, _normalized_path(raw)))

    unique = {path for _, path in candidates}
    if len(unique) > 1:
        rendered = ", ".join(f"{name}={path}" for name, path in candidates)
        raise ValueError(f"conflicting BioXP runtime roots: {rendered}")

    selected = next(iter(unique), CANONICAL_RUNTIME_ROOT)
    try:
        _ensure_directory(selected)
    except OSError as exc:
        raise RuntimeAuditStoreError(
            f"canonical BioXP runtime root is unavailable: {selected}"
        ) from exc
    return selected


def runtime_state_root(root: str | Path | None = None) -> Path:
    return resolve_runtime_state_root(root)


def open_runtime_connection(
    root: str | Path | None = None,
    *,
    timeout: float = 2.0,
    isolation_level: Any = None,
    check_same_thread: bool = False,
) -> RuntimeLifecycleConnection:
    """Open the authoritative database through the shared lifecycle factory."""
    selected_root = resolve_runtime_state_root(root)
    connection = sqlite3.connect(
        selected_root / "bioxp_runtime.db",
        timeout=timeout,
        isolation_level=isolation_level,
        check_same_thread=check_same_thread,
        factory=RuntimeLifecycleConnection,
    )
    connection.bind_lifecycle_root(selected_root)
    return connection


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    )


def request_digest(payload: Mapping[str, Any]) -> str:
    identity = {
        key: payload[key]
        for key in (
            "action_id",
            "operation",
            "entrypoint_id",
            "caller_class",
            "control_class",
            "ownership_generation",
            "lifecycle_stage_id",
            "lifecycle_attempt_id",
            "source_identity",
            "requested_inputs",
        )
        if key in payload
    }
    return hashlib.sha256(canonical_json(identity).encode("utf-8")).hexdigest()


def configure_connection(connection: sqlite3.Connection) -> None:
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=FULL")
    connection.execute("PRAGMA foreign_keys=ON")
    connection.execute("PRAGMA busy_timeout=2000")
    connection.execute("PRAGMA wal_autocheckpoint=256")
    connection.execute("PRAGMA journal_size_limit=4194304")
    connection.execute("PRAGMA temp_store=MEMORY")


_SCHEMA_DDL = """
CREATE TABLE IF NOT EXISTS runtime_schema_migrations (
    version INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    ddl_sha256 TEXT NOT NULL,
    applied_at REAL NOT NULL,
    backup_sha256 TEXT NOT NULL,
    source_json_digests_json TEXT NOT NULL CHECK(json_valid(source_json_digests_json)),
    started_at REAL NOT NULL,
    finished_at REAL NOT NULL,
    result TEXT NOT NULL
) WITHOUT ROWID;
CREATE TABLE IF NOT EXISTS runtime_store_identity (
    identity_id INTEGER PRIMARY KEY CHECK(identity_id = 1),
    database_path TEXT NOT NULL,
    schema_version INTEGER NOT NULL,
    created_at REAL NOT NULL,
    updated_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS runtime_metadata (
    key TEXT PRIMARY KEY,
    value TEXT NOT NULL,
    updated_at REAL NOT NULL
) WITHOUT ROWID;
CREATE TABLE IF NOT EXISTS operator_commands (
    sequence INTEGER PRIMARY KEY AUTOINCREMENT,
    command_id TEXT NOT NULL UNIQUE,
    idempotency_key TEXT NOT NULL,
    canonical_request_sha256 TEXT NOT NULL DEFAULT '',
    operation TEXT NOT NULL DEFAULT 'operator_action',
    command_kind TEXT NOT NULL DEFAULT 'pipette',
    entrypoint_id TEXT NOT NULL DEFAULT 'unknown',
    caller_class TEXT NOT NULL DEFAULT 'operator',
    control_class TEXT NOT NULL DEFAULT 'service',
    idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1 CHECK(idempotency_replay_enabled IN (0,1)),
    action_id TEXT NOT NULL,
    status TEXT NOT NULL,
    safety_class TEXT,
    ownership_generation INTEGER NOT NULL DEFAULT 0,
    connection_generation INTEGER,
    source_identity_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(source_identity_json)),
    requested_inputs_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(requested_inputs_json)),
    effective_inputs_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(effective_inputs_json)),
    started_at TEXT NOT NULL,
    admitted_at TEXT,
    dispatched_at TEXT,
    finished_at TEXT,
    duration_ms REAL,
    delivery_verified INTEGER NOT NULL DEFAULT 0 CHECK(delivery_verified IN (0,1)),
    controller_acknowledged INTEGER NOT NULL DEFAULT 0 CHECK(controller_acknowledged IN (0,1)),
    completion_verified INTEGER NOT NULL DEFAULT 0 CHECK(completion_verified IN (0,1)),
    hardware_precondition_verified INTEGER NOT NULL DEFAULT 0 CHECK(hardware_precondition_verified IN (0,1)),
    hardware_postcondition_verified INTEGER NOT NULL DEFAULT 0 CHECK(hardware_postcondition_verified IN (0,1)),
    physical_effect_verified INTEGER NOT NULL DEFAULT 0 CHECK(physical_effect_verified IN (0,1)),
    outcome TEXT,
    failure_code TEXT,
    evidence_state TEXT,
    receipt_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(receipt_json)),
    response_summary_json TEXT CHECK(response_summary_json IS NULL OR json_valid(response_summary_json)),
    evidence_relpath TEXT,
    evidence_sha256 TEXT,
    evidence_bytes INTEGER,
    updated_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS operator_transitions (
    transition_id INTEGER PRIMARY KEY AUTOINCREMENT,
    command_id TEXT NOT NULL REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    state TEXT NOT NULL,
    observed_at REAL NOT NULL,
    detail_json TEXT CHECK(detail_json IS NULL OR json_valid(detail_json))
);
CREATE TABLE IF NOT EXISTS pipette_operations (
    pipette_operation_id TEXT PRIMARY KEY,
    command_id TEXT NOT NULL UNIQUE REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    operation TEXT NOT NULL,
    entrypoint_id TEXT NOT NULL,
    caller_class TEXT NOT NULL,
    control_class TEXT NOT NULL,
    action_id TEXT NOT NULL DEFAULT '',
    operation_class TEXT NOT NULL DEFAULT 'pipette',
    status TEXT NOT NULL,
    ownership_generation INTEGER NOT NULL,
    connection_generation INTEGER,
    protocol_job_id TEXT,
    protocol_action_id TEXT,
    lifecycle_stage_id TEXT,
    lifecycle_attempt_id TEXT,
    callback_session_id TEXT,
    requested_inputs_json TEXT NOT NULL CHECK(json_valid(requested_inputs_json)),
    effective_inputs_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(effective_inputs_json)),
    source_identity_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(source_identity_json)),
    delivery_verified INTEGER NOT NULL DEFAULT 0 CHECK(delivery_verified IN (0,1)),
    controller_acknowledged INTEGER NOT NULL DEFAULT 0 CHECK(controller_acknowledged IN (0,1)),
    completion_verified INTEGER NOT NULL DEFAULT 0 CHECK(completion_verified IN (0,1)),
    hardware_precondition_verified INTEGER NOT NULL DEFAULT 0 CHECK(hardware_precondition_verified IN (0,1)),
    hardware_postcondition_verified INTEGER NOT NULL DEFAULT 0 CHECK(hardware_postcondition_verified IN (0,1)),
    physical_effect_verified INTEGER NOT NULL DEFAULT 0 CHECK(physical_effect_verified IN (0,1)),
    outcome TEXT,
    failure_code TEXT,
    evidence_state TEXT,
    receipt_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(receipt_json)),
    created_at REAL NOT NULL,
    dispatched_at REAL,
    finished_at REAL,
    updated_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS pipette_channel_observations (
    observation_id TEXT PRIMARY KEY,
    command_id TEXT NOT NULL REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT NOT NULL REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    channel INTEGER NOT NULL CHECK(channel BETWEEN 0 AND 3),
    phase TEXT NOT NULL CHECK(phase IN ('precondition','query','ack','completion','postcondition','error','callback')),
    observed_at REAL NOT NULL,
    semantic_validity TEXT NOT NULL,
    truth_source TEXT NOT NULL,
    tip_loaded INTEGER CHECK(tip_loaded IS NULL OR tip_loaded IN (0,1)),
    pressure REAL,
    pressure_units TEXT,
    status TEXT,
    error_code INTEGER,
    firmware_class TEXT,
    detail_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(detail_json))
);
CREATE TABLE IF NOT EXISTS pipette_transport_exchanges (
    exchange_id TEXT PRIMARY KEY,
    command_id TEXT NOT NULL REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT NOT NULL REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    transaction_id TEXT,
    channel INTEGER CHECK(channel IS NULL OR channel BETWEEN 0 AND 3),
    transaction_phase TEXT NOT NULL,
    command_family INTEGER,
    matcher_name TEXT,
    tx_id INTEGER,
    tx_dlc INTEGER,
    tx_bytes_json TEXT NOT NULL DEFAULT '[]' CHECK(json_valid(tx_bytes_json)),
    expected_rx_id INTEGER,
    observed_rx_id INTEGER,
    rx_dlc INTEGER,
    rx_bytes_json TEXT NOT NULL DEFAULT '[]' CHECK(json_valid(rx_bytes_json)),
    router_generation INTEGER,
    sent_at REAL,
    received_at REAL,
    ack_at REAL,
    completion_at REAL,
    delivery_verified INTEGER NOT NULL DEFAULT 0 CHECK(delivery_verified IN (0,1)),
    semantic_match INTEGER NOT NULL DEFAULT 0 CHECK(semantic_match IN (0,1)),
    controller_acknowledged INTEGER NOT NULL DEFAULT 0 CHECK(controller_acknowledged IN (0,1)),
    completion_verified INTEGER NOT NULL DEFAULT 0 CHECK(completion_verified IN (0,1)),
    completion_before_ack INTEGER NOT NULL DEFAULT 0 CHECK(completion_before_ack IN (0,1)),
    multipart_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(multipart_json)),
    raw_exchange_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(raw_exchange_json)),
    CHECK(tx_id IS NULL OR expected_rx_id = (tx_id | 1024))
);
CREATE TABLE IF NOT EXISTS runtime_events (
    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
    command_id TEXT REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    event_source TEXT NOT NULL,
    event_kind TEXT NOT NULL,
    event_json TEXT NOT NULL CHECK(json_valid(event_json)),
    channel INTEGER CHECK(channel IS NULL OR channel BETWEEN 0 AND 3),
    stream_session_id TEXT,
    transaction_id TEXT,
    reader_generation INTEGER,
    ownership_generation INTEGER,
    source_sequence INTEGER,
    semantic_validity TEXT,
    observed_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS pipette_pressure_streams (
    stream_session_id TEXT PRIMARY KEY,
    command_id TEXT NOT NULL REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT NOT NULL REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    channels_json TEXT NOT NULL CHECK(json_valid(channels_json)),
    sample_period_ms REAL,
    started_at REAL NOT NULL,
    stopped_at REAL,
    source_generation INTEGER,
    reader_generation INTEGER,
    offset_identity TEXT,
    terminal_state TEXT NOT NULL,
    loss_count INTEGER NOT NULL DEFAULT 0
);
CREATE TABLE IF NOT EXISTS pipette_pressure_chunks (
    chunk_id TEXT PRIMARY KEY,
    stream_session_id TEXT NOT NULL REFERENCES pipette_pressure_streams(stream_session_id) ON DELETE RESTRICT,
    channel INTEGER NOT NULL CHECK(channel BETWEEN 0 AND 3),
    chunk_sequence INTEGER NOT NULL CHECK(chunk_sequence >= 0),
    first_sample_sequence INTEGER,
    last_sample_sequence INTEGER,
    first_time REAL,
    last_time REAL,
    sample_count INTEGER NOT NULL CHECK(sample_count >= 0),
    lost_sample_count INTEGER NOT NULL DEFAULT 0 CHECK(lost_sample_count >= 0),
    units TEXT NOT NULL,
    raw_min REAL,
    raw_max REAL,
    raw_mean REAL,
    corrected_min REAL,
    corrected_max REAL,
    corrected_mean REAL,
    offset_identity TEXT,
    chunk_schema TEXT NOT NULL,
    byte_count INTEGER NOT NULL CHECK(byte_count >= 0),
    sha256 TEXT NOT NULL,
    evidence_artifact_id TEXT,
    sample_summary_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(sample_summary_json)),
    UNIQUE(stream_session_id, channel, chunk_sequence)
);
CREATE TABLE IF NOT EXISTS runtime_evidence_links (
    evidence_link_id INTEGER PRIMARY KEY AUTOINCREMENT,
    evidence_artifact_id TEXT NOT NULL REFERENCES runtime_evidence_objects(evidence_artifact_id) ON DELETE RESTRICT,
    target_kind TEXT NOT NULL CHECK(target_kind IN (
        'command','pipette_operation','channel_observation','transport_exchange',
        'runtime_event','pressure_stream','pressure_chunk','transition',
        'assessment','migration','export'
    )),
    target_identity TEXT NOT NULL,
    command_id TEXT REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    link_kind TEXT NOT NULL,
    created_at REAL NOT NULL,
    CHECK(
        (target_kind='command' AND command_id=target_identity AND pipette_operation_id IS NULL)
        OR (target_kind='pipette_operation' AND pipette_operation_id=target_identity)
        OR (target_kind NOT IN ('command','pipette_operation') AND command_id IS NULL AND pipette_operation_id IS NULL)
    )
);
CREATE TABLE IF NOT EXISTS runtime_evidence_objects (
    evidence_artifact_id TEXT PRIMARY KEY,
    command_id TEXT REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    original_relpath TEXT NOT NULL,
    active_relpath TEXT,
    sha256 TEXT NOT NULL,
    byte_count INTEGER NOT NULL CHECK(byte_count >= 0),
    created_at REAL NOT NULL,
    retention_deadline REAL,
    legal_hold INTEGER NOT NULL DEFAULT 0 CHECK(legal_hold IN (0,1)),
    expiry_state TEXT NOT NULL,
    expiry_receipt_id TEXT,
    updated_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS runtime_evidence_events (
    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
    evidence_artifact_id TEXT NOT NULL REFERENCES runtime_evidence_objects(evidence_artifact_id) ON DELETE RESTRICT,
    event_kind TEXT NOT NULL,
    observed_at REAL NOT NULL,
    detail_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(detail_json))
);
CREATE TABLE IF NOT EXISTS runtime_migration_receipts (
    migration_id TEXT PRIMARY KEY,
    source_kind TEXT NOT NULL,
    source_digest TEXT NOT NULL,
    source_count INTEGER NOT NULL,
    imported_count INTEGER NOT NULL,
    duplicate_count INTEGER NOT NULL DEFAULT 0,
    quarantined_count INTEGER NOT NULL,
    status TEXT NOT NULL,
    archive_relpath TEXT,
    retired_at REAL,
    retirement_sha256 TEXT,
    created_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS runtime_migration_retirements (
    migration_id TEXT PRIMARY KEY,
    source_digest TEXT NOT NULL,
    archive_relpath TEXT NOT NULL,
    retired_at REAL NOT NULL,
    retirement_sha256 TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS runtime_migration_evidence (
    migration_id TEXT PRIMARY KEY,
    source_path TEXT NOT NULL,
    source_digest TEXT NOT NULL,
    source_bytes INTEGER NOT NULL CHECK(source_bytes >= 0),
    source_count INTEGER NOT NULL CHECK(source_count >= 0),
    imported_count INTEGER NOT NULL CHECK(imported_count >= 0),
    duplicate_count INTEGER NOT NULL CHECK(duplicate_count >= 0),
    quarantined_count INTEGER NOT NULL CHECK(quarantined_count >= 0),
    backup_relpath TEXT,
    archive_relpath TEXT,
    created_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS report_exports (
    export_id TEXT PRIMARY KEY,
    format TEXT NOT NULL CHECK(format IN ('json','csv')),
    filter_json TEXT NOT NULL CHECK(json_valid(filter_json)),
    filter_sha256 TEXT NOT NULL,
    snapshot_json TEXT NOT NULL CHECK(json_valid(snapshot_json)),
    row_count INTEGER NOT NULL CHECK(row_count >= 0),
    sha256 TEXT NOT NULL,
    byte_count INTEGER NOT NULL CHECK(byte_count >= 0),
    status TEXT NOT NULL,
    artifact_relpath TEXT NOT NULL,
    created_at REAL NOT NULL,
    completed_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS serial206_receipts (
    stream TEXT NOT NULL,
    receipt_id TEXT NOT NULL,
    command_id TEXT,
    idempotency_key TEXT,
    idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1 CHECK(idempotency_replay_enabled IN (0,1)),
    status TEXT,
    observed_at REAL NOT NULL,
    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
    PRIMARY KEY(stream, receipt_id)
) WITHOUT ROWID;
CREATE INDEX IF NOT EXISTS operator_commands_history_idx
    ON operator_commands(sequence DESC);
CREATE INDEX IF NOT EXISTS operator_commands_updated_idx
    ON operator_commands(updated_at DESC, sequence DESC);
CREATE INDEX IF NOT EXISTS operator_commands_action_status_idx
    ON operator_commands(action_id, status, sequence DESC);
CREATE INDEX IF NOT EXISTS operator_transitions_command_idx
    ON operator_transitions(command_id, transition_id);
CREATE INDEX IF NOT EXISTS pipette_operations_command_idx
    ON pipette_operations(command_id);
CREATE INDEX IF NOT EXISTS pipette_channel_observations_operation_idx
    ON pipette_channel_observations(pipette_operation_id, observed_at, observation_id);
CREATE INDEX IF NOT EXISTS pipette_transport_exchanges_operation_idx
    ON pipette_transport_exchanges(pipette_operation_id, sent_at, exchange_id);
CREATE INDEX IF NOT EXISTS runtime_events_command_idx
    ON runtime_events(command_id, event_id);
CREATE INDEX IF NOT EXISTS runtime_events_kind_idx
    ON runtime_events(event_kind, observed_at DESC, event_id DESC);
CREATE INDEX IF NOT EXISTS runtime_evidence_links_command_idx
    ON runtime_evidence_links(command_id, evidence_link_id);
CREATE INDEX IF NOT EXISTS runtime_evidence_links_target_idx
    ON runtime_evidence_links(target_kind, target_identity, evidence_link_id);
CREATE INDEX IF NOT EXISTS runtime_evidence_objects_command_idx
    ON runtime_evidence_objects(command_id, created_at, evidence_artifact_id);
CREATE INDEX IF NOT EXISTS runtime_evidence_events_artifact_idx
    ON runtime_evidence_events(evidence_artifact_id, event_id);
CREATE INDEX IF NOT EXISTS report_exports_time_idx
    ON report_exports(created_at DESC, export_id);
CREATE INDEX IF NOT EXISTS serial206_receipts_command_idx
    ON serial206_receipts(stream, command_id);
CREATE UNIQUE INDEX IF NOT EXISTS serial206_receipts_idempotency_idx
    ON serial206_receipts(stream, idempotency_key)
    WHERE idempotency_key IS NOT NULL AND idempotency_replay_enabled=1;
CREATE INDEX IF NOT EXISTS serial206_receipts_time_idx
    ON serial206_receipts(stream, observed_at DESC);
CREATE INDEX IF NOT EXISTS pipette_pressure_streams_operation_idx
    ON pipette_pressure_streams(pipette_operation_id, started_at, stream_session_id);
CREATE INDEX IF NOT EXISTS pipette_pressure_chunks_stream_idx
    ON pipette_pressure_chunks(stream_session_id, channel, chunk_sequence);
CREATE TABLE IF NOT EXISTS reference_state_authority (
    authority_key TEXT PRIMARY KEY CHECK(authority_key='reference_state'),
    payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
    payload_sha256 TEXT NOT NULL CHECK(length(payload_sha256)=64),
    updated_at REAL NOT NULL
) WITHOUT ROWID;
CREATE TRIGGER IF NOT EXISTS reference_state_authority_coherence_v1
BEFORE INSERT ON reference_state_authority
WHEN NEW.payload_json<>canonical_json(NEW.payload_json)
  OR NEW.payload_sha256<>sha256_utf8(NEW.payload_json)
BEGIN SELECT RAISE(ABORT,'reference authority bytes are incoherent'); END;
CREATE TRIGGER IF NOT EXISTS reference_state_authority_authorized_insert_v1
BEFORE INSERT ON reference_state_authority
WHEN reference_write_allowed()<>1
BEGIN SELECT RAISE(ABORT,'reference authority writer is not authoritative'); END;
CREATE TRIGGER IF NOT EXISTS reference_state_authority_authorized_update_v1
BEFORE UPDATE ON reference_state_authority
WHEN reference_write_allowed()<>1
  OR NEW.authority_key IS NOT OLD.authority_key
  OR NEW.payload_json<>canonical_json(NEW.payload_json)
  OR NEW.payload_sha256<>sha256_utf8(NEW.payload_json)
BEGIN SELECT RAISE(ABORT,'reference authority update is not authoritative'); END;
CREATE TRIGGER IF NOT EXISTS reference_state_authority_no_delete_v1
BEFORE DELETE ON reference_state_authority
BEGIN SELECT RAISE(ABORT,'reference authority cannot be deleted'); END;
"""

_APPEND_ONLY_TABLES = (
    "runtime_schema_migrations",
    "operator_transitions",
    "runtime_events",
    "runtime_evidence_links",
    "runtime_evidence_events",
    "runtime_migration_receipts",
    "runtime_migration_retirements",
    "runtime_migration_evidence",
    "report_exports",
    "pipette_channel_observations",
    "pipette_transport_exchanges",
    "pipette_pressure_chunks",
)


def runtime_audit_migration_identity() -> RuntimeMigrationIdentity:
    ddl_source = "\n".join(
        (
            _SCHEMA_DDL,
            inspect.getsource(_add_missing_columns),
            inspect.getsource(_install_append_only_triggers),
        )
    ).encode("utf-8")
    return RuntimeMigrationIdentity(
        version=RUNTIME_AUDIT_MIGRATION_VERSION,
        name="runtime_audit_foundation",
        ddl_sha256=hashlib.sha256(ddl_source).hexdigest(),
    )


def assert_migration_slot(
    connection: sqlite3.Connection,
    identity: RuntimeMigrationIdentity,
) -> bool:
    table = connection.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name='runtime_schema_migrations'"
    ).fetchone()
    if table is None:
        return False
    row = connection.execute(
        "SELECT name,ddl_sha256 FROM runtime_schema_migrations WHERE version=?",
        (identity.version,),
    ).fetchone()
    if row is None:
        return False
    if str(row[0]) != identity.name or str(row[1]) != identity.ddl_sha256:
        raise RuntimeAuditStoreError(
            "occupied runtime migration version has mismatched name or digest: "
            f"version={identity.version} expected={identity.name}:{identity.ddl_sha256} "
            f"found={row[0]}:{row[1]}"
        )
    return True

def expected_runtime_migration_ledger() -> tuple[dict[str, Any], ...]:
    """Return the exact canonical ordered v1-v4 migration registry."""
    from .oem_runtime_store import canonical_runtime_migration_registry

    return tuple(
        {
            "version": identity.version,
            "name": identity.name,
            "ddl_sha256": identity.ddl_sha256,
        }
        for identity in canonical_runtime_migration_registry()
    )


def expected_runtime_schema_objects() -> dict[tuple[str, str], str]:
    """Return normalized SQLite-owned SQL for every audit-owned object."""
    expected = _expected_foundation_connection()
    try:
        expected.executescript(RUNTIME_RELEASE_RECEIPTS_DDL)
        expected.executescript(RUNTIME_RELEASE_RECEIPTS_INDEX_DDL)
        for statement in RUNTIME_RELEASE_RECEIPTS_TRIGGER_DDL:
            expected.executescript(statement)
        return dict(_foundation_schema_manifest(expected)["objects"])
    finally:
        expected.close()


def _add_missing_columns(connection: sqlite3.Connection) -> None:
    additions_by_table = {
        "operator_commands": {
            "idempotency_replay_enabled": "INTEGER NOT NULL DEFAULT 1 CHECK(idempotency_replay_enabled IN (0,1))",
            "canonical_request_sha256": "TEXT NOT NULL DEFAULT ''",
            "operation": "TEXT NOT NULL DEFAULT 'operator_action'",
            "command_kind": "TEXT NOT NULL DEFAULT 'pipette'",
            "entrypoint_id": "TEXT NOT NULL DEFAULT 'unknown'",
            "caller_class": "TEXT NOT NULL DEFAULT 'operator'",
            "control_class": "TEXT NOT NULL DEFAULT 'service'",
            "source_identity_json": "TEXT NOT NULL DEFAULT '{}'",
            "requested_inputs_json": "TEXT NOT NULL DEFAULT '{}'",
            "effective_inputs_json": "TEXT NOT NULL DEFAULT '{}'",
            "connection_generation": "INTEGER",
            "admitted_at": "TEXT",
            "dispatched_at": "TEXT",
            "delivery_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(delivery_verified IN (0,1))",
            "completion_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(completion_verified IN (0,1))",
            "hardware_precondition_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(hardware_precondition_verified IN (0,1))",
            "hardware_postcondition_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(hardware_postcondition_verified IN (0,1))",
            "outcome": "TEXT",
            "failure_code": "TEXT",
            "evidence_state": "TEXT",
        },
        "pipette_operations": {
            "action_id": "TEXT NOT NULL DEFAULT ''",
            "operation_class": "TEXT NOT NULL DEFAULT 'pipette'",
            "connection_generation": "INTEGER",
            "protocol_job_id": "TEXT",
            "protocol_action_id": "TEXT",
            "lifecycle_stage_id": "TEXT",
            "lifecycle_attempt_id": "TEXT",
            "callback_session_id": "TEXT",
            "source_identity_json": "TEXT NOT NULL DEFAULT '{}'",
            "delivery_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(delivery_verified IN (0,1))",
            "controller_acknowledged": "INTEGER NOT NULL DEFAULT 0 CHECK(controller_acknowledged IN (0,1))",
            "completion_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(completion_verified IN (0,1))",
            "hardware_precondition_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(hardware_precondition_verified IN (0,1))",
            "hardware_postcondition_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(hardware_postcondition_verified IN (0,1))",
            "physical_effect_verified": "INTEGER NOT NULL DEFAULT 0 CHECK(physical_effect_verified IN (0,1))",
            "outcome": "TEXT",
            "failure_code": "TEXT",
            "evidence_state": "TEXT",
            "receipt_json": "TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(receipt_json))",
            "dispatched_at": "REAL",
            "finished_at": "REAL",
        },
        "runtime_events": {
            "channel": "INTEGER CHECK(channel IS NULL OR channel BETWEEN 0 AND 3)",
            "stream_session_id": "TEXT",
            "transaction_id": "TEXT",
            "reader_generation": "INTEGER",
            "ownership_generation": "INTEGER",
            "source_sequence": "INTEGER",
            "semantic_validity": "TEXT",
        },
        "pipette_transport_exchanges": {
            "transaction_id": "TEXT",
        },
        "runtime_migration_receipts": {
            "duplicate_count": "INTEGER NOT NULL DEFAULT 0",
            "archive_relpath": "TEXT",
            "retired_at": "REAL",
            "retirement_sha256": "TEXT",
        },
    }
    for table, additions in additions_by_table.items():
        columns = {str(row["name"]) for row in connection.execute(f"PRAGMA table_info({table})")}
        for name, definition in additions.items():
            if name not in columns:
                connection.execute(f"ALTER TABLE {table} ADD COLUMN {name} {definition}")
    for index in connection.execute("PRAGMA index_list(operator_commands)").fetchall():
        if not index["unique"] or index["origin"] != "c" or index["partial"]:
            continue
        name = str(index["name"])
        indexed_columns = [
            str(row["name"])
            for row in connection.execute("SELECT name FROM pragma_index_info(?)", (name,))
        ]
        if indexed_columns == ["idempotency_key"]:
            quoted = name.replace('"', '""')
            connection.execute(f'DROP INDEX "{quoted}"')
    placeholders = ",".join("?" for _ in NONREPLAYABLE_INTERRUPT_ACTIONS)
    connection.execute(
        f"UPDATE operator_commands SET idempotency_replay_enabled=0 WHERE action_id IN ({placeholders})",
        tuple(sorted(NONREPLAYABLE_INTERRUPT_ACTIONS)),
    )


def _install_append_only_triggers(connection: sqlite3.Connection) -> None:
    for table in _APPEND_ONLY_TABLES:
        connection.execute(
            f"""
            CREATE TRIGGER IF NOT EXISTS {table}_append_only_update
            BEFORE UPDATE ON {table}
            BEGIN
                SELECT RAISE(ABORT, 'append-only table cannot be updated');
            END;
            """
        )
        connection.execute(
            f"""
            CREATE TRIGGER IF NOT EXISTS {table}_append_only_delete
            BEFORE DELETE ON {table}
            BEGIN
                SELECT RAISE(ABORT, 'append-only table cannot be deleted');
            END;
            """
        )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_evidence_objects_retention_immutable
        BEFORE UPDATE OF retention_deadline,legal_hold ON runtime_evidence_objects
        WHEN NOT (
            NEW.retention_deadline IS OLD.retention_deadline
            AND NEW.legal_hold IS OLD.legal_hold
        )
        BEGIN
            SELECT RAISE(ABORT, 'retention authority is immutable');
        END;
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_evidence_objects_delete_rejected
        BEFORE DELETE ON runtime_evidence_objects
        BEGIN
            SELECT RAISE(ABORT, 'retention authority cannot be deleted');
        END;
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_evidence_hold_assessment_shape
        BEFORE INSERT ON runtime_evidence_events
        WHEN NEW.event_kind='legal_hold_assessment'
         AND (
             COALESCE(json_type(NEW.detail_json,'$.legal_hold'),'') NOT IN ('true','false')
             OR COALESCE(json_type(NEW.detail_json,'$.actor'),'') <> 'text'
             OR LENGTH(TRIM(COALESCE(json_extract(NEW.detail_json,'$.actor'),''))) = 0
         )
        BEGIN
            SELECT RAISE(ABORT, 'legal-hold assessment requires an exact boolean and named actor');
        END;
        """
    )


def _normalized_schema_sql(value: str | None) -> str:
    return "" if value is None else "".join(str(value).upper().split())


def _foundation_schema_manifest(connection: sqlite3.Connection) -> dict[str, Any]:
    objects = connection.execute(
        """
        SELECT type,name,sql FROM sqlite_master
        WHERE type IN ('table','index','trigger')
          AND name NOT LIKE 'sqlite_%'
          AND sql IS NOT NULL
        ORDER BY type,name
        """
    ).fetchall()
    tables = tuple(str(row["name"]) for row in objects if str(row["type"]) == "table")
    return {
        "objects": {
            (str(row["type"]), str(row["name"])): _normalized_schema_sql(row["sql"])
            for row in objects
        },
        "columns": {
            table: tuple(
                (
                    str(row["name"]),
                    str(row["type"]).upper(),
                    int(row["notnull"]),
                    None if row["dflt_value"] is None else str(row["dflt_value"]),
                    int(row["pk"]),
                )
                for row in connection.execute(f'PRAGMA table_info("{table}")')
            )
            for table in tables
        },
        "foreign_keys": {
            table: tuple(tuple(row) for row in connection.execute(f'PRAGMA foreign_key_list("{table}")'))
            for table in tables
        },
        "indexes": {
            table: tuple(sorted(
                (
                    str(row["name"]),
                    int(row["unique"]),
                    str(row["origin"]),
                    int(row["partial"]),
                    tuple(
                        str(item["name"])
                        for item in connection.execute(
                            f'PRAGMA index_info("{str(row["name"]).replace(chr(34), chr(34) * 2)}")'
                        )
                    ),
                )
                for row in connection.execute(f'PRAGMA index_list("{table}")')
            ))
            for table in tables
        },
    }


def _expected_foundation_connection() -> sqlite3.Connection:
    expected = sqlite3.connect(":memory:")
    expected.row_factory = sqlite3.Row
    expected.execute("PRAGMA foreign_keys=ON")
    expected.executescript(_SCHEMA_DDL)
    expected.execute(
        """
        CREATE UNIQUE INDEX IF NOT EXISTS operator_commands_replay_key_idx
            ON operator_commands(idempotency_key)
            WHERE idempotency_replay_enabled=1
        """
    )
    _install_append_only_triggers(expected)
    return expected


def verify_runtime_audit_foundation(connection: sqlite3.Connection) -> None:
    expected = _expected_foundation_connection()
    try:
        expected_manifest = _foundation_schema_manifest(expected)
    finally:
        expected.close()
    actual_manifest = _foundation_schema_manifest(connection)
    expected_objects = expected_manifest["objects"]
    actual_objects = actual_manifest["objects"]
    missing = sorted(set(expected_objects) - set(actual_objects))
    mismatched = sorted(
        identity
        for identity, sql in expected_objects.items()
        if actual_objects.get(identity) != sql
    )
    shape_mismatches = sorted(
        table
        for table in expected_manifest["columns"]
        if actual_manifest["columns"].get(table) != expected_manifest["columns"][table]
        or actual_manifest["foreign_keys"].get(table) != expected_manifest["foreign_keys"][table]
        or actual_manifest["indexes"].get(table) != expected_manifest["indexes"][table]
    )
    if missing or mismatched or shape_mismatches:
        raise RuntimeAuditStoreError(
            "runtime audit foundation schema mismatch: "
            f"missing={missing} objects={mismatched} tables={shape_mismatches}"
        )


def _rebuild_runtime_audit_foundation(connection: sqlite3.Connection) -> None:
    expected = _expected_foundation_connection()
    try:
        manifest = _foundation_schema_manifest(expected)
        expected_objects = expected.execute(
            """
            SELECT type,name,sql FROM sqlite_master
            WHERE type IN ('table','index','trigger')
              AND name NOT LIKE 'sqlite_%' AND sql IS NOT NULL
            ORDER BY CASE type WHEN 'table' THEN 0 WHEN 'index' THEN 1 ELSE 2 END,name
            """
        ).fetchall()
    finally:
        expected.close()
    table_names = tuple(manifest["columns"])
    suffix = f"__legacy_foundation_{uuid.uuid4().hex}"
    renamed: dict[str, str] = {}
    foreign_keys_enabled = int(connection.execute("PRAGMA foreign_keys").fetchone()[0])
    connection.execute("PRAGMA foreign_keys=OFF")
    try:
        connection.execute("BEGIN IMMEDIATE")
        for object_type in ("trigger", "index"):
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type=? AND name NOT LIKE 'sqlite_%'",
                (object_type,),
            ).fetchall():
                name = str(row["name"])
                if (object_type, name) in manifest["objects"]:
                    quoted = name.replace('"', '""')
                    connection.execute(f'DROP {object_type.upper()} "{quoted}"')
        for table in table_names:
            if connection.execute(
                "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
                (table,),
            ).fetchone() is None:
                continue
            legacy = f"{table}{suffix}"
            connection.execute(
                f'ALTER TABLE "{table.replace(chr(34), chr(34) * 2)}" '
                f'RENAME TO "{legacy.replace(chr(34), chr(34) * 2)}"'
            )
            renamed[table] = legacy
        for row in expected_objects:
            if str(row["type"]) == "table":
                connection.execute(str(row["sql"]))
        for table, legacy in renamed.items():
            actual_columns = {
                str(row["name"])
                for row in connection.execute(f'PRAGMA table_info("{legacy}")')
            }
            selected = tuple(name for name, *_ in manifest["columns"][table] if name in actual_columns)
            if selected:
                projection = ",".join(
                    f'"{name.replace(chr(34), chr(34) * 2)}"' for name in selected
                )
                connection.execute(
                    f'INSERT INTO "{table.replace(chr(34), chr(34) * 2)}" ({projection}) '
                    f'SELECT {projection} FROM "{legacy.replace(chr(34), chr(34) * 2)}"'
                )
        for legacy in reversed(tuple(renamed.values())):
            connection.execute(f'DROP TABLE "{legacy.replace(chr(34), chr(34) * 2)}"')
        for row in expected_objects:
            if str(row["type"]) in {"index", "trigger"}:
                connection.execute(str(row["sql"]))
        verify_runtime_audit_foundation(connection)
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise
    finally:
        connection.execute(f"PRAGMA foreign_keys={1 if foreign_keys_enabled else 0}")
    if foreign_keys_enabled:
        violations = connection.execute("PRAGMA foreign_key_check").fetchall()
        if violations:
            raise RuntimeAuditStoreError(
                f"runtime audit foundation rebuild left foreign-key violations: {violations}"
            )


def ensure_schema(
    connection: sqlite3.Connection,
    root: Path | None = None,
    *,
    backup_sha256: str | None = None,
) -> None:
    """Apply only the canonical audit-foundation migration.

    The lifespan migration owner calls this after creating the pre-DDL backup;
    ordinary runtime store construction can opt out with ``initialize_schema=False``.
    """
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    if version > SCHEMA_VERSION:
        raise RuntimeAuditStoreError(f"unsupported runtime schema version {version}")
    configure_connection(connection)
    identity = runtime_audit_migration_identity()
    try:
        existing_exact = assert_migration_slot(connection, identity)
    except sqlite3.Error:
        if backup_sha256 is None:
            raise RuntimeAuditStoreError(
                "runtime audit foundation repair requires a verified pre-DDL backup"
            )
        _rebuild_runtime_audit_foundation(connection)
        existing_exact = assert_migration_slot(connection, identity)
    if existing_exact:
        if version < identity.version:
            raise RuntimeAuditStoreError("runtime migration ledger is ahead of PRAGMA user_version")
        try:
            verify_runtime_audit_foundation(connection)
        except RuntimeAuditStoreError:
            if backup_sha256 is None:
                raise RuntimeAuditStoreError(
                    "runtime audit foundation repair requires a verified pre-DDL backup"
                )
            _rebuild_runtime_audit_foundation(connection)
        verify_runtime_audit_foundation(connection)
        return
    if backup_sha256 is None:
        raise RuntimeAuditStoreError(
            "runtime audit foundation requires a verified pre-DDL backup from the lifespan migration owner"
        )
    preexisting_objects = connection.execute(
        """
        SELECT COUNT(*) FROM sqlite_master
        WHERE type IN ('table','index','trigger') AND name NOT LIKE 'sqlite_%'
        """
    ).fetchone()[0]
    if int(preexisting_objects) > 0:
        _rebuild_runtime_audit_foundation(connection)
    try:
        connection.executescript(_SCHEMA_DDL)
        migration_columns = {
            str(row["name"])
            for row in connection.execute("PRAGMA table_info(runtime_schema_migrations)")
        }
        for name, definition in {
            "name": "TEXT",
            "ddl_sha256": "TEXT",
            "applied_at": "REAL",
            "backup_sha256": "TEXT",
            "source_json_digests_json": "TEXT",
            "started_at": "REAL",
            "finished_at": "REAL",
            "result": "TEXT",
        }.items():
            if name not in migration_columns:
                connection.execute(
                    f"ALTER TABLE runtime_schema_migrations ADD COLUMN {name} {definition}"
                )
        connection.execute("BEGIN IMMEDIATE")
        _add_missing_columns(connection)
        connection.execute(
            """
            CREATE UNIQUE INDEX IF NOT EXISTS operator_commands_replay_key_idx
                ON operator_commands(idempotency_key)
                WHERE idempotency_replay_enabled=1
            """
        )
        _install_append_only_triggers(connection)
        now = time.time()
        if not existing_exact:
            occupied = connection.execute(
                "SELECT name,ddl_sha256 FROM runtime_schema_migrations WHERE version=?",
                (identity.version,),
            ).fetchone()
            if occupied is not None:
                raise RuntimeAuditStoreError(
                    "runtime migration version became occupied before canonical publication"
                )
            connection.execute(
                """
                INSERT INTO runtime_schema_migrations(
                    version,name,ddl_sha256,applied_at,backup_sha256,source_json_digests_json,
                    started_at,finished_at,result
                ) VALUES(?,?,?,?,?,?,?,?,?)
                """,
                (
                    identity.version,
                    identity.name,
                    identity.ddl_sha256,
                    now,
                    backup_sha256,
                    "{}",
                    now,
                    now,
                    "committed",
                ),
            )
        if root is not None:
            db_path = str((root / "bioxp_runtime.db").resolve())
            connection.execute(
                """
                INSERT INTO runtime_store_identity(identity_id, database_path, schema_version, created_at, updated_at)
                VALUES(1,?,?,?,?)
                ON CONFLICT(identity_id) DO UPDATE SET
                    database_path=excluded.database_path,
                    schema_version=MAX(runtime_store_identity.schema_version, excluded.schema_version),
                    updated_at=excluded.updated_at
                """,
                (db_path, RUNTIME_AUDIT_MIGRATION_VERSION, now, now),
            )
        connection.execute(
            f"PRAGMA user_version={max(version, RUNTIME_AUDIT_MIGRATION_VERSION)}"
        )
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise


class RuntimeAuditDatabase:
    def __init__(
        self,
        root: str | Path | None = None,
        *,
        initialize_schema: bool = False,
        pre_ddl_backup_sha256: str | None = None,
    ) -> None:
        self.root = resolve_runtime_state_root(root)
        self.path = self.root / "bioxp_runtime.db"
        self.coordinator = runtime_write_coordinator(self.root)
        self.writer_lock = self.coordinator.lock
        try:
            self.connection = open_runtime_connection(
                self.root,
                timeout=2.0,
                isolation_level=None,
                check_same_thread=False,
            )
            configure_connection(self.connection)
            if initialize_schema:
                raise RuntimeAuditStoreError(
                    "constructor-owned migration is retired; call the explicit canonical migration utility"
                )
        except (OSError, sqlite3.Error) as exc:
            raise RuntimeAuditStoreError(
                f"canonical BioXP audit database is unavailable: {self.path}"
            ) from exc

    def _begin_write(self) -> bool:
        """Begin a writer transaction unless the caller already owns one."""
        if self.connection.in_transaction:
            return False
        self.connection.execute("BEGIN IMMEDIATE")
        return True

    def _commit_write(self, owns_transaction: bool) -> None:
        if owns_transaction:
            self.connection.execute("COMMIT")

    def _rollback_write(self, owns_transaction: bool) -> None:
        if owns_transaction and self.connection.in_transaction:
            self.connection.execute("ROLLBACK")

    @contextmanager
    def pipette_finalization_transaction(self) -> Iterator[None]:
        """Commit normalized pipette evidence and its terminal CAS together."""
        with self.writer_lock:
            owns_transaction = self._begin_write()
            if not owns_transaction:
                raise RuntimeAuditStoreError(
                    "pipette finalization transaction must own the outer write"
                )
            try:
                yield
                self._commit_write(owns_transaction)
            except Exception:
                self._rollback_write(owns_transaction)
                raise

    @serialized_runtime_write
    def claim(
        self,
        payload: Mapping[str, Any],
        *,
        pipette: bool = False,
        attach_to_existing_command: bool = False,
    ) -> tuple[dict[str, Any], bool]:
        required = (
            "command_id",
            "idempotency_key",
            "action_id",
            "operation",
            "entrypoint_id",
            "caller_class",
            "control_class",
            "ownership_generation",
            "requested_inputs",
        )
        missing = [name for name in required if not str(payload.get(name, "")).strip() and name != "ownership_generation"]
        if missing:
            raise ValueError(f"runtime claim missing fields: {missing}")
        digest = request_digest(payload)
        command_id = str(payload["command_id"])
        idempotency_key = str(payload["idempotency_key"])
        now = time.time()
        started_at = str(payload.get("started_at") or now)
        source_identity = payload.get("source_identity") or {"authority": "robot_runtime"}
        requested_inputs = payload.get("requested_inputs") or {}
        action_id = str(payload["action_id"])
        connection = self.connection

        def ensure_pipette_child(row: sqlite3.Row | Mapping[str, Any]) -> str:
            existing_operation = connection.execute(
                "SELECT pipette_operation_id FROM pipette_operations WHERE command_id=?",
                (str(row["command_id"]),),
            ).fetchone()
            if existing_operation is not None:
                return str(existing_operation["pipette_operation_id"])
            operation_id = str(payload.get("pipette_operation_id") or f"pipette.{row['command_id']}")
            connection.execute(
                """
                INSERT INTO pipette_operations(
                    pipette_operation_id,command_id,operation,entrypoint_id,caller_class,
                    control_class,action_id,operation_class,status,ownership_generation,
                    connection_generation,protocol_job_id,protocol_action_id,lifecycle_stage_id,
                    lifecycle_attempt_id,callback_session_id,requested_inputs_json,source_identity_json,created_at,updated_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    operation_id,
                    str(row["command_id"]),
                    str(payload.get("operation") or row["operation"]),
                    str(payload.get("entrypoint_id") or row["entrypoint_id"]),
                    str(payload.get("caller_class") or row["caller_class"]),
                    str(payload.get("control_class") or row["control_class"]),
                    str(payload.get("action_id") or row["action_id"]),
                    str(payload.get("operation_class") or "pipette"),
                    str(row["status"]),
                    int(payload.get("ownership_generation") or row["ownership_generation"] or 0),
                    payload.get("connection_generation"),
                    payload.get("protocol_job_id"),
                    payload.get("protocol_action_id"),
                    payload.get("lifecycle_stage_id"),
                    payload.get("lifecycle_attempt_id"),
                    payload.get("callback_session_id"),
                    canonical_json(payload.get("requested_inputs") or {}),
                    canonical_json(payload.get("source_identity") or {}),
                    now,
                    now,
                ),
            )
            return operation_id

        def claim_projection(row: sqlite3.Row | Mapping[str, Any]) -> dict[str, Any]:
            result = dict(row)
            if pipette:
                result["pipette_operation_id"] = ensure_pipette_child(row)
            return result

        owns_transaction = False
        try:
            owns_transaction = self._begin_write()
            if attach_to_existing_command:
                if not pipette:
                    raise ValueError("only pipette children can attach to an existing command")
                existing_command = connection.execute(
                    "SELECT * FROM operator_commands WHERE command_id=?",
                    (command_id,),
                ).fetchone()
                if existing_command is None:
                    raise ValueError("operator command does not exist for pipette child attachment")
                if str(existing_command["idempotency_key"]) != idempotency_key:
                    raise ValueError("pipette child idempotency key does not match operator command")
                if int(existing_command["ownership_generation"] or 0) != int(payload["ownership_generation"]):
                    raise ValueError("pipette child ownership generation does not match operator command")
                existing_child = connection.execute(
                    "SELECT pipette_operation_id FROM pipette_operations WHERE command_id=?",
                    (command_id,),
                ).fetchone()
                projection = claim_projection(existing_command)
                self._commit_write(owns_transaction)
                return projection, existing_child is None
            existing = connection.execute(
                "SELECT * FROM operator_commands WHERE idempotency_key=? AND idempotency_replay_enabled=1",
                (idempotency_key,),
            ).fetchone()
            if existing is not None:
                existing_digest = str(existing["canonical_request_sha256"] or "")
                if existing_digest and existing_digest != digest:
                    raise ValueError("idempotency key conflict")
                projection = claim_projection(existing)
                self._commit_write(owns_transaction)
                return projection, False

            existing_command = connection.execute(
                "SELECT * FROM operator_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if existing_command is not None:
                if str(existing_command["canonical_request_sha256"] or "") != digest:
                    raise ValueError("command identity conflict")
                projection = claim_projection(existing_command)
                self._commit_write(owns_transaction)
                return projection, False

            receipt_json = canonical_json(
                {
                    "command_id": command_id,
                    "idempotency_key": idempotency_key,
                    "action_id": action_id,
                    "operation": str(payload["operation"]),
                    "entrypoint_id": str(payload["entrypoint_id"]),
                    "caller_class": str(payload["caller_class"]),
                    "control_class": str(payload["control_class"]),
                    "ownership_generation": int(payload["ownership_generation"]),
                    "connection_generation": payload.get("connection_generation"),
                    "protocol_job_id": payload.get("protocol_job_id"),
                    "protocol_action_id": payload.get("protocol_action_id"),
                    "lifecycle_stage_id": payload.get("lifecycle_stage_id"),
                    "lifecycle_attempt_id": payload.get("lifecycle_attempt_id"),
                    "callback_session_id": payload.get("callback_session_id"),
                    "requested_inputs": requested_inputs,
                    "status": "reserved",
                }
            )
            connection.execute(
                """
                INSERT INTO operator_commands(
                    command_id,idempotency_key,canonical_request_sha256,operation,command_kind,entrypoint_id,
                    caller_class,control_class,idempotency_replay_enabled,action_id,status,
                    safety_class,ownership_generation,connection_generation,source_identity_json,requested_inputs_json,
                    effective_inputs_json,started_at,receipt_json,updated_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    command_id,
                    idempotency_key,
                    digest,
                    str(payload["operation"]),
                    str(payload.get("command_kind") or ("pipette" if pipette else "operator")),
                    str(payload["entrypoint_id"]),
                    str(payload["caller_class"]),
                    str(payload["control_class"]),
                    1 if payload.get("idempotency_replay_enabled", True) else 0,
                    action_id,
                    "reserved",
                    payload.get("safety_class"),
                    int(payload["ownership_generation"]),
                    payload.get("connection_generation"),
                    canonical_json(source_identity),
                    canonical_json(requested_inputs),
                    canonical_json(payload.get("effective_inputs") or {}),
                    started_at,
                    receipt_json,
                    now,
                ),
            )
            connection.execute(
                "INSERT INTO operator_transitions(command_id,state,observed_at,detail_json) VALUES(?,?,?,?)",
                (command_id, "reserved", now, canonical_json({"claim_digest": digest})),
            )
            if pipette:
                operation_id = str(payload.get("pipette_operation_id") or command_id)
                connection.execute(
                    """
                    INSERT INTO pipette_operations(
                        pipette_operation_id,command_id,operation,entrypoint_id,caller_class,
                    control_class,action_id,operation_class,status,ownership_generation,
                    connection_generation,protocol_job_id,protocol_action_id,lifecycle_stage_id,
                    lifecycle_attempt_id,callback_session_id,requested_inputs_json,source_identity_json,created_at,updated_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                    """,
                    (
                        operation_id,
                        command_id,
                        str(payload["operation"]),
                        str(payload["entrypoint_id"]),
                        str(payload["caller_class"]),
                        str(payload["control_class"]),
                        action_id,
                        str(payload.get("operation_class") or "pipette"),
 "reserved",
 int(payload["ownership_generation"]),
 payload.get("connection_generation"),
 payload.get("protocol_job_id"),
 payload.get("protocol_action_id"),
 payload.get("lifecycle_stage_id"),
 payload.get("lifecycle_attempt_id"),
 payload.get("callback_session_id"),
 canonical_json(requested_inputs),
                        canonical_json(source_identity),
                        now,
                        now,
                    ),
                )
            row = connection.execute(
                "SELECT * FROM operator_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if row is None:
                raise RuntimeAuditStoreError("durable claim disappeared before commit")
            projection = claim_projection(row)
            self._commit_write(owns_transaction)
            return projection, True
        except Exception:
            self._rollback_write(owns_transaction)
            raise

    @serialized_runtime_write
    def attach_pipette_child(self, payload: Mapping[str, Any]) -> tuple[dict[str, Any], bool]:
        """Attach one pipette operation without replacing the outer claim digest."""
        command_id = str(payload.get("command_id") or "")
        idempotency_key = str(payload.get("idempotency_key") or "")
        if not command_id or not idempotency_key:
            raise ValueError("pipette child attachment requires command_id and idempotency_key")
        requested_json = canonical_json(payload.get("requested_inputs") or {})
        source_json = canonical_json(payload.get("source_identity") or {})
        connection = self.connection
        try:
            connection.execute("BEGIN IMMEDIATE")
            outer = connection.execute(
                "SELECT * FROM operator_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if outer is None:
                raise ValueError("outer operator claim not found")
            if str(outer["idempotency_key"]) != idempotency_key:
                raise ValueError("idempotency key conflict")
            if int(outer["ownership_generation"] or 0) != int(payload.get("ownership_generation") or 0):
                raise ValueError("ownership generation conflict")
            if str(outer["action_id"] or "") != str(payload.get("action_id") or ""):
                raise ValueError("trusted operator action identity conflict")
            existing = connection.execute(
                "SELECT * FROM pipette_operations WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if existing is not None:
                expected = {
                    "operation": str(payload.get("operation") or ""),
                    "entrypoint_id": str(payload.get("entrypoint_id") or ""),
                    "caller_class": str(payload.get("caller_class") or ""),
                    "control_class": str(payload.get("control_class") or ""),
                    "action_id": str(payload.get("action_id") or ""),
                    "ownership_generation": int(payload.get("ownership_generation") or 0),
                    "connection_generation": payload.get("connection_generation"),
                    "protocol_job_id": payload.get("protocol_job_id"),
                    "protocol_action_id": payload.get("protocol_action_id"),
                    "lifecycle_stage_id": payload.get("lifecycle_stage_id"),
                    "lifecycle_attempt_id": payload.get("lifecycle_attempt_id"),
                    "callback_session_id": payload.get("callback_session_id"),
                    "requested_inputs_json": requested_json,
                    "source_identity_json": source_json,
                }
                if any(existing[name] != value for name, value in expected.items()):
                    raise ValueError("idempotency key conflict")
                result = dict(outer)
                result["pipette_operation_id"] = str(existing["pipette_operation_id"])
                connection.execute("COMMIT")
                return result, False
            now = time.time()
            operation_id = str(payload.get("pipette_operation_id") or f"pipette.{command_id}")
            connection.execute(
                """
                INSERT INTO pipette_operations(
                    pipette_operation_id,command_id,operation,entrypoint_id,caller_class,
                    control_class,action_id,operation_class,status,ownership_generation,
                    connection_generation,protocol_job_id,protocol_action_id,lifecycle_stage_id,
                    lifecycle_attempt_id,callback_session_id,requested_inputs_json,source_identity_json,created_at,updated_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    operation_id, command_id, str(payload.get("operation") or "pipette"),
                    str(payload.get("entrypoint_id") or "service.pipette"),
                    str(payload.get("caller_class") or "operator"),
                    str(payload.get("control_class") or "physical_liquid_command"),
                    str(payload.get("action_id") or "pipette.operation"),
                    str(payload.get("operation_class") or "pipette"), str(outer["status"]),
                    int(payload.get("ownership_generation") or 0), payload.get("connection_generation"),
                    payload.get("protocol_job_id"), payload.get("protocol_action_id"),
                    payload.get("lifecycle_stage_id"), payload.get("lifecycle_attempt_id"),
                    payload.get("callback_session_id"),
                    requested_json, source_json, now, now,
                ),
            )
            result = dict(outer)
            result["pipette_operation_id"] = operation_id
            connection.execute("COMMIT")
            return result, True
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise

    @staticmethod
    def _truth_flag(result: Mapping[str, Any], name: str) -> int:
        value = result.get(name)
        return int(value) if isinstance(value, bool) else 0

    def _append_pipette_query_attestation(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        requested_semantic: int,
        observed_at: float,
        reconciliation_transition: bool,
    ) -> None:
        existing = self.connection.execute(
            "SELECT command_id,semantic_query_response_verified FROM operator_plane_pipette_query_attestations WHERE pipette_operation_id=? ORDER BY semantic_query_response_verified",
            (str(pipette_operation_id),),
        ).fetchall()
        if any(str(row["command_id"]) != str(command_id) for row in existing):
            raise RuntimeAuditStoreError("pipette query attestation identity conflicts")
        existing_semantics = {int(row["semantic_query_response_verified"]) for row in existing}
        if 1 in existing_semantics and requested_semantic == 0:
            raise RuntimeAuditStoreError("pipette query attestation truth cannot be downgraded")
        if 0 in existing_semantics and requested_semantic == 1 and not reconciliation_transition:
            raise RuntimeAuditStoreError("pipette query attestation truth conflicts")
        inserted = self.connection.execute(
            "INSERT OR IGNORE INTO operator_plane_pipette_query_attestations(pipette_operation_id,command_id,semantic_query_response_verified,observed_at) VALUES(?,?,?,?)",
            (str(pipette_operation_id), str(command_id), int(requested_semantic), float(observed_at)),
        )
        if inserted.rowcount != 1:
            exact = self.connection.execute(
                "SELECT 1 FROM operator_plane_pipette_query_attestations WHERE pipette_operation_id=? AND command_id=? AND semantic_query_response_verified=?",
                (str(pipette_operation_id), str(command_id), int(requested_semantic)),
            ).fetchone()
            if exact is None:
                raise RuntimeAuditStoreError("pipette query attestation append failed")

    @serialized_runtime_write
    def finalize_claim(
        self,
        *,
        command_id: str,
        pipette_operation_id: str | None,
        expected_status: str,
        status: str,
        outcome: str | None,
        failure_code: str | None,
        result: Mapping[str, Any],
        effective_inputs: Mapping[str, Any] | None = None,
        receipt_json: str | None = None,
        _expected_command_status: str | None = None,
        _expected_pipette_status: str | None = None,
        _reconciliation_transition: bool = False,
    ) -> None:
        if status not in TERMINAL_COMMAND_STATES and status not in {"dispatched", "acknowledged"}:
            raise ValueError(f"unsupported claim publication status: {status}")
        if not _reconciliation_transition and (
            _expected_command_status is not None or _expected_pipette_status is not None
        ):
            raise ValueError("expected protected state is reserved for reconcile_claim")
        now = time.time()
        bounded_result = canonical_json(dict(result))
        effective_json = canonical_json(dict(effective_inputs or {}))
        flags = {
            "delivery_verified": self._truth_flag(result, "delivery_verified"),
            "controller_acknowledged": self._truth_flag(result, "controller_acknowledged"),
            "completion_verified": self._truth_flag(result, "completion_verified"),
            "semantic_query_response_verified": self._truth_flag(result, "semantic_query_response_verified"),
            "hardware_precondition_verified": self._truth_flag(result, "hardware_precondition_verified"),
            "hardware_postcondition_verified": self._truth_flag(result, "hardware_postcondition_verified"),
            "physical_effect_verified": 0,
        }
        connection = self.connection
        owns_transaction = False
        try:
            owns_transaction = self._begin_write()
            command = connection.execute(
                "SELECT command_id,status,command_kind,receipt_json FROM operator_commands WHERE command_id=?",
                (str(command_id),),
            ).fetchone()
            if command is None:
                raise RuntimeAuditStoreError(f"unknown pipette claim: {command_id}")
            operation = None
            if pipette_operation_id is not None:
                operation = connection.execute(
                    "SELECT status FROM pipette_operations WHERE pipette_operation_id=? AND command_id=?",
                    (str(pipette_operation_id), str(command_id)),
                ).fetchone()
                if operation is None:
                    raise RuntimeAuditStoreError(
                        f"unknown pipette child claim: {pipette_operation_id}"
                    )
            command_status = str(command["status"])
            pipette_status = str(operation["status"]) if operation is not None else None
            target_status = str(status)
            if _reconciliation_transition:
                if target_status not in TERMINAL_STATES:
                    raise RuntimeAuditStoreError("reconciliation target must be terminal")
                if (
                    _expected_command_status not in TERMINAL_STATES
                    or command_status != _expected_command_status
                ):
                    raise RuntimeAuditStoreError("stale outer reconciliation expected state")
                if pipette_operation_id is None:
                    if _expected_pipette_status is not None:
                        raise RuntimeAuditStoreError("unexpected pipette reconciliation expected state")
                elif (
                    _expected_pipette_status not in TERMINAL_STATES
                    or pipette_status != _expected_pipette_status
                ):
                    raise RuntimeAuditStoreError("stale pipette reconciliation expected state")
            else:
                if command_status != str(expected_status):
                    raise RuntimeAuditStoreError(
                        f"claim expected-source-state mismatch: {command_id}:"
                        f"expected={expected_status}:actual={command_status}"
                    )
                if pipette_status is not None and pipette_status != str(expected_status):
                    raise RuntimeAuditStoreError(
                        "linked pipette projection expected-source-state mismatch: "
                        f"{pipette_operation_id}:expected={expected_status}:actual={pipette_status}"
                    )
                if command_status in TERMINAL_STATES:
                    raise RuntimeAuditStoreError(
                        f"ordinary transition cannot mutate terminal outer state {command_status}"
                    )
                if pipette_status in TERMINAL_STATES:
                    raise RuntimeAuditStoreError(
                        f"ordinary transition cannot mutate terminal pipette state {pipette_status}"
                    )
            if pipette_operation_id is not None:
                self._append_pipette_query_attestation(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    requested_semantic=flags["semantic_query_response_verified"],
                    observed_at=now,
                    reconciliation_transition=_reconciliation_transition,
                )
            finalize_outer = (
                _reconciliation_transition
                or pipette_operation_id is None
                or str(command["command_kind"] or "operator") != "operator"
            )
            if finalize_outer:
                outer_receipt_json = receipt_json if pipette_operation_id is None else None
                if pipette_operation_id is not None and _reconciliation_transition:
                    try:
                        outer_receipt = json.loads(str(command["receipt_json"] or "{}"))
                    except json.JSONDecodeError:
                        outer_receipt = {}
                    if not isinstance(outer_receipt, dict):
                        outer_receipt = {}
                    try:
                        reconciled_child = json.loads(str(receipt_json or "{}"))
                    except json.JSONDecodeError:
                        reconciled_child = {}
                    truth = {
                        "delivery_verified": bool(flags["delivery_verified"]),
                        "controller_acknowledged": bool(flags["controller_acknowledged"]),
                        "completion_verified": bool(flags["completion_verified"]),
                        "semantic_query_response_verified": bool(flags["semantic_query_response_verified"]),
                        "hardware_precondition_verified": bool(flags["hardware_precondition_verified"]),
                        "hardware_postcondition_verified": bool(flags["hardware_postcondition_verified"]),
                        "physical_effect_verified": False,
                    }
                    outer_receipt.update(
                        {
                            "status": str(status),
                            "outcome": outcome,
                            "failure_code": failure_code,
                            "reconciled_pipette_receipt": reconciled_child,
                            **truth,
                        }
                    )
                    for truth_key in ("truth", "receipt_truth"):
                        nested_truth = outer_receipt.get(truth_key)
                        if isinstance(nested_truth, Mapping):
                            outer_receipt[truth_key] = {**dict(nested_truth), **truth}
                    outer_receipt_json = canonical_json(outer_receipt)
                command_update = connection.execute(
                    """
                    UPDATE operator_commands
                    SET status=?, finished_at=?, response_summary_json=?, receipt_json=COALESCE(?, receipt_json), effective_inputs_json=?,
                        delivery_verified=?, controller_acknowledged=?, completion_verified=?,
                        hardware_precondition_verified=?, hardware_postcondition_verified=?,
                        physical_effect_verified=?, outcome=?, failure_code=?, updated_at=?
                    WHERE command_id=? AND status=?
                    """,
                    (
                        str(status),
                        str(now),
                        bounded_result,
                        outer_receipt_json,
                        effective_json,
                        flags["delivery_verified"],
                        flags["controller_acknowledged"],
                        flags["completion_verified"],
                        flags["hardware_precondition_verified"],
                        flags["hardware_postcondition_verified"],
                        flags["physical_effect_verified"],
                        outcome,
                        failure_code,
                        now,
                        str(command_id),
                        command_status,
                    ),
                )
                if command_update.rowcount != 1:
                    raise RuntimeAuditStoreError("outer claim state changed during finalization")
            if pipette_operation_id is not None:
                pipette_update = connection.execute(
                    """
                    UPDATE pipette_operations
                    SET status=?, effective_inputs_json=?, delivery_verified=?,
                        controller_acknowledged=?, completion_verified=?,
                        hardware_precondition_verified=?, hardware_postcondition_verified=?,
                        physical_effect_verified=?, outcome=?, failure_code=?,
                        receipt_json=COALESCE(?, receipt_json), finished_at=?, updated_at=?
                    WHERE pipette_operation_id=? AND command_id=? AND status=?
                    """,
                    (
                        str(status),
                        effective_json,
                        flags["delivery_verified"],
                        flags["controller_acknowledged"],
                        flags["completion_verified"],
                        flags["hardware_precondition_verified"],
                        flags["hardware_postcondition_verified"],
                        flags["physical_effect_verified"],
                        outcome,
                        failure_code,
                        receipt_json,
                        now,
                        now,
                        str(pipette_operation_id),
                        str(command_id),
                        pipette_status,
                    ),
                )
                if pipette_update.rowcount != 1:
                    raise RuntimeAuditStoreError("pipette claim state changed during finalization")
            connection.execute(
                "INSERT INTO operator_transitions(command_id,state,observed_at,detail_json) VALUES(?,?,?,?)",
                (
                    str(command_id),
                    str(status) if finalize_outer else command_status,
                    now,
                    canonical_json(
                        {
                            "outcome": outcome,
                            "failure_code": failure_code,
                            "pipette_child_status": str(status)
                            if pipette_operation_id is not None
                            else None,
                        }
                    ),
                ),
            )
            if result.get("completion_received") is True and result.get("controller_acknowledged") is not True:
                connection.execute(
                    """
                    INSERT INTO runtime_events(
                        command_id,pipette_operation_id,event_source,event_kind,event_json,
                        channel,transaction_id,semantic_validity,observed_at
                    ) VALUES(?,?,?,?,?,?,?,?,?)
                    """,
                    (
                        str(command_id),
                        str(pipette_operation_id) if pipette_operation_id is not None else None,
                        "pipette_service",
                        "completion_before_ack",
                        canonical_json({"result": dict(result)}),
                        result.get("channel"),
                        result.get("transaction_id"),
                        "tainted",
                        now,
                    ),
                )
            self._commit_write(owns_transaction)
        except Exception:
            self._rollback_write(owns_transaction)
            raise

    def persist_normalized_pipette_result(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        result: Mapping[str, Any],
        normalized: Mapping[str, Any],
    ) -> dict[str, list[str]]:
        """Append normalized pipette evidence inside an owned write transaction."""
        if not self.connection.in_transaction:
            raise RuntimeAuditStoreError(
                "normalized pipette evidence requires an active finalization transaction"
            )
        observation_ids: list[str] = []
        exchange_ids: list[str] = []
        event_ids: list[str] = []
        for row in normalized.get("channels") or ():
            observation_ids.append(
                self.record_channel_observation(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    **dict(row),
                )
            )
        for row in normalized.get("exchanges") or ():
            exchange_ids.append(
                self.record_transport_exchange(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    **dict(row),
                )
            )
        for row in normalized.get("events") or ():
            event_ids.append(
                self.record_event(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    **dict(row),
                )
            )
        pressure_samples = [dict(row) for row in normalized.get("pressure_samples") or ()]
        pressure_chunks = [dict(row) for row in normalized.get("pressure_chunks") or ()]
        pressure_stream = dict(normalized.get("pressure_stream") or {})
        pressure_stream_ids: list[str] = []
        pressure_chunk_ids: list[str] = []
        if pressure_samples or pressure_chunks or pressure_stream:
            if any(type(sample.get("channel")) is not int for sample in pressure_samples):
                raise RuntimeAuditStoreError("pressure sample channel attribution is required")
            if any(type(chunk.get("channel")) is not int for chunk in pressure_chunks):
                raise RuntimeAuditStoreError("pressure chunk channel attribution is required")
            observed_channels = sorted(
                {
                    *[int(sample["channel"]) for sample in pressure_samples],
                    *[int(chunk["channel"]) for chunk in pressure_chunks],
                }
            )
            selected_channels = pressure_stream.get("selected_channels")
            if selected_channels is not None:
                if not isinstance(selected_channels, list) or any(type(channel) is not int for channel in selected_channels):
                    raise RuntimeAuditStoreError("pressure stream selected channels are malformed")
                channels = sorted(set(int(channel) for channel in selected_channels))
            else:
                channels = observed_channels
            if not channels:
                channels = sorted(
                    {
                        int(row["channel"])
                        for row in normalized.get("channels") or ()
                        if row.get("truth_source") == "novo_router_pressure_epoch"
                    }
                )
            stream_id = self.record_pressure_stream(
                command_id=str(command_id),
                pipette_operation_id=str(pipette_operation_id),
                channels=channels,
                sample_period_ms=pressure_stream.get("sample_period_ms", result.get("sample_period_ms")),
                source_generation=pressure_stream.get(
                    "source_generation",
                    result.get("source_generation", result.get("reader_generation", 0)),
                ),
                reader_generation=pressure_stream.get("reader_generation", result.get("reader_generation")),
                offset_identity=pressure_stream.get("offset_identity", result.get("offset_identity")),
            )
            pressure_stream_ids.append(stream_id)
            materialized_chunks = pressure_chunks or [
                {
                    "channel": channel,
                    "chunk_sequence": 0,
                    "samples": [
                        {
                            **sample,
                            "raw_pressure": sample.get("raw_pressure", sample.get("value")),
                            "corrected_pressure": sample.get("corrected_pressure", sample.get("value")),
                            "controller_timestamp": sample.get(
                                "controller_timestamp", sample.get("controller_time")
                            ),
                        }
                        for sample in pressure_samples
                        if int(sample["channel"]) == channel
                    ],
                }
                for channel in channels
            ]
            for chunk in materialized_chunks:
                chunk_samples = [dict(sample) for sample in chunk.get("samples") or ()]
                if not chunk_samples:
                    continue
                sample_units = next(
                    (
                        sample.get("units")
                        for sample in chunk_samples
                        if isinstance(sample.get("units"), str) and sample.get("units")
                    ),
                    None,
                )
                units = chunk.get(
                    "units",
                    pressure_stream.get("units", result.get("pressure_units", sample_units)),
                )
                if not isinstance(units, str) or not units.strip():
                    raise RuntimeAuditStoreError("pressure chunk units are required")
                pressure_chunk_ids.append(
                    self.record_pressure_chunk(
                        stream_session_id=stream_id,
                        channel=int(chunk["channel"]),
                        chunk_sequence=int(chunk.get("chunk_sequence", 0)),
                        samples=chunk_samples,
                        units=units,
                        offset_identity=chunk.get(
                            "offset_identity",
                            pressure_stream.get("offset_identity", result.get("offset_identity")),
                        ),
                        chunk_schema=str(chunk.get("chunk_schema") or "bioxp.pipette.pressure.chunk.v1"),
                        lost_sample_count=int(chunk.get("lost_sample_count", 0)),
                        evidence_artifact_id=chunk.get("evidence_artifact_id"),
                    )
                )
            terminal_state = str(pressure_stream.get("terminal_state") or "outcome_unknown")
            self.connection.execute(
                "UPDATE pipette_pressure_streams SET stopped_at=?,terminal_state=?,loss_count=? WHERE stream_session_id=?",
                (
                    time.time() if terminal_state == "stopped" else None,
                    terminal_state,
                    sum(int(chunk.get("lost_sample_count", 0)) for chunk in materialized_chunks),
                    stream_id,
                ),
            )
        return {
            "observation_ids": observation_ids,
            "exchange_ids": exchange_ids,
            "event_ids": event_ids,
            "pressure_stream_ids": pressure_stream_ids,
            "pressure_chunk_ids": pressure_chunk_ids,
        }

    @serialized_runtime_write
    def reconcile_nonterminal_claims(self) -> int:
        """Converge every startup-active linked projection after a restart."""
        placeholders = ",".join("?" for _ in STARTUP_RECONCILIATION_STATES)
        active = tuple(sorted(STARTUP_RECONCILIATION_STATES))
        connection = self.connection

        def merged_receipt(raw: Any, *, command_id: str, operation_id: str | None, status: str,
                           outcome: Any, failure_code: Any, finished_at: float,
                           reconciliation: Mapping[str, Any], truth: Mapping[str, bool]) -> str:
            try:
                parsed = json.loads(str(raw)) if raw is not None else {}
            except (TypeError, ValueError, json.JSONDecodeError):
                parsed = {}
            receipt = dict(parsed) if isinstance(parsed, Mapping) else {}
            receipt.update(
                {
                    "command_id": command_id,
                    "status": status,
                    "outcome": outcome,
                    "failure_code": failure_code,
                    "finished_at": str(finished_at),
                    "reconciliation": dict(reconciliation),
                    **dict(truth),
                }
            )
            for truth_key in ("truth", "receipt_truth"):
                existing_truth = receipt.get(truth_key)
                if isinstance(existing_truth, Mapping):
                    receipt[truth_key] = {**dict(existing_truth), **dict(truth)}
            if operation_id is not None:
                receipt["pipette_operation_id"] = operation_id
            return canonical_json(receipt)

        try:
            connection.execute("BEGIN IMMEDIATE")
            rows = connection.execute(
                f"""
                SELECT c.command_id,c.status AS command_status,c.outcome AS command_outcome,
                       c.failure_code AS command_failure_code,c.receipt_json AS command_receipt_json,
                       c.delivery_verified AS command_delivery_verified,
                       c.controller_acknowledged AS command_controller_acknowledged,
                       c.completion_verified AS command_completion_verified,
                       c.hardware_precondition_verified AS command_hardware_precondition_verified,
                       c.hardware_postcondition_verified AS command_hardware_postcondition_verified,
                       c.physical_effect_verified AS command_physical_effect_verified,
                       p.pipette_operation_id,p.status AS pipette_status,
                       p.outcome AS pipette_outcome,p.failure_code AS pipette_failure_code,
                       p.receipt_json AS pipette_receipt_json,
                       p.delivery_verified AS pipette_delivery_verified,
                       p.controller_acknowledged AS pipette_controller_acknowledged,
                       p.completion_verified AS pipette_completion_verified,
                       p.hardware_precondition_verified AS pipette_hardware_precondition_verified,
                       p.hardware_postcondition_verified AS pipette_hardware_postcondition_verified,
                       p.physical_effect_verified AS pipette_physical_effect_verified,
                       COALESCE((
                           SELECT MAX(q.semantic_query_response_verified)
                           FROM operator_plane_pipette_query_attestations AS q
                           WHERE q.command_id=c.command_id
                             AND q.pipette_operation_id=p.pipette_operation_id
                       ),0) AS semantic_query_response_verified
                FROM operator_commands AS c
                LEFT JOIN pipette_operations AS p ON p.command_id=c.command_id
                WHERE c.status IN ({placeholders}) OR p.status IN ({placeholders})
                ORDER BY c.sequence
                """,
                active + active,
            ).fetchall()
            now = time.time()
            changed = 0
            for row in rows:
                command_id = str(row["command_id"])
                operation_id = (
                    None
                    if row["pipette_operation_id"] is None
                    else str(row["pipette_operation_id"])
                )
                command_status = str(row["command_status"])
                pipette_status = None if row["pipette_status"] is None else str(row["pipette_status"])
                command_is_terminal = (
                    command_status in TERMINAL_COMMAND_STATES
                    and command_status != "reconciliation_required"
                )
                pipette_is_terminal = (
                    pipette_status in TERMINAL_COMMAND_STATES
                    and pipette_status != "reconciliation_required"
                )
                if command_is_terminal:
                    terminal_status = command_status
                    terminal_outcome = row["command_outcome"]
                    terminal_failure_code = row["command_failure_code"]
                    update_command = False
                    update_pipette = operation_id is not None and not pipette_is_terminal
                    authority = "operator_command"
                elif pipette_is_terminal:
                    terminal_status = str(pipette_status)
                    terminal_outcome = row["pipette_outcome"]
                    terminal_failure_code = row["pipette_failure_code"]
                    update_command = True
                    update_pipette = False
                    authority = "pipette_operation"
                else:
                    terminal_status = "outcome_unknown"
                    terminal_outcome = "outcome_unknown"
                    terminal_failure_code = "startup_reconciliation"
                    update_command = True
                    update_pipette = operation_id is not None
                    authority = "restart_reconciliation"
                truth_source = (
                    "command"
                    if command_is_terminal
                    else "pipette"
                    if pipette_is_terminal
                    else None
                )
                truth = {
                    field: bool(row[f"{truth_source}_{field}"])
                    if truth_source is not None
                    else False
                    for field in (
                        "delivery_verified",
                        "controller_acknowledged",
                        "completion_verified",
                        "hardware_precondition_verified",
                        "hardware_postcondition_verified",
                        "physical_effect_verified",
                    )
                }
                truth["semantic_query_response_verified"] = bool(
                    row["semantic_query_response_verified"]
                )
                detail_value = {
                    "reason": "process_restart_with_nonterminal_projection",
                    "authority": authority,
                    "automatic_retry": False,
                    "prior_command_status": command_status,
                    "prior_pipette_status": pipette_status,
                }
                detail = canonical_json(detail_value)
                response_summary = canonical_json(
                    {"reconciliation": detail_value, **truth}
                )
                if operation_id is not None:
                    self._append_pipette_query_attestation(
                        command_id=command_id,
                        pipette_operation_id=operation_id,
                        requested_semantic=int(truth["semantic_query_response_verified"]),
                        observed_at=now,
                        reconciliation_transition=True,
                    )
                if update_command:
                    command_updated = connection.execute(
                        """
                        UPDATE operator_commands
                        SET status=?,finished_at=?,outcome=?,failure_code=?,
                            response_summary_json=?,receipt_json=?,
                            delivery_verified=?,controller_acknowledged=?,completion_verified=?,
                            hardware_precondition_verified=?,hardware_postcondition_verified=?,
                            physical_effect_verified=?,updated_at=?
                        WHERE command_id=? AND status=?
                        """,
                        (
                            terminal_status,
                            str(now),
                            terminal_outcome,
                            terminal_failure_code,
                            response_summary,
                            merged_receipt(
                                row["command_receipt_json"],
                                command_id=command_id,
                                operation_id=None,
                                status=terminal_status,
                                outcome=terminal_outcome,
                                failure_code=terminal_failure_code,
                                finished_at=now,
                                reconciliation=detail_value,
                                truth=truth,
                            ),
                            int(truth["delivery_verified"]),
                            int(truth["controller_acknowledged"]),
                            int(truth["completion_verified"]),
                            int(truth["hardware_precondition_verified"]),
                            int(truth["hardware_postcondition_verified"]),
                            int(truth["physical_effect_verified"]),
                            now,
                            command_id,
                            command_status,
                        ),
                    ).rowcount
                    if command_updated != 1:
                        raise RuntimeAuditStoreError("operator startup reconciliation CAS lost")
                if update_pipette:
                    if pipette_status is None or operation_id is None:
                        raise RuntimeAuditStoreError("linked pipette startup status is missing")
                    child_updated = connection.execute(
                        """
                        UPDATE pipette_operations
                        SET status=?,finished_at=?,outcome=?,failure_code=?,
                            receipt_json=?,delivery_verified=?,controller_acknowledged=?,
                            completion_verified=?,hardware_precondition_verified=?,
                            hardware_postcondition_verified=?,physical_effect_verified=?,updated_at=?
                        WHERE pipette_operation_id=? AND command_id=? AND status=?
                        """,
                        (
                            terminal_status,
                            now,
                            terminal_outcome,
                            terminal_failure_code,
                            merged_receipt(
                                row["pipette_receipt_json"],
                                command_id=command_id,
                                operation_id=operation_id,
                                status=terminal_status,
                                outcome=terminal_outcome,
                                failure_code=terminal_failure_code,
                                finished_at=now,
                                reconciliation=detail_value,
                                truth=truth,
                            ),
                            int(truth["delivery_verified"]),
                            int(truth["controller_acknowledged"]),
                            int(truth["completion_verified"]),
                            int(truth["hardware_precondition_verified"]),
                            int(truth["hardware_postcondition_verified"]),
                            int(truth["physical_effect_verified"]),
                            now,
                            operation_id,
                            command_id,
                            pipette_status,
                        ),
                    ).rowcount
                    if child_updated != 1:
                        raise RuntimeAuditStoreError("pipette startup reconciliation CAS lost")
                connection.execute(
                    "INSERT INTO operator_transitions(command_id,state,observed_at,detail_json) VALUES(?,?,?,?)",
                    (command_id, terminal_status, now, detail),
                )
                changed += 1
            connection.execute("COMMIT")
            return changed
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise

    @serialized_runtime_write
    def reconcile_claim(
        self,
        *,
        command_id: str,
        pipette_operation_id: str | None,
        expected_command_status: str,
        expected_pipette_status: str | None,
        status: str,
        outcome: str | None,
        failure_code: str | None,
        result: Mapping[str, Any],
        effective_inputs: Mapping[str, Any] | None = None,
        receipt_json: str | None = None,
    ) -> None:
        """Explicitly reconcile a protected claim using exact expected states."""
        self.finalize_claim(
            command_id=command_id,
            pipette_operation_id=pipette_operation_id,
            expected_status=expected_command_status,
            status=status,
            outcome=outcome,
            failure_code=failure_code,
            result=result,
            effective_inputs=effective_inputs,
            receipt_json=receipt_json,
            _expected_command_status=expected_command_status,
            _expected_pipette_status=expected_pipette_status,
            _reconciliation_transition=True,
        )

    @serialized_runtime_write
    def record_channel_observation(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        channel: int,
        phase: str,
        semantic_validity: str,
        truth_source: str,
        tip_loaded: bool | None,
        pressure: float | None,
        pressure_units: str | None,
        status: str | None,
        error_code: int | None,
        firmware_class: str | None = None,
        detail: Mapping[str, Any] | None = None,
    ) -> str:
        observation_id = uuid.uuid4().hex
        owns_transaction = False
        try:
            owns_transaction = self._begin_write()
            self.connection.execute(
                """
                INSERT INTO pipette_channel_observations(
                    observation_id,command_id,pipette_operation_id,channel,phase,observed_at,
                    semantic_validity,truth_source,tip_loaded,pressure,pressure_units,status,
                    error_code,firmware_class,detail_json
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    observation_id,
                    str(command_id),
                    str(pipette_operation_id),
                    int(channel),
                    str(phase),
                    time.time(),
                    str(semantic_validity),
                    str(truth_source),
                    None if tip_loaded is None else int(bool(tip_loaded)),
                    pressure,
                    pressure_units,
                    status,
                    error_code,
                    firmware_class,
                    canonical_json(dict(detail or {})),
                ),
            )
            self._commit_write(owns_transaction)
            return observation_id
        except Exception:
            self._rollback_write(owns_transaction)
            raise

    @serialized_runtime_write
    def record_transport_exchange(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        channel: int | None,
        transaction_phase: str,
        command_family: int | None,
        matcher_name: str | None,
        tx_id: int | None,
        tx_dlc: int | None,
        tx_bytes: list[int] | tuple[int, ...],
        expected_rx_id: int | None,
        observed_rx_id: int | None,
        rx_dlc: int | None,
        rx_bytes: list[int] | tuple[int, ...],
        router_generation: int | None = None,
        sent_at: float | None = None,
        received_at: float | None = None,
        ack_at: float | None = None,
        completion_at: float | None = None,
        delivery_verified: bool = False,
        semantic_match: bool = False,
        controller_acknowledged: bool = False,
        completion_verified: bool = False,
        completion_before_ack: bool = False,
        multipart: Mapping[str, Any] | None = None,
        raw_exchange: Mapping[str, Any] | None = None,
        transaction_id: str | None = None,
    ) -> str:
        exchange_id = uuid.uuid4().hex
        derived_expected = None if tx_id is None else int(tx_id) | 0x400
        if derived_expected is not None and expected_rx_id != derived_expected:
            raise ValueError("expected_rx_id must equal tx_id | 0x400")
        owns_transaction = False
        try:
            owns_transaction = self._begin_write()
            self.connection.execute(
                """
                INSERT INTO pipette_transport_exchanges(
                    exchange_id,command_id,pipette_operation_id,transaction_id,channel,transaction_phase,
                    command_family,matcher_name,tx_id,tx_dlc,tx_bytes_json,expected_rx_id,
                    observed_rx_id,rx_dlc,rx_bytes_json,router_generation,sent_at,received_at,
                    ack_at,completion_at,delivery_verified,semantic_match,controller_acknowledged,
                    completion_verified,completion_before_ack,multipart_json,raw_exchange_json
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    exchange_id,
                    str(command_id),
                    str(pipette_operation_id),
                    transaction_id,
                    channel,
                    str(transaction_phase),
                    command_family,
                    matcher_name,
                    tx_id,
                    tx_dlc,
                    canonical_json(list(tx_bytes)[:64]),
                    expected_rx_id,
                    observed_rx_id,
                    rx_dlc,
                    canonical_json(list(rx_bytes)[:64]),
                    router_generation,
                    sent_at,
                    received_at,
                    ack_at,
                    completion_at,
                    int(bool(delivery_verified)),
                    int(bool(semantic_match)),
                    int(bool(controller_acknowledged)),
                    int(bool(completion_verified)),
                    int(bool(completion_before_ack)),
                    canonical_json(dict(multipart or {})),
                    canonical_json(dict(raw_exchange or {})),
                ),
            )
            self._commit_write(owns_transaction)
            return exchange_id
        except Exception:
            self._rollback_write(owns_transaction)
            raise

    @serialized_runtime_write
    def record_event(
        self,
        *,
        command_id: str | None,
        pipette_operation_id: str | None,
        event_source: str,
        event_kind: str,
        event_payload: Mapping[str, Any],
        channel: int | None = None,
        stream_session_id: str | None = None,
        transaction_id: str | None = None,
        reader_generation: int | None = None,
        ownership_generation: int | None = None,
        source_sequence: int | None = None,
        semantic_validity: str | None = None,
    ) -> str:
        owns_transaction = False
        try:
            owns_transaction = self._begin_write()
            cursor = self.connection.execute(
                """
                INSERT INTO runtime_events(
                    command_id,pipette_operation_id,event_source,event_kind,event_json,channel,
                    stream_session_id,transaction_id,reader_generation,ownership_generation,
                    source_sequence,semantic_validity,observed_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    command_id,
                    pipette_operation_id,
                    str(event_source),
                    str(event_kind),
                    canonical_json(dict(event_payload)),
                    channel,
                    stream_session_id,
                    transaction_id,
                    reader_generation,
                    ownership_generation,
                    source_sequence,
                    semantic_validity,
                    time.time(),
                ),
            )
            event_id = str(cursor.lastrowid)
            self._commit_write(owns_transaction)
            return event_id
        except Exception:
            self._rollback_write(owns_transaction)
            raise

    @serialized_runtime_write
    def record_pressure_stream(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        channels: list[int] | tuple[int, ...],
        sample_period_ms: float | None,
        source_generation: int | None,
        reader_generation: int | None = None,
        offset_identity: str | None = None,
    ) -> str:
        stream_id = uuid.uuid4().hex
        selected = sorted({int(channel) for channel in channels})
        if not selected or any(channel < 0 or channel > 3 for channel in selected):
            raise ValueError("pressure stream channels must be within 0..3")
        owns_transaction = False
        try:
            owns_transaction = self._begin_write()
            self.connection.execute(
                """
                INSERT INTO pipette_pressure_streams(
                    stream_session_id,command_id,pipette_operation_id,channels_json,sample_period_ms,
                    started_at,source_generation,reader_generation,offset_identity,terminal_state,loss_count
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    stream_id,
                    command_id,
                    pipette_operation_id,
                    canonical_json(selected),
                    sample_period_ms,
                    time.time(),
                    source_generation,
                    reader_generation,
                    offset_identity,
                    "running",
                    0,
                ),
            )
            self._commit_write(owns_transaction)
            return stream_id
        except Exception:
            self._rollback_write(owns_transaction)
            raise

    @serialized_runtime_write
    def record_pressure_chunk(
        self,
        *,
        stream_session_id: str,
        channel: int,
        chunk_sequence: int,
        samples: list[Mapping[str, Any]] | tuple[Mapping[str, Any], ...],
        units: str,
        offset_identity: str | None,
        chunk_schema: str,
        lost_sample_count: int = 0,
        evidence_artifact_id: str | None = None,
    ) -> str:
        normalized = [dict(sample) for sample in samples]
        raw_bytes = canonical_json(normalized).encode("utf-8")
        raw_values = [float(sample["raw_pressure"]) for sample in normalized if sample.get("raw_pressure") is not None]
        corrected_values = [float(sample["corrected_pressure"]) for sample in normalized if sample.get("corrected_pressure") is not None]
        sequences = [int(sample["sample_sequence"]) for sample in normalized if sample.get("sample_sequence") is not None]
        times = [float(sample[key]) for sample in normalized for key in ("controller_timestamp", "robot_receive_time") if sample.get(key) is not None]
        chunk_id = uuid.uuid4().hex

        def mean(values: list[float]) -> float | None:
            return sum(values) / len(values) if values else None

        owns_transaction = False
        try:
            owns_transaction = self._begin_write()
            self.connection.execute(
                """
                INSERT INTO pipette_pressure_chunks(
                    chunk_id,stream_session_id,channel,chunk_sequence,first_sample_sequence,
                    last_sample_sequence,first_time,last_time,sample_count,lost_sample_count,units,
                    raw_min,raw_max,raw_mean,corrected_min,corrected_max,corrected_mean,
                    offset_identity,chunk_schema,byte_count,sha256,evidence_artifact_id,sample_summary_json
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    chunk_id,
                    stream_session_id,
                    int(channel),
                    int(chunk_sequence),
                    min(sequences) if sequences else None,
                    max(sequences) if sequences else None,
                    min(times) if times else None,
                    max(times) if times else None,
                    len(normalized),
                    int(lost_sample_count),
                    str(units),
                    min(raw_values) if raw_values else None,
                    max(raw_values) if raw_values else None,
                    mean(raw_values),
                    min(corrected_values) if corrected_values else None,
                    max(corrected_values) if corrected_values else None,
                    mean(corrected_values),
                    offset_identity,
                    str(chunk_schema),
                    len(raw_bytes),
                    hashlib.sha256(raw_bytes).hexdigest(),
                    evidence_artifact_id,
                    canonical_json({"first_sequence": min(sequences) if sequences else None, "last_sequence": max(sequences) if sequences else None}),
                ),
            )
            if evidence_artifact_id is not None:
                if self.connection.execute(
                    "SELECT 1 FROM runtime_evidence_objects WHERE evidence_artifact_id=?",
                    (str(evidence_artifact_id),),
                ).fetchone() is None:
                    raise RuntimeAuditStoreError("pressure chunk evidence authority is unavailable")
                self.connection.execute(
                    "INSERT INTO runtime_evidence_links("
                    "evidence_artifact_id,target_kind,target_identity,command_id,"
                    "pipette_operation_id,link_kind,created_at"
                    ") VALUES(?,?,?,?,?,?,?)",
                    (
                        str(evidence_artifact_id),
                        "pressure_chunk",
                        chunk_id,
                        None,
                        None,
                        "pressure_chunk_evidence",
                        time.time(),
                    ),
                )
            self._commit_write(owns_transaction)
            return chunk_id
        except Exception:
            self._rollback_write(owns_transaction)
            raise

    def close(self) -> None:
        self.connection.close()



def _process_start_observation(pid: int) -> tuple[int, float, str]:
    cgroup = Path(f"/proc/{pid}/cgroup").read_text(encoding="utf-8").strip()
    stat_text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").strip()
    close_paren = stat_text.rfind(")")
    if close_paren < 0:
        raise RuntimeAuditStoreError("application process stat is malformed")
    tail = stat_text[close_paren + 2 :].split()
    start_ticks = int(tail[19])
    boot_time = None
    for line in Path("/proc/stat").read_text(encoding="utf-8").splitlines():
        if line.startswith("btime "):
            boot_time = float(line.split()[1])
            break
    if boot_time is None or not cgroup:
        raise RuntimeAuditStoreError("application process start/cgroup observation is unavailable")
    started_at = boot_time + (start_ticks / float(os.sysconf("SC_CLK_TCK")))
    return start_ticks, started_at, cgroup


def _measured_listener(pid: int, host: str, port: int) -> dict[str, Any] | None:
    socket_inodes: set[str] = set()
    for fd in Path(f"/proc/{pid}/fd").iterdir():
        try:
            target = os.readlink(fd)
        except OSError:
            continue
        if target.startswith("socket:[") and target.endswith("]"):
            socket_inodes.add(target[8:-1])
    wanted_port = f"{int(port):04X}"
    for table_path in (Path("/proc/net/tcp"), Path("/proc/net/tcp6")):
        try:
            rows = table_path.read_text(encoding="utf-8").splitlines()[1:]
        except OSError:
            continue
        for row in rows:
            fields = row.split()
            if len(fields) < 10 or fields[3] != "0A":
                continue
            address, _separator, local_port = fields[1].rpartition(":")
            inode = fields[9]
            if local_port == wanted_port and inode in socket_inodes:
                try:
                    if table_path.name == "tcp":
                        observed_host = socket.inet_ntop(
                            socket.AF_INET,
                            bytes.fromhex(address)[::-1],
                        )
                    else:
                        raw = bytes.fromhex(address)
                        observed_host = socket.inet_ntop(
                            socket.AF_INET6,
                            b"".join(raw[index : index + 4][::-1] for index in range(0, 16, 4)),
                        )
                except (OSError, ValueError):
                    continue
                if observed_host != host:
                    continue
                cgroup = Path(f"/proc/{pid}/cgroup").read_text(encoding="utf-8").strip()
                return {
                    "host": observed_host,
                    "port": int(port),
                    "socket_inode": int(inode),
                    "owner_pid": pid,
                    "owner_cgroup_sha256": hashlib.sha256((cgroup + "\n").encode("utf-8")).hexdigest(),
                }
    return None


def record_runtime_release_start(
    root: str | Path,
    identity: Mapping[str, Any],
    *,
    timeout_s: float = 45.0,
) -> dict[str, Any]:
    """Measure and durably append the canonical v4 release-start receipt."""
    if identity.get("verified") is not True:
        raise RuntimeAuditStoreError("verified release identity is required for a runtime start receipt")
    source_value = identity.get("source")
    image_value = identity.get("image")
    deployment_value = identity.get("deployment")
    binding_value = identity.get("binding")
    if not all(isinstance(value, Mapping) for value in (source_value, image_value, deployment_value, binding_value)):
        raise RuntimeAuditStoreError("release identity sections are incomplete")
    source = dict(source_value)
    image = dict(image_value)
    deployment = dict(deployment_value)
    binding = dict(cast(Mapping[str, Any], binding_value))
    declared_value = binding.get("declared_listener")
    if not isinstance(declared_value, Mapping):
        raise RuntimeAuditStoreError("declared listener is unavailable")
    declared = dict(declared_value)
    declared_port = declared.get("port")
    if not isinstance(declared_port, int):
        raise RuntimeAuditStoreError("declared listener port is unavailable")
    pid = os.getpid()
    start_ticks, started_at, cgroup = _process_start_observation(pid)
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    observed = None
    while observed is None:
        observed = _measured_listener(pid, str(declared.get("host")), declared_port)
        if observed is not None or time.monotonic() >= deadline:
            break
        time.sleep(0.05)
    if observed is None:
        raise RuntimeAuditStoreError("canonical listener ownership was not measured for the application PID")
    receipt = {
        "schema": "bioxp.runtime.release_start.v1",
        "receipt_id": f"runtime-release-{uuid.uuid4().hex}",
        "release_id": str(identity["release_id"]),
        "deployment_receipt_id": str(deployment["receipt_id"]),
        "systemd_invocation_id": str(binding["systemd_invocation_id"]),
        "application_pid": pid,
        "application_cgroup": cgroup,
        "application_cgroup_sha256": hashlib.sha256((cgroup + "\n").encode("utf-8")).hexdigest(),
        "application_start_time_ticks": start_ticks,
        "application_started_at": started_at,
        "canonical_receipt_sha256": str(deployment["receipt_sha256"]),
        "source_manifest_sha256": str(source["manifest_sha256"]),
        "source_aggregate_sha256": str(source["aggregate_sha256"]),
        "image_id": str(image["id"]),
        "image_inspection_receipt_sha256": str(image["inspection_receipt_sha256"]),
        "udocker_path": str(binding["udocker_path"]),
        "udocker_sha256": str(binding["udocker_sha256"]),
        "udocker_tree_sha256": str(binding["udocker_tree_sha256"]),
        "unit_sha256": str(binding["unit_sha256"]),
        "launcher_sha256": str(binding["launcher_sha256"]),
        "configuration_sha256": str(binding["configuration_sha256"]),
        "oem_lock_sha256": str(binding["oem_lock_sha256"]),
        "declared_listener": declared,
        "observed_listener": observed,
        "recorded_at": time.time(),
    }
    receipt_sha256 = hashlib.sha256(canonical_json(receipt).encode("utf-8")).hexdigest()
    receipt["receipt_sha256"] = receipt_sha256
    database = RuntimeAuditDatabase(root=root, initialize_schema=False)
    try:
        with database.writer_lock:
            database.connection.execute("BEGIN IMMEDIATE")
            database.connection.execute(
                """
                INSERT INTO runtime_release_receipts(
                    receipt_id,release_id,deployment_receipt_id,systemd_invocation_id,
                    application_pid,application_cgroup,application_cgroup_sha256,
                    application_start_time_ticks,application_started_at,canonical_receipt_sha256,
                    source_manifest_sha256,source_aggregate_sha256,image_id,
                    image_inspection_receipt_sha256,udocker_path,udocker_sha256,
                    udocker_tree_sha256,unit_sha256,launcher_sha256,
                    configuration_sha256,oem_lock_sha256,declared_listener_json,
                    observed_listener_json,receipt_json,receipt_sha256,recorded_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    receipt["receipt_id"], receipt["release_id"], receipt["deployment_receipt_id"], receipt["systemd_invocation_id"],
                    pid, cgroup, receipt["application_cgroup_sha256"], start_ticks, started_at,
                    receipt["canonical_receipt_sha256"], receipt["source_manifest_sha256"], receipt["source_aggregate_sha256"],
                    receipt["image_id"], receipt["image_inspection_receipt_sha256"], receipt["udocker_path"],
                    receipt["udocker_sha256"], receipt["udocker_tree_sha256"], receipt["unit_sha256"],
                    receipt["launcher_sha256"], receipt["configuration_sha256"], receipt["oem_lock_sha256"],
                    canonical_json(receipt["declared_listener"]), canonical_json(observed), canonical_json(receipt),
                    receipt_sha256, receipt["recorded_at"],
                ),
            )
            stored = database.connection.execute(
                "SELECT receipt_json,receipt_sha256 FROM runtime_release_receipts WHERE receipt_id=?",
                (receipt["receipt_id"],),
            ).fetchone()
            if stored is None or str(stored["receipt_sha256"]) != receipt_sha256 or json.loads(stored["receipt_json"]) != receipt:
                raise RuntimeAuditStoreError("runtime start receipt durable readback failed")
            database.connection.execute("COMMIT")
    except Exception:
        if database.connection.in_transaction:
            database.connection.execute("ROLLBACK")
        raise
    finally:
        database.close()
    return receipt

def connect_runtime_database(root: str | Path | None = None) -> RuntimeAuditDatabase:
    return RuntimeAuditDatabase(root)
