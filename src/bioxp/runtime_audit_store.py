from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import time
import uuid
from pathlib import Path
from typing import Any, Mapping


CANONICAL_RUNTIME_ROOT = Path("/var/lib/bioxp-oem-runtime")
RUNTIME_ROOT_ENV_NAMES = (
    "BIOXP_OEM_RUNTIME_STATE_ROOT",
    "BIOXP_OEM_RUNTIME_ROOT",
)
SCHEMA_VERSION = 2


class RuntimeAuditStoreError(RuntimeError):
    """The robot audit authority cannot guarantee a durable operation claim."""


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
    applied_at REAL NOT NULL
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
    evidence_artifact_id TEXT NOT NULL,
    command_id TEXT REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    link_kind TEXT NOT NULL,
    created_at REAL NOT NULL
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
    quarantined_count INTEGER NOT NULL,
    status TEXT NOT NULL,
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
"""

_APPEND_ONLY_TABLES = (
    "runtime_schema_migrations",
    "operator_transitions",
    "runtime_events",
    "runtime_evidence_links",
    "runtime_evidence_events",
    "runtime_migration_receipts",
    "report_exports",
    "pipette_channel_observations",
    "pipette_transport_exchanges",
    "pipette_pressure_chunks",
)


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
    }
    for table, additions in additions_by_table.items():
        columns = {str(row["name"]) for row in connection.execute(f"PRAGMA table_info({table})")}
        for name, definition in additions.items():
            if name not in columns:
                connection.execute(f"ALTER TABLE {table} ADD COLUMN {name} {definition}")


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


def ensure_schema(connection: sqlite3.Connection, root: Path | None = None) -> None:
    configure_connection(connection)
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
        ddl_digest = hashlib.sha256(_SCHEMA_DDL.encode("utf-8")).hexdigest()
        connection.execute(
            """
            INSERT OR IGNORE INTO runtime_schema_migrations(version, name, ddl_sha256, applied_at)
            VALUES(?,?,?,?)
            """,
            (SCHEMA_VERSION, "runtime_audit_foundation", ddl_digest, now),
        )
        if root is not None:
            db_path = str((root / "bioxp_runtime.db").resolve())
            connection.execute(
                """
                INSERT INTO runtime_store_identity(identity_id, database_path, schema_version, created_at, updated_at)
                VALUES(1,?,?,?,?)
                ON CONFLICT(identity_id) DO UPDATE SET
                    database_path=excluded.database_path,
                    schema_version=excluded.schema_version,
                    updated_at=excluded.updated_at
                """,
                (db_path, SCHEMA_VERSION, now, now),
            )
        connection.execute(f"PRAGMA user_version={SCHEMA_VERSION}")
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise


class RuntimeAuditDatabase:
    def __init__(self, root: str | Path | None = None) -> None:
        self.root = resolve_runtime_state_root(root)
        self.path = self.root / "bioxp_runtime.db"
        try:
            self.connection = sqlite3.connect(
                self.path,
                timeout=2.0,
                isolation_level=None,
                check_same_thread=False,
            )
            ensure_schema(self.connection, self.root)
        except (OSError, sqlite3.Error) as exc:
            raise RuntimeAuditStoreError(
                f"canonical BioXP audit database is unavailable: {self.path}"
            ) from exc

    def claim(self, payload: Mapping[str, Any], *, pipette: bool = False) -> tuple[dict[str, Any], bool]:
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

        def claim_projection(row: sqlite3.Row | Mapping[str, Any]) -> dict[str, Any]:
            result = dict(row)
            if pipette:
                operation_row = connection.execute(
                    "SELECT pipette_operation_id FROM pipette_operations WHERE command_id=?",
                    (str(result["command_id"]),),
                ).fetchone()
                result["pipette_operation_id"] = (
                    str(operation_row["pipette_operation_id"])
                    if operation_row is not None
                    else str(result["command_id"])
                )
            return result

        try:
            connection.execute("BEGIN IMMEDIATE")
            existing = connection.execute(
                "SELECT * FROM operator_commands WHERE idempotency_key=? AND idempotency_replay_enabled=1",
                (idempotency_key,),
            ).fetchone()
            if existing is not None:
                existing_digest = str(existing["canonical_request_sha256"] or "")
                if existing_digest and existing_digest != digest:
                    raise ValueError("idempotency key conflict")
                connection.execute("COMMIT")
                return claim_projection(existing), False

            existing_command = connection.execute(
                "SELECT * FROM operator_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if existing_command is not None:
                if str(existing_command["canonical_request_sha256"] or "") != digest:
                    raise ValueError("command identity conflict")
                connection.execute("COMMIT")
                return claim_projection(existing_command), False

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
                    "requested_inputs": requested_inputs,
                    "status": "reserved",
                }
            )
            connection.execute(
                """
                INSERT INTO operator_commands(
                    command_id,idempotency_key,canonical_request_sha256,operation,command_kind,entrypoint_id,
                    caller_class,control_class,idempotency_replay_enabled,action_id,status,
                    safety_class,ownership_generation,source_identity_json,requested_inputs_json,
                    effective_inputs_json,started_at,receipt_json,updated_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
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
                        requested_inputs_json,source_identity_json,created_at,updated_at
                    ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)
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
            connection.execute("COMMIT")
            if row is None:
                raise RuntimeAuditStoreError("durable claim disappeared before commit")
            return claim_projection(row), True
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise

    @staticmethod
    def _truth_flag(result: Mapping[str, Any], name: str) -> int:
        value = result.get(name)
        return int(value) if isinstance(value, bool) else 0

    def finalize_claim(
        self,
        *,
        command_id: str,
        pipette_operation_id: str | None,
        status: str,
        outcome: str | None,
        failure_code: str | None,
        result: Mapping[str, Any],
        effective_inputs: Mapping[str, Any] | None = None,
        receipt_json: str | None = None,
    ) -> None:
        now = time.time()
        bounded_result = canonical_json(dict(result))
        effective_json = canonical_json(dict(effective_inputs or {}))
        flags = {
            "delivery_verified": self._truth_flag(result, "delivery_verified"),
            "controller_acknowledged": self._truth_flag(result, "controller_acknowledged"),
            "completion_verified": self._truth_flag(result, "completion_verified"),
            "hardware_precondition_verified": self._truth_flag(result, "hardware_precondition_verified"),
            "hardware_postcondition_verified": self._truth_flag(result, "hardware_postcondition_verified"),
            "physical_effect_verified": 0,
        }
        connection = self.connection
        try:
            connection.execute("BEGIN IMMEDIATE")
            command = connection.execute(
                "SELECT command_id FROM operator_commands WHERE command_id=?",
                (str(command_id),),
            ).fetchone()
            if command is None:
                raise RuntimeAuditStoreError(f"unknown pipette claim: {command_id}")
            connection.execute(
                """
                UPDATE operator_commands
                SET status=?, finished_at=?, response_summary_json=?, receipt_json=COALESCE(?, receipt_json), effective_inputs_json=?,
                    delivery_verified=?, controller_acknowledged=?, completion_verified=?,
                    hardware_precondition_verified=?, hardware_postcondition_verified=?,
                    physical_effect_verified=?, outcome=?, failure_code=?, updated_at=?
                WHERE command_id=?
                """,
                (
                    str(status),
                    str(now),
                    bounded_result,
                    receipt_json,
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
                ),
            )
            if pipette_operation_id is not None:
                connection.execute(
                    """
                    UPDATE pipette_operations
                    SET status=?, effective_inputs_json=?, delivery_verified=?,
                        controller_acknowledged=?, completion_verified=?,
                        hardware_precondition_verified=?, hardware_postcondition_verified=?,
                        physical_effect_verified=?, outcome=?, failure_code=?,
                        receipt_json=COALESCE(?, receipt_json), finished_at=?, updated_at=?
                    WHERE pipette_operation_id=? AND command_id=?
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
                    ),
                )
            connection.execute(
                "INSERT INTO operator_transitions(command_id,state,observed_at,detail_json) VALUES(?,?,?,?)",
                (
                    str(command_id),
                    str(status),
                    now,
                    canonical_json({"outcome": outcome, "failure_code": failure_code}),
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
            connection.execute("COMMIT")
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise

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
        try:
            self.connection.execute("BEGIN IMMEDIATE")
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
            self.connection.execute("COMMIT")
            return observation_id
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

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
        try:
            self.connection.execute("BEGIN IMMEDIATE")
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
            self.connection.execute("COMMIT")
            return exchange_id
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

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
        try:
            self.connection.execute("BEGIN IMMEDIATE")
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
            self.connection.execute("COMMIT")
            return event_id
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

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
        try:
            self.connection.execute("BEGIN IMMEDIATE")
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
            self.connection.execute("COMMIT")
            return stream_id
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

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

        try:
            self.connection.execute("BEGIN IMMEDIATE")
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
            self.connection.execute("COMMIT")
            return chunk_id
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

    def close(self) -> None:
        self.connection.close()


def connect_runtime_database(root: str | Path | None = None) -> RuntimeAuditDatabase:
    return RuntimeAuditDatabase(root)
