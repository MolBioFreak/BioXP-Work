from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import time
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
    entrypoint_id TEXT NOT NULL DEFAULT 'unknown',
    caller_class TEXT NOT NULL DEFAULT 'operator',
    control_class TEXT NOT NULL DEFAULT 'service',
    idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1 CHECK(idempotency_replay_enabled IN (0,1)),
    action_id TEXT NOT NULL,
    status TEXT NOT NULL,
    safety_class TEXT,
    ownership_generation INTEGER NOT NULL DEFAULT 0,
    source_identity_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(source_identity_json)),
    requested_inputs_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(requested_inputs_json)),
    effective_inputs_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(effective_inputs_json)),
    started_at TEXT NOT NULL,
    finished_at TEXT,
    duration_ms REAL,
    controller_acknowledged INTEGER NOT NULL DEFAULT 0 CHECK(controller_acknowledged IN (0,1)),
    physical_effect_verified INTEGER NOT NULL DEFAULT 0 CHECK(physical_effect_verified IN (0,1)),
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
    status TEXT NOT NULL,
    ownership_generation INTEGER NOT NULL,
    requested_inputs_json TEXT NOT NULL CHECK(json_valid(requested_inputs_json)),
    effective_inputs_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(effective_inputs_json)),
    created_at REAL NOT NULL,
    updated_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS runtime_events (
    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
    command_id TEXT REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    event_source TEXT NOT NULL,
    event_kind TEXT NOT NULL,
    event_json TEXT NOT NULL CHECK(json_valid(event_json)),
    observed_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS runtime_evidence_links (
    evidence_link_id INTEGER PRIMARY KEY AUTOINCREMENT,
    evidence_artifact_id TEXT NOT NULL,
    command_id TEXT REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
    pipette_operation_id TEXT REFERENCES pipette_operations(pipette_operation_id) ON DELETE RESTRICT,
    link_kind TEXT NOT NULL,
    created_at REAL NOT NULL
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
CREATE INDEX IF NOT EXISTS runtime_events_command_idx
    ON runtime_events(command_id, event_id);
CREATE INDEX IF NOT EXISTS runtime_events_kind_idx
    ON runtime_events(event_kind, observed_at DESC, event_id DESC);
CREATE INDEX IF NOT EXISTS runtime_evidence_links_command_idx
    ON runtime_evidence_links(command_id, evidence_link_id);
CREATE INDEX IF NOT EXISTS serial206_receipts_command_idx
    ON serial206_receipts(stream, command_id);
CREATE UNIQUE INDEX IF NOT EXISTS serial206_receipts_idempotency_idx
    ON serial206_receipts(stream, idempotency_key)
    WHERE idempotency_key IS NOT NULL AND idempotency_replay_enabled=1;
CREATE INDEX IF NOT EXISTS serial206_receipts_time_idx
    ON serial206_receipts(stream, observed_at DESC);
"""

_APPEND_ONLY_TABLES = (
    "runtime_schema_migrations",
    "operator_transitions",
    "runtime_events",
    "runtime_evidence_links",
    "runtime_migration_receipts",
)


def _add_missing_columns(connection: sqlite3.Connection) -> None:
    columns = {
        str(row["name"])
        for row in connection.execute("PRAGMA table_info(operator_commands)")
    }
    additions = {
        "idempotency_replay_enabled": "INTEGER NOT NULL DEFAULT 1 CHECK(idempotency_replay_enabled IN (0,1))",
        "canonical_request_sha256": "TEXT NOT NULL DEFAULT ''",
        "operation": "TEXT NOT NULL DEFAULT 'operator_action'",
        "entrypoint_id": "TEXT NOT NULL DEFAULT 'unknown'",
        "caller_class": "TEXT NOT NULL DEFAULT 'operator'",
        "control_class": "TEXT NOT NULL DEFAULT 'service'",
        "source_identity_json": "TEXT NOT NULL DEFAULT '{}'",
        "requested_inputs_json": "TEXT NOT NULL DEFAULT '{}'",
        "effective_inputs_json": "TEXT NOT NULL DEFAULT '{}'",
    }
    for name, definition in additions.items():
        if name not in columns:
            connection.execute(f"ALTER TABLE operator_commands ADD COLUMN {name} {definition}")


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
                return dict(existing), False

            existing_command = connection.execute(
                "SELECT * FROM operator_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if existing_command is not None:
                if str(existing_command["canonical_request_sha256"] or "") != digest:
                    raise ValueError("command identity conflict")
                connection.execute("COMMIT")
                return dict(existing_command), False

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
                    command_id,idempotency_key,canonical_request_sha256,operation,entrypoint_id,
                    caller_class,control_class,idempotency_replay_enabled,action_id,status,
                    safety_class,ownership_generation,source_identity_json,requested_inputs_json,
                    effective_inputs_json,started_at,receipt_json,updated_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    command_id,
                    idempotency_key,
                    digest,
                    str(payload["operation"]),
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
                        control_class,status,ownership_generation,requested_inputs_json,created_at,updated_at
                    ) VALUES(?,?,?,?,?,?,?,?,?,?,?)
                    """,
                    (
                        operation_id,
                        command_id,
                        str(payload["operation"]),
                        str(payload["entrypoint_id"]),
                        str(payload["caller_class"]),
                        str(payload["control_class"]),
                        "reserved",
                        int(payload["ownership_generation"]),
                        canonical_json(requested_inputs),
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
            return dict(row), True
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise

    def close(self) -> None:
        self.connection.close()


def connect_runtime_database(root: str | Path | None = None) -> RuntimeAuditDatabase:
    return RuntimeAuditDatabase(root)
