from __future__ import annotations

import ast
import hashlib
import fcntl
import inspect
import json
import os
import re
import secrets
import sqlite3
import tempfile
import time
import uuid
from collections.abc import Callable, Iterable, Mapping
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from .oem_runtime_types import OEMRuntimeSnapshot, utc_ts
from .runtime_audit_store import (
    RuntimeAuditDatabase,
    RuntimeMigrationIdentity,
    RUNTIME_RELEASE_RECEIPTS_DDL,
    RUNTIME_RELEASE_RECEIPTS_INDEX_DDL,
    RUNTIME_RELEASE_RECEIPTS_TRIGGER_DDL,
    _expected_foundation_connection,
    assert_migration_slot,
    ensure_schema,
    runtime_audit_migration_identity,
    runtime_lifecycle_lock,
    runtime_write_coordinator,
    verify_runtime_audit_foundation,
)


MAX_SERIAL206_INTERRUPT_FALLBACK_ARCHIVES = 8

SERIAL206_SCHEMA_VERSION = 2
REPORT_IDENTITY_SCHEMA_VERSION = 3
RUNTIME_RELEASE_SCHEMA_VERSION = 4
OPERATOR_COMMAND_PLANE_SCHEMA_VERSION = 5
SERIAL206_BOARD4_MEMBERS = {"y": 0, "z": 1, "gripper": 2}
_RUNTIME_PHYSICAL_SCHEMA_SHA256_BY_VERSION = {
    2: "1a6937590cbf8b12ec96faf2f8f62bb8d5560d1c3d1e24d1ff2f3599cc8a4075",
    3: "c10b9517ff0134b44c0fcec240fdcfafc640d3c56634c1fb9a88eaea87995317",
    4: "c10b9517ff0134b44c0fcec240fdcfafc640d3c56634c1fb9a88eaea87995317",
    5: "0ce5be874ced2cf3dcf94034c31e9202469517fd7355238fd4b5981e5aad289a",
}
_RUNTIME_PHYSICAL_TABLES = {
    "runtime_metadata", "runtime_retired_json_artifacts", "serial206_authority_snapshots",
    "runtime_state_snapshots", "runtime_journal", "runtime_movement_runs", "serial206_receipts",
    "operator_commands", "operator_transitions", "serial206_board_authority",
    "serial206_board_transitions", "serial206_axis_authority",
    "serial206_interrupt_imports",
}
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
        if not path.is_file():
            digests[name] = None
            continue
        raw = path.read_bytes()
        json.loads(raw)
        digests[name] = hashlib.sha256(raw).hexdigest()
    return digests


def _load_schema_sources(root: Path) -> dict[str, dict[str, Any] | None]:
    payloads: dict[str, dict[str, Any] | None] = {}
    for name in ("reference-state.json", "serial206_oem_initialization_state.json"):
        path = root / name
        if not path.is_file():
            payloads[name] = None
            continue
        value = json.loads(path.read_bytes())
        if not isinstance(value, dict):
            raise RuntimeError(f"runtime migration source {name} must contain a JSON object")
        payloads[name] = value
    return payloads


def _legacy_axis_reference(payload: Mapping[str, Any] | None, axis: str) -> Mapping[str, Any]:
    if not isinstance(payload, Mapping):
        return {}
    rows = payload.get("rows")
    row = rows.get(axis) if isinstance(rows, Mapping) else None
    return row if isinstance(row, Mapping) else {}


def _find_nonnegative_int(payload: Any, *keys: str) -> int | None:
    pending = [payload]
    visited = 0
    wanted = set(keys)
    while pending and visited < 500:
        current = pending.pop(0)
        visited += 1
        if isinstance(current, Mapping):
            for key, value in current.items():
                if str(key) in wanted and type(value) is int and value >= 0:
                    return value
                if isinstance(value, (Mapping, list, tuple)):
                    pending.append(value)
        elif isinstance(current, (list, tuple)):
            pending.extend(current)
    return None


def _legacy_table_identity(
    connection: sqlite3.Connection,
    table: str,
    columns: tuple[str, ...] | None = None,
) -> tuple[tuple[str, ...], int, str] | None:
    if connection.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (table,)).fetchone() is None:
        return None
    info = connection.execute(f"PRAGMA table_info({table})").fetchall()
    selected_columns = columns or tuple(str(row[1]) for row in info)
    available = {str(row[1]) for row in info}
    if any(column not in available for column in selected_columns):
        raise RuntimeError(f"runtime v2 migration removed frozen legacy columns: {table}")
    primary = [str(row[1]) for row in sorted((row for row in info if int(row[5]) > 0), key=lambda row: int(row[5]))]
    order = ",".join(f'"{column.replace(chr(34), chr(34) * 2)}"' for column in primary) or "rowid"
    projection = ",".join(f'"{column.replace(chr(34), chr(34) * 2)}"' for column in selected_columns)
    rows = connection.execute(f"SELECT {projection} FROM {table} ORDER BY {order}").fetchall()
    encoded = json.dumps([list(row) for row in rows], sort_keys=False, separators=(",", ":"), default=str).encode("utf-8")
    return selected_columns, len(rows), hashlib.sha256(encoded).hexdigest()


def _is_legacy_v1_serial206_receipts_schema(connection: sqlite3.Connection) -> bool:
    table_row = connection.execute(
        "SELECT sql FROM sqlite_master WHERE type='table' AND name='serial206_receipts'"
    ).fetchone()
    if table_row is None:
        return False
    normalized = "".join(str(table_row[0] or "").upper().split())
    if "CHECK(IDEMPOTENCY_REPLAY_ENABLEDIN(0,1))" in normalized:
        return False
    columns = tuple(str(row[1]) for row in connection.execute("PRAGMA table_info(serial206_receipts)"))
    expected = (
        "stream", "receipt_id", "command_id", "idempotency_key",
        "idempotency_replay_enabled", "status", "observed_at", "receipt_json",
    )
    if columns != expected:
        raise RuntimeError("serial206_receipts is not the recognized v1 lineage")
    return True


def _normalize_legacy_v1_serial206_receipts(connection: sqlite3.Connection) -> None:
    """Restore the canonical replay constraint on the recognized v1 receipt table."""
    if not _is_legacy_v1_serial206_receipts_schema(connection):
        return
    connection.execute("BEGIN IMMEDIATE")
    try:
        connection.execute("ALTER TABLE serial206_receipts RENAME TO serial206_receipts_legacy_v1")
        connection.execute(
            """
            CREATE TABLE serial206_receipts (
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
            ) WITHOUT ROWID
            """
        )
        connection.execute(
            "INSERT INTO serial206_receipts SELECT * FROM serial206_receipts_legacy_v1"
        )
        connection.execute("DROP TABLE serial206_receipts_legacy_v1")
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise



def _execute_schema_batch(connection: sqlite3.Connection, script: str) -> None:
    for statement in script.split(";"):
        statement = statement.strip()
        if statement:
            connection.execute(statement)


def _runtime_physical_schema_sha256(connection: sqlite3.Connection) -> str:
    def normalize_sql(value: Any) -> str:
        return "".join(str(value or "").upper().split())

    manifest: dict[str, Any] = {}
    for table in sorted(_RUNTIME_PHYSICAL_TABLES):
        table_row = connection.execute(
            "SELECT sql FROM sqlite_master WHERE type='table' AND name=?", (table,)
        ).fetchone()
        if table_row is None:
            return "missing:" + table
        indexes = []
        for index_row in connection.execute(f'PRAGMA index_list("{table}")').fetchall():
            index_name = str(index_row[1])
            index_sql_row = connection.execute(
                "SELECT sql FROM sqlite_master WHERE type='index' AND name=?", (index_name,)
            ).fetchone()
            indexes.append((
                index_name,
                int(index_row[2]),
                str(index_row[3]),
                int(index_row[4]),
                normalize_sql(index_sql_row[0] if index_sql_row else ""),
                [tuple(row) for row in connection.execute(f'PRAGMA index_xinfo("{index_name}")').fetchall()],
            ))
        manifest[table] = {
            "sql": normalize_sql(table_row[0]),
            "columns": [tuple(row) for row in connection.execute(f'PRAGMA table_xinfo("{table}")').fetchall()],
            "indexes": sorted(indexes),
            "foreign_keys": [tuple(row) for row in connection.execute(f'PRAGMA foreign_key_list("{table}")').fetchall()],
            "triggers": [
                (str(row[0]), normalize_sql(row[1]))
                for row in connection.execute(
                    "SELECT name,sql FROM sqlite_master WHERE type='trigger' AND tbl_name=? ORDER BY name",
                    (table,),
                ).fetchall()
            ],
        }
    encoded = json.dumps(manifest, sort_keys=True, separators=(",", ":"), allow_nan=False, default=str)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


_OPERATOR_COMMAND_COLUMNS = (
    "sequence", "command_id", "idempotency_key", "canonical_request_sha256", "operation",
    "command_kind", "entrypoint_id", "caller_class", "control_class",
    "idempotency_replay_enabled", "action_id", "status", "safety_class",
    "ownership_generation", "connection_generation", "source_identity_json",
    "requested_inputs_json", "effective_inputs_json", "started_at", "admitted_at",
    "dispatched_at", "finished_at", "duration_ms", "delivery_verified",
    "controller_acknowledged", "completion_verified", "hardware_precondition_verified",
    "hardware_postcondition_verified", "physical_effect_verified", "outcome", "failure_code",
    "evidence_state", "receipt_json", "response_summary_json", "evidence_relpath",
    "evidence_sha256", "evidence_bytes", "updated_at",
)


_LEGACY_V1_OPERATOR_COMMAND_COLUMNS = {
    "sequence", "command_id", "idempotency_key", "idempotency_replay_enabled",
    "action_id", "status", "safety_class", "ownership_generation", "started_at",
    "finished_at", "duration_ms", "controller_acknowledged",
    "physical_effect_verified", "receipt_json", "response_summary_json",
    "evidence_relpath", "evidence_sha256", "evidence_bytes", "updated_at",
}


def _is_legacy_v1_operator_schema(connection: sqlite3.Connection) -> bool:
    command_columns = {
        str(row[1]) for row in connection.execute("PRAGMA table_info(operator_commands)")
    }
    transition_columns = tuple(
        str(row[1]) for row in connection.execute("PRAGMA table_info(operator_transitions)")
    )
    return (
        command_columns == _LEGACY_V1_OPERATOR_COMMAND_COLUMNS
        and transition_columns == ("transition_id", "command_id", "state", "observed_at", "detail_json")
    )


def _is_additive_operator_schema(connection: sqlite3.Connection) -> bool:
    command_columns = {
        str(row[1]) for row in connection.execute("PRAGMA table_info(operator_commands)")
    }
    if command_columns != set(_OPERATOR_COMMAND_COLUMNS):
        return False
    transition_columns = tuple(
        str(row[1]) for row in connection.execute("PRAGMA table_info(operator_transitions)")
    )
    if transition_columns != ("transition_id", "command_id", "state", "observed_at", "detail_json"):
        return False
    transition_fks = connection.execute("PRAGMA foreign_key_list(operator_transitions)").fetchall()
    return len(transition_fks) == 1 and str(transition_fks[0][6]).upper() in {"CASCADE", "NO ACTION"}


def _rebuild_additive_operator_schema(connection: sqlite3.Connection) -> None:
    """Rebuild a recognized legacy/additive lineage into the canonical schema."""
    if not (_is_additive_operator_schema(connection) or _is_legacy_v1_operator_schema(connection)):
        raise RuntimeError("operator schema is not a recognized rebuildable lineage")

    source_command_columns = {
        str(row[1]) for row in connection.execute("PRAGMA table_info(operator_commands)")
    }
    foreign_keys = int(connection.execute("PRAGMA foreign_keys").fetchone()[0])
    legacy_alter_table = int(connection.execute("PRAGMA legacy_alter_table").fetchone()[0])
    if connection.in_transaction:
        raise RuntimeError("operator schema rebuild requires a transaction boundary")
    sequence_rows = dict(connection.execute(
        "SELECT name,seq FROM sqlite_sequence WHERE name IN ('operator_commands','operator_transitions')"
    ).fetchall())
    columns = ",".join(_OPERATOR_COMMAND_COLUMNS)
    try:
        connection.execute("PRAGMA foreign_keys=OFF")
        connection.execute("PRAGMA legacy_alter_table=ON")
        connection.execute("BEGIN IMMEDIATE")
        connection.execute("ALTER TABLE operator_transitions RENAME TO operator_transitions_additive_v2")
        connection.execute("ALTER TABLE operator_commands RENAME TO operator_commands_additive_v2")
        connection.execute("PRAGMA legacy_alter_table=OFF")
        _execute_schema_batch(connection,
            """
            CREATE TABLE operator_commands (
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
            CREATE TABLE operator_transitions (
                transition_id INTEGER PRIMARY KEY AUTOINCREMENT,
                command_id TEXT NOT NULL REFERENCES operator_commands(command_id) ON DELETE RESTRICT,
                state TEXT NOT NULL,
                observed_at REAL NOT NULL,
                detail_json TEXT CHECK(detail_json IS NULL OR json_valid(detail_json))
            );
            """
        )
        canonical_column_info = {
            str(row[1]): row for row in connection.execute("PRAGMA table_info(operator_commands)")
        }
        select_expressions = []
        for column in _OPERATOR_COMMAND_COLUMNS:
            if column in source_command_columns:
                select_expressions.append(f'"{column}"')
                continue
            default_value = canonical_column_info[column][4]
            select_expressions.append(str(default_value) if default_value is not None else "NULL")
        connection.execute(
            f"INSERT INTO operator_commands({columns}) SELECT {','.join(select_expressions)} "
            "FROM operator_commands_additive_v2"
        )
        connection.execute(
            "INSERT INTO operator_transitions(transition_id,command_id,state,observed_at,detail_json) "
            "SELECT transition_id,command_id,state,observed_at,detail_json "
            "FROM operator_transitions_additive_v2"
        )
        connection.execute("DROP TABLE operator_transitions_additive_v2")
        connection.execute("DROP TABLE operator_commands_additive_v2")
        _execute_schema_batch(connection,
            """
            CREATE INDEX operator_commands_history_idx ON operator_commands(sequence DESC);
            CREATE INDEX operator_commands_updated_idx ON operator_commands(updated_at DESC, sequence DESC);
            CREATE INDEX operator_commands_action_status_idx ON operator_commands(action_id, status, sequence DESC);
            CREATE UNIQUE INDEX operator_commands_replay_key_idx
                ON operator_commands(idempotency_key) WHERE idempotency_replay_enabled=1;
            CREATE INDEX operator_transitions_command_idx
                ON operator_transitions(command_id, transition_id);
            """
        )
        connection.execute(
            """
            CREATE TRIGGER operator_transitions_append_only_update
                BEFORE UPDATE ON operator_transitions
                BEGIN
                    SELECT RAISE(ABORT, 'append-only table cannot be updated');
                END
            """
        )
        connection.execute(
            """
            CREATE TRIGGER operator_transitions_append_only_delete
                BEFORE DELETE ON operator_transitions
                BEGIN
                    SELECT RAISE(ABORT, 'append-only table cannot be deleted');
                END
            """
        )
        for name, sequence in sequence_rows.items():
            connection.execute("UPDATE sqlite_sequence SET seq=? WHERE name=?", (int(sequence), str(name)))
        foreign_key_errors = connection.execute("PRAGMA foreign_key_check").fetchall()
        if foreign_key_errors:
            raise RuntimeError(f"operator schema rebuild foreign-key failure: {foreign_key_errors[:3]}")
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise
    finally:
        connection.execute(f"PRAGMA legacy_alter_table={legacy_alter_table}")
        connection.execute(f"PRAGMA foreign_keys={foreign_keys}")
def _drop_unconditional_idempotency_indexes(connection: sqlite3.Connection) -> None:
    for row in connection.execute("PRAGMA index_list(operator_commands)").fetchall():
        if not row[1] or row[3] != "c" or row[4]:
            continue
        name = str(row[1])
        columns = [
            str(info[2])
            for info in connection.execute(
                "SELECT * FROM pragma_index_info(?)", (name,)
            ).fetchall()
        ]
        if columns == ["idempotency_key"]:
            connection.execute(f'DROP INDEX "{name.replace(chr(34), chr(34) * 2)}"')


def _create_v1_runtime_schema(connection: sqlite3.Connection) -> None:
    _execute_schema_batch(connection,
        """
        CREATE TABLE IF NOT EXISTS runtime_metadata (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL,
            updated_at REAL NOT NULL
        ) WITHOUT ROWID;
        CREATE TABLE IF NOT EXISTS runtime_retired_json_artifacts (
            source_name TEXT PRIMARY KEY,
            content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
            content_blob BLOB NOT NULL,
            imported_at REAL NOT NULL
        ) WITHOUT ROWID;
        CREATE TABLE IF NOT EXISTS serial206_authority_snapshots (
            sequence INTEGER PRIMARY KEY AUTOINCREMENT,
            state_json TEXT NOT NULL CHECK(json_valid(state_json)),
            state_sha256 TEXT NOT NULL CHECK(length(state_sha256)=64),
            receipt_set_json TEXT NOT NULL CHECK(json_valid(receipt_set_json)),
            receipt_set_sha256 TEXT NOT NULL CHECK(length(receipt_set_sha256)=64),
            created_at REAL NOT NULL
        );
        CREATE TABLE IF NOT EXISTS runtime_state_snapshots (
            sequence INTEGER PRIMARY KEY,
            state_json TEXT NOT NULL CHECK(json_valid(state_json)),
            state_sha256 TEXT NOT NULL CHECK(length(state_sha256)=64),
            created_at REAL NOT NULL
        );
        CREATE TABLE IF NOT EXISTS runtime_journal (
            sequence INTEGER PRIMARY KEY,
            stream TEXT NOT NULL,
            payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
            payload_sha256 TEXT NOT NULL CHECK(length(payload_sha256)=64),
            created_at REAL NOT NULL
        );
        CREATE INDEX IF NOT EXISTS runtime_journal_stream_idx
            ON runtime_journal(stream,sequence DESC);
        CREATE TABLE IF NOT EXISTS runtime_movement_runs (
            run_id TEXT PRIMARY KEY,
            sequence INTEGER NOT NULL,
            run_json TEXT NOT NULL CHECK(json_valid(run_json)),
            run_sha256 TEXT NOT NULL CHECK(length(run_sha256)=64),
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
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_authority_snapshots_coherence_v1
        BEFORE INSERT ON serial206_authority_snapshots
        WHEN NEW.state_json<>canonical_json(NEW.state_json)
          OR NEW.state_sha256<>sha256_utf8(NEW.state_json)
          OR NEW.receipt_set_json<>canonical_json(NEW.receipt_set_json)
          OR NEW.receipt_set_sha256<>sha256_utf8(NEW.receipt_set_json)
        BEGIN SELECT RAISE(ABORT, 'serial-206 authority snapshot is incoherent'); END
        """
    )
    for table in ("serial206_authority_snapshots", "runtime_state_snapshots", "runtime_journal", "runtime_retired_json_artifacts"):
        connection.execute(
            f"CREATE TRIGGER IF NOT EXISTS {table}_no_update_v1 BEFORE UPDATE ON {table} BEGIN SELECT RAISE(ABORT,'{table} is append-only'); END"
        )
        connection.execute(
            f"CREATE TRIGGER IF NOT EXISTS {table}_no_delete_v1 BEFORE DELETE ON {table} BEGIN SELECT RAISE(ABORT,'{table} is append-only'); END"
        )
    for table in ("serial206_authority_snapshots", "runtime_state_snapshots", "runtime_journal"):
        connection.execute(
            f"CREATE TRIGGER IF NOT EXISTS {table}_authorized_insert_v2 BEFORE INSERT ON {table} "
            f"WHEN authority_write_allowed()<>1 BEGIN SELECT RAISE(ABORT, '{table} insert requires authoritative writer'); END"
        )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_state_snapshots_coherence_v1
        BEFORE INSERT ON runtime_state_snapshots
        WHEN NEW.state_json<>canonical_json(NEW.state_json)
          OR NEW.state_sha256<>sha256_utf8(NEW.state_json)
        BEGIN SELECT RAISE(ABORT, 'runtime state snapshot is incoherent'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_journal_coherence_v1
        BEFORE INSERT ON runtime_journal
        WHEN NEW.payload_json<>canonical_json(NEW.payload_json)
          OR NEW.payload_sha256<>sha256_utf8(NEW.payload_json)
        BEGIN SELECT RAISE(ABORT, 'runtime journal row is incoherent'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_retired_json_artifacts_coherence_v1
        BEFORE INSERT ON runtime_retired_json_artifacts
        WHEN NEW.content_sha256<>sha256_blob(NEW.content_blob)
          OR authority_write_allowed()<>1
        BEGIN SELECT RAISE(ABORT, 'retired runtime JSON evidence is incoherent'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_movement_runs_coherence_insert_v1
        BEFORE INSERT ON runtime_movement_runs
        WHEN NEW.run_json<>canonical_json(NEW.run_json)
          OR NEW.run_sha256<>sha256_utf8(NEW.run_json)
          OR json_extract(NEW.run_json,'$.run_id') IS NOT NEW.run_id
          OR json_extract(NEW.run_json,'$.sequence') IS NOT NEW.sequence
        BEGIN SELECT RAISE(ABORT, 'runtime movement run is incoherent'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_movement_runs_coherence_update_v1
        BEFORE UPDATE ON runtime_movement_runs
        WHEN NEW.run_id IS NOT OLD.run_id
          OR NEW.run_json<>canonical_json(NEW.run_json)
          OR NEW.run_sha256<>sha256_utf8(NEW.run_json)
          OR json_extract(NEW.run_json,'$.run_id') IS NOT NEW.run_id
          OR json_extract(NEW.run_json,'$.sequence') IS NOT NEW.sequence
        BEGIN SELECT RAISE(ABORT, 'runtime movement run is incoherent'); END
        """
    )
    for trigger_name, operation in (
        ("runtime_movement_runs_authorized_insert_v2", "INSERT"),
        ("runtime_movement_runs_authorized_update_v2", "UPDATE"),
        ("runtime_movement_runs_authorized_delete_v2", "DELETE"),
    ):
        connection.execute(
            f"CREATE TRIGGER IF NOT EXISTS {trigger_name} BEFORE {operation} ON runtime_movement_runs "
            "WHEN authority_write_allowed()<>1 "
            "BEGIN SELECT RAISE(ABORT,'runtime movement-run writer is not authoritative'); END"
        )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_metadata_serial206_authority_no_insert_v1
        BEFORE INSERT ON runtime_metadata
        WHEN NEW.key='serial206_oem_initialization_state'
        BEGIN SELECT RAISE(ABORT, 'mutable serial-206 metadata authority is retired'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_metadata_serial206_authority_no_update_v1
        BEFORE UPDATE ON runtime_metadata
        WHEN OLD.key='serial206_oem_initialization_state' OR NEW.key='serial206_oem_initialization_state'
        BEGIN SELECT RAISE(ABORT, 'mutable serial-206 metadata authority is retired'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS runtime_metadata_serial206_authority_no_delete_v1
        BEFORE DELETE ON runtime_metadata
        WHEN OLD.key='serial206_oem_initialization_state'
        BEGIN SELECT RAISE(ABORT, 'mutable serial-206 metadata authority is retired'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_xyz_receipts_no_update_v1
        BEFORE UPDATE ON serial206_receipts
        WHEN OLD.stream IN ('x','y','z')
        BEGIN SELECT RAISE(ABORT, 'serial-206 provider receipts are append-only'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_xyz_receipts_no_delete_v1
        BEFORE DELETE ON serial206_receipts
        WHEN OLD.stream IN ('x','y','z')
        BEGIN SELECT RAISE(ABORT, 'serial-206 provider receipts are append-only'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_xyz_receipts_authorized_insert_v2
        BEFORE INSERT ON serial206_receipts
        WHEN NEW.stream IN ('x','y','z') AND authority_write_allowed()<>1
        BEGIN SELECT RAISE(ABORT, 'serial-206 provider receipt writer is not authoritative'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_initialization_runs_no_delete
        BEFORE DELETE ON serial206_receipts
        WHEN OLD.stream IN ('initialize_motors','initialize_motion')
        BEGIN SELECT RAISE(ABORT, 'serial-206 initialization runs are immutable'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_initialization_runs_terminal_immutable
        BEFORE UPDATE ON serial206_receipts
        WHEN OLD.stream IN ('initialize_motors','initialize_motion') AND (
            OLD.status <> 'running'
            OR NEW.stream <> OLD.stream
            OR NEW.receipt_id <> OLD.receipt_id
            OR NEW.command_id IS NOT OLD.command_id
            OR NEW.idempotency_key IS NOT OLD.idempotency_key
            OR NEW.idempotency_replay_enabled <> OLD.idempotency_replay_enabled
            OR NEW.status NOT IN ('completed','failed','ambiguous')
        )
        BEGIN SELECT RAISE(ABORT, 'serial-206 initialization run identity or terminal receipt is immutable'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_initialization_request_identity_immutable_v2
        BEFORE UPDATE ON serial206_receipts
        WHEN OLD.stream IN ('initialize_motors','initialize_motion') AND (
            json_extract(NEW.receipt_json,'$.schema_version') IS NOT json_extract(OLD.receipt_json,'$.schema_version')
            OR json_extract(NEW.receipt_json,'$.receipt_id') IS NOT json_extract(OLD.receipt_json,'$.receipt_id')
            OR json_extract(NEW.receipt_json,'$.command_id') IS NOT json_extract(OLD.receipt_json,'$.command_id')
            OR json_extract(NEW.receipt_json,'$.run_id') IS NOT json_extract(OLD.receipt_json,'$.run_id')
            OR json_extract(NEW.receipt_json,'$.idempotency_key') IS NOT json_extract(OLD.receipt_json,'$.idempotency_key')
            OR json_extract(NEW.receipt_json,'$.intent') IS NOT json_extract(OLD.receipt_json,'$.intent')
            OR json_extract(NEW.receipt_json,'$.started_at') IS NOT json_extract(OLD.receipt_json,'$.started_at')
            OR json_extract(NEW.receipt_json,'$.request_sha256') IS NOT json_extract(OLD.receipt_json,'$.request_sha256')
            OR json(json_extract(NEW.receipt_json,'$.request')) IS NOT json(json_extract(OLD.receipt_json,'$.request'))
        )
        BEGIN SELECT RAISE(ABORT, 'serial-206 initialization request identity is immutable'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_initialization_receipt_coherence_insert_v3
        BEFORE INSERT ON serial206_receipts
        WHEN NEW.stream IN ('initialize_motors','initialize_motion') AND (
            NEW.status NOT IN ('running','completed','failed','ambiguous')
            OR json_type(NEW.receipt_json,'$.status') IS NOT 'text'
            OR json_extract(NEW.receipt_json,'$.status') IS NOT NEW.status
            OR json_extract(NEW.receipt_json,'$.receipt_id') IS NOT NEW.receipt_id
            OR json_extract(NEW.receipt_json,'$.command_id') IS NOT NEW.command_id
            OR json_extract(NEW.receipt_json,'$.run_id') IS NOT NEW.command_id
            OR json_extract(NEW.receipt_json,'$.idempotency_key') IS NOT NEW.idempotency_key
            OR json_extract(NEW.receipt_json,'$.idempotency_replay_enabled') IS NOT NEW.idempotency_replay_enabled
            OR json_extract(NEW.receipt_json,'$.intent') IS NOT NEW.stream
            OR json_type(NEW.receipt_json,'$.request_sha256') IS NOT 'text'
            OR length(json_extract(NEW.receipt_json,'$.request_sha256')) <> 64
            OR (NEW.status='running' AND json_type(NEW.receipt_json,'$.response') IS NOT 'null')
            OR (NEW.status IN ('completed','failed','ambiguous') AND json_type(NEW.receipt_json,'$.response') IS NOT 'object')
        )
        BEGIN SELECT RAISE(ABORT, 'serial-206 initialization receipt is incoherent'); END
        """
    )
    connection.execute(
        """
        CREATE TRIGGER IF NOT EXISTS serial206_initialization_receipt_coherence_update_v3
        BEFORE UPDATE ON serial206_receipts
        WHEN NEW.stream IN ('initialize_motors','initialize_motion') AND (
            NEW.status NOT IN ('running','completed','failed','ambiguous')
            OR json_type(NEW.receipt_json,'$.status') IS NOT 'text'
            OR json_extract(NEW.receipt_json,'$.status') IS NOT NEW.status
            OR json_extract(NEW.receipt_json,'$.receipt_id') IS NOT NEW.receipt_id
            OR json_extract(NEW.receipt_json,'$.command_id') IS NOT NEW.command_id
            OR json_extract(NEW.receipt_json,'$.run_id') IS NOT NEW.command_id
            OR json_extract(NEW.receipt_json,'$.idempotency_key') IS NOT NEW.idempotency_key
            OR json_extract(NEW.receipt_json,'$.idempotency_replay_enabled') IS NOT NEW.idempotency_replay_enabled
            OR json_extract(NEW.receipt_json,'$.intent') IS NOT NEW.stream
            OR json_type(NEW.receipt_json,'$.request_sha256') IS NOT 'text'
            OR length(json_extract(NEW.receipt_json,'$.request_sha256')) <> 64
            OR (NEW.status='running' AND json_type(NEW.receipt_json,'$.response') IS NOT 'null')
            OR (NEW.status IN ('completed','failed','ambiguous') AND json_type(NEW.receipt_json,'$.response') IS NOT 'object')
        )
        BEGIN SELECT RAISE(ABORT, 'serial-206 initialization receipt is incoherent'); END
        """
    )


_RUNTIME_AUTHORITY_TRIGGER_NAMES = (
    "serial206_authority_snapshots_coherence_v1",
    "serial206_authority_snapshots_no_update_v1",
    "serial206_authority_snapshots_no_delete_v1",
    "runtime_state_snapshots_no_update_v1",
    "runtime_state_snapshots_no_delete_v1",
    "runtime_journal_no_update_v1",
    "runtime_journal_no_delete_v1",
    "serial206_authority_snapshots_authorized_insert_v2",
    "runtime_state_snapshots_authorized_insert_v2",
    "runtime_journal_authorized_insert_v2",
    "runtime_retired_json_artifacts_no_update_v1",
    "runtime_retired_json_artifacts_no_delete_v1",
    "runtime_retired_json_artifacts_coherence_v1",
    "runtime_state_snapshots_coherence_v1",
    "runtime_journal_coherence_v1",
    "runtime_movement_runs_coherence_insert_v1",
    "runtime_movement_runs_coherence_update_v1",
    "runtime_movement_runs_authorized_insert_v2",
    "runtime_movement_runs_authorized_update_v2",
    "runtime_movement_runs_authorized_delete_v2",
    "runtime_metadata_serial206_authority_no_insert_v1",
    "runtime_metadata_serial206_authority_no_update_v1",
    "runtime_metadata_serial206_authority_no_delete_v1",
    "serial206_xyz_receipts_no_update_v1",
    "serial206_xyz_receipts_no_delete_v1",
    "serial206_xyz_receipts_authorized_insert_v2",
    "serial206_initialization_runs_no_delete",
    "serial206_initialization_runs_terminal_immutable",
    "serial206_initialization_request_identity_immutable_v2",
    "serial206_initialization_receipt_coherence_insert_v3",
    "serial206_initialization_receipt_coherence_update_v3",
    "serial206_board_transitions_no_update",
    "serial206_board_transitions_no_delete",
    "serial206_board_authority_authorized_insert_v3",
    "serial206_board_authority_authorized_update_v3",
    "serial206_board_authority_authorized_delete_v3",
    "serial206_axis_authority_authorized_insert_v3",
    "serial206_axis_authority_authorized_update_v3",
    "serial206_axis_authority_authorized_delete_v3",
    "serial206_board_transitions_authorized_insert_v3",
)


def _reinstall_runtime_authority_triggers(connection: sqlite3.Connection) -> None:
    for name in _RUNTIME_AUTHORITY_TRIGGER_NAMES:
        connection.execute(f'DROP TRIGGER IF EXISTS "{name}"')
    _create_v1_runtime_schema(connection)
    _create_v2_authority_schema(connection)


def _create_v2_authority_schema(connection: sqlite3.Connection) -> None:
    _execute_schema_batch(connection,
        """
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
        CREATE TABLE IF NOT EXISTS serial206_board_transitions (
            sequence INTEGER PRIMARY KEY AUTOINCREMENT,
            transition_id TEXT NOT NULL UNIQUE,
            requested_active INTEGER NOT NULL CHECK(requested_active IN (0,1)),
            ownership_generation INTEGER NOT NULL CHECK(ownership_generation>=0),
            delivery_attempted INTEGER NOT NULL CHECK(delivery_attempted IN (0,1)),
            reply_valid INTEGER NOT NULL CHECK(reply_valid IN (0,1)),
            status_code INTEGER NOT NULL,
            continuity_proven INTEGER NOT NULL CHECK(continuity_proven IN (0,1)),
            accepted INTEGER NOT NULL CHECK(accepted IN (0,1)),
            state_before TEXT NOT NULL,
            state_after TEXT NOT NULL,
            prior_board_epoch INTEGER,
            active_board_epoch INTEGER,
            created_at REAL NOT NULL
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
    connection.execute(
        "CREATE TRIGGER IF NOT EXISTS serial206_board_transitions_no_update "
        "BEFORE UPDATE ON serial206_board_transitions "
        "BEGIN SELECT RAISE(ABORT,'serial206 board transition history is immutable'); END"
    )
    connection.execute(
        "CREATE TRIGGER IF NOT EXISTS serial206_board_transitions_no_delete "
        "BEFORE DELETE ON serial206_board_transitions "
        "BEGIN SELECT RAISE(ABORT,'serial206 board transition history is immutable'); END"
    )
    for trigger_name, table_name, operation in (
        ("serial206_board_authority_authorized_insert_v3", "serial206_board_authority", "INSERT"),
        ("serial206_board_authority_authorized_update_v3", "serial206_board_authority", "UPDATE"),
        ("serial206_board_authority_authorized_delete_v3", "serial206_board_authority", "DELETE"),
        ("serial206_axis_authority_authorized_insert_v3", "serial206_axis_authority", "INSERT"),
        ("serial206_axis_authority_authorized_update_v3", "serial206_axis_authority", "UPDATE"),
        ("serial206_axis_authority_authorized_delete_v3", "serial206_axis_authority", "DELETE"),
        ("serial206_board_transitions_authorized_insert_v3", "serial206_board_transitions", "INSERT"),
    ):
        connection.execute(
            f"CREATE TRIGGER IF NOT EXISTS {trigger_name} BEFORE {operation} ON {table_name} "
            "WHEN authority_write_allowed()<>1 "
            "BEGIN SELECT RAISE(ABORT,'serial206 authority writer is not authoritative'); END"
        )


def serial206_runtime_migration_identity() -> RuntimeMigrationIdentity:
    ddl_source = inspect.getsource(_create_v2_authority_schema).encode("utf-8")
    return RuntimeMigrationIdentity(
        version=SERIAL206_SCHEMA_VERSION,
        name="serial206_runtime_authority",
        ddl_sha256=hashlib.sha256(ddl_source).hexdigest(),
    )


_REPORT_IDENTITY_TRIGGER_DDL = (
    """
    CREATE TRIGGER runtime_metadata_report_identity_insert_shape
    BEFORE INSERT ON runtime_metadata
    WHEN NEW.key IN ('database_incarnation_id','report_cursor_hmac_key')
         AND (NEW.value='' OR (NEW.key='report_cursor_hmac_key'
              AND (length(NEW.value)<64 OR length(NEW.value)%2<>0
                   OR NEW.value<>lower(NEW.value)
                   OR NEW.value GLOB '*[^0-9a-f]*')))
    BEGIN
        SELECT RAISE(ABORT, 'report identity metadata has invalid shape');
    END
    """,
    """
    CREATE TRIGGER runtime_metadata_report_identity_immutable_update
    BEFORE UPDATE ON runtime_metadata
    WHEN OLD.key IN ('database_incarnation_id','report_cursor_hmac_key')
    BEGIN
        SELECT RAISE(ABORT, 'report identity metadata is immutable');
    END
    """,
    """
    CREATE TRIGGER runtime_metadata_report_identity_immutable_delete
    BEFORE DELETE ON runtime_metadata
    WHEN OLD.key IN ('database_incarnation_id','report_cursor_hmac_key')
    BEGIN
        SELECT RAISE(ABORT, 'report identity metadata is immutable');
    END
    """,
)


def _apply_report_identity_metadata_v1(connection: sqlite3.Connection, now: float) -> None:
    rows = {
        str(row["key"]): str(row["value"])
        for row in connection.execute(
            "SELECT key,value FROM runtime_metadata WHERE key IN (?,?)",
            ("database_incarnation_id", "report_cursor_hmac_key"),
        ).fetchall()
    }
    incarnation = rows.get("database_incarnation_id")
    if incarnation is None:
        incarnation = str(uuid.uuid4())
        connection.execute(
            "INSERT INTO runtime_metadata(key,value,updated_at) VALUES(?,?,?)",
            ("database_incarnation_id", incarnation, now),
        )
    cursor_key = rows.get("report_cursor_hmac_key")
    if cursor_key is None:
        cursor_key = secrets.token_hex(32)
        connection.execute(
            "INSERT INTO runtime_metadata(key,value,updated_at) VALUES(?,?,?)",
            ("report_cursor_hmac_key", cursor_key, now),
        )
    try:
        canonical_incarnation = str(uuid.UUID(incarnation))
    except (ValueError, AttributeError) as exc:
        raise RuntimeError("database_incarnation_id is not a UUID") from exc
    if canonical_incarnation != incarnation:
        raise RuntimeError("database_incarnation_id is not canonical")
    try:
        key_bytes = bytes.fromhex(cursor_key)
    except ValueError as exc:
        raise RuntimeError("report_cursor_hmac_key is not canonical hex") from exc
    if len(key_bytes) < 32 or cursor_key != cursor_key.lower() or cursor_key != key_bytes.hex():
        raise RuntimeError("report_cursor_hmac_key must be canonical hex for at least 32 bytes")
    for statement in _REPORT_IDENTITY_TRIGGER_DDL:
        connection.execute(statement)


def report_identity_migration_identity() -> RuntimeMigrationIdentity:
    ddl_source = (
        "\n".join(_REPORT_IDENTITY_TRIGGER_DDL)
        + inspect.getsource(_apply_report_identity_metadata_v1)
    ).encode("utf-8")
    return RuntimeMigrationIdentity(
        version=REPORT_IDENTITY_SCHEMA_VERSION,
        name="report_identity_metadata_v1",
        ddl_sha256=hashlib.sha256(ddl_source).hexdigest(),
    )



def _apply_runtime_release_start(connection: sqlite3.Connection) -> None:
    connection.execute(RUNTIME_RELEASE_RECEIPTS_DDL)
    connection.execute(RUNTIME_RELEASE_RECEIPTS_INDEX_DDL)
    for statement in RUNTIME_RELEASE_RECEIPTS_TRIGGER_DDL:
        connection.execute(statement)


def runtime_release_migration_identity() -> RuntimeMigrationIdentity:
    ddl_source = "\n".join(
        (
            RUNTIME_RELEASE_RECEIPTS_DDL,
            RUNTIME_RELEASE_RECEIPTS_INDEX_DDL,
            *RUNTIME_RELEASE_RECEIPTS_TRIGGER_DDL,
            inspect.getsource(_apply_runtime_release_start),
        )
    ).encode("utf-8")
    return RuntimeMigrationIdentity(
        version=RUNTIME_RELEASE_SCHEMA_VERSION,
        name="canonical_runtime_release_start",
        ddl_sha256=hashlib.sha256(ddl_source).hexdigest(),
    )


_OPERATOR_COMMAND_PLANE_TABLE_DDL = (
    """
    CREATE TABLE IF NOT EXISTS operator_plane_metadata (
        key TEXT PRIMARY KEY, value TEXT NOT NULL, updated_at REAL NOT NULL
    ) WITHOUT ROWID;
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_idempotency (
        operation_kind TEXT NOT NULL,
        idempotency_key TEXT NOT NULL,
        fingerprint TEXT NOT NULL,
        command_id TEXT,
        method_id TEXT,
        response_json TEXT NOT NULL,
        created_at REAL NOT NULL,
        PRIMARY KEY(operation_kind, idempotency_key)
    ) WITHOUT ROWID;
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_methods (
        method_id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        source_json TEXT NOT NULL,
        digest TEXT NOT NULL,
        failure_policy TEXT NOT NULL,
        status TEXT NOT NULL,
        version INTEGER NOT NULL,
        ownership_generation INTEGER NOT NULL,
        expanded_count INTEGER NOT NULL,
        first_stream_sequence INTEGER,
        last_stream_sequence INTEGER,
        queued_at REAL NOT NULL,
        updated_at REAL NOT NULL,
        recovery_outcome_pending INTEGER NOT NULL DEFAULT 0
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_commands (
        command_id TEXT PRIMARY KEY,
        stream_sequence INTEGER NOT NULL UNIQUE,
        method_id TEXT,
        method_sequence INTEGER,
        action_id TEXT NOT NULL,
        requested_json TEXT NOT NULL,
        effective_json TEXT NOT NULL,
        status TEXT NOT NULL,
        version INTEGER NOT NULL,
        ownership_generation INTEGER NOT NULL,
        dispatch_attempt_id TEXT,
        dispatcher_epoch INTEGER,
        dispatch_global_safety_epoch INTEGER,
        dispatch_axis_safety_epoch INTEGER,
        interrupt_id TEXT,
        interrupt_global_safety_epoch INTEGER,
        interrupt_axis_safety_epoch INTEGER,
        source_noop INTEGER NOT NULL DEFAULT 0,
        source_noop_reason TEXT,
        controller_acknowledged INTEGER NOT NULL DEFAULT 0,
        remote_acknowledged INTEGER NOT NULL DEFAULT 0,
        physical_effect_verified INTEGER NOT NULL DEFAULT 0,
        terminal_json TEXT,
        queued_at REAL NOT NULL,
        dispatched_at REAL,
        finished_at REAL,
        updated_at REAL NOT NULL,
        FOREIGN KEY(method_id) REFERENCES operator_plane_methods(method_id)
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_transitions (
        transition_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
        event_kind TEXT NOT NULL,
        command_id TEXT,
        method_id TEXT,
        state TEXT NOT NULL,
        payload_json TEXT NOT NULL,
        created_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_lane (
        singleton INTEGER PRIMARY KEY CHECK(singleton=1),
        active_command_id TEXT,
        active_attempt_id TEXT,
        dispatcher_epoch INTEGER NOT NULL,
        owner_id TEXT,
        owner_lease_until REAL,
        updated_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_safety (
        singleton INTEGER PRIMARY KEY CHECK(singleton=1),
        global_epoch INTEGER NOT NULL,
        x_epoch INTEGER NOT NULL,
        y_epoch INTEGER NOT NULL DEFAULT 0,
        z_epoch INTEGER NOT NULL,
        recovery_epoch INTEGER NOT NULL,
        recovery_version INTEGER NOT NULL,
        recovery_hold INTEGER NOT NULL,
        updated_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_z_home_authority (
        singleton INTEGER PRIMARY KEY CHECK(singleton=1),
        state TEXT NOT NULL CHECK(state IN ('invalid','valid')),
        command_id TEXT,
        ownership_generation INTEGER NOT NULL DEFAULT 0 CHECK(ownership_generation>=0),
        board_lifecycle_generation INTEGER,
        authority_version INTEGER NOT NULL DEFAULT 0 CHECK(authority_version>=0),
        invalidation_reason TEXT,
        updated_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_board_authority (
        board_id INTEGER PRIMARY KEY CHECK(board_id=5),
        state TEXT NOT NULL CHECK(state IN ('active','faulted')),
        active_board_epoch INTEGER,
        state_version INTEGER NOT NULL DEFAULT 1,
        updated_at REAL NOT NULL
    ) WITHOUT ROWID;
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_snapshots (
        token TEXT PRIMARY KEY,
        method_id TEXT NOT NULL,
        watermark INTEGER NOT NULL,
        expires_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_outbox (
        outbox_id TEXT PRIMARY KEY,
        command_id TEXT NOT NULL,
        transition_sequence INTEGER NOT NULL,
        state TEXT NOT NULL,
        payload_json TEXT,
        attempts INTEGER NOT NULL DEFAULT 0,
        updated_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_interrupt_history (
        record_sha256 TEXT PRIMARY KEY,
        stream TEXT NOT NULL CHECK(stream IN ('x','y','z','aggregate')),
        interrupt_attempt_id TEXT NOT NULL,
        receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
        imported_at REAL NOT NULL,
        UNIQUE(stream,interrupt_attempt_id)
    ) WITHOUT ROWID;
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_command_versions (
        version_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
        command_id TEXT NOT NULL,
        source_sequence INTEGER NOT NULL,
        row_json TEXT NOT NULL CHECK(json_valid(row_json)),
        deleted INTEGER NOT NULL DEFAULT 0 CHECK(deleted IN (0,1)),
        versioned_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_pipette_versions (
        version_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
        pipette_operation_id TEXT NOT NULL,
        command_id TEXT NOT NULL,
        source_rowid INTEGER NOT NULL,
        row_json TEXT NOT NULL CHECK(json_valid(row_json)),
        deleted INTEGER NOT NULL DEFAULT 0 CHECK(deleted IN (0,1)),
        versioned_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_pressure_stream_versions (
        version_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
        stream_session_id TEXT NOT NULL,
        source_rowid INTEGER NOT NULL,
        row_json TEXT NOT NULL CHECK(json_valid(row_json)),
        deleted INTEGER NOT NULL DEFAULT 0 CHECK(deleted IN (0,1)),
        versioned_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_evidence_versions (
        version_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
        evidence_artifact_id TEXT NOT NULL,
        source_rowid INTEGER NOT NULL,
        row_json TEXT NOT NULL CHECK(json_valid(row_json)),
        deleted INTEGER NOT NULL DEFAULT 0 CHECK(deleted IN (0,1)),
        versioned_at REAL NOT NULL
    );
    """,
    """
    CREATE TABLE IF NOT EXISTS operator_plane_pipette_query_attestations (
        pipette_operation_id TEXT NOT NULL,
        command_id TEXT NOT NULL,
        semantic_query_response_verified INTEGER NOT NULL CHECK(semantic_query_response_verified IN (0,1)),
        observed_at REAL NOT NULL,
        PRIMARY KEY(pipette_operation_id,semantic_query_response_verified),
        FOREIGN KEY(pipette_operation_id) REFERENCES pipette_operations(pipette_operation_id),
        FOREIGN KEY(command_id) REFERENCES operator_commands(command_id)
    ) WITHOUT ROWID;
    """,
)

_OPERATOR_COMMAND_PLANE_INDEX_DDL = (
    """
    CREATE INDEX IF NOT EXISTS operator_plane_commands_ready_idx
        ON operator_plane_commands(status, stream_sequence);
    """,
    """
    CREATE INDEX IF NOT EXISTS operator_plane_commands_method_idx
        ON operator_plane_commands(method_id, method_sequence);
    """,
    """
    CREATE INDEX IF NOT EXISTS operator_plane_command_versions_lookup_idx
        ON operator_plane_command_versions(command_id, version_sequence DESC);
    """,
    """
    CREATE INDEX IF NOT EXISTS operator_plane_pipette_versions_lookup_idx
        ON operator_plane_pipette_versions(pipette_operation_id, version_sequence DESC);
    """,
    """
    CREATE INDEX IF NOT EXISTS operator_plane_pressure_stream_versions_lookup_idx
        ON operator_plane_pressure_stream_versions(stream_session_id, version_sequence DESC);
    """,
    """
    CREATE INDEX IF NOT EXISTS operator_plane_evidence_versions_lookup_idx
        ON operator_plane_evidence_versions(evidence_artifact_id, version_sequence DESC);
    """,
)

_OPERATOR_COMMAND_PLANE_TRIGGER_DDL = (
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_no_delete
    BEFORE DELETE ON operator_plane_transitions
    BEGIN SELECT RAISE(ABORT, 'operator transitions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_no_update
    BEFORE UPDATE ON operator_plane_transitions
    BEGIN SELECT RAISE(ABORT, 'operator transitions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_commands_no_terminal_delete
    BEFORE DELETE ON operator_plane_commands
    WHEN OLD.status IN ('completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted')
    BEGIN SELECT RAISE(ABORT, 'terminal operator commands are immutable'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_methods_no_terminal_delete
    BEFORE DELETE ON operator_plane_methods
    WHEN OLD.status IN ('completed','failed','cancelled','stopped','aborted','interrupted')
    BEGIN SELECT RAISE(ABORT, 'terminal operator methods are immutable'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_command_versions_insert
    AFTER INSERT ON operator_commands
    BEGIN
        INSERT INTO operator_plane_command_versions(command_id,source_sequence,row_json,deleted,versioned_at)
        VALUES(NEW.command_id,NEW.sequence,json_object(
            'sequence',NEW.sequence,'command_id',NEW.command_id,'idempotency_key',NEW.idempotency_key,
            'operation',NEW.operation,'command_kind',NEW.command_kind,'entrypoint_id',NEW.entrypoint_id,
            'caller_class',NEW.caller_class,'control_class',NEW.control_class,'action_id',NEW.action_id,
            'status',NEW.status,'outcome',NEW.outcome,'failure_code',NEW.failure_code,
            'ownership_generation',NEW.ownership_generation,'connection_generation',NEW.connection_generation,
            'started_at',NEW.started_at,'admitted_at',NEW.admitted_at,'dispatched_at',NEW.dispatched_at,
            'finished_at',NEW.finished_at,'duration_ms',NEW.duration_ms,'delivery_verified',NEW.delivery_verified,
            'controller_acknowledged',NEW.controller_acknowledged,'completion_verified',NEW.completion_verified,
            'hardware_precondition_verified',NEW.hardware_precondition_verified,
            'hardware_postcondition_verified',NEW.hardware_postcondition_verified,
            'physical_effect_verified',NEW.physical_effect_verified,'evidence_state',NEW.evidence_state,
            'requested_inputs_json',NEW.requested_inputs_json,'effective_inputs_json',NEW.effective_inputs_json,
            'source_identity_json',NEW.source_identity_json,'updated_at',NEW.updated_at,
            'semantic_query_response_verified',COALESCE((SELECT MAX(semantic_query_response_verified) FROM operator_plane_pipette_query_attestations WHERE command_id=NEW.command_id),0)
        ),0,NEW.updated_at);
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_command_versions_update
    AFTER UPDATE ON operator_commands
    BEGIN
        INSERT INTO operator_plane_command_versions(command_id,source_sequence,row_json,deleted,versioned_at)
        VALUES(NEW.command_id,NEW.sequence,json_object(
            'sequence',NEW.sequence,'command_id',NEW.command_id,'idempotency_key',NEW.idempotency_key,
            'operation',NEW.operation,'command_kind',NEW.command_kind,'entrypoint_id',NEW.entrypoint_id,
            'caller_class',NEW.caller_class,'control_class',NEW.control_class,'action_id',NEW.action_id,
            'status',NEW.status,'outcome',NEW.outcome,'failure_code',NEW.failure_code,
            'ownership_generation',NEW.ownership_generation,'connection_generation',NEW.connection_generation,
            'started_at',NEW.started_at,'admitted_at',NEW.admitted_at,'dispatched_at',NEW.dispatched_at,
            'finished_at',NEW.finished_at,'duration_ms',NEW.duration_ms,'delivery_verified',NEW.delivery_verified,
            'controller_acknowledged',NEW.controller_acknowledged,'completion_verified',NEW.completion_verified,
            'hardware_precondition_verified',NEW.hardware_precondition_verified,
            'hardware_postcondition_verified',NEW.hardware_postcondition_verified,
            'physical_effect_verified',NEW.physical_effect_verified,'evidence_state',NEW.evidence_state,
            'requested_inputs_json',NEW.requested_inputs_json,'effective_inputs_json',NEW.effective_inputs_json,
            'source_identity_json',NEW.source_identity_json,'updated_at',NEW.updated_at,
            'semantic_query_response_verified',COALESCE((SELECT MAX(semantic_query_response_verified) FROM operator_plane_pipette_query_attestations WHERE command_id=NEW.command_id),0)
        ),0,NEW.updated_at);
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pipette_versions_insert
    AFTER INSERT ON pipette_operations
    BEGIN
        INSERT INTO operator_plane_pipette_versions(pipette_operation_id,command_id,source_rowid,row_json,deleted,versioned_at)
        VALUES(NEW.pipette_operation_id,NEW.command_id,NEW.rowid,json_object(
            'pipette_operation_id',NEW.pipette_operation_id,'command_id',NEW.command_id,'operation',NEW.operation,
            'entrypoint_id',NEW.entrypoint_id,'caller_class',NEW.caller_class,'control_class',NEW.control_class,
            'action_id',NEW.action_id,'status',NEW.status,'outcome',NEW.outcome,'failure_code',NEW.failure_code,
            'ownership_generation',NEW.ownership_generation,'connection_generation',NEW.connection_generation,
            'protocol_job_id',NEW.protocol_job_id,'protocol_action_id',NEW.protocol_action_id,
            'lifecycle_stage_id',NEW.lifecycle_stage_id,'lifecycle_attempt_id',NEW.lifecycle_attempt_id,
            'callback_session_id',NEW.callback_session_id,
            'delivery_verified',NEW.delivery_verified,'controller_acknowledged',NEW.controller_acknowledged,
            'completion_verified',NEW.completion_verified,'hardware_precondition_verified',NEW.hardware_precondition_verified,
            'hardware_postcondition_verified',NEW.hardware_postcondition_verified,'dispatched_at',NEW.dispatched_at,
            'finished_at',NEW.finished_at,
            'physical_effect_verified',NEW.physical_effect_verified,'evidence_state',NEW.evidence_state,
            'requested_inputs_json',NEW.requested_inputs_json,'effective_inputs_json',NEW.effective_inputs_json,
            'source_identity_json',NEW.source_identity_json,'updated_at',NEW.updated_at,
            'semantic_query_response_verified',COALESCE((SELECT MAX(semantic_query_response_verified) FROM operator_plane_pipette_query_attestations WHERE pipette_operation_id=NEW.pipette_operation_id),0)
        ),0,NEW.updated_at);
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pipette_versions_update
    AFTER UPDATE ON pipette_operations
    BEGIN
        INSERT INTO operator_plane_pipette_versions(pipette_operation_id,command_id,source_rowid,row_json,deleted,versioned_at)
        VALUES(NEW.pipette_operation_id,NEW.command_id,NEW.rowid,json_object(
            'pipette_operation_id',NEW.pipette_operation_id,'command_id',NEW.command_id,'operation',NEW.operation,
            'entrypoint_id',NEW.entrypoint_id,'caller_class',NEW.caller_class,'control_class',NEW.control_class,
            'action_id',NEW.action_id,'status',NEW.status,'outcome',NEW.outcome,'failure_code',NEW.failure_code,
            'ownership_generation',NEW.ownership_generation,'connection_generation',NEW.connection_generation,
            'protocol_job_id',NEW.protocol_job_id,'protocol_action_id',NEW.protocol_action_id,
            'lifecycle_stage_id',NEW.lifecycle_stage_id,'lifecycle_attempt_id',NEW.lifecycle_attempt_id,
            'callback_session_id',NEW.callback_session_id,
            'delivery_verified',NEW.delivery_verified,'controller_acknowledged',NEW.controller_acknowledged,
            'completion_verified',NEW.completion_verified,'hardware_precondition_verified',NEW.hardware_precondition_verified,
            'hardware_postcondition_verified',NEW.hardware_postcondition_verified,'dispatched_at',NEW.dispatched_at,
            'finished_at',NEW.finished_at,
            'physical_effect_verified',NEW.physical_effect_verified,'evidence_state',NEW.evidence_state,
            'requested_inputs_json',NEW.requested_inputs_json,'effective_inputs_json',NEW.effective_inputs_json,
            'source_identity_json',NEW.source_identity_json,'updated_at',NEW.updated_at,
            'semantic_query_response_verified',COALESCE((SELECT MAX(semantic_query_response_verified) FROM operator_plane_pipette_query_attestations WHERE pipette_operation_id=NEW.pipette_operation_id),0)
        ),0,NEW.updated_at);
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pressure_stream_versions_insert
    AFTER INSERT ON pipette_pressure_streams
    BEGIN
        INSERT INTO operator_plane_pressure_stream_versions(stream_session_id,source_rowid,row_json,deleted,versioned_at)
        VALUES(NEW.stream_session_id,NEW.rowid,json_object(
            'stream_session_id',NEW.stream_session_id,'command_id',NEW.command_id,
            'pipette_operation_id',NEW.pipette_operation_id,'channels_json',NEW.channels_json,
            'sample_period_ms',NEW.sample_period_ms,'started_at',NEW.started_at,'stopped_at',NEW.stopped_at,
            'source_generation',NEW.source_generation,'reader_generation',NEW.reader_generation,
            'offset_identity',NEW.offset_identity,'terminal_state',NEW.terminal_state,'loss_count',NEW.loss_count
        ),0,NEW.started_at);
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pressure_stream_versions_update
    AFTER UPDATE ON pipette_pressure_streams
    BEGIN
        INSERT INTO operator_plane_pressure_stream_versions(stream_session_id,source_rowid,row_json,deleted,versioned_at)
        VALUES(NEW.stream_session_id,NEW.rowid,json_object(
            'stream_session_id',NEW.stream_session_id,'command_id',NEW.command_id,
            'pipette_operation_id',NEW.pipette_operation_id,'channels_json',NEW.channels_json,
            'sample_period_ms',NEW.sample_period_ms,'started_at',NEW.started_at,'stopped_at',NEW.stopped_at,
            'source_generation',NEW.source_generation,'reader_generation',NEW.reader_generation,
            'offset_identity',NEW.offset_identity,'terminal_state',NEW.terminal_state,'loss_count',NEW.loss_count
        ),0,CAST(strftime('%s','now') AS REAL));
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_versions_insert
    AFTER INSERT ON runtime_evidence_objects
    BEGIN
        INSERT INTO operator_plane_evidence_versions(evidence_artifact_id,source_rowid,row_json,deleted,versioned_at)
        VALUES(NEW.evidence_artifact_id,NEW.rowid,json_object(
            'evidence_artifact_id',NEW.evidence_artifact_id,'command_id',NEW.command_id,
            'pipette_operation_id',NEW.pipette_operation_id,'original_relpath',NEW.original_relpath,
            'active_relpath',NEW.active_relpath,'sha256',NEW.sha256,'byte_count',NEW.byte_count,
            'created_at',NEW.created_at,'retention_deadline',NEW.retention_deadline,'legal_hold',NEW.legal_hold,
            'expiry_state',NEW.expiry_state,'expiry_receipt_id',NEW.expiry_receipt_id,'updated_at',NEW.updated_at
        ),0,NEW.updated_at);
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_versions_update
    AFTER UPDATE ON runtime_evidence_objects
    BEGIN
        INSERT INTO operator_plane_evidence_versions(evidence_artifact_id,source_rowid,row_json,deleted,versioned_at)
        VALUES(NEW.evidence_artifact_id,NEW.rowid,json_object(
            'evidence_artifact_id',NEW.evidence_artifact_id,'command_id',NEW.command_id,
            'pipette_operation_id',NEW.pipette_operation_id,'original_relpath',NEW.original_relpath,
            'active_relpath',NEW.active_relpath,'sha256',NEW.sha256,'byte_count',NEW.byte_count,
            'created_at',NEW.created_at,'retention_deadline',NEW.retention_deadline,'legal_hold',NEW.legal_hold,
            'expiry_state',NEW.expiry_state,'expiry_receipt_id',NEW.expiry_receipt_id,'updated_at',NEW.updated_at
        ),0,NEW.updated_at);
    END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pipette_query_attestations_no_update
    BEFORE UPDATE ON operator_plane_pipette_query_attestations
    BEGIN SELECT RAISE(ABORT, 'pipette semantic-query attestations are immutable'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pipette_query_attestations_no_delete
    BEFORE DELETE ON operator_plane_pipette_query_attestations
    BEGIN SELECT RAISE(ABORT, 'pipette semantic-query attestations are immutable'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_command_versions_no_update
    BEFORE UPDATE ON operator_plane_command_versions
    BEGIN SELECT RAISE(ABORT, 'operator command versions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_command_versions_no_delete
    BEFORE DELETE ON operator_plane_command_versions
    BEGIN SELECT RAISE(ABORT, 'operator command versions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pipette_versions_no_update
    BEFORE UPDATE ON operator_plane_pipette_versions
    BEGIN SELECT RAISE(ABORT, 'pipette operation versions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pipette_versions_no_delete
    BEFORE DELETE ON operator_plane_pipette_versions
    BEGIN SELECT RAISE(ABORT, 'pipette operation versions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_commands_history_source_no_delete
    BEFORE DELETE ON operator_commands
    BEGIN SELECT RAISE(ABORT, 'operator command history sources cannot be deleted'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS pipette_operations_history_source_no_delete
    BEFORE DELETE ON pipette_operations
    BEGIN SELECT RAISE(ABORT, 'pipette operation history sources cannot be deleted'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pressure_stream_versions_no_update
    BEFORE UPDATE ON operator_plane_pressure_stream_versions
    BEGIN SELECT RAISE(ABORT, 'pressure stream versions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_pressure_stream_versions_no_delete
    BEFORE DELETE ON operator_plane_pressure_stream_versions
    BEGIN SELECT RAISE(ABORT, 'pressure stream versions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_versions_no_update
    BEFORE UPDATE ON operator_plane_evidence_versions
    BEGIN SELECT RAISE(ABORT, 'evidence versions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_versions_no_delete
    BEFORE DELETE ON operator_plane_evidence_versions
    BEGIN SELECT RAISE(ABORT, 'evidence versions are append-only'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS pipette_pressure_streams_history_source_no_delete
    BEFORE DELETE ON pipette_pressure_streams
    BEGIN SELECT RAISE(ABORT, 'pressure stream history sources cannot be deleted'); END;
    """,
    """
    CREATE TRIGGER IF NOT EXISTS runtime_evidence_objects_history_source_no_delete
    BEFORE DELETE ON runtime_evidence_objects
    BEGIN SELECT RAISE(ABORT, 'evidence history sources cannot be deleted'); END;
    """,
)


def _apply_operator_command_plane_schema_v1(connection: sqlite3.Connection) -> None:
    for statement in (*_OPERATOR_COMMAND_PLANE_TABLE_DDL, *_OPERATOR_COMMAND_PLANE_INDEX_DDL):
        connection.execute(statement)

    command_fields = (
        "sequence", "command_id", "idempotency_key", "operation", "command_kind", "entrypoint_id",
        "caller_class", "control_class", "action_id", "status", "outcome", "failure_code",
        "ownership_generation", "connection_generation", "started_at", "admitted_at", "dispatched_at",
        "finished_at", "duration_ms", "delivery_verified", "controller_acknowledged", "completion_verified",
        "hardware_precondition_verified", "hardware_postcondition_verified", "physical_effect_verified",
        "evidence_state", "requested_inputs_json", "effective_inputs_json", "source_identity_json", "updated_at",
    )
    pipette_fields = (
        "pipette_operation_id", "command_id", "operation", "entrypoint_id", "caller_class", "control_class",
        "action_id", "status", "outcome", "failure_code", "ownership_generation", "connection_generation",
        "protocol_job_id", "protocol_action_id", "lifecycle_stage_id", "lifecycle_attempt_id", "callback_session_id",
        "delivery_verified", "controller_acknowledged", "completion_verified", "hardware_precondition_verified",
        "hardware_postcondition_verified", "physical_effect_verified", "evidence_state", "dispatched_at",
        "finished_at", "requested_inputs_json", "effective_inputs_json", "source_identity_json", "updated_at",
    )
    pressure_fields = (
        "stream_session_id", "command_id", "pipette_operation_id", "channels_json", "sample_period_ms",
        "started_at", "stopped_at", "source_generation", "reader_generation", "offset_identity",
        "terminal_state", "loss_count",
    )
    evidence_fields = (
        "evidence_artifact_id", "command_id", "pipette_operation_id", "original_relpath", "active_relpath",
        "sha256", "byte_count", "created_at", "retention_deadline", "legal_hold", "expiry_state",
        "expiry_receipt_id", "updated_at",
    )

    command_cursor = connection.execute(
        f"SELECT {','.join(command_fields)} FROM operator_commands ORDER BY sequence,command_id"
    )
    for values in command_cursor.fetchall():
        row = {str(description[0]): values[index] for index, description in enumerate(command_cursor.description)}
        if connection.execute(
            "SELECT 1 FROM operator_plane_command_versions WHERE command_id=? LIMIT 1",
            (str(row["command_id"]),),
        ).fetchone() is not None:
            continue
        semantic = connection.execute(
            "SELECT COALESCE(MAX(semantic_query_response_verified),0) FROM operator_plane_pipette_query_attestations WHERE command_id=?",
            (str(row["command_id"]),),
        ).fetchone()[0]
        payload = {field: row[field] for field in command_fields}
        payload["semantic_query_response_verified"] = int(semantic)
        connection.execute(
            "INSERT INTO operator_plane_command_versions(command_id,source_sequence,row_json,deleted,versioned_at) VALUES(?,?,?,?,?)",
            (
                str(row["command_id"]), int(row["sequence"]),
                json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False),
                0, float(row["updated_at"]),
            ),
        )

    pipette_cursor = connection.execute(
        f"SELECT rowid AS source_rowid,{','.join(pipette_fields)} FROM pipette_operations ORDER BY pipette_operation_id,command_id"
    )
    for values in pipette_cursor.fetchall():
        row = {str(description[0]): values[index] for index, description in enumerate(pipette_cursor.description)}
        if connection.execute(
            "SELECT 1 FROM operator_plane_pipette_versions WHERE pipette_operation_id=? LIMIT 1",
            (str(row["pipette_operation_id"]),),
        ).fetchone() is not None:
            continue
        semantic = connection.execute(
            "SELECT COALESCE(MAX(semantic_query_response_verified),0) FROM operator_plane_pipette_query_attestations WHERE pipette_operation_id=?",
            (str(row["pipette_operation_id"]),),
        ).fetchone()[0]
        payload = {field: row[field] for field in pipette_fields}
        payload["semantic_query_response_verified"] = int(semantic)
        connection.execute(
            "INSERT INTO operator_plane_pipette_versions(pipette_operation_id,command_id,source_rowid,row_json,deleted,versioned_at) VALUES(?,?,?,?,?,?)",
            (
                str(row["pipette_operation_id"]), str(row["command_id"]), int(row["source_rowid"]),
                json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False),
                0, float(row["updated_at"]),
            ),
        )

    pressure_cursor = connection.execute(
        f"SELECT rowid AS source_rowid,{','.join(pressure_fields)} FROM pipette_pressure_streams ORDER BY stream_session_id"
    )
    for values in pressure_cursor.fetchall():
        row = {str(description[0]): values[index] for index, description in enumerate(pressure_cursor.description)}
        if connection.execute(
            "SELECT 1 FROM operator_plane_pressure_stream_versions WHERE stream_session_id=? LIMIT 1",
            (str(row["stream_session_id"]),),
        ).fetchone() is not None:
            continue
        payload = {field: row[field] for field in pressure_fields}
        connection.execute(
            "INSERT INTO operator_plane_pressure_stream_versions(stream_session_id,source_rowid,row_json,deleted,versioned_at) VALUES(?,?,?,?,?)",
            (
                str(row["stream_session_id"]), int(row["source_rowid"]),
                json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False), 0,
                float(row["stopped_at"] if row["stopped_at"] is not None else row["started_at"]),
            ),
        )

    evidence_cursor = connection.execute(
        f"SELECT rowid AS source_rowid,{','.join(evidence_fields)} FROM runtime_evidence_objects ORDER BY evidence_artifact_id"
    )
    for values in evidence_cursor.fetchall():
        row = {str(description[0]): values[index] for index, description in enumerate(evidence_cursor.description)}
        if connection.execute(
            "SELECT 1 FROM operator_plane_evidence_versions WHERE evidence_artifact_id=? LIMIT 1",
            (str(row["evidence_artifact_id"]),),
        ).fetchone() is not None:
            continue
        payload = {field: row[field] for field in evidence_fields}
        connection.execute(
            "INSERT INTO operator_plane_evidence_versions(evidence_artifact_id,source_rowid,row_json,deleted,versioned_at) VALUES(?,?,?,?,?)",
            (
                str(row["evidence_artifact_id"]), int(row["source_rowid"]),
                json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False),
                0, float(row["updated_at"]),
            ),
        )

    for statement in _OPERATOR_COMMAND_PLANE_TRIGGER_DDL:
        connection.execute(statement)
    now = time.time()
    connection.execute(
        "INSERT OR IGNORE INTO operator_plane_lane(singleton,dispatcher_epoch,updated_at) VALUES(1,1,?)",
        (now,),
    )
    connection.execute(
        "INSERT OR IGNORE INTO operator_plane_safety(singleton,global_epoch,x_epoch,y_epoch,z_epoch,recovery_epoch,recovery_version,recovery_hold,updated_at) VALUES(1,0,0,0,0,0,1,0,?)",
        (now,),
    )
    connection.execute(
        "INSERT OR IGNORE INTO operator_plane_z_home_authority(singleton,state,updated_at) VALUES(1,'invalid',?)",
        (now,),
    )
    connection.execute(
        "INSERT OR IGNORE INTO operator_plane_board_authority(board_id,state,active_board_epoch,updated_at) VALUES(5,'faulted',NULL,?)",
        (now,),
    )


def operator_command_plane_migration_identity() -> RuntimeMigrationIdentity:
    ddl_source = "\n".join(
        (
            *_OPERATOR_COMMAND_PLANE_TABLE_DDL,
            *_OPERATOR_COMMAND_PLANE_INDEX_DDL,
            *_OPERATOR_COMMAND_PLANE_TRIGGER_DDL,
            inspect.getsource(_apply_operator_command_plane_schema_v1),
        )
    ).encode("utf-8")
    return RuntimeMigrationIdentity(
        version=OPERATOR_COMMAND_PLANE_SCHEMA_VERSION,
        name="operator_command_plane_schema_v1",
        ddl_sha256=hashlib.sha256(ddl_source).hexdigest(),
    )


def normalize_sql_definition(value: str | None) -> str:
    """Normalize SQLite syntax while preserving every quoted literal byte."""
    text = "" if value is None else str(value).strip()
    if text.endswith(";"):
        text = text[:-1]
    output: list[str] = []
    quote: str | None = None
    index = 0
    while index < len(text):
        character = text[index]
        if quote is not None:
            output.append(character)
            if quote == "]":
                if character == "]":
                    quote = None
            elif character == quote:
                if index + 1 < len(text) and text[index + 1] == quote:
                    index += 1
                    output.append(text[index])
                else:
                    quote = None
        elif character in {"'", '"', "`"}:
            quote = character
            output.append(character)
        elif character == "[":
            quote = "]"
            output.append(character)
        elif not character.isspace():
            output.append(character.upper())
        index += 1
    return "".join(output)


_OPERATOR_COMMAND_PLANE_TABLE_NAMES = (
    "operator_plane_metadata",
    "operator_plane_idempotency",
    "operator_plane_methods",
    "operator_plane_commands",
    "operator_plane_transitions",
    "operator_plane_lane",
    "operator_plane_safety",
    "operator_plane_z_home_authority",
    "operator_plane_board_authority",
    "operator_plane_snapshots",
    "operator_plane_outbox",
    "operator_plane_interrupt_history",
    "operator_plane_command_versions",
    "operator_plane_pipette_versions",
    "operator_plane_pressure_stream_versions",
    "operator_plane_evidence_versions",
    "operator_plane_pipette_query_attestations",
)
_OPERATOR_COMMAND_PLANE_INDEX_NAMES = (
    "operator_plane_commands_ready_idx",
    "operator_plane_commands_method_idx",
    "operator_plane_command_versions_lookup_idx",
    "operator_plane_pipette_versions_lookup_idx",
    "operator_plane_pressure_stream_versions_lookup_idx",
    "operator_plane_evidence_versions_lookup_idx",
)
_OPERATOR_COMMAND_PLANE_TRIGGER_NAMES = (
    "operator_plane_transitions_no_delete",
    "operator_plane_transitions_no_update",
    "operator_plane_commands_no_terminal_delete",
    "operator_plane_methods_no_terminal_delete",
    "operator_plane_command_versions_insert",
    "operator_plane_command_versions_update",
    "operator_plane_pipette_versions_insert",
    "operator_plane_pipette_versions_update",
    "operator_plane_pressure_stream_versions_insert",
    "operator_plane_pressure_stream_versions_update",
    "operator_plane_evidence_versions_insert",
    "operator_plane_evidence_versions_update",
    "operator_plane_pipette_query_attestations_no_update",
    "operator_plane_pipette_query_attestations_no_delete",
    "operator_plane_command_versions_no_update",
    "operator_plane_command_versions_no_delete",
    "operator_plane_pipette_versions_no_update",
    "operator_plane_pipette_versions_no_delete",
    "operator_commands_history_source_no_delete",
    "pipette_operations_history_source_no_delete",
    "operator_plane_pressure_stream_versions_no_update",
    "operator_plane_pressure_stream_versions_no_delete",
    "operator_plane_evidence_versions_no_update",
    "operator_plane_evidence_versions_no_delete",
    "pipette_pressure_streams_history_source_no_delete",
    "runtime_evidence_objects_history_source_no_delete",
)
_OPERATOR_COMMAND_PLANE_EXPECTED_COLUMNS = {
    "operator_plane_metadata": (
        ("key", "TEXT", 1, 1), ("value", "TEXT", 1, 0), ("updated_at", "REAL", 1, 0),
    ),
    "operator_plane_idempotency": (
        ("operation_kind", "TEXT", 1, 1), ("idempotency_key", "TEXT", 1, 2),
        ("fingerprint", "TEXT", 1, 0), ("command_id", "TEXT", 0, 0),
        ("method_id", "TEXT", 0, 0), ("response_json", "TEXT", 1, 0),
        ("created_at", "REAL", 1, 0),
    ),
    "operator_plane_methods": (
        ("method_id", "TEXT", 0, 1), ("name", "TEXT", 1, 0),
        ("source_json", "TEXT", 1, 0), ("digest", "TEXT", 1, 0),
        ("failure_policy", "TEXT", 1, 0), ("status", "TEXT", 1, 0),
        ("version", "INTEGER", 1, 0), ("ownership_generation", "INTEGER", 1, 0),
        ("expanded_count", "INTEGER", 1, 0), ("first_stream_sequence", "INTEGER", 0, 0),
        ("last_stream_sequence", "INTEGER", 0, 0), ("queued_at", "REAL", 1, 0),
        ("updated_at", "REAL", 1, 0), ("recovery_outcome_pending", "INTEGER", 1, 0),
    ),
    "operator_plane_commands": (
        ("command_id", "TEXT", 0, 1), ("stream_sequence", "INTEGER", 1, 0),
        ("method_id", "TEXT", 0, 0), ("method_sequence", "INTEGER", 0, 0),
        ("action_id", "TEXT", 1, 0), ("requested_json", "TEXT", 1, 0),
        ("effective_json", "TEXT", 1, 0), ("status", "TEXT", 1, 0),
        ("version", "INTEGER", 1, 0), ("ownership_generation", "INTEGER", 1, 0),
        ("dispatch_attempt_id", "TEXT", 0, 0), ("dispatcher_epoch", "INTEGER", 0, 0),
        ("dispatch_global_safety_epoch", "INTEGER", 0, 0),
        ("dispatch_axis_safety_epoch", "INTEGER", 0, 0), ("interrupt_id", "TEXT", 0, 0),
        ("interrupt_global_safety_epoch", "INTEGER", 0, 0),
        ("interrupt_axis_safety_epoch", "INTEGER", 0, 0), ("source_noop", "INTEGER", 1, 0),
        ("source_noop_reason", "TEXT", 0, 0), ("controller_acknowledged", "INTEGER", 1, 0),
        ("remote_acknowledged", "INTEGER", 1, 0), ("physical_effect_verified", "INTEGER", 1, 0),
        ("terminal_json", "TEXT", 0, 0), ("queued_at", "REAL", 1, 0),
        ("dispatched_at", "REAL", 0, 0), ("finished_at", "REAL", 0, 0),
        ("updated_at", "REAL", 1, 0),
    ),
    "operator_plane_transitions": (
        ("transition_sequence", "INTEGER", 0, 1), ("event_kind", "TEXT", 1, 0),
        ("command_id", "TEXT", 0, 0), ("method_id", "TEXT", 0, 0),
        ("state", "TEXT", 1, 0), ("payload_json", "TEXT", 1, 0),
        ("created_at", "REAL", 1, 0),
    ),
    "operator_plane_lane": (
        ("singleton", "INTEGER", 0, 1), ("active_command_id", "TEXT", 0, 0),
        ("active_attempt_id", "TEXT", 0, 0), ("dispatcher_epoch", "INTEGER", 1, 0),
        ("owner_id", "TEXT", 0, 0), ("owner_lease_until", "REAL", 0, 0),
        ("updated_at", "REAL", 1, 0),
    ),
    "operator_plane_safety": (
        ("singleton", "INTEGER", 0, 1), ("global_epoch", "INTEGER", 1, 0),
        ("x_epoch", "INTEGER", 1, 0), ("y_epoch", "INTEGER", 1, 0),
        ("z_epoch", "INTEGER", 1, 0), ("recovery_epoch", "INTEGER", 1, 0),
        ("recovery_version", "INTEGER", 1, 0), ("recovery_hold", "INTEGER", 1, 0),
        ("updated_at", "REAL", 1, 0),
    ),
    "operator_plane_z_home_authority": (
        ("singleton", "INTEGER", 0, 1), ("state", "TEXT", 1, 0),
        ("command_id", "TEXT", 0, 0), ("ownership_generation", "INTEGER", 1, 0),
        ("board_lifecycle_generation", "INTEGER", 0, 0), ("authority_version", "INTEGER", 1, 0),
        ("invalidation_reason", "TEXT", 0, 0), ("updated_at", "REAL", 1, 0),
    ),
    "operator_plane_board_authority": (
        ("board_id", "INTEGER", 1, 1), ("state", "TEXT", 1, 0),
        ("active_board_epoch", "INTEGER", 0, 0), ("state_version", "INTEGER", 1, 0),
        ("updated_at", "REAL", 1, 0),
    ),
    "operator_plane_snapshots": (
        ("token", "TEXT", 0, 1), ("method_id", "TEXT", 1, 0),
        ("watermark", "INTEGER", 1, 0), ("expires_at", "REAL", 1, 0),
    ),
    "operator_plane_outbox": (
        ("outbox_id", "TEXT", 0, 1), ("command_id", "TEXT", 1, 0),
        ("transition_sequence", "INTEGER", 1, 0), ("state", "TEXT", 1, 0),
        ("payload_json", "TEXT", 0, 0), ("attempts", "INTEGER", 1, 0),
        ("updated_at", "REAL", 1, 0),
    ),
    "operator_plane_interrupt_history": (
        ("record_sha256", "TEXT", 1, 1), ("stream", "TEXT", 1, 0),
        ("interrupt_attempt_id", "TEXT", 1, 0), ("receipt_json", "TEXT", 1, 0),
        ("imported_at", "REAL", 1, 0),
    ),
    "operator_plane_command_versions": (
        ("version_sequence", "INTEGER", 0, 1), ("command_id", "TEXT", 1, 0),
        ("source_sequence", "INTEGER", 1, 0), ("row_json", "TEXT", 1, 0),
        ("deleted", "INTEGER", 1, 0), ("versioned_at", "REAL", 1, 0),
    ),
    "operator_plane_pipette_versions": (
        ("version_sequence", "INTEGER", 0, 1), ("pipette_operation_id", "TEXT", 1, 0),
        ("command_id", "TEXT", 1, 0), ("source_rowid", "INTEGER", 1, 0),
        ("row_json", "TEXT", 1, 0),
        ("deleted", "INTEGER", 1, 0), ("versioned_at", "REAL", 1, 0),
    ),
    "operator_plane_pressure_stream_versions": (
        ("version_sequence", "INTEGER", 0, 1), ("stream_session_id", "TEXT", 1, 0),
        ("source_rowid", "INTEGER", 1, 0), ("row_json", "TEXT", 1, 0),
        ("deleted", "INTEGER", 1, 0), ("versioned_at", "REAL", 1, 0),
    ),
    "operator_plane_evidence_versions": (
        ("version_sequence", "INTEGER", 0, 1), ("evidence_artifact_id", "TEXT", 1, 0),
        ("source_rowid", "INTEGER", 1, 0), ("row_json", "TEXT", 1, 0),
        ("deleted", "INTEGER", 1, 0), ("versioned_at", "REAL", 1, 0),
    ),
    "operator_plane_pipette_query_attestations": (
        ("pipette_operation_id", "TEXT", 1, 1),
        ("command_id", "TEXT", 1, 0),
        ("semantic_query_response_verified", "INTEGER", 1, 2),
        ("observed_at", "REAL", 1, 0),
    ),
}

def _verify_operator_command_plane_schema_v1(connection: sqlite3.Connection) -> None:
    def stored_definition(source: str) -> str:
        return normalize_sql_definition(source).replace("IFNOTEXISTS", "", 1)

    expected_sql = {
        **{
            name: ("table", stored_definition(statement))
            for name, statement in zip(_OPERATOR_COMMAND_PLANE_TABLE_NAMES, _OPERATOR_COMMAND_PLANE_TABLE_DDL, strict=True)
        },
        **{
            name: ("index", stored_definition(statement))
            for name, statement in zip(_OPERATOR_COMMAND_PLANE_INDEX_NAMES, _OPERATOR_COMMAND_PLANE_INDEX_DDL, strict=True)
        },
        **{
            name: ("trigger", stored_definition(statement))
            for name, statement in zip(_OPERATOR_COMMAND_PLANE_TRIGGER_NAMES, _OPERATOR_COMMAND_PLANE_TRIGGER_DDL, strict=True)
        },
    }
    actual_rows = connection.execute(
        "SELECT type,name,sql FROM sqlite_master WHERE name IN (%s) ORDER BY type,name"
        % ",".join("?" for _ in expected_sql),
        tuple(sorted(expected_sql)),
    ).fetchall()
    actual_sql = {
        str(row[1]): (str(row[0]), normalize_sql_definition(row[2]))
        for row in actual_rows
    }
    if actual_sql != expected_sql:
        missing = sorted(set(expected_sql) - set(actual_sql))
        unexpected = sorted(set(actual_sql) - set(expected_sql))
        mismatched = sorted(name for name in set(actual_sql) & set(expected_sql) if actual_sql[name] != expected_sql[name])
        raise RuntimeError(
            f"operator command-plane schema object attestation failed: missing={missing},unexpected={unexpected},mismatched={mismatched}"
        )
    for table, expected in _OPERATOR_COMMAND_PLANE_EXPECTED_COLUMNS.items():
        actual = tuple(
            (str(row[1]), str(row[2]).upper(), int(row[3]), int(row[5]))
            for row in connection.execute(f"PRAGMA table_info({table})")
        )
        if actual != expected:
            raise RuntimeError(f"operator command-plane column attestation failed: {table}")
    expected_index_columns = {
        "operator_plane_commands_ready_idx": ("status", "stream_sequence"),
        "operator_plane_commands_method_idx": ("method_id", "method_sequence"),
    }
    for name, expected in expected_index_columns.items():
        actual = tuple(str(row[2]) for row in connection.execute(f"PRAGMA index_info({name})"))
        if actual != expected:
            raise RuntimeError(f"operator command-plane index-column attestation failed: {name}")
    expected_foreign_keys: dict[str, tuple[tuple[str, str, str, str, str, str], ...]] = {
        table: () for table in _OPERATOR_COMMAND_PLANE_TABLE_NAMES
    }
    expected_foreign_keys["operator_plane_commands"] = (
        ("operator_plane_methods", "method_id", "method_id", "NO ACTION", "NO ACTION", "NONE"),
    )
    expected_foreign_keys["operator_plane_pipette_query_attestations"] = (
        ("operator_commands", "command_id", "command_id", "NO ACTION", "NO ACTION", "NONE"),
        ("pipette_operations", "pipette_operation_id", "pipette_operation_id", "NO ACTION", "NO ACTION", "NONE"),
    )
    for table, expected in expected_foreign_keys.items():
        actual = tuple(
            (str(row[2]), str(row[3]), str(row[4]), str(row[5]), str(row[6]), str(row[7]))
            for row in connection.execute(f"PRAGMA foreign_key_list({table})")
        )
        if actual != expected:
            raise RuntimeError(f"operator command-plane foreign-key attestation failed: {table}")
    singleton_seeds = {
        "operator_plane_lane": ("singleton", 1),
        "operator_plane_safety": ("singleton", 1),
        "operator_plane_z_home_authority": ("singleton", 1),
        "operator_plane_board_authority": ("board_id", 5),
    }
    for table, (column, expected_value) in singleton_seeds.items():
        actual = tuple(int(row[0]) for row in connection.execute(f"SELECT {column} FROM {table} ORDER BY {column}"))
        if actual != (expected_value,):
            raise RuntimeError(f"operator command-plane singleton seed attestation failed: {table}")
    violations = connection.execute("PRAGMA foreign_key_check").fetchall()
    if violations:
        raise RuntimeError(f"operator command-plane migration left foreign-key violations: {violations}")
    missing_command_history = connection.execute(
        "SELECT c.command_id FROM operator_commands c LEFT JOIN operator_plane_command_versions v ON v.command_id=c.command_id WHERE v.command_id IS NULL LIMIT 1"
    ).fetchone()
    if missing_command_history is not None:
        raise RuntimeError("operator command history backfill is incomplete")
    missing_pipette_history = connection.execute(
        "SELECT p.pipette_operation_id FROM pipette_operations p LEFT JOIN operator_plane_pipette_versions v ON v.pipette_operation_id=p.pipette_operation_id WHERE v.pipette_operation_id IS NULL LIMIT 1"
    ).fetchone()
    if missing_pipette_history is not None:
        raise RuntimeError("pipette operation history backfill is incomplete")
    missing_pressure_history = connection.execute(
        "SELECT s.stream_session_id FROM pipette_pressure_streams s LEFT JOIN operator_plane_pressure_stream_versions v ON v.stream_session_id=s.stream_session_id WHERE v.stream_session_id IS NULL LIMIT 1"
    ).fetchone()
    if missing_pressure_history is not None:
        raise RuntimeError("pressure stream history backfill is incomplete")
    missing_evidence_history = connection.execute(
        "SELECT e.evidence_artifact_id FROM runtime_evidence_objects e LEFT JOIN operator_plane_evidence_versions v ON v.evidence_artifact_id=e.evidence_artifact_id WHERE v.evidence_artifact_id IS NULL LIMIT 1"
    ).fetchone()
    if missing_evidence_history is not None:
        raise RuntimeError("evidence history backfill is incomplete")


def _verify_runtime_release_start(connection: sqlite3.Connection) -> None:
    def stored_definition(source: str) -> str:
        return normalize_sql_definition(source).replace("IFNOTEXISTS", "", 1)

    table = connection.execute(
        "SELECT sql FROM sqlite_master WHERE type='table' AND name='runtime_release_receipts'"
    ).fetchone()
    if table is None or normalize_sql_definition(table[0]) != stored_definition(RUNTIME_RELEASE_RECEIPTS_DDL):
        raise RuntimeError("canonical runtime release receipt table attestation failed")
    expected_columns = (
        ("receipt_id", "TEXT", 0, 1),
        ("release_id", "TEXT", 1, 0),
        ("deployment_receipt_id", "TEXT", 1, 0),
        ("systemd_invocation_id", "TEXT", 1, 0),
        ("application_pid", "INTEGER", 1, 0),
        ("application_cgroup", "TEXT", 1, 0),
        ("application_cgroup_sha256", "TEXT", 1, 0),
        ("application_start_time_ticks", "INTEGER", 1, 0),
        ("application_started_at", "REAL", 1, 0),
        ("canonical_receipt_sha256", "TEXT", 1, 0),
        ("source_manifest_sha256", "TEXT", 1, 0),
        ("source_aggregate_sha256", "TEXT", 1, 0),
        ("image_id", "TEXT", 1, 0),
        ("image_inspection_receipt_sha256", "TEXT", 1, 0),
        ("udocker_path", "TEXT", 1, 0),
        ("udocker_sha256", "TEXT", 1, 0),
        ("udocker_tree_sha256", "TEXT", 1, 0),
        ("unit_sha256", "TEXT", 1, 0),
        ("launcher_sha256", "TEXT", 1, 0),
        ("configuration_sha256", "TEXT", 1, 0),
        ("oem_lock_sha256", "TEXT", 1, 0),
        ("declared_listener_json", "TEXT", 1, 0),
        ("observed_listener_json", "TEXT", 1, 0),
        ("receipt_json", "TEXT", 1, 0),
        ("receipt_sha256", "TEXT", 1, 0),
        ("recorded_at", "REAL", 1, 0),
    )
    actual_columns = tuple(
        (str(row[1]), str(row[2]).upper(), int(row[3]), int(row[5]))
        for row in connection.execute("PRAGMA table_info(runtime_release_receipts)")
    )
    if actual_columns != expected_columns:
        raise RuntimeError("canonical runtime release receipt column attestation failed")
    index = connection.execute(
        "SELECT sql FROM sqlite_master WHERE type='index' AND name='runtime_release_receipts_time_idx'"
    ).fetchone()
    if index is None or normalize_sql_definition(index[0]) != stored_definition(RUNTIME_RELEASE_RECEIPTS_INDEX_DDL):
        raise RuntimeError("canonical runtime release receipt index attestation failed")
    index_columns = tuple(
        str(row[2]) for row in connection.execute("PRAGMA index_info(runtime_release_receipts_time_idx)")
    )
    if index_columns != ("recorded_at", "receipt_id"):
        raise RuntimeError("canonical runtime release receipt index shape mismatch")
    expected_triggers = {
        "runtime_release_receipts_append_only_update": stored_definition(RUNTIME_RELEASE_RECEIPTS_TRIGGER_DDL[0]),
        "runtime_release_receipts_append_only_delete": stored_definition(RUNTIME_RELEASE_RECEIPTS_TRIGGER_DDL[1]),
    }
    actual_triggers = {
        str(row[0]): normalize_sql_definition(row[1])
        for row in connection.execute(
            "SELECT name,sql FROM sqlite_master WHERE type='trigger' AND name IN (?,?)",
            tuple(sorted(expected_triggers)),
        ).fetchall()
    }
    if actual_triggers != expected_triggers:
        raise RuntimeError("canonical runtime release append-only trigger attestation failed")

def canonical_runtime_migration_registry() -> tuple[RuntimeMigrationIdentity, ...]:
    registry = (
        runtime_audit_migration_identity(),
        serial206_runtime_migration_identity(),
        report_identity_migration_identity(),
        runtime_release_migration_identity(),
        operator_command_plane_migration_identity(),
    )
    versions = tuple(item.version for item in registry)
    if versions != tuple(sorted(set(versions))):
        raise RuntimeError("runtime migration registry versions must be unique and monotonic")
    return registry


def _verify_exact_v2_objects(connection: sqlite3.Connection) -> None:
    expected = sqlite3.connect(":memory:")
    expected.row_factory = sqlite3.Row
    expected.execute("PRAGMA foreign_keys=ON")
    try:
        _create_v2_authority_schema(expected)
        expected_objects = expected.execute(
            """
            SELECT type,name,sql FROM sqlite_master
            WHERE type IN ('table','index') AND name NOT LIKE 'sqlite_%' AND sql IS NOT NULL
            ORDER BY type,name
            """
        ).fetchall()
        for expected_object in expected_objects:
            object_type = str(expected_object["type"])
            name = str(expected_object["name"])
            actual = connection.execute(
                "SELECT sql FROM sqlite_master WHERE type=? AND name=?",
                (object_type, name),
            ).fetchone()
            expected_sql = normalize_sql_definition(str(expected_object["sql"]))
            actual_sql = "" if actual is None else normalize_sql_definition(str(actual[0]))
            if actual_sql != expected_sql:
                raise RuntimeError(f"serial-206 exact schema object mismatch: {object_type}:{name}")
        tables = tuple(
            str(row["name"])
            for row in expected_objects
            if str(row["type"]) == "table"
        )
        for table in tables:
            expected_columns = tuple(tuple(row) for row in expected.execute(f'PRAGMA table_info("{table}")'))
            actual_columns = tuple(tuple(row) for row in connection.execute(f'PRAGMA table_info("{table}")'))
            expected_fks = tuple(tuple(row) for row in expected.execute(f'PRAGMA foreign_key_list("{table}")'))
            actual_fks = tuple(tuple(row) for row in connection.execute(f'PRAGMA foreign_key_list("{table}")'))
            expected_indexes = tuple(tuple(row) for row in expected.execute(f'PRAGMA index_list("{table}")'))
            actual_indexes = tuple(tuple(row) for row in connection.execute(f'PRAGMA index_list("{table}")'))
            if actual_columns != expected_columns or actual_fks != expected_fks or actual_indexes != expected_indexes:
                raise RuntimeError(f"serial-206 exact table shape mismatch: {table}")
    finally:
        expected.close()


def _verify_v2_schema(connection: sqlite3.Connection) -> None:
    verify_runtime_audit_foundation(connection)
    _verify_exact_v2_objects(connection)
    required = {
        "runtime_schema_migrations",
        "runtime_retired_json_artifacts",
        "serial206_board_authority",
        "serial206_board_transitions",
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
    trigger_rows = {
        str(row[0]): str(row[1] or "")
        for row in connection.execute(
            "SELECT name,sql FROM sqlite_master WHERE type='trigger'"
        ).fetchall()
    }
    missing_triggers = sorted(set(_RUNTIME_AUTHORITY_TRIGGER_NAMES) - set(trigger_rows))
    if missing_triggers:
        raise RuntimeError(f"runtime authority triggers missing: {','.join(missing_triggers)}")
    exclusively_runtime_owned_tables = {
        "serial206_authority_snapshots",
        "runtime_state_snapshots",
        "runtime_journal",
        "runtime_movement_runs",
        "runtime_retired_json_artifacts",
        "serial206_board_authority",
        "serial206_axis_authority",
        "serial206_board_transitions",
    }
    expected_owned_triggers = {
        str(row[0])
        for row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type='trigger' AND name IN (%s) AND tbl_name IN (%s)"
            % (
                ",".join("?" for _ in _RUNTIME_AUTHORITY_TRIGGER_NAMES),
                ",".join("?" for _ in exclusively_runtime_owned_tables),
            ),
            tuple(_RUNTIME_AUTHORITY_TRIGGER_NAMES) + tuple(sorted(exclusively_runtime_owned_tables)),
        ).fetchall()
    }
    actual_owned_triggers = {
        str(row[0])
        for row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type='trigger' AND tbl_name IN (%s)"
            % ",".join("?" for _ in exclusively_runtime_owned_tables),
            tuple(sorted(exclusively_runtime_owned_tables)),
        ).fetchall()
    }
    if actual_owned_triggers != expected_owned_triggers:
        raise RuntimeError("runtime authority trigger set is not exact")
    coherence_sql = trigger_rows["serial206_authority_snapshots_coherence_v1"]
    if "canonical_json" not in coherence_sql or "sha256_utf8" not in coherence_sql:
        raise RuntimeError("runtime authority trigger definition mismatch")
    expected_columns = {
        "runtime_schema_migrations": (("version", "INTEGER", 1, 1), ("backup_sha256", "TEXT", 1, 0), ("source_json_digests_json", "TEXT", 1, 0), ("started_at", "REAL", 1, 0), ("finished_at", "REAL", 1, 0), ("result", "TEXT", 1, 0)),
        "runtime_retired_json_artifacts": (("source_name", "TEXT", 1, 1), ("content_sha256", "TEXT", 1, 0), ("content_blob", "BLOB", 1, 0), ("imported_at", "REAL", 1, 0)),
        "serial206_authority_snapshots": (("sequence", "INTEGER", 0, 1), ("state_json", "TEXT", 1, 0), ("state_sha256", "TEXT", 1, 0), ("receipt_set_json", "TEXT", 1, 0), ("receipt_set_sha256", "TEXT", 1, 0), ("created_at", "REAL", 1, 0)),
        "runtime_state_snapshots": (("sequence", "INTEGER", 0, 1), ("state_json", "TEXT", 1, 0), ("state_sha256", "TEXT", 1, 0), ("created_at", "REAL", 1, 0)),
        "runtime_journal": (("sequence", "INTEGER", 0, 1), ("stream", "TEXT", 1, 0), ("payload_json", "TEXT", 1, 0), ("payload_sha256", "TEXT", 1, 0), ("created_at", "REAL", 1, 0)),
        "runtime_movement_runs": (("run_id", "TEXT", 1, 1), ("sequence", "INTEGER", 1, 0), ("run_json", "TEXT", 1, 0), ("run_sha256", "TEXT", 1, 0), ("updated_at", "REAL", 1, 0)),
        "serial206_receipts": (("stream", "TEXT", 1, 1), ("receipt_id", "TEXT", 1, 2), ("command_id", "TEXT", 0, 0), ("idempotency_key", "TEXT", 0, 0), ("idempotency_replay_enabled", "INTEGER", 1, 0), ("status", "TEXT", 0, 0), ("observed_at", "REAL", 1, 0), ("receipt_json", "TEXT", 1, 0)),
        "serial206_board_authority": (("board_id", "INTEGER", 0, 1), ("state", "TEXT", 1, 0), ("prior_board_epoch", "INTEGER", 0, 0), ("active_board_epoch", "INTEGER", 0, 0), ("transition_id", "TEXT", 0, 0), ("deactivation_attempt_id", "TEXT", 0, 0), ("deactivation_delivery", "INTEGER", 0, 0), ("deactivation_reply_valid", "INTEGER", 0, 0), ("deactivation_status_code", "INTEGER", 0, 0), ("activation_attempt_id", "TEXT", 0, 0), ("activation_delivery", "INTEGER", 0, 0), ("activation_reply_valid", "INTEGER", 0, 0), ("activation_status_code", "INTEGER", 0, 0), ("member_motors_json", "TEXT", 1, 0), ("state_version", "INTEGER", 1, 0), ("updated_at", "REAL", 1, 0)),
        "serial206_board_transitions": (("sequence", "INTEGER", 0, 1), ("transition_id", "TEXT", 1, 0), ("requested_active", "INTEGER", 1, 0), ("ownership_generation", "INTEGER", 1, 0), ("delivery_attempted", "INTEGER", 1, 0), ("reply_valid", "INTEGER", 1, 0), ("status_code", "INTEGER", 1, 0), ("continuity_proven", "INTEGER", 1, 0), ("accepted", "INTEGER", 1, 0), ("state_before", "TEXT", 1, 0), ("state_after", "TEXT", 1, 0), ("prior_board_epoch", "INTEGER", 0, 0), ("active_board_epoch", "INTEGER", 0, 0), ("created_at", "REAL", 1, 0)),
        "serial206_axis_authority": (("axis", "TEXT", 1, 1), ("board_id", "INTEGER", 1, 0), ("motor_id", "INTEGER", 1, 0), ("ownership_generation", "INTEGER", 1, 0), ("prepared_board_epoch", "INTEGER", 0, 0), ("profile_fingerprint", "TEXT", 0, 0), ("lifecycle_state", "TEXT", 1, 0), ("reference_state", "TEXT", 1, 0), ("origin_position_steps", "INTEGER", 0, 0), ("observed_position_steps", "INTEGER", 0, 0), ("last_discrepancy_steps", "INTEGER", 0, 0), ("last_command_id", "TEXT", 0, 0), ("last_receipt_id", "TEXT", 0, 0), ("interrupt_epoch", "INTEGER", 1, 0), ("state_version", "INTEGER", 1, 0), ("updated_at", "REAL", 1, 0)),
        "serial206_movement_methods": (("method_id", "TEXT", 1, 1), ("idempotency_key", "TEXT", 1, 0), ("action_id", "TEXT", 1, 0), ("canonical_inputs_sha256", "TEXT", 1, 0), ("state", "TEXT", 1, 0), ("state_version", "INTEGER", 1, 0), ("failure_policy", "TEXT", 1, 0), ("child_count", "INTEGER", 1, 0), ("accepted_at", "REAL", 1, 0), ("started_at", "REAL", 0, 0), ("finished_at", "REAL", 0, 0)),
        "serial206_movement_commands": (("sequence", "INTEGER", 0, 1), ("command_id", "TEXT", 1, 0), ("idempotency_key", "TEXT", 1, 0), ("action_id", "TEXT", 1, 0), ("method_id", "TEXT", 0, 0), ("method_order", "INTEGER", 1, 0), ("parallel_group", "INTEGER", 1, 0), ("axis_scope", "TEXT", 0, 0), ("board_scope_json", "TEXT", 1, 0), ("ownership_generation", "INTEGER", 1, 0), ("expected_board_epochs_json", "TEXT", 1, 0), ("canonical_inputs_sha256", "TEXT", 1, 0), ("state", "TEXT", 1, 0), ("state_version", "INTEGER", 1, 0), ("admitted_interrupt_epochs_json", "TEXT", 1, 0), ("accepted_at", "REAL", 1, 0), ("queued_at", "REAL", 1, 0), ("dispatched_at", "REAL", 0, 0), ("finished_at", "REAL", 0, 0), ("terminal_receipt_id", "TEXT", 0, 0)),
        "serial206_command_resources": (("command_id", "TEXT", 1, 1), ("resource_key", "TEXT", 1, 2)),
        "serial206_command_dependencies": (("command_id", "TEXT", 1, 1), ("depends_on_command_id", "TEXT", 1, 2), ("required_terminal", "TEXT", 1, 0)),
        "serial206_interrupt_imports": (("record_sha256", "TEXT", 1, 1), ("interrupt_attempt_id", "TEXT", 1, 0), ("axis", "TEXT", 1, 0), ("interrupt_epoch", "INTEGER", 1, 0), ("imported_at", "REAL", 1, 0), ("receipt_id", "TEXT", 1, 0)),
    }
    for table, expected in expected_columns.items():
        actual = tuple((str(row[1]), str(row[2]).upper(), int(row[3]), int(row[5])) for row in connection.execute(f"PRAGMA table_info({table})"))
        if table == "runtime_schema_migrations":
            actual_by_name = {row[0]: (row[1], row[3]) for row in actual}
            if any(actual_by_name.get(row[0]) != (row[1], row[3]) for row in expected):
                raise RuntimeError(f"serial-206 v2 schema shape mismatch: {table}")
        elif actual != expected:
            raise RuntimeError(f"serial-206 v2 schema shape mismatch: {table}")
    index_shapes = {
        "serial206_movement_commands_idempotency_idx": (True, ("idempotency_key",)),
        "serial206_movement_commands_ready_idx": (False, ("state", "sequence")),
        "serial206_movement_commands_method_idx": (False, ("method_id", "method_order", "parallel_group", "sequence")),
        "serial206_command_resources_lookup_idx": (False, ("resource_key", "command_id")),
        "serial206_command_dependencies_reverse_idx": (False, ("depends_on_command_id", "command_id")),
    }
    for name, (unique, columns) in index_shapes.items():
        row = connection.execute("SELECT sql FROM sqlite_master WHERE type='index' AND name=?", (name,)).fetchone()
        actual_columns = tuple(str(item[2]) for item in connection.execute(f"PRAGMA index_info({name})"))
        actual_unique = "CREATE UNIQUE INDEX" in str(row[0]).upper() if row else False
        if row is None or actual_columns != columns or actual_unique != unique:
            raise RuntimeError(f"serial-206 v2 schema index shape mismatch: {name}")
    fk = connection.execute("PRAGMA foreign_key_list(serial206_axis_authority)").fetchall()
    axis_fk = {
        (str(row[3]), str(row[2]), str(row[4]), str(row[5]).upper(), str(row[6]).upper(), str(row[7]).upper())
        for row in fk
    }
    if axis_fk != {("board_id", "serial206_board_authority", "board_id", "NO ACTION", "NO ACTION", "NONE")}:
        raise RuntimeError("serial-206 v2 schema foreign-key shape mismatch")
    expected_fks = {
        "serial206_movement_commands": {
            ("method_id", "serial206_movement_methods", "method_id", "NO ACTION", "CASCADE", "NONE"),
        },
        "serial206_command_resources": {
            ("command_id", "serial206_movement_commands", "command_id", "NO ACTION", "CASCADE", "NONE"),
        },
        "serial206_command_dependencies": {
            ("command_id", "serial206_movement_commands", "command_id", "NO ACTION", "CASCADE", "NONE"),
            ("depends_on_command_id", "serial206_movement_commands", "command_id", "NO ACTION", "CASCADE", "NONE"),
        },
    }
    for table, expected in expected_fks.items():
        rows = connection.execute(f"PRAGMA foreign_key_list({table})").fetchall()
        actual = {
            (str(row[3]), str(row[2]), str(row[4]), str(row[5]).upper(), str(row[6]).upper(), str(row[7]).upper())
            for row in rows
        }
        if actual != expected:
            raise RuntimeError(f"serial-206 v2 schema foreign-key shape mismatch: {table}")
    constraint_fragments = {
        "serial206_board_authority": ("CHECK(BOARD_ID=4)", "JSON_VALID(MEMBER_MOTORS_JSON)"),
        "serial206_board_transitions": ("REQUESTED_ACTIVE IN (0,1)", "ACCEPTED IN (0,1)"),
        "serial206_axis_authority": ("AXIS IN ('y','z','gripper')", "WITHOUT ROWID"),
        "serial206_movement_methods": ("FAILURE_POLICY='require_completed'", "WITHOUT ROWID"),
        "serial206_movement_commands": ("JSON_VALID(EXPECTED_BOARD_EPOCHS_JSON)", "STATE IN ('queued','dispatched','issued_pending','interrupting','completed','failed','cleared','interrupted','ambiguous','rejected')"),
        "serial206_command_dependencies": ("CHECK(COMMAND_ID<>DEPENDS_ON_COMMAND_ID)", "WITHOUT ROWID"),
        "serial206_interrupt_imports": ("CHECK(LENGTH(RECORD_SHA256)=64)", "WITHOUT ROWID"),
    }
    for table, fragments in constraint_fragments.items():
        row = connection.execute("SELECT sql FROM sqlite_master WHERE type='table' AND name=?", (table,)).fetchone()
        normalized = normalize_sql_definition(str(row[0])) if row else ""
        if any(normalize_sql_definition(fragment) not in normalized for fragment in fragments):
            raise RuntimeError(f"serial-206 v2 schema constraint shape mismatch: {table}")
    physical_schema_sha256 = _runtime_physical_schema_sha256(connection)
    schema_version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    expected_physical_sha256 = _RUNTIME_PHYSICAL_SCHEMA_SHA256_BY_VERSION.get(schema_version)
    if expected_physical_sha256 is None:
        raise RuntimeError(f"runtime physical schema version is unsupported: {schema_version}")
    allowed_physical_sha256 = {expected_physical_sha256}
    if schema_version == SERIAL206_SCHEMA_VERSION:
        allowed_physical_sha256.add("c10b9517ff0134b44c0fcec240fdcfafc640d3c56634c1fb9a88eaea87995317")
    if physical_schema_sha256 not in allowed_physical_sha256:
        raise RuntimeError(f"runtime physical schema fingerprint mismatch: {physical_schema_sha256}")


def verify_runtime_database_v2(connection: sqlite3.Connection) -> None:
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    if version != SERIAL206_SCHEMA_VERSION:
        raise RuntimeError(f"runtime schema v2 is not prepared (found {version})")
    _verify_v2_schema(connection)


def _verify_report_identity_metadata_v1(connection: sqlite3.Connection) -> None:
    rows = {
        str(row["key"]): str(row["value"])
        for row in connection.execute(
            "SELECT key,value FROM runtime_metadata WHERE key IN (?,?)",
            ("database_incarnation_id", "report_cursor_hmac_key"),
        ).fetchall()
    }
    incarnation = rows.get("database_incarnation_id", "")
    cursor_key = rows.get("report_cursor_hmac_key", "")
    try:
        if str(uuid.UUID(incarnation)) != incarnation:
            raise ValueError("noncanonical UUID")
        key_bytes = bytes.fromhex(cursor_key)
    except ValueError as exc:
        raise RuntimeError("runtime report identity metadata is invalid") from exc
    if len(key_bytes) < 32 or cursor_key != key_bytes.hex():
        raise RuntimeError("runtime report cursor key is invalid")
    expected_triggers = {
        "runtime_metadata_report_identity_insert_shape": normalize_sql_definition(
            _REPORT_IDENTITY_TRIGGER_DDL[0]
        ),
        "runtime_metadata_report_identity_immutable_update": normalize_sql_definition(
            _REPORT_IDENTITY_TRIGGER_DDL[1]
        ),
        "runtime_metadata_report_identity_immutable_delete": normalize_sql_definition(
            _REPORT_IDENTITY_TRIGGER_DDL[2]
        ),
    }
    triggers = {
        str(row[0]): normalize_sql_definition(row[1])
        for row in connection.execute(
            "SELECT name,sql FROM sqlite_master WHERE type='trigger' AND name IN (?,?,?)",
            tuple(sorted(expected_triggers)),
        ).fetchall()
    }
    if triggers != expected_triggers:
        raise RuntimeError("runtime report identity trigger attestation failed")


def _manifest_statement(statement: str) -> tuple[tuple[str, str], str]:
    normalized = normalize_sql_definition(statement)
    match = re.match(
        r"^CREATE(?:UNIQUE)?(TABLE|INDEX|TRIGGER|VIEW)(?:IFNOTEXISTS)?([A-Z_][A-Z0-9_]*)",
        normalized,
    )
    if match is None:
        raise RuntimeError("canonical migration DDL contains an unclassifiable schema statement")
    return (match.group(1).lower(), match.group(2).lower()), normalized


def canonical_runtime_schema_manifest() -> dict[tuple[str, str], str]:
    """Return the exact union of every registered non-SQLite schema object."""
    expected = _expected_foundation_connection()
    try:
        expected.create_function("authority_write_allowed", 0, lambda: 1)
        expected.create_function(
            "sha256_utf8",
            1,
            lambda value: hashlib.sha256(str(value).encode("utf-8")).hexdigest(),
            deterministic=True,
        )
        expected.create_function(
            "sha256_blob",
            1,
            lambda value: hashlib.sha256(bytes(value)).hexdigest(),
            deterministic=True,
        )
        expected.create_function(
            "canonical_json",
            1,
            lambda value: json.dumps(
                json.loads(str(value)), sort_keys=True, separators=(",", ":"), allow_nan=False
            ),
            deterministic=True,
        )
        _create_v1_runtime_schema(expected)
        _create_v2_authority_schema(expected)
        for statement in _REPORT_IDENTITY_TRIGGER_DDL:
            expected.execute(statement)
        _apply_runtime_release_start(expected)
        _apply_operator_command_plane_schema_v1(expected)
        return {
            (str(row[0]), str(row[1])): normalize_sql_definition(row[2])
            for row in expected.execute(
                """
                SELECT type,name,sql FROM sqlite_master
                WHERE type IN ('table','index','trigger','view')
                  AND name NOT LIKE 'sqlite_%'
                  AND sql IS NOT NULL
                """
            ).fetchall()
        }
    finally:
        expected.close()


def verify_canonical_runtime_database(connection: sqlite3.Connection) -> None:
    registry = canonical_runtime_migration_registry()
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    if version != registry[-1].version:
        raise RuntimeError(f"canonical runtime schema is not prepared (found {version})")
    rows = connection.execute(
        "SELECT version,name,ddl_sha256 FROM runtime_schema_migrations ORDER BY version"
    ).fetchall()
    actual = tuple((int(row[0]), str(row[1]), str(row[2])) for row in rows)
    expected = tuple((item.version, item.name, item.ddl_sha256) for item in registry)
    if actual != expected:
        raise RuntimeError("runtime migration ledger is not the exact canonical ordered registry")
    _verify_v2_schema(connection)
    _verify_report_identity_metadata_v1(connection)
    _verify_runtime_release_start(connection)
    _verify_operator_command_plane_schema_v1(connection)
    expected_manifest = canonical_runtime_schema_manifest()
    actual_manifest = {
        (str(row[0]), str(row[1])): normalize_sql_definition(row[2])
        for row in connection.execute(
            """
            SELECT type,name,sql FROM sqlite_master
            WHERE type IN ('table','index','trigger','view')
              AND name NOT LIKE 'sqlite_%'
              AND sql IS NOT NULL
            """
        ).fetchall()
    }
    if actual_manifest != expected_manifest:
        missing = sorted(set(expected_manifest) - set(actual_manifest))
        unexpected = sorted(set(actual_manifest) - set(expected_manifest))
        mismatched = sorted(
            key for key in set(expected_manifest) & set(actual_manifest)
            if expected_manifest[key] != actual_manifest[key]
        )
        raise RuntimeError(
            f"canonical runtime schema manifest mismatch: missing={missing},unexpected={unexpected},mismatched={mismatched}"
        )
    identity = connection.execute(
        "SELECT schema_version FROM runtime_store_identity WHERE identity_id=1"
    ).fetchone()
    if identity is None or int(identity[0]) != OPERATOR_COMMAND_PLANE_SCHEMA_VERSION:
        raise RuntimeError("runtime store identity does not attest canonical schema v5")


def _verified_sqlite_backup(connection: sqlite3.Connection, root: Path) -> str:
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    with runtime_lifecycle_lock(root, exclusive=True):
        backup_path = root / f"bioxp_runtime.db.pre-v2.{time.time_ns()}.sqlite3"
        backup = sqlite3.connect(backup_path)
        try:
            connection.backup(backup)
            integrity = backup.execute("PRAGMA integrity_check").fetchone()
            if integrity is None or str(integrity[0]).lower() != "ok":
                raise RuntimeError("runtime SQLite backup integrity verification failed")
        finally:
            backup.close()
        fd = os.open(backup_path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        directory_fd = os.open(root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        digest = hashlib.sha256(backup_path.read_bytes()).hexdigest()
        if len(digest) != 64:
            raise RuntimeError("runtime SQLite backup digest verification failed")
        return digest



def _record_runtime_migration(
    connection: sqlite3.Connection,
    *,
    identity: RuntimeMigrationIdentity,
    backup_sha256: str,
    source_digests: dict[str, str | None],
    started_at: float,
    finished_at: float,
) -> None:
    if assert_migration_slot(connection, identity):
        return
    columns = {
        str(row[1])
        for row in connection.execute("PRAGMA table_info(runtime_schema_migrations)").fetchall()
    }
    values: dict[str, Any] = {
        "version": identity.version,
        "backup_sha256": backup_sha256,
        "source_json_digests_json": json.dumps(source_digests, sort_keys=True, separators=(",", ":")),
        "started_at": started_at,
        "finished_at": finished_at,
        "result": "committed",
        "name": identity.name,
        "ddl_sha256": identity.ddl_sha256,
        "applied_at": finished_at,
    }
    selected = [(name, values[name]) for name in values if name in columns]
    names = ",".join(name for name, _ in selected)
    placeholders = ",".join("?" for _ in selected)
    connection.execute(
        f"INSERT INTO runtime_schema_migrations({names}) VALUES({placeholders})",
        tuple(value for _, value in selected),
    )


def _migrate_report_identity_metadata_v1(
    connection: sqlite3.Connection,
    root: Path,
    identity: RuntimeMigrationIdentity,
) -> None:
    if assert_migration_slot(connection, identity):
        _verify_report_identity_metadata_v1(connection)
        return
    if int(connection.execute("PRAGMA user_version").fetchone()[0]) >= identity.version:
        raise RuntimeError("runtime report identity version is occupied without exact ledger identity")
    backup_sha256 = _verified_sqlite_backup(connection, root)
    started_at = time.time()
    connection.execute("BEGIN IMMEDIATE")
    try:
        occupied_trigger = connection.execute(
            """
            SELECT name FROM sqlite_master WHERE type='trigger'
            AND name IN (
                'runtime_metadata_report_identity_immutable_update',
                'runtime_metadata_report_identity_immutable_delete'
            ) LIMIT 1
            """
        ).fetchone()
        if occupied_trigger is not None:
            raise RuntimeError(f"unregistered report identity trigger exists: {occupied_trigger[0]}")
        now = time.time()
        _apply_report_identity_metadata_v1(connection, now)
        _record_runtime_migration(
            connection,
            identity=identity,
            backup_sha256=backup_sha256,
            source_digests={},
            started_at=started_at,
            finished_at=now,
        )
        connection.execute(
            "UPDATE runtime_store_identity SET schema_version=?,updated_at=? WHERE identity_id=1",
            (identity.version, now),
        )
        connection.execute(f"PRAGMA user_version={identity.version}")
        _verify_report_identity_metadata_v1(connection)
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise



def _migrate_runtime_release_start(
    connection: sqlite3.Connection,
    root: Path,
    identity: RuntimeMigrationIdentity,
) -> None:
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    if assert_migration_slot(connection, identity):
        if version != identity.version:
            raise RuntimeError("runtime release migration ledger and PRAGMA user_version disagree")
        _verify_runtime_release_start(connection)
        return
    if version >= identity.version:
        raise RuntimeError("runtime release migration version is occupied without exact ledger identity")
    if version != REPORT_IDENTITY_SCHEMA_VERSION:
        raise RuntimeError("runtime release migration requires the exact canonical v1-v3 prefix")
    occupied = connection.execute(
        """
        SELECT type,name FROM sqlite_master
        WHERE name IN (
            'runtime_release_receipts',
            'runtime_release_receipts_time_idx',
            'runtime_release_receipts_append_only_update',
            'runtime_release_receipts_append_only_delete'
        ) LIMIT 1
        """
    ).fetchone()
    if occupied is not None:
        raise RuntimeError(f"unregistered runtime release schema object exists: {occupied[0]}:{occupied[1]}")
    backup_sha256 = _verified_sqlite_backup(connection, root)
    started_at = time.time()
    connection.execute("BEGIN IMMEDIATE")
    try:
        _apply_runtime_release_start(connection)
        finished_at = time.time()
        _record_runtime_migration(
            connection,
            identity=identity,
            backup_sha256=backup_sha256,
            source_digests={},
            started_at=started_at,
            finished_at=finished_at,
        )
        connection.execute(
            "UPDATE runtime_store_identity SET schema_version=?,updated_at=? WHERE identity_id=1",
            (identity.version, finished_at),
        )
        connection.execute(f"PRAGMA user_version={identity.version}")
        _verify_runtime_release_start(connection)
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise


def _migrate_operator_command_plane_schema_v1(
    connection: sqlite3.Connection,
    root: Path,
    identity: RuntimeMigrationIdentity,
) -> None:
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    if assert_migration_slot(connection, identity):
        if version != identity.version:
            raise RuntimeError("operator command-plane migration ledger and PRAGMA user_version disagree")
        _verify_operator_command_plane_schema_v1(connection)
        return
    if version >= identity.version:
        raise RuntimeError("operator command-plane migration version is occupied without exact ledger identity")
    if version != RUNTIME_RELEASE_SCHEMA_VERSION:
        raise RuntimeError("operator command-plane migration requires the exact canonical v1-v4 prefix")
    backup_sha256 = _verified_sqlite_backup(connection, root)
    started_at = time.time()
    connection.execute("BEGIN IMMEDIATE")
    try:
        # CREATE IF NOT EXISTS permits adoption only when every pre-ledger object
        # already has the exact canonical definition. Any legacy shape other
        # than that frozen v5 schema fails the attestation and rolls back.
        _apply_operator_command_plane_schema_v1(connection)
        _verify_operator_command_plane_schema_v1(connection)
        finished_at = time.time()
        _record_runtime_migration(
            connection,
            identity=identity,
            backup_sha256=backup_sha256,
            source_digests={},
            started_at=started_at,
            finished_at=finished_at,
        )
        connection.execute(
            "UPDATE runtime_store_identity SET schema_version=?,updated_at=? WHERE identity_id=1",
            (identity.version, finished_at),
        )
        connection.execute(f"PRAGMA user_version={identity.version}")
        _verify_operator_command_plane_schema_v1(connection)
        connection.execute("COMMIT")
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise

def migrate_runtime_database_v2(connection: sqlite3.Connection, root: str | Path) -> None:
    """Apply the canonical ordered registry under the process-wide owner fence."""
    coordinator = runtime_write_coordinator(root)
    with coordinator.lock:
        _migrate_runtime_database_v2_locked(connection, root)


def _migrate_runtime_database_v2_locked(connection: sqlite3.Connection, root: str | Path) -> None:
    selected_root = Path(root).expanduser().resolve(strict=False)
    registry = canonical_runtime_migration_registry()
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    if version > registry[-1].version:
        raise RuntimeError(f"unsupported runtime schema version {version}")
    if version == SERIAL206_SCHEMA_VERSION:
        strict_tables = {
            "serial206_board_authority",
            "serial206_axis_authority",
            "serial206_movement_methods",
            "serial206_movement_commands",
            "serial206_command_resources",
            "serial206_command_dependencies",
            "serial206_interrupt_imports",
        }
        found_tables = {
            str(row[0])
            for row in connection.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
        }
        found_strict_tables = strict_tables & found_tables
        if found_strict_tables:
            rebuild_operator = (
                _is_additive_operator_schema(connection)
                or _is_legacy_v1_operator_schema(connection)
            )
            rebuild_receipts = _is_legacy_v1_serial206_receipts_schema(connection)
            if rebuild_operator or rebuild_receipts:
                _verified_sqlite_backup(connection, selected_root)
                if rebuild_operator:
                    _rebuild_additive_operator_schema(connection)
                if rebuild_receipts:
                    _normalize_legacy_v1_serial206_receipts(connection)
            _reinstall_runtime_authority_triggers(connection)
            ledger_row = connection.execute(
                "SELECT MAX(version) FROM runtime_schema_migrations"
            ).fetchone()
            ledger_version = int(ledger_row[0]) if ledger_row and ledger_row[0] is not None else version
            if ledger_version > version:
                expected_ledger_hash = _RUNTIME_PHYSICAL_SCHEMA_SHA256_BY_VERSION.get(ledger_version)
                if expected_ledger_hash is None:
                    raise RuntimeError(f"unsupported runtime schema version {ledger_version}")
                physical_hash = _runtime_physical_schema_sha256(connection)
                if physical_hash == expected_ledger_hash:
                    connection.execute(f"PRAGMA user_version={ledger_version}")
                    version = ledger_version
            _verify_v2_schema(connection)
            if version == registry[-1].version:
                verify_canonical_runtime_database(connection)
            journal_mode = str(connection.execute("PRAGMA journal_mode").fetchone()[0]).lower()
            synchronous = int(connection.execute("PRAGMA synchronous").fetchone()[0])
            if journal_mode != "wal" or synchronous != 2:
                raise RuntimeError("runtime schema v2 durability settings are not WAL/synchronous FULL")
            return

    try:
        verify_runtime_audit_foundation(connection)
    except Exception:
        foundation_backup_sha256 = _verified_sqlite_backup(connection, selected_root)
        ensure_schema(
            connection,
            selected_root,
            backup_sha256=foundation_backup_sha256,
        )
    verify_runtime_audit_foundation(connection)

    ledger_exists = connection.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name='runtime_schema_migrations'"
    ).fetchone() is not None
    if ledger_exists:
        ledger_rows = connection.execute(
            "SELECT version,name,ddl_sha256 FROM runtime_schema_migrations ORDER BY version"
        ).fetchall()
        actual_registry = tuple(
            (int(row[0]), str(row[1]), str(row[2])) for row in ledger_rows
        )
        expected_registry = tuple(
            (item.version, item.name, item.ddl_sha256) for item in registry
        )
        if actual_registry != expected_registry[: len(actual_registry)]:
            raise RuntimeError("runtime migration ledger is not a canonical ordered prefix")

    foundation = registry[0]
    if not assert_migration_slot(connection, foundation):
        foundation_backup_sha256 = _verified_sqlite_backup(connection, selected_root)
        ensure_schema(
            connection,
            selected_root,
            backup_sha256=foundation_backup_sha256,
        )
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])

    serial_identity = registry[1]
    if assert_migration_slot(connection, serial_identity):
        if version < serial_identity.version or version > registry[-1].version:
            raise RuntimeError("runtime migration ledger and PRAGMA user_version disagree")
        _verify_v2_schema(connection)
        report_identity = registry[2]
        _migrate_report_identity_metadata_v1(connection, selected_root, report_identity)
        if int(connection.execute("PRAGMA user_version").fetchone()[0]) < OPERATOR_COMMAND_PLANE_SCHEMA_VERSION:
            _migrate_runtime_release_start(connection, selected_root, registry[3])
        _migrate_operator_command_plane_schema_v1(connection, selected_root, registry[4])
        verify_canonical_runtime_database(connection)
        journal_mode = str(connection.execute("PRAGMA journal_mode").fetchone()[0]).lower()
        synchronous = int(connection.execute("PRAGMA synchronous").fetchone()[0])
        if journal_mode != "wal" or synchronous != 2:
            raise RuntimeError("canonical runtime durability settings are not WAL/synchronous FULL")
        return
    if version >= serial_identity.version:
        raise RuntimeError(
            "runtime PRAGMA user_version occupies serial-206 migration version without exact ledger identity"
        )

    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=FULL")
    connection.execute("PRAGMA foreign_keys=ON")
    source_payloads = _load_schema_sources(selected_root)
    source_digests = _schema_source_digests(selected_root)
    preserved_identities = {}
    for table in ("runtime_metadata", "operator_commands", "operator_transitions", "serial206_receipts"):
        columns = None
        if table == "operator_commands" and connection.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (table,)).fetchone() is not None:
            columns = tuple(
                str(row[1])
                for row in connection.execute(f"PRAGMA table_info({table})")
                if str(row[1]) != "idempotency_replay_enabled"
            )
        preserved_identities[table] = _legacy_table_identity(connection, table, columns)
    backup_sha256 = _verified_sqlite_backup(connection, selected_root)
    if _is_additive_operator_schema(connection) or _is_legacy_v1_operator_schema(connection):
        _rebuild_additive_operator_schema(connection)
    _normalize_legacy_v1_serial206_receipts(connection)
    started_at = time.time()
    connection.execute("BEGIN IMMEDIATE")
    try:
        table = connection.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='operator_commands'").fetchone()
        if table is not None and connection.execute("SELECT 1 FROM operator_commands WHERE status='executing' LIMIT 1").fetchone() is not None:
            raise RuntimeError("runtime v2 migration requires quiesced operator mutation admission")
        strict_existing = connection.execute(
            """
            SELECT name FROM sqlite_master
            WHERE type='table' AND name IN (
                'serial206_board_authority','serial206_axis_authority',
                'serial206_movement_methods','serial206_movement_commands',
                'serial206_command_resources','serial206_command_dependencies',
                'serial206_interrupt_imports'
            ) LIMIT 1
            """
        ).fetchone()
        if strict_existing is not None:
            raise RuntimeError(
                f"serial-206 migration found unregistered authority table: {strict_existing[0]}"
            )
        _create_v1_runtime_schema(connection)
        _create_v2_authority_schema(connection)
        reference_payload = source_payloads.get("reference-state.json")
        initialization_payload = source_payloads.get("serial206_oem_initialization_state.json")
        z_board_epoch = _find_nonnegative_int(initialization_payload, "board_lifecycle_generation")
        connection.execute(
            """
            INSERT INTO serial206_board_authority(
                board_id,state,prior_board_epoch,active_board_epoch,transition_id,
                member_motors_json,state_version,updated_at
            ) VALUES(4,'faulted',?,NULL,'migration-v2-continuity-unproved',?,1,?)
            """,
            (z_board_epoch, json.dumps(SERIAL206_BOARD4_MEMBERS, sort_keys=True, separators=(",", ":")), time.time()),
        )
        for axis, motor_id in SERIAL206_BOARD4_MEMBERS.items():
            legacy = _legacy_axis_reference(reference_payload, axis)
            legacy_state = str(legacy.get("state") or "unreferenced").lower()
            origin = legacy.get("origin_position_steps")
            if type(origin) is not int:
                origin = legacy.get("origin") if type(legacy.get("origin")) is int else None
            if axis == "gripper":
                lifecycle_state = "unprepared"
                reference_state = "unreferenced"
                origin = None
                prepared_epoch = None
            elif axis == "y":
                lifecycle_state = "generation_stale" if legacy else "unprepared"
                reference_state = "generation_stale" if legacy_state in {"referenced", "home", "homed"} else "reconciliation_required" if legacy_state in {"desynced", "reconciliation_required"} else "unreferenced"
                prepared_epoch = None
            else:
                lifecycle_state = "unprepared" if legacy_state == "unprepared" or not legacy else "generation_stale"
                reference_state = "generation_stale" if legacy_state in {"referenced", "home", "homed"} else "reconciliation_required" if legacy_state in {"desynced", "failed_latched", "reconciliation_required"} else "unreferenced"
                prepared_epoch = z_board_epoch
            connection.execute(
                """
                INSERT INTO serial206_axis_authority(
                    axis,board_id,motor_id,ownership_generation,prepared_board_epoch,
                    lifecycle_state,reference_state,origin_position_steps,
                    interrupt_epoch,state_version,updated_at
                ) VALUES(?,4,?,0,?,?,?,?,0,1,?)
                """,
                (axis, motor_id, prepared_epoch, lifecycle_state, reference_state, origin, time.time()),
            )
        for table in (
            "serial206_movement_methods",
            "serial206_movement_commands",
            "serial206_command_resources",
            "serial206_command_dependencies",
            "serial206_interrupt_imports",
        ):
            if int(connection.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]) != 0:
                raise RuntimeError(f"runtime v2 migration found unexpected preexisting rows: {table}")
        finished_at = time.time()
        _record_runtime_migration(
            connection,
            identity=serial_identity,
            backup_sha256=backup_sha256,
            source_digests=source_digests,
            started_at=started_at,
            finished_at=finished_at,
        )
        connection.execute(
            "UPDATE runtime_store_identity SET schema_version=?,updated_at=? WHERE identity_id=1",
            (serial_identity.version, finished_at),
        )
        connection.execute(f"PRAGMA user_version={serial_identity.version}")
        _verify_v2_schema(connection)
        violations = connection.execute("PRAGMA foreign_key_check").fetchall()
        if violations:
            raise RuntimeError(f"runtime v2 migration foreign-key verification failed: {violations}")
        for table, before in preserved_identities.items():
            if before is not None and _legacy_table_identity(connection, table, before[0]) != before:
                raise RuntimeError(f"runtime v2 migration changed frozen legacy rows: {table}")
        connection.execute("COMMIT")
        _migrate_report_identity_metadata_v1(connection, selected_root, registry[2])
        _migrate_runtime_release_start(connection, selected_root, registry[3])
        _migrate_operator_command_plane_schema_v1(connection, selected_root, registry[4])
        verify_canonical_runtime_database(connection)
    except Exception:
        if connection.in_transaction:
            connection.execute("ROLLBACK")
        raise


def _compact_controller_state(
    value: Any,
    *,
    key: str | None = None,
    depth: int = 0,
    budget: list[int] | None = None,
) -> Any:
    if budget is None:
        budget = [2048]
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False, default=str)
    digest_summary: dict[str, Any] = {
        "value_omitted_from_current_state": True,
        "content_sha256": hashlib.sha256(encoded.encode("utf-8")).hexdigest(),
        "encoded_bytes": len(encoded.encode("utf-8")),
    }
    if budget[0] <= 0 or depth >= 12:
        return digest_summary
    budget[0] -= 1
    raw_keys = {
        "wait", "events", "raw_packet", "raw_packets", "packets",
        "event_snapshot", "last_ack", "ack", "attempt_diagnostics",
        "controller_response",
    }
    if key in raw_keys:
        if isinstance(value, (list, tuple)):
            digest_summary["item_count"] = len(value)
        if isinstance(value, Mapping):
            for scalar_key in (
                "ok", "stopped", "target_reached", "timed_out", "last_speed",
                "failure", "source_return_code", "status", "value", "return_status",
            ):
                scalar = value.get(scalar_key)
                if scalar is None or isinstance(scalar, (str, int, float, bool)):
                    if scalar_key in value:
                        digest_summary[scalar_key] = scalar
        digest_summary["controller_payload_omitted_to_provider_receipt"] = True
        return digest_summary
    if isinstance(value, str) and len(value.encode("utf-8")) > 512:
        return digest_summary
    if isinstance(value, Mapping):
        items = []
        for original_key, child in value.items():
            child_key = str(original_key)
            if len(child_key.encode("utf-8")) > 128:
                child_key = "_oversized_key_" + hashlib.sha256(child_key.encode("utf-8")).hexdigest()
            items.append((child_key, child))
        items.sort(key=lambda row: row[0])
        selected = items[:64]
        result = {
            child_key: _compact_controller_state(child, key=child_key, depth=depth + 1, budget=budget)
            for child_key, child in selected
        }
        if len(items) > len(selected):
            result["_omitted_current_state_items"] = {**digest_summary, "item_count": len(items) - len(selected)}
        return result
    if isinstance(value, (list, tuple)):
        selected = list(value[:64])
        result = [_compact_controller_state(child, depth=depth + 1, budget=budget) for child in selected]
        if len(value) > len(selected):
            result.append({"_omitted_current_state_items": {**digest_summary, "item_count": len(value) - len(selected)}})
        return result
    return value

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
        self._audit_database = RuntimeAuditDatabase(root=root, initialize_schema=False)
        self.root = self._audit_database.root
        self.serial206_interrupt_fallback_path = self.root / "serial206_interrupt_fallback.jsonl"
        self.serial206_interrupt_fallback_lock_path = self.root / "serial206_interrupt_fallback.lock"
        self._lock = self._audit_database.writer_lock
        self._db = self._audit_database.connection
        self._authority_write_depth = 0
        self._db.create_function(
            "sha256_utf8",
            1,
            lambda value: hashlib.sha256(str(value).encode("utf-8")).hexdigest(),
            deterministic=True,
        )
        self._db.create_function(
            "canonical_json",
            1,
            lambda value: json.dumps(json.loads(str(value)), sort_keys=True, separators=(",", ":"), allow_nan=False),
            deterministic=True,
        )
        self._db.create_function(
            "authority_write_allowed",
            0,
            lambda: 1 if self._authority_write_depth > 0 else 0,
        )
        self._db.create_function(
            "sha256_blob",
            1,
            lambda value: hashlib.sha256(bytes(value)).hexdigest(),
            deterministic=True,
        )
        self._closed = False
        with self._authority_write():
            migrate_runtime_database_v2(self._db, self.root)
        self._authority_schema_version = int(self._db.execute("PRAGMA schema_version").fetchone()[0])
        self._seq = self._load_seq()
        with self._authority_write():
            self._import_retired_runtime_json_files()
        self._import_serial206_interrupt_fallback()

    @contextmanager
    def _authority_write(self):
        if self._authority_write_depth == 0 and hasattr(self, "_authority_schema_version"):
            schema_version = int(self._db.execute("PRAGMA schema_version").fetchone()[0])
            if schema_version != self._authority_schema_version:
                _verify_v2_schema(self._db)
                self._authority_schema_version = schema_version
        self._authority_write_depth += 1
        try:
            yield
        finally:
            self._authority_write_depth -= 1

    def _import_retired_runtime_json_files(self) -> None:
        names = (
            "operator_command_store.json",
            "sequence_state.json",
            "oem_runtime_state.json",
            "oem_full_lifecycle_runs.json",
            "oem_serial206_interrupt_journal.json",
            "oem_initialization_state.json",
            "reference-state.json",
            "serial206_oem_initialization_state.json",
        )
        for name in names:
            source = self.root / name
            if not source.is_file():
                continue
            content = source.read_bytes()
            digest = hashlib.sha256(content).hexdigest()
            self._db.execute(
                "INSERT OR IGNORE INTO runtime_retired_json_artifacts(source_name,content_sha256,content_blob,imported_at) VALUES(?,?,?,?)",
                (name, digest, content, time.time()),
            )
            existing = self._db.execute(
                "SELECT content_sha256,content_blob FROM runtime_retired_json_artifacts WHERE source_name=?",
                (name,),
            ).fetchone()
            if existing is None or str(existing["content_sha256"]) != digest or bytes(existing["content_blob"]) != content:
                raise RuntimeError(f"retired runtime JSON identity conflicts: {name}")
            archive = self.root / f"{name}.retired.{digest}"
            os.replace(source, archive)
            directory_fd = os.open(self.root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)

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

    def require_axis_reconciliation(self, axis: str, *, receipt_id: str) -> dict[str, Any]:
        selected = str(axis).strip().lower()
        if selected not in SERIAL206_BOARD4_MEMBERS:
            raise ValueError("unsupported board-4 axis")
        with self._lock, self._authority_write():
            self._db.execute("BEGIN IMMEDIATE")
            try:
                changed = self._db.execute(
                    "UPDATE serial206_axis_authority SET lifecycle_state='reconciliation_required',reference_state='reconciliation_required',prepared_board_epoch=NULL,last_receipt_id=?,interrupt_epoch=interrupt_epoch+1,state_version=state_version+1,updated_at=? WHERE axis=?",
                    (str(receipt_id), time.time(), selected),
                ).rowcount
                self._db.execute("COMMIT")
            except Exception:
                self._db.execute("ROLLBACK")
                raise
        return {"ok": changed == 1, "axis": selected, "receipt_id": str(receipt_id), "reconciliation_required": changed == 1}

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
        with self._lock, self._authority_write():
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
                self._db.execute(
                    """
                    INSERT INTO serial206_board_transitions(
                        transition_id,requested_active,ownership_generation,
                        delivery_attempted,reply_valid,status_code,continuity_proven,
                        accepted,state_before,state_after,prior_board_epoch,
                        active_board_epoch,created_at
                    ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)
                    """,
                    (
                        transition_id,
                        int(active),
                        int(ownership_generation),
                        1,
                        int(reply_valid),
                        status_code,
                        int(bool(continuity_proven)),
                        int(accepted),
                        str(current["state"]),
                        state,
                        prior_epoch,
                        active_epoch,
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
                            SET ownership_generation=?, prepared_board_epoch=NULL, lifecycle_state=?, reference_state=?,
                                state_version=state_version+1, updated_at=?
                            WHERE axis=?
                            """,
                            (int(ownership_generation), lifecycle, reference, now, axis),
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
        with self._lock, self._authority_write():
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
        with self._lock, self._authority_write():
            board = self._board4_row_locked()
            row = self._axis_rows_locked().get(axis, {})
            if board.get("state") != "active" or row.get("prepared_board_epoch") != board.get("active_board_epoch"):
                return {"ok": False, "failure": "axis_board_epoch_not_current", "axis": axis, "board": board, "axis_state": row}
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
        with self._lock, self._authority_write():
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

        pending_paths = sorted({
            *self.root.glob("serial206_interrupt_fallback.pending.*.jsonl"),
            *self.root.glob("serial206_interrupt_fallback.operator.pending.*.jsonl"),
            # Older command-plane code archived Y rows after importing only
            # idempotency projections. Re-read those archives into canonical
            # per-stream receipt history; receipt upserts make this repeatable.
            *self.root.glob("serial206_interrupt_fallback.operator.imported.*.jsonl"),
        })
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
            if ".pending." in pending.name:
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
            return {
                "ok": False,
                "failure": "sqlite_connection_busy",
                "persistence_state": "recovery_required",
            }
        try:
            self._db.execute("PRAGMA busy_timeout=0")
            try:
                return self.append_serial206_receipt(stream, dict(receipt))
            except sqlite3.Error as exc:
                return {
                    "ok": False,
                    "failure": f"{type(exc).__name__}: {exc}",
                    "persistence_state": "recovery_required",
                }
            finally:
                self._db.execute("PRAGMA busy_timeout=2000")
        finally:
            self._lock.release()

    def _serial206_receipt_set_locked(self) -> tuple[str, str]:
        rows = self._db.execute(
            "SELECT stream,receipt_id,receipt_json FROM serial206_receipts ORDER BY stream,receipt_id"
        ).fetchall()
        receipt_set = [
            [str(row["stream"]), str(row["receipt_id"]), hashlib.sha256(str(row["receipt_json"]).encode("utf-8")).hexdigest()]
            for row in rows
        ]
        encoded = json.dumps(receipt_set, sort_keys=True, separators=(",", ":"), allow_nan=False)
        return encoded, hashlib.sha256(encoded.encode("utf-8")).hexdigest()

    def _append_serial206_authority_snapshot_locked(self, state: Mapping[str, Any]) -> None:
        state_json = json.dumps(dict(state), sort_keys=True, separators=(",", ":"), allow_nan=False)
        receipt_set_json, receipt_set_sha256 = self._serial206_receipt_set_locked()
        self._db.execute(
            "INSERT INTO serial206_authority_snapshots(state_json,state_sha256,receipt_set_json,receipt_set_sha256,created_at) VALUES(?,?,?,?,?)",
            (
                state_json,
                hashlib.sha256(state_json.encode("utf-8")).hexdigest(),
                receipt_set_json,
                receipt_set_sha256,
                time.time(),
            ),
        )

    def _rebind_latest_serial206_authority_snapshot_locked(self) -> None:
        latest = self._db.execute(
            "SELECT state_json FROM serial206_authority_snapshots ORDER BY sequence DESC LIMIT 1"
        ).fetchone()
        if latest is not None:
            self._append_serial206_authority_snapshot_locked(json.loads(str(latest["state_json"])))

    def read_oem_serial206_initialization_state(self) -> dict[str, Any] | None:
        """Read append-only serial-206 authority bound to the immutable receipt set."""
        with self._lock:
            selected = self._db.execute(
                "SELECT * FROM serial206_authority_snapshots ORDER BY sequence DESC LIMIT 1"
            ).fetchone()
            if selected is None:
                legacy = self._db.execute(
                    "SELECT value FROM runtime_metadata WHERE key='serial206_oem_initialization_state'"
                ).fetchone()
                if legacy is None:
                    return None
                payload = json.loads(str(legacy[0]))
                if not isinstance(payload, dict):
                    raise ValueError("serial-206 initialization state must be an object")
                self._db.execute("BEGIN IMMEDIATE")
                try:
                    self._append_serial206_authority_snapshot_locked(payload)
                    self._db.execute("COMMIT")
                except Exception:
                    self._db.execute("ROLLBACK")
                    raise
                selected = self._db.execute(
                    "SELECT * FROM serial206_authority_snapshots ORDER BY sequence DESC LIMIT 1"
                ).fetchone()
            state_json = str(selected["state_json"])
            canonical_state_json = json.dumps(
                json.loads(state_json), sort_keys=True, separators=(",", ":"), allow_nan=False
            )
            if (
                state_json != canonical_state_json
                or str(selected["state_sha256"])
                != hashlib.sha256(state_json.encode("utf-8")).hexdigest()
            ):
                raise RuntimeError("serial-206 authority snapshot state bytes or hash are incoherent")
            stored_receipt_set_json = str(selected["receipt_set_json"])
            canonical_receipt_set_json = json.dumps(
                json.loads(stored_receipt_set_json),
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            if (
                stored_receipt_set_json != canonical_receipt_set_json
                or str(selected["receipt_set_sha256"])
                != hashlib.sha256(stored_receipt_set_json.encode("utf-8")).hexdigest()
            ):
                raise RuntimeError("serial-206 authority snapshot receipt-set bytes or hash are incoherent")
            receipt_set_json, receipt_set_sha256 = self._serial206_receipt_set_locked()
            if stored_receipt_set_json != receipt_set_json or str(selected["receipt_set_sha256"]) != receipt_set_sha256:
                raise RuntimeError("serial-206 authority snapshot is not bound to the current immutable receipt set")
            payload = json.loads(state_json)
        if not isinstance(payload, dict):
            raise ValueError("serial-206 initialization state must be an object")
        return payload

    def write_oem_serial206_initialization_state(self, state: dict[str, Any]) -> dict[str, Any]:
        """Append compact current authority with an exact immutable receipt-set binding."""
        payload = dict(state)
        required = {"movement_ledger", "used_approvals", "initialize_motion_ledger"}
        if not required.issubset(payload):
            raise ValueError("serial-206 state must contain all lifecycle ledgers")
        stored_payload = _compact_controller_state(payload)
        if not isinstance(stored_payload, dict):
            raise ValueError("serial-206 state compaction must preserve object shape")
        for lifecycle_key in ("z_lifecycle", "x_lifecycle"):
            lifecycle = stored_payload.get(lifecycle_key)
            if isinstance(lifecycle, dict):
                compact_lifecycle = dict(lifecycle)
                receipts = compact_lifecycle.get("receipts")
                if isinstance(receipts, list):
                    compact_lifecycle["receipts"] = receipts[-1:]
                    compact_lifecycle["receipts_omitted_to_sqlite"] = max(0, len(receipts) - 1)
                stored_payload[lifecycle_key] = compact_lifecycle
        encoded_current = json.dumps(stored_payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
        if len(encoded_current.encode("utf-8")) > 262_144:
            stored_payload = _compact_controller_state(payload, budget=[256])
            if not isinstance(stored_payload, dict):
                raise ValueError("serial-206 bounded state compaction must preserve object shape")
            encoded_current = json.dumps(stored_payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
        if len(encoded_current.encode("utf-8")) > 262_144:
            raise ValueError("serial-206 compact current state exceeds encoded-byte ceiling")
        with self._lock:
            with self._authority_write():
                self._db.execute("BEGIN IMMEDIATE")
                try:
                    self._append_serial206_authority_snapshot_locked(stored_payload)
                    self._db.execute("COMMIT")
                except Exception:
                    self._db.execute("ROLLBACK")
                    raise
        return payload

    def append_serial206_receipts_atomic(
        self,
        receipts: Iterable[tuple[str, dict[str, Any]]],
    ) -> list[dict[str, Any]]:
        """Persist several stream receipts in one SQLite transaction."""
        normalized: list[tuple[str, dict[str, Any], tuple[Any, ...]]] = []
        for stream, receipt in receipts:
            selected_stream = str(stream).strip().lower()
            if selected_stream not in {"x", "y", "z", "initialize_motors", "initialize_motion"}:
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
            self._authority_write_depth += 1
            self._db.execute("BEGIN IMMEDIATE")
            try:
                for selected_stream, _payload, row in normalized:
                    if selected_stream in {"x", "y", "z"}:
                        self._db.execute(
                            """
                            INSERT OR IGNORE INTO serial206_receipts(
                                stream,receipt_id,command_id,idempotency_key,
                                idempotency_replay_enabled,status,observed_at,receipt_json
                            ) VALUES(?,?,?,?,?,?,?,?)
                            """,
                            row,
                        )
                        existing = self._db.execute(
                            "SELECT receipt_json FROM serial206_receipts WHERE stream=? AND receipt_id=?",
                            (row[0], row[1]),
                        ).fetchone()
                        if existing is None or str(existing[0]) != str(row[7]):
                            raise ValueError("serial-206 provider receipt identity conflicts with immutable receipt")
                    else:
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
                self._rebind_latest_serial206_authority_snapshot_locked()
                self._db.execute("COMMIT")
            except Exception:
                self._db.execute("ROLLBACK")
                self._authority_write_depth -= 1
                raise
            self._authority_write_depth -= 1
        return [payload for _stream, payload, _row in normalized]

    def append_serial206_receipt(self, stream: str, receipt: dict[str, Any]) -> dict[str, Any]:
        """Persist one provider receipt without expanding the current-state file."""
        return self.append_serial206_receipts_atomic([(stream, receipt)])[0]
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
        row = self._db.execute(
            "SELECT value FROM runtime_metadata WHERE key='runtime_sequence'"
        ).fetchone()
        return int(row[0]) if row is not None else 0

    def next_seq(self) -> int:
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                row = self._db.execute(
                    "SELECT value FROM runtime_metadata WHERE key='runtime_sequence'"
                ).fetchone()
                current = int(row[0]) if row is not None else 0
                self._seq = max(int(self._seq), current) + 1
                self._db.execute(
                    "INSERT INTO runtime_metadata(key,value,updated_at) VALUES('runtime_sequence',?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value,updated_at=excluded.updated_at",
                    (str(self._seq), time.time()),
                )
                self._db.execute("COMMIT")
            except Exception:
                self._db.execute("ROLLBACK")
                raise
            return self._seq

    def write_state(self, snapshot: OEMRuntimeSnapshot | dict[str, Any]) -> dict[str, Any]:
        if isinstance(snapshot, OEMRuntimeSnapshot):
            payload: dict[str, Any] = snapshot.to_dict()
        else:
            payload = dict(snapshot)
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
        sequence = self.next_seq()
        payload["sequence"] = sequence
        payload["updated_at"] = utc_ts()
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
        with self._lock:
            with self._authority_write():
                self._db.execute(
                    "INSERT INTO runtime_state_snapshots(sequence,state_json,state_sha256,created_at) VALUES(?,?,?,?)",
                    (
                        sequence,
                        encoded,
                        hashlib.sha256(encoded.encode("utf-8")).hexdigest(),
                        time.time(),
                    ),
                )
        return payload

    def read_state(self) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute(
                "SELECT state_json,state_sha256 FROM runtime_state_snapshots ORDER BY sequence DESC LIMIT 1"
            ).fetchone()
        if row is None:
            return None
        encoded = str(row["state_json"])
        if encoded != json.dumps(json.loads(encoded), sort_keys=True, separators=(",", ":"), allow_nan=False):
            raise RuntimeError("runtime state snapshot JSON is not canonical")
        if str(row["state_sha256"]) != hashlib.sha256(encoded.encode("utf-8")).hexdigest():
            raise RuntimeError("runtime state snapshot digest mismatch")
        return json.loads(encoded)


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
        """Persist one full OEM movement-lifecycle run in SQLite."""
        payload = dict(run)
        run_id = str(payload.get("run_id") or "").strip()
        if not run_id or "/" in run_id or "\\" in run_id or run_id in {".", ".."}:
            raise ValueError("valid robot-owned run_id required")
        with self._lock, self._authority_write():
            sequence = self.next_seq()
            payload["sequence"] = sequence
            encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
            self._db.execute(
                """
                INSERT INTO runtime_movement_runs(run_id,sequence,run_json,run_sha256,updated_at)
                VALUES(?,?,?,?,?)
                ON CONFLICT(run_id) DO UPDATE SET
                    sequence=excluded.sequence,
                    run_json=excluded.run_json,
                    run_sha256=excluded.run_sha256,
                    updated_at=excluded.updated_at
                """,
                (run_id, sequence, encoded, hashlib.sha256(encoded.encode("utf-8")).hexdigest(), time.time()),
            )
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
        with self._lock:
            row = self._db.execute(
                "SELECT run_json,run_sha256 FROM runtime_movement_runs WHERE run_id=?", (selected,)
            ).fetchone()
        if row is None:
            return None
        encoded = str(row["run_json"])
        if encoded != json.dumps(json.loads(encoded), sort_keys=True, separators=(",", ":"), allow_nan=False):
            raise RuntimeError("runtime movement run JSON is not canonical")
        if str(row["run_sha256"]) != hashlib.sha256(encoded.encode("utf-8")).hexdigest():
            raise RuntimeError("runtime movement run digest mismatch")
        return json.loads(encoded)

    def list_oem_full_lifecycle_runs(self) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                "SELECT run_json,run_sha256 FROM runtime_movement_runs ORDER BY sequence,run_id"
            ).fetchall()
        result = []
        for row in rows:
            encoded = str(row["run_json"])
            if encoded != json.dumps(json.loads(encoded), sort_keys=True, separators=(",", ":"), allow_nan=False):
                raise RuntimeError("runtime movement run JSON is not canonical")
            if str(row["run_sha256"]) != hashlib.sha256(encoded.encode("utf-8")).hexdigest():
                raise RuntimeError("runtime movement run digest mismatch")
            result.append(json.loads(encoded))
        return result

    def append_journal(self, name: str, payload: dict[str, Any]) -> dict[str, Any]:
        row = dict(payload)
        row.setdefault("created_at", utc_ts())
        sequence = self.next_seq()
        row["sequence"] = sequence
        encoded = json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False)
        with self._lock:
            with self._authority_write():
                self._db.execute(
                    "INSERT INTO runtime_journal(sequence,stream,payload_json,payload_sha256,created_at) VALUES(?,?,?,?,?)",
                    (sequence, str(name), encoded, hashlib.sha256(encoded.encode("utf-8")).hexdigest(), time.time()),
                )
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
        bounded = max(1, min(int(limit), 500))
        with self._lock:
            rows = self._db.execute(
                "SELECT payload_json,payload_sha256 FROM runtime_journal WHERE stream=? ORDER BY sequence DESC LIMIT ?",
                (str(name), bounded),
            ).fetchall()
        result = []
        for row in reversed(rows):
            encoded = str(row["payload_json"])
            if encoded != json.dumps(json.loads(encoded), sort_keys=True, separators=(",", ":"), allow_nan=False):
                raise RuntimeError("runtime journal JSON is not canonical")
            if str(row["payload_sha256"]) != hashlib.sha256(encoded.encode("utf-8")).hexdigest():
                raise RuntimeError("runtime journal digest mismatch")
            result.append(json.loads(encoded))
        return result

    def recover_state(self) -> dict[str, Any]:
        state = self.read_state()
        if state is None:
            return {"recovery": "fresh", "state": None, "recovery_required": False}
        worker = state.get("worker") or {}
        active = worker.get("active_command")
        running = worker.get("state") == "running" or active is not None
        return {"recovery": "active_command" if running else "idle", "state": state, "recovery_required": bool(running)}
