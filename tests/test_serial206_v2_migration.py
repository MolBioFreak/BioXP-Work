import json
import sqlite3

import pytest

import src.bioxp.oem_runtime_store as runtime_store_module
from src.bioxp.oem_runtime_store import (
    OEMRuntimeStore,
    canonical_runtime_migration_registry,
)
from src.bioxp.runtime_audit_store import RuntimeAuditDatabase


def test_plain_runtime_connection_defers_substantive_ddl_to_migration_owner(tmp_path):
    database = RuntimeAuditDatabase(tmp_path, initialize_schema=False)

    assert database.connection.execute(
        "SELECT COUNT(*) FROM sqlite_master WHERE type IN ('table','index','trigger')"
    ).fetchone()[0] == 0
    database.close()


def test_fresh_runtime_database_has_serial206_v2_authority(tmp_path):
    store = OEMRuntimeStore(tmp_path)

    assert store._db.execute("PRAGMA user_version").fetchone()[0] == 5
    tables = {
        row[0]
        for row in store._db.execute(
            "SELECT name FROM sqlite_master WHERE type='table'"
        )
    }
    assert {
        "runtime_schema_migrations",
        "serial206_board_authority",
        "serial206_axis_authority",
        "serial206_movement_methods",
        "serial206_movement_commands",
        "serial206_command_resources",
        "serial206_command_dependencies",
        "serial206_interrupt_imports",
    } <= tables
    assert store._db.execute(
        "SELECT state FROM serial206_board_authority WHERE board_id=4"
    ).fetchone()[0] == "faulted"
    assert store._db.execute(
        "SELECT COUNT(*) FROM serial206_axis_authority"
    ).fetchone()[0] == 3


def test_existing_v1_operator_rows_are_preserved_and_migrated(tmp_path):
    db = tmp_path / "bioxp_runtime.db"
    connection = sqlite3.connect(db)
    connection.executescript(
        """
        PRAGMA user_version=1;
        CREATE TABLE runtime_metadata (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL,
            updated_at REAL NOT NULL
        ) WITHOUT ROWID;
        CREATE TABLE operator_commands (
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
            controller_acknowledged INTEGER NOT NULL DEFAULT 0,
            physical_effect_verified INTEGER NOT NULL DEFAULT 0,
            receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
            response_summary_json TEXT,
            evidence_relpath TEXT,
            evidence_sha256 TEXT,
            evidence_bytes INTEGER,
            updated_at REAL NOT NULL
        );
        CREATE TABLE operator_transitions (
            transition_id INTEGER PRIMARY KEY AUTOINCREMENT,
            command_id TEXT NOT NULL REFERENCES operator_commands(command_id),
            state TEXT NOT NULL,
            observed_at REAL NOT NULL,
            detail_json TEXT
        );
        CREATE TABLE serial206_receipts (
            stream TEXT NOT NULL,
            receipt_id TEXT NOT NULL,
            command_id TEXT,
            idempotency_key TEXT,
            idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1,
            status TEXT,
            observed_at REAL NOT NULL,
            receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
            PRIMARY KEY(stream, receipt_id)
        ) WITHOUT ROWID;
        INSERT INTO operator_commands(
            command_id,idempotency_key,action_id,status,ownership_generation,
            started_at,receipt_json,updated_at
        ) VALUES('old-command','old-key','old.action','completed',1,'1','{}',1);
        """
    )
    connection.commit()
    connection.close()

    store = OEMRuntimeStore(tmp_path)

    assert store._db.execute("PRAGMA user_version").fetchone()[0] == 5
    row = store._db.execute(
        "SELECT command_id,receipt_json FROM operator_commands"
    ).fetchone()
    assert tuple(row) == ("old-command", "{}")
    assert store._db.execute(
        "SELECT COUNT(*) FROM runtime_schema_migrations"
    ).fetchone()[0] == 5


def test_existing_v2_additive_operator_schema_is_rebuilt_without_data_loss(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    store.close()
    db = tmp_path / "bioxp_runtime.db"
    connection = sqlite3.connect(db)
    connection.execute("PRAGMA foreign_keys=OFF")
    connection.execute("PRAGMA legacy_alter_table=ON")
    connection.executescript(
        """
        ALTER TABLE operator_transitions RENAME TO operator_transitions_canonical;
        ALTER TABLE operator_commands RENAME TO operator_commands_canonical;
        CREATE TABLE operator_commands (
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
            response_summary_json TEXT,
            evidence_relpath TEXT,
            evidence_sha256 TEXT,
            evidence_bytes INTEGER,
            updated_at REAL NOT NULL,
            canonical_request_sha256 TEXT NOT NULL DEFAULT '',
            operation TEXT NOT NULL DEFAULT 'operator_action',
            command_kind TEXT NOT NULL DEFAULT 'pipette',
            entrypoint_id TEXT NOT NULL DEFAULT 'unknown',
            caller_class TEXT NOT NULL DEFAULT 'operator',
            control_class TEXT NOT NULL DEFAULT 'service',
            source_identity_json TEXT NOT NULL DEFAULT '{}',
            requested_inputs_json TEXT NOT NULL DEFAULT '{}',
            effective_inputs_json TEXT NOT NULL DEFAULT '{}',
            connection_generation INTEGER,
            admitted_at TEXT,
            dispatched_at TEXT,
            delivery_verified INTEGER NOT NULL DEFAULT 0
                CHECK(delivery_verified IN (0,1)),
            completion_verified INTEGER NOT NULL DEFAULT 0
                CHECK(completion_verified IN (0,1)),
            hardware_precondition_verified INTEGER NOT NULL DEFAULT 0
                CHECK(hardware_precondition_verified IN (0,1)),
            hardware_postcondition_verified INTEGER NOT NULL DEFAULT 0
                CHECK(hardware_postcondition_verified IN (0,1)),
            outcome TEXT,
            failure_code TEXT,
            evidence_state TEXT
        );
        CREATE TABLE operator_transitions (
            transition_id INTEGER PRIMARY KEY AUTOINCREMENT,
            command_id TEXT NOT NULL REFERENCES operator_commands(command_id) ON DELETE CASCADE,
            state TEXT NOT NULL,
            observed_at REAL NOT NULL,
            detail_json TEXT CHECK(detail_json IS NULL OR json_valid(detail_json))
        );
        INSERT INTO operator_commands(
            sequence,command_id,idempotency_key,action_id,status,ownership_generation,
            started_at,receipt_json,updated_at
        ) VALUES(7,'persisted-command','persisted-key','old.action','completed',3,'1','{}',1);
        INSERT INTO operator_transitions(
            transition_id,command_id,state,observed_at,detail_json
        ) VALUES(9,'persisted-command','completed',1,'{}');
        UPDATE sqlite_sequence SET seq=15 WHERE name='operator_commands';
        UPDATE sqlite_sequence SET seq=21 WHERE name='operator_transitions';
        DROP TABLE operator_transitions_canonical;
        DROP TABLE operator_commands_canonical;
        PRAGMA user_version=2;
        """
    )
    connection.commit()
    connection.close()

    migrated = OEMRuntimeStore(tmp_path)

    command = migrated._db.execute(
        "SELECT sequence,command_id,idempotency_key,status,ownership_generation "
        "FROM operator_commands WHERE command_id='persisted-command'"
    ).fetchone()
    transition = migrated._db.execute(
        "SELECT transition_id,command_id,state FROM operator_transitions "
        "WHERE command_id='persisted-command'"
    ).fetchone()
    assert tuple(command) == (7, "persisted-command", "persisted-key", "completed", 3)
    assert tuple(transition) == (9, "persisted-command", "completed")
    sequences = dict(migrated._db.execute(
        "SELECT name,seq FROM sqlite_sequence WHERE name IN ('operator_commands','operator_transitions')"
    ).fetchall())
    assert sequences == {"operator_commands": 15, "operator_transitions": 21}
    assert migrated._db.execute("PRAGMA foreign_key_check").fetchall() == []


def test_existing_v2_legacy_receipt_constraint_is_repaired_without_data_loss(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    store.close()
    db = tmp_path / "bioxp_runtime.db"
    connection = sqlite3.connect(db)
    for (trigger_name,) in connection.execute(
        "SELECT name FROM sqlite_master WHERE type='trigger' AND tbl_name='serial206_receipts'"
    ).fetchall():
        connection.execute(f'DROP TRIGGER "{trigger_name}"')
    connection.executescript(
        """
        ALTER TABLE serial206_receipts RENAME TO serial206_receipts_canonical;
        CREATE TABLE serial206_receipts (
            stream TEXT NOT NULL,
            receipt_id TEXT NOT NULL,
            command_id TEXT,
            idempotency_key TEXT,
            idempotency_replay_enabled INTEGER NOT NULL DEFAULT 1,
            status TEXT,
            observed_at REAL NOT NULL,
            receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
            PRIMARY KEY(stream, receipt_id)
        ) WITHOUT ROWID;
        INSERT INTO serial206_receipts(
            stream,receipt_id,command_id,idempotency_key,
            idempotency_replay_enabled,status,observed_at,receipt_json
        ) VALUES('x','legacy-receipt','legacy-command','legacy-key',1,'completed',1,'{}');
        DROP TABLE serial206_receipts_canonical;
        PRAGMA user_version=2;
        """
    )
    connection.commit()
    connection.close()

    migrated = OEMRuntimeStore(tmp_path)

    row = migrated._db.execute(
        "SELECT stream,receipt_id,command_id,idempotency_key,status,receipt_json "
        "FROM serial206_receipts WHERE receipt_id='legacy-receipt'"
    ).fetchone()
    assert tuple(row) == (
        "x", "legacy-receipt", "legacy-command", "legacy-key", "completed", "{}"
    )
    table_sql = migrated._db.execute(
        "SELECT sql FROM sqlite_master WHERE type='table' AND name='serial206_receipts'"
    ).fetchone()[0]
    assert "CHECK(idempotency_replay_enabled IN (0, 1))" in table_sql


def test_future_schema_version_refuses_without_mutation(tmp_path):
    db = tmp_path / "bioxp_runtime.db"
    connection = sqlite3.connect(db)
    connection.execute("PRAGMA user_version=99")
    connection.commit()
    connection.close()

    with pytest.raises(RuntimeError, match="unsupported runtime schema version"):
        OEMRuntimeStore(tmp_path)

    connection = sqlite3.connect(db)
    assert connection.execute("PRAGMA user_version").fetchone()[0] == 99
    assert connection.execute(
        "SELECT COUNT(*) FROM sqlite_master WHERE type='table'"
    ).fetchone()[0] == 0
    connection.close()


def test_v2_migration_is_idempotent(tmp_path):
    first = OEMRuntimeStore(tmp_path)
    first.close()
    second = OEMRuntimeStore(tmp_path)

    assert second._db.execute("PRAGMA user_version").fetchone()[0] == 5
    assert second._db.execute(
        "SELECT COUNT(*) FROM runtime_schema_migrations"
    ).fetchone()[0] == 5


def test_canonical_registry_has_unique_monotonic_exact_identities(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    registry = canonical_runtime_migration_registry()

    assert [item.version for item in registry] == [1, 2, 3, 4, 5]
    assert len({item.version for item in registry}) == len(registry)
    rows = store._db.execute(
        """
        SELECT version,name,ddl_sha256,backup_sha256
        FROM runtime_schema_migrations ORDER BY version
        """
    ).fetchall()
    assert [(row["version"], row["name"], row["ddl_sha256"]) for row in rows] == [
        (item.version, item.name, item.ddl_sha256) for item in registry
    ]
    assert all(len(row["backup_sha256"]) == 64 for row in rows)


def test_occupied_mismatched_migration_version_is_rejected_without_replacement(tmp_path):
    db = tmp_path / "bioxp_runtime.db"
    connection = sqlite3.connect(db)
    connection.executescript(
        """
        PRAGMA user_version=1;
        CREATE TABLE runtime_schema_migrations (
            version INTEGER PRIMARY KEY,
            name TEXT NOT NULL,
            ddl_sha256 TEXT NOT NULL,
            applied_at REAL NOT NULL
        ) WITHOUT ROWID;
        INSERT INTO runtime_schema_migrations(version,name,ddl_sha256,applied_at)
        VALUES(1,'occupied-by-other-owner','deadbeef',1);
        """
    )
    connection.close()

    with pytest.raises(RuntimeError, match="mismatched name or digest"):
        OEMRuntimeStore(tmp_path)

    connection = sqlite3.connect(db)
    assert connection.execute(
        "SELECT name,ddl_sha256 FROM runtime_schema_migrations WHERE version=1"
    ).fetchone() == ("occupied-by-other-owner", "deadbeef")
    connection.close()


def test_shared_interrupt_fallback_import_preserves_x_y_z_history(tmp_path):
    rows = [
        {"stream": axis, "receipt": {"receipt_id": f"{axis}-stop-1", "interrupt_attempt_id": f"{axis}-attempt-1", "intent": "stop", "status": "stopped"}}
        for axis in ("x", "y", "z")
    ]
    (tmp_path / "serial206_interrupt_fallback.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )
    store = OEMRuntimeStore(tmp_path)
    for axis in ("x", "y", "z"):
        receipts = store.list_serial206_receipts(axis)
        assert [row["interrupt_attempt_id"] for row in receipts] == [f"{axis}-attempt-1"]
    assert not (tmp_path / "serial206_interrupt_fallback.jsonl").exists()
    assert len(list(tmp_path.glob("serial206_interrupt_fallback.imported.*.jsonl"))) == 1


@pytest.mark.parametrize(
    ("blocked_symbol", "committed_version"),
    [
        ("_migrate_runtime_release_start", 3),
        ("_migrate_operator_command_plane_schema_v1", 4),
    ],
)
def test_committed_intermediate_migration_resumes_to_v5(
    tmp_path,
    monkeypatch,
    blocked_symbol,
    committed_version,
):
    original = getattr(runtime_store_module, blocked_symbol)

    def interrupt(*_args, **_kwargs):
        raise RuntimeError(f"simulated interruption after v{committed_version}")

    monkeypatch.setattr(runtime_store_module, blocked_symbol, interrupt)
    with pytest.raises(RuntimeError, match=f"simulated interruption after v{committed_version}"):
        OEMRuntimeStore(tmp_path)

    connection = sqlite3.connect(tmp_path / "bioxp_runtime.db")
    try:
        assert connection.execute("PRAGMA user_version").fetchone()[0] == committed_version
    finally:
        connection.close()

    monkeypatch.setattr(runtime_store_module, blocked_symbol, original)
    resumed = OEMRuntimeStore(tmp_path)
    assert resumed._db.execute("PRAGMA user_version").fetchone()[0] == 5
    resumed.close()
