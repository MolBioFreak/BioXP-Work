import json
import sqlite3

import pytest

from src.bioxp.oem_runtime_store import OEMRuntimeStore


def test_fresh_runtime_database_has_serial206_v2_authority(tmp_path):
    store = OEMRuntimeStore(tmp_path)

    assert store._db.execute("PRAGMA user_version").fetchone()[0] == 2
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

    assert store._db.execute("PRAGMA user_version").fetchone()[0] == 2
    row = store._db.execute(
        "SELECT command_id,receipt_json FROM operator_commands"
    ).fetchone()
    assert tuple(row) == ("old-command", "{}")
    assert store._db.execute(
        "SELECT COUNT(*) FROM runtime_schema_migrations"
    ).fetchone()[0] == 1


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

    assert second._db.execute("PRAGMA user_version").fetchone()[0] == 2
    assert second._db.execute(
        "SELECT COUNT(*) FROM runtime_schema_migrations"
    ).fetchone()[0] == 1


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
