import sqlite3

from bioxp.oem_runtime_store import (
    OEMRuntimeStore,
    SERIAL206_SCHEMA_VERSION,
    migrate_runtime_database_v2,
)
from bioxp.runtime_audit_store import (
    _expected_foundation_connection,
    _rebuild_runtime_audit_foundation,
    verify_runtime_audit_foundation,
)


def test_foundation_verification_accepts_default_tuple_row_factory():
    connection = _expected_foundation_connection()
    try:
        connection.row_factory = None
        verify_runtime_audit_foundation(connection)
        assert connection.row_factory is None
    finally:
        connection.close()


def test_foundation_rebuild_derives_legacy_evidence_link_targets():
    connection = sqlite3.connect(":memory:")
    connection.row_factory = sqlite3.Row
    connection.executescript(
        """
        CREATE TABLE runtime_evidence_links (
            evidence_link_id INTEGER PRIMARY KEY,
            evidence_artifact_id TEXT NOT NULL,
            command_id TEXT,
            pipette_operation_id TEXT,
            link_kind TEXT NOT NULL,
            created_at REAL NOT NULL
        );
        INSERT INTO runtime_evidence_links VALUES
            (1,'evidence:one','command-1',NULL,'command_evidence',1.0),
            (2,'evidence:two',NULL,'pipette-1','pipette_evidence',2.0);
        """
    )
    try:
        _rebuild_runtime_audit_foundation(connection)
        rows = connection.execute(
            "SELECT target_kind,target_identity FROM runtime_evidence_links ORDER BY evidence_link_id"
        ).fetchall()
        assert [tuple(row) for row in rows] == [
            ("command", "command-1"),
            ("pipette_operation", "pipette-1"),
        ]
    finally:
        connection.close()


def test_serial206_fast_path_repairs_recognized_foundation_drift(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    store.close()
    connection = sqlite3.connect(tmp_path / "bioxp_runtime.db")
    try:
        connection.execute("DROP INDEX runtime_evidence_links_target_idx")
        connection.execute(f"PRAGMA user_version={SERIAL206_SCHEMA_VERSION}")
        connection.commit()
        migrate_runtime_database_v2(connection, tmp_path)
        verify_runtime_audit_foundation(connection)
    finally:
        connection.close()
