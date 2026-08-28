import sqlite3

from bioxp.oem_runtime_store import (
    OEMRuntimeStore,
    SERIAL206_SCHEMA_VERSION,
    migrate_runtime_database_v2,
)
from bioxp.runtime_audit_store import (
    _expected_foundation_connection,
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
