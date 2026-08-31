from __future__ import annotations

import sqlite3

import pytest

from bioxp.operator_command_plane import (
    OperatorCommandStore,
    _OPERATOR_PHYSICAL_SCHEMA_SHA256,
    _operator_physical_schema_sha256,
)
from bioxp.oem_runtime_store import SERIAL206_SCHEMA_VERSION


def test_fresh_database_has_exact_wp9_schema_manifest_and_single_migration_ledger_row(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        assert store.connection.execute("PRAGMA foreign_key_check").fetchall() == []
        assert int(store.connection.execute("PRAGMA user_version").fetchone()[0]) == SERIAL206_SCHEMA_VERSION
        assert _operator_physical_schema_sha256(store.connection) == _OPERATOR_PHYSICAL_SCHEMA_SHA256
        rows = store.connection.execute(
            "SELECT version,result FROM runtime_schema_migrations ORDER BY version"
        ).fetchall()
        assert [(int(row[0]), str(row[1])) for row in rows] == [(SERIAL206_SCHEMA_VERSION, "committed")]
    finally:
        store.connection.close()


def test_direct_sql_rejects_forged_command_plan_stage_and_recovery_rows(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "INSERT INTO operator_plane_deck_commands("
                "command_id,target,target_label,resolved_location_id,destination_catalog_revision,"
                "position_table_revision,authority_snapshot_digest,complete_authority_digest,plan_digest,"
                "source_branch,source_anchors_json,source_hazards_json,planned_at) "
                "VALUES('forged','LOC_MS','Manual Scan',1,?,?,?,?,?,'ordinary','[]','[]',1.0)",
                ("0" * 64, "1" * 64, "2" * 64, "3" * 64, "4" * 64),
            )
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "DELETE FROM operator_plane_deck_semantic_state WHERE singleton=1"
            )
    finally:
        store.connection.close()
