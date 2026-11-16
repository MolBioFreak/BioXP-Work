"""Offline real-schema publication regression; no dispatcher or hardware."""
import json
import os
import sqlite3
import threading
import time
from pathlib import Path

import pytest
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp import runtime_workflow_pacing_v16 as pacing

JOB = "protocol-live-fe245b5cb1606a7a2563d241347983c4a2da37a87a8e053dc7e94a42b8d86181"


def scratch_store(path, source):
    path.mkdir(parents=True, exist_ok=True)
    source_db = sqlite3.connect(f"file:{source}?mode=ro", uri=True)
    connection = sqlite3.connect(path / "bioxp_runtime.db", isolation_level=None)
    try:
        source_db.backup(connection)
    finally:
        source_db.close()
    store = object.__new__(OperatorCommandStore)
    store.root, store.connection = path, connection
    connection.row_factory = sqlite3.Row
    store._lock = threading.RLock()
    store._authority_write_depth = 0
    store._transition_pending = False
    store._deck_owner_authority_reader = None
    store._configure()
    store.owner_id = connection.execute("SELECT owner_id FROM operator_plane_lane").fetchone()[0]
    with store._transaction() as conn:
        conn.execute("UPDATE operator_plane_lane SET owner_lease_until=?", (time.time() + 3600,))
    return store


def versions(store):
    return tuple(store.connection.execute(
        "SELECT count(*),coalesce(sum(length(row_json)),0) FROM operator_plane_command_versions WHERE command_id=?",
        (JOB,)).fetchone())


def test_real_store_publication_and_migration(tmp_path):
    source = os.environ.get("WORKFLOW_PACING_BASELINE")
    if not source:
        pytest.skip("requires read-only retained workflow DB")
    store = scratch_store(tmp_path / "store", source)
    c = store.connection
    try:
        with pytest.raises(RuntimeError, match="quiesced"):
            pacing.migrate(c, store.root, pacing.migration_identity())
        assert c.execute("PRAGMA user_version").fetchone()[0] == 15
        pacing.apply(c)
        before = versions(store)
        original = store.get_workflow(JOB)
        payload = json.loads(c.execute("SELECT receipt_json FROM operator_commands WHERE command_id=?", (JOB,)).fetchone()[0])
        original_children = list(payload["execution"]["runtime_state"]["workflow"]["child_command_ids"])
        v = original["command"]["state_version"]
        for index in range(3):
            payload["execution"]["runtime_state"]["workflow"]["held_reason"] = f"offline-boundary-{index}"
            result = store.publish_workflow(JOB, payload=payload)
            assert result["command"]["state_version"] == v + index + 1
            assert result["protocol"] == original["protocol"]
            observation = store.get_workflow_observation(JOB)
            assert observation["execution"]["runtime_state"]["workflow"]["held_reason"] == f"offline-boundary-{index}"
        assert payload["execution"]["runtime_state"]["workflow"]["child_command_ids"] == original_children
        assert versions(store) == before
        store.owner_id = "stale-offline-owner"
        with pytest.raises(ValueError, match="workflow_authority_lost"):
            store.publish_workflow(JOB, payload=payload)
        assert versions(store) == before
        # A control/status change must still create the full historical snapshot.
        with store._transaction() as conn:
            conn.execute("UPDATE operator_commands SET status='ambiguous' WHERE command_id=?", (JOB,))
            conn.execute("UPDATE operator_plane_commands SET status='ambiguous' WHERE command_id=?", (JOB,))
        assert versions(store)[0] == before[0] + 1
        # Restore V15 DDL, then exercise the actual backed-up migration and reopen.
        from bioxp.oem_runtime_store import _OPERATOR_COMMAND_PLANE_TRIGGER_DDL, verify_canonical_runtime_database
        c.execute('DROP TRIGGER ' + pacing.TRIGGER)
        c.execute(next(sql for sql in _OPERATOR_COMMAND_PLANE_TRIGGER_DDL if 'CREATE TRIGGER IF NOT EXISTS ' + pacing.TRIGGER in sql))
        pacing.migrate(c, store.root, pacing.migration_identity())
        assert c.execute("PRAGMA user_version").fetchone()[0] == 16
        verify_canonical_runtime_database(c, full_data_check=True)
        assert versions(store)[0] == before[0] + 1
        reopened = sqlite3.connect(store.root / "bioxp_runtime.db")
        reopened.row_factory = sqlite3.Row
        try:
            verify_canonical_runtime_database(reopened)
            retained = json.loads(reopened.execute("SELECT receipt_json FROM operator_commands WHERE command_id=?", (JOB,)).fetchone()[0])
            assert retained["execution"]["runtime_state"]["workflow"]["held_reason"] == "offline-boundary-2"
        finally:
            reopened.close()
    finally:
        c.close()
