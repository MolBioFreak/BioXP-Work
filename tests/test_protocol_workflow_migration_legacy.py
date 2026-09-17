"""V10 retained movement memberships migrate without fabricated canonical claims.

Run only in the offline runner, with BIOXP_WORKFLOW_V10_BASELINE naming an
explicit immutable V10 database. The fixture is never opened for writing.
"""
import hashlib
import json
import os
from pathlib import Path
import sqlite3

import pytest

from bioxp import oem_runtime_store as owner


def _rows_digest(connection, table, columns):
    # Compare original column values, including all historical receipt bytes.
    rows = connection.execute(
        f'SELECT {",".join(chr(34) + c + chr(34) for c in columns)} FROM "{table}"'
    )
    hashes = sorted(hashlib.sha256(repr(tuple(row)).encode()).hexdigest() for row in rows)
    return len(hashes), hashlib.sha256("".join(hashes).encode()).hexdigest()


def _assert_union_owner_deletion():
    # Isolate the real membership DDL from command-history retention triggers:
    # maintenance may authorize parent deletion, but must not orphan resources
    # or erase a membership that still has its other genuine owner.
    with sqlite3.connect(':memory:', isolation_level=None) as connection:
        for table in ('operator_commands', 'serial206_movement_commands'):
            connection.execute(f'CREATE TABLE {table}(command_id TEXT PRIMARY KEY)')
        connection.execute(owner._WORKFLOW_DDL[4].replace('workflow_resource_migration', 'serial206_command_resources'))
        for statement in owner._WORKFLOW_RESOURCE_TRIGGER_DDL:
            connection.execute(statement)
        for first, second in (('operator_commands', 'serial206_movement_commands'),
                              ('serial206_movement_commands', 'operator_commands')):
            for table in (first, second):
                connection.execute(f'INSERT INTO {table} VALUES (?)', ('shared',))
            connection.execute("INSERT INTO serial206_command_resources VALUES ('shared','axis:x')")
            connection.execute(f"DELETE FROM {first} WHERE command_id='shared'")
            assert connection.execute('SELECT COUNT(*) FROM serial206_command_resources').fetchone()[0] == 1
            with pytest.raises(sqlite3.IntegrityError, match='owner is missing'):
                connection.execute(f"UPDATE {second} SET command_id='unknown' WHERE command_id='shared'")
            connection.execute(f"DELETE FROM {second} WHERE command_id='shared'")
            assert connection.execute('SELECT COUNT(*) FROM serial206_command_resources').fetchone()[0] == 0
        with pytest.raises(sqlite3.IntegrityError, match='owner is missing'):
            connection.execute("INSERT INTO serial206_command_resources VALUES ('missing','axis:x')")


def test_genuine_v10_legacy_resources_preserved_and_enforced(tmp_path):
    source = Path(os.environ['BIOXP_WORKFLOW_V10_BASELINE'])
    source_digest = hashlib.sha256(source.read_bytes()).hexdigest()
    root = tmp_path / 'private-v10'
    root.mkdir()
    path = root / 'bioxp_runtime.db'
    with sqlite3.connect(source.as_uri() + '?mode=ro&immutable=1', uri=True) as baseline:
        assert baseline.execute('PRAGMA user_version').fetchone()[0] == 10
        legacy = baseline.execute(
            'SELECT r.command_id,r.resource_key FROM serial206_command_resources r '
            'LEFT JOIN operator_commands c USING(command_id) WHERE c.command_id IS NULL '
            'ORDER BY r.command_id,r.resource_key'
        ).fetchall()
        assert legacy, 'fixture must contain genuine noncanonical movement memberships'
        assert baseline.execute('PRAGMA foreign_key_check').fetchall() == []
        columns = {row[0]: [col[1] for col in baseline.execute(f'PRAGMA table_info("{row[0]}")')]
                   for row in baseline.execute("SELECT name FROM sqlite_master WHERE type='table' "
                                               "AND name NOT LIKE 'sqlite_%'")}
        before = {table: _rows_digest(baseline, table, cols) for table, cols in columns.items()
                  if table not in {'runtime_store_identity', 'runtime_schema_migrations'}}
        ledger = baseline.execute('SELECT * FROM runtime_schema_migrations ORDER BY version').fetchall()
        with sqlite3.connect(path) as copy:
            baseline.backup(copy)

    runtime = owner.OEMRuntimeStore(root)
    try:
        connection = runtime._db
        owner.verify_canonical_runtime_database(connection, full_data_check=True)
        assert connection.execute('PRAGMA user_version').fetchone()[0] == 11
        assert connection.execute('PRAGMA foreign_key_check').fetchall() == []
        after = {table: _rows_digest(connection, table, columns[table]) for table in before}
        assert after == before
        assert [tuple(row) for row in connection.execute(
            'SELECT * FROM runtime_schema_migrations WHERE version<=10 ORDER BY version')] == ledger
        assert connection.execute('SELECT COUNT(*) FROM operator_commands WHERE parent_command_id IS NOT NULL').fetchone()[0] == 0
        assert connection.execute('SELECT workflow_command_id FROM operator_plane_lane').fetchone()[0] is None

        # Authoritative-writer context cannot bypass owner membership, sealing or
        # immutability. No operator/native command is submitted by this test.
        connection.create_function('authority_write_allowed', 0, lambda: 1)
        with pytest.raises(sqlite3.IntegrityError, match='owner is missing'):
            connection.execute('INSERT INTO serial206_command_resources VALUES (?,?)', ('missing-owner', 'axis:x'))
        cid, resource = legacy[0]
        with pytest.raises(sqlite3.IntegrityError):
            connection.execute('UPDATE serial206_command_resources SET command_id=? WHERE command_id=?', ('missing-owner', cid))
        with pytest.raises(sqlite3.IntegrityError):
            connection.execute('DELETE FROM serial206_command_resources WHERE command_id=?', (cid,))
        with pytest.raises(sqlite3.IntegrityError):
            connection.execute('DELETE FROM serial206_movement_commands WHERE command_id=?', (cid,))
        with pytest.raises(sqlite3.IntegrityError):
            connection.execute('UPDATE serial206_movement_commands SET command_id=? WHERE command_id=?', ('renamed-owner', cid))
        sealed = connection.execute('SELECT r.command_id FROM serial206_command_resources r '
                                    'JOIN operator_plane_transitions t USING(command_id) LIMIT 1').fetchone()
        assert sealed is not None
        with pytest.raises(sqlite3.IntegrityError, match='sealed at admission'):
            connection.execute('INSERT INTO serial206_command_resources VALUES (?,?)', (sealed[0], 'r7-extra'))
        assert {table: _rows_digest(connection, table, columns[table]) for table in before} == before
    finally:
        runtime.close()

    # Normal repeat preparation/reopen must attest the same schema, not migrate
    # again, replay work or rewrite retained evidence.
    reopened = owner.OEMRuntimeStore(root)
    try:
        owner.verify_canonical_runtime_database(reopened._db, full_data_check=True)
        assert {table: _rows_digest(reopened._db, table, columns[table]) for table in before} == before
        assert reopened._db.execute('SELECT COUNT(*) FROM runtime_schema_migrations WHERE version=11').fetchone()[0] == 1
    finally:
        reopened.close()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_digest
    _assert_union_owner_deletion()
    evidence = {'source_sha256': source_digest, 'source_version': 10, 'migrated_version': 11,
                'legacy_resource_rows_preserved': len(legacy), 'retained_tables_unchanged': len(before),
                'retained_table_digests': before, 'source_unchanged': True}
    export = os.environ.get('BIOXP_WORKFLOW_EXPORT')
    if export:
        Path(export).write_text(json.dumps(evidence, indent=2))
