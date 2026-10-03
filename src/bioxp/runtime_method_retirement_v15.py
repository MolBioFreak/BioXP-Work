"""V15 retires robot batch/XY method storage, not command evidence.

Frozen migration sources stay byte-identical. Restored associations and entire
method records become immutable, read-only evidence; they cannot admit work.
"""
from __future__ import annotations

import hashlib
import inspect
import json
import re
import sqlite3
import sys
import time
from pathlib import Path

VERSION = 15
RETIRED_TABLES = frozenset({
    'operator_plane_methods', 'serial206_movement_methods', 'operator_plane_snapshots',
})
REMOVED_COLUMNS = {
    'operator_plane_commands': {'method_id', 'method_sequence'},
    'serial206_movement_commands': {'method_id', 'method_order', 'parallel_group'},
    'operator_plane_transitions': {'method_id'},
    'operator_plane_idempotency': {'method_id'},
}
ARCHIVE_SQL = """CREATE TABLE runtime_retired_records (
    source_table TEXT NOT NULL,
    record_key TEXT NOT NULL,
    payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
    PRIMARY KEY(source_table,record_key)
) WITHOUT ROWID"""


def _archive(connection, table, where='1'):
    cursor = connection.execute(f'SELECT * FROM "{table}" WHERE {where}')
    columns = [item[0] for item in cursor.description]
    primary = [row[1] for row in sorted(connection.execute(f'PRAGMA table_info("{table}")'), key=lambda r: r[5]) if row[5]]
    for values in cursor:
        record = dict(zip(columns, values))
        key = json.dumps([record[name] for name in primary], separators=(',', ':'))
        connection.execute('INSERT INTO runtime_retired_records VALUES(?,?,?)',
            (table, key, json.dumps(record, sort_keys=True, separators=(',', ':'))))


def apply(connection):
    """Apply within an exclusive transaction with FK enforcement disabled.

    Rebuild only the four affected tables. All unrelated tables, command IDs,
    sequences, timestamps, receipt bytes and version snapshots remain intact.
    """
    objects = list(connection.execute("SELECT type,name,tbl_name,sql FROM sqlite_master WHERE sql IS NOT NULL AND type IN ('trigger','index')"))
    # Drop triggers before copying; restore their exact definitions except the
    # retired feature's triggers and the removed identity-column comparisons.
    for kind, name, table, sql in objects:
        if kind == 'trigger':
            connection.execute(f'DROP TRIGGER "{name}"')
    connection.execute(ARCHIVE_SQL)
    for table in sorted(RETIRED_TABLES):
        _archive(connection, table)
    for table in REMOVED_COLUMNS:
        where = ('method_id IS NOT NULL OR operation_kind IN (\'method\',\'pause\',\'resume\',\'cancel\')'
                 if table == 'operator_plane_idempotency' else 'method_id IS NOT NULL')
        _archive(connection, table, where)
    _archive(connection, 'operator_commands',
             'command_id IN (SELECT command_id FROM operator_plane_commands WHERE method_id IS NOT NULL)')
    # No retired queued batch child may become an ordinary queued action.
    # Keep the original row in the archive and every original version snapshot.
    queued = [row[0] for row in connection.execute(
        "SELECT command_id FROM operator_plane_commands WHERE method_id IS NOT NULL AND status='queued'")]
    retired_at = time.time()
    connection.execute("UPDATE operator_plane_commands SET status='cancelled',version=version+1,terminal_json=?,finished_at=?,updated_at=? WHERE method_id IS NOT NULL AND status='queued'",
        ('{"reason":"method_feature_retired","delivery_attempted":false}', retired_at, retired_at))
    connection.execute("UPDATE serial206_movement_commands SET state='cleared',state_version=state_version+1 WHERE method_id IS NOT NULL AND state='queued'")
    connection.execute("UPDATE operator_commands SET status='cancelled' WHERE status='queued' AND command_id IN (SELECT command_id FROM operator_plane_commands WHERE method_id IS NOT NULL AND status='cancelled')")
    connection.execute("DELETE FROM operator_plane_idempotency WHERE method_id IS NOT NULL OR operation_kind IN ('method','pause','resume','cancel')")
    for table, removed in REMOVED_COLUMNS.items():
        sequence = connection.execute('SELECT seq FROM sqlite_sequence WHERE name=?', (table,)).fetchone()
        sql = connection.execute('SELECT sql FROM sqlite_master WHERE type=\'table\' AND name=?', (table,)).fetchone()[0]
        kept = []
        for line in sql.splitlines():
            if any(re.search(r'\b' + column + r'\b', line) for column in removed):
                continue
            kept.append(line)
        sql = re.sub(r',\s*\)', '\n)', '\n'.join(kept))
        temporary = table + '_v15'
        sql = re.sub(r'CREATE TABLE(?: IF NOT EXISTS)?\s+"?' + table + r'"?', 'CREATE TABLE ' + temporary, sql, count=1, flags=re.I)
        connection.execute(sql)
        columns = [row[1] for row in connection.execute(f'PRAGMA table_info("{table}")') if row[1] not in removed]
        selected = ','.join('"' + column + '"' for column in columns)
        connection.execute(f'INSERT INTO "{temporary}" ({selected}) SELECT {selected} FROM "{table}"')
        connection.execute(f'DROP TABLE "{table}"')
        connection.execute(f'ALTER TABLE "{temporary}" RENAME TO "{table}"')
        if sequence is not None:
            updated = connection.execute('UPDATE sqlite_sequence SET seq=? WHERE name=?', (sequence[0], table))
            if not updated.rowcount:
                connection.execute('INSERT INTO sqlite_sequence(name,seq) VALUES(?,?)', (table, sequence[0]))
    for table in sorted(RETIRED_TABLES):
        connection.execute(f'DROP TABLE "{table}"')
    for kind, name, table, sql in objects:
        if table in RETIRED_TABLES or name in {'operator_plane_commands_method_idx', 'serial206_movement_commands_method_idx'}:
            continue
        if kind == 'index' and table not in REMOVED_COLUMNS:
            continue
        if kind == 'trigger' and table in REMOVED_COLUMNS:
            sql = '\n'.join(line for line in sql.splitlines() if not any(
                re.search(r'\b' + column + r'\b', line) for column in REMOVED_COLUMNS[table]))
        connection.execute(sql)
    # Add a new report version for the explicit no-replay disposition without
    # changing any prior snapshot. Reports after migration see settled children.
    for command_id in queued:
        cursor = connection.execute('SELECT * FROM operator_plane_commands WHERE command_id=?', (command_id,))
        record = dict(zip((item[0] for item in cursor.description), cursor.fetchone()))
        connection.execute(
            'INSERT INTO operator_plane_command_versions(command_id,source_sequence,row_json,versioned_at) VALUES(?,?,?,?)',
            (command_id, record['stream_sequence'], json.dumps(record, sort_keys=True, separators=(',', ':')), retired_at))
    for operation in ('INSERT', 'UPDATE', 'DELETE'):
        connection.execute(f"CREATE TRIGGER runtime_retired_records_no_{operation.lower()} BEFORE {operation} ON runtime_retired_records BEGIN SELECT RAISE(ABORT, 'retired evidence is immutable'); END")


def migration_identity():
    from .runtime_audit_store import RuntimeMigrationIdentity
    return RuntimeMigrationIdentity(version=VERSION, name='robot_method_retirement_v15',
        ddl_sha256=hashlib.sha256(inspect.getsource(sys.modules[__name__]).encode()).hexdigest())


def migrate(connection, root: Path, identity):
    from . import oem_runtime_store as owner
    lifecycle = (connection.exclusive_lifecycle() if isinstance(connection, owner.RuntimeLifecycleConnection)
                 else owner.runtime_lifecycle_lock(root, exclusive=True))
    with lifecycle:
        if owner.assert_migration_slot(connection, identity):
            owner.verify_canonical_runtime_database(connection)
            return
        if connection.execute('PRAGMA user_version').fetchone()[0] != 14:
            raise RuntimeError('method retirement requires exact v1-v14 prefix')
        owner.verify_canonical_runtime_database(connection, version=14, full_data_check=True)
        if connection.execute("SELECT 1 FROM operator_plane_commands WHERE status IN ('dispatched','issued_pending','stop_requested','abort_requested') LIMIT 1").fetchone():
            raise RuntimeError('method retirement migration requires quiesced mutation admission')
        digest = owner._verified_sqlite_backup(connection, root, lifecycle_lock_held=True)
        started = time.time()
        connection.execute('PRAGMA foreign_keys=OFF')
        try:
            connection.execute('BEGIN IMMEDIATE')
            apply(connection)
            finished = time.time()
            owner._record_runtime_migration(connection, identity=identity, backup_sha256=digest,
                source_digests={}, started_at=started, finished_at=finished)
            connection.execute('UPDATE runtime_store_identity SET schema_version=?,updated_at=? WHERE identity_id=1', (VERSION, finished))
            connection.execute('PRAGMA user_version=15')
            owner.verify_canonical_runtime_database(connection, full_data_check=True)
            connection.execute('COMMIT')
        except Exception:
            if connection.in_transaction:
                connection.execute('ROLLBACK')
            raise
        finally:
            connection.execute('PRAGMA foreign_keys=ON')


def command_history(connection, command_id):
    """Return read-only original association/parent evidence for a retired child."""
    key = json.dumps([str(command_id)], separators=(',', ':'))
    row = connection.execute('SELECT payload_json FROM runtime_retired_records WHERE source_table=? AND record_key=?',
        ('operator_plane_commands', key)).fetchone()
    if row is None:
        return None
    original = json.loads(row[0])
    method_key = json.dumps([original['method_id']], separators=(',', ':'))
    parents = {table: json.loads(value) for table, value in connection.execute(
        'SELECT source_table,payload_json FROM runtime_retired_records WHERE source_table IN (?,?) AND record_key=?',
        ('operator_plane_methods', 'serial206_movement_methods', method_key))}
    return {'original_command': original, 'parents': parents}
