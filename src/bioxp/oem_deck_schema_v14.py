"""V14: delivery and background completion use current owners, not history.

Frozen V1-V13 identities and every operational row remain unchanged.
"""
from __future__ import annotations
import hashlib
import inspect
import sqlite3
import sys
import time
from pathlib import Path
from . import oem_deck_schema_v8 as v8
from .oem_deck_schema_v6 import DECK_SCHEMA_V6_EXTRA_SQL, _statements

VERSION = 14
DELIVERY_TRIGGER = v8.TRIGGER
BACKGROUND_TRIGGER = 'operator_plane_wp8_background_tasks_terminal_authority'
DELIVERY_SQL = v8.SQL.replace(v8.NEW, """     AND m.state='dispatched'
     AND deck_owner_authority_current(NEW.ownership_generation,NEW.board_epoch_4,NEW.board_epoch_5)=1""").replace(
    '   JOIN operator_plane_deck_semantic_state semantic ON semantic.singleton=1\n', '')
BACKGROUND_SQL = next(sql for sql in _statements(DECK_SCHEMA_V6_EXTRA_SQL)
    if sql.startswith('CREATE TRIGGER IF NOT EXISTS ' + BACKGROUND_TRIGGER + ' BEFORE')).replace(
    '   JOIN operator_plane_deck_semantic_state semantic ON semantic.singleton=1\n', '').replace(
    """     AND semantic.ownership_generation=attempt.ownership_generation
     AND semantic.board_epoch_4=attempt.board_epoch_4
     AND semantic.board_epoch_5=attempt.board_epoch_5""",
    '     AND deck_owner_authority_current(attempt.ownership_generation,attempt.board_epoch_4,attempt.board_epoch_5)=1')


def apply(connection):
    for name, sql in ((DELIVERY_TRIGGER, DELIVERY_SQL), (BACKGROUND_TRIGGER, BACKGROUND_SQL)):
        connection.execute('DROP TRIGGER "' + name + '"')
        connection.execute(sql)


def migration_identity():
    from .runtime_audit_store import RuntimeMigrationIdentity
    return RuntimeMigrationIdentity(version=VERSION, name='deck_current_owner_delivery_v14',
        ddl_sha256=hashlib.sha256(inspect.getsource(sys.modules[__name__]).encode()).hexdigest())


def migrate(connection, root: Path, identity):
    from . import oem_runtime_store as owner
    lifecycle = (connection.exclusive_lifecycle() if isinstance(connection, owner.RuntimeLifecycleConnection)
                 else owner.runtime_lifecycle_lock(root, exclusive=True))
    with lifecycle:
        if owner.assert_migration_slot(connection, identity):
            owner.verify_canonical_runtime_database(connection)
            return
        if connection.execute('PRAGMA user_version').fetchone()[0] != 13:
            raise RuntimeError('deck current-owner migration requires exact v1-v13 prefix')
        started = time.time()
        connection.execute('BEGIN IMMEDIATE')
        try:
            owner.verify_canonical_runtime_database(connection, version=13, full_data_check=True)
            if connection.execute("SELECT 1 FROM operator_plane_commands WHERE status IN "
                "('queued','dispatched','issued_pending','stop_requested','abort_requested') LIMIT 1").fetchone():
                raise RuntimeError('deck current-owner migration requires quiesced mutation admission')
            backup = sqlite3.connect(root / 'bioxp_runtime.db', timeout=2, isolation_level=None)
            try:
                digest = owner._verified_sqlite_backup(backup, root, lifecycle_lock_held=True)
            finally:
                backup.close()
            apply(connection)
            finished = time.time()
            owner._record_runtime_migration(connection, identity=identity, backup_sha256=digest,
                source_digests={}, started_at=started, finished_at=finished)
            connection.execute('UPDATE runtime_store_identity SET schema_version=?,updated_at=? WHERE identity_id=1',
                               (VERSION, finished))
            connection.execute('PRAGMA user_version=14')
            owner.verify_canonical_runtime_database(connection, full_data_check=True)
            connection.execute('COMMIT')
        except Exception:
            if connection.in_transaction:
                connection.execute('ROLLBACK')
            raise
