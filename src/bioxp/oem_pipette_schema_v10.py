"""V10: bounded issued-order collection-source claim lookup.

An expression index uses SQLite's implicit rowid suffix for issuance order.
No operational rows or V1-V9 objects/identities are changed.
"""
from __future__ import annotations
import hashlib
import inspect
import sqlite3
import sys
import time
from pathlib import Path

VERSION = 10
SQL = "CREATE INDEX pipette_collection_claim_idx ON pipette_operations(json_extract(source_identity_json, '$.collection_source_affecting'))"


def apply(connection):
    connection.execute(SQL)


def migration_identity():
    from .runtime_audit_store import RuntimeMigrationIdentity
    return RuntimeMigrationIdentity(version=VERSION, name='pipette_collection_owner_v10',
        ddl_sha256=hashlib.sha256(inspect.getsource(sys.modules[__name__]).encode()).hexdigest())


def migrate(connection, root: Path, identity):
    from . import oem_runtime_store as owner
    lifecycle = (connection.exclusive_lifecycle() if isinstance(connection, owner.RuntimeLifecycleConnection)
                 else owner.runtime_lifecycle_lock(root, exclusive=True))
    with lifecycle:
        if owner.assert_migration_slot(connection, identity):
            owner.verify_canonical_runtime_database(connection)
            return
        if connection.execute('PRAGMA user_version').fetchone()[0] != 9:
            raise RuntimeError('collection migration requires exact v1-v9 prefix')
        started = time.time()
        connection.execute('BEGIN IMMEDIATE')
        try:
            owner.verify_canonical_runtime_database(connection, version=9, full_data_check=True)
            if connection.execute("SELECT 1 FROM operator_plane_commands WHERE status IN "
                "('queued','dispatched','issued_pending','stop_requested','abort_requested') LIMIT 1").fetchone():
                raise RuntimeError('collection migration requires quiesced mutation admission')
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
            connection.execute('PRAGMA user_version=10')
            owner.verify_canonical_runtime_database(connection, full_data_check=True)
            connection.execute('COMMIT')
        except Exception:
            if connection.in_transaction:
                connection.execute('ROLLBACK')
            raise
