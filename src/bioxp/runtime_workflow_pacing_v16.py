"""Do not snapshot unchanged report inputs for workflow receipt publications.

Receipts are not in command_versions. Their owner still commits every runtime
boundary atomically with the plane version; report history retains admissions,
status/authority/input changes, and terminal outcomes. Existing history is intact.
"""
import hashlib
import inspect
import re
import sqlite3
import sys
import time
from pathlib import Path

VERSION = 16
TRIGGER = "operator_plane_command_versions_update"


def trigger_sql():
    from .oem_runtime_store import _OPERATOR_COMMAND_PLANE_TRIGGER_DDL
    original = next(sql for sql in _OPERATOR_COMMAND_PLANE_TRIGGER_DDL
                    if "CREATE TRIGGER IF NOT EXISTS " + TRIGGER in sql)
    # Freeze earlier migration identities. Only this successor changes the trigger.
    fields = sorted(set(re.findall(r"NEW\.(\w+)", original)) - {"updated_at"})
    changed = " OR ".join(f"OLD.{field} IS NOT NEW.{field}" for field in fields)
    return original.replace("AFTER UPDATE ON operator_commands\n    BEGIN",
        "AFTER UPDATE ON operator_commands\n    WHEN NEW.command_kind <> 'protocol_workflow' OR "
        + changed + "\n    BEGIN")


def apply(connection):
    connection.execute('DROP TRIGGER "' + TRIGGER + '"')
    connection.execute(trigger_sql())


def migration_identity():
    from .runtime_audit_store import RuntimeMigrationIdentity
    return RuntimeMigrationIdentity(version=VERSION, name='workflow_publication_pacing_v16',
        ddl_sha256=hashlib.sha256((inspect.getsource(sys.modules[__name__]) + trigger_sql()).encode()).hexdigest())


def migrate(connection, root: Path, identity):
    from . import oem_runtime_store as owner
    lifecycle = (connection.exclusive_lifecycle() if isinstance(connection, owner.RuntimeLifecycleConnection)
                 else owner.runtime_lifecycle_lock(root, exclusive=True))
    with lifecycle:
        if owner.assert_migration_slot(connection, identity):
            owner.verify_canonical_runtime_database(connection)
            return
        if connection.execute('PRAGMA user_version').fetchone()[0] != 15:
            raise RuntimeError('workflow pacing migration requires exact v1-v15 prefix')
        started = time.time()
        connection.execute('BEGIN IMMEDIATE')
        try:
            owner.verify_canonical_runtime_database(connection, version=15, full_data_check=True)
            if connection.execute("SELECT 1 FROM operator_plane_commands WHERE status IN "
                "('queued','dispatched','issued_pending','stop_requested','abort_requested') LIMIT 1").fetchone():
                raise RuntimeError('workflow pacing migration requires quiesced mutation admission')
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
            connection.execute('PRAGMA user_version=16')
            owner.verify_canonical_runtime_database(connection, full_data_check=True)
            connection.execute('COMMIT')
        except Exception:
            if connection.in_transaction:
                connection.execute('ROLLBACK')
            raise
