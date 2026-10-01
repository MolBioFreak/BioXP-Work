"""V9: observed loaded presence with explicitly unknown ancillary location.

Only the loaded-location NULL prohibition changes. Known locations remain the
OEM integer group/channel domain; every writer, coherence and history fence is
retained. V1-V8 and all retained operational rows are immutable.
"""
from __future__ import annotations
import hashlib
import inspect
import sqlite3
import sys
import time
from pathlib import Path
from .oem_deck_schema_v6 import DECK_SCHEMA_V6_EXTRA_SQL, _statements, _normalized_sql
from .oem_deck_schema_v7 import _canonical_v7_attestation

VERSION = 9
from .oem_deck_schema_v7 import GROUP_TRIGGER_NAME, DECK_SCHEMA_V7_TRIGGER_SQL, _GROUP_LOADED_BOUND
from . import oem_deck_schema_v8 as v8
TRIGGER = GROUP_TRIGGER_NAME
OLD = _GROUP_LOADED_BOUND
NEW = "NEW.tip_loaded=1 AND NEW.tip_location IS NOT NULL AND (typeof(NEW.tip_location)<>'integer' OR NEW.tip_location NOT BETWEEN -1 AND 3)"
if DECK_SCHEMA_V7_TRIGGER_SQL.count(OLD) != 1:
    raise RuntimeError('frozen loaded-tip predicate is not exact')
SQL = DECK_SCHEMA_V7_TRIGGER_SQL.replace(OLD, NEW)


def apply(connection):
    connection.execute('DROP TRIGGER "' + TRIGGER + '"')
    connection.execute(SQL)


def migration_identity():
    from .runtime_audit_store import RuntimeMigrationIdentity
    return RuntimeMigrationIdentity(version=VERSION, name='oem_deck_loaded_unknown_v9',
        ddl_sha256=hashlib.sha256(inspect.getsource(sys.modules[__name__]).encode()).hexdigest())


def verify(connection):
    """Exact SQL attestation of unchanged v8 plus the one v9 trigger.

    Runtime owner additionally attests the complete union schema and migration
    ledger. Exact table SQL retains constraints, indexes and FK definitions.
    """
    from .oem_deck_schema_v6 import _deck_domain_object
    expected, _constraints = _canonical_v7_attestation()
    expected[('trigger', v8.TRIGGER)] = expected[('trigger', v8.TRIGGER)].replace(
        _normalized_sql(v8.OLD), _normalized_sql(v8.NEW))
    expected[('trigger', TRIGGER)] = expected[('trigger', TRIGGER)].replace(
        _normalized_sql(OLD), _normalized_sql(NEW))
    names = {name for kind, name in expected}
    actual = {(str(row[0]), str(row[1])): _normalized_sql(row[3]) for row in connection.execute(
        "SELECT type,name,tbl_name,sql FROM sqlite_master WHERE name NOT LIKE 'sqlite_%'")
        if _deck_domain_object(str(row[1]), str(row[2]), names)}
    if set(actual) != set(expected):
        raise RuntimeError('canonical deck schema v9 object manifest is not exact')
    for identity, sql in expected.items():
        accepted = {hashlib.sha256(sql.encode()).hexdigest()}
        if identity == ('table', 'operator_plane_deck_semantic_state'):
            accepted.update({'216bd31d911a5808693213ba7243e33fd62928af383d17a7694dd2fac78d48ee',
                             'bf7e7c749b21399d3076970b3263649176583c59cf155e66db0cb1572092729b'})
        if hashlib.sha256(actual[identity].encode()).hexdigest() not in accepted:
            raise RuntimeError('canonical deck schema v9 SQL is not exact:' + identity[1])
    if connection.execute('PRAGMA foreign_key_check').fetchone() is not None:
        raise RuntimeError('canonical deck schema v9 foreign-key check failed')


def migrate(connection, root: Path, identity):
    from . import oem_runtime_store as owner
    lifecycle = (connection.exclusive_lifecycle() if isinstance(connection, owner.RuntimeLifecycleConnection)
                 else owner.runtime_lifecycle_lock(root, exclusive=True))
    with lifecycle:
        if owner.assert_migration_slot(connection, identity):
            owner.verify_canonical_runtime_database(connection)
            return
        if connection.execute('PRAGMA user_version').fetchone()[0] != 8:
            raise RuntimeError('scoped deck migration requires exact v1-v8 prefix')
        started = time.time()
        connection.execute('BEGIN IMMEDIATE')
        try:
            owner.verify_canonical_runtime_database(connection, version=8, full_data_check=True)
            if connection.execute("SELECT 1 FROM operator_plane_commands WHERE status IN "
                "('queued','dispatched','issued_pending','stop_requested','abort_requested') LIMIT 1").fetchone():
                raise RuntimeError('scoped deck migration requires quiesced mutation admission')
            backup = sqlite3.connect(root / 'bioxp_runtime.db', timeout=2, isolation_level=None)
            try:
                digest = owner._verified_sqlite_backup(backup, root, lifecycle_lock_held=True)
            finally:
                backup.close()
            apply(connection)
            verify(connection)
            finished = time.time()
            owner._record_runtime_migration(connection, identity=identity, backup_sha256=digest,
                source_digests={}, started_at=started, finished_at=finished)
            connection.execute('UPDATE runtime_store_identity SET schema_version=?,updated_at=? WHERE identity_id=1',
                               (VERSION, finished))
            connection.execute('PRAGMA user_version=9')
            owner.verify_canonical_runtime_database(connection)
            connection.execute('COMMIT')
        except Exception:
            if connection.in_transaction:
                connection.execute('ROLLBACK')
            raise
