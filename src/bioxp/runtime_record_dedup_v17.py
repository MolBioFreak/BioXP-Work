"""V17: one copy of each command's inputs.

Command version snapshots no longer embed requested/effective inputs: those
live once on operator_commands and readers take them from there. A plane row
whose effective inputs equal its request stores '{}' for effective_json and
readers use the request. The command claim keeps the prepared plan's digest,
not the plan (it lives on the plane request and the WP8 plan row), and WP8
children store '{}' arguments meaning "as in the plan". The deck recovery
trigger accepts these forms. No historical rows are rewritten.
"""
import hashlib
import inspect
import sqlite3
import sys
import time
from pathlib import Path

VERSION = 17
INSERT_TRIGGER = "operator_plane_command_versions_insert"
UPDATE_TRIGGER = "operator_plane_command_versions_update"
_INPUTS = "'requested_inputs_json',NEW.requested_inputs_json,'effective_inputs_json',NEW.effective_inputs_json,"
_EQ = "      AND canonical_json(c.requested_json)=canonical_json(c.effective_json)\n"
_EQ_NEW = "      AND (c.effective_json='{}' OR canonical_json(c.requested_json)=canonical_json(c.effective_json))\n"


def _strip(sql):
    assert sql.count(_INPUTS) == 1
    return sql.replace(_INPUTS, "")


def insert_sql():
    from .oem_runtime_store import _OPERATOR_COMMAND_PLANE_TRIGGER_DDL
    return _strip(next(s for s in _OPERATOR_COMMAND_PLANE_TRIGGER_DDL
                       if "CREATE TRIGGER IF NOT EXISTS " + INSERT_TRIGGER in s))


def update_sql():
    from . import runtime_workflow_pacing_v16 as v16
    return _strip(v16.trigger_sql())


_RECOVERY = (
    (_EQ, _EQ_NEW),
    ("      AND canonical_json(json_extract(a.requested_inputs_json,'$.prepared_plan'))=canonical_json(w.plan_json)\n",
     "      AND (canonical_json(json_extract(a.requested_inputs_json,'$.prepared_plan'))=canonical_json(w.plan_json)\n"
     "           OR json_extract(a.requested_inputs_json,'$.prepared_plan_digest')=w.plan_digest)\n"),
    ("          OR canonical_json(ch.arguments_json) IS NOT canonical_json(json_extract(p.value,'$.arguments'))\n",
     "          OR (ch.arguments_json<>'{}' AND canonical_json(ch.arguments_json) IS NOT canonical_json(json_extract(p.value,'$.arguments')))\n"),
    ("AND child.arguments_json='{\"rehome\":false}'",
     "AND (child.arguments_json='{\"rehome\":false}' OR (child.arguments_json='{}' AND "
     "canonical_json(json_extract(w.plan_json,'$.children[0].arguments'))='{\"rehome\":false}'))"),
)


def recovery_sql():
    from . import oem_deck_recovery_schema_v13 as v13
    sql = v13.SQL
    for old, new in _RECOVERY:
        assert sql.count(old) == 1, old
        sql = sql.replace(old, new)
    return sql


def apply(connection):
    from . import oem_deck_recovery_schema_v13 as v13
    for name, sql in ((INSERT_TRIGGER, insert_sql()), (UPDATE_TRIGGER, update_sql()), (v13.TRIGGER, recovery_sql())):
        connection.execute('DROP TRIGGER "' + name + '"')
        connection.execute(sql)


def migration_identity():
    from .runtime_audit_store import RuntimeMigrationIdentity
    return RuntimeMigrationIdentity(version=VERSION, name='command_record_dedup_v17',
        ddl_sha256=hashlib.sha256((inspect.getsource(sys.modules[__name__]) + insert_sql() + update_sql()
                                   + recovery_sql()).encode()).hexdigest())


def migrate(connection, root: Path, identity):
    from . import oem_runtime_store as owner
    lifecycle = (connection.exclusive_lifecycle() if isinstance(connection, owner.RuntimeLifecycleConnection)
                 else owner.runtime_lifecycle_lock(root, exclusive=True))
    with lifecycle:
        if owner.assert_migration_slot(connection, identity):
            owner.verify_canonical_runtime_database(connection)
            return
        if connection.execute('PRAGMA user_version').fetchone()[0] != 16:
            raise RuntimeError('command record dedup migration requires exact v1-v16 prefix')
        started = time.time()
        connection.execute('BEGIN IMMEDIATE')
        try:
            owner.verify_canonical_runtime_database(connection, version=16)
            if connection.execute("SELECT 1 FROM operator_plane_commands WHERE status IN "
                "('queued','dispatched','issued_pending','stop_requested','abort_requested') LIMIT 1").fetchone():
                raise RuntimeError('command record dedup migration requires quiesced mutation admission')
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
            connection.execute('PRAGMA user_version=17')
            owner.verify_canonical_runtime_database(connection)
            connection.execute('COMMIT')
        except Exception:
            if connection.in_transaction:
                connection.execute('ROLLBACK')
            raise
