"""V12: admit the immutable WP8 first-Park caller to governed decisions.

Named-deck identity checks and all historical operational rows are unchanged.
WP8 catalog columns describe CURRENT observation, never historical admission.
"""
from __future__ import annotations
import hashlib
import inspect
import sqlite3
import sys
import time
from pathlib import Path
from .oem_deck_schema_v6 import DECK_SCHEMA_V6_EXTRA_SQL, _statements

VERSION = 12
TRIGGER = 'operator_plane_deck_recovery_decisions_authorized_insert_v1'
_sources = [s for s in _statements(DECK_SCHEMA_V6_EXTRA_SQL)
            if s.startswith('CREATE TRIGGER IF NOT EXISTS ' + TRIGGER)]
assert len(_sources) == 1
OLD = "  )\nBEGIN SELECT RAISE(ABORT,'deck recovery decision is unauthorized or rebound'); END;"
NEW = """  ) AND NOT EXISTS(
    SELECT 1 FROM operator_plane_commands c
    JOIN operator_plane_wp8_operations w USING(command_id)
    JOIN operator_commands a USING(command_id)
    JOIN operator_commands parent ON parent.command_id=a.parent_command_id
    WHERE c.command_id=NEW.command_id
      AND c.action_id='oem.deck._finite_operation'
      AND c.status IN ('ambiguous','interrupted') AND a.status=c.status
      AND parent.command_kind='protocol_workflow'
      AND parent.status IN ('ambiguous','interrupted')
      AND c.stream_sequence=NEW.stream_sequence
      AND c.dispatch_attempt_id=NEW.dispatch_attempt_id
      AND w.operation='thermal_door' AND w.plan_digest=NEW.plan_digest
      AND w.authority_digest=NEW.authority_snapshot_digest
      AND json_extract(NEW.decision_json,'$.finite_current_state.kind')='wp8_first_park_current_home_no_tip'
      AND json_extract(NEW.decision_json,'$.finite_current_state.parent_command_id')=parent.command_id
      AND json_extract(NEW.decision_json,'$.current_position_table_revision')=NEW.position_table_revision
      AND json_extract(NEW.decision_json,'$.current_destination_catalog_revision')=NEW.destination_catalog_revision
      AND json_extract(NEW.decision_json,'$.approved_home_state.state')='serial206_xyz_referenced_home'
      AND EXISTS(SELECT 1 FROM operator_plane_delivery_attempts d WHERE d.command_id=c.command_id
        AND d.attempt_sequence=json_extract(NEW.decision_json,'$.finite_current_state.attempt_sequence')
        AND d.dispatch_attempt_id=c.dispatch_attempt_id AND d.plan_digest=w.plan_digest
        AND d.work_kind='wp8_child' AND d.work_identity='child:0:parkGantry')
      AND (SELECT COUNT(*) FROM operator_plane_delivery_attempts d WHERE d.command_id=c.command_id)=1
      AND EXISTS(SELECT 1 FROM operator_plane_wp8_children child WHERE child.command_id=c.command_id
        AND child.child_order=0 AND child.operation='parkGantry' AND child.arguments_json='{"rehome":false}'
        AND child.terminal_state='ambiguous')
      AND NOT EXISTS(SELECT 1 FROM operator_plane_wp8_children child WHERE child.command_id=c.command_id
        AND child.child_order>0 AND (child.terminal_state<>'planned' OR child.terminal_evidence_json IS NOT NULL))
      AND NOT EXISTS(SELECT 1 FROM operator_plane_wp8_background_tasks task WHERE task.command_id=c.command_id)
      AND EXISTS(SELECT 1 FROM operator_plane_recovery_acknowledgements r WHERE r.command_id=parent.command_id
        AND r.operation='cancel_pending'
        AND json_extract(r.receipt_json,'$.workflow_custody')='abandoned'
        AND json_extract(r.receipt_json,'$.workflow_command_id')=parent.command_id)
      AND NOT EXISTS(SELECT 1 FROM operator_plane_deck_commands d WHERE d.command_id=c.command_id)
  )
BEGIN SELECT RAISE(ABORT,'deck recovery decision is unauthorized or rebound'); END;"""
assert _sources[0].count(OLD) == 1
SQL = _sources[0].replace(OLD, NEW)


def apply(connection):
    connection.execute('DROP TRIGGER "' + TRIGGER + '"')
    connection.execute(SQL)


def migration_identity():
    from .runtime_audit_store import RuntimeMigrationIdentity
    return RuntimeMigrationIdentity(version=VERSION, name='wp8_current_state_reconciliation_v12',
        ddl_sha256=hashlib.sha256(inspect.getsource(sys.modules[__name__]).encode()).hexdigest())


def migrate(connection, root: Path, identity):
    from . import oem_runtime_store as owner
    lifecycle = (connection.exclusive_lifecycle() if isinstance(connection, owner.RuntimeLifecycleConnection)
                 else owner.runtime_lifecycle_lock(root, exclusive=True))
    with lifecycle:
        if owner.assert_migration_slot(connection, identity):
            owner.verify_canonical_runtime_database(connection)
            return
        if connection.execute('PRAGMA user_version').fetchone()[0] != 11:
            raise RuntimeError('finite reconciliation migration requires exact v1-v11 prefix')
        started = time.time()
        connection.execute('BEGIN IMMEDIATE')
        try:
            owner.verify_canonical_runtime_database(connection, version=11, full_data_check=True)
            if connection.execute("SELECT 1 FROM operator_plane_commands WHERE status IN "
                "('queued','dispatched','issued_pending','stop_requested','abort_requested') LIMIT 1").fetchone():
                raise RuntimeError('finite reconciliation migration requires quiesced mutation admission')
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
            connection.execute('PRAGMA user_version=12')
            owner.verify_canonical_runtime_database(connection, full_data_check=True)
            connection.execute('COMMIT')
        except Exception:
            if connection.in_transaction:
                connection.execute('ROLLBACK')
            raise
