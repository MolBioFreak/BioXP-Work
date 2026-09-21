"""V13: current-Home recovery for immutable critical-image finite history.

Extend the canonical decision trigger; retain named-deck and first-Park SQL
verbatim. No historical inputs, plans, custody or outcomes are rewritten.
"""
from __future__ import annotations

import hashlib
import inspect
import sqlite3
import sys
import time
from pathlib import Path

from . import oem_deck_recovery_schema_v12 as previous

VERSION = 13
TRIGGER = previous.TRIGGER
# This is a third authorization branch, not a change to applied V12.
CRITICAL_IMAGES = """  ) AND NOT EXISTS(
    SELECT 1 FROM operator_plane_commands c
    JOIN operator_plane_wp8_operations w USING(command_id)
    JOIN operator_commands a USING(command_id)
    JOIN serial206_movement_commands m USING(command_id)
    JOIN operator_commands parent ON parent.command_id=a.parent_command_id
    JOIN operator_plane_commands pc ON pc.command_id=parent.command_id
    JOIN operator_plane_deck_semantic_state s ON s.singleton=1
    JOIN json_each(json_array(json_extract(NEW.decision_json,'$.finite_current_state'))) f
    WHERE c.command_id=NEW.command_id
      AND c.action_id='oem.deck._finite_operation'
      AND c.status IN ('ambiguous','interrupted') AND a.status=c.status AND m.state=c.status
      AND c.finished_at IS NOT NULL
      AND parent.command_kind='protocol_workflow'
      AND parent.status IN ('ambiguous','interrupted') AND pc.status=parent.status
      AND c.stream_sequence=NEW.stream_sequence AND c.dispatch_attempt_id=NEW.dispatch_attempt_id
      AND w.operation='critical_item_images' AND w.plan_digest=NEW.plan_digest
      AND w.authority_digest=NEW.authority_snapshot_digest
      AND json_extract(f.value,'$.kind')='wp8_critical_images_current_home_no_tip'
      AND json_extract(f.value,'$.parent_command_id')=parent.command_id
      AND json_extract(NEW.decision_json,'$.command_id')=c.command_id
      AND json_extract(NEW.decision_json,'$.stream_sequence')=c.stream_sequence
      AND json_extract(NEW.decision_json,'$.dispatch_attempt_id')=c.dispatch_attempt_id
      AND json_extract(NEW.decision_json,'$.plan_digest')=w.plan_digest
      AND json_extract(NEW.decision_json,'$.authority_snapshot_digest')=w.authority_digest
      AND json_extract(NEW.decision_json,'$.current_position_table_revision')=NEW.position_table_revision
      AND json_extract(NEW.decision_json,'$.current_destination_catalog_revision')=NEW.destination_catalog_revision
      AND json_extract(NEW.decision_json,'$.controller_observation_id')=NEW.controller_observation_id
      AND json_extract(NEW.decision_json,'$.recovery_epoch')=NEW.recovery_epoch
      AND json_extract(NEW.decision_json,'$.approved_home_state.state')='serial206_xyz_referenced_home'
      AND json_type(NEW.decision_json,'$.current_location')='null'
      AND json_type(NEW.decision_json,'$.current_well')='null'
      AND canonical_json(c.requested_json)=canonical_json(c.effective_json)
      AND json_extract(c.requested_json,'$.operation')=w.operation
      AND json_type(c.requested_json,'$.operation_inputs')='object'
      AND canonical_json(json_extract(c.requested_json,'$.prepared_plan'))=canonical_json(w.plan_json)
      AND canonical_json(json_extract(a.requested_inputs_json,'$.prepared_plan'))=canonical_json(w.plan_json)
      AND json_extract(a.requested_inputs_json,'$.operation')=w.operation
      AND canonical_json(json_extract(a.requested_inputs_json,'$.operation_inputs'))=
          canonical_json(json_extract(c.requested_json,'$.operation_inputs'))
      AND json_extract(a.requested_inputs_json,'$.workflow_binding.parent_command_id')=parent.command_id
      AND json_extract(w.plan_json,'$.schema_version')='bioxp.oem_wp8_operation.v1'
      AND json_extract(w.plan_json,'$.operation')=w.operation
      AND json_type(w.plan_json,'$.source_owned')='true'
      AND json_type(w.plan_json,'$.parent_return_allows_background_pending')='false'
      AND json_extract(w.plan_json,'$.plan_digest')=w.plan_digest
      AND sha256_utf8(canonical_json(json_remove(w.plan_json,'$.plan_digest')))=w.plan_digest
      AND json_extract(w.plan_json,'$.authority_digest')=w.authority_digest
      AND canonical_json(json_extract(f.value,'$.historical_authority_stamps'))=canonical_json(w.authority_stamps_json)
      AND json_extract(w.authority_stamps_json,'$.ownership_generation')=c.ownership_generation
      AND json_extract(w.authority_stamps_json,'$.board_epoch_4')=json_extract(m.expected_board_epochs_json,'$."4"')
      AND json_extract(w.authority_stamps_json,'$.board_epoch_5')=json_extract(m.expected_board_epochs_json,'$."5"')
      AND json_type(f.value,'$.ambiguous_child_order')='integer'
      AND json_extract(f.value,'$.ambiguous_child_order')>=0
      AND json_type(w.plan_json,'$.children')='array'
      AND json_array_length(w.plan_json,'$.children')=(SELECT count(*) FROM operator_plane_wp8_children WHERE command_id=c.command_id)
      AND EXISTS(SELECT 1 FROM operator_plane_wp8_children ch WHERE ch.command_id=c.command_id
        AND ch.child_order=json_extract(f.value,'$.ambiguous_child_order') AND ch.terminal_state='ambiguous')
      AND NOT EXISTS(
        SELECT 1 FROM operator_plane_wp8_children ch
        LEFT JOIN json_each(w.plan_json,'$.children') p ON p.key=ch.child_order
        WHERE ch.command_id=c.command_id AND (
          p.key IS NULL OR json_extract(p.value,'$.order') IS NOT ch.child_order
          OR ch.operation IS NOT json_extract(p.value,'$.operation')
          OR canonical_json(ch.arguments_json) IS NOT canonical_json(json_extract(p.value,'$.arguments'))
          OR canonical_json(ch.dependency_order_json) IS NOT CASE WHEN ch.child_order=0 THEN '[]' ELSE json_array(ch.child_order-1) END
          OR canonical_json(json_extract(p.value,'$.depends_on')) IS NOT canonical_json(ch.dependency_order_json)
          OR ch.awaited<>1 OR json_type(p.value,'$.awaited') IS NOT 'true'
          OR ch.ignored_return IS NOT json_extract(p.value,'$.ignored_return')
          OR ch.exception_policy<>'propagate' OR json_extract(p.value,'$.exception_policy') IS NOT 'propagate'
          OR canonical_json(ch.state_mutation_json) IS NOT canonical_json(json_extract(p.value,'$.state_mutation'))
          OR canonical_json(json_extract(p.value,'$.source_condition')) IS NOT '{}'
          OR ch.operation NOT IN ('sourceImageGantryLoad','sourceMoveTo','sourceMoveZ','updateLocation','SnapshotImage')
          OR (ch.operation='sourceImageGantryLoad' AND (json_type(ch.state_mutation_json,'$.pseudo_z_home') IS NULL
              OR (SELECT count(*) FROM json_each(ch.state_mutation_json))<>1))
          OR (ch.operation='updateLocation' AND (json_type(ch.state_mutation_json,'$.current_location') IS NULL
              OR json_type(ch.state_mutation_json,'$.current_well') IS NULL
              OR (SELECT count(*) FROM json_each(ch.state_mutation_json))<>2))
          OR (ch.operation IN ('sourceMoveTo','sourceMoveZ','SnapshotImage') AND canonical_json(ch.state_mutation_json)<>'{}')
          OR (ch.child_order<json_extract(f.value,'$.ambiguous_child_order') AND ch.terminal_state<>'completed')
          OR (ch.child_order>json_extract(f.value,'$.ambiguous_child_order') AND
              (ch.terminal_state<>'planned' OR ch.terminal_evidence_json IS NOT NULL))
        ))
      AND json_extract(f.value,'$.completed_prefix_orders')=(SELECT json_group_array(child_order) FROM (
        SELECT child_order FROM operator_plane_wp8_children WHERE command_id=c.command_id
          AND child_order<json_extract(f.value,'$.ambiguous_child_order') ORDER BY child_order))
      AND json_extract(f.value,'$.unissued_tail_orders')=(SELECT json_group_array(child_order) FROM (
        SELECT child_order FROM operator_plane_wp8_children WHERE command_id=c.command_id
          AND child_order>json_extract(f.value,'$.ambiguous_child_order') ORDER BY child_order))
      AND (SELECT count(*) FROM operator_plane_delivery_attempts WHERE command_id=c.command_id)=json_extract(f.value,'$.ambiguous_child_order')+1
      AND json_extract(f.value,'$.attempt_sequences')=(SELECT json_group_array(attempt_sequence) FROM (
        SELECT attempt_sequence FROM operator_plane_delivery_attempts WHERE command_id=c.command_id ORDER BY attempt_sequence))
      AND json_extract(f.value,'$.attempt_sequence')=(SELECT min(attempt_sequence) FROM operator_plane_delivery_attempts WHERE command_id=c.command_id)
      AND NOT EXISTS(
        SELECT 1 FROM operator_plane_wp8_children ch
        LEFT JOIN operator_plane_delivery_attempts d ON d.command_id=ch.command_id
          AND d.attempt_sequence=json_extract(f.value,'$.attempt_sequences['||ch.child_order||']')
        WHERE ch.command_id=c.command_id AND ch.child_order<=json_extract(f.value,'$.ambiguous_child_order') AND (
          d.attempt_sequence IS NULL OR d.work_kind<>'wp8_child'
          OR d.work_identity IS NOT 'child:'||ch.child_order||':'||ch.operation
          OR d.dispatch_attempt_id IS NOT c.dispatch_attempt_id OR d.plan_digest IS NOT w.plan_digest
          OR d.ownership_generation IS NOT json_extract(w.authority_stamps_json,'$.ownership_generation')
          OR d.board_epoch_4 IS NOT json_extract(w.authority_stamps_json,'$.board_epoch_4')
          OR d.board_epoch_5 IS NOT json_extract(w.authority_stamps_json,'$.board_epoch_5')
          OR d.created_at>=c.finished_at
          OR json_type(ch.terminal_evidence_json,'$.result') IS NOT 'object'
          OR json_type(ch.terminal_evidence_json,'$.terminalized_at') IS NULL
          OR json_type(ch.terminal_evidence_json,'$.terminalized_at') NOT IN ('real','integer')
          OR json_extract(ch.terminal_evidence_json,'$.terminalized_at')<d.created_at
          OR json_extract(ch.terminal_evidence_json,'$.terminalized_at')>c.finished_at
          OR (ch.child_order<json_extract(f.value,'$.ambiguous_child_order') AND (
            json_type(ch.terminal_evidence_json,'$.result.ok') IS NOT 'true'
            OR json_extract(ch.terminal_evidence_json,'$.terminalized_at')>(SELECT created_at FROM operator_plane_delivery_attempts
                WHERE command_id=c.command_id AND attempt_sequence=json_extract(f.value,'$.attempt_sequences['||(ch.child_order+1)||']'))
            OR (ch.operation IN ('sourceMoveTo','sourceMoveZ') AND (
              json_type(ch.terminal_evidence_json,'$.result.controller_completion_verified') IS NOT 'true'
              OR (json_type(ch.terminal_evidence_json,'$.result.controller_command_acknowledged') IS NOT 'true'
                  AND json_type(ch.terminal_evidence_json,'$.result.hardware_postcondition_verified') IS NOT 'true')))
            OR (ch.operation IN ('sourceImageGantryLoad','updateLocation') AND (
              json_extract(ch.terminal_evidence_json,'$.result.published.producer_operation') IS NOT ch.operation
              OR json_extract(ch.terminal_evidence_json,'$.result.published.transition_provenance.upstream_source_command_id')
                IS NOT c.command_id||':'||ch.child_order||':'||w.plan_digest))))
          OR (ch.child_order=json_extract(f.value,'$.ambiguous_child_order') AND (
            json_type(ch.terminal_evidence_json,'$.result.delivery_attempted') IS NOT 'true'
            OR NOT (json_type(ch.terminal_evidence_json,'$.result.ok') IS 'false' OR (
              json_type(ch.terminal_evidence_json,'$.result.ok') IS NULL
              AND json_type(ch.terminal_evidence_json,'$.result.exception_type') IS 'text'
              AND length(json_extract(ch.terminal_evidence_json,'$.result.exception_type'))>0
              AND json_type(ch.terminal_evidence_json,'$.result.exception') IS 'text'
              AND length(json_extract(ch.terminal_evidence_json,'$.result.exception'))>0))))
        ))
      AND NOT EXISTS(SELECT 1 FROM operator_plane_wp8_background_tasks WHERE command_id=c.command_id)
      AND NOT EXISTS(SELECT 1 FROM operator_plane_deck_commands WHERE command_id=c.command_id)
      AND NOT EXISTS(SELECT 1 FROM operator_plane_commands WHERE status IN ('queued','dispatched','issued_pending','stop_requested','abort_requested'))
      AND NOT EXISTS(SELECT 1 FROM operator_plane_lane WHERE workflow_command_id IS NOT NULL)
      AND NOT EXISTS(SELECT 1 FROM operator_plane_wp8_background_tasks WHERE state IN ('issued_pending','running','planned'))
      AND NOT EXISTS(SELECT 1 FROM serial206_command_resources r JOIN operator_commands ac USING(command_id)
        WHERE ac.status IN ('running','issued_pending','interrupting'))
      AND NOT EXISTS(SELECT 1 FROM json_each(json_array(c.command_id,parent.command_id)) member
        WHERE NOT EXISTS(SELECT 1 FROM operator_plane_recovery_acknowledgements r WHERE r.command_id=member.value
          AND r.operation='cancel_pending'
          AND r.recovery_epoch=(SELECT max(recovery_epoch) FROM operator_plane_recovery_acknowledgements
              WHERE command_id=member.value AND operation='cancel_pending')
          AND json_extract(r.receipt_json,'$.workflow_custody')='abandoned'
          AND json_extract(r.receipt_json,'$.workflow_command_id')=parent.command_id
          AND json_extract(r.receipt_json,'$.outcome_remains')='unknown'))
      AND s.producer_operation='governed_deck_reconciliation' AND s.producer_command_id=c.command_id
      AND s.semantic_state_revision=json_extract(f.value,'$.no_tip_semantic_revision')+1
      AND s.tip_loaded=0 AND s.tip_dirty=0 AND s.tip_location=-1
      AND canonical_json(json_extract(s.transition_provenance_json,'$.finite_current_state'))=canonical_json(f.value)
  )
BEGIN SELECT RAISE(ABORT,'deck recovery decision is unauthorized or rebound'); END;"""
assert previous.SQL.count(previous.OLD) == 1
SQL = previous.SQL.replace(previous.OLD, CRITICAL_IMAGES)


def apply(connection):
    connection.execute('DROP TRIGGER "' + TRIGGER + '"')
    connection.execute(SQL)


def migration_identity():
    from .runtime_audit_store import RuntimeMigrationIdentity
    return RuntimeMigrationIdentity(version=VERSION, name='critical_images_current_state_reconciliation_v13',
        ddl_sha256=hashlib.sha256(inspect.getsource(sys.modules[__name__]).encode()).hexdigest())


def migrate(connection, root: Path, identity):
    from . import oem_runtime_store as owner
    lifecycle = (connection.exclusive_lifecycle() if isinstance(connection, owner.RuntimeLifecycleConnection)
                 else owner.runtime_lifecycle_lock(root, exclusive=True))
    with lifecycle:
        if owner.assert_migration_slot(connection, identity):
            owner.verify_canonical_runtime_database(connection)
            return
        if connection.execute('PRAGMA user_version').fetchone()[0] != 12:
            raise RuntimeError('critical images reconciliation migration requires exact v1-v12 prefix')
        started = time.time()
        connection.execute('BEGIN IMMEDIATE')
        try:
            owner.verify_canonical_runtime_database(connection, version=12, full_data_check=True)
            if connection.execute("SELECT 1 FROM operator_plane_commands WHERE status IN "
                "('queued','dispatched','issued_pending','stop_requested','abort_requested') LIMIT 1").fetchone():
                raise RuntimeError('critical images reconciliation migration requires quiesced mutation admission')
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
            connection.execute('PRAGMA user_version=13')
            owner.verify_canonical_runtime_database(connection, full_data_check=True)
            connection.execute('COMMIT')
        except Exception:
            if connection.in_transaction:
                connection.execute('ROLLBACK')
            raise
