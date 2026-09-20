"""Captured image plan, real workflow/abandon/Home/reconcile owners; offline leaves."""
import copy
import json
import os
from pathlib import Path
from contextlib import contextmanager

import pytest
from fastapi.testclient import TestClient
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_live_prepare_connected import connected_motion_wait
from tests.test_workflow_recovery_abandon import abandon
from tests.test_finite_current_reconciliation_f import stopped_failure, finite_home, raw_history
from tests.test_deck_home_recovery import homed_replacement, home_body

CAPTURE = Path('/home/dalab/.hermes/profiles/fresh/robot-audit/deck-command-audit/oem-live-acceptance/closeout/critical-images-recovery/captured-current-rows.json')


def captured():
    return json.loads(CAPTURE.read_text())


@pytest.fixture
def failed_workflow(integrated_rig, monkeypatch):
    rig = integrated_rig
    capture = captured()
    plan = json.loads(capture['rows']['operator_plane_wp8_operations'][0]['plan_json'])
    children = capture['rows']['operator_plane_wp8_children']
    failed = next(c for c in children if c['terminal_state'] == 'ambiguous')
    failed_args = json.loads(failed['arguments_json'])
    # Recompile from the recorded offsets, then compare the WHOLE child plan.
    # No historical authority stamps or plan digests are transplanted.
    payload = rig.payload('critical-images-recovery')
    settings = payload['document']['metadata']['source_settings']
    first_xy = next(c['arguments'] for c in plan['children'] if c['operation'] == 'sourceMoveTo')
    first_z = next(c['arguments'] for c in plan['children'] if c['operation'] == 'sourceMoveZ')
    settings.update(JobName='offline-critical-images', CameraXOffset=first_xy['offset_x']-1895,
        CameraYOffset=first_xy['offset_y']+710, CameraZOffset=first_z['value']-17395)
    rig.provider.bind_oem_snapshot_image(lambda **kw: {'ok': True, 'capture_ok': True, 'artifact_saved': True})
    physical = rig.provider.primitives.oem_move_z
    def uncertain_z(value, **kwargs):
        if value == failed_args['value']:
            raise RuntimeError('isolated recorded critical image Z failure')
        return physical(value, **kwargs)
    monkeypatch.setattr(rig.provider.primitives, 'oem_move_z', uncertain_z)
    job = rig.start(payload)
    done = rig.terminal(job)
    assert done['command']['status'] == 'ambiguous', done
    ids = [job['job_id'], *[r['command_id'] for r in rig.child_rows(job)]]
    assert rig.store.wait_for_command_workers(ids, timeout=5)
    child = rig.store.connection.execute("SELECT w.* FROM operator_plane_wp8_operations w JOIN operator_commands a USING(command_id) WHERE a.parent_command_id=? AND w.operation='critical_item_images'", (job['job_id'],)).fetchone()
    assert child is not None, done
    assert json.loads(child['plan_json'])['children'] == plan['children']
    actual = rig.store.connection.execute('SELECT * FROM operator_plane_wp8_children WHERE command_id=? ORDER BY child_order', (child['command_id'],)).fetchall()
    assert [(c['operation'], c['terminal_state']) for c in actual] == [(c['operation'], c['terminal_state']) for c in children]
    monkeypatch.setattr(rig.provider.primitives, 'oem_move_z', physical)
    return rig, job, done, child['command_id']


def reconcile(fixture):
    app, provider, *_, data = fixture
    return TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))


def test_images_current_reconciliation_preserves_prefix_and_custody(finite_home):
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    before = raw_history(store, [data['parent_id'], data['command_id']])
    custody = store.deck_semantic_state()
    response = reconcile(finite_home)
    assert response.status_code == 200, response.text
    assert store.deck_recovery_blocker() is None
    assert raw_history(store, [data['parent_id'], data['command_id']]) == before
    after = store.deck_semantic_state()
    for key in ('plate_on_gantry', 'movable_plate_locations', 'current_tray', 'tip_loaded', 'tip_dirty', 'tip_location', 'pseudo_z_home'):
        assert after[key] == custody[key]
    assert reconcile(finite_home).json() == response.json()
    decision = json.loads(store.connection.execute('SELECT decision_json FROM operator_plane_deck_recovery_decisions WHERE command_id=?', (data['command_id'],)).fetchone()[0])
    assert decision['finite_current_state']['kind'] == 'wp8_critical_images_current_home_no_tip'
    assert 'position_table_revision' not in decision
    Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.critical-images.json').write_text(json.dumps({'decision': decision, 'history_unchanged': True}, indent=2))


def test_images_recovery_releases_new_queued_finite_child(finite_home):
    from tests.test_abandoned_workflow_admission import test_reconciled_history_releases_real_queued_finite_child
    test_reconciled_history_releases_real_queued_finite_child(finite_home)


def test_exact_captured_history_reaches_current_query_gate(finite_home, monkeypatch):
    """Actual historical rows, not rewritten originals or invented revisions.

    This is historical applicability only. Current Home/query acceptance is
    separately exercised by the connected fixture; no captured state is written.
    """
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    capture = captured(); rows = capture['rows']; cid = capture['command_id']
    command = next(r for r in rows['operator_plane_commands'] if r['command_id'] == cid)
    audit = next(r for r in rows['operator_commands'] if r['command_id'] == cid)
    wp8 = rows['operator_plane_wp8_operations'][0]
    movement = rows['serial206_movement_commands'][0]
    parent_audit = next(r for r in rows['operator_commands'] if r['command_id'] == capture['parent_command_id'])
    parent_plane = next(r for r in rows['operator_plane_commands'] if r['command_id'] == capture['parent_command_id'])
    joined = {**command, 'target': wp8['operation'], 'plan_digest': wp8['plan_digest'],
        'authority_snapshot_digest': wp8['authority_digest'], 'authority_stamps_json': wp8['authority_stamps_json'],
        'plan_json': wp8['plan_json'], 'parent_command_id': audit['parent_command_id'],
        'audit_status': audit['status'], 'issued_sequence': audit['sequence'],
        'expected_board_epochs_json': movement['expected_board_epochs_json'], 'movement_state': movement['state']}
    original = store._transaction
    class CapturedReads:
        def __init__(self, conn): self.conn = conn
        def execute(self, sql, args=()):
            if sql.startswith('SELECT c.status,c.action_id'): return Rows([])
            if sql.startswith('SELECT c.*,w.operation AS target'): return Rows([joined])
            if sql.startswith('SELECT a.status,a.command_kind'): return Rows([{'status': parent_audit['status'], 'command_kind': parent_audit['command_kind'], 'plane_status': parent_plane['status']}])
            if sql.startswith('SELECT requested_inputs_json FROM operator_commands'): return Rows([(audit['requested_inputs_json'],)])
            if sql.startswith('SELECT receipt_json FROM operator_plane_recovery_acknowledgements'):
                return Rows([(r['receipt_json'],) for r in rows['operator_plane_recovery_acknowledgements'] if r['command_id'] == args[0]])
            for table in ('operator_plane_wp8_children', 'operator_plane_delivery_attempts'):
                if sql.startswith('SELECT * FROM '+table): return Rows(rows[table])
            assert sql.startswith('SELECT'), 'historical applicability must not write'
            return self.conn.execute(sql, args)
    @contextmanager
    def captured_reads(*args, **kwargs):
        with original(*args, **kwargs) as conn: yield CapturedReads(conn)
    authority = provider.deck_home_reconciliation_snapshot(expected_generation=provider.generation_provider())
    class ReachedCurrentQuery(Exception): pass
    def current_query(): raise ReachedCurrentQuery()
    from bioxp.oem_deck_catalog import DeckCatalog
    catalog_revision = DeckCatalog.from_position_table(app.state.oem_deck_position_table_provider()).revision
    body = home_body(provider)
    with monkeypatch.context() as historical, pytest.raises(ReachedCurrentQuery):
        historical.setattr(store, '_transaction', captured_reads)
        store.reconcile_deck_recovery(command_id=cid, current_location=None, current_well=None,
            current_authority=authority, current_position_table_revision=authority['position_table_sha256'],
            current_destination_catalog_revision=catalog_revision,
            decision={k: body[k] for k in ('decision_id', 'approved_by', 'reason')},
            approved_home_state=body['approved_home_state'], current_collection_reader=current_query)
    monkeypatch.setattr(store, '_transaction', original)
    Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.captured-applicability.json').write_text(json.dumps({
        'command_id': cid, 'plan_digest': wp8['plan_digest'], 'historical_checks_passed': True,
        'current_state_acceptance': 'separate connected fixture', 'capture_sha256': __import__('hashlib').sha256(CAPTURE.read_bytes()).hexdigest()}, indent=2))


class Rows:
    def __init__(self, rows): self.rows = rows
    def fetchone(self): return self.rows[0] if self.rows else None
    def fetchall(self): return self.rows


@pytest.mark.parametrize('fault', ['missing_ack', 'active_work', 'prefix', 'tail', 'missing_input', 'changed_input', 'claim_input', 'missing_evidence', 'changed_evidence', 'missing_attempt', 'changed_attempt', 'custody'])
def test_images_corrupt_evidence_refuses(finite_home, monkeypatch, fault):
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    before = raw_history(store, [data['parent_id'], data['command_id']])
    original = store._transaction
    semantic_reads = [0]
    class ReadFault:
        def __init__(self, conn): self.conn = conn
        def execute(self, sql, args=()):
            result = self.conn.execute(sql, args)
            if fault == 'missing_ack' and sql.startswith('SELECT receipt_json FROM operator_plane_recovery_acknowledgements'): return Rows([])
            if fault == 'active_work' and sql.startswith('SELECT 1 FROM serial206_command_resources'): return Rows([(1,)])
            if fault in {'missing_input', 'changed_input'} and sql.startswith('SELECT c.*,w.operation AS target'):
                row = dict(result.fetchone()); value = json.loads(row['requested_json'])
                if fault == 'missing_input': value.pop('operation_inputs')
                else: value['operation_inputs'] = {'camera_z_offset': 999}
                row['requested_json'] = json.dumps(value); return Rows([row])
            if fault == 'claim_input' and sql.startswith('SELECT requested_inputs_json FROM operator_commands'): return Rows([('{}',)])
            if fault in {'prefix', 'tail', 'missing_evidence', 'changed_evidence'} and sql.startswith('SELECT * FROM operator_plane_wp8_children'):
                rows = [dict(r) for r in result.fetchall()]
                if fault == 'prefix': rows[0]['terminal_state'] = 'planned'
                elif fault == 'tail': rows[-1]['terminal_state'] = 'completed'
                elif fault == 'missing_evidence': rows[0]['terminal_evidence_json'] = None
                else:
                    e = json.loads(rows[0]['terminal_evidence_json']); e['result']['ok'] = False
                    rows[0]['terminal_evidence_json'] = json.dumps(e)
                return Rows(rows)
            if fault in {'missing_attempt', 'changed_attempt'} and sql.startswith('SELECT * FROM operator_plane_delivery_attempts'):
                rows = [dict(r) for r in result.fetchall()]
                if fault == 'missing_attempt': rows.pop()
                else: rows[-1]['work_identity'] = rows[0]['work_identity']
                return Rows(rows)
            if fault == 'custody' and sql == 'SELECT * FROM operator_plane_deck_semantic_state WHERE singleton=1':
                semantic_reads[0] += 1
                if semantic_reads[0] > 1:
                    row = dict(result.fetchone()); row['plate_on_gantry'] = 'changed-custody'; return Rows([row])
            return result
    @contextmanager
    def faulted(*args, **kwargs):
        with original(*args, **kwargs) as conn: yield ReadFault(conn)
    with monkeypatch.context() as negative:
        negative.setattr(store, '_transaction', faulted)
        response = reconcile(finite_home)
    assert response.status_code == 409, response.text
    assert store.deck_recovery_blocker() == 'deck_recovery_hold'
    assert raw_history(store, [data['parent_id'], data['command_id']]) == before
