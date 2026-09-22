"""F: current-state finite consumer; actual workflow/query/Home owners, offline leaves."""
import json
import os
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_live_prepare_connected import connected_motion_wait
from tests.test_workflow_recovery_abandon import failed_workflow, abandon, snapshot
from tests.test_deck_home_recovery import homed_replacement, home_body


@pytest.fixture
def stopped_failure(failed_workflow, monkeypatch):
    from tests.test_deck_postmove_reference import USBLeaf
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    rig, job, done, child = failed_workflow
    abandon(rig)
    leaf = USBLeaf()
    primitive = rig.provider.primitives.observations
    # Integrated workflow's narrow lambda lacks the named latch-reader contract.
    # Reuse this physical fixture's existing complete latch reply for recovery.
    def deck_io_query_type(kind):
        assert kind == 3
        return primitive.query_latch()
    monkeypatch.setattr(primitive, 'deck_io_query_type', deck_io_query_type)
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=rig.provider.generation_provider, reference_store=rig.references)
    monkeypatch.setattr(primitive, '_read_axis_position', adapter._read_axis_position)
    monkeypatch.setattr(primitive, 'read_deck_semantic_observation', adapter.read_deck_semantic_observation, raising=False)
    return rig.app, rig.provider, primitive, rig.references, rig.root, leaf, {
        'command_id': child, 'parent_id': job['job_id']}


@pytest.fixture
def finite_home(homed_replacement):
    from bioxp import api
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    app.add_api_route('/liquid/tip-status', api.liquid_tip_status, methods=['POST'])
    app.middleware('http')(api.bind_direct_pipette_idempotency)
    client = TestClient(app)
    query = client.post('/liquid/tip-status', headers={'Idempotency-Key': 'F-post-failure-no-tip'})
    assert query.status_code == 200, query.text
    assert query.json().get('deck_state_publication', {}).get('status') == 'published', query.text
    data['no_tip_query'] = query.json()
    store = app.state.operator_command_plane.store
    collection = provider._park_collection_state()
    Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.F-query-debug.json').write_text(json.dumps({
        'collection': collection, 'query': query.json(),
        'stored': dict(store.connection.execute('SELECT * FROM pipette_operations WHERE command_id=?', (collection['command_id'],)).fetchone()),
        'failed': dict(store.connection.execute('SELECT * FROM operator_plane_commands WHERE command_id=?', (data['command_id'],)).fetchone())}, indent=2))
    yield homed_replacement


def raw_history(store, ids):
    return {t: [dict(r) for r in store.connection.execute('SELECT * FROM '+t+' WHERE command_id IN (?,?) ORDER BY command_id', ids)]
        for t in ('operator_commands', 'operator_plane_commands', 'operator_plane_wp8_operations',
                  'operator_plane_wp8_children', 'operator_plane_delivery_attempts')}


def test_finite_current_state_home_no_tip_preserves_unknown(finite_home):
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    ids = [data['parent_id'], data['command_id']]
    before = raw_history(store, ids)
    response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert response.status_code == 200, response.text
    assert raw_history(store, ids) == before
    assert store.deck_recovery_blocker() is None
    semantic = store.deck_semantic_state()
    again = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert again.status_code == 200 and again.json() == response.json(), again.text
    assert store.deck_semantic_state() == semantic and raw_history(store, ids) == before
    changed = home_body(provider); changed['reason'] = 'different decision identity'
    conflict = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=changed)
    assert conflict.status_code == 409 and 'identity conflict' in conflict.text
    assert store.connection.execute('SELECT 1 FROM operator_plane_deck_commands WHERE command_id=?', (ids[1],)).fetchone() is None
    row = dict(store.connection.execute('SELECT * FROM operator_plane_deck_recovery_decisions WHERE command_id=?', (ids[1],)).fetchone())
    decision = json.loads(row['decision_json'])
    assert decision['finite_current_state']['parent_command_id'] == ids[0]
    assert decision['finite_current_state']['collection']['command_id'] == data['no_tip_query']['command_id']
    assert 'position_table_revision' not in decision and 'destination_catalog_revision' not in decision
    assert row['position_table_revision'] == decision['current_position_table_revision']
    assert row['destination_catalog_revision'] == decision['current_destination_catalog_revision']
    for field in ('plan_digest', 'authority_snapshot_digest', 'dispatch_attempt_id', 'position_table_revision'):
        forged = dict(row); forged['decision_id'] = 'forged-'+field; forged[field] = 'f'*64
        forged_decision = dict(decision); forged_decision['decision_id'] = forged['decision_id']
        forged['decision_json'] = json.dumps(forged_decision, sort_keys=True, separators=(',', ':'))
        with pytest.raises(sqlite3.IntegrityError, match='unauthorized or rebound'):
            with store._transaction() as conn, store._authority_write():
                conn.execute('INSERT INTO operator_plane_deck_recovery_decisions('+','.join(forged)+') VALUES('+','.join('?' for _ in forged)+')', tuple(forged.values()))
    assert raw_history(store, ids) == before
    code = 'import json,sys; from bioxp.operator_command_plane import OperatorCommandStore; from tests.test_finite_current_reconciliation_f import raw_history; s=OperatorCommandStore(sys.argv[1]); print(json.dumps({"hold":s.deck_recovery_blocker(),"history":raw_history(s,[sys.argv[3],sys.argv[2]]),"decision":dict(s.connection.execute("SELECT * FROM operator_plane_deck_recovery_decisions WHERE command_id=?",(sys.argv[2],)).fetchone())}));s.stop()'
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', code, str(root), ids[1], ids[0]], text=True))
    assert reopened == {'hold': None, 'decision': row, 'history': before}
    Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.finite-current-state.json').write_text(json.dumps({
        'history_unchanged': True, 'decision': decision, 'receipt': response.json(), 'reopened': reopened}, indent=2))


def _effective_inputs_read_fault(store, transform):
    """Fault ONLY the consumer's read view of the WP8 child effective inputs."""
    from contextlib import contextmanager
    original = store._transaction

    class Rows:
        def __init__(self, rows): self.rows = rows
        def fetchone(self): return self.rows[0] if self.rows else None
        def fetchall(self): return self.rows

    class ReadFault:
        def __init__(self, conn): self.conn = conn
        def execute(self, sql, args=()):
            result = self.conn.execute(sql, args)
            if sql.startswith('SELECT c.*,w.operation AS target'):
                row = dict(result.fetchone())
                effective = json.loads(row['effective_json'])
                transform(effective)
                row['effective_json'] = json.dumps(effective)
                return Rows([row])
            return result

    @contextmanager
    def faulted(*args, **kwargs):
        with original(*args, **kwargs) as conn:
            yield ReadFault(conn)
    return faulted


def test_finite_historical_missing_prepared_plan_is_admitted(finite_home, monkeypatch):
    # Historical first-Park rows (e.g. the 2026-09-21 thermal-door child) were
    # dispatched without the workflow binding of the WP8 plan into effective
    # inputs.  The reconcile gate must align with the v12 authorization trigger
    # and admit them when every other immutable-identity proof holds, instead
    # of freezing the retained ambiguity forever.
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    ids = [data['parent_id'], data['command_id']]
    before = raw_history(store, ids)
    monkeypatch.setattr(store, '_transaction',
        _effective_inputs_read_fault(store, lambda effective: effective.pop('prepared_plan', None)))
    response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert response.status_code == 200, response.text
    row = dict(store.connection.execute('SELECT * FROM operator_plane_deck_recovery_decisions WHERE command_id=?', (ids[1],)).fetchone())
    decision = json.loads(row['decision_json'])
    assert decision['finite_current_state']['kind'] == 'wp8_first_park_current_home_no_tip'
    assert decision['finite_current_state']['parent_command_id'] == ids[0]
    assert store.deck_recovery_blocker() is None
    assert raw_history(store, ids) == before


def test_finite_mismatched_prepared_plan_still_refuses(finite_home, monkeypatch):
    # A plan that IS carried must still match the immutable WP8 plan exactly.
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    ids = [data['parent_id'], data['command_id']]
    before = raw_history(store, ids)
    monkeypatch.setattr(store, '_transaction',
        _effective_inputs_read_fault(store, lambda effective: effective.__setitem__('prepared_plan', {'tampered': True})))
    response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert response.status_code == 409 and 'immutable identity' in response.text, response.text
    assert store.connection.execute('SELECT 1 FROM operator_plane_deck_recovery_decisions WHERE command_id=?', (ids[1],)).fetchone() is None
    assert store.deck_recovery_blocker() == 'deck_recovery_hold'
    assert raw_history(store, ids) == before


@pytest.mark.parametrize('fault', ['collection_owner', 'collection_interrupt', 'loaded', 'unrelated_publication', 'final_collection', 'active_worker'])
def test_finite_current_no_tip_fences(finite_home, monkeypatch, fault):
    from bioxp import api
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    before = raw_history(store, [data['parent_id'], data['command_id']])
    transport = api._get_pipette_transport()
    if fault == 'collection_interrupt': monkeypatch.setattr(transport, '_interrupt_epoch', transport._interrupt_epoch + 1)
    elif fault == 'collection_owner':
        reader = provider._park_collection_state
        def wrong():
            value = reader(); value['command_id'] = 'unrelated-query'; return value
        monkeypatch.setattr(provider, '_park_collection_state', wrong)
    elif fault == 'loaded': provider.publish_pipette_owner_state(tip_loaded=True, tip_dirty=True, tip_location=0, source_command_id='later-loaded-owner')
    elif fault == 'unrelated_publication': provider.publish_pipette_owner_state(tip_loaded=False, tip_dirty=False, tip_location=-1, source_command_id='unrelated-no-tip-owner')
    elif fault == 'active_worker': store.bind_workflow_controls(data['parent_id'], lambda *a: None)
    else:
        read = provider._park_collection_state
        calls = [0]
        def drift():
            value = read(); calls[0] += 1
            if calls[0] == 2: value['receipt_id'] = 'changed'
            return value
        monkeypatch.setattr(provider, '_park_collection_state', drift)
    try:
        response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
        assert response.status_code == 409, response.text
        expected = {'collection_owner': 'post-failure native no-tip query',
            'collection_interrupt': 'pipette_collection_reader_or_stop_changed',
            'loaded': 'linked current canonical no-tip publication',
            'unrelated_publication': 'linked current canonical no-tip publication',
            'final_collection': 'current collection changed', 'active_worker': 'active or pending work'}
        assert expected[fault] in response.text, response.text
        assert raw_history(store, [data['parent_id'], data['command_id']]) == before
        assert store.deck_recovery_blocker() == 'deck_recovery_hold'
        assert store.connection.execute('SELECT 1 FROM operator_plane_deck_recovery_decisions WHERE command_id=?',(data['command_id'],)).fetchone() is None
    finally:
        if fault == 'active_worker': store.unbind_workflow_controls(data['parent_id'])


@pytest.mark.parametrize('fault,reason', [
    ('missing_abandonment', 'explicit terminal parent abandonment'),
    ('parent_identity', 'immutable identity'), ('native_plan', 'immutable identity'),
    ('issued_order', 'issued delivery identity'), ('dispatch_identity', 'issued delivery identity'),
    ('door_touched', 'untouched door children'), ('missing_attempt', 'issued delivery identity'),
    ('old_query', 'post-failure native no-tip query'),
    ('pending_claim', 'active or pending work'),
    ('query_issued_before_failure', 'post-failure native no-tip query'),
])
def test_finite_corrupt_or_missing_evidence_refuses(finite_home, monkeypatch, fault, reason):
    # Fault only the consumer's SQL read view. Never disable a trigger or edit
    # immutable history to arrange a negative control.
    from contextlib import contextmanager
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    before = raw_history(store, [data['parent_id'], data['command_id']])
    original = store._transaction
    class Rows:
        def __init__(self, rows): self.rows = rows
        def fetchone(self): return self.rows[0] if self.rows else None
        def fetchall(self): return self.rows
    class ReadFault:
        def __init__(self, conn): self.conn = conn
        def execute(self, sql, args=()):
            result = self.conn.execute(sql, args)
            if fault == 'pending_claim' and sql.startswith('SELECT 1 FROM serial206_command_resources r JOIN operator_commands'):
                return Rows([(1,)])
            if fault == 'query_issued_before_failure' and sql.startswith('SELECT sequence FROM operator_commands'):
                return Rows([(1,)])
            if fault == 'missing_abandonment' and sql.startswith('SELECT receipt_json FROM operator_plane_recovery_acknowledgements'):
                return Rows([])
            if fault in {'parent_identity', 'native_plan'} and sql.startswith('SELECT c.*,w.operation AS target'):
                row = dict(result.fetchone())
                if fault == 'parent_identity': row['parent_command_id'] = 'unrelated-parent'
                else:
                    plan = json.loads(row['plan_json']); plan['source_owned'] = False
                    row['plan_json'] = json.dumps(plan)
                return Rows([row])
            if fault == 'door_touched' and sql.startswith('SELECT * FROM operator_plane_wp8_children'):
                rows = [dict(r) for r in result.fetchall()]; rows[1]['terminal_state'] = 'completed'
                return Rows(rows)
            if fault in {'issued_order', 'dispatch_identity', 'missing_attempt'} and sql.startswith('SELECT * FROM operator_plane_delivery_attempts'):
                rows = [dict(r) for r in result.fetchall()]
                if fault == 'missing_attempt': rows = []
                elif fault == 'issued_order': rows[0]['work_identity'] = 'child:1:setDoorStallThresholdPlus2'
                else: rows[0]['dispatch_attempt_id'] = 'unrelated-dispatch'
                return Rows(rows)
            if fault == 'old_query' and sql.startswith('SELECT * FROM pipette_operations'):
                row = dict(result.fetchone()); row['created_at'] = 1.0; return Rows([row])
            return result
    @contextmanager
    def faulted(*args, **kwargs):
        with original(*args, **kwargs) as conn: yield ReadFault(conn)
    monkeypatch.setattr(store, '_transaction', faulted)
    response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert response.status_code == 409 and reason in response.text, response.text
    assert raw_history(store, [data['parent_id'], data['command_id']]) == before
    assert store.deck_recovery_blocker() == 'deck_recovery_hold'


@pytest.mark.parametrize('fault', ['reference', 'owner', 'nonzero', 'speed', 'latch'])
def test_finite_keeps_current_home_fences(finite_home, monkeypatch, fault):
    from bioxp.services.reference_service import MarkAxisDesyncedCommand
    app, provider, primitive, refs, root, leaf, data = finite_home
    store = app.state.operator_command_plane.store
    if fault == 'reference': refs.mark_desynced(MarkAxisDesyncedCommand('z', reason='F-negative'))
    elif fault == 'owner': provider._home_recovery_owner_id = 'unrelated-owner'
    elif fault == 'nonzero': leaf.positions[5, 0] = 1
    elif fault == 'speed': leaf.motor_get_speed = lambda *a, **k: {'ok': True, 'speed': 1, 'ack': {'status': 100}}
    else: monkeypatch.setattr(primitive, 'query_latch', lambda: {'ok': False, 'value': 0})
    response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert response.status_code == 409, response.text
    assert store.deck_recovery_blocker() == 'deck_recovery_hold'
    assert store.connection.execute('SELECT 1 FROM operator_plane_deck_recovery_decisions WHERE command_id=?', (data['command_id'],)).fetchone() is None
