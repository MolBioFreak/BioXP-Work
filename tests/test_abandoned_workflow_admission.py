"""Workflow custody after real failed WP8, abandonment and governed recovery."""
import json

import pytest
from fastapi.testclient import TestClient

from tests.test_workflow_recovery_abandon import (
    integrated_rig, retained_rig, installed_retained, query_rig,
    connected_motion_wait, failed_workflow, abandon, snapshot,
)
from tests.test_finite_current_reconciliation_f import stopped_failure, finite_home, raw_history
from tests.test_deck_home_recovery import homed_replacement, home_body


def admission(store, parent, generation, key='independent-after-recovery'):
    # Retain a real compiled workflow bundle; only the new request identity and
    # current generation differ. Exercise the canonical admission owner itself.
    row = store.connection.execute(
        'SELECT requested_inputs_json FROM operator_commands WHERE command_id=?',
        (parent,)).fetchone()
    inputs = json.loads(row[0])
    footprint = inputs.pop('workflow_footprint')
    return dict(command_id=key, idempotency_key=key,
        plan_fingerprint=inputs['plan_fingerprint'], requested_inputs=inputs,
        ownership_generation=generation, resources=footprint['resources'],
        board_epochs=footprint['board_epochs'])


def reconcile(finite_home):
    app, provider, _, _, _, _, data = finite_home
    response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile',
        json=home_body(provider))
    assert response.status_code == 200, response.text
    store = app.state.operator_command_plane.store
    assert store.deck_recovery_blocker() is None
    assert store._acquire_owner()
    assert store._renew_owner()
    # Drive claim_next synchronously, not an invented scheduler. No dispatch
    # thread or physical callback runs for the new admission in these tests.
    store.bind_workflow_dispatcher(lambda claimed: pytest.fail('unexpected physical dispatch'))
    return store, provider, data


def test_unabandoned_failed_workflow_no_longer_busy(failed_workflow):
    rig, job, _, child = failed_workflow
    before = snapshot(rig, [job['job_id'], child])
    request = admission(rig.store, job['job_id'], rig.provider.generation_provider())
    # 2026-09-21: non-live custody history never refuses admission.
    accepted = rig.store.admit_workflow(**request)
    assert accepted['status'] == 'queued'
    assert snapshot(rig, [job['job_id'], child]) == before
    assert rig.store.get_workflow(request['command_id'])['status'] == 'queued'


def test_abandoned_unreconciled_no_longer_holds_new_work(failed_workflow):
    rig, job, _, child = failed_workflow
    before = snapshot(rig, [job['job_id'], child])
    abandon(rig)
    request = admission(rig.store, job['job_id'], rig.provider.generation_provider())
    accepted = rig.store.admit_workflow(**request)
    assert accepted['status'] == 'queued'
    assert rig.store.deck_recovery_blocker() == 'deck_recovery_hold'
    # 2026-09-21: the unresolved abandonment record no longer holds new work.
    claimed = rig.store.claim_next()
    assert claimed is not None and claimed['command_id'] == request['command_id']
    assert rig.store.get_workflow(request['command_id'])['status'] == 'dispatched'
    assert snapshot(rig, [job['job_id'], child]) == before


def test_reconciled_abandoned_history_allows_new_dispatch_and_replay(finite_home):
    store, provider, data = reconcile(finite_home)
    ids = [data['parent_id'], data['command_id']]
    before = raw_history(store, ids)
    request = admission(store, data['parent_id'], provider.generation_provider())
    first = store.admit_workflow(**request)
    assert first['status'] == 'queued'
    assert store.admit_workflow(**request) == first
    with pytest.raises(ValueError, match='idempotency key conflict'):
        store.admit_workflow(**{**request, 'plan_fingerprint': 'different'})
    claimed = store.claim_next()
    assert claimed is not None and claimed['command_id'] == request['command_id']
    assert store.get_workflow(request['command_id'])['status'] == 'dispatched'
    assert store.admit_workflow(**request)['status'] == 'dispatched'
    assert store.connection.execute('SELECT workflow_command_id FROM operator_plane_lane').fetchone()[0] == request['command_id']
    assert raw_history(store, ids) == before
    assert store.get_workflow(data['parent_id'])['status'] == 'ambiguous'


@pytest.mark.parametrize('state', ['queued', 'dispatched'])
def test_new_active_workflow_remains_busy_after_historical_recovery(finite_home, state):
    store, provider, data = reconcile(finite_home)
    request = admission(store, data['parent_id'], provider.generation_provider())
    store.admit_workflow(**request)
    if state == 'dispatched':
        assert store.claim_next()['command_id'] == request['command_id']
    with pytest.raises(ValueError, match='workflow_busy'):
        store.admit_workflow(**{**request, 'command_id': 'second', 'idempotency_key': 'second'})
    assert store.get_workflow('second') is None


def test_new_independent_live_route_completes_after_governed_recovery(finite_home, monkeypatch):
    from bioxp import api
    from bioxp.protocols.models import ProtocolActionKind
    from bioxp.runtime_audit_store import workflow_claim_context
    from tests.test_protocol_workflow_connected import mount_protocol_routes, request_payload, await_job
    app, provider, _, _, _, _, data = finite_home
    store = app.state.operator_command_plane.store
    before = raw_history(store, [data['parent_id'], data['command_id']])
    calls = []
    def leaf(action, state):
        calls.append((action.action_id, workflow_claim_context()))
        return {'ok': True, 'fixture_only': True, 'physical_operation': False}
    monkeypatch.setattr(api, '_protocol_live_handlers', lambda: {ProtocolActionKind.LED: leaf})
    mount_protocol_routes(app)
    payload = request_payload('independent-live-route-after-recovery')
    with TestClient(app) as client:
        response = client.post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
        assert response.status_code == 200, response.text
        response = client.post('/protocol/execute', json=payload)
        assert response.status_code == 202, response.text
        job = response.json()
        app.state.operator_command_plane.start()
        done = await_job(client, job['job_id'], lambda value: value['command']['terminal'])
        assert done['command']['status'] == 'completed', done
        assert [name for name, _ in calls] == ['first', 'second']
        assert all(binding['parent_command_id'] == job['job_id'] for _, binding in calls)
        replay = client.post('/protocol/execute', json=payload)
        assert replay.status_code == 200 and replay.json()['job_id'] == job['job_id']
        assert len(calls) == 2
        assert raw_history(store, [data['parent_id'], data['command_id']]) == before


@pytest.mark.parametrize('evidence', ['abandonment', 'decision'])
def test_missing_canonical_disposition_no_longer_blocks(finite_home, monkeypatch, evidence):
    # Hide evidence only from the consumer read; never edit immutable history.
    from contextlib import contextmanager
    store, provider, data = reconcile(finite_home)
    request = admission(store, data['parent_id'], provider.generation_provider())
    if evidence == 'decision':
        store.admit_workflow(**request)
    original = store._transaction
    class MissingEvidence:
        def __init__(self, conn): self.conn = conn
        def execute(self, sql, args=()):
            if sql.startswith('SELECT 1 FROM operator_commands c'):
                if evidence == 'abandonment':
                    sql = sql.replace("r.operation='cancel_pending'", '0')
                else:
                    sql = sql.replace('WHERE d.command_id=c.command_id', 'WHERE 0 AND d.command_id=c.command_id')
            return self.conn.execute(sql, args)
    @contextmanager
    def read_fault(*args, **kwargs):
        with original(*args, **kwargs) as conn:
            yield MissingEvidence(conn)
    monkeypatch.setattr(store, '_transaction', read_fault)
    if evidence == 'abandonment':
        # 2026-09-21: missing abandonment evidence no longer refuses admission.
        accepted = store.admit_workflow(**request)
        assert accepted['status'] == 'queued'
    else:
        # 2026-09-21: dispatch is not gated on the decision record.
        claimed = store.claim_next()
        assert claimed is not None and claimed['command_id'] == request['command_id']
        assert store.get_workflow(request['command_id'])['status'] == 'dispatched'


def test_reconciled_history_releases_real_queued_finite_child(finite_home):
    from tests.test_protocol_workflow_connected import mount_protocol_routes, request_payload, await_job
    app, provider, _, _, _, _, data = finite_home
    store = app.state.operator_command_plane.store
    ids = [data['parent_id'], data['command_id']]
    before = raw_history(store, ids)
    old_movement = dict(store.connection.execute(
        'SELECT * FROM serial206_movement_commands WHERE command_id=?', (ids[1],)).fetchone())
    mount_protocol_routes(app)
    # Native Close on the fixture's already-closed door is a genuine finite
    # source noop. It still must claim the same resources through the live
    # handler, durable finite queue and dispatcher; host-only work cannot prove it.
    payload = request_payload('independent-finite-after-recovery', actions=[
        {'action_id': 'new-door', 'kind': 'thermal_door', 'params': {'door_command': 'DC'}}])
    with TestClient(app) as client:
        assert store.deck_recovery_blocker() == 'deck_recovery_hold'
        assert store.claim_next() is None
        response = client.post('/operator/recovery/deck/'+ids[1]+'/reconcile', json=home_body(provider))
        assert response.status_code == 200, response.text
        assert store.deck_recovery_blocker() is None
        response = client.post('/protocol/execute', json=payload)
        assert response.status_code == 202, response.text
        job = response.json()
        app.state.operator_command_plane.start()
        done = await_job(client, job['job_id'], lambda value: value['command']['terminal'])
        assert done is not None and done['command']['status'] == 'completed', done
        children = store.connection.execute(
            'SELECT c.command_id,c.status,c.action_id,m.state FROM operator_commands c '
            'JOIN serial206_movement_commands m USING(command_id) WHERE c.parent_command_id=?',
            (job['job_id'],)).fetchall()
        assert len(children) == 1, [dict(row) for row in children]
        child = children[0]
        assert child['action_id'] == 'oem.deck._finite_operation'
        assert child['status'] == child['state'] == 'completed'
        states = [row[0] for row in store.connection.execute(
            'SELECT state FROM operator_plane_transitions WHERE command_id=? ORDER BY transition_sequence',
            (child['command_id'],))]
        assert 'queued' in states and 'dispatched' in states and 'completed' in states, states
        native = store.connection.execute(
            'SELECT plan_json FROM operator_plane_wp8_operations WHERE command_id=?',
            (child['command_id'],)).fetchone()
        assert json.loads(native[0])['source_noop'] is True
        assert raw_history(store, ids) == before
        assert dict(store.connection.execute(
            'SELECT * FROM serial206_movement_commands WHERE command_id=?', (ids[1],)).fetchone()) == old_movement
