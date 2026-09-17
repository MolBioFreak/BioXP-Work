"""Accepted wake publishes real source model without resetting deck custody."""
import json
import sqlite3
import subprocess
import sys

import pytest

from tests.protocol_v1_integration_fixture import integrated_rig, PhysicalLeafGate
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_v1_integrated_lifecycle import assert_reopened, hooks
from tests.test_workflow_wake_canonical_transition import wake, run


SEMANTIC = 'SELECT * FROM operator_plane_deck_semantic_state WHERE singleton=1'
CHANGED = {'semantic_state_revision', 'producer_operation', 'producer_command_id',
           'ownership_generation', 'board_epoch_4', 'board_epoch_5',
           'transition_provenance_json', 'updated_at'}


def immutable(conn, parent):
    return [list(conn.execute(sql, (parent,)).fetchone()) for sql in (
        'SELECT requested_inputs_json,canonical_request_sha256 FROM operator_commands WHERE command_id=?',
        'SELECT requested_json,effective_json FROM operator_plane_commands WHERE command_id=?')]


def fresh(root):
    code = '''import json,sys
from pathlib import Path
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.services.reference_service import ReferenceStateStore
s=OperatorCommandStore(sys.argv[1])
try:
 print(json.dumps({'semantic':s.deck_semantic_state(),
  'references':ReferenceStateStore(Path(sys.argv[1])/'bioxp_runtime.db').snapshot(('x','y','z','g'))}))
finally: s.stop()
'''
    return json.loads(subprocess.check_output([sys.executable, '-c', code, str(root)],
                                              text=True, timeout=25))


@pytest.mark.parametrize('prior_door', [False, True])
def test_connected_full_wake_exact_publication(integrated_rig, monkeypatch, prior_door):
    rig = integrated_rig
    rig.body_gate = PhysicalLeafGate()
    payload = rig.payload('lineage-' + str(prior_door))
    payload['document']['stages'][0]['actions'].append({
        'action_id': 'after', 'kind': 'oem_operation', 'oem_opcode': 'led',
        'source_occurrence_id': 'source:after', 'params': {'arguments': ['1','1','1']}})
    job = rig.start(payload)
    assert rig.body_gate.entered.wait(8)
    assert rig.control(job, 'early', action='wake', gate_id='missing').status_code == 409
    assert rig.control(job, 'pause', action='pause', mode='deferred').status_code == 200
    rig.body_gate.release.set()
    gate = rig.gate(job, 'deferred_pause')
    gate_id = gate['execution']['runtime_state']['workflow']['gate_id']
    # Explicit source-model fixture input, not homing/reference/setup evidence.
    # Deferred pause is reached and no physical child is in flight.
    with rig.provider._lock:
        state = rig.provider._load_state()
        state['machine_status']['thermal_door_open'] = prior_door
        rig.provider._save_state(state)
    old_inputs = immutable(rig.store.connection, job['job_id'])
    seen = []
    publish = rig.store._publish_deck_owner_state

    def observe(conn, **kwargs):
        if (not kwargs['source_command_id'].startswith('workflow-lifecycle-')
                or ':' in kwargs['source_command_id']):
            return publish(conn, **kwargs)
        before = dict(conn.execute(SEMANTIC).fetchone())
        value = publish(conn, **kwargs)
        after = dict(conn.execute(SEMANTIC).fetchone())
        seen.append((before, after, kwargs))
        return value

    monkeypatch.setattr(rig.store, '_publish_deck_owner_state', observe)
    awake = rig.control(job, 'wake', action='wake', gate_id=gate_id)
    assert awake.status_code == 200, awake.text
    rig.wait(job, lambda row: rig.executors[job['job_id']]._wake_complete is True)
    assert len(seen) == 1
    before, after, published = seen[0]
    assert published['updates'] == {'thermal_door_open': prior_door}
    assert {k:v for k,v in before.items() if k not in CHANGED} == {
        k:v for k,v in after.items() if k not in CHANGED}
    assert after['semantic_state_revision'] == before['semantic_state_revision'] + 1
    wake_result = rig.native_results(job, 'wake_prepare')[0]
    assert wake_result['status'] == 'completed'
    result = wake_result['response']
    assert result['source_prior_door_open'] is prior_door
    assert json.loads(after['transition_provenance_json'])['upstream_source_command_id'] == result['command_id']
    assert after['board_epoch_5'] == result['wake_authority']['native_generation_after']
    refs = result['source_children'][1]['reference_publications']
    assert set(refs) == {'x','y','z','g'}
    assert all(p['published'] is True for p in refs.values())
    assert after['board_epoch_4'] == refs['y']['fence']['board_epoch']
    assert immutable(rig.store.connection, job['job_id']) == old_inputs
    immediate = {'semantic': rig.store.deck_semantic_state(),
                 'references': rig.references.snapshot(('x','y','z','g'))}
    assert all(r['state'] == 'referenced' for r in immediate['references']['rows'].values())
    assert fresh(rig.root) == immediate
    # Same public control is receipt-only; stale new control cannot republish.
    children = rig.child_rows(job)
    repeated = rig.control(job, 'wake', action='wake', gate_id=gate_id)
    assert repeated.status_code == 200 and repeated.json() == awake.json()
    assert rig.control(job, 'stale', action='wake', gate_id='stale').status_code == 409
    assert rig.child_rows(job) == children and len(seen) == 1
    with pytest.raises(ValueError):
        with rig.provider.deck_owner_authority_scope(), rig.store._transaction() as conn:
            rig.store._accept_workflow_wake_initialization(conn, child_id=result['command_id'])
    assert rig.store.deck_semantic_state() == immediate['semantic']
    assert rig.control(job, 'continue', action='continue', gate='deferred_pause',
                       gate_id=gate_id).status_code == 200
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed', done
    assert 'epilogue_park' in hooks(done) and 'script_finally' in hooks(done)
    assert any(row[0] == 'rgb' and row[1] == (1,1,1) for row in rig.trace)
    assert all(r['ok'] is True for r in done['execution']['runtime_state']['action_results'])
    assert_reopened(rig, done)
    assert fresh(rig.root) == {'semantic': rig.store.deck_semantic_state(),
                              'references': rig.references.snapshot(('x','y','z','g'))}
    assert immutable(rig.store.connection, job['job_id']) == old_inputs


@pytest.mark.parametrize('fault', ['after_write', 'changed_owner'])
def test_semantic_publication_failure_rolls_back_whole_acceptance(wake, monkeypatch, fault):
    store = wake.store
    before = dict(store.connection.execute(SEMANTIC).fetchone())
    inputs = immutable(store.connection, 'parent')
    transitions = list(store.connection.execute('SELECT * FROM operator_plane_deck_semantic_transitions'))
    publish = store._publish_deck_owner_state
    executed = []

    def fail(conn, **kwargs):
        if fault == 'changed_owner':
            # Exact owner is revalidated at the semantic publisher boundary.
            wake.stamps['ownership_generation'] = 2
        result = publish(conn, **kwargs)
        executed.append(dict(conn.execute(SEMANTIC).fetchone()))
        if fault == 'after_write':
            raise sqlite3.OperationalError('injected failure after semantic publication')
        return result

    monkeypatch.setattr(store, '_publish_deck_owner_state', fail)
    result = run(wake)
    assert result['status'] == 'failed' and result['ok'] is False
    assert result['source_children'][1]['ok'] is True
    assert all(p['published'] for p in result['source_children'][1]['reference_publications'].values())
    if fault == 'after_write':
        assert len(executed) == 1
        assert executed[0]['semantic_state_revision'] == before['semantic_state_revision'] + 1
        assert 'injected failure' in result['authority_error']
    else:
        assert not executed
        assert 'authority' in result['authority_error']
    assert dict(store.connection.execute(SEMANTIC).fetchone()) == before
    assert list(store.connection.execute('SELECT * FROM operator_plane_deck_semantic_transitions')) == transitions
    assert immutable(store.connection, 'parent') == inputs
    assert json.loads(store.connection.execute(
        "SELECT effective_inputs_json FROM operator_commands WHERE command_id='parent'").fetchone()[0]) == {}
    called = list(wake.called)
    with pytest.raises(ValueError):
        run(wake)
    assert wake.called == called
    with pytest.raises(ValueError):
        run(wake, 'resume_temperature')
    assert store.connection.execute("SELECT authority_write_allowed()").fetchone()[0] == 0
