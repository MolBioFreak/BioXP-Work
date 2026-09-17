"""Real deferred wake -> native refs -> semantic publication -> full epilogue.

Only shared physical I/O leaves are doubled. Wrappers below observe the real
accepted-wake boundary; no native, provider, lifecycle or store success doubles.
"""
import json
import subprocess
import sys

import pytest
from tests.protocol_v1_integration_fixture import integrated_rig, PhysicalLeafGate
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_v1_integrated_lifecycle import assert_reopened, hooks
from tests.test_wake_lineage_semantic_publication import state, immutable, METADATA


def fresh_semantics_refs(root):
    code = '''import json,sys
from pathlib import Path
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.services.reference_service import ReferenceStateStore
store=OperatorCommandStore(sys.argv[1])
try:
 print(json.dumps({'semantic':dict(store.connection.execute('SELECT * FROM operator_plane_deck_semantic_state').fetchone()),'projection':store.deck_semantic_state(),'refs':ReferenceStateStore(Path(sys.argv[1])/'bioxp_runtime.db').snapshot(('x','y','z','g'))}))
finally:
 store.stop()
'''
    return json.loads(subprocess.check_output([sys.executable, '-c', code, str(root)], text=True, timeout=25))


@pytest.mark.parametrize('prior', [False, True])
def test_connected_wake_preserves_lineage_through_successful_park(integrated_rig, monkeypatch, prior):
    rig = integrated_rig
    rig.body_gate = PhysicalLeafGate()
    payload = rig.payload('lineage-' + str(prior))
    payload['document']['stages'][0]['actions'].append({
        'action_id': 'after', 'kind': 'oem_operation', 'oem_opcode': 'led',
        'source_occurrence_id': 'source:after', 'params': {'arguments': ['1','1','1']}})
    job = rig.start(payload)
    assert rig.body_gate.entered.wait(8)
    assert rig.control(job, 'early', action='wake', gate_id='not-reached').status_code == 409
    assert rig.control(job, 'pause', action='pause', mode='deferred').status_code == 200
    rig.body_gate.release.set()
    gate = rig.gate(job, 'deferred_pause')
    gate_id = gate['execution']['runtime_state']['workflow']['gate_id']
    # Explicit source software-model fixture condition at the reached gate.
    # This is not physical door evidence or a native method replacement.
    with rig.provider._lock:
        model = rig.provider._load_state()
        model['machine_status']['thermal_door_open'] = prior
        rig.provider._save_state(model)
    rig.store.publish_deck_owner_state(source_operation='updateThermalDoorOpen',
        source_command_id='lineage-test-model:' + str(prior),
        updates={'thermal_door_open': prior}, **rig.provider.deck_owner_authority_stamps())
    conn = rig.store.connection
    before = state(conn)
    parent_identity = immutable(conn, job['job_id'])
    plane_identity = tuple(conn.execute('SELECT requested_json,effective_json FROM operator_plane_commands WHERE command_id=?', (job['job_id'],)).fetchone())
    generation = rig.native.tester.oem_current_board_lifecycle_generation()
    observed = []
    accept = rig.store._accept_workflow_wake_initialization
    def observe(tx, *, child_id):
        before_publish = state(tx)
        child_identity = immutable(tx, child_id)
        refs = rig.references.snapshot(('x', 'y', 'z', 'g'))
        accept(tx, child_id=child_id)
        after_publish = state(tx)
        assert immutable(tx, child_id) == child_identity
        assert rig.references.snapshot(('x', 'y', 'z', 'g')) == refs
        observed.append((before_publish, after_publish, refs, child_id))
    monkeypatch.setattr(rig.store, '_accept_workflow_wake_initialization', observe)
    stale = rig.control(job, 'stale', action='wake', gate_id='old-gate')
    assert stale.status_code == 409
    awake = rig.control(job, 'wake', action='wake', gate_id=gate_id)
    assert awake.status_code == 200, awake.text
    rig.wait(job, lambda row: rig.executors[job['job_id']]._wake_complete is True)
    assert len(observed) == 1
    before_publish, after_publish, refs, child_id = observed[0]
    assert before_publish == before
    assert {k:v for k,v in after_publish.items() if k not in METADATA} == {k:v for k,v in before.items() if k not in METADATA}
    assert after_publish['semantic_state_revision'] == before['semantic_state_revision'] + 1
    assert (after_publish['board_epoch_4'], after_publish['board_epoch_5']) == (before['board_epoch_4'] + 1, generation + 1)
    proof = json.loads(after_publish['transition_provenance_json'])
    assert proof['updates'] == {'thermal_door_open': prior}
    assert proof['upstream_source_command_id'] == child_id
    wake = rig.native_results(job, 'wake_prepare')
    assert len(wake) == 1 and wake[0]['status'] == 'completed', wake
    raw = wake[0]['response']
    assert raw['source_prior_door_open'] is prior
    assert raw['source_children'][0]['initial_check']['board_lifecycle_generation']['source_order'] == ['cmd64=0', 'cmd64=1']
    assert all(pub['published'] for pub in raw['source_children'][1]['reference_publications'].values())
    assert refs['durable_clean'] and all(r['state'] == 'referenced' for r in refs['rows'].values())
    assert rig.native.tester.oem_current_board_lifecycle_generation() == generation + 1
    assert ('restore_door_model' in rig.control_chain(job)) is prior
    assert rig.native_results(job, 'resume_temperature')[0]['response']['ok'] is True
    immediate = {'semantic':state(conn), 'projection':rig.store.deck_semantic_state(),
                 'refs':rig.references.snapshot(('x', 'y', 'z', 'g'))}
    assert fresh_semantics_refs(rig.root) == immediate
    assert immutable(conn, job['job_id']) == parent_identity
    assert tuple(conn.execute('SELECT requested_json,effective_json FROM operator_plane_commands WHERE command_id=?', (job['job_id'],)).fetchone()) == plane_identity
    children_before = rig.children(job)
    duplicate = rig.control(job, 'wake', action='wake', gate_id=gate_id)
    assert duplicate.status_code == 200 and duplicate.json() == awake.json()
    assert rig.children(job) == children_before
    assert fresh_semantics_refs(rig.root) == immediate
    with pytest.raises(ValueError, match='workflow_wake_'):
        with rig.provider.deck_owner_authority_scope(), rig.store._transaction() as tx:
            accept(tx, child_id=child_id)
    assert state(conn) == immediate['semantic']
    continued = rig.control(job, 'continue', action='continue', gate='deferred_pause', gate_id=gate_id)
    assert continued.status_code == 200, continued.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed', done
    assert ('rgb', (1, 1, 1)) in rig.trace
    results = done['execution']['runtime_state']['action_results']
    for action in ('lifecycle:wake', 'after', 'lifecycle:epilogue_sweep', 'lifecycle:epilogue_lid', 'lifecycle:epilogue_park'):
        assert any(r['action_id'] == action and r['ok'] is True for r in results), results
    assert hooks(done)[-1] == 'script_finally'
    assert_reopened(rig, done)
    assert fresh_semantics_refs(rig.root) == {'semantic':state(conn), 'projection':rig.store.deck_semantic_state(),
        'refs':rig.references.snapshot(('x', 'y', 'z', 'g'))}
    assert immutable(conn, job['job_id']) == parent_identity
    assert rig.native.tester.oem_no24v_state() is False


def test_connected_publication_failure_retains_real_native_refs(integrated_rig, monkeypatch):
    from bioxp.runtime_audit_store import RuntimeAuditDatabase
    import sqlite3

    rig = integrated_rig
    rig.body_gate = PhysicalLeafGate()
    payload = rig.payload('lineage-publication-failure')
    payload['document']['stages'][0]['actions'].append({
        'action_id': 'after', 'kind': 'oem_operation', 'oem_opcode': 'led',
        'source_occurrence_id': 'source:after', 'params': {'arguments': ['1','1','1']}})
    job = rig.start(payload)
    assert rig.body_gate.entered.wait(8)
    assert rig.control(job, 'pause', action='pause', mode='deferred').status_code == 200
    rig.body_gate.release.set()
    gate = rig.gate(job, 'deferred_pause')
    gate_id = gate['execution']['runtime_state']['workflow']['gate_id']
    conn = rig.store.connection
    before = state(conn)
    parent_before = tuple(conn.execute('SELECT requested_inputs_json,canonical_request_sha256,effective_inputs_json FROM operator_commands WHERE command_id=?', (job['job_id'],)).fetchone())
    transitions = [tuple(r) for r in conn.execute('SELECT * FROM operator_plane_deck_semantic_transitions')]
    published = []
    accept = rig.store._accept_workflow_wake_initialization
    def fail(tx, *, child_id):
        accept(tx, child_id=child_id)
        assert state(tx)['semantic_state_revision'] == before['semantic_state_revision'] + 1
        refs = rig.references.snapshot(('x', 'y', 'z', 'g'))
        assert refs['durable_clean'] and all(r['state'] == 'referenced' for r in refs['rows'].values())
        published.append((child_id, refs))
        raise sqlite3.OperationalError('connected injected publication rollback')
    monkeypatch.setattr(rig.store, '_accept_workflow_wake_initialization', fail)
    settled = []
    finalize = RuntimeAuditDatabase.finalize_claim
    def observe_finalization(db, **kwargs):
        result = finalize(db, **kwargs)
        if kwargs['result'].get('failure') == 'workflow_wake_authority_not_accepted':
            # Failure CAS has committed on the restored receipt connection;
            # the native parent has not received its source failure return yet.
            assert db.connection is not conn
            assert state(conn) == before
            assert tuple(conn.execute('SELECT requested_inputs_json,canonical_request_sha256,effective_inputs_json FROM operator_commands WHERE command_id=?', (job['job_id'],)).fetchone()) == parent_before
            assert [tuple(r) for r in conn.execute('SELECT * FROM operator_plane_deck_semantic_transitions')] == transitions
            now = {'semantic':state(conn), 'projection':rig.store.deck_semantic_state(),
                   'refs':rig.references.snapshot(('x', 'y', 'z', 'g'))}
            assert now['refs'] == published[0][1]
            assert fresh_semantics_refs(rig.root) == now
            child = dict(conn.execute('SELECT * FROM operator_commands WHERE command_id=?', (published[0][0],)).fetchone())
            assert child['status'] == 'failed'
            raw = json.loads(child['response_summary_json'])
            assert raw['authority_error'] == 'connected injected publication rollback'
            assert all(r['ok'] is True for r in raw['source_children'])
            assert all(p['published'] for p in raw['source_children'][1]['reference_publications'].values())
            settled.append(child)
        return result
    monkeypatch.setattr(RuntimeAuditDatabase, 'finalize_claim', observe_finalization)
    response = rig.control(job, 'wake', action='wake', gate_id=gate_id)
    assert response.status_code == 200, response.text
    done = rig.terminal(job)
    # The child is truthfully failed. Unsettled post-cycle parent authority
    # remains ambiguous/reconciling under the existing workflow finalizer.
    assert done['command']['status'] == 'ambiguous', done
    assert done['execution']['runtime_state']['workflow']['held_reason'] == 'workflow_settlement_unknown'
    assert len(published) == len(settled) == 1
    assert len(rig.native_results(done, 'wake_prepare')) == 1
    assert not rig.native_results(done, 'resume_temperature')
    assert dict(conn.execute('SELECT * FROM operator_commands WHERE command_id=?', (published[0][0],)).fetchone()) == settled[0]
    assert_reopened(rig, done)
