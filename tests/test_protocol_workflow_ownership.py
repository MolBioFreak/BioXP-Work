"""Canonical workflow custody on private SQLite; no native product entry."""
import pytest
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.runtime_audit_store import RuntimeAuditDatabase


@pytest.fixture
def store(tmp_path):
    preparation = OEMRuntimeStore(tmp_path)
    preparation.close()
    owner = OperatorCommandStore(tmp_path)
    owner.bind_workflow_dispatcher(lambda claimed: None)
    yield owner
    owner.stop()


def admit(store, name='parent', resources=('axis:x',)):
    return store.admit_workflow(command_id=name, idempotency_key='key-'+name,
        plan_fingerprint='plan', requested_inputs={'bundle': {'execution': {'runtime_state': {}}}},
        ownership_generation=1, resources=resources, board_epochs={})


def direct(store, key='direct'):
    db = RuntimeAuditDatabase(store.root)
    return db, dict(command_id=key, idempotency_key=key, action_id='pipette.mix', operation='mix',
        entrypoint_id='test', caller_class='protocol', control_class='physical_liquid_command',
        ownership_generation=1, requested_inputs={'volume': 1})


def test_parent_without_movement_and_replay(store):
    first = admit(store)
    assert first['command']['status'] == 'queued'
    assert admit(store) == first
    assert store.connection.execute('SELECT COUNT(*) FROM serial206_movement_commands').fetchone()[0] == 0
    assert store.claim_next()['command_kind'] == 'protocol_workflow'
    assert store.get_workflow('parent')['command']['status'] == 'dispatched'
    assert store.connection.execute('SELECT workflow_command_id FROM operator_plane_lane').fetchone()[0] == 'parent'


def test_second_parent_excluded_and_direct_admitted(store):
    admit(store)
    with pytest.raises(ValueError, match='workflow_busy'):
        admit(store, 'second')
    store.claim_next()
    db, payload = direct(store)
    # 2026-09-21: custody never refuses a direct claim.
    child, created = db.claim(payload, pipette=True)
    assert created and child['parent_command_id'] is None


def test_real_direct_child_and_unsettled_custody(store):
    admit(store)
    store.claim_next()
    db, payload = direct(store)
    with store.workflow_context('parent', source_occurrence_id='occurrence-1'):
        child, created = db.claim(payload, pipette=True)
    assert created and child['parent_command_id'] == 'parent'
    assert child['command_id'] != 'parent'
    result = store.finish_workflow('parent', status='completed', payload={}, lifecycle_settled=True)
    assert result['command']['status'] == 'ambiguous'
    assert store.connection.execute('SELECT workflow_command_id FROM operator_plane_lane').fetchone()[0] == 'parent'


def test_future_child_inherits_original_stop_epoch(store):
    admit(store)
    store.claim_next()
    with store._transaction() as conn:
        conn.execute('UPDATE operator_plane_safety SET x_epoch=x_epoch+1')
    with pytest.raises(ValueError, match='workflow_interrupted'):
        with store.workflow_context('parent', source_occurrence_id='future'):
            pytest.fail('must not enter')


def test_unaffected_scoped_stop_preserves_thermal_parent(store):
    admit(store, resources=('thermal',))
    store.claim_next()
    with store._transaction() as conn:
        conn.execute('UPDATE operator_plane_safety SET x_epoch=x_epoch+1')
    store.assert_workflow_current('parent')
    result = store.finish_workflow('parent', status='completed', payload={'known': True}, lifecycle_settled=True)
    assert result['command']['status'] == 'completed'
    assert store.connection.execute('SELECT workflow_command_id FROM operator_plane_lane').fetchone()[0] is None


def test_control_same_key_reconciles_after_settlement(store):
    admit(store)
    store.claim_next()
    calls = []
    def control(cid, request):
        calls.append(cid)
        return {'phase': 'waiting', 'gate': 'ordinary_pause', 'gate_id': cid}
    store.bind_workflow_controls('parent', control)
    request = {'action': 'pause', 'mode': 'ordinary', 'command_id': 'parent',
               'expected_ownership_generation': 1, 'idempotency_key': 'pause-key'}
    response = store.control_workflow('parent', request=request)
    store.finish_workflow('parent', status='interrupted', payload={}, lifecycle_settled=True)
    assert store.control_workflow('parent', request=request) == response
    assert len(calls) == 1


def test_direct_mutation_defers_parent_activation_and_then_releases(store):
    with store.normal_mutation_scope(resources=('thermal',)):
        admit(store)
        assert store.claim_next() is None
    assert store.claim_next()['command_id'] == 'parent'
    # 2026-09-21: custody never refuses an unrelated setter.
    with store.normal_mutation_scope(resources=('thermal',)) as unrelated:
        assert store.connection.execute(
            'SELECT parent_command_id FROM operator_commands WHERE command_id=?', (unrelated,)).fetchone()[0] is None
    with store.workflow_context('parent', source_occurrence_id='native-setter'):
        with store.normal_mutation_scope(resources=('thermal',)) as child:
            assert store.connection.execute('SELECT parent_command_id FROM operator_commands WHERE command_id=?', (child,)).fetchone()[0] == 'parent'
    assert store.finish_workflow('parent', status='completed', payload={}, lifecycle_settled=True)['command']['status'] == 'completed'


def test_forged_public_binding_and_empty_occurrence_refused(store):
    admit(store)
    store.claim_next()
    db, payload = direct(store)
    payload['requested_inputs']['workflow_binding'] = {'parent_command_id': 'parent'}
    with pytest.raises(ValueError, match='untrusted workflow binding'):
        db.claim(payload, pipette=True)
    with pytest.raises(ValueError, match='workflow_binding_changed'):
        with store.workflow_context('parent', source_occurrence_id=''):
            db.claim({**payload, 'requested_inputs': {}}, pipette=True)


def test_original_stop_wakes_bound_wait_once_and_future_child_cannot_enter(store):
    admit(store)
    store.claim_next()
    calls = []
    store.bind_workflow_controls('parent', lambda cid, request: calls.append((cid, request)))
    with store._transaction() as conn:
        conn.execute('UPDATE operator_plane_safety SET x_epoch=x_epoch+1')
    store._notify_workflow_interrupt()
    store._notify_workflow_interrupt()
    assert len(calls) == 1 and calls[0][1] == {'action': '_addressed_stop'}
    with pytest.raises(ValueError, match='workflow_interrupted'):
        with store.workflow_context('parent', source_occurrence_id='after-stop'):
            pass


def test_restart_custody_is_ambiguous_and_non_blocking(store):
    admit(store)
    store.claim_next()
    store._startup_recover()
    bundle = store.get_workflow('parent')
    assert bundle['command']['status'] == bundle['status'] == 'ambiguous'
    assert bundle['execution']['runtime_state']['workflow']['phase'] == 'reconciling'
    assert store.claim_next() is None
    # 2026-09-21: ambiguous custody is recorded, never blocking.
    with store.normal_mutation_scope(resources=('thermal',)) as admitted:
        assert admitted


def test_owner_loss_forbids_late_parent_publication_and_terminal_upgrade(store):
    admit(store)
    store.claim_next()
    with store._transaction() as conn:
        conn.execute('UPDATE operator_plane_lane SET owner_id=?', ('different-owner',))
    with pytest.raises(ValueError, match='workflow_authority_lost'):
        store.publish_workflow('parent', payload={'late': True})
    with pytest.raises(ValueError, match='workflow_authority_lost'):
        store.finish_workflow('parent', status='completed', payload={'late': True}, lifecycle_settled=True)
    with store._transaction() as conn:
        conn.execute('UPDATE operator_plane_lane SET owner_id=?', (store.owner_id,))


def test_control_signal_failure_is_durable_and_not_repeated(store):
    admit(store)
    store.claim_next()
    calls = []
    def reject(cid, request):
        calls.append(cid)
        raise ValueError('wrong_gate')
    store.bind_workflow_controls('parent', reject)
    request = {'action': 'continue', 'command_id': 'parent', 'expected_ownership_generation': 1,
               'idempotency_key': 'wrong-gate', 'gate': 'ordinary_pause', 'gate_id': 'stale'}
    with pytest.raises(ValueError, match='wrong_gate'):
        store.control_workflow('parent', request=request)
    receipt = store.control_workflow('parent', request=request)
    assert receipt['accepted'] is False and receipt['control_command_id'] == calls[0]
    assert len(calls) == 1
