"""Wake receipt/transition fencing; source witnesses are explicit unit fixtures.

These store tests do not replace the parent API/native composition qualification.
"""
from contextlib import nullcontext
from types import SimpleNamespace
from threading import RLock
import json

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.operator_controls import make_workflow_lifecycle_control_executor


@pytest.fixture
def wake(tmp_path):
    source = OEMRuntimeStore(tmp_path)
    source.record_board4_transition(active=True, ack={'status': 100}, transition_id='startup', ownership_generation=1)
    native = {'movement_ledger': [], 'used_approvals': [], 'initialize_motion_ledger': [],
              'x_lifecycle': {'board_lifecycle_generation': 10}, 'machine_status': {'thermal_door_open': True}}
    source.write_oem_serial206_initialization_state(native)
    store = OperatorCommandStore(tmp_path)
    store.bind_workflow_dispatcher(lambda claimed: None)
    store.admit_workflow(command_id='parent', idempotency_key='parent-key', plan_fingerprint='plan',
        requested_inputs={}, ownership_generation=1, resources=('axis:x', 'pipette', 'thermal'),
        board_epochs={'4': 1, '5': 10})
    assert store.claim_next()['command_id'] == 'parent'
    workflow = dict(command_id='parent', phase='waking', gate='deferred_pause', gate_id='gate:1',
                    requested_control={'action': 'wake'}, child_command_ids=[])
    store.bind_workflow_controls('parent', lambda cid, request: {**workflow, 'last_control_id': cid})
    control = store.control_workflow('parent', request=dict(action='wake', command_id='parent',
        expected_ownership_generation=1, idempotency_key='wake-1', gate_id='gate:1'))
    workflow['last_control_id'] = control['control_command_id']
    store.publish_workflow('parent', payload={'execution': {'runtime_state': {'workflow': workflow}}})
    state = SimpleNamespace(workflow=SimpleNamespace(**workflow))
    tester = SimpleNamespace(_oem_board_lifecycle_generation=10, _oem_transport_generation=2,
        _oem_abort_generation=0, _oem_24v_dropped=False, active_generation=10)
    tester.oem_current_board_lifecycle_generation = lambda: tester.active_generation
    tester.oem_resume_temperature = lambda: {'ok': True}
    stamps = dict(ownership_generation=1, board_epoch_4=1, board_epoch_5=10)
    store.bind_deck_owner_authority_reader(lambda: stamps.copy(), scope=nullcontext)
    provider = SimpleNamespace(primitives=SimpleNamespace(tester=tester), generation_provider=lambda: 1,
        _lock=RLock(), _load_state=lambda: native,
        deck_owner_authority_scope=nullcontext,
        reference_store=SimpleNamespace(snapshot=lambda axes: {'durable_clean': True,
            'rows': {axis: {'state': 'referenced'} for axis in axes}}))
    called = []

    def initial(state, *, validate_current):
        called.append('initial')
        assert validate_current('admission') is True
        assert validate_current('deactivate_boards') is True
        source.record_board4_transition(active=False, ack={'status': 100}, transition_id='wake-off', ownership_generation=1)
        native['x_lifecycle']['board_lifecycle_generation'] = None
        source.write_oem_serial206_initialization_state(native)
        tester.active_generation = None
        assert validate_current('activate_boards') is True
        source.record_board4_transition(active=True, ack={'status': 100}, transition_id='wake-on', ownership_generation=1)
        assert validate_current('oem_begin_board_lifecycle_generation') is True
        tester._oem_board_lifecycle_generation = tester.active_generation = 11
        assert validate_current('completion') is True
        return {'ok': True, 'source_return': True, 'initial_check': {'board_lifecycle_generation': {
            'ok': True, 'board_lifecycle_generation': 11, 'transport_generation': 2,
            'deactivation_complete': True, 'activation_complete': True, 'source_order': ['cmd64=0', 'cmd64=1']}}}

    def initialize(*, mode):
        called.append('initialize')
        assert mode == 'live'
        native['x_lifecycle']['board_lifecycle_generation'] = tester.active_generation
        source.write_oem_serial206_initialization_state(native)
        epoch = source.board4_authority_projection()['board']['active_board_epoch']
        stamps.update(board_epoch_4=epoch, board_epoch_5=tester.active_generation)
        return {'ok': True, 'reference_publications': {axis: {'published': True, 'fence': {
            'generation': 1, 'board_generation': tester.active_generation, 'board_epoch': epoch}}
            for axis in ('x', 'y', 'z', 'g')}}
    provider.initialize_motors = initialize
    execute = make_workflow_lifecycle_control_executor(store, lambda: provider, initial_check=initial)
    rig = SimpleNamespace(store=store, source=source, tester=tester, state=state, provider=provider,
        execute=execute, initial=initial, called=called, stamps=stamps, control=control['control_command_id'])
    yield rig
    store.stop()
    source.close()


def run(wake, operation='wake_prepare'):
    with wake.store.workflow_context('parent', source_occurrence_id='wake'):
        return wake.execute(operation, wake.state, source_occurrence_id='wake:1', control_id=wake.control)


def test_wake_one_child_proven_transition_and_following_thermal(wake):
    result = run(wake)
    assert result['ok'] is True, result
    assert wake.called == ['initial', 'initialize']
    assert run(wake) == result
    assert wake.called == ['initial', 'initialize']
    assert result['source_prior_door_open'] is True
    assert len(result['source_children']) == 2
    assert result['source_children'][0]['initial_check']['board_lifecycle_generation']['board_lifecycle_generation'] == 11
    wake.store.assert_workflow_current('parent')
    assert run(wake, 'resume_temperature')['ok'] is True
    row = wake.store.connection.execute("SELECT effective_inputs_json,requested_inputs_json FROM operator_commands WHERE command_id='parent'").fetchone()
    assert json.loads(row[0])['workflow_board_epochs'] == {'4': 2, '5': 11}
    assert json.loads(row[1])['workflow_footprint']['board_epochs'] == {'4': 1, '5': 10}
    assert wake.store.connection.execute("SELECT COUNT(*) FROM operator_commands WHERE operation='wake_prepare'").fetchone()[0] == 1
    # Accepted input/plane request and digest remain immutable; replay is a read.
    plane = json.loads(wake.store.connection.execute("SELECT requested_json FROM operator_plane_commands WHERE command_id='parent'").fetchone()[0])
    assert plane['workflow_footprint']['board_epochs'] == {'4': 1, '5': 10}
    replay = wake.store.admit_workflow(command_id='new-request-id', idempotency_key='parent-key',
        plan_fingerprint='plan', requested_inputs={}, ownership_generation=1,
        resources=('axis:x', 'pipette', 'thermal'), board_epochs={'4': 1, '5': 10})
    assert replay['command']['command_id'] == 'parent'
    assert wake.called == ['initial', 'initialize']
    with pytest.raises(ValueError, match='workflow_wake_binding_changed'):
        with wake.store._transaction() as conn:
            wake.store._accept_workflow_wake_initialization(conn, child_id=result['command_id'])
    wake.store.finish_workflow('parent', status='completed', payload={}, lifecycle_settled=True)
    again = wake.store.admit_workflow(command_id='another-request-id', idempotency_key='parent-key',
        plan_fingerprint='plan', requested_inputs={}, ownership_generation=1,
        resources=('axis:x', 'pipette', 'thermal'), board_epochs={'4': 1, '5': 10})
    assert again['command']['status'] == 'completed'
    assert wake.called == ['initial', 'initialize']


@pytest.mark.parametrize('fault', ['reference', 'owner', 'generation', 'safety', 'gate', 'missing_publication', 'extra_cycle'])
def test_wake_after_source_failure_never_rebases(wake, fault):
    initialize = wake.provider.initialize_motors
    def broken(**kwargs):
        result = initialize(**kwargs)
        if fault == 'reference':
            wake.provider.reference_store.snapshot = lambda axes: {'durable_clean': False}
        elif fault == 'owner':
            wake.provider.generation_provider = lambda: 2
        elif fault == 'generation':
            wake.tester.active_generation = 12
        elif fault == 'safety':
            wake.store._priority_fence.set()
        elif fault == 'gate':
            wake.store.publish_workflow('parent', payload={})
        elif fault == 'missing_publication':
            result['reference_publications']['x']['published'] = False
        else:
            wake.source.record_board4_transition(active=True, ack={'status': 100}, transition_id='external', ownership_generation=1)
        return result
    wake.provider.initialize_motors = broken
    result = run(wake)
    assert result['ok'] is False and result['status'] == 'failed'
    assert result['source_children'][1]['ok'] is True
    row = wake.store.connection.execute("SELECT requested_json FROM operator_plane_commands WHERE command_id='parent'").fetchone()
    assert json.loads(row[0])['workflow_footprint']['board_epochs'] == {'4': 1, '5': 10}
    child = wake.store.connection.execute('SELECT status,response_summary_json FROM operator_commands WHERE command_id=?', (result['command_id'],)).fetchone()
    assert child[0] == 'failed'
    assert json.loads(child[1])['failure'] == 'workflow_wake_authority_not_accepted'
    with pytest.raises(ValueError):
        run(wake, 'resume_temperature')


def test_wake_midcycle_external_generation_refused_before_init(wake):
    def wrong(state, *, validate_current):
        validate_current('admission')
        wake.tester.active_generation = 12
        validate_current('can_ready')
        pytest.fail('external generation must fail before leaf')
    wake.execute = make_workflow_lifecycle_control_executor(wake.store, lambda: wake.provider, initial_check=wrong)
    result = run(wake)
    assert result['status'] == 'ambiguous' and not result['ok']
    assert wake.called == []


def test_unreached_or_wrong_control_never_enters_native(wake):
    wake.control = 'different'
    with pytest.raises(ValueError, match='workflow_wake_gate_changed'):
        run(wake)
    assert wake.called == []


def test_wake_source_false_without_cycle_still_initializes(wake):
    def initial(state, *, validate_current):
        validate_current('admission')
        validate_current('completion')
        return {'ok': True, 'source_return': False, 'initial_check': {'error': 'CAN_READY_timeout'}}
    wake.execute = make_workflow_lifecycle_control_executor(wake.store, lambda: wake.provider, initial_check=initial)
    result = run(wake)
    assert result['ok'] is True, result
    assert result['source_children'][0]['source_return'] is False
    assert wake.called == ['initialize']
    wake.store.assert_workflow_current('parent')
