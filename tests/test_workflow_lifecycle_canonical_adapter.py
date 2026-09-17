"""Canonical lifecycle claims with doubled physical leaves; no hardware entry."""
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from threading import Event, Lock
from types import SimpleNamespace

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.operator_controls import (
    _DISPATCH_CONTEXT, current_operator_dispatch_context,
    make_workflow_lifecycle_control_executor,
)
from bioxp.runtime_audit_store import RuntimeAuditDatabase


@pytest.fixture
def rig(tmp_path):
    seed = OEMRuntimeStore(tmp_path)
    seed.close()
    store = OperatorCommandStore(tmp_path)
    store.bind_workflow_dispatcher(lambda claimed: None)
    store.admit_workflow(command_id='parent', idempotency_key='parent-key',
        plan_fingerprint='plan', requested_inputs={}, ownership_generation=1,
        resources=('axis:x', 'pipette', 'thermal'), board_epochs={})
    assert store.claim_next()['command_id'] == 'parent'
    state = SimpleNamespace(workflow=SimpleNamespace(command_id='parent', child_command_ids=[]))
    calls = []
    def leaf(name):
        def call(*args, **kwargs):
            calls.append((name, args, kwargs, current_operator_dispatch_context()))
            return {'ok': True, 'source_return': 0, 'completion_verified': False}
        return call
    tester = SimpleNamespace(**{name: leaf(name) for name in (
        'oem_set_tc_temperature', 'oem_set_lid_temperature', 'oem_chiller_set_temperature',
        'oem_set_chiller_pwm', 'oem_turn_off_heater', 'oem_thermal_bailout', 'oem_resume_temperature')})
    provider = SimpleNamespace(primitives=SimpleNamespace(tester=tester), generation_provider=lambda: 1,
        execute_x_stop_interrupt=leaf('abort'), initialize_motors=leaf('initialize'),
        wp8_update_thermal_door_open=leaf('restore'))
    execute = make_workflow_lifecycle_control_executor(store, lambda: provider, initial_check=leaf('initial'))
    yield SimpleNamespace(store=store, state=state, tester=tester, provider=provider,
                         calls=calls, execute=execute)
    store.stop()


def run(rig, operation, occurrence='step:1', arguments=None, control_id=None):
    with rig.store.workflow_context('parent', source_occurrence_id='outer'):
        return rig.execute(operation, rig.state, source_occurrence_id=occurrence,
                           arguments=arguments, control_id=control_id)


def test_identity_nonreplay_nested_dispatch_and_fresh_process(rig):
    token = _DISPATCH_CONTEXT.set({'operator_command_id': 'parent', 'idempotency_key': 'parent-key'})
    try:
        first = run(rig, 'epilogue_lid')
        assert current_operator_dispatch_context()['operator_command_id'] == 'parent'
        replay = run(rig, 'epilogue_lid')
        second = run(rig, 'epilogue_lid', 'step:2')
        third = run(rig, 'epilogue_lid', 'step:2', control_id='different-control')
    finally:
        _DISPATCH_CONTEXT.reset(token)
    assert first == replay
    assert len({first['command_id'], second['command_id'], third['command_id'], 'parent'}) == 4
    assert len(rig.calls) == 3
    assert rig.calls[0][1:3] == ((30.0, 0, -20.0), {'wait': False})
    assert rig.calls[0][3]['operator_command_id'] == first['command_id']
    assert rig.store.get_workflow('parent')['command']['status'] == 'dispatched'
    code = '''import json,sqlite3,sys
c=sqlite3.connect(sys.argv[1]); c.row_factory=sqlite3.Row
r=dict(c.execute('SELECT * FROM operator_commands WHERE command_id=?',(sys.argv[2],)).fetchone())
print(json.dumps(r))
'''
    row = json.loads(subprocess.check_output([sys.executable, '-c', code,
        str(rig.store.root / 'bioxp_runtime.db'), first['command_id']], text=True))
    assert row['status'] == 'completed'
    assert row['parent_command_id'] == 'parent'
    assert json.loads(row['receipt_json']) == first['receipt']
    assert json.loads(row['response_summary_json']) == {k: v for k, v in first.items() if k != 'receipt'}
    assert row['physical_effect_verified'] == 0


@pytest.mark.parametrize('operation,arguments,native', [
    ('set_tc_temperature', dict(temp_c=65.2, duration=10, rate_c_s=1.5), 'oem_set_tc_temperature'),
    ('set_lid_temperature', dict(temp_c=30.0, duration=0, rate_c_s=-20.0, wait=False), 'oem_set_lid_temperature'),
    ('set_chiller_temperature', dict(bank=1, temp_c=4.0), 'oem_chiller_set_temperature'),
    ('resume_temperature', {}, 'oem_resume_temperature'),
    ('thermal_bailout', {}, 'oem_thermal_bailout'),
    ('turn_off_heater', {}, 'oem_turn_off_heater'),
    ('set_chiller_pwm', dict(chiller=None, pwm=0), 'oem_set_chiller_pwm'),
    ('restore_door_model', dict(value=False), 'restore'),
])
def test_finite_roster(rig, operation, arguments, native):
    result = run(rig, operation, arguments=arguments)
    assert result['ok'] is True
    assert rig.calls[0][0] == native
    resources = [row[0] for row in rig.store.connection.execute(
        'SELECT resource_key FROM serial206_command_resources WHERE command_id=?', (result['command_id'],))]
    if operation.startswith('set_') and operation != 'set_chiller_pwm' or operation == 'resume_temperature':
        assert resources == ['thermal']
    if operation in ('thermal_bailout', 'turn_off_heater', 'set_chiller_pwm'):
        assert resources == []


def test_partial_shutdown_retains_first_write_and_never_replays(rig):
    def fail():
        raise RuntimeError('failed after native write')
    rig.tester.oem_turn_off_heater = fail
    first = run(rig, 'shutdown_temperature')
    assert first['status'] == 'ambiguous' and not first['ok']
    assert first['source_children'][0]['ok'] is True
    assert first['source_children'][1]['outcome_unknown'] is True
    assert run(rig, 'shutdown_temperature') == first
    assert len(rig.calls) == 1
    assert rig.store.finish_workflow('parent', status='failed', payload={},
                                    lifecycle_settled=True)['command']['status'] == 'ambiguous'


def test_known_failure_is_failed_not_ambiguous(rig):
    rig.tester.oem_set_tc_temperature = lambda **kw: {'ok': False, 'failure': 'source_rejected', 'source_return': False}
    result = run(rig, 'set_tc_temperature', arguments=dict(temp_c=1, duration=0, rate_c_s=1))
    assert result['status'] == 'failed'
    assert run(rig, 'set_tc_temperature', arguments=dict(temp_c=1, duration=0, rate_c_s=1)) == result


@pytest.mark.parametrize('release_operation', ['thermal_bailout', 'shutdown_temperature', 'software_abort'])
def test_active_child_control_delivery_not_fifo_or_tester_lock(rig, release_operation):
    entered, release = Event(), Event()
    tester_lock = Lock()
    def wait_leaf(**kw):
        with tester_lock:
            entered.set()
            assert release.wait(5), 'release control waited behind active child'
        return {'ok': True}
    def deliver(*args, **kwargs):
        assert tester_lock.locked()
        release.set()
        return {'ok': True, 'physical_effect_verified': False}
    rig.tester.oem_set_tc_temperature = wait_leaf
    if release_operation == 'thermal_bailout':
        rig.tester.oem_thermal_bailout = deliver
    elif release_operation == 'shutdown_temperature':
        rig.tester.oem_set_chiller_pwm = deliver
    else:
        rig.provider.execute_x_stop_interrupt = deliver
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending = pool.submit(run, rig, 'set_tc_temperature', 'active', dict(temp_c=60, duration=100, rate_c_s=1))
        try:
            assert entered.wait(5)
            with pytest.raises(ValueError, match='normal_resource_busy'):
                run(rig, 'resume_temperature', 'conflict')
            control = pool.submit(run, rig, release_operation, 'control').result(timeout=5)
            assert control['ok'] is True
            child = pending.result(timeout=5)
            assert child['command_id'] != control['command_id']
        finally:
            release.set()
    assert rig.store.connection.execute('SELECT COUNT(*) FROM operator_plane_commands').fetchone()[0] == 1


def test_unrelated_normal_work_denied(rig):
    db = RuntimeAuditDatabase(rig.store.root)
    try:
        with pytest.raises(ValueError, match='workflow_busy'):
            db.claim(dict(command_id='manual', idempotency_key='manual', action_id='manual', operation='manual',
                entrypoint_id='test', caller_class='operator', control_class='physical_command',
                ownership_generation=1, requested_inputs={'temp': 20}, resources=('thermal',)))
    finally:
        db.close()
    assert not rig.calls


def test_abort_finalizes_after_authority_invalidated_and_cannot_readmit(rig):
    def abort(values, *, abort):
        assert abort is True
        assert values['command_id'] == current_operator_dispatch_context()['operator_command_id']
        rig.tester.no24v = True
        rig.provider.reference_state = 'desynced'
        rig.store._priority_fence.set()
        return {'ok': True, 'physical_effect_verified': False, 'reference_state': 'desynced'}
    rig.provider.execute_x_stop_interrupt = abort
    result = run(rig, 'software_abort')
    assert result['status'] == 'completed'
    assert rig.tester.no24v and rig.provider.reference_state == 'desynced'
    with pytest.raises(ValueError, match='workflow_interrupted'):
        run(rig, 'resume_temperature')
    assert rig.store.publish_workflow('parent', payload={})
    final = rig.store.finish_workflow('parent', status='failed', payload={}, lifecycle_settled=True)
    assert final['command']['status'] == 'failed'


@pytest.mark.parametrize('operation,args', [('arbitrary_method', {}), ('set_chiller_pwm', {'chiller': None, 'pwm': 1}),
    ('set_tc_temperature', {'temp_c': float('nan'), 'duration': 0, 'rate_c_s': 1}),
    ('set_lid_temperature', {'temp_c': 1, 'duration': 0, 'rate_c_s': 1, 'wait': 'F'})])
def test_invalid_preclaim_emits_nothing(rig, operation, args):
    with pytest.raises(ValueError):
        run(rig, operation, arguments=args)
    assert not rig.calls and not rig.state.workflow.child_command_ids


def test_conflicting_duplicate_arguments_fail_before_native(rig):
    run(rig, 'set_chiller_temperature', arguments={'bank': 0, 'temp_c': 4})
    with pytest.raises(ValueError, match='idempotency key conflict'):
        run(rig, 'set_chiller_temperature', arguments={'bank': 0, 'temp_c': 8})
    assert len(rig.calls) == 1


def test_attempt_owned_native_error_callback_on_timer_thread(rig):
    entered, released = Event(), Event()
    callbacks, messages = [], []
    original = lambda message: None
    rig.tester._oem_thermal_error_callback = original
    def thermal(**kwargs):
        callbacks.append(rig.tester._oem_thermal_error_callback)
        entered.set()
        assert released.wait(5)
        return {'ok': False, 'source_board_error_event': True}
    rig.tester.oem_set_tc_temperature = thermal
    def execute():
        with rig.store.workflow_context('parent', source_occurrence_id='thermal'):
            return rig.execute('set_tc_temperature', rig.state, source_occurrence_id='thermal:1',
                arguments=dict(temp_c=60, duration=10, rate_c_s=1),
                source_error_callback=lambda message: (messages.append(message), released.set()))
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending = pool.submit(execute)
        try:
            assert entered.wait(5)
            # Independent timer-like thread, deliberately without ContextVars.
            pool.submit(callbacks[0], 'native board timeout').result(timeout=5)
            assert pending.result(timeout=5)['status'] == 'failed'
        finally:
            released.set()
    assert messages == ['native board timeout']
    assert rig.tester._oem_thermal_error_callback is original
    callbacks[0]('late error from old timer')
    assert messages == ['native board timeout']


def test_stale_timer_callback_does_not_signal_changed_attempt(rig):
    called = []
    def thermal(**kwargs):
        callback = rig.tester._oem_thermal_error_callback
        rig.store._priority_fence.set()
        callback('stale after addressed interrupt')
        return {'ok': False}
    rig.tester.oem_set_tc_temperature = thermal
    with rig.store.workflow_context('parent', source_occurrence_id='thermal'):
        result = rig.execute('set_tc_temperature', rig.state, source_occurrence_id='thermal:1',
            arguments=dict(temp_c=60, duration=10, rate_c_s=1), source_error_callback=called.append)
    assert not result['ok'] and not called


@pytest.mark.parametrize('stopped', [True, False])
def test_exceptional_native_timer_retains_resource_and_publishes_completion(rig, stopped):
    from concurrent.futures import Future
    timer = Future()
    events = []
    saved = []
    def thermal(**kwargs):
        saved.append(rig.tester._oem_thermal_error_callback)
        return {'ok': False, 'source_exception': 'native wait exited',
                'source_timer_enabled': True, 'source_timer_pending': True,
                'source_timer_future': timer, 'source_children': [{'ok': False, 'source_timer_future': timer}]}
    rig.tester.oem_set_tc_temperature = thermal
    with rig.store.workflow_context('parent', source_occurrence_id='thermal'):
        result = rig.execute('set_tc_temperature', rig.state, source_occurrence_id='thermal:1',
            arguments=dict(temp_c=60, duration=10, rate_c_s=1), source_error_callback=events.append)
    assert result['status'] == 'dispatched' and result['ok'] is False
    owned = result['owned_children'][0]
    assert not owned.future.done()
    assert 'source_timer_future' not in json.dumps(result['receipt'])
    saved[0]('source timer still active')
    assert events == ['source timer still active']
    with pytest.raises(ValueError, match='normal_resource_busy'):
        run(rig, 'resume_temperature', 'conflict')
    replay = run(rig, 'set_tc_temperature', 'thermal:1', dict(temp_c=60, duration=10, rate_c_s=1))
    assert replay['owned_children'][0].future is owned.future
    assert len(saved) == 1
    timer.set_result({'ok': stopped, 'source_timer_stopped': stopped, 'source_timer_replaced': not stopped})
    final = owned.future.result(timeout=5)
    assert final['status'] == ('failed' if stopped else 'ambiguous')
    assert final['source_timer_pending'] is False
    assert final['source_exception'] == 'native wait exited'
    saved[0]('late after actual timer return')
    assert events == ['source timer still active']
    raw = rig.store.connection.execute('SELECT status,receipt_json FROM operator_commands WHERE command_id=?',
                                      (owned.command_id,)).fetchone()
    assert raw[0] == final['status']
    assert json.loads(raw[1]) == final['receipt']
    if stopped:
        assert run(rig, 'resume_temperature', 'released')['ok']
    else:
        with pytest.raises(ValueError, match='normal_resource_busy'):
            run(rig, 'resume_temperature', 'still-unknown')


def test_real_provider_and_native_software_abort_keep_reference_invalidation(rig):
    from bioxp.oem_serial206_initialization import (
        Serial206OemInitializationProvider, Serial206ProductionPrimitiveAdapter,
    )
    from bioxp.services.reference_service import ReferenceStateStore
    from bioxp.usb_driver import BioXpTester
    # No constructor, transport, device, receiver thread or live router.
    tester = BioXpTester.__new__(BioXpTester)
    released = []
    tester.novo_router = SimpleNamespace(set_motor_abort_event=lambda board, motor: released.append((board, motor)))
    tester._oem_24v_dropped = False
    tester._oem_thermal_board_timer_enabled = True
    source = OEMRuntimeStore(rig.store.root)
    refs = ReferenceStateStore(rig.store.root / 'bioxp_runtime.db')
    primitives = Serial206ProductionPrimitiveAdapter(tester, None, authority_provider=lambda: None,
        generation_provider=lambda: 1, reference_store=refs)
    provider = Serial206OemInitializationProvider(primitives, state_store=source,
        reference_store=refs, generation_provider=lambda: 1)
    rig.execute = make_workflow_lifecycle_control_executor(rig.store, lambda: provider)
    try:
        result = run(rig, 'software_abort')
        assert result['ok'] is True, result
        assert result['result']['software_abort'] is True
        assert result['result']['stop_delivery_attempted'] is False
        assert result['physical_effect_verified'] is False
        assert tester._oem_24v_dropped is True
        assert tester._oem_thermal_board_timer_enabled is False
        assert set(released) == {(4, 0), (4, 1), (4, 2), (5, 0), (6, 0)}
        assert provider._load_state()['x_lifecycle']['reference_state'] == 'desynced'
        assert provider._load_state()['z_lifecycle']['reference_state'] == 'desynced'
        assert refs.snapshot(('x', 'z'))['rows']['x']['state'] == 'desynced'
        assert refs.snapshot(('x', 'z'))['rows']['z']['state'] == 'desynced'
        before = list(released)
        assert run(rig, 'software_abort') == result
        assert released == before
        raw = rig.store.connection.execute('SELECT receipt_json FROM operator_commands WHERE command_id=?',
                                           (result['command_id'],)).fetchone()[0]
        assert json.loads(raw)['response']['result']['aggregate_authority_invalidation']['ok'] is True
    finally:
        source.close()


def test_factory_does_not_resolve_native_owner(rig):
    def forbidden():
        pytest.fail('eager native resolution')
    assert callable(make_workflow_lifecycle_control_executor(rig.store, forbidden))
