"""Offline aggregate Abort ordering through real provider/runtime owners."""
import threading
from types import SimpleNamespace

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider, Serial206ProductionPrimitiveAdapter
from bioxp.serial206_y_provider import Serial206YProvider
from bioxp.services.reference_service import ReferenceStateStore
from test_oem_abort_g_repair import driver
from test_serial206_y_provider import FakeYTester


def spawn(fn):
    result = []
    def run():
        try:
            result.append(fn())
        except BaseException as exc:
            result.append(exc)
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, result


def finish(job):
    thread, result = job
    thread.join(8)
    assert not thread.is_alive(), 'owner deadlocked'
    assert len(result) == 1 and not isinstance(result[0], BaseException), result
    return result[0]


@pytest.fixture
def owners(tmp_path, monkeypatch):
    monkeypatch.setattr('bioxp.oem_serial206_initialization.load_oem_parity_config', lambda _: SimpleNamespace(
        blockers=[], values={'SerialNumber': 206, 'CameraCalibrated': False}, calibration_source='offline'))
    store = OEMRuntimeStore(tmp_path / 'runtime')
    references = ReferenceStateStore(tmp_path / 'runtime' / 'references.json')
    d = driver()
    d._send_motor = lambda *a, **k: pytest.fail('software Abort transmitted a controller command')
    adapter = Serial206ProductionPrimitiveAdapter(d, None, authority_provider=lambda: None,
        generation_provider=lambda: 1, reference_store=references)
    provider = Serial206OemInitializationProvider(adapter, state_store=store,
        reference_store=references, generation_provider=lambda: 1)
    store.record_board4_transition(active=True, ack={'status': 100}, transition_id='offline-active', ownership_generation=1)
    for axis in ('y', 'z', 'gripper'):
        assert store.prepare_axis_authority(axis, ownership_generation=1, profile_fingerprint='offline')['ok']
        assert store.publish_axis_reference(axis, ownership_generation=1, position_steps=0)['ok']
    state = provider._load_state()
    state['z_lifecycle'].update(state='referenced_ready', reference_state='referenced', generation=1, board_lifecycle_generation=1)
    provider._save_state(state)
    y = Serial206YProvider(FakeYTester(), state_store=store, generation_provider=lambda: 1)
    calls = []
    original = d.motor_oem_force_abort_motion
    def abort(**kw):
        calls.append('softwareAbort')
        return original(**kw)
    monkeypatch.setattr(d, 'motor_oem_force_abort_motion', abort)
    yield provider, y, store, calls
    store.close()


def aggregate(p):
    return p.execute_x_stop_interrupt({'command_id': 'offline-abort'}, abort=True)


def assert_invalid(p, store):
    axes = store.board4_authority_projection()['axes']
    for axis in ('y', 'z', 'gripper'):
        assert axes[axis]['reference_state'] == 'reconciliation_required'
        assert axes[axis]['prepared_board_epoch'] is None
    state = p._load_state()
    assert state['x_lifecycle']['state'] == 'failed_latched'
    z = state['z_lifecycle']
    assert z['state'] == 'failed_latched' and z['reference_state'] == 'desynced'
    assert z['board_lifecycle_generation'] is None


def test_aggregate_invalidates_all_existing_owners_without_redispatch(owners):
    p, y, store, calls = owners
    result = aggregate(p)
    assert result['ok'], result
    assert calls == ['softwareAbort']
    assert_invalid(p, store)
    assert not p._z_interrupt_active
    assert y.prepare()['ok']  # Deliberate fresh preparation, not stale work.


def test_cancellation_does_not_wait_for_database_and_admission_is_fenced(owners, monkeypatch):
    p, y, store, calls = owners
    cancelled = threading.Event()
    original = p.primitives.tester.motor_oem_force_abort_motion
    def abort(**kw):
        result = original(**kw)
        cancelled.set()
        return result
    monkeypatch.setattr(p.primitives.tester, 'motor_oem_force_abort_motion', abort)
    with store._lock:
        job = spawn(lambda: aggregate(p))
        assert cancelled.wait(3), 'software waiters blocked behind database lock'
        assert not y.prepare()['ok']
        assert not p.execute_z_intent('prepare', expected_generation=1, idempotency_key='blocked')['ok']
    assert finish(job)['ok']
    assert calls == ['softwareAbort']
    assert_invalid(p, store)


@pytest.mark.parametrize('operation', ['home', 'prepare', 'move'])
def test_y_stale_completion_and_publication_after_abort_and_fresh_prepare(owners, monkeypatch, operation):
    p, y, store, calls = owners
    entered, release = threading.Event(), threading.Event()
    name = {'home': 'motor_oem_home_axis', 'prepare': 'motor_oem_require_no_motion_profile', 'move': 'motor_y_move_relative_strict'}[operation]
    original = getattr(y.tester, name)
    def delayed(*a, **kw):
        result = original(*a, **kw)
        entered.set()
        assert release.wait(5)
        return result
    monkeypatch.setattr(y.tester, name, delayed)
    op = {'home': lambda: y.home('diagnostic'), 'prepare': y.prepare, 'move': lambda: y.move_steps(10)}[operation]
    job = spawn(op)
    try:
        assert entered.wait(3)
        assert aggregate(p)['ok']
        fresh = Serial206YProvider(FakeYTester(), state_store=store, generation_provider=lambda: 1)
        assert fresh.prepare()['ok']
    finally:
        release.set()
    result = finish(job)
    assert not result['ok'], result
    assert not result.get('reference_published')
    assert store.board4_authority_projection()['axes']['y']['reference_state'] == 'unreferenced'
    assert calls == ['softwareAbort']


def test_y_pending_receipt_cannot_terminalize_in_new_epoch(owners):
    p, y, store, calls = owners
    issued = y.move_absolute(1200, wait_for_stop=False)
    assert issued['ok'], issued
    assert aggregate(p)['ok']
    assert y.prepare()['ok']
    result = y.terminalize_absolute(issued)
    assert not result['ok']


@pytest.mark.parametrize('operation', ['move_steps', 'prepare'])
def test_z_running_owner_is_interrupted_without_z_stop(owners, monkeypatch, operation):
    p, y, store, calls = owners
    entered, release = threading.Event(), threading.Event()
    monkeypatch.setattr(p.primitives, 'current_board_lifecycle_generation', lambda: 1)
    def delayed(**kw):
        entered.set()
        assert release.wait(5)
        return {'ok': True, 'controller_command_acknowledged': True, 'controller_terminal_state_verified': True,
                'board_lifecycle_generation': 1, 'board_lifecycle_reused': True}
    monkeypatch.setattr(p.primitives, 'z_move_steps' if operation == 'move_steps' else 'prepare_for_initialize_motors', delayed)
    move = spawn(lambda: p.execute_z_intent(operation, inputs={'steps': 10}, expected_generation=1, idempotency_key='z-old'))
    try:
        assert entered.wait(3)
        cancellation = threading.Event()
        original = p.primitives.tester.motor_oem_force_abort_motion
        def abort(**kw):
            result = original(**kw)
            cancellation.set()
            return result
        monkeypatch.setattr(p.primitives.tester, 'motor_oem_force_abort_motion', abort)
        aborted = spawn(lambda: aggregate(p))
        assert cancellation.wait(3)
        assert not p.execute_z_intent('prepare', expected_generation=1, idempotency_key='new')['ok']
    finally:
        release.set()
    assert not finish(move)['ok']
    assert finish(aborted)['ok']
    assert_invalid(p, store)
    assert calls == ['softwareAbort']


def test_reentrant_abort_preserves_outer_owner_flags(owners, monkeypatch):
    p, y, store, calls = owners
    original = p.primitives.tester.motor_oem_force_abort_motion
    entered = False
    def abort(**kw):
        nonlocal entered
        result = original(**kw)
        if not entered:
            entered = True
            assert aggregate(p)['ok']
            assert p._x_interrupt_active and p._z_interrupt_active
            assert not y.prepare()['ok']
        return result
    monkeypatch.setattr(p.primitives.tester, 'motor_oem_force_abort_motion', abort)
    assert finish(spawn(lambda: aggregate(p)))['ok']
    assert calls == ['softwareAbort', 'softwareAbort']  # Two explicit invocations only.
    assert not p._x_interrupt_active and not p._z_interrupt_active
    assert_invalid(p, store)


def test_overlapping_z_interrupt_keeps_its_fence_after_aggregate_finishes(owners, monkeypatch):
    p, y, store, calls = owners
    entered, release = threading.Event(), threading.Event()
    def z_stop(**kw):
        entered.set()
        assert release.wait(5)
        return {'ok': True, 'controller_command_acknowledged': True, 'controller_terminal_state_verified': True}
    monkeypatch.setattr(p.primitives, 'z_stop', z_stop)
    stopped = spawn(lambda: p.execute_z_stop_interrupt(expected_generation=1, idempotency_key='independent-z'))
    try:
        assert entered.wait(3)
        assert aggregate(p)['ok']
        assert p._z_interrupt_active
        assert not p.execute_z_intent('prepare', expected_generation=1, idempotency_key='blocked')['ok']
    finally:
        release.set()
    finish(stopped)
    assert not p._z_interrupt_active
    assert_invalid(p, store)
    assert calls == ['softwareAbort']


@pytest.mark.parametrize('fault', ['controller', 'runtime', 'lifecycle'])
def test_exception_cleanup_invalidates_other_owners_and_retains_failed_hold(owners, monkeypatch, fault):
    p, y, store, calls = owners
    if fault == 'controller':
        original = p.primitives.tester.motor_oem_force_abort_motion
        def fail(**kw):
            original(**kw)
            raise RuntimeError('injected controller wrapper failure')
        monkeypatch.setattr(p.primitives.tester, 'motor_oem_force_abort_motion', fail)
    elif fault == 'runtime':
        original = store.require_axis_reconciliation
        def fail(axis, **kw):
            if axis == 'y':
                raise RuntimeError('injected Y storage failure')
            return original(axis, **kw)
        monkeypatch.setattr(store, 'require_axis_reconciliation', fail)
    else:
        original = p._save_state
        monkeypatch.setattr(p, '_save_state', lambda *a: (_ for _ in ()).throw(RuntimeError('injected lifecycle failure')))
    result = aggregate(p)
    assert not result['ok']
    assert calls == ['softwareAbort']
    assert not p._x_interrupt_active
    if fault != 'controller':
        assert p._z_interrupt_active
        assert not y.prepare()['ok']
    axes = store.board4_authority_projection()['axes']
    assert axes['z']['reference_state'] == axes['gripper']['reference_state'] == 'reconciliation_required'
    if fault != 'controller':
        if fault == 'runtime':
            monkeypatch.setattr(store, 'require_axis_reconciliation', original)
        else:
            monkeypatch.setattr(p, '_save_state', original)
        assert p._reconcile_aggregate_software_abort('offline-abort')['ok']
        assert not p._z_interrupt_active
        assert y.prepare()['ok']
        assert calls == ['softwareAbort']  # Persistence retry is not dispatch.


def test_direct_z_software_abort_uses_same_aggregate_owner_fences(owners, monkeypatch):
    p, y, store, calls = owners
    original = p.primitives.tester.motor_oem_force_abort_motion
    def abort(**kw):
        assert p._x_interrupt_active and p._z_interrupt_active
        assert not y.prepare()['ok']
        assert not p.execute_x_intent('terminal_status')['ok']
        return original(**kw)
    monkeypatch.setattr(p.primitives.tester, 'motor_oem_force_abort_motion', abort)
    result = p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='direct-z-abort')
    assert not result['result']['controller_command_acknowledged']
    assert_invalid(p, store)
    assert calls == ['softwareAbort']
    assert not p._x_interrupt_active and not p._z_interrupt_active


@pytest.mark.parametrize('acknowledged', [True, False])
def test_late_x_stop_preserves_newer_abort_lifecycle(owners, monkeypatch, acknowledged):
    p, y, store, calls = owners
    state = p._load_state()
    state['x_lifecycle'].update(state='referenced_ready', reference_state='referenced')
    p._save_state(state)
    entered, release = threading.Event(), threading.Event()
    stops = []
    def stop(**kw):
        stops.append('stop')
        entered.set()
        assert release.wait(5)
        return {'ok': acknowledged, 'controller_command_acknowledged': acknowledged}
    monkeypatch.setattr(p.primitives, 'x_stop', stop)
    job = spawn(lambda: p.execute_x_stop_interrupt({'command_id': 'old-stop'}))
    try:
        assert entered.wait(3)
        assert aggregate(p)['ok']
        newer = p._load_state()['x_lifecycle']
        assert p._x_interrupt_active
    finally:
        release.set()
    result = finish(job)
    current = p._load_state()['x_lifecycle']
    assert current['state'] == 'failed_latched'
    assert current['reference_state'] == 'desynced'
    assert current['last_failure'] == newer['last_failure']
    assert not result['ok']
    assert result['authority_receipt']['status'] == 'failed'
    assert result['result']['failure'] == 'x_interrupt_superseded_by_safety_command'
    assert calls == ['softwareAbort'] and stops == ['stop']
    assert not p._x_interrupt_active
    assert_invalid(p, store)


@pytest.mark.parametrize('successful', [True, False])
@pytest.mark.parametrize('boundary', ['prepare', 'board_generation', 'admission'])
def test_reentrant_z_abort_x_prepare_early_return_is_stale(owners, monkeypatch, successful, boundary):
    p, y, store, calls = owners
    def abort():
        p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='prepare-z-abort')
    def prepare(**kw):
        if boundary == 'prepare':
            abort()
        return {'ok': successful, 'physical_motion': False, 'board_lifecycle_generation': 1}
    reads = []
    def generation():
        reads.append(1)
        if (boundary == 'board_generation' and len(reads) == 2) or (boundary == 'admission' and len(reads) == 1):
            abort()
        return 1
    monkeypatch.setattr(p.primitives, 'prepare_x', prepare)
    monkeypatch.setattr(p.preparation_provider, 'current_board_lifecycle_generation', generation)
    result = p.execute_x_intent('prepare')
    assert_invalid(p, store)
    assert not result['ok']
    assert result['result']['failure'] == 'x_intent_interrupted_by_safety_command'
    assert calls == ['softwareAbort']


@pytest.mark.parametrize('confirmed', [True, False])
def test_x_observe_home_early_return_preserves_nested_abort(owners, monkeypatch, confirmed):
    p, y, store, calls = owners
    state = p._load_state()
    state['x_lifecycle'].update(state='awaiting_operator_observation',
        awaiting_observation_receipt_id='home', reference_state='desynced')
    p._save_state(state)
    def abort(*a, **kw):
        p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='observation-abort')
        return {'ok': True, 'durable_clean': True}
    if confirmed:
        monkeypatch.setattr(p.reference_store, 'mark_referenced', abort)
    else:
        monkeypatch.setattr(p.primitives, '_x_desync', abort)
    result = p.execute_x_intent('observe_home', {'receipt_id': 'home', 'confirmed': confirmed})
    assert_invalid(p, store)
    assert not result['ok']
    assert result['failure'] == 'x_intent_interrupted_by_safety_command'
    assert calls == ['softwareAbort']


@pytest.mark.parametrize('pending', [True, False])
@pytest.mark.parametrize('boundary', ['primitive', 'completion_generation'])
def test_x_completion_preserves_newer_failure_and_both_receipts(owners, monkeypatch, pending, boundary):
    p, y, store, calls = owners
    newer, ready = [], []
    def abort():
        p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='current-abort')
        newer.append(p._load_state())
    def current(**kw):
        if boundary == 'primitive':
            abort()
        else:
            ready.append(True)
        return {'ok': True, 'pending_motion': pending}
    def generation():
        if ready:
            ready.clear()
            abort()
        return 1
    monkeypatch.setattr(p, 'generation_provider', generation)
    monkeypatch.setattr(p.primitives, 'x_enable_xyz_current_mode', current)
    result = p.execute_x_intent('enable_xyz_current', {'enabled': True, 'command_id': 'stale-current'})
    assert not result['ok']
    assert_invalid(p, store)
    state = p._load_state()
    for axis in ('x', 'z'):
        life = state[axis + '_lifecycle']
        assert life['last_failure'] == newer[0][axis + '_lifecycle']['last_failure']
        stale = [r for r in life['receipts'] if r['command_id'] == 'stale-current']
        assert stale and all(r['status'] == 'failed' and not r['result']['ok'] for r in stale)
        durable = store.read_serial206_receipt(axis, 'stale-current')
        assert durable['status'] == 'failed' and not durable['result']['ok']
        assert not durable['result'].get('pending_motion')
    assert calls == ['softwareAbort']


def test_old_abort_completion_cannot_clear_newer_failed_reconciliation_hold(owners, monkeypatch):
    p, y, store, calls = owners
    reconciled, release = threading.Event(), threading.Event()
    original = p._reconcile_aggregate_software_abort
    def reconcile(command_id, **kw):
        result = original(command_id, **kw)
        if command_id == 'old-abort':
            reconciled.set()
            assert release.wait(5)
        return result
    monkeypatch.setattr(p, '_reconcile_aggregate_software_abort', reconcile)
    old = spawn(lambda: p.execute_x_stop_interrupt({'command_id': 'old-abort'}, abort=True))
    try:
        assert reconciled.wait(3)
        runtime_reconcile = store.require_axis_reconciliation
        def fail(axis, **kw):
            if axis == 'y':
                raise RuntimeError('newer Y reconciliation failure')
            return runtime_reconcile(axis, **kw)
        monkeypatch.setattr(store, 'require_axis_reconciliation', fail)
        assert not aggregate(p)['ok']
        assert p._z_interrupt_recovery_required
    finally:
        release.set()
    finish(old)
    assert p._z_interrupt_recovery_required and p._z_interrupt_active
    assert not p.execute_z_intent('prepare', expected_generation=1, idempotency_key='failed-hold')['ok']
    assert calls == ['softwareAbort', 'softwareAbort']


def test_x_stop_completion_metadata_reentry_keeps_latest_snapshot(owners, monkeypatch):
    p, y, store, calls = owners
    state = p._load_state()
    state['x_lifecycle'].update(state='referenced_ready', reference_state='referenced')
    p._save_state(state)
    ready = []
    def stop(**kw):
        ready.append(True)
        return {'ok': True, 'controller_command_acknowledged': True}
    def generation():
        if ready:
            ready.clear()
            p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='metadata-abort')
        return 1
    monkeypatch.setattr(p.primitives, 'x_stop', stop)
    monkeypatch.setattr(p, 'generation_provider', generation)
    result = p.execute_x_stop_interrupt({'command_id': 'metadata-stop'})
    assert_invalid(p, store)
    assert not result['ok']
    assert result['result']['failure'] == 'x_interrupt_superseded_by_safety_command'
    assert calls == ['softwareAbort']


@pytest.mark.parametrize('acknowledged', [True, False])
def test_late_z_stop_keeps_abort_failure_and_reports_stale(owners, monkeypatch, acknowledged):
    p, y, store, calls = owners
    entered, release = threading.Event(), threading.Event()
    def stop(**kw):
        entered.set()
        assert release.wait(5)
        return {'ok': acknowledged, 'controller_command_acknowledged': acknowledged,
                'controller_terminal_state_verified': acknowledged}
    monkeypatch.setattr(p.primitives, 'z_stop', stop)
    job = spawn(lambda: p.execute_z_stop_interrupt(expected_generation=1, idempotency_key='old-z-stop'))
    try:
        assert entered.wait(3)
        assert aggregate(p)['ok']
        newer = p._load_state()['z_lifecycle']['last_failure']
    finally:
        release.set()
    result = finish(job)
    assert p._load_state()['z_lifecycle']['last_failure'] == newer
    assert not result['ok']
    assert result['authority_receipt']['status'] == 'failed'
    assert result['result']['failure'] == 'z_interrupt_superseded_by_safety_command'
    assert_invalid(p, store)
    assert calls == ['softwareAbort']


@pytest.mark.parametrize('abort_before_publication', [True, False])
def test_x_observation_owner_preserves_abort_and_compensates_publication(owners, monkeypatch, abort_before_publication):
    p, y, store, calls = owners
    state = p._load_state()
    x = state['x_lifecycle']
    x.update(state='awaiting_operator_observation', generation=1,
        board_lifecycle_generation=1, awaiting_observation_receipt_id='observed-home')
    x['receipts'].append({'command_id': 'observed-home', 'status': 'completed',
        'board_lifecycle_generation': 1, 'motion_kind': 'home'})
    p._save_state(state)
    original = p.reference_store.mark_referenced
    def publish(command):
        if abort_before_publication:
            p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='observation-z-abort')
        result = original(command)
        if not abort_before_publication:
            p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='observation-z-abort')
        return result
    monkeypatch.setattr(p.reference_store, 'mark_referenced', publish)
    result = p.record_x_observation(command_id='observed-home', verdict='pass',
        physical_motion_observed=True, expected_direction_observed=True,
        home_endpoint_observed=True, stopped_observed=True, note='', expected_generation=1)
    assert not result['ok']
    assert result['observation_receipt']['status'] == 'failed'
    assert not result['observation_receipt']['reference_eligible']
    assert_invalid(p, store)
    assert p.reference_store.snapshot(['x'])['rows']['x']['state'] == 'desynced'
    assert calls == ['softwareAbort']


def test_reentrant_z_abort_does_not_let_x_restore_non_x_snapshot(owners, monkeypatch):
    p, y, store, calls = owners
    def status():
        p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='nested-z-abort')
        return {'ok': True}
    monkeypatch.setattr(p.primitives, 'x_terminal_status', status)
    result = p.execute_x_intent('terminal_status')
    assert not result['ok']
    assert calls == ['softwareAbort']
    assert_invalid(p, store)


@pytest.mark.parametrize('wait_for_stop', [False, True])
def test_y_absolute_abort_during_authority_read_never_dispatches(owners, monkeypatch, wait_for_stop):
    p, y, store, calls = owners
    entered, release = threading.Event(), threading.Event()
    original = store.board4_authority_projection
    def delayed():
        entered.set()
        assert release.wait(5)
        return original()
    monkeypatch.setattr(store, 'board4_authority_projection', delayed)
    job = spawn(lambda: y.move_absolute(1200, wait_for_stop=wait_for_stop))
    try:
        assert entered.wait(3)
        assert aggregate(p)['ok']
    finally:
        release.set()
    result = finish(job)
    assert not result['ok']
    assert y.tester.calls == [], 'stale absolute motion reached primitive'
    assert calls == ['softwareAbort']


def test_second_abort_releases_waiters_while_first_is_stalled_on_storage(owners, monkeypatch):
    p, y, store, calls = owners
    first, second = threading.Event(), threading.Event()
    original = p.primitives.tester.motor_oem_force_abort_motion
    def abort(**kw):
        result = original(**kw)
        (first if len(calls) == 1 else second).set()
        return result
    monkeypatch.setattr(p.primitives.tester, 'motor_oem_force_abort_motion', abort)
    with store._lock:
        a = spawn(lambda: aggregate(p))
        assert first.wait(3)
        b = spawn(lambda: aggregate(p))
        second_before_storage_release = second.wait(3)
    finish(a)
    finish(b)
    assert second_before_storage_release, 'second cancellation waited behind first persistence'
    assert calls == ['softwareAbort', 'softwareAbort']
    assert_invalid(p, store)
    assert not p._x_interrupt_active and not p._z_interrupt_active


def test_concurrent_aggregate_and_z_owner_reentry_have_no_lock_cycle(owners, monkeypatch):
    p, y, store, calls = owners
    entered, first_cancelled = threading.Event(), threading.Event()
    original = p.primitives.tester.motor_oem_force_abort_motion
    def abort(**kw):
        result = original(**kw)
        first_cancelled.set()
        return result
    monkeypatch.setattr(p.primitives.tester, 'motor_oem_force_abort_motion', abort)
    monkeypatch.setattr(p.primitives, 'current_board_lifecycle_generation', lambda: 1)
    def move(**kw):
        entered.set()
        assert first_cancelled.wait(5)
        assert aggregate(p)['ok']  # Reentry while this real Z owner holds _lock.
        return {'ok': True, 'controller_command_acknowledged': True, 'controller_terminal_state_verified': True}
    monkeypatch.setattr(p.primitives, 'z_move_steps', move)
    z = spawn(lambda: p.execute_z_intent('move_steps', inputs={'steps': 10}, expected_generation=1, idempotency_key='reentrant-z'))
    assert entered.wait(3)
    a = spawn(lambda: aggregate(p))
    assert not finish(z)['ok']
    assert finish(a)['ok']
    assert calls == ['softwareAbort', 'softwareAbort']
    assert_invalid(p, store)
    assert not p._x_interrupt_active and not p._z_interrupt_active


def test_bound_api_abort_traverses_owner_once(owners, monkeypatch):
    import asyncio
    import bioxp.api as api
    from bioxp import operator_controls
    p, y, store, calls = owners
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', p)
    monkeypatch.setattr(p.primitives, 'capability_status', lambda: {'initialize_motors_exact_primitives_bound': True})
    monkeypatch.setattr(api, '_get_tester', lambda: p.primitives.tester)
    monkeypatch.setattr(api, '_tester_transition_lock', asyncio.Lock())
    token = operator_controls._DISPATCH_CONTEXT.set({
        'operator_command_id': 'offline-api-abort', 'idempotency_key': 'offline-api',
        'expected_ownership_generation': 1, 'action_id': 'oem.abort_all',
    })
    try:
        result = asyncio.run(api.motion_oem_x_abort())
    finally:
        operator_controls._DISPATCH_CONTEXT.reset(token)
    assert result['ok'] and result['software_abort']
    assert not result['stop_delivery_attempted']
    assert calls == ['softwareAbort']
    assert_invalid(p, store)


def test_runtime_reference_expected_epoch_rejects_old_home_after_reprepare(owners):
    p, y, store, calls = owners
    old = store.board4_authority_projection()['axes']['y']['interrupt_epoch']
    store.require_axis_reconciliation('y', receipt_id='separate-owner-interrupt')
    assert y.prepare()['ok']
    result = store.publish_axis_reference('y', ownership_generation=1, position_steps=0, expected_interrupt_epoch=old)
    assert not result['ok']
    assert store.board4_authority_projection()['axes']['y']['reference_state'] == 'unreferenced'


def test_z_reference_reentrant_abort_cannot_republish_stale_authority(owners, monkeypatch):
    p, y, store, calls = owners
    original = p.reference_store.mark_referenced
    def crossing(command):
        assert aggregate(p)['ok']
        return original(command)
    monkeypatch.setattr(p.reference_store, 'mark_referenced', crossing)
    monkeypatch.setattr(p.primitives, 'current_board_lifecycle_generation', lambda: 1)
    monkeypatch.setattr(p.primitives, 'z_terminal_status', lambda: {'ok': True, 'position_steps': 0, 'speed_steps_s': 0})
    monkeypatch.setattr(p.primitives, 'z_set_home', lambda: {'ok': True, 'controller_command_acknowledged': True,
        'controller_terminal_state_verified': True, 'source_return_code': 0})
    result = finish(spawn(lambda: p.execute_z_intent('set_home', expected_generation=1, idempotency_key='late-reference')))
    assert not result['ok'], result
    assert_invalid(p, store)
    assert calls == ['softwareAbort']

@pytest.mark.parametrize('boundary', ['inside', 'finalizer'])
@pytest.mark.parametrize('direct_z', [False, True])
def test_recovery_publication_is_generation_owned_and_retryable(owners, monkeypatch, boundary, direct_z):
    import sys
    p, y, store, calls = owners
    paused, release = threading.Event(), threading.Event()
    original = p._reconcile_aggregate_software_abort
    def reconcile(command_id, **kw):
        result = original(command_id, **kw)
        if boundary == 'finalizer' and command_id == 'older':
            paused.set()
            assert release.wait(5)
        return result
    monkeypatch.setattr(p, '_reconcile_aggregate_software_abort', reconcile)
    def old():
        def trace(frame, event, arg):
            if (boundary == 'inside' and event == 'line'
                    and frame.f_code.co_name == '_reconcile_aggregate_software_abort'
                    and frame.f_locals.get('command_id') == 'older'
                    and 'state' in frame.f_locals
                    and frame.f_lineno > original.__func__.__code__.co_firstlineno
                    and __import__('linecache').getline(frame.f_code.co_filename, frame.f_lineno).startswith('        if invalidate_x:')):
                paused.set()
                assert release.wait(5)
            return trace
        sys.settrace(trace)
        try:
            if direct_z:
                return p.execute_z_stop_interrupt(inputs={'command_id': 'older'}, abort=True, expected_generation=1, idempotency_key='older')
            return p.execute_x_stop_interrupt({'command_id': 'older'}, abort=True)
        finally:
            sys.settrace(None)
    job = spawn(old)
    runtime = store.require_axis_reconciliation
    try:
        assert paused.wait(3)
        def fail(axis, **kw):
            if axis == 'y':
                raise RuntimeError('newer Y persistence failure')
            return runtime(axis, **kw)
        monkeypatch.setattr(store, 'require_axis_reconciliation', fail)
        assert not p.execute_z_stop_interrupt(abort=True, expected_generation=1, idempotency_key='newer')['ok']
    finally:
        release.set()
    finish(job)
    assert p._x_interrupt_recovery_required and p._x_interrupt_active
    assert p._z_interrupt_recovery_required and p._z_interrupt_active
    assert all(store.axis_interrupt_snapshot(a)['active'] for a in ('y', 'z', 'gripper'))
    assert not y.prepare()['ok']
    assert not p.execute_x_intent('prepare', {'command_id': 'blocked-x'})['ok']
    assert not p.execute_z_intent('prepare', expected_generation=1, idempotency_key='blocked-z')['ok']
    monkeypatch.setattr(store, 'require_axis_reconciliation', runtime)
    retry = p._reconcile_aggregate_software_abort('retry', invalidate_x=True)
    assert retry['ok'] and retry['controller_dispatches'] == 0
    assert not p._x_interrupt_active and not p._z_interrupt_active
    assert all(not store.axis_interrupt_snapshot(a)['active'] for a in ('y', 'z', 'gripper'))
    assert y.prepare()['ok']
    assert calls == ['softwareAbort', 'softwareAbort']


@pytest.mark.parametrize('crossing', ['before', 'after', 'failure'])
def test_z_observation_receipt_matches_failed_reference_publication(owners, monkeypatch, crossing):
    p, y, store, calls = owners
    monkeypatch.setattr(p.primitives, 'current_board_lifecycle_generation', lambda: 1)
    receipt = {'command_id': 'observed-home', 'receipt_id': 'observed-home', 'intent': 'home',
               'status': 'completed', 'board_lifecycle_generation': 1,
               'result': {'ok': True, 'controller_terminal_state_verified': True,
                          'controller_command_acknowledged': True}}
    state = p._load_state()
    state['z_lifecycle'].update(state='awaiting_operator_observation', generation=1,
        board_lifecycle_generation=1, awaiting_observation_receipt_id='observed-home', receipts=[receipt])
    p._save_state(state)
    original = p.reference_store.mark_referenced
    def publish(command):
        if crossing == 'failure':
            raise RuntimeError('reference storage unavailable')
        if crossing == 'before':
            assert aggregate(p)['ok']
        result = original(command)
        if crossing == 'after':
            assert aggregate(p)['ok']
        return result
    monkeypatch.setattr(p.reference_store, 'mark_referenced', publish)
    result = p.record_z_observation(command_id='observed-home', observation_command_id='crossed-observation',
        verdict='pass', physical_motion_observed=True, expected_direction_observed=True,
        home_endpoint_observed=True, stopped_observed=True, note='offline', expected_generation=1)
    assert not result['ok']
    durable = store.read_serial206_receipt('z', 'crossed-observation')
    for row in (result['observation_receipt'], durable):
        assert row['status'] == 'failed' and row['reference_eligible'] is False
    assert result['observation']['reference_eligible'] is False
    assert p._load_state()['z_lifecycle']['state'] == 'failed_latched'
    assert calls == ([] if crossing == 'failure' else ['softwareAbort'])


@pytest.mark.parametrize('direct_z', [False, True])
def test_persistence_retry_before_failed_finalizer_does_not_relatch(owners, monkeypatch, direct_z):
    p, y, store, calls = owners
    paused, release = threading.Event(), threading.Event()
    runtime = store.require_axis_reconciliation
    original = p._reconcile_aggregate_software_abort
    def fail(axis, **kw):
        if axis == 'y':
            raise RuntimeError('Y unavailable')
        return runtime(axis, **kw)
    monkeypatch.setattr(store, 'require_axis_reconciliation', fail)
    def reconcile(command_id, **kw):
        result = original(command_id, **kw)
        if command_id == 'failed-old':
            paused.set()
            assert release.wait(5)
        return result
    monkeypatch.setattr(p, '_reconcile_aggregate_software_abort', reconcile)
    def abort():
        if direct_z:
            return p.execute_z_stop_interrupt(inputs={'command_id': 'failed-old'}, abort=True,
                expected_generation=1, idempotency_key='failed-old')
        return p.execute_x_stop_interrupt({'command_id': 'failed-old'}, abort=True)
    job = spawn(abort)
    try:
        assert paused.wait(3)
        monkeypatch.setattr(store, 'require_axis_reconciliation', runtime)
        assert p._reconcile_aggregate_software_abort('persistence-retry', invalidate_x=True)['ok']
        assert not p._x_interrupt_recovery_required and not p._z_interrupt_recovery_required
    finally:
        release.set()
    finish(job)
    assert not p._x_interrupt_active and not p._z_interrupt_active
    assert all(not store.axis_interrupt_snapshot(a)['active'] for a in ('y', 'z', 'gripper'))
    assert y.prepare()['ok']
    assert calls == ['softwareAbort']


def test_failed_persistence_only_retry_keeps_all_runtime_holds(owners, monkeypatch):
    p, y, store, calls = owners
    save = p._save_state
    monkeypatch.setattr(p, '_save_state', lambda *a: (_ for _ in ()).throw(RuntimeError('lifecycle unavailable')))
    assert not p._reconcile_aggregate_software_abort('failed-retry', invalidate_x=True)['ok']
    assert p._x_interrupt_active and p._z_interrupt_active
    assert all(store.axis_interrupt_snapshot(a)['active'] for a in ('y', 'z', 'gripper'))
    assert not y.prepare()['ok']
    monkeypatch.setattr(p, '_save_state', save)
    assert p._reconcile_aggregate_software_abort('successful-retry', invalidate_x=True)['ok']
    assert not p._x_interrupt_active and not p._z_interrupt_active
    assert all(not store.axis_interrupt_snapshot(a)['active'] for a in ('y', 'z', 'gripper'))
    assert y.prepare()['ok']
    assert calls == []


def test_z_observation_annotation_lookup_crossing_preserves_newer_owner(owners, monkeypatch):
    p, y, store, calls = owners
    state = p._load_state()
    state['z_lifecycle']['receipts'] = [{'command_id': 'historical-z', 'status': 'failed', 'intent': 'home'}]
    p._save_state(state)
    lookup = p._durable_serial206_receipt
    newer = []
    def read(axis, command_id):
        if command_id == 'crossed-annotation':
            assert aggregate(p)['ok']
            newer.append(p._load_state()['z_lifecycle'])
        return lookup(axis, command_id)
    monkeypatch.setattr(p, '_durable_serial206_receipt', read)
    result = p.record_z_observation(command_id='historical-z', observation_command_id='crossed-annotation',
        verdict='pass', physical_motion_observed=True, expected_direction_observed=True,
        home_endpoint_observed=True, stopped_observed=True, note='offline', expected_generation=1)
    assert not result['ok']
    for row in (result['observation_receipt'], store.read_serial206_receipt('z', 'crossed-annotation')):
        assert row['status'] == 'failed' and not row['reference_eligible']
    state = p._load_state()['z_lifecycle']
    assert state['last_failure'] == newer[0]['last_failure']
    assert state['state'] == 'failed_latched'
    assert calls == ['softwareAbort']


def test_late_failed_persistence_attempt_cannot_relatch_successful_retry(owners, monkeypatch):
    import linecache
    import sys
    p, y, store, calls = owners
    paused, release = threading.Event(), threading.Event()
    runtime = store.require_axis_reconciliation
    def fail(axis, **kw):
        if kw['receipt_id'] == 'old-persistence' and axis == 'y':
            raise RuntimeError('old persistence failure')
        return runtime(axis, **kw)
    monkeypatch.setattr(store, 'require_axis_reconciliation', fail)
    def old():
        def trace(frame, event, arg):
            if (event == 'line' and frame.f_code.co_name == '_reconcile_aggregate_software_abort'
                    and frame.f_locals.get('command_id') == 'old-persistence'
                    and 'state' in frame.f_locals
                    and linecache.getline(frame.f_code.co_filename, frame.f_lineno).startswith('        if invalidate_x:')):
                paused.set()
                assert release.wait(5)
            return trace
        sys.settrace(trace)
        try:
            return p._reconcile_aggregate_software_abort('old-persistence', invalidate_x=True)
        finally:
            sys.settrace(None)
    job = spawn(old)
    try:
        assert paused.wait(3)
        assert p._reconcile_aggregate_software_abort('new-persistence', invalidate_x=True)['ok']
    finally:
        release.set()
    assert not finish(job)['ok']
    assert not p._x_interrupt_active and not p._z_interrupt_active
    assert all(not store.axis_interrupt_snapshot(a)['active'] for a in ('y', 'z', 'gripper'))
    assert y.prepare()['ok']
    assert calls == []


def test_old_partial_reconciliation_cannot_claim_newer_runtime_generation(owners, monkeypatch):
    p, y, store, calls = owners
    paused, release = threading.Event(), threading.Event()
    runtime = store.require_axis_reconciliation
    def reconcile(axis, **kw):
        if kw['receipt_id'] == 'partial-old' and axis == 'y':
            paused.set()
            assert release.wait(5)
        if kw['receipt_id'] == 'new-failure' and axis == 'y':
            raise RuntimeError('new Y failure')
        return runtime(axis, **kw)
    monkeypatch.setattr(store, 'require_axis_reconciliation', reconcile)
    job = spawn(lambda: p.execute_z_stop_interrupt(inputs={'command_id': 'partial-old'}, abort=True,
        expected_generation=1, idempotency_key='partial-old'))
    try:
        assert paused.wait(3)
        assert not p.execute_z_stop_interrupt(inputs={'command_id': 'new-failure'}, abort=True,
            expected_generation=1, idempotency_key='new-failure')['ok']
    finally:
        release.set()
    finish(job)
    assert p._x_interrupt_active and p._z_interrupt_active
    assert all(store.axis_interrupt_snapshot(a)['active'] for a in ('y', 'z', 'gripper'))
    assert not y.prepare()['ok']
    assert p._reconcile_aggregate_software_abort('covering-retry', invalidate_x=True)['ok']
    assert not p._x_interrupt_active and not p._z_interrupt_active
    assert all(not store.axis_interrupt_snapshot(a)['active'] for a in ('y', 'z', 'gripper'))
    assert y.prepare()['ok']
    assert calls == ['softwareAbort', 'softwareAbort']
