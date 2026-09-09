"""Cross-layer regressions; real driver waits/SQLite/fence and routed handlers."""
import asyncio
import json
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from test_oem_abort_g_repair import driver, wire_event
from test_oem_deck_command_queue import _operator_store, _state


@pytest.mark.parametrize('version', [0, 1])
def test_manual_uninitialized_normal_scalar_http_noop(monkeypatch, version):
    from bioxp.oem_gripper import gripper_home
    d = driver(); calls = []
    d._oem_board_state = lambda: {4: False}
    d._motion_oem_axis_profile = lambda *a, **k: {'board': 4, 'motor': 2, 'gripper_version': version}
    d.motor_set_axis_param = lambda b, p, v, **k: calls.append((b, p, v, k['motor'])) or {'ok': True}
    d._send_motor = lambda *a, **k: pytest.fail('uninitialized goHome must not transmit')
    app = FastAPI()
    @app.post('/home')
    def home():
        return gripper_home(d, operator_ack='GRIPPER_HOME', reason='offline scalar regression')
    response = TestClient(app).post('/home')
    assert response.status_code == 200, response.text
    data = response.json()
    assert data['source_return_code'] == 1 and data['motion_commanded'] is False
    assert data['physical_effect_verified'] is False
    assert data['acceptance']['home_payload_ok'] is False
    assert calls == [(4, 6, 31, 2)] + ([(4, 6, 10, 2)] if version == 1 else [])


@pytest.mark.parametrize('fallback', [False, True])
@pytest.mark.parametrize('kind', ['single', 'pair', 'sequential'])
@pytest.mark.parametrize('signal', ['initial', 'abort'])
@pytest.mark.parametrize('timeout', [0, .001])
def test_software_latches_continue_without_target_proof(fallback, kind, signal, timeout):
    d = driver(); targets = {(5, 0), (4, 0)}
    d._oem_motor_initial_signals = set(targets) if signal == 'initial' else set()
    if fallback:
        d.novo_router = None
        d._oem_abort_signals = set(targets) if signal == 'abort' else set()
    elif signal == 'abort':
        for b, m in targets: d.novo_router.set_motor_abort_event(b, m)
    if kind == 'single':
        result = d.motor_oem_wait_target_reached(5, 0, timeout_s=timeout)
        waits = [result]
    else:
        result = d.motor_wait_target_reached_many(sorted(targets), timeout_s=timeout, sta_sequential=kind == 'sequential')
        waits = list(result['per_axis'].values())
    assert result['ok'] is True
    for wait in waits:
        assert wait['source_wait_signaled'] is True
        assert wait['target_reached'] is False and wait['event'] is None
        assert wait['physical_effect_verified'] is False


def test_g_blocks_real_deck_child_before_reconciliation(tmp_path):
    from test_oem_deck_command_queue import _request, _bootstrap_named_store
    store = _operator_store(tmp_path)
    try:
        _bootstrap_named_store(store)
        admitted = store.admit_command(_request(), state=_state())
        assert store.claim_next() is not None
        store.assert_deck_execution_current(admitted['command_id'])
        store.mark_interrupt_delivery_active('g-stop', 'oem.g.stop')
        # No begin_interrupt/SQLite invalidation has run yet: immediate barrier.
        with pytest.raises(RuntimeError, match='interrupt_fence_active'):
            store.assert_deck_execution_current(admitted['command_id'])
        assert store.claim_next() is None
    finally: store.stop()


@pytest.mark.parametrize('failure_stage', ['admission', 'finalization'])
def test_software_abort_recovery_receipt_and_spool_no_stop(tmp_path, monkeypatch, failure_stage):
    from test_operator_controls import make_app
    app, calls = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    store = plane.store
    target = 'begin_interrupt' if failure_stage == 'admission' else 'finalize_interrupt'
    original = getattr(store, target)
    def fail(*a, **kw): raise RuntimeError('offline persistence fault')
    monkeypatch.setattr(store, target, fail)
    response = asyncio.run(plane.compat_invoke('oem.abort_all', {'idempotency_key': 'fallback'}))
    assert calls == [('abort_all', None)]
    assert response['recovery_hold'] is True
    assert response['invocation_attempted'] is True
    assert response['stop_delivery_attempted'] is False and response['controller_stop_attempted'] is False
    assert response['physical_scope'] == 'none_software_flags_and_waiters'
    monkeypatch.setattr(store, target, original)
    assert store.reconcile_pending_interrupts() == 1
    assert calls == [('abort_all', None)]
    assert not store.action_fenced('oem.z.home_gz')


@pytest.mark.parametrize('fallback', [False, True])
def test_provider_initial_wait_source_success_not_verified(fallback):
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    d = driver(); d._oem_motor_initial_signals = {(5, 0)}
    if fallback: d.novo_router = None
    p = object.__new__(Serial206ProductionPrimitiveAdapter); p.tester = d; p.reference_store = None
    result = p._x_finalize({'ok': True, 'command_issued': True}, timeout_s=.001, motion_kind='test')
    assert result['ok'] is True and result['source_wait_signaled'] is True
    assert result['wait_verified'] is False
    assert result['controller_terminal_state_verified'] is False


def test_wire_event_still_observed():
    d = driver(); wire_event(d.novo_router, 5, 0)
    result = d.motor_oem_wait_target_reached(5, 0, timeout_s=0)
    assert result['target_reached'] is True and result['event']['status'] == 128


@pytest.mark.parametrize('action', ['oem.abort_all', 'oem.z.abort'])
def test_software_abort_attempt_and_durable_terminal_no_stop(tmp_path, action):
    store = _operator_store(tmp_path)
    try:
        receipt = store.begin_interrupt(action, state=_state(), request={'idempotency_key': 'software'})
        attempted = store.mark_interrupt_attempted(idempotency_key='software')
        assert attempted['invocation_attempted'] is True
        assert attempted['controller_stop_attempted'] is False
        terminal = store.finalize_interrupt(idempotency_key='software', receipt=receipt, attempted=True, acknowledged=True,
            response={'ok': True, 'source_call_completed': True, 'controller_command_acknowledged': False})
        assert terminal['software_abort'] is True
        assert terminal['stop_delivery_attempted'] is False and terminal['controller_stop_attempted'] is False
        assert terminal['physical_scope'] == 'none_software_flags_and_waiters'
        replay = store.begin_interrupt(action, state=_state(), request={'idempotency_key': 'software'}, interrupt_attempt_id=receipt['interrupt_id'])
        assert replay['invocation_attempted'] is True and replay['controller_stop_attempted'] is False
    finally: store.stop()


def test_api_and_z_primitive_software_abort_no_tx(monkeypatch):
    import bioxp.api as api
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    d = driver(); d._send_motor = lambda *a, **k: pytest.fail('software Abort cannot transmit Stop')
    p = object.__new__(Serial206ProductionPrimitiveAdapter); p.tester = d; p.reference_store = None
    monkeypatch.setattr(api, '_execute_provider_x_intent', lambda intent, values: p.x_abort())
    async def execute(label, fn, **kw): return fn(d)
    monkeypatch.setattr(api, '_run_safety_interrupt_blocking', execute)
    app = FastAPI(); app.post('/abort')(api.motion_oem_x_abort)
    response = TestClient(app).post('/abort')
    assert response.status_code == 200
    assert response.json()['physical_scope'] == 'none_software_flags_and_waiters'
    assert p.z_abort()['physical_scope'] == 'none_software_flags_and_waiters'


@pytest.mark.parametrize('other', ['oem.g.stop', 'oem.abort_all', 'oem.z.abort'])
def test_g_overlap_pending_restart_and_release_preserves_aggregate_fence(tmp_path, other):
    store = _operator_store(tmp_path)
    store.mark_interrupt_delivery_active('g', 'oem.g.stop')
    store.mark_interrupt_delivery_active('other', other)
    for action in ['oem.x.move_steps', 'oem.y.move_absolute', 'oem.z.move_gz', 'oem.z.home_gz', 'oem.deck.move_to_location']:
        assert store.action_fenced(action)
    store.mark_interrupt_delivery_inactive('g', 'oem.g.stop')
    assert store.release_interrupt_fence('oem.g.stop') is False
    store.queue_pending_interrupt_reconciliation({'action_id': other, 'state': _state(),
        'request': {'idempotency_key': 'pending'}, 'interrupt_attempt_id': 'other',
        'attempted': True, 'acknowledged': True, 'response': {'ok': True, 'controller_command_acknowledged': False}})
    store.mark_interrupt_delivery_inactive('other', other)
    assert store.release_interrupt_fence('oem.g.stop') is False
    store.connection.execute('UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1')
    store.connection.close()
    second = _operator_store(tmp_path)
    try:
        assert second.action_fenced('oem.z.home_gz')
        assert second.reconcile_pending_interrupts() == 1
        assert not second.action_fenced('oem.z.home_gz')
    finally: second.stop()


@pytest.mark.parametrize('action', ['oem.x.move_steps', 'oem.z.move_gz', 'oem.z.home_gz'])
def test_g_invalidates_existing_conflict_roots_and_epochs(tmp_path, action):
    from bioxp.operator_command_plane import ACTION_REQUEST_SCHEMA
    store = _operator_store(tmp_path)
    try:
        # Real admission/claim; axis G is absent from these resource roots.
        admitted = store.admit_command({'schema_version': ACTION_REQUEST_SCHEMA, 'idempotency_key': 'move',
            'expected_ownership_generation': 7, 'expected_board_epoch_by_board': {},
            'action_id': action, 'inputs': ({'steps': 1} if action == 'oem.x.move_steps' else {'gripper_position_steps': 100, 'z_position_steps': 100} if action == 'oem.z.move_gz' else {'reason': 'offline fence test'})}, state=_state())
        assert store.claim_next() is not None
        before = dict(store.connection.execute('SELECT * FROM operator_plane_safety').fetchone())
        receipt = store.begin_interrupt('oem.g.stop', state=_state(), request={'idempotency_key': 'g'})
        assert admitted['command_id'] in receipt['active_command_ids']
        assert receipt['scope'] == 'g' and receipt['oem_abort_latched'] is False
        assert receipt['fence_scope'] == 'aggregate'
        for axis in ['global', 'x', 'y', 'z']:
            assert receipt[axis + '_safety_epoch'] == before[axis + '_epoch'] + 1
        rows = store.connection.execute("SELECT * FROM serial206_axis_authority WHERE axis IN ('y','z','gripper')").fetchall()
        assert rows and all(row['reference_state'] == 'reconciliation_required' for row in rows)
    finally: store.stop()


def test_routed_diagnostic_g_is_fenced_before_exactly_one_dispatch(monkeypatch, tmp_path):
    from test_operator_controls import make_app, action_for
    from bioxp.operator_command_plane import OperatorCommandStore
    app, calls = make_app(tmp_path, monkeypatch)
    store = app.state.operator_command_plane.store
    # Existing routed handler replaced only at device-facing leaf; real operator
    # route and reconciliation deliver controller_delivery exactly once.
    route = next(r for r in app.routes if getattr(r, 'path', '') == '/motion/diagnostics/stop')
    import bioxp.api as api
    d = driver(); packets = []
    d._send_motor = lambda b, c, t, m, v, **kw: packets.append((b, c, t, m, v)) or {'status': 100}
    monkeypatch.setattr(api, 'restore_gripper_idle_current', lambda *a, **kw: {'ok': True})
    monkeypatch.setattr(api, '_collect_axis_diagnostic_status', lambda *a, **kw: {'rows': {'g': {
        'speed': {'speed': 0, 'speed_reply_valid': True, 'ack': {'status': 100}},
        'current': {'run_current_param6': 10, 'standby_current_param7': 10}}}})
    async def execute(label, fn, **kw): return fn(d)
    monkeypatch.setattr(api, '_run_safety_interrupt_blocking', execute)
    async def checked(*args, **kwargs):
        assert store.action_fenced('oem.deck.move_to_location')
        assert store.action_fenced('oem.z.home_gz')
        calls.append(('diagnostic_stop', None))
        return await api.motion_diagnostics_stop(api.AxisDiagnosticStopRequest(axis='g', operator_ack='STOP_AXIS'))
    route.dependant.call = checked
    client = TestClient(app)
    catalog = client.get('/operator/control-catalog').json()
    action = action_for(catalog, 'POST', '/motion/diagnostics/stop')
    response = client.post('/operator/actions/' + action['action_id'], json={'expected_generation': catalog['ownership_generation'],
        'idempotency_key': 'routed-g', 'inputs': {'axis': 'g'}})
    assert response.status_code == 200, response.text
    assert calls == [('diagnostic_stop', None)]
    # One routed Stop invocation means exactly the source's two explicit MSTs.
    assert packets == [(4, 3, 0, 2, 0), (4, 3, 0, 2, 0)]
    assert not d.oem_no24v_state()
