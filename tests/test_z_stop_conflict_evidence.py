"""Offline qualification: actual Stop producer, HTTP consumer and SQLite.

Uses the repository's retained test_operator_controls / test_z_stop_outer_lease
fixtures. Run in an isolated network namespace; never initialize BioXpTester.
"""
import asyncio
import copy
import json
import os
import sqlite3
import subprocess
import sys
import threading
from pathlib import Path

import pytest
from fastapi import HTTPException
from fastapi.testclient import TestClient

from bioxp import api, operator_controls
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider, Serial206ProductionPrimitiveAdapter
from bioxp.services.reference_service import ReferenceStateStore
from bioxp.usb_driver import BioXpTester
from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import novo_decode
from tests.z_stop_fixtures import HardwareEndpoints, make_app


@pytest.fixture
def producer(tmp_path, monkeypatch):
    hardware = HardwareEndpoints()
    tester = BioXpTester.__new__(BioXpTester)
    tester._transport_lock = threading.RLock()
    tester._motor_last_tx_ts = {}
    tester._motor_noresp_streak = {}
    tester._oem_board_initialized = {4: True}
    tester._oem_active_board_lifecycle_generation = 1
    monkeypatch.setattr(tester, '_oem_board_present', lambda board: True)
    monkeypatch.setattr(tester, 'oem_no24v_state', lambda: False)
    from types import SimpleNamespace
    monkeypatch.setattr(tester, '_machine_config_bundle', lambda: {'ok': True, 'config': {
        'config': {'GripperVersion': 1}, 'calibration': {'Calibrated': 1},
        'axis_limits': {'z': {'max_steps': 160000}}, 'offsets': {
            'm_Z_MOTOR_MAX_CURRENT_DOWN': 25, 'm_Z_MOTOR_MAX_CURRENT_UP': 31,
            'm_Z_MOTOR_STALL_GUARD_THRESHOLD': 3}}})
    monkeypatch.setattr('bioxp.oem_serial206_initialization.load_oem_parity_config', lambda _: SimpleNamespace(
        blockers=[], values={'SerialNumber': 206, 'CameraCalibrated': False}, calibration_source='offline-fixture'))
    router = NovoRouter(ep_in=hardware, ep_out=hardware, decode=novo_decode, read_timeout_ms=10)
    tester.novo_router = router
    store = OEMRuntimeStore(tmp_path / 'producer')
    references = ReferenceStateStore(tmp_path / 'producer' / 'references.json')
    adapter = Serial206ProductionPrimitiveAdapter(tester, None, authority_provider=lambda: None,
        generation_provider=lambda: 1, reference_store=references)
    provider = Serial206OemInitializationProvider(adapter, state_store=store,
        reference_store=references, generation_provider=lambda: 1)
    router.start()
    try:
        yield provider, tester, hardware
    finally:
        router.shutdown()
        store.close()


def test_reported_http409_replay_keeps_ack_without_inventing_source_return(tmp_path, monkeypatch):
    recorded = json.loads((Path(__file__).parent / 'fixtures/z_stop_reported_http409.json').read_text())
    app, _ = make_app(tmp_path, monkeypatch)
    result = asyncio.run(app.state.operator_command_plane.compat_invoke('oem.z.stop',
        {'idempotency_key': 'reported-stop', 'observed_ownership_generation': 1},
        controller_delivery=(recorded['http_status'], recorded['body'])))
    assert result['controller_stop_acknowledged'] is True
    assert result['source_call_completed'] is True
    # Historical lifecycle overwrote ok and did not retain source_return_ok.
    assert result['source_return_ok'] is None
    assert result['controller_terminal_state_verified'] is False
    assert result['error'] == 'controller_interrupt_http_409'


def test_real_double_delivery_does_not_invent_terminal_observation(producer):
    provider, tester, hardware = producer
    result = provider.execute_z_stop_interrupt(inputs={'command_id': 'raw-stop'}, expected_generation=1, idempotency_key='raw-stop')
    source = result['result']
    assert hardware.stop_writes == 2
    assert source['stop']['double_stop_acknowledged'] is True
    assert source['controller_command_acknowledged'] is True
    assert source['source_call_completed'] is True
    assert source['source_return_ok'] is True
    assert source['wait'] is None
    assert source['controller_terminal_state_verified'] is False
    assert source['physical_effect_verified'] is False
    assert result['ok'] is True
    assert result['authority_receipt']['status'] == 'completed'
    assert result['z_lifecycle']['reference_state'] == 'desynced'


@pytest.mark.parametrize('first,second', [(True, True), (False, True), (True, False), (False, False)])
def test_partial_delivery_preserves_void_source_return(producer, monkeypatch, first, second):
    provider, tester, hardware = producer
    hardware.stop_statuses = iter([100 if first else 2, 100 if second else 2])
    delivered = provider.execute_z_stop_interrupt(inputs={'command_id': 'acks'}, expected_generation=1, idempotency_key='acks')
    source = delivered['result']
    assert delivered['ok'] is second
    assert delivered['authority_receipt']['status'] == ('completed' if second else 'failed')
    assert source['source_return_ok'] is True
    assert source['controller_command_acknowledged'] is second
    assert source['stop']['double_stop_acknowledged'] is (first and second)
    assert source['first_stop_acknowledged'] is first
    assert source['second_stop_acknowledged'] is second
    persisted = provider.state_store.read_serial206_receipt('z', 'acks')
    assert persisted['result']['first_stop_acknowledged'] is first
    assert persisted['result']['second_stop_acknowledged'] is second
    assert source['controller_terminal_state_verified'] is False
    assert source['wait'] is None


@pytest.mark.parametrize('terminal', [True, False, None], ids=['verified-readback', 'still-moving-or-failed', 'missing-readback'])
def test_http_conflict_retains_independent_evidence(tmp_path, monkeypatch, producer, terminal):
    provider, tester, hardware = producer
    delivered = provider.execute_z_stop_interrupt(inputs={'command_id': 'http-stop'}, expected_generation=1, idempotency_key='http-stop')
    # Consumer qualification only: a separately supplied terminal observation is
    # not added to OEM Stop. Actual producer case above remains unverified.
    if terminal is None:
        delivered['result'].pop('controller_terminal_state_verified')
    else:
        delivered['result']['controller_terminal_state_verified'] = terminal
    app, _ = make_app(tmp_path / 'outer', monkeypatch)
    plane = app.state.operator_command_plane
    receipt = asyncio.run(plane.compat_invoke('oem.z.stop', {
        'idempotency_key': 'conflict', 'observed_ownership_generation': 1,
    }, controller_delivery=(409, {'detail': delivered})))
    assert receipt['persistence_state'] == 'committed'
    assert receipt['source_call_completed'] is True
    assert receipt['source_return_ok'] is True
    assert receipt['controller_stop_acknowledged'] is True
    assert receipt['controller_terminal_state_verified'] is (terminal is True)
    assert receipt['physical_effect_verified'] is False
    assert receipt['error'] == 'controller_interrupt_http_409'
    db = plane.store.path
    code = "import sqlite3,json,sys; c=sqlite3.connect(sys.argv[1]); print(c.execute(\"SELECT receipt_json FROM operator_plane_interrupt_attempts WHERE phase='terminal' ORDER BY rowid DESC LIMIT 1\").fetchone()[0])"
    # Fresh-process query against the actual on-disk schema (not in-memory pool).
    columns = plane.store.connection.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE '%interrupt%'").fetchall()
    assert columns
    saved = plane.store.connection.execute("SELECT receipt_json FROM operator_plane_interrupt_attempts WHERE phase='terminal' ORDER BY rowid DESC LIMIT 1").fetchone()
    persisted = json.loads(saved[0])
    assert persisted['controller_stop_acknowledged'] is True
    fresh = json.loads(subprocess.check_output([sys.executable, '-c', code, str(db)], text=True))
    assert fresh == persisted


@pytest.mark.parametrize('armed', [True, False], ids=['armed', 'unarmed'])
@pytest.mark.parametrize('first', [True, False], ids=['both-acks', 'second-ack-only'])
def test_actual_route_and_compact_detail_keep_source_completion_truth(tmp_path, monkeypatch, producer, armed, first):
    provider, tester, hardware = producer
    hardware.stop_statuses = iter([100 if first else 2, 100])
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    monkeypatch.setattr(api, '_tester', tester)
    monkeypatch.setattr(api, '_tester_transition_lock', asyncio.Lock())
    app, _ = make_app(tmp_path / 'outer', monkeypatch, z_stop_route=api.motion_oem_z_stop, armed=armed, generation=1)
    client = TestClient(app)
    response = client.post('/operator/v2/actions/oem.z.stop', json={
        'schema_version': 'bioxp.operator_interrupt_request.v1',
        'idempotency_key': 'actual-route-stop', 'reason': 'offline qualification',
        'observed_ownership_generation': 1, 'observed_board_epoch_by_board': {},
    })
    assert response.status_code == 200, response.text
    row = response.json()
    assert hardware.stop_writes == 2, row
    assert row['status'] == 'completed', row
    captures = {}
    for detail in (False, True):
        projected = client.get(row['status_path'], params={'detail': detail}).json()
        assert projected['status'] == 'completed', projected
        assert projected['error'] is None
        evidence = projected['interrupt_evidence']
        assert evidence['source_call_completed'] is True
        assert evidence['source_return_ok'] is True
        assert evidence['first_stop_acknowledged'] is first
        assert evidence['second_stop_acknowledged'] is True
        assert evidence['controller_stop_acknowledged'] is True
        assert evidence['controller_terminal_state_verified'] is False
        deck = evidence['details']['deck_reconciliation']
        assert deck['error'] is None
        assert deck['source_call_completed'] is True
        assert deck['source_return_ok'] is True
        assert deck['first_stop_acknowledged'] is first
        assert deck['second_stop_acknowledged'] is True
        assert deck['controller_terminal_state_verified'] is False
        assert deck['physical_effect_verified'] is False
        assert deck['controller_stop_acknowledged'] is True
        assert projected['physical_effect_verified'] is False
        raw = app.state.operator_receipt_store.by_command(row['command_id'], include_evidence=detail)
        assert raw['status'] == 'completed'
        assert raw['response']['http_status'] == 200
        assert raw['response']['body']['ok'] is True
        code = "import json,sys; from bioxp.operator_receipt_store import OperatorReceiptStore; s=OperatorReceiptStore(sys.argv[1]); print(json.dumps(s.by_command(sys.argv[2],include_evidence=sys.argv[3]=='True')))"
        fresh = json.loads(subprocess.check_output([sys.executable, '-c', code,
            str(app.state.operator_receipt_store.root), row['command_id'], str(detail)], text=True))
        assert fresh == raw
        captures['detail' if detail else 'compact'] = {'projected': projected, 'fresh': fresh}
    history = client.get('/operator/actions/history').json()['items']
    saved = next(item for item in history if item['command_id'] == row['command_id'])
    assert saved['status'] == 'completed'
    assert saved['error'] is None
    assert saved['physical_effect_verified'] is False

    export = os.environ.get('BMS_STOP_OUTPUT')
    if export:
        with open(export, 'a', encoding='utf-8') as output:
            output.write(json.dumps({'armed': armed, 'first_ack': first,
                                     'mutation': row, 'history': saved,
                                     'captures': captures}) + '\n')


def test_next_addressed_stop_delivers_while_prior_sqlite_writer_waits(tmp_path, monkeypatch, producer):
    provider, tester, hardware = producer
    provider._save_state(provider._load_state())
    entered_reconciliation = threading.Event()
    fourth_write = threading.Event()
    real_desync = provider._z_mark_desynced
    real_write = hardware.write
    def desync(*args, **kwargs):
        entered_reconciliation.set()
        return real_desync(*args, **kwargs)
    def write(*args, **kwargs):
        result = real_write(*args, **kwargs)
        if hardware.stop_writes == 4:
            fourth_write.set()
        return result
    monkeypatch.setattr(provider, '_z_mark_desynced', desync)
    monkeypatch.setattr(hardware, 'write', write)
    locker = sqlite3.connect(tmp_path / 'producer' / 'bioxp_runtime.db', timeout=2)
    locker.execute('BEGIN IMMEDIATE')
    results = {}
    def invoke(command):
        results[command] = provider.execute_z_stop_interrupt(inputs={'command_id': command},
            expected_generation=1, idempotency_key=command)
    first = threading.Thread(target=invoke, args=('contended-first',))
    second = threading.Thread(target=invoke, args=('contended-next',))
    first.start()
    try:
        assert entered_reconciliation.wait(2)
        second.start()
        assert fourth_write.wait(1), 'next addressed Stop is blocked behind prior persistence'
        assert first.is_alive(), 'prior SQLite writer must still be waiting'
        assert 'contended-first' not in results
    finally:
        locker.rollback()
        locker.close()
        first.join(10)
        if second.ident:
            second.join(10)
    assert not first.is_alive() and not second.is_alive()
    for command in ('contended-first', 'contended-next'):
        assert results[command]['result']['controller_command_acknowledged'] is True
        persisted = provider.state_store.read_serial206_receipt('z', command)
        assert persisted['controller_command_acknowledged'] is True
        assert persisted['controller_terminal_state_verified'] is False


@pytest.mark.parametrize('speed,status,verified', [(0,100,True), (123,100,False), (0,2,False)])
def test_separate_raw_terminal_readback_is_not_stop_ack(producer, speed, status, verified):
    provider, tester, hardware = producer
    stop = provider.primitives.z_stop()
    hardware.gap_value = speed
    hardware.gap_status = status
    observation = tester.motor_wait_stopped(4, motor=1, timeout_s=0.3, min_polls=1)
    assert stop['controller_command_acknowledged'] is True
    assert stop['controller_terminal_state_verified'] is False
    assert observation['controller_terminal_state_verified'] is verified
    assert hardware.stop_writes == 2


def test_missing_terminal_reply_cannot_verify_stop(producer, monkeypatch):
    provider, tester, hardware = producer
    stop = provider.primitives.z_stop()
    monkeypatch.setattr(tester, '_send_motor', lambda *a, **kw: None)
    observation = tester.motor_wait_stopped(4, motor=1, timeout_s=0.3, min_polls=1)
    assert stop['controller_command_acknowledged'] is True
    assert observation['controller_terminal_state_verified'] is False


def test_actual_http_next_stop_bypasses_prior_sqlite_reconciliation(tmp_path, monkeypatch, producer):
    from httpx import ASGITransport, AsyncClient
    provider, tester, hardware = producer
    provider._save_state(provider._load_state())
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    monkeypatch.setattr(api, '_tester', tester)
    monkeypatch.setattr(api, '_tester_transition_lock', asyncio.Lock())
    app, _ = make_app(tmp_path / 'outer', monkeypatch, z_stop_route=api.motion_oem_z_stop, generation=1)
    entered = threading.Event()
    fourth = threading.Event()
    original_desync, original_write = provider._z_mark_desynced, hardware.write
    def desync(*args, **kwargs):
        entered.set()
        return original_desync(*args, **kwargs)
    def write(*args, **kwargs):
        assert api._tester_transition_lock.locked(), 'physical call escaped ownership lease'
        result = original_write(*args, **kwargs)
        if hardware.stop_writes == 4:
            fourth.set()
        return result
    monkeypatch.setattr(provider, '_z_mark_desynced', desync)
    monkeypatch.setattr(hardware, 'write', write)
    locker = sqlite3.connect(tmp_path / 'producer' / 'bioxp_runtime.db', timeout=2)
    locker.execute('BEGIN IMMEDIATE')
    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url='http://testserver') as client:
            def request(key):
                return client.post('/operator/v2/actions/oem.z.stop', json={
                    'schema_version': 'bioxp.operator_interrupt_request.v1',
                    'idempotency_key': key, 'reason': 'offline contention',
                    'observed_ownership_generation': 1, 'observed_board_epoch_by_board': {}})
            first = asyncio.create_task(request('http-first'))
            second = None
            try:
                assert await asyncio.to_thread(entered.wait, 2)
                second = asyncio.create_task(request('http-next'))
                assert await asyncio.to_thread(fourth.wait, 1), 'second HTTP Stop blocked behind recording'
                assert not first.done()
                assert not second.done()
            finally:
                locker.rollback()
                locker.close()
                responses = await asyncio.wait_for(asyncio.gather(first, *([second] if second else [])), 10)
            for index, response in enumerate(responses):
                assert response.status_code == 200, response.text
                row = response.json()
                # The first delivery is superseded by the second interrupt;
                # only the last source command may report completion.
                assert row['status'] == ('failed' if index == 0 else 'completed'), row
                assert row['interrupt_evidence']['controller_stop_acknowledged'] is True
                for detail in (False, True):
                    saved = (await client.get(row['status_path'], params={'detail': detail})).json()
                    assert saved['status'] == row['status']
                    assert saved['interrupt_evidence']['controller_stop_acknowledged'] is True
                    assert saved['interrupt_evidence']['controller_terminal_state_verified'] is False
    asyncio.run(scenario())


@pytest.mark.parametrize('waiter_end', ['cancel', 'timeout'])
def test_api_ownership_lease_survives_waiter_end_until_delivery(tmp_path, monkeypatch, producer, waiter_end):
    provider, tester, hardware = producer
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    monkeypatch.setattr(api, '_tester', tester)
    monkeypatch.setattr(api, '_tester_transition_lock', asyncio.Lock())
    entered, release = threading.Event(), threading.Event()
    original_write = hardware.write
    def write(*args, **kwargs):
        assert api._tester_transition_lock.locked()
        if hardware.stop_writes == 0:
            entered.set()
            assert release.wait(5)
        return original_write(*args, **kwargs)
    monkeypatch.setattr(hardware, 'write', write)
    async def scenario():
        token = operator_controls._DISPATCH_CONTEXT.set({
            'operator_command_id': 'waiter-' + waiter_end, 'idempotency_key': waiter_end,
            'expected_ownership_generation': 1, 'action_id': 'oem.z.stop'})
        task = asyncio.create_task(api._run_safety_interrupt_blocking(
            'offline-' + waiter_end,
            lambda _: api._execute_provider_z_intent('stop', defer_reconciliation=True),
            timeout_s=0.1 if waiter_end == 'timeout' else 10, delivery_only_lease=True))
        try:
            assert await asyncio.to_thread(entered.wait, 2)
            if waiter_end == 'cancel':
                task.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await task
            else:
                with pytest.raises(HTTPException) as exc:
                    await task
                assert exc.value.status_code == 504
            assert api._tester_transition_lock.locked()
            assert hardware.stop_writes == 0
        finally:
            release.set()
            workers = [t for t in asyncio.all_tasks() if t.get_name() == 'bioxp-interrupt:offline-' + waiter_end]
            await asyncio.wait_for(asyncio.gather(*workers, return_exceptions=True), 5)
            operator_controls._DISPATCH_CONTEXT.reset(token)
        assert hardware.stop_writes == 2
        assert not api._tester_transition_lock.locked()
        receipt = provider.state_store.read_serial206_receipt('z', 'waiter-' + waiter_end)
        assert receipt['controller_command_acknowledged'] is True
    asyncio.run(scenario())


def test_storage_failure_does_not_erase_source_return_or_repeat_delivery(producer, monkeypatch):
    provider, tester, hardware = producer
    def fail(*args, **kwargs):
        raise sqlite3.OperationalError('offline injected storage failure')
    monkeypatch.setattr(provider, '_save_state', fail)
    for index in (1, 2):
        result = provider.execute_z_stop_interrupt(inputs={'command_id': f'storage-{index}'},
            expected_generation=1, idempotency_key=f'storage-{index}')
        assert result['persistence_state'] == 'recovery_required'
        assert result['recovery_hold'] is True
        assert result['source_return_ok'] is True
        assert result['controller_command_acknowledged'] is True
        assert result['controller_terminal_state_verified'] is False
        assert hardware.stop_writes == index * 2


def test_generation_change_after_delivery_is_retained_without_more_controller_calls(producer):
    provider, tester, hardware = producer
    reconcile = provider.execute_z_stop_interrupt(inputs={'command_id': 'generation-fence'},
        expected_generation=1, idempotency_key='generation-fence', defer_reconciliation=True)
    assert hardware.stop_writes == 2
    provider.generation_provider = lambda: 2
    result = reconcile()
    assert result['result']['ownership_generation_match'] is False
    assert result['authority_receipt']['observed_generation'] == 2
    assert result['ok'] is False
    assert result['z_lifecycle']['reference_state'] == 'desynced'
    assert hardware.stop_writes == 2


@pytest.mark.parametrize('failure', ['negative-observation', 'failed-readback', 'missing-source-return', 'missing-ack', 'raw-failed-ack', 'source-exception', 'no24v', 'storage'])
def test_actual_route_preserves_real_failure(tmp_path, monkeypatch, producer, failure):
    provider, tester, hardware = producer
    original = provider.primitives.z_stop
    if failure == 'raw-failed-ack':
        hardware.stop_statuses = iter([100, 2])
    elif failure == 'no24v':
        # Head.stopMotor itself rejects No24V: do not bypass the OEM guard.
        monkeypatch.setattr(tester, 'oem_no24v_state', lambda: True)
    elif failure == 'storage':
        def fail_save(*args, **kwargs):
            raise sqlite3.OperationalError('offline storage fault')
        monkeypatch.setattr(provider, '_save_state', fail_save)
    else:
        def stop(**kwargs):
            if failure == 'source-exception':
                raise RuntimeError('offline source exception')
            result = original(**kwargs)
            if failure in {'negative-observation', 'failed-readback'}:
                hardware.gap_value = 123 if failure == 'negative-observation' else 0
                hardware.gap_status = 100 if failure == 'negative-observation' else 2
                result['wait'] = tester.motor_wait_stopped(4, motor=1, timeout_s=0.3, min_polls=1)
                assert result['wait']['controller_terminal_state_verified'] is False
            elif failure == 'missing-source-return':
                result.pop('source_return_ok')
            else:
                result['controller_command_acknowledged'] = False
            return result
        monkeypatch.setattr(provider.primitives, 'z_stop', stop)
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    monkeypatch.setattr(api, '_tester', tester)
    monkeypatch.setattr(api, '_tester_transition_lock', asyncio.Lock())
    app, _ = make_app(tmp_path / 'outer', monkeypatch, z_stop_route=api.motion_oem_z_stop, generation=1, armed=False)
    client = TestClient(app)
    response = client.post('/operator/v2/actions/oem.z.stop', json={
        'schema_version': 'bioxp.operator_interrupt_request.v1',
        'idempotency_key': 'negative-' + failure, 'reason': 'offline negative qualification',
        'observed_ownership_generation': 1, 'observed_board_epoch_by_board': {}})
    assert response.status_code == 200
    row = response.json()
    assert row['status'] == 'failed', row
    raw = app.state.operator_receipt_store.by_command(row['command_id'], include_evidence=True)
    assert raw['response']['http_status'] == 409
    assert hardware.stop_writes == (0 if failure in {'source-exception', 'no24v'} else 2)
    for detail in (False, True):
        saved = client.get(row['status_path'], params={'detail': detail}).json()
        assert saved['status'] == 'failed'
        assert saved['physical_effect_verified'] is False
        assert saved['interrupt_evidence']['controller_terminal_state_verified'] is not True


def test_source_completed_stop_does_not_terminalize_active_motion(tmp_path, monkeypatch, producer):
    from bioxp.operator_command_plane import ACTION_REQUEST_SCHEMA
    provider, tester, hardware = producer
    app, _ = make_app(tmp_path / 'outer', monkeypatch, generation=1)
    store = app.state.operator_command_plane.store
    state = {'ownership_generation': 1, 'serial206_initialization_provider': {
        'x_authority': {'active_board_epoch': 11}, 'board4_authority': {'active_board_epoch': 10}}}
    admitted = store.admit_command({'schema_version': ACTION_REQUEST_SCHEMA,
        'idempotency_key': 'active-move', 'expected_ownership_generation': 1,
        'expected_board_epoch_by_board': {}, 'action_id': 'oem.z.move_gz',
        'inputs': {'gripper_position_steps': 100, 'z_position_steps': 100}}, state=state)
    assert store.claim_next() is not None
    receipt = store.begin_interrupt('oem.z.stop', state=state, request={'idempotency_key': 'interrupt'})
    assert admitted['command_id'] in receipt['active_command_ids']
    store.mark_interrupt_attempted(idempotency_key='interrupt')
    delivered = provider.execute_z_stop_interrupt(inputs={'command_id': 'source-stop'}, expected_generation=1, idempotency_key='source-stop')
    assert delivered['ok'] is True
    terminal = store.finalize_interrupt(idempotency_key='interrupt', receipt=receipt,
        attempted=True, acknowledged=True, response=delivered)
    assert terminal['error'] is None
    assert terminal['source_return_ok'] is True
    assert terminal['controller_terminal_state_verified'] is False
    active = store.connection.execute('SELECT status FROM operator_plane_commands WHERE command_id=?', (admitted['command_id'],)).fetchone()
    assert active['status'] == 'ambiguous'
    canonical = store.connection.execute('SELECT state FROM serial206_movement_commands WHERE command_id=?', (admitted['command_id'],)).fetchone()
    assert canonical['state'] == 'ambiguous'
    code = "import sqlite3,sys; c=sqlite3.connect(sys.argv[1]); print(c.execute('SELECT status FROM operator_plane_commands WHERE command_id=?',(sys.argv[2],)).fetchone()[0])"
    assert subprocess.check_output([sys.executable, '-c', code, str(store.path), admitted['command_id']], text=True).strip() == 'ambiguous'
