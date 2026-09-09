"""Held provider mutexes: intake timing, not physical Stop guarantees."""
import asyncio
import json
import threading
import time

import pytest
from httpx import ASGITransport, AsyncClient
import test_operator_controls as fixtures


def locked_app(tmp_path, monkeypatch):
    lock = threading.Lock()
    entered, release, holding = threading.Event(), threading.Event(), threading.Event()
    original_install = fixtures.install_operator_control_plane
    reads = []

    def install(app, **kwargs):
        original = kwargs['serial206_initialization_state_provider']
        def state():
            reads.append(threading.current_thread().name)
            if holding.is_set():
                entered.set()
            with lock:
                return original()
        kwargs['serial206_initialization_state_provider'] = state
        return original_install(app, **kwargs)

    monkeypatch.setattr(fixtures, 'install_operator_control_plane', install)
    app, dispatched = fixtures.make_app(tmp_path, monkeypatch)
    def hold():
        with lock:
            holding.set()
            release.wait(3)  # independent baseline watchdog, never loop-driven
    return app, dispatched, reads, entered, release, holding, hold


def close(app):
    app.state.operator_poll_cache.close()
    for name in ('operator_admission_state_reader', 'operator_invoke_state_reader'):
        reader = getattr(app.state, name, None)
        if reader:
            reader.close()
    app.state.operator_command_plane.stop()


@pytest.mark.parametrize('route', ['admission', 'invoke', 'v2'])
def test_held_state_admission_and_invoke_leave_stop_and_cached_status_reachable(tmp_path, monkeypatch, route):
    app, dispatched, reads, entered, release, holding, hold = locked_app(tmp_path, monkeypatch)
    async def run():
        async with AsyncClient(transport=ASGITransport(app=app), base_url='http://offline') as client:
            assert (await client.get('/operator/dashboard')).status_code == 200
            holder = threading.Thread(target=hold)
            holder.start()
            assert await asyncio.to_thread(holding.wait, 1)
            payload = {'expected_generation': 7, 'inputs': {'steps': 20}}
            if route == 'admission':
                path = '/operator/actions/oem.x.move_steps/admission'
            elif route == 'invoke':
                path = '/operator/actions/oem.x.move_steps'
                payload['idempotency_key'] = 'state-invoke-1'
            else:
                path = '/operator/v2/actions/oem.x.move_steps'
                payload = {'schema_version': 'bioxp.operator_action_request.v2',
                           'idempotency_key': 'state-v2-invoke-1',
                           'expected_ownership_generation': 7,
                           'expected_board_epoch_by_board': {}, 'inputs': {'steps': 20}}
            begin = time.monotonic()
            ordinary = asyncio.create_task(client.post(path, json=payload))
            assert await asyncio.to_thread(entered.wait, 4)
            heartbeat = time.monotonic() - begin
            stop = asyncio.create_task(client.post('/operator/actions/oem.abort_all', json={
                'expected_generation': 7, 'idempotency_key': 'held-state-stop', 'inputs': {}}))
            for _ in range(10000):
                if ('abort_all', None) in dispatched:
                    break
                await asyncio.sleep(0)
            intake = time.monotonic() - begin
            poll_start = time.monotonic()
            response = await client.get('/operator/dashboard')
            poll = time.monotonic() - poll_start
            assert response.status_code == 200
            assert ('abort_all', None) in dispatched
            assert max(heartbeat, intake, poll) < 1, (heartbeat, intake, poll)
            assert not any(row[0] == 'x_move_steps' for row in dispatched)
            print(json.dumps({'route': route, 'heartbeat_s': heartbeat, 'stop_dispatch_s': intake,
                              'cached_status_s': poll}))
            release.set()
            result = await asyncio.wait_for(ordinary, 2)
            if route == 'admission':
                assert result.status_code == 200
                assert result.json()['enabled'] is False
            else:
                assert result.status_code == 409, result.text
                assert result.json()['detail']['error'] == 'action_unavailable'
                assert any(d['key'] == 'x_board_lifecycle_fresh' and d['met'] is False
                           for d in result.json()['detail']['dependencies'])
            assert not any(row[0] == 'x_move_steps' for row in dispatched)
            assert (await asyncio.wait_for(stop, 2)).status_code == 200
            holder.join(1)
            db = app.state.operator_command_plane.store.connection
            assert db.execute('PRAGMA integrity_check').fetchone()[0] == 'ok'
            assert db.execute('PRAGMA foreign_key_check').fetchall() == []
    try:
        asyncio.run(run())
    finally:
        release.set()
        close(app)


def test_cancelled_state_read_retains_invoke_serialization_and_rechecks_generation(tmp_path, monkeypatch):
    app, dispatched, reads, entered, release, holding, hold = locked_app(tmp_path, monkeypatch)
    async def run():
        async with AsyncClient(transport=ASGITransport(app=app), base_url='http://offline') as client:
            holder = threading.Thread(target=hold)
            holder.start()
            assert await asyncio.to_thread(holding.wait, 1)
            async def invoke(key):
                return await client.post('/operator/actions/oem.x.move_steps', json={
                    'expected_generation': 7, 'idempotency_key': key, 'inputs': {'steps': 20}})
            first = asyncio.create_task(invoke('cancel-state-first'))
            assert await asyncio.to_thread(entered.wait, 4)
            for _ in range(5):
                first.cancel()
                await asyncio.sleep(0)
            assert not first.done()
            second = asyncio.create_task(invoke('cancel-state-second'))
            await asyncio.sleep(.05)
            assert len(reads) == 1, 'cancelled read released execution lock or queued provider work'
            app.state.operator_test_hardware.ownership_epoch = 8
            release.set()
            with pytest.raises(asyncio.CancelledError):
                await first
            result = await asyncio.wait_for(second, 2)
            assert result.status_code == 409
            assert not dispatched
            holder.join(1)
    try:
        asyncio.run(run())
    finally:
        release.set()
        close(app)


def test_admission_flood_cancellation_retains_one_metadata_read(tmp_path, monkeypatch):
    app, dispatched, reads, entered, release, holding, hold = locked_app(tmp_path, monkeypatch)
    async def run():
        async with AsyncClient(transport=ASGITransport(app=app), base_url='http://offline') as client:
            holder = threading.Thread(target=hold)
            holder.start()
            assert await asyncio.to_thread(holding.wait, 1)
            async def admission():
                return await client.post('/operator/actions/oem.x.move_steps/admission', json={
                    'expected_generation': 7, 'inputs': {'steps': 20}})
            first = asyncio.create_task(admission())
            assert await asyncio.to_thread(entered.wait, 4)
            first.cancel()
            with pytest.raises(asyncio.CancelledError):
                await first
            replies = await asyncio.gather(*(admission() for _ in range(40)))
            assert all(r.status_code == 503 and r.json()['detail'] == 'operator_state_warming' for r in replies)
            assert len(reads) == 1
            assert not dispatched
            release.set()
            holder.join(1)
    try:
        asyncio.run(run())
    finally:
        release.set()
        close(app)
