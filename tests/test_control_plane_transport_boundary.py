"""Source dispatch != Stop TX when the sole transport owner is busy."""
import asyncio
import threading
import time
import json
from httpx import AsyncClient, ASGITransport
from test_operator_controls import make_app
from bioxp import operator_controls


def test_pending_transport_and_second_interrupt_keep_loop_reachable(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    transport = threading.Lock()
    transport.acquire()  # fake existing source-prescribed synchronous exchange
    received = []
    transmitted = []
    acknowledged = []
    origin = time.monotonic()

    def transaction():
        with transport:
            transmitted.append(time.monotonic() - origin)
            acknowledged.append(time.monotonic() - origin)
        return 200, {'ok': True, 'source_call_completed': True,
                     'controller_command_acknowledged': True,
                     'physical_effect_verified': False}

    async def dispatch(*args, **kwargs):
        received.append(time.monotonic() - origin)
        return await asyncio.to_thread(transaction)
    monkeypatch.setattr(operator_controls, '_dispatch_asgi', dispatch)
    async def run():
        async with AsyncClient(transport=ASGITransport(app=app), base_url='http://offline') as client:
            assert (await client.get('/operator/v2/dashboard')).status_code == 200
            first = asyncio.create_task(client.post('/operator/actions/oem.abort_all', json={
                'expected_generation': 7, 'idempotency_key': 'transport-first', 'inputs': {}}))
            while not received:
                await asyncio.sleep(0)
            assert received[0] < 1
            second = asyncio.create_task(client.post('/operator/actions/oem.abort_all', json={
                'expected_generation': 7, 'idempotency_key': 'transport-second', 'inputs': {}}))
            begin = time.monotonic()
            for _ in range(30):
                assert (await client.get('/operator/v2/dashboard')).status_code == 200
                await asyncio.sleep(0)  # heartbeat while both HTTP tasks are pending
            assert time.monotonic() - begin < 1
            assert not transmitted and not acknowledged
            assert not first.done() and not second.done()
            assert len(received) == 1  # second interrupt retains existing serialization
            transport.release()
            results = await asyncio.wait_for(asyncio.gather(first, second), 2)
            assert all(r.status_code == 200 for r in results)
            assert len(transmitted) == len(acknowledged) == 2
            assert all(r.json()['physical_effect_verified'] is False for r in results)
            print(json.dumps({'fake_source_dispatch_s': received, 'fake_transport_tx_s': transmitted,
                              'fake_ack_s': acknowledged, 'http_complete_s': time.monotonic()-origin}))
    try:
        asyncio.run(run())
    finally:
        if transport.locked():
            transport.release()
        app.state.operator_poll_cache.close()
        app.state.operator_command_plane.stop()


def test_repeated_cancellation_cannot_queue_reconciliation_workers(tmp_path, monkeypatch):
    import pytest
    app, dispatched = make_app(tmp_path, monkeypatch)
    entered, release = threading.Event(), threading.Event()
    reconciliations = []
    async def reconcile(*args, **kwargs):
        reconciliations.append(1)
        entered.set()
        release.wait(3)
        return {'persistence_state': 'committed', 'recovery_hold': False}
    monkeypatch.setattr(app.state.operator_command_plane, 'compat_invoke', reconcile)
    async def run():
        async with AsyncClient(transport=ASGITransport(app=app), base_url='http://offline') as client:
            first = asyncio.create_task(client.post('/operator/actions/oem.abort_all', json={
                'expected_generation': 7, 'idempotency_key': 'cancel-first', 'inputs': {}}))
            assert await asyncio.to_thread(entered.wait, 1)
            first.cancel()
            await asyncio.sleep(0)
            first.cancel()
            await asyncio.sleep(0)
            second = asyncio.create_task(client.post('/operator/actions/oem.abort_all', json={
                'expected_generation': 7, 'idempotency_key': 'cancel-second', 'inputs': {}}))
            for _ in range(20):
                await asyncio.sleep(0)
            assert dispatched == [('abort_all', None)]
            assert reconciliations == [1]
            assert not first.done() and not second.done()
            release.set()
            with pytest.raises(asyncio.CancelledError):
                await first
            assert (await second).status_code == 200
            assert reconciliations == [1, 1]
    try:
        asyncio.run(run())
    finally:
        release.set()
        app.state.operator_poll_cache.close()
        app.state.operator_command_plane.stop()
