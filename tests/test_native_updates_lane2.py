"""Shared notification producer, real SQLite/ASGI, no hardware or network."""
import asyncio
from contextlib import nullcontext
import json
import os
from pathlib import Path
import socket
import threading

import httpx
import pytest
from fastapi import APIRouter, FastAPI

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandPlane, OperatorCommandStore


@pytest.fixture
def store(tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('network forbidden')
    monkeypatch.setattr(socket.socket, 'connect', forbidden)
    runtime = OEMRuntimeStore(tmp_path)
    value = OperatorCommandStore(tmp_path)
    yield value
    value.stop()
    runtime.close()


def insert(store, command_id=None, kind='lane2_evidence', state='completed'):
    with store._transaction() as conn:
        return store._insert_transition(conn, command_id=command_id, event_kind=kind, state=state)


def admit(store):
    return store.admit_command({'action_id': 'oem.x.prepare', 'inputs': {},
        'expected_ownership_generation': 7, 'idempotency_key': 'lane2-command'},
        state={'ownership_generation': 7}, assessment={'enabled': True})['command_id']


def cursor(body):
    return dict(after_sequence=body['next_after_sequence'], after_pose_sequence=body['pose_sequence'])


def test_commit_nested_rollback_signals(store, monkeypatch):
    signals = []
    def signal():
        assert not store.connection.in_transaction
        signals.append(store.transitions(after=0, limit=200)['high_watermark'])
    monkeypatch.setattr(store, '_notify_updates', signal)
    with store._transaction():
        insert(store)
        assert signals == []
    assert len(signals) == 1
    before = signals[-1]
    with pytest.raises(ValueError), store._transaction():
        insert(store)
        raise ValueError('rollback outer')
    assert signals == [before]
    with store._transaction():
        with pytest.raises(ValueError), store._transaction():
            insert(store)
            raise ValueError('rollback savepoint')
    assert signals == [before]
    with store._transaction():
        insert(store)
        with pytest.raises(ValueError), store._transaction():
            insert(store)
            raise ValueError('preserve earlier pending transition')
    assert len(signals) == 2
    assert signals[-1] == before + 1


def test_failed_commit_never_signals_or_publishes(store, monkeypatch):
    import sqlite3
    connection = store.connection
    before = store.transitions(after=0, limit=200)['high_watermark']
    signals = []
    class FailCommit:
        def __getattr__(self, key):
            return getattr(connection, key)
        def execute(self, sql, *args):
            if sql == 'COMMIT':
                raise sqlite3.OperationalError('injected commit failure')
            return connection.execute(sql, *args)
    with monkeypatch.context() as patched:
        patched.setattr(store, 'connection', FailCommit())
        patched.setattr(store, '_notify_updates', lambda: signals.append(True))
        with pytest.raises(sqlite3.OperationalError, match='injected commit failure'):
            insert(store)
    assert not signals and not connection.in_transaction
    assert store.transitions(after=0, limit=200)['high_watermark'] == before
    assert store._transition_pending is False


def test_async_commit_pose_wake_idle_and_cancel(store, monkeypatch):
    async def run():
        initial = await store.updates()
        reads = []
        snapshot = store._updates_snapshot
        ready = asyncio.Event()
        loop = asyncio.get_running_loop()
        def read(*args):
            result = snapshot(*args)
            reads.append(result)
            loop.call_soon_threadsafe(ready.set)
            return result
        monkeypatch.setattr(store, '_updates_snapshot', read)
        task = asyncio.create_task(store.updates(**cursor(initial), wait_s=25))
        await ready.wait()
        await asyncio.sleep(.02)
        assert len(reads) == 1 and not task.done()
        # No SQLite/writer lock or worker is occupied by the long wait.
        await asyncio.to_thread(insert, store)
        result = await asyncio.wait_for(task, 1)
        assert result['next_after_sequence'] > initial['next_after_sequence']
        assert not store._updates_waiters
        ready.clear()
        task = asyncio.create_task(store.updates(**cursor(result), wait_s=25))
        await ready.wait()
        store.publish_axis_observation(axis='x', position_steps=17, observed_at=10., ownership_generation=7)
        pose = await asyncio.wait_for(task, 1)
        assert pose['pose']['axes'][0]['position_steps'] == 17
        ready.clear()
        task = asyncio.create_task(store.updates(**cursor(pose), wait_s=25))
        await ready.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert not store._updates_waiters
        count = len(reads)
        result = await store.updates(**cursor(pose), wait_s=.02)
        assert len(reads) == count + 1
        assert result == pose
    asyncio.run(run())


@pytest.mark.parametrize('timing', ['before_read', 'after_read'])
def test_commit_in_registration_read_wait_race(store, monkeypatch, timing):
    async def run():
        initial = await store.updates()
        original = store._updates_snapshot
        first = True
        def snapshot(*args):
            nonlocal first
            if first:
                first = False
                if timing == 'before_read':
                    insert(store)
                result = original(*args)
                if timing == 'after_read':
                    insert(store)
                return result
            return original(*args)
        monkeypatch.setattr(store, '_updates_snapshot', snapshot)
        result = await asyncio.wait_for(store.updates(**cursor(initial), wait_s=25), 1)
        assert result['next_after_sequence'] > initial['next_after_sequence']
        assert not store._updates_waiters
    asyncio.run(run())


def test_pose_partial_invalid_old_epoch_and_clear(store):
    async def run():
        first = await store.updates()
        store.publish_axis_observation(axis='x', position_steps=10, observed_at=10, ownership_generation=7)
        store.publish_axis_observation(axis='y', position_steps=20, observed_at=11, ownership_generation=7)
        store.publish_axis_observation(axis='x', position_steps=12, observed_at=12, ownership_generation=7)
        result = await store.updates()
        assert result['pose']['axes'] == [dict(axis='x', position_steps=12, observed_at=12),
                                        dict(axis='y', position_steps=20, observed_at=11)]
        for overrides in ({'axis': 'u'}, {'position_steps': float('nan')}, {'position_steps': True},
                          {'observed_at': float('inf')}, {'ownership_generation': -1},
                          {'ownership_generation': 6}, {'observed_at': 9}):
            store.publish_axis_observation(**dict(dict(axis='x', position_steps=99,
                observed_at=12, ownership_generation=7), **overrides))
        assert await store.updates() == result
        store.set_observation_generation(ownership_generation=8)
        cleared = await store.updates()
        assert cleared['pose'] is None and cleared['ownership_generation'] == 8
        assert cleared['pose_sequence'] > result['pose_sequence'] > first['pose_sequence']
        store.publish_axis_observation(axis='z', position_steps=50, observed_at=13, ownership_generation=8)
        fresh = await store.updates()
        assert fresh['pose']['axes'] == [dict(axis='z', position_steps=50, observed_at=13)]
    asyncio.run(run())


def test_initial_catchup_ambiguous_reconciliation_reset_and_restart(store):
    async def run():
        cid = admit(store)
        initial = await store.updates()
        assert initial['changed_command_ids'] == []
        assert initial['active_command_ids'] == [cid]
        claim = store.claim_next()
        store.finish(cid, status='ambiguous', payload={'reason': 'offline ambiguity'}, claimed=claim)
        terminal = await store.updates(**cursor(initial))
        assert terminal['changed_command_ids'] == [cid]
        assert terminal['active_command_ids'] == []
        before = tuple(store.connection.execute('SELECT state,state_version FROM serial206_movement_commands WHERE command_id=?', (cid,)).fetchone())
        store.mark_deck_recovery_required(cid, reason='offline later recovery evidence')
        recovery = await store.updates(**cursor(terminal))
        assert recovery['changed_command_ids'] == [cid]
        insert(store, cid, kind='deck_reconciled', state='reconciled')
        reconciled = await store.updates(**cursor(recovery))
        assert reconciled['changed_command_ids'] == [cid]
        after = tuple(store.connection.execute('SELECT state,state_version FROM serial206_movement_commands WHERE command_id=?', (cid,)).fetchone())
        assert before == after and before[0] == 'ambiguous'
        with store._transaction():
            for i in range(205):
                insert(store, cid if i % 2 else None)
        page1 = await store.updates(**cursor(reconciled))
        assert page1['has_more'] and page1['changed_command_ids'] == [cid]
        assert page1['next_after_sequence'] == reconciled['next_after_sequence'] + 200
        page2 = await store.updates(**cursor(page1))
        assert not page2['has_more']
        assert page2['next_after_sequence'] == page1['next_after_sequence'] + 5
        reset = await store.updates(after_sequence=9999999, after_pose_sequence=9999999)
        assert reset['reset'] and reset['changed_command_ids'] == []
        assert reset['next_after_sequence'] == page2['next_after_sequence']
        old_instance = initial['source_instance_id']
        store.stop()
        replacement = OperatorCommandStore(store.root)
        try:
            restarted = await replacement.updates()
            assert restarted['source_instance_id'] != old_instance
            assert restarted['pose'] is None
            assert restarted['next_after_sequence'] >= page2['next_after_sequence']
        finally:
            replacement.stop()
    asyncio.run(run())


def test_location_only_preserves_unknown_tips_and_tip_authority(store):
    stamps = dict(ownership_generation=7, board_epoch_4=10, board_epoch_5=20)
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    def publish(operation, updates):
        return store.publish_deck_owner_state(source_operation=operation,
            source_command_id='lane2-' + str(store.connection.total_changes), updates=updates, **stamps)
    unknown = publish('pipette_owner', dict(tip_loaded=True, tip_dirty=None, tip_location=None))
    for location, well in [('LOC_OC', 4), ('LOC_MS', 12)]:
        result = publish('updateLocation', dict(current_location=location, current_well=well))
        assert result['current_location'] == location and result['current_well'] == well
        for field in ('tip_loaded', 'tip_dirty', 'tip_location'):
            assert result[field] == unknown[field]
    with pytest.raises(ValueError, match='loaded tip requires'):
        publish('GantryLoad', {'tip_loaded': True})
    with pytest.raises(ValueError, match='outside the source domain'):
        publish('pipette_owner', {'tip_location': 99})
    with pytest.raises(ValueError, match='not permitted'):
        publish('updateLocation', dict(current_location='LOC_OC', current_well=1, tip_loaded=False))
    loaded = publish('pipette_owner', dict(tip_loaded=True, tip_dirty=False, tip_location=2))
    assert loaded['tip_location'] == 2
    cleared = publish('clearTipLoaded', dict(tip_loaded=False))
    assert cleared['tip_loaded'] is False


def test_asgi_disconnect_cleans_waiter(store, monkeypatch):
    async def run():
        app = FastAPI()
        plane = object.__new__(OperatorCommandPlane)
        plane.app, plane.store = app, store
        plane.router = APIRouter(prefix='/operator')
        plane._install_routes()
        app.include_router(plane.router)
        initial = await store.updates()
        ready = asyncio.Event()
        loop = asyncio.get_running_loop()
        original = store._updates_snapshot
        def snapshot(*args):
            result = original(*args)
            loop.call_soon_threadsafe(ready.set)
            return result
        monkeypatch.setattr(store, '_updates_snapshot', snapshot)
        async def receive():
            await ready.wait()
            return {'type': 'http.disconnect'}
        async def send(message):
            raise AssertionError('disconnected request must not send a response')
        query = f"after_sequence={initial['next_after_sequence']}&after_pose_sequence={initial['pose_sequence']}&wait_s=25"
        scope = dict(type='http', asgi={'version': '3.0'}, http_version='1.1', method='GET',
                     scheme='http', path='/operator/updates', raw_path=b'/operator/updates',
                     query_string=query.encode(), root_path='', headers=[],
                     client=('offline', 0), server=('offline', 80))
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(app(scope, receive, send), 1)
        assert not store._updates_waiters
    asyncio.run(run())


def test_actual_asgi_response_exports(store):
    async def run():
        app = FastAPI()
        plane = object.__new__(OperatorCommandPlane)
        plane.app, plane.store = app, store
        plane.router = APIRouter(prefix='/operator')
        plane._install_routes()
        app.include_router(plane.router)
        exports = {}
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app), base_url='http://offline') as client:
            async def get(name, **params):
                response = await client.get('/operator/updates', params={'wait_s': 0, **params})
                assert response.status_code == 200, response.text
                exports[name] = response.json()
                return exports[name]
            initial = await get('initial')
            cid = admit(store)
            active = await get('active', **cursor(initial))
            claim = store.claim_next()
            store.finish(cid, status='ambiguous', payload={'reason': 'offline ambiguity'}, claimed=claim)
            terminal = await get('terminal_ambiguous', **cursor(active))
            insert(store, cid, kind='deck_reconciled', state='reconciled')
            await get('reconciliation', **cursor(terminal))
            store.publish_axis_observation(axis='x', position_steps=135, observed_at=1700000000., ownership_generation=7)
            await get('pose_x')
            store.publish_axis_observation(axis='y', position_steps=246, observed_at=1700000001., ownership_generation=7)
            await get('pose_xy')
            store.publish_axis_observation(axis='x', position_steps=137, observed_at=1700000002., ownership_generation=7)
            await get('pose_partial_x')
            store.set_observation_generation(ownership_generation=8)
            await get('ownership_clear')
            prior = await get('before_catchup')
            with store._transaction():
                for _ in range(205):
                    insert(store, cid, kind='deck_reconciled', state='reconciled')
            page = await get('catchup_page1', **cursor(prior))
            await get('catchup_page2', **cursor(page))
            await get('reset', after_sequence=999999, after_pose_sequence=999999)
            for params in ({'wait_s': 26}, {'after_sequence': -1}, {'after_pose_sequence': -1}):
                response = await client.get('/operator/updates', params=params)
                assert response.status_code == 422
        if os.environ.get('NATIVE_UPDATES_EXPORT_DIR'):
            root = Path(os.environ['NATIVE_UPDATES_EXPORT_DIR'])
            root.mkdir(parents=True, exist_ok=True)
            for name, body in exports.items():
                (root / (name + '.json')).write_text(json.dumps(body, indent=2) + '\n')
    asyncio.run(run())
