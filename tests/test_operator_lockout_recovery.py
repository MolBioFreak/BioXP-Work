"""Ordinary manual ownership: no hidden waiters, no failure-induced latch.

Actual catalog, ASGI routes, tester lease/precheck and SQLite receipt store;
only controller leaf operations and hardware observations are offline doubles.
"""
import asyncio
import threading
from types import SimpleNamespace

import httpx
import pytest
from fastapi import FastAPI

from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained


@pytest.fixture
def manual_rig(installed_retained, monkeypatch):
    from bioxp import api, operator_controls as controls
    from bioxp.hardware_status import HardwareStateOwner
    old_app, provider, primitive, references, root = installed_retained
    old_app.state.operator_command_plane.stop()
    owner = HardwareStateOwner()
    observations = {'axes': {}, 'power': {'safety_valid': True},
        'latch': {'door_sensor': 1, 'latch_sensor': 1},
        'interlock': {'motion_arm': {'armed': True}}}
    def collect():
        return owner.collect(list(observations), {
            name: (lambda context, value=value: value) for name, value in observations.items()})
    collect()
    monkeypatch.setattr(controls.hardware_state, 'project', owner.project)
    monkeypatch.setattr(api, '_maintenance_state', {'motion_blocked': False, 'recovery_required': False})
    monkeypatch.setattr(api, '_tester_lock', asyncio.Lock())
    entered, release = threading.Event(), threading.Event()
    calls = []
    def door(**kwargs):
        calls.append(kwargs)
        entered.set()
        assert release.wait(5), 'offline controller leaf must be released'
        return {'ok': False, 'failure': 'offline native manual failure',
            'controller_command_acknowledged': True, 'physical_effect_verified': False}
    monkeypatch.setattr(api, '_get_tester', lambda: SimpleNamespace(motor_oem_open_thermal_door=door))
    app = FastAPI()
    app.add_api_route('/motion/thermal_door/open', api.motion_thermal_door_open, methods=['POST'])
    app.add_api_route('/motion/thermal_door/home', api.motion_thermal_door_home, methods=['POST'])
    app.add_api_route('/motion/gripper/home', api.motion_gripper_home, methods=['POST'])
    app.add_api_route('/motion/oem/move_xy', api.motion_oem_move_xy, methods=['POST'])
    actions, dispatch = controls._build_catalog(app)
    controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: api._maintenance_state,
        reference_state_provider=lambda: {'rows': {'door': {'state': 'referenced'}}},
        lifecycle_state_provider=lambda: {'operation_state': 'stopped'})
    monkeypatch.setattr(api, 'app', app)
    by_path = {row['informational_path']: row['action_id'] for row in actions
        if row['action_id'] in dispatch and row['action_id'] not in controls._PRIVATE_METHOD_ACTION_IDS}
    yield app, by_path, int(provider.generation_provider()), entered, release, calls, owner, collect
    release.set()
    app.state.operator_command_plane.stop()
    app.state.operator_poll_cache.close()
    app.state.operator_admission_state_reader.close()
    app.state.operator_preview_state_reader.close()


def body(generation, key):
    return {'expected_generation': generation, 'idempotency_key': key, 'inputs': {}}


@pytest.mark.parametrize('other_path', ['/motion/gripper/home', '/motion/thermal_door/home', '/motion/oem/move_xy'])
def test_legacy_manual_competitors_reject_without_waiting_and_recover(manual_rig, other_path):
    app, ids, generation, entered, release, calls, owner, collect = manual_rig
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as client:
            url = '/operator/actions/' + ids['/motion/thermal_door/open']
            first = asyncio.create_task(client.post(url, json=body(generation, 'native-first')))
            assert await asyncio.to_thread(entered.wait, 3)
            competitor = asyncio.create_task(client.post('/operator/actions/' + ids[other_path],
                json=body(generation, 'must-not-queue')))
            try:
                done, _ = await asyncio.wait([competitor], timeout=.5)
                assert done, 'legacy invocation queued invisibly behind active manual owner'
                rejected = competitor.result()
                assert rejected.status_code == 409 and rejected.json()['detail']['error'] == 'operator_action_busy'
                assert app.state.operator_receipt_store.by_idempotency('must-not-queue') is None
                assert app.state.operator_normal_action_active()
            finally:
                release.set()
                await first
                if not competitor.done():
                    competitor.cancel()
                await asyncio.gather(competitor, return_exceptions=True)
            receipt = first.result().json()
            assert receipt['status'] == 'failed', receipt
            assert not app.state.operator_normal_action_active()
            persisted = app.state.operator_receipt_store.by_command(receipt['command_id'])
            assert persisted['status'] == 'failed'
            # A manual native failure is a terminal report, not an added Home,
            # reset, emergency or one-minute host latch. Next deliberate action
            # re-enters the unchanged precheck and actual tester worker.
            next_response = await client.post(url, json=body(generation, 'native-next'))
            assert next_response.status_code == 200 and next_response.json()['status'] == 'failed'
            assert len(calls) == 2 and not app.state.operator_normal_action_active()
    asyncio.run(scenario())


@pytest.mark.parametrize('version', ['legacy', 'v2'])
def test_active_named_worker_does_not_create_manual_admission_waiter(installed_retained, retained_rig, monkeypatch, version):
    from fastapi.testclient import TestClient
    from tests.test_deck_complete_admission import ready
    from tests.test_deck_automatic_refresh_owner import request, finish
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    entered, release = threading.Event(), threading.Event()
    native = leaf.motor_oem_move_absolute
    def delayed(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        return native(*args, **kwargs)
    monkeypatch.setattr(leaf, 'motor_oem_move_absolute', delayed)
    client = TestClient(app)
    req = request(provider, 'active-worker-' + version)
    req['inputs']['target'] = 'LOC_RC'
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=req)
    assert response.status_code == 200, response.text
    command_id = response.json()['command_id']
    app.state.operator_command_plane.start()
    assert entered.wait(3), 'actual named dispatcher/native leaf was not reached'
    assert command_id in app.state.operator_command_plane.store.live_command_worker_ids()
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as current:
            if version == 'v2':
                path = '/operator/v2/actions/oem.deck.collect_authority'
                payload = {**req, 'idempotency_key': 'no-worker-waiter', 'inputs': {}}
            else:
                path = '/operator/actions/oem.deck.collect_authority'
                payload = body(int(provider.generation_provider()), 'no-worker-waiter')
            task = asyncio.create_task(current.post(path, json=payload))
            try:
                done, _ = await asyncio.wait([task], timeout=.5)
                assert done, 'manual admission blocked behind live named provider lock'
                result = task.result()
                assert result.status_code == 409 and result.json()['detail']['error'] == 'operator_action_busy'
                assert app.state.operator_receipt_store.by_idempotency('no-worker-waiter') is None
            finally:
                release.set()
                if not task.done():
                    task.cancel()
                await asyncio.gather(task, return_exceptions=True)
    try:
        asyncio.run(scenario())
    finally:
        release.set()
    result = finish(client, command_id)
    assert result['status'] == 'completed', result
    assert app.state.operator_command_plane.store.wait_for_command_workers([command_id], timeout=3)
    assert not app.state.operator_normal_action_active()
    # Actual retained V2 admission enters its own lane and then reconciles the
    # committed result; the busy guard must not reject its self-owned task.
    async def next_query():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as current:
            again = await current.post('/operator/v2/actions/oem.deck.collect_authority', json={
                **req, 'idempotency_key': 'after-native-exit', 'inputs': {}})
            assert again.status_code == 200, again.text
            cid = again.json()['command_id']
            for _ in range(300):
                row = (await current.get('/operator/v2/actions/receipts/' + cid)).json()
                if not app.state.operator_normal_action_active():
                    break
                await asyncio.sleep(.01)
            assert not app.state.operator_normal_action_active()
            assert row['status'] == 'completed', row
    asyncio.run(next_query())


def test_simultaneous_legacy_starts_have_one_owner_and_readonly_replay(manual_rig):
    app, ids, generation, entered, release, calls, owner, collect = manual_rig
    # The HTTP entrypoint has no private dispatcher admission parameter.
    operation = app.openapi()['paths']['/operator/actions/{action_id}']['post']
    assert all(p['name'] != '_admitted' for p in operation.get('parameters', []))
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as client:
            url = '/operator/actions/' + ids['/motion/thermal_door/open']
            start = asyncio.Event()
            async def submit(index):
                await start.wait()
                return await client.post(url, json=body(generation, 'manual-burst-' + str(index)))
            tasks = [asyncio.create_task(submit(index)) for index in range(8)]
            start.set()
            try:
                assert await asyncio.to_thread(entered.wait, 3)
                done, pending = await asyncio.wait(tasks, timeout=.5)
                assert len(done) == 7 and len(pending) == 1, 'check-to-acquire admitted hidden waiters'
                assert all(t.result().status_code == 409 and t.result().json()['detail']['error'] == 'operator_action_busy' for t in done)
                assert len(calls) == 1
                active_index = tasks.index(next(iter(pending)))
                replay = await client.post(url + '?_admitted=forged', json=body(generation, 'manual-burst-' + str(active_index)))
                assert replay.status_code == 200, replay.text
                replay_id = replay.json()['command_id']
                assert replay.json()['status'] == 'queued'  # committed admission, not source completion
                assert len(calls) == 1 and app.state.operator_normal_action_active()
            finally:
                release.set()
                results = await asyncio.gather(*tasks, return_exceptions=True)
            assert not app.state.operator_normal_action_active()
            terminal = await client.post(url, json=body(generation, 'manual-burst-' + str(active_index)))
            assert terminal.status_code == 200 and terminal.json()['status'] == 'failed'
            assert terminal.json()['command_id'] == replay_id and len(calls) == 1
    asyncio.run(scenario())


def test_stale_after_tester_lease_wait_rejects_before_native_leaf(manual_rig):
    from bioxp import api
    app, ids, generation, entered, release, calls, owner, collect = manual_rig
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as client:
            await api._tester_lock.acquire()
            url = '/operator/actions/' + ids['/motion/thermal_door/open']
            task = asyncio.create_task(client.post(url, json=body(generation, 'stale-at-dispatch')))
            try:
                for _ in range(200):
                    if any(t.get_name().startswith('bioxp-tester:') for t in asyncio.all_tasks()):
                        break
                    await asyncio.sleep(.01)
                else:
                    pytest.fail('real tester worker was not scheduled')
                owner.invalidate_domains('axes', reason='offline source changed before dispatch')
            finally:
                api._tester_lock.release()
            response = await task
            assert response.status_code == 200
            row = response.json()
            assert row['status'] == 'failed'
            assert row['response']['http_status'] == 409
            assert not entered.is_set() and not calls
            assert not app.state.operator_normal_action_active()
            collect()
            release.set()
            response = await client.post(url, json=body(generation, 'fresh-deliberate-next'))
            assert response.json()['status'] == 'failed' and len(calls) == 1
    asyncio.run(scenario())
