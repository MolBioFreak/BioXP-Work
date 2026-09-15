"""Named admission is command-owned fresh work, not a metadata waiter."""
import asyncio
import threading
import time

import httpx
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_automatic_refresh_owner import request, finish
from tests.test_deck_near_terminal import named_rig

URL = '/operator/v2/actions/oem.deck.move_to_location'


def ready(installed, monkeypatch, retained):
    from bioxp import api
    app, provider, primitive, references, root = installed
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    # Real adapter and Y driver with controller leaf I/O only substituted.
    _, _, runtime, _, _, _ = retained
    leaf, raw, _ = named_rig((provider, primitive, runtime, references,
        app.state.operator_command_plane.store, root), monkeypatch, (0, 0))
    collected = api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='offline-explicit-refresh')
    assert collected['deck_authority']['enabled'] is True
    assert catalog_action(app)['enabled'] is True
    return leaf, raw


def block_reader(app, monkeypatch):
    reader = app.state.operator_admission_state_reader
    original = reader._collect
    entered, release = threading.Event(), threading.Event()
    def collect():
        entered.set()
        assert release.wait(5), 'test must release provider collection'
        return original()
    monkeypatch.setattr(reader, '_collect', collect)
    return entered, release


@pytest.mark.parametrize('drift', [False, True])
def test_named_waits_for_fresh_state_then_executes_or_rejects(installed_retained, retained_rig, monkeypatch, drift):
    from bioxp import api
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    body = request(provider, 'slow-current-' + str(drift))
    body['inputs']['target'] = 'LOC_RC'
    if drift:
        body['expected_ownership_generation'] += 1
    entered, release = block_reader(app, monkeypatch)
    responses = []
    thread = threading.Thread(target=lambda: responses.append(TestClient(app).post(URL, json=body)))
    thread.start()
    try:
        assert entered.wait(3)
        thread.join(.7)
        assert thread.is_alive(), 'named command was incorrectly subject to metadata 0.5s expiry'
        assert app.state.operator_normal_action_active()
        assert api._collect_and_publish_hardware_snapshot(['axes'], reason='offline-auto', automatic=True)['reason'] == 'operator_action_pending'
        # Warm passive polling remains responsive while the command waits.
        started = time.monotonic()
        assert TestClient(app).get('/operator/v2/control-catalog').status_code == 200
        assert time.monotonic() - started < 1
        assert not leaf.moves
    finally:
        release.set()
        thread.join(8)
    assert not thread.is_alive()
    assert responses[0].status_code == (409 if drift else 200), responses[0].text
    assert not app.state.operator_normal_action_active()
    if drift:
        assert not leaf.moves and not app.state.operator_command_plane.store.queue()['pending_count']
        return
    command_id = responses[0].json()['command_id']
    app.state.operator_command_plane.start()
    row = finish(TestClient(app), command_id)
    assert row['status'] == 'completed', row
    assert row['deck_movement']['semantic_state_committed'] is True
    assert raw and leaf.moves
    # Same identity retrieves durable admission without a second execution.
    before = list(leaf.moves)
    replay = TestClient(app).post(URL, json=body)
    assert replay.status_code == 200 and replay.json()['command_id'] == command_id
    assert leaf.moves == before


def test_cancelled_named_collection_drains_without_admission(installed_retained, retained_rig, monkeypatch):
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    body = request(provider, 'cancel-before-admission')
    entered, release = block_reader(app, monkeypatch)
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as client:
            task = asyncio.create_task(client.post(URL, json=body))
            assert await asyncio.to_thread(entered.wait, 3)
            task.cancel()
            await asyncio.sleep(.05)
            task.cancel()
            await asyncio.sleep(.05)
            try:
                assert not task.done(), 'cancelled HTTP must not relinquish provider ownership'
                assert app.state.operator_normal_action_active()
            finally:
                release.set()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert not app.state.operator_normal_action_active()
            assert not app.state.operator_command_plane.store.queue()['pending_count']
            assert not leaf.moves
    asyncio.run(scenario())


def test_preview_still_bounded_and_separate(installed_retained, retained_rig, monkeypatch):
    app, provider, primitive, references, root = installed_retained
    ready(installed_retained, monkeypatch, retained_rig)
    reader = app.state.operator_preview_state_reader
    original = reader._collect
    entered, release = threading.Event(), threading.Event()
    def delayed():
        entered.set()
        assert release.wait(5)
        return original()
    monkeypatch.setattr(reader, '_collect', delayed)
    client = TestClient(app)
    try:
        started = time.monotonic()
        response = client.post('/operator/actions/oem.deck.move_to_location/admission', json={
            'expected_generation': int(provider.generation_provider()), 'inputs': {}})
        assert entered.is_set()
        assert response.status_code == 503 and response.json()['detail'] == 'operator_state_warming'
        assert time.monotonic() - started < 1.5
        assert not app.state.operator_normal_action_active()
        response = client.post(URL, json=request(provider, 'independent-preview'))
        assert response.status_code == 200, response.text
    finally:
        release.set()
        reader._pending.result(timeout=3)


def test_cancelled_sqlite_admission_retains_owner_and_single_identity(installed_retained, retained_rig, monkeypatch):
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    store = app.state.operator_command_plane.store
    original = store.admit_command
    entered, release = threading.Event(), threading.Event()
    committed = []
    def delayed(*args, **kwargs):
        if args[0]['idempotency_key'] == 'cancel-in-sqlite':
            entered.set()
            assert release.wait(5)
        result = original(*args, **kwargs)
        committed.append(result)
        return result
    monkeypatch.setattr(store, 'admit_command', delayed)
    body = request(provider, 'cancel-in-sqlite')
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as client:
            task = asyncio.create_task(client.post(URL, json=body))
            assert await asyncio.to_thread(entered.wait, 3)
            task.cancel()
            await asyncio.sleep(.05)
            task.cancel()
            await asyncio.sleep(.05)
            try:
                assert not task.done() and app.state.operator_normal_action_active()
                busy = await client.post(URL, json={**body, 'idempotency_key': 'competing'})
                assert busy.status_code == 200, busy.text  # independent queued intent
                # Actual metadata route stays reachable while SQLite work waits.
                assert (await client.get('/operator/v2/control-catalog')).status_code == 200
            finally:
                release.set()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert not app.state.operator_normal_action_active()
            assert len(committed) == 2 and not leaf.moves
            replay = await client.post(URL, json=body)
            assert replay.status_code == 200
            assert replay.json()['command_id'] == committed[1]['command_id']
            assert store.queue()['pending_count'] == 2
    asyncio.run(scenario())
    app.state.operator_command_plane.start()
    for item in committed:
        row = finish(TestClient(app), item['command_id'])
        assert row['status'] == 'completed', row


def test_live_reference_loss_during_read_is_not_cached_admission(installed_retained, retained_rig, monkeypatch):
    from bioxp.services.reference_service import MarkAxisDesyncedCommand
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    body = request(provider, 'fresh-reference-loss')
    entered, release = block_reader(app, monkeypatch)
    responses = []
    thread = threading.Thread(target=lambda: responses.append(TestClient(app).post(URL, json=body)))
    thread.start()
    try:
        assert entered.wait(3)
        references.mark_desynced(MarkAxisDesyncedCommand('x', reason='offline concurrent reference loss'))
    finally:
        release.set()
        thread.join(8)
    assert not thread.is_alive()
    assert responses[0].status_code == 200, responses[0].text
    app.state.operator_command_plane.start()
    row = finish(TestClient(app), responses[0].json()['command_id'])
    assert row['status'] == 'failed', row
    assert not leaf.moves and not app.state.operator_command_plane.store.queue()['pending_count']


def test_populated_finite_poll_views_and_immediate_next_named_move(installed_retained, retained_rig, monkeypatch):
    from bioxp import api
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    client = TestClient(app)
    app.state.operator_command_plane.start()
    ids = []
    for target in ('LOC_RC', 'TECANRACK2', 'LOC_MS'):
        body = request(provider, 'populated-next-' + target)
        body['inputs']['target'] = target
        response = client.post(URL, json=body)
        assert response.status_code == 200, response.text
        ids.append(response.json()['command_id'])
        receipt = finish(client, ids[-1])
        assert receipt['status'] == 'completed', receipt
        assert app.state.operator_command_plane.store.wait_for_command_workers([ids[-1]], timeout=2)
        assert api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='offline-next-refresh')['deck_authority']['enabled']
        # Demand every finite registered dashboard/catalog view with populated
        # durable receipts. Service warm refreshes as real subsequent polls do.
        paths = ('/operator/dashboard', '/operator/control-catalog',
                 '/operator/v2/dashboard', '/operator/v2/control-catalog')
        for _ in range(3):
            for path in paths:
                result = client.get(path)
                assert result.status_code == 200, (path, result.text)
                pending = app.state.operator_poll_cache._pending
                if pending is not None:
                    pending.result(timeout=3)
        assert catalog_action(app)['enabled']
        dashboard = client.get('/operator/v2/dashboard').json()
        assert dashboard['deck']['current_location'] == target
    assert len(set(ids)) == 3 and len(raw) >= 3
