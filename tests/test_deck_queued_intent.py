"""General named intent FIFO, using installed API/native executor/private SQLite."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import threading

import httpx
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained, catalog_payload, catalog_action
from tests.test_deck_complete_admission import ready
from tests.test_deck_automatic_refresh_owner import request, finish

URL = '/operator/v2/actions/oem.deck.move_to_location'


def submit(client, provider, key, target='LOC_RC', **changes):
    body = request(provider, key)
    body['inputs']['target'] = target
    body.update(changes)
    response = client.post(URL, json=body)
    assert response.status_code == 200, response.text
    return body, response.json()['command_id']


@pytest.mark.parametrize('additional', [2, 7])
def test_overlapping_continuing_native_fifo(installed_retained, retained_rig, monkeypatch, additional):
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    plane = app.state.operator_command_plane
    entered, release = threading.Event(), threading.Event()
    original = app.state.oem_deck_command_executor
    entries = []
    samples = []
    native_sample = provider.deck_authority_snapshot
    def sample(**kwargs):
        samples.append(threading.current_thread().name)
        return native_sample(**kwargs)
    monkeypatch.setattr(provider, 'deck_authority_snapshot', sample)
    def executor(**kwargs):
        entries.append(kwargs['command_id'])
        if len(entries) == 1:
            entered.set()
            assert release.wait(12)
        return original(**kwargs)
    monkeypatch.setattr(app.state, 'oem_deck_command_executor', executor)
    client = TestClient(app)
    body, first = submit(client, provider, 'stream-first')
    assert samples == [], 'HTTP admission must not actively sample deck readiness'
    plane.start()
    ids = [first]
    bodies = [body]
    try:
        assert entered.wait(5)
        assert samples == [], 'worker must not repeat executor readiness acquisition'
        async def overlap():
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as async_client:
                requests = []
                for index in range(additional):
                    intent = request(provider, 'stream-' + str(index))
                    intent['inputs']['target'] = ('LOC_OC', 'LOC_MS', 'LOC_RC')[index % 3]
                    bodies.append(intent)
                    requests.append(async_client.post(URL, json=intent))
                return await asyncio.gather(*requests)
        responses = asyncio.run(overlap())
        for response in responses:
            assert response.status_code == 200, response.text
            ids.append(response.json()['command_id'])
        # More deliberate intent while the same first executor is still held.
        body, continued = submit(client, provider, 'stream-continuing', 'TECANRACK2')
        bodies.append(body)
        ids.append(continued)
        queue = plane.store.queue()
        ordered_ids = [row['command_id'] for row in queue['items']]
        assert set(ordered_ids) == set(ids) and len(set(ids)) == additional + 2
        assert ordered_ids[0] == first
        assert [row['sequence'] for row in queue['items']] == sorted(row['sequence'] for row in queue['items'])
        assert entries == [first] and not leaf.moves
        assert len(plane.store.live_command_worker_ids()) == 1
        payload = catalog_payload(app)
        assert next(row for row in payload['actions'] if row['action_id'] == 'oem.deck.move_to_location')['enabled']
        assert payload['dashboard']['command_queue']['items'] == queue['items']
        assert all(set(row) == {'command_id', 'sequence', 'status', 'method_id', 'resource_keys', 'accepted_at'} for row in queue['items'])
        assert all(row['resource_keys'] for row in queue['items'])
        if os.environ.get('C_QUEUE_EXPORT'):
            Path(os.environ['C_QUEUE_EXPORT'] + f'-{additional}.json').write_text(json.dumps(payload))
        for intent, cid in zip(bodies, ids):
            assert plane.store.get_command(cid)['requested_inputs'] == intent['inputs']
    finally:
        release.set()
    receipts = {cid: finish(client, cid) for cid in ids}
    assert all(row['status'] == 'completed' for row in receipts.values()), receipts
    assert entries == ordered_ids
    assert plane.store.wait_for_command_workers(ids, timeout=3)
    assert plane.store.queue()['items'] == []
    assert all(name.startswith('bioxp-operator-command-') for name in samples)
    assert len(samples) == 2 * len(ids), 'only leased planning and final pre-TX samples per ordinary move'
    # Read the same canonical results through a fresh SQLite process.
    plane.stop()
    code = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
            'print(json.dumps({cid:fresh_process_receipts(sys.argv[1],cid)["detail"] for cid in sys.argv[2:]}))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', code, str(root), *ids], text=True, timeout=20))
    assert reopened == receipts


@pytest.mark.parametrize('fault', ['maintenance', 'recovery', 'emergency'])
def test_worker_retains_host_fault_gates_without_sampling(installed_retained, retained_rig, monkeypatch, fault):
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    client = TestClient(app)
    body, cid = submit(client, provider, 'host-fault-' + fault)
    plane = app.state.operator_command_plane
    source = plane.machine_state_provider
    def state():
        value = source()
        if fault == 'maintenance':
            value['maintenance'].update(motion_blocked=True, block_reason='offline-maintenance-fault')
        elif fault == 'recovery':
            value['maintenance']['recovery_required'] = True
        else:
            value['lifecycle']['operation_state'] = 'emergency'
        return value
    monkeypatch.setattr(plane, 'machine_state_provider', state)
    monkeypatch.setattr(provider, 'deck_authority_snapshot', lambda **kwargs: pytest.fail('faulted worker sampled controller'))
    plane.start()
    row = finish(client, cid)
    assert row['status'] == 'failed', row
    assert not leaf.moves and not raw


@pytest.mark.parametrize('drift', ['latch', 'board', 'owner', 'reference'])
def test_execution_rechecks_actual_authority(installed_retained, retained_rig, monkeypatch, drift):
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    client = TestClient(app)
    body, cid = submit(client, provider, 'execution-' + drift)
    if drift == 'latch':
        monkeypatch.setattr(primitive, 'query_latch', lambda: {'ok': True, 'value': 0})
    elif drift == 'board':
        state = provider._load_state()
        state['x_lifecycle']['board_lifecycle_generation'] += 1
        provider._save_state(state)
    elif drift == 'owner':
        generation = provider.generation_provider()
        monkeypatch.setattr(provider, 'generation_provider', lambda: generation + 1)
    else:
        from bioxp.services.reference_service import MarkAxisDesyncedCommand
        references.mark_desynced(MarkAxisDesyncedCommand('x', reason='offline drift'))
    app.state.operator_command_plane.start()
    row = finish(client, cid)
    assert row['status'] == 'failed', row
    assert not leaf.moves and not raw
    # Even after owner/readiness changes the original key resolves without sampling.
    monkeypatch.setattr(app.state.operator_admission_state_reader, '_collect', lambda: pytest.fail('replay sampled state'))
    replay = client.post(URL, json=body)
    assert replay.status_code == 200 and replay.json()['command_id'] == cid, replay.text
    assert client.get('/operator/idempotency/command/' + body['idempotency_key']).json()['command_id'] == cid
    for changed in ({'inputs': {'target': 'LOC_OC', 'camera_offset': False}},
                    {'expected_ownership_generation': body['expected_ownership_generation'] + 1},
                    {'expected_board_epoch_by_board': {'4': 999, '5': 999}}):
        conflict = client.post(URL, json={**body, **changed})
        assert conflict.status_code == 409 and conflict.json()['detail']['error'] == 'idempotency_conflict'


@pytest.mark.parametrize('after_commit', [False, True])
def test_lost_response_get_is_not_absence_or_replay(installed_retained, retained_rig, monkeypatch, after_commit):
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    store = app.state.operator_command_plane.store
    original = store.admit_command
    entered, release = threading.Event(), threading.Event()
    committed = []
    def delayed(*args, **kwargs):
        if after_commit:
            committed.append(original(*args, **kwargs))
        entered.set()
        assert release.wait(8)
        if not after_commit:
            committed.append(original(*args, **kwargs))
        return committed[0]
    monkeypatch.setattr(store, 'admit_command', delayed)
    body = request(provider, 'lost-' + str(after_commit))
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as client:
            task = asyncio.create_task(client.post(URL, json=body))
            assert await asyncio.to_thread(entered.wait, 4)
            task.cancel()
            try:
                lookup = await client.get('/operator/idempotency/command/' + body['idempotency_key'])
                assert lookup.status_code == (200 if after_commit else 404)
                assert app.state.operator_normal_action_active()
            finally:
                release.set()
            with pytest.raises(asyncio.CancelledError):
                await task
            lookup = await client.get('/operator/idempotency/command/' + body['idempotency_key'])
            assert lookup.status_code == 200
            assert lookup.json()['command_id'] == committed[0]['command_id']
    asyncio.run(scenario())
    assert len(committed) == 1 and not leaf.moves
    app.state.operator_command_plane.start()
    row = finish(TestClient(app), committed[0]['command_id'])
    assert row['status'] == 'completed', row


def test_native_leaf_wait_accepts_intents_and_keeps_manual_busy(installed_retained, retained_rig, monkeypatch):
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    entered, release = threading.Event(), threading.Event()
    original = leaf.motor_wait_target_reached
    def wait(*args, **kwargs):
        entered.set()
        assert release.wait(8)
        return original(*args, **kwargs)
    monkeypatch.setattr(leaf, 'motor_wait_target_reached', wait)
    client = TestClient(app)
    _, first = submit(client, provider, 'leaf-held-first')
    plane = app.state.operator_command_plane
    plane.start()
    body = request(provider, 'same-key-overlap')
    try:
        assert entered.wait(4)
        async def overlap():
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline') as ac:
                return await asyncio.wait_for(asyncio.gather(*[ac.post(URL, json=body) for _ in range(3)]), timeout=3)
        responses = asyncio.run(overlap())
        assert all(r.status_code == 200 for r in responses), [r.text for r in responses]
        ids = {r.json()['command_id'] for r in responses}
        assert len(ids) == 1
        assert plane.store.queue()['pending_count'] == 2
        # The installed fixture exposes this real direct lifecycle action.
        # It uses the same retain_direct_action busy gate as raw primitives.
        manual = client.post('/operator/v2/actions/meta.activate_motion', json={**body,
            'idempotency_key': 'manual-during-deck', 'inputs': {}})
        assert manual.status_code == 409 and manual.json()['detail']['error'] == 'operator_action_busy', manual.text
    finally:
        release.set()
    for cid in [first, *ids]:
        assert finish(client, cid)['status'] == 'completed'


def test_collector_yields_after_waiting_for_lease(installed_retained, monkeypatch):
    app, provider, primitive, references, root = installed_retained
    entered, yield_now = threading.Event(), threading.Event()
    results = []
    original = provider.movement_lease
    from contextlib import contextmanager
    @contextmanager
    def lease():
        entered.set()
        with original():
            yield
    monkeypatch.setattr(provider, 'movement_lease', lease)
    monkeypatch.setattr(provider, 'deck_authority_snapshot', lambda **kwargs: pytest.fail('yielded collection sampled'))
    with original():
        thread = threading.Thread(target=lambda: results.append(app.state.oem_deck_authority_collector(yield_requested=yield_now.is_set)))
        thread.start()
        assert entered.wait(3)
        yield_now.set()
    thread.join(4)
    assert not thread.is_alive()
    assert results == [{'enabled': False, 'disabled_reason': 'operator_action_pending'}]


def test_invalid_intents_methods_capacity_and_same_key(installed_retained, monkeypatch):
    from bioxp import operator_command_plane as module
    from fastapi import HTTPException
    app, provider, primitive, references, root = installed_retained
    client = TestClient(app)
    store = app.state.operator_command_plane.store
    monkeypatch.setattr(provider, 'deck_authority_snapshot', lambda **kwargs: pytest.fail('admission sampled'))
    body = request(provider, 'valid-intent')
    for inputs in ({'target': 'NOT_A_LOCATION', 'camera_offset': False},
                   {'target': 'LOC_RC', 'camera_offset': 1},
                   {'target': 'LOC_PARK', 'camera_offset': True}):
        assert client.post(URL, json={**body, 'inputs': inputs}).status_code == 422
    body, cid = submit(client, provider, 'valid-intent', inputs={'target': 'LOC_RC', 'camera_offset': True})
    assert store.get_command(cid)['requested_inputs'] == body['inputs']
    replay = client.post(URL, json=body)
    assert replay.status_code == 200 and replay.json()['command_id'] == cid
    for epochs in ({}, {'4': -1, '5': 1}, {**body['expected_board_epoch_by_board'], '6': -1}):
        malformed = client.post(URL, json={**body, 'expected_board_epoch_by_board': epochs})
        assert malformed.status_code in {409, 422}
    generic = client.post('/operator/methods', json={
        'schema_version': 'bioxp.operator_method_request.v1', 'name': 'not-a-deck-batch',
        'idempotency_key': 'invalid-method', 'failure_policy': 'fail_fast',
        'expected_ownership_generation': body['expected_ownership_generation'],
        'steps': [{'action_id': 'oem.deck.move_to_location', 'inputs': body['inputs']}]})
    assert generic.status_code == 422 and generic.json()['detail']['error'] == 'method_action_not_allowed'
    with pytest.raises(HTTPException) as exc:
        store.admit_method({'expected_ownership_generation': body['expected_ownership_generation'],
            'steps': [{'action_id': 'oem.deck.move_to_location', 'inputs': body['inputs']}]}, state={})
    assert exc.value.detail['error'] == 'method_action_not_allowed'
    monkeypatch.setattr(module, 'COMMAND_CAPACITY', store.queue()['pending_count'])
    full = client.post(URL, json={**body, 'idempotency_key': 'capacity-full'})
    assert full.status_code == 429 and full.json()['detail']['error'] == 'capacity_exceeded'
    assert store.queue()['pending_count'] == 1


def test_noop_then_valid_and_queued_cancel(installed_retained, retained_rig, monkeypatch):
    from bioxp import api
    from tests.test_deck_near_terminal import named_rig
    app, provider, primitive, references, root = installed_retained
    ready(installed_retained, monkeypatch, retained_rig)
    leaf, raw, _ = named_rig((provider, primitive, retained_rig[2], references,
        app.state.operator_command_plane.store, root), monkeypatch, (26213, 42413))
    plane = app.state.operator_command_plane
    client = TestClient(app)
    _, noop = submit(client, provider, 'queued-noop', 'LOC_OC')
    _, cancelled = submit(client, provider, 'queued-cancel', 'LOC_RC')
    _, valid = submit(client, provider, 'queued-valid', 'LOC_MS')
    safety = plane.store.recovery()
    cancel = {'schema_version': 'bioxp.operator_mutation.v1', 'idempotency_key': 'cancel-intent',
        'expected_version': plane.store.get_command(cancelled)['state_version'],
        'expected_recovery_epoch': safety['recovery_epoch'],
        'expected_global_safety_epoch': safety['global_safety_epoch'],
        'expected_axis_safety_epoch': safety['z_safety_epoch']}
    response = client.post('/operator/commands/' + cancelled + '/cancel', json=cancel)
    assert response.status_code == 200, response.text
    assert [row['command_id'] for row in plane.store.queue()['items']] == [noop, valid]
    plane.start()
    first = finish(client, noop)
    last = finish(client, valid)
    assert first['status'] == last['status'] == 'completed'
    assert first['completion_class'] == 'source_noop'
    assert first['deck_movement']['delivery_attempted'] is False
    assert last['deck_movement']['controller_completion_verified'] is True
    assert len(leaf.moves) == 1
    assert finish(client, cancelled)['status'] == 'cleared'


def test_uncertain_native_result_holds_later_intent(installed_retained, retained_rig, monkeypatch):
    from tests.test_deck_near_terminal import named_rig
    app, provider, primitive, references, root = installed_retained
    ready(installed_retained, monkeypatch, retained_rig)
    plane = app.state.operator_command_plane
    leaf, raw, _ = named_rig((provider, primitive, retained_rig[2], references, plane.store, root),
        monkeypatch, (0, 0), 'missing_event')
    client = TestClient(app)
    body, first = submit(client, provider, 'uncertain-first', 'LOC_OC')
    _, later = submit(client, provider, 'uncertain-later', 'LOC_MS')
    plane.start()
    failed = finish(client, first)
    assert failed['status'] == 'ambiguous', failed
    assert plane.store.wait_for_command_workers([first], timeout=3)
    assert plane.store.get_command(later)['status'] == 'queued'
    assert plane.store.claim_next() is None
    assert plane.store.deck_recovery_blocker() == 'deck_recovery_hold'
    before = list(leaf.moves)
    replay = client.post(URL, json=body)
    assert replay.status_code == 200 and replay.json()['command_id'] == first
    # 2026-09-21: an uncertain outcome no longer refuses new intents.
    admitted = client.post(URL, json={**body, 'idempotency_key': 'uncertain-new'})
    assert admitted.status_code == 200 and 'deck_recovery_hold' not in admitted.text, admitted.text
    assert leaf.moves == before
    assert catalog_action(app)['enabled'] is True
