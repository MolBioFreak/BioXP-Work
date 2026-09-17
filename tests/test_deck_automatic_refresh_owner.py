"""Automatic query refresh must yield to the actual named-command owner."""
import json
import os
import threading
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained


def request(provider, key):
    stamps = provider.deck_owner_authority_stamps()
    return {'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': key,
        'expected_ownership_generation': stamps['ownership_generation'],
        'expected_board_epoch_by_board': {'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']},
        'inputs': {'target': 'TECANRACK2', 'camera_offset': False}}


def finish(client, command_id):
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        row = client.get('/operator/v2/actions/receipts/' + command_id + '?detail=true').json()
        if row['terminal']:
            return row
        time.sleep(.01)
    pytest.fail('real named command did not finish')


@pytest.mark.parametrize('boundary', ['http', 'worker'])
def test_named_dispatch_excludes_automatic_refresh(installed_retained, monkeypatch, boundary):
    from bioxp import api
    app, provider, primitives, references, root = installed_retained
    # Let the real projection retire copied old-generation lifecycle first;
    # only then establish this offline fixture's explicit current references.
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    client = TestClient(app)
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=request(provider, boundary))
    assert response.status_code == 200, response.text
    command_id = response.json()['command_id']
    entered, release = threading.Event(), threading.Event()
    original = provider._fresh_deck_latch_observation
    def latch():
        result = original()
        if (threading.current_thread().name.startswith('bioxp-operator-command-')
                and provider._load_state()['machine_status'].get('pseudo_home_command_id') == command_id):
            entered.set()
            assert release.wait(5), 'test did not release observation'
        return result
    monkeypatch.setattr(provider, '_fresh_deck_latch_observation', latch)
    collections = []
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    def collect(*args, **kwargs):
        collections.append(True)
        return {'ok': False, 'snapshot': {}}
    monkeypatch.setattr(api.hardware_state, 'collect', collect)
    app.state.operator_command_plane.start()
    try:
        assert entered.wait(5), 'named source never reached post-ForceToHighHome observation'
        assert command_id in app.state.operator_command_plane.store.live_command_worker_ids()
        epoch = provider._deck_authority_cache_epoch
        if boundary == 'http':
            refreshed = client.post('/hardware/snapshot/collect', json={'automatic': True, 'domains': ['axes', 'latch']}).json()
        else:
            # The automatic HTTP precheck may pass before its worker is scheduled.
            refreshed = api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='offline-automatic', automatic=True)
        same_epoch = epoch is provider._deck_authority_cache_epoch
    finally:
        release.set()
    receipt = finish(client, command_id)
    evidence = {'boundary': boundary, 'refresh': refreshed, 'cache_epoch_preserved': same_epoch,
        'collections': len(collections), 'receipt': receipt, 'primitive_calls': primitives.calls}
    output = os.environ.get('DECK_TEST_OUTPUT')
    if output:
        Path(output + '.' + boundary + '.json').write_text(json.dumps(evidence, indent=2))
    assert refreshed == {'ok': False, 'published': False, 'reason': 'operator_action_pending'}, evidence
    assert same_epoch and not collections
    assert receipt['status'] == 'completed', receipt
    assert receipt['deck_movement']['semantic_state_committed'] is True
    assert receipt['deck_movement']['controller_completion_verified'] is True
    assert app.state.operator_command_plane.store.wait_for_command_workers([command_id], timeout=2)
    assert app.state.operator_normal_action_active() is False


@pytest.mark.parametrize('generation_delta', [0, 1])
def test_named_admission_yields_refresh_and_releases_on_exit(installed_retained, monkeypatch, generation_delta):
    from bioxp import api
    app, provider, primitives, references, root = installed_retained
    # Let the real projection retire copied old-generation lifecycle first;
    # only then establish this offline fixture's explicit current references.
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    body = request(provider, 'admission-' + str(generation_delta))
    body['expected_ownership_generation'] += generation_delta
    reader = app.state.operator_admission_state_reader
    original = reader._collect
    entered, release = threading.Event(), threading.Event()
    def collect_state():
        entered.set()
        assert release.wait(3)
        return original()
    monkeypatch.setattr(reader, '_collect', collect_state)
    collections = []
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    monkeypatch.setattr(api.hardware_state, 'collect', lambda *a, **k: collections.append(True) or {'ok': False})
    results = []
    def submit():
        results.append(TestClient(app, raise_server_exceptions=False).post('/operator/v2/actions/oem.deck.move_to_location', json=body))
    thread = threading.Thread(target=submit)
    thread.start()
    try:
        assert entered.wait(3)
        pending = app.state.operator_normal_action_active()
        refreshed = api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='offline-automatic', automatic=True)
    finally:
        release.set()
        thread.join(5)
    assert not thread.is_alive()
    assert pending is True
    assert refreshed == {'ok': False, 'published': False, 'reason': 'operator_action_pending'}
    assert not collections
    assert app.state.operator_normal_action_active() is False
    assert results[0].status_code == (200 if generation_delta == 0 else 409), results[0].text


def test_automatic_query_refresh_resumes_when_idle(installed_retained, monkeypatch):
    from bioxp import api
    app, provider, primitives, references, root = installed_retained
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    called = []
    monkeypatch.setattr(api.hardware_state, 'collect', lambda *a, **k: called.append(True) or {'ok': False})
    assert app.state.operator_normal_action_active() is False
    result = api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='offline-idle', automatic=True)
    assert result == {'ok': False} and called == [True]


def test_admission_preview_metadata_timeout_remains_fail_closed(installed_retained, monkeypatch):
    app, provider, primitives, references, root = installed_retained
    reader = app.state.operator_preview_state_reader
    entered, release = threading.Event(), threading.Event()
    def blocked_state():
        entered.set()
        assert release.wait(3)
        raise RuntimeError('offline read released after caller timed out')
    monkeypatch.setattr(reader, '_collect', blocked_state)
    body = {'expected_generation': int(provider.generation_provider()), 'inputs': {}}
    results = []
    thread = threading.Thread(target=lambda: results.append(TestClient(app).post(
        '/operator/actions/oem.deck.move_to_location/admission', json=body)))
    thread.start()
    try:
        assert entered.wait(2)
        assert app.state.operator_normal_action_active() is False
        thread.join(1.5)
        assert not thread.is_alive(), 'existing bounded metadata timeout was removed'
        assert results[0].status_code == 503
        assert results[0].json() == {'detail': 'operator_state_warming'}
        assert app.state.operator_normal_action_active() is False
        assert not app.state.operator_command_plane.store.live_command_worker_ids()
    finally:
        release.set()
        thread.join(3)
        try:
            reader._pending.result(timeout=3)
        except RuntimeError as exc:
            assert str(exc) == 'offline read released after caller timed out'
