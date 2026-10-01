"""Current SQLite location must not wait behind cached hardware/history views."""
import asyncio
import json
import threading
import time

import pytest
from fastapi.testclient import TestClient

from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained


def qualify_display_predecessor(retained_rig):
    # Reuse the retained fixture's real provider/store bootstrap. Constructor
    # identity is irrelevant to this display test and is deliberately preserved.
    from bioxp.oem_deck_movement import OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS
    provider, primitive, runtime, references, store, root = retained_rig
    qualify_test_references(references)
    state = provider._load_state()
    state['machine_status'].update(current_location='LOC_PARK', current_well=0,
        tip_loaded=False, tip_dirty=False, tip_location=-1, clean_path=False,
        plate_on_gantry=None, movable_plate_locations=dict(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS))
    state['machine_status'].pop('latch_observation_id', None)
    state['machine_status'].pop('latch_closed', None)
    provider._save_state(state)
    provider.bind_tip_tray_state_reader(store.tip_tray_state)
    store.publish_tip_tray_transition(tray_id=0, transition='construct',
        operation_id='display-fixture-construction', command_id='display-fixture-construction',
        provenance={'source': 'explicit isolated display predecessor'},
        **provider.deck_owner_authority_stamps())
    snapshot = provider.deck_authority_snapshot(
        expected_generation=int(provider.generation_provider()), target='LOC_PARK')
    assert snapshot['current_location_id'] == 'LOC_PARK'



VIEWS = [
    '/operator/v2/dashboard',
    '/operator/v2/control-catalog',
    '/operator/dashboard?schema_version=bioxp.operator_dashboard.v2',
    '/operator/control-catalog?schema_version=bioxp.operator_control_catalog.v2',
]


def dashboard(body):
    return body.get('dashboard', body)


def warm(client, app, path):
    response = client.get(path)
    pending = app.state.operator_poll_cache._pending
    if pending is not None:
        pending.result(timeout=5)
    response = client.get(path)
    assert response.status_code == 200, response.text
    pending = app.state.operator_poll_cache._pending
    if pending is not None:
        pending.result(timeout=5)
    return response.json()


def complete_move(client, app, provider, target):
    catalog = warm(client, app, '/operator/v2/control-catalog')
    action = next(row for row in catalog['actions'] if row['action_id'] == 'oem.deck.move_to_location')
    key = 'display-regression-' + target
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2',
        'idempotency_key': key,
        'expected_ownership_generation': int(provider.generation_provider()),
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': target, 'camera_offset': False},
    }, headers={'Idempotency-Key': key})
    assert response.status_code == 200, response.text
    cid = response.json()['command_id']
    app.state.operator_command_plane.start()
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        receipt = client.get('/operator/v2/actions/receipts/' + cid).json()
        if receipt['terminal']:
            assert receipt['status'] == 'completed', receipt
            return cid
        time.sleep(.01)
    pytest.fail('isolated queued move did not terminalize')


@pytest.mark.parametrize('path', VIEWS)
def test_committed_arrival_updates_warm_view_during_blocked_refresh(
        installed_retained, retained_rig, monkeypatch, path):
    from bioxp import operator_controls as controls
    app, provider, primitive, references, root = installed_retained
    entered, release = threading.Event(), threading.Event()
    with TestClient(app) as client:
        # Establish the actual copied owner and source predecessor, then warm
        # the ordinary API view before the changing-location publication.
        warm(client, app, '/operator/v2/control-catalog')
        qualify_display_predecessor(retained_rig)
        initial = warm(client, app, path)
        assert dashboard(initial)['deck']['current_location'] == 'LOC_PARK'
        cid = complete_move(client, app, provider, 'LOC_OC')
        actual = app.state.operator_command_plane.store.deck_semantic_state()
        assert actual['current_location'] == 'LOC_OC'
        assert actual['producer_command_id'] == cid
        before = list(primitive.calls)
        project = controls.hardware_state.project

        def blocked_project(*args, **kwargs):
            if controls._PASSIVE_OPERATOR_POLL.get():
                entered.set()
                assert release.wait(8), 'blocked metadata fixture was not released'
            return project(*args, **kwargs)

        monkeypatch.setattr(controls.hardware_state, 'project', blocked_project)
        try:
            response = client.get(path)
            assert response.status_code == 200, response.text
            assert entered.wait(2), 'the existing hardware refresh did not run'
            body = response.json()
            assert dashboard(body)['deck']['current_location'] == 'LOC_OC'
            assert dashboard(body)['deck']['semantic_state_revision'] == actual['semantic_state_revision']
            assert body.get('actions') == initial.get('actions')
            assert primitive.calls == before  # display read never queries/moves hardware
        finally:
            release.set()
            pending = app.state.operator_poll_cache._pending
            if pending is not None:
                pending.result(timeout=5)


@pytest.mark.parametrize('path', VIEWS)
def test_standalone_unknown_updates_warm_location_without_motion(
        installed_retained, retained_rig, path):
    from bioxp.deck_location_invalidation import invalidate_at_dispatch, named_location_published
    app, provider, primitive, references, root = installed_retained
    with TestClient(app) as client:
        warm(client, app, '/operator/v2/control-catalog')
        qualify_display_predecessor(retained_rig)
        initial = warm(client, app, path)
        assert dashboard(initial)['deck']['current_location'] == 'LOC_PARK'
        store = app.state.operator_command_plane.store
        try:
            # Reuse the real dispatch bookkeeping primitive on the copied DB.
            # No synthetic motor execution or physical-effect claim is made.
            with store._deck_owner_authority_scope(), store._transaction() as conn:
                invalidate_at_dispatch(conn, root=root, command_id='fixture-standalone-dispatch')
            actual = store.deck_semantic_state()
            assert actual['current_location'] == 'UNKNOWN'
            before = list(primitive.calls)
            response = client.get(path)
            assert response.status_code == 200, response.text
            assert dashboard(response.json())['deck']['current_location'] == 'UNKNOWN'
            assert dashboard(response.json())['deck']['semantic_state_revision'] == actual['semantic_state_revision']
            assert response.json().get('actions') == initial.get('actions')
            assert primitive.calls == before
        finally:
            named_location_published(root)


def test_narrow_display_reader_uses_one_read_only_query(installed_retained):
    app, provider, primitive, references, root = installed_retained
    store = app.state.operator_command_plane.store
    sql = []
    before = list(primitive.calls)
    with store._lock:
        store.connection.set_trace_callback(sql.append)
        try:
            display = store.deck_display_state()
        finally:
            store.connection.set_trace_callback(None)
    assert set(display) == {'current_location', 'current_well', 'semantic_state_revision', 'ambiguity_state'}
    assert len(sql) == 1 and sql[0].lstrip().upper().startswith('SELECT ')
    assert primitive.calls == before


@pytest.mark.parametrize('path', VIEWS)
def test_unavailable_display_read_keeps_warm_controls(installed_retained, monkeypatch, path):
    from sqlite3 import OperationalError
    app, provider, primitive, references, root = installed_retained
    with TestClient(app) as client:
        initial = warm(client, app, path)
        reader = app.state.operator_deck_display_reader

        def unavailable():
            raise OperationalError('isolated display read failure')

        monkeypatch.setattr(reader, '_collect', unavailable)
        before = list(primitive.calls)
        response = client.get(path)
        assert response.status_code == 200, response.text
        body = response.json()
        assert body.get('actions') == initial.get('actions')
        assert dashboard(body)['deck'] == dashboard(initial)['deck']
        assert primitive.calls == before


def test_slow_display_read_coalesces_and_preserves_warm_view(installed_retained, monkeypatch):
    app, provider, primitive, references, root = installed_retained
    entered, release = threading.Event(), threading.Event()
    attempts = []
    with TestClient(app) as client:
        initial = warm(client, app, '/operator/v2/dashboard')
        reader = app.state.operator_deck_display_reader
        original = reader._collect

        def held():
            attempts.append(1)
            entered.set()
            assert release.wait(8)
            return original()

        monkeypatch.setattr(reader, '_collect', held)
        before = list(primitive.calls)
        try:
            for _ in range(2):
                response = client.get('/operator/v2/dashboard')
                assert response.status_code == 200, response.text
                assert dashboard(response.json())['deck'] == dashboard(initial)['deck']
            assert entered.is_set() and len(attempts) == 1
            assert primitive.calls == before
        finally:
            release.set()
            reader._pending.result(timeout=5)
    assert reader._closed is True
