"""Record-only deck/tip-tray correction and read routes: real provider, store and SQLite.

No controller I/O is expected; the wire recorder must stay empty across records.
"""
from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from tests.test_debloat_core import named  # noqa: F401  (connected provider + SQLite fixture)
from tests.test_wake_setup_debloat import rig  # noqa: F401
from bioxp.operator_command_plane import OperatorCommandPlane


@pytest.fixture
def client(named, monkeypatch):
    run, provider, store, counts, frames, fault = named
    app = FastAPI()
    app.state.oem_deck_provider = provider
    plane = OperatorCommandPlane(app, machine_state_provider=lambda: {}, actions=[], dispatch={})
    plane.store.stop()
    plane.store = store
    app.include_router(plane.router)
    return TestClient(app), provider, store, frames


def body(record_id, **fields):
    return {'schema_version': 'bioxp.operator_deck_record_request.v1', 'record_id': record_id,
            'reason': 'cover physically on output chiller', **fields}


def test_cover_record_correction_moves_nothing_and_reaches_both_stores(client):
    http, provider, store, frames = client
    frames.clear()
    before = store.deck_semantic_state()
    response = http.post('/operator/deck/record', json=body('rec-cover-0001',
        movable_objects={'OUTPUT_COVER': 'LOC_OC_COVER'}))
    assert response.status_code == 200, response.text
    result = response.json()
    assert result['motion_commanded'] is False
    after = store.deck_semantic_state()
    assert after['movable_plate_locations']['OUTPUT_COVER'] == 'LOC_OC_COVER'
    assert after['semantic_state_revision'] == before['semantic_state_revision'] + 1
    assert after['producer_operation'] == 'updatePlateLocation'
    machine = provider._load_state()['machine_status']
    assert machine['movable_plate_locations']['OUTPUT_COVER'] == 'LOC_OC_COVER'
    # The catch planner now reads the corrected source location (OEM catchPlate input).
    assert provider.mov_execution_machine_state()['plate_locations'][4] == 17
    assert frames == []
    state = http.get('/operator/deck/state').json()
    assert state['movable_objects']['OUTPUT_COVER'] == {'location': 'LOC_OC_COVER', 'source': 'sqlite_record'}
    # Same record id again is reported, not replayed.
    again = http.post('/operator/deck/record', json=body('rec-cover-0001',
        movable_objects={'OUTPUT_COVER': 'LOC_OC_COVER'}))
    assert again.status_code == 409


def test_invalid_objects_and_empty_record_are_422(client):
    http, *_ = client
    assert http.post('/operator/deck/record', json=body('rec-bad-0001',
        movable_objects={'OUTPUT_COVER': 'NOWHERE'})).status_code == 422
    assert http.post('/operator/deck/record', json=body('rec-empty-001')).status_code == 422
    collide = http.post('/operator/deck/record', json=body('rec-coll-001',
        movable_objects={'OUTPUT_COVER': 'LOC_RC_COVER', 'REAGENT_COVER': 'LOC_RC_COVER'}))
    assert collide.status_code == 422


def test_tip_tray_records_full_empty_and_partial(client):
    http, provider, store, frames = client
    frames.clear()
    response = http.post('/operator/deck/record', json=body('rec-tips-0001', tip_trays=[
        {'tray': 1, 'tips': 'full'}, {'tray': 2, 'tips': 'empty'},
        {'tray': 3, 'tips': ['A1', 'C1', 'E1', 'G1', 'B2']}]))
    assert response.status_code == 200, response.text
    trays = {row['tray']: row for row in http.get('/operator/tip-trays').json()['trays']}
    assert trays[1]['tips_present'] == 96 and trays[1]['four_channel_groups_available'] == 24
    assert trays[2]['tips_present'] == 0 and trays[2]['tip_available'] is False
    assert trays[3]['tips_present'] == 5 and trays[3]['four_channel_groups_available'] == 1
    assert store.tip_tray_state(2)['occupancy'][0] is True  # A1 on tray 3
    assert frames == []
