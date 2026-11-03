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
    # Bind the plane to the fixture's real store instead of a fresh default root.
    monkeypatch.setattr('bioxp.operator_command_plane.OperatorCommandStore', lambda *a, **k: store)
    plane = OperatorCommandPlane(app, machine_state_provider=lambda: {}, actions=[], dispatch={})
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


def test_tip_tray_type_record_is_read_back_and_used_by_pickup(client):
    http, provider, store, frames = client
    frames.clear()
    response = http.post('/operator/deck/record', json=body('rec-type-0001', tip_trays=[
        {'tray': 1, 'tip_type': 10}, {'tray': 2, 'tip_type': 10, 'tips': 'full'}]))
    assert response.status_code == 200, response.text
    assert [row['tip_type'] for row in response.json()['tip_types']] == ['T10', 'T10']
    trays = {row['tray']: row for row in http.get('/operator/tip-trays').json()['trays']}
    assert trays[1]['tip_type'] == 'T10' and trays[2]['tip_type'] == 'T10' and trays[3]['tip_type'] == 'T50'
    assert provider._read_constructed_tip_tray(0)['tip_type'] == 'T10'
    assert frames == []
    # Live records that predate the persisted tray constructor still accept a size.
    state = provider._load_state()
    state['machine_status'].pop('constructed_tip_trays')
    state['machine_status'].pop('construction_id')
    provider._save_state(state)
    legacy = http.post('/operator/deck/record', json=body('rec-type-0004', tip_trays=[{'tray': 3, 'tip_type': 10}]))
    assert legacy.status_code == 200, legacy.text
    trays = {row['tray']: row for row in http.get('/operator/tip-trays').json()['trays']}
    assert trays[1]['tip_type'] == 'T10' and trays[3]['tip_type'] == 'T10' and trays[4]['tip_type'] is None
    assert http.post('/operator/deck/record', json=body('rec-type-0002',
        tip_trays=[{'tray': 1, 'tip_type': 20}])).status_code == 422
    assert http.post('/operator/deck/record', json=body('rec-type-0003',
        tip_trays=[{'tray': 1}])).status_code == 422


def test_gantry_record_clears_stuck_cover_without_motion(client):
    http, provider, store, frames = client
    provider.gantry_load(tip_loaded=False, plate_on_gantry='OUTPUT_COVER')
    assert store.deck_semantic_state()['plate_on_gantry'] == 4
    frames.clear()
    response = http.post('/operator/deck/record', json={'gantry': 'empty',
        'movable_objects': {'OUTPUT_COVER': 'LOC_OC_COVER'}})
    assert response.status_code == 200, response.text
    result = response.json()
    assert result['motion_commanded'] is False and result['gantry']['plate_on_gantry'] is None
    after = store.deck_semantic_state()
    assert after['plate_on_gantry'] is None and after['pseudo_z_home'] == 65000
    assert provider._load_state()['machine_status']['plate_on_gantry'] is None
    assert frames == []
    # Setting a held object is the same setter; unknown names are 422.
    held = http.post('/operator/deck/record', json={'gantry': 'REAGENT_COVER'})
    assert held.status_code == 200 and store.deck_semantic_state()['plate_on_gantry'] == 5
    assert http.post('/operator/deck/record', json={'gantry': 'NOPE'}).status_code == 422


def test_record_stamps_do_not_refuse_unknown_board_epochs():
    import threading
    from types import SimpleNamespace
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as P
    fake = SimpleNamespace(generation_provider=lambda: 3, state_store=None, preparation_provider=None,
                           _lock=threading.RLock(), _load_state=lambda: {})
    assert P._deck_owner_authority_stamps_locked(fake) == {
        'ownership_generation': 3, 'board_epoch_4': 0, 'board_epoch_5': 0}


def test_cavro_failure_with_every_step_recorded_is_failed_not_ambiguous():
    from bioxp.oem_deck_movement import _known_cavro_failure
    done = [{'status': 'completed'}, {'status': 'failed'}]
    assert _known_cavro_failure({'kind': 'cavro_application', 'ok': False, 'events': done})
    assert not _known_cavro_failure({'kind': 'cavro_application', 'ok': False, 'events': done, 'interrupted': True})
    assert not _known_cavro_failure({'kind': 'cavro_application', 'ok': False, 'events': [{'status': 'in_flight'}]})
    assert not _known_cavro_failure({'kind': 'cavro_application', 'ok': False, 'events': []})
    assert not _known_cavro_failure({'kind': 'other', 'ok': False, 'events': done})


def test_known_failure_travels_on_the_deck_exception():
    from bioxp.oem_deck_movement import DeckExecutionFailure
    assert DeckExecutionFailure('x', delivery_attempted=True, known_failure=True).known_failure is True
    assert DeckExecutionFailure('x', delivery_attempted=True).known_failure is False


def test_manual_pickup_removes_rack_tips_and_sets_tip_type_from_rack_size():
    from types import SimpleNamespace
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as P
    calls = []
    fake = SimpleNamespace(tip_tray_type=lambda tray: 'T10')
    transport = SimpleNamespace(loadTip=lambda value, location: calls.append(('type', value, location)) or {'ok': True})
    out = P._manual_pickup_bookkeeping(fake, {'tray': 1, 'well': 'A2'},
        lambda name, inputs: calls.append((name, inputs)) or {'ok': True},
        lambda name, call: call(transport))
    assert calls[0] == ('pipette_tip_transition', {'tray_id': 0, 'well_ids': [1, 25, 49, 73], 'transition': 'remove'})
    assert calls[1] == ('type', 10, -1)
    assert out['rack_tips_removed'] == [1, 25, 49, 73]
    calls.clear()
    P._manual_pickup_bookkeeping(SimpleNamespace(tip_tray_type=lambda tray: None), {'tray': 3, 'well': 'B1'},
        lambda name, inputs: calls.append((name, inputs)) or {'ok': True}, lambda name, call: call(transport))
    assert calls == [('pipette_tip_transition', {'tray_id': 2, 'well_ids': [12, 36, 60, 84], 'transition': 'remove'})]


def test_partial_pickup_releases_unneeded_tips_after_short_lift_and_books_only_kept_wells():
    from types import SimpleNamespace
    from bioxp.pipette.oem_calibration import CalibrationBindings, ManualTipLoadRequest, manual_load_tip
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as P
    calls = []
    chans = [SimpleNamespace(_tip_loaded=True) for _ in range(4)]
    group = SimpleNamespace(_transports=chans,
        query_tip_status_all=lambda: {'ok': True, 'channels': [
            {'channel': ch, 'tip_loaded': True, 'result': {'ok': True, 'semantic_ok': True,
             'hardware_truth_level': 'hardware_query'}} for ch in range(4)]},
        _eject_tip_channels_once=lambda ch, wait_for_completion: calls.append(('eject', ch)) or [
            {'channel': channel, 'result': {'ok': True, 'completion_verified': True}}
            for channel in ch])
    b = CalibrationBindings(
        finite=lambda name, inputs: calls.append((name, dict(inputs))) or {'ok': True},
        native=lambda name, call, inputs: call(), pipette=lambda name, call: call(group),
        facts=dict, tray_location=lambda i: 7,
        lower_pipette=lambda loc, op: {'ok': True, 'target_position_steps': 150000},
        z_low=lambda loc: None, set_z_speed=None, detection_move=None, stop_z=None)
    out = manual_load_tip(ManualTipLoadRequest(tray=1, well='A2', keep_channels=(2, 0)), b)
    assert ('pipette_move_z', {'value': 150000 - round(10.0 * 2015.748)}) in calls
    assert ('eject', [1, 3]) in calls and calls.index(('pipette_move_z', {'value': 150000 - round(10.0 * 2015.748)})) < calls.index(('eject', [1, 3]))
    assert [c._tip_loaded for c in chans] == [True, False, True, False]
    assert out['kept_channels'] == [0, 2] and out['released_channels'] == [1, 3]
    books = []
    P._manual_pickup_bookkeeping(SimpleNamespace(tip_tray_type=lambda tray: None),
        {'tray': 1, 'well': 'A2', 'keep_channels': [0, 2]},
        lambda name, inputs: books.append(inputs) or {'ok': True}, lambda name, call: None)
    assert [entry['well_ids'] for entry in books] == [[1], [49]]
    calls.clear()
    manual_load_tip(ManualTipLoadRequest(tray=1, well='A2'), b)
    assert not any(c[0] in ('eject', 'pipette_move_z') for c in calls)


def test_partial_pickup_accepts_frozen_protocol_params():
    from bioxp.manual_pipetting import manual_physical_plan
    plan = manual_physical_plan({'operation': 'load_tip', 'tray': 1, 'well': 'A10', 'overpress': False,
                                 'lift_z': True, 'keep_channels': (0,), 'release_lift_mm': 5.0})
    child = plan['children'][0] if 'children' in plan else plan['plan']['children'][0]
    assert child['arguments']['keep_channels'] == [0] and child['arguments']['release_lift_mm'] == 5.0
    from bioxp.operator_command_plane import _digest
    import json
    stored = json.loads(json.dumps(plan))
    assert stored['plan_digest'] == _digest({k: v for k, v in stored.items() if k != 'plan_digest'})
    plain = manual_physical_plan({'operation': 'load_tip', 'tray': 1, 'well': 'A10', 'overpress': False, 'lift_z': True})
    child = plain['children'][0] if 'children' in plain else plain['plan']['children'][0]
    assert 'keep_channels' not in child['arguments']
