"""Public V2 plate custody: real API/store/finite worker/provider/adapter, inert wire."""
import time
import pytest
from tests.test_debloat_core import named
from tests.test_wake_setup_debloat import rig
from tests.test_fast_root_fifo import terminal
from tests.test_movement_modes import body


@pytest.fixture
def public(named, monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from bioxp import api, operator_controls
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    from tests import z_stop_fixtures
    _, provider, store, _, frames, fault = named
    # The inert controller starts with the native constructor's real profile
    # writes, not manually seeded profile/lifecycle success flags.
    assert provider.primitives.tester.oem_initialize_without_motion_test_case()['ok'] is True
    driver = provider.primitives.tester
    wire = driver.send_tmcl
    door = {'position': int(driver._motion_oem_axis_profile('door')['open_position'])}
    def exchange(board, command, typ, motor, value, **kwargs):
        reply = wire(board, command, typ, motor, value, **kwargs)
        if (board, motor) == (6, 0):
            if command == 4:
                door['position'] = value
            elif command == 6 and typ == 1:
                reply['value'] = door['position']
            elif command == 6 and typ in {9, 10}:
                reply['value'] = int(door['position'] == 0 if typ == 9 else door['position'] != 0)
        return reply
    monkeypatch.setattr(driver, 'send_tmcl', exchange)
    monkeypatch.setattr(driver, 'send_tmcl_retry', exchange)
    app = FastAPI()
    monkeypatch.setattr(api, '_get_tester', lambda: provider.primitives.tester)
    for path, endpoint in (
        ('/motion/oem/x/abort', api.motion_oem_x_abort),
        ('/motion/oem/x/stop', api.motion_oem_x_stop),
        ('/motion/oem/y/stop', api.motion_oem_y_stop),
        ('/motion/oem/z/stop', api.motion_oem_z_stop),
    ):
        app.add_api_route(path, endpoint, methods=['POST'])
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    monkeypatch.setattr(z_stop_fixtures, 'FastAPI', lambda: app)
    monkeypatch.setattr('bioxp.operator_command_plane.OperatorCommandStore', lambda *a, **kw: store)
    install = operator_controls.install_operator_control_plane
    def with_owner(application, **kwargs):
        kwargs.update(oem_deck_provider=lambda: provider,
            oem_deck_position_table_provider=load_bound_oem_position_table,
            serial206_initialization_state_provider=api.serial206_oem_initialization_provider_status)
        return install(application, **kwargs)
    monkeypatch.setattr(operator_controls, 'install_operator_control_plane', with_owner)
    z_stop_fixtures.make_app(store.root, monkeypatch)
    # Constructor logical no-tip branch, not a sensor absence assertion.
    provider.bind_pipette_collection_state_reader(lambda: {'tip_exists': False})
    project = operator_controls.hardware_state.project
    monkeypatch.setattr(operator_controls.hardware_state, 'project',
        lambda *a, include_lifecycle=False, **kw: project(*a, **kw))
    provider.wp8_update_thermal_door_open('updateThermalDoorOpen', {'value': True},
        command_id='fixture-source-door', child_order=0, plan_digest='fixture')
    store.publish_deck_owner_state(source_operation='updatePlateLocation', source_command_id='fixture-output',
        updates={'movable_plate_locations': {'OUTPUT_PLATE': 'LOC_P_OC'}},
        **provider.deck_owner_authority_stamps())
    plane = app.state.operator_command_plane
    with TestClient(app) as http:
        plane.start()
        try:
            yield http, provider, store, frames, fault
        finally:
            plane.stop()


def cold_custody(store, held, location):
    import json, subprocess, sys
    code = ('import json,sqlite3,sys; c=sqlite3.connect(sys.argv[1]); '
        'r=c.execute("SELECT plate_on_gantry,movable_plate_locations_json FROM operator_plane_deck_semantic_state").fetchone(); '
        'print(json.dumps([r[0],json.loads(r[1])["OUTPUT_PLATE"]])); c.close()')
    observed = json.loads(subprocess.check_output([sys.executable, '-c', code,
        str(store.root / 'bioxp_runtime.db')], text=True))
    # SQLite stores the canonical object name; the native reader projects its ordinal.
    assert observed == ['OUTPUT_PLATE' if held == 1 else None, location]


def submit(http, action, inputs, key):
    response = http.post('/operator/v2/actions/oem.deck.' + action, json=body(key, inputs))
    assert response.status_code == 200, response.text
    return response.json()['command_id']


def test_public_catalog_and_connected_pickup_then_dropoff(public):
    http, provider, store, frames, fault = public
    catalog = http.get('/operator/v2/control-catalog').json()
    rows = {r['action_id']: r for r in catalog['actions']}
    for action in ('catch_plate', 'release_plate'):
        assert 'oem.deck.' + action in rows
        assert rows['oem.deck.' + action]['enabled'] is True
    metadata = http.get('/operator/v2/control-catalog?view=metadata').json()
    for action in ('catch_plate', 'release_plate'):
        assert next(r for r in metadata['actions'] if r['action_id'] == 'oem.deck.' + action)['inputs'] == rows['oem.deck.' + action]['inputs']
    pickup = submit(http, 'catch_plate', {'plate': 1}, 'manual-pickup')
    caught = terminal(http, pickup)
    assert caught['action_id'] == 'oem.deck.catch_plate'
    assert caught['status'] == 'completed', caught
    compact = http.get('/operator/v2/actions/receipts/' + pickup).json()
    assert compact['plate_operation']['source_return_ok'] is True
    assert compact['plate_operation']['operation'] == 'catch_plate'
    assert compact['physical_effect_verified'] is False
    assert store.deck_semantic_state()['plate_on_gantry'] == 1
    assert store.deck_semantic_state()['movable_plate_locations']['OUTPUT_PLATE'] == 'LOC_GANTRY'
    cold_custody(store, 1, 'LOC_GANTRY')
    dropoff = submit(http, 'release_plate', {'destination': 21, 'press_plate': False,
        'run_in_parallel': False}, 'manual-dropoff')
    released = terminal(http, dropoff)
    assert released['action_id'] == 'oem.deck.release_plate'
    assert released['status'] == 'completed', released
    assert pickup != dropoff
    assert store.deck_semantic_state()['plate_on_gantry'] is None
    assert store.deck_semantic_state()['movable_plate_locations']['OUTPUT_PLATE'] == 'LOC_P_OC'
    cold_custody(store, None, 'LOC_P_OC')
    for cid in (pickup, dropoff):
        evidence = store.wp8_operation_evidence(cid)
        assert evidence['children']
        assert all(c['terminal_state'] != 'planned' for c in evidence['children'])
        assert store.connection.execute('SELECT parent_command_id FROM operator_commands WHERE command_id=?',
            (cid,)).fetchone()[0] is None
        assert store.connection.execute("SELECT count(*) FROM operator_commands WHERE parent_command_id=?",
            (cid,)).fetchone()[0] == 0
    import json, os
    from pathlib import Path
    if output := os.environ.get('BIOXP_MANUAL_PLATE_EXPORT'):
        Path(output).write_text(json.dumps({'catalog': [rows['oem.deck.' + a] for a in ('catch_plate', 'release_plate')],
            'caught': caught, 'released': released,
            'catch_evidence': store.wp8_operation_evidence(pickup),
            'release_evidence': store.wp8_operation_evidence(dropoff)}, indent=2))
    from bioxp.operator_receipt_store import OperatorReceiptStore
    cold = OperatorReceiptStore(store.root)
    try:
        assert cold.by_command(pickup)['action_id'] == 'oem.deck.catch_plate'
        assert cold.by_command(pickup)['plate_operation']['source_return_ok'] is True
        assert cold.by_command(dropoff)['action_id'] == 'oem.deck.release_plate'
    finally:
        cold.connection.close()


@pytest.mark.parametrize('action,inputs', [
    ('catch_plate', {}), ('catch_plate', {'plate': True}),
    ('catch_plate', {'plate': -1}), ('catch_plate', {'plate': 999}),
    ('catch_plate', {'plate': 1, 'run_in_parallel': 1}),
    ('catch_plate', {'plate': 1, 'plate_location': 21}),
    ('release_plate', {'destination': '21'}), ('release_plate', {'destination': True}),
    ('release_plate', {'destination': 999}),
    ('release_plate', {'destination': 21, 'press_plate': 0}),
    ('release_plate', {'destination': 21, 'run_in_parallel': None}),
])
def test_source_types_enum_and_no_private_machine_inputs(public, action, inputs):
    http, provider, store, frames, fault = public
    frames.clear()
    response = http.post('/operator/v2/actions/oem.deck.' + action,
        json=body('invalid-manual-plate', inputs))
    assert response.status_code == 422, response.text
    assert not frames


@pytest.mark.parametrize('parallel', [False, True])
@pytest.mark.parametrize('press', [False, True])
def test_release_explicit_source_options_and_replay(public, parallel, press):
    http, provider, store, frames, fault = public
    pickup = submit(http, 'catch_plate', {'plate': 1, 'run_in_parallel': parallel}, 'options-catch')
    assert terminal(http, pickup)['status'] == 'completed'
    inputs = {'destination': 21, 'press_plate': press, 'run_in_parallel': parallel}
    release = submit(http, 'release_plate', inputs, 'options-release')
    result = terminal(http, release)
    assert result['status'] == 'completed', result['source_receipt'].get('terminal_evidence')
    operations = [c['operation'] for c in store.wp8_operation_evidence(release)['children']]
    assert ('moveZPress' in operations) is press
    assert ('LockGripperOperation' in operations) is parallel
    assert store.deck_semantic_state()['plate_on_gantry'] is None
    before = list(frames)
    assert submit(http, 'release_plate', inputs, 'options-release') == release
    changed = http.post('/operator/v2/actions/oem.deck.release_plate',
        json=body('options-release', {**inputs, 'destination': 23}))
    assert changed.status_code == 409
    assert frames == before


@pytest.mark.parametrize('phase', ['catch', 'release'])
def test_transport_failure_keeps_actual_custody_and_child_receipts(public, phase):
    http, provider, store, frames, fault = public
    if phase == 'release':
        pickup = submit(http, 'catch_plate', {'plate': 1}, 'before-release-failure')
        assert terminal(http, pickup)['status'] == 'completed'
        assert store.deck_semantic_state()['plate_on_gantry'] == 1
    frames.clear()
    def fail(board, command, typ, motor, value):
        if command == 4 and board == 4 and motor == (2 if phase == 'catch' else 1):
            raise RuntimeError('offline-public-custody-transport-failure')
    fault['on_frame'] = fail
    cid = submit(http, 'catch_plate' if phase == 'catch' else 'release_plate',
        {'plate': 1} if phase == 'catch' else {'destination': 21, 'run_in_parallel': False},
        'public-custody-failure')
    failed = terminal(http, cid)
    assert failed['status'] != 'completed', failed
    assert failed['physical_effect_verified'] is False
    state = store.deck_semantic_state()
    assert state['plate_on_gantry'] == (None if phase == 'catch' else 1)
    assert state['movable_plate_locations']['OUTPUT_PLATE'] == ('LOC_P_OC' if phase == 'catch' else 'LOC_GANTRY')
    children = store.wp8_operation_evidence(cid)['children']
    assert children and any(c['terminal_state'] != 'completed' for c in children)
    assert not any(c['operation'] == 'updatePlateLocation' and c['terminal_state'] == 'completed' for c in children)


def test_mixed_head_pickup_dropoff_fifo_has_no_workflow_wrapper(public):
    import threading
    http, provider, store, frames, fault = public
    entered, release = threading.Event(), threading.Event()
    def hold(board, command, typ, motor, value):
        if command == 4 and board == 5 and not entered.is_set():
            entered.set()
            assert release.wait(10)
    fault['on_frame'] = hold
    requests = [('move_to_location', {'target': 'LOC_OC', 'camera_offset': False}),
        ('catch_plate', {'plate': 1}),
        ('release_plate', {'destination': 21, 'press_plate': False, 'run_in_parallel': False}),
        ('move_to_location', {'target': 'LOC_MS', 'camera_offset': False})]
    ids = []
    try:
        for index, (action, inputs) in enumerate(requests):
            start = time.monotonic()
            ids.append(submit(http, action, inputs, 'mixed-plate-fifo-' + str(index)))
            assert time.monotonic() - start < 2
            assert entered.wait(3)
        assert [store.get_command(cid)['status'] for cid in ids] == ['dispatched', 'queued', 'queued', 'queued']
    finally:
        release.set()
    receipts = [terminal(http, cid) for cid in ids]
    assert [r['status'] for r in receipts] == ['completed'] * len(ids), receipts
    assert [r['dispatched_at'] for r in receipts] == sorted(r['dispatched_at'] for r in receipts)
    assert all(receipts[i]['finished_at'] <= receipts[i+1]['dispatched_at'] for i in range(len(receipts)-1))
    assert len(set(ids)) == len(ids)
    assert store.deck_semantic_state()['movable_plate_locations']['OUTPUT_PLATE'] == 'LOC_P_OC'
    assert store.deck_semantic_state()['plate_on_gantry'] is None


@pytest.mark.parametrize('change', ['oem.x.stop', 'oem.y.stop', 'oem.z.stop', 'oem.abort_all', 'owner'])
def test_real_finite_fence_after_pickup_preserves_held_state(public, change):
    import threading
    http, provider, store, frames, fault = public
    caught = submit(http, 'catch_plate', {'plate': 1}, 'fence-catch')
    assert terminal(http, caught)['status'] == 'completed'
    entered, release = threading.Event(), threading.Event()
    def hold(board, command, typ, motor, value):
        if command == 4 and (board, motor) == (4, 0) and not entered.is_set():
            entered.set()
            assert release.wait(10)
    fault['on_frame'] = hold
    cid = None
    queued = []
    try:
        cid = submit(http, 'release_plate', {'destination': 25, 'press_plate': False,
            'run_in_parallel': False}, 'fence-release')
        assert entered.wait(3)
        if change != 'owner':
            queued = [submit(http, 'catch_plate', {'plate': 1}, 'queued-after-release'),
                submit(http, 'release_plate', {'destination': 21}, 'queued-release-suffix')]
            response = http.post('/operator/v2/actions/' + change, json={
                'schema_version': 'bioxp.operator_interrupt_request.v1',
                'idempotency_key': 'manual-release-stop', 'reason': 'offline qualification',
                'observed_ownership_generation': 7, 'observed_board_epoch_by_board': {}})
            assert response.status_code == 200, response.text
        else:
            provider.generation_provider = lambda: 8
    finally:
        release.set()
    result = terminal(http, cid)
    assert result['status'] != 'completed', result
    assert [terminal(http, q)['status'] for q in queued] == ['cleared'] * len(queued)
    cold_custody(store, 1, 'LOC_GANTRY')
    assert store.deck_semantic_state()['plate_on_gantry'] == 1
    assert store.deck_semantic_state()['movable_plate_locations']['OUTPUT_PLATE'] == 'LOC_GANTRY'
    assert not any(c['operation'] == 'LoadGantryNull' and c['terminal_state'] == 'completed'
        for c in store.wp8_operation_evidence(cid)['children'])


@pytest.mark.parametrize('plate,source,destination', [
    (0, 'LOC_P_TC', 23), (2, 'LOC_RC', 3),
    (4, 'LOC_OC_COVER', 18), (5, 'LOC_RC_COVER', 20),
])
def test_public_objects_use_native_custody_and_source_door_prerequisites(public, plate, source, destination):
    from bioxp.oem_deck_movement import OEM_PLATE_NAME_ORDINALS
    http, provider, store, frames, fault = public
    name = next(k for k, v in OEM_PLATE_NAME_ORDINALS.items() if v == plate)
    store.publish_deck_owner_state(source_operation='updatePlateLocation', source_command_id='fixture-object',
        updates={'movable_plate_locations': {name: source}}, **provider.deck_owner_authority_stamps())
    catch = submit(http, 'catch_plate', {'plate': plate}, 'native-object-catch')
    result = terminal(http, catch)
    if result['status'] != 'completed':
        import json, os
        from pathlib import Path
        if output := os.environ.get('BIOXP_MANUAL_PLATE_EXPORT'):
            Path(output + '.failure').write_text(json.dumps({'receipt': result, 'evidence': store.wp8_operation_evidence(catch), 'wire': frames}, indent=2))
    assert result['status'] == 'completed', (result['source_receipt'].get('terminal_evidence') or {}).get('response', {}).get('failed_child')
    assert store.deck_semantic_state()['plate_on_gantry'] == plate
    assert store.deck_semantic_state()['movable_plate_locations'][name] == 'LOC_GANTRY'
    release = submit(http, 'release_plate', {'destination': destination,
        'press_plate': False, 'run_in_parallel': False}, 'native-object-release')
    result = terminal(http, release)
    assert result['status'] == 'completed', result['source_receipt'].get('terminal_evidence')
    assert store.deck_semantic_state()['plate_on_gantry'] is None
    operations = [c['operation'] for c in store.wp8_operation_evidence(catch)['children']]
    assert ('doorOpen' in operations) is (plate in {4, 5})


def test_omitted_release_defaults_are_native_not_a_new_wrapper(public):
    http, provider, store, frames, fault = public
    catch = submit(http, 'catch_plate', {'plate': 1}, 'defaults-catch')
    assert terminal(http, catch)['status'] == 'completed'
    release = submit(http, 'release_plate', {'destination': 21}, 'defaults-release')
    assert terminal(http, release)['status'] == 'completed'
    operations = [c['operation'] for c in store.wp8_operation_evidence(release)['children']]
    assert 'LockGripperOperation' in operations
    assert 'moveZPress' not in operations
    assert store.get_command(release)['requested_inputs'] == {'destination': 21}
