"""Canonical queued/store qualification of camera, barcode and full-state park."""
import json
import subprocess
import sys
import time
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_complete_effective import adapter_rig
from tests.test_deck_complete_oem import export


def seal_park_fixture(provider, store, monkeypatch, *, loaded=False):
    from bioxp import oem_machine_bundle
    from bioxp.oem_deck_movement import OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS
    snapshot = oem_machine_bundle.get_active_oem_machine_snapshot()
    snapshot = oem_machine_bundle.load_oem_machine_snapshot(snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json',
        operator_label_serial=206, require_operator_label=True)
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot', snapshot)
    state = provider._load_state()
    state['machine_status'].update(current_location='LOC_OC', current_well=0,
        tip_loaded=loaded, tip_dirty=False, tip_location=0 if loaded else -1, clean_path=False,
        plate_on_gantry=None, movable_plate_locations=dict(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS))
    state['machine_status'].pop('latch_observation_id', None)
    state['machine_status'].pop('latch_closed', None)
    provider._save_state(state)
    provider.bind_tip_tray_state_reader(store.tip_tray_state)
    store.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='fixture-only-construction',
        command_id='fixture-only-construction', provenance={'source': 'explicit test predecessor'},
        **provider.deck_owner_authority_stamps())
    snapshot = provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()), target='LOC_PARK')
    assert snapshot['current_location_id'] == 'LOC_OC' and snapshot['tip_loaded'] is loaded
    return snapshot


@pytest.mark.parametrize('start_y', [0, 1000])
@pytest.mark.parametrize('start_z', [0, 65000])
@pytest.mark.parametrize('target,camera,xyz', [
    ('LOC_OC_COVER_STORAGE', True, (87751, 0, 0)),
    ('LOC_TC_BARCODE', False, (59258, 9079, 1794)),
    ('LOC_RC_BARCODE', False, (47246, 45094, 3145)),
    ('LOC_PARK', False, (1506, 71, 114092)),
])
def test_canonical_special_and_camera(installed_retained, retained_rig, monkeypatch, target, camera, xyz, start_z, start_y):
    from bioxp import api
    app, provider, observations, refs, root = installed_retained
    store = app.state.operator_command_plane.store
    generation = int(provider.generation_provider())
    api.serial206_oem_initialization_provider_status()
    leaf, adapter, raw = adapter_rig((provider, observations, retained_rig[2], refs, store, root),
        monkeypatch, (1000, start_y))
    leaf.positions[(4, 1)] = start_z
    if camera and start_z > 500:
        xyz = (xyz[0], xyz[1], 500)
    # Receive ownership is the real installed fixture generation, not constant3.
    original_wait = leaf.motor_wait_target_reached
    def wait(*args, **kwargs):
        result = original_wait(*args, **kwargs)
        if result.get('event'):
            result['event']['owner_generation'] = generation
        return result
    monkeypatch.setattr(leaf, 'motor_wait_target_reached', wait)
    monkeypatch.setattr(leaf, 'motor_oem_wait_target_reached', wait)
    monkeypatch.setattr(leaf, 'begin_bus_event_window', lambda **kwargs: {
        'after_sequence': 0, 'receive_owner': 'offline-usb', 'owner_generation': generation})
    def many(axes, **kwargs):
        assert kwargs['sta_sequential'] is (target != 'LOC_PARK')
        return {'ok': True, 'per_axis': {a: wait(b) for a, b in [('x', 5), ('y', 4)]}}
    monkeypatch.setattr(leaf, 'motor_wait_target_reached_many', many)
    monkeypatch.setattr(adapter, 'generation_provider', lambda: generation)
    monkeypatch.setattr(observations, 'oem_move_z', adapter.oem_move_z, raising=False)
    monkeypatch.setattr(observations, 'oem_initialize_motion_scriptmove_to_waste',
        adapter.oem_initialize_motion_scriptmove_to_waste, raising=False)
    if target == 'LOC_PARK':
        seal_park_fixture(provider, store, monkeypatch)
    assert api._collect_and_publish_hardware_snapshot(['axes', 'latch'],
        reason='isolated-effective-target')['deck_authority']['enabled']
    action = catalog_action(app)
    option = next(row for row in action['destination_options'] if row['target'] == target)
    assert option['enabled'], option
    client = TestClient(app)
    body = {'schema_version': 'bioxp.operator_action_request.v2',
        'idempotency_key': 'canonical-effective-' + target,
        'expected_ownership_generation': generation,
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': target, 'camera_offset': camera}}
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert response.status_code == 200, response.text
    cid = response.json()['command_id']
    app.state.operator_command_plane.start()
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        compact = client.get('/operator/v2/actions/receipts/' + cid).json()
        if compact['status'] not in {'queued', 'dispatched', 'issued_pending'}:
            break
        time.sleep(.01)
    detail = client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
    export('canonical-' + target + '-z' + str(start_z) + '-y' + str(start_y), {'detail': detail, 'compact': compact, 'moves': leaf.moves,
        'native': raw, 'xy': adapter.xy_rows})
    assert detail['status'] == 'completed', json.dumps(detail, indent=2)
    assert detail['deck_movement']['semantic_state_committed']
    assert detail['deck_movement']['controller_completion_verified']
    assert leaf.positions == {(5, 0): xyz[0], (4, 0): xyz[1], (4, 1): xyz[2]}
    if camera:
        proof = detail['deck_movement']['stages'][3]['terminal_evidence']['provider_evidence']
        assert proof['raw_requested_x_steps'] == 87751
        assert proof['raw_requested_y_steps'] == -1687
        assert proof['oem_effective_x_steps'] == 87751
        assert proof['oem_effective_y_steps'] == 0
        if start_y == 0:
            xy = adapter.xy_rows[-1]
            assert xy['branch'] == 'parallel'
            assert xy['axis_evidence']['y']['source_noop_verified']
            assert not xy['commands']['y']['controller_command_acknowledged']
            assert not xy['waits']['y']['ok'] and xy['waits']['y']['event'] is None
            assert not any(b == 4 and m == 0 for b, m, t in leaf.moves)
    if start_z == 65000:
        assert (4, 1, 500) in leaf.moves
        assert not any(m == 1 and t == 65000 for b, m, t in leaf.moves)
    prior = list(leaf.moves)
    for _ in range(3):
        assert client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json() == detail
        assert catalog_action(app)
    replay = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert replay.status_code == 200 and replay.json()['command_id'] == cid
    assert leaf.moves == prior
    app.state.operator_command_plane.stop()
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
        'print(json.dumps(fresh_process_receipts(sys.argv[1],sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root), cid], text=True, timeout=12))
    assert reopened == {'compact': compact, 'detail': detail}
