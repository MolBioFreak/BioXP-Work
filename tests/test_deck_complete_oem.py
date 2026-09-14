"""All finite OEM branches, offline native primitives and controller leaves only."""
import json
import os
from pathlib import Path
import pytest
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_near_terminal import NearUSB, named_rig
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.oem_deck_catalog import DeckCatalog, public_target_keys

def run_named(rig, execute, target, key):
    provider, _, _, _, store, _ = rig
    stamps = provider.deck_owner_authority_stamps()
    epochs = {'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
    request = dict(schema_version='bioxp.operator_action_request.v2',
        action_id='oem.deck.move_to_location', expected_ownership_generation=3,
        expected_board_epoch_by_board=epochs, idempotency_key=key,
        inputs={'target': target, 'camera_offset': False})
    admitted = store.admit_command(request, state={'ownership_generation': 3,
        'serial206_initialization_provider': {'x_authority': {'current_board_lifecycle_generation': epochs['5']},
        'board4_authority': {'active_board_epoch': epochs['4']}}}, assessment={'enabled': True})
    claimed = store.claim_next()
    assert claimed['command_id'] == admitted['command_id']
    # The real dispatcher renews while it owns queued/running work. This
    # synchronous test helper must not silently let a 23-command sequence
    # outlive that same unchanged lease without its normal owner heartbeat.
    assert store._renew_owner()
    result = execute(command_id=admitted['command_id'], target=target, camera_offset=False,
        expected_ownership_generation=3, expected_board_epoch_by_board=epochs)
    if not result['ok']:
        return result
    store.finish(admitted['command_id'], status='completed', payload=result, claimed=claimed,
        controller_acknowledged=result['controller_command_acknowledged'], full_response=result)
    assert store.deck_semantic_state()['current_location'] == DeckCatalog.from_position_table(load_bound_oem_position_table()).resolve(target).location_name
    assert result['semantic_state_committed']
    assert result['physical_effect_verified'] is False
    return result



TARGETS = sorted(public_target_keys() - {'LOC_PARK'})


def export(target, row):
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.' + target + '.json').write_text(json.dumps(row, indent=2))


def expected(table, destination):
    item = table.resolve(location_id=destination.location_name)
    camera = table.resolve(location_id='CAMERA_OFFSET')
    ox = oy = 0
    z = 0
    if destination.branch == 'barcode':
        ox = {2: -11847, 3: -23930}[destination.location_id] + camera.base_coordinates['x']
        oy = 7582 + camera.base_coordinates['y']
        z = int(camera.z_low - 1350.5511600000034) if destination.location_id == 2 else camera.z_low
    requested = {'x': item.base_coordinates['x'] + ox, 'y': item.base_coordinates['y'] + oy, 'z': z}
    wire = {**requested, 'x': 90213 if requested['x'] > 90263 else requested['x'],
            'y': 102906 if requested['y'] > 102956 else requested['y']}
    return requested, wire


@pytest.mark.parametrize('target', ['LOC_TC_BARCODE', 'LOC_RC_BARCODE'])
def test_barcode_missing_z_completion_retains_xy_not_success(retained_rig, monkeypatch, target):
    from bioxp.oem_deck_movement import DeckExecutionFailure
    leaf, raw, execute = named_rig(retained_rig, monkeypatch, (0, 0))
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: 3, reference_store=retained_rig[3])
    def z_fault(*args, **kwargs):
        leaf.fault = 'missing_event'  # only controller completion leaf changes
        return adapter.oem_move_z(*args, **kwargs)
    monkeypatch.setattr(retained_rig[1], 'oem_move_z', z_fault, raising=False)
    try:
        result = run_named(retained_rig, execute, target, 'failed-z-' + target)
    except DeckExecutionFailure as error:
        assert any(r.get('controller_completion_verified') is True for r in error.provider_results)
    else:
        assert not result['ok'] and not result['semantic_state_committed']
    assert raw[-1]['controller_completion_verified'] is True
    assert retained_rig[4].deck_semantic_state()['current_location'] is None
    assert any(motor == 1 for board, motor, target in leaf.moves)


@pytest.mark.parametrize('target', sorted(public_target_keys() - {'LOC_PARK', 'LOC_TC_BARCODE', 'LOC_RC_BARCODE'}))
def test_ordinary_camera_checkbox_native(retained_rig, monkeypatch, target):
    provider, observations, _, refs, _, _ = retained_rig
    leaf, raw, _ = named_rig(retained_rig, monkeypatch, (1000, 1000))
    provider.force_to_high_home()
    authority = provider.deck_authority_snapshot(expected_generation=3, target=target)
    result = provider.moveTo(location_id=DeckCatalog.from_position_table(load_bound_oem_position_table()).resolve(target).location_id,
        camera_offset=True, authority_snapshot=authority)
    table = load_bound_oem_position_table()
    row = table.resolve(location_id=target)
    camera = table.resolve(location_id='CAMERA_OFFSET')
    x = row.base_coordinates['x'] + camera.base_coordinates['x']
    y = row.base_coordinates['y'] + camera.base_coordinates['y']
    export(target + '-camera-probe', {'x': x, 'y': y, 'moves': leaf.moves, 'result': result})
    if y < 0:
        # Preserve the OEM lower clamp and qualify its actual effective target.
        # Raw caller intent is retained; no source dispatch or limits change.
        assert target == 'LOC_OC_COVER_STORAGE' and y == -1687
        assert result['ok'] and result['controller_completion_verified']
        assert result['raw_requested_y_steps'] == result['oem_requested_y_steps'] == y
        assert result['oem_effective_y_steps'] == leaf.positions[(4, 0)] == 0
        export(target + '-camera', {'requested': {'x': x, 'y': y, 'z': 0},
            'wire_expected': {'x': x, 'y': 0, 'z': 0},
            'qualification': 'offline native provider; OEM lower-clamped target verified', 'result': result})
        return
    assert result['ok'] and result['controller_completion_verified']
    assert leaf.positions == {(5, 0): 90213 if x > 90263 else x,
        (4, 0): 102906 if y > 102956 else y, (4, 1): 0}
    export(target + '-camera', {'requested': {'x': x, 'y': y, 'z': 0},
        'wire_expected': {'x': leaf.positions[(5, 0)], 'y': leaf.positions[(4, 0)], 'z': 0},
        'qualification': 'offline real provider, not canonical admission', 'result': result})


@pytest.mark.parametrize('target', TARGETS)
def test_all25_named_native_terminal(retained_rig, monkeypatch, target):
    leaf, raw, execute = named_rig(retained_rig, monkeypatch, (0, 0))
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: 3, reference_store=retained_rig[3])
    monkeypatch.setattr(retained_rig[1], 'oem_move_z', adapter.oem_move_z, raising=False)
    table = load_bound_oem_position_table()
    destination = DeckCatalog.from_position_table(table).resolve(target)
    requested, wire = expected(table, destination)
    result = run_named(retained_rig, execute, target, 'all26-' + target)
    assert result['ok'] and result['controller_completion_verified']
    assert leaf.positions == {(5, 0): wire['x'], (4, 0): wire['y'], (4, 1): wire['z']}
    assert result['physical_effect_verified'] is False
    export(target, {'requested': requested, 'wire_expected': wire, 'moves': leaf.moves,
        'native': raw, 'result': result, 'qualification': 'offline native + leaf doubles; not physical'})


def test_park_native_source_z_and_already_parked(retained_rig, monkeypatch):
    provider, observations, runtime, refs, store, _ = retained_rig
    qualify_test_references(refs)
    # Existing source fixture pattern: validate the isolated bundle with its
    # synthetic label; never remove blockers or access physical hardware.
    from bioxp import oem_machine_bundle
    snapshot = oem_machine_bundle.get_active_oem_machine_snapshot()
    snapshot = oem_machine_bundle.load_oem_machine_snapshot(
        snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json', operator_label_serial=206,
        require_operator_label=True)
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot', snapshot)
    class ParkUSB(NearUSB):
        def motor_wait_target_reached_many(self, axes, **kwargs):
            assert kwargs['sta_sequential'] is False  # native script context, not btnLOC
            return {'ok': True, 'per_axis': {a: self.motor_wait_target_reached(b)
                for a, b in [('x', 5), ('y', 4)]}}
    leaf = ParkUSB((26213, 42413))
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: 3, reference_store=refs)
    monkeypatch.setattr(observations, 'oem_initialize_motion_scriptmove_to_waste',
        adapter.oem_initialize_motion_scriptmove_to_waste, raising=False)
    # Legacy full sealed input, checked by real provider. Not admission evidence.
    authority = dict(tip_loaded=False, tip_dirty=False, tip_location=-1, clean_path=False,
        pseudo_z_home=500, ownership_generation=3, board_epoch_4=7, board_epoch_5=9,
        current_location_id='LOC_OC', current_well_id=0, machine_state_revision=1,
        semantic_state_provenance_digest='a'*64, plate_on_gantry=None, gripper_confirmed=True)
    result = provider.parkGantry(authority_snapshot=authority)
    table = load_bound_oem_position_table()
    target = table.resolve(location_id='LOC_PARK')
    assert result['ok'] and result['controller_completion_verified']
    assert leaf.positions == {(5, 0): target.base_coordinates['x'], (4, 0): target.base_coordinates['y'], (4, 1): target.z_low}
    assert target.z_low == 114092
    before = list(leaf.moves)
    noop = provider.parkGantry(authority_snapshot={**authority, 'current_location_id': 'LOC_PARK'})
    assert noop['source_noop'] and not noop['delivery_attempted']
    assert not noop['controller_command_acknowledged'] and not noop['controller_completion_verified']
    assert leaf.moves == before
    export('LOC_PARK', {'requested': dict(target.base_coordinates), 'wire_expected':
        {'x': target.base_coordinates['x'], 'y': target.base_coordinates['y'], 'z': target.z_low},
        'moves': leaf.moves, 'result': result, 'noop': noop,
        'qualification': 'offline native provider with full sealed fixture, NOT canonical admission or physical'})
