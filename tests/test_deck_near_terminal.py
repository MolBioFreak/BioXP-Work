"""OFFLINE actual adapter/Y provider -> named executor -> retained SQLite."""
import json
import os
from pathlib import Path

import pytest

from tests.test_deck_postmove_reference import USBLeaf
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.serial206_y_provider import Serial206YProvider
from bioxp.oem_deck_movement import make_deck_command_executor, DeckExecutionFailure
from bioxp.oem_compat.position_table import load_bound_oem_position_table


class NearUSB(USBLeaf):
    def __init__(self, before, fault=None):
        super().__init__()
        self.positions.update({(5, 0): before[0], (4, 0): before[1]})
        self.fault = fault

    def _motion_oem_axis_profile(self, axis, startup=True):
        return {**super()._motion_oem_axis_profile(axis, startup), 'speed': 1800,
                'acc': 400, 'run_current': 31, 'stall_guard': 16, 'disable_right': True}

    def motor_oem_axis_board_present(self, axis):
        return self.fault != 'missing_' + axis

    def motor_get_position(self, board, motor=0):
        row = super().motor_get_position(board, motor)
        if self.fault == 'invalid_position':
            row.update(ok=False, ack={'status': 2, 'value': row['position']})
        return row

    def motor_oem_move_absolute(self, board, target, motor=0, **kwargs):
        row = super().motor_oem_move_absolute(board, target, motor, **kwargs)
        if self.fault == 'invalid_child':
            return None
        wait = self.motor_wait_target_reached(board, motor)
        row.update(oem_wait_for_stop=kwargs.get('wait_for_stop', True), wait=wait,
                   completion_class='event_128' if wait['ok'] else 'timeout',
                   terminal_position=self.motor_get_position(board, motor), source_return_code=0)
        if self.fault == 'failed_child':
            row['ok'] = False
        if self.fault == 'missing_ack':
            row['ack'] = None
        return row

    def motor_wait_target_reached(self, board, motor=0, **kwargs):
        accepted = self.fault not in {'ack_only', 'stale', 'missing_event'}
        event = {'source': 'novo_router_async', 'latch_disposition': 'consumed',
                 'board': board, 'motor': motor, 'status': 128, 'event_sequence': 1,
                 'receive_owner': 'offline-usb', 'owner_generation': 3}
        if self.fault == 'stale':
            event['owner_generation'] = 2
        return {'ok': accepted, 'target_reached': accepted,
                'event': event if self.fault != 'missing_event' else None}

    def motor_wait_target_reached_many(self, axes, **kwargs):
        assert kwargs['sta_sequential'] is True
        return {'ok': True, 'per_axis': {a: self.motor_wait_target_reached(b)
                                        for a, b in [('x', 5), ('y', 4)]}}


def named_rig(rig, monkeypatch, before, fault=None):
    provider, observations, runtime, references, store, root = rig
    qualify_test_references(references)
    leaf = NearUSB(before, fault)
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None,
        authority_provider=lambda: {}, generation_provider=lambda: 3, reference_store=references)
    adapter.y_provider = Serial206YProvider(leaf, state_store=runtime,
        generation_provider=lambda: 3, reference_store=references)
    if fault == 'unavailable':
        adapter.y_provider = None
    raw = []
    def move(*args, **kwargs):
        result = adapter.oem_move_to(*args, **kwargs)
        raw.append(result)
        return result
    monkeypatch.setattr(observations, 'oem_move_to', move)
    execute = make_deck_command_executor(provider_getter=lambda: provider,
        position_table_provider=load_bound_oem_position_table, command_store=store)
    return leaf, raw, execute


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
    assert store.deck_semantic_state()['current_location'] == target
    assert result['semantic_state_committed']
    assert result['physical_effect_verified'] is False
    return result


@pytest.mark.parametrize('before,branch,moved', [
    ((25029, 71755), 'near_axis_sequential', [5]),
    ((60571, 42413), 'near_axis_sequential', [4]),
    ((60561, 71745), 'near_axis_sequential', [5, 4]),
    ((60571, 71755), 'source_noop', []),
    ((0, 0), 'parallel', [4, 5]),
])
def test_named_xy_completion(retained_rig, monkeypatch, before, branch, moved):
    leaf, raw, execute = named_rig(retained_rig, monkeypatch, before)
    result = run_named(retained_rig, execute, 'TECANRACK1', 'near-success')
    assert result['ok'], result
    assert result['controller_completion_verified'] is bool(moved)
    assert raw[-1]['operations'][0]['branch'] == branch
    assert sorted(b for b, m, t in leaf.moves) == sorted(moved)
    assert leaf.positions == {(5, 0): 60571, (4, 0): 71755, (4, 1): 0}
    if not moved:
        operation = raw[-1]['operations'][0]
        assert operation['controller_command_acknowledged'] is False
        assert operation['target_event_128_observed'] is False
        assert operation['physical_motion_commanded'] is False
        assert result['controller_command_acknowledged'] is False


@pytest.mark.parametrize('before', [(25029, 71755), (60571, 42413), (60561, 71745), (0, 0)])
@pytest.mark.parametrize('fault', ['failed_child', 'invalid_child', 'ack_only', 'missing_ack',
                                  'missing_event', 'stale', 'missing_x', 'missing_y', 'unavailable'])
def test_named_xy_missing_proof_fails_closed(retained_rig, monkeypatch, before, fault):
    if fault == 'unavailable' and before == (0, 0):
        # Parallel deliberately does not depend on the public near-Y provider.
        leaf, raw, execute = named_rig(retained_rig, monkeypatch, before, fault)
        assert run_named(retained_rig, execute, 'TECANRACK1', 'parallel-no-provider')['ok']
        assert raw[-1]['operations'][0]['branch'] == 'parallel'
        return
    leaf, raw, execute = named_rig(retained_rig, monkeypatch, before, fault)
    try:
        result = run_named(retained_rig, execute, 'TECANRACK1', 'near-failure')
    except DeckExecutionFailure:
        pass
    else:
        assert result['ok'] is False and result['semantic_state_committed'] is False
    assert retained_rig[4].deck_semantic_state()['current_location'] is None
    assert leaf.positions[(4, 1)] == 0
    if raw:
        assert raw[-1]['controller_completion_verified'] is False or raw[-1]['ok'] is False


def test_invalid_xy_noop_readback_fails_closed(retained_rig, monkeypatch):
    _, _, execute = named_rig(retained_rig, monkeypatch, (60571, 71755), 'invalid_position')
    with pytest.raises(DeckExecutionFailure):
        run_named(retained_rig, execute, 'TECANRACK1', 'invalid-noop')
    assert retained_rig[4].deck_semantic_state()['current_location'] is None


def test_selected23_high_clearance_sequence(retained_rig, monkeypatch):
    fixture = json.loads(Path(__file__).with_name('deck_near_terminal_targets.json').read_text())
    assert len(fixture['roster']) == 26
    assert set(fixture['excluded']) == {'LOC_PARK', 'LOC_TC_BARCODE', 'LOC_RC_BARCODE'}
    selected = fixture['selected']
    assert len(selected) == 23
    assert {r['target'] for r in selected} == set(fixture['roster']) - set(fixture['excluded'])
    leaf, raw, execute = named_rig(retained_rig, monkeypatch, (0, 0))
    table = load_bound_oem_position_table()
    completed = []
    for index, row in enumerate(selected):
        target = row['target']
        calibration = table.resolve(location_id=target)
        assert dict(calibration.coordinates) == row['calibrated']
        expected = calibration.oem_offset_move_coordinates(offset_x=0, offset_y=0,
            x_high_limit=90263, y_high_limit=102956)
        # This named high-clearance branch requests pseudo500 but leaves Z0;
        # the table helper's default Z65000 is not a move or observed position.
        expected = {'x': expected['x'], 'y': expected['y'], 'z': 0}
        prior = len(leaf.moves)
        result = run_named(retained_rig, execute, target, 'offline-sequence-' + str(index))
        assert result['ok'], (target, result)
        assert leaf.positions[(5, 0)] == expected['x']
        assert leaf.positions[(4, 0)] == expected['y']
        assert leaf.positions[(4, 1)] == 0
        assert all(motor == 0 for board, motor, position in leaf.moves)
        completed.append({'target': target, 'expected': expected,
            'branch': raw[-1]['operations'][0]['branch'], 'moves': leaf.moves[prior:],
            'semantic_state': retained_rig[4].deck_semantic_state(), 'result': result,
            'receipt': retained_rig[4].command_detail_v2(
                retained_rig[4].deck_semantic_state()['producer_command_id'])})
        assert completed[-1]['receipt']['status'] == 'completed'
    assert len(completed) == 23
    assert {'near_axis_sequential', 'parallel', 'source_noop'} <= {r['branch'] for r in completed}
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.selected23.json').write_text(json.dumps(
            {'qualification': 'OFFLINE USB doubles, not physical acceptance', 'completed': completed}, indent=2))
