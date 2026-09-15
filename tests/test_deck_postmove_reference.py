"""Named post-move fences with the real primitive and SQLite reference owner."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.oem_deck_movement import make_deck_command_executor, DeckExecutionFailure
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.services.reference_service import (
    ReferenceStateStore, MarkAxisReferencedCommand, MarkAxisDesyncedCommand,
)


class USBLeaf:
    """Controlled board responses only: no primitive/finalizer/reference stubs."""
    def __init__(self):
        self.positions = {(5, 0): 0, (4, 0): 0, (4, 1): 0}
        self.moves = []

    def _motion_oem_axis_profile(self, axis, startup=True):
        board, motor = {'x': (5, 0), 'y': (4, 0), 'z': (4, 1)}[axis]
        return {'board': board, 'motor': motor, 'axis_min_steps': 0,
                'axis_max_steps': {'x': 90263, 'y': 102956, 'z': 120000}[axis]}

    def _machine_config_axis_max(self, axis):
        return self._motion_oem_axis_profile(axis)['axis_max_steps'], 'fixture'

    def _oem_board_present(self, board):
        return True

    def _oem_board_state(self):
        return {4: True, 5: True}

    def oem_no24v_state(self):
        return False

    def motor_get_position(self, board, motor=0):
        value = self.positions[board, motor]
        return {'ok': True, 'position': value, 'ack': {'status': 100, 'value': value}}

    def motor_get_speed(self, board, motor=0):
        return {'ok': True, 'speed': 0, 'ack': {'status': 100, 'value': 0}}

    def motor_set_axis_param(self, *args, **kwargs):
        return {'ok': True, 'ack': {'status': 100}}

    def begin_bus_event_window(self):
        return {'after_sequence': 0, 'receive_owner': 'offline-usb', 'owner_generation': 3}

    def motor_oem_move_absolute(self, board, target, motor=0, **kwargs):
        before = self.motor_get_position(board, motor)
        self.moves.append((board, motor, target))
        self.positions[board, motor] = target
        return {'ok': True, 'command_sent': True, 'ack': {'status': 100},
                'before': before, 'event_window': self.begin_bus_event_window()}

    def motor_wait_target_reached_many(self, axes, **kwargs):
        assert kwargs['sta_sequential'] is True
        return {'ok': True, 'per_axis': {
            axis: {'ok': True, 'target_reached': True, 'event': {'source': 'novo_router_async',
                'latch_disposition': 'consumed', 'board': board, 'motor': 0,
                'status': 128, 'event_sequence': index + 1,
                'receive_owner': 'offline-usb', 'owner_generation': 3}}
            for index, (axis, board) in enumerate((('x', 5), ('y', 4)))}}

    def collect_bus_events(self, **kwargs):
        return []


@pytest.mark.parametrize('invalidation', [None, 'desync', 'rereference', 'generation', 'semantic'])
def test_real_named_primitive_postmove_fence(retained_rig, monkeypatch, invalidation):
    provider, observations, runtime, references, store, root = retained_rig
    qualify_test_references(references)
    versions = {axis: row['state_version'] for axis, row in references.snapshot(('x', 'y'))['rows'].items()}
    leaf = USBLeaf()
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None,
        authority_provider=lambda: {}, generation_provider=lambda: 3, reference_store=references)
    raw = []
    def real_move(*args, **kwargs):
        result = adapter.oem_move_to(*args, **kwargs)
        raw.append(result)
        if invalidation == 'desync':
            references.mark_desynced(MarkAxisDesyncedCommand('x', reason='offline independent invalidation'))
        elif invalidation == 'rereference':
            references.mark_referenced(MarkAxisReferencedCommand('x', 0, source='independent replacement'))
        elif invalidation == 'generation':
            monkeypatch.setattr(provider, 'generation_provider', lambda: 4)
        elif invalidation == 'semantic':
            state = provider._load_state()
            state['machine_status']['tip_loaded'] = True
            provider._save_state(state)
        return result
    monkeypatch.setattr(observations, 'oem_move_to', real_move)
    stamps = provider.deck_owner_authority_stamps()
    epochs = {'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
    request = dict(schema_version='bioxp.operator_action_request.v2', action_id='oem.deck.move_to_location',
        expected_ownership_generation=3, expected_board_epoch_by_board=epochs,
        idempotency_key='real-primitive-postmove', inputs={'target': 'LOC_OC', 'camera_offset': False})
    admitted = store.admit_command(request, state={'ownership_generation': 3,
        'serial206_initialization_provider': {'x_authority': {'current_board_lifecycle_generation': epochs['5']},
        'board4_authority': {'active_board_epoch': epochs['4']}}}, assessment={'enabled': True})
    claimed = store.claim_next()
    execute = make_deck_command_executor(provider_getter=lambda: provider,
        position_table_provider=load_bound_oem_position_table, command_store=store)
    try:
        result = execute(command_id=admitted['command_id'], target='LOC_OC', camera_offset=False,
            expected_ownership_generation=3, expected_board_epoch_by_board=epochs)
    except DeckExecutionFailure as exc:
        assert raw[-1]['branch'] == 'confirmed_gripper_no_tip_moveXY'
        assert raw[-1]['source_return_code'] == 0
        cause = str(exc.__cause__)
        chain = []
        current = exc
        while current is not None:
            chain.append({'type': type(current).__name__, 'message': str(current)})
            current = current.__cause__
        if os.environ.get('DECK_TEST_OUTPUT'):
            Path(os.environ['DECK_TEST_OUTPUT'] + '.' + str(invalidation) + '.json').write_text(json.dumps(
                {'cause': cause, 'chain': chain, 'provider_results': exc.provider_results,
                 'raw': raw, 'moves': leaf.moves, 'references': references.snapshot(('x', 'y', 'z', 'g'))}, indent=2))
        assert exc.controller_completion_verified is True
        assert exc.delivery_attempted is True
        assert exc.controller_command_acknowledged is True
        assert exc.provider_results[-1]['controller_completion_verified'] is True
        assert invalidation is not None, cause
        assert store.deck_semantic_state()['current_location'] is None
        assert store.deck_semantic_state()['pseudo_z_home'] == 500
        assert cause.endswith('_changed')
        assert not isinstance(exc.__cause__, DeckExecutionFailure)
        return
    assert invalidation is None, 'independent authority change was not fenced'
    assert result['ok'] and result['semantic_state_committed'] and result['controller_completion_verified']
    assert result['physical_effect_verified'] is False
    assert leaf.positions == {(5, 0): 26213, (4, 0): 42413, (4, 1): 0}
    assert set(leaf.moves) == {(5, 0, 26213), (4, 0, 42413)}  # no pseudo500 Z descent
    assert raw[-1]['branch'] == 'confirmed_gripper_no_tip_moveXY'
    for axis in ('x', 'y'):
        row = references.snapshot(('x', 'y', 'z', 'g'))['rows'][axis]
        assert row['last_motion_kind'] == 'move_xy' and row['state_version'] == versions[axis]
    store.finish(admitted['command_id'], status='completed', payload=result, claimed=claimed,
        controller_acknowledged=True, full_response=result)
    semantic = store.deck_semantic_state()
    assert semantic['current_location'] == 'LOC_OC' and semantic['current_well'] == 0
    assert semantic['tip_dirty'] is None
    script = ('import json,sys; from bioxp.operator_command_plane import OperatorCommandStore; '
              's=OperatorCommandStore(sys.argv[1]); print(json.dumps(s.deck_semantic_state())); s.stop()')
    assert json.loads(subprocess.check_output([sys.executable, '-c', script, str(root)], text=True)) == semantic


def test_reference_metadata_preserves_authority_not_unknown_promotion(retained_rig):
    _, _, _, store, _, root = retained_rig
    first = store.mark_referenced(MarkAxisReferencedCommand('x', 0, source='offline home'))
    assert first['ok']
    moved = store.record_motion('x', 'absolute')
    assert moved['ok'] and moved['last_motion_kind'] == 'absolute'
    assert moved['state_version'] == first['state_version']
    replaced = store.mark_referenced(MarkAxisReferencedCommand('x', 0, source='offline new home'))
    assert replaced['state_version'] > moved['state_version']
    desynced = store.mark_desynced(MarkAxisDesyncedCommand('x', reason='offline failure'))
    assert desynced['state_version'] > replaced['state_version']
    assert store.record_motion('x', 'absolute')['state'] == 'desynced'
    assert store.record_motion('unused_test_axis', 'absolute')['state'] == 'unknown'
    assert ReferenceStateStore(root / 'bioxp_runtime.db').snapshot(('x', 'y', 'z', 'g'))['rows']['x']['state'] == 'desynced'
