"""Receipt qualification: real board primitive, only controller transport doubles."""
import pytest
from tests.test_deck_near_terminal import NearUSB
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_complete_oem import export
from bioxp.usb_driver import BioXpTester
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.serial206_y_provider import Serial206YProvider


class BoardLeaf(NearUSB):
    BOARD_DECK = 5
    BOARD_HEAD = 4
    BOARD_THERMAL = 3
    motor_oem_move_absolute = BioXpTester.motor_oem_move_absolute
    _tmcl_success = staticmethod(lambda row: isinstance(row, dict) and row.get('status') == 100)

    def __init__(self, before, fault=None):
        super().__init__(before, fault)
        self.wait_calls = []
        self.board_results = []

    def motor_axis_key_for_channel(self, board, motor=0):
        return {(5, 0): 'x', (4, 0): 'y', (4, 1): 'z'}[board, motor]

    def begin_bus_event_window(self, **kwargs):
        return super().begin_bus_event_window()

    def motor_query_motor_stop(self, board, motor=0):
        return {'ok': True, 'wait_latch_reset': True}

    def _send_motor(self, board, command, typ, motor, target, **kwargs):
        assert command == 4 and typ == 0
        self.moves.append((board, motor, target))
        self.positions[board, motor] = target + (100 if self.fault == 'wrong_position' else 0)
        return {'status': 2 if self.fault == 'missing_ack' else 100}

    def motor_wait_target_reached(self, board, motor=0, **kwargs):
        self.wait_calls.append((board, motor))
        # No event may be manufactured for a board exact-noop.
        if not any(b == board and m == motor for b, m, t in self.moves):
            return {'ok': False, 'target_reached': False, 'event': None, 'failure': 'timeout'}
        return super().motor_wait_target_reached(board, motor, **kwargs)

    motor_oem_wait_target_reached = motor_wait_target_reached

    def motor_get_speed(self, board, motor=0):
        row = super().motor_get_speed(board, motor)
        if self.fault == 'moving':
            row['speed'] = 1
        return row


def adapter_rig(rig, monkeypatch, before, fault=None):
    provider, observations, runtime, refs, store, root = rig
    qualify_test_references(refs)
    leaf = BoardLeaf(before, fault)
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: 3, reference_store=refs)
    adapter.y_provider = Serial206YProvider(leaf, state_store=runtime,
        generation_provider=lambda: 3, reference_store=refs)
    raw = []
    xy_rows = []
    native_xy = adapter.move_xy
    def capture_xy(*args, **kwargs):
        result = native_xy(*args, **kwargs)
        xy_rows.append(result)
        return result
    monkeypatch.setattr(adapter, 'move_xy', capture_xy)
    adapter.xy_rows = xy_rows
    def native(*args, **kwargs):
        row = adapter.oem_move_to(*args, **kwargs)
        raw.append(row)
        return row
    monkeypatch.setattr(observations, 'oem_move_to', native)
    return leaf, adapter, raw


@pytest.mark.parametrize('before', [(1000, 1000), (1000, 0), (87751, 1000), (87751, 0)])
def test_camera_effective_native(retained_rig, monkeypatch, before):
    provider = retained_rig[0]
    leaf, adapter, raw = adapter_rig(retained_rig, monkeypatch, before)
    provider.force_to_high_home()
    authority = provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC_COVER_STORAGE')
    from bioxp.oem_deck_catalog import DeckCatalog
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    destination = DeckCatalog.from_position_table(load_bound_oem_position_table()).resolve('LOC_OC_COVER_STORAGE')
    result = provider.moveTo(location_id=destination.location_id, camera_offset=True, authority_snapshot=authority)
    export('camera-' + str(before), {'native': raw, 'result': result, 'moves': leaf.moves})
    assert result['ok']
    operation = adapter.xy_rows[-1]
    if before == (87751, 0):
        assert result['source_noop'] and not result['controller_command_acknowledged']
        assert operation['source_noop_verified'] and not leaf.moves
    else:
        assert result['controller_completion_verified']
    assert operation['requested'] == {'x': 87751, 'y': -1687}
    assert leaf.positions[(4, 0)] == 0
    if operation['branch'] == 'parallel':
        assert operation['effective_target'] == {'x': 87751, 'y': 0}
        assert operation['axis_evidence']['y']['position_verified']
        if before[1] == 0:
            y = operation['axis_evidence']['y']
            assert y['source_noop_verified'] and not y['target_event_128_observed']
            assert not operation['commands']['y']['controller_command_acknowledged']
            assert not operation['waits']['y']['ok']
            assert all(b != 4 for b, m, t in leaf.moves)


@pytest.mark.parametrize('axis', ['x', 'y'])
@pytest.mark.parametrize('start', [0, 1000])
@pytest.mark.parametrize('fault', [None, 'missing_event', 'missing_ack', 'wrong_position', 'moving', 'stale'])
def test_parallel_clamp_same_branch_proof(retained_rig, monkeypatch, axis, start, fault):
    before = (start, 1000) if axis == 'x' else (1000, start)
    requested = (-1687, 4000) if axis == 'x' else (4000, -1687)
    leaf, adapter, raw = adapter_rig(retained_rig, monkeypatch, before, fault)
    result = adapter.move_xy(*requested, wait_timeout_s=5, source_context='ClassControlInterface.btnLOC1_Click')
    export('parallel-' + axis + '-' + str(start) + '-' + str(fault), result)
    assert result['branch'] == 'parallel'
    assert result['requested'] == dict(zip(('x', 'y'), requested))
    if fault is None:
        assert result['controller_terminal_state_verified'] and result['target_position_verified']
        assert result['effective_target'][axis] == 0
        if start == 0:
            assert result['axis_evidence'][axis]['source_noop_verified']
            assert not result['axis_evidence'][axis]['target_event_128_observed']
            assert not result['waits'][axis]['ok']
    else:
        assert not (result['controller_terminal_state_verified'] and result['target_position_verified'] and result['controller_command_acknowledged'])


@pytest.mark.parametrize('before', [(1000, 4000), (60, 4000), (60, 3990)])
def test_near_x_lower60_is_not_parallel_lower0(retained_rig, monkeypatch, before):
    leaf, adapter, raw = adapter_rig(retained_rig, monkeypatch, before)
    result = adapter.move_xy(-1687, 4000, wait_timeout_s=5,
        source_context='ClassControlInterface.btnLOC1_Click')
    export('near-x-' + str(before), result)
    assert result['branch'] == 'near_axis_sequential'
    assert result['requested'] == {'x': -1687, 'y': 4000}
    assert result['effective_target'] == {'x': 60, 'y': 4000}
    assert result['controller_terminal_state_verified']
    assert leaf.positions[(5, 0)] == 60
    if before == (60, 4000):
        assert result['source_noop_verified'] and not leaf.moves
        assert not result['controller_command_acknowledged']
        assert adapter._oem_controller_child_evidence(result) == {
            'command_required': False, 'acknowledged': False, 'terminal': True}



def test_parallel_both_board_noops_keep_real_timeouts(retained_rig, monkeypatch):
    leaf, adapter, raw = adapter_rig(retained_rig, monkeypatch, (0, 0))
    result = adapter.move_xy(-1000, -1687, wait_timeout_s=5,
        source_context='ClassControlInterface.btnLOC1_Click')
    assert result['branch'] == 'parallel'
    assert result['requested'] == {'x': -1000, 'y': -1687}
    assert result['effective_target'] == {'x': 0, 'y': 0}
    assert result['source_noop_verified'] and not leaf.moves
    assert not result['controller_command_acknowledged']
    assert not result['target_event_128_observed']
    assert all(not row['ok'] and row['event'] is None for row in result['waits'].values())
    assert adapter._oem_controller_child_evidence(result) == {
        'command_required': False, 'acknowledged': False, 'terminal': True}
    export('both-board-noops', result)
