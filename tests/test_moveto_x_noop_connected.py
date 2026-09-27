"""Real moveTo/scriptmoveTo and cover custody; only native transport replaced."""
import pytest

from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_authority import retained_rig


@pytest.fixture
def exact_native(connected, monkeypatch):
    rig = connected
    original = rig.native.motor_oem_move_absolute
    rig.noops = []

    def move(board, target, *, motor, wait_for_stop, max_position=None):
        if (board, motor) in ((5, 0), (4, 0)) and rig.native.positions[board, motor] == target:
            rig.noops.append((board, motor, target))
            return {'ok': True, 'source_noop': True,
                    'short_circuit': 'current_position_equals_target',
                    'requested_position': target, 'source_return_code': target,
                    'command_sent': False,
                    'ack': None, 'before': rig.native.motor_get_position(board, motor=motor)}
        return original(board, target, motor=motor, wait_for_stop=wait_for_stop,
                        max_position=max_position)

    monkeypatch.setattr(rig.native, 'motor_oem_move_absolute', move)
    return rig


@pytest.mark.parametrize('parallel', [False, True])
@pytest.mark.parametrize('before,target', [
    ((84252, 36267), (84252, 6057)),
    ((84252, 6057), (84252, 36267)),
    ((84252, 36267), (84252, 50000)),
    ((84252, 36267), (42788, 36267)),
    ((42788, 36267), (84252, 36267)),
    ((84252, 36267), (84252, 36267)),
])
def test_real_moveto_mixed_and_all_noops(exact_native, before, target, parallel):
    rig = exact_native
    rig.native.positions.update({(5, 0): before[0], (4, 0): before[1], (4, 1): 500})
    result = rig.provider.primitives.oem_move_to(*target, 500, pseudo_home_steps=500,
        gripper_confirmed=False, tip_loaded=False, run_in_parallel=parallel)
    assert result['ok'] is True
    assert result['source_return_code'] == 0
    assert result['controller_completion_verified'] is True
    moved = before != target
    assert result['controller_command_acknowledged'] is moved
    assert result['source_noop'] is (not moved)
    x = next(row for row in result['operations'] if row.get('axis') == 'x')
    assert x['source_mode'] == 'ClassControlInterface.moveTo.moveX'
    if before[0] == target[0]:
        assert x['source_noop'] is True
        assert x['source_call_completed'] is True
        assert x['source_return_code'] == target[0]
        assert x['command_issued'] is False
        assert x['controller_command_acknowledged'] is False
        assert rig.provider.primitives._oem_controller_child_evidence(x) == {
            'command_required': False, 'acknowledged': False, 'terminal': True}
    if before[0] == target[0]:
        assert not any(e[:2] == ('move', 'x') for e in rig.native.events)


def test_missing_noop_readback_proof_preserves_source_success(exact_native, monkeypatch):
    rig = exact_native
    rig.native.positions.update({(5, 0): 84252, (4, 0): 36267, (4, 1): 500})
    original = rig.native.motor_oem_move_absolute

    def missing_proof(board, target, **kwargs):
        result = original(board, target, **kwargs)
        if board == 5 and result.get('source_noop') is True:
            result['before']['ack'] = None
        return result

    monkeypatch.setattr(rig.native, 'motor_oem_move_absolute', missing_proof)
    result = rig.provider.primitives.oem_move_to(84252, 6057, 500,
        pseudo_home_steps=500, gripper_confirmed=False, tip_loaded=False,
        run_in_parallel=False)
    assert result['ok'] is True
    assert result['source_return_code'] == 0
    assert result['controller_completion_verified'] is False
    x = result['operations'][0]
    assert x['source_call_completed'] is True
    assert x['source_return_code'] == 84252
    assert x['physical_effect_verified'] is False
    assert rig.provider.primitives._oem_controller_child_evidence(x) == {
        'command_required': False, 'acknowledged': False, 'terminal': False}
    assert rig.native.positions[4, 0] == 6057


@pytest.mark.parametrize('failure', ['native', 'stop'])
def test_x_noop_does_not_hide_y_failure_or_stop(exact_native, failure):
    rig = exact_native
    rig.native.positions.update({(5, 0): 84252, (4, 0): 36267, (4, 1): 500})
    kwargs = dict(pseudo_home_steps=500, gripper_confirmed=False,
                  tip_loaded=False, run_in_parallel=False)
    if failure == 'native':
        rig.native.fail_at = ('y', 6057)
        with pytest.raises(RuntimeError, match='injected_native_transfer_failure'):
            rig.provider.primitives.oem_move_to(84252, 6057, 500, **kwargs)
    else:
        kwargs['interrupt_reason'] = lambda: 'operator_stop' if rig.noops else None
        result = rig.provider.primitives.oem_move_to(84252, 6057, 500, **kwargs)
        assert result['ok'] is False
        assert result['failure'] == 'operator_stop'
        assert result['interrupted_before_stage'] == 'descending_sequential_y'
        assert not rig.native.events
    assert rig.native.positions[4, 0] == 36267


def test_both_cover_round_trips_use_real_scriptmoveto(exact_native, monkeypatch):
    rig = exact_native
    raw = []
    original = rig.provider.primitives.oem_move_to

    def observe(*args, **kwargs):
        result = original(*args, **kwargs)
        raw.append(result)
        return result

    monkeypatch.setattr(rig.provider.primitives, 'oem_move_to', observe)
    for index, (plate, destination, name) in enumerate([
        (4, 18, 'LOC_OC_COVER_STORAGE'), (5, 20, 'LOC_RC_COVER_STORAGE'),
        (4, 17, 'LOC_OC_COVER'), (5, 19, 'LOC_RC_COVER'),
    ]):
        if index == 2:
            # Replay the live pre-pickup gripper predicate at the transport
            # boundary: not home and position >= 50 selects moveTo.moveX,
            # rather than the confirmed-gripper moveXY path.
            rig.native.home = False
            rig.native.positions[4, 2] = 100
        result = rig.provider._wp8_compile_and_execute(operation='move_plate',
            inputs={'plate': plate, 'destination': destination,
                    'press_plate': False, 'run_in_parallel': True},
            command_id=f'offline-x-noop-roundtrip-{index}', owner_identity=rig.owner)
        assert result['ok'] is True
        for task in rig.provider._wp8_tasks.values():
            task['thread'].join(timeout=2)
            assert task['state'] == 'completed'
        state = rig.store.deck_semantic_state()
        assert state['plate_on_gantry'] is None
        assert state['movable_plate_locations'][{4: 'OUTPUT_COVER', 5: 'REAGENT_COVER'}[plate]] == name
        assert all(child['result']['ok'] is True for child in result['source_children'])
    mixed = [r for r in raw if r.get('before', {}).get('x') == r['target']['x'] == 84252
             and r['before']['y'] == 36267 and r['target']['y'] == 6057]
    assert mixed
    assert all(r['controller_completion_verified'] is True for r in mixed)
    assert any(any(op.get('source_mode') == 'ClassControlInterface.moveTo.moveX'
                   and op.get('source_noop') is True for op in r['operations']) for r in mixed)
    _, positions = rig.provider._wp8_calibration()
    for xy in [(84252, 6057), (84252, 36267), (1324, 42129), (42788, 44972)]:
        assert any(e[:3] == ('move', 'g', positions['open']) and e[3:] == xy
                   for e in rig.native.events)
