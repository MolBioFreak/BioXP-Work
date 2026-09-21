"""Close-scoped primitive Z regressions; physical calls are offline doubles."""
import copy

import pytest

from tests.test_deck_scoped_authority import rig  # noqa: F401


def bind_z(primitive, *, ok=True):
    def move(target, **kwargs):
        primitive.calls.append(('z', target, kwargs))
        return {'ok': ok, 'controller_command_acknowledged': ok,
                'controller_completion_verified': ok}
    primitive.oem_move_z = move


@pytest.mark.parametrize('pseudo', [500, 65000])
@pytest.mark.parametrize('owner', ['canonical', 'retained'])
def test_close_z_consumes_only_real_pseudo_home(rig, pseudo, owner):
    provider, primitive, semantic, state = rig
    bind_z(primitive)
    semantic.update(semantic_state_revision=1,
                    transition_provenance={'source_operation': 'sourceImageGantryLoad', 'command_id': 'close-z'},
                    producer_operation='sourceImageGantryLoad', producer_command_id='close-z',
                    pseudo_z_home=pseudo if owner == 'canonical' else None)
    state['machine_status']['tip_loaded'] = None
    state['machine_status']['psudo_z_home_steps'] = pseudo if owner == 'retained' else None
    before = copy.deepcopy((semantic, state))
    result = provider.wp8_source_move_z('pipette_move_z', {'value': 42000})
    assert result['ok'] is True
    assert result['controller_completion_verified'] is True
    assert primitive.calls == [('z', 42000, {'pseudo_home_steps': pseudo,
        'motor_current': 31, 'wait_for_stop': True})]
    assert (semantic, state) == before
    assert semantic['clean_path'] is None
    assert semantic['tip_loaded'] is None


@pytest.mark.parametrize('pseudo', [None, True, '500', 501])
def test_close_z_missing_or_invalid_consumed_pseudo_fails_before_delivery(rig, pseudo):
    provider, primitive, semantic, state = rig
    bind_z(primitive)
    semantic['pseudo_z_home'] = pseudo
    state['machine_status']['psudo_z_home_steps'] = None
    before = copy.deepcopy((semantic, state))
    with pytest.raises(RuntimeError, match='deck_semantic_state_not_authoritative:pseudo_z_home'):
        provider.wp8_source_move_z('pipette_move_z', {'value': 42000})
    assert primitive.calls == []
    assert (semantic, state) == before


@pytest.mark.parametrize('changes,reason', [
    ({'ambiguity_state': 'ambiguous'}, 'ambiguity'),
    ({'semantic_state_revision': -1}, 'location_revision'),
    ({'transition_provenance': None}, 'provenance'),
    ({'semantic_state_revision': 1}, 'producer_provenance'),
])
def test_close_z_existing_semantic_fences_still_fail(rig, changes, reason):
    provider, primitive, semantic, state = rig
    bind_z(primitive)
    semantic.update(changes)
    with pytest.raises(RuntimeError, match='deck_semantic_state_not_authoritative:' + reason):
        provider.wp8_source_move_z('pipette_move_z', {'value': 42000})
    assert primitive.calls == []


def test_close_z_failure_receipt_not_promoted(rig):
    provider, primitive, semantic, state = rig
    bind_z(primitive, ok=False)
    result = provider.wp8_source_move_z('pipette_move_z', {'value': 42000})
    assert result['ok'] is False
    assert result['controller_completion_verified'] is False
    assert semantic['clean_path'] is None


def test_close_offset_scope_still_requires_tip(rig):
    provider, primitive, semantic, state = rig
    state['machine_status']['tip_loaded'] = None
    with pytest.raises(RuntimeError, match='deck_semantic_state_not_authoritative:tip_loaded'):
        provider._offset_deck_semantic_state(gripper_confirmed=True)


def test_close_y_scope_is_unchanged(rig, monkeypatch):
    provider, primitive, semantic, state = rig
    def full_reader():
        raise RuntimeError('close-full-reader-sentinel')
    monkeypatch.setattr(provider, 'mov_execution_machine_state', full_reader)
    with pytest.raises(RuntimeError, match='close-full-reader-sentinel'):
        provider.moveY(42000)
    assert primitive.calls == []


@pytest.mark.parametrize('door', [False, True, None])
def test_close_thermal_plan_consumes_only_source_door_state(rig, monkeypatch, door):
    from bioxp.oem_deck_movement import compile_finite_plate_operation
    provider, primitive, semantic, state = rig
    state['machine_status']['thermal_door_open'] = door
    before = copy.deepcopy((semantic, state))
    def full_reader():
        raise AssertionError('unrelated full-deck read')
    monkeypatch.setattr(provider, 'mov_execution_machine_state', full_reader)
    selected = provider.wp8_operation_machine_state('thermal_door', {'open': True})
    assert selected['door_is_open'] is door
    if door is None:
        with pytest.raises(RuntimeError, match='thermal_door_state'):
            compile_finite_plate_operation('thermal_door', source_leaf_available=True,
                                           open=True, **selected)
    else:
        plan = compile_finite_plate_operation('thermal_door', source_leaf_available=True,
                                              open=True, **selected)
        assert bool(plan.get('source_noop')) is door
        if not door:
            assert plan['children'][0]['operation'] == 'parkGantry'
    assert (semantic, state) == before
    assert primitive.calls == []


@pytest.mark.parametrize('operation', ['park_gantry', 'critical_item_images'])
def test_close_literal_lifecycle_plan_defers_child_state_reads(rig, monkeypatch, operation):
    provider, primitive, semantic, state = rig
    before = copy.deepcopy((semantic, state))
    def full_reader():
        raise AssertionError('unrelated full-deck read')
    monkeypatch.setattr(provider, 'mov_execution_machine_state', full_reader)
    assert provider.wp8_operation_machine_state(operation, {}) == {}
    assert (semantic, state) == before
    assert primitive.calls == []
