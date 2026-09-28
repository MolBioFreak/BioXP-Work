"""Record-only history cannot substitute for or block consumed OEM inputs."""
import copy

import pytest

from tests.test_deck_scoped_authority import rig, retained_rig
from tests.test_cover_carry_release_connected import connected


@pytest.mark.parametrize('history', ['ambiguous', 'missing', 'mismatched'])
def test_unbound_offset_move_uses_only_consumed_facts(rig, history):
    provider, primitive, semantic, state = rig
    semantic.update(semantic_state_revision=4, ambiguity_state='ambiguous',
                    producer_command_id=None, producer_operation=None)
    semantic['transition_provenance'] = (None if history == 'missing' else
        {'source_operation': 'old', 'command_id': 'old'})
    if history != 'ambiguous':
        semantic['ambiguity_state'] = 'none'
    before = copy.deepcopy(semantic)
    result = provider.moveTo(location_id=1)
    assert result['ok'] is True
    assert primitive.calls[-1][0] == 'move'
    assert primitive.calls[-1][2]['pseudo_home_steps'] == 65000
    assert semantic == before
    assert semantic['current_location'] is None


@pytest.mark.parametrize('revision', [0, 4])
def test_full_projection_keeps_history_missing_and_binds_actual_owner(rig, revision):
    provider, primitive, semantic, state = rig
    semantic.update(current_location='LOC_OC', current_well=0, tip_loaded=False,
        tip_dirty=False, tip_location=-1, clean_path=False, plate_on_gantry=None,
        semantic_state_revision=revision, ambiguity_state='ambiguous', transition_provenance=None)
    before = copy.deepcopy(semantic)
    full = provider._canonical_deck_semantic_state()
    assert full['ownership_generation'] is None
    assert full['transition_provenance'] is None
    assert full['ambiguity_state'] == 'ambiguous'
    assert full['latch_observation_id'] is None
    native = provider.mov_execution_machine_state()
    assert {key: native[key] for key in provider.deck_owner_authority_stamps()} == provider.deck_owner_authority_stamps()
    assert semantic == before
    semantic['tip_dirty'] = None
    with pytest.raises(RuntimeError, match='tip_dirty'):
        provider._canonical_deck_semantic_state()


@pytest.mark.parametrize('axis', ['Y', 'Z'])
def test_axis_leaf_reaches_real_adapter_without_full_semantics(connected, axis):
    provider = connected.provider
    if axis == 'Y':
        def unused():
            pytest.fail('Y must not read any deck semantic input')
        provider.bind_deck_semantic_state_reader(unused)
    else:
        provider.bind_deck_semantic_state_reader(lambda: {
            'semantic_state_revision': 3, 'ambiguity_state': 'ambiguous',
            'pseudo_z_home': 65000, 'transition_provenance': None})
    result = getattr(provider, 'move' + axis)(70000)
    assert result['ok'] is True, result
    assert connected.native.moves


def test_consumed_state_and_current_owner_changes_still_refuse(rig):
    provider, primitive, semantic, state = rig
    authority = provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    semantic['pseudo_z_home'] = 500
    with pytest.raises(RuntimeError, match='semantic_authority_changed'):
        provider.assert_deck_observation_current(authority)
    semantic['pseudo_z_home'] = 65000
    state['x_lifecycle']['board_lifecycle_generation'] = 10
    with pytest.raises(RuntimeError, match='owner_authority_changed'):
        provider.assert_deck_observation_current(authority)
    assert not any(row[0] == 'move' for row in primitive.calls)
