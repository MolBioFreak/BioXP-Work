"""Unrelated semantic publications do not fence current execution owners."""
import pytest
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_r1_constructor_owner_connected import prepare_constructor


def test_offset_reads_source_facts_without_host_revision_precondition(query_rig, monkeypatch):
    app, provider, primitive, references, root, receipts, calls, wire, owner = query_rig
    read = provider._deck_semantic_state_reader
    def legacy_projection():
        result = dict(read())  # actual SQLite projection, optional revision omitted
        result.pop('semantic_state_revision')
        return result
    monkeypatch.setattr(provider, '_deck_semantic_state_reader', legacy_projection)
    semantic = provider._offset_deck_semantic_state(gripper_confirmed=True)
    assert semantic['tip_loaded'] is False
    assert semantic['pseudo_z_home'] in (500, 65000)
    assert app.state.operator_command_plane.store.deck_semantic_state()['semantic_state_revision'] == 0
    assert calls == []


def test_same_fact_publication_does_not_change_offset_permission(query_rig):
    app, provider, primitive, references, root, receipts, calls, wire, owner = query_rig
    snapshot = provider.deck_authority_snapshot(expected_generation=provider.generation_provider(), target='LOC_OC')
    assert app.state.operator_command_plane.store.deck_semantic_state()['tip_loaded'] is None
    provider.publish_pipette_owner_state(tip_loaded=snapshot['tip_loaded'],
        tip_dirty=None, tip_location=None, source_command_id='actual-same-tip-state')
    provider.assert_deck_observation_current(snapshot)
    result = provider.moveTo(location_id=1, authority_snapshot=snapshot)
    assert result['ok'] is True
    assert any(row[0] == 'move' for row in primitive.calls)


def publish_unrelated(store, provider, key):
    return store.publish_deck_owner_state(source_operation='updatePlateLocation',
        source_command_id=key, updates={'movable_plate_locations': {'OUTPUT_PLATE': 'LOC_P_OC'}},
        **provider.deck_owner_authority_stamps())


@pytest.mark.parametrize('target', ['LOC_OC', 'LOC_PARK'])
def test_unrelated_publication_during_observation_is_not_refusal(query_rig, monkeypatch, target):
    from bioxp import api
    app, provider, primitive, references, root, receipts, calls, wire, owner = query_rig
    prepare_constructor(query_rig, monkeypatch)
    store = app.state.operator_command_plane.store
    # Park's location and well belong to the preceding ordinary named move.
    store.publish_deck_owner_state(source_operation='updateLocation', source_command_id='source-location',
        updates={'current_location': 'LOC_OC', 'current_well': 0}, **provider.deck_owner_authority_stamps())
    before = store.deck_semantic_state()
    original = primitive._read_axis_position
    def observe(axis):
        if axis == 'x':
            publish_unrelated(store, provider, 'unrelated-during-axis-read')
        return original(axis)
    monkeypatch.setattr(primitive, '_read_axis_position', observe)
    snapshot = provider.deck_authority_snapshot(expected_generation=provider.generation_provider(), target=target)
    after = store.deck_semantic_state()
    assert after['semantic_state_revision'] > snapshot['machine_state_revision']
    assert before['tip_loaded'] == after['tip_loaded'] is None
    assert snapshot['board_epoch_4'] == provider.deck_owner_authority_stamps()['board_epoch_4']
    assert calls == []


def test_unrelated_publication_after_observation_is_not_native_entry_refusal(query_rig):
    app, provider, primitive, references, root, receipts, calls, wire, owner = query_rig
    snapshot = provider.deck_authority_snapshot(expected_generation=provider.generation_provider(), target='LOC_OC')
    publish_unrelated(app.state.operator_command_plane.store, provider, 'unrelated-before-native-entry')
    result = provider.moveTo(location_id=1, authority_snapshot=snapshot)
    assert result['ok'] is True
    assert any(row[0] == 'move' for row in primitive.calls)
