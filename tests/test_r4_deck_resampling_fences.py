"""Receipt identity is distinct from resampled state/owner identity."""
from dataclasses import replace
import pytest
from bioxp.oem_deck_movement import DeckAuthoritySnapshot
import bioxp.oem_deck_movement as movement
from test_r4_deck_collection_integration import installed, ready
from test_r4_named_destination_vectors import rig


@pytest.mark.parametrize('field,value', [
    ('ownership_generation', 8), ('provider_owner_id', 'new-owner'),
    ('board_epoch_4', 99), ('board_epoch_5', 99),
    ('position_table_sha256', '0'*64), ('machine_state_revision', 999),
    ('reference_versions', {'x': 9, 'y': 2, 'z': 3, 'g': 4}),
    ('safety_epochs', {'global': 1, 'x': 0, 'y': 0, 'z': 0}),
    ('controller_position_observation_id', 'new-position'),
    ('current_x', 101), ('current_y', 201), ('current_z', 64000),
    ('current_location_id', 'LOC_OC'), ('current_well_id', 1),
    ('tip_loaded', False), ('tip_dirty', True), ('tip_location', 0),
    ('clean_path', True), ('pseudo_z_home', 500), ('latch_status', False),
    ('machine_latch_closed', False), ('semantic_state_provenance_digest', '0'*64),
    ('latch_observation_id', 'unknown-changed-latch-id'),
])
def test_resampling_never_waives_state_or_owner_drift(installed, field, value):
    _, provider, *_ = installed
    ready(installed)
    before = DeckAuthoritySnapshot(**provider.deck_authority_snapshot(expected_generation=7))
    after = DeckAuthoritySnapshot(**provider.deck_authority_snapshot(expected_generation=7))
    assert before.digest != after.digest
    assert movement._same_authority_after_resampling(before, after)
    assert not movement._same_authority_after_resampling(before, replace(after, **{field: value}))


def test_resampling_preserves_host_latch_identity_and_rejects_backward_time(installed):
    _, provider, *_ = installed
    ready(installed)
    before = DeckAuthoritySnapshot(**provider.deck_authority_snapshot(expected_generation=7))
    after = DeckAuthoritySnapshot(**provider.deck_authority_snapshot(expected_generation=7))
    assert before.latch_observation_id != after.latch_observation_id
    assert movement._same_authority_after_resampling(before, after)
    assert not movement._same_authority_after_resampling(before, replace(after, captured_at=before.captured_at-1))
    changed_host = after.latch_observation_id.replace('host=fake-host-latch:0:1', 'host=fake-host-latch:1:1')
    assert not movement._same_authority_after_resampling(before, replace(after, latch_observation_id=changed_host))
