"""Fresh source lifecycle callers consume the admitted semantic vocabulary."""
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_tip_query_publication import query_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig


def test_fresh_deck_semantics_maps_actual_canonical_location_and_well(integrated_rig):
    provider = integrated_rig.provider
    canonical = provider._canonical_deck_semantic_state()
    actual = provider._deck_execution_semantics(None)
    assert actual['current_location_id'] == canonical['current_location']
    assert actual['current_well_id'] == canonical['current_well']
    assert 'current_location' not in actual and 'current_well' not in actual
    assert actual['pseudo_z_home'] == canonical['pseudo_z_home']
    assert actual['transition_provenance_digest'] == canonical['transition_provenance_digest']
