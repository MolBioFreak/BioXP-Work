"""Location publication through the production provider, inert transport only."""
import socket
import pytest
from tests.test_wake_setup_debloat import rig
from tests.test_debloat_core import named


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('network forbidden')
    monkeypatch.setattr(socket.socket, 'connect', forbidden)


def test_provider_location_after_presence_only_tip_publication(named):
    _, provider, store, _, frames, _ = named
    store.publish_deck_owner_state(source_operation='pipette_owner',
        source_command_id='lane2-presence-only',
        updates={'tip_loaded': True, 'tip_dirty': None, 'tip_location': None},
        **provider.deck_owner_authority_stamps())
    before = store.deck_semantic_state()
    # Existing execution fallback is separate from canonical observations.
    resolved = provider._canonical_deck_semantic_state()
    assert resolved['tip_loaded'] is True
    frames.clear()
    for child, (destination, well) in enumerate([(0, 4), (3, 12)]):
        result = provider.wp8_update_location('updateLocation',
            {'destination': destination, 'well': well}, command_id='lane2-well',
            child_order=child, plan_digest='lane2-offline')
        assert result['ok'] is True
        assert result['delivery_attempted'] is False
        assert result['published']['current_well'] == well
        for field in ('tip_loaded', 'tip_dirty', 'tip_location'):
            assert result['published'][field] == before[field]
    assert not frames  # This semantic update is not an OEM motor/tip query.
