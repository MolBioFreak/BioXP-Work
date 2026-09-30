"""Optional tip publication cannot veto the real Park cleanup query."""
import pytest
from tests.test_manual_park_context_connected import park
from tests.test_deck_tip_query_publication import query_rig, query
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig


def test_cleanup_query_uses_tip_owner_without_semantic_publication(park, query_rig, monkeypatch):
    _, provider, primitive, refs, root, receipts, calls, wire, owner = query_rig
    wire['data'] = [[32, 96, 49] for _ in range(4)]
    query(query_rig, key='loaded-park-predecessor')
    assert provider._park_collection_state()['tip_exists'] is True
    assert provider._deck_semantic_state_reader()['tip_loaded'] is None
    authority = dict(park.authority, collection_tip_state=provider._park_collection_state(),
        tip_loaded=None, tip_dirty=None, tip_location=None, clean_path=None)
    park.native.positions.update({(5, 0): 1000, (4, 0): 1000})
    def eject():
        # Inert mechanical exchange changes the bytes returned by the next
        # existing ?31 query. No source state, receipt, or publication is faked.
        wire['data'] = [[32, 96, 48] for _ in range(4)]
        return {'ok': True}
    monkeypatch.setattr(primitive, 'eject_all_tips_for_oem_park', eject, raising=False)
    monkeypatch.setattr(primitive, 'oem_move_to', park.adapter.oem_move_to)
    monkeypatch.setattr(primitive, 'oem_initialize_motion_move_absolute',
        park.adapter.oem_initialize_motion_move_absolute, raising=False)
    def query_existing(**kwargs):
        result = query(query_rig, key='park-cleanup-existing-query')
        assert result['deck_state_publication']['status'] == 'blocked'
        return result
    monkeypatch.setattr(primitive, 'query_all_pipette_tip_states', query_existing, raising=False)
    def unavailable_publication(**kwargs):
        raise RuntimeError('isolated optional publication unavailable')
    monkeypatch.setattr(provider, '_deck_semantic_state_publisher', unavailable_publication)
    result = provider.parkGantry(authority_snapshot=authority)
    assert result['ok'], result
    assert provider._park_collection_state()['tip_exists'] is False
    assert provider._deck_semantic_state_reader()['tip_loaded'] is None
    assert len(calls) == 12  # setup query, loaded query, existing cleanup query
    assert any(row['operation'] == 'queryTipStatus(-1)' for row in result['source_children'])


def test_park_consumes_current_tip_value_not_previous_query_identity(park, query_rig):
    previous = dict(park.authority['collection_tip_state'])
    query(query_rig, key='new-query-same-tip-value')
    current = park.provider._park_collection_state()
    assert current['identity'] != previous['identity']
    assert current['tip_exists'] is previous['tip_exists'] is False
    result = park.provider.parkGantry(authority_snapshot=park.authority)
    assert result['ok'], result
