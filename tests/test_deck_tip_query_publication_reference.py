"""No OEM motor-reference reset from pipette recording failure."""
from tests.test_deck_tip_query_publication import query_rig, query, named_move
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig


def test_pipette_publication_failure_does_not_reset_references(query_rig, monkeypatch):
    app, provider, primitive, references, root, receipts, calls, wire, transport = query_rig
    named_move(query_rig)
    query(query_rig)
    before = references.snapshot(('x', 'y', 'z', 'g'))['rows']
    attempted = []
    def reject_publication(**kwargs):
        attempted.append(kwargs)
        raise RuntimeError('isolated_publication_failure')
    monkeypatch.setattr(provider, 'publish_pipette_owner_state', reject_publication)
    wire['data'][0] = [32, 96, 49]
    result = query(query_rig, 'no-nonoem-reference-reset')
    assert attempted and attempted[0]['tip_loaded'] is True
    assert result['deck_state_publication'] == {'status': 'blocked', 'reason': 'isolated_publication_failure'}
    assert result['semantic_query_response_verified'] is True
    assert references.snapshot(('x', 'y', 'z', 'g'))['rows'] == before
