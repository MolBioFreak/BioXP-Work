"""Collection adapter contract; real R producer qualified by overlay run."""
import pytest
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action


def test_batch_collection_adapter_preserves_per_scope_failure(installed_retained, monkeypatch):
    from bioxp import api
    app, provider, primitive, references, root = installed_retained
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    real_batch = getattr(provider, 'deck_authority_snapshots', None)
    one = provider.deck_authority_snapshot
    # Base C alone has no R batch method. Preserve actual provider samples in
    # the seam fixture; the R-overlay run invokes the real batched producer.
    calls = []
    def batch(*, expected_generation, before_query=None):
        calls.append(expected_generation)
        if real_batch:
            return real_batch(expected_generation=expected_generation, before_query=before_query)
        result = {}
        for target in ('LOC_PARK', 'LOC_MS'):
            before_query()
            try:
                result[target] = one(expected_generation=expected_generation, target=target)
            except Exception as exc:
                result[target] = exc
        return result
    monkeypatch.setattr(provider, 'deck_authority_snapshots', batch, raising=False)
    result = app.state.oem_deck_authority_collector()
    assert len(calls) == 1
    options = {row['target']: row for row in result['destination_options']}
    assert result['enabled'] and options['LOC_MS']['enabled']
    assert not options['LOC_PARK']['enabled']
    assert 'deck_bootstrap_semantic_location_unavailable' in options['LOC_PARK']['disabled_reason']
    before = list(primitive.calls)
    assert catalog_action(app)['enabled']
    assert len(calls) == 1 and primitive.calls == before


def test_full_scopes_use_actual_batch_when_available(installed_retained, retained_rig):
    from tests.test_deck_scoped_integration import qualify_full_predecessor
    app, provider, primitive, references, root = installed_retained
    catalog_action(app)  # settle copied lifecycle before test reference events
    qualify_full_predecessor((provider, primitive, retained_rig[2], references,
        app.state.operator_command_plane.store, root))
    primitive.calls.clear()
    result = app.state.oem_deck_authority_collector()
    assert result['enabled']
    assert all(row['enabled'] for row in result['destination_options'])
    # C alone deliberately preserves old-provider compatibility. R overlay
    # exercises the actual plural producer through this exact collection owner.
    acquisitions = 1 if callable(getattr(provider, 'deck_authority_snapshots', None)) else 2
    assert sum(row[0] == 'xyz' for row in primitive.calls) == 3 * acquisitions
    assert sum(row[0] == 'latch' for row in primitive.calls) == acquisitions


@pytest.mark.parametrize('returned', [False, True])
def test_batch_preemption_never_becomes_scope_failure(installed_retained, monkeypatch, returned):
    from bioxp import api
    from bioxp.hardware_status import HardwareCollectionPreempted
    app, provider, primitive, references, root = installed_retained
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    app.state.oem_deck_authority_collector()
    epoch = provider._deck_authority_cache_epoch
    calls = list(primitive.calls)
    def batch(**kwargs):
        if returned:
            return {'LOC_PARK': HardwareCollectionPreempted('operator_action_pending')}
        raise HardwareCollectionPreempted('operator_action_pending')
    monkeypatch.setattr(provider, 'deck_authority_snapshots', batch, raising=False)
    result = app.state.oem_deck_authority_collector(yield_requested=lambda: False)
    assert result == {'enabled': False, 'disabled_reason': 'operator_action_pending'}
    assert primitive.calls == calls and provider._deck_authority_cache_epoch is epoch
