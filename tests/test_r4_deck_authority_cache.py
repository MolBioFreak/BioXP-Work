"""Passive cache is fed by actual provider authority collection, not GETs."""
import time
import pytest
from test_r4_deck_producer_refresh import owners, complete_predecessor
from test_r4_named_destination_vectors import rig
from test_oem_deck_install_binding import FakeReferenceStore


def observed(owners):
    provider, runtime, store = owners
    complete_predecessor(runtime)
    provider.reference_store = FakeReferenceStore()
    provider.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='fixture-constructor',
        command_id='fixture-construct', provenance={'source':'ClassTipTray..ctor','kind':'host_semantic_default'})
    snapshot = provider.deck_authority_snapshot(expected_generation=7)
    assert snapshot['current_x'] == 100 and snapshot['current_y'] == 200
    return provider, snapshot


def test_cache_absent_is_explicit_not_fabricated(owners):
    with pytest.raises(RuntimeError, match='deck_authority_cache_unavailable'):
        owners[0].deck_authority_cached_snapshot(expected_generation=7)


def test_cache_reads_no_hardware_sqlite_or_provider_lock_and_preserves_age(owners):
    provider, snapshot = observed(owners)
    class Forbidden:
        def __getattr__(self, key):
            raise AssertionError('passive read touched '+key)
        def __enter__(self):
            raise AssertionError('passive read took provider lock')
    provider.primitives = provider.state_store = provider.reference_store = provider._lock = Forbidden()
    provider.generation_provider = lambda: (_ for _ in ()).throw(AssertionError('passive read called owner'))
    start = time.monotonic()
    cached = provider.deck_authority_cached_snapshot(expected_generation=7)
    assert time.monotonic() - start < 1.0
    assert cached == snapshot
    cached['reference_versions']['x'] = 999
    assert provider.deck_authority_cached_snapshot(expected_generation=7) == snapshot


def test_cache_generation_expiry_and_invalidation_are_closed(owners):
    provider, snapshot = observed(owners)
    with pytest.raises(RuntimeError, match='ownership_generation_changed'):
        provider.deck_authority_cached_snapshot(expected_generation=8)
    _, epoch, stored = provider._deck_authority_cache
    provider._deck_authority_cache = (time.monotonic()-15.0, epoch, stored)
    with pytest.raises(RuntimeError, match='deck_authority_cache_stale'):
        provider.deck_authority_cached_snapshot(expected_generation=7)
    provider.invalidate_deck_authority_cache(reason='fixture-board-epoch-changed')
    with pytest.raises(RuntimeError, match='deck_authority_cache_unavailable'):
        provider.deck_authority_cached_snapshot(expected_generation=7)


def test_source_owner_publication_invalidates_cache(owners):
    provider, _ = observed(owners)
    provider.publish_pipette_owner_state(tip_loaded=False, tip_dirty=False, tip_location=-1,
                                        source_command_id='fixture-tip-ejection')
    with pytest.raises(RuntimeError, match='deck_authority_cache_unavailable'):
        provider.deck_authority_cached_snapshot(expected_generation=7)


def test_failed_active_observation_does_not_leave_prior_cache_ready(owners):
    provider, _ = observed(owners)
    provider.reference_store = None
    with pytest.raises(RuntimeError, match='deck_reference_store_not_bound'):
        provider.deck_authority_snapshot(expected_generation=7)
    with pytest.raises(RuntimeError, match='deck_authority_cache_unavailable'):
        provider.deck_authority_cached_snapshot(expected_generation=7)


def test_slow_active_sample_does_not_renew_observation_age(owners, monkeypatch):
    provider, _ = observed(owners)
    clock = {'now': 100.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['now'])
    original = provider.primitives._read_axis_position
    def delayed(axis):
        clock['now'] = 116.0
        return original(axis)
    provider.primitives._read_axis_position = delayed
    provider.deck_authority_snapshot(expected_generation=7)
    with pytest.raises(RuntimeError, match='deck_authority_cache_stale'):
        provider.deck_authority_cached_snapshot(expected_generation=7)


def test_external_reference_changes_during_active_sample_cannot_warm_cache(owners):
    provider, _ = observed(owners)
    refs = provider.reference_store.snapshot(('x', 'y', 'z', 'g'))
    provider.reference_store.snapshot = lambda axes: refs
    original = provider.primitives._read_axis_position
    def changing(axis):
        refs['rows']['x']['state_version'] += 1
        return original(axis)
    provider.primitives._read_axis_position = changing
    with pytest.raises(RuntimeError, match='deck_authority_changed_during_observation'):
        provider.deck_authority_snapshot(expected_generation=7)
    with pytest.raises(RuntimeError, match='deck_authority_cache_unavailable'):
        provider.deck_authority_cached_snapshot(expected_generation=7)
