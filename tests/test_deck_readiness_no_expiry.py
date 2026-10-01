"""Deck readiness stays valid until robot state changes, not for 15 seconds."""
import time

import pytest

from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider


def _provider(sampled_at):
    provider = Serial206OemInitializationProvider.__new__(Serial206OemInitializationProvider)
    epoch = object()
    provider._deck_dependency_scope = lambda target: "offset.v1"
    provider._deck_authority_cache_epoch = epoch
    snapshot = {"ownership_generation": 3, "current_location_id": "LOC_HOME"}
    provider._deck_authority_scoped_cache = {"offset.v1": (sampled_at, epoch, snapshot)}
    return provider, snapshot


def test_an_old_sample_is_still_served():
    provider, snapshot = _provider(time.monotonic() - 3600)
    assert provider.deck_authority_cached_snapshot(expected_generation=3, target="LOC_OC") == snapshot


def test_invalidation_and_generation_change_still_expire_the_sample():
    provider, _ = _provider(time.monotonic())
    with pytest.raises(RuntimeError, match="ownership_generation_changed"):
        provider.deck_authority_cached_snapshot(expected_generation=4, target="LOC_OC")
    provider._deck_authority_cache_epoch = object()  # what invalidate_deck_authority_cache does
    with pytest.raises(RuntimeError, match="deck_authority_cache_unavailable"):
        provider.deck_authority_cached_snapshot(expected_generation=3, target="LOC_OC")
