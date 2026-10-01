"""Park consumes only ClassPipetteCollection.TipExist, never collection metadata."""
import time

import pytest

from bioxp.oem_deck_movement import _tip_exists
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider


def _provider(sampled, current):
    provider = Serial206OemInitializationProvider.__new__(Serial206OemInitializationProvider)
    epoch = object()
    provider._deck_dependency_scope = lambda target: "park.full"
    provider._deck_authority_cache_epoch = epoch
    snapshot = {"ownership_generation": 1, "current_location_id": "LOC_HOME",
                "collection_tip_state": sampled}
    provider._deck_authority_scoped_cache = {"park.full": (time.monotonic(), epoch, snapshot)}
    provider._park_collection_state = lambda: dict(current)
    return provider


def test_collection_metadata_drift_keeps_park_available():
    provider = _provider(
        {"tip_exists": False, "receipt_id": "query-1", "command_id": "c1", "identity": "a"},
        {"tip_exists": False, "receipt_id": "query-2", "command_id": "c2", "identity": "b"},
    )
    snapshot = provider.deck_authority_cached_snapshot(expected_generation=1, target="park")
    assert snapshot["current_location_id"] == "LOC_HOME"


def test_changed_tip_state_still_disables_park():
    provider = _provider({"tip_exists": False}, {"tip_exists": True})
    with pytest.raises(RuntimeError, match="pipette_collection_owner_changed_after_collection"):
        provider.deck_authority_cached_snapshot(expected_generation=1, target="park")


def test_tip_exists_reads_only_the_consumed_fact():
    assert _tip_exists({"tip_exists": True, "receipt_id": "x"}) is True
    assert _tip_exists({"tip_exists": False}) is False
    assert _tip_exists(None) is None
