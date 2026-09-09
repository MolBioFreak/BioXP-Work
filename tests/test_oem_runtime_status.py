import copy
import importlib

import pytest

from src.bioxp.oem_runtime_status import OEMRuntimeStatusService
from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.lifecycle_state import CanonicalLifecycleOwner


class Worker:
    def snapshot(self):
        return {
            "state": "idle",
            "gantry_available": True,
            "queue_depth": 0,
            "active_command": None,
            "last_heartbeat_at": 123.0,
        }


@pytest.fixture
def lifecycle_owner(monkeypatch):
    owner = CanonicalLifecycleOwner()
    module = importlib.import_module("src.bioxp.lifecycle_state")
    monkeypatch.setattr(module, "lifecycle_state", owner)
    return owner


@pytest.mark.parametrize("operation,ok", [
    ("stopped", True), ("paused", True), ("running", True),
    ("error", False), ("emergency", False),
])
def test_missing_persisted_state_uses_live_lifecycle_without_fabricating_snapshot(
    tmp_path, lifecycle_owner, operation, ok,
):
    lifecycle_owner.transition(operation, reason="offline status witness")
    store = OEMRuntimeStore(tmp_path)
    try:
        assert store.read_state() is None
        result = OEMRuntimeStatusService(store=store, worker=Worker()).status()
        assert result["cache_state"] == "missing"
        assert result["runtime_state"] == result["operation_state"] == operation
        assert result["ok"] is ok
        assert result["canonical_hardware_snapshot"] is None
        assert result["lifecycle"] == lifecycle_owner.projection()
        assert "canonical lifecycle remains authoritative" in result["error"]
        assert store.read_state() is None  # status cannot publish its projection
    finally:
        store.close()


class CachedStore(OEMRuntimeStore):
    def __init__(self, state):
        self.state = copy.deepcopy(state)
        self.reads = 0

    def read_state(self):
        self.reads += 1
        return copy.deepcopy(self.state)


def test_idle_worker_and_legacy_cache_cannot_override_live_error_or_create_hardware_proof(lifecycle_owner):
    lifecycle_owner.transition("error", reason="current source failure")
    original = {"ok": True, "runtime_state": "running", "machine_status": {}}
    store = CachedStore(original)
    result = OEMRuntimeStatusService(store=store, worker=Worker()).status()
    assert result["worker"] == Worker().snapshot()
    assert result["operation_state"] == result["runtime_state"] == "error"
    assert result["ok"] is False
    assert result["canonical_hardware_snapshot"] == {
        "snapshot_id": None, "ownership_epoch": None, "available": False,
        "reference_only": True,
        "error": "legacy persisted state has no canonical snapshot reference",
    }
    assert store.reads == 1
    assert store.state == original


def test_live_enclosure_and_latch_remain_distinct_from_cached_thermal_door_and_observation(lifecycle_owner):
    lifecycle_owner.transition("paused", reason="door witness", evidence={
        "door_closed": False, "latch_closed": True,
    })
    observation = {"snapshot_id": "retained-observation", "observed_at": "old", "available": False}
    original = {"canonical_hardware_snapshot": observation, "machine_status": {
        "enclosure_door_closed": True, "latch_closed": False,
        "thermal_door_open": True, "running_job": True,
    }}
    store = CachedStore(original)
    result = OEMRuntimeStatusService(store=store).status()
    state = result["machine_status"]
    assert state["enclosure_door_closed"] is False
    assert state["latch_closed"] is True
    assert state["thermal_door_open"] is True
    assert state["user_paused"] is True
    assert state["running_job"] is False
    assert result["canonical_hardware_snapshot"] == observation
    assert store.reads == 1
    assert store.state == original
