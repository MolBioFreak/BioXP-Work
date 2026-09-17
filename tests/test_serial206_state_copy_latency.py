"""Offline provider-state ownership regressions; no robot/API singleton."""
import copy
import threading
from concurrent.futures import ThreadPoolExecutor

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider


@pytest.fixture(autouse=True)
def isolated_runtime_root(monkeypatch, tmp_path):
    monkeypatch.delenv("BIOXP_OEM_RUNTIME_ROOT", raising=False)
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))


def test_state_roundtrip_only_copies_at_ownership_boundaries(monkeypatch):
    provider = Serial206OemInitializationProvider(object())
    state = provider._new_state()
    original = copy.deepcopy
    copies = []
    def counted(value, *args, **kwargs):
        if isinstance(value, dict) and value.get("schema_version") == state["schema_version"]:
            copies.append(True)
        return original(value, *args, **kwargs)
    monkeypatch.setattr(copy, "deepcopy", counted)
    provider._save_state(state)
    assert len(copies) == 2  # caller -> upgraded state -> retained memory
    copies.clear()
    provider._load_state()
    assert len(copies) == 1  # retained memory -> caller


@pytest.mark.parametrize("durable", [False, True])
def test_populated_state_remains_detached_without_duplicate_copies(tmp_path, durable):
    store = OEMRuntimeStore(tmp_path) if durable else None
    provider = Serial206OemInitializationProvider(object(), state_store=store)
    state = provider._new_state()
    for axis in ("x", "z"):
        state[f"{axis}_lifecycle"]["receipts"] = [{
            "command_id": f"{axis}-completed", "status": "completed",
            "controller_terminal_state_verified": True,
            "physical_effect_verified": False,
            "result": {"ok": True, "terminal_state": {"position_steps": 0}},
        }]
    original = copy.deepcopy(state)
    try:
        saved = provider._save_state(state)
        assert state == original
        state["machine_status"]["constructed_tip_trays"][0]["occupancy"][0] = False
        saved["machine_status"]["constructed_tip_trays"][0]["occupancy"][1] = False
        loaded = provider._load_state()
        assert loaded["machine_status"]["constructed_tip_trays"][0]["occupancy"] == [True] * 96
        loaded["z_lifecycle"]["receipts"][0]["result"]["ok"] = False
        assert provider._load_state()["z_lifecycle"]["receipts"][0]["result"]["ok"] is True
        if store:
            store.close()
            store = OEMRuntimeStore(tmp_path)
            provider = Serial206OemInitializationProvider(object(), state_store=store)
        assert provider._load_state()["x_lifecycle"]["receipts"][0]["physical_effect_verified"] is False
        assert provider._load_state()["z_lifecycle"]["receipts"][0]["controller_terminal_state_verified"] is True
    finally:
        if store:
            store.close()


def test_warm_projection_contention_preserves_new_authority(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    provider = Serial206OemInitializationProvider(object(), state_store=store)
    provider._save_state(provider._new_state())
    entered, release = threading.Event(), threading.Event()
    def poll():
        with provider.projection_scope():
            provider.initialize_motion_projection()
            entered.set()
            assert release.wait(5)
            provider.z_projection()
            provider.x_projection()
    def writer():
        with provider._lock:
            state = provider._load_state()
            state["machine_status"]["tip_loaded"] = True
            provider._save_state(state)
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            reader = pool.submit(poll)
            assert entered.wait(5)
            write = pool.submit(writer)
            release.set()
            reader.result(5)
            write.result(5)
        assert provider._load_state()["machine_status"]["tip_loaded"] is True
    finally:
        release.set()
        store.close()


@pytest.mark.parametrize("invalid", ["schema", "nan"])
def test_copy_removal_does_not_skip_validation(invalid):
    provider = Serial206OemInitializationProvider(object())
    state = provider._new_state()
    if invalid == "schema":
        state["schema_version"] = "invalid"
    else:
        state["machine_status"]["bad_number"] = float("nan")
    with pytest.raises(ValueError):
        provider._save_state(state)
    assert provider._memory_state is None
