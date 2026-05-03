from fastapi.testclient import TestClient

from src.bioxp import oem_runtime_api
from src.bioxp.api import app


def test_runtime_api_status_and_command_enqueue(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    status = client.get("/oem/runtime/status")
    assert status.status_code == 200
    assert status.json()["truth_source"] == "oem_runtime_store"
    resp = client.post("/oem/runtime/commands/initializeSystem", json={"mode": "dry_run", "params": {"run_homing": False}})
    assert resp.status_code == 200
    assert resp.json()["queued"] is True


def test_runtime_api_rejects_live_without_ack(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    resp = client.post("/oem/runtime/commands/initializeSystem", json={"mode": "live", "artifact_root": "/tmp/bioxp-live-runs/test"})
    assert resp.status_code == 409


def test_runtime_api_rejects_unknown_generic_command_with_409(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    resp = client.post("/oem/runtime/commands/enqueue", json={"name": "homeEverything", "mode": "dry_run"})
    assert resp.status_code == 409
    assert "unknown OEM runtime command" in resp.json()["detail"]


def test_runtime_api_door_event_queues_initialize_system(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    resp = client.post("/oem/runtime/events/door", json={"door_closed": True, "latch_closed": True})
    assert resp.status_code == 200
    assert "queued_initializeSystem" in resp.json()["actions_taken"]
