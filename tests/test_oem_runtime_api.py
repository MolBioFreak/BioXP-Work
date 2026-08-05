from fastapi.testclient import TestClient

from src.bioxp import oem_runtime_api
from src.bioxp.api import app


def test_runtime_api_preserves_unrelated_abortjob_command(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    runtime = oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)

    queued = client.post("/oem/runtime/commands/abortjob", json={"mode": "dry_run", "params": {}})

    assert queued.status_code == 200
    assert queued.json()["queued"] is True
    terminal = runtime["worker"].run_next_for_tests()
    assert terminal["ok"] is True
    assert terminal["result"]["safe_action_taken"] == "would_execute_provider_owned_z_abort"


def test_runtime_api_rejects_unknown_generic_command_with_409(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    resp = client.post("/oem/runtime/commands/enqueue", json={"name": "homeEverything", "mode": "dry_run"})
    assert resp.status_code == 409
    assert "unknown OEM runtime command" in resp.json()["detail"]


def test_runtime_api_door_event_does_not_auto_queue_commands(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    resp = client.post("/oem/runtime/events/door", json={"door_closed": True, "latch_closed": True})
    assert resp.status_code == 200
    assert resp.json()["actions_taken"] == ["door_close_observed"]


def test_runtime_api_exact_command_result_does_not_guess_unknown_ids(tmp_path):
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    response = TestClient(app).get("/oem/runtime/commands/not-a-real-command")

    assert response.status_code == 404
