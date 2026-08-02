from fastapi.testclient import TestClient

from src.bioxp import oem_runtime_api
from src.bioxp.api import app


def test_runtime_api_status_and_command_enqueue(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    status = client.get("/oem/runtime/status")
    assert status.status_code == 200
    assert status.json()["available"] is False
    assert status.json()["cache_state"] == "missing"
    resp = client.post("/oem/runtime/commands/initializeSystem", json={"mode": "dry_run", "params": {"run_homing": False}})
    assert resp.status_code == 200
    assert resp.json()["queued"] is True


def test_runtime_configure_does_not_call_startup_factory_until_command_needs_it(tmp_path):
    calls = []

    def factory():
        calls.append("called")
        raise AssertionError("factory must be lazy at API startup")

    oem_runtime_api.configure_runtime(startup_program_factory=factory, store_root=str(tmp_path), autostart=False)
    assert calls == []


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


def test_runtime_api_door_event_does_not_auto_queue_initialize_system(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    resp = client.post("/oem/runtime/events/door", json={"door_closed": True, "latch_closed": True})
    assert resp.status_code == 200
    assert "queued_initializeSystem" not in resp.json()["actions_taken"]


def test_runtime_api_exposes_exact_command_terminal_result_by_id(tmp_path):
    runtime = oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)
    queued = client.post(
        "/oem/runtime/commands/initializeSystem",
        json={"mode": "dry_run", "params": {"run_homing": False}},
    ).json()
    command_id = queued["command"]["command_id"]

    waiting = client.get(f"/oem/runtime/commands/{command_id}")
    assert waiting.status_code == 200
    assert waiting.json()["state"] == "queued"

    assert runtime["worker"].run_next_for_tests()["ok"] is True
    terminal = client.get(f"/oem/runtime/commands/{command_id}")

    assert terminal.status_code == 200
    assert terminal.json()["state"] == "terminal"
    assert terminal.json()["terminal"]["ok"] is True


def test_runtime_api_exact_command_result_does_not_guess_unknown_ids(tmp_path):
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    response = TestClient(app).get("/oem/runtime/commands/not-a-real-command")

    assert response.status_code == 404
