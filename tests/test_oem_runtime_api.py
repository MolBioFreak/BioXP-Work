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


def test_runtime_api_accepts_typed_stage_name_and_binds_robot_owned_artifact_root(tmp_path, monkeypatch):
    captured = []

    class Worker:
        def enqueue(self, command):
            captured.append(command)
            return {"ok": True, "queued": True, "command_id": command.command_id}

    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setattr(oem_runtime_api, "_require_runtime", lambda: (object(), Worker(), object(), object()))
    req = oem_runtime_api.RuntimeCommandRequest(
        mode="live",
        operator_ack="HOME",
        artifact_root="/remote/client/must/not/control/this",
        params={"homing_step": "z-home"},
    )

    result = oem_runtime_api._enqueue("startupHomingStepwise", req)

    assert result["queued"] is True
    assert captured[0].name == "startupHomingStepwise"
    assert captured[0].params == {"homing_step": "z-home"}
    assert captured[0].artifact_root.startswith(str(tmp_path / "artifacts"))
    assert captured[0].artifact_root != req.artifact_root
