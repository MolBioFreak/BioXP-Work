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


def test_runtime_status_overlays_stale_shutdown_with_canonical_lifecycle(tmp_path, monkeypatch):
    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transition("waiting", reason="test_waiting")
    monkeypatch.setattr("src.bioxp.lifecycle_state.lifecycle_state", lifecycle)

    store = OEMRuntimeStore(tmp_path)
    (tmp_path / "runtime_state.json").write_text(
        '{"runtime_state":"shutdown","worker":{"state":"stopped"}}'
    )
    status = OEMRuntimeStatusService(store=store, worker=Worker()).status()
    assert status["runtime_state"] == "waiting"
    assert status["operation_state"] == "waiting"
    assert "stale_runtime_state_corrected" not in status
    assert status["worker"]["state"] == "idle"
