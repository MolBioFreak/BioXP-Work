from src.bioxp.oem_runtime_status import OEMRuntimeStatusService
from src.bioxp.oem_runtime_store import OEMRuntimeStore


class Worker:
    def snapshot(self):
        return {
            "state": "idle",
            "gantry_available": True,
            "queue_depth": 0,
            "active_command": None,
            "last_heartbeat_at": 123.0,
        }


def test_runtime_status_corrects_stale_shutdown_when_worker_is_alive(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    store.write_state({"runtime_state": "shutdown", "worker": {"state": "stopped"}})
    status = OEMRuntimeStatusService(store=store, worker=Worker()).status()
    assert status["runtime_state"] == "idle_not_ready"
    assert status["stale_runtime_state_corrected"] == "shutdown"
    assert status["worker"]["state"] == "idle"
