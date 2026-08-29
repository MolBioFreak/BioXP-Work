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
