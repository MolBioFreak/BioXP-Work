from __future__ import annotations

from .oem_runtime_store import OEMRuntimeStore


class OEMRuntimeStatusService:
    def __init__(self, *, store: OEMRuntimeStore, worker=None):
        self.store = store
        self.worker = worker

    def status(self) -> dict:
        from .lifecycle_state import lifecycle_state

        lifecycle = lifecycle_state.projection()
        state = self.store.read_state()
        if state is None:
            return {
                "ok": lifecycle["operation_state"] not in {"error", "emergency"},
                "available": True,
                "cache_state": "missing",
                "runtime_state": lifecycle["operation_state"],
                "operation_state": lifecycle["operation_state"],
                "startup": lifecycle["startup"],
                "lifecycle": lifecycle,
                "canonical_hardware_snapshot": None,
                "error": "persisted OEM runtime state is unavailable; canonical lifecycle remains authoritative",
            }
        state.setdefault(
            "canonical_hardware_snapshot",
            {
                "snapshot_id": None,
                "ownership_epoch": None,
                "available": False,
                "reference_only": True,
                "error": "legacy persisted state has no canonical snapshot reference",
            },
        )
        if self.worker is not None:
            worker_snapshot = self.worker.snapshot()
            state["worker"] = worker_snapshot
        state["runtime_state"] = lifecycle["operation_state"]
        state["operation_state"] = lifecycle["operation_state"]
        state["startup"] = lifecycle["startup"]
        state["lifecycle"] = lifecycle
        machine_status = state.setdefault("machine_status", {})
        machine_status["enclosure_door_closed"] = lifecycle["door"]["door_closed"]
        machine_status["latch_closed"] = lifecycle["door"]["latch_closed"]
        machine_status["user_paused"] = lifecycle["operation_state"] == "paused"
        machine_status["running_job"] = lifecycle["operation_state"] == "running"
        machine_status["latest_status"] = lifecycle["operation_state"]
        state["ok"] = lifecycle["operation_state"] not in {"error", "emergency"}
        state.setdefault("truth_source", "oem_runtime_store")
        state.setdefault("bms_role", "thin_operator_surface")
        return state
