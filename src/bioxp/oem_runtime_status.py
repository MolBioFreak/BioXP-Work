from __future__ import annotations

from .oem_runtime_store import OEMRuntimeStore
from .oem_runtime_types import OEMRuntimeSnapshot


class OEMRuntimeStatusService:
    def __init__(self, *, store: OEMRuntimeStore, worker=None):
        self.store = store
        self.worker = worker

    def status(self) -> dict:
        state = self.store.read_state() or OEMRuntimeSnapshot().to_dict()
        if self.worker is not None:
            state["worker"] = self.worker.snapshot()
        state.setdefault("ok", True)
        state.setdefault("truth_source", "oem_runtime_store")
        state.setdefault("bms_role", "thin_operator_surface")
        return state
