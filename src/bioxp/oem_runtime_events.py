from __future__ import annotations

from typing import Any

from .oem_runtime_store import OEMRuntimeStore
from .oem_runtime_types import OEMCommandName, OEMRuntimeCommand, OEMRuntimeEvent, OEMRuntimeStateName
from .lifecycle_state import lifecycle_state


class OEMRuntimeEventRouter:
    def __init__(self, *, store: OEMRuntimeStore, worker):
        self.store = store
        self.worker = worker
        self.door_close_retry = 0

    def handle_door_event(self, *, door_open: bool | None = None, door_closed: bool | None = None, latch_closed: bool | None = None, source: str = "api", raw: dict[str, Any] | None = None, mode: str = "dry_run", artifact_root: str | None = None) -> dict[str, Any]:
        if door_closed is None:
            door_closed = not bool(door_open)
        actions: list[str] = []
        queued = None
        lifecycle_before = lifecycle_state.projection()
        if lifecycle_before["operation_state"] == "paused":
            if door_closed and latch_closed:
                actions.append("explicit_resume_required")
            else:
                actions.append("operator_close_door_required")
        elif door_open:
            actions.extend(["initialCheck_required", "forceAbortMotion_required"])
            self.store.append_error({"error_situation": "door_malfunction", "safe_action_taken": "forceAbortMotion_required", "source": source})
        elif door_closed and latch_closed:
            self.door_close_retry = 0
            # Door evidence updates the canonical lifecycle only.  It cannot
            # implicitly execute or bypass a startup predecessor stage.
            actions.append("door_close_observed")
        else:
            self.door_close_retry += 1
            actions.extend(["unlockDoor_required", "operator_wait_for_latch"])
        event = OEMRuntimeEvent(event_type="door", source=source, payload={"door_open": door_open, "door_closed": door_closed, "latch_closed": latch_closed, "raw": raw or {}, "door_close_retry": self.door_close_retry}, actions_taken=actions)
        lifecycle = lifecycle_state.record_door_event(door_closed=door_closed, latch_closed=latch_closed, source=f"runtime_event:{source}")
        row = self.store.append_event(event.to_dict())
        return {"ok": True, "event": row, "actions_taken": actions, "queued": queued, "lifecycle": lifecycle}

    def handle_pause(self, *, source: str = "api") -> dict[str, Any]:
        lifecycle = lifecycle_state.transition("paused", reason=f"pause:{source}")
        row = self.store.append_event(OEMRuntimeEvent(event_type="pause", source=source, actions_taken=["user_paused=true"]).to_dict())
        return {"ok": True, "event": row, "runtime_state": lifecycle["operation_state"], "lifecycle": lifecycle}

    def handle_resume(self, *, source: str = "api", mode: str = "dry_run") -> dict[str, Any]:
        queued = self.worker.enqueue(OEMRuntimeCommand(name=OEMCommandName.WAKE_FROM_PAUSE.value, mode=mode, source="resume_event"))
        lifecycle = lifecycle_state.projection()
        row = self.store.append_event(OEMRuntimeEvent(event_type="resume", source=source, actions_taken=["queued_wakefrompause"]).to_dict())
        return {"ok": True, "event": row, "queued": queued, "runtime_state": lifecycle["operation_state"], "lifecycle": lifecycle}

    def emergency_stop(self, *, source: str = "api", reason: str = "operator_request") -> dict[str, Any]:
        row = self.store.append_event(OEMRuntimeEvent(event_type="emergency_stop", source=source, payload={"reason": reason}, actions_taken=["runtime_emergency_stopped", "hardware_stop_required"]).to_dict())
        self.store.append_error({"error_situation": "emergency_stop", "operator_action_required": True, "reason": reason})
        lifecycle = lifecycle_state.transition("emergency", reason=f"emergency_stop:{reason}")
        return {"ok": True, "event": row, "runtime_state": lifecycle["operation_state"], "lifecycle": lifecycle}
