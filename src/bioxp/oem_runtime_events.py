from __future__ import annotations

from typing import Any

from .oem_runtime_store import OEMRuntimeStore
from .oem_runtime_types import OEMCommandName, OEMRuntimeCommand, OEMRuntimeEvent, OEMRuntimeStateName


class OEMRuntimeEventRouter:
    def __init__(self, *, store: OEMRuntimeStore, worker):
        self.store = store
        self.worker = worker
        self.user_paused = False
        self.waiting_for_door_close = False
        self.door_close_retry = 0

    def handle_door_event(self, *, door_open: bool | None = None, door_closed: bool | None = None, latch_closed: bool | None = None, source: str = "api", raw: dict[str, Any] | None = None, mode: str = "dry_run", artifact_root: str | None = None) -> dict[str, Any]:
        if door_closed is None:
            door_closed = not bool(door_open)
        actions: list[str] = []
        queued = None
        if self.user_paused:
            if door_closed and latch_closed:
                queued = self.worker.enqueue(OEMRuntimeCommand(name=OEMCommandName.WAKE_FROM_PAUSE.value, mode=mode, source="door_event", artifact_root=artifact_root, operator_ack="EVENT" if mode == "live" else None))
                actions.append("queued_wakefrompause")
            else:
                actions.append("operator_close_door_required")
        elif door_open:
            actions.extend(["initialCheck_required", "forceAbortMotion_required"])
            self.store.append_error({"error_situation": "door_malfunction", "safe_action_taken": "forceAbortMotion_required", "source": source})
        elif door_closed and latch_closed:
            self.door_close_retry = 0
            queued = self.worker.enqueue(OEMRuntimeCommand(name=OEMCommandName.INITIALIZE_SYSTEM.value, mode=mode, source="door_event", artifact_root=artifact_root, operator_ack="INITIALIZE" if mode == "live" else None))
            actions.append("queued_initializeSystem")
            self.waiting_for_door_close = False
        else:
            self.door_close_retry += 1
            actions.extend(["unlockDoor_required", "operator_wait_for_latch"])
        event = OEMRuntimeEvent(event_type="door", source=source, payload={"door_open": door_open, "door_closed": door_closed, "latch_closed": latch_closed, "raw": raw or {}, "door_close_retry": self.door_close_retry}, actions_taken=actions)
        row = self.store.append_event(event.to_dict())
        return {"ok": True, "event": row, "actions_taken": actions, "queued": queued}

    def handle_pause(self, *, source: str = "api") -> dict[str, Any]:
        self.user_paused = True
        row = self.store.append_event(OEMRuntimeEvent(event_type="pause", source=source, actions_taken=["user_paused=true"]).to_dict())
        return {"ok": True, "event": row, "runtime_state": OEMRuntimeStateName.USER_PAUSED.value}

    def handle_resume(self, *, source: str = "api", mode: str = "dry_run") -> dict[str, Any]:
        self.user_paused = False
        queued = self.worker.enqueue(OEMRuntimeCommand(name=OEMCommandName.WAKE_FROM_PAUSE.value, mode=mode, source="resume_event"))
        row = self.store.append_event(OEMRuntimeEvent(event_type="resume", source=source, actions_taken=["queued_wakefrompause"]).to_dict())
        return {"ok": True, "event": row, "queued": queued}

    def emergency_stop(self, *, source: str = "api", reason: str = "operator_request") -> dict[str, Any]:
        row = self.store.append_event(OEMRuntimeEvent(event_type="emergency_stop", source=source, payload={"reason": reason}, actions_taken=["runtime_emergency_stopped", "hardware_stop_required"]).to_dict())
        self.store.append_error({"error_situation": "emergency_stop", "operator_action_required": True, "reason": reason})
        return {"ok": True, "event": row, "runtime_state": OEMRuntimeStateName.EMERGENCY_STOPPED.value}
