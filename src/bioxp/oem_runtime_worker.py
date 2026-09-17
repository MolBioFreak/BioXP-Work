from __future__ import annotations

from typing import Any, Callable

from .oem_runtime_store import OEMRuntimeStore
from .oem_runtime_types import OEMRuntimeCommand, OEMWorkerSnapshot, OEMWorkerStateName

Handler = Callable[[OEMRuntimeCommand], dict[str, Any]]
TerminalSnapshotHook = Callable[[OEMRuntimeCommand, dict[str, Any]], dict[str, Any]]


class OEMRuntimeWorker:
    """Retained runtime preview facade, no independent execution custody.

    New physical workflows belong exclusively to the command-plane dispatcher.
    Historical queue/history remain readable in OEMRuntimeStore. Preview results
    return synchronously and do not create competing command or history records.
    """

    def __init__(self, *, store: OEMRuntimeStore, handlers: dict[str, Handler] | None = None,
                 terminal_snapshot_hook: TerminalSnapshotHook | None = None, autostart: bool = False):
        self.store = store
        self.handlers = handlers or {}
        self.state = OEMWorkerStateName.IDLE.value if autostart else OEMWorkerStateName.NOT_STARTED.value
        self.gantry_available = True
        self.active_command = None
        self.last_heartbeat_at = None

    def start(self) -> None:
        self.state = OEMWorkerStateName.IDLE.value

    def stop(self, timeout: float = 2.0) -> None:
        self.state = OEMWorkerStateName.STOPPED.value

    def enqueue(self, command: OEMRuntimeCommand | dict[str, Any]) -> dict[str, Any]:
        cmd = command if isinstance(command, OEMRuntimeCommand) else OEMRuntimeCommand(**command)
        if cmd.mode != "dry_run":
            return {"ok": False, "queued": False, "error": "legacy_runtime_execution_retired",
                    "replacement": "/protocol/execute", "command": cmd.to_dict()}
        handler = self.handlers.get(cmd.name)
        if handler is None:
            return {"ok": False, "queued": False, "error": "runtime_preview_unavailable"}
        result = handler(cmd)
        if not isinstance(result, dict) or type(result.get("ok")) is not bool:
            raise RuntimeError("Runtime preview must return an explicit result")
        return {"ok": result["ok"], "queued": False, "preview_only": True,
                "command": cmd.to_dict(), "result": result, "worker": self.snapshot()}

    def snapshot(self) -> dict[str, Any]:
        return OEMWorkerSnapshot(state=self.state, gantry_available=self.gantry_available,
                                 queue_depth=0, active_command=None,
                                 last_heartbeat_at=None).to_dict()

    def run_next_for_tests(self) -> dict[str, Any]:
        return {"ok": True, "ran": False, "reason": "execution_owner_retired", "worker": self.snapshot()}
