from __future__ import annotations

import queue
import threading
import time
from typing import Any, Callable

from .oem_runtime_store import OEMRuntimeStore
from .oem_runtime_types import OEMRuntimeCommand, OEMRuntimeSnapshot, OEMRuntimeStateName, OEMWorkerSnapshot, OEMWorkerStateName, utc_ts
from .lifecycle_state import lifecycle_state

Handler = Callable[[OEMRuntimeCommand], dict[str, Any]]
TerminalSnapshotHook = Callable[[OEMRuntimeCommand, dict[str, Any]], dict[str, Any]]


class OEMRuntimeWorker:
    def __init__(
        self,
        *,
        store: OEMRuntimeStore,
        handlers: dict[str, Handler] | None = None,
        terminal_snapshot_hook: TerminalSnapshotHook | None = None,
        autostart: bool = False,
    ):
        self.store = store
        self.handlers = handlers or {}
        self.terminal_snapshot_hook = terminal_snapshot_hook
        self._queue: queue.Queue[OEMRuntimeCommand] = queue.Queue()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._state_lock = threading.Lock()
        self.state = OEMWorkerStateName.NOT_STARTED.value
        self.gantry_available = True
        self.active_command: OEMRuntimeCommand | None = None
        self.last_heartbeat_at: float | None = None
        if autostart:
            self.start()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self.state = OEMWorkerStateName.IDLE.value
        self._thread = threading.Thread(target=self._loop, name="bioxp-oem-runtime-worker", daemon=True)
        self._thread.start()
        lifecycle_state.transition("stopped", reason="runtime_worker_started_idle")
        self._write_snapshot(OEMRuntimeStateName.IDLE_NOT_READY.value)

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=timeout)
        self.state = OEMWorkerStateName.STOPPED.value
        self.gantry_available = True
        lifecycle_state.transition("stopped", reason="runtime_worker_stopped")
        self._write_snapshot(OEMRuntimeStateName.SHUTDOWN.value)

    def enqueue(self, command: OEMRuntimeCommand | dict[str, Any]) -> dict[str, Any]:
        cmd = command if isinstance(command, OEMRuntimeCommand) else OEMRuntimeCommand(**command)
        self.store.append_command_queue(cmd.to_dict())
        self._queue.put(cmd)
        self.state = OEMWorkerStateName.QUEUED.value
        lifecycle_state.transition("waiting", reason=f"runtime_command_queued:{cmd.name}")
        self._write_snapshot(OEMRuntimeStateName.IDLE_NOT_READY.value)
        return {"ok": True, "queued": True, "command": cmd.to_dict(), "queue_depth": self._queue.qsize()}

    def snapshot(self) -> dict[str, Any]:
        return OEMWorkerSnapshot(
            state=self.state,
            gantry_available=self.gantry_available,
            queue_depth=self._queue.qsize(),
            active_command=self.active_command.to_dict() if self.active_command else None,
            last_heartbeat_at=self.last_heartbeat_at,
        ).to_dict()

    def run_next_for_tests(self) -> dict[str, Any]:
        try:
            cmd = self._queue.get_nowait()
        except queue.Empty:
            return {"ok": True, "ran": False, "reason": "queue_empty", "worker": self.snapshot()}
        return self._run_command(cmd)

    def _loop(self) -> None:
        while not self._stop.is_set():
            self.last_heartbeat_at = utc_ts()
            try:
                cmd = self._queue.get(timeout=0.1)
            except queue.Empty:
                if self.active_command is None:
                    self.state = OEMWorkerStateName.IDLE.value
                continue
            self._run_command(cmd)

    def _run_command(self, cmd: OEMRuntimeCommand) -> dict[str, Any]:
        with self._state_lock:
            self.active_command = cmd
            self.state = OEMWorkerStateName.RUNNING.value
            self.gantry_available = False
            lifecycle_state.transition("running", reason=f"runtime_worker:{cmd.name}")
            self._write_snapshot(OEMRuntimeStateName.IDLE_NOT_READY.value)
        started = utc_ts()
        history = {"command": cmd.to_dict(), "started_at": started, "gantry_available_before": False}
        try:
            handler = self.handlers.get(cmd.name)
            if handler is None:
                raise RuntimeError(f"no handler registered for OEM runtime command {cmd.name}")
            result = handler(cmd)
            handler_state = result.get("state")
            handler_ok = bool(result.get("ok", True))
            lifecycle_state.transition(
                "stopped" if handler_ok else "error",
                reason=f"runtime_command_{'completed' if handler_ok else 'failed'}:{cmd.name}",
            )
            lifecycle = lifecycle_state.projection()
            # Preserve provider terminal truth for route-specific receipts. The
            # canonical owner state is published separately and remains the
            # only authority for subsequent transition admission.
            result["state"] = handler_state or lifecycle["operation_state"]
            result["operation_state"] = lifecycle["operation_state"]
            result["startup"] = lifecycle["startup"]
            if handler_state is not None:
                result["handler_outcome"] = handler_state
            self._attach_terminal_hardware_snapshot(cmd, result)
            history.update({"ok": handler_ok, "result": result, "finished_at": utc_ts()})
            if not history["ok"]:
                self.state = OEMWorkerStateName.FAILED.value
                lifecycle_state.transition("error", reason=f"runtime_command_failed:{cmd.name}")
            self.store.append_command_history(history)
            return {"ok": history["ok"], "ran": True, "result": result, "worker": self.snapshot()}
        except Exception as exc:
            self.state = OEMWorkerStateName.FAILED.value
            failed_result = {"ok": False, "error": str(exc)}
            self._attach_terminal_hardware_snapshot(cmd, failed_result)
            row = {**history, "ok": False, "error": str(exc), "result": failed_result, "finished_at": utc_ts()}
            self.store.append_command_history(row)
            self.store.append_error({"error_situation": "command_error", "command": cmd.to_dict(), "error": str(exc)})
            lifecycle_state.transition("error", reason=f"runtime_worker_failed:{cmd.name}")
            return {"ok": False, "ran": True, "error": str(exc), "worker": self.snapshot()}
        finally:
            with self._state_lock:
                self.active_command = None
                self.gantry_available = True
                if self.state != OEMWorkerStateName.FAILED.value:
                    self.state = OEMWorkerStateName.IDLE.value
                    lifecycle_state.transition("stopped", reason=f"runtime_worker_completed:{cmd.name}")
                self._write_snapshot(OEMRuntimeStateName.IDLE_NOT_READY.value)

    def _attach_terminal_hardware_snapshot(self, cmd: OEMRuntimeCommand, result: dict[str, Any]) -> None:
        """Publish query-only terminal evidence without changing command truth."""
        hook = self.terminal_snapshot_hook
        if hook is None:
            return
        try:
            snapshot = hook(cmd, dict(result))
            if not isinstance(snapshot, dict):
                raise TypeError("terminal hardware snapshot hook did not return an object")
            result["automatic_hardware_snapshot"] = snapshot
        except Exception as exc:
            result["automatic_hardware_snapshot"] = {
                "ok": False,
                "error": str(exc) or exc.__class__.__name__,
                "query_only": True,
            }

    def _write_snapshot(self, runtime_state: str) -> None:
        snap = OEMRuntimeSnapshot(runtime_state=runtime_state)
        snap.worker = OEMWorkerSnapshot(
            state=self.state,
            gantry_available=self.gantry_available,
            queue_depth=self._queue.qsize(),
            active_command=self.active_command.to_dict() if self.active_command else None,
            last_heartbeat_at=self.last_heartbeat_at,
        )
        self.store.write_state(snap)
