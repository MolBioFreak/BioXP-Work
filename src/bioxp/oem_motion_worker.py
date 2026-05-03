from __future__ import annotations

import json
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Deque


@dataclass
class OemMotionCommand:
    session_id: str
    name: str
    payload: dict = field(default_factory=dict)
    command_id: str = field(default_factory=lambda: uuid.uuid4().hex[:12])
    created_ms: int = field(default_factory=lambda: int(time.time() * 1000))


class OEMMotionWorker:
    def __init__(self, *, artifact_root: str | Path, handlers: dict[str, Callable[[OemMotionCommand], dict]] | None = None):
        self.artifact_root = Path(artifact_root)
        self.artifact_root.mkdir(parents=True, exist_ok=True)
        self.handlers = handlers or {}
        self.queue: Deque[OemMotionCommand] = deque()
        self.state = "idle"
        self.active: OemMotionCommand | None = None
        self.last_result: dict | None = None

    @property
    def event_path(self) -> Path:
        return self.artifact_root / "motion_queue_events.jsonl"

    def _event(self, event: str, command: OemMotionCommand | None = None, **extra) -> None:
        row = {"ts_ms": int(time.time() * 1000), "event": event}
        if command is not None:
            row.update({"command_id": command.command_id, "session_id": command.session_id, "name": command.name})
        row.update(extra)
        with self.event_path.open("a") as fh:
            fh.write(json.dumps(row, sort_keys=True) + "\n")

    def enqueue(self, command: OemMotionCommand) -> dict:
        self.queue.append(command)
        self.state = "queued" if self.active is None else self.state
        self._event("queued", command)
        return {"queued": True, "command_id": command.command_id, "session_id": command.session_id, "name": command.name}

    def run_next(self) -> dict | None:
        if not self.queue:
            self.state = "idle"
            return None
        command = self.queue.popleft()
        self.active = command
        self.state = "running"
        self._event("start", command)
        try:
            handler = self.handlers.get(command.name, lambda cmd: {"ok": True, "unhandled": True})
            result = handler(command)
            self.last_result = result
            self._event("complete", command, ok=bool(result.get("ok", True)))
            return result
        except Exception as exc:
            self.last_result = {"ok": False, "error": str(exc)}
            self.state = "failed"
            self._event("failed", command, error=str(exc))
            return self.last_result
        finally:
            self.active = None
            if self.state != "failed":
                self.state = "queued" if self.queue else "idle"

    def abort(self, *, reason: str = "abort") -> dict:
        active = self.active
        self.queue.clear()
        self.state = "aborted"
        self._event("abort", active, reason=reason)
        self.last_result = {"ok": False, "aborted": True, "reason": reason}
        return {"aborted": True, "reason": reason}

    def status(self) -> dict:
        return {
            "state": self.state,
            "active_command": None if self.active is None else self.active.name,
            "active_session_id": None if self.active is None else self.active.session_id,
            "queue_depth": len(self.queue),
            "last_result": self.last_result,
        }
