from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any

from .oem_runtime_types import OEMRuntimeSnapshot, utc_ts


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True))
    tmp.replace(path)


class OEMRuntimeStore:
    def __init__(self, root: str | Path | None = None):
        self.root = Path(root or os.environ.get("BIOXP_OEM_RUNTIME_ROOT") or "/tmp/bioxp-oem-runtime")
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._seq = self._load_seq()

    def _load_seq(self) -> int:
        p = self.root / "sequence.txt"
        try:
            return int(p.read_text().strip())
        except Exception:
            return 0

    def next_seq(self) -> int:
        with self._lock:
            self._seq += 1
            (self.root / "sequence.txt").write_text(str(self._seq))
            return self._seq

    def write_state(self, snapshot: OEMRuntimeSnapshot | dict[str, Any]) -> dict[str, Any]:
        payload = snapshot.to_dict() if hasattr(snapshot, "to_dict") else dict(snapshot)
        from .hardware_status import hardware_state
        from .lifecycle_state import lifecycle_state

        canonical = hardware_state.completed_snapshot()
        payload["canonical_hardware_snapshot"] = {
            "snapshot_id": None if canonical is None else canonical.get("snapshot_id"),
            "ownership_epoch": hardware_state.ownership_epoch,
            "available": canonical is not None,
            "reference_only": True,
        }
        lifecycle = lifecycle_state.projection()
        payload["runtime_state"] = lifecycle["operation_state"]
        payload["operation_state"] = lifecycle["operation_state"]
        payload["startup"] = lifecycle["startup"]
        payload["lifecycle_revision"] = lifecycle["revision"]
        payload["sequence"] = self.next_seq()
        payload["updated_at"] = utc_ts()
        _atomic_json(self.root / "runtime_state.json", payload)
        return payload

    def read_state(self) -> dict[str, Any] | None:
        p = self.root / "runtime_state.json"
        if not p.exists():
            return None
        return json.loads(p.read_text())

    def write_oem_movement_ledger(self, ledger: dict[str, Any]) -> dict[str, Any]:
        """Persist the robot-owned initializeMotors source-order ledger atomically."""
        payload = dict(ledger)
        with self._lock:
            payload["sequence"] = self.next_seq()
            _atomic_json(self.root / "oem_initialize_motors_ledger.json", payload)
        return payload

    def read_oem_movement_ledger(self) -> dict[str, Any] | None:
        path = self.root / "oem_initialize_motors_ledger.json"
        if not path.exists():
            return None
        return json.loads(path.read_text())

    def append_journal(self, name: str, payload: dict[str, Any]) -> dict[str, Any]:
        row = dict(payload)
        row.setdefault("created_at", utc_ts())
        row["sequence"] = self.next_seq()
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            with path.open("a") as fh:
                fh.write(json.dumps(row, sort_keys=True) + "\n")
        return row

    def append_command_queue(self, command: dict[str, Any]) -> dict[str, Any]:
        return self.append_journal("command_queue.jsonl", command)

    def append_command_history(self, row: dict[str, Any]) -> dict[str, Any]:
        return self.append_journal("command_history.jsonl", row)

    def append_event(self, event: dict[str, Any]) -> dict[str, Any]:
        return self.append_journal("event_journal.jsonl", event)

    def append_error(self, error: dict[str, Any]) -> dict[str, Any]:
        return self.append_journal("runtime_errors.jsonl", error)

    def read_journal(self, name: str, limit: int = 50) -> list[dict[str, Any]]:
        path = self.root / name
        if not path.exists():
            return []
        rows = []
        for line in path.read_text().splitlines():
            if line.strip():
                rows.append(json.loads(line))
        return rows[-limit:]

    def recover_state(self) -> dict[str, Any]:
        state = self.read_state()
        if state is None:
            return {"recovery": "fresh", "state": None, "recovery_required": False}
        worker = state.get("worker") or {}
        active = worker.get("active_command")
        running = worker.get("state") == "running" or active is not None
        return {"recovery": "active_command" if running else "idle", "state": state, "recovery_required": bool(running)}
