from __future__ import annotations

import json
import os
import threading
import tempfile
from collections.abc import Callable
from pathlib import Path
from typing import Any

from .oem_runtime_types import OEMRuntimeSnapshot, utc_ts


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    """Durably replace one private JSON authority file.

    The temporary file and containing state directory are deliberately private.
    ``fsync`` is applied to both file contents and the parent directory so an ACK
    cannot be returned for a transition that only existed in page cache.
    """
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.chmod(path.parent, 0o700)
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    tmp = Path(tmp_name)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "wb", closefd=True) as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
        os.chmod(path, 0o600)
        directory_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            tmp.unlink()
        except FileNotFoundError:
            pass


class OEMRuntimeStore:
    def __init__(self, root: str | Path | None = None):
        self.root = Path(root or os.environ.get("BIOXP_OEM_RUNTIME_ROOT") or "/tmp/bioxp-oem-runtime")
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.root, 0o700)
        self._lock = threading.RLock()
        self._seq = self._load_seq()

    @property
    def serial206_initialization_state_path(self) -> Path:
        return self.root / "serial206_oem_initialization_state.json"

    def read_oem_serial206_initialization_state(self) -> dict[str, Any] | None:
        """Read the single atomic serial-206 lifecycle authority.

        JSON/schema errors intentionally propagate.  Callers must fail closed;
        treating corruption as a fresh state could replay acknowledged motion.
        """
        path = self.serial206_initialization_state_path
        if not path.exists():
            return None
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("serial-206 initialization state must be an object")
        return payload

    def write_oem_serial206_initialization_state(self, state: dict[str, Any]) -> dict[str, Any]:
        """Atomically persist movement, approval, and initializeMotion ledgers."""
        payload = dict(state)
        required = {"movement_ledger", "used_approvals", "initialize_motion_ledger"}
        if not required.issubset(payload):
            raise ValueError("serial-206 state must contain all lifecycle ledgers")
        with self._lock:
            _atomic_json(self.serial206_initialization_state_path, payload)
        return payload

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

    def create_oem_full_lifecycle_run_once(
        self,
        run: dict[str, Any],
        *,
        current_ownership_generation: Callable[[], int] | None = None,
    ) -> dict[str, Any]:
        """Atomically converge same-key creators within the single runtime owner."""
        payload = dict(run)
        key = payload.get("idempotency_key")
        request = payload.get("request")
        if not isinstance(key, str) or not key or len(key) > 128:
            raise ValueError("valid bounded idempotency_key required")
        with self._lock:
            if current_ownership_generation is not None:
                expected = request.get("expected_generation") if isinstance(request, dict) else None
                if expected != current_ownership_generation():
                    raise ValueError("expected_generation no longer matches current robot ownership generation")
            existing_runs = self.list_oem_full_lifecycle_runs()
            for existing in existing_runs:
                if existing.get("idempotency_key") != key:
                    continue
                if existing.get("request") != request:
                    raise ValueError("idempotency_key is already bound to a different request")
                return existing
            active_states = {"planned", "running", "admitted", "acknowledged", "blocked", "reconciliation_required"}
            if any(existing.get("run_state") in active_states for existing in existing_runs):
                raise ValueError("another active OEM lifecycle run already owns the robot lifecycle")
            return self.write_oem_full_lifecycle_run(payload)

    def write_oem_full_lifecycle_run(self, run: dict[str, Any]) -> dict[str, Any]:
        """Persist one full OEM movement-lifecycle run atomically.

        Run files are immutable by identity but replaceable by monotonic state
        updates.  The robot owns the directory and run identifier; callers do
        not supply paths.
        """
        payload = dict(run)
        run_id = str(payload.get("run_id") or "").strip()
        if not run_id or "/" in run_id or "\\" in run_id or run_id in {".", ".."}:
            raise ValueError("valid robot-owned run_id required")
        with self._lock:
            payload["sequence"] = self.next_seq()
            _atomic_json(self.root / "movement_runs" / f"{run_id}.json", payload)
        return payload

    def mutate_oem_full_lifecycle_run(
        self,
        run_id: str,
        mutation: Callable[[dict[str, Any]], dict[str, Any]],
    ) -> dict[str, Any]:
        """Read, validate, and replace one run under the runtime-owner lock."""
        with self._lock:
            payload = self.read_oem_full_lifecycle_run(run_id)
            if payload is None:
                raise ValueError(f"full OEM lifecycle run {run_id!r} not found")
            return self.write_oem_full_lifecycle_run(mutation(payload))

    def read_oem_full_lifecycle_run(self, run_id: str) -> dict[str, Any] | None:
        selected = str(run_id).strip()
        if not selected or "/" in selected or "\\" in selected or selected in {".", ".."}:
            raise ValueError("valid robot-owned run_id required")
        path = self.root / "movement_runs" / f"{selected}.json"
        if not path.exists():
            return None
        return json.loads(path.read_text())

    def list_oem_full_lifecycle_runs(self) -> list[dict[str, Any]]:
        root = self.root / "movement_runs"
        if not root.exists():
            return []
        rows: list[dict[str, Any]] = []
        for path in sorted(root.glob("*.json")):
            rows.append(json.loads(path.read_text()))
        return rows

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
