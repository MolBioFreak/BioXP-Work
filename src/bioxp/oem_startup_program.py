from __future__ import annotations

import json
import time
import uuid
from pathlib import Path
from typing import Any

from .oem_config import find_oem_config
from .oem_motion_worker import OEMMotionWorker, OemMotionCommand
from .oem_startup_types import OemStartupState


def _atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True))
    tmp.replace(path)


class FakeStartupHardware:
    def __init__(self, *, door_closed: bool = True, latch_closed: bool = True, config_status: dict | None = None):
        self.door_closed = door_closed
        self.latch_closed = latch_closed
        self.config_status = config_status
        self.motion_calls: list[str] = []
        self.initial_check_calls = 0

    def load_config(self) -> dict:
        return self.config_status or {"status": "missing", "path": None, "searched_roots": [], "fields": {}}

    def initial_check(self, *, mode: str = "dry_run") -> dict:
        self.initial_check_calls += 1
        return {
            "ok": bool(self.door_closed and self.latch_closed),
            "source_anchor": "ControlLib.initialCheck lines 8728-8759",
            "door_latch": {"door_closed": bool(self.door_closed), "latch_closed": bool(self.latch_closed)},
            "checks": [
                {"name": "door_closed", "ok": bool(self.door_closed)},
                {"name": "latch_closed", "ok": bool(self.latch_closed)},
            ],
        }

    def configure_without_motion(self, *, mode: str = "dry_run") -> dict:
        return {"ok": True, "physical_motion": False, "source_anchor": "ClassControlInterface.initializeMotorsWithoutMotion line 3181"}


class BioXpStartupHardware:
    def __init__(self, tester_factory, *, config_roots: list[str] | None = None):
        self._tester_factory = tester_factory
        self._tester = None
        self.config_roots = config_roots

    @property
    def tester(self):
        if self._tester is None:
            self._tester = self._tester_factory()
        return self._tester

    def load_config(self) -> dict:
        return find_oem_config(self.config_roots)

    def initial_check(self, *, mode: str = "shadow") -> dict:
        tester = self.tester
        if hasattr(tester, "oem_initial_check"):
            return tester.oem_initial_check(mode=mode)
        snap = tester.io_snapshot(tester.BOARD_DECK)
        door_closed = bool(snap.get(1))
        latch_closed = bool(snap.get(3))
        return {
            "ok": door_closed and latch_closed,
            "source_anchor": "ControlLib.initialCheck lines 8728-8759 (compat wrapper)",
            "door_latch": {"door_closed": door_closed, "latch_closed": latch_closed, "snapshot": snap},
            "checks": [{"name": "door_latch", "ok": door_closed and latch_closed}],
        }

    def configure_without_motion(self, *, mode: str = "shadow") -> dict:
        tester = self.tester
        if hasattr(tester, "motor_oem_initialize_without_motion") and mode == "live":
            return tester.motor_oem_initialize_without_motion()
        return {"ok": True, "physical_motion": False, "source_anchor": "ClassControlInterface.initializeMotorsWithoutMotion line 3181", "skipped_live_write": mode != "live"}


class OEMStartupProgram:
    def __init__(self, *, hardware: Any, artifact_base: str | Path = "/tmp/bioxp-live-runs"):
        self.hardware = hardware
        self.artifact_base = Path(artifact_base)
        self.sessions: dict[str, dict] = {}
        self.latest_session_id: str | None = None

    def _new_session(self, req: dict) -> dict:
        sid = time.strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
        artifact_root = Path(req.get("artifact_root") or (self.artifact_base / f"{sid}_OEM_APP_STARTUP_SEQUENCE"))
        artifact_root.mkdir(parents=True, exist_ok=True)
        status = {
            "ok": True,
            "session_id": sid,
            "mode": req.get("mode", "dry_run"),
            "state": OemStartupState.CREATED.value,
            "ready": False,
            "failed": False,
            "failed_closed": False,
            "failure_reason": None,
            "active_stage": None,
            "completed_stages": [],
            "pending_stages": [],
            "artifact_root": str(artifact_root),
            "artifacts": {},
            "source_anchors": {
                "initializeEnvironment": "BioXPMainWindow.initializeEnvironment lines 973-1004",
                "doorEvent": "BioXPMainWindow.m_canControl_handleEnclosureDoorEventProcess lines 2428-2512",
                "motionQueue": "BioXPMainWindow.motion_thread_process lines 2030-2100",
                "initialCheck": "ControlLib.initialCheck lines 8728-8759",
            },
        }
        self.sessions[sid] = status
        self.latest_session_id = sid
        _atomic_json(artifact_root / "startup_request.json", req)
        _atomic_json(artifact_root / "source_anchors.json", status["source_anchors"])
        return status

    def _fail(self, status: dict, reason: str) -> dict:
        status.update({"ok": False, "state": OemStartupState.FAILED_CLOSED.value, "failed": True, "failed_closed": True, "failure_reason": reason})
        _atomic_json(Path(status["artifact_root"]) / "failure.json", {"reason": reason, "state": status["state"]})
        _atomic_json(Path(status["artifact_root"]) / "final_readiness.json", status)
        return dict(status)

    def request_startup(self, request: dict) -> dict:
        req = dict(request or {})
        req.setdefault("mode", "dry_run")
        req.setdefault("require_config", True)
        req.setdefault("door_policy", "wait_for_closed")
        if req["mode"] == "live" and req.get("operator_ack") != "INITIALIZE":
            status = self._new_session(req)
            return self._fail(status, "operator_ack INITIALIZE required for live mode")
        if req["mode"] == "live" and not req.get("artifact_root"):
            status = self._new_session(req)
            return self._fail(status, "artifact_root required for live mode")

        status = self._new_session(req)
        root = Path(status["artifact_root"])
        config = self.hardware.load_config()
        _atomic_json(root / "config_search.json", config)
        _atomic_json(root / "config_binding.json", config)
        status["config"] = config
        if req.get("require_config") and config.get("status") != "loaded":
            return self._fail(status, "config required but OEM config.xml binding is missing")
        status["completed_stages"].append("config_loaded" if config.get("status") == "loaded" else "config_missing")

        backend = {"ok": True, "mode": req["mode"]}
        _atomic_json(root / "backend_ready.json", backend)
        status["completed_stages"].append("backend_ready")
        _atomic_json(root / "control_lib_constructed.json", {"ok": True, "represented": True})
        status["completed_stages"].append("control_lib_constructed")
        _atomic_json(root / "pipette_startup_check.json", {"ok": True, "skipped": True, "reason": "startup shell; detailed pipette parity phase pending"})
        status["completed_stages"].append("pipette_checked")

        prep = self.hardware.configure_without_motion(mode=req["mode"])
        _atomic_json(root / "initialize_motors_without_motion.json", prep)
        status["completed_stages"].append("initialize_motors_without_motion")
        _atomic_json(root / "initialize_environment.json", {"ok": True, "source_anchor": "BioXPMainWindow.initializeEnvironment lines 973-1004"})

        initial = self.hardware.initial_check(mode=req["mode"])
        _atomic_json(root / "initial_check_before_door.json", initial)
        status["door_latch"] = initial.get("door_latch", {})
        status["completed_stages"].append("initial_check_before_door")
        if not initial.get("ok"):
            if req.get("door_policy") == "wait_for_closed":
                status["state"] = OemStartupState.WAITING_FOR_DOOR_CLOSE.value
                _atomic_json(root / "door_wait.json", {"ok": True, "waiting": True, "door_latch": status["door_latch"]})
                _atomic_json(root / "final_readiness.json", status)
                return dict(status)
            return self._fail(status, "door/latch gate failed during initialCheck")
        status["state"] = OemStartupState.DOOR_CLOSE_OBSERVED.value
        return self._queue_initialize_system(status)

    def _queue_initialize_system(self, status: dict) -> dict:
        root = Path(status["artifact_root"])
        worker = OEMMotionWorker(artifact_root=root)
        cmd = OemMotionCommand(session_id=status["session_id"], name="initializeSystem")
        queued = worker.enqueue(cmd)
        status["motion_worker"] = worker.status()
        status["queued"] = True
        status["command"] = queued
        status["state"] = OemStartupState.INITIALIZE_SYSTEM_QUEUED.value
        status["completed_stages"].append("initial_check_after_door" if "initial_check_after_door" in status["completed_stages"] else "initialize_system_queued")
        _atomic_json(root / "initialize_system.json", {"queued": True, "command": queued, "not_run_inline": True})
        _atomic_json(root / "final_readiness.json", status)
        return dict(status)

    def door_event(self, session_id: str | None, *, door_closed: bool, latch_closed: bool) -> dict:
        sid = session_id or self.latest_session_id
        if not sid or sid not in self.sessions:
            raise KeyError("startup session not found")
        status = self.sessions[sid]
        root = Path(status["artifact_root"])
        if hasattr(self.hardware, "door_closed"):
            self.hardware.door_closed = bool(door_closed)
        if hasattr(self.hardware, "latch_closed"):
            self.hardware.latch_closed = bool(latch_closed)
        event = {"door_closed": bool(door_closed), "latch_closed": bool(latch_closed), "source_anchor": "BioXPMainWindow door-close event lines 2428-2512"}
        _atomic_json(root / "door_event.json", event)
        initial = self.hardware.initial_check(mode=status.get("mode", "dry_run"))
        _atomic_json(root / "initial_check_after_door.json", initial)
        status["door_latch"] = initial.get("door_latch", event)
        status["completed_stages"].append("initial_check_after_door")
        if not initial.get("ok"):
            return self._fail(status, "door close event did not satisfy door/latch initialCheck")
        return self._queue_initialize_system(status)

    def status(self, session_id: str | None = None) -> dict:
        sid = session_id or self.latest_session_id
        if not sid or sid not in self.sessions:
            return {"ok": False, "state": "none", "ready": False, "failed": False, "failed_closed": False, "failure_reason": "no startup session"}
        return dict(self.sessions[sid])
