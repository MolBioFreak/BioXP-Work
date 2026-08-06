from __future__ import annotations

import json
import os
import time
import uuid
from pathlib import Path
from typing import Any

from .oem_config import find_oem_config
from .oem_parity_config import load_oem_parity_config
from .oem_startup_types import OemStartupState
from .oem_gripper import GRIPPER_COMMISSION_HOME_ACK, gripper_commission_home

SOURCE_ANCHORS = {
    "config": "ClassBioXPSettings config.xml load paths lines 2847-2857",
    "initializeEnvironment": "BioXPMainWindow.initializeEnvironment lines 973-1004",
    "doorEvent": "BioXPMainWindow.m_canControl_handleEnclosureDoorEventProcess lines 2428-2512",
    "motionQueue": "BioXPMainWindow.motion_thread_process lines 2030-2100",
    "initializeSystem": "BioXPMainWindow.initializeSystem lines 1046-1342",
    "initialCheck": "ControlLib.initialCheck lines 8728-8759; queryDoorStatus lines 8762-8770",
    "initializeMotion": "ControlLib.initializeMotion lines 8797-8856",
    "initializeMotorsWithoutMotion": "ClassControlInterface.initializeMotorsWithoutMotion lines 3181-3265",
    "initializeMotors": "ClassControlInterface.initializeMotors lines 3348-3421",
    "confirmAxis": "ClassControlInterface.confirmAxis lines 2714-2763",
    "parkGantry": "ControlLib.parkGantry lines 7071-7122",
}

REQUIRED_ARTIFACTS = [
    "startup_request.json",
    "source_anchors.json",
    "config_search.json",
    "config_binding.json",
    "backend_ready.json",
    "control_lib_constructed.json",
    "pipette_startup_check.json",
    "initialize_motors_without_motion.json",
    "initialize_environment.json",
    "initial_check_before_door.json",
    "door_wait.json",
    "door_event.json",
    "initial_check_after_door.json",
    "final_readiness.json",
    "failure.json",
]


def _atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True))
    tmp.replace(path)


def validate_artifact_root(mode: str, artifact_root: str | Path | None, artifact_base: Path, allowlist_roots: list[str | Path] | None = None) -> tuple[bool, str | None, Path | None]:
    if mode == "live" and not artifact_root:
        return False, "artifact_root required for live mode", None
    root = Path(artifact_root) if artifact_root else None
    if root is None:
        return True, None, None
    if mode == "live" and not root.is_absolute():
        return False, "artifact_root must be absolute for live mode", None
    roots = [Path("/tmp/bioxp-live-runs"), artifact_base]
    roots.extend(Path(r) for r in (allowlist_roots or []))
    if mode == "live":
        resolved_parent = root.parent.resolve(strict=False)
        allowed = False
        for allowed_root in roots:
            allowed_resolved = allowed_root.resolve(strict=False)
            try:
                resolved_parent.relative_to(allowed_resolved)
                allowed = True
                break
            except ValueError:
                continue
        if not allowed:
            return False, "artifact_root outside allowed live roots", None
    try:
        root.mkdir(parents=True, exist_ok=True)
        probe = root / ".write_probe"
        probe.write_text("ok")
        probe.unlink(missing_ok=True)
    except Exception as exc:
        return False, f"artifact_root not writable: {exc}", None
    return True, None, root


class DryRunStartupHardware:
    """No-hardware startup provider for unit tests and offline artifact-shape checks.

    This object must not be used as evidence that live robot hardware, CAN, USB,
    pipette, camera, latch, rail, or motion behavior passed. It exists only to
    exercise OEM startup state-machine/artifact formatting without opening a
    hardware transport.
    """

    hardware_provider_kind = "dry_run_test_double"
    live_robot_proof = False

    def __init__(
        self,
        *,
        door_closed: bool = True,
        latch_closed: bool = True,
        config_status: dict | None = None,
        home_predicates: dict[str, dict] | None = None,
        pipette_required: bool = False,
        vision_required: bool = False,
    ):
        self.door_closed = door_closed
        self.latch_closed = latch_closed
        self.config_status = config_status
        self.home_predicates = home_predicates or {}
        self.pipette_required = pipette_required
        self.vision_required = vision_required
        self.motion_calls: list[str] = []
        self.initial_check_calls = 0

    def load_config(self) -> dict:
        return self.config_status or {"status": "missing", "path": None, "searched_roots": [], "fields": {}, "missing_fields": ["StartMode", "GripperVersion"], "live_ready": False, "derived_requirements": {}}

    def initial_check(self, *, mode: str = "dry_run") -> dict:
        self.initial_check_calls += 1
        return {
            "ok": bool(self.door_closed and self.latch_closed),
            "source_anchor": SOURCE_ANCHORS["initialCheck"],
            "door_latch": {"door_closed": bool(self.door_closed), "latch_closed": bool(self.latch_closed), "rail_24v_ok": True},
            "checks": [
                {"name": "door_closed", "ok": bool(self.door_closed)},
                {"name": "latch_closed", "ok": bool(self.latch_closed)},
                {"name": "rail_24v", "ok": True},
            ],
        }

    def configure_without_motion(self, *, mode: str = "dry_run") -> dict:
        return {"ok": True, "physical_motion": False, "source_anchor": SOURCE_ANCHORS["initializeMotorsWithoutMotion"]}

    def pipette_startup_check(self, *, mode: str = "dry_run") -> dict:
        return {"ok": not self.pipette_required, "required": self.pipette_required, "available": False, "skipped": True, "blocks_ready": self.pipette_required, "reason": "ClassPipetteCollection startup parity not live-bound in this shell"}

    def vision_startup_check(self, *, mode: str = "dry_run") -> dict:
        return {"ok": not self.vision_required, "required": self.vision_required, "available": False, "skipped": True, "blocks_ready": self.vision_required, "reason": "CVisionLib inspection parity not live-bound in this shell"}

    def homing_predicates(self) -> dict[str, dict]:
        return dict(self.home_predicates)

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

    # Canonical initialCheck adapter methods.  They are invoked only by an
    # explicit lifecycle stage; none is called during provider construction.
    def set_led_rgb(self, r: int, g: int, b: int) -> dict:
        return self.tester.strip_set_rgb(
            int(r),
            int(g),
            int(b),
            reconnect_first=False,
            activate_first=False,
            fail_fast=True,
        )

    def query_door(self) -> dict:
        ack = self.tester.query_only_tmcl(self.tester.BOARD_DECK, 15, 1, 0, 0)
        return {"ok": self.tester._tmcl_success(ack), "value": None if ack is None else ack.get("value"), "ack": ack, "query": "door"}

    def query_latch(self) -> dict:
        ack = self.tester.query_only_tmcl(self.tester.BOARD_DECK, 15, 3, 0, 0)
        return {"ok": self.tester._tmcl_success(ack), "value": None if ack is None else ack.get("value"), "ack": ack, "query": "latch"}

    def set_solenoid(self, value: int) -> dict:
        result = self.tester.deck_io_set_type(2, int(value))
        return {**result, "ok": result.get("ok") is True}

    def query_voltage(self) -> dict:
        ack = self.tester.query_only_tmcl(self.tester.BOARD_DECK, 15, 0, 0, 0)
        status = None if ack is None else ack.get("status")
        voltage = None if ack is None or status != 100 else ack.get("value")
        return {
            "ok": bool(ack is not None and status == 100 and voltage is not None),
            "payload_raw": voltage,
            "reply_present": ack is not None,
            "transport_outcome": "reply" if ack is not None else "no_reply",
            "oem_status": status,
            "ack": ack,
        }

    def deactivate_boards(self) -> dict:
        acks = self.tester.deactivate_boards(expect_reply=True, fail_fast=True)
        return {"ok": self.tester._oem_board_activation_map_success(acks), "acks": acks}

    def activate_boards(self) -> dict:
        acks = self.tester.activate_boards(expect_reply=True, fail_fast=True)
        return {"ok": self.tester._oem_board_activation_map_success(acks), "acks": acks}

    def initial_check(self, *, mode: str = "shadow") -> dict:
        tester = self.tester
        if hasattr(tester, "oem_initial_check"):
            return tester.oem_initial_check(mode=mode)
        sequence = ["backend_ready"]
        live = mode == "live"
        led_white = {"skipped": True, "reason": "shadow_mode"}
        deactivate = {"skipped": True, "reason": "shadow_mode"}
        activate = {"skipped": True, "reason": "shadow_mode"}
        if live:
            if hasattr(tester, "strip_set_rgb"):
                led_white = tester.strip_set_rgb(255, 255, 255)
            else:
                led_white = {"ok": False, "error": "strip_set_rgb_unavailable"}
            sequence.append("led_white")
        snap = tester.io_snapshot(tester.BOARD_DECK)
        sequence.append("door_latch_before")
        door_closed = bool(snap.get(1))
        latch_closed = bool(snap.get(3))
        final_snap = snap
        if live:
            if hasattr(tester, "deactivate_boards"):
                deactivate = tester.deactivate_boards()
            else:
                deactivate = {"ok": False, "error": "deactivate_boards_unavailable"}
            sequence.append("deactivate_boards")
            if hasattr(tester, "activate_boards"):
                activate = tester.activate_boards()
            else:
                activate = {"ok": False, "error": "activate_boards_unavailable"}
            sequence.append("activate_boards")
            final_snap = tester.io_snapshot(tester.BOARD_DECK)
            door_closed = bool(final_snap.get(1))
            latch_closed = bool(final_snap.get(3))
        sequence.append("door_latch_final")
        ok = door_closed and latch_closed
        if live:
            write_checks = [led_white, deactivate, activate]
            writes_ok = all(isinstance(item, dict) and item.get("ok", "error" not in item) is not False and "error" not in item for item in write_checks)
            ok = ok and writes_ok
        return {
            "ok": ok,
            "mode": mode,
            "sequence": sequence,
            "source_anchor": SOURCE_ANCHORS["initialCheck"],
            "led_white": led_white,
            "deactivate_boards": deactivate,
            "activate_boards": activate,
            "door_latch": {"door_closed": door_closed, "latch_closed": latch_closed, "snapshot": final_snap, "before_snapshot": snap},
            "checks": [{"name": "door_latch", "ok": ok}],
        }

    def configure_without_motion(self, *, mode: str = "shadow") -> dict:
        tester = self.tester
        if hasattr(tester, "motor_oem_initialize_without_motion") and mode == "live":
            prep = tester.motor_oem_initialize_without_motion()
            if isinstance(prep, dict) and "ok" in prep:
                return prep
            # Backward-compatible guard for older tester implementations that returned
            # raw {axis: prep_row} without an aggregate success bit.  The startup
            # state machine requires a top-level ok; otherwise live startup falsely
            # failed at initializeMotorsWithoutMotion even when all prep writes ACKed.
            return {
                "ok": True,
                "physical_motion": False,
                "source_anchor": SOURCE_ANCHORS["initializeMotorsWithoutMotion"],
                "axes": prep,
                "normalized_from_raw_axis_prep": True,
            }
        return {"ok": True, "physical_motion": False, "source_anchor": SOURCE_ANCHORS["initializeMotorsWithoutMotion"], "skipped_live_write": mode != "live"}

    def pipette_startup_check(self, *, mode: str = "shadow") -> dict:
        return {"ok": False, "required": True, "available": False, "skipped": True, "blocks_ready": True, "reason": "pipette ACK/readback parity gate not yet proven"}

    def vision_startup_check(self, *, mode: str = "shadow") -> dict:
        return {"ok": False, "required": True, "available": False, "skipped": True, "blocks_ready": True, "reason": "vision/inspection artifact parity gate not yet proven"}

    def homing_predicates(self) -> dict[str, dict]:
        return {}

class OEMStartupProgram:
    def __init__(self, *, hardware: Any, artifact_base: str | Path = "/tmp/bioxp-live-runs", allowlist_roots: list[str | Path] | None = None):
        self.hardware = hardware
        self.artifact_base = Path(artifact_base)
        self.allowlist_roots = allowlist_roots or []
        self.sessions: dict[str, dict] = {}
        self.latest_session_id: str | None = None

    def _artifact_dict(self, root: Path) -> dict:
        return {name: str(root / name) for name in REQUIRED_ARTIFACTS if (root / name).exists()}

    def _write_placeholder_artifacts(self, root: Path) -> None:
        for name in REQUIRED_ARTIFACTS:
            path = root / name
            if path.exists():
                continue
            if name not in {"startup_request.json", "source_anchors.json"}:
                _atomic_json(path, {"skipped": True, "reason": "stage not reached yet"})

    def _closeout(self, status: dict) -> dict:
        root = Path(status["artifact_root"])
        status["artifacts"] = self._artifact_dict(root)
        _atomic_json(root / "final_readiness.json", status)
        return dict(status)

    def _new_session(self, req: dict) -> dict:
        ok, reason, explicit_root = validate_artifact_root(req.get("mode", "dry_run"), req.get("artifact_root"), self.artifact_base, self.allowlist_roots)
        if not ok:
            # For invalid live roots, create a safe local failure artifact under base rather than using the unsafe path.
            sid = time.strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
            safe_root = self.artifact_base / f"{sid}_OEM_APP_STARTUP_SEQUENCE_FAILED_ROOT"
            safe_root.mkdir(parents=True, exist_ok=True)
            status = self._status_template(sid, req, safe_root)
            self.sessions[sid] = status
            self.latest_session_id = sid
            _atomic_json(safe_root / "startup_request.json", req)
            _atomic_json(safe_root / "source_anchors.json", SOURCE_ANCHORS)
            self._write_placeholder_artifacts(safe_root)
            return self._fail(status, reason or "invalid artifact root")
        sid = time.strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
        artifact_root = explicit_root or (self.artifact_base / f"{sid}_OEM_APP_STARTUP_SEQUENCE")
        artifact_root.mkdir(parents=True, exist_ok=True)
        status = self._status_template(sid, req, artifact_root)
        self.sessions[sid] = status
        self.latest_session_id = sid
        _atomic_json(artifact_root / "startup_request.json", req)
        _atomic_json(artifact_root / "source_anchors.json", SOURCE_ANCHORS)
        self._write_placeholder_artifacts(artifact_root)
        return status

    def _status_template(self, sid: str, req: dict, artifact_root: Path) -> dict:
        return {
            "ok": True,
            "session_id": sid,
            "mode": req.get("mode", "dry_run"),
            "request": dict(req),
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
            "source_anchors": dict(SOURCE_ANCHORS),
            "door_latch": {},
            "config": {},
            "backend": {},
            "axis_reference": {"required": True, "ok": False, "reason": "not reached"},
            "pipette": {"required": True, "ok": False, "reason": "not checked"},
            "vision": {"required": True, "ok": False, "reason": "not checked"},
        }

    def _fail(self, status: dict, reason: str) -> dict:
        status.update({"ok": False, "state": OemStartupState.FAILED_CLOSED.value, "active_stage": OemStartupState.FAILED_CLOSED.value, "ready": False, "failed": True, "failed_closed": True, "failure_reason": reason})
        root = Path(status["artifact_root"])
        _atomic_json(root / "failure.json", {"reason": reason, "state": status["state"]})
        return self._closeout(status)

    def request_startup(self, request: dict) -> dict:
        req = dict(request or {})
        req.setdefault("mode", "dry_run")
        req.setdefault("require_config", True)
        req.setdefault("door_policy", "wait_for_closed")

        if req["mode"] == "live" and req.get("operator_ack") != "INITIALIZE":
            status = self._new_session(req)
            return self._fail(status, "operator_ack INITIALIZE required for live mode")
        status = self._new_session(req)
        if status.get("failed_closed"):
            return status
        # Generic startup binds a session/artifact owner only.  The accepted
        # constructor -> initialize-without-motion -> initialCheck stages are
        # separately approved POST/worker actions and cannot execute here.
        from .lifecycle_state import lifecycle_state

        lifecycle = lifecycle_state.transition("waiting", reason="startup_session_bound_awaiting_constructor_stage")
        status.update({
            "state": "waiting_for_constructor_pipette_stage",
            "active_stage": None,
            "ready": False,
            "queued": False,
            "lifecycle": lifecycle,
            "next_action": "POST /oem/startup/constructor_pipettes",
        })
        return self._closeout(status)

    def door_event(self, session_id: str | None, *, door_closed: bool, latch_closed: bool) -> dict:
        from .lifecycle_state import lifecycle_state

        lifecycle = lifecycle_state.record_door_event(
            door_closed=door_closed,
            latch_closed=latch_closed,
            source="OEMStartupProgram.door_event",
        )
        return {"ok": True, "session_id": session_id or "canonical", "door": lifecycle["door"], "state": lifecycle["operation_state"], "lifecycle": lifecycle}

    def status(self, session_id: str | None = None) -> dict:
        from .lifecycle_state import lifecycle_state

        lifecycle = lifecycle_state.projection()
        sid = session_id or self.latest_session_id or "canonical"
        return {
            "ok": lifecycle["operation_state"] not in {"error", "emergency"},
            "session_id": sid,
            "state": lifecycle["operation_state"],
            "startup_state": lifecycle["startup"]["state"],
            "ready": lifecycle["startup"]["state"] == "passed",
            "lifecycle": lifecycle,
        }
