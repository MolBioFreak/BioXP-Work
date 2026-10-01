from __future__ import annotations

import json
from pathlib import Path
from typing import Any


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


    def pipette_startup_check(self, *, mode: str = "dry_run") -> dict:
        return {"ok": not self.pipette_required, "required": self.pipette_required, "available": False, "skipped": True, "blocks_ready": self.pipette_required, "reason": "ClassPipetteCollection startup parity not live-bound in this shell"}



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


    def pipette_startup_check(self, *, mode: str = "shadow") -> dict:
        return {"ok": False, "required": True, "available": False, "skipped": True, "blocks_ready": True, "reason": "pipette ACK/readback parity gate not yet proven"}



class OEMStartupProgram:
    def __init__(self, *, hardware: Any, artifact_base: str | Path = "/tmp/bioxp-live-runs", allowlist_roots: list[str | Path] | None = None):
        self.hardware = hardware
        self.artifact_base = Path(artifact_base)
        self.allowlist_roots = allowlist_roots or []
        self.sessions: dict[str, dict] = {}
        self.latest_session_id: str | None = None


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
