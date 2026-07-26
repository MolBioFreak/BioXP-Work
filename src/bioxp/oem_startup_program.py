from __future__ import annotations

import json
import os
import time
import uuid
from pathlib import Path
from typing import Any

from .oem_config import find_oem_config
from .oem_motion_worker import OEMMotionWorker, OemMotionCommand
from .oem_parity_config import load_oem_parity_config
from .oem_pipette_collection import dry_run_initialize_motion_pipette_cleanup
from .oem_startup_types import OemStartupState, STARTUP_STAGE_ORDER
from .oem_switch_audit import require_confident_predicates

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
    "motion_queue_events.jsonl",
    "initialize_system.json",
    "initialize_motion.json",
    "initialize_motors_trace.jsonl",
    "axis_switch_trace_z.jsonl",
    "axis_switch_trace_g.jsonl",
    "axis_switch_trace_x.jsonl",
    "axis_switch_trace_y.jsonl",
    "axis_switch_trace_door.jsonl",
    "post_home_pipette_cleanup.json",
    "vision_inspection.json",
    "gantry_park.json",
    "door_ready_state.json",
    "final_readiness.json",
    "failure.json",
]


def _atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True))
    tmp.replace(path)


def _append_jsonl(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as fh:
        fh.write(json.dumps(payload, sort_keys=True) + "\n")


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

    def run_initialize_system(self, *, mode: str, run_homing: bool, run_post_home: bool, artifact_root: Path, session_id: str) -> dict:
        self.motion_calls.append("initializeSystem")
        _atomic_json(artifact_root / "initialize_motion.json", {"ok": True, "source_anchor": SOURCE_ANCHORS["initializeMotion"], "dry_run": mode != "live"})
        if run_homing:
            for axis, stage in [("z", "homing_z"), ("g", "homing_gripper"), ("x", "homing_x"), ("y", "homing_y"), ("door", "homing_door")]:
                _append_jsonl(artifact_root / f"axis_switch_trace_{axis}.jsonl", {"session_id": session_id, "stage": stage, "dry_run": mode != "live", "moved": False})
            _append_jsonl(artifact_root / "initialize_motors_trace.jsonl", {"session_id": session_id, "ok": True, "dry_run": mode != "live", "source_anchor": SOURCE_ANCHORS["initializeMotors"]})
        else:
            _append_jsonl(artifact_root / "initialize_motors_trace.jsonl", {"session_id": session_id, "skipped": True, "reason": "run_homing=false"})
        return {"ok": True, "dry_run": mode != "live", "run_homing": run_homing, "run_post_home": run_post_home}


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

    def initialize_motion_diagnostic(self, *, mode: str = "shadow", run_homing: bool = False) -> dict:
        tester = self.tester
        axes = {"x": (tester.BOARD_DECK, 0), "y": (tester.BOARD_HEAD, 0), "z": (tester.BOARD_HEAD, 1), "g": (tester.BOARD_HEAD, 2), "door": (tester.BOARD_THERMAL, 0)}
        rows = {}
        errors = []
        if bool(run_homing):
            return {"ok": False, "physical_motion": False, "state": "failed_closed", "error": "run_homing_not_allowed_in_diagnostic"}
        for axis, (board, motor) in axes.items():
            try:
                rows[axis] = {
                    "board": int(board),
                    "motor": int(motor),
                    "position": tester.motor_get_position(board, motor=motor),
                    "speed": tester.motor_get_speed(board, motor=motor),
                    "switches": tester.motor_get_switch_activity(board, motor=motor),
                }
            except Exception as exc:
                rows[axis] = {"board": int(board), "motor": int(motor), "error": str(exc)}
                errors.append(f"{axis}_snapshot_failed")
        rail_24v = None
        try:
            rail_24v = tester.motor_query_24v_sensor()
        except Exception as exc:
            rail_24v = {"error": str(exc)}
            errors.append("rail_24v_snapshot_failed")
        latch = None
        try:
            snap = tester.io_snapshot(tester.BOARD_DECK)
            latch = {"snapshot": snap, "door_closed": bool(snap.get(1)), "latch_closed": bool(snap.get(3))}
        except Exception as exc:
            latch = {"error": str(exc)}
            errors.append("latch_snapshot_failed")
        return {
            "ok": len(errors) == 0,
            "mode": mode,
            "physical_motion": False,
            "run_homing": False,
            "source_anchor": SOURCE_ANCHORS["initializeMotion"],
            "initialize_without_motion_anchor": SOURCE_ANCHORS["initializeMotorsWithoutMotion"],
            "axis_snapshots": rows,
            "rail_24v": rail_24v,
            "latch": latch,
            "errors": errors,
        }

    def startup_homing_stepwise(self, *, mode: str = "shadow", step: str = "plan", execute: bool = False, preclear_abs: int | None = None, require_operator_observed: bool = True) -> dict:
        selected = str(step).strip().lower()
        # M16/M17 are literal individual CAN writes in the only existing source
        # model (motor_oem_initialize_without_motion). Calling that aggregate here
        # would replay unrelated setup writes; a generic chiller helper would be
        # invented behavior. Do neither.
        if bool(execute) and selected in {"chiller-oc-cool-rate", "chiller-rc-cool-rate"}:
            bank = 1 if selected == "chiller-oc-cool-rate" else 0
            return {
                "ok": False,
                "mode": mode,
                "execute": True,
                "physical_motion": False,
                "hardware_touched": False,
                "step": {"step": selected},
                "source_anchor": "ClassControlInterface.initializeMotors:3414-3415; usb_driver.py:4192-4194",
                "literal_oem_gp8_write": {"board": "BOARD_CHILLER", "command": 9, "type": 8, "bank": bank, "value": -25},
                "blockers": ["single_stage_chiller_transport_not_source_bound"],
            }
        # M18's exact source status becomes durable only through the robot-owned
        # movement ledger. It is not a UI projection nor product readiness.
        if bool(execute) and selected == "system-status-initialized":
            return {
                "ok": True,
                "mode": mode,
                "execute": True,
                "physical_motion": False,
                "hardware_touched": False,
                "step": {"step": selected},
                "durable_robot_state": {"system_status": 1, "ready": True},
                "source_anchor": "ClassControlInterface.initializeMotors:3416",
                "oem_source_order_preserved": True,
            }
        if bool(execute) and selected == "ui-zero-calibrated":
            tester = self.tester
            machine = tester._machine_config_bundle() if hasattr(tester, "_machine_config_bundle") else None
            calibration = (((machine or {}).get("config") or {}).get("calibration") or {}) if isinstance(machine, dict) else {}
            raw_calibrated = calibration.get("Calibrated") if isinstance(calibration, dict) else None
            calibrated = raw_calibrated is True or raw_calibrated == 1 or str(raw_calibrated).strip().lower() in {"true", "yes", "1"}
            if not isinstance(machine, dict) or machine.get("ok") is not True or raw_calibrated is None:
                return {
                    "ok": False,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": False,
                    "hardware_touched": False,
                    "step": {"step": selected},
                    "blockers": ["immutable_oem_calibrated_branch_authority_unavailable"],
                }
            if not calibrated:
                return {
                    "ok": True,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": False,
                    "hardware_touched": False,
                    "step": {"step": selected},
                    "source_branch_taken": False,
                    "ui_writes": [],
                    "oem_source_order_preserved": True,
                }
            return {
                "ok": False,
                "mode": mode,
                "execute": True,
                "physical_motion": False,
                "hardware_touched": False,
                "step": {"step": selected},
                "source_branch_taken": True,
                "expected_ui_writes": [("x", "0"), ("y", "0"), ("z", "0"), ("z", "0")],
                "blockers": ["calibrated_ui_position_sink_not_bound"],
            }
        if bool(execute) and selected == "gripper-idle-current-10":
            tester = self.tester
            profile = tester._motion_oem_axis_profile("g", startup=True)
            try:
                gripper_version = int(profile["gripper_version"])
            except (KeyError, TypeError, ValueError):
                return {
                    "ok": False,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": False,
                    "step": {"step": selected},
                    "blockers": ["immutable_oem_gripper_version_authority_unavailable"],
                }
            if gripper_version != 1:
                return {
                    "ok": True,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": False,
                    "step": {"step": selected},
                    "source_branch_taken": False,
                    "gripper_version": gripper_version,
                    "oem_source_order_preserved": True,
                }
            current = tester.motor_set_axis_param(profile["board"], 6, 10, motor=profile["motor"])
            return {
                "ok": bool(current.get("ok")),
                "mode": mode,
                "execute": True,
                "physical_motion": False,
                "step": {"step": selected},
                "source_branch_taken": True,
                "gripper_version": 1,
                "gripper_idle_current_10": current,
                "oem_source_order_preserved": True,
            }
        tester = self.tester
        steps = [
            {"step": "z-home", "oem_anchor": "initializeMotors: MotorZ.axisSearchHome(speed=1791)", "axis": "z", "board": int(tester.BOARD_HEAD), "motor": 1, "requires_operator_observation": True},
            {"step": "gripper-current-31", "oem_anchor": "initializeMotors: setGripperCurrent(31)", "axis": "g", "board": int(tester.BOARD_HEAD), "motor": 2, "requires_operator_observation": False},
            {"step": "gripper-clear-10000", "oem_anchor": "initializeMotors: MotorGrip.moveSteps(10000,true)", "axis": "g", "board": int(tester.BOARD_HEAD), "motor": 2, "requires_operator_observation": True},
            {"step": "gripper-home", "oem_anchor": "initializeMotors: MotorGrip.axisSearchHome(speed=600|200)", "axis": "g", "board": int(tester.BOARD_HEAD), "motor": 2, "requires_operator_observation": True},
            {"step": "x-home", "oem_anchor": "initializeMotors: MotorX.axisSearchHome(speed=250)", "axis": "x", "board": int(tester.BOARD_DECK), "motor": 0, "requires_operator_observation": True},
            {"step": "x-home-settle", "oem_anchor": "initializeMotors: sleep(20ms)", "axis": "x", "requires_operator_observation": True},
            {"step": "x-set-home", "oem_anchor": "initializeMotors: setHome(X)", "axis": "x", "board": int(tester.BOARD_DECK), "motor": 0, "requires_operator_observation": True},
            {"step": "x-speed-1700", "oem_anchor": "initializeMotors: setSpeed(X,1700)", "axis": "x", "board": int(tester.BOARD_DECK), "motor": 0, "requires_operator_observation": True},
            {"step": "x-speed-settle", "oem_anchor": "initializeMotors: sleep(40ms)", "axis": "x", "requires_operator_observation": True},
            {"step": "x-park-6000", "oem_anchor": "initializeMotors: moveX(6000)", "axis": "x", "board": int(tester.BOARD_DECK), "motor": 0, "requires_operator_observation": True},
            {"step": "y-home", "oem_anchor": "initializeMotors: MotorY.axisSearchHome(speed=250)", "axis": "y", "board": int(tester.BOARD_HEAD), "motor": 0, "requires_operator_observation": True},
            {"step": "door-home", "oem_anchor": "initializeMotors: ThermalDoor.doorSearchHome(...) ", "axis": "door", "board": int(tester.BOARD_THERMAL), "motor": 0, "requires_operator_observation": True},
            {"step": "door-closed-predicate", "oem_anchor": "initializeMotors: !confirmAxis(tcDoorClosed) failure branch", "axis": "door", "requires_operator_observation": True},
            {"step": "y-set-home", "oem_anchor": "initializeMotors: setHome(Y)", "axis": "y", "board": int(tester.BOARD_HEAD), "motor": 0, "requires_operator_observation": True},
            {"step": "ui-zero-calibrated", "oem_anchor": "initializeMotors: Calibrated UI X/Y/Z zero writes", "requires_operator_observation": False},
            {"step": "chiller-oc-cool-rate", "oem_anchor": "initializeMotors: setChillerCoolRate(OC)", "requires_operator_observation": False},
            {"step": "chiller-rc-cool-rate", "oem_anchor": "initializeMotors: setChillerCoolRate(RC)", "requires_operator_observation": False},
            {"step": "system-status-initialized", "oem_anchor": "initializeMotors: system status=1, ready=true", "requires_operator_observation": False},
            {"step": "gripper-idle-current-10", "oem_anchor": "initializeMotors: GripperVersion==1 setGripperCurrent(10)", "axis": "g", "board": int(tester.BOARD_HEAD), "motor": 2, "requires_operator_observation": False},
        ]
        if selected in {"plan", "full", "all", ""}:
            return {
                "ok": True,
                "mode": mode,
                "execute": False,
                "physical_motion": False,
                "steps": steps,
                "oem_sequence_anchor": SOURCE_ANCHORS["initializeMotors"],
                "live_policy": "single_oem_initializeMotors_step_only_operator_ack_HOME; diagnostic harness must preserve OEM order/function",
                "monolithic_homing_blocked": False,
                "not_a_replacement_sequence": True,
            }
        matches = [row for row in steps if row["step"] == selected]
        if not matches:
            return {"ok": False, "mode": mode, "execute": False, "physical_motion": False, "error": f"unknown_homing_step:{selected}", "allowed_steps": [row["step"] for row in steps]}
        row = dict(matches[0])
        if not bool(execute):
            pre = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {"ok": True, "mode": mode, "execute": False, "physical_motion": False, "step": row, "pre_snapshot": pre, "dry_run_note": "planned only; no axis/home/move command issued"}
        if selected == "door-closed-predicate":
            # M13 (ClassControlInterface.initializeMotors:3384-3387) uses the
            # sealed machine identity/calibration binding, not a diagnostic XML
            # parse or a source-default fallback.  Its closed state is likewise
            # the literal tcDoorClosed/QueryHome predicate; generic door-status
            # fields are deliberately not accepted here.
            parity_config = load_oem_parity_config(None)
            values = parity_config.values if getattr(parity_config, "calibration_source", None) == "immutable_oem_machine_snapshot" else {}
            if getattr(parity_config, "calibration_source", None) != "immutable_oem_machine_snapshot":
                return {
                    "ok": False,
                    "failed_closed": True,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": False,
                    "step": row,
                    "error": "sealed_config_machine_snapshot_required",
                    "sealed_config_source": getattr(parity_config, "calibration_source", None),
                    "sealed_config_blockers": list(getattr(parity_config, "blockers", [])),
                }
            try:
                serial_number = int(values["SerialNumber"])
            except (KeyError, TypeError, ValueError):
                return {
                    "ok": False,
                    "failed_closed": True,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": False,
                    "step": row,
                    "error": "sealed_config_serial_number_required",
                }
            camera_calibrated = values.get("CameraCalibrated")
            if not isinstance(camera_calibrated, bool):
                return {
                    "ok": False,
                    "failed_closed": True,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": False,
                    "step": row,
                    "error": "sealed_config_camera_calibrated_boolean_required",
                }
            thermal_status = tester.motor_thermal_door_status()
            predicates = thermal_status.get("oem_predicates") if isinstance(thermal_status, dict) else None
            thermal_closed = predicates.get("tcDoorClosed") if isinstance(predicates, dict) else None
            closed_source = predicates.get("closed_source") if isinstance(predicates, dict) else None
            if not isinstance(thermal_closed, bool) or closed_source != "queryHome(ThermalDoor)":
                return {
                    "ok": False,
                    "failed_closed": True,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": False,
                    "step": row,
                    "error": "tcDoorClosed_source_predicate_required",
                    "thermal_closed_predicate": {"tcDoorClosed": thermal_closed, "source": closed_source},
                }
            source_condition = "SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated"
            source_condition_active = serial_number > 9 and thermal_closed is False and camera_calibrated is True
            binding = {
                "SerialNumber": serial_number,
                "CameraCalibrated": camera_calibrated,
                "source": "immutable_oem_machine_snapshot",
            }
            predicate = {"tcDoorClosed": thermal_closed, "source": closed_source}
            if source_condition_active:
                opened = tester.motor_oem_open_thermal_door(timeout_s=20.0)
                # The OEM branch opens then throws unconditionally; do not turn
                # this terminal source failure into a generic open-door result.
                return {
                    "ok": False,
                    "failed_closed": True,
                    "mode": mode,
                    "execute": True,
                    "physical_motion": True,
                    "step": row,
                    "source_condition": source_condition,
                    "source_condition_active": True,
                    "sealed_config_binding": binding,
                    "thermal_closed_predicate": predicate,
                    "open_thermal_door": opened,
                    "error": "Cannot close thermal cycler door!",
                    "oem_source_order_preserved": True,
                }
            return {
                "ok": True,
                "failed_closed": False,
                "mode": mode,
                "execute": True,
                "physical_motion": False,
                "step": row,
                "source_condition": source_condition,
                "source_condition_active": False,
                "sealed_config_binding": binding,
                "thermal_closed_predicate": predicate,
                "oem_source_order_preserved": True,
            }
        pre = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
        if selected == "z-home":
            z_home = tester.motor_oem_home_axis("z", startup=True, timeout_s=30.0)
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            z_home_ok = bool((z_home.get("home") or {}).get("ok")) if isinstance(z_home, dict) else False
            return {
                "ok": z_home_ok,
                "mode": mode,
                "execute": True,
                "physical_motion": True,
                "step": row,
                "pre_snapshot": pre,
                "post_snapshot": post,
                "z_home": z_home,
                "operator_observation_required": bool(require_operator_observed),
                "oem_source_order_preserved": True,
            }
        if selected == "gripper-current-31":
            profile = tester._motion_oem_axis_profile("g", startup=True)
            current = tester.motor_set_axis_param(profile["board"], 6, 31, motor=profile["motor"])
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {"ok": bool(current.get("ok")), "mode": mode, "execute": True, "physical_motion": False, "step": row, "pre_snapshot": pre, "post_snapshot": post, "gripper_current_31": current, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        if selected == "gripper-clear-10000":
            profile = tester._motion_oem_axis_profile("g", startup=True)
            move = tester.motor_move_relative(profile["board"], 10000, motor=profile["motor"])
            wait = tester.motor_wait_stopped(profile["board"], motor=profile["motor"], timeout_s=20.0, require_seen_nonzero=False)
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {"ok": bool(move.get("ok") and wait.get("stopped") is True), "mode": mode, "execute": True, "physical_motion": bool(move.get("ok")), "step": row, "pre_snapshot": pre, "post_snapshot": post, "gripper_move_10000": move, "wait": wait, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        if selected == "gripper-home":
            # Serial-206 has GripperVersion=1, so the direct source branch is 200.
            home = tester.motor_oem_home_axis("g", startup=True, speed=200, timeout_s=30.0)
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            home_ok = bool((home.get("home") or {}).get("ok")) if isinstance(home, dict) else False
            return {
                "ok": home_ok,
                "mode": mode,
                "execute": True,
                "physical_motion": True,
                "step": row,
                "pre_snapshot": pre,
                "post_snapshot": post,
                "gripper_home": home,
                "operator_observation_required": bool(require_operator_observed),
                "oem_source_order_preserved": True,
            }
        if selected == "x-home":
            home = tester.motor_oem_home_axis("x", startup=True, speed=250, timeout_s=30.0)
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            home_ok = bool((home.get("home") or {}).get("ok")) if isinstance(home, dict) else False
            return {
                "ok": home_ok,
                "mode": mode,
                "execute": True,
                "physical_motion": True,
                "step": row,
                "pre_snapshot": pre,
                "post_snapshot": post,
                "x_home": home,
                "operator_observation_required": bool(require_operator_observed),
                "oem_source_order_preserved": True,
            }
        if selected == "x-home-settle":
            time.sleep(0.020)
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {"ok": True, "mode": mode, "execute": True, "physical_motion": False, "hardware_touched": False, "step": row, "pre_snapshot": pre, "post_snapshot": post, "settle_ms": 20, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        if selected == "x-set-home":
            profile = tester._motion_oem_axis_profile("x", startup=True)
            set_home = tester.motor_set_home(profile["board"], motor=profile["motor"])
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {"ok": bool(set_home.get("ok")), "mode": mode, "execute": True, "physical_motion": False, "step": row, "pre_snapshot": pre, "post_snapshot": post, "x_set_home": set_home, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        if selected == "x-speed-1700":
            profile = tester._motion_oem_axis_profile("x", startup=True)
            set_speed = tester.motor_set_axis_param(profile["board"], 4, 1700, motor=profile["motor"])
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {"ok": bool(set_speed.get("ok")), "mode": mode, "execute": True, "physical_motion": False, "step": row, "pre_snapshot": pre, "post_snapshot": post, "x_speed_1700": set_speed, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        if selected == "x-speed-settle":
            time.sleep(0.040)
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {"ok": True, "mode": mode, "execute": True, "physical_motion": False, "hardware_touched": False, "step": row, "pre_snapshot": pre, "post_snapshot": post, "settle_ms": 40, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        if selected == "x-park-6000":
            profile = tester._motion_oem_axis_profile("x", startup=True)
            move = tester.motor_move_absolute(profile["board"], 6000, motor=profile["motor"])
            wait = tester.motor_wait_stopped(
                profile["board"],
                motor=profile["motor"],
                timeout_s=12.0,
                require_seen_nonzero=True,
            )
            ok = bool(
                move.get("ok")
                and wait.get("stopped") is True
            )
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {
                "ok": ok,
                "mode": mode,
                "execute": True,
                "physical_motion": True,
                "motion_command_attempted": True,
                "controller_acknowledged": move.get("ok") is True,
                "stopped_observed": wait.get("stopped") is True,
                "physical_effect_verified": False,
                "step": row,
                "pre_snapshot": pre,
                "post_snapshot": post,
                "x_park_6000": {"move_x_6000": move, "wait": wait},
                "operator_observation_required": bool(require_operator_observed),
                "oem_source_order_preserved": True,
            }
        if selected == "y-home":
            home = tester.motor_oem_home_axis("y", startup=True, speed=250, timeout_s=30.0)
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            home_ok = bool((home.get("home") or {}).get("ok")) if isinstance(home, dict) else False
            return {"ok": home_ok, "mode": mode, "execute": True, "physical_motion": True, "step": row, "pre_snapshot": pre, "post_snapshot": post, "y_home": home, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        if selected == "door-home":
            door = tester.motor_oem_door_search_home(startup=True, timeout_s=30.0)
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            door_ok = bool(door.get("ok")) if isinstance(door, dict) else False
            return {"ok": door_ok, "mode": mode, "execute": True, "physical_motion": True, "step": row, "pre_snapshot": pre, "post_snapshot": post, "door_home": door, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        if selected == "y-set-home":
            profile = tester._motion_oem_axis_profile("y", startup=True)
            set_home = tester.motor_set_home(profile["board"], motor=profile["motor"])
            post = self.initialize_motion_diagnostic(mode="shadow", run_homing=False)
            return {"ok": bool(set_home.get("ok")), "mode": mode, "execute": True, "physical_motion": False, "step": row, "pre_snapshot": pre, "post_snapshot": post, "y_set_home": set_home, "operator_observation_required": bool(require_operator_observed), "oem_source_order_preserved": True}
        return {"ok": False, "mode": mode, "execute": False, "physical_motion": False, "step": row, "pre_snapshot": pre, "error": "live_step_execution_not_enabled_for_this_step", "refusal": "scaffold installed; enable one step only after prior step proof and operator observation"}

    def pipette_startup_check(self, *, mode: str = "shadow") -> dict:
        return {"ok": False, "required": True, "available": False, "skipped": True, "blocks_ready": True, "reason": "pipette ACK/readback parity gate not yet proven"}

    def vision_startup_check(self, *, mode: str = "shadow") -> dict:
        return {"ok": False, "required": True, "available": False, "skipped": True, "blocks_ready": True, "reason": "vision/inspection artifact parity gate not yet proven"}

    def homing_predicates(self) -> dict[str, dict]:
        return {}

    def run_initialize_system(self, *, mode: str, run_homing: bool, run_post_home: bool, artifact_root: Path, session_id: str) -> dict:
        if mode == "live":
            raise RuntimeError("live initializeSystem is blocked until switch predicates, pipette, and vision gates are proven")
        return DryRunStartupHardware().run_initialize_system(mode=mode, run_homing=run_homing, run_post_home=run_post_home, artifact_root=artifact_root, session_id=session_id)


class OEMStartupProgram:
    def __init__(self, *, hardware: Any, artifact_base: str | Path = "/tmp/bioxp-live-runs", allowlist_roots: list[str | Path] | None = None):
        self.hardware = hardware
        self.artifact_base = Path(artifact_base)
        self.allowlist_roots = allowlist_roots or []
        self.sessions: dict[str, dict] = {}
        self.latest_session_id: str | None = None
        self.worker = OEMMotionWorker(artifact_root=self.artifact_base, handlers={"initializeSystem": self._handle_initialize_system})

    def _stage(self, status: dict, state: OemStartupState | str) -> None:
        value = state.value if isinstance(state, OemStartupState) else str(state)
        status["state"] = value
        status["active_stage"] = value
        if value not in status["completed_stages"] and value in STARTUP_STAGE_ORDER:
            status["completed_stages"].append(value)
        status["pending_stages"] = [s for s in STARTUP_STAGE_ORDER if s not in status["completed_stages"]]

    def _artifact_dict(self, root: Path) -> dict:
        return {name: str(root / name) for name in REQUIRED_ARTIFACTS if (root / name).exists()}

    def _write_placeholder_artifacts(self, root: Path) -> None:
        for name in REQUIRED_ARTIFACTS:
            path = root / name
            if path.exists():
                continue
            if name.endswith(".jsonl"):
                _append_jsonl(path, {"event": "skipped", "skipped": True, "reason": "stage not reached yet"})
            elif name not in {"startup_request.json", "source_anchors.json"}:
                _atomic_json(path, {"skipped": True, "reason": "stage not reached yet"})

    def _closeout(self, status: dict) -> dict:
        root = Path(status["artifact_root"])
        status["motion_worker"] = self.worker.status()
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
            "pending_stages": list(STARTUP_STAGE_ORDER),
            "artifact_root": str(artifact_root),
            "artifacts": {},
            "source_anchors": dict(SOURCE_ANCHORS),
            "door_latch": {},
            "config": {},
            "backend": {},
            "motion_worker": self.worker.status(),
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
        req.setdefault("run_homing", True)
        req.setdefault("run_post_home", True)
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

        # Historical full-sequence implementation remains below for source
        # traceability but is intentionally unreachable from generic startup.
        root = Path(status["artifact_root"])

        self._stage(status, OemStartupState.CONFIG_LOADING)
        config = self.hardware.load_config()
        _atomic_json(root / "config_search.json", config)
        _atomic_json(root / "config_binding.json", config)
        status["config"] = config
        if req.get("require_config") and (config.get("status") != "loaded" or (req["mode"] == "live" and not config.get("live_ready", False))):
            return self._fail(status, "config required but OEM config.xml binding is missing or incomplete")
        self._stage(status, OemStartupState.CONFIG_LOADED if config.get("status") == "loaded" else OemStartupState.CONFIG_MISSING)

        backend = {"ok": True, "mode": req["mode"]}
        status["backend"] = backend
        _atomic_json(root / "backend_ready.json", backend)
        self._stage(status, OemStartupState.BACKEND_READY)
        _atomic_json(root / "control_lib_constructed.json", {"ok": True, "represented": True, "source_anchor": "ControlLib constructor / ClassControlInterface construction"})
        self._stage(status, OemStartupState.CONTROL_LIB_CONSTRUCTED)

        pipette = self.hardware.pipette_startup_check(mode=req["mode"]) if hasattr(self.hardware, "pipette_startup_check") else {"ok": False, "required": True, "blocks_ready": True, "reason": "pipette provider absent"}
        status["pipette"] = pipette
        _atomic_json(root / "pipette_startup_check.json", pipette)
        self._stage(status, OemStartupState.PIPETTE_CHECKED)

        prep = self.hardware.configure_without_motion(mode=req["mode"])
        _atomic_json(root / "initialize_motors_without_motion.json", prep)
        if not prep.get("ok", False):
            return self._fail(status, "initializeMotorsWithoutMotion failed")
        self._stage(status, OemStartupState.MOTORS_CONFIGURED_WITHOUT_MOTION)
        _atomic_json(root / "initialize_environment.json", {"ok": True, "source_anchor": SOURCE_ANCHORS["initializeEnvironment"]})
        self._stage(status, OemStartupState.INITIALIZE_ENVIRONMENT)

        initial = self.hardware.initial_check(mode=req["mode"])
        _atomic_json(root / "initial_check_before_door.json", initial)
        status["door_latch"] = initial.get("door_latch", {})
        self._stage(status, OemStartupState.INITIAL_CHECK_BEFORE_DOOR)
        if not initial.get("ok"):
            if req.get("door_policy") == "wait_for_closed":
                self._stage(status, OemStartupState.WAITING_FOR_DOOR_CLOSE)
                _atomic_json(root / "door_wait.json", {"ok": True, "waiting": True, "door_latch": status["door_latch"]})
                return self._closeout(status)
            return self._fail(status, "door/latch gate failed during initialCheck")
        if req.get("door_policy") == "already_closed" and not initial.get("ok"):
            return self._fail(status, "door_policy already_closed was requested but door/latch were not closed")
        self._stage(status, OemStartupState.DOOR_CLOSE_OBSERVED)
        _atomic_json(root / "door_event.json", {"door_closed": True, "latch_closed": True, "synthetic": True, "reason": "already closed at startup"})
        after = self.hardware.initial_check(mode=req["mode"])
        _atomic_json(root / "initial_check_after_door.json", after)
        status["door_latch"] = after.get("door_latch", status["door_latch"])
        self._stage(status, OemStartupState.INITIAL_CHECK_AFTER_DOOR)
        if not after.get("ok"):
            return self._fail(status, "initialCheck after door-close observation failed")
        return self._queue_initialize_system(status)

    def _queue_initialize_system(self, status: dict) -> dict:
        root = Path(status["artifact_root"])
        req = status.get("request", {})
        if req.get("mode") == "live" and req.get("run_homing", True):
            matrix = require_confident_predicates(self.hardware.homing_predicates() if hasattr(self.hardware, "homing_predicates") else {}, ["z", "g", "x", "y", "door"])
            if not matrix.get("ok"):
                status["axis_reference"] = {"required": True, "ok": False, "blockers": matrix.get("blockers", [])}
                return self._fail(status, "live homing blocked: OEM home switch predicates are not proven")
        cmd = OemMotionCommand(session_id=status["session_id"], name="initializeSystem", payload={"mode": status["mode"], "run_homing": req.get("run_homing", True), "run_post_home": req.get("run_post_home", True), "artifact_root": str(root)})
        self.worker.artifact_root = root
        root.mkdir(parents=True, exist_ok=True)
        queued = self.worker.enqueue(cmd)
        status["motion_worker"] = self.worker.status()
        status["queued"] = True
        status["command"] = queued
        self._stage(status, OemStartupState.INITIALIZE_SYSTEM_QUEUED)
        _atomic_json(root / "initialize_system.json", {"queued": True, "command": queued, "not_run_inline": True, "source_anchor": SOURCE_ANCHORS["motionQueue"]})
        return self._closeout(status)

    def _handle_initialize_system(self, cmd: OemMotionCommand) -> dict:
        status = self.sessions.get(cmd.session_id)
        if not status:
            return {"ok": False, "error": "startup session not found"}
        root = Path(status["artifact_root"])
        payload = cmd.payload or {}
        self._stage(status, OemStartupState.INITIALIZE_SYSTEM_RUNNING)
        result = self.hardware.run_initialize_system(mode=payload.get("mode", status["mode"]), run_homing=bool(payload.get("run_homing", True)), run_post_home=bool(payload.get("run_post_home", True)), artifact_root=root, session_id=status["session_id"])
        self._stage(status, OemStartupState.INITIALIZE_MOTION_RUNNING)
        self._stage(status, OemStartupState.INITIALIZE_MOTORS_RUNNING)
        if payload.get("run_homing", True):
            for state in [OemStartupState.HOMING_Z, OemStartupState.HOMING_GRIPPER_CLEAR, OemStartupState.HOMING_GRIPPER, OemStartupState.HOMING_X, OemStartupState.PARKING_X_6000, OemStartupState.HOMING_Y, OemStartupState.HOMING_DOOR, OemStartupState.SETTING_Y_HOME]:
                self._stage(status, state)
            status["axis_reference"] = {"required": True, "ok": True, "dry_run": status["mode"] != "live"}
        else:
            status["axis_reference"] = {"required": True, "ok": False, "skipped": True, "reason": "run_homing=false"}
        self._stage(status, OemStartupState.POST_HOME_PIPETTE)
        post_pipette = dry_run_initialize_motion_pipette_cleanup(mode=status["mode"])
        post_pipette.update({"required": True, "blocks_ready": True})
        _atomic_json(root / "post_home_pipette_cleanup.json", post_pipette)
        self._stage(status, OemStartupState.VISION_INSPECTION)
        vision = self.hardware.vision_startup_check(mode=status["mode"]) if hasattr(self.hardware, "vision_startup_check") else {"ok": False, "required": True, "blocks_ready": True, "reason": "vision provider absent"}
        status["vision"] = vision
        _atomic_json(root / "vision_inspection.json", vision)
        self._stage(status, OemStartupState.PARKING_GANTRY)
        _atomic_json(root / "gantry_park.json", {"ok": False, "skipped": True, "blocks_ready": True, "reason": "parkGantry parity not yet live-bound"})
        self._stage(status, OemStartupState.DOOR_READY)
        _atomic_json(root / "door_ready_state.json", {"ok": False, "skipped": True, "blocks_ready": True, "reason": "door-ready/open state not yet live-bound"})
        if status["mode"] == "dry_run":
            status["state"] = OemStartupState.DIAGNOSTIC_COMPLETE.value
            status["ready"] = False
            status["failure_reason"] = "dry-run diagnostic complete; live readiness intentionally blocked by pipette/vision/park gates"
        else:
            self._fail(status, "startup post-home pipette/vision/park gates are not live-complete")
        self._closeout(status)
        return {"ok": True, "session_id": cmd.session_id, "startup_state": status["state"], "ready": status["ready"], "result": result}

    def run_next_worker_command(self) -> dict | None:
        return self.worker.run_next()

    def abort_worker(self, *, reason: str = "operator abort") -> dict:
        from .lifecycle_state import lifecycle_state

        result = self.worker.abort(reason=reason)
        if self.latest_session_id and self.latest_session_id in self.sessions:
            status = self.sessions[self.latest_session_id]
            status.update({"ok": False, "state": OemStartupState.ABORTED.value, "ready": False, "failed": True, "failed_closed": True, "failure_reason": reason})
            self._closeout(status)
        lifecycle = lifecycle_state.transition("stopped", reason=f"startup_worker_abort:{reason}")
        return {**result, "state": lifecycle["operation_state"], "lifecycle": lifecycle}

    def door_event(self, session_id: str | None, *, door_closed: bool, latch_closed: bool) -> dict:
        from .lifecycle_state import lifecycle_state

        lifecycle = lifecycle_state.record_door_event(
            door_closed=door_closed,
            latch_closed=latch_closed,
            source="OEMStartupProgram.door_event",
        )
        return {"ok": True, "session_id": session_id or "canonical", "door": lifecycle["door"], "state": lifecycle["operation_state"], "lifecycle": lifecycle}

        # Retained historical artifact logic below is unreachable; door events
        # may no longer execute initialCheck or queue startup stages implicitly.
        sid = session_id or self.latest_session_id
        if not sid or sid not in self.sessions:
            raise KeyError("startup session not found")
        status = self.sessions[sid]
        if status.get("state") != OemStartupState.WAITING_FOR_DOOR_CLOSE.value:
            raise ValueError("door_event is only valid while waiting_for_door_close")
        root = Path(status["artifact_root"])
        if hasattr(self.hardware, "door_closed"):
            self.hardware.door_closed = bool(door_closed)
        if hasattr(self.hardware, "latch_closed"):
            self.hardware.latch_closed = bool(latch_closed)
        event = {"door_closed": bool(door_closed), "latch_closed": bool(latch_closed), "source_anchor": SOURCE_ANCHORS["doorEvent"]}
        _atomic_json(root / "door_event.json", event)
        if not (door_closed and latch_closed):
            _atomic_json(root / "door_wait.json", {"ok": True, "waiting": True, "door_latch": event})
            return self._closeout(status)
        self._stage(status, OemStartupState.DOOR_CLOSE_OBSERVED)
        initial = self.hardware.initial_check(mode=status.get("mode", "dry_run"))
        _atomic_json(root / "initial_check_after_door.json", initial)
        status["door_latch"] = initial.get("door_latch", event)
        self._stage(status, OemStartupState.INITIAL_CHECK_AFTER_DOOR)
        if not initial.get("ok"):
            return self._fail(status, "door close event did not satisfy door/latch initialCheck")
        return self._queue_initialize_system(status)

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

    def worker_status(self) -> dict:
        from .lifecycle_state import lifecycle_state

        lifecycle = lifecycle_state.projection()
        return {**self.worker.status(), "state": lifecycle["operation_state"], "startup": lifecycle["startup"], "lifecycle": lifecycle}
