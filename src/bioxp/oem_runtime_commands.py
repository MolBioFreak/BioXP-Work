from __future__ import annotations

from pathlib import Path
import json
from typing import Any

from .oem_runtime_types import OEMRuntimeCommand


class OEMRuntimeCommandHandlers:
    def __init__(self, *, startup_program=None):
        self.startup_program = startup_program

    def handlers(self) -> dict[str, Any]:
        return {
            "initializeSystem": self.handle_initialize_system,
            "unlockProcess": self.handle_unimplemented_source_blocked,
            "PrepareToRunJob": self.handle_unimplemented_source_blocked,
            "abortjob": self.handle_abortjob,
            "validateJob": self.handle_unimplemented_source_blocked,
            "wakefrompause": self.handle_wakefrompause,
        }

    def handle_initialize_system(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        params = dict(command.params or {})
        if bool(params.get("run_initial_check", False)) or bool(params.get("initial_check_only", False)):
            return self._handle_initial_check_stage(command, params)
        if not bool(params.get("delegate_to_startup_program", False)):
            return {
                "ok": True,
                "ready": False,
                "state": "diagnostic_complete",
                "command": command.name,
                "source_shape": "BioXPMainWindow.motion_thread_process consumed initializeSystem; full initializeSystem body remains staged/fail-closed",
                "implemented_runtime_layer": ["queue_consume", "gantry_available_bracket", "command_history", "durable_state"],
                "blockers": [
                    "live_initialCheck_not_requested",
                    "initializeMotion_not_executed_from_runtime_worker",
                    "home_predicates_unproven",
                    "pipette_gate_unproven",
                    "vision_gate_unproven",
                    "parkGantry_gate_unproven",
                ],
            }
        if self.startup_program is None:
            return {"ok": False, "ready": False, "state": "failed_closed", "blockers": ["startup_program_not_bound"]}
        req = {
            "mode": command.mode,
            "operator_ack": command.operator_ack,
            "artifact_root": command.artifact_root,
            "require_config": bool(params.get("require_config", False)),
            "run_homing": bool(params.get("run_homing", False)),
            "run_post_home": bool(params.get("run_post_home", True)),
        }
        session = self.startup_program.request_startup(req)
        if session.get("state") == "waiting_for_door_close":
            return {"ok": True, "ready": False, "state": "waiting_for_door_close", "startup": session}
        if hasattr(self.startup_program, "run_next_worker_command"):
            ran = self.startup_program.run_next_worker_command()
            status = self.startup_program.status(session.get("session_id"))
            return {"ok": bool(ran.get("ok", True)), "ready": bool(status.get("ready", False)), "state": status.get("state"), "startup": status, "worker_run": ran}
        return {"ok": True, "ready": False, "state": session.get("state"), "startup": session}

    def _handle_initial_check_stage(self, command: OEMRuntimeCommand, params: dict[str, Any]) -> dict[str, Any]:
        if command.mode == "live" and command.operator_ack != "INITIALIZE":
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initialCheck",
                "blockers": ["operator_ack_INITIALIZE_required_for_live_initialCheck"],
            }
        if self.startup_program is None or not hasattr(self.startup_program, "hardware"):
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initialCheck",
                "blockers": ["initialCheck_provider_not_bound"],
            }
        hardware = self.startup_program.hardware
        if not hasattr(hardware, "initial_check"):
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initialCheck",
                "blockers": ["initialCheck_method_unavailable"],
            }
        mode = "live" if command.mode == "live" else "shadow"
        result = hardware.initial_check(mode=mode)
        artifact_path = None
        if command.artifact_root:
            root = Path(command.artifact_root)
            root.mkdir(parents=True, exist_ok=True)
            artifact_path = root / "runtime_initial_check.json"
            tmp = artifact_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps({"command": command.to_dict(), "initial_check": result}, indent=2, sort_keys=True))
            tmp.replace(artifact_path)
        ok = bool(result.get("ok"))
        door_latch = result.get("door_latch") or {}
        blockers: list[str] = []
        if not ok:
            blockers.append("initialCheck_failed")
        return {
            "ok": ok,
            "ready": False,
            "state": "initial_check_passed" if ok else "failed_closed",
            "command": command.name,
            "stage": "initialCheck",
            "mode": mode,
            "initial_check": result,
            "door_latch": door_latch,
            "artifact_path": str(artifact_path) if artifact_path else None,
            "blockers": blockers + [
                "initializeMotion_not_executed_from_runtime_worker",
                "home_predicates_unproven",
                "pipette_gate_unproven",
                "vision_gate_unproven",
                "parkGantry_gate_unproven",
            ],
        }

    def handle_unimplemented_source_blocked(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "blockers": [f"{command.name}_source_parity_not_implemented"]}

    def handle_abortjob(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": True, "ready": False, "state": "aborting_job", "command": command.name, "safe_action_taken": "abortjob_journaled_force_abort_required"}

    def handle_wakefrompause(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "blockers": ["wakefrompause_rehome_predicates_unproven"]}
