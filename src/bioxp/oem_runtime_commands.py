from __future__ import annotations

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

    def handle_unimplemented_source_blocked(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "blockers": [f"{command.name}_source_parity_not_implemented"]}

    def handle_abortjob(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": True, "ready": False, "state": "aborting_job", "command": command.name, "safe_action_taken": "abortjob_journaled_force_abort_required"}

    def handle_wakefrompause(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "blockers": ["wakefrompause_rehome_predicates_unproven"]}
