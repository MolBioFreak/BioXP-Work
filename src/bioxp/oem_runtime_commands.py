from __future__ import annotations

from pathlib import Path
import json
from typing import Any, Mapping

from .oem_runtime_types import OEMRuntimeCommand
from .oem_initialization import run_oem_initialization_controller
from .oem_serial206_initialization import Serial206CommissioningEvidence, Serial206StageApproval


PREPARE_TO_RUN_JOB_READINESS_SCHEMA_VERSION = "bioxp.oem_runtime.prepare_to_run_job_readiness.v1"

PREPARE_TO_RUN_JOB_SOURCE_ANCHORS = [
    "BioXPMainWindow.cs:1588-1808",
    "BioXPMainWindow.cs:2056",
    "ControlLib.cs:4349-4385",
    "ControlLib.cs:4430-4465",
    "ControlLib.cs:3981-4015",
    "ControlLib.cs:4139-4190",
    "ControlLib.cs:4234-4346",
    "ControlLib.cs:4468-4739",
]

PREPARE_TO_RUN_JOB_READINESS_STEPS = [
    ("home_gripper", "BioXPMainWindow.cs:1621"),
    ("inspect_purification_location", "BioXPMainWindow.cs:1624; ControlLib.cs:4349-4385"),
    ("inspect_output_plate", "BioXPMainWindow.cs:1640; ControlLib.cs:4430-4465"),
    ("inspect_trough", "BioXPMainWindow.cs:1664; ControlLib.cs:3981-4015"),
    ("inspect_strip_handle", "BioXPMainWindow.cs:1688; ControlLib.cs:4139-4190"),
    ("inspect_strip_wells", "BioXPMainWindow.cs:1712; ControlLib.cs:4234-4346"),
    ("inspect_tip_trays", "BioXPMainWindow.cs:1622; ControlLib.cs:4468-4739"),
    ("park_gantry", "BioXPMainWindow.cs:1760"),
]


class OEMRuntimeCommandHandlers:
    def __init__(
        self,
        *,
        startup_program=None,
        startup_program_factory=None,
        serial206_provider_factory=None,
        store=None,
    ):
        self.startup_program = startup_program
        self.startup_program_factory = startup_program_factory
        self.serial206_provider_factory = serial206_provider_factory
        self.store = store

    def _startup_program(self):
        if self.startup_program is None and self.startup_program_factory is not None:
            self.startup_program = self.startup_program_factory()
        return self.startup_program

    def _serial206_provider(self, capability: str):
        if self.serial206_provider_factory is None:
            raise RuntimeError("serial-206 initialization provider factory is not bound")
        provider = self.serial206_provider_factory()
        status = provider.capability_status()
        available_field = f"{capability}_live_available"
        if status.get(available_field) is not True:
            raise RuntimeError(f"serial-206 provider lacks {capability}: {status}")
        return provider

    @staticmethod
    def _stage_approval(value: Any) -> Serial206StageApproval | None:
        if value is None:
            return None
        if not isinstance(value, Mapping):
            raise ValueError("stage_approval must be an object")
        return Serial206StageApproval(**dict(value))

    @staticmethod
    def _stage_approvals(value: Any) -> dict[str, Serial206StageApproval]:
        if value is None:
            return {}
        if not isinstance(value, Mapping):
            raise ValueError("stage approvals must be an object keyed by source stage")
        parsed: dict[str, Serial206StageApproval] = {}
        for stage, payload in value.items():
            if not isinstance(payload, Mapping):
                raise ValueError(f"stage approval must be an object: {stage}")
            parsed[str(stage)] = Serial206StageApproval(**dict(payload))
        return parsed

    @staticmethod
    def _commissioning(value: Any) -> dict[str, Serial206CommissioningEvidence]:
        if value is None:
            return {}
        if not isinstance(value, Mapping):
            raise ValueError("commissioning must be an object keyed by component")
        parsed: dict[str, Serial206CommissioningEvidence] = {}
        for component, payload in value.items():
            if not isinstance(payload, Mapping):
                raise ValueError(f"commissioning evidence must be an object: {component}")
            parsed[str(component)] = Serial206CommissioningEvidence(**dict(payload))
        return parsed

    def handlers(self) -> dict[str, Any]:
        return {
            "initializeSystem": self.handle_initialize_system,
            "unlockProcess": self.handle_unimplemented_source_blocked,
            "PrepareToRunJob": self.handle_prepare_to_run_job_readiness,
            "abortjob": self.handle_abortjob,
            "validateJob": self.handle_unimplemented_source_blocked,
            "wakefrompause": self.handle_wakefrompause,
        }

    def handle_initialize_system(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        params = dict(command.params or {})
        if bool(params.get("run_oem_initialization", False)):
            return self._handle_oem_initialization_controller_stage(command, params)
        if bool(params.get("run_stepwise_homing", False)):
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "blockers": ["legacy_run_stepwise_homing_retired_use_serial206_initialization_provider"],
                "physical_motion_commanded": False,
                "hardware_touched": False,
                "replacement": "serial206 initialization provider",
            }
        if bool(params.get("run_initialize_motion", False)) or bool(params.get("initialize_motion_only", False)):
            return self._handle_initialize_motion_stage(command, params)
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
        program = self._startup_program()
        if program is None:
            return {"ok": False, "ready": False, "state": "failed_closed", "blockers": ["startup_program_not_bound"]}
        req = {
            "mode": command.mode,
            "operator_ack": command.operator_ack,
            "artifact_root": command.artifact_root,
            "require_config": bool(params.get("require_config", False)),
            "run_homing": bool(params.get("run_homing", False)),
            "run_post_home": bool(params.get("run_post_home", True)),
        }
        session = program.request_startup(req)
        if session.get("state") == "waiting_for_door_close":
            return {"ok": True, "ready": False, "state": "waiting_for_door_close", "startup": session}
        if hasattr(program, "run_next_worker_command"):
            ran = program.run_next_worker_command()
            status = program.status(session.get("session_id"))
            return {"ok": bool(ran.get("ok", True)), "ready": bool(status.get("ready", False)), "state": status.get("state"), "startup": status, "worker_run": ran}
        return {"ok": True, "ready": False, "state": session.get("state"), "startup": session}

    def _write_stage_artifact(self, command: OEMRuntimeCommand, filename: str, payload: dict[str, Any]) -> str | None:
        if not command.artifact_root:
            return None
        root = Path(command.artifact_root)
        root.mkdir(parents=True, exist_ok=True)
        artifact_path = root / filename
        tmp = artifact_path.with_suffix(artifact_path.suffix + ".tmp")
        tmp.write_text(json.dumps(payload, indent=2, sort_keys=True))
        tmp.replace(artifact_path)
        return str(artifact_path)

    def _handle_oem_initialization_controller_stage(self, command: OEMRuntimeCommand, params: dict[str, Any]) -> dict[str, Any]:
        run_homing = bool(params.get("run_homing", False))
        if run_homing:
            if command.mode != "live":
                return {
                    "ok": True,
                    "ready": False,
                    "state": "dry_run",
                    "command": command.name,
                    "stage": "initializeMotors",
                    "physical_motion_commanded": False,
                    "blockers": ["shadow_runtime_command_does_not_bind_live_serial206_provider"],
                }
            required_ack = "INITIALIZE_MOTORS_STAGE"
            if command.operator_ack != required_ack:
                return {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "command": command.name,
                    "stage": "initializeMotors",
                    "blockers": [f"operator_ack_{required_ack}_required"],
                }
            try:
                provider = self._serial206_provider("initialize_motors")
                result = provider.initialize_motors(
                    mode="live",
                    approval=self._stage_approval(params.get("stage_approval")),
                    commissioning=self._commissioning(params.get("commissioning")),
                    timeout_s=float(params.get("timeout_s", command.timeout_s or 180.0)),
                )
            except Exception as exc:
                result = {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "physical_motion_commanded": False,
                    "blockers": [f"serial206_initializeMotors_binding_or_contract_error:{type(exc).__name__}:{exc}"],
                }
            artifact_path = self._write_stage_artifact(
                command,
                "runtime_serial206_initialize_motors.json",
                {"command": command.to_dict(), "initialize_motors": result},
            )
            return {
                "ok": result.get("ok") is True,
                "ready": result.get("ready") is True,
                "state": result.get("state", "failed_closed"),
                "command": command.name,
                "stage": "initializeMotors",
                "mode": "live",
                "artifact_path": artifact_path,
                "initialize_motors": result,
                "physical_motion_commanded": result.get("physical_motion_commanded") is True,
                "physical_effect_verified": result.get("physical_effect_verified") is True,
                "blockers": list(result.get("blockers") or []),
            }
        required_ack = "OEM_INITIALIZATION_RUN"
        if command.mode == "live" and command.operator_ack != required_ack:
            return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "stage": "oemInitializationController", "blockers": [f"operator_ack_{required_ack}_required_for_live_oem_initialization_controller"]}
        program = self._startup_program()
        if program is None or not hasattr(program, "hardware"):
            return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "stage": "oemInitializationController", "blockers": ["oem_initialization_provider_not_bound"]}
        hardware = program.hardware
        timeout_s = float(params.get("timeout_s", command.timeout_s or 180.0))
        from .lifecycle_state import LifecycleStateError, lifecycle_state

        try:
            lifecycle = lifecycle_state.run_stage(
                "initialization_without_motion",
                lambda: run_oem_initialization_controller(
                    hardware,
                    run_homing=run_homing,
                    restore_door_state=bool(params.get("restore_door_state", False)),
                    include_tip_pipette_cleanup=bool(params.get("include_tip_pipette_cleanup", False)),
                    timeout_s=timeout_s,
                ),
            )
        except LifecycleStateError as exc:
            result = {"ok": False, "error": str(exc), "predecessor_rejected": True}
        else:
            stage = lifecycle["startup"]["stages"]["initialization_without_motion"]
            result = dict(stage.get("evidence") or {})
            result["lifecycle"] = lifecycle
        artifact_path = self._write_stage_artifact(command, "runtime_oem_initialization_controller.json", {"command": command.to_dict(), "oem_initialization": result})
        ok = bool(result.get("ok"))
        return {
            "ok": ok,
            "ready": ok,
            "state": "init_ready" if ok else "init_failed",
            "command": command.name,
            "stage": "oemInitializationController",
            "mode": "live" if command.mode == "live" else "shadow",
            "artifact_path": artifact_path,
            "oem_initialization": result,
            "blockers": [] if ok else ["oem_initialization_controller_failed"],
        }

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
        program = self._startup_program()
        if program is None or not hasattr(program, "hardware"):
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initialCheck",
                "blockers": ["initialCheck_provider_not_bound"],
            }
        hardware = program.hardware
        if command.mode != "live" and not hasattr(hardware, "initial_check"):
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initialCheck",
                "blockers": ["initialCheck_method_unavailable"],
            }
        mode = "live" if command.mode == "live" else "shadow"
        if mode != "live":
            result = {"ok": True, "executed": False, "mode": mode, "reason": "non-live worker command cannot complete canonical initialCheck"}
        else:
            from .lifecycle_state import LifecycleStateError, lifecycle_state

            try:
                lifecycle = lifecycle_state.run_initial_check(
                    hardware,
                    can_ready=lambda: lifecycle_state.projection()["CAN_READY"],
                )
            except LifecycleStateError as exc:
                result = {"ok": False, "error": str(exc), "predecessor_rejected": True}
            else:
                stage = lifecycle["startup"]["stages"]["initial_check"]
                result = {"ok": stage["state"] == "passed", "lifecycle": lifecycle, "evidence": stage["evidence"]}
        artifact_path = self._write_stage_artifact(command, "runtime_initial_check.json", {"command": command.to_dict(), "initial_check": result})
        ok = bool(result.get("ok"))
        door_latch = result.get("door_latch") or {}
        blockers: list[str] = []
        if not ok:
            blockers.append("initialCheck_failed")
        return {
            "ok": ok,
            "ready": False,
            "state": ("diagnostic_complete" if result.get("executed") is False else "initial_check_passed") if ok else "failed_closed",
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

    def _handle_initialize_motion_stage(self, command: OEMRuntimeCommand, params: dict[str, Any]) -> dict[str, Any]:
        if bool(params.get("run_homing", False)):
            if command.mode != "live":
                return {
                    "ok": True,
                    "ready": False,
                    "state": "dry_run",
                    "command": command.name,
                    "stage": "initializeMotion",
                    "physical_motion_commanded": False,
                    "blockers": ["shadow_runtime_command_does_not_bind_live_serial206_provider"],
                }
            required_ack = "INITIALIZE_MOTION_STAGE"
            if command.operator_ack != required_ack:
                return {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "command": command.name,
                    "stage": "initializeMotion",
                    "blockers": [f"operator_ack_{required_ack}_required"],
                }
            try:
                provider = self._serial206_provider("initialize_motion")
                result = provider.initialize_motion(
                    mode="live",
                    approvals=self._stage_approvals(params.get("motor_stage_approvals")),
                    motion_approvals=self._stage_approvals(params.get("motion_stage_approvals")),
                    commissioning=self._commissioning(params.get("commissioning")),
                    timeout_s=float(params.get("timeout_s", command.timeout_s or 180.0)),
                )
            except Exception as exc:
                result = {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "physical_motion_commanded": False,
                    "blockers": [f"serial206_initializeMotion_binding_or_contract_error:{type(exc).__name__}:{exc}"],
                }
            artifact_path = self._write_stage_artifact(
                command,
                "runtime_serial206_initialize_motion.json",
                {"command": command.to_dict(), "initialize_motion": result},
            )
            return {
                "ok": result.get("ok") is True,
                "ready": result.get("ready") is True,
                "state": result.get("state", "failed_closed"),
                "command": command.name,
                "stage": "initializeMotion",
                "mode": "live",
                "artifact_path": artifact_path,
                "initialize_motion": result,
                "physical_motion_commanded": result.get("physical_motion_commanded") is True,
                "physical_effect_verified": result.get("physical_effect_verified") is True,
                "blockers": list(result.get("blockers") or []),
            }
        if command.mode == "live" and command.operator_ack != "INITIALIZE":
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initializeMotion",
                "blockers": ["operator_ack_INITIALIZE_required_for_live_initializeMotion"],
            }
        program = self._startup_program()
        if program is None or not hasattr(program, "hardware"):
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initializeMotion",
                "blockers": ["initializeMotion_provider_not_bound"],
            }
        hardware = program.hardware
        mode = "live" if command.mode == "live" else "shadow"
        initial_result = None
        if bool(params.get("run_initial_check", False)):
            if not hasattr(hardware, "initial_check"):
                return {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "command": command.name,
                    "stage": "initializeMotion",
                    "blockers": ["initialCheck_method_unavailable"],
                }
            initial_result = hardware.initial_check(mode=mode)
            if not bool(initial_result.get("ok")):
                self._write_stage_artifact(command, "runtime_initialize_motion.json", {"command": command.to_dict(), "initial_check": initial_result, "initialize_motion": None})
                return {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "command": command.name,
                    "stage": "initializeMotion",
                    "mode": mode,
                    "initial_check": initial_result,
                    "blockers": ["initialCheck_failed_before_initializeMotion"],
                }
        if not hasattr(hardware, "initialize_motion_diagnostic"):
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initializeMotion",
                "blockers": ["initializeMotion_diagnostic_method_unavailable"],
            }
        diagnostic = hardware.initialize_motion_diagnostic(mode=mode, run_homing=False)
        artifact_path = self._write_stage_artifact(command, "runtime_initialize_motion.json", {"command": command.to_dict(), "initial_check": initial_result, "initialize_motion": diagnostic})
        ok = bool(diagnostic.get("ok"))
        return {
            "ok": ok,
            "ready": False,
            "state": "initialize_motion_diagnostic_complete" if ok else "failed_closed",
            "command": command.name,
            "stage": "initializeMotion",
            "mode": mode,
            "initial_check": initial_result,
            "initialize_motion": diagnostic,
            "artifact_path": artifact_path,
            "blockers": ([] if ok else ["initializeMotion_diagnostic_failed"]) + [
                "home_predicates_unproven",
                "pipette_gate_unproven",
                "vision_gate_unproven",
                "parkGantry_gate_unproven",
            ],
        }

    def handle_prepare_to_run_job_readiness(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        params = dict(command.params or {})
        if command.mode == "live":
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "blockers": ["PrepareToRunJob_live_deck_inspection_not_implemented_use_dry_run_readiness_plan_first"],
            }
        readiness_steps = [
            {
                "name": name,
                "source_anchor": source_anchor,
                "motion_allowed": False,
                "camera_capture_allowed": False,
                "would_touch_hardware_in_live_oem": True,
            }
            for name, source_anchor in PREPARE_TO_RUN_JOB_READINESS_STEPS
        ]
        return {
            "ok": True,
            "ready": False,
            "schema_version": PREPARE_TO_RUN_JOB_READINESS_SCHEMA_VERSION,
            "state": "prepare_to_run_job_readiness_dry_run_complete",
            "command": command.name,
            "mode": "dry_run",
            "source": command.source,
            "source_parity": "source_anchored_prepare_to_run_job_deck_inspection",
            "source_anchors": list(PREPARE_TO_RUN_JOB_SOURCE_ANCHORS),
            "truth_level": "source_anchored_plan_only_no_motion",
            "motion_commanded": False,
            "hardware_touched": False,
            "readiness_steps": readiness_steps,
            "script_requirements": params.get("script_requirements") or {},
            "blockers": ["live_motion_camera_pattern_matching_not_executed_in_dry_run"],
            "notes": [
                "This is a no live hardware motion dry-run plan for OEM PrepareToRunJob readiness/deck inspection.",
                "The dry-run does not prove physical deck readiness; live camera/motion validation remains required before a real job.",
            ],
        }

    def handle_unimplemented_source_blocked(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "blockers": [f"{command.name}_source_parity_not_implemented"]}

    def handle_abortjob(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": True, "ready": False, "state": "aborting_job", "command": command.name, "safe_action_taken": "abortjob_journaled_force_abort_required"}

    def handle_wakefrompause(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "blockers": ["wakefrompause_rehome_predicates_unproven"]}
