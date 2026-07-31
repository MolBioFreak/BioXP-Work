from __future__ import annotations

from pathlib import Path
import json
from typing import Any

from .oem_runtime_types import OEMRuntimeCommand
from .oem_initialization import run_oem_initialization_controller
from .oem_movement_ledger import OemMovementLedger


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
    def __init__(self, *, startup_program=None, startup_program_factory=None, store=None):
        self.startup_program = startup_program
        self.startup_program_factory = startup_program_factory
        self.movement_ledger = OemMovementLedger(store)

    def _startup_program(self):
        if self.startup_program is None and self.startup_program_factory is not None:
            self.startup_program = self.startup_program_factory()
        return self.startup_program

    def handlers(self) -> dict[str, Any]:
        return {
            "initializeSystem": self.handle_initialize_system,
            "startupHomingStepwise": self.handle_startup_homing_stepwise,
            "unlockProcess": self.handle_unimplemented_source_blocked,
            "PrepareToRunJob": self.handle_prepare_to_run_job_readiness,
            "abortjob": self.handle_abortjob,
            "validateJob": self.handle_unimplemented_source_blocked,
            "wakefrompause": self.handle_wakefrompause,
        }

    def handle_startup_homing_stepwise(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        """Execute or observe exactly one ledger-bound initializeMotors stage."""
        return self._handle_stepwise_homing_stage(command, dict(command.params or {}))

    def handle_initialize_system(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        params = dict(command.params or {})
        if bool(params.get("run_oem_initialization", False)):
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "oemInitializationController",
                "blockers": ["legacy_monolithic_initialization_cannot_bypass_canonical_startup_stages"],
                "required_routes": [
                    "POST /oem/startup/constructor_pipettes",
                    "POST /oem/startup/initialize_without_motion",
                    "POST /oem/initial_check",
                ],
            }
        if bool(params.get("run_stepwise_homing", False)):
            return self._handle_stepwise_homing_stage(command, params)
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

    def _fail_stepwise_after_admission(
        self,
        command: OEMRuntimeCommand,
        step: str,
        exc: Exception,
        *,
        executor_result: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        failure: dict[str, Any] = {
            "ok": False,
            "error": str(exc),
            "exception_type": exc.__class__.__name__,
            "physical_effect_verified": False,
        }
        if executor_result is not None:
            # Never copy raw executor output into the terminal fallback. This
            # path may have been entered precisely because that output could
            # not be JSON-serialized; the authoritative ledger and worker
            # journal must remain independently writable.
            failure["executor_result_omitted"] = True
            failure["executor_result_summary"] = {
                "reported_ok": executor_result.get("ok") is True,
                "motion_command_attempted": executor_result.get("motion_command_attempted") is True,
                "controller_acknowledged": executor_result.get("controller_acknowledged") is True,
                "reported_physical_effect_verified": executor_result.get("physical_effect_verified") is True,
            }
        artifact_path = None
        try:
            artifact_path = self._write_stage_artifact(
                command,
                f"runtime_stepwise_homing_{step}_failure.json",
                {"command": command.to_dict(), "stepwise_homing": failure},
            )
        except Exception as artifact_exc:
            failure["failure_artifact_error"] = str(artifact_exc)
            failure["failure_artifact_exception_type"] = artifact_exc.__class__.__name__
        try:
            movement_ledger = self.movement_ledger.record_result(
                stage=step,
                command_id=command.command_id,
                result=failure,
                artifact_path=artifact_path,
            ) if command.mode == "live" else self.movement_ledger.projection()
        except Exception as ledger_exc:
            failure["ledger_failure_error"] = str(ledger_exc)
            failure["ledger_failure_exception_type"] = ledger_exc.__class__.__name__
            try:
                movement_ledger = self.movement_ledger.projection()
            except Exception:
                movement_ledger = None
        return {
            "ok": False,
            "ready": False,
            "state": "failed_closed",
            "command": command.name,
            "stage": "startupHomingStepwise",
            "mode": "live" if command.mode == "live" else "shadow",
            "homing_step": step,
            "stepwise_homing": failure,
            "oem_movement_ledger": movement_ledger,
            "artifact_path": artifact_path,
            "blockers": ["stepwise_homing_stage_exception", "physical_state_requires_operator_review"],
        }

    def _handle_oem_initialization_controller_stage(self, command: OEMRuntimeCommand, params: dict[str, Any]) -> dict[str, Any]:
        run_homing = bool(params.get("run_homing", False))
        if run_homing:
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "oemInitializationController",
                "blockers": ["run_homing_is_not_permitted_in_initializeMotorsWithoutMotion_stage"],
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
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "stage": "initializeMotion",
                "blockers": ["run_homing_true_rejected_by_initializeMotion_diagnostic_stage"],
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

    def _handle_stepwise_homing_stage(self, command: OEMRuntimeCommand, params: dict[str, Any]) -> dict[str, Any]:
        from .lifecycle_state import lifecycle_state

        # Shadow planning is source inspection and requires no completed live
        # predecessor. Live acknowledgement is validated before lifecycle or
        # provider access, then the canonical initialCheck gate is enforced.
        step = str(params.get("homing_step", "plan")).strip().lower()
        observation_requested = params.get("record_stage_observation", False)
        if type(observation_requested) is not bool:
            return {
                "ok": False, "ready": False, "state": "failed_closed",
                "command": command.name, "stage": "startupHomingStepwise",
                "blockers": ["record_stage_observation_must_be_boolean"],
            }
        if observation_requested is True:
            if command.mode != "live" or command.operator_ack != "OBSERVE":
                return {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "command": command.name,
                    "stage": "startupHomingStepwise",
                    "blockers": ["operator_ack_OBSERVE_required_for_oem_initializeMotors_stage_observation"],
                }
            observed_pass = params.get("observed_pass")
            operator_note = params.get("operator_note")
            if type(observed_pass) is not bool:
                return {
                    "ok": False, "ready": False, "state": "failed_closed",
                    "command": command.name, "stage": "startupHomingStepwise",
                    "blockers": ["observed_pass_must_be_boolean"],
                }
            if not isinstance(operator_note, str) or not operator_note.strip():
                return {
                    "ok": False, "ready": False, "state": "failed_closed",
                    "command": command.name, "stage": "startupHomingStepwise",
                    "blockers": ["operator_note_required_for_stage_observation"],
                }
            observation = self.movement_ledger.record_observation(
                stage=step,
                observed_pass=observed_pass,
                note=operator_note.strip(),
                command_id=command.command_id,
            )
            return {
                "ok": bool(observation.get("ok")),
                "ready": False,
                "state": "oem_initializeMotors_stage_observed" if observation.get("ok") else "failed_closed",
                "command": command.name,
                "stage": "startupHomingStepwise",
                "homing_step": step,
                "oem_movement_ledger": observation.get("ledger"),
                "blockers": [] if observation.get("ok") else [str(observation.get("blocker") or "oem_initializeMotors_observation_failed")],
            }
        required_ack = "HOME"
        if command.mode == "live" and command.operator_ack != required_ack:
            return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "stage": "startupHomingStepwise", "blockers": ["operator_ack_HOME_required_for_live_oem_initializeMotors_step"]}
        if command.mode == "live" and step in {"plan", "full", "all"}:
            return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "stage": "startupHomingStepwise", "blockers": ["live_stepwise_homing_requires_single_homing_step"]}
        if command.mode == "live":
            from .lifecycle_state import lifecycle_state

            lifecycle = lifecycle_state.projection()
            if lifecycle["startup"]["stages"]["initial_check"]["state"] != "passed":
                return {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "command": command.name,
                    "stage": "startupHomingStepwise",
                    "blockers": ["canonical_initial_check_predecessor_not_passed"],
                    "startup": lifecycle["startup"],
                }
        program = self._startup_program()
        if program is None or not hasattr(program, "hardware"):
            return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "stage": "startupHomingStepwise", "blockers": ["stepwise_homing_provider_not_bound"]}
        hardware = program.hardware
        if not hasattr(hardware, "startup_homing_stepwise"):
            return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "stage": "startupHomingStepwise", "blockers": ["startup_homing_stepwise_method_unavailable"]}
        admission = None
        if command.mode == "live":
            admission = self.movement_ledger.admit(stage=step, command_id=command.command_id)
            if not admission.get("ok"):
                return {
                    "ok": False,
                    "ready": False,
                    "state": "failed_closed",
                    "command": command.name,
                    "stage": "startupHomingStepwise",
                    "homing_step": step,
                    "oem_movement_ledger": admission.get("ledger"),
                    "blockers": [str(admission["blocker"])],
                }
        stage_kwargs = {
            "mode": ("live" if command.mode == "live" else "shadow"),
            "step": step,
            "execute": command.mode == "live",
            "preclear_abs": params.get("preclear_abs"),
            "require_operator_observed": bool(params.get("require_operator_observed", True)),
        }
        result = None
        try:
            try:
                result = hardware.startup_homing_stepwise(**stage_kwargs)
            except TypeError as exc:
                if "unexpected keyword" not in str(exc):
                    raise
                result = hardware.startup_homing_stepwise(mode=stage_kwargs["mode"], step=step, execute=command.mode == "live")
            artifact_path = self._write_stage_artifact(command, f"runtime_stepwise_homing_{step}.json", {"command": command.to_dict(), "stepwise_homing": result})
            ok = bool(result.get("ok"))
            movement_ledger = self.movement_ledger.record_result(
                stage=step,
                command_id=command.command_id,
                result=result,
                artifact_path=artifact_path,
            ) if command.mode == "live" else self.movement_ledger.projection()
        except Exception as exc:
            return self._fail_stepwise_after_admission(
                command,
                step,
                exc,
                executor_result=result if isinstance(result, dict) else None,
            )
        return {
            "ok": ok,
            "ready": False,
            "state": "stepwise_homing_step_complete" if ok and command.mode == "live" else ("stepwise_homing_plan_ready" if ok else "failed_closed"),
            "command": command.name,
            "stage": "startupHomingStepwise",
            "mode": "live" if command.mode == "live" else "shadow",
            "homing_step": step,
            "stepwise_homing": result,
            "oem_movement_ledger": movement_ledger,
            "artifact_path": artifact_path,
            "blockers": ([] if ok else ["stepwise_homing_stage_failed"]) + [
                "physical_observation_required_for_each_live_oem_initializeMotors_step",
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
