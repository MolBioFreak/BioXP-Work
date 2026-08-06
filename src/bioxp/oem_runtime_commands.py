from __future__ import annotations

from typing import Any, Callable

from .oem_runtime_types import OEMRuntimeCommand


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

OEM_WARNING_SITUATIONS = frozenset({
    "ENCLOSURE_OPEN", "ENCLOSURE_CLOSE", "UNLOCK_DOOR", "ABORT_JOB",
    "WAIT_INITIALIZATION", "POWER_OFF", "MONITOR_SLEEP", "ABORTING_JOB",
    "OPEN_DOOR_EVENT", "NETWORK_PROBLEM", "SOFTWARE_UPDATE",
    "SOFTWARE_UPDATE_DOWNLOAD_FAILURE", "ABORT_PREP", "THERMAL_FAULT",
    "ABORT_DELAY_START", "EXITPROGRAM", "LOCAL_MODE_WARNING",
    "LOCAL_MODE_CHANGE", "REAGENT_BLOCK_SIZE",
})


class OEMRuntimeCommandHandlers:
    def __init__(
        self,
        *,
        z_abort_provider: Callable[[OEMRuntimeCommand], dict[str, Any]] | None = None,
        z_resume_provider: Callable[[OEMRuntimeCommand], dict[str, Any]] | None = None,
    ) -> None:
        self.z_abort_provider = z_abort_provider
        self.z_resume_provider = z_resume_provider

    def handlers(self) -> dict[str, Any]:
        return {
            "unlockProcess": self.handle_unimplemented_source_blocked,
            "PrepareToRunJob": self.handle_prepare_to_run_job_readiness,
            "abortjob": self.handle_abortjob,
            "validateJob": self.handle_unimplemented_source_blocked,
            "wakefrompause": self.handle_wakefrompause,
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
        action = (command.params or {}).get("action", "ABORT_JOB")
        if type(action) is not str or action not in OEM_WARNING_SITUATIONS:
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "blockers": ["invalid_oem_warning_situation"],
            }
        if command.mode != "live":
            return {
                "ok": True,
                "ready": False,
                "state": "abortjob_dry_run_complete",
                "command": command.name,
                "warning_situation": action,
                "safe_action_taken": "would_execute_provider_owned_z_abort",
                "physical_motion_commanded": False,
            }
        if self.z_abort_provider is None:
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "blockers": ["live_z_abort_provider_not_bound"],
            }
        result = self.z_abort_provider(command)
        ok = bool(isinstance(result, dict) and result.get("ok") is True)
        return {
            "ok": ok,
            "ready": False,
            "state": "aborting_job" if ok else "recovery_required",
            "command": command.name,
            "warning_situation": action,
            "safe_action_taken": "provider_owned_z_abort_executed",
            "z_abort": result,
            "physical_effect_verified": False,
        }

    def handle_wakefrompause(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        if command.mode != "live":
            return {
                "ok": True,
                "ready": False,
                "state": "wakefrompause_dry_run_complete",
                "command": command.name,
                "source_order": ["initialCheck", "rehome", "status_update", "dresumeJob"],
                "physical_motion_commanded": False,
            }
        if self.z_resume_provider is None:
            return {
                "ok": False,
                "ready": False,
                "state": "failed_closed",
                "command": command.name,
                "blockers": ["live_z_resume_provider_not_bound"],
            }
        result = self.z_resume_provider(command)
        z_recovered = bool(
            isinstance(result, dict)
            and result.get("ok") is True
            and result.get("z_state") == "referenced_ready"
        )
        return {
            "ok": z_recovered,
            "ready": False,
            "state": "z_recovered_full_wake_required" if z_recovered else "recovery_required",
            "command": command.name,
            "source_order": ["initialCheck", "z_rehome"],
            "z_recovery": result,
            "omitted_non_z_source_work": ["full_rehome", "status_update", "dresumeJob"],
            "blockers": (
                ["full_oem_wakefrompause_not_implemented"]
                if z_recovered
                else ["z_rehome_failed_or_ambiguous"]
            ),
            "truth_level": "z_bearing_projection_only_not_full_wakefrompause",
            "physical_effect_verified": False,
        }
