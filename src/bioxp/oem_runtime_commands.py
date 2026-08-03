from __future__ import annotations

from typing import Any

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


class OEMRuntimeCommandHandlers:
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
        return {
            "ok": True,
            "ready": False,
            "state": "aborting_job",
            "command": command.name,
            "safe_action_taken": "abortjob_journaled_force_abort_required",
        }

    def handle_wakefrompause(self, command: OEMRuntimeCommand) -> dict[str, Any]:
        return {"ok": False, "ready": False, "state": "failed_closed", "command": command.name, "blockers": ["wakefrompause_rehome_predicates_unproven"]}
