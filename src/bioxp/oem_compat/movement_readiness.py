from __future__ import annotations

from typing import Any

READINESS_MATRIX: list[dict[str, Any]] = [
    {
        "layer": "OEM source anchors",
        "oem_expected": "ClassControlInterface + DefaultParameters + config.xml/PositionTable drive target math and path branch selection.",
        "new_system_status": "implemented_for_dry_run",
        "evidence": [
            "docs/plans/2026-06-10-oem-machine-state-pathing-parity.md",
            "src/bioxp/oem_compat/machine_state.py",
            "src/bioxp/oem_compat/position_table.py",
            "src/bioxp/oem_compat/pathing.py",
        ],
        "gap": None,
        "movement_gate": "source parity basis present",
    },
    {
        "layer": "Instrument-specific config",
        "oem_expected": "Use this instrument's original SSD AppData config, calibration flags, axis limits, offsets, and 29-row PositionTable.",
        "new_system_status": "implemented_read_only",
        "evidence": ["GET /motion/oem/machine_config", "GET /motion/oem/position_table", "GET /motion/range/status"],
        "gap": None,
        "movement_gate": "verify source original_ssd_machine_config before motion",
    },
    {
        "layer": "Path planning",
        "oem_expected": "scriptmoveTo target math, pseudo-Z state, confirmAxis('gripper'), currentLoc/currentWell, getMidPoint, cleanPath, tip and waste-bin special branches.",
        "new_system_status": "implemented_for_planning",
        "evidence": ["GET /motion/oem/pathing/default_parameters", "GET /motion/oem/pathing/scriptmove_plan"],
        "gap": None,
        "movement_gate": "compare planned step sequence to the intended operator-supervised move before execution",
    },
    {
        "layer": "Live execution adapter",
        "oem_expected": "OEM moveTo/moveXY/moveZ/moveSteps sequence executes through controller with interlocks, current prep, and waits.",
        "new_system_status": "executor_preview_route_implemented_live_actuation_disabled",
        "evidence": ["POST /motion/oem/pathing/scriptmove_execute", "POST /motion/axis/relative", "POST /motion/axis/absolute"],
        "gap": "executor handoff now consumes OEM path-plan steps, but live actuation of those steps is still disabled pending supervised execution wiring",
        "movement_gate": "do not run full OEM path moves until scriptmove_execute live mode performs guarded step execution and has supervised proof",
    },
    {
        "layer": "Movement observation / operator truth",
        "oem_expected": "OEM relies on commanded controller motion plus the operator/instrument context; it does not provide independent automatic physical-motion proof.",
        "new_system_status": "matches_oem_observation_model",
        "evidence": ["motion_truth remains explicit about evidence level", "operator/camera observation is a commissioning test rule, not an OEM-parity gap"],
        "gap": None,
        "movement_gate": "record operator/camera observation during commissioning, matching practical OEM test discipline",
    },
    {
        "layer": "Arm/interlock/reference live state",
        "oem_expected": "Controller armed, latch/24V gates good, all axes stopped, current state known enough for requested move class.",
        "new_system_status": "must_be_checked_live_immediately_before_motion",
        "evidence": ["GET /motion/power/status", "GET /latch/status", "GET /motion/axes/status", "GET /motion/reference/status"],
        "gap": None,
        "movement_gate": "strict_startup run_homing=false may be required; absolute/path moves require reference reconciliation; desynced axes are not ready for blind absolute/path execution",
    },
    {
        "layer": "Safety/current invariant",
        "oem_expected": "No hot idle gripper; high current scoped to active gripper work; all speeds zero before/after; no hidden homing.",
        "new_system_status": "implemented_and_live_checked_before_motion",
        "evidence": ["/motion/axis/g/status current_safety", "tests/test_motion_phase1.py", "G_CURRENT_IDLE_SAFE invariant"],
        "gap": None,
        "movement_gate": "G speed=0 requires G param6<=10 and param7<=10 unless actively testing gripper",
    },
]


def build_movement_readiness_comparison() -> dict[str, Any]:
    hard_gaps = [row for row in READINESS_MATRIX if row.get("gap")]
    return {
        "ok": True,
        "schema_version": "bioxp.oem_movement_readiness_comparison.v1",
        "scope": "pre-movement no-motion OEM-vs-new comparison",
        "opened_usb": False,
        "physical_motion": False,
        "motion_commanded": False,
        "current_mutation_commanded": False,
        "summary": {
            "config_and_dry_run_pathing": "ready_for_pre_move_review",
            "live_oem_path_execution": "executor_preview_only",
            "blind_absolute_or_full_path_motion": "not_ready_without_reference_and_live_executor_gates",
            "supervised_micro_move_testing": "eligible_only_after_live_gates_pass",
            "hard_gap_count": len(hard_gaps),
        },
        "matrix": READINESS_MATRIX,
        "required_pre_move_live_checks": [
            "GET /status",
            "GET /motion/power/status",
            "GET /latch/status",
            "GET /motion/axes/status?axes=x,y,z,g,door",
            "GET /motion/reference/status?axes=x,y,z,g,door",
            "GET /motion/range/status?axes=x,y,z,g",
            "GET /motion/oem/machine_config",
            "GET /motion/oem/position_table",
            "GET /motion/oem/pathing/scriptmove_plan for intended target/scenario",
        ],
        "movement_test_start_policy": [
            "No homing unless explicitly requested and separately reviewed.",
            "No full OEM path execution until scriptmove_execute live mode performs guarded step execution and is tested.",
            "If arm=false/startup, run only strict_startup with run_homing=false after operator clearance.",
            "If reference state is desynced/unknown, use only supervised relative micro-move proof, not blind absolute/path moves.",
            "During commissioning, record before/after operator or camera observation just as OEM testing depends on observed instrument behavior.",
        ],
    }
