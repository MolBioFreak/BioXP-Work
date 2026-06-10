from __future__ import annotations

import json
from pathlib import Path

import pytest

from src.bioxp.oem_startup_program import DryRunStartupHardware, OEMStartupProgram


EXPECTED_FULL_INIT_SOURCE_ANCHORS = {
    "ControlLib.initializeMotion": "ControlLib.cs:8797-8856",
    "ClassPipetteCollection.initiateGroup": "ClassPipetteCollection.cs:677-693",
    "ClassPipetteCollection.checkedPipetteStatus": "ClassPipetteCollection.cs:726-748",
    "ClassPipetteCollection.ejectAllTips": "ClassPipetteCollection.cs:1176-1235",
    "ClassPipetteCollection.verifyEjectTip": "ClassPipetteCollection.cs:1265-1323",
    "ClassPipetteCollection.queryTipStatus": "ClassPipetteCollection.cs:1336-1357",
    "ClassPipette.QueryTipStatus": "ClassPipette.cs:571-589",
    "ClassPipette.QueryPressure": "ClassPipette.cs:622-628",
    "ControlLib.inspectCover": "ControlLib.cs:3663-3768",
    "ClassFrameGrabber.ScanBarcode": "ClassFrameGrabber.cs:125",
    "ClassFrameGrabber.CamCalibration": "ClassFrameGrabber.cs:4292",
    "ClassFrameGrabber.locateCover": "ClassFrameGrabber.cs:4578",
    "ClassFrameGrabber.checkPoolPlate": "ClassFrameGrabber.cs:6485",
    "ClassFrameGrabber.checkBioSecurityCover": "ClassFrameGrabber.cs:11918",
    "ClassFrameGrabber.adjustFocus": "ClassFrameGrabber.cs:12273",
}


def _loaded_config() -> dict:
    return {
        "status": "loaded",
        "path": "/tmp/fake-config.xml",
        "searched_roots": [],
        "fields": {"StartMode": 0, "GripperVersion": 1},
        "missing_fields": [],
        "live_ready": False,
        "derived_requirements": {"pipette_required": True, "vision_required": True},
    }


def test_full_init_source_coverage_matrix_includes_pipette_and_vision_anchors():
    from src.bioxp.oem_pipette_collection import OEM_FULL_INIT_SOURCE_COVERAGE

    assert OEM_FULL_INIT_SOURCE_COVERAGE == EXPECTED_FULL_INIT_SOURCE_ANCHORS


def test_oem_pipette_collection_dry_run_tip_cleanup_plans_source_order():
    from src.bioxp.oem_pipette_collection import OemPipetteCollection

    collection = OemPipetteCollection.dry_run(tip_loaded=(True, False, True, False))

    plan = collection.plan_initialize_motion_cleanup(mode="dry_run")

    assert plan["ok"] is False
    assert plan["live_ready"] is False
    assert plan["artifact_format"] == "bioxp-oem-pipette-init-v1"
    assert plan["source_anchor"] == "ControlLib.cs:8797-8856"
    assert [step["operation"] for step in plan["steps"]] == [
        "query_tip_status_before",
        "open_thermal_door",
        "scriptmove_to_waste_bin",
        "eject_all_tips",
        "move_z_80000",
        "move_x_79000",
        "query_tip_status_after",
        "clear_tip_machine_state",
        "initiate_group",
        "checked_pipette_status",
    ]
    assert plan["steps"][0]["tip_count"] == 2
    assert plan["steps"][3]["channels_targeted"] == [0, 2]
    assert plan["steps"][8]["source_anchor"] == "ClassPipetteCollection.cs:677-693"
    assert plan["steps"][9]["retry_once_on_failure"] is True
    assert plan["blockers"] == ["pipette_can_shadow_proof_required_before_live"]


def test_oem_pipette_collection_live_mode_is_still_no_hardware_dry_run_truth():
    from src.bioxp.oem_pipette_collection import OemPipetteCollection

    plan = OemPipetteCollection.dry_run(tip_loaded=(True, False, False, False)).plan_initialize_motion_cleanup(mode="live")

    assert plan["ok"] is False
    assert plan["live_ready"] is False
    assert plan["hardware_truth_level"] == "dry_run_model_no_transport"
    assert all(step["physical_motion"] is False for step in plan["steps"])
    assert plan["planned_physical_motion_if_live_oem_bound"] is True


def test_startup_worker_writes_oem_shaped_post_home_pipette_artifact(tmp_path):
    program = OEMStartupProgram(
        hardware=DryRunStartupHardware(config_status=_loaded_config()),
        artifact_base=tmp_path,
    )

    requested = program.request_startup(
        {
            "mode": "dry_run",
            "require_config": False,
            "door_policy": "already_closed",
            "run_homing": False,
            "run_post_home": True,
        }
    )
    assert requested["queued"] is True

    result = program.run_next_worker_command()
    assert result["ok"] is True

    artifact_root = Path(requested["artifact_root"])
    payload = json.loads((artifact_root / "post_home_pipette_cleanup.json").read_text())
    assert payload["artifact_format"] == "bioxp-oem-pipette-init-v1"
    assert payload["source_anchor"] == "ControlLib.cs:8797-8856"
    assert payload["ok"] is False
    assert payload["live_ready"] is False
    assert payload["steps"][0]["operation"] == "query_tip_status_before"
    assert payload["steps"][0]["source_anchor"] == "ClassPipetteCollection.cs:1336-1357"
    assert "pipette_can_shadow_proof_required_before_live" in payload["blockers"]


def test_vision_coverage_matrix_is_present_but_startup_vision_still_blocks_ready(tmp_path):
    from src.bioxp.oem_pipette_collection import OEM_FULL_INIT_SOURCE_COVERAGE

    assert OEM_FULL_INIT_SOURCE_COVERAGE["ControlLib.inspectCover"] == "ControlLib.cs:3663-3768"
    assert OEM_FULL_INIT_SOURCE_COVERAGE["ClassFrameGrabber.locateCover"] == "ClassFrameGrabber.cs:4578"

    program = OEMStartupProgram(
        hardware=DryRunStartupHardware(config_status=_loaded_config(), vision_required=True),
        artifact_base=tmp_path,
    )
    requested = program.request_startup({"mode": "dry_run", "require_config": False, "run_homing": False})
    program.run_next_worker_command()
    payload = json.loads((Path(requested["artifact_root"]) / "vision_inspection.json").read_text())
    assert payload["ok"] is False
    assert payload["blocks_ready"] is True
    assert "CVisionLib" in payload["reason"] or "vision" in payload["reason"].lower()
