from pathlib import Path

from src.bioxp.oem_compat.control_lib import BioXPControlLib
from src.bioxp.oem_compat.script_handler import BioXPScriptHandler
from src.bioxp.protocols.models import ProtocolAction, ProtocolActionKind, ProtocolDocument, ProtocolStage
from src.bioxp.protocols.oem_xml_import import import_oem_xml_protocol

OEM_SCRIPT_ROOT = Path("/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Scripts")


def test_script_handler_expands_imported_oem_actions_into_source_shaped_execution_steps() -> None:
    imported = import_oem_xml_protocol(OEM_SCRIPT_ROOT / "lifetest.xml")

    plan = BioXPScriptHandler.dry_run().interpret_protocol(imported.document)

    assert plan.mode == "dry_run"
    assert plan.source_anchored is True
    assert plan.unsupported_action_count == 0
    assert plan.action_count == imported.coverage.supported_command_count
    assert plan.step_count >= plan.action_count
    assert "ClassBioXPScriptHandler" in plan.oem_layers
    assert "ClassVirtualBioXP" in plan.oem_layers
    assert "ControlLib" in plan.oem_layers
    assert "ClassPipetteCollection" in plan.oem_layers
    assert "operator_gate" in plan.oem_layers

    macro_step = next(step for step in plan.steps if step.oem_verb == "ST")
    assert macro_step.layer == "ClassBioXPScriptHandler"
    assert macro_step.requires_virtual_bioxp_state is True
    assert macro_step.requires_ack_readback is True
    assert macro_step.pipette_plan is not None
    assert macro_step.pipette_plan.operation == "macro_material_transfer"
    assert macro_step.pipette_plan.preflight["material_id"] == "AMP"
    assert macro_step.pipette_plan.preflight["volume_ul"] == 14.0

    delay_step = next(step for step in plan.steps if step.oem_verb == "DELAYPOINT")
    assert delay_step.layer == "operator_gate"
    assert delay_step.live_blockers == ["operator_review_required"]


def test_control_lib_job_lifecycle_includes_script_interpretation_and_proof_ladder() -> None:
    imported = import_oem_xml_protocol(OEM_SCRIPT_ROOT / "lifetest.xml")

    result = BioXPControlLib.dry_run().execute_protocol(imported.document, source_path=imported.source_path)

    assert result.ok is True
    assert result.lifecycle == [
        "created",
        "preflighted",
        "script_interpreted",
        "planned",
        "state_applied",
        "proof_gated",
        "complete",
    ]
    assert result.execution_plan is not None
    assert result.execution_plan.step_count >= result.action_count
    assert result.proof_ladder["mode"] == "dry_run"
    assert result.proof_ladder["dry_run_clean"] is True
    assert result.proof_ladder["source_anchored"] is True
    assert result.proof_ladder["zero_unsupported_commands"] is True
    assert result.proof_ladder["transport_reply_contract_ready"] is False
    assert result.proof_ladder["virtual_state_complete"] is True
    assert result.proof_ladder["pipette_ack_readback_ready"] is False
    assert result.proof_ladder["vision_artifact_contract_ready"] is True
    assert result.proof_ladder["operator_ack_required"] is True
    assert result.proof_ladder["artifact_root_required"] is True
    assert result.proof_ladder["live_allowed"] is False
    assert "live transport validation not complete" in result.proof_ladder["blockers"]
    assert "pipette ACK/readback live validation not complete" in result.proof_ladder["blockers"]
    assert result.artifact["proof_ladder"] == result.proof_ladder
    assert result.artifact["execution_plan"]["step_count"] == result.execution_plan.step_count


def test_live_mode_fails_closed_without_operator_ack_artifact_and_subsystem_proof() -> None:
    imported = import_oem_xml_protocol(OEM_SCRIPT_ROOT / "demo.xml")

    result = BioXPControlLib.dry_run().execute_protocol(
        imported.document,
        source_path=imported.source_path,
        requested_mode="live",
        operator_ack=None,
        artifact_root=None,
    )

    assert result.ok is False
    assert result.mode == "live"
    assert result.lifecycle == ["created", "preflight_failed"]
    assert result.physical_motion is False
    assert result.artifact["fail_closed"] is True
    assert "operator_ack required for live mode" in result.preflight_errors
    assert "artifact_root required for live mode" in result.preflight_errors
    assert "live transport validation not complete" in result.preflight_errors


def test_vision_actions_create_artifact_contracts_and_live_blockers() -> None:
    action = ProtocolAction(
        action_id="inspect-1",
        stage_id="s1",
        kind=ProtocolActionKind.BARCODE_READ,
        params={"location_id": "PL_OUTPUT"},
        metadata={
            "oem_verb": "SCANBARCODE",
            "raw_cmd": "SCANBARCODE PL_OUTPUT",
            "source_node": {"source_file": "unit.xml", "script_position": 1},
        },
    )
    doc = ProtocolDocument(protocol_id="vision-doc", stages=(ProtocolStage(stage_id="s1", actions=(action,)),))

    plan = BioXPScriptHandler.dry_run().interpret_protocol(doc)

    assert plan.step_count == 1
    step = plan.steps[0]
    assert step.layer == "CVisionLib"
    assert step.vision_result is not None
    assert step.vision_result.oem_method == "ScanBarcode"
    assert step.vision_result.artifact_contract["image_required"] is True
    assert "vision live validation not complete" in step.live_blockers
