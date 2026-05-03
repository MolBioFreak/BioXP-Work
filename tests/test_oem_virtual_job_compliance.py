from pathlib import Path

from src.bioxp.oem_compat.control_lib import BioXPControlLib
from src.bioxp.oem_compat.state import VirtualBioXP
from src.bioxp.protocols.models import ProtocolAction, ProtocolActionKind, ProtocolDocument, ProtocolStage
from src.bioxp.protocols.oem_xml_import import import_oem_xml_protocol

OEM_SCRIPT_ROOT = Path("/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Scripts")


def _action(kind, params=None, *, verb="TEST", action_id="a1"):
    return ProtocolAction(
        action_id=action_id,
        stage_id="s1",
        kind=kind,
        params=params or {},
        metadata={"oem_verb": verb, "raw_cmd": verb, "source_node": {"source_file": "unit.xml", "script_position": 1}},
    )


def test_virtual_bioxp_tracks_macro_material_state_without_motion() -> None:
    state = VirtualBioXP()

    first = state.apply_action(
        _action(
            ProtocolActionKind.PIPETTE_ASPIRATE,
            {
                "semantic_action": "material_transfer",
                "macro_verb": "ST",
                "material_id": "AMP",
                "dest_location_id": "PL_POOL",
                "dest_zone": "Z5",
                "volume_ul": 14.0,
                "requires_virtual_bioxp_state": True,
                "requires_ack_readback": True,
                "raw_tokens": ["AMP", "PL_POOL", "Z5", "V14"],
            },
            verb="ST",
            action_id="st-1",
        )
    )

    assert first.status == "planned_state_applied"
    assert first.physical_motion is False
    assert state.materials["AMP"].location_id == "PL_POOL"
    assert state.materials["AMP"].zone == "Z5"
    assert state.materials["AMP"].volume_ul == 14.0
    assert state.actions_applied == 1

    second = state.apply_action(
        _action(
            ProtocolActionKind.PIPETTE_MIX,
            {
                "semantic_action": "zone_wash",
                "macro_verb": "ZW",
                "material_id": "AMP",
                "target_location_id": "PL_POOL",
                "target_zone": "Z5",
                "source_location_id": "PL_OUTPUT",
                "source_zone": "Z9",
                "volume_ul": 120.0,
                "wash_count": 10,
                "mix_repeat": 5,
                "requires_virtual_bioxp_state": True,
                "requires_ack_readback": True,
                "raw_tokens": ["PL_POOL", "Z5", "PL_OUTPUT", "Z9", "V120", "W10", "MR5", "AMP"],
            },
            verb="ZW",
            action_id="zw-1",
        )
    )

    assert second.status == "planned_state_applied"
    assert state.materials["AMP"].last_operation == "zone_wash"
    assert state.materials["AMP"].wash_count == 10
    assert state.materials["AMP"].mix_repeat == 5
    assert state.materials["AMP"].source_location_id == "PL_OUTPUT"
    assert state.materials["AMP"].source_zone == "Z9"
    assert len(state.events) == 2


def test_control_lib_dry_run_job_executes_imported_protocol_into_virtual_state() -> None:
    imported = import_oem_xml_protocol(OEM_SCRIPT_ROOT / "lifetest.xml")

    result = BioXPControlLib.dry_run().execute_protocol(imported.document, source_path=imported.source_path)

    assert result.ok is True
    assert result.mode == "dry_run"
    assert result.executed is False
    assert result.physical_motion is False
    assert result.lifecycle == ["created", "preflighted", "planned", "state_applied", "complete"]
    assert result.action_count == imported.coverage.supported_command_count
    assert result.unsupported_action_count == 0
    assert result.review_required_count == 1  # DELAYPOINT remains explicit operator review gate.
    assert result.source_path.endswith("lifetest.xml")
    assert result.virtual_state.actions_applied == result.action_count
    assert result.virtual_state.materials["AMP"].last_operation in {"zone_wash", "standard_wash", "material_transfer", "material_agitate"}
    assert result.artifact["compliance_gate"] == "initial_oem_testing"
    assert result.artifact["unsupported_action_count"] == 0
    assert result.artifact["opened_usb"] is False
    assert result.artifact["physical_motion"] is False


def test_control_lib_rejects_protocol_with_unsupported_or_unanchored_actions() -> None:
    unsupported = _action(ProtocolActionKind.NOTE, {"unsupported_command_count": 1}, verb="BAD", action_id="bad-1")
    unanchored = ProtocolAction(action_id="bad-2", stage_id="s1", kind=ProtocolActionKind.WAIT, params={"duration_s": 1})
    document = ProtocolDocument(
        protocol_id="bad-doc",
        stages=(ProtocolStage(stage_id="s1", actions=(unsupported, unanchored)),),
        metadata={"coverage": {"unsupported_command_count": 1}},
    )

    result = BioXPControlLib.dry_run().execute_protocol(document, source_path="bad.xml")

    assert result.ok is False
    assert result.lifecycle == ["created", "preflight_failed"]
    assert result.action_count == 2
    assert result.unsupported_action_count == 1
    assert result.unanchored_action_count == 1
    assert result.virtual_state.actions_applied == 0
    assert result.artifact["fail_closed"] is True
    assert result.artifact["preflight_errors"] == [
        "coverage reports unsupported OEM commands",
        "protocol contains unsupported action summaries",
        "protocol contains actions without OEM source anchors",
    ]
