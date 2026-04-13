from __future__ import annotations

from ..domain.capabilities import CapabilityName
from .models import ProtocolActionKind, ProtocolDocument


_ACTION_CAPABILITY_MAP: dict[ProtocolActionKind, CapabilityName | None] = {
    ProtocolActionKind.MOVE: CapabilityName.MOTION,
    ProtocolActionKind.HOME: CapabilityName.MOTION,
    ProtocolActionKind.PIPETTE_INIT: CapabilityName.PIPETTE,
    ProtocolActionKind.PIPETTE_TIP: CapabilityName.PIPETTE,
    ProtocolActionKind.PIPETTE_ASPIRATE: CapabilityName.PIPETTE,
    ProtocolActionKind.PIPETTE_DISPENSE: CapabilityName.PIPETTE,
    ProtocolActionKind.PIPETTE_MIX: CapabilityName.PIPETTE,
    ProtocolActionKind.INSPECT: CapabilityName.INSPECTION,
    ProtocolActionKind.BARCODE_READ: CapabilityName.BARCODE,
    ProtocolActionKind.PAUSE_REVIEW: None,
    ProtocolActionKind.NOTE: None,
    ProtocolActionKind.LED: None,
    ProtocolActionKind.WAIT: None,
    ProtocolActionKind.PLATE_PREPARE: CapabilityName.MOTION,
    ProtocolActionKind.PLATE_MOVE: CapabilityName.MOTION,
    ProtocolActionKind.THERMAL_DOOR: CapabilityName.THERMAL,
    ProtocolActionKind.MOVE_COVER: CapabilityName.MOTION,
    ProtocolActionKind.CHILLER_SETPOINT: CapabilityName.CHILLER,
    ProtocolActionKind.THERMAL_SETPOINT: CapabilityName.THERMAL,
    ProtocolActionKind.LOOP_MARKER: None,
    ProtocolActionKind.SEAL_SEPARATE: CapabilityName.MOTION,
    ProtocolActionKind.LIQUID_ADJUST: CapabilityName.PIPETTE,
    ProtocolActionKind.TIP_EJECT: CapabilityName.PIPETTE,
}


def infer_required_capability(kind: ProtocolActionKind) -> CapabilityName | None:
    return _ACTION_CAPABILITY_MAP.get(kind)


def validate_protocol_document(document: ProtocolDocument) -> ProtocolDocument:
    if not document.stages:
        raise ValueError("Protocol document must include at least one stage.")

    stage_ids: set[str] = set()
    action_ids: set[str] = set()
    for stage in document.stages:
        if stage.stage_id in stage_ids:
            raise ValueError(f"Duplicate stage_id '{stage.stage_id}' in protocol document")
        stage_ids.add(stage.stage_id)

        if not stage.actions:
            raise ValueError(f"Stage '{stage.stage_id}' must include at least one action")

        for action in stage.actions:
            if action.stage_id != stage.stage_id:
                raise ValueError(
                    f"Action '{action.action_id}' is attached to stage '{action.stage_id}', expected '{stage.stage_id}'"
                )
            if action.action_id in action_ids:
                raise ValueError(f"Duplicate action_id '{action.action_id}' in protocol document")
            action_ids.add(action.action_id)

    return document
