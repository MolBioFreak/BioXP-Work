from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from ..protocols.models import ProtocolAction, ProtocolActionKind


@dataclass
class MaterialState:
    material_id: str
    location_id: str | None = None
    zone: str | None = None
    volume_ul: float | None = None
    source_location_id: str | None = None
    source_zone: str | None = None
    tip_type: str | None = None
    tip_option: str | None = None
    wash_count: int | None = None
    mix_repeat: int | None = None
    repeat_count: int | None = None
    last_operation: str | None = None
    operations: list[dict[str, Any]] = field(default_factory=list)

    def to_payload(self) -> dict[str, Any]:
        return {
            "material_id": self.material_id,
            "location_id": self.location_id,
            "zone": self.zone,
            "volume_ul": self.volume_ul,
            "source_location_id": self.source_location_id,
            "source_zone": self.source_zone,
            "tip_type": self.tip_type,
            "tip_option": self.tip_option,
            "wash_count": self.wash_count,
            "mix_repeat": self.mix_repeat,
            "repeat_count": self.repeat_count,
            "last_operation": self.last_operation,
            "operations": list(self.operations),
        }


@dataclass(frozen=True)
class VirtualStateEvent:
    action_id: str
    verb: str | None
    semantic_action: str
    status: str
    physical_motion: bool = False
    state_delta: Mapping[str, Any] = field(default_factory=dict)

    def to_payload(self) -> dict[str, Any]:
        return {
            "action_id": self.action_id,
            "verb": self.verb,
            "semantic_action": self.semantic_action,
            "status": self.status,
            "physical_motion": self.physical_motion,
            "state_delta": dict(self.state_delta),
        }


@dataclass
class VirtualBioXP:
    """Deterministic ClassVirtualBioXP-style state ledger for OEM dry-runs.

    This is intentionally semantic and fail-closed: it records what an OEM XML
    action would do to deck/material state while never claiming physical motion.
    """

    gantry_location: str | None = None
    plate_on_gantry: str | None = None
    tips_loaded: dict[int, bool] = field(default_factory=dict)
    fluids: dict[str, Any] = field(default_factory=dict)
    materials: dict[str, MaterialState] = field(default_factory=dict)
    plates: dict[str, str | None] = field(default_factory=dict)
    covers: dict[str, str | None] = field(default_factory=dict)
    thermal_doors: dict[str, str | None] = field(default_factory=dict)
    chiller_setpoints: dict[str, float | None] = field(default_factory=dict)
    thermal_setpoints: list[dict[str, Any]] = field(default_factory=list)
    events: list[VirtualStateEvent] = field(default_factory=list)
    actions_applied: int = 0

    def update_gantry_location(self, location: str) -> None:
        self.gantry_location = str(location)

    def update_tip_status(self, channel: int, loaded: bool) -> None:
        self.tips_loaded[int(channel)] = bool(loaded)

    def apply_action(self, action: ProtocolAction) -> VirtualStateEvent:
        params = dict(action.params or {})
        metadata = dict(action.metadata or {})
        semantic = str(params.get("semantic_action") or action.kind.value)
        verb = metadata.get("oem_verb") or params.get("macro_verb")
        delta: dict[str, Any] = {}

        if action.kind in {
            ProtocolActionKind.PIPETTE_ASPIRATE,
            ProtocolActionKind.PIPETTE_DISPENSE,
            ProtocolActionKind.PIPETTE_MIX,
            ProtocolActionKind.PIPETTE_INIT,
            ProtocolActionKind.LIQUID_ADJUST,
        }:
            delta.update(self._apply_liquid_action(semantic, params))
        elif action.kind is ProtocolActionKind.PLATE_MOVE:
            plate_id = str(params.get("plate_id"))
            target = params.get("target_location")
            self.plates[plate_id] = None if target is None else str(target)
            self.plate_on_gantry = plate_id if target == "GANTRY" else self.plate_on_gantry
            delta = {"plate_id": plate_id, "target_location": target}
        elif action.kind is ProtocolActionKind.MOVE_COVER:
            cover_id = str(params.get("cover_id"))
            target = params.get("target_location")
            self.covers[cover_id] = None if target is None else str(target)
            delta = {"cover_id": cover_id, "target_location": target}
        elif action.kind is ProtocolActionKind.PLATE_PREPARE:
            plate_id = str(params.get("plate_id"))
            self.plates.setdefault(plate_id, None)
            delta = {"plate_id": plate_id, "prepared": True}
        elif action.kind is ProtocolActionKind.THERMAL_DOOR:
            channel = str(params.get("channel") or "default")
            self.thermal_doors[channel] = None if params.get("door_state") is None else str(params.get("door_state"))
            delta = {"channel": channel, "door_state": self.thermal_doors[channel]}
        elif action.kind is ProtocolActionKind.CHILLER_SETPOINT:
            channel = str(params.get("channel") or "default")
            self.chiller_setpoints[channel] = params.get("setpoint_c")
            delta = {"channel": channel, "setpoint_c": params.get("setpoint_c")}
        elif action.kind is ProtocolActionKind.THERMAL_SETPOINT:
            payload = dict(params)
            self.thermal_setpoints.append(payload)
            delta = payload
        elif action.kind is ProtocolActionKind.TIP_EJECT:
            for channel in list(self.tips_loaded):
                self.tips_loaded[channel] = False
            delta = {"tips_loaded": dict(self.tips_loaded)}
        elif action.kind in {ProtocolActionKind.WAIT, ProtocolActionKind.LOOP_MARKER, ProtocolActionKind.LED, ProtocolActionKind.PAUSE_REVIEW, ProtocolActionKind.NOTE, ProtocolActionKind.SEAL_SEPARATE}:
            delta = {"observed": True}
        else:
            delta = {"unmodeled_kind": action.kind.value}

        self.actions_applied += 1
        event = VirtualStateEvent(
            action_id=action.action_id,
            verb=None if verb is None else str(verb),
            semantic_action=semantic,
            status="planned_state_applied",
            physical_motion=False,
            state_delta=delta,
        )
        self.events.append(event)
        return event

    def _material(self, material_id: str | None) -> MaterialState | None:
        if not material_id:
            return None
        material_key = str(material_id)
        material = self.materials.get(material_key)
        if material is None:
            material = MaterialState(material_id=material_key)
            self.materials[material_key] = material
        return material

    def _apply_liquid_action(self, semantic: str, params: Mapping[str, Any]) -> dict[str, Any]:
        material = self._material(
            params.get("material_id")
            or params.get("wash_material_id")
            or params.get("source_well_id")
            or params.get("well_id")
        )
        if material is None:
            return {"semantic_action": semantic, "material_id": None}

        target_loc = params.get("dest_location_id") or params.get("target_location_id") or params.get("location_id")
        target_zone = params.get("dest_zone") or params.get("target_zone") or params.get("zone")
        source_loc = params.get("source_location_id")
        source_zone = params.get("source_zone")

        if target_loc is not None:
            material.location_id = str(target_loc)
        if target_zone is not None:
            material.zone = str(target_zone)
        if source_loc is not None:
            material.source_location_id = str(source_loc)
        if source_zone is not None:
            material.source_zone = str(source_zone)
        if params.get("volume_ul") is not None:
            material.volume_ul = float(params["volume_ul"])
        if params.get("tip_type") is not None:
            material.tip_type = str(params["tip_type"])
        if params.get("tip_option") is not None:
            material.tip_option = str(params["tip_option"])
        if params.get("wash_count") is not None:
            material.wash_count = int(params["wash_count"])
        if params.get("mix_repeat") is not None:
            material.mix_repeat = int(params["mix_repeat"])
        if params.get("repeat_count") is not None:
            material.repeat_count = int(params["repeat_count"])
        material.last_operation = semantic
        op = {"semantic_action": semantic, "params": dict(params)}
        material.operations.append(op)
        return {"material": material.to_payload()}

    def to_payload(self) -> dict[str, Any]:
        return {
            "gantry_location": self.gantry_location,
            "plate_on_gantry": self.plate_on_gantry,
            "tips_loaded": dict(self.tips_loaded),
            "fluids": dict(self.fluids),
            "materials": {key: value.to_payload() for key, value in sorted(self.materials.items())},
            "plates": dict(sorted(self.plates.items())),
            "covers": dict(sorted(self.covers.items())),
            "thermal_doors": dict(sorted(self.thermal_doors.items())),
            "chiller_setpoints": dict(sorted(self.chiller_setpoints.items())),
            "thermal_setpoints": list(self.thermal_setpoints),
            "actions_applied": int(self.actions_applied),
            "events": [event.to_payload() for event in self.events],
        }


# Backward-compatible name used by earlier workstation dry-run code/tests.
MachineState = VirtualBioXP
