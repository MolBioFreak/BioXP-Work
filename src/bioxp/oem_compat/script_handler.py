from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..protocols.models import ProtocolAction, ProtocolActionKind, ProtocolDocument
from .pipette import PipetteController, PipettePlan
from .vision import VisionFacade, VisionResult


@dataclass(frozen=True)
class ScriptExecutionStep:
    index: int
    action_id: str
    stage_id: str
    layer: str
    oem_layer: str
    action_kind: str
    oem_verb: str | None
    raw_cmd: str | None
    source_node: dict[str, Any]
    params: dict[str, Any] = field(default_factory=dict)
    requires_virtual_bioxp_state: bool = False
    requires_ack_readback: bool = False
    pipette_plan: PipettePlan | None = None
    vision_result: VisionResult | None = None
    live_blockers: list[str] = field(default_factory=list)

    def to_payload(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "action_id": self.action_id,
            "stage_id": self.stage_id,
            "layer": self.layer,
            "oem_layer": self.oem_layer,
            "action_kind": self.action_kind,
            "oem_verb": self.oem_verb,
            "raw_cmd": self.raw_cmd,
            "source_node": dict(self.source_node),
            "params": dict(self.params),
            "requires_virtual_bioxp_state": self.requires_virtual_bioxp_state,
            "requires_ack_readback": self.requires_ack_readback,
            "pipette_plan": None if self.pipette_plan is None else self.pipette_plan.to_payload(),
            "vision_result": None if self.vision_result is None else self.vision_result.to_payload(),
            "live_blockers": list(self.live_blockers),
        }


@dataclass(frozen=True)
class ScriptExecutionPlan:
    mode: str
    action_count: int
    steps: list[ScriptExecutionStep]
    unsupported_action_count: int = 0
    unanchored_action_count: int = 0

    @property
    def step_count(self) -> int:
        return len(self.steps)

    @property
    def source_anchored(self) -> bool:
        return self.unanchored_action_count == 0

    @property
    def oem_layers(self) -> list[str]:
        seen: list[str] = []
        for step in self.steps:
            if step.oem_layer not in seen:
                seen.append(step.oem_layer)
        return seen

    @property
    def live_blockers(self) -> list[str]:
        blockers: list[str] = []
        for step in self.steps:
            for blocker in step.live_blockers:
                if blocker not in blockers:
                    blockers.append(blocker)
        return blockers

    def to_payload(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "action_count": self.action_count,
            "step_count": self.step_count,
            "unsupported_action_count": self.unsupported_action_count,
            "unanchored_action_count": self.unanchored_action_count,
            "source_anchored": self.source_anchored,
            "oem_layers": self.oem_layers,
            "live_blockers": self.live_blockers,
            "steps": [step.to_payload() for step in self.steps],
        }


class BioXPScriptHandler:
    """ClassBioXPScriptHandler-shaped dry-run interpreter.

    This layer converts already-imported OEM actions into source-anchored runtime
    steps that downstream ControlLib/VirtualBioXP/pipette/vision gates can reason
    about. It is intentionally not an executor substitute: live blockers remain explicit.
    """

    def __init__(self, *, mode: str = "dry_run"):
        self.mode = mode
        self.pipette = PipetteController.dry_run()
        self.vision = VisionFacade.dry_run()

    @classmethod
    def dry_run(cls) -> "BioXPScriptHandler":
        return cls(mode="dry_run")

    def interpret_protocol(self, document: ProtocolDocument) -> ScriptExecutionPlan:
        actions = [action for stage in document.stages for action in stage.actions]
        steps: list[ScriptExecutionStep] = []
        unsupported = 0
        unanchored = 0
        for action in actions:
            if self._is_unsupported_summary(action):
                unsupported += 1
            if not self._has_oem_source_anchor(action):
                unanchored += 1
            steps.append(self._step_for_action(len(steps), action))
        return ScriptExecutionPlan(
            mode=self.mode,
            action_count=len(actions),
            steps=steps,
            unsupported_action_count=unsupported,
            unanchored_action_count=unanchored,
        )

    @staticmethod
    def _has_oem_source_anchor(action: ProtocolAction) -> bool:
        metadata = dict(action.metadata or {})
        source_node = metadata.get("source_node")
        return bool(metadata.get("oem_verb") and metadata.get("raw_cmd") and isinstance(source_node, dict) and source_node)

    @staticmethod
    def _is_unsupported_summary(action: ProtocolAction) -> bool:
        params = dict(action.params or {})
        metadata = dict(action.metadata or {})
        return bool(action.kind is ProtocolActionKind.NOTE and (params.get("unsupported_command_count") or metadata.get("unsupported_commands")))

    def _step_for_action(self, index: int, action: ProtocolAction) -> ScriptExecutionStep:
        metadata = dict(action.metadata or {})
        params = dict(action.params or {})
        verb = metadata.get("oem_verb") or params.get("macro_verb")
        raw_cmd = metadata.get("raw_cmd")
        source_node = metadata.get("source_node") if isinstance(metadata.get("source_node"), dict) else {}
        layer = self._layer_for_action(action)
        pipette_plan = self._pipette_plan_for_action(action) if layer == "ClassPipetteCollection" or action.kind in {
            ProtocolActionKind.PIPETTE_INIT,
            ProtocolActionKind.PIPETTE_TIP,
            ProtocolActionKind.PIPETTE_ASPIRATE,
            ProtocolActionKind.PIPETTE_DISPENSE,
            ProtocolActionKind.PIPETTE_MIX,
            ProtocolActionKind.TIP_EJECT,
        } else None
        vision_result = self._vision_result_for_action(action) if layer == "CVisionLib" else None
        live_blockers = self._live_blockers_for_action(action, layer, pipette_plan, vision_result)
        return ScriptExecutionStep(
            index=index,
            action_id=action.action_id,
            stage_id=action.stage_id,
            layer=layer,
            oem_layer=layer,
            action_kind=action.kind.value,
            oem_verb=None if verb is None else str(verb),
            raw_cmd=None if raw_cmd is None else str(raw_cmd),
            source_node=source_node,
            params=params,
            requires_virtual_bioxp_state=bool(params.get("requires_virtual_bioxp_state") or action.kind in {ProtocolActionKind.PLATE_MOVE, ProtocolActionKind.MOVE_COVER, ProtocolActionKind.PLATE_PREPARE}),
            requires_ack_readback=bool(params.get("requires_ack_readback") or pipette_plan is not None),
            pipette_plan=pipette_plan,
            vision_result=vision_result,
            live_blockers=live_blockers,
        )

    @staticmethod
    def _layer_for_action(action: ProtocolAction) -> str:
        if action.review_required or action.kind is ProtocolActionKind.PAUSE_REVIEW:
            return "operator_gate"
        if action.kind in {ProtocolActionKind.BARCODE_READ, ProtocolActionKind.INSPECT}:
            return "CVisionLib"
        if action.kind in {
            ProtocolActionKind.PIPETTE_INIT,
            ProtocolActionKind.PIPETTE_TIP,
            ProtocolActionKind.PIPETTE_ASPIRATE,
            ProtocolActionKind.PIPETTE_DISPENSE,
            ProtocolActionKind.PIPETTE_MIX,
            ProtocolActionKind.TIP_EJECT,
            ProtocolActionKind.LIQUID_ADJUST,
        }:
            return "ClassBioXPScriptHandler" if dict(action.params or {}).get("macro_verb") else "ClassPipetteCollection"
        if action.kind in {ProtocolActionKind.PLATE_MOVE, ProtocolActionKind.MOVE_COVER, ProtocolActionKind.PLATE_PREPARE}:
            return "ClassVirtualBioXP"
        return "ControlLib"

    def _pipette_plan_for_action(self, action: ProtocolAction) -> PipettePlan | None:
        params = dict(action.params or {})
        semantic = str(params.get("semantic_action") or action.kind.value)
        if params.get("macro_verb"):
            return self.pipette.macro_plan(
                semantic_action=semantic,
                material_id=params.get("material_id") or params.get("wash_material_id"),
                volume_ul=params.get("volume_ul"),
                source=params.get("source_location_id"),
                destination=params.get("dest_location_id") or params.get("target_location_id") or params.get("location_id"),
            )
        if action.kind is ProtocolActionKind.PIPETTE_INIT:
            return self.pipette.initiate_group()
        if action.kind is ProtocolActionKind.PIPETTE_ASPIRATE:
            return self.pipette.aspirate(params.get("volume_ul") or 0, source=params.get("source_location_id"))
        if action.kind is ProtocolActionKind.PIPETTE_DISPENSE:
            return self.pipette.dispense(params.get("volume_ul") or 0, destination=params.get("dest_location_id"))
        if action.kind is ProtocolActionKind.PIPETTE_MIX:
            return self.pipette.mix_all(count=params.get("mix_repeat") or params.get("repeat_count") or 1, vol=params.get("volume_ul") or 0)
        if action.kind is ProtocolActionKind.TIP_EJECT:
            return self.pipette.eject_tip()
        if action.kind is ProtocolActionKind.LIQUID_ADJUST:
            return self.pipette.macro_plan(semantic_action=semantic, material_id=params.get("plate_id"), volume_ul=params.get("volume_ul"), destination=params.get("zone"))
        return None

    def _vision_result_for_action(self, action: ProtocolAction) -> VisionResult | None:
        params = dict(action.params or {})
        location_id = params.get("location_id") or params.get("target_location")
        if action.kind is ProtocolActionKind.BARCODE_READ:
            return self.vision.scan_barcode(location_id=None if location_id is None else str(location_id))
        if action.kind is ProtocolActionKind.INSPECT:
            return self.vision.inspect_pool_plate(location_id=None if location_id is None else str(location_id))
        return None

    @staticmethod
    def _live_blockers_for_action(
        action: ProtocolAction,
        layer: str,
        pipette_plan: PipettePlan | None,
        vision_result: VisionResult | None,
    ) -> list[str]:
        blockers: list[str] = []
        if action.review_required or layer == "operator_gate":
            blockers.append("operator_review_required")
        if pipette_plan is not None and pipette_plan.requires_readback:
            blockers.append("pipette ACK/readback live validation not complete")
        if vision_result is not None:
            blockers.append("vision live validation not complete")
        return blockers
