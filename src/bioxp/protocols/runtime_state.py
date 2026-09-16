from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from .models import ProtocolDocument


class StageExecutionStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    FAILED = "failed"
    COMPLETED = "completed"


@dataclass
class ProtocolExecutionEvent:
    sequence: int
    event: str
    stage_id: str | None = None
    action_id: str | None = None
    detail: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ProtocolExecutionEvent":
        return cls(
            sequence=int(payload.get("sequence", 0)),
            event=str(payload.get("event", "unknown")),
            stage_id=payload.get("stage_id"),
            action_id=payload.get("action_id"),
            detail=dict(payload.get("detail") or {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "sequence": int(self.sequence),
            "event": self.event,
            "stage_id": self.stage_id,
            "action_id": self.action_id,
            "detail": dict(self.detail),
        }


@dataclass
class ProtocolStageState:
    stage_id: str
    title: str | None = None
    status: StageExecutionStatus = StageExecutionStatus.PENDING
    review_required: bool = False
    current_action_id: str | None = None
    completed_actions: list[str] = field(default_factory=list)
    pause_marker_action_id: str | None = None

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ProtocolStageState":
        return cls(
            stage_id=str(payload["stage_id"]),
            title=payload.get("title"),
            status=StageExecutionStatus(str(payload.get("status", StageExecutionStatus.PENDING.value))),
            review_required=bool(payload.get("review_required", False)),
            current_action_id=payload.get("current_action_id"),
            completed_actions=[str(value) for value in payload.get("completed_actions") or []],
            pause_marker_action_id=payload.get("pause_marker_action_id"),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "stage_id": self.stage_id,
            "title": self.title,
            "status": self.status.value,
            "review_required": bool(self.review_required),
            "current_action_id": self.current_action_id,
            "completed_actions": list(self.completed_actions),
            "pause_marker_action_id": self.pause_marker_action_id,
        }


WORKFLOW_PHASES = frozenset({
    "queued", "preparing", "starting", "executing", "waiting", "waking",
    "epilogue", "cleanup", "reconciling", "terminal",
})
WORKFLOW_GATES = frozenset({
    "ordinary_pause", "deferred_pause", "delaypoint", "review", "error_hold",
})


@dataclass
class ProtocolWorkflowState:
    """Canonical workflow projection, never a resumable native checkpoint."""

    command_id: str
    phase: str = "queued"
    gate: str | None = None
    gate_id: str | None = None
    source_occurrence_id: str | None = None
    requested_control: dict[str, str] | None = None
    last_control_id: str | None = None
    reached_control_id: str | None = None
    held_reason: str | None = None
    child_command_ids: list[str] = field(default_factory=list)

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ProtocolWorkflowState":
        fields = cls.__dataclass_fields__
        if set(payload) - set(fields):
            raise ValueError("Unknown workflow state field")
        if not isinstance(payload.get("command_id"), str) or not payload["command_id"]:
            raise ValueError("Workflow requires a canonical command_id")
        if payload.get("phase", "queued") not in WORKFLOW_PHASES:
            raise ValueError("Unknown workflow phase")
        if payload.get("gate") is not None and payload["gate"] not in WORKFLOW_GATES:
            raise ValueError("Unknown workflow gate")
        for name in ("gate_id", "source_occurrence_id", "last_control_id", "reached_control_id", "held_reason"):
            if payload.get(name) is not None and not isinstance(payload[name], str):
                raise ValueError(f"Workflow {name} must be a string or null")
        control = payload.get("requested_control")
        if control is not None:
            if not isinstance(control, dict) or set(control) - {"action", "mode", "gate", "gate_id"}:
                raise ValueError("Invalid workflow requested_control")
            if control.get("action") not in {"pause", "wake", "continue", "safe_stop", "abort"}:
                raise ValueError("Unknown workflow requested control")
            if control.get("action") == "pause" and control.get("mode") not in {"ordinary", "deferred"}:
                raise ValueError("Invalid workflow pause mode")
        children = payload.get("child_command_ids", [])
        if not isinstance(children, list) or any(not isinstance(x, str) or not x for x in children):
            raise ValueError("Invalid workflow child identities")
        values = {key: payload[key] for key in fields if key in payload}
        values["child_command_ids"] = list(children)
        values["requested_control"] = None if control is None else dict(control)
        return cls(**values)

    def to_payload(self) -> dict[str, Any]:
        return {
            "command_id": self.command_id, "phase": self.phase,
            "gate": self.gate, "gate_id": self.gate_id,
            "source_occurrence_id": self.source_occurrence_id,
            "requested_control": None if self.requested_control is None else dict(self.requested_control),
            "last_control_id": self.last_control_id,
            "reached_control_id": self.reached_control_id,
            "held_reason": self.held_reason,
            "child_command_ids": list(self.child_command_ids),
        }


@dataclass
class SourceWell:
    content: str | None
    volume: float
    capacity: float
    empty: bool = True
    zone_index: int | None = None

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "SourceWell":
        if set(payload) - {"content", "volume", "capacity", "empty", "zone_index"}:
            raise ValueError("Unknown prepared well field")
        if type(payload.get("empty", True)) is not bool:
            raise ValueError("Prepared well empty must be boolean")
        return cls(payload.get("content"), float(payload["volume"]), float(payload["capacity"]), payload.get("empty", True), payload.get("zone_index"))

    def to_payload(self) -> dict[str, Any]:
        return {"content": self.content, "volume": self.volume, "capacity": self.capacity, "empty": self.empty, "zone_index": self.zone_index}


@dataclass
class SourceTray:
    tray_id: str
    location: int
    wells: list[SourceWell]
    tip_type: int | None = None
    tray_empty: bool | None = None
    strip_color: str | None = None

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "SourceTray":
        if set(payload) - {"tray_id", "location", "wells", "tip_type", "tray_empty", "strip_color"}:
            raise ValueError("Invalid prepared tray fields")
        return cls(str(payload["tray_id"]), int(payload["location"]), [SourceWell.from_payload(w) for w in payload["wells"]],
                   payload.get("tip_type"), payload.get("tray_empty"), payload.get("strip_color"))

    def to_payload(self) -> dict[str, Any]:
        return {"tray_id": self.tray_id, "location": self.location, "wells": [w.to_payload() for w in self.wells],
                "tip_type": self.tip_type, "tray_empty": self.tray_empty, "strip_color": self.strip_color}


@dataclass
class ProtocolSourceModel:
    """Prepared logical inventory only; no physical positions/reference cache."""

    fluid_name: str | None = None
    tip_zone_index: int | None = None
    old_tip_well: str | None = None
    logical_tip_present: bool | None = None
    carried_plate_present: bool | None = None
    allow_to_stop: bool | None = None
    trays: dict[str, SourceTray] = field(default_factory=dict)
    strips: list[SourceTray] = field(default_factory=list)
    tip_trays: list[SourceTray] = field(default_factory=list)
    pressure_baseline: list[Any] = field(default_factory=list)
    pressure_history: list[list[float]] = field(default_factory=lambda: [[0.0] * 5 for _ in range(4)])

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ProtocolSourceModel":
        if set(payload) - set(cls.__dataclass_fields__):
            raise ValueError("Unknown source model field")
        facts = {key: payload.get(key) for key in ("logical_tip_present", "carried_plate_present", "allow_to_stop")}
        if any(value is not None and type(value) is not bool for value in facts.values()):
            raise ValueError("Source model facts must be boolean or unknown")
        from copy import deepcopy
        return cls(**facts,
                   fluid_name=payload.get("fluid_name"),
                   tip_zone_index=payload.get("tip_zone_index"),
                   old_tip_well=payload.get("old_tip_well"),
                   trays={key: SourceTray.from_payload(value) for key, value in payload.get("trays", {}).items()},
                   strips=[SourceTray.from_payload(t) for t in payload.get("strips", [])],
                   tip_trays=[SourceTray.from_payload(t) for t in payload.get("tip_trays", [])],
                   pressure_baseline=list(payload.get("pressure_baseline", [])),
                   pressure_history=[list(row) for row in payload.get("pressure_history", [[0.0] * 5 for _ in range(4)])])

    def to_payload(self) -> dict[str, Any]:
        from copy import deepcopy
        return {
            "fluid_name": self.fluid_name, "tip_zone_index": self.tip_zone_index,
            "old_tip_well": self.old_tip_well,
            "logical_tip_present": self.logical_tip_present,
            "carried_plate_present": self.carried_plate_present, "allow_to_stop": self.allow_to_stop,
            "trays": {key: tray.to_payload() for key, tray in self.trays.items()},
            "strips": [t.to_payload() for t in self.strips],
            "tip_trays": [t.to_payload() for t in self.tip_trays],
            "pressure_baseline": deepcopy(self.pressure_baseline),
            "pressure_history": deepcopy(self.pressure_history),
        }

    @staticmethod
    def _well_index(well: str | int) -> int:
        if isinstance(well, int):
            return well
        return (ord(well[0]) - ord("A")) * 12 + int(well[1:]) - 1

    def _fluid_tray(self, location: int, current_tray: int | str | None) -> SourceTray | None:
        if current_tray is not None:
            names = ("POOL_PLATE", "OUTPUT_PLATE", "REAGENT_PLATE", "BIO_SECURITY_COVER",
                     "OUTPUT_COVER", "REAGENT_COVER", "TIP_TRAY", "STRIP_ONE", "STRIP_TWO",
                     "STRIP_THREE", "STRIP_FOUR", "TROUGH", "SYNTHESIS_PLATE",
                     "OLIGO_QUANTITATION_PLATE", "GENE_QUANTITATION_PLATE", "REF_QUANTITATION_PLATE",
                     "ELUTION_PLATE", "ACCUMULATION_PLATE", "TIP_HOTEL", "TFF_REAGENT_BLOCK",
                     "VOLUME_CALCULATION", "WASTE_BIN")
            key = names[current_tray] if isinstance(current_tray, int) and 0 <= current_tray < len(names) else str(current_tray)
            return self.trays.get(key) if "STRIP" not in key else None
        return next((t for key, t in self.trays.items() if "STRIP" not in key and t.location == location), None)

    def _fluid_strip(self, location: int) -> SourceTray | None:
        selected = None
        if 11 <= location <= 14:
            for strip in self.strips:
                if strip.strip_color is not None:
                    if strip.strip_color.upper() == "X":
                        selected = self.strips[location - 11]
                    elif strip.location == location:
                        selected = strip
        return selected

    def current_well_volume(self, location: int, well: str | int, *, current_tray: int | str | None = None) -> float:
        tray = self._fluid_tray(location, current_tray)
        index = self._well_index(well)
        if tray is not None:
            return tray.wells[index].volume
        strip = self._fluid_strip(location)
        return strip.wells[index // 12].volume if strip is not None else 0.0

    def update_fluid_name(self, location: int, well: str | int, *, current_tray: int | str | None = None) -> None:
        tray = self._fluid_tray(location, current_tray)
        self.fluid_name = tray.wells[self._well_index(well)].content if tray is not None else None

    def update_fluid_level(self, location: int, well: str | int, tip_location: int, delta: float,
                           *, current_tray: int | str | None = None) -> None:
        import struct
        delta = struct.unpack("f", struct.pack("f", float(delta)))[0]
        if location == 16:
            return
        tray = self._fluid_tray(location, current_tray)
        if tray is None:
            tray = self._fluid_tray(location, None)
        index = self._well_index(well)
        if tray is not None:
            for offset in ((0, 24, 48, 72) if tip_location == -1 else (0,)):
                item = tray.wells[index + offset]
                if delta > 0:
                    if item.volume + delta <= item.capacity:
                        if self.fluid_name and item.content is None:
                            item.content = self.fluid_name
                        item.volume += delta
                elif -delta <= item.volume:
                    item.volume += delta
            return
        strip = self._fluid_strip(location)
        # Literal strip addvolume permits negative volume; absent selected strip
        # raises rather than manufacturing an inventory or a successful no-op.
        for offset in (0, 2, 4, 6):
            item = strip.wells[(0 if index == 0 else 1) + offset]
            if item.volume + delta <= item.capacity:
                item.volume += delta

    def select_tip(self, tip_type: int, pipette: int = -1) -> tuple[int, int, str] | None:
        for tray_index, tray in enumerate(self.tip_trays):
            if tray.location == 15 or tray.tip_type != tip_type:
                continue
            selected = None
            for column in range(12):
                for row in range(2):
                    start = column + row * 12 + (0 if pipette == -1 else pipette * 24)
                    indices = [start + offset for offset in ((0, 24, 48, 72) if pipette == -1 else (0,))]
                    if any(tray.wells[i].empty for i in indices):
                        continue
                    # NextLabeledTip keeps val3 even if searchZoneIndex fails.
                    if pipette != -1:
                        selected = start
                    zone = self.tip_zone_index
                    later = zone is not None and any(not w.empty and w.zone_index == zone for w in tray.wells[start:])
                    if later:
                        matches = all(tray.wells[i].zone_index == zone + n * 2 for n, i in enumerate(indices))
                    else:
                        matches = all(tray.wells[i].zone_index is None for i in indices)
                    if matches:
                        selected = start
                        break
                else:
                    continue
                break
            if selected is not None:
                return tray_index, tray.location, f"{'AB'[(selected // 12) % 2]}{selected % 12 + 1}"
        return None

    def tip_hotel_empty(self) -> bool:
        return all(self.tip_trays[4].wells[i].empty for i in (84, 60, 36, 12))

    def next_hotel_tip(self) -> str:
        tray = self.tip_trays[4]
        for index in (84, 60, 36, 12):
            if not tray.wells[index].empty:
                return f"{chr(ord('A') + index // 12)}1"
        for well in tray.wells:
            well.empty = True
        tray.tip_type = 201
        tray.tray_empty = True
        return "H1"

    def hotel_loaded(self) -> None:
        tray = self.tip_trays[4]
        for well in tray.wells:
            well.empty = False
            well.content = None
        tray.tray_empty = False

    def tip_removed(self, tray_index: int, well_name: str, pipette: int = -1, hotel: bool = False) -> None:
        tray = self.tip_trays[tray_index]
        start = self._well_index(well_name)
        indices = ([start] if hotel else [start + pipette * 24] if pipette != -1
                   else [start + offset for offset in (0, 24, 48, 72)])
        for index in indices:
            tray.wells[index].empty = True
            tray.wells[index].content = None

    def tip_restored(self, tray_index: int, well: str | int, label: str | None, zone_index: int | None) -> None:
        # MachineStatus forwards its TipZoneIndex, not the supplied argument.
        start = self._well_index(well)
        offsets = (0, 24, 48, 72) if isinstance(well, str) else (0,)
        for n, offset in enumerate(offsets):
            item = self.tip_trays[tray_index].wells[start + offset]
            item.content = label
            if self.tip_zone_index is not None:
                item.zone_index = self.tip_zone_index + n * 2
        # restoretip labels only; retip separately restores logical availability.

    def la(self, tokens: list[Any]) -> list[Any]:
        # ClassMachineStatus.la -> ClassTray -> ClassWellCollection. The
        # source parses Single, ignores per-well capacity/underflow return codes
        # and mutates grouped wells in order; it does not clamp or roll back.
        import struct
        volume = struct.unpack("f", struct.pack("f", float(tokens[4])))[0]
        plates = (
            "POOL_PLATE", "OUTPUT_PLATE", "REAGENT_PLATE", "BIO_SECURITY_COVER",
            "OUTPUT_COVER", "REAGENT_COVER", "TIP_TRAY", "STRIP_ONE", "STRIP_TWO",
            "STRIP_THREE", "STRIP_FOUR", "TROUGH", "SYNTHESIS_PLATE",
            "OLIGO_QUANTITATION_PLATE", "GENE_QUANTITATION_PLATE", "REF_QUANTITATION_PLATE",
            "ELUTION_PLATE", "ACCUMULATION_PLATE", "TIP_HOTEL", "TFF_REAGENT_BLOCK",
            "VOLUME_CALCULATION", "WASTE_BIN",
        )
        plate = tokens[2].upper()
        try:
            number = int(plate)
            plate = plates[number] if 0 <= number < len(plates) else plate
        except ValueError:
            if plate not in plates:
                return [tokens[0]]
        if "STRIP" in plate or "COVER" in plate:
            raise ValueError("la() - Invalid plate.")
        tray = self.trays[plate]  # Unknown captured inventory is not an empty tray.
        well = tokens[3].upper()
        try:
            index = int(well)
        except ValueError:
            if well == "UNKNOWN":
                index = 96
            elif well[:1] in "ABCDEFGH" and well[1:].isdigit() and 1 <= int(well[1:]) <= 12:
                index = (ord(well[0]) - ord("A")) * 12 + int(well[1:]) - 1
            else:
                return [tokens[0]]
        offsets = (0,) if len(tokens) > 5 and tokens[5] == "F" else (0, 24, 48, 72)
        for offset in offsets:
            if index + offset < 0:
                raise IndexError("Invalid source well")
            item = tray.wells[index + offset]
            if volume <= 0:
                if item.volume - volume <= item.capacity:
                    item.volume -= volume
            elif volume <= item.volume:
                item.volume -= volume
        return [tokens[0]]

    def add_pressure_base(self, values: list[float]) -> None:
        if len(values) != 4:
            raise ValueError("Source pressure baseline requires four channels")
        self.pressure_baseline = list(values)
        for channel, value in enumerate(values):
            self.pressure_history[channel] = self.pressure_history[channel][1:] + [value]

    def get_tip_tray_location(self, tray_index: int) -> int:
        return self.tip_trays[tray_index].location

    def sweep_locations(self, tray_index: int, clearall: bool = False) -> list[str] | None:
        wells = self.tip_trays[tray_index].wells
        selected = []
        for column in range(12):
            for row in range(2):
                group = [wells[(row + offset) * 12 + column].content == "rm" for offset in (0, 2, 4, 6)]
                if (any(group) if clearall else all(group)):
                    selected.append(f"{chr(ord('A') + row)}{column + 1}")
        return selected or None

    def select_strip(self, material: str, volume: float) -> tuple[int, int]:
        # Four source strips, two candidate rows, exact source tolerance and
        # unmatched strip-zero/row-minus-one return (not a fallback selection).
        if len(self.strips) != 4:
            raise ValueError("Four captured source strips required")
        for strip in self.strips:
            for row in (0, 1):
                well = strip.wells[row]
                if well.content == material and well.volume + 0.001 >= volume:
                    return strip.location, row
        return self.strips[0].location, -1

    def retip_wells(self) -> list[tuple[str, list[int]]]:
        return [(tray.tray_id, [i for i, well in enumerate(tray.wells) if well.content == "Reuse" and well.empty])
                for tray in self.tip_trays if tray.location != 15]

    def retip_committed(self, tray_id: str, well_ids: list[int]) -> None:
        tray = next(t for t in self.tip_trays if t.tray_id == tray_id and t.location != 15)
        for index in well_ids:
            well = tray.wells[index]
            if well.content == "Reuse" and well.empty:
                well.empty = False


@dataclass
class ProtocolRuntimeState:
    protocol_id: str
    dry_run: bool
    job_id: str | None = None
    current_stage_id: str | None = None
    paused: bool = False
    awaiting_review: bool = False
    completed: bool = False
    pause_reason: str | None = None
    stage_states: dict[str, ProtocolStageState] = field(default_factory=dict)
    events: list[ProtocolExecutionEvent] = field(default_factory=list)
    action_results: list[dict[str, Any]] = field(default_factory=list)
    workflow: ProtocolWorkflowState | None = None
    source_model: ProtocolSourceModel = field(default_factory=ProtocolSourceModel)

    @classmethod
    def from_document(
        cls,
        document: ProtocolDocument,
        *,
        dry_run: bool,
        job_id: str | None = None,
    ) -> "ProtocolRuntimeState":
        return cls(
            protocol_id=document.protocol_id,
            dry_run=bool(dry_run),
            job_id=job_id,
            stage_states={
                stage.stage_id: ProtocolStageState(
                    stage_id=stage.stage_id,
                    title=stage.title,
                    review_required=bool(stage.review_required),
                )
                for stage in document.stages
            },
        )

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ProtocolRuntimeState":
        boolean_values: dict[str, bool] = {}
        for field in ("dry_run", "paused", "awaiting_review", "completed"):
            value = payload.get(field, False)
            if type(value) is not bool:
                raise ValueError(f"runtime state field '{field}' must be a boolean")
            boolean_values[field] = value
        return cls(
            protocol_id=str(payload["protocol_id"]),
            dry_run=boolean_values["dry_run"],
            job_id=payload.get("job_id"),
            current_stage_id=payload.get("current_stage_id"),
            paused=boolean_values["paused"],
            awaiting_review=boolean_values["awaiting_review"],
            completed=boolean_values["completed"],
            pause_reason=payload.get("pause_reason"),
            stage_states={
                str(stage_id): ProtocolStageState.from_payload(stage_payload)
                for stage_id, stage_payload in (payload.get("stage_states") or {}).items()
            },
            events=[
                ProtocolExecutionEvent.from_payload(event_payload)
                for event_payload in payload.get("events") or []
            ],
            action_results=[dict(entry) for entry in payload.get("action_results") or []],
            workflow=(ProtocolWorkflowState.from_payload(payload["workflow"])
                      if payload.get("workflow") is not None else None),
            source_model=ProtocolSourceModel.from_payload(payload.get("source_model") or {}),
        )

    def record_event(
        self,
        event: str,
        *,
        stage_id: str | None = None,
        action_id: str | None = None,
        detail: dict[str, Any] | None = None,
    ) -> None:
        self.events.append(
            ProtocolExecutionEvent(
                sequence=len(self.events) + 1,
                event=event,
                stage_id=stage_id,
                action_id=action_id,
                detail=dict(detail or {}),
            )
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "protocol_id": self.protocol_id,
            "dry_run": bool(self.dry_run),
            "job_id": self.job_id,
            "workflow": None if self.workflow is None else self.workflow.to_payload(),
            "source_model": self.source_model.to_payload(),
            "current_stage_id": self.current_stage_id,
            "paused": bool(self.paused),
            "awaiting_review": bool(self.awaiting_review),
            "completed": bool(self.completed),
            "pause_reason": self.pause_reason,
            "stage_states": {
                stage_id: state.to_payload()
                for stage_id, state in self.stage_states.items()
            },
            "action_results": [dict(entry) for entry in self.action_results],
            "events": [event.to_payload() for event in self.events],
        }
