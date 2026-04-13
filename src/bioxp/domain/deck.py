from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .capabilities import CapabilityRegistry
from .labware import LabwareDefinition
from .locations import (
    Coordinate3D,
    LocationReference,
    ResolvedLocation,
    UnknownLocationError,
    UnknownWellError,
    ZERO_COORDINATE,
)


def _normalize_key(value: str) -> str:
    return str(value).strip().lower().replace("-", "_").replace(" ", "_")


def _default_layout_path() -> Path:
    return Path(__file__).resolve().parents[3] / "config" / "deck" / "default_layout.yaml"


DEFAULT_DECK_LAYOUT_PATH = _default_layout_path()


@dataclass(frozen=True)
class DeckSlot:
    """A physical deck slot with an origin and optional attached labware."""

    slot_id: str
    display_name: str | None
    origin: Coordinate3D
    aliases: tuple[str, ...] = ()
    labware: LabwareDefinition | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "DeckSlot":
        slot_id = str(data["slot_id"])
        labware_payload = data.get("labware")
        labware = (
            LabwareDefinition.from_mapping(labware_payload)
            if isinstance(labware_payload, Mapping)
            else None
        )
        metadata = {
            key: value
            for key, value in data.items()
            if key not in {"slot_id", "display_name", "origin", "aliases", "labware"}
        }
        return cls(
            slot_id=slot_id,
            display_name=str(data.get("display_name")) if data.get("display_name") is not None else None,
            origin=Coordinate3D.from_mapping(data.get("origin")),
            aliases=tuple(str(alias) for alias in data.get("aliases", ())),
            labware=labware,
            metadata=metadata,
        )


@dataclass(frozen=True)
class DeckLocation:
    """Semantic location anchored to a slot and optional local offset."""

    location_id: str
    slot_id: str
    display_name: str | None = None
    position: Coordinate3D = ZERO_COORDINATE
    aliases: tuple[str, ...] = ()
    labware_id: str | None = None
    default_well_id: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "DeckLocation":
        location_id = str(data["location_id"])
        slot_id = str(data.get("slot") or data.get("slot_id"))
        metadata = {
            key: value
            for key, value in data.items()
            if key not in {
                "location_id",
                "slot",
                "slot_id",
                "display_name",
                "position",
                "aliases",
                "labware_id",
                "default_well_id",
            }
        }
        default_well = data.get("default_well_id")
        return cls(
            location_id=location_id,
            slot_id=slot_id,
            display_name=str(data.get("display_name")) if data.get("display_name") is not None else None,
            position=Coordinate3D.from_mapping(data.get("position")),
            aliases=tuple(str(alias) for alias in data.get("aliases", ())),
            labware_id=str(data.get("labware_id")) if data.get("labware_id") is not None else None,
            default_well_id=str(default_well).upper() if default_well is not None else None,
            metadata=metadata,
        )


@dataclass(frozen=True)
class DeckLayout:
    """Config-driven semantic deck model."""

    deck_id: str
    version: int
    slots: Mapping[str, DeckSlot] = field(default_factory=dict)
    locations: Mapping[str, DeckLocation] = field(default_factory=dict)
    capabilities: CapabilityRegistry = field(default_factory=CapabilityRegistry)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        unknown_slot_ids = sorted(
            location.location_id
            for location in self.locations.values()
            if location.slot_id not in self.slots
        )
        if unknown_slot_ids:
            missing = ", ".join(unknown_slot_ids)
            raise ValueError(f"Deck locations reference unknown slots: {missing}")
        self._validate_reference_uniqueness()

    def _validate_reference_uniqueness(self) -> None:
        claims: dict[str, set[str]] = {}

        def register(owner: str, raw_values: tuple[str, ...]) -> None:
            for raw_value in raw_values:
                normalized = _normalize_key(raw_value)
                if not normalized:
                    continue
                claims.setdefault(normalized, set()).add(owner)

        for slot in self.slots.values():
            register(f"slot:{slot.slot_id}", (slot.slot_id, *slot.aliases))
        for location in self.locations.values():
            register(
                f"location:{location.location_id}",
                (location.location_id, *location.aliases),
            )

        collisions = {
            normalized: sorted(owners)
            for normalized, owners in claims.items()
            if len(owners) > 1
        }
        if collisions:
            details = "; ".join(
                f"{normalized} -> {', '.join(owners)}"
                for normalized, owners in sorted(collisions.items())
            )
            raise ValueError(f"Deck references must be unique across slots and locations: {details}")

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "DeckLayout":
        slot_entries = data.get("slots", ())
        slots: dict[str, DeckSlot] = {}
        for slot_entry in slot_entries:
            slot = DeckSlot.from_mapping(slot_entry)
            if slot.slot_id in slots:
                raise ValueError(f"Duplicate slot_id '{slot.slot_id}' in deck config")
            slots[slot.slot_id] = slot

        location_entries = data.get("locations", ())
        locations: dict[str, DeckLocation] = {}
        for location_entry in location_entries:
            location = DeckLocation.from_mapping(location_entry)
            if location.location_id in locations:
                raise ValueError(f"Duplicate location_id '{location.location_id}' in deck config")
            locations[location.location_id] = location

        metadata = {
            key: value
            for key, value in data.items()
            if key not in {"deck_id", "version", "slots", "locations", "capabilities", "machine_capabilities"}
        }

        return cls(
            deck_id=str(data.get("deck_id", "default")),
            version=int(data.get("version", 1)),
            slots=slots,
            locations=locations,
            capabilities=CapabilityRegistry.from_config(
                data.get("capabilities") or data.get("machine_capabilities")
            ),
            metadata=metadata,
        )

    def find_location(self, value: str) -> DeckLocation | None:
        normalized = _normalize_key(value)
        for location in self.locations.values():
            if normalized == _normalize_key(location.location_id):
                return location
            if any(normalized == _normalize_key(alias) for alias in location.aliases):
                return location
        return None

    def find_slot(self, value: str) -> DeckSlot | None:
        normalized = _normalize_key(value)
        for slot in self.slots.values():
            if normalized == _normalize_key(slot.slot_id):
                return slot
            if any(normalized == _normalize_key(alias) for alias in slot.aliases):
                return slot
        return None

    def resolve(
        self,
        reference: str | LocationReference,
        *,
        well_id: str | None = None,
    ) -> ResolvedLocation:
        ref = LocationReference.parse(reference)
        requested_well = (
            str(well_id).upper()
            if well_id is not None
            else (str(ref.well_id).upper() if ref.well_id is not None else None)
        )

        location = self.find_location(ref.location_id)
        if location is not None:
            slot = self.slots[location.slot_id]
            base_coordinate = slot.origin + location.position
            effective_location_id = location.location_id
            aliases = location.aliases
            metadata = dict(location.metadata)
            labware = self._labware_for(location, slot)
            requested_well = requested_well or location.default_well_id
        else:
            slot = self.find_slot(ref.location_id)
            if slot is None:
                raise UnknownLocationError(
                    f"Unknown location reference '{ref.location_id}'."
                )
            base_coordinate = slot.origin
            effective_location_id = slot.slot_id
            aliases = slot.aliases
            metadata = dict(slot.metadata)
            labware = slot.labware

        coordinate = base_coordinate
        resolved_labware_id = labware.labware_id if labware is not None else None
        resolved_well_id: str | None = None

        if requested_well is not None:
            if labware is None:
                raise UnknownWellError(
                    f"Location '{effective_location_id}' does not expose labware wells."
                )
            well = labware.resolve_well(requested_well)
            coordinate = coordinate + well.offset
            resolved_well_id = well.well_id
            metadata.setdefault("well_metadata", dict(well.metadata))

        return ResolvedLocation(
            location_id=effective_location_id,
            slot_id=slot.slot_id,
            coordinate=coordinate,
            labware_id=resolved_labware_id,
            well_id=resolved_well_id,
            aliases=aliases,
            metadata=metadata,
        )

    def _labware_for(
        self,
        location: DeckLocation,
        slot: DeckSlot,
    ) -> LabwareDefinition | None:
        if location.labware_id is None:
            return slot.labware
        if slot.labware is None:
            raise UnknownWellError(
                f"Location '{location.location_id}' requests labware '{location.labware_id}', but slot '{slot.slot_id}' has no labware."
            )
        if slot.labware.labware_id != location.labware_id:
            raise UnknownWellError(
                f"Location '{location.location_id}' requests labware '{location.labware_id}', but slot '{slot.slot_id}' is configured with '{slot.labware.labware_id}'."
            )
        return slot.labware


def load_layout_data(path: str | Path | None = None) -> Mapping[str, Any]:
    config_path = Path(path) if path is not None else DEFAULT_DECK_LAYOUT_PATH
    raw_text = config_path.read_text(encoding="utf-8")

    try:
        import yaml  # type: ignore
    except ModuleNotFoundError:
        yaml = None

    if yaml is not None:
        data = yaml.safe_load(raw_text)
    else:
        data = json.loads(raw_text)

    if not isinstance(data, Mapping):
        raise ValueError(f"Deck config '{config_path}' must decode to a mapping")
    return data


def load_deck_layout(path: str | Path | None = None) -> DeckLayout:
    return DeckLayout.from_mapping(load_layout_data(path))
