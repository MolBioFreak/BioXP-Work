from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from .locations import Coordinate3D, UnknownWellError


@dataclass(frozen=True)
class WellDefinition:
    """A named well offset within a piece of labware."""

    well_id: str
    offset: Coordinate3D
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(
        cls,
        well_id: str,
        data: Mapping[str, Any] | None,
    ) -> "WellDefinition":
        payload = dict(data or {})
        coordinate_keys = {"x", "y", "z"}
        if coordinate_keys.issubset(payload.keys()):
            offset = Coordinate3D.from_mapping(payload)
            metadata = {}
        else:
            offset = Coordinate3D.from_mapping(payload.get("offset"))
            metadata = {
                key: value
                for key, value in payload.items()
                if key != "offset"
            }

        return cls(well_id=well_id.upper(), offset=offset, metadata=metadata)


@dataclass(frozen=True)
class LabwareDefinition:
    """Minimal typed labware model for deck resolution."""

    labware_id: str
    kind: str
    display_name: str | None = None
    wells: Mapping[str, WellDefinition] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "LabwareDefinition":
        labware_id = str(data["labware_id"])
        kind = str(data.get("kind", "generic"))
        display_name = data.get("display_name")

        wells_payload = data.get("wells", {})
        wells = {
            str(well_id).upper(): WellDefinition.from_mapping(str(well_id), well_data)
            for well_id, well_data in wells_payload.items()
        }

        metadata = {
            key: value
            for key, value in data.items()
            if key not in {"labware_id", "kind", "display_name", "wells"}
        }
        return cls(
            labware_id=labware_id,
            kind=kind,
            display_name=str(display_name) if display_name is not None else None,
            wells=wells,
            metadata=metadata,
        )

    def resolve_well(self, well_id: str) -> WellDefinition:
        normalized_well = str(well_id).strip().upper()
        try:
            return self.wells[normalized_well]
        except KeyError as exc:
            raise UnknownWellError(
                f"Labware '{self.labware_id}' does not define well '{normalized_well}'."
            ) from exc
