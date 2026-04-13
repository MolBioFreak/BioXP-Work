from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


class DomainError(ValueError):
    """Base error for typed BioXP domain failures."""


class UnknownLocationError(DomainError, KeyError):
    """Raised when a semantic location or slot cannot be resolved."""


class UnknownWellError(DomainError, KeyError):
    """Raised when a requested well is not available on the resolved labware."""


@dataclass(frozen=True)
class Coordinate3D:
    """Simple deck-space coordinate in millimeters."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "Coordinate3D":
        if data is None:
            return cls()
        return cls(
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            z=float(data.get("z", 0.0)),
        )

    def __add__(self, other: "Coordinate3D") -> "Coordinate3D":
        return Coordinate3D(
            x=self.x + other.x,
            y=self.y + other.y,
            z=self.z + other.z,
        )

    def as_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


ZERO_COORDINATE = Coordinate3D()


@dataclass(frozen=True)
class LocationReference:
    """Parsed semantic location reference such as ``reagent_rack:B2``."""

    location_id: str
    well_id: str | None = None

    @classmethod
    def parse(cls, value: str | "LocationReference") -> "LocationReference":
        if isinstance(value, cls):
            return value

        raw_value = str(value).strip()
        if not raw_value:
            raise ValueError("location reference must not be empty")

        location_id, separator, well_id = raw_value.partition(":")
        location_id = location_id.strip()
        if not location_id:
            raise ValueError("location reference must include a location identifier")

        parsed_well = well_id.strip().upper() or None if separator else None
        return cls(location_id=location_id, well_id=parsed_well)

    @property
    def qualified_id(self) -> str:
        if self.well_id is None:
            return self.location_id
        return f"{self.location_id}:{self.well_id}"


@dataclass(frozen=True)
class ResolvedLocation:
    """Fully resolved semantic location in deck space."""

    location_id: str
    slot_id: str
    coordinate: Coordinate3D
    labware_id: str | None = None
    well_id: str | None = None
    aliases: tuple[str, ...] = field(default_factory=tuple)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @property
    def address(self) -> str:
        if self.well_id is None:
            return self.location_id
        return f"{self.location_id}:{self.well_id}"
