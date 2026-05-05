from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping


@dataclass(frozen=True)
class PositionTarget:
    location_id: str
    well_id: str | None = None
    plate_name: str | None = None
    base_coordinates: Mapping[str, int] = field(default_factory=dict)
    offsets: Mapping[str, int] = field(default_factory=dict)
    source_anchor: str = ""

    @property
    def coordinates(self) -> dict[str, int]:
        return {
            axis: int(self.base_coordinates.get(axis, 0)) + int(self.offsets.get(axis, 0))
            for axis in ("x", "y", "z")
        }

    def to_payload(self) -> dict[str, Any]:
        return {
            "location_id": self.location_id,
            "well_id": self.well_id,
            "plate_name": self.plate_name,
            "base_coordinates": dict(self.base_coordinates),
            "offsets": dict(self.offsets),
            "coordinates": self.coordinates,
            "source_anchor": self.source_anchor,
        }


class PositionTable:
    """OEM PositionTable-style resolver for dry-run semantic move testing."""

    def __init__(self, targets: Iterable[PositionTarget]) -> None:
        self._targets = tuple(targets)
        self._by_key: dict[tuple[str, str | None, str | None], PositionTarget] = {}
        for target in self._targets:
            self._by_key[self._key(target.location_id, target.well_id, target.plate_name)] = target

    @staticmethod
    def _clean(value: Any) -> str | None:
        if value is None:
            return None
        text = str(value).strip()
        return text or None

    @classmethod
    def _key(cls, location_id: Any, well_id: Any = None, plate_name: Any = None) -> tuple[str, str | None, str | None]:
        location = cls._clean(location_id)
        if not location:
            raise ValueError("OEM PositionTable row requires locationID")
        return (location.upper(), None if well_id is None else str(well_id).strip().upper(), None if plate_name is None else str(plate_name).strip().lower())

    @classmethod
    def from_rows(cls, rows: Iterable[Mapping[str, Any]]) -> "PositionTable":
        targets: list[PositionTarget] = []
        for row in rows:
            location_id = row.get("locationID") or row.get("location_id") or row.get("location")
            well_id = row.get("wellID") or row.get("well_id") or row.get("well")
            plate_name = row.get("plateName") or row.get("plate_name") or row.get("plate")
            coords = {axis: int(float(row.get(axis, 0) or 0)) for axis in ("x", "y", "z")}
            offsets = {
                "x": int(float(row.get("xOffset", row.get("x_offset", 0)) or 0)),
                "y": int(float(row.get("yOffset", row.get("y_offset", 0)) or 0)),
                "z": int(float(row.get("zOffset", row.get("z_offset", 0)) or 0)),
            }
            targets.append(
                PositionTarget(
                    location_id=str(location_id).strip().upper(),
                    well_id=None if well_id is None else str(well_id).strip().upper(),
                    plate_name=None if plate_name is None else str(plate_name).strip().lower(),
                    base_coordinates=coords,
                    offsets=offsets,
                    source_anchor=str(row.get("source") or row.get("source_anchor") or "inline-position-row"),
                )
            )
        return cls(targets)

    def resolve(self, *, location_id: str, well_id: str | None = None, plate_name: str | None = None) -> PositionTarget:
        candidates = [
            self._key(location_id, well_id, plate_name),
            self._key(location_id, well_id, None),
            self._key(location_id, None, plate_name),
            self._key(location_id, None, None),
        ]
        for key in candidates:
            target = self._by_key.get(key)
            if target is not None:
                return target
        raise KeyError(f"No OEM PositionTable target for location={location_id!r} well={well_id!r} plate={plate_name!r}")

    def compile_move_to(self, location_id: str, *, well_id: str | None = None, plate_name: str | None = None) -> dict[str, Any]:
        target = self.resolve(location_id=location_id, well_id=well_id, plate_name=plate_name)
        return {
            "semantic_action": "moveTo",
            "target": target.to_payload(),
            "requires_reference_axes": ["x", "y", "z"],
            "controller_command_planned": False,
            "source_anchor": target.source_anchor,
        }
