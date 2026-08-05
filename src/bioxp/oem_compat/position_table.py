from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

from ..oem_config import find_oem_machine_config_bundle
from ..oem_machine_bundle import OemMachineSnapshot, get_active_oem_machine_snapshot

OEM_X_INCREMENT = -2132
OEM_Y_INCREMENT = 2132
OEM_PSEUDO_Z_HOME_HIGH = 500
OEM_PSEUDO_Z_HOME_LOW = 65000
OEM_SCRIPT_TIP_ADJUST_EXEMPT_LOCATION_IDS = {"TECANRACK1", "TECANRACK2", "TECANRACK3", "TECANRACK4", "WASTE_BIN", "LOC_TROUGH"}


@dataclass(frozen=True)
class PositionTarget:
    location_id: str
    well_id: str | None = None
    plate_name: str | None = None
    base_coordinates: Mapping[str, int] = field(default_factory=dict)
    offsets: Mapping[str, int] = field(default_factory=dict)
    z_low: int | None = None
    z_high: int | None = None
    z_delta: int | None = None
    inc_factor: int = 0
    source_anchor: str = ""

    @property
    def coordinates(self) -> dict[str, int]:
        return {axis: int(self.base_coordinates.get(axis, 0)) + int(self.offsets.get(axis, 0)) for axis in ("x", "y", "z")}

    def oem_move_to_coordinates(self, *, column: int = 0, row: int = 0, high_pos: bool = True) -> dict[str, int]:
        z = self.z_high if bool(high_pos) else self.z_low
        if z is None:
            z = int(self.base_coordinates.get("z", 0))
        return {"x": int(self.base_coordinates.get("x", 0)) + int(self.inc_factor) * OEM_X_INCREMENT * int(column), "y": int(self.base_coordinates.get("y", 0)) + int(self.inc_factor) * OEM_Y_INCREMENT * int(row), "z": int(z)}

    def oem_offset_move_coordinates(self, *, offset_x: int = 0, offset_y: int = 0, x_high_limit: int | None = None, y_high_limit: int | None = None) -> dict[str, int]:
        x = int(self.base_coordinates.get("x", 0)) + int(offset_x)
        y = int(self.base_coordinates.get("y", 0)) + int(offset_y)
        if x_high_limit is not None and x > int(x_high_limit):
            x = int(x_high_limit) - 50
        if y_high_limit is not None and y > int(y_high_limit):
            y = int(y_high_limit) - 50
        return {"x": x, "y": y, "z": OEM_PSEUDO_Z_HOME_LOW}

    def oem_script_move_to_coordinates(self, *, column: int = 0, row: int = 0, positionflag: int = 0, tip_location: int = -1) -> dict[str, int]:
        effective_row = int(row)
        if self.location_id.upper() not in OEM_SCRIPT_TIP_ADJUST_EXEMPT_LOCATION_IDS:
            effective_row = int(row) - (int(tip_location) if int(tip_location) != -1 else 0) * 2
        z = OEM_PSEUDO_Z_HOME_LOW if int(positionflag) == 0 else (self.z_high if int(positionflag) == 1 else self.z_low)
        if z is None:
            z = OEM_PSEUDO_Z_HOME_LOW
        return {"x": int(self.base_coordinates.get("x", 0)) + int(self.inc_factor) * OEM_X_INCREMENT * int(column), "y": int(self.base_coordinates.get("y", 0)) + int(self.inc_factor) * OEM_Y_INCREMENT * effective_row, "z": int(z)}

    def to_payload(self) -> dict[str, Any]:
        return {"location_id": self.location_id, "well_id": self.well_id, "plate_name": self.plate_name, "base_coordinates": dict(self.base_coordinates), "offsets": dict(self.offsets), "coordinates": self.coordinates, "z_low": self.z_low, "z_high": self.z_high, "z_delta": self.z_delta, "inc_factor": self.inc_factor, "source_anchor": self.source_anchor}


class PositionTable:
    """OEM PositionTable resolver/planner using ClassControlInterface formulas."""

    def __init__(self, targets: Iterable[PositionTarget], *, source: str = "inline") -> None:
        self.source = source
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
    def from_rows(cls, rows: Iterable[Mapping[str, Any]], *, source: str = "inline-position-row") -> "PositionTable":
        targets: list[PositionTarget] = []
        for row in rows:
            location_id = row.get("locationID") or row.get("location_id") or row.get("location") or row.get("name")
            well_id = row.get("wellID") or row.get("well_id") or row.get("well")
            plate_name = row.get("plateName") or row.get("plate_name") or row.get("plate")
            x = int(float(row.get("x", 0) or 0)); y = int(float(row.get("y", 0) or 0))
            z_low = int(float(row.get("zLow", row.get("z_low", 0)) or 0)) if row.get("zLow", row.get("z_low")) is not None else None
            z_delta = int(float(row.get("zDelta", row.get("z_delta", 0)) or 0)) if row.get("zDelta", row.get("z_delta")) is not None else None
            z_high = row.get("zHigh", row.get("z_high"))
            if z_high is None and z_low is not None and z_delta is not None:
                z_high = z_low - z_delta
            z_high_int = int(float(z_high)) if z_high is not None else None
            inc_factor = int(float(row.get("inc_factor", row.get("incFactor", 0)) or 0))
            coords = {
                "x": x,
                "y": y,
                # Preserved ClassBioXPSettings PositionTable parsing does not
                # recognize a plain `z` field. Unknown aliases cannot silently
                # become an OEM motion target.
                "z": int(z_high_int if z_high_int is not None else (z_low if z_low is not None else 0)),
            }
            offsets = {"x": int(float(row.get("xOffset", row.get("x_offset", 0)) or 0)), "y": int(float(row.get("yOffset", row.get("y_offset", 0)) or 0)), "z": int(float(row.get("zOffset", row.get("z_offset", 0)) or 0))}
            targets.append(PositionTarget(location_id=str(location_id).strip().upper(), well_id=None if well_id is None else str(well_id).strip().upper(), plate_name=None if plate_name is None else str(plate_name).strip().lower(), base_coordinates=coords, offsets=offsets, z_low=z_low, z_high=z_high_int, z_delta=z_delta, inc_factor=inc_factor, source_anchor=str(row.get("source") or row.get("source_anchor") or source)))
        return cls(targets, source=source)

    @classmethod
    def from_bound_machine_config(cls, root_dir: str | OemMachineSnapshot | None = None) -> "PositionTable":
        if isinstance(root_dir, OemMachineSnapshot):
            return cls.from_rows(root_dir.position_table, source="immutable_oem_machine_snapshot.PositionTable")
        if root_dir is None:
            snapshot = get_active_oem_machine_snapshot()
            return cls.from_rows(snapshot.position_table, source="immutable_oem_machine_snapshot.PositionTable")
        bundle = find_oem_machine_config_bundle(root_dir)
        if not isinstance(bundle, dict) or bundle.get("ok") is not True:
            raise RuntimeError(f"OEM machine config not bound: {bundle}")
        rows = (((bundle.get("config") or {}).get("position_table")) or [])
        return cls.from_rows(rows, source="non_authoritative_diagnostic_machine_config.PositionTable")

    def rows(self) -> list[dict[str, Any]]:
        return [target.to_payload() for target in self._targets]

    def resolve(self, *, location_id: str, well_id: str | None = None, plate_name: str | None = None) -> PositionTarget:
        candidates = [self._key(location_id, well_id, plate_name), self._key(location_id, well_id, None), self._key(location_id, None, plate_name), self._key(location_id, None, None)]
        for key in candidates:
            target = self._by_key.get(key)
            if target is not None:
                return target
        raise KeyError(f"No OEM PositionTable target for location={location_id!r} well={well_id!r} plate={plate_name!r}")

    def compile_move_to(self, location_id: str, *, well_id: str | None = None, plate_name: str | None = None, column: int = 0, row: int = 0, high_pos: bool = True) -> dict[str, Any]:
        target = self.resolve(location_id=location_id, well_id=well_id, plate_name=plate_name)
        return {"semantic_action": "moveTo", "source_formula": "ClassControlInterface.cs:3663-3688", "source_table": self.source, "target": target.to_payload(), "column": int(column), "row": int(row), "high_pos": bool(high_pos), "planned_coordinates": target.oem_move_to_coordinates(column=column, row=row, high_pos=high_pos), "requires_reference_axes": ["x", "y", "z"], "controller_command_planned": False, "opened_usb": False, "physical_motion": False, "source_anchor": target.source_anchor}

    def compile_offset_move_to(self, location_id: str, *, offset_x: int = 0, offset_y: int = 0, x_high_limit: int | None = None, y_high_limit: int | None = None) -> dict[str, Any]:
        target = self.resolve(location_id=location_id)
        return {"semantic_action": "moveTo_offset", "source_formula": "ClassControlInterface.cs:3691-3715", "source_table": self.source, "target": target.to_payload(), "offset_x": int(offset_x), "offset_y": int(offset_y), "planned_coordinates": target.oem_offset_move_coordinates(offset_x=offset_x, offset_y=offset_y, x_high_limit=x_high_limit, y_high_limit=y_high_limit), "controller_command_planned": False, "opened_usb": False, "physical_motion": False}

    def compile_script_move_to(self, location_id: str, *, column: int = 0, row: int = 0, positionflag: int = 0, tip_location: int = -1) -> dict[str, Any]:
        target = self.resolve(location_id=location_id)
        return {"semantic_action": "scriptmoveTo", "source_formula": "ClassControlInterface.cs:3734-3860 initial target-coordinate branch", "source_table": self.source, "target": target.to_payload(), "column": int(column), "row": int(row), "positionflag": int(positionflag), "tip_location": int(tip_location), "planned_coordinates": target.oem_script_move_to_coordinates(column=column, row=row, positionflag=positionflag, tip_location=tip_location), "controller_command_planned": False, "opened_usb": False, "physical_motion": False, "remaining_live_dependencies": ["currentLoc/currentWell pathing/midpoint branch", "MachineStatus.TipLoaded/TipLocation", "confirmAxis(gripper)"]}


def load_bound_oem_position_table(root_dir: str | OemMachineSnapshot | None = None) -> PositionTable:
    return PositionTable.from_bound_machine_config(root_dir)
