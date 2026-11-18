"""One saved record per plate type: the robot's handling values for that plate.

Each plate (keyed by its plate library id, e.g. thermo-4483485-enduraplate-fast) holds
gripper heights by plate station and pipetting depth offsets by tip type and station.
Depth offsets that apply to any plate at a station are station defaults. The file also
records which plate type is loaded as each movable object and which plate sits at each
pipetting station, so manual controls and workflows read the same values. Code
defaults (PLATES) are overridden by saved values in plate_profiles.json. Geometry and
volumes live in the BMS plate library under the same id. Saving records only; no motion.
"""
from __future__ import annotations

import copy
import json
import os
import time
from pathlib import Path
from typing import Any

SAVED_NAME = "plate_profiles.json"
LEGACY_DEPTH_NAME = "depth_offsets.json"
SCHEMA = "bioxp.plate_profiles.v2"

# OEM catchPlate/releasePlate raise the output plate on the magnet block by 30236 steps
# (zLow 143882 -> 113646). Used for any plate without its own gripper entry there.
OEM_MAGNET_BLOCK_OUTPUT_PLATE_Z_OFFSET = -30236
# Plate (gripper) stations and the pipetting station each one serves.
GRIP_TO_STATION = {"LOC_P_MS": "LOC_MS", "LOC_P_TC": "LOC_TC", "LOC_P_OC": "LOC_OC", "LOC_P_RC": "LOC_RC"}

PLATES: dict[str, dict[str, Any]] = {
    "thermo-4483485-enduraplate-fast": {
        "name": "MicroAmp EnduraPlate Optical 96-Well Fast Clear Reaction Plate with Barcode (4483485)",
        "gripper": {
            # Low profile: the OEM height released it ~8-10 mm up and grabs closed on nothing.
            # Titrated live 2026-11-18: 113646 miss, 123646 ~1/3 miss, 128646 solid, 133646 chosen.
            "LOC_P_MS": {"z_low_offset_steps": -10236, "z_steps": 133646,
                         "source": "Operator titration on the magnet block, 2026-11-18 (grab and release)"},
        },
        "depth_offsets": [],
    },
}
LOADED = {"OUTPUT_PLATE": "thermo-4483485-enduraplate-fast"}


def _root() -> Path:
    return Path(os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT", "/var/lib/bioxp-oem-runtime"))


def _read_json(path: Path) -> Any:
    for candidate in (path, path.with_suffix(".bak")):
        try:
            if candidate.is_file():
                return json.loads(candidate.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
    return None


def _norm(value: Any) -> str | None:
    text = str(value).strip().lower() if value not in (None, "") else ""
    return text or None


def _place_depth(data: dict[str, Any], row: dict[str, Any]) -> None:
    plate = _norm(row.get("plate_type"))
    entry = {k: row.get(k) for k in ("tip_type", "location", "offset_mm", "source", "saved_at")}
    if plate is None:
        data["station_depth_offsets"].append(entry)
    else:
        data["plates"].setdefault(plate, {}).setdefault("depth_offsets", []).append(entry)


def _saved() -> dict[str, Any]:
    data = _read_json(_root() / SAVED_NAME)
    data = data if isinstance(data, dict) else {}
    for key in ("plates", "loaded", "stations", "object_stations"):
        data.setdefault(key, {})
    if "station_depth_offsets" not in data:
        # Fold in depth offsets saved before they lived under each plate.
        rows = data.pop("depth_offsets", None)
        if rows is None:
            legacy = _read_json(_root() / LEGACY_DEPTH_NAME)
            rows = legacy.get("entries") if isinstance(legacy, dict) else []
        data["station_depth_offsets"] = []
        for row in rows or []:
            if isinstance(row, dict):
                _place_depth(data, row)
    return data


def _write(data: dict[str, Any]) -> None:
    target = _root() / SAVED_NAME
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_file() and _read_json(target) is not None:
        target.with_suffix(".bak").write_bytes(target.read_bytes())
    tmp = target.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({**data, "schema_version": SCHEMA}, indent=2))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, target)
    legacy = _root() / LEGACY_DEPTH_NAME
    for path in (legacy, legacy.with_suffix(".bak")):
        if path.is_file():
            path.unlink()


def plates() -> dict[str, dict[str, Any]]:
    out = copy.deepcopy(PLATES)
    for plate_id, row in _saved()["plates"].items():
        base = out.setdefault(plate_id, {"name": plate_id, "gripper": {}, "depth_offsets": []})
        for location, entry in (row.get("gripper") or {}).items():
            if entry is None:
                base["gripper"].pop(location, None)
            else:
                base["gripper"][location] = entry
        if "depth_offsets" in row:
            base["depth_offsets"] = list(row["depth_offsets"])
        if row.get("name"):
            base["name"] = row["name"]
    return out


def loaded() -> dict[str, str]:
    return {**LOADED, **_saved()["loaded"]}


def stations() -> dict[str, str]:
    return dict(_saved()["stations"])


def table() -> dict[str, Any]:
    return {"schema_version": SCHEMA, "plates": plates(), "loaded": loaded(), "stations": stations(),
            "station_depth_offsets": _saved()["station_depth_offsets"],
            "gripper_stations": sorted(GRIP_TO_STATION),
            "oem_magnet_block_output_plate_z_offset": OEM_MAGNET_BLOCK_OUTPUT_PLATE_Z_OFFSET,
            "rule": ("gripper: the loaded plate's entry for the station, else the OEM height; "
                     "depth: the entry of the plate at that station for the loaded tip, else the station default, else none")}


def _plate_id(value: Any) -> str | None:
    text = _norm(value)
    if text is None:
        return None
    for plate_id, row in plates().items():
        if text in {plate_id, _norm(row.get("name"))} or (text.isdigit() and f"({text})" in str(row.get("name", ""))):
            return plate_id
    return text


normalize_plate = _plate_id


def gripper_z_offset(movable_object: str, location: str) -> dict[str, Any]:
    """z_low offset for gripping this object at this station; None when the plate has no entry."""
    plate_id = loaded().get(movable_object)
    entry = (plates().get(plate_id) or {}).get("gripper", {}).get(location) if plate_id else None
    if not isinstance(entry, dict) or type(entry.get("z_low_offset_steps")) is not int:
        return {"plate_id": plate_id, "z_low_offset_steps": None}
    return {"plate_id": plate_id, "z_low_offset_steps": int(entry["z_low_offset_steps"]), "source": entry.get("source")}


def save_gripper(*, plate_id: str, location: str, z_low_offset_steps: int | None, source: str | None = None) -> dict[str, Any]:
    """Save a plate's gripper height at a plate station; None removes it (OEM height)."""
    if location not in GRIP_TO_STATION:
        raise ValueError(f"gripper station must be one of {sorted(GRIP_TO_STATION)}")
    data = _saved()
    grip = data["plates"].setdefault(_plate_id(plate_id), {}).setdefault("gripper", {})
    if z_low_offset_steps is None:
        grip[location] = None
    else:
        if type(z_low_offset_steps) is not int or not -60000 <= z_low_offset_steps <= 20000:
            raise ValueError("z_low_offset_steps must be an integer between -60000 and 20000")
        grip[location] = {"z_low_offset_steps": z_low_offset_steps, "source": source or "Operator setting",
                          "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    _write(data)
    return table()


def set_loaded(*, movable_object: str, plate_id: str) -> dict[str, Any]:
    data = _saved()
    data["loaded"][str(movable_object)] = _plate_id(plate_id)
    _write(data)
    return table()


def set_station(*, location: str, plate_id: str | None) -> dict[str, Any]:
    """Record which plate type sits at a pipetting station; empty clears it."""
    from .oem_compat.pathing import LOCATION_NAME_TO_ID
    if location not in LOCATION_NAME_TO_ID:
        raise ValueError(f"unknown station {location}")
    data = _saved()
    if plate_id in (None, ""):
        data["stations"].pop(location, None)
    else:
        data["stations"][location] = _plate_id(plate_id)
    _write(data)
    return table()


def record_plate_move(*, movable_object: str, plate_id: str, destination: str) -> None:
    """A workflow moved this plate type: it is the loaded object and now sits at that station."""
    data = _saved()
    plate = _plate_id(plate_id)
    data["loaded"][movable_object] = plate
    previous = data["object_stations"].get(movable_object)
    if previous and data["stations"].get(previous) == plate:
        data["stations"].pop(previous, None)
    station = GRIP_TO_STATION.get(destination, destination)
    data["object_stations"][movable_object] = station
    data["stations"][station] = plate
    _write(data)


def plate_at(location: Any) -> str | None:
    return stations().get(str(location)) if location not in (None, "") else None


def depth_offset_entries() -> list[dict[str, Any]]:
    """Flat view for matching: station defaults (plate_type None), then each plate's entries."""
    data = _saved()
    rows = [{**r, "plate_type": None} for r in data["station_depth_offsets"]]
    for plate_id, row in plates().items():
        rows += [{**r, "plate_type": plate_id} for r in row.get("depth_offsets") or []]
    return [r for r in rows if isinstance(r.get("offset_mm"), (int, float))]


def write_depth_offsets(rows: list[dict[str, Any]]) -> None:
    data = _saved()
    data["station_depth_offsets"] = []
    for plate_id in set(data["plates"]) | set(PLATES):
        data["plates"].setdefault(plate_id, {})["depth_offsets"] = []
    for row in rows:
        _place_depth(data, row)
    _write(data)
