"""One saved record per plate type: everything the robot needs to handle that plate.

Each plate (keyed by its library id, e.g. thermo-4483485-enduraplate-fast) holds its
name, catalog numbers, max volume, gripper heights by station and pipetting depth
offsets. Which plate type is loaded as each movable object is recorded here too, so
manual controls and workflows read the same values. Defaults live in code (PLATES);
saved values in plate_profiles.json override them. Saving records only; no motion.
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
SCHEMA = "bioxp.plate_profiles.v1"

# OEM catchPlate/releasePlate raise the output plate on the magnet block by 30236 steps
# (zLow 143882 -> 113646). Used for any plate without its own gripper entry there.
OEM_MAGNET_BLOCK_OUTPUT_PLATE_Z_OFFSET = -30236

PLATES: dict[str, dict[str, Any]] = {
    "thermo-4483485-enduraplate-fast": {
        "name": "MicroAmp EnduraPlate Optical 96-Well Fast Clear Reaction Plate with Barcode (4483485)",
        "catalog_numbers": ["4483485", "4483486", "4483487", "4483492", "4483495", "4483493", "4483494", "4483496"],
        "max_volume_ul": 100,
        "gripper": {
            # Low profile: the OEM height released it ~8-10 mm up and grabs closed on nothing.
            # Titrated live 2026-11-18: 113646 miss, 123646 ~1/3 miss, 128646 solid, 133646 chosen.
            "LOC_P_MS": {"z_low_offset_steps": -10236, "z_steps": 133646,
                         "source": "Operator titration on the magnet block, 2026-11-18 (grab and release)"},
        },
        "depth_offsets": [],
    },
    "thermo-4483354-enduraplate": {
        "name": "MicroAmp EnduraPlate Optical 96-Well (4483354)",
        "catalog_numbers": ["4483354"],
        "max_volume_ul": None,
        "gripper": {},
        "depth_offsets": [],
    },
}
LOADED = {"OUTPUT_PLATE": "thermo-4483485-enduraplate-fast"}


def _root() -> Path:
    return Path(os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT", "/var/lib/bioxp-oem-runtime"))


def _read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        backup = path.with_suffix(".bak")
        try:
            return json.loads(backup.read_text(encoding="utf-8")) if backup.is_file() else None
        except (OSError, ValueError):
            return None


def _saved() -> dict[str, Any]:
    target = _root() / SAVED_NAME
    data = _read_json(target) if target.is_file() or target.with_suffix(".bak").is_file() else None
    data = data if isinstance(data, dict) else {}
    data.setdefault("plates", {})
    data.setdefault("loaded", {})
    data.setdefault("depth_offsets", None)
    if data["depth_offsets"] is None:
        # One-time fold-in of the old separate depth offset file.
        legacy = _read_json(_root() / LEGACY_DEPTH_NAME) if (_root() / LEGACY_DEPTH_NAME).is_file() else None
        rows = legacy.get("entries") if isinstance(legacy, dict) else None
        data["depth_offsets"] = [r for r in rows or [] if isinstance(r, dict)]
    return data


def _write(data: dict[str, Any]) -> None:
    target = _root() / SAVED_NAME
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_file() and _read_json(target) is not None:
        target.with_suffix(".bak").write_bytes(target.read_bytes())
    tmp = target.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({"schema_version": SCHEMA, **data}, indent=2))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, target)
    legacy = _root() / LEGACY_DEPTH_NAME
    for path in (legacy, legacy.with_suffix(".bak")):
        if path.is_file():
            path.unlink()


def plates() -> dict[str, dict[str, Any]]:
    saved = _saved()["plates"]
    out = copy.deepcopy(PLATES)
    for plate_id, row in saved.items():
        base = out.setdefault(plate_id, {"name": plate_id, "catalog_numbers": [], "max_volume_ul": None,
                                         "gripper": {}, "depth_offsets": []})
        for key, value in row.items():
            if key == "gripper" and isinstance(value, dict):
                base["gripper"].update(value)
            else:
                base[key] = value
    return out


def loaded() -> dict[str, str]:
    return {**LOADED, **_saved()["loaded"]}


def table() -> dict[str, Any]:
    return {"schema_version": SCHEMA, "plates": plates(), "loaded": loaded(),
            "depth_offsets": depth_offset_entries(),
            "rule": "gripper: loaded plate's entry for the station, else OEM height; depth: most specific match"}


def _plate_id(value: Any) -> str | None:
    if value in (None, ""):
        return None
    text = str(value).strip().lower()
    for plate_id, row in plates().items():
        if text in {plate_id, str(row.get("name", "")).strip().lower(), *map(str, row.get("catalog_numbers") or [])}:
            return plate_id
    return text


def gripper_z_offset(movable_object: str, location: str) -> dict[str, Any]:
    """z_low offset for gripping this object at this station; None when the plate has no entry."""
    plate_id = loaded().get(movable_object)
    entry = (plates().get(plate_id) or {}).get("gripper", {}).get(location) if plate_id else None
    if not isinstance(entry, dict) or type(entry.get("z_low_offset_steps")) is not int:
        return {"plate_id": plate_id, "z_low_offset_steps": None}
    return {"plate_id": plate_id, "z_low_offset_steps": int(entry["z_low_offset_steps"]), "source": entry.get("source")}


def save_gripper(*, plate_id: str, location: str, z_low_offset_steps: int, source: str | None = None) -> dict[str, Any]:
    from .oem_compat.pathing import LOCATION_NAME_TO_ID
    if location not in LOCATION_NAME_TO_ID:
        raise ValueError(f"unknown station {location}")
    if type(z_low_offset_steps) is not int or not -60000 <= z_low_offset_steps <= 20000:
        raise ValueError("z_low_offset_steps must be an integer between -60000 and 20000")
    data = _saved()
    row = data["plates"].setdefault(_plate_id(plate_id), {})
    row.setdefault("gripper", {})[location] = {"z_low_offset_steps": z_low_offset_steps,
        "source": source or "Operator setting", "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    _write(data)
    return table()


def set_loaded(*, movable_object: str, plate_id: str) -> dict[str, Any]:
    data = _saved()
    data["loaded"][str(movable_object)] = _plate_id(plate_id)
    _write(data)
    return table()


def depth_offset_entries() -> list[dict[str, Any]]:
    return [r for r in _saved()["depth_offsets"] if isinstance(r.get("offset_mm"), (int, float))]


def write_depth_offsets(rows: list[dict[str, Any]]) -> None:
    data = _saved()
    data["depth_offsets"] = rows
    _write(data)


def normalize_plate(value: Any) -> str | None:
    return _plate_id(value)
