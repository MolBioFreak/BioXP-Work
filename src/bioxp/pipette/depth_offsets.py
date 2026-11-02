"""Saved extra depth for reaching the last liquid in a well, by tip type, station and plate type.

Default is no offset: the station's calibrated low height plus the tip-length
correction. An entry adds depth (positive mm = deeper) for a matching tip type,
station and/or plate type; any field left empty matches everything. When several
entries match, the most specific one wins (plate > station > tip type), so a
plate-type value overrides a station-wide one. Saving records only; the value is
applied by the next lowering move, liquid search floor or volume check there.
Tip racks never use an offset.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

Z_STEPS_PER_MM = 2015.748
SAVED_NAME = "depth_offsets.json"
MAX_MM = 15.0
# Values found experimentally on this robot; offered as quick picks, never applied unless saved.
STANDARD_MM = [
    {"offset_mm": 0.0, "label": "None (calibrated height)", "source": "OEM station height + tip length"},
    {"offset_mm": 1.0, "label": "+1 mm", "source": "10 uL tips just reach 8-strip tube bottoms (2026-11-01)"},
    {"offset_mm": 2.0, "label": "+2 mm", "source": "10 uL tips flex into last liquid in 8-strip tubes (2026-11-01)"},
    {"offset_mm": 4.4, "label": "+4.4 mm", "source": "10 uL tips at barcode surface, RC 96-well plate (2026-11-01)"},
    {"offset_mm": 5.0, "label": "+5 mm", "source": "10 uL tips reach RC 96-well plate bottom (2026-11-01)"},
]


def _path() -> Path:
    root = os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT", "/var/lib/bioxp-oem-runtime")
    return Path(root) / SAVED_NAME


def _key(entry: dict[str, Any]) -> tuple:
    return (entry.get("tip_type"), entry.get("location"), entry.get("plate_type"))


def entries() -> list[dict[str, Any]]:
    target = _path()
    if not target.is_file():
        return []
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        # A damaged file is kept for inspection; the last good copy is used instead.
        backup = target.with_suffix(".bak")
        try:
            data = json.loads(backup.read_text(encoding="utf-8")) if backup.is_file() else {}
        except (OSError, ValueError):
            data = {}
    rows = data.get("entries") if isinstance(data, dict) else None
    return [r for r in rows or [] if isinstance(r, dict) and isinstance(r.get("offset_mm"), (int, float))]


def _norm_plate(value: Any) -> str | None:
    text = str(value).strip().lower() if value not in (None, "") else ""
    return text or None


def resolve(*, tip_type: Any, location: Any, plate_type: Any = None) -> dict[str, Any]:
    """Offset in force for this move; zero with no matching entry."""
    tip = 200 if tip_type == 201 else tip_type
    plate = _norm_plate(plate_type)
    best, best_rank = None, -1
    for entry in entries():
        fields = ((entry.get("tip_type"), tip, 1), (entry.get("location"), location, 2),
                  (entry.get("plate_type"), plate, 4))
        if any(want is not None and want != have for want, have, _ in fields):
            continue
        rank = sum(weight for want, _, weight in fields if want is not None)
        if rank > best_rank:
            best, best_rank = entry, rank
    if best is None:
        return {"offset_steps": 0, "offset_mm": 0.0, "matched": None}
    return {"offset_steps": round(float(best["offset_mm"]) * Z_STEPS_PER_MM), "offset_mm": float(best["offset_mm"]),
            "matched": {k: best.get(k) for k in ("tip_type", "location", "plate_type", "source", "saved_at")}}


def table() -> dict[str, Any]:
    return {"z_steps_per_mm": Z_STEPS_PER_MM, "max_mm": MAX_MM, "entries": entries(), "standard": STANDARD_MM,
            "rule": "most specific match wins: plate type, then station, then tip type; no match = no offset"}


def _write(rows: list[dict[str, Any]]) -> None:
    target = _path()
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_file():
        try:
            json.loads(target.read_text(encoding="utf-8"))  # only a readable file becomes the backup
            target.with_suffix(".bak").write_bytes(target.read_bytes())
        except (OSError, ValueError):
            pass
    tmp = target.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({"schema_version": "bioxp.depth_offsets.v1", "entries": rows}, indent=2))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, target)


def save(*, tip_type: int | None, location: str | None, plate_type: str | None, offset_mm: float,
         source: str | None = None) -> dict[str, Any]:
    if tip_type not in (None, 10, 50, 200):
        raise ValueError("tip_type must be 10, 50, 200 or empty (all tips)")
    if not isinstance(offset_mm, (int, float)) or not -MAX_MM <= float(offset_mm) <= MAX_MM:
        raise ValueError(f"offset must be between -{MAX_MM} and {MAX_MM} mm")
    if location is not None:
        from ..oem_compat.pathing import LOCATION_NAME_TO_ID, TIP_TRAY_LOCATIONS
        if location not in LOCATION_NAME_TO_ID:
            raise ValueError(f"unknown station {location}")
        if LOCATION_NAME_TO_ID[location] in TIP_TRAY_LOCATIONS:
            raise ValueError("tip racks never use a depth offset")
    entry = {"tip_type": tip_type, "location": location, "plate_type": _norm_plate(plate_type),
             "offset_mm": round(float(offset_mm), 3), "source": source or "Operator setting",
             "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    rows = [r for r in entries() if _key(r) != _key(entry)] + [entry]
    _write(rows)
    return table()


def remove(*, tip_type: int | None, location: str | None, plate_type: str | None) -> dict[str, Any]:
    key = (tip_type, location, _norm_plate(plate_type))
    rows = entries()
    kept = [r for r in rows if _key(r) != key]
    if len(kept) == len(rows):
        raise KeyError("no saved offset for that tip type, station and plate type")
    _write(kept)
    return table()
