"""Per-tip Z correction so every tip type ends at the same height as a 50 uL tip.

The OEM software applies no tip-length correction: loadTip only records the
type. All Tecan LiHa DiTis share the same 7.9 mm collar, so they seat on the
gun the same way and differ only in length below the collar.

Reference lengths come from Tecan data sheet 399504 V2.0 (2020-10). Z steps
increase downward (OEM overpress adds steps), so a shorter tip needs a
positive offset to reach the same point.

Offsets are applied only from measured step values (config/oem/tip_z_offsets.json).
Until a tip type is measured its offset is 0, which is the existing OEM
behaviour; a shorter unmeasured tip simply stops higher, never lower.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

REFERENCE_TIP_TYPE = 50
TIP_LENGTH_MM = {10: 35.3, 50: 58.0, 200: 58.5, 201: 58.5}
Z_STEPS_PER_MM = 2015.748  # OEM ControlLib.cs:7848 (51200 / 25.4)
DATASHEET = "Tecan LiHa DiTi data sheet 399504 V2.0 (2020-10)"
OFFSETS_ENV = "BIOXP_TIP_Z_OFFSETS"
DEFAULT_PATH = Path(__file__).resolve().parents[3] / "config" / "oem" / "tip_z_offsets.json"
# Operator-confirmed values live in writable runtime state and override the release defaults.
CONFIRMED_ENV = "BIOXP_TIP_Z_OFFSETS_CONFIRMED"
CONFIRMED_NAME = "tip_z_offsets_confirmed.json"


def _read(target: Path) -> dict[str, Any]:
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def _confirmed_path() -> Path:
    # Same writable root as the rest of the OEM runtime state (service sets it; tests isolate it).
    root = os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT", "/var/lib/bioxp-oem-runtime")
    return Path(os.environ.get(CONFIRMED_ENV) or Path(root) / CONFIRMED_NAME)


def _load(path: Path | None = None) -> dict[str, Any]:
    data = _read(Path(os.environ.get(OFFSETS_ENV) or path or DEFAULT_PATH)) or {"offsets_steps": {}, "source": None}
    confirmed_file = _confirmed_path() if path is None else None
    if confirmed_file is not None and confirmed_file.is_file():   # stat only; absent until an operator saves
        confirmed = _read(confirmed_file).get("offsets") or {}
        if confirmed:
            data = {**data, "offsets_steps": {**(data.get("offsets_steps") or {}),
                    **{k: v["offset_steps"] for k, v in confirmed.items() if type(v.get("offset_steps")) is int}},
                    "confirmed": confirmed}
    return data


def tip_z_offset(tip_type: int | None, *, path: Path | None = None) -> dict[str, Any]:
    """Signed Z steps to add to a lowering target for this loaded tip type."""
    data = _load(path)
    raw = (data.get("offsets_steps") or {}).get(str(tip_type))
    measured = type(raw) is int
    length = TIP_LENGTH_MM.get(int(tip_type)) if isinstance(tip_type, int) else None
    return {
        "tip_type": tip_type,
        "offset_steps": raw if measured else 0,
        "measured": measured,
        "reference_tip_type": REFERENCE_TIP_TYPE,
        "datasheet_length_mm": length,
        "datasheet_delta_mm": None if length is None else round(TIP_LENGTH_MM[REFERENCE_TIP_TYPE] - length, 3),
        "datasheet_delta_steps": None if length is None else round((TIP_LENGTH_MM[REFERENCE_TIP_TYPE] - length) * Z_STEPS_PER_MM),
        "source": ((data.get("confirmed") or {}).get(str(tip_type), {}).get("source") or data.get("source"))
                  if measured else "not measured; OEM behaviour (no correction)",
        "confirmed": (data.get("confirmed") or {}).get(str(tip_type)),
        "datasheet": DATASHEET,
    }


def offsets_table() -> dict[str, Any]:
    """Every tip type's offset in force, datasheet value and any operator confirmation."""
    return {"z_steps_per_mm": Z_STEPS_PER_MM, "reference_tip_type": REFERENCE_TIP_TYPE,
            "tips": [tip_z_offset(t) for t in sorted(TIP_LENGTH_MM)]}


def save_confirmed(tip_type: int, offset_steps: int, source: str, *, at: str) -> dict[str, Any]:
    """Record an operator-confirmed offset. A record only: nothing moves."""
    if tip_type not in TIP_LENGTH_MM:
        raise ValueError(f"unknown tip type {tip_type}")
    if abs(int(offset_steps)) > 80000:
        raise ValueError("offset is more than 40 mm; check the value")
    target = _confirmed_path()
    data = _read(target)
    offsets = dict(data.get("offsets") or {})
    offsets[str(tip_type)] = {"offset_steps": int(offset_steps), "source": source, "confirmed_at": at}
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(".tmp")
    tmp.write_text(json.dumps({"schema_version": "bioxp.tip_z_offsets_confirmed.v1", "offsets": offsets}, indent=2))
    os.replace(tmp, target)
    return offsets_table()

