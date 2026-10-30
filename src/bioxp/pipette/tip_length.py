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
TIP_LENGTH_MM = {10: 35.3, 50: 58.0, 200: 58.5}
DATASHEET = "Tecan LiHa DiTi data sheet 399504 V2.0 (2020-10)"
OFFSETS_ENV = "BIOXP_TIP_Z_OFFSETS"
DEFAULT_PATH = Path(__file__).resolve().parents[3] / "config" / "oem" / "tip_z_offsets.json"


def _load(path: Path | None = None) -> dict[str, Any]:
    target = Path(os.environ.get(OFFSETS_ENV) or path or DEFAULT_PATH)
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"offsets_steps": {}, "source": None}
    return data if isinstance(data, dict) else {"offsets_steps": {}, "source": None}


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
        "source": data.get("source") if measured else "not measured; OEM behaviour (no correction)",
        "datasheet": DATASHEET,
    }
