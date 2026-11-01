"""pLLD settings per tip size, filtered and unfiltered.

Every liquid search applies the profile for the tip type loaded at that moment,
so a service restart (pumps back to factory values) or a tip change never leaves
a stale threshold. Unfiltered values are the Cavro factory defaults (manual p.55).
Filtered tips need a higher threshold (Tecan 399094 section 3); filtered 50 uL
200 ADC was measured on this robot, the others are provisional until measured.
Operator edits are saved in runtime state and override these defaults.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

TIP_TYPES = (10, 50, 200)
FACTORY = {"threshold_adc": 15, "threshold_duration_ms": 5, "speed_increments_per_second": 150}
DEFAULTS: dict[str, dict[str, dict[str, Any]]] = {
    str(t): {"unfiltered": {**FACTORY, "measured": False, "source": "Cavro factory default"},
             "filtered": {**FACTORY, "threshold_adc": 200, "measured": t == 50,
                          "source": "Measured on this robot 2026-11-01" if t == 50
                          else "Provisional: same as measured filtered 50 uL"}}
    for t in TIP_TYPES}
SAVED_NAME = "lld_profiles.json"
LIMITS = {"threshold_adc": (1, 1000), "threshold_duration_ms": (1, 100), "speed_increments_per_second": (100, 1000)}
SELECTORS = {"threshold_adc": 0, "threshold_duration_ms": 1, "speed_increments_per_second": 3}


def _path() -> Path:
    root = os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT", "/var/lib/bioxp-oem-runtime")
    return Path(root) / SAVED_NAME


def _saved() -> dict[str, Any]:
    target = _path()
    if not target.is_file():  # stat only; absent until an operator saves
        return {}
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def profiles() -> dict[str, Any]:
    saved = _saved()
    table = {t: {k: {**v, **((saved.get("profiles") or {}).get(t, {}).get(k) or {})} for k, v in rows.items()}
             for t, rows in DEFAULTS.items()}
    return {"filtered_tips": bool(saved.get("filtered_tips", True)), "profiles": table,
            "applied": "at the start of every liquid search, for the loaded tip type"}


def profile_for(tip_type: Any) -> dict[str, Any] | None:
    """Settings for the loaded tip type, or None when the type is unknown (e.g. 201 after restart)."""
    data = profiles()
    key = "200" if tip_type == 201 else str(tip_type)
    rows = data["profiles"].get(key)
    if rows is None:
        return None
    kind = "filtered" if data["filtered_tips"] else "unfiltered"
    return {"tip_type": int(key), "kind": kind, **rows[kind]}


def wires(profile: dict[str, Any]) -> list[str]:
    return [f"p{SELECTORS[name]},{int(profile[name])}R" for name in SELECTORS]


def save(*, filtered_tips: bool | None = None, tip_type: int | None = None, kind: str | None = None,
         values: dict[str, Any] | None = None, source: str | None = None) -> dict[str, Any]:
    """Record operator settings. Nothing is sent to the pumps until the next search."""
    data = _saved()
    if filtered_tips is not None:
        data["filtered_tips"] = bool(filtered_tips)
    if tip_type is not None:
        if tip_type not in TIP_TYPES or kind not in {"filtered", "unfiltered"}:
            raise ValueError("tip_type must be 10, 50 or 200 and kind filtered or unfiltered")
        row = {}
        for name, value in (values or {}).items():
            low, high = LIMITS[name]
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"{name} must be an integer {low}-{high}")
            row[name] = value
        row.update(measured=True, source=source or "Operator setting")
        table = data.setdefault("profiles", {}).setdefault(str(tip_type), {})
        table[kind] = {**table.get(kind, {}), **row}
    target = _path()
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(".tmp")
    tmp.write_text(json.dumps({"schema_version": "bioxp.lld_profiles.v1", **data}, indent=2))
    os.replace(tmp, target)
    return profiles()
