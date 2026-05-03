from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable


class FakeSwitchAuditHardware:
    def __init__(self):
        self.move_calls: list[dict] = []

    def switch_snapshot(self, axis: str) -> dict:
        presets = {"x": (5, 0), "y": (4, 0), "z": (4, 1), "g": (4, 2), "door": (6, 0)}
        board, motor = presets.get(axis, (0, 0))
        return {
            "axis": axis,
            "board": board,
            "motor": motor,
            "position": {"value": 0},
            "speed": {"value": 0},
            "gap9_left": {"value": 0},
            "gap10_right": {"value": 1},
            "home_query": {"value": 0},
            "switch_masks": {},
            "current_params": {},
            "oem_profile": {},
        }


def run_switch_audit(hardware, *, axes: Iterable[str], mode: str = "status", artifact_root: str | Path | None = None) -> dict:
    if mode == "live_probe" and artifact_root is None:
        return {"ok": False, "mode": mode, "error": "artifact_root required for live_probe"}
    rows = []
    for axis in axes:
        snap = hardware.switch_snapshot(str(axis))
        snap.setdefault("interpreted", {"home_switch_candidate": None, "active_value_candidate": None, "confidence": "unknown"})
        if "interpreted" not in snap:
            snap["interpreted"] = {"home_switch_candidate": None, "active_value_candidate": None, "confidence": "unknown"}
        rows.append(snap)
    result = {"ok": True, "mode": mode, "axes": rows}
    if artifact_root is not None:
        root = Path(artifact_root)
        root.mkdir(parents=True, exist_ok=True)
        (root / "switch_audit.json").write_text(json.dumps(result, indent=2, sort_keys=True))
    return result
