from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

VALID_AXES = {"x", "y", "z", "g", "door"}


@dataclass(frozen=True)
class OemHomePredicate:
    axis: str
    home_switch: str
    active_value: int
    search_direction: str
    confidence: str = "unknown"
    source: str = "unproven"

    def is_confident(self) -> bool:
        return self.confidence in {"source_anchored", "live_verified", "high"}

    def to_payload(self) -> dict:
        return {
            "axis": self.axis,
            "home_switch": self.home_switch,
            "active_value": self.active_value,
            "search_direction": self.search_direction,
            "confidence": self.confidence,
            "source": self.source,
        }


class FakeSwitchAuditHardware:
    def __init__(self):
        self.move_calls: list[dict] = []
        self.query_calls: list[str] = []

    def switch_snapshot(self, axis: str) -> dict:
        self.query_calls.append(axis)
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


def unknown_predicate(axis: str) -> dict:
    return OemHomePredicate(axis=axis, home_switch="unknown", active_value=-1, search_direction="unknown").to_payload()


def run_switch_audit(hardware, *, axes: Iterable[str], mode: str = "status", artifact_root: str | Path | None = None) -> dict:
    axis_list = [str(a) for a in axes]
    invalid = [a for a in axis_list if a not in VALID_AXES]
    if invalid:
        return {"ok": False, "mode": mode, "error": f"unknown axes: {', '.join(invalid)}"}
    if mode == "live_probe" and artifact_root is None:
        return {"ok": False, "mode": mode, "error": "artifact_root required for live_probe"}
    rows = []
    for axis in axis_list:
        snap = hardware.switch_snapshot(str(axis))
        snap.setdefault("interpreted", unknown_predicate(axis))
        rows.append(snap)
    result = {"ok": True, "mode": mode, "axes": rows, "homing_allowed": False, "reason": "home predicate confidence is unknown until source/live switch matrix is proven"}
    if artifact_root is not None:
        root = Path(artifact_root)
        root.mkdir(parents=True, exist_ok=True)
        (root / "switch_audit.json").write_text(json.dumps(result, indent=2, sort_keys=True))
    return result


def require_confident_predicates(predicates: dict[str, dict], axes: Iterable[str]) -> dict:
    blockers = []
    for axis in axes:
        pred = predicates.get(axis) or unknown_predicate(axis)
        if pred.get("confidence") not in {"source_anchored", "live_verified", "high"}:
            blockers.append({"axis": axis, "reason": "home predicate not proven", "predicate": pred})
    return {"ok": not blockers, "blockers": blockers}
