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


IMPLEMENTATION_MAPPED_HOME_PREDICATES = {
    # Linux TMCL binding: motor_query_home_switch reads GAP9/left switch and
    # motor_get_switch_activity treats raw 1 as active. Keep this below
    # source_anchored/live_verified because the decompiled dump available here
    # references queryHome(axis) but does not expose the board-level queryHome
    # implementation/polarity directly.
    "x": OemHomePredicate(axis="x", home_switch="gap9_left", active_value=1, search_direction="move_left", confidence="implementation_mapped", source="linux motor_query_home_switch GAP9 + MOTOR_SWITCH_ACTIVE_VALUE=1; OEM calls queryHome(axis)"),
    "y": OemHomePredicate(axis="y", home_switch="gap9_left", active_value=1, search_direction="move_left", confidence="implementation_mapped", source="linux motor_query_home_switch GAP9 + MOTOR_SWITCH_ACTIVE_VALUE=1; OEM calls queryHome(axis)"),
    "z": OemHomePredicate(axis="z", home_switch="gap9_left", active_value=1, search_direction="unverified_z_direction", confidence="implementation_mapped", source="linux motor_query_home_switch GAP9 + MOTOR_SWITCH_ACTIVE_VALUE=1; OEM calls MotorZ.queryHome/axisSearchHome; live direction/predicate not yet verified"),
    "g": OemHomePredicate(axis="g", home_switch="gap9_left", active_value=1, search_direction="move_left", confidence="implementation_mapped", source="linux motor_query_home_switch GAP9 + MOTOR_SWITCH_ACTIVE_VALUE=1; OEM calls MotorGrip.queryHome/axisSearchHome"),
    "door": OemHomePredicate(axis="door", home_switch="gap9_left", active_value=1, search_direction="move_left", confidence="implementation_mapped", source="linux motor_query_home_switch GAP9 + MOTOR_SWITCH_ACTIVE_VALUE=1; OEM calls ThermalDoor.queryHome/doorSearchHome"),
}


def _value(row: dict | None) -> int | None:
    if not isinstance(row, dict):
        return None
    val = row.get("value")
    try:
        return int(val)
    except Exception:
        return None


def interpret_home_predicate(axis: str, snap: dict) -> dict:
    pred = IMPLEMENTATION_MAPPED_HOME_PREDICATES.get(axis)
    if pred is None:
        return unknown_predicate(axis)
    payload = pred.to_payload()
    home_val = _value(snap.get("home_query"))
    gap9_val = _value(snap.get("gap9_left"))
    gap10_val = _value(snap.get("gap10_right"))
    payload.update({
        "home_value": home_val,
        "gap9_left_value": gap9_val,
        "gap10_right_value": gap10_val,
        "is_home_now": None if home_val is None else home_val == pred.active_value,
        "homing_enable_state": "blocked_until_live_verified",
        "blockers": [
            "board_level_queryHome_source_not_exposed_in_decompiled_dump",
            "physical_observation_required_for_first_live_step",
        ],
    })
    if axis == "z":
        payload["blockers"].extend([
            "z_direction_reversal_is_quarantined_until_live_verified",
            "z_gap9_home_value_currently_0_not_active_under_active_value_1",
        ])
    return payload


def run_switch_audit(hardware, *, axes: Iterable[str], mode: str = "status", artifact_root: str | Path | None = None) -> dict:
    axis_list = [str(a) for a in axes]
    invalid = [a for a in axis_list if a not in VALID_AXES]
    if invalid:
        return {"ok": False, "mode": mode, "error": f"unknown axes: {', '.join(invalid)}"}
    if mode == "live_probe" and artifact_root is None:
        return {"ok": False, "mode": mode, "error": "artifact_root required for live_probe"}
    rows = []
    per_axis_blockers = []
    for axis in axis_list:
        snap = hardware.switch_snapshot(str(axis))
        interpreted = interpret_home_predicate(axis, snap)
        snap.setdefault("interpreted", interpreted)
        if interpreted.get("confidence") not in {"source_anchored", "live_verified", "high"}:
            per_axis_blockers.append({"axis": axis, "reason": "home predicate not live/source verified", "predicate": interpreted})
        rows.append(snap)
    result = {
        "ok": True,
        "mode": mode,
        "axes": rows,
        "homing_allowed": False,
        "reason": "home predicate confidence remains below live/source-verified threshold",
        "predicate_blockers": per_axis_blockers,
    }
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
