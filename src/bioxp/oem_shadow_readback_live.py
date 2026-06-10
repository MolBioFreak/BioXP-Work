
"""Query-only OEM shadow/readback artifact builder.

The builder is provider-injected so tests and future live routes can guarantee
that artifact construction itself never commands motion, changes current, or
changes switch masks. A live provider may call OEM query contracts only.
"""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Iterable

from .oem_command_contracts import get_command_contract


READBACK_SOURCE_CONTRACTS = {
    "queryActualPosition": get_command_contract("ClassMotor.queryActualPosition"),
    "queryMotorSpeed": get_command_contract("ClassMotor.queryMotorSpeed"),
    "queryLeftSwitchStatus": get_command_contract("ClassMotor.queryLeftSwitchStatus"),
    "queryRightSwitchStatus": get_command_contract("ClassMotor.queryRightSwitchStatus"),
}


def _effective(raw: Any, disabled: Any) -> bool:
    return bool(raw) and not bool(disabled)


def _g_current_invariant(g: dict[str, Any] | None) -> dict[str, Any]:
    g = g or {}
    speed = g.get("speed")
    run = g.get("run_current")
    standby = g.get("standby_current")
    safe = speed == 0 and run == 10 and standby == 10
    unsafe_hot_idle = speed == 0 and ((run is not None and run > 10) or (standby is not None and standby > 10))
    if safe:
        cls = "G_CURRENT_IDLE_SAFE"
    elif unsafe_hot_idle:
        cls = "G_CURRENT_UNSAFE_HOT_IDLE"
    else:
        cls = "G_CURRENT_UNKNOWN_OR_MOVING"
    return {"classification": cls, "speed": speed, "param6_run_current": run, "param7_standby_current": standby}


def build_shadow_readback_artifact(provider: Any, *, axes: Iterable[str] = ("x", "y", "z", "g", "door")) -> dict[str, Any]:
    axis_rows: dict[str, dict[str, Any]] = {}
    for axis in axes:
        row = dict(provider.axis_snapshot(axis))
        row["left_active_effective"] = _effective(row.get("gap9_left_raw"), row.get("left_disabled"))
        row["right_active_effective"] = _effective(row.get("gap10_right_raw"), row.get("right_disabled"))
        axis_rows[axis] = row
    invariant = _g_current_invariant(axis_rows.get("g"))
    ok = invariant["classification"] != "G_CURRENT_UNSAFE_HOT_IDLE"
    artifact = {
        "ok": ok,
        "failed_closed": not ok,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "motion_commanded": False,
        "current_mutation_commanded": False,
        "switch_mask_mutation_commanded": False,
        "axes": axis_rows,
        "g_current_invariant": invariant,
        "interlocks": dict(provider.interlocks()),
        "reference_state": dict(provider.reference_state()),
        "source_contracts": {name: {"source_file": c.source_file, "source_lines": c.source_lines, "command_template": c.command_template} for name, c in READBACK_SOURCE_CONTRACTS.items()},
    }
    if not ok:
        artifact["blockers"] = ["g_current_not_safe_at_idle"]
    return artifact
