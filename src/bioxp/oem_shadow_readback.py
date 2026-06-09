
"""Shadow/readback truth capture for OEM parity without commanded motion."""
from __future__ import annotations

from typing import Any, Protocol

from .oem_parity_predicates import classify_home_switch


class OemShadowReadbackProvider(Protocol):
    def axis_speed(self, axis: str) -> int | float | None: ...
    def axis_current(self, axis: str) -> dict[str, Any]: ...
    def home_switch(self, axis: str) -> int | bool | None: ...
    def interlocks(self) -> dict[str, Any]: ...


def classify_g_current(speed: Any, current: dict[str, Any] | None) -> dict[str, Any]:
    current = current or {}
    param6 = current.get("param6")
    param7 = current.get("param7")
    if speed == 0 and param6 == 10 and param7 == 10:
        cls = "G_CURRENT_IDLE_SAFE"
        ok = True
    elif speed == 0 and (param6 not in (None, 10) or param7 not in (None, 10)):
        cls = "G_CURRENT_UNSAFE_HOT_IDLE"
        ok = False
    else:
        cls = "G_CURRENT_NOT_IDLE_OR_UNKNOWN"
        ok = False
    return {"ok": ok, "classification": cls, "speed": speed, "param6": param6, "param7": param7}


def run_shadow_readback(provider: OemShadowReadbackProvider, *, axes: list[str] | None = None) -> dict[str, Any]:
    axes = axes or ["x", "y", "z", "g", "door"]
    axis_rows: dict[str, Any] = {}
    for axis in axes:
        speed = provider.axis_speed(axis)
        current = provider.axis_current(axis)
        raw_home = provider.home_switch(axis)
        axis_rows[axis] = {
            "speed": speed,
            "current": current,
            "home_switch": classify_home_switch(axis, raw_home),
        }
    g_row = axis_rows.get("g")
    g_inv = classify_g_current(g_row.get("speed") if g_row else None, g_row.get("current") if g_row else None)
    return {
        "ok": g_inv["ok"] if "g" in axes else True,
        "mode": "shadow_readback",
        "opened_usb": "provider_owned",
        "physical_motion": False,
        "motion_commanded": False,
        "axes": axis_rows,
        "interlocks": provider.interlocks(),
        "g_current_invariant": g_inv,
    }
