
"""Switch predicate matrix for fresh OEM parity scaffolding.

This is declarative and fail-closed; it performs no hardware I/O.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SwitchPredicate:
    axis: str
    home_query: str
    home_active_value: int
    left_query: str | None = None
    right_query: str | None = None
    left_switch_can_be_disabled: bool = False
    right_switch_can_be_disabled: bool = False
    requires_current_restore: bool = False
    source_anchor: str = "ClassControlInterface / ClassMotion primitives"

    def to_dict(self) -> dict[str, Any]:
        return self.__dict__.copy()


# `home_active_value` is the OEM helper return-code layer: ClassMotor
# queryLeftSwitchStatus/queryRightSwitchStatus return 0 when raw GAP9/GAP10 == 1.
# Linux raw GAP reads must use `raw_gap_active_value=1` (see usb_driver.MOTOR_SWITCH_ACTIVE_VALUE).
SWITCH_PREDICATES: dict[str, SwitchPredicate] = {
    "x": SwitchPredicate(axis="x", home_query="queryHome", home_active_value=0, left_query="queryLeftSwitchStatus", right_query="queryRightSwitchStatus"),
    "y": SwitchPredicate(axis="y", home_query="queryHome", home_active_value=0, left_query="queryLeftSwitchStatus", right_query="queryRightSwitchStatus", right_switch_can_be_disabled=True),
    "z": SwitchPredicate(axis="z", home_query="queryHome", home_active_value=0, left_query="queryLeftSwitchStatus", right_query="queryRightSwitchStatus"),
    "g": SwitchPredicate(axis="g", home_query="queryHome", home_active_value=0, left_query="queryLeftSwitchStatus", right_query="queryRightSwitchStatus", requires_current_restore=True),
    "door": SwitchPredicate(axis="door", home_query="tcDoorClosed", home_active_value=0, left_query="queryLeftSwitchStatus", right_query="queryRightSwitchStatus", left_switch_can_be_disabled=True, right_switch_can_be_disabled=True),
}


def predicate_matrix() -> dict[str, SwitchPredicate]:
    return dict(SWITCH_PREDICATES)


def classify_home_switch(axis: str, raw_value: int | bool | None) -> dict[str, Any]:
    pred = SWITCH_PREDICATES.get(axis.lower())
    if pred is None:
        return {"axis": axis, "home": False, "classification": "UNKNOWN_AXIS", "raw_value": raw_value}
    if raw_value is None:
        return {"axis": axis, "home": False, "classification": "UNKNOWN_SWITCH_TRUTH", "raw_value": raw_value, "predicate": pred.to_dict()}
    raw_int = int(raw_value)
    home = raw_int == pred.home_active_value
    return {
        "axis": axis.lower(),
        "home": home,
        "classification": "HOME_ACTIVE" if home else "HOME_INACTIVE",
        "raw_value": raw_int,
        "predicate": pred.to_dict(),
    }
