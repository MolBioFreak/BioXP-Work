from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping


class AxisDiagnosticContractError(ValueError):
    """The requested diagnostic is outside the finite operator contract."""


AXIS_DIAGNOSTIC_CATALOG: Mapping[str, tuple[str, ...]] = MappingProxyType(
    {
        "x": ("status", "move-negative", "move-positive", "home", "park-6000", "stop"),
        "y": ("status", "move-negative", "move-positive", "home", "stop"),
        "z": ("status", "move-negative", "move-positive", "home", "stop"),
        "g": ("status", "commission-home", "close", "open", "open-wide", "stop"),
        "door": ("status", "home", "open", "close", "stop"),
    }
)

_RELATIVE_STEPS = MappingProxyType({"x": 100, "y": 100, "z": 100})


@dataclass(frozen=True)
class AxisDiagnosticAction:
    axis: str
    operation: str
    executor: str
    value: int | None = None


def resolve_axis_diagnostic(axis: str, operation: str) -> AxisDiagnosticAction:
    normalized_axis = str(axis).strip().lower()
    normalized_operation = str(operation).strip().lower()
    allowed = AXIS_DIAGNOSTIC_CATALOG.get(normalized_axis)
    if allowed is None or normalized_operation not in allowed:
        raise AxisDiagnosticContractError(
            f"unsupported BioXP axis diagnostic: axis={normalized_axis!r}, operation={normalized_operation!r}"
        )

    if normalized_operation == "status":
        return AxisDiagnosticAction(normalized_axis, normalized_operation, "status")
    if normalized_operation in {"move-negative", "move-positive"}:
        magnitude = int(_RELATIVE_STEPS[normalized_axis])
        value = -magnitude if normalized_operation == "move-negative" else magnitude
        return AxisDiagnosticAction(normalized_axis, normalized_operation, "relative", value)

    if normalized_operation == "park-6000":
        return AxisDiagnosticAction(normalized_axis, normalized_operation, "absolute", 6000)
    if normalized_operation == "home":
        executor = "door-home" if normalized_axis == "door" else "home"
        return AxisDiagnosticAction(normalized_axis, normalized_operation, executor)
    if normalized_operation == "stop":
        return AxisDiagnosticAction(normalized_axis, normalized_operation, "stop")
    if normalized_operation == "commission-home":
        return AxisDiagnosticAction(normalized_axis, normalized_operation, "gripper-commission-home")
    if normalized_axis == "g" and normalized_operation in {"close", "open", "open-wide"}:
        return AxisDiagnosticAction(normalized_axis, normalized_operation, f"gripper-{normalized_operation}")
    if normalized_axis == "door" and normalized_operation in {"open", "close"}:
        return AxisDiagnosticAction(normalized_axis, normalized_operation, f"door-{normalized_operation}")
    raise AxisDiagnosticContractError(
        f"unmapped BioXP axis diagnostic: axis={normalized_axis!r}, operation={normalized_operation!r}"
    )


def diagnostic_catalog() -> dict[str, Any]:
    axes: dict[str, dict[str, dict[str, Any]]] = {}
    for axis, operations in AXIS_DIAGNOSTIC_CATALOG.items():
        axis_projection: dict[str, dict[str, Any]] = {}
        for operation in operations:
            action = resolve_axis_diagnostic(axis, operation)
            row: dict[str, Any] = {
                "axis": axis,
                "operation": operation,
                "executor": action.executor,
                "physical_motion_possible": operation != "status",
            }
            if action.executor == "relative":
                row["steps"] = action.value
            elif action.executor == "absolute":
                row["position_steps"] = action.value
            if action.executor.startswith("gripper-"):
                row["temporary_action_current_internal"] = True
                row["required_idle_readback"] = {"run": 10, "standby": 10}
            axis_projection[operation] = row
        axes[axis] = axis_projection
    return {
        "schema": "bioxp.oem_axis_diagnostics.v1",
        "caller_supplied_motion_values": False,
        "axes": axes,
    }
