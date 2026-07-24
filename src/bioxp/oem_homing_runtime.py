
"""No-USB/no-motion dry-run executor for fresh OEM homing specs."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .oem_homing_spec import get_program
from .oem_parity_artifacts import build_artifact, validate_artifact
from .oem_parity_types import OemProgramSpec


DOOR_CLOSE_FAILURE_CONDITION = "SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated"

# These are direct controller-facing operations in the source model. Composite
# host methods (initializeMotors/scriptmoveTo), state writes, pipette calls,
# sleeps, and throws intentionally receive no synthetic controller receipt.
_FAKE_CONTROLLER_OPERATIONS = frozenset(
    {
        "axisSearchHome",
        "setMaxCurrent",
        "moveSteps",
        "setHome",
        "setSpeed",
        "moveX",
        "moveZ",
        "doorSearchHome",
        "openThermalDoor",
        "setChillerCoolRate",
    }
)


def _branch_is_active(condition: str | None, simulation: dict[str, Any]) -> bool:
    if condition is None:
        return True
    if condition == DOOR_CLOSE_FAILURE_CONDITION:
        return (
            int(simulation.get("serial_number", 206)) > 9
            and simulation.get("tc_door_closed", True) is False
            and simulation.get("camera_calibrated", True) is True
        )
    if condition == "Calibrated":
        return simulation.get("calibrated", False) is True
    if condition == "TipExist":
        return simulation.get("tip_exists", False) is True
    if condition == "!TipExist":
        return simulation.get("tip_exists", False) is False
    if condition == "TipExistAfterEject":
        return simulation.get("tip_exists_after_eject", False) is True
    if condition == "TipExistAfterEject && errorEvent!=null":
        return simulation.get("tip_exists_after_eject", False) is True and simulation.get("error_event_present", False) is True
    if condition == "TipExistAfterEject && errorEvent==null":
        return simulation.get("tip_exists_after_eject", False) is True and simulation.get("error_event_present", False) is False
    if condition == "TipExist && !TipExistAfterEject":
        return simulation.get("tip_exists", False) is True and simulation.get("tip_exists_after_eject", False) is False
    if condition == "PipetteStatusInitialFailed":
        return simulation.get("pipette_initial_failed", False) is True
    if condition == "PipetteStatusRetryFailed":
        return simulation.get("pipette_initial_failed", False) is True and simulation.get("pipette_retry_failed", False) is True
    if condition.startswith("board_present:"):
        axis = condition.split(":", 1)[1]
        board_presence = simulation.get("board_present") or {}
        return bool(board_presence.get(axis, True))
    # Unknown source conditions are not guessed into execution.
    return False


def _fake_trace(program: OemProgramSpec, simulation: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any] | None]:
    """Produce a source-ordered no-USB trace; it never constructs transport."""
    trace: list[dict[str, Any]] = []
    for step in program.steps:
        branch_active = _branch_is_active(step.branch_condition, simulation)
        controller_operation = step.operation in _FAKE_CONTROLLER_OPERATIONS
        row = {
            "step_id": step.step_id,
            "source_anchor": f"{step.source.file}:{step.source.lines}",
            "operation": step.operation,
            "axis": step.axis,
            "board": step.board,
            "motor": step.motor,
            "params": dict(step.params),
            "wait_ms": step.wait_ms,
            "branch_condition": step.branch_condition,
            "execution": "simulated" if branch_active else "branch_not_taken",
            "transport_ack": (
                {"status": 100, "kind": "fake_no_usb", "transport_only": True}
                if branch_active and controller_operation
                else None
            ),
        }
        trace.append(row)
        if branch_active and step.operation == "throw":
            return trace, {
                "step_id": step.step_id,
                "source_anchor": row["source_anchor"],
                "reason": step.failure_modes[0] if step.failure_modes else "oem_source_throw",
            }
        if branch_active and step.operation == "return":
            return trace, None
    return trace, None


class OemHomingDryRunRuntime:
    def __init__(self, *, artifact_root: str | Path | None = None):
        self.artifact_root = Path(artifact_root) if artifact_root is not None else None

    def run(
        self,
        program_name: str,
        *,
        write_artifact: bool = False,
        operator_ack: str | None = None,
        simulation: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Execute the source model against fake no-USB transport only.

        The trace is proof of source ordering and simulated acknowledgement shape;
        it is explicitly not controller or physical-effect validation.
        """
        try:
            program = get_program(program_name)
        except ValueError as exc:
            return {
                "ok": False,
                "failed_closed": True,
                "error": str(exc),
                "program": program_name,
                "mode": "dry_run",
                "opened_usb": False,
                "physical_motion": False,
            }
        artifact = build_artifact(program, mode="dry_run", operator_ack=operator_ack)
        trace, failure = _fake_trace(program, dict(simulation or {}))
        artifact.update(
            {
                "transport_kind": "fake_no_usb",
                "controller_validated": False,
                "physical_effect_verified": False,
                "steps_executed": trace,
                "virtual_elapsed_ms": sum(
                    int(row["wait_ms"] or 0)
                    for row in trace
                    if row["execution"] == "simulated"
                ),
                "simulation": dict(simulation or {}),
            }
        )
        if failure is not None:
            artifact.update({"ok": False, "failed_closed": True, "failure": failure})
        validation = validate_artifact(artifact)
        if not validation["ok"]:
            artifact["ok"] = False
            artifact["failed_closed"] = True
            artifact["validation"] = validation
        if write_artifact:
            if self.artifact_root is None:
                artifact["ok"] = False
                artifact["failed_closed"] = True
                artifact["error"] = "artifact_root_required"
            else:
                self.artifact_root.mkdir(parents=True, exist_ok=True)
                path = self.artifact_root / f"{program.name}_dry_run.json"
                path.write_text(json.dumps(artifact, indent=2, sort_keys=True))
                artifact["artifact_path"] = str(path)
        return artifact
