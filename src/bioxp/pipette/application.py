"""Controlsuite-shaped pipette application planning with physical execution closed.

The plans preserve the OEM ControlLib coupling between pipettes, gantry, Z,
deck state, pressure handling, receipts, and machine-state reconciliation. This
module never opens transport hardware or executes a physical step.
"""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping


class PipettePhysicalMutationBlocked(RuntimeError):
    """Raised when a physical application workflow is requested."""


class PipetteApplicationPlanner:
    WASTE_LOCATION_ID = 6
    PLUNGER_Z_CURRENT = 31
    FLUID_OFFSET_CLASSES = ("TC", "MS", "OC", "RC", "STRIP")

    @staticmethod
    def _plan(
        operation: str,
        requested_inputs: Mapping[str, Any],
        *,
        steps: list[dict[str, Any]],
        required_completion_evidence: list[str],
        constants: Mapping[str, Any] | None = None,
        source_anchor: str,
    ) -> dict[str, Any]:
        return {
            "ok": True,
            "operation": str(operation),
            "mode": "plan_only",
            "execution_admitted": False,
            "motion_commanded": False,
            "liquid_mutation_commanded": False,
            "controller_acknowledged": False,
            "completion_verified": False,
            "physical_effect_verified": False,
            "state_reconciled": False,
            "requested_inputs": dict(requested_inputs),
            "effective_inputs": None,
            "steps": deepcopy(steps),
            "required_completion_evidence": list(required_completion_evidence),
            "constants": dict(constants or {}),
            "oem_source_anchor": source_anchor,
            "blocker": "physical_pipette_execution_not_authorized",
        }

    def plan_load_tip(
        self,
        *,
        tip_tray: str,
        tip_well: str,
        tip_type: int,
        tip_location: int,
        home_z_after: bool = True,
    ) -> dict[str, Any]:
        if not str(tip_tray).strip() or not str(tip_well).strip():
            raise ValueError("tip_tray and tip_well are required")
        if int(tip_location) not in range(4):
            raise ValueError("tip_location must be an integer channel ID 0..3")
        steps = [
            {"action": "resolve_tip_tray_well", "mutates": False},
            {"action": "move_gantry_to_tip", "mutates": True},
            {"action": "update_machine_location", "mutates": True},
            {"action": "lower_pipette_z", "mutates": True},
            {"action": "apply_overpressure", "mutates": True},
            {"action": "query_tip_status", "mutates": False},
        ]
        if bool(home_z_after):
            steps.append({"action": "home_z", "mutates": True})
        steps.append({"action": "clear_tip_dirty", "mutates": True})
        evidence = [
            "gantry_at_tip_location",
            "z_at_load_depth",
            "overpressure_controller_ack",
            "hardware_tip_loaded_readback",
        ]
        if bool(home_z_after):
            evidence.append("z_home_readback")
        evidence.append("machine_state_reconciled")
        return self._plan(
            "load_tip",
            {
                "tip_tray": str(tip_tray),
                "tip_well": str(tip_well),
                "tip_type": int(tip_type),
                "tip_location": int(tip_location),
                "home_z_after": bool(home_z_after),
            },
            steps=steps,
            required_completion_evidence=evidence,
            source_anchor="ControlLib physical tip-load workflow; PagePipetteControl.btnLoadTip",
        )

    def plan_move_to_waste(self) -> dict[str, Any]:
        return self._plan(
            "move_to_waste",
            {},
            steps=[
                {"action": "verify_z_clearance", "mutates": False},
                {"action": "move_gantry_to_waste", "mutates": True, "location_id": self.WASTE_LOCATION_ID},
                {"action": "update_machine_location", "mutates": True},
                {"action": "verify_gantry_at_waste", "mutates": False},
            ],
            required_completion_evidence=["z_clearance_readback", "gantry_at_waste", "machine_state_reconciled"],
            constants={"waste_location_id": self.WASTE_LOCATION_ID},
            source_anchor="ControlLib waste movement; PagePipetteControl.btnMoveToWaste",
        )

    def plan_detect_fluid(self, *, fluid_class: str) -> dict[str, Any]:
        selected = str(fluid_class).upper()
        if selected not in self.FLUID_OFFSET_CLASSES:
            raise ValueError("fluid_class must be TC, MS, OC, RC, or STRIP")
        return self._plan(
            "detect_fluid",
            {"fluid_class": selected},
            steps=[
                {"action": "resolve_fluid_target", "mutates": False},
                {"action": "verify_plate_strip_state", "mutates": False},
                {"action": "move_gantry_to_fluid_target", "mutates": True},
                {"action": "lower_z_to_detection_start", "mutates": True},
                {"action": "enable_pressure_logging", "mutates": True},
                {"action": "send_fluid_detection", "mutates": True, "wire_command": "BR"},
                {"action": "wait_for_correlated_fluid_completion", "mutates": False},
                {"action": "reconcile_detected_fluid_height", "mutates": True},
                {"action": "park_z_and_gantry", "mutates": True},
            ],
            required_completion_evidence=[
                "plate_strip_state_verified",
                "gantry_target_readback",
                "z_detection_start_readback",
                "controller_fluid_completion",
                "detected_height_reconciled",
                "park_position_readback",
            ],
            constants={"supported_offset_classes": list(self.FLUID_OFFSET_CLASSES)},
            source_anchor="ControlLib fluid detection coupled machine workflow; PagePipetteControl.btnDetectFluid",
        )

    def plan_plunger(self, *, direction: str) -> dict[str, Any]:
        selected = str(direction).lower()
        if selected not in {"up", "down"}:
            raise ValueError("direction must be up or down")
        return self._plan(
            f"plunger_{selected}",
            {"direction": selected},
            steps=[
                {"action": "set_z_current", "mutates": True, "current": self.PLUNGER_Z_CURRENT},
                {"action": f"move_plunger_{selected}", "mutates": True},
                {"action": "verify_z_controller_completion", "mutates": False},
            ],
            required_completion_evidence=["z_current_ack", "z_motion_completion", "z_position_readback"],
            constants={"z_current": self.PLUNGER_Z_CURRENT},
            source_anchor="ControlLib plunger controls; PagePipetteControl.btnPup/btnPDown",
        )

    def status(self) -> dict[str, Any]:
        return {
            "ok": True,
            "mode": "plan_only",
            "execution_admitted": False,
            "physical_effect_verified": False,
            "operations": ["load_tip", "move_to_waste", "detect_fluid", "plunger_up", "plunger_down"],
            "blocker": "physical_pipette_execution_not_authorized",
        }

    def execute(self, operation: str, requested_inputs: Mapping[str, Any]) -> None:
        del requested_inputs
        raise PipettePhysicalMutationBlocked(
            f"Physical pipette application operation {operation!r} is outside the authorized no-motion tranche."
        )
