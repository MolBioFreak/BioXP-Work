
"""Proof artifact helpers for fresh OEM parity scaffold."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from .oem_parity_types import OemProgramSpec


def _requires_g_invariant(program: OemProgramSpec) -> bool:
    if "g_current_invariant" in program.required_artifact_fields:
        return True
    return any(step.axis == "g" for step in program.steps)


def build_artifact(program: OemProgramSpec, *, mode: str, operator_ack: str | None = None, blockers: list[str] | None = None) -> dict[str, Any]:
    requires_g = _requires_g_invariant(program)
    return {
        "artifact_format": "bioxp-oem-parity-v1",
        "generated_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
        "program": program.name,
        "mode": mode,
        "source_mode": program.source_mode,
        "oem_symbol": program.oem_symbol,
        "parity_label": program.parity_label,
        "opened_usb": False,
        "physical_motion": False,
        "operator_ack": operator_ack,
        "steps_planned": [step.to_dict() for step in program.steps],
        "steps_executed": [],
        "raw_truth": {
            "axis_speeds": None,
            "axis_currents": None,
            "switches": None,
            "interlocks": None,
            "reference_state": None,
        },
        "g_current_invariant": {
            "required": requires_g,
            "classification": "not_applicable_in_dry_run" if requires_g else "not_required",
        },
        "safety_deviations": sorted({d for step in program.steps for d in step.safety_deviations}),
        "blockers": list(blockers if blockers is not None else program.blockers),
        "ok": True,
    }


def validate_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    required = ["artifact_format", "program", "mode", "opened_usb", "physical_motion", "steps_planned", "raw_truth", "g_current_invariant", "ok"]
    missing = [key for key in required if key not in artifact]
    errors = []
    if artifact.get("artifact_format") != "bioxp-oem-parity-v1":
        errors.append("bad_artifact_format")
    if artifact.get("opened_usb") is not False:
        errors.append("opened_usb_not_false")
    if artifact.get("physical_motion") is not False:
        errors.append("physical_motion_not_false")
    errors.extend(f"missing:{key}" for key in missing)
    return {"ok": not errors, "errors": errors}
