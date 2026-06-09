
"""Contracts/gates for future supervised OEM stepwise live execution.

This module only builds/validates authorization contracts. It does not execute
motion and must remain side-effect free.
"""
from __future__ import annotations

from typing import Any

from .oem_homing_spec import get_program

LIVE_ACK = "OEM_STEPWISE_LIVE"


def build_stepwise_live_contract(program_name: str, step_id: str, *, operator_ack: str | None, shadow_readback: dict[str, Any] | None) -> dict[str, Any]:
    blockers: list[str] = []
    try:
        program = get_program(program_name)
    except ValueError as exc:
        return {"ok": False, "live_allowed": False, "program": program_name, "step_id": step_id, "blockers": [str(exc)]}
    step_ids = {step.step_id for step in program.steps}
    if step_id not in step_ids:
        blockers.append("unknown_step_id")
    if operator_ack != LIVE_ACK:
        blockers.append("operator_ack_required")
    if not isinstance(shadow_readback, dict) or shadow_readback.get("ok") is not True:
        blockers.append("shadow_readback_not_safe")
    g_cls = ((shadow_readback or {}).get("g_current_invariant") or {}).get("classification")
    if g_cls not in {None, "G_CURRENT_IDLE_SAFE"}:
        blockers.append("g_current_not_safe")
    ok = not blockers
    return {
        "ok": ok,
        "live_allowed": ok,
        "program": program.name,
        "step_id": step_id,
        "operator_ack": operator_ack,
        "required_ack": LIVE_ACK,
        "requires_operator_present": True,
        "requires_clear_path_confirmation": True,
        "motion_may_be_commanded": True,
        "full_sequence_allowed": False,
        "blockers": blockers,
        "shadow_readback_summary": shadow_readback,
        "source_mode": program.source_mode,
    }
