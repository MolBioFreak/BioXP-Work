
"""Fresh no-motion OEM homing dry-run/spec routes.

These routes are intentionally source/dry-run only. They must not construct the
BioXpTester, open USB/CAN/camera transports, or command motion.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException

from .oem_homing_runtime import OemHomingDryRunRuntime
from .oem_homing_spec import all_programs, get_program

router = APIRouter(tags=["OEM homing parity dry-run"])


@router.get("/motion/oem/programs")
async def list_oem_homing_programs() -> dict[str, Any]:
    programs = []
    for program in all_programs():
        data = program.to_dict()
        programs.append({
            "name": data["name"],
            "oem_symbol": data["oem_symbol"],
            "source_mode": data["source_mode"],
            "live_allowed_default": data["live_allowed_default"],
            "parity_label": data["parity_label"],
            "blockers": data["blockers"],
            "dry_run_route": f"/motion/oem/{data['name']}/dry_run",
        })
    return {"ok": True, "opened_usb": False, "physical_motion": False, "programs": programs}


@router.get("/motion/oem/programs/{program_name}")
async def get_oem_homing_program(program_name: str) -> dict[str, Any]:
    try:
        program = get_program(program_name)
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    return {"ok": True, "opened_usb": False, "physical_motion": False, "program": program.to_dict()}


@router.post("/motion/oem/{program_name}/dry_run")
async def dry_run_oem_homing_program(program_name: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
    payload = payload or {}
    artifact_root = payload.get("artifact_root")
    try:
        get_program(program_name)
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    runtime = OemHomingDryRunRuntime(artifact_root=Path(artifact_root) if artifact_root else None)
    return runtime.run(program_name, write_artifact=bool(artifact_root), operator_ack=payload.get("operator_ack"))
