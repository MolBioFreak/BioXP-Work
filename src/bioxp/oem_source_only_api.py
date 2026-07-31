"""Strict OEM-source-only program API.

This app deliberately does not import the legacy BioXP API, runtime worker,
USB/CAN driver, camera, motion service, or the mixed legacy homing router.
It is for deterministic source-contract and fake-transport dry-runs only.
"""
from __future__ import annotations

from typing import Any

from fastapi import FastAPI, HTTPException

from .oem_homing_runtime import OemHomingDryRunRuntime
from .oem_homing_spec import all_programs, get_program


def _simulation(payload: dict[str, Any]) -> dict[str, Any]:
    value = payload.get("simulation") or {}
    if not isinstance(value, dict):
        raise HTTPException(status_code=422, detail="simulation must be an object")
    return dict(value)


async def list_programs() -> dict[str, Any]:
    return {
        "ok": True,
        "mode": "source_only_dry_run",
        "opened_usb": False,
        "physical_motion": False,
        "programs": [
            {
                "name": program.name,
                "oem_symbol": program.oem_symbol,
                "source_mode": program.source_mode,
                "live_allowed_default": program.live_allowed_default,
                "blockers": list(program.blockers),
            }
            for program in all_programs()
        ],
    }


async def get_program_detail(program_name: str) -> dict[str, Any]:
    try:
        program = get_program(program_name)
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    return {
        "ok": True,
        "mode": "source_only_dry_run",
        "opened_usb": False,
        "physical_motion": False,
        "program": program.to_dict(),
    }


async def dry_run_program(program_name: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
    """Run the direct-source program against fake no-USB transport only."""
    payload = payload or {}
    try:
        get_program(program_name)
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    if payload.get("artifact_root"):
        raise HTTPException(status_code=409, detail="artifact_root is unavailable in source-only API")
    return OemHomingDryRunRuntime().run(
        program_name,
        operator_ack=payload.get("operator_ack"),
        simulation=_simulation(payload),
    )


def create_oem_source_only_app() -> FastAPI:
    app = FastAPI(title="BioXP OEM Source-Only Dry-Run", version="1")
    app.add_api_route("/motion/oem/programs", list_programs, methods=["GET"])
    app.add_api_route("/motion/oem/programs/{program_name}", get_program_detail, methods=["GET"])
    app.add_api_route("/motion/oem/{program_name}/dry_run", dry_run_program, methods=["POST"])
    return app


app = create_oem_source_only_app()
