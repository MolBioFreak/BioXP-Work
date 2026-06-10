
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
from .oem_shadow_readback_live import build_shadow_readback_artifact

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


class _ApiShadowReadbackProvider:
    """Query-only adapter over existing passive API helpers.

    This adapter intentionally calls status/readback helpers only. It must not
    call motion, current-set, switch-mask, arm, home, or prepare-interlock paths.
    """

    def __init__(self, api_mod):
        self._api = api_mod
        self._tester = api_mod._get_tester()

    def axis_snapshot(self, axis: str) -> dict[str, Any]:
        axis_enum = self._api.AxisName(axis)
        payload = self._api._axis_status_payload(self._tester, axis_enum, include_current=True)
        status = payload.get("status", {}) if isinstance(payload, dict) else {}
        switches = payload.get("switch_activity", {}) if isinstance(payload, dict) else {}
        speed_row = status.get("speed") if isinstance(status, dict) else None
        pos_row = status.get("position") if isinstance(status, dict) else None
        run_row = status.get("max_current") if isinstance(status, dict) else None
        standby_row = status.get("standby_current") if isinstance(status, dict) else None
        return {
            "position": pos_row.get("position") if isinstance(pos_row, dict) else pos_row,
            "speed": speed_row.get("speed") if isinstance(speed_row, dict) else speed_row,
            "gap9_left_raw": switches.get("left_raw_active"),
            "gap10_right_raw": switches.get("right_raw_active"),
            "left_disabled": switches.get("left_disabled"),
            "right_disabled": switches.get("right_disabled"),
            "run_current": run_row.get("value") if isinstance(run_row, dict) else None,
            "standby_current": standby_row.get("value") if isinstance(standby_row, dict) else None,
            "raw_status": payload,
        }

    def interlocks(self) -> dict[str, Any]:
        power = self._api._motion_power_status_payload(self._tester)
        return {
            "rail_24v": power.get("rail_24v"),
            "motion_arm": power.get("motion_arm"),
            "latch_override": power.get("latch_override"),
            "hardware_connected": power.get("hardware_connected"),
        }

    def reference_state(self) -> dict[str, Any]:
        return self._api._reference_state_store.snapshot([
            self._api.AxisName.X,
            self._api.AxisName.Y,
            self._api.AxisName.Z,
            self._api.AxisName.GRIPPER,
            self._api.AxisName.DOOR,
        ])


@router.get("/motion/oem/shadow_readback")
async def oem_shadow_readback(axes: str = "x,y,z,g,door") -> dict[str, Any]:
    """Capture query-only OEM shadow/readback truth.

    This endpoint opens only the existing tester/status path and calls passive
    status helpers. It does not command motion, set current, change switch masks,
    arm, home, or mark references.
    """
    from . import api as api_mod

    requested_axes = [axis.value for axis in api_mod._parse_axes_csv(axes)]
    provider = _ApiShadowReadbackProvider(api_mod)
    return await api_mod._run_blocking(
        "OEM shadow readback",
        lambda: build_shadow_readback_artifact(provider, axes=requested_axes),
        timeout_s=max(20.0, 6.0 * float(len(requested_axes))),
    )


@router.post("/motion/oem/shadow_readback/capture")
async def oem_shadow_readback_capture(payload: dict[str, Any] | None = None) -> dict[str, Any]:
    payload = payload or {}
    axes = str(payload.get("axes") or "x,y,z,g,door")
    return await oem_shadow_readback(axes=axes)
