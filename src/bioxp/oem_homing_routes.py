
"""Fresh no-motion OEM homing dry-run/spec routes.

These routes are intentionally source/dry-run only. They must not construct the
BioXpTester, open USB/CAN/camera transports, or command motion.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException

from .oem_config import find_oem_machine_config_bundle
from .oem_homing_runtime import OemHomingDryRunRuntime
from .oem_homing_spec import all_programs, get_program
from .oem_compat.machine_state import OemDefaultParameters, OemMachineState
from .oem_compat.pathing import OemPathPlanner
from .oem_compat.position_table import load_bound_oem_position_table
from .oem_shadow_readback_live import build_shadow_readback_artifact

router = APIRouter(tags=["OEM homing parity dry-run"])




@router.get("/motion/oem/machine_config")
async def get_oem_machine_config(root_dir: str | None = None) -> dict[str, Any]:
    """Return read-only OEM machine config binding/provenance.

    This endpoint parses extracted original-SSD appdata config files only. It
    does not open USB, talk to CAN, command motion, mutate reference state, or
    make homing decisions. Secrets/serial are redacted from the response.
    """
    result = find_oem_machine_config_bundle(root_dir)
    result.setdefault("opened_usb", False)
    result.setdefault("physical_motion", False)
    result.setdefault("motion_commanded", False)
    result.setdefault("current_mutation_commanded", False)
    result.setdefault("switch_mask_mutation_commanded", False)
    return result




@router.get("/motion/oem/position_table")
async def get_oem_position_table(root_dir: str | None = None) -> dict[str, Any]:
    """Return the bound OEM PositionTable from original SSD config, read-only."""
    table = load_bound_oem_position_table(root_dir)
    rows = table.rows()
    return {
        "ok": True,
        "source": table.source,
        "schema_version": "bioxp.oem_position_table.v1",
        "position_table_count": len(rows),
        "rows": rows,
        "opened_usb": False,
        "physical_motion": False,
        "motion_commanded": False,
        "source_anchors": [
            "ClassControlInterface.cs:3663-3688 moveTo(location,column,row,Tip10,highPos)",
            "ClassControlInterface.cs:3691-3715 moveTo(location,offsetX,offsetY)",
            "ClassControlInterface.cs:3734-3860 scriptmoveTo initial target-coordinate branch",
            "DefaultParameters.cs:47-59 PSUDO_Z_HOME_LOW/HIGH",
        ],
    }


@router.get("/motion/oem/position_table/plan")
async def plan_oem_position_table_move(
    location_id: str,
    column: int = 0,
    row: int = 0,
    high_pos: bool = True,
    mode: str = "moveTo",
    positionflag: int = 0,
    tip_location: int = -1,
    offset_x: int = 0,
    offset_y: int = 0,
    root_dir: str | None = None,
) -> dict[str, Any]:
    """Dry-run exact OEM PositionTable coordinate planning. No USB/motion."""
    table = load_bound_oem_position_table(root_dir)
    mode_key = str(mode).strip().lower()
    if mode_key == "moveto":
        plan = table.compile_move_to(location_id, column=column, row=row, high_pos=high_pos)
    elif mode_key == "scriptmoveto":
        plan = table.compile_script_move_to(location_id, column=column, row=row, positionflag=positionflag, tip_location=tip_location)
    elif mode_key in {"offset", "moveto_offset", "offsetmoveto"}:
        cfg = find_oem_machine_config_bundle(root_dir)
        axis_limits = (((cfg.get("config") or {}).get("axis_limits") or {}) if isinstance(cfg, dict) else {})
        plan = table.compile_offset_move_to(
            location_id,
            offset_x=offset_x,
            offset_y=offset_y,
            x_high_limit=(axis_limits.get("x") or {}).get("max_steps"),
            y_high_limit=(axis_limits.get("y") or {}).get("max_steps"),
        )
    else:
        raise HTTPException(status_code=400, detail=f"Unsupported OEM position-table plan mode: {mode}")
    plan.update({"ok": True, "schema_version": "bioxp.oem_position_plan.v1", "motion_commanded": False, "current_mutation_commanded": False, "switch_mask_mutation_commanded": False})
    return plan



@router.get("/motion/oem/pathing/default_parameters")
async def get_oem_pathing_default_parameters(
    pseudo_z_home: int | None = None,
    force_high_home: bool = False,
    tiploaded: str | None = None,
    plateloaded: str | None = None,
) -> dict[str, Any]:
    """Dry-run OEM DefaultParameters pseudo-Z state. No USB/motion."""
    params = OemDefaultParameters(pseudo_z_home) if pseudo_z_home is not None else OemDefaultParameters()
    if force_high_home:
        params = params.force_to_high_home()
    elif tiploaded is not None or plateloaded is not None:
        params = params.gantry_load(tiploaded=tiploaded, plateloaded=plateloaded)
    payload = params.to_payload()
    payload.update({"ok": True, "opened_usb": False, "physical_motion": False, "motion_commanded": False})
    return payload


@router.get("/motion/oem/pathing/scriptmove_plan")
async def plan_oem_scriptmove_path(
    location_id: str,
    current_loc: str | None = None,
    current_well: str | None = None,
    column: int = 0,
    row: int = 0,
    positionflag: int = 0,
    current_x: int = 0,
    current_y: int = 0,
    current_z: int = 0,
    tip_loaded: bool = False,
    tip_dirty: bool = False,
    tip_location: int = -1,
    clean_path: bool = False,
    device_type: str = "",
    gripper_confirmed: bool = False,
    pseudo_z_home: int | None = None,
    run_in_parallel: bool = True,
    root_dir: str | None = None,
) -> dict[str, Any]:
    """Dry-run exact OEM scriptmoveTo path/branch planner. No USB/motion."""
    table = load_bound_oem_position_table(root_dir)
    cfg = find_oem_machine_config_bundle(root_dir)
    axis_limits = (((cfg.get("config") or {}).get("axis_limits") or {}) if isinstance(cfg, dict) else {})
    planner = OemPathPlanner(
        table,
        x_high_limit=int((axis_limits.get("x") or {}).get("max_steps") or 90263),
        y_high_limit=int((axis_limits.get("y") or {}).get("max_steps") or 102956),
    )
    state = OemMachineState.from_query(
        current_location_id=current_loc,
        current_well_id=current_well,
        current_x=current_x,
        current_y=current_y,
        current_z=current_z,
        tip_loaded=tip_loaded,
        tip_dirty=tip_dirty,
        tip_location=tip_location,
        clean_path=clean_path,
        device_type=device_type,
        gripper_confirmed=gripper_confirmed,
        pseudo_z_home=pseudo_z_home,
    )
    plan = planner.plan_script_move_to(
        current_loc=current_loc,
        location_id=location_id,
        column=column,
        row=row,
        positionflag=positionflag,
        state=state,
        run_in_parallel=run_in_parallel,
    )
    plan.update({"current_mutation_commanded": False, "switch_mask_mutation_commanded": False})
    return plan

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
            self._api.AxisName.THERMAL_DOOR,
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

    def _capture() -> dict[str, Any]:
        try:
            provider = _ApiShadowReadbackProvider(api_mod)
            return build_shadow_readback_artifact(provider, axes=requested_axes)
        except Exception as exc:
            return {
                "ok": False,
                "failed_closed": True,
                "motion_commanded": False,
                "current_mutation_commanded": False,
                "switch_mask_mutation_commanded": False,
                "axes_requested": requested_axes,
                "error": str(exc),
                "blockers": ["shadow_readback_unavailable"],
            }

    return await api_mod._run_blocking(
        "OEM shadow readback",
        _capture,
        timeout_s=max(20.0, 6.0 * float(len(requested_axes))),
    )


@router.post("/motion/oem/shadow_readback/capture")
async def oem_shadow_readback_capture(payload: dict[str, Any] | None = None) -> dict[str, Any]:
    payload = payload or {}
    axes = str(payload.get("axes") or "x,y,z,g,door")
    return await oem_shadow_readback(axes=axes)
