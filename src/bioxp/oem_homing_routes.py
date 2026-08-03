
"""Fresh no-motion OEM homing dry-run/spec routes.

These routes are intentionally source/dry-run only. They must not construct the
BioXpTester, open USB/CAN/camera transports, or command motion.
"""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import time
from typing import Any

from fastapi import APIRouter, HTTPException

from .oem_config import find_oem_machine_config_bundle
from .oem_homing_runtime import OemHomingDryRunRuntime
from .oem_homing_spec import all_programs, get_program
from .oem_compat.machine_state import OemDefaultParameters, OemMachineState
from .oem_compat.pathing import OemPathPlanner
from .oem_compat.movement_readiness import build_movement_readiness_comparison
from .oem_compat.position_table import load_bound_oem_position_table
from .oem_shadow_readback_live import build_shadow_readback_artifact
from .runtime_state import OemRuntimeStateError, get_active_oem_runtime_state_store

router = APIRouter(tags=["OEM homing parity dry-run"])


def _plain_response(value: Any) -> Any:
    """Materialize immutable OEM snapshot views as ordinary JSON containers."""
    if isinstance(value, Mapping):
        return {str(key): _plain_response(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain_response(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def _require_bound_snapshot(root_dir: str | None) -> None:
    if root_dir is not None:
        raise HTTPException(
            status_code=409,
            detail="root_dir is diagnostic-only; production routes use the process-bound accepted OEM snapshot",
        )




@router.get("/motion/oem/machine_config")
async def get_oem_machine_config(root_dir: str | None = None) -> dict[str, Any]:
    """Return read-only OEM machine config binding/provenance.

    This endpoint parses extracted original-SSD appdata config files only. It
    does not open USB, talk to CAN, command motion, mutate reference state, or
    make homing decisions. Secrets/serial are redacted from the response.
    """
    _require_bound_snapshot(root_dir)
    result = find_oem_machine_config_bundle()
    try:
        result["transactional_runtime_state"] = get_active_oem_runtime_state_store().status_projection()
    except OemRuntimeStateError:
        result["transactional_runtime_state"] = {"ok": False, "status": "unbound"}
    result.setdefault("opened_usb", False)
    result.setdefault("physical_motion", False)
    result.setdefault("motion_commanded", False)
    result.setdefault("current_mutation_commanded", False)
    result.setdefault("switch_mask_mutation_commanded", False)
    return _plain_response(result)




@router.get("/motion/oem/position_table")
async def get_oem_position_table(root_dir: str | None = None) -> dict[str, Any]:
    """Return the bound OEM PositionTable from original SSD config, read-only."""
    _require_bound_snapshot(root_dir)
    table = load_bound_oem_position_table()
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
    _require_bound_snapshot(root_dir)
    table = load_bound_oem_position_table()
    mode_key = str(mode).strip().lower()
    if mode_key == "moveto":
        plan = table.compile_move_to(location_id, column=column, row=row, high_pos=high_pos)
    elif mode_key == "scriptmoveto":
        plan = table.compile_script_move_to(location_id, column=column, row=row, positionflag=positionflag, tip_location=tip_location)
    elif mode_key in {"offset", "moveto_offset", "offsetmoveto"}:
        cfg = find_oem_machine_config_bundle()
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





@router.get("/motion/oem/movement_readiness/comparison")
async def get_oem_movement_readiness_comparison() -> dict[str, Any]:
    """No-motion OEM-vs-new movement-test readiness/gap comparison."""
    return build_movement_readiness_comparison()


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
    plate_on_gantry: int | str | None = None,
    location19_y: int | None = None,
    pseudo_z_home: int | None = None,
    run_in_parallel: bool = True,
    root_dir: str | None = None,
) -> dict[str, Any]:
    """Dry-run exact OEM scriptmoveTo path/branch planner. No USB/motion."""
    _require_bound_snapshot(root_dir)
    table = load_bound_oem_position_table()
    cfg = find_oem_machine_config_bundle()
    axis_limits = (((cfg.get("config") or {}).get("axis_limits") or {}) if isinstance(cfg, dict) else {})
    x_limit = (axis_limits.get("x") or {}).get("max_steps")
    y_limit = (axis_limits.get("y") or {}).get("max_steps")
    if x_limit is None or y_limit is None:
        raise HTTPException(status_code=503, detail="immutable OEM machine snapshot axis limits are unavailable")
    planner = OemPathPlanner(
        table,
        x_high_limit=int(x_limit),
        y_high_limit=int(y_limit),
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
        plate_on_gantry=plate_on_gantry,
        location19_y=location19_y,
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



def _execution_preview_for_step(step: dict[str, Any]) -> dict[str, Any]:
    op = str(step.get("op") or "")
    payload = dict(step)
    payload.setdefault("motion_commanded", False)
    payload.setdefault("opened_usb", False)
    payload.setdefault("physical_motion", False)
    if op == "moveTo":
        payload["would_call"] = "/motion/axis/absolute_sequence"
        payload["would_execute"] = [
            {"axis": "x", "position_steps": payload.get("x")},
            {"axis": "y", "position_steps": payload.get("y")},
            {"axis": "z", "position_steps": payload.get("z")},
        ]
    elif op == "moveXY":
        payload["would_call"] = "/motion/axis/absolute_sequence"
        payload["would_execute"] = [
            {"axis": "x", "position_steps": payload.get("x")},
            {"axis": "y", "position_steps": payload.get("y")},
        ]
    elif op == "moveX":
        payload["would_call"] = "/motion/axis/absolute"
        payload["would_execute"] = {"axis": "x", "position_steps": payload.get("x")}
    elif op == "moveY":
        payload["would_call"] = "/motion/axis/absolute"
        payload["would_execute"] = {"axis": "y", "position_steps": payload.get("y")}
    elif op == "moveZ":
        payload["would_call"] = "/motion/axis/absolute"
        payload["would_execute"] = {"axis": "z", "position_steps": payload.get("z")}
    elif op == "moveSteps":
        payload["would_call"] = "/motion/axis/relative"
        payload["would_execute"] = {"axis": payload.get("axis"), "steps": payload.get("delta")}
    elif op == "parallel":
        payload["would_call"] = "/motion/axis/absolute_sequence_parallel_semantics_disabled"
        payload["would_execute"] = [_execution_preview_for_step(child) for child in payload.get("steps") or []]
    elif op == "sleep":
        payload["would_call"] = "sleep"
        payload["would_execute"] = {"milliseconds": payload.get("milliseconds")}
    else:
        payload["would_call"] = "unsupported"
        payload["would_execute"] = None
    return payload


class _ApiPathMotionExecutor:
    """Adapter from OEM path-plan steps to existing guarded API motion primitives."""

    def __init__(self, api_mod):
        self._api = api_mod
        self._tester = api_mod._get_tester()

    def absolute(self, axis: str, position_steps: int, *, speed: int | None = None, acc: int | None = None, wait_timeout_s: float = 12.0) -> dict[str, Any]:
        axis_enum = self._api.AxisName(str(axis))
        if axis_enum is self._api.AxisName.X and self._api._serial206_oem_initialization_provider is not None:
            return self._api._execute_serial206_motion_intent("move_absolute", {"position_steps": int(position_steps), "wait_timeout_s": float(wait_timeout_s), "source_mode": "oem_path.moveX"})
        return self._api._execute_absolute_move(
            self._tester,
            axis_enum,
            int(position_steps),
            float(wait_timeout_s),
            speed=speed,
            acc=acc,
        )

    def move_xy(self, x: int, y: int, *, wait_timeout_s: float = 12.0) -> dict[str, Any]:
        if self._api._serial206_oem_initialization_provider is not None:
            return self._api._execute_serial206_motion_intent("move_xy", {"x": int(x), "y": int(y), "wait_timeout_s": float(wait_timeout_s)})
        return self._tester.oem_move_xy(int(x), int(y), wait_timeout_s=float(wait_timeout_s))

    def relative(self, axis: str, steps: int, *, speed: int | None = None, acc: int | None = None, wait_timeout_s: float = 12.0) -> dict[str, Any]:
        axis_enum = self._api.AxisName(str(axis))
        return self._api._execute_relative_move(
            self._tester,
            axis_enum,
            int(steps),
            float(wait_timeout_s),
            speed=speed,
            acc=acc,
            reuse_prepared=False,
        )

    def sleep(self, milliseconds: int) -> dict[str, Any]:
        delay_s = max(0.0, min(float(milliseconds) / 1000.0, 30.0))
        time.sleep(delay_s)
        return {"ok": True, "milliseconds": int(milliseconds), "slept_s": delay_s}


def _step_result_failed(result: Any) -> bool:
    return not isinstance(result, dict) or result.get("ok") is False or isinstance(result.get("motion_failure"), dict)


def _execute_oem_step_live(
    step: dict[str, Any],
    motion_executor: Any,
    *,
    wait_timeout_s: float,
    speed: int | None,
    acc: int | None,
    path: str,
    pseudo_z_home_steps: int | None = None,
) -> dict[str, Any]:
    op = str(step.get("op") or "")
    result: dict[str, Any] = {"ok": True, "path": path, "op": op, "source_step": dict(step), "results": []}

    def run_absolute(axis: str, position: Any) -> bool:
        if position is None:
            result.update({"ok": False, "error": f"{op} missing {axis} target"})
            return False
        if axis == "z" and callable(getattr(motion_executor, "oem_move_z", None)):
            sub = motion_executor.oem_move_z(
                int(position),
                pseudo_home_steps=int(pseudo_z_home_steps if pseudo_z_home_steps is not None else 65000),
                motor_current=31,
                wait_for_stop=True,
            )
        elif callable(getattr(motion_executor, "oem_move_axis_absolute", None)):
            sub = motion_executor.oem_move_axis_absolute(axis, int(position), wait_for_stop=True)
        else:
            result.update({"ok": False, "error": f"exact OEM absolute primitive is not bound for {axis}"})
            return False
        row = {"axis": axis, "command": "absolute", "target_position": int(position), "result": sub}
        result["results"].append(row)
        if _step_result_failed(sub):
            result.update({"ok": False, "failed_axis": axis, "error": f"{op} absolute {axis} step failed"})
            return False
        return True

    if op == "moveTo":
        move_to = getattr(motion_executor, "oem_move_to", None)
        if callable(move_to):
            if any(step.get(axis) is None for axis in ("x", "y", "z")):
                result.update({"ok": False, "error": "moveTo missing x/y/z target"})
            else:
                authority = step.get("move_to_authority")
                plate_on_gantry = authority.get("plate_on_gantry") if isinstance(authority, Mapping) else None
                plate_authority_valid = (
                    plate_on_gantry is None
                    or (type(plate_on_gantry) is int and type(plate_on_gantry) is not bool)
                    or type(plate_on_gantry) is str
                )
                if (
                    not isinstance(authority, Mapping)
                    or type(authority.get("gripper_confirmed")) is not bool
                    or type(authority.get("tip_loaded")) is not bool
                    or not plate_authority_valid
                    or (
                        plate_on_gantry in {4, 5}
                        and type(authority.get("location19_y")) is not int
                    )
                ):
                    result.update({"ok": False, "error": "OEM moveTo branch authority is not bound"})
                else:
                    sub = move_to(
                        int(step["x"]),
                        int(step["y"]),
                        int(step["z"]),
                        pseudo_home_steps=int(pseudo_z_home_steps if pseudo_z_home_steps is not None else 65000),
                        run_in_parallel=bool(step.get("run_in_parallel", True)),
                        wait_timeout_s=wait_timeout_s,
                        gripper_confirmed=authority["gripper_confirmed"],
                        tip_loaded=authority["tip_loaded"],
                        plate_on_gantry=authority.get("plate_on_gantry"),
                        location19_y=authority.get("location19_y"),
                    )
                    result["results"].append({"command": "moveTo", "result": sub})
                    if _step_result_failed(sub):
                        result.update({"ok": False, "error": "moveTo composite step failed"})
        else:
            result.update({"ok": False, "error": "OEM moveTo composite is not bound"})
    elif op == "moveXY":
        move_xy = getattr(motion_executor, "move_xy", None) or getattr(motion_executor, "oem_move_xy", None)
        if callable(move_xy):
            x = step.get("x")
            y = step.get("y")
            if x is None or y is None:
                result.update({"ok": False, "error": "moveXY missing x/y target"})
            else:
                sub = move_xy(int(x), int(y), wait_timeout_s=wait_timeout_s)
                result["results"].append({"command": "moveXY", "result": sub})
                if _step_result_failed(sub):
                    result.update({"ok": False, "error": "moveXY step failed"})
        else:
            result.update({"ok": False, "error": "exact OEM moveXY composite is not bound"})
    elif op == "moveX":
        run_absolute("x", step.get("x"))
    elif op == "moveY":
        run_absolute("y", step.get("y"))
    elif op == "moveZ":
        run_absolute("z", step.get("z"))
    elif op == "moveSteps":
        axis = str(step.get("axis") or "")
        delta = step.get("delta")
        if axis not in {"x", "y", "z", "g", "door"} or delta is None:
            result.update({"ok": False, "error": "moveSteps requires axis and delta"})
        else:
            sub = motion_executor.relative(axis, int(delta), speed=speed, acc=acc, wait_timeout_s=wait_timeout_s)
            result["results"].append({"axis": axis, "command": "relative", "steps": int(delta), "result": sub})
            if _step_result_failed(sub):
                result.update({"ok": False, "failed_axis": axis, "error": "moveSteps relative step failed"})
    elif op == "sleep":
        ms = int(step.get("milliseconds") or 0)
        sub = motion_executor.sleep(ms)
        result["results"].append({"command": "sleep", "milliseconds": ms, "result": sub})
        if _step_result_failed(sub):
            result.update({"ok": False, "error": "sleep step failed"})
    elif op == "parallel":
        result.update({
            "ok": False,
            "parallel_semantics": "fail_closed_exact_parallel_leaf_not_bound",
            "error": "exact OEM parallel leaf executor is not bound",
        })
    else:
        result.update({"ok": False, "error": f"unsupported OEM path step op: {op}"})
    return result


def _execute_oem_steps_live(
    steps: list[dict[str, Any]],
    motion_executor: Any,
    *,
    wait_timeout_s: float,
    speed: int | None,
    acc: int | None,
    pseudo_z_home_steps: int | None = None,
) -> dict[str, Any]:
    results: list[dict[str, Any]] = []
    motion_commanded = False
    for idx, step in enumerate(steps):
        op = str(step.get("op") or "")
        step_result = _execute_oem_step_live(
            step,
            motion_executor,
            wait_timeout_s=wait_timeout_s,
            speed=speed,
            acc=acc,
            path=str(idx),
            pseudo_z_home_steps=pseudo_z_home_steps,
        )
        results.append(step_result)
        if op != "sleep":
            motion_commanded = True
        if _step_result_failed(step_result):
            return {
                "ok": False,
                "failed_closed": True,
                "executor_status": "failed_closed_step_error",
                "motion_commanded": motion_commanded,
                "execution_results": results,
                "error": step_result.get("error") or "OEM path step failed",
            }
    return {
        "ok": True,
        "executor_status": "live_step_execution_complete",
        "motion_commanded": motion_commanded,
        "execution_results": results,
    }


async def _execute_oem_scriptmove_path_impl(payload: dict[str, Any] | None = None, motion_executor: Any | None = None) -> dict[str, Any]:
    """Guarded OEM scriptmoveTo executor handoff.

    Default mode is preview-only and commands no motion. Live mode requires the
    explicit OEM_PATH_EXECUTE ack plus a non-empty reason, then executes the
    planned OEM steps through the existing guarded motion primitives. The route
    still reports physical_motion=false because controller execution is not
    independent physical proof.
    """
    payload = payload or {}
    mode = str(payload.get("mode") or "dry_run").strip().lower()
    if mode not in {"dry_run", "preview", "live"}:
        raise HTTPException(status_code=400, detail=f"unsupported scriptmove_execute mode: {mode}")
    if mode == "live":
        raise HTTPException(
            status_code=409,
            detail="legacy generic scriptmove live execution is quarantined; only typed OEM lifecycle parity work is permitted",
        )
    live_enabled = False
    if live_enabled and payload.get("operator_ack") != "OEM_PATH_EXECUTE":
        raise HTTPException(status_code=409, detail="operator_ack OEM_PATH_EXECUTE required for live OEM path execution")
    reason = str(payload.get("reason") or payload.get("operator_note") or "").strip()
    if live_enabled and not reason:
        raise HTTPException(status_code=409, detail="live OEM path execution requires a non-empty reason/operator_note")
    plan = await plan_oem_scriptmove_path(
        location_id=str(payload.get("location_id") or "UNKNOWN"),
        current_loc=payload.get("current_loc"),
        current_well=payload.get("current_well"),
        column=int(payload.get("column") or 0),
        row=int(payload.get("row") or 0),
        positionflag=int(payload.get("positionflag") or 0),
        current_x=int(payload.get("current_x") or 0),
        current_y=int(payload.get("current_y") or 0),
        current_z=int(payload.get("current_z") or 0),
        tip_loaded=bool(payload.get("tip_loaded") or False),
        tip_dirty=bool(payload.get("tip_dirty") or False),
        tip_location=int(payload.get("tip_location") if payload.get("tip_location") is not None else -1),
        clean_path=bool(payload.get("clean_path") or False),
        device_type=str(payload.get("device_type") or ""),
        gripper_confirmed=bool(payload.get("gripper_confirmed") or False),
        plate_on_gantry=payload.get("plate_on_gantry"),
        location19_y=payload.get("location19_y"),
        pseudo_z_home=payload.get("pseudo_z_home"),
        run_in_parallel=bool(payload.get("run_in_parallel") if payload.get("run_in_parallel") is not None else True),
        root_dir=payload.get("root_dir"),
    )
    execution_steps = [_execution_preview_for_step(step) for step in plan.get("steps") or []]
    base = {
        "ok": True,
        "schema_version": "bioxp.oem_scriptmove_execution.v1",
        "mode": "live" if live_enabled else "dry_run",
        "plan": plan,
        "execution_steps": execution_steps,
        "opened_usb": False,
        "motion_commanded": False,
        "physical_motion": False,
        "current_mutation_commanded": False,
        "switch_mask_mutation_commanded": False,
        "operator_ack": payload.get("operator_ack") if live_enabled else None,
        "reason": reason or None,
        "live_motion_note": "Controller execution is not independent physical proof; supervised operator/camera observation is still required.",
    }
    if not live_enabled:
        base["executor_status"] = "preview_only"
        return base

    wait_timeout_s = float(payload.get("wait_timeout_s") or 12.0)
    speed = payload.get("speed")
    acc = payload.get("acc")
    speed_i = None if speed is None else int(speed)
    acc_i = None if acc is None else int(acc)

    if motion_executor is None:
        from . import api as api_mod

        def _run_live() -> dict[str, Any]:
            api_mod._require_motion_route_ready()
            executor = _ApiPathMotionExecutor(api_mod)
            return _execute_oem_steps_live(
                list(plan.get("steps") or []),
                executor,
                wait_timeout_s=wait_timeout_s,
                speed=speed_i,
                acc=acc_i,
            )

        live_result = await api_mod._run_blocking(
            "OEM scriptmoveTo guarded live executor",
            _run_live,
            timeout_s=max(30.0, min(300.0, wait_timeout_s * max(1, len(plan.get("steps") or [])) * 4.0)),
        )
    else:
        live_result = _execute_oem_steps_live(
            list(plan.get("steps") or []),
            motion_executor,
            wait_timeout_s=wait_timeout_s,
            speed=speed_i,
            acc=acc_i,
        )

    base.update(live_result)
    base["opened_usb"] = True
    base["current_mutation_commanded"] = bool(base.get("motion_commanded"))
    base["switch_mask_mutation_commanded"] = False
    return base


@router.post("/motion/oem/pathing/scriptmove_execute")
async def execute_oem_scriptmove_path(payload: dict[str, Any] | None = None) -> dict[str, Any]:
    return await _execute_oem_scriptmove_path_impl(payload)


@router.post("/motion/oem/home_gz")
async def execute_oem_home_gz(payload: dict[str, Any] | None = None) -> dict[str, Any]:
    """Dedicated ClassControlInterface.homeGZ composite command."""
    payload = payload or {}
    mode = str(payload.get("mode") or "dry_run").strip().lower()
    if mode not in {"dry_run", "preview", "live"}:
        raise HTTPException(status_code=400, detail=f"unsupported homeGZ mode: {mode}")
    if mode == "live":
        raise HTTPException(
            status_code=409,
            detail="homeGZ live execution remains quarantined by caught_plate_recovery_not_live_proven",
        )
    delay_s = int(payload.get("delay_s") or payload.get("delay") or 0)
    if delay_s < 0 or delay_s > 60:
        raise HTTPException(status_code=400, detail="homeGZ delay must be between 0 and 60 seconds")
    dry_run = {
        "ok": True,
        "mode": "dry_run",
        "opened_usb": False,
        "physical_motion": False,
        "motion_commanded": False,
        "command": "homeGZ",
        "delay_s": delay_s,
        "sequence": [
            "setGripperCurrent(31)",
            "moveZ(PSUDO_Z_HOME, waitforstop=false)",
            "sleep(delay * 1000)",
            "goHome(gripper, version-dependent speed, wait=true)",
            "caught-plate recovery or moveZ(PSUDO_Z_HOME)",
            "restore gripper current 10 for GripperVersion=1",
        ],
        "source_anchor": "ClassControlInterface.homeGZ:4657-4687",
    }
    return dry_run


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
    return runtime.run(
        program_name,
        write_artifact=bool(artifact_root),
        operator_ack=payload.get("operator_ack"),
        simulation=dict(payload.get("simulation") or {}),
    )


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
    """Project the shadow-readback domain of one completed canonical snapshot."""
    from . import api as api_mod

    requested_axes = [axis.value for axis in api_mod._parse_axes_csv(axes)]
    projection = api_mod.hardware_state.project("shadow_readback")
    row = (projection.get("domains") or {}).get("shadow_readback") or {}
    observed = row.get("observation") if row.get("status") == "observed" else None
    axes_observed = ((observed or {}).get("axes") or {}).get("rows") or {}
    return {
        **projection,
        "ok": observed is not None,
        "failed_closed": observed is None,
        "axes_requested": requested_axes,
        "axes": {axis: axes_observed.get(axis) for axis in requested_axes},
        "interlocks": None if observed is None else observed.get("interlocks"),
        "reference_state": None if observed is None else observed.get("reference_state"),
        "motion_commanded": False,
        "current_mutation_commanded": False,
        "switch_mask_mutation_commanded": False,
        "error": None if observed is not None else "canonical shadow_readback observation unavailable",
    }


@router.post("/motion/oem/shadow_readback/capture")
async def oem_shadow_readback_capture(payload: dict[str, Any] | None = None) -> dict[str, Any]:
    del payload
    raise HTTPException(
        status_code=409,
        detail={
            "error": "shadow_readback_collection_moved",
            "required_route": "POST /hardware/snapshot/collect",
            "required_domain": "shadow_readback",
            "hardware_queried": False,
        },
    )
