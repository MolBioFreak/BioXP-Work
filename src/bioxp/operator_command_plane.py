"""Durable robot-owned X/Y/Z command and method execution plane.

This module owns normal movement admission, global ordering, lifecycle rows,
recovery, and the independent Stop/Abort lane. Existing source-shaped provider
routes remain the only hardware execution boundary.
"""
from __future__ import annotations

import asyncio
import fcntl
import hashlib
import json
import os
import sqlite3
import threading
import time
import uuid
from collections.abc import Callable, Mapping
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from fastapi import APIRouter, FastAPI, HTTPException, Query, Request
from pydantic import BaseModel, ConfigDict, Field, StrictInt, field_validator
import rfc8785

from .operator_controls import (
    _dispatch_asgi,
    _DISPATCH_CONTEXT,
    _assess_action,
    _bounded_json,
    _controller_acknowledged,
    _MAX_INPUT_BYTES,
)
from .operator_receipt_store import runtime_state_root
from .runtime_audit_store import runtime_write_coordinator
from .oem_runtime_store import verify_canonical_runtime_database
from .runtime_audit_store import open_runtime_connection

COMMAND_SCHEMA = "bioxp.operator_command_request.v1"
ACTION_REQUEST_SCHEMA = "bioxp.operator_action_request.v2"
INTERRUPT_REQUEST_SCHEMA = "bioxp.operator_interrupt_request.v1"
RECEIPT_SCHEMA = "bioxp.operator_command_receipt.v1"
METHOD_SCHEMA = "bioxp.operator_method_request.v1"
METHOD_RECEIPT_SCHEMA = "bioxp.operator_method_receipt.v1"
ROBOT_IDENTITY = os.getenv("BIOXP_ROBOT_IDENTITY", "serial206").strip() or "serial206"
TRANSITION_SCHEMA = "bioxp.operator_transition_feed.v1"
RECOVERY_SCHEMA = "bioxp.operator_recovery.v1"


def _positive_env(name: str, default: int) -> int:
    try:
        value = int(os.environ.get(name, str(default)))
    except ValueError:
        return default
    return value if value > 0 else default


COMMAND_CAPACITY = _positive_env("BIOXP_OPERATOR_COMMAND_CAPACITY", 10_000)
COMMAND_BYTES_CAPACITY = _positive_env("BIOXP_OPERATOR_COMMAND_BYTES_CAPACITY", 16 * 1024 * 1024)
MAX_METHOD_COMMANDS = 10_000

COMMAND_STATES = frozenset({
    "queued", "dispatched", "issued_pending", "stop_requested", "abort_requested", "completed",
    "failed", "ambiguous", "stopped", "aborted", "cancelled", "cleared",
    "interrupted",
})
COMMAND_NONTERMINAL = frozenset({"queued", "dispatched", "issued_pending", "stop_requested", "abort_requested"})
COMMAND_TERMINAL = COMMAND_STATES - COMMAND_NONTERMINAL
METHOD_STATES = frozenset({
    "queued", "running", "pause_requested", "paused", "cancel_requested",
    "stopping", "aborting", "recovery_required", "completed", "failed",
    "cancelled", "stopped", "aborted", "interrupted",
})
METHOD_NONTERMINAL = frozenset({
    "queued", "running", "pause_requested", "paused", "cancel_requested",
    "stopping", "aborting", "recovery_required",
})

ALLOWED_ACTIONS = frozenset({
    "oem.x.prepare",
    "oem.x.diagnostic_home_axis",
    "oem.x.startup_home",
    "oem.x.move_to_origin_home",
    "oem.x.caught_plate_recovery_home",
    "oem.x.set_home",
    "oem.x.set_max_speed",
    "oem.x.set_max_acc",
    "oem.x.restore_original_speed",
    "oem.x.set_stall_guard",
    "oem.x.reconcile_switch_masks",
    "oem.x.observe",
    "oem.z.manual_home",
    "oem.z.prepare",
    "oem.z.clear",
    "oem.z.diagnostic_home_axis",
    "oem.z.resume_after_abort",
    "oem.z.set_home",
    "oem.z.move_z_home",
    "oem.z.control",
    "oem.z.path_clean_mode",
    "oem.z.move_gz",
    "oem.z.lower_pipette",
    "oem.z.lift_pipette",
    "oem.z.self_test",
    "oem.z.reconcile_switch_masks",
    "oem.z.scriptmove_to",
    "oem.z.home_gz",
    "oem.z.observe",
    "oem.z.move_steps",
    "oem.z.move_absolute",
    "oem.x.manual_panel_home",
    "oem.x.move_steps",
    "oem.x.move_absolute",
    "oem.y.manual_panel_home",

    "oem.y.move_steps",
    "oem.y.move_absolute",
    "oem.xy.move_absolute",
    "oem.xy.home",
})
INTERRUPT_ACTIONS = frozenset({
    "oem.x.stop", "oem.y.stop", "oem.z.stop", "oem.abort_all", "oem.z.abort",
})
CANONICAL_ACTIONS = ALLOWED_ACTIONS | INTERRUPT_ACTIONS
XZ_NORMAL_ACTIONS = frozenset({
    "oem.x.prepare",
    "oem.x.diagnostic_home_axis",
    "oem.x.startup_home",
    "oem.x.move_to_origin_home",
    "oem.x.caught_plate_recovery_home",
    "oem.x.set_home",
    "oem.x.set_max_speed",
    "oem.x.set_max_acc",
    "oem.x.restore_original_speed",
    "oem.x.set_stall_guard",
    "oem.x.reconcile_switch_masks",
    "oem.x.observe",
    "oem.z.manual_home",
    "oem.z.prepare",
    "oem.z.clear",
    "oem.z.diagnostic_home_axis",
    "oem.z.resume_after_abort",
    "oem.z.set_home",
    "oem.z.move_z_home",
    "oem.z.control",
    "oem.z.path_clean_mode",
    "oem.z.move_gz",
    "oem.z.lower_pipette",
    "oem.z.lift_pipette",
    "oem.z.self_test",
    "oem.z.reconcile_switch_masks",
    "oem.z.scriptmove_to",
    "oem.z.home_gz",
    "oem.z.observe",
    "oem.z.move_steps",
    "oem.z.move_absolute",
    "oem.x.manual_panel_home",
    "oem.x.move_steps",
    "oem.x.move_absolute",

})
STRICT_METHOD_ACTIONS = frozenset({
    "oem.xy.move_absolute",
    "oem.xy.home",
    "oem.xyz.move_to",
})
XZ_CANONICAL_ACTION_BY_TARGET_PATH = {
    "/motion/oem/x/prepare": "oem.x.prepare",
    "/motion/oem/x/manual_home": "oem.x.manual_panel_home",
    "/motion/oem/x/diagnostic_home_axis": "oem.x.diagnostic_home_axis",
    "/motion/oem/x/startup_home": "oem.x.startup_home",
    "/motion/oem/x/move_to_origin_home": "oem.x.move_to_origin_home",
    "/motion/oem/x/caught_plate_recovery_home": "oem.x.caught_plate_recovery_home",
    "/motion/oem/x/set_home": "oem.x.set_home",
    "/motion/oem/x/set_max_speed": "oem.x.set_max_speed",
    "/motion/oem/x/set_max_acc": "oem.x.set_max_acc",
    "/motion/oem/x/restore_original_speed": "oem.x.restore_original_speed",
    "/motion/oem/x/set_stall_guard": "oem.x.set_stall_guard",
    "/motion/oem/x/reconcile_switch_masks": "oem.x.reconcile_switch_masks",
    "/motion/oem/x/observation": "oem.x.observe",
    "/motion/oem/x/move_steps": "oem.x.move_steps",
    "/motion/oem/x/move_absolute": "oem.x.move_absolute",
    "/motion/oem/z/clear": "oem.z.clear",
    "/motion/oem/z/prepare": "oem.z.prepare",
    "/motion/oem/z/diagnostic_home_axis": "oem.z.diagnostic_home_axis",
    "/motion/oem/z/resume_after_abort": "oem.z.resume_after_abort",
    "/motion/oem/z/set_home": "oem.z.set_home",
    "/motion/oem/z/move_z_home": "oem.z.move_z_home",
    "/motion/oem/z/control": "oem.z.control",
    "/motion/oem/z/path_clean_mode": "oem.z.path_clean_mode",
    "/motion/oem/z/move_gz": "oem.z.move_gz",
    "/motion/oem/z/lower_pipette": "oem.z.lower_pipette",
    "/motion/oem/z/lift_pipette": "oem.z.lift_pipette",
    "/motion/oem/z/self_test": "oem.z.self_test",
    "/motion/oem/z/reconcile_switch_masks": "oem.z.reconcile_switch_masks",
    "/motion/oem/pathing/scriptmove_execute": "oem.z.scriptmove_to",
    "/motion/oem/home_gz": "oem.z.home_gz",
    "/motion/oem/z/observation": "oem.z.observe",
}
XZ_CANONICAL_ACTION_BY_AXIS_TARGET = {
    ("/motion/oem/manual/relative", "x"): "oem.x.move_steps",
    ("/motion/oem/manual/relative", "z"): "oem.z.move_steps",
    ("/motion/oem/manual/absolute", "x"): "oem.x.move_absolute",
    ("/motion/oem/manual/absolute", "z"): "oem.z.move_absolute",
    ("/motion/oem/manual/home", "x"): "oem.x.manual_panel_home",
    ("/motion/oem/manual/home", "z"): "oem.z.manual_home",
}
AXIS_BY_ACTION = {
    "oem.x.prepare": "x",
    "oem.x.diagnostic_home_axis": "x",
    "oem.x.startup_home": "x",
    "oem.x.move_to_origin_home": "x",
    "oem.x.caught_plate_recovery_home": "x",
    "oem.x.set_home": "x",
    "oem.x.set_max_speed": "x",
    "oem.x.set_max_acc": "x",
    "oem.x.restore_original_speed": "x",
    "oem.x.set_stall_guard": "x",
    "oem.x.reconcile_switch_masks": "x",
    "oem.x.observe": "x",
    "oem.x.manual_panel_home": "x",
    "oem.x.move_steps": "x",
    "oem.x.move_absolute": "x",
    "oem.y.stop": "y",
    "oem.z.manual_home": "z",
    "oem.z.prepare": "z",
    "oem.z.diagnostic_home_axis": "z",
    "oem.z.resume_after_abort": "z",
    "oem.z.set_home": "z",
    "oem.z.move_z_home": "z",
    "oem.z.control": "z",
    "oem.z.path_clean_mode": "z",
    "oem.z.move_gz": "z",
    "oem.z.lower_pipette": "z",
    "oem.z.lift_pipette": "z",
    "oem.z.self_test": "z",
    "oem.z.reconcile_switch_masks": "z",
    "oem.z.scriptmove_to": "z",
    "oem.z.home_gz": "z",
    "oem.z.observe": "z",
    "oem.z.clear": "z",
    "oem.z.move_steps": "z",
    "oem.z.move_absolute": "z",
    "oem.x.stop": "x",
    "oem.z.stop": "z",
}


def _now() -> float:
    return time.time()


def _canonical(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
        default=str,
    )


_OPERATOR_PHYSICAL_SCHEMA_SHA256 = "e90c58225c2bb49fadade1fce4f63cdb88ff334ba84b5b5ba74566a92bc29258"
_OPERATOR_RUNTIME_TABLES = {
    "serial206_movement_methods",
    "serial206_movement_commands",
    "serial206_command_resources",
    "serial206_command_dependencies",
}


def _operator_physical_schema_sha256(connection: sqlite3.Connection) -> str:
    tables = sorted(
        str(row[0])
        for row in connection.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
        if str(row[0]).startswith("operator_plane_") or str(row[0]) in _OPERATOR_RUNTIME_TABLES
    )

    def normalize_sql(value: Any) -> str:
        return "".join(str(value or "").upper().split())

    manifest: dict[str, Any] = {}
    for table in tables:
        table_row = connection.execute(
            "SELECT sql FROM sqlite_master WHERE type='table' AND name=?", (table,)
        ).fetchone()
        indexes = []
        for index_row in connection.execute(f'PRAGMA index_list("{table}")').fetchall():
            index_name = str(index_row[1])
            index_sql_row = connection.execute(
                "SELECT sql FROM sqlite_master WHERE type='index' AND name=?", (index_name,)
            ).fetchone()
            indexes.append((
                index_name,
                int(index_row[2]),
                str(index_row[3]),
                int(index_row[4]),
                normalize_sql(index_sql_row[0] if index_sql_row else ""),
                [tuple(row) for row in connection.execute(f'PRAGMA index_xinfo("{index_name}")').fetchall()],
            ))
        manifest[table] = {
            "sql": normalize_sql(table_row[0] if table_row else ""),
            "columns": [tuple(row) for row in connection.execute(f'PRAGMA table_xinfo("{table}")').fetchall()],
            "indexes": sorted(indexes),
            "foreign_keys": [tuple(row) for row in connection.execute(f'PRAGMA foreign_key_list("{table}")').fetchall()],
            "triggers": [
                (str(row[0]), normalize_sql(row[1]))
                for row in connection.execute(
                    "SELECT name,sql FROM sqlite_master WHERE type='trigger' AND tbl_name=? ORDER BY name",
                    (table,),
                ).fetchall()
            ],
        }
    return hashlib.sha256(_canonical(manifest).encode("utf-8")).hexdigest()


def _canonical_request(value: Any) -> str:
    return rfc8785.dumps(value).decode("utf-8")


def _digest(value: Any) -> str:
    return hashlib.sha256(_canonical_request(value).encode("utf-8")).hexdigest()


def _without_idempotency(request: Mapping[str, Any]) -> dict[str, Any]:
    return {str(key): value for key, value in request.items() if str(key) != "idempotency_key"}


def _json_load(value: str | None, default: Any = None) -> Any:
    if value is None:
        return default
    try:
        return json.loads(value)
    except (TypeError, ValueError):
        return default


def _positive_int(value: Any, *, name: str) -> int:
    if type(value) is not int or value <= 0:
        raise HTTPException(status_code=422, detail={"error": "invalid_integer", "field": name})
    return int(value)


def _validate_inputs(action_id: str, inputs: Any) -> dict[str, Any]:
    if type(inputs) is not dict:
        raise HTTPException(status_code=422, detail={"error": "inputs_must_be_object"})
    allowed: dict[str, set[str]] = {
        "oem.x.prepare": set(),
        "oem.x.diagnostic_home_axis": set(),
        "oem.x.startup_home": set(),
        "oem.x.move_to_origin_home": set(),
        "oem.x.caught_plate_recovery_home": set(),
        "oem.x.set_home": {"operator_ack"},
        "oem.x.set_max_speed": {"value"},
        "oem.x.set_max_acc": {"value"},
        "oem.x.restore_original_speed": set(),
        "oem.x.set_stall_guard": {"value"},
        "oem.x.reconcile_switch_masks": {"confirm"},
        "oem.x.observe": {"command_id", "verdict", "physical_motion_observed", "expected_direction_observed", "home_endpoint_observed", "stopped_observed", "note"},
        "oem.z.manual_home": set(),
        "oem.z.prepare": set(),
        "oem.z.clear": set(),
        "oem.z.diagnostic_home_axis": set(),
        "oem.z.resume_after_abort": {"wait_timeout_s"},
        "oem.z.set_home": {"operator_ack"},
        "oem.z.move_z_home": {"wait_timeout_s", "rehome"},
        "oem.z.control": {"operation", "value"},
        "oem.z.path_clean_mode": {"enabled"},
        "oem.z.move_gz": {"gripper_position_steps", "z_position_steps", "wait_timeout_s"},
        "oem.z.lower_pipette": {"location_id", "overpress"},
        "oem.z.lift_pipette": {"location_id"},
        "oem.z.self_test": {"wait_timeout_s"},
        "oem.z.reconcile_switch_masks": {"confirm"},
        "oem.z.scriptmove_to": {"location_id", "column", "row", "positionflag", "run_in_parallel", "wait_timeout_s", "speed", "acc", "root_dir"},
        "oem.z.home_gz": {"delay_s", "wait_timeout_s", "reason"},
        "oem.z.observe": {"command_id", "verdict", "physical_motion_observed", "expected_direction_observed", "home_endpoint_observed", "stopped_observed", "note"},
        "oem.x.manual_panel_home": set(),
        "oem.y.manual_panel_home": set(),

        "oem.z.move_steps": {"steps"},
        "oem.x.move_steps": {"steps"},
        "oem.y.move_steps": {"steps"},
        "oem.z.move_absolute": {"position_steps"},
        "oem.x.move_absolute": {"position_steps"},
        "oem.y.move_absolute": {"target_steps"},
        "oem.xy.move_absolute": {"x", "y"},
        "oem.xy.home": set(),
        "oem.xyz.move_to": {
            "x",
            "y",
            "z",
            "pseudo_z_home",
            "run_in_parallel",
            "gripper_confirmed",
            "tip_loaded",
            "plate_on_gantry",
            "location19_y",
        },
    }
    unknown = sorted(set(inputs) - allowed.get(action_id, set()))
    if unknown:
        raise HTTPException(status_code=422, detail={"error": "unknown_command_inputs", "unknown": unknown})
    result = dict(inputs)
    if action_id in {"oem.x.reconcile_switch_masks", "oem.z.reconcile_switch_masks"}:
        expected = (
            "RECONCILE_X_SWITCH_MASKS"
            if action_id == "oem.x.reconcile_switch_masks"
            else "RECONCILE_Z_SWITCH_MASKS"
        )
        if result.get("confirm") != expected:
            raise HTTPException(
                status_code=422,
                detail={"error": "invalid_switch_mask_confirmation", "required": expected},
            )
        return result
    if action_id in {"oem.x.observe", "oem.z.observe"}:
        for field in ("command_id", "note"):
            value = result.get(field)
            minimum = 1 if field == "command_id" else 3
            maximum = 128 if field == "command_id" else 1000
            if not isinstance(value, str) or not minimum <= len(value.strip()) <= maximum:
                raise HTTPException(status_code=422, detail={"error": "invalid_observation_field", "field": field})
            result[field] = value.strip()
        if result.get("verdict") not in {"pass", "fail"}:
            raise HTTPException(status_code=422, detail={"error": "invalid_observation_field", "field": "verdict"})
        for field in ("physical_motion_observed", "expected_direction_observed", "home_endpoint_observed", "stopped_observed"):
            if type(result.get(field)) is not bool:
                raise HTTPException(status_code=422, detail={"error": "invalid_observation_field", "field": field})
        return result
    if action_id == "oem.z.scriptmove_to":
        location_id = result.get("location_id")
        if not isinstance(location_id, str) or not 1 <= len(location_id.strip()) <= 64:
            raise HTTPException(status_code=422, detail={"error": "invalid_scriptmove_input", "field": "location_id"})
        result["location_id"] = location_id.strip()
        for field in ("column", "row", "positionflag", "speed", "acc"):
            if field in result and result[field] is not None and type(result[field]) is not int:
                raise HTTPException(status_code=422, detail={"error": "invalid_scriptmove_input", "field": field})
        if "run_in_parallel" in result and type(result["run_in_parallel"]) is not bool:
            raise HTTPException(status_code=422, detail={"error": "invalid_scriptmove_input", "field": "run_in_parallel"})
        if "wait_timeout_s" in result and (type(result["wait_timeout_s"]) is not float or not 0.5 <= result["wait_timeout_s"] <= 120.0):
            raise HTTPException(status_code=422, detail={"error": "invalid_scriptmove_input", "field": "wait_timeout_s"})
        if "root_dir" in result and result["root_dir"] is not None and not isinstance(result["root_dir"], str):
            raise HTTPException(status_code=422, detail={"error": "invalid_scriptmove_input", "field": "root_dir"})
        return result
    if action_id == "oem.z.home_gz":
        reason = result.get("reason")
        if not isinstance(reason, str) or not reason.strip():
            raise HTTPException(status_code=422, detail={"error": "invalid_home_gz_input", "field": "reason"})
        result["reason"] = reason.strip()
        if "delay_s" in result and (type(result["delay_s"]) is not int or not 0 <= result["delay_s"] <= 60):
            raise HTTPException(status_code=422, detail={"error": "invalid_home_gz_input", "field": "delay_s"})
        if "wait_timeout_s" in result and (type(result["wait_timeout_s"]) is not float or not 2.0 <= result["wait_timeout_s"] <= 60.0):
            raise HTTPException(status_code=422, detail={"error": "invalid_home_gz_input", "field": "wait_timeout_s"})
        return result
    if action_id in {"oem.x.set_max_speed", "oem.x.set_max_acc", "oem.x.set_stall_guard"}:
        if type(result.get("value")) is not int:
            raise HTTPException(status_code=422, detail={"error": "invalid_profile_value", "field": "value"})
        return result
    if action_id == "oem.z.control":
        if not isinstance(result.get("operation"), str) or not str(result["operation"]).strip():
            raise HTTPException(status_code=422, detail={"error": "invalid_z_control_operation"})
        if "value" in result and type(result["value"]) is not int:
            raise HTTPException(status_code=422, detail={"error": "invalid_profile_value", "field": "value"})
        return result
    if action_id == "oem.xy.move_absolute":
        for key in ("x", "y"):
            if type(result.get(key)) is not int or not -2_147_483_648 <= result[key] <= 2_147_483_647:
                raise HTTPException(status_code=422, detail={"error": "invalid_xy_target", "field": key, "required": "signed int32"})
        return result
    if action_id == "oem.xyz.move_to":
        for key in ("x", "y", "z", "pseudo_z_home", "location19_y"):
            if key not in result and key == "location19_y":
                result[key] = 0
            if type(result.get(key)) is not int:
                raise HTTPException(status_code=422, detail={"error": "invalid_xyz_target", "field": key})
        for key in ("run_in_parallel", "gripper_confirmed", "tip_loaded"):
            if key not in result:
                result[key] = key == "run_in_parallel"
            if type(result.get(key)) is not bool:
                raise HTTPException(status_code=422, detail={"error": "invalid_xyz_option", "field": key})
        if "plate_on_gantry" in result and result["plate_on_gantry"] is not None and type(result["plate_on_gantry"]) is not int:
            raise HTTPException(status_code=422, detail={"error": "invalid_xyz_option", "field": "plate_on_gantry"})
        return result
    if action_id.endswith("move_steps"):
        steps = result.get("steps")
        if type(steps) is not int or not -2_147_483_648 <= steps <= 2_147_483_647:
            raise HTTPException(status_code=422, detail={"error": "invalid_steps", "required": "signed int32"})
    if action_id.endswith("move_absolute"):
        field = "target_steps" if action_id.startswith("oem.y.") else "position_steps"
        target = result.get(field)
        if type(target) is not int or not -2_147_483_648 <= target <= 2_147_483_647:
            raise HTTPException(status_code=422, detail={"error": "invalid_absolute_target", "field": field, "required": "signed int32"})
    return result


def _state_value(state: Mapping[str, Any], *keys: str) -> Any:
    if not isinstance(state, Mapping):
        return None
    wanted = {str(key) for key in keys}
    pending: list[tuple[Any, int]] = [(state, 0)]
    visited = 0
    while pending and visited < 400:
        current, depth = pending.pop(0)
        visited += 1
        if isinstance(current, Mapping):
            for key, value in current.items():
                if str(key) in wanted and type(value) in {int, float, bool, str}:
                    return value
                if depth < 8 and isinstance(value, (Mapping, list, tuple)):
                    pending.append((value, depth + 1))
        elif isinstance(current, (list, tuple)) and depth < 8:
            pending.extend((item, depth + 1) for item in current)
    return None


def _axis_position(state: Mapping[str, Any], axis: str) -> int | None:
    provider = state.get("serial206_initialization_provider") if isinstance(state, Mapping) else None
    if isinstance(provider, Mapping):
        authority_key = {"x": "x_authority", "y": "y_authority", "z": "z_authority"}.get(axis, "z_authority")
        authority = provider.get(authority_key)
        if isinstance(authority, Mapping):
            live = authority.get("live_status")
            terminal = authority.get("terminal_state")
            for row in (live, terminal, authority):
                if isinstance(row, Mapping):
                    for key in ("position_steps", "current_position_steps", "position"):
                        value = row.get(key)
                        if type(value) is int:
                            return value
    domains = state.get("domains") if isinstance(state, Mapping) else None
    if isinstance(domains, Mapping):
        axes = domains.get("axes")
        observation = axes.get("observation") if isinstance(axes, Mapping) else None
        rows = observation.get("rows") if isinstance(observation, Mapping) else None
        row = rows.get(axis) if isinstance(rows, Mapping) else None
        status = row.get("status") if isinstance(row, Mapping) else None
        if isinstance(status, Mapping):
            position = status.get("position")
            if isinstance(position, Mapping):
                position = position.get("value")
            if type(position) is int:
                return position
            if type(position) is float and position.is_integer():
                return int(position)
    return None


def _axis_reference(state: Mapping[str, Any], axis: str) -> tuple[str, int]:
    references = state.get("references") if isinstance(state, Mapping) else None
    rows = references.get("rows") if isinstance(references, Mapping) else None
    row = rows.get(axis) if isinstance(rows, Mapping) else None
    row_state = "unknown"
    row_version = 0
    if isinstance(row, Mapping):
        row_state = str(row.get("state") or "unknown")
        version = row.get("version")
        row_version = int(version) if type(version) is int else 0
    provider = state.get("serial206_initialization_provider") if isinstance(state, Mapping) else None
    authority_key = {"x": "x_authority", "y": "y_authority", "z": "z_authority"}.get(axis, "z_authority")
    authority = provider.get(authority_key) if isinstance(provider, Mapping) else None
    if isinstance(authority, Mapping):
        authority_state = str(authority.get("reference_state") or authority.get("state") or row_state)
        authority_version = max(
            int(authority.get("current_generation") or 0),
            int(authority.get("board_lifecycle_generation") or 0),
        )
        return (authority_state if authority_state != "unknown" else row_state), max(row_version, authority_version)
    return row_state, row_version


def _z_home_valid(state: Mapping[str, Any]) -> bool:
    reference, _ = _axis_reference(state, "z")
    if reference in {"referenced", "home", "valid", "homed"}:
        return True
    provider = state.get("serial206_initialization_provider") if isinstance(state, Mapping) else None
    authority = provider.get("z_authority") if isinstance(provider, Mapping) else None
    if isinstance(authority, Mapping):
        return str(authority.get("state") or "") == "referenced_ready" and str(authority.get("reference_state") or "") == "referenced"
    return False


def _z_pseudo_home(state: Mapping[str, Any]) -> int:
    for key in ("psudo_z_home_steps", "pseudo_z_home_steps", "pseudo_z_home", "pseudo_home_steps"):
        value = _state_value(state, key)
        if type(value) is int and 0 <= value <= 160000:
            return value
    return 65000


def _effective_inputs(action_id: str, inputs: Mapping[str, Any], state: Mapping[str, Any]) -> dict[str, Any]:
    effective = dict(inputs)
    if action_id == "oem.x.move_absolute":
        effective["position_steps"] = max(60, int(inputs["position_steps"]))
    elif action_id == "oem.z.move_absolute":
        effective["position_steps"] = max(_z_pseudo_home(state), int(inputs["position_steps"]))
    return effective


def _active_board_epochs(state: Mapping[str, Any], action_id: str) -> dict[str, int]:
    provider = state.get("serial206_initialization_provider") if isinstance(state, Mapping) else None
    axis = AXIS_BY_ACTION.get(action_id)
    if (
        action_id.startswith(("oem.xy.", "oem.xyz."))
        or action_id == "oem.z.scriptmove_to"
    ) and isinstance(provider, Mapping):
        epochs: dict[str, int] = {}
        x_authority = provider.get("x_authority")
        if isinstance(x_authority, Mapping):
            x_epoch = x_authority.get("active_board_epoch", x_authority.get("board_lifecycle_generation"))
            if type(x_epoch) is int and x_epoch >= 0:
                epochs["5"] = x_epoch
        y_authority = provider.get("y_authority")
        for candidate in (
            provider.get("board4_authority"),
            y_authority.get("board_authority") if isinstance(y_authority, Mapping) else None,
            y_authority,
            provider,
        ):
            if not isinstance(candidate, Mapping):
                continue
            nested_board = candidate.get("board_authority", candidate.get("board"))
            board = nested_board if isinstance(nested_board, Mapping) else candidate
            board4_epoch = board.get("active_board_epoch")
            if type(board4_epoch) is int and board4_epoch >= 0:
                epochs["4"] = board4_epoch
                break
        return epochs
    if axis == "x" and isinstance(provider, Mapping):
        authority = provider.get("x_authority")
        if isinstance(authority, Mapping):
            value = authority.get("active_board_epoch", authority.get("board_lifecycle_generation"))
            if type(value) is int and value >= 0:
                return {"5": value}
    candidates: list[Any] = []
    if isinstance(provider, Mapping):
        y_authority = provider.get("y_authority")
        candidates.extend((
            provider.get("board4_authority"),
            y_authority.get("board_authority") if isinstance(y_authority, Mapping) else None,
            y_authority,
            provider,
        ))
    for candidate in candidates:
        if not isinstance(candidate, Mapping):
            continue
        nested = candidate.get("board_authority", candidate.get("board"))
        board = nested if isinstance(nested, Mapping) else candidate
        value = board.get("active_board_epoch")
        if type(value) is int and value >= 0:
            return {"4": value}
    return {}


def _axis_limit(state: Mapping[str, Any], axis: str) -> int | None:
    provider = state.get("serial206_initialization_provider") if isinstance(state, Mapping) else None
    if isinstance(provider, Mapping):
        authority_key = {"x": "x_authority", "y": "y_authority", "z": "z_authority"}.get(axis, "z_authority")
        authority = provider.get(authority_key)
        if isinstance(authority, Mapping):
            for key in ("source_max_steps", "axis_max_steps", "max_steps"):
                value = authority.get(key)
                if type(value) is int:
                    return value
    domains = state.get("domains") if isinstance(state, Mapping) else None
    axes = domains.get("axes") if isinstance(domains, Mapping) else None
    observation = axes.get("observation") if isinstance(axes, Mapping) else None
    rows = observation.get("rows") if isinstance(observation, Mapping) else None
    row = rows.get(axis) if isinstance(rows, Mapping) else None
    if isinstance(row, Mapping):
        preset = row.get("preset")
        if isinstance(preset, Mapping):
            for key in ("axis_max_steps", "max_steps"):
                value = preset.get(key)
                if type(value) is int:
                    return value
    return None


def _home_completion_proven(response: Any) -> bool:
    if not isinstance(response, (Mapping, list, tuple)):
        return False
    pending: list[Any] = [response]
    visited = 0
    while pending and visited < 600:
        current = pending.pop(0)
        visited += 1
        if isinstance(current, Mapping):
            if current.get("source_noop") is True and current.get("command_issued") is False:
                return True
            if current.get("home_predicate_active") is True and current.get("set_home_readback_zero") is True:
                return True
            if current.get("home_predicate_confirmed") is True and current.get("controller_terminal_state_verified") is True:
                return True
            if current.get("controller_home_proof_verified") is True:
                return True
            if current.get("home_after_sethome_active") is True and current.get("set_home_acknowledged") is True:
                return True
            if current.get("z_state") == "referenced_ready" and current.get("reference_state") == "referenced":
                return True
            pending.extend(value for value in current.values() if isinstance(value, (Mapping, list, tuple)))
        elif isinstance(current, (list, tuple)):
            pending.extend(value for value in current if isinstance(value, (Mapping, list, tuple)))
    return False


def _addressed_event_proven(response: Any) -> bool:
    if _find_event(response, code=128, name="TARGET_POSITION_REACHED"):
        return True
    if not isinstance(response, (Mapping, list, tuple)):
        return False
    pending: list[Any] = [response]
    visited = 0
    while pending and visited < 600:
        current = pending.pop(0)
        visited += 1
        if isinstance(current, Mapping):
            if any(current.get(key) is True for key in ("addressed_event_128", "target_event_128_verified", "target_event_128_observed")):
                return True
            if current.get("completion_class") == "event_128":
                return True
            pending.extend(value for value in current.values() if isinstance(value, (Mapping, list, tuple)))
        elif isinstance(current, (list, tuple)):
            pending.extend(value for value in current if isinstance(value, (Mapping, list, tuple)))
    return False


def _y_absolute_terminal_disposition(
    *,
    completion_class: Any,
    event_128: bool,
    target_steps: Any,
    observed_position_steps: Any,
    terminal_speed_zero: bool,
) -> str:
    target_equal = type(target_steps) is int and type(observed_position_steps) is int and observed_position_steps == target_steps
    if target_equal and terminal_speed_zero and (
        event_128 or completion_class == "oem_timeout_target_equal"
    ):
        return "completed"
    return "failed"


def _position_readback_proven(response: Any) -> bool:
    if not isinstance(response, (Mapping, list, tuple)):
        return False
    pending: list[Any] = [response]
    visited = 0
    readback_keys = {
        "terminal_position",
        "position_after",
        "reached_position_after",
        "position_readback",
        "terminal_position_readback",
    }
    while pending and visited < 600:
        current = pending.pop(0)
        visited += 1
        if isinstance(current, Mapping):
            if any(current.get(key) is True for key in ("position_readback_verified", "terminal_position_readback_verified", "target_position_readback_verified")):
                return True
            for key, value in current.items():
                if str(key).lower() not in readback_keys:
                    if isinstance(value, (Mapping, list, tuple)):
                        pending.append(value)
                    continue
                if isinstance(value, Mapping):
                    ok = value.get("ok") is True
                    position = value.get("position")
                    if position is None:
                        position = value.get("value")
                    if ok and type(position) is int:
                        return True
                elif type(value) is int:
                    return True
        elif isinstance(current, (list, tuple)):
            pending.extend(value for value in current if isinstance(value, (Mapping, list, tuple)))
    return False


def _terminal_position_steps(response: Any) -> int | None:
    if not isinstance(response, Mapping):
        return None
    for key in ("terminal_position", "position_after", "terminal_position_readback"):
        value = response.get(key)
        if isinstance(value, Mapping):
            position = value.get("position", value.get("value"))
            if value.get("ok") is True and type(position) is int:
                return int(position)
        elif type(value) is int:
            return int(value)
    nested = response.get("response")
    return _terminal_position_steps(nested) if isinstance(nested, Mapping) else None


def _source_noop(action_id: str, requested: Mapping[str, Any], effective: Mapping[str, Any], state: Mapping[str, Any]) -> tuple[bool, str | None]:
    axis = AXIS_BY_ACTION.get(action_id)
    if axis is None:
        return False, None
    current = _axis_position(state, axis)
    if action_id.endswith("move_steps") and not action_id.startswith("oem.y.") and int(requested.get("steps", 1)) == 0:
        return True, "zero_relative_steps"
    if action_id.endswith("move_absolute") and not action_id.startswith("oem.y."):
        target = effective.get("position_steps")
        if type(target) is int and current is not None and current == target:
            return True, "same_effective_absolute_target"
    if action_id == "oem.z.manual_home" and current == 0 and bool(_state_value(state, "MotorHome", "motor_home", "home_latched")):
        return True, "already_home"
    if action_id == "oem.x.manual_panel_home" and current == 0 and bool(_state_value(state, "MotorHome", "motor_home", "home_latched")):
        return True, "already_home"
    if action_id.endswith("move_absolute") and not action_id.startswith("oem.y.") and current is not None:
        target = effective.get("position_steps")
        if type(target) is int and target > current:
            maximum = _axis_limit(state, axis)
            if type(maximum) is int and abs(maximum - current) < 10:
                return True, "source_high_limit_suppression"
    return False, None


def _find_event(value: Any, *, code: int | None = None, name: str | None = None) -> bool:
    wanted_name = name.upper() if name else None
    pending: list[Any] = [value]
    visited = 0
    while pending and visited < 500:
        item = pending.pop(0)
        visited += 1
        if isinstance(item, Mapping):
            for key, child in item.items():
                key_text = str(key).lower()
                if code is not None and key_text in {"code", "event", "event_code", "status_code", "status"} and child == code:
                    return True
                if wanted_name and wanted_name in str(child).upper():
                    return True
                if isinstance(child, (Mapping, list, tuple)):
                    pending.append(child)
        elif isinstance(item, (list, tuple)):
            if code is not None and code in item:
                return True
            pending.extend(item)
        elif wanted_name and wanted_name in str(item).upper():
            return True
    return False


class CommandRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    schema_version: str = Field(pattern=r"^bioxp\.operator_command_request\.v1$")
    idempotency_key: str = Field(min_length=8, max_length=128, pattern=r"^[A-Za-z0-9._:-]+$")
    expected_ownership_generation: StrictInt = Field(ge=0)
    action_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_.-]{0,127}$")
    inputs: dict[str, Any] = Field(default_factory=dict, max_length=16)


class OperatorActionRequestV2(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    schema_version: str = Field(pattern=r"^bioxp\.operator_action_request\.v2$")
    idempotency_key: str = Field(min_length=1, max_length=128)
    expected_ownership_generation: StrictInt = Field(ge=0)
    expected_board_epoch_by_board: dict[str, StrictInt]
    inputs: dict[str, Any]

    @field_validator("idempotency_key")
    @classmethod
    def validate_key_bytes(cls, value: str) -> str:
        if not 1 <= len(value.encode("utf-8")) <= 128:
            raise ValueError("idempotency_key must be 1..128 bytes")
        return value

    @field_validator("expected_board_epoch_by_board")
    @classmethod
    def validate_board_epoch_keys(cls, value: dict[str, StrictInt]) -> dict[str, StrictInt]:
        if any(not key.isdecimal() or str(int(key)) != key or int(epoch) < 0 for key, epoch in value.items()):
            raise ValueError("board epoch keys must be canonical nonnegative decimal board IDs")
        return value


class OperatorInterruptRequestV1(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    schema_version: str = Field(pattern=r"^bioxp\.operator_interrupt_request\.v1$")
    idempotency_key: str = Field(min_length=1, max_length=128)
    reason: str = Field(min_length=1, max_length=500)
    observed_ownership_generation: StrictInt | None
    observed_board_epoch_by_board: dict[str, StrictInt]

    @field_validator("idempotency_key")
    @classmethod
    def validate_idempotency_key_bytes(cls, value: str) -> str:
        if len(value.encode("utf-8")) > 128:
            raise ValueError("idempotency_key must be at most 128 bytes")
        return value

    @field_validator("reason")
    @classmethod
    def validate_reason_bytes(cls, value: str) -> str:
        if not 1 <= len(value.encode("utf-8")) <= 500:
            raise ValueError("reason must be 1..500 bytes")
        return value

    @field_validator("observed_board_epoch_by_board")
    @classmethod
    def validate_observed_board_epoch_keys(cls, value: dict[str, StrictInt]) -> dict[str, StrictInt]:
        if any(not key.isdecimal() or str(int(key)) != key or int(epoch) < 0 for key, epoch in value.items()):
            raise ValueError("observed board epoch keys must be canonical nonnegative decimal board IDs")
        return value


class MethodStep(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    action_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_.-]{0,127}$")
    inputs: dict[str, Any] = Field(default_factory=dict, max_length=16)
    repeat: StrictInt = Field(default=1, ge=1, le=10000)


class MethodRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    schema_version: str = Field(pattern=r"^bioxp\.operator_method_request\.v1$")
    name: str = Field(min_length=1, max_length=200)
    idempotency_key: str = Field(min_length=8, max_length=128, pattern=r"^[A-Za-z0-9._:-]+$")
    expected_ownership_generation: StrictInt = Field(ge=0)
    failure_policy: str = Field(pattern=r"^fail_fast$")
    steps: list[MethodStep] = Field(min_length=1, max_length=10000)
    metadata: dict[str, str | int | bool] = Field(default_factory=dict, max_length=32)


class OperatorMethodRequestV1(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    schema_version: str = Field(pattern=r"^bioxp\.operator_method_request\.v1$")
    idempotency_key: str = Field(min_length=1, max_length=128)
    method_action_id: str = Field(pattern=r"^oem\.(?:xy\.(?:move_absolute|home)|xyz\.move_to)$")
    expected_ownership_generation: StrictInt = Field(ge=0)
    expected_board_epoch_by_board: dict[str, StrictInt]
    inputs: dict[str, Any]

    @field_validator("idempotency_key")
    @classmethod
    def validate_method_key_bytes(cls, value: str) -> str:
        if not 1 <= len(value.encode("utf-8")) <= 128:
            raise ValueError("idempotency_key must be 1..128 bytes")
        return value

    @field_validator("expected_board_epoch_by_board")
    @classmethod
    def validate_method_board_epochs(cls, value: dict[str, StrictInt]) -> dict[str, StrictInt]:
        if any(not key.isdecimal() or str(int(key)) != key or int(epoch) < 0 for key, epoch in value.items()):
            raise ValueError("board epoch keys must be canonical nonnegative decimal board IDs")
        return value


class MutationRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    schema_version: str = Field(pattern=r"^bioxp\.operator_mutation\.v1$")
    idempotency_key: str = Field(min_length=8, max_length=128, pattern=r"^[A-Za-z0-9._:-]+$")
    expected_version: StrictInt = Field(ge=1)
    expected_recovery_epoch: StrictInt = Field(ge=0)
    expected_global_safety_epoch: StrictInt = Field(ge=0)
    expected_axis_safety_epoch: StrictInt = Field(ge=0)
    acknowledge_command_ids: list[str] = Field(default_factory=list, max_length=10000)


class RecoveryResolveRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    schema_version: str = Field(pattern=r"^bioxp\.operator_recovery_resolve\.v1$")
    idempotency_key: str = Field(min_length=8, max_length=128, pattern=r"^[A-Za-z0-9._:-]+$")
    expected_version: StrictInt = Field(ge=1)
    expected_safety_epoch: StrictInt = Field(ge=0)
    acknowledge_command_ids: list[str] = Field(max_length=10000)
    operation: str = Field(pattern=r"^(resume_undispatched|cancel_pending)$")


class OperatorCommandStore:
    """SQLite authority for command order and lifecycle projections."""

    def __init__(self, root: str | Path | None = None) -> None:
        self.root = runtime_state_root(root)
        self.path = self.root / "bioxp_runtime.db"
        self._interrupt_spool_path = self.root / "operator_interrupt_reconciliation.db"
        self._y_interrupt_fallback_path = self.root / "operator_y_interrupt_fallback.v2.jsonl"
        self._y_interrupt_fallback_lock_path = self.root / "operator_y_interrupt_fallback.v2.lock"
        self._lock = runtime_write_coordinator(self.root).lock
        self._priority_fence = threading.Event()
        self._axis_priority_fences = {axis: threading.Event() for axis in ("x", "y", "z")}
        self._interrupt_lock = threading.Lock()
        self._pending_interrupt_lock = threading.RLock()
        self._pending_interrupt_reconciliations: list[dict[str, Any]] = []
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._worker_lock = threading.Lock()
        self._workers: set[threading.Thread] = set()
        self._authority_write_depth = 0
        self._interrupt_spool_write_depth = 0
        self.owner_id = uuid.uuid4().hex
        self._owner_acquired = False
        self._configure_interrupt_spool()
        self.connection = sqlite3.connect(self.path, timeout=2.0, isolation_level=None, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        self._configure()
        verify_canonical_runtime_database(self.connection)
        self._authority_schema_version = int(self.connection.execute("PRAGMA schema_version").fetchone()[0])
        self._import_interrupt_fallback()
        self._owner_acquired = self._acquire_owner()
        if self._owner_acquired:
            self._startup_recover()

    def append_y_interrupt_fallback(self, receipt: Mapping[str, Any], *, reason: str) -> dict[str, Any]:
        """Compatibility wrapper for the original Y-only caller."""
        return self.append_interrupt_fallback(receipt, reason=reason)

    def _import_interrupt_fallback(self) -> None:
        lock_descriptor = os.open(self._y_interrupt_fallback_lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            os.fchmod(lock_descriptor, 0o600)
            fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
            if self._y_interrupt_fallback_path.exists():
                pending = self.root / f"operator_y_interrupt_fallback.v2.pending.{time.time_ns()}.jsonl"
                os.replace(self._y_interrupt_fallback_path, pending)
                directory = os.open(self.root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
            pending_paths = sorted(self.root.glob("operator_y_interrupt_fallback.v2.pending.*.jsonl"))
        finally:
            fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
            os.close(lock_descriptor)
        for pending in pending_paths:
            rows = [json.loads(line) for line in pending.read_text(encoding="utf-8").splitlines() if line.strip()]
            with self._transaction() as conn:
                for wrapper in rows:
                    stream = str(wrapper.get("stream") or "") if isinstance(wrapper, Mapping) else ""
                    if stream not in {"x", "y", "z", "aggregate"} or not isinstance(wrapper.get("receipt"), Mapping):
                        raise RuntimeError("interrupt fallback contains an invalid command-plane row")
                    receipt = dict(wrapper["receipt"])
                    source_wrapper_json = _canonical(wrapper)
                    attempt_id = str(receipt.get("interrupt_attempt_id") or "")
                    if not attempt_id:
                        raise RuntimeError("interrupt fallback is missing interrupt_attempt_id")
                    key = f"interrupt-attempt:{attempt_id}"
                    fingerprint = _digest(receipt)
                    record_sha256 = hashlib.sha256(source_wrapper_json.encode("utf-8")).hexdigest()
                    conn.execute(
                        "INSERT OR IGNORE INTO operator_plane_interrupt_history(record_sha256,stream,interrupt_attempt_id,receipt_json,source_wrapper_json,imported_at) VALUES(?,?,?,?,?,?)",
                        (record_sha256, "y", attempt_id, _canonical(receipt), source_wrapper_json, _now()),
                    )
                    inserted = conn.execute(
                        "INSERT OR IGNORE INTO operator_plane_idempotency(operation_kind,idempotency_key,fingerprint,response_json,created_at) VALUES('interrupt',?,?,?,?)",
                        (key, fingerprint, _canonical(receipt), _now()),
                    ).rowcount
                    if inserted == 1:
                        self._insert_transition(conn, event_kind=f"{stream}_interrupt_fallback_imported", state="interrupted", payload={"interrupt_attempt_id": attempt_id, "persistence_state": "fsync_fallback"})
                    else:
                        existing = conn.execute("SELECT fingerprint FROM operator_plane_idempotency WHERE operation_kind='interrupt' AND idempotency_key=?", (key,)).fetchone()
                        if existing is None or str(existing["fingerprint"]) != fingerprint:
                            raise RuntimeError("interrupt fallback identity conflicts with canonical command-plane persistence")
            archive = self.root / pending.name.replace(".pending.", ".imported.")
            os.replace(pending, archive)
            directory = os.open(self.root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory)
            finally:
                os.close(directory)

    def _interrupt_spool_connection(self) -> sqlite3.Connection:
        connection = sqlite3.connect(
            self._interrupt_spool_path,
            timeout=2.0,
            isolation_level=None,
        )
        connection.row_factory = sqlite3.Row
        connection.create_function(
            "sha256_utf8",
            1,
            lambda value: hashlib.sha256(str(value).encode("utf-8")).hexdigest(),
            deterministic=True,
        )
        connection.create_function(
            "canonical_json",
            1,
            lambda value: _canonical(json.loads(str(value))),
            deterministic=True,
        )
        connection.create_function(
            "interrupt_spool_write_allowed",
            0,
            lambda: 1 if self._interrupt_spool_write_depth > 0 else 0,
        )
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA synchronous=FULL")
        connection.execute("PRAGMA busy_timeout=2000")
        return connection

    def _configure_interrupt_spool(self) -> None:
        connection = self._interrupt_spool_connection()
        try:
            table_row = connection.execute(
                "SELECT sql FROM sqlite_master WHERE type='table' AND name='interrupt_reconciliation_events'"
            ).fetchone()
            if table_row is not None and "'delivered'" not in str(table_row["sql"]):
                legacy_rows = connection.execute(
                    "SELECT sequence,interrupt_attempt_id,phase,payload_json,content_sha256,created_at FROM interrupt_reconciliation_events ORDER BY sequence"
                ).fetchall()
                for legacy_row in legacy_rows:
                    encoded = str(legacy_row["payload_json"])
                    try:
                        canonical = _canonical(json.loads(encoded))
                    except Exception as exc:
                        raise RuntimeError("legacy interrupt reconciliation row is invalid JSON") from exc
                    if (
                        encoded != canonical
                        or str(legacy_row["content_sha256"]) != hashlib.sha256(encoded.encode("utf-8")).hexdigest()
                        or str(json.loads(encoded).get("interrupt_attempt_id") or "") != str(legacy_row["interrupt_attempt_id"])
                        or str(legacy_row["phase"]) not in {"pending", "reconciled"}
                    ):
                        raise RuntimeError("legacy interrupt reconciliation row is incoherent")
                connection.executescript(
                    """
                    BEGIN IMMEDIATE;
                    DROP TRIGGER IF EXISTS interrupt_reconciliation_events_coherence_v1;
                    DROP TRIGGER IF EXISTS interrupt_reconciliation_events_no_update_v1;
                    DROP TRIGGER IF EXISTS interrupt_reconciliation_events_no_delete_v1;
                    DROP TRIGGER IF EXISTS interrupt_reconciliation_events_authorized_lineage_v2;
                    ALTER TABLE interrupt_reconciliation_events RENAME TO interrupt_reconciliation_events_legacy_v1;
                    CREATE TABLE interrupt_reconciliation_events (
                        sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                        interrupt_attempt_id TEXT NOT NULL,
                        phase TEXT NOT NULL CHECK(phase IN ('pending','delivered','reconciled')),
                        payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
                        content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
                        created_at REAL NOT NULL,
                        UNIQUE(interrupt_attempt_id,phase)
                    );
                    INSERT INTO interrupt_reconciliation_events(
                        sequence,interrupt_attempt_id,phase,payload_json,content_sha256,created_at
                    )
                    SELECT sequence,interrupt_attempt_id,phase,payload_json,content_sha256,created_at
                    FROM interrupt_reconciliation_events_legacy_v1;
                    DROP TABLE interrupt_reconciliation_events_legacy_v1;
                    COMMIT;
                    """
                )
            for trigger_name in (
                "interrupt_reconciliation_events_coherence_v1",
                "interrupt_reconciliation_events_no_update_v1",
                "interrupt_reconciliation_events_no_delete_v1",
                "interrupt_reconciliation_events_authorized_lineage_v2",
            ):
                connection.execute(f'DROP TRIGGER IF EXISTS "{trigger_name}"')
            connection.executescript(
                """
                BEGIN IMMEDIATE;
                CREATE TABLE IF NOT EXISTS interrupt_reconciliation_events (
                    sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                    interrupt_attempt_id TEXT NOT NULL,
                    phase TEXT NOT NULL CHECK(phase IN ('pending','delivered','reconciled')),
                    payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
                    content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
                    created_at REAL NOT NULL,
                    UNIQUE(interrupt_attempt_id,phase)
                );
                CREATE TRIGGER IF NOT EXISTS interrupt_reconciliation_events_coherence_v1
                BEFORE INSERT ON interrupt_reconciliation_events
                WHEN NEW.content_sha256 <> sha256_utf8(NEW.payload_json)
                  OR NEW.payload_json <> canonical_json(NEW.payload_json)
                  OR json_extract(NEW.payload_json,'$.interrupt_attempt_id') IS NOT NEW.interrupt_attempt_id
                BEGIN SELECT RAISE(ABORT, 'interrupt reconciliation event is incoherent'); END;
                CREATE TRIGGER IF NOT EXISTS interrupt_reconciliation_events_no_update_v1
                BEFORE UPDATE ON interrupt_reconciliation_events
                BEGIN SELECT RAISE(ABORT, 'interrupt reconciliation events are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS interrupt_reconciliation_events_no_delete_v1
                BEFORE DELETE ON interrupt_reconciliation_events
                BEGIN SELECT RAISE(ABORT, 'interrupt reconciliation events are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS interrupt_reconciliation_events_authorized_lineage_v2
                BEFORE INSERT ON interrupt_reconciliation_events
                WHEN interrupt_spool_write_allowed()<>1
                  OR (NEW.phase IN ('delivered','reconciled') AND NOT EXISTS(
                    SELECT 1 FROM interrupt_reconciliation_events AS pending
                    WHERE pending.interrupt_attempt_id=NEW.interrupt_attempt_id
                      AND pending.phase='pending'
                  ))
                BEGIN SELECT RAISE(ABORT, 'interrupt reconciliation writer or lineage is not authoritative'); END;
                COMMIT;
                """
            )
            expected_columns = (
                ("sequence", "INTEGER", 0, 1),
                ("interrupt_attempt_id", "TEXT", 1, 0),
                ("phase", "TEXT", 1, 0),
                ("payload_json", "TEXT", 1, 0),
                ("content_sha256", "TEXT", 1, 0),
                ("created_at", "REAL", 1, 0),
            )
            actual_columns = tuple(
                (str(row[1]), str(row[2]).upper(), int(row[3]), int(row[5]))
                for row in connection.execute("PRAGMA table_info(interrupt_reconciliation_events)").fetchall()
            )
            if actual_columns != expected_columns:
                raise RuntimeError("interrupt reconciliation schema columns are not exact")
            table_sql_row = connection.execute(
                "SELECT sql FROM sqlite_master WHERE type='table' AND name='interrupt_reconciliation_events'"
            ).fetchone()
            actual_table_sql = "".join(str(table_sql_row[0] if table_sql_row else "").upper().split())
            expected_table_sql = "".join(
                """CREATE TABLE interrupt_reconciliation_events (
                    sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                    interrupt_attempt_id TEXT NOT NULL,
                    phase TEXT NOT NULL CHECK(phase IN ('pending','delivered','reconciled')),
                    payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
                    content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
                    created_at REAL NOT NULL,
                    UNIQUE(interrupt_attempt_id,phase)
                )""".upper().split()
            )
            if actual_table_sql != expected_table_sql:
                raise RuntimeError("interrupt reconciliation table definition is not exact")
            index_shapes = []
            for index_row in connection.execute("PRAGMA index_list(interrupt_reconciliation_events)").fetchall():
                columns = tuple(
                    str(row[2])
                    for row in connection.execute(f'PRAGMA index_info("{index_row[1]}")').fetchall()
                )
                index_shapes.append((int(index_row[2]), str(index_row[3]), int(index_row[4]), columns))
            if index_shapes != [(1, "u", 0, ("interrupt_attempt_id", "phase"))]:
                raise RuntimeError("interrupt reconciliation schema index set is not exact")
            required_triggers = {
                "interrupt_reconciliation_events_coherence_v1",
                "interrupt_reconciliation_events_no_update_v1",
                "interrupt_reconciliation_events_no_delete_v1",
                "interrupt_reconciliation_events_authorized_lineage_v2",
            }
            installed_triggers = {
                str(row[0])
                for row in connection.execute(
                    "SELECT name FROM sqlite_master WHERE type='trigger' AND tbl_name='interrupt_reconciliation_events'"
                ).fetchall()
            }
            if installed_triggers != required_triggers:
                raise RuntimeError("interrupt reconciliation schema trigger attestation failed")
            rows = connection.execute(
                "SELECT interrupt_attempt_id,phase,payload_json,content_sha256 FROM interrupt_reconciliation_events ORDER BY sequence"
            ).fetchall()
            phases_by_attempt: dict[str, set[str]] = {}
            for row in rows:
                attempt_id = str(row["interrupt_attempt_id"])
                phase = str(row["phase"])
                encoded = str(row["payload_json"])
                try:
                    payload = json.loads(encoded)
                except Exception as exc:
                    raise RuntimeError("interrupt reconciliation persisted row is invalid JSON") from exc
                if (
                    encoded != _canonical(payload)
                    or str(row["content_sha256"]) != hashlib.sha256(encoded.encode("utf-8")).hexdigest()
                    or str(payload.get("interrupt_attempt_id") or "") != attempt_id
                ):
                    raise RuntimeError("interrupt reconciliation persisted row is incoherent")
                phases_by_attempt.setdefault(attempt_id, set()).add(phase)
            if any(
                ("delivered" in phases or "reconciled" in phases) and "pending" not in phases
                for phases in phases_by_attempt.values()
            ):
                raise RuntimeError("interrupt reconciliation persisted lineage is incoherent")
        finally:
            connection.close()

    def _append_interrupt_spool_event(self, *, phase: str, payload: Mapping[str, Any]) -> None:
        encoded = _canonical(dict(payload))
        attempt_id = str(payload.get("interrupt_attempt_id") or "")
        digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
        connection = self._interrupt_spool_connection()
        try:
            self._interrupt_spool_write_depth += 1
            connection.execute("BEGIN IMMEDIATE")
            connection.execute(
                "INSERT OR IGNORE INTO interrupt_reconciliation_events(interrupt_attempt_id,phase,payload_json,content_sha256,created_at) VALUES(?,?,?,?,?)",
                (attempt_id, str(phase), encoded, digest, _now()),
            )
            existing = connection.execute(
                "SELECT payload_json,content_sha256 FROM interrupt_reconciliation_events WHERE interrupt_attempt_id=? AND phase=?",
                (attempt_id, str(phase)),
            ).fetchone()
            if existing is None or str(existing["payload_json"]) != encoded or str(existing["content_sha256"]) != digest:
                raise RuntimeError("interrupt reconciliation spool identity conflicts with immutable evidence")
            connection.execute("COMMIT")
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise
        finally:
            self._interrupt_spool_write_depth -= 1
            connection.close()

    def _pending_interrupt_spool_rows(self) -> list[dict[str, Any]]:
        connection = self._interrupt_spool_connection()
        try:
            rows = connection.execute(
                """
                SELECT COALESCE(
                    (SELECT delivered.payload_json
                     FROM interrupt_reconciliation_events AS delivered
                     WHERE delivered.interrupt_attempt_id=pending.interrupt_attempt_id
                       AND delivered.phase='delivered'),
                    pending.payload_json
                ) AS payload_json
                FROM interrupt_reconciliation_events AS pending
                WHERE pending.phase='pending'
                  AND NOT EXISTS(
                    SELECT 1 FROM interrupt_reconciliation_events AS reconciled
                    WHERE reconciled.interrupt_attempt_id=pending.interrupt_attempt_id
                      AND reconciled.phase='reconciled'
                  )
                ORDER BY pending.sequence
                """
            ).fetchall()
            return [json.loads(str(row["payload_json"])) for row in rows]
        finally:
            connection.close()

    def _configure(self) -> None:
        def canonical_json_text(value: Any) -> str | None:
            try:
                return _canonical(json.loads(str(value)))
            except (TypeError, ValueError, json.JSONDecodeError):
                return None

        self.connection.create_function(
            "sha256_utf8",
            1,
            lambda value: hashlib.sha256(str(value).encode("utf-8")).hexdigest(),
            deterministic=True,
        )
        self.connection.create_function("canonical_json", 1, canonical_json_text, deterministic=True)
        self.connection.create_function(
            "authority_write_allowed", 0, lambda: 1 if self._authority_write_depth > 0 else 0
        )
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=FULL")
        self.connection.execute("PRAGMA foreign_keys=ON")
        self.connection.execute("PRAGMA busy_timeout=2000")
        self.connection.execute("PRAGMA wal_autocheckpoint=256")

    def _schema(self) -> None:
        with self._lock:
            required_trigger_names = (
                "operator_plane_transitions_no_delete",
                "operator_plane_transitions_no_update",
                "operator_plane_evidence_no_delete",
                "operator_plane_evidence_no_update",
                "operator_plane_interrupt_evidence_no_delete",
                "operator_plane_interrupt_evidence_no_update",
                "operator_plane_commands_no_terminal_delete",
                "operator_plane_methods_no_terminal_delete",
                "operator_plane_idempotency_no_update_v2",
                "operator_plane_idempotency_no_delete_v2",
                "operator_plane_commands_identity_immutable_v2",
                "operator_plane_commands_terminal_no_update_v2",
                "operator_plane_methods_identity_immutable_v2",
                "operator_plane_methods_terminal_no_update_v2",
                "serial206_movement_commands_identity_immutable_v2",
                "serial206_movement_commands_terminal_no_update_v2",
                "serial206_movement_commands_no_terminal_delete_v2",
                "serial206_movement_methods_identity_immutable_v2",
                "serial206_movement_methods_terminal_no_update_v2",
                "serial206_movement_methods_no_terminal_delete_v2",
                "serial206_command_resources_no_update_v2",
                "serial206_command_resources_no_delete_v2",
                "serial206_command_dependencies_no_update_v2",
                "serial206_command_dependencies_no_delete_v2",
                "operator_plane_interrupt_attempts_no_update_v2",
                "operator_plane_interrupt_attempts_no_delete_v2",
                "serial206_command_resources_sealed_insert_v3",
                "serial206_command_dependencies_sealed_insert_v3",
                "operator_plane_evidence_coherence_insert_v1",
                "operator_plane_interrupt_evidence_coherence_insert_v1",
                "operator_plane_interrupt_attempts_lineage_insert_v1",
                "operator_plane_commands_recoverable_terminal_no_update_v3",
                "serial206_movement_commands_ambiguous_no_update_v3",
                "operator_plane_recovery_acknowledgements_no_update_v1",
                "operator_plane_recovery_acknowledgements_no_delete_v1",
                "operator_plane_recovery_acknowledgements_authorized_insert_v2",
                "operator_plane_commands_authorized_insert_v4",
                "operator_plane_commands_authorized_update_v4",
                "serial206_movement_commands_authorized_insert_v4",
                "serial206_movement_commands_authorized_update_v4",
                "serial206_movement_commands_authorized_delete_v4",
                "serial206_command_resources_authorized_insert_v4",
                "serial206_command_dependencies_authorized_insert_v4",
                "operator_plane_methods_authorized_insert_v4",
                "operator_plane_methods_authorized_update_v4",
                "operator_plane_methods_authorized_delete_v4",
                "serial206_movement_methods_authorized_insert_v4",
                "serial206_movement_methods_authorized_update_v4",
                "serial206_movement_methods_authorized_delete_v4",
                "operator_plane_idempotency_authorized_insert_v4",
                "operator_plane_commands_authorized_delete_v4",
                "operator_plane_interrupt_history_authorized_insert_v4",
                "operator_plane_interrupt_history_no_update_v4",
                "operator_plane_interrupt_history_no_delete_v4",
                "operator_plane_interrupt_history_coherence_insert_v4",
                "operator_plane_transitions_authorized_coherent_insert_v4",
            )
            for trigger_name in required_trigger_names:
                self.connection.execute(f'DROP TRIGGER IF EXISTS "{trigger_name}"')
            self.connection.executescript(
                """
                BEGIN IMMEDIATE;
                CREATE TABLE IF NOT EXISTS operator_plane_metadata (
                    key TEXT PRIMARY KEY, value TEXT NOT NULL, updated_at REAL NOT NULL
                ) WITHOUT ROWID;
                CREATE TABLE IF NOT EXISTS operator_plane_idempotency (
                    operation_kind TEXT NOT NULL,
                    idempotency_key TEXT NOT NULL,
                    fingerprint TEXT NOT NULL,
                    command_id TEXT,
                    method_id TEXT,
                    response_json TEXT NOT NULL,
                    created_at REAL NOT NULL,
                    PRIMARY KEY(operation_kind, idempotency_key)
                ) WITHOUT ROWID;
                CREATE TABLE IF NOT EXISTS operator_plane_methods (
                    method_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    source_json TEXT NOT NULL,
                    digest TEXT NOT NULL,
                    failure_policy TEXT NOT NULL,
                    status TEXT NOT NULL,
                    version INTEGER NOT NULL,
                    ownership_generation INTEGER NOT NULL,
                    expanded_count INTEGER NOT NULL,
                    first_stream_sequence INTEGER,
                    last_stream_sequence INTEGER,
                    queued_at REAL NOT NULL,
                    updated_at REAL NOT NULL,
                    recovery_outcome_pending INTEGER NOT NULL DEFAULT 0
                );
                CREATE TABLE IF NOT EXISTS operator_plane_commands (
                    command_id TEXT PRIMARY KEY,
                    stream_sequence INTEGER NOT NULL UNIQUE,
                    method_id TEXT,
                    method_sequence INTEGER,
                    action_id TEXT NOT NULL,
                    requested_json TEXT NOT NULL,
                    effective_json TEXT NOT NULL,
                    status TEXT NOT NULL,
                    version INTEGER NOT NULL,
                    ownership_generation INTEGER NOT NULL,
                    dispatch_attempt_id TEXT,
                    dispatcher_epoch INTEGER,
                    dispatch_global_safety_epoch INTEGER,
                    dispatch_axis_safety_epoch INTEGER,
                    interrupt_id TEXT,
                    interrupt_global_safety_epoch INTEGER,
                    interrupt_axis_safety_epoch INTEGER,
                    source_noop INTEGER NOT NULL DEFAULT 0,
                    source_noop_reason TEXT,
                    controller_acknowledged INTEGER NOT NULL DEFAULT 0,
                    remote_acknowledged INTEGER NOT NULL DEFAULT 0,
                    physical_effect_verified INTEGER NOT NULL DEFAULT 0,
                    terminal_json TEXT,
                    queued_at REAL NOT NULL,
                    dispatched_at REAL,
                    finished_at REAL,
                    updated_at REAL NOT NULL,
                    FOREIGN KEY(method_id) REFERENCES operator_plane_methods(method_id)
                );
                CREATE INDEX IF NOT EXISTS operator_plane_commands_ready_idx
                    ON operator_plane_commands(status, stream_sequence);
                CREATE INDEX IF NOT EXISTS operator_plane_commands_method_idx
                    ON operator_plane_commands(method_id, method_sequence);
                CREATE TABLE IF NOT EXISTS operator_plane_transitions (
                    transition_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                    event_kind TEXT NOT NULL,
                    command_id TEXT,
                    method_id TEXT,
                    state TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS operator_plane_evidence (
                    evidence_id TEXT PRIMARY KEY,
                    command_id TEXT NOT NULL REFERENCES operator_plane_commands(command_id),
                    evidence_kind TEXT NOT NULL,
                    content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
                    payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
                    payload_bytes INTEGER NOT NULL CHECK(payload_bytes>=0),
                    created_at REAL NOT NULL,
                    UNIQUE(command_id,evidence_kind,content_sha256)
                );
                CREATE TABLE IF NOT EXISTS operator_plane_interrupt_evidence (
                    evidence_id TEXT PRIMARY KEY,
                    interrupt_attempt_id TEXT NOT NULL,
                    action_id TEXT NOT NULL,
                    evidence_kind TEXT NOT NULL,
                    content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
                    payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
                    payload_bytes INTEGER NOT NULL CHECK(payload_bytes>=0),
                    created_at REAL NOT NULL,
                    UNIQUE(interrupt_attempt_id,evidence_kind,content_sha256)
                ) WITHOUT ROWID;
                CREATE TABLE IF NOT EXISTS operator_plane_interrupt_attempts (
                    attempt_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                    interrupt_attempt_id TEXT NOT NULL,
                    idempotency_key TEXT NOT NULL,
                    fingerprint TEXT NOT NULL CHECK(length(fingerprint)=64),
                    action_id TEXT NOT NULL,
                    phase TEXT NOT NULL CHECK(phase IN ('admitted','attempted','terminal')),
                    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
                    created_at REAL NOT NULL,
                    UNIQUE(interrupt_attempt_id,phase)
                );
                CREATE INDEX IF NOT EXISTS operator_plane_interrupt_attempts_key_idx
                    ON operator_plane_interrupt_attempts(idempotency_key,attempt_sequence DESC);
                CREATE TABLE IF NOT EXISTS operator_plane_lane (
                    singleton INTEGER PRIMARY KEY CHECK(singleton=1),
                    active_command_id TEXT,
                    active_attempt_id TEXT,
                    dispatcher_epoch INTEGER NOT NULL,
                    owner_id TEXT,
                    owner_lease_until REAL,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS operator_plane_safety (
                    singleton INTEGER PRIMARY KEY CHECK(singleton=1),
                    global_epoch INTEGER NOT NULL,
                    x_epoch INTEGER NOT NULL,
                    y_epoch INTEGER NOT NULL DEFAULT 0,
                    z_epoch INTEGER NOT NULL,
                    recovery_epoch INTEGER NOT NULL,
                    recovery_version INTEGER NOT NULL,
                    recovery_hold INTEGER NOT NULL,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS operator_plane_z_home_authority (
                    singleton INTEGER PRIMARY KEY CHECK(singleton=1),
                    state TEXT NOT NULL CHECK(state IN ('invalid','valid')),
                    command_id TEXT,
                    ownership_generation INTEGER NOT NULL DEFAULT 0 CHECK(ownership_generation>=0),
                    board_lifecycle_generation INTEGER,
                    authority_version INTEGER NOT NULL DEFAULT 0 CHECK(authority_version>=0),
                    invalidation_reason TEXT,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS operator_plane_board_authority (
                    board_id INTEGER PRIMARY KEY CHECK(board_id=5),
                    state TEXT NOT NULL CHECK(state IN ('active','faulted')),
                    active_board_epoch INTEGER,
                    state_version INTEGER NOT NULL DEFAULT 1,
                    updated_at REAL NOT NULL
                ) WITHOUT ROWID;
                CREATE TABLE IF NOT EXISTS operator_plane_snapshots (
                    token TEXT PRIMARY KEY,
                    method_id TEXT NOT NULL,
                    watermark INTEGER NOT NULL,
                    expires_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS operator_plane_outbox (
                    outbox_id TEXT PRIMARY KEY,
                    command_id TEXT NOT NULL,
                    transition_sequence INTEGER NOT NULL,
                    state TEXT NOT NULL,
                    payload_json TEXT,
                    attempts INTEGER NOT NULL DEFAULT 0,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS operator_plane_recovery_acknowledgements (
                    acknowledgement_id TEXT PRIMARY KEY,
                    command_id TEXT NOT NULL REFERENCES operator_plane_commands(command_id),
                    recovery_epoch INTEGER NOT NULL CHECK(recovery_epoch>=0),
                    operation TEXT NOT NULL CHECK(operation IN ('resume_undispatched','cancel_pending')),
                    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
                    created_at REAL NOT NULL,
                    UNIQUE(command_id,recovery_epoch)
                ) WITHOUT ROWID;
                CREATE TABLE IF NOT EXISTS operator_plane_interrupt_history (
                    record_sha256 TEXT PRIMARY KEY,
                    stream TEXT NOT NULL CHECK(stream IN ('x','y','z')),
                    interrupt_attempt_id TEXT NOT NULL,
                    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
                    source_wrapper_json TEXT NOT NULL CHECK(json_valid(source_wrapper_json)),
                    imported_at REAL NOT NULL,
                    UNIQUE(stream,interrupt_attempt_id)
                ) WITHOUT ROWID;
                INSERT OR IGNORE INTO operator_plane_lane(singleton, dispatcher_epoch, updated_at) VALUES(1, 1, strftime('%s','now'));
                INSERT OR IGNORE INTO operator_plane_safety(singleton, global_epoch, x_epoch, z_epoch, recovery_epoch, recovery_version, recovery_hold, updated_at)
                    VALUES(1, 0, 0, 0, 0, 1, 0, strftime('%s','now'));
                INSERT OR IGNORE INTO operator_plane_z_home_authority(singleton,state,updated_at)
                    VALUES(1, 'invalid', strftime('%s','now'));
                INSERT OR IGNORE INTO operator_plane_board_authority(board_id,state,active_board_epoch,updated_at)
                    VALUES(5, 'faulted', NULL, strftime('%s','now'));
                COMMIT;
                """
            )
            for column, declaration in (("owner_id", "TEXT"), ("owner_lease_until", "REAL")):
                try:
                    self.connection.execute(f"ALTER TABLE operator_plane_lane ADD COLUMN {column} {declaration}")
                except sqlite3.OperationalError as exc:
                    if "duplicate column name" not in str(exc).lower():
                        raise
            try:
                self.connection.execute("ALTER TABLE operator_plane_safety ADD COLUMN y_epoch INTEGER NOT NULL DEFAULT 0")
            except sqlite3.OperationalError as exc:
                if "duplicate column name" not in str(exc).lower():
                    raise
            interrupt_history_columns = tuple(
                str(row[1])
                for row in self.connection.execute(
                    "PRAGMA table_info(operator_plane_interrupt_history)"
                ).fetchall()
            )
            canonical_interrupt_history_columns = (
                "record_sha256", "stream", "interrupt_attempt_id", "receipt_json",
                "source_wrapper_json", "imported_at",
            )
            additive_interrupt_history_columns = (
                "record_sha256", "stream", "interrupt_attempt_id", "receipt_json",
                "imported_at", "source_wrapper_json",
            )
            if interrupt_history_columns == additive_interrupt_history_columns:
                invalid_history = self.connection.execute(
                    """
                    SELECT COUNT(*) FROM operator_plane_interrupt_history
                    WHERE source_wrapper_json IS NULL
                       OR source_wrapper_json<>canonical_json(source_wrapper_json)
                       OR record_sha256<>sha256_utf8(source_wrapper_json)
                       OR json_extract(source_wrapper_json,'$.stream') IS NOT stream
                       OR json_extract(source_wrapper_json,'$.receipt.interrupt_attempt_id') IS NOT interrupt_attempt_id
                       OR canonical_json(json_extract(source_wrapper_json,'$.receipt'))<>receipt_json
                    """
                ).fetchone()
                if invalid_history is not None and int(invalid_history[0]) > 0:
                    raise RuntimeError(
                        "operator interrupt history requires exact-wrapper recovery from archived fallback bytes"
                    )
                try:
                    self.connection.execute("BEGIN IMMEDIATE")
                    self.connection.execute(
                        "ALTER TABLE operator_plane_interrupt_history "
                        "RENAME TO operator_plane_interrupt_history_additive_v4"
                    )
                    self.connection.execute(
                        """
                        CREATE TABLE operator_plane_interrupt_history (
                            record_sha256 TEXT PRIMARY KEY,
                            stream TEXT NOT NULL CHECK(stream IN ('x','y','z')),
                            interrupt_attempt_id TEXT NOT NULL,
                            receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
                            source_wrapper_json TEXT NOT NULL CHECK(json_valid(source_wrapper_json)),
                            imported_at REAL NOT NULL,
                            UNIQUE(stream,interrupt_attempt_id)
                        ) WITHOUT ROWID
                        """
                    )
                    self.connection.execute(
                        """
                        INSERT INTO operator_plane_interrupt_history(
                            record_sha256,stream,interrupt_attempt_id,receipt_json,
                            source_wrapper_json,imported_at
                        )
                        SELECT record_sha256,stream,interrupt_attempt_id,receipt_json,
                               source_wrapper_json,imported_at
                        FROM operator_plane_interrupt_history_additive_v4
                        """
                    )
                    self.connection.execute(
                        "DROP TABLE operator_plane_interrupt_history_additive_v4"
                    )
                    self.connection.execute("COMMIT")
                except Exception:
                    if self.connection.in_transaction:
                        self.connection.execute("ROLLBACK")
                    raise
                interrupt_history_columns = canonical_interrupt_history_columns
            elif "source_wrapper_json" not in interrupt_history_columns:
                self.connection.execute(
                    "ALTER TABLE operator_plane_interrupt_history ADD COLUMN source_wrapper_json TEXT"
                )
                interrupt_history_columns = tuple(
                    str(row[1])
                    for row in self.connection.execute(
                        "PRAGMA table_info(operator_plane_interrupt_history)"
                    ).fetchall()
                )
            incomplete_history = self.connection.execute(
                "SELECT COUNT(*) FROM operator_plane_interrupt_history WHERE source_wrapper_json IS NULL"
            ).fetchone()
            if incomplete_history is not None and int(incomplete_history[0]) > 0:
                raise RuntimeError("operator interrupt history requires exact-wrapper recovery from archived fallback bytes")
            self.connection.executescript(
                """
                CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_no_delete
                BEFORE DELETE ON operator_plane_transitions
                BEGIN SELECT RAISE(ABORT, 'operator transitions are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_no_update
                BEFORE UPDATE ON operator_plane_transitions
                BEGIN SELECT RAISE(ABORT, 'operator transitions are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_no_delete
                BEFORE DELETE ON operator_plane_evidence
                BEGIN SELECT RAISE(ABORT, 'operator evidence is append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_no_update
                BEFORE UPDATE ON operator_plane_evidence
                BEGIN SELECT RAISE(ABORT, 'operator evidence is append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_evidence_no_delete
                BEFORE DELETE ON operator_plane_interrupt_evidence
                BEGIN SELECT RAISE(ABORT, 'operator interrupt evidence is append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_evidence_no_update
                BEFORE UPDATE ON operator_plane_interrupt_evidence
                BEGIN SELECT RAISE(ABORT, 'operator interrupt evidence is append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_no_terminal_delete
                BEFORE DELETE ON operator_plane_commands
                WHEN OLD.status IN ('completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted')
                BEGIN SELECT RAISE(ABORT, 'terminal operator commands are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_no_terminal_delete
                BEFORE DELETE ON operator_plane_methods
                WHEN OLD.status IN ('completed','failed','cancelled','stopped','aborted','interrupted')
                BEGIN SELECT RAISE(ABORT, 'terminal operator methods are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_idempotency_no_update_v2
                BEFORE UPDATE ON operator_plane_idempotency
                BEGIN SELECT RAISE(ABORT, 'operator idempotency receipts are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_idempotency_no_delete_v2
                BEFORE DELETE ON operator_plane_idempotency
                BEGIN SELECT RAISE(ABORT, 'operator idempotency receipts are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_identity_immutable_v2
                BEFORE UPDATE ON operator_plane_commands
                WHEN NEW.command_id IS NOT OLD.command_id
                  OR NEW.stream_sequence IS NOT OLD.stream_sequence
                  OR NEW.method_id IS NOT OLD.method_id
                  OR NEW.method_sequence IS NOT OLD.method_sequence
                  OR NEW.action_id IS NOT OLD.action_id
                  OR NEW.requested_json IS NOT OLD.requested_json
                  OR NEW.effective_json IS NOT OLD.effective_json
                  OR NEW.ownership_generation IS NOT OLD.ownership_generation
                  OR NEW.queued_at IS NOT OLD.queued_at
                BEGIN SELECT RAISE(ABORT, 'operator command identity is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_terminal_no_update_v2
                BEFORE UPDATE ON operator_plane_commands
                WHEN OLD.status IN ('completed','failed','stopped','aborted','cancelled','cleared')
                BEGIN SELECT RAISE(ABORT, 'terminal operator commands are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_identity_immutable_v2
                BEFORE UPDATE ON operator_plane_methods
                WHEN NEW.method_id IS NOT OLD.method_id
                  OR NEW.name IS NOT OLD.name
                  OR NEW.source_json IS NOT OLD.source_json
                  OR NEW.digest IS NOT OLD.digest
                  OR NEW.failure_policy IS NOT OLD.failure_policy
                  OR NEW.ownership_generation IS NOT OLD.ownership_generation
                  OR NEW.expanded_count IS NOT OLD.expanded_count
                  OR NEW.first_stream_sequence IS NOT OLD.first_stream_sequence
                  OR NEW.last_stream_sequence IS NOT OLD.last_stream_sequence
                  OR NEW.queued_at IS NOT OLD.queued_at
                BEGIN SELECT RAISE(ABORT, 'operator method identity is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_terminal_no_update_v2
                BEFORE UPDATE ON operator_plane_methods
                WHEN OLD.status IN ('completed','failed','cancelled','stopped','aborted','interrupted')
                BEGIN SELECT RAISE(ABORT, 'terminal operator methods are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_identity_immutable_v2
                BEFORE UPDATE ON serial206_movement_commands
                WHEN NEW.sequence IS NOT OLD.sequence
                  OR NEW.command_id IS NOT OLD.command_id
                  OR NEW.idempotency_key IS NOT OLD.idempotency_key
                  OR NEW.action_id IS NOT OLD.action_id
                  OR NEW.method_id IS NOT OLD.method_id
                  OR NEW.method_order IS NOT OLD.method_order
                  OR NEW.parallel_group IS NOT OLD.parallel_group
                  OR NEW.axis_scope IS NOT OLD.axis_scope
                  OR NEW.board_scope_json IS NOT OLD.board_scope_json
                  OR NEW.ownership_generation IS NOT OLD.ownership_generation
                  OR NEW.expected_board_epochs_json IS NOT OLD.expected_board_epochs_json
                  OR NEW.canonical_inputs_sha256 IS NOT OLD.canonical_inputs_sha256
                  OR NEW.admitted_interrupt_epochs_json IS NOT OLD.admitted_interrupt_epochs_json
                  OR NEW.accepted_at IS NOT OLD.accepted_at
                  OR NEW.queued_at IS NOT OLD.queued_at
                BEGIN SELECT RAISE(ABORT, 'canonical movement command identity is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_terminal_no_update_v2
                BEFORE UPDATE ON serial206_movement_commands
                WHEN OLD.state IN ('completed','failed','cleared','interrupted','rejected')
                BEGIN SELECT RAISE(ABORT, 'terminal canonical movement commands are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_no_terminal_delete_v2
                BEFORE DELETE ON serial206_movement_commands
                WHEN OLD.state IN ('completed','failed','cleared','interrupted','ambiguous','rejected')
                BEGIN SELECT RAISE(ABORT, 'terminal canonical movement commands are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_identity_immutable_v2
                BEFORE UPDATE ON serial206_movement_methods
                WHEN NEW.method_id IS NOT OLD.method_id
                  OR NEW.idempotency_key IS NOT OLD.idempotency_key
                  OR NEW.action_id IS NOT OLD.action_id
                  OR NEW.canonical_inputs_sha256 IS NOT OLD.canonical_inputs_sha256
                  OR NEW.failure_policy IS NOT OLD.failure_policy
                  OR NEW.child_count IS NOT OLD.child_count
                  OR NEW.accepted_at IS NOT OLD.accepted_at
                BEGIN SELECT RAISE(ABORT, 'canonical movement method identity is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_terminal_no_update_v2
                BEFORE UPDATE ON serial206_movement_methods
                WHEN OLD.state IN ('completed','completed_partial','failed','cleared','interrupted')
                BEGIN SELECT RAISE(ABORT, 'terminal canonical movement methods are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_no_terminal_delete_v2
                BEFORE DELETE ON serial206_movement_methods
                WHEN OLD.state IN ('completed','completed_partial','failed','cleared','interrupted','ambiguous')
                BEGIN SELECT RAISE(ABORT, 'terminal canonical movement methods are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_command_resources_no_update_v2
                BEFORE UPDATE ON serial206_command_resources
                BEGIN SELECT RAISE(ABORT, 'command resource authority is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_command_resources_no_delete_v2
                BEFORE DELETE ON serial206_command_resources
                BEGIN SELECT RAISE(ABORT, 'command resource authority is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_command_dependencies_no_update_v2
                BEFORE UPDATE ON serial206_command_dependencies
                BEGIN SELECT RAISE(ABORT, 'command dependency authority is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_command_dependencies_no_delete_v2
                BEFORE DELETE ON serial206_command_dependencies
                BEGIN SELECT RAISE(ABORT, 'command dependency authority is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_attempts_no_update_v2
                BEFORE UPDATE ON operator_plane_interrupt_attempts
                BEGIN SELECT RAISE(ABORT, 'interrupt attempt history is append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_attempts_no_delete_v2
                BEFORE DELETE ON operator_plane_interrupt_attempts
                BEGIN SELECT RAISE(ABORT, 'interrupt attempt history is append-only'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_command_resources_sealed_insert_v3
                BEFORE INSERT ON serial206_command_resources
                WHEN EXISTS(SELECT 1 FROM operator_plane_transitions WHERE command_id=NEW.command_id)
                BEGIN SELECT RAISE(ABORT, 'command resource authority is sealed at admission'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_command_dependencies_sealed_insert_v3
                BEFORE INSERT ON serial206_command_dependencies
                WHEN EXISTS(SELECT 1 FROM operator_plane_transitions WHERE command_id=NEW.command_id)
                  OR EXISTS(
                    WITH RECURSIVE ancestors(command_id) AS (
                      SELECT NEW.depends_on_command_id
                      UNION
                      SELECT dependency.depends_on_command_id
                      FROM serial206_command_dependencies AS dependency
                      JOIN ancestors ON dependency.command_id=ancestors.command_id
                    )
                    SELECT 1 FROM ancestors WHERE command_id=NEW.command_id
                  )
                BEGIN SELECT RAISE(ABORT, 'command dependency authority is sealed or cyclic'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_coherence_insert_v1
                BEFORE INSERT ON operator_plane_evidence
                WHEN authority_write_allowed()<>1
                  OR NEW.content_sha256 <> sha256_utf8(NEW.payload_json)
                  OR NEW.payload_bytes <> length(CAST(NEW.payload_json AS BLOB))
                  OR NEW.payload_json <> canonical_json(NEW.payload_json)
                  OR NEW.evidence_id <> NEW.command_id || ':' || NEW.evidence_kind || ':' || NEW.content_sha256
                BEGIN SELECT RAISE(ABORT, 'operator evidence bytes and identity are incoherent'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_evidence_coherence_insert_v1
                BEFORE INSERT ON operator_plane_interrupt_evidence
                WHEN authority_write_allowed()<>1
                  OR NEW.content_sha256 <> sha256_utf8(NEW.payload_json)
                  OR NEW.payload_bytes <> length(CAST(NEW.payload_json AS BLOB))
                  OR NEW.payload_json <> canonical_json(NEW.payload_json)
                  OR NEW.evidence_id <> NEW.interrupt_attempt_id || ':' || NEW.evidence_kind || ':' || NEW.content_sha256
                BEGIN SELECT RAISE(ABORT, 'interrupt evidence bytes and identity are incoherent'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_attempts_lineage_insert_v1
                BEFORE INSERT ON operator_plane_interrupt_attempts
                WHEN authority_write_allowed()<>1
                  OR NEW.receipt_json <> canonical_json(NEW.receipt_json)
                  OR json_extract(NEW.receipt_json,'$.interrupt_attempt_id') IS NOT NEW.interrupt_attempt_id
                  OR json_extract(NEW.receipt_json,'$.action_id') IS NOT NEW.action_id
                  OR (NEW.phase='admitted' AND EXISTS(SELECT 1 FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=NEW.interrupt_attempt_id))
                  OR (NEW.phase<>'admitted' AND NOT EXISTS(SELECT 1 FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=NEW.interrupt_attempt_id AND phase='admitted'))
                  OR (NEW.phase<>'admitted' AND EXISTS(SELECT 1 FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=NEW.interrupt_attempt_id AND phase='terminal'))
                  OR (NEW.phase<>'admitted' AND EXISTS(
                      SELECT 1 FROM operator_plane_interrupt_attempts
                      WHERE interrupt_attempt_id=NEW.interrupt_attempt_id AND phase='admitted'
                        AND (idempotency_key IS NOT NEW.idempotency_key OR fingerprint IS NOT NEW.fingerprint OR action_id IS NOT NEW.action_id)
                  ))
                BEGIN SELECT RAISE(ABORT, 'interrupt attempt lineage or receipt identity is incoherent'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_recoverable_terminal_no_update_v3
                BEFORE UPDATE ON operator_plane_commands
                WHEN OLD.status IN ('ambiguous','interrupted')
                BEGIN SELECT RAISE(ABORT, 'recoverable terminal operator commands are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_ambiguous_no_update_v3
                BEFORE UPDATE ON serial206_movement_commands
                WHEN OLD.state='ambiguous'
                BEGIN SELECT RAISE(ABORT, 'ambiguous canonical movement commands are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_recovery_acknowledgements_no_update_v1
                BEFORE UPDATE ON operator_plane_recovery_acknowledgements
                BEGIN SELECT RAISE(ABORT, 'recovery acknowledgements are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_recovery_acknowledgements_no_delete_v1
                BEFORE DELETE ON operator_plane_recovery_acknowledgements
                BEGIN SELECT RAISE(ABORT, 'recovery acknowledgements are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_recovery_acknowledgements_authorized_insert_v2
                BEFORE INSERT ON operator_plane_recovery_acknowledgements
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'recovery acknowledgement writer is not authoritative'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_authorized_insert_v4
                BEFORE INSERT ON operator_plane_commands
                WHEN authority_write_allowed()<>1
                  OR NEW.status NOT IN ('queued','dispatched','issued_pending','stop_requested','abort_requested','completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted','rejected')
                BEGIN SELECT RAISE(ABORT, 'operator command writer or state is not authoritative'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_authorized_update_v4
                BEFORE UPDATE ON operator_plane_commands
                WHEN authority_write_allowed()<>1
                  OR NEW.status NOT IN ('queued','dispatched','issued_pending','stop_requested','abort_requested','completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted','rejected')
                BEGIN SELECT RAISE(ABORT, 'operator command writer or state is not authoritative'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_authorized_insert_v4
                BEFORE INSERT ON serial206_movement_commands
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'canonical command writer is not authoritative'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_authorized_update_v4
                BEFORE UPDATE ON serial206_movement_commands
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'canonical command writer is not authoritative'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_authorized_delete_v4
                BEFORE DELETE ON serial206_movement_commands
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'canonical command delete requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_command_resources_authorized_insert_v4
                BEFORE INSERT ON serial206_command_resources
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'command resource writer is not authoritative'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_command_dependencies_authorized_insert_v4
                BEFORE INSERT ON serial206_command_dependencies
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'dependency insert requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_authorized_insert_v4
                BEFORE INSERT ON operator_plane_methods
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'method insert requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_authorized_update_v4
                BEFORE UPDATE ON operator_plane_methods
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'method update requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_authorized_delete_v4
                BEFORE DELETE ON operator_plane_methods
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'method delete requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_authorized_insert_v4
                BEFORE INSERT ON serial206_movement_methods
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'movement method insert requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_authorized_update_v4
                BEFORE UPDATE ON serial206_movement_methods
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'movement method update requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_authorized_delete_v4
                BEFORE DELETE ON serial206_movement_methods
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'movement method delete requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_idempotency_authorized_insert_v4
                BEFORE INSERT ON operator_plane_idempotency
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'idempotency insert requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_authorized_delete_v4
                BEFORE DELETE ON operator_plane_commands
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'command delete requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_history_authorized_insert_v4
                BEFORE INSERT ON operator_plane_interrupt_history
                WHEN authority_write_allowed()<>1
                BEGIN SELECT RAISE(ABORT, 'interrupt history insert requires authoritative writer'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_history_no_update_v4
                BEFORE UPDATE ON operator_plane_interrupt_history
                BEGIN SELECT RAISE(ABORT, 'interrupt history is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_history_no_delete_v4
                BEFORE DELETE ON operator_plane_interrupt_history
                BEGIN SELECT RAISE(ABORT, 'interrupt history is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_history_coherence_insert_v4
                BEFORE INSERT ON operator_plane_interrupt_history
                WHEN NEW.source_wrapper_json IS NULL
                  OR NEW.source_wrapper_json<>canonical_json(NEW.source_wrapper_json)
                  OR NEW.record_sha256<>sha256_utf8(NEW.source_wrapper_json)
                  OR json_extract(NEW.source_wrapper_json,'$.stream') IS NOT NEW.stream
                  OR json_extract(NEW.source_wrapper_json,'$.receipt.interrupt_attempt_id') IS NOT NEW.interrupt_attempt_id
                  OR canonical_json(json_extract(NEW.source_wrapper_json,'$.receipt'))<>NEW.receipt_json
                BEGIN SELECT RAISE(ABORT, 'interrupt history wrapper is incoherent'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_authorized_coherent_insert_v4
                BEFORE INSERT ON operator_plane_transitions
                WHEN authority_write_allowed()<>1
                  OR (NEW.command_id IS NOT NULL AND NOT EXISTS(
                    SELECT 1
                    FROM operator_plane_commands AS operator
                    JOIN serial206_movement_commands AS canonical ON canonical.command_id=operator.command_id
                    WHERE operator.command_id=NEW.command_id AND (
                      operator.status=canonical.state
                      OR (operator.status='cancelled' AND canonical.state='cleared')
                      OR (operator.status='interrupted' AND canonical.state='ambiguous')
                      OR (operator.status IN ('stop_requested','abort_requested') AND canonical.state='interrupting')
                      OR (operator.status IN ('stopped','aborted') AND canonical.state='interrupted')
                    )
                  ))
                BEGIN SELECT RAISE(ABORT, 'operator and canonical command authority is incoherent'); END;
                """
            )
            installed_trigger_names = {
                str(row[0])
                for row in self.connection.execute("SELECT name FROM sqlite_master WHERE type='trigger'").fetchall()
            }
            missing_trigger_names = sorted(set(required_trigger_names) - installed_trigger_names)
            if missing_trigger_names:
                raise RuntimeError(f"operator command schema missing triggers: {','.join(missing_trigger_names)}")
            trigger_placeholders = ",".join("?" for _ in required_trigger_names)
            governed_tables = {
                str(row[0])
                for row in self.connection.execute(
                    f"SELECT DISTINCT tbl_name FROM sqlite_master WHERE type='trigger' AND name IN ({trigger_placeholders})",
                    tuple(required_trigger_names),
                ).fetchall()
            }
            table_placeholders = ",".join("?" for _ in governed_tables)
            actual_governed_triggers = {
                str(row[0])
                for row in self.connection.execute(
                    f"SELECT name FROM sqlite_master WHERE type='trigger' AND tbl_name IN ({table_placeholders})",
                    tuple(sorted(governed_tables)),
                ).fetchall()
            }
            if actual_governed_triggers != set(required_trigger_names):
                raise RuntimeError("operator command schema trigger set is not exact")
            expected_table_fragments = {
                "operator_plane_idempotency": "primary key(operation_kind, idempotency_key)",
                "operator_plane_methods": "method_id text primary key",
                "operator_plane_commands": "command_id text primary key",
                "serial206_movement_methods": "method_id text primary key",
                "operator_plane_interrupt_history": "record_sha256 text primary key",
            }
            for table_name, fragment in expected_table_fragments.items():
                row = self.connection.execute(
                    "SELECT sql FROM sqlite_master WHERE type='table' AND name=?",
                    (table_name,),
                ).fetchone()
                normalized_sql = " ".join(str(row[0] if row else "").lower().split())
                if fragment not in normalized_sql:
                    raise RuntimeError(f"operator command schema table attestation failed: {table_name}")
            expected_column_names = {
                "operator_plane_idempotency": ("operation_kind","idempotency_key","fingerprint","command_id","method_id","response_json","created_at"),
                "operator_plane_methods": ("method_id","name","source_json","digest","failure_policy","status","version","ownership_generation","expanded_count","first_stream_sequence","last_stream_sequence","queued_at","updated_at","recovery_outcome_pending"),
                "operator_plane_commands": ("command_id","stream_sequence","method_id","method_sequence","action_id","requested_json","effective_json","status","version","ownership_generation","dispatch_attempt_id","dispatcher_epoch","dispatch_global_safety_epoch","dispatch_axis_safety_epoch","interrupt_id","interrupt_global_safety_epoch","interrupt_axis_safety_epoch","source_noop","source_noop_reason","controller_acknowledged","remote_acknowledged","physical_effect_verified","terminal_json","queued_at","dispatched_at","finished_at","updated_at"),
                "operator_plane_transitions": ("transition_sequence","event_kind","command_id","method_id","state","payload_json","created_at"),
                "serial206_movement_methods": ("method_id","idempotency_key","action_id","canonical_inputs_sha256","state","state_version","failure_policy","child_count","accepted_at","started_at","finished_at"),
                "serial206_movement_commands": ("sequence","command_id","idempotency_key","action_id","method_id","method_order","parallel_group","axis_scope","board_scope_json","ownership_generation","expected_board_epochs_json","canonical_inputs_sha256","state","state_version","admitted_interrupt_epochs_json","accepted_at","queued_at","dispatched_at","finished_at","terminal_receipt_id"),
                "serial206_command_resources": ("command_id","resource_key"),
                "serial206_command_dependencies": ("command_id","depends_on_command_id","required_terminal"),
                "operator_plane_interrupt_history": ("record_sha256","stream","interrupt_attempt_id","receipt_json","source_wrapper_json","imported_at"),
            }
            for table_name, expected_names in expected_column_names.items():
                actual_info = self.connection.execute(f"PRAGMA table_info({table_name})").fetchall()
                actual_names = tuple(str(row[1]) for row in actual_info)
                if actual_names != expected_names or any(str(row[2]).upper() not in {"TEXT", "INTEGER", "REAL"} for row in actual_info):
                    raise RuntimeError(f"operator command schema columns are not exact: {table_name}")
            expected_foreign_keys = {
                "operator_plane_commands": {("method_id","operator_plane_methods","method_id","NO ACTION","NO ACTION","NONE")},
                "serial206_movement_commands": {("method_id","serial206_movement_methods","method_id","NO ACTION","CASCADE","NONE")},
                "serial206_command_resources": {("command_id","serial206_movement_commands","command_id","NO ACTION","CASCADE","NONE")},
                "serial206_command_dependencies": {
                    ("command_id","serial206_movement_commands","command_id","NO ACTION","CASCADE","NONE"),
                    ("depends_on_command_id","serial206_movement_commands","command_id","NO ACTION","CASCADE","NONE"),
                },
            }
            for table_name, expected in expected_foreign_keys.items():
                actual = {
                    (str(row[3]),str(row[2]),str(row[4]),str(row[5]).upper(),str(row[6]).upper(),str(row[7]).upper())
                    for row in self.connection.execute(f"PRAGMA foreign_key_list({table_name})").fetchall()
                }
                if actual != expected:
                    raise RuntimeError(f"operator command schema foreign keys are not exact: {table_name}")
            physical_schema_sha256 = _operator_physical_schema_sha256(self.connection)
            if physical_schema_sha256 != _OPERATOR_PHYSICAL_SCHEMA_SHA256:
                raise RuntimeError(
                    f"operator command physical schema fingerprint mismatch: {physical_schema_sha256}"
                )

    @contextmanager
    def _authority_write(self):
        if self._authority_write_depth == 0 and hasattr(self, "_authority_schema_version"):
            schema_version = int(self.connection.execute("PRAGMA schema_version").fetchone()[0])
            if schema_version != self._authority_schema_version:
                physical_schema_sha256 = _operator_physical_schema_sha256(self.connection)
                if physical_schema_sha256 != _OPERATOR_PHYSICAL_SCHEMA_SHA256:
                    raise RuntimeError(
                        f"operator command physical schema fingerprint mismatch: {physical_schema_sha256}"
                    )
                self._authority_schema_version = schema_version
        self._authority_write_depth += 1
        try:
            yield
        finally:
            self._authority_write_depth -= 1

    @contextmanager
    def _transaction(self, *, timeout_ms: int = 2000):
        with self._lock:
            self.connection.execute(f"PRAGMA busy_timeout={int(timeout_ms)}")
            try:
                self.connection.execute("BEGIN IMMEDIATE")
                with self._authority_write():
                    yield self.connection
                self.connection.execute("COMMIT")
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise
            finally:
                self.connection.execute("PRAGMA busy_timeout=2000")

    def _insert_transition(self, conn: sqlite3.Connection, *, event_kind: str, state: str, command_id: str | None = None, method_id: str | None = None, payload: Mapping[str, Any] | None = None) -> int:
        row = conn.execute(
            "INSERT INTO operator_plane_transitions(event_kind,command_id,method_id,state,payload_json,created_at) VALUES(?,?,?,?,?,?) RETURNING transition_sequence",
            (event_kind, command_id, method_id, state, _canonical(payload or {}), _now()),
        ).fetchone()
        assert row is not None
        return int(row[0])

    def _store_evidence(
        self,
        conn: sqlite3.Connection,
        *,
        command_id: str,
        evidence_kind: str,
        payload: Any,
    ) -> dict[str, Any]:
        encoded = _canonical(payload)
        content_sha256 = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
        evidence_id = f"{command_id}:{evidence_kind}:{content_sha256}"
        with self._authority_write():
            conn.execute(
                "INSERT OR IGNORE INTO operator_plane_evidence(evidence_id,command_id,evidence_kind,content_sha256,payload_json,payload_bytes,created_at) VALUES(?,?,?,?,?,?,?)",
                (evidence_id, command_id, evidence_kind, content_sha256, encoded, len(encoded.encode("utf-8")), _now()),
            )
        return {
            "evidence_id": evidence_id,
            "evidence_kind": evidence_kind,
            "content_sha256": content_sha256,
            "payload_bytes": len(encoded.encode("utf-8")),
        }

    def _store_interrupt_evidence(
        self,
        conn: sqlite3.Connection,
        *,
        interrupt_attempt_id: str,
        action_id: str,
        evidence_kind: str,
        payload: Any,
    ) -> dict[str, Any]:
        encoded = _canonical(payload)
        content_sha256 = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
        evidence_id = f"{interrupt_attempt_id}:{evidence_kind}:{content_sha256}"
        with self._authority_write():
            conn.execute(
                "INSERT OR IGNORE INTO operator_plane_interrupt_evidence(evidence_id,interrupt_attempt_id,action_id,evidence_kind,content_sha256,payload_json,payload_bytes,created_at) VALUES(?,?,?,?,?,?,?,?)",
                (evidence_id, interrupt_attempt_id, action_id, evidence_kind, content_sha256, encoded, len(encoded.encode("utf-8")), _now()),
            )
        return {
            "evidence_id": evidence_id,
            "evidence_kind": evidence_kind,
            "content_sha256": content_sha256,
            "payload_bytes": len(encoded.encode("utf-8")),
        }

    def _acquire_owner(self) -> bool:
        try:
            with self._transaction() as conn:
                lane = conn.execute("SELECT owner_id,owner_lease_until FROM operator_plane_lane WHERE singleton=1").fetchone()
                now = _now()
                if lane is not None and lane["owner_id"] not in {None, self.owner_id} and float(lane["owner_lease_until"] or 0.0) > now:
                    return False
                conn.execute("UPDATE operator_plane_lane SET owner_id=?,owner_lease_until=?,dispatcher_epoch=dispatcher_epoch+1,updated_at=? WHERE singleton=1", (self.owner_id, now + 5.0, now))
                return True
        except sqlite3.OperationalError:
            return False

    def _renew_owner(self) -> bool:
        with self._transaction(timeout_ms=100) as conn:
            changed = conn.execute("UPDATE operator_plane_lane SET owner_lease_until=?,updated_at=? WHERE singleton=1 AND owner_id=?", (_now() + 5.0, _now(), self.owner_id)).rowcount
            return changed == 1

    def _startup_recover(self) -> None:
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
            active = conn.execute(
                "SELECT command_id,method_id,status FROM operator_plane_commands WHERE status IN ('dispatched','issued_pending','stop_requested','abort_requested')"
            ).fetchall()
            lane = conn.execute("SELECT active_command_id,active_attempt_id FROM operator_plane_lane WHERE singleton=1").fetchone()
            if not active and (lane is None or (lane["active_command_id"] is None and lane["active_attempt_id"] is None)):
                return
            recovery_epoch = int(row["recovery_epoch"]) + 1
            if active:
                for item in active:
                    command_id = str(item["command_id"])
                    conn.execute(
                        "UPDATE operator_plane_commands SET status='interrupted',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE command_id=?",
                        (_now(), _now(), _canonical({"reason": "process_owner_loss", "outcome_unknown": True}), command_id),
                    )
                    conn.execute("UPDATE serial206_movement_commands SET state='ambiguous',state_version=state_version+1,finished_at=? WHERE command_id=? AND state IN ('dispatched','issued_pending','interrupting')", (_now(), command_id))
                    conn.execute("UPDATE operator_plane_z_home_authority SET state='invalid',authority_version=authority_version+1,invalidation_reason='process_owner_loss',updated_at=? WHERE singleton=1", (_now(),))
                    self._insert_transition(conn, event_kind="command_recovery", command_id=command_id, method_id=item["method_id"], state="interrupted", payload={"reason": "process_owner_loss"})
                method_ids = {str(item["method_id"]) for item in active if item["method_id"]}
                for method_id in method_ids:
                    conn.execute("UPDATE operator_plane_methods SET status='recovery_required',version=version+1,updated_at=? WHERE method_id=? AND status NOT IN ('completed','failed','cancelled','stopped','aborted','interrupted')", (_now(), method_id))
                    self._insert_transition(conn, event_kind="method_recovery", method_id=method_id, state="recovery_required", payload={"reason": "process_owner_loss"})
            conn.execute("UPDATE operator_plane_lane SET active_command_id=NULL,active_attempt_id=NULL,dispatcher_epoch=dispatcher_epoch+1,updated_at=? WHERE singleton=1", (_now(),))
            conn.execute("UPDATE operator_plane_safety SET recovery_epoch=?,recovery_version=recovery_version+1,recovery_hold=0,updated_at=? WHERE singleton=1", (recovery_epoch, _now()))

    def _capacity(self, conn: sqlite3.Connection) -> tuple[int, int]:
        count = int(conn.execute("SELECT COUNT(*) FROM operator_plane_commands WHERE status NOT IN ('completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted')").fetchone()[0])
        byte_count = int(conn.execute("SELECT COALESCE(SUM(length(CAST(requested_json AS BLOB))+length(CAST(effective_json AS BLOB))),0) FROM operator_plane_commands WHERE status NOT IN ('completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted')").fetchone()[0])
        return count, byte_count

    def _z_home_authority_row(self, conn: sqlite3.Connection) -> sqlite3.Row:
        row = conn.execute("SELECT * FROM operator_plane_z_home_authority WHERE singleton=1").fetchone()
        if row is None:
            raise RuntimeError("operator-plane Z Home authority row is missing")
        return row

    @staticmethod
    def _provider_z_authority(state: Mapping[str, Any]) -> Mapping[str, Any] | None:
        provider = state.get("serial206_initialization_provider") if isinstance(state, Mapping) else None
        authority = provider.get("z_authority") if isinstance(provider, Mapping) else None
        return authority if isinstance(authority, Mapping) else None

    def _sync_z_home_authority(self, conn: sqlite3.Connection, state: Mapping[str, Any]) -> sqlite3.Row:
        row = self._z_home_authority_row(conn)
        if str(row["state"]) != "valid":
            return row
        authority = self._provider_z_authority(state)
        current_generation = int(state.get("ownership_generation") or 0)
        provider_state = str(authority.get("state") or "") if authority is not None else ""
        provider_reference = str(authority.get("reference_state") or "") if authority is not None else ""
        provider_board_generation = authority.get("board_lifecycle_generation") if authority is not None else None
        stale = (
            int(row["ownership_generation"]) != current_generation
            or provider_state != "referenced_ready"
            or provider_reference != "referenced"
            or (
                provider_board_generation is not None
                and row["board_lifecycle_generation"] is not None
                and int(provider_board_generation) != int(row["board_lifecycle_generation"])
            )
        )
        if stale:
            conn.execute(
                "UPDATE operator_plane_z_home_authority SET state='invalid',authority_version=authority_version+1,invalidation_reason=?,updated_at=? WHERE singleton=1 AND state='valid'",
                ("provider_generation_or_reference_changed", _now()),
            )
            row = self._z_home_authority_row(conn)
        return row

    def _z_home_authority_valid(self, conn: sqlite3.Connection, state: Mapping[str, Any]) -> bool:
        row = self._sync_z_home_authority(conn, state)
        return str(row["state"]) == "valid" and int(row["ownership_generation"]) == int(state.get("ownership_generation") or 0)

    def z_home_authority_valid(self, state: Mapping[str, Any]) -> bool:
        with self._transaction() as conn:
            return self._z_home_authority_valid(conn, state)

    @staticmethod
    def _payload_board_generation(payload: Mapping[str, Any]) -> int | None:
        pending: list[Any] = [payload]
        visited = 0
        while pending and visited < 400:
            current = pending.pop(0)
            visited += 1
            if isinstance(current, Mapping):
                for key in ("board_lifecycle_generation", "current_board_lifecycle_generation"):
                    value = current.get(key)
                    if type(value) is int and value >= 0:
                        return value
                pending.extend(value for value in current.values() if isinstance(value, (Mapping, list, tuple)))
            elif isinstance(current, (list, tuple)):
                pending.extend(value for value in current if isinstance(value, (Mapping, list, tuple)))
        return None

    def _update_z_home_authority(self, conn: sqlite3.Connection, *, command_id: str, ownership_generation: int, status: str, payload: Mapping[str, Any], source_noop: bool) -> None:
        action = str(conn.execute("SELECT action_id FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()[0])
        if action == "oem.z.manual_home" and status == "completed" and not source_noop and _home_completion_proven(payload):
            conn.execute(
                "UPDATE operator_plane_z_home_authority SET state='valid',command_id=?,ownership_generation=?,board_lifecycle_generation=?,authority_version=authority_version+1,invalidation_reason=NULL,updated_at=? WHERE singleton=1",
                (command_id, int(ownership_generation), self._payload_board_generation(payload), _now()),
            )
        elif action.startswith("oem.z.") and status in {"failed", "ambiguous", "stopped", "aborted", "interrupted"}:
            conn.execute(
                "UPDATE operator_plane_z_home_authority SET state='invalid',authority_version=authority_version+1,invalidation_reason=?,updated_at=? WHERE singleton=1",
                (f"z_command_{status}", _now()),
            )

    def _saved_idempotency(self, conn: sqlite3.Connection, kind: str, key: str, fingerprint: str) -> dict[str, Any] | None:
        row = conn.execute("SELECT * FROM operator_plane_idempotency WHERE operation_kind=? AND idempotency_key=?", (kind, key)).fetchone()
        if row is None:
            return None
        if str(row["fingerprint"]) != fingerprint:
            raise HTTPException(status_code=409, detail={"error": "idempotency_conflict", "operation_kind": kind, "idempotency_key": key})
        payload = _json_load(row["response_json"], {})
        if isinstance(payload, dict):
            payload["idempotent_replay"] = True
        return payload

    def _store_idempotency(self, conn: sqlite3.Connection, *, kind: str, key: str, fingerprint: str, response: Mapping[str, Any], command_id: str | None = None, method_id: str | None = None) -> None:
        conn.execute(
            "INSERT INTO operator_plane_idempotency(operation_kind,idempotency_key,fingerprint,command_id,method_id,response_json,created_at) VALUES(?,?,?,?,?,?,?)",
            (kind, key, fingerprint, command_id, method_id, _canonical(response), _now()),
        )

    @staticmethod
    def _latest_interrupt_attempt(
        conn: sqlite3.Connection,
        *,
        interrupt_attempt_id: str | None = None,
        idempotency_key: str | None = None,
    ) -> sqlite3.Row | None:
        if interrupt_attempt_id is not None:
            return conn.execute(
                "SELECT * FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=? ORDER BY attempt_sequence DESC LIMIT 1",
                (str(interrupt_attempt_id),),
            ).fetchone()
        if idempotency_key is not None:
            return conn.execute(
                "SELECT * FROM operator_plane_interrupt_attempts WHERE idempotency_key=? ORDER BY attempt_sequence DESC LIMIT 1",
                (str(idempotency_key),),
            ).fetchone()
        raise ValueError("interrupt attempt identity is required")

    @staticmethod
    def _append_interrupt_attempt(
        conn: sqlite3.Connection,
        *,
        interrupt_attempt_id: str,
        idempotency_key: str,
        fingerprint: str,
        action_id: str,
        phase: str,
        receipt: Mapping[str, Any],
    ) -> None:
        conn.execute(
            "INSERT INTO operator_plane_interrupt_attempts(interrupt_attempt_id,idempotency_key,fingerprint,action_id,phase,receipt_json,created_at) VALUES(?,?,?,?,?,?,?)",
            (
                str(interrupt_attempt_id),
                str(idempotency_key),
                str(fingerprint),
                str(action_id),
                str(phase),
                _canonical(receipt),
                _now(),
            ),
        )

    def _insert_canonical_command(
        self,
        conn: sqlite3.Connection,
        *,
        command_id: str,
        idempotency_key: str,
        action_id: str,
        inputs: Mapping[str, Any],
        ownership_generation: int,
        accepted_at: float,
        expected_board_epochs: Mapping[str, int] | None = None,
        method_id: str | None = None,
        method_order: int = 0,
    ) -> None:
        axis = AXIS_BY_ACTION.get(action_id)
        motor_by_axis = {"y": 0, "z": 1}
        composite_xy = action_id.startswith("oem.xy.")
        composite_xyz = action_id.startswith("oem.xyz.") or action_id == "oem.z.scriptmove_to"
        axis_scope = "xyz" if composite_xyz else "xy" if composite_xy else axis
        board_scope = (
            {"4": [0, 1], "5": [0]}
            if composite_xyz
            else {"4": [0], "5": [0]}
            if composite_xy
            else {"4": [motor_by_axis[axis]]}
            if axis in motor_by_axis
            else {}
        )
        canonical_hash = _digest(dict(inputs))
        expected_epochs = dict(expected_board_epochs or {}) if axis in motor_by_axis or composite_xy or composite_xyz else {}
        safety = conn.execute("SELECT x_epoch,y_epoch,z_epoch FROM operator_plane_safety WHERE singleton=1").fetchone()
        interrupt_epochs = (
            {"x": int(safety["x_epoch"]), "y": int(safety["y_epoch"]), "z": int(safety["z_epoch"])}
            if composite_xyz
            else {"x": int(safety["x_epoch"]), "y": int(safety["y_epoch"])}
            if composite_xy
            else {axis: int(safety[f"{axis}_epoch"])}
            if axis in {"x", "y", "z"}
            else {}
        )
        conn.execute(
            """
            INSERT INTO serial206_movement_commands(
                command_id,idempotency_key,action_id,method_id,method_order,parallel_group,
                axis_scope,board_scope_json,ownership_generation,expected_board_epochs_json,
                canonical_inputs_sha256,state,state_version,admitted_interrupt_epochs_json,
                accepted_at,queued_at
            ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            """,
            (
                command_id,
                idempotency_key,
                action_id,
                method_id,
                int(method_order),
                0,
                axis_scope,
                _canonical(board_scope),
                int(ownership_generation),
                _canonical(expected_epochs),
                canonical_hash,
                "queued",
                1,
                _canonical(interrupt_epochs),
                accepted_at,
                accepted_at,
            ),
        )
        if composite_xyz:
            resources = ["axis:x", "axis:y", "axis:z", "motor:5:0", "motor:4:0", "motor:4:1"]
        elif composite_xy:
            resources = ["axis:x", "axis:y", "motor:5:0", "motor:4:0"]
        elif axis in {"y", "z"}:
            resources = [f"axis:{axis}", f"motor:4:{motor_by_axis[axis]}"]
        elif axis is not None:
            resources = [f"axis:{axis}"]
        else:
            resources = ["robot:operator"]
        conn.executemany(
            "INSERT INTO serial206_command_resources(command_id,resource_key) VALUES(?,?)",
            [(command_id, resource) for resource in resources],
        )

    def _sync_board_authority(self, conn: sqlite3.Connection, epochs: Mapping[str, int]) -> None:
        now = _now()
        if "4" in epochs:
            conn.execute(
                "UPDATE serial206_board_authority SET state='active',active_board_epoch=?,state_version=state_version+1,updated_at=? WHERE board_id=4",
                (int(epochs["4"]), now),
            )
        if "5" in epochs:
            conn.execute(
                "INSERT INTO operator_plane_board_authority(board_id,state,active_board_epoch,state_version,updated_at) VALUES(5,'active',?,1,?) "
                "ON CONFLICT(board_id) DO UPDATE SET state='active',active_board_epoch=excluded.active_board_epoch,state_version=operator_plane_board_authority.state_version+1,updated_at=excluded.updated_at",
                (int(epochs["5"]), now),
            )

    def _command_response(self, row: sqlite3.Row, *, transition_sequence: int | None = None) -> dict[str, Any]:
        if transition_sequence is None:
            transition_row = self.connection.execute("SELECT MAX(transition_sequence) FROM operator_plane_transitions WHERE command_id=?", (str(row["command_id"]),)).fetchone()
            transition_sequence = transition_row[0] if transition_row and transition_row[0] is not None else None
        canonical = self.connection.execute(
            "SELECT sequence,state,state_version,expected_board_epochs_json,terminal_receipt_id FROM serial206_movement_commands WHERE command_id=?",
            (str(row["command_id"]),),
        ).fetchone()
        terminal = _json_load(row["terminal_json"], None)
        return {
            "schema_version": RECEIPT_SCHEMA,
            "robot_identity": ROBOT_IDENTITY,
            "command_id": str(row["command_id"]),
            "method_id": row["method_id"],
            "method_sequence": row["method_sequence"],
            "stream_sequence": int(row["stream_sequence"]),
            "action_id": str(row["action_id"]),
            "status": str(canonical["state"]) if canonical is not None else str(row["status"]),
            "ownership_generation": int(row["ownership_generation"]),
            "requested_inputs": _json_load(row["requested_json"], {}),
            "effective_inputs": _json_load(row["effective_json"], {}),
            "accepted_at": float(row["queued_at"]),
            "queued_at": float(row["queued_at"]),
            "dispatched_at": row["dispatched_at"],
            "finished_at": row["finished_at"],
            "source_noop": bool(row["source_noop"]),
            "source_noop_reason": row["source_noop_reason"],
            "remote_acknowledged": bool(row["remote_acknowledged"]),
            "controller_acknowledged": bool(row["controller_acknowledged"]),
            "physical_effect_verified": bool(row["physical_effect_verified"]),
            "terminal_evidence": terminal,
            "sequence": int(canonical["sequence"]) if canonical is not None else int(row["stream_sequence"]),
            "state_version": int(canonical["state_version"]) if canonical is not None else int(row["version"]),
            "expected_board_epoch_by_board": _json_load(canonical["expected_board_epochs_json"], {}) if canonical is not None else {},
            "terminal_receipt_id": canonical["terminal_receipt_id"] if canonical is not None else None,
            "completion_class": terminal.get("completion_class") if isinstance(terminal, Mapping) else None,
            "transition_sequence": transition_sequence,
        }

    def _method_response(self, row: sqlite3.Row) -> dict[str, Any]:
        transition_row = self.connection.execute("SELECT MAX(transition_sequence) FROM operator_plane_transitions WHERE method_id=?", (str(row["method_id"]),)).fetchone()
        transition_sequence = transition_row[0] if transition_row and transition_row[0] is not None else None
        pending_count, pending_bytes = self._capacity(self.connection)
        return {
            "schema_version": METHOD_RECEIPT_SCHEMA,
            "robot_identity": ROBOT_IDENTITY,
            "method_id": str(row["method_id"]),
            "name": str(row["name"]),
            "method_digest": str(row["digest"]),
            "status": str(row["status"]),
            "version": int(row["version"]),
            "ownership_generation": int(row["ownership_generation"]),
            "expanded_count": int(row["expanded_count"]),
            "first_stream_sequence": row["first_stream_sequence"],
            "last_stream_sequence": row["last_stream_sequence"],
            "queued_at": float(row["queued_at"]),
            "updated_at": float(row["updated_at"]),
            "failure_policy": str(row["failure_policy"]),
            "transition_sequence": int(transition_sequence) if transition_sequence is not None else None,
            "capacity": {
                "pending_count": pending_count,
                "pending_bytes": pending_bytes,
                "capacity_commands": COMMAND_CAPACITY,
                "capacity_bytes": COMMAND_BYTES_CAPACITY,
            },
        }

    def admit_command(
        self,
        request: Mapping[str, Any],
        *,
        state: Mapping[str, Any],
        assessment: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        action_id = str(request.get("action_id") or "")
        if action_id not in ALLOWED_ACTIONS:
            raise HTTPException(status_code=422, detail={"error": "action_not_allowed", "action_id": action_id})
        inputs = _validate_inputs(action_id, request.get("inputs", {}))
        expected_generation = int(request["expected_ownership_generation"])
        schema_version = str(request.get("schema_version") or COMMAND_SCHEMA)
        if action_id.startswith("oem.y.") and schema_version != ACTION_REQUEST_SCHEMA:
            raise HTTPException(status_code=410, detail={"error": "legacy_y_mutation_retired", "required_schema": ACTION_REQUEST_SCHEMA})
        requested_board_epochs = {
            str(key): int(value)
            for key, value in dict(request.get("expected_board_epoch_by_board") or {}).items()
            if type(value) is int and value >= 0
        }
        observed_board_epochs = _active_board_epochs(state, action_id)
        expected_board_epochs: dict[str, int] = {}
        canonical_request = {
            "schema_version": schema_version,
            "operation_kind": "command",
            "expected_ownership_generation": expected_generation,
            "expected_board_epoch_by_board": expected_board_epochs,
            "requested_board_epoch_by_board": requested_board_epochs,
            "observed_board_epoch_by_board": observed_board_epochs,
            "board_epoch_policy": "observational_not_dispatch_fence",
            "action_id": action_id,
            "inputs": inputs,
        }
        fingerprint = _digest(canonical_request)
        key = str(request["idempotency_key"])
        saved = self.idempotency_checked("command", key, fingerprint)
        if saved is not None:
            current = self.get_command(str(saved.get("command_id") or ""))
            replay_response = current or saved
            replay_response["idempotent_replay"] = True
            return replay_response
        with self._transaction() as conn:
            replay = self._saved_idempotency(conn, "command", key, fingerprint)
            if replay is not None:
                current_row = conn.execute(
                    "SELECT * FROM operator_plane_commands WHERE command_id=?",
                    (str(replay.get("command_id") or ""),),
                ).fetchone()
                replay_response = self._command_response(current_row) if current_row is not None else replay
                replay_response["idempotent_replay"] = True
                return replay_response
            count, bytes_used = self._capacity(conn)
            proposed_bytes = len(_canonical(inputs).encode("utf-8")) + len(_canonical(canonical_request).encode("utf-8"))
            if count >= COMMAND_CAPACITY or bytes_used + proposed_bytes > COMMAND_BYTES_CAPACITY:
                raise HTTPException(status_code=429, detail={"error": "capacity_exceeded", "pending_count": count, "pending_bytes": bytes_used, "max_pending_commands": COMMAND_CAPACITY, "max_pending_bytes": COMMAND_BYTES_CAPACITY})
            command_id = str(uuid.uuid4())
            sequence = int(conn.execute("SELECT COALESCE(MAX(stream_sequence),0)+1 FROM operator_plane_commands").fetchone()[0])
            effective = _effective_inputs(action_id, inputs, state)
            row = {
                "command_id": command_id, "method_id": None, "method_sequence": None,
                "stream_sequence": sequence, "action_id": action_id, "requested_inputs": inputs,
                "effective_inputs": effective, "status": "queued", "ownership_generation": expected_generation,
                "queued_at": _now(),
            }
            conn.execute(
                "INSERT INTO operator_plane_commands(command_id,stream_sequence,method_id,method_sequence,action_id,requested_json,effective_json,status,version,ownership_generation,queued_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
                (command_id, sequence, None, None, action_id, _canonical(inputs), _canonical(effective), "queued", 1, expected_generation, row["queued_at"], row["queued_at"]),
            )
            self._insert_canonical_command(
                conn,
                command_id=command_id,
                idempotency_key=key,
                action_id=action_id,
                inputs=inputs,
                ownership_generation=expected_generation,
                accepted_at=row["queued_at"],
                expected_board_epochs=expected_board_epochs,
            )
            transition = self._insert_transition(
                conn,
                event_kind="command_admitted",
                command_id=command_id,
                state="queued",
                payload={
                    "stream_sequence": sequence,
                    "action_id": action_id,
                    "requested_board_epoch_by_board": requested_board_epochs,
                    "observed_board_epoch_by_board": observed_board_epochs,
                    "board_epoch_policy": "observational_not_dispatch_fence",
                },
            )
            response = {**row, "schema_version": RECEIPT_SCHEMA, "transition_sequence": transition, "remote_acknowledged": False, "controller_acknowledged": False, "physical_effect_verified": False, "source_noop": False, "source_noop_reason": None, "terminal_evidence": None, "dispatched_at": None, "finished_at": None, "idempotent_replay": False}
            self._store_idempotency(conn, kind="command", key=key, fingerprint=fingerprint, response=response, command_id=command_id)
        self._wake.set()
        return response

    def admit_method(
        self,
        request: Mapping[str, Any],
        *,
        state: Mapping[str, Any],
        assessments: list[Mapping[str, Any]] | None = None,
    ) -> dict[str, Any]:
        expected_generation = int(request["expected_ownership_generation"])
        requested_board_epochs = {
            str(key): int(value)
            for key, value in dict(request.get("expected_board_epoch_by_board") or {}).items()
            if type(value) is int and value >= 0
        }
        expected_board_epochs: dict[str, int] = {}
        expanded: list[tuple[str, dict[str, Any]]] = []
        for step in request.get("steps", []):
            action_id = str(step.get("action_id") or "")
            if action_id not in ALLOWED_ACTIONS:
                raise HTTPException(status_code=422, detail={"error": "method_action_not_allowed", "action_id": action_id})
            inputs = _validate_inputs(action_id, step.get("inputs", {}))
            repeat = int(step.get("repeat", 1))
            if repeat < 1 or repeat > MAX_METHOD_COMMANDS or len(expanded) + repeat > MAX_METHOD_COMMANDS:
                raise HTTPException(status_code=422, detail={"error": "method_expansion_limit"})
            for _ in range(repeat):
                expanded.append((action_id, dict(inputs)))
        if not expanded or len(expanded) > MAX_METHOD_COMMANDS:
            raise HTTPException(status_code=422, detail={"error": "method_expansion_limit"})
        source = {
            "schema_version": METHOD_SCHEMA,
            "name": str(request["name"]),
            "failure_policy": "fail_fast",
            "expected_board_epoch_by_board": expected_board_epochs,
            "steps": [{"action_id": a, "inputs": i} for a, i in expanded],
            "metadata": {
                **dict(request.get("metadata") or {}),
                "requested_board_epoch_by_board": requested_board_epochs,
                "board_epoch_policy": "observational_not_dispatch_fence",
            },
        }
        digest = _digest(source)
        canonical_request = {"schema_version": METHOD_SCHEMA, "operation_kind": "method", "expected_ownership_generation": expected_generation, "expected_board_epoch_by_board": expected_board_epochs, "source": source}
        fingerprint = _digest(canonical_request)
        key = str(request["idempotency_key"])
        saved = self.idempotency_checked("method", key, fingerprint)
        if saved is not None:
            current = self.get_method(str(saved.get("method_id") or ""))
            replay_response = current or saved
            replay_response["idempotent_replay"] = True
            return replay_response
        with self._transaction() as conn:
            replay = self._saved_idempotency(conn, "method", key, fingerprint)
            if replay is not None:
                current_row = conn.execute(
                    "SELECT * FROM operator_plane_methods WHERE method_id=?",
                    (str(replay.get("method_id") or ""),),
                ).fetchone()
                replay_response = self._method_response(current_row) if current_row is not None else replay
                replay_response["idempotent_replay"] = True
                return replay_response
            count, bytes_used = self._capacity(conn)
            proposed_bytes = len(_canonical(source).encode("utf-8")) + sum(len(_canonical(i).encode("utf-8")) for _, i in expanded)
            if count + len(expanded) > COMMAND_CAPACITY or bytes_used + proposed_bytes > COMMAND_BYTES_CAPACITY:
                raise HTTPException(status_code=429, detail={"error": "capacity_exceeded", "pending_count": count, "pending_bytes": bytes_used, "max_pending_commands": COMMAND_CAPACITY, "max_pending_bytes": COMMAND_BYTES_CAPACITY})
            method_id = str(uuid.uuid4())
            now = _now()
            first_sequence = int(conn.execute("SELECT COALESCE(MAX(stream_sequence),0)+1 FROM operator_plane_commands").fetchone()[0])
            last_sequence = first_sequence + len(expanded) - 1
            conn.execute(
                "INSERT INTO operator_plane_methods(method_id,name,source_json,digest,failure_policy,status,version,ownership_generation,expanded_count,first_stream_sequence,last_stream_sequence,queued_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (method_id, str(request["name"]), _canonical(source), digest, "fail_fast", "queued", 1, expected_generation, len(expanded), first_sequence, last_sequence, now, now),
            )
            conn.execute(
                "INSERT INTO serial206_movement_methods(method_id,idempotency_key,action_id,canonical_inputs_sha256,state,state_version,failure_policy,child_count,accepted_at) VALUES(?,?,?,?,?,?,?,?,?)",
                (method_id, key, "operator.method", digest, "queued", 1, "require_completed", len(expanded), now),
            )
            previous_command_id: str | None = None
            for index, (action_id, inputs) in enumerate(expanded, start=1):
                sequence = first_sequence + index - 1
                command_id = str(uuid.uuid4())
                effective = _effective_inputs(action_id, inputs, state)
                conn.execute(
                    "INSERT INTO operator_plane_commands(command_id,stream_sequence,method_id,method_sequence,action_id,requested_json,effective_json,status,version,ownership_generation,queued_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
                    (command_id, sequence, method_id, index, action_id, _canonical(inputs), _canonical(effective), "queued", 1, expected_generation, now, now),
                )
                self._insert_canonical_command(
                    conn,
                    command_id=command_id,
                    idempotency_key=f"{key}:{index}",
                    action_id=action_id,
                    inputs=inputs,
                    ownership_generation=expected_generation,
                    accepted_at=now,
                    method_id=method_id,
                    method_order=index,
                    expected_board_epochs={board: epoch for board, epoch in expected_board_epochs.items() if board in _active_board_epochs(state, action_id)},
                )
                if previous_command_id is not None:
                    conn.execute(
                        "INSERT INTO serial206_command_dependencies(command_id,depends_on_command_id,required_terminal) VALUES(?,?,'completed')",
                        (command_id, previous_command_id),
                    )
                previous_command_id = command_id
                self._insert_transition(conn, event_kind="command_admitted", command_id=command_id, method_id=method_id, state="queued", payload={"stream_sequence": sequence, "method_sequence": index, "action_id": action_id})
            transition = self._insert_transition(conn, event_kind="method_admitted", method_id=method_id, state="queued", payload={"expanded_count": len(expanded), "first_stream_sequence": first_sequence, "last_stream_sequence": last_sequence})
            method_row = conn.execute("SELECT * FROM operator_plane_methods WHERE method_id=?", (method_id,)).fetchone()
            assert method_row is not None
            response = {**self._method_response(method_row), "transition_sequence": transition, "idempotent_replay": False}
            self._store_idempotency(conn, kind="method", key=key, fingerprint=fingerprint, response=response, method_id=method_id)
        self._wake.set()
        return response

    def get_command(self, command_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self.connection.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            return self._command_response(row) if row else None

    def list_commands(self, *, limit: int = 100, before_sequence: int | None = None) -> list[dict[str, Any]]:
        bounded = min(max(int(limit), 1), 200)
        with self._lock:
            if before_sequence is None:
                rows = self.connection.execute(
                    "SELECT * FROM operator_plane_commands ORDER BY stream_sequence DESC LIMIT ?",
                    (bounded,),
                ).fetchall()
            else:
                rows = self.connection.execute(
                    "SELECT * FROM operator_plane_commands WHERE stream_sequence<? ORDER BY stream_sequence DESC LIMIT ?",
                    (int(before_sequence), bounded),
                ).fetchall()
            return [self._command_response(row) for row in rows]

    def command_detail_v2(self, command_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self.connection.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                return None
            projection = self._command_response(row)
            raw_terminal = projection.get("terminal_evidence")
            terminal: dict[str, Any] = dict(raw_terminal) if isinstance(raw_terminal, Mapping) else {}
            nested_response = terminal.get("response")
            response: dict[str, Any] = dict(nested_response) if isinstance(nested_response, Mapping) else dict(terminal)
            raw_keys = ("motor_command_raw_return", "board_wrapper_return", "public_wrapper_return", "public_wrapper_return_kind", "motor_command_delivery_count")
            controller_keys = ("completion_class", "controller_completion_verified", "terminal_speed_zero", "source_returned_normally", "event", "event_window", "wait")
            observed_keys = ("position_after", "terminal_position", "terminal_speed", "discrepancy", "observed_position_steps")
            transitions = self.connection.execute(
                "SELECT transition_sequence,state,payload_json,created_at FROM operator_plane_transitions WHERE command_id=? ORDER BY transition_sequence LIMIT 200",
                (command_id,),
            ).fetchall()
            resources = [str(item[0]) for item in self.connection.execute(
                "SELECT resource_key FROM serial206_command_resources WHERE command_id=? ORDER BY resource_key",
                (command_id,),
            ).fetchall()]
            effective_values = dict(projection.get("effective_inputs") or {})
            missing = object()

            def scalar_observation(key: str, value: Any) -> Any:
                if value is None or isinstance(value, (bool, int, float, str)):
                    return value
                if not isinstance(value, Mapping):
                    return missing
                preferred = {
                    "position_after": ("position", "position_steps", "value"),
                    "terminal_position": ("position", "position_steps", "value"),
                    "terminal_speed": ("speed", "speed_steps", "value"),
                    "discrepancy": ("discrepancy", "steps", "value"),
                    "observed_position_steps": ("position", "position_steps", "value"),
                }.get(key, ("value",))
                for nested_key in preferred:
                    nested = value.get(nested_key)
                    if nested is None or isinstance(nested, (bool, int, float, str)):
                        if nested_key in value:
                            return nested
                return missing

            raw_observed = terminal.get("observed_values")
            if not isinstance(raw_observed, Mapping):
                raw_observed = {key: response[key] for key in observed_keys if key in response}
            observed_values = {}
            for key, value in raw_observed.items():
                scalar = scalar_observation(str(key), value)
                if scalar is not missing:
                    observed_values[str(key)] = scalar

            def transition_status(value: Any) -> str:
                return {
                    "stop_requested": "interrupting",
                    "abort_requested": "interrupting",
                    "stopped": "interrupted",
                    "aborted": "interrupted",
                    "cancelled": "cleared",
                }.get(str(value), str(value))
            for key in ("board_effective_target", "motor_effective_target", "near_high_noop"):
                if key in response:
                    effective_values[key] = response[key]
            projection.update({
                "canonical_inputs": dict(projection.get("requested_inputs") or {}),
                "requested_values": dict(projection.get("requested_inputs") or {}),
                "effective_values": effective_values,
                "observed_values": observed_values,
                "raw_return_layers": dict(response.get("raw_return_layers") or terminal.get("raw_return_layers") or {key: response[key] for key in raw_keys if key in response}),
                "controller_evidence": dict(response.get("controller_evidence") or terminal.get("controller_evidence") or {key: response[key] for key in controller_keys if key in response}),
                "transport_artifacts": list(terminal.get("transport_artifacts") or []),
                "child_receipts": list(terminal.get("child_receipts") or []),
                "resource_keys": resources,
                "transitions": [
                    {
                        "transition_id": str(item["transition_sequence"]),
                        "from_status": None if index == 0 else transition_status(transitions[index - 1]["state"]),
                        "to_status": transition_status(item["state"]),
                        "at": float(item["created_at"]),
                        "reason": (_json_load(item["payload_json"], {}) or {}).get("reason"),
                    }
                    for index, item in enumerate(transitions)
                ],
            })
            return projection

    def get_method(self, method_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self.connection.execute("SELECT * FROM operator_plane_methods WHERE method_id=?", (method_id,)).fetchone()
            return self._method_response(row) if row else None

    def list_method_commands(self, method_id: str) -> list[dict[str, Any]]:
        with self._lock:
            rows = self.connection.execute(
                "SELECT * FROM operator_plane_commands WHERE method_id=? ORDER BY method_sequence,stream_sequence",
                (method_id,),
            ).fetchall()
            return [self._command_response(row) for row in rows]

    def method_commands(self, method_id: str, *, after: int, limit: int, token: str | None) -> dict[str, Any]:
        limit = min(max(int(limit), 1), 200)
        with self._transaction() as conn:
            method = conn.execute("SELECT * FROM operator_plane_methods WHERE method_id=?", (method_id,)).fetchone()
            if method is None:
                raise HTTPException(status_code=404, detail="method not found")
            if token is None:
                watermark = int(conn.execute("SELECT COALESCE(MAX(transition_sequence),0) FROM operator_plane_transitions").fetchone()[0])
                token = uuid.uuid4().hex
                conn.execute("INSERT INTO operator_plane_snapshots(token,method_id,watermark,expires_at) VALUES(?,?,?,?)", (token, method_id, watermark, _now() + 900))
            else:
                snapshot = conn.execute("SELECT * FROM operator_plane_snapshots WHERE token=? AND method_id=?", (token, method_id)).fetchone()
                if snapshot is None:
                    raise HTTPException(status_code=409, detail={"error": "snapshot_expired"})
                if float(snapshot["expires_at"]) < _now():
                    conn.execute("DELETE FROM operator_plane_snapshots WHERE token=?", (token,))
                    raise HTTPException(status_code=409, detail={"error": "snapshot_expired"})
                watermark = int(snapshot["watermark"])
            rows = conn.execute("SELECT * FROM operator_plane_commands WHERE method_id=? AND method_sequence>? ORDER BY method_sequence LIMIT ?", (method_id, int(after), limit + 1)).fetchall()
            has_more = len(rows) > limit
            rows = rows[:limit]
            result: list[dict[str, Any]] = []
            for row in rows:
                current = self._command_response(row, transition_sequence=watermark)
                transition = conn.execute("SELECT state,payload_json,transition_sequence FROM operator_plane_transitions WHERE command_id=? AND transition_sequence<=? ORDER BY transition_sequence DESC LIMIT 1", (row["command_id"], watermark)).fetchone()
                if transition is not None:
                    current["status"] = str(transition["state"])
                    current["snapshot_transition_sequence"] = int(transition["transition_sequence"])
                result.append(current)
            next_after = int(rows[-1]["method_sequence"]) if rows else int(after)
            return {"schema_version": "bioxp.operator_method_commands.v1", "method_id": method_id, "commands": result, "next_after_method_sequence": next_after, "has_more": has_more, "snapshot_transition_sequence": watermark, "snapshot_token": token}

    def queue(self) -> dict[str, Any]:
        with self._lock:
            count, bytes_used = self._capacity(self.connection)
            active = self.connection.execute("SELECT * FROM operator_plane_commands WHERE status IN ('dispatched','issued_pending') ORDER BY stream_sequence LIMIT 1").fetchone()
            row = self.connection.execute("SELECT stream_sequence FROM operator_plane_commands WHERE status NOT IN ('completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted') ORDER BY stream_sequence LIMIT 1").fetchone()
            safety = self.connection.execute("SELECT global_epoch FROM operator_plane_safety WHERE singleton=1").fetchone()
            return {"schema_version": "bioxp.operator_queue.v1", "pending_count": count, "pending_bytes": bytes_used, "capacity_commands": COMMAND_CAPACITY, "capacity_bytes": COMMAND_BYTES_CAPACITY, "active_command": self._command_response(active) if active else None, "next_stream_sequence": int(row[0]) if row else None, "global_safety_epoch": int(safety["global_epoch"]) if safety is not None else 0}

    def transitions(self, *, after: int, limit: int) -> dict[str, Any]:
        limit = min(max(int(limit), 1), 200)
        with self._lock:
            rows = self.connection.execute("SELECT * FROM operator_plane_transitions WHERE transition_sequence>? ORDER BY transition_sequence LIMIT ?", (int(after), limit + 1)).fetchall()
            has_more = len(rows) > limit
            rows = rows[:limit]
            events = [{"event_kind": str(row["event_kind"]), "transition_sequence": int(row["transition_sequence"]), "command_id": row["command_id"], "method_id": row["method_id"], "state": str(row["state"]), "payload": _json_load(row["payload_json"], {}) , "created_at": float(row["created_at"])} for row in rows]
            high = int(self.connection.execute("SELECT COALESCE(MAX(transition_sequence),0) FROM operator_plane_transitions").fetchone()[0])
            low = int(self.connection.execute("SELECT COALESCE(MIN(transition_sequence),0) FROM operator_plane_transitions").fetchone()[0])
            next_after = int(rows[-1]["transition_sequence"]) if rows else int(after)
            return {"schema_version": TRANSITION_SCHEMA, "events": events, "next_after": next_after, "has_more": has_more, "low_watermark": low, "high_watermark": high}

    def recovery(self) -> dict[str, Any]:
        with self._lock:
            safety = self.connection.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
            unknown = self.connection.execute("SELECT command_id,method_id,status,stream_sequence FROM operator_plane_commands WHERE status IN ('ambiguous','interrupted') ORDER BY stream_sequence").fetchall()
            methods = sorted({str(row["method_id"]) for row in unknown if row["method_id"]})
            queued_rows = self.connection.execute("SELECT stream_sequence FROM operator_plane_commands WHERE status='queued' ORDER BY stream_sequence").fetchall()
            queued = len(queued_rows)
            lane = self.connection.execute("SELECT dispatcher_epoch FROM operator_plane_lane WHERE singleton=1").fetchone()
            queued_range = {
                "first_stream_sequence": int(queued_rows[0][0]) if queued_rows else None,
                "last_stream_sequence": int(queued_rows[-1][0]) if queued_rows else None,
            }
            return {"schema_version": RECOVERY_SCHEMA, "recovery_epoch": int(safety["recovery_epoch"]), "version": int(safety["recovery_version"]), "hold": bool(safety["recovery_hold"]), "outcome_unknown_command_ids": [str(row["command_id"]) for row in unknown], "affected_method_ids": methods, "queued_count": int(queued), "queued_range": queued_range, "dispatcher_epoch": int(lane["dispatcher_epoch"]) if lane is not None else 0, "global_safety_epoch": int(safety["global_epoch"]), "x_safety_epoch": int(safety["x_epoch"]), "y_safety_epoch": int(safety["y_epoch"]), "z_safety_epoch": int(safety["z_epoch"]), "available_resolutions": ["resume_undispatched", "cancel_pending"] if safety["recovery_hold"] else []}

    def cancel_command(self, command_id: str, request: Mapping[str, Any]) -> dict[str, Any]:
        key = str(request["idempotency_key"])
        fp = _digest({"operation_kind": "cancel_command", "command_id": command_id, **_without_idempotency(request)})
        saved = self.idempotency_checked("cancel_command", key, fp)
        if saved is not None:
            saved["idempotent_replay"] = True
            return saved
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail="command not found")
            if str(row["status"]) != "queued":
                raise HTTPException(status_code=409, detail={"error": "command_not_queued", "status": row["status"]})
            safety = conn.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
            if int(row["version"]) != int(request["expected_version"]):
                raise HTTPException(status_code=409, detail={"error": "command_version_conflict", "current_version": int(row["version"])})
            axis_epoch = int(safety[{"x": "x_epoch", "y": "y_epoch", "z": "z_epoch"}.get(AXIS_BY_ACTION.get(str(row["action_id"])) or "", "z_epoch")])
            if int(safety["recovery_epoch"]) != int(request["expected_recovery_epoch"]) or int(safety["global_epoch"]) != int(request["expected_global_safety_epoch"]) or axis_epoch != int(request["expected_axis_safety_epoch"]):
                raise HTTPException(status_code=409, detail={"error": "command_safety_epoch_conflict"})
            conn.execute("UPDATE operator_plane_commands SET status='cancelled',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE command_id=? AND status='queued' AND version=?", (_now(), _now(), _canonical({"reason": "standalone_cancel"}), command_id, int(row["version"])))
            conn.execute("UPDATE serial206_movement_commands SET state='cleared',state_version=state_version+1,finished_at=? WHERE command_id=? AND state='queued'", (_now(), command_id))
            transition = self._insert_transition(conn, event_kind="command_cancelled", command_id=command_id, state="cancelled", payload={"reason": "standalone_cancel"})
            updated = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            assert updated is not None
            response = self._command_response(updated, transition_sequence=transition)
            self._store_idempotency(conn, kind="cancel_command", key=str(request["idempotency_key"]), fingerprint=fp, response=response, command_id=command_id)
            return response

    def method_mutation(self, method_id: str, operation: str, request: Mapping[str, Any]) -> dict[str, Any]:
        fp = _digest({"operation_kind": operation, "method_id": method_id, **_without_idempotency(request)})
        key = str(request["idempotency_key"])
        saved = self.idempotency_checked(operation, key, fp)
        if saved is not None:
            saved["idempotent_replay"] = True
            return saved
        with self._transaction() as conn:
            method = conn.execute("SELECT * FROM operator_plane_methods WHERE method_id=?", (method_id,)).fetchone()
            if method is None:
                raise HTTPException(status_code=404, detail="method not found")
            replay = self._saved_idempotency(conn, operation, key, fp)
            if replay is not None:
                return replay
            safety = conn.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
            axes = [str(item[0]) for item in conn.execute("SELECT action_id FROM operator_plane_commands WHERE method_id=?", (method_id,)).fetchall()]
            relevant_axis_epoch = max(
                [int(safety["x_epoch"]) for action in axes if action.startswith("oem.x.")]
                + [int(safety["z_epoch"]) for action in axes if action.startswith("oem.z.")]
                + [0]
            )
            if int(safety["recovery_epoch"]) != int(request["expected_recovery_epoch"]) or int(safety["global_epoch"]) != int(request["expected_global_safety_epoch"]) or relevant_axis_epoch != int(request["expected_axis_safety_epoch"]):
                raise HTTPException(status_code=409, detail={"error": "method_safety_epoch_conflict"})
            if int(method["version"]) != int(request["expected_version"]):
                raise HTTPException(status_code=409, detail={"error": "method_version_conflict", "current_version": int(method["version"])})
            status = str(method["status"])
            if operation == "pause":
                if status not in {"queued", "running"}:
                    raise HTTPException(status_code=409, detail={"error": "method_not_pauseable", "status": status})
                new_status = "pause_requested"
            elif operation == "resume":
                if status != "paused":
                    raise HTTPException(status_code=409, detail={"error": "method_not_resumable", "status": status})
                new_status = "running"
            else:
                if status not in METHOD_NONTERMINAL:
                    raise HTTPException(status_code=409, detail={"error": "method_not_cancellable", "status": status})
                new_status = "cancel_requested"
                conn.execute("UPDATE operator_plane_commands SET status='cancelled',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE method_id=? AND status='queued'", (_now(), _now(), _canonical({"reason": "method_cancel"}), method_id))
            conn.execute("UPDATE operator_plane_methods SET status=?,version=version+1,updated_at=? WHERE method_id=? AND version=?", (new_status, _now(), method_id, int(method["version"])))
            transition = self._insert_transition(conn, event_kind=f"method_{operation}", method_id=method_id, state=new_status, payload={"expected_version": int(method["version"])})
            updated = conn.execute("SELECT * FROM operator_plane_methods WHERE method_id=?", (method_id,)).fetchone()
            assert updated is not None
            response = {**self._method_response(updated), "mutation_id": str(uuid.uuid4()), "prior_version": int(method["version"]), "transition_sequence": transition}
            self._store_idempotency(conn, kind=operation, key=key, fingerprint=fp, response=response, method_id=method_id)
        self._wake.set()
        return response

    def resolve_recovery(self, recovery_epoch: int, request: Mapping[str, Any]) -> dict[str, Any]:
        fp = _digest({"operation_kind": "recovery_resolve", "recovery_epoch": recovery_epoch, **_without_idempotency(request)})
        key = str(request["idempotency_key"])
        saved = self.idempotency_checked("recovery_resolve", key, fp)
        if saved is not None:
            saved["idempotent_replay"] = True
            return saved
        with self._transaction() as conn:
            safety = conn.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
            replay = self._saved_idempotency(conn, "recovery_resolve", key, fp)
            if replay is not None:
                return replay
            if int(safety["recovery_epoch"]) != int(recovery_epoch) or int(safety["recovery_version"]) != int(request["expected_version"]):
                raise HTTPException(status_code=409, detail={"error": "recovery_version_conflict"})
            if int(safety["global_epoch"]) != int(request["expected_safety_epoch"]):
                raise HTTPException(status_code=409, detail={"error": "safety_epoch_conflict", "expected": int(request["expected_safety_epoch"]), "actual": int(safety["global_epoch"])})
            unknown = [str(row[0]) for row in conn.execute("SELECT command_id FROM operator_plane_commands WHERE status IN ('ambiguous','interrupted') ORDER BY stream_sequence").fetchall()]
            if sorted(unknown) != sorted([str(item) for item in request.get("acknowledge_command_ids", [])]):
                raise HTTPException(status_code=409, detail={"error": "incomplete_outcome_unknown_acknowledgement", "required": unknown})
            operation = str(request["operation"])
            if operation == "cancel_pending":
                conn.execute("UPDATE operator_plane_commands SET status='cancelled',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE status='queued'", (_now(), _now(), _canonical({"reason": "recovery_cancel_pending"})))
            affected_methods = sorted({str(row["method_id"]) for row in conn.execute("SELECT method_id FROM operator_plane_commands WHERE command_id IN ({})".format(",".join("?" for _ in unknown)), tuple(unknown)).fetchall() if row["method_id"]} if unknown else set())
            for command_id in unknown:
                acknowledgement = {
                    "schema_version": "bioxp.operator_recovery_acknowledgement.v1",
                    "command_id": command_id,
                    "recovery_epoch": int(recovery_epoch),
                    "operation": operation,
                    "outcome_remains": "unknown",
                }
                conn.execute(
                    "INSERT INTO operator_plane_recovery_acknowledgements(acknowledgement_id,command_id,recovery_epoch,operation,receipt_json,created_at) VALUES(?,?,?,?,?,?)",
                    (str(uuid.uuid4()), command_id, int(recovery_epoch), operation, _canonical(acknowledgement), _now()),
                )
            if operation == "cancel_pending":
                for method_id in affected_methods:
                    conn.execute("UPDATE operator_plane_methods SET status='cancelled',version=version+1,updated_at=? WHERE method_id=? AND status NOT IN ('completed','failed','cancelled','stopped','aborted','interrupted')", (_now(), method_id))
            else:
                for method_id in affected_methods:
                    conn.execute("UPDATE operator_plane_methods SET status='running',version=version+1,updated_at=? WHERE method_id=? AND status='recovery_required'", (_now(), method_id))
            conn.execute("UPDATE operator_plane_safety SET recovery_hold=0,recovery_version=recovery_version+1,updated_at=? WHERE singleton=1", (_now(),))
            transition = self._insert_transition(conn, event_kind="recovery_resolved", state="recovery_resolved", payload={"operation": request["operation"], "acknowledged": unknown})
            response = {"schema_version": "bioxp.operator_recovery_resolution.v1", "recovery_epoch": int(recovery_epoch), "operation": request["operation"], "acknowledged_command_ids": unknown, "transition_sequence": transition}
            self._store_idempotency(conn, kind="recovery_resolve", key=key, fingerprint=fp, response=response)
        self._wake.set()
        return response

    def idempotency_checked(self, operation_kind: str, key: str, fingerprint: str) -> dict[str, Any] | None:
        with self._lock:
            if operation_kind == "interrupt":
                row = self._latest_interrupt_attempt(
                    self.connection,
                    idempotency_key=key,
                )
                if row is None:
                    return None
                if str(row["fingerprint"]) != fingerprint:
                    raise HTTPException(status_code=409, detail={"error": "idempotency_conflict", "operation_kind": operation_kind, "idempotency_key": key})
                return _json_load(row["receipt_json"], None)
            row = self.connection.execute("SELECT fingerprint,response_json FROM operator_plane_idempotency WHERE operation_kind=? AND idempotency_key=?", (operation_kind, key)).fetchone()
            if row is None:
                return None
            if str(row["fingerprint"]) != fingerprint:
                raise HTTPException(status_code=409, detail={"error": "idempotency_conflict", "operation_kind": operation_kind, "idempotency_key": key})
            return _json_load(row["response_json"], None)

    def idempotency(self, operation_kind: str, key: str) -> dict[str, Any] | None:
        with self._lock:
            if operation_kind == "interrupt":
                row = self._latest_interrupt_attempt(
                    self.connection,
                    idempotency_key=key,
                )
                if row is None:
                    return None
                return {
                    "schema_version": "bioxp.operator_idempotency_receipt.v1",
                    "robot_identity": ROBOT_IDENTITY,
                    "operation_kind": "interrupt",
                    "idempotency_key": str(key),
                    "fingerprint": str(row["fingerprint"]),
                    "command_id": None,
                    "method_id": None,
                    "response": _json_load(row["receipt_json"], {}),
                }
            row = self.connection.execute("SELECT * FROM operator_plane_idempotency WHERE operation_kind=? AND idempotency_key=?", (operation_kind, key)).fetchone()
            if row is None:
                return None
            return {
                "schema_version": "bioxp.operator_idempotency_receipt.v1",
                "robot_identity": ROBOT_IDENTITY,
                "operation_kind": str(operation_kind),
                "idempotency_key": str(key),
                "fingerprint": str(row["fingerprint"]),
                "command_id": row["command_id"],
                "method_id": row["method_id"],
                "response": _json_load(row["response_json"], {}),
            }

    def method_home_established(self, method_id: str | None, method_sequence: int | None) -> bool:
        if not method_id or method_sequence is None:
            return False
        with self._lock:
            row = self.connection.execute("SELECT 1 FROM operator_plane_commands WHERE method_id=? AND method_sequence<? AND action_id='oem.z.manual_home' AND status='completed' LIMIT 1", (method_id, int(method_sequence))).fetchone()
            return row is not None

    def claim_next(self) -> dict[str, Any] | None:
        with self._transaction() as conn:
            safety = conn.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
            lane = conn.execute("SELECT * FROM operator_plane_lane WHERE singleton=1").fetchone()
            now = _now()
            if lane["owner_id"] != self.owner_id or float(lane["owner_lease_until"] or 0.0) <= now:
                return None
            queued_rows = conn.execute(
                "SELECT * FROM operator_plane_commands WHERE status='queued' ORDER BY stream_sequence"
            ).fetchall()
            row = None
            canonical = None
            for candidate in queued_rows:
                if self.action_fenced(str(candidate["action_id"])):
                    continue
                candidate_canonical = conn.execute(
                    "SELECT * FROM serial206_movement_commands WHERE command_id=?",
                    (candidate["command_id"],),
                ).fetchone()
                if candidate_canonical is None or str(candidate_canonical["state"]) != "queued":
                    continue
                if candidate["method_id"]:
                    method = conn.execute(
                        "SELECT status FROM operator_plane_methods WHERE method_id=?",
                        (candidate["method_id"],),
                    ).fetchone()
                    if method is None or str(method["status"]) not in {"queued", "running"}:
                        continue
                dependency_pending = conn.execute(
                    """
                    SELECT 1
                    FROM serial206_command_dependencies dependency
                    LEFT JOIN serial206_movement_commands prerequisite
                      ON prerequisite.command_id=dependency.depends_on_command_id
                    WHERE dependency.command_id=?
                      AND COALESCE(prerequisite.state,'missing')<>dependency.required_terminal
                    LIMIT 1
                    """,
                    (candidate["command_id"],),
                ).fetchone()
                if dependency_pending is not None:
                    continue
                resource_busy = conn.execute(
                    """
                    SELECT 1
                    FROM serial206_command_resources requested
                    JOIN serial206_command_resources active_resource
                      ON active_resource.resource_key=requested.resource_key
                    JOIN serial206_movement_commands active
                      ON active.command_id=active_resource.command_id
                    WHERE requested.command_id=?
                      AND active.command_id<>requested.command_id
                      AND active.state IN ('dispatched','issued_pending','interrupting')
                    LIMIT 1
                    """,
                    (candidate["command_id"],),
                ).fetchone()
                if resource_busy is not None:
                    continue
                row = candidate
                canonical = candidate_canonical
                break
            if row is None or canonical is None:
                return None

            if row["method_id"]:
                method = conn.execute(
                    "SELECT status FROM operator_plane_methods WHERE method_id=?",
                    (row["method_id"],),
                ).fetchone()
                if method is not None and str(method["status"]) == "queued":
                    conn.execute(
                        "UPDATE operator_plane_methods SET status='running',version=version+1,updated_at=? WHERE method_id=? AND status='queued'",
                        (now, row["method_id"]),
                    )
                    self._insert_transition(
                        conn,
                        event_kind="method_running",
                        method_id=str(row["method_id"]),
                        state="running",
                        payload={"first_child_command_id": str(row["command_id"])},
                    )

            attempt_id = str(uuid.uuid4())
            epoch = int(lane["dispatcher_epoch"])
            action_id = str(row["action_id"])
            action_axis = AXIS_BY_ACTION.get(action_id)
            axis_epoch = (
                int(safety["x_epoch"]) + int(safety["y_epoch"]) + int(safety["z_epoch"])
                if action_id.startswith("oem.xyz.") or action_id == "oem.z.scriptmove_to"
                else int(safety["x_epoch"]) + int(safety["y_epoch"])
                if action_id.startswith("oem.xy.")
                else int(safety[{"x": "x_epoch", "y": "y_epoch", "z": "z_epoch"}.get(action_axis or "", "z_epoch")])
            )
            command_claimed = conn.execute(
                "UPDATE operator_plane_commands SET status='dispatched',version=version+1,dispatch_attempt_id=?,dispatcher_epoch=?,dispatch_global_safety_epoch=?,dispatch_axis_safety_epoch=?,dispatched_at=?,updated_at=? "
                "WHERE command_id=? AND status='queued' AND version=?",
                (
                    attempt_id,
                    epoch,
                    int(safety["global_epoch"]),
                    axis_epoch,
                    now,
                    now,
                    row["command_id"],
                    int(row["version"]),
                ),
            ).rowcount
            if command_claimed != 1:
                return None
            conn.execute(
                "UPDATE serial206_movement_commands SET state='dispatched',state_version=state_version+1,dispatched_at=? WHERE command_id=? AND state='queued'",
                (now, row["command_id"]),
            )
            self._insert_transition(
                conn,
                event_kind="command_dispatched",
                command_id=str(row["command_id"]),
                method_id=row["method_id"],
                state="dispatched",
                payload={"dispatch_attempt_id": attempt_id, "dispatcher_epoch": epoch},
            )
            claimed = conn.execute(
                "SELECT * FROM operator_plane_commands WHERE command_id=?",
                (row["command_id"],),
            ).fetchone()
            assert claimed is not None
            return {
                "command_id": str(claimed["command_id"]),
                "method_id": claimed["method_id"],
                "method_sequence": claimed["method_sequence"],
                "action_id": str(claimed["action_id"]),
                "requested_inputs": _json_load(claimed["requested_json"], {}),
                "effective_inputs": _json_load(claimed["effective_json"], {}),
                "dispatch_attempt_id": str(claimed["dispatch_attempt_id"]),
                "dispatcher_epoch": int(claimed["dispatcher_epoch"]),
                "dispatch_global_safety_epoch": int(claimed["dispatch_global_safety_epoch"]),
                "dispatch_axis_safety_epoch": int(claimed["dispatch_axis_safety_epoch"]),
                "ownership_generation": int(claimed["ownership_generation"]),
                "command_version": int(claimed["version"]),
            }

    def mark_dispatched(self, command_id: str, *, payload: Mapping[str, Any], full_response: Any = None) -> dict[str, Any]:
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                return {"command_id": command_id, "status": "missing"}
            if str(row["status"]) != "dispatched":
                return self._command_response(row)
            now = _now()
            pending_payload = dict(payload)
            if full_response is not None:
                pending_payload["response_evidence"] = self._store_evidence(
                    conn,
                    command_id=command_id,
                    evidence_kind="provider_response_pending",
                    payload=full_response,
                )
            evidence = _canonical(pending_payload)
            conn.execute("UPDATE operator_plane_commands SET status='issued_pending',version=version+1,terminal_json=?,updated_at=? WHERE command_id=? AND version=? AND status='dispatched'", (evidence, now, command_id, int(row["version"])))
            conn.execute("UPDATE serial206_movement_commands SET state='issued_pending',state_version=state_version+1 WHERE command_id=? AND state='dispatched'", (command_id,))
            self._insert_transition(conn, event_kind="command_pending", command_id=command_id, method_id=row["method_id"], state="issued_pending", payload=dict(payload))
            updated = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            assert updated is not None
            result = self._command_response(updated)
        self._wake.set()
        return result

    def finish(self, command_id: str, *, status: str, payload: Mapping[str, Any], source_noop: bool = False, source_noop_reason: str | None = None, remote_acknowledged: bool = False, controller_acknowledged: bool = False, physical_effect_verified: bool = False, claimed: Mapping[str, Any] | None = None, full_response: Any = None) -> dict[str, Any]:
        if status not in COMMAND_TERMINAL:
            raise ValueError(status)
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                return {"command_id": command_id, "status": "missing"}
            if str(row["status"]) in COMMAND_TERMINAL:
                return self._command_response(row)
            if claimed is not None:
                lane = conn.execute("SELECT * FROM operator_plane_lane WHERE singleton=1").fetchone()
                now = _now()
                if (
                    str(row["status"]) != "dispatched"
                    or int(row["version"]) != int(claimed["command_version"])
                    or str(row["dispatch_attempt_id"] or "") != str(claimed["dispatch_attempt_id"])
                    or int(row["dispatcher_epoch"] or 0) != int(claimed["dispatcher_epoch"])
                    or int(row["dispatch_global_safety_epoch"] or 0) != int(claimed["dispatch_global_safety_epoch"])
                    or int(row["dispatch_axis_safety_epoch"] or 0) != int(claimed["dispatch_axis_safety_epoch"])
                    or lane is None
                    or lane["owner_id"] != self.owner_id
                    or float(lane["owner_lease_until"] or 0.0) <= now
                ):
                    return self._command_response(row)
            if full_response is not None:
                payload = {
                    **dict(payload),
                    "response_evidence": self._store_evidence(
                        conn,
                        command_id=command_id,
                        evidence_kind="provider_response",
                        payload=full_response,
                    ),
                }
            current_status = str(row["status"])
            if current_status in {"stop_requested", "abort_requested"} and status == "completed":
                status = "aborted" if current_status == "abort_requested" else "stopped"
                payload = {**dict(payload), "completion_after_interrupt_request": True}
            now = _now()
            finish_where = "command_id=? AND version=? AND status IN ('dispatched','issued_pending','stop_requested','abort_requested')"
            finish_args: tuple[Any, ...] = (command_id, int(row["version"]))
            if claimed is not None:
                finish_where += " AND dispatch_attempt_id=? AND dispatcher_epoch=? AND dispatch_global_safety_epoch=? AND dispatch_axis_safety_epoch=?"
                finish_args += (
                    str(claimed["dispatch_attempt_id"]),
                    int(claimed["dispatcher_epoch"]),
                    int(claimed["dispatch_global_safety_epoch"]),
                    int(claimed["dispatch_axis_safety_epoch"]),
                )
            changed = conn.execute(f"UPDATE operator_plane_commands SET status=?,version=version+1,source_noop=?,source_noop_reason=?,remote_acknowledged=?,controller_acknowledged=?,physical_effect_verified=?,terminal_json=?,finished_at=?,updated_at=? WHERE {finish_where}", (status, int(source_noop), source_noop_reason, int(remote_acknowledged), int(controller_acknowledged), int(physical_effect_verified), _canonical(payload), now, now, *finish_args)).rowcount
            if changed != 1:
                current = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
                return self._command_response(current) if current is not None else {"command_id": command_id, "status": "missing"}
            self._update_z_home_authority(
                conn,
                command_id=command_id,
                ownership_generation=int(row["ownership_generation"]),
                status=status,
                payload=payload,
                source_noop=source_noop,
            )
            canonical_status = {
                "stopped": "interrupted",
                "aborted": "interrupted",
                "cancelled": "cleared",
                "cleared": "cleared",
            }.get(status, status)
            conn.execute(
                "UPDATE serial206_movement_commands SET state=?,state_version=state_version+1,finished_at=?,terminal_receipt_id=? WHERE command_id=? AND state NOT IN ('completed','failed','cleared','interrupted','ambiguous','rejected')",
                (canonical_status, now, str(payload.get("receipt_id")) if payload.get("receipt_id") is not None else None, command_id),
            )
            transition = self._insert_transition(conn, event_kind="command_terminal", command_id=command_id, method_id=row["method_id"], state=status, payload={"source_noop": source_noop, "reason": source_noop_reason, **dict(payload)})
            conn.execute("INSERT INTO operator_plane_outbox(outbox_id,command_id,transition_sequence,state,payload_json,updated_at) VALUES(?,?,?,?,?,?)", (str(uuid.uuid4()), command_id, transition, "pending", _canonical(_bounded_json(payload, 131072)), _now()))
            method_id = row["method_id"]

            if method_id:
                self._derive_method(conn, str(method_id))
            updated = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            assert updated is not None
            result = self._command_response(updated, transition_sequence=transition)
        self._wake.set()
        return result

    def _derive_method(self, conn: sqlite3.Connection, method_id: str) -> None:
        method = conn.execute("SELECT * FROM operator_plane_methods WHERE method_id=?", (method_id,)).fetchone()
        if method is None:
            return
        rows = conn.execute("SELECT status FROM operator_plane_commands WHERE method_id=? ORDER BY method_sequence", (method_id,)).fetchall()
        states = [str(row[0]) for row in rows]
        current = str(method["status"])
        if any(state in {"ambiguous", "interrupted"} for state in states):
            target = "recovery_required"
        elif any(state == "failed" for state in states):
            target = "failed"
        elif current == "pause_requested" and not any(state in {"dispatched", "stop_requested", "abort_requested"} for state in states):
            target = "paused"
        elif current in {"cancel_requested", "stopping", "aborting"} and all(state in COMMAND_TERMINAL for state in states):
            target = "cancelled" if current == "cancel_requested" else ("stopped" if current == "stopping" else "aborted")
        elif states and all(state == "completed" for state in states):
            target = "completed"
        elif states and all(state in COMMAND_TERMINAL for state in states) and any(
            state in {"stopped", "aborted", "cleared"} for state in states
        ):
            target = "interrupted"
        elif any(state == "dispatched" for state in states):
            target = "running"
        else:
            target = current
        if target in {"failed", "cancelled", "stopped", "aborted"}:
            child_state = "cancelled" if target in {"failed", "cancelled"} else "cleared"
            reason = "method_fail_fast" if target == "failed" else f"method_{target}"
            pending_children = conn.execute(
                "SELECT command_id FROM operator_plane_commands WHERE method_id=? AND status='queued' ORDER BY method_sequence",
                (method_id,),
            ).fetchall()
            conn.execute("UPDATE operator_plane_commands SET status=?,version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE method_id=? AND status='queued'", (child_state, _now(), _now(), _canonical({"reason": reason}), method_id))
            conn.execute("UPDATE serial206_movement_commands SET state='cleared',state_version=state_version+1,finished_at=? WHERE method_id=? AND state='queued'", (_now(), method_id))
            for pending_child in pending_children:
                child_id = str(pending_child[0])
                terminal_payload = {"reason": reason, "method_id": method_id, "delivery_attempted": False}
                transition = self._insert_transition(conn, event_kind="method_sibling_terminal", command_id=child_id, method_id=method_id, state=child_state, payload=terminal_payload)
                conn.execute(
                    "INSERT INTO operator_plane_outbox(outbox_id,command_id,transition_sequence,state,payload_json,updated_at) VALUES(?,?,?,?,?,?)",
                    (str(uuid.uuid4()), child_id, transition, "pending", _canonical(terminal_payload), _now()),
                )
        if target != current and target in METHOD_STATES:
            conn.execute("UPDATE operator_plane_methods SET status=?,version=version+1,updated_at=? WHERE method_id=?", (target, _now(), method_id))
            canonical_target = {
                "queued": "queued",
                "running": "active",
                "pause_requested": "active",
                "paused": "active",
                "cancel_requested": "active",
                "stopping": "active",
                "aborting": "active",
                "recovery_required": "ambiguous",
                "completed": "completed",
                "failed": "failed",
                "cancelled": "cleared",
                "stopped": "interrupted",
                "aborted": "interrupted",
                "interrupted": "interrupted",
            }[target]
            conn.execute(
                "UPDATE serial206_movement_methods SET state=?,state_version=state_version+1,started_at=CASE WHEN ?='active' THEN COALESCE(started_at,?) ELSE started_at END,finished_at=CASE WHEN ? IN ('completed','failed','cleared','interrupted','ambiguous') THEN ? ELSE finished_at END WHERE method_id=?",
                (canonical_target, canonical_target, _now(), canonical_target, _now(), method_id),
            )
            self._insert_transition(conn, event_kind="method_derived", method_id=method_id, state=target, payload={"child_states": states})

    def begin_interrupt(self, action_id: str, *, state: Mapping[str, Any], request: Mapping[str, Any], interrupt_attempt_id: str | None = None) -> dict[str, Any]:
        if action_id not in INTERRUPT_ACTIONS:
            raise HTTPException(status_code=422, detail={"error": "interrupt_action_not_allowed"})
        key = str(request["idempotency_key"])
        fp = _digest({"operation_kind": "interrupt", "action_id": action_id, **_without_idempotency(request)})
        with self._lock:
            saved_attempt = self._latest_interrupt_attempt(
                self.connection,
                idempotency_key=key,
            )
        if saved_attempt is not None and str(saved_attempt["fingerprint"]) != fp:
            raise HTTPException(
                status_code=409,
                detail={
                    "error": "idempotency_conflict",
                    "operation_kind": "interrupt",
                    "idempotency_key": key,
                },
            )
        replay = saved_attempt is not None
        aggregate_interrupt = action_id in {"oem.abort_all", "oem.z.abort"}
        if aggregate_interrupt:
            self._priority_fence.set()
        with self._interrupt_lock:
            timeout_ms = 5 if action_id == "oem.y.stop" else 25
            try:
                with self._transaction(timeout_ms=timeout_ms) as conn:
                    safety = conn.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
                    interrupt_id = str(interrupt_attempt_id or uuid.uuid4())
                    existing_attempt = self._latest_interrupt_attempt(
                        conn,
                        interrupt_attempt_id=interrupt_id,
                    )
                    if existing_attempt is not None:
                        if (
                            str(existing_attempt["idempotency_key"]) != key
                            or str(existing_attempt["fingerprint"]) != fp
                            or str(existing_attempt["action_id"]) != action_id
                        ):
                            raise RuntimeError("interrupt attempt identity conflicts with durable history")
                        existing_receipt = _json_load(existing_attempt["receipt_json"], {})
                        if not isinstance(existing_receipt, dict):
                            raise RuntimeError("interrupt attempt receipt is invalid")
                        existing_receipt["idempotent_replay"] = True
                        return existing_receipt
                    axis = AXIS_BY_ACTION.get(action_id)
                    active_rows = conn.execute(
                        "SELECT DISTINCT c.command_id FROM serial206_movement_commands c LEFT JOIN serial206_command_resources r ON r.command_id=c.command_id WHERE c.state IN ('dispatched','issued_pending') AND (? OR r.resource_key=?)",
                        (int(aggregate_interrupt), f"axis:{axis}"),
                    ).fetchall()
                    active_ids = [str(item[0]) for item in active_rows]
                    active_id = active_ids[0] if active_ids else None
                    global_epoch = int(safety["global_epoch"]) + (1 if action_id in {"oem.abort_all", "oem.z.abort"} else 0)
                    x_epoch = int(safety["x_epoch"]) + (1 if action_id == "oem.x.stop" else 0)
                    y_epoch = int(safety["y_epoch"]) + (1 if action_id == "oem.y.stop" else 0)
                    z_epoch = int(safety["z_epoch"]) + (1 if action_id == "oem.z.stop" else 0)
                    if action_id in {"oem.abort_all", "oem.z.abort"}:
                        x_epoch += 1
                        y_epoch += 1
                        z_epoch += 1
                    cutoff = int(conn.execute("SELECT COALESCE(MAX(stream_sequence),0) FROM operator_plane_commands").fetchone()[0])
                    conn.execute("UPDATE operator_plane_safety SET global_epoch=?,x_epoch=?,y_epoch=?,z_epoch=?,recovery_version=recovery_version+1,updated_at=? WHERE singleton=1", (global_epoch, x_epoch, y_epoch, z_epoch, _now()))
                    if aggregate_interrupt:
                        queued_rows = conn.execute(
                            "SELECT command_id,method_id FROM operator_plane_commands WHERE stream_sequence<=? AND status='queued' ORDER BY stream_sequence",
                            (cutoff,),
                        ).fetchall()
                    else:
                        queued_rows = conn.execute(
                            "SELECT DISTINCT c.command_id,c.method_id FROM operator_plane_commands c JOIN serial206_command_resources r ON r.command_id=c.command_id WHERE c.stream_sequence<=? AND c.status='queued' AND r.resource_key=? ORDER BY c.stream_sequence",
                            (cutoff, f"axis:{axis}"),
                        ).fetchall()
                    queued_by_id = {str(row["command_id"]): row["method_id"] for row in queued_rows}
                    dependency_roots = [*active_ids, *queued_by_id.keys()]
                    if not aggregate_interrupt and dependency_roots:
                        placeholders = ",".join("?" for _ in dependency_roots)
                        dependent_rows = conn.execute(
                            f"""
                            WITH RECURSIVE dependent(command_id) AS (
                                SELECT command_id FROM serial206_command_dependencies
                                WHERE depends_on_command_id IN ({placeholders})
                                UNION
                                SELECT dependency.command_id
                                FROM serial206_command_dependencies dependency
                                JOIN dependent parent ON dependency.depends_on_command_id=parent.command_id
                            )
                            SELECT command.command_id,command.method_id
                            FROM operator_plane_commands command
                            JOIN dependent ON dependent.command_id=command.command_id
                            WHERE command.status='queued' AND command.stream_sequence<=?
                            ORDER BY command.stream_sequence
                            """,
                            (*dependency_roots, cutoff),
                        ).fetchall()
                        for dependent in dependent_rows:
                            queued_by_id[str(dependent["command_id"])] = dependent["method_id"]
                    affected_method_ids = {
                        str(method_id) for method_id in queued_by_id.values() if method_id
                    }
                    for active_method in conn.execute("SELECT method_id FROM operator_plane_commands WHERE command_id IN (%s)" % ",".join("?" for _ in active_ids), tuple(active_ids)).fetchall() if active_ids else []:
                        if active_method[0]:
                            affected_method_ids.add(str(active_method[0]))
                    reason_json = _canonical({"reason": action_id, "cutoff": cutoff})
                    now = _now()
                    for queued_id, method_id in queued_by_id.items():
                        conn.execute(
                            "UPDATE operator_plane_commands SET status='cleared',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE command_id=? AND status='queued'",
                            (now, now, reason_json, queued_id),
                        )
                        conn.execute(
                            "UPDATE serial206_movement_commands SET state='cleared',state_version=state_version+1,finished_at=? WHERE command_id=? AND state='queued'",
                            (now, queued_id),
                        )
                        terminal_payload = {"reason": action_id, "cutoff": cutoff, "delivery_attempted": False}
                        transition_sequence = self._insert_transition(
                            conn,
                            event_kind="interrupt_cleared_queued",
                            command_id=queued_id,
                            method_id=method_id,
                            state="cleared",
                            payload=terminal_payload,
                        )
                        conn.execute(
                            "INSERT INTO operator_plane_outbox(outbox_id,command_id,transition_sequence,state,payload_json,updated_at) VALUES(?,?,?,?,?,?)",
                            (str(uuid.uuid4()), queued_id, transition_sequence, "pending", _canonical(terminal_payload), now),
                        )
                    if aggregate_interrupt:
                        for method_id in affected_method_ids:
                            conn.execute("UPDATE operator_plane_methods SET status='aborting',version=version+1,updated_at=? WHERE method_id=? AND status NOT IN ('completed','failed','cancelled','stopped','aborted','interrupted','recovery_required')", (now, method_id))
                            self._insert_transition(conn, event_kind="method_derived", method_id=method_id, state="aborting", payload={"reason": action_id, "cutoff": cutoff})
                    if aggregate_interrupt:
                        conn.execute(
                            "UPDATE serial206_axis_authority SET interrupt_epoch=interrupt_epoch+1,lifecycle_state='reconciliation_required',reference_state='reconciliation_required',prepared_board_epoch=NULL,state_version=state_version+1,updated_at=? WHERE axis IN ('y','z','gripper')",
                            (now,),
                        )
                    elif axis in {"y", "z"}:
                        conn.execute("UPDATE serial206_axis_authority SET interrupt_epoch=interrupt_epoch+1,state_version=state_version+1,updated_at=? WHERE axis=?", (now, axis))
                    transition = None
                    for active_id in active_ids:
                        row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (str(active_id),)).fetchone()
                        if row is None or str(row["status"]) not in {"dispatched", "issued_pending", "stop_requested", "abort_requested"}:
                            continue
                        requested_state = "abort_requested" if aggregate_interrupt else "stop_requested"
                        conn.execute("UPDATE operator_plane_commands SET status=?,version=version+1,interrupt_id=?,interrupt_global_safety_epoch=?,interrupt_axis_safety_epoch=?,updated_at=?,terminal_json=? WHERE command_id=? AND status IN ('dispatched','issued_pending')", (requested_state, interrupt_id, global_epoch, {"x": x_epoch, "y": y_epoch, "z": z_epoch}.get(axis or "", z_epoch), _now(), _canonical({"interrupt_id": interrupt_id, "action_id": action_id}), str(active_id)))
                        conn.execute("UPDATE serial206_movement_commands SET state='interrupting',state_version=state_version+1 WHERE command_id=? AND state IN ('dispatched','issued_pending')", (str(active_id),))
                        transition = self._insert_transition(conn, event_kind="interrupt_requested", command_id=str(active_id), method_id=row["method_id"], state=requested_state, payload={"interrupt_id": interrupt_id, "action_id": action_id, "cutoff": cutoff})
                    for method_id in affected_method_ids:
                        self._derive_method(conn, method_id)
                    active_id = active_ids[0] if active_ids else None
                    response = {"schema_version": "bioxp.operator_interrupt_receipt.v1", "robot_identity": ROBOT_IDENTITY, "ownership_generation": int(state.get("ownership_generation") or 0), "interrupt_id": interrupt_id, "interrupt_attempt_id": interrupt_id, "action_id": action_id, "scope": "aggregate" if action_id in {"oem.abort_all", "oem.z.abort"} else axis, "cutoff": cutoff, "active_command_id": active_id, "active_command_ids": active_ids, "global_safety_epoch": global_epoch, "x_safety_epoch": x_epoch, "y_safety_epoch": y_epoch, "z_safety_epoch": z_epoch, "oem_abort_latched": action_id in {"oem.abort_all", "oem.z.abort"}, "controller_stop_attempted": False, "source_call_completed": False, "source_return_ok": False, "controller_stop_acknowledged": False, "physical_effect_verified": False, "persistence_state": "committed", "transition_sequence": transition, "idempotent_replay": replay}
                    response.update({"observed_ownership_generation": request.get("observed_ownership_generation"), "observed_board_epoch_by_board": dict(request.get("observed_board_epoch_by_board") or {}), "recovery_hold": False, "terminal_transition_sequences": []})
                    self._append_interrupt_attempt(
                        conn,
                        interrupt_attempt_id=interrupt_id,
                        idempotency_key=key,
                        fingerprint=fp,
                        action_id=action_id,
                        phase="admitted",
                        receipt=response,
                    )
            except sqlite3.OperationalError as exc:
                if "locked" not in str(exc).lower() and "busy" not in str(exc).lower():
                    raise
                failed_attempt_id = str(interrupt_attempt_id or uuid.uuid4())
                return {"schema_version": "bioxp.operator_interrupt_receipt.v1", "robot_identity": ROBOT_IDENTITY, "ownership_generation": int(state.get("ownership_generation") or 0), "observed_ownership_generation": request.get("observed_ownership_generation"), "observed_board_epoch_by_board": dict(request.get("observed_board_epoch_by_board") or {}), "interrupt_id": failed_attempt_id, "interrupt_attempt_id": failed_attempt_id, "action_id": action_id, "scope": "aggregate" if action_id in {"oem.abort_all", "oem.z.abort"} else AXIS_BY_ACTION.get(action_id), "oem_abort_latched": action_id in {"oem.abort_all", "oem.z.abort"}, "controller_stop_attempted": False, "source_call_completed": False, "source_return_ok": False, "controller_stop_acknowledged": False, "physical_effect_verified": False, "persistence_state": "lock_timeout", "recovery_hold": True, "idempotent_replay": replay}
        self._wake.set()
        return response

    def mark_interrupt_attempted(self, *, idempotency_key: str) -> dict[str, Any]:
        with self._transaction(timeout_ms=5) as conn:
            saved = self._latest_interrupt_attempt(
                conn,
                idempotency_key=str(idempotency_key),
            )
            if saved is None:
                raise RuntimeError("interrupt admission receipt is missing")
            current = _json_load(saved["receipt_json"], {})
            if not isinstance(current, dict):
                raise RuntimeError("interrupt admission receipt is invalid")
            if str(saved["phase"]) in {"attempted", "terminal"} or current.get("controller_stop_attempted") is True:
                current["idempotent_replay"] = True
                return current
            current.update({
                "controller_stop_attempted": True,
                "source_call_completed": False,
                "source_return_ok": False,
                "controller_stop_acknowledged": False,
                "controller_response": None,
                "error": "controller_stop_outcome_pending",
                "persistence_state": "recovery_required",
                "recovery_hold": True,
            })
            self._append_interrupt_attempt(
                conn,
                interrupt_attempt_id=str(saved["interrupt_attempt_id"]),
                idempotency_key=str(saved["idempotency_key"]),
                fingerprint=str(saved["fingerprint"]),
                action_id=str(saved["action_id"]),
                phase="attempted",
                receipt=current,
            )
        return current

    def finalize_interrupt(self, *, idempotency_key: str, receipt: Mapping[str, Any], attempted: bool, acknowledged: bool, response: Any, error: str | None = None) -> dict[str, Any]:
        interrupt_id = str(receipt.get("interrupt_id") or "")
        interrupt_attempt_id = str(receipt.get("interrupt_attempt_id") or interrupt_id)
        active_id = receipt.get("active_command_id")
        active_ids = [str(value) for value in receipt.get("active_command_ids", []) if value]
        if active_id and str(active_id) not in active_ids:
            active_ids.append(str(active_id))
        terminal_state = "aborted" if bool(receipt.get("oem_abort_latched")) else "stopped"
        with self._transaction() as conn:
            saved = self._latest_interrupt_attempt(
                conn,
                interrupt_attempt_id=interrupt_attempt_id,
            )
            if saved is None:
                raise RuntimeError("interrupt admission receipt is missing")
            if str(saved["idempotency_key"]) != str(idempotency_key):
                raise RuntimeError("interrupt idempotency identity conflicts with durable attempt")
            current = _json_load(saved["receipt_json"], dict(receipt))
            if not isinstance(current, dict):
                raise RuntimeError("interrupt admission receipt is invalid")
            if str(saved["phase"]) == "terminal":
                current["idempotent_replay"] = True
                return current
            controller_acknowledged = bool(isinstance(response, Mapping) and response.get("controller_command_acknowledged") is True)
            source_return_ok = bool(isinstance(response, Mapping) and response.get("ok") is True)
            exact_response_evidence = self._store_interrupt_evidence(
                conn,
                interrupt_attempt_id=interrupt_attempt_id,
                action_id=str(receipt.get("action_id") or "unknown"),
                evidence_kind="controller_response",
                payload=response,
            )
            current.update({
                "interrupt_attempt_id": interrupt_attempt_id,
                "controller_stop_attempted": bool(attempted),
                "source_call_completed": bool(acknowledged),
                "source_return_ok": source_return_ok,
                "controller_stop_acknowledged": controller_acknowledged,
                "physical_effect_verified": False,
                "controller_response": _bounded_json(response, 131072),
                "controller_response_evidence": exact_response_evidence,
                "error": error,
                "persistence_state": "committed",
                "recovery_hold": False,
            })
            affected_method_ids: set[str] = set()
            for active_id in active_ids:
                row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (str(active_id),)).fetchone()
                if row is not None and str(row["status"]) in {"dispatched", "stop_requested", "abort_requested"}:
                    final_state = terminal_state if source_return_ok else "ambiguous"
                    terminal_payload = {"interrupt_id": interrupt_id, "source_call_completed": bool(acknowledged), "source_return_ok": source_return_ok, "controller_acknowledged": controller_acknowledged, "error": error}
                    conn.execute("UPDATE operator_plane_commands SET status=?,version=version+1,finished_at=?,updated_at=?,terminal_json=?,controller_acknowledged=? WHERE command_id=?", (final_state, _now(), _now(), _canonical(terminal_payload), int(controller_acknowledged), str(active_id)))
                    if row["method_id"]:
                        affected_method_ids.add(str(row["method_id"]))
                    self._update_z_home_authority(conn, command_id=str(active_id), ownership_generation=int(row["ownership_generation"]), status=final_state, payload=terminal_payload, source_noop=False)
                    canonical_state = "interrupted" if source_return_ok else "ambiguous"
                    conn.execute("UPDATE serial206_movement_commands SET state=?,state_version=state_version+1,finished_at=?,terminal_receipt_id=? WHERE command_id=? AND state='interrupting'", (canonical_state, _now(), interrupt_id, str(active_id)))
                    transition = self._insert_transition(conn, event_kind="interrupt_terminal", command_id=str(active_id), method_id=row["method_id"], state=final_state, payload={"interrupt_id": interrupt_id, "source_call_completed": bool(acknowledged), "source_return_ok": source_return_ok, "controller_acknowledged": controller_acknowledged, "error": error})
                    current.setdefault("terminal_transition_sequences", []).append(transition)
                    conn.execute("INSERT INTO operator_plane_outbox(outbox_id,command_id,transition_sequence,state,payload_json,updated_at) VALUES(?,?,?,?,?,?)", (str(uuid.uuid4()), str(active_id), transition, "pending", _canonical(terminal_payload), _now()))
            for method_id in affected_method_ids:
                self._derive_method(conn, method_id)
            self._store_interrupt_evidence(
                conn,
                interrupt_attempt_id=interrupt_attempt_id,
                action_id=str(receipt.get("action_id") or "unknown"),
                evidence_kind="interrupt_receipt",
                payload=current,
            )
            self._append_interrupt_attempt(
                conn,
                interrupt_attempt_id=interrupt_attempt_id,
                idempotency_key=str(saved["idempotency_key"]),
                fingerprint=str(saved["fingerprint"]),
                action_id=str(saved["action_id"]),
                phase="terminal",
                receipt=current,
            )
        if str(current.get("action_id")) in {"oem.abort_all", "oem.z.abort"}:
            self.clear_priority_fence()
        self._wake.set()
        return current

    def clear_priority_fence(self) -> None:
        self._priority_fence.clear()

    @staticmethod
    def _axes_for_action(action_id: str) -> set[str]:
        if action_id in {"oem.xy.move_absolute", "oem.xy.home"}:
            return {"x", "y"}
        axis = AXIS_BY_ACTION.get(action_id)
        return {axis} if axis in {"x", "y", "z"} else set()

    def arm_interrupt_fence(self, action_id: str) -> None:
        if action_id in {"oem.abort_all", "oem.z.abort"}:
            self._priority_fence.set()
        else:
            axis = AXIS_BY_ACTION.get(action_id)
            if axis in self._axis_priority_fences:
                self._axis_priority_fences[axis].set()
        self._wake.set()

    def clear_interrupt_fence(self, action_id: str) -> None:
        if action_id in {"oem.abort_all", "oem.z.abort"}:
            self._priority_fence.clear()
        else:
            axis = AXIS_BY_ACTION.get(action_id)
            if axis in self._axis_priority_fences:
                self._axis_priority_fences[axis].clear()
        self._wake.set()

    def action_fenced(self, action_id: str) -> bool:
        if self._priority_fence.is_set():
            return True
        return any(self._axis_priority_fences[axis].is_set() for axis in self._axes_for_action(action_id))

    def queue_pending_interrupt_reconciliation(self, record: Mapping[str, Any]) -> None:
        pending = dict(record)
        attempt_id = str(pending.get("interrupt_attempt_id") or "").strip()
        action_id = str(pending.get("action_id") or "").strip()
        request = pending.get("request")
        if not attempt_id or action_id not in INTERRUPT_ACTIONS or not isinstance(request, Mapping):
            raise ValueError("pending interrupt reconciliation identity is invalid")
        pending["interrupt_attempt_id"] = attempt_id
        pending["action_id"] = action_id
        pending["state"] = dict(pending.get("state") or {})
        pending["request"] = dict(request)
        self._append_interrupt_spool_event(phase="pending", payload=pending)
        with self._pending_interrupt_lock:
            self._pending_interrupt_reconciliations = [
                row for row in self._pending_interrupt_reconciliations
                if str(row.get("interrupt_attempt_id")) != attempt_id
            ]
            self._pending_interrupt_reconciliations.append(pending)
        self._wake.set()

    def reconcile_pending_interrupts(self) -> int:
        pending = self._pending_interrupt_spool_rows()
        completed: set[str] = set()
        for row in pending:
            attempt_id = str(row["interrupt_attempt_id"])
            try:
                receipt = self.begin_interrupt(
                    str(row["action_id"]),
                    state=dict(row.get("state") or {}),
                    request=dict(row["request"]),
                    interrupt_attempt_id=attempt_id,
                )
                if str(receipt.get("persistence_state")) != "committed":
                    continue
                terminal_receipt = self.finalize_interrupt(
                    idempotency_key=str(dict(row["request"])["idempotency_key"]),
                    receipt={**dict(receipt), "interrupt_attempt_id": attempt_id},
                    attempted=bool(row.get("attempted", True)),
                    acknowledged=bool(row.get("acknowledged", False)),
                    response=row.get("response"),
                    error=str(row["error"]) if row.get("error") is not None else None,
                )
                self._append_interrupt_spool_event(
                    phase="reconciled",
                    payload={
                        "interrupt_attempt_id": attempt_id,
                        "terminal_receipt": terminal_receipt,
                    },
                )
            except (KeyError, RuntimeError, sqlite3.Error, ValueError):
                continue
            completed.add(attempt_id)
        if completed:
            with self._pending_interrupt_lock:
                self._pending_interrupt_reconciliations = [
                    row for row in self._pending_interrupt_reconciliations
                    if str(row.get("interrupt_attempt_id")) not in completed
                ]
        return len(completed)


    def start(self, dispatch_one: Callable[[dict[str, Any]], None]) -> None:
        if not self._owner_acquired:
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._dispatch_loop, args=(dispatch_one,), daemon=True, name="bioxp-operator-dispatcher")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        with self._worker_lock:
            workers = list(self._workers)
        for worker in workers:
            worker.join(timeout=1.0)
        self.reconcile_pending_interrupts()
        if self._owner_acquired:
            try:
                with self._transaction(timeout_ms=100) as conn:
                    conn.execute("UPDATE operator_plane_lane SET owner_id=NULL,owner_lease_until=NULL,updated_at=? WHERE singleton=1 AND owner_id=?", (_now(), self.owner_id))
            except sqlite3.OperationalError:
                pass

    def _dispatch_loop(self, dispatch_one: Callable[[dict[str, Any]], None]) -> None:
        next_renewal = 0.0
        while not self._stop.is_set():
            self.reconcile_pending_interrupts()
            if _now() >= next_renewal:
                try:
                    if not self._renew_owner():
                        self._priority_fence.set()
                        self._stop.set()
                        break
                except sqlite3.OperationalError:
                    self._priority_fence.set()
                    self._stop.set()
                    break
                next_renewal = _now() + 1.0

            spawned = False
            claimed: dict[str, Any] | None = None
            try:
                while not self._priority_fence.is_set() and not self._stop.is_set():
                    with self._worker_lock:
                        self._workers = {worker for worker in self._workers if worker.is_alive()}
                        if len(self._workers) >= 8:
                            break
                    claimed = self.claim_next()
                    if claimed is None:
                        break
                    worker = threading.Thread(
                        target=self._dispatch_worker,
                        args=(dispatch_one, claimed),
                        daemon=True,
                        name=f"bioxp-operator-command-{claimed['command_id']}",
                    )
                    with self._worker_lock:
                        self._workers.add(worker)
                    worker.start()
                    spawned = True
            except Exception:
                if claimed is not None:
                    try:
                        self.finish(
                            claimed["command_id"],
                            status="ambiguous",
                            payload={"error": "dispatcher_exception", "outcome_unknown": True},
                            claimed=claimed,
                        )
                    except Exception:
                        pass
            if not spawned:
                self._wake.wait(timeout=0.05)
                self._wake.clear()

    def _dispatch_worker(self, dispatch_one: Callable[[dict[str, Any]], None], claimed: dict[str, Any]) -> None:
        try:
            dispatch_one(claimed)
        except Exception:
            try:
                self.finish(claimed["command_id"], status="ambiguous", payload={"error": "dispatcher_exception", "outcome_unknown": True}, claimed=claimed)
            except Exception:
                pass
        finally:
            with self._worker_lock:
                self._workers.discard(threading.current_thread())


class OperatorCommandPlane:
    def __init__(self, app: FastAPI, *, machine_state_provider: Callable[[], Mapping[str, Any]], actions: list[dict[str, Any]], dispatch: Mapping[str, Mapping[str, Any]]) -> None:
        self.app = app
        self.machine_state_provider = machine_state_provider
        self.actions = actions
        self.dispatch = dispatch
        self.by_id = {str(row["action_id"]): row for row in actions}
        self.store = OperatorCommandStore()
        self.router = APIRouter(prefix="/operator", tags=["operator-command-plane"])
        self._install_routes()

    def start(self) -> None:
        self.store.start(self._dispatch_one)

    def is_canonical(self, action_id: str) -> bool:
        return action_id in CANONICAL_ACTIONS

    def canonical_xz_target_action(self, action_id: str, inputs: Mapping[str, Any] | None = None) -> str | None:
        """Map a normal X/Z target to the sole v2 action that may own it."""
        if action_id in INTERRUPT_ACTIONS:
            return None
        if action_id in XZ_NORMAL_ACTIONS:
            return action_id
        target = self.dispatch.get(action_id)
        if not isinstance(target, Mapping) or str(target.get("method") or "").upper() != "POST":
            return None
        path = str(target.get("path") or "").lower()
        canonical_action_id = XZ_CANONICAL_ACTION_BY_TARGET_PATH.get(path)
        if canonical_action_id is not None:
            return canonical_action_id
        effective = {**dict(target.get("fixed_inputs") or {}), **dict(inputs or {})}
        axis = effective.get("axis")
        body = effective.get("body")
        if axis is None and isinstance(body, Mapping):
            axis = body.get("axis")
        return XZ_CANONICAL_ACTION_BY_AXIS_TARGET.get((path, str(axis or "").strip().lower()))

    def _state(self) -> dict[str, Any]:
        value = self.machine_state_provider()
        return dict(value) if isinstance(value, Mapping) else {}

    def _current_assessment(
        self,
        action_id: str,
        inputs: Mapping[str, Any],
        state: Mapping[str, Any],
    ) -> dict[str, Any]:
        action = self.by_id.get(action_id)
        if not isinstance(action, Mapping):
            raise HTTPException(
                status_code=503,
                detail={"error": "action_assessment_authority_unavailable", "action_id": action_id},
            )
        target = self.dispatch.get(action_id)
        fixed_inputs = dict(target.get("fixed_inputs") or {}) if isinstance(target, Mapping) else {}
        effective = _effective_inputs(action_id, {**fixed_inputs, **dict(inputs)}, state)
        return _assess_action(action, state, effective)

    def _action_target(self, action_id: str) -> Mapping[str, Any]:
        target = self.dispatch.get(action_id)
        if not isinstance(target, Mapping):
            raise HTTPException(status_code=503, detail={"error": "canonical_provider_unavailable", "action_id": action_id})
        return target


    def _terminalize_y_pending(
        self,
        command_id: str,
        pending_response: Mapping[str, Any],
        claimed: Mapping[str, Any],
    ) -> None:
        token = _DISPATCH_CONTEXT.set({
            "operator_command_id": command_id,
            "idempotency_key": f"terminalize:{claimed['dispatch_attempt_id']}",
            "expected_ownership_generation": claimed["ownership_generation"],
            "action_id": "oem.y.move_absolute",
        })
        try:
            terminalizer = getattr(self.app.state, "serial206_y_terminalizer", None)
            if not callable(terminalizer):
                raise RuntimeError("serial206_y_terminalizer_not_bound")
            terminalized = terminalizer(dict(pending_response), 20.0)
            if not isinstance(terminalized, Mapping):
                raise RuntimeError("serial206_y_terminalizer_returned_invalid_payload")
            response = dict(terminalized)
            ok = response.get("ok") is True
            payload = {
                "receipt_id": f"terminal:{command_id}",
                "http_status": 200,
                "response": _bounded_json(response, 131072),
                "event_128": _addressed_event_proven(response),
                "position_readback": _position_readback_proven(response),
                "observed_position_steps": _terminal_position_steps(response),
                "effective_target_steps": pending_response.get("target_steps"),
                "home_completion_proven": False,
                "dispatch_attempt_id": claimed["dispatch_attempt_id"],
                "completion_class": _state_value(response, "completion_class"),
                "terminal_speed_zero": _state_value(response, "terminal_speed_zero", "speed_zero") is True,
            }
            self.store.finish(
                command_id,
                status="completed" if ok else "failed",
                payload=payload,
                remote_acknowledged=ok,
                controller_acknowledged=_controller_acknowledged(response),
                claimed=None,
                full_response=response,
            )
        except Exception as exc:
            self.store.finish(
                command_id,
                status="ambiguous",
                payload={"error": f"y_terminalizer_exception:{type(exc).__name__}", "outcome_unknown": True},
                claimed=None,
            )
        finally:
            _DISPATCH_CONTEXT.reset(token)
            with self.store._worker_lock:
                self.store._workers.discard(threading.current_thread())

    def _dispatch_one(self, claimed: dict[str, Any]) -> None:
        command_id = str(claimed["command_id"])
        action_id = str(claimed["action_id"])
        state = self._state()
        requested = dict(claimed["requested_inputs"])
        effective = dict(claimed["effective_inputs"])

        if self.store.action_fenced(action_id):
            self.store.finish(command_id, status="interrupted", payload={"reason": "interrupt_fence_won_before_provider"}, claimed=claimed)
            return
        target = self._action_target(action_id)
        token = _DISPATCH_CONTEXT.set({"operator_command_id": command_id, "idempotency_key": f"dispatch:{claimed['dispatch_attempt_id']}", "expected_ownership_generation": claimed["ownership_generation"], "action_id": action_id})
        try:
            status_code, response = asyncio.run(_dispatch_asgi(self.app, str(target["method"]), str(target["path"]), {**dict(target.get("fixed_inputs") or {}), **effective}, target["locations"]))
        except Exception as exc:
            self.store.finish(command_id, status="ambiguous", payload={"error": f"provider_exception:{type(exc).__name__}", "detail": str(exc)[:500], "outcome_unknown": True}, claimed=claimed)
            return
        finally:
            _DISPATCH_CONTEXT.reset(token)
        finish_claim: Mapping[str, Any] | None = claimed
        if (
            action_id == "oem.y.move_absolute"
            and isinstance(response, Mapping)
            and response.get("ok") is True
            and response.get("state") == "issued_pending"
        ):
            self.store.mark_dispatched(
                command_id,
                payload={
                    "receipt_id": f"pending:{command_id}",
                    "response": _bounded_json(response, 131072),
                    "completion_class": "issued_pending",
                    "physical_effect_verified": False,
                },
                full_response=response,
            )
            terminalizer_worker = threading.Thread(
                target=self._terminalize_y_pending,
                args=(command_id, dict(response), dict(claimed)),
                daemon=True,
                name=f"bioxp-y-terminalizer-{command_id}",
            )
            with self.store._worker_lock:
                self.store._workers.add(terminalizer_worker)
            terminalizer_worker.start()
            return
        ok = 200 <= int(status_code) < 300 and not (isinstance(response, Mapping) and response.get("ok") is False)
        event128 = _addressed_event_proven(response)
        position_readback = _position_readback_proven(response)
        home_proof = _home_completion_proven(response)
        controller_ack = _controller_acknowledged(response)
        completion_class = _state_value(response, "completion_class")
        terminal_speed_zero = _state_value(response, "terminal_speed_zero", "speed_zero") is True
        observed_position_steps = _terminal_position_steps(response)
        effective_target_steps = effective.get("target_steps") if action_id == "oem.y.move_absolute" else (
            observed_position_steps if action_id == "oem.y.move_steps" and position_readback else None
        )
        event_sequence = _state_value(response, "controller_event_sequence", "event_sequence", "event_cursor_sequence")
        payload = {"receipt_id": f"terminal:{command_id}", "http_status": int(status_code), "response": _bounded_json(response, 131072), "event_128": event128, "position_readback": position_readback, "observed_position_steps": observed_position_steps, "effective_target_steps": effective_target_steps, "home_completion_proven": home_proof, "dispatch_attempt_id": claimed["dispatch_attempt_id"], "controller_event_sequence": event_sequence if type(event_sequence) is int and event_sequence >= 0 else None}
        source_completed_noop = bool(
            isinstance(response, Mapping)
            and response.get("state") == "completed"
            and isinstance(response.get("result"), Mapping)
            and response["result"].get("source_noop") is True
            and response["result"].get("command_sent") is False
        )
        if ok:
            self.store.finish(command_id, status="completed", payload={**payload, "completion_class": completion_class, "terminal_speed_zero": terminal_speed_zero}, source_noop=source_completed_noop, source_noop_reason="oem_same_effective_target_noop" if source_completed_noop else None, remote_acknowledged=True, controller_acknowledged=controller_ack, claimed=finish_claim, full_response=response)
        else:
            self.store.finish(command_id, status="failed", payload={**payload, "completion_class": completion_class, "terminal_speed_zero": terminal_speed_zero}, remote_acknowledged=False, controller_acknowledged=controller_ack, claimed=finish_claim, full_response=response)

    def _install_routes(self) -> None:
        router = self.router

        @router.post("/commands")
        async def admit_command(request: CommandRequest) -> dict[str, Any]:
            body = request.model_dump()
            state = self._state()
            action_id = str(body.get("action_id") or "")
            assessment = self._current_assessment(action_id, dict(body.get("inputs") or {}), state)
            return await asyncio.to_thread(
                self.store.admit_command,
                body,
                state=state,
                assessment=assessment,
            )

        @router.post("/methods")
        async def admit_method(request: MethodRequest, http_request: Request) -> dict[str, Any]:
            content_length = http_request.headers.get("content-length")
            try:
                body_bytes = int(content_length) if content_length is not None else None
            except ValueError as exc:
                raise HTTPException(status_code=400, detail="Invalid Content-Length") from exc
            if body_bytes is not None and body_bytes > 1_048_576:
                raise HTTPException(status_code=413, detail="BioXP method document exceeds the 1 MiB limit")
            body = request.model_dump()
            state = self._state()
            assessments: list[Mapping[str, Any]] = []
            for step in body.get("steps", []):
                action_id = str(step.get("action_id") or "")
                assessment = self._current_assessment(action_id, dict(step.get("inputs") or {}), state)
                assessments.extend([assessment] * int(step.get("repeat", 1)))
            return await asyncio.to_thread(
                self.store.admit_method,
                body,
                state=state,
                assessments=assessments,
            )

        @router.get("/commands/{command_id}")
        async def command_detail(command_id: str) -> dict[str, Any]:
            row = await asyncio.to_thread(self.store.get_command, command_id)
            if row is None:
                raise HTTPException(status_code=404, detail="command not found")
            return row

        @router.get("/methods/{method_id}")
        async def method_detail(method_id: str) -> dict[str, Any]:
            row = await asyncio.to_thread(self.store.get_method, method_id)
            if row is None:
                raise HTTPException(status_code=404, detail="method not found")
            return row

        @router.get("/methods/{method_id}/commands")
        async def method_commands(method_id: str, after_method_sequence: int = Query(default=0, ge=0), limit: int = Query(default=200, ge=1, le=200), snapshot_token: str | None = None) -> dict[str, Any]:
            return await asyncio.to_thread(self.store.method_commands, method_id, after=after_method_sequence, limit=limit, token=snapshot_token)

        @router.get("/queue")
        async def queue() -> dict[str, Any]:
            return await asyncio.to_thread(self.store.queue)

        @router.get("/transitions")
        async def transitions(after_sequence: int = Query(default=0, ge=0), limit: int = Query(default=100, ge=1, le=200)) -> dict[str, Any]:
            return await asyncio.to_thread(self.store.transitions, after=after_sequence, limit=limit)

        @router.post("/commands/{command_id}/cancel")
        async def cancel_command(command_id: str, request: MutationRequest) -> dict[str, Any]:
            return await asyncio.to_thread(self.store.cancel_command, command_id, request.model_dump())

        @router.post("/methods/{method_id}/pause")
        async def pause_method(method_id: str, request: MutationRequest) -> dict[str, Any]:
            return await asyncio.to_thread(self.store.method_mutation, method_id, "pause", request.model_dump())

        @router.post("/methods/{method_id}/resume")
        async def resume_method(method_id: str, request: MutationRequest) -> dict[str, Any]:
            return await asyncio.to_thread(self.store.method_mutation, method_id, "resume", request.model_dump())

        @router.post("/methods/{method_id}/cancel")
        async def cancel_method(method_id: str, request: MutationRequest) -> dict[str, Any]:
            return await asyncio.to_thread(self.store.method_mutation, method_id, "cancel", request.model_dump())

        @router.get("/recovery")
        async def recovery() -> dict[str, Any]:
            return await asyncio.to_thread(self.store.recovery)

        @router.post("/recovery/{recovery_epoch}/resolve")
        async def resolve_recovery(recovery_epoch: int, request: RecoveryResolveRequest) -> dict[str, Any]:
            return await asyncio.to_thread(self.store.resolve_recovery, recovery_epoch, request.model_dump())

        @router.get("/idempotency/{operation_kind}/{idempotency_key}")
        async def idempotency(operation_kind: str, idempotency_key: str) -> dict[str, Any]:
            value = await asyncio.to_thread(self.store.idempotency, operation_kind, idempotency_key)
            if value is None:
                raise HTTPException(status_code=404, detail="idempotency receipt not found")
            return value

    def compat_admission(self, action_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        if action_id not in CANONICAL_ACTIONS:
            raise HTTPException(status_code=404, detail="unknown canonical action")
        if action_id in INTERRUPT_ACTIONS:
            return {"action_id": action_id, "ownership_generation": int(self._state().get("ownership_generation") or 0), "enabled": True, "disabled_reason": None, "dependencies": []}
        state = self._state()
        effective = _effective_inputs(action_id, payload.get("inputs") or {}, state)
        assessment = _assess_action(self.by_id[action_id], state, effective)
        return {"action_id": action_id, "ownership_generation": int(state.get("ownership_generation") or 0), **assessment}

    async def admit_strict_method(
        self,
        request: Mapping[str, Any],
        *,
        assessment: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        method_action_id = str(request.get("method_action_id"))
        if method_action_id not in STRICT_METHOD_ACTIONS:
            raise HTTPException(status_code=404, detail="unknown v2 operator method")
        state = self._state()
        observed_generation = int(request.get("expected_ownership_generation") or 0)
        actual_generation = int(state.get("ownership_generation") or 0)
        requested_epochs = {str(key): int(value) for key, value in dict(request.get("expected_board_epoch_by_board") or {}).items()}
        actual_epochs = _active_board_epochs(state, method_action_id)
        expected_epochs: dict[str, int] = {}
        raw_inputs = dict(request.get("inputs") or {})
        inputs = (
            {"x": raw_inputs.get("x_steps"), "y": raw_inputs.get("y_steps")}
            if method_action_id == "oem.xy.move_absolute"
            else raw_inputs
        )
        validated_inputs = _validate_inputs(method_action_id, inputs)
        current_assessment = assessment or self._current_assessment(
            method_action_id,
            validated_inputs,
            state,
        )
        generic = {
            "schema_version": "bioxp.operator_method_request.v1",
            "name": method_action_id,
            "idempotency_key": str(request["idempotency_key"]),
            "expected_ownership_generation": actual_generation,
            "expected_board_epoch_by_board": expected_epochs,
            "failure_policy": "fail_fast",
            "steps": [{"action_id": method_action_id, "inputs": validated_inputs, "repeat": 1}],
            "metadata": {
                "method_action_id": method_action_id,
                "requested_ownership_generation": observed_generation,
                "observed_ownership_generation": actual_generation,
                "requested_board_epoch_by_board": requested_epochs,
                "observed_board_epoch_by_board": actual_epochs,
                "board_epoch_policy": "observational_not_dispatch_fence",
            },
        }
        return await asyncio.to_thread(
            self.store.admit_method,
            generic,
            state=state,
            assessments=[current_assessment],
        )

    async def invoke_internal_y_absolute(
        self,
        source_identity: str,
        *,
        target_steps: int,
        acceleration_override: int | None = None,
    ) -> dict[str, Any]:
        """Dispatch the two source-owned M04 overloads without public catalog exposure."""
        targets = {
            "acceleration_overload": "oem.y.internal.acceleration_overload",
            "board_test_my": "oem.y.internal.board_test_my",
        }
        action_id = targets.get(str(source_identity))
        if action_id is None:
            raise HTTPException(status_code=404, detail={"error": "unknown_internal_y_source_identity"})
        body: dict[str, Any] = {"target_steps": int(target_steps)}
        if source_identity == "acceleration_overload":
            if type(acceleration_override) is not int:
                raise HTTPException(status_code=422, detail={"error": "acceleration_override_required"})
            body["acceleration_override"] = int(acceleration_override)
        elif acceleration_override is not None:
            raise HTTPException(status_code=422, detail={"error": "board_test_my_acceleration_is_source_fixed"})
        target = self._action_target(action_id)
        command_id = f"internal-y-{source_identity}-{uuid.uuid4().hex}"
        token = _DISPATCH_CONTEXT.set({"operator_command_id": command_id, "idempotency_key": command_id, "expected_ownership_generation": int(self._state().get("ownership_generation") or 0), "action_id": action_id})
        try:
            status_code, response = await _dispatch_asgi(self.app, str(target["method"]), str(target["path"]), {**dict(target.get("fixed_inputs") or {}), **body}, target["locations"])
        finally:
            _DISPATCH_CONTEXT.reset(token)
        if not 200 <= int(status_code) < 300:
            raise HTTPException(status_code=int(status_code), detail=response)
        return dict(response) if isinstance(response, Mapping) else {"ok": False, "failure": "internal_y_response_not_mapping"}

    async def invoke_y_interrupt(self, request: Mapping[str, Any]) -> dict[str, Any]:
        return await self.compat_invoke("oem.y.stop", request)

    async def _deliver_controller_interrupt_raw(self, action_id: str, *, interrupt_attempt_id: str) -> tuple[int, Any]:
        provider_action = "oem.abort_all" if action_id in {"oem.abort_all", "oem.z.abort"} else action_id
        target = self._action_target(provider_action)
        token = _DISPATCH_CONTEXT.set({
            "operator_command_id": interrupt_attempt_id,
            "operator_interrupt_id": interrupt_attempt_id,
            "idempotency_key": interrupt_attempt_id,
            "expected_ownership_generation": int(self._state().get("ownership_generation") or 0),
            "action_id": action_id,
        })
        try:
            return await _dispatch_asgi(
                self.app,
                str(target["method"]),
                str(target["path"]),
                dict(target.get("fixed_inputs") or {}),
                target["locations"],
            )
        finally:
            _DISPATCH_CONTEXT.reset(token)

    async def _invoke_controller_interrupt(self, action_id: str, *, receipt: Mapping[str, Any], idempotency_key: str) -> dict[str, Any]:
        interrupt_attempt_id = str(receipt.get("interrupt_attempt_id") or receipt.get("interrupt_id") or uuid.uuid4())
        response: Any = None
        acknowledged = False
        error: str | None = None
        try:
            status_code, response = await self._deliver_controller_interrupt_raw(
                action_id,
                interrupt_attempt_id=interrupt_attempt_id,
            )
            http_success = 200 <= int(status_code) < 300
            source_call_field = response.get("source_call_completed") if isinstance(response, Mapping) else None
            acknowledged = bool(http_success and isinstance(response, Mapping) and (source_call_field is True if type(source_call_field) is bool else type(response.get("ok")) is bool))
            if not http_success:
                error = f"controller_interrupt_http_{status_code}"
            elif not acknowledged:
                error = "oem_stop_source_call_failed"
            elif response.get("ok") is not True:
                error = "oem_stop_source_return_failure"
        except Exception as exc:
            error = f"controller_interrupt_exception:{type(exc).__name__}"
        try:
            return await asyncio.to_thread(
                self.store.finalize_interrupt,
                idempotency_key=idempotency_key,
                receipt={**dict(receipt), "interrupt_attempt_id": interrupt_attempt_id},
                attempted=True,
                acknowledged=acknowledged,
                response=response,
                error=error,
            )
        except Exception as exc:
            if action_id in {"oem.abort_all", "oem.z.abort"}:
                self.store.clear_priority_fence()
            return {
                **dict(receipt),
                "interrupt_attempt_id": interrupt_attempt_id,
                "controller_stop_attempted": True,
                "source_call_completed": acknowledged,
                "source_return_ok": bool(isinstance(response, Mapping) and response.get("ok") is True),
                "controller_stop_acknowledged": bool(isinstance(response, Mapping) and response.get("controller_command_acknowledged") is True),
                "controller_response": _bounded_json(response, 131072),
                "error": error or f"interrupt_sqlite_finalization_failed:{type(exc).__name__}",
                "persistence_state": "recovery_required",
                "recovery_hold": True,
            }

    async def compat_invoke(self, action_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        if action_id in INTERRUPT_ACTIONS:
            self.store.arm_interrupt_fence(action_id)
            try:
                interrupt_attempt_id = str(uuid.uuid4())
                state: Mapping[str, Any] = {
                    "ownership_generation": int(payload.get("observed_ownership_generation") or 0),
                    "board_epoch_by_board": dict(payload.get("observed_board_epoch_by_board") or {}),
                }
                pending_spooled = False
                try:
                    self.store.queue_pending_interrupt_reconciliation({
                        "action_id": action_id,
                        "state": dict(state),
                        "request": dict(payload),
                        "interrupt_attempt_id": interrupt_attempt_id,
                        "attempted": True,
                        "acknowledged": False,
                        "response": None,
                        "error": "controller_delivery_pending",
                    })
                    pending_spooled = True
                except Exception:
                    # Interrupt delivery remains independent of persistence health.
                    pending_spooled = False
                response: Any = None
                acknowledged = False
                error: str | None = None
                try:
                    status_code, response = await self._deliver_controller_interrupt_raw(
                        action_id,
                        interrupt_attempt_id=interrupt_attempt_id,
                    )
                    http_success = 200 <= int(status_code) < 300
                    source_call_field = response.get("source_call_completed") if isinstance(response, Mapping) else None
                    acknowledged = bool(http_success and isinstance(response, Mapping) and (source_call_field is True if type(source_call_field) is bool else type(response.get("ok")) is bool))
                    if not http_success:
                        error = f"controller_interrupt_http_{status_code}"
                    elif not acknowledged:
                        error = "oem_stop_source_call_failed"
                    elif response.get("ok") is not True:
                        error = "oem_stop_source_return_failure"
                except Exception as exc:
                    error = f"controller_interrupt_exception:{type(exc).__name__}"
                if pending_spooled:
                    try:
                        self.store._append_interrupt_spool_event(
                            phase="delivered",
                            payload={
                                "action_id": action_id,
                                "state": dict(state),
                                "request": dict(payload),
                                "interrupt_attempt_id": interrupt_attempt_id,
                                "attempted": True,
                                "acknowledged": acknowledged,
                                "response": response,
                                "error": error,
                            },
                        )
                    except Exception:
                        pass

                try:
                    projected_state = self._state()
                    if isinstance(projected_state, Mapping):
                        state = projected_state
                except Exception:
                    # Projection evidence is diagnostic and cannot redefine the source stop result.
                    pass

                def defer_interrupt_reconciliation() -> None:
                    if pending_spooled:
                        return
                    queue_pending = getattr(self.store, "queue_pending_interrupt_reconciliation", None)
                    if not callable(queue_pending):
                        return
                    queue_pending({
                        "action_id": action_id,
                        "state": dict(state),
                        "request": dict(payload),
                        "interrupt_attempt_id": interrupt_attempt_id,
                        "attempted": True,
                        "acknowledged": acknowledged,
                        "response": response,
                        "error": error,
                    })

                try:
                    receipt = await asyncio.to_thread(
                        self.store.begin_interrupt,
                        action_id,
                        state=state,
                        request=payload,
                        interrupt_attempt_id=interrupt_attempt_id,
                    )
                except Exception as exc:
                    if action_id in {"oem.abort_all", "oem.z.abort"}:
                        self.store.clear_priority_fence()
                    defer_interrupt_reconciliation()
                    return {
                        "schema_version": "bioxp.operator_interrupt_receipt.v1",
                        "robot_identity": ROBOT_IDENTITY,
                        "ownership_generation": int(state.get("ownership_generation") or 0),
                        "observed_ownership_generation": payload.get("observed_ownership_generation"),
                        "observed_board_epoch_by_board": dict(payload.get("observed_board_epoch_by_board") or {}),
                        "interrupt_id": interrupt_attempt_id,
                        "interrupt_attempt_id": interrupt_attempt_id,
                        "action_id": action_id,
                        "scope": "aggregate" if action_id in {"oem.abort_all", "oem.z.abort"} else AXIS_BY_ACTION.get(action_id),
                        "oem_abort_latched": action_id in {"oem.abort_all", "oem.z.abort"},
                        "controller_stop_attempted": True,
                        "source_call_completed": acknowledged,
                        "source_return_ok": bool(isinstance(response, Mapping) and response.get("ok") is True),
                        "controller_stop_acknowledged": bool(isinstance(response, Mapping) and response.get("controller_command_acknowledged") is True),
                        "physical_effect_verified": False,
                        "controller_response": _bounded_json(response, 131072),
                        "error": error or f"interrupt_sqlite_admission_failed:{type(exc).__name__}",
                        "persistence_state": "recovery_required",
                        "recovery_hold": True,
                        "idempotent_replay": False,
                    }
                if str(receipt.get("persistence_state")) != "committed":
                    if action_id in {"oem.abort_all", "oem.z.abort"}:
                        self.store.clear_priority_fence()
                    defer_interrupt_reconciliation()
                    return {
                        **dict(receipt),
                        "interrupt_attempt_id": interrupt_attempt_id,
                        "controller_stop_attempted": True,
                        "source_call_completed": acknowledged,
                        "source_return_ok": bool(isinstance(response, Mapping) and response.get("ok") is True),
                        "controller_stop_acknowledged": bool(isinstance(response, Mapping) and response.get("controller_command_acknowledged") is True),
                        "controller_response": _bounded_json(response, 131072),
                        "error": error or "interrupt_sqlite_admission_failed",
                        "persistence_state": "recovery_required",
                        "recovery_hold": True,
                    }
                try:
                    terminal_receipt = await asyncio.to_thread(
                        self.store.finalize_interrupt,
                        idempotency_key=str(payload["idempotency_key"]),
                        receipt={**dict(receipt), "interrupt_attempt_id": interrupt_attempt_id},
                        attempted=True,
                        acknowledged=acknowledged,
                        response=response,
                        error=error,
                    )
                    if pending_spooled:
                        self.store._append_interrupt_spool_event(
                            phase="reconciled",
                            payload={
                                "interrupt_attempt_id": interrupt_attempt_id,
                                "terminal_receipt": terminal_receipt,
                            },
                        )
                    return terminal_receipt
                except Exception as exc:
                    if action_id in {"oem.abort_all", "oem.z.abort"}:
                        self.store.clear_priority_fence()
                    defer_interrupt_reconciliation()
                    return {
                        **dict(receipt),
                        "interrupt_attempt_id": interrupt_attempt_id,
                        "controller_stop_attempted": True,
                        "source_call_completed": acknowledged,
                        "source_return_ok": bool(isinstance(response, Mapping) and response.get("ok") is True),
                        "controller_stop_acknowledged": bool(isinstance(response, Mapping) and response.get("controller_command_acknowledged") is True),
                        "controller_response": _bounded_json(response, 131072),
                        "error": error or f"interrupt_sqlite_finalization_failed:{type(exc).__name__}",
                        "persistence_state": "recovery_required",
                        "recovery_hold": True,
                    }
            finally:
                self.store.clear_interrupt_fence(action_id)

        state = self._state()
        actual_generation = int(state.get("ownership_generation") or 0)
        request = {"schema_version": COMMAND_SCHEMA, "idempotency_key": payload["idempotency_key"], "expected_ownership_generation": actual_generation, "action_id": action_id, "inputs": dict(payload.get("inputs") or {})}
        return await asyncio.to_thread(self.store.admit_command, request, state=state)

    def stop(self) -> None:
        self.store.stop()
