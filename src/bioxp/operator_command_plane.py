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
from .oem_runtime_store import OEMRuntimeStore, migrate_runtime_database_v2

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
    "oem.z.manual_home",
    "oem.z.clear",
    "oem.z.move_steps",
    "oem.z.move_absolute",
    "oem.x.manual_panel_home",
    "oem.x.move_steps",
    "oem.x.move_absolute",
    "oem.y.manual_panel_home",
    "oem.y.prepare",
    "oem.y.move_steps",
    "oem.y.move_absolute",
    "oem.xy.move_absolute",
    "oem.xy.home",
})
INTERRUPT_ACTIONS = frozenset({
    "oem.x.stop", "oem.y.stop", "oem.z.stop", "oem.abort_all", "oem.z.abort",
})
CANONICAL_ACTIONS = ALLOWED_ACTIONS | INTERRUPT_ACTIONS
AXIS_BY_ACTION = {
    "oem.x.manual_panel_home": "x",
    "oem.x.move_steps": "x",
    "oem.x.move_absolute": "x",
    "oem.y.manual_panel_home": "y",
    "oem.y.move_steps": "y",
    "oem.y.move_absolute": "y",
    "oem.y.stop": "y",
    "oem.z.manual_home": "z",
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
        "oem.z.manual_home": set(),
        "oem.z.clear": set(),
        "oem.x.manual_panel_home": set(),
        "oem.y.manual_panel_home": set(),
        "oem.y.prepare": set(),
        "oem.z.move_steps": {"steps"},
        "oem.x.move_steps": {"steps"},
        "oem.y.move_steps": {"steps"},
        "oem.z.move_absolute": {"position_steps"},
        "oem.x.move_absolute": {"position_steps"},
        "oem.y.move_absolute": {"target_steps"},
        "oem.xy.move_absolute": {"x", "y"},
        "oem.xy.home": set(),
    }
    unknown = sorted(set(inputs) - allowed.get(action_id, set()))
    if unknown:
        raise HTTPException(status_code=422, detail={"error": "unknown_command_inputs", "unknown": unknown})
    result = dict(inputs)
    if action_id == "oem.xy.move_absolute":
        for key in ("x", "y"):
            if type(result.get(key)) is not int or not -2_147_483_648 <= result[key] <= 2_147_483_647:
                raise HTTPException(status_code=422, detail={"error": "invalid_xy_target", "field": key, "required": "signed int32"})
        return result
    if action_id.endswith("move_steps"):
        steps = result.get("steps")
        maximum = 102_936 if action_id.startswith("oem.y.") else 160000
        minimum = -maximum
        if type(steps) is not int or not minimum <= steps <= maximum:
            required = f"integer in [-{maximum},{maximum}]"
            raise HTTPException(status_code=422, detail={"error": "invalid_steps", "required": required})
    if action_id.endswith("move_absolute"):
        if action_id.startswith("oem.y."):
            target = result.get("target_steps")
            if type(target) is not int or not 0 <= target <= 102956:
                raise HTTPException(status_code=422, detail={"error": "invalid_target_steps", "required": "integer in [0,102956]"})
            return result
        position = result.get("position_steps")
        if type(position) is not int or not 0 <= position <= (90263 if action_id.startswith("oem.x.") else 160000):
            maximum = 90263 if action_id.startswith("oem.x.") else 160000
            raise HTTPException(status_code=422, detail={"error": "invalid_position_steps", "required": f"integer in [0,{maximum}]"})
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
    if action_id.startswith("oem.xy.") and isinstance(provider, Mapping):
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
    if completion_class == "oem_timeout_target_equal" and not event_128:
        return "ambiguous"
    target_equal = type(target_steps) is int and type(observed_position_steps) is int and observed_position_steps == target_steps
    if event_128 and target_equal and terminal_speed_zero:
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
    reason: str = Field(min_length=1, max_length=500)
    observed_ownership_generation: StrictInt | None
    observed_board_epoch_by_board: dict[str, StrictInt]

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
    method_action_id: str = Field(pattern=r"^oem\.xy\.(move_absolute|home)$")
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
        self._y_interrupt_fallback_path = self.root / "operator_y_interrupt_fallback.v2.jsonl"
        self._y_interrupt_fallback_lock_path = self.root / "operator_y_interrupt_fallback.v2.lock"
        self._lock = threading.RLock()
        self._priority_fence = threading.Event()
        self._interrupt_lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._worker_lock = threading.Lock()
        self._workers: set[threading.Thread] = set()
        self.owner_id = uuid.uuid4().hex
        self._owner_acquired = False
        # Converge the legacy shared X/Y/Z fallback before opening the command
        # plane. The owner-specific v2 Y fallback is imported below.
        shared_importer = OEMRuntimeStore(self.root)
        shared_importer.close()
        self.connection = sqlite3.connect(self.path, timeout=2.0, isolation_level=None, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        self._configure()
        migrate_runtime_database_v2(self.connection, self.root)
        self._schema()
        self._import_y_interrupt_fallback()
        self._owner_acquired = self._acquire_owner()
        if self._owner_acquired:
            self._startup_recover()

    def append_y_interrupt_fallback(self, receipt: Mapping[str, Any], *, reason: str) -> dict[str, Any]:
        row = dict(receipt)
        row["persistence_fallback"] = {"kind": "serial206_interrupt_jsonl", "reason": str(reason)[:500], "recorded_at": _now()}
        raw = (_canonical({"schema_version": "bioxp.serial206_interrupt_fallback.v2", "owner": "operator_command_plane", "stream": "y", "receipt": row}) + "\n").encode("utf-8")
        lock_descriptor = os.open(self._y_interrupt_fallback_lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            os.fchmod(lock_descriptor, 0o600)
            fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
            descriptor = os.open(self._y_interrupt_fallback_path, os.O_APPEND | os.O_CREAT | os.O_WRONLY, 0o600)
            try:
                os.fchmod(descriptor, 0o600)
                written = os.write(descriptor, raw)
                if written != len(raw):
                    raise OSError(f"short Y interrupt fallback write: {written}/{len(raw)} bytes")
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            directory = os.open(self.root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
            os.close(lock_descriptor)
        return row

    def _import_y_interrupt_fallback(self) -> None:
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
                    if not isinstance(wrapper, Mapping) or str(wrapper.get("stream") or "") != "y" or not isinstance(wrapper.get("receipt"), Mapping):
                        raise RuntimeError("Y interrupt fallback contains an invalid command-plane row")
                    receipt = dict(wrapper["receipt"])
                    attempt_id = str(receipt.get("interrupt_attempt_id") or "")
                    if not attempt_id:
                        raise RuntimeError("Y interrupt fallback is missing interrupt_attempt_id")
                    key = f"interrupt-attempt:{attempt_id}"
                    fingerprint = _digest(receipt)
                    record_sha256 = hashlib.sha256(_canonical(wrapper).encode("utf-8")).hexdigest()
                    conn.execute(
                        "INSERT OR IGNORE INTO operator_plane_interrupt_history(record_sha256,stream,interrupt_attempt_id,receipt_json,imported_at) VALUES(?,?,?,?,?)",
                        (record_sha256, "y", attempt_id, _canonical(receipt), _now()),
                    )
                    inserted = conn.execute(
                        "INSERT OR IGNORE INTO operator_plane_idempotency(operation_kind,idempotency_key,fingerprint,response_json,created_at) VALUES('interrupt',?,?,?,?)",
                        (key, fingerprint, _canonical(receipt), _now()),
                    ).rowcount
                    if inserted == 1:
                        self._insert_transition(conn, event_kind="y_interrupt_fallback_imported", state="interrupted", payload={"interrupt_attempt_id": attempt_id, "persistence_state": "fsync_fallback"})
                    else:
                        existing = conn.execute("SELECT fingerprint FROM operator_plane_idempotency WHERE operation_kind='interrupt' AND idempotency_key=?", (key,)).fetchone()
                        if existing is None or str(existing["fingerprint"]) != fingerprint:
                            raise RuntimeError("Y interrupt fallback identity conflicts with canonical command-plane persistence")
            archive = self.root / pending.name.replace(".pending.", ".imported.")
            os.replace(pending, archive)
            directory = os.open(self.root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory)
            finally:
                os.close(directory)

    def _configure(self) -> None:
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=FULL")
        self.connection.execute("PRAGMA foreign_keys=ON")
        self.connection.execute("PRAGMA busy_timeout=2000")
        self.connection.execute("PRAGMA wal_autocheckpoint=256")

    def _schema(self) -> None:
        with self._lock:
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
                CREATE TABLE IF NOT EXISTS operator_plane_interrupt_history (
                    record_sha256 TEXT PRIMARY KEY,
                    stream TEXT NOT NULL CHECK(stream IN ('x','y','z')),
                    interrupt_attempt_id TEXT NOT NULL,
                    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
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
            self.connection.executescript(
                """
                CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_no_delete
                BEFORE DELETE ON operator_plane_transitions
                BEGIN SELECT RAISE(ABORT, 'operator transitions are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_no_update
                BEFORE UPDATE ON operator_plane_transitions
                BEGIN SELECT RAISE(ABORT, 'operator transitions are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_no_terminal_delete
                BEFORE DELETE ON operator_plane_commands
                WHEN OLD.status IN ('completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted')
                BEGIN SELECT RAISE(ABORT, 'terminal operator commands are immutable'); END;
                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_no_terminal_delete
                BEFORE DELETE ON operator_plane_methods
                WHEN OLD.status IN ('completed','failed','cancelled','stopped','aborted','interrupted')
                BEGIN SELECT RAISE(ABORT, 'terminal operator methods are immutable'); END;
                """
            )

    @contextmanager
    def _transaction(self, *, timeout_ms: int = 2000):
        with self._lock:
            self.connection.execute(f"PRAGMA busy_timeout={int(timeout_ms)}")
            try:
                self.connection.execute("BEGIN IMMEDIATE")
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
            conn.execute("UPDATE operator_plane_safety SET recovery_epoch=?,recovery_version=recovery_version+1,recovery_hold=?,updated_at=? WHERE singleton=1", (recovery_epoch, int(bool(active)), _now()))

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
        axis_scope = "xy" if composite_xy else axis
        board_scope = {"4": [0], "5": [0]} if composite_xy else ({"4": [motor_by_axis[axis]]} if axis in motor_by_axis else {})
        canonical_hash = _digest(dict(inputs))
        expected_epochs = dict(expected_board_epochs or {}) if axis in motor_by_axis or composite_xy else {}
        safety = conn.execute("SELECT x_epoch,y_epoch,z_epoch FROM operator_plane_safety WHERE singleton=1").fetchone()
        interrupt_epochs = {"x": int(safety["x_epoch"]), "y": int(safety["y_epoch"])} if composite_xy else ({axis: int(safety[f"{axis}_epoch"])} if axis in {"x", "y", "z"} else {})
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
        if composite_xy:
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

    def admit_command(self, request: Mapping[str, Any], *, state: Mapping[str, Any]) -> dict[str, Any]:
        action_id = str(request.get("action_id") or "")
        if action_id not in ALLOWED_ACTIONS:
            raise HTTPException(status_code=422, detail={"error": "action_not_allowed", "action_id": action_id})
        inputs = _validate_inputs(action_id, request.get("inputs", {}))
        expected_generation = int(request["expected_ownership_generation"])
        schema_version = str(request.get("schema_version") or COMMAND_SCHEMA)
        if action_id.startswith("oem.y.") and schema_version != ACTION_REQUEST_SCHEMA:
            raise HTTPException(status_code=410, detail={"error": "legacy_y_mutation_retired", "required_schema": ACTION_REQUEST_SCHEMA})
        expected_board_epochs = {
            str(key): int(value)
            for key, value in dict(request.get("expected_board_epoch_by_board") or {}).items()
            if type(value) is int and value >= 0
        }
        canonical_request = {"schema_version": schema_version, "operation_kind": "command", "expected_ownership_generation": expected_generation, "expected_board_epoch_by_board": expected_board_epochs, "action_id": action_id, "inputs": inputs}
        fingerprint = _digest(canonical_request)
        key = str(request["idempotency_key"])
        saved = self.idempotency_checked("command", key, fingerprint)
        if saved is not None:
            saved["idempotent_replay"] = True
            return saved
        actual_generation = int(state.get("ownership_generation") or 0)
        if expected_generation != actual_generation:
            raise HTTPException(status_code=409, detail={"error": "ownership_generation_mismatch", "expected": expected_generation, "actual": actual_generation})
        if schema_version == ACTION_REQUEST_SCHEMA:
            actual_epochs = _active_board_epochs(state, action_id)
            required_boards = {"4"} if action_id.startswith("oem.y.") else {"4", "5"} if action_id.startswith("oem.xy.") else set(actual_epochs)
            if set(actual_epochs) != required_boards:
                raise HTTPException(status_code=409, detail={"error": "board_epoch_authority_unavailable", "required_boards": sorted(required_boards), "actual": actual_epochs})
            if expected_board_epochs != actual_epochs:
                raise HTTPException(status_code=409, detail={"error": "board_epoch_mismatch", "expected": expected_board_epochs, "actual": actual_epochs})
        if action_id == "oem.y.move_steps":
            current_y = _axis_position(state, "y")
            if type(current_y) is not int:
                raise HTTPException(status_code=409, detail={"error": "y_position_readback_required"})
            effective_target = current_y + int(inputs["steps"])
            if not 20 <= effective_target <= 102_936:
                raise HTTPException(status_code=409, detail={"error": "y_relative_effective_target_out_of_range", "effective_target_steps": effective_target, "required_range": [20, 102_936]})
        with self._transaction() as conn:
            replay = self._saved_idempotency(conn, "command", key, fingerprint)
            if replay is not None:
                return replay
            if action_id.startswith("oem.x.") and not self._z_home_authority_valid(conn, state):
                raise HTTPException(status_code=409, detail={"error": "z_home_authority_required", "reason": "Standalone X requires current Z Home authority"})
            if action_id.startswith("oem.y.") and action_id != "oem.y.prepare":
                y_authority = conn.execute("SELECT lifecycle_state FROM serial206_axis_authority WHERE axis='y'").fetchone()
                if y_authority is not None and str(y_authority[0]) == "reconciliation_required":
                    raise HTTPException(status_code=409, detail={"error": "y_reconciliation_required", "required_action": "oem.y.prepare"})
            if schema_version == ACTION_REQUEST_SCHEMA:
                self._sync_board_authority(conn, actual_epochs)
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
            transition = self._insert_transition(conn, event_kind="command_admitted", command_id=command_id, state="queued", payload={"stream_sequence": sequence, "action_id": action_id})
            response = {**row, "schema_version": RECEIPT_SCHEMA, "transition_sequence": transition, "remote_acknowledged": False, "controller_acknowledged": False, "physical_effect_verified": False, "source_noop": False, "source_noop_reason": None, "terminal_evidence": None, "dispatched_at": None, "finished_at": None, "idempotent_replay": False}
            self._store_idempotency(conn, kind="command", key=key, fingerprint=fingerprint, response=response, command_id=command_id)
        self._wake.set()
        return response

    def admit_method(self, request: Mapping[str, Any], *, state: Mapping[str, Any]) -> dict[str, Any]:
        expected_generation = int(request["expected_ownership_generation"])
        expected_board_epochs = {
            str(key): int(value)
            for key, value in dict(request.get("expected_board_epoch_by_board") or {}).items()
            if type(value) is int and value >= 0
        }
        expanded: list[tuple[str, dict[str, Any]]] = []
        seen_home = False
        method_requires_home = False
        for step in request.get("steps", []):
            action_id = str(step.get("action_id") or "")
            if action_id.startswith("oem.y.") and not expected_board_epochs:
                raise HTTPException(status_code=410, detail={"error": "legacy_y_method_mutation_retired"})
            if action_id not in ALLOWED_ACTIONS:
                raise HTTPException(status_code=422, detail={"error": "method_action_not_allowed", "action_id": action_id})
            if action_id.startswith("oem.x.") and not seen_home:
                method_requires_home = True
            inputs = _validate_inputs(action_id, step.get("inputs", {}))
            repeat = int(step.get("repeat", 1))
            if repeat < 1 or repeat > MAX_METHOD_COMMANDS or len(expanded) + repeat > MAX_METHOD_COMMANDS:
                raise HTTPException(status_code=422, detail={"error": "method_expansion_limit"})
            for _ in range(repeat):
                expanded.append((action_id, dict(inputs)))
            if action_id == "oem.z.manual_home":
                seen_home = True
        if not expanded or len(expanded) > MAX_METHOD_COMMANDS:
            raise HTTPException(status_code=422, detail={"error": "method_expansion_limit"})
        source = {"schema_version": METHOD_SCHEMA, "name": str(request["name"]), "failure_policy": "fail_fast", "expected_board_epoch_by_board": expected_board_epochs, "steps": [{"action_id": a, "inputs": i} for a, i in expanded], "metadata": dict(request.get("metadata") or {})}
        digest = _digest(source)
        canonical_request = {"schema_version": METHOD_SCHEMA, "operation_kind": "method", "expected_ownership_generation": expected_generation, "expected_board_epoch_by_board": expected_board_epochs, "source": source}
        fingerprint = _digest(canonical_request)
        key = str(request["idempotency_key"])
        saved = self.idempotency_checked("method", key, fingerprint)
        if saved is not None:
            saved["idempotent_replay"] = True
            return saved
        if expected_generation != int(state.get("ownership_generation") or 0):
            raise HTTPException(status_code=409, detail={"error": "ownership_generation_mismatch"})
        actual_method_epochs: dict[str, int] = {}
        for action_id, _ in expanded:
            actual_method_epochs.update(_active_board_epochs(state, action_id))
        method_requires_xy_epochs = any(action_id.startswith("oem.xy.") for action_id, _ in expanded)
        if method_requires_xy_epochs and (
            set(expected_board_epochs) != {"4", "5"}
            or set(actual_method_epochs) != {"4", "5"}
        ):
            raise HTTPException(
                status_code=409,
                detail={
                    "error": "board_epoch_map_required",
                    "required_boards": ["4", "5"],
                    "expected": expected_board_epochs,
                    "actual": actual_method_epochs,
                },
            )
        if expected_board_epochs and expected_board_epochs != actual_method_epochs:
            raise HTTPException(status_code=409, detail={"error": "board_epoch_mismatch", "expected": expected_board_epochs, "actual": actual_method_epochs})
        with self._transaction() as conn:
            replay = self._saved_idempotency(conn, "method", key, fingerprint)
            if replay is not None:
                return replay
            if method_requires_home and not self._z_home_authority_valid(conn, state):
                raise HTTPException(status_code=409, detail={"error": "z_home_authority_required", "reason": "Method X requires current Z Home or an earlier Z Home step"})
            if expected_board_epochs:
                self._sync_board_authority(conn, expected_board_epochs)
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
                "observed_values": dict(terminal.get("observed_values") or {key: response[key] for key in observed_keys if key in response}),
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
            if request["operation"] == "cancel_pending":
                conn.execute("UPDATE operator_plane_commands SET status='cancelled',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE status='queued'", (_now(), _now(), _canonical({"reason": "recovery_cancel_pending"})))
                for command_id in unknown:
                    conn.execute("UPDATE operator_plane_commands SET status='cancelled',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE command_id=? AND status IN ('ambiguous','interrupted')", (_now(), _now(), _canonical({"reason": "recovery_cancel_pending_unknown_outcome_acknowledged"}), command_id))
                for method_id in sorted({str(row["method_id"]) for row in conn.execute("SELECT method_id FROM operator_plane_commands WHERE command_id IN ({})".format(",".join("?" for _ in unknown)), tuple(unknown)).fetchall() if row["method_id"]} if unknown else set()):
                    conn.execute("UPDATE operator_plane_methods SET status='cancelled',version=version+1,updated_at=? WHERE method_id=? AND status NOT IN ('completed','failed','cancelled','stopped','aborted','interrupted')", (_now(), method_id))
            else:
                for command_id in unknown:
                    conn.execute("UPDATE operator_plane_commands SET status='interrupted',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE command_id=? AND status IN ('ambiguous','interrupted')", (_now(), _now(), _canonical({"reason": "recovery_resume_undispatched_unknown_outcome_acknowledged"}), command_id))
                for method_id in sorted({str(row["method_id"]) for row in conn.execute("SELECT method_id FROM operator_plane_commands WHERE command_id IN ({})".format(",".join("?" for _ in unknown)), tuple(unknown)).fetchall() if row["method_id"]} if unknown else set()):
                    conn.execute("UPDATE operator_plane_methods SET status='running',version=version+1,updated_at=? WHERE method_id=? AND status='recovery_required'", (_now(), method_id))
            conn.execute("UPDATE operator_plane_safety SET recovery_hold=0,recovery_version=recovery_version+1,updated_at=? WHERE singleton=1", (_now(),))
            transition = self._insert_transition(conn, event_kind="recovery_resolved", state="recovery_resolved", payload={"operation": request["operation"], "acknowledged": unknown})
            response = {"schema_version": "bioxp.operator_recovery_resolution.v1", "recovery_epoch": int(recovery_epoch), "operation": request["operation"], "acknowledged_command_ids": unknown, "transition_sequence": transition}
            self._store_idempotency(conn, kind="recovery_resolve", key=key, fingerprint=fp, response=response)
        self._wake.set()
        return response

    def idempotency_checked(self, operation_kind: str, key: str, fingerprint: str) -> dict[str, Any] | None:
        with self._lock:
            row = self.connection.execute("SELECT fingerprint,response_json FROM operator_plane_idempotency WHERE operation_kind=? AND idempotency_key=?", (operation_kind, key)).fetchone()
            if row is None:
                return None
            if str(row["fingerprint"]) != fingerprint:
                raise HTTPException(status_code=409, detail={"error": "idempotency_conflict", "operation_kind": operation_kind, "idempotency_key": key})
            return _json_load(row["response_json"], None)

    def idempotency(self, operation_kind: str, key: str) -> dict[str, Any] | None:
        with self._lock:
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
            if bool(safety["recovery_hold"]):
                return None
            lane = conn.execute("SELECT * FROM operator_plane_lane WHERE singleton=1").fetchone()
            if lane["owner_id"] != self.owner_id or float(lane["owner_lease_until"] or 0.0) <= _now():
                return None
            if lane["active_command_id"] is not None or lane["active_attempt_id"] is not None:
                return None
            row = conn.execute(
                "SELECT * FROM operator_plane_commands WHERE status='queued' ORDER BY stream_sequence LIMIT 1"
            ).fetchone()
            if row is None:
                return None
            canonical = conn.execute(
                "SELECT * FROM serial206_movement_commands WHERE command_id=?",
                (row["command_id"],),
            ).fetchone()
            if canonical is not None and str(canonical["state"]) != "queued":
                return None
            if canonical is not None:
                expected_epochs = _json_load(canonical["expected_board_epochs_json"], {})
                for board_id, expected_epoch in dict(expected_epochs or {}).items():
                    authority = (
                        conn.execute("SELECT state,active_board_epoch FROM serial206_board_authority WHERE board_id=4").fetchone()
                        if str(board_id) == "4"
                        else conn.execute("SELECT state,active_board_epoch FROM operator_plane_board_authority WHERE board_id=5").fetchone()
                        if str(board_id) == "5"
                        else None
                    )
                    if authority is None or authority["state"] != "active" or authority["active_board_epoch"] != expected_epoch:
                        now = _now()
                        terminal = {"error": "board_epoch_changed_before_dispatch", "board_id": str(board_id), "expected_epoch": expected_epoch, "delivery_attempted": False}
                        conn.execute("UPDATE serial206_movement_commands SET state='rejected',state_version=state_version+1,finished_at=? WHERE command_id=? AND state='queued'", (now, row["command_id"]))
                        conn.execute("UPDATE operator_plane_commands SET status='failed',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE command_id=? AND status='queued'", (now, now, _canonical(terminal), row["command_id"]))
                        transition = self._insert_transition(conn, event_kind="command_rejected", command_id=str(row["command_id"]), method_id=row["method_id"], state="failed", payload=terminal)
                        conn.execute("INSERT INTO operator_plane_outbox(outbox_id,command_id,transition_sequence,state,payload_json,updated_at) VALUES(?,?,?,?,?,?)", (str(uuid.uuid4()), str(row["command_id"]), transition, "pending", _canonical(terminal), now))
                        if row["method_id"]:
                            self._derive_method(conn, str(row["method_id"]))
                        return None
            if row["method_id"]:
                method = conn.execute(
                    "SELECT status FROM operator_plane_methods WHERE method_id=?",
                    (row["method_id"],),
                ).fetchone()
                if method is None or str(method["status"]) not in {"queued", "running"}:
                    return None
            if row["method_id"]:
                method = conn.execute("SELECT status FROM operator_plane_methods WHERE method_id=?", (row["method_id"],)).fetchone()
                if method is not None and str(method["status"]) == "queued":
                    conn.execute("UPDATE operator_plane_methods SET status='running',version=version+1,updated_at=? WHERE method_id=? AND status='queued'", (_now(), row["method_id"]))
                    self._insert_transition(conn, event_kind="method_running", method_id=str(row["method_id"]), state="running", payload={"first_child_command_id": str(row["command_id"])})
            attempt_id = str(uuid.uuid4())
            epoch = int(lane["dispatcher_epoch"])
            action_axis = AXIS_BY_ACTION.get(str(row["action_id"]))
            axis_epoch = (
                int(safety["x_epoch"]) + int(safety["y_epoch"])
                if str(row["action_id"]).startswith("oem.xy.")
                else int(safety[{"x": "x_epoch", "y": "y_epoch", "z": "z_epoch"}.get(action_axis or "", "z_epoch")])
            )
            now = _now()
            lane_claimed = conn.execute(
                "UPDATE operator_plane_lane SET active_command_id=?,active_attempt_id=?,updated_at=? "
                "WHERE singleton=1 AND owner_id=? AND owner_lease_until>? AND dispatcher_epoch=? "
                "AND active_command_id IS NULL AND active_attempt_id IS NULL",
                (row["command_id"], attempt_id, now, self.owner_id, now, epoch),
            ).rowcount
            if lane_claimed != 1:
                return None
            command_claimed = conn.execute(
                "UPDATE operator_plane_commands SET status='dispatched',version=version+1,dispatch_attempt_id=?,dispatcher_epoch=?,dispatch_global_safety_epoch=?,dispatch_axis_safety_epoch=?,dispatched_at=?,updated_at=? "
                "WHERE command_id=? AND status='queued' AND version=?",
                (attempt_id, epoch, int(safety["global_epoch"]), axis_epoch, now, now, row["command_id"], int(row["version"])),
            ).rowcount
            if command_claimed != 1:
                conn.execute(
                    "UPDATE operator_plane_lane SET active_command_id=NULL,active_attempt_id=NULL,updated_at=? WHERE singleton=1 AND active_command_id=? AND active_attempt_id=?",
                    (_now(), row["command_id"], attempt_id),
                )
                return None
            conn.execute("UPDATE serial206_movement_commands SET state='dispatched',state_version=state_version+1,dispatched_at=? WHERE command_id=? AND state='queued'", (now, row["command_id"]))
            self._insert_transition(conn, event_kind="command_dispatched", command_id=str(row["command_id"]), method_id=row["method_id"], state="dispatched", payload={"dispatch_attempt_id": attempt_id, "dispatcher_epoch": epoch})
            claimed = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (row["command_id"],)).fetchone()
            assert claimed is not None
            return {"command_id": str(claimed["command_id"]), "method_id": claimed["method_id"], "method_sequence": claimed["method_sequence"], "action_id": str(claimed["action_id"]), "requested_inputs": _json_load(claimed["requested_json"], {}), "effective_inputs": _json_load(claimed["effective_json"], {}), "dispatch_attempt_id": str(claimed["dispatch_attempt_id"]), "dispatcher_epoch": int(claimed["dispatcher_epoch"]), "dispatch_global_safety_epoch": int(claimed["dispatch_global_safety_epoch"]), "dispatch_axis_safety_epoch": int(claimed["dispatch_axis_safety_epoch"]), "ownership_generation": int(claimed["ownership_generation"]), "command_version": int(claimed["version"])}

    def mark_dispatched(self, command_id: str, *, payload: Mapping[str, Any]) -> dict[str, Any]:
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                return {"command_id": command_id, "status": "missing"}
            if str(row["status"]) != "dispatched":
                return self._command_response(row)
            now = _now()
            evidence = _canonical(dict(payload))
            conn.execute("UPDATE operator_plane_commands SET status='issued_pending',version=version+1,terminal_json=?,updated_at=? WHERE command_id=? AND version=? AND status='dispatched'", (evidence, now, command_id, int(row["version"])))
            conn.execute("UPDATE serial206_movement_commands SET state='issued_pending',state_version=state_version+1 WHERE command_id=? AND state='dispatched'", (command_id,))
            self._insert_transition(conn, event_kind="command_pending", command_id=command_id, method_id=row["method_id"], state="issued_pending", payload=dict(payload))
            updated = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            assert updated is not None
            result = self._command_response(updated)
        self._wake.set()
        return result

    def finish(self, command_id: str, *, status: str, payload: Mapping[str, Any], source_noop: bool = False, source_noop_reason: str | None = None, remote_acknowledged: bool = False, controller_acknowledged: bool = False, physical_effect_verified: bool = False, claimed: Mapping[str, Any] | None = None) -> dict[str, Any]:
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
                    or str(lane["active_command_id"] or "") != command_id
                    or str(lane["active_attempt_id"] or "") != str(claimed["dispatch_attempt_id"])
                ):
                    return self._command_response(row)
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
            conn.execute("UPDATE operator_plane_lane SET active_command_id=NULL,active_attempt_id=NULL,updated_at=? WHERE singleton=1 AND active_command_id=?", (_now(), command_id))
            conn.execute("INSERT INTO operator_plane_outbox(outbox_id,command_id,transition_sequence,state,payload_json,updated_at) VALUES(?,?,?,?,?,?)", (str(uuid.uuid4()), command_id, transition, "pending", _canonical(_bounded_json(payload, 131072)), _now()))
            method_id = row["method_id"]
            if status in {"ambiguous", "interrupted"}:
                conn.execute("UPDATE operator_plane_safety SET recovery_hold=1,recovery_epoch=recovery_epoch+1,recovery_version=recovery_version+1,updated_at=? WHERE singleton=1", (_now(),))
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

    def begin_interrupt(self, action_id: str, *, state: Mapping[str, Any], request: Mapping[str, Any]) -> dict[str, Any]:
        if action_id not in INTERRUPT_ACTIONS:
            raise HTTPException(status_code=422, detail={"error": "interrupt_action_not_allowed"})
        key = str(request["idempotency_key"])
        fp = _digest({"operation_kind": "interrupt", "action_id": action_id, **_without_idempotency(request)})
        saved = self.idempotency_checked("interrupt", key, fp)
        if saved is not None:
            saved["idempotent_replay"] = True
            return saved
        self._priority_fence.set()
        with self._interrupt_lock:
            timeout_ms = 5 if action_id == "oem.y.stop" else 25
            try:
                with self._transaction(timeout_ms=timeout_ms) as conn:
                    safety = conn.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
                    lane = conn.execute("SELECT * FROM operator_plane_lane WHERE singleton=1").fetchone()
                    interrupt_id = str(uuid.uuid4())
                    axis = AXIS_BY_ACTION.get(action_id)
                    aggregate_interrupt = action_id in {"oem.abort_all", "oem.z.abort"}
                    active_rows = conn.execute(
                        "SELECT DISTINCT c.command_id FROM serial206_movement_commands c LEFT JOIN serial206_command_resources r ON r.command_id=c.command_id WHERE c.state IN ('dispatched','issued_pending') AND (? OR r.resource_key=?)",
                        (int(aggregate_interrupt), f"axis:{axis}"),
                    ).fetchall()
                    active_ids = [str(item[0]) for item in active_rows]
                    legacy_active_id = str(lane["active_command_id"]) if lane["active_command_id"] else None
                    if legacy_active_id and legacy_active_id not in active_ids:
                        active_ids.append(legacy_active_id)
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
                    method_rows = conn.execute(
                        "SELECT DISTINCT method_id FROM operator_plane_commands WHERE method_id IS NOT NULL AND stream_sequence<=? AND status='queued' AND (? OR action_id LIKE ?)",
                        (cutoff, int(aggregate_interrupt), f"oem.{axis}.%"),
                    ).fetchall()
                    affected_method_ids = {str(item[0]) for item in method_rows if item[0]}
                    for active_method in conn.execute("SELECT method_id FROM operator_plane_commands WHERE command_id IN (%s)" % ",".join("?" for _ in active_ids), tuple(active_ids)).fetchall() if active_ids else []:
                        if active_method[0]:
                            affected_method_ids.add(str(active_method[0]))
                    reason_json = _canonical({"reason": action_id, "cutoff": cutoff})
                    now = _now()
                    if aggregate_interrupt:
                        conn.execute("UPDATE operator_plane_commands SET status='cleared',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE stream_sequence<=? AND status='queued'", (now, now, reason_json, cutoff))
                        conn.execute("UPDATE serial206_movement_commands SET state='cleared',state_version=state_version+1,finished_at=? WHERE state='queued'", (now,))
                    else:
                        conn.execute("UPDATE operator_plane_commands SET status='cleared',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE stream_sequence<=? AND status='queued' AND method_id IS NULL AND action_id LIKE ?", (now, now, reason_json, cutoff, f"oem.{axis}.%"))
                        conn.execute("UPDATE serial206_movement_commands SET state='cleared',state_version=state_version+1,finished_at=? WHERE state='queued' AND method_id IS NULL AND axis_scope=?", (now, axis))
                        for method_id in affected_method_ids:
                            conn.execute("UPDATE operator_plane_commands SET status='cleared',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE method_id=? AND status='queued'", (now, now, reason_json, method_id))
                            conn.execute("UPDATE serial206_movement_commands SET state='cleared',state_version=state_version+1,finished_at=? WHERE method_id=? AND state='queued'", (now, method_id))
                    method_target = "aborting" if aggregate_interrupt else "stopping"
                    for method_id in affected_method_ids:
                        conn.execute("UPDATE operator_plane_methods SET status=?,version=version+1,updated_at=? WHERE method_id=? AND status NOT IN ('completed','failed','cancelled','stopped','aborted','interrupted','recovery_required')", (method_target, now, method_id))
                        self._insert_transition(conn, event_kind="method_derived", method_id=method_id, state=method_target, payload={"reason": action_id, "cutoff": cutoff})
                    if aggregate_interrupt:
                        conn.execute(
                            "UPDATE serial206_axis_authority SET interrupt_epoch=interrupt_epoch+1,lifecycle_state='reconciliation_required',reference_state='reconciliation_required',prepared_board_epoch=NULL,state_version=state_version+1,updated_at=? WHERE axis IN ('y','z','gripper')",
                            (now,),
                        )
                    elif axis in {"y", "z"}:
                        delivered_y_stop = axis == "y" and request.get("controller_stop_delivered") is True
                        if delivered_y_stop:
                            conn.execute(
                                "UPDATE serial206_axis_authority SET interrupt_epoch=interrupt_epoch+1,lifecycle_state='reconciliation_required',reference_state='reconciliation_required',prepared_board_epoch=NULL,state_version=state_version+1,updated_at=? WHERE axis='y'",
                                (now,),
                            )
                        else:
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
                    active_id = active_ids[0] if active_ids else None
                    response = {"schema_version": "bioxp.operator_interrupt_receipt.v1", "robot_identity": ROBOT_IDENTITY, "ownership_generation": int(state.get("ownership_generation") or 0), "interrupt_id": interrupt_id, "action_id": action_id, "scope": "aggregate" if action_id in {"oem.abort_all", "oem.z.abort"} else axis, "cutoff": cutoff, "active_command_id": active_id, "active_command_ids": active_ids, "global_safety_epoch": global_epoch, "x_safety_epoch": x_epoch, "y_safety_epoch": y_epoch, "z_safety_epoch": z_epoch, "oem_abort_latched": action_id in {"oem.abort_all", "oem.z.abort"}, "controller_stop_attempted": False, "controller_stop_acknowledged": False, "physical_effect_verified": False, "persistence_state": "committed", "transition_sequence": transition}
                    self._store_idempotency(conn, kind="interrupt", key=key, fingerprint=fp, response=response)
            except sqlite3.OperationalError as exc:
                if "locked" not in str(exc).lower() and "busy" not in str(exc).lower():
                    raise
                return {"schema_version": "bioxp.operator_interrupt_receipt.v1", "robot_identity": ROBOT_IDENTITY, "ownership_generation": int(state.get("ownership_generation") or 0), "interrupt_id": str(uuid.uuid4()), "action_id": action_id, "scope": "aggregate" if action_id in {"oem.abort_all", "oem.z.abort"} else AXIS_BY_ACTION.get(action_id), "oem_abort_latched": action_id in {"oem.abort_all", "oem.z.abort"}, "controller_stop_attempted": False, "controller_stop_acknowledged": False, "physical_effect_verified": False, "persistence_state": "lock_timeout", "recovery_hold": True}
        self._wake.set()
        return response

    def finalize_interrupt(self, *, idempotency_key: str, receipt: Mapping[str, Any], attempted: bool, acknowledged: bool, response: Any, error: str | None = None) -> dict[str, Any]:
        interrupt_id = str(receipt.get("interrupt_id") or "")
        active_id = receipt.get("active_command_id")
        active_ids = [str(value) for value in receipt.get("active_command_ids", []) if value]
        if active_id and str(active_id) not in active_ids:
            active_ids.append(str(active_id))
        terminal_state = "aborted" if bool(receipt.get("oem_abort_latched")) else "stopped"
        with self._transaction() as conn:
            saved = conn.execute("SELECT response_json FROM operator_plane_idempotency WHERE operation_kind='interrupt' AND idempotency_key=?", (idempotency_key,)).fetchone()
            current = _json_load(saved[0], dict(receipt)) if saved else dict(receipt)
            current.update({
                "controller_stop_attempted": bool(attempted),
                "controller_stop_acknowledged": bool(acknowledged),
                "physical_effect_verified": False,
                "controller_response": _bounded_json(response, 131072),
                "error": error,
                "persistence_state": "committed" if acknowledged else "recovery_required",
            })
            affected_method_ids: set[str] = set()
            for active_id in active_ids:
                row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (str(active_id),)).fetchone()
                if row is not None and str(row["status"]) in {"dispatched", "stop_requested", "abort_requested"}:
                    final_state = terminal_state if acknowledged else "interrupted"
                    terminal_payload = {"interrupt_id": interrupt_id, "controller_acknowledged": bool(acknowledged), "error": error}
                    conn.execute("UPDATE operator_plane_commands SET status=?,version=version+1,finished_at=?,updated_at=?,terminal_json=?,controller_acknowledged=? WHERE command_id=?", (final_state, _now(), _now(), _canonical(terminal_payload), int(acknowledged), str(active_id)))
                    if row["method_id"]:
                        affected_method_ids.add(str(row["method_id"]))
                    self._update_z_home_authority(conn, command_id=str(active_id), ownership_generation=int(row["ownership_generation"]), status=final_state, payload=terminal_payload, source_noop=False)
                    canonical_state = "interrupted" if acknowledged else "ambiguous"
                    conn.execute("UPDATE serial206_movement_commands SET state=?,state_version=state_version+1,finished_at=?,terminal_receipt_id=? WHERE command_id=? AND state='interrupting'", (canonical_state, _now(), interrupt_id, str(active_id)))
                    transition = self._insert_transition(conn, event_kind="interrupt_terminal", command_id=str(active_id), method_id=row["method_id"], state=final_state, payload={"interrupt_id": interrupt_id, "acknowledged": bool(acknowledged), "error": error})
                    current.setdefault("terminal_transition_sequences", []).append(transition)
                    conn.execute("UPDATE operator_plane_lane SET active_command_id=NULL,active_attempt_id=NULL,updated_at=? WHERE singleton=1 AND active_command_id=?", (_now(), str(active_id)))
                    conn.execute("INSERT INTO operator_plane_outbox(outbox_id,command_id,transition_sequence,state,payload_json,updated_at) VALUES(?,?,?,?,?,?)", (str(uuid.uuid4()), str(active_id), transition, "pending", _canonical(terminal_payload), _now()))
            for method_id in affected_method_ids:
                self._derive_method(conn, method_id)
            if not acknowledged:
                conn.execute("UPDATE operator_plane_safety SET recovery_hold=1,recovery_version=recovery_version+1,updated_at=? WHERE singleton=1", (_now(),))
            conn.execute("UPDATE operator_plane_idempotency SET response_json=? WHERE operation_kind='interrupt' AND idempotency_key=?", (_canonical(current), idempotency_key))
        if acknowledged:
            self.clear_priority_fence()
        self._wake.set()
        return current

    def clear_priority_fence(self) -> None:
        self._priority_fence.clear()

    def mark_y_reconciliation_required(self, *, receipt_id: str) -> None:
        with self._transaction() as conn:
            conn.execute(
                "UPDATE serial206_axis_authority SET lifecycle_state='reconciliation_required',reference_state='reconciliation_required',prepared_board_epoch=NULL,last_receipt_id=?,interrupt_epoch=interrupt_epoch+1,state_version=state_version+1,updated_at=? WHERE axis='y'",
                (str(receipt_id), _now()),
            )

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
        if self._owner_acquired:
            try:
                with self._transaction(timeout_ms=100) as conn:
                    conn.execute("UPDATE operator_plane_lane SET owner_id=NULL,owner_lease_until=NULL,updated_at=? WHERE singleton=1 AND owner_id=?", (_now(), self.owner_id))
            except sqlite3.OperationalError:
                pass

    def _dispatch_loop(self, dispatch_one: Callable[[dict[str, Any]], None]) -> None:
        next_renewal = 0.0
        active_worker: threading.Thread | None = None
        while not self._stop.is_set():
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
            if active_worker is not None:
                if active_worker.is_alive():
                    active_worker.join(timeout=0.05)
                    continue
                active_worker = None
            claimed = None
            try:
                if not self._priority_fence.is_set():
                    claimed = self.claim_next()
                if claimed is not None:
                    active_worker = threading.Thread(
                        target=self._dispatch_worker,
                        args=(dispatch_one, claimed),
                        daemon=True,
                        name=f"bioxp-operator-command-{claimed['command_id']}",
                    )
                    with self._worker_lock:
                        self._workers.add(active_worker)
                    active_worker.start()
                    continue
            except Exception:
                if claimed is not None:
                    try:
                        self.finish(claimed["command_id"], status="ambiguous", payload={"error": "dispatcher_exception", "outcome_unknown": True}, claimed=claimed)
                    except Exception:
                        pass
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
        self.store.start(self._dispatch_one)

    def is_canonical(self, action_id: str) -> bool:
        return action_id in CANONICAL_ACTIONS

    def _state(self) -> dict[str, Any]:
        value = self.machine_state_provider()
        return dict(value) if isinstance(value, Mapping) else {}

    def _action_target(self, action_id: str) -> Mapping[str, Any]:
        target = self.dispatch.get(action_id)
        if not isinstance(target, Mapping):
            raise HTTPException(status_code=503, detail={"error": "canonical_provider_unavailable", "action_id": action_id})
        return target

    async def _dispatch_y_failure_stop(self, command_id: str, *, reason: str) -> dict[str, Any]:
        target = self._action_target("oem.y.stop")
        receipt_id = f"failure-stop:{command_id}"
        token = _DISPATCH_CONTEXT.set({"operator_command_id": receipt_id, "operator_interrupt_id": receipt_id, "idempotency_key": receipt_id, "expected_ownership_generation": int(self._state().get("ownership_generation") or 0), "action_id": "oem.y.stop"})
        response: Any = None
        status_code = 500
        error: str | None = None
        try:
            status_code, response = await _dispatch_asgi(self.app, str(target["method"]), str(target["path"]), dict(target.get("fixed_inputs") or {}), target["locations"])
        except Exception as exc:
            error = f"failure_stop_exception:{type(exc).__name__}"
        finally:
            _DISPATCH_CONTEXT.reset(token)
        acknowledged = bool(200 <= int(status_code) < 300 and isinstance(response, Mapping) and response.get("ok") is True and response.get("double_stop_acknowledged") is True and response.get("terminal_speed_zero") is True)
        await asyncio.to_thread(self.store.mark_y_reconciliation_required, receipt_id=receipt_id)
        return {"attempted": True, "acknowledged": acknowledged, "reason": reason, "receipt_id": receipt_id, "http_status": int(status_code), "response": _bounded_json(response, 131072), "error": error}

    def _dispatch_one(self, claimed: dict[str, Any]) -> None:
        command_id = str(claimed["command_id"])
        action_id = str(claimed["action_id"])
        state = self._state()
        requested = dict(claimed["requested_inputs"])
        effective = dict(claimed["effective_inputs"])
        if int(state.get("ownership_generation") or 0) != int(claimed["ownership_generation"]):
            self.store.finish(command_id, status="failed", payload={"error": "ownership_generation_changed_before_dispatch", "dispatch_attempt_id": claimed["dispatch_attempt_id"], "delivery_attempted": False}, claimed=claimed)
            return
        noop, reason = _source_noop(action_id, requested, effective, state)
        if action_id.startswith("oem.x.") and not self.store.z_home_authority_valid(state):
            self.store.finish(command_id, status="failed", payload={"error": "z_home_authority_lost_before_dispatch", "dispatch_attempt_id": claimed["dispatch_attempt_id"]}, claimed=claimed)
            return
        if noop:
            self.store.finish(command_id, status="completed", payload={"source_noop": True, "source_noop_reason": reason, "requested_inputs": requested, "effective_inputs": effective, "position_before": _axis_position(state, AXIS_BY_ACTION.get(action_id, "z")), "position_after": _axis_position(state, AXIS_BY_ACTION.get(action_id, "z"))}, source_noop=True, source_noop_reason=reason, claimed=claimed)
            return
        if self.store._priority_fence.is_set():
            self.store.finish(command_id, status="interrupted", payload={"reason": "interrupt_fence_won_before_provider"}, claimed=claimed)
            return
        target = self._action_target(action_id)
        token = _DISPATCH_CONTEXT.set({"operator_command_id": command_id, "idempotency_key": f"dispatch:{claimed['dispatch_attempt_id']}", "expected_ownership_generation": claimed["ownership_generation"], "action_id": action_id})
        try:
            status_code, response = asyncio.run(_dispatch_asgi(self.app, str(target["method"]), str(target["path"]), {**dict(target.get("fixed_inputs") or {}), **effective}, target["locations"]))
        except Exception as exc:
            failure_stop = asyncio.run(self._dispatch_y_failure_stop(command_id, reason=f"provider_exception:{type(exc).__name__}")) if action_id in {"oem.y.move_steps", "oem.y.move_absolute"} else None
            self.store.finish(command_id, status="ambiguous", payload={"error": f"provider_exception:{type(exc).__name__}", "detail": str(exc)[:500], "outcome_unknown": True, "failure_stop": failure_stop}, claimed=claimed)
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
            )
            terminal_token = _DISPATCH_CONTEXT.set({"operator_command_id": command_id, "idempotency_key": f"terminalize:{claimed['dispatch_attempt_id']}", "expected_ownership_generation": claimed["ownership_generation"], "action_id": action_id})
            try:
                terminalizer = getattr(self.app.state, "serial206_y_terminalizer", None)
                if not callable(terminalizer):
                    raise RuntimeError("serial206_y_terminalizer_not_bound")
                terminalized = terminalizer(dict(response), 20.0)
                if not isinstance(terminalized, Mapping):
                    raise RuntimeError("serial206_y_terminalizer_returned_invalid_payload")
                response = dict(terminalized)
                status_code = 200
                finish_claim = None
            except Exception as exc:
                failure_stop = asyncio.run(self._dispatch_y_failure_stop(command_id, reason=f"y_terminalizer_exception:{type(exc).__name__}"))
                self.store.finish(command_id, status="ambiguous", payload={"error": f"y_terminalizer_exception:{type(exc).__name__}", "outcome_unknown": True, "failure_stop": failure_stop}, claimed=None)
                return
            finally:
                _DISPATCH_CONTEXT.reset(terminal_token)
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
        target_move = action_id in {"oem.x.move_steps", "oem.x.move_absolute", "oem.y.move_steps", "oem.y.move_absolute", "oem.z.move_steps", "oem.z.move_absolute", "oem.z.clear"}
        home_action = action_id in {"oem.x.manual_panel_home", "oem.y.manual_panel_home", "oem.z.manual_home"}
        source_completed_noop = bool(
            isinstance(response, Mapping)
            and response.get("state") == "completed"
            and isinstance(response.get("result"), Mapping)
            and response["result"].get("source_noop") is True
            and response["result"].get("command_sent") is False
        )
        y_target_move = action_id in {"oem.y.move_steps", "oem.y.move_absolute"}
        y_disposition = (
            _y_absolute_terminal_disposition(
                completion_class=completion_class,
                event_128=event128,
                target_steps=effective_target_steps,
                observed_position_steps=observed_position_steps,
                terminal_speed_zero=terminal_speed_zero,
            )
            if y_target_move and not source_completed_noop
            else None
        )
        completion_proven = (
            source_completed_noop or y_disposition == "completed"
            if y_target_move
            else source_completed_noop or (event128 and position_readback)
            if target_move
            else home_proof
            if home_action
            else True
        )

        embedded_failure_stop = response.get("failure_stop") if isinstance(response, Mapping) and isinstance(response.get("failure_stop"), Mapping) else None
        if y_target_move and not (ok and completion_proven) and embedded_failure_stop is None:
            payload["failure_stop"] = asyncio.run(self._dispatch_y_failure_stop(command_id, reason=str(completion_class or "controller_failure")))
        elif embedded_failure_stop is not None:
            payload["failure_stop"] = _bounded_json(embedded_failure_stop, 131072)
            self.store.mark_y_reconciliation_required(receipt_id=f"failure-stop:{command_id}")

        if ok and completion_proven:
            self.store.finish(command_id, status="completed", payload={**payload, "completion_class": completion_class, "terminal_speed_zero": terminal_speed_zero}, source_noop=source_completed_noop, source_noop_reason="oem_same_effective_target_noop" if source_completed_noop else None, remote_acknowledged=True, controller_acknowledged=controller_ack, claimed=finish_claim)
        elif ok and y_disposition == "ambiguous":
            self.store.finish(command_id, status="ambiguous", payload={**payload, "completion_class": completion_class, "terminal_speed_zero": terminal_speed_zero, "error": "missing_addressed_event_128", "outcome_unknown": True}, remote_acknowledged=True, controller_acknowledged=controller_ack, claimed=finish_claim)
        elif ok:
            self.store.finish(command_id, status="failed", payload={**payload, "completion_class": completion_class, "terminal_speed_zero": terminal_speed_zero, "error": "missing_addressed_event_128" if not event128 else "missing_terminal_position_readback" if target_move and not position_readback else "missing_terminal_speed_zero" if y_target_move else "missing_oem_home_completion" if home_action else "completion_unproven"}, remote_acknowledged=True, controller_acknowledged=controller_ack, claimed=finish_claim)
        else:
            self.store.finish(command_id, status="failed", payload={**payload, "completion_class": completion_class, "terminal_speed_zero": terminal_speed_zero}, remote_acknowledged=False, controller_acknowledged=controller_ack, claimed=finish_claim)

    def _install_routes(self) -> None:
        router = self.router

        @router.post("/commands")
        async def admit_command(request: CommandRequest) -> dict[str, Any]:
            return await asyncio.to_thread(self.store.admit_command, request.model_dump(), state=self._state())

        @router.post("/methods")
        async def admit_method(request: MethodRequest, http_request: Request) -> dict[str, Any]:
            content_length = http_request.headers.get("content-length")
            try:
                body_bytes = int(content_length) if content_length is not None else None
            except ValueError as exc:
                raise HTTPException(status_code=400, detail="Invalid Content-Length") from exc
            if body_bytes is not None and body_bytes > 1_048_576:
                raise HTTPException(status_code=413, detail="BioXP method document exceeds the 1 MiB limit")
            return await asyncio.to_thread(self.store.admit_method, request.model_dump(), state=self._state())

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

    async def admit_strict_method(self, request: Mapping[str, Any]) -> dict[str, Any]:
        method_action_id = str(request.get("method_action_id"))
        if method_action_id not in {"oem.xy.move_absolute", "oem.xy.home"}:
            raise HTTPException(status_code=404, detail="unknown v2 operator method")
        state = self._state()
        expected_generation = int(request.get("expected_ownership_generation") or 0)
        actual_generation = int(state.get("ownership_generation") or 0)
        if expected_generation != actual_generation:
            raise HTTPException(status_code=409, detail={"error": "ownership_generation_conflict", "expected": expected_generation, "actual": actual_generation})
        expected_epochs = {str(key): int(value) for key, value in dict(request.get("expected_board_epoch_by_board") or {}).items()}
        actual_epochs = _active_board_epochs(state, method_action_id)
        if expected_epochs != actual_epochs:
            raise HTTPException(status_code=409, detail={"error": "board_epoch_conflict", "expected": expected_epochs, "actual": actual_epochs})
        raw_inputs = dict(request.get("inputs") or {})
        inputs = ({"x": raw_inputs.get("x_steps"), "y": raw_inputs.get("y_steps")} if method_action_id == "oem.xy.move_absolute" else raw_inputs)
        validated_inputs = _validate_inputs(method_action_id, inputs)
        generic = {
            "schema_version": "bioxp.operator_method_request.v1",
            "name": method_action_id,
            "idempotency_key": str(request["idempotency_key"]),
            "expected_ownership_generation": expected_generation,
            "expected_board_epoch_by_board": expected_epochs,
            "failure_policy": "fail_fast",
            "steps": [{"action_id": method_action_id, "inputs": validated_inputs, "repeat": 1}],
            "metadata": {"method_action_id": method_action_id},
        }
        return await asyncio.to_thread(self.store.admit_method, generic, state=state)

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
        interrupt_attempt_id = str(uuid.uuid4())
        action_id = "oem.y.stop"
        state = self._state()
        self.store._priority_fence.set()
        target = self._action_target(action_id)
        token = _DISPATCH_CONTEXT.set({"operator_command_id": interrupt_attempt_id, "operator_interrupt_id": interrupt_attempt_id, "idempotency_key": f"interrupt-attempt:{interrupt_attempt_id}", "expected_ownership_generation": int(state.get("ownership_generation") or 0), "action_id": action_id})
        response: Any = None
        acknowledged = False
        delivered = False
        error: str | None = None
        try:
            status_code, response = await _dispatch_asgi(self.app, str(target["method"]), str(target["path"]), dict(target.get("fixed_inputs") or {}), target["locations"])
            delivered = True
            acknowledged = 200 <= int(status_code) < 300 and not (isinstance(response, Mapping) and response.get("ok") is False)
            if not acknowledged:
                error = f"controller_interrupt_http_{status_code}"
        except Exception as exc:
            error = f"controller_interrupt_exception:{type(exc).__name__}"
        finally:
            _DISPATCH_CONTEXT.reset(token)
        internal_key = f"interrupt-attempt:{interrupt_attempt_id}"
        internal_request = {**dict(request), "idempotency_key": internal_key, "expected_ownership_generation": int(state.get("ownership_generation") or 0), "controller_stop_delivered": delivered}

        async def persist_fallback(reason: str, receipt: Mapping[str, Any] | None = None) -> dict[str, Any]:
            stop = response.get("stop") if isinstance(response, Mapping) and isinstance(response.get("stop"), Mapping) else {}
            terminal_speed = response.get("terminal_speed") if isinstance(response, Mapping) and isinstance(response.get("terminal_speed"), Mapping) else {}
            fallback = {
                "schema_version": "bioxp.operator_interrupt_receipt.v2",
                "robot_identity": ROBOT_IDENTITY,
                "interrupt_attempt_id": interrupt_attempt_id,
                "interrupt_id": str((receipt or {}).get("interrupt_id") or interrupt_attempt_id),
                "action_id": action_id,
                "scope": "y",
                "ownership_generation": int(state.get("ownership_generation") or 0),
                "active_command_id": (receipt or {}).get("active_command_id"),
                "active_command_ids": list((receipt or {}).get("active_command_ids") or []),
                "cutoff": (receipt or {}).get("cutoff"),
                "controller_stop_attempted": True,
                "controller_stop_acknowledged": acknowledged,
                "controller_stop_delivered": delivered,
                "controller_response": _bounded_json(response, 131072),
                "first_stop_ack": _bounded_json(stop.get("first_delivery"), 8192),
                "second_stop_ack": _bounded_json(stop.get("second_delivery"), 8192),
                "terminal_speed_evidence": _bounded_json(terminal_speed, 8192),
                "error": error,
                "physical_effect_verified": False,
                "persistence_state": "fsync_fallback",
                "recovery_hold": True,
                "authority_snapshot": _bounded_json(self._state().get("serial206_initialization_provider"), 65536),
            }
            try:
                return await asyncio.to_thread(self.store.append_y_interrupt_fallback, fallback, reason=reason)
            except Exception as exc:
                raise HTTPException(status_code=503, detail={"error": "y_stop_persistence_and_fallback_failed", "interrupt_attempt_id": interrupt_attempt_id, "controller_stop_attempted": True, "controller_stop_acknowledged": acknowledged, "fallback_error": type(exc).__name__}) from exc

        try:
            receipt = await asyncio.to_thread(self.store.begin_interrupt, action_id, state=self._state(), request=internal_request)
        except Exception as exc:
            return await persist_fallback(f"begin_interrupt_{type(exc).__name__}")
        if str(receipt.get("persistence_state")) != "committed":
            return await persist_fallback(f"begin_interrupt_{receipt.get('persistence_state')}", receipt)
        try:
            finalized = await asyncio.to_thread(self.store.finalize_interrupt, idempotency_key=internal_key, receipt=receipt, attempted=True, acknowledged=acknowledged, response=response, error=error)
        except Exception as exc:
            return await persist_fallback(f"finalize_interrupt_{type(exc).__name__}", receipt)
        return {**finalized, "interrupt_attempt_id": interrupt_attempt_id}

    async def _invoke_controller_interrupt(self, action_id: str, *, receipt: Mapping[str, Any], idempotency_key: str) -> dict[str, Any]:
        provider_action = "oem.abort_all" if action_id in {"oem.abort_all", "oem.z.abort"} else action_id
        target = self._action_target(provider_action)
        interrupt_id = str(receipt.get("interrupt_id") or "")
        token = _DISPATCH_CONTEXT.set({"operator_command_id": str(receipt.get("active_command_id") or interrupt_id), "operator_interrupt_id": interrupt_id, "idempotency_key": idempotency_key, "expected_ownership_generation": int(self._state().get("ownership_generation") or 0), "action_id": action_id})
        attempted = True
        response: Any = None
        acknowledged = False
        error: str | None = None
        try:
            status_code, response = await _dispatch_asgi(self.app, str(target["method"]), str(target["path"]), dict(target.get("fixed_inputs") or {}), target["locations"])
            acknowledged = 200 <= int(status_code) < 300 and not (isinstance(response, Mapping) and response.get("ok") is False)
            if not acknowledged:
                error = f"controller_interrupt_http_{status_code}"
        except Exception as exc:
            error = f"controller_interrupt_exception:{type(exc).__name__}"
        finally:
            _DISPATCH_CONTEXT.reset(token)
        return await asyncio.to_thread(
            self.store.finalize_interrupt,
            idempotency_key=idempotency_key,
            receipt=receipt,
            attempted=attempted,
            acknowledged=acknowledged,
            response=response,
            error=error,
        )

    async def compat_invoke(self, action_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        state = self._state()
        expected_generation = int(payload.get("expected_generation") or -1)
        actual_generation = int(state.get("ownership_generation") or 0)
        if expected_generation != actual_generation:
            raise HTTPException(status_code=409, detail={"error": "ownership_generation_mismatch", "expected": expected_generation, "actual": actual_generation})
        if action_id in INTERRUPT_ACTIONS:
            receipt = await asyncio.to_thread(self.store.begin_interrupt, action_id, state=self._state(), request=payload)
            if bool(receipt.get("idempotent_replay")):
                return receipt
            return await self._invoke_controller_interrupt(
                action_id,
                receipt=receipt,
                idempotency_key=str(payload["idempotency_key"]),
            )
        request = {"schema_version": COMMAND_SCHEMA, "idempotency_key": payload["idempotency_key"], "expected_ownership_generation": int(payload["expected_generation"]), "action_id": action_id, "inputs": dict(payload.get("inputs") or {})}
        return await asyncio.to_thread(self.store.admit_command, request, state=self._state())

    def stop(self) -> None:
        self.store.stop()
