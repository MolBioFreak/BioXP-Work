"""Durable robot-owned X/Z command and method execution plane.

This module owns normal movement admission, global ordering, lifecycle rows,
recovery, and the independent Stop/Abort lane. Existing source-shaped provider
routes remain the only hardware execution boundary.
"""
from __future__ import annotations

import asyncio
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
from pydantic import BaseModel, ConfigDict, Field, StrictInt
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
from .oem_runtime_store import migrate_runtime_database_v2

COMMAND_SCHEMA = "bioxp.operator_command_request.v1"
RECEIPT_SCHEMA = "bioxp.operator_command_receipt.v1"
METHOD_SCHEMA = "bioxp.operator_method_request.v1"
METHOD_RECEIPT_SCHEMA = "bioxp.operator_method_receipt.v1"
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
    "queued", "dispatched", "stop_requested", "abort_requested", "completed",
    "failed", "ambiguous", "stopped", "aborted", "cancelled", "cleared",
    "interrupted",
})
COMMAND_NONTERMINAL = frozenset({"queued", "dispatched", "stop_requested", "abort_requested"})
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
    "oem.y.move_steps",
    "oem.y.move_absolute",
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
    "oem.z.manual_home": "z",
    "oem.z.clear": "z",
    "oem.z.move_steps": "z",
    "oem.z.move_absolute": "z",
    "oem.x.stop": "x",
    "oem.y.stop": "y",
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
        "oem.z.move_steps": {"steps"},
        "oem.x.move_steps": {"steps"},
        "oem.y.move_steps": {"steps"},
        "oem.z.move_absolute": {"position_steps"},
        "oem.x.move_absolute": {"position_steps"},
        "oem.y.move_absolute": {"target_steps"},
    }
    unknown = sorted(set(inputs) - allowed.get(action_id, set()))
    if unknown:
        raise HTTPException(status_code=422, detail={"error": "unknown_command_inputs", "unknown": unknown})
    result = dict(inputs)
    if action_id.endswith("move_steps"):
        steps = result.get("steps")
        maximum = 102956 if action_id.startswith("oem.y.") else 160000
        if type(steps) is not int or not -maximum <= steps <= maximum:
            raise HTTPException(status_code=422, detail={"error": "invalid_steps", "required": f"integer in [-{maximum},{maximum}]"})
    if action_id.endswith("move_absolute"):
        position = result.get("target_steps") if action_id.startswith("oem.y.") else result.get("position_steps")
        maximum = 90263 if action_id.startswith("oem.x.") else 102956 if action_id.startswith("oem.y.") else 160000
        if type(position) is not int or not 0 <= position <= maximum:
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
    authority = provider.get("x_authority" if axis == "x" else "z_authority") if isinstance(provider, Mapping) else None
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


def _source_noop(action_id: str, requested: Mapping[str, Any], effective: Mapping[str, Any], state: Mapping[str, Any]) -> tuple[bool, str | None]:
    axis = AXIS_BY_ACTION.get(action_id)
    if axis is None:
        return False, None
    current = _axis_position(state, axis)
    if action_id.endswith("move_steps") and int(requested.get("steps", 1)) == 0:
        return True, "zero_relative_steps"
    if action_id.endswith("move_absolute"):
        target = effective.get("target_steps") if action_id.startswith("oem.y.") else effective.get("position_steps")
        if type(target) is int and current is not None and current == target:
            return True, "same_effective_absolute_target"
    if action_id == "oem.z.manual_home" and current == 0 and bool(_state_value(state, "MotorHome", "motor_home", "home_latched")):
        return True, "already_home"
    if action_id == "oem.x.manual_panel_home" and current == 0 and bool(_state_value(state, "MotorHome", "motor_home", "home_latched")):
        return True, "already_home"
    if action_id.endswith("move_absolute") and current is not None:
        target = effective.get("target_steps") if action_id.startswith("oem.y.") else effective.get("position_steps")
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
        self.connection = sqlite3.connect(self.path, timeout=2.0, isolation_level=None, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        self._configure()
        migrate_runtime_database_v2(self.connection, self.root)
        self._schema()
        self._owner_acquired = self._acquire_owner()
        if self._owner_acquired:
            self._startup_recover()

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
                INSERT OR IGNORE INTO operator_plane_lane(singleton, dispatcher_epoch, updated_at) VALUES(1, 1, strftime('%s','now'));
                INSERT OR IGNORE INTO operator_plane_safety(singleton, global_epoch, x_epoch, z_epoch, recovery_epoch, recovery_version, recovery_hold, updated_at)
                    VALUES(1, 0, 0, 0, 0, 1, 0, strftime('%s','now'));
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
            recovery_epoch = int(row["recovery_epoch"]) + 1
            active = conn.execute(
                "SELECT command_id,method_id,status FROM operator_plane_commands WHERE status IN ('dispatched','stop_requested','abort_requested')"
            ).fetchall()
            if active:
                for item in active:
                    command_id = str(item["command_id"])
                    conn.execute(
                        "UPDATE operator_plane_commands SET status='interrupted',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE command_id=?",
                        (_now(), _now(), _canonical({"reason": "process_owner_loss", "outcome_unknown": True}), command_id),
                    )
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
    ) -> None:
        axis = AXIS_BY_ACTION.get(action_id)
        motor_by_axis = {"y": 0, "z": 1}
        board_scope = {"4": [motor_by_axis[axis]]} if axis in motor_by_axis else {}
        canonical_hash = _digest(dict(inputs))
        expected_epochs = {"4": 0} if axis in motor_by_axis else {}
        interrupt_epochs = {axis: 0} if axis else {}
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
                None,
                0,
                0,
                axis,
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
        if axis in {"y", "z"}:
            resources = [f"axis:{axis}", f"motor:4:{0 if axis == 'y' else 1}"]
        elif axis is not None:
            resources = [f"axis:{axis}"]
        else:
            resources = ["robot:operator"]
        conn.executemany(
            "INSERT INTO serial206_command_resources(command_id,resource_key) VALUES(?,?)",
            [(command_id, resource) for resource in resources],
        )

    def _command_response(self, row: sqlite3.Row, *, transition_sequence: int | None = None) -> dict[str, Any]:
        if transition_sequence is None:
            transition_row = self.connection.execute("SELECT MAX(transition_sequence) FROM operator_plane_transitions WHERE command_id=?", (str(row["command_id"]),)).fetchone()
            transition_sequence = transition_row[0] if transition_row and transition_row[0] is not None else None
        return {
            "schema_version": RECEIPT_SCHEMA,
            "command_id": str(row["command_id"]),
            "method_id": row["method_id"],
            "method_sequence": row["method_sequence"],
            "stream_sequence": int(row["stream_sequence"]),
            "action_id": str(row["action_id"]),
            "status": str(row["status"]),
            "ownership_generation": int(row["ownership_generation"]),
            "requested_inputs": _json_load(row["requested_json"], {}),
            "effective_inputs": _json_load(row["effective_json"], {}),
            "queued_at": float(row["queued_at"]),
            "dispatched_at": row["dispatched_at"],
            "finished_at": row["finished_at"],
            "source_noop": bool(row["source_noop"]),
            "source_noop_reason": row["source_noop_reason"],
            "remote_acknowledged": bool(row["remote_acknowledged"]),
            "controller_acknowledged": bool(row["controller_acknowledged"]),
            "physical_effect_verified": bool(row["physical_effect_verified"]),
            "terminal_evidence": _json_load(row["terminal_json"], None),
            "transition_sequence": transition_sequence,
        }

    def _method_response(self, row: sqlite3.Row) -> dict[str, Any]:
        transition_row = self.connection.execute("SELECT MAX(transition_sequence) FROM operator_plane_transitions WHERE method_id=?", (str(row["method_id"]),)).fetchone()
        transition_sequence = transition_row[0] if transition_row and transition_row[0] is not None else None
        return {
            "schema_version": METHOD_RECEIPT_SCHEMA,
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
        }

    def admit_command(self, request: Mapping[str, Any], *, state: Mapping[str, Any]) -> dict[str, Any]:
        action_id = str(request.get("action_id") or "")
        if action_id not in ALLOWED_ACTIONS:
            raise HTTPException(status_code=422, detail={"error": "action_not_allowed", "action_id": action_id})
        inputs = _validate_inputs(action_id, request.get("inputs", {}))
        expected_generation = int(request["expected_ownership_generation"])
        canonical_request = {"schema_version": COMMAND_SCHEMA, "operation_kind": "command", "expected_ownership_generation": expected_generation, "action_id": action_id, "inputs": inputs}
        fingerprint = _digest(canonical_request)
        key = str(request["idempotency_key"])
        saved = self.idempotency_checked("command", key, fingerprint)
        if saved is not None:
            saved["idempotent_replay"] = True
            return saved
        actual_generation = int(state.get("ownership_generation") or 0)
        if expected_generation != actual_generation:
            raise HTTPException(status_code=409, detail={"error": "ownership_generation_mismatch", "expected": expected_generation, "actual": actual_generation})
        if action_id.startswith("oem.x.") and not _z_home_valid(state):
            raise HTTPException(status_code=409, detail={"error": "z_home_authority_required", "reason": "Standalone X requires current Z Home authority"})
        with self._transaction() as conn:
            replay = self._saved_idempotency(conn, "command", key, fingerprint)
            if replay is not None:
                return replay
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
            )
            transition = self._insert_transition(conn, event_kind="command_admitted", command_id=command_id, state="queued", payload={"stream_sequence": sequence, "action_id": action_id})
            response = {**row, "schema_version": RECEIPT_SCHEMA, "transition_sequence": transition, "remote_acknowledged": False, "controller_acknowledged": False, "physical_effect_verified": False, "source_noop": False, "source_noop_reason": None, "terminal_evidence": None, "dispatched_at": None, "finished_at": None, "idempotent_replay": False}
            self._store_idempotency(conn, kind="command", key=key, fingerprint=fingerprint, response=response, command_id=command_id)
        self._wake.set()
        return response

    def admit_method(self, request: Mapping[str, Any], *, state: Mapping[str, Any]) -> dict[str, Any]:
        expected_generation = int(request["expected_ownership_generation"])
        expanded: list[tuple[str, dict[str, Any]]] = []
        seen_home = False
        method_requires_home = False
        for step in request.get("steps", []):
            action_id = str(step.get("action_id") or "")
            if action_id not in ALLOWED_ACTIONS:
                raise HTTPException(status_code=422, detail={"error": "method_action_not_allowed", "action_id": action_id})
            if action_id.startswith("oem.x.") and not (_z_home_valid(state) or seen_home):
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
        source = {"schema_version": METHOD_SCHEMA, "name": str(request["name"]), "failure_policy": "fail_fast", "steps": [{"action_id": a, "inputs": i} for a, i in expanded], "metadata": dict(request.get("metadata") or {})}
        digest = _digest(source)
        canonical_request = {"schema_version": METHOD_SCHEMA, "operation_kind": "method", "expected_ownership_generation": expected_generation, "source": source}
        fingerprint = _digest(canonical_request)
        key = str(request["idempotency_key"])
        saved = self.idempotency_checked("method", key, fingerprint)
        if saved is not None:
            saved["idempotent_replay"] = True
            return saved
        if expected_generation != int(state.get("ownership_generation") or 0):
            raise HTTPException(status_code=409, detail={"error": "ownership_generation_mismatch"})
        if method_requires_home:
            raise HTTPException(status_code=409, detail={"error": "z_home_authority_required", "reason": "Method X requires current Z Home or an earlier Z Home step"})
        with self._transaction() as conn:
            replay = self._saved_idempotency(conn, "method", key, fingerprint)
            if replay is not None:
                return replay
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
            for index, (action_id, inputs) in enumerate(expanded, start=1):
                sequence = first_sequence + index - 1
                effective = _effective_inputs(action_id, inputs, state)
                conn.execute(
                    "INSERT INTO operator_plane_commands(command_id,stream_sequence,method_id,method_sequence,action_id,requested_json,effective_json,status,version,ownership_generation,queued_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
                    (str(uuid.uuid4()), sequence, method_id, index, action_id, _canonical(inputs), _canonical(effective), "queued", 1, expected_generation, now, now),
                )
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

    def get_method(self, method_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self.connection.execute("SELECT * FROM operator_plane_methods WHERE method_id=?", (method_id,)).fetchone()
            return self._method_response(row) if row else None

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
            active = self.connection.execute("SELECT * FROM operator_plane_commands WHERE status='dispatched' ORDER BY stream_sequence LIMIT 1").fetchone()
            row = self.connection.execute("SELECT stream_sequence FROM operator_plane_commands WHERE status NOT IN ('completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted') ORDER BY stream_sequence LIMIT 1").fetchone()
            return {"schema_version": "bioxp.operator_queue.v1", "pending_count": count, "pending_bytes": bytes_used, "capacity_commands": COMMAND_CAPACITY, "capacity_bytes": COMMAND_BYTES_CAPACITY, "active_command": self._command_response(active) if active else None, "next_stream_sequence": int(row[0]) if row else None}

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
            return {"schema_version": RECOVERY_SCHEMA, "recovery_epoch": int(safety["recovery_epoch"]), "version": int(safety["recovery_version"]), "hold": bool(safety["recovery_hold"]), "outcome_unknown_command_ids": [str(row["command_id"]) for row in unknown], "affected_method_ids": methods, "queued_count": int(queued), "queued_range": queued_range, "dispatcher_epoch": int(lane["dispatcher_epoch"]) if lane is not None else 0, "global_safety_epoch": int(safety["global_epoch"]), "x_safety_epoch": int(safety["x_epoch"]), "z_safety_epoch": int(safety["z_epoch"]), "available_resolutions": ["resume_undispatched", "cancel_pending"] if safety["recovery_hold"] else []}

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
                + [int(safety["y_epoch"]) for action in axes if action.startswith("oem.y.")]
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
            row = self.connection.execute("SELECT response_json FROM operator_plane_idempotency WHERE operation_kind=? AND idempotency_key=?", (operation_kind, key)).fetchone()
            return _json_load(row[0], None) if row else None

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
            active_resources = {
                str(item[0])
                for item in conn.execute(
                    "SELECT r.resource_key FROM serial206_command_resources r JOIN serial206_movement_commands c ON c.command_id=r.command_id WHERE c.state IN ('dispatched','interrupting')"
                ).fetchall()
            }
            row = None
            for candidate in conn.execute("SELECT * FROM operator_plane_commands WHERE status='queued' ORDER BY stream_sequence").fetchall():
                canonical = conn.execute("SELECT state FROM serial206_movement_commands WHERE command_id=?", (candidate["command_id"],)).fetchone()
                if canonical is not None and str(canonical["state"]) != "queued":
                    continue
                resources = {
                    str(item[0])
                    for item in conn.execute("SELECT resource_key FROM serial206_command_resources WHERE command_id=?", (candidate["command_id"],)).fetchall()
                }
                if resources & active_resources:
                    continue
                if candidate["method_id"]:
                    method = conn.execute("SELECT status FROM operator_plane_methods WHERE method_id=?", (candidate["method_id"],)).fetchone()
                    if method is None or str(method["status"]) in {"paused", "pause_requested", "cancel_requested", "stopping", "aborting", "recovery_required"}:
                        continue
                row = candidate
                break
            if row is None:
                return None
            if row["method_id"]:
                method = conn.execute("SELECT status FROM operator_plane_methods WHERE method_id=?", (row["method_id"],)).fetchone()
                if method is not None and str(method["status"]) == "queued":
                    conn.execute("UPDATE operator_plane_methods SET status='running',version=version+1,updated_at=? WHERE method_id=? AND status='queued'", (_now(), row["method_id"]))
                    self._insert_transition(conn, event_kind="method_running", method_id=str(row["method_id"]), state="running", payload={"first_child_command_id": str(row["command_id"])})
            attempt_id = str(uuid.uuid4())
            epoch = int(lane["dispatcher_epoch"])
            axis_epoch = int(safety[{"x": "x_epoch", "y": "y_epoch", "z": "z_epoch"}.get(AXIS_BY_ACTION.get(str(row["action_id"])) or "", "z_epoch")])
            conn.execute("UPDATE operator_plane_commands SET status='dispatched',version=version+1,dispatch_attempt_id=?,dispatcher_epoch=?,dispatch_global_safety_epoch=?,dispatch_axis_safety_epoch=?,dispatched_at=?,updated_at=? WHERE command_id=? AND status='queued'", (attempt_id, epoch, int(safety["global_epoch"]), axis_epoch, _now(), _now(), row["command_id"]))
            conn.execute("UPDATE serial206_movement_commands SET state='dispatched',state_version=state_version+1,dispatched_at=? WHERE command_id=? AND state='queued'", (_now(), row["command_id"]))
            conn.execute("UPDATE operator_plane_lane SET active_command_id=?,active_attempt_id=?,updated_at=? WHERE singleton=1 AND active_command_id IS NULL", (row["command_id"], attempt_id, _now()))
            self._insert_transition(conn, event_kind="command_dispatched", command_id=str(row["command_id"]), method_id=row["method_id"], state="dispatched", payload={"dispatch_attempt_id": attempt_id, "dispatcher_epoch": epoch})
            claimed = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (row["command_id"],)).fetchone()
            assert claimed is not None
            return {"command_id": str(claimed["command_id"]), "method_id": claimed["method_id"], "method_sequence": claimed["method_sequence"], "action_id": str(claimed["action_id"]), "requested_inputs": _json_load(claimed["requested_json"], {}), "effective_inputs": _json_load(claimed["effective_json"], {}), "dispatch_attempt_id": str(claimed["dispatch_attempt_id"]), "dispatcher_epoch": int(claimed["dispatcher_epoch"]), "dispatch_global_safety_epoch": int(claimed["dispatch_global_safety_epoch"]), "dispatch_axis_safety_epoch": int(claimed["dispatch_axis_safety_epoch"]), "ownership_generation": int(claimed["ownership_generation"])}

    def mark_dispatched(self, command_id: str, *, payload: Mapping[str, Any]) -> dict[str, Any]:
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                return {"command_id": command_id, "status": "missing"}
            if str(row["status"]) != "dispatched":
                return self._command_response(row)
            now = _now()
            evidence = _canonical(dict(payload))
            conn.execute("UPDATE operator_plane_commands SET version=version+1,terminal_json=?,updated_at=? WHERE command_id=? AND version=? AND status='dispatched'", (evidence, now, command_id, int(row["version"])))
            conn.execute("UPDATE serial206_movement_commands SET state_version=state_version+1 WHERE command_id=? AND state='dispatched'", (command_id,))
            self._insert_transition(conn, event_kind="command_pending", command_id=command_id, method_id=row["method_id"], state="dispatched", payload=dict(payload))
            updated = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            assert updated is not None
            result = self._command_response(updated)
        self._wake.set()
        return result

    def finish(self, command_id: str, *, status: str, payload: Mapping[str, Any], source_noop: bool = False, source_noop_reason: str | None = None, remote_acknowledged: bool = False, controller_acknowledged: bool = False, physical_effect_verified: bool = False) -> dict[str, Any]:
        if status not in COMMAND_TERMINAL:
            raise ValueError(status)
        with self._transaction() as conn:
            row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                return {"command_id": command_id, "status": "missing"}
            if str(row["status"]) in COMMAND_TERMINAL:
                return self._command_response(row)
            current_status = str(row["status"])
            if current_status in {"stop_requested", "abort_requested"} and status == "completed":
                status = "aborted" if current_status == "abort_requested" else "stopped"
                payload = {**dict(payload), "completion_after_interrupt_request": True}
            now = _now()
            conn.execute("UPDATE operator_plane_commands SET status=?,version=version+1,source_noop=?,source_noop_reason=?,remote_acknowledged=?,controller_acknowledged=?,physical_effect_verified=?,terminal_json=?,finished_at=?,updated_at=? WHERE command_id=? AND version=? AND status IN ('dispatched','stop_requested','abort_requested')", (status, int(source_noop), source_noop_reason, int(remote_acknowledged), int(controller_acknowledged), int(physical_effect_verified), _canonical(payload), now, now, command_id, int(row["version"])))
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
            conn.execute("UPDATE operator_plane_commands SET status=?,version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE method_id=? AND status='queued'", (child_state, _now(), _now(), _canonical({"reason": reason}), method_id))
        if target != current and target in METHOD_STATES:
            conn.execute("UPDATE operator_plane_methods SET status=?,version=version+1,updated_at=? WHERE method_id=?", (target, _now(), method_id))
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
            timeout_ms = 25
            try:
                with self._transaction(timeout_ms=timeout_ms) as conn:
                    safety = conn.execute("SELECT * FROM operator_plane_safety WHERE singleton=1").fetchone()
                    lane = conn.execute("SELECT * FROM operator_plane_lane WHERE singleton=1").fetchone()
                    interrupt_id = str(uuid.uuid4())
                    axis = AXIS_BY_ACTION.get(action_id)
                    aggregate_interrupt = action_id in {"oem.abort_all", "oem.z.abort"}
                    active_rows = conn.execute(
                        "SELECT DISTINCT c.command_id FROM serial206_movement_commands c LEFT JOIN serial206_command_resources r ON r.command_id=c.command_id WHERE c.state='dispatched' AND (? OR r.resource_key=?)",
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
                    scope_clause = "1=1" if aggregate_interrupt else "action_id LIKE ?"
                    params: tuple[Any, ...] = (cutoff,) if aggregate_interrupt else (cutoff, f"oem.{axis}.%")
                    conn.execute(f"UPDATE operator_plane_commands SET status='cleared',version=version+1,finished_at=?,updated_at=?,terminal_json=? WHERE stream_sequence<=? AND status='queued' AND {scope_clause}", (_now(), _now(), _canonical({"reason": action_id, "cutoff": cutoff}), *params))
                    canonical_scope = "1=1" if aggregate_interrupt else "axis_scope=?"
                    canonical_params: tuple[Any, ...] = () if aggregate_interrupt else (axis,)
                    conn.execute(f"UPDATE serial206_movement_commands SET state='cleared',state_version=state_version+1,finished_at=? WHERE state='queued' AND {canonical_scope}", (_now(), *canonical_params))
                    conn.execute("UPDATE serial206_axis_authority SET interrupt_epoch=interrupt_epoch+1,state_version=state_version+1,updated_at=? WHERE axis IN ('x','y','z')" if aggregate_interrupt else "UPDATE serial206_axis_authority SET interrupt_epoch=interrupt_epoch+1,state_version=state_version+1,updated_at=? WHERE axis=?", (_now(),) if aggregate_interrupt else (_now(), axis))
                    transition = None
                    for active_id in active_ids:
                        row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (str(active_id),)).fetchone()
                        if row is None or str(row["status"]) not in {"dispatched", "stop_requested", "abort_requested"}:
                            continue
                        requested_state = "abort_requested" if aggregate_interrupt else "stop_requested"
                        conn.execute("UPDATE operator_plane_commands SET status=?,version=version+1,interrupt_id=?,interrupt_global_safety_epoch=?,interrupt_axis_safety_epoch=?,updated_at=?,terminal_json=? WHERE command_id=? AND status='dispatched'", (requested_state, interrupt_id, global_epoch, {"x": x_epoch, "y": y_epoch, "z": z_epoch}.get(axis or "", z_epoch), _now(), _canonical({"interrupt_id": interrupt_id, "action_id": action_id}), str(active_id)))
                        conn.execute("UPDATE serial206_movement_commands SET state='interrupting',state_version=state_version+1 WHERE command_id=? AND state='dispatched'", (str(active_id),))
                        transition = self._insert_transition(conn, event_kind="interrupt_requested", command_id=str(active_id), method_id=row["method_id"], state=requested_state, payload={"interrupt_id": interrupt_id, "action_id": action_id, "cutoff": cutoff})
                    active_id = active_ids[0] if active_ids else None
                    response = {"schema_version": "bioxp.operator_interrupt_receipt.v1", "interrupt_id": interrupt_id, "action_id": action_id, "scope": "aggregate" if action_id in {"oem.abort_all", "oem.z.abort"} else axis, "cutoff": cutoff, "active_command_id": active_id, "active_command_ids": active_ids, "global_safety_epoch": global_epoch, "x_safety_epoch": x_epoch, "z_safety_epoch": z_epoch, "oem_abort_latched": action_id in {"oem.abort_all", "oem.z.abort"}, "controller_stop_attempted": False, "controller_stop_acknowledged": False, "physical_effect_verified": False, "persistence_state": "committed", "transition_sequence": transition}
                    self._store_idempotency(conn, kind="interrupt", key=key, fingerprint=fp, response=response)
            except sqlite3.OperationalError as exc:
                if "locked" not in str(exc).lower() and "busy" not in str(exc).lower():
                    raise
                return {"schema_version": "bioxp.operator_interrupt_receipt.v1", "interrupt_id": str(uuid.uuid4()), "action_id": action_id, "scope": "aggregate" if action_id in {"oem.abort_all", "oem.z.abort"} else AXIS_BY_ACTION.get(action_id), "oem_abort_latched": action_id in {"oem.abort_all", "oem.z.abort"}, "controller_stop_attempted": False, "controller_stop_acknowledged": False, "physical_effect_verified": False, "persistence_state": "lock_timeout", "recovery_hold": True}
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
            for active_id in active_ids:
                row = conn.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (str(active_id),)).fetchone()
                if row is not None and str(row["status"]) in {"dispatched", "stop_requested", "abort_requested"}:
                    final_state = terminal_state if acknowledged else "interrupted"
                    conn.execute("UPDATE operator_plane_commands SET status=?,version=version+1,finished_at=?,updated_at=?,terminal_json=?,controller_acknowledged=? WHERE command_id=?", (final_state, _now(), _now(), _canonical({"interrupt_id": interrupt_id, "controller_acknowledged": bool(acknowledged), "error": error}), int(acknowledged), str(active_id)))
                    canonical_state = "interrupted" if acknowledged else "ambiguous"
                    conn.execute("UPDATE serial206_movement_commands SET state=?,state_version=state_version+1,finished_at=?,terminal_receipt_id=? WHERE command_id=? AND state='interrupting'", (canonical_state, _now(), interrupt_id, str(active_id)))
                    transition = self._insert_transition(conn, event_kind="interrupt_terminal", command_id=str(active_id), method_id=row["method_id"], state=final_state, payload={"interrupt_id": interrupt_id, "acknowledged": bool(acknowledged), "error": error})
                    current.setdefault("terminal_transition_sequences", []).append(transition)
                    conn.execute("UPDATE operator_plane_lane SET active_command_id=NULL,active_attempt_id=NULL,updated_at=? WHERE singleton=1 AND active_command_id=?", (_now(), str(active_id)))
            if not acknowledged:
                conn.execute("UPDATE operator_plane_safety SET recovery_hold=1,recovery_version=recovery_version+1,updated_at=? WHERE singleton=1", (_now(),))
            conn.execute("UPDATE operator_plane_idempotency SET response_json=? WHERE operation_kind='interrupt' AND idempotency_key=?", (_canonical(current), idempotency_key))
        if acknowledged:
            self.clear_priority_fence()
        self._wake.set()
        return current

    def clear_priority_fence(self) -> None:
        self._priority_fence.clear()

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
        while not self._stop.is_set():
            if _now() >= next_renewal:
                try:
                    if not self._renew_owner():
                        self._stop.set()
                        break
                except sqlite3.OperationalError:
                    self._stop.set()
                    break
                next_renewal = _now() + 1.0
            claimed = None
            try:
                if not self._priority_fence.is_set():
                    claimed = self.claim_next()
                if claimed is not None:
                    worker = threading.Thread(
                        target=self._dispatch_worker,
                        args=(dispatch_one, claimed),
                        daemon=True,
                        name=f"bioxp-operator-command-{claimed['command_id']}",
                    )
                    with self._worker_lock:
                        self._workers.add(worker)
                    worker.start()
                    continue
            except Exception:
                if claimed is not None:
                    try:
                        self.finish(claimed["command_id"], status="ambiguous", payload={"error": "dispatcher_exception", "outcome_unknown": True})
                    except Exception:
                        pass
            self._wake.wait(timeout=0.05)
            self._wake.clear()

    def _dispatch_worker(self, dispatch_one: Callable[[dict[str, Any]], None], claimed: dict[str, Any]) -> None:
        try:
            dispatch_one(claimed)
        except Exception:
            try:
                self.finish(claimed["command_id"], status="ambiguous", payload={"error": "dispatcher_exception", "outcome_unknown": True})
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

    def _dispatch_one(self, claimed: dict[str, Any]) -> None:
        command_id = str(claimed["command_id"])
        action_id = str(claimed["action_id"])
        state = self._state()
        requested = dict(claimed["requested_inputs"])
        effective = dict(claimed["effective_inputs"])
        if int(state.get("ownership_generation") or 0) != int(claimed["ownership_generation"]):
            self.store.finish(command_id, status="failed", payload={"error": "ownership_generation_changed_before_dispatch", "dispatch_attempt_id": claimed["dispatch_attempt_id"], "delivery_attempted": False})
            return
        noop, reason = _source_noop(action_id, requested, effective, state)
        if action_id.startswith("oem.x.") and not (_z_home_valid(state) or self.store.method_home_established(claimed.get("method_id"), claimed.get("method_sequence"))):
            self.store.finish(command_id, status="failed", payload={"error": "z_home_authority_lost_before_dispatch", "dispatch_attempt_id": claimed["dispatch_attempt_id"]})
            return
        if noop:
            self.store.finish(command_id, status="completed", payload={"source_noop": True, "source_noop_reason": reason, "requested_inputs": requested, "effective_inputs": effective, "position_before": _axis_position(state, AXIS_BY_ACTION.get(action_id, "z")), "position_after": _axis_position(state, AXIS_BY_ACTION.get(action_id, "z"))}, source_noop=True, source_noop_reason=reason)
            return
        if self.store._priority_fence.is_set():
            self.store.finish(command_id, status="interrupted", payload={"reason": "interrupt_fence_won_before_provider"})
            return
        target = self._action_target(action_id)
        token = _DISPATCH_CONTEXT.set({"operator_command_id": command_id, "idempotency_key": f"dispatch:{claimed['dispatch_attempt_id']}", "expected_ownership_generation": claimed["ownership_generation"], "action_id": action_id})
        try:
            status_code, response = asyncio.run(_dispatch_asgi(self.app, str(target["method"]), str(target["path"]), {**dict(target.get("fixed_inputs") or {}), **effective}, target["locations"]))
        except Exception as exc:
            self.store.finish(command_id, status="ambiguous", payload={"error": f"provider_exception:{type(exc).__name__}", "detail": str(exc)[:500], "outcome_unknown": True})
            return
        finally:
            _DISPATCH_CONTEXT.reset(token)
        ok = 200 <= int(status_code) < 300 and not (isinstance(response, Mapping) and response.get("ok") is False)
        event128 = _addressed_event_proven(response)
        position_readback = _position_readback_proven(response)
        home_proof = _home_completion_proven(response)
        controller_ack = _controller_acknowledged(response)
        payload = {"http_status": int(status_code), "response": _bounded_json(response, 131072), "event_128": event128, "position_readback": position_readback, "home_completion_proven": home_proof, "dispatch_attempt_id": claimed["dispatch_attempt_id"], "controller_event_sequence": time.monotonic_ns()}
        target_move = action_id in {"oem.x.move_steps", "oem.x.move_absolute", "oem.y.move_steps", "oem.y.move_absolute", "oem.z.move_steps", "oem.z.move_absolute", "oem.z.clear"}
        home_action = action_id in {"oem.x.manual_panel_home", "oem.y.manual_panel_home", "oem.z.manual_home"}
        completion_proven = ((event128 and position_readback) if target_move else home_proof if home_action else True)
        if action_id == "oem.y.move_absolute" and ok and isinstance(response, Mapping) and response.get("state") == "issued_pending":
            self.store.mark_dispatched(command_id, payload={**payload, "pending": True})
            return
        if ok and completion_proven:
            self.store.finish(command_id, status="completed", payload=payload, remote_acknowledged=True, controller_acknowledged=controller_ack)
        elif ok:
            self.store.finish(command_id, status="failed", payload={**payload, "error": "missing_addressed_event_128" if not event128 else "missing_terminal_position_readback" if target_move else "missing_oem_home_completion" if home_action else "completion_unproven"}, remote_acknowledged=True, controller_acknowledged=controller_ack)
        else:
            self.store.finish(command_id, status="failed", payload=payload, remote_acknowledged=False, controller_acknowledged=controller_ack)

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
