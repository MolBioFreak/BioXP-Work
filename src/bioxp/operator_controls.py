"""Robot-owned catalog, exact-route dispatcher, and bounded durable action receipts.

The browser/BMS submits only an action_id and typed inputs.  The catalog is built
from the final FastAPI route table once at install time, so arbitrary paths or
methods cannot cross the control boundary.  One primitive invocation dispatches
exactly one existing ASGI route.  Meta actions are explicit robot-owned plans and
remain unavailable until their complete provider sequence is bound.
"""
from __future__ import annotations

import asyncio
import hashlib
import json
import math
import os
import re
import time
import uuid
from contextvars import ContextVar
from contextlib import nullcontext
from typing import Any, Callable, Mapping, Sequence
from urllib.parse import urlencode

from fastapi import APIRouter, FastAPI, HTTPException, Query
from pydantic import BaseModel, ConfigDict, Field, StrictBool, StrictInt, field_validator
from starlette.types import Message, Scope

from .hardware_status import hardware_state
from .lifecycle_state import lifecycle_state
from .oem_full_lifecycle import (
    OemFullLifecycleError,
    current_authority_identity,
    current_registry_sha256,
)
from .oem_machine_bundle import OEM_MACHINE_SERIAL
from .operator_receipt_store import OperatorHistoryReader, OperatorReceiptStore
from .command_exchange_observer import exchange_scope
from .release_identity import current_release_identity
from .oem_serial206_initialization_contract import OEM_INITIALIZE_MOTORS_STAGE_KEYS
from .oem_serial206_initialization import SERIAL206_INITIALIZE_MOTION_STAGE_SPECS

CATALOG_SCHEMA = "bioxp.operator_control_catalog.v1"
RECEIPT_SCHEMA = "bioxp.operator_action_receipt.v1"
HISTORY_SCHEMA = "bioxp.operator_action_history.v1"
INTERRUPT_ACTIONS = frozenset({
    "oem.x.stop", "oem.y.stop", "oem.z.stop", "oem.abort_all",
})
_CANONICAL_META_CATEGORIES = frozenset({"activation", "recovery"})


def _is_v2_canonical_action(action: Mapping[str, Any]) -> bool:
    action_id = str(action.get("action_id") or "")
    if action_id.startswith("oem."):
        informational_path = str(action.get("informational_path") or "")
        return (
            action_id not in _PRIVATE_METHOD_ACTION_IDS
            and "/internal/" not in informational_path
        )
    return (
        str(action.get("kind") or "") == "meta"
        and str(action.get("category") or "") in _CANONICAL_META_CATEGORIES
    )


def _v2_canonical_action_ids(
    actions: Sequence[Mapping[str, Any]],
    dispatch: Mapping[str, Mapping[str, Any]],
) -> frozenset[str]:
    by_route: dict[str, list[str]] = {}
    for action in actions:
        if not _is_v2_canonical_action(action):
            continue
        action_id = str(action.get("action_id") or "")
        route = str(action.get("informational_path") or f"@{action_id}")
        by_route.setdefault(route, []).append(action_id)

    selected: set[str] = set()
    for route, action_ids in by_route.items():
        generic_ids = [
            action_id
            for action_id in action_ids
            if not dict(dispatch.get(action_id, {}).get("fixed_inputs") or {})
        ]
        if len(generic_ids) > 1:
            raise RuntimeError(f"duplicate generic V2 action authority for route {route}")
        selected.update(generic_ids or action_ids)
    return frozenset(selected)


_DISPATCH_CONTEXT: ContextVar[dict[str, Any] | None] = ContextVar(
    "bioxp_operator_dispatch_context", default=None
)


def current_operator_dispatch_context() -> dict[str, Any] | None:
    value = _DISPATCH_CONTEXT.get()
    return dict(value) if isinstance(value, Mapping) else None


_MAX_INPUT_BYTES = 65_536
_MAX_RESPONSE_BYTES = 131_072
_MAX_INTERNAL_RESPONSE_BYTES = 8_388_608
_ACTION_RE = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,127}$")
_IDEMPOTENCY_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{7,127}$")
_LINKED_FINALIZATION_KEY = "_bioxp_linked_pipette_finalization"


class InvokeRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    expected_generation: StrictInt = Field(ge=0)
    idempotency_key: str = Field(min_length=8, max_length=128)
    inputs: dict[str, Any] = Field(default_factory=dict)


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


class AdmissionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    expected_generation: StrictInt = Field(ge=0)
    inputs: dict[str, Any] = Field(default_factory=dict)


class AssessmentRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    expected_generation: StrictInt = Field(ge=0)
    idempotency_key: str = Field(min_length=8, max_length=128)
    verdict: str
    note: str = Field(min_length=1, max_length=2000)
    legal_hold: StrictBool | None = None
    actor: str | None = Field(default=None, min_length=1, max_length=128)


# Keep the old import surface while replacing its full-document JSON behavior.
BoundedReceiptStore = OperatorReceiptStore


def _controller_acknowledged(value: Any) -> bool:
    """Detect a successful controller/TMCL ACK independently of HTTP status."""
    if isinstance(value, Mapping):
        for key in ("controller_command_acknowledged", "controller_acknowledged"):
            explicit = value.get(key)
            if type(explicit) is bool:
                return explicit
        status = value.get("status")
        if status == 100 and any(key in value for key in ("status", "raw", "command", "cmd")):
            return True
        ack = value.get("ack")
        if isinstance(ack, Mapping) and ack.get("status") == 100:
            return True
        return any(_controller_acknowledged(item) for item in value.values())
    if isinstance(value, list):
        return any(_controller_acknowledged(item) for item in value)
    return False


def _bounded_json(value: Any, limit: int) -> Any:
    try:
        raw = json.dumps(value, default=str, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    except Exception:
        return {"bounded": True, "detail": "unserializable response"}
    if len(raw) <= limit:
        return json.loads(raw)
    digest = hashlib.sha256(raw).hexdigest()
    return {"bounded": True, "original_bytes": len(raw), "sha256": digest, "preview": raw[: min(limit // 2, 4096)].decode("utf-8", "replace")}


def _resolve_schema(schema: Mapping[str, Any], document: Mapping[str, Any]) -> dict[str, Any]:
    selected = dict(schema)
    ref = selected.get("$ref")
    if isinstance(ref, str) and ref.startswith("#/components/schemas/"):
        name = ref.rsplit("/", 1)[-1]
        target = document.get("components", {}).get("schemas", {}).get(name, {})
        return _resolve_schema(target, document) if isinstance(target, Mapping) else {}
    for union_key in ("anyOf", "oneOf"):
        variants = selected.get(union_key)
        if isinstance(variants, list):
            non_null = [item for item in variants if isinstance(item, Mapping) and item.get("type") != "null"]
            if len(non_null) == 1:
                return _resolve_schema(non_null[0], document)
    return selected


def _input_spec(name: str, schema: Mapping[str, Any], *, required: bool, location: str, description: str = "") -> dict[str, Any]:
    enum_values = schema.get("enum") if isinstance(schema.get("enum"), list) else []
    raw_type = schema.get("type")
    value_type = "enum" if enum_values else raw_type if raw_type in {"string", "integer", "number", "boolean"} else "json"
    default = schema.get("default")
    return {
        "name": re.sub(r"[^a-z0-9_]", "_", name.lower()),
        "wire_name": name,
        "label": schema.get("title") or name.replace("_", " ").title(),
        "value_type": value_type,
        "location": location,
        "required": bool(required),
        "description": str(description or schema.get("description") or "")[:1000],
        "unit": None,
        "enum_values": [str(item) for item in enum_values[:100]],
        "minimum": schema.get("minimum") if isinstance(schema.get("minimum"), (int, float)) else None,
        "maximum": schema.get("maximum") if isinstance(schema.get("maximum"), (int, float)) else None,
        "exclusive_minimum": schema.get("exclusiveMinimum") if isinstance(schema.get("exclusiveMinimum"), (int, float)) else None,
        "exclusive_maximum": schema.get("exclusiveMaximum") if isinstance(schema.get("exclusiveMaximum"), (int, float)) else None,
        "default": _bounded_json(default, 4096) if default is not None else None,
    }


_IMPLICIT_OPERATOR_ACK_BY_PATH = {
    "/diagnostics/usb-sniff/start": "USB_SNIFF",
    "/diagnostics/usb-sniff/stop": "USB_SNIFF",
    "/maintenance/usb/recover_motion": "RECOVER",
    "/motion/oem/x/set_home": "SET_HOME_CURRENT_POSITION",
    "/motion/arm/strict_startup": "RECOVER_MOTION",
    "/motion/diagnostics/execute": "RUN_AXIS_DIAGNOSTIC",
    "/motion/diagnostics/stop": "STOP_AXIS",
    "/motion/gripper/clear": "GRIPPER_CLEAR",
    "/motion/gripper/home": "GRIPPER_HOME",
    "/motion/interlock/override": "INTERLOCK_OVERRIDE",
    "/motion/oem/home_xy": "HOMEXY",
    "/motion/oem/move_xy": "MOVEXY",
    "/motion/oem/initialization/initialize_motors": "INITIALIZE_MOTORS_STAGE",
    "/motion/oem/initialization/initialize_motion": "INITIALIZE_MOTION_STAGE",
    "/motion/thermal_door/home": "HOME_THERMAL_DOOR",
    "/motion/thermal_door/open": "OPEN_THERMAL_DOOR",
    "/motion/thermal_door/close": "CLOSE_THERMAL_DOOR",
    "/oem/initial_check": "INITIALIZE",
    "/oem/startup/initialize_environment": "INITIALIZE",
    "/oem/startup/request": "INITIALIZE",
    "/oem/runtime/movement-runs": "OEM_PATH_EXECUTE",
    "/motion/oem/pathing/scriptmove_execute": "OEM_PATH_EXECUTE",
}


def _implicit_operator_ack(path: str, inputs: Mapping[str, Any]) -> Any:
    if path == "/motion/axes/current":
        return True
    if path.startswith("/oem/runtime/commands/"):
        return "INITIALIZE" if inputs.get("mode") == "live" else None
    return _IMPLICIT_OPERATOR_ACK_BY_PATH.get(path)


_LATCH_CAPABLE_INITIALIZATION_PATHS = {
    "/oem/startup/initialize_environment",
    "/oem/initial_check",
}

_NO_MOTION_PREPARATION_PATHS = {
    "/motion/oem/prepare_without_motion",
    "/motion/arm/strict_startup",
    # ClassMotor.setHome is a controller-coordinate write (SAP1=0), not a
    # movement/homing action.  It must remain available to repair a stale Z
    # coordinate while the physical-motion arm is deliberately disarmed.
    "/motion/oem/z/set_home",

    "/motion/oem/x/set_home",
    "/motion/oem/x/set_max_speed",
    "/motion/oem/x/set_max_acc",
    "/motion/oem/x/restore_original_speed",
    "/motion/oem/x/set_stall_guard",
}


def _safety(method: str, path: str) -> str:
    lower = path.lower()
    if "emergency" in lower or "e_stop" in lower or "estop" in lower:
        return "emergency"
    if any(token in lower for token in ("/stop", "/abort", "/cancel")) or lower == "/oem/runtime/events/pause":
        return "stop"
    if lower in _NO_MOTION_PREPARATION_PATHS:
        return "service"
    if "constructor_pipettes" in lower:
        return "service" if method != "GET" else "read_only"
    if "/liquid/" in lower:
        return "motion" if method != "GET" else "read_only"
    if lower == "/protocol/execute" or lower.startswith("/oem/runtime/commands/") or lower == "/oem/runtime/events/resume":
        return "motion" if method != "GET" else "read_only"
    if lower in _LATCH_CAPABLE_INITIALIZATION_PATHS:
        return "motion" if method != "GET" else "read_only"
    if any(token in lower for token in ("motion", "move", "home", "axis", "gantry", "gripper", "pipette", "door", "latch")):
        return "motion" if method != "GET" else "read_only"
    return "read_only" if method == "GET" else "service"


def _value(row: Any, *keys: str) -> Any:
    current = row
    for key in keys:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    if isinstance(current, Mapping):
        return current.get("value")
    return current


def _motor_motion_action(action: Mapping[str, Any]) -> bool:
    if str(action.get("informational_method")) == "GET":
        return False
    if str(action.get("safety_class")) in {"stop", "emergency"}:
        return False
    path = str(action.get("informational_path") or "").lower()
    if path in _NO_MOTION_PREPARATION_PATHS:
        return False
    if any(token in path for token in ("/stop", "/abort", "/cancel", "emergency")):
        return False
    if "constructor_pipettes" in path:
        return False
    if path in _LATCH_CAPABLE_INITIALIZATION_PATHS:
        return True
    return any(token in path for token in (
        "/motion", "/axis", "/motor", "/gantry", "/gripper", "/latch/", "/thermal-door",
        "/thermal_door", "/pipette", "/liquid/", "/aspirate", "/dispense", "/mix",
    )) or path == "/protocol/execute" or path.startswith("/oem/runtime/commands/") or path == "/oem/runtime/events/resume"


def _home_action(action: Mapping[str, Any]) -> bool:
    path = str(action.get("informational_path") or "").lower()
    if path.endswith("/move_z_home"):
        return True
    return "home" in path and not any(token in path for token in ("move", "park", "position"))


_Z_NO_MOTION_STATE_ACTIONS = frozenset({
    "oem.z.set_home",
})

_Y_NO_MOTION_STATE_ACTIONS = frozenset()

_X_NO_MOTION_STATE_ACTIONS = frozenset({
    "oem.x.set_home",
    "oem.x.set_max_speed",
    "oem.x.set_max_acc",
    "oem.x.restore_original_speed",
    "oem.x.set_stall_guard",
    "oem.xy.enable",
    "oem.xyz.enable",
})

_Z_AUTO_PREREQUISITE_ACTIONS = frozenset({
    "oem.z.manual_home",
    "oem.z.diagnostic_home_axis",
    "oem.z.move_steps",
    "oem.z.move_absolute",
    "oem.z.clear",
})


def _z_no_motion_state_action(action: Mapping[str, Any]) -> bool:
    return str(action.get("action_id") or "") in _Z_NO_MOTION_STATE_ACTIONS


def _required_reference_axes(action: Mapping[str, Any], inputs: Mapping[str, Any]) -> list[str]:
    if not _motor_motion_action(action) or _home_action(action):
        return []
    if str(action.get("action_id") or "") in {"oem.z.scriptmove_to", "oem.xyz.move_to"}:
        return ["x", "y", "z"]
    path = str(action.get("informational_path") or "").lower()
    if any(token in path for token in ("pipette", "aspirate", "dispense", "mix")):
        return ["x", "y", "z"]
    candidates: list[Any] = [inputs.get("axis")]
    body = inputs.get("body")
    if isinstance(body, Mapping):
        candidates.append(body.get("axis"))
    for axis in ("x", "y", "z", "g", "door"):
        if re.search(rf"(?:^|[/_.-]){re.escape(axis)}(?:$|[/_.-])", path):
            candidates.append(axis)
    normalized = []
    aliases = {"gripper": "g", "thermal_door": "door", "thermal-door": "door"}
    for candidate in candidates:
        value = aliases.get(str(candidate).lower(), str(candidate).lower()) if candidate is not None else ""
        if value in {"x", "y", "z", "g", "door"} and value not in normalized:
            normalized.append(value)
    return normalized


def _dependency(key: str, label: str, met: bool, reason: str | None = None) -> dict[str, Any]:
    return {"key": key, "label": label, "met": bool(met), "reason": None if met else reason}


_TRANSPORT_BOOTSTRAP_PATHS = {
    "/reconnect",
    # This source-grounded, no-motion route establishes board/profile readiness;
    # blocking it on stale published transport state creates an admission deadlock.
    "/motion/oem/prepare_without_motion",
}

_LOCAL_ONLY_PATH_PREFIXES = (
    "/maintenance/usb/",
)

_OPERATOR_SEMANTIC_QUARANTINE_PATHS = {
    "/motion/interlock/prepare": "Quarantined: this legacy route performs inferred latch/power writes and is not the source-grounded serial-206 preparation provider.",
    "/motion/power/diag": "Quarantined: this diagnostic can enter the same unverified power-enable sequence and lacks truthful aggregate acknowledgment/readback.",
}

_CAN_BOOTSTRAP_PATHS = {
    "/hardware/snapshot/collect",
    "/motion/oem/prepare_without_motion",
}


def _operation_motion_dependency(machine_state: Mapping[str, Any]) -> dict[str, Any]:
    lifecycle_value = machine_state.get("lifecycle")
    lifecycle: Mapping[str, Any] = lifecycle_value if isinstance(lifecycle_value, Mapping) else {}
    operation_state = lifecycle.get("operation_state")
    return _dependency(
        "operation_allows_motion",
        "Operation state allows motion",
        operation_state != "emergency",
        "Motion is blocked while operation state is emergency.",
    )


def _motion_readiness(machine_state: Mapping[str, Any], required_axes: list[str]) -> dict[str, Any]:
    """One fail-closed predicate shared by motion admission and dashboard truth."""
    dependencies: list[dict[str, Any]] = [_operation_motion_dependency(machine_state)]
    ownership_value = machine_state.get("ownership")
    ownership: Mapping[str, Any] = ownership_value if isinstance(ownership_value, Mapping) else {}
    transport_live = bool(
        ownership.get("transport") == "owned"
        and ownership.get("usb") == "service"
        and ownership.get("router") == "running"
    )
    dependencies.append(_dependency("transport_live", "Robot transport live", transport_live, "Robot transport is unavailable."))
    dependencies.append(_dependency(
        "can_ready", "Same-epoch CAN ready", ownership.get("CAN_READY") is True,
        "Same-epoch CAN readiness has not been established.",
    ))
    snapshot_present = isinstance(machine_state.get("snapshot_id"), str) and bool(machine_state.get("snapshot_id"))
    dependencies.append(_dependency(
        "canonical_snapshot", "Canonical hardware snapshot", snapshot_present,
        "Fresh canonical hardware snapshot is unavailable.",
    ))
    freshness = machine_state.get("freshness") if isinstance(machine_state.get("freshness"), Mapping) else {}
    freshness_reason = "Canonical hardware snapshot is stale." if freshness.get("state") == "stale" else "Fresh canonical hardware snapshot is unavailable."
    dependencies.append(_dependency(
        "snapshot_fresh", "Canonical snapshot fresh", freshness.get("state") == "fresh", freshness_reason,
    ))
    maintenance_value = machine_state.get("maintenance")
    maintenance: Mapping[str, Any] = maintenance_value if isinstance(maintenance_value, Mapping) else {}
    motion_enabled = maintenance.get("motion_blocked") is False and maintenance.get("recovery_required") is False
    dependencies.append(_dependency(
        "motion_enabled", "Motion enabled", motion_enabled,
        "Motion is inactive. Activate motion before moving this motor.",
    ))
    domains = machine_state.get("domains") if isinstance(machine_state.get("domains"), Mapping) else {}
    power_row = domains.get("power") if isinstance(domains.get("power"), Mapping) else {}
    power = power_row.get("observation") if isinstance(power_row, Mapping) else None
    dependencies.append(_dependency(
        "power_ready", "24 V rail sensor valid",
        isinstance(power, Mapping) and power.get("safety_valid") is True,
        "24 V rail sensor is not confirmed ready.",
    ))
    lifecycle = machine_state.get("lifecycle") if isinstance(machine_state.get("lifecycle"), Mapping) else {}
    door = lifecycle.get("door") if isinstance(lifecycle.get("door"), Mapping) else {}
    latch_row = domains.get("latch") if isinstance(domains.get("latch"), Mapping) else {}
    latch = latch_row.get("observation") if isinstance(latch_row, Mapping) else None
    enclosure_ok = bool(
        isinstance(latch, Mapping)
        and type(latch.get("door_sensor")) is int
        and latch.get("door_sensor") == 1
        and type(latch.get("latch_sensor")) is int
        and latch.get("latch_sensor") == 1
    )
    dependencies.append(_dependency(
        "enclosure_ready", "Door closed and latched", enclosure_ok,
        "Robot door is not confirmed closed and latched.",
    ))
    interlock_row = domains.get("interlock") if isinstance(domains.get("interlock"), Mapping) else {}
    interlock = interlock_row.get("observation") if isinstance(interlock_row, Mapping) else None
    motion_arm = interlock.get("motion_arm") if isinstance(interlock, Mapping) else None
    dependencies.append(_dependency(
        "motion_arm", "Motion arm confirmed",
        isinstance(motion_arm, Mapping) and motion_arm.get("armed") is True,
        "Motion arm is not confirmed.",
    ))
    references = machine_state.get("references") if isinstance(machine_state.get("references"), Mapping) else {}
    rows = references.get("rows") if isinstance(references.get("rows"), Mapping) else {}
    for axis in required_axes:
        row = rows.get(axis) if isinstance(rows, Mapping) else None
        referenced = isinstance(row, Mapping) and row.get("state") == "referenced"
        dependencies.append(_dependency(
            f"axis_{axis}_referenced", f"{axis.upper()} axis homed", referenced,
            f"{axis.upper()} axis is not homed.",
        ))
    failed = next((row for row in dependencies if not row["met"]), None)
    return {
        "enabled": failed is None,
        "disabled_reason": None if failed is None else failed["reason"],
        "dependencies": dependencies,
    }


def _provider_z_motion_readiness(machine_state: Mapping[str, Any]) -> dict[str, Any]:
    """Readiness owned by the stable Serial-206 Z lifecycle.

    The provider performs live controller/interlock checks inside each command.
    Its motion admission must not depend on the short-lived global analytics
    snapshot, which is collected through the same serialized USB transport.
    """
    ownership = machine_state.get("ownership") if isinstance(machine_state.get("ownership"), Mapping) else {}
    maintenance = machine_state.get("maintenance") if isinstance(machine_state.get("maintenance"), Mapping) else {}
    provider = (
        machine_state.get("serial206_initialization_provider")
        if isinstance(machine_state.get("serial206_initialization_provider"), Mapping)
        else {}
    )
    z_authority = provider.get("z_authority") if isinstance(provider.get("z_authority"), Mapping) else {}
    dependencies = [
        _operation_motion_dependency(machine_state),
        _dependency(
            "can_ready",
            "Same-epoch CAN ready",
            ownership.get("CAN_READY") is True,
            "Same-epoch CAN readiness has not been established.",
        ),
        _dependency(
            "motion_enabled",
            "Motion enabled",
            maintenance.get("motion_blocked") is False and maintenance.get("recovery_required") is False,
            "Motion is inactive. Activate motion before moving this motor.",
        ),
        _dependency(
            "z_board_lifecycle_fresh",
            "Serial-206 Z board lifecycle current",
            z_authority.get("board_lifecycle_generation_fresh") is True,
            "Z board lifecycle changed; activate motion again.",
        ),
    ]
    failed = next((row for row in dependencies if not row["met"]), None)
    return {
        "enabled": failed is None,
        "disabled_reason": None if failed is None else failed["reason"],
        "dependencies": dependencies,
    }


def _provider_x_motion_readiness(machine_state: Mapping[str, Any]) -> dict[str, Any]:
    ownership_value = machine_state.get("ownership")
    ownership: Mapping[str, Any] = ownership_value if isinstance(ownership_value, Mapping) else {}
    maintenance_value = machine_state.get("maintenance")
    maintenance: Mapping[str, Any] = maintenance_value if isinstance(maintenance_value, Mapping) else {}
    provider_value = machine_state.get("serial206_initialization_provider")
    provider: Mapping[str, Any] = provider_value if isinstance(provider_value, Mapping) else {}
    x_authority_value = provider.get("x_authority")
    x_authority: Mapping[str, Any] = x_authority_value if isinstance(x_authority_value, Mapping) else {}
    lifecycle_value = x_authority.get("lifecycle")
    lifecycle: Mapping[str, Any] = lifecycle_value if isinstance(lifecycle_value, Mapping) else x_authority
    board_fresh = lifecycle.get("board_lifecycle_generation_fresh")
    dependencies = [
        _operation_motion_dependency(machine_state),
        _dependency("can_ready", "Same-epoch CAN ready", ownership.get("CAN_READY") is True, "Same-epoch CAN readiness has not been established."),
        _dependency("motion_enabled", "Motion enabled", maintenance.get("motion_blocked") is False and maintenance.get("recovery_required") is False, "Motion is inactive. Activate motion before moving this motor."),
        _dependency("x_board_lifecycle_fresh", "Serial-206 X board lifecycle current", board_fresh is True, "X board lifecycle is unavailable or changed; prepare X again."),
    ]
    failed = next((row for row in dependencies if not row["met"]), None)
    return {"enabled": failed is None, "disabled_reason": None if failed is None else failed["reason"], "dependencies": dependencies}


def _assess_action(action: Mapping[str, Any], machine_state: Mapping[str, Any], inputs: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Assess one action using only already-published machine state."""
    values = dict(inputs or {})
    if str(action.get("action_id") or "").startswith("oem.z."):
        values.setdefault("axis", "z")
    if str(action.get("action_id") or "").startswith("oem.x."):
        values.setdefault("axis", "x")
    dependencies: list[dict[str, Any]] = []
    provider_available = bool(action.get("provider_available", action.get("available", True)))
    provider_reason = str(action.get("provider_unavailable_reason") or action.get("unavailable_reason") or "Robot provider is not available.")
    required_provider_capability = action.get("required_provider_capability")
    if isinstance(required_provider_capability, str) and required_provider_capability:
        provider_state_value = machine_state.get("serial206_initialization_provider")
        provider_state = provider_state_value if isinstance(provider_state_value, Mapping) else {}
        capability_field = f"{required_provider_capability}_live_available"
        capability_available = provider_state.get("bound") is True and provider_state.get(capability_field) is True
        provider_available = provider_available and capability_available
        if not capability_available:
            provider_reason = f"Serial-206 live provider capability unavailable: {required_provider_capability}."
    dependencies.append(_dependency("provider_available", "Provider available", provider_available, provider_reason))

    action_id = str(action.get("action_id") or "")
    x_provider_action = action_id.startswith("oem.x.") or action_id.startswith("oem.xy.") or action_id.startswith("oem.xyz.") or action_id == "oem.abort_all"
    y_provider_action = action_id.startswith("oem.y.") or action_id.startswith("oem.xy.") or action_id.startswith("oem.xyz.") or action_id == "oem.abort_all"
    method = str(action.get("informational_method") or "GET")
    safety = str(action.get("safety_class") or "read_only")
    path = str(action.get("informational_path") or "").lower()
    source_initializer = path in {
        "/motion/oem/initialization/initialize_motors",
        "/motion/oem/initialization/initialize_motion",
    }
    requires_transport = (method != "GET" or safety in {"stop", "emergency"}) and path not in _TRANSPORT_BOOTSTRAP_PATHS
    ownership_value = machine_state.get("ownership")
    ownership: Mapping[str, Any] = ownership_value if isinstance(ownership_value, Mapping) else {}
    maintenance_value = machine_state.get("maintenance")
    maintenance: Mapping[str, Any] = maintenance_value if isinstance(maintenance_value, Mapping) else {}
    transport_live = bool(
        ownership.get("transport") == "owned"
        and ownership.get("usb") == "service"
        and ownership.get("router") == "running"
    )
    if requires_transport:
        dependencies.append(_dependency("transport_live", "Robot transport live", transport_live, "Robot transport is unavailable."))
    if action_id == "meta.recover_motion_non_homing":
        dependencies.append(_dependency(
            "recovery_required",
            "Non-homing recovery required",
            maintenance.get("recovery_required") is True,
            "Non-homing recovery is not currently required.",
        ))

    if (
        str(values.get("axis") or "").lower() == "z"
        and path in {"/motion/oem/manual/relative", "/motion/oem/manual/absolute", "/motion/oem/manual/home"}
        and not action_id.startswith("oem.z.")
    ):
        dependencies.append(_dependency(
            "stable_z_semantic_action",
            "Stable provider-owned Z semantic action",
            False,
            "Use the corresponding oem.z.* action ID; generic route-derived Z actions are retired.",
        ))

    if (
        str(values.get("axis") or "").lower() == "x"
        and path in {"/motion/oem/manual/relative", "/motion/oem/manual/absolute", "/motion/oem/manual/home"}
        and not action_id.startswith("oem.x.")
    ):
        dependencies.append(_dependency(
            "stable_x_semantic_action",
            "Stable provider-owned X semantic action",
            False,
            "Use the corresponding oem.x.* action ID; generic route-derived X actions are retired.",
        ))

    provider_owned_x_motion = bool(
        x_provider_action
        and action_id not in _X_NO_MOTION_STATE_ACTIONS
        and action_id not in {"oem.x.status", "oem.x.stop", "oem.abort_all", "oem.x.observe"}
    )
    provider_owned_z_motion = bool(
        action_id.startswith("oem.z.")
        and action_id not in _Z_NO_MOTION_STATE_ACTIONS
        and action_id not in {"oem.z.status", "oem.z.stop", "oem.z.observe"}
    )
    provider_owned_y_motion = bool(
        y_provider_action
        and action_id not in _Y_NO_MOTION_STATE_ACTIONS
        and action_id not in {"oem.y.status", "oem.y.stop", "oem.y.observe"}
    )
    motion_action = source_initializer or safety == "motion" or _motor_motion_action(action)
    if motion_action:
        if not (provider_owned_x_motion or provider_owned_y_motion or provider_owned_z_motion):
            dependencies.append(_operation_motion_dependency(machine_state))
        x_state_establishing = action_id in _X_NO_MOTION_STATE_ACTIONS
        z_state_establishing = _z_no_motion_state_action(action) or action_id == "oem.z.resume_after_abort"
        y_state_establishing = action_id in _Y_NO_MOTION_STATE_ACTIONS
        observation_action = action_id in {"oem.x.observe", "oem.z.observe"}
        required_axes = (
            []
            if source_initializer or z_state_establishing or x_state_establishing or y_state_establishing or observation_action
            else _required_reference_axes(action, values)
        )
        if provider_owned_x_motion:
            readiness = _provider_x_motion_readiness(machine_state)
        elif (
            x_state_establishing
            or y_state_establishing
            or z_state_establishing
            or observation_action
            or provider_owned_y_motion
            or provider_owned_z_motion
        ):
            readiness = {"dependencies": []}
        else:
            readiness = _motion_readiness(machine_state, required_axes)
        existing_keys = {str(row.get("key")) for row in dependencies}
        dependencies.extend(
            row for row in readiness["dependencies"] if str(row.get("key")) not in existing_keys
        )
        effective_axis = str(values.get("axis") or "").lower()
        if effective_axis == "z" and not provider_owned_z_motion:
            domains = machine_state.get("domains") if isinstance(machine_state.get("domains"), Mapping) else {}
            axes_domain = domains.get("axes") if isinstance(domains, Mapping) else None
            axes_observation = axes_domain.get("observation") if isinstance(axes_domain, Mapping) else None
            axis_rows = axes_observation.get("rows") if isinstance(axes_observation, Mapping) else None
            z_row = axis_rows.get("z") if isinstance(axis_rows, Mapping) else None
            z_status = z_row.get("status") if isinstance(z_row, Mapping) else None
            z_switches = z_row.get("switch_activity") if isinstance(z_row, Mapping) else None
            z_preset = z_row.get("preset") if isinstance(z_row, Mapping) else None
            left_home_enabled = isinstance(z_switches, Mapping) and z_switches.get("left_disabled") is False
            right_switch_enabled = isinstance(z_switches, Mapping) and z_switches.get("right_disabled") is False
            # OEM no-motion preparation establishes the motor profile without
            # a prior Z GAP12/GAP13 snapshot. Keep this replacement check only
            # on ordinary generic Z operations, never on state-establishing
            # recovery actions.
            if not z_state_establishing:
                dependencies.append(_dependency(
                    "z_switch_masks_machine_bound",
                    "Z machine-bound GAP12/GAP13 state",
                    left_home_enabled and right_switch_enabled,
                    "Expected GAP12/right enabled and GAP13/left enabled; run Z mask reconciliation.",
                ))
            # Preparation and mask reconciliation do not move Z and therefore
            # must not require an already-valid OEM coordinate.  Movement and
            # ordinary non-home Z actions retain the envelope/target gates.
            if not _home_action(action) and not z_state_establishing:
                position = _value(z_status, "position")
                minimum = z_preset.get("axis_min_steps") if isinstance(z_preset, Mapping) else 0
                maximum = z_preset.get("axis_max_steps") if isinstance(z_preset, Mapping) else 160000
                in_envelope = bool(
                    type(position) is int
                    and type(minimum) is int
                    and type(maximum) is int
                    and minimum <= position <= maximum
                )
                dependencies.append(_dependency(
                    "z_position_oem_envelope",
                    "Z position in OEM 0..160000 envelope",
                    in_envelope,
                    "Z controller position is unavailable or outside the OEM nonnegative envelope.",
                ))
                target = None
                if type(position) is int and type(values.get("steps")) is int:
                    target = position + int(values["steps"])
                elif type(values.get("position_steps")) is int:
                    target = int(values["position_steps"])
                if target is not None:
                    target_ok = bool(type(minimum) is int and type(maximum) is int and minimum <= target <= maximum)
                    dependencies.append(_dependency(
                        "z_target_oem_envelope",
                        "Z target in OEM 0..160000 envelope",
                        target_ok,
                        "Requested Z target is outside the OEM nonnegative envelope.",
                    ))

    failed = next((row for row in dependencies if not row["met"]), None)
    return {"enabled": failed is None, "disabled_reason": None if failed is None else failed["reason"], "dependencies": dependencies}


_TEMPERATURE_LABELS = {
    "tc_temp_c": "Thermal cycler block",
    "lid_temp_c": "Thermal cycler lid",
    "ped_temp_c": "Thermal cycler pedestal",
    "rc_temp_c": "Reagent chiller",
    "rc_pedestal_c": "Reagent chiller pedestal",
    "oc_temp_c": "Oligo chiller",
    "oc_pedestal_c": "Oligo chiller pedestal",
}


def _temperature_payload(sensor: Any, value_row: Any) -> dict[str, Any]:
    sensor_id = str(sensor)
    label = _TEMPERATURE_LABELS.get(
        sensor_id,
        re.sub(r"(?:_temp)?_c$", "", sensor_id).replace("_", " ").strip().title() or sensor_id,
    )
    raw: Any = None
    scale = 1.0
    if isinstance(value_row, Mapping):
        if "temp_c" in value_row:
            raw = value_row.get("temp_c")
        else:
            raw = value_row.get("value")
            scale = 0.001
    available = (
        type(raw) in (int, float)
        and math.isfinite(float(raw))
        and isinstance(value_row, Mapping)
        and value_row.get("ok", True) is True
    )
    return {
        "sensor": sensor_id,
        "label": label,
        "unit": "°C",
        "temperature_c": round(float(raw) * scale, 3) if available else None,
        "available": bool(available),
    }


def _dashboard_payload(machine_state: Mapping[str, Any]) -> dict[str, Any]:
    domains = machine_state.get("domains") if isinstance(machine_state.get("domains"), Mapping) else {}
    ownership_value = machine_state.get("ownership")
    ownership: Mapping[str, Any] = ownership_value if isinstance(ownership_value, Mapping) else {}
    maintenance_value = machine_state.get("maintenance")
    maintenance: Mapping[str, Any] = maintenance_value if isinstance(maintenance_value, Mapping) else {}
    lifecycle = machine_state.get("lifecycle") if isinstance(machine_state.get("lifecycle"), Mapping) else {}
    references = machine_state.get("references") if isinstance(machine_state.get("references"), Mapping) else {}
    reference_rows = references.get("rows") if isinstance(references.get("rows"), Mapping) else {}

    axes_observation = ((domains.get("axes") or {}).get("observation") or {}) if isinstance(domains.get("axes"), Mapping) else {}
    axis_rows = axes_observation.get("rows") if isinstance(axes_observation.get("rows"), Mapping) else {}
    axes = []
    for axis, row in axis_rows.items():
        status_value = row.get("status") if isinstance(row, Mapping) else None
        status: Mapping[str, Any] = status_value if isinstance(status_value, Mapping) else {}
        switches_value = row.get("switch_activity") if isinstance(row, Mapping) else None
        switches: Mapping[str, Any] = switches_value if isinstance(switches_value, Mapping) else {}
        axes.append({
            "axis": str(axis),
            "reference": ((reference_rows.get(axis) or {}).get("state") if isinstance(reference_rows.get(axis), Mapping) else "unknown") or "unknown",
            "position_steps": _value(status, "position"),
            "speed_steps_s": _value(status, "speed"),
            "run_current": _value(status, "max_current"),
            "standby_current": _value(status, "standby_current"),
            "left_switch_state": switches.get("left_state"),
            "right_switch_state": switches.get("right_state"),
            "left_switch_raw_active": switches.get("left_raw_active"),
            "right_switch_raw_active": switches.get("right_raw_active"),
            "left_switch_active": switches.get("left_effective_active"),
            "right_switch_active": switches.get("right_effective_active"),
            "left_switch_disabled": switches.get("left_disabled"),
            "right_switch_disabled": switches.get("right_disabled"),
            "coordinate_contract": ((row.get("preset") or {}).get("coordinate_contract") if isinstance(row, Mapping) else None),
            "min_steps": ((row.get("preset") or {}).get("axis_min_steps") if isinstance(row, Mapping) else None),
            "max_steps": ((row.get("preset") or {}).get("axis_max_steps") if isinstance(row, Mapping) else None),
            "motor_temperature_c": None,
            "motor_temperature_available": False,
        })

    temperatures = []
    for domain in ("thermal", "chiller"):
        row = domains.get(domain)
        observation = row.get("observation") if isinstance(row, Mapping) else None
        temps = observation.get("temps") if isinstance(observation, Mapping) else None
        if not isinstance(temps, Mapping):
            continue
        for sensor, value_row in temps.items():
            temperatures.append(_temperature_payload(sensor, value_row))

    pipette_row = domains.get("pipette")
    pipette_observation = pipette_row.get("observation") if isinstance(pipette_row, Mapping) else None
    pipettes = dict(pipette_observation) if isinstance(pipette_observation, Mapping) else {"ok": False, "channels": [], "error": "Pipette status is not reported."}
    enclosure = lifecycle.get("door") if isinstance(lifecycle.get("door"), Mapping) else {}
    latch_row = domains.get("latch") if isinstance(domains.get("latch"), Mapping) else {}
    latch = latch_row.get("observation") if isinstance(latch_row, Mapping) else None
    freshness_value = machine_state.get("freshness")
    freshness: Mapping[str, Any] = (
        freshness_value
        if isinstance(freshness_value, Mapping)
        else {"state": "missing", "age_s": None, "fresh_for_s": None}
    )
    connection_live = bool(ownership.get("transport") == "owned" and ownership.get("usb") == "service" and ownership.get("router") == "running")
    # Report X and Z motion status from their stable provider authorities. The
    # cached analytics row remains available in ``axes`` but cannot replace a
    # bound provider's authority-bearing terminal projection.
    cached_x_axis = next((row for row in axes if row.get("axis") == "x"), None)
    z_axis = next((row for row in axes if row.get("axis") == "z"), None)
    provider_state_value = machine_state.get("serial206_initialization_provider")
    provider_state: Mapping[str, Any] = provider_state_value if isinstance(provider_state_value, Mapping) else {}
    initialize_motors = provider_state.get("initialize_motors")
    x_authority_value = provider_state.get("x_authority") if isinstance(provider_state, Mapping) else None
    x_authority: Mapping[str, Any] = x_authority_value if isinstance(x_authority_value, Mapping) else {}
    x_lifecycle_value = x_authority.get("lifecycle")
    x_lifecycle: Mapping[str, Any] = x_lifecycle_value if isinstance(x_lifecycle_value, Mapping) else {}
    x_live_value = x_authority.get("live_status")
    x_live_status: Mapping[str, Any] = x_live_value if isinstance(x_live_value, Mapping) else {}
    provider_x_available = bool(
        provider_state.get("bound") is True
        and x_authority.get("authority") == "Serial206OemInitializationProvider"
    )
    x_axis = (
        {
            "axis": "x",
            "reference": x_lifecycle.get("reference_state") or "unknown",
            "position_steps": x_live_status.get("position_steps"),
            "speed_steps_s": x_live_status.get("speed_steps_s"),
            "run_current": x_live_status.get("max_current"),
            "standby_current": None,
            "left_switch_state": x_live_status.get("left_switch_state"),
            "right_switch_state": x_live_status.get("right_switch_state"),
            "left_switch_raw_active": None,
            "right_switch_raw_active": None,
            "left_switch_active": None,
            "right_switch_active": None,
            "left_switch_disabled": x_live_status.get("left_switch_disabled"),
            "right_switch_disabled": x_live_status.get("right_switch_disabled"),
            "coordinate_contract": "serial206_x_machine_config_max_effective_min_60_relative_margin_20",
            "min_steps": x_authority.get("source_min_steps"),
            "max_steps": x_authority.get("source_max_steps"),
            "motor_temperature_c": None,
            "motor_temperature_available": False,
            "telemetry_authority": x_live_status.get("authority"),
            "physical_position_verified": False,
        }
        if provider_x_available
        else cached_x_axis
    )
    x_provider = {
        **x_authority,
        "bound": provider_state.get("bound") is True,
        "physical_position_verified": False,
    }
    z_authority_value = provider_state.get("z_authority") if isinstance(provider_state, Mapping) else None
    z_authority: Mapping[str, Any] = z_authority_value if isinstance(z_authority_value, Mapping) else {}
    terminal_state = z_authority.get("terminal_state") if isinstance(z_authority.get("terminal_state"), Mapping) else {}
    if z_axis is None and (
        type(terminal_state.get("position_steps")) is int
        or type(terminal_state.get("speed_steps_s")) is int
    ):
        z_axis = {
            "axis": "z",
            "reference": z_authority.get("reference_state") or "unknown",
            "position_steps": terminal_state.get("position_steps"),
            "speed_steps_s": terminal_state.get("speed_steps_s"),
            "run_current": None,
            "standby_current": None,
            "left_switch_state": terminal_state.get("left_switch_state"),
            "right_switch_state": terminal_state.get("right_switch_state"),
            "left_switch_raw_active": None,
            "right_switch_raw_active": None,
            "left_switch_active": None,
            "right_switch_active": None,
            "left_switch_disabled": terminal_state.get("left_switch_disabled"),
            "right_switch_disabled": terminal_state.get("right_switch_disabled"),
            "coordinate_contract": z_authority.get("coordinate_contract"),
            "min_steps": z_authority.get("source_min_steps"),
            "max_steps": z_authority.get("source_max_steps"),
            "motor_temperature_c": None,
            "motor_temperature_available": False,
            "telemetry_authority": terminal_state.get("authority"),
        }
        axes.append(z_axis)
    provider_z_available = bool(
        provider_state.get("bound") is True
        and z_authority.get("state") not in {None, "unbound", "corrupt"}
    )
    motion_readiness = (
        _provider_z_motion_readiness(machine_state)
        if provider_z_available
        else _motion_readiness(machine_state, [])
    )
    z_terminal_value = z_authority.get("terminal_state")
    z_terminal_state: Mapping[str, Any] = z_terminal_value if isinstance(z_terminal_value, Mapping) else {}
    z_provider = {
        **z_authority,
        "bound": provider_state.get("bound") if isinstance(provider_state, Mapping) else False,
        "expected_startup_stage": initialize_motors.get("expected_next_stage") if isinstance(initialize_motors, Mapping) else None,
        "startup_terminal_state": initialize_motors.get("terminal_state") if isinstance(initialize_motors, Mapping) else None,
        "switch_mask_policy": "observed_only_oem_source_omits_z_writes",
        "switch_mask_tuple": z_terminal_state.get("switch_mask_tuple"),
    }
    return {
        "schema_version": "bioxp.operator_dashboard.v1",
        "ownership_generation": int(machine_state.get("ownership_generation") or 0),
        "connection": {"live": connection_live, "ownership": dict(ownership)},
        "motion": {"enabled": motion_readiness["enabled"], "reason": motion_readiness["disabled_reason"]},
        "operation": {"state": lifecycle.get("operation_state"), "reason": lifecycle.get("operation_reason")},
        "enclosure": {
            "door_closed": (latch.get("door_sensor") == 1) if isinstance(latch, Mapping) else None,
            "latch_closed": (latch.get("latch_sensor") == 1) if isinstance(latch, Mapping) else None,
        },
        "axes": axes,
        "x_axis": {
            "status": x_axis,
            "provider": x_provider,
            "snapshot_freshness": dict(freshness),
            "last_failure": x_lifecycle.get("last_failure"),
            "latest_receipt": x_lifecycle.get("latest_receipt"),
            "authority": "Serial206OemInitializationProvider" if provider_x_available else "unbound",
            "physical_position_verified": False,
        },
        "z_axis": {
            "status": z_axis,
            "provider": z_provider,
            "snapshot_freshness": dict(freshness),
            "last_failure": maintenance.get("z_home_failure") if isinstance(maintenance, Mapping) else None,
            "authority": "Serial206OemInitializationProvider",
        },
        "temperatures": temperatures,
        "pipettes": pipettes,
        "snapshot": {"snapshot_id": machine_state.get("snapshot_id"), "freshness": dict(freshness), "collection_triggered": False},
    }


def _subsystem(path: str) -> str:
    pieces = [part for part in path.split("/") if part]
    if not pieces:
        return "system"
    return ".".join(pieces[:2]) if pieces[0] in {"motion", "oem", "maintenance"} and len(pieces) > 1 else pieces[0]


def _path_action_id(method: str, path: str, operation_id: str | None) -> str:
    base = operation_id or f"{method}_{path}"
    slug = re.sub(r"[^a-z0-9_.-]+", "_", base.lower()).strip("_.-")
    digest = hashlib.sha256(f"{method} {path}".encode()).hexdigest()[:8]
    return f"route.{slug[:96]}.{digest}"


def _extract_inputs(operation: Mapping[str, Any], document: Mapping[str, Any]) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    specs: list[dict[str, Any]] = []
    locations: dict[str, dict[str, Any]] = {}
    for parameter in operation.get("parameters", []):
        if not isinstance(parameter, Mapping) or parameter.get("in") not in {"path", "query"}:
            continue
        name = str(parameter.get("name") or "")
        if not name:
            continue
        location = str(parameter["in"])
        schema = _resolve_schema(parameter.get("schema", {}), document)
        spec = _input_spec(name, schema, required=bool(parameter.get("required")), location=location, description=str(parameter.get("description") or ""))
        specs.append(spec)
        locations[spec["name"]] = {"location": location, "wire_name": name}
    request_body = operation.get("requestBody")
    if isinstance(request_body, Mapping):
        body_schema = request_body.get("content", {}).get("application/json", {}).get("schema", {})
        body_schema = _resolve_schema(body_schema, document) if isinstance(body_schema, Mapping) else {}
        properties = body_schema.get("properties") if isinstance(body_schema.get("properties"), Mapping) else None
        required_names = set(body_schema.get("required", [])) if isinstance(body_schema.get("required"), list) else set()
        if properties:
            for raw_name, raw_schema in properties.items():
                schema = _resolve_schema(raw_schema, document) if isinstance(raw_schema, Mapping) else {}
                name = re.sub(r"[^a-z0-9_]", "_", str(raw_name).lower())
                if raw_name in {"operator_ack", "operator_reason"}:
                    locations[name] = {
                        "location": "body",
                        "wire_name": str(raw_name),
                        "implicit_operator_control": True,
                    }
                    continue
                spec = _input_spec(name, schema, required=raw_name in required_names, location="body")
                spec["wire_name"] = str(raw_name)
                specs.append(spec)
                locations[name] = {"location": "body", "wire_name": str(raw_name)}
        else:
            specs.append(_input_spec("body", body_schema, required=bool(request_body.get("required")), location="body", description=str(request_body.get("description") or "Request JSON body")))
            locations["body"] = {"location": "body", "wire_name": "body"}
    return specs, locations


_NON_OPERATOR_COMPAT_PATHS = {
    "/oem/startup/status/{session_id}",
    # Retired generic primitives must not compete with dedicated serial-206 OEM controls.
    "/motion/axis/relative",
    "/motion/axis/absolute",
    "/motion/axis/home",
    "/motion/axis/zero",
    "/motion/oem/z/live_right_reference",
    "/motion/oem/z/abort",
}
_SERIAL206_PROVIDER_CAPABILITIES = {
    "/motion/oem/initialization/initialize_motors": "initialize_motors",
    "/motion/oem/initialization/initialize_motion": "initialize_motion",
    "/motion/oem/manual/relative": "initialize_motors",
    "/motion/oem/manual/absolute": "initialize_motors",
    "/motion/oem/manual/home": "initialize_motors",
    "/motion/oem/x/status": "initialize_motors",
    "/motion/oem/x/prepare": "initialize_motors",

    "/motion/oem/x/move_steps": "initialize_motors",
    "/motion/oem/x/move_absolute": "initialize_motors",
    "/motion/oem/x/manual_home": "initialize_motors",
    "/motion/oem/x/diagnostic_home_axis": "initialize_motors",
    "/motion/oem/x/set_home": "initialize_motors",
    "/motion/oem/x/set_max_speed": "initialize_motors",
    "/motion/oem/x/set_max_acc": "initialize_motors",
    "/motion/oem/x/restore_original_speed": "initialize_motors",
    "/motion/oem/x/set_stall_guard": "initialize_motors",
    "/motion/oem/x/stop": "initialize_motors",
    "/motion/oem/x/abort": "initialize_motors",
    "/motion/oem/x/observation": "initialize_motors",
    "/motion/oem/y/status": "initialize_motors",
    "/motion/oem/y/prepare": "initialize_motors",
    "/motion/oem/y/move_steps": "initialize_motors",
    "/motion/oem/y/move_absolute": "initialize_motors",
    "/motion/oem/y/home": "initialize_motors",
    "/motion/oem/y/set_home": "initialize_motors",
    "/motion/oem/y/stop": "initialize_motors",
    "/motion/oem/z/prepare": "initialize_motors",
    "/motion/oem/z/move_z_home": "initialize_motors",
    "/motion/oem/z/control": "initialize_motors",
    "/motion/oem/z/path_clean_mode": "initialize_motors",
    "/motion/oem/pathing/scriptmove_execute": "initialize_motors",
    "/motion/oem/home_gz": "initialize_motors",
    "/motion/oem/z/move_gz": "initialize_motors",
    "/motion/oem/z/lower_pipette": "initialize_motors",
    "/motion/oem/z/lift_pipette": "initialize_motors",
    "/motion/oem/z/self_test": "initialize_motors",

    "/motion/oem/z/set_home": "initialize_motors",
    "/motion/oem/z/diagnostic_home_axis": "initialize_motors",
    "/motion/oem/z/stop": "initialize_motors",

    "/motion/oem/z/resume_after_abort": "initialize_motors",
    "/motion/oem/z/observation": "initialize_motors",
}


_PRIVATE_METHOD_ACTION_IDS = frozenset({
    "meta.home_xy",
    "oem.xy.home_xy",
    "oem.xy.move_xy",
    "oem.xyz.move_to",
})
_CANONICAL_COMPOSITE_ACTION_IDS = frozenset({
    "oem.xy.home",
    "oem.xy.move_absolute",
})


def _build_catalog(app: FastAPI) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    document = app.openapi()
    actions: list[dict[str, Any]] = []
    dispatch: dict[str, dict[str, Any]] = {}
    excluded = {"/", "/openapi.json", "/docs", "/docs/oauth2-redirect", "/redoc", *_NON_OPERATOR_COMPAT_PATHS}
    for path, path_item in sorted(document.get("paths", {}).items()):
        if path in excluded or path.startswith("/operator/") or not isinstance(path_item, Mapping):
            continue
        for method in ("get", "post", "put", "patch", "delete"):
            operation = path_item.get(method)
            if not isinstance(operation, Mapping):
                continue
            upper = method.upper()
            action_id = _path_action_id(upper, path, operation.get("operationId"))
            inputs, locations = _extract_inputs(operation, document)
            safety = _safety(upper, path)
            local_only = any(path.startswith(prefix) for prefix in _LOCAL_ONLY_PATH_PREFIXES)
            semantic_quarantine_reason = _OPERATOR_SEMANTIC_QUARANTINE_PATHS.get(path)
            unavailable_reason = (
                "Local-only maintenance route is not callable through the operator relay."
                if local_only
                else semantic_quarantine_reason
            )
            dispatchable = unavailable_reason is None
            action = {
                "action_id": action_id,
                "label": str(operation.get("summary") or operation.get("operationId") or f"{upper} {path}")[:160],
                "subsystem": _subsystem(path),
                "category": "route",
                "kind": "primitive",
                "safety_class": safety,
                "description": str(operation.get("description") or f"Exact existing robot route: {upper} {path}")[:2000],
                "source_anchor": None,
                "informational_method": upper,
                "informational_path": path,
                "required_provider_capability": _SERIAL206_PROVIDER_CAPABILITIES.get(path),
                "provider_available": dispatchable,
                "provider_unavailable_reason": unavailable_reason,
                "available": dispatchable,
                "unavailable_reason": unavailable_reason,
                "enabled": dispatchable,
                "disabled_reason": unavailable_reason,
                "dependencies": [],
                "requires_confirmation": safety not in {"read_only", "emergency", "stop"},
                "timeout_seconds": (
                    360.0
                    if path in _SERIAL206_PROVIDER_CAPABILITIES
                    else (120.0 if safety == "motion" else 30.0)
                ),
                "inputs": inputs,
                "stages": [],
            }
            actions.append(action)
            if dispatchable:
                dispatch[action_id] = {"method": upper, "path": path, "locations": locations, "inputs": {row["name"] for row in inputs}}

    def add_semantic_alias(
        *,
        action_id: str,
        path: str,
        label: str,
        description: str,
        source_anchor: str,
        fixed_inputs: Mapping[str, Any] | None = None,
        required_provider_capability: str | None = None,
    ) -> None:
        provider = next(
            (row for row in actions if row["informational_path"] == path and row["informational_method"] in {"GET", "POST"}),
            None,
        )
        if provider is None:
            return
        route = next(
            (row for row in dispatch.values() if row["path"] == path and row["method"] == provider["informational_method"]),
            None,
        )
        if route is None:
            if provider.get("provider_available") is not True:
                return
            route = {
                "method": str(provider.get("informational_method") or "POST"),
                "path": path,
                "locations": {},
                "inputs": {row["name"] for row in provider.get("inputs", []) if isinstance(row, Mapping) and row.get("name")},
            }
        fixed = dict(fixed_inputs or {})
        visible_inputs = [dict(row) for row in provider.get("inputs", []) if row.get("name") not in fixed]
        action_bounds = {
            "oem.z.move_steps": ("steps", -(2**31), 2**31 - 1),
            "oem.z.move_absolute": ("position_steps", -(2**31), 2**31 - 1),
            "oem.x.move_steps": ("steps", -(2**31), 2**31 - 1),
            "oem.x.move_absolute": ("position_steps", -(2**31), 2**31 - 1),
            "oem.y.move_steps": ("steps", -(2**31), 2**31 - 1),
            "oem.y.move_absolute": ("target_steps", -(2**31), 2**31 - 1),
        }.get(action_id)
        if action_bounds is not None:
            input_name, minimum, maximum = action_bounds
            for row in visible_inputs:
                if row.get("name") == input_name:
                    row.update(
                        {
                            "minimum": minimum,
                            "exclusive_minimum": None,
                            "maximum": maximum,
                            "exclusive_maximum": None,
                        }
                    )
        alias = {
            **dict(provider),
            "action_id": action_id,
            "label": label,
            "description": description,
            "source_anchor": source_anchor,
            "requires_confirmation": action_id in {
                "oem.z.set_home",
                "oem.x.set_home",
            },
            "category": (
                "x-axis"
                if action_id.startswith("oem.x.")
                else "y-axis"
                if action_id.startswith("oem.y.")
                else "x-composite"
                if action_id.startswith("oem.xy.") or action_id.startswith("oem.xyz.")
                else "z-axis"
            ),
            "inputs": visible_inputs,
            "required_provider_capability": required_provider_capability,
        }
        target = {
            **dict(route),
            "fixed_inputs": fixed,
            "inputs": {row["name"] for row in visible_inputs},
        }
        if action_id in _CANONICAL_COMPOSITE_ACTION_IDS:
            generated_action_id = str(provider["action_id"])
            provider.update(alias)
            dispatch.pop(generated_action_id, None)
            dispatch[action_id] = target
            return
        actions.append(alias)
        dispatch[action_id] = target

    x_semantic_actions = (
        ("oem.x.status", "/motion/oem/x/status", "X axis controller status", "Provider-owned X terminal telemetry and durable receipt projection. SAP12/SAP13 are observed because recovered OEM X initialization writes neither register.", "ClassMotor GAP1/GAP3/GAP4/GAP5/GAP6/GAP9/GAP10/GAP12/GAP13/GAP205"),

        ("oem.x.move_steps", "/motion/oem/x/move_steps", "OEM X moveSteps", "Signed-int32 relative request; ClassMotor returns -1 without movement when the resulting target crosses its 20-step inner margin.", "ClassControlInterface.moveSteps:4165-4204"),
        ("oem.x.move_absolute", "/motion/oem/x/move_absolute", "OEM X moveX", "Signed-int32 absolute request; the public wrapper clamps values below 60 and ClassHeadBoard returns the current position without movement at or above the high guard.", "ClassControlInterface.moveX:4206-4243"),
        ("oem.x.manual_panel_home", "/motion/oem/x/manual_home", "OEM X manual panel Home", "Exact manual-panel goHome(rehome=true, speed=500) identity.", "ClassControlInterface manual panel Home:2270-2278"),
        ("oem.x.diagnostic_home_axis", "/motion/oem/x/diagnostic_home_axis", "Diagnostic X HomeAxis", "Exact diagnostic axisSearchHome(X,250) identity; not the manual Home action.", "ClassControlInterface.HomeAxis:4997-5008"),
        ("oem.x.startup_home", "/motion/oem/x/startup_home", "OEM X startup home", "Exact startup axisSearchHome(X,250), setHome, profile restore, and park sequence.", "ClassControlInterface.initializeMotors:3367-3375"),
        ("oem.x.move_to_origin_home", "/motion/oem/x/move_to_origin_home", "OEM X moveTo-origin home", "Exact all-zero moveTo X child goHome(rehome=true, speed=1700).", "ClassControlInterface.moveTo:4463-4506"),
        ("oem.x.caught_plate_recovery_home", "/motion/oem/x/caught_plate_recovery_home", "OEM X caught-plate recovery home", "Exact caught-plate recovery X child goHome(rehome=false, speed=1700).", "ClassControlInterface.homeGZ:4657-4687"),
        ("oem.x.set_home", "/motion/oem/x/set_home", "Set OEM X home at current position", "Recovery-only no-motion SAP1=0 with direct readback and observation-gated reference publication.", "ClassMotor.setHome"),
        ("oem.x.set_max_speed", "/motion/oem/x/set_max_speed", "OEM X setMaxSpeed", "Set X maximum speed; input zero selects source default 1700.", "ClassControlInterface.setMaxSpeed:4689-4703"),
        ("oem.x.set_max_acc", "/motion/oem/x/set_max_acc", "OEM X setMaxAcc", "Set X maximum acceleration; input zero selects source default 350.", "ClassControlInterface.setMaxAcc:4729-4743"),
        ("oem.x.restore_original_speed", "/motion/oem/x/restore_original_speed", "OEM X restoreOriginalSpeed", "Restore X speed parameter 4 to source default 1700.", "ClassControlInterface.restoreOriginalSpeed:4769-4779"),
        ("oem.x.set_stall_guard", "/motion/oem/x/set_stall_guard", "OEM X setStallGuard", "Set X stall guard; input zero selects source default 16.", "ClassControlInterface.setStallGuard:4869-4883"),
        ("oem.x.stop", "/motion/oem/x/stop", "OEM X double-stop", "Source-exact double StopMotor. The source method does not wait for zero speed; terminal-state proof is a separate observation.", "ClassMotor.StopMotor:161-183"),
        ("oem.abort_all", "/motion/oem/x/abort", "Aggregate OEM abort (all boards)", "Invoke forceAbortMotion across all present OEM motion boards. This action has aggregate machine scope.", "ClassControlInterface.forceAbortMotion:5095-5106"),
        ("oem.x.observe", "/motion/oem/x/observation", "Record physical X observation", "Bind an independent physical pass/fail observation to the exact provider command and ownership generation.", "Serial206OemInitializationProvider X observation contract"),
        ("oem.xy.home_xy", "/motion/oem/home_xy", "OEM HomeXY", "Source-shaped concurrent X/Y home with signed source returns and provider-owned reference publication.", "ClassControlInterface.HomeXY:5054-5070"),
        ("oem.xy.move_xy", "/motion/oem/move_xy", "OEM moveXY", "Source-shaped X/Y coordinated movement, including the literal missing-board fallbacks.", "ClassControlInterface.moveXY:4285-4367"),
        ("oem.xyz.move_to", "/motion/oem/move_to", "OEM moveTo", "Provider-owned XYZ moveTo composite with X lifecycle and all-zero observation authority.", "ClassControlInterface.moveTo:4463-4506"),
        ("oem.xy.enable", "/motion/oem/x/internal/enable_xy", "OEM enableXY current mode", "Provider-owned X/Y current transaction. Advanced catalog only; this is not a normal X-card action.", "ClassControlInterface.enableXY:5161-5194"),
        ("oem.xyz.enable", "/motion/oem/x/internal/enable_xyz", "OEM enableXYZ current mode", "Provider-owned X/Y/Z current transaction. Advanced catalog only; this is not a normal X-card action.", "ClassControlInterface.enableXYZ:5113-5159"),
    )
    for action_id, path, label, description, source_anchor in x_semantic_actions:
        add_semantic_alias(
            action_id=action_id,
            path=path,
            label=label,
            description=description,
            source_anchor=source_anchor,
            fixed_inputs=(
                {"operator_ack": "MOVETO", "timeout_s": 120.0}
                if action_id == "oem.xyz.move_to"
                else None
            ),
            required_provider_capability="initialize_motors",
        )
    y_semantic_actions = (
        ("oem.y.status", "/motion/oem/y/status", "OEM Y status", "Passive Serial-206 Y authority and controller-status projection.", "ClassControlInterface Y status", {}),
        ("oem.y.manual_panel_home", "/motion/oem/y/home", "OEM Y manual-panel Home", "Exact manual-panel Y home identity.", "ClassControlInterface.btnYHome_Click:1847-1856", {"source_mode": "manual_panel"}),
        ("oem.y.move_steps", "/motion/oem/y/move_steps", "OEM Y moveSteps", "Signed-int32 relative request; ClassMotor returns -1 without movement when the resulting target crosses its 20-step inner margin.", "ClassControlInterface.moveSteps:4165-4204", {}),
        ("oem.y.move_absolute", "/motion/oem/y/move_absolute", "OEM Y moveY absolute", "Signed-int32 source-default blocking request; ClassHeadBoard clamps negative values to zero and returns the current position without movement at or above the high guard.", "ClassControlInterface.moveY overloads:4206-4252", {}),
        ("oem.y.stop", "/motion/oem/y/stop", "OEM Y Stop", "Independent priority-lane Y stop.", "ClassMotor.stopMotor", {}),
    )
    for action_id, path, label, description, source_anchor, fixed_inputs in y_semantic_actions:
        add_semantic_alias(
            action_id=action_id,
            path=path,
            label=label,
            description=description,
            source_anchor=source_anchor,
            fixed_inputs=fixed_inputs,
            required_provider_capability="initialize_motors",
        )
    add_semantic_alias(
        action_id="oem.y.internal.acceleration_overload",
        path="/motion/oem/y/internal/acceleration_overload",
        label="Internal Y acceleration overload",
        description="Typed source-only M04 acceleration overload with mandatory restoration.",
        source_anchor="ClassControlInterface.moveY acceleration overload",
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.y.internal.board_test_my",
        path="/motion/oem/y/internal/board_test_my",
        label="Internal board_test_my Y move",
        description="Typed board-test moveY source identity with fixed source acceleration.",
        source_anchor="board_test_my",
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.xy.move_absolute",
        path="/motion/oem/move_xy",
        label="OEM moveXY absolute",
        description="Exact OEM parallel X/Y absolute composite.",
        source_anchor="ClassControlInterface.moveXY",
        fixed_inputs={"timeout_s": 120.0},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.xy.home",
        path="/motion/oem/home_xy",
        label="OEM HomeXY",
        description="Exact OEM parallel HomeXY composite.",
        source_anchor="ClassControlInterface.HomeXY",
        fixed_inputs={"operator_ack": "HOMEXY"},
        required_provider_capability="initialize_motors",
    )
    x_abort_action = next((row for row in actions if row.get("action_id") == "oem.abort_all"), None)
    if x_abort_action is not None:
        x_abort_action.update({
            "aggregate_abort": True,
            "physical_scope": "aggregate_oem_all_present_boards",
            "x_only": False,
            "category": "x-axis",
        })

    add_semantic_alias(
        action_id="oem.z.set_home",
        path="/motion/oem/z/set_home",
        label="Set OEM Z home at current position (no motion)",
        description="No-motion manual home: record the current physical position as controller 0 via ClassMotor.setHome (SAP param 1 = 0) with readback and a durable reference mark.",
        source_anchor="ClassMotor.setHome; SAP param 1 = actual position",
        fixed_inputs={"axis": "z"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.move_steps",
        path="/motion/oem/manual/relative",
        label="OEM Z moveSteps",
        description="Source-shaped relative Z movement on serial-206 board 4 motor 1.",
        source_anchor="ClassControlInterface.moveSteps:4165-4204",
        fixed_inputs={"axis": "z"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.move_absolute",
        path="/motion/oem/manual/absolute",
        label="OEM Z moveZ",
        description="Source-shaped absolute Z movement with robot-owned DefaultParameters.PSUDO_Z_HOME.",
        source_anchor="ClassControlInterface.moveZ:4254-4265; DefaultParameters:47-84",
        fixed_inputs={"axis": "z", "wait_timeout_s": 20.0},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.clear",
        path="/motion/oem/z/clear",
        label="OEM Z Clear",
        description="Move Z from the home interlock to the robot-owned PSUDO_Z_HOME selected from durable tip and gantry state.",
        source_anchor="DefaultParameters:47-84; ClassControlInterface.moveZ:4254-4265",
        fixed_inputs={"axis": "z"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.manual_home",
        path="/motion/oem/manual/home",
        label="OEM Z manual goHome (1791)",
        description="Source btnHomeZ_Click Z home: goHome(rehome=true, speed=1791) with post-home controller proof.",
        source_anchor="ClassControlInterface.btnHomeZ_Click:2370-2378",
        fixed_inputs={"axis": "z"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.move_z_home",
        path="/motion/oem/z/move_z_home",
        label="OEM Z MoveZHome",
        description="Distinct source MoveZHome identity: set Z max current, then goHome(rehome=true, speed=1791).",
        source_anchor="ClassControlInterface.MoveZHome:4623-4632",
        fixed_inputs={"rehome": True},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.control",
        path="/motion/oem/z/control",
        label="OEM Z control",
        description="Canonical typed Z control operation for the existing serial-206 provider route.",
        source_anchor="ClassControlInterface.setMaxSpeed/setMaxAcc/restoreOriginalSpeed/setZaxisVmax/setZaxisCurrentmax",
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.path_clean_mode",
        path="/motion/oem/z/path_clean_mode",
        label="OEM Z path clean mode",
        description="Canonical typed clean-path mode for source-shaped moveTo/scriptmoveTo planning.",
        source_anchor="ClassControlInterface.scriptmoveTo:3875-3903",
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.set_max_speed",
        path="/motion/oem/z/control",
        label="OEM Z setMaxSpeed",
        description="Set the Z maximum speed; the OEM default argument 0 is mapped to 1791.",
        source_anchor="ClassControlInterface.setMaxSpeed:4689-4724",
        fixed_inputs={"operation": "set_max_speed"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.set_max_acc",
        path="/motion/oem/z/control",
        label="OEM Z setMaxAcc",
        description="Set the Z maximum acceleration; zero restores the source default 576.",
        source_anchor="ClassControlInterface.setMaxAcc:4729-4764",
        fixed_inputs={"operation": "set_max_acc"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.set_vmax",
        path="/motion/oem/z/control",
        label="OEM Z setZaxisVmax",
        description="Set source Z Vmax/speed parameter 4; OEM argument zero restores 1791.",
        source_anchor="ClassControlInterface.setZaxisVmax:4835-4848",
        fixed_inputs={"operation": "set_vmax"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.set_current_max",
        path="/motion/oem/z/control",
        label="OEM Z setZaxisCurrentmax",
        description="Set Z maximum current; OEM default sentinel 100 or omission selects the machine-bound down-current value.",
        source_anchor="ClassControlInterface.setZaxisCurrentmax:4850-4860",
        fixed_inputs={"operation": "set_current_max"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.restore_original_speed",
        path="/motion/oem/z/control",
        label="OEM Z restoreOriginalSpeed",
        description="Restore source-original Z speed parameter 4 to 1791.",
        source_anchor="ClassControlInterface.restoreOriginalSpeed:4769-4792",
        fixed_inputs={"operation": "restore_original_speed"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.set_clean_path",
        path="/motion/oem/z/path_clean_mode",
        label="Set OEM clean-path mode",
        description="Persist m_controlLib.cleanPath under the current provider generation; subsequent live scriptmoveTo planning reads this state instead of caller payload.",
        source_anchor="ClassControlInterface.scriptmoveTo:3875-3903",
        fixed_inputs={"axis": "z"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.scriptmove_to",
        path="/motion/oem/pathing/scriptmove_execute",
        label="OEM Z coordinated scriptmoveTo",
        description="Execute source-shaped coordinated moveTo/scriptmoveTo with every Z leg owned by the serial-206 Z lifecycle.",
        source_anchor="ClassControlInterface.scriptmoveTo:3718-4014; moveTo:4463-4620",
        fixed_inputs={
            "mode": "live",
            "operator_ack": "OEM_PATH_EXECUTE",
            "reason": "operator-invoked OEM scriptmoveTo via BioModStack operator plane",
            "current_loc": None,
            "current_well": None,
            "current_x": 0,
            "current_y": 0,
            "current_z": 0,
            "tip_loaded": False,
            "tip_dirty": False,
            "tip_location": -1,
            "clean_path": False,
            "device_type": "BIOXP",
            "gripper_confirmed": False,
            "plate_on_gantry": None,
            "location19_y": None,
            "pseudo_z_home": None,
        },
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.move_gz",
        path="/motion/oem/z/move_gz",
        label="OEM Z moveGZ",
        description="Launch gripper and Z absolute targets together and wait for both source events.",
        source_anchor="ClassControlInterface.moveGZ:4369-4399",
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.home_gz",
        path="/motion/oem/home_gz",
        label="OEM Z homeGZ",
        description="Execute source homeGZ pseudo-home, gripper-home, caught-plate branch, and current restore transaction.",
        source_anchor="ClassControlInterface.homeGZ:4657-4687",
        fixed_inputs={"mode": "live", "operator_ack": "OEM_HOME_GZ"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.lower_pipette",
        path="/motion/oem/z/lower_pipette",
        label="OEM Z lowerPipette",
        description="Move Z to immutable PositionTable zLow, with the exact optional +4030 overpress branch and one-second settle.",
        source_anchor="ClassControlInterface.lowerPipette:4401-4421",
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.lift_pipette",
        path="/motion/oem/z/lift_pipette",
        label="OEM Z liftPipette",
        description="Move Z to immutable PositionTable zHigh.",
        source_anchor="ClassControlInterface.liftPipette:4423-4431",
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.self_test",
        path="/motion/oem/z/self_test",
        label="OEM Z self-test",
        description="Run the source-bearing Z self-test order: MoveZHome, move to immutable SelfTestZMax, final HomeAxis(\"z\"), and require travel error at most 100 steps.",
        source_anchor="ControlLib.selfTest:10744-10749; ClassBioXPSettings.SelfTestZMax",
        fixed_inputs={},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.diagnostic_home_axis",
        path="/motion/oem/z/diagnostic_home_axis",
        label="Diagnostic Z HomeAxis (597)",
        description="Diagnostic-only HomeAxis(\"z\") at 597; never presented as manual or startup home.",
        source_anchor="ClassControlInterface.HomeAxis:4997-5052",
        fixed_inputs={"axis": "z"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.stop",
        path="/motion/oem/z/stop",
        label="OEM Z double-stop",
        description="Send the source-shaped unconditional double StopMotor and verify terminal zero speed.",
        source_anchor="ClassMotor.StopMotor:161-182",
        fixed_inputs={"axis": "z"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.resume_after_abort",
        path="/motion/oem/z/resume_after_abort",
        label="OEM Z after-abort recovery rehome",
        description="Execute only the Z-bearing recovery sequence: powered initialCheck, then startup axisSearchHome(1791). This does not claim or release full application wakefrompause readiness.",
        source_anchor="ControlLib.wakefrompause; ControlLib.rehome:8784-8796; initializeMotors Z stage:3350-3353",
        fixed_inputs={},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.observe",
        path="/motion/oem/z/observation",
        label="Record physical Z observation",
        description="Bind an independent physical pass/fail observation to the exact provider command receipt.",
        source_anchor="Serial206OemInitializationProvider Z durable observation contract",
        fixed_inputs={"axis": "z"},
        required_provider_capability="initialize_motors",
    )
    add_semantic_alias(
        action_id="oem.z.status",
        path="/motion/oem/z/status",
        label="Z axis authority status",
        description="Robot-owned Z lifecycle, position, switch masks, reference, receipts, and freshness projection.",
        source_anchor="ClassMotor GAP1/GAP3/GAP9/GAP10/GAP12/GAP13",
        fixed_inputs={"axis": "z"},
    )
    prepare_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/oem/prepare_without_motion" and row["informational_method"] == "POST"),
        None,
    )
    recovery_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/arm/strict_startup" and row["informational_method"] == "POST"),
        None,
    )

    home_xy_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/oem/home_xy" and row["informational_method"] == "POST"),
        None,
    )
    initialize_motors_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/oem/initialization/initialize_motors" and row["informational_method"] == "POST"),
        None,
    )
    initialize_motion_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/oem/initialization/initialize_motion" and row["informational_method"] == "POST"),
        None,
    )
    prepare_bound = prepare_provider is not None and prepare_provider.get("provider_available") is True
    recovery_bound = recovery_provider is not None and recovery_provider.get("provider_available") is True

    home_xy_bound = home_xy_provider is not None and home_xy_provider.get("provider_available") is True
    initialize_motors_bound = initialize_motors_provider is not None and initialize_motors_provider.get("provider_available") is True
    initialize_motion_bound = initialize_motion_provider is not None and initialize_motion_provider.get("provider_available") is True
    meta = [
        {
            "action_id": "meta.activate_motion",
            "label": "Activate 24 V / Prepare Motion (OEM, No Movement)",
            "subsystem": "meta",
            "category": "activation",
            "kind": "meta",
            "safety_class": "service",
            "description": "Prepare the serial-206 motion path using the OEM safety check, cmd64=0→1 board cycle, and initializeMotorsWithoutMotion sequence. This does not home or move an axis.",
            "source_anchor": "ClassIOControl.query24VSensor:92-110; ClassControlInterface.initializeMotorsWithoutMotion:3181-3265; activateBoard:3474-3493",
            "informational_method": "POST",
            "informational_path": "/motion/oem/prepare_without_motion",
            "provider_available": prepare_bound,
            "provider_unavailable_reason": None if prepare_bound else "Source-grounded no-motion preparation provider is not bound.",
            "available": prepare_bound,
            "unavailable_reason": None if prepare_bound else "Source-grounded no-motion preparation provider is not bound.",
            "enabled": prepare_bound,
            "disabled_reason": None if prepare_bound else "Source-grounded no-motion preparation provider is not bound.",
            "dependencies": [],
            "requires_confirmation": True,
            "timeout_seconds": 120.0,
            "inputs": list(prepare_provider.get("inputs", [])) if prepare_provider else [],
            "stages": ["serial-206 authority", "24 V/door/latch query", "cmd64=0 boards 4/5/6/7", "cmd64=1 boards 4/5/6/7", "mint board generation", "initializeMotorsWithoutMotion", "exact parameter readback"],
        },
        {
            "action_id": "meta.recover_motion_non_homing",
            "label": "OEM Non-homing Motion Recovery",
            "subsystem": "meta",
            "category": "recovery",
            "kind": "meta",
            "safety_class": "service",
            "description": "Run the existing robot-owned strict startup recovery with homing disabled. This action is available only while the maintenance latch requires recovery.",
            "source_anchor": "Motion strict startup; run_homing=false",
            "informational_method": "POST",
            "informational_path": "/motion/arm/strict_startup",
            "provider_available": recovery_bound,
            "provider_unavailable_reason": None if recovery_bound else "Robot-owned non-homing recovery provider is not bound.",
            "available": recovery_bound,
            "unavailable_reason": None if recovery_bound else "Robot-owned non-homing recovery provider is not bound.",
            "enabled": recovery_bound,
            "disabled_reason": None if recovery_bound else "Robot-owned non-homing recovery provider is not bound.",
            "dependencies": [],
            "requires_confirmation": False,
            "timeout_seconds": 90.0,
            "inputs": [],
            "stages": ["strict startup", "no homing", "maintenance latch completion"],
        },

        {
            "action_id": "meta.home_xy",
            "label": "Home XY (OEM Task.Run/WaitAll)",
            "subsystem": "meta",
            "category": "homing",
            "kind": "meta",
            "safety_class": "motion",
            "description": "OEM HomeXY mode: set X/Y speed/acceleration, launch X/Y goHome concurrently, wait for both, and restore action current.",
            "source_anchor": "ClassControlInterface.HomeXY:5056-5061",
            "informational_method": "POST",
            "informational_path": "/motion/oem/home_xy",
            "provider_available": home_xy_bound,
            "provider_unavailable_reason": None if home_xy_bound else "OEM HomeXY provider route is not bound.",
            "available": home_xy_bound,
            "unavailable_reason": None if home_xy_bound else "OEM HomeXY provider route is not bound.",
            "enabled": home_xy_bound,
            "disabled_reason": None if home_xy_bound else "OEM HomeXY provider route is not bound.",
            "dependencies": [],
            "requires_confirmation": True,
            "timeout_seconds": 180.0,
            "inputs": list(home_xy_provider.get("inputs", [])) if home_xy_provider else [],
            "stages": ["set X/Y speedacc=200", "launch X goHome", "launch Y goHome", "wait all", "restore action current"],
        },
        {
            "action_id": "meta.initialize_motors",
            "label": "OEM initializeMotors — Execute Expected Stage",
            "subsystem": "meta",
            "category": "initialization",
            "kind": "meta",
            "safety_class": "motion",
            "description": "Execute exactly one durable, source-ordered ClassControlInterface.initializeMotors stage using its matching approval/commissioning block.",
            "source_anchor": "ClassControlInterface.initializeMotors:3348-3421",
            "informational_method": "POST",
            "informational_path": "/motion/oem/initialization/initialize_motors",
            "required_provider_capability": "initialize_motors",
            "provider_available": initialize_motors_bound,
            "provider_unavailable_reason": None if initialize_motors_bound else "OEM initializeMotors provider route is not bound.",
            "available": initialize_motors_bound,
            "unavailable_reason": None if initialize_motors_bound else "OEM initializeMotors provider route is not bound.",
            "enabled": initialize_motors_bound,
            "disabled_reason": None if initialize_motors_bound else "OEM initializeMotors provider route is not bound.",
            "dependencies": [],
            "requires_confirmation": True,
            "timeout_seconds": 360.0,
            "inputs": list(initialize_motors_provider.get("inputs", [])) if initialize_motors_provider else [],
            "stages": list(OEM_INITIALIZE_MOTORS_STAGE_KEYS),
        },
        {
            "action_id": "meta.initialize_motion",
            "label": "OEM initializeMotion — Execute Expected Stage",
            "subsystem": "meta",
            "category": "initialization",
            "kind": "meta",
            "safety_class": "motion",
            "description": "Execute exactly one durable ControlLib.initializeMotion stage, including initializeMotors call-through, tip branch/ejection, source scriptmoveTo, final X/Z moves, pipette initiateGroup retry, and OEM exception effects.",
            "source_anchor": "ControlLib.initializeMotion:8797-8856",
            "informational_method": "POST",
            "informational_path": "/motion/oem/initialization/initialize_motion",
            "required_provider_capability": "initialize_motion",
            "provider_available": initialize_motion_bound,
            "provider_unavailable_reason": None if initialize_motion_bound else "OEM initializeMotion provider route is not bound.",
            "available": initialize_motion_bound,
            "unavailable_reason": None if initialize_motion_bound else "OEM initializeMotion provider route is not bound.",
            "enabled": initialize_motion_bound,
            "disabled_reason": None if initialize_motion_bound else "OEM initializeMotion provider route is not bound.",
            "dependencies": [],
            "requires_confirmation": True,
            "timeout_seconds": 360.0,
            "inputs": list(initialize_motion_provider.get("inputs", [])) if initialize_motion_provider else [],
            "stages": [spec.key for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS],
        },
    ]
    # Meta actions dispatch to exact bound provider routes; no synthesized sequence.
    home_route = next((row for row in dispatch.values() if row["method"] == "POST" and row["path"] == "/motion/oem/home_xy"), None)
    prepare_route = next((row for row in dispatch.values() if row["method"] == "POST" and row["path"] == "/motion/oem/prepare_without_motion"), None)
    recovery_route = next((row for row in dispatch.values() if row["method"] == "POST" and row["path"] == "/motion/arm/strict_startup"), None)

    initialize_motors_route = next((row for row in dispatch.values() if row["method"] == "POST" and row["path"] == "/motion/oem/initialization/initialize_motors"), None)
    initialize_motion_route = next((row for row in dispatch.values() if row["method"] == "POST" and row["path"] == "/motion/oem/initialization/initialize_motion"), None)
    if home_route:
        dispatch["meta.home_xy"] = home_route
    if prepare_route:
        dispatch["meta.activate_motion"] = prepare_route
    if recovery_route:
        dispatch["meta.recover_motion_non_homing"] = {
            **dict(recovery_route),
            "fixed_inputs": {"run_homing": False},
            "inputs": set(),
        }

    if initialize_motors_route:
        dispatch["meta.initialize_motors"] = initialize_motors_route
    if initialize_motion_route:
        dispatch["meta.initialize_motion"] = initialize_motion_route
    actions.extend(meta)
    actions = [
        row for row in actions
        if not str(row.get("action_id", "")).startswith("oem.y.internal.")
        and not (
            str(row.get("informational_path") or "") == "/motion/oem/x/abort"
            and row.get("action_id") != "oem.abort_all"
        )
    ]
    # Internal XYZ composite helpers remain outside the operator action catalog.
    dispatch = {key: value for key, value in dispatch.items() if not key.startswith("oem.xyz.")}
    return actions, dispatch


async def _dispatch_asgi(app: FastAPI, method: str, path_template: str, inputs: dict[str, Any], locations: Mapping[str, Mapping[str, Any]]) -> tuple[int, Any]:
    path = path_template
    query: dict[str, Any] = {}
    body: dict[str, Any] = {}
    allowed = set(locations)
    if set(inputs) - allowed:
        raise HTTPException(status_code=422, detail={"error": "unknown_action_inputs", "unknown": sorted(set(inputs) - allowed)})
    for name, metadata in locations.items():
        if metadata.get("implicit_operator_control"):
            if name == "operator_ack":
                value = _implicit_operator_ack(path_template, inputs)
                if value is None:
                    continue
            else:
                value = "operator control invocation"
        else:
            if name not in inputs:
                continue
            value = inputs[name]
        location = str(metadata["location"])
        wire_name = str(metadata["wire_name"])
        if location == "path":
            path = path.replace("{" + wire_name + "}", str(value))
        elif location == "query":
            query[wire_name] = value
        elif name == "body" and isinstance(value, dict):
            body = value
        else:
            body[wire_name] = value
    if "{" in path or "}" in path:
        raise HTTPException(status_code=422, detail="required path input missing")
    raw_body = json.dumps(body, separators=(",", ":")).encode() if body else b""
    if len(raw_body) > _MAX_INPUT_BYTES:
        raise HTTPException(status_code=413, detail="action body exceeds bounded input limit")
    response_start: dict[str, Any] = {}
    chunks: list[bytes] = []
    request_sent = False

    async def receive() -> Message:
        nonlocal request_sent
        if request_sent:
            return {"type": "http.disconnect"}
        request_sent = True
        return {"type": "http.request", "body": raw_body, "more_body": False}

    async def send(message: Message) -> None:
        if message["type"] == "http.response.start":
            response_start.update(message)
        elif message["type"] == "http.response.body":
            chunks.append(message.get("body", b""))

    headers = [(b"content-type", b"application/json"), (b"accept", b"application/json")]
    scope: Scope = {
        "type": "http", "asgi": {"version": "3.0", "spec_version": "2.3"}, "http_version": "1.1",
        "method": method, "scheme": "http", "path": path, "raw_path": path.encode(),
        "query_string": urlencode(query, doseq=True).encode(), "root_path": "", "headers": headers,
        "client": ("127.0.0.1", 0), "server": ("bioxp-internal", 80), "state": {},
    }
    await app(scope, receive, send)
    status = int(response_start.get("status", 500))
    raw = b"".join(chunks)
    if len(raw) > _MAX_INTERNAL_RESPONSE_BYTES:
        return status, {
            "ok": False,
            "failure": "internal_response_exceeded_evidence_limit",
            "response_bytes": len(raw),
            "evidence_limit_bytes": _MAX_INTERNAL_RESPONSE_BYTES,
            "sha256": hashlib.sha256(raw).hexdigest(),
        }
    try:
        return status, json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return status, {"body": raw.decode("utf-8", "replace")}


def install_operator_control_plane(
    app: FastAPI,
    *,
    maintenance_state_provider: Callable[[], Mapping[str, Any]] | None = None,
    reference_state_provider: Callable[[], Mapping[str, Any]] | None = None,
    lifecycle_state_provider: Callable[[], Mapping[str, Any]] | None = None,
    serial206_initialization_state_provider: Callable[[], Mapping[str, Any]] | None = None,
    pipette_status_provider: Callable[[], Mapping[str, Any]] | None = None,
) -> None:
    """Snapshot final routes and mount the robot-authoritative operator plane."""
    actions, dispatch = _build_catalog(app)
    actions = [
        row
        for row in actions
        if str(row.get("action_id", "")) not in _PRIVATE_METHOD_ACTION_IDS
    ]
    by_id = {row["action_id"]: row for row in actions}
    v2_canonical_action_ids = _v2_canonical_action_ids(actions, dispatch)
    store = OperatorReceiptStore()
    invoke_lock = asyncio.Lock()
    interrupt_lock = asyncio.Lock()
    router = APIRouter(prefix="/operator", tags=["operator-controls"])

    def machine_state() -> dict[str, Any]:
        domain_names = ("transport", "boards", "axes", "range", "power", "interlock", "latch", "gripper", "thermal", "chiller", "pipette")
        domains: dict[str, Any] = {}
        snapshot_id = None
        freshness_rows: list[dict[str, Any]] = []
        for name in domain_names:
            projection = hardware_state.project(name)
            row = (projection.get("domains") or {}).get(name)
            domains[name] = row if isinstance(row, Mapping) else {"status": "unknown", "observation": None, "error": "not reported"}
            if projection.get("snapshot_id"):
                snapshot_id = projection.get("snapshot_id")
            freshness = projection.get("freshness")
            if isinstance(freshness, Mapping):
                freshness_rows.append(dict(freshness))
        if pipette_status_provider is not None:
            pipette_status = pipette_status_provider()
            if not isinstance(pipette_status, Mapping):
                raise RuntimeError("pipette status provider returned a non-mapping payload")
            domains["pipette"] = {
                "status": "observed" if pipette_status.get("ok") is True else "unavailable",
                "observation": dict(pipette_status),
                "error": None if pipette_status.get("ok") is True else "passive pipette status unavailable",
            }
        ownership_projection = hardware_state.ownership_projection()
        state_rank = {"fresh": 0, "stale": 1, "missing": 2}
        freshness = max(
            freshness_rows,
            key=lambda row: state_rank.get(str(row.get("state")), 2),
            default={"state": "missing", "age_s": None, "fresh_for_s": 30.0},
        )
        ages = [float(row["age_s"]) for row in freshness_rows if isinstance(row.get("age_s"), (int, float))]
        windows = [float(row["fresh_for_s"]) for row in freshness_rows if isinstance(row.get("fresh_for_s"), (int, float))]
        freshness = {
            **dict(freshness),
            "age_s": max(ages) if ages else None,
            "fresh_for_s": min(windows) if windows else 30.0,
        }
        return {
            "ownership_generation": int(ownership_projection["ownership_epoch"]),
            "ownership": ownership_projection["ownership"],
            "maintenance": dict(maintenance_state_provider()) if maintenance_state_provider else {
                "motion_blocked": True,
                "recovery_required": True,
                "block_reason": "Motion state is not bound to the operator plane.",
            },
            "lifecycle": dict(lifecycle_state_provider()) if lifecycle_state_provider else lifecycle_state.projection(),
            "serial206_initialization_provider": (
                dict(serial206_initialization_state_provider())
                if serial206_initialization_state_provider
                else {"bound": False, "initialize_motors_live_available": False, "initialize_motion_live_available": False}
            ),
            "references": dict(reference_state_provider()) if reference_state_provider else {"rows": {}},
            "domains": domains,
            "freshness": freshness or {"state": "missing", "age_s": None, "fresh_for_s": 30.0},
            "snapshot_id": snapshot_id,
        }

    legacy_command_store = OperatorHistoryReader()
    app.state.operator_history_reader = legacy_command_store
    app.state.operator_receipt_store = store

    def replay_authority_fingerprint(state: Mapping[str, Any]) -> str:
        authority_projection = {
            "ownership_generation": state.get("ownership_generation"),
            "ownership": state.get("ownership"),
            "maintenance": state.get("maintenance"),
            "lifecycle": state.get("lifecycle"),
            "serial206_initialization_provider": state.get("serial206_initialization_provider"),
            "references": state.get("references"),
            "domains": state.get("domains"),
            "freshness_state": (state.get("freshness") or {}).get("state"),
            "snapshot_id": state.get("snapshot_id"),
        }
        encoded = json.dumps(
            authority_projection,
            sort_keys=True,
            separators=(",", ":"),
            default=str,
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def replay_source_identity() -> dict[str, Any]:
        release = current_release_identity()
        if release.get("verified") is not True:
            raise HTTPException(status_code=409, detail="verified release identity is required for replay")
        source_value = release.get("source")
        source = source_value if isinstance(source_value, Mapping) else {}
        try:
            source_authority = current_authority_identity()
            registry_sha256 = current_registry_sha256()
        except Exception as exc:
            raise HTTPException(status_code=409, detail=f"serial-206 replay authority unavailable: {exc}") from exc
        if source_authority.get("evidence_lock_identity_verified") is not True:
            raise HTTPException(status_code=409, detail="verified serial-206 evidence-lock identity is required for replay")
        return {
            "robot_identity": os.getenv("BIOXP_ROBOT_IDENTITY", "serial206").strip() or "serial206",
            "release_id": release.get("release_id") if isinstance(release.get("release_id"), str) else None,
            "source_manifest_sha256": source.get("manifest_sha256") if isinstance(source.get("manifest_sha256"), str) else None,
            "source_aggregate_sha256": source.get("aggregate_sha256") if isinstance(source.get("aggregate_sha256"), str) else None,
            "release_verified": True,
            "registry_sha256": registry_sha256,
            "evidence_lock_sha256": source_authority.get("evidence_lock_sha256"),
            "evidence_lock_identity_verified": True,
        }

    def verify_replay_source_identity(receipt: Mapping[str, Any]) -> None:
        stored = receipt.get("source_identity")
        if not isinstance(stored, Mapping):
            raise HTTPException(status_code=409, detail="idempotency receipt source identity unavailable")
        for key, expected_value in replay_source_identity().items():
            if stored.get(key) != expected_value:
                raise HTTPException(status_code=409, detail=f"idempotency receipt {key} is stale")

    def assessed_action(action: Mapping[str, Any], state: Mapping[str, Any], inputs: Mapping[str, Any] | None = None) -> dict[str, Any]:
        assessment = _assess_action(action, state, inputs)
        return {
            **dict(action),
            **assessment,
            "available": assessment["enabled"],
            "unavailable_reason": assessment["disabled_reason"],
        }

    def authority() -> dict[str, Any]:
        try:
            identity = current_authority_identity()
            return {
                "registry_sha256": current_registry_sha256(),
                "evidence_lock_sha256": identity["evidence_lock_sha256"],
                "source_authority_verified": bool(identity.get("evidence_lock_identity_verified")),
            }
        except (OemFullLifecycleError, OSError, ValueError, KeyError):
            return {"registry_sha256": "unavailable", "evidence_lock_sha256": "unavailable", "source_authority_verified": False}

    def _v2_bounded_failure_detail(row: Mapping[str, Any]) -> dict[str, Any] | None:
        response = row.get("response")
        body = response.get("body") if isinstance(response, Mapping) else None
        detail = body.get("detail") if isinstance(body, Mapping) else None
        result = detail.get("result") if isinstance(detail, Mapping) else None
        home_container = result.get("home") if isinstance(result, Mapping) else None
        home = home_container.get("home") if isinstance(home_container, Mapping) else None
        lifecycle = detail.get("z_lifecycle") if isinstance(detail, Mapping) else None
        if not isinstance(detail, Mapping):
            return None
        if not isinstance(result, Mapping):
            return None
        if not isinstance(home, Mapping):
            return None
        if not isinstance(lifecycle, Mapping):
            return None
        provider_failure = result.get("failure")
        failure = home.get("failure")
        axis = home.get("axis")
        board = home.get("board")
        motor = home.get("motor")
        source_return_code = home.get("source_return_code")
        controller_acknowledged = result.get("controller_command_acknowledged")
        controller_terminal_state_verified = result.get("controller_terminal_state_verified")
        physical_effect_verified = result.get("physical_effect_verified")
        lifecycle_state = lifecycle.get("state")
        reference_state = lifecycle.get("reference_state")
        if not (
            isinstance(provider_failure, str) and 0 < len(provider_failure) <= 160
            and isinstance(failure, str) and 0 < len(failure) <= 160
            and isinstance(axis, str) and 0 < len(axis) <= 16
            and type(board) is int and 0 <= board <= 255
            and type(motor) is int and 0 <= motor <= 255
            and type(source_return_code) is int and -(2 ** 31) <= source_return_code < 2 ** 31
            and type(controller_acknowledged) is bool
            and type(controller_terminal_state_verified) is bool
            and type(physical_effect_verified) is bool
            and isinstance(lifecycle_state, str) and 0 < len(lifecycle_state) <= 160
            and isinstance(reference_state, str) and 0 < len(reference_state) <= 160
        ):
            return None
        return {
            "provider_failure": provider_failure,
            "failure": failure,
            "axis": axis,
            "board": board,
            "motor": motor,
            "source_return_code": source_return_code,
            "controller_acknowledged": controller_acknowledged,
            "controller_terminal_state_verified": controller_terminal_state_verified,
            "physical_effect_verified": physical_effect_verified,
            "lifecycle_state": lifecycle_state,
            "reference_state": reference_state,
        }

    def _v2_compact_receipt(row: Mapping[str, Any]) -> dict[str, Any]:
        raw_status = str(row.get("status") or "queued")
        status = {
            "acknowledged": "queued",
            "admission_pending": "queued",
            "blocked": "rejected",
            "reconciliation_required": "failed",
            "stop_requested": "interrupting",
            "abort_requested": "interrupting",
            "stopped": "interrupted",
            "aborted": "interrupted",
            "cancelled": "cleared",
        }.get(raw_status, raw_status)
        if status not in {"queued", "dispatched", "issued_pending", "interrupting", "completed", "failed", "cleared", "interrupted", "ambiguous", "rejected"}:
            status = "failed"
        now = time.time()
        def finite_time(value: Any) -> float | None:
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                return None
            return parsed if math.isfinite(parsed) else None
        accepted = finite_time(row.get("accepted_at") or row.get("started_at")) or now
        queued = finite_time(row.get("queued_at")) or accepted
        dispatched = finite_time(row.get("dispatched_at"))
        finished = finite_time(row.get("finished_at"))
        error = None
        if status in {"failed", "rejected", "ambiguous"}:
            error = {"code": str(row.get("error") or row.get("reason") or raw_status), "message": str(row.get("error") or row.get("reason") or raw_status), "retryable": status == "ambiguous"}
            failure_detail = _v2_bounded_failure_detail(row)
            if failure_detail is not None:
                error["detail"] = failure_detail
        return {
            "schema_version": "bioxp.operator_action_receipt.v2",
            "command_id": str(row.get("command_id") or "unknown"),
            "action_id": str(row.get("action_id") or "unknown"),
            "status": status,
            "terminal": status not in {"queued", "dispatched", "issued_pending", "interrupting"},
            "sequence": int(row.get("sequence") or row.get("stream_sequence") or 1),
            "method_id": row.get("method_id"),
            "ownership_generation": int(row.get("ownership_generation") or 0),
            "expected_board_epoch_by_board": {str(key): int(value) for key, value in dict(row.get("expected_board_epoch_by_board") or {}).items()},
            "state_version": max(1, int(row.get("state_version") or row.get("version") or 1)),
            "status_path": f"/operator/v2/actions/receipts/{row.get('command_id')}",
            "accepted_at": accepted,
            "queued_at": queued,
            "dispatched_at": dispatched,
            "finished_at": finished,
            "terminal_receipt_id": row.get("terminal_receipt_id"),
            "completion_class": row.get("completion_class"),
            "physical_effect_verified": bool(row.get("physical_effect_verified") is True),
            "error": error,
            "transport_exchanges": list(row.get("transport_exchanges") or []),
            "transport_retention_errors": list(row.get("transport_retention_errors") or []),
        }

    def _v2_y_axis(state: Mapping[str, Any]) -> dict[str, Any]:
        provider = state.get("serial206_initialization_provider")
        provider_map = dict(provider) if isinstance(provider, Mapping) else {}
        raw = provider_map.get("y_authority")
        if not isinstance(raw, Mapping):
            raw = {}
        if isinstance(raw.get("authority"), Mapping):
            raw = {**dict(raw.get("authority") or {}), **{key: value for key, value in raw.items() if key != "authority"}}
        now = time.time()
        return {
            "axis": "y", "board_id": 4, "motor_id": 0,
            "ownership_generation": int(raw.get("ownership_generation") or state.get("ownership_generation") or 0),
            "prior_board_epoch": raw.get("prior_board_epoch"), "active_board_epoch": raw.get("active_board_epoch"), "prepared_board_epoch": raw.get("prepared_board_epoch"),
            "lifecycle_state": str(raw.get("lifecycle_state") or raw.get("state") or "unprepared"),
            "reference_state": str(raw.get("reference_state") or "unreferenced"),
            "position_steps": raw.get("position_steps"), "position_reply_valid": bool(raw.get("position_reply_valid", False)), "position_status_code": raw.get("position_status_code"),
            "speed_steps_s": raw.get("speed_steps_s"), "speed_reply_valid": bool(raw.get("speed_reply_valid", False)), "speed_status_code": raw.get("speed_status_code"),
            "left_switch_raw": raw.get("left_switch_raw"), "left_switch_reply_valid": bool(raw.get("left_switch_reply_valid", False)), "left_switch_status_code": raw.get("left_switch_status_code"),
            "home_effective": raw.get("home_effective"), "profile_fingerprint": raw.get("profile_fingerprint"), "profile_readback_valid": bool(raw.get("profile_readback_valid", False)), "profile_mismatches": list(raw.get("profile_mismatches") or []),
            "active_command": None, "interrupt_epoch": int(raw.get("interrupt_epoch") or 0), "latest_compact_receipt": None, "last_discrepancy_steps": raw.get("last_discrepancy_steps"), "state_version": max(1, int(raw.get("state_version") or 1)), "updated_at": float(raw.get("updated_at") or now), "physical_position_verified": bool(raw.get("physical_position_verified", False)),
        }

    def _v2_dashboard(state: Mapping[str, Any], queue_projection: Mapping[str, Any] | None = None, rows: list[dict[str, Any]] | None = None) -> dict[str, Any]:
        queue_projection = queue_projection or {}
        rows = rows or []
        compact = [_v2_compact_receipt(row) for row in rows]
        active = [row for row in compact if not row["terminal"]]
        provider = state.get("serial206_initialization_provider")
        provider_map = dict(provider) if isinstance(provider, Mapping) else {}
        board = provider_map.get("board4_authority") if isinstance(provider_map.get("board4_authority"), Mapping) else {}
        if not board and isinstance(provider_map.get("y_authority"), Mapping):
            candidate = provider_map["y_authority"].get("board_authority")
            board = candidate if isinstance(candidate, Mapping) else {}
        now = time.time()
        board4 = {"state": str(board.get("state") or "unknown"), "prior_board_epoch": board.get("prior_board_epoch"), "active_board_epoch": board.get("active_board_epoch"), "transition_phase": str(board.get("transition_phase") or "unknown"), "transition_evidence": dict(board.get("transition_evidence") or {}), "member_motors": {str(key): int(value) for key, value in dict(board.get("member_motors") or {"y": 0, "z": 1, "gripper": 2}).items()}, "state_version": max(1, int(board.get("state_version") or 1)), "updated_at": float(board.get("updated_at") or now)}
        y_axis = _v2_y_axis(state)
        y_rows = [row for row in compact if str(row.get("action_id", "")).startswith("oem.y.")]
        y_axis["active_command"] = next((row for row in y_rows if not row["terminal"]), None)
        y_axis["latest_compact_receipt"] = y_rows[0] if y_rows else None
        return {"schema_version": "bioxp.operator_dashboard.v2", "generated_at": now, "ownership_generation": int(state.get("ownership_generation") or 0), "board4": board4, "y_axis": y_axis, "active_commands": active, "latest_receipts": compact[:100]}

    async def history_rows(limit: int, *, before_sequence: int | None = None) -> list[dict[str, Any]]:
        current = await asyncio.to_thread(store.list, limit, before_sequence=before_sequence)
        if len(current) >= limit:
            return current
        current_ids = {str(row.get("command_id")) for row in current}
        legacy = await asyncio.to_thread(
            legacy_command_store.list_commands,
            limit=limit - len(current),
            before_sequence=before_sequence,
            exclude_command_ids=current_ids,
        )
        return [*current, *legacy]

    @router.get("/v2/dashboard")
    async def operator_dashboard_v2() -> dict[str, Any]:
        state = machine_state()
        rows = await asyncio.to_thread(store.list, 25)
        return _v2_dashboard(state, {}, rows)

    @router.get("/v2/control-catalog")
    async def control_catalog_v2() -> dict[str, Any]:
        state = machine_state()
        dashboard = _v2_dashboard(state, {}, await asyncio.to_thread(store.list, 25))
        return {"schema_version": "bioxp.operator_control_catalog.v2", "dashboard": dashboard, "actions": [{"action_id": str(action["action_id"]), "request_schema_version": "bioxp.operator_interrupt_request.v1" if str(action["safety_class"]) == "stop" else "bioxp.operator_action_request.v2", "response_schema_version": "bioxp.operator_interrupt_receipt.v1" if str(action["safety_class"]) == "stop" else "bioxp.operator_action_receipt.v2", "interrupt": str(action["safety_class"]) == "stop", "enabled": bool(assessed_action(action, state).get("enabled")), "disabled_reason": assessed_action(action, state).get("disabled_reason")} for action in actions if str(action["action_id"]) in v2_canonical_action_ids]}

    @router.post("/v2/actions/{action_id}")
    async def invoke_action_v2(action_id: str, payload: OperatorActionRequestV2 | OperatorInterruptRequestV1) -> dict[str, Any]:
        if action_id in {"oem.x.stop", "oem.y.stop", "oem.z.stop", "oem.abort_all"}:
            if not isinstance(payload, OperatorInterruptRequestV1):
                raise HTTPException(status_code=422, detail={"error": "interrupt_request_schema_required"})
            direct_receipt = await invoke_action(action_id, payload)
            return _v2_compact_receipt(direct_receipt)
        action = by_id.get(action_id)
        if (
            isinstance(payload, OperatorActionRequestV2)
            and action is not None
            and action_id not in INTERRUPT_ACTIONS
            and action_id in v2_canonical_action_ids
        ):
            direct_payload = InvokeRequest(
                expected_generation=int(payload.expected_ownership_generation),
                idempotency_key=payload.idempotency_key,
                inputs=dict(payload.inputs),
            )
            direct_receipt = await invoke_action(action_id, direct_payload)
            if str(direct_receipt.get("status") or "") in {"failed", "blocked", "rejected", "outcome_unknown", "ambiguous"}:
                detailed_receipt = await asyncio.to_thread(
                    store.by_command,
                    str(direct_receipt.get("command_id") or ""),
                    include_evidence=True,
                )
                if detailed_receipt is not None:
                    direct_receipt = detailed_receipt
            return _v2_compact_receipt(direct_receipt)
        raise HTTPException(status_code=404, detail="unknown v2 operator action_id")

    @router.get("/v2/actions/history")
    async def action_history_v2(limit: int = Query(default=100, ge=1, le=200), cursor: str | None = None) -> dict[str, Any]:
        before = None
        if cursor is not None:
            if not cursor.isdecimal() or int(cursor) < 1:
                raise HTTPException(status_code=422, detail={"error": "invalid_history_cursor"})
            before = int(cursor)
        rows = await history_rows(limit, before_sequence=before)
        items = [_v2_compact_receipt(row) for row in rows]
        next_cursor = str(items[-1]["sequence"]) if len(items) == limit else None
        return {"schema_version": "bioxp.operator_action_history.v2", "items": items, "next_cursor": next_cursor, "limit": limit}

    @router.get("/v2/actions/receipts/{command_id}")
    async def action_receipt_v2(command_id: str, detail: bool = False) -> dict[str, Any]:
        row = await asyncio.to_thread(
            store.by_command,
            command_id,
            include_evidence=detail,
        )
        if (
            row is not None
            and not detail
            and str(row.get("status") or "") in {"failed", "blocked", "rejected", "outcome_unknown", "ambiguous"}
        ):
            detailed_row = await asyncio.to_thread(
                store.by_command,
                command_id,
                include_evidence=True,
            )
            if detailed_row is not None:
                row = detailed_row
        if row is None:
            row = await asyncio.to_thread(
                legacy_command_store.command_detail_v2 if detail else legacy_command_store.get_command,
                command_id,
            )
        if row is None:
            raise HTTPException(status_code=404, detail="operator action receipt not found")
        compact = _v2_compact_receipt(row)
        if not detail:
            return compact
        return {**compact, "canonical_inputs": dict(row.get("canonical_inputs") or {}), "requested_values": dict(row.get("requested_values") or {}), "effective_values": dict(row.get("effective_values") or {}), "observed_values": dict(row.get("observed_values") or {}), "raw_return_layers": dict(row.get("raw_return_layers") or {}), "controller_evidence": dict(row.get("controller_evidence") or {}), "transport_artifacts": list(row.get("transport_artifacts") or []), "child_receipts": list(row.get("child_receipts") or []), "transitions": list(row.get("transitions") or [])}

    async def _v2_method_receipt(method: Mapping[str, Any]) -> dict[str, Any]:
        raw_status = str(method.get("status") or "queued")
        status = {"running": "active", "cancelled": "cleared", "stopped": "interrupted", "aborted": "interrupted", "recovery_required": "ambiguous"}.get(raw_status, raw_status)
        children = await asyncio.to_thread(legacy_command_store.list_method_commands, str(method["method_id"]))
        terminal = status in {"completed", "completed_partial", "failed", "cleared", "interrupted", "ambiguous"}
        accepted_at = float(method.get("queued_at") or time.time())
        return {
            "schema_version": "bioxp.operator_method.v1",
            "method_id": str(method["method_id"]),
            "action_id": str(method.get("name") or ""),
            "status": status,
            "state_version": max(1, int(method.get("version") or 1)),
            "child_receipts": [_v2_compact_receipt(row) for row in children],
            "accepted_at": accepted_at,
            "finished_at": float(method.get("updated_at") or accepted_at) if terminal else None,
        }

    @router.get("/v2/methods/{method_id}")
    async def method_status_v2(method_id: str) -> dict[str, Any]:
        method = await asyncio.to_thread(legacy_command_store.get_method, method_id)
        if method is None:
            raise HTTPException(status_code=404, detail="operator method not found")
        return await _v2_method_receipt(method)

    @router.get("/v2/commands/{command_id}")
    async def command_status_v2(command_id: str, detail: bool = True) -> dict[str, Any]:
        return await action_receipt_v2(command_id, detail=detail)

    @router.get("/control-catalog")
    async def control_catalog(schema_version: str | None = Query(default=None)) -> dict[str, Any]:
        state = machine_state()
        if schema_version == "bioxp.operator_control_catalog.v2":
            dashboard = _v2_dashboard(
                state,
                {},
                await asyncio.to_thread(store.list, 100),
            )
            return {
                "schema_version": "bioxp.operator_control_catalog.v2",
                "dashboard": dashboard,
                "actions": [
                    {
                        "action_id": str(action["action_id"]),
                        "request_schema_version": "bioxp.operator_interrupt_request.v1" if str(action["safety_class"]) == "stop" else "bioxp.operator_action_request.v2",
                        "response_schema_version": "bioxp.operator_interrupt_receipt.v1" if str(action["safety_class"]) == "stop" else "bioxp.operator_action_receipt.v2",
                        "interrupt": str(action["safety_class"]) == "stop",
                        "enabled": bool(assessed_action(action, state).get("enabled")),
                        "disabled_reason": assessed_action(action, state).get("disabled_reason"),
                    }
                    for action in actions
                    if str(action["action_id"]) in v2_canonical_action_ids
                ],
            }
        return {
            "schema_name": "bioxp.operator_control_catalog",
            "schema_version": CATALOG_SCHEMA,
            "machine_serial": str(OEM_MACHINE_SERIAL),
            "ownership_generation": int(hardware_state.ownership_epoch),
            **authority(),
            "dashboard": _dashboard_payload(state),
            "actions": [assessed_action(action, state) for action in actions],
        }

    @router.get("/dashboard")
    async def operator_dashboard(schema_version: str | None = Query(default=None)) -> dict[str, Any]:
        if schema_version == "bioxp.operator_dashboard.v2":
            state = machine_state()
            rows = await asyncio.to_thread(store.list, 100)
            return _v2_dashboard(state, {}, rows)
        return _dashboard_payload(machine_state())

    @router.post("/actions/{action_id}/admission")
    async def action_admission(action_id: str, payload: AdmissionRequest) -> dict[str, Any]:
        if not _ACTION_RE.fullmatch(action_id) or action_id not in by_id:
            raise HTTPException(status_code=404, detail="unknown operator action_id")
        state = machine_state()
        if payload.expected_generation != int(state["ownership_generation"]):
            raise HTTPException(status_code=409, detail="ownership generation mismatch")
        target = dispatch.get(action_id, {})
        effective_inputs = {**dict(target.get("fixed_inputs") or {}), **dict(payload.inputs)}
        return {"action_id": action_id, "ownership_generation": state["ownership_generation"], **_assess_action(by_id[action_id], state, effective_inputs)}

    @router.get("/actions/history")
    async def action_history(
        limit: int = Query(default=100, ge=1, le=200),
        schema_version: str | None = Query(default=None),
        cursor: str | None = Query(default=None),
    ) -> dict[str, Any]:
        if schema_version == "bioxp.operator_action_history.v2":
            before = None
            if cursor is not None:
                if not cursor.isdecimal() or int(cursor) < 1:
                    raise HTTPException(status_code=422, detail={"error": "invalid_history_cursor"})
                before = int(cursor)
            rows = await history_rows(limit, before_sequence=before)
            items = [_v2_compact_receipt(row) for row in rows]
            next_cursor = str(items[-1]["sequence"]) if len(items) == limit else None
            return {"schema_version": "bioxp.operator_action_history.v2", "items": items, "next_cursor": next_cursor, "limit": limit}
        rows = await history_rows(limit)
        return {"schema_version": HISTORY_SCHEMA, "receipts": rows}

    @router.get("/actions/receipts/{command_id}")
    async def action_receipt(command_id: str, detail: bool = False) -> dict[str, Any]:
        row = await asyncio.to_thread(
            store.by_command,
            command_id,
            include_evidence=detail,
        )
        if row is not None:
            return row
        command_row = await asyncio.to_thread(
            legacy_command_store.command_detail_v2 if detail else legacy_command_store.get_command,
            command_id,
        )
        if command_row is None:
            raise HTTPException(status_code=404, detail="operator action receipt not found")
        return _v2_compact_receipt(command_row)

    @router.post("/actions/receipts/{command_id}/assessment")
    async def assess_action(command_id: str, payload: AssessmentRequest) -> dict[str, Any]:
        if payload.expected_generation != int(hardware_state.ownership_epoch):
            raise HTTPException(status_code=409, detail="ownership generation mismatch")
        if payload.verdict not in {"pass", "fail"}:
            raise HTTPException(status_code=422, detail="verdict must be pass or fail")
        if (payload.legal_hold is None) != (payload.actor is None):
            raise HTTPException(
                status_code=422,
                detail="legal_hold and actor must be supplied together",
            )
        if not _IDEMPOTENCY_RE.fullmatch(payload.idempotency_key):
            raise HTTPException(status_code=422, detail="invalid idempotency_key")
        row = await asyncio.to_thread(store.by_command, command_id)
        if row is None:
            raise HTTPException(status_code=404, detail="operator action receipt not found")
        if row.get("action_id") == "oem.z.manual_home":
            raise HTTPException(
                status_code=409,
                detail={
                    "error": "provider_owned_z_observation_required",
                    "replacement_action_id": "oem.z.observe",
                    "authority_receipt_id": row.get("authority_receipt_id") or command_id,
                },
            )
        try:
            return await asyncio.to_thread(
                store.assess,
                command_id,
                expected_generation=payload.expected_generation,
                verdict=payload.verdict,
                note=payload.note,
                idempotency_key=payload.idempotency_key,
                legal_hold=payload.legal_hold,
                actor=payload.actor,
            )
        except KeyError as exc:
            raise HTTPException(status_code=404, detail="operator action receipt not found") from exc
        except (ValueError, RuntimeError) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router.post("/actions/{action_id}")
    async def invoke_action(action_id: str, payload: InvokeRequest | OperatorActionRequestV2 | OperatorInterruptRequestV1) -> dict[str, Any]:
        if isinstance(payload, OperatorInterruptRequestV1):
            if action_id.startswith(("oem.z.", "oem.x.", "oem.y.")) or action_id == "oem.abort_all":
                expected_generation = (
                    int(payload.observed_ownership_generation)
                    if payload.observed_ownership_generation is not None
                    else int(hardware_state.ownership_epoch)
                )
                payload = InvokeRequest(
                    expected_generation=expected_generation,
                    idempotency_key=payload.idempotency_key,
                    inputs={},
                )
            else:
                raise HTTPException(status_code=404, detail="unknown X/Y/Z interrupt action_id")
        if isinstance(payload, OperatorActionRequestV2):
            if action_id.startswith(("oem.z.", "oem.x.", "oem.y.", "oem.xy.")):
                payload = InvokeRequest(
                    expected_generation=int(payload.expected_ownership_generation),
                    idempotency_key=payload.idempotency_key,
                    inputs=dict(payload.inputs),
                )
            else:
                raise HTTPException(status_code=404, detail="unknown normal v2 operator action_id")
        request_received_at = time.time()
        if not _IDEMPOTENCY_RE.fullmatch(payload.idempotency_key):
            raise HTTPException(status_code=422, detail="invalid idempotency_key")
        encoded_inputs = json.dumps(payload.inputs, default=str, separators=(",", ":")).encode()
        if len(encoded_inputs) > _MAX_INPUT_BYTES:
            raise HTTPException(status_code=413, detail="action inputs exceed bounded limit")
        action = by_id[action_id]
        existing = await asyncio.to_thread(
            store.by_idempotency,
            payload.idempotency_key,
            include_evidence=False,
        )
        if existing is not None:
            if existing.get("action_id") != action_id or existing.get(
                "requested_inputs", existing.get("inputs")
            ) != payload.inputs:
                raise HTTPException(
                    status_code=409,
                    detail="idempotency_key already bound to different action request",
                )
            if int(existing.get("ownership_generation", -1)) != payload.expected_generation:
                raise HTTPException(
                    status_code=409,
                    detail="idempotency receipt ownership generation mismatch",
                )
            verify_replay_source_identity(existing)
            return existing
        expected = int(hardware_state.ownership_epoch)
        if payload.expected_generation != expected:
            raise HTTPException(status_code=409, detail="ownership generation mismatch")
        is_safety_interrupt = action_id in {
            "oem.x.stop",
            "oem.y.stop",
            "oem.z.stop",
            "oem.abort_all",
        }
        if action_id not in dispatch:
            assessment = _assess_action(action, machine_state(), payload.inputs)
            raise HTTPException(status_code=409, detail={"error": "action_unavailable", "reason": assessment["disabled_reason"], "dependencies": assessment["dependencies"]})
        target = dispatch[action_id]
        is_safety_interrupt = is_safety_interrupt or (
            target.get("method") == "POST"
            and target.get("path") in {
                "/motion/diagnostics/stop",
                "/motion/oem/x/stop",
                "/motion/oem/x/abort",
                "/motion/oem/y/stop",
                "/motion/oem/z/stop",
            }
        )
        unknown_inputs = set(payload.inputs) - set(target["inputs"])
        if unknown_inputs:
            raise HTTPException(
                status_code=422,
                detail={"error": "unknown_action_inputs", "unknown": sorted(unknown_inputs)},
            )
        if action_id == "oem.z.move_steps":
            steps = payload.inputs.get("steps")
            if type(steps) is not int or not -(2**31) <= steps <= 2**31 - 1:
                raise HTTPException(
                    status_code=422,
                    detail={
                        "error": "invalid_z_relative_steps",
                        "required": "signed int32; OEM moveSteps applies its own 20-step inner-limit predicate",
                    },
                )
        elif action_id == "oem.z.move_absolute":
            position = payload.inputs.get("position_steps")
            if type(position) is not int or not -(2**31) <= position <= 2**31 - 1:
                raise HTTPException(
                    status_code=422,
                    detail={
                        "error": "invalid_z_absolute_position",
                        "required": "signed int32; OEM moveZ applies its own pseudo-home and axis-limit clamps",
                    },
                )
        elif action_id == "oem.z.observe":
            boolean_fields = (
                "physical_motion_observed",
                "expected_direction_observed",
                "home_endpoint_observed",
                "stopped_observed",
            )
            invalid_boolean_fields = [
                name for name in boolean_fields if type(payload.inputs.get(name)) is not bool
            ]
            command_id = payload.inputs.get("command_id")
            verdict = payload.inputs.get("verdict")
            note = payload.inputs.get("note")
            if (
                invalid_boolean_fields
                or not isinstance(command_id, str)
                or not command_id.strip()
                or verdict not in {"pass", "fail"}
                or not isinstance(note, str)
                or len(note.strip()) < 3
            ):
                raise HTTPException(
                    status_code=422,
                    detail={
                        "error": "invalid_z_observation",
                        "invalid_boolean_fields": invalid_boolean_fields,
                        "required": "command_id, pass/fail verdict, four booleans, and a 3+ character note",
                    },
                )
        effective_inputs = {**dict(target.get("fixed_inputs") or {}), **dict(payload.inputs)}
        action_lock = interrupt_lock if is_safety_interrupt else invoke_lock
        async with action_lock:
            lock_acquired_at = time.time()
            locked_state = None if is_safety_interrupt else machine_state()
            locked_expected = (
                int(hardware_state.ownership_epoch)
                if locked_state is None
                else int(locked_state["ownership_generation"])
            )
            if not is_safety_interrupt and payload.expected_generation != locked_expected:
                raise HTTPException(status_code=409, detail="ownership generation mismatch")
            effective_inputs = {**dict(target.get("fixed_inputs") or {}), **dict(payload.inputs)}
            current_authority_fingerprint = replay_authority_fingerprint(locked_state or {})
            existing = None
            if not is_safety_interrupt:
                existing = await asyncio.to_thread(
                    store.by_idempotency,
                    payload.idempotency_key,
                    include_evidence=False,
                )
            if existing is not None:
                if existing.get("action_id") != action_id or existing.get("requested_inputs", existing.get("inputs")) != payload.inputs:
                    raise HTTPException(status_code=409, detail="idempotency_key already bound to different action request")
                if int(existing.get("ownership_generation", -1)) != locked_expected:
                    raise HTTPException(status_code=409, detail="idempotency receipt ownership generation mismatch")
                replay_assessment = _assess_action(action, locked_state or {}, effective_inputs)
                if not replay_assessment["enabled"]:
                    raise HTTPException(
                        status_code=409,
                        detail={
                            "error": "action_unavailable",
                            "reason": replay_assessment["disabled_reason"],
                            "dependencies": replay_assessment["dependencies"],
                        },
                    )
                if existing.get("authority_fingerprint") != current_authority_fingerprint:
                    raise HTTPException(status_code=409, detail="idempotency replay current authority mismatch")
                verify_replay_source_identity(existing)
                return existing
            command_id = f"operator_{int(time.time() * 1000)}_{uuid.uuid4().hex[:12]}"
            started = time.time()
            receipt = {
                "schema_version": RECEIPT_SCHEMA,
                "command_id": command_id,
                "action_id": action_id,
                "kind": action["kind"],
                "safety_class": action["safety_class"],
                "status": "admission_pending",
                "idempotency_key": payload.idempotency_key,
                "idempotency_replay_enabled": not is_safety_interrupt,
                "ownership_generation": locked_expected,
                "authority_fingerprint": current_authority_fingerprint,
                "source_identity": replay_source_identity(),
                "started_at": str(started),
                "request_received_at": request_received_at,
                "lock_acquired_at": lock_acquired_at,
                "admission_completed_at": None,
                "provider_entry_at": None,
                "provider_returned_at": None,
                "finished_at": None,
                "duration_ms": None,
                "remote_acknowledged": False,
                "controller_acknowledged": False,
                "physical_effect_verified": False,
                "machine_assessment": "unverified",
                "operator_assessment": None,
                "operator_note": None,
                "requested_inputs": _bounded_json(payload.inputs, _MAX_INPUT_BYTES),
                "inputs": _bounded_json(effective_inputs, _MAX_INPUT_BYTES),
                "response": None,
                "error": None,
                "stage_receipts": [],
            }
            claim_expected_status: str | None = None
            if not is_safety_interrupt:
                claimed, created = await asyncio.to_thread(store.claim, receipt)
                if not created:
                    if claimed.get("action_id") != action_id or claimed.get(
                        "requested_inputs", claimed.get("inputs")
                    ) != payload.inputs:
                        raise HTTPException(
                            status_code=409,
                            detail="idempotency_key already bound to different action request",
                        )
                    if int(claimed.get("ownership_generation", -1)) != locked_expected:
                        raise HTTPException(
                            status_code=409,
                            detail="idempotency receipt ownership generation mismatch",
                        )
                    if claimed.get("authority_fingerprint") != current_authority_fingerprint:
                        raise HTTPException(
                            status_code=409,
                            detail="idempotency replay current authority mismatch",
                        )
                    verify_replay_source_identity(claimed)
                    return claimed
                claim_expected_status = str(claimed["status"])
            assessment = (
                {"enabled": True, "disabled_reason": None, "dependencies": []}
                if is_safety_interrupt
                else _assess_action(action, locked_state or {}, effective_inputs)
            )
            if not assessment["enabled"]:
                detail = {
                    "error": "action_unavailable",
                    "reason": assessment["disabled_reason"],
                    "dependencies": assessment["dependencies"],
                }
                finished = time.time()
                receipt.update({
                    "status": "rejected",
                    "finished_at": str(finished),
                    "duration_ms": (finished - started) * 1000.0,
                    "machine_assessment": "fail",
                    "response": _bounded_json({"http_status": 409, "body": {"detail": detail}}, _MAX_RESPONSE_BYTES),
                    "error": "operator admission returned HTTP 409",
                })
                if not is_safety_interrupt:
                    await asyncio.to_thread(
                        store.put,
                        receipt,
                        _expected_status=claim_expected_status,
                    )
                raise HTTPException(status_code=409, detail=detail)
            receipt["status"] = "queued"
            receipt["queued_at"] = time.time()
            if not is_safety_interrupt:
                queued = await asyncio.to_thread(
                    store.put,
                    receipt,
                    _expected_status=claim_expected_status,
                )
                claim_expected_status = str(queued["status"])
            receipt["admission_completed_at"] = time.time()
            target = dispatch[action_id]
            wire_inputs = {
                name: value
                for name, value in effective_inputs.items()
                if name in target["locations"]
            }
            def retain_exchanges(evidence: dict[str, Any]) -> None:
                store.merge_transport_evidence(command_id, evidence)

            exchange_context = (
                nullcontext(None) if is_safety_interrupt else exchange_scope(
                    command_id,
                    sink=retain_exchanges,
                )
            )
            exchange_owner = exchange_context.__enter__()
            context_token = _DISPATCH_CONTEXT.set({
                "operator_command_id": command_id,
                "idempotency_key": payload.idempotency_key,
                "expected_ownership_generation": payload.expected_generation,
                "action_id": action_id,
            })
            linked_pipette_finalization = None
            try:
                receipt["provider_entry_at"] = time.time()
                receipt["status"] = "dispatched"
                receipt["dispatched_at"] = receipt["provider_entry_at"]
                if action_id == "meta.activate_motion":
                    # OEM preparation is one synchronous controller transaction.
                    # A disconnected caller must not abandon it after the durable
                    # command reservation or release the action lock while its
                    # blocking controller work continues.
                    provider_task = asyncio.create_task(
                        _dispatch_asgi(
                            app,
                            target["method"],
                            target["path"],
                            wire_inputs,
                            target["locations"],
                        ),
                        name=f"operator-{command_id}-activate-motion",
                    )
                    try:
                        status_code, response = await asyncio.shield(provider_task)
                    except asyncio.CancelledError:
                        status_code, response = await provider_task
                else:
                    status_code, response = await asyncio.wait_for(
                        _dispatch_asgi(app, target["method"], target["path"], wire_inputs, target["locations"]),
                        timeout=float(action["timeout_seconds"]),
                    )
                receipt["provider_returned_at"] = time.time()
                if isinstance(response, dict):
                    candidate = response.pop(_LINKED_FINALIZATION_KEY, None)
                    detail_value = response.get("detail")
                    if candidate is None and isinstance(detail_value, dict):
                        candidate = detail_value.pop(_LINKED_FINALIZATION_KEY, None)
                    if isinstance(candidate, Mapping):
                        linked_pipette_finalization = dict(candidate)
                full_response = {"http_status": status_code, "body": response}
                ok = 200 <= status_code < 300 and not (isinstance(response, dict) and response.get("ok") is False)
                authority_receipt = None
                observation_receipt = None
                pipette_truth = None
                completion_ambiguous = False
                if isinstance(response, dict):
                    receipt_source = response
                    detail = response.get("detail")
                    if isinstance(detail, Mapping):
                        receipt_source = detail
                    completion_ambiguous = bool(
                        receipt_source.get("completion_ambiguous") is True
                        or receipt_source.get("outcome_unknown") is True
                        or receipt_source.get("error") == "tester_operation_completion_ambiguous"
                    )
                    authority_receipt = receipt_source.get("authority_receipt")
                    observation_receipt = receipt_source.get("observation_receipt")
                    pipette_truth = response.get("receipt_truth")
                authority_controller_acknowledged = (
                    authority_receipt.get("controller_command_acknowledged")
                    if (
                        isinstance(authority_receipt, Mapping)
                        and type(authority_receipt.get("controller_command_acknowledged")) is bool
                    )
                    else None
                )
                linked_status = None
                if isinstance(pipette_truth, Mapping):
                    linked_status = (
                        "observed"
                        if pipette_truth.get("semantic_query_response_verified") is True
                        else "completed"
                        if pipette_truth.get("completion_verified") is True
                        else "acknowledged"
                        if pipette_truth.get("controller_acknowledged") is True
                        else "dispatched"
                        if pipette_truth.get("delivery_verified") is True
                        else "failed"
                    )
                receipt.update({
                    "status": (
                        "outcome_unknown"
                        if completion_ambiguous
                        else linked_status
                        if linked_status is not None
                        else "completed"
                        if ok
                        else "failed"
                    ),
                    "remote_acknowledged": 200 <= status_code < 300,
                    "delivery_verified": bool(
                        isinstance(pipette_truth, Mapping)
                        and pipette_truth.get("delivery_verified") is True
                    ) or bool(
                        isinstance(response, Mapping)
                        and response.get("delivery_verified") is True
                    ),
                    "controller_acknowledged": (
                        pipette_truth.get("controller_acknowledged")
                        if (
                            isinstance(pipette_truth, Mapping)
                            and type(pipette_truth.get("controller_acknowledged")) is bool
                        )
                        else authority_controller_acknowledged
                        if type(authority_controller_acknowledged) is bool
                        else _controller_acknowledged(response)
                    ),
                    "completion_verified": bool(
                        isinstance(pipette_truth, Mapping)
                        and pipette_truth.get("completion_verified") is True
                    ) or bool(
                        isinstance(response, Mapping)
                        and response.get("completion_verified") is True
                    ),
                    "hardware_precondition_verified": bool(
                        isinstance(pipette_truth, Mapping)
                        and pipette_truth.get("hardware_precondition_verified") is True
                    ) or bool(
                        isinstance(response, Mapping)
                        and response.get("hardware_precondition_verified") is True
                    ),
                    "hardware_postcondition_verified": bool(
                        isinstance(pipette_truth, Mapping)
                        and pipette_truth.get("hardware_postcondition_verified") is True
                    ) or bool(
                        isinstance(response, Mapping)
                        and response.get("hardware_postcondition_verified") is True
                    ),
                    "physical_effect_verified": bool(
                        isinstance(response, Mapping) and response.get("physical_effect_verified") is True
                    ),
                    "machine_assessment": "unverified" if completion_ambiguous else "pass" if ok else "fail",
                    "response": full_response,
                    "error": (
                        "pipette outcome unknown; reconciliation required and retry forbidden"
                        if completion_ambiguous
                        else None if ok else f"robot route returned HTTP {status_code}"
                    ),
                    "completion_ambiguous": completion_ambiguous,
                    "reconciliation_required": completion_ambiguous,
                    "retry_forbidden": completion_ambiguous,
                    "authority_receipt_id": (
                        authority_receipt.get("command_id")
                        if isinstance(authority_receipt, Mapping) else None
                    ),
                    "authority_receipt_status": (
                        authority_receipt.get("status")
                        if isinstance(authority_receipt, Mapping) else None
                    ),
                    "observation_receipt_id": (
                        observation_receipt.get("command_id")
                        if isinstance(observation_receipt, Mapping) else None
                    ),
                    "observes_command_id": (
                        observation_receipt.get("observes_command_id")
                        if isinstance(observation_receipt, Mapping) else None
                    ),
                    "stage_receipts": [],
                })
            except asyncio.TimeoutError:
                receipt["provider_returned_at"] = time.time()
                receipt.update({
                    "status": "outcome_unknown",
                    "machine_assessment": "unverified",
                    "error": "operator action timed out; physical outcome is unknown",
                    "automatic_retry": False,
                    "physical_outcome": "ambiguous",
                    "completion_ambiguous": True,
                    "reconciliation_required": True,
                    "retry_forbidden": True,
                })
            except Exception as exc:
                receipt["provider_returned_at"] = time.time()
                receipt.update({"status": "failed", "machine_assessment": "fail", "error": f"{type(exc).__name__}: {exc}"[:2000]})
            finally:
                _DISPATCH_CONTEXT.reset(context_token)
                try:
                    if exchange_owner is not None:
                        receipt.update(exchange_owner.snapshot())
                finally:
                    exchange_context.__exit__(None, None, None)
            finished = time.time()
            receipt["finished_at"] = str(finished)
            receipt["duration_ms"] = (finished - started) * 1000.0
            receipt["receipt_persist_started_at"] = time.time()
            if is_safety_interrupt:
                persisted = await asyncio.to_thread(store.put_interrupt, receipt)
            else:
                persisted = await asyncio.to_thread(
                    store.put,
                    receipt,
                    _expected_status=claim_expected_status,
                )
            return persisted

    app.state.invoke_operator_action_v2 = invoke_action_v2
    app.include_router(router)
    app.openapi_schema = None
