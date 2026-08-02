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
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Callable, Mapping
from urllib.parse import urlencode

from fastapi import APIRouter, FastAPI, HTTPException
from pydantic import BaseModel, ConfigDict, Field, StrictInt
from starlette.types import Message, Scope

from .hardware_status import hardware_state
from .lifecycle_state import lifecycle_state
from .oem_full_lifecycle import (
    OemFullLifecycleError,
    current_authority_identity,
    current_registry_sha256,
)
from .oem_machine_bundle import OEM_MACHINE_SERIAL
from .oem_movement_ledger import OEM_INITIALIZE_MOTORS_STAGE_KEYS
from .oem_serial206_initialization import SERIAL206_INITIALIZE_MOTION_STAGE_SPECS

CATALOG_SCHEMA = "bioxp.operator_control_catalog.v1"
RECEIPT_SCHEMA = "bioxp.operator_action_receipt.v1"
HISTORY_SCHEMA = "bioxp.operator_action_history.v1"
_MAX_INPUT_BYTES = 65_536
_MAX_RESPONSE_BYTES = 131_072
_MAX_RECEIPTS = 512
_ACTION_RE = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,127}$")
_IDEMPOTENCY_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{7,127}$")


class InvokeRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    expected_generation: StrictInt = Field(ge=0)
    idempotency_key: str = Field(min_length=8, max_length=128)
    inputs: dict[str, Any] = Field(default_factory=dict)


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


class BoundedReceiptStore:
    def __init__(self, root: str | Path | None = None) -> None:
        base = Path(root or os.environ.get("BIOXP_OEM_RUNTIME_ROOT") or "/tmp/bioxp-oem-runtime")
        base.mkdir(parents=True, exist_ok=True)
        self.path = base / "operator_action_receipts.json"
        self.lock = threading.RLock()

    def _read(self) -> list[dict[str, Any]]:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return []
        rows: Any = payload.get("receipts") if isinstance(payload, dict) else None
        return [row for row in rows if isinstance(row, dict)][-_MAX_RECEIPTS:] if isinstance(rows, list) else []

    def _write(self, rows: list[dict[str, Any]]) -> None:
        payload = {"schema_version": HISTORY_SCHEMA, "receipts": rows[-_MAX_RECEIPTS:]}
        tmp = self.path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(payload, sort_keys=True, separators=(",", ":")), encoding="utf-8")
        tmp.replace(self.path)

    def list(self, limit: int = 100) -> list[dict[str, Any]]:
        with self.lock:
            return list(reversed(self._read()[-max(1, min(limit, 200)):]))

    def by_command(self, command_id: str) -> dict[str, Any] | None:
        with self.lock:
            return next((row for row in reversed(self._read()) if row.get("command_id") == command_id), None)

    def by_idempotency(self, key: str) -> dict[str, Any] | None:
        with self.lock:
            return next((row for row in reversed(self._read()) if row.get("idempotency_key") == key), None)

    def put(self, receipt: Mapping[str, Any]) -> dict[str, Any]:
        row = dict(receipt)
        command_id = row.get("command_id")
        with self.lock:
            rows = [item for item in self._read() if item.get("command_id") != command_id]
            rows.append(row)
            self._write(rows)
        return row


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
    if value_type == "json" and default is None:
        default = {} if raw_type == "object" else [] if raw_type == "array" else None
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


_LATCH_CAPABLE_INITIALIZATION_PATHS = {
    "/oem/startup/initialize_environment",
    "/oem/initial_check",
}

_NO_MOTION_PREPARATION_PATHS = {
    "/motion/oem/prepare_without_motion",
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
    return "home" in path and not any(token in path for token in ("move", "park", "position"))


def _required_reference_axes(action: Mapping[str, Any], inputs: Mapping[str, Any]) -> list[str]:
    if not _motor_motion_action(action) or _home_action(action):
        return []
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
}

_LOCAL_ONLY_PATH_PREFIXES = (
    "/maintenance/usb/",
)

_OPERATOR_SEMANTIC_QUARANTINE_PATHS = {
    "/motion/interlock/prepare": "Quarantined: this legacy route performs inferred latch/power writes and is not the source-grounded serial-206 preparation provider.",
    "/motion/power/enable": "Quarantined: this route uses unacknowledged reverse-engineered relay writes and is not proven equivalent to an OEM global 24 V On operation.",
    "/motion/power/diag": "Quarantined: this diagnostic can enter the same unverified power-enable sequence and lacks truthful aggregate acknowledgment/readback.",
    "/oem/runtime/emergency_stop": "Quarantined: this route records lifecycle emergency state but does not dispatch the OEM physical aggregate abort sequence.",
}

_CAN_BOOTSTRAP_PATHS = {
    "/hardware/snapshot/collect",
    "/motion/oem/prepare_without_motion",
}


def _motion_readiness(machine_state: Mapping[str, Any], required_axes: list[str]) -> dict[str, Any]:
    """One fail-closed predicate shared by motion admission and dashboard truth."""
    dependencies: list[dict[str, Any]] = []
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
        door.get("door_closed") is True
        and door.get("latch_closed") is True
        and isinstance(latch, Mapping)
        and latch.get("door_sensor") == 1
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


def _assess_action(action: Mapping[str, Any], machine_state: Mapping[str, Any], inputs: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Assess one action using only already-published machine state."""
    values = inputs or {}
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

    method = str(action.get("informational_method") or "GET")
    safety = str(action.get("safety_class") or "read_only")
    path = str(action.get("informational_path") or "").lower()
    source_initializer = path in {
        "/motion/oem/serial206/initialize_motors",
        "/motion/oem/serial206/initialize_motion",
    }
    requires_transport = (method != "GET" or safety in {"stop", "emergency"}) and path not in _TRANSPORT_BOOTSTRAP_PATHS
    ownership_value = machine_state.get("ownership")
    ownership: Mapping[str, Any] = ownership_value if isinstance(ownership_value, Mapping) else {}
    transport_live = bool(
        ownership.get("transport") == "owned"
        and ownership.get("usb") == "service"
        and ownership.get("router") == "running"
    )
    if requires_transport:
        dependencies.append(_dependency("transport_live", "Robot transport live", transport_live, "Robot transport is unavailable."))

    requires_can = (
        requires_transport
        and safety not in {"stop", "emergency"}
        and path not in _CAN_BOOTSTRAP_PATHS
        and not source_initializer
        and (safety == "motion" or _motor_motion_action(action))
    )
    if requires_can:
        dependencies.append(_dependency(
            "can_ready",
            "Same-epoch CAN ready",
            ownership.get("CAN_READY") is True,
            "Same-epoch CAN readiness has not been established.",
        ))

    if source_initializer:
        maintenance_value = machine_state.get("maintenance")
        maintenance = maintenance_value if isinstance(maintenance_value, Mapping) else {}
        motion_enabled = maintenance.get("motion_blocked") is False and maintenance.get("recovery_required") is False
        dependencies.append(_dependency(
            "motion_enabled", "Motion enabled", motion_enabled,
            "Motion is inactive. Activate motion before starting OEM initialization.",
        ))
    elif safety == "motion" or _motor_motion_action(action):
        readiness = _motion_readiness(machine_state, _required_reference_axes(action, values))
        existing_keys = {row["key"] for row in dependencies}
        dependencies.extend(row for row in readiness["dependencies"] if row["key"] not in existing_keys)

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
        status = row.get("status") if isinstance(row, Mapping) and isinstance(row.get("status"), Mapping) else {}
        switches = row.get("switch_activity") if isinstance(row, Mapping) and isinstance(row.get("switch_activity"), Mapping) else {}
        axes.append({
            "axis": str(axis),
            "reference": ((reference_rows.get(axis) or {}).get("state") if isinstance(reference_rows.get(axis), Mapping) else "unknown") or "unknown",
            "position_steps": _value(status, "position"),
            "speed_steps_s": _value(status, "speed"),
            "run_current": _value(status, "max_current"),
            "standby_current": _value(status, "standby_current"),
            "left_switch_active": switches.get("left_raw_active"),
            "right_switch_active": switches.get("right_raw_active"),
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
    freshness = machine_state.get("freshness") if isinstance(machine_state.get("freshness"), Mapping) else {"state": "missing", "age_s": None, "fresh_for_s": None}
    connection_live = bool(ownership.get("transport") == "owned" and ownership.get("usb") == "service" and ownership.get("router") == "running")
    motion_readiness = _motion_readiness(machine_state, ["x", "y", "z", "g", "door"])
    return {
        "schema_version": "bioxp.operator_dashboard.v1",
        "ownership_generation": int(machine_state.get("ownership_generation") or 0),
        "connection": {"live": connection_live, "ownership": dict(ownership)},
        "motion": {"enabled": motion_readiness["enabled"], "reason": motion_readiness["disabled_reason"]},
        "operation": {"state": lifecycle.get("operation_state"), "reason": lifecycle.get("operation_reason")},
        "enclosure": {"door_closed": enclosure.get("door_closed"), "latch_closed": enclosure.get("latch_closed")},
        "axes": axes,
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
}
_SERIAL206_PROVIDER_CAPABILITIES = {
    "/motion/oem/serial206/initialize_motors": "initialize_motors",
    "/motion/oem/serial206/initialize_motion": "initialize_motion",
}


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
    prepare_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/oem/prepare_without_motion" and row["informational_method"] == "POST"),
        None,
    )
    emergency_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/emergency_stop" and row["informational_method"] == "POST"),
        None,
    )
    home_xy_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/oem/home_xy" and row["informational_method"] == "POST"),
        None,
    )
    initialize_motors_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/oem/serial206/initialize_motors" and row["informational_method"] == "POST"),
        None,
    )
    initialize_motion_provider = next(
        (row for row in actions if row["informational_path"] == "/motion/oem/serial206/initialize_motion" and row["informational_method"] == "POST"),
        None,
    )
    prepare_bound = prepare_provider is not None and prepare_provider.get("provider_available") is True
    emergency_bound = emergency_provider is not None and emergency_provider.get("provider_available") is True
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
            "description": "Activate the serial-206 motor power path using the OEM board-activation and initializeMotorsWithoutMotion sequence, then verify 24 V/door/latch readbacks. This does not home or move an axis.",
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
            "stages": ["serial-206 authority", "activate boards 4/5/6", "resolve board 7 as non-motor", "initializeMotorsWithoutMotion", "exact parameter readback", "24 V query", "door query", "latch query"],
        },
        {
            "action_id": "meta.emergency_stop",
            "label": "Physical Aggregate Emergency Stop",
            "subsystem": "meta",
            "category": "emergency",
            "kind": "meta",
            "safety_class": "emergency",
            "description": "Interrupt path that sends ClassMotor StopMotor to every serial-206 movement component and verifies exact ACK plus terminal zero speed.",
            "source_anchor": "ClassMotor.StopMotor:161+; ControlLib.forceAbortMotion:10564-10606; ClassControlInterface.forceAbortMotion:5095-5104",
            "informational_method": "POST",
            "informational_path": "/motion/emergency_stop",
            "provider_available": emergency_bound,
            "provider_unavailable_reason": None if emergency_bound else "Physical aggregate stop provider is not bound.",
            "available": emergency_bound,
            "unavailable_reason": None if emergency_bound else "Physical aggregate stop provider is not bound.",
            "enabled": emergency_bound,
            "disabled_reason": None if emergency_bound else "Physical aggregate stop provider is not bound.",
            "dependencies": [],
            "requires_confirmation": False,
            "timeout_seconds": 30.0,
            "inputs": list(emergency_provider.get("inputs", [])) if emergency_provider else [],
            "stages": ["stop x", "stop y", "stop z", "stop gripper", "stop thermal door", "terminal speed readback"],
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
            "informational_path": "/motion/oem/serial206/initialize_motors",
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
            "informational_path": "/motion/oem/serial206/initialize_motion",
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
    emergency_route = next((row for row in dispatch.values() if row["method"] == "POST" and row["path"] == "/motion/emergency_stop"), None)
    initialize_motors_route = next((row for row in dispatch.values() if row["method"] == "POST" and row["path"] == "/motion/oem/serial206/initialize_motors"), None)
    initialize_motion_route = next((row for row in dispatch.values() if row["method"] == "POST" and row["path"] == "/motion/oem/serial206/initialize_motion"), None)
    if home_route:
        dispatch["meta.home_xy"] = home_route
    if prepare_route:
        dispatch["meta.activate_motion"] = prepare_route
    if emergency_route:
        dispatch["meta.emergency_stop"] = emergency_route
    if initialize_motors_route:
        dispatch["meta.initialize_motors"] = initialize_motors_route
    if initialize_motion_route:
        dispatch["meta.initialize_motion"] = initialize_motion_route
    actions.extend(meta)
    return actions, dispatch


async def _dispatch_asgi(app: FastAPI, method: str, path_template: str, inputs: dict[str, Any], locations: Mapping[str, Mapping[str, Any]]) -> tuple[int, Any]:
    path = path_template
    query: dict[str, Any] = {}
    body: dict[str, Any] = {}
    allowed = set(locations)
    if set(inputs) - allowed:
        raise HTTPException(status_code=422, detail={"error": "unknown_action_inputs", "unknown": sorted(set(inputs) - allowed)})
    for name, metadata in locations.items():
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
    if len(raw) > _MAX_RESPONSE_BYTES:
        return status, _bounded_json({"raw": raw.decode("utf-8", "replace")}, _MAX_RESPONSE_BYTES)
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
) -> None:
    """Snapshot final routes and mount the robot-authoritative operator plane."""
    actions, dispatch = _build_catalog(app)
    by_id = {row["action_id"]: row for row in actions}
    store = BoundedReceiptStore()
    invoke_lock = asyncio.Lock()
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

    @router.get("/control-catalog")
    async def control_catalog() -> dict[str, Any]:
        state = machine_state()
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
    async def operator_dashboard() -> dict[str, Any]:
        return _dashboard_payload(machine_state())

    @router.post("/actions/{action_id}/admission")
    async def action_admission(action_id: str, payload: AdmissionRequest) -> dict[str, Any]:
        if not _ACTION_RE.fullmatch(action_id) or action_id not in by_id:
            raise HTTPException(status_code=404, detail="unknown operator action_id")
        state = machine_state()
        if payload.expected_generation != int(state["ownership_generation"]):
            raise HTTPException(status_code=409, detail="ownership generation mismatch")
        return {"action_id": action_id, "ownership_generation": state["ownership_generation"], **_assess_action(by_id[action_id], state, payload.inputs)}

    @router.get("/actions/history")
    async def action_history(limit: int = 100) -> dict[str, Any]:
        return {"schema_version": HISTORY_SCHEMA, "receipts": store.list(limit)}

    @router.get("/actions/receipts/{command_id}")
    async def action_receipt(command_id: str) -> dict[str, Any]:
        row = store.by_command(command_id)
        if row is None:
            raise HTTPException(status_code=404, detail="operator action receipt not found")
        return row

    @router.post("/actions/receipts/{command_id}/assessment")
    async def assess_action(command_id: str, payload: AssessmentRequest) -> dict[str, Any]:
        if payload.expected_generation != int(hardware_state.ownership_epoch):
            raise HTTPException(status_code=409, detail="ownership generation mismatch")
        if payload.verdict not in {"pass", "fail"}:
            raise HTTPException(status_code=422, detail="verdict must be pass or fail")
        if not _IDEMPOTENCY_RE.fullmatch(payload.idempotency_key):
            raise HTTPException(status_code=422, detail="invalid idempotency_key")
        row = store.by_command(command_id)
        if row is None:
            raise HTTPException(status_code=404, detail="operator action receipt not found")
        row = dict(row)
        row["operator_assessment"] = payload.verdict
        row["operator_note"] = payload.note.strip()
        row["operator_assessment_idempotency_key"] = payload.idempotency_key
        row["operator_assessed_at"] = time.time()
        return store.put(row)

    @router.post("/actions/{action_id}")
    async def invoke_action(action_id: str, payload: InvokeRequest) -> dict[str, Any]:
        if not _ACTION_RE.fullmatch(action_id) or action_id not in by_id:
            raise HTTPException(status_code=404, detail="unknown operator action_id")
        if not _IDEMPOTENCY_RE.fullmatch(payload.idempotency_key):
            raise HTTPException(status_code=422, detail="invalid idempotency_key")
        expected = int(hardware_state.ownership_epoch)
        if payload.expected_generation != expected:
            raise HTTPException(status_code=409, detail="ownership generation mismatch")
        encoded_inputs = json.dumps(payload.inputs, default=str, separators=(",", ":")).encode()
        if len(encoded_inputs) > _MAX_INPUT_BYTES:
            raise HTTPException(status_code=413, detail="action inputs exceed bounded limit")
        action = by_id[action_id]
        if action_id not in dispatch:
            assessment = _assess_action(action, machine_state(), payload.inputs)
            raise HTTPException(status_code=409, detail={"error": "action_unavailable", "reason": assessment["disabled_reason"], "dependencies": assessment["dependencies"]})
        target = dispatch[action_id]
        unknown_inputs = set(payload.inputs) - set(target["inputs"])
        if unknown_inputs:
            raise HTTPException(
                status_code=422,
                detail={"error": "unknown_action_inputs", "unknown": sorted(unknown_inputs)},
            )
        async with invoke_lock:
            locked_state = machine_state()
            locked_expected = int(locked_state["ownership_generation"])
            if payload.expected_generation != locked_expected:
                raise HTTPException(status_code=409, detail="ownership generation mismatch")
            existing = store.by_idempotency(payload.idempotency_key)
            if existing is not None:
                if existing.get("action_id") != action_id or existing.get("inputs") != payload.inputs:
                    raise HTTPException(status_code=409, detail="idempotency_key already bound to different action request")
                if int(existing.get("ownership_generation", -1)) != locked_expected:
                    raise HTTPException(status_code=409, detail="idempotency receipt ownership generation mismatch")
                return existing
            assessment = _assess_action(action, locked_state, payload.inputs)
            if not assessment["enabled"]:
                raise HTTPException(status_code=409, detail={"error": "action_unavailable", "reason": assessment["disabled_reason"], "dependencies": assessment["dependencies"]})
            command_id = f"operator_{int(time.time() * 1000)}_{uuid.uuid4().hex[:12]}"
            started = time.time()
            receipt = {
                "schema_version": RECEIPT_SCHEMA,
                "command_id": command_id,
                "action_id": action_id,
                "kind": action["kind"],
                "safety_class": action["safety_class"],
                "status": "queued",
                "idempotency_key": payload.idempotency_key,
                "ownership_generation": locked_expected,
                "started_at": str(started),
                "finished_at": None,
                "duration_ms": None,
                "remote_acknowledged": False,
                "physical_effect_verified": False,
                "machine_assessment": "unverified",
                "operator_assessment": None,
                "operator_note": None,
                "inputs": _bounded_json(payload.inputs, _MAX_INPUT_BYTES),
                "response": None,
                "error": None,
                "stage_receipts": [],
            }
            store.put(receipt)
            target = dispatch[action_id]
            try:
                status_code, response = await asyncio.wait_for(
                    _dispatch_asgi(app, target["method"], target["path"], payload.inputs, target["locations"]),
                    timeout=float(action["timeout_seconds"]),
                )
                bounded = _bounded_json({"http_status": status_code, "body": response}, _MAX_RESPONSE_BYTES)
                ok = 200 <= status_code < 300 and not (isinstance(response, dict) and response.get("ok") is False)
                stages = []
                if isinstance(response, dict):
                    candidate = response.get("stage_receipts") or response.get("stages")
                    if isinstance(candidate, list):
                        stages = candidate[:128]
                    elif isinstance(candidate, dict):
                        stages = list(candidate.values())[:128]
                receipt.update({
                    "status": "completed" if ok else "failed",
                    "remote_acknowledged": 200 <= status_code < 300,
                    "machine_assessment": "pass" if ok else "fail",
                    "response": bounded,
                    "error": None if ok else f"robot route returned HTTP {status_code}",
                    "stage_receipts": _bounded_json(stages, _MAX_RESPONSE_BYTES),
                })
            except asyncio.TimeoutError:
                receipt.update({"status": "failed", "machine_assessment": "fail", "error": "operator action timed out"})
            except Exception as exc:
                receipt.update({"status": "failed", "machine_assessment": "fail", "error": f"{type(exc).__name__}: {exc}"[:2000]})
            finished = time.time()
            receipt["finished_at"] = str(finished)
            receipt["duration_ms"] = (finished - started) * 1000.0
            return store.put(receipt)

    app.include_router(router)
    app.openapi_schema = None
