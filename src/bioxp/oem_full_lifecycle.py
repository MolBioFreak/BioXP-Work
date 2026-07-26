"""Robot-owned full OEM startup/movement lifecycle planning and durable state.

This module implements the source-shaped application-to-terminal graph without a
transport dependency.  It is therefore safe for authority/configuration checks,
dry-run planning, restart/cancellation tests, and API projection.  Physical
providers are attached only after their individual commissioning gates pass.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Mapping

from .oem_machine_bundle import (
    OEM_ACQUISITION_ID,
    OEM_LOCK_SHA256,
    OEM_MACHINE_SERIAL,
)
from .oem_movement_ledger import OEM_INITIALIZE_MOTORS_STAGES
from .oem_runtime_store import OEMRuntimeStore
from .oem_runtime_types import new_id, utc_ts


FULL_LIFECYCLE_SCHEMA = "bioxp.oem_full_movement_lifecycle.v1"
FULL_LIFECYCLE_COMMAND = "initialize_oem_movement_lifecycle"
_REGISTRY_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json"
)


class OemFullLifecycleError(RuntimeError):
    pass


def current_registry_sha256() -> str:
    return hashlib.sha256(_REGISTRY_PATH.read_bytes()).hexdigest()


# The request carries only lifecycle predicates read by the robot.  It cannot
# carry an axis, board, position, speed, current, timeout, frame, or stage name.
_ALLOWED_INPUTS = frozenset(
    {
        "ownership_generation",
        "saved_status",
        "ship_mode",
        "start_mode",
        "tip_present",
        "self_test_due",
        "camera_required",
        "deck_inspection",
    }
)
_REQUIRED_INPUTS = _ALLOWED_INPUTS
_ALLOWED_REQUEST_KEYS = frozenset(
    {
        "command",
        "operator_ack",
        "expected_machine_serial",
        "expected_registry_sha256",
        "idempotency_key",
        "mode",
        "inputs",
    }
)


def _stage(
    stage_id: str,
    source_anchor: str,
    *,
    would_command_hardware: bool = False,
    would_command_physical_motion: bool = False,
    branch: str | None = None,
    movement_ledger_stage: str | None = None,
) -> dict[str, Any]:
    return {
        "stage_id": stage_id,
        "source_anchor": source_anchor,
        "branch": branch,
        "movement_ledger_stage": movement_ledger_stage,
        "status": "pending",
        "would_command_hardware": would_command_hardware or would_command_physical_motion,
        "would_command_physical_motion": would_command_physical_motion,
        "physical_motion_commanded": False,
        "controller_acknowledged": False,
        "postcondition_verified": False,
        "physical_effect_verified": False,
        "started_at": None,
        "completed_at": None,
        "blocked_reason": None,
    }


def _base_plan(inputs: Mapping[str, Any]) -> tuple[list[dict[str, Any]], str]:
    stages = [
        _stage("initialize_environment", "BioXPMainWindow.cs:973-1027"),
        _stage("enqueue_initialize_system", "BioXPMainWindow.cs:989-1003"),
        _stage("worker_claim_initialize_system", "BioXPMainWindow.cs:2030-2100"),
        _stage("initialize_system_reentry_guard", "BioXPMainWindow.cs:1046-1058"),
    ]

    if inputs["ship_mode"] == "PARK":
        stages.append(
            _stage(
                "ship_mode_park_shutdown_ready",
                "BioXPMainWindow.cs:1127-1134",
                would_command_physical_motion=True,
                branch="ship_mode_park",
            )
        )
        return stages, "oem_movement_blocked_ship_mode_shutdown_ready"

    stages.append(_stage("initialize_system_initial_check", "BioXPMainWindow.cs:1140-1144"))

    if inputs["saved_status"] in {3, 4}:
        stages.extend(
            [
                _stage(
                    "saved_status_initialize_motion",
                    "BioXPMainWindow.cs:1146-1148; ControlLib.cs:8797-8856",
                    would_command_physical_motion=True,
                    branch="saved_status_recovery",
                ),
                _stage(
                    "saved_status_inspect_cover",
                    "BioXPMainWindow.cs:1149; ControlLib.cs:3663-3768",
                    would_command_physical_motion=True,
                    branch="saved_status_recovery",
                ),
                _stage(
                    "saved_status_unlock_warning_return",
                    "BioXPMainWindow.cs:1146-1155",
                    would_command_physical_motion=True,
                    branch="saved_status_recovery",
                ),
            ]
        )
        return stages, "oem_movement_blocked_saved_status_recovery"

    stages.extend(
        [
            _stage(
                "configure_motors_without_motion",
                "ClassControlInterface.initializeMotorsWithoutMotion:3181-3265",
                would_command_hardware=True,
            ),
            _stage("initialize_motion_flags", "ControlLib.cs:8797-8804"),
        ]
    )
    for number, movement in enumerate(OEM_INITIALIZE_MOTORS_STAGES, start=1):
        movement_key = str(movement["key"])
        stages.append(
            _stage(
                f"initialize_motors_m{number:02d}_{movement_key.replace('-', '_')}",
                str(movement["source_anchor"]),
                would_command_hardware=movement_key not in {"ui-zero-calibrated", "system-status-initialized"},
                would_command_physical_motion=bool(movement.get("requires_operator_observation", True)),
                movement_ledger_stage=movement_key,
            )
        )
    stages.append(_stage("initialize_motion_tip_query", "ControlLib.cs:8806-8813", would_command_hardware=True))

    if inputs["tip_present"] is True:
        stages.extend(
            [
                _stage("tip_open_thermal_door", "ControlLib.cs:8814-8818", would_command_physical_motion=True, branch="stale_tip"),
                _stage("tip_route_park_to_waste", "ControlLib.cs:8819-8825", would_command_physical_motion=True, branch="stale_tip"),
                _stage("tip_eject_all", "ClassPipetteCollection.cs:1176-1323", would_command_physical_motion=True, branch="stale_tip"),
                _stage("tip_move_z_80000", "ControlLib.cs:8835-8836", would_command_physical_motion=True, branch="stale_tip"),
                _stage("tip_move_x_79000", "ControlLib.cs:8837-8838", would_command_physical_motion=True, branch="stale_tip"),
                _stage("tip_verify_empty", "ControlLib.cs:8839-8845; ClassPipetteCollection.cs:1336-1358", branch="stale_tip"),
                _stage("pipette_reinitialize_retry_once", "ControlLib.cs:8846-8856; ClassPipetteCollection.cs:677-748", branch="stale_tip"),
            ]
        )
    else:
        stages.append(_stage("initialize_motion_no_tip", "ControlLib.cs:8854-8856", branch="no_tip"))

    stages.append(_stage("self_test_gate", "BioXPMainWindow.cs:1163-1171"))
    if inputs["self_test_due"] is True:
        stages.extend(
            [
                _stage("self_test_tc_rc_oc", "ControlLib.cs:10688-10999", would_command_physical_motion=True, branch="self_test_due"),
                _stage("self_test_motion", "ControlLib.cs:10688-10785", would_command_physical_motion=True, branch="self_test_due"),
            ]
        )

    stages.append(_stage("camera_gate", "BioXPMainWindow.cs:1172-1180"))
    if inputs["camera_required"] is True:
        stages.append(
            _stage("camera_check", "ControlLib.cs:1929-1960", would_command_physical_motion=True, branch="camera_required")
        )

    if inputs["deck_inspection"] is True:
        stages.append(
            _stage("cover_inspection", "ControlLib.cs:3663-3768", would_command_physical_motion=True, branch="deck_inspection")
        )

    stages.append(_stage("gantry_park", "ControlLib.cs:7071-7122", would_command_physical_motion=True))

    mode = str(inputs["start_mode"])
    terminal_by_mode = {
        "DevMode": ("start_mode_manual_ready", "oem_movement_ready_manual", "BioXPMainWindow.cs:1203-1212"),
        "TradeShowMode": ("start_mode_trade_show_ready", "oem_movement_ready_trade_show", "BioXPMainWindow.cs:1213-1221"),
        "WebMode": ("start_mode_web_job_admission", "oem_movement_ready_job_admission", "BioXPMainWindow.cs:1222-1295"),
        "LocalMode": ("start_mode_local_job_admission", "oem_movement_ready_job_admission", "BioXPMainWindow.cs:1222-1295"),
    }
    try:
        terminal_stage, terminal_state, source = terminal_by_mode[mode]
    except KeyError as exc:
        raise OemFullLifecycleError(f"unsupported source StartMode {mode!r}") from exc
    stages.append(_stage(terminal_stage, source, would_command_physical_motion=True, branch=mode))
    return stages, terminal_state


class OemFullLifecycleRuns:
    """Durable full-lifecycle runs with no transport or auto-resume behavior."""

    def __init__(self, store: OEMRuntimeStore):
        self.store = store

    @staticmethod
    def _validate_request(request: Mapping[str, Any]) -> dict[str, Any]:
        unknown = set(request) - _ALLOWED_REQUEST_KEYS
        if unknown:
            raise OemFullLifecycleError(f"unknown lifecycle request field(s): {sorted(unknown)}")
        if request.get("command") != FULL_LIFECYCLE_COMMAND:
            raise OemFullLifecycleError(f"command must be {FULL_LIFECYCLE_COMMAND!r}")
        if request.get("operator_ack") != "INITIALIZE":
            raise OemFullLifecycleError("operator_ack INITIALIZE required")
        if type(request.get("expected_machine_serial")) is not int or request.get("expected_machine_serial") != OEM_MACHINE_SERIAL:
            raise OemFullLifecycleError(f"expected machine serial must be {OEM_MACHINE_SERIAL}")
        actual_registry = current_registry_sha256()
        if request.get("expected_registry_sha256") != actual_registry:
            raise OemFullLifecycleError("expected registry SHA-256 does not match frozen runtime registry")
        if request.get("mode") != "dry_run":
            raise OemFullLifecycleError("full lifecycle physical execution is blocked until commissioned providers are bound")
        key = request.get("idempotency_key")
        if not isinstance(key, str) or not key.strip():
            raise OemFullLifecycleError("nonblank idempotency_key required")
        inputs = request.get("inputs")
        if not isinstance(inputs, Mapping):
            raise OemFullLifecycleError("typed lifecycle inputs required")
        unknown_inputs = set(inputs) - _ALLOWED_INPUTS
        if unknown_inputs:
            raise OemFullLifecycleError(f"unknown lifecycle input(s): {sorted(unknown_inputs)}")
        missing_inputs = _REQUIRED_INPUTS - set(inputs)
        if missing_inputs:
            raise OemFullLifecycleError(f"missing lifecycle input(s): {sorted(missing_inputs)}")
        if type(inputs["ownership_generation"]) is not int or inputs["ownership_generation"] < 0:
            raise OemFullLifecycleError("ownership_generation must be a nonnegative integer")
        if type(inputs["saved_status"]) is not int:
            raise OemFullLifecycleError("saved_status must be an integer")
        if not isinstance(inputs["ship_mode"], str):
            raise OemFullLifecycleError("ship_mode must be a string")
        if not isinstance(inputs["start_mode"], str):
            raise OemFullLifecycleError("start_mode must be a string")
        for field in ("tip_present", "self_test_due", "camera_required", "deck_inspection"):
            if type(inputs[field]) is not bool:
                raise OemFullLifecycleError(f"{field} must be an exact boolean")
        return {**dict(request), "idempotency_key": key.strip(), "inputs": dict(inputs)}

    def create(self, request: Mapping[str, Any]) -> dict[str, Any]:
        req = self._validate_request(request)
        for existing in self.store.list_oem_full_lifecycle_runs():
            if existing.get("idempotency_key") == req["idempotency_key"]:
                if existing.get("request") != req:
                    raise OemFullLifecycleError("idempotency_key is already bound to a different request")
                return existing
        stages, planned_terminal = _base_plan(req["inputs"])
        run_id = new_id("oem_move")
        now = utc_ts()
        payload = {
            "schema_version": FULL_LIFECYCLE_SCHEMA,
            "run_id": run_id,
            "command": FULL_LIFECYCLE_COMMAND,
            "idempotency_key": req["idempotency_key"],
            "request": req,
            "run_state": "planned",
            "terminal_state": None,
            "planned_terminal_state": planned_terminal,
            "current_stage": None,
            "expected_next_stage": stages[0]["stage_id"] if stages else None,
            "blocked_reason": None,
            "source_authority_verified": True,
            "configuration_verified": True,
            "transport_owner_verified": False,
            "controller_acknowledged": False,
            "postcondition_verified": False,
            "physical_motion_commanded": False,
            "physical_effect_verified": False,
            "safety_deviation": [],
            "registry_sha256": current_registry_sha256(),
            "evidence_lock_sha256": OEM_LOCK_SHA256,
            "acquisition_id": OEM_ACQUISITION_ID,
            "machine_serial": OEM_MACHINE_SERIAL,
            "ownership_generation": req["inputs"]["ownership_generation"],
            "transport_frames": [],
            "stages": stages,
            "created_at": now,
            "updated_at": now,
        }
        return self.store.write_oem_full_lifecycle_run(payload)

    def get(self, run_id: str) -> dict[str, Any]:
        payload = self.store.read_oem_full_lifecycle_run(run_id)
        if payload is None:
            raise OemFullLifecycleError(f"full OEM lifecycle run {run_id!r} not found")
        return payload

    def execute_dry_run(self, run_id: str) -> dict[str, Any]:
        payload = self.get(run_id)
        if payload["request"]["mode"] != "dry_run":
            raise OemFullLifecycleError("only dry-run execution is available before physical commissioning")
        if payload["run_state"] != "planned":
            raise OemFullLifecycleError(f"run is not executable from state {payload['run_state']!r}")
        payload["run_state"] = "running"
        for index, row in enumerate(payload["stages"]):
            row["status"] = "running"
            row["started_at"] = utc_ts()
            payload["current_stage"] = row["stage_id"]
            payload["expected_next_stage"] = row["stage_id"]
            payload["updated_at"] = utc_ts()
            self.store.write_oem_full_lifecycle_run(payload)
            # Deliberately no provider or transport call here.
            row["status"] = "completed"
            row["completed_at"] = utc_ts()
            payload["expected_next_stage"] = (
                payload["stages"][index + 1]["stage_id"]
                if index + 1 < len(payload["stages"])
                else None
            )
            payload["updated_at"] = utc_ts()
            self.store.write_oem_full_lifecycle_run(payload)
        payload["current_stage"] = None
        payload["run_state"] = "dry_run_complete"
        payload["terminal_state"] = payload["planned_terminal_state"]
        payload["updated_at"] = utc_ts()
        return self.store.write_oem_full_lifecycle_run(payload)

    def recover(self, run_id: str) -> dict[str, Any]:
        payload = self.get(run_id)
        if payload["run_state"] == "running" or payload.get("current_stage") is not None:
            for row in payload["stages"]:
                if row["status"] in {"running", "admitted", "acknowledged"}:
                    row["status"] = "blocked"
                    row["blocked_reason"] = "restart_during_physical_or_unresolved_stage_requires_operator_inspection"
            payload.update(
                {
                    "run_state": "blocked",
                    "terminal_state": "oem_movement_blocked",
                    "blocked_reason": "restart_during_physical_or_unresolved_stage_requires_operator_inspection",
                    "current_stage": None,
                    "expected_next_stage": None,
                    "physical_effect_verified": False,
                    "updated_at": utc_ts(),
                }
            )
            return self.store.write_oem_full_lifecycle_run(payload)
        return payload

    def cancel(self, run_id: str) -> dict[str, Any]:
        payload = self.get(run_id)
        if payload["run_state"] != "planned" or payload.get("current_stage") is not None:
            raise OemFullLifecycleError("run is not at a safe cancellation boundary")
        payload.update(
            {
                "run_state": "cancelled",
                "terminal_state": "cancelled",
                "expected_next_stage": None,
                "updated_at": utc_ts(),
            }
        )
        return self.store.write_oem_full_lifecycle_run(payload)
