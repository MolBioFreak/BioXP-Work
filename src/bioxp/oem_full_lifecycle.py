"""Robot-owned full OEM startup/movement lifecycle planning and durable state.

This module implements the source-shaped application-to-terminal graph without a
transport dependency.  It is therefore safe for authority/configuration checks,
dry-run planning, restart/cancellation tests, and API projection.  Physical
providers are attached only after their individual commissioning gates pass.
"""
from __future__ import annotations

import hashlib
import json
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
INITIALIZE_SYSTEM_PRODUCERS = (
    {"producer": "initializeEnvironment", "source_anchor": "BioXPMainWindow.initializeEnvironment:989-997"},
    {"producer": "door_close_event", "source_anchor": "BioXPMainWindow.m_canControl_handleEnclosureDoorEventProcess:2487-2495"},
    {"producer": "optional_update_download_failure", "source_anchor": "BioXPMainWindow.m_pageWarning_buttonclicked:2617-2625"},
    {"producer": "software_update_cancel", "source_anchor": "BioXPMainWindow.m_pageSoftwareUpdate_Cancel_Click:2861-2868"},
    {"producer": "fetch_button", "source_anchor": "BioXPMainWindow.btnFetch_Click:4050-4058"},
)
_REGISTRY_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json"
)


class OemFullLifecycleError(RuntimeError):
    pass


def current_registry_sha256() -> str:
    return hashlib.sha256(_REGISTRY_PATH.read_bytes()).hexdigest()


def current_authority_identity() -> dict[str, Any]:
    """Verify the registry-selected lock bytes before publishing provenance."""
    try:
        registry = json.loads(_REGISTRY_PATH.read_text(encoding="utf-8"))
        authority = registry["authority"]
        lock_path = Path(authority["evidence_lock_path"])
        expected = authority["evidence_lock_sha256"]
        lock_bytes = lock_path.read_bytes()
        lock = json.loads(lock_bytes)
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        raise OemFullLifecycleError(f"canonical OEM evidence authority unavailable: {exc}") from exc
    actual = hashlib.sha256(lock_bytes).hexdigest()
    if expected != OEM_LOCK_SHA256 or actual != expected:
        raise OemFullLifecycleError("canonical OEM evidence lock identity mismatch")
    if lock.get("schema_id") != "bioxp.oem_evidence_lock.v4" or lock.get("schema_version") != 4:
        raise OemFullLifecycleError("canonical OEM evidence lock schema mismatch")
    if lock.get("acquisition", {}).get("session_id") != OEM_ACQUISITION_ID:
        raise OemFullLifecycleError("canonical OEM acquisition identity mismatch")
    return {
        "evidence_lock_path": str(lock_path),
        "evidence_lock_sha256": actual,
        "evidence_lock_schema": "bioxp.oem_evidence_lock.v4",
        "acquisition_id": OEM_ACQUISITION_ID,
        "evidence_lock_identity_verified": True,
    }


# The request carries only lifecycle predicates read by the robot.  It cannot
# carry an axis, board, position, speed, current, timeout, frame, or stage name.
_ALLOWED_INPUTS = frozenset(
    {
        "ownership_generation",
        "can_ready",
        "board_test_mode",
        "pipette_exists",
        "initialize_system_producer",
        "update_check_suppresses_initialize_system",
        "system_in_motion_at_entry",
        "enclosure_door_closed",
        "latch_closed",
        "saved_status",
        "ship_mode",
        "start_mode",
        "tip_present",
        "self_test_due",
        "check_camera",
        "camera_installed",
        "is_development_machine",
        "deck_inspection",
    }
)
_REQUIRED_INPUTS = _ALLOWED_INPUTS
_ALLOWED_REQUEST_KEYS = frozenset(
    {
        "command",
        "operator_ack",
        "expected_generation",
        "bms_connection_generation",
        "expected_machine_serial",
        "expected_registry_sha256",
        "expected_evidence_lock_sha256",
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


def _terminal_for_start_mode(mode: str) -> str:
    try:
        return {
            "DevMode": "oem_movement_ready_development",
            "WebMode": "oem_movement_ready_job_admission",
            "LocalMode": "oem_movement_ready_local_job_admission",
            "TradeShowMode": "oem_movement_ready_trade_show",
        }[mode]
    except KeyError as exc:
        raise OemFullLifecycleError(f"unsupported source StartMode {mode!r}") from exc


def _source_parity_plan(inputs: Mapping[str, Any]) -> tuple[list[dict[str, Any]], str]:
    """Build the selected OEM path in literal constructor→environment→worker order."""
    stages: list[dict[str, Any]] = []

    def add(
        stage_id: str,
        anchor: str,
        *,
        hardware: bool = False,
        motion: bool = False,
        **evidence: Any,
    ) -> None:
        row = _stage(
            stage_id,
            anchor,
            would_command_hardware=hardware,
            would_command_physical_motion=motion,
        )
        row.update(evidence)
        stages.append(row)

    # ControlLib is constructed before BioXPMainWindow.initializeEnvironment().
    add("construct_control_lib", "BioXPMainWindow:672-675")
    if inputs["can_ready"] is True:
        if inputs["board_test_mode"]:
            if inputs["pipette_exists"]:
                add(
                    "constructor_board_test_pipette_initiate_pressure",
                    "ControlLib..ctor:963-972",
                    hardware=True,
                    execution_semantics="BoardTestMode and pipetteExist nested branch: initiateGroup then ReadPressure",
                )
        else:
            add(
                "constructor_pipette_initiate_status_retry",
                "ControlLib..ctor:973-982",
                hardware=True,
                execution_semantics="initiateGroup; checkedPipetteCondition; checkedPipetteStatus; one retry on false",
            )
        add(
            "configure_motors_without_motion",
            "ControlLib..ctor:983;ClassControlInterface.initializeMotorsWithoutMotion:3181-3265",
            hardware=True,
        )
    add("initialize_environment", "BioXPMainWindow:973-1026")

    if inputs["can_ready"] is not True:
        if inputs["start_mode"] == "DevMode":
            add("can_unavailable_development_ready", "BioXPMainWindow:1005-1026")
            return stages, "oem_ready_without_can_development"
        add("can_unavailable_nonmanual_return", "BioXPMainWindow:1005-1010")
        return stages, "oem_blocked_can_unavailable"

    add(
        "initialize_environment_initial_check",
        "BioXPMainWindow:976-979;ControlLib:8728-8795",
        hardware=True,
        result_semantics="return_value_ignored_by_oem_caller",
    )
    door_closed = inputs["enclosure_door_closed"]
    latch_closed = inputs["latch_closed"]
    if door_closed is False and latch_closed is False:
        add("admission_warning_enclosure_open", "BioXPMainWindow:979-982")
        return stages, "oem_waiting_enclosure_open"
    if door_closed is False and latch_closed is True:
        add("admission_unlock_and_retry", "BioXPMainWindow:983-988", hardware=True)
        return stages, "oem_waiting_door_close_retry"
    if door_closed is not True or latch_closed is not True:
        add("admission_unlock_ready_fallback", "BioXPMainWindow:998-1003", hardware=True)
        return stages, "oem_ready_admission_fallback"

    add(
        "enqueue_initialize_system",
        "BioXPMainWindow.initializeEnvironment:989-997",
        producer=inputs["initialize_system_producer"],
    )
    add(
        "worker_update_check_gate",
        "BioXPMainWindow.motion_thread_process:2030-2051;BioXPMainWindow.UpdateCheck:4264-4309",
        source_predicate="!UpdateCheck()",
    )
    if inputs["update_check_suppresses_initialize_system"]:
        add(
            "worker_update_check_suppressed_initialize_system",
            "BioXPMainWindow.motion_thread_process:2030-2051;BioXPMainWindow.UpdateCheck:4264-4309",
            result_semantics="queued initializeSystem is not called when UpdateCheck() returns true",
        )
        return stages, "oem_initialize_system_suppressed_by_update_check"
    add("initialize_system_reentry_guard", "BioXPMainWindow:1094-1100")
    if inputs["system_in_motion_at_entry"]:
        add(
            "initialize_system_reentry_return",
            "BioXPMainWindow.initializeSystem:1094-1098",
            result_semantics="return_before_m_systemInmotion_assignment",
        )
        return stages, "oem_initialize_system_reentry_suppressed"
    add(
        "initialize_system_latch_set",
        "BioXPMainWindow.initializeSystem:1099-1100",
        host_state="m_systemInmotion=true",
    )
    if inputs["ship_mode"] == "PARK":
        add(
            "ship_mode_close_door_result_ignored",
            "BioXPMainWindow.initializeSystem:1127-1133;ControlLib.doorOpen",
            hardware=True,
            result_semantics="boolean_result_ignored_by_oem_caller",
        )
        add(
            "ship_mode_shutdown_requested",
            "BioXPMainWindow.initializeSystem:1129-1135",
            hardware=True,
            execution_semantics="OEM requests Windows shutdown and returns before the later try/finally",
        )
        return stages, "oem_shipping_shutdown_requested_latch_remains_set"

    add(
        "initialize_system_initial_check",
        "BioXPMainWindow:1140-1144;ControlLib:8728-8795",
        hardware=True,
        result_semantics="return_value_ignored_by_oem_caller",
    )
    if inputs["saved_status"] in {3, 4}:
        add("saved_status_initialize_motion", "BioXPMainWindow:1144-1150", hardware=True, motion=True)
        add("saved_status_inspect_cover", "BioXPMainWindow:1144-1150", hardware=True, motion=True)
        add("saved_status_unlock_warning_return", "BioXPMainWindow:1152-1156", hardware=True)
        add(
            "initialize_system_latch_cleared_finally",
            "BioXPMainWindow.initializeSystem:1335-1340",
            host_state="m_systemInmotion=false",
        )
        return stages, "oem_movement_blocked_saved_status_recovery"

    add("initialize_motion_flags", "ControlLib.initializeMotion:8797-8804")
    for number, source_stage in enumerate(OEM_INITIALIZE_MOTORS_STAGES, start=1):
        add(
            f"initialize_motors_m{number:02d}_{source_stage['key'].replace('-', '_')}",
            source_stage["source_anchor"],
            hardware=True,
            motion=True,
            movement_ledger_stage=source_stage["key"],
            movement_ledger_stage_id=f"M{number:02d}",
            movement_ledger_schema="bioxp.oem_initialize_motors_ledger.v1",
        )
    add("initialize_motion_tip_query", "ControlLib.initializeMotion:8805-8807", hardware=True)
    if inputs["tip_present"]:
        for stage_id, anchor, motion in (
            ("tip_open_thermal_door", "ControlLib.initializeMotion:8808-8811", False),
            ("tip_route_park_to_waste", "ControlLib.initializeMotion:8812-8813", True),
            ("tip_eject_all", "ControlLib.initializeMotion:8814", False),
            ("tip_move_z_80000", "ControlLib.initializeMotion:8815", True),
            ("tip_move_x_79000", "ControlLib.initializeMotion:8816", True),
            ("tip_verify_empty", "ControlLib.initializeMotion:8817-8828", False),
            ("pipette_reinitialize_retry_once", "ControlLib.initializeMotion:8829-8841", False),
        ):
            add(stage_id, anchor, hardware=True, motion=motion)
    else:
        add("initialize_motion_no_tip", "ControlLib.initializeMotion:8843-8846")

    add("self_test_gate", "BioXPMainWindow:1163-1171")
    if inputs["self_test_due"]:
        add(
            "self_test_launch_tc_rc_oc",
            "ControlLib:10688-10712",
            hardware=True,
            execution_semantics="three ThreadPool thermal/chiller branches launched before motion self-test",
            parallel_branches=["TC", "RC", "OC"],
        )
        add(
            "self_test_motion_while_thermal_running",
            "ControlLib:10713-10761",
            hardware=True,
            motion=True,
            execution_semantics="motion self-test overlaps TC/RC/OC branches",
        )
        add(
            "self_test_join_and_reset_chillers",
            "ControlLib:10762-10785",
            hardware=True,
            join_timeout_ms=100000,
            cleanup="setChillerPWM",
        )

    add(
        "camera_gate",
        "BioXPMainWindow:1172-1181",
        source_predicate="CheckCamera && CameraInstalled && !IsDevelopmentMachine()",
    )
    if inputs["check_camera"] and inputs["camera_installed"] and not inputs["is_development_machine"]:
        add("camera_check", "BioXPMainWindow.initializeSystem:1172-1181;ControlLib.CheckCamera:1929-1960", hardware=True, motion=True)

    # inspectCover always calls ForceToHighHome before its DeckInspection early return.
    add("cover_force_high_home", "ControlLib:3663-3677", hardware=True, motion=True)
    if inputs["deck_inspection"]:
        add("cover_inspection", "BioXPMainWindow:1182-1203;ControlLib:3663-3768", hardware=True, motion=True)
    else:
        add("cover_inspection_disabled_return_ok", "ControlLib:3672-3677")
    add("gantry_park", "BioXPMainWindow.initializeSystem:1203-1204;ControlLib.parkGantry:7071-7122", hardware=True, motion=True)

    terminal = _terminal_for_start_mode(inputs["start_mode"])
    add(
        f"start_mode_{inputs['start_mode'].lower()}_terminal",
        "BioXPMainWindow:1204-1307",
        hardware=True,
    )
    add(
        "initialize_system_latch_cleared_finally",
        "BioXPMainWindow.initializeSystem:1335-1340",
        host_state="m_systemInmotion=false",
    )
    return stages, terminal


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
        if type(request.get("expected_generation")) is not int or request["expected_generation"] < 1:
            raise OemFullLifecycleError("expected_generation must be a positive integer")
        if type(request.get("bms_connection_generation")) is not int or request["bms_connection_generation"] < 1:
            raise OemFullLifecycleError("bms_connection_generation must be a positive integer")
        if type(request.get("expected_machine_serial")) is not int or request.get("expected_machine_serial") != OEM_MACHINE_SERIAL:
            raise OemFullLifecycleError(f"expected machine serial must be {OEM_MACHINE_SERIAL}")
        actual_registry = current_registry_sha256()
        if request.get("expected_registry_sha256") != actual_registry:
            raise OemFullLifecycleError("expected registry SHA-256 does not match frozen runtime registry")
        if request.get("expected_evidence_lock_sha256") != current_authority_identity()["evidence_lock_sha256"]:
            raise OemFullLifecycleError("expected evidence-lock SHA-256 does not match canonical OEM authority")
        if request.get("mode") != "dry_run":
            raise OemFullLifecycleError("full lifecycle physical execution is blocked until commissioned providers are bound")
        key = request.get("idempotency_key")
        if not isinstance(key, str) or not key.strip() or len(key.strip()) > 128:
            raise OemFullLifecycleError("nonblank idempotency_key of at most 128 characters required")
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
        if request["expected_generation"] != inputs["ownership_generation"]:
            raise OemFullLifecycleError("expected_generation does not match current robot ownership generation")
        if type(inputs["saved_status"]) is not int:
            raise OemFullLifecycleError("saved_status must be an integer")
        if inputs["ship_mode"] not in {"", "PARK"}:
            raise OemFullLifecycleError("ship_mode must be the exact OEM value '' or 'PARK'")
        if inputs["start_mode"] not in {"DevMode", "WebMode", "LocalMode", "TradeShowMode"}:
            raise OemFullLifecycleError("start_mode must be an exact OEM OperationMode name")
        if inputs["initialize_system_producer"] != "initializeEnvironment":
            raise OemFullLifecycleError("this application-start route requires initializeEnvironment producer identity")
        if type(inputs["board_test_mode"]) is not bool:
            raise OemFullLifecycleError("board_test_mode must be an exact boolean")
        if inputs["board_test_mode"] and type(inputs["pipette_exists"]) is not bool:
            raise OemFullLifecycleError("robot-owned pipette_exists must be an exact boolean in BoardTestMode")
        if not inputs["board_test_mode"] and inputs["pipette_exists"] is not None:
            raise OemFullLifecycleError("pipette_exists is only consumed by the BoardTestMode constructor branch")
        for field in (
            "can_ready",
            "enclosure_door_closed",
            "latch_closed",
            "tip_present",
            "self_test_due",
            "check_camera",
            "camera_installed",
            "is_development_machine",
            "update_check_suppresses_initialize_system",
            "system_in_motion_at_entry",
            "deck_inspection",
        ):
            if type(inputs[field]) is not bool:
                raise OemFullLifecycleError(f"{field} must be an exact boolean")
        return {**dict(request), "idempotency_key": key.strip(), "inputs": dict(inputs)}

    def create(
        self,
        request: Mapping[str, Any],
        *,
        machine_configuration_verified: bool = False,
    ) -> dict[str, Any]:
        if type(machine_configuration_verified) is not bool:
            raise OemFullLifecycleError("machine configuration verification must be an exact internal boolean")
        req = self._validate_request(request)
        authority = current_authority_identity()
        stages, planned_terminal = _source_parity_plan(req["inputs"])
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
            # Lock/registry identity verification is not closed-world source,
            # live-provider, machine-configuration, or physical-effect proof.
            "source_authority_verified": False,
            "configuration_verified": False,
            "evidence_lock_verified": authority["evidence_lock_identity_verified"],
            "source_registry_identity_verified": True,
            "machine_configuration_verified": machine_configuration_verified,
            "transport_owner_verified": False,
            "controller_acknowledged": False,
            "postcondition_verified": False,
            "physical_motion_commanded": False,
            "physical_effect_verified": False,
            "safety_deviation": [],
            "registry_sha256": current_registry_sha256(),
            **authority,
            "acquisition_id": OEM_ACQUISITION_ID,
            "machine_serial": OEM_MACHINE_SERIAL,
            "ownership_generation": req["inputs"]["ownership_generation"],
            "transport_frames": [],
            "stages": stages,
            "created_at": now,
            "updated_at": now,
        }
        if req["inputs"]["ship_mode"] == "PARK":
            payload["safety_deviation"] = [{
                "deviation_id": "ship_mode_shutdown_interlock",
                "oem_semantics": "doorOpen(false,false) result ignored; shutdown requested; m_systemInmotion remains true",
                "linux_safety_policy": "never request OS shutdown or infer door closure from an unbound dry-run provider",
                "live_execution_blocked": True,
            }]
        try:
            return self.store.create_oem_full_lifecycle_run_once(payload)
        except ValueError as exc:
            raise OemFullLifecycleError(str(exc)) from exc

    def get(self, run_id: str) -> dict[str, Any]:
        try:
            payload = self.store.read_oem_full_lifecycle_run(run_id)
        except ValueError as exc:
            raise OemFullLifecycleError(str(exc)) from exc
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
            row["status"] = "dry_run_simulated"
            row["completed_at"] = utc_ts()
            payload["expected_next_stage"] = (
                payload["stages"][index + 1]["stage_id"]
                if index + 1 < len(payload["stages"])
                else None
            )
            payload["updated_at"] = utc_ts()
            self.store.write_oem_full_lifecycle_run(payload)
        payload["current_stage"] = None
        payload["run_state"] = "dry_run_non_ready"
        payload["terminal_state"] = "dry_run_non_readiness_terminal"
        payload["updated_at"] = utc_ts()
        return self.store.write_oem_full_lifecycle_run(payload)

    def recover_all(self) -> list[dict[str, Any]]:
        recovered: list[dict[str, Any]] = []
        for payload in self.store.list_oem_full_lifecycle_runs():
            run_id = payload.get("run_id")
            if isinstance(run_id, str):
                recovered.append(self.recover(run_id))
        return recovered

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

    def cancel(self, run_id: str, expected: Mapping[str, Any]) -> dict[str, Any]:
        def mutation(payload: dict[str, Any]) -> dict[str, Any]:
            request = payload.get("request")
            authority = current_authority_identity()
            if not isinstance(request, Mapping):
                raise OemFullLifecycleError("persisted lifecycle request is malformed")
            for field in (
                "expected_generation",
                "bms_connection_generation",
                "expected_machine_serial",
                "expected_registry_sha256",
                "expected_evidence_lock_sha256",
            ):
                if request.get(field) != expected.get(field):
                    raise OemFullLifecycleError(f"cancel {field} does not match the admitted lifecycle request")
            if expected.get("expected_registry_sha256") != current_registry_sha256():
                raise OemFullLifecycleError("cancel registry authority is no longer current")
            if expected.get("expected_evidence_lock_sha256") != authority["evidence_lock_sha256"]:
                raise OemFullLifecycleError("cancel evidence-lock authority is no longer current")
            if payload["run_state"] != "planned" or payload.get("current_stage") is not None:
                raise OemFullLifecycleError("run is not at a safe cancellation boundary")
            payload.update({
                "run_state": "cancelled",
                "terminal_state": "cancelled",
                "expected_next_stage": None,
                "updated_at": utc_ts(),
            })
            return payload

        try:
            return self.store.mutate_oem_full_lifecycle_run(run_id, mutation)
        except ValueError as exc:
            raise OemFullLifecycleError(str(exc)) from exc
