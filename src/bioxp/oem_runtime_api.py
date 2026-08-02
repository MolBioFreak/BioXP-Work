from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, ConfigDict, Field, StrictInt

from .hardware_status import hardware_state

from .oem_full_lifecycle import (
    INITIALIZE_SYSTEM_PRODUCERS,
    OemFullLifecycleError,
    OemFullLifecycleRuns,
    current_authority_identity,
    current_registry_sha256,
)
from .oem_runtime_commands import OEMRuntimeCommandHandlers
from .oem_movement_ledger import OemMovementLedger
from .oem_runtime_events import OEMRuntimeEventRouter
from .oem_runtime_status import OEMRuntimeStatusService
from .oem_runtime_store import OEMRuntimeStore
from .oem_runtime_types import OEMCommandName, OEMRuntimeCommand, new_id
from .lifecycle_state import lifecycle_state

router = APIRouter(prefix="/oem/runtime", tags=["OEM runtime app parity"])

_store: OEMRuntimeStore | None = None
_worker = None
_events: OEMRuntimeEventRouter | None = None
_status: OEMRuntimeStatusService | None = None
_startup_program_factory = None
_serial206_provider_factory = None
_full_lifecycle_runs: OemFullLifecycleRuns | None = None


class FullLifecycleCreateRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    command: str
    operator_ack: str
    expected_generation: StrictInt = Field(ge=1, le=9223372036854775807)
    bms_connection_generation: StrictInt = Field(ge=1, le=9223372036854775807)
    expected_machine_serial: int = Field(ge=206, le=206)
    expected_registry_sha256: str
    expected_evidence_lock_sha256: str
    idempotency_key: str = Field(min_length=1, max_length=128)
    mode: str = "dry_run"


class FullLifecycleCancelRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    expected_generation: StrictInt = Field(ge=1, le=9223372036854775807)
    bms_connection_generation: StrictInt = Field(ge=1, le=9223372036854775807)
    expected_machine_serial: int = Field(ge=206, le=206)
    expected_registry_sha256: str
    expected_evidence_lock_sha256: str


class RuntimeCommandRequest(BaseModel):
    mode: str = "dry_run"
    source: str = "api"
    params: dict[str, Any] = Field(default_factory=dict)
    operator_ack: str | None = None
    artifact_root: str | None = None
    timeout_s: float = Field(default=30.0, gt=0.1, le=900.0)


class GenericCommandRequest(RuntimeCommandRequest):
    name: str


class DoorEventRequest(BaseModel):
    door_open: bool | None = None
    door_closed: bool | None = None
    latch_closed: bool | None = None
    source: str = "api"
    raw: dict[str, Any] = Field(default_factory=dict)
    mode: str = "dry_run"
    artifact_root: str | None = None


def configure_runtime(
    *,
    startup_program_factory=None,
    serial206_provider_factory=None,
    store_root: str | None = None,
    terminal_snapshot_hook=None,
    autostart: bool = True,
):
    global _store, _worker, _events, _status, _startup_program_factory, _serial206_provider_factory, _full_lifecycle_runs
    _startup_program_factory = startup_program_factory
    _serial206_provider_factory = serial206_provider_factory
    _store = OEMRuntimeStore(store_root or os.environ.get("BIOXP_OEM_RUNTIME_ROOT") or "/tmp/bioxp-oem-runtime")
    # Do not call startup_program_factory during API startup: on robot it may open USB.
    # Runtime commands call it lazily only when an initialCheck/startup substage is explicitly requested.
    handlers = OEMRuntimeCommandHandlers(
        startup_program=None,
        startup_program_factory=startup_program_factory,
        serial206_provider_factory=serial206_provider_factory,
        store=_store,
    )
    from .oem_runtime_worker import OEMRuntimeWorker
    _full_lifecycle_runs = OemFullLifecycleRuns(_store)
    _full_lifecycle_runs.recover_all()
    _worker = OEMRuntimeWorker(
        store=_store,
        handlers=handlers.handlers(),
        terminal_snapshot_hook=terminal_snapshot_hook,
        autostart=autostart,
    )
    _events = OEMRuntimeEventRouter(store=_store, worker=_worker)
    _status = OEMRuntimeStatusService(store=_store, worker=_worker)
    return {"store": _store, "worker": _worker, "events": _events, "status": _status, "full_lifecycle_runs": _full_lifecycle_runs}


def shutdown_runtime():
    if _worker is not None:
        _worker.stop()


def movement_ledger_projection() -> dict[str, Any] | None:
    if _store is None:
        return None
    stored = _store.read_oem_movement_ledger()
    if stored is None:
        return None
    return OemMovementLedger(_store).projection()


def _require_runtime():
    if _store is None or _worker is None or _events is None or _status is None:
        configure_runtime(autostart=True)
    return _store, _worker, _events, _status


def _runtime_artifact_root() -> str:
    base = (
        os.environ.get("BIOXP_OEM_RUNTIME_ARTIFACT_ROOT")
        or os.path.join(
            os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT")
            or os.environ.get("BIOXP_OEM_RUNTIME_ROOT")
            or "/tmp/bioxp-oem-runtime",
            "artifacts",
        )
    )
    return os.path.join(base, new_id("startupHomingStepwise"))


def _runtime_unavailable(component: str) -> dict[str, Any]:
    lifecycle = lifecycle_state.projection()
    return {
        "ok": lifecycle["operation_state"] not in {"error", "emergency"},
        "available": False,
        "cache_state": "missing",
        "component": component,
        "error": "OEM runtime owner has not been created by an explicit lifecycle POST",
        "runtime_state": lifecycle["operation_state"],
        "operation_state": lifecycle["operation_state"],
        "startup": lifecycle["startup"],
        "lifecycle": lifecycle,
    }


def _enqueue(name: str, req: RuntimeCommandRequest) -> dict:
    _, worker, _, _ = _require_runtime()
    artifact_root = req.artifact_root
    if name == OEMCommandName.STARTUP_HOMING_STEPWISE.value:
        # The robot, not a remote BMS client, owns artifact filesystem paths.
        artifact_root = _runtime_artifact_root()
    try:
        cmd = OEMRuntimeCommand(name=name, mode=req.mode, source=req.source, params=req.params, operator_ack=req.operator_ack, artifact_root=artifact_root, timeout_s=req.timeout_s)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return worker.enqueue(cmd)


def _derive_full_lifecycle_inputs() -> dict[str, Any]:
    """Derive branch predicates from robot-owned state only.

    Missing or non-exact predicates block creation.  The request model has no
    fields through which BMS can select a stage or supply movement parameters.
    """
    from .oem_machine_bundle import get_active_oem_machine_snapshot

    try:
        snapshot = get_active_oem_machine_snapshot()
    except Exception as exc:
        raise OemFullLifecycleError(f"serial-206 OEM machine snapshot is not bound: {exc}") from exc
    if _status is None or _store is None or _store.read_state() is None:
        raise OemFullLifecycleError("OEM runtime status is not durably available")
    status = _status.status()
    machine = status.get("machine_status")
    if not isinstance(machine, dict):
        raise OemFullLifecycleError("robot-owned machine status is unavailable")

    tip_present = machine.get("tip_loaded")
    self_test_due = machine.get("self_test_due")
    saved_status = machine.get("saved_status")
    ship_mode = machine.get("ship_mode")
    update_check_suppresses = machine.get("update_check_suppresses_initialize_system")
    system_in_motion = machine.get("system_in_motion")
    is_development_machine = machine.get("is_development_machine")
    hardware = status.get("hardware")
    can_ready = hardware.get("can_ready") if isinstance(hardware, dict) else None
    lifecycle = lifecycle_state.projection()
    board_test_mode = lifecycle.get("board_test_mode")
    if type(board_test_mode) is not bool:
        raise OemFullLifecycleError("robot-owned board_test_mode must be an exact boolean")
    pipette_exists = None
    if board_test_mode:
        pipette_exists = hardware.get("pipette_exists") if isinstance(hardware, dict) else None
        if type(pipette_exists) is not bool:
            raise OemFullLifecycleError("robot-owned pipette_exists must be an exact boolean in BoardTestMode")
    enclosure_door_closed = machine.get("enclosure_door_closed")
    latch_closed = machine.get("latch_closed")
    for field, value in (
        ("can_ready", can_ready),
        ("enclosure_door_closed", enclosure_door_closed),
        ("latch_closed", latch_closed),
        ("tip_loaded", tip_present),
        ("self_test_due", self_test_due),
        ("update_check_suppresses_initialize_system", update_check_suppresses),
        ("system_in_motion", system_in_motion),
        ("is_development_machine", is_development_machine),
    ):
        if type(value) is not bool:
            raise OemFullLifecycleError(f"robot-owned {field} must be an exact boolean")
    if type(saved_status) is not int:
        raise OemFullLifecycleError("robot-owned saved_status must be an integer")
    if not isinstance(ship_mode, str):
        raise OemFullLifecycleError("robot-owned ship_mode must be a string")
    camera_installed = snapshot.fields["machine.camera_installed"].value
    check_camera = snapshot.operation_parameters["CheckCamera"]
    if type(camera_installed) is not bool or type(check_camera) is not bool:
        raise OemFullLifecycleError("selected CameraInstalled/CheckCamera values must be exact booleans")
    deck_inspection = snapshot.operation_parameters["DeckInspection"]
    if type(deck_inspection) is not bool:
        raise OemFullLifecycleError("selected DeckInspection value must be an exact boolean")
    return {
        "ownership_generation": int(hardware_state.ownership_epoch),
        "can_ready": can_ready,
        "board_test_mode": board_test_mode,
        "pipette_exists": pipette_exists,
        "initialize_system_producer": "initializeEnvironment",
        "update_check_suppresses_initialize_system": update_check_suppresses,
        "system_in_motion_at_entry": system_in_motion,
        "enclosure_door_closed": enclosure_door_closed,
        "latch_closed": latch_closed,
        "saved_status": saved_status,
        "ship_mode": ship_mode,
        "start_mode": snapshot.startup_mode,
        "tip_present": tip_present,
        "self_test_due": self_test_due,
        "check_camera": check_camera,
        "camera_installed": camera_installed,
        "is_development_machine": is_development_machine,
        "deck_inspection": deck_inspection,
    }


def _require_full_lifecycle_runs() -> OemFullLifecycleRuns:
    global _full_lifecycle_runs
    if _full_lifecycle_runs is None:
        _require_runtime()
    assert _full_lifecycle_runs is not None
    return _full_lifecycle_runs


def _full_lifecycle_plan_blockers() -> list[str]:
    try:
        _derive_full_lifecycle_inputs()
    except OemFullLifecycleError as exc:
        return [str(exc)]
    return []


@router.get("/movement-runs/contract")
def full_lifecycle_contract():
    authority = current_authority_identity()
    try:
        lifecycle_inputs = _derive_full_lifecycle_inputs()
        plan_blockers: list[str] = []
        ownership_generation: int | None = lifecycle_inputs["ownership_generation"]
    except OemFullLifecycleError as exc:
        plan_blockers = [str(exc)]
        ownership_generation = None
    return {
        "schema_version": "bioxp.oem_full_lifecycle_contract.v1",
        "command": "initialize_oem_movement_lifecycle",
        "machine_serial": 206,
        "ownership_generation": ownership_generation,
        "registry_sha256": current_registry_sha256(),
        **authority,
        "evidence_lock_verified": authority["evidence_lock_identity_verified"],
        "source_registry_identity_verified": True,
        "machine_configuration_verified": not plan_blockers,
        "source_authority_verified": False,
        "initialize_system_producers": list(INITIALIZE_SYSTEM_PRODUCERS),
        "plan_available": not plan_blockers,
        "plan_blockers": plan_blockers,
        "live_creation_enabled": False,
        "physical_commissioning_complete": False,
        "providers": {
            "initial_check": {"source_contract": True, "implemented": True, "live_bound": True, "commissioned": False},
            "configure_motors_without_motion": {"source_contract": True, "implemented": True, "live_bound": True, "commissioned": False},
            "initialize_motors_m01_m19": {"source_contract": True, "implemented": True, "live_bound": True, "commissioned": False},
            "pipette_tip_query_and_remediation": {"source_contract": True, "implemented": True, "live_bound": False, "commissioned": False},
            "tc_rc_oc_motion_self_test": {"source_contract": True, "implemented": "receipt_evaluator", "live_bound": False, "commissioned": False},
            "check_camera": {"source_contract": True, "implemented": "receipt_evaluator", "live_bound": False, "commissioned": False},
            "inspect_cover": {"source_contract": True, "implemented": "receipt_evaluator", "live_bound": False, "commissioned": False},
            "park_gantry": {"source_contract": True, "implemented": "receipt_evaluator", "live_bound": False, "commissioned": False},
            "start_mode_terminal": {"source_contract": True, "implemented": "typed_plan", "live_bound": False, "commissioned": False},
            "shutdown_camera_disposal": {"source_contract": True, "implemented": False, "live_bound": False, "commissioned": False},
        },
        "safety_boundary": {
            "caller_supplied_motion_parameters": False,
            "dry_run_commands_hardware": False,
            "queue_acceptance_is_execution": False,
            "physical_effect_verified": False,
        },
    }


@router.post("/movement-runs")
def create_full_lifecycle_run(request: FullLifecycleCreateRequest):
    runs = _require_full_lifecycle_runs()
    try:
        with hardware_state.ownership_lease():
            inputs = _derive_full_lifecycle_inputs()
            return runs.create(
                {**request.model_dump(), "inputs": inputs},
                machine_configuration_verified=True,
                current_ownership_generation=lambda: hardware_state.ownership_epoch,
            )
    except OemFullLifecycleError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@router.get("/movement-runs/{run_id}")
def get_full_lifecycle_run(run_id: str):
    try:
        return _require_full_lifecycle_runs().get(run_id)
    except OemFullLifecycleError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.get("/movement-runs/{run_id}/ledger")
def get_full_lifecycle_ledger(run_id: str):
    try:
        return _require_full_lifecycle_runs().get(run_id)
    except OemFullLifecycleError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.post("/movement-runs/{run_id}/cancel")
def cancel_full_lifecycle_run(run_id: str, request: FullLifecycleCancelRequest):
    try:
        with hardware_state.ownership_lease():
            return _require_full_lifecycle_runs().cancel(
                run_id,
                request.model_dump(),
                current_ownership_generation=lambda: hardware_state.ownership_epoch,
            )
    except OemFullLifecycleError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@router.get("/status")
def runtime_status():
    if _status is None or _store is None:
        return _runtime_unavailable("status")
    if _store.read_state() is None:
        return _runtime_unavailable("status")
    return _status.status()


@router.get("/state")
def runtime_state():
    if _store is None or _status is None:
        return _runtime_unavailable("state")
    if _store.read_state() is None:
        return _runtime_unavailable("state")
    return {"ok": True, "state": _status.status(), "recovery": _store.recover_state()}


@router.get("/events/latest")
def runtime_events_latest(limit: int = 50):
    if _store is None:
        return {**_runtime_unavailable("events"), "events": []}
    return {"ok": True, "events": _store.read_journal("event_journal.jsonl", limit=limit)}


@router.get("/commands/history")
def runtime_commands_history(limit: int = 50):
    if _store is None:
        return {**_runtime_unavailable("commands"), "commands": []}
    return {"ok": True, "commands": _store.read_journal("command_history.jsonl", limit=limit)}


@router.get("/commands/{command_id}")
def runtime_command_result(command_id: str):
    if _store is None or _worker is None:
        return _runtime_unavailable("command_result")
    for row in reversed(_store.read_journal("command_history.jsonl", limit=500)):
        command = row.get("command") if isinstance(row, dict) else None
        if isinstance(command, dict) and command.get("command_id") == command_id:
            return {"ok": True, "state": "terminal", "terminal": row}
    active = _worker.active_command
    if active is not None and active.command_id == command_id:
        return {"ok": True, "state": "running", "command": active.to_dict()}
    for command in reversed(_store.read_journal("command_queue.jsonl", limit=500)):
        if isinstance(command, dict) and command.get("command_id") == command_id:
            return {"ok": True, "state": "queued", "command": command}
    raise HTTPException(status_code=404, detail="OEM runtime command id was not found")


@router.get("/worker/status")
def runtime_worker_status():
    if _worker is None:
        return {**_runtime_unavailable("worker"), "worker": None}
    lifecycle = lifecycle_state.projection()
    return {"ok": lifecycle["operation_state"] not in {"error", "emergency"}, "worker": _worker.snapshot(), "runtime_state": lifecycle["operation_state"], "startup": lifecycle["startup"], "lifecycle": lifecycle}


@router.post("/commands/enqueue")
def runtime_commands_enqueue(req: GenericCommandRequest):
    try:
        OEMCommandName.validate(req.name)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return _enqueue(req.name, req)


@router.post("/commands/initializeSystem")
def runtime_command_initialize_system(req: RuntimeCommandRequest):
    return _enqueue(OEMCommandName.INITIALIZE_SYSTEM.value, req)


@router.post("/commands/unlockProcess")
def runtime_command_unlock_process(req: RuntimeCommandRequest):
    return _enqueue(OEMCommandName.UNLOCK_PROCESS.value, req)


@router.post("/commands/PrepareToRunJob")
def runtime_command_prepare_to_run_job(req: RuntimeCommandRequest):
    return _enqueue(OEMCommandName.PREPARE_TO_RUN_JOB.value, req)


@router.post("/readiness/prepare-to-run-job/dry-run")
def runtime_prepare_to_run_job_readiness_dry_run(req: RuntimeCommandRequest):
    params = dict(req.params or {})
    params["no_motion"] = True
    params["deck_inspection"] = True
    try:
        cmd = OEMRuntimeCommand(
            name=OEMCommandName.PREPARE_TO_RUN_JOB.value,
            mode="dry_run",
            source=req.source,
            params=params,
            operator_ack=req.operator_ack,
            artifact_root=req.artifact_root,
            timeout_s=req.timeout_s,
        )
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    handlers = OEMRuntimeCommandHandlers(
        startup_program_factory=_startup_program_factory,
        serial206_provider_factory=_serial206_provider_factory,
    )
    return handlers.handle_prepare_to_run_job_readiness(cmd)


@router.post("/commands/abortjob")
def runtime_command_abortjob(req: RuntimeCommandRequest):
    return _enqueue(OEMCommandName.ABORT_JOB.value, req)


@router.post("/commands/validateJob")
def runtime_command_validate_job(req: RuntimeCommandRequest):
    return _enqueue(OEMCommandName.VALIDATE_JOB.value, req)


@router.post("/commands/wakefrompause")
def runtime_command_wakefrompause(req: RuntimeCommandRequest):
    return _enqueue(OEMCommandName.WAKE_FROM_PAUSE.value, req)


@router.post("/events/door")
def runtime_event_door(req: DoorEventRequest):
    _, _, events, _ = _require_runtime()
    return events.handle_door_event(door_open=req.door_open, door_closed=req.door_closed, latch_closed=req.latch_closed, source=req.source, raw=req.raw, mode=req.mode, artifact_root=req.artifact_root)


@router.post("/events/pause")
def runtime_event_pause():
    _, _, events, _ = _require_runtime()
    return events.handle_pause()


@router.post("/events/resume")
def runtime_event_resume(req: RuntimeCommandRequest = RuntimeCommandRequest()):
    _, _, events, _ = _require_runtime()
    return events.handle_resume(mode=req.mode)


@router.post("/emergency_stop")
def runtime_emergency_stop(reason: str = "operator_request"):
    _, _, events, _ = _require_runtime()
    return events.emergency_stop(reason=reason)


@router.post("/recover")
def runtime_recover():
    store, _, _, status = _require_runtime()
    return {"ok": True, "recovery": store.recover_state(), "status": status.status()}
