from __future__ import annotations

import os
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, ConfigDict, Field

from .oem_full_lifecycle import OemFullLifecycleError, OemFullLifecycleRuns
from .oem_runtime_commands import OEMRuntimeCommandHandlers
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
_full_lifecycle_runs: OemFullLifecycleRuns | None = None


class FullLifecycleCreateRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    command: str
    operator_ack: str
    expected_machine_serial: int
    expected_registry_sha256: str
    idempotency_key: str
    mode: str = "dry_run"


class RuntimeCommandRequest(BaseModel):
    mode: str = "dry_run"
    source: str = "api"
    params: dict[str, Any] = Field(default_factory=dict)
    operator_ack: str | None = None
    artifact_root: str | None = None
    timeout_s: float = 30.0


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


def configure_runtime(*, startup_program_factory=None, store_root: str | None = None, autostart: bool = True):
    global _store, _worker, _events, _status, _startup_program_factory, _full_lifecycle_runs
    _startup_program_factory = startup_program_factory
    _store = OEMRuntimeStore(store_root or os.environ.get("BIOXP_OEM_RUNTIME_ROOT") or "/tmp/bioxp-oem-runtime")
    # Do not call startup_program_factory during API startup: on robot it may open USB.
    # Runtime commands call it lazily only when an initialCheck/startup substage is explicitly requested.
    handlers = OEMRuntimeCommandHandlers(
        startup_program=None,
        startup_program_factory=startup_program_factory,
        store=_store,
    )
    from .oem_runtime_worker import OEMRuntimeWorker
    _worker = OEMRuntimeWorker(store=_store, handlers=handlers.handlers(), autostart=autostart)
    _events = OEMRuntimeEventRouter(store=_store, worker=_worker)
    _status = OEMRuntimeStatusService(store=_store, worker=_worker)
    _full_lifecycle_runs = OemFullLifecycleRuns(_store)
    return {"store": _store, "worker": _worker, "events": _events, "status": _status, "full_lifecycle_runs": _full_lifecycle_runs}


def shutdown_runtime():
    if _worker is not None:
        _worker.stop()


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
    from .hardware_status import hardware_state
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
    for field, value in (("tip_loaded", tip_present), ("self_test_due", self_test_due)):
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
    camera_required = check_camera and camera_installed
    deck_inspection = snapshot.operation_parameters["DeckInspection"]
    if type(deck_inspection) is not bool:
        raise OemFullLifecycleError("selected DeckInspection value must be an exact boolean")
    return {
        "ownership_generation": int(hardware_state.ownership_epoch),
        "saved_status": saved_status,
        "ship_mode": ship_mode,
        "start_mode": snapshot.startup_mode,
        "tip_present": tip_present,
        "self_test_due": self_test_due,
        "camera_required": camera_required,
        "deck_inspection": deck_inspection,
    }


def _require_full_lifecycle_runs() -> OemFullLifecycleRuns:
    global _full_lifecycle_runs
    if _full_lifecycle_runs is None:
        _require_runtime()
    assert _full_lifecycle_runs is not None
    return _full_lifecycle_runs


@router.post("/movement-runs")
def create_full_lifecycle_run(req: FullLifecycleCreateRequest):
    runs = _require_full_lifecycle_runs()
    try:
        inputs = _derive_full_lifecycle_inputs()
        return runs.create({**req.model_dump(), "inputs": inputs})
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
def cancel_full_lifecycle_run(run_id: str):
    try:
        return _require_full_lifecycle_runs().cancel(run_id)
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
    handlers = OEMRuntimeCommandHandlers(startup_program_factory=_startup_program_factory)
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
