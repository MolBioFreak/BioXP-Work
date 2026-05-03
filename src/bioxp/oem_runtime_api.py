from __future__ import annotations

import os
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from .oem_runtime_commands import OEMRuntimeCommandHandlers
from .oem_runtime_events import OEMRuntimeEventRouter
from .oem_runtime_status import OEMRuntimeStatusService
from .oem_runtime_store import OEMRuntimeStore
from .oem_runtime_types import OEMCommandName, OEMRuntimeCommand

router = APIRouter(prefix="/oem/runtime", tags=["OEM runtime app parity"])

_store: OEMRuntimeStore | None = None
_worker = None
_events: OEMRuntimeEventRouter | None = None
_status: OEMRuntimeStatusService | None = None
_startup_program_factory = None


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
    global _store, _worker, _events, _status, _startup_program_factory
    _startup_program_factory = startup_program_factory
    _store = OEMRuntimeStore(store_root or os.environ.get("BIOXP_OEM_RUNTIME_ROOT") or "/tmp/bioxp-oem-runtime")
    startup_program = startup_program_factory() if startup_program_factory else None
    handlers = OEMRuntimeCommandHandlers(startup_program=startup_program)
    from .oem_runtime_worker import OEMRuntimeWorker
    _worker = OEMRuntimeWorker(store=_store, handlers=handlers.handlers(), autostart=autostart)
    _events = OEMRuntimeEventRouter(store=_store, worker=_worker)
    _status = OEMRuntimeStatusService(store=_store, worker=_worker)
    return {"store": _store, "worker": _worker, "events": _events, "status": _status}


def shutdown_runtime():
    if _worker is not None:
        _worker.stop()


def _require_runtime():
    if _store is None or _worker is None or _events is None or _status is None:
        configure_runtime(autostart=True)
    return _store, _worker, _events, _status


def _enqueue(name: str, req: RuntimeCommandRequest) -> dict:
    _, worker, _, _ = _require_runtime()
    try:
        cmd = OEMRuntimeCommand(name=name, mode=req.mode, source=req.source, params=req.params, operator_ack=req.operator_ack, artifact_root=req.artifact_root, timeout_s=req.timeout_s)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return worker.enqueue(cmd)


@router.get("/status")
def runtime_status():
    _, _, _, status = _require_runtime()
    return status.status()


@router.get("/state")
def runtime_state():
    store, _, _, status = _require_runtime()
    return {"ok": True, "state": status.status(), "recovery": store.recover_state()}


@router.get("/events/latest")
def runtime_events_latest(limit: int = 50):
    store, _, _, _ = _require_runtime()
    return {"ok": True, "events": store.read_journal("event_journal.jsonl", limit=limit)}


@router.get("/commands/history")
def runtime_commands_history(limit: int = 50):
    store, _, _, _ = _require_runtime()
    return {"ok": True, "commands": store.read_journal("command_history.jsonl", limit=limit)}


@router.get("/worker/status")
def runtime_worker_status():
    _, worker, _, _ = _require_runtime()
    return {"ok": True, "worker": worker.snapshot()}


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
