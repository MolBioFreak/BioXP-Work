from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from tempfile import gettempdir
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .control_lib import BioXPControlLib
from .scripts import OemScript
from .transport import ReplayTransport, SafetyContractViolation, assert_transport_safety, _frame_to_json

router = APIRouter(prefix="/oem-compat", tags=["BioXP OEM compatibility dry-run"])


class StartupDryRunRequest(BaseModel):
    run_homing: bool = True
    artifact_path: str | None = None


class ScriptTranslateRequest(BaseModel):
    xml: str


def _trace_to_dict(trace) -> dict[str, Any]:
    return {"name": trace.name, "operations": [asdict(op) for op in trace.operations]}


def _validate_artifact_path(path_text: str) -> Path:
    path = Path(path_text).expanduser().resolve()
    allowed_roots = [
        Path(gettempdir()).resolve(),
        Path.cwd().resolve(),
        Path("/mnt/BioModStack").resolve(),
        Path.home().resolve() / "Desktop" / "BioXP 3200 Development Work",
    ]
    if not any(path == root or root in path.parents for root in allowed_roots):
        raise HTTPException(status_code=400, detail=f"artifact_path outside allowed dry-run roots: {path}")
    return path


def _write_startup_artifact(path: Path, *, report, frames) -> dict[str, Any]:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "format": "bioxp-oem-compat-trace-v1",
        "mode": report.mode,
        "physical_motion": report.physical_motion,
        "trace_names": report.trace_names,
        "traces": [_trace_to_dict(t) for t in report.traces],
        "frame_count": len(frames),
        "frames": [_frame_to_json(f) for f in frames],
    }
    import json

    path.write_text(json.dumps(payload, indent=2, sort_keys=True))
    replay = ReplayTransport.from_file(path)
    for frame in frames:
        replay.transmit(frame)
    replay.assert_complete()
    return {"artifact_path": str(path), "replay_ok": replay.position == len(frames)}


@router.post("/startup/dry-run")
def startup_dry_run(request: StartupDryRunRequest) -> dict[str, Any]:
    control = BioXPControlLib.dry_run()
    report = control.startup(run_homing=request.run_homing)
    try:
        assert_transport_safety(
            mode=report.mode,
            opened_usb=control.transport.opened_usb,
            physical_motion=report.physical_motion,
        )
    except SafetyContractViolation as exc:
        raise HTTPException(status_code=500, detail=f"OEM compatibility safety contract violation: {exc}") from exc
    body = {
        "ok": report.ok,
        "mode": report.mode,
        "physical_motion": report.physical_motion,
        "trace_names": report.trace_names,
        "traces": [_trace_to_dict(t) for t in report.traces],
        "frame_count": len(control.transport.frames),
        "opened_usb": control.transport.opened_usb,
    }
    if request.artifact_path:
        body.update(_write_startup_artifact(_validate_artifact_path(request.artifact_path), report=report, frames=control.transport.frames))
    return body


@router.post("/scripts/translate/dry-run")
def script_translate_dry_run(request: ScriptTranslateRequest) -> dict[str, Any]:
    control = BioXPControlLib.dry_run()
    script = OemScript.from_text(request.xml)
    result = control.execute_script(script)
    return {"mode": result.mode, "executed": result.executed, "actions": [asdict(a) for a in result.actions]}
