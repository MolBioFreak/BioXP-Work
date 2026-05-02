from __future__ import annotations

from dataclasses import asdict
from typing import Any

from fastapi import APIRouter
from pydantic import BaseModel

from .control_lib import BioXPControlLib
from .scripts import OemScript

router = APIRouter(prefix="/oem-compat", tags=["BioXP OEM compatibility dry-run"])


class StartupDryRunRequest(BaseModel):
    run_homing: bool = True


class ScriptTranslateRequest(BaseModel):
    xml: str


def _trace_to_dict(trace) -> dict[str, Any]:
    return {
        "name": trace.name,
        "operations": [asdict(op) for op in trace.operations],
    }


@router.post("/startup/dry-run")
def startup_dry_run(request: StartupDryRunRequest) -> dict[str, Any]:
    control = BioXPControlLib.dry_run()
    report = control.startup(run_homing=request.run_homing)
    return {
        "ok": report.ok,
        "mode": report.mode,
        "physical_motion": report.physical_motion,
        "trace_names": report.trace_names,
        "traces": [_trace_to_dict(t) for t in report.traces],
        "frame_count": len(control.transport.frames),
        "opened_usb": control.transport.opened_usb,
    }


@router.post("/scripts/translate/dry-run")
def script_translate_dry_run(request: ScriptTranslateRequest) -> dict[str, Any]:
    control = BioXPControlLib.dry_run()
    script = OemScript.from_text(request.xml)
    result = control.execute_script(script)
    return {
        "mode": result.mode,
        "executed": result.executed,
        "actions": [asdict(a) for a in result.actions],
    }
