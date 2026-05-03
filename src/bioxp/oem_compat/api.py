from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from tempfile import NamedTemporaryFile, gettempdir
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .control_lib import BioXPControlLib
from .scripts import OemScript
from ..protocols.oem_xml_import import import_oem_xml_protocol
from .boards import axis_profile_matrix
from .transport import ReplayTransport, SafetyContractViolation, assert_transport_safety, _frame_to_json

router = APIRouter(prefix="/oem-compat", tags=["BioXP OEM compatibility dry-run"])


class StartupDryRunRequest(BaseModel):
    run_homing: bool = True
    artifact_path: str | None = None


class ScriptTranslateRequest(BaseModel):
    xml: str


class ProtocolImportDryRunRequest(BaseModel):
    xml: str
    source_name: str = "inline-oem-protocol.xml"


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


@router.get("/capabilities/test-prep")
def capability_matrix_test_prep() -> dict[str, Any]:
    """Machine-readable handoff for BMS/thin-control test prep surfaces."""
    return {
        "schema_version": "bioxp.oem_compat.capability_matrix.v1",
        "robot_hardware_assumption": "functional_under_oem",
        "truth_source": "robot_local_oem_compat_layer",
        "bms_role": "thin_operator_surface",
        "capabilities": {
            "motion": {
                "prep_ready": True,
                "axis_profile_matrix": axis_profile_matrix(),
                "truth_model": "controller_readback_plus_artifacts; physical_observation_optional_field_not_implied",
            },
            "deck_semantics": {
                "prep_ready": True,
                "position_table_module": "src.bioxp.oem_compat.position_table",
                "semantic_actions": ["moveTo"],
            },
            "oem_xml_jobs": {
                "prep_ready": True,
                "supported_verbs_include": ["MT", "FP", "RT", "SA", "ST", "SW", "TT", "ZW"],
                "zero_unsupported_corpus_gate": True,
                "virtual_state_dry_run_endpoint": "/oem-compat/protocols/import/dry-run",
                "dry_run_required_before_live": True,
                "artifact_bundle_required_for_live": True,
            },
            "pipette": {
                "prep_ready": True,
                "ack_readback_required": True,
                "fail_closed_without_reference_tip_deck_pressure_state": True,
            },
            "vision": {
                "prep_ready": True,
                "artifacted_failures": True,
                "transport_health_distinct_from_inspection_semantics": True,
            },
        },
    }


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


@router.post("/protocols/import/dry-run")
def protocol_import_dry_run(request: ProtocolImportDryRunRequest) -> dict[str, Any]:
    source_name = Path(request.source_name).name or "inline-oem-protocol.xml"
    if not source_name.lower().endswith(".xml"):
        source_name = f"{source_name}.xml"
    with NamedTemporaryFile("w", suffix=f"-{source_name}", prefix="bioxp-oem-import-", delete=False) as handle:
        temp_path = Path(handle.name)
        handle.write(request.xml)
    try:
        imported = import_oem_xml_protocol(temp_path)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"OEM XML import failed: {exc}") from exc
    finally:
        temp_path.unlink(missing_ok=True)

    job = BioXPControlLib.dry_run().execute_protocol(imported.document, source_path=source_name)
    return {
        "ok": job.ok,
        "mode": job.mode,
        "coverage": imported.coverage.to_payload(),
        "protocol_id": imported.document.protocol_id,
        "source_name": source_name,
        "job": job.to_payload(),
    }
