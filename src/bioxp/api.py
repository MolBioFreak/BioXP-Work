import asyncio
import base64
import os
import signal
import subprocess
import tempfile
import time
from contextlib import asynccontextmanager
from enum import Enum
from typing import Any, Optional

from fastapi import FastAPI, HTTPException, Query, Request

from .oem_compat.api import router as oem_compat_router
from .oem_runtime_api import configure_runtime as configure_oem_runtime, router as oem_runtime_router, shutdown_runtime as shutdown_oem_runtime
from .oem_startup_program import BioXpStartupHardware, OEMStartupProgram, FakeStartupHardware
from .oem_startup_types import OemDoorEventRequest, OemInitialCheckRequest, OemStartupRequest, OemSwitchAuditRequest
from .oem_switch_audit import FakeSwitchAuditHardware, run_switch_audit
from pydantic import BaseModel, Field
from starlette.background import BackgroundTask
from starlette.concurrency import run_in_threadpool
from starlette.responses import StreamingResponse

from .domain.capabilities import CapabilityRegistry
from .domain.deck import load_deck_layout
from .pipette import (
    PipetteAspirateCommand,
    PipetteDispenseCommand,
    PipetteInitCommand,
    PipetteMixCommand,
    PipettePreflightError,
    PipetteTipAction,
    PipetteTipCommand,
    build_default_pipette_transport,
)
from .services.artifact_service import create_motion_validation_bundle
from .services.motion_service import (
    AbsoluteMoveCommand,
    HomeAxisCommand,
    RelativeMoveCommand,
    dry_run_motion_response as service_dry_run_motion_response,
    run_absolute_motion_command,
    run_home_axis_command,
    run_relative_motion_command,
)
from .services.pipette_service import (
    run_pipette_aspirate_command,
    run_pipette_dispense_command,
    run_pipette_init_command,
    run_pipette_mix_command,
    run_pipette_status,
    run_pipette_tip_command,
)
from .services.protocol_service import (
    ProtocolLiveContractError,
    create_protocol_job,
    get_protocol_job,
    list_protocol_jobs,
    review_protocol_job,
    compile_protocol_source,
)
from .services.reference_service import (
    MarkAxisDesyncedCommand,
    MarkAxisReferencedCommand,
    ReferenceStateStore,
)
from .services.vision_service import (
    run_barcode_read_command,
    run_inspection_command,
)
try:
    from .usb_driver import BioXpTester
except ModuleNotFoundError as exc:
    if exc.name != "usb":
        raise
    _usb_driver_import_error = exc

    class BioXpTester:  # type: ignore[no-redef]
        THERMAL_BANK_NEST = 0
        THERMAL_BANK_LID = 1
        CHILLER_BANK_RC = 0
        CHILLER_BANK_OC = 1

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise RuntimeError(f"BioXP USB runtime dependency unavailable: {_usb_driver_import_error}") from _usb_driver_import_error

from .vision.barcode import BarcodeReadCommand
from .vision.inspection import InspectionCommand

_tester: Optional[BioXpTester] = None
_startup_error: Optional[str] = None
_tester_lock = asyncio.Lock()
_camera_stream_lock = asyncio.Lock()
_oem_startup_program: Optional[OEMStartupProgram] = None


def _default_reference_state_path() -> str:
    xdg_state_home = os.environ.get("XDG_STATE_HOME")
    if xdg_state_home:
        base_dir = xdg_state_home
    else:
        home_dir = os.path.expanduser("~")
        if home_dir and home_dir != "~" and os.path.isabs(home_dir):
            base_dir = os.path.join(home_dir, ".local", "state")
        else:
            base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".state"))
    return os.path.join(base_dir, "bioxp", "reference-state.json")


_reference_state_store = ReferenceStateStore(
    state_path=os.environ.get("BIOXP_REFERENCE_STATE_PATH") or _default_reference_state_path()
)
_pipette_transport = build_default_pipette_transport()
try:
    _vision_capabilities = load_deck_layout().capabilities
except Exception:
    _vision_capabilities = CapabilityRegistry.from_config({"inspection": True, "barcode": False})
_camera_stream_state = {
    "active": False,
    "device": None,
    "fps": None,
    "quality": None,
    "width": None,
    "height": None,
    "frames_emitted": 0,
    "started_at": None,
    "last_frame_at": None,
    "last_error": None,
}
RESET_PROVENANCE_SCHEMA_VERSION = "bioxp.reset_provenance.v1"


def _reset_provenance(*, subsystem: str, source: str, reset_scope: str, **extra: Any) -> dict[str, Any]:
    payload = {
        "schema_version": RESET_PROVENANCE_SCHEMA_VERSION,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "subsystem": subsystem,
        "source": source,
        "reset_scope": reset_scope,
        "software_recovery": bool(extra.pop("software_recovery", True)),
        "hardware_component_fault_proven": bool(extra.pop("hardware_component_fault_proven", False)),
    }
    payload.update(extra)
    return payload


@asynccontextmanager
async def lifespan(app: FastAPI):
    del app
    global _tester, _startup_error, _pipette_transport
    _pipette_transport = build_default_pipette_transport()
    try:
        alt = int(os.environ.get("BIOXP_USB_ALT", "1"))
        _tester = BioXpTester(alt=alt)
        _startup_error = None
    except Exception as exc:
        _tester = None
        _startup_error = str(exc)
        print(f"[WARN] BioXP USB runtime unavailable: {_startup_error}")
    try:
        configure_oem_runtime(startup_program_factory=lambda: _get_oem_startup_program(dry_safe=True), autostart=True)
    except Exception as exc:
        print(f"[WARN] BioXP OEM runtime unavailable: {exc}")
    try:
        yield
    finally:
        shutdown_oem_runtime()
        _tester = None
        close_fn = getattr(_pipette_transport, "close", None)
        if callable(close_fn):
            close_fn()
        _pipette_transport = build_default_pipette_transport()


app = FastAPI(
    title="BioXP 3200 Control API",
    description="REST interface backed by the canonical USB runtime (usb_driver.py).",
    version="0.3.0",
    lifespan=lifespan,
)
app.include_router(oem_compat_router)
app.include_router(oem_runtime_router)


def _get_tester() -> BioXpTester:
    if _tester is None:
        raise HTTPException(
            status_code=503,
            detail=_startup_error or "BioXP USB runtime not available.",
        )
    return _tester


def _get_oem_startup_program(*, dry_safe: bool = True) -> OEMStartupProgram:
    global _oem_startup_program
    base = os.environ.get("BIOXP_OEM_STARTUP_ARTIFACT_BASE") or "/tmp/bioxp-live-runs"
    want_live_provider = not dry_safe
    current_is_fake = isinstance(getattr(_oem_startup_program, "hardware", None), FakeStartupHardware)
    if _oem_startup_program is None or (want_live_provider and current_is_fake):
        if want_live_provider and _tester is not None:
            hardware = BioXpStartupHardware(_get_tester)
        else:
            hardware = FakeStartupHardware(door_closed=False, latch_closed=False)
        _oem_startup_program = OEMStartupProgram(hardware=hardware, artifact_base=base)
    return _oem_startup_program


class _BioXpSwitchAuditHardware:
    def __init__(self, tester: BioXpTester):
        self.tester = tester
        self.move_calls: list[dict] = []

    def switch_snapshot(self, axis: str) -> dict:
        preset = self.tester._motion_oem_axis_profile(axis) if hasattr(self.tester, "_motion_oem_axis_profile") else None
        if not isinstance(preset, dict):
            return FakeSwitchAuditHardware().switch_snapshot(axis)
        board = int(preset.get("board", 0))
        motor = int(preset.get("motor", 0))
        def safe(fn, default=None):
            try:
                return fn()
            except Exception as exc:
                return {"error": str(exc)} if default is None else default
        return {
            "axis": axis,
            "board": board,
            "motor": motor,
            "position": safe(lambda: self.tester.motor_get_position(board, motor=motor)),
            "speed": safe(lambda: self.tester.motor_get_speed(board, motor=motor)),
            "gap9_left": safe(lambda: self.tester.motor_get_axis_param(board, 9, motor=motor)) if hasattr(self.tester, "motor_get_axis_param") else {},
            "gap10_right": safe(lambda: self.tester.motor_get_axis_param(board, 10, motor=motor)) if hasattr(self.tester, "motor_get_axis_param") else {},
            "home_query": safe(lambda: self.tester.motor_query_home_switch(board, motor=motor)),
            "switch_masks": {},
            "current_params": {},
            "oem_profile": preset,
        }


def _get_pipette_transport():
    return _pipette_transport


@app.post("/oem/startup/request")
async def oem_startup_request(req: OemStartupRequest):
    request_payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    if request_payload.get("mode") == "live" and request_payload.get("operator_ack") != "INITIALIZE":
        raise HTTPException(status_code=409, detail="operator_ack INITIALIZE required for live OEM startup")
    if request_payload.get("mode") == "live" and not request_payload.get("artifact_root"):
        raise HTTPException(status_code=409, detail="artifact_root required for live OEM startup")
    program = _get_oem_startup_program(dry_safe=request_payload.get("mode") != "live")
    status = program.request_startup(request_payload)
    if status.get("failed_closed") and request_payload.get("mode") == "live":
        raise HTTPException(status_code=409, detail=status.get("failure_reason") or "OEM startup failed closed")
    return {
        "ok": bool(status.get("ok", True)),
        "session_id": status.get("session_id"),
        "status": status.get("state"),
        "state": status.get("state"),
        "mode": status.get("mode"),
        "queued": bool(status.get("queued", False)),
        "artifact_root": status.get("artifact_root"),
        "startup_status_url": f"/oem/startup/status/{status.get('session_id')}",
        "source_anchors": list((status.get("source_anchors") or {}).values()),
    }


@app.get("/oem/startup/status/latest")
async def oem_startup_status_latest():
    return _get_oem_startup_program().status()


@app.get("/oem/startup/status/{session_id}")
async def oem_startup_status(session_id: str):
    status = _get_oem_startup_program().status(session_id)
    if status.get("state") == "none":
        raise HTTPException(status_code=404, detail="startup session not found")
    return status


@app.post("/oem/startup/door_event")
async def oem_startup_door_event(req: OemDoorEventRequest):
    payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    try:
        return _get_oem_startup_program().door_event(
            payload.get("session_id"),
            door_closed=bool(payload.get("door_closed")),
            latch_closed=bool(payload.get("latch_closed")),
        )
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@app.post("/oem/initial_check")
async def oem_initial_check(req: OemInitialCheckRequest | None = None):
    payload = (req.model_dump() if hasattr(req, "model_dump") else req.dict()) if req is not None else {"mode": "shadow"}
    mode = payload.get("mode", "shadow")
    if mode == "live" and payload.get("operator_ack") != "INITIALIZE":
        raise HTTPException(status_code=409, detail="operator_ack INITIALIZE required for live OEM initialCheck")
    program = _get_oem_startup_program(dry_safe=(mode != "live" or _tester is None))
    if hasattr(program.hardware, "initial_check"):
        return program.hardware.initial_check(mode=mode if _tester is not None else "dry_run")
    raise HTTPException(status_code=503, detail="OEM initialCheck provider unavailable")


@app.get("/oem/motion_worker/status")
async def oem_motion_worker_status():
    return _get_oem_startup_program().worker_status()


@app.post("/oem/motion_worker/run_next")
async def oem_motion_worker_run_next():
    result = _get_oem_startup_program().run_next_worker_command()
    return {"ok": result is not None, "result": result, "status": _get_oem_startup_program().worker_status()}


@app.post("/oem/motion_worker/abort")
async def oem_motion_worker_abort(reason: str = "operator abort"):
    return _get_oem_startup_program().abort_worker(reason=reason)


@app.post("/oem/switch_audit")
async def oem_switch_audit(req: OemSwitchAuditRequest):
    payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    mode = payload.get("mode", "status")
    artifact_root = payload.get("artifact_root") or os.environ.get("BIOXP_OEM_STARTUP_ARTIFACT_BASE")
    hardware = _BioXpSwitchAuditHardware(_get_tester()) if _tester is not None else FakeSwitchAuditHardware()
    result = run_switch_audit(hardware, axes=payload.get("axes") or ["x", "y", "z", "g", "door"], mode=mode, artifact_root=artifact_root)
    if not result.get("ok"):
        raise HTTPException(status_code=409, detail=result.get("error"))
    return result


def _get_vision_capabilities() -> CapabilityRegistry:
    return _vision_capabilities


def _require_local_maintenance_client(request: Request) -> None:
    host = request.client.host if request.client else ""
    if host not in {"127.0.0.1", "::1", "localhost", "testclient"}:
        raise HTTPException(status_code=403, detail="USB maintenance endpoints are localhost-only")


class AxisName(str, Enum):
    X = "x"
    Y = "y"
    Z = "z"
    GRIPPER = "g"
    THERMAL_DOOR = "door"


_LIQUID_REQUIRED_REFERENCE_AXES = (AxisName.X, AxisName.Y, AxisName.Z)


def _liquid_reference_preflight(operation: str, command: Any | None = None) -> dict[str, Any]:
    snapshot = _reference_state_store.snapshot(_LIQUID_REQUIRED_REFERENCE_AXES)
    rows = snapshot.get("rows", {}) if isinstance(snapshot, dict) else {}
    missing_axes = []
    for axis in _LIQUID_REQUIRED_REFERENCE_AXES:
        axis_key = axis.value
        row = rows.get(axis_key, {}) if isinstance(rows, dict) else {}
        if not isinstance(row, dict) or row.get("state") != "referenced":
            missing_axes.append(axis_key)
    requested = command.to_payload() if hasattr(command, "to_payload") else None
    payload = {
        "ok": not missing_axes,
        "operation": str(operation),
        "required_reference_axes": [axis.value for axis in _LIQUID_REQUIRED_REFERENCE_AXES],
        "missing_reference_axes": missing_axes,
        "reference_snapshot": snapshot,
        "hardware_truth_required": True,
        "requested": requested,
    }
    if missing_axes:
        raise PipettePreflightError(
            "Liquid operation requires referenced x/y/z axes before hardware pipetting.",
            details=payload,
        )
    return payload


class ThermalBankName(str, Enum):

    NEST = "nest"
    LID = "lid"
    PEDESTAL = "pedestal"


class ChillerBankName(str, Enum):
    RC = "rc"
    OC = "oc"


class MotionArtifactRequest(BaseModel):
    capture_bundle: bool = Field(
        False,
        description=(
            "If true, write a supervised validation bundle under "
            "/mnt/BioModStack/bms_results/bioxp_validation/."
        ),
    )
    dry_run_bundle: bool = Field(
        False,
        description=(
            "If true with capture_bundle, skip hardware I/O and only materialize the validation bundle."
        ),
    )
    operator_note: Optional[str] = Field(
        None,
        max_length=2000,
        description="Optional operator note to include in the validation bundle metadata.",
    )
    snapshot_refs: list[str] = Field(
        default_factory=list,
        description="Existing snapshot/image references to attach to the validation bundle.",
    )


class MoveRelativeRequest(MotionArtifactRequest):
    axis: AxisName
    steps: int = Field(..., description="Relative target in motor steps")
    wait_timeout_s: float = Field(12.0, gt=0.1, le=60.0)
    reuse_prepared: bool = Field(
        False,
        description=(
            "Debug-only compatibility flag. Ignored unless "
            "BIOXP_ENABLE_PREP_REUSE_DEBUG=1 and strict-arm/live state is healthy."
        ),
    )


class MoveAbsoluteRequest(MotionArtifactRequest):
    axis: AxisName
    position_steps: int = Field(..., description="Absolute target in motor steps")
    wait_timeout_s: float = Field(12.0, gt=0.1, le=60.0)


class HomeAxisRequest(MotionArtifactRequest):
    axis: AxisName
    speed: Optional[int] = Field(None, gt=0)
    timeout_s: float = Field(15.0, gt=0.1, le=90.0)


class ReferenceMarkRequest(BaseModel):
    axis: AxisName
    position_steps: int = Field(0, description="Controller position to treat as the trusted reference origin.")
    source: str = Field("manual", max_length=120)
    note: Optional[str] = Field(None, max_length=2000)


class ReferenceDesyncRequest(BaseModel):
    axis: AxisName
    reason: str = Field(..., min_length=1, max_length=2000)
    source: str = Field("manual", max_length=120)


class MotionHardResetRequest(BaseModel):
    rounds: int = Field(2, ge=1, le=5)


class MotionArmStartupRequest(BaseModel):
    run_homing: bool = False


class OemStartupStepRequest(BaseModel):
    step: str = Field(..., pattern=r"^(z-home|gripper-clear|gripper-home|x-home|x-park-6000|y-home|door-home|y-set-home)$")
    timeout_s: float = Field(25.0, gt=0.1, le=90.0)


class MotionAxisCurrentRequest(BaseModel):
    axes: list[AxisName] = Field(default_factory=lambda: [AxisName.X, AxisName.Y, AxisName.Z])
    run_current: int = Field(31, ge=0, le=31)
    standby_current: int = Field(31, ge=0, le=31)


class LiquidLocationRequest(BaseModel):
    location_id: str = Field(..., min_length=1, max_length=120)
    well_id: Optional[str] = Field(None, max_length=120)
    plate_name: Optional[str] = Field(None, max_length=120)
    z_offset_steps: Optional[int] = None


class PipetteInitRequest(BaseModel):
    pressure_profile: str = Field("1R", min_length=1, max_length=2, pattern=r"^[A-Za-z0-9]+$")
    prime_volume_ul: Optional[float] = Field(None, gt=0.0, le=1000.0)


class PipetteTipRequest(BaseModel):
    action: PipetteTipAction
    tip_id: Optional[str] = Field(None, max_length=120)
    operator: Optional[str] = Field(None, max_length=120)
    metadata: dict[str, Any] = Field(default_factory=dict)


class PipetteAspirateRequest(BaseModel):
    volume_ul: float = Field(..., gt=0.0, le=1000.0)
    pressure_profile: str = Field("1R", min_length=1, max_length=2, pattern=r"^[A-Za-z0-9]+$")
    source: Optional[LiquidLocationRequest] = None
    liquid_class: Optional[str] = Field(None, max_length=120)
    tip_id: Optional[str] = Field(None, max_length=120)
    air_gap_ul: Optional[float] = Field(None, ge=0.0, le=1000.0)
    operator: Optional[str] = Field(None, max_length=120)
    metadata: dict[str, Any] = Field(default_factory=dict)


class PipetteDispenseRequest(BaseModel):
    volume_ul: float = Field(..., gt=0.0, le=1000.0)
    pressure_profile: str = Field("1R", min_length=1, max_length=2, pattern=r"^[A-Za-z0-9]+$")
    blow_out: bool = False
    destination: Optional[LiquidLocationRequest] = None
    dest: Optional[LiquidLocationRequest] = None
    liquid_class: Optional[str] = Field(None, max_length=120)
    tip_id: Optional[str] = Field(None, max_length=120)
    air_gap_ul: Optional[float] = Field(None, ge=0.0, le=1000.0)
    operator: Optional[str] = Field(None, max_length=120)
    metadata: dict[str, Any] = Field(default_factory=dict)


class PipetteMixRequest(BaseModel):
    volume_ul: float = Field(..., gt=0.0, le=1000.0)
    cycles: int = Field(..., ge=1, le=50)
    pressure_profile: str = Field("1R", min_length=1, max_length=2, pattern=r"^[A-Za-z0-9]+$")
    location: Optional[LiquidLocationRequest] = None
    source: Optional[LiquidLocationRequest] = None
    destination: Optional[LiquidLocationRequest] = None
    dest: Optional[LiquidLocationRequest] = None
    liquid_class: Optional[str] = Field(None, max_length=120)
    tip_id: Optional[str] = Field(None, max_length=120)
    operator: Optional[str] = Field(None, max_length=120)
    metadata: dict[str, Any] = Field(default_factory=dict)


class ThermalRequest(BaseModel):
    bank: ThermalBankName = Field(..., description="nest, lid, or pedestal")
    target_temp_c: float = Field(..., ge=0.0, le=120.0)


class ChillerRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    target_temp_c: float = Field(..., ge=-20.0, le=40.0)


class CameraSnapshotRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")


class CameraHealthRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")
    seconds: int = Field(5, ge=1, le=30)


class CameraRecoverRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")
    max_resets: int = Field(2, ge=0, le=5)


class CameraControlRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")
    cid: int = Field(..., ge=0)
    value: int = Field(..., description="Raw V4L2 control value")


class InspectionRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")
    location_id: Optional[str] = Field(None, max_length=120)
    requested_checks: list[str] = Field(default_factory=list)
    include_image_data: bool = True


class BarcodeReadRequest(BaseModel):
    device: str = Field("/dev/video0", description="Preferred V4L2 device")
    location_id: Optional[str] = Field(None, max_length=120)
    symbologies: list[str] = Field(default_factory=list)
    include_image_data: bool = False


class ProtocolCompileRequest(BaseModel):
    source_type: str = Field("native", pattern=r"^(native|oem_xml)$")
    document: Optional[dict[str, Any]] = None
    xml_path: Optional[str] = None


class ProtocolExecuteRequest(ProtocolCompileRequest):
    dry_run: bool = True
    live_execution: Optional[dict[str, Any]] = Field(
        None,
        description="Required contract block for dry_run=false live execution: operator ack, deck manifest, preflight, and artifact refs.",
    )
    live_execution_ack: bool = Field(False, description="Explicit operator acknowledgement for dry_run=false live protocol execution.")
    operator_id: Optional[str] = Field(None, max_length=120)
    physical_console_verified: bool = Field(False)
    deck_manifest: Optional[dict[str, Any]] = None
    preflight: Optional[dict[str, Any]] = None
    artifact_refs: list[str] = Field(default_factory=list)
    snapshot_refs: list[str] = Field(default_factory=list)


class ProtocolReviewRequest(BaseModel):
    reviewer: str = Field("operator", min_length=1, max_length=120)
    note: Optional[str] = Field(None, max_length=4000)


class LedRgbRequest(BaseModel):
    r: int = Field(..., ge=0, le=255)
    g: int = Field(..., ge=0, le=255)
    b: int = Field(..., ge=0, le=255)
    reconnect_first: bool = True


class LedIntensityRequest(BaseModel):
    pct: int = Field(..., ge=0, le=100)


class ThermalFanRequest(BaseModel):
    speed: int = Field(..., ge=0, le=255)


class ThermalPwmRequest(BaseModel):
    bank: ThermalBankName = Field(..., description="nest or lid")
    pwm: int = Field(..., ge=0, le=100)


class ThermalRatesRequest(BaseModel):
    bank: ThermalBankName = Field(..., description="nest or lid")
    cool_rate_c_s: float = Field(..., ge=-2.0, le=0.0)
    heat_rate_c_s: float = Field(..., ge=0.0, le=2.0)


class ChillerFanRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    speed: int = Field(..., ge=0, le=255)


class ChillerPwmRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    pwm: int = Field(..., ge=0, le=100)


class ChillerRatesRequest(BaseModel):
    bank: ChillerBankName = Field(..., description="rc or oc")
    cool_rate_c_s: float = Field(..., ge=-2.0, le=0.0)
    heat_rate_c_s: float = Field(..., ge=0.0, le=2.0)


_THERMAL_BANK_MAP = {
    ThermalBankName.NEST: BioXpTester.THERMAL_BANK_NEST,
    ThermalBankName.LID: BioXpTester.THERMAL_BANK_LID,
}

_CHILLER_BANK_MAP = {
    ChillerBankName.RC: BioXpTester.CHILLER_BANK_RC,
    ChillerBankName.OC: BioXpTester.CHILLER_BANK_OC,
}

_DEFAULT_MOTION_SPEED = 100
_DEFAULT_MOTION_ACC = 50
_MOTION_NO_DELTA_TIMEOUT_S = 2.0
_MOTION_HOME_PRECLEAR_STEPS = 500
_MOTION_PREP_REUSE_DEBUG_ENV = "BIOXP_ENABLE_PREP_REUSE_DEBUG"


def _axis_preset(tester: BioXpTester, axis: AxisName):
    preset = tester.motor_function_preset(axis.value)
    if not isinstance(preset, dict):
        raise HTTPException(status_code=404, detail=f"Unknown axis preset for {axis.value}")
    out = dict(preset)
    out["speed"] = _DEFAULT_MOTION_SPEED
    out["acc"] = _DEFAULT_MOTION_ACC
    return out


def _switch_activity_from_switches(tester: BioXpTester, board: int, motor: int, switches: Optional[dict]) -> dict:
    left = None
    right = None
    if isinstance(switches, dict):
        left = switches.get("left_state")
        right = switches.get("right_state")
    active_val = int(tester.MOTOR_SWITCH_ACTIVE_VALUE)
    return {
        "board": int(board),
        "motor": int(motor),
        "left_state": left,
        "right_state": right,
        "left_active": None if left is None else (int(left) == active_val),
        "right_active": None if right is None else (int(right) == active_val),
        "switches": switches,
    }


def _axis_status_payload(tester: BioXpTester, axis: AxisName, *, include_current: bool = True) -> dict:
    preset = _axis_preset(tester, axis)
    board = int(preset["board"])
    motor = int(preset["motor"])
    switches = tester.motor_get_switches(board, motor=motor)
    status = {
        "board": board,
        "motor": motor,
        "position": tester.motor_get_position(board, motor=motor),
        "speed": tester.motor_get_speed(board, motor=motor),
        "switches": switches,
    }
    if include_current:
        status["max_current"] = tester.motor_get_axis_param(board, 6, motor=motor)
    return {
        "axis": axis.value,
        "preset": preset,
        "status": status,
        "switch_activity": _switch_activity_from_switches(tester, board, motor, switches),
    }


def _parse_axes_csv(axes_csv: str) -> list[AxisName]:
    parsed: list[AxisName] = []
    seen: set[AxisName] = set()
    for raw in str(axes_csv).split(","):
        token = raw.strip().lower()
        if not token:
            continue
        try:
            axis = AxisName(token)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=f"Unknown axis '{token}'.") from exc
        if axis in seen:
            continue
        parsed.append(axis)
        seen.add(axis)
    if not parsed:
        raise HTTPException(status_code=400, detail="At least one axis must be requested.")
    return parsed


def _axis_status_batch_payload(tester: BioXpTester, axes: list[AxisName]) -> dict:
    rows = {}
    for axis in axes:
        rows[axis.value] = _axis_status_payload(tester, axis, include_current=False)
    return {
        "axes": [axis.value for axis in axes],
        "rows": rows,
    }


def _position_value(row: Optional[dict]) -> Optional[int]:
    if not isinstance(row, dict):
        return None
    value = row.get("position")
    return int(value) if isinstance(value, int) else None


def _speed_value(row: Optional[dict]) -> Optional[int]:
    if not isinstance(row, dict):
        return None
    value = row.get("speed")
    return int(value) if isinstance(value, int) else None


def _position_delta(before: Optional[dict], after: Optional[dict]) -> Optional[int]:
    left = _position_value(before)
    right = _position_value(after)
    if left is None or right is None:
        return None
    return int(right) - int(left)


def _env_flag(name: str) -> bool:
    value = str(os.environ.get(name, "")).strip().lower()
    return value in {"1", "true", "yes", "on"}


def _dry_run_motion_response(command: str, axis: AxisName, **kwargs) -> dict:
    return service_dry_run_motion_response(command, axis, **kwargs)


def _motion_truth_payload() -> dict:
    return {
        "evidence_level": "controller_only",
        "controller_reported_position": True,
        "controller_reported_switches": True,
        "physical_motion_confirmed": False,
        "independent_evidence_required": True,
        "summary": (
            "Controller-reported motion only. Capture paired images and/or an operator note "
            "before treating this as confirmed physical displacement."
        ),
        "recommended_next_evidence": [
            "capture before/after images",
            "record operator confirmation",
        ],
    }


def _motion_prep_policy(
    *,
    axis: AxisName,
    reuse_requested: bool,
    armed_and_live: bool,
    debug_flag_enabled: bool,
    reuse_used: bool,
    interlock_reused: bool,
) -> dict:
    if axis is AxisName.THERMAL_DOOR:
        note = "Thermal door axis always uses fresh prep."
    elif reuse_used:
        note = (
            f"Prepared axis reuse allowed by debug flag {_MOTION_PREP_REUSE_DEBUG_ENV}=1; "
            "board activation still executed."
        )
    elif reuse_requested and not debug_flag_enabled:
        note = (
            f"reuse_prepared was requested but ignored because {_MOTION_PREP_REUSE_DEBUG_ENV}=1 "
            "is not enabled."
        )
    elif reuse_requested and not armed_and_live:
        note = "reuse_prepared was requested but ignored because strict-arm/live state is not healthy."
    elif interlock_reused:
        note = "Strict-arm/live gate reused only for interlock wake; board activation and axis prep still ran."
    else:
        note = "Fresh board activation and axis prep executed."
    return {
        "axis": axis.value,
        "reuse_requested": bool(reuse_requested),
        "armed_and_live": bool(armed_and_live),
        "debug_flag_required": True,
        "debug_flag_enabled": bool(debug_flag_enabled),
        "reuse_allowed": bool(reuse_requested and debug_flag_enabled and armed_and_live and axis is not AxisName.THERMAL_DOOR),
        "reuse_used": bool(reuse_used),
        "interlock_reused": bool(interlock_reused),
        "board_activation_skipped": False,
        "axis_prep_skipped": bool(reuse_used),
        "note": note,
    }


def _guard_direction(axis: AxisName, steps: int, switch_activity: Optional[dict], preset: Optional[dict] = None) -> None:
    if not isinstance(switch_activity, dict) or steps == 0:
        return
    # Simultaneous left+right assertions are not physically credible as hard-stop
    # data on this instrument; treat that as unreliable raw switch state.
    if switch_activity.get("left_active") is True and switch_activity.get("right_active") is True:
        return
    left_masked = bool((preset or {}).get("disable_left", False))
    right_masked = bool((preset or {}).get("disable_right", False))
    if steps < 0 and switch_activity.get("left_active") is True and not left_masked:
        raise HTTPException(status_code=409, detail=f"Axis {axis.value} negative travel blocked by active left limit.")
    if steps > 0 and switch_activity.get("right_active") is True and not right_masked:
        raise HTTPException(status_code=409, detail=f"Axis {axis.value} positive travel blocked by active right limit.")


def _guard_absolute_target(
    axis: AxisName,
    current_position: Optional[dict],
    target_steps: int,
    switch_activity: Optional[dict],
    preset: Optional[dict] = None,
) -> None:
    current = _position_value(current_position)
    if current is None:
        return
    _guard_direction(axis, int(target_steps) - int(current), switch_activity, preset)


def _wait_for_motion_with_guardrails(
    tester: BioXpTester,
    board: int,
    motor: int,
    timeout_s: float,
    *,
    no_delta_timeout_s: float = _MOTION_NO_DELTA_TIMEOUT_S,
    poll_s: float = 0.10,
    require_seen_nonzero: bool = True,
) -> dict:
    started = time.monotonic()
    deadline = started + max(0.5, float(timeout_s))
    position_row = tester.motor_get_position(board, motor=motor)
    last_position = _position_value(position_row)
    last_progress_at = started
    seen_nonzero = False
    polls = 0
    log_tail = []

    while True:
        speed_row = tester.motor_get_speed(board, motor=motor)
        position_row = tester.motor_get_position(board, motor=motor)
        switch_row = tester.motor_get_switch_activity(board, motor=motor)
        now = time.monotonic()
        speed = _speed_value(speed_row)
        position = _position_value(position_row)
        polls += 1
        if isinstance(speed, int) and speed != 0:
            seen_nonzero = True

        if position is not None and last_position is None:
            last_position = position
            last_progress_at = now
        elif position is not None and last_position is not None and position != last_position:
            last_position = position
            last_progress_at = now

        log_tail.append({"elapsed_ms": int((now - started) * 1000), "speed": speed, "position": position})
        if len(log_tail) > 20:
            log_tail = log_tail[-20:]

        if speed == 0 and polls >= 3:
            if bool(require_seen_nonzero) and not seen_nonzero:
                stop = tester.motor_stop(board, motor=motor)
                return {
                    "ok": False,
                    "stopped": False,
                    "error": "motion command produced no nonzero speed before reporting stopped; treating as ambiguous/no physical motion.",
                    "elapsed_ms": int((now - started) * 1000),
                    "last_speed": speed,
                    "seen_nonzero": seen_nonzero,
                    "position_after": position_row,
                    "switch_activity_after": switch_row,
                    "stop": stop,
                    "log_tail": log_tail,
                }
            return {
                "ok": True,
                "stopped": True,
                "elapsed_ms": int((now - started) * 1000),
                "last_speed": speed,
                "seen_nonzero": seen_nonzero,
                "position_after": position_row,
                "switch_activity_after": switch_row,
                "log_tail": log_tail,
            }

        if now >= deadline or now - last_progress_at >= max(0.5, float(no_delta_timeout_s)):
            stop = tester.motor_stop(board, motor=motor)
            settle = tester.motor_wait_stopped(board, motor=motor, timeout_s=2.0, poll_s=0.06)
            return {
                "ok": False,
                "stopped": False,
                "error": "no position delta detected for 2.0s; motion aborted."
                if now - last_progress_at >= max(0.5, float(no_delta_timeout_s))
                else "motion timed out before the motor reported stop; motion aborted.",
                "elapsed_ms": int((now - started) * 1000),
                "last_speed": speed,
                "seen_nonzero": seen_nonzero,
                "position_after": position_row,
                "switch_activity_after": switch_row,
                "stop": stop,
                "settle": settle,
                "log_tail": log_tail,
            }

        time.sleep(max(0.02, float(poll_s)))


def _guarded_home_search(
    tester: BioXpTester,
    preset: dict,
    *,
    speed: int,
    timeout_s: float,
) -> dict:
    board = int(preset["board"])
    motor = int(preset["motor"])
    active_value = int(tester.MOTOR_SWITCH_ACTIVE_VALUE)
    effective_speed = max(1, min(int(speed), _DEFAULT_MOTION_SPEED))
    started = time.monotonic()
    deadline = started + max(2.0, float(timeout_s))
    position_before = tester.motor_get_position(board, motor=motor)
    home_before = tester.motor_query_home_switch(board, motor=motor)
    switch_before = tester.motor_get_switch_activity(board, motor=motor)

    preclear = None
    preclear_wait = None
    home_cleared = None
    if home_before.get("value") == active_value:
        preclear = tester.motor_move_relative(board, _MOTION_HOME_PRECLEAR_STEPS, motor=motor)
        if not preclear.get("ok"):
            raise HTTPException(status_code=409, detail=f"Axis {preset['label']} home preclear command failed.")
        preclear_wait = _wait_for_motion_with_guardrails(tester, board, motor, timeout_s=6.0)
        if not preclear_wait.get("ok"):
            raise HTTPException(status_code=409, detail=preclear_wait.get("error"))
        home_cleared = tester.motor_query_home_switch(board, motor=motor)
        if home_cleared.get("value") == active_value:
            raise HTTPException(status_code=409, detail=f"Axis {preset['label']} home switch stayed active after preclear; refusing to home.")

    sethome_init = tester.motor_set_home(board, motor=motor)
    move_left = tester.motor_move_left(board, speed=effective_speed, motor=motor)
    if not move_left.get("ok"):
        raise HTTPException(status_code=409, detail=f"Axis {preset['label']} homing command failed.")

    last_position = _position_value(position_before)
    last_progress_at = time.monotonic()
    polls = []
    home_hit = None
    while time.monotonic() < deadline:
        home_row = tester.motor_query_home_switch(board, motor=motor)
        speed_row = tester.motor_get_speed(board, motor=motor)
        position_row = tester.motor_get_position(board, motor=motor)
        switch_row = tester.motor_get_switch_activity(board, motor=motor)
        now = time.monotonic()
        position = _position_value(position_row)
        speed_now = _speed_value(speed_row)
        if position is not None and last_position is None:
            last_position = position
            last_progress_at = now
        elif position is not None and last_position is not None and position != last_position:
            last_position = position
            last_progress_at = now
        polls.append(
            {
                "elapsed_ms": int((now - started) * 1000),
                "home": home_row.get("value"),
                "speed": speed_now,
                "position": position,
                "left_active": switch_row.get("left_active"),
                "right_active": switch_row.get("right_active"),
            }
        )
        if len(polls) > 20:
            polls = polls[-20:]
        if home_row.get("value") == active_value:
            home_hit = home_row
            break
        if now - last_progress_at >= _MOTION_NO_DELTA_TIMEOUT_S:
            tester.motor_stop(board, motor=motor)
            tester.motor_wait_stopped(board, motor=motor, timeout_s=2.0, poll_s=0.06)
            raise HTTPException(status_code=409, detail=f"Axis {preset['label']} homing aborted after 2.0s with no position change.")
        time.sleep(0.08)

    stop = tester.motor_stop(board, motor=motor)
    wait = tester.motor_wait_stopped(board, motor=motor, timeout_s=2.0, poll_s=0.06)
    if home_hit is None:
        raise HTTPException(status_code=409, detail=f"Axis {preset['label']} homing timed out before the home switch triggered.")

    sethome_final = tester.motor_set_home(board, motor=motor)
    home_after = tester.motor_query_home_switch(board, motor=motor)
    position_after = tester.motor_get_position(board, motor=motor)
    return {
        "board": board,
        "motor": motor,
        "speed": effective_speed,
        "acc": int(preset["acc"]),
        "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
        "position_before": position_before,
        "position_after": position_after,
        "position_delta": _position_delta(position_before, position_after),
        "switch_activity_before": switch_before,
        "switch_activity_after": tester.motor_get_switch_activity(board, motor=motor),
        "home_before": home_before,
        "home_after": home_after,
        "preclear": preclear,
        "preclear_wait": preclear_wait,
        "home_cleared": home_cleared,
        "sethome_init": sethome_init,
        "move_left": move_left,
        "home_hit": home_hit,
        "stop": stop,
        "wait": wait,
        "sethome_final": sethome_final,
        "elapsed_ms": int((time.monotonic() - started) * 1000),
        "log_tail": polls,
    }


def _execute_relative_move(
    tester: BioXpTester,
    axis: AxisName,
    steps: int,
    wait_timeout_s: float,
    *,
    reuse_prepared: bool = False,
) -> dict:
    preset, board_status, interlock, prep, prep_policy = _prepare_motion_axis(tester, axis, reuse_prepared=reuse_prepared)
    position_before = tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_before = tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    _guard_direction(axis, steps, switch_before, preset)
    move = tester.motor_move_relative(preset["board"], steps, motor=preset["motor"])
    if not move.get("ok"):
        raise HTTPException(status_code=409, detail=f"Axis {axis.value} relative move command failed.")
    wait = _wait_for_motion_with_guardrails(tester, preset["board"], preset["motor"], timeout_s=wait_timeout_s)
    if not wait.get("ok"):
        raise HTTPException(status_code=409, detail=wait.get("error"))
    position_after = wait.get("position_after") or tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_after = wait.get("switch_activity_after") or tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    return {
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "prep_policy": prep_policy,
        "motion_truth": _motion_truth_payload(),
        "motion_profile": {
            "speed": int(preset["speed"]),
            "acc": int(preset["acc"]),
            "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
        },
        "position_before": position_before,
        "position_after": position_after,
        "position_delta": _position_delta(position_before, position_after),
        "switch_activity_before": switch_before,
        "switch_activity_after": switch_after,
        "move": move,
        "wait": wait,
    }


def _execute_absolute_move(tester: BioXpTester, axis: AxisName, position_steps: int, wait_timeout_s: float) -> dict:
    preset, board_status, interlock, prep, prep_policy = _prepare_motion_axis(tester, axis)
    position_before = tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_before = tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    _guard_absolute_target(axis, position_before, position_steps, switch_before, preset)
    move = tester.motor_move_absolute(preset["board"], position_steps, motor=preset["motor"])
    if not move.get("ok"):
        raise HTTPException(status_code=409, detail=f"Axis {axis.value} absolute move command failed.")
    wait = _wait_for_motion_with_guardrails(tester, preset["board"], preset["motor"], timeout_s=wait_timeout_s)
    if not wait.get("ok"):
        raise HTTPException(status_code=409, detail=wait.get("error"))
    position_after = wait.get("position_after") or tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_after = wait.get("switch_activity_after") or tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    return {
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "prep_policy": prep_policy,
        "motion_truth": _motion_truth_payload(),
        "motion_profile": {
            "speed": int(preset["speed"]),
            "acc": int(preset["acc"]),
            "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
        },
        "position_before": position_before,
        "position_after": position_after,
        "position_delta": _position_delta(position_before, position_after),
        "target_position": int(position_steps),
        "switch_activity_before": switch_before,
        "switch_activity_after": switch_after,
        "move": move,
        "wait": wait,
    }


def _unwrap_oem_home_payload(home_payload: object) -> dict:
    if isinstance(home_payload, dict) and isinstance(home_payload.get("home"), dict):
        return home_payload["home"]
    if isinstance(home_payload, dict):
        return home_payload
    return {}


def _validate_oem_home_request_speed(tester: BioXpTester, axis: AxisName, speed: Optional[int]) -> Optional[int]:
    if speed is None:
        return None
    effective_speed = int(speed)
    profile = tester._motion_oem_axis_profile(axis.value)
    max_home_speed = int(profile.get("home_speed", profile.get("speed", 250)))
    if effective_speed < 1 or effective_speed > max_home_speed:
        raise HTTPException(
            status_code=422,
            detail=(
                f"Axis {axis.value} home speed must be between 1 and {max_home_speed} "
                f"steps/s to stay within the OEM homing profile."
            ),
        )
    return effective_speed


def _ensure_oem_home_succeeded(tester: BioXpTester, axis: AxisName, home_payload: object) -> None:
    home = _unwrap_oem_home_payload(home_payload)
    if home.get("ok") is True:
        return
    home_after = home.get("home_after")
    home_state = None
    if isinstance(home_after, dict):
        home_state = home_after.get("value")
    raise HTTPException(
        status_code=409,
        detail=(
            f"Axis {axis.value} OEM homing did not confirm the home switch after motion; "
            f"reported home_after={home_state!r}. Refusing to mark the axis homed."
        ),
    )


def _execute_home_axis(tester: BioXpTester, axis: AxisName, speed: Optional[int], timeout_s: float) -> dict:
    preset, board_status, interlock, prep, prep_policy = _prepare_motion_axis(tester, axis)
    effective_speed = _validate_oem_home_request_speed(tester, axis, speed)
    home = tester.motor_oem_home_axis(
        axis.value,
        speed=effective_speed,
        timeout_s=min(float(timeout_s), 20.0),
    )
    _ensure_oem_home_succeeded(tester, axis, home)
    return {
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "prep_policy": prep_policy,
        "motion_truth": _motion_truth_payload(),
        "motion_profile": {
            "requested_speed": effective_speed,
            "preset_speed": int(preset["speed"]),
            "acc": int(preset["acc"]),
            "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
            "vendor_path": "oem_home_axis",
        },
        "home": home,
    }


def _current_param_row(row: dict | None) -> dict | None:
    if not isinstance(row, dict):
        return row
    out = {k: row.get(k) for k in ("board", "param", "motor", "value", "set_value", "ok") if k in row}
    ack = row.get("ack")
    if isinstance(ack, dict):
        out["ack"] = {k: ack.get(k) for k in ("board", "cmd", "status", "status_str", "value") if k in ack}
    rb = row.get("readback")
    if isinstance(rb, dict):
        out["readback"] = {k: rb.get(k) for k in ("board", "param", "motor", "value") if k in rb}
        rb_ack = rb.get("ack")
        if isinstance(rb_ack, dict):
            out["readback"]["ack"] = {
                k: rb_ack.get(k) for k in ("board", "cmd", "status", "status_str", "value") if k in rb_ack
            }
    return out


def _set_motion_axis_currents(
    tester: BioXpTester,
    axes: list[AxisName],
    *,
    run_current: int,
    standby_current: int,
) -> dict:
    allowed = {AxisName.X, AxisName.Y, AxisName.Z}
    normalized = [AxisName(axis) for axis in axes]
    invalid = [axis.value for axis in normalized if axis not in allowed]
    if invalid:
        raise HTTPException(status_code=400, detail=f"current set route is restricted to gantry x/y/z; invalid={invalid}")
    run = max(0, min(31, int(run_current)))
    standby = max(0, min(run, int(standby_current)))
    rows = {}
    all_ok = True
    for axis in normalized:
        preset = _axis_preset(tester, axis)
        board = int(preset["board"])
        motor = int(preset["motor"])
        before6 = tester.motor_get_axis_param(board, 6, motor=motor)
        before7 = tester.motor_get_axis_param(board, 7, motor=motor)
        # Write standby first; live testing showed writing param 7 can drag param 6 down.
        # Re-assert run current after standby so requested run/standby pairs survive readback.
        write7 = tester.motor_set_axis_param(board, 7, standby, motor=motor)
        write6 = tester.motor_set_axis_param(board, 6, run, motor=motor)
        after6 = tester.motor_get_axis_param(board, 6, motor=motor)
        after7 = tester.motor_get_axis_param(board, 7, motor=motor)
        speed = tester.motor_get_speed(board, motor=motor)
        ok = (
            bool(write6.get("ok"))
            and bool(write7.get("ok"))
            and after6.get("value") == run
            and after7.get("value") == standby
        )
        all_ok = all_ok and ok
        rows[axis.value] = {
            "label": preset.get("label"),
            "board": board,
            "motor": motor,
            "requested": {"run_current_param6": int(run_current), "standby_current_param7": int(standby_current)},
            "applied": {"run_current_param6": run, "standby_current_param7": standby},
            "before": {"param6": _current_param_row(before6), "param7": _current_param_row(before7)},
            "writes": {"param6": _current_param_row(write6), "param7": _current_param_row(write7)},
            "after": {"param6": _current_param_row(after6), "param7": _current_param_row(after7)},
            "speed": speed,
            "ok": ok,
        }
    return {
        "ok": bool(all_ok),
        "axes": rows,
        "current_param_bounds": (
            "0..31 controller-accepted range; controller returned Invalid value for 37 during live testing; "
            "standby is capped at run_current; no movement is commanded."
        ),
        "motion_commanded": False,
    }


def _prepare_motion_axis(tester: BioXpTester, axis: AxisName, *, reuse_prepared: bool = False):
    preset = _axis_preset(tester, axis)
    board_status = None
    interlock = None
    prep = None
    arm_state = {}
    live_gate = {}
    armed_and_live = False
    interlock_reused = False
    debug_flag_enabled = _env_flag(_MOTION_PREP_REUSE_DEBUG_ENV)
    reuse_requested = bool(reuse_prepared and axis is not AxisName.THERMAL_DOOR)
    if axis is not AxisName.THERMAL_DOOR:
        arm_state = tester.motion_arm_state()
        live_gate = tester.motion_gate_live_snapshot()
        armed_and_live = bool(arm_state.get("armed")) and bool(live_gate.get("ok"))
        board_status = tester.activate_boards(expect_reply=True)
        interlock = tester.motor_prepare_motion_interlock(force_lock=True)
        if isinstance(interlock, dict):
            interlock.setdefault("armed", bool(arm_state.get("armed")))
            interlock.setdefault("reason", arm_state.get("reason"))
            interlock["reused"] = False
            interlock["note"] = "Fresh interlock wake executed before motion to mirror OEM XYZ enable behavior."
            interlock["live_gate_before"] = live_gate
    if board_status is None:
        board_status = tester.activate_boards(expect_reply=True)
    reuse_used = bool(reuse_requested and armed_and_live and debug_flag_enabled)
    if prep is None:
        if reuse_used:
            prep = {
                "reused": True,
                "armed": True,
                "reason": arm_state.get("reason"),
                "note": (
                    f"strict-arm prepared profile reused for debug compatibility; "
                    f"board activation still executed because {_MOTION_PREP_REUSE_DEBUG_ENV}=1"
                ),
                "preset": preset,
                "elapsed_ms": 0,
            }
        else:
            prep = tester.motor_prepare_axis(
                preset["board"],
                motor=preset["motor"],
                run_current=preset["run_current"],
                standby_current=preset["standby_current"],
                speed=preset["speed"],
                acc=preset["acc"],
                stall_guard=preset.get("stall_guard"),
                ramp_mode=preset.get("ramp_mode"),
                disable_right=bool(preset.get("disable_right", False)),
                disable_left=bool(preset.get("disable_left", False)),
                rdiv=preset.get("rdiv"),
                pdiv=preset.get("pdiv"),
                warm_enable=bool(preset.get("warm_enable", False)),
            )
    prep_policy = _motion_prep_policy(
        axis=axis,
        reuse_requested=reuse_requested,
        armed_and_live=armed_and_live,
        debug_flag_enabled=debug_flag_enabled,
        reuse_used=reuse_used,
        interlock_reused=interlock_reused,
    )
    return preset, board_status, interlock, prep, prep_policy


def _hardware_connected_from_board_status(board_status: Any) -> bool:
    return bool(isinstance(board_status, dict) and any(reply is not None for reply in board_status.values()))



def _passive_board_status(tester: BioXpTester) -> Any:
    board_status = tester.activate_boards(expect_reply=True)
    if _hardware_connected_from_board_status(board_status):
        return board_status
    retry_board_status = tester.activate_boards(expect_reply=True)
    if _hardware_connected_from_board_status(retry_board_status):
        return retry_board_status
    return retry_board_status



def _dedicated_chiller_status_payload(tester: BioXpTester, board_status: Any) -> Optional[dict[str, Any]]:
    if not isinstance(board_status, dict):
        return None
    chiller_board = getattr(tester, "BOARD_CHILLER", 0x07)
    if board_status.get(chiller_board) is not None:
        return None

    try:
        activate = tester.chiller_activate()
    except Exception as exc:
        activate = {"ack": None, "ok": False, "error": str(exc)}

    try:
        firmware = tester.chiller_query_firmware()
    except Exception as exc:
        firmware = {"ack": None, "ok": False, "fw_hex": None, "error": str(exc)}

    return {
        "alive": bool(activate.get("ok") or firmware.get("ok")),
        "activate": activate,
        "firmware": firmware,
        "note": "Dedicated chiller probe; board_status[7] from activate_boards is advisory.",
    }



def _status_payload() -> dict:
    runtime_available = _tester is not None
    board_status = None
    chiller_status = None
    deck_io_snapshot = None
    status_error = None
    hardware_connected = False
    status = "degraded"

    if runtime_available:
        try:
            board_status = _passive_board_status(_tester)
            hardware_connected = _hardware_connected_from_board_status(board_status)
            status = "ok" if hardware_connected else "degraded"
            if hardware_connected:
                try:
                    deck_io_snapshot = _tester.io_snapshot(_tester.BOARD_DECK)
                except Exception as exc:
                    status_error = f"deck IO snapshot unavailable: {exc}"
            chiller_status = _dedicated_chiller_status_payload(_tester, board_status)
        except Exception as exc:
            status_error = str(exc)

    return {
        "status": status if runtime_available else "degraded",
        "transport": "usb",
        "runtime_available": runtime_available,
        "hardware_connected": hardware_connected,
        "startup_error": _startup_error,
        "status_error": status_error,
        "board_status": board_status,
        "chiller_status": chiller_status,
        "deck_io_snapshot": deck_io_snapshot,
    }


def _motion_power_status_payload(tester: BioXpTester) -> dict:
    board_status = _passive_board_status(tester)
    hardware_connected = _hardware_connected_from_board_status(board_status)
    chiller_status = _dedicated_chiller_status_payload(tester, board_status)
    deck_io_snapshot = tester.io_snapshot(tester.BOARD_DECK) if hardware_connected else None
    return {
        "hardware_connected": hardware_connected,
        "board_status": board_status,
        "chiller_status": chiller_status,
        "deck_io_snapshot": deck_io_snapshot,
        "rail_24v": tester.motor_query_24v_sensor(),
        "motion_arm": tester.motion_arm_state(),
        "latch_override": tester.motion_latch_override_state(),
    }


def _camera_snapshot_with_data(tester: BioXpTester, preferred: str) -> dict:
    result = _camera_capture_snapshot_direct(tester, preferred)
    image_b64 = None
    image_error = None
    path = result.get("path")
    if result.get("ok") and path and os.path.exists(path):
        try:
            with open(path, "rb") as handle:
                image_b64 = base64.b64encode(handle.read()).decode("ascii")
        except Exception as exc:
            image_error = str(exc)
    result["image_b64"] = image_b64
    result["image_error"] = image_error
    return result


async def _run_blocking(label: str, func, timeout_s: float = 30.0):
    async with _tester_lock:
        try:
            return await asyncio.wait_for(run_in_threadpool(func), timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise HTTPException(status_code=504, detail=f"{label} timed out after {timeout_s:.0f}s") from exc


def _pick_capture_device(tester: BioXpTester, preferred: str) -> dict:
    pick = tester.camera_wait_pick_device(
        preferred=preferred,
        timeout_s=4.0,
        poll_s=0.25,
        require_controls=True,
    )
    if not pick.get("ok"):
        return {
            "ok": False,
            "device": None,
            "error": pick.get("error") or "no capture-capable camera device found",
            "rows": pick.get("rows", []),
        }
    return pick


def _camera_device_rows_local() -> list[dict]:
    rows = []
    for name in sorted(os.listdir("/dev")):
        if not name.startswith("video"):
            continue
        device = f"/dev/{name}"
        label = ""
        try:
            with open(f"/sys/class/video4linux/{name}/name", "r", encoding="utf-8", errors="ignore") as handle:
                label = handle.read().strip()
        except OSError:
            label = ""
        rows.append({"device": device, "name": label})
    return rows


def _pick_stream_device(preferred: str) -> dict:
    rows = _camera_device_rows_local()
    if preferred and os.path.exists(preferred):
        return {"ok": True, "device": preferred, "rows": rows}
    for row in rows:
        if row["device"] == "/dev/video0":
            return {"ok": True, "device": row["device"], "rows": rows}
    if rows:
        return {"ok": True, "device": rows[0]["device"], "rows": rows}
    return {"ok": False, "device": None, "rows": [], "error": "no /dev/video* devices found"}


def _camera_reset_local(preferred: str) -> dict:
    device = preferred if preferred and os.path.exists(preferred) else "/dev/video0"
    ps = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    killed: list[int] = []
    errors: list[dict] = []
    for raw in (ps.stdout or "").splitlines():
        line = raw.strip()
        if not line or "ffmpeg" not in line or device not in line:
            continue
        parts = line.split(None, 1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            killed.append(pid)
        except ProcessLookupError:
            continue
        except Exception as exc:
            errors.append({"pid": pid, "error": str(exc)})

    if killed:
        time.sleep(0.5)
        for pid in killed:
            try:
                os.kill(pid, 0)
            except OSError:
                continue
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                continue
            except Exception as exc:
                errors.append({"pid": pid, "error": str(exc)})

    lock_released = False
    if _camera_stream_lock.locked():
        try:
            _camera_stream_lock.release()
            lock_released = True
        except RuntimeError:
            lock_released = False

    _camera_stream_state.update(
        {
            "active": False,
            "device": device,
            "last_error": "stream stopped",
            "last_frame_at": None,
        }
    )

    remaining = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    survivors = []
    for raw in (remaining.stdout or "").splitlines():
        line = raw.strip()
        if not line or "ffmpeg" not in line or device not in line:
            continue
        parts = line.split(None, 1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        survivors.append({"pid": pid, "cmd": parts[1] if len(parts) > 1 else ""})

    return {
        "ok": not survivors,
        "device": device,
        "killed_pids": killed,
        "lock_released": lock_released,
        "survivors": survivors,
        "errors": errors,
        "reset_provenance": _reset_provenance(
            subsystem="camera",
            source="camera_reset_local",
            reset_scope="ffmpeg_process_and_stream_lock",
            requested_device=preferred,
            resolved_device=device,
            hardware_usb_reset_performed=False,
            killed_pids=killed,
            lock_released=lock_released,
            survivor_count=len(survivors),
            errors=errors,
        ),
    }


def _camera_stream_state_payload() -> dict:
    last_frame_at = _camera_stream_state.get("last_frame_at")
    started_at = _camera_stream_state.get("started_at")
    return {
        **_camera_stream_state,
        "last_frame_age_s": None if not last_frame_at else round(max(0.0, time.time() - float(last_frame_at)), 2),
        "stream_age_s": None if not started_at else round(max(0.0, time.time() - float(started_at)), 2),
    }


def _camera_capture_snapshot_direct(tester: BioXpTester, preferred: str) -> dict:
    pick = _pick_capture_device(tester, preferred)
    if not pick.get("ok"):
        return {"ok": False, "error": pick.get("error"), "device": None, "path": None}

    device = pick["device"]
    out_dir = os.path.join(tempfile.gettempdir(), "bioxp-api-camera")
    os.makedirs(out_dir, exist_ok=True)
    ts = __import__("time").strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"snapshot_{ts}.jpg")
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "v4l2",
        "-i",
        device,
        "-frames:v",
        "1",
        out_path,
    ]
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "ffmpeg snapshot timeout", "device": device, "path": out_path}

    output = (proc.stdout or "").strip()
    exists = os.path.exists(out_path)
    size = os.path.getsize(out_path) if exists else 0
    ok = proc.returncode == 0 and exists and size > 0
    return {
        "ok": ok,
        "device": device,
        "path": out_path,
        "size": size,
        "rc": int(proc.returncode),
        "output": output,
        "error": None if ok else (output or "snapshot failed"),
        "pick": pick,
    }


def _camera_stream_health_direct(tester: BioXpTester, preferred: str, seconds: int) -> dict:
    if _camera_stream_lock.locked():
        return {
            "ok": False,
            "busy": True,
            "stream_active": True,
            "device": preferred,
            "error": "live stream is active; stop the stream before running stream health",
            "stream_state": _camera_stream_state_payload(),
        }

    pick = _pick_capture_device(tester, preferred)
    if not pick.get("ok"):
        return {"ok": False, "error": pick.get("error"), "device": None}

    device = pick["device"]
    dur = max(1, int(seconds))
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "info",
        "-f",
        "v4l2",
        "-i",
        device,
        "-t",
        str(dur),
        "-f",
        "null",
        "-",
    ]
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=float(dur) + 8.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "ffmpeg stream timeout", "device": device}

    output = proc.stdout or ""
    import re
    frames = [int(x) for x in re.findall(r"frame=\s*([0-9]+)", output)]
    fps_vals = [float(x) for x in re.findall(r"fps=\s*([0-9.]+)", output)]
    frame_max = max(frames) if frames else 0
    fps_last = fps_vals[-1] if fps_vals else None
    ok = proc.returncode == 0 and frame_max > 0
    return {
        "ok": ok,
        "device": device,
        "seconds": dur,
        "frames": frame_max,
        "fps_last": fps_last,
        "rc": int(proc.returncode),
        "error": None if ok else "stream health failed",
        "output_tail": output[-600:],
        "pick": pick,
    }


def _camera_devices_payload(tester: BioXpTester) -> dict:
    rows = tester.camera_device_rows()
    preferred = None
    pick = _pick_capture_device(tester, "/dev/video0")
    if pick.get("ok"):
        preferred = pick.get("device")
    for row in rows:
        row["capture_candidate"] = row.get("device") == preferred
    return {"ok": True, "rows": rows, "preferred_device": preferred}


async def _camera_mjpeg_response(
    tester: BioXpTester,
    preferred: str,
    fps: int,
    quality: int,
    width: int,
    height: int,
):
    fps = max(1, min(int(fps), 30))
    quality = max(2, min(int(quality), 15))
    width = max(160, min(int(width), 1920))
    height = max(120, min(int(height), 1080))
    if _camera_stream_lock.locked():
        await run_in_threadpool(_camera_reset_local, preferred)
        await asyncio.sleep(0.15)
    await _camera_stream_lock.acquire()
    proc = None
    try:
        pick = _pick_stream_device(preferred)
        if not pick.get("ok"):
            raise HTTPException(status_code=503, detail=pick.get("error") or "No capture-capable camera device found")

        device = pick["device"]
        _camera_stream_state.update(
            {
                "active": True,
                "device": device,
                "fps": fps,
                "quality": quality,
                "width": width,
                "height": height,
                "frames_emitted": 0,
                "started_at": time.time(),
                "last_frame_at": None,
                "last_error": None,
            }
        )
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-avioflags",
            "direct",
            "-f",
            "v4l2",
            "-input_format",
            "mjpeg",
            "-framerate",
            str(fps),
            "-video_size",
            f"{width}x{height}",
            "-i",
            device,
            "-an",
            "-vf",
            f"fps={fps}",
            "-q:v",
            str(quality),
            "-vcodec",
            "mjpeg",
            "-f",
            "image2pipe",
            "pipe:1",
        ]
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        if proc.stdout is None:
            raise HTTPException(status_code=500, detail="ffmpeg stream stdout unavailable")
        await asyncio.sleep(0.25)
        if proc.returncode is not None:
            stderr = b""
            if proc.stderr is not None:
                try:
                    stderr = await asyncio.wait_for(proc.stderr.read(), timeout=0.5)
                except asyncio.TimeoutError:
                    stderr = b""
            detail = stderr.decode("utf-8", errors="replace").strip() or "camera stream exited before producing frames"
            raise HTTPException(status_code=503, detail=detail)
    except Exception as exc:
        detail = exc.detail if isinstance(exc, HTTPException) else str(exc)
        _camera_stream_state.update({"active": False, "last_error": detail})
        if proc is not None and proc.returncode is None:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), timeout=3.0)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
        if _camera_stream_lock.locked():
            _camera_stream_lock.release()
        raise

    cleanup_started = False
    cleanup_guard = asyncio.Lock()

    async def cleanup():
        nonlocal cleanup_started
        async with cleanup_guard:
            if cleanup_started:
                return
            cleanup_started = True
            if proc is not None and proc.returncode is None:
                proc.terminate()
                try:
                    await asyncio.wait_for(proc.wait(), timeout=3.0)
                except asyncio.TimeoutError:
                    proc.kill()
                    await proc.wait()
            _camera_stream_state["active"] = False
            if _camera_stream_lock.locked():
                _camera_stream_lock.release()

    async def iterator():
        buffer = bytearray()
        soi = b"\xff\xd8"
        eoi = b"\xff\xd9"
        try:
            while True:
                if proc.stdout is None:
                    break
                chunk = await proc.stdout.read(16384)
                if not chunk:
                    break
                buffer.extend(chunk)
                while True:
                    start = buffer.find(soi)
                    if start == -1:
                        if len(buffer) > 65536:
                            buffer.clear()
                        break
                    if start > 0:
                        del buffer[:start]
                    end = buffer.find(eoi, 2)
                    if end == -1:
                        break
                    frame = bytes(buffer[: end + 2])
                    del buffer[: end + 2]
                    _camera_stream_state["frames_emitted"] = int(_camera_stream_state.get("frames_emitted") or 0) + 1
                    _camera_stream_state["last_frame_at"] = time.time()
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                    )
                    yield header + frame + b"\r\n"
        finally:
            await cleanup()

    headers = {
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
        "Pragma": "no-cache",
        "X-BioXp-Camera-Device": device,
    }
    return StreamingResponse(
        iterator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers=headers,
        background=BackgroundTask(cleanup),
    )


@app.get("/status")
async def get_status():
    return await _run_blocking("BioXP status", _status_payload, timeout_s=20.0)


@app.post("/reconnect")
async def reconnect_runtime():
    tester = _get_tester()
    await _run_blocking("BioXP reconnect", tester.reconnect, timeout_s=20.0)
    return {
        "ok": True,
        "message": "USB runtime reconnect requested.",
        **(await _run_blocking("BioXP status", _status_payload, timeout_s=20.0)),
    }


@app.post("/maintenance/usb/release")
async def maintenance_usb_release(request: Request):
    _require_local_maintenance_client(request)
    global _tester, _startup_error
    async with _tester_lock:
        tester = _tester
        if tester is not None:
            disconnect = getattr(tester, "_disconnect", None)
            if callable(disconnect):
                await asyncio.wait_for(run_in_threadpool(disconnect), timeout=20.0)
        _tester = None
        _startup_error = "BioXP USB runtime manually released for direct maintenance testing."
    return {"ok": True, "mode": "maintenance", "usb_owner": "released", "message": _startup_error}


@app.post("/maintenance/usb/reconnect")
async def maintenance_usb_reconnect(request: Request):
    _require_local_maintenance_client(request)
    global _tester, _startup_error
    async with _tester_lock:
        try:
            alt = int(os.environ.get("BIOXP_USB_ALT", "1"))
            _tester = await asyncio.wait_for(run_in_threadpool(lambda: BioXpTester(alt=alt)), timeout=20.0)
            _startup_error = None
        except Exception as exc:
            _tester = None
            _startup_error = str(exc)
            raise HTTPException(status_code=503, detail=_startup_error) from exc
    return {"ok": True, "mode": "maintenance", "usb_owner": "service", "message": "USB runtime reconnected."}


@app.get("/motion/axis/{axis}/status")
async def axis_status(axis: AxisName):
    tester = _get_tester()
    return await _run_blocking(
        f"Axis {axis.value} status",
        lambda: _axis_status_payload(tester, axis, include_current=True),
        timeout_s=20.0,
    )


@app.get("/motion/axes/status")
async def axes_status(axes: str = Query("x,y,z", description="Comma-separated axis names, e.g. x,y,z")):
    tester = _get_tester()
    requested_axes = _parse_axes_csv(axes)
    return await _run_blocking(
        "Axes status",
        lambda: _axis_status_batch_payload(tester, requested_axes),
        timeout_s=max(20.0, 5.0 * float(len(requested_axes))),
    )


@app.post("/motion/interlock/prepare")
async def prepare_interlock():
    tester = _get_tester()
    return await _run_blocking(
        "Motion interlock prep",
        lambda: (
            tester.activate_boards(expect_reply=True),
            tester.motor_prepare_motion_interlock(force_lock=True),
        )[1],
        timeout_s=25.0,
    )


@app.get("/motion/power/status")
async def motion_power_status():
    tester = _get_tester()
    return await _run_blocking(
        "Motion power status",
        lambda: _motion_power_status_payload(tester),
        timeout_s=25.0,
    )


@app.post("/motion/power/enable")
async def motion_power_enable():
    tester = _get_tester()
    return await _run_blocking(
        "Motion power enable",
        lambda: tester.motor_enable_sequence(conservative=True),
        timeout_s=35.0,
    )


@app.post("/motion/power/diag")
async def motion_power_diag():
    tester = _get_tester()
    return await _run_blocking(
        "Motion driver power diagnostic",
        lambda: tester.motor_driver_power_diag(axis_keys=("x", "y", "z", "g", "door")),
        timeout_s=45.0,
    )


@app.post("/motion/axes/current")
async def motion_axes_current(req: MotionAxisCurrentRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Set gantry motor currents",
        lambda: _set_motion_axis_currents(
            tester,
            req.axes,
            run_current=req.run_current,
            standby_current=req.standby_current,
        ),
        timeout_s=25.0,
    )


@app.post("/motion/arm/strict_startup")
async def motion_arm_strict_startup(req: MotionArmStartupRequest):
    if bool(req.run_homing):
        raise HTTPException(
            status_code=409,
            detail=(
                "Monolithic strict_startup run_homing is disabled after live testing showed wrong/hidden motion "
                "and USB-busy emergency-stop failure. Use scripts/bioxp_supervised_oem_startup_homing_stepwise.sh."
            ),
        )
    tester = _get_tester()
    response = await _run_blocking(
        "Motion strict startup",
        lambda: tester.motion_arm_strict_startup(run_homing=False),
        timeout_s=90.0,
    )
    return response


@app.post("/motion/hard_reset")
async def motion_hard_reset(req: MotionHardResetRequest):
    tester = _get_tester()
    response = await _run_blocking(
        "Motion hard reset",
        lambda: tester.motor_hard_reset(rounds=req.rounds),
        timeout_s=max(45.0, 20.0 * float(req.rounds)),
    )
    for axis in AxisName:
        _reference_state_store.mark_desynced(
            MarkAxisDesyncedCommand(
                axis=axis,
                reason="Motion hard reset executed; reference must be re-established.",
                source="motion_hard_reset",
                motion_kind="hard_reset",
            )
        )
    return response


@app.post("/motion/clear_lock")
async def clear_lock():
    tester = _get_tester()
    return await _run_blocking(
        "Head lock clear",
        lambda: (
            tester.activate_boards(expect_reply=True),
            tester.motor_ensure_head_clearance(
                force_rehome=True,
                ensure_interlock=True,
                preclear_abs=tester.MOTOR_HEAD_CLEARANCE_LIFT_ABS,
            ),
        )[1],
        timeout_s=45.0,
    )


@app.get("/latch/status")
async def latch_status():
    tester = _get_tester()
    snapshot = await _run_blocking(
        "Latch status",
        lambda: tester.io_snapshot(tester.BOARD_DECK),
        timeout_s=20.0,
    )
    return {
        "snapshot": snapshot,
        "rail_24v": snapshot.get(0),
        "door_sensor": snapshot.get(1),
        "solenoid_state": snapshot.get(2),
        "latch_sensor": snapshot.get(3),
    }


@app.post("/latch/lock")
async def latch_lock():
    tester = _get_tester()
    return await _run_blocking("Latch lock", lambda: tester.latch_oem(True), timeout_s=20.0)


@app.post("/latch/unlock")
async def latch_unlock():
    tester = _get_tester()
    return await _run_blocking("Latch unlock", lambda: tester.latch_oem(False), timeout_s=20.0)


@app.post("/led/off")
async def led_off():
    tester = _get_tester()
    return await _run_blocking("LED off", tester.strip_off, timeout_s=20.0)


@app.post("/led/on")
async def led_on():
    tester = _get_tester()
    return await _run_blocking("LED on", tester.strip_on, timeout_s=20.0)


@app.post("/led/pct")
async def led_pct(req: LedIntensityRequest):
    tester = _get_tester()
    return await _run_blocking("LED percent", lambda: tester.strip_set_pct(req.pct), timeout_s=20.0)


@app.post("/led/rgb")
async def led_rgb(req: LedRgbRequest):
    tester = _get_tester()
    return await _run_blocking(
        "LED RGB",
        lambda: tester.strip_set_rgb(
            req.r,
            req.g,
            req.b,
            reconnect_first=bool(req.reconnect_first),
        ),
        timeout_s=20.0,
    )


def _execute_oem_startup_step(tester: BioXpTester, step: str, timeout_s: float) -> dict:
    step = str(step).strip().lower()
    t0 = time.monotonic()
    arm_state = tester.motion_arm_state()
    live_gate = tester.motion_gate_live_snapshot()
    if not (bool(arm_state.get("armed")) and bool(live_gate.get("ok"))):
        raise HTTPException(
            status_code=409,
            detail={"error": "strict startup arm/live gate is not green; run no-homing strict startup first", "arm_state": arm_state, "live_gate": live_gate},
        )
    board_status = tester.activate_boards(expect_reply=True)
    interlock = tester.motor_prepare_motion_interlock(force_lock=True)
    # Keep each supervised step source-shaped to ClassControlInterface.initializeMotors().
    if step == "z-home":
        result = tester.motor_oem_home_axis("z", startup=True, timeout_s=timeout_s)
    elif step == "gripper-clear":
        preset = tester._motion_oem_axis_profile("g", startup=True)
        set_current = tester.motor_set_axis_param(preset["board"], 6, 31, motor=preset["motor"])
        move = tester.motor_move_relative(preset["board"], 10000, motor=preset["motor"])
        wait = tester.motor_wait_stopped(
            preset["board"],
            motor=preset["motor"],
            timeout_s=min(float(timeout_s), 12.0),
            require_seen_nonzero=True,
        )
        result = {"set_gripper_current_31": set_current, "move_steps_10000": move, "wait": wait}
        if not (move.get("ok") and wait.get("stopped") is True):
            raise HTTPException(status_code=409, detail=f"OEM gripper clear failed/ambiguous: {wait}")
    elif step == "gripper-home":
        result = tester.motor_oem_home_axis("g", startup=True, timeout_s=timeout_s)
    elif step == "x-home":
        result = tester.motor_oem_home_axis("x", startup=True, timeout_s=timeout_s)
    elif step == "x-park-6000":
        preset = tester._motion_oem_axis_profile("x", startup=True)
        sethome = tester.motor_set_home(preset["board"], motor=preset["motor"])
        set_speed = tester.motor_set_axis_param(preset["board"], 4, 1700, motor=preset["motor"])
        time.sleep(0.04)
        move = tester.motor_move_absolute(preset["board"], 6000, motor=preset["motor"])
        wait = tester.motor_wait_stopped(
            preset["board"],
            motor=preset["motor"],
            timeout_s=min(float(timeout_s), 12.0),
            require_seen_nonzero=True,
        )
        result = {"set_home_x": sethome, "set_speed_1700": set_speed, "move_x_6000": move, "wait": wait}
        if not (move.get("ok") and wait.get("stopped") is True):
            raise HTTPException(status_code=409, detail=f"OEM X park failed/ambiguous: {wait}")
    elif step == "y-home":
        result = tester.motor_oem_home_axis("y", startup=True, timeout_s=timeout_s)
    elif step == "door-home":
        result = tester.motor_oem_home_axis("door", startup=True, timeout_s=timeout_s)
    elif step == "y-set-home":
        preset = tester._motion_oem_axis_profile("y", startup=True)
        result = tester.motor_set_home(preset["board"], motor=preset["motor"])
    else:
        raise HTTPException(status_code=422, detail=f"Unknown OEM startup step: {step}")
    if step in {"z-home", "gripper-home", "x-home", "y-home", "door-home"}:
        _ensure_oem_home_succeeded(tester, AxisName("g" if step == "gripper-home" else "door" if step == "door-home" else step.split("-")[0]), result)
    return {
        "ok": True,
        "step": step,
        "board_status": board_status,
        "interlock": interlock,
        "arm_state": arm_state,
        "live_gate": live_gate,
        "result": result,
        "motion_truth": _motion_truth_payload(),
        "oem_reference": "ClassControlInterface.initializeMotors lines 3348-3391",
        "elapsed_ms": int((time.monotonic() - t0) * 1000),
    }


@app.post("/motion/oem/startup_step")
async def motion_oem_startup_step(req: OemStartupStepRequest):
    tester = _get_tester()
    return await _run_blocking(
        f"OEM startup homing step {req.step}",
        lambda: _execute_oem_startup_step(tester, req.step, req.timeout_s),
        timeout_s=min(max(float(req.timeout_s) + 10.0, 15.0), 120.0),
    )


@app.post("/motion/axis/relative")
async def move_axis_relative(req: MoveRelativeRequest):
    response = await run_relative_motion_command(
        RelativeMoveCommand.from_request(req),
        get_tester=_get_tester,
        run_blocking=_run_blocking,
        execute_relative_move=_execute_relative_move,
        dry_run_response_factory=_dry_run_motion_response,
    )
    if not bool(response.get("dry_run")):
        _reference_state_store.record_motion(req.axis, "relative")
    return response


@app.post("/motion/axis/absolute")
async def move_axis_absolute(req: MoveAbsoluteRequest):
    response = await run_absolute_motion_command(
        AbsoluteMoveCommand.from_request(req),
        get_tester=_get_tester,
        run_blocking=_run_blocking,
        execute_absolute_move=_execute_absolute_move,
        dry_run_response_factory=_dry_run_motion_response,
    )
    if not bool(response.get("dry_run")):
        _reference_state_store.record_motion(req.axis, "absolute")
    return response


@app.post("/motion/axis/home")
async def home_axis(req: HomeAxisRequest):
    response = await run_home_axis_command(
        HomeAxisCommand.from_request(req),
        get_tester=_get_tester,
        run_blocking=_run_blocking,
        execute_home_axis=_execute_home_axis,
        dry_run_response_factory=_dry_run_motion_response,
    )
    if not bool(response.get("dry_run")):
        _reference_state_store.mark_referenced(
            MarkAxisReferencedCommand(
                axis=req.axis,
                position_steps=0,
                source="home_axis",
                note="Axis homed via API route.",
                motion_kind="home",
            )
        )
    return response


@app.get("/motion/reference/status")
async def motion_reference_status(
    axes: str = Query("x,y,z,g,door", description="Comma-separated axes to inspect reference state for."),
):
    return _reference_state_store.snapshot(_parse_axes_csv(axes))


@app.post("/motion/reference/mark_referenced")
async def motion_reference_mark_referenced(req: ReferenceMarkRequest):
    return _reference_state_store.mark_referenced(
        MarkAxisReferencedCommand(
            axis=req.axis,
            position_steps=req.position_steps,
            source=req.source,
            note=req.note,
            motion_kind="manual_reference",
        )
    )


@app.post("/motion/reference/mark_desynced")
async def motion_reference_mark_desynced(req: ReferenceDesyncRequest):
    return _reference_state_store.mark_desynced(
        MarkAxisDesyncedCommand(
            axis=req.axis,
            reason=req.reason,
            source=req.source,
            motion_kind="manual_desync",
        )
    )


@app.post("/thermal/baseline")
async def thermal_baseline():
    tester = _get_tester()
    return await _run_blocking("Thermal baseline", lambda: tester.thermal_apply_vendor_baseline(verify=True), timeout_s=30.0)


@app.get("/thermal/snapshot")
async def thermal_snapshot():
    tester = _get_tester()
    return await _run_blocking("Thermal snapshot", tester.thermal_snapshot, timeout_s=25.0)


@app.post("/thermal/set_temp")
async def set_thermal_temp(req: ThermalRequest):
    tester = _get_tester()
    if req.bank is ThermalBankName.PEDESTAL:
        return await _run_blocking("Thermal pedestal setpoint", lambda: tester.thermal_set_ped_temp(req.target_temp_c, verify=True), timeout_s=25.0)
    bank = _THERMAL_BANK_MAP[req.bank]
    return await _run_blocking("Thermal setpoint", lambda: tester.thermal_set_target_temp(bank, req.target_temp_c, verify=True), timeout_s=25.0)


@app.post("/thermal/fan")
async def set_thermal_fan(req: ThermalFanRequest):
    tester = _get_tester()
    return await _run_blocking("Thermal fan", lambda: tester.thermal_set_fan(req.speed, verify=True), timeout_s=25.0)


@app.post("/thermal/pwm")
async def set_thermal_pwm(req: ThermalPwmRequest):
    tester = _get_tester()
    if req.bank not in _THERMAL_BANK_MAP:
        raise HTTPException(status_code=400, detail="PWM is only supported for nest or lid.")
    return await _run_blocking("Thermal PWM", lambda: tester.thermal_set_pwm(_THERMAL_BANK_MAP[req.bank], req.pwm, verify=True), timeout_s=25.0)


@app.post("/thermal/rates")
async def set_thermal_rates(req: ThermalRatesRequest):
    tester = _get_tester()
    if req.bank not in _THERMAL_BANK_MAP:
        raise HTTPException(status_code=400, detail="Rate control is only supported for nest or lid.")
    return await _run_blocking(
        "Thermal rates",
        lambda: tester.thermal_set_rates(
            _THERMAL_BANK_MAP[req.bank],
            cool_rate_c_s=req.cool_rate_c_s,
            heat_rate_c_s=req.heat_rate_c_s,
            verify=True,
        ),
        timeout_s=25.0,
    )


@app.post("/thermal/fast_profile")
async def thermal_fast_profile():
    tester = _get_tester()
    return await _run_blocking("Thermal fast profile", lambda: tester.thermal_apply_fast_profile(verify=True), timeout_s=30.0)


@app.post("/thermal/hard_reset")
async def thermal_hard_reset():
    tester = _get_tester()
    return await _run_blocking("Thermal hard reset", tester.thermal_hard_reset, timeout_s=40.0)


@app.post("/chiller/baseline")
async def chiller_baseline():
    tester = _get_tester()
    return await _run_blocking("Chiller baseline", lambda: tester.chiller_apply_vendor_baseline(verify=True), timeout_s=30.0)


@app.get("/chiller/snapshot")
async def chiller_snapshot():
    tester = _get_tester()
    return await _run_blocking("Chiller snapshot", tester.chiller_snapshot, timeout_s=25.0)


@app.post("/chiller/set_temp")
async def set_chiller_temp(req: ChillerRequest):
    tester = _get_tester()
    bank = _CHILLER_BANK_MAP[req.bank]
    return await _run_blocking(
        "Chiller setpoint",
        lambda: (
            tester.chiller_activate(),
            tester.chiller_set_target_temp(bank, req.target_temp_c, verify=True),
        )[1],
        timeout_s=25.0,
    )


@app.post("/chiller/fan")
async def set_chiller_fan(req: ChillerFanRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Chiller fan",
        lambda: (
            tester.chiller_activate(),
            tester.chiller_set_fan(_CHILLER_BANK_MAP[req.bank], req.speed, verify=True),
        )[1],
        timeout_s=25.0,
    )


@app.post("/chiller/pwm")
async def set_chiller_pwm(req: ChillerPwmRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Chiller PWM",
        lambda: (
            tester.chiller_activate(),
            tester.chiller_set_pwm(_CHILLER_BANK_MAP[req.bank], req.pwm, verify=True),
        )[1],
        timeout_s=25.0,
    )


@app.post("/chiller/rates")
async def set_chiller_rates(req: ChillerRatesRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Chiller rates",
        lambda: (
            tester.chiller_activate(),
            tester.chiller_set_rates(
                _CHILLER_BANK_MAP[req.bank],
                cool_rate_c_s=req.cool_rate_c_s,
                heat_rate_c_s=req.heat_rate_c_s,
                verify=True,
            ),
        )[1],
        timeout_s=25.0,
    )


@app.post("/chiller/hard_reset")
async def chiller_hard_reset():
    tester = _get_tester()
    return await _run_blocking("Chiller hard reset", tester.chiller_hard_reset, timeout_s=40.0)


@app.get("/camera/devices")
async def camera_devices():
    tester = _get_tester()
    return await _run_blocking("Camera devices", lambda: _camera_devices_payload(tester), timeout_s=15.0)


@app.get("/camera/controls")
async def camera_controls(device: str = "/dev/video0"):
    tester = _get_tester()
    return await _run_blocking("Camera controls", lambda: tester.camera_enumerate_controls(device=device), timeout_s=15.0)


@app.post("/camera/control")
async def camera_control(req: CameraControlRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Camera control",
        lambda: {
            **tester.v4l2_set_ctrl(req.cid, req.value, device=req.device),
            "stream_active": bool(_camera_stream_state.get("active")),
            "stream_state": _camera_stream_state_payload(),
        },
        timeout_s=15.0,
    )


@app.post("/camera/snapshot")
async def camera_snapshot(req: CameraSnapshotRequest):
    tester = _get_tester()
    return await _run_blocking("Camera snapshot", lambda: _camera_snapshot_with_data(tester, req.device), timeout_s=20.0)


@app.post("/vision/inspect")
async def vision_inspect(req: InspectionRequest):
    return await run_inspection_command(
        InspectionCommand.from_request(req),
        get_capabilities=_get_vision_capabilities,
        capture_snapshot=lambda device: _camera_snapshot_with_data(_get_tester(), device),
        run_blocking=_run_blocking,
    )


@app.post("/vision/barcode/read")
async def vision_barcode_read(req: BarcodeReadRequest):
    return await run_barcode_read_command(
        BarcodeReadCommand.from_request(req),
        get_capabilities=_get_vision_capabilities,
        capture_snapshot=lambda device: _camera_snapshot_with_data(_get_tester(), device),
        run_blocking=_run_blocking,
    )


@app.post("/camera/stream_health")
async def camera_stream_health(req: CameraHealthRequest):
    tester = _get_tester()
    return await _run_blocking(
        "Camera stream health",
        lambda: _camera_stream_health_direct(tester, req.device, req.seconds),
        timeout_s=max(20.0, req.seconds + 10.0),
    )


@app.post("/camera/auto_recover")
async def camera_auto_recover(req: CameraRecoverRequest):
    if _camera_stream_lock.locked():
        return {
            "ok": False,
            "busy": True,
            "stream_active": True,
            "device": req.device,
            "error": "live stream is active; stop the stream before running auto recover",
            "stream_state": _camera_stream_state_payload(),
        }
    tester = _get_tester()
    return await _run_blocking(
        "Camera auto recover",
        lambda: tester.camera_auto_oneclick(device=req.device, max_resets=req.max_resets),
        timeout_s=75.0,
    )


@app.post("/camera/reset")
async def camera_reset(req: CameraSnapshotRequest):
    return _camera_reset_local(req.device)


@app.post("/camera/stop")
async def camera_stop(req: CameraSnapshotRequest):
    return await run_in_threadpool(_camera_reset_local, req.device)


@app.get("/camera/stream_state")
async def camera_stream_state():
    return _camera_stream_state_payload()


@app.get("/camera/mjpeg")
async def camera_mjpeg(
    device: str = "/dev/video0",
    fps: int = Query(8, ge=1, le=30),
    quality: int = Query(7, ge=2, le=15),
    width: int = Query(640, ge=160, le=1920),
    height: int = Query(480, ge=120, le=1080),
):
    tester = _get_tester()
    return await _camera_mjpeg_response(
        tester,
        preferred=device,
        fps=fps,
        quality=quality,
        width=width,
        height=height,
    )


@app.get("/liquid/status")
async def liquid_status():
    return await run_pipette_status(
        get_transport=_get_pipette_transport,
        run_blocking=_run_blocking,
    )


@app.post("/liquid/init")
async def liquid_init(req: PipetteInitRequest):
    return await run_pipette_init_command(
        PipetteInitCommand.from_request(req),
        get_transport=_get_pipette_transport,
        run_blocking=_run_blocking,
    )


@app.post("/liquid/tip")
async def liquid_tip(req: PipetteTipRequest):
    return await run_pipette_tip_command(
        PipetteTipCommand.from_request(req),
        get_transport=_get_pipette_transport,
        run_blocking=_run_blocking,
    )


@app.post("/liquid/aspirate")
async def liquid_aspirate(req: PipetteAspirateRequest):
    return await run_pipette_aspirate_command(
        PipetteAspirateCommand.from_request(req),
        get_transport=_get_pipette_transport,
        run_blocking=_run_blocking,
        preflight=_liquid_reference_preflight,
    )


@app.post("/liquid/dispense")
async def liquid_dispense(req: PipetteDispenseRequest):
    return await run_pipette_dispense_command(
        PipetteDispenseCommand.from_request(req),
        get_transport=_get_pipette_transport,
        run_blocking=_run_blocking,
        preflight=_liquid_reference_preflight,
    )


@app.post("/liquid/mix")
async def liquid_mix(req: PipetteMixRequest):
    return await run_pipette_mix_command(
        PipetteMixCommand.from_request(req),
        get_transport=_get_pipette_transport,
        run_blocking=_run_blocking,
        preflight=_liquid_reference_preflight,
    )


@app.post("/protocol/compile")
async def protocol_compile(req: ProtocolCompileRequest):
    compiled = compile_protocol_source(req.model_dump(exclude_none=True))
    return compiled.to_payload()


@app.post("/protocol/execute")
async def protocol_execute(req: ProtocolExecuteRequest):
    try:
        return await run_in_threadpool(
            create_protocol_job,
            req.model_dump(exclude_none=True),
            dry_run=bool(req.dry_run),
        )
    except ProtocolLiveContractError as exc:
        raise HTTPException(status_code=409, detail=exc.to_payload()) from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.get("/protocol/jobs")
async def protocol_jobs(limit: int = Query(20, ge=1, le=100)):
    return {
        "rows": await run_in_threadpool(list_protocol_jobs, limit=limit),
    }


@app.get("/protocol/jobs/{job_id}")
async def protocol_job_detail(job_id: str):
    try:
        return await run_in_threadpool(get_protocol_job, job_id)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@app.post("/protocol/jobs/{job_id}/review")
async def protocol_job_review(job_id: str, req: ProtocolReviewRequest):
    try:
        return await run_in_threadpool(
            review_protocol_job,
            job_id,
            reviewer=req.reviewer,
            note=req.note,
        )
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
