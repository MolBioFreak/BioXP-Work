import asyncio
import base64
import copy
import hashlib
import json
import os
import signal
import shutil
import subprocess
import tarfile
import tempfile
import threading
import time
import uuid
from contextlib import asynccontextmanager
from enum import Enum
from typing import Any, Optional

from fastapi import FastAPI, HTTPException, Query, Request

from .oem_config import harmonized_motion_config
from .hardware_status import CANONICAL_DOMAINS, CollectionContext, hardware_state
from .lifecycle_state import LifecycleStateError, lifecycle_state
from .oem_machine_bundle import configure_oem_machine_snapshot_from_env
from .runtime_state import configure_oem_runtime_state_from_env
from .oem_gripper import gripper_clear, gripper_home, gripper_status, restore_gripper_idle_current
from .oem_initialization import run_oem_initialization_controller
from .oem_compat.api import router as oem_compat_router
from .oem_homing_routes import router as oem_homing_router
from .oem_runtime_api import configure_runtime as configure_oem_runtime, router as oem_runtime_router, shutdown_runtime as shutdown_oem_runtime
from .oem_startup_program import BioXpStartupHardware, OEMStartupProgram, DryRunStartupHardware
from .oem_startup_types import OemDoorEventRequest, OemInitialCheckRequest, OemStartupRequest, OemSwitchAuditRequest
from .oem_switch_audit import OfflineSwitchAuditFixture, interpret_home_predicate, run_switch_audit
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
from .protocols import ProtocolActionKind
from .services.reference_service import (
    MarkAxisDesyncedCommand,
    MarkAxisReferencedCommand,
    ReferenceStateStore,
)
from .services.vision_service import (
    run_barcode_read_command,
    run_inspection_command,
)

BMS_COMMISSIONING_CAPABILITIES = (
    "collect_hardware_snapshot",
    "initialize_oem_environment",
)

# Explicit camera routes own camera evidence. A generic hardware snapshot must
# neither activate a camera nor claim an unqueried cache as observed.
DEFAULT_HARDWARE_SNAPSHOT_DOMAINS = tuple(
    domain for domain in CANONICAL_DOMAINS if domain != "camera"
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
_quarantined_tester: Optional[BioXpTester] = None
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
_pipette_transport = None
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
_camera_projection_lock = threading.RLock()
_camera_projection_epoch = 0
_camera_probe_cache: dict[str, Any] | None = None
_camera_session: dict[str, Any] | None = None
USB_SNIFF_ACK = "USB_SNIFF"
USB_SNIFF_PROFILES = {"passive", "manual_observe", "debug"}
RESET_PROVENANCE_SCHEMA_VERSION = "bioxp.reset_provenance.v1"
MAINTENANCE_STATE_SCHEMA_VERSION = "bioxp.maintenance_state.v1"
MAINTENANCE_RECOVERY_ACK = "RECOVER"
_maintenance_state: dict[str, Any] = {
    "schema_version": MAINTENANCE_STATE_SCHEMA_VERSION,
    "usb_owner": "service",
    "motion_blocked": False,
    "recovery_required": False,
    "block_reason": None,
    "recovery_hint": None,
    "blocked_by": None,
    "last_transition": "service_start",
    "last_transition_at": None,
    "last_recovery": None,
}


def _now_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _reset_provenance(*, subsystem: str, source: str, reset_scope: str, **extra: Any) -> dict[str, Any]:
    payload = {
        "schema_version": RESET_PROVENANCE_SCHEMA_VERSION,
        "created_at": _now_utc(),
        "subsystem": subsystem,
        "source": source,
        "reset_scope": reset_scope,
        "software_recovery": bool(extra.pop("software_recovery", True)),
        "hardware_component_fault_proven": bool(extra.pop("hardware_component_fault_proven", False)),
    }
    payload.update(extra)
    return payload


def _maintenance_state_payload() -> dict[str, Any]:
    return dict(_maintenance_state)


def _set_maintenance_state(*, transition: str, **updates: Any) -> dict[str, Any]:
    _maintenance_state.update(updates)
    _maintenance_state["last_transition"] = str(transition)
    _maintenance_state["last_transition_at"] = _now_utc()
    return _maintenance_state_payload()


def _ownership_changed(*, reason: str, transport: str, usb: str, router: str) -> int:
    """Invalidate every hardware and camera projection on ownership changes."""
    global _camera_projection_epoch, _camera_probe_cache
    epoch = hardware_state.change_ownership(
        reason=reason,
        transport=transport,
        usb=usb,
        router=router,
    )
    with _camera_projection_lock:
        _camera_projection_epoch += 1
        _camera_probe_cache = None
        _camera_stream_state.update(
            {
                "active": False,
                "last_error": f"invalidated by ownership change: {reason}",
                "last_frame_at": None,
            }
        )
    return epoch


def _mark_post_maintenance_motion_block(*, source: str, reason: str, **extra: Any) -> dict[str, Any]:
    payload = {
        "usb_owner": extra.pop("usb_owner", _maintenance_state.get("usb_owner", "service")),
        "motion_blocked": True,
        "recovery_required": True,
        "block_reason": str(reason),
        "recovery_hint": (
            "Run non-homing strict startup or localhost-only POST /maintenance/usb/recover_motion "
            "before any axis/home/clear-lock motion."
        ),
        "blocked_by": str(source),
    }
    payload.update(extra)
    return _set_maintenance_state(transition=source, **payload)


def _clear_post_maintenance_motion_block(*, source: str, evidence: Optional[dict[str, Any]] = None) -> dict[str, Any]:
    return _set_maintenance_state(
        transition=source,
        usb_owner="service",
        motion_blocked=False,
        recovery_required=False,
        block_reason=None,
        recovery_hint=None,
        blocked_by=None,
        last_recovery={"source": str(source), "at": _now_utc(), "evidence": evidence or {}},
    )


def _require_motion_not_blocked_by_maintenance() -> None:
    if not bool(_maintenance_state.get("motion_blocked")):
        return
    raise HTTPException(
        status_code=409,
        detail={
            "error": "post_maintenance_motion_recovery_required",
            "message": str(
                _maintenance_state.get("block_reason")
                or "USB maintenance/reconnect occurred; motion recovery is required before commanding axes."
            ),
            "hardware_motion_commanded": False,
            "maintenance_state": _maintenance_state_payload(),
            "allowed_recovery": [
                "POST /motion/arm/strict_startup with {\"run_homing\": false}",
                f"localhost POST /maintenance/usb/recover_motion with operator_ack={MAINTENANCE_RECOVERY_ACK!r}",
            ],
        },
    )


@asynccontextmanager
async def lifespan(app: FastAPI):
    del app
    global _tester, _startup_error, _pipette_transport, _quarantined_tester
    try:
        machine_snapshot = configure_oem_machine_snapshot_from_env(require_operator_label=True)
        configure_oem_runtime_state_from_env(machine_snapshot)
        # OEM BioXPMainWindow sets BoardTestMode only when the process command
        # line contains the separate token "boardtest".  It is not StartMode.
        board_test_mode = os.environ.get("BIOXP_BOARD_TEST_MODE", "").strip().lower() in {"1", "true", "yes", "boardtest"}
        lifecycle_state.bind_configuration(
            start_mode=machine_snapshot.startup_mode,
            board_test_mode=board_test_mode,
            check_camera=bool(machine_snapshot.operation_parameters["CheckCamera"]),
        )
    except Exception as exc:
        _tester = None
        _startup_error = f"OEM machine/runtime-state authority unavailable: {exc}"
        print(f"[WARN] {_startup_error}")
    else:
        _tester = None
        _startup_error = (
            "USB transport is intentionally unbound during generic API lifespan; "
            "use an explicit ownership POST before hardware collection."
        )
    _ownership_changed(
        reason="generic_lifespan_unbound",
        transport="unbound",
        usb="unbound",
        router="unbound",
    )
    try:
        yield
    finally:
        await _stop_owned_camera_session(reason="lifespan shutdown")
        shutdown_oem_runtime()
        close_fn = getattr(_pipette_transport, "close", None)
        if callable(close_fn):
            close_fn()
        testers = [tester for tester in (_tester, _quarantined_tester) if tester is not None]
        for tester in testers:
            disconnect = getattr(tester, "_disconnect", None)
            if callable(disconnect):
                try:
                    disconnect()
                except Exception:
                    pass
        _tester = None
        _quarantined_tester = None
        _pipette_transport = None
        _ownership_changed(
            reason="generic_lifespan_shutdown",
            transport="unbound",
            usb="unbound",
            router="stopped",
        )


app = FastAPI(
    title="BioXP 3200 Control API",
    description="REST interface backed by the canonical USB runtime (usb_driver.py).",
    version="0.3.0",
    lifespan=lifespan,
)
app.include_router(oem_compat_router)
app.include_router(oem_runtime_router)
app.include_router(oem_homing_router)


def _get_tester() -> BioXpTester:
    if _quarantined_tester is not None:
        raise HTTPException(
            status_code=503,
            detail=_startup_error or "BioXP USB runtime is quarantined after cleanup failure.",
        )
    if _tester is None:
        raise HTTPException(
            status_code=503,
            detail=_startup_error or "BioXP USB runtime not available.",
        )
    return _tester


def _get_oem_startup_program(*, dry_safe: bool = False) -> OEMStartupProgram:
    global _oem_startup_program
    base = os.environ.get("BIOXP_OEM_STARTUP_ARTIFACT_BASE") or "/tmp/bioxp-live-runs"
    want_live_provider = not dry_safe
    current_is_offline = isinstance(getattr(_oem_startup_program, "hardware", None), DryRunStartupHardware)
    if want_live_provider and _tester is None:
        raise HTTPException(status_code=503, detail=_startup_error or "BioXP USB runtime not available for live/shadow OEM startup provider.")
    if _oem_startup_program is None or (want_live_provider and current_is_offline):
        if want_live_provider and _tester is not None:
            hardware = BioXpStartupHardware(_get_tester)
        else:
            hardware = DryRunStartupHardware(door_closed=False, latch_closed=False)
        _oem_startup_program = OEMStartupProgram(hardware=hardware, artifact_base=base)
    return _oem_startup_program


def _get_existing_oem_startup_program() -> OEMStartupProgram:
    if _oem_startup_program is not None:
        return _oem_startup_program
    return _get_oem_startup_program(dry_safe=False)


class _BioXpSwitchAuditHardware:
    def __init__(self, tester: BioXpTester):
        self.tester = tester
        self.move_calls: list[dict] = []

    def switch_snapshot(self, axis: str) -> dict:
        preset = self.tester._motion_oem_axis_profile(axis) if hasattr(self.tester, "_motion_oem_axis_profile") else None
        if not isinstance(preset, dict):
            return {"axis": axis, "error": "live_axis_profile_unavailable", "live_robot_proof": False}
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


class _LifecycleHardware:
    """Exact startup-stage adapter; constructed only inside explicit POST work."""

    def __init__(self, tester: BioXpTester):
        self.tester = tester

    def set_led_rgb(self, r: int, g: int, b: int) -> dict[str, Any]:
        return self.tester.strip_set_rgb(
            int(r),
            int(g),
            int(b),
            reconnect_first=False,
            activate_first=False,
            fail_fast=True,
        )

    def query_door(self) -> dict[str, Any]:
        ack = self.tester.query_only_tmcl(self.tester.BOARD_DECK, 15, 1, 0, 0)
        return {
            "ok": self.tester._tmcl_success(ack),
            "value": None if ack is None else ack.get("value"),
            "ack": ack,
            "query": "door",
        }

    def query_latch(self) -> dict[str, Any]:
        ack = self.tester.query_only_tmcl(self.tester.BOARD_DECK, 15, 3, 0, 0)
        return {
            "ok": self.tester._tmcl_success(ack),
            "value": None if ack is None else ack.get("value"),
            "ack": ack,
            "query": "latch",
        }

    def set_solenoid(self, value: int) -> dict[str, Any]:
        result = self.tester.deck_io_set_type(2, int(value))
        return {**result, "ok": result.get("ok") is True}

    def query_voltage(self) -> dict[str, Any]:
        ack = self.tester.query_only_tmcl(self.tester.BOARD_DECK, 15, 0, 0, 0)
        status = None if ack is None else ack.get("status")
        value = None if ack is None or status != 100 else ack.get("value")
        return {
            "ok": bool(ack is not None and status == 100 and value is not None),
            "payload_raw": value,
            "reply_present": ack is not None,
            "transport_outcome": "reply" if ack is not None else "no_reply",
            "oem_status": status,
            "ack": ack,
        }

    def deactivate_boards(self) -> dict[str, Any]:
        acks = self.tester.deactivate_boards(expect_reply=True, fail_fast=True)
        return {"ok": self.tester._oem_board_activation_map_success(acks), "acks": acks}

    def activate_boards(self) -> dict[str, Any]:
        acks = self.tester.activate_boards(expect_reply=True, fail_fast=True)
        return {"ok": self.tester._oem_board_activation_map_success(acks), "acks": acks}


def _can_ready_observation() -> bool | None:
    return (hardware_state.ownership_projection().get("ownership") or {}).get("CAN_READY")


def _constructor_pipette_action() -> dict[str, Any]:
    owner = _get_pipette_transport()
    if owner.__class__.__name__ != "FourPipetteTransport":
        return {"ok": False, "error": "exact_four_pipette_owner_required"}
    result = owner.initialize(PipetteInitCommand())
    return {
        **result,
        "ok": bool(result.get("ok")),
        "channel_count": 4,
        "channels_constructed_unconditionally": [0, 1, 2, 3],
        "predecessor": "CAN_READY",
        "can_ready_evidence": _can_ready_observation(),
        "motion_commanded": False,
    }


def _initialize_without_motion_action() -> dict[str, Any]:
    tester = _get_tester()
    motor_current = tester.motor_oem_initialize_without_motion()
    if bool(motor_current.get("physical_motion")):
        return {"ok": False, "error": "initialize_without_motion_reported_motion", "motor_current_verification": motor_current}
    lifecycle = lifecycle_state.projection()
    operations = motor_current.get("operations") if isinstance(motor_current, dict) else None
    led_white = [
        row for row in operations or []
        if isinstance(row, dict) and str(row.get("name", "")).startswith("setColor.white.")
    ]
    ok = bool(motor_current.get("ok") and len(led_white) == 3 and all(row.get("ok") for row in led_white))
    return {
        "ok": ok,
        "motor_current_verification": motor_current,
        "led_white": led_white,
        "start_mode_assignment": {
            "value": lifecycle.get("start_mode"),
            "source": "immutable OperationParameters.Mode",
            "assigned_to": "canonical lifecycle owner",
        },
        "physical_motion": False,
        "homing_performed": False,
    }


def _get_pipette_transport():
    if _pipette_transport is None:
        raise HTTPException(status_code=503, detail="Pipette transport owner is unavailable")
    return _pipette_transport


def _oem_non_motion_startup_result(
    projection: dict[str, Any],
    *,
    failed_stage: str | None = None,
) -> dict[str, Any]:
    return {
        "ok": failed_stage is None and projection["startup"]["state"] == "passed",
        "failed_stage": failed_stage,
        "sequence": [
            "constructor_pipette_stage",
            "initialization_without_motion",
            "initial_check",
        ],
        "lifecycle": projection,
        "physical_motion": False,
        "homing_performed": False,
        "initialize_system_started": False,
        "next_oem_boundary": "initializeSystem",
        "source_anchors": {
            "constructor": "ControlLib.cs:700,963-984",
            "environment": "BioXPMainWindow.cs:821,973-997",
        },
    }


def _run_oem_non_motion_startup_sequence(
    *,
    sleep=time.sleep,
    clock=time.monotonic,
) -> dict[str, Any]:
    """Mirror the automatic OEM startup path through initialCheck only.

    The OEM ControlLib constructor performs the four-pipette sequence followed
    by initializeMotorsWithoutMotion. BioXPMainWindow then invokes
    initializeEnvironment/initialCheck during application load. The subsequent
    initializeSystem command is intentionally outside this non-motion boundary.
    """
    projection = lifecycle_state.projection()
    stages = projection["startup"]["stages"]
    if any(stages[name]["state"] != expected for name, expected in (
        ("constructor_pipette_stage", "not_run"),
        ("initialization_without_motion", "blocked"),
        ("initial_check", "blocked"),
    )):
        raise LifecycleStateError(
            "OEM non-motion startup requires a fresh ownership epoch; completed or partial one-shot stages cannot be replayed"
        )

    projection = lifecycle_state.run_stage("constructor_pipette_stage", _constructor_pipette_action)
    if projection["startup"]["stages"]["constructor_pipette_stage"]["state"] != "passed":
        return _oem_non_motion_startup_result(projection, failed_stage="constructor_pipette_stage")

    projection = lifecycle_state.run_stage("initialization_without_motion", _initialize_without_motion_action)
    if projection["startup"]["stages"]["initialization_without_motion"]["state"] != "passed":
        return _oem_non_motion_startup_result(projection, failed_stage="initialization_without_motion")

    try:
        initial_check_hardware = _LifecycleHardware(_get_tester())
    except Exception as exc:
        projection = lifecycle_state.run_stage(
            "initial_check",
            lambda: {
                "ok": False,
                "error": "initial_check_hardware_unavailable",
                "detail": str(exc),
                "exception_type": type(exc).__name__,
                "physical_motion": False,
            },
        )
        return _oem_non_motion_startup_result(projection, failed_stage="initial_check")

    projection = lifecycle_state.run_initial_check(
        initial_check_hardware,
        can_ready=_can_ready_observation,
        sleep=sleep,
        clock=clock,
    )
    if projection["startup"]["stages"]["initial_check"]["state"] != "passed":
        return _oem_non_motion_startup_result(projection, failed_stage="initial_check")
    return _oem_non_motion_startup_result(projection)


@app.post("/oem/startup/initialize_environment")
async def oem_startup_initialize_environment(req: OemInitialCheckRequest):
    payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    if payload.get("mode") != "live" or payload.get("operator_ack") != "INITIALIZE":
        raise HTTPException(
            status_code=409,
            detail="live mode and operator_ack INITIALIZE required for OEM non-motion startup",
        )
    if _can_ready_observation() is not True:
        raise HTTPException(status_code=409, detail="OEM non-motion startup requires explicit CAN_READY=true evidence")
    try:
        result = await _run_blocking(
            "OEM automatic non-motion startup through initializeEnvironment/initialCheck",
            _run_oem_non_motion_startup_sequence,
            timeout_s=460.0,
        )
    except LifecycleStateError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    if not result.get("ok"):
        raise HTTPException(status_code=409, detail=result)
    return result


@app.post("/oem/startup/request")
async def oem_startup_request(req: OemStartupRequest):
    request_payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    if request_payload.get("mode") == "live" and request_payload.get("operator_ack") != "INITIALIZE":
        raise HTTPException(status_code=409, detail="operator_ack INITIALIZE required for live OEM startup")
    if request_payload.get("mode") == "live" and not request_payload.get("artifact_root"):
        raise HTTPException(status_code=409, detail="artifact_root required for live OEM startup")
    # This legacy request binds the broader motion-startup owner only. The
    # BMS non-motion startup action uses /oem/startup/initialize_environment;
    # initializeSystem/homing remain outside that aggregate boundary.
    _get_oem_startup_program(dry_safe=request_payload.get("mode") == "dry_run")
    status = lifecycle_state.transition("waiting", reason="startup_requested_awaiting_constructor_stage")
    return {
        "ok": True,
        "session_id": "canonical",
        "status": status["startup"]["state"],
        "state": status["operation_state"],
        "mode": request_payload.get("mode"),
        "queued": False,
        "artifact_root": request_payload.get("artifact_root"),
        "startup_status_url": "/oem/startup/status/canonical",
        "lifecycle": status,
        "next_action": "POST /oem/startup/constructor_pipettes",
    }


@app.get("/oem/startup/status/latest")
async def oem_startup_status_latest():
    projection = lifecycle_state.projection()
    return {"ok": True, "available": True, "cache_state": "memory", "state": projection["startup"]["state"], "lifecycle": projection}


@app.get("/oem/startup/status/{session_id}")
async def oem_startup_status(session_id: str):
    if session_id != "canonical":
        raise HTTPException(status_code=404, detail="startup session not found")
    projection = lifecycle_state.projection()
    return {"ok": True, "available": True, "cache_state": "memory", "session_id": session_id, "state": projection["startup"]["state"], "lifecycle": projection}


@app.post("/oem/startup/constructor_pipettes")
async def oem_startup_constructor_pipettes():
    if _can_ready_observation() is not True:
        raise HTTPException(status_code=409, detail="constructor pipette stage requires explicit CAN_READY=true evidence")
    try:
        projection = await _run_blocking(
            "OEM constructor four-pipette stage",
            lambda: lifecycle_state.run_stage("constructor_pipette_stage", _constructor_pipette_action),
            timeout_s=260.0,
        )
    except LifecycleStateError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    if projection["startup"]["stages"]["constructor_pipette_stage"]["state"] != "passed":
        raise HTTPException(status_code=409, detail=projection)
    return {"ok": True, "lifecycle": projection}


@app.post("/oem/startup/initialize_without_motion")
async def oem_startup_initialize_without_motion():
    try:
        projection = await _run_blocking(
            "OEM initializeMotorsWithoutMotion stage",
            lambda: lifecycle_state.run_stage("initialization_without_motion", _initialize_without_motion_action),
            timeout_s=120.0,
        )
    except LifecycleStateError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    if projection["startup"]["stages"]["initialization_without_motion"]["state"] != "passed":
        raise HTTPException(status_code=409, detail=projection)
    return {"ok": True, "lifecycle": projection}


@app.post("/oem/startup/door_event")
async def oem_startup_door_event(req: OemDoorEventRequest):
    payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    projection = lifecycle_state.record_door_event(
        door_closed=payload.get("door_closed"),
        latch_closed=payload.get("latch_closed"),
        source="POST /oem/startup/door_event",
    )
    return {"ok": True, "lifecycle": projection, "door": projection["door"]}


@app.post("/oem/initial_check")
async def oem_initial_check(req: OemInitialCheckRequest | None = None):
    payload = (req.model_dump() if hasattr(req, "model_dump") else req.dict()) if req is not None else {"mode": "shadow"}
    mode = payload.get("mode", "shadow")
    if mode == "live" and payload.get("operator_ack") != "INITIALIZE":
        raise HTTPException(status_code=409, detail="operator_ack INITIALIZE required for live OEM initialCheck")
    if mode != "live":
        return {
            "ok": True,
            "executed": False,
            "mode": mode,
            "lifecycle": lifecycle_state.projection(),
            "reason": "shadow/dry-run cannot claim completion of the OEM physical initialCheck stage",
        }
    tester = _get_tester()
    try:
        projection = await _run_blocking(
            "OEM initialCheck stage",
            lambda: lifecycle_state.run_initial_check(
                _LifecycleHardware(tester),
                can_ready=_can_ready_observation,
            ),
            timeout_s=60.0,
        )
    except LifecycleStateError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    if projection["startup"]["stages"]["initial_check"]["state"] != "passed":
        raise HTTPException(status_code=409, detail=projection)
    return {"ok": True, "lifecycle": projection}


@app.get("/oem/motion_worker/status")
async def oem_motion_worker_status():
    if _oem_startup_program is None:
        return {"ok": False, "available": False, "cache_state": "missing", "worker": None, "error": "OEM startup worker has not been created by an explicit startup POST"}
    return {**_oem_startup_program.worker_status(), "lifecycle": lifecycle_state.projection()}


@app.post("/oem/motion_worker/run_next")
async def oem_motion_worker_run_next():
    result = _get_existing_oem_startup_program().run_next_worker_command()
    return {"ok": result is not None, "result": result, "status": _get_existing_oem_startup_program().worker_status()}


@app.post("/oem/motion_worker/abort")
async def oem_motion_worker_abort(reason: str = "operator abort"):
    return _get_existing_oem_startup_program().abort_worker(reason=reason)


@app.post("/oem/switch_audit")
async def oem_switch_audit(req: OemSwitchAuditRequest):
    payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    mode = payload.get("mode", "status")
    artifact_root = payload.get("artifact_root") or os.environ.get("BIOXP_OEM_STARTUP_ARTIFACT_BASE")
    if _tester is None:
        raise HTTPException(status_code=503, detail=_startup_error or "BioXP USB runtime not available for live switch audit.")
    hardware = _BioXpSwitchAuditHardware(_get_tester())
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
            "If true, write a supervised validation bundle under the robot-local "
            "validation artifact root. Set BIOXP_VALIDATION_ARTIFACT_ROOT to override."
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
    speed: Optional[int] = Field(
        None,
        gt=0,
        description="Optional commissioning override for TMCL SAP4 max speed; clamped by the axis OEM speed/acc normalization.",
    )
    acc: Optional[int] = Field(
        None,
        gt=0,
        description="Optional commissioning override for TMCL SAP5 max acceleration; clamped by the axis OEM speed/acc normalization.",
    )
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
    speed: Optional[int] = Field(
        None,
        gt=0,
        description="Optional commissioning override for TMCL SAP4 max speed; clamped by the axis OEM speed/acc normalization.",
    )
    acc: Optional[int] = Field(
        None,
        gt=0,
        description="Optional commissioning override for TMCL SAP5 max acceleration; clamped by the axis OEM speed/acc normalization.",
    )


class HomeAxisRequest(MotionArtifactRequest):
    axis: AxisName
    speed: Optional[int] = Field(None, gt=0)
    timeout_s: float = Field(15.0, gt=0.1, le=90.0)
    allow_implementation_mapped_predicate: bool = Field(
        False,
        description=(
            "Safety override for supervised commissioning only. Default false blocks manual homing "
            "unless the per-axis home predicate is live/source verified. When true, the route still "
            "uses the guarded axisSearchHome inactive->active transition path, not controller-zero."
        ),
    )


class MoveAxisZeroRequest(MotionArtifactRequest):
    axis: AxisName
    speed: Optional[int] = Field(None, gt=0)
    wait_timeout_s: float = Field(15.0, gt=0.1, le=60.0)


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


class MotionInterlockOverrideRequest(BaseModel):
    enabled: bool = Field(..., description="Master commissioning override toggle. If per-sensor fields are omitted, applies to both latch sensor and 24V sense.")
    override_latch_sensor: Optional[bool] = Field(None, description="Treat latch/magnetic sensor as permissive while still requiring door and solenoid readbacks.")
    override_rail_24v: Optional[bool] = Field(None, description="Treat 24V sense input as permissive for commissioning when the sense line is known/misaligned faulty.")
    operator_ack: str = Field(..., description="Must be exactly INTERLOCK_OVERRIDE; this bypasses normal latch/24V interlock checks.")
    reason: Optional[str] = Field(None, max_length=2000, description="Operator reason alias accepted from BMS cockpit; stored with the override audit note.")
    operator_note: Optional[str] = Field(None, max_length=2000)


class UsbSniffCaptureRequest(BaseModel):
    profile: str = Field("passive", description="Capture profile: passive, manual_observe, or debug.")
    duration_s: int = Field(300, ge=1, le=1800)
    include_pcap: bool = True
    include_driver_ledger: bool = True
    passive_only: bool = True
    reason: Optional[str] = Field(None, max_length=2000)
    operator_ack: Optional[str] = Field(None, description="Must be exactly USB_SNIFF for start/stop actions.")
    operator: Optional[str] = Field("bms-cockpit", max_length=200)
    stop_existing: bool = False
    run_id: Optional[str] = Field(None, max_length=120)
    tail: Optional[int] = Field(None, ge=1, le=1000)


class MaintenanceRecoverMotionRequest(BaseModel):
    operator_ack: str = Field(
        ...,
        description=f"Must be exactly {MAINTENANCE_RECOVERY_ACK!r}; prevents accidental unblocking after USB/direct maintenance.",
    )
    operator_note: Optional[str] = Field(None, max_length=2000)
    include_diag: bool = True


class OemStartupStepRequest(BaseModel):
    step: str = Field(..., pattern=r"^(z-home|gripper-clear|gripper-home|x-home|x-park-6000|y-home|door-home|y-set-home)$")
    timeout_s: float = Field(25.0, gt=0.1, le=90.0)


class GripperActionRequest(BaseModel):
    operator_ack: str = Field(..., description="Must be GRIPPER_CLEAR or GRIPPER_HOME depending on the operation.")
    reason: str = Field(..., min_length=1, max_length=2000)
    timeout_s: float = Field(15.0, gt=0.1, le=90.0)
    capture_bundle: bool = False
    operator: Optional[str] = Field("bms-cockpit", max_length=200)


class GripperRestoreIdleRequest(BaseModel):
    reason: str = Field("operator_restore_idle_current", max_length=2000)
    operator: Optional[str] = Field("bms-cockpit", max_length=200)


class ThermalDoorActionRequest(BaseModel):
    operator_ack: str = Field(..., description="Must be HOME_THERMAL_DOOR, OPEN_THERMAL_DOOR, or CLOSE_THERMAL_DOOR for the selected route.")
    reason: str = Field(..., min_length=1, max_length=2000)
    timeout_s: float = Field(20.0, gt=0.1, le=60.0)
    capture_bundle: bool = False
    operator: Optional[str] = Field("bms-cockpit", max_length=200)


class OemHomeXYRequest(BaseModel):
    operator_ack: str = Field(..., description="Must be exactly HOMEXY for the direct OEM HomeXY mode.")
    timeout_s: float = Field(30.0, gt=0.1, le=120.0)
    allow_implementation_mapped_predicate: bool = Field(
        False,
        description="Supervised commissioning override; each axis still uses guarded switch-search and does not collapse to /motion/axis/zero.",
    )


class OemRehomeRequest(BaseModel):
    operator_ack: str = Field(..., description="Must be exactly REHOME for the direct ControlLib.rehome wrapper.")
    run_homing: bool = Field(False, description="Default false returns fail-closed/no-motion route metadata; true runs the monolithic wrapper.")
    timeout_s: float = Field(120.0, gt=1.0, le=240.0)


class OemInitializeMotionRequest(BaseModel):
    operator_ack: str = Field(..., description="INITIALIZE for no-homing diagnostic; INITIALIZE_WITH_HOMING for homing body.")
    run_homing: bool = Field(False, description="False calls initialize-without-motion only; true delegates to the homing/rehome wrapper.")
    include_tip_pipette_cleanup: bool = Field(False, description="Reserved label only; cleanup remains not ported until source-equivalent primitives exist.")
    timeout_s: float = Field(120.0, gt=1.0, le=240.0)


class OemInitializationRunRequest(BaseModel):
    operator_ack: str = Field(..., description="OEM_INITIALIZATION_RUN for no-homing dry/controller pass; OEM_INITIALIZATION_RUN_WITH_HOMING for live homing body.")
    run_homing: bool = Field(False, description="False executes prep/controller phases without homing; true runs source-shaped homing/controller phases.")
    restore_door_state: bool = Field(False, description="Request explicit source-mode door restore policy after init. Open restore remains explicit, not silent.")
    include_tip_pipette_cleanup: bool = Field(False, description="Reserved label; cleanup remains explicitly unsupported until source-equivalent primitives exist.")
    timeout_s: float = Field(180.0, gt=1.0, le=300.0)


OEM_IDLE_STANDBY_CURRENT = 10


class MotionAxisCurrentRequest(BaseModel):
    axes: list[AxisName] = Field(default_factory=lambda: [AxisName.X, AxisName.Y, AxisName.Z])
    run_current: int = Field(OEM_IDLE_STANDBY_CURRENT, ge=0, le=31)
    # OEM source separates explicit movement current from low gripper/idle hold current:
    # DefaultParameters.G_MOTOR_HOLD_CURRENT=10 and ClassControlInterface.enableXYZ(false)
    # drops X/Y/Z to current=1. Never default the raw route to full-current standby.
    standby_current: int = Field(OEM_IDLE_STANDBY_CURRENT, ge=0, le=31)
    operator_ack: bool = Field(False, description="Required with commissioning_override for standby_current above OEM idle.")
    commissioning_override: bool = Field(False, description="Explicit troubleshooting override for hot standby current; not a default path.")


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
    try:
        preset = tester._motion_oem_axis_profile(axis.value)
    except Exception:
        preset = tester.motor_function_preset(axis.value)
    if not isinstance(preset, dict):
        raise HTTPException(status_code=404, detail=f"Unknown axis preset for {axis.value}")
    out = dict(preset)
    out.setdefault("speed", _DEFAULT_MOTION_SPEED)
    out.setdefault("acc", _DEFAULT_MOTION_ACC)
    return out


def _switch_activity_from_switches(tester: BioXpTester, board: int, motor: int, switches: Optional[dict]) -> dict:
    left = None
    right = None
    left_disabled = False
    right_disabled = False
    left_raw_active = None
    right_raw_active = None
    if isinstance(switches, dict):
        left = switches.get("left_state")
        right = switches.get("right_state")
        left_disabled = bool(switches.get("left_disabled", False))
        right_disabled = bool(switches.get("right_disabled", False))
        left_raw_active = switches.get("left_raw_active")
        right_raw_active = switches.get("right_raw_active")
    active_val = int(tester.MOTOR_SWITCH_ACTIVE_VALUE)
    if left_raw_active is None and left is not None:
        left_raw_active = int(left) == active_val
    if right_raw_active is None and right is not None:
        right_raw_active = int(right) == active_val
    return {
        "board": int(board),
        "motor": int(motor),
        "active_raw_value": active_val,
        "left_state": left,
        "right_state": right,
        "left_disabled": left_disabled,
        "right_disabled": right_disabled,
        "left_raw_active": left_raw_active,
        "right_raw_active": right_raw_active,
        "left_active": None if left_raw_active is None else (bool(left_raw_active) and not left_disabled),
        "right_active": None if right_raw_active is None else (bool(right_raw_active) and not right_disabled),
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
        status["standby_current"] = tester.motor_get_axis_param(board, 7, motor=motor)
    current_safety = None
    if axis == AxisName.GRIPPER and include_current:
        speed_value = status.get("speed", {}).get("speed") if isinstance(status.get("speed"), dict) else None
        run_value = status.get("max_current", {}).get("value") if isinstance(status.get("max_current"), dict) else None
        standby_value = status.get("standby_current", {}).get("value") if isinstance(status.get("standby_current"), dict) else None
        unsafe_hot_idle = bool(speed_value == 0 and ((isinstance(run_value, int) and run_value > OEM_IDLE_STANDBY_CURRENT) or (isinstance(standby_value, int) and standby_value > OEM_IDLE_STANDBY_CURRENT)))
        current_safety = {
            "classification": "G_CURRENT_UNSAFE_HOT_IDLE" if unsafe_hot_idle else "G_CURRENT_IDLE_SAFE",
            "speed": speed_value,
            "run_current_param6": run_value,
            "standby_current_param7": standby_value,
            "safe_idle_max": OEM_IDLE_STANDBY_CURRENT,
            "motion_commanded": False,
        }
    return {
        "axis": axis.value,
        "preset": preset,
        "status": status,
        "switch_activity": _switch_activity_from_switches(tester, board, motor, switches),
        "current_safety": current_safety,
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
        rows[axis.value] = _axis_status_payload(tester, axis, include_current=(axis == AxisName.GRIPPER))
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


def _motion_request_is_validation_only(req: MotionArtifactRequest) -> bool:
    return bool(getattr(req, "capture_bundle", False)) and bool(getattr(req, "dry_run_bundle", False))


def _require_motion_route_ready(req: Optional[MotionArtifactRequest] = None) -> None:
    if req is not None and _motion_request_is_validation_only(req):
        return
    _require_motion_not_blocked_by_maintenance()


def _reference_row_for_axis(axis: AxisName) -> dict:
    snapshot = _reference_state_store.snapshot([axis])
    rows = snapshot.get("rows", {}) if isinstance(snapshot, dict) else {}
    return rows.get(axis.value, {}) if isinstance(rows, dict) else {}


def _require_axis_not_operator_desynced(axis: AxisName, *, command: str) -> None:
    row = _reference_row_for_axis(axis)
    if isinstance(row, dict) and row.get("state") == "desynced":
        raise HTTPException(
            status_code=409,
            detail={
                "message": (
                    f"Axis {axis.value} reference is marked desynced; refusing {command} "
                    "until the operator re-establishes a trusted physical reference."
                ),
                "axis": axis.value,
                "state": row,
                "blocked_command": command,
                "reason": "operator_physical_truth_over_controller_counter_truth",
            },
        )


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


def _motion_failure_truth_payload(message: str) -> dict:
    payload = _motion_truth_payload()
    payload["summary"] = f"No confirmed physical displacement: {message}"
    payload["physical_motion_confirmed"] = False
    payload["controller_motion_ambiguous"] = True
    return payload


def _motion_failure_response(
    *,
    axis: AxisName,
    category: str,
    message: str,
    board_status: Any,
    interlock: Any,
    prep: Any,
    prep_policy: dict,
    motion_profile: dict,
    position_before: Optional[dict],
    position_after: Optional[dict],
    switch_before: Optional[dict],
    switch_after: Optional[dict],
    move: Optional[dict] = None,
    wait: Optional[dict] = None,
    target_position: Optional[int] = None,
    raw_events: Optional[list[dict]] = None,
    event_capture_attempted: bool = False,
) -> dict:
    response = {
        "ok": False,
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "prep_policy": prep_policy,
        "motion_truth": _motion_failure_truth_payload(message),
        "motion_failure": {
            "category": category,
            "message": message,
            "route_error": False,
            "http_status_previously": 409,
            "physical_motion_confirmed": False,
            "requires_hardware_rca": True,
        },
        "motion_profile": motion_profile,
        "position_before": position_before,
        "position_after": position_after,
        "position_delta": _position_delta(position_before, position_after),
        "switch_activity_before": switch_before,
        "switch_activity_after": switch_after,
    }
    if target_position is not None:
        response["target_position"] = int(target_position)
    if move is not None:
        response["move"] = move
    if wait is not None:
        response["wait"] = wait
    response["motion_evidence"] = _build_motion_evidence(
        preset={},
        prep=prep,
        interlock=interlock,
        position_before=position_before,
        switch_before=switch_before,
        position_after=position_after,
        switch_after=switch_after,
        move=move,
        wait=wait,
        motion_profile=motion_profile,
        raw_events=raw_events,
        event_capture_attempted=event_capture_attempted,
    )
    return response


def _trim_motion_ack(ack: Any) -> Any:
    if not isinstance(ack, dict):
        return ack
    return {k: ack.get(k) for k in ("board", "cmd", "status", "status_str", "value") if k in ack}


def _extract_motion_value(row: Any) -> Any:
    if not isinstance(row, dict):
        return None
    if "value" in row:
        return row.get("value")
    rb = row.get("readback")
    if isinstance(rb, dict):
        return rb.get("value")
    return None


def _prep_ops_by_name(prep: Any) -> dict[str, dict]:
    if not isinstance(prep, dict):
        return {}
    ops = prep.get("ops")
    if not isinstance(ops, list):
        return {}
    out: dict[str, dict] = {}
    for op in ops:
        if isinstance(op, dict) and isinstance(op.get("op"), str):
            out[op["op"]] = op
    return out


def _prep_param_summary(ops: dict[str, dict], op_name: str, *, fallback_set: Any = None, as_bool: bool = False) -> dict:
    op = ops.get(op_name) if isinstance(ops, dict) else None
    set_value = op.get("set") if isinstance(op, dict) and "set" in op else fallback_set
    readback = _extract_motion_value(op.get("rb")) if isinstance(op, dict) else None
    if as_bool and set_value is not None:
        set_value = bool(set_value)
    if as_bool and readback is not None:
        readback = bool(readback)
    ack = _trim_motion_ack(op.get("ack")) if isinstance(op, dict) else None
    ok = None
    if isinstance(ack, dict) and "status" in ack:
        ok = ack.get("status") == 100
    return {"set": set_value, "readback": readback, "ack": ack, "ok": ok}


def _normalize_motion_prep_params(preset: Any, prep: Any, motion_profile: Optional[dict] = None) -> dict:
    preset = preset if isinstance(preset, dict) else {}
    motion_profile = motion_profile if isinstance(motion_profile, dict) else {}
    ops = _prep_ops_by_name(prep)
    return {
        "speed": motion_profile.get("speed", preset.get("speed")),
        "acc": motion_profile.get("acc", preset.get("acc")),
        "run_current_param6": _prep_param_summary(ops, "sap6-run_current", fallback_set=preset.get("run_current")),
        "standby_current_param7": _prep_param_summary(ops, "sap7-standby_current", fallback_set=preset.get("standby_current")),
        "speed_param4": _prep_param_summary(ops, "sap4-max_speed", fallback_set=motion_profile.get("speed", preset.get("speed"))),
        "acc_param5": _prep_param_summary(ops, "sap5-max_acc", fallback_set=motion_profile.get("acc", preset.get("acc"))),
        "stallguard_param205": _prep_param_summary(ops, "sap205-stall_guard", fallback_set=preset.get("stall_guard")),
        "switch_masks": {
            "right_param12": _prep_param_summary(ops, "sap12-disable_right", fallback_set=preset.get("disable_right"), as_bool=True),
            "left_param13": _prep_param_summary(ops, "sap13-disable_left", fallback_set=preset.get("disable_left"), as_bool=True),
        },
        "source": "motor_prepare_axis.ops",
    }


_MOTION_EVENT_BUCKETS = {
    128: "target_reached_128",
    130: "stallguard_130",
    13: "voltage_drop_13",
    14: "max_current_reference_deviation_14",
    132: "door_latch_132",
}


def _event_matches_motion_axis(event: dict, preset: Any) -> bool:
    if not isinstance(event, dict):
        return False
    if not isinstance(preset, dict):
        return True
    expected_board = preset.get("board")
    expected_motor = preset.get("motor")
    try:
        if expected_board is not None and event.get("board") is not None and int(event.get("board")) != int(expected_board):
            return False
    except (TypeError, ValueError):
        return False
    # OEM dispatches TARGET_POSITION_REACHED/STALL_GUARD to a specific motor.
    # If the event carries a motor byte, it must match. If it does not carry a
    # motor byte, do not use it as positive target-reached proof.
    if event.get("status") == 128 and event.get("motor") is None:
        return False
    try:
        if expected_motor is not None and event.get("motor") is not None and int(event.get("motor")) != int(expected_motor):
            return False
    except (TypeError, ValueError):
        return False
    return True


def _filter_motion_events_for_axis(raw_events: Optional[list[dict]], preset: Any) -> list[dict]:
    out = []
    for event in raw_events or []:
        if not isinstance(event, dict):
            continue
        status = event.get("status")
        try:
            status_int = int(status)
        except (TypeError, ValueError):
            out.append(event)
            continue
        if status_int == 128:
            if _event_matches_motion_axis(event, preset):
                out.append(event)
            continue
        if status_int in {130, 14, 13}:
            # Error events fail closed. If motor-specific, require matching motor;
            # if not motor-specific, keep them because OEM raises board errors.
            if event.get("motor") is None or _event_matches_motion_axis(event, preset):
                out.append(event)
            continue
        out.append(event)
    return out


def _classify_motion_events(raw_events: Optional[list[dict]] = None, *, capture_attempted: bool = False) -> dict:
    raw = [event for event in (raw_events or []) if isinstance(event, dict)]
    out = {"capture_attempted": bool(capture_attempted), "raw": raw, "captured_count": len(raw)}
    for bucket in _MOTION_EVENT_BUCKETS.values():
        out[bucket] = []
    for event in raw:
        status = event.get("status")
        try:
            status_int = int(status)
        except (TypeError, ValueError):
            continue
        bucket = _MOTION_EVENT_BUCKETS.get(status_int)
        if bucket:
            out[bucket].append(event)
    out["bucket_counts"] = {bucket: len(out[bucket]) for bucket in _MOTION_EVENT_BUCKETS.values()}
    if not capture_attempted:
        out["note"] = "Async bus-event capture is deliberately not enabled in the default movement path; no USB read-stream drain was added."
    elif not raw:
        out["note"] = "USB bus-event capture was attempted for this motion window, but no non-heartbeat async frames were observed."
    return out


def _dedupe_motion_events(events: list[dict]) -> list[dict]:
    out: list[dict] = []
    seen = set()
    for event in events:
        if not isinstance(event, dict):
            continue
        raw = event.get("raw")
        if isinstance(raw, list):
            key = tuple(raw)
        else:
            key = (event.get("board"), event.get("status"), event.get("cmd"), event.get("value"), event.get("observed_ms"))
        if key in seen:
            continue
        seen.add(key)
        out.append(event)
    return out


def _clear_motion_event_capture(tester: BioXpTester) -> None:
    clear = getattr(tester, "clear_bus_event_buffer", None)
    drain = getattr(tester, "drain", None)
    if callable(clear):
        try:
            clear()
        except Exception:
            pass
    drained = False
    if callable(drain):
        try:
            # Flush endpoint-stale async frames immediately before the motion window.
            # drain() preserves frames into the buffer, so clear again afterward.
            drain(max_reads=12, timeout_ms=2)
            drained = True
        except Exception:
            pass
    if drained and callable(clear):
        try:
            clear()
        except Exception:
            pass


def _collect_motion_event_capture(tester: BioXpTester) -> tuple[list[dict], bool]:
    raw_events: list[dict] = []
    attempted = False
    pop = getattr(tester, "pop_bus_event_buffer", None)
    if callable(pop):
        attempted = True
        try:
            popped = pop()
            if isinstance(popped, list):
                raw_events.extend(event for event in popped if isinstance(event, dict))
        except Exception as exc:
            raw_events.append({"status": None, "source": "pop_bus_event_buffer_error", "error": repr(exc)})
    collect = getattr(tester, "collect_bus_events", None)
    if callable(collect):
        attempted = True
        try:
            tail = collect(duration_s=0.25, timeout_ms=12, max_events=96)
            if isinstance(tail, list):
                raw_events.extend(event for event in tail if isinstance(event, dict))
        except Exception as exc:
            raw_events.append({"status": None, "source": "collect_bus_events_error", "error": repr(exc)})
    return _dedupe_motion_events(raw_events), attempted


def _motion_log_nonzero_speed(wait: Any) -> bool:
    if isinstance(wait, dict) and wait.get("seen_nonzero") is True:
        return True
    tail = wait.get("log_tail") if isinstance(wait, dict) else None
    if not isinstance(tail, list):
        return False
    for row in tail:
        if not isinstance(row, dict):
            continue
        speed = row.get("speed")
        if isinstance(speed, int) and speed != 0:
            return True
    return False


def _gap8_target_reached_value(wait: Any) -> Any:
    if not isinstance(wait, dict):
        return None
    row = wait.get("reached_position_after")
    if isinstance(row, dict) and "target_reached" in row:
        return row.get("target_reached")
    tail = wait.get("log_tail")
    if isinstance(tail, list):
        for entry in reversed(tail):
            if isinstance(entry, dict) and "target_reached" in entry:
                return entry.get("target_reached")
    return None


def _gap8_target_confirmed(wait: Any) -> bool:
    # Decompiled OEM ClassMotor.queryReachedPosition() only treats GAP8 query ACK
    # success as method return 0. It is tracked as telemetry but not used as the
    # normal movement-complete proof; async event 128 is the proof.
    return False


def _first_controller_motion_error(events: Any) -> Optional[dict]:
    if not isinstance(events, dict):
        return None
    for bucket, status in (("stallguard_130", 130), ("max_current_reference_deviation_14", 14), ("voltage_drop_13", 13)):
        rows = events.get(bucket)
        if isinstance(rows, list) and rows:
            event = rows[0] if isinstance(rows[0], dict) else {"raw": rows[0]}
            out = dict(event)
            out.setdefault("status", status)
            out.setdefault("bucket", bucket)
            return out
    return None


def _motion_telemetry_packet(position_before: Any, switch_before: Any, position_after: Any, switch_after: Any, wait: Any) -> dict:
    during = wait.get("log_tail") if isinstance(wait, dict) and isinstance(wait.get("log_tail"), list) else []
    last_speed = wait.get("last_speed") if isinstance(wait, dict) else None
    gap8_value = _gap8_target_reached_value(wait)
    gap8_after = wait.get("reached_position_after") if isinstance(wait, dict) else None
    gap8_available = isinstance(gap8_after, dict)
    return {
        "before": {
            "gap1_position": position_before,
            "gap3_speed": None,
            "gap8_target_reached": {"available": False, "value": None, "source": "not_sampled_before_move"},
            "gap9_gap10_switches": switch_before,
        },
        "during": during,
        "after": {
            "gap1_position": position_after,
            "gap3_speed": {"speed": last_speed} if last_speed is not None else None,
            "gap8_target_reached": {"available": gap8_available, "value": gap8_value, "row": gap8_after, "source": "GAP8/ClassMotor.queryReachedPosition" if gap8_available else "not_available"},
            "gap9_gap10_switches": switch_after,
        },
    }


def _motion_classification(
    move: Any,
    wait: Any,
    position_before: Any,
    position_after: Any,
    events: dict,
    *,
    switch_transition: bool = False,
    home_predicate_confirmed: bool = False,
) -> dict:
    delta = _position_delta(position_before, position_after)
    nonzero_speed_seen = _motion_log_nonzero_speed(wait)
    target_event_seen = bool(events.get("target_reached_128")) if isinstance(events, dict) else False
    stall_event_seen = bool(events.get("stallguard_130")) if isinstance(events, dict) else False
    voltage_drop_seen = bool(events.get("voltage_drop_13")) if isinstance(events, dict) else False
    max_current_reference_deviation_seen = bool(events.get("max_current_reference_deviation_14")) if isinstance(events, dict) else False
    door_latch_event_seen = bool(events.get("door_latch_132")) if isinstance(events, dict) else False
    gap8_value = _gap8_target_reached_value(wait)
    gap8_confirmed = _gap8_target_confirmed(wait)
    controller_error_seen = bool(stall_event_seen or voltage_drop_seen or max_current_reference_deviation_seen)
    move_ack_ok = bool(isinstance(move, dict) and move.get("ok") is True)
    controller_target_confirmed = bool(target_event_seen or gap8_confirmed)
    # GAP1/GAP3 deltas are not independent proof of motion because the controller
    # can report phantom position/speed when the axis is stalled or desynced.
    # Accept only source-shaped controller evidence:
    # (a) a target-reached event for normal moves, or
    # (b) the home predicate confirmed after stop + setHome for homing.
    # A switch transition is retained as telemetry, but OEM goHome ultimately keys
    # on queryHome() active after the search, not on our Linux-only requirement that
    # an inactive sample must have appeared in the polling window.
    controller_motion_evidence = bool(
        move_ack_ok
        and not controller_error_seen
        and (controller_target_confirmed or home_predicate_confirmed)
    )
    return {
        "move_ack_ok": move_ack_ok,
        "controller_motion_evidence": controller_motion_evidence,
        "stall_event_seen": stall_event_seen,
        "target_reached_event_seen": target_event_seen,
        "voltage_drop_seen": voltage_drop_seen,
        "max_current_reference_deviation_seen": max_current_reference_deviation_seen,
        "door_latch_event_seen": door_latch_event_seen,
        "position_delta": delta,
        "nonzero_speed_seen": nonzero_speed_seen,
        "gap8_target_reached": gap8_value,
        "controller_target_confirmed": controller_target_confirmed,
        "controller_error_seen": controller_error_seen,
        "switch_transition": switch_transition,
        "home_predicate_confirmed": bool(home_predicate_confirmed),
        "physical_motion_confirmed": False,
        "physical_proof_source": None,
        "evidence_level": "controller_only",
    }


def _build_motion_evidence(
    *,
    preset: Any,
    prep: Any,
    interlock: Any,
    position_before: Any,
    switch_before: Any,
    position_after: Any,
    switch_after: Any,
    move: Any,
    wait: Any,
    motion_profile: Optional[dict] = None,
    raw_events: Optional[list[dict]] = None,
    event_capture_attempted: bool = False,
    switch_transition: bool = False,
    home_predicate_confirmed: bool = False,
) -> dict:
    filtered_events = _filter_motion_events_for_axis(raw_events, preset)
    events = _classify_motion_events(filtered_events, capture_attempted=event_capture_attempted)
    telemetry = _motion_telemetry_packet(position_before, switch_before, position_after, switch_after, wait)
    return {
        "prep_params": _normalize_motion_prep_params(preset, prep, motion_profile=motion_profile),
        "telemetry": telemetry,
        "events": events,
        "interlock": {
            "before": interlock.get("snap_before") if isinstance(interlock, dict) else None,
            "after": interlock.get("snap_after") if isinstance(interlock, dict) else None,
            "raw": interlock,
        },
        "classification": _motion_classification(
            move,
            wait,
            position_before,
            position_after,
            events,
            switch_transition=switch_transition,
            home_predicate_confirmed=home_predicate_confirmed,
        ),
        "switch_transition": switch_transition,
        "home_predicate_confirmed": bool(home_predicate_confirmed),
    }


def _build_home_motion_evidence(*, preset: Any, prep: Any, interlock: Any, home: Any, motion_profile: Optional[dict] = None) -> dict:
    payload = _unwrap_oem_home_payload(home)
    position_before = payload.get("position_before") if isinstance(payload, dict) else None
    position_after = payload.get("position_after") if isinstance(payload, dict) else None
    switch_before = payload.get("switch_activity_before") if isinstance(payload, dict) else None
    switch_after = payload.get("switch_activity_after") if isinstance(payload, dict) else None
    switch_transition = bool(payload.get("switch_transition")) if isinstance(payload, dict) else False
    active_value = payload.get("home_active_value") if isinstance(payload, dict) else None
    home_after = payload.get("home_after") if isinstance(payload, dict) else None
    set_home = payload.get("set_home") if isinstance(payload, dict) else None
    home_after_active = bool(
        active_value is not None
        and isinstance(home_after, dict)
        and home_after.get("value") is not None
        and int(home_after.get("value")) == int(active_value)
    )
    set_home_ok = bool(isinstance(set_home, dict) and set_home.get("ok") is True)
    # OEM goHome proof is queryHome() active after the search/stop followed by setHome().
    # The inactive->active transition is useful telemetry, not a Linux-only hard gate.
    home_predicate_confirmed = bool(home_after_active and set_home_ok and payload.get("ok") is True)
    wait = {
        "ok": payload.get("ok") if isinstance(payload, dict) else None,
        "log_tail": payload.get("log_tail") if isinstance(payload, dict) else [],
        "seen_nonzero": _motion_log_nonzero_speed({"log_tail": payload.get("log_tail")}) if isinstance(payload, dict) else False,
        "home_after_active": home_after_active,
        "set_home_ok": set_home_ok,
    }
    move = payload.get("move_left") or payload if isinstance(payload, dict) else None
    return _build_motion_evidence(
        preset=preset,
        prep=prep,
        interlock=interlock,
        position_before=position_before,
        switch_before=switch_before,
        position_after=position_after,
        switch_after=switch_after,
        move=move,
        wait=wait,
        motion_profile=motion_profile,
        switch_transition=switch_transition,
        home_predicate_confirmed=home_predicate_confirmed,
    )


def _motion_interlock_override_reason(req: MotionInterlockOverrideRequest) -> str:
    reason = str(req.reason if req.reason is not None else (req.operator_note or "")).strip()
    if bool(req.enabled) and not reason:
        raise HTTPException(status_code=409, detail="enabled interlock override requires a non-empty reason or operator_note")
    parts = ["api_interlock_override"]
    if reason:
        parts.append(reason[:2000])
    return " | ".join(parts)


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
    """Do not software-block supervised operator moves from raw limit-switch telemetry.

    The live BioXP switch lines can report impossible combinations (for example both
    left and right active on an axis that the operator can physically see is at one
    end of travel). Treat those rows as readback/diagnostic telemetry only here;
    low-level controller behavior, no-delta timeout/stop handling, emergency stop,
    and the watching operator remain the live safety boundary.
    """
    return


def _axis_limit_row(axis: AxisName) -> dict:
    config = harmonized_motion_config()
    limits = config.get("axis_limits", {}) if isinstance(config, dict) else {}
    row = limits.get(axis.value) if isinstance(limits, dict) else None
    out = dict(row) if isinstance(row, dict) else {}
    if axis == AxisName.Z:
        source_min = _limit_int(out, "min_steps")
        source_max = _limit_int(out, "max_steps")
        # OEM ClassBioXPSettings stores source-space Z as 0..ZHighLimit,
        # while the commissioned live controller reports upward travel as
        # negative counts with reference at zero. Preserve both contracts.
        if (
            source_min is not None
            and source_max is not None
            and source_min >= 0
            and source_max >= source_min
        ):
            out.update({
                "source_min_steps": source_min,
                "source_max_steps": source_max,
                "min_steps": -source_max,
                "max_steps": 0,
                "coordinate_contract": "observed_live_signed_z",
                "source_coordinate_contract": "oem_source_nonnegative_z",
                "oem_equivalent": False,
            })
    return out


def _limit_int(row: dict, key: str) -> Optional[int]:
    try:
        value = row.get(key)
        return None if value is None else int(value)
    except (TypeError, ValueError):
        return None


def _guard_axis_position_within_oem_limits(axis: AxisName, position_row: Optional[dict], *, command: str) -> None:
    position = _position_value(position_row)
    if position is None:
        return
    limit = _axis_limit_row(axis)
    min_steps = _limit_int(limit, "min_steps")
    max_steps = _limit_int(limit, "max_steps")
    if (min_steps is not None and int(position) < min_steps) or (max_steps is not None and int(position) > max_steps):
        raise HTTPException(
            status_code=409,
            detail={
                "message": f"Axis {axis.value} controller position is outside the OEM configured envelope; refusing {command}.",
                "axis": axis.value,
                "position_steps": int(position),
                "configured_limit": limit,
                "motion_blocked": True,
                "reason": "controller_position_outside_oem_axis_limits",
            },
        )


def _guard_absolute_target(
    axis: AxisName,
    current_position: Optional[dict],
    target_steps: int,
    switch_activity: Optional[dict],
    preset: Optional[dict] = None,
) -> None:
    current = _position_value(current_position)
    limit = _axis_limit_row(axis)
    min_steps = _limit_int(limit, "min_steps")
    max_steps = _limit_int(limit, "max_steps")
    target = int(target_steps)
    if (min_steps is not None and target < min_steps) or (max_steps is not None and target > max_steps):
        raise HTTPException(
            status_code=409,
            detail={
                "message": f"Axis {axis.value} target is outside the configured live envelope; refusing motion.",
                "axis": axis.value,
                "current_position_steps": None if current is None else int(current),
                "target_position_steps": target,
                "configured_limit": limit,
                "motion_blocked": True,
                "reason": "target_outside_oem_axis_limits",
            },
        )
    if current is None:
        raise HTTPException(
            status_code=409,
            detail={
                "message": f"Axis {axis.value} current controller position is unavailable; refusing motion.",
                "axis": axis.value,
                "target_position_steps": target,
                "configured_limit": limit,
                "motion_blocked": True,
                "reason": "current_position_unavailable_for_motion_guard",
            },
        )
    _guard_axis_position_within_oem_limits(axis, current_position, command="axis absolute/relative move")
    _guard_direction(axis, target - int(current), switch_activity, preset)


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
        reached_row = {"available": False, "target_reached": None, "source": "motor_get_reached_position_unavailable"}
        get_reached = getattr(tester, "motor_get_reached_position", None)
        if callable(get_reached):
            reached_row = get_reached(board, motor=motor)
        now = time.monotonic()
        speed = _speed_value(speed_row)
        position = _position_value(position_row)
        target_reached = reached_row.get("target_reached") if isinstance(reached_row, dict) else None
        polls += 1
        if isinstance(speed, int) and speed != 0:
            seen_nonzero = True

        if position is not None and last_position is None:
            last_position = position
            last_progress_at = now
        elif position is not None and last_position is not None and position != last_position:
            last_position = position
            last_progress_at = now

        log_tail.append({"elapsed_ms": int((now - started) * 1000), "speed": speed, "position": position, "target_reached": target_reached})
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
                    "reached_position_after": reached_row,
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
                "reached_position_after": reached_row,
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
                "reached_position_after": reached_row,
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
    speed: Optional[int] = None,
    acc: Optional[int] = None,
    reuse_prepared: bool = False,
) -> dict:
    _require_motion_not_blocked_by_maintenance()
    preset, board_status, interlock, prep, prep_policy = _prepare_motion_axis(
        tester,
        axis,
        speed=speed,
        acc=acc,
        reuse_prepared=reuse_prepared,
    )
    position_before = tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_before = tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    current = _position_value(position_before)
    if current is not None:
        _guard_absolute_target(axis, position_before, int(current) + int(steps), switch_before, preset)
    else:
        _guard_direction(axis, steps, switch_before, preset)
    motion_profile = {
        "speed": int(preset["speed"]),
        "acc": int(preset["acc"]),
        "requested_speed": None if speed is None else int(speed),
        "requested_acc": None if acc is None else int(acc),
        "normalization": preset.get("speed_acc_normalization"),
        "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
    }
    _clear_motion_event_capture(tester)
    move = tester.motor_move_relative(preset["board"], steps, motor=preset["motor"])
    if not move.get("ok"):
        raw_events, event_capture_attempted = _collect_motion_event_capture(tester)
        return _motion_failure_response(
            axis=axis,
            category="controller_command_failed",
            message=f"Axis {axis.value} relative move command was not acknowledged by the controller.",
            board_status=board_status,
            interlock=interlock,
            prep=prep,
            prep_policy=prep_policy,
            motion_profile=motion_profile,
            position_before=position_before,
            position_after=None,
            switch_before=switch_before,
            switch_after=None,
            move=move,
            raw_events=raw_events,
            event_capture_attempted=event_capture_attempted,
        )
    wait = _wait_for_motion_with_guardrails(tester, preset["board"], preset["motor"], timeout_s=wait_timeout_s)
    if not wait.get("ok"):
        raw_events, event_capture_attempted = _collect_motion_event_capture(tester)
        return _motion_failure_response(
            axis=axis,
            category="guardrail_no_motion",
            message=str(wait.get("error") or f"Axis {axis.value} motion was not confirmed."),
            board_status=board_status,
            interlock=interlock,
            prep=prep,
            prep_policy=prep_policy,
            motion_profile=motion_profile,
            position_before=position_before,
            position_after=wait.get("position_after"),
            switch_before=switch_before,
            switch_after=wait.get("switch_activity_after"),
            move=move,
            wait=wait,
            raw_events=raw_events,
            event_capture_attempted=event_capture_attempted,
        )
    position_after = wait.get("position_after") or tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_after = wait.get("switch_activity_after") or tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    motion_profile = {
        "speed": int(preset["speed"]),
        "acc": int(preset["acc"]),
        "requested_speed": None if speed is None else int(speed),
        "requested_acc": None if acc is None else int(acc),
        "normalization": preset.get("speed_acc_normalization"),
        "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
    }
    raw_events, event_capture_attempted = _collect_motion_event_capture(tester)
    events = _classify_motion_events(_filter_motion_events_for_axis(raw_events, preset), capture_attempted=event_capture_attempted)
    controller_error = _first_controller_motion_error(events)
    if controller_error is not None:
        response = _motion_failure_response(
            axis=axis,
            category="controller_motion_error",
            message=f"Axis {axis.value} controller reported motion error event {controller_error.get('status')} during relative move.",
            board_status=board_status,
            interlock=interlock,
            prep=prep,
            prep_policy=prep_policy,
            motion_profile=motion_profile,
            position_before=position_before,
            position_after=position_after,
            switch_before=switch_before,
            switch_after=switch_after,
            move=move,
            wait=wait,
            raw_events=raw_events,
            event_capture_attempted=event_capture_attempted,
        )
        response["motion_failure"]["event_status"] = controller_error.get("status")
        response["motion_failure"]["event_bucket"] = controller_error.get("bucket")
        response["motion_failure"]["event"] = controller_error
        return response
    evidence = _build_motion_evidence(
        preset=preset,
        prep=prep,
        interlock=interlock,
        position_before=position_before,
        switch_before=switch_before,
        position_after=position_after,
        switch_after=switch_after,
        move=move,
        wait=wait,
        motion_profile=motion_profile,
        raw_events=raw_events,
        event_capture_attempted=event_capture_attempted,
    )
    if not (((evidence.get("classification") or {}).get("controller_motion_evidence")) is True):
        return _motion_failure_response(
            axis=axis,
            category="oem_motion_evidence_missing",
            message=f"Axis {axis.value} move lacked OEM target-reached/switch-transition evidence; refusing success from GAP1/GAP3 alone.",
            board_status=board_status,
            interlock=interlock,
            prep=prep,
            prep_policy=prep_policy,
            motion_profile=motion_profile,
            position_before=position_before,
            position_after=position_after,
            switch_before=switch_before,
            switch_after=switch_after,
            move=move,
            wait=wait,
            raw_events=raw_events,
            event_capture_attempted=event_capture_attempted,
        )
    return {
        "ok": True,
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "prep_policy": prep_policy,
        "motion_truth": _motion_truth_payload(),
        "motion_evidence": evidence,
        "motion_profile": motion_profile,
        "position_before": position_before,
        "position_after": position_after,
        "position_delta": _position_delta(position_before, position_after),
        "switch_activity_before": switch_before,
        "switch_activity_after": switch_after,
        "move": move,
        "wait": wait,
    }


def _execute_absolute_move(
    tester: BioXpTester,
    axis: AxisName,
    position_steps: int,
    wait_timeout_s: float,
    *,
    speed: Optional[int] = None,
    acc: Optional[int] = None,
) -> dict:
    _require_motion_not_blocked_by_maintenance()
    preset, board_status, interlock, prep, prep_policy = _prepare_motion_axis(tester, axis, speed=speed, acc=acc)
    position_before = tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_before = tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    _guard_absolute_target(axis, position_before, position_steps, switch_before, preset)
    motion_profile = {
        "speed": int(preset["speed"]),
        "acc": int(preset["acc"]),
        "requested_speed": None if speed is None else int(speed),
        "requested_acc": None if acc is None else int(acc),
        "normalization": preset.get("speed_acc_normalization"),
        "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
    }
    _clear_motion_event_capture(tester)
    move = tester.motor_move_absolute(preset["board"], position_steps, motor=preset["motor"])
    if not move.get("ok"):
        raw_events, event_capture_attempted = _collect_motion_event_capture(tester)
        return _motion_failure_response(
            axis=axis,
            category="controller_command_failed",
            message=f"Axis {axis.value} absolute move command was not acknowledged by the controller.",
            board_status=board_status,
            interlock=interlock,
            prep=prep,
            prep_policy=prep_policy,
            motion_profile=motion_profile,
            position_before=position_before,
            position_after=None,
            switch_before=switch_before,
            switch_after=None,
            move=move,
            target_position=position_steps,
            raw_events=raw_events,
            event_capture_attempted=event_capture_attempted,
        )
    wait = _wait_for_motion_with_guardrails(tester, preset["board"], preset["motor"], timeout_s=wait_timeout_s)
    if not wait.get("ok"):
        raw_events, event_capture_attempted = _collect_motion_event_capture(tester)
        return _motion_failure_response(
            axis=axis,
            category="guardrail_no_motion",
            message=str(wait.get("error") or f"Axis {axis.value} motion was not confirmed."),
            board_status=board_status,
            interlock=interlock,
            prep=prep,
            prep_policy=prep_policy,
            motion_profile=motion_profile,
            position_before=position_before,
            position_after=wait.get("position_after"),
            switch_before=switch_before,
            switch_after=wait.get("switch_activity_after"),
            move=move,
            wait=wait,
            target_position=position_steps,
            raw_events=raw_events,
            event_capture_attempted=event_capture_attempted,
        )
    position_after = wait.get("position_after") or tester.motor_get_position(preset["board"], motor=preset["motor"])
    switch_after = wait.get("switch_activity_after") or tester.motor_get_switch_activity(preset["board"], motor=preset["motor"])
    motion_profile = {
        "speed": int(preset["speed"]),
        "acc": int(preset["acc"]),
        "requested_speed": None if speed is None else int(speed),
        "requested_acc": None if acc is None else int(acc),
        "normalization": preset.get("speed_acc_normalization"),
        "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
    }
    raw_events, event_capture_attempted = _collect_motion_event_capture(tester)
    events = _classify_motion_events(_filter_motion_events_for_axis(raw_events, preset), capture_attempted=event_capture_attempted)
    controller_error = _first_controller_motion_error(events)
    if controller_error is not None:
        response = _motion_failure_response(
            axis=axis,
            category="controller_motion_error",
            message=f"Axis {axis.value} controller reported motion error event {controller_error.get('status')} during absolute move.",
            board_status=board_status,
            interlock=interlock,
            prep=prep,
            prep_policy=prep_policy,
            motion_profile=motion_profile,
            position_before=position_before,
            position_after=position_after,
            switch_before=switch_before,
            switch_after=switch_after,
            move=move,
            wait=wait,
            raw_events=raw_events,
            event_capture_attempted=event_capture_attempted,
        )
        response["motion_failure"]["event_status"] = controller_error.get("status")
        response["motion_failure"]["event_bucket"] = controller_error.get("bucket")
        response["motion_failure"]["event"] = controller_error
        return response
    evidence = _build_motion_evidence(
        preset=preset,
        prep=prep,
        interlock=interlock,
        position_before=position_before,
        switch_before=switch_before,
        position_after=position_after,
        switch_after=switch_after,
        move=move,
        wait=wait,
        motion_profile=motion_profile,
        raw_events=raw_events,
        event_capture_attempted=event_capture_attempted,
    )
    if not (((evidence.get("classification") or {}).get("controller_motion_evidence")) is True):
        return _motion_failure_response(
            axis=axis,
            category="oem_motion_evidence_missing",
            message=f"Axis {axis.value} move lacked OEM target-reached/switch-transition evidence; refusing success from GAP1/GAP3 alone.",
            board_status=board_status,
            interlock=interlock,
            prep=prep,
            prep_policy=prep_policy,
            motion_profile=motion_profile,
            position_before=position_before,
            position_after=position_after,
            switch_before=switch_before,
            switch_after=switch_after,
            move=move,
            wait=wait,
            target_position=position_steps,
            raw_events=raw_events,
            event_capture_attempted=event_capture_attempted,
        )
    return {
        "ok": True,
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "prep_policy": prep_policy,
        "motion_truth": _motion_truth_payload(),
        "motion_evidence": evidence,
        "motion_profile": motion_profile,
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


def _home_predicate_snapshot(tester: BioXpTester, axis: AxisName) -> dict:
    hardware = _BioXpSwitchAuditHardware(tester)
    snapshot = hardware.switch_snapshot(axis.value)
    interpreted = interpret_home_predicate(axis.value, snapshot)
    snapshot["interpreted"] = interpreted
    return snapshot


def _require_home_predicate_guard(axis: AxisName, snapshot: dict, *, allow_implementation_mapped_predicate: bool) -> None:
    interpreted = snapshot.get("interpreted") if isinstance(snapshot, dict) else {}
    confidence = interpreted.get("confidence") if isinstance(interpreted, dict) else None
    verified = confidence in {"source_anchored", "live_verified", "high"}
    if axis.value == "z" and confidence == "implementation_mapped":
        raise HTTPException(
            status_code=409,
            detail={
                "message": (
                    "Z manual switch-search home is fail-closed until the Z GAP9/GAP10 predicate matrix "
                    "is live/source verified; the implementation-mapped override is deliberately disabled for Z."
                ),
                "axis": axis.value,
                "blocked_route": "/motion/axis/home",
                "predicate": interpreted,
                "current_snapshot": snapshot,
                "incident_guard": "manual_z_home_gap9_search_can_ignore_physical_gap10_bottom_reference",
                "safe_alternatives": [
                    "/motion/oem/startup_step for the supervised OEM startup recipe",
                    "/motion/axis/zero for controller-coordinate return-to-zero when operator-supervised",
                    "/motion/axis/relative for bounded operator-watched jogs",
                ],
                "required_fix_before_override": [
                    "bounded Z cmd1/cmd2 x GAP9/GAP10 predicate matrix",
                    "immediate stop on either physical limit predicate",
                    "queryHome active after search/stop and setHome success",
                    "camera/operator confirmation before reporting physical homing",
                ],
            },
        )
    override_allowed = bool(allow_implementation_mapped_predicate) and confidence == "implementation_mapped"
    if verified or override_allowed:
        return
    raise HTTPException(
        status_code=409,
        detail={
            "message": "Manual switch-search home is fail-closed until the per-axis home predicate is live/source verified.",
            "axis": axis.value,
            "blocked_route": "/motion/axis/home",
            "predicate": interpreted,
            "current_snapshot": snapshot,
            "safe_alternatives": [
                "/motion/axis/zero for controller-coordinate return-to-zero",
                "/motion/oem/startup_step for the supervised OEM startup recipe",
                "/motion/axis/relative for operator-watched bounded jogs",
            ],
            "override_for_supervised_commissioning": "Set allow_implementation_mapped_predicate=true only with camera/operator proof; route still requires OEM queryHome/setHome proof.",
        },
    )


def _execute_home_axis(
    tester: BioXpTester,
    axis: AxisName,
    speed: Optional[int],
    timeout_s: float,
    *,
    allow_implementation_mapped_predicate: bool = False,
) -> dict:
    _require_motion_not_blocked_by_maintenance()
    effective_speed = _validate_oem_home_request_speed(tester, axis, speed)
    predicate_snapshot = _home_predicate_snapshot(tester, axis)
    _require_home_predicate_guard(
        axis,
        predicate_snapshot,
        allow_implementation_mapped_predicate=allow_implementation_mapped_predicate,
    )
    preset, board_status, interlock, prep, prep_policy = _prepare_motion_axis(tester, axis, allow_desynced_reference=True)
    position_before = tester.motor_get_position(preset["board"], motor=preset["motor"])
    # Homing is the OEM operation that re-establishes the controller reference.
    # A pre-home GAP1 value outside the post-reference travel envelope is telemetry,
    # not a reason to block the home search before it can repair the reference.
    pre_home_axis_limit = _axis_limit_row(axis)
    transition_home = getattr(tester, "motor_oem_switch_search_home_axis", None)
    if callable(transition_home):
        home = transition_home(
            axis.value,
            speed=effective_speed,
            timeout_s=min(float(timeout_s), 20.0),
        )
    else:
        home = tester.motor_oem_home_axis(
            axis.value,
            speed=effective_speed,
            timeout_s=min(float(timeout_s), 20.0),
            startup=True,
        )
    motion_profile = {
        "requested_speed": effective_speed,
        "preset_speed": int(preset["speed"]),
        "acc": int(preset["acc"]),
        "no_delta_timeout_s": _MOTION_NO_DELTA_TIMEOUT_S,
        "vendor_path": "oem_axis_search_home_queryHome_setHome",
        "predicate_snapshot_before": predicate_snapshot,
        "position_before_home_search": position_before,
        "pre_home_axis_limit": pre_home_axis_limit,
        "pre_home_position_outside_limit_policy": "tracked_not_blocked_home_reestablishes_reference",
        "allow_implementation_mapped_predicate": bool(allow_implementation_mapped_predicate),
    }
    try:
        _ensure_oem_home_succeeded(tester, axis, home)
    except HTTPException as exc:
        message = str(exc.detail)
        return {
            "ok": False,
            "axis": axis.value,
            "board_status": board_status,
            "interlock": interlock,
            "prep": prep,
            "prep_policy": prep_policy,
            "motion_truth": _motion_failure_truth_payload(message),
            "motion_evidence": _build_home_motion_evidence(
                preset=preset,
                prep=prep,
                interlock=interlock,
                home=home,
                motion_profile=motion_profile,
            ),
            "motion_failure": {
                "category": "home_not_confirmed",
                "message": message,
                "route_error": False,
                "http_status_previously": int(exc.status_code),
                "physical_motion_confirmed": False,
                "requires_hardware_rca": True,
            },
            "motion_profile": motion_profile,
            "home": home,
        }
    return {
        "ok": True,
        "axis": axis.value,
        "board_status": board_status,
        "interlock": interlock,
        "prep": prep,
        "prep_policy": prep_policy,
        "motion_truth": _motion_truth_payload(),
        "motion_evidence": _build_home_motion_evidence(
            preset=preset,
            prep=prep,
            interlock=interlock,
            home=home,
            motion_profile=motion_profile,
        ),
        "motion_profile": motion_profile,
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
    operator_ack: bool = False,
    commissioning_override: bool = False,
) -> dict:
    allowed = {AxisName.X, AxisName.Y, AxisName.Z, AxisName.GRIPPER}
    normalized = [AxisName(axis) for axis in axes]
    invalid = [axis.value for axis in normalized if axis not in allowed]
    if invalid:
        raise HTTPException(status_code=400, detail=f"current set route is restricted to gantry/gripper x/y/z/g; invalid={invalid}")
    run = max(0, min(31, int(run_current)))
    requested_standby = max(0, min(31, int(standby_current)))
    if requested_standby > int(OEM_IDLE_STANDBY_CURRENT) and not (bool(operator_ack) and bool(commissioning_override)):
        raise HTTPException(
            status_code=409,
            detail=(
                f"standby_current above OEM idle {OEM_IDLE_STANDBY_CURRENT} requires "
                "operator_ack=true and commissioning_override=true; raw default full-current hold is forbidden"
            ),
        )
    standby = max(0, min(run, requested_standby))
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
            f"standby is capped at run_current and defaults to OEM idle {OEM_IDLE_STANDBY_CURRENT}; "
            "hot standby requires explicit commissioning override; no movement is commanded."
        ),
        "motion_commanded": False,
    }


def _prepare_motion_axis(
    tester: BioXpTester,
    axis: AxisName,
    *,
    speed: Optional[int] = None,
    acc: Optional[int] = None,
    reuse_prepared: bool = False,
    allow_desynced_reference: bool = False,
):
    if not bool(allow_desynced_reference):
        _require_axis_not_operator_desynced(axis, command="axis motion/prepare")
    preset = dict(_axis_preset(tester, axis))
    requested_profile = {
        "speed": None if speed is None else int(speed),
        "acc": None if acc is None else int(acc),
    }
    if speed is not None or acc is not None:
        norm = tester.motor_normalize_speed_acc(
            preset["board"],
            motor=preset["motor"],
            speed=preset["speed"] if speed is None else int(speed),
            acc=preset["acc"] if acc is None else int(acc),
        )
        preset["speed"] = int(norm["speed"])
        preset["acc"] = int(norm["acc"])
        preset["requested_speed"] = requested_profile["speed"]
        preset["requested_acc"] = requested_profile["acc"]
        preset["speed_acc_normalization"] = norm
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
                disable_right=preset.get("disable_right") if "disable_right" in preset else None,
                disable_left=preset.get("disable_left") if "disable_left" in preset else None,
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



def _domain_observation(projection: dict[str, Any], domain: str) -> Any:
    row = (projection.get("domains") or {}).get(domain)
    return row.get("observation") if isinstance(row, dict) and row.get("status") == "observed" else None


def _status_payload() -> dict:
    """Compatibility envelope projected from canonical state only."""
    projection = hardware_state.project("transport", "boards", "latch", "chiller")
    transport = _domain_observation(projection, "transport") or {}
    boards = _domain_observation(projection, "boards")
    latch = _domain_observation(projection, "latch") or {}
    chiller = _domain_observation(projection, "chiller")
    can_ready = transport.get("CAN_READY") if isinstance(transport, dict) else None
    if can_ready is None:
        can_ready = (projection.get("ownership") or {}).get("CAN_READY")
    ownership = projection.get("ownership") or {}
    runtime_available = bool(
        _tester is not None
        and _pipette_transport is not None
        and ownership.get("transport") == "owned"
        and ownership.get("usb") == "service"
        and ownership.get("router") == "running"
    )
    lifecycle = lifecycle_state.projection()
    return {
        **projection,
        "capabilities": list(BMS_COMMISSIONING_CAPABILITIES),
        "status": "ok" if can_ready is True and projection["cache_state"] == "fresh" else "degraded",
        "transport": "usb",
        "runtime_available": runtime_available,
        "hardware_connected": can_ready,
        "hardware_connected_deprecated": "maps only to OEM CAN_READY",
        "startup_error": _startup_error,
        "status_error": None if projection.get("available") else "canonical hardware snapshot unavailable",
        "board_status": boards,
        "chiller_status": chiller,
        "deck_io_snapshot": latch.get("snapshot") if isinstance(latch, dict) else None,
        "maintenance_state": _maintenance_state_payload(),
        "operation_state": lifecycle["operation_state"],
        "startup": lifecycle["startup"],
        "lifecycle": lifecycle,
    }


def _motion_power_status_payload(tester: BioXpTester | None = None) -> dict:
    """Pure compatibility projection; ``tester`` is ignored intentionally."""
    del tester
    projection = hardware_state.project("transport", "boards", "power", "interlock", "latch", "chiller")
    transport = _domain_observation(projection, "transport") or {}
    power = _domain_observation(projection, "power") or {}
    interlock = _domain_observation(projection, "interlock") or {}
    latch = _domain_observation(projection, "latch") or {}
    lifecycle = lifecycle_state.projection()
    return {
        **projection,
        "hardware_connected": transport.get("CAN_READY", (projection.get("ownership") or {}).get("CAN_READY")),
        "hardware_connected_deprecated": "maps only to OEM CAN_READY",
        "board_status": _domain_observation(projection, "boards"),
        "chiller_status": _domain_observation(projection, "chiller"),
        "deck_io_snapshot": latch.get("snapshot"),
        "rail_24v": power.get("rail_24v"),
        "motion_arm": interlock.get("motion_arm"),
        "latch_override": interlock.get("latch_override"),
        "operation_state": lifecycle["operation_state"],
        "startup": lifecycle["startup"],
    }


def _query_motor(tester: BioXpTester, board: int, command: int, cmd_type: int, motor: int = 0) -> dict[str, Any]:
    ack = tester.query_only_tmcl(int(board), int(command), int(cmd_type), int(motor), 0)
    return {"ack": ack, "value": None if ack is None else ack.get("value")}


def _query_axis_for_snapshot(tester: BioXpTester, axis: AxisName) -> dict[str, Any]:
    preset = _axis_preset(tester, axis)
    board, motor = int(preset["board"]), int(preset["motor"])
    params = {param: _query_motor(tester, board, 6, param, motor) for param in (1, 3, 6, 7, 9, 10, 12, 13)}
    left, right = params[9]["value"], params[10]["value"]
    left_disabled = None if params[13]["value"] is None else bool(int(params[13]["value"]) != 0)
    right_disabled = None if params[12]["value"] is None else bool(int(params[12]["value"]) != 0)
    active = int(tester.MOTOR_SWITCH_ACTIVE_VALUE)
    return {
        "axis": axis.value,
        "preset": preset,
        "status": {
            "board": board,
            "motor": motor,
            "position": {"position": params[1]["value"], **params[1]},
            "speed": {"speed": params[3]["value"], **params[3]},
            "max_current": {"param": 6, **params[6]},
            "standby_current": {"param": 7, **params[7]},
            "switches": {"left": params[9], "right": params[10], "right_disable": params[12], "left_disable": params[13]},
        },
        "switch_activity": {
            "left_raw_active": None if left is None else int(left) == active,
            "right_raw_active": None if right is None else int(right) == active,
            "left_disabled": left_disabled,
            "right_disabled": right_disabled,
        },
    }


def _query_io_snapshot(tester: BioXpTester) -> dict[int, Any]:
    return {channel: _query_motor(tester, tester.BOARD_DECK, 15, channel)["value"] for channel in (0, 1, 2, 3)}


def _query_aux_snapshot(tester: BioXpTester, kind: str) -> dict[str, Any]:
    if kind == "thermal":
        banks = (tester.THERMAL_BANK_NEST, tester.THERMAL_BANK_LID)
        # Named OEM ClassThermalControl/ClassThermalBoard read paths. Fan
        # speed GP21 exists only on the nest bank; there is no OEM GP22 read.
        gp_params_by_bank = {
            tester.THERMAL_BANK_NEST: (7, 8, 13, 21, 23),
            tester.THERMAL_BANK_LID: (7, 8, 13, 23),
        }
    else:
        banks = (tester.CHILLER_BANK_RC, tester.CHILLER_BANK_OC)
        # Named OEM ClassChillerBoard getters: target, heat/cool ramp, fan.
        gp_params_by_bank = {bank: (4, 7, 8, 21) for bank in banks}

    def query(command: int, cmd_type: int, motor: int) -> dict[str, Any]:
        ack = tester.query_only_tmcl(
            tester.BOARD_THERMAL if kind == "thermal" else tester.BOARD_CHILLER,
            command,
            cmd_type,
            motor,
            0,
        )
        return {"ack": ack, "value": None if ack is None else ack.get("value"), "ok": bool(ack is not None and int(ack.get("status", -1)) == 100)}

    firmware = query(173, 0, 0)
    if kind == "thermal":
        temperatures = {
            "tc_temp_c": {**query(10, 4, tester.THERMAL_BANK_NEST), "bank": tester.THERMAL_BANK_NEST},
            "lid_temp_c": {**query(10, 4, tester.THERMAL_BANK_LID), "bank": tester.THERMAL_BANK_LID},
            "ped_temp_c": {**query(143, 0, 2), "axis": 2},
        }
    else:
        temperatures = {label: {**query(143, 0, int(axis)), "axis": int(axis)} for label, axis in tester.CHILLER_TEMP_AXES}
    gp = {
        bank: [
            {"param": param, "bank": bank, **query(10, param, bank)}
            for param in gp_params_by_bank[bank]
        ]
        for bank in banks
    }
    return {"activation_attempted": False, "firmware": firmware, "temps": temperatures, "gp": gp, "alive": bool(firmware["ok"] or any(row["ok"] for row in temperatures.values()))}


def _hardware_collectors(tester: BioXpTester) -> dict[str, Any]:
    axes_cache: dict[str, Any] | None = None
    io_cache: dict[int, Any] | None = None

    def axes(_: CollectionContext) -> dict[str, Any]:
        nonlocal axes_cache
        if axes_cache is None:
            axes_cache = {axis.value: _query_axis_for_snapshot(tester, axis) for axis in AxisName}
        return {"axes": [axis.value for axis in AxisName], "rows": axes_cache}

    def io() -> dict[int, Any]:
        nonlocal io_cache
        if io_cache is None:
            io_cache = _query_io_snapshot(tester)
        return io_cache

    def transport(_: CollectionContext) -> dict[str, Any]:
        state = tester.query_only_transport_state()
        return {
            "runtime_available": True,
            "CAN_READY": _can_ready_observation(),
            "transport_internal_observation": state,
            "board_response_is_separate": True,
            "truth_source": "canonical transport ownership",
        }

    def boards(_: CollectionContext) -> dict[int, Any]:
        rows = {}
        for board in tester.BOARDS:
            if int(board) == int(tester.BOARD_THERMAL):
                rows[int(board)] = _query_aux_snapshot(tester, "thermal")["firmware"]
            elif int(board) == int(tester.BOARD_CHILLER):
                rows[int(board)] = _query_aux_snapshot(tester, "chiller")["firmware"]
            else:
                rows[int(board)] = _query_motor(tester, int(board), 173, 0)
        return rows

    def range_projection(context: CollectionContext) -> dict[str, Any]:
        return {"motion_config": harmonized_motion_config(), "axis_observations": axes(context)}

    def power(_: CollectionContext) -> dict[str, Any]:
        row = _query_motor(tester, tester.BOARD_DECK, 15, 0)
        ack = row.get("ack")
        reply_present = ack is not None
        oem_status = ack.get("status") if isinstance(ack, dict) else None
        payload_raw = row.get("value") if isinstance(ack, dict) and oem_status == 100 else None
        rail = lifecycle_state.voltage_observation(
            payload_raw=payload_raw,
            oem_status=oem_status,
            reply_present=reply_present,
            transport_outcome="reply" if reply_present else "no_reply",
            provenance={"route": "POST /hardware/snapshot/collect", "ack": ack},
        )
        rail.update({
            "oem_status": oem_status,
            "payload_raw": payload_raw,
            "ack": ack,
        })
        return {
            "rail_24v": rail,
            "safety_valid": rail["safety_valid"],
            "oem_power_ok": rail["zero_valid_sample"],
        }

    def interlock(_: CollectionContext) -> dict[str, Any]:
        return {"motion_arm": tester.motion_arm_state(), "latch_override": tester.motion_latch_override_state(), "deck_io": io()}

    def latch(_: CollectionContext) -> dict[str, Any]:
        snapshot = io()
        return {"snapshot": snapshot, "rail_24v": snapshot.get(0), "door_sensor": snapshot.get(1), "solenoid_state": snapshot.get(2), "latch_sensor": snapshot.get(3)}

    def gripper(context: CollectionContext) -> dict[str, Any]:
        return (axes(context).get("rows") or {}).get(AxisName.GRIPPER.value)

    def pipette(_: CollectionContext) -> dict[str, Any]:
        transport_owner = _pipette_transport
        if transport_owner is None or transport_owner.__class__.__name__ != "FourPipetteTransport":
            raise RuntimeError("four-pipette shared transport owner is unavailable")
        status = transport_owner.get_status()
        if not isinstance(status, dict) or status.get("live_query_performed") is not False:
            raise RuntimeError("pipette owner did not provide a cache-only four-channel projection")
        return status

    def shadow(context: CollectionContext) -> dict[str, Any]:
        return {"axes": axes(context), "interlocks": interlock(context), "reference_state": _reference_state_store.snapshot(list(AxisName)), "motion_commanded": False}

    def camera(_: CollectionContext) -> dict[str, Any]:
        _, session_projection = _camera_session_projection()
        probe_projection = _camera_cache_envelope(_camera_probe_cache)
        if not probe_projection.get("available") and not session_projection.get("available"):
            raise RuntimeError(
                "camera evidence unavailable; use explicit POST /camera/probe or "
                "POST /camera/stream/start"
            )
        return {
            "probe": probe_projection,
            "session": session_projection,
            "hardware_queried": False,
            "truth_source": "explicit camera cache",
        }

    return {
        "transport": transport,
        "boards": boards,
        "axes": axes,
        "range": range_projection,
        "power": power,
        "interlock": interlock,
        "latch": latch,
        "gripper": gripper,
        "thermal": lambda _: _query_aux_snapshot(tester, "thermal"),
        "chiller": lambda _: _query_aux_snapshot(tester, "chiller"),
        "pipette": pipette,
        "camera": camera,
        "shadow_readback": shadow,
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



def _json_safe(value: Any) -> Any:
    try:
        json.dumps(value)
        return value
    except TypeError:
        return str(value)


class UsbSniffManager:
    """Robot-local capture manager for Novo USB observability.

    This is capture-only diagnostics: it never homes, arms, recovers, or moves axes.
    It can run a driver TX/RX JSONL ledger immediately. Kernel usbmon pcap capture
    is enabled only when the service user can see a tcpdump usbmon interface.
    """

    def __init__(self) -> None:
        self.root = os.path.abspath(os.environ.get("BIOXP_USB_SNIFF_ROOT") or os.path.join(os.environ.get("BIOXP_LOG_ROOT") or "/tmp/bioxp-live-runs", "usb-sniff"))
        self._lock = threading.RLock()
        self._active: dict[str, Any] | None = None
        self._latest_run_id: str | None = None

    def _ensure_root(self) -> None:
        os.makedirs(self.root, exist_ok=True)

    def _safe_run_id(self, run_id: str) -> str:
        safe = str(run_id or "").strip()
        if not safe or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-" for ch in safe):
            raise HTTPException(status_code=400, detail="invalid usb-sniff run_id")
        return safe

    def _run_dir(self, run_id: str) -> str:
        return os.path.join(self.root, self._safe_run_id(run_id))

    def _read_json(self, path: str, fallback: Any = None) -> Any:
        try:
            with open(path, "r", encoding="utf-8") as handle:
                return json.load(handle)
        except Exception:
            return fallback

    def _write_json(self, path: str, payload: dict[str, Any]) -> None:
        tmp = f"{path}.tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        os.replace(tmp, path)

    def _append_jsonl(self, path: str, payload: dict[str, Any]) -> None:
        with open(path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, sort_keys=True, default=str) + "\n")

    def _tool(self, name: str) -> str | None:
        return shutil.which(name)

    def _command_output(self, args: list[str], timeout_s: float = 5.0) -> dict[str, Any]:
        try:
            cp = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout_s)
            return {"ok": cp.returncode == 0, "returncode": cp.returncode, "output": cp.stdout}
        except Exception as exc:
            return {"ok": False, "returncode": None, "output": str(exc), "error": str(exc)}

    def _usb_inventory(self) -> dict[str, Any]:
        return {
            "lsusb": self._command_output(["lsusb"], timeout_s=5.0),
            "lsusb_tree": self._command_output(["lsusb", "-t"], timeout_s=5.0),
            "target_vid_pid": "03eb:2423",
            "target_hint": "Novo USB-to-CAN Board; expected on USB bus 002 at full-speed 12M on this robot",
        }

    def _pcap_probe(self) -> dict[str, Any]:
        tcpdump = self._tool("tcpdump")
        probe: dict[str, Any] = {
            "available": False,
            "tool": tcpdump,
            "mode": None,
            "interface": None,
            "reason": None,
            "interfaces": [],
        }
        if tcpdump:
            result = self._command_output([tcpdump, "-D"], timeout_s=8.0)
            probe["tcpdump_D"] = {k: result.get(k) for k in ("ok", "returncode", "error") if k in result}
            interfaces: list[str] = []
            for raw_line in str(result.get("output") or "").splitlines():
                line = raw_line.strip()
                if not line:
                    continue
                # tcpdump -D format: "1.eth0 [...]" or "10.usbmon2 ..."
                name = line.split(" ", 1)[0].split(".", 1)[-1]
                interfaces.append(name)
            probe["interfaces"] = interfaces
            preferred = os.environ.get("BIOXP_USB_SNIFF_IFACE") or "usbmon2"
            if preferred in interfaces:
                probe.update({"available": True, "mode": "tcpdump_pcap", "interface": preferred, "reason": "tcpdump_usbmon_interface_available"})
                return probe
            for name in interfaces:
                if name.startswith("usbmon"):
                    probe.update({"available": True, "mode": "tcpdump_pcap", "interface": name, "reason": "tcpdump_usbmon_interface_available"})
                    return probe
        else:
            probe["tcpdump_D"] = {"ok": False, "returncode": None, "error": "tcpdump_not_installed_in_runtime"}

        # Fallback: the robot handler runs as root inside the udocker/proot runtime,
        # where tcpdump may be absent but debugfs usbmon text streams can be readable.
        # This is still packet-level IN/OUT evidence, just not pcapng.
        candidates = []
        env_path = os.environ.get("BIOXP_USBMON_TEXT_PATH")
        if env_path:
            candidates.append(env_path)
        bus_hint = os.environ.get("BIOXP_USBMON_BUS") or "2"
        candidates.extend([
            f"/sys/kernel/debug/usb/usbmon/{bus_hint}u",
            f"/sys/kernel/debug/usb/usbmon/{bus_hint}t",
            "/sys/kernel/debug/usb/usbmon/0u",
            "/sys/kernel/debug/usb/usbmon/0t",
        ])
        checked = []
        for path in candidates:
            if not path or path in checked:
                continue
            checked.append(path)
            try:
                if os.path.exists(path) and os.access(path, os.R_OK):
                    probe.update({"available": True, "mode": "usbmon_text", "interface": path, "reason": "debugfs_usbmon_text_available", "checked_paths": checked})
                    return probe
            except Exception:
                continue
        probe["checked_paths"] = checked
        probe["reason"] = "usbmon_not_visible_to_service_user" if tcpdump else "tcpdump_not_installed_and_usbmon_text_not_readable"
        return probe

    def _status_payload_locked(self) -> dict[str, Any]:
        active = self._active
        runs = self._list_runs_locked(limit=8)
        pcap = self._pcap_probe()
        return {
            "ok": True,
            "available": True,
            "active": active is not None,
            "current_run": self._public_run(active) if active else None,
            "latest_run": runs[0] if runs else None,
            "runs": runs,
            "capture_only": True,
            "safety_boundary": {
                "homes_axes": False,
                "arms_motion": False,
                "recovers_motion": False,
                "commands_axis_motion": False,
                "requires_operator_ack": USB_SNIFF_ACK,
            },
            "capabilities": {
                "driver_ledger": True,
                "pcap": bool(pcap.get("available")),
                "pcap_probe": pcap,
                "export_bundle": True,
                "tail": True,
            },
            "root": self.root,
            "target": {"vid_pid": "03eb:2423", "endpoints": {"in": "0x81", "out": "0x02"}, "speed": "full-speed 12M"},
            "warnings": [] if pcap.get("available") else ["kernel usbmon pcap is not visible to the bioxp-api service user; driver ledger can still run, but full packet capture needs usbmon/debugfs privilege"],
        }

    def status(self) -> dict[str, Any]:
        with self._lock:
            return self._status_payload_locked()

    def _public_run(self, run: dict[str, Any] | None) -> dict[str, Any] | None:
        if not run:
            return None
        return {k: v for k, v in run.items() if k not in {"process", "timer"}}

    def _run_summary_from_manifest(self, manifest: dict[str, Any]) -> dict[str, Any]:
        files = manifest.get("files") if isinstance(manifest.get("files"), dict) else {}
        return {
            "run_id": manifest.get("run_id"),
            "status": manifest.get("status"),
            "profile": manifest.get("profile"),
            "reason": manifest.get("reason"),
            "operator": manifest.get("operator"),
            "started_at": manifest.get("started_at"),
            "ended_at": manifest.get("ended_at"),
            "duration_s": manifest.get("duration_s"),
            "run_dir": manifest.get("run_dir"),
            "pcap_available": bool(((manifest.get("capabilities") or {}).get("pcap_probe") or {}).get("available")),
            "driver_ledger": bool(files.get("driver_ledger")),
            "pcap": bool(files.get("pcap")),
            "export": files.get("export"),
            "packet_accounting": manifest.get("packet_accounting"),
            "warnings": manifest.get("warnings") or [],
        }

    def _list_runs_locked(self, limit: int | None = None) -> list[dict[str, Any]]:
        self._ensure_root()
        rows = []
        for name in os.listdir(self.root):
            if name.startswith("."):
                continue
            mpath = os.path.join(self.root, name, "manifest.json")
            if not os.path.exists(mpath):
                continue
            manifest = self._read_json(mpath, {})
            if isinstance(manifest, dict):
                rows.append(self._run_summary_from_manifest(manifest))
        rows.sort(key=lambda row: str(row.get("started_at") or row.get("run_id") or ""), reverse=True)
        return rows[:limit] if limit else rows

    def runs(self) -> dict[str, Any]:
        with self._lock:
            return {"ok": True, "available": True, "active": self._active is not None, "runs": self._list_runs_locked(limit=50)}

    def _new_run_id(self) -> str:
        return f"{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}_{uuid.uuid4().hex[:8]}"

    def _start_pcap(self, run_dir: str, pcap_probe: dict[str, Any]) -> tuple[subprocess.Popen | None, str | None, dict[str, Any]]:
        if not pcap_probe.get("available"):
            return None, None, {"started": False, "reason": pcap_probe.get("reason") or "pcap_unavailable"}
        mode = str(pcap_probe.get("mode") or "tcpdump_pcap")
        iface = str(pcap_probe.get("interface") or "usbmon2")
        if mode == "usbmon_text":
            capture_path = os.path.join(run_dir, "usbmon_capture.txt")
            log_path = os.path.join(run_dir, "usbmon_text.log")
            cmd = ["cat", iface]
            out_handle = open(capture_path, "ab")
            log_handle = open(log_path, "ab")
            try:
                proc = subprocess.Popen(cmd, stdout=out_handle, stderr=log_handle)
            except Exception as exc:
                out_handle.close()
                log_handle.close()
                return None, None, {"started": False, "reason": "usbmon_text_start_failed", "error": str(exc), "command": cmd}
            return proc, capture_path, {"started": True, "pid": proc.pid, "mode": mode, "interface": iface, "command": cmd, "log": log_path, "format": "linux_usbmon_text"}
        tcpdump = str(pcap_probe.get("tool") or "tcpdump")
        pcap_path = os.path.join(run_dir, "usbmon_capture.pcap")
        log_path = os.path.join(run_dir, "tcpdump.log")
        cmd = [tcpdump, "-i", iface, "-w", pcap_path, "-U", "-s", "0"]
        log_handle = open(log_path, "ab")
        try:
            proc = subprocess.Popen(cmd, stdout=log_handle, stderr=subprocess.STDOUT)
        except Exception as exc:
            log_handle.close()
            return None, None, {"started": False, "reason": "tcpdump_start_failed", "error": str(exc), "command": cmd}
        return proc, pcap_path, {"started": True, "pid": proc.pid, "mode": mode, "interface": iface, "command": cmd, "log": log_path, "format": "pcap"}

    def start(self, req: "UsbSniffCaptureRequest") -> dict[str, Any]:
        payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
        if str(payload.get("operator_ack") or "") != USB_SNIFF_ACK:
            raise HTTPException(status_code=409, detail=f"operator_ack {USB_SNIFF_ACK} required for USB capture")
        reason = str(payload.get("reason") or "").strip()
        if not reason:
            raise HTTPException(status_code=409, detail="reason is required for USB packet capture")
        profile = str(payload.get("profile") or "passive")
        if profile not in USB_SNIFF_PROFILES:
            raise HTTPException(status_code=400, detail=f"profile must be one of {sorted(USB_SNIFF_PROFILES)}")
        duration_s = max(1, min(1800, int(payload.get("duration_s") or 300)))
        with self._lock:
            if self._active is not None:
                if bool(payload.get("stop_existing")):
                    self._stop_locked(reason="replaced by new USB capture", operator=str(payload.get("operator") or "bms-cockpit"))
                else:
                    raise HTTPException(status_code=409, detail={"error": "usb_sniff_already_active", "current_run": self._public_run(self._active)})
            self._ensure_root()
            run_id = self._new_run_id()
            run_dir = self._run_dir(run_id)
            os.makedirs(run_dir, exist_ok=False)
            ledger_path = os.path.join(run_dir, "driver_ledger.jsonl")
            event_path = os.path.join(run_dir, "events.jsonl")
            manifest_path = os.path.join(run_dir, "manifest.json")
            pcap_probe = self._pcap_probe()
            inventory = self._usb_inventory()
            pcap_proc = None
            pcap_path = None
            pcap_start = {"started": False, "reason": "not_requested"}
            if bool(payload.get("include_pcap", True)):
                pcap_proc, pcap_path, pcap_start = self._start_pcap(run_dir, pcap_probe)
            tester = None
            driver_ledger_enabled = False
            driver_ledger_result: dict[str, Any] = {"enabled": False, "reason": "not_requested"}
            if bool(payload.get("include_driver_ledger", True)):
                try:
                    tester = _get_tester()
                    enable = getattr(tester, "set_usb_sniff_ledger_path", None)
                    if callable(enable):
                        driver_ledger_result = enable(ledger_path, run_id=run_id)
                        driver_ledger_enabled = bool(driver_ledger_result.get("enabled"))
                    else:
                        driver_ledger_result = {"enabled": False, "error": "BioXpTester lacks set_usb_sniff_ledger_path"}
                except Exception as exc:
                    driver_ledger_result = {"enabled": False, "error": str(exc)}
            warnings = []
            if bool(payload.get("include_pcap", True)) and not bool(pcap_start.get("started")):
                warnings.append(f"pcap_not_active:{pcap_start.get('reason')}")
            if bool(payload.get("include_driver_ledger", True)) and not driver_ledger_enabled:
                warnings.append("driver_ledger_not_active")
            if bool(payload.get("include_pcap", True)) and bool(payload.get("include_driver_ledger", True)) and not bool(pcap_start.get("started")) and not driver_ledger_enabled:
                raise HTTPException(status_code=503, detail={"error": "no_usb_capture_channel_available", "pcap": pcap_start, "driver_ledger": driver_ledger_result})
            run = {
                "run_id": run_id,
                "status": "active",
                "profile": profile,
                "duration_s": duration_s,
                "passive_only": bool(payload.get("passive_only", True)),
                "reason": reason,
                "operator": str(payload.get("operator") or "bms-cockpit"),
                "started_at": _now_utc(),
                "run_dir": run_dir,
                "manifest_path": manifest_path,
                "ledger_path": ledger_path if driver_ledger_enabled else None,
                "event_path": event_path,
                "pcap_path": pcap_path,
                "process": pcap_proc,
                "pcap_start": pcap_start,
                "driver_ledger": driver_ledger_result,
                "warnings": warnings,
            }
            manifest = dict(run)
            manifest.pop("process", None)
            manifest["schema_version"] = "bioxp.usb_sniff_run.v1"
            manifest["capture_only"] = True
            manifest["safety_boundary"] = {"homes_axes": False, "arms_motion": False, "recovers_motion": False, "commands_axis_motion": False}
            manifest["capabilities"] = {"pcap_probe": pcap_probe, "driver_ledger": driver_ledger_result}
            manifest["usb_inventory"] = inventory
            manifest["files"] = {"manifest": manifest_path, "driver_ledger": ledger_path if driver_ledger_enabled else None, "pcap": pcap_path, "events": event_path}
            manifest["packet_accounting"] = {"driver_ledger_lines": 0, "pcap_size_bytes": 0, "unmatched_frames": None, "status": "pending"}
            self._write_json(manifest_path, manifest)
            self._append_jsonl(event_path, {"at": _now_utc(), "event": "start", "run_id": run_id, "pcap": pcap_start, "driver_ledger": driver_ledger_result})
            timer = threading.Timer(duration_s, lambda: self.stop(reason="duration elapsed", operator="auto_timer"))
            timer.daemon = True
            timer.start()
            run["timer"] = timer
            self._active = run
            self._latest_run_id = run_id
            return {"ok": True, "available": True, "active": True, "current_run": self._public_run(run), "status": self._status_payload_locked()}

    def _ledger_line_count(self, path: str | None) -> int:
        if not path or not os.path.exists(path):
            return 0
        try:
            with open(path, "rb") as handle:
                return sum(1 for _ in handle)
        except Exception:
            return 0

    def _file_sha256(self, path: str) -> str | None:
        if not path or not os.path.exists(path):
            return None
        h = hashlib.sha256()
        try:
            with open(path, "rb") as handle:
                for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                    h.update(chunk)
            return h.hexdigest()
        except Exception:
            return None

    def _finalize_manifest(self, run: dict[str, Any], *, status: str, reason: str, operator: str) -> dict[str, Any]:
        manifest_path = str(run.get("manifest_path"))
        manifest = self._read_json(manifest_path, {})
        if not isinstance(manifest, dict):
            manifest = {}
        manifest.update({
            "run_id": run.get("run_id"),
            "status": status,
            "ended_at": _now_utc(),
            "stop_reason": reason,
            "stop_operator": operator,
            "run_dir": run.get("run_dir"),
        })
        files = manifest.get("files") if isinstance(manifest.get("files"), dict) else {}
        for key in ("driver_ledger", "pcap", "events"):
            path = files.get(key)
            if path and os.path.exists(path):
                files[f"{key}_size_bytes"] = os.path.getsize(path)
                files[f"{key}_sha256"] = self._file_sha256(path)
        manifest["files"] = files
        manifest["packet_accounting"] = {
            "driver_ledger_lines": self._ledger_line_count(files.get("driver_ledger")),
            "pcap_size_bytes": os.path.getsize(files.get("pcap")) if files.get("pcap") and os.path.exists(files.get("pcap")) else 0,
            "unmatched_frames": None,
            "status": "pcap_and_driver_reconciliation_pending" if files.get("pcap") and files.get("driver_ledger") else "partial_capture_channel",
        }
        self._write_json(manifest_path, manifest)
        return manifest

    def _stop_locked(self, *, reason: str, operator: str) -> dict[str, Any]:
        run = self._active
        if run is None:
            return {"ok": True, "available": True, "active": False, "message": "no active USB capture", "latest_run": self._list_runs_locked(limit=1)[0] if self._list_runs_locked(limit=1) else None}
        timer = run.get("timer")
        if timer is not None:
            try:
                timer.cancel()
            except Exception:
                pass
        proc = run.get("process")
        pcap_stop: dict[str, Any] = {"stopped": False, "reason": "no_process"}
        if proc is not None:
            try:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=4)
                pcap_stop = {"stopped": True, "returncode": proc.returncode}
            except Exception as exc:
                pcap_stop = {"stopped": False, "error": str(exc)}
        try:
            tester = _get_tester()
            disable = getattr(tester, "set_usb_sniff_ledger_path", None)
            if callable(disable):
                disable(None, run_id=str(run.get("run_id") or ""))
        except Exception:
            pass
        event_path = run.get("event_path")
        if event_path:
            self._append_jsonl(str(event_path), {"at": _now_utc(), "event": "stop", "run_id": run.get("run_id"), "reason": reason, "operator": operator, "pcap_stop": pcap_stop})
        manifest = self._finalize_manifest(run, status="stopped", reason=reason, operator=operator)
        self._latest_run_id = str(run.get("run_id") or self._latest_run_id or "")
        self._active = None
        return {"ok": True, "available": True, "active": False, "stopped_run": self._run_summary_from_manifest(manifest), "pcap_stop": pcap_stop, "status": self._status_payload_locked()}

    def stop(self, *, reason: str = "operator stopped USB capture", operator: str = "bms-cockpit") -> dict[str, Any]:
        with self._lock:
            return self._stop_locked(reason=reason, operator=operator)

    def _resolve_run_id_locked(self, run_id: str | None = None) -> str:
        if run_id:
            return self._safe_run_id(run_id)
        if self._active is not None:
            return str(self._active.get("run_id"))
        if self._latest_run_id:
            return self._latest_run_id
        rows = self._list_runs_locked(limit=1)
        if rows:
            return str(rows[0].get("run_id"))
        raise HTTPException(status_code=404, detail="no USB capture run available")

    def tail(self, run_id: str, limit: int = 200) -> dict[str, Any]:
        limit = max(1, min(1000, int(limit or 200)))
        with self._lock:
            rid = self._safe_run_id(run_id)
            run_dir = self._run_dir(rid)
            ledger = os.path.join(run_dir, "driver_ledger.jsonl")
            events = os.path.join(run_dir, "events.jsonl")
            rows: list[str] = []
            for path in [ledger, events]:
                if not os.path.exists(path):
                    continue
                try:
                    with open(path, "r", encoding="utf-8", errors="replace") as handle:
                        rows.extend(handle.readlines()[-limit:])
                except Exception:
                    continue
            rows = rows[-limit:]
            parsed = []
            for line in rows:
                try:
                    parsed.append(json.loads(line))
                except Exception:
                    parsed.append({"raw": line.rstrip("\n")})
            return {"ok": True, "run_id": rid, "limit": limit, "lines": [line.rstrip("\n") for line in rows], "events": parsed}

    def files(self, run_id: str) -> dict[str, Any]:
        with self._lock:
            rid = self._safe_run_id(run_id)
            run_dir = self._run_dir(rid)
            if not os.path.isdir(run_dir):
                raise HTTPException(status_code=404, detail="USB capture run not found")
            files = []
            for name in sorted(os.listdir(run_dir)):
                path = os.path.join(run_dir, name)
                if not os.path.isfile(path):
                    continue
                files.append({"name": name, "path": path, "size_bytes": os.path.getsize(path), "sha256": self._file_sha256(path)})
            return {"ok": True, "run_id": rid, "run_dir": run_dir, "files": files}

    def export(self, run_id: str | None = None) -> dict[str, Any]:
        with self._lock:
            rid = self._resolve_run_id_locked(run_id)
            run_dir = self._run_dir(rid)
            if not os.path.isdir(run_dir):
                raise HTTPException(status_code=404, detail="USB capture run not found")
            export_path = os.path.join(run_dir, f"{rid}.tar.gz")
            with tarfile.open(export_path, "w:gz") as tar:
                for name in sorted(os.listdir(run_dir)):
                    path = os.path.join(run_dir, name)
                    if os.path.isfile(path) and path != export_path:
                        tar.add(path, arcname=os.path.join(rid, name))
            manifest_path = os.path.join(run_dir, "manifest.json")
            manifest = self._read_json(manifest_path, {})
            if isinstance(manifest, dict):
                files = manifest.get("files") if isinstance(manifest.get("files"), dict) else {}
                files["export"] = export_path
                files["export_size_bytes"] = os.path.getsize(export_path)
                files["export_sha256"] = self._file_sha256(export_path)
                manifest["files"] = files
                self._write_json(manifest_path, manifest)
            return {"ok": True, "run_id": rid, "export_path": export_path, "size_bytes": os.path.getsize(export_path), "sha256": self._file_sha256(export_path), "files": self.files(rid)["files"]}


_usb_sniff_manager = UsbSniffManager()


async def _run_blocking(label: str, func, timeout_s: float | None = 30.0):
    async with _tester_lock:
        worker = asyncio.create_task(run_in_threadpool(func))
        try:
            if timeout_s is None:
                return await asyncio.shield(worker)
            return await asyncio.wait_for(asyncio.shield(worker), timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            # A Python worker thread cannot be killed safely. Keep the hardware
            # exclusion lock until it has actually stopped before reporting the
            # elapsed API deadline; otherwise a second command can overlap it.
            try:
                await asyncio.shield(worker)
            except Exception:
                pass
            raise HTTPException(status_code=504, detail=f"{label} timed out after {timeout_s:.0f}s") from exc
        except asyncio.CancelledError:
            # Client disconnect/task cancellation must not release ownership
            # while the non-cancellable hardware worker is still running.
            try:
                await asyncio.shield(worker)
            finally:
                raise


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


def _camera_missing_dependency_payload(dependency: str, device: str | None = None, path: str | None = None, **extra) -> dict:
    return {
        "ok": False,
        "device": device,
        "path": path,
        "rc": None,
        "output": "",
        "error": f"missing required camera runtime dependency: {dependency}",
        "missing_dependency": dependency,
        "runtime_owner": os.getenv("BIOXP_RUNTIME_OWNER"),
        "failure_class": "camera_runtime_contract_failed",
        **extra,
    }


def _camera_ffmpeg_processes(device: str) -> list[dict]:
    rows: list[dict] = []
    proc_root = "/proc"
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return rows

    for name in entries:
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == os.getpid():
            continue
        cmdline_path = os.path.join(proc_root, name, "cmdline")
        try:
            with open(cmdline_path, "rb") as handle:
                raw = handle.read()
        except OSError:
            continue
        if not raw:
            continue
        args = [part.decode("utf-8", errors="replace") for part in raw.split(b"\0") if part]
        cmd = " ".join(args)
        executable = os.path.basename(args[0]) if args else ""
        if "ffmpeg" not in executable and " ffmpeg" not in f" {cmd}":
            continue
        if device not in args and device not in cmd:
            continue
        rows.append({"pid": pid, "cmd": cmd})
    return rows


def _camera_reset_local(preferred: str) -> dict:
    device = preferred if preferred and os.path.exists(preferred) else "/dev/video0"
    killed: list[int] = []
    errors: list[dict] = []

    for row in _camera_ffmpeg_processes(device):
        pid = int(row["pid"])
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

    survivors = _camera_ffmpeg_processes(device)

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
    if shutil.which("ffmpeg") is None:
        return _camera_missing_dependency_payload("ffmpeg", device=device, path=None, size=0, pick=pick)

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
    if shutil.which("ffmpeg") is None:
        return _camera_missing_dependency_payload("ffmpeg", device=device, seconds=dur, frames=0, fps_last=None, output_tail="", pick=pick)

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
        if shutil.which("ffmpeg") is None:
            detail = _camera_missing_dependency_payload(
                "ffmpeg",
                device=device,
                fps=fps,
                quality=quality,
                width=width,
                height=height,
            )
            _camera_stream_state.update({"active": False, "device": device, "last_error": detail["error"]})
            raise HTTPException(status_code=503, detail=detail)

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


def _camera_cache_envelope(cache: dict[str, Any] | None) -> dict[str, Any]:
    with _camera_projection_lock:
        epoch = _camera_projection_epoch
        row = None if cache is None else copy.deepcopy(cache)
    if row is None or row.get("camera_ownership_epoch") != epoch:
        return {"available": False, "cache_state": "missing", "camera_ownership_epoch": epoch, "freshness": {"state": "missing", "age_s": None}, "provenance": "POST /camera/probe"}
    age = max(0.0, time.time() - float(row.get("observed_unix", time.time())))
    state = "fresh" if age <= 30.0 else "stale"
    return {"available": True, "cache_state": state, "camera_ownership_epoch": epoch, "freshness": {"state": state, "age_s": round(age, 3), "fresh_for_s": 30.0}, "provenance": "POST /camera/probe", "probe": row}


def _camera_process_active() -> bool:
    with _camera_projection_lock:
        session = _camera_session
        process = None if session is None else session.get("process")
        return bool(process is not None and process.returncode is None)


async def _stop_owned_camera_session(*, reason: str) -> dict[str, Any]:
    global _camera_session, _camera_projection_epoch, _camera_probe_cache
    with _camera_projection_lock:
        session = _camera_session
        _camera_session = None
        _camera_projection_epoch += 1
        _camera_probe_cache = None
        epoch = _camera_projection_epoch
    hardware_state.invalidate(reason=f"camera ownership changed: {reason}")
    lifecycle_state.record_camera_evidence(None)
    if session is not None:
        proc = session.get("process")
        if proc is not None and proc.returncode is None:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), timeout=3.0)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
        task = session.get("reader_task")
        if task is not None and task is not asyncio.current_task() and not task.done():
            task.cancel()
    _camera_stream_state.update({"active": False, "last_error": reason, "last_frame_at": None})
    return {"ok": True, "stopped_session_id": None if session is None else session.get("session_id"), "replacement": reason == "replacement", "camera_ownership_epoch": epoch}


async def _start_owned_camera_session(payload: dict[str, Any]) -> dict[str, Any]:
    global _camera_session, _camera_projection_epoch, _camera_probe_cache
    replacement = _camera_session is not None
    if replacement:
        await _stop_owned_camera_session(reason="replacement")
    device = str(payload.get("device") or "/dev/video0")
    fps = max(1, min(int(payload.get("fps") or 8), 30))
    quality = max(2, min(int(payload.get("quality") or 7), 15))
    width = max(160, min(int(payload.get("width") or 640), 1920))
    height = max(120, min(int(payload.get("height") or 480), 1080))
    pick = _pick_stream_device(device)
    if not pick.get("ok"):
        raise HTTPException(status_code=503, detail=pick.get("error") or "No capture-capable camera device found")
    device = str(pick["device"])
    if shutil.which("ffmpeg") is None:
        raise HTTPException(status_code=503, detail=_camera_missing_dependency_payload("ffmpeg", device=device))
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-fflags", "nobuffer", "-flags", "low_delay", "-avioflags", "direct", "-f", "v4l2", "-input_format", "mjpeg", "-framerate", str(fps), "-video_size", f"{width}x{height}", "-i", device, "-an", "-vf", f"fps={fps}", "-q:v", str(quality), "-vcodec", "mjpeg", "-f", "image2pipe", "pipe:1"]
    proc = await asyncio.create_subprocess_exec(*cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
    if proc.stdout is None:
        proc.terminate()
        raise HTTPException(status_code=500, detail="ffmpeg stream stdout unavailable")
    queue: asyncio.Queue[bytes | None] = asyncio.Queue(maxsize=8)
    session_id = uuid.uuid4().hex
    with _camera_projection_lock:
        _camera_projection_epoch += 1
        _camera_probe_cache = None
        epoch = _camera_projection_epoch
        session = {"session_id": session_id, "camera_ownership_epoch": epoch, "device": device, "fps": fps, "quality": quality, "width": width, "height": height, "queue": queue, "process": proc, "reader_task": None, "started_at": time.time(), "frames_emitted": 0, "error": None}
        _camera_session = session
    hardware_state.invalidate(reason="camera ownership changed: stream started")
    _camera_stream_state.update({"active": True, "device": device, "fps": fps, "quality": quality, "width": width, "height": height, "frames_emitted": 0, "started_at": session["started_at"], "last_frame_at": None, "last_error": None, "session_id": session_id, "camera_ownership_epoch": epoch})

    async def reader() -> None:
        buffer = bytearray()
        try:
            while proc.returncode is None:
                chunk = await proc.stdout.read(16384)
                if not chunk:
                    break
                buffer.extend(chunk)
                while True:
                    start = buffer.find(b"\xff\xd8")
                    if start < 0:
                        if len(buffer) > 65536:
                            buffer.clear()
                        break
                    if start:
                        del buffer[:start]
                    end = buffer.find(b"\xff\xd9", 2)
                    if end < 0:
                        break
                    frame = bytes(buffer[: end + 2])
                    del buffer[: end + 2]
                    part = b"--frame\r\nContent-Type: image/jpeg\r\n" + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii") + frame + b"\r\n"
                    if queue.full():
                        try:
                            queue.get_nowait()
                        except asyncio.QueueEmpty:
                            pass
                    queue.put_nowait(part)
                    session["frames_emitted"] = int(session.get("frames_emitted") or 0) + 1
                    _camera_stream_state["frames_emitted"] = session["frames_emitted"]
                    _camera_stream_state["last_frame_at"] = time.time()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            session["error"] = str(exc)
            _camera_stream_state["last_error"] = str(exc)
        finally:
            _camera_stream_state["active"] = False
            try:
                queue.put_nowait(None)
            except asyncio.QueueFull:
                try:
                    queue.get_nowait()
                    queue.put_nowait(None)
                except asyncio.QueueEmpty:
                    pass

    task = asyncio.create_task(reader(), name=f"bioxp-camera-session-{session_id}")
    session["reader_task"] = task
    return {"ok": True, "session_id": session_id, "camera_ownership_epoch": epoch, "replacement": replacement, "device": device, "fps": fps, "quality": quality, "width": width, "height": height, "queue_max_frames": queue.maxsize, "mjpeg_url": "/camera/mjpeg"}


def _camera_session_projection() -> tuple[dict[str, Any] | None, dict[str, Any]]:
    with _camera_projection_lock:
        epoch = _camera_projection_epoch
        session = _camera_session
        if session is None or session.get("camera_ownership_epoch") != epoch:
            return None, {"available": False, "cache_state": "missing", "camera_ownership_epoch": epoch, "freshness": {"state": "missing", "age_s": None}, "provenance": "POST /camera/stream/start"}
        projection = {key: value for key, value in session.items() if key not in {"queue", "process", "reader_task"}}
    age = max(0.0, time.time() - float(projection["started_at"]))
    active = bool(_camera_stream_state.get("active"))
    state = "fresh" if active else "stale"
    return session, {"available": active, "cache_state": state, "camera_ownership_epoch": epoch, "freshness": {"state": state, "age_s": round(age, 3)}, "provenance": "POST /camera/stream/start", "session": projection}


@app.get("/status")
async def get_status():
    return _status_payload()


@app.post("/hardware/snapshot/collect")
async def hardware_snapshot_collect(payload: dict[str, Any] | None = None):
    """Explicit serialized query-only collection; never recovers or activates."""
    tester = _get_tester()
    requested = (payload or {}).get("domains") or list(DEFAULT_HARDWARE_SNAPSHOT_DOMAINS)
    if not isinstance(requested, list) or not all(isinstance(item, str) for item in requested):
        raise HTTPException(status_code=400, detail="domains must be a list of canonical domain names")
    try:
        return await _run_blocking(
            "Canonical hardware snapshot collection",
            lambda: hardware_state.collect(requested, _hardware_collectors(tester)),
            timeout_s=max(30.0, 15.0 * float(len(requested))),
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/reconnect")
async def reconnect_runtime():
    global _pipette_transport
    tester = _get_tester()
    close_fn = getattr(_pipette_transport, "close", None)
    if callable(close_fn):
        close_fn()
    await _run_blocking("BioXP reconnect", tester.reconnect, timeout_s=20.0)
    _pipette_transport = build_default_pipette_transport(shared_usb=tester)
    _ownership_changed(reason="runtime_reconnect", transport="owned", usb="service", router="running")
    maintenance = _mark_post_maintenance_motion_block(
        source="runtime_reconnect",
        reason="USB runtime reconnect completed; non-homing motion recovery is required before live motion.",
        usb_owner="service",
    )
    return {
        "ok": True,
        "message": "USB runtime reconnect requested; motion is blocked until recovery.",
        "maintenance_state": maintenance,
        **(await _run_blocking("BioXP status", _status_payload, timeout_s=20.0)),
    }


@app.get("/maintenance/usb/state")
async def maintenance_usb_state():
    return {"ok": True, "maintenance_state": _maintenance_state_payload()}


@app.post("/maintenance/usb/release")
async def maintenance_usb_release(request: Request):
    _require_local_maintenance_client(request)
    global _tester, _startup_error, _pipette_transport
    async with _tester_lock:
        tester = _tester
        if tester is not None:
            close_fn = getattr(_pipette_transport, "close", None)
            if callable(close_fn):
                close_fn()
            disconnect = getattr(tester, "_disconnect", None)
            if callable(disconnect):
                await asyncio.wait_for(run_in_threadpool(disconnect), timeout=20.0)
        _tester = None
        _pipette_transport = None
        _ownership_changed(reason="maintenance_usb_release", transport="unbound", usb="released", router="stopped")
        _startup_error = "BioXP USB runtime manually released for direct maintenance testing."
        maintenance = _mark_post_maintenance_motion_block(
            source="maintenance_usb_release",
            reason="USB runtime was manually released for direct maintenance testing; service motion is blocked until recovery.",
            usb_owner="released",
        )
    return {"ok": True, "mode": "maintenance", "usb_owner": "released", "message": _startup_error, "maintenance_state": maintenance}


@app.post("/maintenance/usb/reconnect")
async def maintenance_usb_reconnect(request: Request):
    _require_local_maintenance_client(request)
    global _tester, _startup_error, _pipette_transport
    async with _tester_lock:
        try:
            alt = int(os.environ.get("BIOXP_USB_ALT", "1"))
            _tester = await asyncio.wait_for(run_in_threadpool(lambda: BioXpTester(alt=alt)), timeout=20.0)
            _pipette_transport = build_default_pipette_transport(shared_usb=_tester)
            _startup_error = None
            _ownership_changed(reason="maintenance_usb_reconnect", transport="owned", usb="service", router="running")
        except Exception as exc:
            _tester = None
            _startup_error = str(exc)
            raise HTTPException(status_code=503, detail=_startup_error) from exc
        maintenance = _mark_post_maintenance_motion_block(
            source="maintenance_usb_reconnect",
            reason="USB runtime was reconnected after maintenance; non-homing motion recovery is required before live motion.",
            usb_owner="service",
        )
    return {"ok": True, "mode": "maintenance", "usb_owner": "service", "message": "USB runtime reconnected; motion is blocked until recovery.", "maintenance_state": maintenance}


async def _oem_runtime_activate_service_transaction(abort_requested: threading.Event) -> dict[str, Any]:
    global _tester, _startup_error, _pipette_transport, _quarantined_tester
    async with _tester_lock:
        if _quarantined_tester is not None:
            raise HTTPException(
                status_code=503,
                detail=_startup_error or "BioXP USB runtime is quarantined after cleanup failure",
            )
        if (_tester is None) != (_pipette_transport is None):
            raise HTTPException(status_code=503, detail="BioXP runtime ownership is internally inconsistent")
        already_active = _tester is not None and _pipette_transport is not None
        if _tester is None:
            candidate = None
            try:
                alt = int(os.environ.get("BIOXP_USB_ALT", "1"))
                candidate = await run_in_threadpool(lambda: BioXpTester(alt=alt))
                if abort_requested.is_set():
                    disconnect = getattr(candidate, "_disconnect", None)
                    if callable(disconnect):
                        await run_in_threadpool(disconnect)
                    raise asyncio.CancelledError
                candidate_transport = build_default_pipette_transport(shared_usb=candidate)
                _tester = candidate
                _pipette_transport = candidate_transport
                _startup_error = None
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                candidate = candidate or getattr(exc, "partial_owner", None)
                cleanup_error = None
                disconnect = getattr(candidate, "_disconnect", None)
                if callable(disconnect):
                    try:
                        await run_in_threadpool(disconnect)
                    except Exception as cleanup_exc:
                        cleanup_error = str(cleanup_exc)
                cleanup_failed = bool(cleanup_error or getattr(candidate, "_construction_cleanup_failed", False))
                _tester = None
                _quarantined_tester = candidate if cleanup_failed else None
                _pipette_transport = None
                _startup_error = str(exc)
                if cleanup_error:
                    _startup_error = f"{_startup_error}; partial USB owner cleanup failed: {cleanup_error}"
                raise HTTPException(status_code=503, detail=_startup_error) from exc
        elif not already_active:
            raise HTTPException(status_code=503, detail="BioXP runtime ownership is internally inconsistent")
        if already_active:
            maintenance = _maintenance_state_payload()
        else:
            _ownership_changed(
                reason="oem_runtime_activate_service",
                transport="owned",
                usb="service",
                router="running",
            )
            maintenance = _mark_post_maintenance_motion_block(
                source="oem_runtime_activate_service",
                reason="Service USB ownership activated; motion remains blocked.",
                usb_owner="service",
            )
    return {
        "ok": True,
        "usb_owner": "service",
        "snapshot_collected": False,
        "motion_recovered": False,
        "motion_commanded": False,
        "already_active": already_active,
        "maintenance_state": maintenance,
    }


@app.post("/oem/runtime/activate_service")
async def oem_runtime_activate_service():
    """Claim service USB ownership without snapshot, recovery, homing, or motion."""
    abort_requested = threading.Event()
    transaction = asyncio.create_task(_oem_runtime_activate_service_transaction(abort_requested))
    try:
        return await asyncio.shield(transaction)
    except asyncio.CancelledError as cancellation:
        abort_requested.set()
        try:
            await asyncio.shield(transaction)
        except BaseException:
            pass
        raise cancellation


@app.post("/maintenance/usb/recover_motion")
async def maintenance_usb_recover_motion(req: MaintenanceRecoverMotionRequest, request: Request):
    _require_local_maintenance_client(request)
    if req.operator_ack != MAINTENANCE_RECOVERY_ACK:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "operator_ack_required",
                "expected_operator_ack": MAINTENANCE_RECOVERY_ACK,
                "hardware_motion_commanded": False,
                "maintenance_state": _maintenance_state_payload(),
            },
        )
    tester = _get_tester()
    recovery = await _run_blocking(
        "Post-maintenance motion recovery",
        lambda: tester.motion_arm_strict_startup(run_homing=False),
        timeout_s=90.0,
    )
    evidence: dict[str, Any] = {"strict_startup": recovery, "operator_note": req.operator_note}
    if bool(req.include_diag):
        try:
            evidence["motion_power_status"] = await _run_blocking(
                "Post-maintenance motion power status",
                lambda: _motion_power_status_payload(tester),
                timeout_s=25.0,
            )
        except Exception as exc:
            evidence["motion_power_status_error"] = str(exc)
    maintenance = _clear_post_maintenance_motion_block(source="maintenance_usb_recover_motion", evidence=evidence)
    return {
        "ok": True,
        "message": "Post-maintenance motion recovery completed; live motion routes are unblocked.",
        "recovery": recovery,
        "maintenance_state": maintenance,
    }


@app.get("/motion/axis/{axis}/status")
async def axis_status(axis: AxisName):
    projection = hardware_state.project("axes")
    observed = _domain_observation(projection, "axes") or {}
    row = (observed.get("rows") or {}).get(axis.value)
    return {**projection, "axis": axis.value, "axis_status": row, **(row or {})}


@app.get("/motion/axes/status")
async def axes_status(axes: str = Query("x,y,z", description="Comma-separated axis names, e.g. x,y,z")):
    requested_axes = _parse_axes_csv(axes)
    projection = hardware_state.project("axes")
    observed = _domain_observation(projection, "axes") or {}
    all_rows = observed.get("rows") or {}
    return {**projection, "axes": [axis.value for axis in requested_axes], "rows": {axis.value: all_rows.get(axis.value) for axis in requested_axes}}


@app.get("/motion/range/status")
async def motion_range_status(axes: str = Query("x,y,z,g", description="Comma-separated axis names to include")):
    requested_axes = _parse_axes_csv(axes)
    projection = hardware_state.project("range")
    observed = _domain_observation(projection, "range") or {}
    config = observed.get("motion_config") or {}
    live_rows = ((observed.get("axis_observations") or {}).get("rows") or {})
    configured = config.get("axis_limits", {}) if isinstance(config, dict) else {}
    rows = {}
    for axis in requested_axes:
        key = axis.value
        limit = dict(configured.get(key, {})) if isinstance(configured, dict) else {}
        live = live_rows.get(key)
        position = (((live or {}).get("status") or {}).get("position") or {}).get("position") if isinstance(live, dict) else None
        rows[key] = {"axis": key, "configured_limit": limit, "current_position_steps": position, "distance_to_min_steps": None if position is None or "min_steps" not in limit else int(position) - int(limit["min_steps"]), "distance_to_max_steps": None if position is None or "max_steps" not in limit else int(limit["max_steps"]) - int(position), "live_status": live}
    return {**projection, "ok": projection.get("available", False), "axes": [axis.value for axis in requested_axes], "rows": rows, "motion_config": config, "live_status_available": bool(live_rows), "live_status_error": None if live_rows else "canonical range observation unavailable"}


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
    return _motion_power_status_payload()


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
            operator_ack=req.operator_ack,
            commissioning_override=req.commissioning_override,
        ),
        timeout_s=25.0,
    )


@app.get("/motion/interlock/override")
async def motion_interlock_override_status():
    projection = hardware_state.project("interlock")
    observed = _domain_observation(projection, "interlock") or {}
    return {**projection, "state": observed.get("latch_override"), "gate": observed.get("deck_io"), "motion_arm": observed.get("motion_arm"), "warning": "commissioning override only; cached projection is not independent physical safety proof"}


@app.post("/motion/interlock/override")
async def motion_interlock_override_set(req: MotionInterlockOverrideRequest):
    if req.operator_ack != "INTERLOCK_OVERRIDE":
        raise HTTPException(
            status_code=409,
            detail="operator_ack must be exactly INTERLOCK_OVERRIDE to toggle latch/24V commissioning override",
        )
    tester = _get_tester()
    override_latch = bool(req.enabled) if req.override_latch_sensor is None else bool(req.override_latch_sensor)
    override_rail = bool(req.enabled) if req.override_rail_24v is None else bool(req.override_rail_24v)
    if not bool(req.enabled):
        override_latch = False
        override_rail = False
    reason = _motion_interlock_override_reason(req)
    return await _run_blocking(
        "Motion interlock override set",
        lambda: tester.motion_latch_override_set(
            enabled=bool(req.enabled),
            reason=reason,
            override_latch_sensor=override_latch,
            override_rail_24v=override_rail,
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
    maintenance = _clear_post_maintenance_motion_block(
        source="motion_arm_strict_startup",
        evidence={"strict_startup": response, "run_homing": False},
    )
    if isinstance(response, dict):
        response = dict(response)
        response["maintenance_state"] = maintenance
    return response


@app.post("/motion/hard_reset")
async def motion_hard_reset(req: MotionHardResetRequest):
    tester = _get_tester()
    response = await _run_blocking(
        "Motion hard reset",
        lambda: tester.motor_hard_reset(rounds=req.rounds),
        timeout_s=max(45.0, 20.0 * float(req.rounds)),
    )
    _ownership_changed(reason="motion_hard_reset", transport="owned", usb="service", router="running")
    for axis in AxisName:
        _reference_state_store.mark_desynced(
            MarkAxisDesyncedCommand(
                axis=axis,
                reason="Motion hard reset executed; reference must be re-established.",
                source="motion_hard_reset",
                motion_kind="hard_reset",
            )
        )
    maintenance = _mark_post_maintenance_motion_block(
        source="motion_hard_reset",
        reason="Motion hard reset executed; non-homing strict startup/recovery is required before live axis motion.",
        usb_owner="service",
    )
    if isinstance(response, dict):
        response = dict(response)
        response["maintenance_state"] = maintenance
    return response


@app.post("/motion/clear_lock")
async def clear_lock():
    _require_motion_route_ready()
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
    projection = hardware_state.project("latch")
    observed = _domain_observation(projection, "latch") or {}
    return {**projection, **observed}


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
    _require_motion_not_blocked_by_maintenance()
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
        # Harmonize the stepwise startup route with the full initializeMotors Z stage.
        # OEM source says initializeMotors starts with MotorZ.axisSearchHome(1791), but on
        # this live stack the reconstructed startup semantics for that stage are the proven
        # Z reference contract: accept an already-established reference, otherwise return to
        # controller reference 0 with the Z-only helper and then verify.
        z_probe = tester.motor_oem_axis_already_home("z", tolerance_steps=2)
        z_reference = None
        z_verify = None
        z_ok = bool(isinstance(z_probe, dict) and z_probe.get("ok") is True)
        if not z_ok:
            z_reference = tester.motor_oem_move_z_to_reference(target_position=0, timeout_s=min(float(timeout_s), 90.0))
            if isinstance(z_reference, dict) and z_reference.get("ok") is True:
                z_verify = tester.motor_oem_axis_already_home("z", tolerance_steps=2)
                z_ok = bool(isinstance(z_verify, dict) and z_verify.get("ok") is True)
        result = {
            "ok": bool(z_ok),
            "home": z_verify if isinstance(z_verify, dict) else z_probe,
            "z_axisSearchHome_1791_probe": z_probe,
            "z_reference_return": z_reference,
            "z_reference_verify_after_return": z_verify,
            "z_reference_return_skipped": bool(z_ok and z_reference is None),
            "oem_z_stage_contract": "initializeMotors Z startup stage mirrored from full init live-reference recovery",
        }
    elif step == "gripper-clear":
        result = gripper_clear(
            tester,
            operator_ack="GRIPPER_CLEAR",
            reason="OEM startup_step gripper-clear",
            timeout_s=timeout_s,
        )
    elif step == "gripper-home":
        result = gripper_home(
            tester,
            operator_ack="GRIPPER_HOME",
            reason="OEM startup_step gripper-home",
            timeout_s=timeout_s,
        )
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


def _execute_oem_home_xy(tester: BioXpTester, *, timeout_s: float, allow_implementation_mapped_predicate: bool = False) -> dict:
    _require_motion_not_blocked_by_maintenance()
    predicate_snapshots = {}
    for axis in (AxisName.X, AxisName.Y):
        snapshot = _home_predicate_snapshot(tester, axis)
        predicate_snapshots[axis.value] = snapshot
        _require_home_predicate_guard(
            axis,
            snapshot,
            allow_implementation_mapped_predicate=bool(allow_implementation_mapped_predicate),
        )
    result = tester.motor_oem_home_xy(timeout_s=timeout_s)
    ok = bool(isinstance(result, dict) and result.get("ok") is True)
    return {
        "ok": ok,
        "source_mode": "HomeXY",
        "route_semantics": {
            "source_command": "HomeXY",
            "home_semantics": "direct_oem_homexy_mode_guarded_switch_search",
            "not_equivalent_to": ["/motion/axis/zero", "/motion/axis/home single-axis manual route"],
            "raw_fastapi_route": "/motion/oem/home_xy",
        },
        "predicate_snapshots_before": predicate_snapshots,
        "allow_implementation_mapped_predicate": bool(allow_implementation_mapped_predicate),
        "result": result,
    }


def _execute_oem_rehome(tester: BioXpTester, *, timeout_s: float) -> dict:
    _require_motion_not_blocked_by_maintenance()
    result = tester.motor_oem_rehome(timeout_s=timeout_s)
    return {
        "ok": bool(isinstance(result, dict) and result.get("ok") is True),
        "source_mode": "ControlLib.rehome",
        "route_semantics": {
            "source_command": "ControlLib.rehome",
            "home_semantics": "rehome_wrapper_over_initializeMotors",
            "not_equivalent_to": ["/motion/axis/zero", "/motion/axis/home"],
            "raw_fastapi_route": "/motion/oem/rehome",
        },
        "result": result,
    }


def _execute_oem_initialize_motion(
    tester: BioXpTester,
    *,
    run_homing: bool,
    timeout_s: float,
    include_tip_pipette_cleanup: bool = False,
) -> dict:
    _require_motion_not_blocked_by_maintenance()
    result = tester.motor_oem_initialize_motion(
        run_homing=bool(run_homing),
        timeout_s=timeout_s,
        include_tip_pipette_cleanup=bool(include_tip_pipette_cleanup),
    )
    return {
        "ok": bool(isinstance(result, dict) and result.get("ok") is True),
        "source_mode": "ControlLib.initializeMotion",
        "route_semantics": {
            "source_command": "ControlLib.initializeMotion",
            "home_semantics": "initializeMotion_wrapper_run_homing_true_delegates_to_initializeMotors",
            "not_equivalent_to": ["/motion/axis/zero", "/motion/axis/home"],
            "raw_fastapi_route": "/motion/oem/initialize_motion",
        },
        "result": result,
    }


@app.get("/motion/gripper/status")
async def motion_gripper_status():
    projection = hardware_state.project("gripper")
    observed = _domain_observation(projection, "gripper")
    return {**projection, "gripper": observed, **(observed or {})}


@app.post("/motion/gripper/restore_idle_current")
async def motion_gripper_restore_idle_current(req: GripperRestoreIdleRequest):
    tester = _get_tester()
    return await _run_blocking(
        "OEM gripper restore idle current",
        lambda: restore_gripper_idle_current(tester, reason=req.reason),
        timeout_s=20.0,
    )


@app.post("/motion/gripper/clear")
async def motion_gripper_clear(req: GripperActionRequest):
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM gripper clear",
        lambda: gripper_clear(tester, operator_ack=req.operator_ack, reason=req.reason, timeout_s=req.timeout_s),
        timeout_s=min(max(float(req.timeout_s) + 10.0, 20.0), 120.0),
    )


@app.post("/motion/gripper/home")
async def motion_gripper_home(req: GripperActionRequest):
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM gripper home",
        lambda: gripper_home(tester, operator_ack=req.operator_ack, reason=req.reason, timeout_s=req.timeout_s),
        timeout_s=min(max(float(req.timeout_s) + 10.0, 20.0), 120.0),
    )


def _thermal_door_success_or_409(result: dict) -> dict:
    if isinstance(result, dict) and result.get("ok") is True:
        return result
    raise HTTPException(status_code=409, detail=result)


@app.post("/motion/thermal_door/home")
async def motion_thermal_door_home(req: ThermalDoorActionRequest):
    if req.operator_ack != "HOME_THERMAL_DOOR":
        raise HTTPException(status_code=422, detail="operator_ack must be HOME_THERMAL_DOOR")
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM thermal door home",
        lambda: _thermal_door_success_or_409(
            tester.motor_oem_door_search_home(timeout_s=req.timeout_s, startup=False)
        ),
        timeout_s=max(25.0, float(req.timeout_s) + 10.0),
    )


@app.post("/motion/thermal_door/open")
async def motion_thermal_door_open(req: ThermalDoorActionRequest):
    if req.operator_ack != "OPEN_THERMAL_DOOR":
        raise HTTPException(status_code=422, detail="operator_ack must be OPEN_THERMAL_DOOR")
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM thermal door open",
        lambda: _thermal_door_success_or_409(
            tester.motor_oem_open_thermal_door(timeout_s=req.timeout_s)
        ),
        timeout_s=max(25.0, float(req.timeout_s) + 10.0),
    )


@app.post("/motion/thermal_door/close")
async def motion_thermal_door_close(req: ThermalDoorActionRequest):
    if req.operator_ack != "CLOSE_THERMAL_DOOR":
        raise HTTPException(status_code=422, detail="operator_ack must be CLOSE_THERMAL_DOOR")
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM thermal door close",
        lambda: _thermal_door_success_or_409(
            tester.motor_oem_close_thermal_door(timeout_s=req.timeout_s)
        ),
        timeout_s=max(25.0, float(req.timeout_s) + 10.0),
    )


@app.post("/motion/oem/startup_step")
async def motion_oem_startup_step(req: OemStartupStepRequest):
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        f"OEM startup homing step {req.step}",
        lambda: _execute_oem_startup_step(tester, req.step, req.timeout_s),
        timeout_s=min(max(float(req.timeout_s) + 10.0, 15.0), 120.0),
    )


@app.post("/motion/oem/home_xy")
async def motion_oem_home_xy(req: OemHomeXYRequest):
    if req.operator_ack != "HOMEXY":
        raise HTTPException(status_code=409, detail="operator_ack HOMEXY required for direct OEM HomeXY mode")
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM HomeXY direct mode",
        lambda: _execute_oem_home_xy(
            tester,
            timeout_s=req.timeout_s,
            allow_implementation_mapped_predicate=req.allow_implementation_mapped_predicate,
        ),
        timeout_s=min(max(float(req.timeout_s) + 10.0, 20.0), 150.0),
    )


@app.post("/motion/oem/rehome")
async def motion_oem_rehome(req: OemRehomeRequest):
    if req.operator_ack != "REHOME":
        raise HTTPException(status_code=409, detail="operator_ack REHOME required for direct OEM rehome wrapper")
    if not bool(req.run_homing):
        return {
            "ok": False,
            "failed_closed": True,
            "source_mode": "ControlLib.rehome",
            "physical_motion_commanded": False,
            "message": "Direct rehome wrapper is available, but run_homing=true is required to command monolithic live homing.",
            "route_semantics": {"raw_fastapi_route": "/motion/oem/rehome", "not_equivalent_to": ["/motion/axis/zero", "/motion/axis/home"]},
        }
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM ControlLib.rehome direct wrapper",
        lambda: _execute_oem_rehome(tester, timeout_s=req.timeout_s),
        timeout_s=min(max(float(req.timeout_s) + 10.0, 30.0), 260.0),
    )


@app.post("/motion/oem/initialization/run")
async def motion_oem_initialization_run(req: OemInitializationRunRequest):
    expected_ack = "OEM_INITIALIZATION_RUN_WITH_HOMING" if bool(req.run_homing) else "OEM_INITIALIZATION_RUN"
    if req.operator_ack != expected_ack:
        raise HTTPException(status_code=409, detail=f"operator_ack {expected_ack} required for OEM initialization controller run_homing={bool(req.run_homing)}")
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM initialization controller",
        lambda: run_oem_initialization_controller(
            tester,
            run_homing=req.run_homing,
            restore_door_state=req.restore_door_state,
            include_tip_pipette_cleanup=req.include_tip_pipette_cleanup,
            timeout_s=req.timeout_s,
        ),
        timeout_s=min(max(float(req.timeout_s) + 20.0, 45.0), 360.0),
    )


@app.post("/motion/oem/initialize_motion")
async def motion_oem_initialize_motion(req: OemInitializeMotionRequest):
    expected_ack = "INITIALIZE_WITH_HOMING" if bool(req.run_homing) else "INITIALIZE"
    if req.operator_ack != expected_ack:
        raise HTTPException(status_code=409, detail=f"operator_ack {expected_ack} required for OEM initializeMotion run_homing={bool(req.run_homing)}")
    _require_motion_route_ready()
    tester = _get_tester()
    return await _run_blocking(
        "OEM ControlLib.initializeMotion direct wrapper",
        lambda: _execute_oem_initialize_motion(
            tester,
            run_homing=req.run_homing,
            timeout_s=req.timeout_s,
            include_tip_pipette_cleanup=req.include_tip_pipette_cleanup,
        ),
        timeout_s=min(max(float(req.timeout_s) + 10.0, 30.0), 260.0),
    )


def _motion_response_allows_reference_update(response: dict) -> bool:
    if bool(response.get("dry_run")):
        return False
    if response.get("ok") is False:
        return False
    if isinstance(response.get("motion_failure"), dict):
        return False
    evidence = response.get("motion_evidence")
    classification = evidence.get("classification") if isinstance(evidence, dict) else None
    if isinstance(classification, dict) and classification.get("controller_motion_evidence") is True:
        return True
    truth = response.get("motion_truth")
    if isinstance(truth, dict) and truth.get("physical_motion_confirmed") is True:
        return True
    return False


@app.post("/motion/axis/relative")
async def move_axis_relative(req: MoveRelativeRequest):
    _require_motion_route_ready(req)
    response = await run_relative_motion_command(
        RelativeMoveCommand.from_request(req),
        get_tester=_get_tester,
        run_blocking=_run_blocking,
        execute_relative_move=_execute_relative_move,
        dry_run_response_factory=_dry_run_motion_response,
    )
    if _motion_response_allows_reference_update(response):
        _reference_state_store.record_motion(req.axis, "relative")
    return response


@app.post("/motion/axis/absolute")
async def move_axis_absolute(req: MoveAbsoluteRequest):
    _require_motion_route_ready(req)
    response = await run_absolute_motion_command(
        AbsoluteMoveCommand.from_request(req),
        get_tester=_get_tester,
        run_blocking=_run_blocking,
        execute_absolute_move=_execute_absolute_move,
        dry_run_response_factory=_dry_run_motion_response,
    )
    if _motion_response_allows_reference_update(response):
        _reference_state_store.record_motion(req.axis, "absolute")
    return response


@app.post("/motion/axis/zero")
async def move_axis_zero(req: MoveAxisZeroRequest):
    """Compatibility-safe operator command: return an axis to controller coordinate 0."""
    _require_motion_route_ready(req)
    response = await run_absolute_motion_command(
        AbsoluteMoveCommand(
            axis=req.axis,
            position_steps=0,
            wait_timeout_s=float(req.wait_timeout_s),
            speed=None if req.speed is None else int(req.speed),
            acc=None,
            artifact=AbsoluteMoveCommand.from_request(
                MoveAbsoluteRequest(
                    axis=req.axis,
                    position_steps=0,
                    wait_timeout_s=float(req.wait_timeout_s),
                    speed=req.speed,
                    capture_bundle=req.capture_bundle,
                    dry_run_bundle=req.dry_run_bundle,
                    operator_note=req.operator_note,
                    snapshot_refs=req.snapshot_refs,
                )
            ).artifact,
        ),
        get_tester=_get_tester,
        run_blocking=_run_blocking,
        execute_absolute_move=_execute_absolute_move,
        dry_run_response_factory=_dry_run_motion_response,
    )
    response.setdefault("route_semantics", {})
    response["route_semantics"].update(
        {
            "source_command": "move_axis_zero",
            "home_semantics": "not_homing_absolute_zero_only",
            "target_position_steps": 0,
            "oem_switch_search_homing_executed": False,
        }
    )
    if _motion_response_allows_reference_update(response):
        _reference_state_store.record_motion(req.axis, "absolute_zero")
    return response


@app.post("/motion/axis/home")
async def home_axis(req: HomeAxisRequest):
    _require_motion_route_ready(req)
    response = await run_home_axis_command(
        HomeAxisCommand.from_request(req),
        get_tester=_get_tester,
        run_blocking=_run_blocking,
        execute_home_axis=_execute_home_axis,
        dry_run_response_factory=_dry_run_motion_response,
    )
    response.setdefault("route_semantics", {})
    response["route_semantics"].update(
        {
            "source_command": "home_axis_manual_button_goHome_guarded",
            "home_semantics": "manual_goHome_style_switch_search_not_startup_axisSearchHome_not_zero",
            "oem_switch_search_homing_executed": not bool(response.get("dry_run")),
            "manual_home_safety": "fail_closed_until_predicate_matrix_live_or_source_verified; OEM proof is queryHome active after stop plus setHome success",
            "not_equivalent_to": ["/motion/axis/zero", "/motion/oem/startup_step", "/motion/oem/rehome", "/motion/oem/home_xy"],
        }
    )
    if _motion_response_allows_reference_update(response):
        _reference_state_store.mark_referenced(
            MarkAxisReferencedCommand(
                axis=req.axis,
                position_steps=0,
                source="home_axis",
                note="Axis homed via OEM switch-search API route.",
                motion_kind="home",
            )
        )
    elif not bool(response.get("dry_run")):
        _reference_state_store.mark_desynced(
            MarkAxisDesyncedCommand(
                axis=req.axis,
                reason="Home route failed or did not produce OEM motion evidence; axis reference blocked until re-established.",
                source="home_axis_failure_guard",
                motion_kind="home_failed_desync",
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


@app.get("/diagnostics/usb-sniff/status")
async def diagnostics_usb_sniff_status():
    return _usb_sniff_manager.status()


@app.get("/diagnostics/usb-sniff/runs")
async def diagnostics_usb_sniff_runs():
    return _usb_sniff_manager.runs()


@app.post("/diagnostics/usb-sniff/start")
async def diagnostics_usb_sniff_start(req: UsbSniffCaptureRequest):
    return _usb_sniff_manager.start(req)


@app.post("/diagnostics/usb-sniff/stop")
async def diagnostics_usb_sniff_stop(req: UsbSniffCaptureRequest | None = None):
    reason = "operator stopped USB packet capture"
    operator = "bms-cockpit"
    if req is not None:
        payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
        if str(payload.get("operator_ack") or "") != USB_SNIFF_ACK:
            raise HTTPException(status_code=409, detail=f"operator_ack {USB_SNIFF_ACK} required for USB capture stop")
        reason = str(payload.get("reason") or reason).strip() or reason
        operator = str(payload.get("operator") or operator)
    return _usb_sniff_manager.stop(reason=reason, operator=operator)


@app.post("/diagnostics/usb-sniff/export")
async def diagnostics_usb_sniff_export(req: UsbSniffCaptureRequest | None = None):
    run_id = None
    if req is not None:
        payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
        if payload.get("operator_ack") and str(payload.get("operator_ack")) != USB_SNIFF_ACK:
            raise HTTPException(status_code=409, detail=f"operator_ack {USB_SNIFF_ACK} required for USB capture export")
        run_id = payload.get("run_id")
    return _usb_sniff_manager.export(run_id=run_id)


@app.get("/diagnostics/usb-sniff/runs/{run_id}/tail")
async def diagnostics_usb_sniff_tail(run_id: str, limit: int = Query(200, ge=1, le=1000)):
    return _usb_sniff_manager.tail(run_id, limit=limit)


@app.get("/diagnostics/usb-sniff/runs/{run_id}/files")
async def diagnostics_usb_sniff_files(run_id: str):
    return _usb_sniff_manager.files(run_id)


@app.post("/thermal/baseline")
async def thermal_baseline():
    tester = _get_tester()
    return await _run_blocking("Thermal baseline", lambda: tester.thermal_apply_vendor_baseline(verify=True), timeout_s=30.0)


@app.get("/thermal/snapshot")
async def thermal_snapshot():
    projection = hardware_state.project("thermal")
    observed = _domain_observation(projection, "thermal")
    return {**projection, "thermal": observed, **(observed or {})}


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
    result = await _run_blocking("Thermal hard reset", tester.thermal_hard_reset, timeout_s=40.0)
    _ownership_changed(reason="thermal_hard_reset", transport="owned", usb="service", router="running")
    return result


@app.post("/chiller/baseline")
async def chiller_baseline():
    tester = _get_tester()
    return await _run_blocking("Chiller baseline", lambda: tester.chiller_apply_vendor_baseline(verify=True), timeout_s=30.0)


@app.get("/chiller/snapshot")
async def chiller_snapshot():
    projection = hardware_state.project("chiller")
    observed = _domain_observation(projection, "chiller")
    return {**projection, "chiller": observed, **(observed or {})}


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
    result = await _run_blocking("Chiller hard reset", tester.chiller_hard_reset, timeout_s=40.0)
    _ownership_changed(reason="chiller_hard_reset", transport="owned", usb="service", router="running")
    return result


@app.get("/camera/devices")
async def camera_devices():
    envelope = _camera_cache_envelope(_camera_probe_cache)
    probe = envelope.get("probe") or {}
    return {**envelope, "ok": envelope.get("available", False), "rows": probe.get("rows", []), "preferred_device": probe.get("preferred_device")}


@app.get("/camera/controls")
async def camera_controls(device: str = "/dev/video0"):
    envelope = _camera_cache_envelope(_camera_probe_cache)
    probe = envelope.get("probe") or {}
    controls = (probe.get("controls") or {}).get(device)
    return {**envelope, "ok": controls is not None, "device": device, "rows": [] if controls is None else controls.get("rows", []), "error": "camera controls not present in explicit probe cache" if controls is None else controls.get("error")}


@app.post("/camera/probe")
async def camera_probe(payload: dict[str, Any] | None = None):
    global _camera_probe_cache
    if _camera_process_active():
        raise HTTPException(status_code=409, detail="stop the owned camera stream before probing capabilities")
    tester = _get_tester()
    requested = (payload or {}).get("devices")

    def collect() -> dict[str, Any]:
        devices = _camera_devices_payload(tester)
        names = requested if isinstance(requested, list) else [row.get("device") for row in devices.get("rows", [])]
        controls = {}
        for name in names:
            if isinstance(name, str):
                controls[name] = tester.camera_enumerate_controls(device=name)
        return {**devices, "controls": controls}

    result = await _run_blocking("Explicit camera capability probe", collect, timeout_s=30.0)
    with _camera_projection_lock:
        _camera_probe_cache = {**result, "probe_id": uuid.uuid4().hex, "available": bool(result.get("ok", True)), "camera_ownership_epoch": _camera_projection_epoch, "observed_at": _now_utc(), "observed_unix": time.time(), "provenance": "POST /camera/probe"}
        lifecycle_state.record_camera_evidence(_camera_probe_cache)
        return {"ok": True, "published": True, "probe": dict(_camera_probe_cache)}


@app.post("/camera/stream/start")
async def camera_stream_start(payload: dict[str, Any] | None = None):
    result = await _start_owned_camera_session(payload or {})
    lifecycle_state.record_camera_evidence({**result, "available": bool(result.get("ok")), "provenance": "POST /camera/stream/start"})
    return result


@app.post("/camera/control")
async def camera_control(req: CameraControlRequest):
    global _camera_probe_cache
    tester = _get_tester()
    result = await _run_blocking(
        "Camera control",
        lambda: {
            **tester.v4l2_set_ctrl(req.cid, req.value, device=req.device),
            "stream_active": bool(_camera_stream_state.get("active")),
            "stream_state": _camera_stream_state_payload(),
        },
        timeout_s=15.0,
    )
    with _camera_projection_lock:
        _camera_probe_cache = None
    lifecycle_state.record_camera_evidence(None)
    hardware_state.invalidate(reason="camera controls changed")
    return result


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
    if _camera_process_active():
        return {
            "ok": False,
            "busy": True,
            "stream_active": True,
            "device": req.device,
            "error": "live stream is active; stop the stream before running auto recover",
            "stream_state": _camera_stream_state_payload(),
        }
    tester = _get_tester()
    ownership = await _stop_owned_camera_session(reason="auto recover")
    result = await _run_blocking(
        "Camera auto recover",
        lambda: tester.camera_auto_oneclick(device=req.device, max_resets=req.max_resets),
        timeout_s=75.0,
    )
    return {**result, "camera_ownership": ownership}


@app.post("/camera/reset")
async def camera_reset(req: CameraSnapshotRequest):
    stopped = await _stop_owned_camera_session(reason="explicit reset")
    reset = await run_in_threadpool(_camera_reset_local, req.device)
    return {**reset, "owned_session": stopped}


@app.post("/camera/stop")
async def camera_stop(req: CameraSnapshotRequest):
    del req
    return await _stop_owned_camera_session(reason="explicit stop")


@app.get("/camera/stream_state")
async def camera_stream_state():
    _, projection = _camera_session_projection()
    return {**projection, **_camera_stream_state_payload()}


@app.get("/camera/mjpeg")
async def camera_mjpeg(
    device: str = "/dev/video0",
    fps: int = Query(8, ge=1, le=30),
    quality: int = Query(7, ge=2, le=15),
    width: int = Query(640, ge=160, le=1920),
    height: int = Query(480, ge=120, le=1080),
):
    del device, fps, quality, width, height
    session, projection = _camera_session_projection()
    if session is None or not projection.get("available"):
        raise HTTPException(status_code=503, detail={**projection, "error": "no active POST-started camera session"})
    queue = session["queue"]

    async def iterator():
        while True:
            part = await queue.get()
            if part is None:
                break
            yield part

    return StreamingResponse(
        iterator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store", "X-BioXp-Camera-Session": str(session["session_id"]), "X-BioXp-Camera-Ownership-Epoch": str(session["camera_ownership_epoch"])},
    )


@app.get("/liquid/status")
async def liquid_status():
    projection = hardware_state.project("pipette")
    observed = _domain_observation(projection, "pipette")
    return {**projection, "pipette": observed, "channels": None if not isinstance(observed, dict) else observed.get("channels")}


@app.post("/liquid/init")
async def liquid_init(req: PipetteInitRequest):
    command = PipetteInitCommand.from_request(req)
    if command.prime_volume_ul is not None:
        raise HTTPException(status_code=409, detail="constructor initialization cannot prime or mutate liquid")
    if _can_ready_observation() is not True:
        raise HTTPException(status_code=409, detail="constructor pipette stage requires explicit CAN_READY=true evidence")

    def action() -> dict[str, Any]:
        result = _get_pipette_transport().initialize(command)
        return {**result, "channel_count": 4, "channels_constructed_unconditionally": [0, 1, 2, 3], "motion_commanded": False}

    try:
        projection = await _run_blocking(
            "OEM constructor four-pipette stage",
            lambda: lifecycle_state.run_stage("constructor_pipette_stage", action),
            timeout_s=260.0,
        )
    except LifecycleStateError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    if projection["startup"]["stages"]["constructor_pipette_stage"]["state"] != "passed":
        raise HTTPException(status_code=409, detail=projection)
    return {"ok": True, "lifecycle": projection}


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


def _optional_int_payload(params: dict[str, Any], *keys: str) -> int | None:
    for key in keys:
        value = params.get(key)
        if value is not None:
            return int(value)
    return None


def _protocol_live_move_handler(action, state):
    _require_motion_route_ready()
    params = dict(action.params or {})
    raw_axis = params.get("axis") or params.get("axis_name")
    if raw_axis is None:
        raise ValueError(f"Protocol move action '{action.action_id}' is missing params.axis")
    axis = AxisName(str(raw_axis).strip().lower())
    wait_timeout_s = float(params.get("wait_timeout_s") or params.get("timeout_s") or 30.0)
    speed = _optional_int_payload(params, "speed", "max_speed")
    acc = _optional_int_payload(params, "acc", "acceleration", "max_acc")
    tester = _get_tester()

    position_steps = _optional_int_payload(params, "position_steps", "target_position", "target_steps", "position")
    if position_steps is not None:
        result = _execute_absolute_move(
            tester,
            axis,
            int(position_steps),
            wait_timeout_s,
            speed=speed,
            acc=acc,
        )
        _reference_state_store.record_motion(axis, "protocol_absolute")
        return {"ok": True, "protocol_action_id": action.action_id, "move_mode": "absolute", "move": result}

    steps = _optional_int_payload(params, "steps", "delta_steps", "relative_steps")
    if steps is None:
        raise ValueError(
            f"Protocol move action '{action.action_id}' needs absolute position_steps/target_position or relative steps/delta_steps"
        )
    result = _execute_relative_move(
        tester,
        axis,
        int(steps),
        wait_timeout_s,
        speed=speed,
        acc=acc,
        reuse_prepared=bool(params.get("reuse_prepared", False)),
    )
    _reference_state_store.record_motion(axis, "protocol_relative")
    return {"ok": True, "protocol_action_id": action.action_id, "move_mode": "relative", "move": result}


def _protocol_live_handlers() -> dict[ProtocolActionKind, Any]:
    return {
        ProtocolActionKind.MOVE: _protocol_live_move_handler,
    }


@app.post("/protocol/execute")
async def protocol_execute(req: ProtocolExecuteRequest):
    try:
        return await run_in_threadpool(
            create_protocol_job,
            req.model_dump(exclude_none=True),
            dry_run=bool(req.dry_run),
            handlers=None if req.dry_run else _protocol_live_handlers(),
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
