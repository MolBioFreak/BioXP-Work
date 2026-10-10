"""Original ADP application instructions inside the existing finite native owner.

399156 V1.0 pp2–5: P/D/A0/V, ordered air/liquid phases and L(n1,n2).
30053815-C pp35–37: original ADP v/c speed setters and their interactions.
Manufacturer manual mirror: https://www.docin.com/p-4632231525.html
399155 V1.0 p3: p0..p8. 399094 V1.0 pp2–3: BR + robotic Z.
No firmware identification, Water resolution or pickup geometry is
inferred here. Unknown executable fields are representation errors, not gates
on existing OEM operations. Requested raw JSON is never normalized in-place.
"""
from __future__ import annotations

from copy import deepcopy
from decimal import Decimal, InvalidOperation
import time
from typing import Annotated, Any, Literal, Mapping, Union

from pydantic import AfterValidator, BaseModel, ConfigDict, Field, TypeAdapter, ValidationError, field_validator

from .models import PipetteCommandError
from .cavro_geometry import (SurfaceMove, SharedHead, ProfilePickup, resolve_surface_move,
                             resolve_displacement, shared_head_evidence)
from .cavro_commands import Number, DeviceCommand, DeviceProgram, compile_device_command, compile_device_program
from .pressure_capture import PressureCaptureRequest
from .cavro_geometry import RackTransform, resolve_rack_transform, mounted_trajectory
from .effect_accounting import AccountingContext, StrokeIntent

IMPLEMENTATION = "original-adp.application.v1"


def _nonnegative_number(value):
    if Decimal(str(value)) < 0:
        raise ValueError("expected a nonnegative number")
    return value


def _positive_number(value):
    if Decimal(str(value)) <= 0:
        raise ValueError("expected a positive number")
    return value


# Decimal text preserves native analysis output through the existing wire
# encoder. Existing numeric requests retain their values and range semantics.
NonnegativeNumber = Annotated[Number, AfterValidator(_nonnegative_number)]
PositiveNumber = Annotated[Number, AfterValidator(_positive_number)]


class _Input(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class _Selected(_Input):
    channels: list[Annotated[int, Field(ge=0, le=3)]] = Field(min_length=1, max_length=4)
    timeout_ms: int = Field(gt=0)

    @field_validator("channels")
    @classmethod
    def unique(cls, value):
        if len(set(value)) != len(value):
            raise ValueError("channels must be unique")
        return value


class Stroke(_Selected):
    operation: Literal["aspirate", "dispense", "leading_air", "trailing_air", "reaspirate"]
    volume_ul: NonnegativeNumber
    speed_ul_s: PositiveNumber
    target_liquid_ul: NonnegativeNumber | None = None
    accounting_intent: StrokeIntent | None = None


class Empty(_Selected):
    operation: Literal["empty"]
    speed_ul_s: PositiveNumber


class Settings(_Selected):
    operation: Literal["settings"]
    # Exact settings only: no null -> default/Water rewrite.
    values: dict[str, Any]


class Delay(_Input):
    operation: Literal["delay"]
    duration_ms: int = Field(ge=0)


class Position(_Input):
    operation: Literal["position"]
    positioning: dict[str, Any]
    rack_transform: RackTransform | None = None


class ZMove(_Input):
    operation: Literal["z_move"]
    target_steps: int
    speed_native: int = Field(gt=0)


class Plld(_Selected):
    operation: Literal["plld"]
    start_mode: Literal[0, 1] = 0
    detection_id: str = Field(default="last_detection", min_length=1)
    start_steps: int
    search_target_steps: int
    search_speed_native: int = Field(gt=0)
    z_motor_current: int
    # Omitted means no extra motion, not an inferred submerge or retract.
    after_detection_steps: int | None = None
    after_detection_speed_native: int | None = Field(default=None, gt=0)

    @field_validator("start_mode", mode="before")
    @classmethod
    def integer_mode(cls, value):
        if type(value) is not int:
            raise ValueError("start_mode must be integer 0 or 1")
        return value


class TrackedAspirate(_Selected):
    operation: Literal["tracked_aspirate"]
    volume_ul: NonnegativeNumber
    speed_ul_s: PositiveNumber
    target_liquid_ul: NonnegativeNumber | None = None
    accounting_intent: StrokeIntent | None = None
    trajectory: SurfaceMove
    z_motor_current: int
    z_timeout_ms: int = Field(gt=0)


class DeviceCommandInstruction(_Selected):
    operation: Literal["device_command"]
    command: DeviceCommand


class DeviceProgramInstruction(_Selected):
    operation: Literal["device_program"]
    program: DeviceProgram


class PressureCapture(_Input):
    operation: Literal["pressure_capture"]
    capture: PressureCaptureRequest


Instruction = Annotated[Union[Stroke, Empty, Settings, Delay, Position, ZMove, Plld,
                              SurfaceMove, TrackedAspirate, DeviceCommandInstruction, DeviceProgramInstruction,
                              ProfilePickup, PressureCapture],
                        Field(discriminator="operation")]
_INSTRUCTION = TypeAdapter(Instruction)


class ApplicationRequest(_Input):
    implementation: Literal["original-adp.application.v1"]
    operations: list[Instruction]
    # Resolved class ledger is inert metadata, never a hidden executable input.
    liquid_settings: dict[str, Any] = Field(default_factory=dict)
    event_policy: Literal["stop", "pause_for_operator"] = "stop"
    shared_head: SharedHead | None = None
    accounting_context: AccountingContext | None = None


# 399155 p3: numeric selector, domain, published default, unit. These defaults
# are documentary: no operation is emitted for an absent setting.
PRESSURE_PARAMETERS = {
    "plld_threshold_adc": (0, 1, 1000, 15, "ADC counts"),
    "plld_persistence_ms": (1, 1, 100, 5, "ms"),
    "plld_start_position_increments": (2, 0, 44000, 0, "increments"),
    "plld_speed_increments_s": (3, 100, 1000, 150, "increments/s"),
    "plld_travel_increments": (4, 500, 44000, 44000, "increments"),
    "plld_direction": (5, 0, 1, 0, "0 aspirate; 1 dispense"),
    "pressure_interval_ms": (6, 1, 20, 20, "ms"),
    "pressure_samples_per_can": (7, 1, 3, 3, "samples"),
    "hybrid_interval_ms": (8, 1, 100, 50, "ms"),
}


def _number(value: Any) -> str:
    """Plain decimal wire spelling; never scientific notation or silent roundoff."""
    if isinstance(value, bool):
        raise ValueError("expected finite number, not Boolean")
    try:
        number = Decimal(str(value))
    except InvalidOperation:
        raise ValueError("expected finite number") from None
    if not number.is_finite():
        raise ValueError("expected finite number")
    encoded = format(number, "f")
    return encoded.rstrip("0").rstrip(".") if "." in encoded else encoded


def setting_commands(values: Mapping[str, Any]) -> list[tuple[str, str]]:
    commands = []
    for field, value in values.items():
        if field in PRESSURE_PARAMETERS:
            selector, low, high, _, _ = PRESSURE_PARAMETERS[field]
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"{field}: expected integer {low}..{high} (399155 p3)")
            wire = f"p{selector},{value}R"
        elif field == "slope":
            if not isinstance(value, (list, tuple)) or len(value) != 2 or any(type(v) is not int for v in value):
                raise ValueError("slope: expected both integer components [n1,n2]")
            wire = f"L{value[0]},{value[1]}R"
        elif field == "pressure_streaming":
            if type(value) is not bool:
                raise ValueError("pressure_streaming: expected Boolean")
            # b automatically persists heartbeat periodicity (30053815-C p53).
            # Stream selection is only o; heartbeat is a separate explicit
            # device command, never an implicit persistent setting.
            wire = "o0,1R" if value else "o0,0R"
        elif field in {"start_speed_ul_s", "cutoff_speed_ul_s"}:
            # Original ADP Operating Manual 30053815-C pp35–37, not ADP Detect:
            # lowercase v/c, explicit microliters/second selector 1, execute R.
            # Keep conversion to increments and temporary top-speed limiting
            # device-owned. Never infer defaults, cap cutoff at the advisory
            # 100 uL/s recommendation, insert delays, or save to NVRAM.
            encoded = _number(value)
            number = Decimal(encoded)
            maximum = 100 if field == "start_speed_ul_s" else 200
            if not Decimal("2.5") <= number <= maximum:
                raise ValueError(f"{field}: expected 2.500..{maximum}.000 uL/s (30053815-C p36)")
            if len(encoded.partition(".")[2]) > 3:
                raise ValueError(f"{field}: at most three decimal places (30053815-C p36); no rounding")
            wire = f"{'v' if field == 'start_speed_ul_s' else 'c'}{encoded},1R"
        elif field == "backlash_increments":
            # 30053815-C p38: K[n], 0..500 plunger increments added at the end
            # of each aspiration to take up direction-change slack.
            if type(value) is not int or not 0 <= value <= 500:
                raise ValueError("backlash_increments: expected integer 0..500 (30053815-C p38)")
            wire = f"K{value}R"
        else:
            raise ValueError(f"{field}: unknown executable setting")
        commands.append((field, wire))
    return commands


def _json_copy(value):
    """Thaw native frozen JSON without changing scalar/null/omission semantics."""
    if isinstance(value, Mapping):
        return {key: _json_copy(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_copy(item) for item in value]
    return deepcopy(value)


def _wire_advisories(operations):
    """Document certain original-manual domains without changing legacy IO."""
    warnings = []
    for index, operation in enumerate(operations):
        for command in operation.get("wire_commands", []):
            wire = command["ascii"]
            if wire.startswith("V"):
                number = Decimal(wire[1:-3])
                if not Decimal("0.025") <= number <= 1000 or len(_number(number).partition(".")[2]) > 3:
                    warnings.append({"code": "documented_top_speed_domain", "operation_index": index,
                        "ascii": wire, "source": "30053815-C p35", "execution_unchanged": True,
                        "message": "Published V microliter range .025..1000.000, at most three decimals; device behavior unmeasured."})
            elif wire.startswith("L"):
                if any(not 1 <= int(v) <= 40 for v in wire[1:-1].split(",")):
                    warnings.append({"code": "documented_slope_domain", "operation_index": index,
                        "ascii": wire, "source": "30053815-C p37", "execution_unchanged": True,
                        "message": "Published L components 1..40; device behavior unmeasured."})
    return warnings


def compile_application(request: Mapping[str, Any]) -> dict[str, Any]:
    raw = _json_copy(request)
    issues = []
    try:
        parsed = ApplicationRequest.model_validate(raw)
    except ValidationError as exc:
        for error in exc.errors(include_url=False, include_context=False):
            issues.append({"code": "invalid_application", "category": "representation",
                           "path": "/" + "/".join(map(str, error["loc"])), "message": error["msg"]})
        return {"implementation": IMPLEMENTATION, "requested": raw, "operations": None, "issues": issues}
    emitted = []
    for index, instruction in enumerate(parsed.operations):
        op = instruction.model_dump(exclude_unset=True)
        try:
            if isinstance(instruction, Settings):
                op["wire_commands"] = []
                for name, value in instruction.values.items():
                    try:
                        op["wire_commands"].extend({"field": f, "ascii": w}
                            for f, w in setting_commands({name: value}))
                    except ValueError as exc:
                        pointer = name.replace("~", "~0").replace("/", "~1")
                        issues.append({"code": "unrepresentable_setting", "category": "representation",
                            "path": f"/operations/{index}/values/{pointer}", "message": str(exc)})
            elif isinstance(instruction, (Stroke, TrackedAspirate)):
                if isinstance(instruction, TrackedAspirate):
                    resolve_displacement(op["trajectory"]["displacement"])
                prefix = "D" if instruction.operation == "dispense" else "P"
                op["wire_commands"] = [{"field": "speed_ul_s", "ascii": f"V{_number(instruction.speed_ul_s)},1R"},
                                       {"field": "volume_ul", "ascii": f"{prefix}{_number(instruction.volume_ul)},1R"}]
            elif isinstance(instruction, Empty):
                op["wire_commands"] = [{"field": "speed_ul_s", "ascii": f"V{_number(instruction.speed_ul_s)},1R"},
                                       {"field": "empty", "ascii": "A0R"}]
            elif isinstance(instruction, Position):
                from ..manual_pipetting import manual_position_plan
                op["native_plan"] = manual_position_plan(instruction.positioning)
                if instruction.rack_transform is not None:
                    op["resolved_rack"] = resolve_rack_transform(op["rack_transform"], raw.get("shared_head"))
                    if op["resolved_rack"]["geometry_used"]:
                        from ..oem_deck_movement import compile_finite_plate_operation
                        xyz = op["resolved_rack"]["target_steps"]
                        op["native_plan"] = compile_finite_plate_operation("pipette_move_xy", source_leaf_available=True, x=xyz["x"], y=xyz["y"])
            elif isinstance(instruction, Plld):
                if (instruction.after_detection_steps is None) != (instruction.after_detection_speed_native is None):
                    raise ValueError("after_detection_steps and after_detection_speed_native must be specified together")
                op["wire_commands"] = [{"field": "plld", "ascii": "B1R" if instruction.start_mode == 1 else "BR"}]
            elif isinstance(instruction, SurfaceMove):
                resolve_displacement(op["displacement"])
                if instruction.reference.kind in {"authored", "vessel", "rack"}:
                    op["resolved_geometry"] = resolve_surface_move(op, {})
            elif isinstance(instruction, DeviceCommandInstruction):
                op["compiled_command"] = compile_device_command(op["command"])
            elif isinstance(instruction, DeviceProgramInstruction):
                op["compiled_command"] = compile_device_program(op["program"])
            elif isinstance(instruction, ProfilePickup):
                from ..manual_pipetting import manual_position_plan
                op["native_plan"] = manual_position_plan(op["calibration"]["positioning"])
                if op["calibration"].get("rack_transform") is not None:
                    op["resolved_rack"] = resolve_rack_transform(op["calibration"]["rack_transform"], raw.get("shared_head"))
                    if op["resolved_rack"]["geometry_used"]:
                        from ..oem_deck_movement import compile_finite_plate_operation
                        xyz = op["resolved_rack"]["target_steps"]
                        op["native_plan"] = compile_finite_plate_operation("pipette_move_xy", source_leaf_available=True, x=xyz["x"], y=xyz["y"])
                for phase in ("pickup", "retract"):
                    if op["calibration"][phase] is not None:
                        resolve_displacement(op["calibration"][phase]["displacement"])
            emitted.append(op)
        except (ValueError, TypeError) as exc:
            issues.append({"code": "unrepresentable_application", "category": "representation",
                           "path": f"/operations/{index}", "message": str(exc)})
    return {"implementation": IMPLEMENTATION, "requested": raw,
            "operations": None if issues else emitted, "issues": issues,
            "warnings": _wire_advisories(emitted),
            "liquid_settings": deepcopy(parsed.liquid_settings),
            "physical_effect_verified": False}


def capability_catalog() -> dict[str, Any]:
    return {"implementation": IMPLEMENTATION, "controller_family": "original Cavro ADP",
            "installed_firmware": None, "installed_options": None,
            "schema": ApplicationRequest.model_json_schema(),
            "pressure_parameters": {name: {"selector": spec[0], "minimum": spec[1],
                "maximum": spec[2], "published_default": spec[3], "unit": spec[4],
                "source": "399155 V1.0 p3", "automatically_applied": False}
                for name, spec in PRESSURE_PARAMETERS.items()},
            "slope": {"wire": "Ln1,n2R", "source": "399156 V1.0 p5",
                      "phase": "explicit ordered settings instruction"},
            "start_cutoff_speed": {
                "status": "implemented", "source": "30053815-C pp35–37,45",
                "source_url": "https://www.docin.com/p-4632231525.html",
                "unit": "uL/s", "decimal_places": 3, "automatically_applied": False,
                "start_speed_ul_s": {"wire": "vn,1R", "minimum": 2.5, "maximum": 100},
                "cutoff_speed_ul_s": {"wire": "cn,1R", "minimum": 2.5, "maximum": 200},
                "phase": "explicit ordered settings; applies to subsequent plunger moves until changed",
                "controller_conversion": "configured maximum volume; increments/s rounded to nearest integer",
                "controller_interaction": "actual start/cutoff temporarily limited by top speed; readback reports programmed value",
                "persistence": "working memory only; no NVRAM save emitted",
                "advisory": "30053815-C p36 discourages cutoff >100 uL/s (lost steps/overload), especially adjacent moves without >10 ms delay; 399156 p9 uses 200 uL/s. No cap or delay is inserted."},
            "backlash_increments": {"wire": "KnR", "minimum": 0, "maximum": 500, "published_default": 0,
                "unit": "plunger increments", "source": "30053815-C p38", "automatically_applied": False},
            "existing_native_families": {
                "diagnostic_pipette": ["aspirate", "dispense", "dispense_all", "diagnoses", "initialize", "get_data", "last_error", "eject", "plunger_up", "plunger_down"],
                "pipette_manual_physical": ["load_tip", "source_load_tips", "measure_fluid_height", "source_fluid_offset", "source_calwith_fluid", "source_mix", "source_purge"]},
            "raw_pressure_streaming": {"status": "implemented",
                "enable_wire": "o0,1R", "implicit_heartbeat_nvram_write": False,
                "samples_owner": "existing NovoRouter pressure observation/receipt path",
                "capture_operation": "pressure_capture",
                "capture_effects": "listen only; stream selection and sample settings remain separately authored",
                "readback": "/pipettes/pressure-captures/{stream_session_id}"},
            "surface_coordination": {"operations": ["surface_move", "tracked_aspirate"],
                "axis_units": ["steps", "mm with authored calibration"],
                "independent_channel_z": False, "automatic_geometry": False,
                "tracking": "Authored constant-speed shared-Z trajectory starts after selected pump sends and before their completion waits; not continuous level feedback."},
            "geometry_context": {"status": "implemented_explicit_opt_in",
                "surface_references": ["authored", "detected", "vessel", "rack"],
                "cross_section_models": ["constant", "linear_area"],
                "volume_height_units": "uL = mm^3; exact Decimal integration/inversion",
                "rack_transform": "Signed measured 3D basis and affine native-step rows; explicit reference channel; XY and Z selected separately",
                "missing_or_nonintegral_geometry": "Nonblocking observation; retain explicit authored fallback",
                "mounted_tips": "All supplied mounted tips, including inactive and mixed lengths; predicted common-head footprints, not collision or fit proof"},
            "completed_effect_accounting": {"status": "implemented_nonblocking_observations",
                "context": "Optional per-channel explicit pneumatic/liquid-intent initial compartments; null means unknown, empty list means authored empty",
                "inventories": ["commanded_pneumatic", "authored_liquid_intent"],
                "evidence": ["interfaces", "peak_occupancy", "cumulative_travel", "phase_roles", "vessel_accessibility", "completed_prefix"],
                "physical_delivery_verified": False, "capacity_enforced": False,
                "correction_apportioned_to_liquid": False},
            "tip_profiles": {name: {"design_intent": True, "physical_fit_verified": False,
                "pickup_mapping": "source T50/T200 or explicit existing manual tray/well pickup; no inferred rack geometry",
                "profile_context": "shared_head.mounted_tips; metric surface_move requires explicit tip_offset_mm"}
                for name in ("T10", "T50", "T200", "T1000")}}


def _send_wire(group, channels, wire, timeout_ms, *, after_sends=None, effect="cavro_settings", inputs=None):
    """Use the live collection's transport lock, owner tokens and epoch waits."""
    if group._forceabort():
        raise PipetteCommandError("Stopped by user or force abort")
    def send(channel, transport, defer):
        transport._require_initialized()
        driver = transport._get_driver()
        command_name = {"fluid detection": "start_fluid_detection", "aspirate air": "aspirate_air",
                        "cavro_reaspirate": "aspirate_air"}.get(effect, effect)
        raw = driver._send_pipette_command(wire, command_name=command_name, wait_for_completion=False)
        # The existing multipart encoder returns its owner in provenance but
        # does not copy it to the driver's single-frame owner slot. Capture the
        # actual returned token, never reuse a preceding speed command's owner.
        token = (raw.get("provenance") or {}).get("completion_owner_token")
        if isinstance(token, str) and token:
            driver._pipette_completion_owner_token = token
        # Reuse the existing send-failure semantics. The collection attaches
        # earlier channels to exceptions; keep the failing raw response too.
        transport._assert_driver_result(effect, raw)
        values = dict(inputs or {})
        context = values.get("accounting_context")
        if context is not None:
            values["accounting_initial"] = next((v for v in context.get("initial_channels", []) if v["channel"] == channel), None)
            values["accounting_vessels"] = context.get("vessels", [])
        intent = values.get("accounting_intent") or {}
        values["accounting_vessel_id"] = intent.get("channel_vessels", {}).get(str(channel))
        return {"ok": raw.get("ok") is True, "driver_result": raw,
                **transport._driver_evidence(raw), **values}
    try:
        outcome = group._run_group_liquid_operation(effect, channels, send,
            timeout_ms=timeout_ms, after_sends=after_sends,
            set_allow_to_stop=effect != "fluid detection")
    except Exception as exc:
        partial = getattr(exc, "pipette_partial_result", None)
        if partial is not None:
            raise PipetteCommandError(str(exc), details=partial) from exc
        raise
    if effect == "set_top_speed":
        for row in outcome["channels"]:
            if row.get("completion", {}).get("ok") is True and not outcome["interrupted_by_terminate"]:
                group._transports[row["channel"]]._top_speed = float(wire[1:-3])
    return outcome


def run_application_inline(provider, request: Mapping[str, Any], *, command_id: str,
                           owner_identity: Mapping[str, Any]) -> dict[str, Any]:
    plan = compile_application(request)
    request = plan["requested"]
    result = {"kind": "cavro_application", "implementation": IMPLEMENTATION,
              "requested": plan["requested"], "liquid_settings": plan.get("liquid_settings", {}),
              "warnings": plan.get("warnings", []),
              "events": [], "ok": False, "completed": False,
              "physical_effect_verified": False, "delivery_attempted": False}
    if plan["issues"]:
        return {**result, "issues": plan["issues"]}
    group = provider.primitives.pipette_transport
    epoch = group._interrupt_epoch
    events = result["events"]
    detections = {}
    last_rack = None
    head_targets = {}
    result["shared_head"] = shared_head_evidence(request.get("shared_head"), [])

    def fence(identity):
        provider._wp8_execution_fence_checker(command_id, boundary=identity)
        if epoch != group._interrupt_epoch or group._forceabort():
            raise PipetteCommandError("Cavro application interrupted; no replay")

    def record(name, call, *, inputs, pipette=False):
        identity = {**owner_identity, "source_identity":
                    f"{owner_identity['source_identity']}:cavro:{len(events)}:{name}"}
        fence(identity["source_identity"])
        event = {"operation": name, "source_identity": identity["source_identity"],
                 "origin": "native_device" if pipette else "robot_orchestration",
                 "inputs": deepcopy(inputs), "status": "in_flight", "reported_applied": None}
        events.append(event)
        try:
            result["delivery_attempted"] = True
            body = (provider._manual_pipette_receipt_runner(name, call, command_id, identity, request)
                    if pipette else call())
            # Source-return success is distinct from missing adapter evidence.
            # Do not turn an untyped return into either a failed event followed
            # by success or a new execution gate. Explicit False still fails.
            outcome = body.get("ok")
            event.update(result=body, status=("completed" if outcome is True else
                         "failed" if outcome is False else "unknown"))
            # Completion is not a settings readback or independent fluid proof.
            event["reported_applied"] = {"controller_outcome": body.get("ok"),
                "completion_verified": body.get("completion_verified"), "readback": None}
            if body.get("ok") is False:
                raise PipetteCommandError(f"Cavro application child failed: {name}", details=body)
            fence(identity["source_identity"])
            return body
        except Exception as exc:
            event.update(status="failed", error=str(exc), partial=getattr(exc, "details",
                getattr(exc, "detail", getattr(exc, "pipette_partial_result", None))))
            raise

    def z_move(target, speed):
        record("z_speed", lambda: provider.primitives.z_set_max_speed(speed), inputs={"speed_native": speed})
        body = record("z_move", lambda: provider.moveZ(target), inputs={"target_steps": target})
        head_targets["z"] = target
        return body

    def footprints(geometry, move, channels=()):
        if request.get("shared_head") is not None:
            transform = move["reference"].get("transform") if move["reference"]["kind"] == "rack" else last_rack
            geometry.update(mounted_trajectory(request["shared_head"],
                {**head_targets, "z": geometry["target_steps"]}, transform, channels))
        return geometry

    try:
        for index, op in enumerate(plan["operations"]):
            result["operation_index"] = index
            kind = op["operation"]
            if kind in {"device_command", "device_program"}:
                record(kind, lambda t: t.run_cavro_command(op["compiled_command"],
                    channels=op["channels"], timeout_ms=op["timeout_ms"]), inputs=op, pipette=True)
            elif kind == "pressure_capture":
                record("pressure_capture", lambda t: t.capture_pressure(op["capture"]),
                       inputs=op, pipette=True)
            elif kind == "delay":
                deadline = time.monotonic() + op["duration_ms"] / 1000
                while True:
                    fence(f"{owner_identity['source_identity']}:delay:{index}")
                    left = deadline - time.monotonic()
                    if left <= 0:
                        break
                    time.sleep(min(left, .05))
                events.append({"operation": "delay", "origin": "robot_orchestration",
                               "inputs": op, "status": "completed", "result": {"ok": True}})
            elif kind == "position":
                if op.get("resolved_rack") is not None:
                    record("rack_geometry", lambda: {"ok": True, **op["resolved_rack"]}, inputs=op)
                record("position", lambda: provider._wp8_execute_nested_plan(
                    plan=op["native_plan"], command_id=command_id,
                    owner_identity={**owner_identity, "source_identity":
                        f"{owner_identity['source_identity']}:cavro:position:{index}"}), inputs=op["positioning"])
                if (op.get("resolved_rack") or {}).get("geometry_used"):
                    last_rack = op["rack_transform"]
                    head_targets.update({a: op["resolved_rack"]["target_steps"][a] for a in ("x", "y")})
            elif kind == "profile_pickup":
                if op.get("resolved_rack") is not None:
                    record("rack_geometry", lambda: {"ok": True, **op["resolved_rack"]}, inputs=op)
                # Explicit general-purpose pickup. Retain existing station/well
                # routing and finite ownership; do not assign OEM T50/T200 type,
                # inventory or tip-location from a requested profile.
                record("profile_pickup_position", lambda: provider._wp8_execute_nested_plan(
                    plan=op["native_plan"], command_id=command_id, owner_identity={**owner_identity,
                        "source_identity": f"{owner_identity['source_identity']}:cavro:pickup:{index}"}),
                    inputs=op["calibration"])
                if (op.get("resolved_rack") or {}).get("geometry_used"):
                    last_rack = op["calibration"]["rack_transform"]
                    head_targets.update({a: op["resolved_rack"]["target_steps"][a] for a in ("x", "y")})
                geometry = resolve_surface_move(op["calibration"]["pickup"], detections)
                footprints(geometry, op["calibration"]["pickup"])
                record("profile_pickup_geometry", lambda: {"ok": True, **geometry,
                    "profile": op["profile"], "calibration": op["calibration"],
                    "physical_fit_verified": False}, inputs=op)
                z_move(geometry["target_steps"], geometry["speed_native"])
                record("query_tip_status_all", lambda t: t.query_tip_status_all(), inputs=op, pipette=True)
                if op["calibration"]["retract"] is not None:
                    geometry = resolve_surface_move(op["calibration"]["retract"], detections)
                    z_move(geometry["target_steps"], geometry["speed_native"])
            elif kind == "z_move":
                z_move(op["target_steps"], op["speed_native"])
            elif kind == "surface_move":
                geometry = resolve_surface_move(op, detections)
                footprints(geometry, op)
                record("surface_reference", lambda: {"ok": True, **geometry}, inputs=geometry)
                z_move(geometry["target_steps"], geometry["speed_native"])
            elif kind == "plld":
                z_move(op["start_steps"], op["search_speed_native"])
                search = {"z_search_attempted": False, "z_stop_attempted": False,
                          "final_position_steps": None, "z_settled": None}
                result["plld"] = search
                def start_z():
                    # Existing source pseudo-home/current handling; no synthetic pose.
                    state = provider._offset_deck_semantic_state(gripper_confirmed=False, pseudo_home_only=True)
                    def dispatch():
                        search["z_search_attempted"] = True
                        return provider.primitives.oem_move_z(op["search_target_steps"],
                            pseudo_home_steps=state["pseudo_z_home"],
                            motor_current=op["z_motor_current"], wait_for_stop=False)
                    search["start_result"] = record("plld_z_start", dispatch, inputs=op)
                def detect(t):
                    body = _send_wire(t, op["channels"], op["wire_commands"][0]["ascii"], op["timeout_ms"],
                                      after_sends=start_z, effect="fluid detection")
                    if body.get("ok") is False and not body.get("interrupted_by_terminate"):
                        # ControlLib.detectFluidLevel:7901: false wait terminates
                        # pumps, not Z. No finally lift/home/Stop is in source.
                        # Existing Stop/owner fences remain authoritative.
                        try:
                            fence(f"{owner_identity['source_identity']}:plld:terminate")
                            body["source_timeout_termination"] = t.terminate()
                        except Exception as exc:
                            body["source_timeout_termination"] = {"error": str(exc),
                                "exception_type": type(exc).__name__}
                    return body
                detected = record("detect_fluid_level", detect, inputs=op, pipette=True)
                def stop_z():
                    search["z_stop_attempted"] = True
                    return provider.primitives.z_stop()
                search["stop_result"] = record("plld_z_stop", stop_z, inputs={})
                position = record("plld_z_position", lambda: {
                    "ok": True, "position_steps": provider.primitives._read_axis_position("z"),
                    "source_return_completed": True}, inputs={})
                timestamps = {}
                for row in detected.get("channels", []):
                    stamp = (row.get("completion", {}).get("pipette_message_state") or {}).get("fluid_timestamp")
                    timestamps[row["channel"]] = stamp
                    group._fluid_detection_timestamps[row["channel"]] = stamp
                search.update(channels=detected.get("channels", []), z=position,
                              final_position_steps=position["position_steps"], fluid_timestamps=timestamps)
                search["shared_head"] = shared_head_evidence(request.get("shared_head"), op["channels"])
                search["start_mode"] = op.get("start_mode", 0)
                detections[op.get("detection_id", "last_detection")] = deepcopy(search)
                if op.get("after_detection_steps") is not None:
                    z_move(op["after_detection_steps"], op["after_detection_speed_native"])
            else:
                tracking: dict[str, Any] = {}
                if kind == "tracked_aspirate":
                    tracking = resolve_surface_move(op["trajectory"], detections)
                    footprints(tracking, op["trajectory"], op["channels"])
                    tracking["shared_head"] = shared_head_evidence(request.get("shared_head"), op["channels"])
                    record("tracking_reference", lambda: {"ok": True, **tracking}, inputs=op)
                    record("z_speed", lambda: provider.primitives.z_set_max_speed(tracking["speed_native"]),
                           inputs={"speed_native": tracking["speed_native"]})
                def start_tracking():
                    state = provider._offset_deck_semantic_state(gripper_confirmed=False, pseudo_home_only=True)
                    return record("tracking_z_start", lambda: provider.primitives.oem_move_z(
                        tracking["target_steps"], pseudo_home_steps=state["pseudo_z_home"],
                        motor_current=op["z_motor_current"], wait_for_stop=False), inputs=tracking)
                for command in op["wire_commands"]:
                    field, wire = command["field"], command["ascii"]
                    effect = ("set_top_speed" if field == "speed_ul_s" else
                              "aspirate" if kind in {"aspirate", "tracked_aspirate"} else
                              "cavro_reaspirate" if kind == "reaspirate" else
                              "aspirate air" if kind in {"leading_air", "trailing_air"} else
                              "dispense" if kind == "dispense" else
                              "dispense_all" if kind == "empty" else "cavro_settings")
                    inputs = {"volume_ul": op.get("volume_ul"), "front_air": kind == "leading_air",
                              "target_liquid_ul": op.get("target_liquid_ul"),
                              "commanded_displacement_ul": op.get("volume_ul"), "liquid_phase": kind}
                    if request.get("accounting_context") is not None:
                        inputs.update(accounting_context=request["accounting_context"],
                                      accounting_scope=command_id + ":" + owner_identity["source_identity"])
                    if op.get("accounting_intent") is not None:
                        inputs["accounting_intent"] = op["accounting_intent"]
                    record(effect, lambda t, w=wire, e=effect, values=inputs:
                        _send_wire(t, op["channels"], w, op["timeout_ms"], effect=e, inputs=values,
                                   after_sends=start_tracking if tracking and e == "aspirate" else None),
                        inputs={"operation_index": index, "field": field, "ascii": wire,
                                "channels": op["channels"], **inputs}, pipette=True)
                if tracking:
                    record("tracking_z_wait", lambda: provider.wp8_wait_z("waitZ",
                        {"timeout_ms": op["z_timeout_ms"]}), inputs={"timeout_ms": op["z_timeout_ms"]})
        result.update(ok=True, completed=True, source_return_completed=True,
                      has_unknown_outcomes=any(e["status"] == "unknown" for e in events))
    except Exception as exc:
        result.update(error=str(exc), exception_type=type(exc).__name__,
                      interrupted=epoch != group._interrupt_epoch,
                      event_policy=request.get("event_policy", "stop"),
                      requested_control=request.get("event_policy", "stop"),
                      partial_effects=True)
    if request.get("accounting_context") is not None:
        from .effect_accounting import application_accounting
        try:
            result["accounting_summary"] = application_accounting(events, request["accounting_context"], incomplete=not result["completed"])
        except Exception as exc:
            result["accounting_observation_error"] = str(exc)
    return result
