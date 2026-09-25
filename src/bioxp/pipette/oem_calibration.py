"""OEM manual tip pickup and fluid-height measurement, not a second controller.

Invoke these bodies *inside* the existing canonical motion/workflow owner.
Bindings execute existing finite deck plans and shared-pipette receipt operations;
they must not construct transports or schedule independent hardware work.
Source: SSD ControlLib.cs:1324-1368,3387-3627,4782-4841,7887-7902.
"""
from __future__ import annotations

from dataclasses import dataclass
import re
import time
from typing import Any, Callable, Mapping

from .models import PipetteCommandError
from .transport import FourPipetteTransport
from ..oem_compat.position_table import well_id_from_label

Result = dict[str, Any]


@dataclass(frozen=True)
class ManualTipLoadRequest:
    tray: int  # OEM UI is one-based, including hotel = 5
    well: str  # tipLocation: A1,B1,...,A12,B12 (four-channel pickup)
    overpress: bool = False
    lift_z: bool = False

    def __post_init__(self) -> None:
        if type(self.tray) is not int or not 1 <= self.tray <= 5:
            raise ValueError("tray must be an OEM tray number 1..5")
        if not isinstance(self.well, str) or not re.fullmatch(r"[ABab](?:[1-9]|1[0-2])", self.well):
            raise ValueError("tip well must be A1..B12")
        if type(self.overpress) is not bool or type(self.lift_z) is not bool:
            raise ValueError("overpress and lift_z must be booleans")


@dataclass(frozen=True)
class FluidHeightRequest:
    speed: int = 300

    def __post_init__(self) -> None:
        if type(self.speed) is not int:
            raise ValueError("speed must be an integer OEM Z speed")


@dataclass(frozen=True)
class CalibrationBindings:
    """Trusted dependencies, not a user-supplied command language.

    finite(operation, inputs) executes a canonical finite deck plan.
    native(name, callback, inputs) executes a native child on that same owner.
    pipette(name, callback) executes against its existing shared collection.
    facts/tray_location are existing source-model readers, not new admissions.
    """
    finite: Callable[[str, Mapping[str, Any]], Result]
    native: Callable[[str, Callable[[], Result], Mapping[str, Any]], Result]
    pipette: Callable[[str, Callable[[FourPipetteTransport], Result]], Result]
    facts: Callable[[], Mapping[str, Any]]
    tray_location: Callable[[int], int]
    lower_pipette: Callable[[int, bool], Result]
    z_low: Callable[[int], int | None]
    set_z_speed: Callable[[int], Result]
    detection_move: Callable[[int], Result]
    stop_z: Callable[[], Result]
    sleep: Callable[[float], None] = time.sleep
    clock: Callable[[], float] = time.monotonic


class CalibrationExecutionError(RuntimeError):
    def __init__(self, message: str, evidence: Result):
        super().__init__(message)
        self.evidence = evidence


class _Sequence:
    def __init__(self, source: str):
        self.result: Result = {"ok": True, "source_anchor": source, "steps": [],
                               "physical_effect_verified": False, "calibration_persisted": False}

    def step(self, name: str, call: Callable[[], Result], *, allow_false: bool = False) -> Result:
        try:
            result = call()
        except Exception as exc:
            if isinstance(exc, CalibrationExecutionError) and exc.evidence is self.result:
                raise
            self.result["ok"] = False
            self.result["steps"].append({"operation": name, "error": repr(exc),
                                          "evidence": getattr(exc, "evidence", None)})
            raise CalibrationExecutionError(str(exc), self.result) from exc
        self.result["steps"].append({"operation": name, "result": result})
        # Preserve actual native failures. This is not an extra observation gate.
        if result.get("ok") is False and not allow_false:
            self.result["ok"] = False
            raise CalibrationExecutionError(f"{name} failed", self.result)
        return result


def manual_load_tip(request: ManualTipLoadRequest, bindings: CalibrationBindings) -> Result:
    """Physical btnLoadTip_Click body. No loadTip(host-only) substitution.

    The source does NOT remove inventory, set TipLoaded, change tip type, select
    a pipette channel or invent an occupied-tip admission check.
    """
    b = bindings
    seq = _Sequence("ControlLib.btnLoadTip_Click:1324-1368")
    location = b.tray_location(request.tray - 1)
    well = well_id_from_label(request.well)
    row, column = divmod(well, 12)
    seq.result.update(tray=request.tray, well=request.well, location=location)
    seq.step("moveTo", lambda: b.finite("pipette_hotel", dict(
        location=location, column=column, row=row, high_pos=True, run_in_parallel=False)))
    seq.step("updateLocation", lambda: b.finite("pipette_location", dict(destination=location, well=well)))
    seq.step("lowerPipette", lambda: b.native("lowerPipette", lambda: b.lower_pipette(location, request.overpress),
                                               {"location": location, "overpress": request.overpress}))
    seq.step("queryTipStatus", lambda: b.pipette("query_tip_status_all", lambda t: t.query_tip_status_all()))
    if request.lift_z:
        home = seq.step("MoveZHome", lambda: b.finite("pipette_home", {"rehome": False}))
        lost = home.get("source_return")
        seq.result["lost_steps"] = lost
        seq.result["lost_steps_warning"] = abs(lost) > 100 if isinstance(lost, (int, float)) else None
    seq.step("TipDirty=false", lambda: b.finite("pipette_tip_state", {"changes": {"tip_dirty": False}}))
    return seq.result


def detect_fluid_with_motion(transport: FourPipetteTransport, move: Callable[[], Result],
                             *, sleep: Callable[[float], None] = time.sleep) -> Result:
    """Send all four BRs, start nonblocking Z, THEN wait on their owners.

    Existing detect_fluid() waits before returning and cannot be followed by Z
    for ControlLib.detectFluidLevel. Reuse its transport's completion owner and
    interrupt epoch instead of creating a second connection or background task.
    """
    if transport._forceabort():
        raise PipetteCommandError("Stopped by user or force abort")
    motion: Result | None = None

    def after_sends() -> None:
        nonlocal motion
        sleep(0.001)  # ClassPipetteCollection.detectfluid:1388
        motion = move()

    result = transport._run_group_liquid_operation(
        "fluid detection", list(transport.CHANNELS),
        lambda channel, child, defer: child.start_fluid_detection(wait_for_completion=not defer),
        timeout_ms=15_000, after_sends=after_sends, set_allow_to_stop=False)
    timestamps = []
    for row in result["channels"]:
        state = row.get("completion", {}).get("pipette_message_state", {})
        stamp = state.get("fluid_timestamp")
        timestamps.append(stamp)
        transport._fluid_detection_timestamps[row["channel"]] = stamp
    return {**result, "motion": motion, "fluid_timestamps": timestamps,
            "planned_command": "BR", "source_anchor": "ControlLib.detectFluidLevel:7887-7902"}


def evaluate_fluid_timing(delays: list[float]) -> Result:
    """Literal four-channel OEM outlier calculation; not an admission policy."""
    if len(delays) != 4:
        raise ValueError("four channel delays required")
    ordered = sorted(delays)
    median = (ordered[1] + ordered[2]) / 2.0
    spread = (ordered[2] + ordered[3]) / 2.0 - (ordered[0] + ordered[1]) / 2.0
    threshold = max(median + 1.5 * spread, median + 1.0)
    failed = [i + 1 for i, delay in enumerate(delays) if delay > threshold or delay > 10]
    return {"delays_s": delays, "threshold_s": threshold, "failed_pipettes_1_based": failed}


def aspirate_calibration_air(transport: FourPipetteTransport) -> Result:
    """ClassPipetteCollection.AspirateAir(15,false):884-913.

    The OEM ignores its wait bool. Preserve it as evidence rather than turning
    a lost completion observation into a new source exception.
    """
    selected = transport._tip_location_channels()
    speed_channel = transport._tip_location if transport._tip_location != -1 else 0
    timeout = 5 * int(15 / transport._transports[speed_channel]._top_speed * 1000) + 4000
    result = transport._run_group_liquid_operation(
        "aspirate air", selected,
        lambda channel, child, defer: child.aspirate_air(15, front_air=False, wait_for_completion=not defer),
        timeout_ms=timeout, post_send_delay_s=.001, after_sends=lambda: transport._sleep(.020),
        set_allow_to_stop=False)
    return {**result, "ok": not result["interrupted_by_terminate"],
            "source_wait_return": result["ok"], "source_return_completed": True,
            "volume_ul": 15, "front_air": False, "planned_command": "P15,1R"}


def measure_fluid_height(request: FluidHeightRequest, bindings: CalibrationBindings) -> Result:
    """scrFluidDetection at the current source well; does not calibrate config.

    Position at the requested named well first using the parent's well workflow.
    Timeout and source outlier exceptions intentionally do not add an invented
    speed/lift/dispense finally. Existing Stop remains owned by the controller.
    """
    b = bindings
    seq = _Sequence("ControlLib.scrFluidDetection:4782-4841")
    facts = b.facts()
    location = facts["current_location"]
    seq.result.update(location=location, well_id=facts["current_well"], speed=request.speed)
    seq.step("liftTo", lambda: b.finite("pipette_lift", {"location": location, "height": None}))
    seq.step("AspirateAir", lambda: b.pipette("aspirate_calibration_air", aspirate_calibration_air))
    b.sleep(0.500)
    before = seq.step("getCurrentPosition", lambda: b.finite("pipette_position", {}))
    seq.step("setMaxSpeed", lambda: b.native("setMaxSpeed", lambda: b.set_z_speed(request.speed), {"speed": request.speed}))
    started = b.clock()  # shared router timestamps use monotonic seconds
    z_low = b.z_low(location)
    if z_low is None:
        raise TypeError("PositionTable zLow is absent; no detection target can be calculated")
    target = z_low + 2015

    def move() -> Result:
        return seq.step("moveZ", lambda: b.native("fluidDetectionMoveZ", lambda: b.detection_move(target),
                                                  {"position": target, "wait_for_stop": False}))

    # Source wait failure has its own TR branch, before throwing.
    detection = seq.step("detectFluidLevel", lambda: b.pipette(
        "detect_fluid_level", lambda t: detect_fluid_with_motion(t, move, sleep=b.sleep)), allow_false=True)
    if not detection["ok"]:
        seq.step("terminatecommands", lambda: b.pipette("terminate", lambda t: t.terminate()))
        seq.result["ok"] = False
        raise CalibrationExecutionError("Detect fluid level timeout", seq.result)
    seq.step("stopMotor(z)", lambda: b.native("stopMotorZ", b.stop_z, {}))
    position = seq.step("getCurrentPosition", lambda: b.finite("pipette_position", {}))["z"]
    stamps = detection["fluid_timestamps"]
    timing = evaluate_fluid_timing([stamp - started for stamp in stamps]) if all(
        isinstance(stamp, (int, float)) for stamp in stamps) else None
    seq.result.update(position_steps=position, source_return=position, detection_target_steps=target,
                      fluid_timestamps=stamps, timing=timing)
    # Absent observation is evidence, not a new physical gate.
    if timing is not None and timing["failed_pipettes_1_based"]:
        seq.result["ok"] = False
        raise CalibrationExecutionError("Fluilid clibration failure", seq.result)
    after = seq.step("getCurrentPosition", lambda: b.finite("pipette_position", {}))
    if timing is not None and max(timing["delays_s"]) != 0:
        seq.result["observed_speed_mm_s"] = (after["z"] - before["z"]) / 2015.748 / max(timing["delays_s"])
    seq.step("restoreMaxSpeed", lambda: b.native("setMaxSpeed", lambda: b.set_z_speed(1791), {"speed": 1791}))
    seq.step("liftTo", lambda: b.finite("pipette_lift", {"location": location, "height": None}))
    seq.step("DispenseAll", lambda: b.pipette("dispense_all", lambda t: t.dispense_all()))
    return seq.result


def bind_calibration_provider(provider: Any, *, finite: Callable, native: Callable,
                              pipette: Callable, tray_location: Callable[[int], int],
                              z_current_down: int, sleep: Callable = time.sleep,
                              clock: Callable = time.monotonic) -> CalibrationBindings:
    """Connect real provider primitives to the existing owner's child runners.

    finite takes (compiler-operation, explicit inputs); native takes (name,
    zero-argument body, explicit inputs); pipette takes (name, collection body).
    Resolve tray_location from the canonical source model's get_tip_tray_location.
    z_current_down is captured OEM Z_MOTOR_MAX_CURRENT_DOWN, never guessed.
    """
    from ..oem_compat.position_table import load_bound_oem_position_table
    from ..oem_compat.pathing import LOCATION_ID_TO_NAME

    def lower(location: int, overpress: bool) -> Result:
        return provider.primitives.z_pipette_position(
            location_id=LOCATION_ID_TO_NAME[location], operation="lower_pipette", overpress=overpress)

    def detection_move(target: int) -> Result:
        state = provider._offset_deck_semantic_state(gripper_confirmed=False, pseudo_home_only=True)
        return provider.primitives.oem_move_z(target, pseudo_home_steps=state["pseudo_z_home"],
                                             motor_current=z_current_down, wait_for_stop=False)

    return CalibrationBindings(finite=finite, native=native, pipette=pipette,
        facts=provider.mov_execution_machine_state, tray_location=tray_location,
        lower_pipette=lower,
        z_low=lambda location: load_bound_oem_position_table().resolve(location_id=LOCATION_ID_TO_NAME[location]).z_low,
        set_z_speed=provider.primitives.z_set_max_speed, detection_move=detection_move,
        stop_z=provider.primitives.z_stop, sleep=sleep, clock=clock)


CALIBRATION_PLATES = {"TC": (2, 0, 12, 25), "MS": (0, 0, 12, 25),
                      "OC": (1, 1, 12, 25), "RC": (3, 2, 12, 40),
                      "STRIP": (12, 8, 3, 40), "OCMS": (0, 1, 12, 40)}


def calibration_samples(plate: str, skip_steps: int = 4) -> tuple[str, ...]:
    """Exact zOffset A/B sample order; diagnostic uses skip=1, cal uses 4."""
    if plate not in CALIBRATION_PLATES or type(skip_steps) is not int or skip_steps <= 0:
        raise ValueError("known OEM calibration plate and positive skip_steps required")
    return tuple(f"{row}{column + 1}" for column in range(0, CALIBRATION_PLATES[plate][2], skip_steps)
                 for row in ("A", "B"))


def calibration_adjustment(plate: str, measured_z: int, *, z_lows: Mapping[int, int],
                           fluid_reference: Mapping[str, int]) -> Result:
    """ClassBioXPSettings.adjustZ arithmetic ONLY. Never writes the active table.

    Caller must supply actual measured data and captured FluidReference. Output
    is a proposal, not a claim of measurement, calibration or persistence.
    """
    location = CALIBRATION_PLATES[plate][0]
    field = "FLUID_STRIP_OFFSET" if plate == "STRIP" else "FLUID_RC_OFFSET" if plate in {"RC", "OCMS"} else "FLUID_TC_OFFSET"
    new_low = measured_z + fluid_reference[field] - (1500 if plate == "STRIP" else 0)
    updates = {} if plate == "OCMS" else {location: new_low}
    if plate == "STRIP":
        delta = new_low - z_lows[12]
        updates = {loc: z_lows[loc] + delta for loc in (12, 11, 13, 14)}
    return {"source_anchor": "ClassBioXPSettings.adjustZ:6039-6138", "plate": plate,
            "position_table_z_low_updates": updates,
            "settings_updates": {"m_current_tool": "FluidReference", "m_liquid_cal_revision": str(fluid_reference["REVISION"]),
                                 **({"m_PLLow": new_low} if plate == "MS" else {}),
                                 **({"m_OPBufferLow": new_low} if plate == "OCMS" else {})},
            "calibration_persisted": False, "proposal_only": True}
