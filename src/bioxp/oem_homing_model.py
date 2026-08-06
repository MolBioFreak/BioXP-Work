"""Source-anchored OEM BioXP homing model.

This module is intentionally data/model only.  It must not import the live USB
runtime, open hardware, or execute motion.  It captures the separated OEM modes
Christian asked for before any further Linux route repair: startup preparation,
full startup homing, manual button homing, HomeAxis/HomeXY utilities, rehome,
initializeMotion, board primitives, and raw-FastAPI-vs-BMS route semantics.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Iterable


@dataclass(frozen=True)
class OemSourceAnchor:
    """Exact source location backing a model step."""

    file: str
    lines: str
    symbol: str
    note: str = ""


@dataclass(frozen=True)
class OemTraceStep:
    """One ordered source-level operation in an OEM mode."""

    name: str
    source: OemSourceAnchor
    operation: str
    axis: str | None = None
    board: str | None = None
    motor: int | None = None
    params: dict[str, Any] | None = None
    notes: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        if payload["params"] is None:
            payload["params"] = {}
        return payload




@dataclass(frozen=True)
class LiveTargetMapping:
    """Current Linux target for an OEM source mode/primitive.

    This is a static no-motion audit row.  It does not prove that the target is
    physically equivalent; it states where the current code appears to route and
    labels deviations that still block calling it a clean OEM port.
    """

    source_mode: str
    target_file: str
    target_symbol: str
    target_line: int
    target_status: str
    deviations: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ApiRouteMapping:
    """Route-to-OEM-mode classification; no network calls are made here."""

    surface: str
    route: str
    method: str
    maps_to: str
    equivalence: str
    notes: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


CCI = "BioXPControlLib/ClassControlInterface.cs"
CL = "BioXPControlLib/ControlLib.cs"
DECK = "ClassCanLib/ClassDeckBoard.cs"
HEAD = "ClassCanLib/ClassHeadBoard.cs"
THERMAL = "ClassCanLib/ClassThermalBoard.cs"
MOTOR = "ClassCanLib/ClassMotor.cs"
API = "src/bioxp/api.py"
USB = "src/bioxp/usb_driver.py"
BMS = "BioModStack/BMS proxy /api/bioxp/* surface"


AXIS_TO_BOARD = {
    "x": {"board": "deck/CAN5", "motor": 0, "oem_designator": "MotorX"},
    "y": {"board": "head/CAN4", "motor": 0, "oem_designator": "MotorY"},
    "z": {"board": "head/CAN4", "motor": 1, "oem_designator": "MotorZ"},
    "g": {"board": "head/CAN4", "motor": 2, "oem_designator": "MotorGrip"},
    "door": {"board": "thermal/CAN6", "motor": 0, "oem_designator": "ThermalDoor"},
}


BOARD_PRIMITIVE_ANCHORS: dict[str, OemSourceAnchor] = {
    "goHome.deck": OemSourceAnchor(DECK, "73-132", "ClassDeckBoard.goHome", "move-left search, stop on home, setHome"),
    "goHome.head": OemSourceAnchor(HEAD, "60-119", "ClassHeadBoard.goHome", "same host algorithm for head-board axes"),
    "goHome.thermal": OemSourceAnchor(THERMAL, "118-177", "ClassThermalBoard.goHome", "same host algorithm for thermal-board motor"),
    "axisSearchHome.deck": OemSourceAnchor(DECK, "362-380", "ClassDeckBoard.axisSearchHome", "setHome; optional preclear to 10000; MotorHome=false; goHome(false)") ,
    "axisSearchHome.head": OemSourceAnchor(HEAD, "368-386", "ClassHeadBoard.axisSearchHome", "setHome; optional preclear to 10000; MotorHome=false; goHome(false)"),
    "axisSearchHome.thermal": OemSourceAnchor(THERMAL, "412-430", "ClassThermalBoard.axisSearchHome", "setHome; optional preclear to 10000; MotorHome=false; goHome(false)"),
    "doorSearchHome.deck": OemSourceAnchor(DECK, "318-360", "ClassDeckBoard.doorSearchHome", "stallguard preclear then stall/home search"),
    "doorSearchHome.head": OemSourceAnchor(HEAD, "320-362", "ClassHeadBoard.doorSearchHome", "stallguard preclear then stall/home search"),
    "doorSearchHome.thermal": OemSourceAnchor(THERMAL, "364-410", "ClassThermalBoard.doorSearchHome", "thermal door throws if calibrated and home not found"),
    "queryHome.deck": OemSourceAnchor(DECK, "383-395", "ClassDeckBoard.queryHome", "left switch status 0 means home true"),
    "queryHome.head": OemSourceAnchor(HEAD, "389-401", "ClassHeadBoard.queryHome", "left switch status 0 means home true"),
    "queryHome.thermal": OemSourceAnchor(THERMAL, "433-445", "ClassThermalBoard.queryHome", "left switch status 0 means home true"),
    "queryRightSensor.deck": OemSourceAnchor(DECK, "397-409", "ClassDeckBoard.queryRightSensor", "right switch status 0 means right sensor true"),
    "queryLeftSwitchStatus.motor": OemSourceAnchor(MOTOR, "641-664", "ClassMotor.queryLeftSwitchStatus", "CAN payload {6,9,axis,...}; reply byte 6 == 1 returns 0"),
    "queryRightSwitchStatus.motor": OemSourceAnchor(MOTOR, "666-689", "ClassMotor.queryRightSwitchStatus", "CAN payload {6,10,axis,...}; reply byte 6 == 1 returns 0"),
    "setHome.motor": OemSourceAnchor(MOTOR, "492-517", "ClassMotor.setHome", "sets current position 0, clears stall, CAN payload {5,1,axis,...}"),
}


def _step(
    name: str,
    source: OemSourceAnchor,
    operation: str,
    *,
    axis: str | None = None,
    params: dict[str, Any] | None = None,
    notes: Iterable[str] = (),
) -> OemTraceStep:
    board = AXIS_TO_BOARD.get(axis or "", {}).get("board")
    motor = AXIS_TO_BOARD.get(axis or "", {}).get("motor")
    return OemTraceStep(
        name=name,
        source=source,
        operation=operation,
        axis=axis,
        board=board,
        motor=motor,
        params=params or {},
        notes=tuple(notes),
    )


INITIALIZE_MOTORS_WITHOUT_MOTION_TRACE: tuple[OemTraceStep, ...] = (
    _step("x.setup.speed_acc", OemSourceAnchor(CCI, "3187-3189", "initializeMotorsWithoutMotion"), "setSpeedAcc", axis="x", params={"speed": 1700, "acc": 350}),
    _step("x.setup.current", OemSourceAnchor(CCI, "3190-3192", "initializeMotorsWithoutMotion"), "setMaxCurrent/readMaxCurrent", axis="x", params={"run_current": 31}),
    _step("x.setup.stallguard", OemSourceAnchor(CCI, "3193-3194", "initializeMotorsWithoutMotion"), "setStallGuardThreshold", axis="x", params={"threshold": 16}),
    _step("y.setup.speed_acc", OemSourceAnchor(CCI, "3196-3198", "initializeMotorsWithoutMotion"), "setSpeedAcc", axis="y", params={"speed": 1800, "acc": 400}),
    _step("y.setup.current", OemSourceAnchor(CCI, "3199-3201", "initializeMotorsWithoutMotion"), "setMaxCurrent/readMaxCurrent", axis="y", params={"run_current": 31}),
    _step("y.setup.stallguard_disable_right", OemSourceAnchor(CCI, "3202-3205", "initializeMotorsWithoutMotion"), "setStallGuardThreshold + disableRightSwitch", axis="y", params={"threshold": 16}),
    _step("z.setup.speed_acc", OemSourceAnchor(CCI, "3207-3209", "initializeMotorsWithoutMotion"), "setSpeedAcc", axis="z", params={"speed": 1791, "acc": 576}),
    _step("z.setup.current_stallguard", OemSourceAnchor(CCI, "3210-3215", "initializeMotorsWithoutMotion"), "setMaxCurrent/readMaxCurrent/setStallGuardThreshold", axis="z", params={"run_current": "Z_MOTOR_MAX_CURRENT_UP", "stallguard": "Z_MOTOR_STALL_GUARD_THRESHOLD"}),
    _step("g.setup.versioned", OemSourceAnchor(CCI, "3217-3242", "initializeMotorsWithoutMotion"), "setSpeedAcc/current/stallguard/RDIV/PDIV", axis="g", params={"speed": "600 or 200 by GripperVersion", "run_current": "31 or 10 by GripperVersion", "rdiv": 6, "pdiv": 2}),
    _step("door.setup", OemSourceAnchor(CCI, "3244-3255", "initializeMotorsWithoutMotion"), "setSpeedAcc/current/stallguard/disable switches", axis="door", params={"speed": "TC_DOOR_VELOCITY", "acc": "TC_DOOR_ACCELERATION", "run_current": "TC_DOOR_MAX_CURRENT", "stallguard": "TCDoorStallGuardThreshold"}),
)


INITIALIZE_MOTORS_TRACE: tuple[OemTraceStep, ...] = (
    _step("z.axisSearchHome", OemSourceAnchor(CCI, "3350-3353", "initializeMotors"), "axisSearchHome", axis="z", params={"speed": 1791}),
    _step("g.setMaxCurrent.before_clear", OemSourceAnchor(CCI, "3354", "initializeMotors"), "setMaxCurrent", axis="g", params={"run_current": 31}),
    _step("g.clear.moveSteps", OemSourceAnchor(CCI, "3355", "initializeMotors"), "moveSteps", axis="g", params={"steps": 10000, "waitforstop": True}),
    _step("g.axisSearchHome", OemSourceAnchor(CCI, "3356-3365", "initializeMotors"), "axisSearchHome", axis="g", params={"speed": "600 if GripperVersion==0 else 200"}),
    _step("x.axisSearchHome", OemSourceAnchor(CCI, "3367-3369", "initializeMotors"), "axisSearchHome", axis="x", params={"speed": 250}),
    _step("x.setHome", OemSourceAnchor(CCI, "3370-3371", "initializeMotors"), "setHome", axis="x"),
    _step("x.setSpeed.restore", OemSourceAnchor(CCI, "3372-3373", "initializeMotors"), "setSpeed", axis="x", params={"speed": 1700}),
    _step("x.park_6000", OemSourceAnchor(CCI, "3374-3375", "initializeMotors"), "moveX", axis="x", params={"position": 6000}),
    _step("y.axisSearchHome", OemSourceAnchor(CCI, "3376-3379", "initializeMotors"), "axisSearchHome", axis="y", params={"speed": 250}),
    _step("door.doorSearchHome", OemSourceAnchor(CCI, "3380-3383", "initializeMotors"), "doorSearchHome", axis="door", params={"speed": "TC_DOOR_VELOCITY", "stallguard": "TCDoorStallGuardThreshold"}),
    _step("y.setHome.final", OemSourceAnchor(CCI, "3389-3392", "initializeMotors"), "setHome", axis="y"),
    _step("g.restore_current.version1", OemSourceAnchor(CCI, "3417-3420", "initializeMotors"), "setMaxCurrent", axis="g", params={"run_current": 10, "condition": "GripperVersion == 1"}),
)


MANUAL_HOME_TRACE: dict[str, tuple[OemTraceStep, ...]] = {
    "x": (_step("manual.x.goHome", OemSourceAnchor(CCI, "2262-2274", "btnHomeX_Click"), "goHome", axis="x", params={"rehome": True, "speed": 500, "waitforstop": True}),),
    "y": (_step("manual.y.goHome", OemSourceAnchor(CCI, "2302-2314", "btnHomeY_Click"), "goHome", axis="y", params={"rehome": True, "speed": 500, "waitforstop": True}),),
    "z": (_step("manual.z.goHome", OemSourceAnchor(CCI, "2350-2362", "btnHomeZ_Click"), "goHome", axis="z", params={"rehome": True, "speed": 1791, "waitforstop": True}),),
    "g": (
        _step("manual.g.current_high", OemSourceAnchor(CCI, "2046-2054", "btnGripperHome_Click"), "setMaxCurrent", axis="g", params={"run_current": 31}),
        _step("manual.g.goHome", OemSourceAnchor(CCI, "2059-2064", "btnGripperHome_Click"), "goHome", axis="g", params={"rehome": True, "speed": "600 or 200 by GripperVersion", "waitforstop": True}),
        _step("manual.g.restore_current", OemSourceAnchor(CCI, "2065-2069", "btnGripperHome_Click"), "setMaxCurrent", axis="g", params={"run_current": 10, "condition": "GripperVersion == 1"}),
    ),
    "door": (_step("manual.door.doorSearchHome", OemSourceAnchor(CCI, "1224-1240", "btnDHome_Click"), "doorSearchHome", axis="door", params={"speed": "TC_DOOR_VELOCITY", "stallguard": "TCDoorStallGuardThreshold"}),),
}


HOME_AXIS_TRACE: tuple[OemTraceStep, ...] = (
    _step("HomeAxis.x.axisSearchHome", OemSourceAnchor(CCI, "5002-5007", "HomeAxis"), "axisSearchHome", axis="x", params={"speed": 250}),
    _step("HomeAxis.y.axisSearchHome", OemSourceAnchor(CCI, "5009-5014", "HomeAxis"), "axisSearchHome", axis="y", params={"speed": 250}),
    _step("HomeAxis.z.current_axisSearchHome", OemSourceAnchor(CCI, "5016-5022", "HomeAxis"), "setMaxCurrent + axisSearchHome", axis="z", params={"speed": 597, "run_current": 31}),
    _step("HomeAxis.g.current_stall_axisSearchHome", OemSourceAnchor(CCI, "5024-5035", "HomeAxis"), "setMaxCurrent + setStallGuard + axisSearchHome", axis="g", params={"speed": "200 or 150 by GripperVersion"}),
    _step("HomeAxis.door.preclear_doorSearchHome", OemSourceAnchor(CCI, "5037-5048", "HomeAxis"), "doorSearchHome", axis="door"),
)


HOME_XY_TRACE: tuple[OemTraceStep, ...] = (
    _step("HomeXY.set_xy_speedacc_200", OemSourceAnchor(CCI, "5056-5061", "HomeXY"), "setSpeedAcc", params={"x": [200, 200], "y": [200, 200]}),
    _step("HomeXY.parallel_x_goHome", OemSourceAnchor(CCI, "5062", "HomeXY"), "goHome", axis="x", params={"rehome": False, "speed": 200, "waitforstop": True}),
    _step("HomeXY.parallel_y_goHome", OemSourceAnchor(CCI, "5063", "HomeXY"), "goHome", axis="y", params={"rehome": False, "speed": 200, "waitforstop": True}),
    _step("HomeXY.restore_xy_speedacc", OemSourceAnchor(CCI, "5064-5067", "HomeXY"), "setSpeedAcc", params={"x": [1700, 350], "y": [1800, 400]}),
)


REHOME_TRACE: tuple[OemTraceStep, ...] = (
    _step("rehome.save_door_state", OemSourceAnchor(CL, "8784-8788", "ControlLib.rehome"), "save door-open state"),
    _step("rehome.initializeMotors", OemSourceAnchor(CL, "8789", "ControlLib.rehome"), "initializeMotors"),
    _step("rehome.sleep_restore_door", OemSourceAnchor(CL, "8790-8795", "ControlLib.rehome"), "sleep and restore door/thermal state"),
)


INITIALIZE_MOTION_TRACE: tuple[OemTraceStep, ...] = (
    _step("initializeMotion.flags", OemSourceAnchor(CL, "8797-8810", "ControlLib.initializeMotion"), "set motion/script stop flags"),
    _step("initializeMotion.initializeMotors", OemSourceAnchor(CL, "8811-8814", "ControlLib.initializeMotion"), "initializeMotors"),
    _step("initializeMotion.tip_pipette_cleanup", OemSourceAnchor(CL, "8815-8848", "ControlLib.initializeMotion"), "tip/pipette cleanup and park-related motion"),
)




RAW_FASTAPI_ROUTE_TABLE: tuple[dict[str, str], ...] = (
    {"methods": "POST", "path": "/motion/oem/home_xy", "name": "motion_oem_home_xy", "classification": "direct HomeXY mode surface; guarded X/Y switch-search"},
    {"methods": "POST", "path": "/motion/oem/initialization/initialize_motors", "name": "motion_oem_serial206_initialize_motors", "classification": "canonical serial-206 initializeMotors provider"},
    {"methods": "POST", "path": "/motion/oem/initialization/initialize_motion", "name": "motion_oem_serial206_initialize_motion", "classification": "canonical serial-206 initializeMotion provider"},
    {"methods": "GET", "path": "/motion/oem/initialization/provider-status", "name": "motion_oem_initialization_provider_status", "classification": "canonical serial-206 initialization state"},
    {"methods": "POST", "path": "/motion/axis/home", "name": "home_axis", "classification": "manual/goHome-style route"},
    {"methods": "POST", "path": "/motion/axis/zero", "name": "move_axis_zero", "classification": "Linux absolute controller-zero, not OEM homing"},
    {"methods": "POST", "path": "/motion/arm/strict_startup", "name": "motion_arm_strict_startup", "classification": "strict startup arm; homing guarded/blocked by policy"},
    {"methods": "POST", "path": "/oem/startup/request", "name": "oem_startup_request", "classification": "OEM app/startup artifact state machine"},
    {"methods": "POST", "path": "/oem/initial_check", "name": "oem_initial_check", "classification": "ControlLib.initialCheck-style diagnostic"},
    {"methods": "POST", "path": "/oem/switch_audit", "name": "oem_switch_audit", "classification": "predicate audit; not motion proof"},
    {"methods": "GET", "path": "/oem/runtime/status", "name": "runtime_status", "classification": "OEM runtime worker/status surface"},
)


LIVE_TARGET_MAPPINGS: tuple[LiveTargetMapping, ...] = (
    LiveTargetMapping(
        "initializeMotorsWithoutMotion",
        USB,
        "BioXpTester.motor_oem_initialize_without_motion",
        3397,
        "implemented_source_shaped_setup",
        ("Constants are partly reconstructed/defaulted without recovered machine config.xml.",),
    ),
    LiveTargetMapping(
        "initializeMotors/initializeMotion",
        API,
        "Serial206OemInitializationProvider",
        1307,
        "canonical_atomic_serial206_authority",
        ("The provider owns admission, physical execution, atomic state, observation, and receipts.",),
    ),
    LiveTargetMapping(
        "startup axisSearchHome",
        USB,
        "BioXpTester.motor_oem_axis_search_home",
        3418,
        "partial_guarded_reconstruction",
        ("Poll/guard implementation is Linux reconstruction of board primitive semantics.", "Z startup may bypass source GAP9 search via GAP10/controller-zero workaround."),
    ),
    LiveTargetMapping(
        "manual button goHome(true)",
        USB,
        "BioXpTester.motor_oem_go_home / motor_oem_home_axis(startup=False)",
        3510,
        "unsafe_until_predicate_matrix_fixed",
        ("Manual /motion/axis/home has known Z/X failure incidents and must not be treated as proven true homing.", "Requires switch deassert->active proof before setHome/reporting reference."),
    ),
    LiveTargetMapping(
        "doorSearchHome",
        USB,
        "BioXpTester.motor_oem_door_search_home",
        3725,
        "partial_guarded_reconstruction",
        ("Door search is implemented separately but still needs physical predicate proof for parity.",),
    ),
    LiveTargetMapping(
        "HomeAxis",
        USB,
        "BioXpTester.motor_oem_switch_search_home_axis / motor_oem_home_axis",
        3787,
        "not_clean_one_to_one_port",
        ("Current public home route routes through Linux helper selection, not a direct HomeAxis source clone.",),
    ),
    LiveTargetMapping(
        "HomeXY",
        USB,
        "BioXpTester.motor_oem_home_xy / motion_oem_home_xy",
        3925,
        "direct_oem_parallel_task_run_waitall",
        ("Direct HomeXY label/setup/restore surface exists.", "X and Y goHome(false, axis, 200, true) are launched concurrently to match OEM Task.Run/WaitAll semantics.", "This is a source-parity surface, not a manual single-axis home or controller-zero route."),
    ),
)


ROUTE_MAPPINGS: tuple[ApiRouteMapping, ...] = (
    ApiRouteMapping("raw-fastapi", "/motion/oem/home_xy", "POST", "direct HomeXY mode surface", "not-equivalent-to-single-axis-home-or-zero", ("Preserves HomeXY source-mode label and launches X/Y goHome concurrently like OEM Task.Run/WaitAll.", "This is not manual single-axis Home and not controller Zero.")),
    ApiRouteMapping("raw-fastapi", "/motion/oem/initialization/initialize_motors", "POST", "canonical serial-206 initializeMotors stage", "canonical-provider-authority", ("Admission, execution, observation, and receipt state are atomic in the serial-206 provider.",)),
    ApiRouteMapping("raw-fastapi", "/motion/oem/initialization/initialize_motion", "POST", "canonical serial-206 initializeMotion stage", "canonical-provider-authority", ("The provider advances only the expected approved stage.",)),
    ApiRouteMapping("raw-fastapi", "/motion/axis/home", "POST", "manual button goHome-style route", "not-equivalent-to-startup-axisSearchHome", ("Historically routed through _execute_home_axis(... startup=False).", "Unsafe until per-axis predicates/transitions are repaired and proven.")),
    ApiRouteMapping("raw-fastapi", "/motion/axis/zero", "POST", "Linux absolute controller-zero route", "linux-only-not-oem-home", ("Return-to-controller-zero is not switch/reference homing.",)),
    ApiRouteMapping("bms-proxy", "/api/bioxp/operator-controls/actions/{action_id}/invoke", "POST", "robot-owned catalog action invocation", "proxy-not-authority", ("BMS forwards robot action identifiers and robot admission receipts without local mutation policy.",)),
)


MODEL_TRACES: dict[str, tuple[OemTraceStep, ...]] = {
    "initializeMotorsWithoutMotion": INITIALIZE_MOTORS_WITHOUT_MOTION_TRACE,
    "initializeMotors": INITIALIZE_MOTORS_TRACE,
    "HomeAxis": HOME_AXIS_TRACE,
    "HomeXY": HOME_XY_TRACE,
    "rehome": REHOME_TRACE,
    "initializeMotion": INITIALIZE_MOTION_TRACE,
}


def trace_for(mode: str, axis: str | None = None) -> tuple[OemTraceStep, ...]:
    """Return an ordered no-motion OEM trace for a mode."""

    if mode == "manual_home":
        if axis is None:
            raise ValueError("manual_home requires axis")
        try:
            return MANUAL_HOME_TRACE[axis]
        except KeyError as exc:
            raise ValueError(f"unknown manual home axis: {axis}") from exc
    try:
        return MODEL_TRACES[mode]
    except KeyError as exc:
        raise ValueError(f"unknown OEM mode: {mode}") from exc


def operation_names(mode: str, axis: str | None = None) -> list[str]:
    return [step.name for step in trace_for(mode, axis=axis)]


def source_matrix() -> dict[str, Any]:
    """Serialize the source-to-target matrix for docs/tests."""

    return {
        "truth_level": "source_model_only_no_motion_no_usb",
        "axis_to_board": AXIS_TO_BOARD,
        "board_primitives": {name: asdict(anchor) for name, anchor in BOARD_PRIMITIVE_ANCHORS.items()},
        "traces": {name: [step.to_dict() for step in trace] for name, trace in MODEL_TRACES.items()},
        "manual_home": {axis: [step.to_dict() for step in trace] for axis, trace in MANUAL_HOME_TRACE.items()},
        "routes": [mapping.to_dict() for mapping in ROUTE_MAPPINGS],
        "raw_fastapi_route_table": list(RAW_FASTAPI_ROUTE_TABLE),
        "live_target_mappings": [mapping.to_dict() for mapping in LIVE_TARGET_MAPPINGS],
        "deviations_to_live_linux": (
            "Current live Linux homing is a guarded reconstruction with safety/workaround paths, not a clean line-by-line port.",
            "Z startup may use a live GAP10/controller-zero reference workaround; source model keeps that separate from OEM ClassHeadBoard.queryHome/GAP9 provenance.",
            "BMS /api/bioxp/* is a proxy/linkage layer and may not expose every raw FastAPI route.",
        ),
    }
