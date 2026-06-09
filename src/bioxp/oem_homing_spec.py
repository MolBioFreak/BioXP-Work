
"""Fresh no-USB/no-motion OEM homing source specs.

This module intentionally does not import BioXpTester, pyusb, CAN, camera, or
old Linux homing helpers. It is source-model only.
"""
from __future__ import annotations

from .oem_parity_types import OemProgramSpec, OemProgramStep, OemSourceAnchor

NO_USB_IMPORTS = True
CCI = "BioXPControlLib/ClassControlInterface.cs"
CL = "BioXPControlLib/ControlLib.cs"


def A(file: str, symbol: str, lines: str) -> OemSourceAnchor:
    return OemSourceAnchor(file=file, symbol=symbol, lines=lines)


def S(step_id: str, symbol: str, lines: str, operation: str, *, axis=None, params=None, safety_deviations=(), blockers_note=None):
    return OemProgramStep(
        step_id=step_id,
        source=A(CCI if symbol.startswith("ClassControlInterface") else CL, symbol, lines),
        operation=operation,
        axis=axis,
        params=params or {},
        safety_deviations=tuple(safety_deviations),
        failure_modes=(blockers_note,) if blockers_note else (),
    )

G_IDLE_DEVIATION = "linux_safety_deviation_requires_G_CURRENT_IDLE_SAFE_when_speed_zero"
COMMON_ARTIFACT_FIELDS = (
    "program", "mode", "source_mode", "opened_usb", "physical_motion", "steps_planned", "steps_executed", "blockers"
)
G_ARTIFACT_FIELDS = COMMON_ARTIFACT_FIELDS + ("g_current_invariant", "g_current_before", "g_current_after", "g_speed_after", "g_current_classification")

init_without_motion_steps = (
    S("waitForBoard", "ClassControlInterface.initializeMotorsWithoutMotion", "3183", "waitForBoard"),
    S("turnOffHeater", "ClassControlInterface.initializeMotorsWithoutMotion", "3184", "turnOffHeater"),
    S("setChillerPWM", "ClassControlInterface.initializeMotorsWithoutMotion", "3185", "setChillerPWM"),
    S("x.setup.speed_acc", "ClassControlInterface.initializeMotorsWithoutMotion", "3187-3189", "setSpeedAcc", axis="x", params={"speed": 1700, "acc": 350}),
    S("x.setup.current", "ClassControlInterface.initializeMotorsWithoutMotion", "3190-3192", "setMaxCurrent", axis="x", params={"run_current": 31}),
    S("x.setup.stallguard", "ClassControlInterface.initializeMotorsWithoutMotion", "3193-3194", "setStallGuardThreshold", axis="x", params={"threshold": 16}),
    S("y.setup.speed_acc", "ClassControlInterface.initializeMotorsWithoutMotion", "3196-3198", "setSpeedAcc", axis="y", params={"speed": 1800, "acc": 400}),
    S("y.setup.current", "ClassControlInterface.initializeMotorsWithoutMotion", "3199-3201", "setMaxCurrent", axis="y", params={"run_current": 31}),
    S("y.setup.stallguard_disable_right", "ClassControlInterface.initializeMotorsWithoutMotion", "3202-3205", "setStallGuardThreshold+disableRightSwitch", axis="y", params={"threshold": 16}),
    S("z.setup.speed_acc", "ClassControlInterface.initializeMotorsWithoutMotion", "3207-3209", "setSpeedAcc", axis="z", params={"speed": 1791, "acc": 576}),
    S("z.setup.current_stallguard", "ClassControlInterface.initializeMotorsWithoutMotion", "3210-3215", "setMaxCurrent/readMaxCurrent/setStallGuardThreshold", axis="z", params={"run_current": "Z_MOTOR_MAX_CURRENT_UP"}),
    S("g.setup.versioned", "ClassControlInterface.initializeMotorsWithoutMotion", "3217-3242", "setSpeedAcc/current/stallguard/RDIV/PDIV", axis="g", params={"run_current": "31 or 10 by GripperVersion"}, safety_deviations=(G_IDLE_DEVIATION,)),
    S("door.setup", "ClassControlInterface.initializeMotorsWithoutMotion", "3244-3255", "setSpeedAcc/current/stallguard/disable switches", axis="door"),
    S("setChillerCoolRate.OC", "ClassControlInterface.initializeMotorsWithoutMotion", "3257", "setChillerCoolRate", params={"chiller": "OC"}),
    S("setChillerCoolRate.RC", "ClassControlInterface.initializeMotorsWithoutMotion", "3258", "setChillerCoolRate", params={"chiller": "RC"}),
    S("setTCHeatCoolRate", "ClassControlInterface.initializeMotorsWithoutMotion", "3259-3263", "setTCHeatRate+setTCCoolRate"),
    S("setColor.white", "ClassControlInterface.initializeMotorsWithoutMotion", "3264", "setColor", params={"rgb": [255,255,255]}),
)

initialize_motors_steps = (
    S("z.axisSearchHome", "ClassControlInterface.initializeMotors", "3350-3353", "axisSearchHome", axis="z", params={"speed": 1791}),
    S("g.setMaxCurrent.before_clear", "ClassControlInterface.initializeMotors", "3354", "setMaxCurrent", axis="g", params={"run_current": 31}, safety_deviations=(G_IDLE_DEVIATION,)),
    S("g.clear.moveSteps", "ClassControlInterface.initializeMotors", "3355", "moveSteps", axis="g", params={"steps": 10000, "waitforstop": True}, safety_deviations=(G_IDLE_DEVIATION,)),
    S("g.axisSearchHome", "ClassControlInterface.initializeMotors", "3356-3365", "axisSearchHome", axis="g", params={"speed": "600 if GripperVersion==0 else 200"}, safety_deviations=(G_IDLE_DEVIATION,)),
    S("x.axisSearchHome", "ClassControlInterface.initializeMotors", "3367-3369", "axisSearchHome", axis="x", params={"speed": 250}),
    S("x.setHome", "ClassControlInterface.initializeMotors", "3370-3371", "setHome", axis="x"),
    S("x.setSpeed.restore", "ClassControlInterface.initializeMotors", "3372-3373", "setSpeed", axis="x", params={"speed": 1700}),
    S("x.park_6000", "ClassControlInterface.initializeMotors", "3374-3375", "moveX", axis="x", params={"position": 6000}),
    S("y.axisSearchHome", "ClassControlInterface.initializeMotors", "3376-3379", "axisSearchHome", axis="y", params={"speed": 250}),
    S("door.doorSearchHome", "ClassControlInterface.initializeMotors", "3380-3383", "doorSearchHome", axis="door"),
    S("y.setHome.final", "ClassControlInterface.initializeMotors", "3389-3392", "setHome", axis="y"),
    S("g.restore_current.version1", "ClassControlInterface.initializeMotors", "3417-3420", "setMaxCurrent", axis="g", params={"run_current": 10, "condition": "GripperVersion==1"}, safety_deviations=(G_IDLE_DEVIATION,)),
)


def manual(axis: str, speed):
    return OemProgramSpec(
        name=f"manual_home_{axis}", oem_symbol=f"ClassControlInterface.manual_{axis}", source_mode="manual_button_goHome", live_allowed_default=False,
        steps=(S(f"manual.{axis}.goHome", f"ClassControlInterface.btnHome{axis.upper()}_Click", "manual", "goHome", axis=axis, params={"rehome": True, "speed": speed}),),
        required_artifact_fields=COMMON_ARTIFACT_FIELDS,
        blockers=("manual_home_live_requires_switch_predicate_matrix",),
    )

PROGRAMS = {
    "initialize_motors_without_motion": OemProgramSpec("initialize_motors_without_motion", "ClassControlInterface.initializeMotorsWithoutMotion", "no_motion_hardware_setup", False, init_without_motion_steps, G_ARTIFACT_FIELDS, ("hardware_mutating_no_motion_requires_ack_before_live",)),
    "initialize_motors": OemProgramSpec("initialize_motors", "ClassControlInterface.initializeMotors", "physical_startup_homing", False, initialize_motors_steps, G_ARTIFACT_FIELDS, ("monolithic_live_homing_blocked_until_stepwise_proof",)),
    "manual_home_x": manual("x", 500),
    "manual_home_y": manual("y", 500),
    "manual_home_z": manual("z", 1791),
    "manual_home_g": OemProgramSpec("manual_home_g", "ClassControlInterface.btnGripperHome_Click", "manual_button_goHome", False, (S("manual.g.current_high", "ClassControlInterface.btnGripperHome_Click", "2046-2054", "setMaxCurrent", axis="g", params={"run_current":31}, safety_deviations=(G_IDLE_DEVIATION,)), S("manual.g.goHome", "ClassControlInterface.btnGripperHome_Click", "2059-2064", "goHome", axis="g", params={"rehome":True}), S("manual.g.restore_current", "ClassControlInterface.btnGripperHome_Click", "2065-2069", "setMaxCurrent", axis="g", params={"run_current":10}, safety_deviations=(G_IDLE_DEVIATION,))), G_ARTIFACT_FIELDS, ("manual_g_live_requires_operator_physical_proof",)),
    "manual_home_door": OemProgramSpec("manual_home_door", "ClassControlInterface.btnDHome_Click", "manual_door_home", False, (S("manual.door.doorSearchHome", "ClassControlInterface.btnDHome_Click", "1224-1246", "doorSearchHome", axis="door"),), COMMON_ARTIFACT_FIELDS, ("door_predicate_matrix_required",)),
    "home_axis": OemProgramSpec("home_axis", "ClassControlInterface.HomeAxis", "generic_home_axis", False, (S("HomeAxis.x.axisSearchHome", "ClassControlInterface.HomeAxis", "4997-5052", "axisSearchHome", axis="x"), S("HomeAxis.y.axisSearchHome", "ClassControlInterface.HomeAxis", "4997-5052", "axisSearchHome", axis="y"), S("HomeAxis.z.current_axisSearchHome", "ClassControlInterface.HomeAxis", "4997-5052", "axisSearchHome", axis="z"), S("HomeAxis.g.current_stall_axisSearchHome", "ClassControlInterface.HomeAxis", "4997-5052", "axisSearchHome", axis="g", safety_deviations=(G_IDLE_DEVIATION,)), S("HomeAxis.door.preclear_doorSearchHome", "ClassControlInterface.HomeAxis", "4997-5052", "doorSearchHome", axis="door")), G_ARTIFACT_FIELDS, ("axis_parameterized_live_route_blocked",)),
    "home_xy": OemProgramSpec("home_xy", "ClassControlInterface.HomeXY", "parallel_home_xy", False, (S("HomeXY.set_xy_speedacc_200", "ClassControlInterface.HomeXY", "5054-5061", "setSpeedAcc"), S("HomeXY.parallel_x_goHome", "ClassControlInterface.HomeXY", "5062", "goHome", axis="x"), S("HomeXY.parallel_y_goHome", "ClassControlInterface.HomeXY", "5063", "goHome", axis="y"), S("HomeXY.restore_xy_speedacc", "ClassControlInterface.HomeXY", "5064-5070", "setSpeedAcc")), COMMON_ARTIFACT_FIELDS, ("lost_step_result_semantics_required",)),
    "move_z_home": OemProgramSpec("move_z_home", "ClassControlInterface.MoveZHome", "distinct_z_home", False, (S("MoveZHome.set_z_current", "ClassControlInterface.MoveZHome", "4623-4632", "setMaxCurrent", axis="z"), S("MoveZHome.goHome_z", "ClassControlInterface.MoveZHome", "4623-4632", "goHome", axis="z", params={"speed":1791})), COMMON_ARTIFACT_FIELDS, ("z_live_predicate_conflict_unresolved",)),
    "home_gz": OemProgramSpec("home_gz", "ClassControlInterface.homeGZ", "caught_plate_gz_recovery", False, (S("homeGZ.pseudo_z_home", "ClassControlInterface.homeGZ", "4657-4687", "pseudoZHome", axis="z"), S("homeGZ.g_home", "ClassControlInterface.homeGZ", "4657-4687", "goHome", axis="g", safety_deviations=(G_IDLE_DEVIATION,)), S("homeGZ.caught_plate_recovery", "ClassControlInterface.homeGZ", "4657-4687", "caughtPlateRecovery")), G_ARTIFACT_FIELDS, ("caught_plate_recovery_not_live_proven",)),
    "door_search_home": OemProgramSpec("door_search_home", "ClassControlInterface.doorSearchHome callers", "door_search_home", False, (S("doorSearchHome.search", "ClassControlInterface.initializeMotors", "3380-3387", "doorSearchHome", axis="door"),), COMMON_ARTIFACT_FIELDS, ("door_closed_failure_branch_required",)),
    "rehome": OemProgramSpec("rehome", "ControlLib.rehome", "rehome_wrapper", False, (S("rehome.save_door_state", "ControlLib.rehome", "8784-8788", "saveDoorState"), S("rehome.initializeMotors", "ControlLib.rehome", "8789", "initializeMotors"), S("rehome.restore_resume", "ControlLib.rehome", "8790-8795", "restoreDoorAndResumeTemperature")), G_ARTIFACT_FIELDS, ("thermal_door_restore_not_live_proven",)),
    "initialize_motion": OemProgramSpec("initialize_motion", "ControlLib.initializeMotion", "app_level_initialize_motion", False, (S("initializeMotion.flags", "ControlLib.initializeMotion", "8797-8802", "setFlags"), S("initializeMotion.initializeMotors", "ControlLib.initializeMotion", "8803", "initializeMotors"), S("initializeMotion.tip_pipette_cleanup", "ControlLib.initializeMotion", "8805-8841", "tipPipetteCleanup")), G_ARTIFACT_FIELDS, ("pipette_cleanup_not_ported", "vision_inspection_not_ported")),
}


def program_names() -> list[str]:
    return sorted(PROGRAMS)


def get_program(name: str) -> OemProgramSpec:
    try:
        return PROGRAMS[name]
    except KeyError as exc:
        raise ValueError(f"unknown OEM homing program: {name}") from exc


def all_programs() -> tuple[OemProgramSpec, ...]:
    return tuple(PROGRAMS[name] for name in program_names())
