
"""Fresh no-USB/no-motion OEM homing source specs.

This module intentionally does not import BioXpTester, pyusb, CAN, camera, or
old Linux homing helpers. It is source-model only.
"""
from __future__ import annotations

from .oem_parity_types import OemProgramSpec, OemProgramStep, OemSourceAnchor

NO_USB_IMPORTS = True
CCI = "BioXPControlLib/ClassControlInterface.cs"
CL = "BioXPControlLib/ControlLib.cs"


CCI_SHA256 = "86093e5270c82ea2e45cb4de449076372ca79d9485ba6de9565d5eb255811e6e"
CL_SHA256 = "f69b3529dcb9723c705ac55ecb3f035010cc294d3891de096c165bb20116f6c2"


def A(file: str, symbol: str, lines: str) -> OemSourceAnchor:
    return OemSourceAnchor(
        file=file,
        symbol=symbol,
        lines=lines,
        sha256=CCI_SHA256 if symbol.startswith("ClassControlInterface") else CL_SHA256,
    )


def S(
    step_id: str,
    symbol: str,
    lines: str,
    operation: str,
    *,
    axis=None,
    board: str | None = None,
    motor: int | None = None,
    params=None,
    wait_ms: int | None = None,
    branch_condition: str | None = None,
    safety_deviations=(),
    blockers_note=None,
):
    return OemProgramStep(
        step_id=step_id,
        source=A(CCI if symbol.startswith("ClassControlInterface") else CL, symbol, lines),
        operation=operation,
        axis=axis,
        board=board,
        motor=motor,
        params=params or {},
        wait_ms=wait_ms,
        branch_condition=branch_condition,
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
    S("z.axisSearchHome", "ClassControlInterface.initializeMotors", "3350-3353", "axisSearchHome", axis="z", board="head", motor=1, params={"speed": 1791}, branch_condition="board_present:z"),
    S("g.setGripperCurrent.before_clear", "ClassControlInterface.initializeMotors", "3354", "setGripperCurrent", axis="g", board="head", motor=2, params={"run_current": 31}, safety_deviations=(G_IDLE_DEVIATION,)),
    S("g.clear.moveSteps", "ClassControlInterface.initializeMotors", "3355", "moveSteps", axis="g", board="head", motor=2, params={"steps": 10000, "waitforstop": True}, safety_deviations=(G_IDLE_DEVIATION,)),
    S("g.axisSearchHome", "ClassControlInterface.initializeMotors", "3356-3365", "axisSearchHome", axis="g", board="head", motor=2, params={"speed": "600 if GripperVersion==0 else 200"}, branch_condition="board_present:g", safety_deviations=(G_IDLE_DEVIATION,)),
    S("x.axisSearchHome", "ClassControlInterface.initializeMotors", "3367-3369", "axisSearchHome", axis="x", board="head", motor=0, params={"speed": 250}, branch_condition="board_present:x"),
    S("x.sleep.after_home", "ClassControlInterface.initializeMotors", "3370", "sleep", wait_ms=20, branch_condition="board_present:x"),
    S("x.setHome", "ClassControlInterface.initializeMotors", "3371", "setHome", axis="x", board="head", motor=0, branch_condition="board_present:x"),
    S("x.setSpeed.restore", "ClassControlInterface.initializeMotors", "3372", "setSpeed", axis="x", board="head", motor=0, params={"speed": 1700}, branch_condition="board_present:x"),
    S("x.sleep.after_speed", "ClassControlInterface.initializeMotors", "3373", "sleep", wait_ms=40, branch_condition="board_present:x"),
    S("x.park_6000", "ClassControlInterface.initializeMotors", "3374", "moveX", axis="x", board="head", motor=0, params={"position": 6000}, branch_condition="board_present:x"),
    S("y.axisSearchHome", "ClassControlInterface.initializeMotors", "3376-3379", "axisSearchHome", axis="y", board="head", motor=0, params={"speed": 250}, branch_condition="board_present:y"),
    S("door.doorSearchHome", "ClassControlInterface.initializeMotors", "3380-3383", "doorSearchHome", axis="door", board="thermal", motor=0, params={"speed": "TC_DOOR_VELOCITY", "stallguard": "TCDoorStallGuardThreshold"}, branch_condition="board_present:door"),
    S("door.open_after_failed_close", "ClassControlInterface.initializeMotors", "3384-3386", "openThermalDoor", axis="door", branch_condition="SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated"),
    S("door.throw_after_failed_close", "ClassControlInterface.initializeMotors", "3387", "throw", branch_condition="SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated", blockers_note="Cannot close thermal cycler door!"),
    S("y.setHome.final", "ClassControlInterface.initializeMotors", "3389-3392", "setHome", axis="y", board="head", motor=0, branch_condition="board_present:y"),
    S("ui.zero_calibrated_positions", "ClassControlInterface.initializeMotors", "3393-3413", "setUiPositionText", params={"x": "0", "y": "0", "z": "0", "z_write_count": 2}, branch_condition="Calibrated"),
    S("chiller.setCoolRate.OC", "ClassControlInterface.initializeMotors", "3414", "setChillerCoolRate", params={"chiller": "OC"}),
    S("chiller.setCoolRate.RC", "ClassControlInterface.initializeMotors", "3415", "setChillerCoolRate", params={"chiller": "RC"}),
    S("status.initialized", "ClassControlInterface.initializeMotors", "3416", "setStatus", params={"system_status": 1, "ready": True}),
    S("g.restore_current.version1", "ClassControlInterface.initializeMotors", "3417-3420", "setGripperCurrent", axis="g", board="head", motor=2, params={"run_current": 10}, branch_condition="GripperVersion==1", safety_deviations=(G_IDLE_DEVIATION,)),
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
    "initialize_motion": OemProgramSpec(
        "initialize_motion",
        "ControlLib.initializeMotion",
        "app_level_initialize_motion",
        False,
        (
            S("initializeMotion.stop_scripts", "ControlLib.initializeMotion", "8799", "setFlag", params={"m_stopScripts": True}),
            S("initializeMotion.clear_forceabort", "ControlLib.initializeMotion", "8800", "setFlag", params={"forceabort": False}),
            S("initializeMotion.initializeMotors", "ControlLib.initializeMotion", "8803", "initializeMotors"),
            S("initializeMotion.thermal_door_closed", "ControlLib.initializeMotion", "8804", "setMachineStatus", params={"ThermalDoorOpen": False}),
            S("initializeMotion.queryTipStatus.initial", "ControlLib.initializeMotion", "8805", "queryTipStatus", params={"index": -1}),
            S("initializeMotion.sleep.after_tip_query", "ControlLib.initializeMotion", "8806", "sleep", wait_ms=500),
            S("initializeMotion.openThermalDoor.tip_exists", "ControlLib.initializeMotion", "8807-8809", "openThermalDoor", axis="door", branch_condition="TipExist"),
            S("initializeMotion.thermal_door_open.tip_exists", "ControlLib.initializeMotion", "8810", "setMachineStatus", params={"ThermalDoorOpen": True}, branch_condition="TipExist"),
            S("initializeMotion.tip_loaded.tip_exists", "ControlLib.initializeMotion", "8811", "setMachineStatus", params={"TipLoaded": True}, branch_condition="TipExist"),
            S("initializeMotion.scriptmoveTo.tip_exists", "ControlLib.initializeMotion", "8812", "scriptmoveTo", params={"from_location": 28, "from_well": 0, "to_location": 6, "column": 0, "row": 0}, branch_condition="TipExist"),
            S("initializeMotion.updateLocation.tip_exists", "ControlLib.initializeMotion", "8813", "updateLocation", params={"location": 6, "well": 0}, branch_condition="TipExist"),
            S("initializeMotion.ejectAllTips.tip_exists", "ControlLib.initializeMotion", "8814", "ejectAllTips", params={"arg0": False, "arg1": True}, branch_condition="TipExist"),
            S("initializeMotion.moveZ.tip_exists", "ControlLib.initializeMotion", "8815", "moveZ", axis="z", params={"position": 80000}, branch_condition="TipExist"),
            S("initializeMotion.moveX.tip_exists", "ControlLib.initializeMotion", "8816", "moveX", axis="x", params={"position": 79000}, branch_condition="TipExist"),
            S("initializeMotion.queryTipStatus.after_eject", "ControlLib.initializeMotion", "8817", "queryTipStatus", params={"index": -1}, branch_condition="TipExist"),
            S("initializeMotion.sleep.after_eject_query", "ControlLib.initializeMotion", "8818", "sleep", wait_ms=100, branch_condition="TipExist"),
            S("initializeMotion.pause_scripts.eject_failed", "ControlLib.initializeMotion", "8819-8821", "setFlag", params={"m_pauseScripts": True}, branch_condition="TipExist && TipExistAfterEject"),
            S("initializeMotion.error_event.eject_failed", "ControlLib.initializeMotion", "8822-8825", "emitError", params={"message": "Eject tip failed"}, branch_condition="TipExist && TipExistAfterEject && errorEvent!=null"),
            S("initializeMotion.throw.eject_failed", "ControlLib.initializeMotion", "8825", "throw", branch_condition="TipExist && TipExistAfterEject && errorEvent!=null", blockers_note="Eject tip failed"),
            S("initializeMotion.return.eject_failed_without_handler", "ControlLib.initializeMotion", "8827", "return", branch_condition="TipExist && TipExistAfterEject && errorEvent==null"),
            S("initializeMotion.tip_dirty_false", "ControlLib.initializeMotion", "8829", "setMachineStatus", params={"TipDirty": False}, branch_condition="TipExist && !TipExistAfterEject"),
            S("initializeMotion.tip_loaded_false.after_eject", "ControlLib.initializeMotion", "8830", "setMachineStatus", params={"TipLoaded": False}, branch_condition="TipExist && !TipExistAfterEject"),
            S("initializeMotion.sleep.before_initiate_group", "ControlLib.initializeMotion", "8831", "sleep", wait_ms=2, branch_condition="TipExist && !TipExistAfterEject"),
            S("initializeMotion.initiateGroup.initial", "ControlLib.initializeMotion", "8832", "initiateGroup", branch_condition="TipExist && !TipExistAfterEject"),
            S("initializeMotion.checkedPipetteStatus.initial", "ControlLib.initializeMotion", "8833", "checkedPipetteStatus", branch_condition="TipExist && !TipExistAfterEject"),
            S("initializeMotion.initiateGroup.retry", "ControlLib.initializeMotion", "8834-8835", "initiateGroup", branch_condition="TipExist && !TipExistAfterEject && PipetteStatusInitialFailed"),
            S("initializeMotion.checkedPipetteStatus.retry", "ControlLib.initializeMotion", "8836", "checkedPipetteStatus", branch_condition="TipExist && !TipExistAfterEject && PipetteStatusInitialFailed"),
            S("initializeMotion.error_event.eject_failed_after_retry", "ControlLib.initializeMotion", "8837-8839", "emitError", params={"message": "Eject tip failed"}, branch_condition="TipExist && !TipExistAfterEject && PipetteStatusInitialFailed && PipetteStatusRetryFailed"),
            S("initializeMotion.throw.eject_failed_after_retry", "ControlLib.initializeMotion", "8839", "throw", branch_condition="TipExist && !TipExistAfterEject && PipetteStatusInitialFailed && PipetteStatusRetryFailed", blockers_note="Eject tip failed"),
            S("initializeMotion.tip_loaded_false.no_tip", "ControlLib.initializeMotion", "8843-8846", "setMachineStatus", params={"TipLoaded": False}, branch_condition="!TipExist"),
            S("initializeMotion.catch.error_event", "ControlLib.initializeMotion", "8848-8852", "emitError", params={"message": "exception.Message"}, branch_condition="Exception && errorEvent!=null"),
            S("initializeMotion.catch.rethrow", "ControlLib.initializeMotion", "8853", "rethrow", branch_condition="Exception && errorEvent!=null", blockers_note="initializeMotion_exception_rethrown"),
            S("initializeMotion.catch.swallow_without_handler", "ControlLib.initializeMotion", "8848-8855", "swallowException", branch_condition="Exception && errorEvent==null"),
        ),
        G_ARTIFACT_FIELDS,
        ("pipette_cleanup_not_ported", "vision_inspection_not_ported"),
    ),
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
