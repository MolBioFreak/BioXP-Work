from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from pathlib import Path

from .oem_config import find_oem_machine_config_bundle, oem_thermal_door_defaults, parse_oem_machine_config_bundle


@dataclass(frozen=True)
class OemSourceAnchor:
    name: str
    file: str
    lines: str
    summary: str

    def to_dict(self) -> dict[str, str]:
        return asdict(self)


@dataclass(frozen=True)
class OemInitPhase:
    name: str
    source_command: str
    source_anchor: OemSourceAnchor
    physical_motion_possible: bool
    linux_status: str
    notes: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        row = asdict(self)
        row["source_anchor"] = self.source_anchor.to_dict()
        row["notes"] = list(self.notes)
        return row


SOURCE_ANCHORS: dict[str, OemSourceAnchor] = {
    "genbot_initialize_system": OemSourceAnchor(
        name="GenBotApp.initializeSystem",
        file="decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs",
        lines="1128-1165,2040-2120",
        summary="GenBotApp startup and command-worker entrypoints call initialCheck/initializeMotion and update UI/runtime status.",
    ),
    "prepare_to_run_job": OemSourceAnchor(
        name="GenBotApp.PrepareToRunJob",
        file="decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs",
        lines="1240-1288,1370-1425",
        summary="Job preparation/validation remains separate from motion initialization and parks gantry after job prep.",
    ),
    "control_initialize_motion": OemSourceAnchor(
        name="ControlLib.initializeMotion",
        file="decompiled_src/BioXPControlLib/ControlLib.cs",
        lines="8790-8845",
        summary="Top-level motion initializer wraps ControlInterface initialization and cleanup/park work.",
    ),
    "control_initialize_motors": OemSourceAnchor(
        name="ClassControlInterface.initializeMotors",
        file="decompiled_src/BioXPControlLib/ClassControlInterface.cs",
        lines="3180-3410",
        summary="Source-shaped motor initialization/homing order and parameter setup.",
    ),
    "homexy": OemSourceAnchor(
        name="ClassControlInterface.HomeXY",
        file="decompiled_src/BioXPControlLib/ClassControlInterface.cs",
        lines="5050-5070",
        summary="OEM HomeXY sets X/Y speed+acc to 200, runs X/Y home concurrently, then restores profiles.",
    ),
    "thermal_door": OemSourceAnchor(
        name="Thermal door home/open/close",
        file="decompiled_src/BioXPControlLib/ClassControlInterface.cs",
        lines="1218-1240,1960-2018",
        summary="Door home uses doorSearchHome; open/close are absolute moves with closed/open predicates.",
    ),
    "z_home": OemSourceAnchor(
        name="MoveZHome",
        file="decompiled_src/BioXPControlLib/ClassControlInterface.cs",
        lines="4620-4665",
        summary="Z reference establishment is first in initializeMotors and must establish safe top/home before XY travel.",
    ),
    "board_go_home": OemSourceAnchor(
        name="Board goHome / doorSearchHome",
        file="decompiled_src_can/ClassCanLib/*.cs",
        lines="ClassBaseBoard.cs:140-170; ClassHeadBoard.cs:50-80; ClassThermalBoard.cs:110-130",
        summary="CAN board homing primitives and thermal door search primitive.",
    ),
    "machine_settings": OemSourceAnchor(
        name="ClassBioXPSettings machine config",
        file="decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs",
        lines="220-275,3138-3160,3828-3855",
        summary="Machine-specific XML settings override compiled defaults for door, gripper and axis limits.",
    ),
    "gripper": OemSourceAnchor(
        name="MotorGrip home/confirm/machine positions",
        file="decompiled_src/BioXPControlLib/ClassControlInterface.cs",
        lines="56-60,2028-2055,2736-2745,3354-3365",
        summary="MotorGrip is head-board axis 2; home confirmation is queryHome(MotorGrip) OR getG()<50; initializeMotors clears +10000 then homes at version-specific speed.",
    ),
}

OEM_INIT_PHASES: tuple[OemInitPhase, ...] = (
    OemInitPhase("accepted", "GenBotApp command worker", SOURCE_ANCHORS["genbot_initialize_system"], False, "modeled"),
    OemInitPhase("initial_check", "m_control.initialCheck", SOURCE_ANCHORS["genbot_initialize_system"], False, "existing_route"),
    OemInitPhase("interlock_prepare", "Linux safety gate for OEM prep", SOURCE_ANCHORS["control_initialize_motors"], False, "existing_route"),
    OemInitPhase("initialize_without_motion", "initializeMotorsWithoutMotion", SOURCE_ANCHORS["control_initialize_motors"], False, "existing_route"),
    OemInitPhase("door_state_capture", "ControlLib.rehome door state save", SOURCE_ANCHORS["control_initialize_motion"], False, "phase4"),
    OemInitPhase("z_reference", "MoveZHome", SOURCE_ANCHORS["z_home"], True, "phase2"),
    OemInitPhase("g_reference", "Gripper home/init", SOURCE_ANCHORS["control_initialize_motors"], True, "phase5"),
    OemInitPhase("home_xy", "HomeXY", SOURCE_ANCHORS["homexy"], True, "phase3"),
    OemInitPhase("door_home_or_restore", "doorSearchHome/open/close", SOURCE_ANCHORS["thermal_door"], True, "phase4"),
    OemInitPhase("tip_pipette_cleanup_or_unsupported", "ControlLib.initializeMotion cleanup", SOURCE_ANCHORS["control_initialize_motion"], True, "deferred"),
    OemInitPhase("park_or_ready_position", "PrepareToRunJob parkGantry", SOURCE_ANCHORS["prepare_to_run_job"], True, "deferred"),
    OemInitPhase("final_readiness", "ClassStatusLog/runtime ready", SOURCE_ANCHORS["genbot_initialize_system"], False, "phase6"),
)


def _axis_limit_from_bundle(bundle: dict[str, Any], axis: str) -> dict[str, Any] | None:
    limits = (((bundle.get("config") or {}).get("axis_limits") or {}) if isinstance(bundle, dict) else {})
    row = limits.get(axis.lower()) if isinstance(limits, dict) else None
    return dict(row) if isinstance(row, dict) else None


def _config_attr(bundle: dict[str, Any], key: str) -> Any:
    """Read an OEM config value from parsed config.xml sections.

    ClassBioXPSettings field names often carry an m_ prefix in XML
    CalibrationFactors/Offsets while Linux-facing names omit it.
    """
    parsed = bundle.get("config") or {} if isinstance(bundle, dict) else {}
    candidates = [key]
    if not key.startswith("m_"):
        candidates.append("m_" + key)
    if key.startswith("TCDoor"):
        candidates.append("m_" + key)
    if key.startswith("TC_DOOR"):
        candidates.append("m_" + key)
    for section_name in ("config", "calibration", "offsets"):
        section = parsed.get(section_name) if isinstance(parsed, dict) else None
        if not isinstance(section, dict):
            continue
        for candidate in candidates:
            if candidate in section:
                return section[candidate]
    return None


def _default_repo_machine_config_bundle() -> dict[str, Any] | None:
    # Repo-local copy of the original SSD AppData config. This is read-only and
    # used when env vars are not set on the robot/runtime.
    root = Path.cwd() / "config" / "oem" / "original_ssd_appdata_20260610"
    if root.exists():
        return parse_oem_machine_config_bundle(root)
    return None


def build_machine_calibration_manifest(bundle: dict[str, Any] | None = None, *, serial_number: int | str | None = None) -> dict[str, Any]:
    """Return the OEM machine-calibration values the initialization controller must consume.

    Source defaults are reported only as fallback; when the extracted SSD config exists
    its values must be preferred and tagged as machine config.
    """
    if bundle is None:
        bundle = find_oem_machine_config_bundle()
        if not bool(isinstance(bundle, dict) and bundle.get("ok")):
            bundle = _default_repo_machine_config_bundle() or bundle
    defaults = oem_thermal_door_defaults(serial_number)
    ok = bool(isinstance(bundle, dict) and bundle.get("ok"))
    config_path = None
    if ok:
        files = bundle.get("files") or {}
        cfg_file = files.get("config_xml") if isinstance(files, dict) else None
        if isinstance(cfg_file, dict):
            config_path = cfg_file.get("path")
    def value(name: str, default: Any) -> dict[str, Any]:
        if ok:
            v = _config_attr(bundle, name)
            if v is not None:
                return {"value": v, "source": "original_ssd_machine_config", "fallback": False, "config_path": config_path}
        return {"value": default, "source": "oem_source_default", "fallback": True, "config_path": config_path}

    manifest = {
        "ok": ok,
        "source_anchor": SOURCE_ANCHORS["machine_settings"].to_dict(),
        "config_path": config_path,
        "machine_calibrated": bool(bundle.get("machine_calibrated")) if isinstance(bundle, dict) else False,
        "thermal_door": {
            "TCDoorOpen": value("TCDoorOpen", defaults["TCDoorOpen"]),
            "TC_DOOR_VELOCITY": value("TC_DOOR_VELOCITY", defaults["TC_DOOR_VELOCITY"]),
            "TC_DOOR_ACCELERATION": value("TC_DOOR_ACCELERATION", defaults["TC_DOOR_ACCELERATION"]),
            "TC_DOOR_MAX_CURRENT": value("TC_DOOR_MAX_CURRENT", defaults["TC_DOOR_MAX_CURRENT"]),
            "TCDoorStallGuardThreshold": value("TCDoorStallGuardThreshold", defaults["TCDoorStallGuardThreshold"]),
        },
        "gripper": {
            "originOffsetG": value("originOffsetG", None),
            "GripperClosePOS": value("GripperClosePOS", None),
            "GripperOpenPOS": value("GripperOpenPOS", None),
            "GripperOpenWide": value("GripperOpenWide", None),
        },
        "axis_limits": {},
        "phases": [phase.to_dict() for phase in OEM_INIT_PHASES],
    }
    for axis in ("x", "y", "z", "g"):
        row = _axis_limit_from_bundle(bundle, axis) if ok else None
        manifest["axis_limits"][axis] = {
            "value": row,
            "source": "original_ssd_machine_config" if row is not None else "oem_source_default_or_unavailable",
            "fallback": row is None,
            "config_path": config_path,
        }
    return manifest


def oem_initialization_phase_catalog() -> list[dict[str, Any]]:
    return [phase.to_dict() for phase in OEM_INIT_PHASES]



def classify_thermal_door_state(status: dict[str, Any]) -> dict[str, Any]:
    """Classify thermal-door state using OEM predicates.

    OEM predicate anchors:
    - closed: queryHome(ThermalDoor) / tcDoorClosed
    - opened: queryRightSensor(ThermalDoor) / tcDoorOpened
    """
    predicates = status.get("oem_predicates") if isinstance(status, dict) else None
    closed = None
    opened = None
    if isinstance(predicates, dict):
        closed = predicates.get("tcDoorClosed")
        opened = predicates.get("tcDoorOpened")
    if closed is None:
        closed = status.get("closed") if isinstance(status, dict) else None
    if opened is None:
        opened = status.get("opened") if isinstance(status, dict) else None
    if closed is True and opened is False:
        state = "closed"
        safe = True
    elif opened is True and closed is False:
        state = "open"
        safe = True
    else:
        state = "ambiguous"
        safe = False
    return {
        "state": state,
        "safe": safe,
        "tcDoorClosed": closed,
        "tcDoorOpened": opened,
        "source_anchor": SOURCE_ANCHORS["thermal_door"].to_dict(),
        "closed_source": "queryHome(ThermalDoor)",
        "opened_source": "queryRightSensor(ThermalDoor)",
    }


def build_thermal_door_state_restore_plan(before: dict[str, Any], *, restore_requested: bool = True) -> dict[str, Any]:
    """Build a source-explicit restore policy for ControlLib.rehome door handling.

    Linux currently has source-equivalent open/close/home primitives, but the full
    ControlLib door-state setter/restore wrapper is not implemented as an automatic
    hidden side effect. The safe controller policy is therefore explicit:
    - if restore is not requested, leave/prove closed at end;
    - if restore is requested for an originally open door, require an explicit
      post-init open action rather than silently reopening during homing;
    - ambiguous before-state fails closed.
    """
    classified = classify_thermal_door_state(before)
    state = classified["state"]
    if state == "ambiguous":
        action = "fail_closed"
        supported = False
        reason = "pre_init_door_state_ambiguous"
    elif not restore_requested:
        action = "leave_closed_after_initialization"
        supported = True
        reason = "restore_not_requested"
    elif state == "closed":
        action = "ensure_closed_after_initialization"
        supported = True
        reason = "source_safe_closed_state"
    else:
        action = "explicit_open_restore_required_after_init"
        supported = False
        reason = "automatic_open_restore_not_implemented_safe_policy"
    return {
        "ok": state != "ambiguous",
        "implemented": supported,
        "restore_requested": bool(restore_requested),
        "before": classified,
        "recommended_action": action,
        "reason": reason,
        "source_command": "ControlLib.rehome door state save/restore around initializeMotors",
        "source_anchor": SOURCE_ANCHORS["control_initialize_motion"].to_dict(),
        "not_silent": True,
    }



def _safe_call(label: str, fn, *, movement: bool = False) -> dict[str, Any]:
    try:
        result = fn()
        ok = bool(isinstance(result, dict) and result.get("ok", True) is True)
        return {"name": label, "ok": ok, "physical_motion_commanded": bool(movement), "result": result}
    except Exception as exc:  # pragma: no cover - exercised by API integration paths
        return {"name": label, "ok": False, "physical_motion_commanded": bool(movement), "error": f"{type(exc).__name__}: {exc}"}


def run_oem_initialization_controller(
    tester: Any,
    *,
    run_homing: bool,
    restore_door_state: bool = False,
    include_tip_pipette_cleanup: bool = False,
    timeout_s: float = 180.0,
) -> dict[str, Any]:
    """Run a first-class source-anchored OEM initialization controller.

    This is the Phase-6 controller layer.  It orchestrates already-hardened lower
    primitives and records every phase.  It does not hide unsupported semantics.
    """
    phases: list[dict[str, Any]] = []
    manifest = build_machine_calibration_manifest()

    def add(phase_name: str, row: dict[str, Any]) -> dict[str, Any]:
        catalog = {p.name: p for p in OEM_INIT_PHASES}
        phase = catalog.get(phase_name)
        row = dict(row)
        row.setdefault("phase", phase_name)
        if phase is not None:
            row.setdefault("source_command", phase.source_command)
            row.setdefault("source_anchor", phase.source_anchor.to_dict())
            row.setdefault("linux_status", phase.linux_status)
        phases.append(row)
        return row

    add("accepted", {"ok": True, "run_homing": bool(run_homing), "restore_door_state": bool(restore_door_state), "include_tip_pipette_cleanup": bool(include_tip_pipette_cleanup)})

    initial_check = _safe_call("initial_check", lambda: {"ok": True, "note": "API route preconditions/interlocks checked by caller when live"})
    add("initial_check", initial_check)

    interlock = _safe_call("interlock_prepare", lambda: tester.motion_gate_live_snapshot() if hasattr(tester, "motion_gate_live_snapshot") else {"ok": True})
    add("interlock_prepare", interlock)
    if not interlock.get("ok"):
        return _finalize_oem_initialization(False, phases, manifest, "interlock_prepare")

    prep = _safe_call("initialize_without_motion", lambda: tester.motor_oem_initialize_without_motion())
    add("initialize_without_motion", prep)
    if not prep.get("ok"):
        return _finalize_oem_initialization(False, phases, manifest, "initialize_without_motion")

    door_capture = _safe_call("door_state_capture", lambda: tester.motor_plan_thermal_door_restore(restore_requested=restore_door_state))
    add("door_state_capture", door_capture)
    if not door_capture.get("ok"):
        return _finalize_oem_initialization(False, phases, manifest, "door_state_capture")

    if bool(run_homing):
        z_home = _safe_call("z_reference", lambda: tester.motor_oem_axis_already_home("z", tolerance_steps=2))
        z_result = z_home.get("result") if isinstance(z_home, dict) else None
        if not (isinstance(z_result, dict) and z_result.get("ok") is True):
            z_home = _safe_call("z_reference", lambda: tester.motor_oem_home_axis("z", startup=True, timeout_s=timeout_s), movement=True)
        add("z_reference", z_home)
        if not z_home.get("ok"):
            return _finalize_oem_initialization(False, phases, manifest, "z_reference")

        z_clear = _safe_call("z_clearance", lambda: tester.motor_oem_verify_z_clearance_for_xy(target=-15000, min_clearance=-10000, timeout_s=min(float(timeout_s), 30.0)), movement=True)
        add("z_reference", {**z_clear, "phase_detail": "z_clearance_for_xy"})
        if not z_clear.get("ok"):
            return _finalize_oem_initialization(False, phases, manifest, "z_clearance_for_xy")

        g_status = _safe_call("g_reference_already_home", lambda: _controller_gripper_status(tester), movement=False)
        g_body = g_status.get("result") if isinstance(g_status, dict) else None
        g_pred = (g_body.get("oem_home_predicate") if isinstance(g_body, dict) else {}) or {}
        if bool(g_pred.get("oem_confirmed_home")):
            add("g_reference", {**g_status, "already_home": True, "reason": "OEM confirmAxis(g) true: queryHome(MotorGrip) OR getG()<50; skip clear/home motion"})
        else:
            g_clear = _safe_call("g_reference_clear", lambda: _controller_gripper_clear(tester, timeout_s=timeout_s), movement=True)
            add("g_reference", g_clear)
            if not g_clear.get("ok"):
                return _finalize_oem_initialization(False, phases, manifest, "g_reference_clear")

            g_home = _safe_call("g_reference_home", lambda: _controller_gripper_home(tester, timeout_s=timeout_s), movement=True)
            add("g_reference", g_home)
            if not g_home.get("ok"):
                return _finalize_oem_initialization(False, phases, manifest, "g_reference_home")

        home_xy = _safe_call("home_xy", lambda: tester.motor_oem_home_xy(timeout_s=min(float(timeout_s), 60.0)), movement=True)
        add("home_xy", home_xy)
        if not home_xy.get("ok"):
            return _finalize_oem_initialization(False, phases, manifest, "home_xy")

        door_home = _safe_call("door_home_or_restore", lambda: tester.motor_oem_door_search_home(timeout_s=min(float(timeout_s), 45.0), startup=False), movement=True)
        add("door_home_or_restore", door_home)
        if not door_home.get("ok"):
            return _finalize_oem_initialization(False, phases, manifest, "door_home_or_restore")
    else:
        for phase in ("z_reference", "g_reference", "home_xy", "door_home_or_restore"):
            add(phase, {"ok": True, "skipped": True, "reason": "run_homing_false", "physical_motion_commanded": False})

    cleanup = {
        "ok": True,
        "requested": bool(include_tip_pipette_cleanup),
        "implemented": False,
        "reason": "tip/pipette cleanup remains separate until source-equivalent primitives are ported",
        "physical_motion_commanded": False,
    }
    add("tip_pipette_cleanup_or_unsupported", cleanup)

    park = {"ok": True, "implemented": False, "reason": "parkGantry/PrepareToRunJob remains separate; controller ends at initialized home/reference state", "physical_motion_commanded": False}
    add("park_or_ready_position", park)

    final = _safe_call("final_readiness", lambda: _controller_final_status(tester))
    add("final_readiness", final)
    ok = bool(final.get("ok"))
    return _finalize_oem_initialization(ok, phases, manifest, None if ok else "final_readiness")


def _controller_gripper_status(tester: Any) -> dict[str, Any]:
    from .oem_gripper import gripper_status

    return gripper_status(tester)


def _controller_gripper_clear(tester: Any, *, timeout_s: float) -> dict[str, Any]:
    from .oem_gripper import gripper_clear

    return gripper_clear(tester, operator_ack="GRIPPER_CLEAR", reason="OEM initialization controller G clear", timeout_s=min(float(timeout_s), 20.0))


def _controller_gripper_home(tester: Any, *, timeout_s: float) -> dict[str, Any]:
    from .oem_gripper import gripper_home

    return gripper_home(tester, operator_ack="GRIPPER_HOME", reason="OEM initialization controller G home", timeout_s=min(float(timeout_s), 20.0))


def _controller_final_status(tester: Any) -> dict[str, Any]:
    status: dict[str, Any] = {"ok": True}
    if hasattr(tester, "motion_arm_state"):
        status["motion_arm"] = tester.motion_arm_state()
    if hasattr(tester, "motor_axis_status") and hasattr(tester, "_motion_oem_axis_profile"):
        axes = {}
        for axis in ("x", "y", "z", "g", "door"):
            try:
                preset = tester._motion_oem_axis_profile(axis, startup=True)
                axes[axis] = tester.motor_axis_status(preset["board"], motor=preset["motor"])
            except Exception as exc:  # pragma: no cover - defensive live path
                axes[axis] = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        status["axes"] = axes
        for axis, row in axes.items():
            speed = row.get("speed") if isinstance(row, dict) else None
            if isinstance(speed, dict) and speed.get("speed") not in (0, None):
                status["ok"] = False
                status.setdefault("blockers", []).append(f"{axis}_speed_nonzero")
    return status


def _finalize_oem_initialization(ok: bool, phases: list[dict[str, Any]], manifest: dict[str, Any], failed_at: str | None) -> dict[str, Any]:
    return {
        "ok": bool(ok),
        "ready": bool(ok),
        "schema": "bioxp.oem_initialization_controller.v1",
        "source_mode": "GenBotApp.initializeSystem -> ControlLib.initializeMotion -> initializeMotors",
        "route_semantics": {"not_equivalent_to": ["/motion/axis/home", "/motion/axis/zero", "/motion/oem/rehome monolith"]},
        "failed_at": failed_at,
        "machine_calibration_manifest": manifest,
        "phases": phases,
        "phase_names": [row.get("phase") for row in phases],
    }
