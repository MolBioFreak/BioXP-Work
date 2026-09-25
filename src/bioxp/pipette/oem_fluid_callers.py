"""Pure caller ordering for ControlLib Detect Fluid:1440-1469 and calwithFluid:2782-2850.

Callbacks execute under the finite owner's existing claim. This module never binds
saved PositionTable geometry to a running machine or commands hardware itself.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping, cast

from ..oem_calibration_settings import PositionName

from ..oem_calibration_settings import CalibrationSettingsPatch, CalibrationSettingsService, PositionCalibrationPatch
from .oem_calibration import calibration_adjustment


@dataclass(frozen=True)
class FluidCallerBindings:
    log_file: Callable[[], None]
    initiate_group: Callable[[], None]
    catch_plate: Callable[[int], None]
    release_plate: Callable[[int, bool], None]  # destination, press_plate
    press_plates: Callable[[tuple[int, ...]], None]
    scan: Callable[[str, int, bool, int], Mapping[str, Any]]  # plate, speed, transfer, skip
    mark_strip: Callable[[], None]
    park: Callable[[], None]
    error: Callable[[str, Exception], None]
    # Calibration-only callbacks. No synthetic loadTips or rebinding in this layer.
    reset_status: Callable[[], None] | None = None
    tip_exists: Callable[[], bool] | None = None
    move_waste: Callable[[], None] | None = None
    sleep_ms: Callable[[int], None] | None = None
    eject_tips: Callable[[], None] | None = None
    completed: Callable[[], None] | None = None
    finish_ui: Callable[[], None] | None = None
    set_z_acceleration: Callable[[int], None] | None = None
    compare: Callable[[Mapping[str, Any], Mapping[str, Any]], bool | None] | None = None
    save_history: Callable[[], None] | None = None
    restore: Callable[[Mapping[str, Any] | None], None] | None = None


def detect_fluid(bindings: FluidCallerBindings) -> dict[str, Any]:
    """Diagnostic button: no adjustZ/saveConfig and no finally cleanup."""
    scans: list[dict[str, Any]] = []
    result: dict[str, Any] = {"source": "ControlLib.btnDetectFluid_Click:1440-1469", "scans": scans, "completed": False}
    def scan(plate: str) -> None:
        raw = bindings.scan(plate, 300, False, 1)
        scans.append({"plate": plate, "measured_raw_z": raw["source_return"], "scan": raw})
    try:
        bindings.log_file()
        bindings.catch_plate(0)
        bindings.release_plate(23, True)
        # initiateGroup is not zOffset.loadTips: owner supplies this separate callback.
        # It is required at this source position, before TC.
        bindings.initiate_group()
        scan("TC")
        bindings.catch_plate(0)
        bindings.release_plate(25, True)
        scan("MS")
        bindings.press_plates((1,))
        scan("OC")
        bindings.press_plates((2,))
        scan("RC")
        bindings.mark_strip()
        scan("STRIP")
        bindings.park()
        result["completed"] = True
    except Exception as exc:
        result["error"] = str(exc)
        bindings.error("btnDetectFluid_Click", exc)
    return result


def calwith_fluid(bindings: FluidCallerBindings, settings: CalibrationSettingsService,
                  fluid_reference: Mapping[str, int], *, machine_calibrated: bool) -> dict[str, Any]:
    """Calibration worker body, after dialog backup/UI setup in the OEM outer click.

    `compare` supplies the explicit acceptance choice (True, False, or None).
    `restore` must atomically reinstate the prior saved revision, including None.
    Without a decision/restore, the outcome is incomplete, never accepted.
    """
    before = settings.read()
    measurements: list[dict[str, Any]] = []
    result: dict[str, Any] = {"source": "ControlLib.calwithFluid:2782-2850",
        "adjustment_source": "ClassBioXPSettings.adjustZ:6039-6138",
        "fluid_reference_revision": str(fluid_reference["REVISION"]),
        "previous_saved_revision_id": before["saved_revision_id"],
        "active_revision_id": before["active_revision_id"], "measurements": measurements,
        "body_completed": False, "outcome": "incomplete"}

    def required(name: str) -> Callable[..., Any]:
        fn = getattr(bindings, name)
        if fn is None:
            raise ValueError(f"{name} callback required")
        return fn

    def measure(plate: str) -> None:
        raw = bindings.scan(plate, 300, True, 4)
        z = raw["source_return"]
        current = settings.read()["saved_positions"]
        by_name = {row["name"]: row for row in current}
        id_names = {2: "LOC_TC", 0: "LOC_MS", 1: "LOC_OC", 3: "LOC_RC",
                    11: "LOC_STRIP1", 12: "LOC_STRIP2", 13: "LOC_STRIP3", 14: "LOC_STRIP4"}
        lows = {idx: by_name[name]["zLow"] for idx, name in id_names.items()}
        derived = calibration_adjustment(plate, z, z_lows=lows, fluid_reference=fluid_reference)
        updates = derived["position_table_z_low_updates"]
        patch = CalibrationSettingsPatch(positions=tuple(
            PositionCalibrationPatch(name=cast(PositionName, id_names[idx]), zLow=low) for idx, low in updates.items()))
        # Source adjustZ then saveConfig, including each intermediate saved revision.
        saved = settings.save(patch, liquid_reference_revision=str(fluid_reference["REVISION"]))
        measurements.append({"plate": plate, "measured_raw_z": z,
            "calculated_z_lows": {id_names[idx]: low for idx, low in updates.items()},
            "settings_updates": derived["settings_updates"],
            "saved_revision_id": saved["saved_revision_id"],
            "pending_restart": saved["pending_restart"], "scan": raw})

    try:
        try:
            bindings.log_file()
            required("reset_status")()
            bindings.press_plates((0,))
            measure("TC")
            bindings.catch_plate(0)
            bindings.release_plate(25, True)
            measure("MS")
            bindings.catch_plate(0)
            bindings.release_plate(23, False)
            bindings.press_plates((1,))
            measure("OC")
            bindings.press_plates((2,))
            measure("RC")
            bindings.mark_strip()
            measure("STRIP")
            result["body_completed"] = True
        except Exception as exc:
            result["error"] = str(exc)
            bindings.error("calwithFluid", exc)
        finally:
            if required("tip_exists")():
                required("move_waste")()
                required("sleep_ms")(20)
                required("eject_tips")()
                required("sleep_ms")(20)
            bindings.park()
            required("completed")()
            # OEM compares even after a body failure. Backup exists only when
            # m_calibrated==1; absent backup yields OEM's true, not user consent.
            if machine_calibrated and bindings.compare is not None:
                choice = bindings.compare(before, settings.read())
                result["comparison_choice"] = choice
                if choice is True and result["body_completed"]:
                    if bindings.save_history is not None:
                        bindings.save_history()
                        result["outcome"] = "accepted"
                elif choice is False:
                    if bindings.restore is not None:
                        bindings.restore(before["saved_revision"])
                        restored = settings.read()
                        if restored["saved_revision"] == before["saved_revision"]:
                            result["outcome"] = "rejected_restored"
                        else:
                            result["restore_error"] = "saved revision readback differs"
            result["saved_revision_id"] = settings.read()["saved_revision_id"]
            result["pending_restart"] = settings.read()["pending_restart"]
            required("finish_ui")()
            required("set_z_acceleration")(576)
    except Exception as exc:
        result["finalization_error"] = str(exc)
    return result
