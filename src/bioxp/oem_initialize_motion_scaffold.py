
"""Source-only initializeMotion pipette/vision parity scaffold."""
from __future__ import annotations

from typing import Any


def _step(step_id: str, source_anchor: str, intent: str, *, blocker: str | None = None) -> dict[str, Any]:
    row = {"step_id": step_id, "source_anchor": source_anchor, "intent": intent, "live_ported": False}
    if blocker:
        row["blocker"] = blocker
    return row


def initialize_motion_parity_plan() -> dict[str, Any]:
    blockers = ["pipette_cleanup_not_live_ported", "vision_inspection_not_oem_equivalent", "requires_initialize_motors_stepwise_signoff"]
    steps = [
        _step("initializeMotors", "ControlLib.initializeMotion 8797-8805", "Run OEM initializeMotors before tip cleanup.", blocker="requires_initialize_motors_stepwise_signoff"),
        _step("queryTipStatus.initial", "ControlLib.initializeMotion 8806-8813", "Read pipette tip status before cleanup branch.", blocker="pipette_cleanup_not_live_ported"),
        _step("openThermalDoor.tip_cleanup", "ControlLib.initializeMotion 8814-8818", "Open thermal door only if tip cleanup branch is active.", blocker="thermal_door_restore_not_signed_off"),
        _step("scriptmoveTo.tip_cleanup", "ControlLib.initializeMotion 8819-8824", "Move from location 28 to 6 during tip cleanup.", blocker="pipette_cleanup_not_live_ported"),
        _step("updateLocation", "ControlLib.initializeMotion 8825", "Update control interface location after script move.", blocker="location_model_not_bound"),
        _step("ejectAllTips", "ControlLib.initializeMotion 8826-8834", "Eject all tips and clear tip state.", blocker="pipette_cleanup_not_live_ported"),
        _step("moveZ.tip_cleanup", "ControlLib.initializeMotion 8835-8836", "Move Z to cleanup clearance.", blocker="live_motion_not_signed_off"),
        _step("moveX.tip_cleanup", "ControlLib.initializeMotion 8837-8838", "Move X to cleanup clearance.", blocker="live_motion_not_signed_off"),
        _step("queryTipStatus.verify_empty", "ControlLib.initializeMotion 8839-8845", "Verify no tips remain.", blocker="pipette_cleanup_not_live_ported"),
        _step("initiateGroup", "ControlLib.initializeMotion 8846-8851", "Initialize pipette group.", blocker="pipette_group_not_oem_equivalent"),
        _step("checkedPipetteStatus.retry", "ControlLib.initializeMotion 8852-8856", "Retry checked pipette status.", blocker="pipette_status_retry_not_oem_equivalent"),
        _step("vision.camera_calibrated_gate", "ClassControlInterface.initializeMotors 3408-3413", "Preserve camera-calibrated thermal-door failure gate.", blocker="vision_inspection_not_oem_equivalent"),
    ]
    return {
        "program": "initialize_motion",
        "source_mode": "source_only_scaffold",
        "opened_usb": False,
        "physical_motion": False,
        "live_allowed_default": False,
        "steps": steps,
        "blockers": blockers,
    }
