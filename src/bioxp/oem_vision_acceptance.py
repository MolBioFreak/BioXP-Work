"""Acceptance contracts for OEM CheckCamera and CVision startup operations."""
from __future__ import annotations

from typing import Any


class OemVisionReceiptError(ValueError):
    pass


CHECK_CAMERA_SOURCE_ANCHOR = "ControlLib.CheckCamera:1929-1960"


def _exact_bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise OemVisionReceiptError(f"{name} must be an exact bool")
    return value


def _mapping(value: Any, name: str, keys: set[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise OemVisionReceiptError(f"{name} must be an object")
    missing = sorted(keys - set(value))
    extra = sorted(set(value) - keys)
    if missing:
        raise OemVisionReceiptError(f"{name} missing fields: {missing}")
    if extra:
        raise OemVisionReceiptError(f"{name} has unexpected fields: {extra}")
    return value


def _validate_attempt(attempt: Any, *, index: int, expected_offset_x: int) -> tuple[bool, bool]:
    row = _mapping(
        attempt,
        f"attempts[{index}]",
        {"location_id", "offset_x", "offset_y", "check_label", "failure_snapshot_written"},
    )
    if type(row["location_id"]) is not int or row["location_id"] != 23:
        raise OemVisionReceiptError(f"attempts[{index}].location_id must be exact OEM location 23")
    if type(row["offset_x"]) is not int or type(row["offset_y"]) is not int:
        raise OemVisionReceiptError(f"attempts[{index}] offsets must be exact integers")
    if row["offset_x"] != expected_offset_x or row["offset_y"] != 0:
        ordinal = "first" if index == 0 else "second"
        raise OemVisionReceiptError(f"{ordinal} camera attempt does not match the OEM offset")
    detected = _exact_bool(row["check_label"], f"attempts[{index}].check_label")
    snapshot = _exact_bool(row["failure_snapshot_written"], f"attempts[{index}].failure_snapshot_written")
    if detected and snapshot:
        raise OemVisionReceiptError(f"attempts[{index}] cannot write a failure snapshot after success")
    if not detected and not snapshot:
        raise OemVisionReceiptError(f"attempts[{index}] must persist the OEM failure snapshot")
    return detected, snapshot


def evaluate_check_camera_receipt(receipt: dict[str, Any]) -> dict[str, Any]:
    row = _mapping(
        receipt,
        "receipt",
        {
            "camera_owner",
            "settings_item",
            "led_white_acknowledged",
            "door_closed_verified",
            "attempts",
            "location_23_persisted",
            "all_leds_off_acknowledged",
            "gantry_park_verified",
        },
    )
    if row["camera_owner"] != "oem_full_lifecycle":
        raise OemVisionReceiptError("camera_owner must be the robot-owned full lifecycle")
    if type(row["settings_item"]) is not int or row["settings_item"] != 4:
        raise OemVisionReceiptError("settings_item must be OEM InspectionItems value 4")
    led_white = _exact_bool(row["led_white_acknowledged"], "led_white_acknowledged")
    door_closed = _exact_bool(row["door_closed_verified"], "door_closed_verified")
    attempts = row["attempts"]
    if not isinstance(attempts, list) or len(attempts) not in {1, 2}:
        raise OemVisionReceiptError("attempts must contain one or two OEM camera attempts")
    first_detected, _ = _validate_attempt(attempts[0], index=0, expected_offset_x=4738)
    if first_detected and len(attempts) != 1:
        raise OemVisionReceiptError("CheckCamera must stop after first success")
    if not first_detected and len(attempts) != 2:
        raise OemVisionReceiptError("CheckCamera requires the exact second attempt after first failure")
    second_detected = False
    if len(attempts) == 2:
        second_detected, _ = _validate_attempt(attempts[1], index=1, expected_offset_x=1895)

    cleanup_failures: list[str] = []
    for field, failure in (
        ("location_23_persisted", "location_23_not_persisted"),
        ("all_leds_off_acknowledged", "all_leds_off_not_verified"),
        ("gantry_park_verified", "gantry_park_not_verified"),
    ):
        if not _exact_bool(row[field], field):
            cleanup_failures.append(failure)
    if not led_white:
        cleanup_failures.append("white_led_not_verified")
    if not door_closed:
        cleanup_failures.append("door_closed_not_verified")
    label_detected = first_detected or second_detected
    failure = None if label_detected else "camera_label_not_detected"
    ok = bool(label_detected and not cleanup_failures)
    return {
        "ok": ok,
        "production_admission_pass": ok,
        "physical_effect_verified": ok,
        "label_detected": label_detected,
        "attempt_count": len(attempts),
        "failure": failure,
        "cleanup_verified": not cleanup_failures,
        "cleanup_failures": cleanup_failures,
        "source_anchor": CHECK_CAMERA_SOURCE_ANCHOR,
        "exact_attempt_offsets_x": [4738, 1895],
        "camera_session_disposition": "not_released_by_CheckCamera",
    }


INSPECT_COVER_SOURCE_ANCHOR = "ControlLib.inspectCover:3663-3768"


def evaluate_inspect_cover_receipt(receipt: dict[str, Any]) -> dict[str, Any]:
    row = _mapping(
        receipt,
        "receipt",
        {
            "deck_inspection",
            "screen_resolution_high",
            "observed_locations",
            "inspection_methods",
            "cover_detected",
            "relocations",
            "final_cover_locations",
            "door_closed_verified",
            "door_open_verified",
            "inspection_log_only",
        },
    )
    enabled = _exact_bool(row["deck_inspection"], "deck_inspection")
    high_resolution = _exact_bool(row["screen_resolution_high"], "screen_resolution_high")
    inspection_log_only = _exact_bool(row["inspection_log_only"], "inspection_log_only")
    door_closed = _exact_bool(row["door_closed_verified"], "door_closed_verified")
    door_open = _exact_bool(row["door_open_verified"], "door_open_verified")
    observed = row["observed_locations"]
    if not isinstance(observed, list) or any(type(location) is not int for location in observed):
        raise OemVisionReceiptError("observed_locations must be an exact integer list")
    source_order = [17, 19, 20, 18]
    if observed != source_order[: len(observed)]:
        raise OemVisionReceiptError("observed_locations must be an exact OEM source-order prefix")

    if not enabled:
        if observed or row["inspection_methods"] != {} or row["cover_detected"] != {} or row["relocations"] != []:
            raise OemVisionReceiptError("disabled deck inspection must not contain camera or movement evidence")
        if row["final_cover_locations"] is not None or door_closed or door_open:
            raise OemVisionReceiptError("disabled deck inspection cannot claim door or cover effects")
        return {
            "ok": True,
            "outcome": "deck_inspection_disabled_after_force_high_home",
            "production_admission_pass": True,
            "oem_effective_pass": True,
            "physical_effect_verified": False,
            "terminal_after_location": None,
            "source_anchor": INSPECT_COVER_SOURCE_ANCHOR,
            "camera_session_disposition": "not_released_by_inspectCover",
        }

    if not door_closed:
        raise OemVisionReceiptError("enabled inspectCover requires verified doorOpen(false)")
    expected_all_methods = (
        {17: "InspectOutputLocation(1)", 19: "checkRCCover", 20: "checkCoverStorage", 18: "checkCoverStorage"}
        if high_resolution
        else {17: "checkChillerCover", 19: "checkChillerCover", 20: "checkChillerCover", 18: "checkChillerCover"}
    )
    expected_keys = {str(location) for location in observed}
    methods = _mapping(row["inspection_methods"], "inspection_methods", expected_keys)
    detected = _mapping(row["cover_detected"], "cover_detected", expected_keys)
    for location in observed:
        if methods[str(location)] != expected_all_methods[location]:
            raise OemVisionReceiptError("inspection_methods do not match the selected OEM resolution branch")
    detected_bool = {
        location: _exact_bool(detected[str(location)], f"cover_detected.{location}")
        for location in observed
    }

    cover_count = 0
    found_chillers: list[int] = []
    empty_storage: list[int] = []
    error_status: str | None = None
    terminal_after_location: int | None = None
    for location in observed:
        present = detected_bool[location]
        if location in {17, 19}:
            if present:
                found_chillers.append(location)
                cover_count += 1
            continue
        if present:
            if cover_count >= 2:
                error_status = "OVER_CHILLER_COVER"
                if not inspection_log_only:
                    terminal_after_location = location
                    break
            cover_count += 1
        else:
            empty_storage.append(location)

    if terminal_after_location is not None:
        if observed[-1] != terminal_after_location:
            raise OemVisionReceiptError("receipt claims observations after the OEM early-return location")
    elif observed != source_order:
        raise OemVisionReceiptError("nonterminal inspectCover receipt must include all four OEM locations")

    if terminal_after_location is None and cover_count < 2:
        error_status = "SHORT_CHILLER_COVER"
    expected_relocations: list[dict[str, Any]] = []
    if cover_count == 2 and error_status is None:
        if len(found_chillers) != len(empty_storage):
            raise OemVisionReceiptError("cover topology cannot be paired by the OEM source order")
        expected_relocations = [
            {"cover": "reagent" if source == 19 else "output", "from": source, "to": destination}
            for source, destination in zip(found_chillers, empty_storage)
        ]
    if row["relocations"] != expected_relocations:
        raise OemVisionReceiptError("relocations do not match the exact OEM detected/empty list order")

    production_pass = error_status is None and cover_count == 2
    if production_pass:
        if row["final_cover_locations"] != {"output": 18, "reagent": 20}:
            raise OemVisionReceiptError("final_cover_locations must be output=18 and reagent=20")
        if not door_open:
            raise OemVisionReceiptError("successful inspectCover must verify doorOpen(true)")
    else:
        if row["final_cover_locations"] is not None:
            raise OemVisionReceiptError("failed cover path cannot claim final cover locations")
        if door_open:
            raise OemVisionReceiptError("failed cover path cannot claim the successful door-open terminal")

    oem_effective_pass = production_pass or bool(inspection_log_only and error_status is not None)
    return {
        "ok": production_pass,
        "outcome": "covers_canonicalized" if production_pass else "cover_inspection_failed",
        "production_admission_pass": production_pass,
        "oem_effective_pass": oem_effective_pass,
        "physical_effect_verified": production_pass,
        "cover_count": cover_count,
        "error_status": error_status,
        "terminal_after_location": terminal_after_location,
        "observed_locations": observed,
        "expected_relocations": expected_relocations,
        "source_anchor": INSPECT_COVER_SOURCE_ANCHOR,
        "camera_session_disposition": "not_released_by_inspectCover",
    }
