"""Acceptance contracts for OEM CheckCamera and CVision startup operations."""
from __future__ import annotations

import json
from typing import Any, Mapping


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
        "status": "receipt_valid" if ok else "receipt_rejected",
        "receipt_validation_pass": ok,
        "production_admission_pass": False,
        "provider_live_bound": False,
        "physical_motion_commanded": False,
        "physical_effect_verified": False,
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


def _validate_force_to_high_home(value: Any) -> list[str]:
    force = _mapping(
        value,
        "force_to_high_home",
        {
            "provider_id", "command_id", "attempted", "controller_acknowledged",
            "postcondition_verified", "attempted_at_ms", "acknowledged_at_ms",
            "postcondition_verified_at_ms",
        },
    )
    for field in ("provider_id", "command_id"):
        if not isinstance(force[field], str) or not force[field].strip() or len(force[field]) > 128:
            raise OemVisionReceiptError(f"force_to_high_home.{field} must be a bounded nonblank identifier")
    for field in ("attempted_at_ms", "acknowledged_at_ms", "postcondition_verified_at_ms"):
        if type(force[field]) is not int or force[field] < 0:
            raise OemVisionReceiptError(f"force_to_high_home.{field} must be an exact nonnegative integer")
    if not (force["attempted_at_ms"] <= force["acknowledged_at_ms"] <= force["postcondition_verified_at_ms"]):
        raise OemVisionReceiptError("force_to_high_home timestamps must preserve attempt/ack/postcondition order")
    failures: list[str] = []
    for field in ("attempted", "controller_acknowledged", "postcondition_verified"):
        if not _exact_bool(force[field], f"force_to_high_home.{field}"):
            failures.append(f"force_to_high_home_{field}_false")
    return failures


def evaluate_inspect_cover_receipt(receipt: dict[str, Any]) -> dict[str, Any]:
    row = _mapping(
        receipt,
        "receipt",
        {
            "force_to_high_home",
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
    force_failures = _validate_force_to_high_home(row["force_to_high_home"])
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
        receipt_validation_pass = not force_failures
        return {
            "ok": receipt_validation_pass,
            "status": "receipt_valid" if receipt_validation_pass else "receipt_rejected",
            "outcome": "deck_inspection_disabled_after_force_high_home",
            "receipt_validation_pass": receipt_validation_pass,
            "production_admission_pass": False,
            "provider_live_bound": False,
            "physical_motion_commanded": False,
            "oem_effective_pass": receipt_validation_pass,
            "physical_effect_verified": False,
            "failures": force_failures,
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
        # The OEM zips stations [17,19] with empty storage [20,18],
        # crossing both covers. Storage observations do not identify a cover;
        # only two observed station identities and two empty targets prove a
        # safe complete transfer and justify the terminal custody assertion.
        if found_chillers != [17, 19] or empty_storage != [20, 18]:
            error_status = "UNSAFE_COVER_TOPOLOGY"
        else:
            expected_relocations = [
                {"cover": "output", "from": 17, "to": 18},
                {"cover": "reagent", "from": 19, "to": 20},
            ]
    if row["relocations"] != expected_relocations:
        raise OemVisionReceiptError("relocations do not match safe observed source-to-storage pairing")

    semantic_pass = error_status is None and cover_count == 2
    if semantic_pass:
        if row["final_cover_locations"] != {"output": 18, "reagent": 20}:
            raise OemVisionReceiptError("final_cover_locations must be output=18 and reagent=20")
        if not door_open:
            raise OemVisionReceiptError("successful inspectCover must verify doorOpen(true)")
    else:
        if row["final_cover_locations"] is not None:
            raise OemVisionReceiptError("failed cover path cannot claim final cover locations")
        if door_open:
            raise OemVisionReceiptError("failed cover path cannot claim the successful door-open terminal")

    failures = list(force_failures)
    if not semantic_pass and error_status is not None:
        failures.append(error_status.lower())
    receipt_validation_pass = semantic_pass and not force_failures
    oem_effective_pass = not force_failures and (
        semantic_pass or bool(inspection_log_only and error_status in {"OVER_CHILLER_COVER", "SHORT_CHILLER_COVER"})
    )
    return {
        "ok": receipt_validation_pass,
        "status": "receipt_valid" if receipt_validation_pass else "receipt_rejected",
        "outcome": "covers_canonicalized" if semantic_pass else "cover_inspection_failed",
        "receipt_validation_pass": receipt_validation_pass,
        "production_admission_pass": False,
        "provider_live_bound": False,
        "physical_motion_commanded": False,
        "oem_effective_pass": oem_effective_pass,
        "physical_effect_verified": False,
        "failures": failures,
        "cover_count": cover_count,
        "error_status": error_status,
        "terminal_after_location": terminal_after_location,
        "observed_locations": observed,
        "expected_relocations": expected_relocations,
        "source_anchor": INSPECT_COVER_SOURCE_ANCHOR,
        "camera_session_disposition": "not_released_by_inspectCover",
    }


def plan_cover_relocations(detected: Mapping[Any, bool]) -> dict[str, Any]:
    """Execution planner for inspectCover's num==2 relocation block (3727-3766).

    Preserve the OEM's source-order observations and count errors, but never
    execute its crossed index pairing. Only identified covers at both stations
    with both storage targets empty can justify a complete, safe relocation.
    """
    order = (17, 19, 20, 18)
    values: dict[int, bool] = {}
    for location in order:
        values[location] = _exact_bool(
            detected.get(location, detected.get(str(location))),
            f"cover_detected.{location}",
        )
    cover_count = 0
    found: list[int] = []
    empty: list[int] = []
    error_status: str | None = None
    for location in order:
        present = values[location]
        if location in {17, 19}:
            if present:
                found.append(location)
                cover_count += 1
            continue
        if present:
            if cover_count >= 2:
                # Source marks OVER_CHILLER_COVER; the exact count beyond the
                # gate cannot reach the num==2 relocation branch either way.
                error_status = "OVER_CHILLER_COVER"
                break
            cover_count += 1
        else:
            empty.append(location)
    if error_status is None and cover_count < 2:
        error_status = "SHORT_CHILLER_COVER"
    relocations: list[dict[str, Any]] = []
    if cover_count == 2 and error_status is None:
        if found != [17, 19] or empty != [20, 18]:
            error_status = "UNSAFE_COVER_TOPOLOGY"
        else:
            relocations = [
                {"cover": "output", "from": 17, "to": 18},
                {"cover": "reagent", "from": 19, "to": 20},
            ]
    return {
        "detected": {location: values[location] for location in order},
        "cover_count": cover_count,
        "error_status": error_status,
        "found_chillers": found,
        "empty_storage": empty,
        "relocations": relocations,
        "source_anchor": INSPECT_COVER_SOURCE_ANCHOR,
    }


def _recursive_bool(value: Any, key: str) -> bool | None:
    if isinstance(value, dict):
        if type(value.get(key)) is bool:
            return value[key]
        for item in value.values():
            found = _recursive_bool(item, key)
            if found is not None:
                return found
    elif isinstance(value, (list, tuple)):
        for item in value:
            found = _recursive_bool(item, key)
            if found is not None:
                return found
    return None


def assemble_inspect_cover_receipt(evidence: Mapping[str, Any], *, settings: Mapping[str, Any]) -> dict[str, Any]:
    """Assemble the evaluate_inspect_cover_receipt-shaped receipt.

    Everything comes from the command's own child ledger and transitions; no
    field is synthesized that the run did not record.
    """
    children = [dict(row) for row in (evidence.get("children") or [])]
    transitions = [dict(row) for row in (evidence.get("state_transitions") or [])]

    def parse(row):
        raw = row.get("terminal_evidence_json")
        data = json.loads(raw) if isinstance(raw, str) and raw.strip() else None
        if isinstance(data, dict):
            inner = data.get("result")
            return dict(inner) if isinstance(inner, dict) else dict(data)
        return {}

    def child_result(operation: str) -> dict:
        for row in children:
            if str(row.get("operation")) == operation:
                return parse(row)
        return {}

    detected: dict = {}
    methods: dict = {}
    for row in children:
        if str(row.get("operation")) != "inspectCoverAt":
            continue
        result = parse(row)
        location = result.get("location")
        if type(location) is int:
            detected[str(location)] = bool(result.get("cover_detected"))
            methods[str(location)] = str(result.get("method"))
    relocate = child_result("coverInspectionRelocate")
    door_close = child_result("doorOpen")
    door_closed = _recursive_bool(door_close, "door_closed")

    attempted_ms = 0
    acknowledged_ms = 0
    verified_ms = 0
    stamps = sorted(
        float(row.get("created_at") or 0.0)
        for row in transitions
        if int(row.get("child_order", -1)) == 0
    )
    if stamps:
        attempted_ms = int(stamps[0] * 1000)
        acknowledged_ms = int(stamps[-1] * 1000)
        all_stamps = [float(row.get("created_at") or 0.0) for row in transitions]
        verified_ms = int((max(all_stamps) if all_stamps else stamps[-1]) * 1000)
    acknowledged_ms = max(attempted_ms, acknowledged_ms)
    verified_ms = max(acknowledged_ms, verified_ms)
    # DefaultParameters.ForceToHighHome is a software pseudo-home: the child's
    # completion IS the publication (no controller delivery exists to ack); the
    # receipt records exactly that, never a fabricated controller exchange.
    force_rows = [row for row in children if str(row.get("operation")) == "sourceForceToHighHome"]
    force_attempted = any(str(row.get("terminal_state")) != "planned" for row in force_rows)
    force_completed = any(str(row.get("terminal_state")) == "completed" for row in force_rows)
    return {
        "force_to_high_home": {
            "provider_id": "serial206-cover-inspection",
            "command_id": str(evidence.get("operation", {}).get("command_id") or ""),
            "attempted": bool(force_attempted),
            "controller_acknowledged": bool(force_completed),
            "postcondition_verified": bool(force_completed),
            "attempted_at_ms": attempted_ms,
            "acknowledged_at_ms": acknowledged_ms,
            "postcondition_verified_at_ms": verified_ms,
        },
        "deck_inspection": bool(settings["deck_inspection"]),
        "screen_resolution_high": bool(settings["screen_resolution_high"]),
        "observed_locations": [17, 19, 20, 18],
        "inspection_methods": methods,
        "cover_detected": detected,
        "relocations": list(relocate.get("relocations") or []),
        "final_cover_locations": relocate.get("final_cover_locations"),
        "door_closed_verified": bool(door_closed is True),
        "door_open_verified": bool(relocate.get("door_open_verified") is True),
        "inspection_log_only": bool(settings["inspection_log_only"]),
    }
