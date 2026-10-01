"""Acceptance contracts for OEM CheckCamera and CVision startup operations."""
from __future__ import annotations

import json
from typing import Any, Mapping


class OemVisionReceiptError(ValueError):
    pass




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

    # OEM doorOpen(false) may return successfully from its retained-state
    # no-op without reading the physical switches. Preserve the observed
    # evidence bit, but do not reject completed inspection solely for its
    # absence or pretend that the no-op proves physical closure.
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


def inspection_transfer_outcome(result: Mapping[str, Any]) -> dict[str, Any]:
    """Keep source outcomes, not duplicate nested controller transport trees."""
    fields = (
        "ok", "operation", "order", "source_return", "source_return_code",
        "source_noop", "source_branch_skipped", "delivery_attempted",
        "controller_command_acknowledged", "controller_completion_verified",
        "controller_terminal_state_verified", "hardware_postcondition_verified",
        "physical_effect_verified", "exception_suppressed", "exception_type",
        "exception_message", "exception", "error", "failure", "failed_child",
        "residual_state", "background_pending", "source_plan_digest",
        "semantic_state_committed", "door_open", "door_closed", "phase",
        "command_id", "child_order", "dispatch_attempt_id", "plan_digest",
        "ownership_generation", "board_epoch_4", "board_epoch_5", "source_anchor",
        "target", "effective_target", "completion_class", "outcome_unknown",
        "physical_motion_commanded", "command_issued", "source_call_completed",
    )
    outcome = {key: result[key] for key in fields if key in result}
    for key in ("completed_children", "failure_evidence", "provider_results"):
        if isinstance(result.get(key), (list, tuple)):
            outcome[key] = [inspection_transfer_outcome(row) if isinstance(row, Mapping) else row
                            for row in result[key]]
    for key in ("result", "motion_evidence"):
        if isinstance(result.get(key), Mapping):
            outcome[key] = inspection_transfer_outcome(result[key])
    return outcome


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
