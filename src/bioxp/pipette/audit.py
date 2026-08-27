from __future__ import annotations

from collections.abc import Mapping
from typing import Any


class PipetteAuditIntegrityError(ValueError):
    """A producer payload violates a typed transport or observation invariant."""


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _first(mapping: Mapping[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in mapping:
            return mapping[key]
    return None


def _as_items(value: Any) -> list[dict[str, Any]]:
    if isinstance(value, Mapping):
        return [dict(value)]
    if isinstance(value, (list, tuple)):
        return [dict(item) for item in value if isinstance(item, Mapping)]
    return []


def _flatten_query_row(value: Mapping[str, Any]) -> dict[str, Any]:
    """Merge bounded producer `result` wrappers while preserving outer identity."""
    row = dict(value)
    for _ in range(8):
        nested = row.get("result")
        if not isinstance(nested, Mapping):
            return row
        outer = {key: item for key, item in row.items() if key != "result"}
        row = {**dict(nested), **outer}
    if isinstance(row.get("result"), Mapping):
        raise PipetteAuditIntegrityError("pipette query result nesting exceeds the bound")
    return row


def _channel_rows(result: Mapping[str, Any], provenance: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = [_flatten_query_row(row) for row in _as_items(result.get("channels"))]
    if not rows:
        rows = [
            _flatten_query_row(row)
            for row in _as_items(result.get("channel_observations"))
        ]
    if not rows:
        rows = [_flatten_query_row(row) for row in _as_items(result.get("results"))]
    if not rows:
        for send in _as_items(result.get("sends")):
            payload = _flatten_query_row(_mapping(send.get("result")))
            driver_result = _mapping(payload.get("driver_result"))
            row = {**payload, "channel": send.get("channel")}
            row.setdefault("hardware_tip_status", payload.get("hardware_tip_status"))
            row.setdefault("hardware_pressure", payload.get("hardware_pressure"))
            detail = dict(_mapping(row.get("detail")))
            if driver_result:
                detail["driver_result"] = driver_result
            row["detail"] = detail
            rows.append(row)
    if not rows and isinstance(result.get("result"), Mapping):
        row = _flatten_query_row(result)
        nested_provenance = _mapping(row.get("provenance"))
        channel = row.get("channel", nested_provenance.get("channel"))
        if channel is not None:
            row["channel"] = channel
            rows.append(row)
    if not rows:
        for candidate in (result.get("driver_result"), result.get("last_transaction")):
            nested = _flatten_query_row(_mapping(candidate))
            nested_provenance = _mapping(nested.get("provenance"))
            if nested_provenance.get("channel") is not None or result.get("channel") is not None:
                rows.append(
                    {
                        **nested,
                        "channel": result.get("channel", nested_provenance.get("channel")),
                        "phase": "completion" if result.get("completion_received") else "query",
                        "hardware_tip_status": result.get("hardware_tip_status"),
                        "hardware_pressure": result.get("hardware_pressure"),
                        "detail": {"driver_result": nested},
                    }
                )
                break
    if not rows and provenance.get("channel") is not None:
        rows = [
            {
                "channel": provenance.get("channel"),
                "phase": "completion" if result.get("completion_received") else "query",
                "hardware_tip_status": result.get("hardware_tip_status"),
                "hardware_pressure": result.get("hardware_pressure"),
                "status": result.get("status"),
                "error_code": result.get("oem_error_code"),
                "detail": {"producer": "novo_usb_can", "provenance": dict(provenance)},
            }
        ]
    return rows


def _normal_channel(row: Mapping[str, Any], result: Mapping[str, Any]) -> dict[str, Any]:
    tip = _mapping(_first(row, "tip", "hardware_tip_status"))
    pressure = _mapping(_first(row, "pressure", "hardware_pressure"))
    status = _mapping(row.get("status"))
    firmware = _mapping(row.get("firmware"))
    data = _mapping(row.get("data"))
    tip_loaded = _first(row, "tip_loaded", "software_tip_loaded")
    if tip_loaded is None:
        tip_loaded = _first(tip, "tip_loaded", "loaded")
    pressure_value = _first(row, "pressure_value", "pressure")
    if isinstance(pressure_value, Mapping):
        pressure_value = _first(_mapping(pressure_value), "value", "pressure")
    if pressure_value is None:
        pressure_value = _first(pressure, "value", "pressure")
    units = _first(row, "pressure_units", "units")
    if units is None:
        units = pressure.get("units")
    status_value = _first(row, "status_code", "status_name")
    if status_value is None:
        status_value = _first(status, "code", "name", "status")
    if status_value is None and row.get("status") is not None and not status:
        status_value = row.get("status")
    error_code = _first(row, "error_code", "oem_error_code")
    if error_code is None:
        error_code = _first(status, "error_code", "code")
    semantic = row.get("semantic_validity")
    if semantic is None:
        semantic_ok = row.get("semantic_ok")
        correlated = row.get("query_response_correlated")
        nested_result = row.get("result")
        if semantic_ok is None and isinstance(nested_result, Mapping):
            semantic_ok = nested_result.get("semantic_ok")
            correlated = nested_result.get("query_response_correlated")
        semantic = "valid" if semantic_ok is True and correlated is True else "unknown"
    if tip_loaded is not None and type(tip_loaded) is not bool:
        raise PipetteAuditIntegrityError("tip_loaded must be an exact boolean")
    if pressure_value is not None and type(pressure_value) not in {int, float}:
        raise PipetteAuditIntegrityError("pressure must be an exact number")
    detail = dict(_mapping(row.get("detail")))
    if firmware:
        detail["firmware"] = dict(firmware)
    if data:
        detail["data"] = dict(data)
    firmware_class = row.get("firmware_class")
    if firmware_class is None:
        firmware_class = _first(firmware, "class", "version", "firmware_class")
    return {
        "channel": int(row["channel"]),
        "phase": str(row.get("phase") or "query"),
        "semantic_validity": str(semantic),
        "truth_source": str(row.get("truth_source") or "novo_router"),
        "tip_loaded": tip_loaded,
        "pressure": None if pressure_value is None else float(pressure_value),
        "pressure_units": None if units is None else str(units),
        "status": None if status_value is None else str(status_value),
        "error_code": None if error_code is None else int(error_code),
        "firmware_class": None if firmware_class is None else str(firmware_class),
        "detail": detail,
    }


def _exchange_rows(result: Mapping[str, Any], provenance: Mapping[str, Any]) -> list[dict[str, Any]]:
    def nested_exchanges(value: Any, *, channel: Any, depth: int = 0) -> list[dict[str, Any]]:
        if depth > 12:
            raise PipetteAuditIntegrityError("pipette exchange nesting exceeds the bound")
        found: list[dict[str, Any]] = []
        if isinstance(value, Mapping):
            candidate = _mapping(value.get("provenance"))
            if candidate.get("tx_id") is not None:
                row = dict(candidate)
                row.setdefault("channel", channel)
                row["delivery_verified"] = bool(
                    value.get("delivery_verified") is True
                    or value.get("tx_write_completed_at") is not None
                    or candidate.get("tx_write_completed_at") is not None
                )
                row["controller_acknowledged"] = bool(
                    value.get("controller_acknowledged") is True
                    or value.get("immediate_ack_received") is True
                )
                row["completion_verified"] = bool(
                    value.get("completion_verified") is True
                    or value.get("completion_received") is True
                )
                terminal = _mapping(value.get("completion"))
                if terminal:
                    row["observed_rx_id"] = terminal.get("observed_rx_id")
                    row["rx_dlc"] = terminal.get("observed_rx_dlc")
                    row["rx_data"] = list(
                        terminal.get("data")
                        or terminal.get("observed_rx_raw")
                        or []
                    )
                    row["observed_rx_raw"] = list(terminal.get("observed_rx_raw") or [])
                    row["receive_timestamp"] = terminal.get("receive_timestamp")
                    row["received_at"] = terminal.get("receive_timestamp")
                    row["completion_at"] = terminal.get("receive_timestamp")
                    row["completion_command_name"] = terminal.get("command_name")
                row["transaction_phase"] = (
                    "completion"
                    if terminal
                    else "ack"
                    if row["controller_acknowledged"]
                    else "query_response"
                    if value.get("query_response_correlated") is True
                    else "tx"
                )
                row.setdefault(
                    "semantic_match",
                    value.get("query_response_correlated") is True
                    and value.get("semantic_ok") is True,
                )
                found.append(row)
            for key, nested in value.items():
                if key not in {"provenance", "raw", "tx_raw", "observed_rx_raw"}:
                    found.extend(nested_exchanges(nested, channel=channel, depth=depth + 1))
        elif isinstance(value, (list, tuple)):
            for nested in value:
                found.extend(nested_exchanges(nested, channel=channel, depth=depth + 1))
        return found

    rows = _as_items(result.get("transport_exchanges"))
    if not rows:
        rows = _as_items(result.get("exchanges"))
    if not rows:
        for channel_row in _as_items(result.get("channels")):
            channel = channel_row.get("channel")
            payload = _flatten_query_row(_mapping(channel_row.get("result")))
            rows.extend(nested_exchanges(payload, channel=channel))
    if not rows:
        for send in _as_items(result.get("sends")):
            payload = _flatten_query_row(_mapping(send.get("result")))
            rows.extend(nested_exchanges(payload, channel=send.get("channel")))
    if not rows:
        for item in _as_items(result.get("results")):
            payload = _flatten_query_row(item)
            rows.extend(nested_exchanges(payload, channel=payload.get("channel")))
    if not rows and isinstance(result.get("result"), Mapping):
        rows.extend(nested_exchanges(_flatten_query_row(result), channel=result.get("channel")))
    if not rows:
        for candidate in (result.get("driver_result"), result.get("last_transaction")):
            rows.extend(nested_exchanges(candidate, channel=result.get("channel")))
            if rows:
                break
    if not rows and provenance.get("tx_id") is not None:
        fallback = dict(provenance)
        fallback["delivery_verified"] = provenance.get("tx_write_completed_at") is not None
        fallback["controller_acknowledged"] = False
        fallback["completion_verified"] = False
        fallback["transaction_phase"] = "tx"
        rows = [fallback]
    distinct: list[dict[str, Any]] = []
    seen: set[tuple[Any, ...]] = set()
    for row in rows:
        key = (
            row.get("transaction_id"),
            row.get("tx_id"),
            row.get("channel"),
            row.get("receive_timestamp"),
            tuple(row.get("observed_rx_raw") or row.get("rx_data") or ()),
        )
        if key not in seen:
            seen.add(key)
            distinct.append(dict(row))
    return distinct


def _normal_exchange(row: Mapping[str, Any], result: Mapping[str, Any]) -> dict[str, Any]:
    tx_id = row.get("tx_id")
    expected = row.get("expected_rx_id")
    if tx_id is not None:
        tx_id = int(tx_id)
        derived = tx_id | 0x400
        if expected is None:
            expected = derived
        if int(expected) != derived:
            raise PipetteAuditIntegrityError("expected_rx_id must equal tx_id | 0x400")
    observed = row.get("observed_rx_id")
    ack = row.get("controller_acknowledged")
    if not isinstance(ack, bool):
        ack = False
    completion = row.get("completion_verified")
    if not isinstance(completion, bool):
        completion = False
    completion_before_ack = bool(completion and not ack)
    phase = row.get("transaction_phase")
    if phase is None:
        phase = "completion" if completion else "ack" if ack else "tx"
    return {
        "channel": row.get("channel"),
        "transaction_id": row.get("transaction_id"),
        "transaction_phase": str(phase),
        "command_family": row.get("command_family"),
        "matcher_name": row.get("matcher_name"),
        "tx_id": tx_id,
        "tx_dlc": row.get("tx_dlc"),
        "tx_bytes": list(row.get("tx_data") or row.get("tx_bytes") or [])[:64],
        "expected_rx_id": None if expected is None else int(expected),
        "observed_rx_id": None if observed is None else int(observed),
        "rx_dlc": row.get("rx_dlc", result.get("observed_rx_dlc")),
        "rx_bytes": list(
            row.get("rx_data")
            or row.get("rx_bytes")
            or row.get("observed_rx_raw")
            or result.get("observed_rx_raw")
            or []
        )[:64],
        "router_generation": row.get("router_generation"),
        "sent_at": row.get("sent_at"),
        "received_at": row.get("received_at"),
        "ack_at": row.get("ack_at"),
        "completion_at": row.get("completion_at"),
        "delivery_verified": row.get("delivery_verified") is True,
        "semantic_match": bool(
            row.get("semantic_match") is True
            or (
                row.get("query_response_correlated") is True
                and row.get("semantic_ok") is True
            )
        ),
        "controller_acknowledged": bool(ack),
        "completion_verified": bool(completion),
        "completion_before_ack": completion_before_ack,
        "multipart": _mapping(row.get("multipart")),
        "raw_exchange": dict(row),
    }


def normalize_pipette_result(result: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(result, Mapping):
        raise PipetteAuditIntegrityError("pipette result must be a mapping")
    provenance = dict(_mapping(result.get("provenance")))
    for key in ("observed_rx_id", "observed_rx_dlc", "observed_rx_raw", "receive_timestamp"):
        if key not in provenance and result.get(key) is not None:
            provenance[key] = result[key]
    inherited = dict(result)
    for key in (
        "duplicate_terminal_count",
        "late_completion",
        "event_error_code",
        "q1_error_code",
        "control_lib_error_event",
        "pipette_error",
        "callback_error",
    ):
        if inherited.get(key) is None and provenance.get(key) is not None:
            inherited[key] = provenance[key]
    result = inherited
    channels = [_normal_channel(row, result) for row in _channel_rows(result, provenance) if row.get("channel") is not None]
    offset_evidence = _mapping(result.get("pressure_offset_evidence"))
    pressure_offset_samples: list[dict[str, Any]] = []
    for raw_channel, raw_evidence in offset_evidence.items():
        evidence = _mapping(raw_evidence)
        channel = int(raw_channel)
        valid = evidence.get("valid") is True
        offset = evidence.get("offset")
        if valid and type(offset) not in {int, float}:
            raise PipetteAuditIntegrityError("valid pressure offset requires an exact number")
        numeric_offset = (
            float(offset)
            if isinstance(offset, (int, float)) and not isinstance(offset, bool)
            else None
        )
        channels.append(
            {
                "channel": channel,
                "phase": "precondition",
                "semantic_validity": "valid" if valid else "unknown",
                "truth_source": "novo_router_pressure_epoch",
                "tip_loaded": None,
                "pressure": numeric_offset if valid else None,
                "pressure_units": str(evidence.get("units")) if isinstance(evidence.get("units"), str) else None,
                "status": "measured" if valid else "missing_samples",
                "error_code": None,
                "firmware_class": None,
                "detail": dict(evidence),
            }
        )
        if valid:
            decoded_samples = evidence.get("samples")
            if not isinstance(decoded_samples, list) or len(decoded_samples) != int(evidence.get("sample_count") or 0):
                raise PipetteAuditIntegrityError("pressure offset evidence requires every decoded sample")
            for sample_sequence, sample_value in enumerate(decoded_samples):
                if type(sample_value) not in {int, float}:
                    raise PipetteAuditIntegrityError("decoded pressure sample requires an exact number")
                pressure_offset_samples.append({
                    "channel": channel,
                    "sample_sequence": sample_sequence,
                    "value": float(sample_value),
                    "raw_pressure": float(sample_value),
                    "corrected_pressure": float(sample_value),
                    "units": str(evidence.get("units") or "controller_pressure_counts"),
                })
    exchanges = [_normal_exchange(row, result) for row in _exchange_rows(result, provenance)]
    events: list[dict[str, Any]] = []
    if result.get("completion_received") is True and result.get("controller_acknowledged") is not True:
        events.append(
            {
                "event_source": "novo_router",
                "event_kind": "completion_before_ack",
                "event_payload": {"provenance": dict(provenance), "result_outcome": result.get("outcome")},
                "channel": provenance.get("channel"),
                "transaction_id": provenance.get("transaction_id"),
                "semantic_validity": "tainted",
            }
        )
    if result.get("event_error_code") not in (None, 0):
        events.append(
            {
                "event_source": "pipette_transport",
                "event_kind": "controller_error",
                "event_payload": {"event_error_code": result.get("event_error_code"), "provenance": dict(provenance)},
                "channel": provenance.get("channel"),
                "transaction_id": provenance.get("transaction_id"),
                "semantic_validity": "valid",
            }
        )
    distinct_events = (
        ("control_lib_error_event", "ControlLib.errorEvent"),
        ("pipette_error", "pipetteError"),
        ("q1_error_code", "q1_error_state"),
        ("callback_error", "callback_delivery_failure"),
    )
    for source_key, event_kind in distinct_events:
        value = result.get(source_key)
        if value not in (None, 0, ""):
            events.append(
                {
                    "event_source": "oem_pipette",
                    "event_kind": event_kind,
                    "event_payload": {source_key: value},
                    "channel": provenance.get("channel"),
                    "transaction_id": provenance.get("transaction_id"),
                    "semantic_validity": "valid",
                }
            )
    if int(result.get("duplicate_terminal_count") or 0) > 0:
        events.append(
            {
                "event_source": "novo_router",
                "event_kind": "duplicate_completion",
                "event_payload": {"duplicate_terminal_count": int(result["duplicate_terminal_count"])},
                "channel": provenance.get("channel"),
                "transaction_id": provenance.get("transaction_id"),
                "semantic_validity": "valid",
            }
        )
    if result.get("late_completion") is True:
        events.append(
            {
                "event_source": "novo_router",
                "event_kind": "late_completion",
                "event_payload": {"late_completion": True},
                "channel": provenance.get("channel"),
                "transaction_id": provenance.get("transaction_id"),
                "semantic_validity": "tainted",
            }
        )
    return {
        "channels": channels,
        "exchanges": exchanges,
        "pressure_stream": dict(_mapping(result.get("pressure_stream"))),
        "pressure_samples": [
            *[dict(item) for item in _as_items(result.get("pressure_samples"))],
            *pressure_offset_samples,
        ],
        "pressure_chunks": [dict(item) for item in _as_items(result.get("pressure_chunks"))],
        "events": events,
        "completion_before_ack": bool(events and events[0]["event_kind"] == "completion_before_ack"),
    }
