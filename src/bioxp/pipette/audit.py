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


def _channel_rows(result: Mapping[str, Any], provenance: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = _as_items(result.get("channels"))
    if not rows:
        rows = _as_items(result.get("channel_observations"))
    if not rows:
        for send in _as_items(result.get("sends")):
            payload = dict(_mapping(send.get("result")))
            driver_result = _mapping(payload.get("driver_result"))
            row = {**payload, "channel": send.get("channel")}
            row.setdefault("hardware_tip_status", payload.get("hardware_tip_status"))
            row.setdefault("hardware_pressure", payload.get("hardware_pressure"))
            detail = dict(_mapping(row.get("detail")))
            if driver_result:
                detail["driver_result"] = driver_result
            row["detail"] = detail
            rows.append(row)
    if not rows:
        for candidate in (result.get("driver_result"), result.get("last_transaction")):
            nested = _mapping(candidate)
            nested_provenance = _mapping(nested.get("provenance"))
            if nested_provenance.get("channel") is not None or result.get("channel") is not None:
                rows.append(
                    {
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
    error_code = _first(row, "error_code", "oem_error_code")
    if error_code is None:
        error_code = _first(status, "error_code", "code")
    semantic = row.get("semantic_validity")
    if semantic is None:
        semantic = "valid" if result.get("ok") is True else "unknown"
    detail = _mapping(row.get("detail"))
    return {
        "channel": int(row["channel"]),
        "phase": str(row.get("phase") or "query"),
        "semantic_validity": str(semantic),
        "truth_source": str(row.get("truth_source") or "novo_router"),
        "tip_loaded": None if tip_loaded is None else bool(tip_loaded),
        "pressure": None if pressure_value is None else float(pressure_value),
        "pressure_units": None if units is None else str(units),
        "status": None if status_value is None else str(status_value),
        "error_code": None if error_code is None else int(error_code),
        "firmware_class": row.get("firmware_class"),
        "detail": dict(detail),
    }


def _exchange_rows(result: Mapping[str, Any], provenance: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = _as_items(result.get("transport_exchanges"))
    if not rows:
        rows = _as_items(result.get("exchanges"))
    if not rows:
        for send in _as_items(result.get("sends")):
            payload = _mapping(send.get("result"))
            candidate = _mapping(payload.get("provenance"))
            if not candidate:
                candidate = _mapping(_mapping(payload.get("driver_result")).get("provenance"))
            if candidate.get("tx_id") is not None:
                row = dict(candidate)
                row.setdefault("channel", send.get("channel"))
                for key in ("observed_rx_id", "observed_rx_dlc", "observed_rx_raw"):
                    if key not in row and payload.get(key) is not None:
                        row[key] = payload[key]
                rows.append(row)
    if not rows:
        for candidate in (result.get("driver_result"), result.get("last_transaction")):
            nested = _mapping(candidate)
            nested_provenance = _mapping(nested.get("provenance"))
            if nested_provenance.get("tx_id") is not None:
                rows = [dict(nested_provenance)]
                break
    if not rows and provenance.get("tx_id") is not None:
        rows = [dict(provenance)]
    return rows


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
        ack = result.get("controller_acknowledged") is True or result.get("ack_received") is True
    completion = row.get("completion_verified")
    if not isinstance(completion, bool):
        completion = result.get("completion_received") is True
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
        "rx_bytes": list(row.get("rx_data") or row.get("rx_bytes") or result.get("observed_rx_raw") or [])[:64],
        "router_generation": row.get("router_generation"),
        "sent_at": row.get("sent_at"),
        "received_at": row.get("received_at"),
        "ack_at": row.get("ack_at"),
        "completion_at": row.get("completion_at"),
        "delivery_verified": bool(row.get("delivery_verified", result.get("delivery_verified", False))),
        "semantic_match": bool(row.get("semantic_match", result.get("semantic_query_response_verified", False))),
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
        "pressure_samples": [dict(item) for item in _as_items(result.get("pressure_samples"))],
        "pressure_chunks": [dict(item) for item in _as_items(result.get("pressure_chunks"))],
        "events": events,
        "completion_before_ack": bool(events and events[0]["event_kind"] == "completion_before_ack"),
    }
