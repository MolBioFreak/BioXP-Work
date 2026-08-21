from __future__ import annotations

import base64
import csv
import hashlib
import io
import json
import os
import stat
import tempfile
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator, Mapping

from fastapi import APIRouter, Body, Depends, HTTPException, Query
from fastapi.responses import Response

from .operator_receipt_store import OperatorReceiptStore, _fsync_directory


_SCHEMA_VERSION = 2
_MAX_EXPORT_ROWS = 100_000
_CURSOR_TTL_S = 15 * 60


def _canonical(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), default=str)


def _decode_json(value: Any, default: Any) -> Any:
    if value is None:
        return default
    try:
        return json.loads(value)
    except (TypeError, ValueError, json.JSONDecodeError):
        return default


def _encode_cursor(value: Mapping[str, Any]) -> str:
    return base64.urlsafe_b64encode(_canonical(value).encode()).decode().rstrip("=")


def _decode_cursor(value: str) -> dict[str, Any]:
    try:
        padding = "=" * (-len(value) % 4)
        decoded = base64.urlsafe_b64decode((value + padding).encode()).decode()
        result = json.loads(decoded)
    except (ValueError, TypeError, json.JSONDecodeError, UnicodeError) as exc:
        raise HTTPException(status_code=409, detail={"error": "invalid_report_cursor"}) from exc
    if not isinstance(result, dict) or float(result.get("expires_at", 0)) < time.time():
        raise HTTPException(status_code=409, detail={"error": "expired_report_cursor"})
    return result


@contextmanager
def _read_snapshot(store: OperatorReceiptStore) -> Iterator[Any]:
    with store.lock:
        store.connection.execute("BEGIN")
        try:
            yield store.connection
        finally:
            if store.connection.in_transaction:
                store.connection.execute("ROLLBACK")


def _store_identity(connection: Any) -> dict[str, Any]:
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    identity = connection.execute(
        "SELECT database_path,schema_version FROM runtime_store_identity WHERE identity_id=1"
    ).fetchone()
    return {
        "database_identity": "robot_authoritative_sqlite",
        "schema_version": version,
        "database_path_exposed": False,
        "identity_version": None if identity is None else int(identity["schema_version"]),
    }


def _base_filters(
    *,
    start: float | None,
    end: float | None,
    status: str | None,
    operation: str | None,
    action: str | None,
    channel: int | None,
    entrypoint: str | None = None,
    caller_class: str | None = None,
    control_class: str | None = None,
    protocol_job_id: str | None = None,
    protocol_action_id: str | None = None,
    lifecycle_stage_id: str | None = None,
    outcome: str | None = None,
    event_source: str | None = None,
    event_kind: str | None = None,
    pressure_stream_id: str | None = None,
    delivery_verified: bool | None = None,
    controller_acknowledged: bool | None = None,
    completion_verified: bool | None = None,
    hardware_postcondition_verified: bool | None = None,
    physical_effect_verified: bool | None = None,
    evidence_state: str | None = None,
    command_id: str | None = None,
    pipette_operation_id: str | None = None,
    connection_generation: int | None = None,
    ownership_generation: int | None = None,
    limit: int = 100,
) -> dict[str, Any]:
    if start is not None and end is not None and start >= end:
        raise HTTPException(status_code=422, detail={"error": "invalid_time_window"})
    if start is not None and end is not None and end - start > 31 * 24 * 60 * 60:
        raise HTTPException(status_code=422, detail={"error": "time_window_exceeds_31_days"})
    if channel is not None and channel not in range(4):
        raise HTTPException(status_code=422, detail={"error": "invalid_channel"})
    return {
        "start": start,
        "end": end,
        "status": status,
        "operation": operation,
        "action": action,
        "channel": channel,
        "entrypoint": entrypoint,
        "caller_class": caller_class,
        "control_class": control_class,
        "protocol_job_id": protocol_job_id,
        "protocol_action_id": protocol_action_id,
        "lifecycle_stage_id": lifecycle_stage_id,
        "outcome": outcome,
        "event_source": event_source,
        "event_kind": event_kind,
        "pressure_stream_id": pressure_stream_id,
        "delivery_verified": delivery_verified,
        "controller_acknowledged": controller_acknowledged,
        "completion_verified": completion_verified,
        "hardware_postcondition_verified": hardware_postcondition_verified,
        "physical_effect_verified": physical_effect_verified,
        "evidence_state": evidence_state,
        "command_id": command_id,
        "pipette_operation_id": pipette_operation_id,
        "connection_generation": connection_generation,
        "ownership_generation": ownership_generation,
        "limit": limit,
    }


def _command_where(filters: Mapping[str, Any], *, high_water: int, last_sequence: int | None = None) -> tuple[str, list[Any]]:
    clauses = ["c.sequence <= ?"]
    params: list[Any] = [high_water]
    if last_sequence is not None:
        clauses.append("c.sequence < ?")
        params.append(last_sequence)
    if filters.get("start") is not None:
        clauses.append("c.updated_at >= ?")
        params.append(float(filters["start"]))
    if filters.get("end") is not None:
        clauses.append("c.updated_at < ?")
        params.append(float(filters["end"]))
    for key, column in (("status", "c.status"), ("operation", "c.operation"), ("action", "c.action_id"), ("entrypoint", "c.entrypoint_id"), ("caller_class", "c.caller_class"), ("control_class", "c.control_class"), ("outcome", "c.outcome"), ("evidence_state", "c.evidence_state"), ("command_id", "c.command_id"), ("connection_generation", "c.connection_generation"), ("ownership_generation", "c.ownership_generation")):
        if filters.get(key) is not None:
            clauses.append(f"{column} = ?")
            params.append(filters[key] if key in {"connection_generation", "ownership_generation"} else str(filters[key]))
    for key, column in (("delivery_verified", "c.delivery_verified"), ("controller_acknowledged", "c.controller_acknowledged"), ("completion_verified", "c.completion_verified"), ("hardware_postcondition_verified", "c.hardware_postcondition_verified"), ("physical_effect_verified", "c.physical_effect_verified")):
        if filters.get(key) is not None:
            clauses.append(f"{column} = ?")
            params.append(int(bool(filters[key])))
    if filters.get("channel") is not None:
        clauses.append(
            "EXISTS (SELECT 1 FROM pipette_channel_observations o WHERE o.command_id=c.command_id AND o.channel=?)"
        )
        params.append(int(filters["channel"]))
    for key, column in (("protocol_job_id", "p.protocol_job_id"), ("protocol_action_id", "p.protocol_action_id"), ("lifecycle_stage_id", "p.lifecycle_stage_id"), ("pipette_operation_id", "p.pipette_operation_id")):
        if filters.get(key) is not None:
            clauses.append(f"EXISTS (SELECT 1 FROM pipette_operations p WHERE p.command_id=c.command_id AND {column} = ?)")
            params.append(str(filters[key]))
    if filters.get("event_source") is not None:
        clauses.append("EXISTS (SELECT 1 FROM runtime_events e WHERE e.command_id=c.command_id AND e.event_source=?)")
        params.append(str(filters["event_source"]))
    if filters.get("pressure_stream_id") is not None:
        clauses.append("EXISTS (SELECT 1 FROM pipette_pressure_streams s WHERE s.pipette_operation_id=(SELECT p.pipette_operation_id FROM pipette_operations p WHERE p.command_id=c.command_id) AND s.stream_session_id=?)")
        params.append(str(filters["pressure_stream_id"]))
    return " AND ".join(clauses), params


def _evidence_rows(connection: Any, command_id: str) -> list[dict[str, Any]]:
    rows = connection.execute(
        """
        SELECT evidence_artifact_id,sha256,byte_count,created_at,retention_deadline,
               legal_hold,expiry_state,expiry_receipt_id
        FROM runtime_evidence_objects WHERE command_id=? ORDER BY created_at,evidence_artifact_id
        """,
        (command_id,),
    ).fetchall()
    return [
        {
            "evidence_artifact_id": str(row["evidence_artifact_id"]),
            "sha256": str(row["sha256"]),
            "byte_count": int(row["byte_count"]),
            "created_at": row["created_at"],
            "retention_deadline": row["retention_deadline"],
            "legal_hold": bool(row["legal_hold"]),
            "expiry_state": row["expiry_state"],
            "expiry_receipt_id": row["expiry_receipt_id"],
        }
        for row in rows
    ]


def _command_projection(connection: Any, row: Any, *, detail: bool = False) -> dict[str, Any]:
    result = {
        "sequence": int(row["sequence"]),
        "command_id": str(row["command_id"]),
        "idempotency_key": str(row["idempotency_key"]),
        "operation": row["operation"],
        "command_kind": row["command_kind"],
        "entrypoint_id": row["entrypoint_id"],
        "caller_class": row["caller_class"],
        "control_class": row["control_class"],
        "action_id": row["action_id"],
        "status": row["status"],
        "outcome": row["outcome"],
        "failure_code": row["failure_code"],
        "ownership_generation": row["ownership_generation"],
        "connection_generation": row["connection_generation"],
        "started_at": row["started_at"],
        "admitted_at": row["admitted_at"],
        "dispatched_at": row["dispatched_at"],
        "finished_at": row["finished_at"],
        "duration_ms": row["duration_ms"],
        "delivery_verified": bool(row["delivery_verified"]),
        "controller_acknowledged": bool(row["controller_acknowledged"]),
        "completion_verified": bool(row["completion_verified"]),
        "hardware_precondition_verified": bool(row["hardware_precondition_verified"]),
        "hardware_postcondition_verified": bool(row["hardware_postcondition_verified"]),
        "physical_effect_verified": bool(row["physical_effect_verified"]),
        "evidence_state": row["evidence_state"],
    }
    if detail:
        result["requested_inputs"] = _decode_json(row["requested_inputs_json"], {})
        result["effective_inputs"] = _decode_json(row["effective_inputs_json"], {})
        result["source_identity"] = _decode_json(row["source_identity_json"], {})
        result["transitions"] = [
            {
                "transition_id": int(item["transition_id"]),
                "state": item["state"],
                "observed_at": item["observed_at"],
                "detail": _decode_json(item["detail_json"], {}),
            }
            for item in connection.execute(
                "SELECT transition_id,state,observed_at,detail_json FROM operator_transitions WHERE command_id=? ORDER BY transition_id",
                (row["command_id"],),
            ).fetchall()
        ]
        result["evidence"] = _evidence_rows(connection, str(row["command_id"]))
        pipette = connection.execute(
            "SELECT * FROM pipette_operations WHERE command_id=?",
            (row["command_id"],),
        ).fetchone()
        result["pipette"] = None if pipette is None else _pipette_projection(connection, pipette, detail=True)
    return result


def _pipette_projection(connection: Any, row: Any, *, detail: bool = False) -> dict[str, Any]:
    result = {
        "pipette_operation_id": str(row["pipette_operation_id"]),
        "command_id": str(row["command_id"]),
        "operation": row["operation"],
        "entrypoint_id": row["entrypoint_id"],
        "caller_class": row["caller_class"],
        "control_class": row["control_class"],
        "action_id": row["action_id"],
        "status": row["status"],
        "outcome": row["outcome"],
        "failure_code": row["failure_code"],
        "delivery_verified": bool(row["delivery_verified"]),
        "controller_acknowledged": bool(row["controller_acknowledged"]),
        "completion_verified": bool(row["completion_verified"]),
        "hardware_postcondition_verified": bool(row["hardware_postcondition_verified"]),
        "physical_effect_verified": bool(row["physical_effect_verified"]),
        "evidence_state": row["evidence_state"],
    }
    if detail:
        op_id = str(row["pipette_operation_id"])
        result["channels"] = [
            {
                "observation_id": str(item["observation_id"]),
                "command_id": str(item["command_id"]),
                "pipette_operation_id": op_id,
                "channel": int(item["channel"]),
                "phase": item["phase"],
                "observed_at": item["observed_at"],
                "semantic_validity": item["semantic_validity"],
                "truth_source": item["truth_source"],
                "tip_loaded": None if item["tip_loaded"] is None else bool(item["tip_loaded"]),
                "pressure": item["pressure"],
                "pressure_units": item["pressure_units"],
                "status": item["status"],
                "error_code": item["error_code"],
                "firmware_class": item["firmware_class"],
                "detail": _decode_json(item["detail_json"], {}),
            }
            for item in connection.execute(
                "SELECT * FROM pipette_channel_observations WHERE pipette_operation_id=? ORDER BY observed_at,observation_id",
                (op_id,),
            ).fetchall()
        ]
        result["exchanges"] = [
            {
                "exchange_id": str(item["exchange_id"]),
                "transaction_id": item["transaction_id"],
                "channel": item["channel"],
                "transaction_phase": item["transaction_phase"],
                "command_family": item["command_family"],
                "matcher_name": item["matcher_name"],
                "tx_id": item["tx_id"],
                "expected_rx_id": item["expected_rx_id"],
                "observed_rx_id": item["observed_rx_id"],
                "tx_bytes": _decode_json(item["tx_bytes_json"], []),
                "rx_bytes": _decode_json(item["rx_bytes_json"], []),
                "delivery_verified": bool(item["delivery_verified"]),
                "semantic_match": bool(item["semantic_match"]),
                "controller_acknowledged": bool(item["controller_acknowledged"]),
                "completion_verified": bool(item["completion_verified"]),
                "completion_before_ack": bool(item["completion_before_ack"]),
                "sent_at": item["sent_at"],
                "received_at": item["received_at"],
                "ack_at": item["ack_at"],
                "completion_at": item["completion_at"],
            }
            for item in connection.execute(
                "SELECT * FROM pipette_transport_exchanges WHERE pipette_operation_id=? ORDER BY sent_at,exchange_id",
                (op_id,),
            ).fetchall()
        ]
        result["events"] = [
            {
                "event_id": int(item["event_id"]),
                "event_source": item["event_source"],
                "event_kind": item["event_kind"],
                "observed_at": item["observed_at"],
                "event": _decode_json(item["event_json"], {}),
            }
            for item in connection.execute(
                "SELECT * FROM runtime_events WHERE pipette_operation_id=? ORDER BY event_id",
                (op_id,),
            ).fetchall()
        ]
        result["pressure_streams"] = [
            {
                "stream_session_id": item["stream_session_id"],
                "channels": _decode_json(item["channels_json"], []),
                "sample_period_ms": item["sample_period_ms"],
                "started_at": item["started_at"],
                "stopped_at": item["stopped_at"],
                "source_generation": item["source_generation"],
                "reader_generation": item["reader_generation"],
                "offset_identity": item["offset_identity"],
                "terminal_state": item["terminal_state"],
                "loss_count": item["loss_count"],
            }
            for item in connection.execute(
                "SELECT * FROM pipette_pressure_streams WHERE pipette_operation_id=? ORDER BY started_at,stream_session_id",
                (op_id,),
            ).fetchall()
        ]
    return result


def _command_page(connection: Any, filters: dict[str, Any], cursor: str | None) -> dict[str, Any]:
    high_water = int(connection.execute("SELECT COALESCE(MAX(sequence),0) FROM operator_commands").fetchone()[0])
    filter_digest = hashlib.sha256(_canonical(filters).encode()).hexdigest()
    last_sequence = None
    if cursor:
        state = _decode_cursor(cursor)
        if state.get("resource") != "commands" or state.get("filter_sha256") != filter_digest:
            raise HTTPException(status_code=409, detail={"error": "cursor_filter_mismatch"})
        high_water = int(state["high_water"])
        last_sequence = int(state["last_sequence"])
    where, params = _command_where(filters, high_water=high_water, last_sequence=last_sequence)
    total = int(connection.execute(f"SELECT COUNT(*) FROM operator_commands c WHERE {where}", params).fetchone()[0])
    rows = connection.execute(
        f"SELECT c.* FROM operator_commands c WHERE {where} ORDER BY c.sequence DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "filters": filters,
        "snapshot": {"high_water_sequence": high_water, **_store_identity(connection)},
        "returned_count": len(rows),
        "filtered_total": total,
        "has_more": has_more,
        "next_cursor": None,
        "commands": [_command_projection(connection, row) for row in rows],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor({
            "resource": "commands",
            "filter_sha256": filter_digest,
            "high_water": high_water,
            "last_sequence": int(rows[-1]["sequence"]),
            "expires_at": time.time() + _CURSOR_TTL_S,
        })
    return body


def _pipette_page(connection: Any, filters: dict[str, Any], cursor: str | None) -> dict[str, Any]:
    clauses = ["p.rowid <= (SELECT COALESCE(MAX(rowid),0) FROM pipette_operations)"]
    params: list[Any] = []
    for key, column in (("status", "p.status"), ("operation", "p.operation"), ("action", "p.action_id"), ("entrypoint", "p.entrypoint_id"), ("caller_class", "p.caller_class"), ("control_class", "p.control_class"), ("protocol_job_id", "p.protocol_job_id"), ("protocol_action_id", "p.protocol_action_id"), ("lifecycle_stage_id", "p.lifecycle_stage_id"), ("outcome", "p.outcome"), ("evidence_state", "p.evidence_state"), ("pipette_operation_id", "p.pipette_operation_id"), ("connection_generation", "p.connection_generation"), ("ownership_generation", "p.ownership_generation")):
        if filters.get(key) is not None:
            clauses.append(f"{column}=?")
            params.append(filters[key] if key in {"connection_generation", "ownership_generation"} else str(filters[key]))
    if filters.get("command_id") is not None:
        clauses.append("p.command_id=?")
        params.append(str(filters["command_id"]))
    for key, column in (("delivery_verified", "p.delivery_verified"), ("controller_acknowledged", "p.controller_acknowledged"), ("completion_verified", "p.completion_verified"), ("hardware_postcondition_verified", "p.hardware_postcondition_verified"), ("physical_effect_verified", "p.physical_effect_verified")):
        if filters.get(key) is not None:
            clauses.append(f"{column}=?")
            params.append(int(bool(filters[key])))
    if filters.get("event_source") is not None:
        clauses.append("EXISTS (SELECT 1 FROM runtime_events e WHERE e.pipette_operation_id=p.pipette_operation_id AND e.event_source=?)")
        params.append(str(filters["event_source"]))
    if filters.get("channel") is not None:
        clauses.append("EXISTS (SELECT 1 FROM pipette_channel_observations o WHERE o.pipette_operation_id=p.pipette_operation_id AND o.channel=?)")
        params.append(filters["channel"])
    if filters.get("pressure_stream_id") is not None:
        clauses.append("EXISTS (SELECT 1 FROM pipette_pressure_streams s WHERE s.pipette_operation_id=p.pipette_operation_id AND s.stream_session_id=?)")
        params.append(str(filters["pressure_stream_id"]))
    last_rowid = None
    filter_digest = hashlib.sha256(_canonical(filters).encode()).hexdigest()
    high_water = int(connection.execute("SELECT COALESCE(MAX(rowid),0) FROM pipette_operations").fetchone()[0])
    if cursor:
        state = _decode_cursor(cursor)
        if state.get("resource") != "pipette" or state.get("filter_sha256") != filter_digest:
            raise HTTPException(status_code=409, detail={"error": "cursor_filter_mismatch"})
        high_water = int(state["high_water"])
        last_rowid = int(state["last_rowid"])
        clauses.append("p.rowid < ?")
        params.append(last_rowid)
    if filters.get("start") is not None:
        clauses.append("p.updated_at >= ?")
        params.append(float(filters["start"]))
    if filters.get("end") is not None:
        clauses.append("p.updated_at < ?")
        params.append(float(filters["end"]))
    where = " AND ".join(clauses)
    total = int(connection.execute(f"SELECT COUNT(*) FROM pipette_operations p WHERE {where}", params).fetchone()[0])
    rows = connection.execute(
        f"SELECT p.* FROM pipette_operations p WHERE {where} ORDER BY p.rowid DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "filters": filters,
        "snapshot": {"high_water_rowid": high_water, **_store_identity(connection)},
        "returned_count": len(rows),
        "filtered_total": total,
        "has_more": has_more,
        "next_cursor": None,
        "pipette": [_pipette_projection(connection, row) for row in rows],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor({
            "resource": "pipette",
            "filter_sha256": filter_digest,
            "high_water": high_water,
            "last_rowid": int(rows[-1]["rowid"]),
            "expires_at": time.time() + _CURSOR_TTL_S,
        })
    return body


def _summary(connection: Any, filters: dict[str, Any]) -> dict[str, Any]:
    high_water = int(connection.execute("SELECT COALESCE(MAX(sequence),0) FROM operator_commands").fetchone()[0])
    where, params = _command_where(filters, high_water=high_water)
    total = int(connection.execute(f"SELECT COUNT(*) FROM operator_commands c WHERE {where}", params).fetchone()[0])
    statuses = {
        str(row["status"]): int(row["count"])
        for row in connection.execute(f"SELECT c.status,COUNT(*) AS count FROM operator_commands c WHERE {where} GROUP BY c.status", params).fetchall()
    }
    aggregate = connection.execute(
        f"""
        SELECT
            COUNT(*) AS total,
            COALESCE(SUM(c.delivery_verified), 0) AS delivered,
            COALESCE(SUM(c.controller_acknowledged), 0) AS acknowledged,
            COALESCE(SUM(c.completion_verified), 0) AS completed,
            COALESCE(SUM(c.hardware_postcondition_verified), 0) AS postcondition,
            COALESCE(SUM(c.physical_effect_verified), 0) AS physical_effect,
            COALESCE(SUM(CASE WHEN c.failure_code IS NOT NULL OR c.status IN ('failed','blocked','rejected') THEN 1 ELSE 0 END), 0) AS failures,
            COALESCE(AVG(CASE WHEN c.duration_ms IS NOT NULL THEN c.duration_ms END), 0.0) AS average_ms,
            COALESCE(MAX(CASE WHEN c.duration_ms IS NOT NULL THEN c.duration_ms ELSE 0 END), 0.0) AS maximum_ms
        FROM operator_commands c WHERE {where}
        """,
        params,
    ).fetchone()
    total_for_rates = max(1, int(aggregate["total"]))
    error_rows = connection.execute(
        f"SELECT COALESCE(c.failure_code, c.outcome, 'unknown') AS code, COUNT(*) AS count FROM operator_commands c WHERE {where} AND (c.failure_code IS NOT NULL OR c.status IN ('failed','blocked','rejected')) GROUP BY code ORDER BY code",
        params,
    ).fetchall()
    errors = {str(row["code"]): int(row["count"]) for row in error_rows}
    scope_keys = tuple(key for key in filters if key != "limit")
    scope = "filtered" if any(filters.get(key) is not None for key in scope_keys) else "window"
    pipette_total = int(connection.execute(
        f"SELECT COUNT(*) FROM pipette_operations p WHERE EXISTS (SELECT 1 FROM operator_commands c WHERE c.command_id=p.command_id AND {where})",
        params,
    ).fetchone()[0])
    event_total = int(connection.execute(
        f"SELECT COUNT(*) FROM runtime_events e WHERE EXISTS (SELECT 1 FROM operator_commands c WHERE c.command_id=e.command_id AND {where})",
        params,
    ).fetchone()[0])
    pressure_stream_total = int(connection.execute(
        f"SELECT COUNT(*) FROM pipette_pressure_streams s WHERE EXISTS (SELECT 1 FROM pipette_operations p WHERE p.pipette_operation_id=s.pipette_operation_id AND EXISTS (SELECT 1 FROM operator_commands c WHERE c.command_id=p.command_id AND {where}))",
        params,
    ).fetchone()[0])
    pressure_chunk_total = int(connection.execute(
        f"SELECT COUNT(*) FROM pipette_pressure_chunks pc WHERE EXISTS (SELECT 1 FROM pipette_pressure_streams s WHERE s.stream_session_id=pc.stream_session_id AND EXISTS (SELECT 1 FROM pipette_operations p WHERE p.pipette_operation_id=s.pipette_operation_id AND EXISTS (SELECT 1 FROM operator_commands c WHERE c.command_id=p.command_id AND {where})))",
        params,
    ).fetchone()[0])
    return {
        "scope": scope,
        "filters": filters,
        "snapshot": {"high_water_sequence": high_water, **_store_identity(connection)},
        "commands": {"total": total, "by_status": statuses},
        "pipette_operations": {"total": pipette_total},
        "runtime_events": {"total": event_total},
        "pressure": {"streams": pressure_stream_total, "chunks": pressure_chunk_total},
        "rates": {
            "delivery_rate": float(aggregate["delivered"]) / total_for_rates,
            "ack_rate": float(aggregate["acknowledged"]) / total_for_rates,
            "completion_rate": float(aggregate["completed"]) / total_for_rates,
            "postcondition_rate": float(aggregate["postcondition"]) / total_for_rates,
            "physical_effect_rate": float(aggregate["physical_effect"]) / total_for_rates,
            "failure_rate": float(aggregate["failures"]) / total_for_rates,
        },
        "latency": {"average_ms": float(aggregate["average_ms"]), "maximum_ms": float(aggregate["maximum_ms"])},
        "errors": {"by_code": errors},
    }


def _pressure_stream_page(connection: Any, filters: dict[str, Any], cursor: str | None) -> dict[str, Any]:
    high_water = int(connection.execute("SELECT COALESCE(MAX(rowid),0) FROM pipette_pressure_streams").fetchone()[0])
    filter_digest = hashlib.sha256(_canonical(filters).encode()).hexdigest()
    clauses = ["s.rowid <= ?"]
    params: list[Any] = [high_water]
    if cursor:
        state = _decode_cursor(cursor)
        if state.get("resource") != "pressure_streams" or state.get("filter_sha256") != filter_digest:
            raise HTTPException(status_code=409, detail={"error": "cursor_filter_mismatch"})
        high_water = int(state["high_water"])
        clauses.append("s.rowid < ?")
        params.append(int(state["last_rowid"]))
    if filters.get("start") is not None:
        clauses.append("s.started_at >= ?")
        params.append(float(filters["start"]))
    if filters.get("end") is not None:
        clauses.append("s.started_at < ?")
        params.append(float(filters["end"]))
    if filters.get("pressure_stream_id") is not None:
        clauses.append("s.stream_session_id = ?")
        params.append(str(filters["pressure_stream_id"]))
    if filters.get("channel") is not None:
        clauses.append("EXISTS (SELECT 1 FROM json_each(s.channels_json) WHERE CAST(json_each.value AS INTEGER)=?)")
        params.append(int(filters["channel"]))
    command_high_water = int(connection.execute("SELECT COALESCE(MAX(sequence),0) FROM operator_commands").fetchone()[0])
    command_where, command_params = _command_where(filters, high_water=command_high_water)
    clauses.append(f"EXISTS (SELECT 1 FROM pipette_operations p WHERE p.pipette_operation_id=s.pipette_operation_id AND EXISTS (SELECT 1 FROM operator_commands c WHERE c.command_id=p.command_id AND {command_where}))")
    params.extend(command_params)
    where = " AND ".join(clauses)
    rows = connection.execute(
        f"SELECT s.* FROM pipette_pressure_streams s WHERE {where} ORDER BY s.rowid DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "filters": filters,
        "snapshot": {"high_water_rowid": high_water, **_store_identity(connection)},
        "returned_count": len(rows),
        "has_more": has_more,
        "next_cursor": None,
        "pressure_streams": [
            {
                "stream_session_id": row["stream_session_id"],
                "pipette_operation_id": row["pipette_operation_id"],
                "channels": _decode_json(row["channels_json"], []),
                "sample_period_ms": row["sample_period_ms"],
                "started_at": row["started_at"],
                "stopped_at": row["stopped_at"],
                "source_generation": row["source_generation"],
                "reader_generation": row["reader_generation"],
                "offset_identity": row["offset_identity"],
                "terminal_state": row["terminal_state"],
                "loss_count": row["loss_count"],
            }
            for row in rows
        ],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor({
            "resource": "pressure_streams",
            "filter_sha256": filter_digest,
            "high_water": high_water,
            "last_rowid": int(rows[-1]["rowid"]),
            "expires_at": time.time() + _CURSOR_TTL_S,
        })
    return body


def _pressure_samples_page(connection: Any, stream_session_id: str, filters: dict[str, Any], cursor: str | None) -> dict[str, Any]:
    high_water = int(connection.execute("SELECT COALESCE(MAX(rowid),0) FROM pipette_pressure_chunks WHERE stream_session_id=?", (stream_session_id,)).fetchone()[0])
    filter_digest = hashlib.sha256(_canonical({"stream_session_id": stream_session_id, **filters}).encode()).hexdigest()
    clauses = ["c.stream_session_id=?", "c.rowid <= ?"]
    params: list[Any] = [stream_session_id, high_water]
    if cursor:
        state = _decode_cursor(cursor)
        if state.get("resource") != "pressure_samples" or state.get("filter_sha256") != filter_digest:
            raise HTTPException(status_code=409, detail={"error": "cursor_filter_mismatch"})
        high_water = int(state["high_water"])
        clauses.append("c.rowid < ?")
        params.append(int(state["last_rowid"]))
    if filters.get("channel") is not None:
        clauses.append("c.channel=?")
        params.append(int(filters["channel"]))
    command_high_water = int(connection.execute("SELECT COALESCE(MAX(sequence),0) FROM operator_commands").fetchone()[0])
    command_where, command_params = _command_where(filters, high_water=command_high_water)
    clauses.append(f"EXISTS (SELECT 1 FROM pipette_pressure_streams s WHERE s.stream_session_id=c.stream_session_id AND EXISTS (SELECT 1 FROM pipette_operations p WHERE p.pipette_operation_id=s.pipette_operation_id AND EXISTS (SELECT 1 FROM operator_commands oc WHERE oc.command_id=p.command_id AND {command_where})))")
    params.extend(command_params)
    rows = connection.execute(
        f"SELECT c.* FROM pipette_pressure_chunks c WHERE {' AND '.join(clauses)} ORDER BY c.rowid DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "stream_session_id": stream_session_id,
        "filters": filters,
        "snapshot": {"high_water_rowid": high_water, **_store_identity(connection)},
        "returned_count": len(rows),
        "has_more": has_more,
        "next_cursor": None,
        "samples": [
            {
                "chunk_id": row["chunk_id"],
                "channel": row["channel"],
                "chunk_sequence": row["chunk_sequence"],
                "sample_count": row["sample_count"],
                "lost_sample_count": row["lost_sample_count"],
                "units": row["units"],
                "sha256": row["sha256"],
                "byte_count": row["byte_count"],
                "evidence_artifact_id": row["evidence_artifact_id"],
                "summary": _decode_json(row["sample_summary_json"], {}),
            }
            for row in rows
        ],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor({
            "resource": "pressure_samples",
            "filter_sha256": filter_digest,
            "high_water": high_water,
            "last_rowid": int(rows[-1]["rowid"]),
            "expires_at": time.time() + _CURSOR_TTL_S,
        })
    return body


def _event_page(connection: Any, filters: dict[str, Any], cursor: str | None) -> dict[str, Any]:
    high_water = int(connection.execute("SELECT COALESCE(MAX(event_id),0) FROM runtime_events").fetchone()[0])
    filter_digest = hashlib.sha256(_canonical(filters).encode()).hexdigest()
    clauses = ["e.event_id <= ?"]
    params: list[Any] = [high_water]
    last_event_id = None
    if cursor:
        state = _decode_cursor(cursor)
        if state.get("resource") != "events" or state.get("filter_sha256") != filter_digest:
            raise HTTPException(status_code=409, detail={"error": "cursor_filter_mismatch"})
        high_water = int(state["high_water"])
        last_event_id = int(state["last_event_id"])
        clauses[0] = "e.event_id <= ? AND e.event_id < ?"
        params = [high_water, last_event_id]
    if filters.get("start") is not None:
        clauses.append("e.observed_at >= ?")
        params.append(float(filters["start"]))
    if filters.get("end") is not None:
        clauses.append("e.observed_at < ?")
        params.append(float(filters["end"]))
    for key, column in (("event_kind", "e.event_kind"), ("event_source", "e.event_source"), ("command_id", "e.command_id"), ("pipette_operation_id", "e.pipette_operation_id")):
        if filters.get(key) is not None:
            clauses.append(f"{column}=?")
            params.append(str(filters[key]))
    if filters.get("channel") is not None:
        clauses.append("json_extract(e.event_json, '$.channel') = ?")
        params.append(int(filters["channel"]))
    command_high_water = int(connection.execute("SELECT COALESCE(MAX(sequence),0) FROM operator_commands").fetchone()[0])
    command_where, command_params = _command_where(filters, high_water=command_high_water)
    clauses.append(f"EXISTS (SELECT 1 FROM operator_commands c WHERE c.command_id=e.command_id AND {command_where})")
    params.extend(command_params)
    where = " AND ".join(clauses)
    rows = connection.execute(
        f"SELECT * FROM runtime_events e WHERE {where} ORDER BY e.event_id DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "event_kind": filters.get("event_kind"),
        "filters": filters,
        "snapshot": {"high_water_event_id": high_water, **_store_identity(connection)},
        "returned_count": len(rows),
        "has_more": has_more,
        "next_cursor": None,
        "events": [
            {
                "event_id": int(row["event_id"]),
                "command_id": row["command_id"],
                "pipette_operation_id": row["pipette_operation_id"],
                "event_source": row["event_source"],
                "event_kind": row["event_kind"],
                "observed_at": row["observed_at"],
                "event": _decode_json(row["event_json"], {}),
            }
            for row in rows
        ],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor({
            "resource": "events",
            "filter_sha256": filter_digest,
            "high_water": high_water,
            "last_event_id": int(rows[-1]["event_id"]),
            "expires_at": time.time() + _CURSOR_TTL_S,
        })
    return body


def _write_export(store: OperatorReceiptStore, *, export_id: str, fmt: str, payload: Mapping[str, Any]) -> tuple[str, int, str]:
    export_root = store.root / "report_exports"
    export_root.mkdir(parents=True, exist_ok=True, mode=0o700)
    relpath = Path("report_exports") / f"{export_id}.{fmt}"
    final_path = store.root / relpath
    if fmt == "json":
        raw = (_canonical(payload) + "\n").encode()
    else:
        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=["command_id", "operation", "status", "outcome", "failure_code"])
        writer.writeheader()
        for row in payload.get("commands", []):
            writer.writerow({key: row.get(key) for key in writer.fieldnames})
        raw = output.getvalue().encode()
    digest = hashlib.sha256(raw).hexdigest()
    fd, temporary_name = tempfile.mkstemp(prefix=f".{export_id}.", suffix=".tmp", dir=export_root)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(raw)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, final_path)
        _fsync_directory(export_root)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return relpath.as_posix(), len(raw), digest


def _read_export_artifact(store: OperatorReceiptStore, row: Any) -> bytes:
    relative = Path(str(row["artifact_relpath"]))
    if relative.is_absolute() or not relative.parts or any(part in {"", ".", ".."} for part in relative.parts):
        raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"})
    root_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0)
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    root_fd = None
    current_fd = None
    try:
        root_fd = os.open(str(store.root), root_flags)
        current_fd = root_fd
        for index, component in enumerate(relative.parts):
            flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | nofollow
            if index < len(relative.parts) - 1:
                flags |= getattr(os, "O_DIRECTORY", 0)
            next_fd = os.open(component, flags, dir_fd=current_fd)
            if current_fd != root_fd:
                os.close(current_fd)
            current_fd = next_fd
        before = os.fstat(current_fd)
        if not stat.S_ISREG(before.st_mode):
            raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"})
        chunks: list[bytes] = []
        while True:
            chunk = os.read(current_fd, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        content = b"".join(chunks)
        after = os.fstat(current_fd)
        if before.st_size != after.st_size or len(content) != int(row["byte_count"]):
            raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"})
        if hashlib.sha256(content).hexdigest() != str(row["sha256"]):
            raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"})
        return content
    except HTTPException:
        raise
    except FileNotFoundError as exc:
        raise HTTPException(status_code=409, detail={"error": "export_artifact_missing"}) from exc
    except OSError as exc:
        raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"}) from exc
    finally:
        if current_fd is not None and current_fd != root_fd:
            os.close(current_fd)
        if root_fd is not None:
            os.close(root_fd)


def create_operator_reports_router(store: Any) -> APIRouter:
    router = APIRouter(prefix="/operator", tags=["operator-reports"])

    def filters(
        start: float | None = None,
        end: float | None = None,
        status: str | None = None,
        operation: str | None = None,
        action: str | None = None,
        channel: int | None = Query(default=None, ge=0, le=3),
        entrypoint: str | None = None,
        caller_class: str | None = None,
        control_class: str | None = None,
        protocol_job_id: str | None = None,
        protocol_action_id: str | None = None,
        lifecycle_stage_id: str | None = None,
        outcome: str | None = None,
        event_source: str | None = None,
        event_kind: str | None = None,
        pressure_stream_id: str | None = None,
        delivery_verified: bool | None = None,
        controller_acknowledged: bool | None = None,
        completion_verified: bool | None = None,
        hardware_postcondition_verified: bool | None = None,
        physical_effect_verified: bool | None = None,
        evidence_state: str | None = None,
        command_id: str | None = None,
        pipette_operation_id: str | None = None,
        connection_generation: int | None = Query(default=None, ge=0),
        ownership_generation: int | None = Query(default=None, ge=0),
        limit: int = Query(100, ge=1, le=1000),
    ) -> dict[str, Any]:
        return _base_filters(
            start=start, end=end, status=status, operation=operation, action=action, channel=channel,
            entrypoint=entrypoint, caller_class=caller_class, control_class=control_class,
            protocol_job_id=protocol_job_id, protocol_action_id=protocol_action_id,
            lifecycle_stage_id=lifecycle_stage_id, outcome=outcome, event_source=event_source,
            event_kind=event_kind, pressure_stream_id=pressure_stream_id, delivery_verified=delivery_verified,
            controller_acknowledged=controller_acknowledged, completion_verified=completion_verified,
            hardware_postcondition_verified=hardware_postcondition_verified,
            physical_effect_verified=physical_effect_verified, evidence_state=evidence_state,
            command_id=command_id, pipette_operation_id=pipette_operation_id,
            connection_generation=connection_generation, ownership_generation=ownership_generation,
            limit=limit,
        )

    @router.get("/reports/summary")
    def report_summary(
        selected: dict[str, Any] = Depends(filters),
    ) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            return _summary(connection, selected)

    @router.get("/reports/commands")
    def report_commands(
        selected: dict[str, Any] = Depends(filters),
        cursor: str | None = None,
    ) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            return _command_page(connection, selected, cursor)

    @router.get("/reports/commands/{command_id}")
    def report_command_detail(command_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM operator_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "command_not_found", "command_id": command_id})
            return _command_projection(connection, row, detail=True)

    @router.get("/reports/commands/{command_id}/transitions")
    def report_command_transitions(command_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            rows = connection.execute("SELECT transition_id,state,observed_at,detail_json FROM operator_transitions WHERE command_id=? ORDER BY transition_id", (command_id,)).fetchall()
            return {"command_id": command_id, "transitions": [{"transition_id": int(row["transition_id"]), "state": row["state"], "observed_at": row["observed_at"], "detail": _decode_json(row["detail_json"], {})} for row in rows]}

    @router.get("/reports/pipette")
    def report_pipette(
        selected: dict[str, Any] = Depends(filters),
        cursor: str | None = None,
    ) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            return _pipette_page(connection, selected, cursor)

    @router.get("/reports/pipette/{pipette_operation_id}")
    def report_pipette_detail(pipette_operation_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM pipette_operations WHERE pipette_operation_id=?", (pipette_operation_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "pipette_operation_not_found", "pipette_operation_id": pipette_operation_id})
            return _pipette_projection(connection, row, detail=True)

    @router.get("/reports/pipette/{pipette_operation_id}/channels")
    def report_pipette_channels(pipette_operation_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM pipette_operations WHERE pipette_operation_id=?", (pipette_operation_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "pipette_operation_not_found"})
            return {"pipette_operation_id": pipette_operation_id, "channels": _pipette_projection(connection, row, detail=True)["channels"]}

    @router.get("/reports/pipette/{pipette_operation_id}/exchanges")
    def report_pipette_exchanges(pipette_operation_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM pipette_operations WHERE pipette_operation_id=?", (pipette_operation_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "pipette_operation_not_found"})
            return {"pipette_operation_id": pipette_operation_id, "exchanges": _pipette_projection(connection, row, detail=True)["exchanges"]}

    @router.get("/reports/events")
    def report_events(selected: dict[str, Any] = Depends(filters), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            return _event_page(connection, selected, cursor)

    @router.get("/reports/events/{event_id}")
    def report_event_detail(event_id: int) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM runtime_events WHERE event_id=?", (event_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "event_not_found"})
            return {"event_id": int(row["event_id"]), "command_id": row["command_id"], "pipette_operation_id": row["pipette_operation_id"], "event_source": row["event_source"], "event_kind": row["event_kind"], "observed_at": row["observed_at"], "event": _decode_json(row["event_json"], {})}

    @router.get("/reports/pressure-streams")
    def report_pressure_streams(selected: dict[str, Any] = Depends(filters), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            return _pressure_stream_page(connection, selected, cursor)

    @router.get("/reports/pressure-streams/{stream_session_id}")
    def report_pressure_stream_detail(stream_session_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM pipette_pressure_streams WHERE stream_session_id=?", (stream_session_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "pressure_stream_not_found"})
            chunks = connection.execute("SELECT * FROM pipette_pressure_chunks WHERE stream_session_id=? ORDER BY channel,chunk_sequence", (stream_session_id,)).fetchall()
            return {"stream_session_id": row["stream_session_id"], "pipette_operation_id": row["pipette_operation_id"], "channels": _decode_json(row["channels_json"], []), "sample_period_ms": row["sample_period_ms"], "started_at": row["started_at"], "stopped_at": row["stopped_at"], "terminal_state": row["terminal_state"], "loss_count": row["loss_count"], "chunks": [{"chunk_id": c["chunk_id"], "channel": c["channel"], "chunk_sequence": c["chunk_sequence"], "sample_count": c["sample_count"], "lost_sample_count": c["lost_sample_count"], "units": c["units"], "sha256": c["sha256"], "byte_count": c["byte_count"], "evidence_artifact_id": c["evidence_artifact_id"]} for c in chunks]}

    @router.get("/reports/pressure-streams/{stream_session_id}/samples")
    def report_pressure_stream_samples(stream_session_id: str, selected: dict[str, Any] = Depends(filters), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            return _pressure_samples_page(connection, stream_session_id, selected, cursor)

    @router.post("/reports/exports")
    def create_export(payload: dict[str, Any] = Body(default_factory=dict)) -> dict[str, Any]:
        fmt = str(payload.get("format") or "json").lower()
        if fmt not in {"json", "csv"}:
            raise HTTPException(status_code=422, detail={"error": "unsupported_export_format"})
        limit = int(payload.get("limit", 1000))
        if limit < 1 or limit > _MAX_EXPORT_ROWS:
            raise HTTPException(status_code=422, detail={"error": "export_limit_exceeded", "max_rows": _MAX_EXPORT_ROWS})
        selected = _base_filters(
            start=payload.get("start"),
            end=payload.get("end"),
            status=payload.get("status"),
            operation=payload.get("operation"),
            action=payload.get("action"),
            channel=payload.get("channel"),
            limit=limit,
        )
        with _read_snapshot(store) as connection:
            page = _command_page(connection, selected, None)
            snapshot = page["snapshot"]
            export_payload = {"schema_version": "bioxp.operator_report_export.v1", "filters": selected, "snapshot": snapshot, "commands": page["commands"]}
        export_id = uuid.uuid4().hex
        try:
            relpath, byte_count, digest = _write_export(store, export_id=export_id, fmt=fmt, payload=export_payload)
            created_at = time.time()
            with store.lock:
                store.connection.execute("BEGIN IMMEDIATE")
                try:
                    store.connection.execute(
                        "INSERT INTO report_exports(export_id,format,filter_json,filter_sha256,snapshot_json,row_count,sha256,byte_count,status,artifact_relpath,created_at,completed_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
                        (export_id, fmt, _canonical(selected), hashlib.sha256(_canonical(selected).encode()).hexdigest(), _canonical(snapshot), len(page["commands"]), digest, byte_count, "completed", relpath, created_at, created_at),
                    )
                    store.connection.execute("COMMIT")
                except Exception:
                    if store.connection.in_transaction:
                        store.connection.execute("ROLLBACK")
                    raise
        except Exception as exc:
            raise HTTPException(status_code=503, detail={"error": "export_generation_failed", "reason": str(exc)[:500]}) from exc
        return {"export_id": export_id, "status": "completed", "format": fmt, "row_count": len(page["commands"]), "sha256": digest, "byte_count": byte_count, "download": f"/operator/reports/exports/{export_id}/download"}

    @router.get("/reports/exports/{export_id}")
    def export_metadata(export_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM report_exports WHERE export_id=?", (export_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "export_not_found"})
            return {"export_id": row["export_id"], "format": row["format"], "filter": _decode_json(row["filter_json"], {}), "filter_sha256": row["filter_sha256"], "snapshot": _decode_json(row["snapshot_json"], {}), "row_count": row["row_count"], "sha256": row["sha256"], "byte_count": row["byte_count"], "status": row["status"], "created_at": row["created_at"], "completed_at": row["completed_at"], "download": f"/operator/reports/exports/{export_id}/download"}

    @router.get("/reports/exports/{export_id}/download")
    def export_download(export_id: str):
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM report_exports WHERE export_id=?", (export_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "export_not_found"})
            content = _read_export_artifact(store, row)
            headers = {
                "X-Content-SHA256": str(row["sha256"]),
                "Content-Disposition": f'attachment; filename="bioxp-report-{export_id}.{row["format"]}"',
            }
            media_type = "application/json" if row["format"] == "json" else "text/csv"
            return Response(content=content, media_type=media_type, headers=headers)

    @router.get("/audit-health")
    def audit_health() -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            page_count = int(connection.execute("PRAGMA page_count").fetchone()[0])
            page_size = int(connection.execute("PRAGMA page_size").fetchone()[0])
            return {
                "status": "ok",
                "store": _store_identity(connection),
                "database_bytes": page_count * page_size,
                "wal_bytes": (store.path.with_name(store.path.name + "-wal").stat().st_size if store.path.with_name(store.path.name + "-wal").exists() else 0),
                "commands": int(connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0]),
                "pipette_operations": int(connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0]),
                "retained_evidence": int(connection.execute("SELECT COUNT(*) FROM runtime_evidence_objects WHERE expiry_state='active'").fetchone()[0]),
                "pending_expiry_evidence": int(connection.execute("SELECT COUNT(*) FROM runtime_evidence_objects WHERE expiry_state='expiry_pending'").fetchone()[0]),
                "integrity_failures": int(connection.execute("SELECT COUNT(*) FROM runtime_evidence_events WHERE event_kind='integrity_failure'").fetchone()[0]),
                "migration_receipts": int(connection.execute("SELECT COUNT(*) FROM runtime_migration_receipts").fetchone()[0]),
                "exports": int(connection.execute("SELECT COUNT(*) FROM report_exports").fetchone()[0]),
            }

    return router
