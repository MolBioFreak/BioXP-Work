"""One read-only, paginated history projection over the canonical SQLite DB.

The archive stays in its original tables. Selection, deduplication and paging
happen in one SQLite snapshot; polling never opens evidence files or acquires
the audit writer's Python lock. Only selected scalar summary fields cross into
Python. Full source receipts remain available by command identity on demand.
"""
from __future__ import annotations

import base64
import json
import math
import sqlite3
from contextlib import closing
from datetime import datetime
from pathlib import Path
from typing import Any
from collections.abc import Mapping

HistoryKey = tuple[float, int, int, str]


def receipt_timestamp(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        # Older source receipts used timezone-qualified ISO timestamps. Keep
        # those instants; never guess the timezone of an unqualified string.
        if not isinstance(value, str):
            return None
        try:
            parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
            if parsed.tzinfo is None:
                return None
            result = parsed.timestamp()
        except (ValueError, OverflowError, OSError):
            return None
    return result if math.isfinite(result) else None


def _timestamp(value: Any) -> float:
    parsed = receipt_timestamp(value)
    return 0.0 if parsed is None else parsed


def receipt_accepted_at(row: Mapping[str, Any]) -> float:
    return _timestamp(row.get("accepted_at", row.get("queued_at", row.get("started_at"))))


def decode_cursor(cursor: str | None) -> HistoryKey | None:
    if cursor is None:
        return None
    try:
        if not cursor or len(cursor) > 1024:
            raise ValueError
        payload = json.loads(base64.b64decode(
            cursor + "=" * (-len(cursor) % 4), altchars=b"-_", validate=True,
        ))
        key = payload["key"]
        if (
            payload.get("v") != 1 or set(payload) != {"v", "key"}
            or not isinstance(key, list) or len(key) != 4
            or type(key[0]) not in (int, float) or not math.isfinite(key[0])
            or type(key[1]) is not int or key[1] not in (0, 1)
            or type(key[2]) is not int or not 0 <= key[2] <= 2**63 - 1
            or not isinstance(key[3], str) or not key[3] or len(key[3]) > 160
        ):
            raise ValueError
        return float(key[0]), key[1], key[2], key[3]
    except (ValueError, TypeError, KeyError, UnicodeError, OverflowError) as exc:
        raise ValueError("invalid_history_cursor") from exc


def encode_cursor(key: HistoryKey) -> str:
    raw = json.dumps({"v": 1, "key": key}, separators=(",", ":"), allow_nan=False).encode()
    return base64.urlsafe_b64encode(raw).decode().rstrip("=")


def _json(value: Any, default: Any = None) -> Any:
    return default if value is None else json.loads(value)


def read_history_page(root: str | Path, limit: int, cursor: str | None = None) -> tuple[list[dict[str, Any]], str | None]:
    """Return one page, plus a cursor only if another recorded row exists."""
    if type(limit) is not int or not 1 <= limit <= 200:
        raise ValueError("invalid_history_limit")
    boundary = decode_cursor(cursor)
    path = Path(root).resolve() / "bioxp_runtime.db"
    with closing(sqlite3.connect(path.as_uri() + "?mode=ro", uri=True, timeout=2.0, isolation_level=None)) as db:
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA query_only=ON")
        db.create_function("history_timestamp", 1, _timestamp, deterministic=True)
        db.execute("BEGIN")
        try:
            tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
            sources = []
            if "operator_commands" in tables:
                sources.append("""
                    SELECT command_id,sequence,1 AS source_rank,
                        history_timestamp(CASE
                            WHEN json_type(receipt_json,'$.accepted_at') IS NOT NULL
                                THEN json_extract(receipt_json,'$.accepted_at')
                            WHEN json_type(receipt_json,'$.queued_at') IS NOT NULL
                                THEN json_extract(receipt_json,'$.queued_at')
                            ELSE json_extract(receipt_json,'$.started_at') END) AS accepted_at
                    FROM operator_commands
                """)
            if "operator_plane_commands" in tables:
                join = "LEFT JOIN serial206_movement_commands c ON c.command_id=p.command_id" if "serial206_movement_commands" in tables else ""
                sequence = "COALESCE(c.sequence,p.stream_sequence)" if join else "p.stream_sequence"
                # Direct source wins before cursor filtering, including when its
                # timestamp conflicts with the retained projection's timestamp.
                exclusion = "WHERE NOT EXISTS (SELECT 1 FROM operator_commands d WHERE d.command_id=p.command_id)" if "operator_commands" in tables else ""
                sources.append(f"""
                    SELECT p.command_id,{sequence} AS sequence,0 AS source_rank,
                        history_timestamp(p.queued_at) AS accepted_at
                    FROM operator_plane_commands p {join} {exclusion}
                """)
            if not sources:
                return [], None
            where = "WHERE (accepted_at,source_rank,sequence,command_id)<(?,?,?,?)" if boundary else ""
            parameters = (*boundary, limit + 1) if boundary else (limit + 1,)
            selected = db.execute(f"""
                WITH history_keys AS ({' UNION ALL '.join(sources)})
                SELECT command_id,sequence,source_rank,accepted_at FROM history_keys
                {where} ORDER BY accepted_at DESC,source_rank DESC,sequence DESC,command_id DESC LIMIT ?
            """, parameters).fetchall()
            page = selected[:limit]
            if any(not row["command_id"] for row in page):
                raise RuntimeError("history_receipt_missing_command_id")
            result: dict[str, dict[str, Any]] = {}
            direct_ids = [r["command_id"] for r in page if r["source_rank"] == 1]
            retained_ids = [r["command_id"] for r in page if r["source_rank"] == 0]
            if direct_ids:
                assessment = """(SELECT detail_json FROM operator_transitions t
                    WHERE t.command_id=d.command_id AND json_type(t.detail_json,'$.operator_assessment')='text'
                    ORDER BY transition_id DESC LIMIT 1)""" if "operator_transitions" in tables else "NULL"
                rows = db.execute(f"""
                    SELECT d.command_id,d.sequence,d.action_id,d.status,d.ownership_generation,
                        d.controller_acknowledged,d.physical_effect_verified,d.response_summary_json,
                        json_extract(d.receipt_json,'$.schema_version') AS source_schema,
                        json_extract(d.receipt_json,'$.accepted_at') AS accepted_at,
                        json_extract(d.receipt_json,'$.queued_at') AS queued_at,
                        json_extract(d.receipt_json,'$.started_at') AS started_at,
                        json_extract(d.receipt_json,'$.dispatched_at') AS dispatched_at,
                        json_extract(d.receipt_json,'$.finished_at') AS finished_at,
                        json_extract(d.receipt_json,'$.remote_acknowledged') AS remote_acknowledged,
                        json_extract(d.receipt_json,'$.controller_terminal_state_verified') AS controller_terminal_state_verified,
                        json_extract(d.receipt_json,'$.machine_assessment') AS machine_assessment,
                        json_extract(d.receipt_json,'$.operator_assessment') AS saved_operator_assessment,
                        json_extract(d.receipt_json,'$.operator_note') AS saved_operator_note,
                        json_extract(d.receipt_json,'$.interrupt_evidence') AS interrupt_json,
                        json_extract(d.receipt_json,'$.transport_retention_errors') AS retention_json,
                        json_extract(d.receipt_json,'$.state_version') AS state_version,
                        json_extract(d.receipt_json,'$.method_id') AS method_id,
                        json_extract(d.receipt_json,'$.terminal_receipt_id') AS terminal_receipt_id,
                        json_extract(d.receipt_json,'$.completion_class') AS completion_class,
                        json_extract(d.receipt_json,'$.expected_board_epoch_by_board') AS epochs,
                        {assessment} AS assessment_json
                    FROM operator_commands d WHERE d.command_id IN ({','.join('?' for _ in direct_ids)})
                """, direct_ids).fetchall()
                for row in rows:
                    item = dict(row)
                    assessment_value = _json(item.pop("assessment_json"), {})
                    item["response"] = _json(item.pop("response_summary_json"))
                    item["interrupt_evidence"] = _json(item.pop("interrupt_json"))
                    item["transport_retention_errors"] = _json(item.pop("retention_json"), [])
                    item["expected_board_epoch_by_board"] = _json(item.pop("epochs"), {})
                    item["physical_effect_verified"] = bool(item["physical_effect_verified"])
                    item["history"] = {
                        "source": "direct", "source_schema": item.pop("source_schema"),
                        "recorded_status": item["status"],
                        "remote_acknowledged": None if item["remote_acknowledged"] is None else bool(item["remote_acknowledged"]),
                        "controller_acknowledged": bool(item["controller_acknowledged"]),
                        "controller_terminal_state_verified": None if item["controller_terminal_state_verified"] is None else bool(item["controller_terminal_state_verified"]),
                        "machine_assessment": item["machine_assessment"],
                        "operator_assessment": assessment_value.get("operator_assessment", item.pop("saved_operator_assessment")),
                        "operator_note": assessment_value.get("operator_note", item.pop("saved_operator_note")),
                    }
                    result[item["command_id"]] = item
            if retained_ids:
                canonical = "serial206_movement_commands" in tables
                join = "LEFT JOIN serial206_movement_commands c ON c.command_id=p.command_id" if canonical else ""
                projection = "COALESCE(c.state,p.status) AS recorded_status,COALESCE(c.state_version,p.version) AS state_version,c.expected_board_epochs_json AS epochs,c.terminal_receipt_id" if canonical else "p.status AS recorded_status,p.version AS state_version,NULL AS epochs,NULL AS terminal_receipt_id"
                rows = db.execute(f"""
                    SELECT p.command_id,p.action_id,p.method_id,p.ownership_generation,p.queued_at,
                        p.dispatched_at,p.finished_at,p.remote_acknowledged,p.controller_acknowledged,
                        p.physical_effect_verified,{projection},
                        json_extract(p.terminal_json,'$.completion_class') AS completion_class
                    FROM operator_plane_commands p {join}
                    WHERE p.command_id IN ({','.join('?' for _ in retained_ids)})
                """, retained_ids).fetchall()
                for row in rows:
                    item = dict(row)
                    recorded = item.pop("recorded_status")
                    item["status"] = "ambiguous" if recorded in {"queued", "dispatched", "issued_pending", "stop_requested", "abort_requested"} else recorded
                    item["accepted_at"] = item["queued_at"]
                    item["expected_board_epoch_by_board"] = _json(item.pop("epochs"), {})
                    item["physical_effect_verified"] = bool(item["physical_effect_verified"])
                    item["history"] = {
                        "source": "retained", "source_schema": "bioxp.operator_command_receipt.v1",
                        "recorded_status": recorded,
                        "remote_acknowledged": bool(item["remote_acknowledged"]),
                        "controller_acknowledged": bool(item["controller_acknowledged"]),
                        "controller_terminal_state_verified": None,
                        "machine_assessment": None, "operator_assessment": None, "operator_note": None,
                    }
                    result[item["command_id"]] = item
            ordered = []
            for key in page:
                item = result.get(key["command_id"])
                if item is None:
                    raise RuntimeError("history_receipt_missing_in_snapshot")
                item["sequence"] = key["sequence"]
                item["accepted_at"] = key["accepted_at"]
                item["__history_key"] = (key["accepted_at"], key["source_rank"], key["sequence"], key["command_id"])
                ordered.append(item)
            next_cursor = encode_cursor(ordered[-1]["__history_key"]) if len(selected) > limit else None
            return ordered, next_cursor
        finally:
            db.execute("ROLLBACK")
