"""Bounded D2 maintenance for the frozen runtime schema (no runtime hook).

Run with the service stopped, after making the separately approved backup.
The frozen schema blocks D2 row deletions. Canonically owned completed named
receipt duplicates can be cleared after 14 days; SQL rows, outcome facts and
historical report versions remain. This deliberately does not migrate.
"""
from __future__ import annotations

import math
import sqlite3
from pathlib import Path
from typing import Any
from urllib.parse import quote

from .runtime_audit_store import runtime_lifecycle_lock, runtime_write_coordinator
from .storage_operations import StorageEvidenceError

# Exact producer/reader inventory is in the data-final audit. These history
# sources are not interchangeable with a current command-plane projection.
_TARGETS = {
    "pipette_operations": ("created_at", "pipette_operations_history_source_no_delete",
        "collection_state/replay_result and immutable history source; keep latest/channel and all other rows until a separately approved schema change"),
    "serial206_authority_snapshots": ("created_at", "serial206_authority_snapshots_no_delete_v1",
        "hash-bound append-only authority snapshots; deletion requires separately approved migration"),
    "operator_plane_command_versions": ("versioned_at", "operator_plane_command_versions_no_delete",
        "append-only; operator_reports reconstructs rows at retained export high-watermarks, not only latest version"),
    "operator_plane_pipette_versions": ("versioned_at", "operator_plane_pipette_versions_no_delete",
        "append-only; operator_reports reconstructs rows at retained export high-watermarks, not only latest version"),
}


def _verify(connection: sqlite3.Connection) -> None:
    from .oem_runtime_store import verify_canonical_runtime_database
    if [row[0] for row in connection.execute("PRAGMA quick_check")] != ["ok"]:
        raise StorageEvidenceError("retention database quick_check failed")
    verify_canonical_runtime_database(connection, full_data_check=True)


def _legacy_receipts(connection: sqlite3.Connection, cutoff: float) -> list[sqlite3.Row]:
    # Protect the entire parent/child component when any linked command is
    # recent or unsuccessful. A successful child of a failed workflow stays.
    return connection.execute("""
        WITH RECURSIVE protected(command_id) AS (
            SELECT command_id FROM operator_commands
            WHERE status NOT IN ('completed','observed') OR updated_at>=?
            UNION
            SELECT c.parent_command_id FROM operator_commands c JOIN protected k
                ON c.command_id=k.command_id WHERE c.parent_command_id IS NOT NULL
            UNION
            SELECT c.command_id FROM operator_commands c JOIN protected k
                ON c.parent_command_id=k.command_id
        )
        SELECT c.command_id,length(CAST(c.receipt_json AS BLOB))-2 AS receipt_bytes
        FROM operator_commands c JOIN operator_plane_commands p USING(command_id)
        JOIN serial206_movement_commands m USING(command_id)
        WHERE c.entrypoint_id='operator_command_plane' AND c.command_kind='operator'
          AND c.action_id='oem.deck.move_to_location' AND p.action_id=c.action_id AND m.action_id=c.action_id
          AND c.status='completed' AND p.status='completed' AND m.state='completed'
          AND c.updated_at<? AND p.updated_at<? AND p.queued_at<? AND p.finished_at<?
          AND c.receipt_json<>'{}'
          AND NOT EXISTS (SELECT 1 FROM protected k WHERE k.command_id=c.command_id)
          AND NOT EXISTS (SELECT 1 FROM runtime_retired_records r
              WHERE r.source_table='operator_plane_commands'
                AND r.record_key=json_array(p.command_id))
          AND json_type(c.receipt_json,'$.operator_assessment') IS NULL
          AND json_type(c.receipt_json,'$.operator_note') IS NULL
          AND json_type(c.receipt_json,'$.operator_assessment_idempotency_key') IS NULL
          AND json_type(c.receipt_json,'$.operator_assessed_at') IS NULL
        ORDER BY c.sequence
    """, (cutoff, cutoff, cutoff, cutoff, cutoff)).fetchall()


def _plan(connection: sqlite3.Connection, as_of: float) -> dict[str, Any]:
    cutoff = as_of - 14 * 24 * 60 * 60
    tables: dict[str, Any] = {}
    for table, (clock, blocker, reason) in _TARGETS.items():
        trigger = connection.execute(
            "SELECT sql FROM sqlite_master WHERE type='trigger' AND name=? AND tbl_name=?",
            (blocker, table),
        ).fetchone()
        if trigger is None:
            # This tool has no deletion contract for a different schema. Never
            # infer approval from a missing/altered trigger.
            raise StorageEvidenceError(f"retention requires the inventoried trigger: {blocker}")
        total, aged = connection.execute(
            f'SELECT COUNT(*),COALESCE(SUM("{clock}"<?),0) FROM "{table}"', (cutoff,)
        ).fetchone()
        tables[table] = {"rows": total, "older_than_14d_rows": aged,
            "eligible_rows": 0, "removed_rows": 0, "blocker": blocker,
            "blocker_sql": trigger[0], "keep_reason": reason}
    total = connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0]
    candidates = _legacy_receipts(connection, cutoff)
    tables["operator_commands"] = {"rows": total,
        "older_completed_named_with_canonical_rows": len(candidates),
        "eligible_rows": len(candidates), "cleared_receipts": 0,
        "eligible_receipt_bytes": sum(row["receipt_bytes"] for row in candidates),
        "eligible_command_ids": [row["command_id"] for row in candidates],
        "keep_reason": "rows and historical report versions stay; only canonical completed named receipt duplicates are eligible; recent/unsuccessful linked commands and blob-only assessments stay"}
    return {"status": "verified", "as_of": as_of, "cutoff": cutoff,
        "tables": tables, "removed_rows": 0, "cleared_receipts": 0,
        "receipt_bytes_removed": 0, "schema_changed": False,
        "policy": "keep all last 14d, failed/ambiguous/interrupted/cancelled and linked records; frozen triggers and genuinely-read history remain intact"}


def retain_runtime_rows(root: str | Path, *, as_of: float, apply: bool = False) -> dict[str, Any]:
    """Return a deterministic plan; apply verifies it in one locked transaction.

    Append-only targets remain excluded. Explicit apply clears only eligible
    canonical named duplicates, with no outcome or timestamp rewrite. No backup,
    migration, trigger bypass or replacement evidence is implicit. Dry-run opens
    SQLite read-only and does not touch lifecycle files.
    """
    if not math.isfinite(as_of) or as_of <= 0:
        raise ValueError("as_of must be a finite positive epoch")
    runtime_root = Path(root).absolute()
    database = runtime_root / "bioxp_runtime.db"
    if database.is_symlink() or not database.is_file():
        raise StorageEvidenceError("retention requires the existing canonical database")

    def run() -> dict[str, Any]:
        uri = f"file:{quote(str(database))}?mode={'rw' if apply else 'ro'}"
        connection = sqlite3.connect(uri, uri=True, isolation_level=None)
        connection.row_factory = sqlite3.Row
        try:
            connection.execute("PRAGMA foreign_keys=ON")
            connection.execute("BEGIN IMMEDIATE" if apply else "BEGIN")
            _verify(connection)
            result = _plan(connection, float(as_of))
            if apply:
                candidates = _legacy_receipts(connection, result["cutoff"])
                connection.executemany(
                    "UPDATE operator_commands SET receipt_json='{}' WHERE command_id=?",
                    [(row["command_id"],) for row in candidates],
                )
                # Read back exact targets before claiming clearance. The fixed
                # schema's ordinary version trigger remains enabled throughout.
                for row in candidates:
                    saved = connection.execute(
                        "SELECT receipt_json FROM operator_commands WHERE command_id=?", (row["command_id"],),
                    ).fetchone()
                    if saved is None or saved[0] != '{}':
                        raise StorageEvidenceError("legacy receipt clearance readback failed")
                result["cleared_receipts"] = len(candidates)
                result["receipt_bytes_removed"] = sum(row["receipt_bytes"] for row in candidates)
                result["tables"]["operator_commands"]["cleared_receipts"] = len(candidates)
            _verify(connection)
            connection.execute("COMMIT")
            return {**result, "dry_run": not apply, "quick_check": "ok",
                "canonical_verified": True, "database_bytes": database.stat().st_size}
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise
        finally:
            connection.close()

    if not apply:
        return run()
    with runtime_write_coordinator(runtime_root).lock, runtime_lifecycle_lock(runtime_root, exclusive=True):
        return run()
