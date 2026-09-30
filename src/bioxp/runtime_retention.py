"""Bounded D2 maintenance for the frozen runtime schema (no runtime hook).

Run with the service stopped, after making the separately approved backup.
The current schema blocks every proposed D2 deletion; history readers also
require legacy receipts. Report those facts rather than disabling triggers or
pretending that VACUUM implements retention. This deliberately does not migrate.
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
    candidates = connection.execute(
        "SELECT COUNT(*) FROM operator_commands c JOIN operator_plane_commands p USING(command_id) "
        "WHERE c.action_id='oem.deck.move_to_location' AND c.status='completed' AND p.status='completed' "
        "AND c.receipt_json<>'{}' AND c.updated_at<? AND p.updated_at<?",
        (cutoff, cutoff),
    ).fetchone()[0]
    tables["operator_commands"] = {"rows": total,
        "older_completed_named_with_canonical_rows": candidates,
        "eligible_rows": 0, "cleared_receipts": 0,
        "keep_reason": "OperatorReceiptStore._row_receipt/by_command/by_idempotency read receipt_json directly; v1/v2 receipt routes consult this row before canonical fallback"}
    return {"status": "verified", "as_of": as_of, "cutoff": cutoff,
        "tables": tables, "removed_rows": 0, "cleared_receipts": 0,
        "receipt_bytes_removed": 0, "schema_changed": False,
        "policy": "keep all last 14d, failed/ambiguous/interrupted/cancelled and linked records; frozen triggers and genuinely-read history remain intact"}


def retain_runtime_rows(root: str | Path, *, as_of: float, apply: bool = False) -> dict[str, Any]:
    """Return a deterministic plan; apply verifies it in one locked transaction.

    Current D2 targets are all excluded by exact triggers/readers. No backup,
    migration, receipt rewriting, trigger bypass, or replacement evidence is
    implicit. Dry-run opens SQLite read-only and does not touch lifecycle files.
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
