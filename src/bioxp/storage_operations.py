from __future__ import annotations

import ctypes
import errno
import hashlib
import json
import os
import re
import shutil
import sqlite3
import stat
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping
from urllib.parse import quote

from .release_identity import current_release_identity
from .runtime_audit_store import (
    SCHEMA_VERSION,
    expected_runtime_migration_ledger,
    expected_runtime_schema_objects,
    runtime_lifecycle_lock,
    runtime_write_coordinator,
)


class StorageEvidenceError(RuntimeError):
    """A runtime storage evidence operation failed closed."""


_BACKUP_SCHEMA = "bioxp.runtime.backup.v2"
_BACKUP_RECEIPT_SCHEMA = "bioxp.runtime.backup.receipt.v2"
_RESTORE_RECEIPT_SCHEMA = "bioxp.runtime.restore.source_receipt.v2"
_CHECKPOINT_RECEIPT_SCHEMA = "bioxp.runtime.checkpoint.receipt.v2"
_CAPACITY_SCHEMA = "bioxp.runtime.capacity.v2"
_HEALTH_SCHEMA = "bioxp.runtime.audit_health.v2"
_GOVERNED_EVIDENCE_ROOTS = (
    "operator_evidence",
    "pipette/evidence",
    "report_exports",
    "artifacts",
)
_SUPPORT_ROOTS = ("archive",)
_MIN_REPRESENTATIVE_COMMANDS = 100
_MIN_REPRESENTATIVE_DAYS = 7
_REQUIRED_SAMPLE_TABLES = (
    "pipette_operations",
    "pipette_channel_observations",
    "pipette_transport_exchanges",
    "runtime_events",
    "pipette_pressure_chunks",
)
_REPRESENTATIVE_SAMPLE_POLICY = {
    "pipette_operations": (20, 0.25, "commands"),
    "pipette_channel_observations": (20, 0.50, "pipette_operations"),
    "pipette_transport_exchanges": (20, 0.50, "pipette_operations"),
    "runtime_events": (20, 0.50, "commands"),
    "pipette_pressure_chunks": (10, 0.10, "pipette_operations"),
}
_REPORT_EXPORT_FINAL_RE = re.compile(r"^(?P<export_id>[0-9a-f]{32})\.(?P<format>json|csv)$")
_REPORT_EXPORT_TEMP_RE = re.compile(r"^\.[0-9a-f]{32}\.[^.]+\.tmp$")
_REPORT_EXPORT_CLEANUP_RE = re.compile(r"^\.cleanup-[0-9a-f]{32}-[0-9a-f]{32}\.(?:json|csv)$")


def _contains_absolute_filesystem_path(value: Any) -> bool:
    if isinstance(value, Mapping):
        return any(_contains_absolute_filesystem_path(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return any(_contains_absolute_filesystem_path(item) for item in value)
    return isinstance(value, str) and Path(value).is_absolute()


def report_export_evidence_id(export_id: str) -> str:
    selected = str(export_id)
    if re.fullmatch(r"[0-9a-f]{32}", selected) is None:
        raise StorageEvidenceError("report export identity is malformed")
    return f"report-export:{selected}"


def report_export_retention_deadline(created_at: float) -> float:
    current = datetime.fromtimestamp(float(created_at), tz=timezone.utc)
    try:
        future = current.replace(year=current.year + 5)
    except ValueError:
        future = current.replace(year=current.year + 5, day=28)
    return future.timestamp()


def _report_export_detail(row: Mapping[str, Any], evidence_id: str, deadline: float) -> dict[str, Any]:
    return {
        "export_id": str(row["export_id"]),
        "evidence_artifact_id": evidence_id,
        "relpath": str(row["artifact_relpath"]),
        "sha256": str(row["sha256"]),
        "byte_count": int(row["byte_count"]),
        "format": str(row["format"]),
        "filter_sha256": str(row["filter_sha256"]),
        "filter_json": str(row["filter_json"]),
        "created_at": float(row["created_at"]),
        "retention_deadline": float(deadline),
    }


_LEGACY_REPORT_EXPORT_SNAPSHOT_FIELDS = frozenset(
    {
        "high_water_sequence",
        "database_identity",
        "schema_version",
        "database_path_exposed",
        "identity_version",
    }
)


def _is_legacy_report_export_snapshot(value: Any) -> bool:
    return bool(
        isinstance(value, Mapping)
        and set(value) == _LEGACY_REPORT_EXPORT_SNAPSHOT_FIELDS
        and type(value.get("high_water_sequence")) is int
        and int(value["high_water_sequence"]) >= 0
        and value.get("database_identity") == "robot_authoritative_sqlite"
        and type(value.get("schema_version")) is int
        and int(value["schema_version"]) >= 1
        and value.get("database_path_exposed") is False
        and (
            value.get("identity_version") is None
            or (
                type(value.get("identity_version")) is int
                and int(value["identity_version"]) >= 1
            )
        )
        and not _contains_absolute_filesystem_path(value)
    )


def _validate_report_export_row(row: Mapping[str, Any]) -> str:
    try:
        filters = json.loads(str(row["filter_json"]))
        receipt = json.loads(str(row["snapshot_json"]))
        expected_relpath = f"report_exports/{row['export_id']}.{row['format']}"
        canonical_filters = json.dumps(filters, sort_keys=True, separators=(",", ":"))
        common_valid = (
            isinstance(filters, dict)
            and isinstance(receipt, dict)
            and canonical_filters == str(row["filter_json"])
            and hashlib.sha256(canonical_filters.encode()).hexdigest() == str(row["filter_sha256"])
            and str(row["status"]) == "completed"
            and str(row["artifact_relpath"]) == expected_relpath
            and str(row["format"]) in {"json", "csv"}
            and type(row["row_count"]) is int
            and int(row["row_count"]) >= 0
            and type(row["byte_count"]) is int
            and int(row["byte_count"]) >= 0
        )
        if common_valid and _is_legacy_report_export_snapshot(receipt):
            return "legacy_snapshot_v0"
        artifact = receipt.get("artifact") if isinstance(receipt, dict) else None
        high_waters = receipt.get("source_high_waters") if isinstance(receipt, dict) else None
        valid = (
            common_valid
            and not _contains_absolute_filesystem_path(filters)
            and isinstance(artifact, dict)
            and isinstance(high_waters, dict)
            and receipt.get("receipt_schema") == "bioxp.operator_report_export_receipt.v1"
            and receipt.get("publisher_identity") == "bioxp.operator_reports"
            and str(receipt.get("export_id")) == str(row["export_id"])
            and str(receipt.get("evidence_artifact_id")) == report_export_evidence_id(str(row["export_id"]))
            and receipt.get("normalized_filters") == filters
            and str(receipt.get("filter_sha256")) == str(row["filter_sha256"])
            and int(receipt.get("row_count", -1)) == int(row["row_count"])
            and float(receipt.get("created_at", -1)) == float(row["created_at"])
            and float(receipt.get("retention_deadline", -1)) == report_export_retention_deadline(float(row["created_at"]))
            and str(artifact.get("format")) == str(row["format"])
            and str(artifact.get("sha256")) == str(row["sha256"])
            and int(artifact.get("byte_count", -1)) == int(row["byte_count"])
            and str(artifact.get("relpath")) == expected_relpath
            and isinstance(receipt.get("schema_identity"), dict)
            and isinstance(receipt.get("release_identity"), dict)
            and receipt.get("release_identity") == receipt["schema_identity"].get("release_identity")
            and not _contains_absolute_filesystem_path(receipt)
            and isinstance(receipt.get("database_incarnation_id"), str)
            and bool(receipt.get("database_incarnation_id"))
            and set(high_waters) == {
                "operator_commands", "operator_transitions", "pipette_operations",
                "pipette_channel_observations", "pipette_transport_exchanges", "runtime_events",
                "pipette_pressure_streams", "pipette_pressure_chunks", "runtime_evidence_objects",
                "runtime_evidence_links", "runtime_evidence_events",
                "operator_plane_command_versions", "operator_plane_pipette_versions",
                "operator_plane_pressure_stream_versions", "operator_plane_evidence_versions",
            }
            and all(type(value) is int and value >= 0 for value in high_waters.values())
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError, RuntimeError) as exc:
        raise StorageEvidenceError("report export receipt is not parseable") from exc
    if not valid:
        raise StorageEvidenceError(f"report export receipt identity contradiction: {row['export_id']}")
    return "receipt_v1"


def _complete_report_orphan_quarantine(
    connection: sqlite3.Connection,
    runtime_root: Path,
    row: Mapping[str, Any],
) -> None:
    original_relpath = str(row["original_relpath"])
    quarantine_relpath = str(row["active_relpath"])
    try:
        _read_regular(
            runtime_root,
            quarantine_relpath,
            expected_sha256=str(row["sha256"]),
            expected_bytes=int(row["byte_count"]),
        )
    except FileNotFoundError:
        _read_regular(
            runtime_root,
            original_relpath,
            expected_sha256=str(row["sha256"]),
            expected_bytes=int(row["byte_count"]),
        )
        source_parent, source_leaf = _open_parent(runtime_root, original_relpath)
        target_parent, target_leaf = _open_parent(runtime_root, quarantine_relpath, create=True)
        try:
            _rename_noreplace_at(
                source_parent,
                source_leaf,
                target_leaf,
                destination_parent_fd=target_parent,
            )
            os.fsync(source_parent)
            os.fsync(target_parent)
        finally:
            os.close(source_parent)
            os.close(target_parent)
        _read_regular(
            runtime_root,
            quarantine_relpath,
            expected_sha256=str(row["sha256"]),
            expected_bytes=int(row["byte_count"]),
        )
    existing = connection.execute(
        "SELECT 1 FROM runtime_evidence_events WHERE evidence_artifact_id=? AND event_kind='quarantined'",
        (str(row["evidence_artifact_id"]),),
    ).fetchone()
    if existing is None:
        connection.execute("BEGIN IMMEDIATE")
        try:
            connection.execute(
                "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                (
                    str(row["evidence_artifact_id"]),
                    "quarantined",
                    time.time(),
                    json.dumps(
                        {"original_relpath": original_relpath, "quarantine_relpath": quarantine_relpath},
                        sort_keys=True,
                        separators=(",", ":"),
                    ),
                ),
            )
            connection.execute("COMMIT")
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise


def _register_report_orphan(
    connection: sqlite3.Connection,
    runtime_root: Path,
    relative: str,
) -> None:
    _raw, digest, size = _read_regular(runtime_root, relative)
    identity_suffix = hashlib.sha256(f"{relative}:{digest}:{size}".encode()).hexdigest()
    evidence_id = f"report-orphan:{identity_suffix}"
    quarantine_relpath = f"quarantine/report_exports/{identity_suffix}.{Path(relative).name}"
    now = time.time()
    row = connection.execute(
        "SELECT * FROM runtime_evidence_objects WHERE evidence_artifact_id=?", (evidence_id,)
    ).fetchone()
    if row is None:
        connection.execute("BEGIN IMMEDIATE")
        try:
            connection.execute(
                "INSERT INTO runtime_evidence_objects(evidence_artifact_id,command_id,pipette_operation_id,original_relpath,active_relpath,sha256,byte_count,created_at,retention_deadline,legal_hold,expiry_state,expiry_receipt_id,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    evidence_id, None, None, relative, quarantine_relpath, digest, size, now,
                    report_export_retention_deadline(now), 0, "active", None, now,
                ),
            )
            connection.execute(
                "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                (
                    evidence_id,
                    "quarantine_pending",
                    now,
                    json.dumps(
                        {"original_relpath": relative, "quarantine_relpath": quarantine_relpath},
                        sort_keys=True,
                        separators=(",", ":"),
                    ),
                ),
            )
            connection.execute("COMMIT")
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise
        row = connection.execute(
            "SELECT * FROM runtime_evidence_objects WHERE evidence_artifact_id=?", (evidence_id,)
        ).fetchone()
    if row is None:
        raise StorageEvidenceError("report orphan evidence publication failed")
    _complete_report_orphan_quarantine(connection, runtime_root, row)


def reconcile_report_exports(root: str | Path) -> dict[str, int]:
    """Recover the report-file/evidence/receipt graph under the caller's lifecycle lock."""
    runtime_root = Path(root).absolute()
    database = runtime_root / "bioxp_runtime.db"
    export_root = runtime_root / "report_exports"
    _require_directory(runtime_root)
    if database.is_symlink() or not database.is_file():
        raise StorageEvidenceError("report export recovery requires the canonical runtime database")
    export_root.mkdir(mode=0o700, exist_ok=True)
    _require_directory(export_root)
    connection = sqlite3.connect(database, timeout=10.0)
    connection.row_factory = sqlite3.Row
    backfill: list[tuple[sqlite3.Row, str, float, dict[str, Any]]] = []
    governed_relpaths: set[str] = set()
    report_ids: set[str] = set()
    try:
        connection.execute("PRAGMA foreign_keys=ON")
        orphan_rows = connection.execute(
            "SELECT * FROM runtime_evidence_objects WHERE evidence_artifact_id LIKE 'report-orphan:%'"
        ).fetchall()
        for orphan_row in orphan_rows:
            _complete_report_orphan_quarantine(connection, runtime_root, orphan_row)
        rows = connection.execute("SELECT * FROM report_exports ORDER BY export_id").fetchall()
        for row in rows:
            export_id = str(row["export_id"])
            _validate_report_export_row(row)
            report_ids.add(export_id)
            evidence_id = report_export_evidence_id(export_id)
            expected_relpath = f"report_exports/{export_id}.{row['format']}"
            if str(row["status"]) != "completed" or str(row["artifact_relpath"]) != expected_relpath:
                raise StorageEvidenceError(f"report export receipt identity contradiction: {export_id}")
            deadline = report_export_retention_deadline(float(row["created_at"]))
            detail = _report_export_detail(row, evidence_id, deadline)
            objects = connection.execute(
                "SELECT * FROM runtime_evidence_objects WHERE evidence_artifact_id=?", (evidence_id,)
            ).fetchall()
            events = connection.execute(
                "SELECT * FROM runtime_evidence_events WHERE evidence_artifact_id=?", (evidence_id,)
            ).fetchall()
            links = connection.execute(
                "SELECT * FROM runtime_evidence_links WHERE evidence_artifact_id=?", (evidence_id,)
            ).fetchall()
            if not objects and not events and not links:
                raw, digest, size = _read_regular(runtime_root, expected_relpath)
                del raw
                if digest != str(row["sha256"]) or size != int(row["byte_count"]):
                    raise StorageEvidenceError(f"report export file identity contradiction: {export_id}")
                backfill.append((row, evidence_id, deadline, detail))
                governed_relpaths.add(expected_relpath)
            elif len(objects) != 1:
                raise StorageEvidenceError(f"report export evidence object is partial or duplicated: {export_id}")
            else:
                obj = objects[0]
                published = [
                    event
                    for event in events
                    if str(event["event_kind"]) == "published"
                    and float(event["observed_at"]) == float(row["created_at"])
                    and json.loads(str(event["detail_json"])) == detail
                ]
                export_links = [
                    link
                    for link in links
                    if str(link["target_kind"]) == "export"
                    and str(link["target_identity"]) == export_id
                    and link["command_id"] is None
                    and link["pipette_operation_id"] is None
                    and str(link["link_kind"]) == f"report_export:{export_id}"
                    and float(link["created_at"]) == float(row["created_at"])
                ]
                state = str(obj["expiry_state"])
                retained = state in {"active", "retained", "expiry_pending"}
                expired = state == "expired"
                path_valid = (
                    retained and str(obj["active_relpath"]) == expected_relpath
                ) or (
                    expired
                    and obj["active_relpath"] is None
                    and obj["expiry_receipt_id"] is not None
                ) or state in {"missing", "integrity_failed"}
                if (
                    len(published) != 1
                    or len(export_links) != 1
                    or obj["command_id"] is not None
                    or obj["pipette_operation_id"] is not None
                    or str(obj["original_relpath"]) != expected_relpath
                    or str(obj["sha256"]) != str(row["sha256"])
                    or int(obj["byte_count"]) != int(row["byte_count"])
                    or float(obj["created_at"]) != float(row["created_at"])
                    or float(obj["retention_deadline"]) != deadline
                    or state not in {
                        "active",
                        "retained",
                        "expiry_pending",
                        "expired",
                        "missing",
                        "integrity_failed",
                    }
                    or not path_valid
                ):
                    raise StorageEvidenceError(f"report export evidence identity contradiction: {export_id}")
                if retained:
                    raw, digest, size = _read_regular(runtime_root, expected_relpath)
                    del raw
                    if digest != str(row["sha256"]) or size != int(row["byte_count"]):
                        raise StorageEvidenceError(f"report export file identity contradiction: {export_id}")
                    governed_relpaths.add(expected_relpath)

        evidence_rows = connection.execute(
            "SELECT evidence_artifact_id,active_relpath FROM runtime_evidence_objects "
            "WHERE evidence_artifact_id LIKE 'report-export:%' OR active_relpath LIKE 'report_exports/%'"
        ).fetchall()
        authoritative_ids: set[str] = set()
        for evidence in evidence_rows:
            artifact_id = str(evidence["evidence_artifact_id"])
            if not artifact_id.startswith("report-export:"):
                raise StorageEvidenceError("report export path is bound to a non-report evidence identity")
            authoritative_ids.add(artifact_id)
            export_id = artifact_id.split(":", 1)[1]
            if export_id not in report_ids:
                raise StorageEvidenceError(f"report evidence has no report receipt: {export_id}")
        for table in ("runtime_evidence_events", "runtime_evidence_links"):
            rows = connection.execute(
                f"SELECT DISTINCT evidence_artifact_id FROM {table} WHERE evidence_artifact_id LIKE 'report-export:%'"
            ).fetchall()
            for evidence in rows:
                artifact_id = str(evidence["evidence_artifact_id"])
                authoritative_ids.add(artifact_id)
                export_id = artifact_id.split(":", 1)[1]
                if export_id not in report_ids:
                    raise StorageEvidenceError(f"report evidence has no report receipt: {export_id}")
        report_links = connection.execute(
            "SELECT evidence_artifact_id,link_kind FROM runtime_evidence_links WHERE link_kind LIKE 'report_export:%'"
        ).fetchall()
        for link in report_links:
            artifact_id = str(link["evidence_artifact_id"])
            export_id = str(link["link_kind"]).split(":", 1)[1]
            if artifact_id != report_export_evidence_id(export_id) or export_id not in report_ids:
                raise StorageEvidenceError("report export link has contradictory receipt/evidence authority")
        expected_authoritative_ids = {report_export_evidence_id(export_id) for export_id in report_ids}
        if authoritative_ids - expected_authoritative_ids:
            raise StorageEvidenceError("report export evidence authority contains an unknown identity")

        connection.execute("BEGIN IMMEDIATE")
        try:
            for row, evidence_id, deadline, detail in backfill:
                created_at = float(row["created_at"])
                relpath = str(row["artifact_relpath"])
                connection.execute(
                    "INSERT INTO runtime_evidence_objects("
                    "evidence_artifact_id,command_id,pipette_operation_id,original_relpath,active_relpath,"
                    "sha256,byte_count,created_at,retention_deadline,legal_hold,expiry_state,expiry_receipt_id,updated_at"
                    ") VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)",
                    (evidence_id, None, None, relpath, relpath, str(row["sha256"]), int(row["byte_count"]),
                     created_at, deadline, 0, "active", None, created_at),
                )
                connection.execute(
                    "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                    (evidence_id, "published", created_at, json.dumps(detail, sort_keys=True, separators=(",", ":"))),
                )
                connection.execute(
                    "INSERT INTO runtime_evidence_links(evidence_artifact_id,target_kind,target_identity,command_id,pipette_operation_id,link_kind,created_at) VALUES(?,?,?,?,?,?,?)",
                    (evidence_id, "export", str(row["export_id"]), None, None, f"report_export:{row['export_id']}", created_at),
                )
            connection.execute("COMMIT")
        except Exception:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise
    finally:
        connection.close()

    quarantined = 0
    recovery = sqlite3.connect(database, timeout=10.0)
    recovery.row_factory = sqlite3.Row
    try:
        recovery.execute("PRAGMA foreign_keys=ON")
        for entry in os.scandir(export_root):
            relative = f"report_exports/{entry.name}"
            is_known_final = _REPORT_EXPORT_FINAL_RE.fullmatch(entry.name) is not None
            is_known_temp = _REPORT_EXPORT_TEMP_RE.fullmatch(entry.name) is not None
            is_known_cleanup = _REPORT_EXPORT_CLEANUP_RE.fullmatch(entry.name) is not None
            if relative in governed_relpaths or not (is_known_final or is_known_temp or is_known_cleanup):
                continue
            _register_report_orphan(recovery, runtime_root, relative)
            quarantined += 1
    finally:
        recovery.close()
    return {"backfilled": len(backfill), "quarantined_orphans": quarantined}



def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    ).encode("utf-8")


def _normalized_sql(value: Any) -> str:
    return "".join(str(value or "").upper().split())


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(
        path,
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0),
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _require_directory(path: Path) -> None:
    selected = path.lstat()
    if not stat.S_ISDIR(selected.st_mode) or stat.S_ISLNK(selected.st_mode):
        raise StorageEvidenceError(f"storage root is not a non-symlink directory: {path}")


def _safe_relative(value: str | Path) -> Path:
    selected = Path(str(value))
    if selected.is_absolute() or not selected.parts:
        raise StorageEvidenceError(f"absolute or empty storage path is not allowed: {selected}")
    if any(part in {"", ".", ".."} for part in selected.parts):
        raise StorageEvidenceError(f"unsafe storage path: {selected}")
    return selected


def _open_parent(
    root: Path,
    relative: str | Path,
    *,
    create: bool = False,
) -> tuple[int, str]:
    selected = _safe_relative(relative)
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(root, os.O_RDONLY | os.O_DIRECTORY | nofollow)
    try:
        for part in selected.parts[:-1]:
            try:
                child = os.open(
                    part,
                    os.O_RDONLY | os.O_DIRECTORY | nofollow,
                    dir_fd=descriptor,
                )
            except FileNotFoundError:
                if not create:
                    raise
                os.mkdir(part, mode=0o700, dir_fd=descriptor)
                os.fsync(descriptor)
                child = os.open(
                    part,
                    os.O_RDONLY | os.O_DIRECTORY | nofollow,
                    dir_fd=descriptor,
                )
            os.close(descriptor)
            descriptor = child
        return descriptor, selected.parts[-1]
    except Exception:
        os.close(descriptor)
        raise


def _read_regular(
    root: Path,
    relative: str | Path,
    *,
    expected_sha256: str | None = None,
    expected_bytes: int | None = None,
) -> tuple[bytes, str, int]:
    parent, leaf = _open_parent(root, relative)
    descriptor: int | None = None
    try:
        descriptor = os.open(
            leaf,
            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=parent,
        )
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise StorageEvidenceError(f"storage object is not a regular file: {relative}")
        digest = hashlib.sha256()
        chunks: list[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
            digest.update(chunk)
        after = os.fstat(descriptor)
        if (before.st_dev, before.st_ino, before.st_size) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
        ):
            raise StorageEvidenceError(f"storage object changed while read: {relative}")
        actual_sha256 = digest.hexdigest()
        actual_bytes = int(before.st_size)
        if expected_sha256 is not None and actual_sha256 != str(expected_sha256):
            raise StorageEvidenceError(f"storage object digest mismatch: {relative}")
        if expected_bytes is not None and actual_bytes != int(expected_bytes):
            raise StorageEvidenceError(f"storage object size mismatch: {relative}")
        return b"".join(chunks), actual_sha256, actual_bytes
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent)


def _write_exclusive(root: Path, relative: str | Path, data: bytes) -> None:
    parent, leaf = _open_parent(root, relative, create=True)
    descriptor: int | None = None
    try:
        descriptor = os.open(
            leaf,
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_NOFOLLOW", 0),
            0o600,
            dir_fd=parent,
        )
        written = 0
        while written < len(data):
            count = os.write(descriptor, data[written:])
            if count <= 0:
                raise StorageEvidenceError(f"short storage write: {relative}")
            written += count
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.fsync(parent)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent)


def _copy_regular(
    source_root: Path,
    source_relative: str | Path,
    destination_root: Path,
    destination_relative: str | Path,
    *,
    expected_sha256: str | None = None,
    expected_bytes: int | None = None,
) -> dict[str, Any]:
    raw, digest, byte_count = _read_regular(
        source_root,
        source_relative,
        expected_sha256=expected_sha256,
        expected_bytes=expected_bytes,
    )
    _write_exclusive(destination_root, destination_relative, raw)
    _, copied_digest, copied_bytes = _read_regular(
        destination_root,
        destination_relative,
        expected_sha256=digest,
        expected_bytes=byte_count,
    )
    return {"sha256": copied_digest, "byte_count": copied_bytes}


def _open_directory_fd(path: Path) -> int:
    descriptor = os.open(
        path,
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_CLOEXEC", 0),
    )
    if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise StorageEvidenceError(f"storage root is not a directory: {path}")
    return descriptor


def _open_parent_at(
    root_fd: int,
    relative: str | Path,
    *,
    create: bool = False,
) -> tuple[int, str]:
    selected = _safe_relative(relative)
    descriptor = os.dup(root_fd)
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    try:
        for part in selected.parts[:-1]:
            try:
                child = os.open(
                    part,
                    os.O_RDONLY | os.O_DIRECTORY | nofollow,
                    dir_fd=descriptor,
                )
            except FileNotFoundError:
                if not create:
                    raise
                os.mkdir(part, mode=0o700, dir_fd=descriptor)
                os.fsync(descriptor)
                child = os.open(
                    part,
                    os.O_RDONLY | os.O_DIRECTORY | nofollow,
                    dir_fd=descriptor,
                )
            os.close(descriptor)
            descriptor = child
        return descriptor, selected.parts[-1]
    except Exception:
        os.close(descriptor)
        raise


def _read_regular_at(
    root_fd: int,
    relative: str | Path,
    *,
    expected_sha256: str | None = None,
    expected_bytes: int | None = None,
) -> tuple[bytes, str, int]:
    parent, leaf = _open_parent_at(root_fd, relative)
    descriptor: int | None = None
    try:
        descriptor = os.open(
            leaf,
            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=parent,
        )
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise StorageEvidenceError(f"storage object is not a regular file: {relative}")
        digest = hashlib.sha256()
        chunks: list[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
            digest.update(chunk)
        after = os.fstat(descriptor)
        if (before.st_dev, before.st_ino, before.st_size) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
        ):
            raise StorageEvidenceError(f"storage object changed while read: {relative}")
        actual_sha256 = digest.hexdigest()
        actual_bytes = int(before.st_size)
        if expected_sha256 is not None and actual_sha256 != str(expected_sha256):
            raise StorageEvidenceError(f"storage object digest mismatch: {relative}")
        if expected_bytes is not None and actual_bytes != int(expected_bytes):
            raise StorageEvidenceError(f"storage object size mismatch: {relative}")
        return b"".join(chunks), actual_sha256, actual_bytes
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent)


def _write_exclusive_at(root_fd: int, relative: str | Path, data: bytes) -> None:
    parent, leaf = _open_parent_at(root_fd, relative, create=True)
    descriptor: int | None = None
    try:
        descriptor = os.open(
            leaf,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
            0o600,
            dir_fd=parent,
        )
        written = 0
        while written < len(data):
            count = os.write(descriptor, data[written:])
            if count <= 0:
                raise StorageEvidenceError(f"short storage write: {relative}")
            written += count
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.fsync(parent)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent)


def _copy_regular_at(
    source_fd: int,
    source_relative: str | Path,
    destination_fd: int,
    destination_relative: str | Path,
    *,
    expected_sha256: str,
    expected_bytes: int,
) -> None:
    raw, digest, byte_count = _read_regular_at(
        source_fd,
        source_relative,
        expected_sha256=expected_sha256,
        expected_bytes=expected_bytes,
    )
    _write_exclusive_at(destination_fd, destination_relative, raw)
    _read_regular_at(
        destination_fd,
        destination_relative,
        expected_sha256=digest,
        expected_bytes=byte_count,
    )


def _remove_tree_at(parent_fd: int, leaf: str) -> None:
    try:
        child_fd = os.open(
            leaf,
            os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=parent_fd,
        )
    except FileNotFoundError:
        return
    try:
        for name in os.listdir(child_fd):
            selected = os.stat(name, dir_fd=child_fd, follow_symlinks=False)
            if stat.S_ISDIR(selected.st_mode):
                _remove_tree_at(child_fd, name)
            else:
                os.unlink(name, dir_fd=child_fd)
        os.fsync(child_fd)
    finally:
        os.close(child_fd)
    os.rmdir(leaf, dir_fd=parent_fd)
    os.fsync(parent_fd)


def _rename_noreplace_at(
    source_parent_fd: int,
    source: str,
    destination: str,
    *,
    destination_parent_fd: int | None = None,
) -> None:
    target_parent_fd = source_parent_fd if destination_parent_fd is None else destination_parent_fd
    try:
        renameat2 = ctypes.CDLL(None, use_errno=True).renameat2
    except AttributeError as exc:
        raise StorageEvidenceError("atomic no-replace publication is unavailable") from exc
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    if renameat2(source_parent_fd, os.fsencode(source), target_parent_fd, os.fsencode(destination), 1) == 0:
        return
    error = ctypes.get_errno()
    if error == errno.EEXIST:
        raise StorageEvidenceError("restore target must not already exist")
    if error in {errno.ENOSYS, errno.EINVAL, errno.ENOTSUP}:
        raise StorageEvidenceError("atomic no-replace publication is unavailable")
    raise OSError(error, os.strerror(error), destination)


def _write_atomic(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    _require_directory(path.parent)
    temporary = path.parent / f".{path.name}.{uuid.uuid4().hex}.tmp"
    descriptor = os.open(
        temporary,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
        0o600,
    )
    try:
        written = 0
        while written < len(data):
            count = os.write(descriptor, data[written:])
            if count <= 0:
                raise StorageEvidenceError(f"short storage write: {path}")
            written += count
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    try:
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _read_only_connection(path: Path) -> sqlite3.Connection:
    if path.is_symlink() or not path.is_file():
        raise StorageEvidenceError(f"runtime database is not a regular file: {path}")
    uri = f"file:{quote(str(path.absolute()))}?mode=ro"
    connection = sqlite3.connect(uri, uri=True, timeout=5.0)
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA query_only=ON")
    connection.execute("PRAGMA foreign_keys=ON")
    return connection


def _schema_identity(connection: sqlite3.Connection) -> dict[str, Any]:
    objects: list[dict[str, Any]] = []
    for row in connection.execute(
        """
        SELECT type,name,tbl_name,sql FROM sqlite_master
        WHERE type IN ('table','index','trigger','view')
          AND name NOT LIKE 'sqlite_%'
        ORDER BY type,name
        """
    ).fetchall():
        normalized = _normalized_sql(row["sql"])
        objects.append(
            {
                "type": str(row["type"]),
                "name": str(row["name"]),
                "table": str(row["tbl_name"]),
                "sql_sha256": _sha256_bytes(normalized.encode("utf-8")),
            }
        )
    migration_columns: list[str] = []
    migration_rows: list[dict[str, Any]] = []
    has_ledger = connection.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name='runtime_schema_migrations'"
    ).fetchone()
    if has_ledger is not None:
        migration_columns = [
            str(row["name"])
            for row in connection.execute("PRAGMA table_info(runtime_schema_migrations)").fetchall()
        ]
        projection = ",".join(f'"{name.replace(chr(34), chr(34) * 2)}"' for name in migration_columns)
        for row in connection.execute(
            f"SELECT {projection} FROM runtime_schema_migrations ORDER BY version"
        ).fetchall():
            migration_rows.append({name: row[name] for name in migration_columns})
    trigger_rows = [row for row in objects if row["type"] == "trigger"]
    return {
        "user_version": int(connection.execute("PRAGMA user_version").fetchone()[0]),
        "schema_objects": objects,
        "schema_sha256": _sha256_bytes(_canonical_bytes(objects)),
        "triggers": trigger_rows,
        "trigger_sha256": _sha256_bytes(_canonical_bytes(trigger_rows)),
        "migration_columns": migration_columns,
        "migration_rows": migration_rows,
        "migration_sha256": _sha256_bytes(
            _canonical_bytes({"columns": migration_columns, "rows": migration_rows})
        ),
    }


def _schema_contract_issues(connection: sqlite3.Connection) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    canonical = False
    try:
        from .oem_runtime_store import verify_canonical_runtime_database

        verify_canonical_runtime_database(connection)
        canonical = True
    except Exception as exc:
        issues.append(
            {"check": "canonical_schema_identity", "error": str(exc)[:500]}
        )
    if canonical:
        return issues
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    if version != SCHEMA_VERSION:
        issues.append(
            {"check": "schema_version", "expected": SCHEMA_VERSION, "actual": version}
        )
    expected_objects = expected_runtime_schema_objects()
    for (object_type, name), expected_sql in expected_objects.items():
        row = connection.execute(
            "SELECT sql FROM sqlite_master WHERE type=? AND name=?",
            (object_type, name),
        ).fetchone()
        if row is None:
            issues.append({"check": "schema_object_missing", "type": object_type, "name": name})
            continue
        actual_sql = _normalized_sql(row["sql"])
        if actual_sql != expected_sql:
            issues.append(
                {
                    "check": "schema_object_identity",
                    "type": object_type,
                    "name": name,
                    "expected_sha256": _sha256_bytes(expected_sql.encode("utf-8")),
                    "actual_sha256": _sha256_bytes(actual_sql.encode("utf-8")),
                }
            )
    expected_ledger = [dict(row) for row in expected_runtime_migration_ledger()]
    actual_ledger = [
        {"version": int(row[0]), "name": str(row[1]), "ddl_sha256": str(row[2])}
        for row in connection.execute(
            "SELECT version,name,ddl_sha256 FROM runtime_schema_migrations ORDER BY version"
        ).fetchall()
    ]
    if actual_ledger != expected_ledger:
        issues.append(
            {
                "check": "migration_ledger_identity",
                "expected": expected_ledger,
                "actual": actual_ledger,
            }
        )
    return issues


def inspect_database(path: Path) -> dict[str, Any]:
    connection = _read_only_connection(path)
    try:
        integrity = [str(row[0]) for row in connection.execute("PRAGMA integrity_check").fetchall()]
        foreign_keys = [tuple(row) for row in connection.execute("PRAGMA foreign_key_check").fetchall()]
        tables = [
            str(row[0])
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name"
            ).fetchall()
        ]
        counts: dict[str, int] = {}
        for table in (
            "operator_commands",
            "pipette_operations",
            "pipette_channel_observations",
            "pipette_transport_exchanges",
            "runtime_events",
            "pipette_pressure_streams",
            "pipette_pressure_chunks",
            "runtime_evidence_objects",
            "runtime_evidence_events",
            "runtime_migration_receipts",
            "runtime_migration_retirements",
            "runtime_migration_evidence",
            "report_exports",
        ):
            if table in tables:
                counts[table] = int(
                    connection.execute(f'SELECT COUNT(*) FROM "{table}"').fetchone()[0]
                )
        return {
            "byte_count": path.stat().st_size,
            "sha256": sha256_file(path),
            "journal_mode": str(connection.execute("PRAGMA journal_mode").fetchone()[0]).lower(),
            "synchronous": int(connection.execute("PRAGMA synchronous").fetchone()[0]),
            "integrity_check": integrity,
            "foreign_key_check": foreign_keys,
            "tables": tables,
            "counts": counts,
            "schema_identity": _schema_identity(connection),
            "schema_contract_issues": _schema_contract_issues(connection),
        }
    finally:
        connection.close()


def _remove_read_only_sqlite_sidecars(database: Path) -> None:
    wal = Path(f"{database}-wal")
    shm = Path(f"{database}-shm")
    if wal.exists() and wal.stat().st_size != 0:
        raise StorageEvidenceError("read-only backup inspection produced nonempty SQLite WAL")
    for sidecar in (wal, shm):
        if sidecar.exists():
            sidecar.unlink()


def _scan_tree(root: Path, relative_root: str, kind: str) -> list[dict[str, Any]]:
    selected = root / _safe_relative(relative_root)
    if not selected.exists():
        return []
    _require_directory(selected)
    rows: list[dict[str, Any]] = []
    for directory, directory_names, file_names in os.walk(selected, topdown=True, followlinks=False):
        current = Path(directory)
        for name in list(directory_names):
            candidate = current / name
            mode = candidate.lstat().st_mode
            if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
                raise StorageEvidenceError(f"non-directory in governed storage tree: {candidate}")
        for name in file_names:
            candidate = current / name
            relative = _safe_relative(candidate.relative_to(root))
            raw, digest, byte_count = _read_regular(root, relative)
            del raw
            rows.append(
                {
                    "kind": kind,
                    "source_relpath": relative.as_posix(),
                    "sha256": digest,
                    "byte_count": byte_count,
                }
            )
    return sorted(rows, key=lambda row: str(row["source_relpath"]))


def _scan_tree_at(root_descriptor: int, relative_root: str, kind: str) -> list[dict[str, Any]]:
    """Scan a governed tree below one already pinned directory descriptor."""
    selected = _safe_relative(relative_root)
    descriptor = os.dup(root_descriptor)
    try:
        try:
            for part in selected.parts:
                child = os.open(
                    part,
                    os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0),
                    dir_fd=descriptor,
                )
                os.close(descriptor)
                descriptor = child
        except FileNotFoundError:
            return []

        rows: list[dict[str, Any]] = []

        def walk(current_descriptor: int, current_relative: Path) -> None:
            for name in sorted(os.listdir(current_descriptor)):
                metadata = os.stat(name, dir_fd=current_descriptor, follow_symlinks=False)
                relative = current_relative / name
                if stat.S_ISDIR(metadata.st_mode):
                    child_descriptor = os.open(
                        name,
                        os.O_RDONLY
                        | getattr(os, "O_DIRECTORY", 0)
                        | getattr(os, "O_NOFOLLOW", 0),
                        dir_fd=current_descriptor,
                    )
                    try:
                        walk(child_descriptor, relative)
                    finally:
                        os.close(child_descriptor)
                elif stat.S_ISREG(metadata.st_mode):
                    raw, digest, byte_count = _read_regular_at(root_descriptor, relative)
                    del raw
                    rows.append(
                        {
                            "kind": kind,
                            "source_relpath": relative.as_posix(),
                            "sha256": digest,
                            "byte_count": byte_count,
                        }
                    )
                else:
                    raise StorageEvidenceError(
                        f"non-regular entry in governed storage tree: {relative.as_posix()}"
                    )

        walk(descriptor, selected)
        return sorted(rows, key=lambda row: str(row["source_relpath"]))
    finally:
        os.close(descriptor)


def _database_evidence_rows(database: Path) -> list[dict[str, Any]]:
    connection = _read_only_connection(database)
    try:
        has_table = connection.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='runtime_evidence_objects'"
        ).fetchone()
        if has_table is None:
            raise StorageEvidenceError("runtime evidence authority table is missing")
        rows = connection.execute(
            """
            SELECT o.evidence_artifact_id,o.command_id,o.pipette_operation_id,o.active_relpath,
                   o.sha256,o.byte_count,o.expiry_state,
                   COALESCE((
                       SELECT json_extract(e.detail_json,'$.legal_hold')
                       FROM runtime_evidence_events e
                       WHERE e.evidence_artifact_id=o.evidence_artifact_id
                         AND e.event_kind='legal_hold_assessment'
                         AND json_type(e.detail_json,'$.legal_hold') IN ('true','false')
                       ORDER BY e.event_id DESC LIMIT 1
                   ),o.legal_hold) AS legal_hold,
                   o.retention_deadline
            FROM runtime_evidence_objects o
            WHERE o.active_relpath IS NOT NULL
            ORDER BY o.active_relpath,o.evidence_artifact_id
            """
        ).fetchall()
        output: list[dict[str, Any]] = []
        physical: dict[str, tuple[str, int]] = {}
        for row in rows:
            relpath = _safe_relative(str(row["active_relpath"])).as_posix()
            if not any(
                relpath == governed or relpath.startswith(governed + "/")
                for governed in _GOVERNED_EVIDENCE_ROOTS
            ):
                raise StorageEvidenceError(f"database evidence path is outside governed roots: {relpath}")
            if type(row["byte_count"]) is not int or int(row["byte_count"]) < 0:
                raise StorageEvidenceError(f"database evidence size is not an exact integer: {relpath}")
            digest = str(row["sha256"])
            if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
                raise StorageEvidenceError(f"database evidence digest is invalid: {relpath}")
            identity = (digest, int(row["byte_count"]))
            prior = physical.get(relpath)
            if prior is not None and prior != identity:
                raise StorageEvidenceError(f"duplicate evidence receipts disagree: {relpath}")
            physical[relpath] = identity
            output.append(
                {
                    "evidence_artifact_id": str(row["evidence_artifact_id"]),
                    "command_id": row["command_id"],
                    "pipette_operation_id": row["pipette_operation_id"],
                    "source_relpath": relpath,
                    "sha256": digest,
                    "byte_count": int(row["byte_count"]),
                    "expiry_state": str(row["expiry_state"]),
                    "legal_hold": bool(row["legal_hold"]),
                    "retention_deadline": row["retention_deadline"],
                }
            )
        command_rows = connection.execute(
            """
            SELECT command_id,evidence_relpath,evidence_sha256,evidence_bytes
            FROM operator_commands WHERE evidence_relpath IS NOT NULL
            ORDER BY command_id
            """
        ).fetchall()
        by_path = {
            (row["source_relpath"], row["sha256"], row["byte_count"])
            for row in output
        }
        for row in command_rows:
            identity = (
                _safe_relative(str(row["evidence_relpath"])).as_posix(),
                str(row["evidence_sha256"]),
                int(row["evidence_bytes"]),
            )
            if identity not in by_path:
                raise StorageEvidenceError(
                    f"operator evidence projection has no exact evidence object: {row['command_id']}"
                )
        return output
    finally:
        connection.close()


def _evidence_closure(root: Path, database: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    receipts = _database_evidence_rows(database)
    physical_by_path: dict[str, tuple[str, int]] = {}
    for row in receipts:
        physical_by_path[str(row["source_relpath"])] = (
            str(row["sha256"]),
            int(row["byte_count"]),
        )
    files: list[dict[str, Any]] = []
    for relative_root in _GOVERNED_EVIDENCE_ROOTS:
        files.extend(_scan_tree(root, relative_root, "governed_evidence"))
    actual_by_path = {
        str(row["source_relpath"]): (str(row["sha256"]), int(row["byte_count"]))
        for row in files
    }
    if actual_by_path != physical_by_path:
        missing = sorted(set(physical_by_path) - set(actual_by_path))
        unbound = sorted(set(actual_by_path) - set(physical_by_path))
        mismatched = sorted(
            path
            for path in set(actual_by_path) & set(physical_by_path)
            if actual_by_path[path] != physical_by_path[path]
        )
        raise StorageEvidenceError(
            "database/evidence closure mismatch: "
            + json.dumps(
                {"missing": missing, "unbound": unbound, "mismatched": mismatched},
                sort_keys=True,
            )
        )
    return receipts, sorted(files, key=lambda row: str(row["source_relpath"]))


def _evidence_closure_at(
    root_descriptor: int,
    database: Path,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    receipts = _database_evidence_rows(database)
    physical_by_path = {
        str(row["source_relpath"]): (str(row["sha256"]), int(row["byte_count"]))
        for row in receipts
    }
    files: list[dict[str, Any]] = []
    for relative_root in _GOVERNED_EVIDENCE_ROOTS:
        files.extend(
            _scan_tree_at(root_descriptor, relative_root, "governed_evidence")
        )
    actual_by_path = {
        str(row["source_relpath"]): (str(row["sha256"]), int(row["byte_count"]))
        for row in files
    }
    if actual_by_path != physical_by_path:
        missing = sorted(set(physical_by_path) - set(actual_by_path))
        unbound = sorted(set(actual_by_path) - set(physical_by_path))
        mismatched = sorted(
            path
            for path in set(actual_by_path) & set(physical_by_path)
            if actual_by_path[path] != physical_by_path[path]
        )
        raise StorageEvidenceError(
            "database/evidence closure mismatch: "
            + json.dumps(
                {"missing": missing, "unbound": unbound, "mismatched": mismatched},
                sort_keys=True,
            )
        )
    return receipts, sorted(files, key=lambda row: str(row["source_relpath"]))


def _support_files(root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for relative_root in _SUPPORT_ROOTS:
        rows.extend(_scan_tree(root, relative_root, "migration_support"))
    legacy = root / "pipette" / "receipts.jsonl"
    if legacy.exists():
        raw, digest, byte_count = _read_regular(root, "pipette/receipts.jsonl")
        del raw
        rows.append(
            {
                "kind": "legacy_jsonl_source",
                "source_relpath": "pipette/receipts.jsonl",
                "sha256": digest,
                "byte_count": byte_count,
            }
        )
    return sorted(rows, key=lambda row: (str(row["kind"]), str(row["source_relpath"])))


def _online_backup(source: Path, destination_root: Path, destination_relative: str) -> dict[str, Any]:
    parent, leaf = _open_parent(destination_root, destination_relative, create=True)
    try:
        descriptor = os.open(
            leaf,
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_NOFOLLOW", 0),
            0o600,
            dir_fd=parent,
        )
        os.close(descriptor)
        os.fsync(parent)
    finally:
        os.close(parent)
    destination = destination_root / _safe_relative(destination_relative)
    source_connection = _read_only_connection(source)
    target = sqlite3.connect(destination, timeout=5.0)
    try:
        target.execute("PRAGMA journal_mode=DELETE")
        target.execute("PRAGMA synchronous=FULL")
        source_connection.backup(target, pages=128, sleep=0.05)
        target.commit()
    finally:
        target.close()
        source_connection.close()
    descriptor = os.open(destination, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    _fsync_directory(destination.parent)
    return {"sha256": sha256_file(destination), "byte_count": destination.stat().st_size}


def _checkpoint_locked(database: Path, mode: str) -> dict[str, Any]:
    if mode not in {"PASSIVE", "FULL", "RESTART", "TRUNCATE"}:
        raise StorageEvidenceError(f"unsupported SQLite checkpoint mode: {mode}")
    if database.is_symlink() or not database.is_file():
        raise StorageEvidenceError(f"runtime database is not a regular file: {database}")
    connection = sqlite3.connect(database, timeout=10.0)
    try:
        connection.execute("PRAGMA busy_timeout=5000")
        selected = connection.execute(f"PRAGMA wal_checkpoint({mode})").fetchone()
        if selected is None:
            raise StorageEvidenceError("SQLite checkpoint returned no receipt")
        result = tuple(int(value) for value in selected)
    finally:
        connection.close()
    return {
        "mode": mode,
        "busy": result[0],
        "wal_pages": result[1],
        "checkpointed_pages": result[2],
    }


def _write_sha256sums(unit: Path) -> str:
    rows: list[str] = []
    for directory, directory_names, file_names in os.walk(unit, topdown=True, followlinks=False):
        current = Path(directory)
        for name in directory_names:
            candidate = current / name
            if candidate.is_symlink():
                raise StorageEvidenceError(f"symlink in backup staging tree: {candidate}")
        for name in sorted(file_names):
            candidate = current / name
            if candidate.name == "SHA256SUMS":
                continue
            relative = candidate.relative_to(unit).as_posix()
            _, digest, _ = _read_regular(unit, relative)
            rows.append(f"{digest}  {relative}")
    data = ("\n".join(sorted(rows)) + "\n").encode("utf-8")
    _write_exclusive(unit, "SHA256SUMS", data)
    return _sha256_bytes(data)


def _verify_sha256sums(unit: Path) -> dict[str, str]:
    raw, sums_digest, _ = _read_regular(unit, "SHA256SUMS")
    listed: dict[str, str] = {}
    for line in raw.decode("utf-8").splitlines():
        if not line:
            continue
        try:
            digest, relative = line.split("  ", 1)
        except ValueError as exc:
            raise StorageEvidenceError("backup SHA256SUMS contains a malformed row") from exc
        selected = _safe_relative(relative).as_posix()
        if selected == "SHA256SUMS" or selected in listed:
            raise StorageEvidenceError(f"backup SHA256SUMS contains an invalid duplicate: {selected}")
        _read_regular(unit, selected, expected_sha256=digest)
        listed[selected] = digest
    actual: set[str] = set()
    for directory, directory_names, file_names in os.walk(unit, topdown=True, followlinks=False):
        current = Path(directory)
        for name in directory_names:
            candidate = current / name
            if candidate.is_symlink():
                raise StorageEvidenceError(f"symlink in backup unit: {candidate}")
        for name in file_names:
            candidate = current / name
            relative = candidate.relative_to(unit).as_posix()
            if relative != "SHA256SUMS":
                actual.add(relative)
    if actual != set(listed):
        raise StorageEvidenceError(
            "backup listed-file closure mismatch: "
            + json.dumps(
                {"missing": sorted(set(listed) - actual), "extra": sorted(actual - set(listed))},
                sort_keys=True,
            )
        )
    return {"sha256sums_sha256": sums_digest, "listed_file_count": str(len(listed))}


def verify_backup_unit(unit: str | Path) -> dict[str, Any]:
    backup_root = Path(unit).absolute()
    _require_directory(backup_root)
    sums = _verify_sha256sums(backup_root)
    manifest_raw, manifest_sha256, _ = _read_regular(backup_root, "manifest.json")
    try:
        manifest = json.loads(manifest_raw)
    except json.JSONDecodeError as exc:
        raise StorageEvidenceError("backup manifest is invalid JSON") from exc
    if manifest.get("schema") != _BACKUP_SCHEMA:
        raise StorageEvidenceError("unsupported backup manifest schema")
    database = backup_root / "bioxp_runtime.db"
    health = inspect_database(database)
    _remove_read_only_sqlite_sidecars(database)
    if health["integrity_check"] != ["ok"] or health["foreign_key_check"]:
        raise StorageEvidenceError("backup database failed integrity checks")
    if health["schema_identity"] != manifest.get("backup_database", {}).get("schema_identity"):
        raise StorageEvidenceError("backup database schema/migration/trigger identity mismatch")
    if health["schema_contract_issues"]:
        raise StorageEvidenceError("backup database does not match the source-owned schema contract")
    database_receipts = _database_evidence_rows(database)
    _remove_read_only_sqlite_sidecars(database)
    if database_receipts != manifest.get("evidence_receipts"):
        raise StorageEvidenceError("backup manifest does not exactly match database evidence receipts")
    expected_package_files = {"bioxp_runtime.db", "manifest.json"}
    for row in [*manifest.get("evidence_files", []), *manifest.get("support_files", [])]:
        backup_relpath = _safe_relative(str(row["backup_relpath"])).as_posix()
        _read_regular(
            backup_root,
            backup_relpath,
            expected_sha256=str(row["sha256"]),
            expected_bytes=int(row["byte_count"]),
        )
        expected_package_files.add(backup_relpath)
    listed = set()
    sums_raw, _, _ = _read_regular(backup_root, "SHA256SUMS")
    for line in sums_raw.decode("utf-8").splitlines():
        if line:
            listed.add(line.split("  ", 1)[1])
    if listed != expected_package_files:
        raise StorageEvidenceError("backup manifest/package file identity mismatch")
    publication_epoch = manifest.get("publication_epoch")
    if type(publication_epoch) not in {int, float} or float(publication_epoch) <= 0:
        raise StorageEvidenceError("backup manifest publication time is missing or invalid")
    release_identity = manifest.get("release_identity")
    if not isinstance(release_identity, Mapping):
        raise StorageEvidenceError("backup manifest release identity is missing")
    return {
        "schema": _BACKUP_SCHEMA,
        "status": "source_verified",
        "backup_id": str(manifest["backup_id"]),
        "manifest_sha256": manifest_sha256,
        "sha256sums_sha256": sums["sha256sums_sha256"],
        "database": health,
        "evidence_file_count": len(manifest.get("evidence_files", [])),
        "publication_epoch": float(publication_epoch),
        "published_at": manifest.get("published_at"),
        "release_identity": dict(release_identity),
        "execution_performed": False,
    }


def _verify_external_backup_receipt(
    runtime_root: Path,
    backup_root: Path,
    verified: Mapping[str, Any],
    manifest: Mapping[str, Any],
) -> dict[str, Any]:
    backup_id = str(manifest.get("backup_id") or "")
    expected_unit = runtime_root / "backups" / backup_id
    if backup_root != expected_unit or backup_root.parent != runtime_root / "backups":
        raise StorageEvidenceError("backup unit is not the receipt-bound direct backup child")
    receipt_path = runtime_root / "backups" / "receipts" / f"{backup_id}.json"
    raw, _, _ = _read_regular(receipt_path.parent, receipt_path.name)
    try:
        receipt = json.loads(raw)
    except (TypeError, json.JSONDecodeError) as exc:
        raise StorageEvidenceError("external backup receipt is invalid JSON") from exc
    if not isinstance(receipt, dict):
        raise StorageEvidenceError("external backup receipt is not an object")
    identity = {
        "schema": _BACKUP_RECEIPT_SCHEMA,
        "status": "verified",
        "backup_id": backup_id,
        "unit_relpath": f"backups/{backup_id}",
        "published_at": manifest.get("published_at"),
        "publication_epoch": verified.get("publication_epoch"),
        "manifest_sha256": verified.get("manifest_sha256"),
        "sha256sums_sha256": verified.get("sha256sums_sha256"),
        "database_sha256": verified.get("database", {}).get("sha256"),
        "schema_sha256": verified.get("database", {}).get("schema_identity", {}).get("schema_sha256"),
        "migration_sha256": verified.get("database", {}).get("schema_identity", {}).get("migration_sha256"),
        "trigger_sha256": verified.get("database", {}).get("schema_identity", {}).get("trigger_sha256"),
        "evidence_file_count": verified.get("evidence_file_count"),
        "release_identity": verified.get("release_identity"),
        "closure": "exact_bidirectional_database_evidence",
    }
    mismatches = {
        key: {"expected": value, "actual": receipt.get(key)}
        for key, value in identity.items()
        if receipt.get(key) != value
    }
    if mismatches:
        raise StorageEvidenceError(
            "external backup receipt identity mismatch: "
            + json.dumps(mismatches, sort_keys=True, default=str)
        )
    return receipt


def _validated_label(label: str) -> str:
    selected = str(label).strip()
    if (
        not selected
        or len(selected) > 80
        or any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-" for character in selected)
    ):
        raise StorageEvidenceError("backup label must use 1..80 safe filename characters")
    return selected


def create_backup_unit(
    root: str | Path,
    *,
    label: str,
    phase: str,
    source_kind: str | None = None,
    source_digest: str | None = None,
) -> dict[str, Any]:
    runtime_root = Path(root).absolute()
    _require_directory(runtime_root)
    database = runtime_root / "bioxp_runtime.db"
    if database.is_symlink() or not database.is_file():
        raise StorageEvidenceError(f"runtime database is missing or unsafe: {database}")
    selected_label = _validated_label(label)
    backup_id = f"{selected_label}.{time.time_ns()}.{uuid.uuid4().hex}"
    backups = runtime_root / "backups"
    backups.mkdir(mode=0o700, exist_ok=True)
    _require_directory(backups)
    staging = backups / f".{backup_id}.staging"
    final = backups / backup_id
    os.mkdir(staging, mode=0o700)
    published = False
    try:
        with runtime_write_coordinator(runtime_root).lock, runtime_lifecycle_lock(
            runtime_root, exclusive=True
        ):
            reconcile_report_exports(runtime_root)
            checkpoint = _checkpoint_locked(database, "TRUNCATE")
            if checkpoint["busy"] != 0:
                raise StorageEvidenceError("runtime database checkpoint was busy")
            source_health = inspect_database(database)
            if source_health["integrity_check"] != ["ok"] or source_health["foreign_key_check"]:
                raise StorageEvidenceError("source database failed integrity checks before backup")
            if source_health["schema_contract_issues"]:
                raise StorageEvidenceError("source database failed exact schema/migration/trigger attestation")
            _online_backup(database, staging, "bioxp_runtime.db")
            backup_health = inspect_database(staging / "bioxp_runtime.db")
            _remove_read_only_sqlite_sidecars(staging / "bioxp_runtime.db")
            if backup_health["integrity_check"] != ["ok"] or backup_health["foreign_key_check"]:
                raise StorageEvidenceError("online backup failed integrity checks")
            if source_health["schema_identity"] != backup_health["schema_identity"]:
                raise StorageEvidenceError("online backup changed schema/migration/trigger identity")
            evidence_receipts, evidence_files = _evidence_closure(
                runtime_root,
                staging / "bioxp_runtime.db",
            )
            _remove_read_only_sqlite_sidecars(staging / "bioxp_runtime.db")
            support_files = _support_files(runtime_root)
            for row in evidence_files:
                destination = f"evidence/{row['source_relpath']}"
                _copy_regular(
                    runtime_root,
                    str(row["source_relpath"]),
                    staging,
                    destination,
                    expected_sha256=str(row["sha256"]),
                    expected_bytes=int(row["byte_count"]),
                )
                row["backup_relpath"] = destination
            for row in support_files:
                destination = f"support/{row['source_relpath']}"
                _copy_regular(
                    runtime_root,
                    str(row["source_relpath"]),
                    staging,
                    destination,
                    expected_sha256=str(row["sha256"]),
                    expected_bytes=int(row["byte_count"]),
                )
                row["backup_relpath"] = destination
            release_identity = current_release_identity()
            publication_epoch = time.time()
            published_at = datetime.fromtimestamp(
                publication_epoch, timezone.utc
            ).isoformat().replace("+00:00", "Z")
            manifest = {
                "schema": _BACKUP_SCHEMA,
                "backup_id": backup_id,
                "created_at": published_at,
                "published_at": published_at,
                "publication_epoch": publication_epoch,
                "phase": str(phase),
                "source_kind": source_kind,
                "source_digest": source_digest,
                "source_database": source_health,
                "backup_database": backup_health,
                "checkpoint": checkpoint,
                "evidence_receipts": evidence_receipts,
                "evidence_files": evidence_files,
                "support_files": support_files,
                "evidence_file_count": len(evidence_files),
                "evidence_file_bytes": sum(int(row["byte_count"]) for row in evidence_files),
                "source_deployment": release_identity,
                "release_identity": release_identity,
                "closure": "exact_bidirectional_database_evidence",
            }
            _write_exclusive(
                staging,
                "manifest.json",
                json.dumps(manifest, sort_keys=True, indent=2).encode("utf-8") + b"\n",
            )
            _write_sha256sums(staging)
            verified = verify_backup_unit(staging)
            if final.exists() or final.is_symlink():
                raise StorageEvidenceError("opaque backup identity unexpectedly already exists")
            _fsync_directory(staging)
            os.rename(staging, final)
            published = True
            _fsync_directory(backups)
        receipt = {
            "schema": _BACKUP_RECEIPT_SCHEMA,
            "status": "verified",
            "backup_id": backup_id,
            "unit_relpath": f"backups/{backup_id}",
            "created_at": manifest["published_at"],
            "published_at": manifest["published_at"],
            "publication_epoch": manifest["publication_epoch"],
            "manifest_sha256": verified["manifest_sha256"],
            "sha256sums_sha256": verified["sha256sums_sha256"],
            "database_sha256": verified["database"]["sha256"],
            "schema_sha256": verified["database"]["schema_identity"]["schema_sha256"],
            "migration_sha256": verified["database"]["schema_identity"]["migration_sha256"],
            "trigger_sha256": verified["database"]["schema_identity"]["trigger_sha256"],
            "evidence_file_count": verified["evidence_file_count"],
            "release_identity": verified["release_identity"],
            "closure": "exact_bidirectional_database_evidence",
        }
        receipt_root = backups / "receipts"
        receipt_root.mkdir(mode=0o700, exist_ok=True)
        _require_directory(receipt_root)
        _write_exclusive(
            receipt_root,
            f"{backup_id}.json",
            json.dumps(receipt, sort_keys=True, indent=2).encode("utf-8") + b"\n",
        )
        return receipt
    except Exception:
        if not published:
            shutil.rmtree(staging, ignore_errors=True)
            _fsync_directory(backups)
        raise


def _restore_target(runtime_root: Path, requested: str | Path | None, backup_id: str) -> Path:
    base = runtime_root / "restore_drills"
    base.mkdir(mode=0o700, exist_ok=True)
    _require_directory(base)
    if requested is None:
        return base / f"{backup_id}.{uuid.uuid4().hex}"
    candidate = Path(requested)
    selected = candidate if candidate.is_absolute() else base / candidate
    selected = selected.absolute()
    if selected.parent != base or selected.name in {"", ".", ".."}:
        raise StorageEvidenceError("restore target must be a direct child of the isolated restore-drill root")
    if any(
        character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-"
        for character in selected.name
    ):
        raise StorageEvidenceError("restore target child name contains unsafe characters")
    return selected


def restore_drill(unit: str | Path, *, target: str | Path | None = None) -> dict[str, Any]:
    backup_root = Path(unit).absolute()
    verified = verify_backup_unit(backup_root)
    manifest_raw, _, _ = _read_regular(backup_root, "manifest.json")
    manifest = json.loads(manifest_raw)
    backups = backup_root.parent
    runtime_root = backups.parent
    _require_directory(runtime_root)
    external_receipt = _verify_external_backup_receipt(
        runtime_root,
        backup_root,
        verified,
        manifest,
    )
    destination = _restore_target(runtime_root, target, str(manifest["backup_id"]))
    staging_name = f".{destination.name}.{uuid.uuid4().hex}.staging"
    with runtime_lifecycle_lock(runtime_root, exclusive=True):
        backup_fd: int | None = None
        restore_fd: int | None = None
        staging_fd: int | None = None
        staging_created = False
        published = False
        try:
            backup_fd = _open_directory_fd(backup_root)
            restore_fd = _open_directory_fd(destination.parent)
            pinned_manifest_raw, pinned_manifest_sha256, _ = _read_regular_at(
                backup_fd,
                "manifest.json",
                expected_sha256=str(verified["manifest_sha256"]),
            )
            if pinned_manifest_sha256 != str(verified["manifest_sha256"]):
                raise StorageEvidenceError("pinned backup manifest identity differs")
            pinned_manifest = json.loads(pinned_manifest_raw)
            if pinned_manifest != manifest:
                raise StorageEvidenceError("pinned backup manifest changed after verification")
            try:
                os.stat(destination.name, dir_fd=restore_fd, follow_symlinks=False)
            except FileNotFoundError:
                pass
            else:
                raise StorageEvidenceError("restore target must not already exist")
            os.mkdir(staging_name, mode=0o700, dir_fd=restore_fd)
            staging_created = True
            os.fsync(restore_fd)
            staging_fd = os.open(
                staging_name,
                os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0),
                dir_fd=restore_fd,
            )
            _copy_regular_at(
                backup_fd,
                "bioxp_runtime.db",
                staging_fd,
                "bioxp_runtime.db",
                expected_sha256=str(manifest["backup_database"]["sha256"]),
                expected_bytes=int(manifest["backup_database"]["byte_count"]),
            )
            for row in manifest.get("evidence_files", []):
                _copy_regular_at(
                    backup_fd,
                    str(row["backup_relpath"]),
                    staging_fd,
                    str(row["source_relpath"]),
                    expected_sha256=str(row["sha256"]),
                    expected_bytes=int(row["byte_count"]),
                )
            for row in manifest.get("support_files", []):
                _copy_regular_at(
                    backup_fd,
                    str(row["backup_relpath"]),
                    staging_fd,
                    str(row["source_relpath"]),
                    expected_sha256=str(row["sha256"]),
                    expected_bytes=int(row["byte_count"]),
                )
            staging_root = Path(f"/proc/self/fd/{staging_fd}")
            restored_health = inspect_database(staging_root / "bioxp_runtime.db")
            if restored_health != manifest["backup_database"]:
                raise StorageEvidenceError("isolated restore database identity differs from backup")
            restored_receipts, restored_files = _evidence_closure_at(
                staging_fd,
                staging_root / "bioxp_runtime.db",
            )
            if restored_receipts != manifest.get("evidence_receipts"):
                raise StorageEvidenceError("isolated restore database receipt closure differs")
            expected_files = [
                {
                    key: row[key]
                    for key in ("kind", "source_relpath", "sha256", "byte_count")
                }
                for row in manifest.get("evidence_files", [])
            ]
            if restored_files != expected_files:
                raise StorageEvidenceError("isolated restore filesystem closure differs")
            os.fsync(staging_fd)
            _rename_noreplace_at(restore_fd, staging_name, destination.name)
            published = True
            os.fsync(restore_fd)
        except Exception:
            if staging_created and not published and restore_fd is not None:
                _remove_tree_at(restore_fd, staging_name)
            raise
        finally:
            if staging_fd is not None:
                os.close(staging_fd)
            if restore_fd is not None:
                os.close(restore_fd)
            if backup_fd is not None:
                os.close(backup_fd)
    receipt = {
        "schema": _RESTORE_RECEIPT_SCHEMA,
        "status": "source_verified",
        "backup_id": str(manifest["backup_id"]),
        "restore_id": destination.name,
        "target_relpath": destination.relative_to(runtime_root).as_posix(),
        "created_at": _utc_now(),
        "manifest_sha256": verified["manifest_sha256"],
        "external_receipt_schema": external_receipt["schema"],
        "external_receipt_publication_epoch": external_receipt["publication_epoch"],
        "database_sha256": restored_health["sha256"],
        "schema_sha256": restored_health["schema_identity"]["schema_sha256"],
        "migration_sha256": restored_health["schema_identity"]["migration_sha256"],
        "trigger_sha256": restored_health["schema_identity"]["trigger_sha256"],
        "restored_file_count": len(restored_files),
        "release_identity": verified["release_identity"],
        "closure": "source_only",
        "execution_performed": False,
        "service_started": False,
        "hardware_touched": False,
        "production_database_modified": False,
    }
    receipt_root = runtime_root / "restore_receipts"
    receipt_root.mkdir(mode=0o700, exist_ok=True)
    _require_directory(receipt_root)
    _write_exclusive(
        receipt_root,
        f"{destination.name}.json",
        json.dumps(receipt, sort_keys=True, indent=2).encode("utf-8") + b"\n",
    )
    return receipt


def checkpoint_database(path: str | Path, mode: str = "PASSIVE") -> dict[str, Any]:
    database = Path(path).absolute()
    root = database.parent
    _require_directory(root)
    with runtime_write_coordinator(root).lock:
        with runtime_lifecycle_lock(root, exclusive=True):
            reconcile_report_exports(root)
            checkpoint = _checkpoint_locked(database, mode)
            health = inspect_database(database)
    status = (
        "verified"
        if checkpoint["busy"] == 0
        and health["integrity_check"] == ["ok"]
        and not health["foreign_key_check"]
        and not health["schema_contract_issues"]
        else "degraded"
    )
    database_path = str(database.resolve())
    schema_version = int(health["schema_identity"]["user_version"])
    checkpoint_identity = _sha256_bytes(
        _canonical_bytes(
            {
                "database_path": database_path,
                "database_sha256": health["sha256"],
                "schema_version": schema_version,
                "schema_sha256": health["schema_identity"]["schema_sha256"],
                "checkpoint": checkpoint,
            }
        )
    )
    receipt = {
        "schema": _CHECKPOINT_RECEIPT_SCHEMA,
        "status": status,
        "created_at": _utc_now(),
        "created_epoch": time.time(),
        "database_path": database_path,
        "database_sha256": health["sha256"],
        "schema_version": schema_version,
        "schema_sha256": health["schema_identity"]["schema_sha256"],
        "checkpoint_identity": checkpoint_identity,
        "checkpoint": checkpoint,
        **checkpoint,
    }
    health_root = root / "health_receipts"
    health_root.mkdir(mode=0o700, exist_ok=True)
    _write_atomic(
        health_root / "last-checkpoint.json",
        json.dumps(receipt, sort_keys=True, indent=2).encode("utf-8") + b"\n",
    )
    return receipt


def _parse_epoch(value: Any) -> float | None:
    if value is None:
        return None
    try:
        numeric = float(value)
        if numeric > 0:
            return numeric
    except (TypeError, ValueError):
        pass
    if isinstance(value, str):
        try:
            return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()
        except ValueError:
            return None
    return None


def _tree_bytes(root: Path) -> int:
    if not root.exists():
        return 0
    _require_directory(root)
    return sum(int(row["byte_count"]) for row in _scan_tree(root.parent, root.name, "size"))


def capacity_report(root: str | Path, allocation_bytes: int | None = None) -> dict[str, Any]:
    runtime_root = Path(root).absolute()
    _require_directory(runtime_root)
    database = runtime_root / "bioxp_runtime.db"
    health = inspect_database(database)
    usage = shutil.disk_usage(runtime_root)
    allocated = int(allocation_bytes if allocation_bytes is not None else usage.total)
    if allocated <= 0:
        raise StorageEvidenceError("capacity allocation must be positive")
    connection = _read_only_connection(database)
    try:
        tables = set(health["tables"])
        command_rows = connection.execute("SELECT started_at FROM operator_commands").fetchall()
        daily: dict[str, int] = {}
        timestamps: list[float] = []
        for row in command_rows:
            stamp = _parse_epoch(row[0])
            if stamp is None:
                continue
            timestamps.append(stamp)
            day = datetime.fromtimestamp(stamp, timezone.utc).date().isoformat()
            daily[day] = daily.get(day, 0) + 1
        observed_days = len(daily)
        sample_counts = {
            table: int(connection.execute(f'SELECT COUNT(*) FROM "{table}"').fetchone()[0])
            if table in tables
            else 0
            for table in _REQUIRED_SAMPLE_TABLES
        }
        database_bytes = int(database.stat().st_size)
        wal = database.with_name(database.name + "-wal")
        wal_bytes = wal.stat().st_size if wal.exists() and not wal.is_symlink() else 0
        evidence_bytes = sum(
            int(row["byte_count"])
            for relative in _GOVERNED_EVIDENCE_ROOTS
            for row in _scan_tree(runtime_root, relative, "capacity")
        )
        backup_bytes = _tree_bytes(runtime_root / "backups") if (runtime_root / "backups").exists() else 0
        command_count = len(command_rows)
        span_days = (
            (max(timestamps) - min(timestamps)) / (24 * 60 * 60)
            if len(timestamps) >= 2
            else 0.0
        )
        representative_reasons: list[str] = []
        if command_count < _MIN_REPRESENTATIVE_COMMANDS:
            representative_reasons.append("command_sample_below_100")
        if observed_days < _MIN_REPRESENTATIVE_DAYS or span_days < (_MIN_REPRESENTATIVE_DAYS - 1):
            representative_reasons.append("observation_window_below_7_days")
        sample_requirements: dict[str, int] = {}
        for table, count in sample_counts.items():
            minimum, ratio, denominator = _REPRESENTATIVE_SAMPLE_POLICY[table]
            base_count = command_count if denominator == "commands" else sample_counts["pipette_operations"]
            required = max(int(minimum), int(base_count * float(ratio) + 0.999999))
            sample_requirements[table] = required
            if count < required:
                representative_reasons.append(
                    f"nonrepresentative_{table}_sample:{count}_below_{required}"
                )
        if evidence_bytes <= 0:
            representative_reasons.append("missing_full_evidence_sample")
        mean_daily_commands = command_count / observed_days if observed_days else 0.0
        peak_daily_commands = max(daily.values(), default=0)
        years_days = 365 * 5 + 1
        projected_commands = int(round(mean_daily_commands * years_days))
        per_command_database = database_bytes / command_count if command_count else 0.0
        per_command_evidence = evidence_bytes / command_count if command_count else 0.0
        projected_database = max(database_bytes, int(round(per_command_database * projected_commands)))
        projected_evidence = max(evidence_bytes, int(round(per_command_evidence * projected_commands)))
        overhead_ratio = (
            (wal_bytes + backup_bytes) / max(1, database_bytes + evidence_bytes)
        )
        projected_overhead = int(round((projected_database + projected_evidence) * overhead_ratio))
        projected_total = projected_database + projected_evidence + projected_overhead
        threshold = int(allocated * 0.60)
        if representative_reasons:
            status = "insufficient_evidence"
            confidence = "nonrepresentative_sample"
        else:
            status = "pass" if projected_total <= threshold else "fail"
            confidence = "representative_observed_workload_projection"
        return {
            "schema": _CAPACITY_SCHEMA,
            "status": status,
            "confidence": confidence,
            "generated_at": _utc_now(),
            "allocation_bytes": allocated,
            "free_bytes": usage.free,
            "threshold_bytes": threshold,
            "representative_sample": not representative_reasons,
            "insufficient_evidence_reasons": representative_reasons,
            "observed": {
                "command_count": command_count,
                "observed_days": observed_days,
                "observed_span_days": span_days,
                "peak_commands_per_day": peak_daily_commands,
                "mean_commands_per_day": mean_daily_commands,
                "first_command_at": min(timestamps) if timestamps else None,
                "last_command_at": max(timestamps) if timestamps else None,
                "database_bytes": database_bytes,
                "wal_bytes": wal_bytes,
                "evidence_bytes": evidence_bytes,
                "backup_bytes": backup_bytes,
                "sample_counts": sample_counts,
            },
            "five_year_projection": {
                "days": years_days,
                "commands": projected_commands,
                "database_bytes": projected_database,
                "evidence_bytes": projected_evidence,
                "wal_backup_overhead_bytes": projected_overhead,
                "total_bytes": projected_total,
                "headroom_bytes": allocated - projected_total,
            },
            "database": health,
        }
    finally:
        connection.close()


def _read_json_receipt(path: Path) -> dict[str, Any] | None:
    try:
        if not path.exists():
            return None
        if path.is_symlink() or not path.is_file():
            raise StorageEvidenceError(f"health receipt is not a regular file: {path}")
        value = json.loads(path.read_bytes())
    except StorageEvidenceError:
        raise
    except (OSError, ValueError, TypeError, UnicodeError, json.JSONDecodeError) as exc:
        raise StorageEvidenceError(f"health receipt is unreadable: {path}: {str(exc)[:300]}") from exc
    if not isinstance(value, dict):
        raise StorageEvidenceError(f"health receipt is not an object: {path}")
    return value


def _latest_backup_receipt(root: Path) -> dict[str, Any] | None:
    receipt_root = root / "backups" / "receipts"
    if not receipt_root.exists():
        return None
    _require_directory(receipt_root)
    verified_rows: list[dict[str, Any]] = []
    for path in receipt_root.iterdir():
        if path.is_symlink() or not path.is_file():
            continue
        try:
            receipt = _read_json_receipt(path)
            if receipt is None or receipt.get("schema") != _BACKUP_RECEIPT_SCHEMA:
                continue
            backup_id = str(receipt.get("backup_id") or "")
            unit = root / "backups" / backup_id
            verified = verify_backup_unit(unit)
            manifest_raw, _, _ = _read_regular(unit, "manifest.json")
            manifest = json.loads(manifest_raw)
            bound = _verify_external_backup_receipt(root, unit, verified, manifest)
            verified_rows.append({**bound, "verified_unit": verified})
        except (OSError, ValueError, TypeError, json.JSONDecodeError, StorageEvidenceError):
            continue
    if not verified_rows:
        return None
    return max(verified_rows, key=lambda row: float(row["verified_unit"]["publication_epoch"]))


def audit_health_report(
    root: str | Path,
    *,
    connection: sqlite3.Connection | None = None,
    writer_health: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    runtime_root = Path(root).absolute()
    _require_directory(runtime_root)
    database = runtime_root / "bioxp_runtime.db"
    selected_now = time.time()
    owned_connection = connection is None
    selected_connection = _read_only_connection(database) if owned_connection else connection
    if selected_connection is None:
        raise StorageEvidenceError("audit health connection is unavailable")
    checks: dict[str, Any] = {}
    degraded: list[str] = []
    schema_identity: dict[str, Any] | None = None
    authoritative_database_path = str(database.resolve())
    try:
        try:
            schema_issues = _schema_contract_issues(selected_connection)
        except (sqlite3.Error, ValueError, StorageEvidenceError) as exc:
            schema_issues = [{"check": "schema_attestation_error", "error": str(exc)[:500]}]
        checks["schema"] = {"status": "ok" if not schema_issues else "degraded", "issues": schema_issues}
        if schema_issues:
            degraded.append("schema_or_migration_identity")
        try:
            schema_identity = _schema_identity(selected_connection)
            main_row = next(
                row
                for row in selected_connection.execute("PRAGMA database_list").fetchall()
                if str(row[1]) == "main"
            )
            opened_database_path = str(Path(str(main_row[2])).resolve())
            identity_row = selected_connection.execute(
                "SELECT database_path,schema_version FROM runtime_store_identity WHERE identity_id=1"
            ).fetchone()
            recorded_database_path = None if identity_row is None else str(identity_row["database_path"])
            recorded_schema_version = None if identity_row is None else int(identity_row["schema_version"])
            current_schema_version = int(schema_identity["user_version"])
            store_identity_ok = (
                identity_row is not None
                and opened_database_path == authoritative_database_path
                and recorded_database_path == authoritative_database_path
                and recorded_schema_version == current_schema_version == SCHEMA_VERSION
            )
            checks["store_identity"] = {
                "status": "ok" if store_identity_ok else "degraded",
                "authoritative_database_path": authoritative_database_path,
                "opened_database_path": opened_database_path,
                "recorded_database_path": recorded_database_path,
                "recorded_schema_version": recorded_schema_version,
                "current_schema_version": current_schema_version,
            }
        except (StopIteration, OSError, sqlite3.Error, TypeError, ValueError) as exc:
            store_identity_ok = False
            checks["store_identity"] = {
                "status": "degraded",
                "authoritative_database_path": authoritative_database_path,
                "error": str(exc)[:500],
            }
        if not store_identity_ok:
            degraded.append("runtime_store_identity")
        try:
            integrity = [str(row[0]) for row in selected_connection.execute("PRAGMA integrity_check").fetchall()]
        except sqlite3.Error as exc:
            integrity = [f"integrity_check_error:{str(exc)[:500]}"]
        checks["integrity"] = {"status": "ok" if integrity == ["ok"] else "degraded", "rows": integrity}
        if integrity != ["ok"]:
            degraded.append("integrity_check")
        try:
            foreign_keys = [tuple(row) for row in selected_connection.execute("PRAGMA foreign_key_check").fetchall()]
        except sqlite3.Error as exc:
            foreign_keys = [("foreign_key_check_error", str(exc)[:500])]
        checks["foreign_keys"] = {"status": "ok" if not foreign_keys else "degraded", "rows": foreign_keys}
        if foreign_keys:
            degraded.append("foreign_key_check")
        try:
            journal_mode = str(selected_connection.execute("PRAGMA journal_mode").fetchone()[0]).lower()
            synchronous = int(selected_connection.execute("PRAGMA synchronous").fetchone()[0])
        except (sqlite3.Error, TypeError, ValueError) as exc:
            journal_mode = f"error:{str(exc)[:500]}"
            synchronous = -1
        checks["durability"] = {
            "status": "ok" if journal_mode == "wal" and synchronous == 2 else "degraded",
            "journal_mode": journal_mode,
            "synchronous": synchronous,
        }
        if checks["durability"]["status"] != "ok":
            degraded.append("sqlite_durability")
        checkpoint_error: str | None = None
        try:
            checkpoint = _read_json_receipt(runtime_root / "health_receipts" / "last-checkpoint.json")
            checkpoint_epoch = None if checkpoint is None else _parse_epoch(checkpoint.get("created_epoch"))
            checkpoint_age = None if checkpoint_epoch is None else selected_now - checkpoint_epoch
            checkpoint_payload = None if checkpoint is None else checkpoint.get("checkpoint")
            recomputed_checkpoint_identity = None
            if checkpoint is not None and isinstance(checkpoint_payload, Mapping):
                recomputed_checkpoint_identity = _sha256_bytes(
                    _canonical_bytes(
                        {
                            "database_path": checkpoint.get("database_path"),
                            "database_sha256": checkpoint.get("database_sha256"),
                            "schema_version": checkpoint.get("schema_version"),
                            "schema_sha256": checkpoint.get("schema_sha256"),
                            "checkpoint": dict(checkpoint_payload),
                        }
                    )
                )
            checkpoint_ok = (
                checkpoint is not None
                and schema_identity is not None
                and checkpoint.get("schema") == _CHECKPOINT_RECEIPT_SCHEMA
                and checkpoint.get("status") == "verified"
                and checkpoint.get("database_path") == authoritative_database_path
                and type(checkpoint.get("schema_version")) is int
                and checkpoint.get("schema_version") == schema_identity["user_version"]
                and checkpoint.get("schema_sha256") == schema_identity["schema_sha256"]
                and isinstance(checkpoint.get("database_sha256"), str)
                and bool(checkpoint.get("database_sha256"))
                and recomputed_checkpoint_identity == checkpoint.get("checkpoint_identity")
                and checkpoint_age is not None
                and 0 <= checkpoint_age <= 24 * 60 * 60
            )
        except (OSError, ValueError, TypeError, UnicodeError, json.JSONDecodeError, StorageEvidenceError) as exc:
            checkpoint = None
            checkpoint_age = None
            recomputed_checkpoint_identity = None
            checkpoint_ok = False
            checkpoint_error = str(exc)[:500]
        checks["checkpoint"] = {
            "status": "ok" if checkpoint_ok else "degraded",
            "age_seconds": checkpoint_age,
            "receipt": checkpoint,
            "recomputed_checkpoint_identity": recomputed_checkpoint_identity,
            "error": checkpoint_error,
        }
        if not checkpoint_ok:
            degraded.append("checkpoint_freshness_or_identity")
        try:
            backup = _latest_backup_receipt(runtime_root)
            backup_epoch = (
                None
                if backup is None
                else _parse_epoch(backup["verified_unit"].get("publication_epoch"))
            )
            backup_age = None if backup_epoch is None else selected_now - backup_epoch
            backup_ok = backup is not None and backup_age is not None and 0 <= backup_age <= 24 * 60 * 60
            checks["backup"] = {
                "status": "ok" if backup_ok else "degraded",
                "age_seconds": backup_age,
                "backup_id": None if backup is None else backup.get("backup_id"),
            }
        except (OSError, ValueError, json.JSONDecodeError, StorageEvidenceError) as exc:
            backup_ok = False
            checks["backup"] = {"status": "degraded", "error": str(exc)[:500]}
        if not backup_ok:
            degraded.append("backup_freshness_or_integrity")
        writer = dict(writer_health or {})
        writer_status = writer.get("status")
        queue_depth = writer.get("queue_depth")
        writer_ok = writer_status == "ok" and type(queue_depth) is int and queue_depth >= 0
        checks["writer"] = {
            "status": "ok" if writer_ok else "degraded",
            "writer_status": writer_status,
            "queue_depth": queue_depth,
            "telemetry_available": writer_health is not None,
            "mode": writer.get("mode"),
            "error": writer.get("error"),
        }
        if not writer_ok:
            degraded.append("audit_writer_evidence")
        usage = shutil.disk_usage(runtime_root)
        database_bytes = database.stat().st_size
        wal = database.with_name(database.name + "-wal")
        wal_bytes = wal.stat().st_size if wal.exists() and not wal.is_symlink() else 0
        wal_threshold = min(1024**3, max(1, int(usage.total * 0.10)))
        minimum_free = max(1024**3, int(usage.total * 0.10))
        try:
            evidence_bytes = sum(
                int(row["byte_count"])
                for relative in _GOVERNED_EVIDENCE_ROOTS
                for row in _scan_tree(runtime_root, relative, "health")
            )
            backup_bytes = _tree_bytes(runtime_root / "backups") if (runtime_root / "backups").exists() else 0
            storage_error = None
        except (OSError, StorageEvidenceError) as exc:
            evidence_bytes = None
            backup_bytes = None
            storage_error = str(exc)[:500]
        storage_ok = (
            storage_error is None
            and usage.free >= minimum_free
            and wal_bytes <= wal_threshold
        )
        checks["storage"] = {
            "status": "ok" if storage_ok else "degraded",
            "database_bytes": database_bytes,
            "wal_bytes": wal_bytes,
            "wal_threshold_bytes": wal_threshold,
            "free_bytes": usage.free,
            "minimum_free_bytes": minimum_free,
            "evidence_bytes": evidence_bytes,
            "backup_bytes": backup_bytes,
            "error": storage_error,
        }
        if not storage_ok:
            degraded.append("storage_threshold")
        def bounded_count(table: str, where: str = "1=1") -> int | None:
            try:
                return int(
                    selected_connection.execute(
                        f'SELECT COUNT(*) FROM "{table}" WHERE {where}'
                    ).fetchone()[0]
                )
            except (sqlite3.Error, TypeError, ValueError):
                return None

        counts = {
            "commands": bounded_count("operator_commands"),
            "pipette_operations": bounded_count("pipette_operations"),
            "retained_evidence": bounded_count(
                "runtime_evidence_objects",
                "expiry_state IN ('active','retained')",
            ),
            "pending_expiry_evidence": bounded_count(
                "runtime_evidence_objects",
                "expiry_state='expiry_pending'",
            ),
            "integrity_failures": bounded_count(
                "runtime_evidence_events",
                "event_kind='integrity_failure'",
            ),
        }
        return {
            "schema": _HEALTH_SCHEMA,
            "status": "ok" if not degraded else "degraded",
            "generated_at": _utc_now(),
            "degraded_reasons": degraded,
            "checks": checks,
            "counts": counts,
            "release_identity": current_release_identity(),
            "physical_admission_gate_added": False,
        }
    finally:
        if owned_connection:
            selected_connection.close()


def main(argv: Iterable[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Verify BioXP runtime storage and recovery evidence")
    parser.add_argument("operation", choices=("backup", "restore", "capacity", "checkpoint", "health"))
    parser.add_argument("--root", default=os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT", "/var/lib/bioxp-oem-runtime"))
    parser.add_argument("--unit")
    parser.add_argument("--target")
    parser.add_argument("--label", default=f"runtime-backup-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}")
    parser.add_argument("--allocation-bytes", type=int)
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        if args.operation == "backup":
            result = create_backup_unit(args.root, label=args.label, phase="scheduled")
        elif args.operation == "restore":
            if not args.unit:
                parser.error("restore requires --unit")
            result = restore_drill(args.unit, target=args.target)
        elif args.operation == "checkpoint":
            result = checkpoint_database(Path(args.root) / "bioxp_runtime.db")
        elif args.operation == "health":
            result = audit_health_report(args.root)
        else:
            result = capacity_report(args.root, args.allocation_bytes)
        print(json.dumps(result, sort_keys=True, indent=2))
        return 0 if result.get("status") in {"verified", "source_verified", "pass", "ok"} else 1
    except (OSError, sqlite3.Error, StorageEvidenceError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, sort_keys=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
