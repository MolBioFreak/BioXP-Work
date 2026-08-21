from __future__ import annotations

import hashlib
import json
import os
import shutil
import sqlite3
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping
from urllib.parse import quote


class StorageEvidenceError(RuntimeError):
    """A runtime storage evidence operation failed closed."""


_BACKUP_SCHEMA = "bioxp.runtime.backup.v1"
_CAPACITY_SCHEMA = "bioxp.runtime.capacity.v1"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    fd = os.open(str(path), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{time.time_ns()}.tmp")
    try:
        with temporary.open("wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _read_only_connection(path: Path) -> sqlite3.Connection:
    uri = f"file:{quote(str(path.resolve()))}?mode=ro"
    connection = sqlite3.connect(uri, uri=True, timeout=5.0)
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA query_only=ON")
    connection.execute("PRAGMA foreign_keys=ON")
    return connection


def inspect_database(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise StorageEvidenceError(f"runtime database is not a regular file: {path}")
    connection = _read_only_connection(path)
    try:
        quick = [str(row[0]) for row in connection.execute("PRAGMA quick_check").fetchall()]
        foreign_keys = [tuple(row) for row in connection.execute("PRAGMA foreign_key_check").fetchall()]
        user_version = int(connection.execute("PRAGMA user_version").fetchone()[0])
        journal_mode = str(connection.execute("PRAGMA journal_mode").fetchone()[0]).lower()
        synchronous = int(connection.execute("PRAGMA synchronous").fetchone()[0])
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
            "runtime_migration_receipts",
            "runtime_migration_retirements",
            "report_exports",
        ):
            if table in tables:
                counts[table] = int(connection.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0])
        migration_ledger = []
        if "runtime_schema_migrations" in tables:
            migration_ledger = [
                {"version": int(row[0]), "name": str(row[1]), "ddl_sha256": str(row[2])}
                for row in connection.execute(
                    "SELECT version,name,ddl_sha256 FROM runtime_schema_migrations ORDER BY version"
                ).fetchall()
            ]
        return {
            "path": str(path),
            "byte_count": path.stat().st_size,
            "sha256": sha256_file(path),
            "user_version": user_version,
            "journal_mode": journal_mode,
            "synchronous": synchronous,
            "quick_check": quick,
            "foreign_key_check": foreign_keys,
            "tables": tables,
            "counts": counts,
            "migration_ledger": migration_ledger,
        }
    finally:
        connection.close()


def _safe_relative(path: Path, root: Path) -> Path:
    if path.is_absolute():
        raise StorageEvidenceError(f"absolute storage path is not allowed: {path}")
    if any(part in {"", ".", ".."} for part in path.parts):
        raise StorageEvidenceError(f"unsafe storage path: {path}")
    return path


def _inventory_tree(root: Path, relative_root: str, kind: str) -> list[dict[str, Any]]:
    selected = root / relative_root
    if not selected.exists():
        return []
    if selected.is_symlink() or not selected.is_dir():
        raise StorageEvidenceError(f"storage evidence root is not a directory: {selected}")
    rows: list[dict[str, Any]] = []
    for path in sorted(selected.rglob("*")):
        if path.is_symlink() or not path.is_file():
            if path.is_symlink():
                raise StorageEvidenceError(f"symlink in governed evidence root: {path}")
            continue
        relative = _safe_relative(path.relative_to(root), root)
        stat = path.stat()
        rows.append(
            {
                "kind": kind,
                "source_relpath": relative.as_posix(),
                "byte_count": stat.st_size,
                "sha256": sha256_file(path),
            }
        )
    return rows


def _copy_stable(source: Path, destination: Path) -> dict[str, Any]:
    if source.is_symlink() or not source.is_file():
        raise StorageEvidenceError(f"source is not a regular file: {source}")
    before = source.stat()
    before_digest = sha256_file(source)
    destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.{time.time_ns()}.tmp")
    try:
        with source.open("rb") as src, temporary.open("wb") as dst:
            shutil.copyfileobj(src, dst, length=1024 * 1024)
            dst.flush()
            os.fsync(dst.fileno())
        after = source.stat()
        after_digest = sha256_file(source)
        if (before.st_size, before.st_mtime_ns, before_digest) != (
            after.st_size,
            after.st_mtime_ns,
            after_digest,
        ):
            raise StorageEvidenceError(f"source changed during backup: {source}")
        if sha256_file(temporary) != before_digest:
            raise StorageEvidenceError(f"backup digest mismatch: {source}")
        os.replace(temporary, destination)
        _fsync_directory(destination.parent)
        return {"byte_count": before.st_size, "sha256": before_digest}
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _online_backup(source: Path, destination: Path) -> dict[str, Any]:
    destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.{time.time_ns()}.tmp")
    source_connection = _read_only_connection(source)
    try:
        target = sqlite3.connect(temporary, timeout=5.0)
        try:
            target.execute("PRAGMA journal_mode=DELETE")
            target.execute("PRAGMA synchronous=FULL")
            source_connection.backup(target, pages=128, sleep=0.05)
            target.commit()
            target.execute("PRAGMA wal_checkpoint(TRUNCATE)")
            target.close()
        except Exception:
            target.close()
            raise
        os.replace(temporary, destination)
        _fsync_directory(destination.parent)
        return {"byte_count": destination.stat().st_size, "sha256": sha256_file(destination)}
    finally:
        source_connection.close()
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _source_files(root: Path) -> list[dict[str, Any]]:
    files: list[dict[str, Any]] = []
    for relative_root, kind in (
        ("operator_evidence", "operator_evidence"),
        ("pipette/evidence", "pipette_evidence"),
        ("report_exports", "report_export"),
        ("artifacts", "runtime_artifact"),
        ("archive", "migration_archive"),
    ):
        files.extend(_inventory_tree(root, relative_root, kind))
    legacy = root / "pipette" / "receipts.jsonl"
    if legacy.exists():
        if legacy.is_symlink() or not legacy.is_file():
            raise StorageEvidenceError(f"legacy receipt source is not a regular file: {legacy}")
        files.append(
            {
                "kind": "legacy_jsonl",
                "source_relpath": "pipette/receipts.jsonl",
                "byte_count": legacy.stat().st_size,
                "sha256": sha256_file(legacy),
            }
        )
    return sorted(files, key=lambda row: (str(row["kind"]), str(row["source_relpath"])))


def _write_sha256sums(unit: Path) -> str:
    rows: list[str] = []
    for path in sorted(unit.rglob("*")):
        if not path.is_file() or path.name == "SHA256SUMS":
            continue
        rows.append(f"{sha256_file(path)}  {path.relative_to(unit).as_posix()}")
    data = ("\n".join(rows) + "\n").encode("utf-8")
    _write_bytes(unit / "SHA256SUMS", data)
    return hashlib.sha256(data).hexdigest()


def verify_backup_unit(unit: str | Path) -> dict[str, Any]:
    backup_root = Path(unit).resolve()
    manifest_path = backup_root / "manifest.json"
    if not manifest_path.is_file():
        raise StorageEvidenceError(f"backup manifest is missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != _BACKUP_SCHEMA:
        raise StorageEvidenceError("unsupported backup manifest schema")
    sums_path = backup_root / "SHA256SUMS"
    if not sums_path.is_file():
        raise StorageEvidenceError("backup SHA256SUMS is missing")
    for line in sums_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        digest, relative = line.split("  ", 1)
        path = backup_root / _safe_relative(Path(relative), backup_root)
        if not path.is_file() or path.is_symlink() or sha256_file(path) != digest:
            raise StorageEvidenceError(f"backup checksum mismatch: {relative}")
    database = backup_root / "bioxp_runtime.db"
    health = inspect_database(database)
    if health["quick_check"] != ["ok"] or health["foreign_key_check"]:
        raise StorageEvidenceError("restored backup database failed integrity checks")
    return {"status": "verified", "unit": str(backup_root), "database": health}


def create_backup_unit(
    root: str | Path,
    *,
    label: str,
    phase: str,
    source_kind: str | None = None,
    source_digest: str | None = None,
) -> dict[str, Any]:
    runtime_root = Path(root).resolve()
    database = runtime_root / "bioxp_runtime.db"
    if not database.is_file():
        raise StorageEvidenceError(f"runtime database is missing: {database}")
    unit = runtime_root / "backups" / label
    unit.mkdir(parents=True, exist_ok=True, mode=0o700)
    source_health = inspect_database(database)
    if source_health["quick_check"] != ["ok"] or source_health["foreign_key_check"]:
        raise StorageEvidenceError("source database failed integrity checks before backup")
    database_backup = _online_backup(database, unit / "bioxp_runtime.db")
    files = _source_files(runtime_root)
    for row in files:
        source = runtime_root / row["source_relpath"]
        destination = unit / "evidence" / row["source_relpath"]
        copied = _copy_stable(source, destination)
        if copied != {"byte_count": row["byte_count"], "sha256": row["sha256"]}:
            raise StorageEvidenceError(f"copied source does not match manifest: {source}")
        row["backup_relpath"] = destination.relative_to(unit).as_posix()
    manifest = {
        "schema": _BACKUP_SCHEMA,
        "created_at": _utc_now(),
        "phase": str(phase),
        "source_kind": source_kind,
        "source_digest": source_digest,
        "source_root": str(runtime_root),
        "source_database": source_health,
        "backup_database": database_backup,
        "files": files,
        "file_count": len(files),
        "file_bytes": sum(int(row["byte_count"]) for row in files),
        "source_deployment": {
            "commit": os.environ.get("BIOXP_SOURCE_COMMIT"),
            "tree": os.environ.get("BIOXP_SOURCE_TREE"),
        },
    }
    _write_bytes(unit / "manifest.json", (json.dumps(manifest, sort_keys=True, indent=2) + "\n").encode("utf-8"))
    _write_sha256sums(unit)
    verified = verify_backup_unit(unit)
    receipt = {
        "schema": "bioxp.runtime.backup.receipt.v1",
        "status": "verified",
        "unit": str(unit),
        "manifest_sha256": sha256_file(unit / "manifest.json"),
        "sha256sums_sha256": sha256_file(unit / "SHA256SUMS"),
        "database_sha256": sha256_file(unit / "bioxp_runtime.db"),
        "file_count": len(files),
        "file_bytes": sum(int(row["byte_count"]) for row in files),
        "database": verified["database"],
    }
    _write_bytes(unit / "backup-receipt.json", (json.dumps(receipt, sort_keys=True, indent=2) + "\n").encode("utf-8"))
    _write_sha256sums(unit)
    return receipt


def restore_drill(unit: str | Path) -> dict[str, Any]:
    backup_root = Path(unit).resolve()
    verified = verify_backup_unit(backup_root)
    manifest = json.loads((backup_root / "manifest.json").read_text(encoding="utf-8"))
    temporary = Path(tempfile.mkdtemp(prefix="bioxp-restore-drill-", dir=backup_root.parent))
    restored_files = 0
    try:
        restored_db = temporary / "bioxp_runtime.db"
        _copy_stable(backup_root / "bioxp_runtime.db", restored_db)
        restored_db_health = inspect_database(restored_db)
        for row in manifest.get("files", []):
            source = backup_root / _safe_relative(Path(str(row["backup_relpath"])), backup_root)
            destination = temporary / "evidence" / _safe_relative(Path(str(row["source_relpath"])), temporary)
            _copy_stable(source, destination)
            if destination.stat().st_size != int(row["byte_count"]) or sha256_file(destination) != str(row["sha256"]):
                raise StorageEvidenceError(f"restore evidence digest mismatch: {row['source_relpath']}")
            restored_files += 1
        return {
            "schema": "bioxp.runtime.restore.receipt.v1",
            "status": "verified",
            "backup_unit": str(backup_root),
            "restored_database": restored_db_health,
            "restored_file_count": restored_files,
            "source_backup_verified": verified["status"] == "verified",
        }
    finally:
        shutil.rmtree(temporary, ignore_errors=True)


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


def capacity_report(root: str | Path, allocation_bytes: int | None = None) -> dict[str, Any]:
    runtime_root = Path(root).resolve()
    database = runtime_root / "bioxp_runtime.db"
    health = inspect_database(database)
    usage = shutil.disk_usage(runtime_root)
    allocated = int(allocation_bytes or usage.total)
    connection = _read_only_connection(database)
    try:
        rows = connection.execute("SELECT started_at FROM operator_commands").fetchall()
        daily: dict[str, int] = {}
        timestamps: list[float] = []
        for row in rows:
            stamp = _parse_epoch(row[0])
            if stamp is None:
                continue
            timestamps.append(stamp)
            day = datetime.fromtimestamp(stamp, timezone.utc).date().isoformat()
            daily[day] = daily.get(day, 0) + 1
        observed_days = max(1, len(daily))
        peak_daily_commands = max(daily.values(), default=len(rows))
        mean_daily_commands = len(rows) / observed_days
        db_bytes = int(database.stat().st_size)
        evidence_bytes = 0
        evidence_files = 0
        for relative in ("operator_evidence", "pipette/evidence", "report_exports", "artifacts", "archive"):
            selected = runtime_root / relative
            if selected.exists():
                for path in selected.rglob("*"):
                    if path.is_file() and not path.is_symlink():
                        evidence_files += 1
                        evidence_bytes += path.stat().st_size
        years_days = 365 * 5 + 1
        projected_commands = int(round(mean_daily_commands * years_days))
        per_command_db = db_bytes / max(1, len(rows))
        per_command_evidence = evidence_bytes / max(1, len(rows))
        projected_db = max(db_bytes, int(round(per_command_db * projected_commands)))
        projected_evidence = max(evidence_bytes, int(round(per_command_evidence * projected_commands)))
        projected_total = projected_db + projected_evidence
        threshold = int(allocated * 0.60)
        return {
            "schema": _CAPACITY_SCHEMA,
            "status": "pass" if projected_total <= threshold else "fail",
            "confidence": "observed_workload_projection",
            "generated_at": _utc_now(),
            "root": str(runtime_root),
            "allocation_bytes": allocated,
            "free_bytes": usage.free,
            "threshold_bytes": threshold,
            "observed": {
                "command_count": len(rows),
                "observed_days": observed_days,
                "peak_commands_per_day": peak_daily_commands,
                "mean_commands_per_day": mean_daily_commands,
                "first_command_at": min(timestamps) if timestamps else None,
                "last_command_at": max(timestamps) if timestamps else None,
                "database_bytes": db_bytes,
                "evidence_bytes": evidence_bytes,
                "evidence_files": evidence_files,
            },
            "five_year_projection": {
                "days": years_days,
                "commands": projected_commands,
                "database_bytes": projected_db,
                "evidence_bytes": projected_evidence,
                "total_bytes": projected_total,
                "headroom_bytes": allocated - projected_total,
            },
            "database": health,
        }
    finally:
        connection.close()


def main(argv: Iterable[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Verify BioXP runtime storage and recovery evidence")
    parser.add_argument("operation", choices=("backup", "restore", "capacity"))
    parser.add_argument("--root", default=os.environ.get("BIOXP_OEM_RUNTIME_STATE_ROOT", "/var/lib/bioxp-oem-runtime"))
    parser.add_argument("--unit")
    parser.add_argument("--label", default=f"runtime-backup-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}")
    parser.add_argument("--allocation-bytes", type=int)
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        if args.operation == "backup":
            result = create_backup_unit(args.root, label=args.label, phase="scheduled")
        elif args.operation == "restore":
            if not args.unit:
                parser.error("restore requires --unit")
            result = restore_drill(args.unit)
        else:
            result = capacity_report(args.root, args.allocation_bytes)
        print(json.dumps(result, sort_keys=True, indent=2))
        return 0 if result.get("status") in {"verified", "pass"} else 1
    except (OSError, sqlite3.Error, StorageEvidenceError, ValueError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, sort_keys=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
