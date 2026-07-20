from __future__ import annotations

import hashlib
import fcntl
import json
import os
import stat
import threading
import uuid
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from contextlib import contextmanager, nullcontext
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Any, Mapping

from .oem_machine_bundle import (
    OemMachineSnapshot,
    parse_operation_parameters_bytes,
)


OEM_RUNTIME_STATE_ROOT_ENV = "BIOXP_OEM_RUNTIME_STATE_ROOT"
_JOURNAL_NAME = "transaction_provenance.jsonl"
_LOCK_NAME = "transaction.lock"
_STATE_PATHS = frozenset({
    "appdata/Operation_parameters.xml",
    "appdata/processtime.xml",
    "appdata/pressurebuffer.txt",
    "appdata_parent/Config_History/config_history.csv",
})
_STATE_DIRECTORIES = frozenset({"appdata", "appdata_parent", "appdata_parent/Config_History"})
_active_store_lock = threading.Lock()
_active_store: "OemRuntimeStateStore | None" = None


class OemRuntimeStateError(RuntimeError):
    """The separately versioned OEM-writable state failed closed."""


@dataclass(frozen=True)
class RuntimeStateTransaction:
    transaction_id: str
    timestamp_utc: str
    relative_path: str
    operation: str
    writer: str
    call_path: str
    baseline_sha256: str
    before_sha256: str | None
    after_sha256: str | None
    outcome: str
    error: str | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            "transaction_id": self.transaction_id,
            "timestamp_utc": self.timestamp_utc,
            "relative_path": self.relative_path,
            "operation": self.operation,
            "writer": self.writer,
            "call_path": self.call_path,
            "baseline_sha256": self.baseline_sha256,
            "before_sha256": self.before_sha256,
            "after_sha256": self.after_sha256,
            "outcome": self.outcome,
            "error": self.error,
        }


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _relative(raw: str) -> str:
    path = PurePosixPath(raw)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise OemRuntimeStateError(f"invalid runtime-state relative path: {raw!r}")
    value = path.as_posix()
    if value not in _STATE_PATHS:
        raise OemRuntimeStateError(f"record is not OEM-writable in this tranche: {value}")
    return value


def _assert_unique_regular(path: Path) -> os.stat_result:
    try:
        info = path.lstat()
    except FileNotFoundError as exc:
        raise OemRuntimeStateError(f"runtime-state file is missing: {path}") from exc
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise OemRuntimeStateError(f"runtime-state file must be a unique regular file: {path}")
    return info


def _read_no_follow(path: Path) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise OemRuntimeStateError(f"cannot open runtime-state file {path}: {exc}") from exc
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise OemRuntimeStateError(f"runtime-state file must be a unique regular file: {path}")
        chunks: list[bytes] = []
        while True:
            chunk = os.read(fd, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(fd)


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0)
    fd = os.open(path, flags)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _atomic_replace(path: Path, data: bytes) -> None:
    temporary = path.parent / f".{path.name}.{uuid.uuid4().hex}.tmp"
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(temporary, flags, 0o600)
    try:
        view = memoryview(data)
        while view:
            written = os.write(fd, view)
            if written <= 0:
                raise OemRuntimeStateError(f"short atomic write for {path}")
            view = view[written:]
        os.fsync(fd)
    except Exception:
        os.close(fd)
        try:
            temporary.unlink()
        except OSError:
            pass
        raise
    else:
        os.close(fd)
    os.replace(temporary, path)
    _fsync_directory(path.parent)


def _validate_record_bytes(relative_path: str, data: bytes) -> None:
    if relative_path == "appdata/Operation_parameters.xml":
        parse_operation_parameters_bytes(
            data,
            record_relative_path=relative_path,
            require_serial206_baseline=False,
        )
        return
    if relative_path == "appdata/processtime.xml":
        try:
            root = ET.fromstring(data)
        except ET.ParseError as exc:
            raise OemRuntimeStateError(f"transactional processtime.xml is malformed: {exc}") from exc
        process = next((element for element in root.iter() if element.tag.rsplit("}", 1)[-1] == "processTime"), None)
        if process is None or not list(process):
            raise OemRuntimeStateError("transactional processtime.xml lacks processTime entries")
        for element in list(process):
            if "process" not in element.attrib:
                raise OemRuntimeStateError("transactional process-time entry lacks process attribute")
            float(element.attrib["process"])


class OemRuntimeStateStore:
    """Four-record, baseline-versioned, atomic OEM mutable state authority."""

    def __init__(self, root: str | Path, snapshot: OemMachineSnapshot) -> None:
        if not snapshot.mutation_authorized:
            raise OemRuntimeStateError("runtime state cannot be seeded before physical-label/machine authority is matched")
        base = Path(root)
        if not base.is_absolute():
            raise OemRuntimeStateError("runtime-state root must be explicit and absolute")
        if ".." in base.parts:
            raise OemRuntimeStateError("indirected runtime-state root is forbidden")
        state_root = base.resolve(strict=False)
        evidence_root = snapshot.bundle_root.resolve(strict=True)
        if state_root == evidence_root or state_root.is_relative_to(evidence_root) or evidence_root.is_relative_to(state_root):
            raise OemRuntimeStateError("runtime-state root must be separate from immutable evidence")
        self.snapshot = snapshot
        self.base_root = base
        self.version_root = base / f"serial-{snapshot.machine_serial}" / snapshot.lock_sha256
        self._journal_path = self.version_root / _JOURNAL_NAME
        self._transaction_lock_path = self.version_root / _LOCK_NAME
        self._lock = threading.RLock()
        self._latest: dict[str, RuntimeStateTransaction] = {}
        self._ensure_directories()
        with self._transaction_guard():
            self._load_journal()
            self._seed_or_verify_all()
            operation, _ = parse_operation_parameters_bytes(
                self._verified_read("appdata/Operation_parameters.xml"),
                record_relative_path="appdata/Operation_parameters.xml",
                require_serial206_baseline=False,
            )
            self._operation_parameters = dict(operation)
        self._status = self._build_status()

    @classmethod
    def from_environment(cls, snapshot: OemMachineSnapshot) -> "OemRuntimeStateStore":
        root = os.environ.get(OEM_RUNTIME_STATE_ROOT_ENV)
        if not root:
            raise OemRuntimeStateError(f"{OEM_RUNTIME_STATE_ROOT_ENV} is required")
        return cls(root, snapshot)

    def _ensure_directories(self) -> None:
        current = Path(self.version_root.anchor)
        for part in self.version_root.parts[1:]:
            current = current / part
            if current.exists():
                info = current.lstat()
                if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
                    raise OemRuntimeStateError(f"runtime-state path component is not a real directory: {current}")
            else:
                current.mkdir(mode=0o700)
                _fsync_directory(current.parent)
        for relative in sorted(_STATE_DIRECTORIES, key=lambda item: (item.count("/"), item)):
            directory = self.version_root.joinpath(*PurePosixPath(relative).parts)
            if directory.exists():
                info = directory.lstat()
                if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
                    raise OemRuntimeStateError(f"runtime-state directory contract failed: {directory}")
            else:
                directory.mkdir(mode=0o700)
                _fsync_directory(directory.parent)
        lock_flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        lock_fd = os.open(self._transaction_lock_path, lock_flags, 0o600)
        try:
            info = os.fstat(lock_fd)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise OemRuntimeStateError("transaction lock must be a unique regular file")
            os.fsync(lock_fd)
        finally:
            os.close(lock_fd)
        expected_entries = set(_STATE_DIRECTORIES) | {_LOCK_NAME} | {path for path in _STATE_PATHS if (self.version_root / path).exists()}
        if self._journal_path.exists():
            expected_entries.add(_JOURNAL_NAME)
        actual: set[str] = set()
        pending = [(self.version_root, "")]
        while pending:
            directory, prefix = pending.pop()
            with os.scandir(directory) as entries:
                for entry in entries:
                    relative = f"{prefix}/{entry.name}" if prefix else entry.name
                    info = entry.stat(follow_symlinks=False)
                    if stat.S_ISLNK(info.st_mode):
                        raise OemRuntimeStateError(f"symlinked runtime-state entry is forbidden: {relative}")
                    if stat.S_ISDIR(info.st_mode):
                        actual.add(relative)
                        pending.append((Path(entry.path), relative))
                    elif stat.S_ISREG(info.st_mode):
                        if info.st_nlink != 1:
                            raise OemRuntimeStateError(f"hardlinked runtime-state file is forbidden: {relative}")
                        actual.add(relative)
                    else:
                        raise OemRuntimeStateError(f"non-regular runtime-state entry is forbidden: {relative}")
        if actual != expected_entries:
            raise OemRuntimeStateError(f"runtime-state entry set mismatch; unexpected={sorted(actual - expected_entries)}")
        lowered: dict[str, str] = {}
        for relative in actual:
            key = relative.casefold()
            if key in lowered and lowered[key] != relative:
                raise OemRuntimeStateError(f"case-colliding runtime-state entries: {lowered[key]}, {relative}")
            lowered[key] = relative

    @contextmanager
    def _transaction_guard(self):
        flags = os.O_RDWR | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        fd = os.open(self._transaction_lock_path, flags)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise OemRuntimeStateError("transaction lock must be a unique regular file")
            fcntl.flock(fd, fcntl.LOCK_EX)
            yield
        finally:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            finally:
                os.close(fd)

    def _load_journal(self) -> None:
        if not self._journal_path.exists():
            return
        data = _read_no_follow(self._journal_path)
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise OemRuntimeStateError("transaction provenance journal is not UTF-8") from exc
        for line_number, line in enumerate(text.splitlines(), 1):
            try:
                row = json.loads(line)
                transaction = RuntimeStateTransaction(**row)
            except Exception as exc:
                raise OemRuntimeStateError(f"invalid transaction provenance at line {line_number}: {exc}") from exc
            if transaction.relative_path not in _STATE_PATHS:
                raise OemRuntimeStateError(f"journal references unowned path: {transaction.relative_path}")
            baseline = self.snapshot.mutable_seeds[transaction.relative_path].sha256
            if transaction.baseline_sha256 != baseline:
                raise OemRuntimeStateError(f"journal baseline authority mismatch: {transaction.relative_path}")
            if transaction.outcome == "committed":
                self._latest[transaction.relative_path] = transaction

    def _append_journal(self, transaction: RuntimeStateTransaction) -> None:
        payload = (json.dumps(transaction.to_payload(), sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        fd = os.open(self._journal_path, flags, 0o600)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise OemRuntimeStateError("transaction provenance journal is not a unique regular file")
            view = memoryview(payload)
            while view:
                written = os.write(fd, view)
                if written <= 0:
                    raise OemRuntimeStateError("short transaction provenance append")
                view = view[written:]
            os.fsync(fd)
        finally:
            os.close(fd)
        _fsync_directory(self.version_root)

    def _seed_or_verify_all(self) -> None:
        for relative in sorted(_STATE_PATHS):
            record = self.snapshot.mutable_seeds.get(relative)
            if record is None:
                raise OemRuntimeStateError(f"snapshot lacks required mutable seed: {relative}")
            path = self.version_root.joinpath(*PurePosixPath(relative).parts)
            if not path.exists():
                _atomic_replace(path, record.raw_bytes)
                transaction = RuntimeStateTransaction(
                    transaction_id=str(uuid.uuid4()),
                    timestamp_utc=_utc_now(),
                    relative_path=relative,
                    operation="seed_exact_immutable_baseline",
                    writer="OemRuntimeStateStore.__init__",
                    call_path="runtime_state.seed",
                    baseline_sha256=record.sha256,
                    before_sha256=None,
                    after_sha256=record.sha256,
                    outcome="committed",
                )
                self._append_journal(transaction)
                self._latest[relative] = transaction
                continue
            data = _read_no_follow(path)
            actual = _sha256(data)
            latest = self._latest.get(relative)
            allowed = record.sha256 if latest is None else latest.after_sha256
            if allowed is None or actual != allowed:
                raise OemRuntimeStateError(f"runtime-state hash lacks matching committed provenance: {relative}")
            _validate_record_bytes(relative, data)
            if latest is None:
                transaction = RuntimeStateTransaction(
                    transaction_id=str(uuid.uuid4()),
                    timestamp_utc=_utc_now(),
                    relative_path=relative,
                    operation="adopt_verified_exact_baseline",
                    writer="OemRuntimeStateStore.__init__",
                    call_path="runtime_state.seed_verification",
                    baseline_sha256=record.sha256,
                    before_sha256=actual,
                    after_sha256=actual,
                    outcome="committed",
                )
                self._append_journal(transaction)
                self._latest[relative] = transaction

    def _build_status(self) -> Mapping[str, Any]:
        return MappingProxyType({
            "ok": True,
            "schema_version": "bioxp.oem_runtime_state.v1",
            "serial": self.snapshot.machine_serial,
            "acquisition_id": self.snapshot.acquisition_id,
            "baseline_lock_sha256": self.snapshot.lock_sha256,
            "version_root": str(self.version_root),
            "records": MappingProxyType({
                relative: MappingProxyType({
                    "baseline_sha256": self.snapshot.mutable_seeds[relative].sha256,
                    "current_sha256": self._latest[relative].after_sha256,
                    "last_transaction_id": self._latest[relative].transaction_id,
                    "runtime_state_relative_path": relative,
                })
                for relative in sorted(_STATE_PATHS)
            }),
            "operation_parameters": MappingProxyType(dict(self._operation_parameters)),
            "evidence_mutated": False,
        })

    def status_projection(self) -> dict[str, Any]:
        def thaw(value: Any) -> Any:
            if isinstance(value, Mapping):
                return {key: thaw(item) for key, item in value.items()}
            return value
        return thaw(self._status)

    def _verified_read(self, relative_path: str) -> bytes:
        relative = _relative(relative_path)
        path = self.version_root.joinpath(*PurePosixPath(relative).parts)
        data = _read_no_follow(path)
        actual = _sha256(data)
        latest = self._latest.get(relative)
        if latest is None or latest.after_sha256 != actual:
            raise OemRuntimeStateError(f"runtime-state read has no matching committed provenance: {relative}")
        return data

    def read_operation_parameters(self) -> bytes:
        """OEM current-directory-first/AppData-second semantics after collision proof."""
        modeled_current = self.snapshot.bundle_root / "current_directory"
        collisions = [entry.name for entry in os.scandir(modeled_current) if entry.name.casefold() == "operation_parameters.xml".casefold()]
        process_collisions = [entry.name for entry in os.scandir(Path.cwd()) if entry.name.casefold() == "operation_parameters.xml".casefold()]
        if collisions or process_collisions:
            raise OemRuntimeStateError("unpinned current-directory Operation_parameters.xml authority collision")
        return self._verified_read("appdata/Operation_parameters.xml")

    def operation_parameters_projection(self) -> dict[str, Any]:
        return dict(self._operation_parameters)

    def read_record(self, relative_path: str) -> bytes:
        return self._verified_read(relative_path)

    def write_record(
        self,
        relative_path: str,
        data: bytes,
        *,
        writer: str,
        call_path: str,
        _transaction_guard_held: bool = False,
    ) -> RuntimeStateTransaction:
        relative = _relative(relative_path)
        if relative == "appdata/pressurebuffer.txt":
            raise OemRuntimeStateError("pressurebuffer mutation is disabled until the exact pressure-model acceptance contract is proven")
        if not isinstance(data, bytes):
            raise TypeError("transactional OEM state writes require exact bytes")
        if not writer.strip() or not call_path.strip():
            raise OemRuntimeStateError("writer and OEM call-path provenance are required")
        _validate_record_bytes(relative, data)
        guard = nullcontext() if _transaction_guard_held else self._transaction_guard()
        with self._lock, guard:
            before = self._verified_read(relative)
            before_hash = _sha256(before)
            after_hash = _sha256(data)
            record = self.snapshot.mutable_seeds[relative]
            transaction_id = str(uuid.uuid4())
            path = self.version_root.joinpath(*PurePosixPath(relative).parts)
            try:
                _atomic_replace(path, data)
                committed = _read_no_follow(path)
                if _sha256(committed) != after_hash:
                    raise OemRuntimeStateError(f"post-replace verification failed: {relative}")
                transaction = RuntimeStateTransaction(
                    transaction_id=transaction_id,
                    timestamp_utc=_utc_now(),
                    relative_path=relative,
                    operation="atomic_replace",
                    writer=writer,
                    call_path=call_path,
                    baseline_sha256=record.sha256,
                    before_sha256=before_hash,
                    after_sha256=after_hash,
                    outcome="committed",
                )
                self._append_journal(transaction)
                self._latest[relative] = transaction
                if relative == "appdata/Operation_parameters.xml":
                    operation, _ = parse_operation_parameters_bytes(
                        data,
                        record_relative_path=relative,
                        require_serial206_baseline=False,
                    )
                    self._operation_parameters = dict(operation)
                self._status = self._build_status()
                return transaction
            except Exception as exc:
                failure = RuntimeStateTransaction(
                    transaction_id=transaction_id,
                    timestamp_utc=_utc_now(),
                    relative_path=relative,
                    operation="atomic_replace",
                    writer=writer,
                    call_path=call_path,
                    baseline_sha256=record.sha256,
                    before_sha256=before_hash,
                    after_sha256=after_hash,
                    outcome="failed",
                    error=f"{type(exc).__name__}: {exc}",
                )
                try:
                    self._append_journal(failure)
                except Exception as journal_exc:
                    raise OemRuntimeStateError(f"state transaction and failure-provenance append both failed: {exc}; {journal_exc}") from exc
                raise

    def write_operation_parameters(self, data: bytes, *, writer: str, call_path: str) -> RuntimeStateTransaction:
        return self.write_record("appdata/Operation_parameters.xml", data, writer=writer, call_path=call_path)

    def write_process_times(self, data: bytes, *, writer: str, call_path: str) -> RuntimeStateTransaction:
        return self.write_record("appdata/processtime.xml", data, writer=writer, call_path=call_path)

    def write_pressure_buffer(self, data: bytes, *, writer: str, call_path: str) -> RuntimeStateTransaction:
        return self.write_record("appdata/pressurebuffer.txt", data, writer=writer, call_path=call_path)

    def append_config_history(self, data: bytes, *, writer: str, call_path: str) -> RuntimeStateTransaction:
        if not isinstance(data, bytes) or not data:
            raise OemRuntimeStateError("config-history append requires non-empty exact bytes")
        with self._lock, self._transaction_guard():
            before = self._verified_read("appdata_parent/Config_History/config_history.csv")
            return self.write_record(
                "appdata_parent/Config_History/config_history.csv",
                before + data,
                writer=writer,
                call_path=call_path,
                _transaction_guard_held=True,
            )


def set_active_oem_runtime_state_store(store: OemRuntimeStateStore) -> OemRuntimeStateStore:
    global _active_store
    with _active_store_lock:
        if _active_store is not None and _active_store is not store:
            raise OemRuntimeStateError("OEM runtime-state store is already bound and cannot be replaced in-process")
        _active_store = store
    return store


def get_active_oem_runtime_state_store() -> OemRuntimeStateStore:
    with _active_store_lock:
        store = _active_store
    if store is None:
        raise OemRuntimeStateError("OEM runtime-state store is not bound")
    return store


def configure_oem_runtime_state_from_env(snapshot: OemMachineSnapshot) -> OemRuntimeStateStore:
    return set_active_oem_runtime_state_store(OemRuntimeStateStore.from_environment(snapshot))
