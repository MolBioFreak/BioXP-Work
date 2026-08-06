from __future__ import annotations

import hashlib
import json
import os
import stat
import threading
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Any, Mapping


OEM_MACHINE_BUNDLE_LOCK_ENV = "BIOXP_OEM_MACHINE_BUNDLE_LOCK"
OEM_PHYSICAL_LABEL_SERIAL_ENV = "BIOXP_PHYSICAL_LABEL_SERIAL"
OEM_EVIDENCE_SCHEMA_ID = "bioxp.oem_evidence_lock.v4"
OEM_EVIDENCE_SCHEMA_VERSION = 4
OEM_ACQUISITION_ID = "20260719T024740Z-4a7fe6783205846c"
OEM_MACHINE_SERIAL = 206
# Current canonical bytes named by the movement registry. The historical
# acceptance package recorded an earlier revision (148b...); it must not be
# reported as the hash of the present authoritative lock file.
OEM_LOCK_SHA256 = "a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c"


class OemMachineBundleError(RuntimeError):
    """The explicit serial-206 evidence bundle failed a closed-world contract."""


@dataclass(frozen=True)
class _RecordContract:
    logical_relative_path: str
    bundle_relative_path: str
    size: int
    sha256: str
    role: str
    mutation_policy: str
    consumer_status: str
    runtime_state_relative_path: str | None = None


_READ_ONLY = "read_only_commissioned_input"
_COPY_ON_WRITE = "copy_on_write_runtime_state_seed"
_MACHINE_INPUT = "OEM machine/AppData configuration input"
_VISION_INPUT = "OEM case-sensitive vision/template input"

_LIVE_RECORD_CONTRACTS = (
    _RecordContract("Config_History/config_history.csv", "appdata_parent/Config_History/config_history.csv", 2477, "ba715995d2fc63a1bcce97d50b85d9d6928b015aaf8cd93a5ddfa47a317ad57b", "mutable_audit_history_seed", _COPY_ON_WRITE, "OEM append target; immutable captured baseline seeds separate transactional audit history", "appdata_parent/Config_History/config_history.csv"),
    _RecordContract("GenBotApp/calreference.xml", "appdata/calreference.xml", 840, "f941cb252028a1ee649ffb4185f9b0314f7b483e7f482e926ba10efe32a94e20", "machine_configuration", _READ_ONLY, _MACHINE_INPUT),
    _RecordContract("GenBotApp/config.xml", "appdata/config.xml", 4473, "33aadf87f631cf33f2e0b4c86948c92be3b21412ca5477ea8fa8bc7848cbf475", "machine_configuration", _READ_ONLY, _MACHINE_INPUT),
    _RecordContract("GenBotApp/cover.jpg", "appdata/cover.jpg", 71082, "d467be79fd298b5703f86ad79d0e6a28e4b8b6fcea159af810d29224ded4028d", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/EmptyStorage.jpg", "appdata/EmptyStorage.jpg", 21387, "1c911218f17dd542c5cf88153fc4bcc4402c6b48d274f8e015dc1a58d9e3cc69", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/EmptyTrough.jpg", "appdata/EmptyTrough.jpg", 15007, "8a896cdb82732a26464f6c8a9611c19b3ecdd9a38ed5daac8edeee3273e13b3d", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/InspectionSettings.xml", "appdata/InspectionSettings.xml", 20395, "d38220177e7e01b3d6d50892e0ffbbe27b1eb46087c4623cd6ca4757cc80b2d7", "machine_configuration", _READ_ONLY, _MACHINE_INPUT),
    _RecordContract("GenBotApp/LowerHandle.jpg", "appdata/LowerHandle.jpg", 45521, "ff1aefce8e7f586379a247e4b4a6dcdc573b27c54f4b56b9096c5e1a77b6a251", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/Operation_parameters.xml", "appdata/Operation_parameters.xml", 857, "d032d58c08312706892a2d5c7a9a319f817a6d9ef73e75d30dd933ae110c9685", "mutable_operation_policy_seed", _COPY_ON_WRITE, "OEM read/write operation policy; current-directory lookup precedes AppData, no current-directory copy was captured, and OEM writes AppData; immutable captured baseline seeds separate transactional AppData state", "appdata/Operation_parameters.xml"),
    _RecordContract("GenBotApp/output.jpg", "appdata/output.jpg", 20360, "a5b6853674bd6a72f905de2809bdb9b83943032f7c3de8e71492bb9948bf4c1c", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/output_empty.jpg", "appdata/output_empty.jpg", 19608, "fba247c64b05761abfafed74a9793e3c291bbc581dbd1f9f209f47f7ff983f49", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/outputw_foil.jpg", "appdata/outputw_foil.jpg", 18543, "0338b5622e19d2aef758d018cd110f6c5b46ed19d7be3b065aff175092eca2d4", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/pressurebuffer.txt", "appdata/pressurebuffer.txt", 39277, "96213f97bdf9b429642e0e1e8ed86500c22bd45f17dd96a546b3b37362295c42", "mutable_runtime_state_seed", _COPY_ON_WRITE, "OEM AppData runtime read/write state; captured bytes seed a separate mutable state root", "appdata/pressurebuffer.txt"),
    _RecordContract("GenBotApp/processtime.xml", "appdata/processtime.xml", 1414, "4ed472e39626cbdef3af013532a9233326c4c913bf7b11e222e9104f0511d186", "mutable_runtime_state_seed", _COPY_ON_WRITE, "OEM AppData runtime read/write state; captured bytes seed a separate mutable state root", "appdata/processtime.xml"),
    _RecordContract("GenBotApp/purificationTray.jpg", "appdata/purificationTray.jpg", 31412, "a57ab7e6376397a902fc6441ae9e5ef2d761b22b04d6d02e6c949b1448832796", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/reagentEmpty.jpg", "appdata/reagentEmpty.jpg", 39460, "2b36b053655f5d3c3ad042ffd991560f8384bf53ad0357433eae13bbd5645aac", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/reagentTray.jpg", "appdata/reagentTray.jpg", 42479, "796f848393da3282df9891db6525d75f51fa8fcdbea4cb2520632167ab9db0cb", "vision_template", _READ_ONLY, _VISION_INPUT),
    _RecordContract("GenBotApp/Trough.jpg", "appdata/Trough.jpg", 8508, "a80bb90bab79062022336bf55d6f115dfbf550e6a8d38aeed650d478e6570498", "vision_template_candidate", _READ_ONLY, "captured template candidate; direct reader unresolved and must not be invented"),
    _RecordContract("GenBotApp/UpperHandle.jpg", "appdata/UpperHandle.jpg", 48035, "43aed9f668a4723f915f055bacc4b63e30e4d1cc9e277908b3e8fcd17abb073b", "vision_template", _READ_ONLY, _VISION_INPUT),
)

_CURRENT_CALIBRATION_CONTRACT = _RecordContract(
    "ClickOnce/current-directory/calreference.xml",
    "current_directory/calreference.xml",
    843,
    "3115c3428b52721d4a0f1c3bdd3b8ea7a437179a4166eb9ae0e682e760632762",
    "current_directory_latest_calibration_reference",
    _READ_ONLY,
    "OEM current-directory latest calibration comparison source",
)
_DEPLOYMENT_PROCESS_TIME_CONTRACT = _RecordContract(
    "ClickOnce/ProcessTime.xml",
    "deployment_evidence/ProcessTime.xml",
    1012,
    "8024408747098b8e483d92a6c344b80bc06544342b7a82c4c090031b170fccbb",
    "deployment_template_evidence",
    _READ_ONLY,
    "retained comparison evidence; forbidden as AppData runtime source",
)

_EXPECTED_DIRECTORIES = frozenset({
    "appdata",
    "appdata_parent",
    "appdata_parent/Config_History",
    "current_directory",
    "deployment_evidence",
})
_EXPECTED_FILES = frozenset(
    {contract.bundle_relative_path for contract in _LIVE_RECORD_CONTRACTS}
    | {_CURRENT_CALIBRATION_CONTRACT.bundle_relative_path, _DEPLOYMENT_PROCESS_TIME_CONTRACT.bundle_relative_path, "OEM_EVIDENCE_LOCK.json"}
)
_REQUIRED_VISION_TEMPLATES = frozenset(
    contract.bundle_relative_path
    for contract in _LIVE_RECORD_CONTRACTS
    if contract.role == "vision_template"
)


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, set):
        return frozenset(_freeze(item) for item in value)
    return value


def _thaw(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, (tuple, frozenset)):
        return [_thaw(item) for item in value]
    return value


@dataclass(frozen=True)
class OemRecordProvenance:
    logical_relative_path: str
    bundle_relative_path: str
    original_source_path: str
    role: str
    mutation_policy: str
    consumer_status: str
    size: int
    sha256: str
    creation_utc: str | None
    last_write_utc: str | None
    acquisition_id: str
    runtime_state_relative_path: str | None
    raw_bytes: bytes

    def public_projection(self) -> dict[str, Any]:
        return {
            "logical_relative_path": self.logical_relative_path,
            "bundle_relative_path": self.bundle_relative_path,
            "original_source_path": self.original_source_path,
            "role": self.role,
            "mutation_policy": self.mutation_policy,
            "consumer_status": self.consumer_status,
            "size": self.size,
            "sha256": self.sha256,
            "creation_utc": self.creation_utc,
            "last_write_utc": self.last_write_utc,
            "acquisition_id": self.acquisition_id,
            "runtime_state_relative_path": self.runtime_state_relative_path,
            "verification": "exact_path_case_type_size_sha256_verified",
        }


@dataclass(frozen=True)
class OemFieldProvenance:
    field: str
    value: Any
    raw_value: str
    raw_attribute: str
    record_relative_path: str
    xml_path: str
    units: str | None = None
    interpretation: str | None = None

    def public_projection(self) -> dict[str, Any]:
        return {
            "field": self.field,
            "value": _thaw(self.value),
            "raw_value": self.raw_value,
            "raw_attribute": self.raw_attribute,
            "record_relative_path": self.record_relative_path,
            "xml_path": self.xml_path,
            "units": self.units,
            "interpretation": self.interpretation,
        }


@dataclass(frozen=True)
class OemMachineSnapshot:
    acquisition_id: str
    machine_serial: int
    lock_sha256: str
    bundle_root: Path
    records: Mapping[str, OemRecordProvenance]
    fields: Mapping[str, OemFieldProvenance]
    axis_limits: Mapping[str, Mapping[str, Any]]
    position_table: tuple[Mapping[str, Any], ...]
    config_sections: Mapping[str, Mapping[str, Any]]
    operation_parameters: Mapping[str, Any]
    inspection_profile_name: str
    inspection_profile: Mapping[str, Any]
    calibration_comparison: Mapping[str, Any]
    process_times: Mapping[str, float]
    mutable_seeds: Mapping[str, OemRecordProvenance]
    operator_label_matched: bool
    validation_conflicts: tuple[str, ...] = ()

    @property
    def machine_calibrated(self) -> bool:
        return bool(self.fields["machine.calibrated"].value)

    @property
    def camera_calibrated(self) -> bool:
        return bool(self.fields["machine.camera_calibrated"].value)

    @property
    def startup_mode(self) -> str:
        return str(self.operation_parameters["Mode"])

    @property
    def vision_ready(self) -> bool:
        required = bool(self.operation_parameters["DeckInspection"] or self.operation_parameters["CheckCamera"])
        return (not required) or (
            self.inspection_profile_name == "Settings3200"
            and _REQUIRED_VISION_TEMPLATES.issubset(self.records)
        )

    @property
    def mutation_authorized(self) -> bool:
        return self.operator_label_matched and not self.validation_conflicts

    def config_status_projection(self) -> dict[str, Any]:
        return {
            "ok": not self.validation_conflicts,
            "schema_version": "bioxp.oem_machine_snapshot.v1",
            "serial": self.machine_serial,
            "acquisition_id": self.acquisition_id,
            "lock_sha256": self.lock_sha256,
            "record_count": len(self.records),
            "live_machine_record_count": len(_LIVE_RECORD_CONTRACTS),
            "records": [self.records[path].public_projection() for path in sorted(self.records)],
            "field_provenance": [self.fields[name].public_projection() for name in sorted(self.fields)],
            "startup_mode": self.startup_mode,
            "machine_calibrated": self.machine_calibrated,
            "camera_calibrated": self.camera_calibrated,
            "inspection_profile": self.inspection_profile_name,
            "vision_ready": self.vision_ready,
            "calibration_comparison": _thaw(self.calibration_comparison),
            "mutable_seeds": {
                path: record.public_projection() for path, record in sorted(self.mutable_seeds.items())
            },
            "operator_label_matched": self.operator_label_matched,
            "mutation_authorized": self.mutation_authorized,
            "validation_conflicts": list(self.validation_conflicts),
            "secrets_exposed": False,
            "opened_usb": False,
            "physical_motion": False,
            "motion_commanded": False,
            "current_mutation_commanded": False,
            "switch_mask_mutation_commanded": False,
        }


_snapshot_lock = threading.Lock()
_active_snapshot: OemMachineSnapshot | None = None


def set_active_oem_machine_snapshot(snapshot: OemMachineSnapshot) -> OemMachineSnapshot:
    global _active_snapshot
    with _snapshot_lock:
        if _active_snapshot is not None and _active_snapshot != snapshot:
            raise OemMachineBundleError("OEM machine snapshot is already bound and cannot be replaced in-process")
        _active_snapshot = snapshot
    return snapshot


def get_active_oem_machine_snapshot() -> OemMachineSnapshot:
    with _snapshot_lock:
        snapshot = _active_snapshot
    if snapshot is None:
        raise OemMachineBundleError("OEM machine snapshot is not bound")
    return snapshot


def configure_oem_machine_snapshot_from_env(*, require_operator_label: bool = True) -> OemMachineSnapshot:
    lock_path = os.environ.get(OEM_MACHINE_BUNDLE_LOCK_ENV)
    if not lock_path:
        raise OemMachineBundleError(f"{OEM_MACHINE_BUNDLE_LOCK_ENV} is required")
    label = os.environ.get(OEM_PHYSICAL_LABEL_SERIAL_ENV)
    snapshot = load_oem_machine_snapshot(
        lock_path,
        operator_label_serial=label,
        require_operator_label=require_operator_label,
    )
    return set_active_oem_machine_snapshot(snapshot)


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _coerce(raw: str) -> Any:
    text = raw.strip()
    if text.lower() in {"true", "false"}:
        return text.lower() == "true"
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return text


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _assert_absolute_non_symlink_path(path: Path) -> None:
    if not path.is_absolute():
        raise OemMachineBundleError(f"explicit lock path must be absolute: {path}")
    if ".." in path.parts:
        raise OemMachineBundleError(f"indirected explicit lock path is forbidden: {path}")
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current = current / part
        try:
            mode = current.lstat().st_mode
        except FileNotFoundError as exc:
            raise OemMachineBundleError(f"required path component is missing: {current}") from exc
        if stat.S_ISLNK(mode):
            raise OemMachineBundleError(f"symlinked path component is forbidden: {current}")


def _safe_relative_path(raw: str) -> str:
    path = PurePosixPath(raw)
    if path.is_absolute() or not path.parts or any(part in {"", ".", ".."} for part in path.parts):
        raise OemMachineBundleError(f"indirected or escaping bundle path is forbidden: {raw!r}")
    return path.as_posix()


def _inventory_bundle(root: Path) -> tuple[set[str], set[str]]:
    files: set[str] = set()
    directories: set[str] = set()
    pending = [(root, "")]
    while pending:
        directory, prefix = pending.pop()
        with os.scandir(directory) as entries:
            for entry in entries:
                relative = f"{prefix}/{entry.name}" if prefix else entry.name
                info = entry.stat(follow_symlinks=False)
                if stat.S_ISLNK(info.st_mode):
                    raise OemMachineBundleError(f"symlinked bundle entry is forbidden: {relative}")
                if stat.S_ISDIR(info.st_mode):
                    directories.add(relative)
                    pending.append((Path(entry.path), relative))
                elif stat.S_ISREG(info.st_mode):
                    if info.st_nlink != 1:
                        raise OemMachineBundleError(f"hardlinked bundle file is forbidden: {relative}")
                    files.add(relative)
                else:
                    raise OemMachineBundleError(f"non-regular bundle entry is forbidden: {relative}")
    lowered: dict[str, str] = {}
    for relative in sorted(files | directories):
        key = relative.casefold()
        previous = lowered.get(key)
        if previous is not None and previous != relative:
            raise OemMachineBundleError(f"case-colliding bundle entries: {previous!r}, {relative!r}")
        lowered[key] = relative
    return files, directories


def _read_exact_file(root: Path, relative: str, *, size: int, sha256: str) -> bytes:
    relative = _safe_relative_path(relative)
    path = root.joinpath(*PurePosixPath(relative).parts)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise OemMachineBundleError(f"cannot open exact bundle file {relative}: {exc}") from exc
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise OemMachineBundleError(f"bundle file is not a unique regular file: {relative}")
        chunks: list[bytes] = []
        while True:
            chunk = os.read(fd, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        data = b"".join(chunks)
    finally:
        os.close(fd)
    actual_hash = _sha256(data)
    if len(data) != size or actual_hash != sha256:
        raise OemMachineBundleError(
            f"exact byte contract failed for {relative}: size={len(data)}/{size}, sha256={actual_hash}/{sha256}"
        )
    return data


def _require_lock_shape(lock: Mapping[str, Any]) -> None:
    if lock.get("schema_id") != OEM_EVIDENCE_SCHEMA_ID or lock.get("schema_version") != OEM_EVIDENCE_SCHEMA_VERSION:
        raise OemMachineBundleError("evidence schema identity/version mismatch")
    acquisition = lock.get("acquisition")
    if not isinstance(acquisition, Mapping) or acquisition.get("session_id") != OEM_ACQUISITION_ID:
        raise OemMachineBundleError("acquisition identity mismatch")
    live = lock.get("live_machine_corpus")
    if not isinstance(live, Mapping):
        raise OemMachineBundleError("live machine corpus is missing")
    required_live = {
        "contract_schema_id": "bioxp.live_machine_corpus.serial206.v1",
        "root_label": "localappdata_synthetic_genomics",
        "expected_root_label": "localappdata_synthetic_genomics",
        "record_kind": "file_default_stream",
        "expected_record_kind": "file_default_stream",
        "expected_record_count": len(_LIVE_RECORD_CONTRACTS),
    }
    for key, expected in required_live.items():
        if live.get(key) != expected:
            raise OemMachineBundleError(f"live machine corpus authority mismatch: {key}")


def _lock_live_records(lock: Mapping[str, Any]) -> Mapping[str, Mapping[str, Any]]:
    live = lock["live_machine_corpus"]
    rows = live.get("records") if isinstance(live, Mapping) else None
    if not isinstance(rows, list) or len(rows) != len(_LIVE_RECORD_CONTRACTS):
        raise OemMachineBundleError("live machine record count mismatch")
    indexed: dict[str, Mapping[str, Any]] = {}
    for row in rows:
        if not isinstance(row, Mapping) or not isinstance(row.get("runtime_projection_relative_path"), str):
            raise OemMachineBundleError("malformed live machine record")
        path = _safe_relative_path(str(row["runtime_projection_relative_path"]))
        if path in indexed:
            raise OemMachineBundleError(f"duplicate live runtime projection: {path}")
        indexed[path] = row
    expected = {contract.bundle_relative_path for contract in _LIVE_RECORD_CONTRACTS}
    if set(indexed) != expected:
        raise OemMachineBundleError("live machine projection set mismatch")
    return MappingProxyType(indexed)


def _validate_live_metadata(row: Mapping[str, Any], contract: _RecordContract) -> None:
    expected = {
        "logical_relative_path": contract.logical_relative_path,
        "runtime_projection_relative_path": contract.bundle_relative_path,
        "size": contract.size,
        "sha256": contract.sha256,
        "receiver_sha256": contract.sha256,
        "role": contract.role,
        "mutation_policy": contract.mutation_policy,
        "consumer_status": contract.consumer_status,
        "stable_capture": True,
        "upload_status": 201,
    }
    if contract.runtime_state_relative_path is not None:
        expected["runtime_state_relative_path"] = contract.runtime_state_relative_path
    elif "runtime_state_relative_path" in row:
        raise OemMachineBundleError(f"unexpected runtime-state authority for {contract.bundle_relative_path}")
    for key, value in expected.items():
        if row.get(key) != value:
            raise OemMachineBundleError(f"authority-role contract mismatch for {contract.bundle_relative_path}: {key}")
    source_path = row.get("source_path")
    relative_path = row.get("relative_path")
    if not isinstance(source_path, str) or not isinstance(relative_path, str) or not relative_path.startswith("source_roots/localappdata_synthetic_genomics/"):
        raise OemMachineBundleError(f"source-root contract mismatch for {contract.bundle_relative_path}")


def _record_from_live(row: Mapping[str, Any], contract: _RecordContract, raw_bytes: bytes) -> OemRecordProvenance:
    return OemRecordProvenance(
        logical_relative_path=contract.logical_relative_path,
        bundle_relative_path=contract.bundle_relative_path,
        original_source_path=str(row["source_path"]),
        role=contract.role,
        mutation_policy=contract.mutation_policy,
        consumer_status=contract.consumer_status,
        size=contract.size,
        sha256=contract.sha256,
        creation_utc=str(row.get("creation_utc")) if row.get("creation_utc") is not None else None,
        last_write_utc=str(row.get("last_write_utc")) if row.get("last_write_utc") is not None else None,
        acquisition_id=OEM_ACQUISITION_ID,
        runtime_state_relative_path=contract.runtime_state_relative_path,
        raw_bytes=raw_bytes,
    )


def _cross_root_records(lock: Mapping[str, Any], root: Path) -> tuple[OemRecordProvenance, OemRecordProvenance]:
    machine_bundle = lock.get("machine_bundle")
    if not isinstance(machine_bundle, list):
        raise OemMachineBundleError("machine_bundle authority records are missing")
    candidates = [row for row in machine_bundle if isinstance(row, Mapping) and row.get("bundle_relative_path") == _CURRENT_CALIBRATION_CONTRACT.bundle_relative_path]
    if len(candidates) != 1:
        raise OemMachineBundleError("current-directory calibration authority is missing or duplicated")
    cal = candidates[0]
    expected_cal = {
        "size": _CURRENT_CALIBRATION_CONTRACT.size,
        "sha256": _CURRENT_CALIBRATION_CONTRACT.sha256,
        "receiver_sha256": _CURRENT_CALIBRATION_CONTRACT.sha256,
        "stable_capture": True,
        "upload_status": 201,
        "oem_role": "current-directory/ClickOnce latest calibration comparison source",
    }
    for key, value in expected_cal.items():
        if cal.get(key) != value:
            raise OemMachineBundleError(f"current-directory calibration authority mismatch: {key}")
    cal_bytes = _read_exact_file(root, _CURRENT_CALIBRATION_CONTRACT.bundle_relative_path, size=_CURRENT_CALIBRATION_CONTRACT.size, sha256=_CURRENT_CALIBRATION_CONTRACT.sha256)
    cal_record = OemRecordProvenance(
        logical_relative_path=_CURRENT_CALIBRATION_CONTRACT.logical_relative_path,
        bundle_relative_path=_CURRENT_CALIBRATION_CONTRACT.bundle_relative_path,
        original_source_path=str(cal.get("source_path")),
        role=_CURRENT_CALIBRATION_CONTRACT.role,
        mutation_policy=_READ_ONLY,
        consumer_status=_CURRENT_CALIBRATION_CONTRACT.consumer_status,
        size=_CURRENT_CALIBRATION_CONTRACT.size,
        sha256=_CURRENT_CALIBRATION_CONTRACT.sha256,
        creation_utc=None,
        last_write_utc=str(cal.get("last_write_utc")) if cal.get("last_write_utc") is not None else None,
        acquisition_id=OEM_ACQUISITION_ID,
        runtime_state_relative_path=None,
        raw_bytes=cal_bytes,
    )

    additional = lock.get("additional_config_evidence")
    if not isinstance(additional, list) or len(additional) != 1 or not isinstance(additional[0], Mapping):
        raise OemMachineBundleError("deployment process-time authority is missing or duplicated")
    template = additional[0]
    expected_template = {
        "logical_name": "ClickOnce ProcessTime.xml",
        "selected_for_immediate_runtime_bundle": False,
        "evidence_projection_relative_path": _DEPLOYMENT_PROCESS_TIME_CONTRACT.bundle_relative_path,
        "size": _DEPLOYMENT_PROCESS_TIME_CONTRACT.size,
        "sha256": _DEPLOYMENT_PROCESS_TIME_CONTRACT.sha256,
    }
    for key, value in expected_template.items():
        if template.get(key) != value:
            raise OemMachineBundleError(f"deployment process-time authority mismatch: {key}")
    source_records = template.get("source_records")
    if not isinstance(source_records, list) or len(source_records) != 2:
        raise OemMachineBundleError("deployment process-time source provenance mismatch")
    for source in source_records:
        if not isinstance(source, Mapping) or source.get("receiver_sha256") != _DEPLOYMENT_PROCESS_TIME_CONTRACT.sha256 or source.get("stable_capture") is not True:
            raise OemMachineBundleError("deployment process-time source hash/stability mismatch")
    template_bytes = _read_exact_file(root, _DEPLOYMENT_PROCESS_TIME_CONTRACT.bundle_relative_path, size=_DEPLOYMENT_PROCESS_TIME_CONTRACT.size, sha256=_DEPLOYMENT_PROCESS_TIME_CONTRACT.sha256)
    template_record = OemRecordProvenance(
        logical_relative_path=_DEPLOYMENT_PROCESS_TIME_CONTRACT.logical_relative_path,
        bundle_relative_path=_DEPLOYMENT_PROCESS_TIME_CONTRACT.bundle_relative_path,
        original_source_path=str(source_records[0].get("source_path")),
        role=_DEPLOYMENT_PROCESS_TIME_CONTRACT.role,
        mutation_policy=_READ_ONLY,
        consumer_status=_DEPLOYMENT_PROCESS_TIME_CONTRACT.consumer_status,
        size=_DEPLOYMENT_PROCESS_TIME_CONTRACT.size,
        sha256=_DEPLOYMENT_PROCESS_TIME_CONTRACT.sha256,
        creation_utc=None,
        last_write_utc=None,
        acquisition_id=OEM_ACQUISITION_ID,
        runtime_state_relative_path=None,
        raw_bytes=template_bytes,
    )
    return cal_record, template_record


def _field(name: str, raw: str, attr: str, record: str, xml_path: str, *, units: str | None = None, interpretation: str | None = None) -> OemFieldProvenance:
    return OemFieldProvenance(name, _freeze(_coerce(raw)), raw, attr, record, xml_path, units, interpretation)


def _parse_config(data: bytes) -> tuple[dict[str, OemFieldProvenance], dict[str, dict[str, Any]], dict[str, dict[str, Any]], tuple[Mapping[str, Any], ...]]:
    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise OemMachineBundleError(f"appdata/config.xml is malformed: {exc}") from exc
    if _local_name(root.tag) != "BioXPCommonLib":
        raise OemMachineBundleError("appdata/config.xml root mismatch")
    paths = {
        "serial": ("./GenBot/SerialNumber", "GenBot"),
        "gripper_version": ("./GenBot/Config", "GripperVersion"),
        "calibrated": ("./GenBot/Calibration", "Calibrated"),
        "calibration_tool": ("./GenBot/Calibration", "m_cal_tool"),
        "calibration_revision": ("./GenBot/Calibration", "m_cal_reversion"),
        "liquid_revision": ("./GenBot/Calibration", "m_liquid_cal_reversion"),
        "camera_installed": ("./GenBot/CameraInstalled", "Camera"),
        "camera_calibrated": ("./GenBot/CameraInstalled", "Cameracalibrated"),
    }
    fields: dict[str, OemFieldProvenance] = {}
    for short, (xml_path, attribute) in paths.items():
        element = root.find(xml_path)
        if element is None or attribute not in element.attrib:
            raise OemMachineBundleError(f"mandatory config field missing: {xml_path}/@{attribute}")
        fields[f"machine.{short}"] = _field(f"machine.{short}", element.attrib[attribute], attribute, "appdata/config.xml", f"{xml_path}/@{attribute}")
    axis_limits: dict[str, dict[str, Any]] = {}
    for axis in ("x", "y", "z", "g"):
        tag = f"{axis.upper()}_limit"
        element = root.find(f"./AxisLimits/{tag}")
        if element is None or set(("minSteps", "maxSteps")) - set(element.attrib):
            raise OemMachineBundleError(f"mandatory exact axis limit missing: {tag}")
        minimum = int(element.attrib["minSteps"])
        maximum = int(element.attrib["maxSteps"])
        if minimum > maximum:
            raise OemMachineBundleError(f"contradictory axis limits: {tag}")
        axis_limits[axis] = {
            "min_steps": minimum,
            "max_steps": maximum,
            "source": "serial_206_oem_machine_snapshot",
            "record_relative_path": "appdata/config.xml",
            "xml_path": f"./AxisLimits/{tag}",
            "units": "controller_steps",
        }
        fields[f"axis.{axis}.min_steps"] = _field(f"axis.{axis}.min_steps", element.attrib["minSteps"], "minSteps", "appdata/config.xml", f"./AxisLimits/{tag}/@minSteps", units="controller_steps")
        fields[f"axis.{axis}.max_steps"] = _field(f"axis.{axis}.max_steps", element.attrib["maxSteps"], "maxSteps", "appdata/config.xml", f"./AxisLimits/{tag}/@maxSteps", units="controller_steps")
    sections: dict[str, dict[str, Any]] = {}
    for name, xml_path in {
        "config": "./GenBot/Config",
        "calibration": "./GenBot/Calibration",
        "camera": "./GenBot/CameraInstalled",
        "offsets": "./CalibrationFactors/Offsets",
        "seal_cut": "./CalibrationFactors/SealCut",
        "reagent_chiller": "./CalibrationFactors/ReagentChiller",
        "output_chiller": "./CalibrationFactors/OutputChiller",
        "scale_port": "./ScalePort/Port",
    }.items():
        element = root.find(xml_path)
        if element is None:
            raise OemMachineBundleError(f"mandatory config section missing: {xml_path}")
        sections[name] = {key: _coerce(value) for key, value in element.attrib.items()}
        for key, value in element.attrib.items():
            field_name = f"config.{name}.{key}"
            fields[field_name] = _field(field_name, value, key, "appdata/config.xml", f"{xml_path}/@{key}")
    table = root.find("./PositionTable")
    if table is None:
        raise OemMachineBundleError("mandatory PositionTable is missing")
    rows: list[Mapping[str, Any]] = []
    seen: set[str] = set()
    for element in list(table):
        name = _local_name(element.tag)
        if name in seen:
            raise OemMachineBundleError(f"duplicate PositionTable location: {name}")
        seen.add(name)
        required = {"x", "y", "zLow", "zDelta", "inc_factor"}
        if required - set(element.attrib):
            raise OemMachineBundleError(f"incomplete PositionTable location: {name}")
        row = {
            "name": name,
            **{key: _coerce(value) for key, value in element.attrib.items()},
            "raw_attributes": dict(element.attrib),
        }
        row["zHigh"] = int(row["zLow"]) - int(row["zDelta"])
        row["source"] = "serial_206_oem_machine_snapshot:appdata/config.xml"
        rows.append(_freeze(row))
    if len(rows) != 29:
        raise OemMachineBundleError(f"serial-206 PositionTable must contain exactly 29 rows, got {len(rows)}")
    return fields, axis_limits, sections, tuple(rows)


def parse_operation_parameters_bytes(
    data: bytes,
    *,
    record_relative_path: str = "appdata/Operation_parameters.xml",
    require_serial206_baseline: bool = True,
) -> tuple[Mapping[str, Any], Mapping[str, OemFieldProvenance]]:
    """Parse both OEM element/attribute serialization shapes without rewriting bytes."""
    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise OemMachineBundleError(f"{record_relative_path} is malformed: {exc}") from exc
    operation = next((element for element in root.iter() if _local_name(element.tag) == "OperationParameters"), None)
    if operation is None:
        raise OemMachineBundleError(f"{record_relative_path} lacks OperationParameters")
    values: dict[str, Any] = {}
    provenance: dict[str, OemFieldProvenance] = {}
    for element in list(operation):
        name = _local_name(element.tag)
        if name in element.attrib:
            attribute = name
            raw = element.attrib[name]
        elif len(element.attrib) == 1:
            attribute, raw = next(iter(element.attrib.items()))
        elif (element.text or "").strip():
            attribute = "#text"
            raw = (element.text or "").strip()
        else:
            raise OemMachineBundleError(f"ambiguous operation parameter serialization: {name}")
        if name in values:
            raise OemMachineBundleError(f"duplicate operation parameter: {name}")
        values[name] = _coerce(raw)
        provenance[f"operation.{name}"] = _field(f"operation.{name}", raw, attribute, record_relative_path, f"./OperationParameters/{name}/@{attribute}")
    required = {"Mode", "DeckInspection", "CheckCamera"}
    if required - set(values):
        raise OemMachineBundleError(f"mandatory operation policy fields missing: {sorted(required - set(values))}")
    if require_serial206_baseline and values["Mode"] != "WebMode":
        raise OemMachineBundleError(f"serial-206 operation mode mismatch: {values['Mode']!r}")
    for key in ("DeckInspection", "CheckCamera"):
        if not isinstance(values[key], bool):
            raise OemMachineBundleError(f"operation policy {key} is not Boolean")
    return _freeze(values), MappingProxyType(provenance)


def _parse_inspection(data: bytes) -> Mapping[str, Any]:
    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise OemMachineBundleError(f"appdata/InspectionSettings.xml is malformed: {exc}") from exc
    settings = { _local_name(element.tag): element for element in list(root) }
    profile = settings.get("Settings3200")
    if profile is None or not list(profile):
        raise OemMachineBundleError("InspectionSettings.xml lacks populated Settings3200")
    keys: list[str] = []
    for item in list(profile):
        key = next((child for child in list(item) if _local_name(child.tag) == "Key"), None)
        if key is not None and (key.text or "").strip():
            keys.append((key.text or "").strip())
    if not keys or len(keys) != len(set(keys)):
        raise OemMachineBundleError("Settings3200 inspection keys are missing or duplicated")
    return _freeze({"entry_count": len(keys), "inspection_items": tuple(keys), "selected_for_serial": OEM_MACHINE_SERIAL})


def _parse_calibration(data: bytes, *, spelling: str, record: str) -> Mapping[str, Any]:
    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise OemMachineBundleError(f"{record} is malformed: {exc}") from exc
    tools: dict[str, Any] = {}
    fluid: dict[str, Any] | None = None
    for element in list(root):
        name = _local_name(element.tag)
        if name == "FluidReference":
            ref = next((child for child in list(element) if _local_name(child.tag) == "ref_location"), None)
            if ref is None or spelling not in ref.attrib:
                raise OemMachineBundleError(f"{record} fluid reference lacks exact {spelling} spelling")
            fluid = {"revision": int(ref.attrib[spelling]), "raw_attribute": spelling, "parameters": {key: _coerce(value) for key, value in ref.attrib.items() if key != spelling}}
            continue
        reference = next((child for child in list(element) if _local_name(child.tag) == "Reference"), None)
        parameters = next((child for child in list(element) if _local_name(child.tag) == "Parameters"), None)
        if reference is None or parameters is None or spelling not in reference.attrib:
            raise OemMachineBundleError(f"{record} tool reference {name} lacks exact {spelling} spelling")
        tools[name.removeprefix("PartNumber_")] = {
            "revision": int(reference.attrib[spelling]),
            "raw_attribute": spelling,
            "parameters": {key: _coerce(value) for key, value in parameters.attrib.items()},
        }
    if fluid is None or not tools:
        raise OemMachineBundleError(f"{record} calibration collection is incomplete")
    return _freeze({"tools": tools, "fluid": fluid, "record_relative_path": record})


def _calibration_comparison(installed: Mapping[str, Any], latest: Mapping[str, Any], tool_name: str) -> Mapping[str, Any]:
    installed_tool = installed["tools"].get(tool_name)
    latest_tool = latest["tools"].get(tool_name)
    if not isinstance(installed_tool, Mapping) or not isinstance(latest_tool, Mapping):
        raise OemMachineBundleError(f"calibration tool {tool_name!r} is absent from installed/latest collection")
    tool_installed = int(installed_tool["revision"])
    tool_latest = int(latest_tool["revision"])
    fluid_installed = int(installed["fluid"]["revision"])
    fluid_latest = int(latest["fluid"]["revision"])
    return _freeze({
        "tool_name": tool_name,
        "installed_record": "appdata/calreference.xml",
        "latest_record": "current_directory/calreference.xml",
        "installed_tool_revision": tool_installed,
        "latest_tool_revision": tool_latest,
        "installed_fluid_revision": fluid_installed,
        "latest_fluid_revision": fluid_latest,
        "tool_update_pending": tool_latest > tool_installed,
        "fluid_update_pending": fluid_latest > fluid_installed,
        "update_pending": tool_latest > tool_installed or fluid_latest > fluid_installed,
        "installed_raw_spelling": installed_tool["raw_attribute"],
        "latest_raw_spelling": latest_tool["raw_attribute"],
    })


def _parse_process_times(data: bytes) -> tuple[Mapping[str, float], Mapping[str, OemFieldProvenance]]:
    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise OemMachineBundleError(f"appdata/processtime.xml is malformed: {exc}") from exc
    process = next((element for element in root.iter() if _local_name(element.tag) == "processTime"), None)
    if process is None or not list(process):
        raise OemMachineBundleError("appdata/processtime.xml lacks processTime entries")
    values: dict[str, float] = {}
    fields: dict[str, OemFieldProvenance] = {}
    for element in list(process):
        name = _local_name(element.tag)
        if name in values or "process" not in element.attrib:
            raise OemMachineBundleError(f"invalid process-time entry: {name}")
        values[name] = float(element.attrib["process"])
        field_name = f"process_time.{name}"
        fields[field_name] = _field(
            field_name,
            element.attrib["process"],
            "process",
            "appdata/processtime.xml",
            f"./processTime/{name}/@process",
            units="seconds",
        )
    return _freeze(values), MappingProxyType(fields)


def _reject_operation_policy_collisions(root: Path, process_current_directory: Path | None) -> None:
    modeled = root / "current_directory"
    collisions = [entry.name for entry in os.scandir(modeled) if entry.name.casefold() == "operation_parameters.xml".casefold()]
    if collisions:
        raise OemMachineBundleError(f"unpinned modeled current-directory operation policy collision: {collisions}")
    cwd = process_current_directory if process_current_directory is not None else Path.cwd()
    try:
        entries = list(os.scandir(cwd))
    except OSError as exc:
        raise OemMachineBundleError(f"cannot verify process current directory for authority collision: {exc}") from exc
    collisions = [entry.name for entry in entries if entry.name.casefold() == "operation_parameters.xml".casefold()]
    if collisions:
        raise OemMachineBundleError(f"unpinned process-current-directory operation policy collision: {cwd}/{collisions[0]}")


def load_oem_machine_snapshot(
    lock_path: str | Path,
    *,
    operator_label_serial: int | str | None = None,
    require_operator_label: bool = False,
    process_current_directory: str | Path | None = None,
) -> OemMachineSnapshot:
    """Load the one accepted serial-206 bundle without search or repair."""
    explicit_lock = Path(lock_path)
    _assert_absolute_non_symlink_path(explicit_lock)
    if explicit_lock.name != "OEM_EVIDENCE_LOCK.json":
        raise OemMachineBundleError("explicit lock must retain exact filename OEM_EVIDENCE_LOCK.json")
    root = explicit_lock.parent
    files, directories = _inventory_bundle(root)
    if files != set(_EXPECTED_FILES):
        raise OemMachineBundleError(f"bundle file set mismatch; missing={sorted(_EXPECTED_FILES - files)}, extra={sorted(files - _EXPECTED_FILES)}")
    if directories != set(_EXPECTED_DIRECTORIES):
        raise OemMachineBundleError(f"bundle directory set mismatch; missing={sorted(_EXPECTED_DIRECTORIES - directories)}, extra={sorted(directories - _EXPECTED_DIRECTORIES)}")
    lock_bytes = _read_exact_file(root, "OEM_EVIDENCE_LOCK.json", size=explicit_lock.stat().st_size, sha256=OEM_LOCK_SHA256)
    try:
        lock = json.loads(lock_bytes)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise OemMachineBundleError(f"evidence lock is not exact JSON: {exc}") from exc
    if not isinstance(lock, Mapping):
        raise OemMachineBundleError("evidence lock root must be an object")
    _require_lock_shape(lock)
    live_rows = _lock_live_records(lock)
    records: dict[str, OemRecordProvenance] = {}
    for contract in _LIVE_RECORD_CONTRACTS:
        row = live_rows[contract.bundle_relative_path]
        _validate_live_metadata(row, contract)
        data = _read_exact_file(root, contract.bundle_relative_path, size=contract.size, sha256=contract.sha256)
        records[contract.bundle_relative_path] = _record_from_live(row, contract, data)
    current_cal, deployment_time = _cross_root_records(lock, root)
    records[current_cal.bundle_relative_path] = current_cal
    records[deployment_time.bundle_relative_path] = deployment_time
    _reject_operation_policy_collisions(root, Path(process_current_directory) if process_current_directory is not None else None)

    config_fields, axis_limits, sections, positions = _parse_config(records["appdata/config.xml"].raw_bytes)
    operation, operation_fields = parse_operation_parameters_bytes(records["appdata/Operation_parameters.xml"].raw_bytes)
    fields = {**config_fields, **operation_fields}
    serial = int(fields["machine.serial"].value)
    if serial != OEM_MACHINE_SERIAL:
        raise OemMachineBundleError(f"machine identity mismatch: expected {OEM_MACHINE_SERIAL}, got {serial}")
    if fields["machine.calibrated"].value not in (1, True) or fields["machine.camera_calibrated"].value is not True:
        raise OemMachineBundleError("serial-206 asserted machine/camera calibration is missing")
    if operation["DeckInspection"] is not True or operation["CheckCamera"] is not True:
        raise OemMachineBundleError("serial-206 operation policy readiness flags mismatch")
    inspection = _parse_inspection(records["appdata/InspectionSettings.xml"].raw_bytes)
    missing_templates = sorted(_REQUIRED_VISION_TEMPLATES - set(records))
    if missing_templates:
        raise OemMachineBundleError(f"mandatory case-sensitive vision templates are missing: {missing_templates}")
    installed_cal = _parse_calibration(records["appdata/calreference.xml"].raw_bytes, spelling="REVISION", record="appdata/calreference.xml")
    latest_cal = _parse_calibration(records["current_directory/calreference.xml"].raw_bytes, spelling="REVERSION", record="current_directory/calreference.xml")
    comparison = _calibration_comparison(installed_cal, latest_cal, str(fields["machine.calibration_tool"].value))
    process_times, process_time_fields = _parse_process_times(records["appdata/processtime.xml"].raw_bytes)
    fields.update(process_time_fields)
    fields["inspection.selected_profile"] = _field(
        "inspection.selected_profile",
        "Settings3200",
        "element_name",
        "appdata/InspectionSettings.xml",
        "./Settings3200",
        interpretation="serial-206 is not evidenced as BioXP 3250",
    )
    fields["calibration.installed.tool_revision"] = _field(
        "calibration.installed.tool_revision",
        str(comparison["installed_tool_revision"]),
        "REVISION",
        "appdata/calreference.xml",
        f"./PartNumber_{comparison['tool_name']}/Reference/@REVISION",
    )
    fields["calibration.latest.tool_revision"] = _field(
        "calibration.latest.tool_revision",
        str(comparison["latest_tool_revision"]),
        "REVERSION",
        "current_directory/calreference.xml",
        f"./PartNumber_{comparison['tool_name']}/Reference/@REVERSION",
    )

    label_matched = False
    if operator_label_serial is not None:
        try:
            label_matched = int(operator_label_serial) == OEM_MACHINE_SERIAL
        except (TypeError, ValueError):
            label_matched = False
        if not label_matched:
            raise OemMachineBundleError("operator-controlled physical-label serial does not match immutable machine identity")
    if require_operator_label and not label_matched:
        raise OemMachineBundleError(f"{OEM_PHYSICAL_LABEL_SERIAL_ENV}=206 is required before accepted live mutation")
    mutable = {
        record.runtime_state_relative_path: record
        for record in records.values()
        if record.runtime_state_relative_path is not None
    }
    return OemMachineSnapshot(
        acquisition_id=OEM_ACQUISITION_ID,
        machine_serial=serial,
        lock_sha256=OEM_LOCK_SHA256,
        bundle_root=root,
        records=MappingProxyType(records),
        fields=MappingProxyType(fields),
        axis_limits=_freeze(axis_limits),
        position_table=positions,
        config_sections=_freeze(sections),
        operation_parameters=operation,
        inspection_profile_name="Settings3200",
        inspection_profile=inspection,
        calibration_comparison=comparison,
        process_times=process_times,
        mutable_seeds=MappingProxyType(mutable),
        operator_label_matched=label_matched,
        validation_conflicts=(),
    )
