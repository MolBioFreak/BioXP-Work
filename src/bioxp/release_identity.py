from __future__ import annotations

import hashlib
import json
import os
import re
import stat
import threading
from pathlib import Path
from typing import Any, Mapping, cast


_RELEASE_DIR = Path("/run/bioxp-release")
_RELEASE_MODE_MARKER = _RELEASE_DIR / "release-mode"
_RECEIPT_PATH = _RELEASE_DIR / "release-identity.json"
_SOURCE_MANIFEST_PATH = _RELEASE_DIR / "source-manifest.json"
_IMAGE_INSPECTION_PATH = _RELEASE_DIR / "image-inspection.json"
_RUNTIME_BINDING_PATH = _RELEASE_DIR / "runtime-binding.json"
_RUNTIME_OEM_LOCK_PATH = Path("/app/.oem_lock/OEM_EVIDENCE_LOCK.json")
_HEX40 = re.compile(r"^[0-9a-f]{40}$")
_HEX64 = re.compile(r"^[0-9a-f]{64}$")
_IMAGE_ID = re.compile(r"^sha256:[0-9a-f]{64}$")
_EXPECTED_CONFIGURATION = {
    "declared_listener": {"host": "0.0.0.0", "port": 8123},
    "database_root": "/app/.oem_runtime_state",
    "working_directory": "/app",
    "argv": ["python", "-m", "uvicorn", "bioxp.api:app", "--host", "0.0.0.0", "--port", "8123"],
    "environment": {
        "BIOXP_OEM_MACHINE_BUNDLE_LOCK": "/app/.oem_lock/OEM_EVIDENCE_LOCK.json",
        "BIOXP_OEM_RUNTIME_ROOT": "/app/.oem_runtime_state",
        "BIOXP_OEM_RUNTIME_STATE_ROOT": "/app/.oem_runtime_state",
        "BIOXP_PHYSICAL_LABEL_SERIAL": "206",
        "PYTHONPATH": "/app/src",
    },
}
_lock = threading.RLock()
_cached: dict[str, Any] | None = None


class ReleaseIdentityError(RuntimeError):
    """The canonical release packet is absent, malformed, or contradictory."""


def _canonical(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _process_cgroup() -> str:
    try:
        return Path("/proc/self/cgroup").read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _observation(*, observed_listener: Mapping[str, Any] | None = None, database_root: str | None = None, started_at: float | None = None) -> dict[str, Any]:
    cgroup = _process_cgroup()
    return {
        "pid": os.getpid(),
        "cgroup": cgroup or None,
        "cgroup_sha256": _sha256((cgroup + "\n").encode("utf-8")) if cgroup else None,
        "started_at": started_at,
        "listener": dict(observed_listener) if isinstance(observed_listener, Mapping) else None,
        "database_root": database_root,
    }


def _unverified(reason: str) -> dict[str, Any]:
    return {
        "schema": "bioxp.runtime.release_identity.v1",
        "status": "unverified",
        "verified": False,
        "reason": reason,
        "release_id": None,
        "source": {
            "commit": None,
            "tree": None,
            "mode": None,
            "root": None,
            "manifest_sha256": None,
            "aggregate_sha256": None,
        },
        "image": {"id": None, "inspection_receipt_sha256": None},
        "deployment": {"receipt_id": None, "installed_at": None, "receipt_sha256": None},
        "binding": {
            "service_unit": None,
            "unit_path": None,
            "unit_sha256": None,
            "launcher_path": None,
            "launcher_sha256": None,
            "configuration_sha256": None,
            "oem_lock_path": None,
            "oem_lock_sha256": None,
            "declared_listener": None,
            "observed_listener": None,
            "database_root": None,
            "systemd_invocation_id": None,
        },
        "runtime_release_receipt": None,
        "observation": _observation(),
    }


def _read_protected_json(path: Path, *, root_owned: bool) -> tuple[dict[str, Any], bytes]:
    try:
        info = path.lstat()
    except OSError as exc:
        raise ReleaseIdentityError(f"release packet file unavailable: {path}: {exc}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise ReleaseIdentityError(f"release packet path is not a regular non-symlink file: {path}")
    if info.st_mode & 0o022:
        raise ReleaseIdentityError(f"release packet file is group/world writable: {path}")
    if root_owned and info.st_uid != 0:
        raise ReleaseIdentityError(f"release packet file is not root-owned: {path}")
    try:
        raw = path.read_bytes()
        value = json.loads(raw)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ReleaseIdentityError(f"release packet file is unreadable JSON: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ReleaseIdentityError(f"release packet file is not a JSON object: {path}")
    return value, raw


def _required_text(value: Mapping[str, Any], key: str) -> str:
    selected = value.get(key)
    if not isinstance(selected, str) or not selected:
        raise ReleaseIdentityError(f"release packet field is missing: {key}")
    return selected


def _require_digest(value: str, field: str, pattern: re.Pattern[str] = _HEX64) -> str:
    if pattern.fullmatch(value) is None:
        raise ReleaseIdentityError(f"release packet field is not a canonical digest: {field}")
    return value


def _manifest_aggregate(manifest: Mapping[str, Any]) -> str:
    files = manifest.get("files")
    file_count = manifest.get("file_count")
    total_size = manifest.get("total_size")
    if (
        not isinstance(files, list)
        or not files
        or type(file_count) is not int
        or int(file_count) != len(files)
        or type(total_size) is not int
        or int(total_size) < 0
    ):
        raise ReleaseIdentityError("source manifest inventory is incomplete")
    if files != sorted(files, key=lambda row: str(row.get("path", "")).encode("utf-8") if isinstance(row, Mapping) else b""):
        raise ReleaseIdentityError("source manifest inventory is not bytewise sorted")
    aggregate = hashlib.sha256()
    total = 0
    seen: set[str] = set()
    for row in files:
        if not isinstance(row, Mapping) or set(row) != {"path", "size", "sha256"}:
            raise ReleaseIdentityError("source manifest inventory row is malformed")
        path = row.get("path")
        size = row.get("size")
        digest = row.get("sha256")
        if (
            not isinstance(path, str)
            or not path
            or path.startswith("/")
            or ".." in Path(path).parts
            or path in seen
            or type(size) is not int
            or size < 0
            or not isinstance(digest, str)
            or _HEX64.fullmatch(digest) is None
        ):
            raise ReleaseIdentityError("source manifest inventory row is unsafe or noncanonical")
        seen.add(path)
        total += size
        aggregate.update(path.encode("utf-8") + b"\0" + str(size).encode("ascii") + b"\0" + digest.encode("ascii") + b"\n")
    if int(total_size) != total:
        raise ReleaseIdentityError("source manifest total size is contradictory")
    return aggregate.hexdigest()


def _verified_identity() -> dict[str, Any]:
    receipt, receipt_raw = _read_protected_json(_RECEIPT_PATH, root_owned=True)
    manifest, manifest_raw = _read_protected_json(_SOURCE_MANIFEST_PATH, root_owned=True)
    inspection, inspection_raw = _read_protected_json(_IMAGE_INSPECTION_PATH, root_owned=True)
    runtime, _ = _read_protected_json(_RUNTIME_BINDING_PATH, root_owned=False)

    if receipt.get("schema") != "bioxp.release.identity.v1" or receipt.get("status") != "verified":
        raise ReleaseIdentityError("canonical deployment receipt schema/status is not verified")
    if manifest.get("schema") != "bioxp.release.source_manifest.v1" or manifest.get("status") != "verified":
        raise ReleaseIdentityError("canonical source manifest schema/status is not verified")
    if inspection.get("schema") != "bioxp.release.image_inspection.v1" or inspection.get("status") != "verified":
        raise ReleaseIdentityError("external image inspection schema/status is not verified")
    if runtime.get("schema") != "bioxp.release.runtime_binding.v1" or runtime.get("status") != "verified":
        raise ReleaseIdentityError("runtime binding schema/status is not verified")
    if set(receipt) != {"schema", "status", "release_id", "source", "image", "deployment", "binding"}:
        raise ReleaseIdentityError("canonical deployment receipt has unknown or missing fields")
    if set(manifest) != {
        "schema", "status", "commit", "tree", "root", "inventory_algorithm",
        "aggregate_algorithm", "file_count", "total_size", "aggregate_sha256", "files",
    }:
        raise ReleaseIdentityError("source manifest has unknown or missing fields")
    if (
        manifest.get("inventory_algorithm") != "sorted-utf8-path-size-sha256-v1"
        or manifest.get("aggregate_algorithm")
        != "sha256(path_utf8+nul+decimal_size+nul+file_sha256+lf)"
    ):
        raise ReleaseIdentityError("source manifest algorithms are not canonical")
    if set(inspection) != {
        "schema", "status", "inspected_at", "verifier", "requested_image_id",
        "resolved_local_image_id", "oci_labels", "embedded_source_manifest",
        "source_bytes_verified",
    }:
        raise ReleaseIdentityError("image inspection receipt has unknown or missing fields")
    if not isinstance(inspection.get("inspected_at"), str) or not inspection["inspected_at"]:
        raise ReleaseIdentityError("image inspection timestamp is missing")
    if set(runtime) != {
        "schema", "status", "release_id", "deployment_receipt_id", "installed_at",
        "canonical_receipt_sha256", "source_manifest_sha256", "source_aggregate_sha256",
        "image_id", "image_inspection_receipt_sha256", "service_unit", "unit_sha256",
        "udocker_path", "udocker_sha256", "udocker_tree_sha256",
        "launcher_sha256", "configuration_sha256", "oem_lock_sha256", "launcher_pid",
        "launcher_cgroup", "launcher_cgroup_sha256", "systemd_invocation_id",
        "declared_listener", "observed_listener", "database_root", "binding_created_at",
    }:
        raise ReleaseIdentityError("runtime binding has unknown or missing fields")
    if not isinstance(runtime.get("binding_created_at"), (int, float)):
        raise ReleaseIdentityError("runtime binding creation time is invalid")
    if runtime.get("udocker_path") != "/opt/bioxp/udocker-runtime/venv/bin/udocker":
        raise ReleaseIdentityError("runtime binding does not use the immutable udocker launcher")
    udocker_sha256 = _require_digest(_required_text(runtime, "udocker_sha256"), "runtime.udocker_sha256")
    udocker_tree_sha256 = _require_digest(_required_text(runtime, "udocker_tree_sha256"), "runtime.udocker_tree_sha256")

    source = receipt.get("source")
    image = receipt.get("image")
    deployment = receipt.get("deployment")
    binding = receipt.get("binding")
    if not all(isinstance(item, Mapping) for item in (source, image, deployment, binding)):
        raise ReleaseIdentityError("canonical deployment receipt has incomplete identity sections")
    source = cast(Mapping[str, Any], source)
    image = cast(Mapping[str, Any], image)
    deployment = cast(Mapping[str, Any], deployment)
    binding = cast(Mapping[str, Any], binding)
    if set(source) != {
        "commit", "tree", "mode", "root", "host_path", "manifest_sha256", "aggregate_sha256",
    }:
        raise ReleaseIdentityError("deployment source identity has unknown or missing fields")
    if set(image) != {"id", "inspection_receipt_sha256"}:
        raise ReleaseIdentityError("deployment image identity has unknown or missing fields")
    if set(deployment) != {"receipt_id", "installed_at"}:
        raise ReleaseIdentityError("deployment identity has unknown or missing fields")
    if set(binding) != {
        "service_unit", "unit_path", "unit_sha256", "launcher_path", "launcher_sha256",
        "configuration", "configuration_sha256", "oem_lock_path", "oem_lock_sha256",
    }:
        raise ReleaseIdentityError("deployment binding has unknown or missing fields")

    commit = _require_digest(_required_text(source, "commit"), "source.commit", _HEX40)
    tree = _require_digest(_required_text(source, "tree"), "source.tree", _HEX40)
    manifest_sha256 = _require_digest(_required_text(source, "manifest_sha256"), "source.manifest_sha256")
    aggregate_sha256 = _require_digest(_required_text(source, "aggregate_sha256"), "source.aggregate_sha256")
    image_id = _require_digest(_required_text(image, "id"), "image.id", _IMAGE_ID)
    inspection_sha256 = _require_digest(_required_text(image, "inspection_receipt_sha256"), "image.inspection_receipt_sha256")
    source_mode = _required_text(source, "mode")
    source_root = _required_text(source, "root")
    host_path = source.get("host_path")
    if (
        source_mode != "exact_commit_materialization"
        or source_root != "/app"
        or host_path != f"/opt/bioxp/releases/{commit}"
    ):
        raise ReleaseIdentityError("exact source materialization binding is contradictory")

    if _sha256(manifest_raw) != manifest_sha256:
        raise ReleaseIdentityError("source manifest byte digest does not match the deployment receipt")
    if _sha256(inspection_raw) != inspection_sha256:
        raise ReleaseIdentityError("image inspection byte digest does not match the deployment receipt")
    if manifest.get("commit") != commit or manifest.get("tree") != tree or manifest.get("root") != source_root:
        raise ReleaseIdentityError("source manifest commit/tree/root contradicts the deployment receipt")
    if manifest.get("aggregate_sha256") != aggregate_sha256 or _manifest_aggregate(manifest) != aggregate_sha256:
        raise ReleaseIdentityError("source manifest aggregate digest contradicts the deployment receipt")

    if inspection.get("requested_image_id") != image_id or inspection.get("resolved_local_image_id") != image_id:
        raise ReleaseIdentityError("external image inspection does not resolve the immutable image ID")
    if inspection.get("source_bytes_verified") is not True:
        raise ReleaseIdentityError("external image inspection did not verify source bytes")
    verifier = inspection.get("verifier")
    labels = inspection.get("oci_labels")
    embedded = inspection.get("embedded_source_manifest")
    if not all(isinstance(item, Mapping) for item in (verifier, labels, embedded)):
        raise ReleaseIdentityError("external image inspection authority is incomplete")
    verifier = cast(Mapping[str, Any], verifier)
    labels = cast(Mapping[str, Any], labels)
    embedded = cast(Mapping[str, Any], embedded)
    if set(verifier) != {"path", "sha256"}:
        raise ReleaseIdentityError("external image inspector identity is malformed")
    if set(labels) != {
        "org.opencontainers.image.revision",
        "com.bioxp.source.tree",
        "com.bioxp.source.manifest.sha256",
    }:
        raise ReleaseIdentityError("external image labels are malformed")
    if set(embedded) != {"path", "sha256", "aggregate_sha256"}:
        raise ReleaseIdentityError("embedded source manifest identity is malformed")
    if verifier.get("path") != "/usr/local/libexec/bioxp-udocker-image-inspector":
        raise ReleaseIdentityError("external image inspector path is not canonical")
    _require_digest(_required_text(verifier, "sha256"), "image_inspection.verifier.sha256")
    if labels.get("org.opencontainers.image.revision") != commit or labels.get("com.bioxp.source.tree") != tree:
        raise ReleaseIdentityError("external image inspection source labels are contradictory")
    if labels.get("com.bioxp.source.manifest.sha256") != manifest_sha256:
        raise ReleaseIdentityError("external image inspection manifest label is contradictory")
    if (
        embedded.get("path") != "/usr/share/bioxp-release/source-manifest.json"
        or embedded.get("sha256") != manifest_sha256
        or embedded.get("aggregate_sha256") != aggregate_sha256
    ):
        raise ReleaseIdentityError("external image inspection embedded manifest is contradictory")

    if binding.get("service_unit") != "bioxp-api.service" or binding.get("unit_path") != "/etc/systemd/system/bioxp-api.service":
        raise ReleaseIdentityError("deployment receipt does not bind the canonical service unit")
    if binding.get("launcher_path") != "/usr/local/libexec/bioxp-release-container-run":
        raise ReleaseIdentityError("deployment receipt does not bind the canonical launcher")
    if binding.get("oem_lock_path") != "/var/lib/bioxp-oem-authority/OEM_EVIDENCE_LOCK.json":
        raise ReleaseIdentityError("deployment receipt does not bind the canonical OEM lock")
    unit_sha256 = _require_digest(_required_text(binding, "unit_sha256"), "binding.unit_sha256")
    launcher_sha256 = _require_digest(_required_text(binding, "launcher_sha256"), "binding.launcher_sha256")
    configuration_sha256 = _require_digest(_required_text(binding, "configuration_sha256"), "binding.configuration_sha256")
    oem_lock_sha256 = _require_digest(_required_text(binding, "oem_lock_sha256"), "binding.oem_lock_sha256")
    configuration = binding.get("configuration")
    if configuration != _EXPECTED_CONFIGURATION or _sha256(_canonical(configuration)) != configuration_sha256:
        raise ReleaseIdentityError("canonical runtime configuration or its digest is contradictory")
    try:
        if _sha256(_RUNTIME_OEM_LOCK_PATH.read_bytes()) != oem_lock_sha256:
            raise ReleaseIdentityError("mounted OEM authority lock digest is contradictory")
    except OSError as exc:
        raise ReleaseIdentityError("mounted OEM authority lock is unavailable") from exc

    declared_listener = _EXPECTED_CONFIGURATION["declared_listener"]
    database_root = _EXPECTED_CONFIGURATION["database_root"]
    release_id = _required_text(receipt, "release_id")
    receipt_id = _required_text(deployment, "receipt_id")
    installed_at = _required_text(deployment, "installed_at")
    receipt_sha256 = _sha256(receipt_raw)
    runtime_pairs = {
        "release_id": release_id,
        "deployment_receipt_id": receipt_id,
        "installed_at": installed_at,
        "canonical_receipt_sha256": receipt_sha256,
        "source_manifest_sha256": manifest_sha256,
        "source_aggregate_sha256": aggregate_sha256,
        "image_id": image_id,
        "image_inspection_receipt_sha256": inspection_sha256,
        "service_unit": "bioxp-api.service",
        "unit_sha256": unit_sha256,
        "launcher_sha256": launcher_sha256,
        "configuration_sha256": configuration_sha256,
        "oem_lock_sha256": oem_lock_sha256,
        "declared_listener": declared_listener,
        "observed_listener": None,
        "database_root": database_root,
    }
    for key, expected in runtime_pairs.items():
        if runtime.get(key) != expected:
            raise ReleaseIdentityError(f"runtime binding contradicts canonical receipt field: {key}")
    release_environment = {
        "BIOXP_RELEASE_ID": release_id,
        "BIOXP_RELEASE_IMAGE_ID": image_id,
        "BIOXP_RELEASE_SOURCE_COMMIT": commit,
        "BIOXP_RELEASE_UDOCKER_SHA256": udocker_sha256,
        "BIOXP_RELEASE_UDOCKER_TREE_SHA256": udocker_tree_sha256,
    }
    if {key: os.environ.get(key) for key in release_environment} != release_environment:
        raise ReleaseIdentityError("application environment contradicts the launcher release/image/runtime binding")
    invocation_id = _required_text(runtime, "systemd_invocation_id")

    cgroup = _process_cgroup()
    if not any(row.endswith("/system.slice/bioxp-api.service") for row in cgroup.splitlines()):
        raise ReleaseIdentityError("runtime process is not owned by the canonical bioxp-api.service cgroup")
    runtime_cgroup = runtime.get("launcher_cgroup")
    if not isinstance(runtime_cgroup, str) or not any(row.endswith("/system.slice/bioxp-api.service") for row in runtime_cgroup.splitlines()):
        raise ReleaseIdentityError("launcher runtime binding is not owned by the canonical service cgroup")
    launcher_cgroup_sha256 = _sha256((runtime_cgroup.strip() + "\n").encode("utf-8"))
    if runtime.get("launcher_cgroup_sha256") != launcher_cgroup_sha256:
        raise ReleaseIdentityError("launcher cgroup bytes contradict their runtime binding digest")
    if cgroup.strip() != runtime_cgroup.strip():
        raise ReleaseIdentityError("runtime process and launcher do not share the exact canonical cgroup")
    launcher_pid = runtime.get("launcher_pid")
    if not isinstance(launcher_pid, int) or launcher_pid <= 1:
        raise ReleaseIdentityError("runtime binding does not contain a valid launcher PID")
    try:
        observed_launcher_cgroup = Path(f"/proc/{launcher_pid}/cgroup").read_text(encoding="utf-8").strip()
    except OSError as exc:
        raise ReleaseIdentityError("bound launcher PID is not observable") from exc
    if observed_launcher_cgroup != runtime_cgroup.strip():
        raise ReleaseIdentityError("bound launcher PID no longer owns the canonical cgroup")

    return {
        "schema": "bioxp.runtime.release_identity.v1",
        "status": "verified",
        "verified": True,
        "reason": None,
        "release_id": release_id,
        "source": {
            "commit": commit,
            "tree": tree,
            "mode": source_mode,
            "root": source_root,
            "host_path": host_path or None,
            "manifest_sha256": manifest_sha256,
            "aggregate_sha256": aggregate_sha256,
        },
        "image": {"id": image_id, "inspection_receipt_sha256": inspection_sha256},
        "deployment": {
            "receipt_id": receipt_id,
            "installed_at": installed_at,
            "receipt_sha256": receipt_sha256,
        },
        "binding": {
            "service_unit": "bioxp-api.service",
            "unit_path": "/etc/systemd/system/bioxp-api.service",
            "unit_sha256": unit_sha256,
            "launcher_path": "/usr/local/libexec/bioxp-release-container-run",
            "launcher_sha256": launcher_sha256,
            "configuration_sha256": configuration_sha256,
            "oem_lock_path": "/var/lib/bioxp-oem-authority/OEM_EVIDENCE_LOCK.json",
            "oem_lock_sha256": oem_lock_sha256,
            "udocker_path": "/opt/bioxp/udocker-runtime/venv/bin/udocker",
            "udocker_sha256": udocker_sha256,
            "udocker_tree_sha256": udocker_tree_sha256,
            "declared_listener": dict(declared_listener),
            "observed_listener": None,
            "database_root": database_root,
            "launcher_pid": launcher_pid,
            "launcher_cgroup_sha256": launcher_cgroup_sha256,
            "systemd_invocation_id": invocation_id,
        },
        "runtime_release_receipt": None,
        "observation": _observation(database_root=database_root),
    }


def configure_release_identity() -> dict[str, Any]:
    """Load the one canonical receipt; fail closed whenever release mode is marked."""
    global _cached
    with _lock:
        release_mode = _RELEASE_MODE_MARKER.exists()
        packet_present = any(
            path.exists()
            for path in (_RECEIPT_PATH, _SOURCE_MANIFEST_PATH, _IMAGE_INSPECTION_PATH, _RUNTIME_BINDING_PATH)
        )
        if not release_mode and not packet_present:
            _cached = _unverified("canonical_release_packet_absent")
            return json.loads(_canonical(_cached))
        try:
            _cached = _verified_identity()
        except ReleaseIdentityError as exc:
            if release_mode:
                raise
            _cached = _unverified(f"canonical_release_packet_unverified:{exc}")
        return json.loads(_canonical(_cached))


def publish_runtime_release_receipt(receipt: Mapping[str, Any]) -> dict[str, Any]:
    """Publish only a previously committed runtime-start receipt as observation."""
    global _cached
    with _lock:
        if _cached is None or _cached.get("verified") is not True:
            raise ReleaseIdentityError("verified release identity must precede runtime observation")
        if receipt.get("release_id") != _cached.get("release_id"):
            raise ReleaseIdentityError("runtime start receipt release identity mismatch")
        binding = _cached.get("binding")
        if not isinstance(binding, dict):
            raise ReleaseIdentityError("runtime release binding is unavailable")
        digest_pairs = {
            "canonical_receipt_sha256": _cached["deployment"]["receipt_sha256"],
            "source_manifest_sha256": _cached["source"]["manifest_sha256"],
            "source_aggregate_sha256": _cached["source"]["aggregate_sha256"],
            "image_id": _cached["image"]["id"],
            "image_inspection_receipt_sha256": _cached["image"]["inspection_receipt_sha256"],
            "unit_sha256": binding["unit_sha256"],
            "launcher_sha256": binding["launcher_sha256"],
            "configuration_sha256": binding["configuration_sha256"],
            "oem_lock_sha256": binding["oem_lock_sha256"],
            "udocker_path": binding["udocker_path"],
            "udocker_sha256": binding["udocker_sha256"],
            "udocker_tree_sha256": binding["udocker_tree_sha256"],
            "systemd_invocation_id": binding["systemd_invocation_id"],
        }
        for key, expected in digest_pairs.items():
            if receipt.get(key) != expected:
                raise ReleaseIdentityError(f"runtime start receipt contradicts release identity: {key}")
        observed_listener = receipt.get("observed_listener")
        if not isinstance(observed_listener, Mapping):
            raise ReleaseIdentityError("runtime start receipt has no measured listener")
        binding["observed_listener"] = dict(observed_listener)
        _cached["runtime_release_receipt"] = {
            "receipt_id": receipt.get("receipt_id"),
            "receipt_sha256": receipt.get("receipt_sha256"),
            "recorded_at": receipt.get("recorded_at"),
        }
        _cached["observation"] = _observation(
            observed_listener=observed_listener,
            database_root=binding.get("database_root"),
            started_at=receipt.get("application_started_at"),
        )
        return json.loads(_canonical(_cached))


def current_release_identity() -> dict[str, Any]:
    global _cached
    with _lock:
        if _cached is None:
            return configure_release_identity()
        value = json.loads(_canonical(_cached))
        binding = value.get("binding") if isinstance(value.get("binding"), dict) else {}
        observed = binding.get("observed_listener") if isinstance(binding.get("observed_listener"), dict) else None
        existing_observation = value.get("observation") if isinstance(value.get("observation"), dict) else {}
        value["observation"] = _observation(
            observed_listener=observed,
            database_root=binding.get("database_root"),
            started_at=existing_observation.get("started_at"),
        )
        return value


def public_release_identity(value: Mapping[str, Any]) -> dict[str, Any]:
    """Return the path-free release identity allowed on public routes."""
    source_value = value.get("source")
    source = source_value if isinstance(source_value, Mapping) else {}
    image_value = value.get("image")
    image = image_value if isinstance(image_value, Mapping) else {}
    deployment_value = value.get("deployment")
    deployment = deployment_value if isinstance(deployment_value, Mapping) else {}
    binding_value = value.get("binding")
    binding = binding_value if isinstance(binding_value, Mapping) else {}
    declared_value = binding.get("declared_listener")
    declared = declared_value if isinstance(declared_value, Mapping) else None
    observed_value = binding.get("observed_listener")
    observed = observed_value if isinstance(observed_value, Mapping) else None
    return {
        "schema": value.get("schema"),
        "status": value.get("status"),
        "verified": value.get("verified") is True,
        "reason_code": None
        if value.get("verified") is True
        else "release_identity_unavailable",
        "release_id": value.get("release_id"),
        "source": {
            key: source.get(key)
            for key in (
                "commit",
                "tree",
                "mode",
                "manifest_sha256",
                "aggregate_sha256",
            )
        },
        "image": {
            key: image.get(key)
            for key in ("id", "inspection_receipt_sha256")
        },
        "deployment": {
            key: deployment.get(key)
            for key in ("receipt_id", "installed_at", "receipt_sha256")
        },
        "binding": {
            **{
                key: binding.get(key)
                for key in (
                "service_unit",
                "unit_sha256",
                "launcher_sha256",
                "configuration_sha256",
                "oem_lock_sha256",
                "udocker_sha256",
                "udocker_tree_sha256",
                )
            },
            "declared_listener": (
                None
                if declared is None
                else {key: declared.get(key) for key in ("host", "port")}
            ),
            "observed_listener": (
                None
                if observed is None
                else {key: observed.get(key) for key in ("host", "port")}
            )
        },
    }
