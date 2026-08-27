#!/usr/bin/env python3
"""Recompute and seal a canonical BioXP immutable release identity."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import re
import stat
from datetime import datetime
from pathlib import Path
from types import ModuleType
from typing import Any

IDENTIFIER_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
IMAGE_ID_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
CONFIGURATION = {
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


def _inspector() -> ModuleType:
    path = Path(__file__).with_name("bioxp_udocker_image_inspector.py")
    spec = importlib.util.spec_from_file_location("_bioxp_seal_inspector", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load repository image inspector")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def canonical_digest(value: Any) -> str:
    raw = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def sha256_file(path: Path) -> str:
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or path.is_symlink():
        raise RuntimeError(f"authority is not a safe regular file: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _need(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _verify_source(root: Path, manifest: dict[str, Any]) -> None:
    _need(root.is_dir() and not root.is_symlink(), "source root is absent or symlinked")
    expected: dict[str, tuple[int, str]] = {}
    for row in manifest["files"]:
        expected[row["path"]] = (row["size"], row["sha256"])
    observed: dict[str, tuple[int, str]] = {}
    for path in root.rglob("*"):
        relative = path.relative_to(root)
        if relative.parts and relative.parts[0] == ".bioxp-release":
            continue
        info = path.lstat()
        _need(not stat.S_ISLNK(info.st_mode), f"source symlink is forbidden: {relative.as_posix()}")
        if stat.S_ISREG(info.st_mode):
            observed[relative.as_posix()] = (info.st_size, hashlib.sha256(path.read_bytes()).hexdigest())
        else:
            _need(stat.S_ISDIR(info.st_mode), f"unsupported source file type: {relative.as_posix()}")
    _need(observed == expected, "source bytes do not exactly match the manifest inventory")


def _verify_inspection(
    inspection_path: Path,
    manifest: dict[str, Any],
    manifest_sha: str,
    image_id: str,
    inspector_source: Path,
) -> str:
    inspector = _inspector()
    raw = inspection_path.read_bytes()
    try:
        receipt = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError("image inspection receipt is malformed") from exc
    _need(isinstance(receipt, dict), "image inspection receipt is not an object")
    _need(receipt.get("schema") == "bioxp.release.image_inspection.v1" and receipt.get("status") == "verified", "image inspection receipt is not verified v1")
    _need(receipt.get("requested_image_id") == image_id and receipt.get("resolved_local_image_id") == image_id, "image inspection does not bind the exact image ID")
    _need(receipt.get("source_bytes_verified") is True, "image inspection did not verify source bytes")
    verifier = receipt.get("verifier")
    _need(isinstance(verifier, dict) and verifier == {"path": inspector.CANONICAL_VERIFIER_PATH, "sha256": sha256_file(inspector_source)}, "image inspector digest/path mismatch")
    labels = receipt.get("oci_labels")
    expected_labels = {
        "org.opencontainers.image.revision": manifest["commit"],
        "com.bioxp.source.tree": manifest["tree"],
        "com.bioxp.source.manifest.sha256": manifest_sha,
    }
    _need(labels == expected_labels, "image inspection OCI labels contradict source authority")
    embedded = receipt.get("embedded_source_manifest")
    _need(embedded == {"path": inspector.EMBEDDED_MANIFEST_PATH, "sha256": manifest_sha, "aggregate_sha256": manifest["aggregate_sha256"]}, "image inspection embedded manifest authority mismatch")
    _need(set(receipt) == {"schema", "status", "inspected_at", "verifier", "requested_image_id", "resolved_local_image_id", "oci_labels", "embedded_source_manifest", "source_bytes_verified"}, "image inspection receipt has unknown fields")
    return hashlib.sha256(raw).hexdigest()


def seal_release(
    *,
    source_root: Path,
    source_manifest: Path,
    image_inspection: Path,
    image_id: str,
    inspector_source: Path,
    unit: Path,
    launcher: Path,
    oem_lock: Path,
    release_id: str,
    deployment_receipt_id: str,
    installed_at: str,
) -> dict[str, Any]:
    _need(IDENTIFIER_RE.fullmatch(release_id) is not None, "release ID is not canonical")
    _need(IDENTIFIER_RE.fullmatch(deployment_receipt_id) is not None, "deployment receipt ID is not canonical")
    _need(IMAGE_ID_RE.fullmatch(image_id) is not None, "image ID must be full immutable sha256:<64-hex>")
    try:
        parsed_time = datetime.fromisoformat(installed_at.replace("Z", "+00:00"))
    except ValueError as exc:
        raise RuntimeError("installation timestamp is not RFC 3339") from exc
    _need(parsed_time.tzinfo is not None, "installation timestamp must include a timezone")

    inspector = _inspector()
    manifest_raw = source_manifest.read_bytes()
    manifest = inspector.validate_manifest_bytes(manifest_raw)
    _verify_source(Path(source_root), manifest)
    manifest_sha = hashlib.sha256(manifest_raw).hexdigest()
    inspection_sha = _verify_inspection(Path(image_inspection), manifest, manifest_sha, image_id, Path(inspector_source))
    configuration = json.loads(json.dumps(CONFIGURATION))
    return {
        "schema": "bioxp.release.identity.v1",
        "status": "verified",
        "release_id": release_id,
        "source": {
            "commit": manifest["commit"],
            "tree": manifest["tree"],
            "mode": "exact_commit_materialization",
            "root": "/app",
            "host_path": f"/opt/bioxp/releases/{manifest['commit']}",
            "manifest_sha256": manifest_sha,
            "aggregate_sha256": manifest["aggregate_sha256"],
        },
        "image": {"id": image_id, "inspection_receipt_sha256": inspection_sha},
        "deployment": {"receipt_id": deployment_receipt_id, "installed_at": installed_at},
        "binding": {
            "service_unit": "bioxp-api.service",
            "unit_path": "/etc/systemd/system/bioxp-api.service",
            "unit_sha256": sha256_file(Path(unit)),
            "launcher_path": "/usr/local/libexec/bioxp-release-container-run",
            "launcher_sha256": sha256_file(Path(launcher)),
            "configuration": configuration,
            "configuration_sha256": canonical_digest(configuration),
            "oem_lock_path": "/var/lib/bioxp-oem-authority/OEM_EVIDENCE_LOCK.json",
            "oem_lock_sha256": sha256_file(Path(oem_lock)),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    for name in ("source-root", "source-manifest", "image-inspection", "inspector-source", "unit", "launcher", "oem-lock", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--release-id", required=True)
    parser.add_argument("--deployment-receipt-id", required=True)
    parser.add_argument("--installed-at", required=True)
    args = parser.parse_args()
    receipt = seal_release(
        source_root=args.source_root,
        source_manifest=args.source_manifest,
        image_inspection=args.image_inspection,
        image_id=args.image_id,
        inspector_source=args.inspector_source,
        unit=args.unit,
        launcher=args.launcher,
        oem_lock=args.oem_lock,
        release_id=args.release_id,
        deployment_receipt_id=args.deployment_receipt_id,
        installed_at=args.installed_at,
    )
    args.output.write_bytes(canonical_bytes(receipt))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
