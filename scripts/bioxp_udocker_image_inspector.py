#!/usr/bin/env python3
"""Inspect and re-verify normalized immutable uDocker image stores."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import shutil
import stat
import tarfile
import tempfile
from pathlib import Path
from typing import Any

IMAGE_ID_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
HEX40_RE = re.compile(r"^[0-9a-f]{40}$")
HEX64_RE = re.compile(r"^[0-9a-f]{64}$")
CANONICAL_VERIFIER_PATH = "/usr/local/libexec/bioxp-udocker-image-inspector"
EMBEDDED_MANIFEST_PATH = "/usr/share/bioxp-release/source-manifest.json"
REQUIRED_LABELS = (
    "org.opencontainers.image.revision",
    "com.bioxp.source.tree",
    "com.bioxp.source.manifest.sha256",
)


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _need(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _safe_regular(path: Path, description: str) -> bytes:
    info = path.lstat()
    _need(stat.S_ISREG(info.st_mode) and not path.is_symlink(), f"{description} is not a safe regular file")
    return path.read_bytes()


def validate_manifest_bytes(raw: bytes) -> dict[str, Any]:
    try:
        manifest = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError("source manifest is malformed") from exc
    _need(isinstance(manifest, dict), "source manifest is not an object")
    _need(manifest.get("schema") == "bioxp.release.source_manifest.v1" and manifest.get("status") == "verified", "source manifest schema/status is invalid")
    _need(HEX40_RE.fullmatch(str(manifest.get("commit", ""))) is not None, "source commit is not full 40-hex")
    _need(HEX40_RE.fullmatch(str(manifest.get("tree", ""))) is not None, "source tree is not full 40-hex")
    _need(manifest.get("root") == "/app", "source manifest root is not /app")
    _need(manifest.get("inventory_algorithm") == "sorted-utf8-path-size-sha256-v1", "source inventory algorithm is invalid")
    _need(manifest.get("aggregate_algorithm") == "sha256(path_utf8+nul+decimal_size+nul+file_sha256+lf)", "source aggregate algorithm is invalid")
    rows = manifest.get("files")
    _need(isinstance(rows, list) and bool(rows), "source manifest inventory is empty")
    _need(type(manifest.get("file_count")) is int and manifest["file_count"] == len(rows), "source manifest file count is invalid")
    aggregate = hashlib.sha256()
    total = 0
    seen: set[str] = set()
    previous: bytes | None = None
    for row in rows:
        _need(isinstance(row, dict) and set(row) == {"path", "size", "sha256"}, "source manifest row is malformed")
        relative, size, digest = row["path"], row["size"], row["sha256"]
        _need(isinstance(relative, str) and bool(relative) and not relative.startswith("/") and ".." not in Path(relative).parts, "source manifest path is unsafe")
        encoded = relative.encode("utf-8")
        _need(relative not in seen and (previous is None or previous < encoded), "source manifest paths are duplicate or unsorted")
        _need(type(size) is int and size >= 0, "source manifest size is invalid")
        _need(isinstance(digest, str) and HEX64_RE.fullmatch(digest) is not None, "source manifest file digest is invalid")
        seen.add(relative)
        previous = encoded
        total += size
        aggregate.update(encoded + b"\0" + str(size).encode("ascii") + b"\0" + digest.encode("ascii") + b"\n")
    _need(type(manifest.get("total_size")) is int and manifest["total_size"] == total, "source manifest total size is invalid")
    _need(manifest.get("aggregate_sha256") == aggregate.hexdigest(), "source manifest aggregate digest mismatch")
    return manifest


def _image_root(store: Path, image_id: str) -> Path:
    _need(IMAGE_ID_RE.fullmatch(image_id) is not None, "image ID must be a full immutable sha256:<64-hex>")
    _need(store.exists() and store.is_dir() and not store.is_symlink(), "uDocker image store is absent or symlinked")
    root = store / "images" / "sha256" / image_id[7:]
    if root.exists() and root.is_dir() and not root.is_symlink():
        return root
    matches = []
    for tag_root in sorted((store / "repos").glob("*/*")):
        config_path = tag_root / "container.json"
        if not config_path.is_file():
            config_path = tag_root / f"{image_id[7:]}.layer"
        if config_path.is_file() and sha256_file(config_path) == image_id[7:]:
            matches.append(tag_root)
    _need(len(matches) == 1, "full immutable image ID does not have exactly one uDocker tag authority")
    return matches[0]


def _materialize_udocker_rootfs(store: Path, tag_root: Path, destination: Path) -> None:
    ancestry = json.loads(_safe_regular(tag_root / "ancestry", "uDocker ancestry"))
    _need(isinstance(ancestry, list) and bool(ancestry), "uDocker ancestry is invalid")
    destination.mkdir(parents=True, exist_ok=True)
    for digest in ancestry:
        _need(isinstance(digest, str) and HEX64_RE.fullmatch(digest) is not None, "uDocker layer digest is invalid")
        layer = store / "layers" / f"{digest}.layer"
        _need(layer.is_file() and not layer.is_symlink() and sha256_file(layer) == digest, f"uDocker layer is absent or corrupt: {digest}")
        with tarfile.open(layer, "r:*") as archive:
            members = archive.getmembers()
            for member in members:
                parts = Path(member.name).parts
                _need(not member.name.startswith("/") and ".." not in parts, f"unsafe image layer path: {member.name}")
                name = Path(member.name).name
                parent = destination.joinpath(*parts[:-1]) if parts[:-1] else destination
                if name == ".wh..wh..opq" and parent.is_dir():
                    for child in parent.iterdir():
                        if child.is_dir() and not child.is_symlink():
                            shutil.rmtree(child)
                        else:
                            child.unlink()
                elif name.startswith(".wh."):
                    target = parent / name[4:]
                    if target.exists() or target.is_symlink():
                        if target.is_dir() and not target.is_symlink():
                            shutil.rmtree(target)
                        else:
                            target.unlink()
            def image_filter(member: tarfile.TarInfo, path: str) -> tarfile.TarInfo | None:
                selected = member
                if member.issym() and member.linkname.startswith("/"):
                    selected = copy.copy(member)
                    parent = Path(member.name).parent
                    selected.linkname = os.path.relpath(member.linkname.lstrip("/"), parent.as_posix() or ".")
                return tarfile.data_filter(selected, path)

            archive.extractall(destination, filter=image_filter)
    for marker in destination.rglob(".wh.*"):
        marker.unlink()


def _measure(store: Path, image_id: str, source_manifest: Path) -> tuple[dict[str, Any], dict[str, str]]:
    root = _image_root(store, image_id)
    normalized = (root / "oci-config.json").is_file()
    for path in (root, *root.rglob("*")):
        if normalized:
            _need(not stat.S_ISLNK(path.lstat().st_mode), f"image store symlink is forbidden: {path}")
        elif path.is_symlink():
            resolved = path.resolve(strict=True)
            _need(resolved.parent == (store / "layers").resolve(), f"uDocker image symlink escapes the layer store: {path}")
    config_path = root / "oci-config.json" if normalized else root / "container.json"
    if not normalized and not config_path.is_file():
        config_path = root / f"{image_id[7:]}.layer"
    config_raw = _safe_regular(config_path, "OCI config") if normalized else config_path.read_bytes()
    _need("sha256:" + hashlib.sha256(config_raw).hexdigest() == image_id, "OCI config digest does not equal the requested full image ID")
    try:
        config = json.loads(config_raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError("OCI config is malformed") from exc
    config_section = config.get("config") if isinstance(config, dict) else None
    labels = config_section.get("Labels") if isinstance(config_section, dict) else None
    _need(isinstance(labels, dict), "OCI labels are absent")
    selected = {name: labels.get(name) for name in REQUIRED_LABELS}
    _need(HEX40_RE.fullmatch(str(selected[REQUIRED_LABELS[0]] or "")) is not None, "OCI commit label is not exact")
    _need(HEX40_RE.fullmatch(str(selected[REQUIRED_LABELS[1]] or "")) is not None, "OCI tree label is not exact")
    _need(HEX64_RE.fullmatch(str(selected[REQUIRED_LABELS[2]] or "")) is not None, "OCI manifest label is not exact")

    external_raw = _safe_regular(source_manifest, "external source manifest")
    external = validate_manifest_bytes(external_raw)
    temporary: tempfile.TemporaryDirectory[str] | None = None
    if normalized:
        rootfs = root / "rootfs"
    else:
        temporary = tempfile.TemporaryDirectory(prefix="bioxp-udocker-inspection-")
        rootfs = Path(temporary.name)
        _materialize_udocker_rootfs(store, root, rootfs)
    embedded_path = rootfs / EMBEDDED_MANIFEST_PATH.lstrip("/")
    embedded_raw = _safe_regular(embedded_path, "embedded source manifest")
    _need(embedded_raw == external_raw, "embedded source manifest bytes differ from the external manifest")
    embedded = validate_manifest_bytes(embedded_raw)
    manifest_sha = hashlib.sha256(external_raw).hexdigest()
    _need(selected[REQUIRED_LABELS[0]] == external["commit"], "OCI commit label contradicts the source manifest")
    _need(selected[REQUIRED_LABELS[1]] == external["tree"], "OCI tree label contradicts the source manifest")
    _need(selected[REQUIRED_LABELS[2]] == manifest_sha, "OCI manifest label contradicts the source manifest bytes")

    app = rootfs / "app"
    _need(app.is_dir() and not app.is_symlink(), "image /app source root is absent or symlinked")
    for row in embedded["files"]:
        candidate = app / row["path"]
        raw = _safe_regular(candidate, f"image source file {row['path']}")
        _need(len(raw) == row["size"] and hashlib.sha256(raw).hexdigest() == row["sha256"], f"image source bytes mismatch: {row['path']}")
    if temporary is not None:
        temporary.cleanup()
    return external, selected


def inspect_image(
    store: Path,
    image_id: str,
    source_manifest: Path,
    inspected_at: str,
    verifier_source: Path,
) -> dict[str, Any]:
    _need(isinstance(inspected_at, str) and bool(inspected_at), "inspection timestamp is required")
    manifest, labels = _measure(Path(store), image_id, Path(source_manifest))
    manifest_raw = Path(source_manifest).read_bytes()
    return {
        "schema": "bioxp.release.image_inspection.v1",
        "status": "verified",
        "inspected_at": inspected_at,
        "verifier": {"path": CANONICAL_VERIFIER_PATH, "sha256": sha256_file(Path(verifier_source))},
        "requested_image_id": image_id,
        "resolved_local_image_id": image_id,
        "oci_labels": labels,
        "embedded_source_manifest": {
            "path": EMBEDDED_MANIFEST_PATH,
            "sha256": hashlib.sha256(manifest_raw).hexdigest(),
            "aggregate_sha256": manifest["aggregate_sha256"],
        },
        "source_bytes_verified": True,
    }


def verify_receipt(
    receipt: dict[str, Any],
    store: Path,
    source_manifest: Path,
    image_id: str,
    verifier_source: Path,
) -> dict[str, Any]:
    _need(isinstance(receipt, dict), "image inspection receipt is not an object")
    inspected_at = receipt.get("inspected_at")
    _need(isinstance(inspected_at, str), "image inspection timestamp is absent")
    expected = inspect_image(store, image_id, source_manifest, str(inspected_at), verifier_source)
    _need(receipt == expected, "image inspection receipt does not match recomputed local image authority")
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    create = sub.add_parser("inspect")
    verify = sub.add_parser("verify")
    for command in (create, verify):
        command.add_argument("--store", type=Path, default=Path("/opt/bioxp/udocker-runtime/store"))
        command.add_argument("--source-manifest", type=Path, required=True)
        command.add_argument("--image-id", required=True)
        command.add_argument("--verifier-source", type=Path, default=Path(__file__))
    create.add_argument("--inspected-at", required=True)
    create.add_argument("--output", type=Path, required=True)
    verify.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "inspect":
        receipt = inspect_image(args.store, args.image_id, args.source_manifest, args.inspected_at, args.verifier_source)
        args.output.write_bytes(canonical_bytes(receipt))
    else:
        receipt = json.loads(args.receipt.read_bytes())
        verify_receipt(receipt, args.store, args.source_manifest, args.image_id, args.verifier_source)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
