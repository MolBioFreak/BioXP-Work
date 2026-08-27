#!/usr/bin/env python3
"""Materialize and verify deterministic BioXP release source bytes."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import tarfile
import tempfile
from pathlib import Path
from typing import Any

HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^[0-9a-f]{64}$")
IGNORED_TOP_LEVEL = {".git", ".bioxp-release"}


def git(repository: Path, *args: str) -> str:
    completed = subprocess.run(
        ["git", "-C", str(repository), *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def source_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for path in root.rglob("*"):
        relative = path.relative_to(root)
        if relative.parts and relative.parts[0] in IGNORED_TOP_LEVEL:
            continue
        if path.is_symlink():
            raise RuntimeError(f"release source symlink is forbidden: {relative.as_posix()}")
        if path.is_file():
            files.append(path)
    return sorted(files, key=lambda item: item.relative_to(root).as_posix().encode("utf-8"))


def inventory(root: Path) -> tuple[list[dict[str, Any]], str, int]:
    rows: list[dict[str, Any]] = []
    aggregate = hashlib.sha256()
    total_size = 0
    for path in source_files(root):
        relative = path.relative_to(root).as_posix()
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        size = len(raw)
        rows.append({"path": relative, "size": size, "sha256": digest})
        aggregate.update(relative.encode("utf-8") + b"\0" + str(size).encode("ascii") + b"\0" + digest.encode("ascii") + b"\n")
        total_size += size
    if not rows:
        raise RuntimeError("release source inventory is empty")
    return rows, aggregate.hexdigest(), total_size


def build_manifest(root: Path, commit: str, tree: str) -> dict[str, Any]:
    rows, aggregate, total_size = inventory(root)
    return {
        "schema": "bioxp.release.source_manifest.v1",
        "status": "verified",
        "commit": commit,
        "tree": tree,
        "root": "/app",
        "inventory_algorithm": "sorted-utf8-path-size-sha256-v1",
        "aggregate_algorithm": "sha256(path_utf8+nul+decimal_size+nul+file_sha256+lf)",
        "file_count": len(rows),
        "total_size": total_size,
        "aggregate_sha256": aggregate,
        "files": rows,
    }


def verify(root: Path, manifest_path: Path) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_bytes())
    if not isinstance(manifest, dict):
        raise RuntimeError("source manifest is not an object")
    if manifest.get("schema") != "bioxp.release.source_manifest.v1" or manifest.get("status") != "verified":
        raise RuntimeError("source manifest schema/status is not verified")
    commit = str(manifest.get("commit") or "")
    tree = str(manifest.get("tree") or "")
    aggregate = str(manifest.get("aggregate_sha256") or "")
    if HEX40.fullmatch(commit) is None or HEX40.fullmatch(tree) is None or HEX64.fullmatch(aggregate) is None:
        raise RuntimeError("source manifest contains a noncanonical digest")
    files = manifest.get("files")
    if (
        type(manifest.get("file_count")) is not int
        or type(manifest.get("total_size")) is not int
        or not isinstance(files, list)
        or not files
        or manifest["file_count"] != len(files)
        or manifest["total_size"] < 0
    ):
        raise RuntimeError("source manifest count/size fields are not exact JSON integers")
    for row in files:
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "size", "sha256"}
            or type(row.get("size")) is not int
            or row["size"] < 0
        ):
            raise RuntimeError("source manifest file size is not an exact JSON integer")
    expected = build_manifest(root, commit, tree)
    if manifest != expected:
        raise RuntimeError("materialized source bytes do not exactly match the deterministic manifest")
    return manifest


def materialize(repository: Path, commitish: str, output: Path) -> Path:
    repository = repository.resolve(strict=True)
    status = git(repository, "status", "--porcelain", "--untracked-files=all")
    if status:
        raise RuntimeError("source repository must be clean before release materialization")
    commit = git(repository, "rev-parse", "--verify", f"{commitish}^{{commit}}")
    tree = git(repository, "rev-parse", "--verify", f"{commit}^{{tree}}")
    if HEX40.fullmatch(commit) is None or HEX40.fullmatch(tree) is None:
        raise RuntimeError("Git did not return full commit/tree identities")
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(suffix=".tar") as archive:
        subprocess.run(
            ["git", "-C", str(repository), "archive", "--format=tar", "--output", archive.name, commit],
            check=True,
        )
        with tarfile.open(archive.name, "r:") as payload:
            for member in payload.getmembers():
                target = (output / member.name).resolve(strict=False)
                if output not in target.parents and target != output:
                    raise RuntimeError(f"archive path escapes materialization: {member.name}")
                if member.issym() or member.islnk():
                    raise RuntimeError(f"release source links are forbidden: {member.name}")
            payload.extractall(output, filter="data")
    packet_dir = output / ".bioxp-release"
    packet_dir.mkdir(mode=0o700)
    manifest_path = packet_dir / "source-manifest.json"
    manifest_path.write_bytes(canonical_bytes(build_manifest(output, commit, tree)))
    os.chmod(manifest_path, 0o444)
    verify(output, manifest_path)
    return manifest_path


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    create = subparsers.add_parser("materialize")
    create.add_argument("--repository", type=Path, required=True)
    create.add_argument("--commit", required=True)
    create.add_argument("--output", type=Path, required=True)
    check = subparsers.add_parser("verify")
    check.add_argument("--root", type=Path, required=True)
    check.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "materialize":
        manifest_path = materialize(args.repository, args.commit, args.output)
        print(manifest_path)
    else:
        print(json.dumps(verify(args.root.resolve(strict=True), args.manifest.resolve(strict=True)), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
