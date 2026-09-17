#!/usr/bin/env python3
"""Publish one complete immutable BioXP release packet transactionally."""
from __future__ import annotations

import argparse
import os
import shutil
import stat
import tempfile
from pathlib import Path


def _need(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def validate_immutable_tree(root: Path, required_uid: int) -> None:
    _need(root.exists() and root.is_dir() and not root.is_symlink(), "release packet is absent or symlinked")
    entries = (root, *root.rglob("*"))
    _need(len(entries) > 1, "release packet is empty")
    for path in entries:
        info = path.lstat()
        _need(not stat.S_ISLNK(info.st_mode), f"release authority symlink is forbidden: {path}")
        _need(info.st_uid == required_uid, f"release authority has wrong owner: {path}")
        _need(info.st_mode & 0o222 == 0, f"release authority is writable: {path}")
        _need(stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode), f"unsupported release authority file type: {path}")


def _remove(path: Path) -> None:
    if not path.exists():
        return
    if path.is_dir() and not path.is_symlink():
        for directory in (path, *[item for item in path.rglob("*") if item.is_dir() and not item.is_symlink()]):
            os.chmod(directory, directory.stat().st_mode | 0o700)
        shutil.rmtree(path)
    else:
        path.unlink()


def publish_packet(
    candidate: Path,
    publication_root: Path,
    *,
    fault: str | None = None,
    required_uid: int = 0,
) -> Path:
    """Copy-verify then swap a whole packet, restoring the old packet on error."""
    candidate = Path(candidate)
    publication_root = Path(publication_root)
    validate_immutable_tree(candidate, required_uid)
    _need(publication_root.exists() and publication_root.is_dir() and not publication_root.is_symlink(), "publication root is absent or symlinked")
    staged = Path(tempfile.mkdtemp(prefix=".bioxp-publish-new-", dir=publication_root))
    staged.rmdir()
    backup = Path(tempfile.mkdtemp(prefix=".bioxp-publish-old-", dir=publication_root))
    backup.rmdir()
    _need(staged.parent.stat().st_dev == publication_root.stat().st_dev, "publication staging is not on the destination filesystem")
    current = publication_root / "current"
    had_current = current.exists()
    try:
        shutil.copytree(candidate, staged)
        validate_immutable_tree(staged, required_uid)
        if fault == "before_publish":
            raise RuntimeError("injected publication fault before swap")
        if had_current:
            _need(current.is_dir() and not current.is_symlink(), "current release packet is unsafe")
            validate_immutable_tree(current, required_uid)
            os.replace(current, backup)
        try:
            if fault == "after_old_rename":
                raise RuntimeError("injected publication fault after old authority rename")
            os.replace(staged, current)
            validate_immutable_tree(current, required_uid)
            if fault == "after_new_rename":
                raise RuntimeError("injected publication fault after new authority rename")
        except BaseException:
            _remove(current)
            if had_current:
                os.replace(backup, current)
            raise
        _remove(backup)
        return current
    finally:
        _remove(staged)
        _remove(backup)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--publication-root", type=Path, required=True)
    parser.add_argument("--required-uid", type=int, default=0)
    parser.add_argument("--fault", choices=("before_publish", "after_old_rename", "after_new_rename"))
    args = parser.parse_args()
    print(publish_packet(args.candidate, args.publication_root, fault=args.fault, required_uid=args.required_uid))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
