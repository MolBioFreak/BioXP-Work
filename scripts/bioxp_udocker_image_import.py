#!/usr/bin/env python3
"""Stage, verify, and transactionally publish an immutable image import."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import shutil
import stat
import tempfile
from pathlib import Path
from types import ModuleType


def _inspector() -> ModuleType:
    path = Path(__file__).with_name("bioxp_udocker_image_inspector.py")
    spec = importlib.util.spec_from_file_location("_bioxp_release_inspector", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load repository image inspector")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _need(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def validate_authority_tree(root: Path, required_uid: int) -> None:
    _need(root.exists() and root.is_dir() and not root.is_symlink(), "image bundle is absent or symlinked")
    for path in (root, *root.rglob("*")):
        info = path.lstat()
        _need(not stat.S_ISLNK(info.st_mode), f"authority symlink is forbidden: {path}")
        _need(info.st_uid == required_uid, f"authority has wrong owner: {path}")
        _need(info.st_mode & 0o222 == 0, f"authority is writable: {path}")
        _need(stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode), f"unsupported authority file type: {path}")


def tree_digest(root: Path) -> str:
    digest = hashlib.sha256()
    if not root.exists():
        return digest.hexdigest()
    for path in sorted((root, *root.rglob("*")), key=lambda item: item.relative_to(root).as_posix().encode("utf-8") if item != root else b""):
        info = path.lstat()
        relative = "." if path == root else path.relative_to(root).as_posix()
        if stat.S_ISLNK(info.st_mode):
            raise RuntimeError(f"store symlink is forbidden: {relative}")
        if stat.S_ISDIR(info.st_mode):
            kind, identity = "directory", ""
        elif stat.S_ISREG(info.st_mode):
            kind, identity = "file", hashlib.sha256(path.read_bytes()).hexdigest()
        else:
            raise RuntimeError(f"unsupported store file type: {relative}")
        digest.update(relative.encode() + b"\0" + kind.encode() + b"\0" + str(info.st_size).encode() + b"\0" + identity.encode() + b"\n")
    return digest.hexdigest()


def _remove(path: Path) -> None:
    if not path.exists():
        return
    if path.is_dir() and not path.is_symlink():
        for directory in (path, *[item for item in path.rglob("*") if item.is_dir() and not item.is_symlink()]):
            os.chmod(directory, directory.stat().st_mode | 0o700)
        shutil.rmtree(path)
    else:
        path.unlink()


def _make_staging_writable(root: Path) -> None:
    for path in (root, *[item for item in root.rglob("*") if item.is_dir() and not item.is_symlink()]):
        os.chmod(path, 0o755)


def _seal_tree(root: Path) -> None:
    for path in sorted(root.rglob("*"), reverse=True):
        if path.is_dir() and not path.is_symlink():
            os.chmod(path, 0o555)
        elif path.is_file() and not path.is_symlink():
            relative = path.relative_to(root)
            os.chmod(path, 0o555 if relative.parts and relative.parts[0] == "bin" else 0o444)
    os.chmod(root, 0o555)


def import_image(
    bundle: Path,
    store: Path,
    image_id: str,
    source_manifest: Path,
    receipt_output: Path,
    inspected_at: str,
    verifier_source: Path,
    required_uid: int = 0,
    fault: str | None = None,
):
    inspector = _inspector()
    bundle = Path(bundle)
    store = Path(store)
    receipt_output = Path(receipt_output)
    validate_authority_tree(bundle, required_uid)
    _need(inspector.IMAGE_ID_RE.fullmatch(image_id) is not None, "image ID must be a full immutable sha256:<64-hex>")
    parent = store.parent.resolve(strict=True)
    receipt_parent = receipt_output.parent.resolve(strict=True)
    _need(parent.stat().st_dev == receipt_parent.stat().st_dev, "store and receipt must be published on the same filesystem")

    transaction = Path(tempfile.mkdtemp(prefix=".bioxp-import-", dir=parent))
    staged_store = transaction / "store"
    staged_receipt = transaction / "image-inspection.json"
    store_backup = transaction / "old-store"
    receipt_backup = transaction / "old-receipt"
    old_store = store.exists()
    old_receipt = receipt_output.exists()
    runtime_identity: tuple[int, int, int] | None = None
    published_runtime_root: Path | None = None
    try:
        if old_store:
            _need(store.is_dir() and not store.is_symlink(), "existing store is not a safe directory")
            store_root = store.resolve(strict=True)
            runtime_root = store / "containers"
            _need(not runtime_root.is_symlink(), "existing container runtime root is symlinked")
            if runtime_root.exists():
                _need(runtime_root.is_dir(), "existing container runtime root is not a directory")
                runtime_info = runtime_root.stat()
                runtime_identity = (
                    runtime_info.st_uid,
                    runtime_info.st_gid,
                    stat.S_IMODE(runtime_info.st_mode),
                )

            def ignore_mutable_runtime(source: str, names: list[str]) -> set[str]:
                return {"containers"} if Path(source).resolve(strict=True) == store_root and "containers" in names else set()

            shutil.copytree(store, staged_store, ignore=ignore_mutable_runtime)
            _make_staging_writable(staged_store)
        else:
            staged_store.mkdir()
        actual_udocker_bundle = (bundle / "repos").is_dir() and (bundle / "layers").is_dir()
        if actual_udocker_bundle:
            for config_path in list(staged_store.glob("repos/*/*/container.json")):
                if config_path.is_file() and hashlib.sha256(config_path.read_bytes()).hexdigest() == image_id[7:]:
                    shutil.rmtree(config_path.parent)
            (staged_store / "layers").mkdir(parents=True, exist_ok=True)
            for layer in sorted((bundle / "layers").glob("*.layer")):
                target = staged_store / "layers" / layer.name
                if target.exists():
                    _need(hashlib.sha256(target.read_bytes()).hexdigest() == layer.stem, f"existing layer digest mismatch: {layer.name}")
                else:
                    shutil.copy2(layer, target)
            for repository in sorted((bundle / "repos").iterdir()):
                _need(repository.is_dir() and not repository.is_symlink(), "bundle repository authority is invalid")
                target = staged_store / "repos" / repository.name
                if target.exists():
                    shutil.copytree(repository, target, dirs_exist_ok=True)
                else:
                    shutil.copytree(repository, target)
        else:
            destination = staged_store / "images" / "sha256" / image_id[7:]
            if destination.exists():
                shutil.rmtree(destination)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(bundle, destination)
        if runtime_identity is not None:
            published_runtime_root = staged_store / "containers"
            published_runtime_root.mkdir(mode=0o700)
        _seal_tree(staged_store)
        if runtime_identity is not None:
            assert published_runtime_root is not None
            os.chown(published_runtime_root, runtime_identity[0], runtime_identity[1])
            os.chmod(published_runtime_root, runtime_identity[2])
        receipt = inspector.inspect_image(staged_store, image_id, source_manifest, inspected_at, verifier_source)
        staged_receipt.write_bytes(inspector.canonical_bytes(receipt))
        os.chmod(staged_receipt, 0o444)
        inspector.verify_receipt(receipt, staged_store, source_manifest, image_id, verifier_source)
        os.chmod(staged_store, 0o755)
        if fault == "before_publish":
            raise RuntimeError("injected import fault before publication")

        if old_store:
            os.replace(store, store_backup)
        if old_receipt:
            os.replace(receipt_output, receipt_backup)
        try:
            if fault == "after_old_rename":
                raise RuntimeError("injected import fault after old authority rename")
            os.replace(staged_store, store)
            os.chmod(store, 0o555)
            os.replace(staged_receipt, receipt_output)
            if fault == "after_publish":
                raise RuntimeError("injected import fault after publication")
        except BaseException:
            _remove(store)
            _remove(receipt_output)
            if old_store:
                os.replace(store_backup, store)
            if old_receipt:
                os.replace(receipt_backup, receipt_output)
            raise
        return receipt
    finally:
        _remove(transaction)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--store", type=Path, required=True)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--source-manifest", type=Path, required=True)
    parser.add_argument("--receipt-output", type=Path, required=True)
    parser.add_argument("--inspected-at", required=True)
    parser.add_argument("--verifier-source", type=Path, default=Path(__file__).with_name("bioxp_udocker_image_inspector.py"))
    parser.add_argument("--required-uid", type=int, default=0)
    args = parser.parse_args()
    import_image(
        args.bundle, args.store, args.image_id, args.source_manifest,
        args.receipt_output, args.inspected_at, args.verifier_source, args.required_uid,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
