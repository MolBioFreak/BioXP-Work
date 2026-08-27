from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import stat
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
INSPECTOR_PATH = ROOT / "scripts" / "bioxp_udocker_image_inspector.py"
SEALER_PATH = ROOT / "scripts" / "bioxp_release_seal.py"
IMPORTER_PATH = ROOT / "scripts" / "bioxp_udocker_image_import.py"
BOOTSTRAP_PATH = ROOT / "scripts" / "bootstrap_bioxp_immutable_release.py"


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical(value: object) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode()


def source_packet(tmp_path: Path) -> tuple[Path, Path, dict]:
    source = tmp_path / "source"
    source.mkdir()
    payload = source / "payload.txt"
    payload.write_bytes(b"exact source bytes\n")
    digest = hashlib.sha256(payload.read_bytes()).hexdigest()
    aggregate = hashlib.sha256(
        b"payload.txt\0" + str(payload.stat().st_size).encode() + b"\0" + digest.encode() + b"\n"
    ).hexdigest()
    manifest = {
        "schema": "bioxp.release.source_manifest.v1",
        "status": "verified",
        "commit": "1" * 40,
        "tree": "2" * 40,
        "root": "/app",
        "inventory_algorithm": "sorted-utf8-path-size-sha256-v1",
        "aggregate_algorithm": "sha256(path_utf8+nul+decimal_size+nul+file_sha256+lf)",
        "file_count": 1,
        "total_size": payload.stat().st_size,
        "aggregate_sha256": aggregate,
        "files": [{"path": "payload.txt", "size": payload.stat().st_size, "sha256": digest}],
    }
    packet = source / ".bioxp-release"
    packet.mkdir()
    manifest_path = packet / "source-manifest.json"
    manifest_path.write_bytes(canonical(manifest))
    return source, manifest_path, manifest


def image_bundle(tmp_path: Path, manifest_path: Path, manifest: dict) -> tuple[Path, str]:
    bundle = tmp_path / "image-bundle"
    embedded = bundle / "rootfs" / "usr" / "share" / "bioxp-release" / "source-manifest.json"
    embedded.parent.mkdir(parents=True)
    embedded.write_bytes(manifest_path.read_bytes())
    app = bundle / "rootfs" / "app"
    app.mkdir(parents=True)
    source_root = manifest_path.parents[1]
    for row in manifest["files"]:
        destination = app / row["path"]
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((source_root / row["path"]).read_bytes())
    config = {
        "config": {
            "Labels": {
                "org.opencontainers.image.revision": manifest["commit"],
                "com.bioxp.source.tree": manifest["tree"],
                "com.bioxp.source.manifest.sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
            }
        }
    }
    config_raw = canonical(config)
    (bundle / "oci-config.json").write_bytes(config_raw)
    for path in sorted(bundle.rglob("*"), reverse=True):
        os.chmod(path, 0o555 if path.is_dir() else 0o444)
    os.chmod(bundle, 0o555)
    return bundle, "sha256:" + hashlib.sha256(config_raw).hexdigest()


def imported_store(tmp_path: Path, bundle: Path, image_id: str) -> Path:
    store = tmp_path / "store"
    image = store / "images" / "sha256" / image_id.removeprefix("sha256:")
    image.parent.mkdir(parents=True)
    import shutil

    shutil.copytree(bundle, image)
    return store


def test_inspector_requires_full_config_digest_and_exact_embedded_authority(tmp_path: Path):
    inspector = load("bioxp_udocker_image_inspector", INSPECTOR_PATH)
    _, manifest_path, manifest = source_packet(tmp_path)
    bundle, image_id = image_bundle(tmp_path, manifest_path, manifest)
    store = imported_store(tmp_path, bundle, image_id)

    receipt = inspector.inspect_image(
        store=store,
        image_id=image_id,
        source_manifest=manifest_path,
        inspected_at="2026-08-27T12:00:00Z",
        verifier_source=INSPECTOR_PATH,
    )

    assert receipt["requested_image_id"] == image_id == receipt["resolved_local_image_id"]
    assert receipt["oci_labels"]["org.opencontainers.image.revision"] == manifest["commit"]
    assert receipt["embedded_source_manifest"]["sha256"] == hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    assert receipt["embedded_source_manifest"]["aggregate_sha256"] == manifest["aggregate_sha256"]
    assert receipt["verifier"]["sha256"] == hashlib.sha256(INSPECTOR_PATH.read_bytes()).hexdigest()
    assert inspector.verify_receipt(receipt, store, manifest_path, image_id, INSPECTOR_PATH) == receipt

    with pytest.raises(RuntimeError, match="full immutable"):
        inspector.inspect_image(store, image_id[:20], manifest_path, "2026-08-27T12:00:00Z", INSPECTOR_PATH)
    config_path = store / "images" / "sha256" / image_id[7:] / "oci-config.json"
    os.chmod(config_path, 0o644)
    config_path.write_bytes(b"{}\n")
    with pytest.raises(RuntimeError, match="config digest"):
        inspector.verify_receipt(receipt, store, manifest_path, image_id, INSPECTOR_PATH)


def test_importer_stages_and_publishes_nothing_on_failed_inspection(tmp_path: Path):
    importer = load("bioxp_udocker_image_import", IMPORTER_PATH)
    _, manifest_path, manifest = source_packet(tmp_path)
    bundle, image_id = image_bundle(tmp_path, manifest_path, manifest)
    store = tmp_path / "store"
    store.mkdir()
    (store / "old-authority").write_bytes(b"old\n")
    before = importer.tree_digest(store)
    receipt = tmp_path / "image-inspection.json"
    embedded = bundle / "rootfs" / "usr" / "share" / "bioxp-release" / "source-manifest.json"
    os.chmod(embedded, 0o644)
    embedded.write_bytes(embedded.read_bytes() + b"tampered")
    os.chmod(embedded, 0o444)

    with pytest.raises(RuntimeError, match="embedded source manifest"):
        importer.import_image(
            bundle=bundle,
            store=store,
            image_id=image_id,
            source_manifest=manifest_path,
            receipt_output=receipt,
            inspected_at="2026-08-27T12:00:00Z",
            verifier_source=INSPECTOR_PATH,
            required_uid=os.getuid(),
        )

    assert importer.tree_digest(store) == before
    assert (store / "old-authority").read_bytes() == b"old\n"
    assert not receipt.exists()


def test_importer_publishes_verified_full_id_and_receipt_together(tmp_path: Path):
    importer = load("bioxp_udocker_image_import_ok", IMPORTER_PATH)
    inspector = load("bioxp_udocker_image_inspector_ok", INSPECTOR_PATH)
    _, manifest_path, manifest = source_packet(tmp_path)
    bundle, image_id = image_bundle(tmp_path, manifest_path, manifest)
    store = tmp_path / "store"
    receipt_path = tmp_path / "image-inspection.json"

    receipt = importer.import_image(
        bundle, store, image_id, manifest_path, receipt_path,
        "2026-08-27T12:00:00Z", INSPECTOR_PATH, os.getuid(),
    )

    assert receipt_path.read_bytes() == canonical(receipt)
    assert inspector.verify_receipt(receipt, store, manifest_path, image_id, INSPECTOR_PATH) == receipt
    assert not list(tmp_path.glob(".bioxp-import-*"))


def test_sealer_recomputes_every_digest_and_is_canonical(tmp_path: Path):
    inspector = load("bioxp_udocker_image_inspector_seal", INSPECTOR_PATH)
    sealer = load("bioxp_release_seal", SEALER_PATH)
    source, manifest_path, manifest = source_packet(tmp_path)
    bundle, image_id = image_bundle(tmp_path, manifest_path, manifest)
    store = imported_store(tmp_path, bundle, image_id)
    inspection = inspector.inspect_image(store, image_id, manifest_path, "2026-08-27T12:00:00Z", INSPECTOR_PATH)
    inspection_path = tmp_path / "inspection.json"
    inspection_path.write_bytes(canonical(inspection))
    unit = tmp_path / "bioxp-api.service"
    launcher = tmp_path / "launcher"
    lock = tmp_path / "OEM_EVIDENCE_LOCK.json"
    unit.write_bytes(b"unit\n")
    launcher.write_bytes(b"launcher\n")
    lock.write_bytes(b"lock\n")

    kwargs = dict(
        source_root=source,
        source_manifest=manifest_path,
        image_inspection=inspection_path,
        image_id=image_id,
        inspector_source=INSPECTOR_PATH,
        unit=unit,
        launcher=launcher,
        oem_lock=lock,
        release_id="release-001",
        deployment_receipt_id="deployment-001",
        installed_at="2026-08-27T12:05:00Z",
    )
    first = sealer.seal_release(**kwargs)
    second = sealer.seal_release(**kwargs)

    assert canonical(first) == canonical(second)
    assert first["source"]["manifest_sha256"] == hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    assert first["source"]["aggregate_sha256"] == manifest["aggregate_sha256"]
    assert first["image"]["inspection_receipt_sha256"] == hashlib.sha256(inspection_path.read_bytes()).hexdigest()
    assert first["binding"]["unit_sha256"] == hashlib.sha256(unit.read_bytes()).hexdigest()
    assert first["binding"]["launcher_sha256"] == hashlib.sha256(launcher.read_bytes()).hexdigest()
    assert first["binding"]["oem_lock_sha256"] == hashlib.sha256(lock.read_bytes()).hexdigest()

    manifest["aggregate_sha256"] = "0" * 64
    manifest_path.write_bytes(canonical(manifest))
    with pytest.raises(RuntimeError, match="aggregate"):
        sealer.seal_release(**kwargs)


def make_candidate(tmp_path: Path, name: str, value: bytes) -> Path:
    candidate = tmp_path / name
    (candidate / "identity").mkdir(parents=True)
    (candidate / "identity" / "release-identity.json").write_bytes(value)
    os.chmod(candidate / "identity" / "release-identity.json", 0o444)
    os.chmod(candidate / "identity", 0o555)
    os.chmod(candidate, 0o555)
    return candidate


def test_bootstrap_fault_rollback_preserves_complete_old_or_new_packet(tmp_path: Path):
    bootstrap = load("bootstrap_bioxp_immutable_release", BOOTSTRAP_PATH)
    publication = tmp_path / "publication"
    publication.mkdir()
    old = make_candidate(tmp_path, "old", b"old-complete\n")
    bootstrap.publish_packet(old, publication, required_uid=os.getuid())
    assert (publication / "current" / "identity" / "release-identity.json").read_bytes() == b"old-complete\n"

    new = make_candidate(tmp_path, "new", b"new-complete\n")
    with pytest.raises(RuntimeError, match="injected"):
        bootstrap.publish_packet(new, publication, fault="after_old_rename", required_uid=os.getuid())
    assert (publication / "current" / "identity" / "release-identity.json").read_bytes() == b"old-complete\n"
    assert not list(publication.glob(".bioxp-publish-*"))

    bootstrap.publish_packet(new, publication, required_uid=os.getuid())
    assert (publication / "current" / "identity" / "release-identity.json").read_bytes() == b"new-complete\n"


def test_bootstrap_rejects_symlink_writable_and_wrong_owner_authority(tmp_path: Path):
    bootstrap = load("bootstrap_bioxp_immutable_release_reject", BOOTSTRAP_PATH)
    publication = tmp_path / "publication"
    publication.mkdir()

    writable = make_candidate(tmp_path, "writable", b"bytes\n")
    os.chmod(writable / "identity" / "release-identity.json", 0o644)
    with pytest.raises(RuntimeError, match="writable"):
        bootstrap.publish_packet(writable, publication, required_uid=os.getuid())

    linked = make_candidate(tmp_path, "linked", b"bytes\n")
    os.chmod(linked / "identity", 0o755)
    (linked / "identity" / "escape").symlink_to(tmp_path / "outside")
    os.chmod(linked / "identity", 0o555)
    with pytest.raises(RuntimeError, match="symlink"):
        bootstrap.publish_packet(linked, publication, required_uid=os.getuid())

    owned = make_candidate(tmp_path, "owned", b"bytes\n")
    with pytest.raises(RuntimeError, match="owner"):
        bootstrap.publish_packet(owned, publication, required_uid=os.getuid() + 1)
