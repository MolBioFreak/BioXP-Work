from __future__ import annotations

import json
from pathlib import Path


_REGISTRY_PATH = (
    Path(__file__).resolve().parents[1]
    / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json"
)


def bind_serial206_oem_snapshot(monkeypatch):
    """Bind the exact hash-verified serial-206 bundle for read-only tests."""
    from bioxp import oem_machine_bundle

    registry = json.loads(_REGISTRY_PATH.read_text(encoding="utf-8"))
    canonical_lock = Path(registry["authority"]["evidence_lock_path"])
    bundle_lock = (
        canonical_lock.parent
        / "oem_machine_runtime_bundle_serial206"
        / "OEM_EVIDENCE_LOCK.json"
    )
    if not bundle_lock.is_file():
        raise AssertionError(f"immutable serial-206 OEM test bundle is unavailable: {bundle_lock}")

    snapshot = oem_machine_bundle.load_oem_machine_snapshot(
        bundle_lock,
        require_operator_label=False,
    )
    assert snapshot.machine_serial == oem_machine_bundle.OEM_MACHINE_SERIAL == 206
    assert snapshot.lock_sha256 == registry["authority"]["evidence_lock_sha256"]
    assert snapshot.mutation_authorized is False
    monkeypatch.setattr(oem_machine_bundle, "_active_snapshot", snapshot)
    return snapshot
