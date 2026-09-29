"""Canonical OEM movement-registry and evidence-authority identity.

The historical standalone lifecycle planner is retired; these helpers remain
shared by the live operator catalog and pipette receipts.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any

from .oem_machine_bundle import (
    OEM_ACQUISITION_ID,
    OEM_LOCK_SHA256,
    OEM_MACHINE_BUNDLE_LOCK_ENV,
)


_REGISTRY_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json"
)


class OemFullLifecycleError(RuntimeError):
    pass


def current_registry_sha256() -> str:
    return hashlib.sha256(_REGISTRY_PATH.read_bytes()).hexdigest()


def current_authority_identity() -> dict[str, Any]:
    """Verify the registry-selected lock bytes before publishing provenance."""
    try:
        registry = json.loads(_REGISTRY_PATH.read_text(encoding="utf-8"))
        authority = registry["authority"]
        lock_path = Path(
            os.environ.get(OEM_MACHINE_BUNDLE_LOCK_ENV, str(authority["evidence_lock_path"]))
        )
        expected = authority["evidence_lock_sha256"]
        lock_bytes = lock_path.read_bytes()
        lock = json.loads(lock_bytes)
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        raise OemFullLifecycleError(f"canonical OEM evidence authority unavailable: {exc}") from exc
    actual = hashlib.sha256(lock_bytes).hexdigest()
    if expected != OEM_LOCK_SHA256 or actual != expected:
        raise OemFullLifecycleError("canonical OEM evidence lock identity mismatch")
    if lock.get("schema_id") != "bioxp.oem_evidence_lock.v4" or lock.get("schema_version") != 4:
        raise OemFullLifecycleError("canonical OEM evidence lock schema mismatch")
    if lock.get("acquisition", {}).get("session_id") != OEM_ACQUISITION_ID:
        raise OemFullLifecycleError("canonical OEM acquisition identity mismatch")
    return {
        "evidence_lock_path": str(lock_path),
        "evidence_lock_sha256": actual,
        "evidence_lock_schema": "bioxp.oem_evidence_lock.v4",
        "acquisition_id": OEM_ACQUISITION_ID,
        "evidence_lock_identity_verified": True,
    }
