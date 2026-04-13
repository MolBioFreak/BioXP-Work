from __future__ import annotations

import json
import os
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

BIOXP_VALIDATION_ARTIFACT_ROOT_ENV = "BIOXP_VALIDATION_ARTIFACT_ROOT"
DEFAULT_VALIDATION_ARTIFACT_ROOT = Path("/mnt/BioModStack/bms_results/bioxp_validation")
MOTION_VALIDATION_SCHEMA_VERSION = "bioxp.motion_validation_bundle/v1"


def get_validation_artifact_root() -> Path:
    raw = str(os.environ.get(BIOXP_VALIDATION_ARTIFACT_ROOT_ENV, "")).strip()
    return Path(raw).expanduser() if raw else DEFAULT_VALIDATION_ARTIFACT_ROOT


def _slug_token(value: Any, *, fallback: str) -> str:
    text = "" if value is None else str(value).strip().lower()
    if not text:
        return fallback
    out = []
    last_dash = False
    for char in text:
        keep = char.isalnum()
        if keep:
            out.append(char)
            last_dash = False
            continue
        if not last_dash:
            out.append("-")
            last_dash = True
    slug = "".join(out).strip("-")
    return slug or fallback


def _json_default(value: Any):
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=_json_default) + "\n", encoding="utf-8")


def _request_summary(command: str, axis: str, request_payload: dict[str, Any]) -> dict[str, Any]:
    return {
        "command": command,
        "axis": axis,
        "steps": request_payload.get("steps"),
        "position_steps": request_payload.get("position_steps"),
        "wait_timeout_s": request_payload.get("wait_timeout_s"),
        "timeout_s": request_payload.get("timeout_s"),
        "speed": request_payload.get("speed"),
        "capture_bundle": bool(request_payload.get("capture_bundle")),
        "dry_run_bundle": bool(request_payload.get("dry_run_bundle")),
    }


def _response_summary(response_payload: dict[str, Any], motion_truth: dict[str, Any] | None) -> dict[str, Any]:
    return {
        "position_delta": response_payload.get("position_delta"),
        "target_position": response_payload.get("target_position"),
        "dry_run": bool(response_payload.get("dry_run")),
        "skipped_hardware_io": bool(response_payload.get("skipped_hardware_io")),
        "truth_evidence_level": (motion_truth or {}).get("evidence_level"),
        "wait_elapsed_ms": (response_payload.get("wait") or {}).get("elapsed_ms"),
        "home_elapsed_ms": (response_payload.get("home") or {}).get("elapsed_ms"),
    }


def create_motion_validation_bundle(
    *,
    command: str,
    axis: str,
    request_payload: dict[str, Any],
    response_payload: dict[str, Any],
    motion_truth: dict[str, Any] | None = None,
    prep_policy: dict[str, Any] | None = None,
    operator_note: str | None = None,
    snapshot_refs: list[str] | None = None,
    dry_run: bool = False,
) -> dict[str, Any]:
    created_at = datetime.now(timezone.utc)
    root = get_validation_artifact_root()
    dated_root = root / created_at.strftime("%Y") / created_at.strftime("%m") / created_at.strftime("%d")
    bundle_id = (
        f"{created_at.strftime('%Y%m%dT%H%M%SZ')}-"
        f"{_slug_token(command, fallback='command')}-"
        f"{_slug_token(axis, fallback='axis')}-"
        f"{uuid.uuid4().hex[:8]}"
    )
    bundle_dir = dated_root / bundle_id
    bundle_dir.mkdir(parents=True, exist_ok=False)

    normalized_operator_note = None
    if operator_note is not None:
        normalized_operator_note = str(operator_note).strip() or None
    normalized_snapshot_refs = [
        str(item).strip()
        for item in (snapshot_refs or [])
        if str(item).strip()
    ]

    metadata_path = bundle_dir / "metadata.json"
    request_path = bundle_dir / "request.json"
    response_path = bundle_dir / "response.json"

    bundle_summary = {
        "schema_version": MOTION_VALIDATION_SCHEMA_VERSION,
        "bundle_kind": "motion_validation",
        "bundle_id": bundle_id,
        "created_at": created_at.isoformat(),
        "command": command,
        "axis": axis,
        "dry_run": bool(dry_run),
        "root": str(root),
        "bundle_dir": str(bundle_dir),
        "metadata_path": str(metadata_path),
        "request_path": str(request_path),
        "response_path": str(response_path),
        "operator_note": normalized_operator_note,
        "snapshot_refs": normalized_snapshot_refs,
    }

    metadata = {
        **bundle_summary,
        "request_summary": _request_summary(command, axis, request_payload),
        "response_summary": _response_summary(response_payload, motion_truth),
        "prep_policy": prep_policy,
        "motion_truth": motion_truth,
        "files": {
            "metadata": metadata_path.name,
            "request": request_path.name,
            "response": response_path.name,
        },
        "snapshot_ref_count": len(normalized_snapshot_refs),
    }

    _write_json(request_path, request_payload)
    _write_json(response_path, response_payload)
    _write_json(metadata_path, metadata)

    return bundle_summary
