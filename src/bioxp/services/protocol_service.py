from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping
from uuid import uuid4

from ..protocols import ProtocolDocument, ProtocolExecutor, ProtocolRuntimeState, StageExecutionStatus, compile_native_protocol, import_oem_xml_protocol

BIOXP_PROTOCOL_JOBS_ROOT_ENV = "BIOXP_PROTOCOL_JOBS_ROOT"
DEFAULT_PROTOCOL_JOBS_ROOT = Path("/mnt/BioModStack/bms_results/bioxp_protocol_jobs")
FALLBACK_PROTOCOL_JOBS_ROOT = Path.home() / ".bioxp" / "protocol_jobs"
PROTOCOL_OPERATOR_BUNDLE_SCHEMA_VERSION = "bioxp.protocol_operator_bundle.v1"


@dataclass(frozen=True)
class CompiledProtocolSource:
    source_type: str
    document: ProtocolDocument
    source_path: str | None = None
    coverage: Mapping[str, Any] | None = None
    experiment: Mapping[str, Any] | None = None
    inventory: Mapping[str, Any] | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            "source_type": self.source_type,
            "source_path": self.source_path,
            "coverage": dict(self.coverage or {}),
            "experiment": dict(self.experiment or {}),
            "inventory": dict(self.inventory or {}),
            "document": self.document.to_payload(),
        }


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _job_status_from_state(state: ProtocolRuntimeState) -> str:
    if state.completed:
        return "completed"
    if state.awaiting_review or state.paused:
        return "awaiting_review"
    return "running"


def _pending_review_payload(state: ProtocolRuntimeState) -> dict[str, Any] | None:
    if not state.awaiting_review:
        return None
    action_id = None
    if state.current_stage_id and state.current_stage_id in state.stage_states:
        action_id = state.stage_states[state.current_stage_id].pause_marker_action_id
    return {
        "stage_id": state.current_stage_id,
        "action_id": action_id,
        "reason": state.pause_reason,
    }


def get_protocol_jobs_root() -> Path:
    configured = os.environ.get(BIOXP_PROTOCOL_JOBS_ROOT_ENV)
    if configured:
        return Path(configured).expanduser().resolve()
    try:
        DEFAULT_PROTOCOL_JOBS_ROOT.mkdir(parents=True, exist_ok=True)
        return DEFAULT_PROTOCOL_JOBS_ROOT
    except OSError:
        FALLBACK_PROTOCOL_JOBS_ROOT.mkdir(parents=True, exist_ok=True)
        return FALLBACK_PROTOCOL_JOBS_ROOT


class ProtocolOperatorBundleStore:
    def __init__(self, root: str | Path | None = None) -> None:
        self.root = Path(root).expanduser().resolve() if root is not None else get_protocol_jobs_root()
        self.root.mkdir(parents=True, exist_ok=True)

    def _job_dir(self, job_id: str) -> Path:
        return self.root / job_id

    def _bundle_path(self, job_id: str) -> Path:
        return self._job_dir(job_id) / "bundle.json"

    def save(self, bundle: Mapping[str, Any]) -> dict[str, Any]:
        job_id = str(bundle["job_id"])
        job_dir = self._job_dir(job_id)
        job_dir.mkdir(parents=True, exist_ok=True)
        payload = dict(bundle)
        payload.setdefault("artifacts", {})
        payload["artifacts"] = {
            **dict(payload.get("artifacts") or {}),
            "job_dir": str(job_dir),
            "bundle_path": str(self._bundle_path(job_id)),
        }
        self._bundle_path(job_id).write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        return payload

    def load(self, job_id: str) -> dict[str, Any]:
        path = self._bundle_path(job_id)
        if not path.exists():
            raise FileNotFoundError(f"Unknown protocol job '{job_id}'")
        return json.loads(path.read_text(encoding="utf-8"))

    def list(self, *, limit: int = 20) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for bundle_path in sorted(self.root.glob("*/bundle.json"), key=lambda value: value.stat().st_mtime, reverse=True):
            try:
                payload = json.loads(bundle_path.read_text(encoding="utf-8"))
            except Exception:
                continue
            rows.append(
                {
                    "job_id": payload.get("job_id"),
                    "status": payload.get("status"),
                    "dry_run": payload.get("execution", {}).get("dry_run"),
                    "protocol_id": payload.get("protocol", {}).get("document", {}).get("protocol_id"),
                    "source_type": payload.get("protocol", {}).get("source_type"),
                    "created_at": payload.get("created_at"),
                    "updated_at": payload.get("updated_at"),
                    "pending_review": payload.get("operator", {}).get("pending_review"),
                }
            )
            if len(rows) >= limit:
                break
        return rows


def compile_protocol_source(payload: Mapping[str, Any]) -> CompiledProtocolSource:
    source_type = str(payload.get("source_type") or ("oem_xml" if payload.get("xml_path") else "native")).strip().lower()
    if source_type == "oem_xml":
        xml_path = payload.get("xml_path") or payload.get("source_path")
        if not xml_path:
            raise ValueError("OEM XML compile requests must include xml_path.")
        imported = import_oem_xml_protocol(str(xml_path))
        return CompiledProtocolSource(
            source_type="oem_xml",
            document=imported.document,
            source_path=imported.source_path,
            coverage=imported.coverage.to_payload(),
            experiment=dict(imported.experiment),
            inventory=dict(imported.inventory),
        )
    document_payload = payload.get("document") if isinstance(payload.get("document"), Mapping) else payload
    document = compile_native_protocol(document_payload)
    return CompiledProtocolSource(
        source_type="native",
        document=document,
        source_path=None,
        coverage={},
        experiment={},
        inventory={},
    )


def _build_operator_bundle(
    *,
    job_id: str,
    compiled: CompiledProtocolSource,
    state: ProtocolRuntimeState,
    dry_run: bool,
    created_at: str,
    reviews: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    updated_at = _utc_now_iso()
    status = _job_status_from_state(state)
    return {
        "schema_version": PROTOCOL_OPERATOR_BUNDLE_SCHEMA_VERSION,
        "job_id": job_id,
        "created_at": created_at,
        "updated_at": updated_at,
        "status": status,
        "protocol": compiled.to_payload(),
        "execution": {
            "dry_run": bool(dry_run),
            "runtime_state": state.to_payload(),
        },
        "operator": {
            "manual_review_required": bool(state.awaiting_review),
            "pending_review": _pending_review_payload(state),
            "reviews": list(reviews or []),
        },
        "artifacts": {},
    }


def create_protocol_job(
    payload: Mapping[str, Any],
    *,
    dry_run: bool = True,
    store: ProtocolOperatorBundleStore | None = None,
) -> dict[str, Any]:
    compiled = compile_protocol_source(payload)
    state = ProtocolExecutor(dry_run=dry_run).execute(compiled.document)
    created_at = _utc_now_iso()
    job_id = f"protocol-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{uuid4().hex[:8]}"
    bundle = _build_operator_bundle(
        job_id=job_id,
        compiled=compiled,
        state=state,
        dry_run=dry_run,
        created_at=created_at,
    )
    active_store = store or ProtocolOperatorBundleStore()
    return active_store.save(bundle)


def get_protocol_job(job_id: str, *, store: ProtocolOperatorBundleStore | None = None) -> dict[str, Any]:
    active_store = store or ProtocolOperatorBundleStore()
    return active_store.load(job_id)


def list_protocol_jobs(*, limit: int = 20, store: ProtocolOperatorBundleStore | None = None) -> list[dict[str, Any]]:
    active_store = store or ProtocolOperatorBundleStore()
    return active_store.list(limit=limit)


def review_protocol_job(
    job_id: str,
    *,
    reviewer: str = "operator",
    note: str | None = None,
    store: ProtocolOperatorBundleStore | None = None,
) -> dict[str, Any]:
    active_store = store or ProtocolOperatorBundleStore()
    bundle = active_store.load(job_id)
    state = ProtocolRuntimeState.from_payload(bundle["execution"]["runtime_state"])
    if not state.awaiting_review:
        raise ValueError(f"Protocol job '{job_id}' is not awaiting review.")

    stage_id = state.current_stage_id
    if stage_id and stage_id in state.stage_states:
        stage_state = state.stage_states[stage_id]
        stage_state.status = StageExecutionStatus.PAUSED
        stage_state.current_action_id = None
        stage_state.pause_marker_action_id = None

    state.paused = False
    state.awaiting_review = False
    state.pause_reason = None
    state.record_event(
        "review_acknowledged",
        stage_id=stage_id,
        detail={"reviewer": reviewer, "note": note},
    )

    compiled = CompiledProtocolSource(
        source_type=str(bundle["protocol"].get("source_type") or "native"),
        document=ProtocolDocument.from_payload(bundle["protocol"]["document"]),
        source_path=bundle["protocol"].get("source_path"),
        coverage=dict(bundle["protocol"].get("coverage") or {}),
        experiment=dict(bundle["protocol"].get("experiment") or {}),
        inventory=dict(bundle["protocol"].get("inventory") or {}),
    )
    resumed_state = ProtocolExecutor(dry_run=bool(bundle["execution"].get("dry_run", True))).execute(
        compiled.document,
        state=state,
    )
    reviews = list(bundle.get("operator", {}).get("reviews") or [])
    reviews.append(
        {
            "reviewed_at": _utc_now_iso(),
            "reviewer": reviewer,
            "note": note,
            "stage_id": stage_id,
        }
    )
    updated_bundle = _build_operator_bundle(
        job_id=job_id,
        compiled=compiled,
        state=resumed_state,
        dry_run=bool(bundle["execution"].get("dry_run", True)),
        created_at=str(bundle.get("created_at") or _utc_now_iso()),
        reviews=reviews,
    )
    return active_store.save(updated_bundle)
