from __future__ import annotations

import fcntl
import hashlib
import json
import os
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping
from uuid import uuid4

from ..protocols import ProtocolAction, ProtocolDocument, ProtocolExecutor, ProtocolRuntimeState, StageExecutionStatus, compile_native_protocol, import_oem_xml_protocol
from ..protocols.models import ProtocolActionKind, normalize_action_kind

BIOXP_PROTOCOL_JOBS_ROOT_ENV = "BIOXP_PROTOCOL_JOBS_ROOT"
DEFAULT_PROTOCOL_JOBS_ROOT = Path("/mnt/BioModStack/bms_results/bioxp_protocol_jobs")
FALLBACK_PROTOCOL_JOBS_ROOT = Path.home() / ".bioxp" / "protocol_jobs"
PROTOCOL_OPERATOR_BUNDLE_SCHEMA_VERSION = "bioxp.protocol_operator_bundle.v1"
PROTOCOL_LIVE_CONTRACT_SCHEMA_VERSION = "bioxp.protocol_live_execution_contract.v1"
PROTOCOL_LIVE_RESERVATION_SCHEMA_VERSION = "bioxp.protocol_live_idempotency_reservation.v1"
LIVE_REFERENCE_REQUIRED_AXES = ("x", "y", "z")
REFERENCE_REQUIRED_ACTION_KINDS = {
    ProtocolActionKind.MOVE,
    ProtocolActionKind.PIPETTE_TIP,
    ProtocolActionKind.PIPETTE_ASPIRATE,
    ProtocolActionKind.PIPETTE_DISPENSE,
    ProtocolActionKind.PIPETTE_MIX,
    ProtocolActionKind.INSPECT,
    ProtocolActionKind.BARCODE_READ,
    ProtocolActionKind.PLATE_PREPARE,
    ProtocolActionKind.PLATE_MOVE,
    ProtocolActionKind.MOVE_COVER,
    ProtocolActionKind.SEAL_SEPARATE,
    ProtocolActionKind.LIQUID_ADJUST,
    ProtocolActionKind.TIP_EJECT,
}
ActionHandler = Callable[[ProtocolAction, ProtocolRuntimeState], Mapping[str, Any] | None]


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


class ProtocolLiveContractError(ValueError):
    def __init__(self, message: str, *, details: Mapping[str, Any] | None = None) -> None:
        super().__init__(message)
        self.details = dict(details or {})

    def to_payload(self) -> dict[str, Any]:
        return {
            "error": "live_protocol_contract_failed",
            "message": str(self),
            **self.details,
        }


def _as_mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _clean_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _unique_text_list(*values: Any) -> list[str]:
    rows: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value is None:
            continue
        if isinstance(value, str):
            candidates = [value]
        elif isinstance(value, Mapping):
            candidates = value.values()
        else:
            try:
                candidates = list(value)
            except TypeError:
                candidates = [value]
        for candidate in candidates:
            text = _clean_text(candidate)
            if text and text not in seen:
                seen.add(text)
                rows.append(text)
    return rows


def _iter_document_actions(document: ProtocolDocument) -> list[ProtocolAction]:
    return [action for stage in document.stages for action in stage.actions]


def _normalized_handler_kinds(handlers: Mapping[ProtocolActionKind | str, ActionHandler] | None) -> set[ProtocolActionKind]:
    kinds: set[ProtocolActionKind] = set()
    for raw_kind in (handlers or {}).keys():
        kinds.add(normalize_action_kind(raw_kind))
    return kinds


def _reference_axes_from_snapshot(snapshot: Mapping[str, Any]) -> list[str]:
    rows = snapshot.get("rows", {}) if isinstance(snapshot, Mapping) else {}
    verified: list[str] = []
    for axis in LIVE_REFERENCE_REQUIRED_AXES:
        row = rows.get(axis, {}) if isinstance(rows, Mapping) else {}
        if isinstance(row, Mapping) and row.get("state") == "referenced":
            verified.append(axis)
    return verified


def _build_live_execution_contract(
    *,
    payload: Mapping[str, Any],
    compiled: CompiledProtocolSource,
    handlers: Mapping[ProtocolActionKind | str, ActionHandler] | None,
) -> dict[str, Any]:
    live_payload = _as_mapping(payload.get("live_execution") or payload.get("live_contract"))

    def get_value(name: str, default: Any = None) -> Any:
        if name in live_payload:
            return live_payload[name]
        return payload.get(name, default)

    preflight = _as_mapping(get_value("preflight"))
    deck_manifest = _as_mapping(get_value("deck_manifest"))
    reference_snapshot = _as_mapping(preflight.get("reference_snapshot") or get_value("reference_snapshot"))
    artifact_refs = _unique_text_list(
        get_value("artifact_refs"),
        get_value("snapshot_refs"),
        preflight.get("artifact_refs"),
        preflight.get("snapshot_refs"),
    )
    operator_id = _clean_text(get_value("operator_id") or get_value("operator") or get_value("reviewer"))
    live_ack = bool(
        get_value("live_execution_ack")
        or get_value("operator_ack")
        or get_value("operator_acknowledged_risk")
    )
    physical_console_verified = bool(
        get_value("physical_console_verified")
        or preflight.get("physical_console_verified")
        or preflight.get("operator_console_verified")
    )

    actions = _iter_document_actions(compiled.document)
    hardware_action_kinds = sorted(
        {
            action.kind.value
            for action in actions
            if action.required_capability is not None
        }
    )
    reference_required_action_kinds = sorted(
        {
            action.kind.value
            for action in actions
            if action.kind in REFERENCE_REQUIRED_ACTION_KINDS
        }
    )
    handler_kinds = _normalized_handler_kinds(handlers)
    missing_live_handlers = sorted(
        {
            action.kind.value
            for action in actions
            if action.required_capability is not None and action.kind not in handler_kinds
        }
    )
    reference_axes_verified = _reference_axes_from_snapshot(reference_snapshot)
    missing_reference_axes = [
        axis
        for axis in LIVE_REFERENCE_REQUIRED_AXES
        if reference_required_action_kinds and axis not in reference_axes_verified
    ]

    missing_contract_fields: list[str] = []
    if not live_ack:
        missing_contract_fields.append("live_execution_ack")
    if not operator_id:
        missing_contract_fields.append("operator_id")
    if not physical_console_verified:
        missing_contract_fields.append("physical_console_verified")
    if not deck_manifest:
        missing_contract_fields.append("deck_manifest")
    if not artifact_refs:
        missing_contract_fields.append("preflight.artifact_refs")
    if reference_required_action_kinds and not reference_snapshot:
        missing_contract_fields.append("preflight.reference_snapshot")

    if missing_contract_fields or missing_reference_axes or missing_live_handlers:
        raise ProtocolLiveContractError(
            "Live protocol execution requires an explicit operator contract, verified preflight, artifacts, and registered hardware handlers.",
            details={
                "missing_contract_fields": missing_contract_fields,
                "missing_reference_axes": missing_reference_axes,
                "missing_live_handlers": missing_live_handlers,
                "hardware_action_kinds": hardware_action_kinds,
                "reference_required_action_kinds": reference_required_action_kinds,
                "required_reference_axes": list(LIVE_REFERENCE_REQUIRED_AXES),
            },
        )

    return {
        "schema_version": PROTOCOL_LIVE_CONTRACT_SCHEMA_VERSION,
        "mode": "live",
        "created_at": _utc_now_iso(),
        "operator_id": operator_id,
        "live_execution_ack": True,
        "physical_console_verified": True,
        "protocol_id": compiled.document.protocol_id,
        "source_type": compiled.source_type,
        "action_count": len(actions),
        "hardware_action_kinds": hardware_action_kinds,
        "reference_required_action_kinds": reference_required_action_kinds,
        "deck_manifest": deck_manifest,
        "preflight": {
            "reference_snapshot": reference_snapshot,
            "reference_axes_verified": reference_axes_verified,
            "required_reference_axes": list(LIVE_REFERENCE_REQUIRED_AXES) if reference_required_action_kinds else [],
            "artifact_refs": artifact_refs,
        },
        "artifacts": {
            "required": True,
            "refs": artifact_refs,
            "preflight_artifact_name": "preflight.json",
        },
    }


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _job_status_from_state(state: ProtocolRuntimeState) -> str:
    if state.completed:
        return "completed"
    if any(stage.status is StageExecutionStatus.FAILED for stage in state.stage_states.values()):
        return "failed"
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

    def _reservation_path(self, job_id: str) -> Path:
        return self._job_dir(job_id) / "idempotency-reservation.json"

    @contextmanager
    def live_creation_lock(self, job_id: str):
        job_dir = self._job_dir(job_id)
        job_dir.mkdir(parents=True, exist_ok=True)
        root_fd = os.open(self.root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(root_fd)
        finally:
            os.close(root_fd)
        lock_path = job_dir / ".idempotency.lock"
        with lock_path.open("a+b") as lock_file:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)

    def load_live_reservation(self, job_id: str) -> dict[str, Any] | None:
        path = self._reservation_path(job_id)
        if not path.exists():
            return None
        try:
            reservation = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise ProtocolLiveContractError(
                "The live protocol idempotency reservation is unreadable; operator recovery is required.",
                details={"idempotency_recovery_required": True, "job_id": job_id},
            ) from exc
        if not isinstance(reservation, dict):
            raise ProtocolLiveContractError(
                "The live protocol idempotency reservation is invalid; operator recovery is required.",
                details={"idempotency_recovery_required": True, "job_id": job_id},
            )
        return reservation

    def save_live_reservation(self, reservation: Mapping[str, Any]) -> dict[str, Any]:
        payload = dict(reservation)
        job_id = str(payload["job_id"])
        path = self._reservation_path(job_id)
        temporary_path = path.with_name(f".{path.name}.{os.getpid()}.{uuid4().hex}.tmp")
        try:
            with temporary_path.open("w", encoding="utf-8") as temporary_file:
                json.dump(payload, temporary_file, indent=2, sort_keys=True)
                temporary_file.flush()
                os.fsync(temporary_file.fileno())
            os.replace(temporary_path, path)
            directory_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            temporary_path.unlink(missing_ok=True)
        return payload

    def save(self, bundle: Mapping[str, Any]) -> dict[str, Any]:
        job_id = str(bundle["job_id"])
        job_dir = self._job_dir(job_id)
        job_dir.mkdir(parents=True, exist_ok=True)
        payload = dict(bundle)
        payload.setdefault("artifacts", {})
        artifacts = {
            **dict(payload.get("artifacts") or {}),
            "job_dir": str(job_dir),
            "bundle_path": str(self._bundle_path(job_id)),
        }
        live_contract = _as_mapping(_as_mapping(payload.get("execution")).get("live_contract"))
        if live_contract:
            preflight_path = job_dir / "preflight.json"
            artifacts["preflight_path"] = str(preflight_path)
            preflight_path.write_text(
                json.dumps(
                    {
                        "schema_version": live_contract.get("schema_version"),
                        "job_id": job_id,
                        "protocol_id": live_contract.get("protocol_id"),
                        "created_at": live_contract.get("created_at"),
                        "operator_id": live_contract.get("operator_id"),
                        "physical_console_verified": live_contract.get("physical_console_verified"),
                        "deck_manifest": live_contract.get("deck_manifest"),
                        "preflight": live_contract.get("preflight"),
                        "artifact_requirements": {
                            "required": _as_mapping(live_contract.get("artifacts")).get("required"),
                            "preflight_artifact_name": _as_mapping(live_contract.get("artifacts")).get("preflight_artifact_name"),
                        },
                    },
                    indent=2,
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
        payload["artifacts"] = artifacts
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
    nested_document = payload.get("document")
    document_payload = (
        nested_document
        if isinstance(nested_document, Mapping)
        else {key: value for key, value in payload.items() if key != "idempotency_key"}
    )
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
    live_contract: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    updated_at = _utc_now_iso()
    status = _job_status_from_state(state)
    execution = {
        "dry_run": bool(dry_run),
        "runtime_state": state.to_payload(),
    }
    if live_contract is not None:
        execution["live_contract"] = dict(live_contract)
    return {
        "schema_version": PROTOCOL_OPERATOR_BUNDLE_SCHEMA_VERSION,
        "job_id": job_id,
        "created_at": created_at,
        "updated_at": updated_at,
        "status": status,
        "protocol": compiled.to_payload(),
        "execution": execution,
        "operator": {
            "manual_review_required": bool(state.awaiting_review),
            "pending_review": _pending_review_payload(state),
            "reviews": list(reviews or []),
        },
        "artifacts": {},
    }


def _request_fingerprint(protocol: Mapping[str, Any], live_contract: Mapping[str, Any]) -> str:
    fingerprint_contract = dict(live_contract)
    fingerprint_contract.pop("created_at", None)
    return hashlib.sha256(
        json.dumps(
            {"protocol": dict(protocol), "live_contract": fingerprint_contract},
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()


def _idempotency_recovery_error(job_id: str, message: str) -> ProtocolLiveContractError:
    return ProtocolLiveContractError(
        message,
        details={"idempotency_recovery_required": True, "job_id": job_id},
    )


def _validate_terminal_live_replay(
    *,
    job_id: str,
    key_digest: str,
    request_fingerprint: str,
    reservation: Mapping[str, Any],
    bundle: Any,
) -> dict[str, Any]:
    if (
        reservation.get("schema_version") != PROTOCOL_LIVE_RESERVATION_SCHEMA_VERSION
        or reservation.get("job_id") != job_id
        or reservation.get("idempotency_key_digest") != key_digest
        or not isinstance(reservation.get("request_fingerprint"), str)
    ):
        raise _idempotency_recovery_error(
            job_id,
            "The live protocol idempotency reservation is invalid; operator recovery is required.",
        )
    if not isinstance(bundle, Mapping):
        raise _idempotency_recovery_error(
            job_id,
            "The live protocol operator bundle is invalid; operator recovery is required.",
        )
    execution = bundle.get("execution")
    if not isinstance(execution, Mapping):
        raise _idempotency_recovery_error(
            job_id,
            "The live protocol operator bundle execution record is invalid; operator recovery is required.",
        )
    runtime_state = execution.get("runtime_state")
    live_contract = execution.get("live_contract")
    binding = execution.get("idempotency_binding")
    protocol = bundle.get("protocol")
    protocol_document = protocol.get("document") if isinstance(protocol, Mapping) else None
    stage_states = runtime_state.get("stage_states") if isinstance(runtime_state, Mapping) else None
    action_results = runtime_state.get("action_results") if isinstance(runtime_state, Mapping) else None
    failed_stage_ids = {
        str(stage_id)
        for stage_id, stage_state in stage_states.items()
        if isinstance(stage_state, Mapping) and stage_state.get("status") == "failed"
    } if isinstance(stage_states, Mapping) else set()
    has_failed_action_result = (
        isinstance(action_results, list)
        and any(
            isinstance(result, Mapping)
            and result.get("stage_id") in failed_stage_ids
            and result.get("ok") is not True
            for result in action_results
        )
    )
    completed_terminal = (
        bundle.get("status") == "completed"
        and isinstance(runtime_state, Mapping)
        and runtime_state.get("completed") is True
    )
    failed_terminal = (
        bundle.get("status") == "failed"
        and isinstance(runtime_state, Mapping)
        and runtime_state.get("completed") is False
        and runtime_state.get("paused") is False
        and runtime_state.get("awaiting_review") is False
        and bool(failed_stage_ids)
        and has_failed_action_result
    )
    if (
        bundle.get("schema_version") != PROTOCOL_OPERATOR_BUNDLE_SCHEMA_VERSION
        or bundle.get("job_id") != job_id
        or not (completed_terminal or failed_terminal)
        or execution.get("dry_run") is not False
        or not isinstance(runtime_state, Mapping)
        or runtime_state.get("job_id") != job_id
        or runtime_state.get("dry_run") is not False
        or not isinstance(protocol, Mapping)
        or not isinstance(protocol_document, Mapping)
        or runtime_state.get("protocol_id") != protocol_document.get("protocol_id")
        or not isinstance(live_contract, Mapping)
        or not isinstance(binding, Mapping)
    ):
        raise _idempotency_recovery_error(
            job_id,
            "The live protocol operator bundle is not a terminal bound execution; operator recovery is required.",
        )
    persisted_fingerprint = _request_fingerprint(protocol, live_contract)
    if (
        binding.get("idempotency_key_digest") != key_digest
        or binding.get("request_fingerprint") != persisted_fingerprint
        or reservation.get("request_fingerprint") != persisted_fingerprint
    ):
        raise _idempotency_recovery_error(
            job_id,
            "The live protocol replay artifacts do not share one request binding; operator recovery is required.",
        )
    if request_fingerprint != persisted_fingerprint:
        raise ProtocolLiveContractError(
            "The live protocol idempotency key is already bound to different execution intent.",
            details={"idempotency_conflict": True, "job_id": job_id},
        )
    return dict(bundle)


def create_protocol_job(
    payload: Mapping[str, Any],
    *,
    dry_run: bool = True,
    store: ProtocolOperatorBundleStore | None = None,
    handlers: Mapping[ProtocolActionKind | str, ActionHandler] | None = None,
) -> dict[str, Any]:
    compiled = compile_protocol_source(payload)
    live_contract = None
    if not dry_run:
        live_contract = _build_live_execution_contract(payload=payload, compiled=compiled, handlers=handlers)
    created_at = _utc_now_iso()
    active_store = store or ProtocolOperatorBundleStore()
    if dry_run:
        job_id = f"protocol-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{uuid4().hex[:8]}"
        state = ProtocolExecutor(dry_run=True, job_id=job_id, handlers=handlers).execute(compiled.document)
        bundle = _build_operator_bundle(
            job_id=job_id,
            compiled=compiled,
            state=state,
            dry_run=True,
            created_at=created_at,
        )
        return active_store.save(bundle)

    raw_idempotency_key = payload.get("idempotency_key")
    if not isinstance(raw_idempotency_key, str) or not raw_idempotency_key.strip():
        raise ProtocolLiveContractError(
            "Live protocol execution requires an explicit idempotency key.",
            details={"missing_contract_fields": ["idempotency_key"]},
        )
    idempotency_key = raw_idempotency_key.strip()
    if len(idempotency_key) > 256:
        raise ProtocolLiveContractError(
            "The live protocol idempotency key exceeds 256 characters.",
            details={"invalid_contract_fields": ["idempotency_key"]},
        )
    key_digest = hashlib.sha256(idempotency_key.encode("utf-8")).hexdigest()
    job_id = f"protocol-live-{key_digest}"
    request_fingerprint = _request_fingerprint(compiled.to_payload(), dict(live_contract or {}))

    with active_store.live_creation_lock(job_id):
        reservation = active_store.load_live_reservation(job_id)
        if reservation is not None:
            try:
                replay_bundle = active_store.load(job_id)
            except Exception as exc:
                raise _idempotency_recovery_error(
                    job_id,
                    "A matching live protocol execution is reserved but its bundle is unreadable or incomplete; operator recovery is required.",
                ) from exc
            return _validate_terminal_live_replay(
                job_id=job_id,
                key_digest=key_digest,
                request_fingerprint=request_fingerprint,
                reservation=reservation,
                bundle=replay_bundle,
            )

        active_store.save_live_reservation(
            {
                "schema_version": PROTOCOL_LIVE_RESERVATION_SCHEMA_VERSION,
                "job_id": job_id,
                "idempotency_key_digest": key_digest,
                "request_fingerprint": request_fingerprint,
                "reserved_at": created_at,
            }
        )
        state = ProtocolExecutor(dry_run=False, job_id=job_id, handlers=handlers).execute(compiled.document)
        bundle = _build_operator_bundle(
            job_id=job_id,
            compiled=compiled,
            state=state,
            dry_run=False,
            created_at=created_at,
            live_contract=live_contract,
        )
        bundle["execution"]["idempotency_binding"] = {
            "idempotency_key_digest": key_digest,
            "request_fingerprint": request_fingerprint,
        }
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
        live_contract=_as_mapping(bundle.get("execution", {}).get("live_contract")) or None,
    )
    return active_store.save(updated_bundle)
