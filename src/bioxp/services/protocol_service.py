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
    oem_handlers: Mapping[str, ActionHandler] | None = None,
    validate: bool = True,
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
    live_ack = any(
        value is True
        for value in (
            get_value("live_execution_ack"),
            get_value("operator_ack"),
            get_value("operator_acknowledged_risk"),
        )
    )
    physical_console_verified = any(
        value is True
        for value in (
            get_value("physical_console_verified"),
            preflight.get("physical_console_verified"),
            preflight.get("operator_console_verified"),
        )
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
            if action.kind not in handler_kinds
            and action.kind not in {ProtocolActionKind.NOTE, ProtocolActionKind.PAUSE_REVIEW}
            and not (action.kind is ProtocolActionKind.OEM_OPERATION and
                     action.oem_opcode in ({"step", "delaypoint", "wait"} | set(oem_handlers or {})))
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

    if validate and (missing_contract_fields or missing_reference_axes or missing_live_handlers):
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
        "live_execution_ack": live_ack,
        "physical_console_verified": physical_console_verified,
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
    if state.workflow is not None:
        if state.workflow.phase == "queued":
            return "queued"
        if state.workflow.phase == "reconciling":
            return "ambiguous"
        if state.workflow.phase != "terminal":
            requested = state.workflow.requested_control or {}
            return "interrupting" if requested.get("action") in {"safe_stop", "abort"} else "dispatched"
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
        stage = state.stage_states[state.current_stage_id]
        action_id = stage.pause_marker_action_id
        if state.workflow is not None and state.workflow.gate == "review":
            action_id = None if state.workflow.gate_id == state.current_stage_id else stage.current_action_id
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

    @staticmethod
    def _save_json_atomically(path: Path, payload: Mapping[str, Any]) -> None:
        temporary_path = path.with_name(
            f".{path.name}.{os.getpid()}.{uuid4().hex}.tmp"
        )
        try:
            with temporary_path.open("w", encoding="utf-8") as temporary_file:
                json.dump(dict(payload), temporary_file, indent=2, sort_keys=True)
                temporary_file.flush()
                os.fsync(temporary_file.fileno())
            os.replace(temporary_path, path)
            directory_fd = os.open(
                path.parent,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
            )
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            temporary_path.unlink(missing_ok=True)

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
            self._save_json_atomically(
                preflight_path,
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
            )
        payload["artifacts"] = artifacts
        self._save_json_atomically(self._bundle_path(job_id), payload)
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
    idempotency_binding: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    updated_at = _utc_now_iso()
    status = _job_status_from_state(state)
    execution = {
        "dry_run": bool(dry_run),
        "runtime_state": state.to_payload(),
    }
    if live_contract is not None:
        execution["live_contract"] = dict(live_contract)
    if idempotency_binding is not None:
        execution["idempotency_binding"] = dict(idempotency_binding)
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


def _workflow_resources(document: ProtocolDocument) -> list[str]:
    # OEM lifecycle includes future Park/door/pipette work, not only current leaf.
    if document.metadata.get("input_mode") == "oem_prepared":
        return ["axis:x", "axis:y", "axis:z", "axis:g", "axis:door",
                "motor:4:0", "motor:4:1", "motor:5:0", "pipette", "thermal"]
    resources: set[str] = set()
    for action in _iter_document_actions(document):
        if action.kind in REFERENCE_REQUIRED_ACTION_KINDS or action.kind is ProtocolActionKind.THERMAL_DOOR:
            resources.update(("axis:x", "axis:y", "axis:z", "axis:g", "axis:door",
                              "motor:4:0", "motor:4:1", "motor:5:0"))
        if action.kind.value.startswith("pipette") or action.kind is ProtocolActionKind.TIP_EJECT:
            resources.add("pipette")
        if action.kind.value.startswith("thermal"):
            resources.add("thermal")
    return sorted(resources)


class ProtocolBindings(tuple):
    """Internal factory triple with separate optional host lifetime callbacks.

    Plain legacy triples and existing three-value unpacking remain supported.
    These attributes never enter the public OEM lifecycle/opcode dictionaries.
    """
    source_script_begin: Callable | None
    source_script_returned: Callable | None

    def __new__(cls, handlers, oem_handlers, lifecycle_handlers, *,
                source_script_begin=None, source_script_returned=None):
        value = super().__new__(cls, (handlers, oem_handlers, lifecycle_handlers))
        value.source_script_begin = source_script_begin
        value.source_script_returned = source_script_returned
        return value


def bind_protocol_dispatcher(command_store, *, binding_factory, artifact_store=None) -> None:
    """Register on the existing dispatch owner; no service worker or active registry."""
    def dispatch(command):
        job_id = command["command_id"]
        bundle = command["requested_inputs"]["bundle"]
        document = ProtocolDocument.from_payload(bundle["protocol"]["document"])
        executor = None
        bindings = binding_factory(bundle, source_executor=lambda: executor)
        handlers, oem_handlers, lifecycle_handlers = bindings
        state = ProtocolRuntimeState.from_payload(bundle["execution"]["runtime_state"])
        reviews = list(bundle["operator"]["reviews"])

        def publish(current):
            bundle["execution"]["runtime_state"] = current.to_payload()
            bundle["status"] = _job_status_from_state(current)
            bundle["updated_at"] = _utc_now_iso()
            bundle["operator"].update(manual_review_required=current.awaiting_review,
                                       pending_review=_pending_review_payload(current), reviews=reviews)
            canonical = command_store.publish_workflow(job_id, payload=bundle)
            current.workflow.child_command_ids[:] = canonical["execution"]["runtime_state"]["workflow"]["child_command_ids"]

        def wrap_action(handler):
            def invoke(action, current):
                identity = action.source_occurrence_id or action.action_id
                with command_store.workflow_context(job_id, source_occurrence_id=identity):
                    command_store.assert_workflow_current(job_id)
                    return handler(action, current)
            return invoke

        def wrap_lifecycle(name, handler):
            def invoke(current):
                with command_store.workflow_context(job_id, source_occurrence_id=f"lifecycle:{name}"):
                    command_store.assert_workflow_current(job_id)
                    return handler(current)
            return invoke

        executor = ProtocolExecutor(
            dry_run=False, job_id=job_id,
            handlers={key: wrap_action(handler) for key, handler in handlers.items()},
            oem_handlers={key: wrap_action(handler) for key, handler in oem_handlers.items()},
            lifecycle_handlers={key: wrap_lifecycle(key, handler) for key, handler in lifecycle_handlers.items()},
            on_state_change=publish,
            source_script_begin=getattr(bindings, "source_script_begin", None),
            source_script_returned=getattr(bindings, "source_script_returned", None),
            before_native_entry=lambda identity, current: command_store.assert_workflow_current(job_id),
        )

        def control(control_id, request):
            if request["action"] == "_addressed_stop":
                executor.interrupt(control_id=control_id, affected=True)
            elif request["action"] == "review":
                pending = _pending_review_payload(state)
                if not pending or pending["stage_id"] != request.get("stage_id") or pending["action_id"] != request.get("action_id"):
                    raise ProtocolLiveContractError("Review occurrence is not current.")
                executor.acknowledge_review(control_id=control_id, gate_id=state.workflow.gate_id)
                reviews.append({"reviewed_at": _utc_now_iso(), "reviewer": request["reviewer"],
                                "note": request.get("note"), "stage_id": request["stage_id"],
                                "action_id": request.get("action_id"), "control_command_id": control_id})
            else:
                executor.request_control(request["action"], control_id=control_id,
                                         **{key: request[key] for key in ("mode", "gate", "gate_id") if key in request})
            return state.workflow.to_payload()

        command_store.bind_workflow_controls(job_id, control)
        try:
            executor.execute(document, state=state)
            publish(state)
            bundle["status"] = executor.outcome
            bundle = command_store.finish_workflow(
                job_id, status=executor.outcome, payload=bundle,
                lifecycle_settled=state.workflow.phase == "terminal",
            )
            state.workflow.child_command_ids[:] = bundle["execution"]["runtime_state"]["workflow"]["child_command_ids"]
            try:
                (artifact_store or ProtocolOperatorBundleStore()).save(bundle)
            except Exception:
                # Canonical SQLite custody is already settled; an artifact-copy
                # failure does not undo or retry native execution.
                pass
        finally:
            command_store.unbind_workflow_controls(job_id)
    command_store.bind_workflow_dispatcher(dispatch)


def control_protocol_job(job_id: str, request: Mapping[str, Any], *, command_store) -> dict[str, Any]:
    variants = {"pause": {"mode"}, "wake": {"gate_id"}, "continue": {"gate", "gate_id"},
                "safe_stop": set(), "abort": set(), "review": {"reviewer", "note", "stage_id", "action_id"}}
    action = request.get("action")
    common = {"action", "idempotency_key", "command_id", "expected_ownership_generation"}
    if action not in variants or set(request) - (common | variants[action]):
        raise ProtocolLiveContractError("Invalid finite workflow control.")
    if type(request.get("expected_ownership_generation")) is not int or not isinstance(request.get("idempotency_key"), str) or not request["idempotency_key"].strip():
        raise ProtocolLiveContractError("Control requires canonical generation and idempotency key.")
    if request.get("command_id") != job_id:
        raise ProtocolLiveContractError("Control target does not match URL job.", details={"target_mismatch": True})
    return command_store.control_workflow(job_id, request=dict(request))


def create_protocol_job(
    payload: Mapping[str, Any],
    *,
    dry_run: bool = True,
    store: ProtocolOperatorBundleStore | None = None,
    handlers: Mapping[ProtocolActionKind | str, ActionHandler] | None = None,
    oem_handlers: Mapping[str, ActionHandler] | None = None,
    lifecycle_handlers: Mapping[str, Callable] | None = None,
    command_store=None,
    ownership_generation: int | None = None,
    board_epochs: Mapping[str, int] | None = None,
    binding_factory=None,
    authority_factory=None,
) -> dict[str, Any]:
    compiled = compile_protocol_source(payload)
    live_contract = None
    if not dry_run:
        if command_store is None:
            raise ProtocolLiveContractError("Canonical workflow custody is unavailable.", details={"reconciliation_required": True})
        live_contract = _build_live_execution_contract(
            payload=payload, compiled=compiled, handlers=handlers, oem_handlers=oem_handlers, validate=False,
        )
    created_at = _utc_now_iso()
    if dry_run:
        active_store = store or ProtocolOperatorBundleStore()
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

    retained = command_store.get_workflow(job_id)
    if retained is not None:
        binding = retained["execution"]["idempotency_binding"]
        if binding["request_fingerprint"] != request_fingerprint:
            raise ProtocolLiveContractError("The idempotency key is bound to different execution intent.",
                                            details={"idempotency_conflict": True, "job_id": job_id})
        return retained
    if authority_factory is not None:
        ownership_generation, board_epochs = authority_factory(compiled.document)
    bindings = None
    if binding_factory is not None:
        bindings = binding_factory({"protocol": compiled.to_payload()})
        handlers, oem_handlers, lifecycle_handlers = bindings
    if ownership_generation is None:
        raise ProtocolLiveContractError("Current canonical ownership is unavailable.")
    _build_live_execution_contract(payload=payload, compiled=compiled, handlers=handlers, oem_handlers=oem_handlers)
    executor = ProtocolExecutor(dry_run=False, handlers=handlers, oem_handlers=oem_handlers,
                                source_script_begin=getattr(bindings, "source_script_begin", None),
                                source_script_returned=getattr(bindings, "source_script_returned", None),
                                lifecycle_handlers=lifecycle_handlers,
                                before_native_entry=lambda identity, state: command_store.assert_workflow_current(state.job_id))
    support = executor.preflight(compiled.document)
    if support["ok"] is not True:
        raise ProtocolLiveContractError("Selected source dependencies are unbound.", details={"support": support})
    from ..protocols.validators import validate_protocol_support
    validate_protocol_support(compiled.document, handlers=handlers or {}, oem_handlers=oem_handlers or {},
                              lifecycle_handlers=lifecycle_handlers or {},
                              required_lifecycle=executor.required_lifecycle(compiled.document))
    if "epilogue_sweep" in executor.required_lifecycle(compiled.document):
        # Source sweep indexes four captured 96-well tip trays. Reject incomplete
        # preparation before native entry; never manufacture empty inventory.
        from ..protocols.runtime_state import ProtocolSourceModel
        model = ProtocolSourceModel.from_payload(compiled.document.to_payload()["metadata"].get("source_model", {}))
        trays_needed = 5 if any(
            action.oem_opcode == "ldtip" and len(action.params["arguments"]) > 3
            and action.params["arguments"][3] == "H"
            for action in _iter_document_actions(compiled.document)
        ) else 4
        if len(model.tip_trays) < trays_needed or any(len(tray.wells) < 96 for tray in model.tip_trays[:trays_needed]):
            raise ProtocolLiveContractError("Prepared source model lacks required captured tip-tray wells.")
    # Historical file custody is not permission to reissue under the new owner.
    active_store = store or ProtocolOperatorBundleStore()
    if active_store.load_live_reservation(job_id) is not None or active_store._bundle_path(job_id).exists():
        raise _idempotency_recovery_error(job_id, "Historical live custody requires reconciliation; it cannot be replayed.")
    from ..protocols.runtime_state import ProtocolWorkflowState, ProtocolSourceModel
    state = ProtocolRuntimeState.from_document(compiled.document, job_id=job_id, dry_run=False)
    if compiled.document.metadata.get("source_model") is not None:
        state.source_model = ProtocolSourceModel.from_payload(compiled.document.to_payload()["metadata"]["source_model"])
    state.workflow = ProtocolWorkflowState(command_id=job_id, phase="queued")
    bundle = _build_operator_bundle(
        job_id=job_id, compiled=compiled, state=state, dry_run=False,
        created_at=created_at, live_contract=live_contract,
        idempotency_binding={"idempotency_key_digest": key_digest, "request_fingerprint": request_fingerprint},
    )
    command_store.admit_workflow(
        command_id=job_id, idempotency_key=idempotency_key, plan_fingerprint=request_fingerprint,
        requested_inputs={"bundle": bundle, "plan_fingerprint": request_fingerprint},
        ownership_generation=ownership_generation, resources=_workflow_resources(compiled.document),
        board_epochs=dict(board_epochs or {}),
    )
    return command_store.get_workflow(job_id)


def get_protocol_job(job_id: str, *, store: ProtocolOperatorBundleStore | None = None, command_store=None) -> dict[str, Any]:
    if command_store is not None:
        canonical = command_store.get_workflow(job_id)
        if canonical is not None:
            return canonical
    active_store = store or ProtocolOperatorBundleStore()
    return active_store.load(job_id)


def list_protocol_jobs(*, limit: int = 20, store: ProtocolOperatorBundleStore | None = None, command_store=None) -> list[dict[str, Any]]:
    canonical = command_store.list_workflows(limit=limit) if command_store is not None else []
    seen = {row["job_id"] for row in canonical}
    try:
        historical = (store or ProtocolOperatorBundleStore()).list(limit=limit)
    except OSError:
        if not canonical:
            raise
        historical = []
    return (canonical + [row for row in historical if row["job_id"] not in seen])[:limit]


def _review_protocol_job_locked(
    job_id: str,
    *,
    reviewer: str = "operator",
    note: str | None = None,
    active_store: ProtocolOperatorBundleStore,
    handlers: Mapping[Any, Any] | None = None,
) -> dict[str, Any]:
    loaded_bundle = active_store.load(job_id)
    bundle = _as_mapping(loaded_bundle)
    execution = _as_mapping(bundle.get("execution"))
    if execution.get("dry_run") is False:
        raise _idempotency_recovery_error(job_id, "Historical live review cannot reconstruct an executor; reconcile canonical custody.")
    operator = _as_mapping(bundle.get("operator"))
    protocol = _as_mapping(bundle.get("protocol"))
    runtime_state_payload = _as_mapping(execution.get("runtime_state"))
    raw_reviews = operator.get("reviews")
    if (
        not bundle
        or not execution
        or not protocol
        or not runtime_state_payload
        or bundle.get("job_id") != job_id
        or runtime_state_payload.get("job_id") != job_id
        or type(execution.get("dry_run")) is not bool
        or (raw_reviews is not None and not isinstance(raw_reviews, list))
        or any(not isinstance(review, Mapping) for review in (raw_reviews or []))
    ):
        raise ProtocolLiveContractError(
            "The stored protocol review bundle is malformed.",
            details={"review_contract_invalid": True, "job_id": job_id},
        )
    try:
        state = ProtocolRuntimeState.from_payload(runtime_state_payload)
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise ProtocolLiveContractError(
            "The stored protocol runtime state is malformed.",
            details={"review_contract_invalid": True, "job_id": job_id},
        ) from exc
    if state.current_stage_id is not None and not isinstance(state.current_stage_id, str):
        raise ProtocolLiveContractError(
            "The stored protocol runtime state is malformed.",
            details={"review_contract_invalid": True, "job_id": job_id},
        )
    reviews = [dict(review) for review in (raw_reviews or [])]
    if (
        bundle.get("status") == "completed"
        and state.completed is True
        and not state.awaiting_review
        and reviews
        and reviews[-1].get("reviewer") == reviewer
        and reviews[-1].get("note") == note
    ):
        return bundle
    if bundle.get("status") != "awaiting_review" or not state.awaiting_review:
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

    try:
        compiled = CompiledProtocolSource(
            source_type=str(protocol.get("source_type") or "native"),
            document=ProtocolDocument.from_payload(protocol["document"]),
            source_path=protocol.get("source_path"),
            coverage=_as_mapping(protocol.get("coverage")),
            experiment=_as_mapping(protocol.get("experiment")),
            inventory=_as_mapping(protocol.get("inventory")),
        )
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise ProtocolLiveContractError(
            "The stored protocol document is malformed.",
            details={"review_contract_invalid": True, "job_id": job_id},
        ) from exc
    dry_run = bool(execution.get("dry_run", True))
    idempotency_binding: Mapping[str, Any] | None = None
    claim_id = uuid4().hex
    claimed_bundle = dict(bundle)
    claimed_bundle["status"] = "review_dispatching"
    claimed_bundle["updated_at"] = _utc_now_iso()
    claimed_execution = dict(claimed_bundle.get("execution") or {})
    claimed_execution["review_claim"] = {
        "claim_id": claim_id,
        "claimed_at": claimed_bundle["updated_at"],
        "reviewer": reviewer,
    }
    claimed_bundle["execution"] = claimed_execution
    active_store.save(claimed_bundle)
    try:
        resumed_state = ProtocolExecutor(dry_run=dry_run, handlers=handlers).execute(
            compiled.document,
            state=state,
        )
    except Exception:
        failed_bundle = dict(claimed_bundle)
        failed_bundle["status"] = "review_failed_ambiguous" if not dry_run else "review_failed"
        failed_bundle["updated_at"] = _utc_now_iso()
        active_store.save(failed_bundle)
        raise
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
        dry_run=dry_run,
        created_at=str(bundle.get("created_at") or _utc_now_iso()),
        reviews=reviews,
        live_contract=_as_mapping(bundle.get("execution", {}).get("live_contract")) or None,
        idempotency_binding=idempotency_binding,
    )
    return active_store.save(updated_bundle)


def review_protocol_job(
    job_id: str,
    *,
    reviewer: str = "operator",
    note: str | None = None,
    store: ProtocolOperatorBundleStore | None = None,
    handlers: Mapping[Any, Any] | None = None,
    command_store=None,
    request: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    if command_store is not None and command_store.get_workflow(job_id) is not None:
        if request is None:
            raise ProtocolLiveContractError("Live review requires canonical target, generation, key and occurrence.")
        control_protocol_job(job_id, {**dict(request), "action": "review", "reviewer": reviewer, "note": note}, command_store=command_store)
        return command_store.get_workflow(job_id)
    active_store = store or ProtocolOperatorBundleStore()
    with active_store.live_creation_lock(job_id):
        return _review_protocol_job_locked(
            job_id,
            reviewer=reviewer,
            note=note,
            active_store=active_store,
            handlers=handlers,
        )
