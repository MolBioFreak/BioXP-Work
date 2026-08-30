from __future__ import annotations

import base64
import hashlib
import fcntl
from functools import wraps
import json
import os
import sqlite3
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from ..hardware_status import hardware_state
from ..oem_full_lifecycle import current_authority_identity, current_registry_sha256
from ..runtime_audit_store import (
    NONTERMINAL_COMMAND_STATES,
    TERMINAL_COMMAND_STATES,
    RuntimeAuditDatabase,
    canonical_json,
)
from ..release_identity import current_release_identity
from ..storage_operations import create_backup_unit, verify_backup_unit
from .audit import PipetteAuditIntegrityError, normalize_pipette_result


_LINKED_FINALIZATION_KEY = "_bioxp_linked_pipette_finalization"


class PipetteReceiptError(RuntimeError):
    """A pipette operation cannot be durably represented."""

    def __init__(
        self,
        message: str,
        *,
        linked_finalization: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.linked_finalization = (
            None if linked_finalization is None else dict(linked_finalization)
        )


def _current_replay_identity() -> dict[str, Any]:
    release = current_release_identity()
    if release.get("verified") is not True:
        raise PipetteReceiptError("verified release identity is required for pipette claims and replay")
    source_value = release.get("source")
    source = source_value if isinstance(source_value, Mapping) else {}
    try:
        authority = current_authority_identity()
        registry_sha256 = current_registry_sha256()
    except Exception as exc:
        raise PipetteReceiptError(f"serial-206 replay authority is unavailable: {exc}") from exc
    if authority.get("evidence_lock_identity_verified") is not True:
        raise PipetteReceiptError("verified serial-206 evidence-lock identity is required for replay")
    return {
        "robot_identity": os.getenv("BIOXP_ROBOT_IDENTITY", "serial206").strip()
        or "serial206",
        "release_id": release.get("release_id")
        if isinstance(release.get("release_id"), str)
        else None,
        "source_manifest_sha256": source.get("manifest_sha256")
        if isinstance(source.get("manifest_sha256"), str)
        else None,
        "source_aggregate_sha256": source.get("aggregate_sha256")
        if isinstance(source.get("aggregate_sha256"), str)
        else None,
        "release_verified": True,
        "registry_sha256": registry_sha256,
        "evidence_lock_sha256": authority.get("evidence_lock_sha256"),
        "evidence_lock_identity_verified": True,
    }


def _claim_source_identity(runtime_binding: Mapping[str, Any] | None) -> dict[str, Any]:
    identity = dict(runtime_binding or {"authority": "robot_runtime"})
    for key, value in _current_replay_identity().items():
        if key in identity and identity[key] != value:
            raise PipetteReceiptError(f"pipette claim {key} contradicts current authority")
        identity[key] = value
    return identity


class _LegacyReceiptValidationError(ValueError):
    def __init__(self, code: str, detail: str) -> None:
        super().__init__(detail)
        self.code = str(code)
        self.detail = str(detail)


def _migration_file_lock(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        lock_path = self.root / "pipette-migration.lock"
        lock_path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        with lock_path.open("a+") as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
            try:
                return method(self, *args, **kwargs)
            finally:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)

    return wrapped


_SECRET_KEYS = frozenset(
    {
        "password",
        "passwd",
        "secret",
        "token",
        "api_key",
        "apikey",
        "access_token",
        "refresh_token",
        "connection_string",
    }
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _redact(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {
            str(key): "[REDACTED]" if str(key).lower() in _SECRET_KEYS else _redact(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_redact(item) for item in value]
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class PipetteReceiptStore:
    """Durable private journal for source-bound pipette operation receipts."""

    def __init__(
        self,
        root: str | Path | None = None,
        *,
        initialize_schema: bool = False,
    ) -> None:
        self._audit_database = RuntimeAuditDatabase(root=root, initialize_schema=False)
        self.connection = self._audit_database.connection
        self.root = self._audit_database.root
        if initialize_schema:
            raise PipetteReceiptError("constructor-owned migration is retired; prepare the canonical database explicitly")
        from ..oem_runtime_store import verify_canonical_runtime_database

        verify_canonical_runtime_database(self.connection)
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.root, 0o700)
        self.path = self._audit_database.path
        self.receipts_path = None
        root_legacy = self.root / "receipts.jsonl"
        pipette_legacy = self.root / "pipette" / "receipts.jsonl"
        existing_legacy = [path for path in (root_legacy, pipette_legacy) if path.is_file()]
        if len(existing_legacy) > 1:
            raise PipetteReceiptError("multiple active pipette JSONL sources require explicit cleanup")
        self._legacy_path = existing_legacy[0] if existing_legacy else pipette_legacy
        self._legacy_path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self._legacy_path.parent, 0o700)
        self._lock = self._audit_database.writer_lock
        self.lock = self._lock

    def _source_identity(self) -> dict[str, Any]:
        repo = Path(__file__).resolve().parents[3]
        tracked = {
            "pipette_models": repo / "src/bioxp/pipette/models.py",
            "pipette_transport": repo / "src/bioxp/pipette/transport.py",
            "pipette_receipts": repo / "src/bioxp/pipette/receipts.py",
            "can_driver": repo / "src/bioxp/can_driver.py",
            "novo_router": repo / "src/bioxp/novo_router.py",
            "novo_usb_can": repo / "src/bioxp/novo_usb_can.py",
            "pipette_service": repo / "src/bioxp/services/pipette_service.py",
            "pipette_spec": repo / "docs/specs/2026-08-02-pipette-oem-gap-rectification-spec.md",
        }
        missing = [name for name, path in tracked.items() if not path.is_file()]
        if missing:
            raise PipetteReceiptError(f"pipette source identity is incomplete: {missing}")
        try:
            authority = current_authority_identity()
            registry_sha256 = current_registry_sha256()
        except Exception as exc:  # pragma: no cover - authority failure is environment-specific
            raise PipetteReceiptError(f"serial-206 evidence authority unavailable: {exc}") from exc
        return {
            "repository_root": str(repo),
            "source_sha256": {name: _sha256(path) for name, path in tracked.items()},
            "registry_sha256": registry_sha256,
            "evidence_authority": authority,
            "authority_verified": bool(authority.get("evidence_lock_identity_verified")),
            "release_identity": current_release_identity(),
        }

    def _is_linked_operator_claim(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
    ) -> bool:
        row = self.connection.execute(
            """
            SELECT c.command_kind
            FROM operator_commands AS c
            JOIN pipette_operations AS p ON p.command_id=c.command_id
            WHERE c.command_id=? AND p.pipette_operation_id=?
            """,
            (str(command_id), str(pipette_operation_id)),
        ).fetchone()
        if row is None:
            raise PipetteReceiptError("linked pipette claim is unavailable")
        return str(row["command_kind"] or "") == "operator"

    @staticmethod
    def _linked_finalization(
        *,
        command_id: str,
        pipette_operation_id: str,
        expected_status: str,
        status: str,
        outcome: str,
        failure_code: str | None,
        result: Mapping[str, Any],
        effective_inputs: Mapping[str, Any] | None,
        receipt: Mapping[str, Any],
        normalized: Mapping[str, Any],
    ) -> dict[str, Any]:
        return {
            "command_id": str(command_id),
            "pipette_operation_id": str(pipette_operation_id),
            "expected_status": str(expected_status),
            "status": str(status),
            "outcome": str(outcome),
            "failure_code": None if failure_code is None else str(failure_code),
            "result": dict(result),
            "effective_inputs": dict(effective_inputs or {}),
            "receipt": dict(receipt),
            "normalized": dict(normalized),
        }

    @staticmethod
    def _truth(result: Mapping[str, Any]) -> dict[str, Any]:
        driver_result = result.get("driver_result")
        driver = driver_result if isinstance(driver_result, Mapping) else {}
        provenance_result = driver.get("provenance")
        provenance = provenance_result if isinstance(provenance_result, Mapping) else {}

        delivery = result.get("delivery_verified")
        if not isinstance(delivery, bool):
            driver_delivery = driver.get("delivery_verified")
            delivery = driver_delivery if isinstance(driver_delivery, bool) else bool(driver.get("tx_ok", False))

        controller = result.get("controller_acknowledged")
        if not isinstance(controller, bool):
            driver_controller = driver.get("controller_acknowledged")
            controller = driver_controller if isinstance(driver_controller, bool) else False

        completion = result.get("completion_verified")
        completion_explicit = isinstance(completion, bool)
        if not completion_explicit:
            driver_completion = driver.get("completion_verified")
            if isinstance(driver_completion, bool):
                completion = driver_completion
                completion_explicit = True
            else:
                completion = bool(provenance.get("completion_received", False))
        if not completion_explicit:
            completion = False

        precondition = result.get("hardware_precondition_verified")
        if not isinstance(precondition, bool):
            precondition = False
        hardware = result.get("hardware_postcondition_verified")
        if not isinstance(hardware, bool):
            hardware = False
        semantic_query = result.get("semantic_query_response_verified") is True
        if semantic_query:
            delivery = False
            controller = False
            completion = False
            precondition = False
            hardware = False
        # No caller in the authorized no-motion envelope can certify physical effect.
        return {
            "delivery_verified": bool(delivery),
            "controller_acknowledged": bool(controller),
            "completion_verified": bool(completion),
            "semantic_query_response_verified": semantic_query,
            "hardware_precondition_verified": bool(precondition),
            "hardware_postcondition_verified": bool(hardware),
            "physical_effect_verified": False,
            "physical_effect_claim_suppressed": True,
        }

    def record(
        self,
        *,
        operation: str,
        requested_inputs: Mapping[str, Any] | None,
        result: Mapping[str, Any],
        effective_inputs: Mapping[str, Any] | None = None,
        runtime_binding: Mapping[str, Any] | None = None,
        command_id: str | None = None,
        pipette_operation_id: str | None = None,
        expected_status: str | None = None,
    ) -> dict[str, Any]:
        if not isinstance(result, Mapping):
            raise PipetteReceiptError("receipt result must be a mapping")
        if (command_id is None) != (pipette_operation_id is None):
            raise PipetteReceiptError("typed receipt persistence requires command_id and pipette_operation_id together")
        if command_id is not None and expected_status is None:
            raise PipetteReceiptError("typed receipt finalization requires expected_status")
        if command_id is None and pipette_operation_id is None:
            binding = dict(runtime_binding or {})
            idempotency_key = str(binding.get("idempotency_key") or f"record:{operation}:{uuid.uuid4().hex}")
            binding.setdefault(
                "callback_session_id",
                f"pipette-callback:{hashlib.sha256(idempotency_key.encode('utf-8')).hexdigest()[:32]}",
            )
            claim, _ = self.claim(
                operation=str(operation),
                requested_inputs=requested_inputs,
                entrypoint_id=str(binding.get("entrypoint_id") or "legacy.record"),
                caller_class=str(binding.get("caller_class") or "legacy"),
                control_class=str(binding.get("control_class") or "pipette_state_command"),
                idempotency_key=idempotency_key,
                ownership_generation=int(binding.get("ownership_generation") or getattr(hardware_state, "ownership_epoch", 0)),
                connection_generation=binding.get("connection_generation", 0),
                protocol_job_id=binding.get("protocol_job_id"),
                protocol_action_id=binding.get("protocol_action_id"),
                lifecycle_stage_id=binding.get("lifecycle_stage_id"),
                callback_session_id=binding.get("callback_session_id"),
                runtime_binding=binding,
            )
            return self.record(
                operation=operation,
                requested_inputs=requested_inputs,
                result=result,
                effective_inputs=effective_inputs,
                runtime_binding=binding,
                command_id=claim["command_id"],
                pipette_operation_id=claim["pipette_operation_id"],
                expected_status=str(claim["status"]),
            )
        source_identity = self._source_identity()
        receipt = {
            "schema": "bioxp.pipette.receipt.v1",
            "receipt_id": uuid.uuid4().hex,
            "created_at": _utc_now(),
            "operation": str(operation),
            "requested_inputs": _redact(dict(requested_inputs or {})),
            "effective_inputs": _redact(dict(effective_inputs or {})),
            "result": _redact(dict(result)),
            "truth": self._truth(result),
            "runtime_binding": _redact(dict(runtime_binding or {"owner": "pipette_receipt_store"})),
            "ownership_epoch": int(getattr(hardware_state, "ownership_epoch", 0)),
            "source_identity": source_identity,
            "deployment_identity": current_release_identity(),
        }
        if command_id is not None and pipette_operation_id is not None:
            linked_operator_claim = self._is_linked_operator_claim(
                command_id=str(command_id),
                pipette_operation_id=str(pipette_operation_id),
            )
            target_status = (
                "observed"
                if (
                    receipt["truth"]["semantic_query_response_verified"]
                    or receipt["truth"]["hardware_postcondition_verified"]
                )
                else "completed"
                if receipt["truth"]["completion_verified"]
                else "acknowledged"
                if receipt["truth"]["controller_acknowledged"]
                else "dispatched"
                if receipt["truth"]["delivery_verified"]
                else "failed"
            )
            outcome = str(
                result.get("outcome")
                or ("completed" if result.get("ok") is True else "failed")
            )
            failure_code = (
                None
                if result.get("ok") is True
                else str(result.get("error") or result.get("code") or "pipette_operation_failed")
            )
            try:
                normalized_result = normalize_pipette_result(result)
                linked_finalization = self._linked_finalization(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    expected_status=str(expected_status),
                    status=target_status,
                    outcome=outcome,
                    failure_code=failure_code,
                    result=result,
                    effective_inputs=effective_inputs,
                    receipt=receipt,
                    normalized=normalized_result,
                )
                if linked_operator_claim:
                    receipt[_LINKED_FINALIZATION_KEY] = linked_finalization
                else:
                    with self._audit_database.pipette_finalization_transaction():
                        self._audit_database.finalize_claim(
                            command_id=str(command_id),
                            pipette_operation_id=str(pipette_operation_id),
                            expected_status=str(expected_status),
                            status=target_status,
                            outcome=outcome,
                            failure_code=failure_code,
                            result=result,
                            effective_inputs=effective_inputs,
                            receipt_json=json.dumps(receipt, sort_keys=True),
                        )
                        self._audit_database.persist_normalized_pipette_result(
                            command_id=str(command_id),
                            pipette_operation_id=str(pipette_operation_id),
                            result=result,
                            normalized=normalized_result,
                        )
            except (PipetteAuditIntegrityError, TypeError, ValueError, KeyError) as exc:
                failure_result = {
                    "ok": False,
                    "outcome": "normalization_failed",
                    "error": str(exc),
                    "failure_code": "pipette_result_normalization_failed",
                    "physical_effect_verified": False,
                }
                receipt["result"] = failure_result
                receipt["failure_code"] = "pipette_result_normalization_failed"
                receipt["normalization_error"] = str(exc)
                if linked_operator_claim:
                    linked_finalization = self._linked_finalization(
                        command_id=str(command_id),
                        pipette_operation_id=str(pipette_operation_id),
                        expected_status=str(expected_status),
                        status="failed",
                        outcome="normalization_failed",
                        failure_code="pipette_result_normalization_failed",
                        result=failure_result,
                        effective_inputs=effective_inputs,
                        receipt=receipt,
                        normalized={
                            "channels": [],
                            "exchanges": [],
                            "events": [],
                            "pressure_samples": [],
                        },
                    )
                    raise PipetteReceiptError(
                        str(exc), linked_finalization=linked_finalization
                    ) from exc
                try:
                    self._audit_database.finalize_claim(
                        command_id=str(command_id),
                        pipette_operation_id=str(pipette_operation_id),
                        expected_status=str(expected_status),
                        status="failed",
                        outcome="normalization_failed",
                        failure_code="pipette_result_normalization_failed",
                        result=failure_result,
                        effective_inputs=effective_inputs,
                        receipt_json=json.dumps(receipt, sort_keys=True),
                    )
                except Exception as finalize_exc:
                    raise PipetteReceiptError(
                        f"typed pipette normalization failure could not be finalized: {finalize_exc}"
                    ) from finalize_exc
                raise PipetteReceiptError(str(exc)) from exc
            except PipetteReceiptError:
                raise
            except Exception as exc:
                raise PipetteReceiptError(f"typed pipette audit persistence failed: {exc}") from exc
        return receipt

    def _persist_normalized_result(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        result: Mapping[str, Any],
        normalized: Mapping[str, Any] | None = None,
    ) -> dict[str, list[str]]:
        normalized_result = (
            dict(normalized) if normalized is not None else normalize_pipette_result(result)
        )
        return self._audit_database.persist_normalized_pipette_result(
            command_id=str(command_id),
            pipette_operation_id=str(pipette_operation_id),
            result=result,
            normalized=normalized_result,
        )

    def record_failure(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        operation: str,
        failure_code: str,
        message: str,
        expected_status: str,
        status: str = "failed",
        requested_inputs: Mapping[str, Any] | None = None,
        runtime_binding: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        outcome_unknown = status in {"outcome_unknown", "reconciliation_required"}
        result = {
            "ok": False,
            "outcome": "outcome_unknown" if outcome_unknown else "rejected" if status == "rejected" else "failed",
            "error": str(message),
            "failure_code": str(failure_code),
            "runtime_binding": dict(runtime_binding or {}),
            "completion_ambiguous": outcome_unknown,
            "reconciliation_required": outcome_unknown,
            "retry_forbidden": outcome_unknown,
            "physical_effect_verified": False,
        }
        try:
            normalized_result = normalize_pipette_result(result)
            if self._is_linked_operator_claim(
                command_id=str(command_id),
                pipette_operation_id=str(pipette_operation_id),
            ):
                linked_finalization = self._linked_finalization(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    expected_status=str(expected_status),
                    status=str(status),
                    outcome=str(result["outcome"]),
                    failure_code=str(failure_code),
                    result=result,
                    effective_inputs={},
                    receipt=result,
                    normalized=normalized_result,
                )
                return {**result, _LINKED_FINALIZATION_KEY: linked_finalization}
            with self._audit_database.pipette_finalization_transaction():
                self._audit_database.finalize_claim(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    expected_status=str(expected_status),
                    status=str(status),
                    outcome=str(result["outcome"]),
                    failure_code=str(failure_code),
                    result=result,
                    effective_inputs={},
                    receipt_json=json.dumps(result, sort_keys=True),
                )
                self._audit_database.persist_normalized_pipette_result(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    result=result,
                    normalized=normalized_result,
                )
        except Exception as exc:
            raise PipetteReceiptError(f"pipette failure persistence failed: {exc}") from exc
        return result

    def reconcile_nonterminal_claims(self) -> int:
        return self._audit_database.reconcile_nonterminal_claims()

    def record_channel_observation(self, **kwargs: Any) -> str:
        return self._audit_database.record_channel_observation(**kwargs)

    def record_transport_exchange(self, **kwargs: Any) -> str:
        try:
            return self._audit_database.record_transport_exchange(**kwargs)
        except ValueError as exc:
            raise PipetteAuditIntegrityError(str(exc)) from exc

    def record_event(self, **kwargs: Any) -> str:
        return self._audit_database.record_event(**kwargs)

    def record_pressure_stream(self, **kwargs: Any) -> str:
        return self._audit_database.record_pressure_stream(**kwargs)

    def record_pressure_chunk(self, **kwargs: Any) -> str:
        return self._audit_database.record_pressure_chunk(**kwargs)

    def claim(
        self,
        *,
        operation: str,
        requested_inputs: Mapping[str, Any] | None,
        entrypoint_id: str,
        caller_class: str,
        control_class: str,
        idempotency_key: str,
        action_id: str | None = None,
        command_id: str | None = None,
        ownership_generation: int = 0,
        connection_generation: int | None = None,
        protocol_job_id: str | None = None,
        protocol_action_id: str | None = None,
        lifecycle_stage_id: str | None = None,
        lifecycle_attempt_id: str | None = None,
        callback_session_id: str | None = None,
        runtime_binding: Mapping[str, Any] | None = None,
        attach_to_existing_command: bool = False,
    ) -> tuple[dict[str, Any], bool]:
        command = str(command_id or f"pipette_{uuid.uuid4().hex}")
        payload = {
            "command_id": command,
            "idempotency_key": str(idempotency_key),
            "action_id": str(action_id or f"pipette.{operation}"),
            "operation": str(operation),
            "entrypoint_id": str(entrypoint_id),
            "caller_class": str(caller_class),
            "control_class": str(control_class),
            "ownership_generation": int(ownership_generation),
            "connection_generation": connection_generation,
            "protocol_job_id": protocol_job_id,
            "protocol_action_id": protocol_action_id,
            "lifecycle_stage_id": lifecycle_stage_id,
            "lifecycle_attempt_id": lifecycle_attempt_id,
            "callback_session_id": callback_session_id,
            "source_identity": _claim_source_identity(runtime_binding),
            "requested_inputs": dict(requested_inputs or {}),
        }
        return self._audit_database.claim(
            payload,
            pipette=True,
            attach_to_existing_command=bool(attach_to_existing_command),
        )

    def attach_to_operator_claim(self, **kwargs: Any) -> tuple[dict[str, Any], bool]:
        runtime_binding = kwargs.pop("runtime_binding", None)
        payload = {
            **dict(kwargs),
            "action_id": str(kwargs.get("action_id") or f"pipette.{kwargs['operation']}"),
            "operation_class": "pipette",
            "source_identity": _claim_source_identity(runtime_binding),
        }
        return self._audit_database.attach_pipette_child(payload)

    def replay_result(self, *, command_id: str, pipette_operation_id: str) -> dict[str, Any]:
        row = self.connection.execute(
            "SELECT * FROM pipette_operations WHERE command_id=? AND pipette_operation_id=?",
            (str(command_id), str(pipette_operation_id)),
        ).fetchone()
        if row is None:
            raise PipetteReceiptError("durable pipette replay receipt is unavailable")
        current_generation = int(getattr(hardware_state, "ownership_epoch", 0))
        if int(row["ownership_generation"]) != current_generation:
            raise PipetteReceiptError("pipette replay ownership generation is stale")
        try:
            source_identity = json.loads(str(row["source_identity_json"] or "{}"))
        except json.JSONDecodeError as exc:
            raise PipetteReceiptError("pipette replay source identity is malformed") from exc
        if not isinstance(source_identity, Mapping):
            raise PipetteReceiptError("pipette replay source identity is unavailable")
        for key, expected in _current_replay_identity().items():
            if source_identity.get(key) != expected:
                raise PipetteReceiptError(f"pipette replay {key} is stale")
        receipt: dict[str, Any] = {}
        if row["receipt_json"]:
            parsed = json.loads(str(row["receipt_json"]))
            if isinstance(parsed, Mapping):
                receipt = dict(parsed)
        nested = receipt.get("result")
        replay = dict(nested) if isinstance(nested, Mapping) else dict(receipt)
        status = str(row["status"] or "reserved")
        replay.update({
            "replayed": True,
            "command_id": str(command_id),
            "pipette_operation_id": str(pipette_operation_id),
            "status": status,
        })
        known_states = NONTERMINAL_COMMAND_STATES | TERMINAL_COMMAND_STATES
        unresolved = status not in known_states or status in {
            "ambiguous",
            "outcome_unknown",
            "reconciliation_required",
        }
        if status in NONTERMINAL_COMMAND_STATES or unresolved:
            replay["ok"] = False
            replay["outcome"] = (
                "outcome_unknown"
                if unresolved
                else "in_progress"
            )
            replay["retry_forbidden"] = True
            replay["reconciliation_required"] = unresolved
        elif status in {"failed", "rejected", "cleared", "cancelled"}:
            replay["ok"] = False
            replay.setdefault("outcome", status)
        return replay

    def record_runtime_error_event(self, channel: int, error_code: int) -> str:
        """Persist an asynchronous OEM pipette error without inventing command ownership."""
        return self._audit_database.record_event(
            command_id=None,
            pipette_operation_id=None,
            event_source="ClassPipetteCollection.handlePipetteMessage",
            event_kind="pipette_error",
            event_payload={
                "channel": int(channel),
                "error_code": int(error_code),
            },
            channel=int(channel),
            ownership_generation=int(getattr(hardware_state, "ownership_epoch", 0)),
            semantic_validity="valid",
        )

    def _migration_backup_relpath(self, unit_relpath: str | None) -> str | None:
        if not unit_relpath:
            return None
        selected = Path(unit_relpath)
        if (
            selected.is_absolute()
            or not selected.parts
            or selected.parts[0] != "backups"
            or any(part in {"", ".", ".."} for part in selected.parts)
        ):
            raise PipetteReceiptError("migration backup must be a runtime-root-relative backup unit")
        return selected.as_posix()

    def _ensure_migration_evidence(
        self,
        *,
        migration_id: str,
        source_path: str,
        source_digest: str,
        source_bytes: int,
        source_count: int,
        imported_count: int,
        duplicate_count: int,
        quarantined_count: int,
        backup_relpath: str | None,
        archive_relpath: str | None,
    ) -> None:
        existing = self.connection.execute(
            """
            SELECT migration_id,source_path,source_digest,source_bytes,source_count,
                   imported_count,duplicate_count,quarantined_count,backup_relpath,
                   archive_relpath
            FROM runtime_migration_evidence WHERE migration_id=?
            """,
            (migration_id,),
        ).fetchone()
        if existing is not None:
            expected = {
                "migration_id": migration_id,
                "source_path": source_path,
                "source_digest": source_digest,
                "source_bytes": int(source_bytes),
                "source_count": int(source_count),
                "imported_count": int(imported_count),
                "duplicate_count": int(duplicate_count),
                "quarantined_count": int(quarantined_count),
                "backup_relpath": backup_relpath,
                "archive_relpath": archive_relpath,
            }
            if any(existing[name] != value for name, value in expected.items()):
                raise PipetteReceiptError("migration evidence conflicts with validated candidate")
            return
        self.connection.execute(
            """
            INSERT INTO runtime_migration_evidence(
                migration_id,source_path,source_digest,source_bytes,source_count,
                imported_count,duplicate_count,quarantined_count,backup_relpath,
                archive_relpath,created_at
            ) VALUES(?,?,?,?,?,?,?,?,?,?,?)
            """,
            (
                migration_id,
                source_path,
                source_digest,
                int(source_bytes),
                int(source_count),
                int(imported_count),
                int(duplicate_count),
                int(quarantined_count),
                backup_relpath,
                archive_relpath,
                time.time(),
            ),
        )

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        descriptor = os.open(
            str(path),
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0),
        )
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _migration_artifact_path(self, relpath: str) -> Path:
        relative = Path(str(relpath))
        if relative.is_absolute():
            raise PipetteReceiptError("migration artifact path must be relative")
        root = self.root.resolve()
        candidate = (root / relative).resolve(strict=False)
        try:
            candidate.relative_to(root)
        except ValueError as exc:
            raise PipetteReceiptError("migration artifact escaped the runtime root") from exc
        return candidate

    def _write_immutable_migration_artifact(
        self,
        path: Path,
        data: bytes,
        *,
        expected_sha256: str,
    ) -> None:
        if hashlib.sha256(data).hexdigest() != expected_sha256:
            raise PipetteReceiptError("migration artifact digest does not match supplied bytes")
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(path.parent, 0o700)
        if path.is_symlink():
            raise PipetteReceiptError("migration artifact path is a symbolic link")
        if path.exists():
            if not path.is_file() or path.read_bytes() != data:
                raise PipetteReceiptError("immutable migration artifact contains different bytes")
            os.chmod(path, 0o400)
            persisted = path.read_bytes()
            if (
                len(persisted) != len(data)
                or persisted != data
                or hashlib.sha256(persisted).hexdigest() != expected_sha256
            ):
                raise PipetteReceiptError("immutable migration artifact digest mismatch")
            self._fsync_directory(path.parent)
            return
        temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("xb") as handle:
                os.chmod(temporary, 0o600)
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
            if hashlib.sha256(temporary.read_bytes()).hexdigest() != expected_sha256:
                raise PipetteReceiptError("migration artifact write verification failed")
            os.replace(temporary, path)
            os.chmod(path, 0o400)
            self._fsync_directory(path.parent)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    @staticmethod
    def _validate_legacy_receipt(legacy: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
        if legacy.get("schema") != "bioxp.pipette.receipt.v1":
            raise _LegacyReceiptValidationError("unsupported_schema", "schema must be bioxp.pipette.receipt.v1")
        receipt_id = legacy.get("receipt_id")
        if not isinstance(receipt_id, str) or not receipt_id.strip():
            raise _LegacyReceiptValidationError("invalid_receipt_id", "receipt_id must be a non-empty string")
        created_at = legacy.get("created_at")
        if not isinstance(created_at, str) or not created_at.strip():
            raise _LegacyReceiptValidationError("invalid_created_at", "created_at must be a timezone-aware timestamp")
        try:
            parsed_time = datetime.fromisoformat(created_at.replace("Z", "+00:00"))
        except ValueError as exc:
            raise _LegacyReceiptValidationError("invalid_created_at", "created_at is not an ISO-8601 timestamp") from exc
        if parsed_time.tzinfo is None or parsed_time.utcoffset() is None:
            raise _LegacyReceiptValidationError("invalid_created_at", "created_at must include a timezone")
        operation = legacy.get("operation")
        if not isinstance(operation, str) or not operation.strip():
            raise _LegacyReceiptValidationError("invalid_operation", "operation must be a non-empty string")
        for field in ("requested_inputs", "effective_inputs", "result", "truth", "runtime_binding", "source_identity"):
            if not isinstance(legacy.get(field), Mapping):
                raise _LegacyReceiptValidationError(f"invalid_{field}", f"{field} must be an object")
        source_identity = legacy["source_identity"]
        if not source_identity:
            raise _LegacyReceiptValidationError("invalid_source_identity", "source_identity must not be empty")
        source_hashes = source_identity.get("source_sha256")
        if not isinstance(source_hashes, Mapping) or not source_hashes:
            raise _LegacyReceiptValidationError(
                "invalid_source_identity",
                "source_identity.source_sha256 must be a non-empty object",
            )
        for name, digest in source_hashes.items():
            if (
                not isinstance(name, str)
                or not name
                or not isinstance(digest, str)
                or len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
            ):
                raise _LegacyReceiptValidationError(
                    "invalid_source_identity",
                    "source_identity.source_sha256 entries must be lowercase SHA-256 values",
                )
        registry_sha256 = source_identity.get("registry_sha256")
        if (
            not isinstance(registry_sha256, str)
            or len(registry_sha256) != 64
            or any(character not in "0123456789abcdef" for character in registry_sha256)
        ):
            raise _LegacyReceiptValidationError(
                "invalid_source_identity",
                "source_identity.registry_sha256 must be a lowercase SHA-256 value",
            )
        if not isinstance(source_identity.get("evidence_authority"), Mapping):
            raise _LegacyReceiptValidationError(
                "invalid_source_identity",
                "source_identity.evidence_authority must be an object",
            )
        if not isinstance(source_identity.get("authority_verified"), bool):
            raise _LegacyReceiptValidationError(
                "invalid_source_identity",
                "source_identity.authority_verified must be boolean",
            )
        truth = legacy["truth"]
        truth_fields = (
            "delivery_verified",
            "controller_acknowledged",
            "completion_verified",
            "semantic_query_response_verified",
            "hardware_precondition_verified",
            "hardware_postcondition_verified",
            "physical_effect_verified",
            "physical_effect_claim_suppressed",
        )
        for field in truth_fields:
            if field == "semantic_query_response_verified" and field not in truth:
                continue
            if not isinstance(truth.get(field), bool):
                raise _LegacyReceiptValidationError("invalid_truth_type", f"truth.{field} must be boolean")
        result = legacy["result"]
        if not isinstance(result.get("ok"), bool):
            raise _LegacyReceiptValidationError("invalid_result_truth", "result.ok must be boolean")
        if not isinstance(result.get("outcome"), str) or not str(result["outcome"]).strip():
            raise _LegacyReceiptValidationError("invalid_result_outcome", "result.outcome must be a non-empty string")
        try:
            normalized = normalize_pipette_result(result)
        except (PipetteAuditIntegrityError, TypeError, ValueError, KeyError) as exc:
            raise _LegacyReceiptValidationError("invalid_result_contract", str(exc)) from exc
        return receipt_id, dict(normalized)

    def _validate_legacy_source(
        self,
        raw: bytes,
        *,
        migration_id: str,
        source_sha256: str,
    ) -> tuple[list[dict[str, Any]], bytes, int]:
        valid: list[dict[str, Any]] = []
        quarantine: list[dict[str, Any]] = []
        seen_receipt_ids: set[str] = set()
        source_count = 0
        byte_start = 0
        line_number = 0
        while byte_start < len(raw):
            byte_end = byte_start
            while byte_end < len(raw) and raw[byte_end] not in (10, 13):
                byte_end += 1
            if byte_end < len(raw):
                if raw[byte_end] == 13 and byte_end + 1 < len(raw) and raw[byte_end + 1] == 10:
                    byte_end += 2
                else:
                    byte_end += 1
            line_byte_start = byte_start
            raw_line = raw[line_byte_start:byte_end]
            byte_start = byte_end
            line_number += 1
            content = raw_line.rstrip(b"\r\n")
            if not content.strip():
                continue
            source_count += 1
            line_sha256 = hashlib.sha256(raw_line).hexdigest()
            try:
                try:
                    decoded = content.decode("utf-8")
                except UnicodeDecodeError as exc:
                    raise _LegacyReceiptValidationError("invalid_utf8", "line is not valid UTF-8") from exc
                try:
                    parsed = json.loads(decoded)
                except json.JSONDecodeError as exc:
                    raise _LegacyReceiptValidationError("invalid_json", "line is not valid JSON") from exc
                if not isinstance(parsed, Mapping):
                    raise _LegacyReceiptValidationError("receipt_not_object", "receipt line must be a JSON object")
                receipt_id, normalized = self._validate_legacy_receipt(parsed)
                if receipt_id in seen_receipt_ids:
                    raise _LegacyReceiptValidationError(
                        "duplicate_receipt_id",
                        "receipt_id is repeated within the legacy source",
                    )
                seen_receipt_ids.add(receipt_id)
                receipt = dict(parsed)
                receipt_truth = dict(receipt["truth"])
                receipt_truth.setdefault("semantic_query_response_verified", False)
                receipt["truth"] = receipt_truth
                valid.append(
                    {
                        "line_number": line_number,
                        "line_bytes": len(raw_line),
                        "byte_start": line_byte_start,
                        "byte_end": byte_end,
                        "line_sha256": line_sha256,
                        "receipt_id": receipt_id,
                        "receipt": receipt,
                        "normalized": normalized,
                    }
                )
            except _LegacyReceiptValidationError as exc:
                quarantine.append(
                    {
                        "schema": "bioxp.pipette.legacy-quarantine-line.v1",
                        "migration_id": migration_id,
                        "source_sha256": source_sha256,
                        "line_number": line_number,
                        "byte_start": line_byte_start,
                        "byte_end": byte_end,
                        "line_sha256": line_sha256,
                        "line_bytes": len(raw_line),
                        "raw_base64": base64.b64encode(raw_line).decode("ascii"),
                        "reason_code": exc.code,
                        "reason_detail": exc.detail,
                    }
                )
        quarantine_bytes = b"".join(
            (canonical_json(row) + "\n").encode("utf-8") for row in quarantine
        )
        return valid, quarantine_bytes, source_count

    def _attest_retired_v1_imports(
        self,
        raw: bytes,
        *,
        migration_id: str,
        source_sha256: str,
        source_count: int,
        imported_count: int,
        duplicate_count: int,
        quarantined_count: int,
        current_quarantine: bytes,
    ) -> int:
        """Attest a completed v1 import without rewriting its historical source."""
        if (
            migration_id != f"pipette-jsonl:{source_sha256}"
            or source_count <= 0
            or imported_count != source_count
            or duplicate_count != 0
            or quarantined_count != 0
        ):
            raise PipetteReceiptError("retired migration valid-row count mismatch")
        try:
            current_quarantine_rows = [
                json.loads(line) for line in current_quarantine.splitlines() if line.strip()
            ]
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
            raise PipetteReceiptError("retired migration quarantine evidence is invalid") from exc
        if not current_quarantine_rows or any(
            not isinstance(row, Mapping) or row.get("reason_code") != "invalid_result_outcome"
            for row in current_quarantine_rows
        ):
            raise PipetteReceiptError("retired migration valid-row count mismatch")

        archived: dict[str, Mapping[str, Any]] = {}
        for raw_line in raw.splitlines():
            if not raw_line.strip():
                continue
            try:
                row = json.loads(raw_line)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
                raise PipetteReceiptError("retired v1 archive is not valid JSONL") from exc
            if not isinstance(row, Mapping):
                raise PipetteReceiptError("retired v1 archive row is not an object")
            receipt_id = row.get("receipt_id")
            operation = row.get("operation")
            requested_inputs = row.get("requested_inputs")
            result = row.get("result")
            if (
                not isinstance(receipt_id, str)
                or not receipt_id
                or receipt_id in archived
                or not isinstance(operation, str)
                or not operation.strip()
                or not isinstance(requested_inputs, Mapping)
                or not isinstance(result, Mapping)
                or not isinstance(result.get("ok"), bool)
            ):
                raise PipetteReceiptError("retired v1 archive row identity is invalid")
            archived[receipt_id] = row
        if len(archived) != source_count:
            raise PipetteReceiptError("retired v1 archive source count mismatch")

        expected_ids = set(archived)
        observed_ids: set[str] = set()
        for receipt_id, legacy in archived.items():
            command_id = f"legacy.pipette.{receipt_id}"
            prior = self.connection.execute(
                """
                SELECT c.idempotency_key,c.action_id,c.operation,c.entrypoint_id,
                       c.caller_class,c.control_class,c.requested_inputs_json,
                       c.source_identity_json AS command_source_identity_json,
                       p.pipette_operation_id,p.operation AS pipette_operation,
                       p.entrypoint_id AS pipette_entrypoint_id,p.caller_class AS pipette_caller_class,
                       p.control_class AS pipette_control_class,p.status,p.outcome,
                       p.source_identity_json AS pipette_source_identity_json
                FROM operator_commands AS c
                JOIN pipette_operations AS p ON p.command_id=c.command_id
                WHERE c.command_id=?
                """,
                (command_id,),
            ).fetchone()
            expected_source = {
                "legacy_receipt_id": receipt_id,
                "source_sha256": source_sha256,
            }
            if (
                prior is None
                or str(prior["idempotency_key"]) != f"legacy-pipette:{receipt_id}"
                or str(prior["action_id"]) != f"pipette.{legacy['operation']}"
                or str(prior["operation"]) != str(legacy["operation"])
                or str(prior["pipette_operation"]) != str(legacy["operation"])
                or str(prior["entrypoint_id"]) != "migration.pipette_jsonl.v1"
                or str(prior["pipette_entrypoint_id"]) != "migration.pipette_jsonl.v1"
                or str(prior["caller_class"]) != "legacy_migration"
                or str(prior["pipette_caller_class"]) != "legacy_migration"
                or str(prior["control_class"]) != "historical_import"
                or str(prior["pipette_control_class"]) != "historical_import"
                or str(prior["pipette_operation_id"]) != command_id
                or str(prior["requested_inputs_json"])
                != canonical_json(dict(legacy["requested_inputs"]))
                or json.loads(str(prior["command_source_identity_json"])) != expected_source
                or json.loads(str(prior["pipette_source_identity_json"])) != expected_source
                or str(prior["status"]) in NONTERMINAL_COMMAND_STATES
                or not str(prior["outcome"] or "").strip()
            ):
                raise PipetteReceiptError("retired v1 import binding conflicts with its archive")
            observed_ids.add(receipt_id)

        rows = self.connection.execute(
            """
            SELECT c.source_identity_json,p.source_identity_json
            FROM operator_commands AS c
            JOIN pipette_operations AS p ON p.command_id=c.command_id
            WHERE c.entrypoint_id='migration.pipette_jsonl.v1'
              AND p.entrypoint_id='migration.pipette_jsonl.v1'
            """
        ).fetchall()
        source_bound_ids: set[str] = set()
        for row in rows:
            command_source = json.loads(str(row[0]))
            pipette_source = json.loads(str(row[1]))
            if command_source != pipette_source:
                raise PipetteReceiptError("retired v1 import source identities disagree")
            if command_source.get("source_sha256") == source_sha256:
                receipt_id = command_source.get("legacy_receipt_id")
                if not isinstance(receipt_id, str) or not receipt_id:
                    raise PipetteReceiptError("retired v1 import source identity is invalid")
                source_bound_ids.add(receipt_id)
        if observed_ids != expected_ids or source_bound_ids != expected_ids:
            raise PipetteReceiptError("retired v1 import inventory conflicts with its archive")
        return len(observed_ids)

    @staticmethod
    def _verify_retired_v1_backup(
        backup_root: Path,
        *,
        source_sha256: str,
        archive_raw: bytes,
    ) -> dict[str, Any]:
        if backup_root.is_symlink() or not backup_root.is_dir():
            raise PipetteReceiptError("retired v1 migration backup is unavailable")
        allowed = {
            "SHA256SUMS",
            "bioxp_runtime.db",
            "bioxp_runtime.db-shm",
            "bioxp_runtime.db-wal",
            "receipts.jsonl",
        }
        observed = {path.name for path in backup_root.iterdir()}
        required = {"SHA256SUMS", "bioxp_runtime.db", "receipts.jsonl"}
        if not required.issubset(observed) or not observed.issubset(allowed):
            raise PipetteReceiptError("retired v1 migration backup inventory is invalid")
        for path in backup_root.iterdir():
            if path.is_symlink() or not path.is_file():
                raise PipetteReceiptError("retired v1 migration backup contains an invalid file")

        sums_raw = (backup_root / "SHA256SUMS").read_bytes()
        listed: dict[str, str] = {}
        try:
            lines = sums_raw.decode("utf-8").splitlines()
        except UnicodeDecodeError as exc:
            raise PipetteReceiptError("retired v1 migration checksum file is invalid") from exc
        for line in lines:
            try:
                digest, selected = line.split("  ", 1)
            except ValueError as exc:
                raise PipetteReceiptError("retired v1 migration checksum row is invalid") from exc
            selected_path = Path(selected)
            if (
                len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
                or not selected_path.is_absolute()
                or selected_path.parent != backup_root
                or selected_path.name not in {"bioxp_runtime.db", "receipts.jsonl"}
                or selected_path.name in listed
            ):
                raise PipetteReceiptError("retired v1 migration checksum binding is invalid")
            raw = (backup_root / selected_path.name).read_bytes()
            if hashlib.sha256(raw).hexdigest() != digest:
                raise PipetteReceiptError("retired v1 migration backup digest mismatch")
            listed[selected_path.name] = digest
        if set(listed) != {"bioxp_runtime.db", "receipts.jsonl"}:
            raise PipetteReceiptError("retired v1 migration checksum closure is incomplete")
        if (
            listed["receipts.jsonl"] != source_sha256
            or (backup_root / "receipts.jsonl").read_bytes() != archive_raw
        ):
            raise PipetteReceiptError("retired v1 migration backup source binding is invalid")
        wal = backup_root / "bioxp_runtime.db-wal"
        if wal.exists() and wal.stat().st_size != 0:
            raise PipetteReceiptError("retired v1 migration backup has unsealed WAL state")
        database = backup_root / "bioxp_runtime.db"
        with sqlite3.connect(f"file:{database}?mode=ro&immutable=1", uri=True) as connection:
            if connection.execute("PRAGMA quick_check").fetchone()[0] != "ok":
                raise PipetteReceiptError("retired v1 migration backup database is invalid")
        return {
            "status": "verified_retired_v1",
            "backup_id": backup_root.name,
            "sha256sums_sha256": hashlib.sha256(sums_raw).hexdigest(),
        }

    def _retirement_authority(
        self,
        *,
        migration_id: str,
        source_sha256: str,
        archive_relpath: str,
    ) -> None:
        self.connection.execute("BEGIN IMMEDIATE")
        try:
            migration = self.connection.execute(
                "SELECT source_digest,archive_relpath,status FROM runtime_migration_receipts WHERE migration_id=?",
                (migration_id,),
            ).fetchone()
            if migration is None or str(migration["source_digest"]) != source_sha256:
                raise PipetteReceiptError("durable migration authority is missing or mismatched")
            if str(migration["archive_relpath"] or "") != archive_relpath:
                raise PipetteReceiptError("durable migration archive binding is mismatched")
            if str(migration["status"]) not in {"completed", "completed_with_quarantine"}:
                raise PipetteReceiptError("durable migration is not complete")
            retirement = self.connection.execute(
                "SELECT source_digest,archive_relpath,retirement_sha256 FROM runtime_migration_retirements WHERE migration_id=?",
                (migration_id,),
            ).fetchone()
            if retirement is None:
                self.connection.execute(
                    """
                    INSERT INTO runtime_migration_retirements(
                        migration_id,source_digest,archive_relpath,retired_at,retirement_sha256
                    ) VALUES(?,?,?,?,?)
                    """,
                    (migration_id, source_sha256, archive_relpath, time.time(), source_sha256),
                )
            elif (
                str(retirement["source_digest"]) != source_sha256
                or str(retirement["archive_relpath"]) != archive_relpath
                or str(retirement["retirement_sha256"]) != source_sha256
            ):
                raise PipetteReceiptError("retirement authority conflicts with the legacy source")
            self.connection.execute("COMMIT")
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

    @_migration_file_lock
    def attest_first_install_absence(self) -> None:
        if self._legacy_path.exists():
            return
        source_kind = "pipette_receipts_jsonl_first_install_absence"
        existing = self.connection.execute(
            "SELECT 1 FROM runtime_migration_receipts WHERE source_kind=? LIMIT 1",
            (source_kind,),
        ).fetchone()
        if existing is not None:
            return
        release_identity = current_release_identity()
        archive_raw = canonical_json({
            "schema": "bioxp.pipette.legacy_source_absence.v1",
            "source_kind": source_kind,
            "release_identity": release_identity,
        }).encode("utf-8")
        digest = hashlib.sha256(archive_raw).hexdigest()
        migration_id = f"pipette-jsonl-absence:{digest}"
        archive_relpath = f"archive/pipette-receipts-jsonl-absence.{digest}.json"
        self._write_immutable_migration_artifact(
            self._migration_artifact_path(archive_relpath),
            archive_raw,
            expected_sha256=digest,
        )
        backup = create_backup_unit(
            self.root,
            label=f"pipette-audit-first-install-{digest[:16]}",
            phase="pre_migration",
            source_kind=source_kind,
            source_digest=digest,
        )
        backup_relpath = str(backup["unit_relpath"])
        now = time.time()
        self.connection.execute("BEGIN IMMEDIATE")
        try:
            self.connection.execute(
                """
                INSERT INTO runtime_migration_receipts(
                    migration_id,source_kind,source_digest,source_count,imported_count,
                    duplicate_count,quarantined_count,status,archive_relpath,retired_at,
                    retirement_sha256,created_at
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (migration_id, source_kind, digest, 0, 0, 0, 0, "completed",
                 archive_relpath, now, digest, now),
            )
            self.connection.execute(
                """
                INSERT INTO runtime_migration_retirements(
                    migration_id,source_digest,archive_relpath,retired_at,retirement_sha256
                ) VALUES(?,?,?,?,?)
                """,
                (migration_id, digest, archive_relpath, now, digest),
            )
            self._ensure_migration_evidence(
                migration_id=migration_id,
                source_path=str(self._legacy_path),
                source_digest=digest,
                source_bytes=len(archive_raw),
                source_count=0,
                imported_count=0,
                duplicate_count=0,
                quarantined_count=0,
                backup_relpath=backup_relpath,
                archive_relpath=archive_relpath,
            )
            self.connection.execute("COMMIT")
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise

    def _first_install_absence_attestation(self, release_identity: Mapping[str, Any]) -> dict[str, Any]:
        source_kind = "pipette_receipts_jsonl_first_install_absence"
        existing = self.connection.execute(
            "SELECT * FROM runtime_migration_receipts WHERE source_kind=? ORDER BY created_at DESC LIMIT 1",
            (source_kind,),
        ).fetchone()
        if existing is None:
            raise PipetteReceiptError(
                "first-install source absence is not attested by the canonical migration authority"
            )
        if existing is None or str(existing["status"]) != "completed":
            raise PipetteReceiptError("first-install source-absence attestation is incomplete")
        migration_id = str(existing["migration_id"])
        digest = str(existing["source_digest"])
        archive_relpath = str(existing["archive_relpath"] or "")
        retirement = self.connection.execute(
            "SELECT * FROM runtime_migration_retirements WHERE migration_id=?", (migration_id,)
        ).fetchone()
        evidence = self.connection.execute(
            "SELECT * FROM runtime_migration_evidence WHERE migration_id=?", (migration_id,)
        ).fetchone()
        if retirement is None or evidence is None or not archive_relpath:
            raise PipetteReceiptError("first-install source-absence evidence closure is incomplete")
        archive = self._migration_artifact_path(archive_relpath)
        if archive.is_symlink() or not archive.is_file():
            raise PipetteReceiptError("first-install source-absence archive identity mismatch")
        archive_raw = archive.read_bytes()
        if hashlib.sha256(archive_raw).hexdigest() != digest:
            raise PipetteReceiptError("first-install source-absence archive identity mismatch")
        try:
            archive_payload = json.loads(archive_raw)
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
            raise PipetteReceiptError("first-install source-absence archive payload is invalid") from exc
        if (
            not isinstance(archive_payload, Mapping)
            or archive_payload.get("schema") != "bioxp.pipette.legacy_source_absence.v1"
            or archive_payload.get("source_kind") != source_kind
            or not isinstance(archive_payload.get("release_identity"), Mapping)
        ):
            raise PipetteReceiptError("first-install source-absence archive payload is invalid")
        attested_release_identity = dict(archive_payload["release_identity"])
        if any(str(retirement[field]) != expected for field, expected in (
            ("source_digest", digest), ("archive_relpath", archive_relpath), ("retirement_sha256", digest)
        )):
            raise PipetteReceiptError("first-install source-absence retirement identity mismatch")
        backup_relpath = self._migration_backup_relpath(str(evidence["backup_relpath"] or ""))
        if backup_relpath is None:
            raise PipetteReceiptError("first-install source-absence backup binding is missing")
        verify_backup_unit(self.root / backup_relpath)
        return {
            "status": "first_install_source_absence_attested",
            "migration_id": migration_id,
            "source_path": str(self._legacy_path),
            "source_sha256": digest,
            "archive_relpath": archive_relpath,
            "pre_migration_backup": {"unit_relpath": backup_relpath},
            "imported_count": 0,
            "release_identity": attested_release_identity,
            "current_release_identity": dict(release_identity),
            "release_identity_matches_current": attested_release_identity == dict(release_identity),
        }

    @_migration_file_lock
    def migrate_legacy_jsonl(self) -> dict[str, Any]:
        with self._lock:
            release_identity = current_release_identity()
            latest = self.connection.execute(
                """
                SELECT migration_id,source_digest,source_count,imported_count,
                       duplicate_count,quarantined_count,status,archive_relpath
                FROM runtime_migration_receipts
                WHERE source_kind=? ORDER BY created_at DESC LIMIT 1
                """,
                ("pipette_receipts_jsonl",),
            ).fetchone()
            if not self._legacy_path.exists():
                if latest is None:
                    return self._first_install_absence_attestation(release_identity)
                if str(latest["status"]) not in {"completed", "completed_with_quarantine"}:
                    raise PipetteReceiptError("legacy source is absent but migration is not complete")
                retirement = self.connection.execute(
                    "SELECT archive_relpath,source_digest,retirement_sha256 FROM runtime_migration_retirements WHERE migration_id=?",
                    (latest["migration_id"],),
                ).fetchone()
                if retirement is None:
                    raise PipetteReceiptError("legacy source is missing before durable retirement authority")
                archive_relpath = str(retirement["archive_relpath"])
                archive_path = self._migration_artifact_path(archive_relpath)
                if archive_path.is_symlink() or not archive_path.is_file():
                    raise PipetteReceiptError("retired migration archive is missing")
                archive_raw = archive_path.read_bytes()
                source_sha256 = hashlib.sha256(archive_raw).hexdigest()
                if (
                    source_sha256 != str(latest["source_digest"])
                    or source_sha256 != str(retirement["source_digest"])
                    or source_sha256 != str(retirement["retirement_sha256"])
                ):
                    raise PipetteReceiptError("retired migration archive digest mismatch")
                valid, quarantine_bytes, source_count = self._validate_legacy_source(
                    archive_raw,
                    migration_id=str(latest["migration_id"]),
                    source_sha256=source_sha256,
                )
                if source_count != int(latest["source_count"]):
                    raise PipetteReceiptError("retired migration source count mismatch")
                validated_count = len(valid)
                retired_v1_compatible = False
                if validated_count != int(latest["imported_count"]) + int(latest["duplicate_count"]):
                    validated_count = self._attest_retired_v1_imports(
                        archive_raw,
                        migration_id=str(latest["migration_id"]),
                        source_sha256=source_sha256,
                        source_count=source_count,
                        imported_count=int(latest["imported_count"]),
                        duplicate_count=int(latest["duplicate_count"]),
                        quarantined_count=int(latest["quarantined_count"]),
                        current_quarantine=quarantine_bytes,
                    )
                    quarantine_bytes = b""
                    retired_v1_compatible = True
                if quarantine_bytes.count(b"\n") != int(latest["quarantined_count"]):
                    raise PipetteReceiptError("retired migration quarantine state mismatch")
                quarantine_relpath = None
                quarantine_sha256 = None
                if quarantine_bytes:
                    quarantine_sha256 = hashlib.sha256(quarantine_bytes).hexdigest()
                    quarantine_relpath = (
                        f"quarantine/pipette-receipts-jsonl/{source_sha256}.{quarantine_sha256}.jsonl"
                    )
                    self._write_immutable_migration_artifact(
                        self._migration_artifact_path(quarantine_relpath),
                        quarantine_bytes,
                        expected_sha256=quarantine_sha256,
                    )
                migration_evidence = self.connection.execute(
                    "SELECT * FROM runtime_migration_evidence WHERE migration_id=?",
                    (str(latest["migration_id"]),),
                ).fetchone()
                if migration_evidence is None:
                    raise PipetteReceiptError("completed migration is missing immutable evidence")
                backup_relpath = self._migration_backup_relpath(
                    None
                    if migration_evidence["backup_relpath"] is None
                    else str(migration_evidence["backup_relpath"])
                )
                if backup_relpath is None:
                    raise PipetteReceiptError("completed migration is missing its bound backup")
                verified_backup = (
                    self._verify_retired_v1_backup(
                        self.root / backup_relpath,
                        source_sha256=source_sha256,
                        archive_raw=archive_raw,
                    )
                    if retired_v1_compatible
                    else verify_backup_unit(self.root / backup_relpath)
                )
                if str(verified_backup.get("backup_id") or "") != Path(backup_relpath).name:
                    raise PipetteReceiptError("migration backup identity mismatch")
                self._ensure_migration_evidence(
                    migration_id=str(latest["migration_id"]),
                    source_path=str(self._legacy_path),
                    source_digest=source_sha256,
                    source_bytes=len(archive_raw),
                    source_count=source_count,
                    imported_count=int(latest["imported_count"]),
                    duplicate_count=int(latest["duplicate_count"]),
                    quarantined_count=int(latest["quarantined_count"]),
                    backup_relpath=backup_relpath,
                    archive_relpath=archive_relpath,
                )
                return {
                    "status": "already_imported",
                    "source_path": str(self._legacy_path),
                    "source_sha256": source_sha256,
                    "source_bytes": len(archive_raw),
                    "source_count": source_count,
                    "imported_count": 0,
                    "duplicate_count": validated_count,
                    "quarantined_count": int(latest["quarantined_count"]),
                    "quarantine_relpath": quarantine_relpath,
                    "quarantine_sha256": quarantine_sha256,
                    "archive_relpath": archive_relpath,
                    "pre_migration_backup": {
                        "status": "source_verified",
                        "unit_relpath": backup_relpath,
                        "backup_id": verified_backup["backup_id"],
                    },
                    "release_identity": release_identity,
                }

            if self._legacy_path.is_symlink() or not self._legacy_path.is_file():
                raise PipetteReceiptError("legacy JSONL source is not a regular file")
            raw = self._legacy_path.read_bytes()
            source_sha256 = hashlib.sha256(raw).hexdigest()
            migration_id = f"pipette-jsonl:{source_sha256}"
            valid, quarantine_bytes, source_count = self._validate_legacy_source(
                raw,
                migration_id=migration_id,
                source_sha256=source_sha256,
            )
            quarantined_count = source_count - len(valid)
            if latest is not None and str(latest["source_digest"]) != source_sha256:
                raise PipetteReceiptError("legacy JSONL digest conflicts with prior migration receipt")

            existing_evidence = self.connection.execute(
                "SELECT backup_relpath FROM runtime_migration_evidence WHERE migration_id=?",
                (migration_id,),
            ).fetchone()
            if existing_evidence is not None:
                backup_relpath = self._migration_backup_relpath(
                    None
                    if existing_evidence["backup_relpath"] is None
                    else str(existing_evidence["backup_relpath"])
                )
                if backup_relpath is None:
                    raise PipetteReceiptError("existing migration evidence is missing its bound backup")
                verified_backup = verify_backup_unit(self.root / backup_relpath)
                if str(verified_backup.get("backup_id") or "") != Path(backup_relpath).name:
                    raise PipetteReceiptError("existing migration backup identity mismatch")
                pre_migration_backup = {
                    "status": "source_verified",
                    "unit_relpath": backup_relpath,
                    "backup_id": verified_backup["backup_id"],
                }
            else:
                pre_migration_backup = create_backup_unit(
                    self.root,
                    label=f"pipette-audit-pre-migration-{source_sha256[:16]}",
                    phase="pre_migration",
                    source_kind="pipette_receipts_jsonl",
                    source_digest=source_sha256,
                )
                backup_relpath = str(pre_migration_backup["unit_relpath"])
            archive_relpath = (
                str(latest["archive_relpath"])
                if latest is not None and latest["archive_relpath"]
                else f"archive/receipts.jsonl.{source_sha256}"
            )
            archive_path = self._migration_artifact_path(archive_relpath)
            self._write_immutable_migration_artifact(
                archive_path,
                raw,
                expected_sha256=source_sha256,
            )
            quarantine_relpath = None
            quarantine_sha256 = None
            if quarantine_bytes:
                quarantine_sha256 = hashlib.sha256(quarantine_bytes).hexdigest()
                quarantine_relpath = (
                    f"quarantine/pipette-receipts-jsonl/{source_sha256}.{quarantine_sha256}.jsonl"
                )
                self._write_immutable_migration_artifact(
                    self._migration_artifact_path(quarantine_relpath),
                    quarantine_bytes,
                    expected_sha256=quarantine_sha256,
                )

            if (
                self._legacy_path.is_symlink()
                or not self._legacy_path.is_file()
                or self._legacy_path.read_bytes() != raw
            ):
                raise PipetteReceiptError("legacy JSONL changed before database mutation")

            imported = 0
            duplicate_count = 0
            evidence_duplicate_count = 0
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                marker = self.connection.execute(
                    "SELECT * FROM runtime_migration_receipts WHERE migration_id=?",
                    (migration_id,),
                ).fetchone()
                for item in valid:
                    legacy = item["receipt"]
                    receipt_id = str(item["receipt_id"])
                    operation = str(legacy["operation"])
                    command_id = f"legacy.pipette.{receipt_id}"
                    claim_entrypoint_id = "migration.pipette_jsonl.v2"
                    source_identity = {
                        "legacy_receipt_id": receipt_id,
                        "legacy_source_sha256": source_sha256,
                        "legacy_line_number": int(item["line_number"]),
                        "legacy_line_bytes": int(item["line_bytes"]),
                        "legacy_byte_start": int(item["byte_start"]),
                        "legacy_byte_end": int(item["byte_end"]),
                        "legacy_line_sha256": str(item["line_sha256"]),
                        "receipt_source_identity": dict(legacy["source_identity"]),
                        "runtime_binding": dict(legacy["runtime_binding"]),
                        "release_identity": release_identity,
                    }
                    prior = self.connection.execute(
                        """
                        SELECT c.idempotency_key,c.operation,c.entrypoint_id,c.requested_inputs_json,
                               c.source_identity_json AS command_source_identity_json,
                               p.pipette_operation_id,p.operation AS pipette_operation,
                               p.entrypoint_id AS pipette_entrypoint_id,p.status,
                               p.source_identity_json AS pipette_source_identity_json
                        FROM operator_commands AS c
                        JOIN pipette_operations AS p ON p.command_id=c.command_id
                        WHERE c.command_id=?
                        """,
                        (command_id,),
                    ).fetchone()
                    if marker is not None and prior is not None:
                        prior_command_source = json.loads(str(prior["command_source_identity_json"]))
                        prior_pipette_source = json.loads(str(prior["pipette_source_identity_json"]))
                        prior_entrypoint = str(prior["entrypoint_id"])
                        if (
                            str(prior["idempotency_key"]) != f"legacy-pipette:{receipt_id}"
                            or str(prior["operation"]) != operation
                            or str(prior["pipette_operation"]) != operation
                            or str(prior["pipette_entrypoint_id"]) != prior_entrypoint
                            or prior_entrypoint
                            not in {"migration.pipette_jsonl.v1", "migration.pipette_jsonl.v2"}
                            or str(prior["requested_inputs_json"])
                            != canonical_json(dict(legacy["requested_inputs"]))
                        ):
                            raise PipetteReceiptError("legacy duplicate binding conflicts with validated source")
                        if (
                            "source_sha256" in prior_pipette_source
                            and "legacy_source_sha256" not in prior_pipette_source
                        ):
                            v1_source_identity = {
                                "legacy_receipt_id": receipt_id,
                                "source_sha256": source_sha256,
                            }
                            if (
                                prior_command_source != v1_source_identity
                                or prior_pipette_source != v1_source_identity
                                or prior_entrypoint != "migration.pipette_jsonl.v1"
                            ):
                                raise PipetteReceiptError("legacy v1 import binding conflicts with validated source")
                            v1_incomplete = str(prior["status"]) in NONTERMINAL_COMMAND_STATES
                            upgraded_source = canonical_json(source_identity)
                            command_update = self.connection.execute(
                                "UPDATE operator_commands SET source_identity_json=?,updated_at=? WHERE command_id=? AND source_identity_json=?",
                                (upgraded_source, time.time(), command_id, canonical_json(v1_source_identity)),
                            )
                            operation_update = self.connection.execute(
                                "UPDATE pipette_operations SET source_identity_json=?,updated_at=? WHERE command_id=? AND source_identity_json=?",
                                (upgraded_source, time.time(), command_id, canonical_json(v1_source_identity)),
                            )
                            if command_update.rowcount != 1 or operation_update.rowcount != 1:
                                raise PipetteReceiptError("legacy v1 import binding could not be durably upgraded")
                            upgraded = self.connection.execute(
                                """
                                SELECT c.source_identity_json AS command_source_identity_json,
                                       p.source_identity_json AS pipette_source_identity_json
                                FROM operator_commands AS c
                                JOIN pipette_operations AS p ON p.command_id=c.command_id
                                WHERE c.command_id=?
                                """,
                                (command_id,),
                            ).fetchone()
                            if (
                                upgraded is None
                                or json.loads(str(upgraded["command_source_identity_json"])) != source_identity
                                or json.loads(str(upgraded["pipette_source_identity_json"])) != source_identity
                            ):
                                raise PipetteReceiptError("legacy v1 import binding upgrade was not durable")
                            prior_command_source = source_identity
                            prior_pipette_source = source_identity
                            if not v1_incomplete:
                                duplicate_count += 1
                                evidence_duplicate_count += 1
                                continue
                            claim_entrypoint_id = "migration.pipette_jsonl.v1"
                        if prior_command_source != source_identity or prior_pipette_source != source_identity:
                            raise PipetteReceiptError("legacy duplicate line binding conflicts with validated source")
                        if str(prior["status"]) not in NONTERMINAL_COMMAND_STATES:
                            duplicate_count += 1
                            if prior_entrypoint == "migration.pipette_jsonl.v1":
                                evidence_duplicate_count += 1
                            continue
                    pipette_operation_id = f"legacy.pipette.operation.{receipt_id}"
                    claim, created = self._audit_database.claim(
                        {
                            "command_id": command_id,
                            "pipette_operation_id": pipette_operation_id,
                            "idempotency_key": f"legacy-pipette:{receipt_id}",
                            "action_id": f"pipette.{operation}",
                            "operation": operation,
                            "entrypoint_id": claim_entrypoint_id,
                            "caller_class": "legacy_migration",
                            "control_class": "historical_import",
                            "ownership_generation": 0,
                            "connection_generation": 0,
                            "source_identity": source_identity,
                            "requested_inputs": dict(legacy["requested_inputs"]),
                        },
                        pipette=True,
                    )
                    operation_row = self.connection.execute(
                        "SELECT status,source_identity_json FROM pipette_operations WHERE pipette_operation_id=? AND command_id=?",
                        (claim["pipette_operation_id"], claim["command_id"]),
                    ).fetchone()
                    if operation_row is None:
                        raise PipetteReceiptError("legacy pipette claim child is missing")
                    persisted_source = json.loads(str(operation_row["source_identity_json"]))
                    if persisted_source != source_identity:
                        raise PipetteReceiptError("legacy pipette claim source binding conflicts")
                    if not created and str(operation_row["status"]) not in NONTERMINAL_COMMAND_STATES:
                        duplicate_count += 1
                        continue
                    result = {
                        **dict(legacy["result"]),
                        **dict(legacy["truth"]),
                        "legacy_receipt_id": receipt_id,
                        "legacy_source_sha256": source_sha256,
                    }
                    self._persist_normalized_result(
                        command_id=str(claim["command_id"]),
                        pipette_operation_id=str(claim["pipette_operation_id"]),
                        result=result,
                        normalized=item["normalized"],
                    )
                    truth = legacy["truth"]
                    status = (
                        "observed"
                        if truth["semantic_query_response_verified"]
                        else "completed"
                        if truth["completion_verified"]
                        else "acknowledged"
                        if truth["controller_acknowledged"]
                        else "dispatched"
                        if truth["delivery_verified"]
                        else "failed"
                    )
                    self._audit_database.finalize_claim(
                        command_id=str(claim["command_id"]),
                        pipette_operation_id=str(claim["pipette_operation_id"]),
                        expected_status=str(claim["status"]),
                        status=status,
                        outcome=str(legacy["result"]["outcome"]),
                        failure_code=None
                        if legacy["result"]["ok"] is True
                        else str(legacy["result"].get("failure_code") or legacy["result"].get("error") or "legacy_pipette_failed"),
                        result=result,
                        effective_inputs=dict(legacy["effective_inputs"]),
                        receipt_json=canonical_json(legacy),
                    )
                    imported += 1

                if marker is None:
                    status = "completed_with_quarantine" if quarantined_count else "completed"
                    self.connection.execute(
                        """
                        INSERT INTO runtime_migration_receipts(
                            migration_id,source_kind,source_digest,source_count,imported_count,
                            duplicate_count,quarantined_count,status,archive_relpath,created_at
                        ) VALUES(?,?,?,?,?,?,?,?,?,?)
                        """,
                        (
                            migration_id,
                            "pipette_receipts_jsonl",
                            source_sha256,
                            source_count,
                            imported,
                            evidence_duplicate_count,
                            quarantined_count,
                            status,
                            archive_relpath,
                            time.time(),
                        ),
                    )
                else:
                    if (
                        str(marker["source_digest"]) != source_sha256
                        or int(marker["source_count"]) != source_count
                        or int(marker["imported_count"]) + int(marker["duplicate_count"]) != len(valid)
                        or int(marker["quarantined_count"]) != quarantined_count
                        or str(marker["archive_relpath"] or "") != archive_relpath
                        or str(marker["status"]) not in {"completed", "completed_with_quarantine"}
                    ):
                        raise PipetteReceiptError("legacy migration marker conflicts with validated source")
                self._ensure_migration_evidence(
                    migration_id=migration_id,
                    source_path=str(self._legacy_path),
                    source_digest=source_sha256,
                    source_bytes=len(raw),
                    source_count=source_count,
                    imported_count=len(valid),
                    duplicate_count=evidence_duplicate_count,
                    quarantined_count=quarantined_count,
                    backup_relpath=backup_relpath,
                    archive_relpath=archive_relpath,
                )
                self.connection.execute("COMMIT")
            except Exception:
                if self.connection.in_transaction:
                    self.connection.execute("ROLLBACK")
                raise

            self._retirement_authority(
                migration_id=migration_id,
                source_sha256=source_sha256,
                archive_relpath=archive_relpath,
            )
            if self._legacy_path.exists():
                if self._legacy_path.is_symlink() or self._legacy_path.read_bytes() != raw:
                    raise PipetteReceiptError("legacy JSONL changed before retirement")
                self._legacy_path.unlink()
                self._fsync_directory(self._legacy_path.parent)
            return {
                "status": "completed_with_quarantine" if quarantined_count else "completed",
                "source_path": str(self._legacy_path),
                "source_sha256": source_sha256,
                "source_bytes": len(raw),
                "source_count": source_count,
                "imported_count": imported,
                "duplicate_count": duplicate_count,
                "quarantined_count": quarantined_count,
                "quarantine_relpath": quarantine_relpath,
                "quarantine_sha256": quarantine_sha256,
                "archive_relpath": archive_relpath,
                "pre_migration_backup": pre_migration_backup["unit_relpath"],
                "release_identity": release_identity,
            }

    def read(self, limit: int = 50) -> list[dict[str, Any]]:
        selected_limit = max(1, min(int(limit), 200))
        rows = self.connection.execute(
            "SELECT receipt_json FROM pipette_operations WHERE receipt_json IS NOT NULL AND receipt_json <> '{}' ORDER BY updated_at DESC LIMIT ?",
            (selected_limit,),
        ).fetchall()
        if rows:
            return [json.loads(row["receipt_json"]) for row in rows]
        if self._legacy_path.exists():
            raise PipetteReceiptError("active pipette JSONL requires JSONL migration")
        return []

    def latest(self) -> dict[str, Any] | None:
        rows = self.read(1)
        return rows[0] if rows else None
