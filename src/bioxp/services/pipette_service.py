from __future__ import annotations

import asyncio
import hashlib
import json
import time
import random
import threading
from dataclasses import dataclass
from collections.abc import Mapping
from contextvars import ContextVar, Token
from typing import Any, Awaitable, Callable, cast
from uuid import uuid4

from fastapi import HTTPException
from ..hardware_status import hardware_state
from ..pipette.models import (
    PipetteAspirateCommand,
    PipetteDispenseCommand,
    PipetteError,
    PipetteInitCommand,
    PipetteMixCommand,
    PipetteTipCommand,
    PipetteValidationError,
)
from ..pipette.receipts import (
    _LINKED_FINALIZATION_KEY,
    PipetteReceiptError,
    PipetteReceiptStore,
)
from ..pipette.transport import PipetteTransport


READ_ONLY_PIPETTE_OPERATIONS = frozenset({
    "live_readback",
    "error_log",
    "tip_status",
    "data",
    "fluid_timestamp",
    "pressure",
    "firmware",
    "condition",
    "checked_status",
    "status_readback",
    "status",
    "readback",
    "query_status",
    "query_pressure",
    "read_pressure",
    "query_tip_status",
    "query_all_pipette_tip_states",
    "query_error_log",
    "get_data",
    "get_all_data",
})

_DIRECT_PIPETTE_IDEMPOTENCY: ContextVar[str | None] = ContextVar(
    "direct_pipette_idempotency",
    default=None,
)


def set_direct_pipette_idempotency_key(value: str) -> Token[str | None]:
    return _DIRECT_PIPETTE_IDEMPOTENCY.set(str(value))


def reset_direct_pipette_idempotency_key(token: Token[str | None]) -> None:
    _DIRECT_PIPETTE_IDEMPOTENCY.reset(token)

_QUERY_NONCLAIM_FIELDS = frozenset({
    "tx_ok",
    "delivery_verified",
    "ack_received",
    "immediate_ack_received",
    "controller_acknowledged",
    "completion_received",
    "completion_verified",
    "completion_deferred",
    "hardware_postcondition_verified",
    "physical_effect_verified",
    "semantic_query_response_verified",
})


def _query_result_without_mutation_claims(value: Any) -> Any:
    if isinstance(value, Mapping):
        sanitized = {
            str(key): (
                False
                if str(key) in _QUERY_NONCLAIM_FIELDS
                else _query_result_without_mutation_claims(item)
            )
            for key, item in value.items()
        }
        ack = sanitized.get("ack")
        if isinstance(ack, dict):
            ack["ok"] = False
            ack["received"] = False
            if ack.get("outcome") in {"ack", "completion", "multipart_completion"}:
                ack["outcome"] = "query_response"
        return sanitized
    if isinstance(value, list):
        return [_query_result_without_mutation_claims(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_query_result_without_mutation_claims(item) for item in value)
    return value


def _semantic_query_correlation(value: Any) -> tuple[int, bool]:
    """Count command-correlated semantic query leaves without inferring ACK truth."""
    if isinstance(value, Mapping):
        if "query_response_correlated" in value:
            return 1, (
                value.get("query_response_correlated") is True
                and value.get("semantic_ok") is True
            )
        count = 0
        valid = True
        for item in value.values():
            child_count, child_valid = _semantic_query_correlation(item)
            count += child_count
            valid = valid and child_valid
        return count, valid
    if isinstance(value, (list, tuple)):
        count = 0
        valid = True
        for item in value:
            child_count, child_valid = _semantic_query_correlation(item)
            count += child_count
            valid = valid and child_valid
        return count, valid
    return 0, True

BlockingRunner = Callable[..., Awaitable[dict[str, Any]]]
TransportGetter = Callable[[], Any]
PipettePreflight = Callable[[str, Any], dict[str, Any]]


def _pipette_error_to_http_exception(exc: PipetteError) -> HTTPException:
    return HTTPException(status_code=exc.status_code, detail=exc.to_payload())


def _validate_oem_admission(operation_name: str | None, command: Any | None) -> None:
    if command is None:
        return
    profile = getattr(command, "pressure_profile", None)
    if profile is not None and str(profile).upper() != "1R":
        raise PipetteValidationError(
            "Only OEM-backed pressure profile 1R is admitted; arbitrary profiles are not source-validated."
        )
    if operation_name in {"aspirate", "dispense", "mix"}:
        if getattr(command, "air_gap_ul", None) is not None:
            raise PipetteValidationError(
                "air_gap_ul is not an OEM ClassPipette operation and is rejected until mapped to pinned evidence."
            )
        if operation_name == "dispense" and bool(getattr(command, "blow_out", False)):
            raise PipetteValidationError(
                "blow_out is not an OEM ClassPipette operation; use an explicit A0R workflow after physical acceptance."
            )


def _persist_failure(
    *,
    receipt_store: Any,
    claim_record: dict[str, Any] | None,
    operation_name: str | None,
    failure_code: str,
    message: str,
    status: str,
    requested_inputs: dict[str, Any],
    runtime_binding: dict[str, Any] | None,
    outcome_may_have_occurred: bool = False,
) -> dict[str, Any] | None:
    if receipt_store is None or claim_record is None or not hasattr(receipt_store, "record_failure"):
        return None
    command_id = claim_record.get("command_id")
    operation_id = claim_record.get("pipette_operation_id") or command_id
    if not command_id or not operation_id:
        return None
    try:
        persisted = receipt_store.record_failure(
            command_id=str(command_id),
            pipette_operation_id=str(operation_id),
            operation=str(operation_name or "pipette"),
            failure_code=str(failure_code),
            message=str(message),
            expected_status=str(claim_record.get("status") or ""),
            status=str(status),
            requested_inputs=requested_inputs,
            runtime_binding=runtime_binding,
        )
        linked = persisted.get(_LINKED_FINALIZATION_KEY) if isinstance(persisted, Mapping) else None
        if isinstance(linked, Mapping):
            ambiguous = status in {"outcome_unknown", "reconciliation_required"}
            status_code = 504 if ambiguous else 400 if status == "rejected" else 503
            raise HTTPException(
                status_code=status_code,
                detail={
                    "error": str(failure_code),
                    "message": str(message),
                    "completion_ambiguous": ambiguous,
                    "outcome_unknown": ambiguous,
                    "reconciliation_required": ambiguous,
                    "retry_forbidden": ambiguous,
                    _LINKED_FINALIZATION_KEY: dict(linked),
                },
            )
        return dict(persisted) if isinstance(persisted, Mapping) else None
    except HTTPException:
        raise
    except (PipetteReceiptError, OSError, RuntimeError, ValueError) as exc:
        ambiguous = bool(
            outcome_may_have_occurred
            or status in {"outcome_unknown", "reconciliation_required"}
        )
        raise HTTPException(
            status_code=504 if ambiguous else 503,
            detail={
                "error": "pipette_failure_receipt_persistence_failed",
                "message": str(exc),
                "completion_ambiguous": ambiguous,
                "outcome_unknown": ambiguous,
                "reconciliation_required": ambiguous,
                "retry_forbidden": ambiguous,
            },
        ) from exc


async def _run_transport_call(
    label: str,
    *,
    timeout_s: float,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    operation: Callable[[Any], dict[str, Any]],
    operation_name: str | None = None,
    command: Any | None = None,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
    runtime_binding: dict[str, Any] | None = None,
    requested_inputs: dict[str, Any] | None = None,
) -> dict[str, Any]:
    requested_for_receipt = (
        dict(requested_inputs)
        if requested_inputs is not None
        else (command.to_payload() if command is not None and hasattr(command, "to_payload") else {})
    )
    claim_record: dict[str, Any] | None = None
    claim_created = False
    replay_claim_record: dict[str, Any] | None = None
    effective_runtime_binding = dict(runtime_binding or {})
    if receipt_store is not None and hasattr(receipt_store, "claim"):
        try:
            from ..operator_controls import current_operator_dispatch_context

            dispatch_context = current_operator_dispatch_context() or {}
        except Exception:
            dispatch_context = {}
        binding = dict(runtime_binding or {})
        operation_key = str(operation_name or label).lower()
        tip_action = getattr(getattr(command, "action", None), "value", getattr(command, "action", None))
        control_class = (
            "hardware_query"
            if operation_key in READ_ONLY_PIPETTE_OPERATIONS
            else "host_state_verification"
            if operation_key == "tip" and str(tip_action).lower() == "load"
            else "physical_liquid_command"
        )
        protocol_identity = (
            f"protocol:{effective_runtime_binding['protocol_job_id']}:{effective_runtime_binding['protocol_action_id']}"
            if effective_runtime_binding.get("protocol_job_id") is not None
            and effective_runtime_binding.get("protocol_action_id") is not None
            else None
        )
        idempotency_identity = (
            effective_runtime_binding.get("idempotency_key")
            or dispatch_context.get("idempotency_key")
            or protocol_identity
            or _DIRECT_PIPETTE_IDEMPOTENCY.get()
        )
        if idempotency_identity is None and operation_key not in READ_ONLY_PIPETTE_OPERATIONS:
            raise HTTPException(
                status_code=422,
                detail={"error": "stable_pipette_idempotency_identity_required"},
            )
        idempotency_key = str(idempotency_identity or f"pipette-query:{uuid4().hex}")
        effective_runtime_binding.setdefault(
            "callback_session_id",
            f"pipette-callback:{hashlib.sha256(idempotency_key.encode('utf-8')).hexdigest()[:32]}",
        )
        binding = effective_runtime_binding
        outer_command_id = dispatch_context.get("operator_command_id")
        # Lifecycle steps have their own keys; only direct pipette actions share the outer claim.
        if outer_command_id and binding.get("caller_class") in {"lifecycle", "protocol_manual"}:
            binding["parent_operator_command_id"] = str(outer_command_id)
            outer_command_id = None
        direct_command_id = binding.get("command_id")
        current_ownership_generation = int(hardware_state.ownership_epoch)
        requested_ownership_generation = binding.get("ownership_generation")
        if requested_ownership_generation is None:
            requested_ownership_generation = dispatch_context.get("expected_ownership_generation")
        if requested_ownership_generation is None:
            requested_ownership_generation = current_ownership_generation
        if int(requested_ownership_generation) != current_ownership_generation:
            raise HTTPException(
                status_code=409,
                detail={
                    "error": "pipette_ownership_generation_mismatch",
                    "retry_forbidden": True,
                },
            )
        source_transport = get_transport()
        source_identity_reader = getattr(source_transport, "prepare_collection_source_identity", None)
        if not callable(source_identity_reader):
            source_identity_reader = getattr(source_transport, "collection_source_identity", None)
        if callable(source_identity_reader):
            collection_owner = source_identity_reader()
            if not isinstance(collection_owner, Mapping):
                raise PipetteReceiptError("pipette_collection_owner_invalid")
            # Revisions change with setters, not the idempotent request identity.
            binding["collection_owner"] = {**collection_owner, "channels": [
                {k: row[k] for k in ("reader", "reader_generation")}
                for row in collection_owner["channels"]]}
            binding["collection_source_affecting"] = (
                operation_key not in READ_ONLY_PIPETTE_OPERATIONS
                or operation_key in {"tip_status", "query_tip_status", "query_all_pipette_tip_states",
                                     "live_readback", "readback"})
        claim_arguments: dict[str, Any] = dict(
                operation=str(operation_name or label),
                requested_inputs=requested_for_receipt,
                entrypoint_id=str(
                    binding.get("entrypoint_id")
                    or dispatch_context.get("entrypoint_id")
                    or f"service.pipette.{operation_name or label}"
                ),
                caller_class=str(
                    binding.get("caller_class")
                    or dispatch_context.get("caller_class")
                    or "direct_api"
                ),
                control_class=str(binding.get("control_class") or control_class),
                action_id=str(
                    dispatch_context.get("action_id")
                    or binding.get("action_id")
                    or f"pipette.{operation_name or label}"
                ),
                idempotency_key=idempotency_key,
                command_id=(str(outer_command_id or direct_command_id) if (outer_command_id or direct_command_id) else None),
                ownership_generation=int(requested_ownership_generation),
                connection_generation=(
                    int(binding["connection_generation"])
                    if binding.get("connection_generation") is not None
                    else (
                        int(dispatch_context["connection_generation"])
                        if dispatch_context.get("connection_generation") is not None
                        else 0
                    )
                ),
                protocol_job_id=binding.get("protocol_job_id"),
                protocol_action_id=binding.get("protocol_action_id"),
                lifecycle_stage_id=binding.get("lifecycle_stage_id"),
                lifecycle_attempt_id=binding.get("lifecycle_attempt_id"),
                callback_session_id=binding.get("callback_session_id"),
                runtime_binding=binding,
        )
        try:
            if outer_command_id and hasattr(receipt_store, "attach_to_operator_claim"):
                claim_record, created = receipt_store.attach_to_operator_claim(**claim_arguments)
            else:
                claim_record, created = receipt_store.claim(**claim_arguments)
            claim_created = bool(created)
        except (PipetteReceiptError, OSError, RuntimeError, ValueError) as exc:
            raise HTTPException(
                status_code=503,
                detail={"error": "pipette_claim_persistence_failed", "message": str(exc)},
            ) from exc
        if not claim_created:
            if int(claim_record.get("ownership_generation", -1)) != current_ownership_generation:
                raise HTTPException(
                    status_code=409,
                    detail={
                        "error": "pipette_replay_ownership_generation_mismatch",
                        "retry_forbidden": True,
                    },
                )
            replay_claim_record = claim_record
    failure_claim_record = claim_record if claim_created else None
    preflight_payload: dict[str, Any] | None = None
    stage = "admission"
    try:
        _validate_oem_admission(operation_name, command)
        if preflight is not None:
            preflight_payload = preflight(operation_name or label, command)
        if replay_claim_record is not None:
            try:
                if receipt_store is not None and hasattr(receipt_store, "replay_result"):
                    return receipt_store.replay_result(
                        command_id=str(replay_claim_record["command_id"]),
                        pipette_operation_id=str(replay_claim_record["pipette_operation_id"]),
                    )
            except (PipetteReceiptError, OSError, RuntimeError, ValueError, KeyError) as exc:
                raise HTTPException(
                    status_code=503,
                    detail={"error": "pipette_replay_receipt_unavailable", "message": str(exc)},
                ) from exc
            return {**replay_claim_record, "replayed": True}
        stage = "transport"
        transport = get_transport()
        stage = "dispatch"
        result = await run_blocking(label, lambda: operation(transport), timeout_s=timeout_s)
        if isinstance(result, dict) and effective_runtime_binding.get("callback_session_id"):
            callback_session_id = str(effective_runtime_binding["callback_session_id"])
            result.setdefault("callback_session_id", callback_session_id)
            provenance = result.get("provenance")
            if isinstance(provenance, dict):
                result["provenance"] = {**provenance, "callback_session_id": callback_session_id}
    except asyncio.CancelledError:
        if stage == "dispatch":
            try:
                _persist_failure(
                    receipt_store=receipt_store,
                    claim_record=failure_claim_record,
                    operation_name=operation_name or label,
                    failure_code="outer_waiter_cancelled_after_dispatch",
                    message="pipette dispatch waiter was cancelled; physical outcome is unknown",
                    status="outcome_unknown",
                    requested_inputs=requested_for_receipt,
                    runtime_binding=effective_runtime_binding,
                    outcome_may_have_occurred=True,
                )
            finally:
                raise
        raise
    except PipetteError as exc:
        if (operation_name in {"tip_status", "query_tip_status", "query_all_pipette_tip_states"} and stage == "dispatch"
                and isinstance(exc.details.get("observed_channels"), list)):
            # Retain every actual return, including strict-invalid rows. Only
            # genuine source exceptions stop progression before later channels.
            result = {**exc.to_payload(), "channels": exc.details["observed_channels"],
                **{key: exc.details[key] for key in (
                    "source_return_completed", "source_exception", "source_return", "source_tip_exists"
                ) if key in exc.details},
                "partial_query": True, "hardware_query_verified": False,
                "hardware_truth_level": "hardware_query"}
        else:
            status = "rejected" if stage == "admission" else "failed"
            _persist_failure(
                receipt_store=receipt_store,
                claim_record=failure_claim_record,
                operation_name=operation_name or label,
                failure_code=exc.code,
                message=exc.message,
                status=status,
                requested_inputs=requested_for_receipt,
                runtime_binding=effective_runtime_binding,
                outcome_may_have_occurred=stage == "dispatch",
            )
            raise _pipette_error_to_http_exception(exc) from exc
    except ValueError as exc:
        failure_code = "validation_error" if stage == "admission" else "malformed_response"
        status = "rejected" if stage == "admission" else "failed"
        _persist_failure(
            receipt_store=receipt_store,
            claim_record=failure_claim_record,
            operation_name=operation_name or label,
            failure_code=failure_code,
            message=str(exc),
            status=status,
            requested_inputs=requested_for_receipt,
            runtime_binding=effective_runtime_binding,
            outcome_may_have_occurred=stage == "dispatch",
        )
        if stage == "admission":
            raise _pipette_error_to_http_exception(PipetteValidationError(str(exc))) from exc
        raise HTTPException(status_code=502, detail={"error": failure_code, "message": str(exc)}) from exc
    except HTTPException as exc:
        detail = exc.detail if isinstance(exc.detail, dict) else {}
        if detail.get("completion_ambiguous") is True or detail.get("outcome_unknown") is True:
            _persist_failure(
                receipt_store=receipt_store,
                claim_record=failure_claim_record,
                operation_name=operation_name or label,
                failure_code=str(detail.get("error") or "tester_operation_completion_ambiguous"),
                message=str(detail.get("message") or exc.detail),
                status="outcome_unknown",
                requested_inputs=requested_for_receipt,
                runtime_binding=effective_runtime_binding,
                outcome_may_have_occurred=True,
            )
            raise
        _persist_failure(
            receipt_store=receipt_store,
            claim_record=failure_claim_record,
            operation_name=operation_name or label,
            failure_code="transport_exception" if stage != "admission" else "admission_exception",
            message=str(exc.detail),
            status="failed" if stage != "admission" else "rejected",
            requested_inputs=requested_for_receipt,
            runtime_binding=effective_runtime_binding,
            outcome_may_have_occurred=stage == "dispatch",
        )
        raise HTTPException(
            status_code=503 if stage != "admission" else 400,
            detail={"error": "pipette_operation_failed", "message": str(exc.detail)},
        ) from exc
    except asyncio.TimeoutError as exc:
        _persist_failure(
            receipt_store=receipt_store,
            claim_record=failure_claim_record,
            operation_name=operation_name or label,
            failure_code="outcome_unknown",
            message="pipette operation completion is ambiguous",
            status="reconciliation_required",
            requested_inputs=requested_for_receipt,
            runtime_binding=effective_runtime_binding,
            outcome_may_have_occurred=True,
        )
        raise HTTPException(
            status_code=504,
            detail={
                "error": "pipette_operation_completion_ambiguous",
                "message": str(exc),
                "completion_ambiguous": True,
                "retry_forbidden": True,
            },
        ) from exc
    except Exception as exc:
        _persist_failure(
            receipt_store=receipt_store,
            claim_record=failure_claim_record,
            operation_name=operation_name or label,
            failure_code="transport_exception" if stage != "admission" else "admission_exception",
            message=str(exc),
            status="failed" if stage != "admission" else "rejected",
            requested_inputs=requested_for_receipt,
            runtime_binding=effective_runtime_binding,
            outcome_may_have_occurred=stage == "dispatch",
        )
        raise HTTPException(
            status_code=503 if stage != "admission" else 400,
            detail={"error": "pipette_operation_failed", "message": str(exc)},
        ) from exc
    if isinstance(result, dict) and stage == "dispatch":
        source_snapshot = getattr(transport, "collection_source_snapshot", None)
        if callable(source_snapshot):
            result["collection_source"] = source_snapshot()
    if preflight_payload is not None and isinstance(result, dict):
        result.setdefault("preflight", preflight_payload)
    if isinstance(result, dict):
        read_only = str(operation_name or label).lower() in READ_ONLY_PIPETTE_OPERATIONS
        if read_only:
            result = dict(_query_result_without_mutation_claims(result))
        correlated_count, correlations_valid = _semantic_query_correlation(result)
        semantic_query_verified = bool(
            read_only
            and result.get("ok") is True
            and result.get("hardware_truth_level") == "hardware_query"
            and correlated_count > 0
            and correlations_valid
        )
        result["semantic_query_response_verified"] = semantic_query_verified
        if semantic_query_verified:
            result["delivery_verified"] = False
            result["controller_acknowledged"] = False
            result["completion_verified"] = False
    if receipt_store is not None and isinstance(result, dict):
        try:
            effective_candidate = result.get("effective")
            effective_inputs = dict(effective_candidate) if isinstance(effective_candidate, dict) else dict(requested_for_receipt)
            if "effective_volume_ul" in result:
                effective_inputs["volume_ul"] = result["effective_volume_ul"]
            if "dispense_type" in result:
                effective_inputs["dispense_type"] = result["dispense_type"]
            receipt = receipt_store.record(
                operation=operation_name or label,
                requested_inputs=requested_for_receipt,
                effective_inputs=effective_inputs,
                result=result,
                runtime_binding=effective_runtime_binding,
                command_id=claim_record.get("command_id") if claim_record is not None else None,
                pipette_operation_id=claim_record.get("pipette_operation_id") if claim_record is not None else None,
                expected_status=claim_record.get("status") if claim_record is not None else None,
            )
        except PipetteReceiptError as exc:
            detail: dict[str, Any] = {
                "error": "pipette_receipt_persistence_failed",
                "message": str(exc),
                "completion_ambiguous": True,
                "outcome_unknown": True,
                "reconciliation_required": True,
                "retry_forbidden": True,
            }
            if exc.linked_finalization is not None:
                detail[_LINKED_FINALIZATION_KEY] = exc.linked_finalization
            if operation_name in {"tip_status", "query_tip_status", "query_all_pipette_tip_states"} and claim_record is not None:
                # Actual collected evidence survives failure to write its receipt.
                # The command claim exists; no receipt ID or persistence is invented.
                detail["query_observation"] = {**result,
                    "command_id": claim_record["command_id"],
                    "source_identity": receipt_store._source_identity()}
            raise HTTPException(
                status_code=503,
                detail=detail,
            ) from exc
        linked_finalization = receipt.pop(_LINKED_FINALIZATION_KEY, None)
        if isinstance(linked_finalization, Mapping):
            result[_LINKED_FINALIZATION_KEY] = dict(linked_finalization)
        result["receipt_id"] = receipt["receipt_id"]
        result["receipt_truth"] = receipt["truth"]
        result["source_identity"] = receipt["source_identity"]
        if claim_record is not None and claim_record.get("command_id"):
            result["command_id"] = claim_record["command_id"]
    return result


async def run_pipette_operation(
    operation_name: str,
    operation: Callable[[Any], dict[str, Any]],
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    timeout_s: float = 600.0,
    receipt_store: PipetteReceiptStore | None = None,
    requested_inputs: dict[str, Any] | None = None,
    preflight: PipettePreflight | None = None,
    runtime_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Route a non-command OEM control through the same receipt/error owner."""
    return await _run_transport_call(
        operation_name.replace("_", " ").title(),
        timeout_s=timeout_s,
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=operation,
        operation_name=operation_name,
        preflight=preflight,
        receipt_store=receipt_store,
        requested_inputs=requested_inputs,
        runtime_binding=runtime_binding,
    )


async def run_pipette_status(
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    receipt_store: PipetteReceiptStore | None = None,
    runtime_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return await _run_transport_call(
        "Pipette status",
        timeout_s=600.0,
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=lambda transport: transport.get_status(),
        operation_name="status",
        receipt_store=receipt_store,
        runtime_binding=runtime_binding,
    )


async def run_pipette_init_command(
    command: PipetteInitCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
    runtime_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return await _run_transport_call(
        "Pipette init",
        timeout_s=1800.0,
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=lambda transport: transport.initialize(command),
        operation_name="init",
        command=command,
        preflight=preflight,
        receipt_store=receipt_store,
        runtime_binding=runtime_binding,
    )


async def run_pipette_tip_command(
    command: PipetteTipCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
    runtime_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return await _run_transport_call(
        "Pipette tip",
        timeout_s=20.0,
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=lambda transport: transport.set_tip(command),
        operation_name="tip",
        command=command,
        preflight=preflight,
        receipt_store=receipt_store,
        runtime_binding=runtime_binding,
    )


async def run_pipette_aspirate_command(
    command: PipetteAspirateCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
    runtime_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return await _run_transport_call(
        "Pipette aspirate",
        timeout_s=20.0,
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=lambda transport: transport.aspirate(command),
        operation_name="aspirate",
        command=command,
        preflight=preflight,
        receipt_store=receipt_store,
        runtime_binding=runtime_binding,
    )


async def run_pipette_dispense_command(
    command: PipetteDispenseCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
    runtime_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return await _run_transport_call(
        "Pipette dispense",
        timeout_s=20.0,
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=lambda transport: transport.dispense(command),
        operation_name="dispense",
        command=command,
        preflight=preflight,
        receipt_store=receipt_store,
        runtime_binding=runtime_binding,
    )


async def run_pipette_mix_command(
    command: PipetteMixCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
    runtime_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return await _run_transport_call(
        "Pipette mix",
        timeout_s=max(20.0, float(command.cycles) * 5.0),
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=lambda transport: transport.mix(command),
        operation_name="mix",
        command=command,
        preflight=preflight,
        receipt_store=receipt_store,
        runtime_binding=runtime_binding,
    )


def _oem_source_result(action: Any, steps: list[dict[str, Any]], *, noop: bool = False) -> dict[str, Any]:
    """A source return is not a new physical-completion criterion."""
    return {
        "ok": all(step["result"]["ok"] for step in steps),
        "source_return": [action.source_key],
        "source_occurrence_id": action.source_occurrence_id,
        "source_noop": noop,
        "native_results": steps,
        "owned_children": [child for step in steps for child in step["result"].get("owned_children", ())],
        "source_return_completed": True,
        "physical_effect_verified": False,
    }


def _oem_source_step(
    action: Any, state: Any, steps: list[dict[str, Any]], name: str,
    before_native_entry: Callable, operation: Callable,
) -> dict[str, Any]:
    """Use the inherited parent fence again at each nested entry/publication."""
    identity = f"{action.source_occurrence_id}:{len(steps)}:{name}"
    try:
        before_native_entry(identity, state)
        result = operation()
        if not isinstance(result, Mapping) or type(result.get("ok")) is not bool:
            raise RuntimeError(f"OEM {name} did not return an explicit native outcome")
    except Exception as exc:
        # Preserve the original exception/receipt and completed sibling results.
        setattr(exc, "oem_partial_results", list(steps))
        raise
    result = dict(result)
    steps.append({"step_id": identity, "operation": name, "result": result})
    return result


@dataclass(frozen=True)
class OemPipetteSourceBindings:
    """Finite trusted native dependencies, not a user method-dispatch surface.

    All physical callbacks end in (action, state) and return native mappings.
    facts/is_pierced/z_low/tip_exists are existing-owner source reads, not new
    device queries. start_child is B's owned task facility, never a D worker.
    """
    facts: Callable
    lift_to: Callable
    lower_to: Callable
    move_xy: Callable
    position: Callable
    home: Callable
    tip_state: Callable
    is_pierced: Callable
    pierce: Callable
    z_low: Callable
    set_color: Callable
    stall_guard: Callable
    lower_pipette: Callable
    lift_pipette: Callable
    snapshot: Callable
    shift_camera: Callable
    unlock: Callable
    hotel_move: Callable
    tip_transition: Callable
    tip_exists: Callable
    start_child: Callable
    stopped: Callable
    script_move: Callable
    publish_location: Callable
    move_z: Callable
    move_x: Callable
    set_z_current_max: Callable
    source_error_event: Callable | None = None
    led2_on: Callable | None = None
    check_tips: Callable | None = None
    take_aspirate_image: Callable | None = None
    source_capabilities: frozenset[str] = frozenset()
    sleep: Callable = time.sleep


def _source_option(options: Mapping, key: str, default: Any) -> Any:
    value = options.get(key)
    return default if value is None else value


def _source_dispense_height(location: int, volume: float) -> float:
    # Literal ControlLib.DispenseHeight:7138-7176; no new fluid geometry.
    if location == 3:
        height = 0.0 if volume < 50.0 else (volume ** 0.55 * 0.33 if volume < 125.0 else 4.7 + (volume - 125.0) / 64.0)
        return min(height, 25.0) * 2015.0
    return ((volume + 26.10952) / 14.15714 if volume > 0.1 else 0.0) * 2015.0


class _OemPipetteBody:
    """One source-call frame; runtime/model/custody remain at existing owners."""
    def __init__(self, action, state, native, pipette_call, fence, settings, prefix=""):
        self.action, self.state, self.n = action, state, native
        self.pipette_call, self.fence, self.settings = pipette_call, fence, settings
        self.steps = []
        self.prefix = prefix

    @property
    def model(self):
        return self.state.source_model

    def facts(self):
        return self.n.facts(self.action, self.state)

    def step(self, name, call):
        return _oem_source_step(self.action, self.state, self.steps, self.prefix + name, self.fence, call)

    def native(self, name, callback, *args):
        return self.step(name, lambda: callback(*args, self.action, self.state))

    def pipette(self, name, call):
        identity = f"{self.action.source_occurrence_id}:{len(self.steps)}:{self.prefix}{name}"
        return self.step(name, lambda: self.pipette_call(name, call, self.action, self.state, identity))

    def speed(self, value):
        return self.pipette("set_top_speed", lambda t: t.set_top_speed(value))

    def asp(self, volume, stream=False):
        if stream:
            return self.pipette("aspirate_for_oem_script", lambda t: t.aspirate_for_oem_script(volume, pressure_stream=True))
        return self.pipette("aspirate", lambda t: t.aspirate(PipetteAspirateCommand(volume_ul=volume)))

    def dsp(self, volume, stream=False):
        if stream:
            return self.pipette("dispense_for_oem_script", lambda t: t.dispense_for_oem_script(volume, pressure_stream=True))
        return self.pipette("dispense", lambda t: t.dispense(PipetteDispenseCommand(volume_ul=volume)))

    def air(self, volume, front=False):
        return self.pipette("aspirate_air", lambda t: t.aspirate_air(int(volume), channels=t._tip_location_channels(), front_air=front))

    def lift(self, height=None):
        return self.native("liftTo", self.n.lift_to, self.facts()["current_location"], height)

    def lower(self):
        return self.native("lowerTo", self.n.lower_to, self.facts()["current_location"])

    def lift_air(self):
        loc = self.facts()["current_location"]
        # Location enum is bound at the native owner; source lift heights unchanged.
        height = 64503 if loc == 16 else 54425 if loc == 3 else 38299 if loc in (11, 12, 13, 14) else 28220
        return self.lift(height)

    def volume(self):
        # MachineStatus1000-1294: CurrentTray can survive an unmapped
        # updateLocation. All three model calls must receive that canonical
        # fact; deriving a tray from CurrentLocation changes source semantics.
        f = self.facts()
        return self.model.current_well_volume(f["current_location"], f["current_well"], current_tray=f["current_tray"])

    def height(self, volume):
        return _source_dispense_height(self.facts()["current_location"], volume)

    def fluid_name(self):
        f = self.facts()
        self.model.update_fluid_name(f["current_location"], f["current_well"], current_tray=f["current_tray"])

    def fluid(self, delta):
        f = self.facts()
        self.model.update_fluid_level(f["current_location"], f["current_well"], f["tip_location"], delta, current_tray=f["current_tray"])

    def tip_state(self, **changes):
        result = self.native("tip_state", self.n.tip_state, changes)
        if result["ok"] and "tip_loaded" in changes:
            self.model.logical_tip_present = changes["tip_loaded"]
        return result

    def delay(self, milliseconds):
        self.n.sleep(milliseconds / 1000.0)

    def pierced_after(self):
        f = self.facts()
        if f["current_location"] == 3 and not self.n.is_pierced(2, f["current_well"], f["tip_location"] != -1, self.action, self.state):
            self.native("MoveZHome", self.n.home, None)
            self.native("PierceFoil", self.n.pierce, 2, f["current_well"], f["tip_location"] != -1)

    def purge(self, speed=30.0, *, amp=False, ntd=False):
        saved_speed = self.facts()["speed"]
        loc = self.facts()["current_location"]
        height = 44346 if ntd and loc == 3 else round(self.height(self.volume())) + 6045 if ntd else round(self.height(self.volume()) + 6045.0)
        self.lift(max(0, height))
        self.speed(speed)
        self.pipette("dispense_all", lambda t: t.dispense_all())
        self.speed(saved_speed)
        self.delay(100)
        if ntd:
            return
        if amp:
            self.air(2)
            self.delay(100)
        self.lift(max(0, round(self.height(self.volume()) - 4030.0)))
        if amp:
            self.lift(max(0, round(self.height(self.volume())) + 6045))
            self.speed(speed)
            self.pipette("dispense_all", lambda t: t.dispense_all())
            self.delay(100)

    def transfer(self, kind, volume=None, speed=None, delay=None):
        # PipetteTransfer10490-10515, including null/zero early return.
        if volume is None or volume <= 0:
            return
        if speed is not None:
            self.speed(speed)
        if kind == "Dispense":
            self.dsp(volume)
        elif kind == "Aspirate":
            self.asp(volume)
        elif kind == "AspirateAir":
            self.air(volume)
        if delay is not None:
            self.delay(delay)

    def da(self):
        try:
            volume = float(self.action.params["arguments"][0])
        except ValueError:
            return _oem_source_result(self.action, [], noop=True)
        self.lift()
        self.speed(30.0)
        self.pipette("dispense_air_source", lambda t: t.dispense_air_for_oem_script(float(int(volume)), front_air=True, pressure_stream=self.settings["LogPressure"]))
        if not self.settings["LogPressure"]:
            self.pipette("read_pressure", lambda t: t.read_pressure_for_oem_source(
                lambda: self.n.tip_exists(self.action, self.state)))
        return self.result()

    def masp(self):
        o = self.action.params["arguments"]
        volume = self.volume() if o.get("m_volume") is None else o["m_volume"] + o.get("m_overaspirate", 0.0)
        air = _source_option(o, "m_air", 20.0)
        speed = _source_option(o, "m_speed", 30.0)
        delay = int(_source_option(o, "m_delay", 1000.0))
        height = _source_option(o, "m_aspirateheight", -1.0)
        return self._masp_transfer(volume, air, speed, delay, height)

    def prefill_aspirate(self, volume):
        # ControlLib:7752-7755 masp(100, v) -> masp(v, 10, 100, 0).
        return self._masp_transfer(volume, 10.0, 100.0, 0, -1.0, prefill=True)

    def _masp_transfer(self, volume, air, speed, delay, height, *, prefill=False):
        def aspirate(amount):
            if prefill:
                return self.pipette("aspirate_for_oem_script", lambda t: t.aspirate_for_oem_script(
                    amount, pressure_stream=self.settings["LogPressure"]))
            return self.asp(amount, self.settings["LogPressure"])

        self.speed(speed)
        self.lift_air()
        self.air(air)
        tray = self.facts()["current_tray"]
        if height == -1.0:
            if volume > 130.0 and tray in (0, 1):
                half = volume / 2.0
                self.lift(max(0, round(self.height(half)) - 6045))
                aspirate(half)
                self.fluid_name(); self.fluid(-half); self.delay(delay)
                self.lower(); aspirate(half)
                self.fluid(-half); self.delay(delay); self.lift(500); self.delay(300)
            else:
                self.lower(); aspirate(volume)
                self.delay(delay); self.lift(500); self.delay(300)
                self.fluid_name(); self.fluid(-volume)
        else:
            height *= 2015.748
            if volume > 130.0 and tray == 0:
                half = volume / 2.0
                self.lift(max(0, min(int(height), round(self.height(half)) + 6045)))
                aspirate(half)
                self.fluid_name(); self.fluid(-half); self.delay(delay)
                self.lift(int(height)); aspirate(half)
                self.fluid(-half); self.delay(delay); self.lift(max(500, int(height))); self.delay(300)
            else:
                self.lift(int(height)); aspirate(volume)
                self.delay(delay); self.lift(max(500, int(height))); self.delay(300)
                self.fluid_name(); self.fluid(-volume)
        self.pierced_after()
        self.tip_state(tip_dirty=True)
        return self.result()

    def dsa(self):
        o = self.action.params["arguments"]
        speed = int(_source_option(o, "m_speed", 30))
        delay = _source_option(o, "m_delay", int(self.facts()["fluid_level"] * 20.0 + 200.0))
        purge = _source_option(o, "m_purge", True)
        ntd = _source_option(o, "m_ntd", False)
        dh = _source_option(o, "m_dispensehigh", False)
        heights = _source_option(o, "m_dispenseheight", -1.0)
        purge_speed = _source_option(o, "m_purgespeed", speed)
        return self._mdsa_transfer(speed, delay, purge, ntd, dh, heights, purge_speed)

    def prefill_dispense(self):
        # ControlLib:7944-7947 mdsa(100) -> mdsa(100, 0), with defaults.
        return self._mdsa_transfer(100.0, 0, True, False, False, -1.0, 30.0, prefill=True)

    def _mdsa_transfer(self, speed, delay, purge, ntd, dh, heights, purge_speed, *, prefill=False):
        def dispense(amount):
            if prefill:
                return self.pipette("dispense_for_oem_script", lambda t: t.dispense_for_oem_script(
                    amount, pressure_stream=self.settings["LogPressure"]))
            return self.dsp(amount, self.settings["LogPressure"])

        f = self.facts(); fluid = f["fluid_level"]
        if f["current_location"] == 32:
            raise ValueError("dispense to unknow location")
        self.speed(speed)
        high = (44346 if f["current_tray"] == 2 else 18141) if heights == -1.0 else int(heights * 2015.748)
        height = min(self.n.z_low(f["current_location"], self.action, self.state), high) if dh else round(self.height(self.volume()) - 2015.0)
        if fluid > 130.0 and f["current_tray"] in (0, 1):
            half = fluid / 2.0
            self.lift(max(0, height))
            dispense(half); self.fluid(half)
            self.lift(max(0, round(self.height(self.volume()) - 2015.0)))
            dispense(fluid - half)
            self.delay(delay); self.fluid(fluid - half)
        else:
            self.lift(max(0, height))
            dispense(fluid)
            self.delay(delay); self.fluid(fluid)
        if purge:
            self.purge(purge_speed, ntd=ntd)
        self.pierced_after()
        return self.result()

    def mix_options(self, debubble=False):
        o = self.action.params["arguments"]
        asp = o["m_aspirateOptions"]
        dsp = o["m_dispenseOptions" if debubble else "m_dispenseAllOptions"]
        volume = min(_source_option(asp, "m_volume", max(self.volume() - 5.0, self.volume() / 2.0)), self.facts()["tip_type"])
        air = _source_option(asp, "m_air", 15.0)
        asp_speed = _source_option(asp, "m_speed", 100.0)
        asp_delay = _source_option(asp, "m_delay", int(volume * 20.0 + 200.0))
        dsp_volume = _source_option(dsp, "m_volume", 5.0) if debubble else 5.0
        dsp_speed = _source_option(dsp, "m_speed", 20.0)
        dsp_delay = _source_option(dsp, "m_delay", int(dsp_volume * 5.0 + 200.0))
        count = _source_option(o, "m_repeat", 0 if debubble else 2)
        tip_dip = o.get("m_tipDip") is None or o["m_tipDip"] == " *"
        return o, volume, air, asp_speed, asp_delay, dsp_volume, dsp_speed, dsp_delay, count, tip_dip

    def mmix(self):
        o, volume, air, asp_speed, asp_delay, _, dsp_speed, dsp_delay, count, tip_dip = self.mix_options()
        if volume <= 5.0:
            return _oem_source_result(self.action, [], noop=True)
        if air > 0:
            self.speed(asp_speed); self.lift_air(); self.air(air)
        f = self.facts()
        if volume > 130.0 and f["current_tray"] in (0, 1):
            half = volume / 2.0
            height = max(0, round(self.height(half)) - 6045)
            self.lift(height)
            for _ in range(count):
                self.speed(asp_speed); self.asp(half, self.settings["LogPressure"])
                self.lift(400); self.asp(half); self.delay(asp_delay)
                self.speed(dsp_speed); self.dsp(half); self.lift(height)
                self.dsp(half); self.delay(dsp_delay)
        else:
            self.lift(400)
            mix_type = _source_option(o, "m_mixType", "N")
            if mix_type == "C":
                p = self.native("getCurrentPosition", self.n.position)
                x, y = p["x"], p["y"]
                for _ in range(count):
                    for nx, ny in ((x+200,y+200),(x-200,y+200),(x-200,y-200),(x+200,y-200),(x+200,y+200),(x,y)):
                        self.speed(asp_speed); self.asp(volume); self.delay(asp_delay)
                        self.speed(dsp_speed); self.dsp(volume); self.delay(dsp_delay)
                        self.native("moveXY", self.n.move_xy, nx, ny)
            else:
                for _ in range(count):
                    if mix_type == "H": self.lift(400)
                    self.speed(asp_speed); self.asp(volume); self.delay(asp_delay)
                    self.speed(dsp_speed)
                    if mix_type == "H": self.lift(4000)
                    self.dsp(volume); self.delay(dsp_delay)
        self.purge(dsp_speed, ntd=not tip_dip)
        self.tip_state(tip_dirty=True)
        return self.result()

    def rmb(self):
        o, volume, air, asp_speed, asp_delay, dsp_volume, dsp_speed, dsp_delay, count, tip_dip = self.mix_options(True)
        if volume <= 5.0:
            return _oem_source_result(self.action, [], noop=True)
        initial = self.volume()
        p = self.native("getCurrentPosition", self.n.position)
        x, y = p["x"], p["y"]
        for _ in range(count):
            portion = dsp_volume
            self.lift_air(); self.speed(asp_speed); self.air(air)
            self.lower(); self.asp(volume); self.delay(asp_delay)
            remaining = initial - volume
            self.speed(dsp_speed)
            while initial - remaining > 0.01:
                height = self.height(remaining)
                if _source_option(o, "m_orbit", False):
                    for nx, ny in ((x+200,y+200),(x-200,y+200),(x-200,y-200),(x+200,y-200),(x+200,y+200),(x,y)):
                        self.native("moveXY", self.n.move_xy, nx, ny)
                self.lift(round(height)); self.dsp(portion); self.delay(dsp_delay)
                remaining += portion
                if initial - remaining < 5.0:
                    portion = initial - remaining
            self.purge(dsp_speed, ntd=not tip_dip)
        self.tip_state(tip_dirty=True)
        return self.result()

    def ampmix(self):
        o = self.action.params["arguments"]
        cwv = self.volume()
        count = _source_option(o, "m_repeat", 40)
        self.transfer("AspirateAir", 50.0, 100.0, 0)
        self.lower(); self.lift()
        self.transfer("AspirateAir", 50.0, 100.0, 0)
        self.lift(int(self.height(self.volume() / 2.0)))
        self.transfer("Aspirate", 10.0, 30.0, 500)
        self.lift(0)
        rng = random.Random()
        for _ in range(count):
            self.lift(int(self.height(rng.random() * 10.0)))
            self.transfer("Aspirate", cwv - 10.0 - 20.0, 600.0, 300)
            self.transfer("Dispense", cwv - 10.0 - 20.0, 600.0, 300)
            self.transfer("Aspirate", cwv / 2.0, 100.0, 500)
            self.lift(int(self.height(self.volume() / 3.0)))
            self.transfer("Dispense", cwv / 2.0, 600.0, 500)
            self.lift(0)
        self.lift(int(self.height(self.volume()))); self.delay(2000)
        self.transfer("Dispense", 10.0, 30.0, 500)
        self.purge(30.0, amp=True)
        self.pierced_after()
        return self.result()

    @staticmethod
    def well_number(well):
        if isinstance(well, int):
            return well
        try:
            return int(well)
        except ValueError:
            return (ord(well[0]) - ord("A")) * 12 + int(well[1:]) - 1

    def query_tips(self, pipette=-1):
        if pipette == -1:
            result = self.pipette("query_tip_status_all", lambda t: t.query_tip_status_all())
        else:
            result = self.pipette("query_tip_status_for_oem_script", lambda t: t.query_tip_status_for_oem_script(pipette))
        return result["source_return"]

    def eject(self, check=True):
        return self.pipette("eject_all_tips", lambda t: t.eject_all_tips(check_missing_tip=check, wait=True))

    def move(self, location, column=0, row=0):
        return self.native("scriptmoveTo", self.n.script_move, location, column, row)

    def publish_location(self, location, well):
        return self.native("updateLocation", self.n.publish_location, location, well)

    def waste(self):
        self.move(6); self.publish_location(6, 0)

    def remove(self, tray, well, pipette=-1, hotel=False):
        base = self.well_number(well)
        indices = [base] if hotel else [base + 24 * pipette] if pipette != -1 else [base + offset for offset in (0, 24, 48, 72)]
        self.native("removeTip", self.n.tip_transition, tray, indices, "remove", None, self.model.tip_zone_index)
        self.model.tip_removed(tray, well, pipette, hotel=hotel)

    def signal_error(self, message):
        if self.n.source_error_event is not None:
            self.n.source_error_event(message, self.action, self.state)

    def pressure_base(self):
        pressure = self.pipette("read_pressure", lambda t: t.read_pressure_for_oem_source(
            lambda: self.n.tip_exists(self.action, self.state)))
        values = [0.0, 0.0, 0.0, 0.0]
        for row in pressure["channels"]:
            values[row["channel"]] = row["result"]["pressure"]
        self.model.add_pressure_base(values)

    def pressload(self, pipette):
        # ControlLib9325-9471. Preserve the native source retry loop, not an
        # outer generic retry, and preserve default versus rehome=false Home.
        required = 1 if pipette != -1 else 4
        self.native("lowerPipette", self.n.lower_pipette, self.facts()["current_location"])
        self.tip_state(tip_loaded=True)
        self.native("liftPipette", self.n.lift_pipette, self.facts()["current_location"])
        count = self.query_tips(pipette)
        queries = iteration = 0
        while count < required and iteration < 100:
            if count < required:
                count = self.query_tips(pipette); queries += 1
            if count < required and queries > 2:
                queries = 0
                self.native("lowerPipette", self.n.lower_pipette, self.facts()["current_location"])
                self.native("MoveZHome", self.n.home, False)
                count = self.query_tips(pipette)
            iteration += 1
            if iteration > 3:
                break
        if 0 < count < required:
            self.tip_state(tip_loaded=True)
            self.native("MoveZHome", self.n.home, False)
            self.native("SnapshotImage", self.n.snapshot, "tip load issue - not enough tips loaded ")
            if self.settings["StartMode"] != 3:
                self.waste(); self.eject(False)
            else:
                self.native("moveZ", self.n.move_z, self.facts()["z_high"])
                self.eject(False)
            self.tip_state(tip_loaded=False)
            if self.settings["StartMode"] != 3:
                self.native("moveZ", self.n.move_z, 80000)
                self.native("moveX", self.n.move_x, 79000)
            # Source num3 remains zero; do not invent the unreachable >1 retry.
        elif required == 1 and count == 0 and self.query_tips(-1) > 0:
            self.tip_state(tip_loaded=True)
            self.native("shiftCameraForTrayPicture", self.n.shift_camera)
            self.native("SnapshotImage", self.n.snapshot, "tip load issue ")
            if self.settings["StartMode"] != 3: self.waste()
            self.eject(False); self.tip_state(tip_loaded=False)
        success = count == required
        if success and pipette != -1:
            self.pipette("KeepTip", lambda t: t.KeepTip(pipette))
        home = self.native("MoveZHome", self.n.home, None)
        if count == 0 and required == 4:
            self.native("shiftCameraForTrayPicture", self.n.shift_camera)
            self.native("SnapshotImage", self.n.snapshot, "tip load issue - no tips loaded")
        if abs(home["source_return"]) > 250:
            self.native("shiftCameraForTrayPicture", self.n.shift_camera)
            self.native("SnapshotImage", self.n.snapshot, "tip load issue - lost steps")
            if self.settings["OverPressChecked"] is None:
                self.native("unlockDoor", self.n.unlock)
        return success

    def approach(self, tip_type, pipette, hotel):
        if hotel:
            well = self.model.next_hotel_tip()
            location, tray = 15, 4
        else:
            selected = self.model.select_tip(tip_type, pipette)
            if selected is None:
                return None
            tray, location, well = selected
            self.model.old_tip_well = well
        row, column = ord(well[0]) - ord("A"), int(well[1:]) - 1
        self.move(location, column, row)
        if not hotel:
            self.publish_location(location, int(well[1:]))
        self.publish_location(location, self.well_number(well))
        return tray, well

    def newload(self, tip_type, force=False, pipette=-1, hotel=False):
        self.query_tips()
        self.native("setColor", self.n.set_color, 255, 255, 255)
        if self.n.led2_on is not None: self.native("led2On", self.n.led2_on)
        self.native("setStallGuard", self.n.stall_guard, 10)
        f = self.facts()
        if f["tip_type"] == tip_type and not force:
            return True
        if self.n.tip_exists(self.action, self.state) and f["tip_type"] != 201 and (force or f["tip_type"] != tip_type):
            self.waste(); self.eject(True)
        loaded = False
        well = "A1"  # source local tiplocation default before selector
        # Hotel's failure removal uses source tiplocation initialized to A1,
        # not the selected hotel location. Preserve that distinct source branch.
        while True:
            selected = self.approach(tip_type, pipette, hotel)
            if selected is None: break
            tray, well = selected
            selected_pipette = 0 if hotel else pipette
            if self.pressload(selected_pipette):
                self.pipette("loadTip", lambda t: t.loadTip(tip_type, tip_location=selected_pipette))
                self.tip_state(tip_location=selected_pipette)
                self.remove(tray, well, selected_pipette, hotel=hotel)
                loaded = True
                break
            self.remove(tray, "A1" if hotel else well, -1)
        if not loaded:
            self.signal_error("Tips are not available")
            raise RuntimeError("Tips are not available")
        self.native("setStallGuard", self.n.stall_guard, None)
        self.tip_state(tip_dirty=False, tip_loaded=True)
        success = True
        if not hotel and self.settings["CameraInstalled"] and self.settings["CameraCalibrated"]:
            checked = self.native("checkTips", self.n.check_tips, 0, well)
            success = checked["source_return"]
            if not success:
                self.waste(); self.eject(False); self.tip_state(tip_loaded=False)
                self.native("moveZ", self.n.move_z, 80000)
                self.native("moveX", self.n.move_x, 79000)
        self.native("setZaxisCurrentmax", self.n.set_z_current_max, None)
        return success

    def fill_hotel(self):
        if self.model.tip_hotel_empty():
            self.newload(50)
            self.native("setZaxisCurrentmax", self.n.set_z_current_max, 31)
            self.native("moveToTipHotel", self.n.hotel_move)
            self.publish_location(15, 12)
            self.eject(True)
            # Existing occupancy publication must commit before model labels.
            self.native("loadAllTipsInTipHotel", self.n.tip_transition, 4, list(range(96)), "restore", None, None)
            self.model.hotel_loaded()

    def ldtip(self):
        args = self.action.params["arguments"]
        self.model.allow_to_stop = False
        self.pressure_base()
        tip_types = {"T50": 50, "T200": 200, "UNKNOWN": 201}
        tip_type = tip_types[args[0]] if args[0] in tip_types else int(args[0])
        force = len(args) > 1 and args[1] in ("T", "True")
        pipette = int(args[2]) if len(args) > 2 else -1
        hotel = len(args) > 3 and args[3] == "H"
        if len(args) > 3:
            if not hotel: self.model.tip_zone_index = int(args[3])
        else:
            self.model.tip_zone_index = None
        while not self.n.stopped(self.action, self.state):
            if hotel: self.fill_hotel()
            if self.newload(tip_type, force, pipette, hotel): break
        return self.result()

    def ejt(self):
        f = self.facts()
        tip_location = f["tip_location"]
        if self.settings["CheckSnapTips"]:
            self.native("takeAspirateImage", self.n.take_aspirate_image)
        started = threading.Event()
        ejection = _OemPipetteBody(self.action, self.state, self.n, self.pipette_call, self.fence, self.settings, "ejt.ejection.")
        movement = _OemPipetteBody(self.action, self.state, self.n, self.pipette_call, self.fence, self.settings, "ejt.motion.")
        def eject_child():
            started.set()
            ejection.eject(True)
            return ejection.result()
        def motion_child():
            current = movement.facts()
            if current["current_location"] != 6:
                label = self.action.params["arguments"][0] if self.action.params["arguments"] else "Reuse"
                old = self.model.old_tip_well
                if tip_location != -1 or old is not None:
                    tray = int(current["current_location_name"][-1]) - 1
                    if tip_location == -1:
                        indices = [self.well_number(old) + offset for offset in (0, 24, 48, 72)]
                        restored_well = old
                    else:
                        restored_well = self.well_number(old) + 24 * tip_location
                        indices = [restored_well]
                    movement.native("restoretip", self.n.tip_transition, tray, indices, "restore", label, self.model.tip_zone_index)
                    self.model.tip_restored(tray, restored_well, label, self.model.tip_zone_index)
            else:
                # DBC is excluded by A preflight; normal source branch only.
                movement.native("moveZ", self.n.move_z, 65000)
                movement.native("moveX", self.n.move_x, 79000)
            return movement.result()
        first = self.n.start_child("ejt.ejection", eject_child, self.action, self.state)
        # Wait on the source start marker, but do not strand on a task rejected
        # before entry. That is task failure, never a fabricated native start.
        while not started.wait(0.01):
            if first.done():
                first.result()
                raise RuntimeError("ejt child settled without its source start marker")
        try:
            second = self.n.start_child("ejt.motion", motion_child, self.action, self.state)
        except Exception as admission_error:
            try:
                first.result()
            except Exception as child_error:
                setattr(admission_error, "oem_child_errors", [child_error])
            self.steps.extend(ejection.steps)
            setattr(admission_error, "oem_partial_results", list(self.steps))
            raise
        errors = []
        for future in (first, second):
            try: future.result()
            except Exception as exc: errors.append(exc)
        self.steps.extend(ejection.steps); self.steps.extend(movement.steps)
        if errors:
            setattr(errors[0], "oem_partial_results", list(self.steps))
            setattr(errors[0], "oem_child_errors", errors)
            raise errors[0]
        exists = self.n.tip_exists(self.action, self.state)
        if exists:
            self.signal_error("Eject tip failed")
        else:
            self.tip_state(tip_dirty=False, tip_loaded=False)
            self.model.logical_tip_present = False
        self.model.allow_to_stop = True
        result = self.result()
        if exists: result.update(source_pause_scripts=True, source_error_event="Eject tip failed")
        return result

    def result(self):
        return _oem_source_result(self.action, self.steps)


@dataclass(frozen=True)
class _OemLifecycleContext:
    source_occurrence_id: str
    source_key: None = None
    params: Any = None


def build_oem_prefill_subprocedures(
    *, state: Any, source_occurrence_id: str, before_native_entry: Callable,
    pipette_call: Callable, source_bindings: OemPipetteSourceBindings,
    settings: Mapping[str, Any],
) -> tuple[Callable[[int], dict[str, Any]], Callable[[], dict[str, Any]]]:
    """Bind zOffset's two liquid calls inside the existing finite owner's claim.

    The caller owns movement, tip loading and the scan. Each invocation gets a
    distinct source identity, including repeated aliquots at the same well.
    """
    if not source_occurrence_id:
        raise ValueError("prefill requires the finite owner's source identity")
    occurrence = 0

    def run(name, method, *args):
        nonlocal occurrence
        occurrence += 1
        context = _OemLifecycleContext(
            f"{source_occurrence_id}:prefill:{occurrence}:{name}", params={"arguments": ()})
        body = _OemPipetteBody(context, state, source_bindings, pipette_call,
                               before_native_entry, settings)
        try:
            result = method(body, *args)
        except Exception as exc:
            if not hasattr(exc, "oem_partial_results"):
                setattr(exc, "oem_partial_results", list(body.steps))
            raise
        return {**result, "source_return": None}

    def aspirate(volume):
        return run("masp(100,v)", _OemPipetteBody.prefill_aspirate, volume)

    def dispense():
        return run("mdsa(100)", _OemPipetteBody.prefill_dispense)

    return aspirate, dispense


def build_oem_pipette_lifecycle_helpers(
    *, before_native_entry: Callable, pipette_call: Callable,
    source_bindings: OemPipetteSourceBindings, settings: Mapping[str, Any],
    move_to_waste: Callable, sweep_handler: Callable,
) -> dict[str, Callable]:
    """Pipette portions only; E/F retain complete lifecycle orchestration.

    Every helper requires an explicit source_occurrence_id. In particular E's
    three prologue pressure reads MUST have distinct occurrence IDs; D neither
    resets a hidden counter nor invents a second custody/receipt store.
    """
    def frame(state, identity):
        if not isinstance(identity, str) or not identity:
            raise ValueError("lifecycle pipette helper requires original source substep identity")
        context = _OemLifecycleContext(identity, params={"arguments": ()})
        return _OemPipetteBody(context, state, source_bindings, pipette_call,
                               before_native_entry, dict(settings))

    def baseline(state, *, source_occurrence_id):
        body = frame(state, source_occurrence_id)
        body.pressure_base()
        return {**body.result(), "source_return": None}

    def run_job_prefix(state, *, source_occurrence_id):
        # ControlLib.run_job5094-5112, not the scriptmove cleanup overload.
        body = frame(state, source_occurrence_id)
        body.query_tips(); body.delay(500)
        if source_bindings.tip_exists(body.action, state):
            body.native("moveToWaste", move_to_waste)
            body.eject(False); body.query_tips(); body.delay(100)
            if source_bindings.tip_exists(body.action, state):
                message = "Please manually remove tips on Pipette then start"
                body.signal_error(message)
                return {**body.result(), "source_return": None, "source_pause_scripts": True,
                        "source_error_event": message, "source_error_hold": True}
        body.tip_state(tip_loaded=False)
        return {**body.result(), "source_return": None}

    def safe_stop_tip_exit(state, *, source_occurrence_id):
        # ControlLib.executeScript:5440-5462. The executor owns the logical
        # no-tip/no-plate boundary and motion join; query actual collection here.
        body = frame(state, source_occurrence_id)
        try:
            queried = body.pipette("query_tip_status_all", lambda t: t.query_tip_status_all())
            if not queried["ok"]:
                return body.result()
            body.delay(1000)
            if source_bindings.tip_exists(body.action, state):
                for step in (
                    lambda: body.native("moveToWaste", move_to_waste),
                    lambda: body.publish_location(6, 96),
                    lambda: body.eject(True),
                    lambda: body.tip_state(tip_dirty=False),
                    lambda: body.native("moveZ", source_bindings.move_z, 80000),
                    lambda: body.native("moveX", source_bindings.move_x, 79000),
                ):
                    if not step()["ok"]:
                        return body.result()
            body.tip_state(tip_loaded=False)
            return {**body.result(), "source_return": None}
        except Exception as exc:
            if not hasattr(exc, "oem_partial_results"):
                setattr(exc, "oem_partial_results", list(body.steps))
            raise

    def cleanup_prefix(state, *, source_occurrence_id):
        # Caller must first execute cleanup10628-10632 stop-wait/door predicate.
        # This is ONLY 10633-10642, never a complete cleanup/thermal hook.
        body = frame(state, source_occurrence_id)
        body.query_tips()
        if source_bindings.tip_exists(body.action, state):
            body.waste(); body.eject(False)
            body.native("moveZ", source_bindings.move_z, 80000)
            body.native("moveX", source_bindings.move_x, 79000)
        body.tip_state(tip_loaded=False)
        return {**body.result(), "source_return": None}

    def sweep(state, *, source_occurrence_id):
        body = frame(state, source_occurrence_id)
        # ControlLib normal epilogue selects any rm member, unlike sweep().
        result = sweep_handler(body.action, state, clearall=True)
        return {**result, "source_return": None}

    return {"pressure_baseline": baseline, "run_job_tip_prefix": run_job_prefix,
            "cleanup_pipette_prefix": cleanup_prefix, "epilogue_sweep": sweep,
            "safe_stop_tip_exit": safe_stop_tip_exit}


def build_oem_pipette_handlers(
    *,
    before_native_entry: Callable,
    script_move: Callable | None = None,
    publish_location: Callable | None = None,
    publish_tip_transition: Callable | None = None,
    pipette_call: Callable | None = None,
    lift_for_air: Callable | None = None,
    move_to_waste: Callable | None = None,
    move_z: Callable | None = None,
    move_x: Callable | None = None,
    tip_load_move: Callable | None = None,
    move_z_home: Callable | None = None,
    set_z_current_max: Callable | None = None,
    remove_tip: Callable | None = None,
    script_move_to_waste: Callable | None = None,
    source_error_event: Callable | None = None,
    source_bindings: OemPipetteSourceBindings | None = None,
    settings: Mapping[str, Any] | None = None,
) -> dict[str, Callable]:
    """Finite source-model bindings composed by the canonical workflow owner.

    Callbacks are trusted composition dependencies, never prepared input.
    Source model semantics belong to ProtocolRuntimeState.source_model; native
    facts and tip occupancy remain at the existing canonical/provider owners.
    """
    def la(action: Any, state: Any) -> dict[str, Any]:
        before_native_entry(action.source_occurrence_id, state)
        tokens = [action.source_key, "la", *action.params["arguments"]]
        source_return = state.source_model.la(tokens)
        return {**_oem_source_result(action, []), "source_return": source_return,
                "source_model_updated": True}

    def ms(action: Any, state: Any) -> dict[str, Any]:
        arguments = action.params["arguments"]
        location, row = state.source_model.select_strip(arguments[0], float(arguments[1]))
        steps: list[dict[str, Any]] = []
        moved = _oem_source_step(
            action, state, steps, "scriptmoveTo", before_native_entry,
            lambda: cast(Callable, script_move)(location, row, action, state),
        )
        if moved["ok"]:
            # ControlLib.processms:8055-8056: the move returns before publication.
            _oem_source_step(
                action, state, steps, "updateLocation", before_native_entry,
                lambda: cast(Callable, publish_location)(location, 12 * row, action, state),
            )
        return _oem_source_result(action, steps)

    def retip(action: Any, state: Any) -> dict[str, Any]:
        steps: list[dict[str, Any]] = []
        for tray_id, well_ids in state.source_model.retip_wells():
            # Empty Reuse wells, excluding hotel; never a constructor reset.
            selected = list(well_ids)
            result = _oem_source_step(
                action, state, steps, "retip", before_native_entry,
                lambda: cast(Callable, publish_tip_transition)(tray_id, selected, action, state),
            )
            if not result["ok"]:
                break
            state.source_model.retip_committed(tray_id, selected)
        return {**_oem_source_result(action, steps), "source_model_updated": True}

    def aa(action: Any, state: Any) -> dict[str, Any]:
        # ControlLib.scriptInterpretor:5840-5849. Invalid numeric text is the
        # documented same-line no-op, not a generic success for a missing body.
        try:
            volume = float(action.params["arguments"][0])
        except ValueError:
            return _oem_source_result(action, [], noop=True)
        steps: list[dict[str, Any]] = []
        _oem_source_step(action, state, steps, "LiftForAir", before_native_entry,
                         lambda: cast(Callable, lift_for_air)(action, state))
        def pipette(name: str, operation: Callable) -> dict[str, Any]:
            step_id = f"{action.source_occurrence_id}:{len(steps)}:{name}"
            return _oem_source_step(
                action, state, steps, name, before_native_entry,
                lambda: cast(Callable, pipette_call)(name, operation, action, state, step_id),
            )
        pipette("set_top_speed", lambda transport: transport.set_top_speed(30.0))
        pipette("aspirate_air", lambda transport: transport.aspirate_air(
            int(volume), channels=transport._tip_location_channels(), front_air=True,
        ))
        if not cast(Mapping, source_settings)["LogPressure"]:
            pipette("read_pressure", lambda transport: transport.read_pressure_for_oem_source(
                lambda: cast(OemPipetteSourceBindings, source_bindings).tip_exists(action, state)))
        return _oem_source_result(action, steps)

    def ini_pipette(action: Any, state: Any) -> dict[str, Any]:
        assert pipette_call is not None and move_to_waste is not None
        assert move_z is not None and move_x is not None
        steps: list[dict[str, Any]] = []
        def pipette(name: str, operation: Callable) -> dict[str, Any]:
            step_id = f"{action.source_occurrence_id}:{len(steps)}:{name}"
            return _oem_source_step(
                action, state, steps, name, before_native_entry,
                lambda: cast(Callable, pipette_call)(name, operation, action, state, step_id),
            )
        initialized = pipette("reinitialize_pipette", lambda t: t.reinitialize_pipette())
        retry = 0
        # ControlLib.initPipette:9280-9316: first call plus at most three
        # source retries. Exceptions are NOT false returns and are never retried.
        while not initialized["ok"] and retry < 3:
            initialized = pipette("reinitialize_pipette", lambda t: t.reinitialize_pipette())
            tips = pipette("query_tip_status_all", lambda t: t.query_tip_status_all())
            if tips["source_tip_exists"]:
                _oem_source_step(action, state, steps, "moveToWaste", before_native_entry,
                                 lambda: cast(Callable, move_to_waste)(action, state))
                pipette("eject_all_tips", lambda t: t.eject_all_tips(check_missing_tip=True, wait=True))
                _oem_source_step(action, state, steps, "moveZ", before_native_entry,
                                 lambda: cast(Callable, move_z)(80000, action, state))
                _oem_source_step(action, state, steps, "moveX", before_native_entry,
                                 lambda: cast(Callable, move_x)(79000, action, state))
            else:
                state.source_model.logical_tip_present = False
            retry += 1
        if not initialized["ok"]:
            return {**_oem_source_result(action, steps), "ok": False,
                    "source_error": "could not initialize pipette"}
        pressure = pipette("read_pressure", lambda t: t.read_pressure_for_oem_source(
            lambda: cast(OemPipetteSourceBindings, source_bindings).tip_exists(action, state)))
        # ClassPipetteCollection.readPressure initializes all four values to
        # zero, querying only source-present tips. These zeros are model data,
        # not fabricated hardware-query verification.
        baseline = [0.0, 0.0, 0.0, 0.0]
        for row in pressure["channels"]:
            baseline[row["channel"]] = row["result"]["pressure"]
        state.source_model.add_pressure_base(baseline)
        # A successful source retry owns the init result; earlier false attempts
        # remain unmodified evidence, not a newly imposed retry-failure policy.
        return {**_oem_source_result(action, steps), "ok": True,
                "source_init_return": 0, "source_model_updated": True}

    def sweep(action: Any, state: Any, *, clearall: bool = False) -> dict[str, Any]:
        """ControlLib.sweep:7012-7049; clearall is an internal caller option."""
        steps: list[dict[str, Any]] = []
        pause_scripts = False
        def step(name: str, operation: Callable) -> dict[str, Any]:
            return _oem_source_step(action, state, steps, name, before_native_entry, operation)
        def pipette(name: str, operation: Callable) -> dict[str, Any]:
            step_id = f"{action.source_occurrence_id}:{len(steps)}:{name}"
            return step(name, lambda: cast(Callable, pipette_call)(name, operation, action, state, step_id))
        for tray in range(4):
            locations = state.source_model.sweep_locations(tray, clearall)
            if locations is None:
                continue
            for well in locations:
                state.source_model.allow_to_stop = False
                row = ord(well[0]) - ord("A")
                column_text = well[1:]
                column = int(column_text) - 1
                location = state.source_model.get_tip_tray_location(tray)
                step("loadTip.scriptmoveTo", lambda: cast(Callable, tip_load_move)(location, column, row, action, state))
                # Literal source parses the stripped numeric suffix as wellID.
                # Do not silently replace it with the original A1-style name.
                step("loadTip.updateLocation", lambda: cast(Callable, publish_location)(location, int(column_text), action, state))
                step("loadTip.MoveZHome", lambda: cast(Callable, move_z_home)(action, state))
                step("loadTip.setZaxisCurrentmax", lambda: cast(Callable, set_z_current_max)(action, state))
                pipette("query_tip_status_all", lambda t: t.query_tip_status_all())
                state.source_model.logical_tip_present = True
                step("removeTip", lambda: cast(Callable, remove_tip)(tray, well, action, state))
                step("sweep.scriptmoveToWaste", lambda: cast(Callable, script_move_to_waste)(action, state))
                pipette("eject_all_tips", lambda t: t.eject_all_tips(check_missing_tip=True, wait=True))
                state.source_model.allow_to_stop = True
                step("moveZ", lambda: cast(Callable, move_z)(80000, action, state))
                step("moveX", lambda: cast(Callable, move_x)(79000, action, state))
                tips = pipette("query_tip_status_all", lambda t: t.query_tip_status_all())
                time.sleep(0.100)
                if tips["source_tip_exists"]:
                    pause_scripts = True
                    if source_error_event is not None:
                        source_error_event("Eject tip failed", action, state)
        result = _oem_source_result(action, steps)
        if pause_scripts:
            result.update(source_pause_scripts=True, source_error_event="Eject tip failed")
        return result

    source_settings = dict(settings) if settings is not None else None
    if source_settings is not None and type(source_settings.get("LogPressure")) is not bool:
        raise ValueError("OEM source settings require the captured LogPressure boolean")
    handlers = {"la": la}
    pressure_source_bound = source_bindings is not None and callable(source_bindings.tip_exists)
    if (pipette_call is not None and lift_for_air is not None and source_settings is not None
            and (source_settings["LogPressure"] or pressure_source_bound)):
        handlers["aa"] = aa
    if (pipette_call is not None and move_to_waste is not None and move_z is not None
            and move_x is not None and pressure_source_bound):
        handlers["iniPipette"] = ini_pipette
    if script_move is not None and publish_location is not None:
        handlers["ms"] = ms
    if all(dependency is not None for dependency in (
        pipette_call, tip_load_move, publish_location, move_z_home,
        set_z_current_max, remove_tip, script_move_to_waste, move_z, move_x,
    )):
        handlers["sweep"] = sweep
    if publish_tip_transition is not None:
        handlers["retip"] = retip
    if source_bindings is not None and pipette_call is not None and source_settings is not None:
        def bind_body(opcode, method):
            def preflight(action):
                args = action.params["arguments"]
                if opcode in {"masp", "dsa", "mmix", "rmb", "ampmix"} and not isinstance(args, Mapping):
                    raise ValueError(f"{opcode} requires captured typed source options")
                required = {
                    "da": ("LogPressure",), "masp": ("LogPressure",),
                    "dsa": ("LogPressure",), "mmix": ("LogPressure",),
                    "rmb": (), "ampmix": (),
                    "ldtip": ("StartMode", "CameraInstalled", "CameraCalibrated", "OverPressChecked"),
                    "ejt": ("CheckSnapTips",),
                }[opcode]
                missing = [key for key in required if key not in source_settings]
                if missing: raise ValueError(f"{opcode} missing captured source settings: {missing}")
                if opcode in {"masp", "dsa", "mmix"} and source_settings["LogPressure"]:
                    needed = {"masp": {"aspirate_pressure_stream"}, "dsa": {"dispense_pressure_stream"}, "mmix": {"aspirate_pressure_stream"}}[opcode]
                    if not needed <= source_bindings.source_capabilities:
                        raise ValueError(f"{opcode} selected unbound native pressure stream overload: {sorted(needed)}")
                if opcode == "ldtip":
                    hotel = len(args) > 3 and args[3] == "H"
                    single = int(args[2]) if len(args) > 2 else -1
                    if (hotel or single != -1) and "query_tip_status_single" not in source_bindings.source_capabilities:
                        raise ValueError("ldtip selected unbound native single queryTipStatus overload")
                    if source_settings["CameraInstalled"] and source_settings["CameraCalibrated"] and source_bindings.check_tips is None:
                        raise ValueError("ldtip selected checkTips semantic CV callback is unbound")
                if opcode == "ejt" and source_settings["CheckSnapTips"] and source_bindings.take_aspirate_image is None:
                    raise ValueError("ejt selected takeAspirateImage callback is unbound")
                return {"ok": True}
            def run(action, state):
                preflight(action)
                body = _OemPipetteBody(action, state, source_bindings, pipette_call, before_native_entry, source_settings)
                try:
                    return method(body)
                except Exception as exc:
                    if not hasattr(exc, "oem_partial_results"):
                        setattr(exc, "oem_partial_results", list(body.steps))
                    raise
            setattr(run, "preflight", preflight)
            return run
        for opcode, method in {
            "da": _OemPipetteBody.da, "masp": _OemPipetteBody.masp,
            "dsa": _OemPipetteBody.dsa, "mmix": _OemPipetteBody.mmix,
            "rmb": _OemPipetteBody.rmb, "ampmix": _OemPipetteBody.ampmix,
            "ldtip": _OemPipetteBody.ldtip, "ejt": _OemPipetteBody.ejt,
        }.items():
            handlers[opcode] = bind_body(opcode, method)
    return handlers
