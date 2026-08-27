from __future__ import annotations

import asyncio
import hashlib
import json
from collections.abc import Mapping
from contextvars import ContextVar, Token
from typing import Any, Awaitable, Callable
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
        count = 0
        valid = True
        if "query_response_correlated" in value:
            count = 1
            valid = (
                value.get("query_response_correlated") is True
                and value.get("semantic_ok") is True
            )
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
