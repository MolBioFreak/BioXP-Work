from __future__ import annotations

from typing import Any, Awaitable, Callable
from uuid import uuid4

from fastapi import HTTPException
from ..pipette.models import (
    PipetteAspirateCommand,
    PipetteDispenseCommand,
    PipetteError,
    PipetteInitCommand,
    PipetteMixCommand,
    PipetteTipCommand,
    PipetteValidationError,
)
from ..pipette.receipts import PipetteReceiptError, PipetteReceiptStore
from ..pipette.transport import PipetteTransport

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
    if receipt_store is not None and hasattr(receipt_store, "claim"):
        try:
            from ..operator_controls import current_operator_dispatch_context

            dispatch_context = current_operator_dispatch_context() or {}
        except Exception:
            dispatch_context = {}
        binding = dict(runtime_binding or {})
        control_class = (
            "hardware_query"
            if str(operation_name or label).lower() in {
                "status",
                "readback",
                "query_status",
                "query_pressure",
                "read_pressure",
                "query_tip_status",
                "query_error_log",
                "get_data",
                "get_all_data",
            }
            else "physical_liquid_command"
        )
        try:
            claim_record, _ = receipt_store.claim(
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
                idempotency_key=str(
                    binding.get("idempotency_key")
                    or dispatch_context.get("idempotency_key")
                    or f"pipette-attempt:{uuid4().hex}"
                ),
                command_id=(
                    str(binding.get("command_id") or dispatch_context.get("operator_command_id"))
                    if (binding.get("command_id") or dispatch_context.get("operator_command_id"))
                    else None
                ),
                ownership_generation=int(
                    binding.get("ownership_generation")
                    or dispatch_context.get("expected_ownership_generation")
                    or 0
                ),
                runtime_binding=binding,
            )
        except (PipetteReceiptError, OSError, RuntimeError, ValueError) as exc:
            raise HTTPException(
                status_code=503,
                detail={"error": "pipette_claim_persistence_failed", "message": str(exc)},
            ) from exc
    preflight_payload: dict[str, Any] | None = None
    try:
        _validate_oem_admission(operation_name, command)
    except PipetteError as exc:
        raise _pipette_error_to_http_exception(exc) from exc
    if preflight is not None:
        try:
            preflight_payload = preflight(operation_name or label, command)
        except PipetteError as exc:
            raise _pipette_error_to_http_exception(exc) from exc
        except ValueError as exc:
            raise _pipette_error_to_http_exception(PipetteValidationError(str(exc))) from exc
    transport = get_transport()
    try:
        result = await run_blocking(label, lambda: operation(transport), timeout_s=timeout_s)
    except PipetteError as exc:
        raise _pipette_error_to_http_exception(exc) from exc
    except ValueError as exc:
        raise _pipette_error_to_http_exception(PipetteValidationError(str(exc))) from exc
    if preflight_payload is not None and isinstance(result, dict):
        result.setdefault("preflight", preflight_payload)
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
                runtime_binding=runtime_binding,
            )
        except PipetteReceiptError as exc:
            raise HTTPException(
                status_code=503,
                detail={"error": "pipette_receipt_persistence_failed", "message": str(exc)},
            ) from exc
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
    )


async def run_pipette_status(
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    receipt_store: PipetteReceiptStore | None = None,
) -> dict[str, Any]:
    return await _run_transport_call(
        "Pipette status",
        timeout_s=600.0,
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=lambda transport: transport.get_status(),
        operation_name="status",
        receipt_store=receipt_store,
    )


async def run_pipette_init_command(
    command: PipetteInitCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
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
    )


async def run_pipette_tip_command(
    command: PipetteTipCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
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
    )


async def run_pipette_aspirate_command(
    command: PipetteAspirateCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
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
    )


async def run_pipette_dispense_command(
    command: PipetteDispenseCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
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
    )


async def run_pipette_mix_command(
    command: PipetteMixCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
    receipt_store: PipetteReceiptStore | None = None,
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
    )
