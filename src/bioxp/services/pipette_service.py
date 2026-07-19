from __future__ import annotations

from typing import Any, Awaitable, Callable

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
from ..pipette.transport import PipetteTransport

BlockingRunner = Callable[..., Awaitable[dict[str, Any]]]
TransportGetter = Callable[[], PipetteTransport]
PipettePreflight = Callable[[str, Any], dict[str, Any]]


def _pipette_error_to_http_exception(exc: PipetteError) -> HTTPException:
    return HTTPException(status_code=exc.status_code, detail=exc.to_payload())


async def _run_transport_call(
    label: str,
    *,
    timeout_s: float,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    operation: Callable[[PipetteTransport], dict[str, Any]],
    operation_name: str | None = None,
    command: Any | None = None,
    preflight: PipettePreflight | None = None,
) -> dict[str, Any]:
    preflight_payload: dict[str, Any] | None = None
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
    return result


async def run_pipette_status(*, get_transport: TransportGetter, run_blocking: BlockingRunner) -> dict[str, Any]:
    return await _run_transport_call(
        "Pipette status",
        timeout_s=600.0,
        get_transport=get_transport,
        run_blocking=run_blocking,
        operation=lambda transport: transport.get_status(),
        operation_name="status",
    )


async def run_pipette_init_command(
    command: PipetteInitCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
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
    )


async def run_pipette_tip_command(
    command: PipetteTipCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
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
    )


async def run_pipette_aspirate_command(
    command: PipetteAspirateCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
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
    )


async def run_pipette_dispense_command(
    command: PipetteDispenseCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
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
    )


async def run_pipette_mix_command(
    command: PipetteMixCommand,
    *,
    get_transport: TransportGetter,
    run_blocking: BlockingRunner,
    preflight: PipettePreflight | None = None,
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
    )
