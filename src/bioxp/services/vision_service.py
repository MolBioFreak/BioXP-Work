from __future__ import annotations

from typing import Any, Awaitable, Callable

from fastapi import HTTPException

from ..domain.capabilities import CapabilityName, CapabilityRegistry
from ..vision.barcode import (
    BarcodeReadCommand,
    BarcodeReadResult,
    decode_barcodes_from_snapshot,
)
from ..vision.inspection import InspectionCommand, InspectionResult

BlockingRunner = Callable[..., Awaitable[dict[str, Any]]]
CapabilityGetter = Callable[[], CapabilityRegistry]
SnapshotCapturer = Callable[[str], dict[str, Any]]
BarcodeDecoder = Callable[..., tuple]


def _require_inspection_capability(registry: CapabilityRegistry) -> str:
    if registry.is_enabled(CapabilityName.INSPECTION):
        return CapabilityName.INSPECTION.value
    raise HTTPException(status_code=409, detail="Inspection capability is unavailable for this BioXP profile.")


def _resolve_barcode_capability(registry: CapabilityRegistry) -> str:
    if registry.is_enabled(CapabilityName.BARCODE):
        return CapabilityName.BARCODE.value
    if registry.is_enabled(CapabilityName.INSPECTION):
        return CapabilityName.INSPECTION.value
    raise HTTPException(
        status_code=409,
        detail="Barcode reading requires either barcode or inspection capability.",
    )


async def run_inspection_command(
    command: InspectionCommand,
    *,
    get_capabilities: CapabilityGetter,
    capture_snapshot: SnapshotCapturer,
    run_blocking: BlockingRunner,
) -> dict[str, Any]:
    capability_used = _require_inspection_capability(get_capabilities())
    snapshot = await run_blocking(
        "Vision inspection",
        lambda: capture_snapshot(command.device),
        timeout_s=20.0,
    )
    return InspectionResult.from_snapshot(
        command,
        snapshot,
        capability_used=capability_used,
    ).to_payload()


async def run_barcode_read_command(
    command: BarcodeReadCommand,
    *,
    get_capabilities: CapabilityGetter,
    capture_snapshot: SnapshotCapturer,
    run_blocking: BlockingRunner,
    decode_barcodes: BarcodeDecoder = decode_barcodes_from_snapshot,
) -> dict[str, Any]:
    capability_used = _resolve_barcode_capability(get_capabilities())
    snapshot = await run_blocking(
        "Vision barcode read",
        lambda: capture_snapshot(command.device),
        timeout_s=20.0,
    )
    candidates = tuple(decode_barcodes(snapshot, symbologies=command.symbologies))
    decoder_available = True
    return BarcodeReadResult.from_snapshot(
        command,
        snapshot,
        capability_used=capability_used,
        candidates=candidates,
        decoder_available=decoder_available,
    ).to_payload()
