from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping


def _normalize_symbology(value: str | None) -> str | None:
    if value is None:
        return None
    normalized = str(value).strip().lower()
    return normalized or None


@dataclass(frozen=True)
class BarcodeReadCommand:
    device: str = "/dev/video0"
    location_id: str | None = None
    symbologies: tuple[str, ...] = ()
    include_image_data: bool = False

    def __post_init__(self) -> None:
        normalized = []
        for raw_value in self.symbologies:
            symbology = _normalize_symbology(raw_value)
            if symbology and symbology not in normalized:
                normalized.append(symbology)
        object.__setattr__(self, "symbologies", tuple(normalized))
        object.__setattr__(self, "device", str(self.device or "/dev/video0"))
        object.__setattr__(self, "location_id", None if self.location_id is None else str(self.location_id))
        object.__setattr__(self, "include_image_data", bool(self.include_image_data))

    @classmethod
    def from_request(cls, req: Any) -> "BarcodeReadCommand":
        return cls(
            device=getattr(req, "device", "/dev/video0"),
            location_id=getattr(req, "location_id", None),
            symbologies=tuple(getattr(req, "symbologies", ()) or ()),
            include_image_data=bool(getattr(req, "include_image_data", False)),
        )


@dataclass(frozen=True)
class BarcodeCandidate:
    value: str
    symbology: str | None = None
    confidence: float | None = None
    source: str = "snapshot_metadata"
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def to_payload(self) -> dict[str, Any]:
        return {
            "value": self.value,
            "symbology": self.symbology,
            "confidence": self.confidence,
            "source": self.source,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class BarcodeReadResult:
    ok: bool
    device: str | None
    location_id: str | None
    capability_used: str
    decoder_available: bool
    barcodes: tuple[BarcodeCandidate, ...] = ()
    snapshot: Mapping[str, Any] = field(default_factory=dict)
    error: str | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            "ok": bool(self.ok),
            "device": self.device,
            "location_id": self.location_id,
            "capability_used": self.capability_used,
            "decoder_available": bool(self.decoder_available),
            "barcode_count": len(self.barcodes),
            "barcodes": [barcode.to_payload() for barcode in self.barcodes],
            "snapshot": dict(self.snapshot),
            "error": self.error,
        }

    @classmethod
    def from_snapshot(
        cls,
        command: BarcodeReadCommand,
        snapshot: Mapping[str, Any],
        *,
        capability_used: str,
        candidates: Iterable[BarcodeCandidate],
        decoder_available: bool,
    ) -> "BarcodeReadResult":
        barcode_entries = tuple(candidates)
        snapshot_payload = {
            "path": snapshot.get("path"),
            "size": snapshot.get("size"),
            "image_b64": snapshot.get("image_b64") if command.include_image_data else None,
            "image_error": snapshot.get("image_error"),
            "metadata": dict(snapshot.get("metadata") or {}),
        }
        ok = bool(snapshot.get("ok")) and (bool(barcode_entries) or bool(decoder_available))
        return cls(
            ok=ok,
            device=snapshot.get("device"),
            location_id=command.location_id,
            capability_used=str(capability_used),
            decoder_available=bool(decoder_available),
            barcodes=barcode_entries,
            snapshot=snapshot_payload,
            error=snapshot.get("error"),
        )


def decode_barcodes_from_snapshot(
    snapshot: Mapping[str, Any],
    *,
    symbologies: Iterable[str] = (),
) -> tuple[BarcodeCandidate, ...]:
    requested = {
        normalized
        for normalized in (_normalize_symbology(value) for value in symbologies)
        if normalized is not None
    }

    raw_entries: list[Any] = []
    detected = snapshot.get("detected_barcodes")
    if isinstance(detected, list):
        raw_entries.extend(detected)

    metadata = snapshot.get("metadata")
    if isinstance(metadata, Mapping):
        barcodes = metadata.get("barcodes")
        if isinstance(barcodes, list):
            raw_entries.extend(barcodes)

    if snapshot.get("barcode") is not None:
        raw_entries.append(snapshot.get("barcode"))

    candidates: list[BarcodeCandidate] = []
    for entry in raw_entries:
        if isinstance(entry, Mapping):
            value = entry.get("value") or entry.get("data")
            if value is None:
                continue
            symbology = _normalize_symbology(entry.get("symbology") or entry.get("type"))
            if requested and symbology not in requested:
                continue
            confidence = entry.get("confidence")
            candidates.append(
                BarcodeCandidate(
                    value=str(value),
                    symbology=symbology,
                    confidence=float(confidence) if confidence is not None else None,
                    source=str(entry.get("source") or "snapshot_metadata"),
                    metadata={
                        key: value
                        for key, value in entry.items()
                        if key not in {"value", "data", "symbology", "type", "confidence", "source"}
                    },
                )
            )
            continue

        value = str(entry).strip()
        if not value:
            continue
        if requested:
            continue
        candidates.append(BarcodeCandidate(value=value))

    return tuple(candidates)
