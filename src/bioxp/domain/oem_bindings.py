from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Mapping

from .deck import DeckLayout, load_deck_layout

OEM_BINDING_SCHEMA = "bioxp-oem-binding-v1"
_ALLOWED_SECTIONS = {
    "position_table",
    "motion_constants",
    "calibration_reference",
    "vision_calibration",
    "pipette_calibration",
    "process_time",
    "thermal_chiller",
    "script_corpus",
    "deck_semantics",
}


class OemBindingStatus(str, Enum):
    """Availability of a source-backed OEM binding section."""

    AVAILABLE = "available"
    UNAVAILABLE = "unavailable"
    UNSUPPORTED = "unsupported"
    NOT_EXTRACTED = "not_extracted"


@dataclass(frozen=True)
class OemBindingSource:
    """Source provenance for extracted OEM config/calibration data."""

    source_path: str
    source_type: str
    source_key: str
    source_hash: str | None = None

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "OemBindingSource":
        missing = [key for key in ("source_path", "source_type", "source_key") if not data.get(key)]
        if missing:
            raise ValueError(f"OEM binding source missing required field(s): {', '.join(missing)}")
        return cls(
            source_path=str(data["source_path"]),
            source_type=str(data["source_type"]),
            source_key=str(data["source_key"]),
            source_hash=str(data["source_hash"]) if data.get("source_hash") is not None else None,
        )

    def as_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "source_path": self.source_path,
            "source_type": self.source_type,
            "source_key": self.source_key,
        }
        if self.source_hash is not None:
            payload["source_hash"] = self.source_hash
        return payload


@dataclass(frozen=True)
class OemBindingSection:
    """One source-backed OEM binding section."""

    name: str
    status: OemBindingStatus
    source_key: str | None = None
    reason: str | None = None
    payload: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, name: str, data: Mapping[str, Any]) -> "OemBindingSection":
        if name not in _ALLOWED_SECTIONS:
            raise ValueError(f"Unsupported OEM binding section '{name}'")
        try:
            status = OemBindingStatus(str(data["status"]))
        except KeyError as exc:
            raise ValueError(f"OEM binding section '{name}' missing status") from exc
        except ValueError as exc:
            raise ValueError(f"OEM binding section '{name}' has unsupported status '{data.get('status')}'") from exc

        payload = {
            key: value
            for key, value in data.items()
            if key not in {"status", "source_key", "reason"}
        }
        if status is OemBindingStatus.AVAILABLE:
            _validate_available_section(name, payload)
        elif not data.get("reason"):
            raise ValueError(f"OEM binding section '{name}' with status '{status.value}' requires reason")

        return cls(
            name=name,
            status=status,
            source_key=str(data.get("source_key")) if data.get("source_key") is not None else None,
            reason=str(data.get("reason")) if data.get("reason") is not None else None,
            payload=payload,
        )

    def as_dict(self) -> dict[str, Any]:
        payload = dict(self.payload)
        payload["status"] = self.status.value
        if self.source_key is not None:
            payload["source_key"] = self.source_key
        if self.reason is not None:
            payload["reason"] = self.reason
        return payload


@dataclass(frozen=True)
class OemBindingData:
    """Read-only source-backed OEM config/calibration binding."""

    schema: str
    source: OemBindingSource
    sections: Mapping[str, OemBindingSection] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "OemBindingData":
        schema = str(data.get("schema", ""))
        if schema != OEM_BINDING_SCHEMA:
            raise ValueError(f"OEM binding schema must be '{OEM_BINDING_SCHEMA}', got '{schema}'")
        source_data = data.get("source")
        if not isinstance(source_data, Mapping):
            raise ValueError("OEM binding source must be a mapping with source_path/source_type/source_key")
        sections_data = data.get("sections")
        if not isinstance(sections_data, Mapping):
            raise ValueError("OEM binding sections must be a mapping")
        sections = {
            str(name): OemBindingSection.from_mapping(str(name), section)
            for name, section in sections_data.items()
            if _require_mapping(section, f"OEM binding section '{name}'")
        }
        return cls(
            schema=schema,
            source=OemBindingSource.from_mapping(source_data),
            sections=sections,
        )

    def as_metadata(self) -> dict[str, Any]:
        return {
            "schema": self.schema,
            "source": self.source.as_dict(),
            "sections": {
                name: section.as_dict()
                for name, section in self.sections.items()
            },
        }


def _require_mapping(value: Any, label: str) -> bool:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return True


def _validate_available_section(name: str, payload: Mapping[str, Any]) -> None:
    if name == "position_table" and not isinstance(payload.get("entries"), Mapping):
        raise ValueError("Available OEM position_table binding requires entries mapping")


def load_oem_binding_data(path: str | Path) -> OemBindingData:
    """Load a read-only OEM binding file without touching hardware or live services."""

    binding_path = Path(path)
    raw_text = binding_path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError:
        yaml = None

    if yaml is not None and binding_path.suffix.lower() in {".yaml", ".yml"}:
        data = yaml.safe_load(raw_text)
    else:
        data = json.loads(raw_text)
    if not isinstance(data, Mapping):
        raise ValueError(f"OEM binding '{binding_path}' must decode to a mapping")
    return OemBindingData.from_mapping(data)


def bind_oem_metadata_to_deck_layout(
    deck_layout_path: str | Path | None,
    binding_path: str | Path,
) -> DeckLayout:
    """Return a DeckLayout copy annotated with read-only OEM binding provenance."""

    layout = load_deck_layout(deck_layout_path)
    binding = load_oem_binding_data(binding_path)
    metadata = dict(layout.metadata)
    metadata["oem_binding"] = binding.as_metadata()
    return DeckLayout(
        deck_id=layout.deck_id,
        version=layout.version,
        slots=layout.slots,
        locations=layout.locations,
        capabilities=layout.capabilities,
        metadata=metadata,
    )
