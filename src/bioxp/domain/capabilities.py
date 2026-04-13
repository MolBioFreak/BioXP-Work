from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Iterable, Iterator, Mapping


class CapabilityName(str, Enum):
    MOTION = "motion"
    PIPETTE = "pipette"
    BARCODE = "barcode"
    THERMAL = "thermal"
    CHILLER = "chiller"
    INSPECTION = "inspection"
    PROTOCOL_EXECUTION = "protocol_execution"


class MissingCapabilityError(RuntimeError):
    """Raised when a required machine capability is unavailable."""


def normalize_capability_name(value: CapabilityName | str) -> CapabilityName:
    if isinstance(value, CapabilityName):
        return value

    normalized = str(value).strip().lower().replace("-", "_").replace(" ", "_")
    return CapabilityName(normalized)


@dataclass(frozen=True)
class Capability:
    """Typed capability state for a deployment profile."""

    name: CapabilityName
    enabled: bool = True
    reason: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


class CapabilityRegistry:
    """Minimal typed capability registry used for feature gating."""

    def __init__(self, capabilities: Iterable[Capability] | None = None):
        self._capabilities: dict[CapabilityName, Capability] = {}
        for capability in capabilities or ():
            self.register(capability)

    def __iter__(self) -> Iterator[Capability]:
        return iter(self._capabilities.values())

    def __len__(self) -> int:
        return len(self._capabilities)

    def __contains__(self, name: CapabilityName | str) -> bool:
        return normalize_capability_name(name) in self._capabilities

    def register(
        self,
        capability: Capability | CapabilityName | str,
        *,
        enabled: bool = True,
        reason: str | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> Capability:
        if isinstance(capability, Capability):
            entry = capability
        else:
            entry = Capability(
                name=normalize_capability_name(capability),
                enabled=enabled,
                reason=reason,
                metadata=dict(metadata or {}),
            )

        self._capabilities[entry.name] = entry
        return entry

    def get(self, name: CapabilityName | str) -> Capability | None:
        return self._capabilities.get(normalize_capability_name(name))

    def is_enabled(self, name: CapabilityName | str) -> bool:
        capability = self.get(name)
        return bool(capability and capability.enabled)

    def require(self, name: CapabilityName | str) -> Capability:
        capability = self.get(name)
        if capability is None or not capability.enabled:
            reason = capability.reason if capability is not None else "not registered"
            raise MissingCapabilityError(
                f"Capability '{normalize_capability_name(name).value}' is unavailable ({reason})."
            )
        return capability

    def enabled_names(self) -> tuple[CapabilityName, ...]:
        return tuple(
            capability.name
            for capability in self._capabilities.values()
            if capability.enabled
        )

    def as_dict(self) -> dict[str, dict[str, Any]]:
        return {
            capability.name.value: {
                "enabled": capability.enabled,
                "reason": capability.reason,
                "metadata": dict(capability.metadata),
            }
            for capability in self._capabilities.values()
        }

    @classmethod
    def from_config(
        cls,
        data: Mapping[str, Any] | Iterable[str] | None,
    ) -> "CapabilityRegistry":
        registry = cls()
        if data is None:
            return registry

        if isinstance(data, Mapping):
            for raw_name, raw_value in data.items():
                if isinstance(raw_value, Mapping):
                    extra_metadata = {
                        key: value
                        for key, value in raw_value.items()
                        if key not in {"enabled", "reason"}
                    }
                    registry.register(
                        raw_name,
                        enabled=bool(raw_value.get("enabled", True)),
                        reason=raw_value.get("reason"),
                        metadata=extra_metadata,
                    )
                else:
                    registry.register(raw_name, enabled=bool(raw_value))
            return registry

        for raw_name in data:
            registry.register(raw_name, enabled=True)
        return registry
