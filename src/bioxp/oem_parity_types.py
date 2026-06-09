
"""Shared no-hardware OEM parity dataclasses."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Mapping


@dataclass(frozen=True)
class OemSourceAnchor:
    file: str
    symbol: str
    lines: str
    sha256: str = "source-oracle"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class OemProgramStep:
    step_id: str
    source: OemSourceAnchor
    operation: str
    axis: str | None = None
    board: str | None = None
    motor: int | None = None
    params: Mapping[str, Any] = field(default_factory=dict)
    wait_ms: int | None = None
    branch_condition: str | None = None
    side_effects: tuple[str, ...] = ()
    failure_modes: tuple[str, ...] = ()
    safety_deviations: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["params"] = dict(self.params)
        return payload


@dataclass(frozen=True)
class OemProgramSpec:
    name: str
    oem_symbol: str
    source_mode: str
    live_allowed_default: bool
    steps: tuple[OemProgramStep, ...]
    required_artifact_fields: tuple[str, ...] = ()
    blockers: tuple[str, ...] = ()
    parity_label: str = "source_model_only_no_usb"

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "oem_symbol": self.oem_symbol,
            "source_mode": self.source_mode,
            "live_allowed_default": self.live_allowed_default,
            "steps": [s.to_dict() for s in self.steps],
            "required_artifact_fields": list(self.required_artifact_fields),
            "blockers": list(self.blockers),
            "parity_label": self.parity_label,
        }
