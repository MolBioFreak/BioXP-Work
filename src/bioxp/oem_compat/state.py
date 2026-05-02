from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class MachineState:
    """Minimal OEM-compatible semantic state placeholder for workstation dry-runs."""

    gantry_location: str | None = None
    plate_on_gantry: str | None = None
    tips_loaded: dict[int, bool] = field(default_factory=dict)
    fluids: dict[str, Any] = field(default_factory=dict)

    def update_gantry_location(self, location: str) -> None:
        self.gantry_location = str(location)

    def update_tip_status(self, channel: int, loaded: bool) -> None:
        self.tips_loaded[int(channel)] = bool(loaded)
