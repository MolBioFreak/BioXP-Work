from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PipettePlan:
    operation: str
    status: str = "planned"
    executed: bool = False


class PipetteController:
    """Dry-run placeholder preserving OEM pipette control-surface names."""

    def initiate_group(self) -> PipettePlan:
        return PipettePlan("initiate_group")

    def query_tip_status(self) -> PipettePlan:
        return PipettePlan("query_tip_status")

    def query_pressure(self) -> PipettePlan:
        return PipettePlan("query_pressure")

    def aspirate(self, *_, **__) -> PipettePlan:
        return PipettePlan("aspirate")

    def dispense(self, *_, **__) -> PipettePlan:
        return PipettePlan("dispense")
