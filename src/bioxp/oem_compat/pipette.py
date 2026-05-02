from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class PipettePlan:
    operation: str
    command_ascii: str
    status: str = "planned"
    executed: bool = False
    requires_readback: bool = True


@dataclass
class PipetteController:
    """Dry-run ClassPipette/ClassPipetteCollection-compatible facade."""

    plans: list[PipettePlan] = field(default_factory=list)

    @classmethod
    def dry_run(cls) -> "PipetteController":
        return cls()

    def _plan(self, operation: str, command_ascii: str, *, requires_readback: bool = True) -> PipettePlan:
        p = PipettePlan(operation=operation, command_ascii=command_ascii, requires_readback=requires_readback)
        self.plans.append(p)
        return p

    def initiate_group(self) -> PipettePlan:
        return self._plan("initiate_group", "WR")

    def dispense_all(self) -> PipettePlan:
        return self._plan("dispense_all", "A0R")

    def eject_tip(self, *, initialized: bool = True) -> PipettePlan:
        return self._plan("eject_tip", "E1R" if initialized else "E0R")

    def query_tip_status(self) -> PipettePlan:
        return self._plan("query_tip_status", "?31")

    def query_pressure(self) -> PipettePlan:
        return self._plan("query_pressure", "?57")

    def query_error_log(self, n: int = 0) -> PipettePlan:
        return self._plan("query_error_log", f"?E{int(n)}")

    def aspirate(self, volume: float = 0, *_args, **_kwargs) -> PipettePlan:
        return self._plan("aspirate", f"P{volume},1R")

    def dispense(self, volume: float = 0, *_args, **_kwargs) -> PipettePlan:
        return self._plan("dispense", f"D{volume},1R")

    def mix_all(self, count: int = 1, vol: float = 0, vigorous: int = 100) -> PipettePlan:
        return self._plan("mix_all", f"M{vol},{count},{vigorous}R")

    def detect_fluid(self) -> PipettePlan:
        return self._plan("detect_fluid", "BR")

    def enable_pressure_stream(self, enabled: bool) -> PipettePlan:
        return self._plan("enable_pressure_stream", "o0,1R" if enabled else "o0,0R")

    def disable_heartbeat_message(self) -> PipettePlan:
        return self._plan("disable_heartbeat_message", "U61R")

    # Backwards-compatible names used by earlier POC tests.
    def initiateGroup(self) -> PipettePlan:  # noqa: N802
        return self.initiate_group()
