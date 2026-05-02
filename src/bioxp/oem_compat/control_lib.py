from __future__ import annotations

from dataclasses import dataclass, field

from .control_interface import BioXPControlInterface, OperationTrace
from .scripts import OemScript, OemScriptCommand
from .transport import DryRunTransport


@dataclass(frozen=True)
class StartupReport:
    ok: bool
    mode: str
    physical_motion: bool
    traces: list[OperationTrace]

    @property
    def trace_names(self) -> list[str]:
        return [trace.name for trace in self.traces]


@dataclass(frozen=True)
class PlannedScriptAction:
    index: int
    verb: str
    raw: str
    status: str = "planned"


@dataclass(frozen=True)
class ScriptDryRunResult:
    mode: str
    executed: bool
    actions: list[PlannedScriptAction] = field(default_factory=list)


class BioXPControlLib:
    """Workstation-safe ControlLib-compatible facade.

    The methods intentionally return dry-run reports. This is the compatibility
    seam that will later wrap live/shadow transports on the robot.
    """

    def __init__(self, control_interface: BioXPControlInterface):
        self.control_interface = control_interface
        self.transport: DryRunTransport = control_interface.transport

    @classmethod
    def dry_run(cls) -> "BioXPControlLib":
        return cls(BioXPControlInterface.dry_run())

    def initial_check(self) -> StartupReport:
        trace = OperationTrace("initial_check")
        trace.add("check_mode", value="dry_run")
        return StartupReport(ok=True, mode="dry_run", physical_motion=False, traces=[trace])

    def initialize_motion(self) -> StartupReport:
        trace = self.control_interface.initialize_motors_without_motion()
        return StartupReport(ok=True, mode="dry_run", physical_motion=False, traces=[trace])

    def startup(self, *, run_homing: bool = True) -> StartupReport:
        traces = [self.control_interface.initialize_motors_without_motion()]
        if run_homing:
            traces.append(self.control_interface.startup_homing())
        return StartupReport(ok=True, mode="dry_run", physical_motion=False, traces=traces)

    def selftest(self) -> StartupReport:
        trace = OperationTrace("selftest")
        trace.add("selftest", value="planned")
        return StartupReport(ok=True, mode="dry_run", physical_motion=False, traces=[trace])

    def execute_script(self, script: OemScript) -> ScriptDryRunResult:
        return ScriptDryRunResult(
            mode="dry_run",
            executed=False,
            actions=[self._plan_command(cmd) for cmd in script.commands],
        )

    def run_job(self, script: OemScript) -> ScriptDryRunResult:
        return self.execute_script(script)

    def _plan_command(self, command: OemScriptCommand) -> PlannedScriptAction:
        return PlannedScriptAction(index=command.index, verb=command.verb, raw=command.raw)

    def pause_script(self) -> dict:
        return {"mode": "dry_run", "status": "planned", "action": "pause"}

    def resume_job(self) -> dict:
        return {"mode": "dry_run", "status": "planned", "action": "resume"}

    def stop_script(self) -> dict:
        return {"mode": "dry_run", "status": "planned", "action": "stop"}

    def cleanup(self) -> dict:
        return {"mode": "dry_run", "status": "planned", "action": "cleanup"}
