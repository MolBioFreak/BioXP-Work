from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..protocols.models import ProtocolAction, ProtocolActionKind, ProtocolDocument
from .control_interface import BioXPControlInterface, OperationTrace
from .scripts import OemScript, OemScriptCommand
from .script_handler import BioXPScriptHandler, ScriptExecutionPlan
from .state import VirtualBioXP, VirtualStateEvent
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


@dataclass(frozen=True)
class ProtocolJobDryRunResult:
    ok: bool
    mode: str
    executed: bool
    physical_motion: bool
    lifecycle: list[str]
    action_count: int
    unsupported_action_count: int
    unanchored_action_count: int
    review_required_count: int
    source_path: str
    virtual_state: VirtualBioXP
    events: list[VirtualStateEvent]
    execution_plan: ScriptExecutionPlan | None = None
    proof_ladder: dict[str, Any] = field(default_factory=dict)
    preflight_errors: list[str] = field(default_factory=list)
    artifact: dict[str, Any] = field(default_factory=dict)

    def to_payload(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "mode": self.mode,
            "executed": self.executed,
            "physical_motion": self.physical_motion,
            "lifecycle": list(self.lifecycle),
            "action_count": self.action_count,
            "unsupported_action_count": self.unsupported_action_count,
            "unanchored_action_count": self.unanchored_action_count,
            "review_required_count": self.review_required_count,
            "source_path": self.source_path,
            "preflight_errors": list(self.preflight_errors),
            "virtual_state": self.virtual_state.to_payload(),
            "events": [event.to_payload() for event in self.events],
            "execution_plan": None if self.execution_plan is None else self.execution_plan.to_payload(),
            "proof_ladder": dict(self.proof_ladder),
            "artifact": dict(self.artifact),
        }


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

    def execute_protocol(
        self,
        document: ProtocolDocument,
        *,
        source_path: str = "",
        requested_mode: str = "dry_run",
        operator_ack: str | None = None,
        artifact_root: str | None = None,
    ) -> ProtocolJobDryRunResult:
        mode = str(requested_mode or "dry_run")
        actions = [action for stage in document.stages for action in stage.actions]
        execution_plan = BioXPScriptHandler.dry_run().interpret_protocol(document)
        unsupported_action_count = self._unsupported_action_count(document, actions)
        unanchored_action_count = sum(1 for action in actions if not self._has_oem_source_anchor(action))
        review_required_count = sum(1 for action in actions if action.review_required or action.kind is ProtocolActionKind.PAUSE_REVIEW)
        proof_ladder = self._proof_ladder(
            mode=mode,
            source_anchored=unanchored_action_count == 0,
            zero_unsupported_commands=self._coverage_unsupported_count(document) == 0 and unsupported_action_count == 0,
            virtual_state_complete=True,
            execution_plan=execution_plan,
        )
        preflight_errors: list[str] = []
        if self._coverage_unsupported_count(document) > 0:
            preflight_errors.append("coverage reports unsupported OEM commands")
        if unsupported_action_count:
            preflight_errors.append("protocol contains unsupported action summaries")
        if unanchored_action_count:
            preflight_errors.append("protocol contains actions without OEM source anchors")
        if mode == "live":
            if operator_ack != "I_ACKNOWLEDGE_LIVE_BIOXP_OEM_COMPAT_RISK":
                preflight_errors.append("operator_ack required for live mode")
            if not artifact_root:
                preflight_errors.append("artifact_root required for live mode")
            preflight_errors.extend(proof_ladder["blockers"])

        if preflight_errors:
            artifact = self._protocol_artifact(
                document=document,
                source_path=source_path,
                mode=mode,
                action_count=len(actions),
                unsupported_action_count=unsupported_action_count,
                unanchored_action_count=unanchored_action_count,
                review_required_count=review_required_count,
                preflight_errors=preflight_errors,
                execution_plan=execution_plan,
                proof_ladder=proof_ladder,
                fail_closed=True,
            )
            return ProtocolJobDryRunResult(
                ok=False,
                mode=mode,
                executed=False,
                physical_motion=False,
                lifecycle=["created", "preflight_failed"],
                action_count=len(actions),
                unsupported_action_count=unsupported_action_count,
                unanchored_action_count=unanchored_action_count,
                review_required_count=review_required_count,
                source_path=source_path,
                virtual_state=VirtualBioXP(),
                events=[],
                execution_plan=execution_plan,
                proof_ladder=proof_ladder,
                preflight_errors=preflight_errors,
                artifact=artifact,
            )

        state = VirtualBioXP()
        events = [state.apply_action(action) for action in actions]
        proof_ladder = self._proof_ladder(
            mode=mode,
            source_anchored=True,
            zero_unsupported_commands=True,
            virtual_state_complete=state.actions_applied == len(actions),
            execution_plan=execution_plan,
        )
        artifact = self._protocol_artifact(
            document=document,
            source_path=source_path,
            mode=mode,
            action_count=len(actions),
            unsupported_action_count=0,
            unanchored_action_count=0,
            review_required_count=review_required_count,
            preflight_errors=[],
            virtual_state=state,
            execution_plan=execution_plan,
            proof_ladder=proof_ladder,
            fail_closed=False,
        )
        return ProtocolJobDryRunResult(
            ok=True,
            mode=mode,
            executed=False,
            physical_motion=False,
            lifecycle=["created", "preflighted", "script_interpreted", "planned", "state_applied", "proof_gated", "complete"],
            action_count=len(actions),
            unsupported_action_count=0,
            unanchored_action_count=0,
            review_required_count=review_required_count,
            source_path=source_path,
            virtual_state=state,
            events=events,
            execution_plan=execution_plan,
            proof_ladder=proof_ladder,
            artifact=artifact,
        )

    @staticmethod
    def _coverage_unsupported_count(document: ProtocolDocument) -> int:
        coverage = dict(document.metadata.get("coverage") or {})
        try:
            return int(coverage.get("unsupported_command_count") or 0)
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def _unsupported_action_count(document: ProtocolDocument, actions: list[ProtocolAction]) -> int:
        count = 0
        for action in actions:
            params = dict(action.params or {})
            metadata = dict(action.metadata or {})
            if action.kind is ProtocolActionKind.NOTE and (
                params.get("unsupported_command_count") or metadata.get("unsupported_commands")
            ):
                count += 1
        if count == 0 and BioXPControlLib._coverage_unsupported_count(document) > 0:
            count = BioXPControlLib._coverage_unsupported_count(document)
        return count

    @staticmethod
    def _has_oem_source_anchor(action: ProtocolAction) -> bool:
        metadata = dict(action.metadata or {})
        source_node = metadata.get("source_node")
        return bool(metadata.get("oem_verb") and metadata.get("raw_cmd") and isinstance(source_node, dict) and source_node)

    @staticmethod
    def _proof_ladder(
        *,
        mode: str,
        source_anchored: bool,
        zero_unsupported_commands: bool,
        virtual_state_complete: bool,
        execution_plan: ScriptExecutionPlan,
    ) -> dict[str, Any]:
        blockers: list[str] = []
        for blocker in execution_plan.live_blockers:
            if blocker not in blockers:
                blockers.append(blocker)
        for blocker in (
            "live transport validation not complete",
            "pipette ACK/readback live validation not complete",
            "vision live validation not complete",
        ):
            if blocker not in blockers:
                blockers.append(blocker)
        if not source_anchored:
            blockers.append("OEM source anchoring incomplete")
        if not zero_unsupported_commands:
            blockers.append("unsupported OEM commands remain")
        if not virtual_state_complete:
            blockers.append("virtual state replay incomplete")
        return {
            "mode": mode,
            "dry_run_clean": mode == "dry_run" and source_anchored and zero_unsupported_commands and virtual_state_complete,
            "source_anchored": source_anchored,
            "zero_unsupported_commands": zero_unsupported_commands,
            "transport_reply_contract_ready": False,
            "virtual_state_complete": virtual_state_complete,
            "pipette_ack_readback_ready": False,
            "vision_artifact_contract_ready": True,
            "operator_ack_required": True,
            "artifact_root_required": True,
            "execution_plan_step_count": execution_plan.step_count,
            "live_allowed": False,
            "blockers": blockers,
        }

    @staticmethod
    def _protocol_artifact(
        *,
        document: ProtocolDocument,
        source_path: str,
        mode: str,
        action_count: int,
        unsupported_action_count: int,
        unanchored_action_count: int,
        review_required_count: int,
        preflight_errors: list[str],
        execution_plan: ScriptExecutionPlan,
        proof_ladder: dict[str, Any],
        virtual_state: VirtualBioXP | None = None,
        fail_closed: bool,
    ) -> dict[str, Any]:
        return {
            "artifact_format": "bioxp-oem-runtime-parity-v1",
            "compliance_gate": "runtime_parity_pipeline",
            "protocol_id": document.protocol_id,
            "source_path": source_path,
            "mode": mode,
            "executed": False,
            "opened_usb": False,
            "physical_motion": False,
            "fail_closed": fail_closed,
            "action_count": int(action_count),
            "unsupported_action_count": int(unsupported_action_count),
            "unanchored_action_count": int(unanchored_action_count),
            "review_required_count": int(review_required_count),
            "preflight_errors": list(preflight_errors),
            "virtual_state": {} if virtual_state is None else virtual_state.to_payload(),
            "execution_plan": execution_plan.to_payload(),
            "proof_ladder": dict(proof_ladder),
        }

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
