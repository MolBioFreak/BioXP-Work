from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from contextvars import copy_context
from time import monotonic
from dataclasses import dataclass
from threading import Condition
from typing import Any, Callable, Mapping

from .models import ProtocolAction, ProtocolActionKind, ProtocolDocument, normalize_action_kind
from .runtime_state import ProtocolRuntimeState, ProtocolStageState, ProtocolWorkflowState, ProtocolSourceModel, StageExecutionStatus
from .validators import validate_protocol_document, validate_protocol_support

ActionHandler = Callable[[ProtocolAction, ProtocolRuntimeState], Mapping[str, Any] | Future]

# Core.scriptInterpretor source event domains: predecessor joins, NOT
# physical resource grants (which remain in the canonical command owner).
SOURCE_DOMAINS = {
    **{op: ("General",) for op in (
        "catchPlate", "catch", "cc", "delaypoint", "dopen", "dclose", "la",
        "led", "mov", "ms", "park", "pressp", "releasePlate", "release",
        "retip", "snapshot", "so", "step", "wait",
    )},
    **{op: ("General", "Tip") for op in (
        "aa", "ampmix", "da", "dsa", "ejt", "ldtip", "masp", "mmix", "rmb", "sweep",
    )},
    "cutseal": ("General", "Gripper"), "iniPipette": ("Tip",),
    "sp": ("TC", "General"), "splid": ("TC",),
}
MOTION_DOMAINS = ("General", "Tip", "Gripper")
ALL_DOMAINS = (*MOTION_DOMAINS, "TC")
LIFECYCLE_HOOKS = frozenset({
    "prepare", "run_job", "script_prologue", "ordinary_pause_prepare",
    "ordinary_pause_restore", "deferred_pause_request", "deferred_pause_enter",
    "wake", "safe_stop_request", "safe_stop_exit", "abort_true_prefix", "abort_true_finish",
    "abort_false", "source_error_request", "source_error", "cleanup", "epilogue_sweep",
    "epilogue_lid", "epilogue_park", "script_finally",
})


@dataclass(frozen=True)
class OwnedOperation:
    """An actually entered child, retained to its source-return boundary."""
    future: Future
    domains: tuple[str, ...]
    command_id: str | None = None

    def __post_init__(self):
        if not isinstance(self.future, Future) or set(self.domains) - set(ALL_DOMAINS):
            raise ValueError("Invalid owned source operation")


class _Diversion(Exception):
    """Host progression only; never cancellation of an entered native call."""


class ProtocolExecutor:
    def __init__(
        self,
        *,
        dry_run: bool = True,
        job_id: str | None = None,
        handlers: Mapping[ProtocolActionKind | str, ActionHandler] | None = None,
        oem_handlers: Mapping[str, ActionHandler] | None = None,
        lifecycle_handlers: Mapping[str, Callable] | None = None,
        on_state_change: Callable[[ProtocolRuntimeState], None] | None = None,
        before_native_entry: Callable[[str, ProtocolRuntimeState], None] | None = None,
        source_script_begin: Callable[[ProtocolRuntimeState], Mapping[str, Any]] | None = None,
        source_script_returned: Callable[[ProtocolRuntimeState], Mapping[str, Any]] | None = None,
    ) -> None:
        self.dry_run = bool(dry_run)
        self.job_id = job_id
        self._handlers = {
            normalize_action_kind(kind): handler
            for kind, handler in (handlers or {}).items()
        }
        self._oem_handlers = dict(oem_handlers or {})
        self._lifecycle_handlers = dict(lifecycle_handlers or {})
        if set(self._oem_handlers) - set(SOURCE_DOMAINS):
            raise ValueError("Unknown OEM operation binding")
        if set(self._lifecycle_handlers) - LIFECYCLE_HOOKS:
            raise ValueError("Unknown lifecycle binding")
        self._on_state_change = on_state_change
        self._before_native_entry = before_native_entry
        if (source_script_begin is None) != (source_script_returned is None):
            raise ValueError("Source lifetime callbacks must be supplied as a pair")
        if source_script_begin is not None and not all(callable(cb) for cb in (source_script_begin, source_script_returned)):
            raise ValueError("Source lifetime callbacks must be callable")
        self._source_script_begin = source_script_begin
        self._source_script_returned = source_script_returned
        self._source_entered = False
        self._source_return_notified = False
        self._source_returned = False
        self._source_cancelled = False
        self._source_stop_requested = False
        self._source_host_finalized = False
        self._source_workflow = None
        self._source_wrappers: list[Future] = []
        self._condition = Condition()
        self._state: ProtocolRuntimeState | None = None
        self._owned: list[tuple[OwnedOperation, dict[str, Any]]] = []
        self._controls: dict[str, dict[str, Any]] = {}
        self._termination: str | None = None
        self._interrupted = False
        self._recording_failed = False
        self._failed = False
        self._unknown = False
        self._pause: tuple[str, str] | None = None
        self._wake_control: str | None = None
        self._wake_complete = False
        self._release_gate: str | None = None
        self._review_released = False
        self._hooks_done: set[str] = set()
        self._servicing = False
        self._settling = False
        self._active = False
        self._oem = False
        self.outcome: str | None = None
        self._pool: ThreadPoolExecutor | None = None
        self._reached_controls: set[str] = set()

    def execute(
        self,
        document: ProtocolDocument,
        *,
        state: ProtocolRuntimeState | None = None,
    ) -> ProtocolRuntimeState:
        if not self.dry_run:
            validate_protocol_support(document, handlers=self._handlers, oem_handlers=self._oem_handlers,
                                      lifecycle_handlers=self._lifecycle_handlers,
                                      required_lifecycle=self.required_lifecycle(document))
        if not self.dry_run and (self.job_id or (state and state.workflow)):
            return self._execute_workflow(document, state=state)
        if not self.dry_run and any(a.kind.value == "oem_operation" for s in document.stages for a in s.actions):
            raise ValueError("OEM execution requires canonical workflow custody")
        return self._execute_legacy(document, state=state)

    def _execute_legacy(
        self, document: ProtocolDocument, *, state: ProtocolRuntimeState | None = None,
    ) -> ProtocolRuntimeState:
        validate_protocol_document(document)
        runtime_state = state or ProtocolRuntimeState.from_document(
            document,
            dry_run=self.dry_run,
            job_id=self.job_id,
        )
        if runtime_state.protocol_id != document.protocol_id:
            raise ValueError(
                f"Protocol state belongs to '{runtime_state.protocol_id}', expected '{document.protocol_id}'."
            )
        if bool(runtime_state.dry_run) != self.dry_run:
            raise ValueError(
                f"Protocol state dry_run={runtime_state.dry_run} does not match executor dry_run={self.dry_run}."
            )
        if runtime_state.completed:
            return runtime_state
        if state is not None and runtime_state.awaiting_review:
            raise ValueError("Protocol state is awaiting review and cannot be resumed until the review gate is cleared.")

        self._ensure_stage_states(document, runtime_state)
        runtime_state.current_stage_id = None
        runtime_state.paused = False
        runtime_state.awaiting_review = False
        runtime_state.completed = False
        runtime_state.pause_reason = None
        if state is None or not runtime_state.events:
            runtime_state.record_event("protocol_started", detail={"dry_run": self.dry_run})
        else:
            runtime_state.record_event("protocol_resumed", detail={"dry_run": self.dry_run})

        for stage in document.stages:
            stage_state = runtime_state.stage_states[stage.stage_id]
            if self._stage_is_complete(stage, stage_state):
                stage_state.current_action_id = None
                stage_state.pause_marker_action_id = None
                stage_state.status = StageExecutionStatus.COMPLETED
                continue

            runtime_state.current_stage_id = stage.stage_id
            stage_state.title = stage.title
            stage_state.review_required = bool(stage.review_required)
            stage_state.current_action_id = None
            if stage_state.completed_actions or stage_state.status is StageExecutionStatus.PAUSED:
                runtime_state.record_event(
                    "stage_resumed",
                    stage_id=stage.stage_id,
                    detail={"completed_actions": list(stage_state.completed_actions)},
                )
            else:
                runtime_state.record_event("stage_started", stage_id=stage.stage_id)
            stage_state.status = StageExecutionStatus.RUNNING

            completed_actions = set(stage_state.completed_actions)
            for action in stage.actions:
                if action.action_id in completed_actions:
                    continue
                stage_state.current_action_id = action.action_id
                runtime_state.record_event(
                    "action_started",
                    stage_id=stage.stage_id,
                    action_id=action.action_id,
                )
                action_result = self._execute_action(action, runtime_state)
                runtime_state.action_results.append(
                    {
                        "stage_id": stage.stage_id,
                        "action_id": action.action_id,
                        "kind": action.kind.value,
                        **action_result,
                    }
                )
                if action_result.get("ok") is not True:
                    stage_state.status = StageExecutionStatus.FAILED
                    runtime_state.completed = False
                    runtime_state.record_event(
                        "action_failed",
                        stage_id=stage.stage_id,
                        action_id=action.action_id,
                        detail=dict(action_result),
                    )
                    return runtime_state
                stage_state.completed_actions.append(action.action_id)
                completed_actions.add(action.action_id)
                stage_state.current_action_id = None
                runtime_state.record_event(
                    "action_completed",
                    stage_id=stage.stage_id,
                    action_id=action.action_id,
                    detail=dict(action_result),
                )

                if action.review_required:
                    return self._pause_for_review(
                        runtime_state,
                        stage_id=stage.stage_id,
                        action_id=action.action_id,
                        message=action.pause_message,
                    )

            stage_state.current_action_id = None
            if stage.review_required and not self._stage_review_was_acknowledged(stage, stage_state):
                return self._pause_for_review(
                    runtime_state,
                    stage_id=stage.stage_id,
                    action_id=None,
                    message=f"Review required after stage '{stage.stage_id}'",
                )

            stage_state.status = StageExecutionStatus.COMPLETED
            stage_state.pause_marker_action_id = None
            runtime_state.record_event("stage_completed", stage_id=stage.stage_id)

        runtime_state.current_stage_id = None
        runtime_state.paused = False
        runtime_state.awaiting_review = False
        runtime_state.completed = True
        runtime_state.pause_reason = None
        runtime_state.record_event("protocol_completed")
        return runtime_state

    @staticmethod
    def _explicit_result(result: Any) -> dict[str, Any]:
        if not isinstance(result, Mapping) or type(result.get("ok")) is not bool:
            return {"ok": False, "error": "missing_explicit_outcome", "uncertain": True}
        return dict(result)

    @staticmethod
    def required_lifecycle(document: ProtocolDocument) -> tuple[str, ...]:
        """Exact selected lifecycle closure, also consumed by service preflight."""
        oem = document.metadata.get("input_mode") == "oem_prepared" or any(
            a.kind.value == "oem_operation" for s in document.stages for a in s.actions
        )
        if not oem:
            return ()
        # Control/termination exits belong to the selected job even before a
        # control is requested. Physical leaves are supplied by the canonical factory.
        required = LIFECYCLE_HOOKS - {"prepare", "ordinary_pause_restore"}
        if document.metadata.get("oem_prepare") is True:
            required = required | {"prepare"}
        return tuple(sorted(required))

    def preflight(self, document: ProtocolDocument) -> dict[str, Any]:
        validate_protocol_document(document)
        required = self.required_lifecycle(document)
        missing = ["lifecycle:" + key for key in required if key not in self._lifecycle_handlers]
        if required and not self.dry_run and self._source_script_begin is None:
            missing.extend(("source_script_begin", "source_script_returned"))
        for stage in document.stages:
            for action in stage.actions:
                if action.kind.value == "oem_operation":
                    opcode = action.oem_opcode
                    if opcode not in SOURCE_DOMAINS:
                        missing.append("opcode:" + str(opcode))
                    elif opcode not in {"step", "delaypoint", "wait"} and opcode not in self._oem_handlers:
                        missing.append("opcode:" + str(opcode))
                elif action.kind not in {ProtocolActionKind.NOTE, ProtocolActionKind.PAUSE_REVIEW} and action.kind not in self._handlers:
                    missing.append("kind:" + action.kind.value)
                if action.params.get("requires_virtual_bioxp_state") or action.params.get("macro_verb") or action.kind == ProtocolActionKind.LOOP_MARKER:
                    missing.append("scientific_generator")
        if document.metadata.get("requires_generator_expansion"):
            missing.append("scientific_generator")
        if document.metadata.get("execution_mode", "normal") != "normal":
            missing.append("normal_execution_mode")
        if self._before_native_entry is None:
            missing.append("canonical_native_entry_check")
        return {"ok": not missing, "required_lifecycle": list(required), "missing": sorted(set(missing))}

    def _publish(self, phase: str | None = None, *, held_reason: str | None = None) -> None:
        state = self._state
        workflow = state.workflow
        if phase is not None:
            workflow.phase = phase
        if held_reason is not None:
            workflow.held_reason = held_reason
        state.paused = workflow.gate in {"ordinary_pause", "deferred_pause", "review"}
        state.awaiting_review = workflow.gate == "review"
        state.pause_reason = workflow.held_reason if workflow.gate else None
        if self._on_state_change is not None and not self._recording_failed:
            try:
                self._on_state_change(state)
            except Exception as exc:
                # Storage failure is not a native failure or cleanup request.
                self._recording_failed = True
                workflow.phase = "reconciling"
                workflow.held_reason = "recording_failed"
                state.record_event("workflow_recording_failed", detail={"error_type": type(exc).__name__})
                self._notify()

    def _entry(self, identity: str) -> None:
        if self._interrupted or self._recording_failed or self._unknown:
            raise _Diversion()
        try:
            self._before_native_entry(identity, self._state)
        except Exception as exc:
            self._unknown = True
            self._state.record_event("native_entry_refused", detail={"identity": identity, "error_type": type(exc).__name__})
            self._publish("reconciling", held_reason="native_authority_unavailable")
            raise _Diversion() from exc

    def request_control(self, action: str, *, control_id: str, mode: str | None = None,
                        gate: str | None = None, gate_id: str | None = None) -> dict[str, Any]:
        """F/C first validate durable identity/generation; no native IO here."""
        request = {"action": action, **({"mode": mode} if mode else {}),
                   **({"gate": gate} if gate else {}), **({"gate_id": gate_id} if gate_id else {})}
        with self._condition:
            if control_id in self._controls:
                if self._controls[control_id] != request:
                    raise ValueError("Control identity conflicts with earlier request")
                return self._control_result(control_id)
            if not control_id or not self._active or self._interrupted or self._recording_failed:
                raise ValueError("Workflow control has no eligible active owner")
            workflow = self._state.workflow
            if action == "pause":
                if (self._termination or (workflow.gate and not (workflow.gate == "delaypoint" and mode == "deferred"))
                        or self._pause or mode not in {"ordinary", "deferred"}):
                    raise ValueError("Pause is not eligible")
                if not self._oem:
                    raise ValueError("OEM pause requires OEM lifecycle")
                self._pause = (mode, control_id)
            elif action == "wake":
                if self._termination or workflow.phase != "waiting" or workflow.gate != "deferred_pause" or gate_id != workflow.gate_id or self._wake_complete or self._wake_control:
                    raise ValueError("Wake requires the reached deferred gate")
                self._wake_control = control_id
            elif action == "continue":
                if self._termination or gate not in {"ordinary_pause", "deferred_pause", "delaypoint"} or gate != workflow.gate or gate_id != workflow.gate_id:
                    raise ValueError("Continue requires the matching gate occurrence")
                if gate == "deferred_pause" and not self._wake_complete:
                    raise ValueError("Deferred Continue requires completed full wake")
                self._release_gate = control_id
            elif action in {"safe_stop", "abort"}:
                # Source errors win; once selected a different termination chain
                # is not launched by a competing control.
                if self._termination is None:
                    self._termination = "source_error" if self._failed else action
            else:
                raise ValueError("Unknown workflow control")
            self._controls[control_id] = request
            workflow.last_control_id = control_id
            workflow.requested_control = dict(request)
            self._condition.notify_all()
            return self._control_result(control_id)

    def _control_result(self, control_id: str) -> dict[str, Any]:
        workflow = self._state.workflow
        return {"accepted": True, "reached": control_id in self._reached_controls,
                "control_command_id": control_id, "phase": workflow.phase,
                "gate": workflow.gate, "gate_id": workflow.gate_id}

    def acknowledge_review(self, *, control_id: str, gate_id: str) -> dict[str, Any]:
        with self._condition:
            request = {"action": "review", "gate_id": gate_id}
            if control_id in self._controls:
                if self._controls[control_id] != request:
                    raise ValueError("Control identity conflicts with earlier request")
                return self._control_result(control_id)
            if not self._active or self._termination or self._interrupted or self._recording_failed:
                raise ValueError("Review has no eligible active owner")
            workflow = self._state.workflow
            if workflow.gate != "review" or workflow.gate_id != gate_id:
                raise ValueError("Review cannot release another gate")
            self._controls[control_id] = request
            self._review_released = True
            self._release_gate = control_id
            workflow.last_control_id = control_id
            self._condition.notify_all()
            return self._control_result(control_id)

    def interrupt(self, *, control_id: str, affected: bool = True) -> None:
        # Physical Stop is exclusively the existing independent interrupt owner.
        if not affected:
            return
        with self._condition:
            self._interrupted = True  # monotonic for this attempt
            if self._state is not None:
                self._state.record_event("workflow_interrupted", detail={"control_id": control_id})
            self._condition.notify_all()

    def source_error(self, *, false_abort: bool = False) -> None:
        with self._condition:
            self._failed = True
            if false_abort or self._termination not in {"source_error", "abort_false"}:
                self._termination = "abort_false" if false_abort else "source_error"
            self._condition.notify_all()

    def _retain(self, owned: OwnedOperation, result: dict[str, Any]) -> None:
        with self._condition:
            self._owned.append((owned, result))
        if owned.command_id and owned.command_id not in self._state.workflow.child_command_ids:
            self._state.workflow.child_command_ids.append(owned.command_id)
        owned.future.add_done_callback(lambda _: self._notify())

    def start_child(self, name: str, operation: Callable, *, domains: tuple[str, ...]) -> Future:
        """Retain a source-prescribed nested task in this same executor."""
        if not name or not domains or set(domains) - set(ALL_DOMAINS):
            raise ValueError("Nested source task requires a name and source domains")
        if not self._active or self._pool is None:
            raise ValueError("Nested source task has no active executor")
        identity = name
        self._entry(identity)
        def entered():
            self._entry(identity)
            return operation()
        future = self._pool.submit(copy_context().run, entered)
        result = {"kind": "owned_child", "action_id": identity, "pending": True}
        with self._condition:
            self._state.action_results.append(result)
            self._retain(OwnedOperation(future, domains), result)
        return future

    def source_stopped(self) -> bool:
        """Existing source stop condition; not a new cancellation mechanism."""
        with self._condition:
            return bool(self._source_stop_requested or self._source_cancelled
                        or self._interrupted or self._unknown or self._recording_failed)

    def _validate_source_owner(self, state: ProtocolRuntimeState) -> None:
        if (state is not self._state or not self._active or not self._source_entered
                or state.workflow is not self._source_workflow
                or state.workflow.command_id != self.job_id):
            raise ValueError("Source callback has no matching active attempt")

    def cancel_source(self, state: ProtocolRuntimeState) -> dict[str, Any]:
        """Mechanical safe-exit token intent; never a native completion signal."""
        with self._condition:
            self._validate_source_owner(state)
            if self._termination not in {"abort", "abort_false", "safe_stop", "source_error"}:
                raise ValueError("Source cancellation has no selected active termination")
            self._source_cancelled = True
            self._source_stop_requested = True
            self._condition.notify_all()
            return {"ok": True, "source_stop_scripts": True}

    def finalize_source_host(self, state: ProtocolRuntimeState) -> dict[str, Any]:
        """Parent's script_finally binding: host gates only, no IO or signal."""
        with self._condition:
            self._validate_source_owner(state)
            if not self._source_return_notified:
                raise ValueError("Source host finalization precedes actual wrapper return")
            self._source_host_finalized = True
            self._pause = self._wake_control = self._release_gate = None
            self._wake_complete = self._review_released = False
            state.workflow.gate = state.workflow.gate_id = None
            state.paused = state.awaiting_review = False
            state.pause_reason = None
            self._condition.notify_all()
            return {"ok": True, "delivery_attempted": False, "source_script_finalized": True}

    def register_child(self, command_id: str) -> None:
        """Canonical owner calls at actual admission, preserving admission order."""
        if not isinstance(command_id, str) or not command_id:
            raise ValueError("Child requires actual canonical identity")
        with self._condition:
            if command_id not in self._state.workflow.child_command_ids:
                self._state.workflow.child_command_ids.append(command_id)

    def _notify(self) -> None:
        with self._condition:
            self._condition.notify_all()

    def _consume(self, value: Any, result: dict[str, Any]) -> None:
        payload = self._explicit_result(value)
        children = payload.pop("owned_children", ())
        result.update(payload)
        result.pop("pending", None)
        command_id = payload.get("command_id")
        if command_id and command_id not in self._state.workflow.child_command_ids:
            self._state.workflow.child_command_ids.append(command_id)
        for child in children:
            if not isinstance(child, OwnedOperation):
                self._unknown = True
                result["owned_child_error"] = "invalid_owned_operation"
                continue
            child_result = {"kind": "owned_child", "parent_action_id": result.get("action_id"), "pending": True}
            self._state.action_results.append(child_result)
            self._retain(child, child_result)
        if payload.get("source_board_error_event") and self._source_entered:
            self.source_error(false_abort=True)
        elif payload.get("source_error_hold"):
            self._failed = True
            self._state.workflow.held_reason = "source_error_hold"
            self._state.workflow.source_occurrence_id = result.get("source_occurrence_id") or result.get("action_id")
        elif payload.get("source_error_event"):
            self.source_error()
        if payload.get("source_stop_scripts") is True:
            self._source_stop_requested = True
            if self._termination is None:
                self._termination = "safe_stop"
        if payload.get("source_unlock_completed") is True:
            # Genuine successful unlatch can precede a failed LED. The provider
            # owns door/latch publication; retain this host status without
            # clearing tip/plate custody or making a failed parent successful.
            self._source_stop_requested = True
            self._pause = self._wake_control = self._release_gate = None
            self._wake_complete = self._review_released = False
            self._state.workflow.gate = self._state.workflow.gate_id = None
            self._state.paused = self._state.awaiting_review = False
            self._state.pause_reason = None
        if type(payload.get("source_allow_to_stop")) is bool:
            self._state.source_model.allow_to_stop = payload["source_allow_to_stop"]
        if payload.get("source_pause_scripts") and self._pause is None:
            self._pause = ("ordinary", self._state.workflow.source_occurrence_id or result.get("action_id"))
        if payload.get("uncertain") or payload.get("status") == "ambiguous":
            self._unknown = True
        if payload.get("ok") is not True:
            self._failed = True
        self._notify()
        self._publish()

    def _reap(self) -> None:
        for owned, result in list(self._owned):
            if not owned.future.done():
                continue
            with self._condition:
                if (owned, result) not in self._owned:
                    continue
                self._owned.remove((owned, result))
                if owned.future in self._source_wrappers:
                    self._source_wrappers.remove(owned.future)
            try:
                value = owned.future.result()
            except Exception as exc:
                value = {"ok": False, "error_type": type(exc).__name__, "error": "source_child_failed"}
                if hasattr(exc, "oem_partial_results"):
                    value["oem_partial_results"] = exc.oem_partial_results
                if isinstance(getattr(exc, "detail", None), Mapping):
                    value["detail"] = dict(exc.detail)
            self._consume(value, result)

    def wait_for_domains(self, domains: tuple[str, ...]) -> None:
        """Trusted lifecycle composite calls before each conflicting leaf.

        Thermal Abort control effects deliberately do not use this join.
        """
        if set(domains) - set(ALL_DOMAINS):
            raise ValueError("Unknown source dependency domain")
        while True:
            self._reap()
            self._service_requests()
            if self._interrupted or self._recording_failed or self._unknown:
                raise _Diversion()
            if not any(set(child.domains).intersection(domains) for child, _ in self._owned):
                break
            with self._condition:
                self._condition.wait(0.05)
        if self._failed and not (self._servicing or self._settling):
            if self._state.workflow.held_reason == "source_error_hold" and not self._termination:
                self._gate("error_hold", self._state.workflow.source_occurrence_id)
            raise _Diversion()

    def _hook(self, name: str, domains: tuple[str, ...] = (), *, once: bool = False) -> dict[str, Any] | None:
        if once and name in self._hooks_done:
            return
        if self._termination and name in {"prepare", "run_job", "script_prologue", "ordinary_pause_prepare", "deferred_pause_enter", "wake"}:
            raise _Diversion()
        self.wait_for_domains(domains)
        if name == "source_error" and self._termination != "source_error":
            raise _Diversion()
        if name == "cleanup" and self._termination not in {"abort", "safe_stop"}:
            raise _Diversion()
        self._entry("lifecycle:" + name)
        if once:
            self._hooks_done.add(name)  # Never retry a partially entered hook.
        result = {"kind": "lifecycle", "action_id": "lifecycle:" + name, "hook": name}
        self._state.action_results.append(result)
        def entered():
            self._entry("lifecycle:" + name)
            value = self._lifecycle_handlers[name](self._state)
            return value.result() if isinstance(value, Future) else value
        value = self._pool.submit(copy_context().run, entered)
        if self._source_entered and not self._source_return_notified:
            self._source_wrappers.append(value)
        if isinstance(value, Future):
            result["pending"] = True
            owned = OwnedOperation(value, domains)
            self._retain(owned, result)
            # Lifecycle source-return itself is awaited; custody remains even
            # if Stop diverts this wait. Do not Future.cancel().
            while any(item is owned for item, _ in self._owned):
                self._reap()
                self._service_requests()
                if self._interrupted or self._recording_failed or self._unknown:
                    raise _Diversion()
                with self._condition:
                    self._condition.wait(0.05)
        else:
            self._consume(value, result)
        if result.get("ok") is not True:
            raise _Diversion()
        return result

    def _inline_hook(self, name: str, *, token: str, host: bool = False) -> dict[str, Any] | None:
        if token in self._hooks_done:
            return None
        if not host:
            self._entry("lifecycle:" + name)
        self._hooks_done.add(token)  # entered, never success proof or retry permission
        result = {"kind": "lifecycle", "action_id": "lifecycle:" + name, "hook": name}
        self._state.action_results.append(result)
        try:
            value = self._lifecycle_handlers[name](self._state)
        except Exception as exc:
            value = {"ok": False, "error": "source_hook_failed", "error_type": type(exc).__name__}
            if hasattr(exc, "oem_partial_results"):
                value["oem_partial_results"] = exc.oem_partial_results
            if isinstance(getattr(exc, "detail", None), Mapping):
                value["detail"] = dict(exc.detail)
        if isinstance(value, Future):
            # Invalid finite callback, but an already-entered child still owns
            # custody. Never cancel it or wait inline behind a saturated pool.
            result.update(pending=True, hook_contract_error="finite_hook_returned_future")
            self._retain(OwnedOperation(value, ()), result)
            if self._source_entered and not self._source_return_notified:
                self._source_wrappers.append(value)
            self._unknown = True
            self._notify()
        else:
            if isinstance(value, Mapping) and value.get("owned_children"):
                value = {**value, "ok": False, "uncertain": True,
                         "hook_contract_error": "finite_hook_returned_children"}
            self._consume(value, result)
        if host and result.get("ok") is not True:
            self._unknown = True
        if result.get("ok") is not True:
            raise _Diversion()
        return result

    def _request_hook(self, name: str, *, token: str) -> dict[str, Any] | None:
        result = self._inline_hook(name, token=token)
        if result is not None and name in {"safe_stop_request", "abort_true_prefix", "abort_false", "source_error_request"}:
            with self._condition:
                # Accepted intent is not the source flag. Shutdown/request
                # effects return first, then native stop consumers may unwind.
                self._source_stop_requested = True
                self._condition.notify_all()
        return result

    def _finalize_script_host(self) -> None:
        if self._source_entered and self._source_return_notified:
            self._inline_hook("script_finally", token="script_finally", host=True)

    def _service_requests(self) -> None:
        if self._servicing or self._interrupted or self._recording_failed or self._unknown:
            return
        self._servicing = True
        try:
            if (self._source_entered and self._failed
                    and (self._termination in {"abort", "safe_stop"}
                         or (self._termination is None and self._state.workflow.held_reason != "source_error_hold"))):
                self._termination = "source_error"
            if self._termination == "abort" and self._oem:
                self._request_hook("abort_true_prefix", token="abort_true_prefix")
            elif self._termination in {"abort_false", "source_error"} and self._oem:
                name = "source_error_request" if self._termination == "source_error" else "abort_false"
                self._request_hook(name, token=name)
            elif self._termination == "safe_stop" and self._oem:
                self._request_hook("safe_stop_request", token="safe_stop_request")
            elif self._pause and self._pause[0] == "deferred" and not self._termination:
                self._request_hook("deferred_pause_request", token="deferred_request:" + self._pause[1])
        finally:
            self._servicing = False

    def _safe_boundary(self) -> bool:
        model = self._state.source_model
        return model.logical_tip_present is False and model.carried_plate_present is False

    def _gate(self, gate: str, gate_id: str) -> None:
        workflow = self._state.workflow
        workflow.gate, workflow.gate_id = gate, gate_id
        self._release_gate = None
        self._review_released = False
        self._wake_complete = False
        if self._pause and gate in {"ordinary_pause", "deferred_pause"}:
            workflow.reached_control_id = self._pause[1]
            self._reached_controls.add(self._pause[1])
        self._publish("waiting")
        while True:
            self._reap()
            self._service_requests()
            if self._interrupted or self._recording_failed or self._unknown:
                raise _Diversion()
            if self._termination:
                # A diversion never acknowledges review or runs restoration.
                if gate == "review" and self._termination in {"abort", "safe_stop"} and not self._safe_boundary():
                    self._unknown = True
                    self._publish("reconciling", held_reason="review_blocks_termination")
                if gate == "error_hold" and self._termination in {"abort", "safe_stop"}:
                    self._termination = "source_error"
                workflow.gate = workflow.gate_id = None
                if gate in {"ordinary_pause", "deferred_pause", "delaypoint"} and self._termination in {"abort", "safe_stop"}:
                    self._publish("executing")
                    return
                raise _Diversion()
            if self._wake_control:
                control_id, self._wake_control = self._wake_control, None
                self._publish("waking")
                self._hook("wake")
                if self._termination or self._interrupted:
                    continue
                self._wake_complete = True
                workflow.reached_control_id = control_id
                self._reached_controls.add(control_id)
                workflow.requested_control = None
                self._publish("waiting")
            if self._release_gate and (gate != "review" or self._review_released):
                workflow.reached_control_id = self._release_gate
                self._reached_controls.add(self._release_gate)
                workflow.requested_control = None
                workflow.gate = workflow.gate_id = None
                self._pause = None
                self._publish("executing")
                return
            with self._condition:
                self._condition.wait(0.05)

    def _boundary(self) -> None:
        self._reap()
        self._service_requests()
        if self._state.workflow.held_reason == "source_error_hold" and not self._termination:
            self._gate("error_hold", self._state.workflow.source_occurrence_id)
        if self._interrupted or self._recording_failed or self._unknown or self._failed:
            raise _Diversion()
        if self._termination in {"source_error", "abort_false"}:
            raise _Diversion()
        if self._pause and self._pause[0] == "deferred":
            self.wait_for_domains(ALL_DOMAINS)
            if self._termination:  # Core 5408 deferred stop-break, not safe path.
                raise _Diversion()
            if self._safe_boundary():
                self._hook("deferred_pause_enter")
                if not self._termination:
                    self._gate("deferred_pause", self._pause[1])
                if self._termination:
                    # Preserve the deferred stop-break when preparation or the
                    # reached gate services Stop; do not enter ordinary exit.
                    raise _Diversion()
        if self._termination in {"abort", "safe_stop"}:
            self.wait_for_domains(ALL_DOMAINS)
            if not self._oem or self._safe_boundary():
                if self._oem:
                    self._hook("safe_stop_exit", MOTION_DOMAINS, once=True)
                raise _Diversion()
            # Only existing source nodes continue while a logical tip/plate is
            # present. Unknown model authority does not invent that permission.
            model = self._state.source_model
            if model.logical_tip_present is None or model.carried_plate_present is None:
                self._unknown = True
                self._publish("reconciling", held_reason="source_model_unknown")
                raise _Diversion()

    def _source_delaypoint(self, action: ProtocolAction, document: ProtocolDocument) -> dict[str, Any]:
        state = self._state
        state.record_event("preparation_complete", action_id=action.action_id)
        if document.metadata.get("delayed_start") is not True:
            return {"ok": True, "source_noop": True}
        workflow = state.workflow
        with self._condition:
            workflow.gate, workflow.gate_id = "delaypoint", action.source_occurrence_id or action.action_id
            self._release_gate = None
        self._publish("waiting")
        # General is retained by this host-only child. Subsequent TC-only
        # source work can still enter; there is no blanket interpreter mutex.
        with self._condition:
            while True:
                if self._interrupted or self._source_stop_requested or self._recording_failed or self._unknown or self._source_cancelled:
                    reason = "interruption" if self._interrupted else "termination"
                    break
                if self._pause and self._pause[0] == "deferred":
                    reason = "deferred_request"
                    break
                if self._release_gate:
                    reason = "normal"
                    workflow.reached_control_id = self._release_gate
                    self._reached_controls.add(self._release_gate)
                    workflow.requested_control = None
                    break
                self._condition.wait()
            workflow.gate = workflow.gate_id = None
        self._publish("executing")
        return {"ok": True, "host_exit": reason}

    def _source_wait(self, action: ProtocolAction, state: ProtocolRuntimeState) -> dict[str, Any]:
        # Core 6641–6674: one-shot timer token; malformed Int32 is a source
        # no-op. A nonpositive Timer.Interval is a source error, not expiry.
        raw = action.params["arguments"][0]
        try:
            seconds = int(raw)
            if not -(2 ** 31) <= seconds < 2 ** 31 or str(raw).strip().lstrip("+-").isdigit() is False:
                raise ValueError()
        except (ValueError, TypeError):
            return {"ok": True, "source_noop": True, "reason": "wait_wrong_format"}
        if seconds <= 0:
            return {"ok": False, "error": "invalid_wait_timer_interval"}
        deadline = monotonic() + seconds
        with self._condition:
            while True:
                if self._interrupted:
                    return {"ok": True, "host_exit": "interruption"}
                if self._source_stop_requested or self._recording_failed or self._unknown or self._source_cancelled:
                    return {"ok": True, "host_exit": "termination"}
                if self._pause and self._pause[0] == "deferred":
                    return {"ok": True, "host_exit": "deferred_request"}
                remaining = deadline - monotonic()
                if remaining <= 0:
                    return {"ok": True, "host_exit": "timer"}
                self._condition.wait(remaining)

    def _dispatch(self, action: ProtocolAction, document: ProtocolDocument) -> None:
        state = self._state
        opcode = action.oem_opcode if action.kind.value == "oem_operation" else None
        domains = SOURCE_DOMAINS[opcode] if opcode else ALL_DOMAINS
        self.wait_for_domains(domains)
        self._boundary()
        identity = action.source_occurrence_id or action.action_id
        self._entry(identity)
        result = {"stage_id": action.stage_id, "action_id": action.action_id,
                  "kind": action.kind.value, "source_occurrence_id": identity}
        state.action_results.append(result)
        if action.kind in {ProtocolActionKind.NOTE, ProtocolActionKind.PAUSE_REVIEW}:
            value = {"ok": True, "host_only": True}
        elif opcode == "step":
            value = {"ok": True, "source_marker": True}
        elif opcode == "delaypoint":
            value = self._pool.submit(copy_context().run, self._source_delaypoint, action, document)
        else:
            handler = self._source_wait if opcode == "wait" else (self._oem_handlers[opcode] if opcode else self._handlers[action.kind])
            # Source Task.Run belongs to this executor, not to synchronous
            # provider wrappers. Preserve canonical thread-forwarded context.
            def entered():
                self._entry(identity)
                result = handler(action, state)
                return result.result() if isinstance(result, Future) else result
            value = self._pool.submit(copy_context().run, entered)
        if isinstance(value, Future):
            result["pending"] = True
            self._retain(OwnedOperation(value, domains), result)
            if self._source_entered and not self._source_return_notified:
                self._source_wrappers.append(value)
        else:
            self._consume(value, result)
        self._reap()
        if result.get("source_error_hold"):
            self._gate("error_hold", identity)
        if self._failed or self._unknown:
            raise _Diversion()
        # Ordinary pause is post-dispatch and skips both catch aliases. A
        # no-tip wait is not an all-child join (TC can still be running).
        if self._pause and self._pause[0] == "ordinary" and opcode not in {"catch", "catchPlate"} and not self._termination:
            if state.source_model.logical_tip_present is None:
                self._unknown = True
                self._publish("reconciling", held_reason="source_model_unknown")
                raise _Diversion()
            if state.source_model.logical_tip_present:
                self.wait_for_domains(ALL_DOMAINS)
                if not self._termination:
                    self._hook("ordinary_pause_prepare")
            if not self._termination:
                self._gate("ordinary_pause", self._pause[1])

    def _execute_workflow(self, document: ProtocolDocument, *, state: ProtocolRuntimeState | None) -> ProtocolRuntimeState:
        report = self.preflight(document)
        if not report["ok"]:
            raise ValueError("Unbound workflow dependencies: " + ", ".join(report["missing"]))
        if self._state is not None or self._active:
            raise ValueError("Executor attempt cannot be replayed")
        if state is not None:
            if state.protocol_id != document.protocol_id or state.dry_run:
                raise ValueError("Workflow state does not match document/executor")
            if state.events or state.action_results or (state.workflow and state.workflow.phase != "queued"):
                raise ValueError("Retained workflow requires reconciliation, not cursor resume")
            if state.workflow and state.workflow.command_id != self.job_id:
                raise ValueError("Workflow canonical identity mismatch")
        if state is None:
            state = ProtocolRuntimeState.from_document(document, dry_run=False, job_id=self.job_id)
            if document.metadata.get("source_model") is not None:
                state.source_model = ProtocolSourceModel.from_payload(document.to_payload()["metadata"]["source_model"])
        state.workflow = state.workflow or ProtocolWorkflowState(command_id=self.job_id)
        self._state, self._active = state, True
        self._pool = ThreadPoolExecutor(max_workers=len(ALL_DOMAINS) + 1, thread_name_prefix="protocol-source")
        self._oem = bool(report["required_lifecycle"])
        self._ensure_stage_states(document, state)
        try:
            self._publish("preparing" if document.metadata.get("oem_prepare") else "starting")
            if self._oem:
                if document.metadata.get("oem_prepare"):
                    self._hook("prepare", once=True)
                self._publish("starting")
                run_job_result = self._hook("run_job", once=True)
                if run_job_result and run_job_result.get("source_pause_scripts"):
                    # Core 5102–5109 returns before executeScript when the
                    # queried tip prefix asks for manual intervention.
                    self._failed = True
                    state.workflow.held_reason = "source_error_hold"
                    self._gate("error_hold", "lifecycle:run_job")
                    raise _Diversion()
                if self._source_script_begin is not None:
                    if self._termination or self.source_stopped():
                        raise _Diversion()
                    self._source_lifetime_call(self._source_script_begin)
                    self._source_workflow = state.workflow
                    self._source_entered = True
                self._hook("script_prologue", once=True)
            self._publish("executing")
            for stage in document.stages:
                state.current_stage_id = stage.stage_id
                stage_state = state.stage_states[stage.stage_id]
                stage_state.status = StageExecutionStatus.RUNNING
                for action in stage.actions:
                    stage_state.current_action_id = action.action_id
                    state.workflow.source_occurrence_id = action.source_occurrence_id or action.action_id
                    self._boundary()
                    self._publish()
                    self._dispatch(action, document)
                    if action.review_required or action.kind == ProtocolActionKind.PAUSE_REVIEW:
                        self.wait_for_domains(ALL_DOMAINS)
                        self._gate("review", state.workflow.source_occurrence_id)
                if stage.review_required:
                    self.wait_for_domains(ALL_DOMAINS)
                    self._gate("review", stage.stage_id)
            self._boundary()
            if self._termination:
                raise _Diversion()
            self.wait_for_domains(MOTION_DOMAINS)
            self._publish("epilogue")
            if self._oem:
                self._hook("epilogue_sweep", MOTION_DOMAINS, once=True)
                # H7: don't hold sweep behind unrelated TC, but retain that
                # predecessor before the conflicting lid setter and Park.
                self._hook("epilogue_lid", ("TC",), once=True)
                self._hook("epilogue_park", MOTION_DOMAINS, once=True)
        except _Diversion:
            pass
        except Exception as exc:
            self._unknown = True
            state.record_event("workflow_owner_error", detail={"error_type": type(exc).__name__})
        try:
            self._finish_source()
        except _Diversion:
            pass
        finally:
            try:
                self._return_source()
            except _Diversion:
                pass
            finally:
                try:
                    self._finalize_script_host()
                except _Diversion:
                    pass
        # Final aggregation is deliberately AFTER the chosen lifecycle return.
        # Stop never cancels an entered Future or substitutes its own ACK.
        while self._owned:
            self._reap()
            if self._termination or self._failed:
                try:
                    self._finish_source()
                except _Diversion:
                    pass
            with self._condition:
                if self._owned:
                    self._condition.wait(0.05)
        self._pool.shutdown(wait=True)
        self._active = False
        if self._recording_failed or self._unknown:
            self.outcome = "ambiguous"
        elif self._interrupted:
            self.outcome = "interrupted"
        elif self._failed:
            self.outcome = "failed"
        elif self._termination:
            self.outcome = "interrupted"
        else:
            self.outcome = "completed"
        state.completed = self.outcome == "completed"
        if self._termination in {"abort", "safe_stop"} and self.outcome == "interrupted" and not self._interrupted and not self._failed:
            for control_id, request in self._controls.items():
                if request["action"] in {"abort", "safe_stop"}:
                    self._reached_controls.add(control_id)
                    state.workflow.reached_control_id = control_id
        for stage_state in state.stage_states.values():
            rows = [r for r in state.action_results if r.get("stage_id") == stage_state.stage_id and r.get("ok") is True]
            stage_state.completed_actions = [r["action_id"] for r in rows]
            stage_state.current_action_id = None
            stage_state.status = StageExecutionStatus.COMPLETED if state.completed else StageExecutionStatus.FAILED
        state.workflow.gate = state.workflow.gate_id = None
        state.workflow.source_occurrence_id = None
        state.current_stage_id = None
        state.record_event("workflow_settled", detail={"outcome": self.outcome})
        self._publish("reconciling" if self.outcome == "ambiguous" else "terminal")
        if self._recording_failed:
            state.completed = False
            self.outcome = "ambiguous"
        return state

    def _source_lifetime_call(self, callback, *, returned: bool = False) -> None:
        # Host facts only; native-entry and persistence guards must not suppress
        # this notification after an interruption.
        try:
            result = self._explicit_result(callback(self._state))
            if result.get("ok") is not True or (returned and result.get("source_script_returned") is not True):
                raise ValueError("Source lifetime notification refused")
        except Exception as exc:
            self._unknown = True
            assert self._state is not None
            self._state.record_event("source_lifetime_failed", detail={
                "boundary": "returned" if returned else "begin", "error_type": type(exc).__name__,
            })
            raise _Diversion() from exc

    def _return_source(self) -> None:
        if not self._source_entered or self._source_return_notified:
            return
        # Source wrappers include flattened Future returns, but not native
        # children handed back by them or started via start_child.
        while any(not future.done() for future in self._source_wrappers):
            self._reap()
            try:
                self._service_requests()
            except _Diversion:
                pass  # still drain every entered wrapper after request failure
            with self._condition:
                self._condition.wait(0.05)
        self._reap()
        self._source_return_notified = True  # failed notification is not retried
        self._source_lifetime_call(self._source_script_returned, returned=True)
        self._source_returned = True

    def _hook_succeeded(self, name: str) -> bool:
        return any(row.get("hook") == name and row.get("ok") is True
                   and not row.get("pending") and not row.get("hook_contract_error")
                   for row in self._state.action_results)

    def _finish_source(self) -> None:
        if not self._oem or not self._source_entered:
            return
        if self._failed and self._termination not in {"source_error", "abort_false"}:
            self._termination = "source_error"
        if not (self._interrupted or self._unknown or self._recording_failed):
            self._service_requests()
            if self._termination in {"abort", "safe_stop"} and not self._source_return_notified:
                deferred_break = self._pause and self._pause[0] == "deferred"
                if not self._safe_boundary() and not deferred_break:
                    self._unknown = True
                    self._publish("reconciling", held_reason="termination_boundary_unreached")
                elif not deferred_break:
                    self._hook("safe_stop_exit", MOTION_DOMAINS, once=True)
        try:
            self._return_source()
        finally:
            self._finalize_script_host()
        if self._interrupted or self._unknown or self._recording_failed or not self._source_returned:
            return
        # Requests may have arrived while a flattened wrapper was returning.
        if self._failed and self._termination not in {"source_error", "abort_false"}:
            self._termination = "source_error"
        self._service_requests()
        self._settling = True
        try:
            if self._termination == "source_error":
                if not self._hook_succeeded("source_error_request"):
                    raise _Diversion()
                self._publish("cleanup")
                self._hook("source_error", ALL_DOMAINS, once=True)
            elif self._termination in {"abort", "safe_stop"}:
                if self._termination == "abort":
                    if not self._hook_succeeded("abort_true_prefix"):
                        raise _Diversion()
                    self._hook("abort_true_finish", once=True)
                    if not self._hook_succeeded("abort_true_finish"):
                        raise _Diversion()
                elif not self._hook_succeeded("safe_stop_request"):
                    raise _Diversion()
                if "safe_stop_exit" in self._hooks_done and not self._hook_succeeded("safe_stop_exit"):
                    raise _Diversion()
                self._publish("cleanup")
                self._hook("cleanup", MOTION_DOMAINS, once=True)
            # False Abort has no application cleanup tail.
        finally:
            self._settling = False

    def _ensure_stage_states(self, document: ProtocolDocument, state: ProtocolRuntimeState) -> None:
        for stage in document.stages:
            state.stage_states.setdefault(
                stage.stage_id,
                ProtocolStageState(
                    stage_id=stage.stage_id,
                    title=stage.title,
                    review_required=bool(stage.review_required),
                ),
            )

    def _stage_is_complete(self, stage, stage_state: ProtocolStageState) -> bool:
        return (
            stage_state.status is StageExecutionStatus.COMPLETED
            and len(stage_state.completed_actions) >= len(stage.actions)
        )

    def _stage_review_was_acknowledged(self, stage, stage_state: ProtocolStageState) -> bool:
        return (
            bool(stage.review_required)
            and len(stage_state.completed_actions) >= len(stage.actions)
            and stage_state.pause_marker_action_id is None
            and stage_state.status is StageExecutionStatus.PAUSED
        )

    def _execute_action(self, action: ProtocolAction, state: ProtocolRuntimeState) -> dict[str, Any]:
        if self.dry_run:
            return {
                "ok": True,
                "dry_run": True,
                "params": action.to_payload()["params"],
            }

        handler = self._handlers.get(action.kind)
        if handler is None:
            return {
                "ok": False,
                "dry_run": False,
                "error": f"No handler registered for action kind '{action.kind.value}'",
            }

        result = handler(action, state)
        payload = self._explicit_result(result)
        payload.setdefault("dry_run", False)
        return payload

    def _pause_for_review(
        self,
        state: ProtocolRuntimeState,
        *,
        stage_id: str,
        action_id: str | None,
        message: str | None,
    ) -> ProtocolRuntimeState:
        stage_state = state.stage_states[stage_id]
        stage_state.status = StageExecutionStatus.PAUSED
        stage_state.current_action_id = None
        stage_state.pause_marker_action_id = action_id
        state.current_stage_id = stage_id
        state.paused = True
        state.awaiting_review = True
        state.completed = False
        state.pause_reason = message or f"Review required for stage '{stage_id}'"
        state.record_event(
            "paused_for_review",
            stage_id=stage_id,
            action_id=action_id,
            detail={"reason": state.pause_reason},
        )
        return state
