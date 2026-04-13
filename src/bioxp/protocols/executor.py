from __future__ import annotations

from typing import Any, Callable, Mapping

from .models import ProtocolAction, ProtocolActionKind, ProtocolDocument, normalize_action_kind
from .runtime_state import ProtocolRuntimeState, ProtocolStageState, StageExecutionStatus
from .validators import validate_protocol_document

ActionHandler = Callable[[ProtocolAction, ProtocolRuntimeState], Mapping[str, Any] | None]


class ProtocolExecutor:
    def __init__(
        self,
        *,
        dry_run: bool = True,
        handlers: Mapping[ProtocolActionKind | str, ActionHandler] | None = None,
    ) -> None:
        self.dry_run = bool(dry_run)
        self._handlers = {
            normalize_action_kind(kind): handler
            for kind, handler in (handlers or {}).items()
        }

    def execute(
        self,
        document: ProtocolDocument,
        *,
        state: ProtocolRuntimeState | None = None,
    ) -> ProtocolRuntimeState:
        validate_protocol_document(document)
        runtime_state = state or ProtocolRuntimeState.from_document(document, dry_run=self.dry_run)
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
                "params": dict(action.params),
            }

        handler = self._handlers.get(action.kind)
        if handler is None:
            return {
                "ok": False,
                "dry_run": False,
                "error": f"No handler registered for action kind '{action.kind.value}'",
            }

        result = handler(action, state)
        payload = dict(result or {})
        payload.setdefault("ok", True)
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
