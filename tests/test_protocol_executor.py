from src.bioxp.protocols.compiler import compile_native_protocol
from src.bioxp.protocols.executor import ProtocolExecutor
from src.bioxp.protocols.models import ProtocolActionKind
from src.bioxp.protocols.runtime_state import StageExecutionStatus


def test_protocol_executor_pauses_at_review_marker_in_dry_run_mode():
    document = compile_native_protocol(
        {
            "protocol_id": "review-protocol",
            "stages": [
                {
                    "stage_id": "prep",
                    "actions": [
                        {"kind": "move", "target": "reagent_rack"},
                        {"kind": "pause_review", "message": "Verify deck setup before transfer"},
                    ],
                },
                {
                    "stage_id": "transfer",
                    "actions": [
                        {"kind": "pipette_aspirate", "volume_ul": 10.0, "source": "reagent_rack:A1"},
                    ],
                },
            ],
        }
    )

    result = ProtocolExecutor(dry_run=True).execute(document)

    assert result.paused is True
    assert result.awaiting_review is True
    assert result.completed is False
    assert result.pause_reason == "Verify deck setup before transfer"
    assert result.current_stage_id == "prep"
    assert result.stage_states["prep"].status is StageExecutionStatus.PAUSED
    assert result.stage_states["transfer"].status is StageExecutionStatus.PENDING
    assert len(result.action_results) == 2
    assert result.action_results[0]["dry_run"] is True
    assert result.events[-1].event == "paused_for_review"


def test_protocol_executor_completes_tiny_native_protocol_in_dry_run_mode():
    document = compile_native_protocol(
        {
            "protocol_id": "tiny-native",
            "stages": [
                {
                    "stage_id": "prep",
                    "actions": [
                        {"kind": "move", "target": "reagent_rack"},
                    ],
                },
                {
                    "stage_id": "inspect",
                    "review_required": False,
                    "actions": [
                        {"kind": "inspect", "location_id": "reagent_rack", "requested_checks": ["focus"]},
                    ],
                },
            ],
        }
    )

    result = ProtocolExecutor(dry_run=True).execute(document)

    assert result.completed is True
    assert result.paused is False
    assert result.awaiting_review is False
    assert result.current_stage_id is None
    assert all(state.status is StageExecutionStatus.COMPLETED for state in result.stage_states.values())
    assert [entry["kind"] for entry in result.action_results] == ["move", "inspect"]
    assert result.events[-1].event == "protocol_completed"


def test_protocol_runtime_state_retains_generated_job_identity():
    document = compile_native_protocol(
        {
            "protocol_id": "job-bound",
            "stages": [{"stage_id": "inspect", "actions": [{"kind": "inspect"}]}],
        }
    )

    result = ProtocolExecutor(dry_run=True, job_id="protocol-job-123").execute(document)

    assert result.job_id == "protocol-job-123"
    assert result.to_payload()["job_id"] == "protocol-job-123"


def test_live_handler_false_result_fails_stage_without_completing_later_actions():
    document = compile_native_protocol(
        {
            "protocol_id": "live-failure",
            "stages": [
                {
                    "stage_id": "run",
                    "actions": [
                        {"action_id": "move-1", "kind": "move", "target": "a"},
                        {"action_id": "move-2", "kind": "move", "target": "b"},
                    ],
                }
            ],
        }
    )
    calls = []
    result = ProtocolExecutor(
        dry_run=False,
        handlers={ProtocolActionKind.MOVE: lambda action, state: calls.append(action.action_id) or {"ok": False, "error": "denied"}},
    ).execute(document)

    assert result.completed is False
    assert result.stage_states["run"].status is StageExecutionStatus.FAILED
    assert result.stage_states["run"].completed_actions == []
    assert calls == ["move-1"]
    assert result.action_results[-1]["ok"] is False
    assert result.events[-1].event == "action_failed"
