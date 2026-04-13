from src.bioxp.protocols.compiler import compile_native_protocol
from src.bioxp.protocols.executor import ProtocolExecutor
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
