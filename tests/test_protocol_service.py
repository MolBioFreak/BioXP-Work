from pathlib import Path

from src.bioxp.protocols.compiler import compile_native_protocol
from src.bioxp.protocols.executor import ProtocolExecutor
from src.bioxp.protocols.runtime_state import StageExecutionStatus
from src.bioxp.services import protocol_service
from src.bioxp.services.protocol_service import ProtocolOperatorBundleStore, compile_protocol_source, create_protocol_job, get_protocol_jobs_root, review_protocol_job


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "testdata" / "oem_xml"
DEMO_XML = FIXTURE_ROOT / "demo.xml"


def test_protocol_executor_can_resume_after_action_level_review_gate() -> None:
    document = compile_native_protocol(
        {
            "protocol_id": "resume-action-review",
            "stages": [
                {
                    "stage_id": "stage-a",
                    "actions": [
                        {"action_id": "move-1", "kind": "move", "axis": "x", "steps": 10},
                        {
                            "action_id": "pause-1",
                            "kind": "pause_review",
                            "review_required": True,
                            "pause_message": "check deck",
                        },
                        {"action_id": "note-1", "kind": "note", "message": "continue"},
                    ],
                }
            ],
        }
    )

    executor = ProtocolExecutor(dry_run=True)
    paused = executor.execute(document)
    assert paused.awaiting_review is True
    assert paused.stage_states["stage-a"].status is StageExecutionStatus.PAUSED

    stage_state = paused.stage_states["stage-a"]
    stage_state.pause_marker_action_id = None
    paused.awaiting_review = False
    paused.paused = False
    paused.pause_reason = None

    resumed = executor.execute(document, state=paused)
    assert resumed.completed is True
    assert resumed.stage_states["stage-a"].status is StageExecutionStatus.COMPLETED
    assert [entry["action_id"] for entry in resumed.action_results] == ["move-1", "pause-1", "note-1"]


def test_compile_protocol_source_supports_oem_xml() -> None:
    compiled = compile_protocol_source({"source_type": "oem_xml", "xml_path": str(DEMO_XML)})
    assert compiled.source_type == "oem_xml"
    assert compiled.document.protocol_id == "oem-system-demo-v4"
    assert compiled.coverage["supported_command_count"] == 68


def test_get_protocol_jobs_root_falls_back_when_default_root_is_unwritable(monkeypatch, tmp_path) -> None:
    fallback_root = tmp_path / "fallback_protocol_jobs"
    monkeypatch.delenv("BIOXP_PROTOCOL_JOBS_ROOT", raising=False)
    monkeypatch.setattr(protocol_service, "DEFAULT_PROTOCOL_JOBS_ROOT", Path("/proc/forbidden/protocol_jobs"))
    monkeypatch.setattr(protocol_service, "FALLBACK_PROTOCOL_JOBS_ROOT", fallback_root)

    resolved = get_protocol_jobs_root()

    assert resolved == fallback_root
    assert fallback_root.exists()


def test_protocol_operator_bundle_persists_and_resumes_review_job(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    bundle = create_protocol_job(
        {
            "source_type": "native",
            "document": {
                "protocol_id": "persist-review-job",
                "stages": [
                    {
                        "stage_id": "stage-a",
                        "actions": [
                            {"action_id": "note-1", "kind": "note", "message": "before"},
                            {
                                "action_id": "pause-1",
                                "kind": "pause_review",
                                "review_required": True,
                                "pause_message": "operator must inspect",
                            },
                            {"action_id": "note-2", "kind": "note", "message": "after"},
                        ],
                    }
                ],
            },
        },
        dry_run=True,
        store=store,
    )

    persisted = store.load(bundle["job_id"])
    assert persisted["status"] == "awaiting_review"
    assert persisted["operator"]["pending_review"]["action_id"] == "pause-1"
    assert Path(persisted["artifacts"]["bundle_path"]).exists()

    resumed = review_protocol_job(bundle["job_id"], reviewer="test", note="looks good", store=store)
    assert resumed["status"] == "completed"
    assert resumed["operator"]["manual_review_required"] is False
    assert resumed["operator"]["pending_review"] is None
    assert resumed["operator"]["reviews"][0]["reviewer"] == "test"
    assert resumed["execution"]["runtime_state"]["completed"] is True
