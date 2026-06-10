from fastapi.testclient import TestClient

from src.bioxp import oem_runtime_api
from src.bioxp.api import app
from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers
from src.bioxp.oem_runtime_types import OEMRuntimeCommand


PREPARE_TO_RUN_JOB_READINESS_ROUTE = "/oem/runtime/readiness/prepare-to-run-job/dry-run"

EXPECTED_OEM_READINESS_STEPS = [
    "home_gripper",
    "inspect_purification_location",
    "inspect_output_plate",
    "inspect_trough",
    "inspect_strip_handle",
    "inspect_strip_wells",
    "inspect_tip_trays",
    "park_gantry",
]

EXPECTED_SOURCE_ANCHORS = [
    "BioXPMainWindow.cs:1588-1808",
    "BioXPMainWindow.cs:2056",
    "ControlLib.cs:4349-4385",
    "ControlLib.cs:4430-4465",
    "ControlLib.cs:3981-4015",
    "ControlLib.cs:4139-4190",
    "ControlLib.cs:4234-4346",
    "ControlLib.cs:4468-4739",
]


def _assert_prepare_to_run_job_readiness_payload(payload: dict) -> None:
    assert payload["ok"] is True
    assert payload["schema_version"] == "bioxp.oem_runtime.prepare_to_run_job_readiness.v1"
    assert payload["command"] == "PrepareToRunJob"
    assert payload["mode"] == "dry_run"
    assert payload["state"] == "prepare_to_run_job_readiness_dry_run_complete"
    assert payload["motion_commanded"] is False
    assert payload["hardware_touched"] is False
    assert payload["ready"] is False
    assert payload["truth_level"] == "source_anchored_plan_only_no_motion"
    assert payload["source_parity"] == "source_anchored_prepare_to_run_job_deck_inspection"
    assert payload["source_anchors"] == EXPECTED_SOURCE_ANCHORS

    step_names = [step["name"] for step in payload["readiness_steps"]]
    assert step_names == EXPECTED_OEM_READINESS_STEPS
    assert all(step["motion_allowed"] is False for step in payload["readiness_steps"])
    assert all(step["camera_capture_allowed"] is False for step in payload["readiness_steps"])
    assert all(step.get("source_anchor") for step in payload["readiness_steps"])

    notes = "\n".join(payload["notes"])
    assert "no live hardware motion" in notes
    assert "dry-run does not prove physical deck readiness" in notes


def test_prepare_to_run_job_command_handler_returns_source_anchored_no_motion_readiness_plan() -> None:
    handlers = OEMRuntimeCommandHandlers().handlers()
    result = handlers["PrepareToRunJob"](
        OEMRuntimeCommand(
            name="PrepareToRunJob",
            mode="dry_run",
            params={
                "no_motion": True,
                "deck_inspection": True,
                "script_requirements": {
                    "output_plate_required": True,
                    "trough_required": True,
                    "tip_trays_required": True,
                },
            },
        )
    )

    _assert_prepare_to_run_job_readiness_payload(result)


def test_prepare_to_run_job_named_dry_run_api_exposes_readiness_without_queueing_motion(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)

    response = client.post(
        PREPARE_TO_RUN_JOB_READINESS_ROUTE,
        json={
            "mode": "dry_run",
            "source": "pytest-red-test",
            "params": {
                "no_motion": True,
                "deck_inspection": True,
                "script_requirements": {
                    "output_plate_required": True,
                    "trough_required": True,
                    "tip_trays_required": True,
                },
            },
        },
    )

    assert response.status_code == 200
    _assert_prepare_to_run_job_readiness_payload(response.json())


def test_prepare_to_run_job_queued_runtime_command_executes_same_no_motion_readiness_plan(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    runtime = oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    client = TestClient(app)

    enqueue = client.post(
        "/oem/runtime/commands/PrepareToRunJob",
        json={"mode": "dry_run", "params": {"no_motion": True, "deck_inspection": True}},
    )
    assert enqueue.status_code == 200

    run = runtime["worker"].run_next_for_tests()

    assert run["ok"] is True
    assert run["ran"] is True
    _assert_prepare_to_run_job_readiness_payload(run["result"])
