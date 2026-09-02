from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import threading

import pytest

from src.bioxp.protocols.compiler import compile_native_protocol
from src.bioxp.protocols.executor import ProtocolExecutor
from src.bioxp.protocols.runtime_state import StageExecutionStatus
from src.bioxp.services import protocol_service
from src.bioxp.services.protocol_service import (
    ProtocolLiveContractError,
    ProtocolOperatorBundleStore,
    compile_protocol_source,
    create_protocol_job,
    get_protocol_jobs_root,
    review_protocol_job,
)


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "testdata" / "oem_xml"
DEMO_XML = FIXTURE_ROOT / "demo.xml"


def test_operator_bundle_save_uses_atomic_replace(monkeypatch, tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    replacements: list[Path] = []
    real_replace = protocol_service.os.replace

    def track_replace(source, destination):
        replacements.append(Path(destination))
        return real_replace(source, destination)

    monkeypatch.setattr(protocol_service.os, "replace", track_replace)
    saved = store.save({
        "schema_version": "test",
        "job_id": "atomic-bundle-save",
        "status": "queued",
        "execution": {"dry_run": True},
    })

    assert store._bundle_path("atomic-bundle-save") in replacements
    assert store.load("atomic-bundle-save") == saved
    assert list((tmp_path / "atomic-bundle-save").glob(".*.tmp")) == []


def test_protocol_review_contract_error_is_a_controlled_conflict(monkeypatch) -> None:
    from fastapi.testclient import TestClient
    from src.bioxp import api

    def reject_review(*_args, **_kwargs):
        raise ProtocolLiveContractError(
            "Persisted review contract is invalid.",
            details={"review_contract_invalid": True},
        )

    monkeypatch.setattr(api, "review_protocol_job", reject_review)
    response = TestClient(api.app, raise_server_exceptions=False).post(
        "/protocol/jobs/protocol-live-invalid/review",
        json={"reviewer": "operator", "note": "retry"},
    )

    assert response.status_code == 409
    assert response.json()["detail"]["review_contract_invalid"] is True


def test_protocol_review_rejects_valid_json_non_mapping_bundle_without_http_500(
    monkeypatch, tmp_path,
) -> None:
    from fastapi.testclient import TestClient
    from src.bioxp import api

    job_id = "protocol-valid-json-invalid-shape"
    job_dir = tmp_path / job_id
    job_dir.mkdir()
    (job_dir / "bundle.json").write_text("[]\n", encoding="utf-8")
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path))

    response = TestClient(api.app, raise_server_exceptions=False).post(
        f"/protocol/jobs/{job_id}/review",
        json={"reviewer": "operator", "note": "retry"},
    )

    assert response.status_code == 409
    assert response.json()["detail"]["error"] == "live_protocol_contract_failed"
    assert response.json()["detail"]["review_contract_invalid"] is True


@pytest.mark.parametrize(
    "current_stage_id", [["bad"], {"bad": "stage"}], ids=["array", "object"]
)
def test_protocol_review_rejects_non_string_current_stage_id_without_http_500(
    monkeypatch, tmp_path, current_stage_id,
) -> None:
    from fastapi.testclient import TestClient
    from src.bioxp import api

    store = ProtocolOperatorBundleStore(tmp_path)
    bundle = create_protocol_job(
        {
            "source_type": "native",
            "document": {
                "protocol_id": "malformed-stage-id",
                "stages": [{
                    "stage_id": "stage-a",
                    "actions": [{
                        "action_id": "pause-1", "kind": "pause_review",
                        "review_required": True, "pause_message": "inspect",
                    }],
                }],
            },
        },
        dry_run=True,
        store=store,
    )
    bundle["execution"]["runtime_state"]["current_stage_id"] = current_stage_id
    store.save(bundle)
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path))

    response = TestClient(api.app, raise_server_exceptions=False).post(
        f"/protocol/jobs/{bundle['job_id']}/review",
        json={"reviewer": "operator", "note": "retry"},
    )

    assert response.status_code == 409
    assert response.json()["detail"]["error"] == "live_protocol_contract_failed"
    assert response.json()["detail"]["review_contract_invalid"] is True


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


def _live_protocol_payload(**live_overrides):
    live_execution = {
        "operator_id": "integration-operator",
        "live_execution_ack": True,
        "physical_console_verified": True,
        "deck_manifest": {
            "source": "operator-preflight",
            "locations": {
                "reagent_rack:A1": {"contents": "water", "volume_ul": 100.0},
                "waste:A1": {"contents": "empty"},
            },
        },
        "preflight": {
            "reference_snapshot": {
                "rows": {
                    "x": {"state": "referenced"},
                    "y": {"state": "referenced"},
                    "z": {"state": "referenced"},
                }
            },
            "artifact_refs": ["snapshot://deck-before-live-run"],
        },
    }
    live_execution.update(live_overrides)
    return {
        "source_type": "native",
        "idempotency_key": "operator-live-execution-7-2",
        "document": {
            "protocol_id": "live-gated-protocol",
            "stages": [
                {
                    "stage_id": "move-stage",
                    "actions": [
                        {"action_id": "move-x", "kind": "move", "axis": "x", "steps": 4},
                    ],
                }
            ],
        },
        "live_execution": live_execution,
    }


def test_live_protocol_execution_requires_ack_manifest_reference_and_artifacts(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    payload = _live_protocol_payload(
        operator_id="",
        live_execution_ack=False,
        physical_console_verified=False,
        deck_manifest={},
        preflight={"reference_snapshot": {"rows": {"x": {"state": "referenced"}}}, "artifact_refs": []},
    )

    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(payload, dry_run=False, store=store)

    details = exc_info.value.to_payload()
    assert details["error"] == "live_protocol_contract_failed"
    assert set(details["missing_contract_fields"]) >= {
        "live_execution_ack",
        "operator_id",
        "physical_console_verified",
        "deck_manifest",
        "preflight.artifact_refs",
    }
    assert details["missing_reference_axes"] == ["y", "z"]

    string_false = _live_protocol_payload(
        live_execution_ack="false", physical_console_verified="false",
    )
    with pytest.raises(ProtocolLiveContractError) as string_exc:
        create_protocol_job(
            string_false, dry_run=False, store=store,
            handlers={"move": lambda *_: {"ok": True}},
        )
    assert set(string_exc.value.to_payload()["missing_contract_fields"]) >= {
        "live_execution_ack", "physical_console_verified",
    }


def test_live_protocol_execution_rejects_hardware_actions_without_registered_handlers(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)

    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(_live_protocol_payload(), dry_run=False, store=store)

    details = exc_info.value.to_payload()
    assert details["error"] == "live_protocol_contract_failed"
    assert details["missing_live_handlers"] == ["move"]
    assert details["hardware_action_kinds"] == ["move"]


def test_live_protocol_execution_rejects_any_importable_action_without_a_handler(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    payload = _live_protocol_payload()
    payload["document"]["stages"][0]["actions"].append(
        {"action_id": "note-after-move", "kind": "note", "message": "source note"}
    )

    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(
            payload,
            dry_run=False,
            store=store,
            handlers={"move": lambda _action, _state: {"ok": True}},
        )

    assert exc_info.value.to_payload()["missing_live_handlers"] == ["note"]


def test_live_review_resume_reuses_the_validated_hardware_handlers(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    payload = _live_protocol_payload()
    actions = payload["document"]["stages"][0]["actions"]
    actions[0]["review_required"] = True
    actions[0]["pause_message"] = "inspect after first move"
    actions.append({"action_id": "move-x-2", "kind": "move", "axis": "x", "steps": 2})
    calls: list[str] = []

    def move_handler(action, _state):
        calls.append(action.action_id)
        return {"ok": True}

    created = create_protocol_job(
        payload,
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )
    assert created["status"] == "awaiting_review"
    assert calls == ["move-x"]

    resumed = review_protocol_job(
        created["job_id"],
        reviewer="operator",
        store=store,
        handlers={"move": move_handler},
    )

    assert resumed["status"] == "completed"
    assert calls == ["move-x", "move-x-2"]

    replayed = create_protocol_job(
        payload,
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )
    assert replayed["job_id"] == resumed["job_id"]
    assert replayed["execution"]["idempotency_binding"] == created["execution"]["idempotency_binding"]
    assert calls == ["move-x", "move-x-2"]

    review_replay = review_protocol_job(
        created["job_id"],
        reviewer="operator",
        store=store,
        handlers={"move": move_handler},
    )
    assert review_replay == resumed
    assert calls == ["move-x", "move-x-2"]


def test_live_protocol_review_rejects_bundle_drift_from_reserved_request(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    payload = _live_protocol_payload()
    actions = payload["document"]["stages"][0]["actions"]
    actions[0]["review_required"] = True
    actions[0]["pause_message"] = "inspect after first move"
    actions.append({"action_id": "move-x-2", "kind": "move", "axis": "x", "steps": 2})
    calls: list[str] = []

    def move_handler(action, _state):
        calls.append(action.action_id)
        return {"ok": True}

    created = create_protocol_job(
        payload,
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )
    bundle = store.load(created["job_id"])
    bundle["protocol"]["document"]["stages"][0]["actions"][1]["steps"] = 999
    store.save(bundle)

    with pytest.raises(ProtocolLiveContractError) as exc:
        review_protocol_job(
            created["job_id"],
            reviewer="operator",
            store=store,
            handlers={"move": move_handler},
        )

    assert exc.value.details["idempotency_recovery_required"] is True
    assert calls == ["move-x"]


@pytest.mark.parametrize(
    "mutation", ["non_boolean_mode", "runtime_identity", "runtime_boolean"]
)
def test_live_protocol_review_rejects_malformed_execution_identity(
    tmp_path, mutation: str,
) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    payload = _live_protocol_payload()
    payload["idempotency_key"] = f"malformed-review-{mutation}"
    actions = payload["document"]["stages"][0]["actions"]
    actions[0]["review_required"] = True
    actions[0]["pause_message"] = "inspect"
    actions.append({"action_id": "move-x-2", "kind": "move", "axis": "x", "steps": 2})
    calls: list[str] = []

    def move_handler(action, _state):
        calls.append(action.action_id)
        return {"ok": True}

    created = create_protocol_job(
        payload, dry_run=False, store=store, handlers={"move": move_handler}
    )
    bundle = store.load(created["job_id"])
    if mutation == "non_boolean_mode":
        bundle["execution"]["dry_run"] = 0
    elif mutation == "runtime_identity":
        bundle["execution"]["runtime_state"]["job_id"] = "different-job"
    else:
        bundle["execution"]["runtime_state"]["awaiting_review"] = 1
    store.save(bundle)

    with pytest.raises(ProtocolLiveContractError):
        review_protocol_job(
            created["job_id"], reviewer="operator", store=store,
            handlers={"move": move_handler},
        )
    assert calls == ["move-x"]


def test_live_protocol_review_rejects_rebound_job_digest(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    payload = _live_protocol_payload()
    payload["idempotency_key"] = "review-rebound-job-digest"
    actions = payload["document"]["stages"][0]["actions"]
    actions[0]["review_required"] = True
    actions[0]["pause_message"] = "inspect"
    actions.append({"action_id": "move-x-2", "kind": "move", "axis": "x", "steps": 2})
    calls: list[str] = []

    def move_handler(action, _state):
        calls.append(action.action_id)
        return {"ok": True}

    created = create_protocol_job(
        payload, dry_run=False, store=store, handlers={"move": move_handler}
    )
    forged_digest = "a" * 64
    bundle = store.load(created["job_id"])
    bundle["execution"]["idempotency_binding"]["idempotency_key_digest"] = forged_digest
    store.save(bundle)
    reservation = store.load_live_reservation(created["job_id"])
    assert reservation is not None
    reservation["idempotency_key_digest"] = forged_digest
    store.save_live_reservation(reservation)

    with pytest.raises(ProtocolLiveContractError):
        review_protocol_job(
            created["job_id"], reviewer="operator", store=store,
            handlers={"move": move_handler},
        )

    assert calls == ["move-x"]


def test_live_review_claim_prevents_replay_after_handler_failure(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    payload = _live_protocol_payload()
    actions = payload["document"]["stages"][0]["actions"]
    actions[0]["review_required"] = True
    actions.append({"action_id": "move-x-2", "kind": "move", "axis": "x", "steps": 2})
    calls: list[str] = []

    def move_handler(action, _state):
        calls.append(action.action_id)
        if action.action_id == "move-x-2":
            raise RuntimeError("provider failure after review claim")
        return {"ok": True}

    created = create_protocol_job(
        payload, dry_run=False, store=store, handlers={"move": move_handler},
    )
    with pytest.raises(RuntimeError, match="provider failure after review claim"):
        review_protocol_job(
            created["job_id"], store=store, handlers={"move": move_handler},
        )
    assert store.load(created["job_id"])["status"] == "review_failed_ambiguous"
    with pytest.raises(ValueError, match="not awaiting review"):
        review_protocol_job(
            created["job_id"], store=store, handlers={"move": move_handler},
        )
    assert calls == ["move-x", "move-x-2"]


def test_live_protocol_execution_persists_contract_preflight_artifact_and_uses_handlers(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    calls = []

    def move_handler(action, state):
        calls.append({"action_id": action.action_id, "protocol_id": state.protocol_id, "params": dict(action.params)})
        return {"ok": True, "handler": "test-move", "position_delta": action.params["steps"]}

    bundle = create_protocol_job(
        _live_protocol_payload(),
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )

    assert calls == [{"action_id": "move-x", "protocol_id": "live-gated-protocol", "params": {"axis": "x", "steps": 4}}]
    assert bundle["status"] == "completed"
    assert bundle["execution"]["dry_run"] is False
    contract = bundle["execution"]["live_contract"]
    assert contract["schema_version"] == "bioxp.protocol_live_execution_contract.v1"
    assert contract["operator_id"] == "integration-operator"
    assert contract["preflight"]["reference_axes_verified"] == ["x", "y", "z"]
    assert contract["artifacts"]["required"] is True
    assert contract["artifacts"]["refs"] == ["snapshot://deck-before-live-run"]
    preflight_path = Path(bundle["artifacts"]["preflight_path"])
    assert preflight_path.exists()
    assert preflight_path.read_text(encoding="utf-8").count("snapshot://deck-before-live-run") == 1


def test_live_protocol_retry_replays_one_completed_job_with_stable_runtime_identity(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    calls = []

    def move_handler(action, state):
        subcommand_key = f"protocol:{state.job_id}:{action.action_id}"
        calls.append({"job_id": state.job_id, "subcommand_key": subcommand_key})
        return {"ok": True, "subcommand_key": subcommand_key}

    first = create_protocol_job(
        _live_protocol_payload(),
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )
    second = create_protocol_job(
        _live_protocol_payload(),
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )

    assert first == second
    assert first["job_id"] == first["execution"]["runtime_state"]["job_id"]
    assert calls == [
        {
            "job_id": first["job_id"],
            "subcommand_key": f"protocol:{first['job_id']}:move-x",
        }
    ]


def test_live_protocol_terminal_failure_persists_and_replays_without_second_handler(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    calls = []
    failure_evidence = {
        "ok": False,
        "failure_code": "controller-rejected",
        "receipt": {"sequence": 17, "detail": ["stable", "evidence"]},
    }

    def move_handler(action, state):
        calls.append((state.job_id, action.action_id))
        return failure_evidence

    first = create_protocol_job(
        _live_protocol_payload(),
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )
    persisted = store.load(first["job_id"])
    second = create_protocol_job(
        _live_protocol_payload(),
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )

    assert first == persisted == second
    assert first["status"] == "failed"
    runtime_state = first["execution"]["runtime_state"]
    assert runtime_state["completed"] is False
    assert runtime_state["stage_states"]["move-stage"]["status"] == "failed"
    assert runtime_state["action_results"] == [
        {
            "stage_id": "move-stage",
            "action_id": "move-x",
            "kind": "move",
            **failure_evidence,
            "dry_run": False,
        }
    ]
    assert calls == [(first["job_id"], "move-x")]

    conflicting = _live_protocol_payload()
    conflicting["document"]["stages"][0]["actions"][0]["steps"] = 99
    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(
            conflicting,
            dry_run=False,
            store=store,
            handlers={"move": move_handler},
        )
    assert exc_info.value.to_payload()["idempotency_conflict"] is True
    assert calls == [(first["job_id"], "move-x")]


def test_live_protocol_concurrent_same_key_creators_share_one_completed_job(tmp_path) -> None:
    attempted_lock = threading.Event()
    handler_entered = threading.Event()
    start = threading.Barrier(2)
    calls = []
    attempts = 0
    attempts_guard = threading.Lock()

    class OverlapStore(ProtocolOperatorBundleStore):
        def live_creation_lock(self, job_id):
            nonlocal attempts
            with attempts_guard:
                attempts += 1
                if attempts == 2:
                    attempted_lock.set()
            return super().live_creation_lock(job_id)

    store = OverlapStore(tmp_path)

    def move_handler(action, state):
        calls.append((state.job_id, action.action_id))
        handler_entered.set()
        assert attempted_lock.wait(timeout=5), "second creator did not overlap the first handler"
        return {"ok": True}

    def create():
        start.wait(timeout=5)
        return create_protocol_job(
            _live_protocol_payload(),
            dry_run=False,
            store=store,
            handlers={"move": move_handler},
        )

    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [executor.submit(create) for _ in range(2)]
        assert handler_entered.wait(timeout=5), "neither creator reached the handler"
        first, second = (future.result(timeout=10) for future in futures)

    assert len(calls) == 1
    assert first == second
    assert first["status"] == second["status"] == "completed"
    assert first["job_id"] == second["job_id"]
    assert first["job_id"] == first["execution"]["runtime_state"]["job_id"]


def test_live_protocol_conflicting_same_key_rejects_before_second_handler(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    calls = []

    def move_handler(action, state):
        calls.append((state.job_id, action.params["steps"]))
        return {"ok": True}

    create_protocol_job(
        _live_protocol_payload(),
        dry_run=False,
        store=store,
        handlers={"move": move_handler},
    )
    conflicting = _live_protocol_payload()
    conflicting["document"]["stages"][0]["actions"][0]["steps"] = 5

    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(
            conflicting,
            dry_run=False,
            store=store,
            handlers={"move": move_handler},
        )

    assert exc_info.value.to_payload()["idempotency_conflict"] is True
    assert len(calls) == 1


def test_live_protocol_missing_idempotency_key_rejects_before_handler(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    calls = []
    payload = _live_protocol_payload()
    payload.pop("idempotency_key")

    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(
            payload,
            dry_run=False,
            store=store,
            handlers={"move": lambda *_args: calls.append("called")},
        )

    assert exc_info.value.to_payload()["missing_contract_fields"] == ["idempotency_key"]
    assert calls == []


def test_live_protocol_orphan_reservation_fails_closed_without_reinvoking_handler(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    retry_calls = []

    def interrupted_handler(*_args):
        raise RuntimeError("simulated interruption after reservation")

    with pytest.raises(RuntimeError, match="simulated interruption"):
        create_protocol_job(
            _live_protocol_payload(),
            dry_run=False,
            store=store,
            handlers={"move": interrupted_handler},
        )

    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(
            _live_protocol_payload(),
            dry_run=False,
            store=store,
            handlers={"move": lambda *_args: retry_calls.append("called")},
        )

    assert exc_info.value.to_payload()["idempotency_recovery_required"] is True
    assert retry_calls == []


def test_live_protocol_root_directory_durability_precedes_handler_and_fails_closed(
    tmp_path, monkeypatch
) -> None:
    failed_store = ProtocolOperatorBundleStore(tmp_path / "failed")
    failed_calls = []
    failed_events = []
    real_fsync = protocol_service.os.fsync
    failed_root = failed_store.root.stat()

    def fail_root_fsync(fd):
        target = protocol_service.os.fstat(fd)
        if (target.st_dev, target.st_ino) == (failed_root.st_dev, failed_root.st_ino):
            failed_events.append("root-fsync")
            raise OSError("injected root fsync failure")
        return real_fsync(fd)

    monkeypatch.setattr(protocol_service.os, "fsync", fail_root_fsync)

    with pytest.raises(OSError, match="injected root fsync failure"):
        create_protocol_job(
            _live_protocol_payload(),
            dry_run=False,
            store=failed_store,
            handlers={"move": lambda *_args: failed_calls.append("handler")},
        )

    assert failed_events == ["root-fsync"]
    assert failed_calls == []

    successful_store = ProtocolOperatorBundleStore(tmp_path / "successful")
    successful_events = []
    successful_root = successful_store.root.stat()

    def track_root_fsync(fd):
        target = protocol_service.os.fstat(fd)
        if (target.st_dev, target.st_ino) == (successful_root.st_dev, successful_root.st_ino):
            successful_events.append("root-fsync")
        return real_fsync(fd)

    monkeypatch.setattr(protocol_service.os, "fsync", track_root_fsync)
    create_protocol_job(
        _live_protocol_payload(),
        dry_run=False,
        store=successful_store,
        handlers={"move": lambda *_args: successful_events.append("handler") or {"ok": True}},
    )

    assert successful_events == ["root-fsync", "handler"]


@pytest.mark.parametrize("failing_fsync_call", [2, 3], ids=["reservation-file", "reservation-directory"])
def test_live_protocol_reservation_durability_failure_rejects_before_handler(
    tmp_path, monkeypatch, failing_fsync_call
) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    calls = []
    real_fsync = protocol_service.os.fsync
    fsync_calls = 0

    def fault_injected_fsync(fd):
        nonlocal fsync_calls
        fsync_calls += 1
        if fsync_calls == failing_fsync_call:
            raise OSError(f"injected fsync failure {failing_fsync_call}")
        return real_fsync(fd)

    monkeypatch.setattr(protocol_service.os, "fsync", fault_injected_fsync)

    with pytest.raises(OSError, match="injected fsync failure"):
        create_protocol_job(
            _live_protocol_payload(),
            dry_run=False,
            store=store,
            handlers={"move": lambda *_args: calls.append("called")},
        )

    assert calls == []


def _seed_replay_artifacts(tmp_path):
    store = ProtocolOperatorBundleStore(tmp_path)
    calls = []
    bundle = create_protocol_job(
        _live_protocol_payload(),
        dry_run=False,
        store=store,
        handlers={"move": lambda *_args: calls.append("initial") or {"ok": True}},
    )
    reservation_path = tmp_path / bundle["job_id"] / "idempotency-reservation.json"
    bundle_path = tmp_path / bundle["job_id"] / "bundle.json"
    reservation = json.loads(reservation_path.read_text(encoding="utf-8"))
    persisted_bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
    persisted_bundle["execution"]["idempotency_binding"] = {
        "idempotency_key_digest": reservation["idempotency_key_digest"],
        "request_fingerprint": reservation["request_fingerprint"],
    }
    bundle_path.write_text(json.dumps(persisted_bundle, indent=2, sort_keys=True), encoding="utf-8")
    calls.clear()
    return store, calls, reservation_path, bundle_path, reservation, persisted_bundle


def test_live_protocol_bundle_persists_digest_and_fingerprint_without_raw_key(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    payload = _live_protocol_payload()

    bundle = create_protocol_job(
        payload,
        dry_run=False,
        store=store,
        handlers={"move": lambda *_args: {"ok": True}},
    )

    reservation = store.load_live_reservation(bundle["job_id"])
    assert reservation is not None
    assert bundle["execution"]["idempotency_binding"] == {
        "idempotency_key_digest": reservation["idempotency_key_digest"],
        "request_fingerprint": reservation["request_fingerprint"],
    }
    assert payload["idempotency_key"] not in json.dumps(bundle, sort_keys=True)


def test_live_flat_protocol_excludes_raw_key_from_metadata_and_fingerprint(tmp_path) -> None:
    store = ProtocolOperatorBundleStore(tmp_path)
    raw_keys = ["flat-private-key-alpha", "flat-private-key-beta"]
    bundles = []
    reservations = []

    for raw_key in raw_keys:
        nested = _live_protocol_payload()
        document = nested.pop("document")
        nested["idempotency_key"] = raw_key
        flat_payload = {
            **nested,
            **document,
            "flat_metadata_marker": {"retained": ["exact", 7]},
        }
        bundle = create_protocol_job(
            flat_payload,
            dry_run=False,
            store=store,
            handlers={"move": lambda *_args: {"ok": True}},
        )
        bundles.append(bundle)
        reservations.append(store.load_live_reservation(bundle["job_id"]))

    for raw_key, bundle, reservation in zip(raw_keys, bundles, reservations):
        assert reservation is not None
        digest = hashlib.sha256(raw_key.encode("utf-8")).hexdigest()
        assert reservation["idempotency_key_digest"] == digest
        assert bundle["job_id"] == f"protocol-live-{digest}"
        assert "idempotency_key" not in bundle["protocol"]["document"]["metadata"]
        assert bundle["protocol"]["document"]["metadata"]["flat_metadata_marker"] == {
            "retained": ["exact", 7]
        }
        for artifact_path in (tmp_path / bundle["job_id"]).glob("*.json"):
            assert raw_key not in artifact_path.read_text(encoding="utf-8")

    assert reservations[0]["request_fingerprint"] == reservations[1]["request_fingerprint"]


@pytest.mark.parametrize(
    "field,bad_value",
    [
        ("schema_version", "wrong.reservation.schema"),
        ("job_id", "wrong-job"),
        ("idempotency_key_digest", "0" * 64),
        ("request_fingerprint", "f" * 64),
    ],
)
def test_live_protocol_replay_rejects_mismatched_reservation_artifact_before_handler(
    tmp_path, field, bad_value
) -> None:
    store, calls, reservation_path, _bundle_path, reservation, _bundle = _seed_replay_artifacts(tmp_path)
    reservation[field] = bad_value
    reservation_path.write_text(json.dumps(reservation, indent=2, sort_keys=True), encoding="utf-8")

    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(
            _live_protocol_payload(),
            dry_run=False,
            store=store,
            handlers={"move": lambda *_args: calls.append("replayed")},
        )

    assert exc_info.value.to_payload()["idempotency_recovery_required"] is True
    assert calls == []


@pytest.mark.parametrize(
    "defect",
    [
        "corrupt_json",
        "malformed_payload",
        "schema_version",
        "job_id",
        "runtime_job_id",
        "runtime_protocol_id",
        "runtime_dry_run",
        "live_mode",
        "missing_binding",
        "binding_digest",
        "binding_fingerprint",
        "protocol_fingerprint",
        "live_contract_fingerprint",
        "noncompleted_status",
        "runtime_not_completed",
    ],
)
def test_live_protocol_replay_rejects_invalid_bundle_artifact_before_handler(tmp_path, defect) -> None:
    store, calls, _reservation_path, bundle_path, _reservation, bundle = _seed_replay_artifacts(tmp_path)
    if defect == "corrupt_json":
        bundle_path.write_text("{not-json", encoding="utf-8")
    elif defect == "malformed_payload":
        bundle_path.write_text("[]", encoding="utf-8")
    else:
        if defect == "schema_version":
            bundle["schema_version"] = "wrong.bundle.schema"
        elif defect == "job_id":
            bundle["job_id"] = "wrong-job"
        elif defect == "runtime_job_id":
            bundle["execution"]["runtime_state"]["job_id"] = "wrong-runtime-job"
        elif defect == "runtime_protocol_id":
            bundle["execution"]["runtime_state"]["protocol_id"] = "wrong-runtime-protocol"
        elif defect == "runtime_dry_run":
            bundle["execution"]["runtime_state"]["dry_run"] = 0
        elif defect == "live_mode":
            bundle["execution"]["dry_run"] = True
        elif defect == "missing_binding":
            bundle["execution"].pop("idempotency_binding")
        elif defect == "binding_digest":
            bundle["execution"]["idempotency_binding"]["idempotency_key_digest"] = "0" * 64
        elif defect == "binding_fingerprint":
            bundle["execution"]["idempotency_binding"]["request_fingerprint"] = "f" * 64
        elif defect == "protocol_fingerprint":
            bundle["protocol"]["document"]["stages"][0]["actions"][0]["params"]["steps"] = 99
        elif defect == "live_contract_fingerprint":
            bundle["execution"]["live_contract"]["operator_id"] = "different-operator"
        elif defect == "noncompleted_status":
            bundle["status"] = "awaiting_review"
        elif defect == "runtime_not_completed":
            bundle["execution"]["runtime_state"]["completed"] = False
        bundle_path.write_text(json.dumps(bundle, indent=2, sort_keys=True), encoding="utf-8")

    with pytest.raises(ProtocolLiveContractError) as exc_info:
        create_protocol_job(
            _live_protocol_payload(),
            dry_run=False,
            store=store,
            handlers={"move": lambda *_args: calls.append("replayed")},
        )

    assert exc_info.value.to_payload()["idempotency_recovery_required"] is True
    assert calls == []
