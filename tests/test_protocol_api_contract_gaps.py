"""Protocol API contract repairs: envelopes, list row kinds, dry-run preview,
manual passthrough, record-only manifest outside the idempotency fingerprint.

Pure service/route calls with no hardware owner.
"""
import asyncio

import pytest
from fastapi import HTTPException
from fastapi.testclient import TestClient

from bioxp import api
from bioxp.operator_command_plane import DeckRecordRequest
from bioxp.services import protocol_service
from tests.test_manual_pipetting import program


def test_deck_manifest_does_not_change_request_fingerprint():
    protocol = {"protocol_id": "p"}
    base = {"protocol_id": "p", "action_count": 1, "created_at": "a"}
    a = protocol_service._request_fingerprint(protocol, {**base, "deck_manifest": {"trays": 1}})
    b = protocol_service._request_fingerprint(protocol, {**base, "deck_manifest": {"trays": 2}, "created_at": "b"})
    c = protocol_service._request_fingerprint(protocol, {**base, "action_count": 2})
    assert a == b
    assert a != c


def test_list_rows_carry_row_kind():
    class Canonical:
        def list_workflows(self, *, limit):
            return [{"job_id": "live-1"}]

    class Historical:
        def list(self, *, limit):
            return [{"job_id": "live-1"}, {"job_id": "old-1"}]

    rows = protocol_service.list_protocol_jobs(limit=5, store=Historical(), command_store=Canonical())
    assert rows == [{"job_id": "live-1", "row_kind": "canonical_workflow"},
                    {"job_id": "old-1", "row_kind": "historical_bundle"}]


def test_dry_run_reports_admission_preview_without_enforcing(monkeypatch, tmp_path):
    store = protocol_service.ProtocolOperatorBundleStore(tmp_path)
    create = protocol_service.create_protocol_job
    monkeypatch.setattr(api, "create_protocol_job", lambda payload, **kw: create(payload, store=store, **kw))
    monkeypatch.setattr(api, "_get_tester", lambda: pytest.fail("dry run touched hardware"))
    response = TestClient(api.app).post("/liquid/manual/execute", json=program(),
                                        headers={"Idempotency-Key": "manual-preview-test"})
    assert response.status_code == 200, response.text
    preview = response.json()["artifacts"]["admission_preview"]
    assert preview["enforced"] is False
    assert preview["binding_support"] == "not_evaluated_in_dry_run"
    assert preview["prepared_tip_inventory"]["ok"] is None


def test_manual_execute_passes_manifest_operator_and_preflight(monkeypatch):
    calls = []

    async def execute(req):
        calls.append(req.model_dump(exclude_unset=True))
        return {"ok": True}

    monkeypatch.setattr(api, "protocol_execute", execute)
    request = api.ManualPipettingExecuteRequest(**program(), operator_id="christian",
        deck_manifest={"note": "tray 2 refilled"}, preflight={"artifact_refs": ["a"]})
    asyncio.run(api.liquid_manual_execute(request))
    assert calls[0]["operator_id"] == "christian"
    assert calls[0]["deck_manifest"] == {"note": "tray 2 refilled"}
    assert calls[0]["preflight"] == {"artifact_refs": ["a"]}


def test_execute_input_error_is_an_object_envelope(monkeypatch):
    def bad(payload, **kwargs):
        raise ValueError("bad document")

    monkeypatch.setattr(api, "create_protocol_job", bad)
    with pytest.raises(HTTPException) as raised:
        asyncio.run(api.protocol_execute(api.ProtocolExecuteRequest(source_type="native", document={}, dry_run=True)))
    assert raised.value.status_code == 400
    assert raised.value.detail["error"] == "protocol_input_invalid"
    assert raised.value.detail["message"] == "bad document"


def test_deck_record_defaults_identity_and_reason():
    first = DeckRecordRequest(tip_trays=[{"tray": 2, "tips": "full"}])
    second = DeckRecordRequest(tip_trays=[{"tray": 2, "tips": "full"}])
    assert first.schema_version == "bioxp.operator_deck_record_request.v1"
    assert first.record_id.startswith("auto-") and first.record_id != second.record_id
    assert first.reason == "operator deck record"


def test_motion_off_toggle_blocks_until_activation(monkeypatch):
    import copy
    saved = copy.deepcopy(api._maintenance_state)
    try:
        result = asyncio.run(api.motion_oem_deactivate())
        assert result["motion_enabled"] is False and result["hardware_motion_commanded"] is False
        state = api._maintenance_state_payload()
        assert state["motion_blocked"] is True and state["block_reason"] == "operator_motion_off"
        actions, dispatch = api._build_operator_catalog(api.app) if hasattr(api, "_build_operator_catalog") else (None, None)
        from bioxp.operator_controls import _build_catalog
        actions, dispatch = _build_catalog(api.app)
        assert "meta.deactivate_motion" in dispatch
        assert any(a["action_id"] == "meta.deactivate_motion" and a["enabled"] for a in actions)
    finally:
        api._maintenance_state.clear(); api._maintenance_state.update(saved)


def test_motion_off_has_no_hardware_dependencies():
    from bioxp import operator_controls as oc
    assert oc._safety("POST", "/motion/oem/deactivate") == "service"
    assert oc._motor_motion_action({"informational_method": "POST", "informational_path": "/motion/oem/deactivate", "safety_class": "service"}) is False
