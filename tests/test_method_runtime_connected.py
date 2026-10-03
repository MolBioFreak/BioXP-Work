"""Real routes/dispatcher/executor/SQLite; controller sends are the replaced leaves."""
import json
import threading
import time
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from bioxp import api
from bioxp.protocols import compile_native_protocol, ProtocolExecutor
from bioxp.protocols.runtime_state import ProtocolRuntimeState, ProtocolWorkflowState
from bioxp.services.protocol_service import _job_status_from_state
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_protocol_workflow_connected import mount_protocol_routes, await_job, request_payload
from tests.test_camera_oem_led_binding import led_rig


def action(kind, params=None, **extra):
    return {"action_id": kind, "kind": kind, "params": params or {}, **extra}


@pytest.fixture
def thermal_leaf(monkeypatch):
    from bioxp.usb_driver import BioXpTester
    tester = object.__new__(BioXpTester)
    writes, values = [], {}
    def send(kind, command, typ, bank, value, **kwargs):
        writes.append((kind, command, typ, bank, value))
        if command == 140:
            values[kind, bank] = value
        return {"status": 100, "value": values.get((kind, bank), 25000)}
    tester._send_thermal = lambda *a, **k: send("thermal", *a, **k)
    tester._send_chiller = lambda *a, **k: send("chiller", *a, **k)
    tester.THERMAL_VERIFY_SETTLE_S = 0
    tester.CHILLER_VERIFY_SETTLE_S = 0
    monkeypatch.setattr(api, "_get_tester", lambda: tester)
    return tester, writes


def mount(installed_retained, monkeypatch, tmp_path):
    app = installed_retained[0]
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    mount_protocol_routes(app)
    app.add_api_route("/protocol/jobs/{job_id}/control", api.protocol_job_control, methods=["POST"])
    return app, TestClient(app)


def submit(app, client, actions, key="method-native"):
    payload = request_payload(key, actions)
    reply = client.post("/protocol/execute", json=payload)
    assert reply.status_code == 202, reply.text
    job = reply.json()["job_id"]
    app.state.operator_command_plane.start()
    return job


def test_native_thermal_timer_profile_real_store(installed_retained, monkeypatch, tmp_path, thermal_leaf):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    actions = [action("timer_start", {"timer_id": "incubation", "seconds": 0}),
        action("thermal_setpoint", {"bank": "nest", "target_temp_c": 37}),
        action("chiller_setpoint", {"bank": "rc", "target_temp_c": 10}),
        action("thermal_hold", {"bank": "lid", "target_temp_c": 45, "duration_s": 0,
             "start": "attainment", "tolerance_c": .1, "timeout_s": 1}),
        action("thermal_profile", {"repeat": 2, "segments": [
             {"bank": "nest", "target_temp_c": 55, "duration_s": 0, "start": "dispatch"},
             {"bank": "nest", "target_temp_c": 25, "duration_s": 0, "start": "attainment", "tolerance_c": 0, "timeout_s": 1}]}),
        action("wait", {"seconds": 0}), action("timer_wait", {"timer_id": "incubation"})]
    actions[3]["metadata"] = {"step_id": "authored", "loop_path": [2], "generated": True}
    job = submit(app, client, actions)
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    assert done["command"]["status"] == "completed", done
    state = done["execution"]["runtime_state"]
    rows = state["action_results"]
    assert [r["action_id"] for r in rows] == [a["action_id"] for a in actions]
    assert rows[1]["setpoint_accepted"] and rows[1]["temperature_reached"] is None
    assert rows[3]["temperature_reached"] and rows[3]["dwell_complete"]
    assert rows[3]["metadata"] == actions[3]["metadata"]
    assert len(rows[4]["children"]) == 4 and rows[4]["profile_complete"]
    assert {e["event"] for e in state["events"]} >= {"setpoint_accepted", "temperature_reached", "dwell_complete", "profile_segment_result"}
    assert client.get("/protocol/jobs/" + job).json()["execution"]["runtime_state"] == state
    assert len([w for w in thermal_leaf[1] if w[0] == "thermal" and w[1] == 140]) == 6
    export = __import__('os').environ.get("METHOD_RUNTIME_EXPORT")
    if export:
        Path(export).mkdir(parents=True, exist_ok=True)
        (Path(export) / "thermal-result.json").write_text(json.dumps(done, indent=2))


def test_camera_illumination_real_owner_and_store(installed_retained, led_rig, monkeypatch, tmp_path):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider, calls, _ = led_rig
    monkeypatch.setattr(api, "_camera_provider", provider)
    job = submit(app, client, [action("camera_illumination", {"channel": 2, "on": True})])
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    assert done["command"]["status"] == "completed", done
    row = done["execution"]["runtime_state"]["action_results"][0]
    assert row["state_source"] == "last_successful_command" and row["physical_effect_verified"] is False
    assert calls.seen and provider.illumination_state()["channels"][1]["on"] is True


def control(client, job, generation, **body):
    return client.post(f"/protocol/jobs/{job}/control", json={"command_id": job,
        "idempotency_key": "control-" + body["action"], "expected_ownership_generation": generation, **body})


def test_abort_wait_no_next_child_or_replay(installed_retained, monkeypatch, tmp_path):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    job = submit(app, client, [action("wait", {"seconds": 30}), action("note")])
    running = await_job(client, job, lambda r: any(x.get("pending") for x in r["execution"]["runtime_state"]["action_results"]))
    generation = int(installed_retained[1].generation_provider())
    response = control(client, job, generation, action="abort")
    assert response.status_code in {200, 202}, response.text
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    assert done["command"]["status"] == "interrupted", done
    assert [r["action_id"] for r in done["execution"]["runtime_state"]["action_results"]] == ["wait"]


def test_error_pause_exact_child_no_continue_or_review(installed_retained, monkeypatch, tmp_path):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    actions = [action("note"), action("timer_wait", {"timer_id": "not-started"}, on_error="pause_for_operator",
               metadata={"step_id": "transfer", "generated": True, "child_index": 3}),
               action("wait", {"seconds": 0})]
    job = submit(app, client, actions)
    held = await_job(client, job, lambda r: r["execution"]["runtime_state"].get("workflow", {}).get("gate") == "error_hold")
    state = held["execution"]["runtime_state"]
    assert state["stage_states"]["one"]["completed_actions"] == ["note"]
    assert state["workflow"]["gate_id"] == "timer_wait"
    assert len(state["action_results"]) == 2
    generation = int(installed_retained[1].generation_provider())
    response = control(client, job, generation, action="continue", gate="ordinary_pause", gate_id="timer_wait")
    assert response.status_code >= 400
    response = control(client, job, generation, action="abort")
    assert response.status_code in {200, 202}, response.text
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    assert done["command"]["status"] == "failed"
    assert len(done["execution"]["runtime_state"]["action_results"]) == 2


@pytest.mark.parametrize("document,path", [
    ({"stages": None}, "/stages"), ({"stages": [None]}, "/stages/0"),
    ({"stages": [{"actions": [None]}]}, "/stages/0/actions/0"),
    ({"stages": [{"actions": [{"kind": "not-real"}]}]}, "/stages/0/actions/0"),
    ({"stages": [{"actions": [{"kind": "note", "params": 7}]}]}, "/stages/0/actions/0/params"),
    ({"version": None}, "/version"),
])
def test_compile_route_malformed_is_structured(document, path):
    app = FastAPI()
    app.add_api_route("/protocol/compile", api.protocol_compile, methods=["POST"])
    response = TestClient(app).post("/protocol/compile", json={"document": document})
    assert response.status_code == 422, response.text
    detail = response.json()["detail"]
    assert detail["category"] == "representation" and detail["path"] == path


def test_active_reconciliation_and_settled_unknown_are_distinct():
    state = ProtocolRuntimeState(protocol_id="p", job_id="j", dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id="j", phase="reconciling")
    assert _job_status_from_state(state) == "dispatched"
    state.record_event("workflow_settled", detail={"outcome": "ambiguous"})
    restored = ProtocolRuntimeState.from_payload(state.to_payload())
    assert _job_status_from_state(restored) == "ambiguous"


def test_photo_only_and_stationary_decoder_real_camera(installed_retained, monkeypatch, tmp_path):
    from io import BytesIO
    from types import SimpleNamespace
    from PIL import Image
    from bioxp.camera_provider import CameraProvider, CameraIdentity
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    image = BytesIO()
    Image.new("RGB", (640, 480), "white").save(image, format="JPEG")
    calls = []
    def capture(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0, stdout=image.getvalue(), stderr=b"")
    provider = CameraProvider(runner=capture)
    monkeypatch.setattr(provider, "discover", lambda: CameraIdentity("/fixture/video0", "camera", "2084", "f37d"))
    monkeypatch.setattr(api, "_camera_provider", provider)
    monkeypatch.setattr(api, "_get_tester", lambda: pytest.fail("photo must not obtain motion tester"))
    job = submit(app, client, [action("snapshot"), action("barcode_read", {"mode": "stationary"})])
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    rows = done["execution"]["runtime_state"]["action_results"]
    assert done["command"]["status"] == "completed", rows
    assert rows[0]["photo_only"] and rows[0]["artifact"]["artifact_saved"]
    assert Path(rows[0]["artifact"]["path"]).read_bytes() == image.getvalue()
    assert rows[1]["decoded"] is False and rows[1]["value"] == ""
    assert len(calls) == 2


def test_profile_partial_attainment_timeout_no_later_segment(installed_retained, monkeypatch, tmp_path, thermal_leaf):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    tester, writes = thermal_leaf
    # Actual driver read method with a physical response that stays at 25C.
    original = tester._send_thermal
    def send(command, typ, bank, value, **kw):
        result = original(command, typ, bank, value, **kw)
        if command == 10 and typ == 4:
            result["value"] = 25000
        return result
    tester._send_thermal = send
    job = submit(app, client, [action("thermal_profile", {"repeat": 1, "segments": [
        {"bank": "nest", "target_temp_c": 25, "duration_s": 0, "start": "dispatch"},
        {"bank": "nest", "target_temp_c": 37, "duration_s": 0, "start": "attainment", "tolerance_c": 0, "timeout_s": 0},
        {"bank": "nest", "target_temp_c": 55, "duration_s": 0, "start": "dispatch"}]}), action("note")])
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    assert done["command"]["status"] == "failed", done
    rows = done["execution"]["runtime_state"]["action_results"]
    assert len(rows) == 1 and len(rows[0]["children"]) == 2
    assert rows[0]["children"][0]["dwell_complete"]
    assert rows[0]["children"][1]["error"] == "temperature_attainment_timeout"
    assert len([w for w in writes if w[1] == 140]) == 2


def test_native_ordinary_pause_continues_without_replay(installed_retained, monkeypatch, tmp_path):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    job = submit(app, client, [action("wait", {"seconds": .3}), action("note")])
    await_job(client, job, lambda r: any(x.get("pending") for x in r["execution"]["runtime_state"]["action_results"]))
    generation = int(installed_retained[1].generation_provider())
    requested = control(client, job, generation, action="pause", mode="ordinary")
    assert requested.status_code in {200, 202}, requested.text
    held = await_job(client, job, lambda r: r["execution"]["runtime_state"].get("workflow", {}).get("gate") == "ordinary_pause")
    state = held["execution"]["runtime_state"]
    assert state["stage_states"]["one"]["completed_actions"] == ["wait"]
    resumed = control(client, job, generation, action="continue", gate="ordinary_pause", gate_id=state["workflow"]["gate_id"])
    assert resumed.status_code in {200, 202}, resumed.text
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    assert done["command"]["status"] == "completed"
    assert [r["action_id"] for r in done["execution"]["runtime_state"]["action_results"]] == ["wait", "note"]
