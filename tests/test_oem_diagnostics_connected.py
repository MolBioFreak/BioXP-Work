"""Source buttons through manual handler, finite provider and both SQLite owners.
Only native IO is replaced; no network/device IO is allowed.
"""
import json
import socket
from contextlib import nullcontext
from pathlib import Path

import pytest
from bioxp.manual_pipetting import compile_manual_pipetting, manual_physical_plan, bind_manual_physical_handler
from bioxp.oem_deck_movement import make_wp8_operation_executor
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.protocols.runtime_state import ProtocolRuntimeState
from tests.test_oem_pipette_calibration import rig

_SOCKET = socket.socket


@pytest.fixture
def diagnostic(rig, monkeypatch, tmp_path):
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    bind_collection_test_identity(monkeypatch)
    monkeypatch.setattr(socket, "socket", lambda family=socket.AF_INET, *a, **kw:
        _SOCKET(family, *a, **kw) if family == socket.AF_UNIX else pytest.fail("network socket"))
    p, store = rig.provider, rig.store
    store.bind_deck_owner_authority_reader(p.deck_owner_authority_stamps, scope=nullcontext)
    receipts = PipetteReceiptStore(tmp_path)
    store.bind_workflow_dispatcher(lambda claimed: None)
    store.admit_workflow(command_id="diagnostic-parent", idempotency_key="diagnostic-parent",
        plan_fingerprint="diagnostic", requested_inputs={"bundle": {"execution": {"runtime_state": {}}}},
        ownership_generation=1, resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
    store.claim_next()
    assert store._renew_owner(lease_seconds=120)
    executor = make_wp8_operation_executor(provider_getter=lambda: p, command_store=store)
    admission = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1},
        "board4_authority": {"active_board_epoch": 1}}}
    ids = []
    def execute(plan, action, state):
        admitted = store.admit_internal_wp8_operation(plan["operation"], inputs={}, state=admission,
            idempotency_key=f"diagnostic-{len(ids)}", prepared_plan=plan)
        ids.append(admitted["command_id"])
        claimed = store.claim_next()
        assert claimed["command_id"] == ids[-1]
        try:
            response = executor(command_id=ids[-1], plan=plan)
        except Exception as exc:
            store.finish(ids[-1], status="failed", payload={"error": str(exc)}, claimed=claimed)
            raise
        store.finish(ids[-1], status="completed", payload={"response": response}, claimed=claimed)
        return response
    handler = bind_manual_physical_handler(command_store=store, execute_plan=execute,
        require_motion_ready=lambda: None, provider_getter=lambda: p, receipt_store_getter=lambda: receipts)
    def run(action, **args):
        doc = compile_manual_pipetting({"protocol_id": "diagnostic", "steps": [
            {"operation": "diagnostic_pipette", "diagnostic": {"action": action, **args}}]})
        runtime = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id="diagnostic-parent")
        response = handler(doc.stages[0].actions[0], runtime)
        body = response["completed_children"][0]["result"]
        assert store.wp8_operation_evidence(ids[-1])["children"][0]["operation"] == "sourceDiagnosticPipette"
        assert store.get_command(ids[-1])["status"] == "completed"
        return body
    for driver in rig.drivers:
        monkeypatch.setattr(driver, "set_top_speed", lambda value, d=driver, **kw:
            d.issue("speed", value=value, **kw), raising=False)
        monkeypatch.setattr(driver, "aspirate", lambda value, d=driver, **kw:
            d.issue("aspirate", volume=value, **kw), raising=False)
        monkeypatch.setattr(driver, "dispense", lambda value, d=driver, **kw:
            d.issue("dispense_liquid", volume=value, **kw), raising=False)
        monkeypatch.setattr(driver, "execute_diagnoses", lambda number, d=driver, **kw:
            d.issue("diagnoses", number=number, **kw), raising=False)
        monkeypatch.setattr(driver, "pipette_eject_tip", lambda d=driver, **kw:
            d.issue("eject", **kw), raising=False)
        monkeypatch.setattr(driver, "query_error_log", lambda value, d=driver:
            (rig.events.append(("error_log", d.channel, value)) or
             {"ok": True, "oem_error_code": 16+d.channel}), raising=False)
        monkeypatch.setattr(driver, "query_firmware", lambda number, d=driver:
            (rig.events.append(("firmware", d.channel, number)) or
             {"ok": True, "firmware": "PART rev A" if number == 0 else "FW1"}), raising=False)
        monkeypatch.setattr(driver, "get_data", lambda query, d=driver, **kw:
            (rig.events.append(("data", d.channel, query)) or
             {"ok": True, "semantic_ok": True, "query": query, "value": "data"}), raising=False)
        monkeypatch.setattr(driver, "pipette_initiate_group", lambda d=driver:
            {**d.issue("WR"), "immediate_ack_received": True}, raising=False)
        monkeypatch.setattr(driver, "pipette_initialize", lambda d=driver, **kw:
            {**d.issue("WR", **kw), "immediate_ack_received": True}, raising=False)
        monkeypatch.setattr(driver, "wait_pipette_initialization_completion", lambda timeout:
            {"ok": True}, raising=False)
        monkeypatch.setattr(driver, "enable_pressure_stream", lambda enabled:
            {"ok": True}, raising=False)
        monkeypatch.setattr(driver, "query_status", lambda d=driver:
            (rig.events.append(("status", d.channel)) or {"ok": True, "reply_received": True,
                "semantic_ok": True, "oem_error_code": 0, "oem_error_free": True}), raising=False)
    yield run, receipts, ids
    receipts.connection.close()


@pytest.mark.parametrize("action", ["aspirate", "dispense"])
def test_reconcile_and_selection(diagnostic, rig, monkeypatch, action):
    run, receipts, _ = diagnostic
    # Stale/no provenance is not a source eligibility gate.
    rig.group._transports[3]._tip_loaded = False
    original = rig.drivers[1].query_tip_status
    monkeypatch.setattr(rig.drivers[1], "query_tip_status", lambda:
        {**original(), "tip_loaded": False, "source_tip_loaded": False, "source_return": 0})
    body = run(action, channels=[2, 1, 0], volume_ul=10., speed=100)
    assert [e[1] for e in rig.events if e[0] == "query"] == [0, 1, 2]
    assert body["lost_tip_channels"] == [1]
    assert body["selected_channels"] == ([0, 1, 2] if action == "aspirate" else [0, 2])
    sends = [e[1] for e in rig.events if e[0] == ("aspirate" if action == "aspirate" else "dispense_liquid")]
    assert sends == body["selected_channels"]
    assert not rig.group._transports[1]._tip_loaded
    assert receipts.read(limit=100)
    keys = [r[0] for r in receipts.connection.execute("SELECT command_id FROM pipette_operations")]
    assert len(keys) == len(set(keys))


def test_cached_dispense_all_and_diagnoses(diagnostic, rig, monkeypatch):
    run, _, _ = diagnostic
    for driver in rig.drivers:
        original = driver.wait_pipette_command_completion
        monkeypatch.setattr(driver, "wait_pipette_command_completion",
            lambda timeout, owner_token, f=original, d=driver: {
                **f(timeout, owner_token=owner_token),
                "pipette_message_state": {"diagnosis": f"#{d.channel} `passed"}})
    rig.group._transports[1]._tip_loaded = False
    rig.group._transports[3]._tip_loaded = False
    rig.group._tip_location = 1  # deliberately unrelated to cached eligibility
    assert rig.group._cached_tip_channels() == []  # ordinary primitive policy stays unchanged
    body = run("dispense_all")
    assert body["cached_tip_channels"] == [0, 2]
    assert body["dispense_all"]["timeout_ms"] == 7000
    assert [e[1] for e in rig.events if e[0] == "dispense"] == [0, 2]
    body = run("diagnoses")
    assert [t["number"] for t in body["tests"]] == [0, 1, 2]
    assert [(e[1], e[2]["number"]) for e in rig.events if e[0] == "diagnoses"] == [
        (0,0),(2,0),(0,1),(2,1),(0,2),(2,2)]
    assert not any(e[0] == "query" for e in rig.events)
    assert all(len(t["channels"]) == 4 for t in body["tests"])
    assert [r["display"] for r in body["tests"][0]["channels"]] == [
        "#0passed", "No data returned", "#2passed", "No data returned"]


@pytest.mark.parametrize("retry", [False, True])
def test_initialize_is_not_constructor(diagnostic, rig, monkeypatch, retry):
    run, _, _ = diagnostic
    driver = rig.drivers[0]
    original = driver.query_status
    count = []
    def status():
        count.append(1)
        return {**original(), "oem_error_code": 1 if retry and len(count) == 1 else 0}
    monkeypatch.setattr(driver, "query_status", status)
    body = run("initialize")
    assert len(body["attempts"]) == (2 if retry else 1)
    assert len([e for e in rig.events if e[0] == "WR"]) == (8 if retry else 4)
    assert not any(e[0] == "firmware" for e in rig.events)


def test_eject_queries_all_but_only_selected_present(diagnostic, rig, monkeypatch):
    run, _, _ = diagnostic
    original = rig.drivers[1].query_tip_status
    monkeypatch.setattr(rig.drivers[1], "query_tip_status", lambda:
        {**original(), "tip_loaded": False, "source_tip_loaded": False, "source_return": 0})
    body = run("eject", channels=[3,1])
    assert body["ejected_channels"] == [3]
    assert [e[:2] for e in rig.events if e[0] in {"query", "eject"}] == [
        ("query",0),("query",1),("query",2),("query",3),("eject",3)]
    assert not rig.native.moves


def test_data_and_last_error_assembled(diagnostic, rig):
    run, _, _ = diagnostic
    body = run("get_data")
    assert [(r["part_number"], r["revision"], r["firmware"]) for r in body["channels"]] == [
        ("PART", "rev A", "FW1")] * 4
    assert [e for e in rig.events if e[0] == "firmware"] == [
        ("firmware",ch,n) for ch in range(4) for n in (0,1)]
    body = run("last_error")
    assert [r["display"] for r in body["channels"]] == ["0x10","0x11","0x12","0x13"]
    assert [e for e in rig.events if e[0] == "error_log"] == [("error_log",ch,1) for ch in range(4)]


@pytest.mark.parametrize("action", ["plunger_up", "plunger_down"])
def test_plunger_source_current_then_relative(diagnostic, rig, monkeypatch, action):
    run, _, _ = diagnostic
    def relative(steps, **kwargs):
        assert rig.native.parameters[4,1,6] == 31
        rig.events.append(("relative_z", steps))
        return {"ok": True, "command_sent": True}
    monkeypatch.setattr(rig.native, "motor_z_move_relative_strict", relative, raising=False)
    body = run(action, steps=123)
    assert [e for e in rig.events if e[0] == "relative_z"] == [("relative_z", -123 if action == "plunger_up" else 123)]
    assert [e["operation"] for e in body["events"]] == ["setZaxisCurrentmax31", "moveStepsZ"]


@pytest.mark.parametrize("diagnostic", [
    {"action":"wire", "opcode":"WR"}, {"action":"last_error", "raw_byte":2},
    {"action":"eject", "channels":[4]}, {"action":"eject", "channels":[1,1]},
    {"action":"aspirate", "channels":[0], "volume_ul":1., "speed":True},
    {"action":"plunger_up", "steps":-1}, {"action":"diagnoses", "number":2}])
def test_closed_contract(diagnostic):
    with pytest.raises(ValueError):
        manual_physical_plan({"operation":"diagnostic_pipette", "diagnostic":diagnostic})


def test_exported_bms_requests_and_schema():
    from bioxp.manual_pipetting import ManualDiagnosticPipette
    from bioxp.protocols.models import ProtocolDocument
    folder = Path(__file__).parents[1] / "testdata" / "pipette_diagnostics"
    examples = json.loads((folder / "requests.json").read_text())
    assert len(examples) == 10
    assert json.loads((folder / "schema.json").read_text()) == ManualDiagnosticPipette.model_json_schema()
    for example in examples:
        document = ProtocolDocument.from_payload(example["native_document"])
        assert document.to_payload() == compile_manual_pipetting(example["manual_request"]).to_payload()
        plan = manual_physical_plan(document.stages[0].actions[0].params)
        assert plan["operation"] == "diagnostic_pipette"
        assert plan["children"][0]["arguments"]["diagnostic"]["action"] == example["action"]


def test_exception_keeps_partial_children(diagnostic, rig, monkeypatch):
    run, receipts, ids = diagnostic
    def fail(number, **kwargs):
        if number == 1:
            raise RuntimeError("injected diagnose transport failure")
        return rig.drivers[2].issue("diagnoses", number=number, **kwargs)
    monkeypatch.setattr(rig.drivers[2], "execute_diagnoses", fail)
    with pytest.raises(Exception) as error:
        run("diagnoses")
    assert "injected diagnose transport failure" in str(error.value.provider_results)
    assert not any(e[0] == "diagnoses" and e[2]["number"] == 2 for e in rig.events)
    assert receipts.read(limit=100)
    assert rig.store.get_command(ids[-1])["status"] == "failed"
    assert len(error.value.provider_results[0]["tests"]) == 1


@pytest.mark.parametrize("action", ["aspirate", "dispense", "dispense_all", "diagnoses"])
def test_ignored_wait_false_is_evidence_not_new_gate(diagnostic, rig, monkeypatch, action):
    run, _, _ = diagnostic
    for driver in rig.drivers:
        original = driver.wait_pipette_command_completion
        monkeypatch.setattr(driver, "wait_pipette_command_completion", lambda timeout, owner_token, f=original:
            {**f(timeout, owner_token=owner_token), "ok": False})
    args = {"channels":[0], "volume_ul":1.25, "speed":100} if action in {"aspirate", "dispense"} else {}
    body = run(action, **args)
    assert body["ok"] and body["source_return_completed"]
    assert not body["controller_outcome_ok"]
    if action == "diagnoses":
        assert len(body["tests"]) == 3


def test_second_bad_status_is_not_third_retry(diagnostic, rig, monkeypatch):
    run, _, _ = diagnostic
    for driver in rig.drivers:
        monkeypatch.setattr(driver, "query_status", lambda: {"ok": False, "oem_error_code": 5})
        monkeypatch.setattr(driver, "wait_pipette_initialization_completion", lambda timeout: {"ok": False})
    try:
        body = run("initialize")
    except Exception as exc:
        pytest.fail(str([(r.get("error"), r.get("exception_type")) for r in exc.provider_results]))
    assert len(body["attempts"]) == 2
    assert body["ok"] and body["source_return_completed"]
    assert not body["controller_outcome_ok"]
    assert len([e for e in rig.events if e[0] == "WR"]) == 8


@pytest.mark.parametrize("action", ["aspirate", "dispense", "dispense_all", "diagnoses"])
def test_empty_selection_and_cache_do_not_acquire_new_gates(diagnostic, rig, action):
    run, _, _ = diagnostic
    for t in rig.group._transports:
        t._tip_loaded = False
    args = {"channels":[], "volume_ul":1., "speed":100} if action in {"aspirate", "dispense"} else {}
    assert run(action, **args)["ok"]
    assert not any(e[0] in {"query", "speed", "aspirate", "dispense_liquid", "dispense", "diagnoses"}
                   for e in rig.events)


@pytest.mark.parametrize("action", ["aspirate", "dispense_all", "diagnoses"])
def test_addressed_stop_interrupts_existing_group_owner(diagnostic, rig, monkeypatch, action):
    run, _, _ = diagnostic
    driver = rig.drivers[0]
    original = driver.wait_pipette_command_completion
    stopped = []
    def interrupt(timeout, *, owner_token):
        if not stopped and not owner_token.startswith("TR"):
            stopped.append(True)
            rig.group.terminate()
        return original(timeout, owner_token=owner_token)
    monkeypatch.setattr(driver, "wait_pipette_command_completion", interrupt)
    args = {"channels":[0], "volume_ul":1., "speed":100} if action == "aspirate" else {}
    with pytest.raises(Exception) as error:
        run(action, **args)
    body = error.value.provider_results[0]
    assert body["interrupted_by_terminate"] and not body["ok"]
    assert [e[1] for e in rig.events if e[0] == "TR"] == [0,1,2,3]
    if action == "aspirate":
        assert not any(e[0] == "aspirate" for e in rig.events)
    if action == "diagnoses":
        assert len(body["tests"]) == 1


def test_malformed_information_stops_at_source_indexing(diagnostic, rig, monkeypatch):
    run, _, _ = diagnostic
    monkeypatch.setattr(rig.drivers[1], "query_firmware", lambda number:
        (rig.events.append(("firmware",1,number)) or {"ok": True, "firmware": "malformed"}))
    with pytest.raises(Exception) as error:
        run("get_data")
    body = error.value.provider_results[0]
    assert body["exception_type"] == "IndexError"
    assert [r["channel"] for r in body["channels"]] == [0]
    assert ("firmware",1,1) not in rig.events


def test_native_interlock_refusal_is_preserved(diagnostic, rig, monkeypatch):
    run, _, _ = diagnostic
    monkeypatch.setattr(rig.native, "motor_oem_require_no_motion_profile", lambda *a, **kw:
        {"ok": False, "failure": "native_motion_interlock"})
    with pytest.raises(Exception) as error:
        run("plunger_down", steps=100)
    assert not error.value.provider_results[0]["ok"]
    assert not any(e[0] == "relative_z" for e in rig.events)
