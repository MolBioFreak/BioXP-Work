"""Isolated saved-method launch behavior at the wire and workflow boundaries."""
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from tests.deck_scoped_rig import retained_rig
from tests.deck_scoped_installed import installed_retained
from tests.pressure_source_v1_support import pressure_source_transport
from tests.test_protocol_workflow_connected import request_payload, mount_protocol_routes, await_job


def head(monkeypatch, mounted, stuck=False, loaded_on_final_query=False):
    group = pressure_source_transport(loaded_channel=None)
    present, wire = set(mounted), []
    for channel, transport in enumerate(group._transports):
        driver = transport._get_driver()
        transport._initialized = True
        def exchange(command, *, channel=channel, original=driver._send_pipette_command, **kwargs):
            wire.append((command, channel))
            if command == "?31":
                if loaded_on_final_query and wire.count(("?31", 0)) == 4 and channel == 0:
                    present.add(0)
                packet = original(command, **kwargs)
                packet["ack"]["data"][-1] = ord("1" if channel in present else "0")
                return packet
            assert command == "E1R" and kwargs["wait_for_completion"]
            if not stuck:
                present.discard(channel)
            return {"ok": True, "outcome": "tx_only"}
        def wait(timeout, channel=channel):
            assert 0 < timeout <= 8
            wire.append(("wait", channel))
            return {"ok": True}
        monkeypatch.setattr(driver, "_send_pipette_command", exchange)
        monkeypatch.setattr(driver, "wait_pipette_command_completion", wait)
    return group, wire


@pytest.mark.parametrize("case", ["empty", "partial", "persistent", "final_query_loaded"])
def test_selected_oem_run_job_prefix(monkeypatch, case):
    from bioxp.services.pipette_service import build_oem_pipette_lifecycle_helpers
    from bioxp.pipette.transport import PipetteTipStateError
    mounted = [] if case == "empty" else [0, 2]
    group, wire = head(monkeypatch, mounted, stuck=case == "persistent",
                       loaded_on_final_query=case == "final_query_loaded")
    events = []
    def record(name, *args):
        events.append(name)
        return {"ok": True}
    def pipette(name, call, action, state, identity):
        result = call(group)
        events.append(name)
        return result
    def exists(*args):
        return any(channel._tip_loaded for channel in group._transports)
    bindings = SimpleNamespace(tip_exists=exists, sleep=lambda seconds: record(seconds),
        source_error_event=lambda message, *args: record(message),
        tip_state=lambda changes, *args: record(changes))
    prefix = build_oem_pipette_lifecycle_helpers(before_native_entry=lambda *args: None,
        pipette_call=pipette, source_bindings=bindings, settings={},
        move_to_waste=lambda *args: record("waste"), sweep_handler=None)["run_job_tip_prefix"]
    state = SimpleNamespace(source_model=SimpleNamespace(logical_tip_present=True))
    query = [("?31", i) for i in range(4)]
    eject = [(cmd, i) for cmd in ("E1R", "wait") for i in mounted]
    if case == "persistent":
        with pytest.raises(PipetteTipStateError) as exc:
            prefix(state, source_occurrence_id="run-job")
        assert exc.value.details["remaining_channels"] == mounted
        retry = [(cmd, i) for cmd in ("E1R", "wait") for i in range(4)]
        assert wire == query + query + eject + query + retry + query
        assert events == ["query_tip_status_all", .5, "waste"]
        assert state.source_model.logical_tip_present
        return
    result = prefix(state, source_occurrence_id="run-job")
    assert result["ok"]
    expected = ["query_tip_status_all", .5]
    if mounted:
        expected += ["waste", "eject_all_tips", "query_tip_status_all", .1]
    assert wire == (query + query + eject + query + query if mounted else query)
    if case == "final_query_loaded":
        assert result["source_pause_scripts"] and result["source_error_hold"]
        assert events == expected + ["Please manually remove tips on Pipette then start"]
        assert state.source_model.logical_tip_present
    else:
        assert events == expected + [{"tip_loaded": False}]
        assert not state.source_model.logical_tip_present


@pytest.mark.parametrize("case", ["empty", "partial", "persistent", "final_loaded", "quick", "recovery"])
def test_saved_method_launch_prefix_and_replay(installed_retained, monkeypatch, tmp_path, case):
    """The worker must finish tip handling before dispatching the first action."""
    from bioxp import api
    from bioxp.protocols.models import ProtocolActionKind
    app, provider, primitive, _, root = installed_retained
    from bioxp.pipette import receipts
    from bioxp.pipette.receipts import PipetteReceiptStore
    monkeypatch.setattr(receipts, "_current_replay_identity", lambda: {})
    monkeypatch.setattr(api, "_pipette_receipts", PipetteReceiptStore(root))
    group, wire = head(monkeypatch, [] if case == "empty" else [0, 2], stuck=case == "persistent",
                       loaded_on_final_query=case == "final_loaded")
    monkeypatch.setattr(api, "_get_pipette_transport", lambda: group)
    monkeypatch.setattr(primitive, "pipette_transport", group, raising=False)
    plans = []
    execute_plan = app.state.oem_workflow_plan_executor
    def track_plan(plan, action, state):
        plans.append(plan["operation"])
        return execute_plan(plan, action, state)
    monkeypatch.setattr(app.state, "oem_workflow_plan_executor", track_plan)
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    starts = []
    def method_action(action, state):
        starts.append((state.job_id, action.action_id))
        return {"ok": True}
    monkeypatch.setattr(api, "_protocol_mechanism_handlers", lambda *args: {ProtocolActionKind.LED: method_action})
    mount_protocol_routes(app)
    client = TestClient(app)
    request = request_payload("launch-tip-method", actions=[{"action_id": "method-entry", "kind": "led", "review_required": case == "empty",
        "params": {"red": 1, "green": 2, "blue": 3}}])
    request["document"]["metadata"] = {"bms_method_run": {"schema": "bms.bioxp-method-run.v1",
        "saved": {"id": "method-123", "revision": 1, "name": "offline witness"}}}
    run = request["document"]["metadata"]["bms_method_run"]
    if case == "quick":
        run["saved"] = None
    elif case == "recovery":
        run["recovery"] = {"original_job_id": "original"}
    if case == "partial":
        dry = client.post("/protocol/execute", json={**request, "dry_run": True, "idempotency_key": "preview"})
        assert dry.status_code == 200, dry.text
        assert not wire and not starts
    accepted = client.post("/protocol/execute", json=request)
    assert accepted.status_code == 202, accepted.text
    job = accepted.json()["job_id"]
    app.state.operator_command_plane.start()
    if case == "empty":
        paused = await_job(client, job, lambda row: row["execution"]["runtime_state"]["workflow"]["gate"] == "review")
        assert wire == [("?31", i) for i in range(4)]
        resumed = client.post(f"/protocol/jobs/{job}/review", json={"idempotency_key": "launch-review",
            "command_id": job, "expected_ownership_generation": paused["command"]["ownership_generation"],
            "stage_id": "one", "action_id": "method-entry", "reviewer": "offline"})
        assert resumed.status_code == 200, resumed.text
    terminal = await_job(client, job, lambda row: row["command"]["terminal"])
    failed = case in {"persistent", "final_loaded"}
    assert (terminal["command"]["status"] == "completed") != failed, terminal["execution"]
    before_replay = wire.copy()
    replay = client.post("/protocol/execute", json=request)
    assert replay.status_code == 200 and replay.json()["job_id"] == job
    assert starts == ([] if failed else [(job, "method-entry")])
    assert wire == before_replay
    if case in {"quick", "recovery"}:
        assert not wire and not plans
        return
    assert [i for cmd, i in wire if cmd == "E1R"] == {"empty": [], "partial": [0, 2], "final_loaded": [0, 2], "persistent": [0, 2, 0, 1, 2, 3]}[case]
    assert plans == ((["pipette_waste"] if case != "empty" else []) + ([] if failed else ["pipette_tip_state"]))
    assert wire.count(("?31", 0)) == {"empty": 1, "partial": 4, "persistent": 4, "final_loaded": 4}[case]
