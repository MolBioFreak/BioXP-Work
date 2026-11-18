"""Launch characterization. The plain-method test records a gap, not parity."""
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


@pytest.mark.parametrize("case", ["empty", "partial", "persistent"])
def test_plain_new_method_enters_without_tip_prefix_and_replays_once(installed_retained, monkeypatch, tmp_path, case):
    """Counterexample: worker starts a method with persistent simulated tips."""
    from bioxp import api
    from bioxp.protocols.models import ProtocolActionKind
    app, *_ = installed_retained
    group, wire = head(monkeypatch, [] if case == "empty" else [0, 2], stuck=case == "persistent")
    monkeypatch.setattr(api, "_get_pipette_transport", lambda: group)
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    starts = []
    def method_action(action, state):
        starts.append((state.job_id, action.action_id))
        return {"ok": True}
    monkeypatch.setattr(api, "_protocol_mechanism_handlers", lambda *args: {ProtocolActionKind.LED: method_action})
    mount_protocol_routes(app)
    client = TestClient(app)
    request = request_payload("launch-tip-method", actions=[{"action_id": "method-entry", "kind": "led",
        "params": {"red": 1, "green": 2, "blue": 3}}])
    request["document"]["metadata"] = {"bms_method": {"method": {"name": "offline witness"}}}
    accepted = client.post("/protocol/execute", json=request)
    assert accepted.status_code == 202, accepted.text
    job = accepted.json()["job_id"]
    app.state.operator_command_plane.start()
    terminal = await_job(client, job, lambda row: row["command"]["terminal"])
    assert terminal["command"]["status"] == "completed"
    replay = client.post("/protocol/execute", json=request)
    assert replay.status_code == 200 and replay.json()["job_id"] == job
    assert starts == [(job, "method-entry")]
    assert wire == []  # known launch gap; replace with desired sequence when ownership is repaired
