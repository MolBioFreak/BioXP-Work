"""Actual API factory/service/SQLite lifetime forwarding, device-free only."""
import json
import subprocess
import sys
from threading import Event

from fastapi.testclient import TestClient

from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_protocol_workflow_connected import mount_protocol_routes, request_payload, await_job


def test_actual_factory_forwards_lifetime_and_refuses_missing_control_adapter(installed_retained, monkeypatch, tmp_path):
    from bioxp import api
    from bioxp.pipette.transport import FourPipetteTransport
    from bioxp.protocols.executor import ProtocolExecutor
    from bioxp.services import protocol_service
    constructions = []
    class ObservedExecutor(ProtocolExecutor):
        def __init__(self, **kwargs):
            constructions.append(kwargs)
            super().__init__(**kwargs)
        def finalize_source_host(self, state):
            trace.append("script_finally")
            assert provider._wp8_source_script_returned and provider._wp8_stop_event.is_set()
            return super().finalize_source_host(state)
    monkeypatch.setattr(protocol_service, "ProtocolExecutor", ObservedExecutor)
    app, provider, primitive, references, root = installed_retained
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    monkeypatch.setattr(primitive, "pipette_transport", object.__new__(FourPipetteTransport), raising=False)
    def forbidden(*args, **kwargs):
        raise AssertionError("No physical plan belongs to this lifetime test")
    monkeypatch.setattr(app.state, "oem_workflow_plan_executor", forbidden, raising=False)
    monkeypatch.setattr(app.state, "oem_workflow_lifecycle_control_executor", None, raising=False)
    payload = request_payload("source-lifetime", [{"action_id": "body", "stage_id": "one", "kind": "oem_operation",
        "oem_opcode": "step", "source_occurrence_id": "source:body", "params": {"arguments": ["1"]}}])
    payload["document"]["metadata"] = {"input_mode": "oem_prepared", "source_settings": {"JobName": None, "LogPressure": False},
        "source_model": {"logical_tip_present": False, "carried_plate_present": False,
            "tip_trays": [{"tray_id": str(i), "location": 7+i,
                "wells": [{"content": None, "volume": 0, "capacity": 0, "empty": True} for _ in range(96)]} for i in range(4)]}}
    bindings = api._protocol_bindings({"protocol": {"document": payload["document"]}})
    assert callable(bindings.source_script_begin) and callable(bindings.source_script_returned)
    ordinary, native, lifecycle = bindings
    assert "cleanup" in lifecycle and "source_error" not in lifecycle
    assert "script_finally" in lifecycle and "source_error_request" in lifecycle
    assert "source_script_begin" not in lifecycle and "source_script_returned" not in native
    mount_protocol_routes(app)
    client = TestClient(app)
    denied = client.post("/protocol/execute", json=payload)
    assert denied.status_code == 409 and "epilogue_lid" in denied.text, denied.text
    assert provider._wp8_source_script_owner is None
    assert app.state.operator_command_plane.store.list_workflows() == []

    # Other lifecycle doubles ONLY to exercise lifetime forwarding and the
    # actual host finalizer; complete native acceptance has its own fixture.
    entered, release = Event(), Event()
    trace = []
    def hook(name):
        def call(state):
            trace.append(name)
            if name == "run_job":
                assert provider._wp8_source_script_owner is None
            if name == "script_prologue":
                assert provider._wp8_source_script_owner == state.workflow.command_id
                assert not provider._wp8_stop_event.is_set()
                entered.set()
                assert release.wait(8)
            return {"ok": True, "offline_lifecycle_double": name}
        return call
    from bioxp.protocols.models import ProtocolDocument
    doc = ProtocolDocument.from_payload(payload["document"])
    monkeypatch.setattr(provider, "build_oem_lifecycle_handlers", lambda **kw: {
        name: hook(name) for name in ProtocolExecutor.required_lifecycle(doc) if name != "script_finally"})
    accepted = client.post("/protocol/execute", json=payload)
    assert accepted.status_code == 202, accepted.text
    job = accepted.json()["job_id"]
    app.state.operator_command_plane.start()
    try:
        assert entered.wait(5)
        row = client.get("/protocol/jobs/" + job).json()
        assert not row["command"]["terminal"]
        assert provider._wp8_source_script_owner == job
        assert not provider._wp8_stop_event.is_set()
    finally:
        release.set()
    done = await_job(client, job, lambda j: j["command"]["terminal"])
    assert done["command"]["status"] == "completed", done
    assert trace.count("script_prologue") == trace.count("script_finally") == 1
    assert any(c.get("job_id") is None for c in constructions)
    assert any(c.get("job_id") == job for c in constructions)
    assert all(callable(c.get("source_script_begin")) and callable(c.get("source_script_returned")) for c in constructions)
    assert "cleanup" not in trace
    assert provider.wp8_wait_stop("waitStop", {})["signaled"]
    provider.wp8_source_script_returned(command_id=job)
    assert not provider._wp8_stop_event.is_set()
    app.state.operator_command_plane.stop()
    code = "import json,sys;from bioxp.operator_command_plane import OperatorCommandStore;s=OperatorCommandStore(sys.argv[1]);print(json.dumps(s.get_workflow(sys.argv[2])));s.stop()"
    reopened = json.loads(subprocess.check_output([sys.executable, "-c", code, str(root), job], text=True, timeout=25))
    assert reopened["command"] == done["command"]
    assert reopened["execution"]["runtime_state"] == done["execution"]["runtime_state"]
