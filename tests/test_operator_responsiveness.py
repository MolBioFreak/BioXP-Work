"""Offline responsiveness regressions: no device, network or physical claims."""
import asyncio

import time
from types import SimpleNamespace

import pytest
from httpx import ASGITransport, AsyncClient

from bioxp.hardware_status import HardwareStateOwner
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
from bioxp.runtime_audit_store import RuntimeAuditDatabase
from bioxp import operator_controls
from test_operator_controls import make_app


@pytest.fixture(autouse=True)
def prohibit_device_discovery(monkeypatch):
    import usb.core
    def forbidden(*args, **kwargs):
        raise AssertionError("offline responsiveness tests must not discover USB")
    monkeypatch.setattr(usb.core, "find", forbidden)


def test_projection_does_not_copy_unrequested_domains():
    owner = HardwareStateOwner()
    owner._snapshot = {
        "snapshot_id": "fixture", "ownership_epoch": 0,
        "completed_monotonic": time.monotonic(), "domains": {
            "transport": {"status": "observed", "observed_unix": time.time(), "observation": {"nested": [1]}},
            "camera": {"status": "observed", "observation": None},
        },
    }
    class DoNotCopy:
        def __deepcopy__(self, memo):
            raise AssertionError("unrequested domain copied")
    owner._snapshot["domains"]["camera"]["observation"] = DoNotCopy()
    projected = owner.project("transport")
    projected["domains"]["transport"]["observation"]["nested"].append(2)
    assert owner._snapshot["domains"]["transport"]["observation"]["nested"] == [1]


def test_provider_projection_scope_verifies_receipts_once_and_never_reuses_across_reads(tmp_path, monkeypatch):
    store = OEMRuntimeStore(tmp_path)
    state = Serial206OemInitializationProvider(object())._new_state()
    store.write_oem_serial206_initialization_state(state)
    calls = []
    original = store._serial206_receipt_set_locked
    def verify():
        calls.append(True)
        return original()
    monkeypatch.setattr(store, "_serial206_receipt_set_locked", verify)
    try:
        with store.serial206_projection_scope():
            for _ in range(6):
                result = store.read_oem_serial206_initialization_state()
                result["fixture_mutation"] = True
        assert len(calls) == 1
        assert "fixture_mutation" not in store.read_oem_serial206_initialization_state()
        assert len(calls) == 2
    finally:
        store.close()


def test_projection_scope_invalidates_for_another_connection_write(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    other = OEMRuntimeStore(tmp_path)
    state = Serial206OemInitializationProvider(object())._new_state()
    store.write_oem_serial206_initialization_state(state)
    try:
        with store.serial206_projection_scope():
            assert "fixture_revision" not in store.read_oem_serial206_initialization_state()
            state["fixture_revision"] = "new"
            other.write_oem_serial206_initialization_state(state)
            assert store.read_oem_serial206_initialization_state()["fixture_revision"] == "new"
    finally:
        other.close()
        store.close()


def test_passive_x_projection_never_queries_controller_but_explicit_read_does():
    calls = []
    primitive = SimpleNamespace(x_terminal_status=lambda: calls.append(True) or {"ok": True})
    provider = Serial206OemInitializationProvider(primitive)
    provider.x_projection()
    assert calls == []
    provider.x_projection(observe_controller=True)
    assert calls == [True]


def test_audit_batch_is_one_full_transaction_preserving_all_records(tmp_path):
    owner = OEMRuntimeStore(tmp_path)
    owner.close()
    db = RuntimeAuditDatabase(tmp_path)
    trace = []
    db.connection.set_trace_callback(trace.append)
    try:
        events = [dict(command_id=None, pipette_operation_id=None,
                       event_source="fixture", event_kind="ingress",
                       event_payload={"value": i}, source_sequence=i) for i in range(128)]
        ids = db.record_event_batch(events)
        assert len(set(ids)) == 128
        assert sum(sql == "COMMIT" for sql in trace) == 1
        assert sum(sql == "BEGIN IMMEDIATE" for sql in trace) == 1
        assert [row[0] for row in db.connection.execute(
            "SELECT source_sequence FROM runtime_events WHERE event_source='fixture' ORDER BY event_id"
        )] == list(range(128))
        assert not db.connection.in_transaction
    finally:
        db.close()


def test_owned_z_board_cycle_still_invalidates_y_reference(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        store.record_board4_transition(active=True, ack={"status": 100}, transition_id="on",
                                       ownership_generation=7, invalidate_axes=False)
        store.prepare_axis_authority("y", ownership_generation=7, profile_fingerprint="p")
        store.publish_axis_reference("y", position_steps=0, ownership_generation=7)
        result = store.record_board4_transition(active=False, ack={"status": 100}, transition_id="off",
                                               ownership_generation=7, invalidate_axes=False)
        assert result["axes"]["y"]["lifecycle_state"] == "generation_stale"
        assert result["axes"]["y"]["reference_state"] == "generation_stale"
    finally:
        store.close()


@pytest.mark.asyncio
async def test_retained_admission_returns_before_provider_and_never_queues_or_reissues(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    entered, release = asyncio.Event(), asyncio.Event()
    calls = []
    async def held_dispatch(*args, **kwargs):
        calls.append(True)
        entered.set()
        await release.wait()
        return 200, {"ok": True}
    monkeypatch.setattr(operator_controls, "_dispatch_asgi", held_dispatch)
    payload = {"schema_version": "bioxp.operator_action_request.v2",
               "expected_ownership_generation": 7, "expected_board_epoch_by_board": {},
               "idempotency_key": "responsive-admission-fixture", "inputs": {}}
    async with app.router.lifespan_context(app):
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            started = time.monotonic()
            response = await asyncio.wait_for(client.post(
                "/operator/v2/actions/meta.activate_motion", json=payload), timeout=1)
            assert time.monotonic() - started < 1
            assert response.status_code == 200, response.text
            receipt = response.json()
            assert receipt["terminal"] is False and receipt["status"] == "queued"
            await asyncio.wait_for(entered.wait(), timeout=1)
            try:
                same = await client.post("/operator/v2/actions/meta.activate_motion", json=payload)
                assert same.json()["command_id"] == receipt["command_id"]
                busy = await client.post("/operator/v2/actions/meta.activate_motion", json={
                    **payload, "idempotency_key": "responsive-distinct-fixture"})
                assert busy.status_code == 409
                assert busy.json()["detail"]["error"] == "operator_action_busy"
                assert calls == [True]
            finally:
                release.set()
            final = {}
            for _ in range(100):
                final = (await client.get(receipt["status_path"])).json()
                if final["terminal"]:
                    break
                await asyncio.sleep(.01)
            assert final["status"] == "completed", final
            assert calls == [True]


@pytest.mark.asyncio
async def test_status_projection_does_not_block_the_event_loop(monkeypatch):
    from bioxp import api
    def slow_projection():
        time.sleep(.2)
        return {"fixture": True}
    monkeypatch.setattr(api, "_status_payload", slow_projection)
    task = asyncio.create_task(api.get_status())
    started = time.monotonic()
    await asyncio.sleep(.01)
    assert time.monotonic() - started < .1
    assert await task == {"fixture": True}


def test_board_presence_does_not_run_auxiliary_diagnostics():
    from bioxp import api
    calls = []
    tester = SimpleNamespace(BOARDS=(6, 7), query_only_tmcl=lambda *args: calls.append(args) or {"status": 100, "value": 1})
    rows = api._hardware_collectors(tester)["boards"](None)
    assert len(rows) == 2
    assert calls == [(6, 173, 0, 0, 0), (7, 173, 0, 0, 0)]


def test_y_timeout_retains_acknowledged_move_evidence(monkeypatch):
    from bioxp import api
    from bioxp.usb_driver import OemMotionCompletionError
    def fail(**kwargs):
        raise OemMotionCompletionError("fixture OEM wait timeout", evidence={
            "command_sent": True, "ack": {"status": 100}, "requested_position": 10000,
            "timeout_position": {"position": 10767}, "home_stage": "preliminary_rehome_move",
            "homing_sweep_started": False})
    provider = SimpleNamespace(home=fail, schema="fixture")
    monkeypatch.setattr(api, "_require_serial206_y_provider", lambda: provider)
    monkeypatch.setattr(api, "current_operator_dispatch_context", lambda: {"operator_command_id": "fixture"})
    result = api._execute_serial206_y_call("home")
    assert result["ok"] is False
    assert result["physical_motion_commanded"] is True
    assert result["controller_command_acknowledged"] is True
    assert result["physical_effect_verified"] is False
    assert result["source_failure_evidence"]["homing_sweep_started"] is False
    assert result["automatic_retry"] is False


def test_preempted_background_collection_keeps_previous_snapshot():
    owner = HardwareStateOwner()
    owner.collect(["transport"], {"transport": lambda _: {"original": True}})
    snapshot_id = owner.project("transport")["snapshot_id"]
    queried = []
    def first(_):
        queried.append("transport")
        return {"original": False}
    result = owner.collect(["transport", "boards"], {
        "transport": first, "boards": lambda _: queried.append("boards")},
        yield_requested=lambda: bool(queried))
    assert result["published"] is False
    assert queried == ["transport"]
    current = owner.project("transport")
    assert current["snapshot_id"] == snapshot_id
    assert current["domains"]["transport"]["observation"] == {"original": True}


def test_missing_sibling_domain_does_not_erase_fresh_domain_in_batched_projection():
    owner = HardwareStateOwner()
    owner.collect(["power"], {"power": lambda _: {"safety_valid": True}})
    result = owner.project("power", "thermal", independent_domains=True)
    assert result["available"] is False
    assert result["domains"]["power"]["freshness"]["state"] == "fresh"
    assert result["domains"]["power"]["observation"]["safety_valid"] is True
    assert result["domains"]["thermal"] is None
    owner.change_ownership(transport="unbound", usb="unbound", router="stopped", reason="fixture")
    assert owner.project("power", independent_domains=True)["domains"]["power"]["observation"] is None


@pytest.mark.asyncio
async def test_automatic_collection_does_not_queue_behind_foreground_action(monkeypatch):
    from bioxp import api
    monkeypatch.setattr(api.app.state, "operator_normal_action_active", lambda: True, raising=False)
    def forbidden():
        raise AssertionError("a deferred automatic read must not touch the tester")
    monkeypatch.setattr(api, "_get_tester", forbidden)
    result = await api.hardware_snapshot_collect({"automatic": True})
    assert result == {"ok": False, "published": False, "reason": "operator_action_pending"}


def test_interrupt_wake_skips_idle_reconciliation_delay():
    import threading
    from bioxp.operator_command_plane import OperatorCommandStore
    waiting = threading.Event()
    class Wake(threading.Event):
        def wait(self, timeout=None):
            waiting.set()
            return super().wait(timeout)
    calls = []
    resumed = threading.Event()
    stop = threading.Event()
    def reconcile():
        calls.append(time.monotonic())
        if len(calls) == 2:
            resumed.set()
            stop.set()
    stub = SimpleNamespace(_stop=stop, _wake=Wake(), reconcile_pending_interrupts=reconcile,
        _renew_owner=lambda: True, _priority_fence=threading.Event(),
        _worker_lock=threading.Lock(), _workers=set(), _worker_commands={}, claim_next=lambda: None)
    worker = threading.Thread(target=OperatorCommandStore._dispatch_loop, args=(stub, lambda _: None))
    worker.start()
    try:
        assert waiting.wait(1)
        assert len(calls) == 1
        stub._wake.set()
        assert resumed.wait(.75), "interrupt wake was lost to the idle polling interval"
    finally:
        stop.set()
        stub._wake.set()
        worker.join(2)
    assert not worker.is_alive()
