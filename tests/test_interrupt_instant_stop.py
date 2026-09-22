"""Instant, durably recorded operator stops (RCA incident-rca-20260920, F1-F3).

Acceptance criteria exercised here:

1. a stop press is durably recorded at admission, *before* physical delivery;
2. a stop is admitted immediately -- it never queues behind the command it is
   stopping (a held provider lifecycle lock), behind another stop, or behind an
   unbounded lock;
3. a stop that cannot be delivered is refused with an explicit, recorded reason
   (never silently parked);
4. a delivered stop keeps a durable attempt/evidence record even when its
   reconciliation cannot complete (``reconciliation_pending``);
5. startup reconciliation materializes journal-only attempts into durable rows.
"""
import asyncio
import json
import threading
import time
from pathlib import Path

import pytest
from fastapi import HTTPException
from httpx import ASGITransport, AsyncClient

from bioxp import api, operator_controls
from bioxp.interrupt_journal import InterruptJournal
from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import novo_decode
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import (
    Serial206OemInitializationProvider,
    Serial206ProductionPrimitiveAdapter,
)
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.services.reference_service import ReferenceStateStore
from bioxp.usb_driver import BioXpTester
from tests.z_stop_fixtures import HardwareEndpoints, make_app

INTERRUPT_BODY = {
    "schema_version": "bioxp.operator_interrupt_request.v1",
    "idempotency_key": "instant-stop-1",
    "reason": "operator stop",
    "observed_ownership_generation": 1,
    "observed_board_epoch_by_board": {},
}


@pytest.fixture
def producer(tmp_path, monkeypatch):
    """Offline X/Z Stop producer (same shape as the Z Stop qualification rig)."""

    hardware = HardwareEndpoints()
    tester = BioXpTester.__new__(BioXpTester)
    tester._transport_lock = threading.RLock()
    tester._motor_last_tx_ts = {}
    tester._motor_noresp_streak = {}
    # Boards 4 (Y/Z) and 5 (X) present and initialized so the addressed OEM
    # double Stop is actually delivered.
    tester._oem_board_initialized = {4: True, 5: True}
    tester._oem_active_board_lifecycle_generation = 1
    monkeypatch.setattr(tester, "_oem_board_present", lambda board: True)
    monkeypatch.setattr(tester, "oem_no24v_state", lambda: False)
    monkeypatch.setattr(tester, "_machine_config_bundle", lambda: {"ok": True, "config": {
        "config": {"GripperVersion": 1}, "calibration": {"Calibrated": 1},
        "axis_limits": {"z": {"max_steps": 160000}}, "offsets": {
            "m_Z_MOTOR_MAX_CURRENT_DOWN": 25, "m_Z_MOTOR_MAX_CURRENT_UP": 31,
            "m_Z_MOTOR_STALL_GUARD_THRESHOLD": 3}}})
    from types import SimpleNamespace

    monkeypatch.setattr(
        "bioxp.oem_serial206_initialization.load_oem_parity_config",
        lambda _: SimpleNamespace(
            blockers=[], values={"SerialNumber": 206, "CameraCalibrated": False},
            calibration_source="offline-fixture",
        ),
    )
    router = NovoRouter(ep_in=hardware, ep_out=hardware, decode=novo_decode, read_timeout_ms=10)
    tester.novo_router = router
    store = OEMRuntimeStore(tmp_path / "producer")
    references = ReferenceStateStore(tmp_path / "producer" / "references.json")
    adapter = Serial206ProductionPrimitiveAdapter(
        tester, None, authority_provider=lambda: None,
        generation_provider=lambda: 1, reference_store=references,
    )
    provider = Serial206OemInitializationProvider(
        adapter, state_store=store, reference_store=references, generation_provider=lambda: 1,
    )
    router.start()
    try:
        yield provider, tester, hardware
    finally:
        router.shutdown()
        store.close()


def _attach_journal(monkeypatch, root, *, with_store=False, app=None):
    """Point both layers at one write-ahead journal (as production does).

    ``install_operator_control_plane`` mints the command plane, so in the running
    service the operator-actions layer and the API stop routes resolve the same
    ``interrupt_journal``. Tests mount the routes on their own app, so the journal
    is attached to the plane *and* published on the API module app.
    """

    from types import SimpleNamespace

    journal = InterruptJournal(root)
    plane = None if app is None else getattr(app.state, "operator_command_plane", None)
    if plane is not None:
        monkeypatch.setattr(plane, "interrupt_journal", journal, raising=False)
        api_plane = SimpleNamespace(
            store=getattr(plane, "store", None), interrupt_journal=journal
        )
    elif with_store:
        OEMRuntimeStore(root).close()  # prepare the canonical runtime database
        api_plane = SimpleNamespace(
            store=OperatorCommandStore(root), interrupt_journal=journal
        )
    else:
        api_plane = SimpleNamespace(store=None, interrupt_journal=journal)
    monkeypatch.setattr(api.app.state, "operator_command_plane", api_plane, raising=False)
    return journal


def _phases_for_attempt(journal_path: Path, attempt_id: str):
    rows = [
        json.loads(line)
        for line in Path(journal_path).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    return [row["phase"] for row in rows if row["interrupt_attempt_id"] == attempt_id]


# ---------------------------------------------------------------------------
# (1) write-ahead durability
# ---------------------------------------------------------------------------
def test_stop_press_is_durable_before_the_physical_delivery(tmp_path, monkeypatch, producer):
    provider, tester, hardware = producer
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    root = tmp_path / "state"
    app, _ = make_app(tmp_path / "outer", monkeypatch, z_stop_route=api.motion_oem_z_stop, generation=1)
    journal = _attach_journal(monkeypatch, root, app=app)
    observed = {}
    original_write = hardware.write

    def write(*args, **kwargs):
        # First physical write of this Stop: the press must already be durable.
        if hardware.stop_writes == 0:
            rows = [
                json.loads(line)
                for line in journal.path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            observed["attempts"] = {row["interrupt_attempt_id"] for row in rows}
            observed["phases_at_first_write"] = [row["phase"] for row in rows]
            observed["admitted_keys"] = [
                row.get("idempotency_key") for row in rows if row["phase"] == "admitted"
            ]
        return original_write(*args, **kwargs)

    monkeypatch.setattr(hardware, "write", write)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://testserver") as client:
            response = await client.post("/operator/v2/actions/oem.z.stop", json=INTERRUPT_BODY)
            compact = response.json()
            saved = await client.get(compact["status_path"], params={"detail": True})
            return response, saved.json()

    response, saved = asyncio.run(scenario())
    assert response.status_code == 200, response.text
    # Recorded before the first byte reached the controller, and recorded as an
    # admission (not as a completion after the fact).
    assert "admitted" in observed["phases_at_first_write"], observed
    assert observed["phases_at_first_write"][0] == "admitted", observed
    assert "delivered" not in observed["phases_at_first_write"], observed
    assert "terminal" not in observed["phases_at_first_write"], observed
    assert observed["admitted_keys"] == [INTERRUPT_BODY["idempotency_key"]]
    attempt_id = next(iter(observed["attempts"]))
    phases = _phases_for_attempt(journal.path, attempt_id)
    assert phases[0] == "admitted"
    assert phases[-1] == "terminal"
    assert "delivered" in phases
    # The operator-plane ledger is the projection of that same attempt identity.
    ledger = {
        row["interrupt_attempt_id"]
        for row in app.state.operator_command_plane.store.connection.execute(
            "SELECT interrupt_attempt_id FROM operator_plane_interrupt_attempts"
        ).fetchall()
    }
    assert attempt_id in ledger, ledger
    # The durable operator receipt carries the addressed-stop evidence, with the
    # physical effect honestly unverified.
    assert saved["interrupt_evidence"]["controller_stop_acknowledged"] is True
    assert saved["interrupt_evidence"]["physical_effect_verified"] is False
    assert saved["physical_effect_verified"] is False


# ---------------------------------------------------------------------------
# (2) no stop waits on what it is stopping / (3) lock-free delivery
# ---------------------------------------------------------------------------
def test_x_stop_delivers_and_returns_while_the_provider_lifecycle_lock_is_held(
    tmp_path, monkeypatch, producer
):
    provider, tester, hardware = producer
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    _attach_journal(monkeypatch, tmp_path / "state")
    # Another command owns the provider lifecycle lock (the incident's stuck
    # oem.xy.home). A Stop must not queue behind it -- not even for recovery.
    provider._lock.acquire()
    try:
        async def scenario():
            token = operator_controls._DISPATCH_CONTEXT.set({
                "operator_command_id": "instant-x-stop",
                "idempotency_key": "instant-x-stop",
                "expected_ownership_generation": 1,
                "action_id": "oem.x.stop",
            })
            try:
                started = time.monotonic()
                result = await api._run_safety_interrupt_blocking(
                    "serial-206 X double-stop",
                    lambda _: api._execute_provider_x_intent(
                        "stop", {"timeout_s": 3.0}, defer_reconciliation=True
                    ),
                    timeout_s=10.0,
                    surface="x",
                )
                return result, time.monotonic() - started
            finally:
                operator_controls._DISPATCH_CONTEXT.reset(token)

        result, elapsed = asyncio.run(scenario())
    finally:
        provider._lock.release()
    # Delivered (the OEM double Stop reached the controller) and honest about
    # the reconciliation it could not complete.
    assert hardware.stop_writes == 2, result
    assert result.get("reconciliation_pending") is True, result
    assert result.get("recovery_hold") is True, result
    assert result.get("source_call_completed") is True, result
    assert result.get("physical_effect_verified") is False, result
    assert elapsed < 5.0, f"stop waited {elapsed:.2f}s behind the held provider lock"
    # Fail-closed: the X reference is desynced, so nothing moves on stale authority.
    reference = provider.reference_store.snapshot(("x",))
    row = (reference.get("rows") or {}).get("x") if isinstance(reference, dict) else None
    assert isinstance(row, dict) and row.get("state") == "desynced", reference


def test_aggregate_abort_does_not_deadlock_behind_the_stuck_command(producer):
    provider, tester, hardware = producer
    provider._lock.acquire()
    try:
        result = {}

        def run():
            result["value"] = provider.execute_x_stop_interrupt(
                {"command_id": "held-abort", "timeout_s": 3.0}, abort=True
            )

        worker = threading.Thread(target=run, name="abort-held-lock", daemon=True)
        started = time.monotonic()
        worker.start()
        worker.join(15)
        elapsed = time.monotonic() - started
    finally:
        provider._lock.release()
    assert not worker.is_alive(), "aggregate software abort deadlocked behind the held provider lock"
    assert elapsed < 10.0, f"abort blocked {elapsed:.2f}s"
    value = result["value"]
    # Delivered: the software abort ran and invalidated axis authority lock-free.
    assert value["ok"] is False
    assert value["recovery_hold"] is True
    assert value["result"]["aggregate_authority_invalidation"]["reconciliation_pending"] is True
    assert hardware.stop_writes == 0, "software abort must not issue a motor Stop"


# ---------------------------------------------------------------------------
# (4) refusals are recorded, never silently parked
# ---------------------------------------------------------------------------
def test_busy_surface_refusal_is_recorded(tmp_path, monkeypatch):
    root = tmp_path / "state"
    journal = _attach_journal(monkeypatch, root, with_store=True)
    monkeypatch.setattr(api, "_INTERRUPT_LANE_WAIT_S", 0.05)

    async def scenario():
        token = operator_controls._DISPATCH_CONTEXT.set({
            "operator_command_id": "refused-stop",
            "idempotency_key": "refused-stop",
            "expected_ownership_generation": 1,
            "action_id": "oem.x.stop",
            "interrupt_attempt_id": "refused-attempt",
        })
        lane = api._safety_interrupt_lane("x")
        await lane.acquire()
        try:
            with pytest.raises(HTTPException) as exc:
                await api._run_safety_interrupt_blocking(
                    "serial-206 X double-stop", lambda _: {"ok": True},
                    timeout_s=1.0, surface="x",
                )
            return exc.value
        finally:
            lane.release()
            operator_controls._DISPATCH_CONTEXT.reset(token)

    refusal = asyncio.run(scenario())
    assert refusal.status_code == 409
    assert refusal.detail["error"] == "interrupt_lane_busy_same_surface"
    assert refusal.detail["recorded"] is True
    assert refusal.detail["delivery_attempted"] is False
    rows = [
        json.loads(line)
        for line in journal.path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    rejected = [row for row in rows if row["phase"] == "rejected"]
    assert len(rejected) == 1, rows
    assert rejected[0]["interrupt_attempt_id"] == "refused-attempt"
    assert rejected[0]["reason"] == "interrupt_lane_busy_same_surface"
    # The refusal is also recorded in the operator plane's append-only decision log.
    decisions = [
        json.loads(line)
        for line in (Path(refusal.detail["journal_path"]).parent
                     / "operator_interrupt_decisions.v1.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert decisions[-1]["kind"] == "rejected"
    assert decisions[-1]["interrupt_attempt_id"] == "refused-attempt"
    assert decisions[-1]["physical_motion_commanded"] is False


def test_schema_mismatched_stop_press_is_recorded_as_rejected_attempt(tmp_path, monkeypatch):
    root = tmp_path / "state"
    app, _ = make_app(tmp_path / "outer", monkeypatch, generation=1)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://testserver") as client:
            return await client.post("/operator/v2/actions/oem.x.stop", json={
                "schema_version": "bioxp.operator_action_request.v2",
                "idempotency_key": "wrong-schema-stop",
                "expected_ownership_generation": 1,
                "inputs": {},
                "expected_board_epoch_by_board": {},
            })

    response = asyncio.run(scenario())
    assert response.status_code == 422, response.text
    assert response.json()["detail"]["error"] == "interrupt_request_schema_required"
    assert response.json()["detail"]["recorded"] is True
    journal = app.state.operator_command_plane.interrupt_journal
    rows = [
        json.loads(line)
        for line in Path(journal.path).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert [row["phase"] for row in rows] == ["rejected"], rows
    assert rows[0]["reason"] == "interrupt_request_schema_required"
    assert rows[0]["idempotency_key"] == "wrong-schema-stop"


# ---------------------------------------------------------------------------
# (5) delivery and reconciliation are separable
# ---------------------------------------------------------------------------
def test_reconciliation_holds_neither_connection_lease_nor_delivery_lane(tmp_path, monkeypatch):
    _attach_journal(monkeypatch, tmp_path / "state")
    monkeypatch.setattr(api, "_tester", object())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    entered, release = threading.Event(), threading.Event()

    def func(_tester):
        def reconcile():
            entered.set()
            assert release.wait(5)
            return {"ok": True, "axis": "z", "reconciled": True}

        reconcile.delivery_result = {"ok": True, "axis": "z", "delivered": True}
        return reconcile

    async def scenario():
        task = asyncio.create_task(api._run_safety_interrupt_blocking(
            "held reconciliation", func, timeout_s=10.0,
            delivery_only_lease=True, surface="held-reconciliation"))
        assert await asyncio.to_thread(entered.wait, 3)
        # Reconciliation is running and owns nothing: no stop may queue behind it.
        assert not api._tester_transition_lock.locked()
        assert not api._safety_interrupt_lane("held-reconciliation").locked()
        other = await asyncio.wait_for(api._run_safety_interrupt_blocking(
            "second stop", lambda _: {"ok": True, "id": 2}, timeout_s=5.0, surface="second-stop"), 5)
        assert other == {"ok": True, "id": 2}
        release.set()
        return await asyncio.wait_for(task, 10)

    result = asyncio.run(scenario())
    assert result["reconciled"] is True


def test_reconciliation_deadline_reports_pending_with_delivery_facts(tmp_path, monkeypatch):
    _attach_journal(monkeypatch, tmp_path / "state")
    monkeypatch.setattr(api, "_tester", object())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    release = threading.Event()

    def func(_tester):
        def reconcile():
            assert release.wait(5)
            return {"ok": True, "reconciled": True}

        reconcile.delivery_result = {
            "ok": True, "axis": "z", "delivery_completed": True,
            "controller_command_acknowledged": True,
        }
        return reconcile

    monkeypatch.setattr(api, "_INTERRUPT_RECONCILIATION_DEADLINE_S", 0.2)

    async def scenario():
        result = await api._run_safety_interrupt_blocking(
            "pending reconciliation", func, timeout_s=5.0, surface="pending-reconciliation")
        release.set()
        return result

    result = asyncio.run(scenario())
    assert result["reconciliation_pending"] is True, result
    assert result["delivery_completed"] is True
    assert result["controller_command_acknowledged"] is True
    assert result["physical_effect_verified"] is False


# ---------------------------------------------------------------------------
# (5) startup materialization
# ---------------------------------------------------------------------------
def test_startup_materialization_projects_orphaned_attempts(tmp_path):
    root = tmp_path / "state"
    journal = InterruptJournal(root)
    journal.record(
        interrupt_attempt_id="orphan-delivered", action_id="oem.y.stop", phase="admitted",
        idempotency_key="orphan-key-1", observed_ownership_generation=1,
        observed_board_epoch_by_board={4: 9},
    )
    journal.record(
        interrupt_attempt_id="orphan-delivered", action_id="oem.y.stop",
        phase="delivery_attempted",
    )
    journal.record(
        interrupt_attempt_id="orphan-not-delivered", action_id="oem.z.stop", phase="admitted",
        idempotency_key="orphan-key-2", observed_ownership_generation=1,
    )
    OEMRuntimeStore(root).close()  # prepare the canonical runtime database
    store = OperatorCommandStore(root)
    summary = store.materialize_interrupt_journal(journal)
    assert sorted(summary["materialized"]) == ["orphan-delivered", "orphan-not-delivered"]
    # The ledger now carries both attempts, and the delivery-unknown one stays
    # fail-closed with an attempted row plus a pending reconciliation row.
    rows = {
        (row["interrupt_attempt_id"], row["phase"])
        for row in store.connection.execute(
            "SELECT interrupt_attempt_id, phase FROM operator_plane_interrupt_attempts"
        ).fetchall()
    }
    assert ("orphan-delivered", "admitted") in rows
    assert ("orphan-delivered", "attempted") in rows
    assert ("orphan-not-delivered", "admitted") in rows
    assert ("orphan-not-delivered", "attempted") not in rows
    # The not-delivered orphan is recorded as a decision, never as a delivery.
    decisions = [
        json.loads(line)
        for line in (root / "operator_interrupt_decisions.v1.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert [row["kind"] for row in decisions] == ["materialized"]
    assert decisions[0]["reason"] == "process_restart_before_interrupt_delivery"
    assert decisions[0]["attempted"] is False
    pending = store._pending_interrupt_spool_rows()
    assert [row["interrupt_attempt_id"] for row in pending] == ["orphan-delivered"]
    assert pending[0]["error"] == "process_restart_during_interrupt_delivery"
    # The journal records that the projection happened and has no orphans left.
    assert journal.unresolved_attempts() == []
    for attempt in ("orphan-delivered", "orphan-not-delivered"):
        assert "materialized" in _phases_for_attempt(journal.path, attempt)
    # Re-running materialization is idempotent (nothing left to project).
    assert store.materialize_interrupt_journal(journal)["materialized"] == []
