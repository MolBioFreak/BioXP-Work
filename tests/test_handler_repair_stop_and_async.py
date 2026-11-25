"""Offline regression tests for handler repairs 6 (Stop truthfulness & bus observation) and 7 (off-loop I/O)."""
from __future__ import annotations

import asyncio
import queue
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import httpx2 as httpx
import pytest
from fastapi import FastAPI, HTTPException

from bioxp import api, operator_controls
from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import NovoUsbCanBus, novo_decode, novo_encode
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.services.reference_service import ReferenceStateStore
from bioxp.usb_driver import BioXpTester
from tests.z_stop_fixtures import HardwareEndpoints, make_app


def _build_tester_with_endpoints(endpoints):
    tester = BioXpTester.__new__(BioXpTester)
    tester._transport_lock = threading.RLock()
    tester.dev = SimpleNamespace(label="fake-dev")
    tester.ep_out = endpoints
    tester.ep_in = endpoints
    tester._receiver_audit = None
    tester._oem_transport_generation = 1
    tester._oem_board_initialized = {int(b): True for b in BioXpTester.BOARDS}
    tester._motor_last_tx_ts = {int(b): 0.0 for b in BioXpTester.MOTOR_BOARDS}
    tester._motor_noresp_streak = {int(b): 0 for b in BioXpTester.MOTOR_BOARDS}
    tester._chiller_last_tx_ts = 0.0
    tester._chiller_noresp_streak = 0
    tester._thermal_last_tx_ts = 0.0
    tester._thermal_noresp_streak = 0
    tester._oem_no24v_dropped = False
    tester.novo_router = NovoRouter(ep_in=endpoints, ep_out=endpoints, decode=novo_decode, read_timeout_ms=10)
    tester.novo_router.start()
    return tester


def test_stop_stalled_before_first_write_reports_no_wire_write_and_lease_blocked(tmp_path, monkeypatch):
    endpoints = HardwareEndpoints()
    tester = _build_tester_with_endpoints(endpoints)
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_tester_quarantine", None)

    # Hold NovoRouter.transaction_lock from an ordinary transaction so Stop cannot write.
    blocker_held = threading.Event()
    release_blocker = threading.Event()

    def hold_tx_lock():
        with tester.novo_router.transaction_lock:
            blocker_held.set()
            release_blocker.wait(timeout=5.0)

    blocker = threading.Thread(target=hold_tx_lock, daemon=True)
    blocker.start()
    assert blocker_held.wait(timeout=2.0)

    try:
        async def run():
            with pytest.raises(HTTPException) as exc:
                await api._run_safety_interrupt_blocking(
                    "test stop pre-write stall",
                    lambda t: t.motor_oem_stop_exact(t.BOARD_HEAD, 0),
                    timeout_s=0.15,
                    surface="x-stall-test",
                )
            return exc.value

        err = asyncio.run(run())
        assert err.status_code == 504
        detail = err.detail
        assert detail["error"] == "safety_interrupt_completion_ambiguous"
        assert detail["physical_delivery_started"] is False
        assert detail["wire_write_observed"] is False
        assert detail["controller_ack_observed"] is False
        assert detail["connection_lease_held"] is True
        assert detail["connection_transition_blocked_until_worker_exit"] is True
        assert detail["delivery_worker_still_running"] is True
        assert endpoints.stop_writes == 0
    finally:
        release_blocker.set()
        blocker.join(timeout=5.0)
        tester.novo_router.shutdown(join_timeout_s=2.0)


def test_stop_delayed_after_wire_write_reports_write_observed_and_no_ack(tmp_path, monkeypatch):
    endpoints = HardwareEndpoints()
    # Drop replies to Stop writes so write succeeds and wait blocks.
    endpoints.stop_statuses = iter([None, None, None, None])
    tester = _build_tester_with_endpoints(endpoints)
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_tester_quarantine", None)

    try:
        async def run():
            with pytest.raises(HTTPException) as exc:
                await api._run_safety_interrupt_blocking(
                    "test stop post-write stall",
                    lambda t: t.motor_oem_stop_exact(t.BOARD_HEAD, 0),
                    timeout_s=0.15,
                    surface="x-postwrite-test",
                )
            return exc.value

        err = asyncio.run(run())
        assert err.status_code == 504
        detail = err.detail
        assert detail["physical_delivery_started"] is True
        assert detail["wire_write_observed"] is True
        assert detail["controller_ack_observed"] is False
        assert detail["connection_lease_held"] is True
        assert detail["connection_transition_blocked_until_worker_exit"] is True
        assert endpoints.stop_writes >= 1
    finally:
        tester.novo_router.shutdown(join_timeout_s=2.0)


def test_stop_outer_timeout_during_reconciliation_returns_reconciliation_pending_not_504(tmp_path, monkeypatch):
    endpoints = HardwareEndpoints()
    tester = _build_tester_with_endpoints(endpoints)
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_INTERRUPT_RECONCILIATION_DEADLINE_S", 5.0)

    reconcile_entered = threading.Event()
    release_reconcile = threading.Event()
    reconcile_finished = threading.Event()
    journal_phases = []
    monkeypatch.setattr(
        api,
        "_journal_interrupt_phase",
        lambda phase, **kw: journal_phases.append((phase, kw)),
    )

    # Verify Z finish_stop forwards reconcile.delivery_result and outer timeout during reconciliation
    # returns 200 reconciliation_pending (NOT 504 failed!).
    class FakeZProvider:
        def execute_z_intent(self, *args, **kwargs):
            raise AssertionError("unexpected non-deferred execute_z_intent")

        def execute_z_stop_interrupt(self, *, inputs, expected_generation, idempotency_key, defer_reconciliation=False):
            res = tester.motor_oem_stop_exact(tester.BOARD_HEAD, 2)
            assert res["ok"] is True

            def reconcile():
                reconcile_entered.set()
                release_reconcile.wait(timeout=5.0)
                reconcile_finished.set()
                return {"ok": True, "reconciled": True, "physical_effect_verified": False}

            reconcile.delivery_result = {
                "ok": True,
                "controller_stop_acknowledged": True,
                "axis_delivery_lease_released": True,
                "physical_effect_verified": False,
            }
            return reconcile

    monkeypatch.setattr(api, "_require_serial206_oem_initialization_provider", lambda *a, **k: FakeZProvider())
    monkeypatch.setattr(
        api,
        "current_operator_dispatch_context",
        lambda: {
            "operator_command_id": "cmd-z-1",
            "expected_ownership_generation": 7,
            "idempotency_key": "idem-z-stop-1",
        },
    )

    try:
        async def run():
            result = await api._run_safety_interrupt_blocking(
                "serial-206 Z stop",
                lambda _t: api._execute_provider_z_intent("stop", {"timeout_s": 3.0}, defer_reconciliation=True),
                timeout_s=0.15,
                delivery_only_lease=True,
                surface="z-reconcile-test",
            )
            assert reconcile_entered.wait(timeout=2.0)
            return result

        out = asyncio.run(run())
        assert out["delivered"] is True
        assert out["reconciliation_pending"] is True
        assert out["controller_stop_acknowledged"] is True
        assert out["connection_lease_held"] is False
        assert out["connection_transition_blocked_until_worker_exit"] is False
        assert out["wire_write_observed"] is True
        assert out["controller_ack_observed"] is True
        assert endpoints.stop_writes == 2
        # Must NOT have overwritten 'delivered' with 'failed'!
        assert all(phase != "failed" for phase, _ in journal_phases)
    finally:
        release_reconcile.set()
        assert reconcile_finished.wait(timeout=3.0)
        tester.novo_router.shutdown(join_timeout_s=2.0)


def test_passive_bus_observation_does_not_delay_stop_write_and_drops_stale_reconnect_events():
    endpoints = HardwareEndpoints()
    tester = _build_tester_with_endpoints(endpoints)
    try:
        # Start a 0.6s passive observation in collect_bus_events and _collect_fallback_motor_signals
        # and verify Stop completes in << 0.2s (not blocked behind the 0.6s sleep!).
        obs_out = []
        t1 = threading.Thread(target=lambda: obs_out.append(tester.collect_bus_events(duration_s=0.6, timeout_ms=10)), daemon=True)
        t1.start()
        time.sleep(0.05)
        t0 = time.monotonic()
        res = tester.motor_oem_stop_exact(tester.BOARD_HEAD, 0)
        elapsed = time.monotonic() - t0
        assert res["ok"] is True
        assert elapsed < 0.25, f"Stop delayed by collect_bus_events: {elapsed:.3f}s"
        t1.join(timeout=2.0)

        # Same check for _collect_fallback_motor_signals!
        t2 = threading.Thread(target=lambda: tester._collect_fallback_motor_signals(duration_s=0.6), daemon=True)
        t2.start()
        time.sleep(0.05)
        t0 = time.monotonic()
        res2 = tester.motor_oem_stop_exact(tester.BOARD_HEAD, 0)
        elapsed2 = time.monotonic() - t0
        assert res2["ok"] is True
        assert elapsed2 < 0.25, f"Stop delayed by _collect_fallback_motor_signals: {elapsed2:.3f}s"
        t2.join(timeout=2.0)

        # Across reconnect during passive observation, stale pre-reconnect async events must NOT be returned!
        async_frame = novo_encode(NovoUsbCanBus.build_payload(0, bytes([4, 128, 0, 0, 0, 0, 0, 0]), 8))
        endpoints.replies.put(async_frame)
        time.sleep(0.05)
        reconnect_obs = []
        t3 = threading.Thread(target=lambda: reconnect_obs.append(tester.collect_bus_events(duration_s=0.4, timeout_ms=10)), daemon=True)
        t3.start()
        time.sleep(0.05)
        with tester._transport_guard():
            tester._oem_transport_generation += 1
        t3.join(timeout=2.0)
        assert reconnect_obs == [[]]  # generation changed -> old events dropped
    finally:
        tester.novo_router.shutdown(join_timeout_s=2.0)


# ---------------------------------------------------------------------------
# Item 7: Move known blocking I/O off the API event loop
# ---------------------------------------------------------------------------


def _init_runtime_db(root: Path):
    import hashlib, json, sqlite3
    from bioxp.oem_runtime_store import migrate_runtime_database_v2

    root.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(root / "bioxp_runtime.db", isolation_level=None)
    conn.row_factory = sqlite3.Row
    conn.create_function("authority_write_allowed", 0, lambda: 1)
    conn.create_function("sha256_utf8", 1, lambda v: hashlib.sha256(str(v).encode()).hexdigest(), deterministic=True)
    conn.create_function("sha256_blob", 1, lambda v: hashlib.sha256(bytes(v)).hexdigest(), deterministic=True)
    conn.create_function("canonical_json", 1, lambda v: json.dumps(json.loads(str(v)), sort_keys=True, separators=(",", ":"), allow_nan=False), deterministic=True)
    try:
        migrate_runtime_database_v2(conn, root)
    finally:
        conn.close()


@pytest.mark.parametrize(
    "route_case",
    [
        "switch_audit",
        "reference_status",
        "reference_mark_referenced",
        "reference_mark_desynced",
        "liquid_status",
        "liquid_requests",
        "liquid_application_plan",
    ],
)
def test_blocking_routes_do_not_block_concurrent_status_on_event_loop(monkeypatch, route_case):
    entered = threading.Event()
    release = threading.Event()
    loop_thread_id = threading.get_ident()
    worker_thread_ids = []

    def block_and_return(payload):
        worker_thread_ids.append(threading.get_ident())
        entered.set()
        assert release.wait(timeout=1.0)
        return payload

    # Keep GET /status ultra-fast in-memory so it verifies the event loop is responsive.
    monkeypatch.setattr(api, "_status_payload", lambda: {"status": "ok"})
    monkeypatch.setattr(api, "_tester", SimpleNamespace())
    monkeypatch.setattr(api, "_tester_quarantine", None)

    if route_case == "switch_audit":
        monkeypatch.setattr(api, "run_switch_audit", lambda *a, **k: block_and_return({"ok": True, "axes": ["x"]}))
        method, url, kwargs = "POST", "/oem/switch_audit", {"json": {"axes": ["x"], "mode": "status"}}
    elif route_case == "reference_status":
        monkeypatch.setattr(api._reference_state_store, "snapshot", lambda axes: block_and_return({"ok": True, "rows": {}}))
        method, url, kwargs = "GET", "/motion/reference/status?axes=x", {}
    elif route_case == "reference_mark_referenced":
        monkeypatch.setattr(api._reference_state_store, "mark_referenced", lambda cmd: block_and_return({"ok": True, "axis": "x"}))
        method, url, kwargs = "POST", "/motion/reference/mark_referenced", {"json": {"axis": "x", "position_steps": 0, "source": "test"}}
    elif route_case == "reference_mark_desynced":
        monkeypatch.setattr(api._reference_state_store, "mark_desynced", lambda cmd: block_and_return({"ok": True, "axis": "x"}))
        method, url, kwargs = "POST", "/motion/reference/mark_desynced", {"json": {"axis": "x", "reason": "test", "source": "test"}}
    elif route_case == "liquid_status":
        fake_receipts = SimpleNamespace(latest=lambda: block_and_return({"receipt_id": "r1"}))
        monkeypatch.setattr(api, "_pipette_receipts", fake_receipts)
        monkeypatch.setattr(api._pipette_application, "status", lambda: {"ok": True})
        method, url, kwargs = "GET", "/liquid/status", {}
    elif route_case == "liquid_requests":
        fake_receipts = SimpleNamespace(
            lookup_direct_request=lambda **kw: block_and_return({"lookup_state": "unknown", "reason": "not_found"})
        )
        monkeypatch.setattr(api, "_pipette_receipts", fake_receipts)
        method, url, kwargs = "GET", "/liquid/requests?request_kind=application_plan", {"headers": {"Idempotency-Key": "idem-lookup-123456"}}
    else:
        fake_receipts = SimpleNamespace(
            record=lambda **kw: block_and_return({"result": {"ok": True}, "receipt_id": "rec-1", "truth": {}})
        )
        monkeypatch.setattr(api, "_pipette_receipts", fake_receipts)
        monkeypatch.setattr(api._pipette_application, "plan_move_to_waste", lambda: {"ok": True, "operation": "move_to_waste"})
        method, url, kwargs = "POST", "/liquid/application/plan", {
            "json": {"operation": "move_to_waste"},
            "headers": {"Idempotency-Key": "idem-plan-123456"},
        }

    async def run():
        transport = httpx.ASGITransport(app=api.app)
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            blocked_task = asyncio.create_task(client.request(method, url, **kwargs))
            # Wait for the blocking collaborator to enter.
            for _ in range(200):
                if entered.is_set():
                    break
                await asyncio.sleep(0.005)
            assert entered.is_set(), f"{route_case} never entered collaborator"
            # Unrelated status request on the SAME event loop MUST complete while collaborator is held!
            status_resp = await asyncio.wait_for(client.get("/status"), timeout=1.0)
            assert status_resp.status_code == 200
            release.set()
            resp = await asyncio.wait_for(blocked_task, timeout=2.0)
            assert resp.status_code == 200, resp.text

    asyncio.run(run())
    assert worker_thread_ids and worker_thread_ids[0] != loop_thread_id


def test_switch_audit_runs_under_tester_lock_without_motion_precheck(monkeypatch):
    precheck_called = []
    monkeypatch.setattr(api, "_tester", SimpleNamespace())
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "run_switch_audit", lambda hw, **kw: {"ok": True, "axes": kw["axes"]})
    monkeypatch.setattr(
        operator_controls,
        "current_operator_dispatch_context",
        lambda: {"motion_snapshot_precheck": lambda: precheck_called.append(True)},
    )
    res = asyncio.run(api.oem_switch_audit(api.OemSwitchAuditRequest(axes=["x"], mode="status")))
    assert res["ok"] is True
    assert precheck_called == []


def test_real_sqlite_stores_in_worker_threads_and_pipette_read_locked(tmp_path, monkeypatch):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot

    root = tmp_path / "runtime"
    bind_serial206_oem_snapshot(monkeypatch)
    monkeypatch.setattr(
        "bioxp.pipette.receipts.current_release_identity",
        lambda: {"verified": True, "release_id": "test-release-async", "source": {"manifest_sha256": "a" * 64, "aggregate_sha256": "b" * 64}},
    )
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(root))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(root))
    monkeypatch.setattr(api, "_status_payload", lambda: {"status": "ok"})
    _init_runtime_db(root)

    ref_store = ReferenceStateStore(state_path=root / "reference_state.json")
    pipette_store = PipetteReceiptStore(root=root)
    monkeypatch.setattr(api, "_reference_state_store", ref_store)
    monkeypatch.setattr(api, "_pipette_receipts", pipette_store)

    # Prove PipetteReceiptStore.read() acquires self._lock (blocks while writer holds lock).
    read_entered = threading.Event()
    read_done = threading.Event()
    with pipette_store._lock:
        def reader():
            read_entered.set()
            pipette_store.read(5)
            read_done.set()

        rt = threading.Thread(target=reader, daemon=True)
        rt.start()
        assert read_entered.wait(timeout=2.0)
        time.sleep(0.05)
        assert not read_done.is_set(), "PipetteReceiptStore.read() ran without acquiring self._lock!"
    assert read_done.wait(timeout=2.0)

    # Exercise actual SQLite stores through the real async endpoints in worker threads,
    # including same-key replay ensuring no duplicate write.
    async def run():
        transport = httpx.ASGITransport(app=api.app)
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            r1 = await client.post(
                "/motion/reference/mark_referenced",
                json={"axis": "x", "position_steps": 100, "source": "worker-test"},
            )
            assert r1.status_code == 200 and r1.json()["ok"] is True
            r2 = await client.get("/motion/reference/status?axes=x")
            assert r2.status_code == 200 and r2.json()["rows"]["x"]["state"] == "referenced"
            r3 = await client.post(
                "/motion/reference/mark_desynced",
                json={"axis": "x", "reason": "worker-desync", "source": "worker-test"},
            )
            assert r3.status_code == 200 and r3.json()["ok"] is True
            r4 = await client.post(
                "/liquid/application/plan",
                json={"operation": "move_to_waste"},
                headers={"Idempotency-Key": "idem-real-plan-0001"},
            )
            assert r4.status_code == 200, r4.text
            rid = r4.json()["receipt_id"]
            # Same-key retry must replay existing receipt rather than writing a duplicate!
            r4_retry = await client.post(
                "/liquid/application/plan",
                json={"operation": "move_to_waste"},
                headers={"Idempotency-Key": "idem-real-plan-0001"},
            )
            assert r4_retry.status_code == 200 and r4_retry.json()["receipt_id"] == rid
            r5 = await client.get("/liquid/status")
            assert r5.status_code == 200 and r5.json()["latest_receipt"]["receipt_id"] == rid
            r6 = await client.get(
                "/liquid/requests?request_kind=application_plan",
                headers={"Idempotency-Key": "idem-real-plan-0001"},
            )
            assert r6.status_code == 200

    asyncio.run(run())
    assert len(pipette_store.read(10)) == 1
    ref_store.close()


def test_operator_v2_interrupt_dispatcher_reports_truthful_stop_boundaries(tmp_path, monkeypatch):
    """Exercise Stop through /operator/v2/actions/oem.z.stop with fake USB endpoints."""
    from fastapi.testclient import TestClient
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot

    root = tmp_path / "op_stop_root"
    bind_serial206_oem_snapshot(monkeypatch)
    endpoints = HardwareEndpoints()
    tester = _build_tester_with_endpoints(endpoints)
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_INTERRUPT_RECONCILIATION_DEADLINE_S", 5.0)

    reconcile_entered = threading.Event()
    release_reconcile = threading.Event()
    reconcile_done = threading.Event()

    class FakeZProvider:
        def execute_z_intent(self, *args, **kwargs):
            raise AssertionError("unexpected non-deferred execute_z_intent")

        def execute_z_stop_interrupt(self, *, inputs, expected_generation, idempotency_key, defer_reconciliation=False):
            res = tester.motor_oem_stop_exact(tester.BOARD_HEAD, 2)
            assert res["ok"] is True

            def reconcile():
                reconcile_entered.set()
                release_reconcile.wait(timeout=5.0)
                reconcile_done.set()
                return {"ok": True, "reconciled": True, "physical_effect_verified": False}

            reconcile.delivery_result = {
                "ok": True,
                "controller_stop_acknowledged": True,
                "axis_delivery_lease_released": True,
                "physical_effect_verified": False,
            }
            return reconcile

    monkeypatch.setattr(api, "_require_serial206_oem_initialization_provider", lambda *a, **k: FakeZProvider())

    async def fast_z_stop():
        return await api._run_safety_interrupt_blocking(
            "serial-206 Z stop",
            lambda _t: api._execute_provider_z_intent("stop", {"timeout_s": 3.0}, defer_reconciliation=True),
            timeout_s=0.15,
            delivery_only_lease=True,
            surface="z-dispatcher-test",
        )

    test_app, _ = make_app(root, monkeypatch, z_stop_route=fast_z_stop)
    try:
        with TestClient(test_app) as client:
            # Boundary 1: stall an ordinary transaction before Stop's first write.
            blocker_held = threading.Event()
            release_blocker = threading.Event()

            def hold_tx():
                with tester.novo_router.transaction_lock:
                    blocker_held.set()
                    release_blocker.wait(timeout=5.0)

            blocker = threading.Thread(target=hold_tx, daemon=True)
            blocker.start()
            assert blocker_held.wait(timeout=2.0)
            try:
                r_pre = client.post(
                    "/operator/v2/actions/oem.z.stop",
                    json={
                        "schema_version": "bioxp.operator_interrupt_request.v1",
                        "idempotency_key": "op-z-stop-prewrite-001",
                        "reason": "pre-write stall boundary",
                        "observed_ownership_generation": 7,
                        "observed_board_epoch_by_board": {},
                    },
                )
                body_pre = r_pre.json()
                ev_pre = body_pre.get("interrupt_evidence") or {}
                assert ev_pre.get("physical_delivery_started") is False
                assert ev_pre.get("wire_write_observed") is False
                assert ev_pre.get("controller_ack_observed") is False
                assert ev_pre.get("connection_lease_held") is True
                assert ev_pre.get("connection_transition_blocked_until_worker_exit") is True
                assert endpoints.stop_writes == 0
                rec_pre = client.get(f"/operator/v2/actions/receipts/{body_pre['command_id']}?detail=true")
                assert rec_pre.status_code == 200, rec_pre.text
                rec_pre_ev = rec_pre.json().get("interrupt_evidence") or {}
                assert rec_pre_ev.get("wire_write_observed") is False
                assert rec_pre_ev.get("connection_lease_held") is True
            finally:
                release_blocker.set()
                blocker.join(timeout=5.0)
                # Wait briefly for the shielded pre-write worker to finish & release its lease.
                deadline = time.monotonic() + 3.0
                while endpoints.stop_writes < 2 and time.monotonic() < deadline:
                    time.sleep(0.02)
                release_reconcile.set()
                reconcile_done.wait(timeout=3.0)
                reconcile_entered.clear()
                release_reconcile.clear()
                reconcile_done.clear()

            # Boundary 2: delay reply after wire write.
            endpoints.stop_writes = 0
            setattr(endpoints, "stop_statuses", iter([None, None, None, None]))
            r_post = client.post(
                "/operator/v2/actions/oem.z.stop",
                json={
                    "schema_version": "bioxp.operator_interrupt_request.v1",
                    "idempotency_key": "op-z-stop-postwrite-001",
                    "reason": "post-write reply delay boundary",
                    "observed_ownership_generation": 7,
                    "observed_board_epoch_by_board": {},
                },
            )
            body_post = r_post.json()
            ev_post = body_post.get("interrupt_evidence") or {}
            assert ev_post.get("physical_delivery_started") is True
            assert ev_post.get("wire_write_observed") is True
            assert ev_post.get("controller_ack_observed") is False
            assert ev_post.get("connection_lease_held") is True
            assert ev_post.get("connection_transition_blocked_until_worker_exit") is True
            assert endpoints.stop_writes >= 1
            rec_post = client.get(f"/operator/v2/actions/receipts/{body_post['command_id']}?detail=true")
            assert rec_post.status_code == 200, rec_post.text
            rec_post_ev = rec_post.json().get("interrupt_evidence") or {}
            assert rec_post_ev.get("wire_write_observed") is True
            assert rec_post_ev.get("controller_ack_observed") is False

            # Tear down the stalled post-write router & build a fresh endpoint/tester for Boundary 3.
            tester.novo_router.shutdown(join_timeout_s=2.0)
            endpoints3 = HardwareEndpoints()
            tester3 = _build_tester_with_endpoints(endpoints3)
            monkeypatch.setattr(api, "_tester", tester3)
            # Wait for the old post-write interrupt lane to release after router shutdown.
            time.sleep(0.1)
            tester = tester3
            endpoints = endpoints3

            # Boundary 3: hold reconciliation after lease release.
            resp = client.post(
                "/operator/v2/actions/oem.z.stop",
                json={
                    "schema_version": "bioxp.operator_interrupt_request.v1",
                    "idempotency_key": "op-z-stop-reconcile-001",
                    "reason": "offline truthful stop boundary test",
                    "observed_ownership_generation": 7,
                    "observed_board_epoch_by_board": {},
                },
            )
            assert resp.status_code == 200, resp.text
            body = resp.json()
            assert body.get("reconciliation_pending") is True
            ev = body.get("interrupt_evidence") or {}
            assert ev.get("delivered") is True
            assert ev.get("controller_stop_acknowledged") is True
            assert ev.get("wire_write_observed") is True
            assert ev.get("controller_ack_observed") is True
            assert ev.get("reconciliation_pending") is True
            assert ev.get("connection_lease_held") is False
            assert ev.get("connection_transition_blocked_until_worker_exit") is False
            assert body.get("status") != "failed"
            assert endpoints.stop_writes == 2
            assert reconcile_entered.wait(timeout=2.0)
            rec_rec = client.get(f"/operator/v2/actions/receipts/{body['command_id']}?detail=true")
            assert rec_rec.status_code == 200, rec_rec.text
            assert rec_rec.json().get("reconciliation_pending") is True
    finally:
        release_reconcile.set()
        reconcile_done.wait(timeout=3.0)
        test_app.state.operator_command_plane.stop()
        tester.novo_router.shutdown(join_timeout_s=2.0)


def test_timed_out_waiter_cannot_trigger_duplicate_liquid_plan_write(tmp_path, monkeypatch):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot

    root = tmp_path / "plan_timeout_root"
    bind_serial206_oem_snapshot(monkeypatch)
    monkeypatch.setattr(
        "bioxp.pipette.receipts.current_release_identity",
        lambda: {"verified": True, "release_id": "test-release-async", "source": {"manifest_sha256": "a" * 64, "aggregate_sha256": "b" * 64}},
    )
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(root))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(root))
    _init_runtime_db(root)
    pipette_store = PipetteReceiptStore(root=root)
    monkeypatch.setattr(api, "_pipette_receipts", pipette_store)

    entered = threading.Event()
    release = threading.Event()
    orig_plan = api._pipette_application.plan_move_to_waste

    def slow_plan():
        entered.set()
        assert release.wait(timeout=5.0)
        return orig_plan()

    monkeypatch.setattr(api._pipette_application, "plan_move_to_waste", slow_plan)

    async def run():
        transport = httpx.ASGITransport(app=api.app)
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            first = asyncio.create_task(
                client.post(
                    "/liquid/application/plan",
                    json={"operation": "move_to_waste"},
                    headers={"Idempotency-Key": "idem-timeout-nodup-001"},
                )
            )
            for _ in range(200):
                if entered.is_set():
                    break
                await asyncio.sleep(0.005)
            assert entered.is_set()
            # Waiter gives up before first completes; first worker still finishes once.
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(asyncio.shield(first), timeout=0.05)
            release.set()
            r1 = await asyncio.wait_for(first, timeout=2.0)
            assert r1.status_code == 200
            # Caller retries with same Idempotency-Key -> replays same receipt, no duplicate row!
            r2 = await client.post(
                "/liquid/application/plan",
                json={"operation": "move_to_waste"},
                headers={"Idempotency-Key": "idem-timeout-nodup-001"},
            )
            assert r2.status_code == 200
            assert r2.json()["receipt_id"] == r1.json()["receipt_id"]

    asyncio.run(run())
    assert len(pipette_store.read(10)) == 1
