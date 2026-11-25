"""Offline regression tests for handler repair 3 (lifespan cleanup & restart)."""
from __future__ import annotations

import asyncio
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from bioxp import api, operator_controls, runtime_audit_store
import src.bioxp.runtime_audit_store as other_runtime
from bioxp.oem_runtime_store import OEMRuntimeStore
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


def _prep_lifespan_env(tmp_path, monkeypatch, *, verified_release: bool):
    root = tmp_path / "runtime"
    root.mkdir(parents=True, exist_ok=True)
    bind_serial206_oem_snapshot(monkeypatch)
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(root))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(root))
    monkeypatch.setattr(runtime_audit_store, "CANONICAL_RUNTIME_ROOT", root)
    monkeypatch.setattr(other_runtime, "CANONICAL_RUNTIME_ROOT", root)
    monkeypatch.setattr(api, "_pipette_receipts", None)
    monkeypatch.setattr(api, "_operator_control_plane_installed", False)
    monkeypatch.setattr(api, "_operator_reports_installed", False)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "_startup_error", None)
    monkeypatch.setattr(api, "_reference_state_store", api.ReferenceStateStore(state_path=root / "reference_state.json"))

    release_identity = {
        "verified": bool(verified_release),
        "release_id": "test-release-lifespan",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64},
    }
    monkeypatch.setattr(api, "configure_release_identity", lambda: dict(release_identity))
    monkeypatch.setattr(operator_controls, "current_release_identity", lambda: dict(release_identity))
    monkeypatch.setattr(operator_controls, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True, "evidence_lock_sha256": "3" * 64,
    })
    monkeypatch.setattr(operator_controls, "current_registry_sha256", lambda: "4" * 64)
    if verified_release:
        monkeypatch.setattr(
            api,
            "record_runtime_release_start",
            lambda rt, ident, timeout_s=45.0: {"receipt_id": "rel-1", "release_identity": dict(ident)},
        )
        monkeypatch.setattr(
            api,
            "publish_runtime_release_receipt",
            lambda receipt: dict(receipt["release_identity"]),
        )

    events = []

    class FakeCalStore:
        def __init__(self):
            self.closed = 0

        def close(self):
            self.closed += 1
            events.append("calibration_store_close")

    cal_stores = []

    def fake_configure_cal(rt):
        st = FakeCalStore()
        cal_stores.append(st)
        snap = SimpleNamespace(startup_mode="normal", operation_parameters={"CheckCamera": False})
        return snap, st, SimpleNamespace()

    monkeypatch.setattr(api, "_configure_machine_calibration", fake_configure_cal)

    class FakeUsbTester:
        BOARD_THERMAL = 8
        BOARD_CHILLER = 10

        def __init__(self, alt=1):
            self.disconnected = 0

        def _oem_ensure_fan_service(self, board):
            return {"board": int(board), "started": True}

        def _disconnect(self):
            self.disconnected += 1
            events.append("usb_disconnect")
            return {"ok": True, "release_interface_ok": True, "dispose_resources_ok": True}

    monkeypatch.setattr(api, "BioXpTester", FakeUsbTester)
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda **kwargs: SimpleNamespace(close=lambda: None))

    async def idle_recovery():
        await asyncio.sleep(3600)

    monkeypatch.setattr(api, "_reference_recovery_runner", idle_recovery)
    return root, events, cal_stores


@pytest.mark.parametrize("verified_release", [False, True])
def test_first_lifespan_shutdown_drains_direct_request_and_closes_all_operator_resources(tmp_path, monkeypatch, verified_release):
    _root, events, cal_stores = _prep_lifespan_env(tmp_path, monkeypatch, verified_release=verified_release)

    test_app = FastAPI(lifespan=api.lifespan)
    test_app.middleware("http")(api.require_durable_release_start_before_readiness)
    monkeypatch.setattr(api, "app", test_app)
    original_lifespan = test_app.router.lifespan_context

    direct_started = threading.Event()
    allow_direct_finish = threading.Event()

    def slow_action():
        direct_started.set()
        assert allow_direct_finish.wait(timeout=5.0)
        events.append("direct_request_finished")
        return {"ok": True}

    test_app.add_api_route("/test/slow_direct", lambda: slow_action(), methods=["POST"])

    async def run_scenario():
        async with api.lifespan(test_app):
            if verified_release:
                for _ in range(200):
                    if getattr(test_app.state, "operator_control_plane_lifecycle", None) is not None:
                        break
                    await asyncio.sleep(0.01)
            # operator_controls must NOT install its own custom lifespan wrapper.
            assert "install_operator_control_plane" not in getattr(test_app.router.lifespan_context, "__qualname__", "")
            lifecycle = test_app.state.operator_control_plane_lifecycle
            assert lifecycle is not None

            # Register a retained in-flight direct request that needs the tester still connected.
            in_flight_entered = asyncio.Event()

            async def retained_work():
                in_flight_entered.set()
                await asyncio.to_thread(slow_action)

            task = asyncio.create_task(retained_work())
            await in_flight_entered.wait()
            assert direct_started.wait(timeout=2.0)
            # Inject into direct_requests via the lifecycle's exposed table to test drain ordering.
            lifecycle.direct_requests["k-inflight"] = ({}, asyncio.get_running_loop().create_future(), task)

            def release_shortly():
                time.sleep(0.08)
                allow_direct_finish.set()

            threading.Thread(target=release_shortly, daemon=True).start()

        # On FIRST exit, all operator resources must already be closed!
        assert test_app.state.operator_poll_cache._closed is True
        assert test_app.state.operator_admission_state_reader._closed is True
        assert test_app.state.operator_preview_state_reader._closed is True
        assert test_app.state.operator_invoke_state_reader._closed is True
        assert test_app.state.operator_deck_display_reader._closed is True
        assert test_app.state.operator_history_reader.connection is None
        assert test_app.state.operator_command_plane.store._thread is None or not test_app.state.operator_command_plane.store._thread.is_alive()

    asyncio.run(run_scenario())

    # In-flight direct request must finish BEFORE USB disconnect and calibration store close.
    assert "direct_request_finished" in events
    assert "usb_disconnect" in events
    assert "calibration_store_close" in events
    assert events.index("direct_request_finished") < events.index("usb_disconnect")
    assert events.index("direct_request_finished") < events.index("calibration_store_close")
    assert cal_stores[0].closed == 1


@pytest.mark.parametrize("verified_release", [False, True])
def test_second_lifespan_cycle_rearms_readers_and_dispatcher_without_duplicating_routes(tmp_path, monkeypatch, verified_release):
    _root, events, cal_stores = _prep_lifespan_env(tmp_path, monkeypatch, verified_release=verified_release)

    test_app = FastAPI(lifespan=api.lifespan)
    test_app.middleware("http")(api.require_durable_release_start_before_readiness)
    dispatched = threading.Event()
    test_app.add_api_route(
        "/motion/oem/x/restore_original_speed",
        lambda: (dispatched.set(), {"ok": True, "cycle": 2})[1],
        methods=["POST"],
    )
    monkeypatch.setattr(api, "app", test_app)
    original_lifespan = test_app.router.lifespan_context

    with TestClient(test_app) as client1:
        if verified_release:
            # Wait for verified release-start task to complete installation.
            r = client1.get("/operator/v2/control-catalog?view=metadata")
            assert r.status_code == 200, r.text
        route_count_1 = len(test_app.routes)
        lifespan_after_cycle_1 = test_app.router.lifespan_context
        assert "install_operator_control_plane" not in getattr(lifespan_after_cycle_1, "__qualname__", "")
        assert test_app.state.operator_poll_cache._closed is False
        assert test_app.state.operator_command_plane.store._thread.is_alive()

    # First exit closed everything cleanly.
    assert test_app.state.operator_poll_cache._closed is True
    assert test_app.state.operator_admission_state_reader._closed is True
    assert test_app.state.operator_deck_display_reader._closed is True
    assert test_app.state.operator_history_reader.connection is None
    assert not test_app.state.operator_command_plane.store._thread.is_alive()
    assert cal_stores[0].closed == 1

    # SECOND lifespan on the SAME app instance!
    with TestClient(test_app) as client2:
        r = client2.get("/operator/v2/control-catalog?view=metadata")
        assert r.status_code == 200, r.text
        assert len(test_app.routes) == route_count_1
        assert test_app.router.lifespan_context is lifespan_after_cycle_1

        # Real reader operations work after restart!
        assert test_app.state.operator_poll_cache._closed is False
        assert test_app.state.operator_admission_state_reader._closed is False
        assert test_app.state.operator_deck_display_reader._closed is False
        assert test_app.state.operator_history_reader.connection is not None
        assert test_app.state.operator_history_reader.get_command("missing-id") is None
        hist = client2.get("/operator/actions/history")
        assert hist.status_code == 200, hist.text

        # Real dispatcher is alive with _stop and _priority_fence cleared!
        store = test_app.state.operator_command_plane.store
        assert store._stop.is_set() is False
        assert store._priority_fence.is_set() is False
        assert store._thread is not None and store._thread.is_alive()

        # Perform a real dispatcher operation on the restarted dispatcher thread.
        ready_state = {
            "ownership_generation": 0,
            "board_epochs": {},
            "ownership": {"transport": "owned", "usb": "service", "router": "running"},
            "serial206_initialization_provider": {"bound": True, "initialize_motors_live_available": True},
            "maintenance": {"motion_blocked": False, "recovery_required": False},
        }
        test_app.state.operator_command_plane.machine_state_provider = lambda: dict(ready_state)
        admitted = store.admit_command(
            {
                "schema_version": "bioxp.operator_action_request.v2",
                "action_id": "oem.x.restore_original_speed",
                "idempotency_key": f"cycle2-dispatch-{verified_release}",
                "expected_ownership_generation": 0,
                "expected_board_epoch_by_board": {},
                "inputs": {},
            },
            state=ready_state,
        )
        assert dispatched.wait(timeout=3.0), "restarted dispatcher did not execute queued command"
        cid = admitted["command_id"]
        terminal_receipt = None
        for _ in range(200):
            rc = client2.get(f"/operator/v2/actions/receipts/{cid}?detail=true")
            if rc.status_code == 200 and rc.json().get("terminal") is True:
                terminal_receipt = rc.json()
                break
            time.sleep(0.01)
        assert terminal_receipt is not None and terminal_receipt["status"] == "completed", terminal_receipt

        # Verify state reader actually executes on its reopened executor.
        state_snap = asyncio.run(test_app.state.operator_deck_display_reader.read())
        assert isinstance(state_snap, dict)

    assert test_app.state.operator_poll_cache._closed is True
    assert test_app.state.operator_history_reader.connection is None
    assert not test_app.state.operator_command_plane.store._thread.is_alive()
    assert cal_stores[1].closed == 1
