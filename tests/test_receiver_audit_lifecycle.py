"""Offline ASGI lifespan/HTTP, real router and governed disposable SQLite."""
import asyncio
import json
import queue
import sqlite3
import threading
from types import SimpleNamespace

import httpx
import pytest
from fastapi import FastAPI

import src.bioxp.api as api
import src.bioxp.receiver_audit_buffer as buffers
from src.bioxp.novo_router import NovoRouter
from src.bioxp.novo_usb_can import novo_decode
from src.bioxp.runtime_audit_store import RuntimeAuditDatabase
from src.bioxp.usb_driver import BioXpTester
import src.bioxp.usb_driver as usb_driver


async def until(predicate):
    async with asyncio.timeout(5):
        while not predicate():
            await asyncio.sleep(.001)


@pytest.fixture
def application(monkeypatch, tmp_path):
    monkeypatch.setattr(api, "configure_release_identity", lambda: {"verified": False})
    monkeypatch.setattr(api, "runtime_state_root", lambda: tmp_path)
    monkeypatch.setattr(api, "reconcile_operator_report_exports", lambda *_: {})
    monkeypatch.setattr(api, "_pipette_receipts", SimpleNamespace(
        root=tmp_path, attest_first_install_absence=lambda: None,
        migrate_legacy_jsonl=lambda: {}, reconcile_nonterminal_claims=lambda: {}))
    def hardware_disabled(**_):
        raise RuntimeError("offline hardware boundary")
    monkeypatch.setattr(api, "configure_oem_machine_snapshot_from_env", hardware_disabled)
    monkeypatch.setattr(api, "_ownership_changed", lambda **_: None)
    monkeypatch.setattr(api, "_publish_quarantined_owner", lambda **_: None)
    async def no_camera(**_):
        pass
    monkeypatch.setattr(api, "_stop_owned_camera_session", no_camera)
    for key in ("_tester", "_tester_quarantine", "_pipette_transport", "_startup_error", "_receiver_audit_shutdown"):
        monkeypatch.setattr(api, key, None)
    monkeypatch.setattr(buffers, "_OWNER_BUFFER", None)
    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda *_: None)
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda *_: None)
    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_operator_control_plane_installed", True)
    monkeypatch.setattr(api, "_operator_reports_installed", True)
    app = FastAPI(lifespan=api.lifespan)
    app.add_api_route("/status", api.get_status)
    return app


class Endpoint:
    def read(self, size, timeout):
        try:
            return queue.Queue().get(timeout=.005)
        except queue.Empty:
            raise TimeoutError()


def owner_for(audit):
    owner = BioXpTester.__new__(BioXpTester)
    owner.dev = SimpleNamespace(label="offline fake device")
    owner.ep_in = owner.ep_out = Endpoint()
    owner._receiver_audit = audit
    owner.novo_router = NovoRouter(ep_in=owner.ep_in, ep_out=owner.ep_out,
                                   decode=novo_decode, audit_buffer=audit)
    owner.novo_router.start()
    return owner


@pytest.mark.parametrize("outcome", ["drain", "timeout", "failure", "released", "usb_failure", "command_failure"])
def test_asgi_shutdown_and_read_only_health(application, monkeypatch, tmp_path, caplog, outcome):
    app = application
    entered, release, joined = threading.Event(), threading.Event(), threading.Event()
    main_thread = threading.get_ident()
    monkeypatch.setattr(api, "_RECEIVER_AUDIT_DRAIN_TIMEOUT_S", .5 if outcome == "timeout" else 1.0)

    class PausedDatabase(RuntimeAuditDatabase):
        def record_event(self, **kwargs):
            if kwargs["event_kind"] == "test_ingress":
                entered.set()
                assert release.wait(5)
                if outcome == "failure":
                    raise OSError("private/path injected disk error")
            return super().record_event(**kwargs)

    async def exercise():
        receive, send = asyncio.Queue(), asyncio.Queue()
        task = asyncio.create_task(app({"type": "lifespan", "asgi": {"version": "3.0"}, "state": {}},
                                       receive.get, send.put))
        await receive.put({"type": "lifespan.startup"})
        assert (await asyncio.wait_for(send.get(), 5))["type"] == "lifespan.startup.complete"
        audit = buffers.ReceiverAuditBuffer(root=tmp_path, database_factory=PausedDatabase)
        monkeypatch.setattr(buffers, "_OWNER_BUFFER", audit)
        owner = owner_for(audit)
        monkeypatch.setattr(api, "_tester", owner)
        router = owner.novo_router
        original_shutdown = router.shutdown
        original_close = audit.close
        def close(timeout_s=0.0):
            if timeout_s > 0:
                assert threading.get_ident() != main_thread
                assert owner.novo_router is None
                assert not api._tester_lock.locked()
                assert not api._tester_transition_lock.locked()
                assert owner._transport_guard().acquire(blocking=False)
                owner._transport_guard().release()
                joined.set()
            return original_close(timeout_s)
        monkeypatch.setattr(audit, "close", close)
        try:
            assert audit.offer("test_ingress", {"raw": "evidence"})
            await until(entered.is_set)
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://offline") as client:
                before = audit.status()
                response = await client.get("/status")
                assert response.status_code == 200
                health = response.json()["receiver_audit"]
                assert health["healthy"] and not health["durable_ownership_claimed"]
                assert health["buffers"][0]["buffered_records"] > 0
                assert health["buffers"][0]["accepted_volatile"] > health["buffers"][0]["committed_records"]
                assert audit.status() == before, "health must not query USB or mutate the writer"
                if outcome == "command_failure":
                    def fail_stop():
                        raise RuntimeError("command workers still active")
                    app.state.operator_command_plane = SimpleNamespace(stop=fail_stop)
                if outcome == "released":
                    assert owner._disconnect()["ok"]
                    api._tester = None
                if outcome == "usb_failure":
                    def fail_shutdown():
                        raise RuntimeError("injected reader shutdown failure")
                    monkeypatch.setattr(router, "shutdown", fail_shutdown)
                await receive.put({"type": "lifespan.shutdown"})
                if outcome in ("usb_failure", "command_failure"):
                    await until(task.done)
                    assert not joined.is_set()
                    assert not audit.status()["closing"]
                else:
                    await until(joined.is_set)
                    # An HTTP request really runs on the same loop while join is blocked.
                    response = await asyncio.wait_for(client.get("/status"), .5)
                    assert response.json()["receiver_audit"]["shutdown"]["state"] == "draining"
                    assert not task.done()
                    if outcome != "timeout":
                        release.set()
                message = await asyncio.wait_for(send.get(), 2)
                if outcome == "command_failure":
                    assert message["type"] == "lifespan.shutdown.failed"
                    with pytest.raises(RuntimeError, match="command workers remain active"):
                        await task
                    assert app.state.receiver_audit_shutdown["reason"] == "command_workers_active"
                    assert owner.novo_router is router
                    return
                assert message["type"] == "lifespan.shutdown.complete", message
                await task
                result = app.state.receiver_audit_shutdown
                assert result["state"] == ("skipped" if outcome == "usb_failure" else
                    "incomplete" if outcome in ("timeout", "failure") else "complete")
                if outcome == "timeout":
                    assert result["buffers"][0]["timed_out"]
                    assert audit.alive and audit.status()["buffered_records"] > 0
                if outcome == "failure":
                    assert result["buffers"][0]["writer_failed"]
                    assert not result["buffers"][0]["clean_drain_committed"]
                    response = await client.get("/status")
                    assert response.json()["receiver_audit"]["buffers"][0]["writer_error"] == "writer_failed"
                    assert "private/path" not in response.text
                if outcome != "usb_failure":
                    assert api._tester_quarantine is None
                if outcome in ("timeout", "failure", "usb_failure"):
                    assert "USB teardown status is separate" in caplog.text
                if outcome in ("drain", "released"):
                    with sqlite3.connect(tmp_path / "bioxp_runtime.db") as db:
                        rows = db.execute("SELECT event_kind,event_json FROM runtime_events WHERE event_source=? ORDER BY event_id",
                                          (audit.SOURCE,)).fetchall()
                    assert any(kind == "test_ingress" for kind, _ in rows)
                    assert rows[-1][0] == "closed"
                    assert json.loads(rows[-1][1])["all_accepted_committed"]
        finally:
            release.set()
            # Restore only the injected failure so the real fake reader is joined.
            original_shutdown()
            original_close(2)
            if not task.done():
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
    asyncio.run(exercise())


def test_status_without_writer_does_not_create_one(application, monkeypatch):
    monkeypatch.setattr(api, "_tester", SimpleNamespace(_receiver_audit_setup_error="secret/path"))
    async def exercise():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=application), base_url="http://offline") as client:
            result = (await client.get("/status")).json()["receiver_audit"]
        assert result["available"] is False and result["healthy"] is False
        assert result["setup_error"] == "setup_failed"
        assert buffers.current_receiver_audit_buffer() is None
    asyncio.run(exercise())
