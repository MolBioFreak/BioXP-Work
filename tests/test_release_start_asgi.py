"""Real listener/SQLite release-start ordering, with hardware boundaries disabled."""
import asyncio
import copy
import json
import os
import socket
import sqlite3
import threading
from types import SimpleNamespace

import httpx
import pytest
import uvicorn
from fastapi import FastAPI


@pytest.mark.parametrize("outcome", ["success", "receipt_error", "plane_error", "shutdown"])
def test_release_start_listener_order(monkeypatch, tmp_path, outcome):
    import src.bioxp.api as api
    import src.bioxp.release_identity as release
    import src.bioxp.runtime_audit_store as audit

    # Reserve but DO NOT listen: only Uvicorn may create the measured listener.
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    identity = {
        "verified": True, "release_id": "hardware-free-test",
        "source": {"manifest_sha256": "a" * 64, "aggregate_sha256": "b" * 64},
        "image": {"id": "sha256:" + "c" * 64, "inspection_receipt_sha256": "d" * 64},
        "deployment": {"receipt_id": "test-deployment", "receipt_sha256": "e" * 64},
        "binding": {
            "declared_listener": {"host": "127.0.0.1", "port": port},
            "systemd_invocation_id": "test-invocation", "udocker_path": "/test/udocker",
            "database_root": str(tmp_path),
            **{key: "f" * 64 for key in (
                "udocker_sha256", "udocker_tree_sha256", "unit_sha256",
                "launcher_sha256", "configuration_sha256", "oem_lock_sha256",
            )},
        },
    }
    monkeypatch.setattr(release, "_cached", copy.deepcopy(identity))
    monkeypatch.setattr(api, "configure_release_identity", lambda: copy.deepcopy(identity))
    monkeypatch.setattr(api, "runtime_state_root", lambda: tmp_path)
    monkeypatch.setattr(api, "reconcile_operator_report_exports", lambda *_: {})
    monkeypatch.setattr(api, "_pipette_receipts", SimpleNamespace(
        root=tmp_path, attest_first_install_absence=lambda: None,
        migrate_legacy_jsonl=lambda: {}, reconcile_nonterminal_claims=lambda: {},
    ))
    monkeypatch.setattr(api, "configure_oem_machine_snapshot_from_env",
                        lambda **_: (_ for _ in ()).throw(RuntimeError("hardware disabled")))
    monkeypatch.setattr(api, "_ownership_changed", lambda **_: None)
    teardown_writer_states = []
    async def no_hardware(**_):
        teardown_writer_states.append(finished.is_set())
    monkeypatch.setattr(api, "_stop_owned_camera_session", no_hardware)
    for key in ("_tester", "_tester_quarantine", "_pipette_transport", "_startup_error"):
        monkeypatch.setattr(api, key, None)
    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_operator_control_plane_installed", False)
    monkeypatch.setattr(api, "_operator_reports_installed", True)
    calls = []
    entered = threading.Event()
    committed = threading.Event()
    proceed = threading.Event()
    finished = threading.Event()
    original_record = audit.record_runtime_release_start

    def record(root, value, **_):
        entered.set()
        try:
            result = original_record(root, value, timeout_s=1)
            # Pause AFTER real listener measurement and durable COMMIT.
            committed.set()
            assert proceed.wait(5)
            return result
        finally:
            finished.set()
    monkeypatch.setattr(api, "record_runtime_release_start", record)

    def committed_receipt():
        with sqlite3.connect(tmp_path / "bioxp_runtime.db") as db:
            rows = db.execute("SELECT receipt_json FROM runtime_release_receipts").fetchall()
        assert len(rows) == 1
        receipt = json.loads(rows[0][0])
        assert receipt["observed_listener"]["port"] == port
        assert receipt["observed_listener"]["owner_pid"] == os.getpid()
        return receipt

    def install(app, **_):
        committed_receipt()
        assert app.state.release_start_ready.is_set() is False
        calls.append("install")
        def start():
            committed_receipt()
            assert not app.state.release_start_ready.is_set()
            if outcome == "plane_error":
                raise RuntimeError("injected worker-start failure")
            calls.append("start")
        app.state.operator_receipt_store = SimpleNamespace(converge_startup_state=lambda: calls.append("converge"))
        app.state.operator_command_plane = SimpleNamespace(start=start, stop=lambda: calls.append("stop"))
    monkeypatch.setattr(api, "install_operator_control_plane", install)

    if outcome == "receipt_error":
        # Real INSERT failure, not a fabricated receipt-return stub.
        original_migrate = api.migrate_runtime_database_v2
        def migrate(conn, root):
            original_migrate(conn, root)
            conn.execute("CREATE TRIGGER reject_test_receipt BEFORE INSERT ON runtime_release_receipts BEGIN SELECT RAISE(ABORT, 'injected receipt failure'); END")
        monkeypatch.setattr(api, "migrate_runtime_database_v2", migrate)

    app = FastAPI(lifespan=api.lifespan)
    app.middleware("http")(api.require_durable_release_start_before_readiness)

    async def wait_until(predicate):
        async with asyncio.timeout(5):
            while not predicate():
                await asyncio.sleep(0.01)

    async def exercise():
        server = uvicorn.Server(uvicorn.Config(app, log_level="error", lifespan="on"))
        task = asyncio.create_task(server.serve(sockets=[sock]))
        request = None
        try:
            await wait_until(lambda: server.started)
            assert entered.is_set()
            if outcome != "receipt_error":
                await wait_until(committed.is_set)
            if outcome == "shutdown":
                server.should_exit = True
                await wait_until(lambda: any(
                    t.get_name() == "bioxp-runtime-release-start-receipt" and t.cancelling()
                    for t in asyncio.all_tasks()
                ))
                assert not task.done(), "shutdown must drain the receipt writer"
                assert not finished.is_set()
                proceed.set()
                await asyncio.wait_for(task, 3)
                assert finished.is_set()
                await asyncio.sleep(0.05)
                assert calls == []
                assert app.state.release_start_receipt is None
                return
            async with httpx.AsyncClient(trust_env=False) as client:
                request = asyncio.create_task(client.get(f"http://127.0.0.1:{port}/openapi.json", timeout=5))
                if outcome != "receipt_error":
                    await asyncio.sleep(0.1)
                    assert not request.done(), "readiness escaped before receipt publication/worker start"
                    assert calls == []
                proceed.set()
                response = await request
                if outcome == "success":
                    assert response.status_code == 200, response.text
                    assert calls == ["install", "converge", "start"]
                    assert app.state.release_start_receipt == committed_receipt()
                else:
                    assert response.status_code == 503
                    assert response.json()["reason_code"] == "release_start_failed"
                    assert "start" not in calls
                    if outcome == "receipt_error":
                        assert "injected receipt failure" in app.state.release_start_error
                        assert calls == []
                        with sqlite3.connect(tmp_path / "bioxp_runtime.db") as db:
                            assert db.execute("SELECT count(*) FROM runtime_release_receipts").fetchone()[0] == 0
        finally:
            proceed.set()
            if request is not None and not request.done():
                request.cancel()
                await asyncio.gather(request, return_exceptions=True)
            server.should_exit = True
            await asyncio.wait_for(task, 5)
            sock.close()
    asyncio.run(exercise())
    assert teardown_writer_states == [True]
    if outcome in {"success", "plane_error"}:
        assert calls[-1] == "stop"
