"""Offline retention: real dispatch/worker/router/driver/SQLite, fake wire and authority.

Authority inputs reuse the frozen operator fixture; no live authorization claim.
The small ASGI adapter deliberately drops child provenance, not transport calls.
"""
import asyncio
import threading
import inspect
import json

import pytest
import usb.core
from httpx import ASGITransport, AsyncClient

from bioxp import api
from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.command_exchange_observer import exchange_scope, current_exchange_owner
from test_operator_controls import make_app, action_for
from test_f06_motor_retries import motor_transport


def harness(tmp_path, monkeypatch, script, *, late=False, mode="query", interrupt=False):
    app, calls = make_app(tmp_path, monkeypatch)
    monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    monkeypatch.setattr(api, "_get_tester", lambda: None)  # Hardware configuration only.
    started, release = threading.Event(), threading.Event()
    captured = {}

    def operation():
        with motor_transport(script) as (driver, router, endpoint, waits, owner):
            captured.update(owner=owner, endpoint=endpoint)
            captured.setdefault("owners", []).append(owner)
            write = endpoint.write

            def deferred_write(raw, timeout):
                started.set()
                if late:
                    assert release.wait(3), "test did not release actual worker"
                return write(raw, timeout)

            endpoint.write = deferred_write
            if mode == "generic":
                result = driver.motor_move_absolute(5, 321)
            elif mode == "board":
                driver.oem_no24v_state = lambda: False
                driver._oem_board_state = lambda: {5: True}
                driver._motion_oem_axis_profile = lambda *a, **kw: {"axis_min_steps": 0, "axis_max_steps": 1000}
                result = driver.motor_oem_move_absolute(5, 321, wait_for_stop=False)
            elif mode == "provider_composite":
                from bioxp.oem_serial206_initialization import (
                    Serial206OemInitializationProvider, Serial206ProductionPrimitiveAdapter,
                )
                from bioxp.oem_runtime_store import OEMRuntimeStore
                # Only board presence and authority/configuration are fixtures.
                driver._oem_board_state = lambda: {5: True, 4: True}
                adapter = Serial206ProductionPrimitiveAdapter(
                    driver, None, authority_provider=lambda: None,
                    generation_provider=lambda: 1,
                )
                provider = Serial206OemInitializationProvider(
                    adapter, state_store=OEMRuntimeStore(tmp_path),
                    generation_provider=lambda: 1,
                )
                result = provider.execute_x_intent("enable_xy_current", {
                    "enabled": False, "command_id": owner.command_id,
                    "expected_generation": 1,
                })
                captured["provider"] = provider
            elif mode == "composite":
                from test_f06_motor_retries import configure_no_motion
                configure_no_motion(driver, monkeypatch)
                result = driver.oem_initialize_without_motion_test_case(board_wait={"ok": True}, components=["x"])
            else:
                result = driver.motor_get_axis_param(5, 1)
            captured["result"] = result
            return {"ok": True}  # Deliberate provenance-free child projection.

    async def route(body: dict):
        calls.append(("real-worker", body))
        if interrupt:
            return await api._run_safety_interrupt_blocking("f06-offline", lambda tester: operation(), timeout_s=.03 if late else 2)
        return await api._run_blocking("f06-offline", operation, timeout_s=.03 if late is True else 2)

    selected = next(r for r in app.routes if r.path == "/motion/test/home_axis")
    selected.dependant.call = route
    if late == "cancel":
        # External test-only response bound: real operator wait_for cancels the
        # actual ASGI/API waiter, never the shielded blocking worker.
        invoke = inspect.getclosurevars(app.state.invoke_operator_action_v2).nonlocals["invoke_action"]
        actions = inspect.getclosurevars(invoke).nonlocals["by_id"]
        for action in actions.values():
            if action["action_id"].startswith("route.home_axis_"):
                action["timeout_seconds"] = .03
        captured["actions"] = actions
    return app, calls, captured, started, release


async def submit(client):
    catalog = (await client.get("/operator/control-catalog")).json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    payload = {"expected_generation": catalog["ownership_generation"], "idempotency_key": "f06-retention-proof", "inputs": {"body": {"axis": "x"}}}
    url = f"/operator/actions/{action['action_id']}"
    response = await client.post(url, json=payload)
    assert response.status_code == 200, response.text
    return response.json(), url, payload


@pytest.mark.parametrize("script,expected", [(["success"], ["response"]), (["timeout", "success"], ["timeout", "response"]), (["timeout", "timeout"], ["timeout", "timeout"])])
def test_operator_retains_real_attempts_and_replay(tmp_path, monkeypatch, script, expected):
    app, calls, captured, _, _ = harness(tmp_path, monkeypatch, script)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            receipt, url, payload = await submit(client)
            replay = await client.post(url, json=payload)
            assert replay.json()["command_id"] == receipt["command_id"]
            return receipt

    receipt = asyncio.run(scenario())
    assert len(calls) == 1
    assert len(captured["endpoint"].writes) == len(expected)
    reopened = OperatorReceiptStore(tmp_path)
    for detail in (False, True):
        saved = reopened.by_command(receipt["command_id"], include_evidence=detail)
        rows = saved.get("transport_exchanges", [])
        assert [row["outcome"] for row in rows] == expected
        assert {row["command_id"] for row in rows} == {receipt["command_id"]}
    assert reopened.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 1


@pytest.mark.parametrize("script", [["timeout", "success"], ["timeout", "timeout"], [ValueError("late wire failure")]])
@pytest.mark.parametrize("late", [True, "cancel"])
def test_late_worker_persists_without_terminal_state_change(tmp_path, monkeypatch, script, late):
    app, calls, captured, started, release = harness(tmp_path, monkeypatch, script, late=late)
    store = app.state.operator_receipt_store

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            try:
                receipt, _, _ = await submit(client)
                assert started.is_set()
                assert receipt["status"] == "outcome_unknown"
                before = dict(store.connection.execute("SELECT * FROM operator_commands").fetchone())
            finally:
                release.set()
            workers = [task for task in asyncio.all_tasks() if task.get_name() == "bioxp-tester:f06-offline"]
            assert len(workers) == 1
            results = await asyncio.gather(*workers, return_exceptions=True)
            if isinstance(script[-1], Exception):
                assert results == [script[-1]]
            after = dict(store.connection.execute("SELECT * FROM operator_commands").fetchone())
            evidence_keys = {"transport_exchanges", "transport_retention_errors"}
            assert {k: v for k, v in json.loads(before["receipt_json"]).items() if k not in evidence_keys} == {k: v for k, v in json.loads(after["receipt_json"]).items() if k not in evidence_keys}
            assert {k: v for k, v in before.items() if k != "receipt_json"} == {k: v for k, v in after.items() if k != "receipt_json"}
            return receipt

    receipt = asyncio.run(scenario())
    saved = OperatorReceiptStore(tmp_path).by_command(receipt["command_id"])
    assert len(saved.get("transport_exchanges", [])) == len(script)
    assert {r["command_id"] for r in saved["transport_exchanges"]} == {receipt["command_id"]}
    assert len(calls) == 1


@pytest.mark.parametrize("script,outcomes", [
    (["empty"], ["signaled_empty"]),
    (["signaled_malformed"], ["signaled_empty"]),
    (["error"], ["status_error"]),
    ([usb.core.USBTimeoutError("offline timeout")], ["write_exception"]),
    ([usb.core.USBError("offline pipe")], ["write_exception"]),
    ([ValueError("offline source exception")], ["write_exception"]),
    (["timeout", ValueError("offline retry exception")], ["timeout", "write_exception"]),
])
def test_operator_failures_survive_projection(tmp_path, monkeypatch, script, outcomes):
    app, calls, captured, _, _ = harness(tmp_path, monkeypatch, script)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            return (await submit(client))[0]

    receipt = asyncio.run(scenario())
    saved = OperatorReceiptStore(tmp_path).by_command(receipt["command_id"])
    rows = saved["transport_exchanges"]
    assert [r["outcome"] for r in rows] == outcomes
    assert len(calls) == 1 and len(captured["endpoint"].writes) == len(script)
    if isinstance(script[-1], ValueError):
        assert receipt["status"] == "failed"
        assert str(script[-1]) in receipt["error"]
    else:
        assert captured["result"]["ack"] is None or captured["result"]["ack"]["status"] == 2


@pytest.mark.parametrize("mode,script,commands", [
    ("generic", ["success", "timeout", "timeout", "timeout", "error"], [138, 4, 4, 4, 4]),
    ("board", ["success", "success", "timeout", "timeout", "timeout", "error"], [6, 138, 4, 4, 4, 4]),
    ("composite", ["timeout"] * 8, [5] * 8),
])
def test_actual_multicall_source_keeps_one_operator_root(tmp_path, monkeypatch, mode, script, commands):
    app, calls, captured, _, _ = harness(tmp_path, monkeypatch, script, mode=mode)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            return (await submit(client))[0]

    receipt = asyncio.run(scenario())
    saved = OperatorReceiptStore(tmp_path).by_command(receipt["command_id"])
    rows = saved["transport_exchanges"]
    assert [r["expected_command"] for r in rows] == commands
    assert len(captured["endpoint"].writes) == len(rows)
    assert {r["command_id"] for r in rows} == {receipt["command_id"]}
    assert len(calls) == 1
    assert saved["response"]["body"] == {"ok": True}
    if mode != "composite":
        mvp = [r for r in rows if r["expected_command"] == 4]
        assert len({r["transaction_id"] for r in mvp}) == 2
        assert [r["attempt_ordinal"] for r in mvp] == [1, 2, 1, 2]
        assert captured["result"]["low_level_source_return_code"] == 0
    else:
        assert len({r["transaction_id"] for r in rows}) == 4


def test_late_flush_wins_race_with_stale_final_put(tmp_path, monkeypatch):
    app, calls, captured, started, release = harness(tmp_path, monkeypatch, ["timeout", "success"], late=True)
    store = app.state.operator_receipt_store
    final_entered, permit_final = threading.Event(), threading.Event()
    original = store.put

    def delayed_final(receipt, **kw):
        if receipt["status"] == "outcome_unknown":
            assert not receipt["transport_exchanges"]
            final_entered.set()
            assert permit_final.wait(3)
        return original(receipt, **kw)

    monkeypatch.setattr(store, "put", delayed_final)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            request = asyncio.create_task(submit(client))
            try:
                assert await asyncio.to_thread(final_entered.wait, 2)
                release.set()
                workers = [t for t in asyncio.all_tasks() if t.get_name() == "bioxp-tester:f06-offline"]
                await asyncio.gather(*workers)
                assert len(store.list()[0]["transport_exchanges"]) == 2
            finally:
                release.set()
                permit_final.set()
            return (await request)[0]

    receipt = asyncio.run(scenario())
    captured["owner"].flush()
    captured["owner"].flush()
    saved = OperatorReceiptStore(tmp_path).by_command(receipt["command_id"])
    assert len(saved["transport_exchanges"]) == 2
    assert len(receipt["transport_exchanges"]) == 2
    assert len(calls) == 1


@pytest.mark.parametrize("late", [False, True])
def test_explicit_interrupt_context_and_actual_exit_flush(tmp_path, monkeypatch, late):
    app, calls, captured, _, release = harness(tmp_path, monkeypatch, ["timeout", "success"], interrupt=True, late=late)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            try:
                receipt = (await submit(client))[0]
            finally:
                release.set()
            workers = [t for t in asyncio.all_tasks() if t.get_name() == "bioxp-interrupt:f06-offline"]
            await asyncio.gather(*workers)
            return receipt

    receipt = asyncio.run(scenario())
    saved = OperatorReceiptStore(tmp_path).by_command(receipt["command_id"])
    assert len(saved["transport_exchanges"]) == 2
    assert {r["command_id"] for r in saved["transport_exchanges"]} == {receipt["command_id"]}
    assert len(calls) == 1


def test_sink_failure_is_visible_without_masking_wire_exception(tmp_path, monkeypatch, caplog):
    error = ValueError("source wire exception")
    app, calls, captured, _, _ = harness(tmp_path, monkeypatch, ["timeout", error])
    store = app.state.operator_receipt_store

    def unavailable(*args):
        raise OSError("offline disk failure")

    monkeypatch.setattr(store, "merge_transport_evidence", unavailable)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            return (await submit(client))[0]

    receipt = asyncio.run(scenario())
    assert receipt["status"] == "failed" and str(error) in receipt["error"]
    assert len(captured["endpoint"].writes) == 2 and len(calls) == 1
    assert receipt["transport_retention_errors"] == [{"stage": "sink", "class": "OSError", "message": "offline disk failure"}]
    assert "Transport exchange retention failed" in caplog.text
    assert len(OperatorReceiptStore(tmp_path).by_command(receipt["command_id"])["transport_exchanges"]) == 2


def test_v2_compact_and_detail_expose_retained_attempts(tmp_path, monkeypatch):
    app, _, _, _, _ = harness(tmp_path, monkeypatch, ["timeout", "success"])

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            receipt = (await submit(client))[0]
            for detail in (False, True):
                response = await client.get(f"/operator/v2/actions/receipts/{receipt['command_id']}", params={"detail": str(detail).lower()})
                assert response.status_code == 200
                assert response.json().get("transport_exchanges") == receipt["transport_exchanges"]

    asyncio.run(scenario())



def test_separate_late_dispatch_roots_do_not_cross_bleed(tmp_path, monkeypatch):
    app, calls, captured, _, release = harness(tmp_path, monkeypatch, ["timeout", "success"], late=True)
    store = app.state.operator_receipt_store

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            try:
                first, url, payload = await submit(client)
                second_response = await client.post(url, json={**payload, "idempotency_key": "second-f06-command"})
                assert second_response.status_code == 200
                second = second_response.json()
                assert first["command_id"] != second["command_id"]
                assert first["status"] == second["status"] == "outcome_unknown"
                workers = [t for t in asyncio.all_tasks() if t.get_name() == "bioxp-tester:f06-offline"]
                assert len(workers) == 2
            finally:
                release.set()
            await asyncio.gather(*workers)
            assert current_exchange_owner() is None
            return first, second

    first, second = asyncio.run(scenario())
    saved = [store.by_command(r["command_id"]) for r in (first, second)]
    for row in saved:
        assert len(row["transport_exchanges"]) == 2
        assert {r["command_id"] for r in row["transport_exchanges"]} == {row["command_id"]}
    assert len({r["transaction_id"] for row in saved for r in row["transport_exchanges"]}) == 2
    assert len(calls) == len(captured["owners"]) == len(store.list()) == 2


def test_late_sink_failure_keeps_original_exception_and_reports_retention_error(tmp_path, monkeypatch, caplog):
    error = ValueError("late source error remains exact")
    app, calls, captured, _, release = harness(tmp_path, monkeypatch, [error], late=True)
    store = app.state.operator_receipt_store

    def unavailable(*args):
        raise OSError("late disk unavailable")

    monkeypatch.setattr(store, "merge_transport_evidence", unavailable)

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            try:
                receipt = (await submit(client))[0]
                before = dict(store.connection.execute("SELECT * FROM operator_commands").fetchone())
            finally:
                release.set()
            workers = [t for t in asyncio.all_tasks() if t.get_name() == "bioxp-tester:f06-offline"]
            assert await asyncio.gather(*workers, return_exceptions=True) == [error]
            assert dict(store.connection.execute("SELECT * FROM operator_commands").fetchone()) == before
            return receipt

    receipt = asyncio.run(scenario())
    assert receipt["status"] == "outcome_unknown"
    assert not store.by_command(receipt["command_id"])["transport_exchanges"]
    snapshot = captured["owner"].snapshot()
    assert len(snapshot["transport_exchanges"]) == 1
    assert snapshot["transport_retention_errors"][-1]["message"] == "late disk unavailable"
    assert "Transport exchange retention failed" in caplog.text
    assert len(calls) == len(captured["endpoint"].writes) == 1


def test_store_merge_requires_exact_existing_owner_and_rolls_back(tmp_path, monkeypatch):
    app, _, captured, _, _ = harness(tmp_path, monkeypatch, ["success"])
    store = app.state.operator_receipt_store

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            return (await submit(client))[0]

    receipt = asyncio.run(scenario())
    before = dict(store.connection.execute("SELECT * FROM operator_commands").fetchone())
    with pytest.raises(KeyError):
        store.merge_transport_evidence("absent", captured["owner"].snapshot())
    bad = captured["owner"].snapshot()
    bad["transport_exchanges"][0]["command_id"] = "other-owner"
    with pytest.raises(ValueError, match="identity mismatch"):
        store.merge_transport_evidence(receipt["command_id"], bad)
    assert dict(store.connection.execute("SELECT * FROM operator_commands").fetchone()) == before
    assert not store.connection.in_transaction
    with pytest.raises(RuntimeError, match="terminal state"):
        store.put(receipt, _expected_status=receipt["status"])
    assert dict(store.connection.execute("SELECT * FROM operator_commands").fetchone()) == before


@pytest.mark.parametrize("script,outcomes", [
    (["timeout"] * 4, ["timeout"] * 4),
    (["success", "timeout", "success"], ["response", "timeout", "response"]),
])
def test_real_provider_composite_retains_omitted_child_provenance(tmp_path, monkeypatch, script, outcomes):
    app, calls, captured, _, _ = harness(tmp_path, monkeypatch, script, mode="provider_composite")

    async def scenario():
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://offline") as client:
            receipt, url, payload = await submit(client)
            assert (await client.post(url, json=payload)).json()["command_id"] == receipt["command_id"]
            return receipt

    receipt = asyncio.run(scenario())
    result = captured["result"]
    assert result["intent"] == "enable_xy_current"
    assert result["authority_receipt"]["command_id"] == receipt["command_id"]
    # The provider's bounded child projection may itself omit nested writes.
    # Verify both addressed components from actual encoded endpoint writes.
    from bioxp.usb_driver import novo_decode
    assert {novo_decode(raw)[3] for raw, _ in captured["endpoint"].writes} == {4, 5}
    from bioxp.oem_runtime_store import OEMRuntimeStore
    child = OEMRuntimeStore(tmp_path).read_serial206_receipt("x", receipt["command_id"])
    assert child["command_id"] == receipt["command_id"]
    assert child["result"] == result["result"]
    for detail in (False, True):
        store = OperatorReceiptStore(tmp_path)
        saved = store.by_command(receipt["command_id"], include_evidence=detail)
        assert saved["response"]["body"] == {"ok": True}
        rows = saved["transport_exchanges"]
        assert [r["outcome"] for r in rows] == outcomes
        assert {r["expected_command"] for r in rows} == {5}
        assert len({r["transaction_id"] for r in rows}) == 2
        assert {r["command_id"] for r in rows} == {receipt["command_id"]}
        assert store.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 1
    assert len(calls) == 1
    assert len(captured["endpoint"].writes) == len(script)
