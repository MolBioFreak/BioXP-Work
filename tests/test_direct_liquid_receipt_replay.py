"""Offline real-route replay regressions; only the hardware boundary is fake.

AST-load this checkout's middleware/models/routes, avoiding API app lifespan.
Readback issues one fake hardware query; planning only records a no-motion plan.
"""
import ast
import asyncio
import json
from pathlib import Path
import re
from typing import Literal, Optional

import httpx
import pytest
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_controls import current_operator_dispatch_context
from bioxp.pipette.application import PipetteApplicationPlanner
from bioxp.pipette import receipts
from bioxp.services.pipette_service import (
    _DIRECT_PIPETTE_IDEMPOTENCY,
    reset_direct_pipette_idempotency_key,
    set_direct_pipette_idempotency_key,
    run_pipette_operation,
)

ROOT = Path(__file__).resolve().parents[1]
KEY = "f33:duplicate-12345678"
CASES = [
    ("readback", "/liquid/readback", {"include_data": False}, {"include_data": True}),
    ("plan", "/liquid/application/plan", {"operation": "detect_fluid", "fluid_class": "RC"},
     {"operation": "detect_fluid", "fluid_class": "MS"}),
]


@pytest.fixture
def owner(tmp_path, monkeypatch):
    runtime = OEMRuntimeStore(tmp_path)
    runtime.close()
    monkeypatch.setattr(receipts, "current_release_identity", lambda: {
        "verified": True, "release_id": "test-release",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64},
    })
    monkeypatch.setattr(receipts, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True, "evidence_lock_sha256": "3" * 64,
    })
    store = receipts.PipetteReceiptStore(tmp_path)
    calls = []
    dependencies = {name: {"bound": True, "generation": 7, "state": {"ready": True}}
                    for name in ("deck", "gantry", "z", "pressure", "pipette", "machine_state")}

    class QueryTransport:
        def readback_all(self, *, include_data):
            calls.append(include_data)
            return {"ok": True, "hardware_truth_level": "hardware_query",
                    "query_response_correlated": True, "semantic_ok": True,
                    "physical_effect_verified": False, "channels": []}

    async def blocking(label, fn, *, timeout_s):
        return fn()

    app = FastAPI()
    source = ROOT / "src/bioxp/api.py"
    names = {"bind_direct_pipette_idempotency", "PipetteReadbackRequest",
             "PipetteApplicationPlanRequest", "liquid_readback", "liquid_application_plan"}
    selected = [node for node in ast.parse(source.read_text()).body
                if getattr(node, "name", None) in names]
    assert {node.name for node in selected} == names
    namespace = {**globals(), "app": app,
                 "_pipette_application": PipetteApplicationPlanner(dependency_resolver=lambda: dependencies),
                 "_pipette_receipts": store, "_get_pipette_transport": lambda: QueryTransport(),
                 "_run_blocking": blocking, "PipetteReceiptError": receipts.PipetteReceiptError}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), "exec"), namespace)
    yield app, namespace, calls, dependencies, tmp_path
    namespace["_pipette_receipts"].connection.close()


@pytest.mark.parametrize("operation,path,body,changed", CASES)
def test_same_key_replays_original_response_after_reopen(owner, operation, path, body, changed):
    app, namespace, calls, dependencies, root = owner

    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://robot") as client:
            first = await client.post(path, json=body, headers={"Idempotency-Key": KEY})
            assert first.status_code == 200, first.text
            original = first.json()
            assert original["receipt_id"]
            assert original["receipt_truth"]["physical_effect_verified"] is False
            assert original["ok"] is True
            store = namespace["_pipette_receipts"]
            persisted_before = [tuple(row) for row in store.connection.execute("SELECT * FROM pipette_operations")]
            dependencies.clear()  # Recovery must not substitute a newly computed plan.
            for reopen in (False, True):
                if reopen:
                    namespace["_pipette_receipts"].connection.close()
                    namespace["_pipette_receipts"] = receipts.PipetteReceiptStore(root)
                response = await client.post(path, json=body, headers={"Idempotency-Key": KEY})
                assert response.status_code == 200, response.text
                replay = response.json()
                assert calls == ([False] if operation == "readback" else [])
                assert replay.get("receipt_id") == original["receipt_id"]
                assert {key: replay.get(key) for key in original} == original
                if operation == "plan":
                    assert replay["motion_commanded"] is False
                    assert replay["execution_admitted"] is False
                    assert replay["liquid_mutation_commanded"] is False
                assert _DIRECT_PIPETTE_IDEMPOTENCY.get() is None
            store = namespace["_pipette_receipts"]
            assert [tuple(row) for row in store.connection.execute("SELECT * FROM pipette_operations")] == persisted_before
            rows = store.connection.execute("SELECT idempotency_key FROM operator_commands").fetchall()
            assert [row[0] for row in rows] == [KEY]
            persisted = store.connection.execute("SELECT receipt_json FROM pipette_operations").fetchall()
            assert len(persisted) == 1
            assert json.loads(persisted[0][0])["receipt_id"] == original["receipt_id"]
    asyncio.run(scenario())


@pytest.mark.parametrize("operation,path,body,changed", CASES)
def test_same_key_different_body_rejects_without_replacing_receipt(owner, operation, path, body, changed):
    app, namespace, calls, _, _ = owner

    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app, raise_app_exceptions=False),
                                     base_url="http://robot") as client:
            first = await client.post(path, json=body, headers={"Idempotency-Key": KEY})
            assert first.status_code == 200
            store = namespace["_pipette_receipts"]
            before = [tuple(row) for row in store.connection.execute("SELECT * FROM pipette_operations")]
            conflict = await client.post(path, json=changed, headers={"Idempotency-Key": KEY})
            assert conflict.status_code >= 400, conflict.text
            assert [tuple(row) for row in store.connection.execute("SELECT * FROM pipette_operations")] == before
            assert calls == ([False] if operation == "readback" else [])
    asyncio.run(scenario())


@pytest.mark.parametrize("operation,path,body,changed", CASES)
def test_unfinished_receipt_never_redispatches_or_finalizes_on_replay(owner, monkeypatch, operation, path, body, changed):
    app, namespace, calls, _, root = owner
    store = namespace["_pipette_receipts"]

    def interrupted(**kwargs):
        raise RuntimeError("offline persistence interruption")

    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app, raise_app_exceptions=False),
                                     base_url="http://robot") as client:
            with monkeypatch.context() as patch:
                patch.setattr(store._audit_database, "finalize_claim", interrupted)
                first = await client.post(path, json=body, headers={"Idempotency-Key": KEY})
                assert first.status_code >= 400
            before = [tuple(row) for row in store.connection.execute("SELECT * FROM pipette_operations")]
            assert len(before) == 1
            store.connection.close()
            reopened = receipts.PipetteReceiptStore(root)
            namespace["_pipette_receipts"] = reopened
            response = await client.post(path, json=body, headers={"Idempotency-Key": KEY})
            if operation == "readback":
                assert response.status_code == 200
                assert response.json()["ok"] is False
                assert response.json()["outcome"] == "in_progress"
                assert response.json()["retry_forbidden"] is True
            else:
                assert response.status_code >= 400
            assert [tuple(row) for row in reopened.connection.execute("SELECT * FROM pipette_operations")] == before
            assert calls == ([False] if operation == "readback" else [])
    asyncio.run(scenario())


@pytest.mark.parametrize("operation,path,body,changed", CASES)
def test_changed_release_cannot_recover_receipt_with_new_hardware_query(owner, monkeypatch, operation, path, body, changed):
    app, namespace, calls, _, _ = owner

    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app, raise_app_exceptions=False),
                                     base_url="http://robot") as client:
            first = await client.post(path, json=body, headers={"Idempotency-Key": KEY})
            assert first.status_code == 200
            store = namespace["_pipette_receipts"]
            before = [tuple(row) for row in store.connection.execute("SELECT * FROM pipette_operations")]
            monkeypatch.setattr(receipts, "current_release_identity", lambda: {"verified": False})
            rejected = await client.post(path, json=body, headers={"Idempotency-Key": KEY})
            assert rejected.status_code >= 400
            assert calls == ([False] if operation == "readback" else [])
            assert [tuple(row) for row in store.connection.execute("SELECT * FROM pipette_operations")] == before
    asyncio.run(scenario())


@pytest.mark.parametrize("operation,path,body,changed", CASES)
def test_missing_key_never_dispatches_or_records(owner, operation, path, body, changed):
    app, namespace, calls, _, _ = owner

    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://robot") as client:
            response = await client.post(path, json=body)
            assert response.status_code == 422
            assert calls == []
            assert namespace["_pipette_receipts"].connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 0
    asyncio.run(scenario())
