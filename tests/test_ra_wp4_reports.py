from __future__ import annotations

import json

from fastapi import FastAPI
from fastapi.testclient import TestClient

from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.operator_reports import create_operator_reports_router


def _app(tmp_path):
    operator_store = OperatorReceiptStore(tmp_path)
    pipette_store = PipetteReceiptStore(tmp_path)
    claim, _ = pipette_store.claim(
        operation="aspirate",
        requested_inputs={"channel": 1, "volume_ul": 10},
        entrypoint_id="api.liquid.aspirate",
        caller_class="direct_api",
        control_class="physical_liquid_command",
        idempotency_key="report-idem-1",
        command_id="report-command-1",
        ownership_generation=4,
        runtime_binding={"bms_invocation_id": "bms-report-1"},
    )
    pipette_store.record(
        operation="aspirate",
        requested_inputs={"channel": 1, "volume_ul": 10},
        effective_inputs={"channel": 1, "volume_ul": 10},
        result={
            "ok": True,
            "outcome": "completion",
            "controller_acknowledged": True,
            "completion_received": True,
            "completion_verified": True,
            "channels": [{"channel": 1, "phase": "completion", "tip_loaded": True, "pressure": 4.5, "pressure_units": "kPa"}],
            "provenance": {"transaction_id": "report-tx-1", "tx_id": 0x109, "tx_dlc": 2, "tx_data": [1, 2], "expected_rx_id": 0x509, "observed_rx_id": 0x509},
            "pressure_samples": [{"channel": 1, "sample_sequence": 0, "controller_time": 1.0, "value": 4.5}],
        },
        runtime_binding={"bms_invocation_id": "bms-report-1"},
        command_id=claim["command_id"],
        pipette_operation_id=claim["pipette_operation_id"],
    )
    app = FastAPI()
    app.include_router(create_operator_reports_router(operator_store))
    return app, operator_store


def test_report_gets_use_one_read_snapshot_and_do_not_write(tmp_path):
    app, store = _app(tmp_path)
    client = TestClient(app)
    before = store.connection.total_changes
    response = client.get("/operator/reports/commands?limit=1")
    assert response.status_code == 200
    body = response.json()
    assert body["returned_count"] == 1
    assert body["has_more"] is False
    assert body["filters"]["limit"] == 1
    assert store.connection.total_changes == before

    channel_one = client.get("/operator/reports/commands?channel=1")
    assert channel_one.json()["filtered_total"] == 1
    channel_zero = client.get("/operator/reports/commands?channel=0")
    assert channel_zero.json()["filtered_total"] == 0

    summary = client.get("/operator/reports/summary?status=completed")
    assert summary.status_code == 200
    assert summary.json()["scope"] == "filtered"
    assert summary.json()["commands"]["total"] == 1
    assert store.connection.total_changes == before


def test_report_detail_exposes_typed_pipette_children_without_paths(tmp_path):
    app, store = _app(tmp_path)
    client = TestClient(app)
    before = store.connection.total_changes
    command = client.get("/operator/reports/commands/report-command-1")
    assert command.status_code == 200
    detail = command.json()
    assert detail["command_id"] == "report-command-1"
    assert detail["pipette"]["operation"] == "aspirate"
    assert detail["pipette"]["channels"][0]["channel"] == 1
    assert detail["pipette"]["exchanges"][0]["observed_rx_id"] == 0x509
    assert all("path" not in item and "relpath" not in item for item in detail["evidence"])
    assert store.connection.total_changes == before


def test_report_export_is_post_then_read_only_download(tmp_path):
    app, store = _app(tmp_path)
    client = TestClient(app)
    created = client.post("/operator/reports/exports", json={"format": "json", "limit": 100})
    assert created.status_code == 200
    export_id = created.json()["export_id"]
    metadata = client.get(f"/operator/reports/exports/{export_id}")
    assert metadata.status_code == 200
    assert metadata.json()["status"] == "completed"
    assert "path" not in metadata.json()
    downloaded = client.get(f"/operator/reports/exports/{export_id}/download")
    assert downloaded.status_code == 200
    payload = downloaded.json()
    assert payload["commands"][0]["command_id"] == "report-command-1"


def test_real_bioxp_app_exposes_report_routes(monkeypatch, tmp_path):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.delenv("BIOXP_OEM_RUNTIME_ROOT", raising=False)
    monkeypatch.delenv("BIOXP_PIPETTE_RECEIPT_ROOT", raising=False)
    from bioxp.api import app

    paths = app.openapi()["paths"]
    assert "/operator/reports/summary" in paths
    assert "/operator/reports/pipette/{pipette_operation_id}/exchanges" in paths
    assert "/operator/reports/exports" in paths
    assert "/operator/audit-health" in paths
