from __future__ import annotations

import base64
import hashlib
import json
import sqlite3
import time

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.operator_reports import create_operator_reports_router
from bioxp.oem_runtime_store import OEMRuntimeStore
import bioxp.pipette.receipts as pipette_receipts_module


@pytest.fixture(autouse=True)
def verified_pipette_authority(monkeypatch):
    monkeypatch.setattr(pipette_receipts_module, "current_release_identity", lambda: {
        "verified": True,
        "release_id": "test-release",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64},
    })
    monkeypatch.setattr(pipette_receipts_module, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True,
        "evidence_lock_sha256": "3" * 64,
    })
    monkeypatch.setattr(pipette_receipts_module, "current_registry_sha256", lambda: "4" * 64)


def _app(tmp_path, *, writer_health_provider=None):
    owner = OEMRuntimeStore(tmp_path)
    owner.close()
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
            "pressure_units": "kPa",
            "pressure_samples": [{"channel": 1, "sample_sequence": 0, "controller_time": 1.0, "value": 4.5, "units": "kPa"}],
        },
        runtime_binding={"bms_invocation_id": "bms-report-1"},
        command_id=claim["command_id"],
        pipette_operation_id=claim["pipette_operation_id"],
        expected_status=claim["status"],
    )
    app = FastAPI()
    app.include_router(
        create_operator_reports_router(
            operator_store,
            writer_health_provider=writer_health_provider,
        )
    )
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
    assert summary.json()["pipette_operations"]["total"] == 1
    assert summary.json()["runtime_events"]["total"] == 0
    assert summary.json()["pressure"]["streams"] == 1
    assert store.connection.total_changes == before


def test_summary_totals_share_the_command_filter_scope(tmp_path):
    app, _store = _app(tmp_path)
    pipette = PipetteReceiptStore(tmp_path)
    claim, _ = pipette.claim(
        operation="dispense",
        requested_inputs={"channel": 0, "volume_ul": 5},
        entrypoint_id="api.liquid.dispense",
        caller_class="direct_api",
        control_class="physical_liquid_command",
        idempotency_key="report-idem-2",
        command_id="report-command-2",
        ownership_generation=4,
        runtime_binding={"bms_invocation_id": "bms-report-2"},
    )
    pipette.record_failure(
        command_id=claim["command_id"],
        pipette_operation_id=claim["pipette_operation_id"],
        operation="dispense",
        failure_code="test_failure",
        message="filtered fixture",
        expected_status=claim["status"],
    )
    pipette.record_event(
        command_id=claim["command_id"],
        pipette_operation_id=claim["pipette_operation_id"],
        event_source="fixture",
        event_kind="fixture_failure",
        event_payload={"ok": False},
    )
    pipette.record_pressure_stream(
        command_id=claim["command_id"],
        pipette_operation_id=claim["pipette_operation_id"],
        channels=[0],
        sample_period_ms=10,
        source_generation=4,
    )
    claim3, _ = pipette.claim(
        operation="mix",
        requested_inputs={"channel": 2},
        entrypoint_id="api.liquid.mix",
        caller_class="direct_api",
        control_class="physical_liquid_command",
        idempotency_key="report-idem-3",
        command_id="report-command-3",
        ownership_generation=4,
        runtime_binding={"bms_invocation_id": "bms-report-3"},
    )
    pipette.record_failure(
        command_id=claim3["command_id"],
        pipette_operation_id=claim3["pipette_operation_id"],
        operation="mix",
        failure_code="test_failure",
        message="filtered fixture two",
        expected_status=claim3["status"],
    )

    summary = TestClient(app).get("/operator/reports/summary?status=completed").json()
    assert summary["commands"]["total"] == 1
    assert summary["pipette_operations"]["total"] == 1
    assert summary["runtime_events"]["total"] == 0
    assert summary["pressure"]["streams"] == 1
    events = TestClient(app).get("/operator/reports/events?status=completed").json()
    assert events["returned_count"] == 0
    pressure = TestClient(app).get("/operator/reports/pressure-streams?status=completed").json()
    assert pressure["returned_count"] == 1
    pipette_page = TestClient(app).get("/operator/reports/pipette?status=failed&limit=1")
    assert pipette_page.status_code == 200
    assert pipette_page.json()["returned_count"] == 1
    assert pipette_page.json()["has_more"] is True
    assert pipette_page.json()["next_cursor"]


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
    expected_sha = metadata.json()["sha256"]
    assert downloaded.headers["x-content-sha256"] == expected_sha
    assert hashlib.sha256(downloaded.content).hexdigest() == expected_sha
    assert len(downloaded.content) == metadata.json()["byte_count"]
    payload = downloaded.json()
    assert payload["commands"][0]["command_id"] == "report-command-1"


def test_export_download_rejects_symlink_artifact(tmp_path):
    app, store = _app(tmp_path)
    client = TestClient(app)
    created = client.post("/operator/reports/exports", json={"format": "json", "limit": 100})
    assert created.status_code == 200
    export_id = created.json()["export_id"]
    row = store.connection.execute(
        "SELECT artifact_relpath FROM report_exports WHERE export_id=?", (export_id,)
    ).fetchone()
    artifact = store.root / row["artifact_relpath"]
    outside = tmp_path.parent / f"outside-{export_id}.json"
    outside.write_bytes(artifact.read_bytes())
    artifact.unlink()
    artifact.symlink_to(outside)

    downloaded = client.get(f"/operator/reports/exports/{export_id}/download")
    assert downloaded.status_code == 409
    assert downloaded.json()["detail"]["error"] == "export_integrity_failure"


def test_audit_health_is_read_only_exact_and_degraded_when_receipts_are_absent(tmp_path):
    app, store = _app(tmp_path)
    before = store.connection.total_changes

    response = TestClient(app).get("/operator/audit-health")

    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "degraded"
    assert body["checks"]["schema"]["status"] == "ok"
    assert body["checks"]["integrity"]["status"] == "ok"
    assert body["checks"]["foreign_keys"]["status"] == "ok"
    assert body["checks"]["checkpoint"]["status"] == "degraded"
    assert body["checks"]["backup"]["status"] == "degraded"
    assert body["checks"]["writer"]["status"] == "degraded"
    assert body["checks"]["writer"]["writer_status"] is None
    assert body["checks"]["writer"]["queue_depth"] is None
    assert body["checks"]["writer"]["telemetry_available"] is False
    assert "audit_writer_evidence" in body["degraded_reasons"]
    assert body["physical_admission_gate_added"] is False
    assert store.connection.total_changes == before


def test_real_bioxp_app_exposes_report_routes(monkeypatch, tmp_path):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    monkeypatch.delenv("BIOXP_PIPETTE_RECEIPT_ROOT", raising=False)
    from bioxp.api import app

    with TestClient(app):
        paths = app.openapi()["paths"]
    assert "/operator/reports/summary" in paths
    assert "/operator/reports/pipette/{pipette_operation_id}/exchanges" in paths
    assert "/operator/reports/exports" in paths
    assert "/operator/audit-health" in paths



def _claim_report_command(tmp_path, suffix: str):
    pipette = PipetteReceiptStore(tmp_path)
    claim, _ = pipette.claim(
        operation=f"report-{suffix}",
        requested_inputs={"channel": 1},
        entrypoint_id=f"api.report.{suffix}",
        caller_class="direct_api",
        control_class="physical_liquid_command",
        idempotency_key=f"report-idem-{suffix}",
        command_id=f"report-command-{suffix}",
        ownership_generation=4,
        runtime_binding={"bms_invocation_id": f"bms-report-{suffix}"},
    )
    return claim


def test_default_window_max_window_and_v3_metadata_remain_immutable_without_get_writes(tmp_path):
    app, store = _app(tmp_path)
    client = TestClient(app)
    before = store.connection.total_changes

    response = client.get("/operator/reports/summary")
    assert response.status_code == 200
    selected = response.json()["filters"]
    assert selected["end"] - selected["start"] == 24 * 60 * 60
    assert store.connection.total_changes == before

    too_wide = client.get(
        "/operator/reports/summary",
        params={"start": 0, "end": 31 * 24 * 60 * 60 + 1},
    )
    assert too_wide.status_code == 422

    with pytest.raises(sqlite3.IntegrityError, match="report identity metadata is immutable"):
        store.connection.execute(
            "DELETE FROM runtime_metadata WHERE key IN (?,?)",
            ("database_incarnation_id", "report_cursor_hmac_key"),
        )
    before_commands = store.connection.total_changes
    available = client.get("/operator/reports/commands")
    assert available.status_code == 200
    assert store.connection.total_changes == before_commands


def test_runtime_event_channel_is_typed_and_export_body_rejects_unknown_or_coerced_fields(tmp_path):
    app, store = _app(tmp_path)
    operation_id = store.connection.execute(
        "SELECT pipette_operation_id FROM pipette_operations WHERE command_id=?",
        ("report-command-1",),
    ).fetchone()["pipette_operation_id"]
    PipetteReceiptStore(tmp_path).record_event(
        command_id="report-command-1",
        pipette_operation_id=operation_id,
        event_source="fixture",
        event_kind="typed_channel",
        event_payload={"observed": True},
        channel=1,
    )
    client = TestClient(app)

    channel_one = client.get("/operator/reports/events", params={"channel": 1})
    assert channel_one.status_code == 200
    assert channel_one.json()["filtered_total"] == 1
    assert channel_one.json()["events"][0]["channel"] == 1
    assert client.get("/operator/reports/events", params={"channel": 0}).json()["filtered_total"] == 0
    assert client.get("/operator/reports/events", params={"channel": "1.5"}).status_code == 422

    assert client.post(
        "/operator/reports/exports",
        json={"format": "json", "limit": 1, "unexpected": True},
    ).status_code == 422
    assert client.post(
        "/operator/reports/exports",
        json={"format": "json", "limit": "1"},
    ).status_code == 422
    assert client.post(
        "/operator/reports/exports",
        json={"format": "json", "limit": 1, "delivery_verified": "false"},
    ).status_code == 422


def test_cursor_is_complete_hmac_bound_and_totals_do_not_shrink_on_continuation(tmp_path):
    app, _store = _app(tmp_path)
    _claim_report_command(tmp_path, "cursor-2")
    _claim_report_command(tmp_path, "cursor-3")
    client = TestClient(app)
    end = time.time() + 60
    params = {"start": end - 120, "end": end, "limit": 1}

    first = client.get("/operator/reports/commands", params=params)
    assert first.status_code == 200
    first_body = first.json()
    assert first_body["filtered_total"] == 3
    assert first_body["has_more"] is True
    cursor = first_body["next_cursor"]
    envelope = json.loads(
        base64.urlsafe_b64decode((cursor + "=" * (-len(cursor) % 4)).encode()).decode()
    )
    payload = envelope["payload"]
    assert payload["report_kind"] == "commands"
    assert payload["filters"] == first_body["filters"]
    assert payload["database_incarnation_id"] == first_body["snapshot"]["database_incarnation_id"]
    assert payload["schema_identity"] == first_body["snapshot"]["schema_identity"]
    assert payload["source_high_waters"] == first_body["snapshot"]["source_high_waters"]
    assert payload["expires_at"] > payload["issued_at"]

    _claim_report_command(tmp_path, "cursor-after-high-water")
    continued = client.get(
        "/operator/reports/commands",
        params={**params, "cursor": cursor},
    )
    assert continued.status_code == 200
    assert continued.json()["filtered_total"] == 3
    assert all(
        row["command_id"] != "report-command-cursor-after-high-water"
        for row in continued.json()["commands"]
    )

    envelope["payload"]["position"]["last_sequence"] = 999999
    tampered = base64.urlsafe_b64encode(
        json.dumps(envelope, sort_keys=True, separators=(",", ":")).encode()
    ).decode().rstrip("=")
    assert client.get(
        "/operator/reports/commands",
        params={**params, "cursor": tampered},
    ).status_code == 409
    assert client.get(
        "/operator/reports/events",
        params={**params, "cursor": cursor},
    ).status_code == 409


def test_detail_children_are_bounded_and_expose_continuation_contracts(tmp_path):
    app, store = _app(tmp_path)
    client = TestClient(app)

    detail = client.get("/operator/reports/commands/report-command-1")
    assert detail.status_code == 200
    body = detail.json()
    assert body["child_page_limit"] == 100
    assert len(body["transitions"]) <= 100
    assert len(body["evidence_preview"]) <= 100
    assert set(body["evidence_continuation"]) == {
        "returned_count", "filtered_total", "has_more", "next_cursor"
    }

    transitions = client.get(
        "/operator/reports/commands/report-command-1/transitions",
        params={"limit": 1},
    ).json()
    evidence = client.get(
        "/operator/reports/commands/report-command-1/evidence",
        params={"limit": 1},
    ).json()
    for child in (transitions, evidence):
        assert child["returned_count"] <= 1
        assert "filtered_total" in child
        assert "has_more" in child
        assert "next_cursor" in child

    operation_id = store.connection.execute(
        "SELECT pipette_operation_id FROM pipette_operations WHERE command_id=?",
        ("report-command-1",),
    ).fetchone()["pipette_operation_id"]
    channels = client.get(
        f"/operator/reports/pipette/{operation_id}/channels",
        params={"limit": 1},
    ).json()
    exchanges = client.get(
        f"/operator/reports/pipette/{operation_id}/exchanges",
        params={"limit": 1},
    ).json()
    assert channels["returned_count"] <= 1
    assert exchanges["returned_count"] <= 1


def test_export_receipt_is_complete_normalized_registered_and_byte_preserving(tmp_path):
    app, store = _app(tmp_path)
    client = TestClient(app)
    created = client.post(
        "/operator/reports/exports",
        json={"format": "JSON", "limit": 1, "channel": 1},
    )
    assert created.status_code == 200
    export_id = created.json()["export_id"]
    metadata = client.get(f"/operator/reports/exports/{export_id}")
    assert metadata.status_code == 200
    body = metadata.json()
    receipt = body["receipt"]
    assert receipt["receipt_schema"] == "bioxp.operator_report_export_receipt.v1"
    assert receipt["publisher_identity"] == "bioxp.operator_reports"
    assert receipt["export_id"] == export_id
    assert receipt["normalized_filters"] == body["filter"]
    assert receipt["filter_sha256"] == body["filter_sha256"]
    assert set(receipt["source_high_waters"]) >= {"operator_commands", "runtime_events"}
    assert receipt["schema_identity"]
    assert receipt["database_incarnation_id"]
    assert receipt["artifact"]["format"] == "json"
    assert receipt["artifact"]["sha256"] == body["sha256"]
    assert receipt["artifact"]["byte_count"] == body["byte_count"]

    download = client.get(f"/operator/reports/exports/{export_id}/download")
    assert download.status_code == 200
    assert download.content == (store.root / receipt["artifact"]["relpath"]).read_bytes()
    assert hashlib.sha256(download.content).hexdigest() == body["sha256"]

    registered = {
        row["artifact_relpath"]
        for row in store.connection.execute("SELECT artifact_relpath FROM report_exports").fetchall()
    }
    artifacts = {
        path.relative_to(store.root).as_posix()
        for path in (store.root / "report_exports").iterdir()
        if path.is_file()
    }
    assert artifacts == registered

    with pytest.raises(sqlite3.IntegrityError, match="append-only table cannot be updated"):
        store.connection.execute(
            "UPDATE report_exports SET snapshot_json=? WHERE export_id=?",
            ("{}", export_id),
        )
    assert client.get(f"/operator/reports/exports/{export_id}").status_code == 200
    assert client.get(f"/operator/reports/exports/{export_id}/download").status_code == 200


def test_audit_health_uses_real_writer_telemetry_and_fails_degraded_when_unavailable(tmp_path):
    app, _store = _app(
        tmp_path,
        writer_health_provider=lambda: {"status": "ok", "queue_depth": 0, "mode": "serialized"},
    )
    writer = TestClient(app).get("/operator/audit-health").json()["checks"]["writer"]
    assert writer == {
        "status": "ok",
        "writer_status": "ok",
        "queue_depth": 0,
        "telemetry_available": True,
        "mode": "serialized",
    }

    def unavailable_writer():
        raise RuntimeError("telemetry offline")

    degraded_app, _degraded_store = _app(
        tmp_path / "degraded",
        writer_health_provider=unavailable_writer,
    )
    degraded = TestClient(degraded_app).get("/operator/audit-health").json()
    assert degraded["status"] == "degraded"
    assert degraded["checks"]["writer"]["status"] == "degraded"
    assert degraded["checks"]["writer"]["writer_status"] == "unavailable"
    assert degraded["checks"]["writer"]["telemetry_available"] is True
    assert "audit_writer_evidence" in degraded["degraded_reasons"]
