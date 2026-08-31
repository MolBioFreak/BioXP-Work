from __future__ import annotations

import asyncio
import hashlib
import importlib
import json
import sqlite3

import pytest

from src.bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.audit import PipetteAuditIntegrityError, normalize_pipette_result
from bioxp.pipette.receipts import PipetteReceiptError
from bioxp.pipette.models import PipetteAspirateCommand, PipetteTransportUnavailableError
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.services.pipette_service import _run_transport_call


@pytest.fixture(autouse=True)
def _canonical_runtime_state(tmp_path, monkeypatch):
    runtime = OEMRuntimeStore(tmp_path)
    runtime.close()
    pipette_receipts_module = importlib.import_module(PipetteReceiptStore.__module__)
    monkeypatch.setattr(pipette_receipts_module, "current_release_identity", lambda: {
        "verified": True,
        "release_id": "test-release",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64},
    })
    monkeypatch.setattr(pipette_receipts_module, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True,
        "evidence_lock_sha256": "3" * 64,
    })


def _claim(store: PipetteReceiptStore):
    return store.claim(
        operation="aspirate",
        requested_inputs={"volume_ul": 10.0},
        entrypoint_id="direct.liquid.aspirate",
        caller_class="direct_api",
        control_class="physical_liquid_command",
        idempotency_key="wp2-key",
    )[0]


def _producer_result() -> dict:
    return {
        "ok": False,
        "outcome": "completion",
        "controller_acknowledged": False,
        "completion_received": True,
        "provenance": {
            "channel": 1,
            "transaction_id": "txn-1",
            "command_family": 1,
            "tx_id": 0x109,
            "tx_dlc": 3,
            "tx_data": [0x41, 0x52, 0x20],
            "expected_rx_id": 0x509,
        },
        "observed_rx_id": 0x509,
        "observed_rx_dlc": 2,
        "observed_rx_raw": [0x20, 0x00],
        "hardware_tip_status": {"channel": 1, "tip_loaded": True},
        "hardware_pressure": {"channel": 1, "value": 12.5, "units": "PSI"},
    }


def test_wp2_persists_typed_channel_exchange_event_and_pressure_chunk(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    claim = _claim(store)
    command_id = claim["command_id"]
    operation_id = claim["pipette_operation_id"]

    observation_id = store.record_channel_observation(
        command_id=command_id,
        pipette_operation_id=operation_id,
        channel=1,
        phase="query",
        semantic_validity="valid",
        truth_source="novo_query",
        tip_loaded=True,
        pressure=12.5,
        pressure_units="PSI",
        status="ok",
        error_code=None,
        detail={"tip": {"loaded": True}},
    )
    exchange_id = store.record_transport_exchange(
        command_id=command_id,
        pipette_operation_id=operation_id,
        channel=1,
        transaction_phase="ack",
        command_family=1,
        matcher_name="pipette_command",
        tx_id=0x109,
        tx_dlc=3,
        tx_bytes=[0x41, 0x52, 0x20],
        expected_rx_id=0x509,
        observed_rx_id=0x509,
        rx_dlc=2,
        rx_bytes=[0x20, 0x00],
        delivery_verified=True,
        semantic_match=True,
        controller_acknowledged=True,
        completion_verified=False,
        completion_before_ack=False,
        raw_exchange={"source": "novo_router"},
    )
    event_id = store.record_event(
        command_id=command_id,
        pipette_operation_id=operation_id,
        event_source="novo_router",
        event_kind="completion_before_ack",
        event_payload={"transaction_id": "txn-1"},
        channel=1,
        transaction_id="txn-1",
        semantic_validity="tainted",
    )
    stream_id = store.record_pressure_stream(
        command_id=command_id,
        pipette_operation_id=operation_id,
        channels=[1],
        sample_period_ms=10,
        source_generation=4,
    )
    raw_samples = [
        {"channel": 1, "sample_sequence": 0, "raw_pressure": 10.0, "corrected_pressure": 9.5},
        {"channel": 1, "sample_sequence": 1, "raw_pressure": 12.0, "corrected_pressure": 11.5},
    ]
    chunk_id = store.record_pressure_chunk(
        stream_session_id=stream_id,
        channel=1,
        chunk_sequence=0,
        samples=raw_samples,
        units="PSI",
        offset_identity="offset-1",
        chunk_schema="bioxp.pipette.pressure.v1",
    )

    assert all(isinstance(value, str) and value for value in (observation_id, exchange_id, event_id, stream_id, chunk_id))
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_channel_observations").fetchone()[0] == 1
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_transport_exchanges").fetchone()[0] == 1
    assert store.connection.execute("SELECT COUNT(*) FROM runtime_events").fetchone()[0] == 1
    chunk = store.connection.execute(
        "SELECT sample_count,raw_min,raw_max,corrected_mean,sha256 FROM pipette_pressure_chunks"
    ).fetchone()
    assert tuple(chunk[:4]) == (2, 10.0, 12.0, 10.5)
    assert len(chunk[4]) == 64

    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        store.connection.execute(
            "UPDATE pipette_channel_observations SET phase='ack' WHERE observation_id=?",
            (observation_id,),
        )
    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        store.connection.execute(
            "DELETE FROM pipette_transport_exchanges WHERE exchange_id=?",
            (exchange_id,),
        )

    with pytest.raises(PipetteAuditIntegrityError, match="expected_rx_id"):
        store.record_transport_exchange(
            command_id=command_id,
            pipette_operation_id=operation_id,
            channel=1,
            transaction_phase="ack",
            command_family=1,
            matcher_name="pipette_command",
            tx_id=0x109,
            tx_dlc=0,
            tx_bytes=[],
            expected_rx_id=0x508,
            observed_rx_id=0x508,
            rx_dlc=0,
            rx_bytes=[],
            delivery_verified=False,
            semantic_match=False,
            controller_acknowledged=False,
            completion_verified=False,
            completion_before_ack=False,
            raw_exchange={},
        )


def test_normalizer_uses_observed_rx_id_and_preserves_completion_taint():
    normalized = normalize_pipette_result(_producer_result())

    assert normalized["channels"][0]["channel"] == 1
    assert normalized["channels"][0]["tip_loaded"] is True
    assert normalized["channels"][0]["pressure"] == 12.5
    assert normalized["exchanges"][0]["observed_rx_id"] == 0x509
    assert normalized["exchanges"][0]["expected_rx_id"] == (0x109 | 0x400)
    assert normalized["events"][0]["event_kind"] == "completion_before_ack"
    assert normalized["events"][0]["semantic_validity"] == "tainted"


def test_normalizer_keeps_oem_error_and_callback_event_kinds_distinct():
    result = {
        "ok": False,
        "outcome": "failed",
        "provenance": {"channel": 1, "transaction_id": "txn-errors"},
        "control_lib_error_event": {"code": 17},
        "pipette_error": {"code": 42},
        "q1_error_code": 9,
        "callback_error": "callback delivery failed",
        "duplicate_terminal_count": 2,
        "late_completion": True,
    }
    kinds = {event["event_kind"] for event in normalize_pipette_result(result)["events"]}
    assert kinds == {
        "ControlLib.errorEvent",
        "pipetteError",
        "q1_error_state",
        "callback_delivery_failure",
        "duplicate_completion",
        "late_completion",
    }


def test_normalizer_flattens_group_send_channel_and_driver_provenance():
    result = {
        "ok": True,
        "outcome": "completion",
        "sends": [
            {
                "channel": 2,
                "result": {
                    "ok": True,
                    "controller_acknowledged": True,
                    "completion_verified": True,
                    "driver_result": {
                        "provenance": {
                            "transaction_id": "txn-group-2",
                            "channel": 2,
                            "tx_id": 0x10B,
                            "tx_dlc": 3,
                            "tx_data": [0x41, 0x52, 0x20],
                            "expected_rx_id": 0x50B,
                            "observed_rx_id": 0x50B,
                            "rx_dlc": 0,
                            "rx_data": [],
                        }
                    },
                },
            }
        ],
    }
    normalized = normalize_pipette_result(result)
    assert normalized["channels"][0]["channel"] == 2
    assert normalized["exchanges"][0]["transaction_id"] == "txn-group-2"


def test_receipt_persists_pressure_samples_as_chunk_metadata(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    claim = _claim(store)
    store.record(
        operation="pressure_stream",
        requested_inputs={"channel": 1},
        result={
            "ok": True,
            "outcome": "completion",
            "channel": 1,
            "pressure_units": "PSI",
            "pressure_samples": [
                {"channel": 1, "sample_sequence": 0, "controller_time": 10.0, "value": 1.0},
                {"channel": 1, "sample_sequence": 1, "controller_time": 10.1, "value": 2.0},
            ],
        },
        command_id=claim["command_id"],
        pipette_operation_id=claim["pipette_operation_id"],
        expected_status=claim["status"],
    )
    stream_count = store.connection.execute("SELECT COUNT(*) FROM pipette_pressure_streams").fetchone()[0]
    chunk_count = store.connection.execute("SELECT COUNT(*) FROM pipette_pressure_chunks").fetchone()[0]
    assert stream_count == 1
    assert chunk_count == 1
    assert not store._legacy_path.exists()


def test_invalid_typed_result_keeps_claim_nonterminal(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    claim = _claim(store)
    with pytest.raises(PipetteReceiptError):
        store.record(
            operation="aspirate",
            requested_inputs={"volume_ul": 10.0},
            result={
                "ok": True,
                "outcome": "completion",
                "provenance": {"channel": 0, "tx_id": 0x101, "expected_rx_id": 0x500},
            },
            command_id=claim["command_id"],
            pipette_operation_id=claim["pipette_operation_id"],
            expected_status=claim["status"],
        )
    status = store.connection.execute(
        "SELECT status FROM operator_commands WHERE command_id=?", (claim["command_id"],)
    ).fetchone()[0]
    assert status == "failed"
    pipette_status = store.connection.execute(
        "SELECT status,failure_code FROM pipette_operations WHERE pipette_operation_id=?",
        (claim["pipette_operation_id"],),
    ).fetchone()
    assert pipette_status["status"] == "failed"
    assert pipette_status["failure_code"] == "pipette_result_normalization_failed"
def test_service_finalizes_admission_failure_after_claim(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    command = PipetteAspirateCommand(volume_ul=10.0, air_gap_ul=1.0)
    transport_called = False

    def get_transport():
        nonlocal transport_called
        transport_called = True
        raise AssertionError("transport must not run after admission rejection")

    async def runner(*args, **kwargs):
        return await kwargs["fn"]()

    with pytest.raises(Exception):
        asyncio.run(
            _run_transport_call(
                "Aspirate",
                timeout_s=1.0,
                get_transport=get_transport,
                run_blocking=runner,
                operation=lambda transport: {"ok": True},
                operation_name="aspirate",
                command=command,
                receipt_store=store,
                runtime_binding={"idempotency_key": "wp2-admission-failure"},
                )
        )

    assert transport_called is False
    row = store.connection.execute(
        "SELECT status,failure_code FROM operator_commands ORDER BY sequence DESC LIMIT 1"
    ).fetchone()
    assert tuple(row) == ("rejected", "validation_error")


def test_service_finalizes_transport_unavailable_after_claim(tmp_path):
    store = PipetteReceiptStore(tmp_path)

    def get_transport():
        raise PipetteTransportUnavailableError()

    async def runner(*args, **kwargs):
        return await kwargs["fn"]()

    with pytest.raises(Exception):
        asyncio.run(
            _run_transport_call(
                "Pipette status",
                timeout_s=1.0,
                get_transport=get_transport,
                run_blocking=runner,
                operation=lambda transport: {"ok": True},
                operation_name="status",
                receipt_store=store,
            )
        )

    row = store.connection.execute(
        "SELECT status,failure_code FROM operator_commands ORDER BY sequence DESC LIMIT 1"
    ).fetchone()
    assert tuple(row) == ("failed", "transport_unavailable")
    assert store.connection.execute(
        "SELECT status,outcome FROM pipette_operations ORDER BY created_at DESC LIMIT 1"
    ).fetchone()[0] == "failed"
