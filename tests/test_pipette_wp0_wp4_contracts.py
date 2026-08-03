from __future__ import annotations

import asyncio
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import HTTPException

from src.bioxp.can_driver import BioXpCanDriver
from src.bioxp import api
from src.bioxp.novo_router import NovoFrame, NovoRouter
from src.bioxp.novo_usb_can import NovoUsbCanError, novo_decode, novo_encode
from src.bioxp.pipette.models import PipetteAspirateCommand, PipetteDispenseCommand
from src.bioxp.pipette.receipts import PipetteReceiptStore
from src.bioxp.services.pipette_service import run_pipette_dispense_command, run_pipette_status


ROOT = Path(__file__).resolve().parents[1]


def test_wp0_matrix_has_all_gap_rows_and_no_unclassified_rows():
    matrix = json.loads((ROOT / "docs/specs/2026-08-02-pipette-oem-parity-matrix.json").read_text())
    spec_bytes = (ROOT / matrix["specification"]["path"]).read_bytes()
    assert matrix["specification"]["sha256"] == hashlib.sha256(spec_bytes).hexdigest()
    assert len(matrix["gap_rows"]) == 24
    assert matrix["matrix_invariants"]["unclassified_row_count"] == 0
    assert all(row.get("classification") for row in matrix["gap_rows"])
    assert all(row.get("status") for row in matrix["gap_rows"])


def test_novo_router_has_twenty_normal_ids_and_four_pressure_ids():
    from src.bioxp import novo_router

    assert len(novo_router.PIPETTE_RX_IDS) == 20
    assert len(novo_router.PRESSURE_STREAM_IDS) == 4
    assert novo_router.PRESSURE_STREAM_IDS.isdisjoint(novo_router.PIPETTE_RX_IDS)
    matcher = NovoRouter.pipette_matcher(channel=2, expected_function=6)
    wrong_channel = NovoRouter._decode_record(
        (0x506).to_bytes(4, "big") + bytes([0]), b"raw", 1.0
    )
    wrong_function = NovoRouter._decode_record(
        (0x510).to_bytes(4, "big") + bytes([0]), b"raw", 1.0
    )
    assert matcher(wrong_channel).matched is False
    assert matcher(wrong_channel).classification == "pipette_wrong_channel"
    assert matcher(wrong_function).matched is False
    assert matcher(wrong_function).classification == "pipette_wrong_function"


def test_fragmented_tx_uses_one_router_transaction_and_expected_id_families():
    class Bus:
        def __init__(self):
            self.calls = []

        def transact_can_many(self, messages, **kwargs):
            self.calls.append((list(messages), kwargs))
            return {
                "ok": True,
                "outcome": "completion",
                "frames": [{"arbitration_id": 0x506, "dlc": 3, "data": [0x20, 0x60, 0x00], "raw": []}],
            }

    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver.bus = Bus()
    driver.pipette_id = 2
    driver.response_timeout_s = 1.0
    result = driver._send_ascii_packet(
        driver.pipette_can_ids()["command"],
        "ABCDEFGHIJK",
        require_ack=True,
        response_timeout_s=1.0,
        command_name="long_command",
    )
    messages, kwargs = driver.bus.calls[0]
    assert result["ok"] is True
    assert result["fragmented"] is True
    assert [message.arbitration_id for message in messages] == [0x113, 0x111]
    assert kwargs["expected_function"] == 1


class _StatusTransport:
    def __init__(self):
        self.calls = []

    def get_status(self):
        self.calls.append("status")
        return {"ok": True, "hardware_truth_level": "cached_transport_state", "physical_effect_verified": False}


async def _run_blocking(_label, operation, *, timeout_s):
    del timeout_s
    return operation()


def test_service_rejects_unmapped_liquid_extensions_before_transport():
    transport = _StatusTransport()
    with pytest.raises(HTTPException) as blow_out:
        asyncio.run(
            run_pipette_dispense_command(
                PipetteDispenseCommand(volume_ul=10, pressure_profile="1R", blow_out=True),
                get_transport=lambda: transport,
                run_blocking=_run_blocking,
            )
        )
    assert blow_out.value.status_code == 400
    assert transport.calls == []

    with pytest.raises(HTTPException) as air_gap:
        asyncio.run(
            run_pipette_dispense_command(
                PipetteDispenseCommand(volume_ul=10, pressure_profile="1R", air_gap_ul=1),
                get_transport=lambda: transport,
                run_blocking=_run_blocking,
            )
        )
    assert air_gap.value.status_code == 400
    assert transport.calls == []


def test_receipt_store_binds_authority_redacts_metadata_and_suppresses_physical_claim(tmp_path):
    store = PipetteReceiptStore(tmp_path / "receipts")
    receipt = store.record(
        operation="status",
        requested_inputs={"metadata": {"token": "secret-value", "run": "fixture"}},
        effective_inputs={"live_query_performed": False},
        result={"ok": True, "hardware_postcondition_verified": True, "physical_effect_verified": True},
        runtime_binding={"owner": "test", "password": "should-not-persist"},
    )
    assert receipt["source_identity"]["authority_verified"] is True
    assert receipt["truth"]["physical_effect_verified"] is False
    assert receipt["requested_inputs"]["metadata"]["token"] == "[REDACTED]"
    assert receipt["runtime_binding"]["password"] == "[REDACTED]"
    assert store.latest()["receipt_id"] == receipt["receipt_id"]


def test_service_status_can_persist_a_source_bound_receipt(tmp_path):
    store = PipetteReceiptStore(tmp_path / "receipts")
    result = asyncio.run(
        run_pipette_status(
            get_transport=lambda: _StatusTransport(),
            run_blocking=_run_blocking,
            receipt_store=store,
        )
    )
    assert result["receipt_id"] == store.latest()["receipt_id"]
    assert result["receipt_truth"]["physical_effect_verified"] is False


def test_pipette_id_range_and_strict_novo_frame_contracts():
    assert BioXpCanDriver._validate_pipette_id(0) == 0
    assert BioXpCanDriver._validate_pipette_id(3) == 3
    with pytest.raises(ValueError):
        BioXpCanDriver._validate_pipette_id(4)

    payload = bytes((0, 0, 0, 0x50, 3, 0x20, 0x60, ord("1")))
    framed = novo_encode(payload)
    assert novo_decode(framed) == payload
    with pytest.raises(NovoUsbCanError):
        novo_decode(b"\x00" + framed)
    with pytest.raises(NovoUsbCanError):
        novo_decode(framed + framed)
    with pytest.raises(NovoUsbCanError):
        novo_encode(payload + b"\x00")


def test_multipart_matcher_requires_explicit_opt_in_and_rebind_fences_completion():
    multipart = NovoRouter._decode_record(
        (0x513).to_bytes(4, "big") + bytes([0]), b"raw", 1.0
    )
    ordinary = NovoRouter.pipette_matcher(channel=2, expected_function=6)
    enabled = NovoRouter.pipette_matcher(channel=2, expected_function=6, allow_multipart=True)
    assert ordinary(multipart).matched is False
    assert ordinary(multipart).classification == "pipette_unexpected_multipart"
    assert enabled(multipart).matched is True

    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw)
    router.prepare_pipette_completion(2, 10.0)
    router._reader_generation += 1
    result = router.wait_pipette_completion(2, 0.0)
    assert result["ok"] is False
    assert result["generation_changed"] is True


def test_generation_matched_pipette_completion_accepts_only_oem_initialized_byte():
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw)
    router.prepare_pipette_completion(1, 10.0)
    frame = NovoFrame(
        arbitration_id=0x509,
        dlc=3,
        data=bytes((0x20, 0x60, 0)),
        raw=b"raw",
        received_at=1.0,
        classification="pipette_report",
    )
    router._dispatch(frame)
    result = router.wait_pipette_completion(1, 0.0)
    assert result["ok"] is True
    assert result["generation_changed"] is False


def test_api_init_rejects_unmapped_pressure_profile_before_readiness_gate():
    with pytest.raises(HTTPException) as failure:
        asyncio.run(api.liquid_init(api.PipetteInitRequest(pressure_profile="2R", prime_volume_ul=None)))
    assert failure.value.status_code == 400
    assert failure.value.detail == {
        "error": "validation_error",
        "field": "pressure_profile",
        "message": "only OEM pressure profile 1R is supported",
    }
