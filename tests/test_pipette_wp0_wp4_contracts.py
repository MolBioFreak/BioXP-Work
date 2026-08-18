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
from src.bioxp.novo_router import NovoFrame, NovoRouter, NovoRouterError
from src.bioxp.novo_usb_can import NovoUsbCanError, novo_decode, novo_encode
from src.bioxp.pipette.models import PipetteAspirateCommand, PipetteDispenseCommand
from src.bioxp.pipette.receipts import PipetteReceiptStore
from src.bioxp.pipette.transport import CanPipetteTransport
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


def test_wp0_application_call_site_denominator_is_row_level():
    matrix = json.loads((ROOT / "docs/specs/2026-08-02-pipette-oem-parity-matrix.json").read_text())
    rows = matrix["denominator"]["application_call_sites"]
    required = {
        "id",
        "source_file",
        "source_sha256",
        "source_line",
        "source_member",
        "call_expression",
        "classification",
        "status",
    }

    assert rows
    assert all(required <= row.keys() for row in rows)
    assert all(row["id"].startswith("APP-CS-") for row in rows)
    assert all(isinstance(row["source_line"], int) and row["source_line"] > 0 for row in rows)
    assert all(len(row["source_sha256"]) == 64 for row in rows)
    assert all(row["blocker"] for row in rows if row["status"] == "blocked")
    identities = {
        (row["source_file"], row["source_line"], row["call_expression"])
        for row in rows
    }
    assert len(identities) == len(rows)
    assert matrix["matrix_invariants"]["application_call_site_count"] == len(rows)
    assert not any("inventory remains open" in row.get("blocker", "") for row in rows)


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
                "frames": [{"arbitration_id": 0x511, "dlc": 0, "data": [], "raw": []}],
                "immediate_ack_received": True,
                "completion_received": True,
                "completion": {
                    "ok": True,
                    "outcome": "completion",
                    "observed_rx_id": 0x511,
                    "observed_rx_dlc": 2,
                    "data": [0x20, 0x00],
                    "command_name": "long_command",
                },
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


def test_receipt_truth_does_not_promote_tx_only_delivery_to_controller_ack():
    truth = PipetteReceiptStore._truth(
        {
            "ok": True,
            "driver_result": {
                "ok": True,
                "tx_ok": True,
                "provenance": {"ok": True, "outcome": "tx_only", "frames": []},
            },
        }
    )

    assert truth["delivery_verified"] is True
    assert truth["controller_acknowledged"] is False
    assert truth["completion_verified"] is False
    assert truth["hardware_postcondition_verified"] is False


def test_transport_keeps_tip_precondition_separate_from_liquid_postcondition():
    class Driver:
        _pipette_message_state = {}

        @staticmethod
        def query_tip_status():
            return {
                "ok": True,
                "tip_loaded": True,
                "hardware_truth_level": "hardware_query",
            }

        @staticmethod
        def aspirate(volume_ul, *, tip_pressure_profile, wait_for_completion):
            del volume_ul, tip_pressure_profile, wait_for_completion
            return {
                "ok": True,
                "tx_ok": True,
                "delivery_verified": True,
                "controller_acknowledged": False,
                "completion_verified": False,
                "provenance": {"outcome": "tx_only", "frames": []},
            }

    transport = CanPipetteTransport(driver_factory=Driver)
    transport._initialized = True
    result = transport.aspirate(
        PipetteAspirateCommand(volume_ul=10.0),
        wait_for_completion=False,
    )

    assert result["hardware_precondition_verified"] is True
    assert result["hardware_postcondition_verified"] is False
    assert result["controller_acknowledged"] is False
    assert result["completion_verified"] is False
    assert result["state_reconciled"] is False
    assert result["liquid_level_ul"] == 0.0
    assert result["physical_effect_verified"] is False


def test_driver_deferred_send_requires_ack_without_claiming_completion():
    class Bus:
        @staticmethod
        def transact_can(message, **kwargs):
            del message, kwargs
            return {
                "ok": True,
                "outcome": "ack",
                "frames": [{"arbitration_id": 0x501, "dlc": 0, "data": [], "raw": []}],
                "immediate_ack_received": True,
                "completion_received": False,
            }

    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver.bus = Bus()
    driver.pipette_id = 0
    driver.response_timeout_s = 0.01
    driver._pipette_message_state = {}
    driver._pipette_last_command = None
    result = driver._send_pipette_command(
        "P10R1,",
        command_name="pipette_aspirate",
        wait_for_completion=False,
    )

    assert result["ok"] is True
    assert result["delivery_verified"] is True
    assert result["controller_acknowledged"] is True
    assert result["completion_verified"] is False


def test_driver_uses_oem_numeric_command_formatting():
    assert BioXpCanDriver._format_pipette_volume(100.5) == "101"

    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    captured = {}

    def send(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        return {"ok": True}

    driver.__dict__["_send_pipette_command"] = send
    result = driver.set_top_speed(12.5)

    assert captured["command"] == "V12.5,1R"
    assert result["effective_top_speed"] == 12.5


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
    token = router.prepare_pipette_completion(2, 10.0)
    router._reader_generation += 1
    result = router.wait_pipette_completion(2, 0.0, owner_token=token)
    assert result["ok"] is False
    assert result["generation_changed"] is True


def test_generation_matched_pipette_completion_accepts_only_oem_initialized_byte():
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw)
    token = router.prepare_pipette_completion(
        1,
        10.0,
        command_family=1,
        command_name="pipette_aspirate",
    )
    started = router._clock()
    router.bind_pipette_completion(
        1,
        owner_token=token,
        transaction_id="tx",
        tx_started_at=started,
    )
    router._dispatch(NovoFrame(
        arbitration_id=0x509,
        dlc=0,
        data=b"",
        raw=b"raw",
        received_at=started + 0.01,
        classification="pipette_report",
    ))
    router._dispatch(NovoFrame(
        arbitration_id=0x509,
        dlc=2,
        data=bytes((0x20, 0)),
        raw=b"raw",
        received_at=started + 0.02,
        classification="pipette_report",
    ))
    result = router.wait_pipette_completion(1, 0.0, owner_token=token)
    assert result["ok"] is True
    assert result["generation_changed"] is False


def test_pipette_completion_requires_bound_command_owner_and_matching_token():
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw)
    token = router.prepare_pipette_completion(
        1,
        10.0,
        command_family=1,
        command_name="pipette_aspirate",
    )
    router.bind_pipette_completion(
        1,
        owner_token=token,
        transaction_id="tx-1",
        tx_started_at=10.0,
    )

    mismatch = router.wait_pipette_completion(1, 0.0, owner_token="wrong-token")
    assert mismatch["ok"] is False
    assert mismatch["outcome"] == "completion_owner_mismatch"

    router._dispatch(
        NovoFrame(
            arbitration_id=0x509,
            dlc=0,
            data=b"",
            raw=b"raw",
            received_at=10.05,
            classification="pipette_report",
        )
    )
    router._dispatch(
        NovoFrame(
            arbitration_id=0x509,
            dlc=2,
            data=bytes((0x20, 0)),
            raw=b"raw",
            received_at=10.1,
            classification="pipette_report",
        )
    )
    result = router.wait_pipette_completion(1, 0.0, owner_token=token)
    assert result["ok"] is True
    assert result["owner_token"] == token
    assert result["transaction_id"] == "tx-1"
    assert result["command_name"] == "pipette_aspirate"
    assert result["command_family"] == 1


def test_completion_timeout_taints_channel_until_router_rebind():
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw)
    token = router.prepare_pipette_completion(
        1,
        10.0,
        command_family=1,
        command_name="pipette_aspirate",
    )
    router.bind_pipette_completion(
        1,
        owner_token=token,
        transaction_id="tx-timeout",
        tx_started_at=router._clock(),
    )

    timeout = router.wait_pipette_completion(1, 0.0, owner_token=token)
    assert timeout["ok"] is False
    assert timeout["outcome"] == "timeout"
    taint = router.pipette_completion_taint(1)
    assert taint is not None
    assert taint["reason"] == "completion_timeout"
    with pytest.raises(NovoRouterError, match="router rebind is required"):
        router.prepare_pipette_completion(1, 10.0, command_family=1, command_name="next")

    router.shutdown()
    assert router.pipette_completion_taint(1) is None
    router.prepare_pipette_completion(1, 10.0, command_family=1, command_name="next")


def test_wait_pipette_completion_rejects_missing_token_without_mutating_owner():
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw)
    token = router.prepare_pipette_completion(
        0,
        10.0,
        command_family=0,
        command_name="pipette_initialize",
    )
    router.bind_pipette_completion(
        0,
        owner_token=token,
        transaction_id="tx-token-required",
        tx_started_at=router._clock(),
    )

    rejected = router.wait_pipette_completion(0, 0.0)
    assert rejected["ok"] is False
    assert rejected["outcome"] == "completion_token_required"
    with router._completion_lock:
        assert 0 in router._pipette_completions

    rejected_empty = router.wait_pipette_completion(0, 0.0, owner_token="")
    assert rejected_empty["ok"] is False
    assert rejected_empty["outcome"] == "completion_token_required"
    with router._completion_lock:
        assert 0 in router._pipette_completions

    mismatch = router.wait_pipette_completion(0, 0.0, owner_token="wrong-token")
    assert mismatch["ok"] is False
    assert mismatch["outcome"] == "completion_owner_mismatch"
    with router._completion_lock:
        assert 0 in router._pipette_completions

    owned = router.wait_pipette_completion(0, 0.0, owner_token=token)
    assert owned["ok"] is False
    assert owned["outcome"] == "timeout"
    with router._completion_lock:
        assert 0 not in router._pipette_completions


def test_api_init_rejects_unmapped_pressure_profile_before_readiness_gate():
    with pytest.raises(HTTPException) as failure:
        asyncio.run(api.liquid_init(api.PipetteInitRequest(pressure_profile="2R", prime_volume_ul=None)))
    assert failure.value.status_code == 400
    assert failure.value.detail == {
        "error": "validation_error",
        "message": "Only OEM-backed pressure profile 1R is admitted.",
        "physical_motion_commanded": False,
    }
