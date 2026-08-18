from __future__ import annotations

from types import SimpleNamespace
import time

import pytest

from src.bioxp.can_driver import BioXpCanDriver, process_pipette_message
from src.bioxp.novo_router import NovoFrame, NovoRouter
from src.bioxp.pipette.models import (
    PipetteDiagnosticCommand,
    PipetteHeartbeatCommand,
    PipetteInitCommand,
    PipetteTerminateCommand,
)
from src.bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport, PipetteCommandError


class _NoDriverCallsTransport:
    _tip_loaded = True
    _top_speed = 100.0
    _initialized = False
    _transport_name = "novo_usb_can"
    _transport_details = {"shared_bioxp_usb_runtime": True}

    def _get_driver(self):
        raise AssertionError("no driver call is allowed while pipette command admission is closed")

    def __getattr__(self, name: str):
        raise AssertionError(f"no transport method is allowed while pipette command admission is closed: {name}")


@pytest.mark.parametrize(
    "invoke",
    [
        lambda owner: owner.initialize(PipetteInitCommand()),
        lambda owner: owner.initiate_group_once_for_oem_initialize_motion(cycle="initializeMotion.initial"),
        lambda owner: owner.reinitialize_pipette(),
        lambda owner: owner.set_top_speed(200.0),
        lambda owner: owner.execute_diagnoses(PipetteDiagnosticCommand(number=1)),
        lambda owner: owner.terminate(PipetteTerminateCommand()),
        lambda owner: owner.heartbeat(PipetteHeartbeatCommand(enabled=True)),
        lambda owner: owner.eject_all_tips(check_missing_tip=False),
        lambda owner: owner.eject_all_tips_for_oem_startup(
            operator_ack="EJECT_STALE_STARTUP_TIPS",
            expected_channels_with_tips=[0],
        ),
        lambda owner: owner.KeepTip(0),
        lambda owner: owner.mix_all(1, 10.0),
    ],
)
def test_closed_production_owner_rejects_every_command_before_driver_io(invoke):
    owner = FourPipetteTransport([_NoDriverCallsTransport() for _ in range(4)])

    with pytest.raises(PipetteCommandError) as exc:
        invoke(owner)

    assert exc.value.details["physical_command_admitted"] is False
    assert exc.value.details["physical_effect_verified"] is False


def test_collection_selection_requires_explicit_channels_or_selection_mode():
    collection = FourPipetteTransport([_NoDriverCallsTransport() for _ in range(4)], liquid_mutation_enabled=True)

    with pytest.raises(PipetteCommandError, match="selection"):
        collection._channels_from_metadata(None)

    assert collection._channels_from_metadata({"channels": [1, 3]}) == [1, 3]
    with pytest.raises(PipetteCommandError, match="unresolved"):
        collection._channels_from_metadata({"selection_mode": "tip_location"})


def test_cached_tip_selection_requires_semantic_hardware_readback():
    class Transport(_NoDriverCallsTransport):
        def __init__(self, status):
            super().__init__()
            self._last_tip_status = status
            self._reader_generation = 7

        def query_pressure(self):
            return {"ok": True, "semantic_ok": True, "pressure": 12.0}

    observed_at = time.time()
    transports = [
        Transport(None),
        Transport({"ok": True, "semantic_ok": False, "hardware_truth_level": "hardware_query", "tip_loaded": True}),
        Transport({
            "ok": True,
            "semantic_ok": True,
            "hardware_truth_level": "hardware_query",
            "tip_loaded": True,
            "observed_at": observed_at,
            "reader_generation": 7,
        }),
        Transport({"ok": True, "semantic_ok": True, "hardware_truth_level": "software_shadow", "tip_loaded": True}),
    ]
    collection = FourPipetteTransport(transports, liquid_mutation_enabled=True)

    result = collection.read_pressure()

    assert result["ok"] is True
    assert [row["channel"] for row in result["channels"]] == [2]
    assert result["eligibility_source"] == "semantic_hardware_tip_readback"
    assert result["eligibility"][2]["eligible"] is True

    transports[2]._last_tip_status["observed_at"] = time.time() - 60.0
    stale = collection.read_pressure()
    assert stale["ok"] is False
    assert stale["eligibility"][2]["reason"] == "stale_tip_observation"

    transports[2]._last_tip_status["observed_at"] = time.time()
    transports[2]._reader_generation = 8
    rebound = collection.read_pressure()
    assert rebound["ok"] is False
    assert rebound["eligibility"][2]["reason"] == "reader_generation_mismatch"

    no_eligible_speed = collection.set_top_speed(123.0)
    assert no_eligible_speed["ok"] is False
    assert no_eligible_speed["outcome"] == "no_eligible_channels"

    no_eligible_diagnosis = collection.execute_diagnoses(PipetteDiagnosticCommand(number=1))
    assert no_eligible_diagnosis["ok"] is False
    assert no_eligible_diagnosis["outcome"] == "no_eligible_channels"


def test_tip_cache_uses_live_router_generation_and_invalidates_after_rebind():
    class Driver:
        def __init__(self, router):
            self.bus = SimpleNamespace(router=router)

        def query_tip_status(self):
            return {
                "ok": True,
                "semantic_ok": True,
                "hardware_truth_level": "hardware_query",
                "tip_loaded": True,
                "provenance": {"owner_generation": self.bus.router.reader_generation},
            }

    class Transport(_NoDriverCallsTransport):
        def __init__(self, driver):
            self._driver = driver
            self._last_tip_status = None
            self._reader_generation = None

        def _get_driver(self):
            return self._driver

    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw)
    router._reader_generation = 7
    transport = Transport(Driver(router))
    collection = FourPipetteTransport([transport] + [_NoDriverCallsTransport() for _ in range(3)])

    CanPipetteTransport._safe_query_tip_status(transport, transport._driver, required=True)
    eligible, ledger = collection._tip_eligibility([0])
    assert eligible == [0]
    assert ledger[0]["reader_generation"] == 7
    assert ledger[0]["current_reader_generation"] == 7

    router._reader_generation = 8
    eligible, ledger = collection._tip_eligibility([0])
    assert eligible == []
    assert ledger[0]["reason"] == "reader_generation_mismatch"


def test_aggregate_get_data_rejects_semantically_invalid_or_incomplete_sweep():
    class Transport(_NoDriverCallsTransport):
        def get_all_data(self, queries):
            return {
                "ok": True,
                "semantic_ok": False,
                "queries": list(queries),
                "results": [{"ok": True, "semantic_ok": False}],
            }

    collection = FourPipetteTransport([Transport() for _ in range(4)])

    result = collection.get_data()

    assert result["ok"] is False
    assert result["semantic_ok"] is False
    assert result["expected_channel_count"] == 4
    assert result["expected_query_count"] == len(collection.OEM_DATA_QUERIES)


def test_four_channel_live_readback_is_semantic_and_mutation_free(monkeypatch):
    class ReadbackTransport(_NoDriverCallsTransport):
        def __init__(self, channel):
            self.channel = channel
            self._last_tip_status = None
            self.get_all_data_wake_if_needed = None

        def _get_driver(self):
            return self

        def _safe_query_tip_status(self, driver, *, required=False):
            return CanPipetteTransport._safe_query_tip_status(self, driver, required=required)

        def query_firmware(self, number):
            return {"ok": True, "semantic_ok": True, "number": number}

        def query_status(self):
            return {"ok": True, "semantic_ok": True, "oem_process_error_code": 32}

        def query_tip_status(self):
            result = {
                "ok": True,
                "semantic_ok": True,
                "hardware_truth_level": "hardware_query",
                "tip_loaded": self.channel in {0, 2},
            }
            self._last_tip_status = result
            return result

        def query_pressure(self):
            return {"ok": True, "semantic_ok": True, "pressure": float(self.channel)}

        def get_all_data(self, queries, *, wake_if_needed=True):
            self.get_all_data_wake_if_needed = wake_if_needed
            return {
                "ok": True,
                "semantic_ok": True,
                "queries": list(queries),
                "expected_query_count": len(queries),
                "data": [
                    {"query": query, "result": {"ok": True, "semantic_ok": True, "query": query}}
                    for query in queries
                ],
            }

    owner = FourPipetteTransport([ReadbackTransport(channel) for channel in range(4)])

    result = owner.readback_all(include_data=False)

    assert result["ok"] is True
    assert result["live_query_performed"] is True
    assert result["truth_source"] == "live_hardware_queries"
    assert [row["channel"] for row in result["channels"]] == [0, 1, 2, 3]
    assert [row["pressure"] is not None for row in result["channels"]] == [True, False, True, False]
    assert result["physical_effect_verified"] is False

    with_data = owner.readback_all(include_data=True)
    assert with_data["ok"] is True
    assert all(transport.get_all_data_wake_if_needed is False for transport in owner._transports)

    from fastapi.testclient import TestClient
    from src.bioxp import api

    monkeypatch.setattr(api, "_pipette_transport", owner)
    response = TestClient(api.app).post("/liquid/readback", json={"include_data": False})
    assert response.status_code == 200
    payload = response.json()
    assert payload["channel_count"] == 4
    assert isinstance(payload["receipt_id"], str)
    assert payload["receipt_truth"]["physical_effect_verified"] is False


def test_oem_process_message_accepts_exact_dlc2_completion_and_uses_data_zero_error_code():
    state = process_pipette_message(
        2,
        [0x20, 0xA5],
        arbitration_id=0x501,
        command_name="pipette_initialize",
    )

    assert state["ok"] is True
    assert state["completion_signal"] is True
    assert state["oem_error_code"] == 0x20
    assert state["message"] == [0x01, 0x05, 0x20, 0xA5]


def test_query_multipart_requires_first_middle_final_order_and_driver_opt_in():
    unordered = NovoRouter.pipette_matcher(
        channel=0,
        expected_function=6,
        allow_multipart=True,
    )
    middle_before_first = unordered(_frame(0x504, b"middle", 1.0))
    assert middle_before_first.matched is False
    assert middle_before_first.classification == "pipette_multipart_wrong_order"

    ordered = NovoRouter.pipette_matcher(
        channel=0,
        expected_function=6,
        allow_multipart=True,
    )
    assert ordered(_frame(0x503, b"first", 1.0)).matched is True
    assert ordered(_frame(0x504, b"middle", 1.1)).matched is True
    final = ordered(_frame(0x506, bytes([0x20, 0x60, 0x31]), 1.2))
    assert final.matched is True
    assert final.terminal is True

    class Bus:
        kwargs = None

        def transact_can(self, message, **kwargs):
            del message
            self.kwargs = kwargs
            return {
                "ok": True,
                "outcome": "completion",
                "frames": [{
                    "arbitration_id": 0x506,
                    "dlc": 3,
                    "data": [0x20, 0x60, 0x31],
                    "raw": [],
                }],
            }

    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver.bus = Bus()
    driver.pipette_id = 0
    driver.response_timeout_s = 1.0
    driver._pipette_message_state = {}
    driver._pipette_last_command = None
    driver._pipette_error_callback = None
    result = driver._send_packet(
        0x106,
        list(b"?31"),
        require_ack=True,
        ack_mode="query",
        command_name="query_tip_status",
    )
    assert result["ok"] is True
    assert driver.bus.kwargs["allow_multipart"] is True


def test_get_data_requires_exact_query_identity_prefix_and_nonempty_ascii_value():
    class Bus:
        response_data = [0x20, 0x60]

        def transact_can(self, message, **kwargs):
            del message, kwargs
            return {
                "ok": True,
                "outcome": "completion",
                "completion_received": True,
                "frames": [{
                    "arbitration_id": 0x506,
                    "dlc": len(self.response_data),
                    "data": list(self.response_data),
                    "raw": [],
                }],
            }

    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver.bus = Bus()
    driver.pipette_id = 0
    driver.response_timeout_s = 1.0
    driver._pipette_message_state = {}
    driver._pipette_last_command = None
    driver._pipette_error_callback = None

    empty = driver.get_data("?40", wake_if_needed=False)
    assert empty["ok"] is False
    assert empty["semantic_ok"] is False
    assert empty["completion_verified"] is False
    assert empty["semantic_query_response_verified"] is True

    driver.bus.response_data = [0x20, 0x60, 0xFF]
    non_ascii = driver.get_data("?40", wake_if_needed=False)
    assert non_ascii["ok"] is False

    driver.bus.response_data = [0x20, 0x60, ord("4"), ord("2")]
    valid = driver.get_data("?40", wake_if_needed=False)
    assert valid["ok"] is True
    assert valid["query"] == "?40"
    assert valid["value"] == "42"


def test_completion_owner_enforces_exact_tx_derived_rx_id():
    clock = [1.0]
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw, clock=lambda: clock[0])
    token = router.prepare_pipette_completion(
        channel=0,
        command_family=1,
        command_name="aspirate",
        expected_rx_id=0x501,
        timeout_s=1.0,
    )
    router.bind_pipette_completion(
        channel=0,
        owner_token=token,
        transaction_id="tx-exact-rx",
        tx_started_at=1.0,
    )
    router._dispatch(_frame(0x509, b"", 1.1))
    router._dispatch(_frame(0x509, bytes([0x20, 0x00]), 1.2))
    clock[0] = 2.1
    result = router.wait_pipette_completion(channel=0, owner_token=token, timeout_s=0.0)
    assert result["ok"] is False
    assert result["expected_rx_id"] == 0x501


def test_non_novo_command_fallback_cannot_collapse_ack_and_completion():
    class Bus:
        def send(self, message):
            del message

        def recv(self, timeout):
            del timeout
            return SimpleNamespace(
                arbitration_id=0x501,
                data=bytes([0x20, 0x00]),
                dlc=2,
                timestamp=1.0,
            )

    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver.bus = Bus()
    driver.pipette_id = 0
    driver.response_timeout_s = 0.01
    driver._pipette_message_state = {}
    driver._pipette_last_command = None
    driver._pipette_error_callback = None

    result = driver._send_packet(
        0x101,
        list(b"E1R"),
        require_ack=True,
        ack_mode="command",
        command_name="pipette_eject_tip",
    )

    assert result["ok"] is False
    assert result["delivery_verified"] is True
    assert result["controller_acknowledged"] is False
    assert result["completion_verified"] is False
    assert result["error"] == "shared_novo_router_required_for_command_lifecycle"


def test_oem_process_message_keeps_immediate_ack_separate_from_completion():
    state = process_pipette_message(
        0,
        [],
        arbitration_id=0x501,
        command_name="pipette_initialize",
    )

    assert state["ok"] is True
    assert state["immediate_ack_received"] is True
    assert state["completion_signal"] is False
    assert state["reply_presence"] == "immediate_ack"


def _frame(can_id: int, data: bytes, at: float) -> NovoFrame:
    return NovoFrame(
        arbitration_id=can_id,
        dlc=len(data),
        data=data,
        raw=b"",
        received_at=at,
        classification="pipette",
    )


def test_completion_owner_enforces_family_ack_order_exact_dlc_and_freezes_first_terminal():
    clock = [1.0]
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw, clock=lambda: clock[0])
    router._reader_generation = 4
    token = router.prepare_pipette_completion(
        0,
        10.0,
        command_family=1,
        command_name="pipette_initialize",
    )
    router.bind_pipette_completion(0, owner_token=token, transaction_id="tx-1", tx_started_at=1.0)

    router._dispatch(_frame(0x500, b"", 1.1))
    router._dispatch(_frame(0x500, bytes([0x20, 0x01]), 1.2))
    assert not router._pipette_completions[0].event.is_set()

    router._dispatch(_frame(0x501, b"", 1.4))
    router._dispatch(_frame(0x501, bytes([0x20]), 1.5))
    assert not router._pipette_completions[0].event.is_set()

    router._dispatch(_frame(0x501, bytes([0x20, 0xA5]), 1.6))
    router._dispatch(_frame(0x501, bytes([0x20, 0xFF]), 1.7))
    result = router.wait_pipette_completion(0, 0.0, owner_token=token)

    assert result["ok"] is True
    assert result["command_family"] == 1
    assert result["observed_rx_id"] == 0x501
    assert result["observed_rx_dlc"] == 2
    assert result["data"] == [0x20, 0xA5]
    assert result["immediate_ack_received"] is True
    assert result["duplicate_terminal_count"] == 1


def test_completion_before_ack_rejects_and_taints_the_lane():
    clock = [1.0]
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda raw: raw, clock=lambda: clock[0])
    token = router.prepare_pipette_completion(
        0,
        10.0,
        command_family=1,
        command_name="aspirate",
    )
    router.bind_pipette_completion(
        0,
        owner_token=token,
        transaction_id="tx-order",
        tx_started_at=1.0,
    )

    router._dispatch(_frame(0x501, bytes([0x20, 0x00]), 1.1))
    router._dispatch(_frame(0x501, b"", 1.2))
    result = router.wait_pipette_completion(0, 0.0, owner_token=token)

    assert result["ok"] is False
    assert result["outcome"] == "completion_before_ack"
    assert router.pipette_completion_taint(0)["reason"] == "completion_before_ack"


def test_completion_success_does_not_invent_controller_acknowledgement():
    from src.bioxp.pipette.transport import FourPipetteTransport

    class CompletedTransport:
        _tip_loaded = True
        _top_speed = 100.0
        _initialized = True
        _transport_name = "novo_usb_can"
        _transport_details = {"shared_bioxp_usb_runtime": True}

        def aspirate(self, *_args, **_kwargs):
            return {
                "ok": True,
                "delivery_verified": True,
                "controller_acknowledged": False,
                "completion_verified": False,
            }

        def wait_for_completion(self, _timeout_s):
            return {"ok": True, "outcome": "completion", "immediate_ack_received": False}

        def apply_completed_effect(self, _operation, _result):
            return {"state_reconciled": True, "state_reconciliation_source": "completion"}

    owner = FourPipetteTransport(
        [CompletedTransport() for _ in range(4)],
        liquid_mutation_enabled=True,
    )
    result = owner._run_group_liquid_operation(
        "aspirate",
        [0],
        lambda _channel, transport, _defer: transport.aspirate(),
        timeout_ms=100,
    )

    assert result["channels"][0]["result"]["completion_verified"] is True
    assert result["channels"][0]["result"]["controller_acknowledged"] is False
    assert result["controller_acknowledged"] is False
