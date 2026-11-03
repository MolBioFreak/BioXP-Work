"""Connected eject receipt regression; hardware returns replay retained incident evidence.

Source c020bd5, serial206 canonical pipette_operations.receipt_json.result:
release: pipette_e1a1dc9f2ca14b419372ed306d820800
 diagnostic: pipette_f23fd22b80074c41bc26a029a86bef02
Only producer ok/channels/sends fields retained; service-owned callback and
collection snapshots excluded. Negative variants below are deliberate mutations,
not claims about incident hardware. No transport or truth owner is mocked.
"""
import asyncio
import copy
import json

import pytest
from fastapi import HTTPException
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.models import PipetteCommandError
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.services.pipette_service import run_pipette_operation

RETAINED = json.loads(r'''
{
  "release_unneeded_tips": {
    "channels": [
      0,
      1,
      3
    ],
    "ok": true,
    "sends": [
      {
        "channel": 0,
        "result": {
          "ack": {
            "arbitration_id": 1281,
            "ascii": "",
            "completion": null,
            "dlc": 0,
            "immediate_ack": {
              "arbitration_id": 1281,
              "classification": "pipette",
              "dlc": 0,
              "owner_generation": 1,
              "receive_owner": "9b1618b134a843a0acb041f0749222bd",
              "receive_sequence": 1756,
              "received_at": 437674.355066714
            },
            "ok": true,
            "outcome": "ack",
            "received": true
          },
          "ack_required": true,
          "ascii_command": "E1R",
          "board_id": 257,
          "command_name": "pipette_eject_tip",
          "completion_deferred": false,
          "completion_verified": true,
          "controller_acknowledged": true,
          "delivery_verified": true,
          "dlc": 3,
          "error": null,
          "immediate_ack_received": true,
          "length": 3,
          "ok": true,
          "payload": [
            69,
            49,
            82
          ],
          "pipette_message_state": {
            "command_name": "pipette_eject_tip",
            "completion_signal": true,
            "diagnosis": null,
            "error_code": 32,
            "error_queue": [],
            "event_error_code": 32,
            "fluid_timestamp": null,
            "immediate_ack_received": true,
            "initialization_counter": 0,
            "initialized": true,
            "message": [
              1,
              5,
              32,
              96
            ],
            "oem_error_code": 32,
            "ok": true,
            "reply_presence": "nonempty",
            "tip_loaded": true,
            "tip_lost_suppressed_for_eject": false,
            "tip_source_actor": "20028a6f72834af8bc3812b11b28548a",
            "tip_source_reader": 129320104214400,
            "tip_source_reader_generation": 1,
            "tip_source_revision": 10,
            "tip_source_transaction_id": "749a0770c6f74979be498ae8cd220802",
            "tip_source_verified": true
          },
          "provenance": {
            "ack_received": true,
            "channel": 0,
            "command_family": 1,
            "completion": {
              "channel": 0,
              "command_family": 1,
              "command_name": "pipette_eject_tip",
              "deadline_timestamp": 437734.353393787,
              "duplicate_terminal_count": 0,
              "expected_rx_id": 1281,
              "generation_changed": false,
              "immediate_ack": {
                "arbitration_id": 1281,
                "classification": "pipette",
                "dlc": 0,
                "owner_generation": 1,
                "receive_owner": "9b1618b134a843a0acb041f0749222bd",
                "receive_sequence": 1756,
                "received_at": 437674.355066714
              },
              "immediate_ack_received": true,
              "observed_rx_dlc": 2,
              "observed_rx_id": 1281,
              "ok": true,
              "outcome": "completion",
              "owner_generation": 1,
              "owner_token": "70584564aca04c5bb80ecfcc3a5152ed",
              "receive_timestamp": 437676.896099064,
              "registration_timestamp": 437674.353393787,
              "transaction_id": "17b944f089384aa69826caa9b0ce4834",
              "tx_started_at": 437674.353703175
            },
            "completion_deferred": false,
            "completion_owner_token": "70584564aca04c5bb80ecfcc3a5152ed",
            "completion_received": true,
            "completion_timeout_ms": 60000,
            "controller_acknowledged": true,
            "expected_rx_id": 1281,
            "immediate_ack_received": true,
            "matcher": "pipette_eject_tip",
            "multipart_received": false,
            "observed_rx_dlc": 0,
            "observed_rx_id": 1281,
            "oem_receive_queue_cleared": 0,
            "ok": true,
            "outcome": "completion",
            "owner_generation": 1,
            "receive_timestamp": 437674.355066714,
            "registration_timestamp": 437674.35367362,
            "skipped_count": 0,
            "skipped_frames_truncated": false,
            "timeout_ms": 60000,
            "transaction_id": "17b944f089384aa69826caa9b0ce4834",
            "tx_data": [
              69,
              49,
              82
            ],
            "tx_dlc": 3,
            "tx_id": 257,
            "tx_timestamp": 437674.353703175,
            "tx_write_completed_at": 437674.35584703,
            "wait_policy": {
              "classification": "OEM-DIRECT",
              "mode": "oem_synchronous_send_wait",
              "source_anchor": "ClassNovo.TransmitMessage; ClassNovoCANUSB.sendCommand"
            }
          },
          "query_response_correlated": false,
          "semantic_query_response_verified": false,
          "tx_ok": true
        }
      },
      {
        "channel": 1,
        "result": {
          "ack": {
            "arbitration_id": 1289,
            "ascii": "",
            "completion": null,
            "dlc": 0,
            "immediate_ack": {
              "arbitration_id": 1289,
              "classification": "pipette",
              "dlc": 0,
              "owner_generation": 1,
              "receive_owner": "9b1618b134a843a0acb041f0749222bd",
              "receive_sequence": 1765,
              "received_at": 437676.900760659
            },
            "ok": true,
            "outcome": "ack",
            "received": true
          },
          "ack_required": true,
          "ascii_command": "E1R",
          "board_id": 265,
          "command_name": "pipette_eject_tip",
          "completion_deferred": false,
          "completion_verified": true,
          "controller_acknowledged": true,
          "delivery_verified": true,
          "dlc": 3,
          "error": null,
          "immediate_ack_received": true,
          "length": 3,
          "ok": true,
          "payload": [
            69,
            49,
            82
          ],
          "pipette_message_state": {
            "command_name": "pipette_eject_tip",
            "completion_signal": true,
            "diagnosis": null,
            "error_code": 32,
            "error_queue": [],
            "event_error_code": 32,
            "fluid_timestamp": null,
            "immediate_ack_received": true,
            "initialization_counter": 0,
            "initialized": true,
            "message": [
              9,
              5,
              32,
              96
            ],
            "oem_error_code": 32,
            "ok": true,
            "reply_presence": "nonempty",
            "tip_loaded": true,
            "tip_lost_suppressed_for_eject": false,
            "tip_source_actor": "9fdac3c25e664b70bc4bee7acd4d5605",
            "tip_source_reader": 129320104214400,
            "tip_source_reader_generation": 1,
            "tip_source_revision": 10,
            "tip_source_transaction_id": "e790210eafc24de1aa04179fbc12c483",
            "tip_source_verified": true
          },
          "provenance": {
            "ack_received": true,
            "channel": 1,
            "command_family": 1,
            "completion": {
              "channel": 1,
              "command_family": 1,
              "command_name": "pipette_eject_tip",
              "deadline_timestamp": 437736.898401664,
              "duplicate_terminal_count": 0,
              "expected_rx_id": 1289,
              "generation_changed": false,
              "immediate_ack": {
                "arbitration_id": 1289,
                "classification": "pipette",
                "dlc": 0,
                "owner_generation": 1,
                "receive_owner": "9b1618b134a843a0acb041f0749222bd",
                "receive_sequence": 1765,
                "received_at": 437676.900760659
              },
              "immediate_ack_received": true,
              "observed_rx_dlc": 2,
              "observed_rx_id": 1289,
              "ok": true,
              "outcome": "completion",
              "owner_generation": 1,
              "owner_token": "2b55bcfe71e740a8945fbd545a9025ae",
              "receive_timestamp": 437679.466858134,
              "registration_timestamp": 437676.898401664,
              "transaction_id": "0679bcc0f8a7408aa3ec2d5462462683",
              "tx_started_at": 437676.898761476
            },
            "completion_deferred": false,
            "completion_owner_token": "2b55bcfe71e740a8945fbd545a9025ae",
            "completion_received": true,
            "completion_timeout_ms": 60000,
            "controller_acknowledged": true,
            "expected_rx_id": 1289,
            "immediate_ack_received": true,
            "matcher": "pipette_eject_tip",
            "multipart_received": false,
            "observed_rx_dlc": 0,
            "observed_rx_id": 1289,
            "oem_receive_queue_cleared": 0,
            "ok": true,
            "outcome": "completion",
            "owner_generation": 1,
            "receive_timestamp": 437676.900760659,
            "registration_timestamp": 437676.898725478,
            "skipped_count": 0,
            "skipped_frames_truncated": false,
            "timeout_ms": 60000,
            "transaction_id": "0679bcc0f8a7408aa3ec2d5462462683",
            "tx_data": [
              69,
              49,
              82
            ],
            "tx_dlc": 3,
            "tx_id": 265,
            "tx_timestamp": 437676.898761476,
            "tx_write_completed_at": 437676.900159282,
            "wait_policy": {
              "classification": "OEM-DIRECT",
              "mode": "oem_synchronous_send_wait",
              "source_anchor": "ClassNovo.TransmitMessage; ClassNovoCANUSB.sendCommand"
            }
          },
          "query_response_correlated": false,
          "semantic_query_response_verified": false,
          "tx_ok": true
        }
      },
      {
        "channel": 3,
        "result": {
          "ack": {
            "arbitration_id": 1305,
            "ascii": "",
            "completion": null,
            "dlc": 0,
            "immediate_ack": {
              "arbitration_id": 1305,
              "classification": "pipette",
              "dlc": 0,
              "owner_generation": 1,
              "receive_owner": "9b1618b134a843a0acb041f0749222bd",
              "receive_sequence": 1771,
              "received_at": 437679.472482626
            },
            "ok": true,
            "outcome": "ack",
            "received": true
          },
          "ack_required": true,
          "ascii_command": "E1R",
          "board_id": 281,
          "command_name": "pipette_eject_tip",
          "completion_deferred": false,
          "completion_verified": true,
          "controller_acknowledged": true,
          "delivery_verified": true,
          "dlc": 3,
          "error": null,
          "immediate_ack_received": true,
          "length": 3,
          "ok": true,
          "payload": [
            69,
            49,
            82
          ],
          "pipette_message_state": {
            "command_name": "pipette_eject_tip",
            "completion_signal": true,
            "diagnosis": null,
            "error_code": 32,
            "error_queue": [],
            "event_error_code": 32,
            "fluid_timestamp": null,
            "immediate_ack_received": true,
            "initialization_counter": 0,
            "initialized": true,
            "message": [
              25,
              5,
              32,
              96
            ],
            "oem_error_code": 32,
            "ok": true,
            "reply_presence": "nonempty",
            "tip_loaded": true,
            "tip_lost_suppressed_for_eject": false,
            "tip_source_actor": "8c225e68518d47dfab65512cf3df0f22",
            "tip_source_reader": 129320104214400,
            "tip_source_reader_generation": 1,
            "tip_source_revision": 10,
            "tip_source_transaction_id": "146eeda4d59d420f88f428cf6444638b",
            "tip_source_verified": true
          },
          "provenance": {
            "ack_received": true,
            "channel": 3,
            "command_family": 1,
            "completion": {
              "channel": 3,
              "command_family": 1,
              "command_name": "pipette_eject_tip",
              "deadline_timestamp": 437739.470085306,
              "duplicate_terminal_count": 0,
              "expected_rx_id": 1305,
              "generation_changed": false,
              "immediate_ack": {
                "arbitration_id": 1305,
                "classification": "pipette",
                "dlc": 0,
                "owner_generation": 1,
                "receive_owner": "9b1618b134a843a0acb041f0749222bd",
                "receive_sequence": 1771,
                "received_at": 437679.472482626
              },
              "immediate_ack_received": true,
              "observed_rx_dlc": 2,
              "observed_rx_id": 1305,
              "ok": true,
              "outcome": "completion",
              "owner_generation": 1,
              "owner_token": "16d9449aa6b34278a14e0049219b0104",
              "receive_timestamp": 437681.980463884,
              "registration_timestamp": 437679.470085306,
              "transaction_id": "ead56f802c824841ad0a60a7036708b3",
              "tx_started_at": 437679.47060177
            },
            "completion_deferred": false,
            "completion_owner_token": "16d9449aa6b34278a14e0049219b0104",
            "completion_received": true,
            "completion_timeout_ms": 60000,
            "controller_acknowledged": true,
            "expected_rx_id": 1305,
            "immediate_ack_received": true,
            "matcher": "pipette_eject_tip",
            "multipart_received": false,
            "observed_rx_dlc": 0,
            "observed_rx_id": 1305,
            "oem_receive_queue_cleared": 0,
            "ok": true,
            "outcome": "completion",
            "owner_generation": 1,
            "receive_timestamp": 437679.472482626,
            "registration_timestamp": 437679.470547396,
            "skipped_count": 0,
            "skipped_frames_truncated": false,
            "timeout_ms": 60000,
            "transaction_id": "ead56f802c824841ad0a60a7036708b3",
            "tx_data": [
              69,
              49,
              82
            ],
            "tx_dlc": 3,
            "tx_id": 281,
            "tx_timestamp": 437679.47060177,
            "tx_write_completed_at": 437679.471805695,
            "wait_policy": {
              "classification": "OEM-DIRECT",
              "mode": "oem_synchronous_send_wait",
              "source_anchor": "ClassNovo.TransmitMessage; ClassNovoCANUSB.sendCommand"
            }
          },
          "query_response_correlated": false,
          "semantic_query_response_verified": false,
          "tx_ok": true
        }
      }
    ]
  },
  "eject_tip": {
    "channels": [
      {
        "channel": 1,
        "result": {
          "ack": {
            "arbitration_id": 1289,
            "ascii": "",
            "completion": null,
            "dlc": 0,
            "immediate_ack": {
              "arbitration_id": 1289,
              "classification": "pipette",
              "dlc": 0,
              "owner_generation": 1,
              "receive_owner": "9b1618b134a843a0acb041f0749222bd",
              "receive_sequence": 1686,
              "received_at": 437660.763589439
            },
            "ok": true,
            "outcome": "ack",
            "received": true
          },
          "ack_required": true,
          "ascii_command": "E1R",
          "board_id": 265,
          "command_name": "pipette_eject_tip",
          "completion_deferred": false,
          "completion_verified": true,
          "controller_acknowledged": true,
          "delivery_verified": true,
          "dlc": 3,
          "error": null,
          "immediate_ack_received": true,
          "length": 3,
          "ok": true,
          "payload": [
            69,
            49,
            82
          ],
          "pipette_message_state": {
            "command_name": "pipette_eject_tip",
            "completion_signal": true,
            "diagnosis": null,
            "error_code": 32,
            "error_queue": [],
            "event_error_code": 32,
            "fluid_timestamp": null,
            "immediate_ack_received": true,
            "initialization_counter": 0,
            "initialized": true,
            "message": [
              9,
              5,
              32,
              96
            ],
            "oem_error_code": 32,
            "ok": true,
            "reply_presence": "nonempty",
            "tip_loaded": true,
            "tip_lost_suppressed_for_eject": false,
            "tip_source_actor": "9fdac3c25e664b70bc4bee7acd4d5605",
            "tip_source_reader": 129320104214400,
            "tip_source_reader_generation": 1,
            "tip_source_revision": 8,
            "tip_source_transaction_id": "bd77ab6820e34b3db4583b0e05fd4187",
            "tip_source_verified": true
          },
          "provenance": {
            "ack_received": true,
            "channel": 1,
            "command_family": 1,
            "completion": {
              "channel": 1,
              "command_family": 1,
              "command_name": "pipette_eject_tip",
              "deadline_timestamp": 437720.759262312,
              "duplicate_terminal_count": 0,
              "expected_rx_id": 1289,
              "generation_changed": false,
              "immediate_ack": {
                "arbitration_id": 1289,
                "classification": "pipette",
                "dlc": 0,
                "owner_generation": 1,
                "receive_owner": "9b1618b134a843a0acb041f0749222bd",
                "receive_sequence": 1686,
                "received_at": 437660.763589439
              },
              "immediate_ack_received": true,
              "observed_rx_dlc": 2,
              "observed_rx_id": 1289,
              "ok": true,
              "outcome": "completion",
              "owner_generation": 1,
              "owner_token": "0cd2502864bd495fb228f50c4e2d5001",
              "receive_timestamp": 437663.335660988,
              "registration_timestamp": 437660.759262312,
              "transaction_id": "7fa1f8567bc44026aee6b6d40499539a",
              "tx_started_at": 437660.760134509
            },
            "completion_deferred": false,
            "completion_owner_token": "0cd2502864bd495fb228f50c4e2d5001",
            "completion_received": true,
            "completion_timeout_ms": 60000,
            "controller_acknowledged": true,
            "expected_rx_id": 1289,
            "immediate_ack_received": true,
            "matcher": "pipette_eject_tip",
            "multipart_received": false,
            "observed_rx_dlc": 0,
            "observed_rx_id": 1289,
            "oem_receive_queue_cleared": 0,
            "ok": true,
            "outcome": "completion",
            "owner_generation": 1,
            "receive_timestamp": 437660.763589439,
            "registration_timestamp": 437660.760092368,
            "skipped_count": 0,
            "skipped_frames_truncated": false,
            "timeout_ms": 60000,
            "transaction_id": "7fa1f8567bc44026aee6b6d40499539a",
            "tx_data": [
              69,
              49,
              82
            ],
            "tx_dlc": 3,
            "tx_id": 265,
            "tx_timestamp": 437660.760134509,
            "tx_write_completed_at": 437660.761900194,
            "wait_policy": {
              "classification": "OEM-DIRECT",
              "mode": "oem_synchronous_send_wait",
              "source_anchor": "ClassNovo.TransmitMessage; ClassNovoCANUSB.sendCommand"
            }
          },
          "query_response_correlated": false,
          "semantic_query_response_verified": false,
          "tx_ok": true
        }
      }
    ],
    "ok": true
  }
}
''')
FLAGS = ("delivery_verified", "controller_acknowledged", "completion_verified")

@pytest.fixture
def store(tmp_path, monkeypatch):
    # Offline claim identity only; neither receipt owner nor hardware truth is replaced.
    import bioxp.pipette.receipts as receipts
    monkeypatch.setattr(receipts, "current_release_identity", lambda: {
        "verified": True, "release_id": "offline-eject-receipts",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64}})
    monkeypatch.setattr(receipts, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True, "evidence_lock_sha256": "3" * 64})
    monkeypatch.setattr(receipts, "current_registry_sha256", lambda: "4" * 64)
    OEMRuntimeStore(tmp_path).close()
    owner = PipetteReceiptStore(root=tmp_path)
    yield owner
    owner.connection.close()


def run(store, operation, result, *, failure=None, transport_call=None):
    calls = []
    def leaf(_):
        calls.append(operation)
        if transport_call is not None:
            return transport_call()
        if failure is not None:
            failure.pipette_partial_result = result
            raise failure
        return result
    async def inline(label, call, *, timeout_s):
        return call()
    try:
        returned = asyncio.run(run_pipette_operation(
            operation, leaf, get_transport=lambda: object(), run_blocking=inline,
            receipt_store=store, requested_inputs={},
            runtime_binding={"idempotency_key": "offline-eject-receipt"}))
    except HTTPException as exc:
        if failure is None:
            raise
        returned = exc
    assert calls == [operation]
    # Read both canonical owners, not just the service's in-memory receipt.
    pipette = dict(store.connection.execute("SELECT * FROM pipette_operations").fetchone())
    command = dict(store.connection.execute(
        "SELECT * FROM operator_commands WHERE command_id=?", (pipette["command_id"],)).fetchone())
    for flag in FLAGS:
        assert pipette[flag] == command[flag]
    assert pipette["status"] == command["status"]
    return returned, pipette, json.loads(pipette["receipt_json"])


@pytest.mark.parametrize("operation", list(RETAINED))
def test_retained_completed_eject_reaches_both_stores(store, operation):
    original = copy.deepcopy(RETAINED[operation])
    returned, row, receipt = run(store, operation, copy.deepcopy(original))
    assert row["status"] == "completed"
    assert tuple(row[f] for f in FLAGS) == (1, 1, 1)
    assert all(returned["receipt_truth"][f] is True for f in FLAGS)
    assert receipt["truth"]["physical_effect_verified"] is False
    for key in original:
        assert returned[key] == original[key]


@pytest.mark.parametrize("variant,expected,status", [
    ("ack_only", (1, 1, 0), "acknowledged"),
    ("deferred", (1, 1, 0), "acknowledged"),
    ("tx_only", (1, 0, 0), "dispatched"),
    ("ok_only", (0, 0, 0), "failed"),
    ("missing_channel", (0, 0, 0), "failed"),
    ("empty", (0, 0, 0), "failed"),
    ("explicit_false", (0, 0, 0), "failed"),
    ("one_unverified", (1, 1, 0), "acknowledged"),
    ("deferred_conflict", (1, 1, 0), "acknowledged"),
    ("source_failure", (0, 0, 0), "failed"),
    ("wait_ok_only", (1, 1, 0), "acknowledged"),
])
def test_no_completion_inferred_from_ok(store, variant, expected, status):
    result = copy.deepcopy(RETAINED["release_unneeded_tips"])
    for send in result["sends"]:
        child = send["result"]
        if variant in {"ack_only", "tx_only", "deferred"}:
            child["completion_verified"] = False
        if variant == "tx_only":
            child["controller_acknowledged"] = False
        if variant == "deferred":
            child["completion_deferred"] = True
        if variant == "ok_only":
            send["result"] = {"ok": True}
    if variant == "one_unverified":
        result["sends"][-1]["result"]["completion_verified"] = False
    if variant == "deferred_conflict":
        result["sends"][-1]["result"]["completion_deferred"] = True
    if variant == "source_failure":
        result["ok"] = False
    if variant == "wait_ok_only":
        result["sends"][-1]["result"]["completion_verified"] = False
        result["sends"][-1]["completion"] = {"ok": True}
    if variant == "missing_channel":
        result["sends"].pop()
    if variant == "empty":
        result.update(channels=[], sends=[])
    if variant == "explicit_false":
        result.update(dict.fromkeys(FLAGS, False))
    _, row, _ = run(store, "release_unneeded_tips", result)
    assert tuple(row[f] for f in FLAGS) == expected
    assert row["status"] == status


@pytest.mark.parametrize("failure,status", [(RuntimeError("partial eject"), "failed"),
                                             (TimeoutError("partial wait"), "reconciliation_required")])
def test_partial_exception_keeps_original_failure_and_channel_evidence(store, failure, status):
    partial = copy.deepcopy(RETAINED["release_unneeded_tips"])
    partial["ok"] = False
    partial["sends"] = partial["sends"][:1]
    returned, row, receipt = run(store, "release_unneeded_tips", partial, failure=failure)
    assert isinstance(returned, HTTPException)
    assert row["status"] == status
    assert row["completion_verified"] == 0
    assert receipt["sends"] == partial["sends"]
    assert returned.detail["pipette_partial_result"]["sends"] == partial["sends"]


@pytest.mark.parametrize("phase", ["send", "wait"])
@pytest.mark.parametrize("error_type", [RuntimeError, TimeoutError, ValueError, PipetteCommandError])
def test_real_eject_throw_keeps_prefix_through_service(store, phase, error_type):
    from types import SimpleNamespace
    from bioxp.pipette.transport import FourPipetteTransport

    failure = error_type("P2 eject failed")
    if error_type is not RuntimeError:
        failure.details = {"owner_token": "original-owner", "channel": 1}
    trace = []
    completed = {"ok": True, **dict.fromkeys(FLAGS, True)}
    deferred = {"ok": True, "outcome": "tx_only", "delivery_verified": True,
                "controller_acknowledged": False, "completion_verified": False}

    class Driver:
        def __init__(self, channel):
            self.channel = channel

        def pipette_eject_tip(self, *, initialized, wait_for_completion):
            assert initialized and wait_for_completion
            trace.append(("send", self.channel))
            if phase == "send" and self.channel == 1:
                raise failure
            return dict(completed if phase == "send" else deferred)

        def wait_pipette_command_completion(self, timeout):
            assert 0 <= timeout <= 8
            trace.append(("wait", self.channel))
            if self.channel == 1:
                raise failure
            return dict(completed)

    transport = object.__new__(FourPipetteTransport)
    transport._transports = [SimpleNamespace(_initialized=True, _get_driver=lambda d=Driver(ch): d)
                             for ch in range(4)]

    def invoke():
        try:
            return transport._eject_tip_channels_once([0, 1, 2, 3], wait_for_completion=True)
        except Exception as exc:
            assert exc is failure  # No replacement type, retry, or owner transfer.
            if error_type is not RuntimeError:
                assert failure.details["owner_token"] == "original-owner"
                assert failure.details["channel"] == 1
            raise

    returned, row, receipt = run(store, "release_unneeded_tips", None,
                                 failure=failure, transport_call=invoke)
    assert returned.__cause__ is failure
    assert trace == ([("send", 0), ("send", 1)] if phase == "send" else
                     [("send", ch) for ch in range(4)] + [("wait", 0), ("wait", 1)])
    sends = failure.details["sends"]
    assert [item["channel"] for item in sends] == ([0] if phase == "send" else [0, 1, 2, 3])
    assert sends[0]["result" if phase == "send" else "completion"] == completed
    if phase == "wait":
        assert all("completion" not in item for item in sends[1:])
    assert receipt["sends"] == sends
    assert returned.detail["pipette_partial_result"]["sends"] == sends
    assert returned.detail["pipette_partial_result"]["ok"] is False
    assert row["status"] == ("reconciliation_required" if error_type is TimeoutError else "failed")
    assert row["completion_verified"] == 0


def test_unrelated_operation_is_not_recursively_promoted(store):
    _, row, _ = run(store, "unrelated", copy.deepcopy(RETAINED["eject_tip"]))
    assert tuple(row[f] for f in FLAGS) == (0, 0, 0)
    assert row["status"] == "failed"
