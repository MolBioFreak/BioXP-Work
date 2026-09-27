"""Constructor pipette completion survives the real typed SQLite receipt path."""
import asyncio
import json
from types import SimpleNamespace
import pytest

from bioxp.pipette.models import PipetteInitCommand
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.services.pipette_service import run_pipette_init_command
from tests.test_deck_tip_query_publication import bind_collection_test_identity


@pytest.mark.parametrize("mode", ["initial", "already_initialized", "conditional_retry", "completion_failed"])
def test_four_channel_init_records_controller_completion(tmp_path, monkeypatch, mode):
    bind_collection_test_identity(monkeypatch)
    runtime = OEMRuntimeStore(tmp_path)
    store = PipetteReceiptStore(tmp_path)
    transports = []
    status_queries = [0] * 4
    for channel in range(4):
        def query_status(_channel=channel):
            status_queries[_channel] += 1
            return {"ok": not (mode == "conditional_retry" and _channel == 2
                               and status_queries[_channel] == 1)}
        router = SimpleNamespace(reader_generation=1,
            begin_pressure_epoch=lambda: 1,
            calculate_pressure_offset_evidence=lambda: {})
        driver = SimpleNamespace(bus=SimpleNamespace(router=router),
            pipette_initiate_group=lambda: {
                "ok": True, "tx_ok": True, "delivery_verified": True,
                "controller_acknowledged": True, "immediate_ack_received": True,
            },
            pipette_initialize=lambda **_: {
            "ok": True, "tx_ok": True, "delivery_verified": True,
            "controller_acknowledged": True, "immediate_ack_received": True,
            "initialized_after_valid_completion": False,
        }, wait_pipette_initialization_completion=lambda _, _channel=channel: {
            "ok": mode != "completion_failed" or _channel != 3},
            enable_pressure_stream=lambda _: {
            "ok": True, "delivery_verified": True, "controller_acknowledged": True,
        }, query_firmware=lambda _: {"ok": True}, query_status=query_status)
        transport = CanPipetteTransport(driver_factory=lambda d=driver: d,
                                        pipette_id=channel)
        transport._initialized = mode == "already_initialized"
        transports.append(transport)
    group = FourPipetteTransport(transports, sleep=lambda _: None)

    async def run_inline(_label, operation, **_kwargs):
        return operation()

    result = asyncio.run(run_pipette_init_command(
        PipetteInitCommand(), get_transport=lambda: group, run_blocking=run_inline,
        receipt_store=store, runtime_binding={
            "entrypoint_id": "lifecycle.constructor_pipette_stage",
            "caller_class": "lifecycle", "control_class": "pipette_state_command",
            "lifecycle_stage_id": "constructor_pipette_stage",
            "lifecycle_attempt_id": "offline-initialization-" + mode,
            "idempotency_key": "constructor_pipette_stage:offline-initialization-" + mode,
        },
    ))
    row = store.connection.execute(
        "SELECT status, outcome, receipt_json, response_summary_json FROM operator_commands WHERE command_id=?",
        (result["command_id"],),
    ).fetchone()
    child = store.connection.execute(
        "SELECT status, outcome, receipt_json FROM pipette_operations WHERE command_id=?",
        (result["command_id"],),
    ).fetchone()
    if mode == "completion_failed":
        assert result["ok"] is False and result["outcome"] == "initial_group_cycle_failed"
        assert result["receipt_truth"]["completion_verified"] is False
        assert row["status"] == child["status"] == "failed"
        assert json.loads(row["response_summary_json"])["ok"] is False
    else:
        assert result["ok"] is True and result["outcome"] == "completion"
        assert [r["result"]["ok"] for r in result["status_readback_final"]] == [True] * 4
        assert result["initial_group"]["completion_verified"] is True
        assert result["single_conditional_retry_performed"] is (mode == "conditional_retry")
        assert all(r["result"]["software_initialized"] for r in result["channels"])
        assert all(result["receipt_truth"][key] is True for key in (
            "delivery_verified", "controller_acknowledged", "completion_verified"))
        assert result["receipt_truth"]["physical_effect_verified"] is False
        assert json.loads(child["receipt_json"])["requested_inputs"] == {
            "pressure_profile": "1R", "prime_volume_ul": None}
        assert row["status"] == child["status"] == "completed"
        assert row["outcome"] == child["outcome"] == "completion"
        assert json.loads(child["receipt_json"])["truth"]["completion_verified"] is True
        assert json.loads(row["response_summary_json"])["ok"] is True
    assert json.loads(row["receipt_json"])["status"] == "reserved"  # original claim identity, not terminal projection
    store._audit_database.close()
    runtime.close()
