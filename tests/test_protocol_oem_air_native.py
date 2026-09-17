"""Exact script DispenseAir overload; real transports, explicit native doubles."""
from concurrent.futures import ThreadPoolExecutor
from threading import Event

import pytest

from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
from bioxp.pipette.models import PipetteCommandError, PipetteTipStateError


class Native:
    def __init__(self, channel, trace):
        self.channel, self.trace = channel, trace
        self.loaded = True
        self.wait_ok = True
        self.fail_send = False
        self.fail_query = False
        self.token = None
        self.block = None
        self.entered = None

    def query_tip_status(self):
        self.trace.append(("query", self.channel))
        if self.fail_query:
            raise RuntimeError("query failed")
        return {"ok": True, "semantic_ok": True, "hardware_truth_level": "hardware_query",
                "source_return_completed": True, "source_return": 1 if self.loaded else 2,
                "tip_loaded": self.loaded, "source_tip_loaded": self.loaded}

    def enable_pressure_stream(self, enabled):
        self.trace.append(("stream", self.channel, enabled))
        return {"ok": True, "marker": (self.channel, enabled)}

    def dispense_air(self, volume, *, dispense_type, wait_for_completion):
        self.trace.append(("dispense", self.channel, volume, dispense_type, wait_for_completion))
        if self.fail_send:
            raise RuntimeError("send failed")
        self.token = f"dispense-{self.channel}"
        return {"ok": True, "tx_ok": True, "ack": {"ok": True, "received": True},
                "completion_verified": False, "native_marker": self.token}

    def current_pipette_completion_owner_token(self):
        return self.token

    def wait_pipette_command_completion(self, timeout, *, owner_token):
        self.trace.append(("wait", self.channel, owner_token, timeout))
        if owner_token.startswith("dispense") and self.block is not None:
            self.entered.set()
            assert self.block.wait(3)
        return {"ok": self.wait_ok, "owner_token": owner_token, "native_wait": True}

    def terminate_pipette(self, *, wait_for_completion):
        self.trace.append(("terminate", self.channel, wait_for_completion))
        self.token = f"stop-{self.channel}"
        return {"ok": True, "tx_ok": True}


def collection(*, callback=None):
    trace = []
    drivers = [Native(ch, trace) for ch in range(4)]
    transports = [CanPipetteTransport(driver_factory=lambda d=d: d, pipette_id=ch)
                  for ch, d in enumerate(drivers)]
    for transport in transports:
        transport._initialized = True
        transport._front_air_level_ul = 100
        transport._rear_air_level_ul = 100
    group = FourPipetteTransport(transports, sleep=lambda s: trace.append(("sleep", s)),
                                 error_callback=callback)
    return group, drivers, trace


@pytest.mark.parametrize("front,kind", [(True, 1), (False, 2)])
@pytest.mark.parametrize("stream", [False, True])
def test_exact_order_source_return_and_raw_evidence(front, kind, stream):
    group, drivers, trace = collection()
    result = group.dispense_air_for_oem_script(5, front, stream)
    expected = [("query", ch) for ch in range(4)]
    if stream:
        expected += [("stream", ch, True) for ch in range(4)] + [("sleep", .2)]
    for ch in range(4):
        expected += [("sleep", .005), ("dispense", ch, 5., kind, False)]
    assert trace[:len(expected)] == expected
    waits = [row for row in trace if row[0] == "wait"]
    assert [row[2] for row in waits] == [f"dispense-{ch}" for ch in range(4)]
    assert all(0 <= row[3] <= 4.025 for row in waits)
    if stream:
        assert trace[-4:] == [("stream", ch, False) for ch in range(4)]
    assert result["source_return"] == 0 and result["ok"]
    assert result["source_return_completed"] and result["completion_verified"]
    assert not result["physical_effect_verified"]
    assert group._allow_to_stop is True  # this overload never writes AllowtoStop
    assert result["channels"][2]["result"]["driver_result"]["native_marker"] == "dispense-2"
    assert result["channels"][2]["completion"]["native_wait"] is True


def test_false_wait_sleeps_then_disables_stream_and_returns_zero():
    group, drivers, trace = collection()
    drivers[1].wait_ok = False
    result = group.dispense_air_for_oem_script(10, True, True)
    assert trace[-5:] == [("sleep", 1.)] + [("stream", ch, False) for ch in range(4)]
    assert result["ok"] and result["source_return_completed"]
    assert result["source_return"] == 0 and result["source_wait_return"] is False
    assert not result["completion_verified"] and not result["state_reconciled"]
    assert group._transports[1]._front_air_level_ul == 100
    assert group._transports[0]._front_air_level_ul == 90


def test_location_speed_and_enable_mask_are_not_tip_occupancy():
    group, drivers, trace = collection()
    group._tip_location = 2
    group._transports[2]._top_speed = 300
    drivers[2].loaded = False  # other tips exist: source count predicate does not throw
    result = group.dispense_air_for_oem_script(10, False, enabled_channels=(2,))
    assert result["timeout_ms"] == 4165
    assert [row[1] for row in trace if row[0] == "dispense"] == [2]
    assert [row[1] for row in trace if row[0] == "query"] == [0, 1, 2, 3]
    assert not any(row == ("sleep", .005) for row in trace)


def test_disabled_channels_settled_without_waiting_stale_owners():
    group, drivers, trace = collection()
    result = group.dispense_air_for_oem_script(1, True, enabled_channels=())
    assert result["ok"] and result["source_return"] == 0
    assert not any(row[0] in {"dispense", "wait"} for row in trace)


def test_lost_tip_requires_source_callback_and_preserves_partial_queries():
    errors = []
    group, drivers, trace = collection(callback=lambda ch, code: errors.append((ch, code)))
    drivers[1].loaded = False
    with pytest.raises(PipetteTipStateError) as caught:
        group.dispense_air_for_oem_script(10, True, True)
    assert errors == [(1, 42)]
    assert len(caught.value.details["before_tip_status"]) == 4
    assert not any(row[0] == "stream" for row in trace)


def test_missing_callback_preserves_source_nonthrow_branch():
    group, drivers, trace = collection()
    drivers[1].loaded = False
    assert group.dispense_air_for_oem_script(10, True)["ok"]
    assert len([row for row in trace if row[0] == "dispense"]) == 4


def test_query_exception_does_not_send_or_retry():
    group, drivers, trace = collection()
    drivers[2].fail_query = True
    with pytest.raises(PipetteCommandError) as caught:
        group.dispense_air_for_oem_script(10, True, True)
    assert trace == [("query", ch) for ch in range(3)]
    assert len(caught.value.details["before_tip_status"]) == 2


def test_send_exception_keeps_partial_evidence_without_invented_finally_cleanup():
    group, drivers, trace = collection()
    drivers[2].fail_send = True
    with pytest.raises(RuntimeError, match="send failed") as caught:
        group.dispense_air_for_oem_script(10, True, True)
    partial = caught.value.oem_partial_results
    assert len(partial["channels"]) == 2
    assert len(partial["pressure_stream"]) == 4
    assert not any(row[0] == "wait" or row[0] == "stream" and not row[2] for row in trace)


def test_real_terminate_can_enter_while_script_waits_and_fences_shutdown():
    group, drivers, trace = collection()
    drivers[0].block, drivers[0].entered = Event(), Event()
    with ThreadPoolExecutor(max_workers=2) as pool:
        future = pool.submit(group.dispense_air_for_oem_script, 10, True, True)
        assert drivers[0].entered.wait(3)
        try:
            stop = pool.submit(group.terminate).result(timeout=3)
            assert stop["ok"]
        finally:
            drivers[0].block.set()
        result = future.result(timeout=3)
    assert result["interrupted_by_terminate"] and not result["ok"]
    assert not result["completion_verified"] and not result["source_return_completed"]
    assert not any(row[0] == "stream" and row[2] is False for row in trace)
    assert [row[2] for row in trace if row[0] == "wait" and row[2].startswith("dispense")] == [
        f"dispense-{ch}" for ch in range(4)]


def test_generic_manual_api_retains_default_type_and_strict_false_wait():
    group, drivers, trace = collection()
    drivers[0].wait_ok = False
    result = group.dispense_air(10)
    assert result["ok"] is False and result["dispense_type"] == 0
    assert all(row[3] == 0 for row in trace if row[0] == "dispense")
    assert not any(row[0] in {"stream", "sleep"} for row in trace)
    assert group._allow_to_stop is False
