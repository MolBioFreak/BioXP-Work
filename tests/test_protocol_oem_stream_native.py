"""Three finite script overloads: real transports with explicit native leaves."""
from concurrent.futures import ThreadPoolExecutor
from threading import Event

import pytest

from bioxp.pipette.models import PipetteCommandError, PipetteTipStateError
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
from tests.test_protocol_oem_air_native import Native


class LiquidNative(Native):
    def _send(self, operation, volume, options):
        self.trace.append((operation, self.channel, volume, options))
        if self.fail_send:
            raise RuntimeError("send failed")
        self.token = f"liquid-{self.channel}"
        return {"ok": True, "tx_ok": True, "ack": {"ok": True, "received": True},
                "completion_verified": False, "native_marker": self.token}

    def aspirate(self, volume, **options):
        return self._send("aspirate", volume, options)

    def dispense(self, volume, **options):
        return self._send("dispense", volume, options)

    def wait_pipette_command_completion(self, timeout, *, owner_token):
        self.trace.append(("wait", self.channel, owner_token, timeout))
        if owner_token.startswith("liquid") and self.block is not None:
            self.entered.set()
            assert self.block.wait(3)
        return {"ok": self.wait_ok, "owner_token": owner_token, "native_wait": True}


def collection(callback=None):
    trace = []
    drivers = [LiquidNative(ch, trace) for ch in range(4)]
    transports = [CanPipetteTransport(driver_factory=lambda d=d: d, pipette_id=ch)
                  for ch, d in enumerate(drivers)]
    for transport in transports:
        transport._initialized = True
        transport._liquid_level_ul = 100
        transport._top_speed = 300
    group = FourPipetteTransport(transports, sleep=lambda s: trace.append(("sleep", s)),
                                 error_callback=callback)
    return group, drivers, trace


def invoke(group, operation, volume=5, stream=True):
    return getattr(group, operation + "_for_oem_script")(volume, pressure_stream=stream)


@pytest.mark.parametrize("operation", ["aspirate", "dispense"])
@pytest.mark.parametrize("stream", [False, True])
@pytest.mark.parametrize("location", [-1, 2])
def test_source_order_arguments_and_selected_completion(operation, stream, location):
    group, drivers, trace = collection()
    group._tip_location = location
    result = invoke(group, operation, stream=stream)
    selected = list(range(4)) if location == -1 else [2]
    expected = [("query", ch) for ch in range(4)]
    if stream and operation == "dispense":
        expected += [("stream", ch, True) for ch in range(4)] + [("sleep", .2)]
    options = {"tip_pressure_profile": "1R", "wait_for_completion": False}
    if operation == "dispense":
        options.update(blow_out=False, dispense_type=0)
    for ch in range(4):
        if stream and operation == "aspirate":
            expected += [("stream", ch, True), ("sleep", .2)]
        if ch in selected:
            if operation == "dispense":
                expected.append(("sleep", .005))
            expected += [(operation, ch, 5., options), ("sleep", .01)]
    assert trace[:len(expected)] == expected
    waits = [row for row in trace if row[0] == "wait"]
    assert [row[2] for row in waits] == [f"liquid-{ch}" for ch in selected]
    timeout = 4080 if operation == "aspirate" else 5083
    assert result["timeout_ms"] == timeout
    assert all(0 <= row[3] <= timeout / 1000 for row in waits)
    assert [row[3] for row in waits] == sorted([row[3] for row in waits], reverse=True)
    assert trace[len(expected) + len(waits):] == (
        [("stream", ch, False) for ch in range(4)] if stream else [])
    assert result["source_return"] == (None if operation == "aspirate" else 0)
    assert result["ok"] and result["source_return_completed"] and result["completion_verified"]
    assert result["delivery_verified"] and result["controller_acknowledged"]
    assert result["state_reconciled"] and not result["physical_effect_verified"]
    assert group._allow_to_stop is (operation != "aspirate")
    for ch in range(4):
        assert group._transports[ch]._liquid_level_ul == (
            100 + (5 if operation == "aspirate" else -5) if ch in selected else 100)
    assert result["channels"][0]["result"]["driver_result"]["native_marker"].startswith("liquid-")


@pytest.mark.parametrize("operation", ["aspirate", "dispense"])
def test_false_wait_is_source_return_not_completion(operation):
    group, drivers, trace = collection()
    drivers[1].wait_ok = False
    result = invoke(group, operation, volume=10)
    assert trace[-5:] == [("sleep", 1.)] + [("stream", ch, False) for ch in range(4)]
    assert result["ok"] and result["source_return_completed"]
    assert not result["source_wait_return"] and not result["completion_verified"]
    assert not result["state_reconciled"] and not result["physical_effect_verified"]
    assert group._transports[1]._liquid_level_ul == 100
    assert group._transports[0]._liquid_level_ul == (110 if operation == "aspirate" else 90)
    assert ("sleep", .005) not in trace


@pytest.mark.parametrize("operation", ["aspirate", "dispense"])
def test_forceabort_is_entry_guard_only_for_aspirate(operation):
    group, drivers, trace = collection()
    group._forceabort = lambda: True
    if operation == "aspirate":
        with pytest.raises(PipetteCommandError, match="Stopped"):
            invoke(group, operation)
        assert trace == [] and group._allow_to_stop
    else:
        assert invoke(group, operation)["source_return_completed"]
        assert group._allow_to_stop


@pytest.mark.parametrize("operation", ["aspirate", "dispense"])
def test_lost_tip_callback_and_missing_callback_branches(operation):
    errors = []
    group, drivers, trace = collection(lambda ch, code: errors.append((ch, code)))
    drivers[1].loaded = False
    with pytest.raises(PipetteTipStateError) as caught:
        invoke(group, operation)
    assert errors == [(1, 42)]
    assert len(caught.value.details["before_tip_status"]) == 4
    assert trace == [("query", ch) for ch in range(4)]
    group._error_callback = None
    assert invoke(group, operation)["source_return_completed"]
    assert len([row for row in trace if row[0] == operation]) == 4


@pytest.mark.parametrize("operation", ["aspirate", "dispense"])
def test_selected_missing_tip_with_other_tips_preserves_source_count_predicate(operation):
    group, drivers, trace = collection(lambda *args: pytest.fail("unexpected lost-tip callback"))
    group._tip_location = 2
    drivers[2].loaded = False
    group._transports[0]._top_speed = 1
    assert invoke(group, operation)["timeout_ms"] == (4080 if operation == "aspirate" else 5083)
    assert [row[1] for row in trace if row[0] == operation] == [2]


@pytest.mark.parametrize("operation", ["aspirate", "dispense"])
def test_query_exception_keeps_partial_query_state_no_send(operation):
    group, drivers, trace = collection()
    drivers[0].loaded = False
    drivers[2].fail_query = True
    with pytest.raises(PipetteCommandError) as caught:
        invoke(group, operation)
    assert trace == [("query", ch) for ch in range(3)]
    assert len(caught.value.details["before_tip_status"]) == 2
    assert group._transports[0]._tip_loaded is False
    assert caught.value.details["source_return_completed"] is False


@pytest.mark.parametrize("operation", ["aspirate", "dispense"])
def test_send_exception_retains_partial_no_finally_cleanup(operation):
    group, drivers, trace = collection()
    drivers[2].fail_send = True
    with pytest.raises(RuntimeError, match="send failed") as caught:
        invoke(group, operation)
    partial = caught.value.oem_partial_results
    assert len(partial["channels"]) == 2
    assert len(partial["pressure_stream"]) == (3 if operation == "aspirate" else 4)
    assert partial["source_return_completed"] is False
    assert not any(row[0] == "wait" or row[0] == "stream" and not row[2] for row in trace)


@pytest.mark.parametrize("operation", ["aspirate", "dispense"])
def test_real_terminate_during_wait_fences_old_completion_and_stream_shutdown(operation):
    group, drivers, trace = collection()
    drivers[0].block, drivers[0].entered = Event(), Event()
    with ThreadPoolExecutor(max_workers=2) as pool:
        future = pool.submit(invoke, group, operation)
        assert drivers[0].entered.wait(3)
        try:
            assert pool.submit(group.terminate).result(timeout=3)["ok"]
        finally:
            drivers[0].block.set()
        result = future.result(timeout=3)
    assert result["interrupted_by_terminate"] and not result["ok"]
    assert not result["source_return_completed"] and not result["completion_verified"]
    assert not result["state_reconciled"]
    assert all(t._liquid_level_ul == 100 for t in group._transports)
    assert not any(row[0] == "stream" and row[2] is False for row in trace)
    assert [row[2] for row in trace if row[0] == "wait" and row[2].startswith("liquid")] == [
        f"liquid-{ch}" for ch in range(4)]


@pytest.mark.parametrize("channel", range(4))
@pytest.mark.parametrize("loaded", [True, False])
def test_selected_query_only_normalizes_source_return(channel, loaded):
    group, drivers, trace = collection()
    drivers[channel].loaded = loaded
    result = group.query_tip_status_for_oem_script(channel)
    assert trace == [("query", channel)]
    assert result["source_return"] == int(loaded)
    assert result["result"]["source_return"] == (1 if loaded else 2)
    assert result["hardware_postcondition_verified"] and result["source_return_completed"]
    assert group._transports[channel]._tip_loaded is loaded


@pytest.mark.parametrize("source_return", [0, 2, -1])
def test_selected_query_source_default_is_not_verified_readback(source_return):
    group, drivers, trace = collection()
    def query():
        trace.append(("query", 2))
        return {"ok": False, "source_return_completed": True, "source_return": source_return,
                "source_tip_loaded": False, "hardware_truth_level": "unavailable"}
    drivers[2].query_tip_status = query
    result = group.query_tip_status_for_oem_script(2)
    assert trace == [("query", 2)]
    assert result["ok"] and result["source_return_completed"]
    assert result["source_return"] == 0 and not result["hardware_postcondition_verified"]


def test_selected_query_exception_does_not_fallback_to_other_channels():
    group, drivers, trace = collection()
    drivers[2].fail_query = True
    with pytest.raises(PipetteCommandError):
        group.query_tip_status_for_oem_script(2)
    assert trace == [("query", 2)]


@pytest.mark.parametrize("channel", [-1, 4, True, "2"])
def test_selected_query_rejects_non_channel_without_native_entry(channel):
    group, drivers, trace = collection()
    with pytest.raises(ValueError):
        group.query_tip_status_for_oem_script(channel)
    assert trace == []
