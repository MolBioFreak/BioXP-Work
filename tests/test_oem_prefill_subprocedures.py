"""zOffset liquid calls through real source body and finite-owner callback shape.

Only the native hardware transport and leaf callbacks are replaced.
"""
import pytest

from bioxp.services.pipette_service import build_oem_prefill_subprocedures
from tests.test_protocol_oem_pipette_composites import composite_bindings


def bound(volume=40, *, fluid=0, tray=3, location=16, log_pressure=True):
    args, state, transport, fences, effects, facts = composite_bindings(volume)
    facts.update(current_location=location, current_tray=tray, fluid_level=fluid)
    args["settings"]["LogPressure"] = log_pressure
    # Mimic the transport's cached FluidLevel, which the real provider reads
    # after each completed standard pipette operation.
    aspirate = transport.aspirate_for_oem_script
    dispense = transport.dispense_for_oem_script
    set_speed = transport.set_top_speed
    def speed(value):
        result = set_speed(value)
        facts["speed"] = value
        return result
    def asp(volume, *, pressure_stream):
        result = aspirate(volume, pressure_stream=pressure_stream)
        facts["fluid_level"] += volume
        return result
    def dsp(volume, *, pressure_stream):
        result = dispense(volume, pressure_stream=pressure_stream)
        facts["fluid_level"] -= volume
        return result
    transport.aspirate_for_oem_script = asp
    transport.dispense_for_oem_script = dsp
    transport.set_top_speed = speed
    callbacks = build_oem_prefill_subprocedures(
        state=state, source_occurrence_id="owner:scan", before_native_entry=args["before_native_entry"],
        pipette_call=args["pipette_call"], source_bindings=args["source_bindings"],
        settings=args["settings"])
    return callbacks, state, transport, fences, effects, facts


def test_prefill_single_trough_and_purge_order():
    (asp, dsp), state, transport, fences, effects, facts = bound()
    a = asp(40)
    assert transport.calls[:3] == [("speed", 100.0), ("air", 10, [2], False), ("asp_stream", 40, True)]
    assert [x["operation"] for x in a["native_results"]] == [
        "set_top_speed", "liftTo", "aspirate_air", "lowerTo", "aspirate_for_oem_script",
        "liftTo", "tip_state"]
    assert ("sleep", .3) in effects and ("sleep", 1.0) not in effects
    assert state.source_model.calls[-2][0] == "name"
    assert state.source_model.calls[-1][-1] == -40
    d = dsp()
    assert ("dsp_stream", 40, True) in transport.calls
    assert ("dspall",) in transport.calls
    assert transport.calls[-3:] == [("speed", 30.0), ("dspall",), ("speed", 100.0)]
    assert [x["operation"] for x in d["native_results"]] == [
        "set_top_speed", "liftTo", "dispense_for_oem_script", "liftTo", "set_top_speed",
        "dispense_all", "set_top_speed", "liftTo"]
    assert ("sleep", .1) in effects and ("sleep", 0.0) in effects
    assert facts["fluid_level"] == 0
    assert a["source_return"] is d["source_return"] is None
    assert len(fences) == len(set(fences))
    assert all(x.startswith("owner:scan:prefill:") for x in fences)


def test_prefill_split_tray_and_pierce_side_effects():
    (asp, dsp), state, transport, fences, effects, facts = bound(
        200, fluid=0, tray=0, location=3)
    # Source checks current well's pierce status only at the end, not as a new gate.
    asp(140)
    assert [x for x in transport.calls if x[0] == "asp_stream"] == [
        ("asp_stream", 70, True), ("asp_stream", 70, True)]
    assert [x[-1] for x in state.source_model.calls if x[0] == "fluid"] == [-70, -70]
    assert ("home", None) in effects and ("pierce", 2, 0, False) in effects
    result = dsp()
    assert [x for x in transport.calls if x[0] == "dsp_stream"] == [
        ("dsp_stream", 70, True), ("dsp_stream", 70, True)]
    assert [x[-1] for x in state.source_model.calls if x[0] == "fluid"][-2:] == [70, 70]
    assert [x["operation"] for x in result["native_results"]].count("liftTo") == 4
    assert len(fences) == len(set(fences))


def test_prefill_failure_preserves_completed_prefix_and_no_purge():
    (asp, dsp), state, transport, fences, effects, facts = bound()

    def fail(*args, **kwargs):
        raise RuntimeError("transport failed")
    transport.dispense_for_oem_script = fail
    with pytest.raises(RuntimeError, match="transport failed") as error:
        dsp()
    assert [x["operation"] for x in error.value.oem_partial_results] == [
        "set_top_speed", "liftTo"]
    assert not any(x[0] == "dspall" for x in transport.calls)
    assert not state.source_model.calls
    assert len(fences) == 3  # failed command was fenced as well


def test_prefill_repeated_aliquots_have_distinct_owner_identities():
    (asp, dsp), state, transport, fences, effects, facts = bound()
    asp(40); dsp(); asp(40); dsp()
    assert len(fences) == len(set(fences))
    assert set(x.split(":prefill:")[1].split(":")[0] for x in fences) == {"1", "2", "3", "4"}


def test_pressure_logging_false_still_uses_standard_oem_liquid_overloads():
    (asp, dsp), _, transport, _, _, _ = bound(log_pressure=False)
    asp(40); dsp()
    assert ("asp_stream", 40, False) in transport.calls
    assert ("dsp_stream", 40, False) in transport.calls
    assert not any(x[0] in ("asp", "dsp") for x in transport.calls)
