"""Hardware-free source-order tests for the bounded zOffset body."""
import pytest

from bioxp.pipette.oem_calibration import CalibrationExecutionError
from bioxp.pipette.oem_fluid_workflows import FluidScanBindings, z_offset


class Source:
    def __init__(self, fail_at=None):
        self.calls = []
        self.location = 7
        self.well = 3
        self.fail_at = fail_at
        self.detected = 0

    def record(self, name, *args):
        self.calls.append((name, *args))
        if self.fail_at == len(self.calls):
            raise RuntimeError("source motion failed")
        return {"ok": True}

    def facts(self):
        self.record("facts")
        return {"current_location": self.location, "current_well": self.well}

    def move(self, location, well, flag):
        result = self.record("move", location, well, flag)
        self.location = location
        return result

    def publish(self, location, well):
        result = self.record("publish", location, well)
        self.location = location
        self.well = well
        return result

    def detect(self, speed):
        self.detected += 1
        return {**self.record("detect", speed), "source_return": 100 + self.detected}

    def bindings(self):
        return FluidScanBindings(facts=self.facts,
            publish_plate=lambda location, plate: self.record("plate", location, plate),
            load_tips=lambda: self.record("loadTips", 50, True),
            move=self.move, publish=self.publish,
            aspirate=lambda volume: self.record("masp", 100, volume),
            dispense=lambda: self.record("mdsa", 100), detect=self.detect,
            query_tips=lambda: {**self.record("query"), "tip_exists": False},
            eject=lambda: self.record("eject", True, True),
            clear_tip_loaded=lambda: self.record("clear"))


def test_diagnostic_scan_a_then_b_and_source_return_publication():
    source = Source()
    result = z_offset("STRIP", source.bindings(), transfer_fluid=False, skip_steps=1)
    assert result["ok"] is True
    assert source.calls[0] == ("plate", 12, 8)
    assert [sample["well"] for sample in result["samples"]] == ["A1", "B1", "A2", "B2", "A3", "B3"]
    assert result["source_return"] == 104  # int(mean(101..106) + .5)
    assert ("move", 7, 3, 1) in source.calls
    assert ("publish", 12, 3) in source.calls
    assert ("loadTips", 50, True) in source.calls
    assert not any(call[0] == "masp" for call in source.calls)
    assert len([call for call in source.calls if call[0] == "clear"]) == 6


def test_accepted_scan_prefills_trough_before_sampling():
    source = Source()
    result = z_offset("RC", source.bindings())
    assert result["ok"] is True
    assert [row["well"] for row in result["samples"]] == [
        "A1", "B1", "A5", "B5", "A9", "B9"]
    assert len([x for x in source.calls if x == ("masp", 100, 40)]) == 6
    assert len([x for x in source.calls if x == ("mdsa", 100)]) == 6
    assert source.calls.index(("move", 16, 0, 0)) < source.calls.index(("detect", 300))
    assert ("publish", 3, 3) in source.calls


def test_tip_query_repeats_until_clear_without_retry_cap():
    source = Source()
    queries = iter((True, True, False))
    bindings = source.bindings()
    from dataclasses import replace
    bindings = replace(bindings, query_tips=lambda: {
        **source.record("query"), "tip_exists": next(queries, False)})
    result = z_offset("STRIP", bindings, transfer_fluid=False, skip_steps=12)
    assert result["ok"] is True
    assert len([x for x in source.calls if x[0] == "eject"]) == 2
    assert len([x for x in source.calls if x[0] == "query"]) == 4


def test_source_ignores_false_load_tips_return_and_continues_sampling():
    from dataclasses import replace
    source = Source()
    def returned_false():
        source.record("loadTips", 50, True)
        return {"ok": False, "source_return": False}
    result = z_offset("STRIP", replace(source.bindings(), load_tips=returned_false),
                      transfer_fluid=False, skip_steps=12)
    assert result["source_return"] == 102
    assert len([x for x in source.calls if x[0] == "detect"]) == 2
    assert [s["result"]["source_return"] for s in result["steps"]
            if s["operation"] == "loadTips"] == [False, False]


def test_ignored_source_move_return_is_recorded_without_new_stop():
    from dataclasses import replace
    source = Source()
    def returned_false(target, well, flag):
        result = source.move(target, well, flag)
        return {**result, "ok": False}
    result = z_offset("STRIP", replace(source.bindings(), move=returned_false),
                      transfer_fluid=False, skip_steps=12)
    assert len(result["samples"]) == 2
    assert any(s["result"].get("ok") is False for s in result["steps"]
               if s["operation"] == "scriptmoveTo")


def test_ms_skips_prefill_and_failure_keeps_partial_effects():
    source = Source()
    result = z_offset("MS", source.bindings())
    assert len(result["samples"]) == 6
    assert not any(x[0] == "masp" for x in source.calls)
    failed = Source(fail_at=6)
    with pytest.raises(CalibrationExecutionError) as info:
        z_offset("TC", failed.bindings(), transfer_fluid=False)
    assert info.value.evidence["ok"] is False
    assert [x["operation"] for x in info.value.evidence["steps"]] == [
        "updatePlateLocation", "loadTips", "scriptmoveTo", "updateLocation", "scrFluidDetection"]
    assert not any(x[0] == "eject" for x in failed.calls)
