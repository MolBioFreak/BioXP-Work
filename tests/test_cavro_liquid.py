"""Recipe decisions are explicit test choices, not missing manufacturer values."""
from decimal import Decimal
import pytest
from bioxp.pipette.cavro_application import PRESSURE_PARAMETERS
from bioxp.pipette.cavro_liquid import compile_liquid_recipe, correction_at_target
from tests.test_cavro_application import rig, run, request


def recipe():
    return {"mode": "single", "channels": [0], "timeout_ms": 30,
        "target_liquid_ul": 20, "commanded_aspiration_ul": 22.75,
        "aspiration_speed_ul_s": 50, "aspiration_delay_ms": 0,
        "leading_air": {"volume_ul": 20, "speed_ul_s": 30},
        "trailing_air": {"volume_ul": 1.25, "speed_ul_s": 40},
        "before_leading_air": [], "before_liquid": [], "after_liquid": [],
        "before_dispense": [], "after_dispense": [], "final_empty_before": [],
        "dispense_segments": [{"volume_ul": 12.75, "speed_ul_s": 375}, {"volume_ul": 10, "speed_ul_s": 875}],
        "final_empty_speed_ul_s": 875, "multi": None,
        "liquid_settings": {"source": "399156 V1.0 p8 glycerol 50% T200 20ul",
            "test_authored_choices": ["air speeds", "zero delay", "final empty speed", "in-place motion lists"]}}


def test_settings_connected_every_parameter_including_fragmented_wire(rig):
    values = {name: spec[3] for name, spec in PRESSURE_PARAMETERS.items()}
    values.update(slope=[20, 40], pressure_streaming=True)
    out = run(rig, request({"operation": "settings", "channels": [0, 2], "timeout_ms": 30, "values": values},
        {"operation": "settings", "channels": [0, 2], "timeout_ms": 30, "values": {"pressure_streaming": False}}))
    assert out["ok"], out
    assert (0, "p4,44000R") in rig.wire
    assert (2, "L20,40R") in rig.wire
    assert rig.wire[-2:] == [(0, "o0,0R"), (2, "o0,0R")]
    assert len({token for _, token in rig.waits}) == len(rig.wire)


def test_segmented_class_displacement_not_target_volume(rig):
    source = recipe()
    plan = compile_liquid_recipe(source)
    assert not plan["issues"], plan
    assert plan["recipe_requested"] == source
    out = run(rig, plan["application"])
    assert out["ok"], out
    assert (0, "P22.75,1R") in rig.wire
    assert (0, "D12.75,1R") in rig.wire
    assert (0, "D10,1R") in rig.wire
    assert (0, "P20,1R") in rig.wire  # leading air, not substitute sample target
    assert out["liquid_settings"]["resolved_recipe"]["target_liquid_ul"] == 20


def test_multi_conditioning_counts_reaspiration_delay_and_single_final_empty(rig):
    source = recipe()
    source.update(mode="multi", commanded_aspiration_ul=125, target_liquid_ul=60,
                  leading_air=None, trailing_air=None, dispense_segments=[])
    source["multi"] = {"sample_count": 3, "sample_volume_ul": 20,
        "conditioning_volume_ul": 10, "conditioning_back_to_source_count": 2,
        "conditioning_speed_ul_s": 450, "conditioning_before": [], "conditioning_after": [],
        "excess_volume_ul": 45, "excess": None,
        "reaspiration": {"volume_ul": 1, "speed_ul_s": 30}, "dispense_to_reaspiration_delay_ms": 20,
        "aliquots": [{"before": [], "segments": [{"volume_ul": 20, "speed_ul_s": 450}], "after": []} for _ in range(3)]}
    plan = compile_liquid_recipe(source)
    assert not plan["issues"], plan
    kinds = [op["operation"] for op in plan["operations"]]
    assert kinds.count("dispense") == 5
    assert kinds.count("reaspirate") == 3
    assert kinds.count("empty") == 1 and kinds[-1] == "empty"
    out = run(rig, plan["application"])
    assert out["ok"], out
    assert rig.wire.count((0, "D10,1R")) == 2
    assert rig.wire.count((0, "D20,1R")) == 3
    assert rig.wire.count((0, "P1,1R")) == 3
    assert rig.wire.count((0, "A0R")) == 1
    assert len([e for e in out["events"] if e["operation"] == "delay" and e["inputs"]["duration_ms"] == 20]) == 3


def test_missing_water_or_invalid_field_not_replaced():
    source = recipe()
    source["aspiration_delay_ms"] = None
    out = compile_liquid_recipe(source)
    assert out["application"] is None and out["issues"]
    assert out["recipe_requested"]["aspiration_delay_ms"] is None
    source = recipe()
    source["phase_settings"] = {"dispense": {"start_speed_ul_s": None}}
    assert compile_liquid_recipe(source)["application"] is None


def test_original_start_cutoff_phase_order_and_no_implicit_reset(rig):
    source = recipe()
    source["phase_settings"] = {
        "leading_air": {"start_speed_ul_s": "2.500", "cutoff_speed_ul_s": 50},
        "aspirate": {"start_speed_ul_s": 25, "cutoff_speed_ul_s": 50},
        "trailing_air": {"start_speed_ul_s": 10},
        "dispense": {"start_speed_ul_s": 25, "cutoff_speed_ul_s": 200, "slope": [20, 10]},
    }
    plan = compile_liquid_recipe(source)
    assert not plan["issues"], plan
    result = run(rig, plan["application"])
    assert result["ok"], result
    assert rig.wire == [(0, wire) for wire in [
        "v2.5,1R", "c50,1R", "V30,1R", "P20,1R",
        "v25,1R", "c50,1R", "V50,1R", "P22.75,1R",
        "v10,1R", "V40,1R", "P1.25,1R",
        "v25,1R", "c200,1R", "L20,10R",
        "V375,1R", "D12.75,1R", "V875,1R", "D10,1R", "V875,1R", "A0R"]]
    assert result["liquid_settings"]["resolved_recipe"]["phase_settings"] == source["phase_settings"]
    # Recommendation is documentary: no >100 cutoff cap, delay, reset or NVRAM save.
    assert [event["operation"] for event in result["events"]].count("delay") == 1
    assert all(event["reported_applied"]["readback"] is None
               for event in result["events"] if event["operation"] != "delay")


def test_exact_correction_points_and_explicit_function():
    correction = {"kind": "points", "points": [{"target_ul": 20, "commanded_ul": 22.75}]}
    assert Decimal(correction_at_target(20, correction)) == Decimal("22.75")
    with pytest.raises(ValueError, match="no interpolation"):
        correction_at_target(21, correction)
    assert Decimal(correction_at_target("0.1", {"kind": "affine", "scale": "1.03", "offset_ul": "0.02"})) == Decimal("0.123")
