"""Recipe air gaps survive the OEM plain-search lift/DispenseAll cleanup."""
from copy import deepcopy

import pytest

# Deny hardware and network even when this module is run directly on the host.
import tests.z_stop_offline_guard  # noqa: F401
from bioxp.pipette.cavro_liquid import compile_liquid_recipe


def _recipe(mode="by_level", air=True, settings=False):
    request = {
        "mode": "single", "channels": [0, 1, 2, 3], "timeout_ms": 5000,
        "target_liquid_ul": "5", "commanded_aspiration_ul": "5.5",
        "aspiration_speed_ul_s": 10, "aspiration_delay_ms": 200,
        "leading_air": {"volume_ul": "1", "speed_ul_s": "20"} if air else None,
        "trailing_air": {"volume_ul": "2", "speed_ul_s": "30"},
        "before_leading_air": [{"operation": "delay", "duration_ms": 11}],
        "before_liquid": [{"operation": "delay", "duration_ms": 22}],
        "after_liquid": [], "before_dispense": [], "after_dispense": [],
        "dispense_segments": [{"volume_ul": "5.5", "speed_ul_s": "10"}],
        "final_empty_speed_ul_s": 40, "final_empty_before": [], "multi": None,
        "liquid_detection": {"location_id": 3, "mode": mode} if mode else None,
    }
    if settings:
        request["phase_settings"] = {"leading_air": {}, "aspirate": {}, "trailing_air": {}, "dispense": {}}
    return request


def _compile(request):
    compiled = compile_liquid_recipe(request)
    assert not compiled["issues"], compiled["issues"]
    return compiled["application"]


@pytest.mark.parametrize("mode", ["by_level", "together", "one_at_a_time"])
def test_air_immediately_after_plain_search_before_surface_draw(mode):
    request = _recipe(mode)
    original = deepcopy(request)
    ops = _compile(request)["operations"]
    assert request == original
    assert [o["operation"] for o in ops] == [
        "delay", "delay", "liquid_search", "leading_air", "surface_aspirate",
        "delay", "trailing_air", "dispense", "empty",
    ]
    assert [o["duration_ms"] for o in ops[:2]] == [11, 22]
    assert ops[3] == {"operation": "leading_air", "channels": [0, 1, 2, 3],
                      "timeout_ms": 5000, "volume_ul": "1", "speed_ul_s": "20"}
    assert "on_contact" not in ops[2]  # retain ordinary OEM search cleanup
    assert ops[4]["mode"] == mode and ops[4]["volume_ul"] == "5.5"


def test_phase_settings_follow_search_and_retain_phase_order_and_real_indices():
    application = _compile(_recipe(settings=True))
    ops = application["operations"]
    assert [o["operation"] for o in ops] == [
        "delay", "delay", "liquid_search", "settings", "leading_air", "settings",
        "surface_aspirate", "delay", "settings", "trailing_air", "settings", "dispense", "empty",
    ]
    ledger = application["liquid_settings"]["phase_settings_emission"]
    assert {phase: item["operation_index"] for phase, item in ledger.items()} == {
        "leading_air": 3, "aspirate": 5, "trailing_air": 8, "dispense": 10,
    }
    assert all(item["emitted"] and item["reported_applied"] is None for item in ledger.values())


@pytest.mark.parametrize("mode", ["on_contact", None])
def test_on_contact_and_no_detection_keep_original_order(mode):
    ops = _compile(_recipe(mode, settings=True))["operations"]
    assert [o["operation"] for o in ops] == [
        "delay", "settings", "leading_air", "delay", "settings",
        "liquid_search" if mode else "aspirate", "delay", "settings", "trailing_air",
        "settings", "dispense", "empty",
    ]
    if mode:
        assert ops[5]["on_contact"]["volume_ul"] == "5.5"
        assert not any(o["operation"] == "surface_aspirate" for o in ops)


@pytest.mark.parametrize("mode", ["by_level", "together", "one_at_a_time"])
def test_plain_search_without_air_keeps_original_settings_order(mode):
    application = _compile(_recipe(mode, air=False, settings=True))
    assert [o["operation"] for o in application["operations"]] == [
        "delay", "delay", "settings", "liquid_search", "surface_aspirate", "delay",
        "settings", "trailing_air", "settings", "dispense", "empty",
    ]
    assert application["liquid_settings"]["phase_settings_emission"]["leading_air"]["emitted"] is False
