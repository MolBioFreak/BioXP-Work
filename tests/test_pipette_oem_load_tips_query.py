"""Source-shaped four-channel query for the OEM loadTips caller; no CAN hardware."""

import pytest

from bioxp.pipette.models import PipetteCommandError
from tests.test_protocol_oem_air_native import collection


@pytest.mark.parametrize(
    "loaded,expected_count,exists,missing",
    [
        ([True] * 4, 4, True, False),
        ([False] * 4, 0, False, True),
        ([False, True, False, True], 2, True, True),
    ],
)
def test_load_tips_query_source_order_count_and_cached_properties(
    loaded, expected_count, exists, missing,
):
    group, drivers, trace = collection()
    group._tip_type = 50
    for driver, value in zip(drivers, loaded):
        driver.loaded = value
    result = group.query_tip_status_for_oem_load_tips()
    assert trace == [("query", channel) for channel in range(4)]
    assert result["source_return"] == expected_count
    assert result["source_tip_exists"] is exists
    assert result["source_tip_missing"] is missing
    assert result["tip_type"] == 50
    assert result["ok"] and result["source_return_completed"]
    assert result["hardware_postcondition_verified"]
    assert result["physical_effect_verified"] is False
    assert [row["channel"] for row in result["channels"]] == list(range(4))
    assert [row["result"]["source_return"] for row in result["channels"]] == [
        1 if value else 2 for value in loaded
    ]
    assert [transport._tip_loaded for transport in group._transports] == loaded


def test_source_completed_unverified_readback_is_not_a_new_refusal():
    group, drivers, trace = collection()
    def source_default():
        trace.append(("query", 1))
        return {"ok": False, "source_return_completed": True, "source_return": 2,
                "source_tip_loaded": False, "hardware_truth_level": "unavailable"}
    drivers[1].query_tip_status = source_default
    result = group.query_tip_status_for_oem_load_tips()
    assert trace == [("query", channel) for channel in range(4)]
    assert result["ok"] and result["source_return_completed"]
    assert result["source_return"] == 3
    assert result["source_tip_exists"] is True
    assert result["source_tip_missing"] is True
    assert result["hardware_postcondition_verified"] is False
    assert result["channels"][1]["result"]["ok"] is False


def test_source_count_is_from_returns_while_tip_properties_use_cached_state():
    group, drivers, trace = collection()
    def mismatched_source():
        trace.append(("query", 0))
        return {"ok": False, "source_return_completed": True, "source_return": 0,
                "source_tip_loaded": True, "hardware_truth_level": "unavailable"}
    drivers[0].query_tip_status = mismatched_source
    result = group.query_tip_status_for_oem_load_tips()
    assert result["source_return"] == 3
    assert result["source_tip_exists"] is True
    assert result["source_tip_missing"] is False
    assert [row[0] for row in trace] == ["query"] * 4


def test_exception_preserves_partial_observations_and_does_not_query_later_channels():
    group, drivers, trace = collection()
    drivers[0].loaded = False
    drivers[2].fail_query = True
    with pytest.raises(PipetteCommandError) as caught:
        group.query_tip_status_for_oem_load_tips()
    assert trace == [("query", 0), ("query", 1), ("query", 2)]
    assert caught.value.details["failed_channel"] == 2
    assert caught.value.details["source_return_completed"] is False
    assert caught.value.details["source_exception"] is True
    observed = caught.value.details["observed_channels"]
    assert [row["channel"] for row in observed] == [0, 1]
    assert [row["result"]["source_return"] for row in observed] == [2, 1]
    assert [transport._tip_loaded for transport in group._transports] == [False, True, False, False]
