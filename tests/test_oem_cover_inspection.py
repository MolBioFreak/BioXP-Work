from __future__ import annotations

import pytest

from src.bioxp.oem_vision_acceptance import OemVisionReceiptError, evaluate_inspect_cover_receipt


def receipt(detections, relocations):
    return {
        "deck_inspection": True,
        "screen_resolution_high": True,
        "inspection_methods": {
            "17": "InspectOutputLocation(1)",
            "19": "checkRCCover",
            "20": "checkCoverStorage",
            "18": "checkCoverStorage",
        },
        "cover_detected": {str(key): value for key, value in detections.items()},
        "relocations": relocations,
        "final_cover_locations": {"output": 18, "reagent": 20},
        "door_open_verified": True,
        "all_leds_off_acknowledged": True,
        "camera_session_released": True,
        "inspection_log_only": False,
    }


def test_cover_inspection_exact_two_in_chillers_requires_source_order_relocation():
    value = receipt(
        {17: True, 19: True, 20: False, 18: False},
        [
            {"cover": "output", "from": 17, "to": 20},
            {"cover": "reagent", "from": 19, "to": 18},
        ],
    )
    result = evaluate_inspect_cover_receipt(value)
    assert result["ok"] is True
    assert result["cover_count"] == 2
    assert result["expected_relocations"] == value["relocations"]


def test_cover_inspection_storage_state_needs_no_relocation():
    result = evaluate_inspect_cover_receipt(receipt({17: False, 19: False, 20: True, 18: True}, []))
    assert result["ok"] is True
    assert result["expected_relocations"] == []


def test_cover_inspection_one_active_one_stored_matches_oem_pairing():
    value = receipt(
        {17: True, 19: False, 20: True, 18: False},
        [{"cover": "output", "from": 17, "to": 18}],
    )
    assert evaluate_inspect_cover_receipt(value)["ok"] is True


def test_cover_count_failures_and_inspection_override_do_not_admit_production():
    short = receipt({17: True, 19: False, 20: False, 18: False}, [])
    short["door_open_verified"] = False
    short["final_cover_locations"] = None
    result = evaluate_inspect_cover_receipt(short)
    assert result["ok"] is False
    assert result["error_status"] == "SHORT_CHILLER_COVER"

    over = receipt({17: True, 19: True, 20: True, 18: False}, [])
    over["inspection_log_only"] = True
    over["door_open_verified"] = False
    over["final_cover_locations"] = None
    result = evaluate_inspect_cover_receipt(over)
    assert result["oem_effective_pass"] is True
    assert result["production_admission_pass"] is False


def test_cover_inspection_rejects_wrong_method_or_relocation_order():
    wrong_method = receipt({17: False, 19: False, 20: True, 18: True}, [])
    wrong_method["inspection_methods"]["17"] = "checkChillerCover"
    with pytest.raises(OemVisionReceiptError, match="inspection_methods"):
        evaluate_inspect_cover_receipt(wrong_method)

    wrong_move = receipt(
        {17: True, 19: True, 20: False, 18: False},
        [
            {"cover": "reagent", "from": 19, "to": 18},
            {"cover": "output", "from": 17, "to": 20},
        ],
    )
    with pytest.raises(OemVisionReceiptError, match="relocations"):
        evaluate_inspect_cover_receipt(wrong_move)


def test_cover_inspection_disabled_is_a_typed_noop():
    value = receipt({17: False, 19: False, 20: False, 18: False}, [])
    value.update(
        deck_inspection=False,
        inspection_methods={},
        cover_detected={},
        final_cover_locations=None,
        door_open_verified=False,
    )
    result = evaluate_inspect_cover_receipt(value)
    assert result["ok"] is True
    assert result["outcome"] == "deck_inspection_disabled"
    assert result["physical_effect_verified"] is False
