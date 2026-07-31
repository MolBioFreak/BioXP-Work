from __future__ import annotations

import pytest

from src.bioxp.oem_vision_acceptance import OemVisionReceiptError, evaluate_check_camera_receipt


def receipt(*, first=True, second_attempted=False, second=False):
    return {
        "camera_owner": "oem_full_lifecycle",
        "settings_item": 4,
        "led_white_acknowledged": True,
        "door_closed_verified": True,
        "attempts": [
            {
                "location_id": 23,
                "offset_x": 4738,
                "offset_y": 0,
                "check_label": first,
                "failure_snapshot_written": not first,
            }
        ] + ([{
            "location_id": 23,
            "offset_x": 1895,
            "offset_y": 0,
            "check_label": second,
            "failure_snapshot_written": not second,
        }] if second_attempted else []),
        "location_23_persisted": True,
        "all_leds_off_acknowledged": True,
        "gantry_park_verified": True,
    }


def test_camera_accepts_first_label_and_exact_cleanup():
    result = evaluate_check_camera_receipt(receipt())
    assert result["ok"] is True
    assert result["receipt_validation_pass"] is True
    assert result["production_admission_pass"] is False
    assert result["physical_effect_verified"] is False
    assert result["provider_live_bound"] is False
    assert result["attempt_count"] == 1
    assert result["camera_session_disposition"] == "not_released_by_CheckCamera"


def test_camera_accepts_exact_second_attempt_after_first_failure():
    result = evaluate_check_camera_receipt(receipt(first=False, second_attempted=True, second=True))
    assert result["ok"] is True
    assert result["attempt_count"] == 2


def test_camera_rejects_both_label_failures_but_requires_failure_snapshots():
    result = evaluate_check_camera_receipt(receipt(first=False, second_attempted=True, second=False))
    assert result["ok"] is False
    assert result["failure"] == "camera_label_not_detected"
    assert result["cleanup_verified"] is True
    assert result["physical_effect_verified"] is False


def test_camera_rejects_wrong_order_or_unnecessary_second_attempt():
    wrong = receipt()
    wrong["attempts"][0]["offset_x"] = 1895
    with pytest.raises(OemVisionReceiptError, match="first camera attempt"):
        evaluate_check_camera_receipt(wrong)

    unnecessary = receipt(first=True, second_attempted=True, second=True)
    with pytest.raises(OemVisionReceiptError, match="must stop after first success"):
        evaluate_check_camera_receipt(unnecessary)


def test_camera_cleanup_or_truthy_flags_fail_closed():
    bad_cleanup = receipt()
    bad_cleanup["all_leds_off_acknowledged"] = False
    result = evaluate_check_camera_receipt(bad_cleanup)
    assert result["ok"] is False
    assert "all_leds_off_not_verified" in result["cleanup_failures"]

    truthy = receipt()
    truthy["attempts"][0]["check_label"] = 1
    with pytest.raises(OemVisionReceiptError, match="exact bool"):
        evaluate_check_camera_receipt(truthy)
