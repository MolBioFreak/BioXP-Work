from __future__ import annotations

import importlib
import sys


def _load_api(monkeypatch):
    monkeypatch.setenv("BIOXP_SKIP_USB_CONNECT", "1")
    sys.modules.pop("src.bioxp.api", None)
    return importlib.import_module("src.bioxp.api")


def test_initialize_without_motion_action_preserves_literal_white_evidence_and_never_adds_red(monkeypatch):
    api = _load_api(monkeypatch)

    class Tester:
        def oem_initialize_without_motion_test_case(self):
            return {
                "ok": True,
                "physical_motion": False,
                "test_case": "oem.initializeMotorsWithoutMotion.live_parity.v1",
                "test_case_note": "Source-faithful command-sequence test; it does not home or command axis movement.",
                "transcript": [
                    {"label": "setColor.white.red", "ok": True},
                    {"label": "setColor.white.green", "ok": True},
                    {"label": "setColor.white.blue", "ok": True},
                ],
            }

        def strip_set_rgb(self, *args, **kwargs):  # pragma: no cover - forbidden OEM invention
            raise AssertionError(f"no wrapper LED write is allowed: {args} {kwargs}")

    monkeypatch.setattr(api, "_get_tester", lambda: Tester())

    result = api._initialize_without_motion_action()

    assert result["ok"] is True
    assert [row["label"] for row in result["motor_current_verification"]["transcript"]] == [
        "setColor.white.red",
        "setColor.white.green",
        "setColor.white.blue",
    ]
    assert "led_red" not in result
    assert result["test_case"] == "oem.initializeMotorsWithoutMotion.live_parity.v1"
    assert result["physical_motion"] is False
