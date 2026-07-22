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
        def motor_oem_initialize_without_motion(self):
            return {
                "ok": True,
                "physical_motion": False,
                "operations": [
                    {"name": "setColor.white.r", "ok": True},
                    {"name": "setColor.white.g", "ok": True},
                    {"name": "setColor.white.b", "ok": True},
                ],
            }

        def strip_set_rgb(self, *args, **kwargs):  # pragma: no cover - forbidden OEM invention
            raise AssertionError(f"no wrapper LED write is allowed: {args} {kwargs}")

    monkeypatch.setattr(api, "_get_tester", lambda: Tester())

    result = api._initialize_without_motion_action()

    assert result["ok"] is True
    assert [row["name"] for row in result["led_white"]] == [
        "setColor.white.r",
        "setColor.white.g",
        "setColor.white.b",
    ]
    assert "led_red" not in result
    assert result["physical_motion"] is False
