from __future__ import annotations

from typing import Any, cast

import pytest

from src.bioxp.lifecycle_state import CanonicalLifecycleOwner, LifecycleStateError


class _InitialCheckHardware:
    def __init__(self):
        self.calls = []

    def set_led_rgb(self, r, g, b):
        self.calls.append(("led", r, g, b))
        return {"ok": True}

    def query_door(self):
        self.calls.append(("door",))
        return {"value": 1, "ack": {"status": 100}}

    def query_latch(self):
        self.calls.append(("latch",))
        return {"value": 0, "ack": {"status": 100}}

    def set_solenoid(self, value):
        self.calls.append(("solenoid", value))
        return {"ok": True}

    def query_voltage(self):
        self.calls.append(("voltage",))
        return {
            "payload_raw": 0,
            "reply_present": True,
            "transport_outcome": "reply",
            "oem_status": 100,
        }

    def deactivate_boards(self):
        self.calls.append(("deactivate",))
        return {"ok": True}

    def activate_boards(self):
        self.calls.append(("activate",))
        return {"ok": True}


def _owner_ready_for_initial_check():
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason="test_transport_ready")
    owner.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    owner.run_stage("initialization_without_motion", lambda: {"ok": True})
    return owner


def test_production_lifecycle_adapter_is_strict_fail_fast_and_returns_aggregate_success():
    from src.bioxp import api

    class Tester:
        BOARD_DECK = 5

        def __init__(self):
            self.calls = []
            self.activation_status = 100

        @staticmethod
        def _tmcl_success(ack):
            return isinstance(ack, dict) and ack.get("status") == 100

        @staticmethod
        def _oem_board_activation_map_success(rows):
            return bool(rows) and all(
                isinstance(row, dict) and (row.get("ack") or {}).get("status") == 100
                for row in rows.values()
            )

        def strip_set_rgb(self, *args, **kwargs):
            self.calls.append(("led", args, kwargs))
            return {"ok": True}

        def query_only_tmcl(self, *args):
            self.calls.append(("query", args))
            return {"status": 100, "value": 1}

        def deck_io_set_type(self, *args):
            self.calls.append(("solenoid", args))
            return {"ok": True}

        def deactivate_boards(self, **kwargs):
            self.calls.append(("deactivate", kwargs))
            return {4: {"ack": {"status": self.activation_status}}}

        def activate_boards(self, **kwargs):
            self.calls.append(("activate", kwargs))
            return {4: {"ack": {"status": self.activation_status}}}

    tester = Tester()
    adapter = api._LifecycleHardware(cast(Any, tester))

    assert adapter.set_led_rgb(255, 255, 255)["ok"] is True
    assert tester.calls[-1][2] == {
        "reconnect_first": False,
        "activate_first": False,
        "fail_fast": True,
    }
    assert adapter.query_door()["ok"] is True
    assert adapter.query_latch()["ok"] is True
    assert adapter.query_voltage()["ok"] is True
    assert adapter.deactivate_boards()["ok"] is True
    assert adapter.activate_boards()["ok"] is True
    assert ("deactivate", {"expect_reply": True, "fail_fast": True}) in tester.calls
    assert ("activate", {"expect_reply": True, "fail_fast": True}) in tester.calls

    tester.activation_status = 2
    failed = adapter.activate_boards()
    assert failed["ok"] is False
    assert failed["acks"] == {4: {"ack": {"status": 2}}}


def test_nonrepeatable_predecessor_stages_remain_single_owner_operations():
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason="test_transport_ready")
    owner.run_stage("constructor_pipette_stage", lambda: {"ok": True})

    with pytest.raises(LifecycleStateError, match="already passed"):
        owner.run_stage("constructor_pipette_stage", lambda: {"ok": True})