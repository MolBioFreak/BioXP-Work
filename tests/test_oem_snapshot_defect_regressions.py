from __future__ import annotations

import pytest

from src.bioxp import api


class FakeSnapshotTester:
    BOARD_DECK = 1
    BOARD_THERMAL = 6
    BOARD_CHILLER = 7
    THERMAL_BANK_NEST = 0
    THERMAL_BANK_LID = 1
    CHILLER_BANK_RC = 0
    CHILLER_BANK_OC = 1
    CHILLER_TEMP_AXES = (("rc_temp_c", 0), ("oc_temp_c", 1))

    def __init__(self) -> None:
        self.calls: list[tuple[int, int, int, int, int]] = []

    def query_only_tmcl(self, board: int, command: int, cmd_type: int, motor: int, value: int):
        self.calls.append((board, command, cmd_type, motor, value))
        return {"board": board, "cmd": command, "status": 100, "value": 0}


def test_power_snapshot_uses_exact_oem_24v_scalar_contract():
    tester = FakeSnapshotTester()

    result = api._hardware_collectors(tester)["power"](None)

    assert result["rail_24v"]["payload_raw"] == 0
    assert result["rail_24v"]["oem_status"] == 100
    assert result["rail_24v"]["oem_scalar"] == 0
    assert result["rail_24v"]["oem_no24v"] is False
    assert result["rail_24v"]["safety_valid"] is True
    assert result["safety_valid"] is True
    assert result["oem_power_ok"] is True


def test_thermal_snapshot_queries_only_named_oem_getters_per_bank():
    tester = FakeSnapshotTester()

    api._query_aux_snapshot(tester, "thermal")

    gp_calls = {
        (cmd_type, motor)
        for board, command, cmd_type, motor, _value in tester.calls
        if board == tester.BOARD_THERMAL and command == 10
    }
    assert gp_calls == {
        (4, 0),
        (4, 1),
        (7, 0),
        (8, 0),
        (13, 0),
        (21, 0),
        (23, 0),
        (7, 1),
        (8, 1),
        (13, 1),
        (23, 1),
    }
    assert (21, 1) not in gp_calls
    assert all(cmd_type != 22 for cmd_type, _motor in gp_calls)


def test_chiller_snapshot_queries_only_named_oem_getters_per_bank():
    tester = FakeSnapshotTester()

    api._query_aux_snapshot(tester, "chiller")

    gp_calls = {
        (cmd_type, motor)
        for board, command, cmd_type, motor, _value in tester.calls
        if board == tester.BOARD_CHILLER and command == 10
    }
    assert gp_calls == {
        (4, 0),
        (7, 0),
        (8, 0),
        (21, 0),
        (4, 1),
        (7, 1),
        (8, 1),
        (21, 1),
    }


def test_default_hardware_snapshot_does_not_claim_unqueried_camera_evidence():
    assert "camera" not in api.DEFAULT_HARDWARE_SNAPSHOT_DOMAINS


def test_explicit_camera_domain_fails_honestly_without_cached_camera_evidence(monkeypatch):
    monkeypatch.setattr(api, "_camera_probe_cache", None)
    monkeypatch.setattr(api, "_camera_session", None)
    tester = FakeSnapshotTester()

    with pytest.raises(RuntimeError, match="explicit POST /camera/probe"):
        api._hardware_collectors(tester)["camera"](None)
