import pytest

from src.bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from src.bioxp.usb_driver import BioXpTester


ACK = {"status": 100, "value": 0}


class _ProfileTester:
    def __init__(self, *, mismatch=False):
        self.values = {}
        self.calls = []
        self.mismatch = mismatch
        self.position = 2500
        self.speed = 0
        self.profile_verifier_result = {"ok": True}

    def _motion_oem_axis_profile(self, axis):
        assert axis == "z"
        return {"board": 4, "motor": 1, "run_current": 31}

    def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides):
        self.calls.append(("verify-profile", axis, dict(expected_overrides)))
        return dict(self.profile_verifier_result)

    def motor_oem_verify_motion_interlock(self):
        return {"ok": True}

    def motor_set_axis_param(self, board, param, value, *, motor):
        self.calls.append(("write", board, motor, param, value))
        self.values[param] = value
        return {"ok": True, "ack": dict(ACK)}

    def motor_get_axis_param(self, board, param, *, motor):
        value = self.values[param] + 1 if self.mismatch else self.values[param]
        return {"ok": True, "ack": dict(ACK), "value": value}

    def motor_get_position(self, board, *, motor):
        self.calls.append(("gap", board, motor, 1))
        return {"ok": True, "ack": dict(ACK), "position": self.position}

    def motor_get_speed(self, board, *, motor):
        self.calls.append(("gap", board, motor, 3))
        return {"ok": True, "ack": dict(ACK), "speed": self.speed}


def _adapter(tester):
    adapter = Serial206ProductionPrimitiveAdapter.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = tester
    adapter._z_profile_overrides = {}
    return adapter


def test_z_profile_controls_use_exact_oem_defaults_and_restore_speed():
    tester = _ProfileTester()
    adapter = _adapter(tester)

    speed = adapter.z_set_max_speed(0)
    acceleration = adapter.z_set_max_acc(0)
    vmax = adapter.z_set_vmax(0)
    current = adapter.z_set_current_max(None)
    restored = adapter.z_restore_original_speed()

    assert [row["ok"] for row in (speed, acceleration, vmax, current, restored)] == [True] * 5
    assert (speed["param"], speed["value"]) == (4, 1791)
    assert (acceleration["param"], acceleration["value"]) == (5, 576)
    assert (vmax["param"], vmax["value"]) == (4, 1791)
    assert (current["param"], current["value"]) == (6, 31)
    assert (restored["param"], restored["value"]) == (4, 1791)
    assert adapter._z_profile_overrides == {4: 1791, 5: 576, 6: 31}
    assert all(row["physical_motion"] is False for row in (speed, acceleration, vmax, current, restored))


def test_z_profile_control_rejects_readback_mismatch_without_recording_override():
    adapter = _adapter(_ProfileTester(mismatch=True))

    result = adapter.z_set_vmax(500)

    assert result["ok"] is False
    assert result["failure"] == "z_profile_parameter_readback_mismatch"
    assert adapter._z_profile_overrides == {}


def test_z_mask_reconciliation_restores_literal_oem_enabled_masks():
    tester = _ProfileTester()
    tester.values.update({9: 0, 10: 0, 12: 1, 13: 1})
    adapter = _adapter(tester)

    result = adapter.z_reconcile_switch_masks()

    assert result["ok"] is True
    assert result["machine_bound_expected"] == {12: 0, 13: 0}
    assert result["classification"] == "controller_state_repair_for_literal_oem_baseline"
    assert tester.values[12] == 0
    assert tester.values[13] == 0


def test_z_mask_repair_fails_when_raw_limit_remains_active_after_enablement():
    tester = _ProfileTester()
    tester.values.update({9: 0, 10: 1, 12: 1, 13: 1})
    adapter = _adapter(tester)

    result = adapter.z_reconcile_switch_masks()

    assert result["ok"] is False
    assert result["failure"] == "raw_active_z_limit_after_mask_convergence"
    assert result["raw_active_limits"] == [10]


def test_z_move_preflight_blocks_an_active_enabled_limit():
    tester = _ProfileTester()
    tester.values.update({9: 0, 10: 1, 12: 0, 13: 0})
    adapter = _adapter(tester)

    result = adapter._z_oem_move_preflight()

    assert result["ok"] is False
    assert result["failure"] == "raw_active_z_limit_after_mask_convergence"
    assert result["raw_active_limits"] == [10]


def test_z_absolute_move_does_not_dispatch_with_active_enabled_limit():
    tester = _ProfileTester()
    tester.values.update({9: 0, 10: 1, 12: 0, 13: 0})
    adapter = _adapter(tester)

    result = adapter.z_move_absolute(requested_position_steps=4000, pseudo_home_steps=0)

    assert result["ok"] is False
    assert result["failure"] == "raw_active_z_limit_after_mask_convergence"
    assert result["command_issued"] is False


def test_z_profile_verifier_accepts_only_reconciled_persistent_masks():
    driver = object.__new__(BioXpTester)
    driver._oem_no_motion_profiles_ready = {"z"}
    driver._oem_active_board_lifecycle_generation = 7
    driver._oem_no_motion_profile_generations = {"z": 7}
    driver._oem_no_motion_profile_fingerprints = {"z": {}}
    driver._motion_oem_axis_profile = lambda axis: {  # type: ignore[method-assign]
        "board": 4,
        "motor": 1,
        "speed": 1791,
        "acc": 576,
        "run_current": 31,
        "stall_guard": 3,
    }
    values = {4: 1791, 5: 576, 6: 31, 205: 3, 12: 0, 13: 0}
    driver.motor_get_axis_param = lambda board, param, *, motor: {  # type: ignore[method-assign]
        "ok": True,
        "ack": {"status": 100},
        "value": values[param],
    }

    result = driver.motor_oem_require_no_motion_profile("z")

    assert result["ok"] is True
    assert result["readbacks"][12]["value"] == 0
    assert result["readbacks"][13]["value"] == 0


def test_z_profile_path_stops_when_exact_profile_verification_fails():
    tester = _ProfileTester()
    tester.profile_verifier_result = {"ok": False, "failure": "z_profile_readback_mismatch"}
    adapter = _adapter(tester)

    with pytest.raises(RuntimeError, match="Z profile verification failed"):
        adapter._z_profile()


def test_z_current_max_oem_sentinel_100_selects_machine_down_current():
    adapter = _adapter(_ProfileTester())

    result = adapter.z_set_current_max(100)

    assert result["ok"] is True
    assert (result["param"], result["value"]) == (6, 31)


def test_z_terminal_status_rejects_typed_but_non_oem_switch_mask():
    tester = _ProfileTester()
    tester.values.update({1: 2500, 3: 0, 4: 1791, 5: 576, 6: 31, 9: 0, 10: 0, 12: 1, 13: 0, 205: 3})
    adapter = _adapter(tester)

    result = adapter.z_terminal_status()

    assert result["ok"] is False
    assert result["profile_verified"] is True
    assert result["switch_mask_verified"] is False
    assert result["failure"] == "z_oem_profile_or_switch_mask_mismatch"
