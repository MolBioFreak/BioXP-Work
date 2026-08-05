from src.bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from src.bioxp.usb_driver import BioXpTester


ACK = {"status": 100, "value": 0}


class _ProfileTester:
    def __init__(self, *, mismatch=False):
        self.values = {}
        self.calls = []
        self.mismatch = mismatch

    def _motion_oem_axis_profile(self, axis):
        assert axis == "z"
        return {"board": 4, "motor": 1, "down_current": 31}

    def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides):
        self.calls.append(("verify-profile", axis, dict(expected_overrides)))
        return {"ok": True}

    def motor_oem_verify_motion_interlock(self):
        return {"ok": True}

    def motor_set_axis_param(self, board, param, value, *, motor):
        self.calls.append(("write", board, motor, param, value))
        self.values[param] = value
        return {"ok": True, "ack": dict(ACK)}

    def motor_get_axis_param(self, board, param, *, motor):
        value = self.values[param] + 1 if self.mismatch else self.values[param]
        return {"ok": True, "ack": dict(ACK), "value": value}


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


def test_z_mask_reconciliation_restores_machine_bound_right_disabled_left_enabled():
    tester = _ProfileTester()
    tester.values[12] = 0
    tester.values[13] = 1
    adapter = _adapter(tester)

    result = adapter.z_reconcile_switch_masks()

    assert result["ok"] is True
    assert result["machine_bound_expected"] == {12: 1, 13: 0}
    assert tester.values[12] == 1
    assert tester.values[13] == 0


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
    values = {4: 1791, 5: 576, 6: 31, 205: 3, 12: 1, 13: 0}
    driver.motor_get_axis_param = lambda board, param, *, motor: {  # type: ignore[method-assign]
        "ok": True,
        "ack": {"status": 100},
        "value": values[param],
    }

    result = driver.motor_oem_require_no_motion_profile("z")

    assert result["ok"] is True
    assert result["readbacks"][12]["value"] == 1
    assert result["readbacks"][13]["value"] == 0


def test_z_current_max_oem_sentinel_100_selects_machine_down_current():
    adapter = _adapter(_ProfileTester())

    result = adapter.z_set_current_max(100)

    assert result["ok"] is True
    assert (result["param"], result["value"]) == (6, 31)
