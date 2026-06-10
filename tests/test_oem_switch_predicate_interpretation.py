from src.bioxp.oem_switch_audit import OfflineSwitchAuditFixture, interpret_home_predicate, run_switch_audit


def test_interpret_z_home_predicate_is_implementation_mapped_but_blocked():
    snap = {"gap9_left": {"value": 0}, "gap10_right": {"value": 1}, "home_query": {"value": 0}}
    pred = interpret_home_predicate("z", snap)
    assert pred["home_switch"] == "gap9_left"
    assert pred["active_value"] == 1
    assert pred["confidence"] == "implementation_mapped"
    assert pred["is_home_now"] is False
    assert pred["homing_enable_state"] == "blocked_until_live_verified"
    assert "z_direction_reversal_is_quarantined_until_live_verified" in pred["blockers"]


def test_switch_audit_artifact_includes_predicate_blockers(tmp_path):
    result = run_switch_audit(OfflineSwitchAuditFixture(), axes=["z"], mode="status", artifact_root=tmp_path)
    assert result["ok"] is True
    assert result["homing_allowed"] is False
    assert result["axes"][0]["interpreted"]["confidence"] == "implementation_mapped"
    assert result["predicate_blockers"][0]["axis"] == "z"
    assert (tmp_path / "switch_audit.json").exists()


def test_api_switch_activity_distinguishes_raw_masked_right_switch():
    from types import SimpleNamespace

    from src.bioxp.api import _switch_activity_from_switches

    tester = SimpleNamespace(MOTOR_SWITCH_ACTIVE_VALUE=1)
    switches = {
        "left_state": 0,
        "right_state": 1,
        "left_disabled": False,
        "right_disabled": True,
        "left_raw_active": False,
        "right_raw_active": True,
    }
    activity = _switch_activity_from_switches(tester, 5, 0, switches)

    assert activity["right_raw_active"] is True
    assert activity["right_disabled"] is True
    assert activity["right_active"] is False
    assert activity["left_active"] is False
