from src.bioxp.oem_switch_audit import FakeSwitchAuditHardware, interpret_home_predicate, run_switch_audit


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
    result = run_switch_audit(FakeSwitchAuditHardware(), axes=["z"], mode="status", artifact_root=tmp_path)
    assert result["ok"] is True
    assert result["homing_allowed"] is False
    assert result["axes"][0]["interpreted"]["confidence"] == "implementation_mapped"
    assert result["predicate_blockers"][0]["axis"] == "z"
    assert (tmp_path / "switch_audit.json").exists()
