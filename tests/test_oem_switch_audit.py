def test_switch_audit_status_mode_never_moves_axes(tmp_path):
    from src.bioxp.oem_switch_audit import run_switch_audit, OfflineSwitchAuditFixture

    hw = OfflineSwitchAuditFixture()
    result = run_switch_audit(hw, axes=["z"], mode="status", artifact_root=tmp_path)

    assert result["ok"] is True
    assert result["mode"] == "status"
    assert hw.move_calls == []
    axis = result["axes"][0]
    assert axis["axis"] == "z"
    assert "gap9_left" in axis
    assert "gap10_right" in axis
    assert axis["interpreted"]["confidence"] == "implementation_mapped"
    assert axis["interpreted"]["homing_enable_state"] == "blocked_until_live_verified"
    assert result["homing_allowed"] is False
    assert result["predicate_blockers"][0]["axis"] == "z"
    assert (tmp_path / "switch_audit.json").exists()


def test_switch_audit_rejects_live_probe_without_artifact_root():
    from src.bioxp.oem_switch_audit import run_switch_audit, OfflineSwitchAuditFixture

    result = run_switch_audit(OfflineSwitchAuditFixture(), axes=["z"], mode="live_probe", artifact_root=None)

    assert result["ok"] is False
    assert "artifact_root" in result["error"]


def test_switch_audit_rejects_unknown_axes():
    from src.bioxp.oem_switch_audit import run_switch_audit, OfflineSwitchAuditFixture

    result = run_switch_audit(OfflineSwitchAuditFixture(), axes=["bogus"], mode="status")

    assert result["ok"] is False
    assert "unknown axes" in result["error"]


def test_predicate_gate_requires_confidence():
    from src.bioxp.oem_switch_audit import require_confident_predicates

    result = require_confident_predicates({"z": {"confidence": "unknown"}, "x": {"confidence": "source_anchored"}}, ["z", "x"])

    assert result["ok"] is False
    assert result["blockers"][0]["axis"] == "z"
