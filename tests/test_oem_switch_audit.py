def test_switch_audit_status_mode_never_moves_axes(tmp_path):
    from src.bioxp.oem_switch_audit import run_switch_audit, FakeSwitchAuditHardware

    hw = FakeSwitchAuditHardware()
    result = run_switch_audit(hw, axes=["z"], mode="status", artifact_root=tmp_path)

    assert result["ok"] is True
    assert result["mode"] == "status"
    assert hw.move_calls == []
    axis = result["axes"][0]
    assert axis["axis"] == "z"
    assert "gap9_left" in axis
    assert "gap10_right" in axis
    assert axis["interpreted"]["confidence"] == "unknown"
    assert (tmp_path / "switch_audit.json").exists()


def test_switch_audit_rejects_live_probe_without_artifact_root():
    from src.bioxp.oem_switch_audit import run_switch_audit, FakeSwitchAuditHardware

    result = run_switch_audit(FakeSwitchAuditHardware(), axes=["z"], mode="live_probe", artifact_root=None)

    assert result["ok"] is False
    assert "artifact_root" in result["error"]
