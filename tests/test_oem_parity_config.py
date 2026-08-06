
from pathlib import Path

from bioxp.oem_parity_config import load_oem_parity_config, OemParityConfig


def test_unbound_config_fails_closed_without_source_default_projection(monkeypatch):
    from bioxp import oem_machine_bundle

    monkeypatch.setattr(oem_machine_bundle, "_active_snapshot", None)
    cfg = load_oem_parity_config(None)
    assert isinstance(cfg, OemParityConfig)
    assert cfg.calibration_source == "unbound_fail_closed"
    assert cfg.machine_calibrated is False
    assert cfg.values == {}
    assert cfg.blockers == ["OemMachineSnapshot_not_bound"]


def test_explicit_xml_is_diagnostic_only_and_never_marks_machine_calibrated(tmp_path):
    xml = tmp_path / "config.xml"
    xml.write_text('''<Configuration>
        <X_MOTOR_MAX_POSITION>90000</X_MOTOR_MAX_POSITION>
        <GripperVersion>1</GripperVersion>
        <CameraCalibrated>true</CameraCalibrated>
    </Configuration>''')
    cfg = load_oem_parity_config(xml)
    assert cfg.calibration_source == f"non_authoritative_diagnostic:{xml}"
    assert cfg.machine_calibrated is False
    assert cfg.values["X_MOTOR_MAX_POSITION"] == 90000
    assert cfg.values["GripperVersion"] == 1
    assert cfg.values["CameraCalibrated"] is True
    assert cfg.values["G_SAFE_IDLE_CURRENT"] == 10
    assert cfg.blockers == [
        "explicit_XML_diagnostic_is_not_accepted_live_authority",
        "source_defaults_present_in_diagnostic_projection",
    ]
    assert cfg.unknown_keys == []
