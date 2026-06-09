
from pathlib import Path

from bioxp.oem_parity_config import load_oem_parity_config, OemParityConfig


def test_default_config_is_explicit_source_defaults_not_machine_calibrated():
    cfg = load_oem_parity_config(None)
    assert isinstance(cfg, OemParityConfig)
    assert cfg.calibration_source == "source_defaults"
    assert cfg.machine_calibrated is False
    assert cfg.values["X_MOTOR_MAX_POSITION"] == 91919
    assert cfg.values["G_SAFE_IDLE_CURRENT"] == 10
    assert "config.xml_not_bound" in cfg.blockers


def test_xml_config_binding_marks_machine_calibrated_and_preserves_unknowns(tmp_path):
    xml = tmp_path / "config.xml"
    xml.write_text('''<Configuration>
        <X_MOTOR_MAX_POSITION>90000</X_MOTOR_MAX_POSITION>
        <GripperVersion>1</GripperVersion>
        <CameraCalibrated>true</CameraCalibrated>
    </Configuration>''')
    cfg = load_oem_parity_config(xml)
    assert cfg.calibration_source == str(xml)
    assert cfg.machine_calibrated is True
    assert cfg.values["X_MOTOR_MAX_POSITION"] == 90000
    assert cfg.values["GripperVersion"] == 1
    assert cfg.values["CameraCalibrated"] is True
    assert cfg.values["G_SAFE_IDLE_CURRENT"] == 10
    assert cfg.unknown_keys == []
