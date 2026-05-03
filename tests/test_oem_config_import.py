from pathlib import Path


def test_find_oem_config_reports_missing_with_searched_roots(tmp_path):
    from src.bioxp.oem_config import find_oem_config

    result = find_oem_config([tmp_path / "missing-a", tmp_path / "missing-b"])

    assert result["status"] == "missing"
    assert len(result["searched_roots"]) == 2
    assert result["path"] is None


def test_parse_oem_config_extracts_startup_fields(tmp_path):
    from src.bioxp.oem_config import load_oem_config

    cfg = tmp_path / "config.xml"
    cfg.write_text(
        """
        <Config>
          <StartMode>Service</StartMode>
          <GripperVersion>2</GripperVersion>
          <XAxisMax>100000</XAxisMax>
          <YAxisMax>200000</YAxisMax>
          <ZMotorMaxCurrentUp>44</ZMotorMaxCurrentUp>
          <ZMotorStallGuardThreshold>9</ZMotorStallGuardThreshold>
          <ThermalDoorMaxVelocity>700</ThermalDoorMaxVelocity>
        </Config>
        """.strip()
    )

    result = load_oem_config(cfg)

    assert result["status"] == "loaded"
    assert result["fields"]["StartMode"] == "Service"
    assert result["fields"]["GripperVersion"] == "2"
    assert result["fields"]["XAxisMax"] == "100000"
    assert result["fields"]["ZMotorStallGuardThreshold"] == "9"
