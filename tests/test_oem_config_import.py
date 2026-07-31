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
    assert result["axis_limits"]["x"]["max_steps"] == 91919
    assert result["axis_limits"]["y"]["max_steps"] == 95247
    assert result["axis_limits"]["z"]["max_steps"] == 160000
    assert result["axis_limits"]["g"]["max_steps"] == 15000
    assert result["fields"]["ZMotorStallGuardThreshold"] == "9"


def test_parse_oem_config_extracts_axislimits_overrides(tmp_path):
    from src.bioxp.oem_config import harmonized_motion_config, load_oem_config

    cfg = tmp_path / "config.xml"
    cfg.write_text(
        """
        <Config>
          <StartMode>Service</StartMode>
          <GripperVersion>2</GripperVersion>
          <AxisLimits>
            <X_limit minSteps="-10" maxSteps="91000" />
            <Y_limit minSteps="5" maxSteps="92000" />
            <Z_limit minSteps="0" maxSteps="170000" />
            <G_limit minSteps="0" maxSteps="15500" />
          </AxisLimits>
        </Config>
        """.strip()
    )

    loaded = load_oem_config(cfg)
    assert loaded["status"] == "loaded"
    assert loaded["axis_limits"]["x"] == {
        "min_steps": -10,
        "max_steps": 91000,
        "source": "config_xml_axislimits",
        "config_tag": "X_limit",
    }
    assert loaded["axis_limits"]["y"]["min_steps"] == 5
    assert loaded["axis_limits"]["z"]["max_steps"] == 170000
    assert loaded["axis_limits"]["g"]["max_steps"] == 15500

    harmonized = harmonized_motion_config(loaded)
    assert harmonized["source"] == "non_authoritative_diagnostic_config_xml"
    assert harmonized["axis_limits"]["x"]["max_steps"] == 91000
    assert harmonized["source_evidence"]["absolute_clamp"]
    assert harmonized["live_ready"] is False


def test_harmonized_motion_config_does_not_promote_defaults_when_diagnostic_config_missing(tmp_path):
    from src.bioxp.oem_config import find_oem_config, harmonized_motion_config

    missing = find_oem_config([tmp_path / "missing"])
    harmonized = harmonized_motion_config(missing)

    assert missing["status"] == "missing"
    assert harmonized["source"] == "diagnostic_config_unavailable"
    assert harmonized["axis_limits"] == {}
    assert harmonized["source_evidence"]["config_loader"]
    assert harmonized["live_ready"] is False
    assert harmonized["deck_coordinate_extents"]["x"]["max_steps"] == 91919
    assert harmonized["deck_coordinate_extents"]["y"]["max_steps"] == 95247
    assert harmonized["axis_limit_diagnostics"]["x"]["configured_max_steps"] is None
    assert harmonized["axis_limit_diagnostics"]["y"]["configured_max_steps"] is None
