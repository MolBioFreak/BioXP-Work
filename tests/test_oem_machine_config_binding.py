
from pathlib import Path


def _write_bundle(root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    (root / "config.xml").write_text(
        """<?xml version="1.0"?>
<BioXPCommonLib>
  <GenBot>
    <SerialNumber GenBot="206" />
    <Config Version="3" GripperVersion="1" TroughVersion="1" />
    <Calibration Calibrated="1" m_cal_software="5.0.0.1" m_cal_tool="50010-01A" m_cal_reversion="0" />
    <CameraInstalled Camera="1" Cameracalibrated="True" />
    <Server Password="changeit" Host="customer.example" />
  </GenBot>
  <CalibrationFactors>
    <Offsets m_originOffsetG="4450" m_GripperClosePOS="27350" m_GripperOpenPOS="31400" m_GripperOpenWide="32400" m_TCDoorOpen="18500" m_TCDoorStallGuardThreshold="6" m_TC_DOOR_VELOCITY="50" m_TC_DOOR_ACCELERATION="20" m_TC_DOOR_MAX_CURRENT="31" m_Z_MOTOR_MAX_CURRENT_DOWN="25" m_Z_MOTOR_MAX_CURRENT_UP="31" m_Z_MOTOR_STALL_GUARD_THRESHOLD="3" />
  </CalibrationFactors>
  <AxisLimits>
    <X_limit minSteps="0" maxSteps="90263" />
    <Y_limit minSteps="0" maxSteps="102956" />
    <Z_limit minSteps="0" maxSteps="160000" />
    <G_limit minSteps="0" maxSteps="15000" />
  </AxisLimits>
  <PositionTable>
    <LOC_MS x="26213" y="9241" zLow="83407" zDelta="37400" inc_factor="1" />
    <LOC_PARK x="1506" y="71" zLow="114092" zDelta="114092" inc_factor="0" />
    <CAMERA_OFFSET x="3499" y="-7744" zLow="3145" zDelta="6842" inc_factor="0" />
  </PositionTable>
</BioXPCommonLib>
""".strip()
    )
    (root / "Operation_parameters.xml").write_text("<OperationParameters><SelfTest>true</SelfTest></OperationParameters>")
    (root / "InspectionSettings.xml").write_text("<InspectionSettings />")
    (root / "processtime.xml").write_text("<processTime><park>5.4</park></processTime>")
    (root / "calreference.xml").write_text("<CalibrationReferences />")


def test_parse_oem_machine_config_bundle_extracts_real_attribute_shape(tmp_path):
    from src.bioxp.oem_config import parse_oem_machine_config_bundle

    _write_bundle(tmp_path)
    result = parse_oem_machine_config_bundle(tmp_path)

    assert result["ok"] is True
    assert result["runtime_binding"] == "read_only"
    assert result["machine_calibrated"] is True
    assert result["config"]["serial_redacted"] == "[REDACTED]"
    assert result["config"]["server_redacted"]["Password"] == "[REDACTED]"
    assert result["config"]["server_redacted"]["Host"] == "[REDACTED]"
    assert result["config"]["config"]["GripperVersion"] == 1
    assert result["config"]["camera"]["Cameracalibrated"] is True
    assert result["config"]["axis_limits"]["x"]["max_steps"] == 90263
    assert result["config"]["axis_limits"]["y"]["max_steps"] == 102956
    assert result["config"]["position_table_count"] == 3
    assert result["config"]["position_table"][1]["name"] == "LOC_PARK"
    assert result["files"]["config_xml"]["sha256"]
    assert result["files"]["operation_parameters_xml"]["role"] == "operation_parameters_xml"


def test_machine_config_diff_flags_axis_changes_but_not_matching_z_current_defaults(tmp_path):
    from src.bioxp.oem_config import parse_oem_machine_config_bundle

    _write_bundle(tmp_path)
    result = parse_oem_machine_config_bundle(tmp_path)
    diff = result["diff_vs_source_defaults"]

    assert diff["axis_limits_changed_from_prior_defaults"]["x"]["machine_config"]["max_steps"] == 90263
    assert diff["axis_limits_changed_from_prior_defaults"]["y"]["machine_config"]["max_steps"] == 102956
    assert "z" not in diff["axis_limits_changed_from_prior_defaults"]
    assert "Z_MOTOR_MAX_CURRENT_UP" not in diff["critical_constants_changed_from_source_defaults"]
    assert diff["gripper_version"] == 1


def test_load_oem_config_reads_attribute_based_gripper_and_camera_fields(tmp_path):
    from src.bioxp.oem_config import load_oem_config

    _write_bundle(tmp_path)
    loaded = load_oem_config(tmp_path / "config.xml")

    assert loaded["status"] == "loaded"
    assert loaded["fields"]["GripperVersion"] == "1"
    assert loaded["fields"]["CameraCalibrated"] == "True"
    assert loaded["fields"]["Calibrated"] == "1"
    assert loaded["axis_limits"]["x"]["max_steps"] == 90263
