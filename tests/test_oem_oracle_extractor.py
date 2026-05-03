from pathlib import Path

from src.bioxp.oem_compat.oracle_extractor import (
    extract_calibration_reference,
    extract_default_parameters,
    extract_location_id_enum,
    extract_position_table,
    extract_process_time,
    write_position_table_binding,
)


LOCATION_ENUM = """
public enum locationID
{
    LOC_MS,
    LOC_OC,
    LOC_RC = 3,
    WASTE_BIN = 6,
    CAMERA_OFFSET = 31,
    UNKNOWN
}
"""


POSITION_TABLE = """
protected Dictionary<locationID, positionStruct> m_positionTable = new Dictionary<locationID, positionStruct>
{
    {
        (locationID)0,
        new positionStruct
        {
            name = "Magnetic Tray",
            x = 25743,
            y = 9399,
            zLow = 81553,
            zDelta = 37400,
            inc_factor = 1
        }
    },
    {
        (locationID)3,
        new positionStruct
        {
            name = "Reagent Tray",
            x = 67294,
            y = 45338,
            zLow = 90385,
            zHigh = 47285,
            zDelta = 43100,
            inc_factor = 1
        }
    }
};
"""


def test_extract_location_id_enum_preserves_ordinals_and_names(tmp_path):
    enum_path = tmp_path / "locationID.cs"
    enum_path.write_text(LOCATION_ENUM, encoding="utf-8")

    enum = extract_location_id_enum(enum_path)

    assert enum[0] == "LOC_MS"
    assert enum[1] == "LOC_OC"
    assert enum[3] == "LOC_RC"
    assert enum[6] == "WASTE_BIN"
    assert enum[31] == "CAMERA_OFFSET"
    assert enum[32] == "UNKNOWN"


def test_extract_position_table_computes_z_high_from_delta(tmp_path):
    source_path = tmp_path / "ClassBioXPSettings.cs"
    source_path.write_text(POSITION_TABLE, encoding="utf-8")

    entries = extract_position_table(source_path, {0: "LOC_MS", 3: "LOC_RC"})

    assert entries["LOC_MS"] == {
        "ordinal": 0,
        "name": "Magnetic Tray",
        "x": 25743,
        "y": 9399,
        "zLow": 81553,
        "zHigh": 44153,
        "zDelta": 37400,
        "inc_factor": 1,
    }
    assert entries["LOC_RC"]["zHigh"] == 47285


def test_extract_calibration_reference_parses_part_numbers_and_fluid_offsets(tmp_path):
    cal_path = tmp_path / "calreference.xml.deploy"
    cal_path.write_text(
        """<?xml version=\"1.0\"?>
<CalibrationReferences>
  <PartNumber_50010-01>
    <Reference REVERSION=\"0\" />
    <Parameters STRIP_WELL=\"4296\" THERMAL_CYCLER=\"24900\" PARK=\"123138\" />
  </PartNumber_50010-01>
  <FluidReference>
    <ref_location REVERSION=\"0\" FLUID_TC_OFFSET=\"5178\" FLUID_RC_OFFSET=\"5300\" />
  </FluidReference>
</CalibrationReferences>
""",
        encoding="utf-8",
    )

    calibration = extract_calibration_reference(cal_path)

    assert calibration["part_numbers"]["PartNumber_50010-01"]["STRIP_WELL"] == 4296
    assert calibration["part_numbers"]["PartNumber_50010-01"]["PARK"] == 123138
    assert calibration["fluid_reference"]["FLUID_TC_OFFSET"] == 5178


def test_extract_process_time_parses_command_seconds(tmp_path):
    process_path = tmp_path / "ProcessTime.xml.deploy"
    process_path.write_text(
        """<?xml version=\"1.0\"?>
<BioXPCommonLib><processTime><mov process=\"7.12\" /><wait process=\"62.502\" /></processTime></BioXPCommonLib>
""",
        encoding="utf-8",
    )

    process_time = extract_process_time(process_path)

    assert process_time == {"mov": 7.12, "wait": 62.502}


def test_extract_default_parameters_parses_numeric_constants(tmp_path):
    default_path = tmp_path / "DefaultParameters.cs"
    default_path.write_text(
        """
public static class DefaultParameters
{
    public const int X_MOTOR_VELOCITY = 1700;
    public const double CHILLER_COOL_RATE = -0.025;
    public const int PSUDO_Z_HOME_LOW = 65000;
}
""",
        encoding="utf-8",
    )

    constants = extract_default_parameters(default_path)

    assert constants["X_MOTOR_VELOCITY"] == 1700
    assert constants["CHILLER_COOL_RATE"] == -0.025
    assert constants["PSUDO_Z_HOME_LOW"] == 65000


def test_write_position_table_binding_creates_source_backed_schema(tmp_path):
    enum_path = tmp_path / "locationID.cs"
    source_path = tmp_path / "ClassBioXPSettings.cs"
    cal_path = tmp_path / "calreference.xml.deploy"
    process_path = tmp_path / "ProcessTime.xml.deploy"
    default_path = tmp_path / "DefaultParameters.cs"
    output_path = tmp_path / "binding.json"
    enum_path.write_text(LOCATION_ENUM, encoding="utf-8")
    source_path.write_text(POSITION_TABLE, encoding="utf-8")
    cal_path.write_text("<CalibrationReferences><FluidReference><ref_location FLUID_TC_OFFSET=\"5178\" /></FluidReference></CalibrationReferences>", encoding="utf-8")
    process_path.write_text("<BioXPCommonLib><processTime><mov process=\"7.12\" /></processTime></BioXPCommonLib>", encoding="utf-8")
    default_path.write_text("public const int X_MOTOR_VELOCITY = 1700;", encoding="utf-8")

    write_position_table_binding(
        class_bioxp_settings_path=source_path,
        location_id_path=enum_path,
        output_path=output_path,
        calreference_path=cal_path,
        process_time_path=process_path,
        default_parameters_path=default_path,
    )

    text = output_path.read_text(encoding="utf-8")
    assert '"schema": "bioxp-oem-binding-v1"' in text
    assert '"position_table"' in text
    assert '"calibration_reference"' in text
    assert '"process_time"' in text
    assert '"motion_constants"' in text
    assert '"LOC_MS"' in text
    assert '"source_type": "decompiled_csharp"' in text
