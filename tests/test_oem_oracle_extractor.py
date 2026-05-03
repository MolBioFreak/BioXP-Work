from pathlib import Path

from src.bioxp.oem_compat.oracle_extractor import (
    extract_location_id_enum,
    extract_position_table,
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


def test_write_position_table_binding_creates_source_backed_schema(tmp_path):
    enum_path = tmp_path / "locationID.cs"
    source_path = tmp_path / "ClassBioXPSettings.cs"
    output_path = tmp_path / "binding.json"
    enum_path.write_text(LOCATION_ENUM, encoding="utf-8")
    source_path.write_text(POSITION_TABLE, encoding="utf-8")

    write_position_table_binding(
        class_bioxp_settings_path=source_path,
        location_id_path=enum_path,
        output_path=output_path,
    )

    text = output_path.read_text(encoding="utf-8")
    assert '"schema": "bioxp-oem-binding-v1"' in text
    assert '"position_table"' in text
    assert '"LOC_MS"' in text
    assert '"source_type": "decompiled_csharp"' in text
