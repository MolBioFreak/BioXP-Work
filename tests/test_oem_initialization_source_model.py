from src.bioxp.oem_initialization import (
    SOURCE_ANCHORS,
    build_machine_calibration_manifest,
    oem_initialization_phase_catalog,
)


def _val(row):
    return row["value"]


def test_oem_initialization_source_anchors_cover_controller_layers():
    for key in [
        "genbot_initialize_system",
        "control_initialize_motion",
        "control_initialize_motors",
        "homexy",
        "thermal_door",
        "z_home",
        "board_go_home",
        "machine_settings",
    ]:
        assert key in SOURCE_ANCHORS
        assert SOURCE_ANCHORS[key].file
        assert SOURCE_ANCHORS[key].lines


def test_phase_catalog_models_init_as_controller_not_single_home_call():
    names = [row["name"] for row in oem_initialization_phase_catalog()]
    assert names[:4] == ["accepted", "initial_check", "interlock_prepare", "initialize_without_motion"]
    assert "z_reference" in names
    assert "home_xy" in names
    assert "door_state_capture" in names
    assert "final_readiness" in names
    assert len(names) >= 10


def test_machine_manifest_prefers_original_ssd_config_values():
    manifest = build_machine_calibration_manifest()
    assert manifest["ok"] is True
    assert manifest["config_path"] and manifest["config_path"].endswith("config.xml")
    assert _val(manifest["thermal_door"]["TCDoorOpen"]) == 18500
    assert manifest["thermal_door"]["TCDoorOpen"]["source"] == "original_ssd_machine_config"
    assert manifest["thermal_door"]["TCDoorOpen"]["fallback"] is False

    assert _val(manifest["gripper"]["originOffsetG"]) == 4450
    assert _val(manifest["gripper"]["GripperClosePOS"]) == 27350
    assert _val(manifest["gripper"]["GripperOpenPOS"]) == 31400
    assert _val(manifest["gripper"]["GripperOpenWide"]) == 32400

    assert manifest["axis_limits"]["x"]["value"]["max_steps"] == 90263
    assert manifest["axis_limits"]["y"]["value"]["max_steps"] == 102956
    assert manifest["axis_limits"]["z"]["value"]["max_steps"] == 160000
    assert manifest["axis_limits"]["g"]["value"]["max_steps"] == 15000
    assert all(not row["fallback"] for row in manifest["axis_limits"].values())
