from src.bioxp.oem_compat.machine_state import (
    OemDefaultParameters,
    OemMachineState,
    PSEUDO_Z_HOME_HIGH,
    PSEUDO_Z_HOME_LOW,
)


def test_default_parameters_match_oem_pseudo_z_transitions():
    params = OemDefaultParameters()
    assert params.pseudo_z_home == PSEUDO_Z_HOME_LOW
    assert params.force_to_high_home().pseudo_z_home == PSEUDO_Z_HOME_HIGH
    assert params.gantry_load(tiploaded="TIP_200UL").pseudo_z_home == PSEUDO_Z_HOME_HIGH
    assert params.gantry_load(tiploaded=None, plateloaded=None).pseudo_z_home == PSEUDO_Z_HOME_LOW
    assert params.gantry_load(plateloaded="BIO_SECURITY_COVER").pseudo_z_home == PSEUDO_Z_HOME_LOW
    assert params.gantry_load(plateloaded="OTHER_PLATE").pseudo_z_home == PSEUDO_Z_HOME_HIGH


def test_machine_state_confirm_axis_and_payload_are_pure():
    state = OemMachineState.from_query(
        current_location_id="LOC_MS",
        current_x=10,
        current_y=20,
        current_z=70000,
        tip_loaded=True,
        tip_dirty=False,
        tip_location=2,
        clean_path=True,
        gripper_confirmed=True,
    )
    assert state.confirm_axis("gripper") is True
    assert state.confirm_axis("g") is True
    assert state.confirm_axis("x") is False
    payload = state.to_payload()
    assert payload["current_position"] == {"x": 10, "y": 20, "z": 70000}
    assert payload["default_parameters"]["pseudo_z_home"] == PSEUDO_Z_HOME_LOW


def test_machine_state_force_and_gantry_load_return_new_instances():
    state = OemMachineState()
    high = state.with_force_to_high_home()
    assert state.default_parameters.pseudo_z_home == PSEUDO_Z_HOME_LOW
    assert high.default_parameters.pseudo_z_home == PSEUDO_Z_HOME_HIGH
    loaded = state.with_gantry_load(tiploaded="TIP")
    assert loaded.default_parameters.pseudo_z_home == PSEUDO_Z_HOME_HIGH
