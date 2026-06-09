
import importlib


def test_fresh_spec_module_is_no_usb_and_lists_required_programs():
    spec = importlib.import_module("bioxp.oem_homing_spec")
    assert spec.NO_USB_IMPORTS is True
    names = set(spec.program_names())
    assert {
        "initialize_motors_without_motion",
        "initialize_motors",
        "manual_home_x",
        "manual_home_y",
        "manual_home_z",
        "manual_home_g",
        "manual_home_door",
        "home_axis",
        "home_xy",
        "move_z_home",
        "home_gz",
        "door_search_home",
        "rehome",
        "initialize_motion",
    } <= names


def test_initialize_motors_spec_preserves_oem_order_and_g_current_invariant():
    from bioxp.oem_homing_spec import get_program
    prog = get_program("initialize_motors")
    assert prog.live_allowed_default is False
    assert [s.step_id for s in prog.steps[:12]] == [
        "z.axisSearchHome",
        "g.setMaxCurrent.before_clear",
        "g.clear.moveSteps",
        "g.axisSearchHome",
        "x.axisSearchHome",
        "x.setHome",
        "x.setSpeed.restore",
        "x.park_6000",
        "y.axisSearchHome",
        "door.doorSearchHome",
        "y.setHome.final",
        "g.restore_current.version1",
    ]
    assert "g_current_invariant" in prog.required_artifact_fields
    assert any("G_CURRENT_IDLE_SAFE" in d for step in prog.steps for d in step.safety_deviations)


def test_without_motion_spec_includes_non_motor_side_effects():
    from bioxp.oem_homing_spec import get_program
    prog = get_program("initialize_motors_without_motion")
    ids = {s.step_id for s in prog.steps}
    assert {"waitForBoard", "turnOffHeater", "setChillerPWM", "setChillerCoolRate.OC", "setChillerCoolRate.RC", "setTCHeatCoolRate", "setColor.white"} <= ids
    assert prog.live_allowed_default is False


def test_move_z_home_home_gz_and_initialize_motion_are_explicit_blocked_programs():
    from bioxp.oem_homing_spec import get_program
    assert get_program("move_z_home").blockers
    assert get_program("home_gz").blockers
    init_motion = get_program("initialize_motion")
    assert any("pipette" in b for b in init_motion.blockers)
    assert any("vision" in b for b in init_motion.blockers)
