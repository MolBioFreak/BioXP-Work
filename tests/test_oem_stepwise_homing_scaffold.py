from src.bioxp.oem_startup_program import BioXpStartupHardware
from test_oem_initialize_motion_diagnostic import MotionDiagTester


def test_startup_homing_stepwise_plan_is_oem_order_and_non_motion():
    result = BioXpStartupHardware(lambda: MotionDiagTester()).startup_homing_stepwise(mode="shadow", step="plan", execute=False)
    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert result["monolithic_homing_blocked"] is False
    assert result["not_a_replacement_sequence"] is True
    assert [row["step"] for row in result["steps"]] == [
        "z-home",
        "gripper-clear",
        "gripper-home",
        "x-home",
        "x-park-6000",
        "y-home",
        "door-home",
        "y-set-home",
    ]


def test_startup_homing_stepwise_live_z_dry_plan_preserves_oem_first_step():
    result = BioXpStartupHardware(lambda: MotionDiagTester()).startup_homing_stepwise(mode="live", step="z-home", execute=False)
    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert result["step"]["axis"] == "z"
    assert result["step"]["step"] == "z-home"
