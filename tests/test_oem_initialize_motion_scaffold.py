
from src.bioxp.oem_initialize_motion_scaffold import initialize_motion_parity_plan


def test_initialize_motion_plan_includes_pipette_and_vision_blockers():
    plan = initialize_motion_parity_plan()
    assert plan["program"] == "initialize_motion"
    assert plan["physical_motion"] is False
    assert plan["opened_usb"] is False
    step_ids = [step["step_id"] for step in plan["steps"]]
    assert "initializeMotors" in step_ids
    assert "queryTipStatus.initial" in step_ids
    assert "scriptmoveTo.tip_cleanup" in step_ids
    assert "ejectAllTips" in step_ids
    assert "checkedPipetteStatus.retry" in step_ids
    assert "vision.camera_calibrated_gate" in step_ids
    assert "pipette_cleanup_not_live_ported" in plan["blockers"]
    assert "vision_inspection_not_oem_equivalent" in plan["blockers"]
