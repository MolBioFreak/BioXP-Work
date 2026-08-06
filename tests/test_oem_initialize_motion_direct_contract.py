from src.bioxp.oem_homing_spec import get_program


def test_initialize_motion_spec_preserves_direct_control_lib_order_and_tip_branches():
    """ControlLib.initializeMotion:8797-8856 is modeled, never inferred from Linux routes."""
    program = get_program("initialize_motion")
    assert {step.source.sha256 for step in program.steps} == {
        "f69b3529dcb9723c705ac55ecb3f035010cc294d3891de096c165bb20116f6c2"
    }
    ids = [step.step_id for step in program.steps]

    assert ids == [
        "initializeMotion.stop_scripts",
        "initializeMotion.clear_forceabort",
        "initializeMotion.initializeMotors",
        "initializeMotion.thermal_door_closed",
        "initializeMotion.queryTipStatus.initial",
        "initializeMotion.sleep.after_tip_query",
        "initializeMotion.openThermalDoor.tip_exists",
        "initializeMotion.thermal_door_open.tip_exists",
        "initializeMotion.tip_loaded.tip_exists",
        "initializeMotion.scriptmoveTo.tip_exists",
        "initializeMotion.updateLocation.tip_exists",
        "initializeMotion.ejectAllTips.tip_exists",
        "initializeMotion.moveZ.tip_exists",
        "initializeMotion.moveX.tip_exists",
        "initializeMotion.queryTipStatus.after_eject",
        "initializeMotion.sleep.after_eject_query",
        "initializeMotion.pause_scripts.eject_failed",
        "initializeMotion.error_event.eject_failed",
        "initializeMotion.throw.eject_failed",
        "initializeMotion.return.eject_failed_without_handler",
        "initializeMotion.tip_dirty_false",
        "initializeMotion.tip_loaded_false.after_eject",
        "initializeMotion.sleep.before_initiate_group",
        "initializeMotion.initiateGroup.initial",
        "initializeMotion.checkedPipetteStatus.initial",
        "initializeMotion.initiateGroup.retry",
        "initializeMotion.checkedPipetteStatus.retry",
        "initializeMotion.error_event.eject_failed_after_retry",
        "initializeMotion.throw.eject_failed_after_retry",
        "initializeMotion.tip_loaded_false.no_tip",
        "initializeMotion.catch.error_event",
        "initializeMotion.catch.rethrow",
        "initializeMotion.catch.swallow_without_handler",
    ]
    steps = {step.step_id: step for step in program.steps}
    assert steps["initializeMotion.sleep.after_tip_query"].wait_ms == 500
    assert steps["initializeMotion.sleep.after_eject_query"].wait_ms == 100
    assert steps["initializeMotion.sleep.before_initiate_group"].wait_ms == 2
    assert steps["initializeMotion.scriptmoveTo.tip_exists"].params == {
        "from_location": 28,
        "from_well": 0,
        "to_location": 6,
        "column": 0,
        "row": 0,
    }
    assert steps["initializeMotion.throw.eject_failed"].branch_condition == (
        "TipExist && TipExistAfterEject && errorEvent!=null"
    )
    retry_condition = "TipExist && !TipExistAfterEject && PipetteStatusInitialFailed"
    assert steps["initializeMotion.initiateGroup.retry"].branch_condition == retry_condition
    assert steps["initializeMotion.checkedPipetteStatus.retry"].branch_condition == retry_condition
    retry_failure_condition = f"{retry_condition} && PipetteStatusRetryFailed"
    assert steps["initializeMotion.error_event.eject_failed_after_retry"].branch_condition == retry_failure_condition
    assert steps["initializeMotion.throw.eject_failed_after_retry"].branch_condition == retry_failure_condition
