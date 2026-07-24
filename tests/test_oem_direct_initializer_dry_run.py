"""Direct-C# contract tests for the fresh, no-USB OEM initializer lane."""
from __future__ import annotations

from src.bioxp.oem_homing_runtime import OemHomingDryRunRuntime
from src.bioxp.oem_homing_spec import get_program


def test_initialize_motors_spec_preserves_direct_oem_waits_failure_branch_and_tail():
    """ClassControlInterface.initializeMotors:3350-3419, not a Linux reconstruction."""
    program = get_program("initialize_motors")
    ids = [step.step_id for step in program.steps]

    assert ids == [
        "z.axisSearchHome",
        "g.setMaxCurrent.before_clear",
        "g.clear.moveSteps",
        "g.axisSearchHome",
        "x.axisSearchHome",
        "x.sleep.after_home",
        "x.setHome",
        "x.setSpeed.restore",
        "x.sleep.after_speed",
        "x.park_6000",
        "y.axisSearchHome",
        "door.doorSearchHome",
        "door.open_after_failed_close",
        "door.throw_after_failed_close",
        "y.setHome.final",
        "ui.zero_calibrated_positions",
        "chiller.setCoolRate.OC",
        "chiller.setCoolRate.RC",
        "status.initialized",
        "g.restore_current.version1",
    ]
    steps = {step.step_id: step for step in program.steps}
    assert steps["x.sleep.after_home"].wait_ms == 20
    assert steps["x.sleep.after_speed"].wait_ms == 40
    assert steps["door.open_after_failed_close"].branch_condition == (
        "SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated"
    )
    assert steps["door.throw_after_failed_close"].operation == "throw"
    assert steps["status.initialized"].params == {"system_status": 1, "ready": True}


def test_dry_run_emits_source_ordered_fake_transport_trace_without_usb_or_motion():
    artifact = OemHomingDryRunRuntime().run("initialize_motors")

    assert artifact["ok"] is True
    assert artifact["opened_usb"] is False
    assert artifact["physical_motion"] is False
    assert artifact["transport_kind"] == "fake_no_usb"
    assert [row["step_id"] for row in artifact["steps_executed"]] == [
        step.step_id for step in get_program("initialize_motors").steps
    ]
    controller_rows = [
        row
        for row in artifact["steps_executed"]
        if row["transport_ack"] is not None
    ]
    assert {"z.axisSearchHome", "g.clear.moveSteps", "x.park_6000", "chiller.setCoolRate.OC"} <= {
        row["step_id"] for row in controller_rows
    }
    assert all(row["transport_ack"]["status"] == 100 for row in controller_rows)
    assert all(
        row["transport_ack"] is None
        for row in artifact["steps_executed"]
        if row["operation"] in {"sleep", "throw"}
    )
    assert artifact["controller_validated"] is False
    assert artifact["physical_effect_verified"] is False
    assert artifact["virtual_elapsed_ms"] == 60


def test_dry_run_fails_closed_on_oem_door_close_failure_branch_without_executing_transport():
    artifact = OemHomingDryRunRuntime().run(
        "initialize_motors",
        simulation={"serial_number": 206, "camera_calibrated": True, "tc_door_closed": False},
    )

    assert artifact["ok"] is False
    assert artifact["failed_closed"] is True
    assert artifact["opened_usb"] is False
    assert artifact["physical_motion"] is False
    assert artifact["failure"]["step_id"] == "door.throw_after_failed_close"


def test_dry_run_route_passes_source_branch_simulation_without_transport():
    import asyncio

    from src.bioxp.oem_homing_routes import dry_run_oem_homing_program

    artifact = asyncio.run(
        dry_run_oem_homing_program(
            "initialize_motors",
            {"simulation": {"serial_number": 206, "camera_calibrated": True, "tc_door_closed": False}},
        )
    )
    assert artifact["ok"] is False
    assert artifact["failed_closed"] is True
    assert artifact["failure"]["source_anchor"].endswith(":3387")
    assert artifact["opened_usb"] is False
    assert artifact["physical_motion"] is False


def test_fake_transport_does_not_forge_controller_ack_for_oem_host_operations():
    artifact = OemHomingDryRunRuntime().run("initialize_motion")
    rows = {row["step_id"]: row for row in artifact["steps_executed"]}

    assert rows["initializeMotion.stop_scripts"]["transport_ack"] is None
    assert rows["initializeMotion.thermal_door_closed"]["transport_ack"] is None
    assert rows["initializeMotion.queryTipStatus.initial"]["transport_ack"] is None


def test_fake_trace_preserves_oem_board_motor_identity_and_null_guard_branch():
    artifact = OemHomingDryRunRuntime().run(
        "initialize_motors", simulation={"board_present": {"z": False}}
    )
    rows = {row["step_id"]: row for row in artifact["steps_executed"]}

    assert (rows["z.axisSearchHome"]["board"], rows["z.axisSearchHome"]["motor"]) == ("head", 1)
    assert rows["z.axisSearchHome"]["execution"] == "branch_not_taken"
    assert rows["z.axisSearchHome"]["transport_ack"] is None
    assert (rows["g.clear.moveSteps"]["board"], rows["g.clear.moveSteps"]["motor"]) == ("head", 2)
    assert (rows["x.axisSearchHome"]["board"], rows["x.axisSearchHome"]["motor"]) == ("head", 0)
    assert (rows["door.doorSearchHome"]["board"], rows["door.doorSearchHome"]["motor"]) == ("thermal", 0)

    calibrated = OemHomingDryRunRuntime().run("initialize_motors", simulation={"calibrated": True})
    ui_row = next(row for row in calibrated["steps_executed"] if row["step_id"] == "ui.zero_calibrated_positions")
    assert ui_row["execution"] == "simulated"
    assert ui_row["params"] == {"x": "0", "y": "0", "z": "0", "z_write_count": 2}
    assert ui_row["transport_ack"] is None


def test_initialize_motion_fake_trace_exercises_source_stale_tip_success_and_return_branches():
    successful = OemHomingDryRunRuntime().run(
        "initialize_motion", simulation={"tip_exists": True, "tip_exists_after_eject": False}
    )
    steps = {row["step_id"]: row for row in successful["steps_executed"]}
    assert successful["ok"] is True
    assert successful["virtual_elapsed_ms"] == 602
    assert steps["initializeMotion.ejectAllTips.tip_exists"]["execution"] == "simulated"
    assert steps["initializeMotion.tip_dirty_false"]["execution"] == "simulated"
    assert steps["initializeMotion.checkedPipetteStatus.initial"]["execution"] == "simulated"
    assert steps["initializeMotion.initiateGroup.retry"]["execution"] == "branch_not_taken"

    returned = OemHomingDryRunRuntime().run(
        "initialize_motion", simulation={"tip_exists": True, "tip_exists_after_eject": True}
    )
    ids = [row["step_id"] for row in returned["steps_executed"]]
    assert returned["ok"] is True
    assert ids[-1] == "initializeMotion.return.eject_failed_without_handler"
    assert "initializeMotion.tip_dirty_false" not in ids
