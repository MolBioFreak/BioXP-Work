
from bioxp.oem_command_contracts import get_command_contract, command_contracts


def test_motor_move_left_contract_is_source_anchored_and_byte_exact():
    c = get_command_contract("ClassMotor.MoveLeft")
    assert c.source_file.endswith("ClassMotor.cs")
    assert c.source_lines == "74-115"
    assert c.command_template == [2, 0, "axis", "speed_be[3]", "speed_be[2]", "speed_be[1]", "speed_be[0]"]
    assert c.success_reply_byte_1 == 100
    assert c.motion_commanded is True


def test_switch_readback_contracts_are_source_anchored_no_motion():
    left = get_command_contract("ClassMotor.queryLeftSwitchStatus")
    right = get_command_contract("ClassMotor.queryRightSwitchStatus")
    assert left.command_template == [6, 9, "axis", 0, 0, 0, 0]
    assert right.command_template == [6, 10, "axis", 0, 0, 0, 0]
    assert left.motion_commanded is False
    assert right.motion_commanded is False
    assert left.success_reply_byte_1 == 100
    assert left.reply_value_byte == 6
    assert left.active_reply_value == 1


def test_contract_inventory_has_core_oem_readback_and_control_methods():
    names = {c.name for c in command_contracts()}
    assert {"ClassMotor.MoveLeft", "ClassMotor.MoveRight", "ClassMotor.StopMotor", "ClassMotor.setHome", "ClassMotor.queryActualPosition", "ClassMotor.queryMotorSpeed", "ClassMotor.queryLeftSwitchStatus", "ClassMotor.queryRightSwitchStatus"}.issubset(names)
