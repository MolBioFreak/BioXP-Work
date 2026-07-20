from pathlib import Path


def test_motor_builds_same_raw_tmcl_frame_shape_as_existing_linux_driver():
    from src.bioxp.oem_compat.motor import Motor
    from src.bioxp.oem_compat.transport import DryRunTransport

    transport = DryRunTransport()
    motor = Motor(board_id=0x05, motor=0, transport=transport, label="X")

    motor.set_max_speed(1700)

    frame = transport.frames[-1]
    assert frame.sidh == 0x05
    assert frame.command == 5  # SAP
    assert frame.cmd_type == 4  # max speed
    assert frame.motor == 0
    assert frame.value == 1700
    assert bytes(frame.raw) == bytes([0x7E, 0, 0, 0, 5, 8, 5, 4, 0, 0, 0, 6, 0xA4, 0, 0xC0, 0x7E])


def test_board_axis_map_locks_oem_physical_assignments():
    from src.bioxp.oem_compat.boards import BioXPBoards

    boards = BioXPBoards.dry_run()

    assert boards.axis("x").board_id == 0x05
    assert boards.axis("x").motor == 0
    assert boards.axis("y").board_id == 0x04
    assert boards.axis("y").motor == 0
    assert boards.axis("z").board_id == 0x04
    assert boards.axis("z").motor == 1
    assert boards.axis("g").board_id == 0x04
    assert boards.axis("g").motor == 2
    assert boards.axis("door").board_id == 0x06
    assert boards.axis("door").motor == 0


def test_initialize_motors_without_motion_emits_oem_ordered_params():
    from src.bioxp.oem_compat.control_interface import BioXPControlInterface

    ci = BioXPControlInterface.dry_run()

    trace = ci.initialize_motors_without_motion()

    names = [op.name for op in trace.operations]
    assert names[:4] == ["wait_for_board", "turn_off_heater", "set_chiller_pwm", "set_chiller_rates"]

    x_ops = [op for op in trace.operations if op.axis == "x" and op.name.startswith("set_")]
    assert [(op.name, op.value) for op in x_ops[:4]] == [
        ("set_speed_acc", {"speed": 1700, "acc": 350}),
        ("set_run_current", 31),
        ("set_standby_current", 10),
        ("set_stall_guard", 16),
    ]

    # Every real motor/config operation should still be dry-run traffic only.
    assert ci.transport.opened_usb is False
    assert len(ci.transport.frames) >= 20


def test_startup_homing_trace_preserves_oem_axis_sequence_and_x_offset_move():
    from src.bioxp.oem_compat.control_interface import BioXPControlInterface

    ci = BioXPControlInterface.dry_run()

    trace = ci.startup_homing()

    compact = [(op.name, op.axis, op.value) for op in trace.operations]
    assert ("move_relative", "g", 10000) in compact
    ordered_homes = [(op.name, op.axis, op.value) for op in trace.operations if op.name == "home_axis"]
    assert ordered_homes == [
        ("home_axis", "z", 1791),
        ("home_axis", "g", 200),
        ("home_axis", "x", 250),
        ("home_axis", "y", 250),
    ]
    assert ("door_search_home", "door", {"speed": 600, "stall_guard": 6}) in compact
    assert ("set_home", "x", 0) in compact
    assert ("set_max_speed", "x", 1700) in compact
    assert ("move_absolute", "x", 6000) in compact


def test_control_lib_startup_dry_run_calls_oem_compatible_control_interface():
    from src.bioxp.oem_compat.control_lib import BioXPControlLib

    control = BioXPControlLib.dry_run()

    report = control.startup(run_homing=True)

    assert report.ok is True
    assert report.mode == "dry_run"
    assert report.physical_motion is False
    assert "initialize_motors_without_motion" in report.trace_names
    assert "startup_homing" in report.trace_names
    assert control.transport.opened_usb is False


def test_oem_xml_script_parser_reads_real_demo_script_commands():
    from src.bioxp.oem_compat.scripts import OemScript

    script_path = Path(__file__).resolve().parents[1] / "scripts" / "demo.xml"
    parsed = OemScript.from_file(script_path)

    assert parsed.root_tag == "WpfGenBotCommonLib"
    assert len(parsed.commands) > 20
    verbs = [cmd.verb for cmd in parsed.commands]
    assert "LED" in verbs
    assert "WAIT" in verbs
    assert any(v in verbs for v in ("TCD", "MP", "MC", "PP"))
    assert all(cmd.raw for cmd in parsed.commands)


def test_control_lib_execute_script_dry_run_returns_explicit_planned_actions_not_done():
    from src.bioxp.oem_compat.control_lib import BioXPControlLib
    from src.bioxp.oem_compat.scripts import OemScript

    control = BioXPControlLib.dry_run()
    script = OemScript.from_text(
        """
        <WpfGenBotCommonLib>
          <script>
            <line1 cmd="LED 0 0 0" />
            <line2 cmd="WAIT 1" />
            <line3 cmd="TCD DO" />
          </script>
        </WpfGenBotCommonLib>
        """
    )

    result = control.execute_script(script)

    assert result.mode == "dry_run"
    assert result.executed is False
    assert [a.verb for a in result.actions] == ["LED", "WAIT", "TCD"]
    assert all(a.status == "planned" for a in result.actions)
