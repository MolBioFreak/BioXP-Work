import json


def test_recording_transport_writes_replayable_artifact(tmp_path):
    from src.bioxp.oem_compat.frames import OemCommandFrame
    from src.bioxp.oem_compat.transport import RecordingTransport, ReplayTransport

    artifact = tmp_path / "trace.json"
    rec = RecordingTransport(artifact_path=artifact)
    rec.transmit(OemCommandFrame.from_payload(5, bytes([5, 4, 0, 0, 0, 6, 0xA4]), message="X setMaxSpeed"))
    rec.close()

    payload = json.loads(artifact.read_text())
    assert payload["format"] == "bioxp-oem-compat-trace-v1"
    assert payload["mode"] == "dry_run"
    assert payload["frames"][0]["oem_payload_hex"] == "050400000006a4"
    assert payload["frames"][0]["category"] == "motion"

    replay = ReplayTransport.from_file(artifact)
    reply = replay.transmit(OemCommandFrame.from_payload(5, bytes([5, 4, 0, 0, 0, 6, 0xA4]), message="X setMaxSpeed"))
    assert reply.synthetic is True
    assert replay.position == 1


def test_motor_methods_emit_oem_7byte_payloads_and_linux_raw_envelope():
    from src.bioxp.oem_compat.motor import Motor
    from src.bioxp.oem_compat.transport import DryRunTransport

    t = DryRunTransport()
    x = Motor(board_id=5, motor=0, transport=t, label="X")

    x.set_max_acc(350)
    x.set_max_current(99)  # OEM clamps >31
    x.disable_right_limit_switch()
    x.move_relative(-10000)
    x.set_home()

    assert t.frames[0].oem_payload == bytes([5, 5, 0, 0, 0, 1, 0x5E])
    assert t.frames[1].oem_payload == bytes([5, 6, 0, 0, 0, 0, 31])
    # OEM ClassMotor sends disable switch twice; both payloads must be present.
    assert [f.oem_payload for f in t.frames[2:4]] == [bytes([5, 12, 0, 0, 0, 0, 1])] * 2
    assert t.frames[4].oem_payload == bytes([4, 1, 0, 0xFF, 0xFF, 0xD8, 0xF0])
    assert t.frames[5].oem_payload == bytes([5, 1, 0, 0, 0, 0, 0])
    assert t.frames[0].raw[0] == 0x7E and t.frames[0].raw[-1] == 0x7E


def test_board_activation_deactivation_and_deck_io_payloads_are_oem_grounded():
    from src.bioxp.oem_compat.boards import BioXPBoards

    b = BioXPBoards.dry_run()
    b.deck.activate()
    b.deck.deactivate()
    b.deck.set_led(mask=3, intensity=255)
    b.deck.query_24v_sensor()
    b.deck.set_solenoid_control(1)

    payloads = [f.oem_payload for f in b.transport.frames]
    assert payloads[0] == bytes([64, 0, 0, 0, 0, 0, 1])
    assert payloads[1] == bytes([64, 0, 0, 0, 0, 0, 0])
    assert payloads[2] == bytes([50, 0, 3, 0, 0, 4, 0])  # 255 -> 1024 PWM
    assert payloads[3] == bytes([15, 0, 0, 0, 0, 0, 0])
    assert payloads[4] == bytes([14, 2, 0, 0, 0, 0, 1])


def test_control_interface_uses_oem_extracted_init_and_homing_values():
    from src.bioxp.oem_compat.control_interface import BioXPControlInterface

    ci = BioXPControlInterface.dry_run()
    init_trace = ci.initialize_motors_without_motion()
    init_ops = [(op.name, op.axis, op.value) for op in init_trace.operations]

    assert ("set_speed_acc", "x", {"speed": 1700, "acc": 350}) in init_ops
    assert ("set_speed_acc", "y", {"speed": 1800, "acc": 400}) in init_ops
    assert ("disable_right_switch", "y", None) in init_ops
    assert ("set_speed_acc", "z", {"speed": 1791, "acc": 576}) in init_ops
    assert ("read_max_current", "z", None) in init_ops
    assert ("set_rdiv_pdiv", "g", {"rdiv": 6, "pdiv": 2}) in init_ops
    assert ("disable_right_switch", "door", None) in init_ops
    assert ("disable_left_switch", "door", None) in init_ops
    assert ("set_tc_heat_rate", None, 2.5) in init_ops
    assert ("set_tc_cool_rate", None, -2.0) in init_ops
    assert ("set_deck_color", None, {"r": 255, "g": 255, "b": 255}) in init_ops

    home_trace = ci.startup_homing()
    home_ops = [(op.name, op.axis, op.value) for op in home_trace.operations]
    assert ("home_axis", "y", 250) in home_ops
    assert ("door_search_home", "door", {"speed": 600, "stall_guard": 6}) in home_ops


def test_oem_script_translator_expands_common_verbs_to_control_lib_command_surface():
    from src.bioxp.oem_compat.scripts import OemScriptTranslator, OemScript

    script = OemScript.from_text("""
    <WpfGenBotCommonLib><script>
      <line1 cmd="LED 1 2 3" />
      <line2 cmd="WAIT 5" />
      <line3 cmd="TCD OPEN" />
      <line4 cmd="ET" />
      <line5 step="42" />
      <line6 cmd="UNKNOWN A B" />
    </script></WpfGenBotCommonLib>
    """)
    translated = OemScriptTranslator().translate(script)

    assert [c.command for c in translated.commands] == ["led", "wait", "tcd", "ejt", "step", "unsupported"]
    assert translated.commands[-1].supported is False
    assert translated.commands[-1].raw == "UNKNOWN A B"


def test_pipette_controller_uses_oem_ascii_commands_and_never_reports_executed_in_dry_run():
    from src.bioxp.oem_compat.pipette import PipetteController

    p = PipetteController.dry_run()
    results = [
        p.initiate_group(),
        p.dispense_all(),
        p.eject_tip(initialized=False),
        p.query_tip_status(),
        p.query_pressure(),
        p.enable_pressure_stream(True),
        p.disable_heartbeat_message(),
    ]

    assert [r.command_ascii for r in results] == ["WR", "A0R", "E0R", "?31", "?57", "o0,1R", "U61R"]
    assert all(r.status == "planned" and r.executed is False for r in results)


def test_vision_facade_exposes_oem_method_names_as_explicit_unavailable_dry_run():
    from src.bioxp.oem_compat.vision import VisionFacade

    v = VisionFacade.dry_run()
    assert v.snapshot_image().oem_method == "SaveImage"
    assert v.scan_barcode().oem_method == "ScanBarcode"
    assert v.set_gain(12.5).oem_method == "setGain"
    assert v.inspect_pool_plate().oem_method == "checkPoolPlate"
    assert v.snapshot_image().status == "unavailable"
