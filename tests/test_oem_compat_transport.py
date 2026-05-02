import pytest


def test_dry_run_transport_records_oem_transmit_message_without_opening_usb():
    from src.bioxp.oem_compat.frames import OemCommandFrame, TrafficCategory
    from src.bioxp.oem_compat.transport import DryRunTransport

    transport = DryRunTransport()
    frame = OemCommandFrame(
        sidh=0x05,
        sidl=0x00,
        cmd=bytes([0x05, 0x04, 0x00, 0x00, 0x06, 0xA4]),
        message="setMaxSpeed X 1700",
        timeout_ms=60000,
    )

    reply = transport.transmit(frame)

    assert transport.opened_usb is False
    assert transport.frames == [frame]
    assert reply.category is TrafficCategory.MOTION
    assert reply.synthetic is True
    assert reply.status == 100


def test_oem_command_frame_classifies_novo_motion_can_ids_4_through_9():
    from src.bioxp.oem_compat.frames import OemCommandFrame, TrafficCategory

    assert OemCommandFrame(sidh=4, sidl=0, cmd=b"").category is TrafficCategory.MOTION
    assert OemCommandFrame(sidh=9, sidl=0, cmd=b"").category is TrafficCategory.MOTION
    assert OemCommandFrame(sidh=0, sidl=0, cmd=b"").category is TrafficCategory.CANOPEN_UIM
    assert OemCommandFrame(sidh=513, sidl=0, cmd=b"").category is TrafficCategory.CANOPEN_UIM
    assert OemCommandFrame(sidh=42, sidl=0, cmd=b"").category is TrafficCategory.PIPETTE


def test_dry_run_transport_can_require_configured_replies_for_strict_oracle_tests():
    from src.bioxp.oem_compat.frames import OemCommandFrame
    from src.bioxp.oem_compat.transport import DryRunTransport, MissingDryRunReply

    transport = DryRunTransport(require_configured_replies=True)

    with pytest.raises(MissingDryRunReply):
        transport.transmit(OemCommandFrame(sidh=5, sidl=0, cmd=b"\x01", message="unconfigured"))
