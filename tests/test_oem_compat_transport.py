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


def test_oem_reply_frame_parses_robot_driver_tmcl_response_shape():
    from src.bioxp.oem_compat.frames import OemReplyFrame, TrafficCategory

    raw = bytes([0, 0, 0, 0, 0, 0, 5, 100, 6, 0, 0, 6, 164, 0])

    reply = OemReplyFrame.from_tmcl_response(raw)

    assert reply.category is TrafficCategory.MOTION
    assert reply.board_id == 5
    assert reply.status == 100
    assert reply.status_str == "Success"
    assert reply.command == 6
    assert reply.value == 1700
    assert reply.synthetic is False
    assert reply.payload == raw


def test_demux_tmcl_responses_separates_matching_reply_from_async_events():
    from src.bioxp.oem_compat.frames import demux_tmcl_responses

    async_event = bytes([0, 0, 0, 0, 0, 0, 4, 132, 9, 0, 0, 0, 1, 0])
    matching_reply = bytes([0, 0, 0, 0, 0, 0, 5, 100, 6, 0, 0, 0, 31, 0])

    reply, events = demux_tmcl_responses([async_event, matching_reply], expected_board_id=5, expected_command=6)

    assert reply is not None
    assert reply.board_id == 5
    assert reply.command == 6
    assert reply.value == 31
    assert [event.board_id for event in events] == [4]
    assert events[0].status_str == "Door/Latch sensor changed"


def test_strict_dry_run_configured_reply_must_match_command_frame():
    from src.bioxp.oem_compat.frames import OemCommandFrame, OemReplyFrame, TrafficCategory
    from src.bioxp.oem_compat.transport import DryRunTransport, ReplyMismatch

    frame = OemCommandFrame.tmcl(5, 6, 4, 0, 0)
    mismatched_reply = OemReplyFrame(category=TrafficCategory.MOTION, status=100, board_id=4, command=6, value=1700)
    transport = DryRunTransport(require_configured_replies=True, configured_replies={frame: mismatched_reply})

    with pytest.raises(ReplyMismatch):
        transport.transmit(frame)


def test_transport_safety_contract_blocks_dry_run_usb_and_shadow_motion():
    from src.bioxp.oem_compat.transport import SafetyContractViolation, assert_transport_safety

    assert_transport_safety(mode="dry_run", opened_usb=False, physical_motion=False)
    assert_transport_safety(mode="shadow", opened_usb=True, physical_motion=False)

    with pytest.raises(SafetyContractViolation):
        assert_transport_safety(mode="dry_run", opened_usb=True, physical_motion=False)
    with pytest.raises(SafetyContractViolation):
        assert_transport_safety(mode="shadow", opened_usb=True, physical_motion=True)


def test_strict_tmcl_reply_matching_rejects_ambiguous_duplicate_replies():
    from src.bioxp.oem_compat.frames import OemCommandFrame
    from src.bioxp.oem_compat.transport import AmbiguousReply, match_tmcl_reply

    frame = OemCommandFrame.tmcl(5, 6, 4, 0, 0)
    first = bytes([0, 0, 0, 0, 0, 0, 5, 100, 6, 0, 0, 0, 31, 0])
    duplicate = bytes([0, 0, 0, 0, 0, 0, 5, 100, 6, 0, 0, 0, 32, 0])

    with pytest.raises(AmbiguousReply):
        match_tmcl_reply([first, duplicate], expected=frame)


def test_strict_tmcl_reply_matching_rejects_non_success_status():
    from src.bioxp.oem_compat.frames import OemCommandFrame
    from src.bioxp.oem_compat.transport import ReplyMismatch, match_tmcl_reply

    frame = OemCommandFrame.tmcl(5, 5, 4, 0, 37)
    invalid_value = bytes([0, 0, 0, 0, 0, 0, 5, 4, 5, 0, 0, 0, 37, 0])

    with pytest.raises(ReplyMismatch, match="Invalid value"):
        match_tmcl_reply([invalid_value], expected=frame)


def test_shadow_transport_allows_query_frames_and_blocks_mutating_frames():
    from src.bioxp.oem_compat.frames import OemCommandFrame
    from src.bioxp.oem_compat.transport import MutatingCommandBlocked, ShadowTransport

    class FakeStatusAdapter:
        def __init__(self):
            self.frames = []

        def transmit(self, frame):
            self.frames.append(frame)
            return [bytes([0, 0, 0, 0, 0, 0, frame.sidh, 100, frame.command, 0, 0, 0, 12, 0])]

    adapter = FakeStatusAdapter()
    transport = ShadowTransport(adapter=adapter)
    query = OemCommandFrame.tmcl(5, 6, 1, 0, 0)
    reply = transport.transmit(query)

    assert transport.opened_usb is True
    assert reply.value == 12
    assert adapter.frames == [query]

    mutating = OemCommandFrame.tmcl(5, 5, 4, 0, 1700)
    with pytest.raises(MutatingCommandBlocked):
        transport.transmit(mutating)
    assert adapter.frames == [query]


def test_live_transport_requires_operator_ack_and_artifact_root_before_adapter_use(tmp_path):
    from src.bioxp.oem_compat.frames import OemCommandFrame
    from src.bioxp.oem_compat.transport import LiveTransport, LiveTransportNotArmed

    class Adapter:
        def __init__(self):
            self.called = False

        def transmit(self, frame):
            self.called = True
            return [bytes([0, 0, 0, 0, 0, 0, frame.sidh, 100, frame.command, 0, 0, 0, 0, 0])]

    adapter = Adapter()
    frame = OemCommandFrame.tmcl(5, 5, 4, 0, 1700)

    with pytest.raises(LiveTransportNotArmed, match="operator"):
        LiveTransport(adapter=adapter, operator_ack=False, artifact_root=tmp_path).transmit(frame)
    with pytest.raises(LiveTransportNotArmed, match="artifact"):
        LiveTransport(adapter=adapter, operator_ack=True, artifact_root=None).transmit(frame)
    assert adapter.called is False


def test_live_transport_fails_closed_on_ambiguous_reply(tmp_path):
    from src.bioxp.oem_compat.frames import OemCommandFrame
    from src.bioxp.oem_compat.transport import AmbiguousReply, LiveTransport

    class AmbiguousAdapter:
        def transmit(self, frame):
            return [
                bytes([0, 0, 0, 0, 0, 0, frame.sidh, 100, frame.command, 0, 0, 0, 1, 0]),
                bytes([0, 0, 0, 0, 0, 0, frame.sidh, 100, frame.command, 0, 0, 0, 2, 0]),
            ]

    frame = OemCommandFrame.tmcl(5, 5, 4, 0, 1700)
    transport = LiveTransport(adapter=AmbiguousAdapter(), operator_ack=True, artifact_root=tmp_path)

    with pytest.raises(AmbiguousReply):
        transport.transmit(frame)
