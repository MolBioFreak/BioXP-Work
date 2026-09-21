"""OEM color occurrences reach the existing RGB driver; physical USB is doubled."""
from bioxp.usb_driver import BioXpTester
from bioxp.oem_deck_movement import compile_finite_plate_operation, execute_finite_plate_operation
from tests.test_deck_scoped_authority import rig  # noqa: F401


def bind_writer(provider, *, bad_channel=None):
    tester = object.__new__(BioXpTester)
    sent = []

    def send(board, command, typ, channel, value, **kwargs):
        sent.append((board, command, typ, channel, value))
        return {'status': 4 if channel == bad_channel else 100,
                'board': board, 'cmd': command, 'value': 0}

    tester.send_tmcl_retry = send
    provider._oem_pipette_rgb_writer = lambda r, g, b: tester.strip_set_rgb(
        r, g, b, reconnect_first=False, activate_first=False)
    return sent


def run_color(provider, rgb, occurrence):
    plan = compile_finite_plate_operation('pipette_color', source_leaf_available=True,
                                          r=rgb[0], g=rgb[1], b=rgb[2])
    return execute_finite_plate_operation(plan, lambda child: provider.execute_wp8_child(
        child, command_id=occurrence, child_order=child['order'], plan_digest=plan['plan_digest']))


def test_equal_white_occurrences_each_write_three_channels_and_report_delivery(rig):
    provider, _, _, _ = rig
    sent = bind_writer(provider)
    expected = [(5, 50, 0, channel, 1024) for channel in (0, 1, 2)]
    for i in range(2):
        result = run_color(provider, (255, 255, 255), 'white-' + str(i))
        assert result['ok'] is True
        leaf = result['completed_children'][0]['result']
        assert leaf['delivery_attempted'] is True
        assert leaf['sent'] == 3
        assert leaf.get('source_noop') is not True
        assert leaf.get('physical_effect_verified') is not True
        assert all(ack['status'] == 100 for ack in leaf['acks'].values())
        assert sent == expected * (i + 1)


def test_native_color_clamp_is_unchanged(rig):
    provider, _, _, _ = rig
    sent = bind_writer(provider)
    result = run_color(provider, (300, -5, 255), 'clamped')
    assert result['ok'] is True
    assert sent == [(5, 50, 0, 0, 1024), (5, 50, 0, 1, 0), (5, 50, 0, 2, 1024)]


def test_rejected_channel_is_not_success_or_a_cached_future_noop(rig):
    provider, _, _, _ = rig
    sent = bind_writer(provider, bad_channel=1)
    # Inspect the physical leaf's actual failed result without converting it to
    # a successful finite completion; the existing composite raises on failure.
    for i in range(2):
        result = provider.execute_wp8_child(
            {'operation': 'sourceColor', 'arguments': {'r': 255, 'g': 255, 'b': 255}, 'order': 0},
            command_id='failed-white-' + str(i), child_order=0, plan_digest='rgb-plan')
        assert result['ok'] is False
        assert result['delivery_attempted'] is True
        assert result['acks']['g']['status'] == 4
        assert result['sent'] == 3
        assert result.get('source_noop') is not True
    assert len(sent) == 6
