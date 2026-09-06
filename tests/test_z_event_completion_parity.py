from test_f05_completion_consumers import adapter_for
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


def test_event_128_is_authoritative_when_final_counter_differs_from_target(monkeypatch):
    # Exercise the real public caller and consuming waiter, not a fabricated
    # collector-only event (which does not prove an AutoResetEvent wait).
    bind_serial206_oem_snapshot(monkeypatch)
    adapter, driver = adapter_for('valid')
    result = adapter.z_move_absolute(
        requested_position_steps=10_000, pseudo_home_steps=0, wait_timeout_s=.002,
    )
    assert result['ok'] is True
    assert result['failure'] is None
    assert result['controller_command_acknowledged'] is True
    assert result['target_position_steps'] == 10_000
    assert result['after_position_steps'] == 10_006
    assert result['target_events'][0]['status'] == 128
    assert result['source_wait']['ok'] is True
    assert not driver.motor_oem_wait_target_reached(4, 1, timeout_s=0)['ok']
