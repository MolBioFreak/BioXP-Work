"""Startup session binding: real provider/driver/reference stores, inert wire."""
import pytest

from tests.test_wake_setup_debloat import rig  # noqa: F401
from bioxp.services.reference_service import ReferenceStateStore


@pytest.mark.parametrize('all_initialized', [True, False])
def test_initialize_motion_current_session(rig, monkeypatch, all_initialized):
    driver, provider, frames, fault, root = rig
    # Accepted activation without a preparation or begin-generation call.
    if not all_initialized:
        fault['reject'] = (driver.BOARD_CHILLER, 64, 0, 0)
    driver.activate_boards()
    assert all(driver._oem_board_state().values()) is all_initialized
    assert driver.oem_current_board_lifecycle_generation() is None
    frames.clear()
    monkeypatch.setattr(provider.primitives, 'query_all_pipette_tip_states',
                        lambda **kw: {'ok': True, 'tip_exists': False})
    result = provider.initialize_motion(mode='live')
    assert result['ok'] is True and result['state'] == 'initializeMotion_complete'
    assert result['ready'] is all_initialized
    assert not any(command == 64 for _, command, *_ in frames)
    homes = [(board, motor) for board, command, _, motor, _ in frames if command == 2]
    assert homes[0] == (4, 1)  # Z before gripper or XY
    states = ReferenceStateStore(root / 'bioxp_runtime.db').snapshot(('x','y','z','g'))['rows']
    assert all(row['state'] == 'referenced' for row in states.values()) is all_initialized
    if all_initialized:
        assert type(driver.oem_current_board_lifecycle_generation()) is int
        assert result['reference_blockers'] == []
    else:
        assert driver.oem_current_board_lifecycle_generation() is None
        assert all(row['state'] != 'referenced' for row in states.values())
        assert {reason.split(':')[0] for reason in result['reference_blockers']} == set('xyzg')
        assert all('current_board_initialization_incomplete_or_transport_changed' in reason
                   for reason in result['reference_blockers'])
        assert result['blockers'] == result['reference_blockers']


def test_outer_preserves_inner_storage_failure(rig, monkeypatch):
    driver, provider, frames, fault, root = rig
    driver.activate_boards()
    frames.clear()
    monkeypatch.setattr(provider.reference_store, 'mark_desynced_many',
                        lambda commands: {'ok': False, 'durable_clean': False})
    result = provider.initialize_motion(mode='live')
    assert not result['ok'] and not result['ready']
    assert result['blockers'] == ['aggregate_reference_invalidation_failed']
    assert frames == []
