"""Software abort reconciliation is non-motion and distinct from controller Stop."""
import threading

from tests.test_v1_abort_host_reference_repair import abort_rig
from tests.test_workflow_lifecycle_canonical_adapter import rig


def test_deferred_abort_reconciles_before_nonmotion_preparation(abort_rig, monkeypatch):
    r = abort_rig
    outcome = {}
    r.provider._lock.acquire()
    try:
        worker = threading.Thread(target=lambda: outcome.setdefault('receipt',
            r.provider.execute_x_stop_interrupt({'command_id': 'abort-under-lock'}, abort=True)))
        worker.start()
        worker.join(8)
        assert not worker.is_alive()
    finally:
        r.provider._lock.release()
    receipt = outcome['receipt']
    invalidation = receipt['result']['aggregate_authority_invalidation']
    assert receipt['ok'] is False
    assert invalidation['errors']['z_lifecycle'] == 'reconciliation_lock_unavailable_bounded'
    assert receipt['source_call_completed'] is True
    assert receipt['controller_command_acknowledged'] is False
    assert receipt['controller_terminal_state_verified'] is False
    assert r.provider._z_interrupt_active is True
    assert r.provider._x_interrupt_recovery_required is True
    assert all(r.runtime.board4_authority_projection()['axes'][a]['lifecycle_state'] ==
               'reconciliation_required' for a in ('y', 'z', 'gripper'))
    assert r.provider._xy_authority_snapshot(r.provider._load_state()['x_lifecycle'])['ok'] is False
    released = list(r.released)
    # The next preparation is admitted only after persistence-only reconciliation.
    # Stub the actual controller preparation: this test never sends a hardware command.
    monkeypatch.setattr('bioxp.oem_serial206_initialization.prepare_motion_without_motion',
                        lambda *args, **kwargs: {'ok': False, 'failure': 'simulated_hardware'})
    # Prior to persistence-only repair this same entrypoint hits the Z gate.
    assert r.provider._z_interrupt_active is True
    result = r.provider.prepare_global_motion_without_motion(r.tester, authority=None)
    assert result['failure'] == 'simulated_hardware'
    assert r.provider._z_interrupt_active is False
    assert r.provider._load_state()['x_lifecycle']['state'] == 'failed_latched'
    assert r.provider._pending_software_abort_reconciliation is None
    assert r.released == released
    assert all(r.refs.snapshot(('x', 'y', 'z', 'g'))['rows'][a]['state'] == 'desynced'
               for a in ('x', 'y', 'z', 'g'))
    # Authority remains unprepared until a real current-generation preparation;
    # logical abort alone never clears Z, establishes XY home, or proves a Stop.
    assert r.provider._xy_authority_snapshot(r.provider._load_state()['x_lifecycle'])['ok'] is False


def test_failed_reconciliation_remains_fenced(abort_rig, monkeypatch):
    r = abort_rig
    original = r.refs.mark_desynced
    monkeypatch.setattr(r.refs, 'mark_desynced', lambda cmd: (
        (_ for _ in ()).throw(OSError('disk unavailable')) if cmd.axis == 'g' else original(cmd)))
    receipt = r.provider.execute_x_stop_interrupt({'command_id': 'abort-write-fails'}, abort=True)
    assert receipt['result']['aggregate_authority_invalidation']['ok'] is False
    released = list(r.released)
    result = r.provider.prepare_global_motion_without_motion(r.tester, authority=None)
    assert result['failure'] == 'software_abort_reconciliation_pending'
    assert result['physical_motion_commanded'] is False
    assert r.provider._z_interrupt_active is True
    assert r.released == released
