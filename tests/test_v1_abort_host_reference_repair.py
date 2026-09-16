"""Bounded real software Abort/store qualification; no device construction."""
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider, Serial206ProductionPrimitiveAdapter
from bioxp.services.reference_service import ReferenceStateStore
from bioxp.usb_driver import BioXpTester
from tests.test_deck_scoped_authority import qualify_test_references
from tests.test_workflow_lifecycle_canonical_adapter import rig, run
from bioxp.operator_controls import make_workflow_lifecycle_control_executor

AXES = ('x', 'y', 'z', 'g')


@pytest.fixture
def abort_rig(rig):
    tester = BioXpTester.__new__(BioXpTester)
    released = []
    tester.novo_router = SimpleNamespace(set_motor_abort_event=lambda board, motor: released.append((board, motor)))
    tester._oem_24v_dropped = False
    tester._oem_thermal_board_timer_enabled = True
    runtime = OEMRuntimeStore(rig.store.root)
    refs = ReferenceStateStore(rig.store.root / 'bioxp_runtime.db')
    qualify_test_references(refs)
    primitives = Serial206ProductionPrimitiveAdapter(tester, None, authority_provider=lambda: None,
        generation_provider=lambda: 1, reference_store=refs)
    provider = Serial206OemInitializationProvider(primitives, state_store=runtime,
        reference_store=refs, generation_provider=lambda: 1)
    rig.execute = make_workflow_lifecycle_control_executor(rig.store, lambda: provider)
    yield SimpleNamespace(canonical=rig, tester=tester, released=released, runtime=runtime,
        refs=refs, provider=provider, before=refs.snapshot(AXES))
    runtime.close()


def dispatch(r, path):
    if path == 'x':
        return run(r.canonical, 'software_abort')
    return r.provider.execute_z_stop_interrupt(inputs={'command_id': 'z-abort'},
        expected_generation=1, idempotency_key='z-abort', abort=True)


def reopened(r):
    code = '''import json,sys
from pathlib import Path
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.services.reference_service import ReferenceStateStore
root=Path(sys.argv[1]); runtime=OEMRuntimeStore(root)
print(json.dumps({'refs':ReferenceStateStore(root/'bioxp_runtime.db').snapshot(('x','y','z','g')),
                 'runtime':runtime.board4_authority_projection()}))
runtime.close()
'''
    return json.loads(subprocess.check_output([sys.executable, '-c', code, str(r.canonical.store.root)], text=True))


def assert_durable(r, record):
    refs = record['refs']
    assert refs['ok'] is True
    assert set(refs['rows']) == set(AXES)
    for axis, row in refs['rows'].items():
        assert row['state'] == 'desynced'
        assert row['state_version'] > r.before['rows'][axis]['state_version']
    for axis in ('y', 'z', 'gripper'):
        row = record['runtime']['axes'][axis]
        assert row['lifecycle_state'] == 'reconciliation_required'
        assert row['reference_state'] == 'reconciliation_required'
        assert row['prepared_board_epoch'] is None


def export(request, **record):
    path = Path(os.environ['BIOXP_WORKFLOW_EXPORT'] + '.' + request.node.name + '.json')
    path.write_text(json.dumps(record, indent=2, default=str))


@pytest.mark.parametrize('path', ['x', 'z'])
def test_abort_all_references_fresh_process(abort_rig, path, request):
    r = abort_rig
    result = dispatch(r, path)
    # Existing Z interrupt receipt requires physical Stop verification; do not
    # change that out-of-scope classification to qualify host reconciliation.
    assert result['ok'] is (path == 'x')
    assert result['result']['aggregate_authority_invalidation']['ok'] is True, result
    assert r.tester._oem_24v_dropped is True
    assert r.tester._oem_thermal_board_timer_enabled is False
    assert r.released == [(4, 0), (4, 1), (4, 2), (5, 0), (6, 0)]
    record = reopened(r)
    assert_durable(r, record)
    if path == 'x':
        assert dispatch(r, path) == result
        assert len(r.released) == 5
    export(request, before=r.before, result=result, reopened=record, released=r.released)


@pytest.mark.parametrize('path', ['x', 'z'])
@pytest.mark.parametrize('axis', ['y', 'g'])
@pytest.mark.parametrize('fault', ['raise', 'unverified'])
def test_abort_failed_write_persistence_only_retry(abort_rig, monkeypatch, path, axis, fault, request):
    r = abort_rig
    original = r.refs.mark_desynced
    def fail(command):
        if command.axis == axis:
            if fault == 'raise':
                raise OSError('injected reference write failure')
            return {'state': 'desynced', 'persisted': True, 'verified': False}
        return original(command)
    monkeypatch.setattr(r.refs, 'mark_desynced', fail)
    failed = dispatch(r, path)
    assert failed['ok'] is False, failed
    assert failed['result']['aggregate_authority_invalidation']['ok'] is False
    assert failed['result']['aggregate_authority_invalidation']['errors']
    assert r.tester._oem_24v_dropped is True
    assert r.provider._z_interrupt_recovery_required is True
    for member in ('y', 'z', 'gripper'):
        assert r.runtime.axis_interrupt_snapshot(member)['active'] is True
    if path == 'x':
        raw = r.canonical.store.connection.execute('SELECT status FROM operator_commands WHERE command_id=?',
            (failed['command_id'],)).fetchone()
        assert raw[0] != 'completed'
    failed_disk = reopened(r)
    assert failed_disk['refs']['rows'][axis]['state'] == 'referenced'
    calls = list(r.released)
    monkeypatch.setattr(r.refs, 'mark_desynced', original)
    retry = r.provider._reconcile_aggregate_software_abort('persistence-retry', invalidate_x=(path == 'z'))
    assert retry == {'ok': True, 'errors': {}, 'controller_dispatches': 0}
    assert r.released == calls
    assert r.tester._oem_24v_dropped is True
    assert r.provider._z_interrupt_recovery_required is False
    for member in ('y', 'z', 'gripper'):
        assert r.runtime.axis_interrupt_snapshot(member)['active'] is False
    record = reopened(r)
    assert_durable(r, record)
    export(request, failed=failed, failed_disk=failed_disk, retry=retry, reopened=record, released=calls)


@pytest.mark.parametrize('path', ['x', 'z'])
def test_abort_overlap_rejects_late_home_publication(abort_rig, monkeypatch, path, request):
    r = abort_rig
    epochs = {axis: r.runtime.axis_interrupt_snapshot(axis)['epoch'] for axis in ('y', 'z', 'gripper')}
    original = r.refs.mark_desynced
    overlap = []
    def publish_during_invalidation(command):
        if command.axis == 'y' and not overlap:
            # A superseding interrupt starts while the older publication owns persistence.
            epoch = r.runtime.begin_axis_interrupt('y')
            late = r.runtime.publish_axis_reference('y', position_steps=0, ownership_generation=1,
                receipt_id='late-home', expected_software_interrupt_epoch=epochs['y'])
            overlap.append(late)
            assert late == {'ok': False, 'failure': 'axis_publication_interrupted', 'axis': 'y'}
            r.runtime.end_axis_interrupt('y', reconciled=False, expected_epoch=epoch, persistence_owner=True)
        return original(command)
    monkeypatch.setattr(r.refs, 'mark_desynced', publish_during_invalidation)
    result = dispatch(r, path)
    # Existing Z interrupt receipt requires physical Stop verification; do not
    # change that out-of-scope classification to qualify host reconciliation.
    assert result['ok'] is (path == 'x')
    assert result['result']['aggregate_authority_invalidation']['ok'] is True
    assert r.runtime.axis_interrupt_snapshot('y')['active'] is True
    for axis in epochs:
        late = r.runtime.publish_axis_reference(axis, position_steps=0, ownership_generation=1,
            receipt_id='late-home', expected_software_interrupt_epoch=epochs[axis])
        assert late['failure'] == 'axis_publication_interrupted'
    calls = list(r.released)
    assert r.provider._reconcile_aggregate_software_abort('new-owner-retry', invalidate_x=(path == 'z'))['ok']
    assert r.released == calls
    record = reopened(r)
    assert_durable(r, record)
    export(request, result=result, overlap=overlap, reopened=record)
