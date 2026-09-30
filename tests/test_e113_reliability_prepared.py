"""Installed prepared plan -> real parent/child/dispatch -> separate Gripper future."""
import json
import threading
from types import SimpleNamespace

import pytest

from tests.test_deck_scoped_authority import retained_rig
from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_integration import installed_retained
from tests.test_e113_reliability_process import emit, eventually


@pytest.fixture
def prepared(connected, retained_rig, monkeypatch):
    generator = installed_retained.__wrapped__(retained_rig, monkeypatch)
    installed = next(generator)
    app, provider, primitive, references, root = installed
    store = app.state.operator_command_plane.store
    monkeypatch.setattr(provider.primitives, 'pipette_transport', None, raising=False)
    # connected installed the real adapter/native source machinery before the
    # control plane captured its provider. Rebind background workers to the
    # current real store, as the production receiving owner does.
    provider.bind_wp8_background_worker_starter(
        lambda worker, name: store.start_wp8_background_worker(worker, name))
    yield app, connected, store
    next(generator, None)


@pytest.mark.parametrize('outcome', ['completed', 'failure', 'stop'])
def test_prepared_pending_handoff_has_separate_terminal_gripper_future(prepared, monkeypatch, outcome):
    from bioxp.oem_deck_movement import compile_finite_plate_operation
    from bioxp.protocols.runtime_state import ProtocolRuntimeState, ProtocolWorkflowState
    app, r, store = prepared
    provider = r.provider
    entered, release, handed, parent_done = [threading.Event() for _ in range(4)]
    real_home = r.native.motor_oem_home_axis
    def home(axis, **kwargs):
        assert axis == 'g'
        entered.set()
        assert release.wait(12)
        if outcome == 'failure':
            raise RuntimeError('controlled_gripper_completion_failure')
        return real_home(axis, **kwargs)
    monkeypatch.setattr(r.native, 'motor_oem_home_axis', home)
    real_execute = app.state.oem_wp8_operation_executor
    def locked_execute(*, command_id, plan):
        # Standalone SendZAndGripperHome source caller owns the same Gripper
        # lock that catch/release normally acquired before entering this leaf.
        claimed = store.connection.execute('SELECT dispatch_attempt_id FROM operator_plane_commands WHERE command_id=?', (command_id,)).fetchone()
        provider.wp8_lock_gripper('LockGripperOperation', {}, command_id=command_id,
            child_order=0, plan_digest=plan['plan_digest'], dispatch_attempt_id=claimed[0],
            **provider.deck_owner_authority_stamps())
        return real_execute(command_id=command_id, plan=plan)
    monkeypatch.setattr(app.state, 'oem_wp8_operation_executor', locked_execute)
    plan = compile_finite_plate_operation('send_z_and_gripper_home',
        source_leaf_available=True, run_in_parallel=True, gripper_position=0, closed_position=0)
    runtime = ProtocolRuntimeState(protocol_id='prepared', job_id='prepared-parent', dry_run=False)
    runtime.workflow = ProtocolWorkflowState(command_id='prepared-parent')
    results, errors = [], []
    def parent(command):
        try:
            with store.workflow_context('prepared-parent', source_occurrence_id='prepared-home'):
                result = app.state.oem_workflow_plan_executor(plan, SimpleNamespace(), runtime)
            results.append(result)
            handed.set()
            result['owned_children'][0].future.result(timeout=12)
        except BaseException as exc:
            errors.append(exc)
        finally:
            parent_done.set()
    store.bind_workflow_dispatcher(parent)
    store.admit_workflow(command_id='prepared-parent', idempotency_key='prepared-parent',
        plan_fingerprint='real-prepared', requested_inputs={'bundle': {'execution': {'runtime_state': {}}}},
        ownership_generation=provider.deck_owner_authority_stamps()['ownership_generation'],
        resources=['axis:x', 'axis:y', 'axis:z', 'gripper'], board_epochs={})
    app.state.operator_command_plane.start()
    try:
        assert entered.wait(8), repr((errors, store.queue(), runtime.workflow.child_command_ids, [store.get_command(cid) for cid in runtime.workflow.child_command_ids]))
        assert handed.wait(8), repr(errors)
        result = results[0]
        cid = result['command_id']
        assert result['status'] == 'issued_pending'
        assert runtime.workflow.child_command_ids == [cid]
        owned = result['owned_children'][0]
        assert owned.domains == ('Gripper',)
        assert not owned.future.done()
        pending = store.workflow_child_completion(cid, include_issued_pending=True)
        assert pending is not owned.future
        assert pending.result(timeout=1)['status'] == 'issued_pending'
        assert store.workflow_child_completion(cid) is owned.future
        db = store.connection
        row = dict(db.execute('SELECT * FROM operator_plane_commands WHERE command_id=?', (cid,)).fetchone())
        assert row['status'] == 'issued_pending' and row['dispatch_attempt_id']
        assert db.execute('SELECT parent_command_id FROM operator_commands WHERE command_id=?', (cid,)).fetchone()[0] == 'prepared-parent'
        tasks_before = [dict(x) for x in db.execute('SELECT * FROM operator_plane_wp8_background_tasks WHERE command_id=?', (cid,))]
        assert any(task['state'] == 'running' for task in tasks_before)
        assert store.has_delivery_attempt(cid)
        if outcome == 'stop':
            interrupt = store.begin_interrupt('oem.g.stop', state=app.state.operator_command_plane._state(),
                request={'idempotency_key': 'prepared-stop'})
            store.mark_interrupt_attempted(idempotency_key='prepared-stop')
            store.finalize_interrupt(idempotency_key='prepared-stop', receipt=interrupt,
                attempted=True, acknowledged=True, response={'ok': True})
            interrupted_receipt = store.get_command(cid)
            running_task = next(task for task in tasks_before if task['state'] == 'running')
            original_owner = store.owner_id
            store.owner_id = 'not-the-real-owner'
            try:
                with pytest.raises(RuntimeError, match='terminal authority is stale'):
                    store.settle_wp8_background_task(running_task['task_id'], state='completed',
                        evidence={'forged_late_completion': True})
            finally:
                store.owner_id = original_owner
            assert store.get_command(cid) == interrupted_receipt
            assert db.execute('SELECT state FROM operator_plane_wp8_background_tasks WHERE task_id=?',
                (running_task['task_id'],)).fetchone()[0] == 'running'
        release.set()
        terminal = owned.future.result(timeout=8)
        assert parent_done.wait(3)
        assert not errors, repr(errors)
        assert terminal['status'] == ('completed' if outcome == 'completed' else ('ambiguous' if outcome == 'failure' else 'interrupted'))
        for task in provider._wp8_tasks.values():
            task['thread'].join(3)
            assert not task['thread'].is_alive()
        tasks_after = [dict(x) for x in db.execute('SELECT * FROM operator_plane_wp8_background_tasks WHERE command_id=?', (cid,))]
        if outcome == 'stop':
            assert store.get_command(cid) == interrupted_receipt
            assert all(task['state'] in {'completed', 'interrupted'} for task in tasks_after)
            late = next(task for task in tasks_after if task['state'] == 'interrupted')
            assert json.loads(late['evidence_json'])['late_source_state'] == 'completed'
        # A separate process reopens the actual SQLite store while the parent
        # still owns its lease; reopening must not recover or change this row.
        import os, subprocess, sys
        script = ('import z_stop_offline_guard,json,sys,os; os.environ["BIOXP_OEM_RUNTIME_STATE_ROOT"]=sys.argv[1]; from bioxp.operator_command_plane import OperatorCommandStore; '
                  's=OperatorCommandStore(sys.argv[1]); print(json.dumps(s.get_command(sys.argv[2]))); s.connection.close()')
        reopened = subprocess.run([sys.executable, '-c', script, str(store.root), cid],
            capture_output=True, text=True, check=True)
        assert json.loads(reopened.stdout) == store.get_command(cid)
        emit('prepared-' + outcome, {'command_id': cid, 'pending_row': row,
            'tasks_before': tasks_before, 'tasks_after': tasks_after, 'terminal_future': terminal,
            'receipt': store.get_command(cid), 'evidence': store.wp8_operation_evidence(cid),
            'native_events': r.native.events})
    finally:
        release.set()
        parent_done.wait(5)
