"""Real canonical delivery/completion with stale history; native seams offline."""
import json
import sqlite3

import pytest

from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_authority import retained_rig


def history(store, value):
    with store._transaction() as db:
        db.execute('UPDATE operator_plane_deck_semantic_state SET ownership_generation=?,board_epoch_4=?,board_epoch_5=?,semantic_state_revision=semantic_state_revision+1',
                   (value, value, value))


def state(provider):
    stamps = provider.deck_owner_authority_stamps()
    return {'ownership_generation': stamps['ownership_generation'],
        'serial206_initialization_provider': {
            'x_authority': {'current_board_lifecycle_generation': stamps['board_epoch_5']},
            'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}}


def scripted(r):
    from bioxp.oem_deck_movement import ClassMoveToIntent, compile_mov_execution, bind_mov_execution_script_plan
    intent = ClassMoveToIntent(1, plate_name=5, well='A1')
    admitted = r.store.admit_internal_mov_execution(intent, state=state(r.provider), idempotency_key='v14-script')
    claimed = r.store.claim_next()
    assert claimed['command_id'] == admitted['command_id']
    plan = compile_mov_execution(intent, r.provider.mov_execution_machine_state())
    plan = bind_mov_execution_script_plan(plan, r.provider.preview_scriptmove_to(plan.steps[0].arguments)['plan'])
    return claimed, plan


@pytest.mark.parametrize('stamp', [None, 999])
def test_script_native_dispatch_and_completion_ignore_history(connected, stamp):
    from bioxp.oem_deck_movement import execute_mov_execution
    r = connected
    claimed, plan = scripted(r)
    history(r.store, stamp)
    result = execute_mov_execution(claimed['command_id'], plan, provider=r.provider, command_store=r.store)
    assert result['ok'] is True
    assert r.native.moves
    rows = r.store.connection.execute('SELECT * FROM operator_plane_delivery_attempts WHERE command_id=?', (claimed['command_id'],)).fetchall()
    assert rows and all(row['work_kind'] == 'wp7_stage' for row in rows)
    assert all(row['ownership_generation'] == r.provider.deck_owner_authority_stamps()['ownership_generation'] for row in rows)
    stages = r.store.connection.execute('SELECT terminal_state FROM operator_plane_deck_stages WHERE command_id=?', (claimed['command_id'],)).fetchall()
    assert stages and all(row[0] == 'completed' for row in stages)
    r.store.finish(claimed['command_id'], status='completed', payload=result, claimed=claimed)


@pytest.mark.parametrize('stamp', [None, 999])
@pytest.mark.parametrize('parallel', [False, True])
@pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning")
def test_finite_native_and_background_completion_ignore_history(connected, stamp, parallel):
    from bioxp.oem_deck_movement import compile_finite_plate_operation, make_wp8_operation_executor
    r = connected
    operation = 'send_z_and_gripper_home'
    inputs = {'run_in_parallel': parallel, 'gripper_position': 0, 'closed_position': 0}
    plan = compile_finite_plate_operation(operation, source_leaf_available=True, **inputs)
    store = r.store
    parent = 'v14-parent'
    store.bind_workflow_dispatcher(lambda command: None)
    store.admit_workflow(command_id=parent, idempotency_key=parent,
        plan_fingerprint='standalone-v14-home',
        requested_inputs={'bundle': {'execution': {'runtime_state': {}}}},
        ownership_generation=r.provider.deck_owner_authority_stamps()['ownership_generation'],
        resources=('axis:x', 'axis:y', 'axis:z', 'gripper'), board_epochs={})
    assert store.claim_next()['command_id'] == parent
    with store.workflow_context(parent, source_occurrence_id='v14-home'):
        admitted = store.admit_internal_wp8_operation(operation, inputs={'run_in_parallel': parallel}, state=state(r.provider),
            idempotency_key='v14-finite', prepared_plan=plan)
    claimed = r.store.claim_next()
    assert claimed['command_id'] == admitted['command_id']
    history(r.store, stamp)
    if parallel:
        # Standalone source caller holds the same gripper lock as catch/release.
        r.provider.wp8_lock_gripper('LockGripperOperation', {}, command_id=claimed['command_id'],
            child_order=0, plan_digest=plan['plan_digest'],
            dispatch_attempt_id=claimed['dispatch_attempt_id'], **r.provider.deck_owner_authority_stamps())
    result = make_wp8_operation_executor(provider_getter=lambda: r.provider, command_store=r.store)(
        command_id=claimed['command_id'], plan=plan)
    for task in r.provider._wp8_tasks.values():
        task['thread'].join(3)
        assert not task['thread'].is_alive()
        assert task['state'] == 'completed', task
    assert result['ok'] is True, result
    assert r.native.events
    children = r.store.wp8_operation_evidence(claimed['command_id'])['children']
    assert children and all(row['terminal_state'] == 'completed' for row in children)
    tasks = r.store.connection.execute('SELECT state FROM operator_plane_wp8_background_tasks WHERE command_id=?', (claimed['command_id'],)).fetchall()
    assert len(tasks) == (2 if parallel else 0)
    assert all(row[0] == 'completed' for row in tasks)
    r.store.finish(claimed['command_id'], status='completed', payload=result, claimed=claimed)


@pytest.mark.parametrize('fault', ['owner', 'board', 'attempt', 'lease'])
def test_script_completion_still_requires_current_owner_and_attempt(connected, monkeypatch, fault):
    r = connected
    claimed, plan = scripted(r)
    cid = claimed['command_id']
    r.store.persist_mov_execution_plan(cid, plan)
    history(r.store, None)
    step = plan.steps[0]
    marker = r.store.record_delivery_attempt(cid, work_kind='wp7_stage',
        work_identity=f'stage:{step.order}:{step.operation}', plan_digest=plan.plan_digest)
    if fault in {'owner', 'board'}:
        stamps = r.provider.deck_owner_authority_stamps()
        stamps['ownership_generation' if fault == 'owner' else 'board_epoch_5'] += 1
        monkeypatch.setattr(r.store, '_deck_owner_authority_reader', lambda: stamps)
    elif fault == 'lease':
        with r.store._transaction() as db:
            db.execute('UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1')
    with pytest.raises(RuntimeError, match='completion authority is stale'):
        r.store.terminalize_mov_execution_stage(cid, step, state='completed', result={},
            dispatch_attempt_id='wrong' if fault == 'attempt' else marker['dispatch_attempt_id'])
    assert r.store.connection.execute('SELECT terminal_state FROM operator_plane_deck_stages WHERE command_id=? AND stage_order=?',
        (cid, step.order)).fetchone()[0] == 'planned'
    assert not r.native.moves


@pytest.mark.parametrize('fault', ['owner', 'board', 'stop', 'plan', 'work', 'command', 'attempt'])
def test_real_dispatch_refusals_keep_native_undelivered(connected, monkeypatch, fault):
    from bioxp.oem_deck_movement import execute_mov_execution
    r = connected
    claimed, plan = scripted(r)
    history(r.store, None)
    cid = claimed['command_id']
    if fault in {'owner', 'board'}:
        stamps = r.provider.deck_owner_authority_stamps()
        stamps['ownership_generation' if fault == 'owner' else 'board_epoch_4'] += 1
        monkeypatch.setattr(r.store, '_deck_owner_authority_reader', lambda: stamps)
    elif fault == 'stop':
        r.store.arm_interrupt_fence('oem.x.stop')
    if fault in {'owner', 'board', 'stop'}:
        with pytest.raises(RuntimeError):
            execute_mov_execution(cid, plan, provider=r.provider, command_store=r.store)
    else:
        r.store.persist_mov_execution_plan(cid, plan)
        step = plan.steps[0]
        if fault == 'attempt':
            # Bypass Python deliberately: the actual trigger must reject a forged attempt.
            stamps = r.provider.deck_owner_authority_stamps()
            with pytest.raises(sqlite3.IntegrityError, match='lineage'):
                with r.store._transaction() as db:
                    db.execute('INSERT INTO operator_plane_delivery_attempts(command_id,work_kind,work_identity,dispatch_attempt_id,plan_digest,owner_id,ownership_generation,board_epoch_4,board_epoch_5,created_at) VALUES(?,?,?,?,?,?,?,?,?,?)',
                        (cid, 'wp7_stage', f'stage:{step.order}:{step.operation}', 'wrong-attempt', plan.plan_digest, r.store.owner_id,
                         stamps['ownership_generation'], stamps['board_epoch_4'], stamps['board_epoch_5'], 1.0))
        else:
            with pytest.raises((RuntimeError, sqlite3.IntegrityError)):
                r.store.record_delivery_attempt('wrong-command' if fault == 'command' else cid,
                    work_kind='wp7_stage', work_identity='wrong-work' if fault == 'work' else f'stage:{step.order}:{step.operation}',
                    plan_digest='0'*64 if fault == 'plan' else plan.plan_digest)
    assert not r.native.moves
    assert not r.store.has_delivery_attempt(cid)
