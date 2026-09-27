"""Offline native handler -> real workflow/queue/finite executor -> SQLite.

Only native motor/door/camera transport and test predecessor are synthetic.
No API requests, client per-action orchestration, or physical timing claim.
"""
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import threading

import pytest
from fastapi import FastAPI, HTTPException

from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_authority import retained_rig


@pytest.fixture
def handoff(connected, monkeypatch):
    from bioxp import api
    from bioxp.operator_command_plane import OperatorCommandPlane
    from bioxp.oem_deck_movement import make_wp8_operation_executor
    from bioxp.oem_compat.position_table import load_bound_oem_position_table

    rig = connected
    store, provider = rig.store, rig.provider
    app = FastAPI()
    monkeypatch.setattr(api, 'app', app)
    # Wire the real dispatch methods to the fixture's existing SQLite owner;
    # no route installation or second store/transport owner is needed.
    plane = object.__new__(OperatorCommandPlane)
    plane.app, plane.store = app, store
    stamps = provider.deck_owner_authority_stamps()
    admission = {'ownership_generation': stamps['ownership_generation'],
        'serial206_initialization_provider': {
            'x_authority': {'current_board_lifecycle_generation': stamps['board_epoch_5']},
            'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}}
    plane.machine_state_provider = lambda: admission
    app.state.operator_command_plane = plane
    app.state.oem_deck_provider = provider
    app.state.oem_deck_position_table_provider = load_bound_oem_position_table
    app.state.oem_wp8_operation_executor = make_wp8_operation_executor(
        provider_getter=lambda: provider, command_store=store)
    store.bind_deck_owner_authority_reader(provider.deck_owner_authority_stamps,
                                          scope=provider.deck_owner_authority_scope)
    admitted = []
    def admit(operation, *, inputs, idempotency_key):
        result = store.admit_internal_wp8_operation(operation, inputs=inputs,
            state=admission, idempotency_key=idempotency_key)
        admitted.append((operation, dict(inputs), result['command_id']))
        return result
    app.state.oem_wp8_operation_admitter = admit
    rig.app, rig.plane, rig.admitted = app, plane, admitted
    yield rig
    store.stop()


@pytest.mark.parametrize('fail', [False, True])
def test_robot_owned_ordered_actions_keep_native_results_and_failure(handoff, monkeypatch, fail):
    from bioxp import api
    from bioxp.services.protocol_service import (
        bind_protocol_dispatcher, create_protocol_job, ProtocolOperatorBundleStore)

    rig = handoff
    store, provider = rig.store, rig.provider
    if fail:
        _, positions = provider._wp8_calibration()
        rig.native.fail_at = ('g', positions['close'])
    payload = {'source_type': 'native', 'idempotency_key': 'handoff-parent',
        'live_execution': {'live_execution_ack': True}, 'document': {
        'protocol_id': 'offline-native-handoff', 'stages': [{'stage_id': 'moves', 'actions': [
            {'action_id': 'output-out', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_OUTPUT', 'target_location': 'LOC_OCS'}},
            {'action_id': 'reagent-out', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_REAGENT', 'target_location': 'LOC_RCS'}},
            {'action_id': 'output-back', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_OUTPUT', 'target_location': 'LOC_OC'}},
        ]}]}}
    done = threading.Event()
    real_finish = store.finish_workflow
    def finish(*args, **kwargs):
        result = real_finish(*args, **kwargs)
        done.set()
        return result
    monkeypatch.setattr(store, 'finish_workflow', finish)
    artifacts = ProtocolOperatorBundleStore(rig.root / 'artifacts')
    handlers = {'move_cover': api._protocol_live_plate_move_handler}
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k: (handlers, {}, {}),
                             artifact_store=artifacts)
    stamps = provider.deck_owner_authority_stamps()
    job = create_protocol_job(payload, dry_run=False, command_store=store, store=artifacts,
        handlers=handlers, ownership_generation=stamps['ownership_generation'], board_epochs={})
    # Observe only, without adding per-action state/diagnostic collections.
    reads = Counter()
    real_get = store.get_command
    def get(cid):
        reads[cid] += 1
        return real_get(cid)
    monkeypatch.setattr(store, 'get_command', get)
    store.start(rig.plane._dispatch_one)
    assert done.wait(15), store.get_workflow(job['job_id'])
    terminal = store.get_workflow(job['job_id'])
    state = terminal['execution']['runtime_state']
    if fail:
        assert terminal['command']['status'] != 'completed'
        assert len(rig.admitted) == 1
        row = real_get(rig.admitted[0][2])
        assert row['status'] == 'ambiguous'
        assert 'injected_native_transfer_failure' in repr(row)
        assert store.deck_semantic_state()['movable_plate_locations']['REAGENT_COVER'] == 'LOC_RC_COVER'
    else:
        assert terminal['command']['status'] == 'completed', terminal
        assert [inputs['destination'] for _, inputs, _ in rig.admitted] == [18, 20, 17]
        for action_result, (_, _, cid) in zip(state['action_results'], rig.admitted):
            assert action_result['ok'] is True
            assert action_result['command'] == real_get(cid)
            assert action_result['command']['status'] == 'completed'
            # Initial read + terminal notification; never full-row polling.
            assert reads[cid] <= 2
        semantic = store.deck_semantic_state()
        assert semantic['movable_plate_locations'] == {
            'OUTPUT_COVER': 'LOC_OC_COVER', 'REAGENT_COVER': 'LOC_RC_COVER_STORAGE'}
        assert semantic['plate_on_gantry'] is None
    rows = store.connection.execute('SELECT parent_command_id FROM operator_commands '
        "WHERE action_id='oem.deck._finite_operation'").fetchall()
    assert rows and all(row[0] == job['job_id'] for row in rows)


def test_wait_uses_notifications_not_full_receipt_polling(handoff, monkeypatch):
    from bioxp import api
    rig, entered, release = handoff, threading.Event(), threading.Event()
    store = rig.store
    cid = rig.app.state.oem_wp8_operation_admitter('script_snapshot', inputs={},
                                                 idempotency_key='completion-wait')['command_id']
    reads = []
    original = store.get_command
    def get(command):
        reads.append(command)
        return original(command)
    monkeypatch.setattr(store, 'get_command', get)
    def dispatch(claimed):
        entered.set()
        assert release.wait(5)
        store.finish(cid, status='completed', payload={'source_return': 0}, claimed=claimed)
    store.start(dispatch)
    try:
        assert entered.wait(5)
        with ThreadPoolExecutor(1) as pool:
            waiter = pool.submit(api._wait_protocol_deck_command, cid, timeout_s=3)
            # Force dispatcher wakeups while the original child is running.
            for _ in range(8):
                store._wake.set()
                threading.Event().wait(.02)
            assert reads == [cid]
            release.set()
            row = waiter.result(4)
        assert row == original(cid)
        assert row['terminal_evidence']['source_return'] == 0
        assert reads == [cid, cid]
    finally:
        release.set()


@pytest.mark.parametrize('status', ['failed', 'ambiguous', 'interrupted', 'stopped', 'aborted', 'cancelled', 'cleared'])
def test_wait_keeps_every_existing_terminal_failure(handoff, status):
    from bioxp import api
    store = handoff.store
    cid = handoff.app.state.oem_wp8_operation_admitter('script_snapshot', inputs={},
        idempotency_key='terminal-' + status)['command_id']
    claimed = store.claim_next()
    future = store.workflow_child_completion(cid)
    store.finish(cid, status=status, payload={'source_return': -7}, claimed=claimed)
    store._settle_workflow_child_waiters()
    assert future.result(timeout=.5)['receipt'] == store.get_command(cid)
    with pytest.raises(HTTPException) as caught:
        api._wait_protocol_deck_command(cid)
    assert caught.value.status_code == 409
    assert caught.value.detail == {'error': 'canonical_deck_command_failed', 'command': store.get_command(cid)}


def test_timeout_does_not_cancel_or_retry_original_child(handoff):
    from bioxp import api
    store = handoff.store
    cid = handoff.app.state.oem_wp8_operation_admitter('script_snapshot', inputs={},
        idempotency_key='timeout-original')['command_id']
    for timeout in (0, .01):
        with pytest.raises(HTTPException) as caught:
            api._wait_protocol_deck_command(cid, timeout_s=timeout)
        assert caught.value.status_code == 504
        assert caught.value.detail == {'error': 'canonical_deck_command_timeout', 'command_id': cid}
    assert store.get_command(cid)['status'] == 'queued'
    assert len(handoff.admitted) == 1
    future = store.workflow_child_completion(cid)
    assert not future.cancelled()
    claimed = store.claim_next()
    store.finish(cid, status='completed', payload={'original': True}, claimed=claimed)
    store._settle_workflow_child_waiters()
    assert future.result(timeout=.5)['status'] == 'completed'
    assert api._wait_protocol_deck_command(cid) == store.get_command(cid)


def test_real_addressed_stop_settles_waiter_with_canonical_interrupt(handoff):
    from bioxp import api
    store = handoff.store
    cid = handoff.app.state.oem_wp8_operation_admitter('move_plate',
        inputs={'plate': 4, 'destination': 18, 'press_plate': False},
        idempotency_key='stop-original')['command_id']
    claimed = store.claim_next()
    assert claimed['command_id'] == cid
    future = store.workflow_child_completion(cid)
    receipt = store.begin_interrupt('oem.x.stop', state=handoff.plane._state(),
                                    request={'idempotency_key': 'stop-handoff'})
    assert cid in receipt['active_command_ids']
    store.mark_interrupt_attempted(idempotency_key='stop-handoff')
    # Explicit offline physical Stop reply; durable fences and disposition real.
    store.finalize_interrupt(idempotency_key='stop-handoff', receipt=receipt,
        attempted=True, acknowledged=True, response={'ok': True})
    store._settle_workflow_child_waiters()
    assert future.result(timeout=.5)['status'] == 'interrupted'
    # A late worker success must not replace the interrupt's SQLite authority.
    store.finish(cid, status='completed', payload={'late_return': True}, claimed=claimed)
    with pytest.raises(HTTPException) as caught:
        api._wait_protocol_deck_command(cid)
    assert caught.value.status_code == 409
    assert caught.value.detail['command']['status'] == 'interrupted'
    assert len(handoff.admitted) == 1


def test_completion_between_first_read_and_subscription_is_not_lost(handoff, monkeypatch):
    from bioxp import api
    store = handoff.store
    cid = handoff.app.state.oem_wp8_operation_admitter('script_snapshot', inputs={},
        idempotency_key='registration-race')['command_id']
    claimed = store.claim_next()
    original = store.get_command
    def first_read(command):
        row = original(command)
        monkeypatch.setattr(store, 'get_command', original)
        store.finish(cid, status='completed', payload={'source_return': -7}, claimed=claimed)
        return row
    monkeypatch.setattr(store, 'get_command', first_read)
    store.start(lambda command: pytest.fail('already claimed command was retried'))
    assert api._wait_protocol_deck_command(cid, timeout_s=1) == original(cid)
