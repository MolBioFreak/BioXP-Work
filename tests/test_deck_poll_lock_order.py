"""Actual provider/coordinator/SQLite race; deadlocks die with the subprocess."""
import os
import subprocess
import sys
import threading

import pytest

from tests.test_deck_scoped_authority import retained_rig, qualify_test_references


@pytest.mark.parametrize('scenario', ['poll', 'sql_owner_drift'])
def test_delivery_lock_order_and_current_sql_fence(retained_rig, monkeypatch, scenario):
    if os.environ.get('DECK_LOCK_CHILD') != scenario:
        env = dict(os.environ, DECK_LOCK_CHILD=scenario)
        try:
            result = subprocess.run(
                [sys.executable, '-m', 'pytest', '-p', 'no:cacheprovider',
                 '-p', 'tests.z_stop_offline_guard', '-s', '-q',
                 f'{__file__}::test_delivery_lock_order_and_current_sql_fence[{scenario}]'],
                env=env, capture_output=True, text=True, timeout=15,
            )
        except subprocess.TimeoutExpired as exc:
            # subprocess.run has killed and reaped the deadlocked interpreter.
            pytest.fail('bounded child deadlock:\n' + (exc.stderr or b'').decode())
        assert result.returncode == 0, result.stdout + result.stderr
        return

    import faulthandler
    faulthandler.dump_traceback_later(8)
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    from bioxp.oem_deck_movement import make_deck_command_executor, DeckExecutionFailure
    provider, primitive, runtime, references, store, root = retained_rig
    qualify_test_references(references)
    stamps = provider.deck_owner_authority_stamps()
    epochs = {'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
    request = dict(schema_version='bioxp.operator_action_request.v2',
        action_id='oem.deck.move_to_location', expected_ownership_generation=3,
        expected_board_epoch_by_board=epochs, idempotency_key='lock-order',
        inputs={'target': 'LOC_OC', 'camera_offset': False})
    admitted = store.admit_command(request, assessment={'enabled': True}, state={
        'ownership_generation': 3, 'serial206_initialization_provider': {
            'x_authority': {'current_board_lifecycle_generation': epochs['5']},
            'board4_authority': {'active_board_epoch': epochs['4']}}})
    claimed = store.claim_next()
    assert claimed['command_id'] == admitted['command_id']
    original_marker = store.record_delivery_attempt
    original_acquire = provider._lock.acquire
    original_reader = store._deck_owner_authority_reader
    polls = []
    reads = []
    armed = False

    def marker(*args, **kwargs):
        nonlocal armed
        if armed:
            return original_marker(*args, **kwargs)
        armed = True
        if scenario == 'sql_owner_drift':
            def reader():
                reads.append(threading.get_ident())
                if len(reads) == 3:
                    # The INSERT trigger, after both Python checks, must read
                    # the actual current provider rather than captured stamps.
                    monkeypatch.setattr(provider, 'generation_provider', lambda: 4)
                return original_reader()
            monkeypatch.setattr(store, '_deck_owner_authority_reader', reader)
            return original_marker(*args, **kwargs)

        start_poll = threading.Event()
        poll_holds_provider = threading.Event()
        delivery_attempts_provider = threading.Event()
        poll_finished = threading.Event()
        delivery_thread = threading.get_ident()
        acquisitions = 0
        def acquire(*a, **k):
            nonlocal acquisitions
            if threading.get_ident() == delivery_thread:
                acquisitions += 1
                # Fixed owner enters provider before writer. The old owner
                # enters it twice in Python, then a third time inside the UDF.
                # Force the poll overlap at that exact SQL boundary on old code.
                if not start_poll.is_set() and (
                    store._lock._owner_thread_id != delivery_thread or acquisitions == 3
                ):
                    start_poll.set()
                    assert poll_holds_provider.wait(3)
                    delivery_attempts_provider.set()
            return original_acquire(*a, **k)
        monkeypatch.setattr(provider._lock, 'acquire', acquire)
        def poll():
            assert start_poll.wait(3)
            with provider._lock:
                poll_holds_provider.set()
                assert delivery_attempts_provider.wait(3)
                # Same provider -> runtime scope as passive operator-poll GET.
                with provider.projection_scope():
                    runtime.read_oem_serial206_initialization_state()
                    polls.append('projected')
            poll_finished.set()
        thread = threading.Thread(target=poll, name='offline-operator-poll')
        thread.start()
        result = original_marker(*args, **kwargs)
        assert poll_finished.wait(3)
        thread.join(3)
        assert not thread.is_alive()
        monkeypatch.setattr(provider._lock, 'acquire', original_acquire)
        return result

    monkeypatch.setattr(store, 'record_delivery_attempt', marker)
    execute = make_deck_command_executor(provider_getter=lambda: provider,
        position_table_provider=load_bound_oem_position_table, command_store=store)
    arguments = dict(command_id=admitted['command_id'], target='LOC_OC', camera_offset=False,
                     expected_ownership_generation=3, expected_board_epoch_by_board=epochs)
    if scenario == 'sql_owner_drift':
        with pytest.raises(DeckExecutionFailure, match='named_delivery_marker_failed'):
            execute(**arguments)
        assert len(reads) == 3
        assert not store.has_delivery_attempt(admitted['command_id'])
        assert not any(call[0] == 'move' for call in primitive.calls)
    else:
        result = execute(**arguments)
        assert polls == ['projected']
        assert result['ok'] and result['semantic_state_committed']
        store.finish(admitted['command_id'], status='completed', payload=result,
                     claimed=claimed, controller_acknowledged=True, full_response=result)
        assert store.has_delivery_attempt(admitted['command_id'])
        assert store.command_detail_v2(admitted['command_id'])['status'] == 'completed'
    faulthandler.cancel_dump_traceback_later()
