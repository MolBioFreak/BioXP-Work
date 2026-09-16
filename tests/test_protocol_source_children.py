"""Nested OEM source tasks remain owned by the existing executor pool."""
from contextvars import ContextVar
from threading import Event
import pytest
from tests.test_protocol_oem_lifecycle import document, engine, start, finish, wait_until


@pytest.mark.parametrize('child_ok', [True, False])
def test_nested_source_future_keeps_context_and_custody_after_parent_return(child_ok):
    doc = document('led')
    entered, release, parent_returned = Event(), Event(), Event()
    lineage = ContextVar('fixture_lineage', default=None)
    witnessed = []
    def composite(action, state):
        token = lineage.set('original-parent')
        def child():
            witnessed.append(lineage.get())
            entered.set()
            assert release.wait(4)
            return {'ok': child_ok, 'native_witness': 'retained-child'}
        try:
            executor.start_child('source:0:nested', child, domains=('Tip',))
            parent_returned.set()
            return {'ok': True, 'native_witness': 'parent-return'}
        finally:
            lineage.reset(token)
    executor, trace = engine(doc, handlers={'led': composite})
    run = start(executor, doc)
    try:
        assert entered.wait(3) and parent_returned.wait(3)
        assert run[0].is_alive() and not executor._state.completed
        assert 'epilogue_park' not in trace
    finally:
        release.set()
    state = finish(run)
    assert witnessed == ['original-parent']
    assert state.completed is child_ok
    assert executor.outcome == ('completed' if child_ok else 'failed')
    nested = [r for r in state.action_results if r.get('action_id') == 'source:0:nested']
    assert len(nested) == 1 and nested[0]['native_witness'] == 'retained-child'
    assert next(r for r in state.action_results if r.get('native_witness') == 'parent-return')['ok'] is True


def test_nested_source_exception_is_retained_not_lost_in_parent_return():
    doc = document('led')
    def composite(action, state):
        def child():
            error = RuntimeError('native fixture error')
            error.oem_partial_results = [{'ok': True, 'source_leaf': 'entered'}]
            raise error
        executor.start_child('source:0:failing', child, domains=('Tip',))
        return {'ok': True}
    executor, trace = engine(doc, handlers={'led': composite})
    state = executor.execute(doc)
    assert executor.outcome == 'failed'
    row = next(r for r in state.action_results if r.get('action_id') == 'source:0:failing')
    assert row['oem_partial_results'] == [{'ok': True, 'source_leaf': 'entered'}]


def test_stop_does_not_cancel_an_entered_nested_task_or_allow_a_new_one():
    doc = document('led')
    entered, release = Event(), Event()
    futures = []
    def composite(action, state):
        def child():
            entered.set()
            assert release.wait(4)
            return {'ok': True, 'native_return': 'after-stop'}
        futures.append(executor.start_child('source:0:entered', child, domains=('Tip',)))
        return {'ok': True}
    executor, trace = engine(doc, handlers={'led': composite})
    run = start(executor, doc)
    try:
        assert entered.wait(3)
        executor.interrupt(control_id='addressed-stop')
        with pytest.raises(Exception):
            executor.start_child('source:0:late', lambda: pytest.fail('late entry'), domains=('Tip',))
        assert run[0].is_alive() and not futures[0].cancelled()
    finally:
        release.set()
    state = finish(run)
    assert executor.outcome == 'interrupted'
    assert any(r.get('native_return') == 'after-stop' for r in state.action_results)
    assert not any(x in trace for x in ('cleanup', 'epilogue_park'))
