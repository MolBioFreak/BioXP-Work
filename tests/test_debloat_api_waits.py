"""A2/C5 boundary tests; no controller or network calls."""
import ast
import contextlib
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import threading
import pytest
from fastapi import HTTPException
from bioxp import api
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp import operator_controls


@pytest.mark.parametrize('status', ['completed', 'failed', 'interrupted', 'ambiguous', 'cleared', 'rejected'])
def test_mov_uses_completion_and_preserves_terminal_receipt(monkeypatch, status):
    receipt = {'status': status, 'terminal_evidence': {'response': {'partial': True}}}
    completion = Future()
    completion.set_result({'receipt': receipt})
    store = SimpleNamespace(workflow_context=lambda *a, **k: contextlib.nullcontext(),
                            assert_workflow_current=lambda *a: None,
                            workflow_child_completion=lambda command_id: completion,
                            _stop=threading.Event())
    monkeypatch.setattr(api, '_protocol_command_store', lambda: store)
    monkeypatch.setattr(api.app.state, 'oem_mov_execution_admitter', lambda *a, **k: {'command_id': 'child'}, raising=False)
    state = SimpleNamespace(job_id='parent', workflow=SimpleNamespace(child_command_ids=[]))
    result = api._protocol_source_mov(object(), SimpleNamespace(source_occurrence_id='step'), state)
    assert result == {'partial': True, 'ok': status == 'completed', 'command_id': 'child', 'receipt': receipt}
    assert state.workflow.child_command_ids == ['child']


def test_mov_owner_loss_does_not_read_receipt(monkeypatch):
    stop = threading.Event()
    stop.set()
    store = SimpleNamespace(workflow_context=lambda *a, **k: contextlib.nullcontext(),
                            assert_workflow_current=lambda *a: None,
                            workflow_child_completion=lambda command_id: Future(), _stop=stop)
    monkeypatch.setattr(api, '_protocol_command_store', lambda: store)
    monkeypatch.setattr(api.app.state, 'oem_mov_execution_admitter', lambda *a, **k: {'command_id': 'child'}, raising=False)
    with pytest.raises(RuntimeError, match='workflow_child_owner_lost'):
        api._protocol_source_mov(object(), SimpleNamespace(source_occurrence_id='step'),
                                 SimpleNamespace(job_id='parent', workflow=None))

