"""Prepared plans wake on issued_pending without settling the terminal waiter."""
from concurrent.futures import Future
import types

from bioxp.operator_command_plane import OperatorCommandStore


class _Conn:
    def __init__(self, status):
        self.status = status

    def execute(self, sql, params):
        status = self.status
        return types.SimpleNamespace(fetchone=lambda: {"status": status})


def _store(status):
    import threading
    s = object.__new__(OperatorCommandStore)
    s._lock = threading.RLock()
    s._wake = threading.Event()
    s._workflow_child_waiters = {}
    s.connection = _Conn(status)
    s.get_command = lambda cid: {"command_id": cid, "status": s.connection.status}
    return s


def test_issued_pending_settles_only_the_opt_in_waiter():
    s = _store("issued_pending")
    terminal = s.workflow_child_completion("c")
    pending = s.workflow_child_completion("c", include_issued_pending=True)
    assert terminal is not pending
    s._settle_workflow_child_waiters()
    assert pending.done() and pending.result()["status"] == "issued_pending"
    assert not terminal.done()
    s.connection.status = "completed"
    s._settle_workflow_child_waiters()
    assert terminal.result()["ok"] is True
    assert s._workflow_child_waiters == {}


def test_default_waiter_is_terminal_only():
    s = _store("dispatched")
    f = s.workflow_child_completion("c")
    s._settle_workflow_child_waiters()
    assert not f.done()
    assert s.workflow_child_completion("c") is f
