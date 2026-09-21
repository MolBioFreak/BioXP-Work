"""Real dispatcher loop with offline predicates; no controller or provider."""
import sqlite3
import threading
import time
from types import SimpleNamespace
from contextlib import contextmanager

from bioxp.operator_command_plane import OperatorCommandStore


def test_empty_queue_never_enters_writer_or_history_query():
    db = sqlite3.connect(':memory:')
    db.execute('CREATE TABLE operator_plane_commands(status TEXT, stream_sequence INT)')
    db.execute('CREATE INDEX ready ON operator_plane_commands(status,stream_sequence)')
    statements = []
    db.set_trace_callback(statements.append)
    @contextmanager
    def forbidden():
        raise AssertionError('empty queue entered a write transaction')
        yield
    store = SimpleNamespace(connection=db, _lock=threading.RLock(), _transaction=forbidden)
    try:
        assert OperatorCommandStore.claim_next(store) is None
        assert len(statements) == 1
        assert 'status=' in statements[0] and 'LIMIT 1' in statements[0]
    finally:
        db.close()


class Loop:
    _dispatch_loop = OperatorCommandStore._dispatch_loop
    def __init__(self):
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._priority_fence = threading.Event()
        self._worker_lock = threading.Lock()
        self._workers = set()
        self._worker_commands = {}
        self.scans = []
        self.renewals = []
        self.reconciliations = 0
        self.scanned = threading.Event()
        self.inject = None
    def _renew_owner(self):
        self.renewals.append(time.monotonic())
        return True
    def reconcile_pending_interrupts(self):
        self.reconciliations += 1
    def _settle_workflow_child_waiters(self): pass
    def _notify_workflow_interrupt(self): pass
    def claim_next(self):
        self.scans.append(time.monotonic())
        self.scanned.set()
        if self.inject:
            action, self.inject = self.inject, None
            action()
        return None


def run_loop(loop):
    thread = threading.Thread(target=loop._dispatch_loop, args=(lambda _: None,))
    thread.start()
    return thread


def close(loop, thread):
    loop._stop.set()
    loop._wake.set()
    thread.join(3)
    assert not thread.is_alive()


def test_idle_has_bounded_scans_and_external_fallback():
    loop = Loop()
    thread = run_loop(loop)
    try:
        assert loop.scanned.wait(2)
        time.sleep(1.15)
        assert 2 <= len(loop.scans) <= 3  # old loop performs ~23 scans
        assert len(loop.renewals) >= 2
    finally:
        close(loop, thread)


def test_signal_during_scan_is_not_lost():
    loop = Loop()
    loop.inject = loop._wake.set
    thread = run_loop(loop)
    try:
        deadline = time.monotonic() + 2
        while len(loop.scans) < 2 and time.monotonic() < deadline:
            time.sleep(.005)
        assert len(loop.scans) >= 2
        assert loop.scans[1] - loop.scans[0] < .3
    finally:
        close(loop, thread)


def test_idle_notification_wakes_without_waiting_for_fallback():
    loop = Loop()
    thread = run_loop(loop)
    try:
        assert loop.scanned.wait(2)
        time.sleep(.05)
        before = time.monotonic()
        loop.scanned.clear()
        loop._wake.set()
        assert loop.scanned.wait(.5)
        assert loop.scans[-1] - before < .5
    finally:
        close(loop, thread)


def test_owner_loss_still_stops_and_fences():
    loop = Loop()
    loop._renew_owner = lambda: False
    thread = run_loop(loop)
    thread.join(2)
    assert not thread.is_alive()
    assert loop._stop.is_set() and loop._priority_fence.is_set()
    assert not loop.scans
