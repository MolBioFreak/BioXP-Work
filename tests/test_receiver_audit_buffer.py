"""Real governed disposable DB + sole router; no physical/device claims."""
import json
import fcntl
import os
import queue
import sqlite3
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import novo_decode, novo_encode
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.receiver_audit_buffer import ReceiverAuditBuffer
from bioxp.runtime_audit_store import RuntimeAuditDatabase, runtime_write_coordinator
from bioxp.usb_driver import BioXpTester


def wait(predicate, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.002)
    assert predicate(), "condition not reached"


@pytest.fixture
def root(tmp_path):
    root = tmp_path / "runtime"
    owner = OEMRuntimeStore(root)
    owner.close()
    return root


def rows(root):
    with sqlite3.connect(root / "bioxp_runtime.db") as db:
        return [(kind, json.loads(payload), command, transaction) for kind, payload, command, transaction in db.execute(
            "SELECT event_kind,event_json,command_id,transaction_id FROM runtime_events "
            "WHERE event_source='novo_receiver_audit' ORDER BY event_id")]


def wire(status=128, command=0, module=99):
    return novo_encode(module.to_bytes(4, 'big') + bytes([8, 4, status, command, 0, 125, 126, 0, 0]))


class Endpoints:
    def __init__(self):
        self.frames = queue.Queue()
        self.writes = []
        self.readers = set()

    def read(self, size, timeout):
        self.readers.add(threading.get_ident())
        try:
            return self.frames.get(timeout=.01)
        except queue.Empty:
            raise TimeoutError()

    def write(self, frame, timeout):
        self.writes.append(bytes(frame))
        payload = novo_decode(bytes(frame))
        board, command = payload[3], payload[5]
        self.frames.put(novo_encode(bytes([0, 0, 0, 0, 8, board, 100, command, 0, 0, 0, 0, 0])))
        return len(frame)


def router_for(buffer):
    ep = Endpoints()
    router = NovoRouter(ep_in=ep, ep_out=ep, decode=novo_decode, audit_buffer=buffer, queue_size=2)
    router.start()
    return router, ep


def test_real_ingress_latches_drain_raw_nullable_and_full_wal(root):
    audit = ReceiverAuditBuffer(root=root)
    router, ep = router_for(audit)
    try:
        raw = wire()
        ep.frames.put(raw)
        ep.frames.put(raw)
        ep.frames.put(raw)
        ep.frames.put(b'bad')
        wait(lambda: router.receive_cursor()['after_sequence'] == 3 and len(router._queues['malformed']) == 1)
        assert len(ep.readers) == 1
        assert len(router.take_motor_events([(4, 0)])) == 1
        assert router.take_motor_events([(4, 0)]) == []
        router.reset_motor_event(4, 0)
    finally:
        router.shutdown()
        status = audit.close(5)
    assert status['clean_drain_committed']
    assert status['committed_records'] == status['accepted_volatile']
    assert status['buffered_records'] == status['buffered_bytes'] == 0
    assert status['durable_ownership_claimed'] is False
    events = rows(root)
    ingress = [p for k, p, _, _ in events if k == 'ingress']
    assert len(ingress) == 3
    assert ingress[0]['raw'] == list(raw)
    assert ingress[0]['data'] == list(novo_decode(raw)[5:])
    assert [p['receive_sequence'] for p in ingress] == [1, 2, 3]
    assert all(c is None and t is None for _, _, c, t in events)
    kinds = [k for k, _, _, _ in events]
    for k in ('opened', 'closed', 'motor_set', 'motor_coalesced_set', 'motor_consume', 'motor_reset',
              'reader_started', 'reader_stopped', 'malformed', 'routing_queue_eviction'):
        assert k in kinds
    db = RuntimeAuditDatabase(root)
    try:
        assert db.connection.execute('PRAGMA synchronous').fetchone()[0] == 2
        assert db.connection.execute('PRAGMA journal_mode').fetchone()[0] == 'wal'
        with pytest.raises(sqlite3.IntegrityError):
            db.connection.execute("UPDATE runtime_events SET event_kind='changed'")
    finally:
        db.close()


@pytest.mark.parametrize('contention', ['sqlite', 'lifecycle', 'coordinator'])
def test_real_sqlite_lock_does_not_block_actual_double_stop_or_reader(root, contention):
    audit = ReceiverAuditBuffer(root=root)
    wait(lambda: audit.status()['writer_open_committed'])
    if contention == 'sqlite':
        lock = sqlite3.connect(root / 'bioxp_runtime.db')
        lock.execute('BEGIN IMMEDIATE')
        def release():
            lock.rollback()
            lock.close()
    elif contention == 'lifecycle':
        fd = os.open(root / 'runtime-storage.lifecycle.lock', os.O_RDWR)
        fcntl.flock(fd, fcntl.LOCK_EX)
        def release():
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)
    else:
        lock = runtime_write_coordinator(root).lock
        lock.acquire()
        release = lock.release
    router, ep = router_for(audit)
    tester = BioXpTester.__new__(BioXpTester)
    tester.novo_router = router
    tester._motor_last_tx_ts = {}
    tester._motor_noresp_streak = {}
    tester._oem_board_initialized = {4: True}
    tester._oem_active_board_lifecycle_generation = 1
    tester.ep_in = tester.ep_out = ep
    done = threading.Event()
    result = []
    errors = []
    def stop():
        try:
            result.append(tester.motor_oem_board_stop(4, motor=0))
        except Exception as exc:
            errors.append(repr(exc))
        finally:
            done.set()
    worker = threading.Thread(target=stop)
    worker.start()
    try:
        assert done.wait(1), 'Stop did not finish within existing offline 1s ceiling under DB lock'
        assert not errors
        assert len(ep.writes) == 2  # existing OEM double delivery, no new retries
        assert result[0]['double_stop_acknowledged']
        assert result[0]['terminal_speed_zero'] is None
        assert router.receive_cursor()['after_sequence'] == 2
        state = audit.status()
        assert state['committed_records'] == 0
        assert state['buffered_records'] > 0
        assert not state['clean_drain_committed']
        router.shutdown()
        state = audit.close(0)
        assert not state['finished'] and not state['clean_drain_committed']
    finally:
        release()
        worker.join(5)
        router.shutdown()
        audit.close(5)
    assert audit.status()['clean_drain_committed']
    assert len([1 for k, _, _, _ in rows(root) if k == 'ingress']) == 2


@pytest.mark.parametrize('bound', ['records', 'bytes'])
def test_bounds_include_inflight_and_overflow_is_durable_gap(root, bound):
    gate, entered = threading.Event(), threading.Event()
    class Paused(RuntimeAuditDatabase):
        def record_event(self, **kw):
            if kw['event_kind'] == 'fixture':
                entered.set()
                assert gate.wait(5)
            return super().record_event(**kw)
    audit = ReceiverAuditBuffer(root=root, max_records=2 if bound == 'records' else 100,
        max_bytes=10000 if bound == 'records' else 100, database_factory=Paused)
    try:
        assert audit.offer('fixture', {'x': 'a' * 30})
        assert entered.wait(5)
        assert audit.offer('fixture', {'x': 'a' * 30})
        assert not audit.offer('fixture', {'x': 'a' * 30})
        s = audit.status()
        assert s['buffered_records'] == 2 and s['overflow_records'] == 1
        assert s['inflight_records'] == 1 and s['queued_records'] == 1
        assert s['committed_records'] == 0
        assert s['buffered_bytes'] <= s['max_bytes']
    finally:
        gate.set()
        s = audit.close(5)
    assert s['committed_records'] == 2
    assert s['committed_gap_counts'] == (1, 0, 0)
    assert s['clean_drain_committed']
    assert [p for k, p, _, _ in rows(root) if k == 'closed'][0]['lossless'] is False


def test_write_failure_gap_without_false_commit_or_decode_change(root):
    class FailsIngress(RuntimeAuditDatabase):
        def record_event(self, **kw):
            if kw['event_kind'] == 'ingress':
                raise OSError(28, 'injected ENOSPC')
            return super().record_event(**kw)
    audit = ReceiverAuditBuffer(root=root, database_factory=FailsIngress)
    router, ep = router_for(audit)
    try:
        ep.frames.put(wire())
        wait(lambda: router.receive_cursor()['after_sequence'] == 1)
        assert not router._queues['malformed']
        assert len(router.take_motor_events([(4, 0)])) == 1
    finally:
        router.shutdown()
        s = audit.close(5)
    assert s['write_failed_records'] == 1
    assert s['committed_records'] == s['accepted_volatile'] - 1
    assert not s['clean_drain_committed'] and s['close_accounting_committed']
    assert not s['healthy']
    gaps = [p for k, p, _, _ in rows(root) if k == 'gap']
    assert gaps[-1]['write_failed_or_commit_uncertain_records'] == 1
    assert gaps[-1]['crash_lost_count'] is None


def test_permanent_writer_failure_bounded_rejections_and_no_clean_claim(root):
    audit = ReceiverAuditBuffer(root=root, database_factory=lambda _root: (_ for _ in ()).throw(OSError('disk failed')))
    wait(lambda: audit.status()['finished'])
    router, ep = router_for(audit)
    try:
        for _ in range(20):
            ep.frames.put(wire())
        wait(lambda: router.receive_cursor()['after_sequence'] == 20)
        assert len(router._queues['valid_async']) == 2
        assert not router._queues['malformed']
    finally:
        router.shutdown()
        s = audit.close(1)
    assert s['rejected_records'] > 20
    assert s['committed_records'] == 0 and s['buffered_records'] == 0
    assert not s['clean_drain_committed']
    assert rows(root) == []


def test_hook_exception_cannot_become_decode_failure(root):
    class Broken:
        def offer(self, *_):
            raise RuntimeError('handoff broken')
    router, ep = router_for(Broken())
    try:
        ep.frames.put(wire())
        wait(lambda: router.receive_cursor()['after_sequence'] == 1)
        assert not router._queues['malformed']
        assert len(router.take_motor_events([(4, 0)])) == 1
        assert router._audit_hook_failures > 0
    finally:
        router.shutdown()


def test_ambient_transaction_is_never_counted_committed(root):
    class Ambient(RuntimeAuditDatabase):
        def record_event(self, **kw):
            if kw['event_kind'] == 'fixture':
                self.connection.execute('BEGIN IMMEDIATE')
            return super().record_event(**kw)
    audit = ReceiverAuditBuffer(root=root, database_factory=Ambient)
    audit.offer('fixture', {})
    s = audit.close(5)
    assert s['committed_records'] == 0
    assert s['write_failed_records'] == 1
    assert not s['clean_drain_committed']
    assert not [p for k, p, _, _ in rows(root) if k == 'fixture']


def test_clean_restart_lifecycle(root):
    first = ReceiverAuditBuffer(root=root)
    assert first.offer('fixture', {})
    assert first.close(5)['clean_drain_committed']
    second = ReceiverAuditBuffer(root=root)
    assert second.close(5)['clean_drain_committed']
    opened = [p for k, p, _, _ in rows(root) if k == 'opened']
    assert opened[0]['unclean_previous_session'] is None
    assert opened[1]['unclean_previous_session'] is False
    assert opened[1]['previous_session'] == first.session
    assert opened[1]['crash_lost_count'] is None
    assert opened[1]['pre_open_ingress_loss_unknown'] is True


def test_process_exit_restart_reports_unknown_loss_not_exact_count(root):
    code = '''
import os, sqlite3, time
from bioxp.receiver_audit_buffer import ReceiverAuditBuffer
b = ReceiverAuditBuffer(root=os.environ['FIXTURE_ROOT'])
while not b.status()['writer_open_committed']:
    time.sleep(.001)
lock = sqlite3.connect(os.path.join(os.environ['FIXTURE_ROOT'], 'bioxp_runtime.db'))
lock.execute('BEGIN IMMEDIATE')
assert b.offer('fixture', {'uncommitted': True})
os._exit(42)
'''
    result = subprocess.run([sys.executable, '-c', code], env={**os.environ, 'FIXTURE_ROOT': str(root)}, timeout=10)
    assert result.returncode == 42
    second = ReceiverAuditBuffer(root=root)
    assert second.close(5)['clean_drain_committed']
    opened = [p for k, p, _, _ in rows(root) if k == 'opened']
    assert len(opened) == 2
    assert opened[1]['unclean_previous_session'] is True
    assert opened[1]['crash_lost_count'] is None
    assert not [p for k, p, _, _ in rows(root) if k == 'fixture']


def test_owner_connect_disconnect_reuses_pending_writer_no_usb_wait(root, monkeypatch):
    import bioxp.usb_driver as driver
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_STATE_ROOT', str(root))
    ep = Endpoints()
    device = SimpleNamespace(is_kernel_driver_active=lambda _: False, set_configuration=lambda: None,
        set_interface_altsetting=lambda **_: None, get_active_configuration=lambda: {(0, 0): object()})
    monkeypatch.setattr(driver.usb.core, 'find', lambda **_: device)
    monkeypatch.setattr(driver.usb.util, 'claim_interface', lambda *_: None)
    monkeypatch.setattr(driver.usb.util, 'release_interface', lambda *_: None)
    monkeypatch.setattr(driver.usb.util, 'dispose_resources', lambda *_: None)
    monkeypatch.setattr(driver.usb.util, 'find_descriptor', lambda *_a, **_k: ep)
    tester = BioXpTester.__new__(BioXpTester)
    tester.alt = 0
    tester.novo_router = None
    tester._connect()
    audit = tester._receiver_audit
    wait(lambda: audit.status()['writer_open_committed'])
    lock = sqlite3.connect(root / 'bioxp_runtime.db')
    lock.execute('BEGIN IMMEDIATE')
    try:
        ep.frames.put(wire())
        wait(lambda: tester.novo_router.receive_cursor()['after_sequence'] == 1)
        start = time.monotonic()
        summary = tester._disconnect()
        assert time.monotonic() - start < 1
        assert summary['router_shutdown_ok'] is True
        assert tester.novo_router is None
        assert not audit.status()['clean_drain_committed']
        tester._connect()
        assert tester._receiver_audit is audit  # no second SQLite worker under arbitrary stall
        ep.frames.put(wire())
        wait(lambda: tester.novo_router.receive_cursor()['after_sequence'] == 1)
        assert tester.receiver_audit_status()['buffer']['rejected_records'] > 0
        tester._disconnect()
        # API can discard the previous driver after USB release. A new owner
        # must still not create another writer while the old commit is blocked.
        replacement = BioXpTester.__new__(BioXpTester)
        replacement.alt = 0
        replacement.novo_router = None
        replacement._connect()
        assert replacement._receiver_audit is audit
        assert replacement.novo_router.running
        replacement._disconnect()
    finally:
        lock.rollback()
        lock.close()
        if tester.novo_router is not None:
            tester._disconnect()
        audit.close(5)


@pytest.mark.parametrize('records,bytes_', [(0, 1), (1, 0), (-1, 10), (1.5, 10)])
def test_invalid_bounds_are_explicit(records, bytes_):
    with pytest.raises(ValueError):
        ReceiverAuditBuffer(max_records=records, max_bytes=bytes_)


def test_oversized_record_and_late_offer_do_not_claim_durable_drain(root):
    audit = ReceiverAuditBuffer(root=root, max_bytes=16)
    assert not audit.offer('oversized', {'raw': 'x' * 100})
    assert audit.status()['buffered_bytes'] == 0
    s = audit.close(5)
    assert s['clean_drain_committed'] and s['committed_gap_counts'] == (1, 0, 0)
    assert not audit.offer('late', {})
    assert not audit.close(0)['clean_drain_committed']
    assert audit.status()['rejected_records'] == 1


def test_exception_after_real_commit_is_uncertain_not_retried(root):
    class AfterCommit(RuntimeAuditDatabase):
        def record_event(self, **kw):
            result = super().record_event(**kw)
            if kw['event_kind'] == 'fixture':
                raise OSError('injected failure after COMMIT returned')
            return result
    audit = ReceiverAuditBuffer(root=root, database_factory=AfterCommit)
    assert audit.offer('fixture', {'one_attempt': True})
    s = audit.close(5)
    assert s['committed_records'] == 0 and s['write_failed_records'] == 1
    assert len([p for k, p, _, _ in rows(root) if k == 'fixture']) == 1
    gap = [p for k, p, _, _ in rows(root) if k == 'gap'][-1]
    assert gap['write_failed_or_commit_uncertain_records'] == 1
    assert gap['crash_lost_count'] is None


def test_gap_write_failure_leaves_unclean_marker_and_exposed_health(root):
    class Broken(RuntimeAuditDatabase):
        def record_event(self, **kw):
            if kw['event_kind'] in ('fixture', 'gap'):
                raise sqlite3.OperationalError('injected permanent disk I/O failure')
            return super().record_event(**kw)
    first = ReceiverAuditBuffer(root=root, database_factory=Broken)
    first.offer('fixture', {})
    s = first.close(5)
    assert s['write_failed_records'] == 1
    assert not s['clean_drain_committed']
    assert s['committed_gap_counts'] == (0, 0, 0)
    second = ReceiverAuditBuffer(root=root)
    assert second.close(5)['clean_drain_committed']
    opened = [p for k, p, _, _ in rows(root) if k == 'opened'][-1]
    assert opened['unclean_previous_session'] is True
    assert opened['crash_lost_count'] is None


@pytest.mark.parametrize('terminal', ['opened_failure', 'gap_failure', 'normal'])
def test_terminal_close_rejects_offers_without_stranding_accepted_records(root, terminal):
    entered, release = threading.Event(), threading.Event()
    closing, release_close = threading.Event(), threading.Event()

    class PausedClose(RuntimeAuditDatabase):
        def record_event(self, **kw):
            kind = kw['event_kind']
            pause_kind = 'opened' if terminal == 'opened_failure' else 'fixture'
            if kind == pause_kind:
                entered.set()
                assert release.wait(5)
                if terminal != 'normal':
                    raise sqlite3.OperationalError('injected terminal write failure')
            if terminal == 'gap_failure' and kind == 'gap':
                raise sqlite3.OperationalError('injected terminal gap failure')
            return super().record_event(**kw)

        def close(self):
            closing.set()
            try:
                assert release_close.wait(5)
            finally:
                super().close()

    audit = ReceiverAuditBuffer(root=root, database_factory=PausedClose)
    try:
        assert audit.offer('fixture', {'first': True})
        assert entered.wait(5)
        assert audit.offer('queued', {'second': True})
        if terminal == 'normal':
            audit.close(0)
        release.set()
        assert closing.wait(5)
        before = audit.status()
        assert audit.alive and not before['finished']
        assert before['closing']
        assert before['accepted_volatile'] == 2
        assert before['committed_records'] == (2 if terminal == 'normal' else 0)
        assert before['write_failed_records'] == (0 if terminal == 'normal' else 2)
        assert before['clean_drain_committed'] == (terminal == 'normal')
        assert not audit.offer('during_db_close', {})
        during = audit.status()
        assert during['rejected_records'] == 1
        assert during['offered'] == 3
        assert not during['clean_drain_committed']
        assert during['committed_gap_counts'] == (0, 0, 0)
        assert during['close_accounting_committed'] == (terminal == 'normal')
    finally:
        release.set()
        release_close.set()
        audit._thread.join(5)
    final = audit.status()
    assert final['finished'] and not audit.alive
    assert not audit.offer('after_finished', {})
    for state in (during, final, audit.status()):
        assert state['buffered_records'] == state['buffered_bytes'] == 0
        assert state['queued_records'] == state['inflight_records'] == state['inflight_bytes'] == 0
        assert state['accepted_volatile'] == state['committed_records'] + state['write_failed_records']
        assert state['offered'] == state['accepted_volatile'] + state['overflow_records'] + state['rejected_records']
        assert not state['clean_drain_committed']
    events = rows(root)
    if terminal == 'normal':
        closed = [p for k, p, _, _ in events if k == 'closed'][0]
        assert closed['accepted_volatile'] == closed['committed_records'] == 2
        assert closed['offered_through'] == 2
        assert closed['gap_counts'] == [0, 0, 0]
        assert closed['later_rejected_offers_unknown'] is True
    else:
        assert not [k for k, _, _, _ in events if k in ('fixture', 'queued', 'closed', 'gap')]


def test_store_open_runs_only_on_writer_and_does_not_migrate(tmp_path):
    seen = []
    def factory(root):
        seen.append(threading.get_ident())
        return RuntimeAuditDatabase(root)
    audit = ReceiverAuditBuffer(root=tmp_path / 'unmigrated', database_factory=factory)
    s = audit.close(5)
    assert len(seen) == 1 and seen[0] != threading.get_ident()
    assert not s['writer_open_committed'] and not s['clean_drain_committed']
    assert 'no such table' in s['writer_error']
    with sqlite3.connect(tmp_path / 'unmigrated' / 'bioxp_runtime.db') as db:
        assert db.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall() == []
