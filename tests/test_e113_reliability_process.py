"""Actual inert process owners, natural leases, real SQLite acquisition.

Controller seams are the established connected fixture; no live transport.
The subprocess entrypoint imports the same offline guard as pytest.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest
from bioxp.operator_command_plane import OperatorCommandStore


HERE = Path(__file__).resolve()


def eventually(predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        threading.Event().wait(.05)
    raise AssertionError('condition did not become true')


def emit(name, value):
    target = os.environ.get('E113_RELIABILITY_EXPORT')
    if target:
        root = Path(target)
        root.mkdir(parents=True, exist_ok=True)
        (root / (name + '.json')).write_text(json.dumps(value, indent=2, default=str))


def owner_main(directory, mode):
    import z_stop_offline_guard  # also protects subprocess imports and I/O
    from tests.test_deck_scoped_authority import retained_rig
    from tests.test_cover_carry_release_connected import connected
    from tests.test_deck_current_owner_v14 import state
    from bioxp.oem_deck_movement import compile_finite_plate_operation
    root = Path(directory)
    mp = pytest.MonkeyPatch()
    real_sleep = time.sleep
    os.environ['BIOXP_OEM_RUNTIME_STATE_ROOT'] = str(root / 'retained-copy')
    generator = retained_rig.__wrapped__(mp, root)
    retained = next(generator)
    r = connected.__wrapped__(retained, mp)
    time.sleep = real_sleep  # fixture's accelerated source sleeps not lease polling
    store = r.store
    store.bind_deck_owner_authority_reader(r.provider.deck_owner_authority_stamps,
        scope=r.provider.deck_owner_authority_scope)
    plan = compile_finite_plate_operation('send_z_and_gripper_home',
        source_leaf_available=True, run_in_parallel=True, gripper_position=0, closed_position=0)
    store.bind_workflow_dispatcher(lambda command: None)
    store.admit_workflow(command_id='owner-parent', idempotency_key='owner-parent',
        plan_fingerprint='inert-owner', requested_inputs={'bundle': {'execution': {'runtime_state': {}}}},
        ownership_generation=3, resources=('axis:x', 'axis:y', 'axis:z', 'gripper'), board_epochs={})
    assert store.claim_next()['command_id'] == 'owner-parent'
    with store.workflow_context('owner-parent', source_occurrence_id='inert-child'):
        admitted = store.admit_internal_wp8_operation('send_z_and_gripper_home',
            inputs={'run_in_parallel': True}, state=state(r.provider),
            idempotency_key='owner-child', prepared_plan=plan)
    cid = admitted['command_id']
    claimed = None if mode == 'queued' else store.claim_next()
    if claimed:
        store.persist_wp8_plan(cid, plan, authority_stamps=r.provider.deck_owner_authority_stamps())
    if mode in {'partial', 'issued_pending', 'stop_requested', 'abort_requested', 'background'}:
        child = plan['children'][0]
        store.record_delivery_attempt(cid, work_kind='wp8_child',
            work_identity=f"child:{child['order']}:{child['operation']}", plan_digest=plan['plan_digest'])
    if mode in {'issued_pending', 'background'}:
        background = next(child for child in plan['children'] if not child.get('awaited', True))
        store.create_wp8_background_task(cid, background['order'], task_id='inert-gripper',
            task_kind=background['operation'], plan_digest=plan['plan_digest'],
            authority_stamps=r.provider.deck_owner_authority_stamps())
        store.mark_wp8_background_task(cid, background['order'], state='running', evidence={'controlled_partial_delivery': True})
        worker = threading.Thread(target=lambda: threading.Event().wait(90), daemon=True,
            name='inert-registered-background')
        store.start_wp8_background_worker(worker, command_id=cid)
        if mode == 'issued_pending':
            store.mark_dispatched(cid, payload={'response': {'ok': True, 'background_pending': True}})
    if mode in {'stop_requested', 'abort_requested'}:
        action = 'oem.x.stop' if mode == 'stop_requested' else 'oem.abort_all'
        receipt = store.begin_interrupt(action, state=state(r.provider), request={'idempotency_key': 'inert-interrupt'})
        store.mark_interrupt_attempted(idempotency_key='inert-interrupt')
    # Real dispatcher renews the existing live-owner lease. Parent is already
    # dispatched and child is either queued or owned; no transport is submitted.
    dispatches = []
    if mode != 'queued':
        store.start(dispatches.append)
    descendant = subprocess.Popen([sys.executable, str(HERE), 'descendant', str(root)])
    eventually(lambda: (root / 'descendant.json').exists())
    record = {'pid': os.getpid(), 'descendants': json.loads((root / 'descendant.json').read_text()),
        'root': str(store.root), 'owner_id': store.owner_id, 'command_id': cid,
        'claimed': claimed, 'plan': plan, 'mode': mode,
        'lane': dict(store.connection.execute('SELECT * FROM operator_plane_lane').fetchone()),
        'attempts': [dict(row) for row in store.connection.execute('SELECT * FROM operator_plane_delivery_attempts WHERE command_id=?', (cid,))]}
    (root / 'ready.json').write_text(json.dumps(record))
    def stop(signum, frame):
        # Keep the actual shutdown guard active with an inert finite worker.
        worker = threading.Thread(target=lambda: threading.Event().wait(90), daemon=True)
        store.start_wp8_background_worker(worker, command_id=cid)
        try:
            store.stop()
        except RuntimeError as exc:
            (root / 'shutdown.json').write_text(json.dumps({'error': str(exc),
                'lane': dict(store.connection.execute('SELECT * FROM operator_plane_lane').fetchone())}))
        threading.Event().wait(90)
    signal.signal(signal.SIGTERM, stop)
    threading.Event().wait(120)


def spawn_owner(tmp_path, mode, managed=False):
    directory = tmp_path / 'owner'
    directory.mkdir()
    argv = [sys.executable, str(HERE), 'owner', str(directory), mode]
    log = (tmp_path / 'owner.log').open('w')
    unit = 'e113-inert-' + str(os.getpid()) + '-' + mode
    if managed:
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
            '-p', 'TimeoutStopSec=20s', '-p', 'KillMode=control-group', '-p', 'SendSIGKILL=yes',
            '--working-directory=' + str(HERE.parents[1])]
        for key in ('PYTHONPATH', 'DECK_RETAINED_BASELINE', 'TMPDIR', 'PYTHONDONTWRITEBYTECODE'):
            command += ['--setenv=' + key + '=' + os.environ.get(key, '')]
        result = subprocess.run(command + argv, capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        process = None
    else:
        process = subprocess.Popen(argv, stdout=log, stderr=log, start_new_session=True)
    try:
        record = eventually(lambda: json.loads((directory / 'ready.json').read_text())
            if (directory / 'ready.json').exists() else None, 25)
    except Exception:
        if process:
            process.kill()
            process.wait()
        raise AssertionError((tmp_path / 'owner.log').read_text())
    finally:
        log.close()
    return process, record, unit, directory


def proc_live(pid):
    path = Path('/proc') / str(pid) / 'stat'
    return path.exists() and path.read_text().split(') ')[1][0] != 'Z'


def reap_descendants(pids):
    for pid in pids:
        try:
            os.waitpid(pid, os.WNOHANG)
        except ChildProcessError:
            pass
    return all(not Path('/proc', str(pid)).exists() for pid in pids)


@pytest.fixture(autouse=True)
def process_subreaper():
    # Own/reap orphaned inert descendants after SIGKILL, rather than merely
    # treating zombies as successful cleanup. Never touches another process.
    import ctypes
    libc = ctypes.CDLL(None, use_errno=True)
    previous = ctypes.c_int()
    assert libc.prctl(37, ctypes.byref(previous), 0, 0, 0) == 0
    assert libc.prctl(36, 1, 0, 0, 0) == 0
    yield
    assert libc.prctl(36, previous.value, 0, 0, 0) == 0


@pytest.mark.parametrize('mode', ['queued', 'dispatched', 'partial', 'issued_pending', 'stop_requested', 'abort_requested', 'background'])
@pytest.mark.parametrize('acquire', ['constructor', 'start'])
def test_actual_owner_death_natural_acquisition_before_dispatch(tmp_path, mode, acquire):
    process, old, unit, directory = spawn_owner(tmp_path, mode)
    replacement = None
    dispatched = []
    try:
        replacement = OperatorCommandStore(Path(old['root'])) if acquire == 'start' else None
        if replacement:
            assert not replacement._owner_acquired
            assert replacement.owner_id != old['owner_id']
        killed_at = time.time()
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)
        eventually(lambda: reap_descendants(old['descendants']))
        if acquire == 'constructor':
            lease = old['lane']['owner_lease_until']
            threading.Event().wait(max(0, lease - time.time()) + .1)
            replacement = OperatorCommandStore(Path(old['root']))
        else:
            first_dispatcher_entry = threading.Event()
            real_loop = replacement._dispatch_loop
            def recovered_before_dispatch(callback):
                current = replacement.connection.execute('SELECT status FROM operator_plane_commands WHERE command_id=?', (old['command_id'],)).fetchone()[0]
                assert current == ('queued' if mode == 'queued' else ('failed' if mode == 'dispatched' else 'interrupted'))
                first_dispatcher_entry.set()
                return real_loop(callback)
            replacement._dispatch_loop = recovered_before_dispatch
            replacement.start(dispatched.append)
            assert first_dispatcher_entry.wait(1)
        assert replacement._owner_acquired
        row = replacement.get_command(old['command_id'])
        internal = dict(replacement.connection.execute('SELECT * FROM operator_plane_commands WHERE command_id=?', (old['command_id'],)).fetchone())
        if mode == 'queued':
            assert internal['status'] == 'queued'  # unclaimed intent is not fabricated delivery
        else:
            assert internal['status'] in {'failed', 'interrupted'}
            evidence = json.loads(internal['terminal_json'])
            assert evidence['reason'].startswith('process_owner_loss')
            assert evidence['delivery_attempted'] == (mode != 'dispatched')
        attempts = [dict(x) for x in replacement.connection.execute('SELECT * FROM operator_plane_delivery_attempts WHERE command_id=?', (old['command_id'],))]
        assert attempts == old['attempts']
        assert not dispatched
        new_dispatches = []
        if mode != 'queued':
            fresh_done = threading.Event()
            def fresh_workflow(command):
                # Actual new admission/claim/worker must see the old command
                # already reconciled, without replaying its original attempt.
                assert command['command_id'] == 'replacement-fresh-work'
                assert replacement.connection.execute('SELECT status FROM operator_plane_commands WHERE command_id=?',
                    (old['command_id'],)).fetchone()[0] in {'failed', 'interrupted'}
                new_dispatches.append(command['command_id'])
                replacement.finish_workflow(command['command_id'], status='completed',
                    payload={'execution': {'runtime_state': {}}}, lifecycle_settled=True)
                fresh_done.set()
            replacement.bind_workflow_dispatcher(fresh_workflow)
            replacement.admit_workflow(command_id='replacement-fresh-work', idempotency_key='replacement-fresh-work',
                plan_fingerprint='fresh-inert', requested_inputs={'bundle': {'execution': {'runtime_state': {}}}},
                ownership_generation=3, resources=('axis:x',), board_epochs={})
            replacement.start(dispatched.append)
            assert fresh_done.wait(3)
            assert new_dispatches == ['replacement-fresh-work']
            assert not dispatched
        emit(mode + '-' + acquire, {'old': old, 'killed_at': killed_at, 'acquired_at': time.time(),
            'terminal': row, 'internal': internal, 'attempts': attempts, 'dispatched': dispatched,
            'new_dispatches_after_recovery': new_dispatches})
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        if replacement:
            replacement.stop()
            replacement.connection.close()


def test_actual_renewing_live_owner_cannot_be_stolen(tmp_path):
    process, old, _, _ = spawn_owner(tmp_path, 'partial')
    challenger = OperatorCommandStore(Path(old['root']))
    try:
        assert not challenger._owner_acquired
        with pytest.raises(RuntimeError, match='ownership unavailable'):
            challenger.start(lambda command: pytest.fail('duplicate live dispatch'))
        lane = dict(challenger.connection.execute('SELECT * FROM operator_plane_lane').fetchone())
        assert lane['owner_id'] == old['owner_id']
        assert lane['owner_lease_until'] > time.time()
        assert proc_live(old['pid'])
        assert challenger.get_command(old['command_id'])['status'] == 'dispatched'
        emit('live-owner-protected', {'old': old, 'current_lane': lane})
    finally:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        eventually(lambda: reap_descendants(old['descendants']))
        challenger.stop()
        challenger.connection.close()


def test_isolated_systemd_control_group_stop_kills_tree_and_retains_guard(tmp_path):
    process, old, unit, directory = spawn_owner(tmp_path, 'partial', managed=True)
    challenger = OperatorCommandStore(Path(old['root']))
    try:
        before = subprocess.run(['systemctl', '--user', 'show', unit,
            '-p', 'MainPID', '-p', 'ControlGroup', '-p', 'TimeoutStopUSec', '-p', 'KillMode', '-p', 'SendSIGKILL'],
            capture_output=True, text=True, check=True).stdout
        control_group = next(line.split('=', 1)[1] for line in before.splitlines() if line.startswith('ControlGroup='))
        memberships = {str(pid): Path('/proc', str(pid), 'cgroup').read_text()
            for pid in [old['pid'], *old['descendants']]}
        assert all(control_group in membership for membership in memberships.values())
        started = time.time()
        stop = subprocess.Popen(['systemctl', '--user', 'stop', unit], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        shutdown = eventually(lambda: json.loads((directory / 'shutdown.json').read_text())
            if (directory / 'shutdown.json').exists() else None)
        assert shutdown['lane']['owner_id'] == old['owner_id']
        assert 25 < shutdown['lane']['owner_lease_until'] - time.time() <= 30
        assert not challenger._acquire_owner()  # actual still-live shutdown guard
        stdout, stderr = stop.communicate(timeout=28)
        assert stop.returncode == 0, stderr.decode()
        stopped_at = time.time()
        pids = [old['pid'], *old['descendants']]
        eventually(lambda: all(not Path('/proc', str(pid)).exists() for pid in pids), 10)
        after_stop_lane = dict(challenger.connection.execute('SELECT * FROM operator_plane_lane').fetchone())
        remaining = after_stop_lane['owner_lease_until'] - time.time()
        threading.Event().wait(max(0, remaining) + .2)
        challenger.start(lambda command: pytest.fail('abandoned work replayed'))
        assert challenger.get_command(old['command_id'])['status'] == 'ambiguous'
        emit('managed-stop', {'old': old, 'unit_properties': before, 'shutdown': shutdown,
            'stop_elapsed_s': stopped_at - started, 'acquisition_after_stop_s': time.time() - stopped_at,
            'after_stop_lane': after_stop_lane, 'cgroup_memberships': memberships, 'reaped_pids': pids,
            'receipt': challenger.get_command(old['command_id'])})
    finally:
        subprocess.run(['systemctl', '--user', 'stop', unit], capture_output=True)
        challenger.stop()
        challenger.connection.close()


if __name__ == '__main__':
    if sys.argv[1] == 'owner':
        owner_main(sys.argv[2], sys.argv[3])
    else:
        root = Path(sys.argv[2])
        child = subprocess.Popen([sys.executable, '-c',
            'import signal,threading; signal.signal(signal.SIGTERM,signal.SIG_IGN); threading.Event().wait(120)'])
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        (root / 'descendant.json').write_text(json.dumps([os.getpid(), child.pid]))
        threading.Event().wait(120)
