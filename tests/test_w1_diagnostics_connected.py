"""W1 exact-e113 differential producer/service/SQLite qualification, offline CAN only."""
import asyncio
import ast
import copy
import inspect
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import threading
import time
import timeit
from types import SimpleNamespace
import pytest
from bioxp.can_driver import BioXpCanDriver
from bioxp.novo_router import NovoRouter, NovoFrame
from bioxp.usb_driver import novo_decode
from bioxp.pipette import transport as tm, receipts as rm
from bioxp.pipette.models import PipetteInitCommand
from bioxp.pipette.audit import normalize_pipette_command_outcome
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.services.pipette_service import run_pipette_init_command
from tests.test_deck_tip_query_publication import bind_collection_test_identity, query_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig

BASE = 'e11351e282c9e2df35187aa704e73cd5ea0e0877'


def bind_full_infrastructure_identity(monkeypatch):
    # Borrow only the existing legal full identity shape, not a live deployment
    # claim. The release id/source below explicitly name this controlled fixture.
    capture = Path(__file__).with_name('w1_runtime_identity_fixture.json')
    identity = json.loads(capture.read_text())
    release = copy.deepcopy(identity['release_identity'])
    release['release_id'] = 'controlled-w1-offline-fixture'
    release['source'].update(commit=BASE, root=str(Path.cwd()), host_path=str(Path.cwd()),
                             manifest_sha256='1' * 64, aggregate_sha256='2' * 64)
    release['deployment'].update(receipt_id='controlled-w1-not-deployed', receipt_sha256='3' * 64)
    monkeypatch.setattr(rm, 'current_release_identity', lambda: copy.deepcopy(release))
    monkeypatch.setattr(rm, 'current_authority_identity', lambda: copy.deepcopy(identity['evidence_authority']))
    monkeypatch.setattr(rm, 'current_registry_sha256', lambda: identity['registry_sha256'])

def baseline_module(path, owner):
    source = subprocess.check_output(['git', 'show', BASE + ':' + path], text=True)
    namespace = {'__name__': owner.__name__ + '_w1_baseline', '__package__': owner.__package__}
    exec(compile(source, path + '@' + BASE, 'exec'), namespace)
    return SimpleNamespace(**namespace)


def artifact(name, value):
    if os.environ.get('W1_ARTIFACT_ROOT'):
        p = Path(os.environ['W1_ARTIFACT_ROOT']) / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(value, indent=2, sort_keys=True))


def stable(value):
    """Only remove nondeterministic execution/identity leaves, not truth/errors/offsets."""
    volatile = {'elapsed_ms', 'observed_at', 'last_updated', 'received_at', 'receive_timestamp',
                'tx_timestamp', 'tx_started_at', 'started_at', 'registered_at', 'deadline_at',
                'pressure_epoch_started_at', 'registration_timestamp', 'deadline_timestamp',
                'completion_owner_token', 'owner_token',
                'requested_owner_token', 'receive_owner', 'transaction_id', 'source_owner',
                'transport_identity', 'reader_identity', 'driver_identity', 'tip_source_reader',
                'tip_source_actor', 'callback_session_id'}
    if isinstance(value, dict):
        return {str(k): stable(v) for k, v in value.items() if k not in volatile}
    if isinstance(value, list): return [stable(x) for x in value]
    return value


def transport_rig(module, mode):
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    wire, timers = [], []
    status_counts, cycles = [0] * 4, [0] * 4
    def frame(channel, data, family=1, classification='pipette'):
        return NovoFrame(0x500 + channel * 8 + family, len(data), bytes(data), b'', time.monotonic(), classification)
    def transact(msg, *, channel, initialization, matcher_name, **kwargs):
        payload = bytes(msg.data)
        command = payload.decode('ascii', errors='replace')
        wire.append((channel, command))
        now = time.monotonic()
        txid = 'controlled-exchange-' + str(len(wire))
        token = None
        ack = True
        data = []
        if initialization:
            cycles[channel] += 1
            token = router.prepare_pipette_completion(channel, 10.0, command_family=1,
                command_name=matcher_name, expected_rx_id=0x501 + channel * 8)
            router.bind_pipette_completion(channel, owner_token=token, transaction_id=txid, tx_started_at=now)
            ack = not (mode == 'initial_ack_failure' and channel == 2 or
                       mode == 'retry_before_pressure' and cycles[channel] == 2 and channel == 2)
            if ack:
                router._dispatch(frame(channel, []))
                data = [0x21 if mode == 'completion_failure' and channel == 3 else 32, 96]
                if mode == 'missing_completion' and channel == 3:
                    pass
                elif mode == 'delayed_completion':
                    timer = threading.Timer(.02, lambda ch=channel, data=bytes(data): router._dispatch(frame(ch, data)))
                    timer.start(); timers.append(timer)
                else: router._dispatch(frame(channel, data))
            data = []
        elif command == 'Q1':
            status_counts[channel] += 1
            retry_mode = mode in {'conditional_retry', 'retry_before_pressure', 'retry_after_pressure'}
            data = [32,96,32,0x21] if (retry_mode and channel == 2 and status_counts[channel] == 1) or (mode == 'partial_channel' and channel == 2) else [32,96,32]
        elif command == '&1':
            data = [] if mode == 'condition_failure' and channel == 1 else [32,96,49]
        elif command == 'o0,1R':
            if mode != 'unknown_offsets' and (mode != 'partial_offsets' or channel < 2):
                pressure = (100 + channel + (cycles[channel] - 1) * 10).to_bytes(2, 'big', signed=True)
                router._dispatch(frame(channel, [49, *pressure], family=4, classification='pressure'))
            ack = not (mode == 'retry_after_pressure' and cycles[channel] == 2 and channel == 2)
        return {'ok': ack, 'immediate_ack_received': ack and not kwargs.get('allow_multipart'),
            'completion_deferred': initialization and ack,
            'completion_received': not initialization and ack and not kwargs.get('allow_multipart'),
            'query_response_correlated': bool(kwargs.get('allow_multipart')),
            'transaction_id': txid, 'tx_timestamp': now, 'receive_timestamp': time.monotonic(),
            'channel': channel, 'owner_generation': router.reader_generation,
            'outcome': ('ack' if initialization or not kwargs.get('allow_multipart') else 'query_response') if ack else 'timeout',
            'tx_ok': True, 'completion_owner_token': token,
            'frames': [{'data': data, 'dlc': len(data), 'arbitration_id': msg.arbitration_id + 0x400,
                        'received_at': time.monotonic()}] if ack else []}
    leaves = []
    for channel in range(4):
        driver = BioXpCanDriver.__new__(BioXpCanDriver)
        driver.pipette_id = channel
        driver.response_timeout_s = .01
        driver._pipette_completion_owner_token = None
        driver._sleep = lambda _: None
        driver.bus = SimpleNamespace(router=router, transact_can=transact,
            wait_pipette_completion=router.wait_pipette_completion, send=lambda _: None)
        leaf = module.CanPipetteTransport(driver_factory=lambda d=driver: d, pipette_id=channel)
        leaf._initialized = mode == 'already_initialized'
        leaves.append(leaf)
    return module.FourPipetteTransport(leaves, sleep=lambda _: None), wire, timers


async def inline(_label, operation, **kwargs): return operation()


MODES = ['initial', 'already_initialized', 'conditional_retry', 'initial_ack_failure',
         'completion_failure', 'missing_completion', 'delayed_completion', 'unknown_offsets',
         'partial_offsets', 'partial_channel', 'condition_failure', 'retry_before_pressure',
         'retry_after_pressure']

@pytest.mark.parametrize('mode', MODES)
def test_constructor_differential_sqlite(tmp_path, monkeypatch, mode):
    bind_collection_test_identity(monkeypatch)
    bind_full_infrastructure_identity(monkeypatch)
    baseline = baseline_module('src/bioxp/pipette/transport.py', tm)
    runs = []
    for label, module in [('baseline', baseline), ('candidate', tm)]:
        root = tmp_path / label
        runtime = OEMRuntimeStore(root)
        store = rm.PipetteReceiptStore(root)
        group, wire, timers = transport_rig(module, mode)
        construction = {'status_payload_calls': 0, 'transaction_attachments': 0,
                        'constructor_status_calls': 0, 'constructor_transaction_attachments': 0}
        initialize = group.initialize
        def measured_initialize(command):
            started = time.perf_counter()
            try: return initialize(command)
            finally: construction['transport_with_controlled_wait_seconds'] = time.perf_counter() - started
        group.initialize = measured_initialize
        payload = module.CanPipetteTransport._status_payload
        def measured_payload(self, **kwargs):
            construction['status_payload_calls'] += 1
            if kwargs.get('include_transaction', True): construction['transaction_attachments'] += 1
            in_constructor = any(f.function == 'initialize' and f.filename.split('@')[0].endswith('/pipette/transport.py') for f in inspect.stack())
            if in_constructor:
                construction['constructor_status_calls'] += 1
                if kwargs.get('include_transaction', True): construction['constructor_transaction_attachments'] += 1
            return payload(self, **kwargs)
        monkeypatch.setattr(module.CanPipetteTransport, '_status_payload', measured_payload)
        try:
            result = asyncio.run(run_pipette_init_command(PipetteInitCommand(), get_transport=lambda: group,
                run_blocking=inline, receipt_store=store, runtime_binding={
                    'entrypoint_id': 'lifecycle.constructor_pipette_stage', 'caller_class': 'lifecycle',
                    'control_class': 'pipette_state_command', 'lifecycle_stage_id': 'constructor_pipette_stage',
                    'lifecycle_attempt_id': 'controlled-' + mode, 'idempotency_key': 'constructor:' + mode}))
        except Exception as exc:
            from fastapi import HTTPException
            assert isinstance(exc, HTTPException)
            result = exc.detail
        for timer in timers: timer.join()
        db = store.connection.execute('PRAGMA database_list').fetchone()[2]
        row = dict(store.connection.execute('SELECT * FROM pipette_operations ORDER BY rowid DESC LIMIT 1').fetchone())
        command_id = row['command_id']
        command = dict(store.connection.execute('SELECT * FROM operator_commands WHERE command_id=?', (command_id,)).fetchone())
        receipt = json.loads(row['receipt_json'])
        # Fresh interpreter, independent SQLite read, no runtime owner initialized.
        script = 'import json,sqlite3,sys; c=sqlite3.connect(sys.argv[1]); c.row_factory=sqlite3.Row; r=dict(c.execute("SELECT * FROM pipette_operations WHERE command_id=?",(sys.argv[2],)).fetchone()); print(json.dumps(r))'
        reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, db, command_id], text=True))
        assert reopened == row
        assert row['lifecycle_attempt_id'] == 'controlled-' + mode
        assert command['status'] == row['status']
        assert command['outcome'] == row['outcome']
        if 'truth' in receipt:
            assert receipt['truth']['physical_effect_verified'] is False
        else:
            # Early native exceptions use the existing failure envelope, not a
            # fabricated finalized PipetteReceipt. Preserve that distinction.
            assert row['status'] == 'failed'
            assert result.get('physical_effect_verified') is not True
        if group.get_status()['last_group_transaction'] is not None:
            assert group.get_status()['last_group_transaction']['outcome'] == result['outcome']
        encoded = json.dumps(result, sort_keys=True)
        if label == 'candidate': assert construction['constructor_transaction_attachments'] == 0
        metrics = {**construction, 'result_bytes': len(encoded.encode()), 'stored_receipt_bytes': len(row['receipt_json'].encode()),
                   'encode_seconds_per_100': timeit.timeit(lambda: json.dumps(result, sort_keys=True), number=100)}
        artifact('constructor-' + mode + '-' + label + '.json', {'result': result, 'receipt': receipt,
            'child_status': row['status'], 'command_status': command['status'], 'wire': wire, 'metrics': metrics,
            'provenance': 'controlled CAN exchange; real driver/router/service/SQLite; not historical wire or live motion'})
        monkeypatch.setattr(module.CanPipetteTransport, '_status_payload', payload)
        runs.append((result, wire, row, command))
        store._audit_database.close(); runtime.close()
    before, after = runs
    expected_ok = mode in {'initial', 'already_initialized', 'conditional_retry', 'delayed_completion',
                           'unknown_offsets', 'partial_offsets'}
    assert before[0]['ok'] is after[0]['ok'] is expected_ok
    assert before[1] == after[1]
    if mode in {'initial', 'partial_offsets', 'unknown_offsets'}:
        evidence = after[0]['pressure_offset_evidence']
        valid_channels = [int(k) for k, v in evidence.items() if v['valid']]
        assert valid_channels == ([0, 1, 2, 3] if mode == 'initial' else [0, 1] if mode == 'partial_offsets' else [])
        for channel in valid_channels:
            assert evidence[channel]['offset'] == 100 + channel
    if mode == 'delayed_completion':
        assert all(x['result']['ok'] and x['result']['immediate_ack_received']
                   for x in after[0]['initial_group']['delayed_completions'])
    if mode == 'completion_failure':
        assert after[0]['initial_group']['delayed_completions'][3]['result']['event_error_code'] == 0x21
    for key in ('ok', 'outcome', 'delivery_verified', 'controller_acknowledged', 'completion_verified',
                'receipt_truth', 'pressure_offsets_valid', 'pressure_offsets', 'pressure_offset_evidence',
                'pressure_epoch', 'pressure_offset_order', 'single_conditional_retry_performed'):
        assert stable(before[0].get(key)) == stable(after[0].get(key)), key
    assert stable(normalize_pipette_command_outcome(before[0])) == stable(normalize_pipette_command_outcome(after[0]))
    assert before[2]['status'] == after[2]['status']
    assert before[2]['outcome'] == after[2]['outcome']
    assert 'status_readback_final' not in after[0]
    assert 'pressure_stream' not in after[0]
    for group_key in ('initial_group', 'retry_group'):
        b, a = before[0].get(group_key), after[0].get(group_key)
        if not b: assert not a; continue
        for key in ('ok', 'outcome', 'cycle', 'delayed_completions', 'pressure_stream', 'completion_verified'):
            assert stable(b.get(key)) == stable(a.get(key)), (group_key, key)
    # The selected pressure attachment is moved, never copied; distinct initial epoch survives retry.
    selected = 'retry_group' if mode in {'conditional_retry', 'retry_after_pressure', 'partial_channel'} else 'initial_group'
    if selected in after[0]:
        assert 'pressure_offset_evidence' not in after[0][selected]
    if mode == 'retry_after_pressure':
        assert after[0]['initial_group']['pressure_epoch']['epoch'] == 1
        assert after[0]['pressure_epoch']['epoch'] == 2


@pytest.mark.parametrize('caller', ['initializeMotion.initial', 'initializeMotion.retry', 'detectFluid'])
def test_shared_group_callers_unchanged(caller):
    baseline = baseline_module('src/bioxp/pipette/transport.py', tm)
    results = []
    for module in (baseline, tm):
        group, wire, timers = transport_rig(module, 'initial')
        result = (group.initiate_group_once_for_oem_detect_fluid() if caller == 'detectFluid' else
                  group.initiate_group_once_for_oem_initialize_motion(cycle=caller))
        results.append((stable(result), wire))
    assert results[0] == results[1]


def test_other_stage_evidence_and_history_are_not_retired():
    from bioxp.lifecycle_state import CanonicalLifecycleOwner
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason='offline no-detail owner')
    # Stage completion never depends on canonical receipt availability.
    owner.run_stage('constructor_pipette_stage', lambda: {'ok': True, 'unique': ['constructor']})
    no_motion = {'ok': True, 'unique': ['motor-current']}
    owner.run_stage('initialization_without_motion', lambda: no_motion)
    no_motion['unique'].append('later-mutated')
    initial = {'ok': True, 'unique': ['initial-check-first']}
    owner.run_stage('initial_check', lambda: initial)
    initial['unique'].append('later-mutated')
    projection = owner.run_stage('initial_check', lambda: {'ok': False, 'error': 'controlled-error', 'unique': ['second']})
    stages = projection['startup']['stages']
    assert stages['constructor_pipette_stage']['evidence'] is None
    assert stages['constructor_pipette_stage']['history'] == []
    assert stages['initialization_without_motion']['evidence']['unique'] == ['motor-current']
    assert stages['initial_check']['history'][0]['evidence']['unique'] == ['initial-check-first']
    assert stages['initial_check']['state'] == 'failed' and stages['initial_check']['error'] == 'controlled-error'


def test_constructor_public_detail_and_lifecycle_owner(query_rig, monkeypatch):
    from bioxp import api
    from bioxp.lifecycle_state import CanonicalLifecycleOwner
    from bioxp.operator_reports import create_operator_reports_router
    from fastapi.testclient import TestClient
    rig = query_rig
    bind_full_infrastructure_identity(monkeypatch)
    group, wire, timers = transport_rig(tm, 'initial')
    monkeypatch.setattr(api, '_pipette_transport', group)
    monkeypatch.setattr(api, '_get_pipette_transport', lambda: group)
    rig[0].include_router(create_operator_reports_router(rig[0].state.operator_receipt_store))
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason='controlled W1 constructor fixture')
    result = {}
    def action():
        attempt = owner.projection()['startup']['stages']['constructor_pipette_stage']['attempt_id']
        result.update(asyncio.run(run_pipette_init_command(PipetteInitCommand(), get_transport=lambda: group,
            run_blocking=inline, receipt_store=rig[5], runtime_binding={
                'entrypoint_id': 'lifecycle.constructor_pipette_stage', 'caller_class': 'lifecycle',
                'control_class': 'pipette_state_command', 'lifecycle_stage_id': 'constructor_pipette_stage',
                'lifecycle_attempt_id': attempt, 'idempotency_key': 'constructor:' + attempt})))
        return result
    copies = []
    import bioxp.lifecycle_state as lm
    original_copy = lm.copy.deepcopy
    def copied(value, *args, **kwargs):
        if (isinstance(value, dict) and 'initial_group' in value
                and inspect.stack()[1].filename.split('@')[0].endswith('/lifecycle_state.py')):
            copies.append(value)
        return original_copy(value, *args, **kwargs)
    monkeypatch.setattr(lm.copy, 'deepcopy', copied)
    projection = owner.run_stage('constructor_pipette_stage', action)
    stage = projection['startup']['stages']['constructor_pipette_stage']
    assert stage['state'] == 'passed' and stage['completed_at'] and stage['attempt_id']
    assert stage['evidence'] is None and stage['history'] == []
    assert copies == []
    row = rig[5].connection.execute('SELECT * FROM pipette_operations WHERE lifecycle_attempt_id=?',
        (stage['attempt_id'],)).fetchone()
    assert row['command_id'] == result['command_id']
    client = TestClient(rig[0])
    public = {'liquid_status': asyncio.run(api.liquid_status())}
    for name, url in {
        'history': '/operator/actions/history',
        'detail': '/operator/actions/receipts/' + row['command_id'] + '?detail=true',
        'detail_v2': '/operator/v2/actions/receipts/' + row['command_id'] + '?detail=true',
        'report_detail': '/operator/reports/pipette/' + row['pipette_operation_id'],
    }.items():
        response = client.get(url)
        assert response.status_code == 200, (name, response.text)
        public[name] = response.json()
    receipt = json.loads(row['receipt_json'])
    assert public['liquid_status']['latest_receipt'] == receipt
    assert receipt['runtime_binding']['lifecycle_attempt_id'] == stage['attempt_id']
    assert receipt['truth']['completion_verified'] is True
    prior = list(wire)
    assert rig[5].replay_result(command_id=row['command_id'], pipette_operation_id=row['pipette_operation_id'])['replayed']
    assert wire == prior
    artifact('public-constructor.json', public)
    candidate_copies = len(copies)
    old = baseline_module('src/bioxp/lifecycle_state.py', lm).CanonicalLifecycleOwner()
    old.transport_changed(True, reason='controlled baseline copy measurement')
    old.run_stage('constructor_pipette_stage', lambda: result)
    assert len(copies) >= 2  # baseline finish retention plus its full projection
    artifact('lifecycle-construction.json', {'constructor_detail_deepcopies': candidate_copies,
        'baseline_constructor_detail_deepcopies': len(copies), 'stage': stage,
        'canonical_command_id': row['command_id'], 'canonical_pipette_operation_id': row['pipette_operation_id']})
