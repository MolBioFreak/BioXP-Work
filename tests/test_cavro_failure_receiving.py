"""D3/D4: actual bounded transport evidence through finite receipt owners.

Only physical leaves are inert. Existing offline guard and store isolation apply.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest
from fastapi import HTTPException
from bioxp.critical_logging import critical_receipt
from bioxp.novo_router import NovoFrame, PRESSURE_STREAM_IDS
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.protocols.operational_results import compact_application
from tests.test_cavro_application import rig, request
from tests.test_cavro_transport_repairs import receiver, compile_command
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action
from tests.test_protocol_workflow_connected import await_job


def export(name, value):
    target = Path(os.environ['NON_OEM_NATIVE_EXPORT_DIR'])
    target.mkdir(parents=True, exist_ok=True)
    (target / (name + '.json')).write_text(json.dumps(value, indent=2, default=str))


def fresh_read(root, session=None):
    code = '''import tests.z_stop_offline_guard
import json,sys,os
os.environ["BIOXP_OEM_RUNTIME_STATE_ROOT"]=sys.argv[1]
from bioxp.pipette.receipts import PipetteReceiptStore
s=PipetteReceiptStore(sys.argv[1])
rows=[dict(r) for r in s.connection.execute("SELECT command_id,pipette_operation_id,status,receipt_json FROM pipette_operations ORDER BY command_id")]
print(json.dumps({'rows':rows,'capture':s.scientific_pressure_capture(sys.argv[2]) if sys.argv[2] else None}))
'''
    return json.loads(subprocess.check_output([sys.executable, '-c', code, str(root), session or ''], text=True))


def failed_rows(store):
    return [dict(row) for row in store.connection.execute(
        "SELECT command_id,pipette_operation_id,status,receipt_json FROM pipette_operations WHERE status='failed' ORDER BY command_id")]


def inject_interrupt(rig):
    def inject(_):
        data = b'1\x00\x08'
        rig.router._dispatch(NovoFrame(min(PRESSURE_STREAM_IDS), len(data), data, b'', time.monotonic(), 'pressure'))
        rig.group._interrupt_epoch += 1
    rig.group._sleep = inject


def test_review_multichannel_partial_preserved(receiver, rig):
    receiver.fault['missing'] = 1
    compiled = compile_command({'command': 'plunger_aspirate', 'value': 2, 'unit': 'ul'})
    original = {}
    def send(_):
        try:
            return receiver.group.run_cavro_command(compiled, channels=[0, 1], timeout_ms=5)
        except Exception as exc:
            original['exception'] = exc
            original['partial'] = critical_receipt(exc.pipette_partial_result)
            raise
    with pytest.raises(HTTPException) as caught:
        rig.provider._manual_pipette_receipt_runner('device_command', send,
            'offline-finite-owner', {'source_identity': 'review-partial'}, compiled['requested'])
    rows = failed_rows(rig.store)
    export('review-partial', {'detail': caught.value.detail, 'rows': rows, 'wire': receiver.wire})
    assert caught.value.__cause__ is original['exception']
    partial = caught.value.detail['pipette_partial_result']
    assert partial == original['partial'] == caught.value.pipette_partial_result
    assert partial['failed_channel'] == 1
    assert [r['channel'] for r in partial['channels']] == [0]
    retained = json.loads(rows[0]['receipt_json'])
    for field in ('channels', 'failed_channel', 'failure_details'):
        assert retained[field] == partial[field]
    assert len(receiver.wire) == 2  # No retries or cleanup.
    assert fresh_read(rig.root)['rows'] == rows


@pytest.mark.parametrize('failure', [None, 'chunks', 'streams', 'terminal', 'receipt'])
def test_review_pressure_interrupt_primary_and_retention(rig, monkeypatch, failure):
    inject_interrupt(rig)
    if failure:
        target = rig.store if failure == 'receipt' else rig.store._audit_database
        name = {'chunks': 'record_pressure_chunk', 'streams': 'record_pressure_stream',
                'terminal': 'finish_pressure_stream', 'receipt': 'record_failure'}[failure]
        def fail(*args, **kwargs):
            raise OSError('injected retention failure')
        monkeypatch.setattr(target, name, fail)
    original = {}
    def capture(_):
        try:
            return rig.group.capture_pressure({'channels': [0], 'max_frames': 2, 'duration_ms': 100})
        except Exception as exc:
            original['exception'] = exc
            original['partial'] = exc.pipette_partial_result
            raise
    with pytest.raises(HTTPException) as caught:
        rig.provider._manual_pipette_receipt_runner('pressure_capture', capture,
            'offline-finite-owner', {'source_identity': 'review-pressure-interrupt'}, {})
    export('review-pressure-' + str(failure), {'detail': caught.value.detail, 'rows': failed_rows(rig.store)})
    assert caught.value.__cause__ is original['exception']
    assert 'Pressure capture interrupted' in caught.value.detail['message']
    assert original['partial']['channels'][0]['samples'][0]['raw_pressure'] == 8
    partial = caught.value.pipette_partial_result
    assert partial['interrupted'] is True
    assert not rig.wire and not rig.native
    if failure == 'receipt':
        assert 'observation_error' in partial
        assert caught.value.detail['error'] == original['exception'].code
        return
    retained = partial['scientific_pressure']
    if failure == 'streams':
        assert 'injected retention failure' in retained['observation_error']
        assert 'stream_session_id' not in retained
        assert fresh_read(rig.root)['rows'] == failed_rows(rig.store)
        return
    session = retained['stream_session_id']
    readback = rig.store.scientific_pressure_capture(session)
    assert readback['stream']['terminal_state'] == (
        'running' if failure == 'terminal' else 'failed' if failure else 'interrupted')
    if failure == 'terminal':
        assert 'injected retention failure' in retained['observation_error']
        assert 'terminal_state' not in retained  # Never claim the failed write landed.
        assert readback['chunks'][0]['scientific_result']['samples'][0]['raw_pressure'] == 8
    elif failure:
        assert 'injected retention failure' in retained['observation_error']
        assert not readback['chunks']
    else:
        assert readback['chunks'][0]['scientific_result']['samples'][0]['raw_pressure'] == 8
        assert 'samples' not in json.dumps(failed_rows(rig.store))
    cold = fresh_read(rig.root, session)
    assert cold['capture'] == readback
    assert cold['rows'] == failed_rows(rig.store)
    export('review-pressure-readback-' + str(failure), cold)


@pytest.mark.parametrize('error_type', [RuntimeError, ValueError, TimeoutError])
def test_completed_capture_followed_by_failure_is_retained_as_failed(rig, error_type):
    original = error_type('failure after bounded acquisition')
    def call(_):
        acquired = rig.group.capture_pressure({'channels': [0], 'duration_ms': 1, 'max_frames': 1})
        original.pipette_partial_result = acquired
        raise original
    with pytest.raises(HTTPException) as caught:
        rig.provider._manual_pipette_receipt_runner('pressure_capture', call,
            'offline-finite-owner', {'source_identity': 'failed-after-acquisition'}, {})
    assert caught.value.__cause__ is original
    session = caught.value.pipette_partial_result['scientific_pressure']['stream_session_id']
    cold = fresh_read(rig.root, session)
    assert cold['capture']['stream']['terminal_state'] == 'failed'
    assert cold['rows'][0]['status'] == ('reconciliation_required' if error_type is TimeoutError else 'failed')
    assert caught.value.status_code == {RuntimeError: 503, ValueError: 502, TimeoutError: 504}[error_type]
    assert not rig.wire and not rig.native
    export('failed-capture-' + error_type.__name__, cold)


def test_capture_retention_error_does_not_change_completed_acquisition(rig, monkeypatch):
    def fail(**kwargs):
        raise OSError('completed capture retention failure')
    monkeypatch.setattr(rig.store._audit_database, 'record_pressure_chunk', fail)
    result = rig.provider._manual_pipette_receipt_runner('pressure_capture',
        lambda _: rig.group.capture_pressure({'channels': [0], 'duration_ms': 1, 'max_frames': 1}),
        'offline-finite-owner', {'source_identity': 'completed-capture-retention-failed'}, {})
    assert result['ok'] is True and result['source_return_completed'] is True
    retained = result['scientific_pressure']
    assert 'completed capture retention failure' in retained['observation_error']
    cold = fresh_read(rig.root, retained['stream_session_id'])
    assert cold['capture']['stream']['terminal_state'] == 'failed'
    assert not rig.wire and not rig.native
    export('completed-acquisition-retention-error', {'result': result, 'fresh': cold})


@pytest.mark.parametrize('kind', ['pressure', 'pressure-retention-error', 'device'])
def test_public_instruction_real_finite_runner_sqlite_compaction_fresh(
        rig, receiver, installed_retained, monkeypatch, tmp_path, kind):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    root = installed_retained[4]
    store = PipetteReceiptStore(root)
    from bioxp import api
    monkeypatch.setattr(api, "_pipette_receipts", store)
    if kind == 'device':
        receiver.fault['missing'] = 1
        provider.primitives.pipette_transport = receiver.group
        op = {'operation': 'device_command', 'channels': [0, 1], 'timeout_ms': 5,
              'command': {'command': 'plunger_aspirate', 'value': 2, 'unit': 'ul'}}
    else:
        provider.primitives.pipette_transport = rig.group
        inject_interrupt(rig)
        op = {'operation': 'pressure_capture', 'capture': {'channels': [0, 1], 'max_frames': 2, 'duration_ms': 100}}
        if kind == 'pressure-retention-error':
            from bioxp.runtime_audit_store import RuntimeAuditDatabase
            original = RuntimeAuditDatabase.record_pressure_chunk
            def fail_second(self, **kwargs):
                if kwargs['channel'] == 1:
                    raise OSError('second channel retention failure')
                return original(self, **kwargs)
            monkeypatch.setattr(RuntimeAuditDatabase, 'record_pressure_chunk', fail_second)
    before = {r['command_id'] for r in store.connection.execute('SELECT command_id FROM pipette_operations')}
    job = submit(app, client, [action('pipette_manual_physical', {
        'operation': 'cavro_application', 'application': request(op)})], key='failure-receiving-' + kind)
    done = await_job(client, job, lambda r: r['command']['terminal'])
    rows = [row for row in failed_rows(store) if row['command_id'] not in before]
    export('finite-first-' + kind, {'job': done, 'rows': rows})
    assert len(rows) == 1
    result = done['execution']['runtime_state']['action_results'][0]['pipette_result']
    assert result['ok'] is False
    compact = compact_application(result)
    partial = compact['events'][0]['partial']['pipette_partial_result']
    retained = json.loads(rows[0]['receipt_json'])
    if kind == 'device':
        assert partial['failed_channel'] == retained['failed_channel'] == 1
        assert partial['channels'][0]['channel'] == 0
        assert partial['failure_details'] == retained['failure_details']
        assert len(receiver.wire) == 2
        session = None
    else:
        assert partial['interrupted'] is True
        session = partial['scientific_pressure']['stream_session_id']
        assert session == retained['scientific_pressure']['stream_session_id']
        readback = store.scientific_pressure_capture(session)
        assert readback['stream']['terminal_state'] == ('failed' if kind.endswith('error') else 'interrupted')
        assert readback['chunks'][0]['scientific_result']['samples'][0]['raw_pressure'] == 8
        assert readback['chunks'][0]['scientific_result']['scientific_result']['reader_generation'] == retained['reader_generation']
        assert readback['stream']['command_id'] == rows[0]['command_id']
        assert readback['stream']['pipette_operation_id'] == rows[0]['pipette_operation_id']
        if kind.endswith('error'):
            assert len(readback['chunks']) == 1
            assert 'second channel retention failure' in partial['scientific_pressure']['observation_error']
        assert not rig.wire and not rig.native
        assert 'samples' not in json.dumps(compact)
    cold = fresh_read(root, session)
    assert [row for row in cold['rows'] if row['command_id'] not in before] == rows
    if session:
        assert cold['capture'] == readback
    export('finite-final-' + kind, {'job': done, 'compact': compact, 'fresh': cold})
