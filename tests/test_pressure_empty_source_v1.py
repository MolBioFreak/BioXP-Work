"""Bounded real source/transport/service/store qualification; CAN replies only doubled."""
import asyncio
import json
import subprocess
import sys
from types import SimpleNamespace

import pytest
from fastapi import HTTPException
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptError
from bioxp.protocols.runtime_state import ProtocolSourceModel
from bioxp.services.pipette_service import _OemPipetteBody, _OemLifecycleContext, run_pipette_operation
from tests.test_deck_tip_query_publication import query_rig, query
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_pipette_collection_owner import async_set


def issue(rig, operation, key='pressure-source'):
    async def run(label, call, **kwargs):
        return call()
    return asyncio.run(run_pipette_operation('read_pressure', operation,
        get_transport=lambda: rig[8], run_blocking=run, receipt_store=rig[5],
        runtime_binding={'idempotency_key': key, 'caller_class': 'lifecycle'}))


def predicate():
    return api._pipette_collection_state()['tip_exists']


def source(t):
    return t.read_pressure_for_oem_source(predicate)


def baseline(rig):
    state = SimpleNamespace(source_model=ProtocolSourceModel())
    action = _OemLifecycleContext('empty-pressure', params={'arguments': ()})
    body = _OemPipetteBody(action, state,
        SimpleNamespace(tip_exists=lambda action, state: predicate()),
        lambda name, call, action, state, step: issue(rig, call),
        lambda *args: None, {})
    body.pressure_base()
    return body.result(), state.source_model


def fresh_receipt(root, receipt_id):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    from bioxp.pipette.receipts import PipetteReceiptStore
    with pytest.MonkeyPatch.context() as mp:
        bind_serial206_oem_snapshot(mp)
        bind_collection_test_identity(mp)
        store = PipetteReceiptStore(root)
        row = store.connection.execute('SELECT status,receipt_json FROM pipette_operations WHERE receipt_json LIKE ?',
            ('%' + receipt_id + '%',)).fetchone()
        return {'status': row['status'], 'receipt': json.loads(row['receipt_json'])}


def test_source_baseline_default_is_not_measured_and_no_native_query(query_rig):
    rig = query_rig
    query(rig)
    before = list(rig[6])
    result, model = baseline(rig)
    assert result['ok'] is True
    assert model.pressure_baseline == [0.0] * 4
    pressure = result['native_results'][0]['result']
    assert pressure['source_noop'] is True and pressure['source_return_completed'] is True
    assert pressure['channels'] == [] and pressure['channel_count'] == 0
    assert pressure['hardware_truth_level'] == 'source_model_default'
    assert pressure['semantic_query_response_verified'] is False
    assert not any(pressure['receipt_truth'][k] for k in (
        'delivery_verified', 'controller_acknowledged', 'completion_verified',
        'hardware_postcondition_verified', 'physical_effect_verified'))
    assert rig[6] == before
    script = ('import json,sys;from tests.test_pressure_empty_source_v1 import fresh_receipt;'
              'print(json.dumps(fresh_receipt(sys.argv[1],sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script,
        str(rig[4]), pressure['receipt_id']], text=True, timeout=20))
    assert reopened['receipt']['result']['source_noop'] is True
    assert reopened['receipt']['result']['channels'] == []
    assert reopened['status'] == rig[5].connection.execute(
        'SELECT status FROM pipette_operations WHERE command_id=?', (pressure['command_id'],)).fetchone()[0]


def test_source_return_canonical_terminal_acceptance(query_rig):
    rig = query_rig
    query(rig)
    pressure = issue(rig, source)
    assert pressure['ok'] is True and pressure['source_return_completed'] is True
    row = rig[5].connection.execute('SELECT status FROM pipette_operations WHERE command_id=?',
        (pressure['command_id'],)).fetchone()
    # Acceptance gate is deliberately not xfailed: current receipt owner blocks closure.
    assert row['status'] == 'completed'


def test_manual_empty_query_policy_unchanged(query_rig):
    query(query_rig)
    before = len(query_rig[6])
    result = issue(query_rig, lambda t: t.read_pressure(), 'manual-empty')
    assert result['ok'] is False and result['channels'] == []
    assert len(query_rig[6]) == before
    assert not result.get('source_noop')


@pytest.mark.parametrize('fault', ['unknown', 'stale', 'owner', 'reader', 'interrupt', 'uncommitted'])
def test_empty_source_requires_current_committed_owner(query_rig, fault):
    rig = query_rig
    if fault == 'unknown':
        rig[7]['data'][0] = [32, 96, 50]
        with pytest.raises(AssertionError):
            query(rig)
    elif fault != 'uncommitted':
        query(rig)
    if fault == 'stale': async_set(rig, 0, 48)
    if fault == 'owner': rig[8]._collection_source_owner = 'changed-owner'
    if fault == 'reader': rig[8]._transports[0]._driver.bus.router.reader_generation += 1
    if fault == 'interrupt': rig[8]._interrupt_epoch += 1
    calls = list(rig[6])
    with pytest.raises(HTTPException):
        issue(rig, source)
    assert rig[6] == calls


def test_owner_changes_during_predicate_read_refused(query_rig):
    rig = query_rig
    query(rig)
    def changed():
        result = predicate()
        rig[8]._interrupt_epoch += 1
        return result
    with pytest.raises(RuntimeError, match='changed_during_pressure'):
        rig[8].read_pressure_for_oem_source(changed)


@pytest.mark.parametrize('payload', [[32,96,49,50,51], [32,96,88]])
def test_loaded_query_uses_real_unchanged_pressure_parser(query_rig, monkeypatch, payload):
    rig = query_rig
    rig[7]['data'][0] = [32,96,49]
    query(rig)
    pressure_calls = []
    for channel, transport in enumerate(rig[8]._transports):
        driver = transport._driver
        original = driver._send_pipette_command
        def exchange(command, *, address, ack_mode, command_name, channel=channel, original=original):
            if command == '?31':
                return original(command, address=address, ack_mode=ack_mode, command_name=command_name)
            assert (command, address, ack_mode, command_name) == ('?57','report','query','query_pressure')
            pressure_calls.append(channel)
            return {'ok': True, 'query_response_correlated': True,
                    'ack': {'received': True, 'data': payload}, 'provenance': {}}
        monkeypatch.setattr(driver, '_send_pipette_command', exchange)
    if payload[-1] == 88:
        result = issue(rig, source)
        assert result['ok'] is False
        assert result['channels'][0]['result']['pressure'] is None
        assert result['semantic_query_response_verified'] is False
        assert not result.get('source_noop')
    else:
        result = issue(rig, source)
        assert result['ok'] is True and result['channels'][0]['result']['pressure'] == 123.0
        assert result['semantic_query_response_verified'] is True
        assert not result.get('source_noop')
    assert pressure_calls == [0]
