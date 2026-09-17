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


def issue(rig, operation, key='pressure-source', operation_name='read_pressure'):
    async def run(label, call, **kwargs):
        return call()
    return asyncio.run(run_pipette_operation(operation_name, operation,
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
        row = store.connection.execute('SELECT * FROM pipette_operations WHERE receipt_json LIKE ?',
            ('%' + receipt_id + '%',)).fetchone()
        from bioxp.operator_receipt_store import OperatorReceiptStore
        from bioxp.hardware_status import hardware_state
        mp.setattr(hardware_state, '_epoch', row['ownership_generation'])
        operator = OperatorReceiptStore(root)
        return {'status': row['status'], 'receipt': json.loads(row['receipt_json']),
            'compact': operator.by_command(row['command_id'], include_evidence=False),
            'detailed': operator.by_command(row['command_id'], include_evidence=True),
            'replay': store.replay_result(command_id=row['command_id'], pipette_operation_id=row['pipette_operation_id'])}


def test_source_baseline_default_is_not_measured_and_no_native_query(query_rig):
    rig = query_rig
    query(rig)
    before = list(rig[6])
    sample_tables = ('pipette_channel_observations', 'pipette_transport_exchanges',
                     'pipette_pressure_streams', 'pipette_pressure_chunks')
    sample_counts = {table: rig[5].connection.execute('SELECT count(*) FROM ' + table).fetchone()[0]
                     for table in sample_tables}
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
    assert sample_counts == {table: rig[5].connection.execute('SELECT count(*) FROM ' + table).fetchone()[0]
                             for table in sample_tables}
    script = ('import json,sys;from tests.test_pressure_empty_source_v1 import fresh_receipt;'
              'print(json.dumps(fresh_receipt(sys.argv[1],sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script,
        str(rig[4]), pressure['receipt_id']], text=True, timeout=20))
    assert reopened['receipt']['result']['source_noop'] is True
    assert reopened['receipt']['result']['channels'] == []
    assert reopened['status'] == 'completed'
    for view in ('compact', 'detailed', 'replay'):
        assert reopened[view]['status'] == 'completed', reopened[view]
        assert reopened[view]['outcome'] == 'completed'
        assert not reopened[view].get('physical_effect_verified')
    assert reopened['detailed']['response']['source_return_kind'] == 'oem_empty_pressure_source_return'
    replay = issue(rig, source)
    assert replay['replayed'] is True and replay['status'] == 'completed' and replay['ok'] is True
    assert rig[6] == before
    from fastapi.testclient import TestClient
    client = TestClient(rig[0])
    for suffix in ('', '?detail=true'):
        response = client.get('/operator/v2/actions/receipts/' + pressure['command_id'] + suffix)
        assert response.status_code == 200, response.text
        view = response.json()
        assert view['status'] == 'completed' and view['terminal'] is True
        assert view['physical_effect_verified'] is False


def test_source_return_canonical_terminal_acceptance(query_rig):
    rig = query_rig
    query(rig)
    pressure = issue(rig, source)
    assert pressure['ok'] is True and pressure['source_return_completed'] is True
    row = rig[5].connection.execute('SELECT status FROM pipette_operations WHERE command_id=?',
        (pressure['command_id'],)).fetchone()
    assert row['status'] == 'completed'


def test_manual_empty_query_policy_unchanged(query_rig):
    query(query_rig)
    before = len(query_rig[6])
    result = issue(query_rig, lambda t: t.read_pressure(), 'manual-empty')
    assert result['ok'] is False and result['channels'] == []
    assert len(query_rig[6]) == before
    assert not result.get('source_noop')
    assert query_rig[5].connection.execute('SELECT status FROM pipette_operations WHERE command_id=?',
        (result['command_id'],)).fetchone()[0] == 'failed'


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
            assert not rig[8]._transaction_lock._is_owned(), 'loaded query inherited source-predicate lock'
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
        assert rig[5].connection.execute('SELECT status FROM pipette_operations WHERE command_id=?',
            (result['command_id'],)).fetchone()[0] == 'failed'
    else:
        result = issue(rig, source)
        assert result['ok'] is True and result['channels'][0]['result']['pressure'] == 123.0
        assert result['semantic_query_response_verified'] is True
        assert not result.get('source_noop')
    assert pressure_calls == [0]


@pytest.mark.parametrize('field,value', [
    ('source_return_kind', None), ('source_return_kind', 'unknown'),
    ('source_return_completed', False), ('source_noop', False), ('source_noop', 1),
    ('ok', False), ('channels', None), ('channel_count', 1), ('channel_count', False),
    ('delivery_attempted', True), ('hardware_query_verified', True),
    ('hardware_truth_level', 'unknown'), ('source_return', [0.0]),
])
def test_corrupted_real_source_return_does_not_complete(query_rig, field, value):
    rig = query_rig
    query(rig)
    before = list(rig[6])
    def corrupted(t):
        result = source(t)  # Real native method returns before fault injection.
        result[field] = value
        return result
    result = issue(rig, corrupted)
    row = rig[5].connection.execute('SELECT status FROM pipette_operations WHERE command_id=?',
        (result['command_id'],)).fetchone()
    assert row['status'] == 'failed'
    assert rig[6] == before


def test_source_marker_cannot_complete_another_query_operation(query_rig):
    query(query_rig)
    result = issue(query_rig, source, operation_name='query_pressure')
    row = query_rig[5].connection.execute('SELECT status FROM pipette_operations WHERE command_id=?',
        (result['command_id'],)).fetchone()
    assert row['status'] == 'failed'
