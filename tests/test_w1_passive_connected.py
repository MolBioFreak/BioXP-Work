"""W1b actual passive producer/service/public readers and constructor-cost qualification."""
from tests.test_w1_diagnostics_connected import (
    asyncio, copy, inspect, json, sqlite3, subprocess, sys, pytest, rm,
    baseline_module, bind_full_infrastructure_identity, artifact, stable, database_artifact,
)
from tests.test_deck_tip_query_publication import query_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig

@pytest.mark.parametrize('mode', ['eligible', 'eligible_true', 'failed_channel', 'missing_all', 'unknown_channel', 'semantic_unverified', 'absent_source', 'explicit'])
def test_passive_actual_service_record_cost_and_exports(query_rig, monkeypatch, mode):
    from bioxp import api
    rig = query_rig
    bind_full_infrastructure_identity(monkeypatch)
    store = rig[5]
    from bioxp.operator_reports import create_operator_reports_router
    rig[0].include_router(create_operator_reports_router(rig[0].state.operator_receipt_store))
    original_record = rm.PipetteReceiptStore.record
    baseline = baseline_module('src/bioxp/pipette/receipts.py', rm)
    baseline.PipetteReceiptStore.record.__globals__['current_release_identity'] = rm.current_release_identity
    exports = []
    if mode == 'failed_channel':
        driver = rig[8]._transports[2]._get_driver()
        exchange = driver._send_pipette_command
        def failed_exchange(*args, **kwargs):
            result = exchange(*args, **kwargs)
            return {**result, 'ack': {**result['ack'], 'received': False}}
        monkeypatch.setattr(driver, '_send_pipette_command', failed_exchange)
    elif mode == 'unknown_channel': rig[7]['data'][2] = [32, 96, 50]
    elif mode == 'eligible_true': rig[7]['data'] = [[32, 96, 49] for _ in range(4)]
    for label, record in [('baseline', baseline.PipetteReceiptStore.record), ('candidate', original_record)]:
        counts = {'full_redact': 0, 'full_critical': 0, 'release_calls': 0}
        globals_ = record.__globals__
        redact, critical, release = globals_['_redact'], globals_['critical_receipt'], globals_['current_release_identity']
        def full(value): return isinstance(value, dict) and 'channels' in value and 'ok' in value
        def redacted(value):
            if full(value): counts['full_redact'] += 1
            return redact(value)
        def copied(value):
            if full(value): counts['full_critical'] += 1
            return critical(value)
        def identity():
            # Count only record's direct discarded field or retained source call.
            if any(f.function == 'record' for f in inspect.stack()): counts['release_calls'] += 1
            return release()
        monkeypatch.setitem(globals_, '_redact', redacted)
        monkeypatch.setitem(globals_, 'critical_receipt', copied)
        # Source identity uses the installed owner's globals, including baseline record.
        monkeypatch.setattr(rm, 'current_release_identity', identity)
        monkeypatch.setitem(globals_, 'current_release_identity', identity)
        monkeypatch.setattr(rm.PipetteReceiptStore, 'record', record)
        # All variants originate at the actual service. Only two representation
        # boundary controls alter the freshly produced input to record.
        def boundary(self, **kwargs):
            if mode in {'semantic_unverified', 'absent_source'}:
                kwargs['result'] = dict(kwargs['result'])
                if mode == 'semantic_unverified': kwargs['result']['semantic_query_response_verified'] = False
                else: kwargs['result'].pop('collection_source', None)
            return record(self, **kwargs)
        monkeypatch.setattr(rm.PipetteReceiptStore, 'record', boundary)
        rig[7]['missing'] = mode == 'missing_all'
        binding = {'caller_class': 'lifecycle', 'idempotency_key': mode + '-' + label,
                   'entrypoint_id': 'liquid.tip_status' if mode == 'explicit' else 'hardware.snapshot.park_tip_observation'}
        prior = len(rig[6])
        try: result = api._query_and_publish_pipette_tip_status(runtime_binding=binding, readiness_bounded=mode != 'explicit')
        except Exception as exc:
            from fastapi import HTTPException
            assert isinstance(exc, HTTPException); result = exc.detail
        receipt = store.latest()
        assert len(rig[6]) - prior == 4, result
        artifact('passive-' + mode + '-' + label + '.json', {'receipt': receipt, 'result': result, 'counts': counts,
            'provenance': 'actual API query/service/SQLite envelope; controlled exchange; synthetic infrastructure identity'})
        assert ('deployment_identity' not in receipt) is (mode in {'eligible', 'eligible_true'})
        if mode != 'absent_source':
            assert receipt['result']['collection_source'] == result['collection_source']
        if label == 'candidate':
            from fastapi.testclient import TestClient
            client = TestClient(rig[0])
            child = store.connection.execute('SELECT * FROM pipette_operations ORDER BY rowid DESC LIMIT 1').fetchone()
            public = {'liquid_status': asyncio.run(api.liquid_status())}
            for name, url in {
                'history': '/operator/actions/history',
                'detail': '/operator/actions/receipts/' + child['command_id'] + '?detail=true',
                'detail_v2': '/operator/v2/actions/receipts/' + child['command_id'] + '?detail=true',
                'report_detail': '/operator/reports/pipette/' + child['pipette_operation_id'],
            }.items():
                response = client.get(url)
                assert response.status_code == 200, (name, response.text)
                public[name] = response.json()
            assert public['liquid_status']['latest_receipt'] == receipt
            artifact('public-passive-' + mode + '.json', public)
            # Reopen the exact receipt in a fresh interpreter without owner creation.
            db = store.connection.execute('PRAGMA database_list').fetchone()[2]
            script = 'import json,sqlite3,sys; c=sqlite3.connect(sys.argv[1]); print(c.execute("SELECT receipt_json FROM pipette_operations WHERE pipette_operation_id=?",(sys.argv[2],)).fetchone()[0])'
            reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, db, child['pipette_operation_id']], text=True))
            assert reopened == receipt
            database_artifact('passive-' + mode + '.db', db)
            if mode in {'eligible', 'eligible_true', 'explicit'}:
                replay = store.replay_result(command_id=child['command_id'], pipette_operation_id=child['pipette_operation_id'])
                assert replay['replayed'] and replay['ok']
                assert replay['collection_source'] == result['collection_source']
                assert len(rig[6]) - prior == 4
        exports.append((receipt, counts))
        monkeypatch.setitem(globals_, '_redact', redact)
        monkeypatch.setitem(globals_, 'critical_receipt', critical)
        monkeypatch.setitem(globals_, 'current_release_identity', release)
        monkeypatch.setattr(rm, 'current_release_identity', release)
    before, after = exports
    def comparable(receipt):
        result = copy.deepcopy(receipt['result'])
        if isinstance(result.get('collection_source'), dict):
            # Each query creates a genuine new source revision. Compare flags,
            # and verify the exact identity against its own actual producer below.
            result['collection_source'].pop('identity', None)
        return stable(result)
    assert comparable(before[0]) == comparable(after[0])
    assert before[0]['source_identity'] == after[0]['source_identity']
    assert before[0]['truth'] == after[0]['truth']
    if mode in {'eligible', 'eligible_true'}:
        assert before[1] == {'full_redact': 1, 'full_critical': 1, 'release_calls': 2}
        assert after[1] == {'full_redact': 0, 'full_critical': 0, 'release_calls': 1}
        state = store.collection_state(identity=rig[8].collection_source_identity(), ownership_generation=rig[1].generation_provider())
        assert state['tip_exists'] is (mode == 'eligible_true')
    else: assert before[1] == after[1]