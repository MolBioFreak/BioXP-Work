"""Additional direct source/proof and readback claim consumers."""
import json
import sqlite3
import pytest
from fastapi.testclient import TestClient
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptError
from tests.test_pipette_collection_owner import state
from tests.test_deck_tip_query_publication import query_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_tip_query_publication_contradiction import warm_no_tip, invoke


@pytest.mark.parametrize('fault', ['channel','transaction','receive_id','duplicate_transaction','future_tx'])
def test_collection_absence_keeps_source_proof_guards(query_rig, monkeypatch, fault):
    rig = query_rig
    warm_no_tip(rig)
    for channel, transport in enumerate(rig[8]._transports):
        driver = transport._driver
        original = driver._send_pipette_command
        def exchange(*args, _original=original, _channel=channel, **kwargs):
            result = _original(*args, **kwargs)
            if fault == 'duplicate_transaction':
                result['provenance']['transaction_id'] = 'same-transaction-for-distinct-channels'
            if _channel == 0:
                if fault == 'channel': result['provenance']['channel'] = 3
                if fault == 'transaction': result['provenance']['transaction_id'] = ''
                if fault == 'receive_id': result['ack']['arbitration_id'] = 0x506 + 8
                if fault == 'future_tx': result['provenance']['tx_timestamp'] += 100
            return result
        monkeypatch.setattr(driver,'_send_pipette_command',exchange)
    response = invoke(rig,'bad-proof-'+fault)
    assert response.status_code in (200,502), response.text
    assert state(rig)['tip_exists'] is None, 'invalid current proof must not become verified absence'


@pytest.mark.parametrize('record_failure', [False,True])
def test_real_live_readback_tip_setter_owns_its_claim(query_rig, monkeypatch, record_failure):
    rig = query_rig
    warm_no_tip(rig)
    rig[0].add_api_route('/liquid/readback',api.liquid_readback,methods=['POST'])
    rig[7]['data'][0] = [32,96,49]
    for transport in rig[8]._transports:
        driver = transport._driver
        monkeypatch.setattr(driver,'query_firmware',lambda *a,**k: {'ok':True,'semantic_ok':True})
        monkeypatch.setattr(driver,'query_status',lambda *a,**k: {'ok':True,'semantic_ok':True})
        monkeypatch.setattr(driver,'query_pressure',lambda *a,**k: {'ok':True,'semantic_ok':True})
    denied = []
    def authorizer(action, table, column, db, trigger):
        if record_failure and action == sqlite3.SQLITE_UPDATE and table == 'pipette_operations' and column == 'receipt_json':
            denied.append(column)
            return sqlite3.SQLITE_DENY
        return sqlite3.SQLITE_OK
    rig[5].connection.set_authorizer(authorizer)
    try:
        response = TestClient(rig[0]).post('/liquid/readback',json={'include_data':False},
            headers={'Idempotency-Key':'direct-readback-owner'})
    finally:
        rig[5].connection.set_authorizer(None)
    assert response.status_code == (503 if record_failure else 200), response.text
    row = rig[5].connection.execute('SELECT * FROM pipette_operations ORDER BY rowid DESC LIMIT 1').fetchone()
    assert row['operation'] == 'live_readback'
    assert json.loads(row['source_identity_json'])['collection_source_affecting'] is True
    if record_failure:
        assert denied
        with pytest.raises(PipetteReceiptError,match='pending'): state(rig)
        result = api._collect_and_publish_hardware_snapshot(['axes','latch'],reason='readback-record-failure')
        assert result['pipette_collection']['available'] is False
    else:
        assert state(rig)['tip_exists'] is True
        assert state(rig)['command_id'] == row['command_id']


def test_unrelated_readonly_claim_does_not_replace_collection_owner(query_rig):
    import asyncio
    from bioxp.services.pipette_service import run_pipette_operation
    rig = query_rig
    warm_no_tip(rig)
    before = state(rig)
    async def runner(label, call, **kwargs): return call()
    # Genuine source get_status is a passive projection, not a tip setter.
    asyncio.run(run_pipette_operation('status',lambda t:t.get_status(),get_transport=lambda:rig[8],
        run_blocking=runner,receipt_store=rig[5],requested_inputs={},
        runtime_binding={'entrypoint_id':'test.passive_status','idempotency_key':'unrelated-passive-status'}))
    assert state(rig) == before
    plan = rig[5].connection.execute("EXPLAIN QUERY PLAN SELECT * FROM pipette_operations WHERE "
        "json_extract(source_identity_json, '$.collection_source_affecting')=1 ORDER BY rowid DESC LIMIT 1").fetchall()
    assert any('pipette_collection_claim_idx' in row[3] for row in plan)
    assert not any('TEMP B-TREE' in row[3] for row in plan)


def test_linked_receipt_id_is_not_committed_collection_authority(query_rig):
    import asyncio
    from bioxp import operator_controls
    from bioxp.services.pipette_service import run_pipette_operation
    rig = query_rig
    warm_no_tip(rig)
    owner = rig[0].state.operator_receipt_store
    generation = rig[1].generation_provider()
    parent, created = owner.claim({'command_id':'linked-collection-parent',
        'idempotency_key':'linked-collection-parent','action_id':'pipette.tip_status',
        'ownership_generation':generation,'requested_inputs':{}})
    assert created
    token = operator_controls._DISPATCH_CONTEXT.set({'operator_command_id':parent['command_id'],
        'idempotency_key':parent['idempotency_key'],'expected_ownership_generation':generation,
        'action_id':'pipette.tip_status','caller_class':'manual_operator'})
    async def runner(label, call, **kwargs): return call()
    rig[7]['data'][0] = [32,96,49]
    try:
        result = asyncio.run(run_pipette_operation('tip_status',lambda t:t.query_tip_status_all(),
            get_transport=lambda:rig[8],run_blocking=runner,receipt_store=rig[5],requested_inputs={}))
    finally:
        operator_controls._DISPATCH_CONTEXT.reset(token)
    linked = result['_bioxp_linked_pipette_finalization']
    assert result['receipt_id'] and linked
    with pytest.raises(PipetteReceiptError,match='pending'): state(rig)
    owner.put({**parent,'status':'observed','finished_at':1,'machine_assessment':'pass',
        'result':result},_expected_status=parent['status'],_linked_pipette_finalization=linked)
    assert state(rig)['tip_exists'] is True
    assert state(rig)['receipt_id'] == result['receipt_id']


def test_v10_migration_keeps_verified_v9_prefix_and_operational_rows(query_rig, tmp_path, monkeypatch):
    import hashlib
    import shutil
    from bioxp.oem_runtime_store import OEMRuntimeStore, verify_canonical_runtime_database
    source = query_rig[4]
    connection = query_rig[5].connection
    digest = connection.execute('SELECT backup_sha256 FROM runtime_schema_migrations WHERE version=10').fetchone()[0]
    backups = [p for p in source.glob('bioxp_runtime.db.pre-v2.*.sqlite3')
               if hashlib.sha256(p.read_bytes()).hexdigest() == digest]
    assert len(backups) == 1
    root = tmp_path/'v9-reopen'
    shutil.copytree(source,root)
    for suffix in ('','-wal','-shm'):
        (root/('bioxp_runtime.db'+suffix)).unlink(missing_ok=True)
    shutil.copyfile(backups[0],root/'bioxp_runtime.db')
    before = sqlite3.connect(root/'bioxp_runtime.db')
    assert before.execute('PRAGMA user_version').fetchone()[0] == 9
    schema = dict(before.execute('SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL'))
    tables = [r[0] for r in before.execute("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'")]
    rows = {t:before.execute('SELECT * FROM "'+t+'"').fetchall() for t in tables
            if t not in {'runtime_schema_migrations','runtime_store_identity'}}
    ledger = before.execute('SELECT * FROM runtime_schema_migrations ORDER BY version').fetchall()
    before.close()
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_STATE_ROOT', str(root))
    runtime = OEMRuntimeStore(root)
    try:
        db = runtime._db
        assert db.execute('PRAGMA user_version').fetchone()[0] == 10
        verify_canonical_runtime_database(db,full_data_check=True)
        assert [tuple(r) for r in db.execute('SELECT * FROM runtime_schema_migrations WHERE version<=9 ORDER BY version')] == ledger
        assert {t:[tuple(r) for r in db.execute('SELECT * FROM "'+t+'"')] for t in rows} == rows
        after = dict(db.execute('SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL'))
        assert set(after)-set(schema) == {'pipette_collection_claim_idx'}
        assert {key:after[key] for key in schema} == schema
    finally:
        runtime.close()
