"""Collection owner qualification: actual source setters, claims and SQLite."""
import asyncio
import json
import time
import threading
import pytest
from tests.test_deck_tip_query_publication import query_rig, query
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_tip_query_publication_contradiction import warm_no_tip, invoke


def read(rig):
    return rig[5].collection_state(identity=rig[8].collection_source_identity(),
        ownership_generation=rig[1].generation_provider())


def async_set(rig, loaded, fault=None):
    driver = rig[8]._transports[0]._get_driver()
    driver.pipette_id = 0
    now = time.monotonic()
    evidence = {'query_response_correlated': True, 'transaction_id': 'async-owner-query',
        'owner_generation': 1, 'channel': 0, 'tx_timestamp': now - .001}
    address = 0x506
    if fault == 'wrong_channel': address += 8
    if fault == 'stale': now -= 10; evidence['tx_timestamp'] -= 10
    if fault == 'unmatched': evidence['query_response_correlated'] = False
    if fault == 'reader': evidence['owner_generation'] = 99
    driver.process_pipette_message(3, [32,96,49 if loaded else 48],
        command_name='query_tip_status', arbitration_id=address,
        received_at=now, source_provenance=evidence)


def collect(rig):
    from bioxp import api
    return api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='isolated-owner-collection')


@pytest.mark.parametrize('fault',[None,'wrong_channel','stale','unmatched','reader'])
def test_async_source_requires_correlated_current_reader(query_rig, fault):
    rig=query_rig
    before=warm_no_tip(rig)
    async_set(rig, False, fault)
    with pytest.raises(RuntimeError): read(rig)
    calls=list(rig[6])
    collect(rig)
    assert rig[6] == calls
    state=read(rig)
    assert state['tip_exists'] is (False if fault is None else None)
    assert rig[0].state.operator_command_plane.store.deck_semantic_state() == before


def test_async_positive_durable_and_clock_reversal_does_not_select_history(query_rig, monkeypatch):
    rig=query_rig
    warm_no_tip(rig)
    async_set(rig, True)
    collect(rig)
    first=read(rig)
    assert first['tip_exists'] is True and first['event_id']
    # Exercise the actual event writer's clock, without altering source timestamps.
    from bioxp import runtime_audit_store
    original=runtime_audit_store.time.time
    with monkeypatch.context() as mp:
        mp.setattr(runtime_audit_store.time,'time',lambda: original()-10000)
        async_set(rig,False)
        rig[5].publish_collection_source(rig[8],ownership_generation=rig[1].generation_provider())
    second=read(rig)
    assert second['tip_exists'] is False and int(second['event_id']) > int(first['event_id'])
    before=rig[5].connection.execute('SELECT count(*) FROM runtime_events').fetchone()[0]
    collect(rig)
    assert rig[5].connection.execute('SELECT count(*) FROM runtime_events').fetchone()[0] == before


@pytest.mark.parametrize('operation,relevant',[('tip_status',True),('query_pressure',False)])
def test_pending_receipt_blocks_only_source_affecting_claim(query_rig,monkeypatch,operation,relevant):
    from bioxp.services.pipette_service import run_pipette_operation
    from bioxp.pipette.receipts import PipetteReceiptError
    from fastapi import HTTPException
    rig=query_rig
    warm_no_tip(rig)
    old=read(rig)
    async def blocking(label, call, **kwargs): return call()
    def failure(**kwargs): raise PipetteReceiptError('isolated receipt refusal')
    with monkeypatch.context() as mp:
        mp.setattr(rig[5],'record',failure)
        with pytest.raises(HTTPException,match='pipette_receipt_persistence_failed'):
            asyncio.run(run_pipette_operation(operation,
                (lambda t:t.query_tip_status_all()) if relevant else (lambda t:{'ok':True}),
                get_transport=lambda:rig[8],run_blocking=blocking,receipt_store=rig[5],
                runtime_binding={'idempotency_key':'pending-'+operation}))
    collect(rig)
    if relevant:
        with pytest.raises(PipetteReceiptError,match='pending'): read(rig)
    else:
        assert read(rig)==old
        assert rig[1].deck_authority_snapshot(expected_generation=rig[1].generation_provider(),target='LOC_PARK')


@pytest.mark.parametrize('fault',['owner','reader','generation','stop'])
def test_current_identity_drift_rejects_retained_false(query_rig,monkeypatch,fault):
    rig=query_rig
    warm_no_tip(rig)
    transport=rig[8]
    if fault=='owner': transport._collection_source_owner='another-owner'
    if fault=='reader': transport._transports[0]._driver.bus.router.reader_generation += 1
    if fault=='generation': monkeypatch.setattr(rig[1],'generation_provider',lambda:999)
    if fault=='stop': transport._interrupt_epoch += 1
    with pytest.raises(RuntimeError): read(rig)


def test_receiver_never_waits_for_sqlite_writer(query_rig):
    rig=query_rig
    warm_no_tip(rig)
    locked, release, received = threading.Event(),threading.Event(),threading.Event()
    def writer():
        with rig[5]._lock:
            locked.set()
            assert release.wait(5)
    worker=threading.Thread(target=writer)
    worker.start(); assert locked.wait(2)
    receiver=threading.Thread(target=lambda:(async_set(rig,True),received.set()))
    receiver.start()
    try: assert received.wait(1), 'source receiver waited for the SQLite writer'
    finally: release.set(); worker.join(3); receiver.join(3)
    collect(rig)
    assert read(rig)['tip_exists'] is True


def test_replay_has_no_query_or_source_publication(query_rig):
    rig=query_rig
    warm_no_tip(rig)
    old=read(rig); calls=list(rig[6])
    count=rig[5].connection.execute('SELECT count(*) FROM runtime_events').fetchone()[0]
    response=query(rig)
    assert response['replayed'] is True and rig[6]==calls and read(rig)==old
    assert rig[5].connection.execute('SELECT count(*) FROM runtime_events').fetchone()[0] == count


def test_collection_claim_lookup_uses_issued_order_index(query_rig):
    rig=query_rig
    warm_no_tip(rig)
    sql="SELECT * FROM pipette_operations WHERE json_extract(source_identity_json, '$.collection_source_affecting')=1 ORDER BY rowid DESC LIMIT 1"
    plan=' '.join(str(tuple(row)) for row in rig[5].connection.execute('EXPLAIN QUERY PLAN '+sql))
    assert 'pipette_collection_claim_idx' in plan and 'TEMP B-TREE' not in plan
