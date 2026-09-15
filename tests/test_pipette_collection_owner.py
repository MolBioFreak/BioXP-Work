"""SQLite collection owner qualification; hardware leaves only are doubled."""
import asyncio
import copy
import json
import sqlite3
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import pytest
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptError
from tests.test_deck_tip_query_publication import query_rig, query, named_move
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_tip_query_publication_contradiction import warm_no_tip, invoke, assert_park_unready, assert_worker_refused, submit_named


def state(rig):
    return rig[5].collection_state(identity=rig[8].collection_source_identity(),
        ownership_generation=rig[1].generation_provider())


def async_set(rig, channel, value):
    driver = rig[8]._transports[channel]._driver
    driver.pipette_id = channel
    now = time.monotonic()
    return driver.process_pipette_message(3, [32,96,value],
        arbitration_id=0x506 + channel * 8, command_name='query_tip_status', received_at=now,
        source_provenance={'query_response_correlated': True, 'channel': channel,
            'transaction_id': f'current-async-reply:{channel}:{now}',
            'owner_generation': driver.bus.router.reader_generation, 'tx_timestamp': now - .001})


def native_leaves(rig, monkeypatch):
    from bioxp import oem_machine_bundle
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    snapshot = oem_machine_bundle.load_oem_machine_snapshot(snapshot.bundle_root/'OEM_EVIDENCE_LOCK.json',
        operator_label_serial=206, require_operator_label=True)
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot', snapshot)
    rig[2].oem_initialize_motion_scriptmove_to_waste = (
        Serial206ProductionPrimitiveAdapter.oem_initialize_motion_scriptmove_to_waste.__get__(rig[2]))


@pytest.mark.parametrize('byte,expected', [(49,True),(48,False),(50,None)])
def test_async_active_collection_and_passive_sql_owner(query_rig, byte, expected):
    rig = query_rig
    before = warm_no_tip(rig)
    old = state(rig)
    calls = list(rig[6])
    async_set(rig, 0, byte)
    with pytest.raises(PipetteReceiptError): state(rig)
    with pytest.raises(RuntimeError):
        rig[1].deck_authority_cached_snapshot(expected_generation=rig[1].generation_provider(), target='LOC_PARK')
    result = api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='async-owner-test')
    assert result['pipette_collection']['tip_exists'] is expected, result
    current = state(rig)
    assert current['tip_exists'] is expected
    assert current['identity'] != old['identity'] and current['event_id']
    assert rig[6] == calls  # ordinary collection does not query pipettes
    assert rig[0].state.operator_command_plane.store.deck_semantic_state() == before
    count = rig[5].connection.execute("SELECT count(*) FROM runtime_events WHERE event_kind LIKE 'pipette_collection_source:%'").fetchone()[0]
    api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='unchanged-owner-test')
    assert rig[5].connection.execute("SELECT count(*) FROM runtime_events WHERE event_kind LIKE 'pipette_collection_source:%'").fetchone()[0] == count


@pytest.mark.parametrize('fault', ['owner','reader','reconnect','interrupt','generation'])
def test_source_identity_drift_never_reuses_false(query_rig, fault):
    rig = query_rig
    warm_no_tip(rig)
    identity = rig[8].collection_source_identity()
    if fault == 'owner': rig[8]._collection_source_owner = 'different-owner'
    if fault == 'reader': rig[8]._transports[0]._driver.bus.router.reader_generation += 1
    if fault == 'reconnect': rig[8]._transports[0]._driver.bus.router = SimpleNamespace(reader_generation=1)
    if fault == 'interrupt': rig[8]._interrupt_epoch += 1
    if fault == 'generation':
        with pytest.raises(PipetteReceiptError):
            rig[5].collection_state(identity=identity, ownership_generation=rig[1].generation_provider()+1)
    else:
        with pytest.raises(PipetteReceiptError): state(rig)
        result = api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='drift-test')
        assert result['pipette_collection']['available'] is False


def test_pending_real_receipt_write_refusal_cannot_publish_ram(query_rig):
    rig = query_rig
    warm_no_tip(rig)
    rig[7]['data'][0] = [32,96,49]
    denied = []
    def authorizer(action, table, column, db, trigger):
        if action == sqlite3.SQLITE_UPDATE and table == 'pipette_operations' and column == 'receipt_json':
            denied.append(column)
            return sqlite3.SQLITE_DENY
        return sqlite3.SQLITE_OK
    rig[5].connection.set_authorizer(authorizer)
    try: response = invoke(rig, 'real-receipt-refusal')
    finally: rig[5].connection.set_authorizer(None)
    assert denied and response.status_code == 503, response.text
    row = rig[5].connection.execute('SELECT * FROM pipette_operations ORDER BY rowid DESC LIMIT 1').fetchone()
    assert row['status'] in ('reserved','queued','dispatched','issued_pending')
    with pytest.raises(PipetteReceiptError, match='pending'): state(rig)
    result = api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='pending-owner-test')
    assert result['pipette_collection']['available'] is False
    refusal = assert_park_unready(rig, 'pipette_collection_receipt_pending')
    assert_worker_refused(rig, 'pending-receipt-active-worker', refusal)


def test_async_receiver_does_not_wait_on_sql_writer_and_midcommit_drift(query_rig):
    rig = query_rig
    warm_no_tip(rig)
    # An independent physical connection/process holds the real SQLite writer.
    code = "import sqlite3,sys;c=sqlite3.connect(sys.argv[1]);c.execute('BEGIN IMMEDIATE');print('held',flush=True);sys.stdin.readline();c.rollback()"
    worker = subprocess.Popen([sys.executable,'-c',code,str(rig[5].path)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    assert worker.stdout.readline().strip() == 'held'
    attempted = threading.Event()
    rig[5].connection.set_trace_callback(lambda sql: attempted.set() if sql == 'BEGIN IMMEDIATE' else None)
    result = []
    async_set(rig,0,49)
    thread = threading.Thread(target=lambda: result.append(api._collect_and_publish_hardware_snapshot(
        ['axes','latch'],reason='blocked-writer-owner-test')))
    try:
        thread.start()
        assert attempted.wait(2)
        done = threading.Event()
        receiver = threading.Thread(target=lambda: (async_set(rig,1,49),done.set()))
        receiver.start()
        assert done.wait(.5), 'receiver waited on SQLite'
        receiver.join(1)
        assert thread.is_alive() and worker.poll() is None
    finally:
        worker.stdin.write('\n'); worker.stdin.flush(); worker.wait(timeout=3)
        thread.join(5)
        rig[5].connection.set_trace_callback(None)
    assert not thread.is_alive()
    assert result[0]['pipette_collection']['available'] is False
    with pytest.raises(PipetteReceiptError): state(rig)
    api._collect_and_publish_hardware_snapshot(['axes','latch'],reason='post-contention-collection')
    assert state(rig)['tip_exists'] is True


def test_partial_exception_retains_earlier_independent_source(query_rig):
    rig = query_rig
    warm_no_tip(rig)
    rig[7]['data'][0] = [32,96,49]
    def later(channel):
        if channel == 1:
            async_set(rig,2,49)
            raise RuntimeError('actual source unwind after independent async setter')
    rig[7]['during'] = later
    result = invoke(rig,'partial-with-async')
    assert result.status_code == 502
    assert state(rig)['tip_exists'] is True
    snapshot = json.loads(rig[5].connection.execute('SELECT receipt_json FROM pipette_operations ORDER BY rowid DESC LIMIT 1').fetchone()[0])['result']['collection_source']
    assert [c['tip_loaded'] for c in snapshot['channels']] == [True,False,True,False]


def fresh_read(root, identity, generation):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    from bioxp.pipette.receipts import PipetteReceiptStore
    with pytest.MonkeyPatch.context() as mp:
        bind_serial206_oem_snapshot(mp)
        from tests.test_deck_tip_query_publication import bind_collection_test_identity
        bind_collection_test_identity(mp)
        return PipetteReceiptStore(root).collection_state(identity=identity, ownership_generation=generation)


def test_fresh_process_exact_identity_and_replay_no_new_publication(query_rig):
    rig = query_rig
    warm_no_tip(rig)
    expected = state(rig)
    script = 'import json,sys;from tests.test_pipette_collection_owner import fresh_read;print(json.dumps(fresh_read(sys.argv[1],json.loads(sys.argv[2]),int(sys.argv[3]))))'
    reopened = json.loads(subprocess.check_output([sys.executable,'-c',script,str(rig[4]),
        json.dumps(rig[8].collection_source_identity()),str(rig[1].generation_provider())],text=True))
    assert reopened == expected
    calls = list(rig[6])
    counts = rig[5].connection.execute('SELECT (SELECT count(*) FROM pipette_operations),(SELECT count(*) FROM runtime_events)').fetchone()
    result = query(rig)
    assert result['replayed'] is True and rig[6] == calls
    assert tuple(rig[5].connection.execute('SELECT (SELECT count(*) FROM pipette_operations),(SELECT count(*) FROM runtime_events)').fetchone()) == tuple(counts)
    assert state(rig) == expected


def test_fully_known_loaded_queued_park_eject_final_query_and_next_command(query_rig, monkeypatch):
    rig = query_rig
    native_leaves(rig,monkeypatch)
    warm_no_tip(rig)
    provider, primitive, receipts, transport = rig[1],rig[2],rig[5],rig[8]
    rig[7]['data'] = [[32,96,49] for _ in range(4)]
    query(rig,key='all-loaded')
    provider.publish_pipette_owner_state(tip_loaded=True,tip_dirty=False,tip_location=-1,source_command_id='known-loaded-owner')
    store = rig[0].state.operator_command_plane.store
    store.publish_tip_tray_transition(tray_id=0,transition='construct',operation_id='known-tray',
        command_id='known-tray',provenance={'source':'explicit offline predecessor'},**provider.deck_owner_authority_stamps())
    provider.publish_clean_path_state(expected_clean_path=False,source_command_id='known-clean-path')
    assert state(rig)['tip_exists'] is True
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    primitive.pipette_audit_runner = api._run_serial206_pipette_audit
    for name in ('_run_audited_pipette','query_all_pipette_tip_states','query_tip_status','eject_all_tips_for_oem_park'):
        setattr(primitive,name,getattr(Serial206ProductionPrimitiveAdapter,name).__get__(primitive))
    # Only physical eject transmissions are doubled; real collection before/after queries and audit remain.
    def physical_eject(channels, **kw):
        rig[7]['data'] = [[32,96,48] for _ in range(4)]
        return []
    monkeypatch.setattr(transport,'_eject_tip_channels_once',physical_eject)
    primitive.oem_initialize_motion_move_absolute = lambda axis,target,**kw: {
        'ok':True,'controller_command_acknowledged':True,'controller_completion_verified':True}
    prior = len(primitive.calls)
    receipt = submit_named(rig,'LOC_PARK','known-loaded-park')
    moves = [c for c in primitive.calls[prior:] if c[0]=='move']
    assert moves and moves[0][2]['tip_loaded'] is True
    assert state(rig)['tip_exists'] is False
    rows = receipts.connection.execute("SELECT operation,receipt_json FROM pipette_operations ORDER BY rowid").fetchall()
    ejected = [json.loads(row['receipt_json']) for row in rows if 'eject' in row['operation']]
    assert ejected and ejected[-1]['result']['after']['source_tip_exists'] is False
    assert all(c['tip_loaded'] is False for c in ejected[-1]['result']['collection_source']['channels'])
    assert store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    assert store.deck_semantic_state()['tip_loaded'] is False
    submit_named(rig,'LOC_OC','next-after-loaded-park')
