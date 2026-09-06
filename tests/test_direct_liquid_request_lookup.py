"""Offline actual handler/SQLite recovery; fake driver only, no API lifespan."""
import ast
import asyncio
import json
from pathlib import Path

import httpx
import pytest

from test_direct_liquid_receipt_replay import owner, ROOT, KEY
from bioxp.pipette.transport import FourPipetteTransport


@pytest.fixture
def lookup_owner(owner):
    app, namespace, calls, dependencies, _ = owner
    for dependency in dependencies.values():
        dependency['blockers'] = []
    source = ROOT / 'src/bioxp/api.py'
    namespace['__package__'] = 'bioxp'
    nodes = [node for node in ast.parse(source.read_text()).body
             if getattr(node, 'name', None) == 'liquid_request_lookup']
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), namespace)

    class FakeDriverTransport:
        """Synthetic query responses, never production hardware evidence."""
        def _get_driver(self):
            return self

        def query_firmware(self, number):
            return self.query_status()

        def query_status(self):
            calls.append('fake-driver-query')
            return {'ok': True, 'semantic_ok': True, 'query_response_correlated': True,
                    'hardware_truth_level': 'hardware_query'}

        def _safe_query_tip_status(self, driver, *, required):
            return {**self.query_status(), 'tip_loaded': False}

        def get_all_data(self, queries, *, wake_if_needed):
            assert wake_if_needed is False
            return {**self.query_status(), 'queries': list(queries)}

    collection = FourPipetteTransport([FakeDriverTransport() for _ in range(4)])
    collection._sleep = lambda _: None
    namespace['_get_pipette_transport'] = lambda: collection
    return owner


def get(app, kind='readback', **kwargs):
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://robot') as client:
            return await client.get('/liquid/requests?request_kind=' + kind,
                                    headers={'Idempotency-Key': KEY}, **kwargs)
    return asyncio.run(scenario())


def post(app, kind, body):
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://robot') as client:
            path = '/liquid/readback' if kind == 'readback' else '/liquid/application/plan'
            return await client.post(path, json=body, headers={'Idempotency-Key': KEY})
    response = asyncio.run(scenario())
    assert response.status_code == 200, response.text
    return response


def guarded_get(lookup_owner, monkeypatch, kind='readback'):
    app, namespace, calls, _, _ = lookup_owner
    store = namespace['_pipette_receipts']
    before = list(store.connection.iterdump())
    settings = ('busy_timeout', 'journal_mode', 'synchronous', 'foreign_keys', 'user_version', 'schema_version', 'query_only')
    settings_before = {name: tuple(store.connection.execute('PRAGMA ' + name).fetchone()) for name in settings}
    queries_before = list(calls)
    def forbidden(*args, **kwargs):
        pytest.fail('GET attempted mutation/constructor/hardware/current authority')
    statements = []
    with monkeypatch.context() as patch:
        patch.setattr(type(store), '__init__', forbidden)
        for name in ('claim', 'record', 'attach_to_operator_claim', 'replay_result', '_source_identity'):
            patch.setattr(store, name, forbidden)
        patch.setitem(namespace, '_get_pipette_transport', forbidden)
        store.connection.set_trace_callback(statements.append)
        response = get(app, kind)
        store.connection.set_trace_callback(None)
    assert list(store.connection.iterdump()) == before
    assert calls == queries_before
    assert len([s for s in statements if s.lstrip().upper().startswith('SELECT')]) == 1
    assert {name: tuple(store.connection.execute('PRAGMA ' + name).fetchone()) for name in settings} == settings_before
    pragmas = [s for s in statements if s.lstrip().upper().startswith('PRAGMA')]
    assert pragmas == ['PRAGMA busy_timeout', 'PRAGMA busy_timeout=0',
                       f'PRAGMA busy_timeout={settings_before["busy_timeout"][0]}']
    assert all(s in pragmas or s.lstrip().split()[0].upper() in {'SELECT', 'BEGIN', 'ROLLBACK'} for s in statements)
    assert response.headers['cache-control'] == 'no-store'
    return response


@pytest.mark.parametrize('kind,body', [
    ('readback', {'include_data': False}),
    ('application_plan', {'operation': 'move_to_waste'}),
])
@pytest.mark.parametrize('duplicate', [False, True])
def test_mustfix2_stored_result_must_be_object(lookup_owner, monkeypatch, kind, body, duplicate):
    app, namespace, *_ = lookup_owner
    post(app, kind, body)
    conn = namespace['_pipette_receipts'].connection
    summary = conn.execute('SELECT response_summary_json FROM operator_commands').fetchone()[0]
    receipt = json.loads(conn.execute('SELECT receipt_json FROM pipette_operations').fetchone()[0])
    pairs = list(receipt['result'].items())
    receipt['result'] = ([['ok', False]] + pairs) if duplicate else pairs
    conn.execute('UPDATE pipette_operations SET receipt_json=?', (json.dumps(receipt),))
    conn.commit()
    response = guarded_get(lookup_owner, monkeypatch, kind)
    assert conn.execute('SELECT response_summary_json FROM operator_commands').fetchone()[0] == summary
    assert response.status_code == 503, response.text
    assert response.json()['reason'] == 'stored_binding_invalid'
    assert response.json()['record'] is None


def test_mustfix1_get_cannot_create_or_chmod_lifecycle(lookup_owner, monkeypatch):
    import os
    from bioxp.runtime_audit_store import RuntimeLifecycleConnection
    conn = lookup_owner[1]['_pipette_receipts'].connection
    assert isinstance(conn, RuntimeLifecycleConnection)
    real_open = os.open
    def observational_open(path, flags, *args, **kwargs):
        assert not flags & (os.O_CREAT | os.O_RDWR | os.O_WRONLY), 'GET lifecycle open is mutating'
        return real_open(path, flags, *args, **kwargs)
    def forbidden(*args):
        pytest.fail('GET chmods lifecycle file')
    with monkeypatch.context() as patch:
        patch.setattr(os, 'open', observational_open)
        patch.setattr(os, 'fchmod', forbidden)
        response = guarded_get(lookup_owner, monkeypatch)
    assert response.status_code == 200, response.text


def test_existing_empty_store_is_unknown_without_writes(lookup_owner, monkeypatch):
    for _ in range(2):
        response = guarded_get(lookup_owner, monkeypatch)
        assert response.status_code == 200, response.text
        assert response.json() == {
            'schema': 'bioxp.direct-liquid.lookup.v1', 'request_kind': 'readback',
            'idempotency_key': KEY, 'lookup_state': 'unknown', 'reason': 'identity_not_found',
            'retry_forbidden': True, 'live_query_performed': False, 'record': None,
        }


@pytest.mark.parametrize('kind,body', [
    ('readback', {'include_data': False}), ('readback', {'include_data': True}),
    ('application_plan', {'operation': 'detect_fluid', 'fluid_class': 'RC'}),
    ('application_plan', {'operation': 'move_to_waste'}),
    ('application_plan', {'operation': 'plunger_up'}),
    ('application_plan', {'operation': 'plunger_down'}),
    ('application_plan', {'operation': 'load_tip', 'tip_tray': 'fixture-tray', 'tip_well': 'A1', 'tip_type': 1, 'tip_location': 0}),
])
def test_real_post_historical_lookup_retains_typed_result(lookup_owner, monkeypatch, kind, body):
    app, namespace, calls, dependencies, _ = lookup_owner
    original = post(app, kind, body).json()
    assert original['ok'] is True
    assert bool(calls) == (kind == 'readback')
    assert len(calls) == (4 * (4 if body['include_data'] else 3) if kind == 'readback' else 0)
    dependencies.clear()
    from bioxp.pipette import receipts
    monkeypatch.setattr(receipts, 'current_release_identity', lambda: {'verified': False})
    from types import SimpleNamespace
    monkeypatch.setattr(receipts, 'hardware_state', SimpleNamespace(ownership_epoch=999))
    for _ in range(2):
        response = guarded_get(lookup_owner, monkeypatch, kind)
        assert response.status_code == 200, response.text
        data = response.json()
        assert data['lookup_state'] == 'resolved', data
        r = data['record']
        assert r['ownership_generation'] != 999
        assert r['requested_inputs'] == (body if kind == 'readback' else {**body, 'home_z_after': True})
        assert r['result']['receipt_id'] == original['receipt_id']
        assert r['result']['receipt_truth'] == original['receipt_truth']
        assert all(original[k] == v for k, v in r['result'].items())
        assert r['result']['physical_effect_verified'] is False
        assert data['live_query_performed'] is False
        if kind == 'readback':
            assert r['result']['live_query_performed'] is True
            assert r['result']['channels'] == original['channels']
        else:
            assert (r['entrypoint_id'], r['caller_class'], r['control_class']) == ('legacy.record', 'legacy', 'pipette_state_command')


def reserve(lookup_owner):
    store = lookup_owner[1]['_pipette_receipts']
    claim, created = store.claim(operation='live_readback', requested_inputs={'include_data': False},
        entrypoint_id='direct.liquid.readback', caller_class='direct_api', control_class='hardware_query',
        idempotency_key=KEY)
    assert created
    return store, claim


@pytest.mark.parametrize('status,state,reason', [
    *[(s, 'pending', 'nonterminal') for s in ('reserved', 'queued', 'admitted', 'dispatched', 'acknowledged', 'executing', 'running', 'blocked')],
    *[(s, 'incomplete', 'outcome_unresolved') for s in ('ambiguous', 'outcome_unknown', 'reconciliation_required', 'future_status')],
    *[(s, 'incomplete', 'receipt_incomplete') for s in ('completed', 'observed', 'failed', 'rejected', 'cleared', 'cancelled')],
])
def test_sqlite_state_matrix(lookup_owner, monkeypatch, status, state, reason):
    store, _ = reserve(lookup_owner)
    # Explicit adversarial/historical SQLite fixture transitions, not production receipts.
    for table in ('operator_commands', 'pipette_operations'):
        store.connection.execute(f'UPDATE {table} SET status=?', (status,))
    store.connection.commit()
    response = guarded_get(lookup_owner, monkeypatch)
    assert response.status_code == 200, response.text
    data = response.json()
    assert (data['lookup_state'], data['reason']) == (state, reason)
    assert data['record']['command_status'] == status
    assert data['record']['pipette_status'] == status
    assert data['record']['result'] is None
    assert data['record']['outcome'] is None


@pytest.mark.parametrize('mode', ['missing_child', 'contradictory_terminal', 'foreign', 'wrong_kind', 'replay_disabled'])
def test_missing_child_and_global_family_lookup(lookup_owner, monkeypatch, mode):
    if mode == 'missing_child':
        from bioxp.pipette.receipts import _claim_source_identity
        store = lookup_owner[1]['_pipette_receipts']
        store._audit_database.claim(dict(command_id='fixture-missing-child', command_kind='pipette',
            idempotency_key=KEY, operation='live_readback', action_id='pipette.live_readback',
            entrypoint_id='direct.liquid.readback', caller_class='direct_api', control_class='hardware_query',
            ownership_generation=0, requested_inputs={'include_data': False},
            lifecycle_stage_id=None, lifecycle_attempt_id=None, source_identity=_claim_source_identity({})))
    else:
        store, _ = reserve(lookup_owner)
    kind = 'readback'
    if mode == 'missing_child':
        pass
    elif mode == 'contradictory_terminal':
        store.connection.execute("UPDATE operator_commands SET status='completed'")
        store.connection.execute("UPDATE pipette_operations SET status='failed'")
    elif mode == 'foreign':
        store.connection.execute("UPDATE operator_commands SET entrypoint_id='foreign.route'")
    elif mode == 'wrong_kind':
        kind = 'application_plan'
    else:
        store.connection.execute('UPDATE operator_commands SET idempotency_replay_enabled=0')
    store.connection.commit()
    response = guarded_get(lookup_owner, monkeypatch, kind)
    data = response.json()
    expected = {'missing_child': 'incomplete', 'contradictory_terminal': 'incomplete', 'foreign': 'conflict', 'wrong_kind': 'conflict', 'replay_disabled': 'unknown'}[mode]
    assert data['lookup_state'] == expected, data
    assert response.status_code == (409 if expected == 'conflict' else 200)
    if expected in {'conflict', 'unknown'}:
        assert data['record'] is None
    if mode == 'missing_child':
        assert data['record']['pipette_operation_id'] is None
        assert data['record']['pipette_status'] is None


@pytest.mark.parametrize('mutation', [
    'digest', 'child_request', 'child_source', 'child_owner', 'receipt_request', 'receipt_operation',
    'receipt_owner', 'receipt_source', 'receipt_source_type', 'receipt_truth_type', 'receipt_truth_effect',
    'receipt_truth_unknown', 'receipt_id', 'result_unknown', 'result_channels', 'result_include_data',
    'result_truth', 'result_callback', 'result_metadata_type', 'result_inner_receipt_id', 'result_summary',
])
def test_corrupt_bindings_never_recover_success(lookup_owner, monkeypatch, mutation):
    app, namespace, *_ = lookup_owner
    post(app, 'readback', {'include_data': False})
    store = namespace['_pipette_receipts']
    conn = store.connection
    if mutation == 'digest':
        conn.execute("UPDATE operator_commands SET canonical_request_sha256=?", ('f' * 64,))
    elif mutation.startswith('child_'):
        field, value = {'child_request': ('requested_inputs_json', '{"include_data":true}'),
                        'child_source': ('source_identity_json', '{}'), 'child_owner': ('ownership_generation', 123)}[mutation]
        conn.execute(f'UPDATE pipette_operations SET {field}=?', (value,))
    elif mutation == 'result_summary':
        conn.execute("UPDATE operator_commands SET response_summary_json='{}'")
    else:
        receipt = json.loads(conn.execute('SELECT receipt_json FROM pipette_operations').fetchone()[0])
        if mutation == 'receipt_request': receipt['requested_inputs']['include_data'] = True
        elif mutation == 'receipt_operation': receipt['operation'] = 'other'
        elif mutation == 'receipt_owner': receipt['ownership_epoch'] = 222
        elif mutation == 'receipt_source': receipt['source_identity']['registry_sha256'] = 'f' * 64
        elif mutation == 'receipt_source_type': receipt['source_identity'] = []
        elif mutation == 'receipt_truth_type': receipt['truth']['semantic_query_response_verified'] = 1
        elif mutation == 'receipt_truth_effect': receipt['truth']['physical_effect_verified'] = True
        elif mutation == 'receipt_truth_unknown': receipt['truth']['unclassified'] = False
        elif mutation == 'receipt_id': receipt['receipt_id'] = 'bad'
        elif mutation == 'result_unknown': receipt['result']['unclassified'] = 'never drop unknowns'
        elif mutation == 'result_channels': receipt['result']['channels'][1]['channel'] = 0
        elif mutation == 'result_include_data': receipt['result']['include_data'] = True
        elif mutation == 'result_truth': receipt['result']['semantic_query_response_verified'] = False
        elif mutation == 'result_callback': receipt['result']['callback_session_id'] = 'other'
        elif mutation == 'result_metadata_type': receipt['result']['hardware_truth_level'] = []
        elif mutation == 'result_inner_receipt_id': receipt['result']['receipt_id'] = 'f' * 32
        conn.execute('UPDATE pipette_operations SET receipt_json=?', (json.dumps(receipt),))
    conn.commit()
    response = guarded_get(lookup_owner, monkeypatch)
    assert response.status_code == 503, response.text
    assert response.json()['lookup_state'] == 'unavailable'
    assert response.json()['record'] is None


@pytest.mark.parametrize('suffix,headers,body', [
    ('', {'Idempotency-Key': KEY}, None),
    ('?request_kind=readback&request_kind=readback', {'Idempotency-Key': KEY}, None),
    ('?request_kind=readback&extra=x', {'Idempotency-Key': KEY}, None),
    ('?request_kind=unknown', {'Idempotency-Key': KEY}, None),
    ('?request_kind=readback', {}, None),
    ('?request_kind=readback', {'Idempotency-Key': 'short'}, None),
    ('?request_kind=readback', [('Idempotency-Key', KEY), ('Idempotency-Key', KEY)], None),
    ('?request_kind=readback', {'Idempotency-Key': KEY}, b'{}'),
])
def test_strict_get_transport_validation(lookup_owner, suffix, headers, body):
    app, namespace, calls, *_ = lookup_owner
    statements = []
    store = namespace['_pipette_receipts']
    store.connection.set_trace_callback(statements.append)
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://robot') as client:
            return await client.request('GET', '/liquid/requests' + suffix, headers=headers, content=body)
    response = asyncio.run(scenario())
    store.connection.set_trace_callback(None)
    assert response.status_code == 422
    assert response.headers['cache-control'] == 'no-store'
    assert statements == [] and calls == []


@pytest.mark.parametrize('mode', ['missing_owner', 'closed_connection', 'missing_schema', 'busy_lock', 'active_transaction'])
def test_unavailable_is_not_unknown(lookup_owner, monkeypatch, mode):
    app, namespace, calls, *_ = lookup_owner
    store = namespace['_pipette_receipts']
    if mode == 'missing_owner':
        monkeypatch.setitem(namespace, '_pipette_receipts', None)
    elif mode == 'closed_connection':
        store.connection.close()
    elif mode == 'missing_schema':
        import sqlite3
        monkeypatch.setattr(store, 'connection', sqlite3.connect(':memory:'))
    elif mode == 'busy_lock':
        class Busy:
            def acquire(self, *, timeout):
                assert 0 < timeout <= 0.25
                return False
        monkeypatch.setattr(store, 'lock', Busy())
    else:
        store.connection.execute('BEGIN')
    response = get(app)
    namespace['_pipette_receipts'] = store
    if mode == 'active_transaction':
        assert store.connection.in_transaction
        store.connection.rollback()
    assert response.status_code == 503
    assert response.json()['reason'] == 'store_unavailable'
    assert response.json()['record'] is None
    assert calls == []


def test_real_handler_failure_has_durable_terminal_evidence(lookup_owner, monkeypatch):
    app, namespace, calls, *_ = lookup_owner
    class FailedQuery:
        def readback_all(self, *, include_data):
            calls.append('failed-fake-query')
            raise ValueError('fixture malformed query response')
    namespace['_get_pipette_transport'] = lambda: FailedQuery()
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://robot') as client:
            return await client.post('/liquid/readback', json={'include_data': False}, headers={'Idempotency-Key': KEY})
    original = asyncio.run(scenario())
    assert original.status_code == 502
    recovered = guarded_get(lookup_owner, monkeypatch)
    assert recovered.status_code == 200
    data = recovered.json()
    assert data['lookup_state'] == 'resolved', data
    assert data['record']['result'] is None
    assert data['record']['command_status'] == 'failed'
    assert data['record']['failure_code'] == 'malformed_response'


@pytest.mark.parametrize('mutation', ['metadata', 'semantic_type', 'inner_identity', 'plan_request', 'source_release_type'])
def test_consistently_tampered_receipt_cannot_bypass_validation(lookup_owner, monkeypatch, mutation):
    app, namespace, *_ = lookup_owner
    kind = 'application_plan' if mutation == 'plan_request' else 'readback'
    body = {'operation': 'detect_fluid', 'fluid_class': 'RC'} if kind == 'application_plan' else {'include_data': False}
    post(app, kind, body)
    conn = namespace['_pipette_receipts'].connection
    receipt = json.loads(conn.execute('SELECT receipt_json FROM pipette_operations').fetchone()[0])
    if mutation == 'metadata': receipt['result']['hardware_truth_level'] = []
    if mutation == 'semantic_type': receipt['result']['semantic_query_response_verified'] = 1
    if mutation == 'inner_identity': receipt['result']['receipt_id'] = 'f' * 32
    if mutation == 'plan_request': receipt['result']['requested_inputs']['fluid_class'] = 'MS'
    if mutation == 'source_release_type': receipt['source_identity']['release_identity'] = []
    conn.execute('UPDATE pipette_operations SET receipt_json=?', (json.dumps(receipt),))
    conn.execute('UPDATE operator_commands SET response_summary_json=?', (json.dumps(receipt['result']),))
    conn.commit()
    response = guarded_get(lookup_owner, monkeypatch, kind)
    assert response.status_code == 503, response.text


@pytest.mark.parametrize('mode', ['missing_database', 'invalid_plan_inputs', 'unsafe_status'])
def test_unusable_stored_identity_fails_closed(lookup_owner, monkeypatch, mode):
    store = lookup_owner[1]['_pipette_receipts']
    if mode == 'missing_database':
        store.path.unlink()  # Test's own disposable SQLite file only.
        response = get(lookup_owner[0])
        assert response.status_code == 503, response.text
        assert response.json()['reason'] == 'store_unavailable'
        return
    if mode == 'invalid_plan_inputs':
        store.claim(operation='application_plan:detect_fluid', requested_inputs={'operation': 'detect_fluid', 'home_z_after': True},
                    entrypoint_id='legacy.record', caller_class='legacy', control_class='pipette_state_command', idempotency_key=KEY)
        kind = 'application_plan'
    else:
        reserve(lookup_owner)
        store.connection.execute("UPDATE operator_commands SET status='/private/fixture/path'")
        store.connection.commit()
        kind = 'readback'
    response = guarded_get(lookup_owner, monkeypatch, kind)
    assert response.status_code == 503, response.text
    assert response.json()['record'] is None


@pytest.mark.parametrize('failure', ['begin_denied', 'partial_begin', 'rollback_denied', 'select_and_rollback'])
def test_snapshot_failure_cleanup_retains_original_error(lookup_owner, monkeypatch, failure):
    import os
    import sqlite3
    import time
    from bioxp.runtime_audit_store import RuntimeLifecycleConnection
    store = lookup_owner[1]['_pipette_receipts']
    conn = store.connection
    coordinator = store._audit_database.coordinator
    before = list(conn.iterdump())
    owner_before = coordinator.snapshot()
    callback = lambda: 0
    conn.set_progress_handler(callback, 7)
    busy = conn.execute('PRAGMA busy_timeout').fetchone()[0]
    real_execute = RuntimeLifecycleConnection.execute
    rollback_denied = []
    def authorizer(action, arg1, *args):
        if action == sqlite3.SQLITE_TRANSACTION:
            if failure == 'begin_denied' and arg1 == 'BEGIN':
                return sqlite3.SQLITE_DENY
            if failure in {'rollback_denied', 'select_and_rollback'} and arg1 == 'ROLLBACK' and not rollback_denied:
                rollback_denied.append(True)
                return sqlite3.SQLITE_DENY
        return sqlite3.SQLITE_OK
    def execute(self, sql, parameters=()):
        result = real_execute(self, sql, parameters)
        if self is conn and sql == 'BEGIN' and failure == 'partial_begin':
            raise sqlite3.OperationalError('injected partial BEGIN')
        return result
    opened, closed = [], []
    real_open, real_close = os.open, os.close
    def open_fd(*args, **kwargs):
        fd = real_open(*args, **kwargs)
        opened.append(fd)
        return fd
    def close_fd(fd):
        closed.append(fd)
        return real_close(fd)
    with monkeypatch.context() as patch:
        patch.setattr(RuntimeLifecycleConnection, 'execute', execute)
        patch.setattr(os, 'open', open_fd)
        patch.setattr(os, 'close', close_fd)
        conn.set_authorizer(authorizer)
        try:
            with pytest.raises(sqlite3.DatabaseError) as caught:
                with coordinator:
                    with conn.direct_request_read_snapshot(coordinator=coordinator, deadline=time.monotonic() + .25):
                        if failure == 'select_and_rollback':
                            raise sqlite3.OperationalError('original SELECT failure')
                        conn.execute('SELECT 1').fetchone()
        finally:
            conn.set_authorizer(None)
    expected = {'begin_denied': 'not authorized', 'partial_begin': 'injected partial BEGIN',
                'rollback_denied': 'not authorized', 'select_and_rollback': 'original SELECT failure'}[failure]
    assert str(caught.value) == expected
    assert not conn.in_transaction
    assert conn._lifecycle_descriptor is None
    assert opened and closed == opened  # one close per allocated descriptor, no EBADF masking
    assert coordinator.snapshot() == owner_before
    assert conn._progress_handler_state == (callback, 7)
    assert conn.execute('PRAGMA busy_timeout').fetchone()[0] == busy
    assert list(conn.iterdump()) == before
    conn.set_progress_handler(None, 0)
    reserve(lookup_owner)  # the normal protected writer still works


def _snapshot_state(store):
    conn = store.connection
    return (list(conn.iterdump()), store._audit_database.coordinator.snapshot(),
            conn._progress_handler_state, conn.execute('PRAGMA busy_timeout').fetchone()[0],
            conn._lifecycle_descriptor, conn.in_transaction)


@pytest.mark.parametrize('contention', ['lifecycle', 'sqlite', 'owner', 'missing_lifecycle'])
def test_real_contention_is_bounded_and_observational(lookup_owner, contention):
    import os
    import select
    import sqlite3
    import subprocess
    import sys
    import threading
    import time
    from bioxp.runtime_audit_store import RUNTIME_LIFECYCLE_LOCK_NAME
    app, namespace, calls, _, root = lookup_owner
    assert str(root).startswith('/tmp/')  # only sandbox disposable paths
    store = namespace['_pipette_receipts']
    conn = store.connection
    # WAL readers do not block on an ordinary WAL writer; use a real rollback-
    # journal exclusive writer to exercise SQLite busy handling, not a mock.
    if contention == 'sqlite':
        conn.execute('PRAGMA wal_checkpoint(TRUNCATE)').fetchall()
        assert conn.execute('PRAGMA journal_mode=DELETE').fetchone()[0] == 'delete'
    callback_hits = []
    def callback():
        callback_hits.append(True)
        return 0
    conn.set_progress_handler(callback, 1)
    before = _snapshot_state(store)
    lock = root / RUNTIME_LIFECYCLE_LOCK_NAME
    lock_bytes, lock_mode = lock.read_bytes(), lock.stat().st_mode
    process = thread = None
    ready, release = threading.Event(), threading.Event()
    try:
        if contention in {'lifecycle', 'sqlite'}:
            code = ("import fcntl,os,sys; fd=os.open(sys.argv[1],os.O_RDONLY); fcntl.flock(fd,fcntl.LOCK_EX); print('ready',flush=True); sys.stdin.read(1)"
                    if contention == 'lifecycle' else
                    "import sqlite3,sys; c=sqlite3.connect(sys.argv[1]); c.execute('BEGIN EXCLUSIVE'); print('ready',flush=True); sys.stdin.read(1); c.rollback(); c.close()")
            process = subprocess.Popen([sys.executable, '-c', code, str(lock if contention == 'lifecycle' else store.path)],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            assert select.select([process.stdout], [], [], 5)[0], 'child did not acquire lock'
            assert process.stdout.readline().strip() == 'ready'
        elif contention == 'owner':
            def hold():
                with store.lock:
                    ready.set()
                    release.wait(5)
            thread = threading.Thread(target=hold)
            thread.start()
            assert ready.wait(2)
        else:
            lock.unlink()
        start = time.monotonic()
        response = get(app)
        elapsed = time.monotonic() - start
        assert response.status_code == 503, response.text
        assert response.json()['reason'] == 'store_unavailable'
        assert response.json()['record'] is None
        assert response.headers['cache-control'] == 'no-store'
        assert elapsed < .75, elapsed  # cooperative .25s budget plus scheduling tolerance
        assert calls == []
        if contention == 'missing_lifecycle':
            assert not lock.exists()
    finally:
        release.set()
        if thread is not None:
            thread.join(6)
            assert not thread.is_alive()
        if process is not None:
            if process.poll() is None:
                process.stdin.write('x')
                process.stdin.flush()
            try:
                process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=3)
            assert process.returncode == 0
    assert _snapshot_state(store) == before
    assert callback_hits  # restored callback actually runs, not merely retained metadata
    if contention != 'missing_lifecycle':
        assert (lock.read_bytes(), lock.stat().st_mode) == (lock_bytes, lock_mode)
    conn.set_progress_handler(None, 0)
    reserve(lookup_owner)


def test_snapshot_deadline_interrupts_sql_vm_and_restores_owner(lookup_owner):
    import sqlite3
    import time
    store = lookup_owner[1]['_pipette_receipts']
    conn = store.connection
    coordinator = store._audit_database.coordinator
    callback = lambda: 0
    conn.set_progress_handler(callback, 13)
    before = _snapshot_state(store)
    started = time.monotonic()
    with pytest.raises(sqlite3.OperationalError, match='interrupted'):
        with coordinator:
            with conn.direct_request_read_snapshot(coordinator=coordinator, deadline=started + .03):
                conn.execute('WITH RECURSIVE numbers(n) AS (VALUES(0) UNION ALL SELECT n+1 FROM numbers WHERE n<100000000) SELECT sum(n) FROM numbers').fetchone()
    assert time.monotonic() - started < .75
    assert _snapshot_state(store) == before
    with coordinator:
        with pytest.raises(sqlite3.OperationalError, match='unavailable'):
            with conn.direct_request_read_snapshot(coordinator=coordinator, deadline=time.monotonic() - 1):
                pytest.fail('expired snapshot entered')
    assert _snapshot_state(store) == before
    conn.set_progress_handler(None, 0)
    reserve(lookup_owner)


@pytest.mark.parametrize('mode', ['caller_transaction', 'foreign_connection', 'wrong_coordinator', 'unowned_coordinator', 'persistent_rollback_denial'])
def test_snapshot_cannot_release_foreign_or_unfinished_ownership(lookup_owner, monkeypatch, mode):
    import sqlite3
    import time
    from bioxp.runtime_audit_store import RuntimeWriteCoordinator
    app, namespace, calls, _, root = lookup_owner
    store = namespace['_pipette_receipts']
    conn = store.connection
    coordinator = store._audit_database.coordinator
    if mode == 'caller_transaction':
        with coordinator:
            conn.execute('BEGIN')
            before = _snapshot_state(store)
            response = get(app)
            assert response.status_code == 503
            assert response.json()['reason'] == 'store_unavailable'
            assert _snapshot_state(store) == before
            conn.rollback()
    elif mode == 'foreign_connection':
        foreign = sqlite3.connect(':memory:')
        foreign.execute('BEGIN')
        try:
            with monkeypatch.context() as patch:
                patch.setattr(store, 'connection', foreign)
                response = get(app)
                assert response.status_code == 503
                assert response.json()['reason'] == 'store_unavailable'
                assert foreign.in_transaction
        finally:
            foreign.close()
    elif mode in {'wrong_coordinator', 'unowned_coordinator'}:
        before = _snapshot_state(store)
        selected = RuntimeWriteCoordinator(root / 'not-the-owner') if mode == 'wrong_coordinator' else coordinator
        with pytest.raises(sqlite3.OperationalError, match='unavailable'):
            with conn.direct_request_read_snapshot(coordinator=selected, deadline=time.monotonic() + .25):
                pytest.fail('unowned snapshot entered')
        assert _snapshot_state(store) == before
    else:
        def authorizer(action, arg1, *args):
            return sqlite3.SQLITE_DENY if action == sqlite3.SQLITE_TRANSACTION and arg1 == 'ROLLBACK' else sqlite3.SQLITE_OK
        before = list(conn.iterdump())
        conn.set_authorizer(authorizer)
        try:
            response = get(app)
            assert response.status_code == 503
            assert response.json()['reason'] == 'store_unavailable'
            assert conn.in_transaction and conn._lifecycle_descriptor is not None
            descriptor = conn._lifecycle_descriptor
            # A second GET does not touch the failed transaction or its lease.
            assert get(app).status_code == 503
            assert conn.in_transaction and conn._lifecycle_descriptor == descriptor
            assert not coordinator.snapshot()['active']
        finally:
            conn.set_authorizer(None)
            with coordinator:
                conn.rollback()
        assert list(conn.iterdump()) == before
        assert conn._lifecycle_descriptor is None
    assert calls == []
    assert not coordinator.snapshot()['active']
    reserve(lookup_owner)
