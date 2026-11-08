"""Offline real V2/SQLite worker/provider/driver Fast-root FIFO contract."""
import threading
import time
from tests.test_movement_modes import connected, body, bind_deck  # noqa: F401
from tests.test_wake_setup_debloat import rig  # noqa: F401

ACTION = '/operator/v2/actions/oem.xy.move_sequence'

def terminal(http, cid):
    for _ in range(1500):
        response = http.get('/operator/v2/actions/receipts/' + cid + '?detail=true')
        assert response.status_code == 200, response.text
        row = response.json()
        if row['terminal']:
            return row
        time.sleep(.01)
    raise AssertionError(row)


def test_three_committed_fast_roots_fifo_and_replay(connected, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    waiting, release = threading.Event(), threading.Event()
    native_wait = driver.motor_wait_target_reached_many
    def wait(*a, **kw):
        waiting.set()
        assert release.wait(10), 'test did not release inert wire wait'
        return native_wait(*a, **kw)
    monkeypatch.setattr(driver, 'motor_wait_target_reached_many', wait)
    requests = [body('fifo-fast-root-%d' % i, {'targets': [
        {'x': 1100 + i * 2000, 'y': 2200 + i * 2000},
        {'x': 1500 + i * 2000, 'y': 2600 + i * 2000}]}) for i in range(3)]
    ids = []
    store = http.app.state.operator_command_plane.store
    try:
        for request in requests:
            start = time.monotonic()
            response = http.post(ACTION, json=request)
            assert response.status_code == 200, response.text
            assert time.monotonic() - start < 2
            ids.append(response.json()['command_id'])
            assert waiting.wait(3), http.get("/operator/v2/actions/receipts/" + ids[-1] + "?detail=true").json()
        assert len(set(ids)) == 3
        rows = [store.get_command(cid) for cid in ids]
        assert all(row is not None for row in rows)
        assert [row['status'] for row in rows] == ['dispatched', 'queued', 'queued']
        assert [row['stream_sequence'] for row in rows] == sorted(row['stream_sequence'] for row in rows)
        replay = http.post(ACTION, json=requests[1])
        assert replay.status_code == 200 and replay.json()['command_id'] == ids[1]
        changed = {**requests[1], 'inputs': {'targets': [{'x': 9999, 'y': 9999}]}}
        assert http.post(ACTION, json=changed).status_code == 409
        assert len([f for f in frames if f[1] == 4]) == 2
    finally:
        release.set()
    receipts = [terminal(http, cid) for cid in ids]
    assert [r['status'] for r in receipts] == ['completed'] * 3, receipts
    expected = []
    for request, receipt in zip(requests, receipts):
        result = receipt['source_receipt']['xy_sequence']
        assert result['completed_targets'] == 2
        assert [c['command_id'] for c in result['children']] == [receipt['command_id'] + ':xy:0', receipt['command_id'] + ':xy:1']
        assert receipt['xy_sequence']['completed_targets'] == 2
        assert receipt['physical_effect_verified'] is False
        for target in request['inputs']['targets']:
            expected.append({(5, target['x']), (4, target['y'])})
    moves = [(b, v) for b, c, t, m, v in frames if c == 4]
    assert [set(moves[i:i+2]) for i in range(0, len(moves), 2)] == expected
    before = list(frames)
    for request, cid in zip(requests, ids):
        response = http.post(ACTION, json=request)
        assert response.status_code == 200 and response.json()['command_id'] == cid
    assert frames == before


import pytest

@pytest.fixture(autouse=True)
def stop_routes(monkeypatch):
    from tests import test_movement_modes
    monkeypatch.setattr(test_movement_modes, 'ROUTES', test_movement_modes.ROUTES + (
        ('/motion/oem/x/stop', 'motion_oem_x_stop'),
        ('/motion/oem/y/stop', 'motion_oem_y_stop'),
        ('/motion/oem/z/stop', 'motion_oem_z_stop'),
        ('/motion/oem/x/abort', 'motion_oem_x_abort'),
    ))


@pytest.mark.parametrize('axis', ['x', 'y', 'z', 'abort_all'])
def test_stop_clears_queued_roots_and_old_epoch_suffix(connected, monkeypatch, axis):
    from bioxp import api
    http, provider, frames, fault, driver, positions = connected
    monkeypatch.setattr(api, '_get_tester', lambda: driver)
    waiting, release = threading.Event(), threading.Event()
    native_wait = driver.motor_wait_target_reached_many
    def wait(*a, **kw):
        waiting.set()
        assert release.wait(10)
        return native_wait(*a, **kw)
    monkeypatch.setattr(driver, 'motor_wait_target_reached_many', wait)
    requests = [body('stop-%s-root-%d' % (axis, i), {'targets': [
        {'x': 1100 + i * 2000, 'y': 2200 + i * 2000},
        {'x': 1500 + i * 2000, 'y': 2600 + i * 2000}]}) for i in range(3)]
    ids = []
    stops = []
    def stop_releases_wait(board, command, typ, motor, value):
        if command == 3:
            stops.append((board, motor))
            if axis != 'x' or len(stops) == 2:
                release.set()  # native Stop wakes the in-flight wait
    fault['on_frame'] = stop_releases_wait
    try:
        for request in requests:
            response = http.post(ACTION, json=request)
            assert response.status_code == 200, response.text
            ids.append(response.json()['command_id'])
            assert waiting.wait(3)
        stop = http.post('/operator/v2/actions/oem.%s%s' % (axis, '' if axis == 'abort_all' else '.stop'), json={
            'schema_version': 'bioxp.operator_interrupt_request.v1',
            'idempotency_key': 'fifo-stop-%s' % axis, 'reason': 'offline FIFO interrupt test',
            'observed_ownership_generation': 7, 'observed_board_epoch_by_board': {}})
        assert stop.status_code == 200, stop.text
        writes = [(b, m) for b, c, t, m, v in frames if c == 3]
        if axis == 'abort_all':
            assert not writes  # software abort is NOT physical Stop
        else:
            assert writes and set(writes) == {({'x': 5, 'y': 4, 'z': 4}[axis], 1 if axis == 'z' else 0)}
    finally:
        release.set()
    receipts = [terminal(http, cid) for cid in ids]
    assert receipts[0]['status'] != 'completed', receipts
    assert [r['status'] for r in receipts[1:]] == ['cleared', 'cleared'], receipts
    # Interrupt terminalization can win before the native worker returns, but
    # must not erase the original source-return child ledger.
    for _ in range(300):
        stopped = http.get('/operator/v2/actions/receipts/' + ids[0] + '?detail=true').json()
        source = stopped['source_receipt'].get('xy_sequence')
        if source is not None:
            break
        time.sleep(.01)
    assert source is not None, stopped
    assert source['ok'] is False and len(source['children']) <= 1
    # The existing fence is reset, not a new queue. Previously admitted suffix
    # remains terminal; a same-key observation cannot resurrect it.
    store = http.app.state.operator_command_plane.store
    store.release_interrupt_fence('oem.' + axis + ('' if axis == 'abort_all' else '.stop'))
    before = list(frames)
    for request, cid in zip(requests, ids):
        replay = http.post(ACTION, json=request)
        assert replay.status_code == 200 and replay.json()['command_id'] == cid
    time.sleep(.1)
    assert frames == before
    assert len([f for f in frames if f[1] == 4]) == 2


def test_failed_root_history_does_not_gate_following_intent(connected):
    http, provider, frames, fault, driver, positions = connected
    def fail(board, command, typ, motor, value):
        if command == 4 and value == 2200:
            raise RuntimeError('offline failed predecessor')
    fault['on_frame'] = fail
    request = body('fifo-failed-predecessor', {'targets': [{'x': 1100, 'y': 2200}]})
    first = http.post(ACTION, json=request)
    assert first.status_code == 200, first.text
    failed = terminal(http, first.json()['command_id'])
    assert failed['status'] != 'completed', failed
    fault['on_frame'] = None
    second = http.post(ACTION, json=body('fifo-after-failed-history', {'targets': [{'x': 5100, 'y': 6200}]}))
    assert second.status_code == 200, second.text
    assert terminal(http, second.json()['command_id'])['status'] == 'completed'


@pytest.mark.parametrize('status', ['failed', 'ambiguous'])
def test_durable_historical_terminal_is_record_only(connected, status):
    http, provider, frames, fault, driver, positions = connected
    store = http.app.state.operator_command_plane.store
    # Deliberately labelled history fixture, using the actual claim/finalizer;
    # not a physical predecessor or synthetic controller completion claim.
    request = {**body('fixture-history-' + status, {'targets': [{'x': 1100, 'y': 2200}]}),
               'action_id': 'oem.xy.move_sequence'}
    with store._lock:
        historical = store.admit_command(request, state={'ownership_generation': 7})
        claim = store.claim_next()
        assert claim['command_id'] == historical['command_id']
        store.finish(claim['command_id'], status=status,
            payload={'fixture_identity': 'offline-durable-history-not-hardware',
                     'delivery_attempted': status == 'ambiguous'}, claimed=claim)
    next_request = body('after-history-' + status, {'targets': [{'x': 5100, 'y': 6200}]})
    response = http.post(ACTION, json=next_request)
    assert response.status_code == 200, response.text
    receipt = terminal(http, response.json()['command_id'])
    assert receipt['status'] == 'completed', receipt
    assert store.get_command(claim['command_id'])['status'] == status
    assert len([f for f in frames if f[1] == 4]) == 2
    from bioxp.operator_receipt_store import OperatorReceiptStore
    cold = OperatorReceiptStore(store.root)
    try:
        saved = cold.by_command(receipt['command_id'])
        assert saved['xy_sequence'] == receipt['source_receipt']['xy_sequence']
    finally:
        cold.connection.close()


from tests.test_debloat_core import named  # noqa: F401


@pytest.mark.parametrize("roster", ["fast", "mixed"])
def test_mixed_fast_named_well_roots_use_existing_fifo(named, monkeypatch, roster):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from bioxp import api, operator_controls
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    from tests import z_stop_fixtures
    run, provider, store, counts, frames, fault = named
    driver = provider.primitives.tester
    provider.y_provider = provider.primitives.y_provider
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    app = FastAPI()
    app.add_api_route('/motion/oem/move_xy_sequence', api.motion_oem_move_xy_sequence, methods=['POST'])
    monkeypatch.setattr(z_stop_fixtures, 'FastAPI', lambda: app)
    monkeypatch.setattr('bioxp.operator_command_plane.OperatorCommandStore', lambda *a, **kw: store)
    install = operator_controls.install_operator_control_plane
    def with_native_owner(application, **kwargs):
        kwargs.update(oem_deck_provider=lambda: provider,
            oem_deck_position_table_provider=load_bound_oem_position_table,
            serial206_initialization_state_provider=api.serial206_oem_initialization_provider_status)
        return install(application, **kwargs)
    monkeypatch.setattr(operator_controls, 'install_operator_control_plane', with_native_owner)
    z_stop_fixtures.make_app(store.root, monkeypatch)
    project = operator_controls.hardware_state.project
    monkeypatch.setattr(operator_controls.hardware_state, 'project',
        lambda *a, include_lifecycle=False, **kw: project(*a, **kw))
    # Existing named fixture's source-logical no-tip predecessor, not a sensor claim.
    provider.bind_pipette_collection_state_reader(lambda: {'tip_exists': False})
    entered, release = threading.Event(), threading.Event()
    native_wait = driver.motor_wait_target_reached_many
    def wait(*a, **kw):
        entered.set()
        assert release.wait(10)
        return native_wait(*a, **kw)
    monkeypatch.setattr(driver, 'motor_wait_target_reached_many', wait)
    plane = app.state.operator_command_plane
    dispatch = plane._dispatch_one
    dispatched = []
    def observe(claim):
        dispatched.append(claim['command_id'])
        return dispatch(claim)
    monkeypatch.setattr(plane, '_dispatch_one', observe)
    requests = [
        ('oem.xy.move_sequence', {'targets': [{'x': 1100, 'y': 2200}, {'x': 1500, 'y': 2600}]}),
        ('oem.deck.move_to_location', {'target': 'LOC_OC', 'camera_offset': False}),
        ('oem.deck.move_to_well', {'location_id': 2, 'well': 'B3', 'position_flag': 1}),
        ('oem.xy.move_sequence', {'targets': [{'x': 5100, 'y': 6200}]}),
    ]
    if roster == "fast":
        requests = [("oem.xy.move_sequence", {"targets": [
            {"x": 1100 + i * 2000, "y": 2200 + i * 2000},
            {"x": 1500 + i * 2000, "y": 2600 + i * 2000}]}) for i in range(3)]
    frames.clear()
    with TestClient(app) as http:
        plane.start()
        ids = []
        admission_s = []
        try:
            for index, (action, inputs) in enumerate(requests):
                start = time.monotonic()
                response = http.post('/operator/v2/actions/' + action, json=body('mixed-fifo-%d' % index, inputs))
                assert response.status_code == 200, response.text
                admission_s.append(time.monotonic() - start)
                assert admission_s[-1] < 2
                ids.append(response.json()['command_id'])
                assert entered.wait(3)
            assert dispatched == [ids[0]]
            assert [store.get_command(cid)['status'] for cid in ids] == ['dispatched'] + ['queued'] * (len(ids) - 1)
        finally:
            release.set()
        try:
            receipts = [terminal(http, cid) for cid in ids]
            assert [r['status'] for r in receipts] == ['completed'] * len(ids), receipts
            assert dispatched == ids
            assert len(set(ids)) == len(requests)
            stamps = provider.deck_owner_authority_stamps()
            for receipt in receipts:
                assert receipt['expected_board_epoch_by_board'] == {
                    '4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
            # Actual producer summaries still survive history/dashboard as well
            # as detail after moving the root into the durable owner.
            from tests.test_xy_compact_receipts import surfaces
            for cid, (action, inputs) in zip(ids, requests):
                if action == 'oem.xy.move_sequence':
                    for surface in surfaces(http, cid):
                        assert surface['xy_sequence']['completed_targets'] == len(inputs['targets'])
            import json, os
            from pathlib import Path
            if output := os.environ.get('BIOXP_FAST_FIFO_EXPORT'):
                destination = Path(output)
                destination.mkdir(parents=True, exist_ok=True)
                (destination / (roster + '.json')).write_text(json.dumps({
                    'fixture_identity': 'offline-real-api-sqlite-inert-wire-not-hardware',
                    'requests': requests, 'command_ids': ids, 'admission_s': admission_s,
                    'dispatched': dispatched, 'receipts': receipts, 'wire': frames}, indent=2))
        finally:
            plane.stop()


@pytest.mark.parametrize('fail', [False, True])
def test_pre_fifo_direct_key_recovers_without_readmission(connected, fail):
    http, provider, frames, fault, driver, positions = connected
    request = body('pre-fifo-key-' + str(fail), {'targets': [{'x': 1100, 'y': 2200}]})
    if fail:
        def reject(board, command, typ, motor, value):
            if command == 4 and value == 2200:
                raise RuntimeError('offline retained direct failure')
        fault['on_frame'] = reject
    old = http.post('/operator/actions/oem.xy.move_sequence', json={
        'expected_generation': 7, 'idempotency_key': request['idempotency_key'],
        'expected_board_epoch_by_board': {}, 'inputs': request['inputs']})
    retained = http.app.state.operator_receipt_store.by_idempotency(request['idempotency_key'])
    assert retained is not None, old.text
    cid = retained['command_id']
    fault['on_frame'] = None
    before = list(frames)
    replay = http.post(ACTION, json=request)
    assert replay.status_code == 200, replay.text
    assert replay.json()['command_id'] == cid
    assert replay.json()['status'] == ('failed' if fail else 'completed')
    assert frames == before
    assert http.app.state.operator_command_plane.store.idempotency('command', request['idempotency_key']) is None
    changed = {**request, 'inputs': {'targets': [{'x': 3100, 'y': 4200}]}}
    assert http.post(ACTION, json=changed).status_code == 409
    assert frames == before



def test_same_key_commit_between_identity_reads_recovers_same_root(connected, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    http, provider, frames, fault, driver, positions = connected
    store = http.app.state.operator_command_plane.store
    identity = store.idempotency
    captured, release = threading.Event(), threading.Event()
    guard = threading.Lock()
    reads = []
    def delayed(kind, key):
        result = identity(kind, key)
        with guard:
            first = not reads
            reads.append((kind, key))
        if first:
            captured.set()
            assert release.wait(5)
        return result  # actual earlier read, not a fabricated row
    monkeypatch.setattr(store, 'idempotency', delayed)
    request = body('fifo-identity-read-race', {'targets': [{'x': 1100, 'y': 2200}]})
    with ThreadPoolExecutor(max_workers=1) as executor:
        overlapping = executor.submit(http.post, ACTION, json=request)
        try:
            assert captured.wait(3)
            committed = http.post(ACTION, json=request)
            assert committed.status_code == 200, committed.text
        finally:
            release.set()
        replay = overlapping.result(5)
    assert replay.status_code == 200, replay.text
    assert replay.json()['command_id'] == committed.json()['command_id']
    assert terminal(http, committed.json()['command_id'])['status'] == 'completed'
    assert len([f for f in frames if f[1] == 4]) == 2
