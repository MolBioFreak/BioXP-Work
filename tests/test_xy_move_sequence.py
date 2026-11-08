"""Connected canonical V2 -> shared worker -> provider -> native adapter/waits.

Only wire exchange is synthetic (existing rig); offline release identity is
explicitly a fixture, not deployed identity or physical evidence.
"""
import time
import threading
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from tests.test_wake_setup_debloat import rig, cycle


@pytest.fixture
def connected(rig, monkeypatch):
    from bioxp import api
    from bioxp.serial206_y_provider import Serial206YProvider
    from tests import z_stop_fixtures
    driver, provider, frames, fault, root = rig
    cycle(driver)
    provider.primitives.y_provider = Serial206YProvider(driver,
        state_store=provider.state_store, generation_provider=lambda: 7,
        reference_store=provider.reference_store)
    positions = {}
    wire = driver.send_tmcl
    def exchange(board, command, typ, motor, value, **kw):
        result = wire(board, command, typ, motor, value, **kw)
        if command == 4 and result['status'] == 100:
            positions[board, motor] = value
        if command == 6 and typ == 1:
            result['value'] = positions.get((board, motor), 0)
        return result
    monkeypatch.setattr(driver, 'send_tmcl', exchange)
    monkeypatch.setattr(driver, 'send_tmcl_retry', exchange)
    initialized = provider.initialize_motors(mode='live')
    assert initialized['ok'], initialized
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    app = FastAPI()
    app.add_api_route('/motion/oem/move_xy_sequence', api.motion_oem_move_xy_sequence, methods=['POST'])
    monkeypatch.setattr(z_stop_fixtures, 'FastAPI', lambda: app)
    app, _ = z_stop_fixtures.make_app(root, monkeypatch)
    from bioxp import operator_controls
    project = operator_controls.hardware_state.project
    monkeypatch.setattr(operator_controls.hardware_state, 'project',
        lambda *a, include_lifecycle=False, **kw: project(*a, **kw))
    frames.clear()
    with TestClient(app) as http:
        app.state.operator_command_plane.start()
        try:
            yield http, provider, frames, fault, driver
        finally:
            app.state.operator_command_plane.store.stop()


def request(key='sequence-test-0001', targets=None):
    return {'schema_version': 'bioxp.operator_action_request.v2',
            'idempotency_key': key, 'expected_ownership_generation': 7,
            'expected_board_epoch_by_board': {},
            'inputs': {'targets': targets if targets is not None else [
                {'x': 1100, 'y': 2200}, {'x': 3300, 'y': 4400}, {'x': 5500, 'y': 6600}]}}


def submit(http, body):
    response = http.post('/operator/v2/actions/oem.xy.move_sequence', json=body)
    assert response.status_code == 200, response.text
    admitted = response.json()
    cid = admitted['command_id']
    for _ in range(1500):
        response = http.get('/operator/v2/actions/receipts/' + cid + '?detail=true')
        assert response.status_code == 200, response.text
        result = response.json()
        if result['status'] not in {'queued', 'dispatched', 'running', 'accepted'}:
            return cid, result
        time.sleep(.01)
    raise AssertionError(result)


def sequence_body(receipt):
    # Real canonical receipt retains the underlying route response.
    return receipt['source_receipt']['xy_sequence']


def test_order_waits_and_same_key_never_replays(connected, monkeypatch):
    http, provider, frames, fault, driver = connected
    waits = []
    original = driver.motor_wait_target_reached_many
    def wait(*a, **kw):
        waits.append((threading.get_ident(), len([f for f in frames if f[1] == 4]), kw['sta_sequential']))
        return original(*a, **kw)
    monkeypatch.setattr(driver, 'motor_wait_target_reached_many', wait)
    cid, receipt = submit(http, request())
    assert receipt['status'] == 'completed', receipt
    result = sequence_body(receipt)
    assert result.get('ok') and result.get('completed_targets') == 3, receipt
    assert [c['target'] for c in result['children']] == request()['inputs']['targets']
    assert [n for _, n, _ in waits] == [2, 4, 6]
    assert len({tid for tid, _, _ in waits}) == 1
    assert all(sta is False for _, _, sta in waits)
    moves = [(b, v) for b, c, t, m, v in frames if c == 4]
    assert [set(moves[i:i+2]) for i in range(0, 6, 2)] == [
        {(4, 2200), (5, 1100)}, {(4, 4400), (5, 3300)}, {(4, 6600), (5, 5500)}]
    before = list(frames)
    again, replay = submit(http, request())
    assert again == cid and replay['status'] == 'completed'
    assert frames == before
    # Each waypoint has a distinct durable provider receipt; none replaces root.
    assert len({child['command_id'] for child in result['children']}) == 3
    for child in result['children']:
        assert provider._durable_serial206_receipt('x', child['command_id']) is not None


def test_failure_stops_future_targets(connected):
    http, provider, frames, fault, driver = connected
    def reject_second(board, command, typ, motor, value):
        if command == 4 and value == 4400:
            raise RuntimeError('offline second-target transport failure')
    fault['on_frame'] = reject_second
    cid, receipt = submit(http, request())
    assert receipt['status'] != 'completed', receipt
    result = sequence_body(receipt)
    assert result['ok'] is False and result['completed_targets'] == 1
    assert result['failed_target_index'] == 1
    assert not any(c == 4 and v in {5500, 6600} for b, c, t, m, v in frames)
    before = list(frames)
    again, replay = submit(http, request())
    assert again == cid and replay['status'] == receipt['status']
    assert frames == before


@pytest.mark.parametrize('axis', ['x', 'y'])
def test_addressed_interrupt_generation_stops_next_point_even_after_clear(connected, monkeypatch, axis):
    http, provider, frames, fault, driver = connected
    # Inject exactly at the settled-child handoff; real provider/receipts and
    # native waits run. Epoch owner is the existing Stop owner, not a new flag.
    original = provider.execute_xy_intent
    def first(*a, **kw):
        result = original(*a, **kw)
        if axis == 'x':
            with provider._x_interrupt_state_lock:
                provider._x_interrupt_epoch += 1
        else:
            epoch = provider.state_store.begin_axis_interrupt('y')
            provider.state_store.end_axis_interrupt('y', reconciled=True, expected_epoch=epoch, persistence_owner=True)
        return result
    monkeypatch.setattr(provider, 'execute_xy_intent', first)
    cid, receipt = submit(http, request())
    result = sequence_body(receipt)
    assert result['interrupted'] and result['next_target_index'] == 1
    assert len([f for f in frames if f[1] == 4]) == 2


@pytest.mark.parametrize('targets', [[], [{'x': True, 'y': 2}], [{'x': 1, 'y': 2, 'speed': 3}], [{'x': 2**31, 'y': 2}], [{'x': 1, 'y': 2}] * 65])
def test_invalid_sequence_never_moves(connected, targets):
    http, provider, frames, fault, driver = connected
    cid, receipt = submit(http, request(targets=targets))
    assert receipt['status'] != 'completed', receipt
    assert not any(c == 4 for b, c, t, m, v in frames)


def test_native_addressed_x_stop_delivers_while_root_owns_provider_lock(connected, monkeypatch):
    http, provider, frames, fault, driver = connected
    waiting = threading.Event()
    delivered = threading.Event()
    original = driver.motor_wait_target_reached_many
    def wait(*a, **kw):
        waiting.set()
        assert delivered.wait(3), 'Stop was blocked behind the sequence owner'
        return original(*a, **kw)
    monkeypatch.setattr(driver, 'motor_wait_target_reached_many', wait)
    returned = []
    errors = []
    def stop():
        try:
            assert waiting.wait(3)
            returned.append(provider.execute_x_intent('stop', {
                'command_id': 'offline-addressed-stop', 'idempotency_key': 'offline-stop-key',
                'defer_reconciliation': True}))
        except BaseException as exc:
            errors.append(exc)
        finally:
            delivered.set()
    worker = threading.Thread(target=stop)
    worker.start()
    cid, receipt = submit(http, request())
    worker.join(3)
    assert not worker.is_alive() and not errors, errors
    assert len([(b, m) for b, c, t, m, v in frames if c == 3]) == 2
    assert all((b, m) == (5, 0) for b, c, t, m, v in frames if c == 3)
    assert receipt['status'] != 'completed'
    assert not any(c == 4 and v in {3300, 4400, 5500, 6600} for b, c, t, m, v in frames)
    assert callable(returned[0])
    returned[0]()  # Existing post-delivery reconciliation, after root settles.


def test_missing_target_notification_preserves_source_outcome_without_new_gate(connected, monkeypatch):
    http, provider, frames, fault, driver = connected
    from tests import test_motor_receive_identity
    receive = test_motor_receive_identity.receive
    def missing_y(driver, *, board, motor):
        if (board, motor) != (4, 0):
            receive(driver, board=board, motor=motor)
    monkeypatch.setattr(test_motor_receive_identity, 'receive', missing_y)
    cid, receipt = submit(http, request())
    result = sequence_body(receipt)
    assert receipt['status'] == 'completed', receipt
    assert result['completed_targets'] == 3
    assert result['children'][0]['source_return_ok'] is True
    assert result['children'][0]['controller_terminal_state_verified'] is False
    assert len([f for f in frames if f[1] == 4]) == 6


def test_full_22_target_ledger_survives_canonical_publication(connected):
    http, provider, frames, fault, driver = connected
    targets = [{'x': 1000 + i * 100, 'y': 2000 + i * 100} for i in range(22)]
    cid, receipt = submit(http, request(targets=targets))
    result = sequence_body(receipt)
    assert result['completed_targets'] == 22 and len(result['children']) == 22
    assert [child['target'] for child in result['children']] == targets


def test_existing_durable_owner_queues_next_root_for_whole_sequence(connected, monkeypatch):
    http, provider, frames, fault, driver = connected
    waiting, release = threading.Event(), threading.Event()
    original = driver.motor_wait_target_reached_many
    def wait(*a, **kw):
        waiting.set()
        assert release.wait(3)
        return original(*a, **kw)
    monkeypatch.setattr(driver, 'motor_wait_target_reached_many', wait)
    try:
        admitted = http.post('/operator/v2/actions/oem.xy.move_sequence', json=request())
        assert admitted.status_code == 200, admitted.text
        assert waiting.wait(3)
        competitor = http.post('/operator/v2/actions/oem.xy.move_sequence', json=request('different-root-key'))
        assert competitor.status_code == 200, competitor.text
        assert competitor.json()["command_id"] != admitted.json()["command_id"]
        assert len([f for f in frames if f[1] == 4]) == 2
        replay = http.post('/operator/v2/actions/oem.xy.move_sequence', json=request())
        assert replay.status_code == 200 and replay.json()['command_id'] == admitted.json()['command_id']
    finally:
        release.set()
    cid, receipt = submit(http, request())
    assert receipt['status'] == 'completed'
    cid2, second = submit(http, request('different-root-key'))
    assert cid2 == competitor.json()['command_id'] and second['status'] == 'completed'
    assert len([f for f in frames if f[1] == 4]) == 12


def test_direct_route_cannot_create_another_owner(connected):
    http, provider, frames, fault, driver = connected
    response = http.post('/motion/oem/move_xy_sequence', json=request()['inputs'])
    assert response.status_code == 409
    assert not frames


def test_one_axis_handoffs_wait_and_noop_is_not_fabricated_ack(connected):
    http, provider, frames, fault, driver = connected
    targets = [{'x': 1100, 'y': 2200}, {'x': 1100, 'y': 4400},
               {'x': 3300, 'y': 4400}, {'x': 3300, 'y': 4400}]
    cid, receipt = submit(http, request(targets=targets))
    result = sequence_body(receipt)
    assert receipt['status'] == 'completed' and result['completed_targets'] == 4, receipt
    assert [child['branch'] for child in result['children']] == [
        'parallel', 'near_axis_sequential', 'near_axis_sequential', 'source_noop']
    assert result['children'][-1]['source_noop'] is True
    assert result['children'][-1]['controller_command_acknowledged'] is False
    assert len([f for f in frames if f[1] == 4]) == 4
