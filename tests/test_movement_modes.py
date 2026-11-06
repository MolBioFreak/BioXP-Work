"""Connected movement-mode tests: V2 route -> canonical worker -> provider ->
native adapter. Only the wire exchange is synthetic (existing rig)."""
import time
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from tests.test_wake_setup_debloat import rig, cycle  # noqa: F401


ROUTES = (
    ('/motion/oem/move_xy_sequence', 'motion_oem_move_xy_sequence'),
    ('/motion/oem/y/move_absolute', 'motion_oem_y_move_absolute'),
    ('/motion/oem/x/move_absolute', 'motion_oem_x_move_absolute'),
    ('/motion/oem/xy_retarget', 'motion_oem_xy_retarget'),
)


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
    provider.y_provider = provider.primitives.y_provider
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
    for path, name in ROUTES:
        app.add_api_route(path, getattr(api, name), methods=['POST'])
    monkeypatch.setattr(z_stop_fixtures, 'FastAPI', lambda: app)
    app, _ = z_stop_fixtures.make_app(root, monkeypatch)
    from bioxp import operator_controls
    project = operator_controls.hardware_state.project
    monkeypatch.setattr(operator_controls.hardware_state, 'project',
        lambda *a, include_lifecycle=False, **kw: project(*a, **kw))
    frames.clear()
    with TestClient(app) as http:
        yield http, provider, frames, fault, driver, positions


def body(key, inputs):
    return {'schema_version': 'bioxp.operator_action_request.v2',
            'idempotency_key': key, 'expected_ownership_generation': 7,
            'expected_board_epoch_by_board': {}, 'inputs': inputs}


def submit(http, action, key, inputs):
    response = http.post('/operator/v2/actions/' + action, json=body(key, inputs))
    if response.status_code != 200:
        return None, {'status': 'http_%d' % response.status_code, 'text': response.text}
    cid = response.json()['command_id']
    for _ in range(1500):
        result = http.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
        if result['status'] not in {'queued', 'dispatched', 'running', 'accepted', 'issued_pending'}:
            return cid, result
        time.sleep(.01)
    raise AssertionError(result)


def record_waits(driver, monkeypatch):
    calls = []
    original = driver.motor_oem_move_absolute
    def move(board, position, *a, **kw):
        calls.append((int(board), int(position), kw.get('wait_for_stop', True)))
        return original(board, position, *a, **kw)
    monkeypatch.setattr(driver, 'motor_oem_move_absolute', move)
    return calls


def test_y_absolute_waits_by_default(connected, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    calls = record_waits(driver, monkeypatch)
    cid, receipt = submit(http, 'oem.y.move_absolute', 'y-default-wait-1', {'target_steps': 3000})
    import json
    assert receipt['status'] == 'completed', json.dumps(receipt, default=str)[:6000]
    assert (4, 3000, True) in calls


def test_y_absolute_explicit_false_does_not_wait(connected, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    calls = record_waits(driver, monkeypatch)
    submit(http, 'oem.y.move_absolute', 'y-nowait-1', {'target_steps': 3000, 'wait_for_stop': False})
    assert (4, 3000, False) in calls
    assert not any(b == 4 and w for b, _, w in calls)


def test_x_absolute_optional_wait_input(connected, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    calls = record_waits(driver, monkeypatch)
    cid, receipt = submit(http, 'oem.x.move_absolute', 'x-wait-1', {'position_steps': 4000})
    assert receipt['status'] == 'completed', receipt
    assert (5, 4000, True) in calls


@pytest.mark.parametrize('inputs', [{'target_steps': 1, 'wait_for_stop': 1}, {'target_steps': 1, 'wait_for_stop': 'no'}])
def test_y_invalid_wait_never_moves(connected, inputs):
    http, provider, frames, fault, driver, positions = connected
    submit(http, 'oem.y.move_absolute', 'y-invalid-%s' % type(inputs['wait_for_stop']).__name__, inputs)
    assert not any(c == 4 for b, c, t, m, v in frames)


def bind_deck(http, provider):
    store = http.app.state.operator_command_plane.store
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    return store


def z_key(provider):
    profile = provider.primitives._axis_profile('z')
    return int(profile['board']), int(profile.get('motor', 0))


def test_sequence_named_well_raw_one_z_raise_and_final_publication(connected, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    store = bind_deck(http, provider)
    zb, zm = z_key(provider)
    positions[zb, zm] = 40000
    targets = [{'target': 'LOC_TC', 'camera_offset': False}, {'x': 3300, 'y': 4400},
               {'location_id': 2, 'well': 'B3'}]
    cid, receipt = submit(http, 'oem.xy.move_sequence', 'seq-mixed-1', {'targets': targets})
    assert receipt['status'] == 'completed', receipt
    result = receipt['source_receipt']['xy_sequence']
    assert result['completed_targets'] == 3
    z_moves = [v for b, c, t, m, v in frames if c == 4 and (b, m) == (zb, zm)]
    assert z_moves == [500], z_moves  # one raise, never lowered
    first_z = next(i for i, f in enumerate(frames) if f[1] == 4 and (f[0], f[3]) == (zb, zm))
    first_xy = next(i for i, f in enumerate(frames) if f[1] == 4 and (f[0], f[3]) != (zb, zm))
    assert first_z < first_xy
    xy = [c['resolved_xy'] for c in result['children']]
    assert xy[1] == {'x': 3300, 'y': 4400}
    named = provider._resolve_xy_target(targets[0])
    assert xy[0] == {'x': named['x'], 'y': named['y']}
    assert result['location_publication']['ok'] is True, result['location_publication']
    semantic = store.deck_semantic_state()
    assert (semantic['current_location'], semantic['current_well']) == ('LOC_TC', 14)


def test_sequence_raw_only_keeps_invalidation_and_no_z(connected):
    http, provider, frames, fault, driver, positions = connected
    zb, zm = z_key(provider)
    positions[zb, zm] = 40000
    cid, receipt = submit(http, 'oem.xy.move_sequence', 'seq-raw-1', {'targets': [{'x': 1100, 'y': 2200}]})
    assert receipt['status'] == 'completed', receipt
    assert receipt['source_receipt']['xy_sequence'].get('location_publication') is None
    assert not any(c == 4 and (b, m) == (zb, zm) for b, c, t, m, v in frames)


@pytest.mark.parametrize('targets', [[{'target': 'NOPE', 'camera_offset': False}],
    [{'target': 'LOC_PARK', 'camera_offset': True}], [{'location_id': 2, 'well': 'Z99'}],
    [{'location_id': 2, 'well': 'A1', 'position_flag': 2}], [{'x': 1, 'target': 'LOC_TC', 'camera_offset': False}]])
def test_invalid_union_never_moves(connected, targets):
    http, provider, frames, fault, driver, positions = connected
    cid, receipt = submit(http, 'oem.xy.move_sequence', 'seq-bad-%d' % abs(hash(str(targets))), {'targets': targets})
    assert receipt['status'] != 'completed'
    assert not any(c == 4 for b, c, t, m, v in frames)


def test_retarget_issues_x_then_y_without_wait_replaces_and_next_move_waits(connected, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    calls = record_waits(driver, monkeypatch)
    cid, receipt = submit(http, 'oem.xy.retarget', 'rt-1-key0', {'target': {'x': 20000, 'y': 30000}})
    assert receipt['status'] == 'completed', receipt
    assert receipt['source_receipt']['xy_retarget']['motion_pending'] is True
    assert receipt['source_receipt']['xy_retarget']['resolved_xy'] == {'x': 20000, 'y': 30000}
    assert calls == [(5, 20000, False), (4, 30000, False)]
    acc = [(b, v) for b, c, t, m, v in frames if c == 5 and t == 5]
    assert (5, 400) in acc and (4, 400) in acc
    waits = []
    for name in ('motor_oem_wait_target_reached', 'motor_wait_target_reached'):
        original = getattr(driver, name, None)
        if callable(original):
            def wait(*a, _o=original, _n=name, **kw):
                waits.append((_n, a[0] if a else kw.get('board')))
                return _o(*a, **kw)
            monkeypatch.setattr(driver, name, wait)
    cid2, second = submit(http, 'oem.xy.retarget', 'rt-2-key0', {'target': {'x': 25000, 'y': 35000}})
    assert second['status'] == 'completed'
    assert second['source_receipt']['xy_retarget']['replaced_command_id'] == receipt['command_id'] or \
        second['source_receipt']['xy_retarget']['replaced_command_id']
    assert calls[2:] == [(5, 25000, False), (4, 35000, False)]
    assert not waits  # replacing never awaits
    cid3, normal = submit(http, 'oem.y.move_absolute', 'after-rt-y', {'target_steps': 1000})
    assert normal['status'] == 'completed', normal
    assert {b for _, b in waits} >= {4, 5}
    assert calls[4] == (4, 1000, True)
    assert provider._pending_xy_retarget is None


def test_stop_during_retarget_stops_and_skips_wait(connected):
    http, provider, frames, fault, driver, positions = connected
    cid, receipt = submit(http, 'oem.xy.retarget', 'rt-stop-key0', {'target': {'x': 20000, 'y': 30000}})
    assert receipt['status'] == 'completed'
    stop = provider.execute_x_intent('stop', {'command_id': 'rt-x-stop', 'idempotency_key': 'rt-x-stop',
                                              'defer_reconciliation': True})
    assert any(c == 3 and b == 5 for b, c, t, m, v in frames)
    if callable(stop):
        stop()
    settle = provider.await_pending_xy_retarget()
    assert settle['x'] == {'skipped': 'interrupted'}


def test_retarget_reads_actual_pose_while_moving_without_blocking_next_move(connected, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    tester = provider.primitives.tester
    reads = []
    original = tester.motor_get_position
    def read(board, motor=0):
        reads.append((int(board), int(motor), time.monotonic()))
        return original(board, motor=motor)
    monkeypatch.setattr(tester, 'motor_get_position', read)
    cid, receipt = submit(http, 'oem.xy.retarget', 'pose-rt-0001', {'target': {'x': 20000, 'y': 30000}})
    assert receipt['status'] == 'completed', receipt
    issued = time.monotonic()
    time.sleep(0.7)
    sampled = [r for r in reads if r[2] > issued]
    assert {(5, 0), (4, 0)} <= {(b, m) for b, m, _ in sampled}, reads
    # A following normal move is not held behind the display sampler.
    cid, moved = submit(http, 'oem.xy.move_sequence', 'pose-seq-0001', {'targets': [{'x': 21000, 'y': 31000}]})
    assert moved['status'] == 'completed', moved
    provider._retarget_pose_thread.join(2)
    assert not provider._retarget_pose_thread.is_alive()


def test_retarget_named_target(connected):
    http, provider, frames, fault, driver, positions = connected
    cid, receipt = submit(http, 'oem.xy.retarget', 'rt-named-key0', {'target': {'target': 'LOC_RC', 'camera_offset': True}})
    assert receipt['status'] == 'completed', receipt
    expected = provider._resolve_xy_target({'target': 'LOC_RC', 'camera_offset': True})
    assert receipt['source_receipt']['xy_retarget']['resolved_xy'] == {'x': expected['x'], 'y': expected['y']}


def test_protocol_move_sequence_runs_provider_sequence_in_worker(connected, monkeypatch):
    from contextlib import contextmanager
    from types import SimpleNamespace
    from bioxp import api
    from bioxp.manual_pipetting import compile_manual_pipetting
    from bioxp.protocols.models import ProtocolActionKind
    http, provider, frames, fault, driver, positions = connected
    store = bind_deck(http, provider)
    zb, zm = z_key(provider)
    positions[zb, zm] = 40000
    targets = [{'location_id': 2, 'well': 'A1'}, {'target': 'LOC_RC', 'camera_offset': False},
               {'location_id': 3, 'well': 5}]
    document = compile_manual_pipetting({'protocol_id': 'fast-lane', 'steps': [
        {'operation': 'move_sequence', 'targets': targets}]})
    (action,) = document.stages[0].actions
    assert action.kind is ProtocolActionKind.PIPETTE_POSITION
    calls = []
    class WorkflowStore:
        @contextmanager
        def workflow_context(self, job_id, source_occurrence_id):
            calls.append(('context', job_id, source_occurrence_id)); yield
        def assert_workflow_current(self, job_id):
            calls.append(('current', job_id))
    monkeypatch.setattr(api, '_protocol_command_store', lambda: WorkflowStore())
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    monkeypatch.setattr(api.app.state, 'oem_workflow_plan_executor', lambda *a: (_ for _ in ()).throw(AssertionError('no per-target plan')), raising=False)
    handlers = api._protocol_live_handlers()
    result = handlers[ProtocolActionKind.PIPETTE_POSITION](action, SimpleNamespace(job_id='job-fast-1'))
    assert result['ok'] is True and result['xy_sequence']['completed_targets'] == 3
    assert [v for b, c, t, m, v in frames if c == 4 and (b, m) == (zb, zm)] == [500]
    semantic = store.deck_semantic_state()
    assert (semantic['current_location'], semantic['current_well']) == ('LOC_RC', 5)
    assert calls[0][0] == 'context'


@pytest.mark.parametrize('targets', [[{'x': 1, 'y': 2}], [], [{'location_id': 2, 'well': 'A1', 'position_flag': 1}]])
def test_protocol_move_sequence_rejects_raw_and_bad_targets(targets):
    from bioxp.manual_pipetting import compile_manual_pipetting
    with pytest.raises(Exception):
        compile_manual_pipetting({'protocol_id': 'fast-lane', 'steps': [
            {'operation': 'move_sequence', 'targets': targets}]})
