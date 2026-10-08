"""Crash follow-up: real provider, dispatcher and SQLite; hardware-only leaves."""
import time

from tests.protocol_v1_integration_fixture import NativePhysicalRecorder

import pytest
from fastapi.testclient import TestClient
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.operator_controls import _route_failure_code
from tests.test_deck_complete_effective import BoardLeaf
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained, catalog_action


class UnmovingZ(BoardLeaf):
    """Simulated controller accepts Z absolute but never reaches its target."""
    def __init__(self):
        super().__init__((1000, 1000))
        self.positions[4, 1] = 14336

    def _send_motor(self, board, command, typ, motor, target, **kwargs):
        if (board, motor) == (4, 1):
            assert (command, typ) == (4, 0)
            self.moves.append((board, motor, target))
            return {'status': 100}
        return super()._send_motor(board, command, typ, motor, target, **kwargs)

    def motor_oem_wait_target_reached(self, board, motor=0, **kwargs):
        if (board, motor) == (4, 1):
            return {'ok': False, 'target_reached': False,
                    'source_wait_signaled': False, 'event': None, 'events': [],
                    'failure': 'oem_moveToAbs_target_event_timeout',
                    'elapsed_ms': 20000, 'no24v': False}
        return super().motor_oem_wait_target_reached(board, motor, **kwargs)


@pytest.mark.parametrize('target', ['LOC_RC', 'LOC_RC_COVER_STORAGE', 'WASTE_BIN'])
def test_named_z_timeout_is_not_a_history_refusal(installed_retained, monkeypatch, target):
    app, provider, observations, references, root = installed_retained
    plane = app.state.operator_command_plane
    leaf = UnmovingZ()
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None,
        authority_provider=lambda: {}, generation_provider=provider.generation_provider,
        reference_store=references)
    monkeypatch.setattr(observations, 'oem_move_to', adapter.oem_move_to)
    client = TestClient(app)
    action = catalog_action(app)
    assert action['enabled']
    before = references.snapshot(('x', 'y', 'z', 'g'))
    body = {'schema_version': 'bioxp.operator_action_request.v2',
            'idempotency_key': 'z-timeout-' + target,
            'expected_ownership_generation': int(provider.generation_provider()),
            'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
            'inputs': {'target': target, 'camera_offset': False}}
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert response.status_code == 200, response.text
    cid = response.json()['command_id']
    plane.start()
    compact = None
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        compact = client.get('/operator/v2/actions/receipts/' + cid).json()
        if compact['terminal']:
            break
        time.sleep(.01)
    assert compact is not None and compact['terminal'], compact
    detail = client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
    assert detail['status'] == 'ambiguous', detail
    stages = detail['deck_movement']['stages']
    assert [s['terminal_state'] for s in stages[:3]] == ['completed'] * 3
    failed = stages[3]['terminal_evidence']['provider_evidence']
    assert failed['board'] == 4 and failed['motor'] == 1
    assert failed['wire_position'] == 500
    assert failed['before']['position'] == failed['timeout_position']['position'] == 14336
    assert failed['ack']['status'] == 100 and failed['source_return_code'] == 0
    assert failed['wait']['failure'] == 'oem_moveToAbs_target_event_timeout'
    assert leaf.moves == [(4, 1, 500)]  # no XY, retry, Home or reset
    assert references.snapshot(('x', 'y', 'z', 'g')) == before
    assert compact['error']['retryable'] is False  # no automatic replay permission
    assert compact['error']['message'] == (
        'Action outcome is uncertain; inspect the exact receipt and controller state. '
        'No automatic retry was performed.')
    plane.stop()
    # Retained ambiguity is evidence, not an admission gate for another intent.
    body['idempotency_key'] += '-next'
    next_response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert next_response.status_code == 200, next_response.text
    assert next_response.json()['command_id'] != cid


def test_actual_z_home_exception_prefix_has_existing_timeout_diagnostic():
    response = {'detail': {'ok': False, 'result': {
        'ok': False, 'error': 'OemMotionCompletionError: Reach GZ position time out! board=4; axis=1; position=10000'}}}
    assert _route_failure_code(409, response) == 'controller_position_wait_timeout'
    # Do not classify arbitrary exception prose or expose it as a diagnostic.
    response['detail']['result']['error'] += '; private prose'
    assert _route_failure_code(409, response) == 'route_http_conflict'


def test_manual_home_retains_native_preliminary_move_timeout(retained_rig, monkeypatch):
    provider, observations, runtime, references, store, root = retained_rig
    native = NativePhysicalRecorder(monkeypatch)
    native.positions = {(4, 0): 1000, (4, 1): 14336, (4, 2): 0, (5, 0): 1000}
    native.door_open_position = 10000
    native.replies[4, 138, 0, 1] = {'status': 100, 'value': 0}
    native.replies[4, 4, 0, 1] = {'status': 100, 'value': 0}
    adapter = Serial206ProductionPrimitiveAdapter(native.tester, None,
        authority_provider=lambda: {}, generation_provider=provider.generation_provider,
        reference_store=references)
    monkeypatch.setattr(observations, 'z_manual_home', adapter.z_manual_home, raising=False)
    try:
        result = provider.execute_z_intent('manual_home', expected_generation=3,
            idempotency_key='native-manual-timeout')
        assert result['ok'] is False
        evidence = result['result']['motion_evidence']
        assert evidence['home_stage'] == 'preliminary_rehome_move'
        assert evidence['homing_sweep_started'] is False
        assert evidence['wire_position'] == 10000
        assert evidence['before']['position'] == evidence['timeout_position']['position'] == 14336
        assert evidence['ack']['status'] == 100
        assert not any(frame[:4] == (4, 2, 0, 1) for frame in native.trace)
        # Real canonical provider state and receipt persist the attached native
        # failure; neither a controller home nor physical success is invented.
        receipt = provider._durable_serial206_receipt_by_idempotency('z', 'native-manual-timeout')
        assert receipt['result']['motion_evidence'] == evidence
        assert receipt['status'] == 'failed'
    finally:
        native.close()


def test_diagnostic_home_axis_keeps_counter_reset_and_597_branch(retained_rig, monkeypatch):
    provider, observations, runtime, references, store, root = retained_rig
    native = NativePhysicalRecorder(monkeypatch)
    native.positions = {(4, 0): 1000, (4, 1): 14336, (4, 2): 0, (5, 0): 1000}
    native.door_open_position = 10000
    switches = iter((0, 1))
    native.replies[4, 6, 9, 1] = lambda: {'status': 100, 'value': next(switches)}
    def search():
        native.positions[4, 1] = 0
        return {'status': 100, 'value': 0}
    native.replies[4, 2, 0, 1] = search
    adapter = Serial206ProductionPrimitiveAdapter(native.tester, None,
        authority_provider=lambda: {}, generation_provider=provider.generation_provider,
        reference_store=references)
    monkeypatch.setattr(observations, 'z_diagnostic_home_axis', adapter.z_diagnostic_home_axis, raising=False)
    try:
        result = provider.execute_z_intent('diagnostic_home_axis', expected_generation=3,
            idempotency_key='native-diagnostic-search')
        assert result['ok'], result
        reset = native.trace.index((4, 5, 1, 1, 0))
        sweep = native.trace.index((4, 2, 0, 1, 597))
        assert reset < sweep
        assert not any(frame[:4] == (4, 4, 0, 1) for frame in native.trace)
        assert result['result']['source_method'] == 'ClassControlInterface.HomeAxis(z) -> axisSearchHome(597)'
        assert provider._durable_serial206_receipt_by_idempotency('z', 'native-diagnostic-search')['status'] == 'completed'
    finally:
        native.close()
