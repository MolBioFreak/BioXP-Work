"""F01/F02 against actual native Z provider and real CAN/group receipts.

No failure finally Stop is invented: recovered ControlLib 7887-7902 stops Z on
successful detection, terminates pumps on false wait, and throws otherwise.
"""
import json
import os
from pathlib import Path
import socket
import pytest
from bioxp.pipette.cavro_application import run_application_inline
from tests.test_cavro_application import rig as wire_rig, request
from tests.test_oem_pipette_calibration import rig as native_rig
_SOCKET = socket.socket


@pytest.fixture
def provider(wire_rig, native_rig, monkeypatch):
    monkeypatch.setattr(socket, 'socket', lambda family=socket.AF_INET, *a, **kw:
        _SOCKET(family, *a, **kw) if family == socket.AF_UNIX else (_ for _ in ()).throw(RuntimeError('offline network')))
    p = native_rig.provider
    p.primitives.pipette_transport = wire_rig.group
    p._manual_pipette_receipt_runner = wire_rig.provider._manual_pipette_receipt_runner
    p._wp8_execution_fence_checker = lambda command_id, **_: wire_rig.owner.assert_workflow_current(command_id)
    return p


def run(provider, operations):
    return run_application_inline(provider, request(*operations), command_id='offline-finite-owner',
                                  owner_identity={'source_identity': 'native-review'})


@pytest.mark.parametrize('fault', ['timeout', 'channel', 'exception', 'stop', 'owner_loss', 'termination_exception'])
def test_post_z_detector_failure_source_lifetime(provider, wire_rig, native_rig, monkeypatch, fault):
    original = wire_rig.drivers[1].bus.wait_pipette_completion
    fired = False
    def wait(channel, timeout, **kw):
        nonlocal fired
        if channel == 1 and not fired:
            fired = True
            assert ('z', 60000, False) in native_rig.native.moves
            if fault in {'timeout', 'channel', 'termination_exception'}:
                original(channel, timeout, **kw)  # settle fixture RX owner before injected outcome
                return {'ok': False, 'outcome': 'timeout' if fault != 'channel' else 'device_error', 'oem_error_code': 1 if fault == 'channel' else None}
            if fault == 'exception':
                raise RuntimeError('primary detector exception')
            if fault == 'stop':
                provider.primitives.z_stop()  # explicit existing Stop; not application cleanup
                wire_rig.group._interrupt_epoch += 1
            if fault == 'owner_loss':
                wire_rig.owner.finish_workflow('offline-finite-owner', status='interrupted', payload={}, lifecycle_settled=True)
        return original(channel, timeout, **kw)
    monkeypatch.setattr(wire_rig.drivers[1].bus, 'wait_pipette_completion', wait)
    if fault == 'termination_exception':
        monkeypatch.setattr(wire_rig.group._transports[0], 'terminate', lambda *a, **kw: (_ for _ in ()).throw(RuntimeError('secondary termination exception')))
    source = {'operation': 'plld', 'channels': [0, 1], 'timeout_ms': 30,
              'start_steps': 40000, 'search_target_steps': 60000, 'search_speed_native': 300,
              'z_motor_current': 17, 'after_detection_steps': 41000, 'after_detection_speed_native': 300}
    result = run(provider, [source, {'operation': 'z_move', 'target_steps': 42000, 'speed_native': 300}])
    assert result['ok'] is False
    assert ('z', 60000, False) in native_rig.native.moves
    assert not any(axis == 'z' and position in {41000, 42000} for axis, position, *_ in native_rig.native.moves)
    assert result['plld']['z_search_attempted'] is True
    assert result['plld']['z_stop_attempted'] is False
    assert result['plld']['final_position_steps'] is result['plld']['z_settled'] is None
    if fault == 'exception':
        assert 'primary detector exception' in result['error']
        detection = next(e for e in result['events'] if e['operation'] == 'detect_fluid_level')
        assert detection['partial']['message'] == 'primary detector exception'
        assert [r['channel'] for r in detection['partial']['details']['channels']] == [0, 1]
    if fault in {'timeout', 'channel'}:
        assert [ch for ch, command in wire_rig.wire if command == 'TR'] == [0, 1, 2, 3], result['events'][2].get('result', {}).get('source_timeout_termination')
    if fault == 'termination_exception':
        assert result['error'] == 'Cavro application child failed: detect_fluid_level'
        assert 'secondary termination exception' in repr(result)
    assert any(e[0] == 'stop_z' for e in native_rig.events) == (fault == 'stop')
    if root := os.environ.get('CAVRO_EVIDENCE_ROOT'):
        target = Path(root, 'native-close-exports'); target.mkdir(exist_ok=True)
        (target / ('plld-' + fault + '.json')).write_text(json.dumps({'application': request(source), 'result': result, 'wire': wire_rig.wire, 'motion': native_rig.native.moves}, indent=2))


@pytest.mark.parametrize('body', [{}, {'ok': None}])
def test_untyped_adapter_outcome_is_unknown_not_failed_or_new_gate(provider, native_rig, monkeypatch, body):
    monkeypatch.setattr(provider.primitives, 'z_set_max_speed', lambda value: body)
    result = run(provider, [{'operation': 'z_move', 'target_steps': 40000, 'speed_native': 300}])
    assert result['ok'] and result['source_return_completed']
    assert result['events'][0]['status'] == 'unknown'
    assert result['events'][0]['reported_applied']['controller_outcome'] is None
    assert result['has_unknown_outcomes'] is True
    assert ('z', 40000, True) in native_rig.native.moves
    assert not any(e['status'] == 'failed' for e in result['events'])


def test_source_no_command_success_needs_no_ack(provider, native_rig, monkeypatch):
    # Actual source Stop board-null branch, not a fabricated success.
    monkeypatch.setattr(provider.primitives.tester, '_oem_board_present', lambda board: False, raising=False)
    result = run(provider, [{'operation': 'plld', 'channels': [0], 'timeout_ms': 30,
        'start_steps': 40000, 'search_target_steps': 60000, 'search_speed_native': 300, 'z_motor_current': 17}])
    stop = next(e for e in result['events'] if e['operation'] == 'plld_z_stop')
    assert stop['status'] == 'completed', result
    assert stop['result']['ok'] is True and stop['result']['source_noop']
    assert stop['result']['controller_command_acknowledged'] is False
