"""Captured OEM Z no-op replay through the finite sourceMoveZ binding."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.test_deck_scoped_authority import rig  # noqa: F401

CAPTURE = Path('/home/dalab/.hermes/profiles/fresh/robot-audit/deck-command-audit/'
               'oem-live-acceptance/live-preparation/close4-critical-images-failure.json')


def captured():
    response = json.loads(CAPTURE.read_text())['body']['terminal_evidence']['response']
    failed = response['provider_results'][0]
    assert failed['ok'] is False
    assert failed['hardware_postcondition_verified'] is False
    assert failed['execution']['ok'] is True
    return failed['execution']['execution_results'][0]['results'][0]['result']


def invoke(rig, native, current):
    provider, _, semantic, state = rig
    semantic['pseudo_z_home'] = 500
    state['machine_status']['psudo_z_home_steps'] = 500
    calls = []
    def set_current(board, param, value, **kwargs):
        calls.append(('current', board, param, value, kwargs))
        return copy.deepcopy(current)
    def move(board, target, **kwargs):
        calls.append(('move', board, target, kwargs))
        return copy.deepcopy(native)
    primitive = object.__new__(Serial206ProductionPrimitiveAdapter)
    primitive.tester = SimpleNamespace(motor_set_axis_param=set_current,
                                      motor_oem_move_absolute=move)
    primitive._axis_profile = lambda axis: {'board': 4, 'motor': 1, 'axis_max_steps': 160000}
    provider.primitives = primitive
    result = provider.execute_wp8_child(
        {'operation': 'sourceMoveZ', 'arguments': {'value': -4696}, 'order': 0},
        command_id='captured-z-noop', child_order=0, plan_digest='captured-z-plan')
    return result, calls


def test_captured_effective_noop_through_native_adapter_and_finite_binding(rig):
    raw = captured()
    result, calls = invoke(rig, raw['move'], raw['current_set'])
    assert result['ok'] is True
    assert result['hardware_postcondition_verified'] is True
    assert result['controller_completion_verified'] is True
    assert result['controller_command_acknowledged'] is False
    # Current=31 was really called; no claim that the entire source call did no I/O.
    assert result['delivery_attempted'] is True
    assert calls == [('current', 4, 6, 31, {'motor': 1}),
                     ('move', 4, 500, {'motor': 1, 'wait_for_stop': True, 'max_position': 160000})]
    step = result['execution']['execution_results'][0]
    wrapper = step['results'][0]['result']
    assert step['source_step']['z'] == -4696
    assert step['results'][0]['target_position'] == -4696
    assert (wrapper['requested'], wrapper['effective'], wrapper['pseudo_z_home']) == (-4696, 500, 500)
    native = wrapper['move']
    assert native['source_return_code'] == 500
    assert native['requested_position'] == native['raw_requested_position'] == 500
    assert native['command_sent'] is False
    assert native['ack'] is None and native['event'] is None
    assert native['terminal_proof'] is None


@pytest.mark.parametrize('case', [
    'missing_native', 'native_error', 'missing_before', 'invalid_reply',
    'error_reply', 'missing_position', 'wrong_position', 'bool_position',
    'missing_target', 'bool_target', 'missing_noop', 'sent_command', 'error_ack',
])
def test_source_move_z_missing_invalid_or_error_evidence_stays_rejected(rig, case):
    raw = captured()
    native = raw['move']
    if case == 'missing_native':
        native = None
    elif case == 'native_error':
        native['ok'] = False
    elif case == 'missing_before':
        native.pop('before')
    elif case == 'invalid_reply':
        native['before']['position_reply_valid'] = False
    elif case == 'error_reply':
        native['before']['ok'] = False
        native['before']['ack'] = {'status': 4}
    elif case == 'missing_position':
        native['before'].pop('position')
    elif case == 'wrong_position':
        native['before']['position'] = 499
    elif case == 'bool_position':
        native['before']['position'] = True
    elif case == 'missing_target':
        native.pop('requested_position')
    elif case == 'bool_target':
        native['requested_position'] = True
    elif case == 'missing_noop':
        native.pop('source_noop')
    elif case == 'sent_command':
        native['command_sent'] = True
    elif case == 'error_ack':
        native['ack'] = {'status': 4}
    result, _ = invoke(rig, native, raw['current_set'])
    assert result['ok'] is False
    assert result['hardware_postcondition_verified'] is False
    assert result['controller_command_acknowledged'] is False
