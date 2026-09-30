"""R4 exact retained receipts, with connected real producer regressions."""
import copy
import json
import os
import sqlite3
from pathlib import Path
from types import SimpleNamespace

import pytest
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.test_deck_scoped_authority import rig, retained_rig  # noqa: F401
from tests.test_moveto_x_noop_connected import exact_native  # noqa: F401
from tests.test_cover_carry_release_connected import connected  # noqa: F401

DB = Path(os.environ.get(
    'BIOXP_R4_RETAINED_CAPTURE',
    '/home/dalab/.hermes/profiles/fresh/robot-audit/debloat-plan-20260929/implementation/finish/live-retained/bioxp_runtime.db',
))


def receipt(command, table='operator_plane_wp8_children'):
    with sqlite3.connect(f'file:{DB}?mode=ro', uri=True) as connection:
        suffix = " AND operation='parkGantry'" if table == 'operator_plane_deck_stages' else " AND terminal_state NOT IN ('completed','planned')"
        row = connection.execute(f'SELECT terminal_evidence_json FROM {table} WHERE command_id=?{suffix}', (command,)).fetchone()
    assert row is not None
    return json.loads(row[0])


def test_exact_retained_catch_z_noop_through_adapter_and_finite_binding(rig):
    failed = receipt('b64f855e-8935-4f68-9ff0-352cf15a6576')['result']
    assert failed['failed_child'] == 'scriptmoveTo'
    child = failed['failure_evidence'][0]['result']
    assert child['ok'] is False and child['execution']['ok'] is True
    raw = child['execution']['execution_results'][0]['results'][0]['result']
    assert raw['move']['before']['position'] == raw['effective'] == 65000
    provider, _, semantic, state = rig
    semantic['pseudo_z_home'] = 65000
    state['machine_status']['psudo_z_home_steps'] = 65000
    calls = []
    def current(*args, **kwargs):
        calls.append(('current', args, kwargs))
        return copy.deepcopy(raw['current_set'])
    def move(*args, **kwargs):
        calls.append(('move', args, kwargs))
        return copy.deepcopy(raw['move'])
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = SimpleNamespace(motor_set_axis_param=current, motor_oem_move_absolute=move)
    adapter._axis_profile = lambda axis: {'board': 4, 'motor': 1, 'axis_max_steps': 160000}
    provider.primitives = adapter
    result = provider.execute_wp8_child({'operation': 'sourceMoveZ', 'arguments': {'value': 65000}, 'order': 0}, command_id='r4-retained-z-noop', child_order=0, plan_digest='r4-retained')
    assert result['ok'] is True
    assert result['controller_completion_verified'] is True
    assert result['controller_command_acknowledged'] is False
    assert [call[0] for call in calls] == ['current', 'move']
    assert calls[1][1] == (4, 65000)


@pytest.mark.parametrize('command', ['31137ac2-ddb1-4b6f-b064-bbf611ed5b36', '3e2097a9-7df7-444c-bcf9-99e5742c0bf8'])
def test_exact_release_duplicate_y_noop_connected(exact_native, command):
    failure = receipt(command)['result']
    raw = failure['failure_evidence'][0]['result']['execution']['execution_results'][0]['results'][0]['result']
    assert raw['branch'] == 'descending_y_loaded_plate'
    assert raw['operations'][-1]['controller_command_required'] is False
    assert raw['controller_child_evidence'][-1]['command_required'] is True
    rig = exact_native
    rig.native.positions.update({(5, 0): raw['before']['x'], (4, 0): raw['before']['y'], (4, 1): raw['before']['z']})
    target = raw['target']
    result = rig.provider.primitives.oem_move_to(target['x'], target['y'], target['z'], pseudo_home_steps=500, gripper_confirmed=False, tip_loaded=False, plate_on_gantry=5, location19_y=44972, run_in_parallel=True)
    assert result['branch'] == raw['branch']
    assert result['controller_completion_verified'] is True
    assert result['controller_child_evidence'][-1] == {'command_required': False, 'acknowledged': False, 'terminal': True}
    assert result['operations'][-1]['target'] == 44972


def test_exact_retained_x_noop_connected(exact_native):
    failed = receipt('f8a77cd6-6068-4464-ba17-0a1a992b7d6e')['result']
    raw = failed['failure_evidence'][0]['result']['execution']['execution_results'][0]['results'][0]['result']
    assert raw['controller_child_evidence'][0] == {'command_required': False, 'acknowledged': False, 'terminal': False}
    rig = exact_native
    rig.native.positions.update({(5, 0): 84252, (4, 0): 36267, (4, 1): 65000})
    result = rig.provider.primitives.oem_move_to(84252, 6057, 65000, pseudo_home_steps=65000, gripper_confirmed=False, tip_loaded=False, run_in_parallel=True)
    assert result['branch'] == raw['branch']
    assert result['controller_completion_verified'] is True
    assert result['controller_child_evidence'][0] == {'command_required': False, 'acknowledged': False, 'terminal': True}


@pytest.mark.parametrize('command,actual', [('255e691c-14f7-470f-97de-fd70e221c5ae', 67110), ('a568d2ad-b233-4db7-b542-259a4475d287', 113641)])
def test_retained_park_head_timeout_source_equivalence(command, actual):
    raw = receipt(command, 'operator_plane_deck_stages')['provider_evidence']
    assert (raw['board'], raw['motor'], raw['requested_position'], raw['wire_position']) == (4, 1, 114092, 114092)
    assert raw['ack']['status'] == 100
    assert raw['wait']['elapsed_ms'] == 20004
    assert raw['wait']['source_wait_signaled'] is False
    assert raw['wait']['events'] == []
    assert raw['timeout_position']['position'] == actual
    assert raw['timeout_position']['position_reply_valid'] is True
    assert actual != raw['requested_position']
