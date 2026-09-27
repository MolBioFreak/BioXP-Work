"""Real handlers -> dispatch -> finite executor -> SQLite/outbox/workflow.

Reproduce literal exports: PIPETTE_ADDITIONAL_EXPORT=1 pytest -q this_file.
Hardware/configuration fixtures are shared, never their direct execution helpers.
"""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from tests.test_oem_pipette_calibration import rig
from tests.test_manual_source_pipetting_connected import connected, load
from tests.test_oem_diagnostics_connected import diagnostic
from bioxp.manual_pipetting import compile_manual_pipetting, bind_manual_physical_handler, bind_manual_position_handler
from bioxp.oem_deck_movement import make_wp8_operation_executor
from bioxp.operator_command_plane import OperatorCommandPlane
from bioxp.operator_controls import _workflow_terminal_result, _bounded_json
from bioxp.protocols.executor import ProtocolExecutor
from bioxp.protocols.models import ProtocolActionKind

EXPORT = Path(__file__).parents[1] / 'testdata/pipette_completion/additional-results.json'
LATEST = {}


def dispatcher(rig, receipts, parent):
    store, provider = rig.store, rig.provider
    admission = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1},
        "board4_authority": {"active_board_epoch": 1}}}
    plane = object.__new__(OperatorCommandPlane)
    plane.store = store
    plane.machine_state_provider = lambda: admission
    plane.app = SimpleNamespace(state=SimpleNamespace(oem_deck_provider=provider,
        oem_wp8_operation_executor=make_wp8_operation_executor(
            provider_getter=lambda: provider, command_store=store)))
    records = []

    def execute(plan, action, state):
        admitted = store.admit_internal_wp8_operation(plan['operation'], inputs={}, state=admission,
            idempotency_key=f'{parent}:{len(records)}:{action.action_id}', prepared_plan=plan)
        claimed = store.claim_next()
        assert claimed['command_id'] == admitted['command_id']
        assert store._renew_owner(lease_seconds=120)
        plane._dispatch_one(claimed)
        receipt = store.get_command(admitted['command_id'])
        native = _workflow_terminal_result(receipt)
        if 'pipette_result' not in native:
            assert native['ok'], native
            return native
        critical = native['pipette_result']
        outbox = [json.loads(row[0]) for row in store.connection.execute(
            'SELECT payload_json FROM operator_plane_outbox WHERE command_id=?',
            (admitted['command_id'],))]
        assert any(_bounded_json(row, 1).get('pipette_result') == critical for row in outbox)
        assert _bounded_json({'response': native, 'bulk': 'x' * 200000}, 131072)['pipette_result'] == critical
        records.append(critical)
        return native

    handler = bind_manual_physical_handler(command_store=store, execute_plan=execute,
        require_motion_ready=lambda: None, provider_getter=lambda: provider,
        receipt_store_getter=lambda: receipts)

    def run(steps):
        doc = compile_manual_pipetting({'protocol_id': parent, 'steps': steps})
        state = ProtocolExecutor(dry_run=False, job_id=parent,
            before_native_entry=lambda *args: store.assert_workflow_current(parent),
            handlers={ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL: handler,
                ProtocolActionKind.PIPETTE_POSITION: bind_manual_position_handler(command_store=store,
                    execute_plan=execute, require_motion_ready=lambda: None)}).execute(doc)
        payload = {'execution': {'runtime_state': state.to_payload()}}
        stored = store.publish_workflow(parent, payload=payload)
        rows = stored['execution']['runtime_state']['action_results']
        assert rows[-1]['pipette_result'] == records[-1], rows[-1]
        LATEST.clear()
        LATEST.update(action_result=rows[-1], request={'protocol_id': parent, 'steps': steps})
        return records[-1]
    return run


def export(name, result):
    if os.environ.get('PIPETTE_ADDITIONAL_EXPORT'):
        EXPORT.parent.mkdir(parents=True, exist_ok=True)
        data = json.loads(EXPORT.read_text()) if EXPORT.exists() else {'cases': []}
        data['cases'] = [row for row in data['cases'] if row['name'] != name]
        data['cases'].append({'name': name, **LATEST})
        EXPORT.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')


@pytest.mark.parametrize('action', ['aspirate', 'dispense', 'dispense_all', 'diagnoses',
    'initialize', 'eject', 'get_data', 'last_error', 'plunger_up', 'plunger_down'])
def test_diagnostic_dispatch(diagnostic, rig, monkeypatch, action):
    _, receipts, _ = diagnostic
    for d in rig.drivers:
        original = d.wait_pipette_command_completion
        monkeypatch.setattr(d, 'wait_pipette_command_completion',
            lambda timeout, owner_token, f=original, d=d: {**f(timeout, owner_token=owner_token),
                'pipette_message_state': {'diagnosis': f'#{d.channel} `passed'}})
    monkeypatch.setattr(rig.native, 'motor_z_move_relative_strict',
        lambda steps, **kw: {'ok': True, 'command_sent': True}, raising=False)
    args = ({'channels': [0, 2], 'volume_ul': 10., 'speed': 100} if action in {'aspirate','dispense'}
        else {'channels': [1,3]} if action == 'eject'
        else {'steps': 123} if action.startswith('plunger_') else {})
    result = dispatcher(rig, receipts, 'diagnostic-parent')([
        {'operation': 'diagnostic_pipette', 'diagnostic': {'action': action, **args}}])
    assert result['kind'] == 'diagnostic_pipette' and result['action'] == action
    assert result['ok']
    if action == 'diagnoses':
        assert [t['number'] for t in result['tests']] == [0,1,2]
        assert result['tests'][0]['channels'][2]['display'] == '#2passed'
    if action == 'get_data':
        assert [c['part_number'] for c in result['channels']] == ['PART'] * 4
        assert all(c['data']['channels'][0]['results'][0]['value'] == 'data' for c in result['channels'])
    if action == 'last_error':
        assert [c['display'] for c in result['channels']] == ['0x10','0x11','0x12','0x13']
    if action.startswith('plunger_'):
        assert result['events'][0]['result']['value'] == 31
        assert result['events'][1]['result']['requested_steps'] == (-123 if action == 'plunger_up' else 123)
    export(action, result)


def manual_dispatch(c):
    parent = 'additional-manual'
    c.store.admit_workflow(command_id=parent, idempotency_key=parent, plan_fingerprint=parent,
        requested_inputs={}, ownership_generation=1,
        resources=('axis:x','axis:y','axis:z','pipette'), board_epochs={})
    c.store.claim_next()
    assert c.store._renew_owner(lease_seconds=120)
    for index in range(5):
        c.provider.publish_tip_tray_transition(tray_id=index, transition='construct',
            operation_id=f'{parent}:construct:{index}', command_id=parent,
            provenance={'source_operation': 'ClassMachineStatus.constructor'})
    return dispatcher(c, c.receipts, parent)


def test_selected_and_matching_dispatch(connected):
    run = manual_dispatch(connected)
    result = run([load(2)])
    assert result['requested_pipette'] == result['tip_location'] == 2
    assert result['alignment_published'] and result['source_return']
    export('selected_load', result)
    result = run([load(1, False)])
    assert result['requested_pipette'] == 1 and result['tip_location'] == 2
    assert result['already_matching_tip_type'] and not result['alignment_published']
    export('matching_load', result)


def test_failed_reload_dispatch(connected):
    run = manual_dispatch(connected)
    assert run([load(2)])['ok']
    connected.native.fail_at = ('z', 90000)
    result = run([load(1)])
    assert not result['ok'] and result['tip_location'] == -1
    assert not result['source_return_completed'] and 'source_return' not in result
    assert result['source_errors']
    export('failed_reload', result)


@pytest.mark.parametrize('operation', ['source_mix','source_aspirate_air','source_dispense_air','source_purge'])
def test_manual_outcomes_dispatch(connected, monkeypatch, operation):
    for d in connected.drivers:
        monkeypatch.setattr(d, 'query_pressure', lambda: {'ok': True, 'semantic_ok': True, 'pressure': 1.}, raising=False)
        monkeypatch.setattr(d, 'dispense_air', lambda v, d=d, **kw: d.issue('dispense_air', volume=v, **kw), raising=False)
    run = manual_dispatch(connected)
    assert run([load(-1)])['ok']
    step = {'operation': operation}
    if operation != 'source_purge':
        step['volume_ul'] = 20. if operation == 'source_mix' else 5.
    if operation == 'source_mix':
        step.update(air_ul=0., aspirate_delay_ms=0, dispense_delay_ms=0, cycles=1)
    result = run([{'operation': 'move', 'location_id': 3, 'well': 'A2', 'position_flag': 1}, step])
    assert result['kind'] == operation and result['ok']
    assert result['native_results']
    export(operation, result)


def test_partial_diagnosis_dispatch(diagnostic, rig, monkeypatch):
    def fail(number, **kwargs):
        if number == 1:
            raise RuntimeError('injected diagnose transport failure')
        return rig.drivers[2].issue('diagnoses', number=number, **kwargs)
    monkeypatch.setattr(rig.drivers[2], 'execute_diagnoses', fail)
    result = dispatcher(rig, diagnostic[1], 'diagnostic-parent')([
        {'operation': 'diagnostic_pipette', 'diagnostic': {'action': 'diagnoses'}}])
    assert not result['ok'] and len(result['tests']) == 1
    assert 'injected diagnose transport failure' in str(result['source_errors'])
    export('partial_diagnosis', result)


def test_initialize_retry_and_ignored_failure_dispatch(diagnostic, rig, monkeypatch):
    for driver in rig.drivers:
        monkeypatch.setattr(driver, 'query_status', lambda: {'ok': False, 'oem_error_code': 5})
        monkeypatch.setattr(driver, 'wait_pipette_initialization_completion', lambda timeout: {'ok': False})
    result = dispatcher(rig, diagnostic[1], 'diagnostic-parent')([
        {'operation': 'diagnostic_pipette', 'diagnostic': {'action': 'initialize'}}])
    assert result['ok'] and result['source_return_completed']
    assert not result['controller_outcome_ok']
    assert [a['attempt'] for a in result['attempts']] == ['initial', 'retry']
    assert all(not a['status']['ok'] for a in result['attempts'])
    export('initialize_retry_ignored_failure', result)


@pytest.mark.parametrize('action', ['aspirate', 'dispense'])
def test_lost_tip_selection_dispatch(diagnostic, rig, monkeypatch, action):
    original = rig.drivers[1].query_tip_status
    monkeypatch.setattr(rig.drivers[1], 'query_tip_status', lambda:
        {**original(), 'tip_loaded': False, 'source_tip_loaded': False, 'source_return': 0})
    result = dispatcher(rig, diagnostic[1], 'diagnostic-parent')([
        {'operation': 'diagnostic_pipette', 'diagnostic': {
            'action': action, 'channels': [0,1,2], 'volume_ul': 10., 'speed': 100}}])
    assert result['lost_tip_channels'] == [1]
    assert result['selected_channels'] == ([0,1,2] if action == 'aspirate' else [0,2])
    export('lost_tip_' + action, result)
