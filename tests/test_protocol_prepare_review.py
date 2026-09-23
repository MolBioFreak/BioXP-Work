"""Offline review/producer-consumer tests. HW and inspection doubles labelled.

Canonical tests retain actual HTTP, SQLite owner, child dispatch and reopen;
inspection output is an explicit double, not real CV or live readiness evidence.
"""
import copy
import hashlib
from threading import Event

import pytest

from tests.test_protocol_oem_lifecycle import document, engine, start, finish, wait_until
from bioxp.protocols.runtime_state import ProtocolSourceModel, SourceTray, SourceWell
from bioxp.services.protocol_service import (
    _build_live_execution_contract, _pending_review_payload, ProtocolLiveContractError,
)
from bioxp.services.protocol_service import compile_protocol_source
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_v1_integrated_lifecycle import assert_reopened, hooks


def output(tmp_path, warning=True):
    # Retained physical-image DOUBLE, never an actual observation claim.
    path = tmp_path / 'inspection-double.jpg'
    from PIL import Image
    Image.new('RGB', (8, 8), 'red').save(path)
    content = path.read_bytes()
    inspections = [{'stage': name, 'source_status': 'OK', 'effective_status': 'OK'}
                   for name in ('purification', 'recovery', 'trough', 'handle', 'wells', 'tips')]
    if warning:
        inspections[2].update(source_status='TROUGH_MISSING', effective_status='TROUGH_MISSING')
    issues = ['trough'] if warning else []
    return {'ok': True, 'source_return': 'OK', 'inspections': inspections,
            'inspection_issues': issues, 'inspection_decision_required': warning,
            'offline_inspection_double': True,
            'deck_manifest': {'physical_observations': False, 'inspections': copy.deepcopy(inspections),
                'inspection_issues': issues, 'artifacts': [{'path': str(path),
                    'sha256': hashlib.sha256(content).hexdigest(), 'size_bytes': len(content)}]}}


def prepared_document():
    # Explicit synthetic source model for scheduler qualification only.
    model = ProtocolSourceModel(logical_tip_present=False, carried_plate_present=False,
        tip_trays=[SourceTray(tray_id=str(i), location=7+i,
            wells=[SourceWell(content=None, volume=0, capacity=0, empty=True) for _ in range(96)]) for i in range(4)])
    return document('step', oem_prepare=True, source_settings={'DeckInspection': True},
                    source_model=model.to_payload())


def test_warning_ignore_is_explicit_and_never_reprepares(tmp_path):
    doc, result = prepared_document(), output(tmp_path)
    original = copy.deepcopy(doc.to_payload())
    calls = []
    executor, trace = engine(doc, overrides={'prepare': lambda state: (calls.append('prepare') or result)})
    run = start(executor, doc)
    try:
        wait_until(lambda: executor._state and executor._state.awaiting_review)
        assert trace == [] and calls == ['prepare']
        assert not executor._source_entered
        pending = _pending_review_payload(executor._state)
        assert pending['stage_id'] == pending['action_id'] == 'lifecycle:prepare'
        assert pending['decisions'] == ['abort', 'ignore']
        with pytest.raises(ValueError, match='explicit'):
            executor.acknowledge_review(control_id='generic', gate_id='lifecycle:prepare')
        with pytest.raises(ValueError):
            executor.request_control('continue', control_id='wrong', gate='review', gate_id='lifecycle:prepare')
        executor.acknowledge_review(control_id='ignore', gate_id='lifecycle:prepare', decision='ignore')
        state = finish(run)
        assert state.completed and calls == ['prepare']
        assert trace[0] == 'run_job'
        assert doc.to_payload() == original
        assert next(row for row in state.action_results if row.get('hook') == 'prepare')['inspection_decision_required'] is True
    finally:
        if run[0].is_alive():
            executor.interrupt(control_id='test-teardown')
            finish(run)


@pytest.mark.parametrize('control', ['review-abort', 'abort', 'safe_stop'])
def test_preparation_abort_only_unlocks_without_source_entry(tmp_path, control):
    doc, result = prepared_document(), output(tmp_path)
    executor, trace = engine(doc, overrides={'prepare': lambda state: result})
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.awaiting_review)
    if control == 'review-abort':
        executor.acknowledge_review(control_id='abort', gate_id='lifecycle:prepare', decision='abort')
    else:
        executor.request_control(control, control_id='abort')
    state = finish(run)
    assert executor.outcome == 'interrupted'
    assert trace == ['preparation_abort']
    assert state.workflow.reached_control_id == 'abort'
    assert not executor._source_entered and not executor._source_return_notified


@pytest.mark.parametrize('defect', ['failed', 'missing', 'images', 'identity', 'physical', 'observations'])
def test_failed_or_missing_manifest_never_enters_runjob(tmp_path, defect):
    doc, result = prepared_document(), output(tmp_path, warning=False)
    if defect == 'failed':
        result['ok'] = False
    elif defect == 'missing':
        del result['deck_manifest']
    elif defect == 'images':
        result['deck_manifest']['artifacts'] = []
    elif defect == 'identity':
        result['deck_manifest']['artifacts'][0]['sha256'] = 'missing-identity'
    elif defect == 'physical':
        result['deck_manifest']['physical_observations'] = True
    else:
        result['deck_manifest']['inspections'] = []
    executor, trace = engine(doc, overrides={'prepare': lambda state: result})
    state = executor.execute(doc)
    assert executor.outcome == 'failed'
    assert trace == [] and not state.awaiting_review
    assert not executor._source_entered


def test_successful_producer_without_warning_enters_runjob_once(tmp_path):
    doc, result = prepared_document(), output(tmp_path, warning=False)
    executor, trace = engine(doc, overrides={'prepare': lambda state: result})
    state = executor.execute(doc)
    assert state.completed and trace.count('run_job') == 1
    assert 'preparation_abort' not in trace


def test_manifest_does_not_replace_required_tip_inventory(tmp_path):
    doc, result = prepared_document(), output(tmp_path, warning=False)
    def prepare(state):
        state.source_model.tip_trays.clear()
        return result
    executor, trace = engine(doc, overrides={'prepare': prepare})
    state = executor.execute(doc)
    assert executor.outcome == 'failed' and not trace
    assert any('tip-tray wells' in str(event) for event in state.events)


def test_image_log_loss_does_not_add_a_gate_after_explicit_ignore(tmp_path):
    doc, result = prepared_document(), output(tmp_path)
    executor, trace = engine(doc, overrides={'prepare': lambda state: result})
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.awaiting_review)
    (tmp_path / 'inspection-double.jpg').unlink()
    executor.acknowledge_review(control_id='ignore', gate_id='lifecycle:prepare', decision='ignore')
    finish(run)
    assert executor.outcome == 'completed' and 'run_job' in trace


def test_ignore_rechecks_the_producer_acquisition_identity(tmp_path):
    doc, result = prepared_document(), output(tmp_path)
    executor, trace = engine(doc, overrides={'prepare': lambda state: result})
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.awaiting_review)
    result['deck_manifest']['artifacts'][0]['sha256'] = 'invalid'
    executor.acknowledge_review(control_id='ignore', gate_id='lifecycle:prepare', decision='ignore')
    finish(run)
    assert executor.outcome == 'failed' and not trace


def test_cancel_in_prepare_drains_entered_producer_without_pressure_or_cleanup(tmp_path):
    doc, result = prepared_document(), output(tmp_path, warning=False)
    entered, release = Event(), Event()
    def prepare(state):
        entered.set()
        assert release.wait(4)
        return result
    executor, trace = engine(doc, overrides={'prepare': prepare})
    run = start(executor, doc)
    assert entered.wait(3)
    executor.request_control('abort', control_id='abort')
    assert run[0].is_alive() and not trace
    release.set()
    finish(run)
    assert executor.outcome == 'interrupted' and not trace
    assert not executor._source_entered


def contract_payload(prepare=True, inspection=True):
    doc = document('step', oem_prepare=prepare, source_settings={'DeckInspection': inspection})
    return {'document': doc.to_payload(), 'live_execution': {'operator_id': 'offline-reviewer',
        'live_execution_ack': True, 'physical_console_verified': True,
        'artifact_refs': ['offline-explicit-preflight'],
        'reference_snapshot': {'rows': {axis: {'state': 'referenced'} for axis in ('x','y','z','g')}}}}


@pytest.mark.parametrize('prepare,inspection', [(False, False), (False, True), (True, False)])
def test_nonproducer_missing_manifest_is_record_only(prepare, inspection):
    payload = contract_payload(prepare, inspection)
    contract = _build_live_execution_contract(payload=payload, compiled=compile_protocol_source(payload), handlers={})
    assert 'deck_manifest' not in contract
    assert contract['artifacts']['refs'] == ['offline-explicit-preflight']


def test_live_record_fields_do_not_refuse_execution():
    payload = contract_payload()
    original = copy.deepcopy(payload)
    contract = _build_live_execution_contract(payload=payload, compiled=compile_protocol_source(payload), handlers={})
    assert 'deck_manifest' not in contract and contract['deck_manifest_producer'] == 'lifecycle:prepare'
    assert payload == original
    for field in ('operator_id', 'physical_console_verified', 'artifact_refs', 'reference_snapshot'):
        record_only = copy.deepcopy(payload)
        del record_only['live_execution'][field]
        observed = _build_live_execution_contract(
            payload=record_only, compiled=compile_protocol_source(record_only), handlers={})
        if field == 'operator_id':
            assert 'operator_id' not in observed
        if field == 'physical_console_verified':
            assert 'physical_console_verified' not in observed
        if field == 'artifact_refs':
            assert 'artifacts' not in observed
        if field == 'reference_snapshot':
            assert 'reference_snapshot' not in observed.get('preflight', {})
    no_ack = copy.deepcopy(payload)
    del no_ack['live_execution']['live_execution_ack']
    with pytest.raises(ProtocolLiveContractError) as exc:
        _build_live_execution_contract(payload=no_ack, compiled=compile_protocol_source(no_ack), handlers={})
    assert exc.value.details['missing_contract_fields'] == ['live_execution_ack']


def test_native_cover_move_keeps_missing_observations_as_records():
    from bioxp.protocols.models import ProtocolActionKind
    payload = {'source_type': 'native', 'document': {
        'protocol_id': 'bms-deck-compound', 'version': 1, 'stages': [{
            'stage_id': 'deck', 'actions': [{
                'action_id': 'transfer', 'stage_id': 'deck', 'kind': 'move_cover',
                'params': {'cover_id': 'CV_OUTPUT', 'target_location': 'LOC_OCS'},
            }],
        }]}, 'live_execution': {'live_execution_ack': True}}
    contract = _build_live_execution_contract(
        payload=payload, compiled=compile_protocol_source(payload),
        handlers={ProtocolActionKind.MOVE_COVER: lambda *_: None},
    )
    assert contract['reference_required_action_kinds'] == ['move_cover']
    assert 'preflight' not in contract
    assert 'deck_manifest' not in contract
    assert 'artifacts' not in contract
    assert 'operator_id' not in contract
    assert 'physical_console_verified' not in contract
    from bioxp import api
    request = api.ProtocolExecuteRequest.model_validate({
        **payload, 'dry_run': False, 'idempotency_key': 'offline-click'})
    assert request.model_dump(exclude_none=True, exclude_unset=True)['live_execution'] == {'live_execution_ack': True}
    with pytest.raises(ProtocolLiveContractError) as missing_handler:
        _build_live_execution_contract(payload=payload, compiled=compile_protocol_source(payload), handlers={})
    assert missing_handler.value.details['missing_live_handlers'] == ['move_cover']


def test_ordinary_review_rejects_inspection_decision():
    doc = document('step')
    from dataclasses import replace
    doc = replace(doc, stages=(replace(doc.stages[0], review_required=True),))
    executor, trace = engine(doc)
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.awaiting_review)
    with pytest.raises(ValueError, match='preparation'):
        executor.acknowledge_review(control_id='wrong', gate_id=doc.stages[0].stage_id, decision='ignore')
    executor.acknowledge_review(control_id='review', gate_id=doc.stages[0].stage_id)
    assert finish(run).completed


@pytest.mark.parametrize('decision', ['ignore', 'abort'])
def test_canonical_sqlite_review_gate_and_child_identity(integrated_rig, monkeypatch, tmp_path, decision):
    """Actual canonical API/SQLite gate; preparation/CV is an explicit double.

    A real canonical confirm_gripper child (HW recorder) precedes the warning;
    run_job and all subsequent lifecycle hooks stay real. This is NOT native CV
    qualification, which belongs to the separate frozen native/image harness.
    """
    from bioxp import api, oem_preparation_runtime
    from bioxp.services.protocol_service import ProtocolBindings
    rig = integrated_rig
    result = output(tmp_path)
    def bind_double(**kwargs):
        def prepare(state):
            child = kwargs['execute_native']('confirm_gripper', {}, state)
            assert child['ok'] is True
            return copy.deepcopy(result)
        return prepare
    monkeypatch.setattr(oem_preparation_runtime, 'bind_selected_preparation', bind_double)
    original_factory = api._protocol_bindings
    def factory(*args, **kwargs):
        bindings = original_factory(*args, **kwargs)
        handlers, oem, lifecycle = bindings
        # Explicit inspection Abort effect double; real unlock is parent-owned.
        lifecycle['preparation_abort'] = lambda state: {'ok': True, 'offline_unlock_double': True}
        return ProtocolBindings(handlers, oem, lifecycle,
            source_script_begin=bindings.source_script_begin,
            source_script_returned=bindings.source_script_returned)
    monkeypatch.setattr(api, '_protocol_bindings', factory)
    payload = rig.payload('canonical-review-' + decision)
    payload['document']['metadata'].update(oem_prepare=True)
    payload['document']['metadata']['source_settings']['DeckInspection'] = True
    payload['live_execution'].pop('deck_manifest', None)
    original = copy.deepcopy(payload)
    job = rig.start(payload)
    gate = rig.gate(job, 'review')
    assert hooks(gate) == ['prepare']
    assert gate['operator']['pending_review']['decisions'] == ['abort', 'ignore']
    executor = rig.executors[job['job_id']]
    assert not executor._source_entered
    request = {'command_id': job['job_id'],
        'expected_ownership_generation': job['command']['ownership_generation'],
        'idempotency_key': 'generic-' + decision, 'reviewer': 'offline-reviewer',
        'stage_id': 'lifecycle:prepare', 'action_id': 'lifecycle:prepare'}
    url = '/protocol/jobs/' + job['job_id'] + '/review'
    rejected = rig.client.post(url, json=request)
    assert rejected.status_code == 409, rejected.text
    request.update(idempotency_key='decision-' + decision, decision=decision)
    response = rig.client.post(url, json=request)
    assert response.status_code == 200, response.text
    done = rig.terminal(job)
    assert done['command']['status'] == ('completed' if decision == 'ignore' else 'interrupted'), done
    if decision == 'abort':
        assert hooks(done) == ['prepare', 'preparation_abort']
        assert not executor._source_entered
    assert payload == original
    assert_reopened(rig, done)
    assert done['operator']['reviews'][-1]['decision'] == decision
    assert 'deck_manifest' not in done['execution']['live_contract']


def test_record_only_contract_does_not_write_attestation(tmp_path):
    from bioxp.services.protocol_service import ProtocolOperatorBundleStore
    payload = {'source_type': 'native', 'document': {'protocol_id': 'p', 'stages': [
        {'stage_id': 's', 'actions': [{'action_id': 'n', 'kind': 'note'}]}]},
        'live_execution': {'live_execution_ack': True}}
    contract = _build_live_execution_contract(
        payload=payload, compiled=compile_protocol_source(payload), handlers={})
    store = ProtocolOperatorBundleStore(tmp_path)
    bundle = store.save({'job_id': 'record-only', 'execution': {'live_contract': contract}})
    assert 'preflight_path' not in bundle['artifacts']
    assert not (tmp_path / 'record-only' / 'preflight.json').exists()
    assert store.load('record-only')['execution']['live_contract'] == contract


def test_plate_command_result_enters_executor_with_raw_child_evidence(monkeypatch):
    """Transport-replaced command queue; neither the child nor placement is inferred."""
    from types import SimpleNamespace
    from fastapi import HTTPException
    from bioxp import api
    from bioxp.protocols.executor import ProtocolExecutor
    from bioxp.protocols.models import ProtocolActionKind
    payload = {'source_type': 'native', 'document': {'protocol_id': 'deck-offline',
        'stages': [{'stage_id': 's', 'actions': [
            {'action_id': 'catch-release', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_OUTPUT', 'target_location': 'LOC_OCS'}},
            {'action_id': 'prepare', 'kind': 'plate_prepare', 'params': {'plate_ids': ['PL_OUTPUT']}},
        ]}]}}
    document = compile_protocol_source(payload).document
    monkeypatch.setattr(api.app.state, 'oem_deck_position_table_provider',
                        lambda: {'target': {'location_id': 'LOC_OC_COVER_STORAGE'}}, raising=False)
    commands = []
    rows = {}
    def admit(operation, *, inputs, idempotency_key):
        cid = 'offline-' + str(len(commands))
        commands.append((operation, inputs, idempotency_key))
        rows[cid] = {'command_id': cid, 'status': 'completed',
                     'physical_effect_verified': False,
                     'terminal_evidence': {'children': [
                         {'operation': 'catchPlate', 'terminal_state': 'completed'},
                         {'operation': 'releasePlate', 'terminal_state': 'completed'}]}}
        return {'command_id': cid}
    monkeypatch.setattr(api.app.state, 'oem_wp8_operation_admitter', admit, raising=False)
    monkeypatch.setattr(api.app.state, 'operator_command_plane',
                        SimpleNamespace(store=SimpleNamespace(get_command=rows.get)), raising=False)
    handlers = {ProtocolActionKind.MOVE_COVER: api._protocol_live_plate_move_handler,
                ProtocolActionKind.PLATE_PREPARE: api._protocol_live_plate_prepare_handler}
    executor = ProtocolExecutor(dry_run=False, handlers=handlers)
    state = executor.execute(document)
    assert state.completed
    assert [command[0] for command in commands] == ['move_plate', 'press_plates']
    assert commands[0][1] == {'plate': 4, 'destination': 18, 'press_plate': False}
    for result in state.action_results:
        assert result['ok'] is True and result['command'] == rows[result['command_id']]
        assert result['command']['physical_effect_verified'] is False
        assert 'physical_effect_verified' not in result
    # A terminal failure preserves its full row in the queue exception; it is
    # not reclassified as a successful completed action.
    rows['offline-0']['status'] = 'failed'
    with pytest.raises(HTTPException) as error:
        api._protocol_deck_action_result('offline-0')
    assert error.value.detail['command'] == rows['offline-0']


def test_class_move_to_command_result_uses_same_explicit_executor_outcome(monkeypatch):
    from types import SimpleNamespace
    from bioxp import api
    row = {'command_id': 'mov-offline', 'status': 'completed',
           'physical_effect_verified': False, 'terminal_evidence': {'children': [{'stage': 'moveXY'}]}}
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    monkeypatch.setattr(api.app.state, 'oem_mov_execution_admitter',
                        lambda intent, *, idempotency_key: {'command_id': 'mov-offline'}, raising=False)
    monkeypatch.setattr(api.app.state, 'operator_command_plane',
                        SimpleNamespace(store=SimpleNamespace(get_command=lambda _: row)), raising=False)
    result = api._protocol_live_move_handler(
        SimpleNamespace(params={'script_line': 1, 'location_id': 6}, action_id='move'),
        SimpleNamespace(job_id='job-offline'))
    assert result == {'ok': True, 'command_id': 'mov-offline', 'command': row}
    assert 'physical_effect_verified' not in result
