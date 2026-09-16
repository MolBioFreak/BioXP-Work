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
def test_nonproducer_still_requires_manifest(prepare, inspection):
    payload = contract_payload(prepare, inspection)
    with pytest.raises(ProtocolLiveContractError) as exc:
        _build_live_execution_contract(payload=payload, compiled=compile_protocol_source(payload), handlers={})
    assert 'deck_manifest' in exc.value.details['missing_contract_fields']


def test_selected_real_producer_only_defers_manifest_not_other_guards():
    payload = contract_payload()
    original = copy.deepcopy(payload)
    contract = _build_live_execution_contract(payload=payload, compiled=compile_protocol_source(payload), handlers={})
    assert contract['deck_manifest'] == {} and contract['deck_manifest_producer'] == 'lifecycle:prepare'
    assert payload == original
    for field in ('operator_id', 'live_execution_ack', 'physical_console_verified', 'artifact_refs'):
        bad = copy.deepcopy(payload)
        del bad['live_execution'][field]
        with pytest.raises(ProtocolLiveContractError):
            _build_live_execution_contract(payload=bad, compiled=compile_protocol_source(bad), handlers={})


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
    assert done['execution']['live_contract']['deck_manifest'] == {}
