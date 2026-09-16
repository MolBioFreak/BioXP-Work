"""Acceptance tests intentionally fail on baseline's missing real lifecycle hooks.

No xfails/skips or all-hook doubles. A preflight failure means the downstream
scenario has NOT executed. Source-error/false-Abort use actual source event entry
on the active executor; they do not invent a public immediate-Abort API.
"""
import pytest
from tests.protocol_v1_integration_fixture import integrated_rig, PhysicalLeafGate
# Imported fixture dependencies must be visible to pytest in this module.
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig


def assert_reopened(rig, done):
    assert done['command']['terminal'], done
    children = rig.children(done)
    assert children, 'complete lifecycle must retain canonical children'
    assert not [row for row in children if row['status'] in ('queued', 'dispatched', 'issued_pending')], children
    fresh = rig.reopen(done)
    assert fresh['command'] == done['command']
    assert fresh['execution']['runtime_state'] == done['execution']['runtime_state']


def test_complete_real_factory_lifecycle_repeated_job_and_reopen(integrated_rig):
    rig = integrated_rig
    ids = []
    for index in range(2):
        payload = rig.payload(f'complete-{index}')
        job = rig.start(payload)
        done = rig.terminal(job)
        assert done['command']['status'] == 'completed', done
        assert_reopened(rig, done)
        before = rig.children(job)
        replay = rig.submit(payload)
        assert replay.status_code == 200 and replay.json()['job_id'] == job['job_id'], replay.text
        assert rig.children(job) == before
        ids.append(job['job_id'])
    assert len(set(ids)) == 2
    assert len(rig.executors) == 2


@pytest.mark.parametrize('signal', ['source_error', 'abort_false'])
def test_real_source_event_terminates_active_owner_and_reopens(integrated_rig, signal):
    rig = integrated_rig
    job = rig.start(rig.payload(signal, delayed=True))
    gate = rig.gate(job, 'delaypoint')
    assert not gate['command']['terminal']
    # Board/application event enters the real executor; all ensuing lifecycle
    # handlers and canonical provider children remain untouched.
    rig.executors[job['job_id']].source_error(false_abort=signal == 'abort_false')
    done = rig.terminal(job)
    assert done['command']['status'] != 'completed', done
    assert_reopened(rig, done)
    assert ('rgb', (0, 0, 0)) not in rig.trace, 'body RGB must not execute after the source event'


@pytest.mark.parametrize('action', ['safe_stop', 'abort'])
def test_real_control_exit_duplicate_control_and_nonreplay(integrated_rig, action):
    rig = integrated_rig
    payload = rig.payload(action, delayed=True)
    job = rig.start(payload)
    rig.gate(job, 'delaypoint')
    response = rig.control(job, action + '-once', action=action)
    assert response.status_code == 200, response.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'interrupted', done
    assert_reopened(rig, done)
    before = rig.children(job)
    replay = rig.control(job, action + '-once', action=action)
    assert replay.status_code == 200 and replay.json() == response.json(), replay.text
    assert rig.submit(payload).json()['job_id'] == job['job_id']
    assert rig.children(job) == before


def test_real_delay_continue_rejects_stale_gate_and_deduplicates(integrated_rig):
    rig = integrated_rig
    job = rig.start(rig.payload('continue', delayed=True))
    rig.gate(job, 'delaypoint')
    wrong = rig.control(job, 'stale', action='continue', gate='delaypoint', gate_id='old')
    assert wrong.status_code == 409, wrong.text
    first = rig.control(job, 'continue-once', action='continue', gate='delaypoint', gate_id='source:delay')
    assert first.status_code == 200, first.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed', done
    assert_reopened(rig, done)
    before = rig.children(job)
    replay = rig.control(job, 'continue-once', action='continue', gate='delaypoint', gate_id='source:delay')
    assert replay.status_code == 200 and replay.json() == first.json(), replay.text
    assert rig.children(job) == before


def test_real_deferred_pause_wake_requires_reached_gate(integrated_rig):
    rig = integrated_rig
    rig.body_gate = PhysicalLeafGate()
    payload = rig.payload('pause-wake')
    payload['document']['stages'][0]['actions'].append({
        'action_id': 'after', 'kind': 'oem_operation', 'oem_opcode': 'led',
        'source_occurrence_id': 'source:after', 'params': {'arguments': ['1','1','1']}})
    job = rig.start(payload)
    assert rig.body_gate.entered.wait(8), 'actual body physical leaf did not enter'
    early = rig.control(job, 'early-wake', action='wake', gate_id='not-reached')
    assert early.status_code == 409, early.text
    paused = rig.control(job, 'pause', action='pause', mode='deferred')
    assert paused.status_code == 200, paused.text
    rig.body_gate.release.set()
    gate = rig.gate(job, 'deferred_pause')
    gate_id = gate['execution']['runtime_state']['workflow']['gate_id']
    awake = rig.control(job, 'wake', action='wake', gate_id=gate_id)
    assert awake.status_code == 200, awake.text
    # Wake is distinct from Continue; wait for the real source flag publication.
    rig.wait(job, lambda row: rig.executors[job['job_id']]._wake_complete is True)
    continued = rig.control(job, 'after-wake', action='continue', gate='deferred_pause', gate_id=gate_id)
    assert continued.status_code == 200, continued.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed', done
    assert_reopened(rig, done)
