"""Acceptance tests intentionally fail on baseline's missing real lifecycle hooks.

No xfails/skips or all-hook doubles. A preflight failure means the downstream
scenario has NOT executed. Source-error/false-Abort use actual source event entry
on the active executor; they do not invent a public immediate-Abort API.
"""
import json
import pytest
from tests.protocol_v1_integration_fixture import integrated_rig, PhysicalLeafGate
# Imported fixture dependencies must be visible to pytest in this module.
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig


def assert_reopened(rig, done):
    assert done['command']['terminal'], done
    children = rig.child_rows(done)
    assert children, 'complete lifecycle must retain canonical children'
    from bioxp.runtime_audit_store import TERMINAL_COMMAND_STATES
    assert all(row['status'] in TERMINAL_COMMAND_STATES for row in children), children
    expected_ids = done['execution']['runtime_state']['workflow']['child_command_ids']
    assert [row['command_id'] for row in children] == expected_ids
    assert len(expected_ids) == len(set(expected_ids))
    assert all(row['parent_command_id'] == done['job_id'] for row in children)
    assert all(json.loads(row['receipt_json']) for row in children), children
    fresh = rig.reopen(done)
    assert fresh['workflow']['command'] == done['command']
    assert fresh['workflow']['execution']['runtime_state'] == done['execution']['runtime_state']
    assert fresh['children'] == children


def hooks(done):
    return [row['hook'] for row in done['execution']['runtime_state']['action_results']
            if 'hook' in row]


def assert_abort_invalidated(rig, done):
    receipts = rig.native_results(done, 'software_abort')
    assert len(receipts) == 1, receipts
    assert receipts[0]['status'] == 'completed', receipts
    raw = receipts[0]['response']
    assert raw['ok'] is True, raw
    assert rig.native.tester.oem_no24v_state() is True
    rows = rig.references.snapshot(('x', 'y', 'z', 'g'))['rows']
    assert all(row['state'] != 'referenced' for row in rows.values()), rows
    assert not any(row[1] == 3 for row in rig.native.trace), 'software Abort must not send addressed Stop'
    assert rig.provider._wp8_source_script_returned is True
    return raw


def test_complete_real_factory_lifecycle_repeated_job_and_reopen(integrated_rig):
    rig = integrated_rig
    ids = []
    for index in range(2):
        payload = rig.payload(f'complete-{index}')
        job = rig.start(payload)
        done = rig.terminal(job)
        assert done['command']['status'] == 'completed', done
        assert rig.control_chain(done) == ['epilogue_lid']
        lid = rig.native_results(done, 'epilogue_lid')[0]['response']
        assert lid['source_body_returned'] is True and lid['ok'] is True
        assert lid['physical_effect_verified'] is False
        assert hooks(done) == ['run_job', 'script_prologue', 'epilogue_sweep',
                               'epilogue_lid', 'epilogue_park', 'script_finally']
        assert rig.native.tester.oem_no24v_state() is False
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
    assert done['command']['status'] == 'failed', done
    assert rig.control_chain(done) == (['shutdown_temperature'] if signal == 'source_error'
                                       else ['shutdown_temperature', 'software_abort'])
    if signal == 'abort_false':
        assert_abort_invalidated(rig, done)
        assert 'cleanup' not in hooks(done) and 'source_error' not in hooks(done)
    else:
        assert hooks(done)[-2:] == ['script_finally', 'source_error']
        assert 'cleanup' not in hooks(done)
        assert rig.native.tester.oem_no24v_state() is False
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
    if action == 'safe_stop':
        assert done['command']['status'] == 'interrupted', done
        assert rig.control_chain(done) == ['shutdown_temperature']
        assert rig.native.tester.oem_no24v_state() is False
        assert hooks(done)[-3:] == ['safe_stop_exit', 'script_finally', 'cleanup']
    else:
        # Source Abort(true) is not an addressed Stop and not a normal cleanup
        # success: the actual software Abort invalidates motion authority first.
        assert_abort_invalidated(rig, done)
        assert rig.control_chain(done) == ['shutdown_temperature', 'software_abort']
        assert done['command']['status'] == 'failed', done
        cleanup = [row for row in done['execution']['runtime_state']['action_results']
                   if row.get('hook') == 'cleanup']
        assert len(cleanup) == 1 and cleanup[0]['ok'] is False, cleanup
        assert not rig.native_results(done, 'epilogue_lid')
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


@pytest.mark.parametrize('mode', ['ordinary', 'deferred'])
def test_real_deferred_pause_wake_requires_reached_gate(integrated_rig, mode):
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
    paused = rig.control(job, 'pause', action='pause', mode=mode)
    assert paused.status_code == 200, paused.text
    rig.body_gate.release.set()
    gate = rig.gate(job, mode + '_pause')
    gate_id = gate['execution']['runtime_state']['workflow']['gate_id']
    if mode == 'ordinary':
        prior = rig.native.tester.oem_current_board_lifecycle_generation()
        continued = rig.control(job, 'ordinary-continue', action='continue',
                                gate='ordinary_pause', gate_id=gate_id)
        assert continued.status_code == 200, continued.text
        done = rig.terminal(job)
        assert done['command']['status'] == 'completed', done
        assert 'ordinary_pause_prepare' in hooks(done)
        assert rig.control_chain(done) == ['epilogue_lid']
        assert rig.native.tester.oem_current_board_lifecycle_generation() == prior
        assert_reopened(rig, done)
        return
    generation_before = rig.native.tester.oem_current_board_lifecycle_generation()
    safety_before = rig.references.snapshot(('x', 'y', 'z', 'g'))
    awake = rig.control(job, 'wake', action='wake', gate_id=gate_id)
    assert awake.status_code == 200, awake.text
    # Wake is distinct from Continue; wait for the real source flag publication.
    rig.wait(job, lambda row: rig.executors[job['job_id']]._wake_complete is True)
    assert rig.native.tester.oem_current_board_lifecycle_generation() == generation_before + 1
    wake = rig.native_results(job, 'wake_prepare')
    assert len(wake) == 1 and wake[0]['status'] == 'completed', wake
    initial = wake[0]['response']['source_children'][0]
    assert initial['ok'] is True and initial['source_return'] is True
    assert initial['initial_check']['board_lifecycle_generation']['source_order'] == ['cmd64=0', 'cmd64=1']
    assert rig.native.tester.oem_no24v_state() is False
    assert all(row['state'] == 'referenced' for row in rig.references.snapshot(('x', 'y', 'z', 'g'))['rows'].values())
    assert safety_before['rows'], 'reference baseline must be populated'
    continued = rig.control(job, 'after-wake', action='continue', gate='deferred_pause', gate_id=gate_id)
    assert continued.status_code == 200, continued.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed', done
    assert_reopened(rig, done)



@pytest.mark.parametrize('release', ['internal_abort', 'board_error'])
def test_native_thermal_pending_child_released_by_real_control(integrated_rig, release):
    """Real timer -> API callback -> Abort; no thermal source body replacement."""
    rig = integrated_rig
    if release == 'board_error':
        rig.native.replies[6, 10, 4, 0] = lambda: (
            None if rig.native.thermal_wait.is_set() else {'status': 100, 'value': 25000})
    job = rig.start(rig.payload('thermal-' + release, opcode='sp', arguments=['60', '100', '2.5']))
    assert rig.native.thermal_wait.wait(8), 'real thermal native wait never entered'
    assert rig.store.get_workflow(job['job_id'])['command']['terminal'] is False
    children = rig.child_rows(job)
    target = [r for r in children if r['action_id'] == 'protocol.oem_lifecycle.set_tc_temperature']
    assert len(target) == 1 and target[0]['status'] == 'reserved', children
    assert rig.provider._wp8_source_script_returned is False
    if release == 'internal_abort':
        rig.executors[job['job_id']].source_error(false_abort=True)
    done = rig.terminal(job)
    assert done['command']['status'] == 'failed', done
    assert rig.control_chain(done) == ['set_tc_temperature', 'shutdown_temperature', 'software_abort']
    assert_abort_invalidated(rig, done)
    receipt = rig.native_results(done, 'set_tc_temperature')[0]
    raw = receipt['response']
    assert raw['source_body_returned'] is True, raw
    assert raw['source_user_stopped'] is True and raw['source_wait_satisfied'] is False, raw
    if release == 'board_error':
        assert receipt['status'] == 'failed'
        assert raw['source_board_error_event'] == 'readTemperature communication error!'
    else:
        assert receipt['status'] == 'completed'
    assert 'cleanup' not in hooks(done) and 'source_error' not in hooks(done)
    assert rig.native.tester._oem_thermal_board_timer_enabled is False
    assert_reopened(rig, done)


def test_native_thermal_deferred_pause_bailout_releases_child(integrated_rig):
    rig = integrated_rig
    job = rig.start(rig.payload('thermal-deferred', opcode='sp', arguments=['60', '100', '2.5']))
    assert rig.native.thermal_wait.wait(8)
    before = rig.safety()
    response = rig.control(job, 'thermal-pause-once', action='pause', mode='deferred')
    assert response.status_code == 200, response.text
    reached = rig.gate(job, 'deferred_pause')
    assert reached['command']['terminal'] is False
    assert rig.control_chain(job) == ['set_tc_temperature', 'thermal_bailout']
    raw = rig.native_results(job, 'set_tc_temperature')[0]['response']
    assert raw['source_body_returned'] is True and raw['source_wait_satisfied'] is True
    assert not raw.get('source_user_stopped', False)
    assert rig.native.tester.oem_no24v_state() == before['no24v'] is False
    assert rig.references.snapshot(('x', 'y', 'z', 'g'))['rows'] == before['references']['rows']
    count = len(rig.native.trace)
    replay = rig.control(job, 'thermal-pause-once', action='pause', mode='deferred')
    assert replay.status_code == 200 and replay.json() == response.json()
    assert len(rig.native.trace) == count
    stopped = rig.control(job, 'thermal-pause-stop', action='safe_stop')
    assert stopped.status_code == 200, stopped.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'interrupted', done
    assert rig.control_chain(done) == ['set_tc_temperature', 'thermal_bailout', 'shutdown_temperature']
    assert 'safe_stop_exit' not in hooks(done)
    assert_reopened(rig, done)
