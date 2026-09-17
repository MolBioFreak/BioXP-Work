"""Test-only scheduling of real OEM writes, waits and timer custody."""
import json
import sys
import threading
import pytest
from tests.protocol_v1_integration_fixture import integrated_rig, PhysicalLeafGate
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_v1_integrated_lifecycle import (
    assert_abort_invalidated, assert_reopened, assert_thermal_abort_outcome, hooks)


@pytest.mark.parametrize('interleaving', ['early_no24v', 'both_flags', 'board_error'])
def test_real_native_abort_interleaving_and_retained_custody(integrated_rig, monkeypatch, interleaving):
    from bioxp import usb_driver
    rig = integrated_rig
    native = rig.native
    tester = native.tester
    real_time = usb_driver.time
    waiting = threading.Event()
    no24v_written = threading.Event()
    body_returned = threading.Event()
    observed = []
    early = interleaving == 'early_no24v'
    # Scope sleep interception to this module and the exact source caller. No
    # altered flags, return values, futures, native body or wire suppression.
    class Clock:
        def __getattr__(self, name):
            return getattr(real_time, name)

        def sleep(self, seconds):
            caller = sys._getframe(1).f_code
            if caller is usb_driver.BioXpTester._oem_thermal_wait.__code__:
                waiting.set()
                assert (no24v_written if early else native.abort_returned).wait(12)
            elif caller is usb_driver.BioXpTester.motor_oem_force_abort_motion.__code__:
                assert tester._oem_24v_dropped is True
                assert tester._oem_user_stopped is False
                no24v_written.set()
                if early:
                    assert body_returned.wait(12)
            real_time.sleep(seconds)
    monkeypatch.setattr(usb_driver, 'time', Clock())
    real_body = tester._oem_thermal_set_board
    def observe_body(*args, **kwargs):
        result = real_body(*args, **kwargs)
        observed.append(dict(result))
        body_returned.set()
        return result
    monkeypatch.setattr(tester, '_oem_thermal_set_board', observe_body)
    if early:
        native.timer_gate = PhysicalLeafGate()
    if interleaving == 'board_error':
        native.replies[6, 10, 4, 0] = lambda: (
            None if native.thermal_wait.is_set() else {'status': 100, 'value': 25000})
    payload = rig.payload('ordered-' + interleaving, opcode='sp', arguments=['60', '100', '2.5'])
    job = rig.start(payload)
    assert waiting.wait(8)
    if interleaving != 'board_error':
        rig.executors[job['job_id']].source_error(false_abort=True)
    assert body_returned.wait(8)
    assert native.abort_returned.wait(8)
    assert observed[0]['source_body_returned'] is (not early)
    if early:
        assert native.timer_gate.entered.wait(8)
        timer = observed[0]['source_timer_future']
        assert not timer.done()
        def child_dispatched(row):
            return any(child['action_id'] == 'protocol.oem_lifecycle.set_tc_temperature'
                       and child['status'] == 'dispatched' for child in rig.child_rows(row))
        pending = rig.wait(job, child_dispatched)
        assert pending['command']['terminal'] is False
        children = rig.child_rows(job)
        child = next(row for row in children if row['action_id'] == 'protocol.oem_lifecycle.set_tc_temperature')
        raw = json.loads(child['receipt_json'])['response']
        assert raw['source_body_returned'] is False
        assert raw['source_exception'] == 'Lost 24V power setTemperature2'
        assert raw['source_timer_pending'] is True
        fresh = rig.reopen(job)
        assert fresh['workflow']['command']['terminal'] is False
        retained = next(row for row in fresh['children'] if row['command_id'] == child['command_id'])
        assert retained == child
        native.timer_gate.release.set()
    done = rig.terminal(job)
    assert done['command']['status'] == 'failed'
    assert rig.control_chain(done) == ['set_tc_temperature', 'shutdown_temperature', 'software_abort']
    assert_abort_invalidated(rig, done)
    receipt = rig.native_results(done, 'set_tc_temperature')[0]
    assert_thermal_abort_outcome(receipt, board_error=interleaving == 'board_error')
    assert receipt['response']['source_body_returned'] is (not early)
    assert tester._oem_thermal_board_timer_enabled is False
    assert 'cleanup' not in hooks(done) and 'source_error' not in hooks(done)
    assert_reopened(rig, done)
    before = rig.children(job)
    wire = list(native.trace)
    rig.executors[job['job_id']].source_error(false_abort=True)
    replay = rig.submit(payload)
    assert replay.status_code == 200 and replay.json()['job_id'] == job['job_id']
    assert rig.children(job) == before
    assert native.trace == wire
