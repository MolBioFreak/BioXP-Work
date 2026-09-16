"""Bounded native composite adapter; no hardware, replacement lifecycle or store."""
import json
import pytest
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_v1_integrated_lifecycle import assert_reopened


def configure(rig, monkeypatch, version):
    machine = rig.provider._load_state()
    machine['machine_status']['GripperVersion'] = version
    rig.provider._save_state(machine)
    monkeypatch.setattr(rig.native.tester, '_motion_oem_gripper_version', lambda: version)
    original = rig.native.exchange
    def exchange(board, command, typ, bank, value, **kwargs):
        if (board, command, typ, bank) == (4, 2, 0, 2) and value in (150, 600):
            rig.native.trace.append((board, command, typ, bank, value))
            rig.native.positions[4, 2] = 0
            return {'status': 100, 'value': 0}
        return original(board, command, typ, bank, value, **kwargs)
    monkeypatch.setattr(rig.native.tester, '_send_motor', exchange)
    monkeypatch.setattr(rig.native.tester, 'send_tmcl_retry', exchange)


def invoke(rig):
    return rig.provider.wp8_send_gripper_home('sendGripperHome', {})


def assert_void(receipt):
    assert receipt['ok'] is True, receipt
    assert receipt['source_call_completed'] is True
    assert receipt['source_return_kind'] == 'void'
    for key in ('controller_command_acknowledged', 'controller_completion_verified',
                'hardware_postcondition_verified', 'physical_effect_verified',
                'independent_physical_motion_verified'):
        assert receipt[key] is False


@pytest.mark.parametrize('version', [0, 1])
def test_native_first_and_cached_source_home(integrated_rig, monkeypatch, version):
    rig = integrated_rig
    configure(rig, monkeypatch, version)
    first = invoke(rig)
    assert_void(first)
    raw = first['primitive_result']
    assert raw['home']['home_decision']['source_returned_normally'] is True
    assert raw['home']['speed'] == (600 if version == 0 else 200)
    assert (raw['restore_current'] is None) == (version == 0)
    before = len(rig.native.trace)
    cached = invoke(rig)
    assert_void(cached)
    home = cached['primitive_result']['home']
    assert home['completion_class'] == 'source_cached_noop'
    assert home['source_return_ok'] is True
    assert home['controller_terminal_state_verified'] is False
    assert home['controller_home_proof_verified'] is False
    assert home['physical_effect_verified'] is False
    assert rig.native.trace[before:] == [(4, 5, 6, 2, 31)] + ([(4, 5, 6, 2, 10)] if version == 1 else [])


@pytest.mark.parametrize('mode', ['board_null', 'uninitialized', 'current_nack', 'current_no_reply', 'prepare_exception', 'restore_exception', 'no24v'])
def test_native_source_void_errors_and_absence(integrated_rig, monkeypatch, mode):
    rig = integrated_rig
    configure(rig, monkeypatch, 1)
    tester = rig.native.tester
    if mode == 'board_null':
        tester._oem_board_presence = {4: False}
    elif mode == 'uninitialized':
        tester._oem_board_initialized[4] = False
    elif mode.startswith('current_'):
        rig.native.replies[4, 5, 6, 2] = None if mode == 'current_no_reply' else {'status': 2, 'value': 0}
    elif mode in ('prepare_exception', 'restore_exception'):
        original = tester._send_motor
        def exchange(board, command, typ, bank, value, **kwargs):
            if (board, command, typ, bank, value) == (4, 5, 6, 2, 31 if mode == 'prepare_exception' else 10):
                raise RuntimeError('synthetic current transport failure')
            return original(board, command, typ, bank, value, **kwargs)
        monkeypatch.setattr(tester, '_send_motor', exchange)
    else:
        tester._oem_24v_dropped = True
    if mode in ('prepare_exception', 'restore_exception', 'no24v'):
        with pytest.raises(RuntimeError, match='synthetic current transport failure|Lost 24V'):
            invoke(rig)
        return
    receipt = invoke(rig)
    assert_void(receipt)
    raw = receipt['primitive_result']
    if mode == 'board_null':
        assert raw['source_noop'] == 'board_null'
        assert rig.native.trace == []
    elif mode == 'uninitialized':
        assert raw['home']['ok'] is False
        assert raw['home']['source_return_code'] == 1
        assert raw['home']['failure'] == 'board_not_initialized'
    else:
        assert raw['prepare']['ack'] == rig.native.replies[4, 5, 6, 2]
        assert raw['restore_current']['ack'] == rig.native.replies[4, 5, 6, 2]
        if mode == 'current_nack':
            assert raw['prepare']['ok'] is False


@pytest.mark.parametrize('version', [0, 1])
def test_connected_safe_stop_cleanup_store_nonreplay(integrated_rig, monkeypatch, version):
    rig = integrated_rig
    configure(rig, monkeypatch, version)
    observed = []
    original = rig.native.tester.motor_oem_home_axis
    def observe(*args, **kwargs):
        result = original(*args, **kwargs)
        if args == ('g',):
            observed.append(result)
        return result
    monkeypatch.setattr(rig.native.tester, 'motor_oem_home_axis', observe)
    payload = rig.payload(f'gripper-stop-{version}', delayed=True)
    job = rig.start(payload)
    rig.gate(job, 'delaypoint')
    response = rig.control(job, 'gripper-stop-once', action='safe_stop')
    assert response.status_code == 200, response.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'interrupted', done
    assert_reopened(rig, done)
    children = rig.child_rows(done)
    cleanup = [receipt for row in children
               if (receipt := json.loads(row['receipt_json'])).get('requested_inputs', {}).get('operation') == 'cleanup']
    assert cleanup, children
    def nested(value):
        if isinstance(value, dict):
            if value.get('source_anchor') == 'ClassControlInterface.sendGripperHome':
                yield value
            for child in value.values():
                yield from nested(child)
        elif isinstance(value, list):
            for child in value:
                yield from nested(child)
    receipts = list(nested(cleanup))
    assert receipts, cleanup
    for receipt in receipts:
        assert_void(receipt)
        assert receipt['primitive_result']['home']['completion_class'] == 'source_cached_noop'
    # The genuine safe-stop sourceHomeGripper already homes G before cleanup;
    # its following sendGripperHome correctly takes the cached branch.
    assert any(row['home'].get('completion_class') == 'source_cached_noop' for row in observed)
    before = list(rig.native.trace)
    assert rig.control(job, 'gripper-stop-once', action='safe_stop').json() == response.json()
    assert rig.submit(payload).json()['job_id'] == job['job_id']
    assert rig.child_rows(done) == children
    assert rig.native.trace == before


def test_native_represented_source_exception_is_not_void_success(integrated_rig, monkeypatch):
    rig = integrated_rig
    configure(rig, monkeypatch, 1)
    # Physical preliminary move did not return a source integer; native goHome
    # preserves this as source_exception rather than a normal ignored return.
    monkeypatch.setattr(rig.native.tester, 'motor_oem_move_absolute',
                        lambda *a, **kw: {'ok': False, 'failure': 'synthetic_uncertain_move'})
    receipt = invoke(rig)
    assert receipt['ok'] is False
    assert receipt['source_call_completed'] is False
    assert receipt['primitive_result']['home']['source_exception']['failure'] == 'synthetic_uncertain_move'


def test_unknown_composite_and_generic_receipt_stay_failed(integrated_rig, monkeypatch):
    rig = integrated_rig
    malformed = {'axis': 'g', 'home': {'ok': True}}
    monkeypatch.setattr(rig.native.tester, 'motor_oem_home_axis', lambda *a, **kw: malformed)
    assert invoke(rig)['ok'] is False
    assert rig.provider._deck_primitive_receipt(malformed, source_anchor='unrelated')['ok'] is False
