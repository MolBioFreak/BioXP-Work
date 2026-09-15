"""Offline native failures through the canonical worker and fresh SQLite reader."""
import json
import os
from pathlib import Path
import subprocess
import sqlite3
import sys
import threading
import time

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_near_terminal import NearUSB
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.serial206_y_provider import Serial206YProvider


@pytest.mark.parametrize('fault', ['marker', 'marker_noop', 'force_reference', 'terminal_reference', 'semantic_reference',
                                  'y_native', 'y_observation', 'y_sqlite', 'xy_observation'])
def test_queued_native_failure_evidence(installed_retained, retained_rig, monkeypatch, fault):
    from bioxp import api
    app, provider, observations, references, root = installed_retained
    plane = app.state.operator_command_plane
    store = plane.store
    generation = int(provider.generation_provider())
    assert catalog_action(app)['enabled'] is False
    qualify_test_references(references)
    snapshot = api._collect_and_publish_hardware_snapshot(['axes', 'latch'],
        reason='isolated-native-fault')
    assert snapshot['deck_authority']['enabled'], snapshot['deck_authority']
    leaf = NearUSB((59258, 9079) if fault == 'marker_noop' else (60561, 71745))
    original_wait = leaf.motor_wait_target_reached
    def wait(*args, **kwargs):
        result = original_wait(*args, **kwargs)
        result['event']['owner_generation'] = generation
        return result
    monkeypatch.setattr(leaf, 'motor_wait_target_reached', wait)
    monkeypatch.setattr(leaf, 'begin_bus_event_window', lambda: {
        'after_sequence': 0, 'receive_owner': 'offline-usb', 'owner_generation': generation})
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: generation, reference_store=references)
    adapter.y_provider = Serial206YProvider(leaf, state_store=retained_rig[2],
        generation_provider=lambda: generation, reference_store=references)
    monkeypatch.setattr(observations, 'oem_move_to', adapter.oem_move_to)
    injected = []
    def fail(*args, **kwargs):
        injected.append(fault)
        raise RuntimeError('isolated_' + fault)
    if fault in {'marker', 'marker_noop'}:
        original = store.record_delivery_attempt
        def marker(*args, **kwargs):
            if kwargs.get('work_identity', '').startswith('stage:4:'):
                return fail()
            return original(*args, **kwargs)
        monkeypatch.setattr(store, 'record_delivery_attempt', marker)
    elif fault.endswith('_reference'):
        original = store.assert_deck_execution_current
        boundary = ('after_force_to_high_home' if fault == 'force_reference' else
                    'before_semantic_commit' if fault == 'semantic_reference'
                    else 'before_terminalize_stage_3_moveTo')
        def assertion(*args, **kwargs):
            if kwargs.get('boundary') == boundary:
                return fail()
            return original(*args, **kwargs)
        monkeypatch.setattr(store, 'assert_deck_execution_current', assertion)
    elif fault == 'y_native':
        original = leaf.motor_oem_move_absolute
        def move(board, *args, **kwargs):
            if board == 4:
                injected.append(fault)
                exc = RuntimeError('isolated_y_native')
                exc.motion_evidence = {'axis': 'y', 'command_sent': True,
                                       'completion_class': 'isolated_timeout'}
                raise exc
            return original(board, *args, **kwargs)
        monkeypatch.setattr(leaf, 'motor_oem_move_absolute', move)
    elif fault == 'y_observation':
        monkeypatch.setattr(adapter.y_provider, '_record_observation', fail)
    elif fault == 'y_sqlite':
        original = adapter.y_provider._record_observation
        def contended(*args, **kwargs):
            script = ('import sqlite3,sys; c=sqlite3.connect(sys.argv[1]); '
                      'c.execute("BEGIN IMMEDIATE"); print("held",flush=True); '
                      'sys.stdin.readline(); c.rollback(); c.close()')
            holder = subprocess.Popen([sys.executable, '-c', script, str(root/'bioxp_runtime.db')],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                assert holder.stdout.readline().strip() == 'held'
                assert holder.poll() is None
                try:
                    return original(*args, **kwargs)
                except sqlite3.OperationalError:
                    assert holder.poll() is None  # independent transaction still owns SQLite
                    injected.append(fault)
                    raise
            finally:
                holder.communicate('release\n', timeout=5)
                assert holder.returncode == 0
        monkeypatch.setattr(adapter.y_provider, '_record_observation', contended)
    else:
        monkeypatch.setattr(adapter.y_provider, 'record_move_xy_observation', fail)
    client = TestClient(app)
    action = catalog_action(app)
    plane.start()
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': fault,
        'expected_ownership_generation': generation,
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': 'LOC_TC_BARCODE' if fault in {'marker', 'marker_noop'} else 'TECANRACK1',
                   'camera_offset': False}})
    assert response.status_code == 200, response.text
    cid = response.json()['command_id']
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        compact = client.get('/operator/v2/actions/receipts/' + cid).json()
        if compact['status'] not in {'queued', 'dispatched', 'issued_pending'}:
            break
        time.sleep(.01)
    detail = client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
    assert injected == [fault], json.dumps(detail)
    delivered = fault not in {'marker_noop', 'force_reference'}
    assert detail['status'] == ('ambiguous' if delivered else 'failed'), json.dumps(detail)
    assert (store.deck_recovery_blocker() is not None) is delivered
    with sqlite3.connect(root / 'bioxp_runtime.db') as db:
        terminal = json.loads(db.execute('SELECT terminal_json FROM operator_plane_commands WHERE command_id=?', (cid,)).fetchone()[0])
    assert terminal['delivery_attempted'] is delivered
    assert detail['physical_effect_verified'] is False
    encoded = json.dumps(detail)
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.' + fault + '.json').write_text(
            json.dumps({'compact': compact, 'detail': detail, 'terminal_json': terminal}, indent=2))
    native = detail['deck_movement']['stages'][3]['terminal_evidence']['provider_evidence']
    if fault in {'y_native', 'y_observation', 'y_sqlite'}:
        assert native['commands']['x']['controller_terminal_state_verified'] is True
        if fault in {'y_observation', 'y_sqlite'}:
            assert native['controller_failure']['result']['ok'] is True
            assert native['controller_failure']['result']['completion_class'] == 'event_128'
        assert 'near_axis_sequential' in encoded and 'commands' in encoded
        assert ('isolated_timeout' if fault == 'y_native' else 'observation_recording_failure') in encoded
        assert 'event_128' in encoded  # completed X remains independent of failed Y
    elif fault == 'xy_observation':
        assert native['ok'] is True
        assert native['commands']['x']['controller_terminal_state_verified'] is True
        assert native['commands']['y']['controller_completion_verified'] is True
        assert native['commands']['y']['result']['completion_class'] == 'event_128'
        assert 'observation_recording_failure' in encoded and 'event_128' in encoded
    elif delivered:
        assert detail['deck_movement']['controller_completion_verified'] is True
    else:
        assert detail['deck_movement']['controller_completion_verified'] is False
        assert leaf.moves == []
    plane.stop()
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
              'print(json.dumps(fresh_process_receipts(sys.argv[1],sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root), cid],
                                                 text=True, timeout=12))
    assert reopened == {'compact': compact, 'detail': detail}


@pytest.mark.parametrize('fault', ['first', 'second', 'both', 'callback'])
def test_native_pair_settles_before_error(monkeypatch, fault):
    leaf = NearUSB((0, 0))
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
                                                   generation_provider=lambda: 3)
    held = threading.Event()
    release = threading.Event()
    second_called = threading.Event()
    ended = threading.Event()
    errors = []
    def first(**kwargs):
        held.set()
        assert release.wait(3)
        if fault in {'first', 'both'}:
            raise RuntimeError('first')
        return {'ok': True, 'axis': 'x', 'command_issued': True}
    monkeypatch.setattr(adapter, 'x_move_absolute', first)
    original = leaf.motor_oem_move_absolute
    def second(*args, **kwargs):
        second_called.set()
        if fault in {'second', 'both'}:
            raise RuntimeError('second')
        return original(*args, **kwargs)
    monkeypatch.setattr(leaf, 'motor_oem_move_absolute', second)
    def callback():
        if held.is_set():
            second_called.set()
            if fault == 'callback':
                raise RuntimeError('callback')
        return None
    def run():
        try:
            adapter.oem_move_to(1000, 1000, 500, pseudo_home_steps=500,
                gripper_confirmed=False, tip_loaded=False, plate_on_gantry=0,
                interrupt_reason=callback)
        except BaseException as exc:
            errors.append(exc)
        finally:
            ended.set()
    parent = threading.Thread(target=run)
    parent.start()
    try:
        assert held.wait(2)
        assert second_called.wait(2)
        assert not ended.wait(.05), 'parent escaped while native first child was held'
    finally:
        release.set()
        parent.join(3)
    assert not parent.is_alive()
    assert len(errors) == 1
    evidence = errors[0].motion_evidence
    assert evidence['children_settled'] is True
    expected = 2 if fault == 'both' else 1
    assert len(evidence['child_failures']) == expected
    if fault in {'second', 'callback'}:
        assert evidence['operations'][0]['axis'] == 'x'


@pytest.mark.parametrize('fault', ['second_issue', 'pair_wait', 'postwait_readback'])
def test_parallel_xy_exception_keeps_issued_children(monkeypatch, fault):
    leaf = NearUSB((0, 0))
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
                                                   generation_provider=lambda: 3)
    def fail(*args, **kwargs):
        raise RuntimeError('isolated_' + fault)
    if fault == 'second_issue':
        original = leaf.motor_oem_move_absolute
        def move(board, *args, **kwargs):
            if board == 4:
                return fail()
            return original(board, *args, **kwargs)
        monkeypatch.setattr(leaf, 'motor_oem_move_absolute', move)
    elif fault == 'pair_wait':
        monkeypatch.setattr(leaf, 'motor_wait_target_reached_many', fail)
    else:
        monkeypatch.setattr(adapter, '_read_axis_position', fail)
    with pytest.raises(RuntimeError) as raised:
        adapter.move_xy(90000, 5000, wait_timeout_s=5.0,
                        source_context="ClassControlInterface.btnLOC1_Click")
    evidence = raised.value.motion_evidence
    assert evidence['commands']['x']['command_issued'] is True
    assert set(evidence['commands']) == ({'x'} if fault == 'second_issue' else {'x', 'y'})
    if fault == 'postwait_readback':
        assert evidence['pair_wait']['ok'] is True
        assert set(evidence['waits']) == {'x', 'y'}
    assert evidence['physical_effect_verified'] is False
