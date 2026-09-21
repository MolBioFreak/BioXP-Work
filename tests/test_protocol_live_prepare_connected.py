"""OFFLINE connected native/CV qualification, not physical readiness.

Only inherited controller/device syscalls and fresh acquisition are doubled.
Synthetic black JPEG pixels deliberately produce genuine CV warnings; operator
Ignore/Abort is explicit, never an injected successful inspection/prepare hook.
"""
import copy
import hashlib
import json
from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_camera_oem_led_binding import led_rig
from tests.test_protocol_v1_integrated_lifecycle import assert_reopened, hooks
from tests.test_protocol_workflow_connected import await_job


@pytest.fixture(autouse=True)
def connected_camera(led_rig, monkeypatch):
    from bioxp import api
    from bioxp.camera_provider import CameraFrame
    import cv2
    import numpy as np
    camera, wire, _ = led_rig
    ok, encoded = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))
    assert ok
    pixels = encoded.tobytes()
    captures = []
    def acquire(settings):
        settings.validate()
        captures.append(settings)
        return SimpleNamespace(frame=CameraFrame(content=pixels,
            provider_generation=camera.generation, sequence=len(captures),
            captured_at=datetime.now(timezone.utc),
            content_sha256=hashlib.sha256(pixels).hexdigest(), identity=camera.discover()),
            source_frames_discarded=5)
    monkeypatch.setattr(camera, 'capture_inspection', acquire)
    monkeypatch.setattr(api, '_camera_provider', camera)
    yield camera, wire, captures
    camera.close()


@pytest.fixture(autouse=True)
def connected_motion_wait(monkeypatch):
    from tests.test_deck_near_terminal import NearUSB

    def wait_many(leaf, targets, **kwargs):
        # The inherited named-motion fixture only models STA waits. Native
        # preparation also uses WaitAll; its instantaneous physical leaves
        # have already updated each addressed motor before this join.
        targets = tuple(targets)
        assert targets and len(set(targets)) == len(targets)
        assert type(kwargs['sta_sequential']) is bool
        per_axis = {}
        for board, motor in targets:
            assert (board, motor) in ((5, 0), (4, 0))
            assert any((b, m) == (board, motor) for b, m, _ in leaf.moves)
            per_axis['x' if board == 5 else 'y'] = leaf.motor_wait_target_reached(board, motor)
        return {'ok': all(row['ok'] for row in per_axis.values()),
                'sta_sequential': kwargs['sta_sequential'], 'per_axis': per_axis}

    monkeypatch.setattr(NearUSB, 'motor_wait_target_reached_many', wait_many)


def authored_job(rig, key):
    payload = rig.payload(key)
    meta = payload['document']['metadata']
    meta.update(oem_prepare=True, oem_job_load=True)
    meta['source_settings'].update(DeckInspection=True, ScreenResolutionHigh=False,
                                   InspectionLogOnly=False)
    meta['source_preparation_requirements'] = {
        'JobName': None, 'OutputPlateRequired': False, 'TroughRequired': False}
    meta['source_model']['tip_trays'] = []
    meta['source_job_metadata'] = {
        'no_liquid': True,
        'tip_trays': [{'tray_id': str(i), 'location': loc, 'tip_type': typ}
            for i, (loc, typ) in enumerate(((7,50),(8,50),(9,50),(10,200),(15,50)))],
        'strips': [{'tray_id': name, 'location': loc, 'strip_color': None}
            for name, loc in (('STRIP_ONE',11),('STRIP_TWO',12),('STRIP_THREE',13),('STRIP_FOUR',14))],
        'trays': {name: {'tray_id': name, 'location': loc}
            for name, loc in (('POOL_PLATE',23),('OUTPUT_PLATE',21),('REAGENT_PLATE',3))},
    }
    payload['live_execution'].pop('deck_manifest', None)
    return payload


@pytest.mark.parametrize('integrated_rig,decision', [(None, 'ignore'), ('partial-clean-path', 'ignore'), ('partial-clean-path', 'abort')], indirect=['integrated_rig'])
def test_actual_api_native_cv_finish_and_partial_unknown(integrated_rig, connected_camera, decision, request):
    rig = integrated_rig
    partial = request.node.callspec.params['integrated_rig'] == 'partial-clean-path'
    # The live startup owns a real collection query before preparation. Exercise
    # that same public producer, not a manufactured collection receipt/value.
    query = rig.client.post('/liquid/tip-status', headers={
        'Idempotency-Key': f'connected-startup-query-{partial}-{decision}'})
    assert query.status_code == 200, query.text
    assert query.json()['hardware_query_verified'] is True
    before = dict(rig.store.connection.execute('SELECT * FROM operator_plane_deck_semantic_state').fetchone())
    assert before['current_location'] == 'LOC_PARK'
    assert before['tip_loaded'] == 0 and before['tip_dirty'] == 0
    assert before['clean_path'] is (None if partial else 0)
    payload = authored_job(rig, f'connected-{partial}-{decision}')
    original = copy.deepcopy(payload)
    job = rig.start(payload)
    gate = await_job(rig.client, job['job_id'], lambda row:
        row['command']['terminal'] or row['execution']['runtime_state']['workflow']['gate'] == 'review', timeout=60)
    if gate['command']['terminal']:
        import os
        from pathlib import Path
        evidence = Path(os.environ['OEM_NATIVE_RESULTS']) / f'connected-{partial}-{decision}-failure.json'
        evidence.write_text(json.dumps({'job': gate, 'children': rig.child_rows(gate)}, indent=2, default=str))
        pytest.fail('Preparation ended before review; evidence: ' + str(evidence))
    assert hooks(gate) == ['prepare']
    result = gate['execution']['runtime_state']['action_results'][0]
    assert result['inspection_decision_required'] is True
    assert any(r['effective_status'] != 'OK' for r in result['inspections'])
    rows = rig.child_rows(gate)
    with rig.store._lock:
        source_only = [dict(row) for row in rig.store.connection.execute(
            "SELECT * FROM operator_plane_wp8_children WHERE operation='sourceForceToHighHome'")]
        assert len(source_only) == 5, source_only
        for row in source_only:
            assert row['command_id'] in {child['command_id'] for child in rows}
            assert row['terminal_state'] == 'completed'
            assert json.loads(row['state_mutation_json']) == {'pseudo_z_home': 500}
            assert rig.store.connection.execute(
                'SELECT COUNT(*) FROM operator_plane_delivery_attempts WHERE command_id=?',
                (row['command_id'],)).fetchone()[0] == 0
            evidence = json.loads(row['terminal_evidence_json'])
            assert not evidence.get('dispatch_attempt_id'), evidence
    assert not rig.executors[job['job_id']]._source_entered
    camera, wire, captures = connected_camera
    assert captures and wire.seen
    request = {'command_id': job['job_id'],
        'expected_ownership_generation': job['command']['ownership_generation'],
        'idempotency_key': 'explicit-' + decision, 'reviewer': 'offline-connected-test',
        'stage_id': 'lifecycle:prepare', 'action_id': 'lifecycle:prepare', 'decision': decision}
    response = rig.client.post('/protocol/jobs/' + job['job_id'] + '/review', json=request)
    assert response.status_code == 200, response.text
    done = rig.terminal(job)
    assert done['command']['status'] == ('completed' if decision == 'ignore' else 'interrupted'), done
    if decision == 'ignore':
        assert hooks(done) == ['prepare', 'run_job', 'script_prologue', 'epilogue_sweep',
                               'epilogue_lid', 'epilogue_park', 'script_finally']
        assert ('rgb', (0, 0, 0)) in rig.trace
    else:
        assert hooks(done) == ['prepare', 'preparation_abort']
    rows = rig.child_rows(done)
    resets = [r for r in rows if '"transition": "reset"' in r['receipt_json'] or '"transition":"reset"' in r['receipt_json']]
    assert len(resets) == 5
    assert payload == original
    assert_reopened(rig, done)
