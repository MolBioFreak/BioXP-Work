"""Canonical API binding qualification; physical I/O remains isolated/doubled."""
import copy
import hashlib
from types import SimpleNamespace

import pytest

from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_v1_integrated_lifecycle import assert_reopened, hooks


def selected(rig, key, inspection):
    payload = rig.payload(key)
    metadata = payload['document']['metadata']
    metadata['oem_prepare'] = True
    metadata['source_settings'].update(DeckInspection=inspection, ScreenResolutionHigh=False)
    metadata['source_preparation_requirements'] = {
        'JobName': None, 'OutputPlateRequired': False, 'TroughRequired': False}
    return payload


def test_real_api_binds_source_no_inspection_without_reset(integrated_rig):
    rig = integrated_rig
    payload = selected(rig, 'selected-no-inspection', False)
    original = copy.deepcopy(payload)
    job = rig.start(payload)
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed', done
    assert hooks(done)[0] == 'prepare'
    result = next(r for r in done['execution']['runtime_state']['action_results'] if r.get('hook') == 'prepare')
    assert 'DeckInspection=false' in str(result)
    assert 'source_preparation_input' in str(result)
    assert 'source_job_load_children' in str(result)
    assert payload == original
    assert not any('pipette_tip_transition' in str(r) for r in rig.child_rows(done))
    assert_reopened(rig, done)


def test_real_api_led_dependency_refuses_before_native_entry(integrated_rig):
    rig = integrated_rig
    payload = selected(rig, 'selected-live-inspection-blocked', True)
    wire = list(rig.native.trace)
    response = rig.submit(payload)
    assert response.status_code == 409, response.text
    assert 'source_preparation_led_backend_unavailable' in response.text
    assert rig.native.trace == wire
    assert rig.executors == {}


def test_explicit_new_job_load_publishes_five_resets_not_constructor(integrated_rig):
    rig = integrated_rig
    payload = selected(rig, 'explicit-new-diagnostic-load', False)
    metadata = payload['document']['metadata']
    metadata['oem_job_load'] = True
    # Existing host custody is retained, but there is no historical stock model.
    metadata['source_model']['tip_trays'] = []
    metadata['source_job_metadata'] = {
        'no_liquid': True,
        'tip_trays': [{'tray_id': str(i), 'location': location, 'tip_type': tip_type}
            for i, (location, tip_type) in enumerate(((7,50),(8,50),(9,50),(10,200),(15,50)))],
        'strips': [{'tray_id': name, 'location': location, 'strip_color': None}
            for name, location in (('STRIP_ONE',11),('STRIP_TWO',12),('STRIP_THREE',13),('STRIP_FOUR',14))],
        'trays': {name: {'tray_id':name, 'location':location}
            for name, location in (('POOL_PLATE',23),('OUTPUT_PLATE',21),('REAGENT_PLATE',3))},
    }
    original = copy.deepcopy(payload)
    done = rig.terminal(rig.start(payload))
    assert done['command']['status'] == 'completed', done
    source = done['execution']['runtime_state']['source_model']
    assert source['logical_tip_present'] is False
    assert source['carried_plate_present'] is False
    assert all(not well['empty'] for tray in source['tip_trays'] for well in tray['wells'])
    assert len(source['tip_trays']) == 5 and all(len(tray['wells']) == 96 for tray in source['tip_trays'])
    assert len(source['strips']) == 4 and all(len(tray['wells']) == 8 for tray in source['strips'])
    assert all(len(tray['wells']) == 96 for tray in source['trays'].values())
    rows = rig.child_rows(done)
    reset_rows = [row for row in rows if '"transition": "reset"' in row['receipt_json'] or '"transition":"reset"' in row['receipt_json']]
    assert len(reset_rows) == 5, rows
    assert payload == original
    assert_reopened(rig, done)


def test_actual_source_capture_preserves_selected_model(integrated_rig):
    from bioxp.oem_machine_bundle import get_active_oem_machine_snapshot
    from bioxp.oem_preparation_runtime import capture_selected_preparation
    metadata = selected(integrated_rig, 'capture-inputs', False)['document']['metadata']
    original = copy.deepcopy(metadata)
    capture = capture_selected_preparation(snapshot=get_active_oem_machine_snapshot(), metadata=metadata)
    assert capture['requirements'] == metadata['source_preparation_requirements']
    assert capture['source_model']['logical_tip_present'] is False
    assert len(capture['source_model']['tip_trays']) == 4  # not constructor's five
    assert capture['source_identity']['inspection_profile'] == 'Settings3200'
    assert capture['capture_sha256']
    capture['source_model']['tip_trays'][0]['wells'][0]['empty'] = False
    assert metadata == original


def test_camera_exposure_uses_canonical_child_claim(integrated_rig, tmp_path):
    from bioxp.oem_preparation_runtime import PreparationCameraRuntime
    from bioxp.operator_controls import make_workflow_lifecycle_control_executor
    rig = integrated_rig
    payload = rig.payload('camera-control-child', delayed=True)
    payload['document']['metadata'].update(selected(rig, 'metadata', False)['document']['metadata'])
    payload['document']['metadata']['delayed_start'] = True
    job = rig.start(payload)
    rig.gate(job, 'delaypoint')
    calls = []
    class Camera:
        def capture_inspection(self, settings):
            settings.validate()
            calls.append(settings)
            return SimpleNamespace(frame=SimpleNamespace(content_sha256='physical-leaf-test-frame'), source_frames_discarded=5)
    runtime = PreparationCameraRuntime(Camera(), artifact_root=tmp_path)
    execute = make_workflow_lifecycle_control_executor(rig.store, lambda: rig.provider, preparation_camera=runtime)
    state = rig.executors[job['job_id']]._state
    with rig.store.workflow_context(job['job_id'], source_occurrence_id='test-camera-source'):
        result = execute('preparation_camera_exposure', state,
            source_occurrence_id='test-camera-source:exposure', arguments={'value':None})
    assert result['ok'] is True, result
    assert len(calls) == 1 and calls[0].exposure == 0
    rows = [row for row in rig.child_rows(job) if row['command_id'] == result['command_id']]
    assert len(rows) == 1 and rows[0]['status'] == 'completed'
    response = rig.control(job, 'finish-camera-test', action='continue', gate='delaypoint', gate_id='source:delay')
    assert response.status_code == 200, response.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed', done
    assert_reopened(rig, done)


def test_snapshot_requires_actual_frame_and_saves_exact_bytes(tmp_path):
    from bioxp.oem_preparation_runtime import PreparationCameraRuntime, snapshot_evidence_missing
    # Encoded pixels are an explicitly synthetic physical camera leaf, not CV verdicts.
    import cv2
    import numpy as np
    ok, image = cv2.imencode('.jpg', np.zeros((480, 640, 3), dtype=np.uint8))
    assert ok
    content = image.tobytes()
    calls = []
    class Camera:
        def capture_inspection(self, settings):
            settings.validate()
            calls.append(settings)
            from bioxp.camera_provider import CameraFrame, CameraIdentity
            from datetime import datetime, timezone
            return SimpleNamespace(frame=CameraFrame(content=content, provider_generation=7,
                sequence=len(calls), captured_at=datetime.now(timezone.utc), content_sha256=hashlib.sha256(content).hexdigest(),
                identity=CameraIdentity('/test-camera-leaf', 'test', 'test', 'test')), source_frames_discarded=1)
    runtime = PreparationCameraRuntime(Camera(), artifact_root=tmp_path)
    artifact = runtime.snapshot_image(condition='../../source condition', artifact_id='canonical-child')
    from pathlib import Path
    assert Path(artifact['path']).read_bytes() == content
    assert artifact['sha256'] == hashlib.sha256(content).hexdigest()
    assert len(calls) == 1
    events = []
    state = SimpleNamespace(job_id='actual-test-job', record_event=lambda name, **kwargs: events.append((name, kwargs)))
    assert runtime.capture_image('checkPurificationStation', state) == content
    assert len(events) == 1 and events[0][0] == 'source_inspection_image'
    saved = events[0][1]['detail']
    assert Path(saved['path']).parent == tmp_path / state.job_id / 'source-images'
    assert Path(saved['path']).read_bytes() == content and saved['sequence'] == 2
    assert not snapshot_evidence_missing({'source_anchor':'ControlLib.SnapshotImage:8951-8994','result':artifact})
    assert snapshot_evidence_missing({'source_anchor':'ControlLib.SnapshotImage:8979-8993','ok':True,'exception_suppressed':True})


def test_empty_camera_never_writes_or_claims_artifact(tmp_path):
    from bioxp.oem_preparation_runtime import PreparationCameraRuntime
    camera = SimpleNamespace(capture_inspection=lambda settings: SimpleNamespace(frame=SimpleNamespace(content=b'')))
    runtime = PreparationCameraRuntime(camera, artifact_root=tmp_path)
    with pytest.raises(RuntimeError, match='source_snapshot_frame_empty'):
        runtime.snapshot_image(condition='empty', artifact_id='canonical-empty')
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('operation,args', [
    ('preparation_led', {'channel':True, 'on':True}),
    ('preparation_led', {'channel':4, 'on':True}),
    ('preparation_led', {'channel':1, 'on':1}),
    ('preparation_camera_exposure', {'value':True}),
    ('preparation_camera_exposure', {'value':float('nan')}),
])
def test_finite_camera_controls_validate_before_claim(operation, args):
    from bioxp.operator_controls import make_workflow_lifecycle_control_executor
    execute = make_workflow_lifecycle_control_executor(None, lambda: pytest.fail('device accessed'))
    with pytest.raises(ValueError, match='workflow_lifecycle_arguments_invalid'):
        execute(operation, None, source_occurrence_id='invalid', arguments=args)
