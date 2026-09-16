"""Artifact logging is not an extra motion gate; acquisition remains real input."""
import hashlib
from pathlib import Path
from bioxp.oem_preparation_runtime import PreparationCameraRuntime
from types import SimpleNamespace
from datetime import datetime, timezone
from bioxp.camera_provider import CameraFrame, CameraIdentity


class _Camera:
    """Synthetic acquired pixels, not a fabricated CV classification."""
    def __init__(self):
        import cv2
        import numpy as np
        ok, encoded = cv2.imencode('.jpg', np.zeros((480, 640, 3), dtype=np.uint8))
        assert ok
        content = encoded.tobytes()
        self.frame = CameraFrame(content=content, provider_generation=1, sequence=1,
            captured_at=datetime.now(timezone.utc), content_sha256=hashlib.sha256(content).hexdigest(),
            identity=CameraIdentity('/fixture', 'fixture', 'fixture', 'fixture'))
    def capture_inspection(self, settings):
        settings.validate()
        return SimpleNamespace(frame=self.frame, source_frames_discarded=1)


def _state():
    events = []
    return SimpleNamespace(job_id='logging-test', events=events,
        record_event=lambda name, **kw: events.append(SimpleNamespace(event=name, detail=kw['detail'])))


def test_image_logging_failure_preserves_acquired_cv_bytes(tmp_path, monkeypatch):
    camera = _Camera()
    runtime = PreparationCameraRuntime(camera, artifact_root=tmp_path)
    state = _state()
    def full(*args, **kwargs):
        raise OSError('fixture disk full')
    with monkeypatch.context() as patching:
        patching.setattr(Path, 'open', full)
        content = runtime.capture_image('checkTrough', state)
    image = state.events[-1].detail
    assert content == camera.frame.content
    assert image['capture_ok'] is True and image['artifact_saved'] is False
    assert image['ok'] is False
    assert image['sha256'] == hashlib.sha256(content).hexdigest()
    assert image['size_bytes'] == len(content)
    assert 'fixture disk full' in image['error']


def test_capture_failure_is_not_converted_to_a_logged_image(tmp_path):
    class FailedCamera:
        def capture_inspection(self, settings):
            raise RuntimeError('fixture acquisition failed')
    import pytest
    runtime = PreparationCameraRuntime(FailedCamera(), artifact_root=tmp_path)
    state = _state()
    with pytest.raises(RuntimeError, match='acquisition failed'):
        runtime.capture_image('checkTrough', state)
    assert not [event for event in state.events if event.event == 'source_inspection_image']
