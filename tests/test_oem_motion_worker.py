def test_motion_worker_serializes_initialize_system_and_writes_events(tmp_path):
    from src.bioxp.oem_motion_worker import OEMMotionWorker, OemMotionCommand

    ran = []
    worker = OEMMotionWorker(artifact_root=tmp_path, handlers={"initializeSystem": lambda cmd: ran.append(cmd.session_id) or {"ok": True}})

    first = worker.enqueue(OemMotionCommand(session_id="s1", name="initializeSystem"))
    second = worker.enqueue(OemMotionCommand(session_id="s2", name="initializeSystem"))

    assert first["queued"] is True
    assert second["queued"] is True
    assert worker.status()["queue_depth"] == 2

    worker.run_next()
    worker.run_next()

    assert ran == ["s1", "s2"]
    assert worker.status()["queue_depth"] == 0
    text = (tmp_path / "motion_queue_events.jsonl").read_text()
    assert '"event": "queued"' in text
    assert '"event": "complete"' in text


def test_motion_worker_abort_marks_active_and_clears_queue(tmp_path):
    from src.bioxp.oem_motion_worker import OEMMotionWorker, OemMotionCommand

    worker = OEMMotionWorker(artifact_root=tmp_path, handlers={})
    worker.enqueue(OemMotionCommand(session_id="s1", name="initializeSystem"))

    result = worker.abort(reason="operator")

    assert result["aborted"] is True
    assert worker.status()["state"] == "aborted"
    assert worker.status()["queue_depth"] == 0


def test_motion_worker_fails_closed_without_handler(tmp_path):
    from src.bioxp.oem_motion_worker import OEMMotionWorker, OemMotionCommand

    worker = OEMMotionWorker(artifact_root=tmp_path, handlers={})
    worker.enqueue(OemMotionCommand(session_id="s1", name="initializeSystem"))

    result = worker.run_next()

    assert result["ok"] is False
    assert "no handler" in result["error"]
    assert worker.status()["state"] == "failed"


def test_motion_command_rejects_unknown_names():
    from pydantic import ValidationError
    from src.bioxp.oem_motion_worker import OemMotionCommand

    try:
        OemMotionCommand(session_id="s1", name="randomMove")
    except ValidationError:
        pass
    else:
        raise AssertionError("unknown OEM motion command was accepted")
