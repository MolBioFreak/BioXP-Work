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
