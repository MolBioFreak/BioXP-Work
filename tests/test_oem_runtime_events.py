from src.bioxp.oem_runtime_events import OEMRuntimeEventRouter
from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.oem_runtime_worker import OEMRuntimeWorker


def _worker(tmp_path, monkeypatch):
    from src.bioxp import oem_runtime_events as events_module
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner

    monkeypatch.setattr(events_module, "lifecycle_state", CanonicalLifecycleOwner())
    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers={"PrepareToRunJob": lambda cmd: {"ok": True}, "wakefrompause": lambda cmd: {"ok": False}})
    return store, worker, OEMRuntimeEventRouter(store=store, worker=worker)


def test_door_close_with_latch_records_evidence_without_auto_initialization(tmp_path, monkeypatch):
    store, worker, events = _worker(tmp_path, monkeypatch)
    result = events.handle_door_event(door_closed=True, latch_closed=True)
    assert result["actions_taken"] == ["door_close_observed"]
    assert worker.snapshot()["queue_depth"] == 0
    assert store.read_journal("event_journal.jsonl")[-1]["event_type"] == "door"


def test_door_close_while_paused_requires_explicit_resume(tmp_path, monkeypatch):
    store, worker, events = _worker(tmp_path, monkeypatch)
    events.handle_pause()
    result = events.handle_door_event(door_closed=True, latch_closed=True)
    assert result["actions_taken"] == ["explicit_resume_required"]
    assert worker.snapshot()["queue_depth"] == 0


def test_door_open_requires_force_abort_motion(tmp_path, monkeypatch):
    store, worker, events = _worker(tmp_path, monkeypatch)
    result = events.handle_door_event(door_open=True, door_closed=False, latch_closed=False)
    assert "forceAbortMotion_required" in result["actions_taken"]
    assert store.read_journal("runtime_errors.jsonl")[-1]["error_situation"] == "door_malfunction"
