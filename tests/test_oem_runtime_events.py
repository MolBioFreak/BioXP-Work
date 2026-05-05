from src.bioxp.oem_runtime_events import OEMRuntimeEventRouter
from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.oem_runtime_worker import OEMRuntimeWorker


def _worker(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers={"initializeSystem": lambda cmd: {"ok": True}, "wakefrompause": lambda cmd: {"ok": False}})
    return store, worker, OEMRuntimeEventRouter(store=store, worker=worker)


def test_door_close_with_latch_queues_initialize_system(tmp_path):
    store, worker, events = _worker(tmp_path)
    result = events.handle_door_event(door_closed=True, latch_closed=True)
    assert "queued_initializeSystem" in result["actions_taken"]
    assert worker.snapshot()["queue_depth"] == 1
    assert store.read_journal("event_journal.jsonl")[-1]["event_type"] == "door"


def test_door_close_while_paused_queues_wakefrompause(tmp_path):
    store, worker, events = _worker(tmp_path)
    events.handle_pause()
    result = events.handle_door_event(door_closed=True, latch_closed=True)
    assert "queued_wakefrompause" in result["actions_taken"]
    assert worker.snapshot()["queue_depth"] == 1


def test_door_open_requires_force_abort_motion(tmp_path):
    store, worker, events = _worker(tmp_path)
    result = events.handle_door_event(door_open=True, door_closed=False, latch_closed=False)
    assert "forceAbortMotion_required" in result["actions_taken"]
    assert store.read_journal("runtime_errors.jsonl")[-1]["error_situation"] == "door_malfunction"
