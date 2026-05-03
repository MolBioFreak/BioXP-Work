from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.oem_runtime_types import OEMRuntimeCommand
from src.bioxp.oem_runtime_worker import OEMRuntimeWorker


def test_worker_brackets_gantry_availability_and_history(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    seen = []
    worker = OEMRuntimeWorker(store=store, handlers={"initializeSystem": lambda cmd: seen.append(worker.snapshot()["gantry_available"]) or {"ok": True}})
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem"))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert seen == [False]
    assert worker.snapshot()["gantry_available"] is True
    assert store.read_journal("command_history.jsonl")[0]["ok"] is True


def test_worker_fails_closed_when_handler_missing(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers={})
    worker.enqueue(OEMRuntimeCommand(name="unlockProcess"))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "no handler" in result["error"]
    assert worker.snapshot()["gantry_available"] is True
    assert store.read_journal("runtime_errors.jsonl")
