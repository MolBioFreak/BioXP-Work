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


def test_runtime_initialize_system_default_completes_diagnostic_without_nested_startup(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=None).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", params={"run_homing": False}))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert result["result"]["state"] == "diagnostic_complete"
    assert result["result"]["ready"] is False
    assert worker.snapshot()["gantry_available"] is True


def test_worker_fails_closed_when_handler_missing(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers={})
    worker.enqueue(OEMRuntimeCommand(name="unlockProcess"))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "no handler" in result["error"]
    assert worker.snapshot()["gantry_available"] is True
    assert store.read_journal("runtime_errors.jsonl")
