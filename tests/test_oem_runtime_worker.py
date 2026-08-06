from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.oem_runtime_types import OEMRuntimeCommand
from src.bioxp.oem_runtime_worker import OEMRuntimeWorker


def test_worker_brackets_gantry_availability_and_history(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    seen = []
    worker = OEMRuntimeWorker(store=store, handlers={"PrepareToRunJob": lambda cmd: seen.append(worker.snapshot()["gantry_available"]) or {"ok": True}})
    worker.enqueue(OEMRuntimeCommand(name="PrepareToRunJob"))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert seen == [False]
    assert worker.snapshot()["gantry_available"] is True
    assert store.read_journal("command_history.jsonl")[0]["ok"] is True


def test_worker_collects_terminal_hardware_snapshot_after_every_command(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    seen = []

    def terminal_snapshot(command, result):
        seen.append((command.command_id, command.name, result["ok"]))
        return {"ok": True, "snapshot": {"snapshot_id": "auto-snapshot-1"}}

    worker = OEMRuntimeWorker(
        store=store,
        handlers={"PrepareToRunJob": lambda _cmd: {"ok": True}},
        terminal_snapshot_hook=terminal_snapshot,
    )
    command = OEMRuntimeCommand(name="PrepareToRunJob")
    worker.enqueue(command)

    result = worker.run_next_for_tests()

    assert result["ok"] is True
    assert seen == [(command.command_id, "PrepareToRunJob", True)]
    assert result["result"]["automatic_hardware_snapshot"]["snapshot"]["snapshot_id"] == "auto-snapshot-1"
    history = store.read_journal("command_history.jsonl")[0]
    assert history["result"]["automatic_hardware_snapshot"]["ok"] is True


def test_terminal_hardware_snapshot_failure_is_reported_without_rewriting_command_result(tmp_path):
    store = OEMRuntimeStore(tmp_path)

    def terminal_snapshot(_command, _result):
        raise RuntimeError("synthetic query-only snapshot failure")

    worker = OEMRuntimeWorker(
        store=store,
        handlers={"PrepareToRunJob": lambda _cmd: {"ok": True}},
        terminal_snapshot_hook=terminal_snapshot,
    )
    worker.enqueue(OEMRuntimeCommand(name="PrepareToRunJob"))

    result = worker.run_next_for_tests()

    assert result["ok"] is True
    assert result["result"]["automatic_hardware_snapshot"] == {
        "ok": False,
        "error": "synthetic query-only snapshot failure",
        "query_only": True,
    }
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
