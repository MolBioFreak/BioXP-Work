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


def test_runtime_initial_check_shadow_uses_bound_provider_and_stays_not_ready(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    calls = []

    class Hardware:
        def initial_check(self, *, mode="shadow"):
            calls.append(mode)
            return {"ok": True, "sequence": ["backend_ready", "door_latch_before", "door_latch_final"], "door_latch": {"door_closed": True, "latch_closed": True}}

    class Program:
        hardware = Hardware()

    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program()).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", params={"run_initial_check": True}))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert calls == ["shadow"]
    assert result["result"]["state"] == "initial_check_passed"
    assert result["result"]["ready"] is False
    assert "initializeMotion_not_executed_from_runtime_worker" in result["result"]["blockers"]


def test_runtime_initial_check_live_requires_initialize_ack_in_handler(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    class Hardware:
        def initial_check(self, *, mode="shadow"):
            raise AssertionError("must not touch hardware without INITIALIZE ack")

    class Program:
        hardware = Hardware()

    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program()).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="YES", artifact_root=str(tmp_path / "live"), params={"run_initial_check": True}))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert result["result"]["state"] == "failed_closed"
    assert "operator_ack_INITIALIZE_required_for_live_initialCheck" in result["result"]["blockers"]


def test_runtime_initial_check_live_writes_artifact(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    class Hardware:
        def initial_check(self, *, mode="shadow"):
            return {"ok": True, "sequence": ["backend_ready", "led_white", "door_latch_before", "deactivate_boards", "activate_boards", "door_latch_final"], "door_latch": {"door_closed": True, "latch_closed": True}}

    class Program:
        hardware = Hardware()

    root = tmp_path / "artifact"
    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program()).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE", artifact_root=str(root), params={"run_initial_check": True}))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert result["result"]["mode"] == "live"
    assert (root / "runtime_initial_check.json").exists()


def test_worker_fails_closed_when_handler_missing(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers={})
    worker.enqueue(OEMRuntimeCommand(name="unlockProcess"))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "no handler" in result["error"]
    assert worker.snapshot()["gantry_available"] is True
    assert store.read_journal("runtime_errors.jsonl")
