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


def test_worker_collects_terminal_hardware_snapshot_after_every_command(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    seen = []

    def terminal_snapshot(command, result):
        seen.append((command.command_id, command.name, result["ok"]))
        return {"ok": True, "snapshot": {"snapshot_id": "auto-snapshot-1"}}

    worker = OEMRuntimeWorker(
        store=store,
        handlers={"initializeSystem": lambda _cmd: {"ok": True}},
        terminal_snapshot_hook=terminal_snapshot,
    )
    command = OEMRuntimeCommand(name="initializeSystem")
    worker.enqueue(command)

    result = worker.run_next_for_tests()

    assert result["ok"] is True
    assert seen == [(command.command_id, "initializeSystem", True)]
    assert result["result"]["automatic_hardware_snapshot"]["snapshot"]["snapshot_id"] == "auto-snapshot-1"
    history = store.read_journal("command_history.jsonl")[0]
    assert history["result"]["automatic_hardware_snapshot"]["ok"] is True


def test_terminal_hardware_snapshot_failure_is_reported_without_rewriting_command_result(tmp_path):
    store = OEMRuntimeStore(tmp_path)

    def terminal_snapshot(_command, _result):
        raise RuntimeError("synthetic query-only snapshot failure")

    worker = OEMRuntimeWorker(
        store=store,
        handlers={"initializeSystem": lambda _cmd: {"ok": True}},
        terminal_snapshot_hook=terminal_snapshot,
    )
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem"))

    result = worker.run_next_for_tests()

    assert result["ok"] is True
    assert result["result"]["automatic_hardware_snapshot"] == {
        "ok": False,
        "error": "synthetic query-only snapshot failure",
        "query_only": True,
    }
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
    assert calls == []
    assert result["result"]["state"] == "diagnostic_complete"
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


def test_runtime_initial_check_live_writes_artifact(tmp_path, monkeypatch):
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

    class Hardware:
        def set_led_rgb(self, _r, _g, _b):
            return {"ok": True}

        def query_door(self):
            return {"value": 1, "ack": {"status": 100}}

        def query_latch(self):
            return {"value": 0, "ack": {"status": 100}}

        def set_solenoid(self, _value):
            return {"ok": True}

        def query_voltage(self):
            return {"payload_raw": 0, "reply_present": True, "transport_outcome": "reply", "oem_status": 100}

        def deactivate_boards(self):
            return {"ok": True}

        def activate_boards(self):
            return {"ok": True}

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


def test_runtime_initialize_motion_diagnostic_runs_after_initial_check_and_blocks_homing(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    calls = []

    class Hardware:
        def initial_check(self, *, mode="shadow"):
            calls.append(("initial", mode))
            return {"ok": True, "door_latch": {"door_closed": True, "latch_closed": True}}

        def initialize_motion_diagnostic(self, *, mode="shadow", run_homing=False):
            calls.append(("motion", mode, run_homing))
            return {"ok": True, "physical_motion": False, "axis_snapshots": {"z": {"switches": {"left_state": 0, "right_state": 1}}}}

    class Program:
        hardware = Hardware()

    root = tmp_path / "artifact"
    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program()).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE", artifact_root=str(root), params={"run_initial_check": True, "run_initialize_motion": True, "run_homing": False}))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert result["result"]["state"] == "initialize_motion_diagnostic_complete"
    assert result["result"]["ready"] is False
    assert calls == [("initial", "live"), ("motion", "live", False)]
    assert (root / "runtime_initialize_motion.json").exists()
    assert "home_predicates_unproven" in result["result"]["blockers"]


def test_runtime_initialize_motion_requires_exact_operator_ack_before_touching_provider(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    def factory():
        raise AssertionError("provider must not be opened when run_homing true is rejected")

    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program_factory=factory).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE", artifact_root=str(tmp_path / "artifact"), params={"run_initialize_motion": True, "run_homing": True}))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "operator_ack_INITIALIZE_MOTION_STAGE_required" in result["result"]["blockers"]


def test_worker_fails_closed_when_handler_missing(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers={})
    worker.enqueue(OEMRuntimeCommand(name="unlockProcess"))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "no handler" in result["error"]
    assert worker.snapshot()["gantry_available"] is True
    assert store.read_journal("runtime_errors.jsonl")
