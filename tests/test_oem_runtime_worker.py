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


def test_runtime_initialize_motion_rejects_run_homing_true_before_touching_provider(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    def factory():
        raise AssertionError("provider must not be opened when run_homing true is rejected")

    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program_factory=factory).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE", artifact_root=str(tmp_path / "artifact"), params={"run_initialize_motion": True, "run_homing": True}))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "run_homing_true_rejected_by_initializeMotion_diagnostic_stage" in result["result"]["blockers"]


def test_runtime_stepwise_homing_plan_and_live_gates(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    calls = []

    class Hardware:
        def startup_homing_stepwise(self, *, mode="shadow", step="plan", execute=False):
            calls.append((mode, step, execute))
            return {"ok": True, "physical_motion": False, "steps": [{"step": "z-home"}], "monolithic_homing_blocked": False, "not_a_replacement_sequence": True}

    class Program:
        hardware = Hardware()

    root = tmp_path / "artifact"
    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program()).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="shadow", artifact_root=str(root), params={"run_stepwise_homing": True, "homing_step": "plan"}))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert result["result"]["state"] == "stepwise_homing_plan_ready"
    assert calls == [("shadow", "plan", False)]
    assert (root / "runtime_stepwise_homing_plan.json").exists()

    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE", artifact_root=str(root), params={"run_stepwise_homing": True, "homing_step": "z-home"}))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "operator_ack_HOME_required_for_live_oem_initializeMotors_step" in result["result"]["blockers"]
    assert calls == [("shadow", "plan", False)]


def test_runtime_stepwise_homing_live_bad_ack_rejects_before_provider_open(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    def factory():
        raise AssertionError("provider must not open before HOME ack is validated")

    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program_factory=factory).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE", artifact_root=str(tmp_path / "artifact"), params={"run_stepwise_homing": True, "homing_step": "z-home"}))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "operator_ack_HOME_required_for_live_oem_initializeMotors_step" in result["result"]["blockers"]


def test_worker_fails_closed_when_handler_missing(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    worker = OEMRuntimeWorker(store=store, handlers={})
    worker.enqueue(OEMRuntimeCommand(name="unlockProcess"))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "no handler" in result["error"]
    assert worker.snapshot()["gantry_available"] is True
    assert store.read_journal("runtime_errors.jsonl")

def test_runtime_oem_initialize_motors_step_requires_home_ack_and_passes_step(tmp_path):
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    calls = []

    class Hardware:
        def startup_homing_stepwise(self, *, mode="shadow", step="plan", execute=False, preclear_abs=None, require_operator_observed=True):
            calls.append((mode, step, execute, preclear_abs, require_operator_observed))
            return {"ok": True, "physical_motion": bool(execute), "step": {"step": step}, "oem_source_order_preserved": True}

    class Program:
        hardware = Hardware()

    root = tmp_path / "artifact"
    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program()).handlers())
    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE", artifact_root=str(root), params={"run_stepwise_homing": True, "homing_step": "z-home"}))
    result = worker.run_next_for_tests()
    assert result["ok"] is False
    assert "operator_ack_HOME_required_for_live_oem_initializeMotors_step" in result["result"]["blockers"]
    assert calls == []

    worker.enqueue(OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="HOME", artifact_root=str(root), params={"run_stepwise_homing": True, "homing_step": "z-home", "require_operator_observed": True}))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert result["result"]["state"] == "stepwise_homing_step_complete"
    assert calls == [("live", "z-home", True, None, True)]
    assert (root / "runtime_stepwise_homing_z-home.json").exists()
