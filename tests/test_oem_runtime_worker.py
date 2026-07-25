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

def test_runtime_oem_initialize_motors_step_requires_home_ack_and_passes_step(tmp_path, monkeypatch):
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    lifecycle.run_stage("initial_check", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

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

    worker.enqueue(OEMRuntimeCommand(name="startupHomingStepwise", mode="live", operator_ack="HOME", artifact_root=str(root), params={"homing_step": "z-home", "require_operator_observed": True}))
    result = worker.run_next_for_tests()
    assert result["ok"] is True
    assert result["result"]["state"] == "stepwise_homing_step_complete"
    assert calls == [("live", "z-home", True, None, True)]
    assert (root / "runtime_stepwise_homing_z-home.json").exists()


def test_runtime_stepwise_homing_rejects_out_of_order_stage_and_persists_expected_next(tmp_path, monkeypatch):
    """OEM initializeMotors must admit Z home before any later physical stage."""
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    lifecycle.run_stage("initial_check", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

    calls = []

    class Hardware:
        def startup_homing_stepwise(self, **kwargs):
            calls.append(kwargs)
            return {"ok": True, "physical_motion": True, "step": {"step": kwargs["step"]}}

    class Program:
        hardware = Hardware()

    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(
        store=store,
        handlers=OEMRuntimeCommandHandlers(startup_program=Program(), store=store).handlers(),
    )
    worker.enqueue(
        OEMRuntimeCommand(
            name="initializeSystem",
            mode="live",
            operator_ack="HOME",
            artifact_root=str(tmp_path / "artifact"),
            params={"run_stepwise_homing": True, "homing_step": "gripper-clear"},
        )
    )

    result = worker.run_next_for_tests()

    assert result["ok"] is False
    assert "oem_initializeMotors_expected_next_stage_z-home" in result["result"]["blockers"]
    assert calls == []
    ledger = store.read_oem_movement_ledger()
    assert ledger["expected_next_stage"] == "z-home"
    assert ledger["stages"]["z-home"]["state"] == "pending"


def test_runtime_stepwise_homing_requires_observation_before_admitting_next_stage(tmp_path, monkeypatch):
    """Acknowledge and physical observation are distinct OEM-stage gates."""
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    lifecycle.run_stage("initial_check", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

    calls = []

    class Hardware:
        def startup_homing_stepwise(self, **kwargs):
            calls.append(kwargs["step"])
            return {"ok": True, "physical_motion": True, "step": {"step": kwargs["step"]}}

    class Program:
        hardware = Hardware()

    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(
        store=store,
        handlers=OEMRuntimeCommandHandlers(startup_program=Program(), store=store).handlers(),
    )
    root = tmp_path / "artifact"

    worker.enqueue(OEMRuntimeCommand(
        name="startupHomingStepwise", mode="live", operator_ack="HOME", artifact_root=str(root),
        params={"homing_step": "z-home"},
    ))
    z_result = worker.run_next_for_tests()
    assert z_result["ok"] is True
    assert store.read_oem_movement_ledger()["stages"]["z-home"]["state"] == "acknowledged"

    worker.enqueue(OEMRuntimeCommand(
        name="startupHomingStepwise", mode="live", operator_ack="HOME", artifact_root=str(root),
        params={"homing_step": "gripper-current-31"},
    ))
    blocked = worker.run_next_for_tests()
    assert blocked["ok"] is False
    assert calls == ["z-home"]

    worker.enqueue(OEMRuntimeCommand(
        name="startupHomingStepwise", mode="live", operator_ack="OBSERVE", artifact_root=str(root),
        params={"homing_step": "z-home", "record_stage_observation": True, "observed_pass": True, "operator_note": "Observed Z reference."},
    ))
    observed = worker.run_next_for_tests()

    assert observed["ok"] is True
    ledger = store.read_oem_movement_ledger()
    assert ledger["stages"]["z-home"]["state"] == "operator_observed"
    assert ledger["expected_next_stage"] == "gripper-current-31"


def test_live_stage_provider_is_validated_before_ledger_admission(tmp_path, monkeypatch):
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    lifecycle.run_stage("initial_check", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

    class Program:
        hardware = object()

    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program(), store=store).handlers())
    worker.enqueue(OEMRuntimeCommand(
        name="startupHomingStepwise", mode="live", operator_ack="HOME",
        artifact_root=str(tmp_path / "artifact"), params={"homing_step": "z-home"},
    ))
    result = worker.run_next_for_tests()

    assert result["ok"] is False
    assert "startup_homing_stepwise_method_unavailable" in result["result"]["blockers"]
    assert store.read_oem_movement_ledger() is None


def test_live_stage_exception_persists_failed_closed_ledger_and_artifact(tmp_path, monkeypatch):
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    lifecycle.run_stage("initial_check", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

    class Hardware:
        def startup_homing_stepwise(self, **kwargs):
            raise RuntimeError("synthetic stage transport failure")

    class Program:
        hardware = Hardware()

    root = tmp_path / "artifact"
    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program(), store=store).handlers())
    worker.enqueue(OEMRuntimeCommand(
        name="startupHomingStepwise", mode="live", operator_ack="HOME",
        artifact_root=str(root), params={"homing_step": "z-home"},
    ))
    result = worker.run_next_for_tests()

    assert result["ok"] is False
    assert result["result"]["state"] == "failed_closed"
    assert result["result"]["stepwise_homing"]["physical_effect_verified"] is False
    ledger = store.read_oem_movement_ledger()
    assert ledger is not None
    assert ledger["stages"]["z-home"]["state"] == "failed"
    assert ledger["stages"]["z-home"]["result"]["exception_type"] == "RuntimeError"
    assert ledger["terminal_state"] == "failed_closed"
    assert (root / "runtime_stepwise_homing_z-home_failure.json").exists()


def test_robot_observation_boundary_rejects_coercible_boolean_and_blank_note(tmp_path, monkeypatch):
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    lifecycle.run_stage("initial_check", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

    class Hardware:
        def startup_homing_stepwise(self, **kwargs):
            return {"ok": True, "physical_effect_verified": False}

    class Program:
        hardware = Hardware()

    store = OEMRuntimeStore(tmp_path / "store")
    worker = OEMRuntimeWorker(store=store, handlers=OEMRuntimeCommandHandlers(startup_program=Program(), store=store).handlers())
    worker.enqueue(OEMRuntimeCommand(
        name="startupHomingStepwise", mode="live", operator_ack="HOME",
        artifact_root=str(tmp_path / "artifact"), params={"homing_step": "z-home"},
    ))
    assert worker.run_next_for_tests()["ok"] is True

    for verdict, note in (("false", "Observed failure."), (True, "   ")):
        worker.enqueue(OEMRuntimeCommand(
            name="startupHomingStepwise", mode="live", operator_ack="OBSERVE",
            artifact_root=str(tmp_path / "artifact"),
            params={
                "homing_step": "z-home",
                "record_stage_observation": True,
                "observed_pass": verdict,
                "operator_note": note,
            },
        ))
        rejected = worker.run_next_for_tests()
        assert rejected["ok"] is False

    ledger = store.read_oem_movement_ledger()
    assert ledger is not None
    assert ledger["stages"]["z-home"]["state"] == "acknowledged"
    assert ledger["stages"]["z-home"]["observation"] is None
    assert ledger["expected_next_stage"] == "z-home"


def test_artifact_failure_after_live_execution_persists_failed_closed_ledger(tmp_path, monkeypatch):
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    lifecycle.run_stage("initial_check", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

    class Hardware:
        def startup_homing_stepwise(self, **kwargs):
            return {"ok": True, "motion_command_attempted": True, "physical_effect_verified": False}

    class Program:
        hardware = Hardware()

    store = OEMRuntimeStore(tmp_path / "store")
    handlers = OEMRuntimeCommandHandlers(startup_program=Program(), store=store)
    monkeypatch.setattr(handlers, "_write_stage_artifact", lambda *args, **kwargs: (_ for _ in ()).throw(OSError("synthetic artifact failure")))
    worker = OEMRuntimeWorker(store=store, handlers=handlers.handlers())
    worker.enqueue(OEMRuntimeCommand(
        name="startupHomingStepwise", mode="live", operator_ack="HOME",
        artifact_root=str(tmp_path / "artifact"), params={"homing_step": "z-home"},
    ))
    result = worker.run_next_for_tests()

    assert result["ok"] is False
    ledger = store.read_oem_movement_ledger()
    assert ledger is not None
    assert ledger["stages"]["z-home"]["state"] == "failed"
    assert ledger["terminal_state"] == "failed_closed"
    assert ledger["stages"]["z-home"]["result"]["physical_effect_verified"] is False
    assert ledger["stages"]["z-home"]["result"]["exception_type"] == "OSError"


def test_nonserializable_executor_result_still_persists_json_safe_failed_closed_ledger(tmp_path, monkeypatch):
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
    from src.bioxp import lifecycle_state as lifecycle_module
    from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers

    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason="test_transport_owned")
    lifecycle.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    lifecycle.run_stage("initialization_without_motion", lambda: {"ok": True})
    lifecycle.run_stage("initial_check", lambda: {"ok": True})
    monkeypatch.setattr(lifecycle_module, "lifecycle_state", lifecycle)

    class Hardware:
        def startup_homing_stepwise(self, **kwargs):
            return {
                "ok": True,
                "motion_command_attempted": True,
                "controller_acknowledged": True,
                "physical_effect_verified": False,
                "nonserializable": object(),
            }

    class Program:
        hardware = Hardware()

    store = OEMRuntimeStore(tmp_path / "store")
    handlers = OEMRuntimeCommandHandlers(startup_program=Program(), store=store)
    worker = OEMRuntimeWorker(store=store, handlers=handlers.handlers())
    worker.enqueue(OEMRuntimeCommand(
        name="startupHomingStepwise", mode="live", operator_ack="HOME",
        artifact_root=str(tmp_path / "artifact"), params={"homing_step": "z-home"},
    ))
    result = worker.run_next_for_tests()

    assert result["ok"] is False
    assert result["result"]["state"] == "failed_closed"
    ledger = store.read_oem_movement_ledger()
    assert ledger is not None
    row = ledger["stages"]["z-home"]
    assert row["state"] == "failed"
    assert ledger["terminal_state"] == "failed_closed"
    assert row["result"]["physical_effect_verified"] is False
    assert row["result"]["executor_result_omitted"] is True
    assert row["result"]["executor_result_summary"] == {
        "reported_ok": True,
        "motion_command_attempted": True,
        "controller_acknowledged": True,
        "reported_physical_effect_verified": False,
    }
    assert "executor_result" not in row["result"]


def test_terminal_system_status_is_persisted_in_robot_owned_ledger_not_ui_state(tmp_path):
    from src.bioxp.oem_movement_ledger import OemMovementLedger, OEM_INITIALIZE_MOTORS_STAGE_KEYS

    store = OEMRuntimeStore(tmp_path / "store")
    ledger = OemMovementLedger(store)
    target = "system-status-initialized"
    for stage in OEM_INITIALIZE_MOTORS_STAGE_KEYS:
        command_id = f"test-{stage}"
        admitted = ledger.admit(stage=stage, command_id=command_id)
        assert admitted["ok"] is True
        result = {"ok": True}
        if stage == target:
            result["durable_robot_state"] = {"system_status": 1, "ready": True}
        completed = ledger.record_result(stage=stage, command_id=command_id, result=result, artifact_path=None)
        if stage == target:
            assert completed["robot_state"] == {
                "system_status": 1,
                "ready": True,
                "source_anchor": "ClassControlInterface.initializeMotors:3416; M18 system status=1 and ready=true",
            }
            break
        if completed["stages"][stage]["requires_operator_observation"]:
            observed = ledger.record_observation(stage=stage, observed_pass=True, note="Observed required stage completion.", command_id=command_id)
            assert observed["ok"] is True

    persisted = store.read_oem_movement_ledger()
    assert persisted["robot_state"] == {
        "system_status": 1,
        "ready": True,
        "source_anchor": "ClassControlInterface.initializeMotors:3416; M18 system status=1 and ready=true",
    }
    assert "ui_positions" not in persisted["robot_state"]
