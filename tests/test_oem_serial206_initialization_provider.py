from __future__ import annotations

import json
import stat
from pathlib import Path

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp import oem_serial206_initialization as subject


ACK = {"status": 100, "value": 0}


def _write(value: int):
    return {"ok": True, "ack": dict(ACK), "set_value": value, "readback": {"ack": dict(ACK), "value": value}}


def _production_axis_home(axis: str):
    position = lambda value: {"ok": True, "ack": dict(ACK), "position": value}
    return {
        "axis": axis,
        "startup": True,
        "prepare": {"board": 4, "motor": 0, "ops": [{"op": "sap6", "set": 31, "ack": dict(ACK), "rb": {"ack": dict(ACK), "value": 31}}]},
        "home": {
            "ok": True,
            "home_active_value": 1,
            "position_before": position(123),
            "position_after": position(4),
            "position_after_sethome": position(0),
            "move_home": {"ok": True, "ack": dict(ACK)},
            "wait": {"stopped": True, "last_speed": 0, "last_ack": dict(ACK)},
            "stop": {"ok": True, "ack": dict(ACK)},
            "home_after": {"ok": True, "ack": dict(ACK), "value": 1},
            "set_home": _write(0),
            "seen_motion": True,
            "switch_transition": True,
            "home_predicate_confirmed": True,
        },
        "restore_current": None,
    }


class FakeSerial206Primitives:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def motor_oem_home_axis(self, axis, *, startup, speed=None, timeout_s):
        assert (axis, startup, speed) == ("z", True, 1791)
        self.calls.append("z-home")
        return _production_axis_home(axis)

    def motor_set_axis_param(self, board, param, value, motor=0):
        assert (board, motor, param, value) == (4, 2, 6, 31)
        self.calls.append("gripper-current-31")
        return _write(value)

    def z_manual_home(self, *, timeout_s=30.0):
        self.calls.append("manual-home")
        return {"ok": True, "source_method": "goHome(true,1791)", "timeout_s": timeout_s}

    def z_diagnostic_home_axis(self, *, timeout_s=30.0):
        self.calls.append("diagnostic-home-axis")
        return {"ok": True, "source_method": "HomeAxis(z,597)", "timeout_s": timeout_s}

    def z_move_steps(self, steps, *, timeout_s=20.0):
        self.calls.append(f"move-steps:{steps}")
        return {"ok": True, "source_method": "moveSteps", "steps": steps, "timeout_s": timeout_s}

    def z_move_absolute(self, requested_steps, pseudo_home_steps, *, timeout_s=20.0):
        self.calls.append(f"move-absolute:{requested_steps}:{pseudo_home_steps}")
        return {"ok": True, "source_method": "moveZ", "effective_target_steps": max(requested_steps, pseudo_home_steps), "timeout_s": timeout_s}

    def z_stop(self):
        self.calls.append("z-stop")
        return {"ok": True, "source_method": "StopMotor double delivery"}

    def z_reconcile_switch_masks(self):
        self.calls.append("reconcile-switch-masks")
        return {"ok": True, "source_method": "explicit_linux_recovery"}


class PreparationProvider:
    def __init__(self, calls: list[str], generation: int = 11) -> None:
        self.calls = calls
        self.generation = generation
        self.board_generation = 41

    def prepare_for_initialize_motors(self, *, expected_generation: int):
        self.calls.append("prepare")
        return {
            "ok": expected_generation == self.generation,
            "observed_generation": self.generation,
            "board_lifecycle_generation": self.board_generation,
            "board_preparation_verified": True,
            "initialize_without_motion_verified": True,
            "physical_motion": False,
        }

    def current_board_lifecycle_generation(self):
        return self.board_generation


class ReferenceStore:
    def __init__(self) -> None:
        self.transitions: list[tuple[str, str]] = []

    def mark_referenced(self, command):
        axis = command.axis.value if hasattr(command.axis, "value") else str(command.axis)
        self.transitions.append(("referenced", axis))
        return {
            "axis": axis,
            "state": "referenced",
            "ok": True,
            "persisted": True,
            "verified": True,
            "durable_clean": True,
        }

    def mark_desynced(self, command):
        axis = command.axis.value if hasattr(command.axis, "value") else str(command.axis)
        self.transitions.append(("desynced", axis))
        return {"axis": axis, "state": "desynced", "persisted": True}


def _commissioning(generation: int = 11):
    rows = {}
    for component in ("z", "g", "x", "y", "door"):
        rows[component] = subject.Serial206CommissioningEvidence(
            component=component,
            generation=generation,
            fresh=True,
            direction_verified=True,
            limits_verified=True,
            switch_verified=True,
            stop_verified=True,
            reference_verified=True,
            gap9_polarity=1 if component == "z" else None,
            gap10_polarity=0 if component == "z" else None,
        )
    return rows


def _approval(stage: str, *, approval_id: str | None = None, generation: int = 11):
    spec = {row.key: row for row in subject.SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS}[stage]
    return subject.Serial206StageApproval(
        approval_id=approval_id or f"approval-{stage}",
        expected_generation=generation,
        expected_component=spec.component,
        expected_direction=spec.direction,
        expected_bound=spec.bound,
        operator_note=f"Approve exact stage {stage}",
        idempotency_key=f"idempotency-{stage}",
    )


def _provider(tmp_path: Path, primitives: FakeSerial206Primitives, references: ReferenceStore | None = None):
    store = OEMRuntimeStore(tmp_path / "runtime")
    return subject.Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        reference_store=references,
        generation_provider=lambda: 11,
        preparation_provider=PreparationProvider(primitives.calls),
        sleep=lambda _: None,
    )


def test_z_first_stage_requires_only_source_specific_z_commissioning_and_not_prior_reference(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)
    z_only = subject.Serial206CommissioningEvidence(
        component="z",
        generation=11,
        fresh=True,
        direction_verified=True,
        limits_verified=True,
        switch_verified=True,
        stop_verified=True,
        reference_verified=False,
        gap9_polarity=1,
        gap10_polarity=0,
    )

    result = provider.initialize_motors(
        mode="live",
        approval=_approval("z-home"),
        commissioning={"z": z_only},
    )

    assert result["ok"] is True
    assert result["stage_receipts"][0]["stage"] == "z-home"
    assert not any("commissioning_evidence_required" in row for row in result.get("blockers", []))
    assert not any("reference_verified_required" in row for row in result.get("blockers", []))


def test_z_first_stage_rejects_non_source_gap9_gap10_polarity(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)
    reversed_z = subject.Serial206CommissioningEvidence(
        component="z",
        generation=11,
        fresh=True,
        direction_verified=True,
        limits_verified=True,
        switch_verified=True,
        stop_verified=True,
        reference_verified=False,
        gap9_polarity=0,
        gap10_polarity=1,
    )

    result = provider.initialize_motors(
        mode="live",
        approval=_approval("z-home"),
        commissioning={"z": reversed_z},
    )

    assert result["ok"] is False
    assert "z_gap9_gap10_polarity_must_match_oem_gap9_active_gap10_inactive" in result["blockers"]
    assert primitives.calls == []


def test_live_call_executes_only_exact_next_stage_and_never_auto_observes(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)

    result = provider.initialize_motors(
        mode="live",
        approval=_approval("z-home"),
        commissioning=_commissioning(),
    )

    assert result["ok"] is True
    assert result["ready"] is False
    assert result["state"] == "awaiting_operator_observation"
    assert [row["stage"] for row in result["stage_receipts"]] == ["z-home"]
    assert primitives.calls == ["prepare", "z-home"]
    row = result["movement_ledger"]["stages"]["z-home"]
    assert row["state"] == "acknowledged"
    assert row["observation"] is None
    assert result["movement_ledger"]["expected_next_stage"] == "z-home"
    assert references.transitions == []


def test_observation_is_strict_no_hardware_transition_then_next_call_runs_one_stage(tmp_path):
    first_primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    first = _provider(tmp_path, first_primitives, references)
    executed = first.initialize_motors(
        mode="live", approval=_approval("z-home"), commissioning=_commissioning()
    )
    command_id = executed["stage_receipts"][0]["approval_id"]

    restarted_primitives = FakeSerial206Primitives()
    restarted = _provider(tmp_path, restarted_primitives, references)
    observed = restarted.record_observation(
        stage="z-home",
        command_id=command_id,
        expected_generation=11,
        observed_pass=True,
        note="Christian observed correct z home movement and stop.",
    )

    assert observed["ok"] is True
    assert observed["physical_motion_commanded"] is False
    assert restarted_primitives.calls == []
    assert observed["movement_ledger"]["expected_next_stage"] == "gripper-current-31"
    assert references.transitions == [("referenced", "z")]

    second = restarted.initialize_motors(
        mode="live",
        approval=_approval("gripper-current-31"),
        commissioning=_commissioning(),
    )
    assert second["ok"] is True
    assert [row["stage"] for row in second["stage_receipts"]] == ["gripper-current-31"]
    assert restarted_primitives.calls == ["gripper-current-31"]
    assert second["movement_ledger"]["expected_next_stage"] == "gripper-clear-10000"


@pytest.mark.parametrize("observed_pass", ["true", "false", 0, 1, None])
def test_observation_rejects_non_boolean_without_hardware_or_state_change(tmp_path, observed_pass):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)
    provider.initialize_motors(mode="live", approval=_approval("z-home"), commissioning=_commissioning())
    before = provider.projection()
    primitives.calls.clear()

    result = provider.record_observation(
        stage="z-home",
        command_id="approval-z-home",
        expected_generation=11,
        observed_pass=observed_pass,
        note="A real note",
    )

    assert result["ok"] is False
    assert "observed_pass_must_be_boolean" in result["blockers"]
    assert primitives.calls == []
    assert provider.projection() == before


def test_restart_resumes_expected_next_stage_without_replaying_acknowledged_stage(tmp_path):
    first_primitives = FakeSerial206Primitives()
    _provider(tmp_path, first_primitives).initialize_motors(
        mode="live", approval=_approval("z-home"), commissioning=_commissioning()
    )

    restarted_primitives = FakeSerial206Primitives()
    result = _provider(tmp_path, restarted_primitives).initialize_motors(
        mode="live", approval=_approval("z-home", approval_id="different-approval"), commissioning=_commissioning()
    )

    assert result["ok"] is False
    assert result["state"] == "awaiting_operator_observation"
    assert "operator_observation_required:z-home" in result["blockers"]
    assert restarted_primitives.calls == []


def test_used_approval_is_rejected_after_restart_before_any_primitive(tmp_path):
    first_primitives = FakeSerial206Primitives()
    first = _provider(tmp_path, first_primitives)
    first.initialize_motors(mode="live", approval=_approval("z-home", approval_id="single-use"), commissioning=_commissioning())
    first.record_observation(
        stage="z-home",
        command_id="single-use",
        expected_generation=11,
        observed_pass=True,
        note="Observed correct movement.",
    )

    restarted_primitives = FakeSerial206Primitives()
    result = _provider(tmp_path, restarted_primitives).initialize_motors(
        mode="live",
        approval=_approval("gripper-current-31", approval_id="single-use"),
        commissioning=_commissioning(),
    )

    assert result["ok"] is False
    assert "approval_id_already_used:single-use" in result["blockers"]
    assert restarted_primitives.calls == []


def test_corrupt_durable_state_fails_closed_before_any_primitive(tmp_path):
    root = tmp_path / "runtime"
    store = OEMRuntimeStore(root)
    state_path = store.serial206_initialization_state_path
    state_path.write_text("{ definitely not JSON")
    primitives = FakeSerial206Primitives()
    provider = subject.Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        generation_provider=lambda: 11,
        preparation_provider=PreparationProvider(primitives.calls),
    )

    result = provider.initialize_motors(
        mode="live", approval=_approval("z-home"), commissioning=_commissioning()
    )

    assert result["ok"] is False
    assert result["state"] == "failed_closed"
    assert "durable_serial206_state_corrupt" in result["blockers"]
    assert primitives.calls == []


def test_single_atomic_state_file_contains_all_ledgers_with_restrictive_permissions(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)
    provider.initialize_motors(mode="live", approval=_approval("z-home"), commissioning=_commissioning())
    store = provider.state_store
    path = store.serial206_initialization_state_path
    payload = json.loads(path.read_text())

    assert set(payload) >= {"movement_ledger", "used_approvals", "initialize_motion_ledger"}
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert stat.S_IMODE(path.parent.stat().st_mode) == 0o700
    assert not path.with_suffix(path.suffix + ".tmp").exists()


def test_initialize_motion_live_is_truthfully_unavailable_when_exact_sequence_is_partial(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)

    status = provider.capability_status()
    result = provider.initialize_motion(mode="live")

    assert status["initialize_motors_live_available"] is True
    assert status["initialize_motion_live_available"] is False
    assert "initializeMotion.scriptmoveTo.tip_exists" in status["initialize_motion_missing_primitives"]
    assert result["ok"] is False
    assert result["physical_motion_commanded"] is False
    assert "initialize_motion_exact_primitives_not_bound" in result["blockers"]
    assert primitives.calls == []


def test_provider_owns_durable_z_prepare_home_observation_and_move_lifecycle(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)

    prepared = provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="z-prepare-1"
    )
    assert prepared["ok"] is True
    assert prepared["z_state"] == "prepared_unreferenced"

    rejected_move = provider.execute_z_intent(
        "move_steps",
        expected_generation=11,
        idempotency_key="z-move-before-home",
        inputs={"steps": 25},
    )
    assert rejected_move["ok"] is False
    assert rejected_move["blockers"] == ["z_state_blocks_intent:prepared_unreferenced:move_steps"]

    home = provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="z-home-1"
    )
    assert home["ok"] is True
    assert home["z_state"] == "awaiting_operator_observation"
    assert references.transitions == [("desynced", "z"), ("desynced", "z")]

    command_id = home["authority_receipt"]["command_id"]
    observation = provider.record_z_observation(
        command_id=command_id,
        verdict="pass",
        note="Observed Z shaft reach the home switch and stop",
        expected_generation=11,
    )
    assert observation["ok"] is True
    assert observation["z_state"] == "referenced_ready"
    assert references.transitions[-1] == ("referenced", "z")

    move = provider.execute_z_intent(
        "move_steps",
        expected_generation=11,
        idempotency_key="z-move-after-home",
        inputs={"steps": 25},
    )
    assert move["ok"] is True
    assert move["z_state"] == "referenced_ready"
    assert primitives.calls == ["prepare", "manual-home", "move-steps:25"]


def test_z_motion_failure_attempts_hardware_stop_and_latches_reference_desynced(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    provider.execute_z_intent("prepare", expected_generation=11, idempotency_key="prepare-failure-case")
    home = provider.execute_z_intent("manual_home", expected_generation=11, idempotency_key="home-failure-case")
    provider.record_z_observation(
        command_id=home["authority_receipt"]["command_id"],
        verdict="pass",
        note="Observed home before failure case",
        expected_generation=11,
    )
    primitives.z_move_steps = lambda steps, timeout_s=20.0: {
        "ok": False,
        "failure": "target_event_128_missing",
        "controller_command_acknowledged": True,
        "controller_terminal_state_verified": False,
    }

    failed = provider.execute_z_intent(
        "move_steps",
        expected_generation=11,
        idempotency_key="move-failure-case",
        inputs={"steps": 100},
    )

    assert failed["ok"] is False
    assert failed["z_state"] == "failed_latched"
    assert failed["result"]["failure_stop"]["ok"] is True
    assert primitives.calls[-1] == "z-stop"
    assert references.transitions[-1] == ("desynced", "z")


def test_non_provider_board4_activation_invalidates_preparation_and_reference(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="z-prepare-before-reactivation"
    )["ok"] is True

    invalidated = provider.notify_board_activation(4, {"status": 100})

    assert invalidated["z_affected"] is True
    projection = provider.z_projection()
    assert projection["state"] == "unprepared"
    assert projection["reference_state"] == "desynced"
    assert projection["last_failure"]["reason"] == "board4_command64_outside_provider_preparation"
    assert references.transitions[-1] == ("desynced", "z")


def test_provider_switch_mask_recovery_is_explicit_confirmed_and_requires_reprepare(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)

    denied = provider.execute_z_intent(
        "reconcile_switch_masks",
        expected_generation=11,
        idempotency_key="z-mask-denied",
        inputs={"confirm": "wrong"},
    )
    assert denied["ok"] is False
    assert any(row.startswith("explicit_confirmation_required") for row in denied["blockers"])
    assert "reconcile-switch-masks" not in primitives.calls

    recovered = provider.execute_z_intent(
        "reconcile_switch_masks",
        expected_generation=11,
        idempotency_key="z-mask-recovery",
        inputs={"confirm": "RECONCILE_Z_SWITCH_MASKS"},
    )
    assert recovered["ok"] is True
    assert recovered["z_state"] == "unprepared"
    assert primitives.calls == ["reconcile-switch-masks"]


def test_production_adapter_exposes_only_source_grounded_existing_primitives(monkeypatch):
    class Tester:
        def motor_oem_home_axis(self, *args, **kwargs): return {"ok": True}
        def motor_set_axis_param(self, *args, **kwargs): return {"ok": True}
        def motor_move_relative(self, *args, **kwargs): return {"ok": True}
        def motor_wait_stopped(self, *args, **kwargs): return {"stopped": True}
        def motor_get_position(self, *args, **kwargs): return {"ok": True, "ack": dict(ACK), "position": 0}
        def motor_set_home(self, *args, **kwargs): return {"ok": True}
        def motor_move_absolute(self, *args, **kwargs): return {"ok": True}
        def motor_oem_door_search_home(self, *args, **kwargs): return {"ok": True}
        def motor_thermal_door_status(self): return {"ok": True}
        def _machine_config_bundle(self):
            return {"ok": True, "config": {"config": {"GripperVersion": 1}, "calibration": {"Calibrated": 1}}}
        def chiller_gp_write(self, *args, **kwargs): return {"ok": True, "verified": True, "ack": dict(ACK), "readback": {"ok": True, "value": args[2]}}
        def oem_set_calibrated_ui_positions_zero(self): return {"ok": True}

    class Pipettes:
        def query_tip_status_all(self): return {"ok": True, "channels": []}
        def eject_all_tips_for_oem_startup(self, **kwargs): return {"ok": True}
        def initialize(self, command): return {"ok": True}

    monkeypatch.setattr(subject, "prepare_motion_without_motion", lambda tester, authority, components=None: {
        "ok": True,
        "physical_motion": False,
        "stage_ledger": [],
    })
    adapter = subject.Serial206ProductionPrimitiveAdapter(
        Tester(), Pipettes(), authority_provider=lambda: object(), generation_provider=lambda: 7
    )

    status = adapter.capability_status()
    prep = adapter.prepare_for_initialize_motors(expected_generation=7)

    assert status["initialize_motors_exact_primitives_bound"] is False
    assert status["initialize_motion_complete"] is False
    assert "mandatory_primitive_not_bound:motor_prepare_axis" in status["initialize_motors_binding_blockers"]
    assert "mandatory_primitive_not_bound:motor_oem_open_thermal_door" in status["initialize_motors_binding_blockers"]
    assert not hasattr(adapter, "scriptmove_to")
    assert not hasattr(adapter, "set_tip_loaded")
    assert prep["ok"] is True
    assert prep["physical_motion"] is False


def test_z_manual_home_rejects_stale_board_lifecycle_generation(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)

    prepared = provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-board-generation"
    )
    assert prepared["ok"] is True
    assert prepared["z_lifecycle"]["board_lifecycle_generation"] == 41

    provider.preparation_provider.board_generation = 42
    projection = provider.z_projection()
    assert projection["state"] == "unprepared"
    assert projection["durable_state_before_board_invalidation"] == "prepared_unreferenced"
    assert projection["board_lifecycle_generation_fresh"] is False

    home = provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="stale-board-home"
    )

    assert home["ok"] is False
    assert "z_state_blocks_intent:unprepared:manual_home" in home["blockers"]
    assert "manual-home" not in primitives.calls

    recovered = provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-after-board-generation-change"
    )
    assert recovered["ok"] is True
    assert recovered["z_lifecycle"]["board_lifecycle_generation"] == 42


def test_diagnostic_home_requires_named_confirmation_and_never_awaits_reference_observation(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-for-diagnostic"
    )["ok"] is True

    rejected = provider.execute_z_intent(
        "diagnostic_home_axis",
        expected_generation=11,
        idempotency_key="diagnostic-without-confirm",
    )
    assert rejected["ok"] is False
    assert "explicit_confirmation_required:DIAGNOSTIC_Z_HOME_597" in rejected["blockers"]

    diagnosed = provider.execute_z_intent(
        "diagnostic_home_axis",
        inputs={"confirm": "DIAGNOSTIC_Z_HOME_597"},
        expected_generation=11,
        idempotency_key="diagnostic-with-confirm",
    )
    assert diagnosed["ok"] is True
    assert diagnosed["z_state"] == "prepared_unreferenced"
    assert diagnosed["z_lifecycle"]["awaiting_observation_receipt_id"] is None


def test_production_adapter_reports_command_and_terminal_receipts_from_nested_move_z_home():
    class Tester:
        def _motion_oem_axis_profile(self, axis):
            assert axis == "z"
            return {"board": 4, "motor": 1}

        def motor_oem_require_no_motion_profile(self, axis):
            assert axis == "z"
            return {"ok": True}

        def motor_oem_verify_motion_interlock(self):
            return {"ok": True}

        def motor_oem_move_z_home(self, *, rehome, timeout_s):
            assert rehome is True
            return {
                "ok": True,
                "home": {
                    "ok": True,
                    "move_home": {"ok": True},
                    "stop": {"ok": True},
                    "wait": {"stopped": True, "last_speed": 0},
                },
            }

    adapter = subject.Serial206ProductionPrimitiveAdapter(
        Tester(), object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )
    result = adapter.z_manual_home(timeout_s=30.0)

    assert result["ok"] is True
    assert result["controller_command_acknowledged"] is True
    assert result["controller_terminal_state_verified"] is True
    assert result["motor_output_state"] == "unknown"
    assert result["physical_effect_verified"] is False


def test_z_observation_rejects_board_generation_change_after_home(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-before-observation-generation"
    )["ok"] is True
    homed = provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="home-before-observation-generation"
    )
    command_id = homed["authority_receipt"]["command_id"]
    provider.preparation_provider.board_generation = 42

    with pytest.raises(ValueError, match="board lifecycle generation changed"):
        provider.record_z_observation(
            command_id=command_id,
            verdict="pass",
            note="Observation arrived after a board lifecycle change.",
            expected_generation=11,
        )


def test_z_reference_is_not_published_when_durable_persistence_fails(tmp_path):
    class FailingReferenceStore(ReferenceStore):
        def mark_referenced(self, command):
            axis = command.axis.value if hasattr(command.axis, "value") else str(command.axis)
            self.transitions.append(("referenced_failed", axis))
            return {
                "axis": axis,
                "state": "unknown",
                "ok": False,
                "persisted": False,
                "verified": False,
                "durable_clean": False,
                "error": "simulated persistence failure",
            }

    primitives = FakeSerial206Primitives()
    references = FailingReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-before-persistence-failure"
    )["ok"] is True
    homed = provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="home-before-persistence-failure"
    )

    observed = provider.record_z_observation(
        command_id=homed["authority_receipt"]["command_id"],
        verdict="pass",
        note="Physical observation passed but persistence is forced to fail.",
        expected_generation=11,
    )

    assert observed["ok"] is False
    assert observed["error"] == "z_reference_persistence_failed"
    assert observed["z_state"] == "failed_latched"
    assert observed["z_lifecycle"]["reference_state"] == "desynced"
