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


class PreparationProvider:
    def __init__(self, calls: list[str], generation: int = 11) -> None:
        self.calls = calls
        self.generation = generation

    def prepare_for_initialize_motors(self, *, expected_generation: int):
        self.calls.append("prepare")
        return {
            "ok": expected_generation == self.generation,
            "observed_generation": self.generation,
            "board_preparation_verified": True,
            "initialize_without_motion_verified": True,
            "physical_motion": False,
        }


class ReferenceStore:
    def __init__(self) -> None:
        self.transitions: list[tuple[str, str]] = []

    def mark_referenced(self, command):
        axis = command.axis.value if hasattr(command.axis, "value") else str(command.axis)
        self.transitions.append(("referenced", axis))
        return {"axis": axis, "state": "referenced", "persisted": True}

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
        def _oem_no_motion_tmcl_with_readback(self, **kwargs): return {"ok": True}
        def oem_set_calibrated_ui_positions_zero(self): return {"ok": True}

    class Pipettes:
        def query_tip_status_all(self): return {"ok": True, "channels": []}
        def eject_all_tips_for_oem_startup(self, **kwargs): return {"ok": True}
        def initialize(self, command): return {"ok": True}

    monkeypatch.setattr(subject, "prepare_motion_without_motion", lambda tester, authority: {
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
