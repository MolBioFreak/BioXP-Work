from __future__ import annotations

import json
import stat
import threading
from pathlib import Path
from typing import Callable

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp import oem_serial206_initialization as subject


ACK = {"status": 100, "value": 0}


def test_terminal_projection_preserves_stopped_state_across_no_motion_set_home():
    z = {
        "receipts": [
            {
                "status": "completed",
                "intent": "move_absolute",
                "command_id": "move-1",
                "finished_at": 10.0,
                "result": {
                    "after_position_steps": 92049,
                    "controller_terminal_state_verified": True,
                },
            },
            {
                "status": "completed",
                "intent": "set_home",
                "command_id": "home-1",
                "finished_at": 11.0,
                "result": {
                    "physical_motion": False,
                    "position": {"position": 0},
                    "controller_terminal_state_verified": True,
                    "terminal_z_state": {
                        "ok": True,
                        "position_steps": 0,
                        "speed_steps_s": 0,
                        "left_switch_state": 1,
                        "right_switch_state": 1,
                        "left_switch_disabled": False,
                        "right_switch_disabled": False,
                    },
                },
            },
        ]
    }

    assert subject.Serial206OemInitializationProvider._z_terminal_state_from_receipts(z) == {
        "position_steps": 0,
        "speed_steps_s": 0,
        "left_switch_state": 1,
        "right_switch_state": 1,
        "left_switch_disabled": False,
        "right_switch_disabled": False,
        "source_command_id": "home-1",
        "observed_at": 11.0,
        "authority": "provider_receipt_terminal_state",
    }


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
        return {
            "ok": True,
            "source_method": "goHome(true,1791)",
            "timeout_s": timeout_s,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }

    def z_diagnostic_home_axis(self, *, timeout_s=30.0):
        self.calls.append("diagnostic-home-axis")
        return {
            "ok": True,
            "source_method": "HomeAxis(z,597)",
            "timeout_s": timeout_s,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }

    def z_move_steps(self, *, steps, wait_timeout_s=20.0):
        self.calls.append(f"move-steps:{steps}")
        return {
            "ok": True,
            "source_method": "moveSteps",
            "steps": steps,
            "timeout_s": wait_timeout_s,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }

    def z_move_absolute(self, *, requested_position_steps, pseudo_home_steps, wait_timeout_s=20.0):
        self.calls.append(f"move-absolute:{requested_position_steps}:{pseudo_home_steps}")
        return {
            "ok": True,
            "source_method": "moveZ",
            "effective_target_steps": max(requested_position_steps, pseudo_home_steps),
            "timeout_s": wait_timeout_s,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }

    def z_execute_path(self, *, steps, wait_timeout_s, pseudo_home_steps):
        self.calls.append("path")
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "execution": [{"ok": True, "op": dict(steps[0]).get("op")}],
        }

    def z_set_current_max(self, value=None):
        selected = 31 if value in {None, 100} else value
        return {
            "ok": True,
            "param": 6,
            "value": selected,
            "readback": {"ok": True, "ack": {"status": 100}, "value": selected},
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "physical_motion": False,
        }

    def z_set_home(self):
        self.calls.append("set-home")
        return {
            "ok": True,
            "source_method": "ClassMotor.setHome",
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "reference_persistence": {"ok": True, "persisted": True, "verified": True, "durable_clean": True},
        }

    def z_stop(self, *, timeout_s=3.0):
        self.calls.append("z-stop")
        return {
            "ok": True,
            "source_method": "StopMotor double delivery",
            "timeout_s": timeout_s,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }

    def z_clear_profile_overrides(self):
        return None

    def z_reconcile_switch_masks(self):
        self.calls.append("reconcile-switch-masks")
        return {"ok": True, "source_method": "explicit_linux_recovery"}


class PreparationProvider:
    def __init__(self, calls: list[str], generation: int = 11) -> None:
        self.calls = calls
        self.generation = generation
        self.board_generation = 41
        self.observer: Callable[..., object] | None = None

    def prepare_for_initialize_motors(self, *, expected_generation: int):
        self.calls.append("prepare")
        if self.observer is not None:
            self.observer(4, {"status": 100}, active=False)
            self.observer(4, {"status": 100}, active=True)
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

    def snapshot(self, axes):
        return {
            "ok": True,
            "rows": {str(axis): {"axis": str(axis), "state": "referenced"} for axis in axes},
        }


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
    preparation = PreparationProvider(primitives.calls)
    provider = subject.Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        reference_store=references,
        generation_provider=lambda: 11,
        preparation_provider=preparation,
        sleep=lambda _: None,
    )
    preparation.observer = provider.notify_board_activation
    return provider


def test_z_terminal_authority_is_saved_before_sql_receipt(tmp_path, monkeypatch):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    events: list[tuple[str, str]] = []
    real_save = provider._save_state
    real_persist = provider._persist_z_receipt

    def save(state):
        events.append(("authority", str(state["z_lifecycle"]["state"])))
        return real_save(state)

    def persist(row):
        events.append(("sql", str(row["status"])))
        return real_persist(row)

    monkeypatch.setattr(provider, "_save_state", save)
    monkeypatch.setattr(provider, "_persist_z_receipt", persist)

    result = provider.execute_z_intent(
        "prepare",
        expected_generation=11,
        idempotency_key="z-authority-order",
    )

    assert result["ok"] is True
    assert events[-2:] == [("authority", "prepared_unreferenced"), ("sql", "completed")]


def test_z_sql_failure_after_authority_save_replays_without_redispatch(tmp_path, monkeypatch):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())

    def fail_persist(_row):
        raise OSError("injected serial receipt failure")

    monkeypatch.setattr(provider, "_persist_z_receipt", fail_persist)
    with pytest.raises(OSError, match="injected serial receipt failure"):
        provider.execute_z_intent(
            "prepare",
            expected_generation=11,
            idempotency_key="z-sql-failure",
        )
    dispatch_count = len(primitives.calls)
    state = provider.state_store.read_oem_serial206_initialization_state()
    assert state["z_lifecycle"]["state"] == "prepared_unreferenced"
    assert state["z_lifecycle"]["receipts"][-1]["idempotency_key"] == "z-sql-failure"

    restarted = _provider(tmp_path, primitives, ReferenceStore())
    replay = restarted.execute_z_intent(
        "prepare",
        expected_generation=11,
        idempotency_key="z-sql-failure",
    )

    assert replay["ok"] is True
    assert len(primitives.calls) == dispatch_count


def test_z_completed_replay_fails_closed_after_generation_invalidation(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare",
        expected_generation=11,
        idempotency_key="z-generation-bound",
    )["ok"] is True
    dispatch_count = len(primitives.calls)
    provider.generation_provider = lambda: 12

    replay = provider.execute_z_intent(
        "prepare",
        expected_generation=11,
        idempotency_key="z-generation-bound",
    )

    assert replay["ok"] is False
    assert replay["replayed"] is True
    assert replay["blockers"] == ["z_replay_authority_invalidated"]
    assert len(primitives.calls) == dispatch_count


def test_initialize_motion_projection_releases_provider_lock_before_copying_ledgers(tmp_path, monkeypatch):
    provider = _provider(tmp_path, FakeSerial206Primitives(), ReferenceStore())
    state = provider._load_state()
    lock_available: list[bool] = []
    real_projection = provider._ledger_status_projection

    monkeypatch.setattr(provider, "_load_state", lambda: state)

    def observed_projection(value):
        if value is state["initialize_motion_ledger"]:
            acquired: list[bool] = []

            def probe_lock():
                locked = provider._lock.acquire(blocking=False)
                acquired.append(locked)
                if locked:
                    provider._lock.release()

            thread = threading.Thread(target=probe_lock)
            thread.start()
            thread.join(timeout=1.0)
            lock_available.extend(acquired)
        return real_projection(value)

    monkeypatch.setattr(provider, "_ledger_status_projection", observed_projection)

    projection = provider.initialize_motion_projection()

    assert projection["initialize_motion_ledger"]["stage_receipt_count"] == 0
    assert "stage_receipts" not in projection["initialize_motion_ledger"]
    assert lock_available == [True]


@pytest.mark.parametrize("pseudo_home_steps", [500, 65000])
def test_z_clear_moves_to_robot_owned_pseudo_home(tmp_path, pseudo_home_steps):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key=f"clear-prepare-{pseudo_home_steps}"
    )["ok"] is True
    assert provider.execute_z_intent(
        "set_home",
        inputs={"operator_ack": "SET_HOME_CURRENT_POSITION"},
        expected_generation=11,
        idempotency_key=f"clear-home-{pseudo_home_steps}",
    )["ok"] is True
    state = provider.state_store.read_oem_serial206_initialization_state()
    state["machine_status"]["psudo_z_home_steps"] = pseudo_home_steps
    provider.state_store.write_oem_serial206_initialization_state(state)

    result = provider.execute_z_intent(
        "clear",
        inputs={"wait_timeout_s": 20.0},
        expected_generation=11,
        idempotency_key=f"clear-{pseudo_home_steps}",
    )

    assert result["ok"] is True
    assert result["z_state"] == "referenced_ready"
    assert result["z_lifecycle"]["state"] == "referenced_ready"
    assert result["z_lifecycle"]["reference_state"] == "referenced"
    assert primitives.calls[-1] == f"move-absolute:{pseudo_home_steps}:{pseudo_home_steps}"
    receipt = result["authority_receipt"]
    assert receipt["intent"] == "clear"
    assert receipt["result"]["selected_pseudo_home_steps"] == pseudo_home_steps
    persisted = provider.state_store.read_oem_serial206_initialization_state()["z_lifecycle"]
    assert persisted["state"] == "referenced_ready"
    assert persisted["reference_state"] == "referenced"
    assert persisted["last_failure"] is None


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


def test_provider_owns_durable_z_prepare_controller_home_and_move_lifecycle(tmp_path):
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
    assert home["z_state"] == "referenced_ready"
    assert home["result"]["reference_persistence"]["ok"] is True
    assert references.transitions == [("desynced", "z"), ("referenced", "z")]

    move = provider.execute_z_intent(
        "move_steps",
        expected_generation=11,
        idempotency_key="z-move-after-home",
        inputs={"steps": 25},
    )
    assert move["ok"] is True
    assert move["z_state"] == "referenced_ready"
    assert primitives.calls == ["prepare", "manual-home", "move-steps:25"]


def test_provider_accepts_oem_current_sentinel_100_and_resolves_machine_default(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())

    prepared = provider.execute_z_intent(
        intent="prepare",
        expected_generation=11,
        idempotency_key="z-prepare-current-sentinel",
    )
    assert prepared["ok"] is True

    result = provider.execute_z_intent(
        intent="set_current_max",
        inputs={"value": 100},
        expected_generation=11,
        idempotency_key="z-current-sentinel-100",
    )

    assert result["ok"] is True, result
    assert result["result"]["value"] == 31


def test_live_path_planning_authority_comes_from_provider_controller_and_durable_state(tmp_path):
    primitives = FakeSerial206Primitives()
    setattr(primitives, "_read_axis_position", lambda axis: {"x": 101, "y": 202, "z": 303}[axis])
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-path-authority"
    )["ok"] is True
    assert provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="home-path-authority"
    )["ok"] is True
    state = provider.state_store.read_oem_serial206_initialization_state()
    state["machine_status"].update({
        "tip_loaded": False,
        "tip_dirty": False,
        "tip_location": -1,
        "plate_on_gantry": None,
        "current_location": 6,
        "current_well": 0,
        "psudo_z_home_steps": 65000,
    })
    provider.state_store.write_oem_serial206_initialization_state(state)

    authority = provider.path_planning_authority(expected_generation=11)

    assert authority["ok"] is True
    assert (authority["current_x"], authority["current_y"], authority["current_z"]) == (101, 202, 303)
    assert authority["current_loc"] == 6
    assert authority["tip_loaded"] is False
    assert authority["clean_path"] is False
    assert authority["gripper_confirmed"] is True

    clean_mode = provider.execute_z_intent(
        intent="set_clean_path",
        inputs={"enabled": True},
        expected_generation=11,
        idempotency_key="clean-path-on",
    )
    assert clean_mode["ok"] is True
    assert clean_mode["result"]["clean_path_persisted"] is True
    assert provider.path_planning_authority(expected_generation=11)["clean_path"] is True

    executed = provider.execute_z_intent(
        intent="path_execute",
        inputs={
            "steps": [{"op": "moveZ", "z": 1000}],
            "path_context": {"current_location": "LOC_MS", "current_well": 25},
        },
        expected_generation=11,
        idempotency_key="path-authority-update",
    )
    assert executed["ok"] is True
    assert executed["result"]["path_context_persisted"] == {
        "current_location": "LOC_MS",
        "current_well": 25,
    }
    authority_after = provider.path_planning_authority(expected_generation=11)
    assert authority_after["current_loc"] == "LOC_MS"
    assert authority_after["current_well"] == 25


def test_live_path_authority_rejects_loaded_tip_without_authoritative_tip_location(tmp_path):
    primitives = FakeSerial206Primitives()
    setattr(primitives, "_read_axis_position", lambda axis: 0)
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-loaded-tip-authority"
    )["ok"] is True
    assert provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="home-loaded-tip-authority"
    )["ok"] is True
    state = provider.state_store.read_oem_serial206_initialization_state()
    state["machine_status"].update({
        "tip_loaded": True,
        "tip_dirty": False,
        "tip_location": -1,
        "current_location": 6,
        "current_well": 0,
    })
    provider.state_store.write_oem_serial206_initialization_state(state)

    authority = provider.path_planning_authority(expected_generation=11)

    assert authority == {"ok": False, "blockers": ["loaded_tip_location_not_authoritative"]}


def test_failed_manual_home_accepts_movement_only_observation_as_historical_annotation(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="historical-prepare"
    )["ok"] is True
    primitives.z_manual_home = lambda *, timeout_s=30.0: {
        "ok": False,
        "failure": "controller_async_error_130",
        "controller_command_acknowledged": True,
        "controller_terminal_state_verified": True,
    }
    failed = provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="historical-home"
    )
    authority_id = failed["authority_receipt"]["command_id"]

    annotated = provider.record_z_observation(
        command_id=authority_id,
        observation_command_id="operator-historical-observation",
        verdict="pass",
        physical_motion_observed=True,
        expected_direction_observed=True,
        home_endpoint_observed=False,
        stopped_observed=True,
        note="Physical movement observed; controller home proof failed.",
        expected_generation=11,
    )

    assert annotated["ok"] is True
    assert annotated["annotation_only"] is True
    assert annotated["z_state"] == "failed_latched"
    assert annotated["observation"]["authority_current"] is False
    assert annotated["observation"]["reference_eligible"] is False
    assert annotated["authority_receipt"]["physical_effect_verified"] is True
    assert annotated["authority_receipt"]["observation_receipt_id"] == "operator-historical-observation"
    durable_authority = provider.state_store.read_serial206_receipt("z", authority_id)
    assert durable_authority["physical_effect_verified"] is True
    assert durable_authority["observation_receipt_id"] == "operator-historical-observation"
    assert durable_authority["operator_assessment"]["verdict"] == "pass"
    assert references.transitions[-1] == ("desynced", "z")


def test_z_motion_failure_attempts_hardware_stop_and_latches_reference_desynced(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    provider.execute_z_intent("prepare", expected_generation=11, idempotency_key="prepare-failure-case")
    home = provider.execute_z_intent("manual_home", expected_generation=11, idempotency_key="home-failure-case")
    observed = provider.record_z_observation(
        command_id=home["authority_receipt"]["command_id"],
        verdict="pass",
        physical_motion_observed=True,
        expected_direction_observed=True,
        home_endpoint_observed=True,
        stopped_observed=True,
        note="Observed home before failure case",
        expected_generation=11,
    )
    durable_authority = provider.state_store.read_serial206_receipt(
        "z", home["authority_receipt"]["command_id"]
    )
    assert durable_authority["observation_receipt_id"] == observed["observation_receipt"]["command_id"]
    assert durable_authority["operator_assessment"]["verdict"] == "pass"
    primitives.z_move_steps = lambda *, steps, wait_timeout_s=20.0: {
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


def test_prepare_scope_rejects_unexpected_extra_board4_command64(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()

    class UnexpectedPreparation(PreparationProvider):
        def prepare_for_initialize_motors(self, *, expected_generation: int):
            result = super().prepare_for_initialize_motors(
                expected_generation=expected_generation
            )
            assert self.observer is not None
            self.observer(4, {"status": 100}, active=True)
            return result

    preparation = UnexpectedPreparation(primitives.calls)
    provider = subject.Serial206OemInitializationProvider(
        primitives,
        state_store=OEMRuntimeStore(tmp_path / "runtime"),
        reference_store=references,
        generation_provider=lambda: 11,
        preparation_provider=preparation,
        sleep=lambda _: None,
    )
    preparation.observer = provider.notify_board_activation

    result = provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="unexpected-command64"
    )

    assert result["ok"] is False
    assert result["result"]["failure"] == "z_prepare_board_transition_scope_violation"
    assert result["z_state"] == "failed_latched"
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
    provider = _provider(tmp_path, primitives, ReferenceStore())

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
    provider = _provider(tmp_path, primitives, ReferenceStore())

    prepared = provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-board-generation"
    )
    assert prepared["ok"] is True
    assert prepared["z_lifecycle"]["board_lifecycle_generation"] == 41

    provider.preparation_provider.board_generation = 42
    projection = provider.z_projection()
    assert projection["state"] == "unprepared"
    assert projection["last_failure"]["durable_state_before_invalidation"] == "prepared_unreferenced"
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


def test_diagnostic_home_has_no_non_oem_confirmation_or_observation_gate(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-for-diagnostic"
    )["ok"] is True

    diagnosed = provider.execute_z_intent(
        "diagnostic_home_axis",
        expected_generation=11,
        idempotency_key="diagnostic-without-confirm",
    )
    assert diagnosed["ok"] is True
    assert diagnosed["z_state"] == "referenced_ready"
    assert diagnosed["result"]["reference_persistence"]["ok"] is True
    assert diagnosed["z_lifecycle"]["awaiting_observation_receipt_id"] is None
    assert references.transitions[-1] == ("referenced", "z")


def test_production_adapter_reports_command_and_terminal_receipts_from_nested_move_z_home():
    class Tester:
        def _motion_oem_axis_profile(self, axis):
            assert axis == "z"
            return {"board": 4, "motor": 1}

        def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides=None):
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
                    "move_home": {"ok": True, "ack": dict(ACK)},
                    "stop": {"ok": True, "first_delivery": dict(ACK), "second_delivery": dict(ACK)},
                    "wait": {"stopped": True, "last_speed": 0, "last_ack": dict(ACK)},
                },
            }

    adapter = subject.Serial206ProductionPrimitiveAdapter(
        Tester(), object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )
    result = adapter.z_move_z_home(timeout_s=30.0)

    assert result["ok"] is True
    assert result["controller_command_acknowledged"] is True
    assert result["controller_terminal_state_verified"] is True
    assert result["motor_output_state"] == "stopped_readback"
    assert result["physical_effect_verified"] is False


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

    assert homed["ok"] is False
    assert homed["result"]["failure"].startswith("z_reference_publication_failed:")
    assert homed["z_state"] == "failed_latched"
    assert homed["z_lifecycle"]["reference_state"] == "desynced"


def test_reference_publication_is_compensated_when_final_lifecycle_commit_fails(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    backing = OEMRuntimeStore(tmp_path / "runtime")

    class FailReferencedReadyStore:
        fail = False

        def read_oem_serial206_initialization_state(self):
            return backing.read_oem_serial206_initialization_state()

        def write_oem_serial206_initialization_state(self, state):
            if self.fail and state["z_lifecycle"]["state"] == "referenced_ready":
                raise OSError("simulated final lifecycle commit failure")
            return backing.write_oem_serial206_initialization_state(state)

    store = FailReferencedReadyStore()
    preparation = PreparationProvider(primitives.calls)
    provider = subject.Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        reference_store=references,
        generation_provider=lambda: 11,
        preparation_provider=preparation,
        sleep=lambda _: None,
    )
    preparation.observer = provider.notify_board_activation
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="atomic-prepare"
    )["ok"] is True
    store.fail = True

    with pytest.raises(OSError, match="final lifecycle commit failure"):
        provider.execute_z_intent(
            "set_home",
            inputs={"operator_ack": "SET_HOME_CURRENT_POSITION"},
            expected_generation=11,
            idempotency_key="atomic-set-home",
        )

    assert references.transitions[-2:] == [("referenced", "z"), ("desynced", "z")]
    store.fail = False
    restarted = subject.Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        reference_store=references,
        generation_provider=lambda: 11,
        preparation_provider=preparation,
        sleep=lambda _: None,
    )
    projection = restarted.z_projection()
    assert projection["state"] == "failed_latched"
    assert projection["reference_state"] == "desynced"


def test_legacy_migration_refuses_lifecycle_rewrite_when_reference_invalidation_fails(tmp_path):
    backing = OEMRuntimeStore(tmp_path / "runtime")
    primitives = FakeSerial206Primitives()
    seed = _provider(tmp_path, primitives, ReferenceStore())
    state = seed._new_state()
    state["z_lifecycle"].update(
        {
            "schema_version": "bioxp.serial206_z_lifecycle.v1",
            "state": "referenced_ready",
            "generation": 11,
            "reference_state": "referenced",
            "prepared_receipt": {"ok": True},
        }
    )
    state["z_lifecycle"].pop("board_lifecycle_generation", None)
    backing.write_oem_serial206_initialization_state(state)

    class FailingReferenceStore(ReferenceStore):
        def mark_desynced(self, command):
            self.transitions.append(("desynced_failed", "z"))
            return {
                "axis": "z",
                "state": "referenced",
                "ok": False,
                "persisted": False,
                "verified": False,
                "durable_clean": False,
            }

    provider = subject.Serial206OemInitializationProvider(
        primitives,
        state_store=backing,
        reference_store=FailingReferenceStore(),
        generation_provider=lambda: 11,
        preparation_provider=PreparationProvider(primitives.calls),
        sleep=lambda _: None,
    )

    projection = provider.z_projection()
    assert projection["available"] is False
    assert "reference invalidation" in projection["failure"]
    stored = backing.read_oem_serial206_initialization_state()
    assert stored["z_lifecycle"]["schema_version"] == "bioxp.serial206_z_lifecycle.v1"
    assert stored["z_lifecycle"]["state"] == "referenced_ready"


def test_generation_drift_projection_durably_invalidates_lifecycle_and_reference(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="projection-prepare"
    )["ok"] is True
    assert provider.execute_z_intent(
        "set_home",
        inputs={"operator_ack": "SET_HOME_CURRENT_POSITION"},
        expected_generation=11,
        idempotency_key="projection-set-home",
    )["ok"] is True
    provider.preparation_provider.board_generation += 1

    projection = provider.z_projection()
    stored = provider.state_store.read_oem_serial206_initialization_state()["z_lifecycle"]

    assert projection["state"] == "unprepared"
    assert stored["state"] == "unprepared"
    assert stored["reference_state"] == "desynced"
    assert references.transitions[-1] == ("desynced", "z")


def test_z_observation_cannot_publish_reference_without_durable_reference_store(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-no-reference-store"
    )["ok"] is True
    provider.reference_store = None
    home = provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="home-no-reference-store"
    )

    assert home["ok"] is False
    assert home["blockers"] == ["z_reference_store_not_bound"]
    assert "manual-home" not in primitives.calls


def test_z_position_value_accepts_controller_position_key_without_coercion():
    adapter = object.__new__(subject.Serial206ProductionPrimitiveAdapter)

    assert adapter._z_value({"position": 0}) == 0
    assert adapter._z_value({"position": 123}) == 123
    assert adapter._z_value({"value": 31}) == 31
    assert adapter._z_value({"position": True}) is None
    assert adapter._z_value({"position": "0"}) is None


def test_z_set_home_accepts_exact_controller_position_shape_and_leaves_persistence_to_provider():
    class Tester:
        def _motion_oem_axis_profile(self, axis):
            assert axis == "z"
            return {"board": 4, "motor": 1}

        def motor_set_home(self, board, *, motor):
            assert (board, motor) == (4, 1)
            return {"ok": True, "ack": dict(ACK), "readback": {"value": 0}}

        def motor_get_position(self, board, *, motor):
            assert (board, motor) == (4, 1)
            return {"ok": True, "ack": dict(ACK), "position": 0}

    class ForbiddenReferenceStore:
        def mark_referenced(self, command):
            raise AssertionError(f"primitive must not publish reference authority: {command}")

    adapter = subject.Serial206ProductionPrimitiveAdapter(
        Tester(), object(), authority_provider=lambda: object(), generation_provider=lambda: 11,
        reference_store=ForbiddenReferenceStore(),
    )

    result = adapter.z_set_home()

    assert result["ok"] is True
    assert result["controller_command_acknowledged"] is True
    assert result["controller_terminal_state_verified"] is True
    assert result["reference_publication_required"] is True
    assert result["reference_publication_owner"] == "Serial206OemInitializationProvider"
    assert result["physical_motion"] is False


def test_provider_dispatches_relative_and_absolute_moves_through_guarded_z_primitives(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-guarded-moves"
    )["ok"] is True
    assert provider.execute_z_intent(
        "set_home",
        inputs={
            "command_id": "operator-set-home-1",
            "operator_ack": "SET_HOME_CURRENT_POSITION",
        },
        expected_generation=11,
        idempotency_key="set-home-guarded-moves",
    )["ok"] is True

    relative = provider.execute_z_intent(
        "move_steps",
        inputs={"steps": 25, "wait_timeout_s": 7.0, "command_id": "operator-relative-1"},
        expected_generation=11,
        idempotency_key="relative-guarded-moves",
    )
    absolute = provider.execute_z_intent(
        "move_absolute",
        inputs={"position_steps": 70000, "wait_timeout_s": 8.0, "command_id": "operator-absolute-1"},
        expected_generation=11,
        idempotency_key="absolute-guarded-moves",
    )

    assert relative["ok"] is True
    assert relative["authority_receipt"]["command_id"] == "operator-relative-1"
    assert absolute["ok"] is True
    assert absolute["authority_receipt"]["command_id"] == "operator-absolute-1"
    assert primitives.calls[-2:] == ["move-steps:25", "move-absolute:70000:65000"]


def test_z_stop_interrupt_dispatches_before_normal_intent_releases_provider_lock(tmp_path):
    class InterruptiblePrimitives(FakeSerial206Primitives):
        def __init__(self):
            super().__init__()
            self.move_started = threading.Event()
            self.release_move = threading.Event()
            self.stop_dispatched = threading.Event()

        def z_move_steps(self, *, steps, wait_timeout_s=20.0):
            self.calls.append(f"move-steps:{steps}")
            self.move_started.set()
            assert self.release_move.wait(2.0), "test did not release the in-flight move"
            return {
                "ok": True,
                "source_method": "moveSteps",
                "steps": steps,
                "timeout_s": wait_timeout_s,
                "controller_command_acknowledged": True,
                "controller_terminal_state_verified": True,
            }

        def z_stop(self, *, timeout_s=3.0):
            self.calls.append("z-stop")
            self.stop_dispatched.set()
            return {
                "ok": True,
                "source_method": "StopMotor double delivery",
                "timeout_s": timeout_s,
                "controller_command_acknowledged": True,
                "controller_terminal_state_verified": True,
            }

    primitives = InterruptiblePrimitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-stop-interrupt"
    )["ok"] is True
    assert provider.execute_z_intent(
        "set_home",
        inputs={"operator_ack": "SET_HOME_CURRENT_POSITION"},
        expected_generation=11,
        idempotency_key="set-home-stop-interrupt",
    )["ok"] is True

    outcomes = {}
    errors = []

    def run_move():
        try:
            outcomes["move"] = provider.execute_z_intent(
                "move_steps",
                inputs={"steps": 25, "command_id": "move-command-1"},
                expected_generation=11,
                idempotency_key="move-stop-interrupt",
            )
        except BaseException as exc:  # pragma: no cover - asserted below
            errors.append(exc)

    def run_stop():
        try:
            outcomes["stop"] = provider.execute_z_stop_interrupt(
                inputs={"command_id": "stop-command-1", "timeout_s": 3.0},
                expected_generation=11,
                idempotency_key="stop-interrupt",
            )
        except BaseException as exc:  # pragma: no cover - asserted below
            errors.append(exc)

    move_thread = threading.Thread(target=run_move)
    stop_thread = threading.Thread(target=run_stop)
    move_thread.start()
    assert primitives.move_started.wait(1.0)
    stop_thread.start()

    assert primitives.stop_dispatched.wait(1.0)
    assert move_thread.is_alive()
    assert stop_thread.is_alive()
    primitives.release_move.set()
    move_thread.join(2.0)
    stop_thread.join(2.0)

    assert not move_thread.is_alive()
    assert not stop_thread.is_alive()
    assert errors == []
    assert outcomes["move"]["ok"] is False
    assert outcomes["move"]["result"]["failure"] == "z_intent_interrupted_by_safety_stop"
    assert outcomes["move"]["result"]["failure_stop"]["delegated_to_safety_interrupt"] is True
    assert outcomes["stop"]["ok"] is True
    assert outcomes["stop"]["result"]["interrupted_command_ids"] == ["move-command-1"]
    assert outcomes["stop"]["z_state"] == "failed_latched"
    assert primitives.calls.count("z-stop") == 1
    assert references.transitions[-1] == ("desynced", "z")


def test_repeated_z_stop_interrupt_key_dispatches_and_persists_each_receipt(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())

    first = provider.execute_z_stop_interrupt(
        inputs={"command_id": "stop-command-repeat-1"},
        expected_generation=11,
        idempotency_key="repeated-stop-interrupt-key",
    )
    second = provider.execute_z_stop_interrupt(
        inputs={"command_id": "stop-command-repeat-1"},
        expected_generation=11,
        idempotency_key="repeated-stop-interrupt-key",
    )

    assert first["ok"] is True
    assert second["ok"] is True
    assert primitives.calls.count("z-stop") == 2
    durable = provider.state_store.list_serial206_receipts("z")
    repeated = [
        row for row in durable
        if row.get("idempotency_key") == "repeated-stop-interrupt-key"
    ]
    assert [row["command_id"] for row in repeated] == [
        "stop-command-repeat-1",
        "stop-command-repeat-1",
    ]
    assert len({row["receipt_id"] for row in repeated}) == 2
    assert all(row["idempotency_replay_enabled"] is False for row in repeated)
    assert provider.state_store.read_serial206_receipt_by_idempotency(
        "z", "repeated-stop-interrupt-key"
    ) is None


def test_z_stop_interrupt_fails_closed_when_terminal_zero_is_unverified(tmp_path):
    class UnverifiedStopPrimitives(FakeSerial206Primitives):
        def z_stop(self, *, timeout_s=3.0):
            self.calls.append("z-stop-unverified")
            return {
                "ok": True,
                "source_method": "StopMotor double delivery",
                "timeout_s": timeout_s,
                "controller_command_acknowledged": True,
                "controller_terminal_state_verified": False,
            }

    references = ReferenceStore()
    provider = _provider(tmp_path, UnverifiedStopPrimitives(), references)

    stopped = provider.execute_z_stop_interrupt(
        inputs={"command_id": "stop-unverified-command"},
        expected_generation=11,
        idempotency_key="stop-unverified",
    )

    assert stopped["ok"] is False
    assert stopped["authority_receipt"]["status"] == "failed"
    assert stopped["z_state"] == "failed_latched"
    assert stopped["z_lifecycle"]["reference_state"] == "desynced"
    assert stopped["z_lifecycle"]["last_failure"]["reason"] == "z_safety_stop_unverified"
    assert references.transitions[-1] == ("desynced", "z")


def test_z_stop_requires_acknowledged_zero_speed_terminal_readback():
    class Tester:
        valid_wait = False

        @staticmethod
        def _tmcl_success(ack):
            return isinstance(ack, dict) and ack.get("status") == 100

        def motor_stop(self, board, *, motor):
            assert (board, motor) == (4, 1)
            return {"ok": True, "first_delivery": dict(ACK), "second_delivery": dict(ACK)}

        def motor_wait_stopped(self, board, *, motor, timeout_s, require_seen_nonzero):
            assert (board, motor, require_seen_nonzero) == (4, 1, False)
            if self.valid_wait:
                return {"stopped": True, "last_speed": 0, "last_ack": dict(ACK)}
            return {"stopped": True, "last_speed": None, "last_ack": None}

    tester = Tester()
    adapter = subject.Serial206ProductionPrimitiveAdapter(
        tester, object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )

    malformed = adapter.z_stop(timeout_s=3.0)
    assert malformed["ok"] is False
    assert malformed["controller_terminal_state_verified"] is False

    tester.valid_wait = True
    verified = adapter.z_stop(timeout_s=3.0)
    assert verified["ok"] is True
    assert verified["controller_terminal_state_verified"] is True


def test_diagnostic_home_reports_nested_command_and_terminal_evidence_truthfully():
    class Tester:
        def _motion_oem_axis_profile(self, axis):
            assert axis == "z"
            return {"board": 4, "motor": 1}

        def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides=None):
            assert axis == "z"
            return {"ok": True}

        def motor_oem_verify_motion_interlock(self):
            return {"ok": True}

        def motor_oem_home_axis_board_test(self, axis, *, timeout_s):
            assert axis == "z"
            return {
                "ok": True,
                "home": {
                    "ok": True,
                    "go_home": {
                        "controller_command_acknowledged": False,
                        "controller_terminal_state_verified": False,
                    },
                },
            }

    adapter = subject.Serial206ProductionPrimitiveAdapter(
        Tester(), object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )
    result = adapter.z_diagnostic_home_axis(timeout_s=30.0)

    assert result["ok"] is True
    assert result["controller_command_acknowledged"] is False
    assert result["controller_terminal_state_verified"] is False


def test_manual_home_preserves_compact_false_home_guard_and_controller_summary():
    class Tester:
        def _motion_oem_axis_profile(self, axis):
            return {"board": 4, "motor": 1}

        def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides=None):
            return {"ok": True}

        def motor_oem_verify_motion_interlock(self):
            return {"ok": True}

        def motor_oem_go_home(self, axis, **kwargs):
            assert axis == "z"
            return {
                "ok": False,
                "false_home_guard": "controller_async_error_130",
                "move_home": {"ok": True, "ack": dict(ACK)},
                "stop": {"ok": True, "first_delivery": dict(ACK), "second_delivery": dict(ACK)},
                "wait": {"stopped": True, "last_speed": 0, "last_ack": dict(ACK)},
                "position_before": {"position": -1808468},
                "position_after": {"position": -1969141},
                "home_before": {"value": 0},
                "home_after_stop": {"value": 1},
                "trace_tail": [{"position": index} for index in range(200)],
            }

    adapter = subject.Serial206ProductionPrimitiveAdapter(
        Tester(), object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )
    result = adapter.z_manual_home(timeout_s=30.0)

    assert result["ok"] is False
    assert result["failure"] == "controller_async_error_130"
    assert result["home_summary"]["before_position_steps"] == -1808468
    assert result["home_summary"]["after_position_steps"] == -1969141
    assert result["home_summary"]["home_before"] == 0
    assert result["home_summary"]["home_after_stop"] == 1


def test_all_zero_move_to_executes_source_compound_home_branch():
    class Tester:
        def __init__(self):
            self.calls = []

        def _motion_oem_axis_profile(self, axis):
            assert axis == "z"
            return {"board": 4, "motor": 1}

        def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides=None):
            assert axis == "z"
            return {"ok": True}

        def motor_oem_verify_motion_interlock(self):
            return {"ok": True}

        def motor_oem_move_z_home(self, **kwargs):
            self.calls.append(("z_home", kwargs))
            return {
                "ok": True,
                "home": {
                    "ok": True,
                    "move_home": {"ok": True, "ack": dict(ACK)},
                    "stop": {"ok": True, "first_delivery": dict(ACK), "second_delivery": dict(ACK)},
                    "wait": {"stopped": True, "last_speed": 0, "last_ack": dict(ACK)},
                },
            }

        def motor_oem_go_home(self, axis, **kwargs):
            self.calls.append(("axis_home", axis, kwargs))
            return {"ok": True}

    tester = Tester()
    adapter = subject.Serial206ProductionPrimitiveAdapter(
        tester, object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )

    result = adapter.oem_move_to(
        0, 0, 0,
        pseudo_home_steps=65000,
        gripper_confirmed=False,
        tip_loaded=False,
    )

    assert result["ok"] is True
    assert result["branch"] == "all_zero_home"
    assert result["z_home_reference_verified"] is True
    assert [row[0] for row in tester.calls].count("z_home") == 1
    assert sorted(row[1] for row in tester.calls if row[0] == "axis_home") == ["x", "y"]


def test_legacy_referenced_z_state_is_durably_desynced_during_migration(tmp_path):
    store = OEMRuntimeStore(tmp_path / "runtime")
    legacy = subject.Serial206OemInitializationProvider._new_state()
    legacy_z = legacy["z_lifecycle"]
    legacy_z["schema_version"] = "bioxp.serial206_z_lifecycle.v1"
    legacy_z.pop("board_lifecycle_generation")
    legacy_z.update({
        "state": "referenced_ready",
        "generation": 11,
        "prepared_receipt": {"ok": True},
        "reference_state": "referenced",
    })
    store.write_oem_serial206_initialization_state(legacy)
    references = ReferenceStore()
    primitives = FakeSerial206Primitives()
    provider = subject.Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        reference_store=references,
        generation_provider=lambda: 11,
        preparation_provider=PreparationProvider(primitives.calls),
    )

    projection = provider.z_projection()
    persisted = store.read_oem_serial206_initialization_state()["z_lifecycle"]

    assert projection["state"] == "unprepared"
    assert persisted["schema_version"] == "bioxp.serial206_z_lifecycle.v2"
    assert persisted["state"] == "unprepared"
    assert persisted["board_lifecycle_generation"] is None
    assert persisted["reference_state"] == "desynced"
    assert persisted["last_failure"]["reason"] == "legacy_z_state_missing_board_lifecycle_generation"
    assert ("desynced", "z") in references.transitions


def test_zero_step_relative_move_is_rejected_before_controller_dispatch(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="zero-prepare"
    )["ok"] is True
    assert provider.execute_z_intent(
        "set_home",
        inputs={"operator_ack": "SET_HOME_CURRENT_POSITION"},
        expected_generation=11,
        idempotency_key="zero-set-home",
    )["ok"] is True
    calls_before = list(primitives.calls)

    result = provider.execute_z_intent(
        "move_steps",
        inputs={"steps": 0},
        expected_generation=11,
        idempotency_key="zero-move",
    )

    assert result["ok"] is False
    assert result["blockers"] == ["z_relative_steps_must_be_nonzero"]
    assert primitives.calls == calls_before


def test_z_absolute_move_refuses_motion_without_valid_pre_position():
    class Tester:
        def _motion_oem_axis_profile(self, axis):
            return {"board": 4, "motor": 1, "run_current": 31}

        def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides=None):
            return {"ok": True}

        def motor_oem_verify_motion_interlock(self):
            return {"ok": True}

        def motor_get_position(self, board, *, motor):
            return {"ok": False, "ack": None, "position": None}

        def motor_set_axis_param(self, *args, **kwargs):
            raise AssertionError("current write must not occur without a valid pre-position")

    adapter = subject.Serial206ProductionPrimitiveAdapter(
        Tester(), object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )

    result = adapter.z_move_absolute(
        requested_position_steps=70000,
        pseudo_home_steps=65000,
        wait_timeout_s=2.0,
    )

    assert result["ok"] is False
    assert result["failure"] == "z_current_position_unavailable"
    assert result["physical_motion_commanded"] is False


def test_z_absolute_same_effective_target_is_verified_noop_without_dispatch():
    class Tester:
        def __init__(self):
            self.calls = []

        def _motion_oem_axis_profile(self, axis):
            return {"board": 4, "motor": 1, "run_current": 31}

        def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides=None):
            return {"ok": True}

        def motor_oem_verify_motion_interlock(self):
            return {"ok": True}

        def motor_get_position(self, board, *, motor):
            self.calls.append("get-position")
            return {"ok": True, "ack": dict(ACK), "position": 65000}

        def motor_wait_stopped(self, board, *, motor, timeout_s, require_seen_nonzero, target_position=None):
            self.calls.append("wait-stopped")
            assert require_seen_nonzero is False
            assert target_position == 65000
            return {
                "stopped": True,
                "last_speed": 0,
                "last_ack": dict(ACK),
                "target_position": target_position,
                "target_reached": True,
            }

        def motor_set_axis_param(self, *args, **kwargs):
            raise AssertionError("same-target no-op must not write motor current")

        def begin_bus_event_window(self):
            raise AssertionError("same-target no-op must not open a movement event window")

        def motor_move_absolute(self, *args, **kwargs):
            raise AssertionError("same-target no-op must not dispatch movement")

    tester = Tester()
    adapter = subject.Serial206ProductionPrimitiveAdapter(
        tester, object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )

    result = adapter.z_move_absolute(
        requested_position_steps=0,
        pseudo_home_steps=65000,
        wait_timeout_s=2.0,
    )

    assert result["ok"] is True
    assert result["source_noop"] is True
    assert result["noop_reason"] == "already_at_effective_target"
    assert result["requested_position_steps"] == 0
    assert result["effective_position_steps"] == 65000
    assert result["before_position_steps"] == 65000
    assert result["after_position_steps"] == 65000
    assert result["physical_motion_commanded"] is False
    assert result["controller_command_acknowledged"] is False
    assert result["controller_terminal_state_verified"] is True
    assert tester.calls == ["get-position", "wait-stopped", "get-position"]


def test_provider_accepts_verified_z_absolute_noop_without_fake_command_ack(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="noop-prepare"
    )["ok"] is True
    assert provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="noop-home"
    )["ok"] is True

    primitives.z_move_absolute = lambda **kwargs: {
        "ok": True,
        "failure": None,
        "source_noop": True,
        "noop_reason": "already_at_effective_target",
        "physical_motion_commanded": False,
        "controller_command_acknowledged": False,
        "controller_terminal_state_verified": True,
        "before_position_steps": 65000,
        "target_position_steps": 65000,
        "after_position_steps": 65000,
        "requested_position_steps": 0,
        "pseudo_home_steps": 65000,
        "effective_position_steps": 65000,
    }

    result = provider.execute_z_intent(
        "move_absolute",
        inputs={"position_steps": 0},
        expected_generation=11,
        idempotency_key="verified-noop-absolute",
    )

    assert result["ok"] is True
    assert result["authority_receipt"]["status"] == "completed"
    assert result["authority_receipt"]["controller_command_acknowledged"] is False
    assert result["authority_receipt"]["controller_terminal_state_verified"] is True
    assert result["result"]["source_noop"] is True
    assert result["z_state"] == "referenced_ready"


def test_provider_rejects_z_noop_with_wrong_effective_target(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="wrong-target-prepare"
    )["ok"] is True
    assert provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="wrong-target-home"
    )["ok"] is True

    primitives.z_move_absolute = lambda **kwargs: {
        "ok": True,
        "failure": None,
        "source_noop": True,
        "noop_reason": "already_at_effective_target",
        "physical_motion_commanded": False,
        "controller_command_acknowledged": False,
        "controller_terminal_state_verified": True,
        "before_position_steps": 65001,
        "target_position_steps": 65001,
        "after_position_steps": 65001,
        "requested_position_steps": 0,
        "pseudo_home_steps": 65000,
        "effective_position_steps": 65001,
    }

    result = provider.execute_z_intent(
        "move_absolute",
        inputs={"position_steps": 0},
        expected_generation=11,
        idempotency_key="wrong-effective-noop",
    )

    assert result["ok"] is False
    assert result["authority_receipt"]["status"] == "failed"
    assert result["result"]["failure"] == "z_controller_command_or_terminal_evidence_unverified"


def test_provider_rejects_z_noop_marker_on_relative_move(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="wrong-intent-prepare"
    )["ok"] is True
    assert provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="wrong-intent-home"
    )["ok"] is True

    primitives.z_move_steps = lambda **kwargs: {
        "ok": True,
        "failure": None,
        "source_noop": True,
        "noop_reason": "already_at_effective_target",
        "physical_motion_commanded": False,
        "controller_command_acknowledged": False,
        "controller_terminal_state_verified": True,
        "before_position_steps": 65000,
        "target_position_steps": 65000,
        "after_position_steps": 65000,
        "effective_position_steps": 65000,
    }

    result = provider.execute_z_intent(
        "move_steps",
        inputs={"steps": 1000},
        expected_generation=11,
        idempotency_key="wrong-intent-noop",
    )

    assert result["ok"] is False
    assert result["authority_receipt"]["status"] == "failed"
    assert result["result"]["failure"] == "z_controller_command_or_terminal_evidence_unverified"


def test_provider_accepts_verified_z_clear_noop_at_selected_pseudo_home(tmp_path):
    primitives = FakeSerial206Primitives()
    provider = _provider(tmp_path, primitives, ReferenceStore())
    assert provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="clear-noop-prepare"
    )["ok"] is True
    assert provider.execute_z_intent(
        "manual_home", expected_generation=11, idempotency_key="clear-noop-home"
    )["ok"] is True

    primitives.z_move_absolute = lambda **kwargs: {
        "ok": True,
        "failure": None,
        "source_noop": True,
        "noop_reason": "already_at_effective_target",
        "physical_motion_commanded": False,
        "controller_command_acknowledged": False,
        "controller_terminal_state_verified": True,
        "before_position_steps": 65000,
        "target_position_steps": 65000,
        "after_position_steps": 65000,
        "requested_position_steps": 65000,
        "pseudo_home_steps": 65000,
        "effective_position_steps": 65000,
    }

    result = provider.execute_z_intent(
        "clear",
        expected_generation=11,
        idempotency_key="verified-clear-noop",
    )

    assert result["ok"] is True
    assert result["authority_receipt"]["status"] == "completed"
    assert result["authority_receipt"]["controller_command_acknowledged"] is False
    assert result["result"]["selected_pseudo_home_steps"] == 65000


def test_z_relative_move_requires_fresh_target_event_and_acknowledged_zero_speed():
    class Tester:
        fresh_event = False
        event_motor: int | None = 1

        @staticmethod
        def _tmcl_success(ack):
            return isinstance(ack, dict) and ack.get("status") == 100

        def __init__(self):
            self.positions = [0, 25, 0, 25, 0, 25]
            self.window_calls = 0

        def _motion_oem_axis_profile(self, axis):
            return {"board": 4, "motor": 1, "axis_min_steps": 0, "axis_max_steps": 160000}

        def motor_oem_require_no_motion_profile(self, axis, *, expected_overrides=None):
            return {"ok": True}

        def motor_oem_verify_motion_interlock(self):
            return {"ok": True}

        def begin_bus_event_window(self):
            self.window_calls += 1
            return {"after_sequence": self.window_calls * 10}

        def clear_bus_event_buffer(self):
            return {"after_sequence": 10}

        def motor_get_position(self, board, *, motor):
            return {"ok": True, "ack": dict(ACK), "position": self.positions.pop(0)}

        def motor_move_relative(self, board, steps, *, motor):
            return {"ok": True, "ack": dict(ACK), "steps": steps}

        def motor_stop(self, board, *, motor):
            return {"ok": True, "first_delivery": dict(ACK), "second_delivery": dict(ACK)}

        def motor_wait_stopped(self, board, *, motor, timeout_s, require_seen_nonzero, target_position=None):
            return {
                "stopped": True,
                "last_speed": 0,
                "seen_nonzero": True,
                "target_position": target_position,
                "target_reached": True,
                "last_position": target_position,
                "last_ack": dict(ACK),
            }

        def collect_bus_events(self, *, duration_s, timeout_ms, max_events):
            sequence = self.window_calls * 10 + (1 if self.fresh_event else 0)
            return [{"board": 4, "motor": self.event_motor, "status": 128, "event_sequence": sequence}]

    tester = Tester()
    adapter = subject.Serial206ProductionPrimitiveAdapter(
        tester, object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    )

    stale = adapter.z_move_steps(steps=25, wait_timeout_s=1.0)
    assert stale["ok"] is False
    assert stale["failure"] == "z_target_event_128_missing_or_stale"

    tester.fresh_event = True
    fresh = adapter.z_move_steps(steps=25, wait_timeout_s=1.0)
    assert fresh["ok"] is True
    assert tester.window_calls == 2

    tester.event_motor = None
    unqualified = adapter.z_move_steps(steps=25, wait_timeout_s=1.0)
    assert unqualified["ok"] is False
    assert unqualified["failure"] == "z_target_event_128_missing_or_stale"
    assert tester.window_calls == 3


def test_board_deactivation_clears_all_preparation_derived_z_authority(tmp_path):
    primitives = FakeSerial206Primitives()
    references = ReferenceStore()
    provider = _provider(tmp_path, primitives, references)
    prepared = provider.execute_z_intent(
        "prepare", expected_generation=11, idempotency_key="prepare-before-board-deactivate"
    )
    assert prepared["ok"] is True

    invalidated = provider.notify_board_activation(4, {"status": 100}, active=False)
    persisted = provider.state_store.read_oem_serial206_initialization_state()["z_lifecycle"]

    assert invalidated["z_affected"] is True
    assert invalidated["invalidation"]["transition"] == "deactivated"
    assert persisted["state"] == "unprepared"
    assert persisted["generation"] is None
    assert persisted["board_lifecycle_generation"] is None
    assert persisted["prepared_receipt"] is None
    assert persisted["active_receipt"] is None
    assert persisted["awaiting_observation_receipt_id"] is None
    assert persisted["reference_state"] == "desynced"
