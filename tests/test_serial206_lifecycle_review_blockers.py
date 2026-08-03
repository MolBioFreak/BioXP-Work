from __future__ import annotations

import asyncio
import copy
import json
import math
from pathlib import Path

import pytest
from fastapi import HTTPException

import bioxp.api as api
from bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_runtime_types import OEMRuntimeCommand
from bioxp import oem_serial206_initialization as subject


ACK = {"status": 100, "value": 0}


def _write(value: int) -> dict:
    return {
        "ok": True,
        "ack": dict(ACK),
        "set_value": value,
        "readback": {"ack": dict(ACK), "value": value},
    }


def _position(value: int) -> dict:
    return {"ok": True, "ack": dict(ACK), "position": value}


def _prepare() -> dict:
    return {
        "board": 5,
        "motor": 0,
        "ops": [
            {"op": "sap6-run_current", "ack": dict(ACK), "rb": {"ack": dict(ACK), "value": 31}, "set": 31},
            {"op": "sap4-max_speed", "ack": dict(ACK), "rb": {"ack": dict(ACK), "value": 250}, "set": 250},
        ],
    }


def _axis_home(axis: str) -> dict:
    return {
        "axis": axis,
        "startup": True,
        "prepare": _prepare(),
        "home": {
            "axis": axis,
            "ok": True,
            "home_active_value": 1,
            "position_before": _position(1200),
            "position_after": _position(4),
            "position_after_sethome": _position(0),
            "move_home": {"ok": True, "ack": dict(ACK)},
            "wait": {"stopped": True, "last_speed": 0, "last_ack": dict(ACK), "seen_nonzero": True},
            "stop": {"ok": True, "ack": dict(ACK)},
            "home_after": {"ok": True, "ack": dict(ACK), "value": 1},
            "set_home": _write(0),
            "seen_motion": True,
            "switch_transition": True,
            "home_predicate_confirmed": True,
        },
        "restore_current": None,
    }


def _gripper_home() -> dict:
    row = _axis_home("g")
    row["restore_current"] = None
    row["restore_idle_current_requested"] = False
    row["source_current_retained"] = True
    row["retained_current_readback"] = {"ok": True, "ack": dict(ACK), "value": 31}
    return row


def _move(before: int, after: int) -> dict:
    return {
        "move": {"ok": True, "ack": dict(ACK)},
        "wait": {"stopped": True, "last_speed": 0, "last_ack": dict(ACK), "seen_nonzero": True},
        "position": {"before": _position(before), "after": _position(after)},
    }


def _door_home() -> dict:
    return {
        "axis": "door",
        "ok": True,
        "status_before": {"position": _position(18000), "oem_predicates": {"tcDoorClosed": False, "closed_source": "queryHome(ThermalDoor)"}},
        "move_left": {"ok": True, "ack": dict(ACK)},
        "wait": {"stopped": True, "last_speed": 0, "last_ack": dict(ACK), "seen_nonzero": True},
        "stop": {"ok": True, "ack": dict(ACK)},
        "status_after": {"position": _position(0), "oem_predicates": {"tcDoorClosed": True, "closed_source": "queryHome(ThermalDoor)"}},
        "home_after": {"ok": True, "ack": dict(ACK), "value": 1},
        "set_home": _write(0),
        "closed_confirmed": True,
    }


def _raw_for(stage: str) -> dict:
    if stage in {"z-home", "x-home", "y-home"}:
        return _axis_home(stage[0])
    if stage == "gripper-home":
        return _gripper_home()
    if stage == "door-home":
        return _door_home()
    if stage == "gripper-current-31":
        return _write(31)
    if stage == "gripper-clear-10000":
        return _move(10, 10010)
    if stage == "x-set-home" or stage == "y-set-home":
        return _write(0)
    if stage == "x-speed-1700":
        return _write(1700)
    if stage == "x-park-6000":
        return _move(0, 6000)
    if stage in {"x-home-settle", "x-speed-settle"}:
        return {"settled": True, "settle_ms": 20 if stage == "x-home-settle" else 40}
    if stage == "door-closed-predicate":
        return {
            "ok": True,
            "source_condition_active": False,
            "branch_binding": {"serial_number": 9, "camera_calibrated": True},
            "oem_predicates": {"tcDoorClosed": True, "closed_source": "queryHome(ThermalDoor)"},
            "position": _position(0),
            "speed": {"speed": 0, "ack": dict(ACK)},
        }
    if stage == "ui-zero-calibrated":
        return {"ok": True, "calibrated": True, "writes": [{"axis": axis, "value": "0", "applied": True} for axis in ("x", "y", "z", "z")], "readback": {"x": "0", "y": "0", "z": "0"}}
    if stage in {"chiller-oc-cool-rate", "chiller-rc-cool-rate"}:
        return {"ok": True, "write": {"ack": dict(ACK), "value": -25}, "readback": {"ack": dict(ACK), "value": -25}}
    if stage == "system-status-initialized":
        return {"ok": True, "controller_state": {"system_status": 1, "initialization_complete": True, "ready": False}}
    if stage == "gripper-idle-current-10":
        return _write(10)
    raise AssertionError(stage)


def _approval(stage: str, generation: int = 11) -> subject.Serial206StageApproval:
    spec = next(row for row in subject.SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS if row.key == stage)
    return subject.Serial206StageApproval(
        approval_id=f"approval-{stage}",
        expected_generation=generation,
        expected_component=spec.component,
        expected_direction=spec.direction,
        expected_bound=spec.bound,
        operator_note="exact supervised stage",
        idempotency_key=f"idempotency-{stage}",
    )


@pytest.mark.parametrize("spec", subject.SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS, ids=lambda row: row.key)
def test_every_stage_accepts_only_its_production_shaped_nested_envelope(spec):
    receipt = subject.Serial206OemInitializationProvider._receipt(spec, _approval(spec.key), _raw_for(spec.key))
    assert receipt["ok"] is True
    assert receipt["physical_effect_verified"] is False
    assert (receipt["durable_robot_state"] or {}).get("ready") is not True


@pytest.mark.parametrize("spec", subject.SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS, ids=lambda row: row.key)
def test_flattened_ok_true_never_satisfies_a_serial206_stage(spec):
    receipt = subject.Serial206OemInitializationProvider._receipt(spec, _approval(spec.key), {"ok": True})
    assert receipt["ok"] is False


def test_production_capability_requires_frozen_selected_branches_and_calibrated_ui_sink(monkeypatch):
    class Tester:
        def motor_oem_home_axis(self, *args, **kwargs): pass
        def motor_set_axis_param(self, *args, **kwargs): pass
        def motor_move_relative(self, *args, **kwargs): pass
        def motor_wait_stopped(self, *args, **kwargs): pass
        def motor_oem_wait_target_reached(self, *args, **kwargs): pass
        def motor_get_position(self, *args, **kwargs): pass
        def motor_set_home(self, *args, **kwargs): pass
        def motor_move_absolute(self, *args, **kwargs): pass
        def motor_oem_move_absolute(self, *args, **kwargs): pass
        def oem_no24v_state(self, *args, **kwargs): pass
        def motor_oem_door_search_home(self, *args, **kwargs): pass
        def motor_thermal_door_status(self, *args, **kwargs): pass
        def motor_prepare_axis(self, *args, **kwargs): pass
        def motor_oem_open_thermal_door(self, *args, **kwargs): pass
        def _motion_oem_axis_profile(self, *args, **kwargs): pass
        def chiller_gp_write(self, *args, **kwargs): pass
        def _machine_config_bundle(self):
            return {"ok": True, "config": {"config": {"GripperVersion": "1"}, "calibration": {"Calibrated": "1"}}}

    class Parity:
        blockers = []
        values = {"SerialNumber": 206, "CameraCalibrated": 1}
        calibration_source = "test-frozen-serial206"

    monkeypatch.setattr(subject, "load_oem_parity_config", lambda _path: Parity())
    adapter = subject.Serial206ProductionPrimitiveAdapter(Tester(), object(), authority_provider=lambda: object(), generation_provider=lambda: 11)
    status = adapter.capability_status()
    assert status["initialize_motors_exact_primitives_bound"] is True
    assert status["initialize_motors_binding_blockers"] == []
    assert status["selected_serial_number"] == 206
    assert status["selected_gripper_version"] == 1
    assert status["selected_calibrated_branch"] is True


@pytest.mark.parametrize(
    ("gripper_version", "calibrated", "expected_blocker"),
    [(0, 1, "serial206_gripper_version_1_not_selected"), (1, 0, "serial206_calibrated_1_branch_not_selected")],
)
def test_production_capability_rejects_nonfrozen_serial206_branch_selection(gripper_version, calibrated, expected_blocker):
    class Tester:
        def motor_oem_home_axis(self, *args, **kwargs): pass
        def motor_set_axis_param(self, *args, **kwargs): pass
        def motor_move_relative(self, *args, **kwargs): pass
        def motor_wait_stopped(self, *args, **kwargs): pass
        def motor_get_position(self, *args, **kwargs): pass
        def motor_set_home(self, *args, **kwargs): pass
        def motor_move_absolute(self, *args, **kwargs): pass
        def motor_oem_door_search_home(self, *args, **kwargs): pass
        def motor_thermal_door_status(self, *args, **kwargs): pass
        def chiller_gp_write(self, *args, **kwargs): pass
        def oem_set_calibrated_ui_positions_zero(self): pass
        def _machine_config_bundle(self):
            return {"ok": True, "config": {"config": {"GripperVersion": gripper_version}, "calibration": {"Calibrated": calibrated}}}

    status = subject.Serial206ProductionPrimitiveAdapter(
        Tester(), object(), authority_provider=lambda: object(), generation_provider=lambda: 11
    ).capability_status()
    assert status["initialize_motors_exact_primitives_bound"] is False
    assert expected_blocker in status["initialize_motors_binding_blockers"]


class _NoIoPrimitives:
    def __init__(self, output=None):
        self.calls: list[str] = []
        self.output = output if output is not None else _axis_home("z")

    def motor_oem_home_axis(self, *args, **kwargs):
        self.calls.append("z-home")
        return self.output


class _Preparation:
    def __init__(self, calls: list[str]):
        self.calls = calls

    def prepare_for_initialize_motors(self, *, expected_generation: int):
        self.calls.append("prepare")
        return {"ok": True, "observed_generation": expected_generation, "board_preparation_verified": True, "initialize_without_motion_verified": True, "physical_motion": False}


def _commissioning(generation: int = 11):
    return {
        key: subject.Serial206CommissioningEvidence(
            component=key,
            generation=generation,
            fresh=True,
            direction_verified=True,
            limits_verified=True,
            switch_verified=True,
            stop_verified=True,
            reference_verified=True,
            gap9_polarity=1 if key == "z" else None,
            gap10_polarity=0 if key == "z" else None,
        )
        for key in ("z", "g", "x", "y", "door")
    }


def _provider(root: Path, primitives: _NoIoPrimitives) -> subject.Serial206OemInitializationProvider:
    return subject.Serial206OemInitializationProvider(
        primitives,
        state_store=OEMRuntimeStore(root),
        generation_provider=lambda: 11,
        preparation_provider=_Preparation(primitives.calls),
        sleep=lambda _: None,
    )


def test_cycle_deep_wide_large_and_nonfinite_evidence_projection_is_deterministic_and_bounded():
    cycle: dict[str, object] = {"text": "x" * 1_000_000, "nonfinite": math.inf, "wide": list(range(1000))}
    cycle["cycle"] = cycle
    cursor = cycle
    for index in range(100):
        child = {"index": index}
        cursor["deep"] = child
        cursor = child
    first = subject._json_safe(cycle)
    second = subject._json_safe(cycle)
    encoded = json.dumps(first, sort_keys=True, allow_nan=False).encode()
    assert first == second
    assert len(encoded) <= subject._EVIDENCE_MAX_BYTES
    assert b"cycle" in encoded or b"limit" in encoded


def test_primitive_output_projection_failure_persists_nonreplayable_failed_state(tmp_path):
    cycle: dict[str, object] = {"ok": True}
    cycle["cycle"] = cycle
    first_primitives = _NoIoPrimitives(cycle)
    first = _provider(tmp_path / "runtime", first_primitives)
    result = first.initialize_motors(mode="live", approval=_approval("z-home"), commissioning=_commissioning())
    assert result["ok"] is False
    assert len(json.dumps(result["stage_receipts"][0], allow_nan=False).encode()) <= subject._EVIDENCE_MAX_BYTES * 2

    restarted_primitives = _NoIoPrimitives()
    restarted = _provider(tmp_path / "runtime", restarted_primitives)
    again = restarted.initialize_motors(mode="live", approval=_approval("z-home", generation=11), commissioning=_commissioning())
    assert again["ok"] is False
    assert restarted_primitives.calls == []


def _valid_admitted_state() -> dict:
    state = subject.Serial206OemInitializationProvider._new_state()
    state["preparation"] = {"generation": 11, "state": "completed", "receipt": {"ok": True}}
    row = state["movement_ledger"]["stages"]["z-home"]
    row.update({"state": "admitted", "command_id": "approval-z-home", "idempotency_key": "idempotency-z-home", "expected_generation": 11})
    state["movement_ledger"]["terminal_state"] = "running"
    state["used_approvals"]["approval-z-home"] = {"stage": "z-home", "generation": 11, "idempotency_key": "idempotency-z-home"}
    return state


def _persist_state(root: Path, state: dict) -> None:
    OEMRuntimeStore(root).write_oem_serial206_initialization_state(state)


def test_restart_from_single_admitted_stage_fails_ambiguous_without_replay(tmp_path):
    root = tmp_path / "runtime"
    _persist_state(root, _valid_admitted_state())
    primitives = _NoIoPrimitives()
    result = _provider(root, primitives).initialize_motors(mode="live", approval=_approval("z-home"), commissioning=_commissioning())
    assert result["state"] == "failed_closed"
    assert "stage_execution_outcome_ambiguous_after_restart:z-home" in result["blockers"]
    assert primitives.calls == []
    persisted = OEMRuntimeStore(root).read_oem_serial206_initialization_state()
    assert persisted["movement_ledger"]["stages"]["z-home"]["state"] == "failed"


@pytest.mark.parametrize("mutate", [
    lambda state: state["movement_ledger"]["stages"]["gripper-current-31"].update({"state": "admitted", "command_id": "other", "idempotency_key": "other-idem", "expected_generation": 11}),
    lambda state: state["movement_ledger"]["stages"]["gripper-current-31"].update({"state": "completed", "command_id": "later"}),
    lambda state: state["used_approvals"]["approval-z-home"].update({"stage": "x-home"}),
    lambda state: state["preparation"].update({"generation": 12}),
    lambda state: state["movement_ledger"].update({"terminal_state": "initializeMotors_complete", "expected_next_stage": "z-home"}),
], ids=["two-admitted", "impossible-order", "used-approval-mismatch", "generation-preparation-mismatch", "terminal-expected-next-inconsistent"])
def test_semantically_corrupt_documents_fail_before_primitive_access(tmp_path, mutate):
    state = _valid_admitted_state()
    mutate(state)
    root = tmp_path / "runtime"
    _persist_state(root, state)
    primitives = _NoIoPrimitives()
    result = _provider(root, primitives).initialize_motors(mode="live", approval=_approval("z-home"), commissioning=_commissioning())
    assert result["ok"] is False
    assert "durable_serial206_state_corrupt" in result["blockers"]
    assert primitives.calls == []


def test_terminal_initialization_completion_never_publishes_machine_ready():
    state = subject.Serial206OemInitializationProvider._new_state()
    ledger = state["movement_ledger"]
    for row in ledger["stages"].values():
        row["state"] = "completed" if row["requires_operator_observation"] is False else "operator_observed"
    ledger["expected_next_stage"] = None
    ledger["terminal_state"] = "initializeMotors_complete"
    result = subject.Serial206OemInitializationProvider(object())._result_from_state(state, ok=True, blockers=[])
    assert result["initialization_complete"] is True
    assert result["ready"] is False


def test_direct_startup_step_route_is_absent_from_openapi_and_operator_catalog():
    from bioxp.operator_controls import _build_catalog

    assert "/motion/oem/startup/step" not in api.app.openapi()["paths"]
    actions, _ = _build_catalog(api.app)
    assert not any(row["informational_path"] == "/motion/oem/startup/step" for row in actions)
