from __future__ import annotations

import importlib.util
from dataclasses import asdict


def test_serial206_literal_initialize_motors_provider_module_exists():
    assert importlib.util.find_spec("src.bioxp.oem_serial206_initialization") is not None


def test_provider_exposes_typed_commissioning_and_literal_provider_contracts():
    from src.bioxp import oem_serial206_initialization as subject

    assert subject.Serial206StageApproval
    assert subject.Serial206CommissioningEvidence
    assert subject.Serial206OemInitializationProvider


EXPECTED_STAGES = [
    "z-home",
    "gripper-current-31",
    "gripper-clear-10000",
    "gripper-home",
    "x-home",
    "x-home-settle",
    "x-set-home",
    "x-speed-1700",
    "x-speed-settle",
    "x-park-6000",
    "y-home",
    "door-home",
    "door-closed-predicate",
    "y-set-home",
    "ui-zero-calibrated",
    "chiller-oc-cool-rate",
    "chiller-rc-cool-rate",
    "system-status-initialized",
    "gripper-idle-current-10",
]


class DurableStore:
    def __init__(self):
        self.ledger = None
        self.motion_ledger = None
        self.approvals = {"used": {}}

    def read_oem_movement_ledger(self):
        return self.ledger

    def write_oem_movement_ledger(self, payload):
        self.ledger = payload
        return payload

    def read_oem_stage_approvals(self):
        return self.approvals

    def write_oem_stage_approvals(self, payload):
        self.approvals = payload
        return payload

    def read_oem_initialize_motion_ledger(self):
        return self.motion_ledger

    def write_oem_initialize_motion_ledger(self, payload):
        self.motion_ledger = payload
        return payload


class ReferenceStore:
    def __init__(self, initial=None):
        self.rows = dict(initial or {})
        self.transitions = []

    def mark_referenced(self, command):
        axis = command.axis.value if hasattr(command.axis, "value") else str(command.axis)
        row = {"axis": axis, "state": "referenced", "origin_position_steps": command.position_steps, "persisted": True}
        self.rows[axis] = row
        self.transitions.append(("referenced", axis))
        return row

    def mark_desynced(self, command):
        axis = command.axis.value if hasattr(command.axis, "value") else str(command.axis)
        previous = self.rows.get(axis, {})
        row = {"axis": axis, "state": "desynced", "origin_position_steps": previous.get("origin_position_steps"), "persisted": True}
        self.rows[axis] = row
        self.transitions.append(("desynced", axis))
        return row


class FakeSerial206Primitives:
    def __init__(self, *, fail_stage=None, no_delta_stage=None, tips=(False, False, False, False), tip_queries=None):
        self.calls = []
        self.fail_stage = fail_stage
        self.no_delta_stage = no_delta_stage
        self.tips = list(tips)
        self.tip_queries = [list(row) for row in (tip_queries or [])]

    def _result(self, stage, *, moving=False, home=False):
        self.calls.append(stage)
        if stage == self.fail_stage:
            return {"ok": False, "command_ack": {"status": 2}, "wait": {"stopped": False}}
        before = 100 if moving or home else 0
        after = before if stage == self.no_delta_stage else (0 if home else before + (10 if moving else 0))
        return {
            "ok": True,
            "command_ack": {"status": 100},
            "wait": {"stopped": True, "terminal_speed": 0, "seen_nonzero": bool(moving or home)},
            "switch_predicate": {"expected": True, "observed": True} if home else None,
            "position_before": before,
            "position_after": after,
            "home_position": 0 if home else None,
            "controller_reference_agrees": True if home else None,
            "physical_effect_verified": bool(moving or home),
            "operator_assessment": "pass",
        }

    def motor_oem_home_axis(self, axis, *, startup, speed=None, timeout_s):
        speed_by_axis = {"z": 1791, "g": 200, "x": 250, "y": 250}
        assert startup is True
        assert speed in {None, speed_by_axis[axis]}
        return self._result(f"{axis}-home" if axis != "g" else "gripper-home", moving=True, home=True)

    def motor_set_axis_param(self, board, param, value, motor=0):
        stage = {
            (4, 2, 6, 31): "gripper-current-31",
            (5, 0, 4, 1700): "x-speed-1700",
            (4, 2, 6, 10): "gripper-idle-current-10",
        }[(board, motor, param, value)]
        return self._result(stage)

    def motor_move_relative(self, board, steps, motor=0):
        assert (board, motor, steps) == (4, 2, 10000)
        return self._result("gripper-clear-10000", moving=True)

    def motor_move_absolute(self, board, value, motor=0):
        assert (board, motor, value) == (5, 0, 6000)
        return self._result("x-park-6000", moving=True)

    def motor_wait_stopped(self, board, motor=0, **kwargs):
        return {"stopped": True, "terminal_speed": 0, "seen_nonzero": True}

    def motor_set_home(self, board, motor=0):
        stage = "x-set-home" if board == 5 else "y-set-home"
        return self._result(stage, home=True)

    def motor_oem_door_search_home(self, *, startup, timeout_s):
        assert startup is True
        return self._result("door-home", moving=True, home=True)

    def motor_thermal_door_status(self):
        return {**self._result("door-closed-predicate"), "oem_predicates": {"tcDoorClosed": True, "closed_source": "queryHome(ThermalDoor)"}}

    def _machine_config_bundle(self):
        return {"ok": True, "config": {"calibration": {"Calibrated": 0}}}

    def _oem_no_motion_tmcl(self, *, name, board, command, cmd_type, motor, value):
        stage = "chiller-oc-cool-rate" if cmd_type == 1 else "chiller-rc-cool-rate"
        assert (board, command, motor, value) == (7, 9, 0, -25)
        return self._result(stage)

    def query_tip_status(self):
        self.calls.append("query-tip-status")
        channels = self.tip_queries.pop(0) if self.tip_queries else list(self.tips)
        return {"ok": True, "channels": channels, "command_ack": {"status": 100}}

    def initiate_pipette_group(self):
        return self._result("pipette-initiate-group")

    def checked_pipette_status(self):
        return self._result("pipette-checked-status")

    def set_stop_scripts(self, value):
        assert value is False
        return self._result("initializeMotion.stop_scripts")

    def set_force_abort(self, value):
        assert value is False
        return self._result("initializeMotion.clear_forceabort")

    def set_thermal_door_state(self, opened):
        stage = "initializeMotion.openThermalDoor.tip_exists" if opened else "initializeMotion.thermal_door_closed"
        return self._result(stage, moving=True)

    def confirm_thermal_door(self, opened):
        stage = "initializeMotion.thermal_door_open.tip_exists" if opened else "initializeMotion.thermal_door_closed"
        return {**self._result(stage), "switch_predicate": {"expected": opened, "observed": opened}}

    def set_tip_loaded(self, loaded, *, source_stage=None):
        if loaded:
            stage = "initializeMotion.tip_loaded.tip_exists"
        elif source_stage == "no_tip":
            stage = "initializeMotion.tip_loaded_false.no_tip"
        else:
            stage = "initializeMotion.tip_loaded_false.after_eject"
        return self._result(stage)

    def set_tip_dirty(self, dirty):
        assert dirty is False
        return self._result("initializeMotion.tip_dirty_false")

    def scriptmove_to(self, **kwargs):
        assert kwargs == {"from_location": 28, "from_well": 0, "to_location": 6, "column": 0, "row": 0}
        return self._result("initializeMotion.scriptmoveTo.tip_exists", moving=True)

    def update_location(self, location):
        assert location == 6
        return self._result("initializeMotion.updateLocation.tip_exists")

    def eject_all_tips(self):
        return self._result("initializeMotion.ejectAllTips.tip_exists", moving=True)

    def move_z_absolute(self, position_steps):
        assert position_steps == 80000
        return self._result("initializeMotion.moveZ.tip_exists", moving=True)

    def move_x_absolute(self, position_steps):
        assert position_steps == 79000
        return self._result("initializeMotion.moveX.tip_exists", moving=True)


class PreparationProvider:
    def __init__(self, calls, generation=11):
        self.calls = calls
        self.generation = generation

    def prepare_for_initialize_motors(self, *, expected_generation):
        self.calls.append("prepare")
        return {
            "ok": expected_generation == self.generation,
            "expected_generation": expected_generation,
            "observed_generation": self.generation,
            "board_preparation_verified": True,
            "initialize_without_motion_verified": True,
            "physical_motion": False,
        }


def _commissioning(subject, generation=11, *, z_gap=True):
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
            gap9_polarity=1 if component == "z" and z_gap else None,
            gap10_polarity=0 if component == "z" and z_gap else None,
        )
    return rows


def _approvals(subject, generation=11):
    approvals = {}
    for spec in subject.SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS:
        approvals[spec.key] = subject.Serial206StageApproval(
            approval_id=f"T-{spec.key}",
            expected_generation=generation,
            expected_component=spec.component,
            expected_direction=spec.direction,
            expected_bound=spec.bound,
            operator_note=f"Operator reviewed {spec.key}",
            idempotency_key=f"idem-{spec.key}",
        )
    return approvals


def _provider(subject, primitives, store, references, generation=11):
    prep = PreparationProvider(primitives.calls, generation=generation)
    return subject.Serial206OemInitializationProvider(
        primitives,
        ledger_store=store,
        approval_store=store,
        reference_store=references,
        generation_provider=lambda: generation,
        preparation_provider=prep,
        sleep=lambda seconds: primitives.calls.append(f"sleep:{seconds:.3f}"),
    )


def test_dry_run_returns_literal_order_without_preparation_or_primitive_io():
    from src.bioxp import oem_serial206_initialization as subject

    primitives = FakeSerial206Primitives()
    result = _provider(subject, primitives, DurableStore(), ReferenceStore()).initialize_motors(mode="dry_run")

    assert result["ok"] is True
    assert result["ready"] is False
    assert result["opened_usb"] is False
    assert result["physical_motion_commanded"] is False
    assert [row["stage"] for row in result["stage_receipts"]] == EXPECTED_STAGES
    assert primitives.calls == []


def test_live_initialize_motors_runs_exact_oem_order_and_persists_full_receipts():
    from src.bioxp import oem_serial206_initialization as subject

    store = DurableStore()
    refs = ReferenceStore({"x": {"state": "desynced"}, "y": {"state": "desynced"}})
    primitives = FakeSerial206Primitives()
    provider = _provider(subject, primitives, store, refs)

    result = provider.initialize_motors(
        mode="live",
        approvals=_approvals(subject),
        commissioning=_commissioning(subject),
    )

    assert result["ok"] is True
    assert result["ready"] is True
    assert result["failed_at"] is None
    assert [row["stage"] for row in result["stage_receipts"]] == EXPECTED_STAGES
    assert primitives.calls == [
        "prepare",
        "z-home",
        "gripper-current-31",
        "gripper-clear-10000",
        "gripper-home",
        "x-home",
        "sleep:0.020",
        "x-set-home",
        "x-speed-1700",
        "sleep:0.040",
        "x-park-6000",
        "y-home",
        "door-home",
        "door-closed-predicate",
        "y-set-home",
        "chiller-oc-cool-rate",
        "chiller-rc-cool-rate",
        "gripper-idle-current-10",
    ]
    for receipt in result["stage_receipts"]:
        assert receipt["approval_id"].startswith("T-")
        assert receipt["command_ack"] is not None or receipt["stage"] in {"x-home-settle", "x-speed-settle", "ui-zero-calibrated", "system-status-initialized"}
        assert "wait" in receipt
        assert "switch_predicate" in receipt
        assert "position_delta" in receipt
        assert "home_position" in receipt
        assert "reference_transition" in receipt
        assert "physical_effect_verified" in receipt
        assert "operator_assessment" in receipt
    assert store.ledger["terminal_state"] == "initializeMotors_complete"
    assert refs.rows["x"]["state"] == "referenced"
    assert refs.rows["y"]["state"] == "referenced"


def test_first_failed_stage_stops_and_no_delta_keeps_existing_reference_desynced():
    from src.bioxp import oem_serial206_initialization as subject

    store = DurableStore()
    refs = ReferenceStore({"x": {"state": "desynced", "origin_position_steps": 123}})
    primitives = FakeSerial206Primitives(no_delta_stage="x-home")
    result = _provider(subject, primitives, store, refs).initialize_motors(
        mode="live",
        approvals=_approvals(subject),
        commissioning=_commissioning(subject),
    )

    assert result["ok"] is False
    assert result["failed_at"] == "x-home"
    assert result["stage_receipts"][-1]["failure"] == "ambiguous_or_unverified_stage_effect"
    assert "x-park-6000" not in primitives.calls
    assert refs.rows["x"]["state"] == "desynced"
    assert refs.rows["x"]["origin_position_steps"] == 123


def test_z_fails_closed_without_fresh_same_epoch_gap9_gap10_commissioning_before_io():
    from src.bioxp import oem_serial206_initialization as subject

    primitives = FakeSerial206Primitives()
    result = _provider(subject, primitives, DurableStore(), ReferenceStore()).initialize_motors(
        mode="live",
        approvals=_approvals(subject),
        commissioning=_commissioning(subject, z_gap=False),
    )

    assert result["ok"] is False
    assert result["failed_at"] == "admission"
    assert "z_gap9_gap10_polarity_not_explicitly_commissioned" in result["blockers"]
    assert primitives.calls == []


def test_single_use_approval_cannot_authorize_a_second_motion():
    from src.bioxp import oem_serial206_initialization as subject

    store = DurableStore()
    approvals = _approvals(subject)
    first_primitives = FakeSerial206Primitives(fail_stage="gripper-current-31")
    first = _provider(subject, first_primitives, store, ReferenceStore()).initialize_motors(
        mode="live", approvals=approvals, commissioning=_commissioning(subject)
    )
    assert first["failed_at"] == "gripper-current-31"

    second_primitives = FakeSerial206Primitives()
    second = _provider(subject, second_primitives, store, ReferenceStore()).initialize_motors(
        mode="live", approvals=approvals, commissioning=_commissioning(subject)
    )
    assert second["ok"] is False
    assert second["failed_at"] == "admission"
    assert "approval_id_already_used:T-z-home" in second["blockers"]
    assert second_primitives.calls == []


def _motion_approvals(subject, generation=11):
    approvals = {}
    for spec in subject.SERIAL206_INITIALIZE_MOTION_STAGE_SPECS:
        approvals[spec.key] = subject.Serial206StageApproval(
            approval_id=f"IM-{spec.key}",
            expected_generation=generation,
            expected_component=spec.component,
            expected_direction=spec.direction,
            expected_bound=spec.bound,
            operator_note=f"Operator approved {spec.key}",
            idempotency_key=f"im-idem-{spec.key}",
        )
    return approvals


def test_initialize_motion_dry_run_plans_both_tip_branches_without_any_io():
    from src.bioxp import oem_serial206_initialization as subject

    primitives = FakeSerial206Primitives(tips=(True, False, False, False))
    result = _provider(subject, primitives, DurableStore(), ReferenceStore()).initialize_motion(mode="dry_run")

    assert result["ok"] is True
    assert result["ready"] is False
    assert result["branch"] == "undetermined_dry_run"
    assert result["physical_motion_commanded"] is False
    assert result["opened_usb"] is False
    assert [row["stage"] for row in result["stage_receipts"]] == [
        spec.key for spec in subject.SERIAL206_INITIALIZE_MOTION_STAGE_SPECS
    ]
    assert primitives.calls == []


def test_initialize_motion_no_tip_branch_stops_after_exact_no_tip_state_update():
    from src.bioxp import oem_serial206_initialization as subject

    store = DurableStore()
    primitives = FakeSerial206Primitives(tips=(False, False, False, False))
    result = _provider(subject, primitives, store, ReferenceStore()).initialize_motion(
        mode="live",
        motor_approvals=_approvals(subject),
        motion_approvals=_motion_approvals(subject),
        commissioning=_commissioning(subject),
    )

    assert result["ok"] is True
    assert result["ready"] is True
    assert result["branch"] == "no_tip"
    motion_stages = [row["stage"] for row in result["stage_receipts"]]
    assert motion_stages == [
        "initializeMotion.stop_scripts",
        "initializeMotion.clear_forceabort",
        "initializeMotion.initializeMotors",
        "initializeMotion.thermal_door_closed",
        "initializeMotion.queryTipStatus.initial",
        "initializeMotion.sleep.after_tip_query",
        "initializeMotion.tip_loaded_false.no_tip",
    ]
    assert "initializeMotion.openThermalDoor.tip_exists" not in primitives.calls
    assert "initializeMotion.ejectAllTips.tip_exists" not in primitives.calls
    assert "pipette-initiate-group" not in primitives.calls
    assert store.motion_ledger["terminal_state"] == "initializeMotion_complete_no_tip"


def test_initialize_motion_stale_tip_branch_runs_literal_cleanup_and_pipette_checks():
    from src.bioxp import oem_serial206_initialization as subject

    store = DurableStore()
    primitives = FakeSerial206Primitives(
        tip_queries=[(True, False, True, False), (False, False, False, False)]
    )
    result = _provider(subject, primitives, store, ReferenceStore()).initialize_motion(
        mode="live",
        motor_approvals=_approvals(subject),
        motion_approvals=_motion_approvals(subject),
        commissioning=_commissioning(subject),
    )

    assert result["ok"] is True
    assert result["branch"] == "stale_tip_remediation"
    assert [row["stage"] for row in result["stage_receipts"]] == [
        "initializeMotion.stop_scripts",
        "initializeMotion.clear_forceabort",
        "initializeMotion.initializeMotors",
        "initializeMotion.thermal_door_closed",
        "initializeMotion.queryTipStatus.initial",
        "initializeMotion.sleep.after_tip_query",
        "initializeMotion.openThermalDoor.tip_exists",
        "initializeMotion.thermal_door_open.tip_exists",
        "initializeMotion.tip_loaded.tip_exists",
        "initializeMotion.scriptmoveTo.tip_exists",
        "initializeMotion.updateLocation.tip_exists",
        "initializeMotion.ejectAllTips.tip_exists",
        "initializeMotion.moveZ.tip_exists",
        "initializeMotion.moveX.tip_exists",
        "initializeMotion.queryTipStatus.after_eject",
        "initializeMotion.sleep.after_eject_query",
        "initializeMotion.tip_dirty_false",
        "initializeMotion.tip_loaded_false.after_eject",
        "initializeMotion.sleep.before_initiate_group",
        "initializeMotion.initiateGroup.initial",
        "initializeMotion.checkedPipetteStatus.initial",
    ]
    assert store.motion_ledger["terminal_state"] == "initializeMotion_complete_stale_tip"
    assert all(row["approval_id"].startswith("IM-") for row in result["stage_receipts"] if row["stage"] != "initializeMotion.initializeMotors")


def test_stale_tip_remediation_fails_closed_when_post_eject_query_is_not_empty():
    from src.bioxp import oem_serial206_initialization as subject

    primitives = FakeSerial206Primitives(
        tip_queries=[(True, False, False, False), (True, False, False, False)]
    )
    result = _provider(subject, primitives, DurableStore(), ReferenceStore()).initialize_motion(
        mode="live",
        motor_approvals=_approvals(subject),
        motion_approvals=_motion_approvals(subject),
        commissioning=_commissioning(subject),
    )

    assert result["ok"] is False
    assert result["failed_at"] == "initializeMotion.queryTipStatus.after_eject"
    assert result["stage_receipts"][-1]["failure"] == "stale_tip_remains_after_eject"
    assert "initializeMotion.tip_dirty_false" not in primitives.calls
    assert "pipette-initiate-group" not in primitives.calls
