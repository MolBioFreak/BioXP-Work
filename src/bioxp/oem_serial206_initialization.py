"""Literal serial-206 OEM initializeMotors/initializeMotion provider.

The provider composes existing robot primitives.  It deliberately owns no TMCL
protocol encoding and never substitutes generic axis motion for an OEM stage.
"""
from __future__ import annotations

import copy
import threading
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .oem_movement_ledger import OemMovementLedger
from .services.reference_service import MarkAxisDesyncedCommand, MarkAxisReferencedCommand


@dataclass(frozen=True)
class Serial206StageApproval:
    approval_id: str
    expected_generation: int
    expected_component: str
    expected_direction: str
    expected_bound: int
    operator_note: str
    idempotency_key: str


@dataclass(frozen=True)
class Serial206CommissioningEvidence:
    component: str
    generation: int
    fresh: bool
    direction_verified: bool
    limits_verified: bool
    switch_verified: bool
    stop_verified: bool
    reference_verified: bool
    gap9_polarity: int | None = None
    gap10_polarity: int | None = None


@dataclass(frozen=True)
class Serial206StageSpec:
    key: str
    component: str
    direction: str
    bound: int
    movement: bool = False
    establishes_reference: bool = False


SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS: tuple[Serial206StageSpec, ...] = (
    Serial206StageSpec("z-home", "z", "toward-gap9-home", 160000, True, True),
    Serial206StageSpec("gripper-current-31", "g", "none", 0),
    Serial206StageSpec("gripper-clear-10000", "g", "positive", 10000, True),
    Serial206StageSpec("gripper-home", "g", "commissioned-home", 15000, True, True),
    Serial206StageSpec("x-home", "x", "toward-home", 91919, True, True),
    Serial206StageSpec("x-home-settle", "x", "none", 0),
    Serial206StageSpec("x-set-home", "x", "none", 0, False, True),
    Serial206StageSpec("x-speed-1700", "x", "none", 0),
    Serial206StageSpec("x-speed-settle", "x", "none", 0),
    Serial206StageSpec("x-park-6000", "x", "positive", 6000, True),
    Serial206StageSpec("y-home", "y", "toward-home", 95247, True, True),
    Serial206StageSpec("door-home", "door", "toward-closed-home", 18500, True, True),
    Serial206StageSpec("door-closed-predicate", "door", "none", 0),
    Serial206StageSpec("y-set-home", "y", "none", 0, False, True),
    Serial206StageSpec("ui-zero-calibrated", "ui", "none", 0),
    Serial206StageSpec("chiller-oc-cool-rate", "chiller-oc", "none", 0),
    Serial206StageSpec("chiller-rc-cool-rate", "chiller-rc", "none", 0),
    Serial206StageSpec("system-status-initialized", "system", "none", 0),
    Serial206StageSpec("gripper-idle-current-10", "g", "none", 0),
)

SERIAL206_INITIALIZE_MOTION_STAGE_SPECS: tuple[Serial206StageSpec, ...] = (
    Serial206StageSpec("initializeMotion.stop_scripts", "system", "none", 0),
    Serial206StageSpec("initializeMotion.clear_forceabort", "system", "none", 0),
    Serial206StageSpec("initializeMotion.initializeMotors", "system", "source-order", 19),
    Serial206StageSpec("initializeMotion.thermal_door_closed", "door", "host-state-closed", 0),
    Serial206StageSpec("initializeMotion.queryTipStatus.initial", "pipette", "query", 4),
    Serial206StageSpec("initializeMotion.sleep.after_tip_query", "system", "none", 500),
    Serial206StageSpec("initializeMotion.openThermalDoor.tip_exists", "door", "toward-open", 18500, True),
    Serial206StageSpec("initializeMotion.thermal_door_open.tip_exists", "door", "predicate-open", 0),
    Serial206StageSpec("initializeMotion.tip_loaded.tip_exists", "pipette", "host-state-loaded", 0),
    Serial206StageSpec("initializeMotion.scriptmoveTo.tip_exists", "gantry", "location-28-to-6", 1, True),
    Serial206StageSpec("initializeMotion.updateLocation.tip_exists", "gantry", "host-location-6", 0),
    Serial206StageSpec("initializeMotion.ejectAllTips.tip_exists", "pipette", "eject", 4, True),
    Serial206StageSpec("initializeMotion.moveZ.tip_exists", "z", "absolute-positive", 80000, True),
    Serial206StageSpec("initializeMotion.moveX.tip_exists", "x", "absolute-positive", 79000, True),
    Serial206StageSpec("initializeMotion.queryTipStatus.after_eject", "pipette", "query-empty", 4),
    Serial206StageSpec("initializeMotion.sleep.after_eject_query", "system", "none", 100),
    Serial206StageSpec("initializeMotion.tip_dirty_false", "pipette", "host-state-clean", 0),
    Serial206StageSpec("initializeMotion.tip_loaded_false.after_eject", "pipette", "host-state-empty", 0),
    Serial206StageSpec("initializeMotion.sleep.before_initiate_group", "system", "none", 2),
    Serial206StageSpec("initializeMotion.initiateGroup.initial", "pipette", "initialize-group", 4),
    Serial206StageSpec("initializeMotion.checkedPipetteStatus.initial", "pipette", "check-group", 4),
    Serial206StageSpec("initializeMotion.initiateGroup.retry", "pipette", "initialize-group-retry", 4),
    Serial206StageSpec("initializeMotion.checkedPipetteStatus.retry", "pipette", "check-group-retry", 4),
    Serial206StageSpec("initializeMotion.tip_loaded_false.no_tip", "pipette", "host-state-empty", 0),
)
_SERIAL206_MOTION_BY_KEY = {spec.key: spec for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS}


_COMMISSIONED_COMPONENTS = ("z", "g", "x", "y", "door")
_HOME_STAGE_AXIS = {
    "z-home": "z",
    "gripper-home": "g",
    "x-home": "x",
    "x-set-home": "x",
    "y-home": "y",
    "y-set-home": "y",
    "door-home": "door",
}


class Serial206OemInitializationProvider:
    """Execute the immutable serial-206 source order through existing primitives."""

    source_mode = "ClassControlInterface.initializeMotors:3348-3421"
    schema = "bioxp.serial206_oem_initialization.v1"

    def __init__(
        self,
        primitives: Any,
        *,
        ledger_store: Any | None = None,
        approval_store: Any | None = None,
        reference_store: Any | None = None,
        generation_provider: Callable[[], int] | None = None,
        preparation_provider: Any | None = None,
        sleep: Callable[[float], None] | None = None,
    ) -> None:
        self.primitives = primitives
        self.ledger = OemMovementLedger(ledger_store)
        self.approval_store = approval_store
        self.reference_store = reference_store
        self.generation_provider = generation_provider or (lambda: 0)
        self.preparation_provider = preparation_provider
        if sleep is None:
            import time

            sleep = time.sleep
        self.sleep = sleep
        self._lock = threading.RLock()
        self._memory_approval_state: dict[str, Any] = {"used": {}}

    def _approval_state(self) -> dict[str, Any]:
        if self.approval_store is not None and hasattr(self.approval_store, "read_oem_stage_approvals"):
            stored = self.approval_store.read_oem_stage_approvals()
            if isinstance(stored, dict):
                return copy.deepcopy(stored)
        return copy.deepcopy(self._memory_approval_state)

    def _save_approval_state(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        state = copy.deepcopy(dict(payload))
        state.setdefault("used", {})
        if self.approval_store is not None and hasattr(self.approval_store, "write_oem_stage_approvals"):
            self.approval_store.write_oem_stage_approvals(state)
        self._memory_approval_state = copy.deepcopy(state)
        return state

    @staticmethod
    def _approval_blockers(
        approvals: Mapping[str, Serial206StageApproval],
        *,
        generation: int,
        used: Mapping[str, Any],
        specs: tuple[Serial206StageSpec, ...] = SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS,
    ) -> list[str]:
        blockers: list[str] = []
        request_ids: set[str] = set()
        request_idempotency: set[str] = set()
        for spec in specs:
            approval = approvals.get(spec.key)
            if not isinstance(approval, Serial206StageApproval):
                blockers.append(f"stage_approval_required:{spec.key}")
                continue
            if not approval.approval_id.strip():
                blockers.append(f"approval_id_required:{spec.key}")
            elif approval.approval_id in used:
                blockers.append(f"approval_id_already_used:{approval.approval_id}")
            elif approval.approval_id in request_ids:
                blockers.append(f"approval_id_reused_in_request:{approval.approval_id}")
            request_ids.add(approval.approval_id)
            if approval.idempotency_key in request_idempotency:
                blockers.append(f"idempotency_key_reused_in_request:{approval.idempotency_key}")
            request_idempotency.add(approval.idempotency_key)
            if approval.expected_generation != generation:
                blockers.append(f"approval_generation_mismatch:{spec.key}")
            if approval.expected_component != spec.component:
                blockers.append(f"approval_component_mismatch:{spec.key}")
            if approval.expected_direction != spec.direction:
                blockers.append(f"approval_direction_mismatch:{spec.key}")
            if approval.expected_bound != spec.bound:
                blockers.append(f"approval_bound_mismatch:{spec.key}")
            if not approval.operator_note.strip():
                blockers.append(f"operator_note_required:{spec.key}")
            if not approval.idempotency_key.strip():
                blockers.append(f"idempotency_key_required:{spec.key}")
        return blockers

    @staticmethod
    def _commissioning_blockers(
        commissioning: Mapping[str, Serial206CommissioningEvidence], *, generation: int
    ) -> list[str]:
        blockers: list[str] = []
        for component in _COMMISSIONED_COMPONENTS:
            evidence = commissioning.get(component)
            if not isinstance(evidence, Serial206CommissioningEvidence):
                blockers.append(f"commissioning_evidence_required:{component}")
                continue
            if evidence.component != component:
                blockers.append(f"commissioning_component_mismatch:{component}")
            if evidence.generation != generation or evidence.fresh is not True:
                blockers.append(f"fresh_same_epoch_commissioning_required:{component}")
            for gate in ("direction_verified", "limits_verified", "switch_verified", "stop_verified", "reference_verified"):
                if getattr(evidence, gate) is not True:
                    blockers.append(f"{component}_{gate}_required")
            if component == "z" and not (
                evidence.gap9_polarity in {0, 1}
                and evidence.gap10_polarity in {0, 1}
                and evidence.gap9_polarity != evidence.gap10_polarity
            ):
                blockers.append("z_gap9_gap10_polarity_not_explicitly_commissioned")
        return blockers

    def _dry_run_receipt(self, spec: Serial206StageSpec) -> dict[str, Any]:
        return {
            "stage": spec.key,
            "status": "planned",
            "component": spec.component,
            "direction": spec.direction,
            "bound": spec.bound,
            "approval_id": None,
            "command_ack": None,
            "wait": None,
            "switch_predicate": None,
            "position_delta": None,
            "home_position": None,
            "reference_transition": None,
            "physical_effect_verified": False,
            "operator_assessment": None,
            "operator_note": None,
            "opened_usb": False,
            "physical_motion_commanded": False,
        }

    def initialize_motors(
        self,
        *,
        mode: str,
        approvals: Mapping[str, Serial206StageApproval] | None = None,
        commissioning: Mapping[str, Serial206CommissioningEvidence] | None = None,
        timeout_s: float = 180.0,
    ) -> dict[str, Any]:
        selected_mode = str(mode).strip().lower()
        if selected_mode == "dry_run":
            return {
                "ok": True,
                "ready": False,
                "schema": self.schema,
                "source_mode": self.source_mode,
                "mode": "dry_run",
                "opened_usb": False,
                "physical_motion_commanded": False,
                "failed_at": None,
                "blockers": ["dry_run_is_not_live_initialization"],
                "stage_receipts": [self._dry_run_receipt(spec) for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS],
            }
        if selected_mode != "live":
            return self._admission_failure(["mode_must_be_dry_run_or_live"])

        with self._lock:
            generation = int(self.generation_provider())
            approval_rows = dict(approvals or {})
            commissioning_rows = dict(commissioning or {})
            approval_state = self._approval_state()
            used = approval_state.setdefault("used", {})
            blockers = self._approval_blockers(approval_rows, generation=generation, used=used)
            blockers.extend(self._commissioning_blockers(commissioning_rows, generation=generation))
            if self.preparation_provider is None or not hasattr(
                self.preparation_provider, "prepare_for_initialize_motors"
            ):
                blockers.append("serial206_board_no_motion_preparation_provider_not_bound")
            if blockers:
                return self._admission_failure(blockers, generation=generation)

            preparation = self.preparation_provider.prepare_for_initialize_motors(
                expected_generation=generation
            )
            prep_ok = bool(
                isinstance(preparation, dict)
                and preparation.get("ok") is True
                and preparation.get("observed_generation") == generation
                and preparation.get("board_preparation_verified") is True
                and preparation.get("initialize_without_motion_verified") is True
                and preparation.get("physical_motion") is False
            )
            if not prep_ok:
                return self._admission_failure(
                    ["serial206_board_no_motion_preparation_not_verified"],
                    generation=generation,
                    preparation=preparation,
                )

            receipts: list[dict[str, Any]] = []
            for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS:
                approval = approval_rows[spec.key]
                used[approval.approval_id] = {
                    "stage": spec.key,
                    "generation": generation,
                    "idempotency_key": approval.idempotency_key,
                }
                self._save_approval_state(approval_state)
                admitted = self.ledger.admit(stage=spec.key, command_id=approval.approval_id)
                if not admitted.get("ok"):
                    return self._finish(
                        ok=False,
                        generation=generation,
                        preparation=preparation,
                        receipts=receipts,
                        failed_at=spec.key,
                        blockers=[str(admitted.get("blocker"))],
                    )
                raw = self._execute_stage(spec, timeout_s=float(timeout_s))
                receipt = self._receipt(spec, approval, raw)
                transition = self._reference_transition(spec, receipt)
                receipt["reference_transition"] = transition
                receipts.append(receipt)
                ledger_state = self.ledger.record_result(
                    stage=spec.key,
                    command_id=approval.approval_id,
                    result=receipt,
                    artifact_path=None,
                )
                if receipt["ok"] and ledger_state["stages"][spec.key]["state"] == "acknowledged":
                    observation = self.ledger.record_observation(
                        stage=spec.key,
                        observed_pass=receipt["operator_assessment"] == "pass",
                        note=approval.operator_note,
                        command_id=approval.approval_id,
                    )
                    if observation.get("ok") is not True:
                        receipt["ok"] = False
                        receipt["failure"] = str(observation.get("blocker") or "operator_assessment_not_accepted")
                if not receipt["ok"]:
                    return self._finish(
                        ok=False,
                        generation=generation,
                        preparation=preparation,
                        receipts=receipts,
                        failed_at=spec.key,
                        blockers=[str(receipt.get("failure") or "stage_failed")],
                    )
            return self._finish(
                ok=True,
                generation=generation,
                preparation=preparation,
                receipts=receipts,
                failed_at=None,
                blockers=[],
            )

    def initialize_motion(
        self,
        *,
        mode: str,
        motor_approvals: Mapping[str, Serial206StageApproval] | None = None,
        motion_approvals: Mapping[str, Serial206StageApproval] | None = None,
        commissioning: Mapping[str, Serial206CommissioningEvidence] | None = None,
        timeout_s: float = 180.0,
    ) -> dict[str, Any]:
        selected_mode = str(mode).strip().lower()
        if selected_mode == "dry_run":
            return {
                "ok": True,
                "ready": False,
                "schema": "bioxp.serial206_initializeMotion.v1",
                "source_mode": "ControlLib.initializeMotion:8797-8856",
                "mode": "dry_run",
                "branch": "undetermined_dry_run",
                "opened_usb": False,
                "physical_motion_commanded": False,
                "failed_at": None,
                "blockers": ["dry_run_is_not_live_initialization"],
                "stage_receipts": [self._dry_run_receipt(spec) for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS],
            }
        if selected_mode != "live":
            return self._motion_failure("admission", ["mode_must_be_dry_run_or_live"], [])

        with self._lock:
            generation = int(self.generation_provider())
            motor_rows = dict(motor_approvals or {})
            motion_rows = dict(motion_approvals or {})
            commissioning_rows = dict(commissioning or {})
            approval_state = self._approval_state()
            used = approval_state.setdefault("used", {})
            blockers = self._approval_blockers(motor_rows, generation=generation, used=used)
            blockers.extend(
                self._approval_blockers(
                    motion_rows,
                    generation=generation,
                    used=used,
                    specs=SERIAL206_INITIALIZE_MOTION_STAGE_SPECS,
                )
            )
            blockers.extend(self._commissioning_blockers(commissioning_rows, generation=generation))
            all_ids = [approval.approval_id for approval in motor_rows.values() if isinstance(approval, Serial206StageApproval)]
            all_ids.extend(approval.approval_id for approval in motion_rows.values() if isinstance(approval, Serial206StageApproval))
            if len(all_ids) != len(set(all_ids)):
                blockers.append("approval_id_reused_across_initializeMotion_and_initializeMotors")
            if blockers:
                return self._motion_failure("admission", blockers, [], generation=generation)

            receipts: list[dict[str, Any]] = []
            for stage in ("initializeMotion.stop_scripts", "initializeMotion.clear_forceabort"):
                receipt = self._run_motion_stage(stage, motion_rows[stage], approval_state, timeout_s)
                receipts.append(receipt)
                self._persist_motion_ledger(receipts, "running", generation)
                if not receipt["ok"]:
                    return self._motion_failure(stage, [receipt["failure"]], receipts, generation=generation)

            init_approval = motion_rows["initializeMotion.initializeMotors"]
            self._consume_approval(init_approval, "initializeMotion.initializeMotors", approval_state)
            initialized = self.initialize_motors(
                mode="live",
                approvals=motor_rows,
                commissioning=commissioning_rows,
                timeout_s=timeout_s,
            )
            init_receipt = {
                "stage": "initializeMotion.initializeMotors",
                "status": "passed" if initialized.get("ok") is True and initialized.get("ready") is True else "failed",
                "ok": initialized.get("ok") is True and initialized.get("ready") is True,
                "failure": None if initialized.get("ok") is True and initialized.get("ready") is True else "initializeMotors_not_accepted",
                "component": "system",
                "direction": "source-order",
                "bound": 19,
                "approval_id": init_approval.approval_id,
                "idempotency_key": init_approval.idempotency_key,
                "expected_generation": init_approval.expected_generation,
                "operator_note": init_approval.operator_note,
                "command_ack": {"accepted": initialized.get("ok") is True, "ready": initialized.get("ready") is True},
                "wait": None,
                "switch_predicate": None,
                "position_delta": None,
                "home_position": None,
                "reference_transition": None,
                "physical_effect_verified": initialized.get("ready") is True,
                "operator_assessment": "pass" if initialized.get("ready") is True else "fail",
                "initialize_motors": initialized,
            }
            receipts.append(init_receipt)
            self._persist_motion_ledger(receipts, "running", generation)
            if not init_receipt["ok"]:
                return self._motion_failure(init_receipt["stage"], [init_receipt["failure"]], receipts, generation=generation)

            prefix = (
                "initializeMotion.thermal_door_closed",
                "initializeMotion.queryTipStatus.initial",
                "initializeMotion.sleep.after_tip_query",
            )
            for stage in prefix:
                receipt = self._run_motion_stage(stage, motion_rows[stage], approval_state, timeout_s)
                receipts.append(receipt)
                self._persist_motion_ledger(receipts, "running", generation)
                if not receipt["ok"]:
                    return self._motion_failure(stage, [receipt["failure"]], receipts, generation=generation)
            tips = receipts[-2].get("tip_channels")
            if not isinstance(tips, list) or len(tips) != 4 or any(type(value) is not bool for value in tips):
                return self._motion_failure(
                    "initializeMotion.queryTipStatus.initial",
                    ["tip_status_not_exact_four_channel_boolean_evidence"],
                    receipts,
                    generation=generation,
                )
            if not any(tips):
                stage = "initializeMotion.tip_loaded_false.no_tip"
                receipt = self._run_motion_stage(stage, motion_rows[stage], approval_state, timeout_s)
                receipts.append(receipt)
                terminal = "initializeMotion_complete_no_tip" if receipt["ok"] else "failed_closed"
                self._persist_motion_ledger(receipts, terminal, generation, branch="no_tip")
                if not receipt["ok"]:
                    return self._motion_failure(stage, [receipt["failure"]], receipts, generation=generation, branch="no_tip")
                return self._motion_success(receipts, generation, "no_tip")

            stale_order = (
                "initializeMotion.openThermalDoor.tip_exists",
                "initializeMotion.thermal_door_open.tip_exists",
                "initializeMotion.tip_loaded.tip_exists",
                "initializeMotion.scriptmoveTo.tip_exists",
                "initializeMotion.updateLocation.tip_exists",
                "initializeMotion.ejectAllTips.tip_exists",
                "initializeMotion.moveZ.tip_exists",
                "initializeMotion.moveX.tip_exists",
                "initializeMotion.queryTipStatus.after_eject",
            )
            for stage in stale_order:
                receipt = self._run_motion_stage(stage, motion_rows[stage], approval_state, timeout_s)
                receipts.append(receipt)
                self._persist_motion_ledger(receipts, "running", generation, branch="stale_tip_remediation")
                if not receipt["ok"]:
                    return self._motion_failure(stage, [receipt["failure"]], receipts, generation=generation, branch="stale_tip_remediation")
            after = receipts[-1].get("tip_channels")
            if not isinstance(after, list) or len(after) != 4 or any(type(value) is not bool for value in after):
                receipts[-1]["ok"] = False
                receipts[-1]["status"] = "failed"
                receipts[-1]["failure"] = "tip_status_not_exact_four_channel_boolean_evidence"
                return self._motion_failure(stale_order[-1], [receipts[-1]["failure"]], receipts, generation=generation, branch="stale_tip_remediation")
            if any(after):
                receipts[-1]["ok"] = False
                receipts[-1]["status"] = "failed"
                receipts[-1]["failure"] = "stale_tip_remains_after_eject"
                self._persist_motion_ledger(receipts, "failed_closed", generation, branch="stale_tip_remediation")
                return self._motion_failure(stale_order[-1], [receipts[-1]["failure"]], receipts, generation=generation, branch="stale_tip_remediation")

            cleanup_tail = (
                "initializeMotion.sleep.after_eject_query",
                "initializeMotion.tip_dirty_false",
                "initializeMotion.tip_loaded_false.after_eject",
                "initializeMotion.sleep.before_initiate_group",
                "initializeMotion.initiateGroup.initial",
                "initializeMotion.checkedPipetteStatus.initial",
            )
            for stage in cleanup_tail:
                receipt = self._run_motion_stage(stage, motion_rows[stage], approval_state, timeout_s)
                receipts.append(receipt)
                self._persist_motion_ledger(receipts, "running", generation, branch="stale_tip_remediation")
                if not receipt["ok"] and stage != "initializeMotion.checkedPipetteStatus.initial":
                    return self._motion_failure(stage, [receipt["failure"]], receipts, generation=generation, branch="stale_tip_remediation")
            if not receipts[-1]["ok"]:
                for stage in ("initializeMotion.initiateGroup.retry", "initializeMotion.checkedPipetteStatus.retry"):
                    receipt = self._run_motion_stage(stage, motion_rows[stage], approval_state, timeout_s)
                    receipts.append(receipt)
                    self._persist_motion_ledger(receipts, "running", generation, branch="stale_tip_remediation")
                    if not receipt["ok"]:
                        return self._motion_failure(stage, [receipt["failure"]], receipts, generation=generation, branch="stale_tip_remediation")
            self._persist_motion_ledger(receipts, "initializeMotion_complete_stale_tip", generation, branch="stale_tip_remediation")
            return self._motion_success(receipts, generation, "stale_tip_remediation")

    def _consume_approval(self, approval: Serial206StageApproval, stage: str, state: dict[str, Any]) -> None:
        state.setdefault("used", {})[approval.approval_id] = {
            "stage": stage,
            "generation": approval.expected_generation,
            "idempotency_key": approval.idempotency_key,
        }
        self._save_approval_state(state)

    def _run_motion_stage(
        self,
        stage: str,
        approval: Serial206StageApproval,
        approval_state: dict[str, Any],
        timeout_s: float,
    ) -> dict[str, Any]:
        spec = _SERIAL206_MOTION_BY_KEY[stage]
        self._consume_approval(approval, stage, approval_state)
        p = self.primitives
        if stage == "initializeMotion.stop_scripts":
            raw = p.set_stop_scripts(False)
        elif stage == "initializeMotion.clear_forceabort":
            raw = p.set_force_abort(False)
        elif stage == "initializeMotion.thermal_door_closed":
            raw = p.set_thermal_door_state(False)
        elif stage in {"initializeMotion.queryTipStatus.initial", "initializeMotion.queryTipStatus.after_eject"}:
            observed = p.query_tip_status()
            raw = {
                **(observed if isinstance(observed, dict) else {"ok": False, "raw": observed}),
                "physical_effect_verified": False,
                "operator_assessment": "pass" if isinstance(observed, dict) and observed.get("ok") is True else "fail",
            }
        elif stage == "initializeMotion.sleep.after_tip_query":
            self.sleep(0.500)
            raw = {"ok": True, "wait_ms": 500, "operator_assessment": "pass", "physical_effect_verified": False}
        elif stage == "initializeMotion.openThermalDoor.tip_exists":
            raw = p.set_thermal_door_state(True)
        elif stage == "initializeMotion.thermal_door_open.tip_exists":
            raw = p.confirm_thermal_door(True)
        elif stage == "initializeMotion.tip_loaded.tip_exists":
            raw = p.set_tip_loaded(True)
        elif stage == "initializeMotion.scriptmoveTo.tip_exists":
            raw = p.scriptmove_to(from_location=28, from_well=0, to_location=6, column=0, row=0)
        elif stage == "initializeMotion.updateLocation.tip_exists":
            raw = p.update_location(6)
        elif stage == "initializeMotion.ejectAllTips.tip_exists":
            raw = p.eject_all_tips()
        elif stage == "initializeMotion.moveZ.tip_exists":
            raw = p.move_z_absolute(80000)
        elif stage == "initializeMotion.moveX.tip_exists":
            raw = p.move_x_absolute(79000)
        elif stage == "initializeMotion.sleep.after_eject_query":
            self.sleep(0.100)
            raw = {"ok": True, "wait_ms": 100, "operator_assessment": "pass", "physical_effect_verified": False}
        elif stage == "initializeMotion.tip_dirty_false":
            raw = p.set_tip_dirty(False)
        elif stage == "initializeMotion.tip_loaded_false.after_eject":
            raw = p.set_tip_loaded(False, source_stage="after_eject")
        elif stage == "initializeMotion.sleep.before_initiate_group":
            self.sleep(0.002)
            raw = {"ok": True, "wait_ms": 2, "operator_assessment": "pass", "physical_effect_verified": False}
        elif stage in {"initializeMotion.initiateGroup.initial", "initializeMotion.initiateGroup.retry"}:
            raw = p.initiate_pipette_group()
        elif stage in {"initializeMotion.checkedPipetteStatus.initial", "initializeMotion.checkedPipetteStatus.retry"}:
            raw = p.checked_pipette_status()
        elif stage == "initializeMotion.tip_loaded_false.no_tip":
            raw = p.set_tip_loaded(False, source_stage="no_tip")
        else:
            raise RuntimeError(f"unmapped serial-206 initializeMotion stage: {stage}")
        receipt = self._receipt(spec, approval, raw)
        if stage.startswith("initializeMotion.queryTipStatus") and isinstance(raw, dict):
            receipt["tip_channels"] = copy.deepcopy(raw.get("channels"))
        return receipt

    def _persist_motion_ledger(
        self,
        receipts: list[Mapping[str, Any]],
        terminal_state: str,
        generation: int,
        *,
        branch: str | None = None,
    ) -> dict[str, Any]:
        payload = {
            "schema": "bioxp.serial206_initializeMotion_ledger.v1",
            "source_anchor": "ControlLib.initializeMotion:8797-8856",
            "generation": generation,
            "branch": branch,
            "terminal_state": terminal_state,
            "stage_receipts": copy.deepcopy(receipts),
        }
        if self.approval_store is not None and hasattr(self.approval_store, "write_oem_initialize_motion_ledger"):
            self.approval_store.write_oem_initialize_motion_ledger(payload)
        return payload

    def _motion_failure(
        self,
        failed_at: str,
        blockers: list[Any],
        receipts: list[dict[str, Any]],
        *,
        generation: int | None = None,
        branch: str | None = None,
    ) -> dict[str, Any]:
        if receipts and generation is not None:
            self._persist_motion_ledger(receipts, "failed_closed", generation, branch=branch)
        return {
            "ok": False,
            "ready": False,
            "schema": "bioxp.serial206_initializeMotion.v1",
            "source_mode": "ControlLib.initializeMotion:8797-8856",
            "mode": "live",
            "branch": branch,
            "opened_usb": bool(receipts),
            "physical_motion_commanded": any(row.get("physical_effect_verified") is True for row in receipts),
            "generation": generation,
            "failed_at": failed_at,
            "blockers": [str(value) for value in blockers],
            "stage_receipts": receipts,
        }

    def _motion_success(self, receipts: list[dict[str, Any]], generation: int, branch: str) -> dict[str, Any]:
        return {
            "ok": True,
            "ready": True,
            "schema": "bioxp.serial206_initializeMotion.v1",
            "source_mode": "ControlLib.initializeMotion:8797-8856",
            "mode": "live",
            "branch": branch,
            "opened_usb": True,
            "physical_motion_commanded": any(row.get("physical_effect_verified") is True for row in receipts),
            "generation": generation,
            "failed_at": None,
            "blockers": [],
            "stage_receipts": receipts,
        }

    def _execute_stage(self, spec: Serial206StageSpec, *, timeout_s: float) -> dict[str, Any]:
        p = self.primitives
        stage = spec.key
        bounded = max(2.0, float(timeout_s))
        if stage == "z-home":
            return p.motor_oem_home_axis("z", startup=True, speed=1791, timeout_s=bounded)
        if stage == "gripper-current-31":
            return p.motor_set_axis_param(4, 6, 31, motor=2)
        if stage == "gripper-clear-10000":
            move = p.motor_move_relative(4, 10000, motor=2)
            wait = p.motor_wait_stopped(4, motor=2, timeout_s=min(bounded, 20.0), require_seen_nonzero=True)
            return self._merge_move_wait(move, wait)
        if stage == "gripper-home":
            return p.motor_oem_home_axis("g", startup=True, speed=200, timeout_s=min(bounded, 30.0))
        if stage == "x-home":
            return p.motor_oem_home_axis("x", startup=True, speed=250, timeout_s=min(bounded, 45.0))
        if stage == "x-home-settle":
            self.sleep(0.020)
            return {"ok": True, "settle_ms": 20, "physical_effect_verified": False, "operator_assessment": "pass"}
        if stage == "x-set-home":
            return p.motor_set_home(5, motor=0)
        if stage == "x-speed-1700":
            return p.motor_set_axis_param(5, 4, 1700, motor=0)
        if stage == "x-speed-settle":
            self.sleep(0.040)
            return {"ok": True, "settle_ms": 40, "physical_effect_verified": False, "operator_assessment": "pass"}
        if stage == "x-park-6000":
            move = p.motor_move_absolute(5, 6000, motor=0)
            wait = p.motor_wait_stopped(5, motor=0, timeout_s=min(bounded, 45.0), require_seen_nonzero=True)
            return self._merge_move_wait(move, wait)
        if stage == "y-home":
            return p.motor_oem_home_axis("y", startup=True, speed=250, timeout_s=min(bounded, 45.0))
        if stage == "door-home":
            return p.motor_oem_door_search_home(startup=True, timeout_s=min(bounded, 45.0))
        if stage == "door-closed-predicate":
            status = p.motor_thermal_door_status()
            predicates = status.get("oem_predicates") if isinstance(status, dict) else None
            closed = predicates.get("tcDoorClosed") if isinstance(predicates, dict) else None
            source = predicates.get("closed_source") if isinstance(predicates, dict) else None
            return {
                **(status if isinstance(status, dict) else {}),
                "ok": closed is True and source == "queryHome(ThermalDoor)",
                "physical_effect_verified": False,
                "operator_assessment": "pass" if closed is True else "fail",
                "failure": None if closed is True else "thermal_door_closed_predicate_not_proven",
            }
        if stage == "y-set-home":
            return p.motor_set_home(4, motor=0)
        if stage == "ui-zero-calibrated":
            machine = p._machine_config_bundle()
            calibration = (((machine or {}).get("config") or {}).get("calibration") or {}) if isinstance(machine, dict) else {}
            calibrated = calibration.get("Calibrated")
            if calibrated in {1, True, "1", "true", "True"}:
                return {"ok": False, "failure": "calibrated_ui_position_sink_not_bound", "physical_effect_verified": False, "operator_assessment": "fail"}
            return {"ok": True, "source_branch_taken": False, "ui_writes": [], "physical_effect_verified": False, "operator_assessment": "pass"}
        if stage == "chiller-oc-cool-rate":
            return p._oem_no_motion_tmcl(name=stage, board=7, command=9, cmd_type=1, motor=0, value=-25)
        if stage == "chiller-rc-cool-rate":
            return p._oem_no_motion_tmcl(name=stage, board=7, command=9, cmd_type=0, motor=0, value=-25)
        if stage == "system-status-initialized":
            return {"ok": True, "durable_robot_state": {"system_status": 1, "ready": True}, "physical_effect_verified": False, "operator_assessment": "pass"}
        if stage == "gripper-idle-current-10":
            return p.motor_set_axis_param(4, 6, 10, motor=2)
        raise RuntimeError(f"unmapped serial-206 OEM stage: {stage}")

    @staticmethod
    def _merge_move_wait(move: Any, wait: Any) -> dict[str, Any]:
        move_row = dict(move) if isinstance(move, dict) else {"ok": False, "raw": move}
        wait_row = dict(wait) if isinstance(wait, dict) else {"stopped": False, "raw": wait}
        return {
            **move_row,
            "ok": move_row.get("ok") is True and wait_row.get("stopped") is True,
            "wait": wait_row,
            "physical_effect_verified": move_row.get("physical_effect_verified") is True,
            "operator_assessment": move_row.get("operator_assessment"),
        }

    @staticmethod
    def _receipt(
        spec: Serial206StageSpec,
        approval: Serial206StageApproval,
        raw: Any,
    ) -> dict[str, Any]:
        result = dict(raw) if isinstance(raw, dict) else {"ok": False, "raw": raw}
        wait = result.get("wait") if isinstance(result.get("wait"), dict) else None
        ack = result.get("command_ack") or result.get("ack")
        switch = result.get("switch_predicate")
        before = result.get("position_before")
        after = result.get("position_after")
        delta = after - before if isinstance(before, int) and isinstance(after, int) else None
        ack_ok = result.get("ok") is True and (
            ack is None
            or (isinstance(ack, dict) and (ack.get("status") == 100 or ack.get("ok") is True))
        )
        wait_ok = not spec.movement or (isinstance(wait, dict) and wait.get("stopped") is True and wait.get("terminal_speed") in {0, None})
        delta_ok = not spec.movement or (isinstance(delta, int) and delta != 0)
        switch_ok = not spec.establishes_reference or (
            isinstance(switch, dict)
            and switch.get("expected") is True
            and switch.get("observed") is True
        )
        reference_agrees = not spec.establishes_reference or result.get("controller_reference_agrees") is True
        physical_ok = not spec.movement or result.get("physical_effect_verified") is True
        operator_assessment = result.get("operator_assessment")
        operator_ok = operator_assessment == "pass"
        ok = bool(ack_ok and wait_ok and delta_ok and switch_ok and reference_agrees and physical_ok and operator_ok)
        failure = result.get("failure")
        if not ok and not failure:
            failure = "ambiguous_or_unverified_stage_effect"
        return {
            "stage": spec.key,
            "status": "passed" if ok else "failed",
            "ok": ok,
            "failure": failure,
            "component": spec.component,
            "direction": spec.direction,
            "bound": spec.bound,
            "approval_id": approval.approval_id,
            "idempotency_key": approval.idempotency_key,
            "expected_generation": approval.expected_generation,
            "operator_note": approval.operator_note,
            "command_ack": ack,
            "wait": wait,
            "switch_predicate": switch,
            "position_before": before,
            "position_after": after,
            "position_delta": delta,
            "home_position": result.get("home_position"),
            "controller_reference_agrees": result.get("controller_reference_agrees"),
            "reference_transition": None,
            "physical_effect_verified": result.get("physical_effect_verified") is True,
            "operator_assessment": operator_assessment,
            "durable_robot_state": result.get("durable_robot_state"),
            "raw_result": result,
        }

    def _reference_transition(self, spec: Serial206StageSpec, receipt: Mapping[str, Any]) -> Any:
        axis = _HOME_STAGE_AXIS.get(spec.key)
        if axis is None or self.reference_store is None:
            return None
        if receipt.get("ok") is True and receipt.get("controller_reference_agrees") is True and (
            receipt.get("physical_effect_verified") is True or not spec.movement
        ):
            return self.reference_store.mark_referenced(
                MarkAxisReferencedCommand(
                    axis=axis,
                    position_steps=int(receipt.get("home_position") or 0),
                    source="serial206_initializeMotors",
                    note=f"Controller and physical evidence agreed at {spec.key}.",
                    motion_kind="oem_initialize_motors_home",
                )
            )
        return self.reference_store.mark_desynced(
            MarkAxisDesyncedCommand(
                axis=axis,
                reason=f"Ambiguous or failed serial-206 initialization stage {spec.key}.",
                source="serial206_initializeMotors",
                motion_kind="oem_initialize_motors_failed",
            )
        )

    def _admission_failure(
        self,
        blockers: list[str],
        *,
        generation: int | None = None,
        preparation: Any = None,
    ) -> dict[str, Any]:
        return {
            "ok": False,
            "ready": False,
            "schema": self.schema,
            "source_mode": self.source_mode,
            "mode": "live",
            "opened_usb": False,
            "physical_motion_commanded": False,
            "generation": generation,
            "preparation": preparation,
            "failed_at": "admission",
            "blockers": blockers,
            "stage_receipts": [],
        }

    def _finish(
        self,
        *,
        ok: bool,
        generation: int,
        preparation: Mapping[str, Any],
        receipts: list[dict[str, Any]],
        failed_at: str | None,
        blockers: list[str],
    ) -> dict[str, Any]:
        return {
            "ok": bool(ok),
            "ready": bool(ok),
            "schema": self.schema,
            "source_mode": self.source_mode,
            "mode": "live",
            "opened_usb": True,
            "physical_motion_commanded": any(
                row["stage"] in {spec.key for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS if spec.movement}
                for row in receipts
            ),
            "generation": generation,
            "preparation": dict(preparation),
            "failed_at": failed_at,
            "blockers": blockers,
            "stage_receipts": receipts,
            "movement_ledger": self.ledger.projection(),
        }
