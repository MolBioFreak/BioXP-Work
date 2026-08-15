"""Physically supervised serial-206 OEM initialization lifecycle.

Every live request may execute only the durable expected-next initializeMotors
stage.  Controller acknowledgement and terminal evidence are recorded first;
physical/operator observation is a separate no-hardware transition.  The three
authority-bearing ledgers are persisted as one atomic state document.
"""
from __future__ import annotations

import copy
import json
import math
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .motion_safety import SERIAL206_X_SWITCH_MASKS, prepare_motion_without_motion
from .oem_compat.machine_state import OemMachineState
from .oem_compat.pathing import OemPathPlanner
from .oem_compat.position_table import load_bound_oem_position_table
from .oem_homing_routes import _execute_oem_steps_live
from .oem_serial206_initialization_contract import (
    OEM_INITIALIZE_MOTORS_STAGE_KEYS,
    SERIAL206_INITIALIZE_MOTORS_LEDGER_SCHEMA,
    advance_initialize_motors_ledger,
    new_initialize_motors_ledger,
)
from .oem_parity_config import load_oem_parity_config
from .pipette.models import PipetteInitCommand
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
_SPEC_BY_KEY = {spec.key: spec for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS}

# This is the audited source order, not an executable capability claim.  The
# production adapter intentionally remains partial because the generic
# scriptmove/state/tip setters are not exact OEM primitives.
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
_MOTION_SPEC_BY_KEY = {spec.key: spec for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS}
_MOTION_STAGE_ORDER = tuple(spec.key for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS)

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
_STATE_SCHEMA = "bioxp.serial206_oem_initialization_state.v2"
_MOTION_LEDGER_SCHEMA = "bioxp.serial206_initializeMotion_ledger.v2"
_Z_LIFECYCLE_SCHEMA = "bioxp.serial206_z_lifecycle.v2"
_X_LIFECYCLE_SCHEMA = "bioxp.serial206_x_lifecycle.v2"
SERIAL206_Z_SELF_TEST_MAX_STEPS = 92049
_LEGACY_Z_LIFECYCLE_SCHEMAS = frozenset({"bioxp.serial206_z_lifecycle.v1"})
_LEGACY_X_LIFECYCLE_SCHEMAS = frozenset({"bioxp.serial206_x_lifecycle.v1"})
_INITIALIZE_MOTION_PARTIAL = (
    "initializeMotion.queryTipStatus.initial",
    "initializeMotion.ejectAllTips.tip_exists",
    "initializeMotion.initiateGroup.initial",
)
_INITIALIZE_MOTION_MISSING = tuple(
    spec.key for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS
    if spec.key not in _INITIALIZE_MOTION_PARTIAL and spec.key != "initializeMotion.initializeMotors"
)


class _MutationPriorityRLock:
    """Reentrant mutex that admits queued mutations before later readers."""

    def __init__(self) -> None:
        self._condition = threading.Condition(threading.Lock())
        self._owner: int | None = None
        self._depth = 0
        self._mutation_waiters = 0

    def acquire(
        self,
        blocking: bool = True,
        timeout: float = -1,
        *,
        mutation: bool = False,
    ) -> bool:
        owner = threading.get_ident()
        deadline = None if timeout is None or timeout < 0 else time.monotonic() + timeout
        with self._condition:
            if self._owner == owner:
                self._depth += 1
                return True
            if mutation:
                self._mutation_waiters += 1
            try:
                while self._owner is not None or (not mutation and self._mutation_waiters > 0):
                    if not blocking:
                        return False
                    remaining = None if deadline is None else deadline - time.monotonic()
                    if remaining is not None and remaining <= 0:
                        return False
                    self._condition.wait(remaining)
                self._owner = owner
                self._depth = 1
                return True
            finally:
                if mutation:
                    self._mutation_waiters -= 1

    def release(self) -> None:
        owner = threading.get_ident()
        with self._condition:
            if self._owner != owner:
                raise RuntimeError("priority lock released by non-owner")
            self._depth -= 1
            if self._depth == 0:
                self._owner = None
                self._condition.notify_all()

    def __enter__(self) -> "_MutationPriorityRLock":
        self.acquire()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.release()

    @contextmanager
    def mutation(self):
        self.acquire(mutation=True)
        try:
            yield self
        finally:
            self.release()


_EVIDENCE_MAX_DEPTH = 8
_EVIDENCE_MAX_ITEMS = 128
_EVIDENCE_MAX_STRING = 512
_EVIDENCE_MAX_KEY = 96
_EVIDENCE_MAX_BYTES = 8192


def _json_safe(
    value: Any,
    *,
    max_depth: int = _EVIDENCE_MAX_DEPTH,
    max_items: int = _EVIDENCE_MAX_ITEMS,
    max_string: int = _EVIDENCE_MAX_STRING,
    max_key: int = _EVIDENCE_MAX_KEY,
    max_bytes: int = _EVIDENCE_MAX_BYTES,
) -> Any:
    """Return a deterministic, cycle-safe, strictly bounded JSON projection."""
    remaining = [max_items]
    active: set[int] = set()

    def omitted(reason: str, kind: str | None = None) -> dict[str, Any]:
        row: dict[str, Any] = {"omitted": reason}
        if kind:
            row["type"] = kind[:max_key]
        return row

    def key_text(key: Any) -> str:
        if type(key) is str:
            text = key
        elif key is None or type(key) in {bool, int}:
            text = str(key)
        elif type(key) is float and math.isfinite(key):
            text = str(key)
        else:
            text = f"<{type(key).__name__}>"
        if len(text) > max_key:
            text = text[: max_key - 14] + "...[truncated]"
        return text

    def visit(item: Any, depth: int) -> Any:
        if remaining[0] <= 0:
            return omitted("item_limit")
        remaining[0] -= 1
        if item is None or type(item) in {bool, int}:
            return item
        if type(item) is float:
            return item if math.isfinite(item) else omitted("non_finite_float")
        if type(item) is str:
            if len(item) <= max_string:
                return item
            return {
                "omitted": "string_limit",
                "prefix": item[:max_string],
                "original_length": len(item),
            }
        if depth >= max_depth:
            return omitted("depth_limit", type(item).__name__)
        if isinstance(item, (Mapping, list, tuple)):
            identity = id(item)
            if identity in active:
                return omitted("cycle", type(item).__name__)
            active.add(identity)
            try:
                if isinstance(item, Mapping):
                    pairs: list[tuple[str, Any]] = []
                    try:
                        iterator = item.items()
                    except Exception:
                        return omitted("mapping_items_error", type(item).__name__)
                    for raw_key, raw_value in iterator:
                        if remaining[0] <= 0:
                            pairs.append(("<omitted>", omitted("item_limit")))
                            break
                        pairs.append((key_text(raw_key), raw_value))
                        if len(pairs) >= max_items:
                            pairs.append(("<omitted>", omitted("mapping_width_limit")))
                            break
                    pairs.sort(key=lambda pair: pair[0])
                    output: dict[str, Any] = {}
                    for selected_key, raw_value in pairs:
                        unique = selected_key
                        suffix = 2
                        while unique in output:
                            unique = f"{selected_key}#{suffix}"
                            suffix += 1
                        output[unique] = visit(raw_value, depth + 1)
                    return output
                output_list = []
                for index, raw_value in enumerate(item):
                    if index >= max_items or remaining[0] <= 0:
                        output_list.append(omitted("sequence_width_limit"))
                        break
                    output_list.append(visit(raw_value, depth + 1))
                return output_list
            finally:
                active.remove(identity)
        return omitted("unsupported_type", type(item).__name__)

    try:
        projected = visit(value, 0)
    except Exception:
        return omitted("projection_error", type(value).__name__)
    try:
        encoded = json.dumps(projected, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    except Exception:
        return omitted("serialization_error", type(value).__name__)
    if len(encoded) > max_bytes:
        return {"omitted": "byte_limit", "encoded_bytes": len(encoded)}
    return projected


def _json_contract_safe(value: Any) -> Any:
    """Project a typed API value without inserting omission markers into its fields."""
    projected = _json_safe(
        value,
        max_depth=24,
        max_items=32_768,
        max_string=16_384,
        max_key=256,
        max_bytes=2_097_152,
    )

    def contains_omission(item: Any) -> bool:
        if isinstance(item, Mapping):
            if "omitted" in item:
                return True
            return any(contains_omission(entry) for entry in item.values())
        if isinstance(item, list):
            return any(contains_omission(entry) for entry in item)
        return False

    if contains_omission(projected):
        raise ValueError("typed JSON contract cannot contain omission markers")
    return projected


class Serial206ProductionPrimitiveAdapter:
    """Source-bound production primitives for both OEM initialization methods."""

    _MOTOR_METHODS = (
        "motor_oem_home_axis",
        "motor_set_axis_param",
        "motor_move_relative",
        "motor_wait_stopped",
        "motor_oem_wait_target_reached",
        "motor_get_position",
        "motor_set_home",
        "motor_move_absolute",
        "motor_oem_move_absolute",
        "oem_no24v_state",
        "motor_prepare_axis",
        "motor_oem_door_search_home",
        "motor_oem_open_thermal_door",
        "motor_thermal_door_status",
        "_motion_oem_axis_profile",
        "_machine_config_bundle",
        "chiller_gp_write",
    )

    _INITIALIZE_MOTION_PIPETTE_METHODS = (
        "query_all_pipette_tip_states",
        "eject_all_pipette_tips_for_oem_startup",
        "initiate_pipette_group_for_oem_initialize_motion",
        "checked_pipette_status_for_oem_initialize_motion",
    )

    _PIPETTE_TRANSPORT_METHODS = (
        "query_tip_status_all",
        "eject_all_tips_for_oem_startup",
        "initiate_group_once_for_oem_initialize_motion",
        "checked_pipette_status_for_oem_initialize_motion",
    )

    _MOVE_TO_ADAPTER_METHODS = (
        "oem_initialize_motion_scriptmove_to_waste",
        "oem_move_to",
        "oem_move_xy",
        "oem_move_z",
        "oem_move_axis_absolute",
    )

    _MOVE_TO_TESTER_METHODS = (
        "_motion_oem_axis_profile",
        "motor_get_position",
        "motor_set_axis_param",
        "motor_oem_move_absolute",
        "motor_oem_wait_targets_reached",
        "motor_oem_go_home",
        "motor_oem_move_z_home",
    )

    def __init__(
        self,
        tester: Any,
        pipette_transport: Any,
        *,
        authority_provider: Callable[[], Any],
        generation_provider: Callable[[], int],
        reference_store: Any | None = None,
    ) -> None:
        self.tester = tester
        self.pipette_transport = pipette_transport
        self.authority_provider = authority_provider
        self.generation_provider = generation_provider
        self.reference_store = reference_store
        self._last_tip_channels: list[int] | None = None
        self._pending_move_position_before: Any = None
        # These source methods intentionally mutate the active Z profile. Keep
        # their verified values bound to this process/board lifecycle so later
        # source commands validate the profile they actually inherit.
        self._z_profile_overrides: dict[int, int] = {}
        self._x_profile_overrides: dict[int, int] = {}
        self._x_lifecycle_executor: Callable[[str, Mapping[str, Any]], dict[str, Any]] | None = None

    def bind_x_lifecycle_executor(
        self,
        executor: Callable[[str, Mapping[str, Any]], dict[str, Any]],
    ) -> None:
        """Bind composite X effects to the durable provider lifecycle."""
        self._x_lifecycle_executor = executor

    def _execute_bound_x_intent(self, intent: str, values: Mapping[str, Any]) -> dict[str, Any]:
        if not callable(self._x_lifecycle_executor):
            return {"ok": False, "failure": "x_lifecycle_executor_not_bound", "intent": intent}
        payload = dict(values)
        payload.setdefault("expected_generation", int(self.generation_provider()))
        result = self._x_lifecycle_executor(intent, payload)
        return dict(result) if isinstance(result, Mapping) else {
            "ok": False,
            "failure": "x_lifecycle_executor_result_not_mapping",
            "intent": intent,
        }

    def capability_status(self) -> dict[str, Any]:
        blockers = [f"mandatory_primitive_not_bound:{name}" for name in self._MOTOR_METHODS if not callable(getattr(self.tester, name, None))]
        gripper_version: int | None = None
        calibrated: bool | None = None
        serial_number: int | None = None
        camera_calibrated: bool | None = None
        branch_authority: dict[str, Any] = {}
        try:
            machine = self.tester._machine_config_bundle()
            if not isinstance(machine, Mapping) or machine.get("ok") is not True:
                raise ValueError("immutable machine envelope unavailable")
            config = machine.get("config")
            if not isinstance(config, Mapping):
                raise ValueError("machine config envelope unavailable")
            core = config.get("config")
            calibration = config.get("calibration")
            if not isinstance(core, Mapping) or not isinstance(calibration, Mapping):
                raise ValueError("selected machine branch envelope unavailable")
            raw_version = core.get("GripperVersion")
            if type(raw_version) is bool or int(raw_version) not in {0, 1}:
                raise ValueError("GripperVersion must be exactly 0 or 1")
            gripper_version = int(raw_version)
            raw_calibrated = calibration.get("Calibrated")
            if raw_calibrated in {1, "1", True, "true", "True"}:
                calibrated = True
            elif raw_calibrated in {0, "0", False, "false", "False"}:
                calibrated = False
            else:
                raise ValueError("Calibrated must select an exact branch")
            parity = load_oem_parity_config(None)
            if parity.blockers:
                raise ValueError("frozen parity branch has blockers")
            raw_serial = parity.values.get("SerialNumber")
            if raw_serial is None or type(raw_serial) is bool:
                raise ValueError("SerialNumber bool is invalid")
            serial_number = int(raw_serial)
            raw_camera = parity.values.get("CameraCalibrated")
            if raw_camera in {1, "1", True, "true", "True"}:
                camera_calibrated = True
            elif raw_camera in {0, "0", False, "false", "False"}:
                camera_calibrated = False
            else:
                raise ValueError("CameraCalibrated must select an exact branch")
            branch_authority = {
                "serial_number": serial_number,
                "camera_calibrated": camera_calibrated,
                "machine_calibrated": calibrated,
                "gripper_version": gripper_version,
                "source": parity.calibration_source,
            }
        except Exception:
            blockers.append("frozen_serial206_machine_branch_not_bound")
        if serial_number is not None and serial_number != 206:
            blockers.append("serial206_serial_number_not_selected")
        if gripper_version is not None and gripper_version != 1:
            blockers.append("serial206_gripper_version_1_not_selected")
        if calibrated is not None and calibrated is not True:
            blockers.append("serial206_calibrated_1_branch_not_selected")
        motion_missing = [
            f"pipette_primitive_not_bound:{name}"
            for name in self._PIPETTE_TRANSPORT_METHODS
            if not callable(getattr(self.pipette_transport, name, None))
        ]
        motion_missing.extend(
            f"initializeMotion_primitive_not_bound:{name}"
            for name in self._INITIALIZE_MOTION_PIPETTE_METHODS
            if not callable(getattr(self, name, None))
        )
        move_to_missing = [
            f"moveTo_adapter_dependency_not_bound:{name}"
            for name in self._MOVE_TO_ADAPTER_METHODS
            if not callable(getattr(self, name, None))
        ]
        move_to_missing.extend(
            f"moveTo_tester_dependency_not_bound:{name}"
            for name in self._MOVE_TO_TESTER_METHODS
            if not callable(getattr(self.tester, name, None))
        )
        try:
            table = load_bound_oem_position_table()
            table_identity = {"source": table.source}
        except Exception:
            table_identity = None
            motion_missing.append("immutable_oem_position_table_not_bound")
        move_to_bound = table_identity is not None and not move_to_missing
        if not move_to_bound:
            motion_missing.append("oem_moveTo_branch_authority_not_bound")
        motors = not blockers
        initialize_motion_complete = motors and not motion_missing
        return {
            "initialize_motors_exact_primitives_bound": motors,
            "initialize_motors_binding_blockers": blockers,
            "selected_gripper_version": gripper_version,
            "selected_calibrated_branch": calibrated,
            "selected_serial_number": serial_number,
            "selected_camera_calibrated": camera_calibrated,
            "initialize_motors_branch_authority": branch_authority,
            "initialize_motion_complete": initialize_motion_complete,
            "initialize_motion_partial_primitives": list(_MOTION_STAGE_ORDER) if initialize_motion_complete else [],
            "initialize_motion_missing_primitives": motion_missing,
            "initialize_motion_moveTo_branch_authority": {
                "bound": move_to_bound,
                "adapter_dependencies": list(self._MOVE_TO_ADAPTER_METHODS),
                "tester_dependencies": list(self._MOVE_TO_TESTER_METHODS),
                "source_anchor": "ClassControlInterface.moveTo:4463-4620; ClassControlInterface.scriptmoveTo:3734-4014",
            },
            "position_table_identity": _json_safe(table_identity),
        }

    def prepare_for_initialize_motors(
        self,
        *,
        expected_generation: int,
        components: tuple[str, ...] = ("z",),
    ) -> dict[str, Any]:
        observed_generation = int(self.generation_provider())
        if expected_generation != observed_generation:
            return {
                "ok": False,
                "observed_generation": observed_generation,
                "physical_motion": False,
                "blocker": "ownership_generation_changed_before_preparation",
            }
        raw = prepare_motion_without_motion(
            self.tester,
            self.authority_provider(),
            components=tuple(components),
        )
        ok = isinstance(raw, Mapping) and raw.get("ok") is True and raw.get("physical_motion") is False
        return {
            "ok": ok,
            "observed_generation": observed_generation,
            "board_lifecycle_generation": raw.get("board_lifecycle_generation") if isinstance(raw, Mapping) else None,
            "board_preparation_verified": ok,
            "initialize_without_motion_verified": ok,
            "physical_motion": False,
            "motor_output_state": "unknown",
            "motor_torque_verified": False,
            "receipt": _json_safe(raw),
        }

    def current_board_lifecycle_generation(self) -> int | None:
        value = getattr(self.tester, "_oem_active_board_lifecycle_generation", None)
        return int(value) if type(value) is int else None

    def motor_set_home(self, *args: Any, **kwargs: Any) -> Any:
        return self.tester.motor_set_home(*args, **kwargs)

    def _x_profile(self) -> Mapping[str, Any]:
        profile = self.tester._motion_oem_axis_profile("x", startup=True)
        if not isinstance(profile, Mapping):
            raise RuntimeError("serial-206 X profile unavailable")
        if int(profile.get("board", -1)) != 5 or int(profile.get("motor", -1)) != 0:
            raise RuntimeError("serial-206 X channel identity mismatch")
        if int(profile.get("axis_min_steps", 0)) != 0 or int(profile.get("axis_max_steps", -1)) != 90263:
            raise RuntimeError("serial-206 X limit authority mismatch")
        return profile

    def _x_tmcl_success(self, ack: Any) -> bool:
        verifier = getattr(self.tester, "_tmcl_success", None)
        if callable(verifier):
            try:
                return verifier(ack) is True
            except Exception:
                return False
        return isinstance(ack, Mapping) and ack.get("status") == 100

    def _x_readback_verified(self, row: Any, expected: int | None = None) -> bool:
        value = self._x_value(row)
        return bool(
            isinstance(row, Mapping)
            and row.get("ok", True) is True
            and self._x_tmcl_success(row.get("ack"))
            and value is not None
            and (expected is None or value == int(expected))
        )

    def _x_require_motion_preflight(self) -> dict[str, Any]:
        profile = dict(self._x_profile())
        require_profile = getattr(self.tester, "motor_oem_require_no_motion_profile", None)
        profile_receipt = require_profile("x", expected_overrides=dict(self._x_profile_overrides)) if callable(require_profile) else None
        masks = {
            param: self.tester.motor_get_axis_param(5, param, motor=0)
            for param in (12, 13)
        }
        if any(
            not self._x_readback_verified(masks[param], expected)
            for param, expected in SERIAL206_X_SWITCH_MASKS.items()
        ):
            raise RuntimeError("serial-206 X switch-mask preflight mismatch")
        return {
            "profile": profile,
            "profile_receipt": _json_safe(profile_receipt),
            "switch_masks": _json_safe(masks),
            "expected_switch_masks": dict(SERIAL206_X_SWITCH_MASKS),
        }

    def _x_profile_parameter(
        self,
        *,
        parameter: int,
        value: int,
        intent: str,
        source_method: str,
    ) -> dict[str, Any]:
        self._x_require_motion_preflight()
        write = self.tester.motor_set_axis_param(5, int(parameter), int(value), motor=0)
        readback = self.tester.motor_get_axis_param(5, int(parameter), motor=0)
        acknowledged = bool(
            isinstance(write, Mapping)
            and write.get("ok") is True
            and self._x_tmcl_success(write.get("ack"))
        )
        verified = acknowledged and self._x_readback_verified(readback, int(value))
        if verified:
            defaults = {4: 1700, 5: 350, 6: 31, 205: 16}
            if defaults.get(int(parameter)) == int(value):
                self._x_profile_overrides.pop(int(parameter), None)
            else:
                self._x_profile_overrides[int(parameter)] = int(value)
        return {
            "ok": verified,
            "axis": "x",
            "intent": intent,
            "source_method": source_method,
            "board": 5,
            "motor": 0,
            "parameter": int(parameter),
            "value": int(value),
            "write": _json_safe(write),
            "readback": _json_safe(readback),
            "controller_command_acknowledged": acknowledged,
            "controller_terminal_state_verified": verified,
            "physical_motion_commanded": False,
            "physical_effect_verified": False,
            "failure": None if verified else "x_profile_parameter_readback_mismatch",
        }

    def x_set_max_speed(self, value: int = 0) -> dict[str, Any]:
        return self._x_profile_parameter(
            parameter=4,
            value=1700 if int(value) == 0 else int(value),
            intent="set_max_speed",
            source_method="ClassControlInterface.setMaxSpeed(x)",
        )

    def x_set_max_acc(self, value: int = 0) -> dict[str, Any]:
        return self._x_profile_parameter(
            parameter=5,
            value=350 if int(value) == 0 else int(value),
            intent="set_max_acc",
            source_method="ClassControlInterface.setMaxAcc(x)",
        )

    def x_restore_original_speed(self) -> dict[str, Any]:
        return self._x_profile_parameter(
            parameter=4,
            value=1700,
            intent="restore_original_speed",
            source_method="ClassControlInterface.restoreOriginalXSpeed",
        )

    def x_set_stall_guard(self, value: int = 0) -> dict[str, Any]:
        return self._x_profile_parameter(
            parameter=205,
            value=16 if int(value) == 0 else int(value),
            intent="set_stall_guard",
            source_method="ClassControlInterface.setStallGuard(x)",
        )

    def x_terminal_status(self) -> dict[str, Any]:
        rows = {
            param: self.tester.motor_get_axis_param(5, param, motor=0)
            for param in (1, 3, 4, 5, 6, 9, 10, 12, 13, 205)
        }
        values = {param: self._x_value(row) for param, row in rows.items()}
        verified = all(self._x_readback_verified(row) for row in rows.values())
        mask_verified = verified and all(
            values[param] == expected for param, expected in SERIAL206_X_SWITCH_MASKS.items()
        )
        expected_profile = {4: 1700, 5: 350, 6: 31, 205: 16}
        expected_profile.update(self._x_profile_overrides)
        profile_verified = verified and all(values[param] == expected for param, expected in expected_profile.items())
        typed_speed_zero = type(values[3]) is int and values[3] == 0
        failure = None
        if not typed_speed_zero:
            failure = "x_terminal_speed_not_typed_integer_zero"
        elif not verified:
            failure = "x_terminal_readback_not_verified"
        return {
            "ok": verified and typed_speed_zero,
            "axis": "x",
            "board": 5,
            "motor": 0,
            "position_steps": values[1],
            "speed_steps_s": values[3],
            "max_speed": values[4],
            "max_acceleration": values[5],
            "max_current": values[6],
            "left_switch_state": values[9],
            "right_switch_state": values[10],
            "right_switch_disabled": None if values[12] is None else values[12] != 0,
            "left_switch_disabled": None if values[13] is None else values[13] != 0,
            "stall_guard": values[205],
            "profile_verified": profile_verified,
            "expected_profile": expected_profile,
            "switch_mask_verified": mask_verified,
            "switch_mask_tuple": {param: values[param] for param in (12, 13)},
            "expected_switch_masks": dict(SERIAL206_X_SWITCH_MASKS),
            "readbacks": rows,
            "authority": "serial206_x_terminal_register_readback",
            "failure": failure,
        }

    def x_reconcile_switch_masks(self) -> dict[str, Any]:
        before = {param: self.tester.motor_get_axis_param(5, param, motor=0) for param in (12, 13)}
        write = self.tester.motor_set_axis_param(5, 12, SERIAL206_X_SWITCH_MASKS[12], motor=0)
        after = {param: self.tester.motor_get_axis_param(5, param, motor=0) for param in (12, 13)}
        acknowledged = bool(
            isinstance(write, Mapping)
            and write.get("ok") is True
            and self._x_tmcl_success(write.get("ack"))
        )
        verified = acknowledged and all(
            self._x_readback_verified(after[param], expected)
            for param, expected in SERIAL206_X_SWITCH_MASKS.items()
        )
        if verified:
            self._x_profile_overrides.clear()
        invalidation = self._x_desync(
            "Serial-206 X switch-mask reconciliation invalidated prior authority.",
            "switch_mask_reconciliation",
        ) if self.reference_store is not None else None
        return {
            "ok": verified,
            "axis": "x",
            "intent": "reconcile_switch_masks",
            "source_exact": False,
            "adaptation": "serial206_machine_safety_adaptation",
            "classification": "serial206_machine_safety_adaptation",
            "writes": {"12": _json_safe(write)},
            "before": _json_safe(before),
            "after": _json_safe(after),
            "machine_bound_expected": dict(SERIAL206_X_SWITCH_MASKS),
            "switch_mask_tuple": {param: self._x_value(after[param]) for param in (12, 13)},
            "preparation_invalidated": True,
            "reference_invalidated": bool(
                self.reference_store is None
                or (isinstance(invalidation, Mapping) and invalidation.get("ok") is True)
            ),
            "reference_invalidation": _json_safe(invalidation),
            "controller_command_acknowledged": acknowledged,
            "controller_terminal_state_verified": verified,
            "physical_motion_commanded": False,
            "physical_effect_verified": False,
            "failure": None if verified else "x_switch_mask_reconciliation_failed",
        }

    def _x_event_fresh(self, event: Any, event_window: Any) -> bool:
        if not isinstance(event, Mapping):
            return False
        checker = getattr(self.tester, "_event_received_after_dispatch", None)
        if callable(checker):
            return checker(event, event_window) is True
        return True

    @staticmethod
    def _x_value(row: Any) -> int | None:
        if not isinstance(row, Mapping):
            return None
        value = row.get("position", row.get("value"))
        return int(value) if type(value) is int else None

    def _reference_snapshot(self, axes: tuple[str, ...], operation: str) -> dict[str, Any]:
        if self.reference_store is None:
            return {"ok": True, "durable_clean": True, "authority_untrusted": False, "rows": {}}
        snapshot = self.reference_store.snapshot(axes)
        if not isinstance(snapshot, Mapping) or snapshot.get("ok") is not True or snapshot.get("durable_clean") is not True:
            raise RuntimeError(f"durable reference authority unavailable for {operation}")
        rows = snapshot.get("rows")
        missing = [axis for axis in axes if not isinstance(rows, Mapping) or not isinstance(rows.get(axis), Mapping) or rows[axis].get("state") != "referenced"]
        if missing:
            raise RuntimeError(f"referenced axes required for {operation}: {','.join(missing)}")
        return dict(snapshot)

    def _x_desync(self, reason: str, motion_kind: str = "absolute") -> dict[str, Any]:
        if self.reference_store is None:
            return {"ok": False, "error": "reference store not bound"}
        result = self.reference_store.mark_desynced(MarkAxisDesyncedCommand(axis="x", reason=reason, source="serial206.x", motion_kind=motion_kind))
        if result.get("ok") is True:
            return result
        return {"ok": False, "desync": result, "recovery": self.reference_store.recover_untrusted_authority(f"X desynchronization failed: {reason}")}

    def _x_issue_absolute(self, requested: int, *, source_mode: str, clamp_low: int = 0, acceleration: int | None = None, event_window: Any = None) -> dict[str, Any]:
        preflight = self._x_require_motion_preflight()
        before = self.tester.motor_get_position(5, motor=0)
        before_value = self._x_value(before)
        if not self._x_readback_verified(before):
            return {"ok": False, "failure": "x_position_before_unavailable", "command_issued": False, "before": _json_safe(before)}
        target = min(90263, max(int(clamp_low), int(requested)))
        result: dict[str, Any] = {"ok": False, "axis": "x", "source_mode": source_mode, "requested_position_steps": int(requested), "target_position_steps": target, "before": _json_safe(before), "before_position_steps": before_value, "preflight": _json_safe(preflight), "command_issued": False, "source_noop": False, "physical_motion_commanded": False, "controller_command_acknowledged": False}
        if acceleration is not None:
            setup = self.tester.motor_set_axis_param(5, 5, int(acceleration), motor=0)
            result["acceleration_set"] = _json_safe(setup)
            if not isinstance(setup, Mapping) or setup.get("ok") is not True or not isinstance(setup.get("readback"), Mapping) or setup["readback"].get("value") != int(acceleration):
                result["failure"] = "x_acceleration_write_or_readback_failed"
                return result
        assert before_value is not None
        current_position = before_value
        noop_reason = None
        if current_position == target:
            noop_reason = "already_at_effective_target"
        elif abs(90263 - current_position) < 10 and target > current_position:
            noop_reason = "high_limit_guard"
        if noop_reason is not None:
            after = self.tester.motor_get_position(5, motor=0)
            speed = self.tester.motor_get_speed(5, motor=0)
            after_value = self._x_value(after)
            speed_value = speed.get("speed") if isinstance(speed, Mapping) else None
            terminal = bool(
                self._x_readback_verified(after, int(before_value))
                and isinstance(speed, Mapping)
                and speed.get("ok") is True
                and self._x_tmcl_success(speed.get("ack"))
                and type(speed_value) is int
                and speed_value == 0
            )
            result.update({
                "ok": terminal,
                "source_noop": True,
                "noop_reason": noop_reason,
                "target_position_steps": after_value,
                "after": _json_safe(after),
                "after_position_steps": after_value,
                "terminal_speed": _json_safe(speed),
                "command_issued": False,
                "physical_motion_commanded": False,
                "controller_command_acknowledged": False,
                "controller_terminal_state_verified": terminal,
                "target_event_128_observed": False,
                "target_position_verified": terminal,
                "physical_effect_verified": False,
                "failure": None if terminal else "x_noop_terminal_state_unverified",
            })
            return result
        result["event_window"] = event_window if event_window is not None else self.tester.begin_bus_event_window()
        move = self.tester.motor_oem_move_absolute(5, target, motor=0, wait_for_stop=False, max_position=90263)
        result["event_window"] = _json_safe(move.get("event_window")) if isinstance(move, Mapping) and isinstance(move.get("event_window"), Mapping) else result["event_window"]
        if isinstance(move, Mapping) and move.get("ok") is True and move.get("source_noop") is True:
            after = self.tester.motor_get_position(5, motor=0)
            speed = self.tester.motor_get_speed(5, motor=0)
            after_value = self._x_value(after)
            speed_value = speed.get("speed") if isinstance(speed, Mapping) else None
            terminal = bool(
                self._x_readback_verified(after, after_value)
                and isinstance(speed, Mapping)
                and speed.get("ok") is True
                and self._x_tmcl_success(speed.get("ack"))
                and type(speed_value) is int
                and speed_value == 0
            )
            result.update({
                "ok": terminal,
                "source_noop": True,
                "noop_reason": str(move.get("short_circuit") or "transport_source_noop"),
                "move": _json_safe(move),
                "target_position_steps": after_value,
                "after": _json_safe(after),
                "after_position_steps": after_value,
                "terminal_speed": _json_safe(speed),
                "command_issued": False,
                "physical_motion_commanded": False,
                "controller_command_acknowledged": False,
                "controller_terminal_state_verified": terminal,
                "target_event_128_observed": False,
                "target_position_verified": terminal,
                "physical_effect_verified": False,
                "failure": None if terminal else "x_transport_noop_terminal_state_unverified",
            })
            return result
        command_acknowledged = bool(
            isinstance(move, Mapping)
            and move.get("ok") is True
            and self._x_tmcl_success(move.get("ack"))
        )
        result.update({"move": _json_safe(move), "command_issued": True, "physical_motion_commanded": True, "controller_command_acknowledged": command_acknowledged, "ok": command_acknowledged})
        if result["ok"] is not True:
            result["failure"] = "x_move_command_not_acknowledged"
        return result

    def _x_finalize(self, ticket: Mapping[str, Any], *, timeout_s: float, motion_kind: str, publish: bool = True) -> dict[str, Any]:
        result = dict(ticket)
        if result.get("source_noop") is True:
            result["ok"] = bool(result.get("ok") is True and result.get("after_position_steps") == result.get("target_position_steps"))
            return result
        if result.get("command_issued") is not True:
            return result
        if result.get("ok") is not True:
            result["physical_effect_ambiguous"] = True
        else:
            wait_fn = getattr(self.tester, "motor_wait_target_reached", None) or getattr(self.tester, "motor_oem_wait_target_reached")
            wait = wait_fn(5, motor=0, timeout_s=float(timeout_s), event_window=result.get("event_window"))
            events = self.tester.collect_bus_events(duration_s=0.30, timeout_ms=12, max_events=96)
            after = self.tester.motor_get_position(5, motor=0)
            speed = self.tester.motor_get_speed(5, motor=0)
            after_value = self._x_value(after)
            event_window = result.get("event_window")
            after_sequence = event_window.get("after_sequence") if isinstance(event_window, Mapping) else None
            addressed_events = [
                event for event in events
                if isinstance(event, Mapping)
                and event.get("board") == 5
                and event.get("motor") == 0
                and type(event.get("event_sequence")) is int
                and type(after_sequence) is int
                and event["event_sequence"] > after_sequence
                and self._x_event_fresh(event, event_window)
            ]
            addressed_errors = [event for event in addressed_events if event.get("status") in {13, 14, 130}]
            target_events = [event for event in addressed_events if event.get("status") == 128]
            terminal_event = bool(target_events)
            position_verified = self._x_readback_verified(after, int(result["target_position_steps"]))
            speed_zero = bool(
                isinstance(speed, Mapping)
                and speed.get("ok") is True
                and self._x_tmcl_success(speed.get("ack"))
                and type(speed.get("speed")) is int
                and speed.get("speed") == 0
            )
            wait_ok = bool(
                isinstance(wait, Mapping)
                and wait.get("ok") is True
                and wait.get("target_reached") is True
            )
            failure = None
            if not wait_ok:
                failure = str(
                    wait.get("failure")
                    if isinstance(wait, Mapping) and wait.get("failure")
                    else "x_target_wait_not_verified"
                )
            elif not speed_zero:
                failure = "x_terminal_zero_speed_not_verified"
            elif addressed_errors:
                failure = "x_controller_error_event"
            elif not terminal_event:
                failure = "x_target_event_128_missing_or_stale"
            elif not position_verified:
                failure = "x_target_position_not_verified"
            result.update({"wait": _json_safe(wait), "wait_verified": wait_ok, "events": _json_safe(events), "after": _json_safe(after), "after_position_steps": after_value, "terminal_speed": _json_safe(speed), "controller_error_events": _json_safe(addressed_errors), "target_events": _json_safe(target_events), "target_event_128_verified": terminal_event, "target_event_128_observed": terminal_event, "target_position_verified": position_verified, "controller_terminal_state_verified": failure is None})
            result["ok"] = bool(result.get("ok") is True and result["controller_terminal_state_verified"])
            if not result["ok"]:
                result["failure"] = str(result.get("failure") or failure or "x_absolute_terminal_evidence_not_accepted")
        if result.get("ok") is True and result.get("command_issued") is True and publish and self.reference_store is not None:
            metadata = self.reference_store.record_motion("x", motion_kind)
            result["reference_state"] = _json_safe(metadata)
            if metadata.get("ok") is not True:
                result["ok"] = False
                result["failure"] = "x_motion_reference_metadata_not_verified"
                result["reference_desync"] = _json_safe(self._x_desync(result["failure"], motion_kind))
        elif result.get("ok") is not True and result.get("command_issued") is True:
            stop = self.x_stop(timeout_s=3.0)
            result["safety_stop"] = _json_safe(stop)
            result["reference_desync"] = _json_safe(self._x_desync(str(result.get("failure") or "X motion ambiguous"), motion_kind)) if self.reference_store is not None else None
        return result

    def x_move_absolute(self, *, position_steps: int, acceleration: int | None = None, wait_for_stop: bool = True, wait_timeout_s: float = 20.0, source_mode: str = "ClassControlInterface.moveX", clamp_low_to_60: bool = True, publish_motion_metadata: bool = True) -> dict[str, Any]:
        reference = self._reference_snapshot(("x",), source_mode)
        result: dict[str, Any] = {
            "ok": False,
            "axis": "x",
            "failure": "x_absolute_not_executed",
            "command_issued": False,
        }
        try:
            ticket = self._x_issue_absolute(int(position_steps), source_mode=source_mode, clamp_low=60 if clamp_low_to_60 else 0, acceleration=acceleration)
            if not wait_for_stop and ticket.get("command_issued") is True:
                ticket.update({"pending_motion": True, "physical_motion": True, "reference_before": _json_safe(reference)})
                result = ticket
            else:
                result = self._x_finalize(ticket, timeout_s=wait_timeout_s, motion_kind="absolute", publish=publish_motion_metadata)
        finally:
            if acceleration is not None:
                restore = self.tester.motor_set_axis_param(5, 5, 350, motor=0)
                restore_ok = bool(
                    isinstance(restore, Mapping)
                    and restore.get("ok") is True
                    and self._x_tmcl_success(restore.get("ack"))
                    and isinstance(restore.get("readback"), Mapping)
                    and self._x_readback_verified(restore["readback"], 350)
                )
                result["acceleration_restore"] = _json_safe(restore)
                result["acceleration_restore_verified"] = restore_ok
                if not restore_ok:
                    result["ok"] = False
                    result["failure"] = result.get("failure") or "x_acceleration_restore_failed"
        result["reference_before"] = _json_safe(reference)
        result["physical_motion"] = bool(result.get("command_issued"))
        return result

    def x_move_steps(self, *, steps: int, wait_timeout_s: float = 20.0) -> dict[str, Any]:
        reference = self._reference_snapshot(("x",), "ClassControlInterface.moveSteps(x)")
        preflight = self._x_require_motion_preflight()
        before = self.tester.motor_get_position(5, motor=0)
        before_value = self._x_value(before)
        if not self._x_readback_verified(before):
            return {"ok": False, "failure": "x_position_before_unavailable", "physical_motion_commanded": False, "before": _json_safe(before)}
        assert before_value is not None
        target = before_value + int(steps)
        if int(steps) == 0:
            speed = self.tester.motor_get_speed(5, motor=0)
            after = self.tester.motor_get_position(5, motor=0)
            terminal = bool(
                self._x_readback_verified(after, target)
                and isinstance(speed, Mapping)
                and speed.get("ok") is True
                and self._x_tmcl_success(speed.get("ack"))
                and type(speed.get("speed")) is int
                and speed.get("speed") == 0
            )
            return {"ok": terminal, "axis": "x", "intent": "move_steps", "source_noop": True, "noop_reason": "zero_steps", "requested_steps": 0, "before_position_steps": before_value, "target_position_steps": target, "after_position_steps": self._x_value(after), "before": _json_safe(before), "after": _json_safe(after), "terminal_speed": _json_safe(speed), "preflight": _json_safe(preflight), "command_issued": False, "target_event_128_observed": False, "physical_motion_commanded": False, "controller_command_acknowledged": False, "controller_terminal_state_verified": terminal, "physical_effect_verified": False, "reference_before": _json_safe(reference), "failure": None if terminal else "x_noop_terminal_state_unverified"}
        if target < 20 or target > 90263 - 20:
            return {"ok": False, "failure": "x_source_coordinate_inside_limit_margin", "command_issued": False, "before_position_steps": before_value, "requested_steps": int(steps), "target_position_steps": target, "source_min_steps": 0, "source_max_steps": 90263, "source_limit_margin_steps": 20, "physical_motion_commanded": False}
        event_window = self.tester.begin_bus_event_window()
        move = self.tester.motor_move_relative(5, int(steps), motor=0)
        acknowledged = bool(isinstance(move, Mapping) and move.get("ok") is True and self._x_tmcl_success(move.get("ack")))
        event_window = self.tester._bind_event_dispatch_cursor(event_window, 5, 0, move.get("ack")) if hasattr(self.tester, "_bind_event_dispatch_cursor") else event_window
        ticket = {"ok": acknowledged, "axis": "x", "source_mode": "ClassControlInterface.moveSteps", "requested_steps": int(steps), "target_position_steps": target, "before": _json_safe(before), "before_position_steps": before_value, "preflight": _json_safe(preflight), "event_window": event_window, "move": _json_safe(move), "command_issued": True, "physical_motion_commanded": True, "controller_command_acknowledged": acknowledged, "failure": None if acknowledged else "x_relative_command_ack_failed"}
        result = self._x_finalize(ticket, timeout_s=float(wait_timeout_s), motion_kind="relative", publish=True)
        result.update({"intent": "move_steps", "reference_before": _json_safe(reference), "physical_motion": True})
        return result

    def _move_xy_y_issue_absolute(self, requested: int, *, event_window: Any) -> dict[str, Any]:
        before = self.tester.motor_get_position(4, motor=0)
        before_value = self._x_value(before)
        target = min(102956, max(0, int(requested)))
        if not self._x_readback_verified(before):
            return {"ok": False, "failure": "y_position_before_unavailable", "command_issued": False}
        if before_value == target:
            return {"ok": True, "axis": "y", "source_noop": True, "command_issued": False, "target_position_steps": target, "before_position_steps": before_value, "after_position_steps": before_value}
        move = self.tester.motor_oem_move_absolute(4, target, motor=0, wait_for_stop=False, max_position=102956)
        acknowledged = bool(
            isinstance(move, Mapping)
            and move.get("ok") is True
            and self._x_tmcl_success(move.get("ack"))
        )
        bound_window = move.get("event_window") if isinstance(move, Mapping) and isinstance(move.get("event_window"), Mapping) else None
        if bound_window is None:
            bound_window = self.tester._bind_event_dispatch_cursor(event_window, 4, 0, move.get("ack")) if hasattr(self.tester, "_bind_event_dispatch_cursor") else event_window
        return {"ok": acknowledged, "axis": "y", "target_position_steps": target, "before_position_steps": before_value, "before": _json_safe(before), "event_window": _json_safe(bound_window), "move": _json_safe(move), "command_issued": True, "physical_motion_commanded": True, "controller_command_acknowledged": acknowledged, "failure": None if acknowledged else "y_move_command_not_acknowledged"}

    def _move_xy_y_finalize(self, ticket: Mapping[str, Any], *, timeout_s: float) -> dict[str, Any]:
        result = dict(ticket)
        if result.get("source_noop") is True:
            return result
        wait_fn = getattr(self.tester, "motor_wait_target_reached", None) or getattr(self.tester, "motor_oem_wait_target_reached")
        wait = wait_fn(4, motor=0, timeout_s=float(timeout_s), event_window=result.get("event_window"))
        events = self.tester.collect_bus_events(duration_s=0.30, timeout_ms=12, max_events=96)
        after = self.tester.motor_get_position(4, motor=0)
        speed = self.tester.motor_get_speed(4, motor=0)
        after_value = self._x_value(after)
        window = result.get("event_window")
        sequence = window.get("after_sequence") if isinstance(window, Mapping) else None
        addressed = [row for row in events if isinstance(row, Mapping) and row.get("board") == 4 and row.get("motor") == 0 and type(row.get("event_sequence")) is int and type(sequence) is int and row["event_sequence"] > sequence and self._x_event_fresh(row, window)]
        errors = [row for row in addressed if row.get("status") in {13, 14, 130}]
        target_events = [row for row in addressed if row.get("status") == 128]
        speed_zero = bool(isinstance(speed, Mapping) and speed.get("ok") is True and self._x_tmcl_success(speed.get("ack")) and type(speed.get("speed")) is int and speed.get("speed") == 0)
        position_verified = self._x_readback_verified(after, int(result["target_position_steps"]))
        failure = None
        if result.get("controller_command_acknowledged") is not True:
            failure = "y_move_command_not_acknowledged"
        elif errors:
            failure = "y_controller_error_event"
        elif not target_events:
            failure = "y_target_event_128_missing_or_stale"
        elif not position_verified:
            failure = "y_target_position_not_verified"
        elif not speed_zero:
            failure = "y_terminal_zero_speed_not_verified"
        result.update({"wait": _json_safe(wait), "events": _json_safe(events), "controller_error_events": _json_safe(errors), "target_events": _json_safe(target_events), "target_event_128_observed": bool(target_events), "after": _json_safe(after), "after_position_steps": after_value, "terminal_speed": _json_safe(speed), "target_position_verified": position_verified, "controller_terminal_state_verified": failure is None, "failure": failure})
        result["ok"] = failure is None
        return result

    def _finalize_move_xy_receipt(self, receipt: dict[str, Any], *, commands: Mapping[str, Any], waits: Mapping[str, Any], after: Mapping[str, int], restore: Mapping[str, Any], required_axes: tuple[str, ...]) -> dict[str, Any]:
        axis_address = {"x": (5, 0), "y": (4, 0)}
        events = self.tester.collect_bus_events(duration_s=0.30, timeout_ms=12, max_events=128) if required_axes else []
        evidence: dict[str, Any] = {}
        fresh_after: dict[str, int] = dict(after)
        for axis in required_axes:
            command = commands.get(axis)
            moved = isinstance(command, Mapping) and command.get("command_issued") is True
            board, motor = axis_address[axis]
            position = self.tester.motor_get_position(board, motor=motor)
            speed = self.tester.motor_get_speed(board, motor=motor)
            position_value = self._x_value(position)
            speed_value = speed.get("speed") if isinstance(speed, Mapping) else None
            if type(position_value) is int:
                fresh_after[axis] = position_value
            window = command.get("event_window") if isinstance(command, Mapping) else None
            sequence = window.get("after_sequence") if isinstance(window, Mapping) else None
            addressed = [row for row in events if isinstance(row, Mapping) and row.get("board") == board and row.get("motor") == motor and type(row.get("event_sequence")) is int and type(sequence) is int and row["event_sequence"] > sequence and self._x_event_fresh(row, window)]
            errors = [row for row in addressed if row.get("status") in {13, 14, 130}]
            targets = [row for row in addressed if row.get("status") == 128]
            command_ok = bool(
                isinstance(command, Mapping)
                and command.get("ok") is True
                and (not moved or command.get("controller_command_acknowledged") is True)
            )
            wait = waits.get(axis)
            wait_ok = bool(isinstance(wait, Mapping) and wait.get("ok") is True and wait.get("target_reached") is True)
            position_ok = self._x_readback_verified(position, receipt["requested"][axis])
            speed_ok = bool(isinstance(speed, Mapping) and speed.get("ok") is True and self._x_tmcl_success(speed.get("ack")) and type(speed_value) is int and speed_value == 0)
            event_ok = bool(not moved or (targets and not errors))
            evidence[axis] = {
                "command_acknowledged": command_ok,
                "wait_accepted": wait_ok,
                "target_event_128_observed": bool(targets) if moved else False,
                "controller_error_events": _json_safe(errors),
                "position": _json_safe(position),
                "position_verified": position_ok,
                "terminal_speed": _json_safe(speed),
                "terminal_speed_verified": speed_ok,
                "ok": bool(command_ok and wait_ok and event_ok and position_ok and speed_ok),
            }
        command_ok = all(evidence[axis]["command_acknowledged"] for axis in required_axes)
        terminal_ok = all(evidence[axis]["ok"] for axis in required_axes)
        target_ok = all(evidence[axis]["position_verified"] for axis in required_axes)
        restore_ok = all(isinstance(restore.get(axis), Mapping) and restore[axis].get("ok") is True and isinstance(restore[axis].get("readback"), Mapping) and restore[axis]["readback"].get("value") == (350 if axis == "x" else 400) for axis in ("x", "y"))
        ok = bool(command_ok and terminal_ok and target_ok and restore_ok)
        receipt.update({"commands": _json_safe(commands), "waits": _json_safe(waits), "events": _json_safe(events), "axis_evidence": _json_safe(evidence), "after": _json_safe(fresh_after), "acceleration_restore": _json_safe(restore), "controller_command_acknowledged": command_ok, "controller_terminal_state_verified": terminal_ok, "target_position_verified": target_ok, "acceleration_restore_verified": restore_ok, "ok": ok})
        moved_axes = tuple(axis for axis in required_axes if isinstance(commands.get(axis), Mapping) and commands[axis].get("command_issued") is True)
        receipt["moved_axes"] = list(moved_axes)
        if ok and self.reference_store is not None and moved_axes:
            if len(moved_axes) == 1:
                metadata = self.reference_store.record_motion(moved_axes[0], "move_xy")
            else:
                metadata = self.reference_store.record_motion_many(
                    tuple((axis, "move_xy") for axis in moved_axes)
                )
            receipt["reference_state"] = _json_safe(metadata)
            if metadata.get("ok") is not True:
                ok = False
                receipt.update({"ok": False, "failure": "moveXY_reference_metadata_not_verified", "reference_desync": _json_safe(self.reference_store.mark_desynced_many((MarkAxisDesyncedCommand(axis="x", reason="moveXY metadata failure", source="serial206.move_xy", motion_kind="move_xy"), MarkAxisDesyncedCommand(axis="y", reason="moveXY metadata failure", source="serial206.move_xy", motion_kind="move_xy"))))})
        delivered = any(isinstance(row, Mapping) and row.get("command_issued") is True for row in commands.values())
        if not ok and delivered:
            receipt.setdefault("failure", "moveXY_terminal_evidence_not_accepted")
            stops = {axis: self.tester.motor_oem_stop_exact(5 if axis == "x" else 4, motor=0) for axis in ("x", "y")}
            receipt["safety_stop"] = _json_safe(stops)
            if self.reference_store is not None:
                receipt["reference_desync"] = _json_safe(self.reference_store.mark_desynced_many(tuple(MarkAxisDesyncedCommand(axis=axis, reason=str(receipt["failure"]), source="serial206.move_xy", motion_kind="move_xy") for axis in ("x", "y"))))
        return receipt


    def x_stop(self, *, timeout_s: float = 3.0) -> dict[str, Any]:
        stop = self.tester.motor_oem_stop_exact(5, motor=0)
        wait = self.tester.motor_wait_stopped(5, motor=0, timeout_s=float(timeout_s), require_seen_nonzero=False)
        acknowledged = bool(
            isinstance(stop, Mapping)
            and stop.get("ok") is True
            and (
                self._x_tmcl_success(stop.get("first_delivery"))
                and self._x_tmcl_success(stop.get("second_delivery"))
                if stop.get("first_delivery") is not None or stop.get("second_delivery") is not None
                else self._x_tmcl_success(stop.get("ack"))
            )
        )
        speed = wait.get("last_speed") if isinstance(wait, Mapping) else None
        speed_ack = wait.get("last_ack") if isinstance(wait, Mapping) else None
        terminal = bool(
            isinstance(wait, Mapping)
            and wait.get("stopped") is True
            and type(speed) is int
            and speed == 0
            and self._x_tmcl_success(speed_ack)
        )
        return {"ok": acknowledged and terminal, "axis": "x", "intent": "stop", "stop": _json_safe(stop), "wait": _json_safe(wait), "controller_command_acknowledged": acknowledged, "controller_terminal_state_verified": terminal, "physical_motion": False, "physical_effect_verified": False, "failure": None if acknowledged and terminal else "x_stop_not_verified"}

    def x_abort(self, *, reason: str = "forceAbortMotion") -> dict[str, Any]:
        abort = self.tester.motor_oem_force_abort_motion(reason=reason)
        stop = self.x_stop(timeout_s=3.0)
        desync = self._x_desync(reason, "abort") if self.reference_store is not None else None
        ok = bool(isinstance(abort, Mapping) and abort.get("ok") is True and stop.get("ok") is True)
        return {"ok": ok, "axis_context": "x", "intent": "aggregate_oem_abort", "physical_scope": "aggregate_oem_all_present_boards", "x_only": False, "logical_abort": _json_safe(abort), "x_terminal_stop": _json_safe(stop), "reference_desync": _json_safe(desync), "controller_command_acknowledged": stop.get("controller_command_acknowledged") is True, "controller_terminal_state_verified": stop.get("controller_terminal_state_verified") is True, "physical_effect_verified": False, "failure": None if ok else "aggregate_oem_abort_not_verified"}

    def prepare_x(self, *, expected_generation: int) -> dict[str, Any]:
        result = self.prepare_for_initialize_motors(expected_generation=int(expected_generation), components=("x",))
        result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "x_prepare_result_not_mapping"}
        if result.get("ok") is True:
            self._x_profile_overrides.clear()
        result.update({"axis": "x", "physical_motion": False, "source_anchor": "ClassControlInterface.initializeMotorsWithoutMotion:3187-3195", "source_exact": True, "literal_switch_mask_writes": []})
        return result

    def x_set_home(self) -> dict[str, Any]:
        """Recovery-only no-motion ClassMotor.setHome for X."""
        before = self.tester.motor_get_position(5, motor=0)
        if not self._x_readback_verified(before):
            return {
                "ok": False,
                "intent": "set_home",
                "failure": "x_set_home_position_before_unavailable",
                "physical_motion_commanded": False,
            }
        set_home = self.tester.motor_set_home(5, motor=0)
        after = self.tester.motor_get_position(5, motor=0)
        acknowledged = bool(
            isinstance(set_home, Mapping)
            and set_home.get("ok") is True
            and (
                self._x_tmcl_success(set_home.get("ack"))
                or set_home.get("readback_verified") is True
            )
        )
        terminal = bool(acknowledged and self._x_readback_verified(after, 0))
        return {
            "ok": terminal,
            "axis": "x",
            "intent": "set_home",
            "source_method": "ClassMotor.setHome (SAP1=0); recovery-only; no motion",
            "source_anchor": "ClassMotor.cs:492-516; ClassHeadBoard.cs:121-124",
            "before": _json_safe(before),
            "set_home": _json_safe(set_home),
            "after": _json_safe(after),
            "controller_command_acknowledged": acknowledged,
            "controller_terminal_state_verified": terminal,
            "reference_publication_required": terminal,
            "physical_motion_commanded": False,
            "physical_effect_verified": False,
            "failure": None if terminal else "x_set_home_zero_readback_not_verified",
        }

    def x_enable_xy_current_mode(self, *, enabled: bool) -> dict[str, Any]:
        result = self.tester.motor_oem_set_xy_current_mode(bool(enabled))
        if isinstance(result, Mapping) and result.get("ok") is True:
            self._x_profile_overrides[6] = 31 if bool(enabled) else 1
        return dict(result) if isinstance(result, Mapping) else {
            "ok": False,
            "failure": "enableXY_result_not_mapping",
            "physical_motion_commanded": False,
        }

    def x_enable_xyz_current_mode(self, *, enabled: bool, z_current_up: int = 31) -> dict[str, Any]:
        result = self.tester.motor_oem_set_xyz_current_mode(bool(enabled), z_current_up=int(z_current_up))
        if isinstance(result, Mapping) and result.get("ok") is True:
            self._x_profile_overrides[6] = 31 if bool(enabled) else 1
            self._z_profile_overrides[6] = int(z_current_up) if bool(enabled) else 1
        return dict(result) if isinstance(result, Mapping) else {
            "ok": False,
            "failure": "enableXYZ_result_not_mapping",
            "physical_motion_commanded": False,
        }

    def x_wait_for_motor(self, *, pending_ticket: Mapping[str, Any], wait_timeout_s: float) -> dict[str, Any]:
        return self._x_finalize(pending_ticket, timeout_s=float(wait_timeout_s), motion_kind="absolute", publish=True)

    def _x_home_result(self, home: Any, *, intent: str, source_method: str) -> dict[str, Any]:
        position = self.tester.motor_get_position(5, motor=0)
        speed = self.tester.motor_get_speed(5, motor=0)
        source_return = home.get("source_return_code") if isinstance(home, Mapping) else None
        short_circuit = home.get("short_circuit") if isinstance(home, Mapping) else None
        command_acknowledged = bool(
            isinstance(home, Mapping)
            and (home.get("controller_command_acknowledged") is True or short_circuit == "MotorHome_and_CurrentPosition_zero")
        )
        terminal = bool(
            isinstance(home, Mapping)
            and home.get("ok") is True
            and home.get("controller_terminal_state_verified") is True
            and home.get("controller_home_proof_verified") is True
            and self._x_readback_verified(position, 0)
            and isinstance(speed, Mapping)
            and speed.get("ok") is True
            and self._x_tmcl_success(speed.get("ack"))
            and type(speed.get("speed")) is int
            and speed.get("speed") == 0
        )
        return {"ok": terminal, "axis": "x", "intent": intent, "source_method": source_method, "home": _json_safe(home), "source_return": source_return, "position": _json_safe(position), "terminal_speed": _json_safe(speed), "controller_command_acknowledged": command_acknowledged, "controller_terminal_state_verified": terminal, "reference_publication_required": terminal, "physical_motion_commanded": short_circuit is None, "physical_effect_verified": False, "failure": None if terminal else "x_home_evidence_not_verified"}

    def _x_go_home(self, *, speed: int, rehome: bool, timeout_s: float, intent: str, source_method: str) -> dict[str, Any]:
        self._x_require_motion_preflight()
        home = self.tester.motor_oem_go_home("x", speed=int(speed), rehome=bool(rehome), timeout_s=max(30.0, float(timeout_s)), require_switch_transition=False)
        return self._x_home_result(home, intent=intent, source_method=source_method)

    def x_startup_home(self, *, timeout_s: float) -> dict[str, Any]:
        self._x_require_motion_preflight()
        home = self.tester.motor_oem_axis_search_home("x", speed=250, timeout_s=float(timeout_s), max_search_abs_delta=None)
        if not isinstance(home, Mapping) or home.get("ok") is not True:
            return self._x_home_result(home, intent="startup_home", source_method="axisSearchHome(x,250)")
        self.__dict__.get("sleep", time.sleep)(0.020)
        set_home = self.tester.motor_set_home(5, motor=0)
        home_evidence = self._x_home_result(home, intent="startup_home", source_method="axisSearchHome(x,250);setHome")
        speed = self.x_set_max_speed(1700)
        self.__dict__.get("sleep", time.sleep)(0.040)
        park_ticket = self._x_issue_absolute(6000, source_mode="initializeMotors.x_park_6000", clamp_low=60)
        park = self._x_finalize(park_ticket, timeout_s=float(timeout_s), motion_kind="startup_park", publish=False)
        ok = bool(home_evidence.get("ok") is True and isinstance(set_home, Mapping) and set_home.get("ok") is True and speed.get("ok") is True and park.get("ok") is True)
        return {"ok": ok, "axis": "x", "intent": "startup_home", "source_method": "axisSearchHome(250);20ms;setHome;SAP4=1700;40ms;moveX(6000)", "home": _json_safe(home), "home_evidence": _json_safe(home_evidence), "set_home": _json_safe(set_home), "speed_restore": _json_safe(speed), "park": _json_safe(park), "controller_command_acknowledged": ok, "controller_terminal_state_verified": ok, "controller_position_steps": park.get("after_position_steps"), "oem_display_position_steps": 0, "reference_publication_required": ok, "physical_motion_commanded": True, "physical_effect_verified": False, "failure": None if ok else "x_startup_home_sequence_failed"}

    def x_home_axis(self, *, timeout_s: float) -> dict[str, Any]:
        self._x_require_motion_preflight()
        home = self.tester.motor_oem_axis_search_home("x", speed=250, timeout_s=float(timeout_s), max_search_abs_delta=None)
        return self._x_home_result(home, intent="diagnostic_home_axis", source_method="ClassControlInterface.HomeAxis(x)->axisSearchHome(250)")

    def x_manual_panel_home(self, *, timeout_s: float) -> dict[str, Any]:
        return self._x_go_home(speed=500, rehome=True, timeout_s=timeout_s, intent="manual_panel_home", source_method="goHome(true,x,500,true)")

    def x_move_to_origin_home(self, *, timeout_s: float) -> dict[str, Any]:
        return self._x_go_home(speed=1700, rehome=True, timeout_s=timeout_s, intent="move_to_origin_home", source_method="goHome(true,x,1700,true)")

    def x_caught_plate_recovery_home(self, *, timeout_s: float) -> dict[str, Any]:
        return self._x_go_home(speed=1700, rehome=False, timeout_s=timeout_s, intent="caught_plate_recovery_home", source_method="goHome(false,x,1700,true)")


    def home_xy(self, *, timeout_s: float = 30.0, allow_implementation_mapped_predicate: bool = False) -> dict[str, Any]:
        del allow_implementation_mapped_predicate
        present = getattr(self.tester, "_oem_board_present", None)
        x_present = not callable(present) or bool(present(5))
        y_present = not callable(present) or bool(present(4))
        if not (x_present and y_present):
            return {
                "ok": True,
                "intent": "home_xy",
                "source_noop": True,
                "source_return": None,
                "source_return_semantics": "null_when_either_board_absent",
                "board_presence": {"x": x_present, "y": y_present},
                "command_issued": False,
                "physical_motion_commanded": False,
                "controller_command_acknowledged": False,
                "target_event_128_observed": False,
                "physical_effect_verified": False,
                "source_anchor": "ClassControlInterface.HomeXY:5054-5070",
            }
        setup = {
            "x_speed": self.tester.motor_set_axis_param(5, 4, 200, motor=0),
            "x_acc": self.tester.motor_set_axis_param(5, 5, 200, motor=0),
            "y_speed": self.tester.motor_set_axis_param(4, 4, 200, motor=0),
            "y_acc": self.tester.motor_set_axis_param(4, 5, 200, motor=0),
        }
        results: dict[str, Any] = {}
        errors: list[str] = []
        restore: dict[str, Any] = {}
        setup_ok = all(isinstance(row, Mapping) and row.get("ok") is True and isinstance(row.get("readback"), Mapping) and row["readback"].get("value") == 200 for row in setup.values())
        try:
            if setup_ok:
                def run(axis: str) -> None:
                    try:
                        results[axis] = self.tester.motor_oem_go_home(axis, speed=200, rehome=False, timeout_s=max(30.0, float(timeout_s)), require_switch_transition=False)
                    except Exception as exc:
                        errors.append(f"{axis}:{type(exc).__name__}:{exc}")
                tx = threading.Thread(target=run, args=("x",), daemon=False)
                ty = threading.Thread(target=run, args=("y",), daemon=False)
                tx.start(); ty.start(); tx.join(); ty.join()
        finally:
            restore = {
                "x_speed": self.tester.motor_set_axis_param(5, 4, 1700, motor=0),
                "x_acc": self.tester.motor_set_axis_param(5, 5, 350, motor=0),
                "y_speed": self.tester.motor_set_axis_param(4, 4, 1800, motor=0),
                "y_acc": self.tester.motor_set_axis_param(4, 5, 400, motor=0),
            }
        expected_restore = {"x_speed": 1700, "x_acc": 350, "y_speed": 1800, "y_acc": 400}
        restore_ok = all(isinstance(restore[name], Mapping) and restore[name].get("ok") is True and isinstance(restore[name].get("readback"), Mapping) and restore[name]["readback"].get("value") == expected for name, expected in expected_restore.items())
        positions = {"x": self.tester.motor_get_position(5, motor=0), "y": self.tester.motor_get_position(4, motor=0)}
        home_ok = bool(setup_ok and not errors and all(isinstance(results.get(axis), Mapping) and results[axis].get("ok") is True and results[axis].get("controller_home_proof_verified") is True for axis in ("x", "y")) and all(self._x_value(positions[axis]) == 0 for axis in ("x", "y")) and restore_ok)
        source_return = {axis: results[axis].get("source_return_code") if isinstance(results.get(axis), Mapping) else None for axis in ("x", "y")}
        receipt = {"ok": home_ok, "intent": "home_xy", "command_issued": bool(setup_ok), "setup": _json_safe(setup), "home": _json_safe(results), "source_return": source_return, "home_errors": errors, "positions": _json_safe(positions), "restore": _json_safe(restore), "controller_command_acknowledged": home_ok, "controller_terminal_state_verified": home_ok, "reference_publication_required": home_ok, "physical_effect_verified": False, "failure": None if home_ok else "homexy_evidence_not_verified", "source_anchor": "ClassControlInterface.HomeXY:5054-5069"}
        if not home_ok:
            receipt["safety_stop"] = _json_safe({
                "x": self.tester.motor_oem_stop_exact(5, motor=0),
                "y": self.tester.motor_oem_stop_exact(4, motor=0),
            })
            if self.reference_store is not None:
                receipt["reference_desync"] = _json_safe(self.reference_store.mark_desynced_many(
                    tuple(
                        MarkAxisDesyncedCommand(
                            axis=axis,
                            reason="HomeXY terminal evidence was not accepted.",
                            source="serial206.home_xy",
                            motion_kind="home_xy",
                        )
                        for axis in ("x", "y")
                    )
                ))
        return receipt

    def _z_profile(self) -> dict[str, Any]:
        profile = dict(self.tester._motion_oem_axis_profile("z"))
        if int(profile.get("board", -1)) != 4 or int(profile.get("motor", -1)) != 1:
            raise RuntimeError("serial-206 Z authority must resolve to board 4 motor 1")
        self.tester.motor_oem_require_no_motion_profile(
            "z", expected_overrides=dict(self._z_profile_overrides)
        )
        interlock = self.tester.motor_oem_verify_motion_interlock()
        if not isinstance(interlock, Mapping) or interlock.get("ok") is not True:
            raise RuntimeError(f"serial-206 Z interlock failed: {interlock}")
        return profile

    def _z_set_profile_parameter(
        self,
        *,
        param: int,
        value: int,
        intent: str,
        source_method: str,
    ) -> dict[str, Any]:
        profile = self._z_profile()
        board = int(profile["board"])
        motor = int(profile["motor"])
        write = self.tester.motor_set_axis_param(board, int(param), int(value), motor=motor)
        readback = self.tester.motor_get_axis_param(board, int(param), motor=motor)
        acknowledged = bool(
            isinstance(write, Mapping)
            and write.get("ok") is True
            and self._z_tmcl_success(write.get("ack"))
        )
        verified = bool(
            acknowledged
            and self._z_tmcl_success(readback.get("ack") if isinstance(readback, Mapping) else None)
            and self._z_value(readback) == int(value)
        )
        if verified:
            self._z_profile_overrides[int(param)] = int(value)
        return {
            "ok": verified,
            "intent": intent,
            "source_method": source_method,
            "source_anchor": "ClassControlInterface.cs:4835-4867",
            "board": board,
            "motor": motor,
            "param": int(param),
            "value": int(value),
            "write": _json_safe(write),
            "readback": _json_safe(readback),
            "controller_command_acknowledged": acknowledged,
            "controller_terminal_state_verified": verified,
            "physical_motion": False,
            "physical_effect_verified": False,
            "failure": None if verified else "z_profile_parameter_readback_mismatch",
        }

    def z_set_max_speed(self, value: int = 1791) -> dict[str, Any]:
        selected = 1791 if int(value) == 0 else int(value)
        return self._z_set_profile_parameter(
            param=4,
            value=selected,
            intent="set_max_speed",
            source_method="ClassControlInterface.setMaxSpeed(z)",
        )

    def z_set_max_acc(self, value: int = 576) -> dict[str, Any]:
        selected = 576 if int(value) == 0 else int(value)
        return self._z_set_profile_parameter(
            param=5,
            value=selected,
            intent="set_max_acc",
            source_method="ClassControlInterface.setMaxAcc(z)",
        )

    def z_set_vmax(self, value: int = 0) -> dict[str, Any]:
        selected = 1791 if int(value) == 0 else int(value)
        return self._z_set_profile_parameter(
            param=4,
            value=selected,
            intent="set_vmax",
            source_method="ClassControlInterface.setZaxisVmax",
        )

    def z_set_current_max(self, value: int | None = None) -> dict[str, Any]:
        profile = self._z_profile()
        source_value = 100 if value is None else int(value)
        selected = int(profile["run_current"]) if source_value == 100 else source_value
        return self._z_set_profile_parameter(
            param=6,
            value=selected,
            intent="set_current_max",
            source_method="ClassControlInterface.setZaxisCurrentmax(current=100)",
        )

    def z_restore_original_speed(self) -> dict[str, Any]:
        return self._z_set_profile_parameter(
            param=4,
            value=1791,
            intent="restore_original_speed",
            source_method="ClassControlInterface.restoreOriginalZSpeed",
        )

    def z_clear_profile_overrides(self) -> None:
        self._z_profile_overrides.clear()

    @staticmethod
    def _z_value(row: Any) -> int | None:
        if not isinstance(row, Mapping):
            return None
        for key in ("value", "position"):
            value = row.get(key)
            if type(value) is int:
                return int(value)
        return None

    def z_terminal_status(self) -> dict[str, Any]:
        """Read the compact terminal Z state needed by the operator cockpit."""
        profile = dict(self.tester._motion_oem_axis_profile("z"))
        board = int(profile["board"])
        motor = int(profile["motor"])
        rows = {
            param: self.tester.motor_get_axis_param(board, param, motor=motor)
            for param in (1, 3, 9, 10, 12, 13)
        }
        values = {param: self._z_value(row) for param, row in rows.items()}
        verified = all(
            isinstance(row, Mapping) and self._z_tmcl_success(row.get("ack"))
            for row in rows.values()
        )
        return {
            "ok": verified,
            "position_steps": values[1],
            "speed_steps_s": values[3],
            "left_switch_state": values[9],
            "right_switch_state": values[10],
            "right_switch_disabled": None if values[12] is None else values[12] != 0,
            "left_switch_disabled": None if values[13] is None else values[13] != 0,
            "readbacks": _json_safe(rows),
            "authority": "serial206_terminal_register_readback",
        }

    def _z_tmcl_success(self, ack: Any) -> bool:
        verifier = getattr(self.tester, "_tmcl_success", None)
        if callable(verifier):
            try:
                return verifier(ack) is True
            except Exception:
                return False
        return isinstance(ack, Mapping) and ack.get("status") == 100

    def _z_stop_acknowledged(self, stop: Any) -> bool:
        if not isinstance(stop, Mapping) or stop.get("ok") is not True:
            return False
        first = stop.get("first_delivery")
        second = stop.get("second_delivery")
        if first is not None or second is not None:
            return self._z_tmcl_success(first) and self._z_tmcl_success(second)
        return self._z_tmcl_success(stop.get("ack"))

    def _z_terminal_zero_verified(self, wait: Any) -> bool:
        return bool(
            isinstance(wait, Mapping)
            and wait.get("stopped") is True
            and type(wait.get("last_speed")) is int
            and int(wait["last_speed"]) == 0
            and self._z_tmcl_success(wait.get("last_ack"))
        )

    def _z_finalize_position_move(
        self,
        *,
        profile: Mapping[str, Any],
        before: Mapping[str, Any],
        target: int,
        move: Mapping[str, Any],
        wait_timeout_s: float,
        pre_command_event_window: Mapping[str, Any],
        event_window: Mapping[str, Any],
    ) -> dict[str, Any]:
        board = int(profile["board"])
        motor = int(profile["motor"])
        wait = self.tester.motor_wait_stopped(
            board,
            motor=motor,
            timeout_s=float(wait_timeout_s),
            require_seen_nonzero=True,
            target_position=int(target),
        )
        events = self.tester.collect_bus_events(duration_s=0.30, timeout_ms=12, max_events=96)
        after_sequence = event_window.get("after_sequence") if isinstance(event_window, Mapping) else None
        axis_events = [
            row for row in events if isinstance(row, Mapping)
            and row.get("board") == board
            and row.get("motor") == motor
            and type(row.get("event_sequence")) is int
            and type(after_sequence) is int
            and int(row["event_sequence"]) > int(after_sequence)
        ]
        target_events = [row for row in axis_events if row.get("status") == 128]
        error_events = [row for row in axis_events if row.get("status") in {13, 14, 130}]
        after = self.tester.motor_get_position(board, motor=motor)
        before_value = self._z_value(before)
        after_value = self._z_value(after)
        move_ack = bool(
            isinstance(move, Mapping)
            and move.get("ok") is True
            and self._z_tmcl_success(move.get("ack"))
        )
        terminal = self._z_terminal_zero_verified(wait)
        failure = None
        if not move_ack:
            failure = "z_move_command_not_acknowledged"
        elif not terminal:
            failure = "z_terminal_zero_speed_not_verified"
        elif error_events:
            failure = "z_controller_error_event"
        elif not target_events:
            failure = "z_target_event_128_missing_or_stale"
        ok = failure is None
        failure_stop = None
        if not ok:
            failure_stop = self.z_stop(timeout_s=3.0)
        return {
            "ok": ok,
            "failure": failure,
            "robot_http_acknowledged": True,
            "controller_command_acknowledged": move_ack,
            "controller_terminal_state_verified": terminal,
            "physical_effect_verified": False,
            "pre_command_event_window": _json_safe(pre_command_event_window),
            "event_window": _json_safe(event_window),
            "target_events": _json_safe(target_events),
            "controller_error_events": _json_safe(error_events),
            "before": _json_safe(before),
            "after": _json_safe(after),
            "before_position_steps": before_value,
            "target_position_steps": int(target),
            "after_position_steps": after_value,
            "move": _json_safe(move),
            "wait": _json_safe(wait),
            "failure_stop": _json_safe(failure_stop),
        }

    def z_move_steps(self, *, steps: int, wait_timeout_s: float = 20.0) -> dict[str, Any]:
        profile = self._z_profile()
        before = self.tester.motor_get_position(4, motor=1)
        before_value = self._z_value(before)
        if not (
            isinstance(before, Mapping)
            and before.get("ok") is True
            and self._z_tmcl_success(before.get("ack"))
            and before_value is not None
        ):
            return {
                "ok": False,
                "error": "z_current_position_unavailable",
                "failure": "z_current_position_unavailable",
                "before": _json_safe(before),
                "physical_motion_commanded": False,
            }
        target = before_value + int(steps)
        if before_value < 0 or before_value > 160000 or target < 0 or target > 160000:
            return {
                "ok": False,
                "error": "z_source_coordinate_out_of_bounds",
                "failure": "z_source_coordinate_out_of_bounds",
                "before_position_steps": before_value,
                "requested_steps": int(steps),
                "target_position_steps": target,
                "source_min_steps": 0,
                "source_max_steps": 160000,
                "physical_motion_commanded": False,
            }
        source_limit_margin_steps = 20
        if target < source_limit_margin_steps or target > 160000 - source_limit_margin_steps:
            return {
                "ok": False,
                "error": "z_source_coordinate_inside_limit_margin",
                "failure": "z_source_coordinate_inside_limit_margin",
                "before_position_steps": before_value,
                "requested_steps": int(steps),
                "target_position_steps": target,
                "source_min_steps": 0,
                "source_max_steps": 160000,
                "source_limit_margin_steps": source_limit_margin_steps,
                "physical_motion_commanded": False,
                "source_anchor": "ClassMotor.moveSteps: lowPosLimits/highPosLimits 20-step margin",
            }
        pre_command_event_window = self.tester.begin_bus_event_window()
        move = self.tester.motor_move_relative(4, int(steps), motor=1)
        result = self._z_finalize_position_move(
            profile=profile, before=before, target=target, move=move,
            wait_timeout_s=wait_timeout_s,
            pre_command_event_window=pre_command_event_window,
            event_window=pre_command_event_window,
        )
        result.update({
            "intent": "move_steps",
            "requested_steps": int(steps),
            "source_anchor": "ClassControlInterface.moveSteps:4165-4204",
        })
        return result

    def z_move_absolute(
        self,
        *,
        requested_position_steps: int,
        pseudo_home_steps: int,
        wait_timeout_s: float = 20.0,
    ) -> dict[str, Any]:
        profile = self._z_profile()
        requested = int(requested_position_steps)
        effective = max(int(pseudo_home_steps), requested)
        if requested < 0 or requested > 160000 or effective < 0 or effective > 160000:
            return {
                "ok": False,
                "error": "z_source_coordinate_out_of_bounds",
                "failure": "z_source_coordinate_out_of_bounds",
                "requested_position_steps": requested,
                "effective_position_steps": effective,
                "source_min_steps": 0,
                "source_max_steps": 160000,
                "physical_motion_commanded": False,
            }
        before = self.tester.motor_get_position(4, motor=1)
        before_value = self._z_value(before)
        if not (
            isinstance(before, Mapping)
            and before.get("ok") is True
            and self._z_tmcl_success(before.get("ack"))
            and before_value is not None
        ):
            return {
                "ok": False,
                "error": "z_current_position_unavailable",
                "failure": "z_current_position_unavailable",
                "before": _json_safe(before),
                "physical_motion_commanded": False,
            }
        if before_value < 0 or before_value > 160000:
            return {
                "ok": False,
                "error": "z_source_coordinate_out_of_bounds",
                "failure": "z_source_coordinate_out_of_bounds",
                "before_position_steps": before_value,
                "source_min_steps": 0,
                "source_max_steps": 160000,
                "physical_motion_commanded": False,
            }
        if before_value == effective:
            wait = self.tester.motor_wait_stopped(
                4,
                motor=1,
                timeout_s=float(wait_timeout_s),
                require_seen_nonzero=False,
                target_position=effective,
            )
            after = self.tester.motor_get_position(4, motor=1)
            after_value = self._z_value(after)
            terminal_verified = bool(
                self._z_terminal_zero_verified(wait)
                and isinstance(after, Mapping)
                and after.get("ok") is True
                and self._z_tmcl_success(after.get("ack"))
                and after_value == effective
            )
            return {
                "ok": terminal_verified,
                "failure": None if terminal_verified else "z_noop_terminal_state_unverified",
                "axis": "z",
                "intent": "move_absolute",
                "source_anchor": "ClassHeadBoard.cs:174-177",
                "source_noop": True,
                "noop_reason": "already_at_effective_target",
                "requested_position_steps": requested,
                "pseudo_home_steps": int(pseudo_home_steps),
                "effective_position_steps": effective,
                "target_position_steps": effective,
                "effective_target_clamped": effective != requested,
                "before": _json_safe(before),
                "before_position_steps": before_value,
                "after": _json_safe(after),
                "after_position_steps": after_value,
                "wait": _json_safe(wait),
                "target_position_verified": after_value == effective,
                "physical_motion_commanded": False,
                "controller_command_acknowledged": False,
                "controller_terminal_state_verified": terminal_verified,
                "physical_effect_verified": False,
            }
        current_write = self.tester.motor_set_axis_param(4, 6, int(profile["run_current"]), motor=1)
        current_readback = self.tester.motor_get_axis_param(4, 6, motor=1)
        if not (
            isinstance(current_write, Mapping)
            and current_write.get("ok") is True
            and self._z_tmcl_success(current_write.get("ack"))
            and isinstance(current_readback, Mapping)
            and self._z_tmcl_success(current_readback.get("ack"))
            and self._z_value(current_readback) == int(profile["run_current"])
        ):
            return {
                "ok": False,
                "error": "z_source_current_readback_failed",
                "failure": "z_source_current_readback_failed",
                "current_write": _json_safe(current_write),
                "current_readback": _json_safe(current_readback),
                "physical_motion_commanded": False,
            }
        pre_command_event_window = self.tester.begin_bus_event_window()
        move = self.tester.motor_move_absolute(4, effective, motor=1)
        result = self._z_finalize_position_move(
            profile=profile, before=before, target=effective, move=move,
            wait_timeout_s=wait_timeout_s,
            pre_command_event_window=pre_command_event_window,
            event_window=pre_command_event_window,
        )
        result.update({
            "intent": "move_absolute",
            "requested_position_steps": requested,
            "effective_position_steps": effective,
            "pseudo_home_steps": int(pseudo_home_steps),
            "source_anchor": "ClassControlInterface.moveZ:4254-4265",
        })
        return result

    def z_manual_home(
        self,
        *,
        timeout_s: float = 30.0,
        _source_identity: str = "manual_panel",
    ) -> dict[str, Any]:
        self._z_profile()
        interlock = self.tester.motor_oem_verify_motion_interlock()
        if interlock.get("ok") is not True:
            return {"ok": False, "failure": "motion_interlock_not_ready", "interlock": interlock}
        if _source_identity == "move_z_home":
            home = self.tester.motor_oem_move_z_home(rehome=True, timeout_s=float(timeout_s))
            self._z_profile_overrides[6] = 31
            intent_name = "move_z_home"
            source_method = "ClassControlInterface.MoveZHome -> setZaxisCurrentmax + goHome(rehome=true, 1791)"
            source_anchor = "ClassControlInterface.cs:4623-4632"
        else:
            manual_home = self.tester.motor_oem_go_home(
                "z",
                speed=1791,
                rehome=True,
                timeout_s=float(timeout_s),
                require_switch_transition=False,
                max_search_abs_delta=160000,
            )
            home = {
                "ok": isinstance(manual_home, Mapping) and manual_home.get("ok") is True,
                "home": manual_home,
            }
            intent_name = "manual_home"
            source_method = "ClassControlInterface.btnHomeZ_Click -> goHome(rehome=true, 1791)"
            source_anchor = "ClassControlInterface.cs:2370-2378"
        ok = isinstance(home, Mapping) and home.get("ok") is True
        home_evidence = home.get("home") if isinstance(home, Mapping) else None
        move_home = home_evidence.get("move_home") if isinstance(home_evidence, Mapping) else None
        stop = home_evidence.get("stop") if isinstance(home_evidence, Mapping) else None
        wait = home_evidence.get("wait") if isinstance(home_evidence, Mapping) else None
        source_short_circuit = bool(
            isinstance(home_evidence, Mapping)
            and home_evidence.get("short_circuit") == "MotorHome_and_CurrentPosition_zero"
        )
        command_acknowledged = (
            bool(
                isinstance(home_evidence, Mapping)
                and home_evidence.get("controller_home_proof_verified") is True
                and home_evidence.get("controller_terminal_state_verified") is True
            )
            if source_short_circuit
            else bool(
                isinstance(move_home, Mapping)
                and move_home.get("ok") is True
                and self._z_tmcl_success(move_home.get("ack"))
            )
        )
        terminal_verified = (
            bool(
                isinstance(home_evidence, Mapping)
                and home_evidence.get("controller_terminal_state_verified") is True
                and home_evidence.get("controller_home_proof_verified") is True
            )
            if source_short_circuit
            else bool(self._z_stop_acknowledged(stop) and self._z_terminal_zero_verified(wait))
        )

        def evidence_value(key: str, *aliases: str) -> int | None:
            if not isinstance(home_evidence, Mapping):
                return None
            for selected in (key, *aliases):
                value = self._z_value(home_evidence.get(selected))
                if value is not None:
                    return value
            return None

        false_guard = home_evidence.get("false_home_guard") if isinstance(home_evidence, Mapping) else None
        failure = None if ok else (
            str(false_guard)
            if isinstance(false_guard, str) and false_guard
            else str(home.get("failure"))
            if isinstance(home, Mapping) and isinstance(home.get("failure"), str)
            else f"z_{intent_name}_evidence_not_verified"
        )
        home_summary = {
            "failure": failure,
            "short_circuit": (
                home_evidence.get("short_circuit")
                if isinstance(home_evidence, Mapping) and isinstance(home_evidence.get("short_circuit"), str)
                else None
            ),
            "before_position_steps": evidence_value("position_before"),
            "after_position_steps": evidence_value("position_after"),
            "after_set_home_position_steps": evidence_value("position_after_sethome", "position_after_set_home"),
            "home_before": evidence_value("home_before"),
            "home_hit": evidence_value("home_hit"),
            "home_after_stop": evidence_value("home_after_stop", "home_after"),
            "right_before": evidence_value("right_before"),
            "right_after_stop": evidence_value("right_after_stop", "right_after"),
            "controller_command_acknowledged": command_acknowledged,
            "controller_home_proof_verified": bool(
                isinstance(home_evidence, Mapping)
                and home_evidence.get("controller_home_proof_verified") is True
            ),
            "controller_terminal_state_verified": terminal_verified,
            "home_decision": _json_safe(
                home_evidence.get("home_decision")
                if isinstance(home_evidence, Mapping) else None
            ),
            "stop_acknowledged": self._z_stop_acknowledged(stop),
            "terminal_zero_speed_verified": self._z_terminal_zero_verified(wait),
            "controller_error_events": _json_safe(
                home_evidence.get("controller_error_events")
                if isinstance(home_evidence, Mapping) else None
            ),
        }
        return {
            "ok": ok,
            "failure": failure,
            "intent": intent_name,
            "source_method": source_method,
            "source_anchor": source_anchor,
            "interlock": interlock,
            "home_summary": home_summary,
            "home": _json_safe(home),
            "controller_command_acknowledged": command_acknowledged,
            "controller_terminal_state_verified": terminal_verified,
            "motor_output_state": "stopped_readback" if terminal_verified else "unverified",
            "motor_torque_verified": False,
            "physical_effect_verified": False,
        }

    def z_move_z_home(self, *, timeout_s: float = 30.0) -> dict[str, Any]:
        return self.z_manual_home(
            timeout_s=float(timeout_s),
            _source_identity="move_z_home",
        )

    def z_set_home(self, *, source: str = "oem.z.set_home") -> dict[str, Any]:
        """No-motion manual home: record the current physical position as controller 0.

        OEM ClassMotor.setHome writes axis parameter 1 (actual position) to 0.
        There is no search and no motion. This primitive proves only the
        controller write/readback; the authority-owning provider performs the
        durable reference publication and lifecycle commit as one compensated
        transaction.
        """
        # This is deliberately *not* ``_z_profile()``.  That helper enforces the
        # Linux no-motion preparation/profile and motion interlock, neither of
        # which is called by the OEM ClassMotor.setHome implementation.  Reusing
        # it made the exact OEM state-establishing write unreachable precisely
        # when it is needed to repair a stale/desynced controller coordinate.
        profile = dict(self.tester._motion_oem_axis_profile("z"))
        if int(profile.get("board", -1)) != 4 or int(profile.get("motor", -1)) != 1:
            raise RuntimeError("serial-206 Z setHome authority must resolve to board 4 motor 1")
        set_home = self.tester.motor_set_home(4, motor=1)
        position = self.tester.motor_get_position(4, motor=1)
        value = self._z_value(position)
        command_acknowledged = bool(
            isinstance(set_home, Mapping)
            and set_home.get("ok") is True
            and self._z_tmcl_success(set_home.get("ack"))
            and self._z_value(set_home.get("readback")) == 0
        )
        terminal_verified = bool(
            isinstance(position, Mapping)
            and position.get("ok") is True
            and self._z_tmcl_success(position.get("ack"))
            and value == 0
        )
        ok = command_acknowledged and terminal_verified
        if not command_acknowledged:
            failure = "z_set_home_command_not_acknowledged"
        elif not terminal_verified:
            failure = "z_set_home_zero_readback_not_verified"
        else:
            failure = None
        return {
            "ok": ok,
            "intent": "set_home",
            "source_method": "ClassMotor.setHome (SAP param 1 = 0) at current physical position; no OEM profile/interlock/search prelude",
            "source_anchor": "ClassMotor.cs:492-516; ClassHeadBoard.cs:121-124",
            "board": 4,
            "motor": 1,
            "set_home": _json_safe(set_home),
            "position": _json_safe(position),
            "reference_publication_required": bool(ok),
            "reference_publication_owner": "Serial206OemInitializationProvider",
            "physical_motion": False,
            "controller_command_acknowledged": command_acknowledged,
            "controller_terminal_state_verified": terminal_verified,
            "failure": failure,
        }

    def z_right_reference(self) -> dict[str, Any]:
        """Expose GAP10 as diagnostic evidence; it is not a production reference."""
        self._z_profile()
        right = self.tester.motor_get_axis_param(4, 10, motor=1)
        return {
            "ok": False,
            "intent": "right_switch_diagnostic_only",
            "failure": "z_live_right_reference_retired",
            "right_switch": _json_safe(right),
            "physical_motion": False,
            "set_home_performed": False,
            "physical_effect_verified": False,
        }

    def z_startup_home(self, *, timeout_s: float = 30.0) -> dict[str, Any]:
        self._z_profile()
        result = self.tester.motor_oem_axis_search_home(
            "z", speed=1791, timeout_s=float(timeout_s), max_search_abs_delta=160000
        )
        return {**dict(result), "intent": "startup_axis_search_1791"}

    def z_diagnostic_home_axis(self, *, timeout_s: float = 30.0) -> dict[str, Any]:
        self._z_profile()
        interlock = self.tester.motor_oem_verify_motion_interlock()
        if interlock.get("ok") is not True:
            return {"ok": False, "failure": "motion_interlock_not_ready", "interlock": interlock}
        home = self.tester.motor_oem_home_axis_board_test("z", timeout_s=float(timeout_s))
        ok = isinstance(home, Mapping) and home.get("ok") is True
        home_evidence = home.get("home") if isinstance(home, Mapping) else None
        go_home = home_evidence.get("go_home") if isinstance(home_evidence, Mapping) else None
        diagnostic_move = go_home.get("move_home") if isinstance(go_home, Mapping) else None
        diagnostic_stop = go_home.get("stop") if isinstance(go_home, Mapping) else None
        diagnostic_wait = go_home.get("wait") if isinstance(go_home, Mapping) else None
        command_acknowledged = bool(
            isinstance(diagnostic_move, Mapping)
            and diagnostic_move.get("ok") is True
            and self._z_tmcl_success(diagnostic_move.get("ack"))
        ) if isinstance(diagnostic_move, Mapping) else bool(
            isinstance(go_home, Mapping)
            and go_home.get("controller_command_acknowledged") is True
        )
        terminal_verified = bool(
            self._z_stop_acknowledged(diagnostic_stop)
            and self._z_terminal_zero_verified(diagnostic_wait)
        ) if isinstance(diagnostic_stop, Mapping) or isinstance(diagnostic_wait, Mapping) else bool(
            isinstance(go_home, Mapping)
            and go_home.get("controller_terminal_state_verified") is True
        )
        return {
            "ok": ok,
            "intent": "diagnostic_home_axis_597",
            "source_method": "ClassControlInterface.HomeAxis(z) -> axisSearchHome(597)",
            "interlock": interlock,
            "home": home,
            "controller_command_acknowledged": command_acknowledged,
            "controller_terminal_state_verified": terminal_verified,
            "physical_effect_verified": False,
        }

    def z_stop(self, *, timeout_s: float = 3.0) -> dict[str, Any]:
        stop = self.tester.motor_stop(4, motor=1)
        wait = self.tester.motor_wait_stopped(
            4, motor=1, timeout_s=float(timeout_s), require_seen_nonzero=False
        )
        command_acknowledged = self._z_stop_acknowledged(stop)
        terminal = self._z_terminal_zero_verified(wait)
        return {
            "ok": bool(command_acknowledged and terminal),
            "intent": "stop",
            "controller_command_acknowledged": command_acknowledged,
            "controller_terminal_state_verified": terminal,
            "physical_effect_verified": False,
            "failure": None if command_acknowledged and terminal else (
                "z_stop_command_not_acknowledged" if not command_acknowledged
                else "z_stop_zero_speed_not_verified"
            ),
            "stop": _json_safe(stop),
            "wait": _json_safe(wait),
        }

    def z_abort(self, *, timeout_s: float = 3.0) -> dict[str, Any]:
        abort = self.tester.motor_oem_force_abort_motion(reason="oem.z.abort")
        stop = self.z_stop(timeout_s=float(timeout_s))
        ok = bool(
            isinstance(abort, Mapping)
            and abort.get("ok") is True
            and abort.get("no24v") is True
            and stop.get("ok") is True
        )
        return {
            "ok": ok,
            "intent": "full_machine_force_abort_from_z_recovery_context",
            "source_method": "ClassControlInterface.forceAbortMotion",
            "source_anchor": "ClassControlInterface.cs:5095-5121",
            "physical_scope": "all_present_motor_boards_then_explicit_z_terminal_verification",
            "z_only": False,
            "abort": _json_safe(abort),
            "stop": _json_safe(stop),
            "no24v": bool(self.tester.oem_no24v_state()),
            "controller_command_acknowledged": stop.get("controller_command_acknowledged") is True,
            "controller_terminal_state_verified": stop.get("controller_terminal_state_verified") is True,
            "physical_effect_verified": False,
            "failure": None if ok else "z_abort_not_verified",
        }

    def z_resume_after_abort(self, *, timeout_s: float = 30.0) -> dict[str, Any]:
        rail = self.tester.motor_query_24v_sensor()
        initial_check_ok = bool(
            isinstance(rail, Mapping)
            and rail.get("reply_valid") is True
            and rail.get("sample_valid") is True
            and rail.get("oem_no24v") is False
            and rail.get("oem_scalar") == 0
        )
        if not initial_check_ok:
            return {
                "ok": False,
                "intent": "resume_after_abort",
                "failure": "wakefrompause_initial_check_24v_failed",
                "initial_check_24v": _json_safe(rail),
                "controller_command_acknowledged": False,
                "controller_terminal_state_verified": False,
            }
        profile = self._z_profile()
        home = self.tester.motor_oem_axis_search_home(
            "z",
            speed=1791,
            timeout_s=float(timeout_s),
            max_search_abs_delta=int(profile["home_search_max_abs_delta"]),
        )
        go_home = home.get("go_home") if isinstance(home, Mapping) else None
        command_acknowledged = bool(
            isinstance(go_home, Mapping)
            and go_home.get("controller_command_acknowledged") is True
        )
        terminal_verified = bool(
            isinstance(go_home, Mapping)
            and go_home.get("controller_terminal_state_verified") is True
            and go_home.get("controller_home_proof_verified") is True
        )
        ok = bool(
            isinstance(home, Mapping)
            and home.get("ok") is True
            and command_acknowledged
            and terminal_verified
        )
        return {
            "ok": ok,
            "intent": "resume_after_abort",
            "source_method": "wakefrompause -> initialCheck -> rehome/initializeMotors Z stage",
            "source_anchor": "ControlLib.wakefrompause; ControlLib.rehome:8784-8796; ClassControlInterface.initializeMotors:3350-3353",
            "source_non_z_span": "restore thermal-door state and resume TC/output-chiller temperatures (not executed by Z-only projection)",
            "initial_check_24v": _json_safe(rail),
            "home": _json_safe(home),
            "home_summary": {
                "short_circuit": go_home.get("short_circuit") if isinstance(go_home, Mapping) else None,
                "controller_home_proof_verified": bool(
                    isinstance(go_home, Mapping)
                    and go_home.get("controller_home_proof_verified") is True
                ),
                "source_return_code": home.get("source_return_code") if isinstance(home, Mapping) else None,
            },
            "controller_command_acknowledged": command_acknowledged,
            "controller_terminal_state_verified": terminal_verified,
            "physical_effect_verified": False,
            "failure": None if ok else "z_resume_rehome_controller_evidence_unverified",
        }

    def z_reconcile_switch_masks(self) -> dict[str, Any]:
        expected = {12: 1, 13: 0}
        before = {param: self.tester.motor_get_axis_param(4, param, motor=1) for param in (12, 13)}
        writes = {
            param: self.tester.motor_set_axis_param(4, param, expected[param], motor=1)
            for param in (12, 13)
        }
        after = {param: self.tester.motor_get_axis_param(4, param, motor=1) for param in (12, 13)}
        ok = all(
            isinstance(writes[param], Mapping) and writes[param].get("ok") is True
            and self._z_value(after[param]) == expected[param]
            for param in (12, 13)
        )
        return {
            "ok": ok,
            "intent": "reconcile_switch_masks",
            "physical_motion": False,
            "physical_effect_verified": False,
            "before": _json_safe(before),
            "writes": _json_safe(writes),
            "after": _json_safe(after),
            "source_exact": False,
            "machine_bound_expected": expected,
            "recovery_reason": (
                "restore serial-206 persistent Z wiring contract: "
                "GAP12/right disabled, GAP13/GAP9-left enabled"
            ),
        }

    def motor_get_axis_param(self, *args: Any, **kwargs: Any) -> Any:
        return self.tester.motor_get_axis_param(*args, **kwargs)

    def motor_move_relative(self, *args: Any, **kwargs: Any) -> Any:
        board = args[0]
        motor = kwargs.get("motor", 0)
        before = self.tester.motor_get_position(board, motor=motor)
        move = self.tester.motor_move_relative(*args, **kwargs)
        self._pending_move_position_before = before
        return {"move": move, "position": {"before": before}}

    def motor_wait_stopped(self, *args: Any, **kwargs: Any) -> Any:
        board = args[0]
        motor = kwargs.get("motor", 0)
        wait = self.tester.motor_wait_stopped(*args, **kwargs)
        after = self.tester.motor_get_position(board, motor=motor)
        return {"wait": wait, "position": {"after": after}}

    def motor_oem_home_axis(self, axis: str, *args: Any, **kwargs: Any) -> Any:
        if str(axis).strip().lower() == "z" and kwargs.get("startup") is True:
            return self.z_startup_home(timeout_s=float(kwargs.get("timeout_s", 30.0)))
        return self.tester.motor_oem_home_axis(axis, *args, **kwargs)

    def motor_set_axis_param(self, *args: Any, **kwargs: Any) -> Any:
        return self.tester.motor_set_axis_param(*args, **kwargs)

    def motor_move_absolute(self, *args: Any, **kwargs: Any) -> Any:
        board = args[0]
        motor = kwargs.get("motor", 0)
        before = self.tester.motor_get_position(board, motor=motor)
        move = self.tester.motor_move_absolute(*args, **kwargs)
        self._pending_move_position_before = before
        return {"move": move, "position": {"before": before}}

    def motor_oem_door_search_home(self, *args: Any, **kwargs: Any) -> Any:
        return self.tester.motor_oem_door_search_home(*args, **kwargs)

    def motor_thermal_door_status(self) -> Any:
        return self.tester.motor_thermal_door_status()

    def _machine_config_bundle(self) -> Any:
        return self.tester._machine_config_bundle()

    def _oem_no_motion_tmcl_with_readback(self, **kwargs: Any) -> Any:
        board = int(kwargs["board"])
        command = int(kwargs["command"])
        if board != int(self.tester.BOARD_CHILLER) or command != 9:
            return {"ok": False, "failure": "unsupported_oem_gp_write", "board": board, "command": command}
        row = self.tester.chiller_gp_write(
            int(kwargs["cmd_type"]), int(kwargs["motor"]), int(kwargs["value"]), verify=True,
        )
        return {
            "name": str(kwargs["name"]),
            "board": board,
            "command": command,
            "cmd_type": int(kwargs["cmd_type"]),
            "motor": int(kwargs["motor"]),
            "value": int(kwargs["value"]),
            "ack": row.get("ack") if isinstance(row, Mapping) else None,
            "readback": row.get("readback") if isinstance(row, Mapping) else None,
            "verified": row.get("verified") if isinstance(row, Mapping) else False,
            "ok": bool(isinstance(row, Mapping) and row.get("ok") is True and row.get("verified") is True),
        }

    def oem_initialize_motors_branch_binding(self) -> dict[str, Any]:
        status = self.capability_status()
        authority = status.get("initialize_motors_branch_authority")
        if not isinstance(authority, Mapping):
            return {"ok": False, "failure": "frozen_serial206_machine_branch_not_bound"}
        serial_number = authority.get("serial_number")
        camera_calibrated = authority.get("camera_calibrated")
        return {
            "ok": type(serial_number) is int and type(camera_calibrated) is bool,
            "serial_number": serial_number,
            "camera_calibrated": camera_calibrated,
            "source": authority.get("source"),
        }

    def oem_set_calibrated_ui_positions_zero(self) -> Any:
        binding = self.oem_initialize_motors_branch_binding()
        if binding.get("ok") is not True:
            return {"ok": False, "failure": "frozen_serial206_machine_branch_not_bound"}
        return {
            "ok": True,
            "calibrated": True,
            "writes": [
                {"axis": "x", "value": "0", "applied": True},
                {"axis": "y", "value": "0", "applied": True},
                {"axis": "z", "value": "0", "applied": True},
                {"axis": "z", "value": "0", "applied": True, "target": "secondary_z_display"},
            ],
            "readback": {"x": "0", "y": "0", "z": "0"},
            "durable_host_sink": "oem_serial206_initialization_state.machine_status.calibrated_ui_positions",
            "source_anchor": "ClassControlInterface.initializeMotors:3388-3393",
        }

    def motor_oem_open_thermal_door(self, *args: Any, **kwargs: Any) -> Any:
        return self.tester.motor_oem_open_thermal_door(*args, **kwargs)

    def query_tip_status(self) -> dict[str, Any]:
        raw = self.pipette_transport.query_tip_status_all()
        rows = raw.get("channels") if isinstance(raw, Mapping) else None
        if not isinstance(rows, list) or len(rows) != 4:
            return {"ok": False, "channels": None, "raw": _json_safe(raw)}
        channels = [row.get("tip_loaded") if isinstance(row, Mapping) else None for row in rows]
        if any(type(value) is not bool for value in channels):
            return {"ok": False, "channels": None, "raw": _json_safe(raw)}
        self._last_tip_channels = [index for index, loaded in enumerate(channels) if loaded]
        return {"ok": True, "channels": channels, "controller_evidence": _json_safe(raw)}

    def query_all_pipette_tip_states(self) -> dict[str, Any]:
        result = self.query_tip_status()
        channels = result.get("channels")
        loaded = [index for index, value in enumerate(channels or []) if value is True]
        return {
            "ok": result.get("ok") is True,
            "tip_exists": bool(loaded),
            "channels_with_tips": loaded,
            "channels": channels,
            "controller_evidence": result.get("controller_evidence"),
        }

    def eject_all_tips(self) -> Any:
        if self._last_tip_channels is None:
            return {"ok": False, "failure": "exact_tip_query_required_before_eject"}
        return self.pipette_transport.eject_all_tips_for_oem_startup(
            operator_ack="EJECT_STALE_STARTUP_TIPS",
            expected_channels_with_tips=list(self._last_tip_channels),
        )

    def eject_all_pipette_tips_for_oem_startup(
        self,
        *,
        operator_ack: str,
        expected_channels_with_tips: list[int],
    ) -> Any:
        return self.pipette_transport.eject_all_tips_for_oem_startup(
            operator_ack=operator_ack,
            expected_channels_with_tips=list(expected_channels_with_tips),
        )

    def initiate_pipette_group(self) -> Any:
        return self.pipette_transport.initialize(PipetteInitCommand())

    def initiate_pipette_group_for_oem_initialize_motion(self, *, cycle: str) -> Any:
        return self.pipette_transport.initiate_group_once_for_oem_initialize_motion(cycle=cycle)

    def checked_pipette_status_for_oem_initialize_motion(self, *, attempt: str) -> Any:
        return self.pipette_transport.checked_pipette_status_for_oem_initialize_motion(attempt=attempt)

    @staticmethod
    def _position_value(result: Any) -> int:
        if not isinstance(result, Mapping) or result.get("ok") is not True:
            raise RuntimeError("axis position readback failed")
        value = result.get("position")
        if type(value) is bool or not isinstance(value, int):
            raise RuntimeError("axis position readback is not an integer")
        return int(value)

    def _axis_profile(self, axis: str) -> Mapping[str, Any]:
        profile = self.tester._motion_oem_axis_profile(str(axis).lower(), startup=True)
        if not isinstance(profile, Mapping):
            raise RuntimeError(f"OEM axis profile unavailable: {axis}")
        return profile

    def _read_axis_position(self, axis: str) -> int:
        profile = self._axis_profile(axis)
        return self._position_value(
            self.tester.motor_get_position(profile["board"], motor=profile.get("motor", 0))
        )

    def _prepare_path_axis(
        self,
        axis: str,
        *,
        speed: int | None = None,
        acc: int | None = None,
    ) -> dict[str, Any]:
        profile = self._axis_profile(axis)
        result = self.tester.motor_prepare_axis(
            profile["board"],
            motor=profile.get("motor", 0),
            run_current=profile.get("run_current"),
            standby_current=profile.get("standby_current"),
            speed=profile.get("speed") if speed is None else int(speed),
            acc=profile.get("acc") if acc is None else int(acc),
            stall_guard=profile.get("stall_guard"),
            disable_right=profile.get("disable_right"),
            disable_left=profile.get("disable_left"),
        )
        if not isinstance(result, Mapping) or result.get("ok") is not True:
            raise RuntimeError(f"OEM axis preparation failed: {axis}")
        return dict(result)

    def oem_initialize_motion_move_absolute(
        self,
        axis: str,
        position: int,
        *,
        timeout_s: float,
        speed: int | None = None,
        acc: int | None = None,
        pseudo_home_steps: int = 65000,
    ) -> dict[str, Any]:
        del speed, acc, timeout_s
        axis_key = str(axis).lower()
        if axis_key == "z":
            return self.oem_move_z(
                int(position),
                pseudo_home_steps=int(pseudo_home_steps),
                motor_current=31,
                wait_for_stop=True,
            )
        return self.oem_move_axis_absolute(axis_key, int(position), wait_for_stop=True)

    def oem_move_axis_absolute(
        self,
        axis: str,
        position: int,
        *,
        wait_for_stop: bool = True,
    ) -> dict[str, Any]:
        """Issue one source-shaped raw axis move without profile preparation."""
        profile = self._axis_profile(axis)
        move = self.tester.motor_oem_move_absolute(
            profile["board"],
            int(position),
            motor=profile.get("motor", 0),
            wait_for_stop=bool(wait_for_stop),
            max_position=profile.get("axis_max_steps"),
        )
        return {
            "ok": bool(isinstance(move, Mapping) and move.get("ok") is True),
            "axis": str(axis).lower(),
            "target": int(position),
            "move": _json_safe(move),
            "source_anchor": "ClassHeadBoard.moveToAbs; ClassControlInterface.moveX/moveY",
        }

    def oem_move_z(
        self,
        position: int,
        *,
        pseudo_home_steps: int,
        motor_current: int = 31,
        wait_for_stop: bool = True,
    ) -> dict[str, Any]:
        """ClassControlInterface.moveZ with dynamic PSUDO_Z_HOME."""
        profile = self._axis_profile("z")
        board = int(profile["board"])
        present = getattr(self.tester, "_oem_board_present", None)
        if callable(present) and not present(board):
            return {
                "ok": True,
                "axis": "z",
                "source_noop": "board_null",
                "source_return_code": 0,
                "source_anchor": "ClassControlInterface.moveZ:4254-4265",
            }
        effective = max(int(pseudo_home_steps), int(position))
        current_set = self.tester.motor_set_axis_param(
            board,
            6,
            int(motor_current),
            motor=profile.get("motor", 0),
        )
        move = self.tester.motor_oem_move_absolute(
            board,
            effective,
            motor=profile.get("motor", 0),
            wait_for_stop=bool(wait_for_stop),
            max_position=profile.get("axis_max_steps"),
        )
        return {
            "ok": bool(isinstance(move, Mapping) and move.get("ok") is True),
            "axis": "z",
            "requested": int(position),
            "effective": effective,
            "pseudo_z_home": int(pseudo_home_steps),
            "motor_current": int(motor_current),
            "current_set": _json_safe(current_set),
            "move": _json_safe(move),
            "source_anchor": "ClassControlInterface.moveZ:4254-4265",
        }

    def oem_move_xy(
        self,
        x: int,
        y: int,
        *,
        wait_timeout_s: float = 5.0,
    ) -> dict[str, Any]:
        """ClassControlInterface.moveXY source ordering and acceleration."""
        return self.move_xy(int(x), int(y), wait_timeout_s=float(wait_timeout_s))
        px = self._axis_profile("x")
        py = self._axis_profile("y")
        present = getattr(self.tester, "_oem_board_present", None)
        x_present = not callable(present) or present(int(px["board"]))
        y_present = not callable(present) or present(int(py["board"]))
        if not y_present:
            move = self.oem_move_axis_absolute("x", int(x), wait_for_stop=True) if x_present else None
            return {"ok": move is None or move.get("ok") is True, "branch": "y_board_null", "move_x": _json_safe(move)}
        if not x_present:
            return {"ok": True, "branch": "x_board_null_literal_moveX_y_noop", "source_noop": "moveX checks the same null board"}
        current_x = self._read_axis_position("x")
        current_y = self._read_axis_position("y")
        target_x, target_y = int(x), int(y)
        distance_x = abs(target_x - current_x)
        distance_y = abs(target_y - current_y)
        commands: dict[str, Any] = {}
        waits: dict[str, Any] = {}
        launch_order: list[str] = []
        if distance_x <= 20 or distance_y <= 20:
            if distance_x != 0:
                commands["x"] = self.oem_move_axis_absolute("x", target_x, wait_for_stop=True)
                launch_order.append("x")
            if distance_y != 0:
                commands["y"] = self.oem_move_axis_absolute("y", target_y, wait_for_stop=True)
                launch_order.append("y")
            return {
                "ok": all(commands[key].get("ok") is True for key in launch_order),
                "targets": {"x": target_x, "y": target_y},
                "before": {"x": current_x, "y": current_y},
                "commands": _json_safe(commands),
                "launch_order": launch_order,
                "source_anchor": "ClassControlInterface.moveXY:4285-4311",
            }

        x_acc = 400 if distance_x > 10000 else 350
        y_acc = 750 if distance_y > 10000 else 400
        set_acc = {
            "x": self.tester.motor_set_axis_param(px["board"], 5, x_acc, motor=px.get("motor", 0)),
            "y": self.tester.motor_set_axis_param(py["board"], 5, y_acc, motor=py.get("motor", 0)),
        }
        event_window = self.tester.begin_bus_event_window()
        if distance_x > distance_y:
            commands["x"] = self.tester.motor_oem_move_absolute(
                px["board"], target_x, motor=px.get("motor", 0), wait_for_stop=False, max_position=px.get("axis_max_steps")
            )
            launch_order.append("x")
            if distance_y > 4000:
                time.sleep(0.050 * distance_x / distance_y)
            commands["y"] = self.tester.motor_oem_move_absolute(
                py["board"], target_y, motor=py.get("motor", 0), wait_for_stop=False, max_position=py.get("axis_max_steps")
            )
            launch_order.append("y")
        else:
            commands["y"] = self.tester.motor_oem_move_absolute(
                py["board"], target_y, motor=py.get("motor", 0), wait_for_stop=False, max_position=py.get("axis_max_steps")
            )
            launch_order.append("y")
            if distance_x > 4000:
                time.sleep(0.050 * distance_y / distance_x)
            commands["x"] = self.tester.motor_oem_move_absolute(
                px["board"], target_x, motor=px.get("motor", 0), wait_for_stop=False, max_position=px.get("axis_max_steps")
            )
            launch_order.append("x")
        time.sleep(0.005)
        waits["pair"] = self.tester.motor_oem_wait_targets_reached(
            ((px["board"], px.get("motor", 0)), (py["board"], py.get("motor", 0))),
            timeout_s=5.0,
            event_window=event_window,
        )
        restore_acc = {
            "x": self.tester.motor_set_axis_param(px["board"], 5, 350, motor=px.get("motor", 0)),
            "y": self.tester.motor_set_axis_param(py["board"], 5, 400, motor=py.get("motor", 0)),
        }
        return {
            "ok": isinstance(waits["pair"], Mapping) and waits["pair"].get("ok") is True,
            "targets": {"x": target_x, "y": target_y},
            "before": {"x": current_x, "y": current_y},
            "commands": _json_safe(commands),
            "waits": _json_safe(waits),
            "set_acc": _json_safe(set_acc),
            "restore_acc": _json_safe(restore_acc),
            "launch_order": launch_order,
            "source_anchor": "ClassControlInterface.moveXY:4285-4366",
        }

    def oem_move_to(
        self,
        x: int,
        y: int,
        z: int,
        *,
        pseudo_home_steps: int,
        run_in_parallel: bool = True,
        wait_timeout_s: float = 5.0,
        gripper_confirmed: bool | None = None,
        tip_loaded: bool | None = None,
        plate_on_gantry: int | None = None,
        location19_y: int | None = None,
    ) -> dict[str, Any]:
        """Literal ClassControlInterface.moveTo branch ordering."""
        pseudo = int(pseudo_home_steps)
        target = {"x": int(x), "y": int(y), "z": int(z)}
        results: list[dict[str, Any]] = []

        def home_axis(axis: str, speed: int) -> dict[str, Any]:
            profile = self._axis_profile(axis)
            present = getattr(self.tester, "_oem_board_present", None)
            if callable(present) and not present(int(profile["board"])):
                return {"ok": True, "axis": axis, "source_noop": "board_null"}
            if axis == "x":
                return self.x_move_to_origin_home(timeout_s=float(wait_timeout_s))
            return self.tester.motor_oem_go_home(
                axis, speed=int(speed), rehome=True,
                timeout_s=max(30.0, float(wait_timeout_s)),
                require_switch_transition=False,
            )

        def move_axis(axis: str, position: int, acc: int) -> dict[str, Any]:
            profile = self._axis_profile(axis)
            effective = max(60, int(position)) if axis == "x" else int(position)
            if axis == "x":
                return self.x_move_absolute(
                    position_steps=effective,
                    acceleration=int(acc),
                    wait_for_stop=True,
                    wait_timeout_s=float(wait_timeout_s),
                    source_mode="ClassControlInterface.moveTo.moveX",
                    clamp_low_to_60=True,
                    publish_motion_metadata=True,
                )
            set_acc = self.tester.motor_set_axis_param(
                profile["board"], 5, int(acc), motor=profile.get("motor", 0)
            )
            try:
                move = self.tester.motor_oem_move_absolute(
                    profile["board"], effective, motor=profile.get("motor", 0),
                    wait_for_stop=True, max_position=profile.get("axis_max_steps"),
                )
            finally:
                restore = self.tester.motor_set_axis_param(
                    profile["board"], 5, 350 if axis == "x" else 400,
                    motor=profile.get("motor", 0),
                )
            return {"ok": isinstance(move, Mapping) and move.get("ok") is True,
                    "axis": axis, "target": effective, "set_acc": _json_safe(set_acc),
                    "move": _json_safe(move), "restore_acc": _json_safe(restore)}

        def run_pair(first: Callable[[], dict[str, Any]], second: Callable[[], dict[str, Any]], delay_s: float) -> list[dict[str, Any]]:
            first_result: list[dict[str, Any]] = []
            first_error: list[BaseException] = []
            started = threading.Event()
            def invoke_first() -> None:
                started.set()
                try:
                    first_result.append(first())
                except BaseException as exc:
                    first_error.append(exc)
            thread = threading.Thread(target=invoke_first, daemon=False)
            thread.start()
            started.wait()
            time.sleep(delay_s)
            second_result = second()
            thread.join()
            if first_error:
                raise first_error[0]
            return first_result + [second_result]

        if target == {"x": 0, "y": 0, "z": 0}:
            z_home = self.z_move_z_home(timeout_s=float(wait_timeout_s))
            xy_results: dict[str, Any] = {}
            xy_errors: list[str] = []

            def home_xy_axis(axis: str, speed: int) -> None:
                try:
                    if axis == "x":
                        xy_results[axis] = self.x_move_to_origin_home(timeout_s=float(wait_timeout_s))
                    else:
                        xy_results[axis] = self.tester.motor_oem_go_home(
                            axis,
                            speed=int(speed),
                            rehome=True,
                            timeout_s=float(wait_timeout_s),
                            require_switch_transition=False,
                        )
                except Exception as exc:
                    xy_errors.append(f"{axis}:{type(exc).__name__}:{exc}")

            if bool(run_in_parallel):
                tx = threading.Thread(target=home_xy_axis, args=("x", 1700), daemon=False)
                ty = threading.Thread(target=home_xy_axis, args=("y", 1800), daemon=False)
                tx.start(); ty.start(); tx.join(); ty.join()
            else:
                home_xy_axis("x", 1700)
                home_xy_axis("y", 1800)
            xy_ok = not xy_errors and all(
                isinstance(xy_results.get(axis), Mapping)
                and xy_results[axis].get("ok") is True
                for axis in ("x", "y")
            )
            # Controller home proof does not publish durable X/Y reference authority.
            # The provider-owned observation transition must perform the atomic pair
            # publication after an independently bound observation receipt.
            reference = {
                "ok": False,
                "state": "awaiting_observation",
                "axes": ["x", "y"],
                "physical_effect_verified": False,
            } if xy_ok else None
            ok = bool(z_home.get("ok") is True and xy_ok)
            return {
                "ok": ok,
                "source_return_code": 0 if ok else 1,
                "branch": "all_zero_home",
                "source_home_semantics": {
                    "z": "ClassControlInterface.MoveZHome -> goHome(true,1791)",
                    "x": "ClassControlInterface.goHome(true,X,1700,true)",
                    "y": "ClassControlInterface.goHome(true,Y,1800,true)",
                },
                "run_in_parallel": bool(run_in_parallel),
                "z_home": _json_safe(z_home),
                "xy_home": _json_safe(xy_results),
                "xy_errors": xy_errors,
                "xy_reference_state": _json_safe(reference),
                "reference_publication_required": bool(xy_ok),
                "z_home_reference_verified": bool(z_home.get("ok") is True),
                "home_summary": _json_safe(z_home.get("home_summary")),
                "controller_command_acknowledged": bool(z_home.get("controller_command_acknowledged") is True),
                "controller_terminal_state_verified": bool(z_home.get("controller_terminal_state_verified") is True),
                "physical_motion_commanded": bool(ok),
                "failure": None if ok else "all_zero_move_to_home_failed",
                "source_anchor": "ClassControlInterface.moveTo:4463-4506",
            }

        if type(gripper_confirmed) is not bool or type(tip_loaded) is not bool:
            raise RuntimeError("moveTo gripper confirmation and TipLoaded authority are required")
        if plate_on_gantry in {4, 5} and type(location19_y) is not int:
            raise RuntimeError("moveTo location-19 Y authority is required for loaded plate branch")
        plate_clear_y = int(location19_y) if type(location19_y) is int else 0

        current = {axis: self._read_axis_position(axis) for axis in ("x", "y", "z")}
        if current["z"] > pseudo:
            results.append(self.oem_move_z(pseudo, pseudo_home_steps=pseudo, motor_current=31, wait_for_stop=True))
        x_acc = 400 if abs(target["x"] - current["x"]) > 10000 else 350
        y_acc = 750 if abs(target["y"] - current["y"]) > 10000 else 400

        def move_y_or_home() -> dict[str, Any]:
            return home_axis("y", 1800) if target["y"] == 0 else move_axis("y", target["y"], y_acc)

        if gripper_confirmed and not tip_loaded:
            results.append(self.oem_move_xy(target["x"], target["y"], wait_timeout_s=5.0))
            branch = "confirmed_gripper_no_tip_moveXY"
        elif target["y"] < current["y"] or target["y"] < 46800:
            if plate_on_gantry in {4, 5}:
                if current["y"] < plate_clear_y and (current["x"] > 66400 or target["x"] > 66400) and abs(current["x"] - target["x"]) > 10000:
                    results.append(move_axis("y", plate_clear_y, y_acc))
                results.append(move_axis("x", target["x"], x_acc))
                time.sleep(0.001)
                results.append(move_y_or_home())
                branch = "descending_y_loaded_plate"
            elif run_in_parallel:
                results.extend(run_pair(lambda: move_axis("x", target["x"], x_acc), move_y_or_home, 0.600))
                branch = "descending_y_parallel_x_first"
            else:
                results.extend((move_axis("x", target["x"], x_acc), move_y_or_home()))
                branch = "descending_y_sequential_x_first"
        elif run_in_parallel:
            results.extend(run_pair(move_y_or_home, lambda: move_axis("x", target["x"], x_acc), 0.300))
            branch = "parallel_y_first"
        else:
            results.extend((move_y_or_home(), move_axis("x", target["x"], x_acc)))
            branch = "sequential_y_first"

        time.sleep(0.001)
        time.sleep(0.001)
        if target["z"] > pseudo:
            results.append(self.oem_move_z(target["z"], pseudo_home_steps=pseudo, motor_current=31, wait_for_stop=True))
        px, py = self._axis_profile("x"), self._axis_profile("y")
        restore = {
            "x": self.tester.motor_set_axis_param(px["board"], 5, 350, motor=px.get("motor", 0)),
            "y": self.tester.motor_set_axis_param(py["board"], 5, 400, motor=py.get("motor", 0)),
        }
        return {
            "ok": all(isinstance(row, Mapping) and row.get("ok") is True for row in results),
            "source_return_code": 0,
            "branch": branch,
            "target": target,
            "before": current,
            "pseudo_z_home": pseudo,
            "operations": _json_safe(results),
            "restore_acc": _json_safe(restore),
            "source_anchor": "ClassControlInterface.moveTo:4463-4620",
        }

    def absolute(
        self,
        axis: str,
        position: int,
        *,
        speed: int | None = None,
        acc: int | None = None,
        wait_timeout_s: float,
    ) -> dict[str, Any]:
        return self.oem_initialize_motion_move_absolute(
            axis,
            position,
            timeout_s=wait_timeout_s,
            speed=speed,
            acc=acc,
        )

    def relative(
        self,
        axis: str,
        delta: int,
        *,
        speed: int | None = None,
        acc: int | None = None,
        wait_timeout_s: float,
    ) -> dict[str, Any]:
        profile = self._axis_profile(axis)
        before = self._read_axis_position(axis)
        prepared = self._prepare_path_axis(axis, speed=speed, acc=acc)
        move = self.tester.motor_move_relative(
            profile["board"],
            int(delta),
            motor=profile.get("motor", 0),
        )
        wait = self.tester.motor_wait_stopped(
            profile["board"],
            motor=profile.get("motor", 0),
            timeout_s=float(wait_timeout_s),
            require_seen_nonzero=True,
        )
        after = self._read_axis_position(axis)
        return {
            "ok": bool(
                isinstance(move, Mapping)
                and move.get("ok") is True
                and isinstance(wait, Mapping)
                and wait.get("stopped") is True
                and after - before == int(delta)
            ),
            "axis": str(axis).lower(),
            "delta": int(delta),
            "prepare": _json_safe(prepared),
            "move": _json_safe(move),
            "wait": _json_safe(wait),
            "position": {"before": before, "after": after},
        }

    def move_xy(
        self,
        x: int,
        y: int,
        *,
        speed: int | None = None,
        acc: int | None = None,
        wait_timeout_s: float,
    ) -> dict[str, Any]:
        requested = {"x": int(x), "y": int(y)}
        present_fn = getattr(self.tester, "motor_oem_axis_board_present", None)
        if callable(present_fn):
            present = {axis: bool(present_fn(axis)) for axis in ("x", "y")}
        else:
            present = {axis: True for axis in ("x", "y")}
        receipt: dict[str, Any] = {
            "ok": False,
            "source_operation": "ClassControlInterface.moveXY",
            "source_anchor": "ClassControlInterface.cs:4285-4367",
            "requested": requested,
            "board_present": present,
            "ignored_compatibility_inputs": {"speed": speed, "acc": acc, "wait_timeout_s": float(wait_timeout_s)},
            "oem_wait_timeout_ms": 5000,
        }
        if not present["y"]:
            fallback = self.x_move_absolute(position_steps=requested["x"], source_mode="moveXY.missing_y.moveX", clamp_low_to_60=True)
            receipt.update({"branch": "missing_y_calls_moveX_x", "fallback": _json_safe(fallback), "ok": fallback.get("ok") is True})
            return receipt
        if not present["x"]:
            fallback = self.x_move_absolute(
                position_steps=requested["y"],
                source_mode="moveXY.missing_x.moveX_y",
                clamp_low_to_60=True,
            )
            receipt.update({
                "branch": "missing_x_calls_moveX_y",
                "fallback": _json_safe(fallback),
                "ok": fallback.get("ok") is True,
            })
            return receipt
        reference = self._reference_snapshot(("x", "y"), "ClassControlInterface.moveXY")
        before_rows = {
            "x": self.tester.motor_get_position(5, motor=0),
            "y": self.tester.motor_get_position(4, motor=0),
        }
        if not all(self._x_readback_verified(row) for row in before_rows.values()):
            receipt.update({"failure": "moveXY_position_before_not_acknowledged", "command_issued": False, "physical_motion_commanded": False})
            return receipt
        x_before = self._x_value(before_rows["x"])
        y_before = self._x_value(before_rows["y"])
        assert type(x_before) is int and type(y_before) is int
        before = {"x": x_before, "y": y_before}
        distances = {axis: abs(requested[axis] - before[axis]) for axis in ("x", "y")}
        receipt.update({"reference_before": _json_safe(reference), "before": before, "distances": distances})
        if distances == {"x": 0, "y": 0}:
            after_rows = {
                "x": self.tester.motor_get_position(5, motor=0),
                "y": self.tester.motor_get_position(4, motor=0),
            }
            speed_rows = {
                "x": self.tester.motor_get_speed(5, motor=0),
                "y": self.tester.motor_get_speed(4, motor=0),
            }
            after = {axis: self._x_value(row) for axis, row in after_rows.items()}
            speeds = {axis: row.get("speed") if isinstance(row, Mapping) else None for axis, row in speed_rows.items()}
            terminal = bool(
                all(self._x_readback_verified(after_rows[axis], requested[axis]) for axis in ("x", "y"))
                and all(isinstance(speed_rows[axis], Mapping) and speed_rows[axis].get("ok") is True and self._x_tmcl_success(speed_rows[axis].get("ack")) and type(speeds[axis]) is int and speeds[axis] == 0 for axis in ("x", "y"))
            )
            receipt.update({
                "ok": terminal,
                "branch": "source_noop",
                "source_noop": True,
                "noop_reason": "both_axes_already_at_target",
                "command_issued": False,
                "physical_motion_commanded": False,
                "controller_command_acknowledged": False,
                "target_event_128_observed": False,
                "controller_error_events": [],
                "motion_metadata_recorded": False,
                "after": after,
                "terminal_speed": _json_safe(speed_rows),
                "terminal_speed_steps_s": speeds,
                "target_position_verified": after == requested,
                "controller_terminal_state_verified": terminal,
                "physical_effect_verified": False,
                "failure": None if terminal else "moveXY_noop_terminal_state_unverified",
            })
            return receipt
        if distances["x"] <= 20 or distances["y"] <= 20:
            commands: dict[str, Any] = {}
            waits: dict[str, Any] = {}
            if distances["x"]:
                commands["x"] = self.x_move_absolute(position_steps=requested["x"], source_mode="moveXY.near_axis.moveX", clamp_low_to_60=True, publish_motion_metadata=False)
                waits["x"] = commands["x"].get("wait") or {"ok": commands["x"].get("ok") is True, "target_reached": commands["x"].get("ok") is True}
            if distances["y"]:
                ticket = self._move_xy_y_issue_absolute(requested["y"], event_window=self.tester.begin_bus_event_window())
                commands["y"] = self._move_xy_y_finalize(ticket, timeout_s=5.0)
                waits["y"] = commands["y"].get("wait") or {"ok": commands["y"].get("ok") is True, "target_reached": commands["y"].get("ok") is True}
            after = {axis: self._read_axis_position(axis) for axis in ("x", "y")}
            restore = {"x": self.tester.motor_set_axis_param(5, 5, 350, motor=0), "y": self.tester.motor_set_axis_param(4, 5, 400, motor=0)}
            return self._finalize_move_xy_receipt({**receipt, "branch": "near_axis_sequential", "launch_order": list(commands)}, commands=commands, waits=waits, after=after, restore=restore, required_axes=tuple(commands))
        x_acc = 400 if distances["x"] > 10000 else 350
        y_acc = 750 if distances["y"] > 10000 else 400
        acceleration_set = {"x": self.tester.motor_set_axis_param(5, 5, x_acc, motor=0), "y": self.tester.motor_set_axis_param(4, 5, y_acc, motor=0)}
        setup_ok = all(isinstance(acceleration_set[axis], Mapping) and acceleration_set[axis].get("ok") is True and isinstance(acceleration_set[axis].get("readback"), Mapping) and acceleration_set[axis]["readback"].get("value") == expected for axis, expected in (("x", x_acc), ("y", y_acc)))
        if not setup_ok:
            restore = {"x": self.tester.motor_set_axis_param(5, 5, 350, motor=0), "y": self.tester.motor_set_axis_param(4, 5, 400, motor=0)}
            receipt.update({"branch": "parallel_setup_failed_before_motion", "acceleration_set": _json_safe(acceleration_set), "acceleration_restore": _json_safe(restore), "command_issued": False, "failure": "moveXY_acceleration_setup_not_verified", "reference_effect": "unchanged_no_motion_delivered"})
            return receipt
        event_window = self.tester.begin_bus_event_window()
        commands: dict[str, Any] = {}
        launch_order: list[str] = []
        stagger_ms = 0
        if distances["x"] > distances["y"]:
            commands["x"] = self._x_issue_absolute(requested["x"], source_mode="moveXY.parallel_x_first", event_window=event_window)
            launch_order.append("x")
            if distances["y"] > 4000:
                stagger_ms = 50 * distances["x"] // distances["y"]
                time.sleep(stagger_ms / 1000.0)
            commands["y"] = self._move_xy_y_issue_absolute(requested["y"], event_window=event_window)
            launch_order.append("y")
        else:
            commands["y"] = self._move_xy_y_issue_absolute(requested["y"], event_window=event_window)
            launch_order.append("y")
            if distances["x"] > 4000:
                stagger_ms = 50 * distances["y"] // distances["x"]
                time.sleep(stagger_ms / 1000.0)
            commands["x"] = self._x_issue_absolute(requested["x"], source_mode="moveXY.parallel_y_first", event_window=event_window)
            launch_order.append("x")
        shared_event_window = dict(event_window)
        shared_cursors: dict[str, float] = {}
        for command in commands.values():
            command_window = command.get("event_window") if isinstance(command, Mapping) else None
            cursors = command_window.get("dispatch_cursors") if isinstance(command_window, Mapping) else None
            if isinstance(cursors, Mapping):
                for key, value in cursors.items():
                    if isinstance(key, str) and isinstance(value, (int, float)):
                        shared_cursors[key] = float(value)
        if shared_cursors:
            shared_event_window["dispatch_cursors"] = shared_cursors
        time.sleep(0.005)
        many_wait = getattr(self.tester, "motor_wait_target_reached_many", None)
        pair_wait: Any = None
        if callable(many_wait):
            pair_wait = many_wait(((5, 0), (4, 0)), event_window=shared_event_window, timeout_s=5.0, sta_sequential=False)
            waits = dict(pair_wait.get("per_axis") or {}) if isinstance(pair_wait, Mapping) else {}
            if not waits:
                waits = {"x": pair_wait, "y": pair_wait}
        else:
            wait_fn = getattr(self.tester, "motor_wait_target_reached", None) or getattr(self.tester, "motor_oem_wait_target_reached")
            waits = {"x": wait_fn(5, motor=0, timeout_s=5.0, event_window=shared_event_window), "y": wait_fn(4, motor=0, timeout_s=5.0, event_window=shared_event_window)}
        restore = {"x": self.tester.motor_set_axis_param(5, 5, 350, motor=0), "y": self.tester.motor_set_axis_param(4, 5, 400, motor=0)}
        after = {axis: self._read_axis_position(axis) for axis in ("x", "y")}
        receipt.update({"branch": "parallel", "acceleration_selected": {"x": x_acc, "y": y_acc}, "acceleration_set": _json_safe(acceleration_set), "event_window": _json_safe(shared_event_window), "launch_order": launch_order, "stagger_ms": stagger_ms, "pre_wait_sleep_ms": 5, "pair_wait": _json_safe(pair_wait) if "pair_wait" in locals() else None})
        return self._finalize_move_xy_receipt(receipt, commands=commands, waits=waits, after=after, restore=restore, required_axes=("x", "y"))


    def parallel(
        self,
        steps: list[dict[str, Any]],
        *,
        speed: int | None = None,
        acc: int | None = None,
        wait_timeout_s: float,
    ) -> dict[str, Any]:
        targets: dict[str, int] = {}
        for step in steps:
            op = str(step.get("op") or "")
            if op == "moveX" and step.get("x") is not None:
                targets["x"] = int(step["x"])
            elif op == "moveY" and step.get("y") is not None:
                targets["y"] = int(step["y"])
            elif op == "moveZ" and step.get("z") is not None:
                targets["z"] = int(step["z"])
            else:
                return {"ok": False, "failure": f"unsupported_parallel_path_step:{op}"}
        if set(targets) == {"x", "y"}:
            return self.move_xy(
                targets["x"],
                targets["y"],
                speed=speed,
                acc=acc,
                wait_timeout_s=wait_timeout_s,
            )
        profiles = {axis: self._axis_profile(axis) for axis in targets}
        before = {axis: self._read_axis_position(axis) for axis in targets}
        prepared = {
            axis: self._prepare_path_axis(axis, speed=speed, acc=acc)
            for axis in targets
        }
        commands = {
            axis: self.tester.motor_move_absolute(
                profiles[axis]["board"],
                target,
                motor=profiles[axis].get("motor", 0),
            )
            for axis, target in targets.items()
        }
        waits = {
            axis: self.tester.motor_wait_stopped(
                profiles[axis]["board"],
                motor=profiles[axis].get("motor", 0),
                timeout_s=float(wait_timeout_s),
                require_seen_nonzero=True,
            )
            for axis in targets
        }
        after = {axis: self._read_axis_position(axis) for axis in targets}
        return {
            "ok": all(
                isinstance(commands[axis], Mapping)
                and commands[axis].get("ok") is True
                and isinstance(waits[axis], Mapping)
                and waits[axis].get("stopped") is True
                and after[axis] == targets[axis]
                for axis in targets
            ),
            "parallel_command_issue": True,
            "targets": targets,
            "before": before,
            "after": after,
            "prepare": _json_safe(prepared),
            "commands": _json_safe(commands),
            "waits": _json_safe(waits),
        }

    def z_execute_path(
        self,
        *,
        steps: list[dict[str, Any]],
        wait_timeout_s: float,
        pseudo_home_steps: int,
        parent_command_id: str | None = None,
    ) -> dict[str, Any]:
        self._z_profile()
        adapter = self

        class LifecycleBoundPathExecutor:
            def __getattr__(self, name: str) -> Any:
                return getattr(adapter, name)

            def oem_move_to(
                self,
                x: int,
                y: int,
                z: int,
                **kwargs: Any,
            ) -> dict[str, Any]:
                command_root = parent_command_id or f"z-path-{time.time_ns()}"
                return adapter._execute_bound_x_intent("move_to", {
                    "command_id": f"{command_root}:move_to",
                    "x": int(x),
                    "y": int(y),
                    "z": int(z),
                    "pseudo_z_home": int(kwargs["pseudo_home_steps"]),
                    "run_in_parallel": kwargs.get("run_in_parallel") is True,
                    "wait_timeout_s": float(kwargs.get("wait_timeout_s", wait_timeout_s)),
                    "gripper_confirmed": kwargs.get("gripper_confirmed") is True,
                    "tip_loaded": kwargs.get("tip_loaded") is True,
                    "plate_on_gantry": kwargs.get("plate_on_gantry"),
                    "location19_y": kwargs.get("location19_y"),
                })

        execution = _execute_oem_steps_live(
            list(steps),
            LifecycleBoundPathExecutor(),
            wait_timeout_s=float(wait_timeout_s),
            speed=None,
            acc=None,
            pseudo_z_home_steps=int(pseudo_home_steps),
        )

        def find_home(value: Any) -> Mapping[str, Any] | None:
            if isinstance(value, Mapping):
                if value.get("z_home_reference_verified") is True:
                    return value
                for child in value.values():
                    match = find_home(child)
                    if match is not None:
                        return match
            elif isinstance(value, (list, tuple)):
                for child in value:
                    match = find_home(child)
                    if match is not None:
                        return match
            return None

        home = find_home(execution)
        ok = bool(isinstance(execution, Mapping) and execution.get("ok") is True)
        return {
            "ok": ok,
            "intent": "path_execute",
            "source_method": "ClassControlInterface.scriptmoveTo/moveTo",
            "source_anchor": "ClassControlInterface.cs:3718-4014;4463-4620",
            "execution": _json_safe(execution),
            "z_home_reference_verified": home is not None,
            "home_summary": _json_safe(home.get("home_summary") if home is not None else None),
            "controller_command_acknowledged": ok,
            "controller_terminal_state_verified": ok,
            "physical_effect_verified": False,
            "failure": None if ok else "z_coordinated_path_execution_failed",
        }

    def z_move_gz(
        self,
        *,
        gripper_position_steps: int,
        z_position_steps: int,
        wait_timeout_s: float = 5.0,
    ) -> dict[str, Any]:
        z_profile = self._z_profile()
        self.tester.motor_oem_require_no_motion_profile("g")
        g_profile = self._axis_profile("g")
        g_target = int(gripper_position_steps)
        z_target = int(z_position_steps)
        for axis, target, profile in (
            ("g", g_target, g_profile),
            ("z", z_target, z_profile),
        ):
            lower = int(profile.get("axis_min_steps", 0))
            upper = int(profile["axis_max_steps"])
            if target < lower or target > upper:
                return {
                    "ok": False,
                    "failure": f"{axis}_target_out_of_bounds:{target}:{lower}:{upper}",
                    "controller_command_acknowledged": False,
                    "controller_terminal_state_verified": False,
                }
        event_window = self.tester.begin_bus_event_window()
        g_move = self.tester.motor_oem_move_absolute(
            int(g_profile["board"]),
            g_target,
            motor=int(g_profile.get("motor", 0)),
            wait_for_stop=False,
            max_position=int(g_profile["axis_max_steps"]),
        )
        z_move = self.tester.motor_oem_move_absolute(
            int(z_profile["board"]),
            z_target,
            motor=int(z_profile["motor"]),
            wait_for_stop=False,
            max_position=int(z_profile["axis_max_steps"]),
        )
        time.sleep(0.005)
        wait = self.tester.motor_oem_wait_targets_reached(
            (
                (int(g_profile["board"]), int(g_profile.get("motor", 0))),
                (int(z_profile["board"]), int(z_profile["motor"])),
            ),
            timeout_s=float(wait_timeout_s),
            event_window=event_window,
        )
        acknowledged = bool(g_move.get("ok") is True and z_move.get("ok") is True)
        terminal = bool(isinstance(wait, Mapping) and wait.get("ok") is True)
        return {
            "ok": bool(acknowledged and terminal),
            "intent": "move_gz",
            "source_method": "ClassControlInterface.moveGZ",
            "source_anchor": "ClassControlInterface.cs:4369-4399",
            "targets": {"g": g_target, "z": z_target},
            "commands": {"g": _json_safe(g_move), "z": _json_safe(z_move)},
            "wait": _json_safe(wait),
            "controller_command_acknowledged": acknowledged,
            "controller_terminal_state_verified": terminal,
            "physical_effect_verified": False,
            "failure": None if acknowledged and terminal else "move_gz_controller_evidence_unverified",
        }

    def z_home_gz(
        self,
        *,
        pseudo_z_home_steps: int,
        delay_s: int = 0,
        wait_timeout_s: float = 30.0,
    ) -> dict[str, Any]:
        profile = self._z_profile()
        parity = load_oem_parity_config(None)
        if parity.blockers:
            return {"ok": False, "failure": "immutable_oem_machine_snapshot_not_bound", "blockers": list(parity.blockers)}
        result = self.tester.motor_oem_home_gz(
            delay_s=int(delay_s),
            pseudo_z_home=int(pseudo_z_home_steps),
            gripper_version=int(parity.values["GripperVersion"]),
            development_machine=False,
            timeout_s=float(wait_timeout_s),
            caught_plate_x_home=lambda: self._execute_bound_x_intent(
                "caught_plate_recovery_home",
                {
                    "command_id": f"z-home-gz-{time.time_ns()}:caught_plate_x_home",
                    "timeout_s": float(wait_timeout_s),
                },
            ),
        )
        # homeGZ writes the source high Z current before gripper homing. The
        # caught-plate exception branch occurs after that write, so retain the
        # verified active value even when the composite result is failed.
        z_current = result.get("z_current") if isinstance(result, Mapping) else None
        z_current_readback = z_current.get("readback") if isinstance(z_current, Mapping) else None
        if (
            isinstance(z_current, Mapping)
            and z_current.get("ok") is True
            and self._z_value(z_current_readback) == int(profile["run_current"])
        ):
            self._z_profile_overrides[6] = int(profile["run_current"])
        ok = bool(isinstance(result, Mapping) and result.get("ok") is True)
        return {
            "ok": ok,
            "intent": "home_gz",
            "source_method": "ClassControlInterface.homeGZ",
            "source_anchor": "ClassControlInterface.cs:4657-4687",
            "pseudo_z_home_steps": int(pseudo_z_home_steps),
            "development_machine": False,
            "result": _json_safe(result),
            "controller_command_acknowledged": ok,
            "controller_terminal_state_verified": ok,
            "physical_effect_verified": False,
            "failure": None if ok else "home_gz_controller_evidence_unverified",
        }

    def z_pipette_position(
        self,
        *,
        location_id: str | int,
        operation: str,
        overpress: bool = False,
    ) -> dict[str, Any]:
        self._z_profile()
        table = load_bound_oem_position_table()
        target_row = table.resolve(location_id=str(location_id))
        if operation == "lower_pipette":
            if target_row.z_low is None:
                return {"ok": False, "failure": "position_table_z_low_missing"}
            target = int(target_row.z_low) + (4030 if bool(overpress) else 0)
            source_anchor = "ClassControlInterface.cs:4401-4421"
        elif operation == "lift_pipette":
            if target_row.z_high is None:
                return {"ok": False, "failure": "position_table_z_high_missing"}
            target = int(target_row.z_high)
            source_anchor = "ClassControlInterface.cs:4423-4431"
        else:
            raise ValueError(f"unsupported pipette Z operation: {operation}")
        move = self.oem_move_axis_absolute("z", target, wait_for_stop=True)
        if operation == "lower_pipette":
            time.sleep(1.0)
        ok = bool(move.get("ok") is True)
        return {
            "ok": ok,
            "intent": operation,
            "source_method": (
                "ClassControlInterface.lowerPipette"
                if operation == "lower_pipette"
                else "ClassControlInterface.liftPipette"
            ),
            "source_anchor": source_anchor,
            "location_id": str(location_id),
            "overpress": bool(overpress) if operation == "lower_pipette" else False,
            "target_position_steps": target,
            "position_table_source": table.source,
            "move": _json_safe(move),
            "controller_command_acknowledged": ok,
            "controller_terminal_state_verified": ok,
            "physical_effect_verified": False,
            "failure": None if ok else f"{operation}_controller_evidence_unverified",
        }

    def z_self_test(
        self,
        *,
        pseudo_z_home_steps: int,
        wait_timeout_s: float = 30.0,
    ) -> dict[str, Any]:
        self._z_profile()
        table = load_bound_oem_position_table()
        parity = load_oem_parity_config(None)
        if parity.blockers:
            return {"ok": False, "failure": "immutable_oem_machine_snapshot_not_bound", "blockers": list(parity.blockers)}
        self_test_z_max = SERIAL206_Z_SELF_TEST_MAX_STEPS
        if not 0 <= int(self_test_z_max) <= int(parity.values["Z_MOTOR_MAX_POSITION"]):
            return {
                "ok": False,
                "failure": "serial206_self_test_z_max_outside_machine_bounds",
                "target": int(self_test_z_max),
                "machine_max": int(parity.values["Z_MOTOR_MAX_POSITION"]),
            }
        initial_home = self.z_move_z_home(timeout_s=float(wait_timeout_s))
        move = (
            self.oem_move_z(
                self_test_z_max,
                pseudo_home_steps=int(pseudo_z_home_steps),
                motor_current=int(parity.values["Z_MOTOR_MAX_CURRENT_UP"]),
                wait_for_stop=True,
            )
            if initial_home.get("ok") is True
            else {"ok": False, "failure": "z_self_test_initial_move_z_home_failed"}
        )
        self._z_profile_overrides[6] = int(parity.values["Z_MOTOR_MAX_CURRENT_UP"])
        home = self.z_diagnostic_home_axis(timeout_s=float(wait_timeout_s))
        home_wrapper = home.get("home") if isinstance(home, Mapping) else None
        axis_home = home_wrapper.get("home") if isinstance(home_wrapper, Mapping) else None
        source_return_code = axis_home.get("source_return_code") if isinstance(axis_home, Mapping) else None
        travel_error_steps = (
            abs(int(source_return_code) - int(self_test_z_max))
            if type(source_return_code) is int
            else None
        )
        self_test_pass = bool(
            initial_home.get("ok") is True
            and move.get("ok") is True
            and isinstance(home, Mapping)
            and home.get("ok") is True
            and type(travel_error_steps) is int
            and travel_error_steps <= 100
        )
        home_summary = {
            "short_circuit": axis_home.get("short_circuit") if isinstance(axis_home, Mapping) else None,
            "controller_home_proof_verified": bool(
                isinstance(axis_home, Mapping)
                and axis_home.get("controller_home_proof_verified") is True
            ),
            "source_return_code": source_return_code,
            "self_test_z_max": self_test_z_max,
            "travel_error_steps": travel_error_steps,
        }
        return {
            "ok": self_test_pass,
            "intent": "self_test",
            "source_method": "ControlLib.selfTest Z segment",
            "source_anchor": "ControlLib.cs:10744-10749; ClassBioXPSettings.SelfTestZMax",
            "source_non_z_span": "HomeAxis(x/y) -> moveX/moveY -> HomeXY (not executed by Z-only projection)",
            "self_test_z_max": self_test_z_max,
            "self_test_tolerance_steps": 100,
            "source_return_code": source_return_code,
            "travel_error_steps": travel_error_steps,
            "self_test_pass": self_test_pass,
            "position_table_source": table.source,
            "initial_move_z_home": _json_safe(initial_home),
            "move": _json_safe(move),
            "home": _json_safe(home),
            "home_summary": home_summary,
            "controller_command_acknowledged": bool(
                initial_home.get("controller_command_acknowledged") is True
                and move.get("ok") is True
                and isinstance(home, Mapping)
                and home.get("controller_command_acknowledged") is True
            ),
            "controller_terminal_state_verified": bool(
                isinstance(home, Mapping)
                and home.get("controller_terminal_state_verified") is True
            ),
            "physical_effect_verified": False,
            "failure": None if self_test_pass else "z_self_test_failed",
        }

    @staticmethod
    def sleep(milliseconds: int) -> dict[str, Any]:
        delay_s = max(0.0, min(float(milliseconds) / 1000.0, 30.0))
        time.sleep(delay_s)
        return {"ok": True, "milliseconds": int(milliseconds), "slept_s": delay_s}

    def oem_initialize_motion_scriptmove_to_waste(
        self,
        *,
        current_location: int,
        current_well: int,
        target_location: int,
        target_well: int,
        position_flag: int,
        tip_loaded: bool,
        tip_dirty: bool,
        timeout_s: float,
        pseudo_home_steps: int = 65000,
        plate_on_gantry: int | str | None = None,
        location19_y: int | None = None,
    ) -> dict[str, Any]:
        table = load_bound_oem_position_table()
        parity = load_oem_parity_config(None)
        if parity.blockers:
            return {"ok": False, "failure": "immutable_oem_machine_snapshot_not_bound", "blockers": list(parity.blockers)}
        current = {
            "x": self._read_axis_position("x"),
            "y": self._read_axis_position("y"),
            "z": self._read_axis_position("z"),
        }
        state = OemMachineState.from_query(
            current_location_id=str(int(current_location)),
            current_well_id=str(int(current_well)),
            current_x=current["x"],
            current_y=current["y"],
            current_z=current["z"],
            tip_loaded=bool(tip_loaded),
            tip_dirty=bool(tip_dirty),
            tip_location=-1,
            clean_path=False,
            device_type="BIOXP",
            gripper_confirmed=False,
            pseudo_z_home=int(pseudo_home_steps),
            plate_on_gantry=plate_on_gantry,
            location19_y=location19_y,
        )
        planner = OemPathPlanner(
            table,
            x_high_limit=int(parity.values.get("XHigh", 90263)),
            y_high_limit=int(parity.values.get("YHigh", 102956)),
        )
        plan = planner.plan_script_move_to(
            current_loc=int(current_location),
            location_id=int(target_location),
            column=0,
            row=0,
            positionflag=int(position_flag),
            state=state,
            run_in_parallel=True,
        )
        execution = _execute_oem_steps_live(
            list(plan.get("steps") or []),
            self,
            wait_timeout_s=float(timeout_s),
            speed=None,
            acc=None,
            pseudo_z_home_steps=int(pseudo_home_steps),
        )
        return {
            "ok": execution.get("ok") is True,
            "plan": _json_safe(plan),
            "execution": _json_safe(execution),
            "position_table_source": table.source,
            "machine_state": state.to_payload(),
            "source_anchor": "ClassControlInterface.scriptmoveTo:3734-4014; ControlLib.initializeMotion:8812",
        }


class Serial206OemInitializationProvider:
    """One durable expected-next stage per generation-bound approval."""

    source_mode = "ClassControlInterface.initializeMotors:3348-3421"
    schema = "bioxp.serial206_oem_initialization.v2"

    def __init__(
        self,
        primitives: Any,
        *,
        state_store: Any | None = None,
        reference_store: Any | None = None,
        generation_provider: Callable[[], int] | None = None,
        preparation_provider: Any | None = None,
        sleep: Callable[[float], None] | None = None,
        # Legacy constructor names are accepted only to keep old local callers
        # fail-closed while they migrate to the unified atomic state store.
        ledger_store: Any | None = None,
        approval_store: Any | None = None,
    ) -> None:
        del approval_store
        self.primitives = primitives
        self.state_store = state_store or ledger_store
        self.reference_store = reference_store
        self.generation_provider = generation_provider or (lambda: 0)
        self.preparation_provider = preparation_provider or primitives
        if sleep is None:
            import time
            sleep = time.sleep
        self.sleep = sleep
        self._lock = _MutationPriorityRLock()
        self._x_interrupt_state_lock = threading.Lock()
        self._x_interrupt_dispatch_lock = threading.Lock()
        self._x_interrupt_epoch = 0
        self._x_interrupt_active = False
        self._z_interrupt_state_lock = threading.Lock()
        self._z_interrupt_dispatch_lock = threading.Lock()
        self._z_interrupt_epoch = 0
        self._z_interrupt_active = False
        self._memory_state: dict[str, Any] | None = None
        self._board_transition_scope: dict[str, Any] | None = None

    @contextmanager
    def z_command_lease(self):
        """Give one Z command priority over status readers for its full composition."""
        with self._lock.mutation():
            yield

    @staticmethod
    def _new_z_lifecycle() -> dict[str, Any]:
        return {
            "schema_version": _Z_LIFECYCLE_SCHEMA,
            "state": "unprepared",
            "generation": None,
            "board_lifecycle_generation": None,
            "prepared_receipt": None,
            "active_receipt": None,
            "awaiting_observation_receipt_id": None,
            "reference_state": "unknown",
            "terminal_state": None,
            "last_failure": None,
            "receipts": [],
        }

    @staticmethod
    def _new_x_lifecycle() -> dict[str, Any]:
        return {"schema_version": _X_LIFECYCLE_SCHEMA, "state": "unprepared", "generation": None, "board_lifecycle_generation": None, "prepared_receipt": None, "active_receipt": None, "pending_ticket": None, "awaiting_observation_receipt_id": None, "reference_state": "unknown", "terminal_state": None, "last_failure": None, "receipts": []}

    @staticmethod
    def _new_state() -> dict[str, Any]:
        movement_ledger = new_initialize_motors_ledger()
        movement_ledger["stage_order"] = list(OEM_INITIALIZE_MOTORS_STAGE_KEYS)
        return {
            "schema_version": _STATE_SCHEMA,
            "movement_ledger": movement_ledger,
            "used_approvals": {},
            "used_motion_approvals": {},
            "z_lifecycle": Serial206OemInitializationProvider._new_z_lifecycle(),
            "x_lifecycle": Serial206OemInitializationProvider._new_x_lifecycle(),
            "initialize_motion_ledger": {
                "schema_version": _MOTION_LEDGER_SCHEMA,
                "source_anchor": "ControlLib.initializeMotion:8797-8856",
                "terminal_state": "not_started",
                "expected_next_stage": "initializeMotion.stop_scripts",
                "stage_receipts": [],
                "context": {
                    "tip_exists_initial": None,
                    "tip_channels_initial": [],
                    "tip_exists_after_eject": None,
                    "checked_status_initial": None,
                    "checked_status_retry": None,
                    "caught_exception": None,
                },
            },
            "machine_status": {
                "stop_scripts": None,
                "forceabort": None,
                "pause_scripts": None,
                "thermal_door_open": None,
                "tip_loaded": None,
                "tip_dirty": None,
                "tip_location": -1,
                "clean_path": False,
                "plate_on_gantry": None,
                "psudo_z_home_steps": 65000,
                "current_location": None,
                "current_well": None,
                "system_status": None,
                "initialization_complete": False,
                "calibrated_ui_positions": {"x": None, "y": None, "z": None},
                "error_events": [],
            },
            "preparation": {
                "generation": None,
                "state": "not_started",
                "receipt": None,
            },
        }

    @classmethod
    def _upgrade_state(cls, state: Any) -> Any:
        """Add exact initializeMotion host authority to pre-binding v2 records."""
        if not isinstance(state, dict) or state.get("schema_version") != _STATE_SCHEMA:
            return state
        upgraded = copy.deepcopy(state)
        defaults = cls._new_state()
        upgraded.setdefault("used_motion_approvals", {})
        upgraded.setdefault("z_lifecycle", copy.deepcopy(defaults["z_lifecycle"]))
        z_lifecycle = upgraded.get("z_lifecycle")
        if isinstance(z_lifecycle, dict):
            if z_lifecycle.get("schema_version") in _LEGACY_Z_LIFECYCLE_SCHEMAS:
                z_lifecycle["schema_version"] = _Z_LIFECYCLE_SCHEMA
            board_generation_missing = "board_lifecycle_generation" not in z_lifecycle
            legacy_authority_present = board_generation_missing and (
                z_lifecycle.get("state") != "unprepared"
                or z_lifecycle.get("generation") is not None
                or z_lifecycle.get("prepared_receipt") is not None
                or z_lifecycle.get("active_receipt") is not None
                or z_lifecycle.get("awaiting_observation_receipt_id") is not None
                or z_lifecycle.get("reference_state") == "referenced"
            )
            for key, value in defaults["z_lifecycle"].items():
                z_lifecycle.setdefault(key, copy.deepcopy(value))
            if legacy_authority_present:
                z_lifecycle.update({
                    "state": "unprepared",
                    "generation": None,
                    "board_lifecycle_generation": None,
                    "prepared_receipt": None,
                    "active_receipt": None,
                    "awaiting_observation_receipt_id": None,
                    "reference_state": "desynced",
                    "last_failure": {
                        "reason": "legacy_z_state_missing_board_lifecycle_generation",
                        "previous_state": state.get("z_lifecycle", {}).get("state"),
                    },
                })
        upgraded.setdefault("x_lifecycle", copy.deepcopy(defaults["x_lifecycle"]))
        x_lifecycle = upgraded.get("x_lifecycle")
        if isinstance(x_lifecycle, dict):
            legacy_x = x_lifecycle.get("schema_version") in _LEGACY_X_LIFECYCLE_SCHEMAS
            legacy_authority = legacy_x and (
                x_lifecycle.get("state") != "unprepared"
                or x_lifecycle.get("generation") is not None
                or x_lifecycle.get("prepared_receipt") is not None
                or x_lifecycle.get("reference_state") == "referenced"
            )
            if legacy_x:
                x_lifecycle["schema_version"] = _X_LIFECYCLE_SCHEMA
            for key, value in defaults["x_lifecycle"].items():
                x_lifecycle.setdefault(key, copy.deepcopy(value))
            if legacy_authority:
                x_lifecycle.update({
                    "state": "failed_latched",
                    "generation": None,
                    "board_lifecycle_generation": None,
                    "prepared_receipt": None,
                    "active_receipt": None,
                    "pending_ticket": None,
                    "awaiting_observation_receipt_id": None,
                    "reference_state": "desynced",
                    "last_failure": "legacy_x_state_missing_board_lifecycle_generation",
                })
        upgraded.setdefault("machine_status", copy.deepcopy(defaults["machine_status"]))
        machine = upgraded.get("machine_status")
        if isinstance(machine, dict):
            pseudo_home_missing = "psudo_z_home_steps" not in machine
            for key, value in defaults["machine_status"].items():
                machine.setdefault(key, copy.deepcopy(value))
            if pseudo_home_missing:
                machine["psudo_z_home_steps"] = 500 if machine.get("tip_loaded") is True else 65000
        motion = upgraded.get("initialize_motion_ledger")
        if isinstance(motion, dict):
            motion.setdefault("stage_receipts", [])
            motion.setdefault("expected_next_stage", _MOTION_STAGE_ORDER[0])
            motion.setdefault("context", copy.deepcopy(defaults["initialize_motion_ledger"]["context"]))
            context = motion.get("context")
            if isinstance(context, dict):
                for key, value in defaults["initialize_motion_ledger"]["context"].items():
                    context.setdefault(key, copy.deepcopy(value))
            if motion.get("terminal_state") == "unavailable_partial_binding" and not motion.get("stage_receipts"):
                motion["terminal_state"] = "not_started"
                motion["expected_next_stage"] = _MOTION_STAGE_ORDER[0]
        return upgraded

    @staticmethod
    def _validate_state(state: Any) -> dict[str, Any]:
        if not isinstance(state, dict) or state.get("schema_version") != _STATE_SCHEMA:
            raise ValueError("serial-206 state schema mismatch")
        used = state.get("used_approvals")
        if not isinstance(used, dict):
            raise ValueError("used approvals ledger is invalid")
        z_lifecycle = state.get("z_lifecycle")
        if not isinstance(z_lifecycle, dict) or z_lifecycle.get("schema_version") != _Z_LIFECYCLE_SCHEMA:
            raise ValueError("serial-206 Z lifecycle is invalid")
        board_generation = z_lifecycle.get("board_lifecycle_generation")
        if board_generation is not None and (
            type(board_generation) is not int or int(board_generation) < 1
        ):
            raise ValueError("serial-206 Z board lifecycle generation is invalid")
        if z_lifecycle.get("state") not in {
            "unprepared", "prepared_unreferenced", "executing",
            "awaiting_operator_observation", "referenced_ready", "failed_latched",
        }:
            raise ValueError("serial-206 Z lifecycle state is invalid")
        z_receipts = z_lifecycle.get("receipts")
        if not isinstance(z_receipts, list) or len(z_receipts) > 128:
            raise ValueError("serial-206 Z receipt ledger is invalid")
        if z_lifecycle.get("state") == "executing" and not isinstance(z_lifecycle.get("active_receipt"), Mapping):
            raise ValueError("executing serial-206 Z lifecycle lacks active receipt")
        if z_lifecycle.get("state") != "executing" and z_lifecycle.get("active_receipt") is not None:
            raise ValueError("inactive serial-206 Z lifecycle carries active receipt")
        x_lifecycle = state.get("x_lifecycle")
        if not isinstance(x_lifecycle, dict) or x_lifecycle.get("schema_version") != _X_LIFECYCLE_SCHEMA:
            raise ValueError("serial-206 X lifecycle is invalid")
        if x_lifecycle.get("state") not in {"unprepared", "prepared_unreferenced", "executing", "awaiting_operator_observation", "referenced_ready", "failed_latched"}:
            raise ValueError("serial-206 X lifecycle state is invalid")
        if not isinstance(x_lifecycle.get("receipts"), list) or len(x_lifecycle["receipts"]) > 128:
            raise ValueError("serial-206 X receipt ledger is invalid")
        if x_lifecycle.get("state") == "executing" and not isinstance(x_lifecycle.get("active_receipt"), Mapping):
            raise ValueError("executing serial-206 X lifecycle lacks active receipt")
        if x_lifecycle.get("state") != "executing" and x_lifecycle.get("active_receipt") is not None:
            raise ValueError("inactive serial-206 X lifecycle carries active receipt")
        x_board_generation = x_lifecycle.get("board_lifecycle_generation")
        if x_board_generation is not None and (type(x_board_generation) is not int or x_board_generation < 1):
            raise ValueError("serial-206 X board lifecycle generation is invalid")
        motion = state.get("initialize_motion_ledger")
        if not isinstance(motion, dict) or motion.get("schema_version") != _MOTION_LEDGER_SCHEMA:
            raise ValueError("initializeMotion ledger is invalid")
        motion_receipts = motion.get("stage_receipts")
        motion_context = motion.get("context")
        used_motion = state.get("used_motion_approvals")
        machine_status = state.get("machine_status")
        if not isinstance(motion_receipts, list) or not isinstance(motion_context, dict):
            raise ValueError("initializeMotion receipt/context authority is invalid")
        if not isinstance(used_motion, dict) or not isinstance(machine_status, dict):
            raise ValueError("initializeMotion approval/machine-state authority is invalid")
        pseudo_home = machine_status.get("psudo_z_home_steps")
        if type(pseudo_home) is not int or pseudo_home not in {500, 65000}:
            raise ValueError("PSUDO_Z_HOME state is invalid")
        terminal = motion.get("terminal_state")
        if terminal not in {"not_started", "running", "awaiting_next_stage", "initializeMotion_complete", "failed_closed"}:
            raise ValueError("initializeMotion terminal state is invalid")
        expected_motion = motion.get("expected_next_stage")
        if expected_motion is not None and expected_motion not in _MOTION_SPEC_BY_KEY:
            raise ValueError("initializeMotion expected-next stage is invalid")
        seen_motion_stages: set[str] = set()
        active_motion = 0
        for receipt in motion_receipts:
            if not isinstance(receipt, dict) or receipt.get("stage") not in _MOTION_SPEC_BY_KEY:
                raise ValueError("initializeMotion receipt is invalid")
            selected_stage = str(receipt["stage"])
            if selected_stage in seen_motion_stages:
                raise ValueError("initializeMotion stage receipt was duplicated")
            seen_motion_stages.add(selected_stage)
            if receipt.get("status") == "admitted":
                active_motion += 1
        if active_motion > 1:
            raise ValueError("multiple active initializeMotion stages")
        if terminal == "initializeMotion_complete" and expected_motion is not None:
            raise ValueError("completed initializeMotion carries expected-next stage")
        if terminal == "failed_closed" and expected_motion is not None:
            raise ValueError("failed initializeMotion carries expected-next stage")
        if terminal not in {"initializeMotion_complete", "failed_closed"} and expected_motion is None:
            raise ValueError("active initializeMotion lacks expected-next stage")
        error_events = machine_status.get("error_events")
        if not isinstance(error_events, list) or len(error_events) > 128:
            raise ValueError("initializeMotion error-event state is invalid")
        prep = state.get("preparation")
        if not isinstance(prep, dict) or prep.get("state") not in {"not_started", "completed", "failed_closed"}:
            raise ValueError("preparation state is invalid")
        if prep.get("state") == "not_started":
            if prep.get("generation") is not None or prep.get("receipt") is not None:
                raise ValueError("unstarted preparation carries evidence")
        elif type(prep.get("generation")) is not int or not isinstance(prep.get("receipt"), Mapping):
            raise ValueError("prepared generation/evidence is invalid")

        ledger = state.get("movement_ledger")
        if not isinstance(ledger, dict) or ledger.get("schema_version") != SERIAL206_INITIALIZE_MOTORS_LEDGER_SCHEMA:
            raise ValueError("movement ledger schema mismatch")
        rows = ledger.get("stages")
        order = list(OEM_INITIALIZE_MOTORS_STAGE_KEYS)
        if not isinstance(rows, dict) or set(rows) != set(order) or ledger.get("stage_order") != order:
            raise ValueError("movement ledger stage-order authority mismatch")
        valid_states = {"pending", "admitted", "acknowledged", "completed", "operator_observed", "failed"}
        for key, row in rows.items():
            if not isinstance(row, dict) or row.get("stage") != key or row.get("state") not in valid_states:
                raise ValueError(f"movement ledger row is invalid: {key}")

        def successful(key: str) -> bool:
            row = rows[key]
            if row.get("requires_operator_observation") is True:
                return row.get("state") == "operator_observed"
            return row.get("state") == "completed"

        first_incomplete = next((key for key in order if not successful(key)), None)
        expected = ledger.get("expected_next_stage")
        if expected != first_incomplete:
            raise ValueError("terminal/expected-next lifecycle inconsistency")
        if first_incomplete is None:
            if ledger.get("terminal_state") != "initializeMotors_complete":
                raise ValueError("complete ledger has non-complete terminal state")
        else:
            index = order.index(first_incomplete)
            if any(rows[key].get("state") != "pending" for key in order[index + 1 :]):
                raise ValueError("impossible out-of-order stage state")
            selected_state = rows[first_incomplete].get("state")
            terminal_by_state = {
                "admitted": {"running"},
                "acknowledged": {"awaiting_operator_observation"},
                "failed": {"failed_closed"},
                "pending": {"not_started", "awaiting_next_stage"},
            }
            if ledger.get("terminal_state") not in terminal_by_state.get(selected_state, set()):
                raise ValueError("terminal state does not match expected-next row")
        active = [key for key in order if rows[key].get("state") in {"admitted", "acknowledged"}]
        if len(active) > 1:
            raise ValueError("multiple active lifecycle stages")

        preparation_generation = prep.get("generation")
        if used and (prep.get("state") != "completed" or type(preparation_generation) is not int):
            raise ValueError("used approval without completed preparation")
        for approval_id, approval_row in used.items():
            if not isinstance(approval_id, str) or not approval_id or not isinstance(approval_row, Mapping):
                raise ValueError("used approval row is invalid")
            stage = approval_row.get("stage")
            if stage not in rows or approval_row.get("generation") != preparation_generation:
                raise ValueError("used approval generation/stage mismatch")
            stage_row = rows[stage]
            if (
                stage_row.get("command_id") != approval_id
                or stage_row.get("idempotency_key") != approval_row.get("idempotency_key")
                or stage_row.get("expected_generation") != approval_row.get("generation")
                or stage_row.get("state") == "pending"
            ):
                raise ValueError("used approval identity mismatch")
        for key in order:
            row = rows[key]
            command_id = row.get("command_id")
            if row.get("state") != "pending" and (not isinstance(command_id, str) or command_id not in used):
                raise ValueError("executed row lacks matching used approval")

        json.dumps(state, allow_nan=False)
        return copy.deepcopy(state)

    def _load_state(self) -> dict[str, Any]:
        if self.state_store is not None and hasattr(self.state_store, "read_oem_serial206_initialization_state"):
            stored = self.state_store.read_oem_serial206_initialization_state()
            if stored is None:
                return self._new_state()
            stored_z = stored.get("z_lifecycle") if isinstance(stored, Mapping) else None
            stored_z_mapping = dict(stored_z) if isinstance(stored_z, Mapping) else {}
            board_generation_missing = (
                isinstance(stored_z, Mapping)
                and "board_lifecycle_generation" not in stored_z
            )
            migration_required = bool(
                board_generation_missing
                or stored_z_mapping.get("schema_version") in _LEGACY_Z_LIFECYCLE_SCHEMAS
            )
            legacy_authority_present = board_generation_missing and (
                stored_z_mapping.get("state") != "unprepared"
                or stored_z_mapping.get("generation") is not None
                or stored_z_mapping.get("prepared_receipt") is not None
                or stored_z_mapping.get("active_receipt") is not None
                or stored_z_mapping.get("awaiting_observation_receipt_id") is not None
                or stored_z_mapping.get("reference_state") == "referenced"
            )
            payload = self._validate_state(self._upgrade_state(stored))
            if migration_required:
                if legacy_authority_present:
                    self._z_mark_desynced(
                        "Legacy serial-206 Z authority lacked board lifecycle generation and was invalidated.",
                        "serial206.z.state_migration",
                    )
                self.state_store.write_oem_serial206_initialization_state(payload)
                self._memory_state = copy.deepcopy(payload)
            z_lifecycle = payload["z_lifecycle"]
            active_receipt = (
                z_lifecycle.get("active_receipt")
                if isinstance(z_lifecycle.get("active_receipt"), Mapping)
                else {}
            )
            scope = self._board_transition_scope
            same_process_board_transition = bool(
                isinstance(scope, Mapping)
                and scope.get("thread_id") == threading.get_ident()
                and scope.get("command_id") == active_receipt.get("command_id")
                and active_receipt.get("intent") == "prepare"
            )
            if z_lifecycle.get("state") == "executing" and not same_process_board_transition:
                self._z_mark_desynced(
                    "Interrupted serial-206 Z transaction had an ambiguous outcome and was invalidated.",
                    "serial206.z.interrupted_transaction_recovery",
                )
                z_lifecycle.update(
                    {
                        "state": "failed_latched",
                        "reference_state": "desynced",
                        "active_receipt": None,
                        "awaiting_observation_receipt_id": None,
                        "last_failure": "interrupted_z_transaction_outcome_ambiguous",
                    }
                )
                payload = self._save_state(payload)
            x_lifecycle = payload["x_lifecycle"]
            if x_lifecycle.get("state") == "executing":
                if self.reference_store is not None:
                    desync = getattr(self.primitives, "_x_desync", None)
                    if callable(desync):
                        desync(
                            "Interrupted serial-206 X transaction had an ambiguous outcome and was invalidated.",
                            "restart_reconciliation",
                        )
                x_lifecycle.update({
                    "state": "failed_latched",
                    "reference_state": "desynced",
                    "active_receipt": None,
                    "pending_ticket": None,
                    "awaiting_observation_receipt_id": None,
                    "last_failure": "interrupted_x_transaction_outcome_ambiguous",
                })
                payload = self._save_state(payload)
            return payload
        if self._memory_state is None:
            self._memory_state = self._new_state()
        return self._validate_state(self._upgrade_state(self._memory_state))

    def _save_state(self, state: Mapping[str, Any]) -> dict[str, Any]:
        payload = self._validate_state(self._upgrade_state(copy.deepcopy(dict(state))))
        if self.state_store is not None and hasattr(self.state_store, "write_oem_serial206_initialization_state"):
            self.state_store.write_oem_serial206_initialization_state(payload)
        elif self.state_store is not None:
            raise RuntimeError("unified atomic serial-206 state store is not available")
        self._memory_state = copy.deepcopy(payload)
        return payload

    def _durable_serial206_receipt(self, stream: str, command_id: str) -> dict[str, Any] | None:
        if self.state_store is None or not hasattr(self.state_store, "read_serial206_receipt"):
            return None
        row = self.state_store.read_serial206_receipt(stream, command_id)
        return dict(row) if isinstance(row, Mapping) else None

    def _durable_serial206_receipt_by_idempotency(self, stream: str, key: str) -> dict[str, Any] | None:
        if self.state_store is None or not hasattr(self.state_store, "read_serial206_receipt_by_idempotency"):
            return None
        row = self.state_store.read_serial206_receipt_by_idempotency(stream, key)
        return dict(row) if isinstance(row, Mapping) else None

    def _corrupt_projection(self) -> dict[str, Any]:
        return {
            "schema_version": SERIAL206_INITIALIZE_MOTORS_LEDGER_SCHEMA,
            "expected_next_stage": None,
            "terminal_state": "failed_closed",
            "compatibility_blocker": "durable_serial206_state_corrupt",
            "stages": {},
        }

    def projection(self) -> dict[str, Any]:
        with self._lock:
            try:
                return copy.deepcopy(self._load_state()["movement_ledger"])
            except Exception:
                return self._corrupt_projection()

    def x_projection(self) -> dict[str, Any]:
        with self._lock:
            try:
                state = self._load_state()
                source_lifecycle = state["x_lifecycle"]
                current_generation = int(self.generation_provider())
                lifecycle_generation = source_lifecycle.get("generation")
                prepared_board_generation = source_lifecycle.get("board_lifecycle_generation")
                board_generation_provider = getattr(self.preparation_provider, "current_board_lifecycle_generation", None)
                current_board_generation = (
                    board_generation_provider()
                    if callable(board_generation_provider)
                    else prepared_board_generation
                )
                generation_drift = bool(isinstance(lifecycle_generation, int) and lifecycle_generation != current_generation)
                board_drift = bool(
                    source_lifecycle.get("state") != "unprepared"
                    and (
                        type(prepared_board_generation) is not int
                        or prepared_board_generation != current_board_generation
                    )
                )
                if (generation_drift or board_drift) and source_lifecycle.get("state") != "unprepared":
                    reason = "x_generation_changed" if generation_drift else "x_board_lifecycle_generation_changed"
                    desync = getattr(self.primitives, "_x_desync", None)
                    if callable(desync):
                        desync(f"Serial-206 X authority invalidated: {reason}", "projection_invalidation")
                    source_lifecycle.update({
                        "state": "unprepared",
                        "generation": None,
                        "board_lifecycle_generation": None,
                        "prepared_receipt": None,
                        "reference_state": "desynced",
                        "active_receipt": None,
                        "pending_ticket": None,
                        "awaiting_observation_receipt_id": None,
                        "last_failure": {
                            "failure": reason,
                            "recorded_generation": lifecycle_generation,
                            "current_generation": current_generation,
                            "recorded_board_lifecycle_generation": prepared_board_generation,
                            "current_board_lifecycle_generation": current_board_generation,
                        },
                    })
                    self._save_state(state)
                lifecycle = {
                    str(key): _json_safe(value)
                    for key, value in source_lifecycle.items()
                    if key not in {"receipts", "receipts_omitted_to_sqlite"}
                }
                receipts = [
                    row for row in list(source_lifecycle.get("receipts") or [])
                    if isinstance(row, Mapping)
                ]
                latest = receipts[-1] if receipts else None
                lifecycle.update({
                    "receipt_storage": "robot_sqlite",
                    "receipt_detail_on_request": True,
                    "recent_receipt_count": len(receipts),
                    "latest_receipt": None if latest is None else {
                        key: _json_safe(latest.get(key))
                        for key in ("command_id", "intent", "status")
                    },
                })
                reference = self.reference_store.snapshot(("x",)) if self.reference_store is not None else {"ok": False, "authority_untrusted": True}
                live_status_fn = getattr(self.primitives, "x_terminal_status", None)
                try:
                    live_status = live_status_fn() if callable(live_status_fn) else {"ok": False, "failure": "x_terminal_status_not_bound"}
                except Exception as exc:
                    live_status = {"ok": False, "failure": f"x_terminal_status_failed:{type(exc).__name__}:{exc}"}
                return {"authority": type(self).__name__, "axis": "x", "board": 5, "motor": 0, "source_min_steps": 0, "source_max_steps": 90263, "effective_absolute_min_steps": 60, "relative_limit_margin_steps": 20, "current_generation": current_generation, "current_board_lifecycle_generation": current_board_generation, "board_generation_fresh": type(prepared_board_generation) is int and prepared_board_generation == current_board_generation, "lifecycle": lifecycle, "live_status": _json_contract_safe(live_status), "switch_masks": {"expected": dict(SERIAL206_X_SWITCH_MASKS), "verified": isinstance(live_status, Mapping) and live_status.get("switch_mask_verified") is True}, "profile": {"expected": {"4": 1700, "5": 350, "6": 31, "205": 16}, "verified": isinstance(live_status, Mapping) and live_status.get("profile_verified") is True}, "reference": _json_safe(reference)}
            except Exception as exc:
                return {"ok": False, "axis": "x", "state": "failed_latched", "failure": f"projection_failed:{type(exc).__name__}"}

    def execute_x_stop_interrupt(
        self,
        values: Mapping[str, Any] | None = None,
        *,
        abort: bool = False,
    ) -> dict[str, Any]:
        values = dict(values or {})
        selected = "abort" if abort else "stop"
        supplied_command_id = values.get("command_id")
        command_id = (
            supplied_command_id
            if isinstance(supplied_command_id, str) and supplied_command_id.strip()
            else f"x-{selected}-{time.time_ns()}"
        )
        idempotency_key = values.get("idempotency_key")
        idempotency_key = idempotency_key if isinstance(idempotency_key, str) and idempotency_key else None
        safe_inputs = _json_safe(values)
        with self._x_interrupt_dispatch_lock:
            with self._x_interrupt_state_lock:
                self._x_interrupt_epoch += 1
                interrupt_epoch = self._x_interrupt_epoch
                self._x_interrupt_active = True
            started_at = time.time()
            try:
                snapshot_active: Mapping[str, Any] | None = None
                try:
                    if self.state_store is not None and hasattr(
                        self.state_store, "read_oem_serial206_initialization_state"
                    ):
                        stored = self.state_store.read_oem_serial206_initialization_state()
                        snapshot = self._validate_state(self._upgrade_state(stored or self._new_state()))
                        candidate = snapshot.get("x_lifecycle", {}).get("active_receipt")
                        if isinstance(candidate, Mapping):
                            snapshot_active = copy.deepcopy(candidate)
                except Exception:
                    snapshot_active = None
                try:
                    raw_result = (
                        self.primitives.x_abort(reason=str(values.get("reason") or "forceAbortMotion"))
                        if abort
                        else self.primitives.x_stop(timeout_s=float(values.get("timeout_s", 3.0)))
                    )
                except Exception as exc:
                    raw_result = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
                result = dict(raw_result) if isinstance(raw_result, Mapping) else {
                    "ok": False,
                    "failure": f"x_{selected}_result_not_mapping",
                }
                with self._lock:
                    state = self._load_state()
                    lifecycle = state["x_lifecycle"]
                    current_active = lifecycle.get("active_receipt")
                    interrupted_ids = {
                        str(row["command_id"])
                        for row in (snapshot_active, current_active)
                        if isinstance(row, Mapping) and row.get("command_id")
                    }
                    generation = int(self.generation_provider())
                    result.update({
                        "interrupt_epoch": interrupt_epoch,
                        "interrupted_command_ids": sorted(interrupted_ids),
                    })
                    receipt = {
                        "command_id": str(command_id),
                        "receipt_id": f"{command_id}:{time.time_ns()}",
                        "intent": selected,
                        "idempotency_key": idempotency_key,
                        "idempotency_replay_enabled": False,
                        "generation": generation,
                        "inputs": safe_inputs,
                        "status": "completed" if result.get("ok") is True else "failed",
                        "started_at": started_at,
                        "finished_at": time.time(),
                        "interrupt_epoch": interrupt_epoch,
                        "interrupted_command_ids": sorted(interrupted_ids),
                        "result": _json_safe(result),
                    }
                    lifecycle.update({
                        "state": "failed_latched",
                        "generation": generation,
                        "active_receipt": None,
                        "pending_ticket": None,
                        "reference_state": "desynced",
                        "last_failure": {
                            "reason": f"x_safety_{selected}_dispatched",
                            "receipt": _json_safe(receipt),
                        },
                    })
                    lifecycle["receipts"].append(receipt)
                    lifecycle["receipts"] = lifecycle["receipts"][-8:]
                    self._save_state(state)
                    if self.state_store is not None and hasattr(
                        self.state_store, "append_serial206_interrupt_receipt"
                    ):
                        self.state_store.append_serial206_interrupt_receipt("x", receipt)
                    return {
                        "ok": result.get("ok") is True,
                        "axis": "x",
                        "intent": selected,
                        "state": "failed_latched",
                        "result": _json_safe(result),
                        "authority_receipt": _json_safe(receipt),
                    }
            finally:
                with self._x_interrupt_state_lock:
                    self._x_interrupt_active = False

    def execute_x_intent(self, intent: str, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = dict(values or {})
        selected = str(intent).strip().lower()
        if selected == "diagnostic_home_axis":
            selected = "home_axis"
        if selected == "observe":
            return self.record_x_observation(
                command_id=str(values.get("observed_command_id") or ""),
                observation_command_id=str(values.get("command_id") or "") or None,
                verdict=str(values.get("verdict") or ""),
                physical_motion_observed=values["physical_motion_observed"],
                expected_direction_observed=values["expected_direction_observed"],
                home_endpoint_observed=values["home_endpoint_observed"],
                stopped_observed=values["stopped_observed"],
                note=str(values.get("note") or ""),
                expected_generation=int(values.get("expected_generation", self.generation_provider())),
            )
        if selected in {"stop", "abort"}:
            return self.execute_x_stop_interrupt(values, abort=selected == "abort")
        with self._x_interrupt_state_lock:
            admitted_interrupt_epoch = self._x_interrupt_epoch
            if self._x_interrupt_active:
                return {"ok": False, "axis": "x", "failure": "x_safety_interrupt_in_progress"}
        with self._lock:
            with self._x_interrupt_state_lock:
                if self._x_interrupt_active or admitted_interrupt_epoch != self._x_interrupt_epoch:
                    return {"ok": False, "axis": "x", "failure": "x_intent_superseded_by_safety_interrupt"}
            try:
                state = self._load_state()
            except Exception as exc:
                return {"ok": False, "axis": "x", "state": "failed_latched", "failure": f"durable_state_unavailable:{exc}"}
            lifecycle = state["x_lifecycle"]
            z_lifecycle_value = state.get("z_lifecycle")
            z_lifecycle: dict[str, Any] = z_lifecycle_value if isinstance(z_lifecycle_value, dict) else {}
            generation = int(self.generation_provider())
            if type(values.get("expected_generation")) is not int or values.get("expected_generation") != generation:
                return {
                    "ok": False,
                    "axis": "x",
                    "state": lifecycle.get("state"),
                    "failure": "x_expected_generation_mismatch",
                    "expected_generation": values.get("expected_generation"),
                    "current_generation": generation,
                    "physical_motion_commanded": False,
                }
            interrupt = False
            idempotency_key = values.get("idempotency_key")
            idempotency_key = idempotency_key if isinstance(idempotency_key, str) and idempotency_key else None
            default_command_id = (
                f"x-{selected}-{generation}-{time.time_ns()}"
                if interrupt
                else idempotency_key or f"x-{selected}-{generation}"
            )
            command_id = str(values.get("command_id") or default_command_id)
            safe_inputs = _json_safe(values)
            if not interrupt:
                existing = next(
                    (
                        row for row in reversed(lifecycle.get("receipts") or [])
                        if isinstance(row, Mapping)
                        and (row.get("command_id") == command_id or (
                            idempotency_key is not None and row.get("idempotency_key") == idempotency_key
                        ))
                    ),
                    None,
                )
                if existing is None:
                    existing = self._durable_serial206_receipt("x", command_id)
                if existing is None and idempotency_key is not None:
                    existing = self._durable_serial206_receipt_by_idempotency("x", idempotency_key)
                if isinstance(existing, Mapping):
                    if (
                        existing.get("intent") != selected
                        or existing.get("inputs") != safe_inputs
                        or existing.get("generation") != generation
                    ):
                        return {
                            "ok": False,
                            "axis": "x",
                            "state": lifecycle.get("state"),
                            "failure": "x_replay_authority_or_request_mismatch",
                            "replayed": True,
                            "authority_receipt": _json_safe(existing),
                        }
                    replay_requires_reference = selected != "prepare"
                    replay_authority_valid = (
                        lifecycle.get("generation") == generation
                        and lifecycle.get("state") not in {"unprepared", "failed_latched", "executing"}
                        and (
                            not replay_requires_reference
                            or (
                                lifecycle.get("state") == "referenced_ready"
                                and lifecycle.get("reference_state") == "referenced"
                            )
                        )
                    )
                    if existing.get("status") == "completed" and selected == "enable_xyz_current":
                        board_generation_fn = getattr(self.preparation_provider, "current_board_lifecycle_generation", None)
                        current_board_generation_for_replay = board_generation_fn() if callable(board_generation_fn) else z_lifecycle.get("board_lifecycle_generation")
                        z_existing = self._durable_serial206_receipt("z", command_id)
                        replay_authority_valid = bool(
                            replay_authority_valid
                            and isinstance(z_existing, Mapping)
                            and z_existing.get("status") == "completed"
                            and z_existing.get("intent") == "enable_xyz_current"
                            and z_existing.get("inputs") == safe_inputs
                            and z_existing.get("x_generation") == generation
                            and z_existing.get("z_generation") == z_lifecycle.get("generation") == generation
                            and z_existing.get("z_board_lifecycle_generation") == z_lifecycle.get("board_lifecycle_generation") == current_board_generation_for_replay
                            and z_existing.get("z_state") == z_lifecycle.get("state")
                            and z_existing.get("z_reference_state") == z_lifecycle.get("reference_state")
                            and z_lifecycle.get("state") in {"prepared_unreferenced", "referenced_ready"}
                            and z_lifecycle.get("reference_state") in {"desynced", "referenced"}
                            and z_lifecycle.get("active_receipt") is None
                            and z_lifecycle.get("awaiting_observation_receipt_id") is None
                        )
                    if existing.get("status") == "completed" and not replay_authority_valid:
                        return {
                            "ok": False,
                            "axis": "x",
                            "state": lifecycle.get("state"),
                            "failure": "x_replay_current_authority_invalid",
                            "replayed": True,
                            "authority_receipt": _json_safe(existing),
                        }
                    replayed_result = dict(existing.get("result") or {})
                    replayed_result.setdefault("ok", existing.get("status") == "completed")
                    replayed_result.update({"replayed": True, "authority_receipt": _json_safe(existing)})
                    return replayed_result
            if lifecycle.get("state") == "executing" and selected not in {"wait_for_motor", "stop", "abort"}:
                lifecycle.update({"state": "failed_latched", "reference_state": "desynced", "last_failure": "restart_or_reentry_during_executing", "active_receipt": None, "pending_ticket": None})
                try: self._save_state(state)
                except Exception: pass
                return {"ok": False, "axis": "x", "state": "failed_latched", "failure": "x_executing_outcome_ambiguous"}
            active = lifecycle.get("active_receipt")
            if isinstance(active, Mapping) and active.get("command_id") == command_id and selected not in {"wait_for_motor", "stop", "abort"}:
                return copy.deepcopy(dict(active.get("result") or active))
            prior_state = str(lifecycle.get("state") or "unprepared")
            prior_reference_state = str(lifecycle.get("reference_state") or "unknown")
            prior_z_state = str(z_lifecycle.get("state") or "unprepared")
            prior_z_reference_state = str(z_lifecycle.get("reference_state") or "unknown")
            current_board_generation_fn = getattr(self.preparation_provider, "current_board_lifecycle_generation", None)
            current_board_generation = (
                current_board_generation_fn()
                if callable(current_board_generation_fn)
                else lifecycle.get("board_lifecycle_generation")
            )
            automatic_prerequisites: list[dict[str, Any]] = []
            if selected == "manual_panel_home" and prior_state == "unprepared":
                prepare_fn = getattr(self.primitives, "prepare_x", None) or getattr(
                    self.primitives, "prepare_for_initialize_motors"
                )
                try:
                    preparation = prepare_fn(expected_generation=generation)
                except Exception as exc:
                    preparation = {
                        "ok": False,
                        "failure": f"x_automatic_prepare_exception:{type(exc).__name__}:{exc}",
                    }
                preparation = dict(preparation) if isinstance(preparation, Mapping) else {
                    "ok": False,
                    "failure": "x_automatic_prepare_result_not_mapping",
                }
                prepared_board_generation = (
                    current_board_generation_fn()
                    if callable(current_board_generation_fn)
                    else preparation.get("board_lifecycle_generation")
                )
                prepare_ok = bool(
                    preparation.get("ok") is True
                    and preparation.get("physical_motion") is False
                    and type(prepared_board_generation) is int
                )
                prerequisite = {
                    "stage": "auto_prepare",
                    "physical_motion_commanded": False,
                    "result": _json_safe(preparation),
                }
                automatic_prerequisites.append(prerequisite)
                if not prepare_ok:
                    lifecycle.update({
                        "state": "failed_latched",
                        "generation": generation,
                        "board_lifecycle_generation": None,
                        "prepared_receipt": _json_safe(preparation),
                        "reference_state": "unknown",
                        "last_failure": _json_safe(preparation),
                    })
                    self._save_state(state)
                    return {
                        "ok": False,
                        "axis": "x",
                        "intent": selected,
                        "state": lifecycle["state"],
                        "failure": "x_automatic_prerequisite_failed",
                        "failed_stage": "auto_prepare",
                        "requested_motion_dispatched": False,
                        "automatic_prerequisites": automatic_prerequisites,
                        "generation": generation,
                    }
                lifecycle.update({
                    "state": "prepared_unreferenced",
                    "generation": generation,
                    "board_lifecycle_generation": prepared_board_generation,
                    "prepared_receipt": _json_safe(preparation),
                    "reference_state": "desynced",
                    "last_failure": None,
                })
                self._save_state(state)
                prior_state = "prepared_unreferenced"
                prior_reference_state = "desynced"
                current_board_generation = prepared_board_generation
            if selected == "observe_home":
                expected_receipt = lifecycle.get("awaiting_observation_receipt_id")
                observed_receipt = values.get("receipt_id")
                confirmed = values.get("confirmed") is True
                if prior_state != "awaiting_operator_observation" or not isinstance(expected_receipt, str) or observed_receipt != expected_receipt:
                    return {"ok": False, "axis": "x", "state": prior_state, "failure": "x_home_observation_authority_mismatch", "expected_receipt_id": expected_receipt}
                if not confirmed:
                    desync = getattr(self.primitives, "_x_desync", None)
                    invalidation = desync("Operator rejected serial-206 X home observation.", "operator_observation") if callable(desync) else None
                    lifecycle.update({"state": "failed_latched", "reference_state": "desynced", "awaiting_observation_receipt_id": None, "last_failure": "operator_rejected_x_home"})
                    self._save_state(state)
                    return {"ok": False, "axis": "x", "state": "failed_latched", "failure": "operator_rejected_x_home", "reference_invalidation": _json_safe(invalidation)}
                if self.reference_store is None:
                    return {"ok": False, "axis": "x", "state": prior_state, "failure": "x_reference_store_not_bound"}
                reference = self.reference_store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=int(values.get("position_steps", 0)), source="serial206.x.operator_observation", motion_kind="home"))
                accepted = bool(isinstance(reference, Mapping) and reference.get("ok") is True and reference.get("durable_clean") is True)
                lifecycle.update({"state": "referenced_ready" if accepted else "failed_latched", "reference_state": "referenced" if accepted else "desynced", "awaiting_observation_receipt_id": None, "last_failure": None if accepted else _json_safe(reference)})
                observation = {"ok": accepted, "axis": "x", "intent": "observe_home", "observed_receipt_id": observed_receipt, "confirmed": True, "reference_state": _json_safe(reference), "physical_motion_commanded": False, "physical_effect_verified": True, "failure": None if accepted else "x_reference_publication_failed"}
                lifecycle["receipts"].append({"command_id": command_id, "receipt_id": command_id, "intent": selected, "idempotency_key": idempotency_key, "generation": generation, "inputs": safe_inputs, "status": "completed" if accepted else "failed", "result": _json_safe(observation)})
                lifecycle["receipts"] = lifecycle["receipts"][-8:]
                self._save_state(state)
                return {"ok": accepted, "axis": "x", "intent": selected, "state": lifecycle["state"], "result": observation, "generation": generation}
            if selected == "prepare":
                prepare_fn = getattr(self.primitives, "prepare_x", None) or getattr(
                    self.primitives, "prepare_for_initialize_motors"
                )
                result = prepare_fn(expected_generation=generation)
                ok = isinstance(result, Mapping) and result.get("ok") is True and result.get("physical_motion") is False
                board_generation_provider = getattr(self.preparation_provider, "current_board_lifecycle_generation", None)
                prepared_board_generation = (
                    board_generation_provider()
                    if callable(board_generation_provider)
                    else result.get("board_lifecycle_generation")
                    if isinstance(result, Mapping)
                    else None
                )
                ok = bool(ok and type(prepared_board_generation) is int)
                lifecycle.update({"state": "prepared_unreferenced" if ok else "failed_latched", "generation": generation, "board_lifecycle_generation": prepared_board_generation if ok else None, "prepared_receipt": _json_safe(result), "reference_state": "desynced" if ok else "unknown", "last_failure": None if ok else _json_safe(result)})
                self._save_state(state)
                return {"ok": ok, "axis": "x", "intent": selected, "state": lifecycle["state"], "result": _json_safe(result), "generation": generation, "board_lifecycle_generation": prepared_board_generation}
            passive_intents = {"terminal_status", "reconcile_switch_masks"}
            profile_intents = {
                "set_max_speed", "set_max_acc", "restore_original_speed", "set_stall_guard",
                "enable_xy_current", "enable_xyz_current",
            }
            motion_intents = {"move_absolute", "move_steps", "wait_for_motor", "move_to"}
            home_intents = {"startup_home", "home_axis", "manual_panel_home", "move_to_origin_home", "caught_plate_recovery_home", "set_home"}
            move_to_all_zero = bool(
                selected == "move_to"
                and all(int(values.get(key, 0)) == 0 for key in ("x", "y", "z"))
            )
            if selected not in passive_intents:
                if type(lifecycle.get("board_lifecycle_generation")) is not int or lifecycle.get("board_lifecycle_generation") != current_board_generation:
                    return {"ok": False, "axis": "x", "state": prior_state, "failure": "x_board_lifecycle_authority_not_current"}
            if selected in profile_intents and prior_state not in {"prepared_unreferenced", "referenced_ready"}:
                return {"ok": False, "axis": "x", "state": prior_state, "failure": "x_not_prepared"}
            if selected == "enable_xyz_current":
                z_authority_current = bool(
                    z_lifecycle.get("generation") == generation
                    and type(z_lifecycle.get("board_lifecycle_generation")) is int
                    and z_lifecycle.get("board_lifecycle_generation") == current_board_generation
                    and prior_z_state in {"prepared_unreferenced", "referenced_ready"}
                    and prior_z_reference_state in {"desynced", "referenced"}
                    and isinstance(z_lifecycle.get("prepared_receipt"), Mapping)
                    and z_lifecycle.get("active_receipt") is None
                    and z_lifecycle.get("awaiting_observation_receipt_id") is None
                )
                if not z_authority_current:
                    return {
                        "ok": False,
                        "axis": "xyz",
                        "state": prior_state,
                        "z_state": prior_z_state,
                        "failure": "xyz_current_requires_current_x_and_z_authority",
                        "generation": generation,
                        "board_lifecycle_generation": current_board_generation,
                    }
            if selected in motion_intents and not move_to_all_zero and (prior_state != "referenced_ready" or prior_reference_state != "referenced"):
                return {"ok": False, "axis": "x", "state": prior_state, "failure": "x_reference_required_before_move"}
            if move_to_all_zero and prior_state not in {"prepared_unreferenced", "referenced_ready", "failed_latched"}:
                return {"ok": False, "axis": "x", "state": prior_state, "failure": "x_not_prepared_for_move_to_origin"}
            if selected in home_intents and prior_state not in {"prepared_unreferenced", "referenced_ready", "failed_latched"}:
                return {"ok": False, "axis": "x", "state": prior_state, "failure": "x_not_prepared_for_home"}
            if selected == "set_home" and values.get("operator_ack") != "SET_HOME_CURRENT_POSITION":
                return {"ok": False, "axis": "x", "state": prior_state, "failure": "explicit_confirmation_required:SET_HOME_CURRENT_POSITION"}
            if selected == "wait_for_motor":
                pending = lifecycle.get("pending_ticket")
                if not isinstance(pending, Mapping):
                    return {"ok": False, "axis": "x", "failure": "x_pending_ticket_missing", "state": lifecycle.get("state")}
                result = self.primitives.x_wait_for_motor(pending_ticket=pending, wait_timeout_s=float(values.get("wait_timeout_s", 20.0)))
            elif selected == "reconcile_switch_masks":
                prior = str(lifecycle.get("state") or "unprepared")
                lifecycle.update({"state": "unprepared", "generation": None, "board_lifecycle_generation": None, "prepared_receipt": None, "active_receipt": None, "pending_ticket": None, "awaiting_observation_receipt_id": None, "reference_state": "desynced", "last_failure": "x_switch_mask_reconciliation_admitted"})
                desync = getattr(self.primitives, "_x_desync", None)
                if callable(desync):
                    desync("Serial-206 X switch-mask reconciliation invalidated prior authority.", "switch_mask_reconciliation")
                self._save_state(state)
                result = self.primitives.x_reconcile_switch_masks()
                result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "x_mask_reconciliation_result_not_mapping"}
                result["prior_lifecycle_state"] = prior
                result["fresh_prepare_required"] = True
            else:
                active_receipt = {"command_id": command_id, "intent": selected, "idempotency_key": idempotency_key, "generation": generation, "inputs": safe_inputs, "status": "executing", "result": None}
                lifecycle.update({"state": "executing", "generation": generation, "active_receipt": active_receipt, "pending_ticket": None})
                if selected == "enable_xyz_current":
                    z_active_receipt = {
                        **copy.deepcopy(active_receipt),
                        "stream": "z",
                        "x_generation": generation,
                        "z_generation": z_lifecycle.get("generation"),
                        "z_board_lifecycle_generation": z_lifecycle.get("board_lifecycle_generation"),
                        "z_state": prior_z_state,
                        "z_reference_state": prior_z_reference_state,
                    }
                    z_lifecycle.update({"state": "executing", "active_receipt": z_active_receipt})
                self._save_state(state)
                try:
                    if selected == "move_absolute":
                        result = self.primitives.x_move_absolute(position_steps=int(values["position_steps"]), acceleration=None if values.get("acceleration") is None else int(values["acceleration"]), wait_for_stop=bool(values.get("wait_for_stop", True)), wait_timeout_s=float(values.get("wait_timeout_s", 20.0)), source_mode=str(values.get("source_mode") or "provider.x.move_absolute"))
                    elif selected == "move_steps":
                        result = self.primitives.x_move_steps(steps=int(values["steps"]), wait_timeout_s=float(values.get("wait_timeout_s", 20.0)))
                    elif selected == "move_to":
                        result = self.primitives.oem_move_to(
                            x=int(values["x"]), y=int(values["y"]), z=int(values["z"]),
                            pseudo_z_home=int(values["pseudo_z_home"]),
                            run_in_parallel=values.get("run_in_parallel") is True,
                            gripper_confirmed=values.get("gripper_confirmed") is True,
                            tip_loaded=values.get("tip_loaded") is True,
                            plate_on_gantry=(None if values.get("plate_on_gantry") is None else int(values["plate_on_gantry"])),
                            location19_y=int(values.get("location19_y", 0)),
                            wait_timeout_s=float(values.get("wait_timeout_s", 30.0)),
                        )
                    elif selected == "set_max_speed":
                        result = self.primitives.x_set_max_speed(value=int(values.get("value", 0)))
                    elif selected == "set_max_acc":
                        result = self.primitives.x_set_max_acc(value=int(values.get("value", 0)))
                    elif selected == "restore_original_speed":
                        result = self.primitives.x_restore_original_speed()
                    elif selected == "set_stall_guard":
                        result = self.primitives.x_set_stall_guard(value=int(values.get("value", 0)))
                    elif selected == "enable_xy_current":
                        result = self.primitives.x_enable_xy_current_mode(enabled=values.get("enabled") is True)
                    elif selected == "enable_xyz_current":
                        result = self.primitives.x_enable_xyz_current_mode(
                            enabled=values.get("enabled") is True,
                            z_current_up=int(values.get("z_current_up", 31)),
                        )
                    elif selected == "set_home":
                        result = self.primitives.x_set_home()
                    elif selected == "terminal_status":
                        result = self.primitives.x_terminal_status()
                    elif selected == "stop":
                        result = self.primitives.x_stop(timeout_s=float(values.get("timeout_s", 3.0)))
                    elif selected == "abort":
                        result = self.primitives.x_abort(reason=str(values.get("reason") or "forceAbortMotion"))
                    elif selected in {"startup_home", "home_axis", "manual_panel_home", "move_to_origin_home", "caught_plate_recovery_home"}:
                        home_fn = getattr(self.primitives, "x_" + selected)
                        result = home_fn(timeout_s=float(values.get("timeout_s", 30.0)))
                    else:
                        raise ValueError(f"unsupported_x_intent:{selected}")
                except Exception as exc:
                    result = {"ok": False, "failure": f"x_intent_exception:{type(exc).__name__}:{exc}", "command_issued": False}
            result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "x_result_not_mapping"}
            if automatic_prerequisites:
                result["automatic_prerequisites"] = automatic_prerequisites
            if selected in home_intents and not (
                result.get("home_predicate_confirmed") is True
                and result.get("controller_terminal_state_verified") is True
            ) and result.get("reference_publication_required") is not True:
                result.update({"ok": False, "failure": "x_home_evidence_not_verified"})
            with self._x_interrupt_state_lock:
                if admitted_interrupt_epoch != self._x_interrupt_epoch:
                    result = {
                        "ok": False,
                        "failure": "x_intent_interrupted_by_safety_command",
                        "command_issued": result.get("command_issued", True),
                        "interrupt_epoch": self._x_interrupt_epoch,
                    }
            current_generation = int(self.generation_provider())
            if current_generation != generation:
                result = {
                    "ok": False,
                    "failure": "x_generation_changed_during_command",
                    "command_issued": result.get("command_issued", True),
                    "recorded_generation": generation,
                    "current_generation": current_generation,
                    "primitive_result": _json_safe(result),
                }
            receipt: dict[str, Any] | None = None
            z_receipt: dict[str, Any] | None = None
            if selected != "wait_for_motor" and result.get("pending_motion") is True:
                lifecycle.update({"state": "executing", "active_receipt": {"command_id": command_id, "intent": selected, "status": "executing", "result": _json_safe(result)}, "pending_ticket": _json_safe(result)})
            elif result.get("ok") is True:
                receipt_id = f"{command_id}:{time.time_ns()}" if interrupt else command_id
                home_requires_observation = bool(
                    (selected in home_intents or move_to_all_zero)
                    and (
                        result.get("reference_publication_required") is True
                        or (
                            result.get("home_predicate_confirmed") is True
                            and result.get("controller_terminal_state_verified") is True
                        )
                    )
                )
                if selected == "reconcile_switch_masks":
                    next_state = "unprepared"
                    next_reference = "desynced"
                elif home_requires_observation:
                    next_state = "awaiting_operator_observation"
                    next_reference = "desynced"
                elif selected in passive_intents or selected in profile_intents:
                    next_state = prior_state
                    next_reference = prior_reference_state
                else:
                    next_state = "referenced_ready"
                    next_reference = "referenced"
                lifecycle.update({"state": next_state, "active_receipt": None, "pending_ticket": None, "reference_state": next_reference, "awaiting_observation_receipt_id": receipt_id if home_requires_observation else None, "last_failure": None})
                receipt = {"command_id": command_id, "receipt_id": receipt_id, "intent": selected, "motion_kind": "home_xy" if move_to_all_zero else "home" if selected in home_intents else "motion", "idempotency_key": idempotency_key, "idempotency_replay_enabled": not interrupt, "generation": generation, "board_lifecycle_generation": lifecycle.get("board_lifecycle_generation"), "inputs": safe_inputs, "status": "completed", "result": _json_safe(result)}
                lifecycle["receipts"].append(receipt)
                lifecycle["receipts"] = lifecycle["receipts"][-8:]
                if selected == "enable_xyz_current":
                    z_receipt = {
                        **copy.deepcopy(receipt),
                        "stream": "z",
                        "intent": "enable_xyz_current",
                        "x_generation": generation,
                        "z_generation": z_lifecycle.get("generation"),
                        "z_board_lifecycle_generation": z_lifecycle.get("board_lifecycle_generation"),
                        "z_state": prior_z_state,
                        "z_reference_state": prior_z_reference_state,
                    }
                    z_lifecycle.update({
                        "state": prior_z_state,
                        "reference_state": prior_z_reference_state,
                        "active_receipt": None,
                        "awaiting_observation_receipt_id": None,
                        "last_failure": None,
                    })
                    z_receipts = list(z_lifecycle.get("receipts") or [])
                    z_receipts.append(z_receipt)
                    z_lifecycle["receipts"] = z_receipts[-8:]
            else:
                if selected in passive_intents:
                    lifecycle.update({"state": prior_state, "active_receipt": None, "pending_ticket": None, "reference_state": prior_reference_state, "last_failure": _json_safe(result)})
                else:
                    lifecycle.update({"state": "failed_latched", "active_receipt": None, "pending_ticket": None, "reference_state": "desynced", "awaiting_observation_receipt_id": None, "last_failure": _json_safe(result)})
                receipt = {"command_id": command_id, "receipt_id": f"{command_id}:{time.time_ns()}" if interrupt else command_id, "intent": selected, "idempotency_key": idempotency_key, "idempotency_replay_enabled": not interrupt, "generation": generation, "inputs": safe_inputs, "status": "failed", "result": _json_safe(result)}
                lifecycle["receipts"].append(receipt)
                lifecycle["receipts"] = lifecycle["receipts"][-8:]
                if selected == "enable_xyz_current":
                    z_receipt = {
                        **copy.deepcopy(receipt),
                        "stream": "z",
                        "intent": "enable_xyz_current",
                        "x_generation": generation,
                        "z_generation": z_lifecycle.get("generation"),
                        "z_board_lifecycle_generation": z_lifecycle.get("board_lifecycle_generation"),
                        "z_state": prior_z_state,
                        "z_reference_state": prior_z_reference_state,
                    }
                    z_lifecycle.update({
                        "state": prior_z_state,
                        "reference_state": prior_z_reference_state,
                        "active_receipt": None,
                        "awaiting_observation_receipt_id": None,
                        "last_failure": _json_safe(result),
                    })
                    z_receipts = list(z_lifecycle.get("receipts") or [])
                    z_receipts.append(z_receipt)
                    z_lifecycle["receipts"] = z_receipts[-8:]
            self._save_state(state)
            if (
                receipt is not None
                and self.state_store is not None
                and hasattr(self.state_store, "append_serial206_receipt")
            ):
                if selected == "enable_xyz_current" and z_receipt is not None:
                    append_atomic = getattr(self.state_store, "append_serial206_receipts_atomic", None)
                    if not callable(append_atomic):
                        raise RuntimeError("atomic_multi_stream_receipt_store_required")
                    append_atomic((("x", receipt), ("z", z_receipt)))
                elif interrupt and hasattr(self.state_store, "append_serial206_interrupt_receipt"):
                    self.state_store.append_serial206_interrupt_receipt("x", receipt)
                else:
                    self.state_store.append_serial206_receipt("x", receipt)
            return {"ok": result.get("ok") is True, "axis": "xyz" if selected == "enable_xyz_current" else "x", "intent": selected, "state": lifecycle["state"], "z_state": z_lifecycle.get("state") if selected == "enable_xyz_current" else None, "result": _json_safe(result), "generation": generation}

    def record_x_observation(
        self,
        *,
        command_id: str,
        observation_command_id: str | None = None,
        verdict: str,
        physical_motion_observed: bool,
        expected_direction_observed: bool,
        home_endpoint_observed: bool,
        stopped_observed: bool,
        note: str,
        expected_generation: int,
    ) -> dict[str, Any]:
        if verdict not in {"pass", "fail"}:
            raise ValueError("X observation verdict must be pass or fail")
        flags = (
            physical_motion_observed,
            expected_direction_observed,
            home_endpoint_observed,
            stopped_observed,
        )
        if any(type(value) is not bool for value in flags):
            raise ValueError("X observation fields must be strict booleans")
        with self._lock:
            state = self._load_state()
            lifecycle = state["x_lifecycle"]
            generation = int(self.generation_provider())
            receipt = next(
                (
                    row for row in reversed(lifecycle.get("receipts") or [])
                    if isinstance(row, Mapping) and row.get("command_id") == command_id
                ),
                None,
            )
            if receipt is None:
                receipt = self._durable_serial206_receipt("x", command_id)
            current = bool(
                isinstance(receipt, Mapping)
                and receipt.get("status") == "completed"
                and lifecycle.get("state") == "awaiting_operator_observation"
                and lifecycle.get("awaiting_observation_receipt_id") == command_id
                and lifecycle.get("generation") == generation == int(expected_generation)
                and receipt.get("board_lifecycle_generation") == lifecycle.get("board_lifecycle_generation")
            )
            eligible = bool(
                current
                and verdict == "pass"
                and physical_motion_observed
                and expected_direction_observed
                and home_endpoint_observed
                and stopped_observed
            )
            observation_id = observation_command_id or f"xobs-{time.time_ns()}"
            observation = {
                "command_id": observation_id,
                "observes_command_id": command_id,
                "intent": "observation",
                "status": "completed" if eligible else "failed",
                "generation": generation,
                "board_lifecycle_generation": lifecycle.get("board_lifecycle_generation"),
                "verdict": verdict,
                "physical_motion_observed": physical_motion_observed,
                "expected_direction_observed": expected_direction_observed,
                "home_endpoint_observed": home_endpoint_observed,
                "stopped_observed": stopped_observed,
                "note": note.strip(),
                "reference_eligible": eligible,
                "idempotency_replay_enabled": True,
            }
            if eligible:
                reference = None
                if self.reference_store is not None:
                    observed_home_xy = isinstance(receipt, Mapping) and receipt.get("motion_kind") == "home_xy"
                    if observed_home_xy:
                        reference = self.reference_store.mark_referenced_many((
                            MarkAxisReferencedCommand(axis="x", position_steps=0, source="serial206.xy.operator_observation", motion_kind="home_xy"),
                            MarkAxisReferencedCommand(axis="y", position_steps=0, source="serial206.xy.operator_observation", motion_kind="home_xy"),
                        ))
                    else:
                        reference = self.reference_store.mark_referenced(
                            MarkAxisReferencedCommand(
                                axis="x",
                                position_steps=0,
                                source="serial206.x.operator_observation",
                                motion_kind="home",
                            )
                        )
                    eligible = bool(
                        isinstance(reference, Mapping)
                        and reference.get("ok") is True
                        and reference.get("durable_clean") is True
                    )
                lifecycle.update({
                    "state": "referenced_ready" if eligible else "failed_latched",
                    "reference_state": "referenced" if eligible else "desynced",
                    "awaiting_observation_receipt_id": None,
                    "last_failure": None if eligible else "x_reference_persistence_failed",
                })
                observation["reference_persistence"] = _json_safe(reference)
            elif current:
                lifecycle.update({
                    "state": "failed_latched",
                    "reference_state": "desynced",
                    "awaiting_observation_receipt_id": None,
                    "last_failure": "x_observation_not_reference_eligible",
                })
            lifecycle["receipts"].append(observation)
            lifecycle["receipts"] = lifecycle["receipts"][-8:]
            self._save_state(state)
            if self.state_store is not None and hasattr(self.state_store, "append_serial206_receipt"):
                self.state_store.append_serial206_receipt("x", observation)
            return {
                "ok": eligible,
                "axis": "x",
                "observation": _json_safe(observation),
                "state": lifecycle.get("state"),
            }

    def execute_xy_intent(self, x: int, y: int, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = dict(values or {})
        with self._x_interrupt_state_lock:
            admitted_interrupt_epoch = self._x_interrupt_epoch
            if self._x_interrupt_active:
                return {"ok": False, "axis": "xy", "state": "failed_latched", "failure": "xy_blocked_by_x_interrupt"}
        with self._lock:
            with self._x_interrupt_state_lock:
                if self._x_interrupt_active or self._x_interrupt_epoch != admitted_interrupt_epoch:
                    return {"ok": False, "axis": "xy", "state": "failed_latched", "failure": "xy_blocked_by_x_interrupt"}
            terminal_authority_saved = False
            state: dict[str, Any] = {}
            lifecycle: dict[str, Any] = {}
            try:
                state = self._load_state()
                lifecycle = state["x_lifecycle"]
                generation = int(self.generation_provider())
                board_generation_fn = getattr(self.preparation_provider, "current_board_lifecycle_generation", None)
                current_board_generation = board_generation_fn() if callable(board_generation_fn) else None
                idempotency_key = values.get("idempotency_key")
                idempotency_key = idempotency_key if isinstance(idempotency_key, str) and idempotency_key else None
                command_id = str(values.get("command_id") or idempotency_key or f"xy-{generation}-{int(x)}-{int(y)}")
                safe_inputs = _json_safe(values)
                if lifecycle.get("state") == "executing":
                    lifecycle.update({
                        "state": "failed_latched",
                        "reference_state": "desynced",
                        "last_failure": "xy_restart_or_reentry_during_executing",
                        "active_receipt": None,
                        "pending_ticket": None,
                    })
                    self._save_state(state)
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": "failed_latched",
                        "failure": "xy_executing_outcome_ambiguous",
                    }
                existing_receipt = next(
                    (
                        receipt for receipt in lifecycle.get("receipts", [])
                        if isinstance(receipt, Mapping) and receipt.get("command_id") == command_id
                    ),
                    None,
                )
                if existing_receipt is None:
                    existing_receipt = self._durable_serial206_receipt("x", command_id)
                if existing_receipt is None and idempotency_key is not None:
                    existing_receipt = self._durable_serial206_receipt_by_idempotency("x", idempotency_key)
                if isinstance(existing_receipt, Mapping):
                    request_matches = (
                        existing_receipt.get("intent") == "move_xy"
                        and existing_receipt.get("generation") == generation
                        and existing_receipt.get("target_x") == int(x)
                        and existing_receipt.get("target_y") == int(y)
                        and existing_receipt.get("inputs") == safe_inputs
                    )
                    authority_valid = (
                        lifecycle.get("generation") == generation
                        and lifecycle.get("state") == "referenced_ready"
                        and lifecycle.get("reference_state") == "referenced"
                    )
                    if not request_matches or (
                        existing_receipt.get("status") == "completed" and not authority_valid
                    ):
                        return {
                            "ok": False,
                            "axis": "xy",
                            "state": lifecycle.get("state"),
                            "failure": "xy_replay_current_authority_or_request_invalid",
                            "replayed": True,
                            "authority_receipt": _json_safe(existing_receipt),
                        }
                    replayed_result = copy.deepcopy(dict(existing_receipt.get("result") or {}))
                    replayed_result.setdefault("ok", existing_receipt.get("status") == "completed")
                    replayed_result.update({"replayed": True, "authority_receipt": _json_safe(existing_receipt)})
                    return replayed_result
                authority_current = bool(
                    lifecycle.get("generation") == generation
                    and type(lifecycle.get("board_lifecycle_generation")) is int
                    and lifecycle.get("board_lifecycle_generation") == current_board_generation
                    and lifecycle.get("state") == "referenced_ready"
                    and lifecycle.get("reference_state") == "referenced"
                    and isinstance(lifecycle.get("prepared_receipt"), Mapping)
                )
                if not authority_current:
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": lifecycle.get("state"),
                        "failure": "xy_current_referenced_authority_required",
                        "generation": generation,
                        "board_lifecycle_generation": current_board_generation,
                    }
                active = {"command_id": command_id, "intent": "move_xy", "idempotency_key": idempotency_key, "generation": generation, "target_x": int(x), "target_y": int(y), "inputs": safe_inputs, "status": "executing", "result": None}
                lifecycle.update({"state": "executing", "generation": generation, "active_receipt": active, "pending_ticket": None})
                self._save_state(state)
                result = self.primitives.move_xy(int(x), int(y), speed=values.get("speed"), acc=values.get("acc"), wait_timeout_s=float(values.get("wait_timeout_s", 5.0)))
                result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "xy_result_not_mapping"}
                with self._x_interrupt_state_lock:
                    interrupted = self._x_interrupt_epoch != admitted_interrupt_epoch
                if interrupted or int(self.generation_provider()) != generation:
                    result = {
                        "ok": False,
                        "failure": "xy_interrupted_or_generation_changed",
                        "primitive_result": _json_safe(result),
                    }
                verified_success = bool(
                    result.get("ok") is True
                    and result.get("controller_terminal_state_verified") is True
                    and result.get("target_position_verified") is True
                    and (
                        result.get("source_noop") is True
                        or result.get("acceleration_restore_verified") is True
                    )
                )
                if result.get("source_noop") is True:
                    verified_success = bool(verified_success and result.get("command_issued") is False)
                result["ok"] = verified_success
                if verified_success:
                    lifecycle.update({"state": "referenced_ready", "active_receipt": None, "reference_state": "referenced", "last_failure": None})
                else:
                    lifecycle.update({"state": "failed_latched", "active_receipt": None, "reference_state": "desynced", "last_failure": _json_safe(result)})
                receipt = {
                    "command_id": command_id,
                    "intent": "move_xy",
                    "idempotency_key": idempotency_key,
                    "idempotency_replay_enabled": True,
                    "generation": generation,
                    "board_lifecycle_generation": lifecycle.get("board_lifecycle_generation"),
                    "target_x": int(x),
                    "target_y": int(y),
                    "inputs": safe_inputs,
                    "status": "completed" if result.get("ok") is True else "failed",
                    "result": _json_safe(result),
                }
                lifecycle["receipts"].append(receipt)
                lifecycle["receipts"] = lifecycle["receipts"][-8:]
                self._save_state(state)
                terminal_authority_saved = True
                if self.state_store is not None and hasattr(self.state_store, "append_serial206_receipt"):
                    self.state_store.append_serial206_receipt("x", receipt)
                return {"ok": result.get("ok") is True, "axis": "xy", "intent": "move_xy", "state": lifecycle["state"], "result": _json_safe(result), "generation": generation}
            except Exception as exc:
                if terminal_authority_saved:
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": lifecycle.get("state", "reconciliation_required"),
                        "failure": f"xy_receipt_publication_exception:{type(exc).__name__}:{exc}",
                    }
                if not state or not lifecycle:
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": "reconciliation_required",
                        "failure": f"xy_authority_load_exception:{type(exc).__name__}:{exc}",
                    }
                try:
                    lifecycle.update({
                        "state": "failed_latched",
                        "reference_state": "desynced",
                        "active_receipt": None,
                        "pending_ticket": None,
                        "last_failure": f"xy_intent_exception:{type(exc).__name__}:{exc}",
                    })
                    self._save_state(state)
                except Exception as save_exc:
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": "reconciliation_required",
                        "failure": f"xy_authority_save_exception:{type(save_exc).__name__}:{save_exc}",
                    }
                return {"ok": False, "axis": "xy", "state": "failed_latched", "failure": f"xy_intent_exception:{type(exc).__name__}:{exc}"}

    def execute_homexy_intent(self, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = dict(values or {})
        with self._x_interrupt_state_lock:
            admitted_interrupt_epoch = self._x_interrupt_epoch
            if self._x_interrupt_active:
                return {"ok": False, "axis": "xy", "state": "failed_latched", "failure": "homexy_blocked_by_x_interrupt"}
        with self._lock:
            with self._x_interrupt_state_lock:
                if self._x_interrupt_active or self._x_interrupt_epoch != admitted_interrupt_epoch:
                    return {"ok": False, "axis": "xy", "state": "failed_latched", "failure": "homexy_blocked_by_x_interrupt"}
            terminal_authority_saved = False
            state: dict[str, Any] = {}
            lifecycle: dict[str, Any] = {}
            try:
                state = self._load_state()
                lifecycle = state["x_lifecycle"]
                generation = int(self.generation_provider())
                board_generation_fn = getattr(self.preparation_provider, "current_board_lifecycle_generation", None)
                current_board_generation = board_generation_fn() if callable(board_generation_fn) else None
                idempotency_key = values.get("idempotency_key")
                idempotency_key = idempotency_key if isinstance(idempotency_key, str) and idempotency_key else None
                command_id = str(values.get("command_id") or idempotency_key or f"homexy-{generation}")
                safe_inputs = _json_safe(values)
                if lifecycle.get("state") == "executing":
                    lifecycle.update({
                        "state": "failed_latched",
                        "reference_state": "desynced",
                        "last_failure": "homexy_restart_or_reentry_during_executing",
                        "active_receipt": None,
                        "pending_ticket": None,
                    })
                    self._save_state(state)
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": "failed_latched",
                        "failure": "homexy_executing_outcome_ambiguous",
                    }
                existing = next(
                    (
                        row for row in reversed(lifecycle.get("receipts") or [])
                        if isinstance(row, Mapping)
                        and (row.get("command_id") == command_id or (
                            idempotency_key is not None and row.get("idempotency_key") == idempotency_key
                        ))
                    ),
                    None,
                )
                if existing is None:
                    existing = self._durable_serial206_receipt("x", command_id)
                if existing is None and idempotency_key is not None:
                    existing = self._durable_serial206_receipt_by_idempotency("x", idempotency_key)
                if isinstance(existing, Mapping):
                    if (
                        existing.get("intent") != "home_xy"
                        or existing.get("inputs") != safe_inputs
                        or existing.get("generation") != generation
                    ):
                        return {
                            "ok": False,
                            "axis": "xy",
                            "state": lifecycle.get("state"),
                            "failure": "homexy_replay_authority_or_request_mismatch",
                            "replayed": True,
                            "authority_receipt": _json_safe(existing),
                        }
                    replay_authority_valid = (
                        lifecycle.get("generation") == generation
                        and lifecycle.get("board_lifecycle_generation") == current_board_generation
                        and lifecycle.get("state") == "awaiting_operator_observation"
                        and lifecycle.get("reference_state") == "desynced"
                    )
                    if existing.get("status") == "completed" and not replay_authority_valid:
                        return {
                            "ok": False,
                            "axis": "xy",
                            "state": lifecycle.get("state"),
                            "failure": "homexy_replay_current_authority_invalid",
                            "replayed": True,
                            "authority_receipt": _json_safe(existing),
                        }
                    replayed_result = dict(existing.get("result") or {})
                    replayed_result.setdefault("ok", existing.get("status") == "completed")
                    replayed_result.update({"replayed": True, "authority_receipt": _json_safe(existing)})
                    return replayed_result
                prior_state = str(lifecycle.get("state") or "unprepared")
                prior_reference_state = str(lifecycle.get("reference_state") or "unknown")
                authority_current = bool(
                    lifecycle.get("generation") == generation
                    and type(lifecycle.get("board_lifecycle_generation")) is int
                    and lifecycle.get("board_lifecycle_generation") == current_board_generation
                    and lifecycle.get("state") in {"prepared_unreferenced", "referenced_ready"}
                    and isinstance(lifecycle.get("prepared_receipt"), Mapping)
                )
                if not authority_current:
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": lifecycle.get("state"),
                        "failure": "homexy_current_prepared_authority_required",
                        "generation": generation,
                        "board_lifecycle_generation": current_board_generation,
                    }
                primitive_tester = getattr(self.primitives, "tester", None)
                present_fn = getattr(primitive_tester, "_oem_board_present", None)
                missing_board_source_noop = bool(
                    callable(present_fn)
                    and (not bool(present_fn(5)) or not bool(present_fn(4)))
                )
                live_preflight = {"skipped": True, "reason": "homexy_missing_board_source_null"}
                if not missing_board_source_noop:
                    preflight_fn = getattr(self.primitives, "_x_require_motion_preflight", None)
                    if not callable(preflight_fn):
                        return {
                            "ok": False,
                            "axis": "xy",
                            "state": lifecycle.get("state"),
                            "failure": "homexy_live_preflight_unavailable",
                        }
                    try:
                        live_preflight = preflight_fn()
                    except Exception as exc:
                        return {
                            "ok": False,
                            "axis": "xy",
                            "state": lifecycle.get("state"),
                            "failure": f"homexy_live_preflight_failed:{type(exc).__name__}:{exc}",
                        }
                active = {"command_id": command_id, "intent": "home_xy", "idempotency_key": idempotency_key, "generation": generation, "inputs": safe_inputs, "status": "executing", "result": None}
                lifecycle.update({"state": "executing", "generation": generation, "active_receipt": active, "pending_ticket": None})
                self._save_state(state)
                result = self.primitives.home_xy(timeout_s=float(values.get("timeout_s", 30.0)), allow_implementation_mapped_predicate=bool(values.get("allow_implementation_mapped_predicate", False)))
                result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "homexy_result_not_mapping"}
                result["live_preflight"] = _json_safe(live_preflight)
                with self._x_interrupt_state_lock:
                    interrupted = self._x_interrupt_epoch != admitted_interrupt_epoch
                if interrupted or int(self.generation_provider()) != generation:
                    result = {
                        "ok": False,
                        "failure": "homexy_interrupted_or_generation_changed",
                        "primitive_result": _json_safe(result),
                    }
                source_noop = bool(
                    result.get("ok") is True
                    and result.get("source_noop") is True
                    and result.get("command_issued") is False
                )
                verified_success = bool(
                    result.get("ok") is True
                    and result.get("controller_terminal_state_verified") is True
                    and result.get("reference_publication_required") is True
                )
                result["ok"] = bool(source_noop or verified_success)
                if source_noop:
                    lifecycle.update({
                        "state": prior_state,
                        "active_receipt": None,
                        "reference_state": prior_reference_state,
                        "awaiting_observation_receipt_id": None,
                        "last_failure": None,
                    })
                else:
                    lifecycle.update({"state": "awaiting_operator_observation" if verified_success else "failed_latched", "active_receipt": None, "reference_state": "desynced", "awaiting_observation_receipt_id": command_id if verified_success else None, "last_failure": None if verified_success else _json_safe(result)})
                receipt = {
                    "command_id": command_id,
                    "intent": "home_xy",
                    "motion_kind": "home_xy",
                    "source_noop": source_noop,
                    "idempotency_key": idempotency_key,
                    "idempotency_replay_enabled": True,
                    "generation": generation,
                    "board_lifecycle_generation": lifecycle.get("board_lifecycle_generation"),
                    "inputs": safe_inputs,
                    "status": "completed" if result.get("ok") is True else "failed",
                    "result": _json_safe(result),
                }
                lifecycle["receipts"].append(receipt)
                lifecycle["receipts"] = lifecycle["receipts"][-8:]
                self._save_state(state)
                terminal_authority_saved = True
                if self.state_store is not None and hasattr(self.state_store, "append_serial206_receipt"):
                    self.state_store.append_serial206_receipt("x", receipt)
                return {"ok": result.get("ok") is True, "axis": "xy", "intent": "home_xy", "state": lifecycle["state"], "result": _json_safe(result), "generation": generation}
            except Exception as exc:
                if terminal_authority_saved:
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": lifecycle.get("state", "reconciliation_required"),
                        "failure": f"homexy_receipt_publication_exception:{type(exc).__name__}:{exc}",
                    }
                if not state or not lifecycle:
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": "reconciliation_required",
                        "failure": f"homexy_authority_load_exception:{type(exc).__name__}:{exc}",
                    }
                try:
                    lifecycle.update({
                        "state": "failed_latched",
                        "reference_state": "desynced",
                        "active_receipt": None,
                        "pending_ticket": None,
                        "last_failure": f"homexy_intent_exception:{type(exc).__name__}:{exc}",
                    })
                    self._save_state(state)
                except Exception as save_exc:
                    return {
                        "ok": False,
                        "axis": "xy",
                        "state": "reconciliation_required",
                        "failure": f"homexy_authority_save_exception:{type(save_exc).__name__}:{save_exc}",
                    }
                return {"ok": False, "axis": "xy", "state": "failed_latched", "failure": f"homexy_intent_exception:{type(exc).__name__}:{exc}"}
    def notify_board_activation(self, board_id: int, ack: Any, *, active: bool | None = None) -> dict[str, Any]:
        """Invalidate axis authority on external board lifecycle changes."""
        if int(board_id) == 5:
            with self._lock:
                state = self._load_state()
                x = state["x_lifecycle"]
                previous = str(x.get("state") or "unprepared")
                transition = "activated" if active is not False else "deactivated"
                has_authority = bool(previous != "unprepared" or x.get("generation") is not None or x.get("board_lifecycle_generation") is not None or x.get("prepared_receipt") is not None or x.get("active_receipt") is not None or x.get("pending_ticket") is not None or x.get("awaiting_observation_receipt_id") is not None or x.get("reference_state") == "referenced")
                if not has_authority:
                    return {"x_affected": False, "board": 5, "already_unprepared": True, "transition": transition}
                invalidation = {"reason": "board5_lifecycle_change", "transition": transition, "command64_value": None if active is None else int(bool(active)), "previous_state": previous, "ack": _json_safe(ack), "invalidated_at": time.time()}
                x.update({"state": "unprepared", "generation": None, "board_lifecycle_generation": None, "prepared_receipt": None, "active_receipt": None, "pending_ticket": None, "awaiting_observation_receipt_id": None, "reference_state": "desynced", "last_failure": invalidation})
                desync = getattr(self.primitives, "_x_desync", None)
                reference_invalidation = desync(f"Board 5 was {transition}; serial-206 X authority was invalidated.", "board_activation_invalidation") if callable(desync) else None
                invalidation["reference_invalidation"] = _json_safe(reference_invalidation)
                self._save_state(state)
                return {"x_affected": True, "board": 5, "invalidation": invalidation}
        if int(board_id) != 4:
            return {"z_affected": False, "board": int(board_id)}
        with self._lock:
            state = self._load_state()
            z = state["z_lifecycle"]
            active_receipt = z.get("active_receipt") if isinstance(z.get("active_receipt"), Mapping) else {}
            scope = self._board_transition_scope
            expected_transition = None
            remaining_expected_transitions: list[Any] = []
            provider_owned = False
            if (
                isinstance(scope, dict)
                and scope.get("thread_id") == threading.get_ident()
                and scope.get("command_id") == active_receipt.get("command_id")
                and active_receipt.get("intent") == "prepare"
            ):
                expected_value = scope.get("expected")
                expected: list[Any] = expected_value if isinstance(expected_value, list) else []
                expected_transition = expected[0] if expected else None
                observed_transition = bool(active) if active is not None else None
                ack_ok = isinstance(ack, Mapping) and type(ack.get("status")) is int and ack.get("status") == 100
                if expected_transition is observed_transition and ack_ok:
                    expected.pop(0)
                    remaining_expected_transitions = list(expected)
                    provider_owned = True
                else:
                    scope["violation"] = {
                        "expected_transition": expected_transition,
                        "observed_transition": observed_transition,
                        "ack": _json_safe(ack),
                    }
            if provider_owned:
                return {
                    "z_affected": False,
                    "board": 4,
                    "provider_owned_preparation": True,
                    "transition": "activated" if active is not False else "deactivated",
                    "remaining_expected_transitions": remaining_expected_transitions,
                }
            previous = str(z.get("state") or "unprepared")
            transition = "activated" if active is not False else "deactivated"
            has_preparation_authority = bool(
                previous != "unprepared"
                or z.get("generation") is not None
                or z.get("board_lifecycle_generation") is not None
                or z.get("prepared_receipt") is not None
                or z.get("active_receipt") is not None
                or z.get("awaiting_observation_receipt_id") is not None
                or z.get("reference_state") == "referenced"
            )
            if not has_preparation_authority:
                return {"z_affected": False, "board": 4, "already_unprepared": True, "transition": transition}
            invalidation = {
                "reason": "board4_command64_outside_provider_preparation",
                "transition": transition,
                "command64_value": None if active is None else int(bool(active)),
                "previous_state": previous,
                "ack": _json_safe(ack),
                "invalidated_at": time.time(),
            }
            z.update({
                "state": "unprepared",
                "generation": None,
                "board_lifecycle_generation": None,
                "prepared_receipt": None,
                "active_receipt": None,
                "reference_state": "desynced",
                "awaiting_observation_receipt_id": None,
                "last_failure": invalidation,
            })
            self._z_mark_desynced(
                f"Board 4 was {transition} outside provider-owned Z preparation.",
                "serial206.z.board_activation_invalidation",
            )
            self._save_state(state)
            return {"z_affected": True, "board": 4, "invalidation": invalidation}

    def z_projection(self) -> dict[str, Any]:
        with self._lock:
            try:
                state = self._load_state()
                z = state["z_lifecycle"]
                generation = int(self.generation_provider())
                prepared_generation = z.get("generation")
                generation_drift = bool(
                    prepared_generation is not None and int(prepared_generation) != generation
                )
                board_generation_provider = getattr(
                    self.preparation_provider, "current_board_lifecycle_generation", None
                )
                current_board_generation = (
                    board_generation_provider() if callable(board_generation_provider) else None
                )
                prepared_board_generation = z.get("board_lifecycle_generation")
                board_generation_fresh = bool(
                    type(prepared_board_generation) is int
                    and current_board_generation == prepared_board_generation
                )
                board_drift = bool(z.get("state") != "unprepared" and not board_generation_fresh)
                if generation_drift or board_drift:
                    prior_state = z.get("state")
                    reason = (
                        "ownership_generation_changed"
                        if generation_drift
                        else "board_lifecycle_generation_changed"
                    )
                    reference_invalidation = self._z_mark_desynced(
                        f"Serial-206 Z authority invalidated because {reason}.",
                        "serial206.z.projection_invalidation",
                    )
                    z.update(
                        {
                            "state": "unprepared",
                            "generation": None,
                            "board_lifecycle_generation": None,
                            "prepared_receipt": None,
                            "active_receipt": None,
                            "awaiting_observation_receipt_id": None,
                            "reference_state": "desynced",
                            "last_failure": {
                                "failure": reason,
                                "durable_state_before_invalidation": prior_state,
                                "reference_invalidation": _json_safe(reference_invalidation),
                            },
                        }
                    )
                    state = self._save_state(state)
                    z = state["z_lifecycle"]
                    board_generation_fresh = False
                projection = {
                    str(key): _json_safe(value)
                    for key, value in z.items()
                    if key not in {"receipts", "receipts_omitted_to_sqlite"}
                }
                receipts = [
                    row for row in list(z.get("receipts") or [])
                    if isinstance(row, Mapping)
                ]
                latest = receipts[-1] if receipts else None
                projection.update({
                    "receipt_storage": "robot_sqlite",
                    "receipt_detail_on_request": True,
                    "recent_receipt_count": len(receipts),
                    "latest_receipt": None if latest is None else {
                        key: _json_safe(latest.get(key))
                        for key in ("command_id", "intent", "status")
                    },
                    "available": True,
                    "ownership_generation": generation,
                    "current_board_lifecycle_generation": current_board_generation,
                    "board_lifecycle_generation_fresh": board_generation_fresh,
                    "authority": type(self).__name__,
                    "board": 4,
                    "motor": 1,
                    "coordinate_contract": "oem_source_nonnegative_z",
                    "source_min_steps": 0,
                    "source_max_steps": 160000,
                    "terminal_state": self._sanitize_z_terminal_state(
                        copy.deepcopy(z.get("terminal_state"))
                        if isinstance(z.get("terminal_state"), Mapping)
                        else self._z_terminal_state_from_receipts(z)
                    ),
                })
                return projection
            except Exception as exc:
                return {
                    "available": False,
                    "state": "corrupt",
                    "blockers": ["durable_serial206_state_corrupt"],
                    "failure": f"{type(exc).__name__}: {exc}",
                }

    @staticmethod
    def _sanitize_z_terminal_state(value: Any) -> dict[str, Any] | None:
        if not isinstance(value, Mapping):
            return None
        return {
            "authority": value.get("authority") if isinstance(value.get("authority"), str) else None,
            "position_steps": value.get("position_steps") if type(value.get("position_steps")) is int else None,
            "speed_steps_s": value.get("speed_steps_s") if type(value.get("speed_steps_s")) is int else None,
            "left_switch_state": value.get("left_switch_state") if type(value.get("left_switch_state")) is int else None,
            "right_switch_state": value.get("right_switch_state") if type(value.get("right_switch_state")) is int else None,
            "left_switch_disabled": value.get("left_switch_disabled") if type(value.get("left_switch_disabled")) is bool else None,
            "right_switch_disabled": value.get("right_switch_disabled") if type(value.get("right_switch_disabled")) is bool else None,
            "source_command_id": value.get("source_command_id") if isinstance(value.get("source_command_id"), str) else None,
            "observed_at": value.get("observed_at") if type(value.get("observed_at")) in {int, float} else None,
        }

    @staticmethod
    def _z_terminal_state_from_receipts(z: Mapping[str, Any]) -> dict[str, Any]:
        """Project compact terminal Z state from provider-owned receipts."""
        position_steps: int | None = None
        speed_steps_s: int | None = None
        left_switch_state: int | None = None
        right_switch_state: int | None = None
        left_switch_disabled: bool | None = None
        right_switch_disabled: bool | None = None
        source_command_id: str | None = None
        observed_at: float | None = None
        for receipt in list(z.get("receipts") or []):
            if not isinstance(receipt, Mapping) or receipt.get("status") != "completed":
                continue
            result = receipt.get("result")
            if not isinstance(result, Mapping):
                continue
            intent = str(receipt.get("intent") or "")
            next_position: int | None = None
            terminal = result.get("terminal_z_state")
            if isinstance(terminal, Mapping) and terminal.get("ok") is True:
                if type(terminal.get("position_steps")) is int:
                    next_position = int(terminal["position_steps"])
                if type(terminal.get("speed_steps_s")) is int:
                    speed_steps_s = int(terminal["speed_steps_s"])
                if type(terminal.get("left_switch_state")) is int:
                    left_switch_state = int(terminal["left_switch_state"])
                if type(terminal.get("right_switch_state")) is int:
                    right_switch_state = int(terminal["right_switch_state"])
                if type(terminal.get("left_switch_disabled")) is bool:
                    left_switch_disabled = bool(terminal["left_switch_disabled"])
                if type(terminal.get("right_switch_disabled")) is bool:
                    right_switch_disabled = bool(terminal["right_switch_disabled"])
            if next_position is None and intent == "set_home" and result.get("physical_motion") is False:
                position = result.get("position")
                value = position.get("position") if isinstance(position, Mapping) else None
                if type(value) is int:
                    next_position = value
            elif next_position is None and intent in {"move_steps", "move_absolute"}:
                value = result.get("after_position_steps")
                if type(value) is int:
                    next_position = value
            elif next_position is None and intent in {"manual_home", "move_z_home", "diagnostic_home_axis", "self_test", "resume_after_abort"}:
                summary = result.get("home_summary")
                if isinstance(summary, Mapping):
                    for key in ("after_set_home_position_steps", "after_position_steps"):
                        value = summary.get(key)
                        if type(value) is int:
                            next_position = value
                            break
            if next_position is not None:
                position_steps = next_position
                source_command_id = str(receipt.get("command_id") or "") or source_command_id
                finished_at = receipt.get("finished_at")
                if type(finished_at) in {int, float}:
                    observed_at = float(finished_at)
            if result.get("controller_terminal_state_verified") is True and intent != "set_home":
                speed_steps_s = 0
        return {
            "position_steps": position_steps,
            "speed_steps_s": speed_steps_s,
            "left_switch_state": left_switch_state,
            "right_switch_state": right_switch_state,
            "left_switch_disabled": left_switch_disabled,
            "right_switch_disabled": right_switch_disabled,
            "source_command_id": source_command_id,
            "observed_at": observed_at,
            "authority": "provider_receipt_terminal_state",
        }

    def path_planning_authority(self, *, expected_generation: int) -> dict[str, Any]:
        """Return provider-owned live ``scriptmoveTo`` branch authority."""
        observed_generation = int(self.generation_provider())
        if int(expected_generation) != observed_generation:
            return {
                "ok": False,
                "blockers": ["ownership_generation_changed"],
                "expected_generation": int(expected_generation),
                "observed_generation": observed_generation,
            }
        with self._lock:
            state = self._load_state()
            z = state["z_lifecycle"]
            machine = dict(state.get("machine_status") or {})
        if z.get("state") != "referenced_ready" or z.get("reference_state") != "referenced":
            return {"ok": False, "blockers": ["z_reference_not_ready"]}
        if type(machine.get("tip_loaded")) is not bool or type(machine.get("tip_dirty")) is not bool:
            return {"ok": False, "blockers": ["tip_state_not_authoritative"]}
        if type(machine.get("clean_path")) is not bool:
            return {"ok": False, "blockers": ["clean_path_not_authoritative"]}
        if machine.get("current_location") is None or machine.get("current_well") is None:
            return {"ok": False, "blockers": ["current_location_not_authoritative"]}
        pseudo = machine.get("psudo_z_home_steps")
        if type(pseudo) is not int or pseudo not in {500, 65000}:
            return {"ok": False, "blockers": ["pseudo_z_home_not_authoritative"]}
        tip_location = machine.get("tip_location", -1)
        if type(tip_location) is not int:
            return {"ok": False, "blockers": ["tip_location_not_authoritative"]}
        if machine["tip_loaded"] is True and tip_location < 0:
            return {"ok": False, "blockers": ["loaded_tip_location_not_authoritative"]}
        read_position = getattr(self.primitives, "_read_axis_position", None)
        if not callable(read_position):
            return {"ok": False, "blockers": ["controller_position_reader_not_bound"]}
        try:
            coordinates = {axis: read_position(axis) for axis in ("x", "y", "z")}
        except Exception as exc:
            return {"ok": False, "blockers": [f"controller_position_read_failed:{type(exc).__name__}:{exc}"]}
        if any(type(value) is not int for value in coordinates.values()):
            return {"ok": False, "blockers": ["controller_position_not_strict_integer"]}
        if self.reference_store is None:
            return {"ok": False, "blockers": ["reference_store_not_bound"]}
        references = self.reference_store.snapshot(("g",))
        g_row = (references.get("rows") or {}).get("g") if isinstance(references, Mapping) else None
        gripper_confirmed = bool(
            isinstance(references, Mapping)
            and references.get("ok") is True
            and isinstance(g_row, Mapping)
            and g_row.get("state") == "referenced"
        )
        return {
            "ok": True,
            "generation": observed_generation,
            "board_lifecycle_generation": z.get("board_lifecycle_generation"),
            "current_x": coordinates["x"],
            "current_y": coordinates["y"],
            "current_z": coordinates["z"],
            "current_loc": machine["current_location"],
            "current_well": machine["current_well"],
            "tip_loaded": machine["tip_loaded"],
            "tip_dirty": machine["tip_dirty"],
            "clean_path": machine["clean_path"],
            "tip_location": tip_location,
            "gripper_confirmed": gripper_confirmed,
            "plate_on_gantry": machine.get("plate_on_gantry"),
            "pseudo_z_home": pseudo,
            "source": "provider_controller_reference_store",
        }

    def _append_z_receipt(self, z: dict[str, Any], receipt: Mapping[str, Any]) -> dict[str, Any]:
        receipts = list(z.get("receipts") or [])
        bounded = _json_safe(dict(receipt))
        # Preserve decision-grade evidence outside the deeply bounded result.
        # A large TMCL trace must not consume the global item budget before the
        # failure, endpoint, or controller-event fields are serialized.
        bounded["result_summary"] = _json_safe(receipt.get("result_summary"))
        bounded["critical_evidence"] = Serial206OemInitializationProvider._critical_z_result_evidence(
            receipt.get("result")
        )
        for key in (
            "command_id", "intent", "idempotency_key", "idempotency_replay_enabled",
            "expected_generation",
            "board_lifecycle_generation", "status", "started_at", "finished_at",
            "robot_http_acknowledged", "controller_command_acknowledged",
            "controller_terminal_state_verified", "physical_effect_verified",
            "operator_assessment",
        ):
            bounded[key] = _json_safe(receipt.get(key))
        receipts.append(bounded)
        z["receipts"] = receipts[-8:]
        return bounded

    def _persist_z_receipt(self, receipt: Mapping[str, Any]) -> None:
        if (
            receipt.get("status") not in {"completed", "failed", "rejected", "ambiguous", "cancelled"}
            or self.state_store is None
            or not hasattr(self.state_store, "append_serial206_receipt")
        ):
            return
        if (
            receipt.get("intent") in {"stop", "abort"}
            and hasattr(self.state_store, "append_serial206_interrupt_receipt")
        ):
            self.state_store.append_serial206_interrupt_receipt("z", receipt)
        else:
            self.state_store.append_serial206_receipt("z", receipt)

    @staticmethod
    def _critical_z_result_evidence(result: Any) -> dict[str, Any]:
        row = result if isinstance(result, Mapping) else {}
        wait = row.get("wait") if isinstance(row.get("wait"), Mapping) else {}

        def event_rows(key: str) -> list[dict[str, Any]]:
            events = row.get(key)
            if not isinstance(events, list):
                return []
            return [
                {
                    field: event.get(field)
                    for field in ("status", "event_sequence")
                    if event.get(field) is not None
                }
                for event in events
                if isinstance(event, Mapping)
            ][:32]

        return {
            "failure": row.get("failure"),
            "before_position_steps": row.get("before_position_steps"),
            "target_position_steps": row.get("target_position_steps"),
            "after_position_steps": row.get("after_position_steps"),
            "terminal_speed_steps_s": wait.get("last_speed"),
            "terminal_stopped": wait.get("stopped"),
            "target_events": event_rows("target_events"),
            "controller_error_events": event_rows("controller_error_events"),
        }

    @staticmethod
    def _z_terminal_state_from_result(
        result: Any,
        *,
        command_id: str,
        observed_at: float,
    ) -> dict[str, Any] | None:
        if not isinstance(result, Mapping):
            return None
        terminal = result.get("terminal_z_state")
        if (
            isinstance(terminal, Mapping)
            and terminal.get("ok") is True
            and type(terminal.get("position_steps")) is int
            and type(terminal.get("speed_steps_s")) is int
        ):
            return {
                "authority": "serial206_terminal_register_readback",
                "position_steps": terminal.get("position_steps"),
                "speed_steps_s": terminal.get("speed_steps_s"),
                "left_switch_state": terminal.get("left_switch_state"),
                "right_switch_state": terminal.get("right_switch_state"),
                "left_switch_disabled": terminal.get("left_switch_disabled"),
                "right_switch_disabled": terminal.get("right_switch_disabled"),
                "source_command_id": command_id,
                "observed_at": observed_at,
            }
        wait = result.get("wait") if isinstance(result.get("wait"), Mapping) else {}
        after = result.get("after") if isinstance(result.get("after"), Mapping) else {}
        after_ack = after.get("ack") if isinstance(after.get("ack"), Mapping) else {}
        position = result.get("after_position_steps")
        if (
            type(position) is not int
            or after_ack.get("status") != 100
            or wait.get("stopped") is not True
            or type(wait.get("last_speed")) is not int
        ):
            return None
        return {
            "authority": "serial206_terminal_register_readback",
            "position_steps": int(position),
            "speed_steps_s": int(wait["last_speed"]),
            "left_switch_state": None,
            "right_switch_state": None,
            "left_switch_disabled": None,
            "right_switch_disabled": None,
            "source_command_id": command_id,
            "observed_at": observed_at,
        }

    @staticmethod
    def _z_lifecycle_projection(z: Mapping[str, Any]) -> dict[str, Any]:
        """Project current Z authority without transferring receipt transcripts."""
        projection = {
            str(key): _json_safe(value)
            for key, value in z.items()
            if key not in {"receipts", "receipts_omitted_to_sqlite"}
        }
        receipts = [row for row in list(z.get("receipts") or []) if isinstance(row, Mapping)]
        latest = receipts[-1] if receipts else None
        projection["receipt_storage"] = "robot_sqlite"
        projection["receipt_detail_on_request"] = True
        projection["recent_receipt_count"] = len(receipts)
        projection["latest_receipt"] = None if latest is None else {
            key: _json_safe(latest.get(key))
            for key in (
                "command_id",
                "intent",
                "status",
                "started_at",
                "finished_at",
                "controller_command_acknowledged",
                "controller_terminal_state_verified",
                "physical_effect_verified",
            )
        }
        return projection

    @staticmethod
    def _z_reference_commit_verified(result: Any, *, expected_state: str) -> bool:
        return bool(
            isinstance(result, Mapping)
            and result.get("state") == expected_state
            and result.get("persisted") is True
            and result.get("verified", True) is True
            and result.get("durable_clean", True) is True
            and result.get("ok", True) is True
        )

    def _z_mark_desynced(self, reason: str, source: str) -> dict[str, Any]:
        if self.reference_store is None:
            raise RuntimeError("durable Z reference invalidation unavailable: reference store not bound")
        try:
            result = self.reference_store.mark_desynced(
                MarkAxisDesyncedCommand(axis="z", reason=reason, source=source)
            )
        except Exception as exc:
            raise RuntimeError(
                f"durable Z reference invalidation failed: {type(exc).__name__}: {exc}"
            ) from exc
        if not self._z_reference_commit_verified(result, expected_state="desynced"):
            raise RuntimeError(f"durable Z reference invalidation failed: {result}")
        return dict(result)

    def _z_mark_referenced(self, *, source: str, motion_kind: str) -> dict[str, Any]:
        if self.reference_store is None:
            raise RuntimeError("durable Z reference publication unavailable: reference store not bound")
        try:
            result = self.reference_store.mark_referenced(
                MarkAxisReferencedCommand(
                    axis="z",
                    position_steps=0,
                    source=source,
                    motion_kind=motion_kind,
                )
            )
        except Exception as exc:
            raise RuntimeError(
                f"durable Z reference publication failed: {type(exc).__name__}: {exc}"
            ) from exc
        if not self._z_reference_commit_verified(result, expected_state="referenced"):
            raise RuntimeError(f"durable Z reference publication failed: {result}")
        return dict(result)

    def execute_z_stop_interrupt(
        self,
        *,
        inputs: Mapping[str, Any] | None = None,
        expected_generation: int,
        idempotency_key: str,
        abort: bool = False,
    ) -> dict[str, Any]:
        """Deliver Z STOP before waiting for the provider lifecycle lock.

        A normal Z intent owns ``self._lock`` across controller execution. STOP
        therefore uses a separate dispatch lease, marks an interrupt epoch, and
        sends the OEM double-stop immediately. It acquires the lifecycle lock
        only afterward to make the interrupted command and reference state
        durably fail-closed.
        """

        values = dict(inputs or {})
        interrupt_intent = "abort" if bool(abort) else "stop"
        supplied_command_id = values.get("command_id")
        command_id = (
            supplied_command_id
            if isinstance(supplied_command_id, str) and supplied_command_id.strip()
            else f"z_{interrupt_intent}_{int(time.time() * 1000)}"
        )
        safe_inputs = _json_safe(values)
        with self._z_interrupt_dispatch_lock:
            with self._z_interrupt_state_lock:
                self._z_interrupt_epoch += 1
                interrupt_epoch = self._z_interrupt_epoch
                self._z_interrupt_active = True
            dispatch_started_at = time.time()
            snapshot_active: Mapping[str, Any] | None = None
            try:
                try:
                    if (
                        self.state_store is not None
                        and hasattr(self.state_store, "read_oem_serial206_initialization_state")
                    ):
                        stored_snapshot = self.state_store.read_oem_serial206_initialization_state()
                        snapshot_state = self._validate_state(
                            self._upgrade_state(
                                stored_snapshot if stored_snapshot is not None else self._new_state()
                            )
                        )
                    elif self._memory_state is None:
                        snapshot_state = self._new_state()
                    else:
                        snapshot_state = self._validate_state(
                            self._upgrade_state(copy.deepcopy(self._memory_state))
                        )
                    candidate = snapshot_state.get("z_lifecycle", {}).get("active_receipt")
                    if isinstance(candidate, Mapping):
                        snapshot_active = copy.deepcopy(candidate)
                except Exception:
                    snapshot_active = None

                try:
                    raw_result = (
                        self.primitives.z_abort(timeout_s=float(values.get("timeout_s", 3.0)))
                        if abort
                        else self.primitives.z_stop(timeout_s=float(values.get("timeout_s", 3.0)))
                    )
                except Exception as exc:
                    raw_result = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
                result = dict(raw_result) if isinstance(raw_result, Mapping) else {
                    "ok": False,
                    "failure": f"z_{interrupt_intent}_result_not_mapping",
                }
                delivery_finished_at = time.time()

                with self._lock:
                    state = self._load_state()
                    z = state["z_lifecycle"]
                    movement_intents = {
                        "manual_home",
                        "move_z_home",
                        "diagnostic_home_axis",
                        "move_steps",
                        "move_absolute",
                        "path_execute",
                        "move_gz",
                        "home_gz",
                        "lower_pipette",
                        "lift_pipette",
                        "self_test",
                    }
                    interrupted_ids: set[str] = set()
                    if (
                        isinstance(snapshot_active, Mapping)
                        and snapshot_active.get("intent") in movement_intents
                        and snapshot_active.get("command_id")
                    ):
                        interrupted_ids.add(str(snapshot_active["command_id"]))

                    latest_by_command: dict[str, Mapping[str, Any]] = {}
                    for row in z.get("receipts") or []:
                        if isinstance(row, Mapping) and row.get("command_id"):
                            latest_by_command[str(row["command_id"])] = row
                    for interrupted in latest_by_command.values():
                        if interrupted.get("intent") not in movement_intents:
                            continue
                        started = interrupted.get("started_at")
                        finished = interrupted.get("finished_at")
                        if not isinstance(started, (int, float)):
                            continue
                        if float(started) > delivery_finished_at:
                            continue
                        if finished is None or (
                            isinstance(finished, (int, float))
                            and float(finished) >= dispatch_started_at
                        ):
                            interrupted_ids.add(str(interrupted["command_id"]))

                    current_active = z.get("active_receipt")
                    if (
                        isinstance(current_active, Mapping)
                        and current_active.get("intent") in movement_intents
                        and current_active.get("command_id")
                    ):
                        interrupted_ids.add(str(current_active["command_id"]))

                    observed_generation = int(self.generation_provider())
                    generation_match = int(expected_generation) == observed_generation
                    controller_acknowledged = result.get("controller_command_acknowledged") is True
                    terminal_verified = result.get("controller_terminal_state_verified") is True
                    stop_verified = bool(
                        result.get("ok") is True
                        and controller_acknowledged
                        and terminal_verified
                    )
                    result.update(
                        {
                            "ok": stop_verified,
                            "interrupt_epoch": interrupt_epoch,
                            "interrupted_command_ids": sorted(interrupted_ids),
                            "ownership_generation_match": generation_match,
                        }
                    )
                    receipt = {
                        "command_id": command_id,
                        "receipt_id": f"{command_id}:{time.time_ns()}",
                        "intent": interrupt_intent,
                        "idempotency_key": idempotency_key,
                        "idempotency_replay_enabled": False,
                        "expected_generation": int(expected_generation),
                        "observed_generation": observed_generation,
                        "board_lifecycle_generation": z.get("board_lifecycle_generation"),
                        "inputs": safe_inputs,
                        "status": "completed" if stop_verified else "failed",
                        "started_at": dispatch_started_at,
                        "finished_at": time.time(),
                        "robot_http_acknowledged": True,
                        "controller_command_acknowledged": controller_acknowledged,
                        "controller_terminal_state_verified": terminal_verified,
                        "physical_effect_verified": False,
                        "interrupt_epoch": interrupt_epoch,
                        "interrupted_command_ids": sorted(interrupted_ids),
                        "result": _json_safe(result),
                        "operator_assessment": None,
                    }
                    must_desync = bool(abort or interrupted_ids or not stop_verified or not generation_match)
                    authority_state_verified = True
                    if must_desync:
                        z.update(
                            {
                                "state": "failed_latched",
                                "active_receipt": None,
                                "awaiting_observation_receipt_id": None,
                                "reference_state": "desynced",
                                "last_failure": {
                                    "reason": (
                                        "z_abort_latched"
                                        if abort
                                        else "z_safety_stop_interrupted_active_intent"
                                        if interrupted_ids
                                        else "z_safety_stop_unverified"
                                        if not stop_verified
                                        else "z_safety_stop_generation_mismatch"
                                    ),
                                    "receipt": _json_safe(receipt),
                                },
                            }
                        )
                        try:
                            result["reference_desync"] = _json_safe(
                                self._z_mark_desynced(
                                    (
                                        "Z abort invalidated reference authority."
                                        if abort
                                        else "Z safety stop interrupted or could not conclusively preserve reference authority."
                                    ),
                                    f"serial206.z.{interrupt_intent}_interrupt",
                                )
                            )
                        except Exception as exc:
                            authority_state_verified = False
                            result["reference_desync_error"] = f"{type(exc).__name__}: {exc}"
                        if not authority_state_verified:
                            result["ok"] = False
                            receipt["status"] = "failed"
                        receipt["authority_state_verified"] = authority_state_verified
                        receipt["result"] = _json_safe(result)
                    durable_receipt = self._append_z_receipt(z, receipt)
                    state = self._save_state(state)
                    self._persist_z_receipt(durable_receipt)
                    final_z = state["z_lifecycle"]
                    return {
                        "ok": bool(stop_verified and authority_state_verified),
                        "result": _json_safe(result),
                        "authority_receipt": _json_safe(receipt),
                        "z_state": final_z.get("state"),
                        "z_lifecycle": self._z_lifecycle_projection(final_z),
                    }
            finally:
                with self._z_interrupt_state_lock:
                    self._z_interrupt_active = False

    def execute_z_intent(
        self,
        intent: str,
        *,
        inputs: Mapping[str, Any] | None = None,
        expected_generation: int,
        idempotency_key: str,
    ) -> dict[str, Any]:
        intent = str(intent).strip()
        values = dict(inputs or {})
        if intent in {"stop", "abort"}:
            return self.execute_z_stop_interrupt(
                inputs=values,
                expected_generation=expected_generation,
                idempotency_key=idempotency_key,
                abort=intent == "abort",
            )
        with self._z_interrupt_state_lock:
            admitted_interrupt_epoch = self._z_interrupt_epoch
            if self._z_interrupt_active:
                return {
                    "ok": False,
                    "blockers": ["z_safety_interrupt_in_progress"],
                }
        with self._lock:
            with self._z_interrupt_state_lock:
                if (
                    self._z_interrupt_active
                    or admitted_interrupt_epoch != self._z_interrupt_epoch
                ):
                    return {
                        "ok": False,
                        "blockers": ["z_intent_superseded_by_safety_interrupt"],
                    }
            state = self._load_state()
            z = state["z_lifecycle"]
            observed_generation = int(self.generation_provider())
            if z.get("generation") is not None and int(z["generation"]) != observed_generation:
                z.update({
                    "state": "unprepared",
                    "generation": None,
                    "board_lifecycle_generation": None,
                    "prepared_receipt": None,
                    "active_receipt": None,
                    "reference_state": "desynced",
                    "awaiting_observation_receipt_id": None,
                    "last_failure": {
                        "reason": "ownership_generation_changed",
                        "observed_generation": observed_generation,
                        "invalidated_at": time.time(),
                    },
                })
                self._z_mark_desynced(
                    "Ownership generation changed; Z preparation/reference invalidated.",
                    "serial206.z.generation_invalidation",
                )
                self._save_state(state)
            board_generation_provider = getattr(
                self.preparation_provider, "current_board_lifecycle_generation", None
            )
            current_board_generation = None
            if callable(board_generation_provider) and z.get("state") != "unprepared":
                stored_board_generation = z.get("board_lifecycle_generation")
                current_board_generation = board_generation_provider()
                if (
                    type(stored_board_generation) is not int
                    or current_board_generation != stored_board_generation
                ):
                    previous_state = str(z.get("state") or "unknown")
                    z.update({
                        "state": "unprepared",
                        "generation": None,
                        "board_lifecycle_generation": None,
                        "prepared_receipt": None,
                        "active_receipt": None,
                        "awaiting_observation_receipt_id": None,
                        "reference_state": "desynced",
                        "last_failure": {
                            "reason": "board_lifecycle_generation_changed",
                            "previous_state": previous_state,
                            "stored_board_generation": stored_board_generation,
                            "current_board_generation": current_board_generation,
                            "invalidated_at": time.time(),
                        },
                    })
                    self._z_mark_desynced(
                        "Board lifecycle generation changed; Z preparation/reference invalidated.",
                        "serial206.z.board_generation_invalidation",
                    )
                    self._save_state(state)
            safe_inputs = _json_safe(values)
            existing = next(
                (
                    row for row in reversed(z.get("receipts") or [])
                    if row.get("idempotency_key") == idempotency_key
                ),
                None,
            )
            if existing is None:
                existing = self._durable_serial206_receipt_by_idempotency("z", idempotency_key)
            if existing is not None:
                if (
                    existing.get("intent") != intent
                    or existing.get("inputs") != safe_inputs
                    or existing.get("expected_generation") != int(expected_generation)
                ):
                    return {
                        "ok": False,
                        "blockers": ["z_idempotency_key_bound_to_different_request"],
                        "authority_receipt": existing,
                    }
                replay_authority_valid = (
                    int(expected_generation) == observed_generation
                    and z.get("generation") == observed_generation
                    and z.get("state") not in {"unprepared", "failed_latched"}
                    and (
                        not callable(board_generation_provider)
                        or (
                            type(z.get("board_lifecycle_generation")) is int
                            and z.get("board_lifecycle_generation") == current_board_generation
                        )
                    )
                )
                if existing.get("status") == "completed" and not replay_authority_valid:
                    return {
                        "ok": False,
                        "replayed": True,
                        "blockers": ["z_replay_authority_invalidated"],
                        "authority_receipt": existing,
                    }
                return {"ok": existing.get("status") == "completed", "replayed": True, "authority_receipt": existing}
            supplied_command_id = values.get("command_id")
            command_id = (
                supplied_command_id
                if isinstance(supplied_command_id, str) and supplied_command_id.strip()
                else f"z_{int(time.time() * 1000)}_{len(z.get('receipts') or []) + 1}"
            )
            receipt = {
                "command_id": command_id,
                "intent": intent,
                "idempotency_key": idempotency_key,
                "expected_generation": int(expected_generation),
                "board_lifecycle_generation": z.get("board_lifecycle_generation"),
                "inputs": safe_inputs,
                "status": "pending_admission",
                "started_at": time.time(),
                "robot_http_acknowledged": True,
                "controller_command_acknowledged": False,
                "controller_terminal_state_verified": False,
                "physical_effect_verified": False,
                "operator_assessment": None,
            }
            self._append_z_receipt(z, receipt)
            self._save_state(state)

            observed_generation = int(self.generation_provider())
            allowed_by_state = {
                "prepare": {"unprepared", "failed_latched"},
                "reconcile_switch_masks": {"unprepared", "failed_latched"},
                "manual_home": {"prepared_unreferenced", "referenced_ready"},
                "move_z_home": {"prepared_unreferenced", "referenced_ready"},
                "diagnostic_home_axis": {"prepared_unreferenced", "referenced_ready"},
                "set_max_speed": {"prepared_unreferenced", "referenced_ready"},
                "set_max_acc": {"prepared_unreferenced", "referenced_ready"},
                "set_vmax": {"prepared_unreferenced", "referenced_ready"},
                "set_current_max": {"prepared_unreferenced", "referenced_ready"},
                "restore_original_speed": {"prepared_unreferenced", "referenced_ready"},
                "set_clean_path": {"prepared_unreferenced", "referenced_ready"},
                # ClassMotor.setHome is the no-motion state-establishing OEM
                # primitive.  It cannot require the prepared/reference state it
                # creates; otherwise a stale controller position has no canonical
                # recovery route.  Operator-plane generation ownership remains
                # mandatory for this mutation.
                "set_home": {"unprepared", "failed_latched", "prepared_unreferenced", "referenced_ready"},
                "move_steps": {"referenced_ready"},
                "move_absolute": {"referenced_ready"},
                "clear": {"referenced_ready"},
                "path_execute": {"referenced_ready"},
                "move_gz": {"referenced_ready"},
                "home_gz": {"referenced_ready"},
                "lower_pipette": {"referenced_ready"},
                "lift_pipette": {"referenced_ready"},
                "self_test": {"referenced_ready"},
                "resume_after_abort": {"failed_latched", "prepared_unreferenced", "referenced_ready"},
                "stop": {"unprepared", "prepared_unreferenced", "executing", "awaiting_operator_observation", "referenced_ready", "failed_latched"},
            }
            blockers = []
            set_home_admitted_board_generation = None
            if expected_generation != observed_generation:
                blockers.append(f"ownership_generation_mismatch:{expected_generation}:{observed_generation}")
            if intent not in allowed_by_state:
                blockers.append(f"unsupported_z_intent:{intent}")
            elif z.get("state") not in allowed_by_state[intent]:
                blockers.append(f"z_state_blocks_intent:{z.get('state')}:{intent}")
            if z.get("generation") is not None and int(z["generation"]) != observed_generation and intent not in {"prepare", "reconcile_switch_masks", "set_home", "stop"}:
                blockers.append("z_preparation_generation_stale")
            if intent == "prepare" and self.preparation_provider is None:
                blockers.append("z_preparation_provider_not_bound")
            if intent != "stop" and self.reference_store is None:
                blockers.append("z_reference_store_not_bound")
            if intent == "reconcile_switch_masks" and values.get("confirm") != "RECONCILE_Z_SWITCH_MASKS":
                blockers.append("explicit_confirmation_required:RECONCILE_Z_SWITCH_MASKS")

            if intent == "move_steps":
                steps_value = values.get("steps")
                if type(steps_value) is not int:
                    blockers.append("z_relative_steps_must_be_integer")
                elif steps_value == 0:
                    blockers.append("z_relative_steps_must_be_nonzero")
            if intent == "move_absolute" and type(values.get("position_steps")) is not int:
                blockers.append("z_absolute_position_must_be_integer")
            if intent == "path_execute":
                steps_value = values.get("steps")
                if not isinstance(steps_value, list) or not 1 <= len(steps_value) <= 256:
                    blockers.append("z_path_steps_must_be_nonempty_bounded_list")
                elif any(not isinstance(row, Mapping) for row in steps_value):
                    blockers.append("z_path_steps_must_be_objects")
                path_context = values.get("path_context")
                if not isinstance(path_context, Mapping):
                    blockers.append("z_path_context_must_be_object")
                elif type(path_context.get("current_location")) not in {str, int}:
                    blockers.append("z_path_context_location_must_be_string_or_integer")
                elif type(path_context.get("current_well")) is not int:
                    blockers.append("z_path_context_well_must_be_integer")
            if intent == "set_clean_path" and type(values.get("enabled")) is not bool:
                blockers.append("z_clean_path_enabled_must_be_boolean")
            if intent == "move_gz":
                if type(values.get("gripper_position_steps")) is not int:
                    blockers.append("move_gz_gripper_position_must_be_integer")
                if type(values.get("z_position_steps")) is not int:
                    blockers.append("move_gz_z_position_must_be_integer")
            if intent == "home_gz":
                x_lifecycle = state.get("x_lifecycle")
                if not isinstance(x_lifecycle, Mapping):
                    blockers.append("home_gz_x_lifecycle_missing")
                else:
                    if x_lifecycle.get("generation") != observed_generation:
                        blockers.append("home_gz_x_generation_stale")
                    if x_lifecycle.get("state") not in {"prepared_unreferenced", "referenced_ready"}:
                        blockers.append("home_gz_x_not_prepared")
                    if x_lifecycle.get("board_lifecycle_generation") != current_board_generation:
                        blockers.append("home_gz_x_board_lifecycle_generation_stale")
            if intent in {"lower_pipette", "lift_pipette"}:
                location_value = values.get("location_id")
                if type(location_value) not in {str, int}:
                    blockers.append(f"{intent}_location_id_must_be_string_or_integer")
            if intent == "lower_pipette" and type(values.get("overpress", False)) is not bool:
                blockers.append("lower_pipette_overpress_must_be_boolean")

            if intent in {"set_max_speed", "set_max_acc", "set_vmax"}:
                control_value = values.get("value")
                if type(control_value) is not int:
                    blockers.append(f"z_{intent}_value_must_be_integer")
                elif control_value < 0 or control_value > 2_147_483_647:
                    blockers.append(f"z_{intent}_value_out_of_int32_range")
            if intent == "set_current_max":
                control_value = values.get("value")
                if control_value is not None and type(control_value) is not int:
                    blockers.append("z_set_current_max_value_must_be_integer_or_null")
                elif type(control_value) is int and not (0 <= control_value <= 31 or control_value == 100):
                    blockers.append("z_set_current_max_value_out_of_range_0_31_or_oem_sentinel_100")
            if intent == "set_home":
                if values.get("operator_ack") != "SET_HOME_CURRENT_POSITION":
                    blockers.append("explicit_confirmation_required:SET_HOME_CURRENT_POSITION")
                board_generation_provider = getattr(
                    self.preparation_provider, "current_board_lifecycle_generation", None
                )
                current_board_generation = (
                    board_generation_provider() if callable(board_generation_provider) else None
                )
                set_home_admitted_board_generation = current_board_generation
                if type(current_board_generation) is not int or current_board_generation < 1:
                    blockers.append("z_board_lifecycle_generation_unavailable_for_set_home")
            if intent in {
                "manual_home", "move_z_home", "diagnostic_home_axis",
                "move_steps", "move_absolute", "clear", "path_execute", "move_gz", "home_gz",
                "lower_pipette", "lift_pipette", "self_test", "resume_after_abort",
                "set_max_speed", "set_max_acc",
                "set_vmax", "set_current_max", "restore_original_speed",
                "set_clean_path",
            }:
                prepared_board_generation = z.get("board_lifecycle_generation")
                board_generation_provider = getattr(
                    self.preparation_provider, "current_board_lifecycle_generation", None
                )
                current_board_generation = (
                    board_generation_provider()
                    if callable(board_generation_provider)
                    else prepared_board_generation
                )
                if (
                    type(prepared_board_generation) is not int
                    or current_board_generation != prepared_board_generation
                ):
                    blockers.append(
                        f"z_board_lifecycle_generation_stale:{prepared_board_generation}:{current_board_generation}"
                    )
            if blockers:
                receipt.update({"status": "rejected", "finished_at": time.time(), "blockers": blockers})
                durable_receipt = self._append_z_receipt(z, receipt)
                self._save_state(state)
                self._persist_z_receipt(durable_receipt)
                return {"ok": False, "blockers": blockers, "authority_receipt": receipt}

            previous_state = z.get("state")
            home_gz_prior_x_state: str | None = None
            home_gz_prior_x_reference: str | None = None
            receipt["status"] = "executing"
            z["active_receipt"] = copy.deepcopy(receipt)
            z["state"] = "executing"
            if intent == "home_gz":
                x_lifecycle = state["x_lifecycle"]
                home_gz_prior_x_state = str(x_lifecycle.get("state") or "unprepared")
                home_gz_prior_x_reference = str(x_lifecycle.get("reference_state") or "desynced")
                x_lifecycle.update({
                    "state": "executing",
                    "active_receipt": {
                        "command_id": f"{command_id}:caught-plate-x",
                        "intent": "caught_plate_recovery_home",
                        "status": "executing",
                        "parent_home_gz_command_id": command_id,
                    },
                    "pending_ticket": None,
                })
            self._save_state(state)
            expected_noop_requested: int | None = None
            expected_noop_pseudo_home: int | None = None
            expected_noop_effective_target: int | None = None
            try:
                if intent == "prepare":
                    preparer = self.preparation_provider
                    if preparer is None:
                        raise RuntimeError("Z preparation provider is not bound")
                    transition_scope = {
                        "thread_id": threading.get_ident(),
                        "command_id": command_id,
                        "expected": [False, True],
                        "violation": None,
                    }
                    self._board_transition_scope = transition_scope
                    try:
                        result = preparer.prepare_for_initialize_motors(
                            expected_generation=observed_generation
                        )
                    finally:
                        self._board_transition_scope = None
                    if transition_scope.get("violation") is not None:
                        result = {
                            "ok": False,
                            "failure": "z_prepare_board_transition_scope_violation",
                            "scope_violation": _json_safe(transition_scope.get("violation")),
                        }
                    elif transition_scope.get("expected"):
                        result = {
                            "ok": False,
                            "failure": "z_prepare_board_transition_sequence_incomplete",
                            "missing_transitions": list(transition_scope["expected"]),
                        }
                elif intent == "reconcile_switch_masks":
                    result = self.primitives.z_reconcile_switch_masks()
                elif intent == "manual_home":
                    result = self.primitives.z_manual_home(timeout_s=float(values.get("timeout_s", 30.0)))
                elif intent == "move_z_home":
                    result = self.primitives.z_move_z_home(timeout_s=float(values.get("timeout_s", 30.0)))
                elif intent == "set_home":
                    result = self.primitives.z_set_home()
                elif intent == "diagnostic_home_axis":
                    result = self.primitives.z_diagnostic_home_axis(timeout_s=float(values.get("timeout_s", 30.0)))
                elif intent == "move_steps":
                    result = self.primitives.z_move_steps(
                        steps=int(values["steps"]),
                        wait_timeout_s=float(values.get("wait_timeout_s", 20.0)),
                    )
                elif intent == "move_absolute":
                    machine_status = state.get("machine_status") or {}
                    pseudo_home = machine_status.get("psudo_z_home_steps")
                    if type(pseudo_home) is not int or pseudo_home not in {500, 65000}:
                        raise RuntimeError("PSUDO_Z_HOME state is invalid")
                    expected_noop_requested = int(values["position_steps"])
                    expected_noop_pseudo_home = int(pseudo_home)
                    expected_noop_effective_target = max(
                        expected_noop_pseudo_home,
                        expected_noop_requested,
                    )
                    result = self.primitives.z_move_absolute(
                        requested_position_steps=expected_noop_requested,
                        pseudo_home_steps=expected_noop_pseudo_home,
                        wait_timeout_s=float(values.get("wait_timeout_s", 20.0)),
                    )
                elif intent == "clear":
                    machine_status = state.get("machine_status") or {}
                    pseudo_home = machine_status.get("psudo_z_home_steps")
                    if type(pseudo_home) is not int or pseudo_home not in {500, 65000}:
                        raise RuntimeError("PSUDO_Z_HOME state is invalid")
                    expected_noop_requested = int(pseudo_home)
                    expected_noop_pseudo_home = int(pseudo_home)
                    expected_noop_effective_target = int(pseudo_home)
                    result = self.primitives.z_move_absolute(
                        requested_position_steps=expected_noop_requested,
                        pseudo_home_steps=expected_noop_pseudo_home,
                        wait_timeout_s=float(values.get("wait_timeout_s", 20.0)),
                    )
                    if isinstance(result, Mapping):
                        result = {**dict(result), "selected_pseudo_home_steps": int(pseudo_home)}
                elif intent == "path_execute":
                    machine_status = state.get("machine_status") or {}
                    pseudo_home = machine_status.get("psudo_z_home_steps")
                    if type(pseudo_home) is not int or pseudo_home not in {500, 65000}:
                        raise RuntimeError("PSUDO_Z_HOME state is invalid")
                    result = self.primitives.z_execute_path(
                        steps=[dict(row) for row in values["steps"]],
                        wait_timeout_s=float(values.get("wait_timeout_s", 20.0)),
                        pseudo_home_steps=int(pseudo_home),
                        parent_command_id=command_id,
                    )
                elif intent == "move_gz":
                    result = self.primitives.z_move_gz(
                        gripper_position_steps=int(values["gripper_position_steps"]),
                        z_position_steps=int(values["z_position_steps"]),
                        wait_timeout_s=float(values.get("wait_timeout_s", 5.0)),
                    )
                elif intent == "home_gz":
                    machine_status = state.get("machine_status") or {}
                    pseudo_home = machine_status.get("psudo_z_home_steps")
                    if type(pseudo_home) is not int or pseudo_home not in {500, 65000}:
                        raise RuntimeError("PSUDO_Z_HOME state is invalid")
                    result = self.primitives.z_home_gz(
                        pseudo_z_home_steps=int(pseudo_home),
                        delay_s=int(values.get("delay_s", 0)),
                        wait_timeout_s=float(values.get("wait_timeout_s", 30.0)),
                    )
                elif intent in {"lower_pipette", "lift_pipette"}:
                    result = self.primitives.z_pipette_position(
                        location_id=values["location_id"],
                        operation=intent,
                        overpress=bool(values.get("overpress", False)),
                    )
                elif intent == "self_test":
                    machine_status = state.get("machine_status") or {}
                    pseudo_home = machine_status.get("psudo_z_home_steps")
                    if type(pseudo_home) is not int or pseudo_home not in {500, 65000}:
                        raise RuntimeError("PSUDO_Z_HOME state is invalid")
                    result = self.primitives.z_self_test(
                        pseudo_z_home_steps=int(pseudo_home),
                        wait_timeout_s=float(values.get("wait_timeout_s", 30.0)),
                    )
                elif intent == "resume_after_abort":
                    result = self.primitives.z_resume_after_abort(
                        timeout_s=float(values.get("wait_timeout_s", 30.0))
                    )
                elif intent == "set_max_speed":
                    result = self.primitives.z_set_max_speed(int(values.get("value", 0)))
                elif intent == "set_max_acc":
                    result = self.primitives.z_set_max_acc(int(values.get("value", 0)))
                elif intent == "set_vmax":
                    result = self.primitives.z_set_vmax(int(values.get("value", 0)))
                elif intent == "set_current_max":
                    raw_value = values.get("value")
                    result = self.primitives.z_set_current_max(
                        None if raw_value is None else int(raw_value)
                    )
                elif intent == "restore_original_speed":
                    result = self.primitives.z_restore_original_speed()
                elif intent == "set_clean_path":
                    result = {
                        "ok": True,
                        "source_state": "m_controlLib.cleanPath",
                        "clean_path": bool(values["enabled"]),
                        "controller_command_acknowledged": True,
                        "controller_terminal_state_verified": True,
                        "motion_commanded": False,
                    }
                else:
                    result = self.primitives.z_stop()
            except Exception as exc:
                result = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}

            result = dict(result) if isinstance(result, Mapping) else {
                "ok": False,
                "failure": "z_result_not_mapping",
            }
            with self._z_interrupt_state_lock:
                current_interrupt_epoch = self._z_interrupt_epoch
                interrupted_by_safety = bool(
                    self._z_interrupt_active
                    or admitted_interrupt_epoch != current_interrupt_epoch
                )
            if interrupted_by_safety:
                result.update(
                    {
                        "ok": False,
                        "failure": "z_intent_interrupted_by_safety_stop",
                        "interrupt_epoch": current_interrupt_epoch,
                    }
                )
            ok = result.get("ok") is True
            x_recovery_receipt: dict[str, Any] | None = None
            if intent == "home_gz":
                raw_home_gz = result.get("result")
                recovery = raw_home_gz.get("recovery") if isinstance(raw_home_gz, Mapping) else None
                x_home = recovery.get("x_home") if isinstance(recovery, Mapping) else None
                if isinstance(x_home, Mapping):
                    x_lifecycle = state["x_lifecycle"]
                    x_recovery_ok = bool(
                        x_home.get("ok") is True
                        and x_home.get("home_predicate_confirmed") is True
                        and x_home.get("controller_terminal_state_verified") is True
                    )
                    x_recovery_command_id = f"{command_id}:caught-plate-x"
                    x_recovery_receipt = {
                        "command_id": x_recovery_command_id,
                        "receipt_id": x_recovery_command_id,
                        "intent": "caught_plate_recovery_home",
                        "idempotency_key": f"{idempotency_key}:caught-plate-x",
                        "idempotency_replay_enabled": True,
                        "generation": observed_generation,
                        "board_lifecycle_generation": x_lifecycle.get("board_lifecycle_generation"),
                        "inputs": {"parent_home_gz_command_id": command_id},
                        "status": "completed" if x_recovery_ok else "failed",
                        "result": _json_safe(x_home),
                    }
                    x_lifecycle.update({
                        "state": "awaiting_operator_observation" if x_recovery_ok else "failed_latched",
                        "active_receipt": None,
                        "pending_ticket": None,
                        "reference_state": "desynced",
                        "awaiting_observation_receipt_id": x_recovery_command_id if x_recovery_ok else None,
                        "last_failure": None if x_recovery_ok else _json_safe(x_recovery_receipt),
                    })
                    x_lifecycle["receipts"].append(x_recovery_receipt)
                    x_lifecycle["receipts"] = x_lifecycle["receipts"][-8:]
                elif result.get("error") is not None:
                    x_lifecycle = state["x_lifecycle"]
                    x_lifecycle.update({
                        "state": "failed_latched",
                        "active_receipt": None,
                        "pending_ticket": None,
                        "reference_state": "desynced",
                        "awaiting_observation_receipt_id": None,
                        "last_failure": {
                            "reason": "home_gz_x_recovery_outcome_ambiguous",
                            "parent_home_gz_command_id": command_id,
                            "provider_error": str(result.get("error")),
                        },
                    })
                else:
                    x_lifecycle = state["x_lifecycle"]
                    x_lifecycle.update({
                        "state": home_gz_prior_x_state,
                        "active_receipt": None,
                        "pending_ticket": None,
                        "reference_state": home_gz_prior_x_reference,
                    })
            if ok and intent == "prepare":
                self.primitives.z_clear_profile_overrides()
                board_generation = result.get("board_lifecycle_generation")
                board_generation_provider = getattr(
                    self.preparation_provider, "current_board_lifecycle_generation", None
                )
                if type(board_generation) is not int and callable(board_generation_provider):
                    board_generation = board_generation_provider()
                if type(board_generation) is not int or board_generation < 1:
                    result.update({
                        "ok": False,
                        "failure": "z_preparation_board_lifecycle_generation_unverified",
                    })
                    ok = False
                else:
                    result["board_lifecycle_generation"] = int(board_generation)
            if ok and intent in {
                "diagnostic_home_axis", "set_home", "move_steps", "move_absolute", "clear", "path_execute",
                "move_gz", "home_gz", "lower_pipette", "lift_pipette", "stop",
                "self_test", "resume_after_abort",
                "set_max_speed", "set_max_acc", "set_vmax", "set_current_max",
                "restore_original_speed",
            }:
                noop_effective = result.get("effective_position_steps")
                verified_no_command_noop = bool(
                    intent in {"move_absolute", "clear"}
                    and type(expected_noop_requested) is int
                    and type(expected_noop_pseudo_home) is int
                    and type(expected_noop_effective_target) is int
                    and result.get("source_noop") is True
                    and result.get("noop_reason") == "already_at_effective_target"
                    and result.get("physical_motion_commanded") is False
                    and result.get("controller_command_acknowledged") is False
                    and result.get("controller_terminal_state_verified") is True
                    and type(noop_effective) is int
                    and result.get("requested_position_steps") == expected_noop_requested
                    and result.get("pseudo_home_steps") == expected_noop_pseudo_home
                    and noop_effective == expected_noop_effective_target
                    and result.get("target_position_steps") == expected_noop_effective_target
                    and result.get("before_position_steps") == expected_noop_effective_target
                    and result.get("after_position_steps") == expected_noop_effective_target
                )
                if (
                    (
                        result.get("controller_command_acknowledged") is not True
                        and not verified_no_command_noop
                    )
                    or result.get("controller_terminal_state_verified") is not True
                ):
                    result.update({
                        "ok": False,
                        "failure": result.get("failure") or "z_controller_command_or_terminal_evidence_unverified",
                    })
                    ok = False
            if ok and intent in {"manual_home", "move_z_home"}:
                summary = result.get("home_summary")
                source_short_circuit = bool(
                    isinstance(summary, Mapping)
                    and summary.get("short_circuit") == "MotorHome_and_CurrentPosition_zero"
                )
                evidence_verified = bool(
                    result.get("controller_terminal_state_verified") is True
                    and (
                        (
                            source_short_circuit
                            and isinstance(summary, Mapping)
                            and summary.get("controller_home_proof_verified") is True
                        )
                        or (
                            not source_short_circuit
                            and result.get("controller_command_acknowledged") is True
                        )
                    )
                )
                if not evidence_verified:
                    result.update({
                        "ok": False,
                        "failure": result.get("failure") or "z_manual_home_controller_evidence_unverified",
                    })
                    ok = False
            if ok and intent in {
                "prepare", "set_home", "manual_home", "move_z_home",
                "diagnostic_home_axis", "move_steps", "move_absolute", "clear",
                "self_test", "resume_after_abort", "stop",
            }:
                terminal_reader = getattr(self.primitives, "z_terminal_status", None)
                if callable(terminal_reader):
                    try:
                        terminal_state = terminal_reader()
                    except Exception as exc:
                        terminal_state = {
                            "ok": False,
                            "error": f"{type(exc).__name__}: {exc}",
                            "authority": "serial206_terminal_register_readback",
                        }
                    result["terminal_z_state"] = _json_safe(terminal_state)
                    if terminal_state.get("ok") is True:
                        z["terminal_state"] = {
                            key: terminal_state.get(key)
                            for key in (
                                "position_steps", "speed_steps_s",
                                "left_switch_state", "right_switch_state",
                                "left_switch_disabled", "right_switch_disabled",
                                "authority",
                            )
                        }
                        z["terminal_state"].update({
                            "source_command_id": command_id,
                            "observed_at": time.time(),
                        })
                    if terminal_state.get("ok") is not True:
                        result["terminal_state_refresh_error"] = (
                            terminal_state.get("error") or "terminal Z register readback was incomplete"
                        )
            if ok and intent not in {"reconcile_switch_masks", "stop"}:
                final_generation = int(self.generation_provider())
                final_board_generation_provider = getattr(
                    self.preparation_provider, "current_board_lifecycle_generation", None
                )
                final_board_generation = (
                    final_board_generation_provider()
                    if callable(final_board_generation_provider)
                    else None
                )
                expected_final_board_generation = (
                    result.get("board_lifecycle_generation")
                    if intent == "prepare"
                    else set_home_admitted_board_generation
                    if intent == "set_home"
                    else z.get("board_lifecycle_generation")
                )
                if final_generation != observed_generation:
                    result.update(
                        {
                            "ok": False,
                            "failure": "z_ownership_generation_changed_during_intent",
                            "final_generation": final_generation,
                        }
                    )
                    ok = False
                elif (
                    type(expected_final_board_generation) is not int
                    or final_board_generation != expected_final_board_generation
                ):
                    result.update(
                        {
                            "ok": False,
                            "failure": "z_board_lifecycle_generation_changed_during_intent",
                            "expected_board_lifecycle_generation": expected_final_board_generation,
                            "final_board_lifecycle_generation": final_board_generation,
                        }
                    )
                    ok = False
            reference_published = False
            reference_intents = {
                "set_home",
                "manual_home",
                "move_z_home",
                "diagnostic_home_axis",
                "self_test",
                "resume_after_abort",
            }
            if ok and intent == "path_execute" and result.get("z_home_reference_verified") is True:
                reference_intents.add("path_execute")
            if ok and intent in reference_intents:
                try:
                    reference = self._z_mark_referenced(
                        source=f"serial206.z.{intent}",
                        motion_kind="manual_set_home" if intent == "set_home" else "controller_proven_home",
                    )
                except Exception as exc:
                    result.update(
                        {
                            "ok": False,
                            "failure": f"z_reference_publication_failed:{type(exc).__name__}:{exc}",
                        }
                    )
                    ok = False
                else:
                    reference_published = True
                    result["reference_persistence"] = _json_safe(reference)
            if not ok and intent not in {"stop", "set_home"} and not interrupted_by_safety:
                try:
                    result = {**dict(result), "failure_stop": _json_safe(self.primitives.z_stop())}
                except Exception as exc:
                    result = {**dict(result), "failure_stop": {"ok": False, "error": f"{type(exc).__name__}: {exc}"}}
            elif interrupted_by_safety:
                result = {
                    **dict(result),
                    "failure_stop": {
                        "delegated_to_safety_interrupt": True,
                        "interrupt_epoch": current_interrupt_epoch,
                    },
                }
            result_summary = {
                "ok": bool(isinstance(result, Mapping) and result.get("ok") is True),
                "failure": result.get("failure") if isinstance(result, Mapping) else None,
                "source_return_code": result.get("source_return_code") if isinstance(result, Mapping) else None,
                "travel_error_steps": result.get("travel_error_steps") if isinstance(result, Mapping) else None,
                "self_test_pass": result.get("self_test_pass") if isinstance(result, Mapping) else None,
                "initial_home_ok": bool(isinstance(result, Mapping) and isinstance(result.get("initial_move_z_home"), Mapping) and result["initial_move_z_home"].get("ok") is True),
                "move_ok": bool(isinstance(result, Mapping) and isinstance(result.get("move"), Mapping) and result["move"].get("ok") is True),
                "final_home_ok": bool(isinstance(result, Mapping) and isinstance(result.get("home"), Mapping) and result["home"].get("ok") is True),
            }
            receipt.update({
                "status": "completed" if ok else "failed",
                "finished_at": time.time(),
                "result_summary": result_summary,
                "result": _json_safe(result),
                "controller_command_acknowledged": bool(isinstance(result, Mapping) and result.get("controller_command_acknowledged") is True),
                "controller_terminal_state_verified": bool(isinstance(result, Mapping) and result.get("controller_terminal_state_verified") is True),
            })
            terminal_state = self._z_terminal_state_from_result(
                result,
                command_id=str(receipt["command_id"]),
                observed_at=float(receipt["finished_at"]),
            )
            if terminal_state is not None:
                z["terminal_state"] = terminal_state
            z["active_receipt"] = None
            if ok and intent == "prepare":
                board_generation = int(result["board_lifecycle_generation"])
                z.update({
                    "state": "prepared_unreferenced",
                    "generation": observed_generation,
                    "board_lifecycle_generation": board_generation,
                    "prepared_receipt": _json_safe(receipt),
                    "reference_state": "desynced",
                    "last_failure": None,
                })
                self._z_mark_desynced("Z profile prepared; a source home or explicit setHome is required.", "serial206.z.prepare")
            elif ok and intent == "reconcile_switch_masks":
                z.update({
                    "state": "unprepared",
                    "generation": None,
                    "board_lifecycle_generation": None,
                    "prepared_receipt": None,
                    "active_receipt": None,
                    "awaiting_observation_receipt_id": None,
                    "reference_state": "desynced",
                    "last_failure": None,
                })
                self._z_mark_desynced("Z switch masks reconciled; source profile must be prepared again.", "serial206.z.reconcile_switch_masks")
            elif ok and intent == "set_home":
                board_generation_provider = getattr(
                    self.preparation_provider, "current_board_lifecycle_generation", None
                )
                set_home_board_generation = (
                    board_generation_provider() if callable(board_generation_provider) else None
                )
                z.update({
                    "state": "referenced_ready",
                    "generation": observed_generation,
                    "board_lifecycle_generation": set_home_board_generation,
                    "reference_state": "referenced",
                    "awaiting_observation_receipt_id": None,
                    "last_failure": None,
                })
            elif ok and intent in {
                "manual_home", "move_z_home", "diagnostic_home_axis",
                "self_test", "resume_after_abort",
            }:
                z.update({
                    "state": "referenced_ready",
                    "awaiting_observation_receipt_id": None,
                    "reference_state": "referenced",
                    "last_failure": None,
                })
            elif ok and intent in {
                "move_steps", "move_absolute", "clear", "move_gz", "home_gz",
                "lower_pipette", "lift_pipette",
            }:
                z.update({"state": "referenced_ready", "reference_state": "referenced", "last_failure": None})
            elif ok and intent == "path_execute":
                path_context = dict(values["path_context"])
                machine_status = state.setdefault("machine_status", {})
                machine_status["current_location"] = path_context["current_location"]
                machine_status["current_well"] = path_context["current_well"]
                result["path_context_persisted"] = _json_safe(path_context)
                if result.get("z_home_reference_verified") is True:
                    z.update({
                        "state": "referenced_ready",
                        "awaiting_observation_receipt_id": None,
                        "reference_state": "referenced",
                        "last_failure": None,
                    })
                else:
                    z.update({"state": "referenced_ready", "reference_state": "referenced", "last_failure": None})
            elif ok and intent == "set_clean_path":
                machine_status = state.setdefault("machine_status", {})
                machine_status["clean_path"] = bool(values["enabled"])
                result["clean_path_persisted"] = True
                z.update({"state": previous_state, "last_failure": None})
            elif ok and intent in {
                "set_max_speed", "set_max_acc", "set_vmax", "set_current_max",
                "restore_original_speed",
            }:
                z.update({"state": previous_state, "last_failure": None})
            elif ok and intent == "stop":
                z["state"] = previous_state if previous_state != "executing" else "failed_latched"
            else:
                z.update({
                    "state": "failed_latched",
                    "reference_state": "desynced",
                    "last_failure": _json_safe(receipt),
                })
                self._z_mark_desynced(f"Failed serial-206 Z intent {intent}.", f"serial206.z.{intent}")
            durable_receipt = self._append_z_receipt(z, receipt)
            try:
                state = self._save_state(state)
            except Exception:
                if reference_published:
                    self._z_mark_desynced(
                        "Final Z lifecycle commit failed after reference publication; authority was compensated to desynced.",
                        "serial206.z.reference_commit_compensation",
                    )
                raise
            self._persist_z_receipt(durable_receipt)
            if (
                x_recovery_receipt is not None
                and self.state_store is not None
                and hasattr(self.state_store, "append_serial206_receipt")
            ):
                self.state_store.append_serial206_receipt("x", x_recovery_receipt)
            z = state["z_lifecycle"]
            return {"ok": ok, "result_summary": result_summary, "result": _json_safe(result), "authority_receipt": _json_safe(receipt), "z_state": z.get("state"), "z_lifecycle": self._z_lifecycle_projection(z)}

    def record_z_observation(
        self,
        *,
        command_id: str,
        observation_command_id: str | None = None,
        verdict: str,
        physical_motion_observed: bool,
        expected_direction_observed: bool,
        home_endpoint_observed: bool,
        stopped_observed: bool,
        note: str,
        expected_generation: int,
    ) -> dict[str, Any]:
        if verdict not in {"pass", "fail"}:
            raise ValueError("Z observation verdict must be pass or fail")
        for name, value in (
            ("physical_motion_observed", physical_motion_observed),
            ("expected_direction_observed", expected_direction_observed),
            ("home_endpoint_observed", home_endpoint_observed),
            ("stopped_observed", stopped_observed),
        ):
            if type(value) is not bool:
                raise ValueError(f"{name} must be a strict boolean")
        with self._lock:
            state = self._load_state()
            z = state["z_lifecycle"]
            receipts = list(z.get("receipts") or [])
            match_row = next((row for row in reversed(receipts) if row.get("command_id") == command_id), None)
            if match_row is None:
                match_row = self._durable_serial206_receipt("z", command_id)
            if match_row is None:
                raise ValueError("Z authority receipt is missing")
            match: dict[str, Any] = match_row
            observation_id = (
                observation_command_id.strip()
                if isinstance(observation_command_id, str) and observation_command_id.strip()
                else f"zobs_{int(time.time() * 1000)}_{len(receipts) + 1}"
            )
            existing_observation = next(
                (row for row in reversed(receipts) if row.get("command_id") == observation_id),
                None,
            )
            if existing_observation is None:
                existing_observation = self._durable_serial206_receipt("z", observation_id)
            if existing_observation is not None:
                if existing_observation.get("observes_command_id") != command_id:
                    raise ValueError("Z observation command is bound to a different authority receipt")
                return {
                    "ok": existing_observation.get("status") == "completed",
                    "replayed": True,
                    "observation_receipt": _json_safe(existing_observation),
                    "authority_receipt": _json_safe(match),
                    "z_state": z.get("state"),
                }

            home_result = match.get("result") if isinstance(match.get("result"), Mapping) else {}
            home_summary_value = home_result.get("home_summary") if isinstance(home_result, Mapping) else None
            home_summary: Mapping[str, Any] = (
                home_summary_value if isinstance(home_summary_value, Mapping) else {}
            )
            source_short_circuit = home_summary.get("short_circuit") == "MotorHome_and_CurrentPosition_zero"
            controller_home_proof = bool(
                isinstance(home_result, Mapping)
                and home_result.get("ok") is True
                and home_result.get("controller_terminal_state_verified") is True
                and (
                    home_result.get("controller_command_acknowledged") is True
                    or (
                        source_short_circuit
                        and home_summary.get("controller_home_proof_verified") is True
                    )
                )
            )
            authority_current = bool(
                match.get("status") == "completed"
                and controller_home_proof
                and z.get("state") == "awaiting_operator_observation"
                and z.get("awaiting_observation_receipt_id") == command_id
            )
            observation = {
                "command_id": observation_id,
                "observes_command_id": command_id,
                "verdict": verdict,
                "note": str(note).strip(),
                "observed_at": time.time(),
                "physical_motion_observed": physical_motion_observed,
                "expected_direction_observed": expected_direction_observed,
                "home_endpoint_observed": home_endpoint_observed,
                "stopped_observed": stopped_observed,
                "physical_effect_verified": physical_motion_observed,
                "authority_current": authority_current,
                "reference_eligible": False,
            }
            observation_receipt = {
                **observation,
                "intent": "observation",
                "status": "completed",
                "expected_generation": int(expected_generation),
                "board_lifecycle_generation": match.get("board_lifecycle_generation"),
                "controller_command_acknowledged": False,
                "controller_terminal_state_verified": False,
            }

            # A movement-only observation of a failed/obsolete command is useful
            # historical evidence but can never publish reference authority.
            if not authority_current:
                match["operator_assessment"] = _json_safe(observation)
                match["physical_effect_verified"] = bool(physical_motion_observed)
                match["observation_receipt_id"] = observation_id
                z["receipts"] = receipts[-128:]
                durable_observation = self._append_z_receipt(z, observation_receipt)
                z["last_observation"] = _json_safe(observation)
                state = self._save_state(state)
                self._persist_z_receipt(durable_observation)
                if self.state_store is not None and hasattr(
                    self.state_store, "append_serial206_receipt"
                ):
                    self.state_store.append_serial206_receipt("z", match)
                return {
                    "ok": True,
                    "annotation_only": True,
                    "observation": observation,
                    "observation_receipt": _json_safe(observation_receipt),
                    "authority_receipt": _json_safe(match),
                    "z_state": state["z_lifecycle"].get("state"),
                    "z_lifecycle": self._z_lifecycle_projection(state["z_lifecycle"]),
                }

            current_generation = int(self.generation_provider())
            if int(expected_generation) != current_generation:
                raise ValueError("ownership generation changed before Z observation")
            if type(z.get("generation")) is not int or int(z["generation"]) != current_generation:
                raise ValueError("prepared ownership generation changed before Z observation")
            board_generation_provider = getattr(
                self.preparation_provider, "current_board_lifecycle_generation", None
            )
            current_board_generation = (
                board_generation_provider()
                if callable(board_generation_provider)
                else z.get("board_lifecycle_generation")
            )
            if (
                type(z.get("board_lifecycle_generation")) is not int
                or current_board_generation != z.get("board_lifecycle_generation")
                or match.get("board_lifecycle_generation") != z.get("board_lifecycle_generation")
            ):
                raise ValueError("board lifecycle generation changed before Z observation")

            reference_published = False
            operation_ok = False
            error = None
            if verdict == "pass":
                reference_eligible = bool(
                    (physical_motion_observed or source_short_circuit)
                    and expected_direction_observed
                    and home_endpoint_observed
                    and stopped_observed
                    and controller_home_proof
                )
                observation["source_already_home_short_circuit"] = source_short_circuit
                observation["reference_eligible"] = reference_eligible
                observation_receipt["source_already_home_short_circuit"] = source_short_circuit
                observation_receipt["reference_eligible"] = reference_eligible
                if reference_eligible:
                    try:
                        reference = self._z_mark_referenced(
                            source="serial206.z.operator_observation",
                            motion_kind="home",
                        )
                    except Exception as exc:
                        publication_error = f"{type(exc).__name__}: {exc}"
                        error = "z_reference_persistence_failed"
                        observation["reference_persistence_error"] = publication_error
                        observation_receipt["reference_persistence_error"] = publication_error
                        z.update(
                            {
                                "state": "failed_latched",
                                "reference_state": "desynced",
                                "awaiting_observation_receipt_id": None,
                                "last_failure": {
                                    "reason": error,
                                    "detail": publication_error,
                                },
                            }
                        )
                    else:
                        reference_published = True
                        operation_ok = True
                        observation["reference_persistence"] = _json_safe(reference)
                        observation_receipt["reference_persistence"] = _json_safe(reference)
                        z.update(
                            {
                                "state": "referenced_ready",
                                "reference_state": "referenced",
                                "awaiting_observation_receipt_id": None,
                                "last_failure": None,
                            }
                        )
                else:
                    error = "z_observation_not_reference_eligible"
            else:
                error = "operator_rejected_z_home"
                reference_invalidation = self._z_mark_desynced(
                    "Operator rejected physical Z home observation.",
                    "serial206.z.operator_observation",
                )
                observation["reference_invalidation"] = _json_safe(reference_invalidation)
                observation_receipt["reference_invalidation"] = _json_safe(reference_invalidation)
                z.update(
                    {
                        "state": "failed_latched",
                        "reference_state": "desynced",
                        "awaiting_observation_receipt_id": None,
                        "last_failure": _json_safe(observation),
                    }
                )

            match["operator_assessment"] = _json_safe(observation)
            match["physical_effect_verified"] = bool(physical_motion_observed)
            match["observation_receipt_id"] = observation_id
            z["receipts"] = receipts[-128:]
            durable_observation = self._append_z_receipt(z, observation_receipt)
            z["last_observation"] = _json_safe(observation)
            try:
                state = self._save_state(state)
            except Exception:
                if reference_published:
                    self._z_mark_desynced(
                        "Final Z observation lifecycle commit failed after reference publication.",
                        "serial206.z.observation_commit_compensation",
                    )
                raise
            self._persist_z_receipt(durable_observation)
            if self.state_store is not None and hasattr(
                self.state_store, "append_serial206_receipt"
            ):
                self.state_store.append_serial206_receipt("z", match)
            return {
                "ok": operation_ok,
                "error": error,
                "annotation_only": False,
                "observation": observation,
                "observation_receipt": _json_safe(observation_receipt),
                "authority_receipt": _json_safe(match),
                "z_state": state["z_lifecycle"].get("state"),
                "z_lifecycle": self._z_lifecycle_projection(state["z_lifecycle"]),
            }

    def initialize_motors_admission_projection(self) -> dict[str, Any]:
        """Publish exact next-stage inputs instead of capability-only availability."""
        with self._lock:
            try:
                state = self._load_state()
            except Exception:
                return {
                    "available": False,
                    "blockers": ["durable_serial206_state_corrupt"],
                    "expected_stage": None,
                }
            ledger = state["movement_ledger"]
            expected = ledger.get("expected_next_stage")
            if expected is None:
                return {
                    "available": False,
                    "blockers": ["initialize_motors_complete"],
                    "expected_stage": None,
                }
            spec = _SPEC_BY_KEY[expected]
            row = ledger["stages"][expected]
            blockers = []
            if row.get("state") == "acknowledged":
                blockers.append(f"operator_observation_required:{expected}")
            elif row.get("state") != "pending":
                blockers.append(f"stage_not_pending:{expected}:{row.get('state')}")
            gates = ["direction_verified", "limits_verified", "switch_verified", "stop_verified"]
            if not spec.establishes_reference:
                gates.append("reference_verified")
            return {
                "available": not blockers,
                "blockers": blockers,
                "expected_stage": expected,
                "stage_state": row.get("state"),
                "approval": {
                    "expected_component": spec.component,
                    "expected_direction": spec.direction,
                    "expected_bound": spec.bound,
                    "expected_generation": int(self.generation_provider()),
                },
                "commissioning": {
                    "required_components": [spec.component] if spec.component in _COMMISSIONED_COMPONENTS else [],
                    "required_gates": gates if spec.component in _COMMISSIONED_COMPONENTS else [],
                    "reference_established_by_stage": bool(spec.establishes_reference),
                    "z_required_polarity": {"gap9": 1, "gap10": 0} if spec.component == "z" else None,
                },
            }

    @staticmethod
    def _stage_status_projection(row: Mapping[str, Any]) -> dict[str, Any]:
        fields = (
            "stage",
            "component",
            "state",
            "status",
            "ok",
            "failure",
            "error",
            "approval_id",
            "command_id",
            "observed_command_id",
            "started_at",
            "finished_at",
            "duration_ms",
            "controller_command_acknowledged",
            "controller_terminal_state_verified",
            "physical_effect_verified",
        )
        return {
            key: _json_safe(row.get(key))
            for key in fields
            if key in row
        }

    @classmethod
    def _ledger_status_projection(cls, ledger: Mapping[str, Any]) -> dict[str, Any]:
        projection = {
            str(key): _json_safe(value)
            for key, value in ledger.items()
            if key not in {"stages", "stage_receipts", "raw_result", "result", "receipt"}
        }
        stages = ledger.get("stages")
        if isinstance(stages, Mapping):
            projection["stages"] = {
                str(key): cls._stage_status_projection(value)
                for key, value in stages.items()
                if isinstance(value, Mapping)
            }
        receipts = ledger.get("stage_receipts")
        if isinstance(receipts, list):
            projected_receipts = [
                cls._stage_status_projection(row)
                for row in receipts
                if isinstance(row, Mapping)
            ]
            projection["stage_receipt_count"] = len(projected_receipts)
            projection["latest_stage_receipt"] = projected_receipts[-1] if projected_receipts else None
        projection["detail_source"] = "robot_local_state_and_command_evidence"
        projection["detail_on_request"] = True
        return projection

    def initialize_motion_projection(self) -> dict[str, Any]:
        try:
            with self._lock:
                state = self._load_state()
            machine_status = copy.deepcopy(state["machine_status"])
            errors = machine_status.get("error_events")
            if isinstance(errors, list):
                machine_status["error_event_count"] = len(errors)
                machine_status["error_events"] = [
                    self._stage_status_projection(row)
                    for row in errors[-4:]
                    if isinstance(row, Mapping)
                ]
            return {
                "initialize_motion_ledger": self._ledger_status_projection(state["initialize_motion_ledger"]),
                "machine_status": machine_status,
                "initialize_motors": self._ledger_status_projection(state["movement_ledger"]),
            }
        except Exception:
            return {
                "initialize_motion_ledger": {
                    "schema_version": _MOTION_LEDGER_SCHEMA,
                    "terminal_state": "failed_closed",
                    "expected_next_stage": None,
                    "compatibility_blocker": "durable_serial206_state_corrupt",
                    "stage_receipts": [],
                },
                "machine_status": None,
                "initialize_motors": self._corrupt_projection(),
            }

    def capability_status(self) -> dict[str, Any]:
        primitive_status = (
            self.primitives.capability_status()
            if callable(getattr(self.primitives, "capability_status", None))
            else {
                "initialize_motors_exact_primitives_bound": True,
                "initialize_motion_complete": False,
                "initialize_motion_partial_primitives": list(_INITIALIZE_MOTION_PARTIAL),
                "initialize_motion_missing_primitives": list(_INITIALIZE_MOTION_MISSING),
            }
        )
        motors = bool(
            primitive_status.get("initialize_motors_exact_primitives_bound") is True
            and self.preparation_provider is not None
            and callable(getattr(self.preparation_provider, "prepare_for_initialize_motors", None))
            and self.state_store is not None
            and callable(getattr(self.state_store, "write_oem_serial206_initialization_state", None))
        )
        return {
            "initialize_motors_live_available": motors,
            "initialize_motors_binding_blockers": list(
                primitive_status.get("initialize_motors_binding_blockers") or []
            ),
            "initialize_motion_live_available": bool(motors and primitive_status.get("initialize_motion_complete") is True),
            "initialize_motion_partial_primitives": list(primitive_status.get("initialize_motion_partial_primitives") or []),
            "initialize_motion_missing_primitives": list(primitive_status.get("initialize_motion_missing_primitives") or []),
        }

    def gantry_load(self, *, tip_loaded: bool | None = None, plate_on_gantry: Any = None) -> dict[str, Any]:
        """Persist the exact DefaultParameters.GantryLoad-derived pseudo-home state."""
        if tip_loaded is not None and type(tip_loaded) is not bool:
            raise ValueError("tip_loaded must be bool or None")
        if tip_loaded is True:
            pseudo_home = 500
        elif plate_on_gantry is None or plate_on_gantry == "":
            pseudo_home = 65000
        elif str(plate_on_gantry).upper() == "BIO_SECURITY_COVER":
            pseudo_home = 65000
        else:
            pseudo_home = 500
        with self._lock:
            state = self._load_state()
            machine = state["machine_status"]
            machine["tip_loaded"] = tip_loaded
            machine["plate_on_gantry"] = plate_on_gantry
            machine["psudo_z_home_steps"] = pseudo_home
            self._save_state(state)
        return {"ok": True, "psudo_z_home_steps": pseudo_home, "source_anchor": "DefaultParameters.GantryLoad:61-79"}

    def force_to_high_home(self) -> dict[str, Any]:
        """Persist DefaultParameters.ForceToHighHome()."""
        with self._lock:
            state = self._load_state()
            state["machine_status"]["psudo_z_home_steps"] = 500
            self._save_state(state)
        return {"ok": True, "psudo_z_home_steps": 500, "source_anchor": "DefaultParameters.ForceToHighHome:81-84"}

    @staticmethod
    def _commissioning_blockers(
        commissioning: Mapping[str, Serial206CommissioningEvidence],
        *,
        generation: int,
        components: tuple[str, ...],
        establishes_reference: bool,
    ) -> list[str]:
        blockers: list[str] = []
        for component in components:
            if component not in _COMMISSIONED_COMPONENTS:
                continue
            evidence = commissioning.get(component)
            if not isinstance(evidence, Serial206CommissioningEvidence):
                blockers.append(f"commissioning_evidence_required:{component}")
                continue
            if evidence.component != component:
                blockers.append(f"commissioning_component_mismatch:{component}")
            if evidence.generation != generation or evidence.fresh is not True:
                blockers.append(f"fresh_same_epoch_commissioning_required:{component}")
            gates = ["direction_verified", "limits_verified", "switch_verified", "stop_verified"]
            if not establishes_reference:
                gates.append("reference_verified")
            for gate in gates:
                if getattr(evidence, gate) is not True:
                    blockers.append(f"{component}_{gate}_required")
            if component == "z" and (evidence.gap9_polarity, evidence.gap10_polarity) != (1, 0):
                blockers.append("z_gap9_gap10_polarity_must_match_oem_gap9_active_gap10_inactive")
        return blockers

    @staticmethod
    def _approval_blockers(
        approval: Serial206StageApproval | None,
        spec: Serial206StageSpec,
        *,
        generation: int,
        used: Mapping[str, Any],
    ) -> list[str]:
        if not isinstance(approval, Serial206StageApproval):
            return [f"stage_approval_required:{spec.key}"]
        blockers: list[str] = []
        if not approval.approval_id.strip():
            blockers.append("approval_id_required")
        elif approval.approval_id in used:
            blockers.append(f"approval_id_already_used:{approval.approval_id}")
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

    def initialize_motors(
        self,
        *,
        mode: str,
        approval: Serial206StageApproval | None = None,
        commissioning: Mapping[str, Serial206CommissioningEvidence] | None = None,
        timeout_s: float = 180.0,
        approvals: Mapping[str, Serial206StageApproval] | None = None,
    ) -> dict[str, Any]:
        selected_mode = str(mode).strip().lower()
        if selected_mode == "dry_run":
            return {
                "ok": True,
                "ready": False,
                "state": "dry_run",
                "schema": self.schema,
                "source_mode": self.source_mode,
                "mode": "dry_run",
                "opened_usb": False,
                "physical_motion_commanded": False,
                "blockers": ["dry_run_is_not_live_initialization"],
                "stage_receipts": [self._dry_run_receipt(spec) for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS],
            }
        if selected_mode != "live":
            return self._failure("admission", ["mode_must_be_dry_run_or_live"])

        with self._lock:
            try:
                state = self._load_state()
            except Exception:
                return self._failure("failed_closed", ["durable_serial206_state_corrupt"], movement_ledger=self._corrupt_projection())
            ledger = state["movement_ledger"]
            expected = ledger.get("expected_next_stage")
            if expected is None:
                return self._result_from_state(state, ok=True, blockers=[])
            spec = _SPEC_BY_KEY[expected]
            row = ledger["stages"][expected]
            if row["state"] == "acknowledged":
                return self._failure(
                    "awaiting_operator_observation",
                    [f"operator_observation_required:{expected}"],
                    movement_ledger=ledger,
                )
            if row["state"] == "admitted":
                row["state"] = "failed"
                row["result"] = {
                    "ok": False,
                    "failure": "stage_execution_outcome_ambiguous_after_restart",
                    "physical_effect_verified": False,
                }
                ledger["terminal_state"] = "failed_closed"
                try:
                    self._save_state(state)
                except Exception:
                    pass
                return self._failure(
                    "failed_closed",
                    [f"stage_execution_outcome_ambiguous_after_restart:{expected}"],
                    movement_ledger=ledger,
                )
            if row["state"] != "pending":
                return self._failure("failed_closed", [f"durable_stage_state_invalid:{expected}"], movement_ledger=ledger)

            if approval is None and approvals is not None:
                # Legacy multi-stage callers cannot authorize a loop.  At most the
                # exact durable next-stage row is selected.
                approval = approvals.get(expected)
            generation = int(self.generation_provider())
            blockers = []
            if state["used_approvals"] and state["preparation"].get("generation") != generation:
                blockers.append("serial206_generation_changed_after_lifecycle_started")
            status = self.capability_status()
            if status["initialize_motors_live_available"] is not True:
                blockers.append("initialize_motors_exact_primitives_not_bound")
            blockers.extend(self._approval_blockers(approval, spec, generation=generation, used=state["used_approvals"]))
            blockers.extend(
                self._commissioning_blockers(
                    dict(commissioning or {}),
                    generation=generation,
                    components=(spec.component,),
                    establishes_reference=bool(spec.establishes_reference),
                )
            )
            if blockers:
                return self._failure("admission", blockers, generation=generation, movement_ledger=ledger)
            assert approval is not None
            assert self.preparation_provider is not None

            preparation = state["preparation"]
            if not (preparation.get("state") == "completed" and preparation.get("generation") == generation):
                try:
                    raw_preparation = self.preparation_provider.prepare_for_initialize_motors(
                        expected_generation=generation
                    )
                except Exception as exc:
                    raw_preparation = {"ok": False, "error": f"{type(exc).__name__}: {exc}", "physical_motion": False}
                prep_ok = bool(
                    isinstance(raw_preparation, Mapping)
                    and raw_preparation.get("ok") is True
                    and raw_preparation.get("observed_generation") == generation
                    and raw_preparation.get("board_preparation_verified") is True
                    and raw_preparation.get("initialize_without_motion_verified") is True
                    and raw_preparation.get("physical_motion") is False
                )
                state["preparation"] = {
                    "generation": generation,
                    "state": "completed" if prep_ok else "failed_closed",
                    "receipt": _json_safe(raw_preparation),
                }
                if prep_ok:
                    state["z_lifecycle"].update({
                        "state": "prepared_unreferenced",
                        "generation": generation,
                        "prepared_receipt": _json_safe(raw_preparation),
                        "active_receipt": None,
                        "reference_state": "desynced",
                        "last_failure": None,
                    })
                try:
                    self._save_state(state)
                except Exception:
                    return self._failure("failed_closed", ["durable_preparation_persistence_failed"], generation=generation, movement_ledger=ledger)
                if not prep_ok:
                    return self._failure(
                        "failed_closed",
                        ["serial206_board_no_motion_preparation_not_verified"],
                        generation=generation,
                        movement_ledger=ledger,
                    )

            if state["preparation"].get("state") == "completed" and state["preparation"].get("generation") == generation and state["z_lifecycle"].get("state") == "unprepared":
                state["z_lifecycle"].update({
                    "state": "prepared_unreferenced",
                    "generation": generation,
                    "prepared_receipt": _json_safe(state["preparation"].get("receipt")),
                    "reference_state": "desynced",
                })

            # One atomic pre-command transition consumes the generation-bound
            # approval and admits exactly the expected stage.
            state["used_approvals"][approval.approval_id] = {
                "stage": spec.key,
                "generation": generation,
                "idempotency_key": approval.idempotency_key,
            }
            row.update({
                "state": "admitted",
                "command_id": approval.approval_id,
                "idempotency_key": approval.idempotency_key,
                "expected_generation": generation,
                "result": None,
                "artifact_path": None,
                "observation": None,
            })
            ledger["terminal_state"] = "running"
            if spec.component == "z":
                state["z_lifecycle"].update({
                    "state": "executing",
                    "active_receipt": {
                        "command_id": approval.approval_id,
                        "intent": "startup_axis_search_1791",
                        "idempotency_key": approval.idempotency_key,
                        "status": "executing",
                    },
                })
            try:
                self._save_state(state)
            except Exception:
                return self._failure("failed_closed", ["durable_stage_admission_failed"], generation=generation, movement_ledger=ledger)

            try:
                raw = self._execute_stage(spec, timeout_s=float(timeout_s))
            except Exception as exc:
                raw = {
                    "ok": False,
                    "failure": f"primitive_exception:{type(exc).__name__}",
                }
            try:
                receipt = self._receipt(spec, approval, raw)
            except Exception:
                receipt = {
                    "stage": spec.key,
                    "status": "failed",
                    "ok": False,
                    "failure": "primitive_output_projection_failed",
                    "approval_id": approval.approval_id,
                    "idempotency_key": approval.idempotency_key,
                    "expected_generation": approval.expected_generation,
                    "physical_effect_verified": False,
                    "durable_robot_state": {"ready": False},
                    "raw_result": _json_safe(raw),
                }
            row["result"] = _json_safe(receipt)
            self._apply_initialize_motors_host_state(state, spec, receipt)
            if receipt["ok"] is not True:
                row["state"] = "failed"
                ledger["terminal_state"] = "failed_closed"
                self._mark_desynced(spec)
            elif spec.movement:
                row["state"] = "acknowledged"
                ledger["terminal_state"] = "awaiting_operator_observation"
            else:
                row["state"] = "completed"
                advance_initialize_motors_ledger(ledger, spec.key)
            if spec.component == "z":
                z = state["z_lifecycle"]
                z["active_receipt"] = None
                z_receipt = {
                    "command_id": approval.approval_id,
                    "intent": "startup_axis_search_1791",
                    "idempotency_key": approval.idempotency_key,
                    "status": "completed" if receipt["ok"] is True else "failed",
                    "result": _json_safe(receipt),
                    "physical_effect_verified": False,
                }
                durable_z_receipt = self._append_z_receipt(z, z_receipt)
                if receipt["ok"] is True:
                    z.update({
                        "state": "awaiting_operator_observation",
                        "awaiting_observation_receipt_id": approval.approval_id,
                        "reference_state": "desynced",
                        "last_failure": None,
                    })
                else:
                    z.update({
                        "state": "failed_latched",
                        "reference_state": "desynced",
                        "last_failure": _json_safe(z_receipt),
                    })
            try:
                self._save_state(state)
            except Exception:
                return self._failure(
                    "failed_closed",
                    ["durable_stage_result_persistence_failed"],
                    generation=generation,
                    movement_ledger=ledger,
                    stage_receipts=[receipt],
                    physical_motion_commanded=spec.movement,
                )
            if spec.component == "z":
                self._persist_z_receipt(durable_z_receipt)
            return self._result_from_state(
                state,
                ok=receipt["ok"] is True,
                blockers=[] if receipt["ok"] is True else [str(receipt["failure"])],
                generation=generation,
                stage_receipts=[receipt],
                physical_motion_commanded=spec.movement,
            )

    def record_observation(
        self,
        *,
        stage: str,
        command_id: str,
        expected_generation: int,
        observed_pass: bool,
        note: str,
    ) -> dict[str, Any]:
        """Apply human physical evidence without touching any hardware primitive."""
        with self._lock:
            try:
                state = self._load_state()
            except Exception:
                return self._failure("failed_closed", ["durable_serial206_state_corrupt"], movement_ledger=self._corrupt_projection())
            ledger = state["movement_ledger"]
            if type(observed_pass) is not bool:
                return self._failure("observation_rejected", ["observed_pass_must_be_boolean"], movement_ledger=ledger)
            if not isinstance(note, str) or not note.strip() or len(note.strip()) > 2000:
                return self._failure("observation_rejected", ["operator_note_required_and_bounded"], movement_ledger=ledger)
            if type(expected_generation) is not int or expected_generation != int(self.generation_provider()):
                return self._failure("observation_rejected", ["observation_generation_mismatch"], movement_ledger=ledger)
            selected = str(stage).strip()
            expected = ledger.get("expected_next_stage")
            if selected != expected or selected not in _SPEC_BY_KEY:
                return self._failure("observation_rejected", [f"expected_next_stage:{expected}"], movement_ledger=ledger)
            spec = _SPEC_BY_KEY[selected]
            row = ledger["stages"][selected]
            if not spec.movement or row.get("state") != "acknowledged":
                return self._failure("observation_rejected", [f"acknowledged_movement_required:{selected}"], movement_ledger=ledger)
            if not isinstance(command_id, str) or command_id != row.get("command_id"):
                return self._failure("observation_rejected", ["observation_command_id_mismatch"], movement_ledger=ledger)
            row["observation"] = {
                "command_id": command_id,
                "expected_generation": expected_generation,
                "observed_pass": observed_pass,
                "note": note.strip(),
            }
            if observed_pass is False:
                row["state"] = "failed"
                ledger["terminal_state"] = "failed_closed"
                self._mark_desynced(spec)
                ok = False
                blockers = [f"operator_observation_failed:{selected}"]
            else:
                row["state"] = "operator_observed"
                self._mark_referenced(spec, row.get("result"))
                advance_initialize_motors_ledger(ledger, selected)
                ok = True
                blockers = []
            if spec.component == "z":
                z = state["z_lifecycle"]
                z["last_observation"] = {
                    "command_id": command_id,
                    "verdict": "pass" if observed_pass else "fail",
                    "note": note.strip(),
                    "observed_at": time.time(),
                    "physical_effect_verified": bool(observed_pass),
                }
                z["awaiting_observation_receipt_id"] = None
                if observed_pass:
                    z.update({
                        "state": "referenced_ready",
                        "reference_state": "referenced",
                        "last_failure": None,
                    })
                else:
                    z.update({
                        "state": "failed_latched",
                        "reference_state": "desynced",
                        "last_failure": _json_safe(z["last_observation"]),
                    })
            try:
                self._save_state(state)
            except Exception:
                return self._failure("failed_closed", ["durable_observation_persistence_failed"], movement_ledger=ledger)
            result = self._result_from_state(state, ok=ok, blockers=blockers, generation=expected_generation)
            result["physical_motion_commanded"] = False
            result["observation_recorded"] = True
            return result

    def initialize_motion(
        self,
        *,
        mode: str,
        approvals: Mapping[str, Serial206StageApproval] | None = None,
        commissioning: Mapping[str, Serial206CommissioningEvidence] | None = None,
        motion_approvals: Mapping[str, Serial206StageApproval] | None = None,
        timeout_s: float = 180.0,
        **_: Any,
    ) -> dict[str, Any]:
        selected_mode = str(mode).strip().lower()
        if selected_mode == "dry_run":
            return {
                "ok": True,
                "ready": False,
                "state": "dry_run",
                "schema": "bioxp.serial206_initializeMotion.v2",
                "mode": "dry_run",
                "opened_usb": False,
                "physical_motion_commanded": False,
                "blockers": ["dry_run_is_not_live_initialization"],
                "stage_receipts": [self._dry_run_receipt(spec) for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS],
            }
        if selected_mode != "live":
            return self._motion_failure("admission", ["mode_must_be_dry_run_or_live"])

        with self._lock:
            try:
                state = self._load_state()
            except Exception:
                return self._motion_failure("failed_closed", ["durable_serial206_state_corrupt"])
            motion = state["initialize_motion_ledger"]
            if motion.get("terminal_state") == "failed_closed":
                return self._motion_result(state, ok=False, blockers=["initializeMotion_failed_closed"])
            if motion.get("terminal_state") == "initializeMotion_complete":
                return self._motion_result(state, ok=True, blockers=[])
            admitted = next(
                (row for row in motion["stage_receipts"] if isinstance(row, Mapping) and row.get("status") == "admitted"),
                None,
            )
            if isinstance(admitted, Mapping):
                failed = dict(admitted)
                failed.update({
                    "status": "failed",
                    "ok": False,
                    "failure": "initializeMotion_stage_outcome_ambiguous_after_restart",
                    "physical_effect_verified": False,
                })
                motion["stage_receipts"][motion["stage_receipts"].index(admitted)] = failed
                self._record_initialize_motion_exception(state, "initializeMotion stage outcome ambiguous after restart")
                try:
                    self._save_state(state)
                except Exception:
                    pass
                return self._motion_result(state, ok=False, blockers=["initializeMotion_stage_outcome_ambiguous_after_restart"])

            expected = motion.get("expected_next_stage")
            if expected == "initializeMotion.initializeMotors":
                motor_result = self.initialize_motors(
                    mode="live",
                    approvals=approvals,
                    commissioning=commissioning,
                    timeout_s=float(timeout_s),
                )
                try:
                    state = self._load_state()
                except Exception:
                    return self._motion_failure("failed_closed", ["durable_serial206_state_corrupt"])
                motion = state["initialize_motion_ledger"]
                if state["movement_ledger"].get("terminal_state") != "initializeMotors_complete":
                    result = self._motion_result(
                        state,
                        ok=bool(motor_result.get("ok")),
                        blockers=list(motor_result.get("blockers") or []),
                    )
                    result["initialize_motors_result"] = _json_safe(motor_result)
                    return result
                receipt = {
                    "stage": expected,
                    "status": "completed",
                    "ok": True,
                    "failure": None,
                    "source_call": "m_ControlInterface.initializeMotors()",
                    "source_anchor": "ControlLib.initializeMotion:8803",
                    "movement_terminal_state": "initializeMotors_complete",
                    "physical_effect_verified": False,
                }
                motion["stage_receipts"].append(receipt)
                motion["expected_next_stage"] = "initializeMotion.thermal_door_closed"
                motion["terminal_state"] = "awaiting_next_stage"
                try:
                    self._save_state(state)
                except Exception:
                    return self._motion_failure("failed_closed", ["durable_initializeMotion_result_persistence_failed"])
                return self._motion_result(state, ok=True, blockers=[], stage_receipts=[receipt])

            if expected not in _MOTION_SPEC_BY_KEY:
                return self._motion_failure("failed_closed", [f"invalid_initializeMotion_expected_stage:{expected}"])
            status = self.capability_status()
            if status.get("initialize_motion_live_available") is not True:
                return self._motion_result(state, ok=False, blockers=["initialize_motion_exact_primitives_not_bound"])
            spec = _MOTION_SPEC_BY_KEY[expected]
            selected_approvals = motion_approvals or {}
            approval = selected_approvals.get(expected)
            generation = int(self.generation_provider())
            blockers = self._approval_blockers(
                approval,
                spec,
                generation=generation,
                used=state["used_motion_approvals"],
            )
            if blockers:
                return self._motion_result(state, ok=False, blockers=blockers)
            assert approval is not None

            state["used_motion_approvals"][approval.approval_id] = {
                "stage": spec.key,
                "generation": generation,
                "idempotency_key": approval.idempotency_key,
            }
            admitted_receipt = {
                "stage": spec.key,
                "status": "admitted",
                "ok": None,
                "approval_id": approval.approval_id,
                "idempotency_key": approval.idempotency_key,
                "expected_generation": generation,
                "component": spec.component,
                "direction": spec.direction,
                "bound": spec.bound,
                "physical_effect_verified": False,
            }
            motion["stage_receipts"].append(admitted_receipt)
            motion["terminal_state"] = "running"
            try:
                self._save_state(state)
            except Exception:
                return self._motion_failure("failed_closed", ["durable_initializeMotion_stage_admission_failed"])

            try:
                raw = self._execute_initialize_motion_stage(state, spec, timeout_s=float(timeout_s))
            except Exception as exc:
                raw = {"ok": False, "failure": f"{type(exc).__name__}: {exc}"}
            receipt = self._initialize_motion_receipt(spec, approval, raw)
            motion["stage_receipts"][-1] = receipt
            if receipt["ok"] is not True:
                self._record_initialize_motion_exception(state, str(receipt.get("failure") or "initializeMotion stage failed"))
            else:
                self._apply_initialize_motion_transition(state, spec, raw)
            try:
                self._save_state(state)
            except Exception:
                return self._motion_failure(
                    "failed_closed",
                    ["durable_initializeMotion_stage_result_persistence_failed"],
                    stage_receipts=[receipt],
                )
            return self._motion_result(
                state,
                ok=receipt["ok"] is True and motion.get("terminal_state") != "failed_closed",
                blockers=[] if receipt["ok"] is True and motion.get("terminal_state") != "failed_closed" else [str(receipt.get("failure") or motion["context"].get("caught_exception") or "initializeMotion_failed_closed")],
                stage_receipts=[receipt],
                physical_motion_commanded=spec.movement,
            )

    def _motion_result(
        self,
        state: Mapping[str, Any],
        *,
        ok: bool,
        blockers: list[str],
        stage_receipts: list[dict[str, Any]] | None = None,
        physical_motion_commanded: bool = False,
    ) -> dict[str, Any]:
        motion = state.get("initialize_motion_ledger") if isinstance(state, Mapping) else None
        machine = state.get("machine_status") if isinstance(state, Mapping) else None
        terminal = motion.get("terminal_state") if isinstance(motion, Mapping) else "failed_closed"
        return {
            "ok": bool(ok),
            "ready": bool(terminal == "initializeMotion_complete"),
            "state": terminal,
            "schema": "bioxp.serial206_initializeMotion.v2",
            "mode": "live",
            "opened_usb": bool(physical_motion_commanded),
            "physical_motion_commanded": bool(physical_motion_commanded),
            "physical_effect_verified": False,
            "blockers": list(blockers),
            "initialize_motion_ledger": _json_safe(motion),
            "movement_ledger": _json_safe(state.get("movement_ledger") if isinstance(state, Mapping) else None),
            "machine_status": _json_safe(machine),
            "stage_receipts": _json_safe(stage_receipts or []),
        }

    def _motion_failure(
        self,
        state_name: str,
        blockers: list[str],
        *,
        stage_receipts: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        return {
            "ok": False,
            "ready": False,
            "state": str(state_name),
            "schema": "bioxp.serial206_initializeMotion.v2",
            "mode": "live",
            "opened_usb": False,
            "physical_motion_commanded": False,
            "physical_effect_verified": False,
            "blockers": list(blockers),
            "stage_receipts": _json_safe(stage_receipts or []),
        }

    @staticmethod
    def _initialize_motion_receipt(
        spec: Serial206StageSpec,
        approval: Serial206StageApproval,
        raw: Any,
    ) -> dict[str, Any]:
        result = raw if isinstance(raw, Mapping) else {}
        ok = result.get("ok") is True
        return {
            "stage": spec.key,
            "status": "completed" if ok else "failed",
            "ok": bool(ok),
            "failure": None if ok else str(result.get("failure") or "initializeMotion_primitive_evidence_not_accepted"),
            "approval_id": approval.approval_id,
            "idempotency_key": approval.idempotency_key,
            "expected_generation": approval.expected_generation,
            "component": spec.component,
            "direction": spec.direction,
            "bound": spec.bound,
            "source_anchor": result.get("source_anchor"),
            "physical_effect_verified": False,
            "raw_result": _json_safe(result),
        }

    def _execute_initialize_motion_stage(
        self,
        state: Mapping[str, Any],
        spec: Serial206StageSpec,
        *,
        timeout_s: float,
    ) -> dict[str, Any]:
        p = self.primitives
        stage = spec.key
        bounded = max(2.0, float(timeout_s))
        if stage == "initializeMotion.stop_scripts":
            return {"ok": True, "value": True, "source_anchor": "ControlLib.initializeMotion:8799"}
        if stage == "initializeMotion.clear_forceabort":
            return {"ok": True, "value": False, "source_anchor": "ControlLib.initializeMotion:8800"}
        if stage == "initializeMotion.thermal_door_closed":
            return {"ok": True, "value": False, "source_anchor": "ControlLib.initializeMotion:8804"}
        if stage == "initializeMotion.queryTipStatus.initial":
            result = p.query_all_pipette_tip_states()
            return {
                **dict(result),
                "ok": bool(isinstance(result, Mapping) and result.get("ok") is True and type(result.get("tip_exists")) is bool),
                "source_anchor": "ControlLib.initializeMotion:8805; ClassPipetteCollection.queryTipStatus",
            }
        if stage == "initializeMotion.sleep.after_tip_query":
            self.sleep(0.500)
            return {"ok": True, "slept_ms": 500, "source_anchor": "ControlLib.initializeMotion:8806"}
        if stage == "initializeMotion.openThermalDoor.tip_exists":
            result = p.motor_oem_open_thermal_door(timeout_s=min(bounded, 20.0))
            return {**dict(result), "source_anchor": "ControlLib.initializeMotion:8809"}
        if stage == "initializeMotion.thermal_door_open.tip_exists":
            return {"ok": True, "value": True, "source_anchor": "ControlLib.initializeMotion:8810"}
        if stage == "initializeMotion.tip_loaded.tip_exists":
            return {"ok": True, "value": True, "source_anchor": "ControlLib.initializeMotion:8811"}
        if stage == "initializeMotion.scriptmoveTo.tip_exists":
            machine = state.get("machine_status") if isinstance(state, Mapping) else {}
            return p.oem_initialize_motion_scriptmove_to_waste(
                current_location=28,
                current_well=0,
                target_location=6,
                target_well=0,
                position_flag=0,
                tip_loaded=True,
                tip_dirty=bool(machine.get("tip_dirty")) if isinstance(machine, Mapping) else False,
                timeout_s=min(bounded, 60.0),
                pseudo_home_steps=int(machine.get("psudo_z_home_steps", 65000)) if isinstance(machine, Mapping) else 65000,
                plate_on_gantry=machine.get("plate_on_gantry") if isinstance(machine, Mapping) else None,
                location19_y=machine.get("location19_y") if isinstance(machine, Mapping) else None,
            )
        if stage == "initializeMotion.updateLocation.tip_exists":
            return {
                "ok": True,
                "location": 6,
                "well": 0,
                "source_anchor": "ControlLib.initializeMotion:8813; MachineStatus.updateLocation",
            }
        if stage == "initializeMotion.ejectAllTips.tip_exists":
            context = state["initialize_motion_ledger"]["context"]
            expected_channels = list(context.get("tip_channels_initial") or [])
            result = p.eject_all_pipette_tips_for_oem_startup(
                operator_ack="EJECT_STALE_STARTUP_TIPS",
                expected_channels_with_tips=expected_channels,
            )
            return {
                "ok": bool(isinstance(result, Mapping) and result.get("ok") is True),
                "command_completed": bool(isinstance(result, Mapping) and result.get("ok") is True),
                "eject_reported_ok": result.get("ok") if isinstance(result, Mapping) else False,
                "eject_result": _json_safe(result),
                "source_anchor": "ControlLib.initializeMotion:8814; ClassPipetteCollection.ejectAllTips(false,true)",
            }
        if stage == "initializeMotion.moveZ.tip_exists":
            machine = state.get("machine_status") if isinstance(state, Mapping) else {}
            return p.oem_initialize_motion_move_absolute(
                "z",
                80000,
                timeout_s=min(bounded, 45.0),
                pseudo_home_steps=int(machine.get("psudo_z_home_steps", 65000)) if isinstance(machine, Mapping) else 65000,
            )
        if stage == "initializeMotion.moveX.tip_exists":
            return p.oem_initialize_motion_move_absolute("x", 79000, timeout_s=min(bounded, 45.0))
        if stage == "initializeMotion.queryTipStatus.after_eject":
            result = p.query_all_pipette_tip_states()
            return {
                **dict(result),
                "ok": bool(isinstance(result, Mapping) and result.get("ok") is True and type(result.get("tip_exists")) is bool),
                "source_anchor": "ControlLib.initializeMotion:8817; ClassPipetteCollection.queryTipStatus",
            }
        if stage == "initializeMotion.sleep.after_eject_query":
            self.sleep(0.100)
            return {"ok": True, "slept_ms": 100, "source_anchor": "ControlLib.initializeMotion:8818"}
        if stage == "initializeMotion.tip_dirty_false":
            return {"ok": True, "value": False, "source_anchor": "ControlLib.initializeMotion:8829"}
        if stage == "initializeMotion.tip_loaded_false.after_eject":
            return {"ok": True, "value": False, "source_anchor": "ControlLib.initializeMotion:8830"}
        if stage == "initializeMotion.sleep.before_initiate_group":
            self.sleep(0.002)
            return {"ok": True, "slept_ms": 2, "source_anchor": "ControlLib.initializeMotion:8831"}
        if stage == "initializeMotion.initiateGroup.initial":
            result = p.initiate_pipette_group_for_oem_initialize_motion(cycle="initializeMotion.initial")
            return {
                "ok": bool(isinstance(result, Mapping) and result.get("ok") is True),
                "group_reported_ok": result.get("ok") if isinstance(result, Mapping) else False,
                "group_result": _json_safe(result),
                "source_anchor": "ControlLib.initializeMotion:8832",
            }
        if stage == "initializeMotion.checkedPipetteStatus.initial":
            result = p.checked_pipette_status_for_oem_initialize_motion(attempt="initial")
            host_forceabort = state["machine_status"].get("forceabort")
            return {
                "ok": isinstance(result, Mapping),
                "checked_status": bool(result.get("ok") is True and host_forceabort is False) if isinstance(result, Mapping) else False,
                "forceabort": host_forceabort,
                "status_result": _json_safe(result),
                "source_anchor": "ControlLib.initializeMotion:8833",
            }
        if stage == "initializeMotion.initiateGroup.retry":
            result = p.initiate_pipette_group_for_oem_initialize_motion(cycle="initializeMotion.retry")
            return {
                "ok": bool(isinstance(result, Mapping) and result.get("ok") is True),
                "group_reported_ok": result.get("ok") if isinstance(result, Mapping) else False,
                "group_result": _json_safe(result),
                "source_anchor": "ControlLib.initializeMotion:8835",
            }
        if stage == "initializeMotion.checkedPipetteStatus.retry":
            result = p.checked_pipette_status_for_oem_initialize_motion(attempt="retry")
            host_forceabort = state["machine_status"].get("forceabort")
            return {
                "ok": isinstance(result, Mapping),
                "checked_status": bool(result.get("ok") is True and host_forceabort is False) if isinstance(result, Mapping) else False,
                "forceabort": host_forceabort,
                "status_result": _json_safe(result),
                "source_anchor": "ControlLib.initializeMotion:8836",
            }
        if stage == "initializeMotion.tip_loaded_false.no_tip":
            return {"ok": True, "value": False, "source_anchor": "ControlLib.initializeMotion:8843-8846"}
        raise RuntimeError(f"unmapped serial-206 initializeMotion stage: {stage}")

    @staticmethod
    def _append_initialize_motion_error_event(state: dict[str, Any], message: str, *, source: str) -> None:
        state["machine_status"]["error_events"].append({
            "message": str(message),
            "source": str(source),
            "error_event_bound": True,
        })

    def _record_initialize_motion_exception(
        self,
        state: dict[str, Any],
        message: str,
        *,
        inner_error_event: bool = False,
        pause_scripts: bool = False,
    ) -> None:
        if pause_scripts:
            state["machine_status"]["pause_scripts"] = True
        if inner_error_event:
            self._append_initialize_motion_error_event(state, message, source="initializeMotion.inner_error_branch")
        self._append_initialize_motion_error_event(state, message, source="initializeMotion.catch")
        motion = state["initialize_motion_ledger"]
        motion["context"]["caught_exception"] = str(message)
        motion["terminal_state"] = "failed_closed"
        motion["expected_next_stage"] = None

    def _apply_initialize_motion_transition(
        self,
        state: dict[str, Any],
        spec: Serial206StageSpec,
        raw: Mapping[str, Any],
    ) -> None:
        stage = spec.key
        machine = state["machine_status"]
        motion = state["initialize_motion_ledger"]
        context = motion["context"]
        next_stage: str | None
        if stage == "initializeMotion.stop_scripts":
            machine["stop_scripts"] = True
            next_stage = "initializeMotion.clear_forceabort"
        elif stage == "initializeMotion.clear_forceabort":
            machine["forceabort"] = False
            next_stage = "initializeMotion.initializeMotors"
        elif stage == "initializeMotion.thermal_door_closed":
            machine["thermal_door_open"] = False
            next_stage = "initializeMotion.queryTipStatus.initial"
        elif stage == "initializeMotion.queryTipStatus.initial":
            context["tip_exists_initial"] = bool(raw["tip_exists"])
            context["tip_channels_initial"] = list(raw.get("channels_with_tips") or [])
            next_stage = "initializeMotion.sleep.after_tip_query"
        elif stage == "initializeMotion.sleep.after_tip_query":
            next_stage = (
                "initializeMotion.openThermalDoor.tip_exists"
                if context.get("tip_exists_initial") is True
                else "initializeMotion.tip_loaded_false.no_tip"
            )
        elif stage == "initializeMotion.openThermalDoor.tip_exists":
            next_stage = "initializeMotion.thermal_door_open.tip_exists"
        elif stage == "initializeMotion.thermal_door_open.tip_exists":
            machine["thermal_door_open"] = True
            next_stage = "initializeMotion.tip_loaded.tip_exists"
        elif stage == "initializeMotion.tip_loaded.tip_exists":
            machine["tip_loaded"] = True
            machine["psudo_z_home_steps"] = 500
            next_stage = "initializeMotion.scriptmoveTo.tip_exists"
        elif stage == "initializeMotion.scriptmoveTo.tip_exists":
            next_stage = "initializeMotion.updateLocation.tip_exists"
        elif stage == "initializeMotion.updateLocation.tip_exists":
            machine["current_location"] = 6
            machine["current_well"] = 0
            next_stage = "initializeMotion.ejectAllTips.tip_exists"
        elif stage == "initializeMotion.ejectAllTips.tip_exists":
            next_stage = "initializeMotion.moveZ.tip_exists"
        elif stage == "initializeMotion.moveZ.tip_exists":
            next_stage = "initializeMotion.moveX.tip_exists"
        elif stage == "initializeMotion.moveX.tip_exists":
            next_stage = "initializeMotion.queryTipStatus.after_eject"
        elif stage == "initializeMotion.queryTipStatus.after_eject":
            context["tip_exists_after_eject"] = bool(raw["tip_exists"])
            next_stage = "initializeMotion.sleep.after_eject_query"
        elif stage == "initializeMotion.sleep.after_eject_query":
            if context.get("tip_exists_after_eject") is True:
                self._record_initialize_motion_exception(
                    state,
                    "Eject tip failed",
                    inner_error_event=True,
                    pause_scripts=True,
                )
                return
            next_stage = "initializeMotion.tip_dirty_false"
        elif stage == "initializeMotion.tip_dirty_false":
            machine["tip_dirty"] = False
            next_stage = "initializeMotion.tip_loaded_false.after_eject"
        elif stage == "initializeMotion.tip_loaded_false.after_eject":
            machine["tip_loaded"] = False
            machine["psudo_z_home_steps"] = 65000
            next_stage = "initializeMotion.sleep.before_initiate_group"
        elif stage == "initializeMotion.sleep.before_initiate_group":
            next_stage = "initializeMotion.initiateGroup.initial"
        elif stage == "initializeMotion.initiateGroup.initial":
            next_stage = "initializeMotion.checkedPipetteStatus.initial"
        elif stage == "initializeMotion.checkedPipetteStatus.initial":
            context["checked_status_initial"] = raw.get("checked_status") is True
            if context["checked_status_initial"]:
                motion["terminal_state"] = "initializeMotion_complete"
                motion["expected_next_stage"] = None
                return
            next_stage = "initializeMotion.initiateGroup.retry"
        elif stage == "initializeMotion.initiateGroup.retry":
            next_stage = "initializeMotion.checkedPipetteStatus.retry"
        elif stage == "initializeMotion.checkedPipetteStatus.retry":
            context["checked_status_retry"] = raw.get("checked_status") is True
            if context["checked_status_retry"]:
                motion["terminal_state"] = "initializeMotion_complete"
                motion["expected_next_stage"] = None
                return
            self._record_initialize_motion_exception(
                state,
                "Eject tip failed",
                inner_error_event=True,
                pause_scripts=False,
            )
            return
        elif stage == "initializeMotion.tip_loaded_false.no_tip":
            machine["tip_loaded"] = False
            machine["psudo_z_home_steps"] = 65000
            motion["terminal_state"] = "initializeMotion_complete"
            motion["expected_next_stage"] = None
            return
        else:
            raise RuntimeError(f"unmapped initializeMotion transition: {stage}")
        motion["expected_next_stage"] = next_stage
        motion["terminal_state"] = "awaiting_next_stage"

    @staticmethod
    def _apply_initialize_motors_host_state(
        state: dict[str, Any],
        spec: Serial206StageSpec,
        receipt: Mapping[str, Any],
    ) -> None:
        if receipt.get("ok") is not True:
            return
        machine = state["machine_status"]
        if spec.key == "ui-zero-calibrated":
            machine["calibrated_ui_positions"] = {"x": 0, "y": 0, "z": 0}
        elif spec.key == "system-status-initialized":
            machine["system_status"] = 1
            machine["initialization_complete"] = True

    def _execute_stage(self, spec: Serial206StageSpec, *, timeout_s: float) -> dict[str, Any]:
        p = self.primitives
        stage = spec.key
        bounded = max(2.0, float(timeout_s))
        if stage == "z-home":
            return p.motor_oem_home_axis("z", startup=True, speed=1791, timeout_s=bounded)
        if stage == "gripper-current-31":
            return p.motor_set_axis_param(4, 6, 31, motor=2)
        if stage == "gripper-clear-10000":
            return self._merge_move_wait(
                p.motor_move_relative(4, 10000, motor=2),
                p.motor_wait_stopped(4, motor=2, timeout_s=min(bounded, 20.0), require_seen_nonzero=True),
            )
        if stage == "gripper-home":
            return p.motor_oem_home_axis(
                "g",
                startup=True,
                speed=200,
                timeout_s=min(bounded, 30.0),
                restore_idle_current=False,
            )
        if stage == "x-home":
            return p.motor_oem_home_axis("x", startup=True, speed=250, timeout_s=min(bounded, 45.0))
        if stage == "x-home-settle":
            self.sleep(0.020)
            return {"settled": True, "settle_ms": 20}
        if stage == "x-set-home":
            return p.motor_set_home(5, motor=0)
        if stage == "x-speed-1700":
            return p.motor_set_axis_param(5, 4, 1700, motor=0)
        if stage == "x-speed-settle":
            self.sleep(0.040)
            return {"settled": True, "settle_ms": 40}
        if stage == "x-park-6000":
            return self._merge_move_wait(
                p.motor_move_absolute(5, 6000, motor=0),
                p.motor_wait_stopped(5, motor=0, timeout_s=min(bounded, 45.0), require_seen_nonzero=True),
            )
        if stage == "y-home":
            return p.motor_oem_home_axis("y", startup=True, speed=250, timeout_s=min(bounded, 45.0))
        if stage == "door-home":
            result = p.motor_oem_door_search_home(startup=True, timeout_s=min(bounded, 45.0))
            status_after = result.get("status_after") if isinstance(result, Mapping) else None
            predicates = status_after.get("oem_predicates") if isinstance(status_after, Mapping) else None
            closed = predicates.get("tcDoorClosed") if isinstance(predicates, Mapping) else None
            binding = p.oem_initialize_motors_branch_binding()
            serial_number = binding.get("serial_number") if isinstance(binding, Mapping) else None
            camera_calibrated = binding.get("camera_calibrated") if isinstance(binding, Mapping) else None
            if type(serial_number) is int and serial_number > 9 and closed is False and camera_calibrated is True:
                opened = p.motor_oem_open_thermal_door(timeout_s=min(bounded, 20.0))
                return {
                    **(dict(result) if isinstance(result, Mapping) else {}),
                    "ok": False,
                    "failure": "Cannot close thermal cycler door!",
                    "exception_type": "Exception",
                    "source_condition": "SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated",
                    "source_condition_active": True,
                    "branch_binding": _json_safe(binding),
                    "openThermalDoor": _json_safe(opened),
                    "open_attempted_before_throw": True,
                }
            return dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "door_search_home_result_invalid"}
        if stage == "door-closed-predicate":
            status = p.motor_thermal_door_status()
            predicates = status.get("oem_predicates") if isinstance(status, Mapping) else None
            closed = predicates.get("tcDoorClosed") if isinstance(predicates, Mapping) else None
            source = predicates.get("closed_source") if isinstance(predicates, Mapping) else None
            binding = p.oem_initialize_motors_branch_binding()
            serial_number = binding.get("serial_number") if isinstance(binding, Mapping) else None
            camera_calibrated = binding.get("camera_calibrated") if isinstance(binding, Mapping) else None
            if type(serial_number) is not int or type(camera_calibrated) is not bool:
                return {
                    **(dict(status) if isinstance(status, Mapping) else {}),
                    "ok": False,
                    "failure": "initializeMotors_serial_camera_branch_authority_not_bound",
                    "branch_binding": _json_safe(binding),
                }
            if type(closed) is not bool or source != "queryHome(ThermalDoor)":
                return {
                    **(dict(status) if isinstance(status, Mapping) else {}),
                    "ok": False,
                    "failure": "tcDoorClosed_source_predicate_required",
                    "branch_binding": _json_safe(binding),
                }
            source_condition_active = serial_number > 9 and closed is False and camera_calibrated is True
            opened = None
            if source_condition_active:
                opened = p.motor_oem_open_thermal_door(timeout_s=min(bounded, 20.0))
                return {
                    **dict(status),
                    "ok": False,
                    "failure": "Cannot close thermal cycler door!",
                    "exception_type": "Exception",
                    "source_condition": "SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated",
                    "source_condition_active": True,
                    "branch_binding": _json_safe(binding),
                    "openThermalDoor": _json_safe(opened),
                    "open_attempted_before_throw": True,
                }
            return {
                **dict(status),
                "ok": True,
                "failure": None,
                "source_condition": "SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated",
                "source_condition_active": False,
                "branch_binding": _json_safe(binding),
                "openThermalDoor": None,
                "open_attempted_before_throw": False,
            }
        if stage == "y-set-home":
            return p.motor_set_home(4, motor=0)
        if stage == "ui-zero-calibrated":
            return p.oem_set_calibrated_ui_positions_zero()
        if stage == "chiller-oc-cool-rate":
            return p._oem_no_motion_tmcl_with_readback(name=stage, board=7, command=9, cmd_type=1, motor=0, value=-25)
        if stage == "chiller-rc-cool-rate":
            return p._oem_no_motion_tmcl_with_readback(name=stage, board=7, command=9, cmd_type=0, motor=0, value=-25)
        if stage == "system-status-initialized":
            return {"ok": True, "controller_state": {"system_status": 1, "initialization_complete": True, "ready": False}}
        if stage == "gripper-idle-current-10":
            return p.motor_set_axis_param(4, 6, 10, motor=2)
        raise RuntimeError(f"unmapped serial-206 OEM stage: {stage}")

    @staticmethod
    def _merge_move_wait(move: Any, wait: Any) -> dict[str, Any]:
        result: dict[str, Any] = {}
        if isinstance(move, Mapping) and isinstance(move.get("move"), Mapping):
            result["move"] = move["move"]
            position = move.get("position")
            if isinstance(position, Mapping) and "before" in position:
                result["position"] = {"before": position["before"]}
        else:
            result["move"] = move
        if isinstance(wait, Mapping) and isinstance(wait.get("wait"), Mapping):
            result["wait"] = wait["wait"]
            position = wait.get("position")
            if isinstance(position, Mapping) and "after" in position:
                result.setdefault("position", {})["after"] = position["after"]
        else:
            result["wait"] = wait
        return result

    @staticmethod
    def _receipt(spec: Serial206StageSpec, approval: Serial206StageApproval, raw: Any) -> dict[str, Any]:
        result = raw if isinstance(raw, Mapping) else {}

        def ack_ok(value: Any) -> bool:
            return isinstance(value, Mapping) and type(value.get("status")) is int and value.get("status") == 100

        def position(value: Any) -> int | None:
            if not isinstance(value, Mapping) or value.get("ok") is not True or not ack_ok(value.get("ack")):
                return None
            observed = value.get("position")
            return observed if type(observed) is int else None

        def write_ok(value: Any, expected: int) -> bool:
            if not isinstance(value, Mapping) or value.get("ok") is not True or not ack_ok(value.get("ack")):
                return False
            if value.get("set_value") != expected:
                return False
            readback = value.get("readback")
            return isinstance(readback, Mapping) and ack_ok(readback.get("ack")) and readback.get("value") == expected

        def wait_ok(value: Any) -> bool:
            return (
                isinstance(value, Mapping)
                and value.get("stopped") is True
                and type(value.get("last_speed")) is int
                and value.get("last_speed") == 0
                and ack_ok(value.get("last_ack"))
            )

        def prepare_ok(value: Any) -> bool:
            if not isinstance(value, Mapping) or type(value.get("board")) is not int or type(value.get("motor")) is not int:
                return False
            ops = value.get("ops")
            if not isinstance(ops, list) or not ops:
                return False
            return all(
                isinstance(row, Mapping)
                and type(row.get("set")) is int
                and ack_ok(row.get("ack"))
                and isinstance(row.get("rb"), Mapping)
                and ack_ok(row["rb"].get("ack"))
                and row["rb"].get("value") == row.get("set")
                for row in ops
            )

        def home_evidence(value: Any, *, gripper: bool = False) -> tuple[bool, int | None, int | None, Any, Any]:
            if not isinstance(value, Mapping) or value.get("startup") is not True or not prepare_ok(value.get("prepare")):
                return False, None, None, None, None
            home = value.get("home")
            if not isinstance(home, Mapping) or home.get("ok") is not True:
                return False, None, None, None, None
            before = position(home.get("position_before"))
            after = position(home.get("position_after"))
            final = position(home.get("position_after_sethome"))
            home_after = home.get("home_after")
            switch_ok = (
                isinstance(home_after, Mapping)
                and ack_ok(home_after.get("ack"))
                and home_after.get("value") == home.get("home_active_value")
                and home.get("switch_transition") is True
                and home.get("home_predicate_confirmed") is True
            )
            motion_ok = (
                isinstance(home.get("move_home"), Mapping)
                and home["move_home"].get("ok") is True
                and ack_ok(home["move_home"].get("ack"))
                and wait_ok(home.get("wait"))
                and isinstance(home.get("stop"), Mapping)
                and home["stop"].get("ok") is True
                and ack_ok(home["stop"].get("ack"))
                and home.get("seen_motion") is True
                and before is not None
                and after is not None
                and before != after
                and final == 0
                and write_ok(home.get("set_home"), 0)
            )
            restore_ok = True
            if gripper:
                restore = value.get("restore_current")
                retained = value.get("retained_current_readback")
                restore_ok = bool(
                    value.get("restore_idle_current_requested") is False
                    and value.get("source_current_retained") is True
                    and restore is None
                    and isinstance(retained, Mapping)
                    and retained.get("ok") is True
                    and ack_ok(retained.get("ack"))
                    and retained.get("value") == 31
                )
            return bool(motion_ok and switch_ok and restore_ok), before, final, home_after, home.get("move_home", {}).get("ack")

        before: int | None = None
        after: int | None = None
        switch: Any = None
        ack: Any = None
        controller_reference_agrees = False
        ok = False
        if spec.key in {"z-home", "x-home", "y-home", "gripper-home"}:
            ok, before, after, switch, ack = home_evidence(result, gripper=spec.key == "gripper-home")
            controller_reference_agrees = ok
        elif spec.key == "door-home":
            status_before = result.get("status_before") if isinstance(result, Mapping) else None
            status_after = result.get("status_after") if isinstance(result, Mapping) else None
            before = position(status_before.get("position")) if isinstance(status_before, Mapping) else None
            after = position(status_after.get("position")) if isinstance(status_after, Mapping) else None
            predicate = status_after.get("oem_predicates") if isinstance(status_after, Mapping) else None
            switch = result.get("home_after")
            ack = result.get("move_left", {}).get("ack") if isinstance(result.get("move_left"), Mapping) else None
            ok = bool(
                result.get("ok") is True
                and before is not None
                and after is not None
                and before != after
                and isinstance(result.get("move_left"), Mapping)
                and result["move_left"].get("ok") is True
                and ack_ok(ack)
                and wait_ok(result.get("wait"))
                and isinstance(result.get("stop"), Mapping)
                and result["stop"].get("ok") is True
                and ack_ok(result["stop"].get("ack"))
                and isinstance(predicate, Mapping)
                and predicate.get("tcDoorClosed") is True
                and predicate.get("closed_source") == "queryHome(ThermalDoor)"
                and isinstance(switch, Mapping)
                and ack_ok(switch.get("ack"))
                and switch.get("value") == 1
                and result.get("closed_confirmed") is True
                and write_ok(result.get("set_home"), 0)
            )
            controller_reference_agrees = ok
        elif spec.key in {"gripper-clear-10000", "x-park-6000"}:
            move = result.get("move")
            positions = result.get("position")
            before = position(positions.get("before")) if isinstance(positions, Mapping) else None
            after = position(positions.get("after")) if isinstance(positions, Mapping) else None
            ack = move.get("ack") if isinstance(move, Mapping) else None
            ok = bool(
                isinstance(move, Mapping)
                and move.get("ok") is True
                and ack_ok(ack)
                and wait_ok(result.get("wait"))
                and before is not None
                and after is not None
                and after != before
                and abs(after - before) <= spec.bound
            )
        elif spec.key in {"gripper-current-31", "x-speed-1700", "gripper-idle-current-10"}:
            expected = {"gripper-current-31": 31, "x-speed-1700": 1700, "gripper-idle-current-10": 10}[spec.key]
            ok = write_ok(result, expected)
            ack = result.get("ack")
        elif spec.key in {"x-set-home", "y-set-home"}:
            ok = write_ok(result, 0)
            ack = result.get("ack")
            controller_reference_agrees = ok
        elif spec.key in {"x-home-settle", "x-speed-settle"}:
            expected = 20 if spec.key == "x-home-settle" else 40
            ok = result.get("settled") is True and result.get("settle_ms") == expected
        elif spec.key == "door-closed-predicate":
            predicate = result.get("oem_predicates")
            pos = result.get("position")
            speed = result.get("speed")
            binding = result.get("branch_binding")
            after = position(pos)
            serial_number = binding.get("serial_number") if isinstance(binding, Mapping) else None
            camera_calibrated = binding.get("camera_calibrated") if isinstance(binding, Mapping) else None
            closed = predicate.get("tcDoorClosed") if isinstance(predicate, Mapping) else None
            expected_condition = bool(
                type(serial_number) is int
                and type(camera_calibrated) is bool
                and serial_number > 9
                and closed is False
                and camera_calibrated is True
            )
            ok = bool(
                result.get("ok") is True
                and result.get("source_condition_active") is False
                and expected_condition is False
                and isinstance(predicate, Mapping)
                and type(closed) is bool
                and predicate.get("closed_source") == "queryHome(ThermalDoor)"
                and after is not None
                and isinstance(speed, Mapping)
                and type(speed.get("speed")) is int
                and speed.get("speed") == 0
                and ack_ok(speed.get("ack"))
            )
            switch = predicate
        elif spec.key == "ui-zero-calibrated":
            writes = result.get("writes")
            readback = result.get("readback")
            ok = bool(
                result.get("ok") is True
                and result.get("calibrated") is True
                and isinstance(writes, list)
                and len(writes) == 4
                and [row.get("axis") for row in writes if isinstance(row, Mapping)] == ["x", "y", "z", "z"]
                and all(isinstance(row, Mapping) and row.get("value") in {0, "0"} and row.get("applied") is True for row in writes)
                and isinstance(readback, Mapping)
                and all(readback.get(axis) in {0, "0"} for axis in ("x", "y", "z"))
            )
        elif spec.key in {"chiller-oc-cool-rate", "chiller-rc-cool-rate"}:
            write = result.get("write")
            readback = result.get("readback")
            ok = bool(
                result.get("ok") is True
                and isinstance(write, Mapping)
                and ack_ok(write.get("ack"))
                and write.get("value") == -25
                and isinstance(readback, Mapping)
                and ack_ok(readback.get("ack"))
                and readback.get("value") == -25
            )
            ack = write.get("ack") if isinstance(write, Mapping) else None
        elif spec.key == "system-status-initialized":
            controller = result.get("controller_state")
            ok = bool(
                result.get("ok") is True
                and isinstance(controller, Mapping)
                and controller.get("system_status") == 1
                and controller.get("initialization_complete") is True
                and controller.get("ready") is False
            )

        delta = after - before if type(before) is int and type(after) is int else None
        durable_robot_state: dict[str, Any] = {"ready": False}
        if spec.key == "system-status-initialized" and ok:
            durable_robot_state.update({"system_status": 1, "initialization_complete": True})
        return {
            "stage": spec.key,
            "status": "controller_acknowledged" if ok else "failed",
            "ok": bool(ok),
            "failure": None if ok else "controller_stage_evidence_not_accepted",
            "component": spec.component,
            "direction": spec.direction,
            "bound": spec.bound,
            "approval_id": approval.approval_id,
            "idempotency_key": approval.idempotency_key,
            "expected_generation": approval.expected_generation,
            "command_ack": _json_safe(ack),
            "wait": _json_safe(result.get("wait")),
            "switch_predicate": _json_safe(switch),
            "position_before": before,
            "position_after": after,
            "position_delta": delta,
            "home_position": 0 if controller_reference_agrees else None,
            "controller_reference_agrees": controller_reference_agrees,
            "controller_reported_operator_assessment": None,
            "controller_reported_physical_effect": False,
            "operator_observation": None,
            "physical_effect_verified": False,
            "durable_robot_state": durable_robot_state,
            "raw_result": _json_safe(result),
        }

    @staticmethod
    def _dry_run_receipt(spec: Serial206StageSpec) -> dict[str, Any]:
        return {
            "stage": spec.key,
            "status": "planned",
            "component": spec.component,
            "direction": spec.direction,
            "bound": spec.bound,
            "approval_id": None,
            "command_ack": None,
            "operator_observation": None,
            "physical_effect_verified": False,
            "opened_usb": False,
            "physical_motion_commanded": False,
        }

    def _mark_referenced(self, spec: Serial206StageSpec, result: Any) -> None:
        axis = _HOME_STAGE_AXIS.get(spec.key)
        if axis is None or self.reference_store is None or not isinstance(result, Mapping):
            return
        if result.get("controller_reference_agrees") is not True:
            return
        self.reference_store.mark_referenced(
            MarkAxisReferencedCommand(
                axis=axis,
                position_steps=int(result.get("home_position") or 0),
                source="serial206_initializeMotors_operator_observed",
                note=f"Controller evidence and separate operator observation agreed at {spec.key}.",
                motion_kind="oem_initialize_motors_home",
            )
        )

    def _mark_desynced(self, spec: Serial206StageSpec) -> None:
        axis = _HOME_STAGE_AXIS.get(spec.key)
        if axis is None or self.reference_store is None:
            return
        self.reference_store.mark_desynced(
            MarkAxisDesyncedCommand(
                axis=axis,
                reason=f"Failed or operator-rejected serial-206 stage {spec.key}.",
                source="serial206_initializeMotors",
                motion_kind="oem_initialize_motors_failed",
            )
        )

    def _result_from_state(
        self,
        state: Mapping[str, Any],
        *,
        ok: bool,
        blockers: list[str],
        generation: int | None = None,
        stage_receipts: list[dict[str, Any]] | None = None,
        physical_motion_commanded: bool = False,
    ) -> dict[str, Any]:
        ledger = copy.deepcopy(state["movement_ledger"])
        terminal = str(ledger.get("terminal_state"))
        return {
            "ok": bool(ok),
            "ready": False,
            "initialization_complete": terminal == "initializeMotors_complete",
            "state": terminal,
            "schema": self.schema,
            "source_mode": self.source_mode,
            "mode": "live",
            "opened_usb": bool(stage_receipts),
            "physical_motion_commanded": bool(physical_motion_commanded),
            "physical_effect_verified": False,
            "generation": generation,
            "failed_at": None if ok else ledger.get("expected_next_stage"),
            "blockers": list(blockers),
            "stage_receipts": list(stage_receipts or []),
            "movement_ledger": ledger,
        }

    def _failure(
        self,
        state: str,
        blockers: list[str],
        *,
        generation: int | None = None,
        movement_ledger: Mapping[str, Any] | None = None,
        stage_receipts: list[dict[str, Any]] | None = None,
        physical_motion_commanded: bool = False,
    ) -> dict[str, Any]:
        return {
            "ok": False,
            "ready": False,
            "state": state,
            "schema": self.schema,
            "source_mode": self.source_mode,
            "mode": "live",
            "opened_usb": bool(stage_receipts),
            "physical_motion_commanded": bool(physical_motion_commanded),
            "physical_effect_verified": False,
            "generation": generation,
            "failed_at": "admission" if state in {"admission", "observation_rejected"} else None,
            "blockers": list(blockers),
            "stage_receipts": list(stage_receipts or []),
            "movement_ledger": copy.deepcopy(dict(movement_ledger or self._corrupt_projection())),
        }
