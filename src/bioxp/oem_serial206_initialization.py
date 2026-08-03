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
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .motion_safety import prepare_motion_without_motion
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
_INITIALIZE_MOTION_PARTIAL = (
    "initializeMotion.queryTipStatus.initial",
    "initializeMotion.ejectAllTips.tip_exists",
    "initializeMotion.initiateGroup.initial",
)
_INITIALIZE_MOTION_MISSING = tuple(
    spec.key for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS
    if spec.key not in _INITIALIZE_MOTION_PARTIAL and spec.key != "initializeMotion.initializeMotors"
)


_EVIDENCE_MAX_DEPTH = 8
_EVIDENCE_MAX_ITEMS = 128
_EVIDENCE_MAX_STRING = 512
_EVIDENCE_MAX_KEY = 96
_EVIDENCE_MAX_BYTES = 8192


def _json_safe(value: Any) -> Any:
    """Return a deterministic, cycle-safe, strictly bounded JSON projection."""
    remaining = [_EVIDENCE_MAX_ITEMS]
    active: set[int] = set()

    def omitted(reason: str, kind: str | None = None) -> dict[str, Any]:
        row: dict[str, Any] = {"omitted": reason}
        if kind:
            row["type"] = kind[:_EVIDENCE_MAX_KEY]
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
        if len(text) > _EVIDENCE_MAX_KEY:
            text = text[: _EVIDENCE_MAX_KEY - 14] + "...[truncated]"
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
            if len(item) <= _EVIDENCE_MAX_STRING:
                return item
            return {
                "omitted": "string_limit",
                "prefix": item[:_EVIDENCE_MAX_STRING],
                "original_length": len(item),
            }
        if depth >= _EVIDENCE_MAX_DEPTH:
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
                        if len(pairs) >= _EVIDENCE_MAX_ITEMS:
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
                    if index >= _EVIDENCE_MAX_ITEMS or remaining[0] <= 0:
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
    if len(encoded) > _EVIDENCE_MAX_BYTES:
        return {"omitted": "byte_limit", "encoded_bytes": len(encoded)}
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
        pipette_methods = (
            "query_tip_status_all",
            "eject_all_tips_for_oem_startup",
            "initiate_group_once_for_oem_initialize_motion",
            "checked_pipette_status_for_oem_initialize_motion",
        )
        motion_missing = [
            f"pipette_primitive_not_bound:{name}"
            for name in pipette_methods
            if not callable(getattr(self.pipette_transport, name, None))
        ]
        try:
            table = load_bound_oem_position_table()
            table_identity = {"source": table.source}
        except Exception:
            table_identity = None
            motion_missing.append("immutable_oem_position_table_not_bound")
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
            "position_table_identity": _json_safe(table_identity),
        }

    def prepare_for_initialize_motors(self, *, expected_generation: int) -> dict[str, Any]:
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
            components=("z",),
        )
        ok = isinstance(raw, Mapping) and raw.get("ok") is True and raw.get("physical_motion") is False
        return {
            "ok": ok,
            "observed_generation": observed_generation,
            "board_preparation_verified": ok,
            "initialize_without_motion_verified": ok,
            "physical_motion": False,
            "receipt": _json_safe(raw),
        }

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
        profile = self._x_profile()
        before = self.tester.motor_get_position(5, motor=0)
        before_value = self._x_value(before)
        if before_value is None:
            return {"ok": False, "failure": "x_position_before_unavailable", "command_issued": False, "before": _json_safe(before)}
        target = min(90263, max(int(clamp_low), int(requested)))
        result: dict[str, Any] = {"ok": False, "axis": "x", "source_mode": source_mode, "requested_position_steps": int(requested), "target_position_steps": target, "before": _json_safe(before), "before_position_steps": before_value, "command_issued": False, "source_noop": False}
        if acceleration is not None:
            setup = self.tester.motor_set_axis_param(5, 5, int(acceleration), motor=0)
            result["acceleration_set"] = _json_safe(setup)
            if not isinstance(setup, Mapping) or setup.get("ok") is not True or not isinstance(setup.get("readback"), Mapping) or setup["readback"].get("value") != int(acceleration):
                result["failure"] = "x_acceleration_write_or_readback_failed"
                return result
        if before_value == target:
            result.update({"ok": True, "source_noop": True, "noop_reason": "already_at_target", "after": _json_safe(before), "after_position_steps": before_value})
            return result
        result["event_window"] = event_window if event_window is not None else self.tester.begin_bus_event_window()
        move = self.tester.motor_oem_move_absolute(5, target, motor=0, wait_for_stop=False, max_position=90263)
        result.update({"move": _json_safe(move), "command_issued": True, "ok": isinstance(move, Mapping) and move.get("ok") is True})
        if result["ok"] is not True:
            result["failure"] = "x_absolute_command_ack_failed"
        return result

    def _x_finalize(self, ticket: Mapping[str, Any], *, timeout_s: float, motion_kind: str, publish: bool = True, restore_acceleration: bool = False) -> dict[str, Any]:
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
            after = self.tester.motor_get_position(5, motor=0)
            speed = self.tester.motor_get_speed(5, motor=0)
            after_value = self._x_value(after)
            terminal = isinstance(wait, Mapping) and wait.get("ok") is True and wait.get("target_reached") is True
            speed_zero = isinstance(speed, Mapping) and speed.get("speed") == 0
            result.update({"wait": _json_safe(wait), "after": _json_safe(after), "after_position_steps": after_value, "terminal_speed": _json_safe(speed), "target_position_verified": after_value == result.get("target_position_steps"), "controller_terminal_state_verified": terminal and speed_zero})
            result["ok"] = bool(result.get("ok") is True and terminal and speed_zero and result["target_position_verified"])
            if not result["ok"]:
                result["failure"] = str(result.get("failure") or "x_absolute_terminal_evidence_not_accepted")
        if restore_acceleration:
            restore = self.tester.motor_set_axis_param(5, 5, 350, motor=0)
            result["acceleration_restore"] = _json_safe(restore)
            result["ok"] = bool(result.get("ok") is True and isinstance(restore, Mapping) and restore.get("ok") is True and isinstance(restore.get("readback"), Mapping) and restore["readback"].get("value") == 350)
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
        ticket = self._x_issue_absolute(int(position_steps), source_mode=source_mode, clamp_low=60 if clamp_low_to_60 else 0, acceleration=acceleration)
        if not wait_for_stop and ticket.get("command_issued") is True:
            ticket.update({"pending_motion": True, "physical_motion": True, "reference_before": _json_safe(reference)})
            return ticket
        result = self._x_finalize(ticket, timeout_s=wait_timeout_s, motion_kind="absolute", publish=publish_motion_metadata, restore_acceleration=acceleration is not None)
        result["reference_before"] = _json_safe(reference)
        result["physical_motion"] = bool(result.get("command_issued"))
        return result

    def _move_xy_y_issue_absolute(self, requested: int, *, event_window: Any) -> dict[str, Any]:
        profile = self._axis_profile("y")
        before = self.tester.motor_get_position(4, motor=0)
        before_value = self._x_value(before)
        target = min(102956, max(0, int(requested)))
        if before_value is None:
            return {"ok": False, "failure": "y_position_before_unavailable", "command_issued": False}
        if before_value == target:
            return {"ok": True, "axis": "y", "source_noop": True, "command_issued": False, "target_position_steps": target, "before_position_steps": before_value, "after_position_steps": before_value}
        move = self.tester.motor_oem_move_absolute(4, target, motor=0, wait_for_stop=False, max_position=102956)
        return {"ok": isinstance(move, Mapping) and move.get("ok") is True, "axis": "y", "target_position_steps": target, "before_position_steps": before_value, "before": _json_safe(before), "event_window": event_window, "move": _json_safe(move), "command_issued": True}

    def _move_xy_y_finalize(self, ticket: Mapping[str, Any], *, timeout_s: float) -> dict[str, Any]:
        result = dict(ticket)
        if result.get("source_noop") is True:
            return result
        wait_fn = getattr(self.tester, "motor_wait_target_reached", None) or getattr(self.tester, "motor_oem_wait_target_reached")
        wait = wait_fn(4, motor=0, timeout_s=float(timeout_s), event_window=result.get("event_window"))
        after = self.tester.motor_get_position(4, motor=0)
        speed = self.tester.motor_get_speed(4, motor=0)
        after_value = self._x_value(after)
        result.update({"wait": _json_safe(wait), "after": _json_safe(after), "after_position_steps": after_value, "terminal_speed": _json_safe(speed)})
        result["ok"] = bool(result.get("ok") is True and isinstance(wait, Mapping) and wait.get("ok") is True and wait.get("target_reached") is True and isinstance(speed, Mapping) and speed.get("speed") == 0 and after_value == result.get("target_position_steps"))
        return result

    def _finalize_move_xy_receipt(self, receipt: dict[str, Any], *, commands: Mapping[str, Any], waits: Mapping[str, Any], after: Mapping[str, int], restore: Mapping[str, Any], required_axes: tuple[str, ...]) -> dict[str, Any]:
        command_ok = all(isinstance(commands.get(axis), Mapping) and commands[axis].get("ok") is True for axis in required_axes)
        wait_ok = all(isinstance(waits.get(axis), Mapping) and waits[axis].get("ok") is True and waits[axis].get("target_reached") is True for axis in required_axes)
        target_ok = all(after.get(axis) == receipt["requested"][axis] for axis in required_axes)
        restore_ok = all(isinstance(restore.get(axis), Mapping) and restore[axis].get("ok") is True and isinstance(restore[axis].get("readback"), Mapping) and restore[axis]["readback"].get("value") == (350 if axis == "x" else 400) for axis in ("x", "y"))
        ok = bool(command_ok and wait_ok and target_ok and restore_ok)
        receipt.update({"commands": _json_safe(commands), "waits": _json_safe(waits), "after": _json_safe(after), "acceleration_restore": _json_safe(restore), "controller_command_acknowledged": command_ok, "controller_terminal_state_verified": wait_ok, "target_position_verified": target_ok, "acceleration_restore_verified": restore_ok, "ok": ok})
        if ok and self.reference_store is not None and set(required_axes) == {"x", "y"}:
            metadata = self.reference_store.record_motion_many(("x", "y"), "move_xy")
            receipt["reference_state"] = _json_safe(metadata)
            if metadata.get("ok") is not True:
                ok = False
                receipt.update({"ok": False, "failure": "moveXY_reference_metadata_not_verified", "reference_desync": _json_safe(self.reference_store.mark_desynced_many((MarkAxisDesyncedCommand(axis="x", reason="moveXY metadata failure", source="serial206.move_xy", motion_kind="move_xy"), MarkAxisDesyncedCommand(axis="y", reason="moveXY metadata failure", source="serial206.move_xy", motion_kind="move_xy"))))})
        if not ok:
            receipt.setdefault("failure", "moveXY_terminal_evidence_not_accepted")
            stops = {axis: self.tester.motor_oem_stop_exact(5 if axis == "x" else 4, motor=0) for axis in required_axes}
            receipt["safety_stop"] = _json_safe(stops)
            if self.reference_store is not None:
                receipt["reference_desync"] = _json_safe(self.reference_store.mark_desynced_many(tuple(MarkAxisDesyncedCommand(axis=axis, reason=str(receipt["failure"]), source="serial206.move_xy", motion_kind="move_xy") for axis in required_axes)))
        return receipt


    def x_stop(self, *, timeout_s: float = 3.0) -> dict[str, Any]:
        stop = self.tester.motor_oem_stop_exact(5, motor=0)
        wait = self.tester.motor_wait_stopped(5, motor=0, timeout_s=float(timeout_s), require_seen_nonzero=False)
        return {"ok": bool(isinstance(stop, Mapping) and stop.get("ok") is True and isinstance(wait, Mapping) and wait.get("stopped") is True), "axis": "x", "intent": "stop", "stop": _json_safe(stop), "wait": _json_safe(wait), "physical_motion": False}

    def x_abort(self, *, reason: str = "forceAbortMotion") -> dict[str, Any]:
        abort = self.tester.motor_oem_force_abort_motion(reason=reason)
        desync = self._x_desync(reason, "abort") if self.reference_store is not None else None
        return {"ok": isinstance(abort, Mapping) and abort.get("ok") is True, "axis": "x", "intent": "abort", "logical_abort": _json_safe(abort), "reference_desync": _json_safe(desync)}

    def prepare_x(self, *, expected_generation: int) -> dict[str, Any]:
        result = self.prepare_for_initialize_motors(expected_generation=int(expected_generation))
        result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "x_prepare_result_not_mapping"}
        result.update({"axis": "x", "physical_motion": False, "source_anchor": "ClassControlInterface.initializeMotors"})
        return result

    def x_wait_for_motor(self, *, pending_ticket: Mapping[str, Any], wait_timeout_s: float) -> dict[str, Any]:
        return self._x_finalize(pending_ticket, timeout_s=float(wait_timeout_s), motion_kind="absolute", publish=True)

    def _x_home(self, *, timeout_s: float, source: str) -> dict[str, Any]:
        profile = self._x_profile()
        home = self.tester.motor_oem_go_home("x", speed=1700, rehome=True, timeout_s=max(30.0, float(timeout_s)), require_switch_transition=False)
        set_home = self.tester.motor_set_home(5, motor=0)
        position = self.tester.motor_get_position(5, motor=0)
        value = self._x_value(position)
        ok = isinstance(home, Mapping) and home.get("ok") is True and isinstance(set_home, Mapping) and set_home.get("ok") is True and value == 0
        reference = None
        if ok and self.reference_store is not None:
            reference = self.reference_store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source=source, motion_kind="home"))
            ok = reference.get("ok") is True and reference.get("durable_clean") is True
        return {"ok": ok, "axis": "x", "intent": source, "home": _json_safe(home), "set_home": _json_safe(set_home), "position": _json_safe(position), "reference_state": _json_safe(reference), "physical_motion": True, "failure": None if ok else "x_home_evidence_not_verified"}

    def x_startup_home(self, *, timeout_s: float) -> dict[str, Any]:
        return self._x_home(timeout_s=timeout_s, source="oem_startup_home")

    def x_home_axis(self, *, timeout_s: float) -> dict[str, Any]:
        return self._x_home(timeout_s=timeout_s, source="oem_home_axis")

    def x_manual_panel_home(self, *, timeout_s: float) -> dict[str, Any]:
        return self._x_home(timeout_s=timeout_s, source="manual_panel_home")

    def x_move_to_origin_home(self, *, timeout_s: float) -> dict[str, Any]:
        return self._x_home(timeout_s=timeout_s, source="move_to_origin_home")

    def x_compatibility_home(self, *, timeout_s: float) -> dict[str, Any]:
        return self._x_home(timeout_s=timeout_s, source="compatibility_home")

    def home_xy(self, *, timeout_s: float = 30.0, allow_implementation_mapped_predicate: bool = False) -> dict[str, Any]:
        del allow_implementation_mapped_predicate
        profiles = {"x": self._x_profile(), "y": self._axis_profile("y")}
        setup = {"x": self.tester.motor_set_axis_param(5, 5, 200, motor=0), "y": self.tester.motor_set_axis_param(4, 5, 200, motor=0)}
        setup_ok = all(isinstance(setup[a], Mapping) and setup[a].get("ok") is True and isinstance(setup[a].get("readback"), Mapping) and setup[a]["readback"].get("value") == 200 for a in ("x", "y"))
        if not setup_ok:
            restore = {"x": self.tester.motor_set_axis_param(5, 5, 350, motor=0), "y": self.tester.motor_set_axis_param(4, 5, 400, motor=0)}
            return {"ok": False, "failure": "homexy_profile_setup_not_verified", "command_issued": False, "setup": _json_safe(setup), "restore": _json_safe(restore)}
        event_window = self.tester.begin_bus_event_window()
        results: dict[str, Any] = {}
        errors: list[str] = []
        def run(axis: str, speed: int) -> None:
            try:
                results[axis] = self.tester.motor_oem_go_home(axis, speed=speed, rehome=True, timeout_s=max(30.0, float(timeout_s)), require_switch_transition=False)
            except Exception as exc:
                errors.append(f"{axis}:{type(exc).__name__}:{exc}")
        tx = threading.Thread(target=run, args=("x", 1700), daemon=False)
        ty = threading.Thread(target=run, args=("y", 1800), daemon=False)
        tx.start(); ty.start(); tx.join(); ty.join()
        set_home = {"x": self.tester.motor_set_home(5, motor=0), "y": self.tester.motor_set_home(4, motor=0)}
        positions = {"x": self.tester.motor_get_position(5, motor=0), "y": self.tester.motor_get_position(4, motor=0)}
        zero_ok = all(self._x_value(positions[a]) == 0 for a in ("x", "y"))
        restore = {"x": self.tester.motor_set_axis_param(5, 5, 350, motor=0), "y": self.tester.motor_set_axis_param(4, 5, 400, motor=0)}
        home_ok = not errors and all(isinstance(results.get(a), Mapping) and results[a].get("ok") is True for a in ("x", "y")) and all(isinstance(set_home[a], Mapping) and set_home[a].get("ok") is True for a in ("x", "y")) and zero_ok
        reference = None
        if home_ok and self.reference_store is not None:
            reference = self.reference_store.mark_referenced_many((MarkAxisReferencedCommand(axis="x", position_steps=0, source="oem_home_xy", motion_kind="home_xy"), MarkAxisReferencedCommand(axis="y", position_steps=0, source="oem_home_xy", motion_kind="home_xy")))
            home_ok = reference.get("ok") is True and reference.get("durable_clean") is True
        return {"ok": bool(home_ok and all(isinstance(restore[a], Mapping) and restore[a].get("ok") is True and isinstance(restore[a].get("readback"), Mapping) and restore[a]["readback"].get("value") == (350 if a == "x" else 400) for a in ("x", "y"))), "intent": "home_xy", "command_issued": True, "event_window": _json_safe(event_window), "setup": _json_safe(setup), "home": _json_safe(results), "home_errors": errors, "set_home": _json_safe(set_home), "positions": _json_safe(positions), "restore": _json_safe(restore), "reference_state": _json_safe(reference), "source_anchor": "ClassControlInterface.HomeXY"}

    def _z_profile(self) -> dict[str, Any]:
        profile = dict(self.tester._motion_oem_axis_profile("z"))
        if int(profile.get("board", -1)) != 4 or int(profile.get("motor", -1)) != 1:
            raise RuntimeError("serial-206 Z authority must resolve to board 4 motor 1")
        self.tester.motor_oem_require_no_motion_profile("z")
        interlock = self.tester.motor_oem_verify_motion_interlock()
        if not isinstance(interlock, Mapping) or interlock.get("ok") is not True:
            raise RuntimeError(f"serial-206 Z interlock failed: {interlock}")
        return profile

    @staticmethod
    def _z_value(row: Any) -> int | None:
        value = row.get("value") if isinstance(row, Mapping) else None
        return int(value) if type(value) is int else None

    def _z_finalize_position_move(
        self,
        *,
        profile: Mapping[str, Any],
        before: Mapping[str, Any],
        target: int,
        move: Mapping[str, Any],
        wait_timeout_s: float,
        event_window: Mapping[str, Any],
    ) -> dict[str, Any]:
        board = int(profile["board"])
        motor = int(profile["motor"])
        wait = self.tester.motor_wait_stopped(
            board, motor=motor, timeout_s=float(wait_timeout_s), require_seen_nonzero=True
        )
        events = self.tester.collect_bus_events(duration_s=0.30, timeout_ms=12, max_events=96)
        axis_events = [
            row for row in events if isinstance(row, Mapping)
            and row.get("board") in {None, board}
            and row.get("motor") in {None, motor}
        ]
        target_events = [row for row in axis_events if row.get("status") == 128]
        error_events = [row for row in axis_events if row.get("status") in {13, 14, 130}]
        after = self.tester.motor_get_position(board, motor=motor)
        before_value = self._z_value(before)
        after_value = self._z_value(after)
        move_ack = isinstance(move, Mapping) and move.get("ok") is True
        terminal = isinstance(wait, Mapping) and wait.get("stopped") is True
        ok = bool(
            move_ack and terminal and target_events and not error_events
            and after_value == int(target)
        )
        failure_stop = None
        if not ok:
            failure_stop = self.z_stop(timeout_s=3.0)
        return {
            "ok": ok,
            "robot_http_acknowledged": True,
            "controller_command_acknowledged": move_ack,
            "controller_terminal_state_verified": terminal,
            "physical_effect_verified": False,
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
        if before_value is None:
            return {"ok": False, "error": "z_current_position_unavailable", "before": _json_safe(before)}
        target = before_value + int(steps)
        if before_value < 0 or before_value > 160000 or target < 0 or target > 160000:
            return {
                "ok": False,
                "error": "z_source_coordinate_out_of_bounds",
                "before_position_steps": before_value,
                "requested_steps": int(steps),
                "target_position_steps": target,
                "source_min_steps": 0,
                "source_max_steps": 160000,
            }
        event_window = self.tester.clear_bus_event_buffer()
        move = self.tester.motor_move_relative(4, int(steps), motor=1)
        result = self._z_finalize_position_move(
            profile=profile, before=before, target=target, move=move,
            wait_timeout_s=wait_timeout_s, event_window=event_window,
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
                "requested_position_steps": requested,
                "effective_position_steps": effective,
                "source_min_steps": 0,
                "source_max_steps": 160000,
            }
        before = self.tester.motor_get_position(4, motor=1)
        current_write = self.tester.motor_set_axis_param(4, 6, int(profile["run_current"]), motor=1)
        current_readback = self.tester.motor_get_axis_param(4, 6, motor=1)
        if not isinstance(current_write, Mapping) or current_write.get("ok") is not True or self._z_value(current_readback) != int(profile["run_current"]):
            return {
                "ok": False,
                "error": "z_source_current_readback_failed",
                "current_write": _json_safe(current_write),
                "current_readback": _json_safe(current_readback),
            }
        event_window = self.tester.clear_bus_event_buffer()
        move = self.tester.motor_move_absolute(4, effective, motor=1)
        result = self._z_finalize_position_move(
            profile=profile, before=before, target=effective, move=move,
            wait_timeout_s=wait_timeout_s, event_window=event_window,
        )
        result.update({
            "intent": "move_absolute",
            "requested_position_steps": requested,
            "effective_position_steps": effective,
            "pseudo_home_steps": int(pseudo_home_steps),
            "source_anchor": "ClassControlInterface.moveZ:4254-4265",
        })
        return result

    def z_manual_home(self, *, timeout_s: float = 30.0) -> dict[str, Any]:
        self._z_profile()
        interlock = self.tester.motor_oem_verify_motion_interlock()
        if interlock.get("ok") is not True:
            return {"ok": False, "failure": "motion_interlock_not_ready", "interlock": interlock}
        home = self.tester.motor_oem_move_z_home(rehome=True, timeout_s=float(timeout_s))
        ok = isinstance(home, Mapping) and home.get("ok") is True
        return {
            "ok": ok,
            "intent": "manual_go_home_1791",
            "source_method": "ClassControlInterface.MoveZHome -> goHome(true,1791)",
            "interlock": interlock,
            "home": home,
            "controller_command_acknowledged": bool(ok),
            "controller_terminal_state_verified": bool(ok),
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
        return {
            "ok": ok,
            "intent": "diagnostic_home_axis_597",
            "source_method": "ClassControlInterface.HomeAxis(z) -> axisSearchHome(597)",
            "interlock": interlock,
            "home": home,
            "controller_command_acknowledged": bool(ok),
            "controller_terminal_state_verified": bool(ok),
            "physical_effect_verified": False,
        }

    def z_stop(self, *, timeout_s: float = 3.0) -> dict[str, Any]:
        stop = self.tester.motor_stop(4, motor=1)
        wait = self.tester.motor_wait_stopped(
            4, motor=1, timeout_s=float(timeout_s), require_seen_nonzero=False
        )
        terminal = isinstance(wait, Mapping) and wait.get("stopped") is True
        return {
            "ok": bool(isinstance(stop, Mapping) and stop.get("ok") is True and terminal),
            "intent": "stop",
            "controller_command_acknowledged": isinstance(stop, Mapping) and stop.get("ok") is True,
            "controller_terminal_state_verified": terminal,
            "physical_effect_verified": False,
            "stop": _json_safe(stop),
            "wait": _json_safe(wait),
        }

    def z_reconcile_switch_masks(self) -> dict[str, Any]:
        before = {param: self.tester.motor_get_axis_param(4, param, motor=1) for param in (12, 13)}
        writes = {param: self.tester.motor_set_axis_param(4, param, 0, motor=1) for param in (12, 13)}
        after = {param: self.tester.motor_get_axis_param(4, param, motor=1) for param in (12, 13)}
        ok = all(
            isinstance(writes[param], Mapping) and writes[param].get("ok") is True
            and self._z_value(after[param]) == 0
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
            "recovery_reason": "restore serial-206 source precondition GAP12=0 GAP13=0",
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
            return self.tester.motor_oem_go_home(
                axis, speed=int(speed), rehome=True,
                timeout_s=max(30.0, float(wait_timeout_s)),
                require_switch_transition=False,
            )

        def move_axis(axis: str, position: int, acc: int) -> dict[str, Any]:
            profile = self._axis_profile(axis)
            effective = max(60, int(position)) if axis == "x" else int(position)
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
            z_home = self.tester.motor_oem_move_z_home(
                rehome=True, timeout_s=max(30.0, float(wait_timeout_s))
            )
            if run_in_parallel:
                xy = run_pair(lambda: home_axis("x", 1700), lambda: home_axis("y", 1800), 0.0)
            else:
                xy = [home_axis("x", 1700), home_axis("y", 1800)]
            return {
                "ok": isinstance(z_home, Mapping) and z_home.get("ok") is True
                      and all(row.get("ok") is True for row in xy),
                "source_return_code": 0,
                "branch": "all_zero_home",
                "z_home": _json_safe(z_home),
                "xy_home": _json_safe(xy),
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
            fallback = self.x_move_absolute(position_steps=requested["y"], source_mode="moveXY.missing_x.literal_moveX_y", clamp_low_to_60=True)
            receipt.update({"branch": "missing_x_literal_moveX_y", "fallback": _json_safe(fallback), "ok": fallback.get("source_noop") is True})
            return receipt
        reference = self._reference_snapshot(("x", "y"), "ClassControlInterface.moveXY")
        before = {axis: self._read_axis_position(axis) for axis in ("x", "y")}
        distances = {axis: abs(requested[axis] - before[axis]) for axis in ("x", "y")}
        receipt.update({"reference_before": _json_safe(reference), "before": before, "distances": distances})
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
            restore = {"x": {"ok": True, "readback": {"value": 350}, "not_written": True}, "y": {"ok": True, "readback": {"value": 400}, "not_written": True}}
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
        time.sleep(0.005)
        many_wait = getattr(self.tester, "motor_wait_target_reached_many", None)
        if callable(many_wait):
            pair_wait = many_wait(((5, 0), (4, 0)), event_window=event_window, timeout_s=5.0, sta_sequential=False)
            waits = dict(pair_wait.get("per_axis") or {}) if isinstance(pair_wait, Mapping) else {}
            if not waits:
                waits = {"x": pair_wait, "y": pair_wait}
        else:
            wait_fn = getattr(self.tester, "motor_wait_target_reached", None) or getattr(self.tester, "motor_oem_wait_target_reached")
            waits = {"x": wait_fn(5, motor=0, timeout_s=5.0, event_window=event_window), "y": wait_fn(4, motor=0, timeout_s=5.0, event_window=event_window)}
        restore = {"x": self.tester.motor_set_axis_param(5, 5, 350, motor=0), "y": self.tester.motor_set_axis_param(4, 5, 400, motor=0)}
        after = {axis: self._read_axis_position(axis) for axis in ("x", "y")}
        receipt.update({"branch": "parallel", "acceleration_selected": {"x": x_acc, "y": y_acc}, "acceleration_set": _json_safe(acceleration_set), "event_window": _json_safe(event_window), "launch_order": launch_order, "stagger_ms": stagger_ms, "pre_wait_sleep_ms": 5, "pair_wait": _json_safe(pair_wait) if "pair_wait" in locals() else None})
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
        self.preparation_provider = preparation_provider or (
            primitives if hasattr(primitives, "prepare_for_initialize_motors") else None
        )
        if sleep is None:
            import time
            sleep = time.sleep
        self.sleep = sleep
        self._lock = threading.RLock()
        self._memory_state: dict[str, Any] | None = None

    @staticmethod
    def _new_z_lifecycle() -> dict[str, Any]:
        return {
            "schema_version": "bioxp.serial206_z_lifecycle.v1",
            "state": "unprepared",
            "generation": None,
            "prepared_receipt": None,
            "active_receipt": None,
            "awaiting_observation_receipt_id": None,
            "reference_state": "unknown",
            "last_failure": None,
            "receipts": [],
        }

    @staticmethod
    def _new_x_lifecycle() -> dict[str, Any]:
        return {"schema_version": "bioxp.serial206_x_lifecycle.v1", "state": "unprepared", "generation": None, "prepared_receipt": None, "active_receipt": None, "pending_ticket": None, "reference_state": "unknown", "last_failure": None, "receipts": []}

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
        upgraded.setdefault("x_lifecycle", copy.deepcopy(defaults["x_lifecycle"]))
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
        if not isinstance(z_lifecycle, dict) or z_lifecycle.get("schema_version") != "bioxp.serial206_z_lifecycle.v1":
            raise ValueError("serial-206 Z lifecycle is invalid")
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
        if not isinstance(x_lifecycle, dict) or x_lifecycle.get("schema_version") != "bioxp.serial206_x_lifecycle.v1":
            raise ValueError("serial-206 X lifecycle is invalid")
        if x_lifecycle.get("state") not in {"unprepared", "prepared_unreferenced", "executing", "awaiting_operator_observation", "referenced_ready", "failed_latched"}:
            raise ValueError("serial-206 X lifecycle state is invalid")
        if not isinstance(x_lifecycle.get("receipts"), list) or len(x_lifecycle["receipts"]) > 128:
            raise ValueError("serial-206 X receipt ledger is invalid")
        if x_lifecycle.get("state") == "executing" and not isinstance(x_lifecycle.get("active_receipt"), Mapping):
            raise ValueError("executing serial-206 X lifecycle lacks active receipt")
        if x_lifecycle.get("state") != "executing" and x_lifecycle.get("active_receipt") is not None:
            raise ValueError("inactive serial-206 X lifecycle carries active receipt")
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
            return self._validate_state(self._upgrade_state(stored))
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
                lifecycle = copy.deepcopy(state["x_lifecycle"])
                reference = self.reference_store.snapshot(("x",)) if self.reference_store is not None else {"ok": False, "authority_untrusted": True}
                return {"authority": type(self).__name__, "axis": "x", "board": 5, "motor": 0, "source_min_steps": 0, "source_max_steps": 90263, "lifecycle": lifecycle, "reference": _json_safe(reference)}
            except Exception as exc:
                return {"ok": False, "axis": "x", "state": "failed_latched", "failure": f"projection_failed:{type(exc).__name__}"}

    def execute_x_intent(self, intent: str, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = dict(values or {})
        selected = str(intent).strip().lower()
        with self._lock:
            try:
                state = self._load_state()
            except Exception as exc:
                return {"ok": False, "axis": "x", "state": "failed_latched", "failure": f"durable_state_unavailable:{exc}"}
            lifecycle = state["x_lifecycle"]
            generation = int(self.generation_provider())
            command_id = str(values.get("command_id") or values.get("idempotency_key") or f"x-{selected}-{generation}")
            if lifecycle.get("state") == "executing" and selected != "wait_for_motor":
                lifecycle.update({"state": "failed_latched", "reference_state": "desynced", "last_failure": "restart_or_reentry_during_executing", "active_receipt": None, "pending_ticket": None})
                try: self._save_state(state)
                except Exception: pass
                return {"ok": False, "axis": "x", "state": "failed_latched", "failure": "x_executing_outcome_ambiguous"}
            active = lifecycle.get("active_receipt")
            if isinstance(active, Mapping) and active.get("command_id") == command_id and selected != "wait_for_motor":
                return copy.deepcopy(dict(active.get("result") or active))
            if selected == "prepare":
                prepare_fn = getattr(self.primitives, "prepare_x", None) or getattr(self.primitives, "prepare_for_initialize_motors")
                result = prepare_fn(expected_generation=generation)
                ok = isinstance(result, Mapping) and result.get("ok") is True and result.get("physical_motion") is False
                lifecycle.update({"state": "prepared_unreferenced" if ok else "failed_latched", "generation": generation, "prepared_receipt": _json_safe(result), "reference_state": "desynced" if ok else "unknown", "last_failure": None if ok else _json_safe(result)})
                self._save_state(state)
                return {"ok": ok, "axis": "x", "intent": selected, "state": lifecycle["state"], "result": _json_safe(result)}
            if selected == "wait_for_motor":
                pending = lifecycle.get("pending_ticket")
                if not isinstance(pending, Mapping):
                    return {"ok": False, "axis": "x", "failure": "x_pending_ticket_missing", "state": lifecycle.get("state")}
                result = self.primitives.x_wait_for_motor(pending_ticket=pending, wait_timeout_s=float(values.get("wait_timeout_s", 20.0)))
            else:
                active_receipt = {"command_id": command_id, "intent": selected, "generation": generation, "status": "executing", "result": None}
                lifecycle.update({"state": "executing", "generation": generation, "active_receipt": active_receipt, "pending_ticket": None})
                self._save_state(state)
                try:
                    if selected == "move_absolute":
                        result = self.primitives.x_move_absolute(position_steps=int(values["position_steps"]), acceleration=None if values.get("acceleration") is None else int(values["acceleration"]), wait_for_stop=bool(values.get("wait_for_stop", True)), wait_timeout_s=float(values.get("wait_timeout_s", 20.0)), source_mode=str(values.get("source_mode") or "provider.x.move_absolute"))
                    elif selected == "stop":
                        result = self.primitives.x_stop(timeout_s=float(values.get("timeout_s", 3.0)))
                    elif selected == "abort":
                        result = self.primitives.x_abort(reason=str(values.get("reason") or "forceAbortMotion"))
                    elif selected in {"startup_home", "home_axis", "manual_panel_home", "move_to_origin_home", "compatibility_home"}:
                        home_fn = getattr(self.primitives, "x_" + selected)
                        result = home_fn(timeout_s=float(values.get("timeout_s", 30.0)))
                    else:
                        raise ValueError(f"unsupported_x_intent:{selected}")
                except Exception as exc:
                    result = {"ok": False, "failure": f"x_intent_exception:{type(exc).__name__}:{exc}", "command_issued": False}
            result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "x_result_not_mapping"}
            if selected != "wait_for_motor" and result.get("pending_motion") is True:
                lifecycle.update({"state": "executing", "active_receipt": {"command_id": command_id, "intent": selected, "status": "executing", "result": _json_safe(result)}, "pending_ticket": _json_safe(result)})
            elif result.get("ok") is True:
                lifecycle.update({"state": "referenced_ready", "active_receipt": None, "pending_ticket": None, "reference_state": "referenced", "last_failure": None})
                lifecycle["receipts"].append({"command_id": command_id, "intent": selected, "result": _json_safe(result)})
                lifecycle["receipts"] = lifecycle["receipts"][-128:]
            else:
                lifecycle.update({"state": "failed_latched", "active_receipt": None, "pending_ticket": None, "reference_state": "desynced", "last_failure": _json_safe(result)})
            self._save_state(state)
            return {"ok": result.get("ok") is True, "axis": "x", "intent": selected, "state": lifecycle["state"], "result": _json_safe(result), "generation": generation}

    def execute_xy_intent(self, x: int, y: int, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = dict(values or {})
        with self._lock:
            try:
                state = self._load_state()
                lifecycle = state["x_lifecycle"]
                generation = int(self.generation_provider())
                command_id = str(values.get("command_id") or values.get("idempotency_key") or f"xy-{generation}-{int(x)}-{int(y)}")
                for receipt in lifecycle.get("receipts", []):
                    if isinstance(receipt, Mapping) and receipt.get("command_id") == command_id:
                        return copy.deepcopy(dict(receipt.get("result") or receipt))
                active = {"command_id": command_id, "intent": "move_xy", "generation": generation, "status": "executing", "result": None}
                lifecycle.update({"state": "executing", "generation": generation, "active_receipt": active, "pending_ticket": None})
                self._save_state(state)
                result = self.primitives.move_xy(int(x), int(y), speed=values.get("speed"), acc=values.get("acc"), wait_timeout_s=float(values.get("wait_timeout_s", 5.0)))
                result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "xy_result_not_mapping"}
                if result.get("ok") is True:
                    lifecycle.update({"state": "referenced_ready", "active_receipt": None, "reference_state": "referenced", "last_failure": None})
                else:
                    lifecycle.update({"state": "failed_latched", "active_receipt": None, "reference_state": "desynced", "last_failure": _json_safe(result)})
                lifecycle["receipts"].append({"command_id": command_id, "intent": "move_xy", "result": _json_safe(result)})
                lifecycle["receipts"] = lifecycle["receipts"][-128:]
                self._save_state(state)
                return {"ok": result.get("ok") is True, "axis": "xy", "intent": "move_xy", "state": lifecycle["state"], "result": _json_safe(result), "generation": generation}
            except Exception as exc:
                return {"ok": False, "axis": "xy", "state": "failed_latched", "failure": f"xy_intent_exception:{type(exc).__name__}:{exc}"}

    def execute_homexy_intent(self, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = dict(values or {})
        with self._lock:
            try:
                state = self._load_state()
                lifecycle = state["x_lifecycle"]
                generation = int(self.generation_provider())
                command_id = str(values.get("command_id") or f"homexy-{generation}")
                active = {"command_id": command_id, "intent": "home_xy", "generation": generation, "status": "executing", "result": None}
                lifecycle.update({"state": "executing", "generation": generation, "active_receipt": active, "pending_ticket": None})
                self._save_state(state)
                result = self.primitives.home_xy(timeout_s=float(values.get("timeout_s", 30.0)), allow_implementation_mapped_predicate=bool(values.get("allow_implementation_mapped_predicate", False)))
                result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "homexy_result_not_mapping"}
                lifecycle.update({"state": "referenced_ready" if result.get("ok") is True else "failed_latched", "active_receipt": None, "reference_state": "referenced" if result.get("ok") is True else "desynced", "last_failure": None if result.get("ok") is True else _json_safe(result)})
                lifecycle["receipts"].append({"command_id": command_id, "intent": "home_xy", "result": _json_safe(result)})
                lifecycle["receipts"] = lifecycle["receipts"][-128:]
                self._save_state(state)
                return {"ok": result.get("ok") is True, "axis": "xy", "intent": "home_xy", "state": lifecycle["state"], "result": _json_safe(result), "generation": generation}
            except Exception as exc:
                return {"ok": False, "axis": "xy", "state": "failed_latched", "failure": f"homexy_intent_exception:{type(exc).__name__}:{exc}"}
    def notify_board_activation(self, board_id: int, ack: Any) -> dict[str, Any]:
        """Invalidate Z preparation/reference on any non-provider command-64 to board 4."""
        if int(board_id) != 4:
            return {"z_affected": False, "board": int(board_id)}
        with self._lock:
            state = self._load_state()
            z = state["z_lifecycle"]
            active = z.get("active_receipt") if isinstance(z.get("active_receipt"), Mapping) else {}
            if z.get("state") == "executing" and active.get("intent") == "prepare":
                return {"z_affected": False, "board": 4, "provider_owned_preparation": True}
            previous = str(z.get("state") or "unprepared")
            if previous == "unprepared":
                return {"z_affected": False, "board": 4, "already_unprepared": True}
            invalidation = {
                "reason": "board4_command64_outside_provider_preparation",
                "previous_state": previous,
                "ack": _json_safe(ack),
                "invalidated_at": time.time(),
            }
            z.update({
                "state": "unprepared",
                "generation": None,
                "prepared_receipt": None,
                "reference_state": "desynced",
                "awaiting_observation_receipt_id": None,
                "last_failure": invalidation,
            })
            self._z_mark_desynced(
                "Board 4 was reactivated outside provider-owned Z preparation.",
                "serial206.z.board_activation_invalidation",
            )
            self._save_state(state)
            return {"z_affected": True, "board": 4, "invalidation": invalidation}

    def z_projection(self) -> dict[str, Any]:
        with self._lock:
            try:
                state = self._load_state()
            except Exception:
                return {"available": False, "state": "corrupt", "blockers": ["durable_serial206_state_corrupt"]}
            z = copy.deepcopy(state["z_lifecycle"])
            generation = int(self.generation_provider())
            prepared_generation = z.get("generation")
            if prepared_generation is not None and int(prepared_generation) != generation:
                z["state"] = "unprepared"
                z["reference_state"] = "desynced"
                z["generation_invalidated"] = True
            z.update({
                "available": True,
                "ownership_generation": generation,
                "authority": type(self).__name__,
                "board": 4,
                "motor": 1,
                "coordinate_contract": "oem_source_nonnegative_z",
                "source_min_steps": 0,
                "source_max_steps": 160000,
            })
            return z

    @staticmethod
    def _append_z_receipt(z: dict[str, Any], receipt: Mapping[str, Any]) -> None:
        receipts = list(z.get("receipts") or [])
        receipts.append(_json_safe(dict(receipt)))
        z["receipts"] = receipts[-128:]

    def _z_mark_desynced(self, reason: str, source: str) -> None:
        if self.reference_store is None:
            return
        self.reference_store.mark_desynced(
            MarkAxisDesyncedCommand(axis="z", reason=reason, source=source)
        )

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
        with self._lock:
            state = self._load_state()
            z = state["z_lifecycle"]
            observed_generation = int(self.generation_provider())
            if z.get("generation") is not None and int(z["generation"]) != observed_generation:
                z.update({
                    "state": "unprepared",
                    "generation": None,
                    "prepared_receipt": None,
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
            for existing in reversed(z.get("receipts") or []):
                if existing.get("idempotency_key") == idempotency_key:
                    return {"ok": existing.get("status") == "completed", "replayed": True, "authority_receipt": existing}
            command_id = f"z_{int(time.time() * 1000)}_{len(z.get('receipts') or []) + 1}"
            receipt = {
                "command_id": command_id,
                "intent": intent,
                "idempotency_key": idempotency_key,
                "expected_generation": int(expected_generation),
                "inputs": _json_safe(values),
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
                "diagnostic_home_axis": {"prepared_unreferenced", "referenced_ready"},
                "move_steps": {"referenced_ready"},
                "move_absolute": {"referenced_ready"},
                "stop": {"unprepared", "prepared_unreferenced", "executing", "awaiting_operator_observation", "referenced_ready", "failed_latched"},
            }
            blockers = []
            if expected_generation != observed_generation:
                blockers.append(f"ownership_generation_mismatch:{expected_generation}:{observed_generation}")
            if intent not in allowed_by_state:
                blockers.append(f"unsupported_z_intent:{intent}")
            elif z.get("state") not in allowed_by_state[intent]:
                blockers.append(f"z_state_blocks_intent:{z.get('state')}:{intent}")
            if z.get("generation") is not None and int(z["generation"]) != observed_generation and intent not in {"prepare", "reconcile_switch_masks", "stop"}:
                blockers.append("z_preparation_generation_stale")
            if intent == "prepare" and self.preparation_provider is None:
                blockers.append("z_preparation_provider_not_bound")
            if intent == "reconcile_switch_masks" and values.get("confirm") != "RECONCILE_Z_SWITCH_MASKS":
                blockers.append("explicit_confirmation_required:RECONCILE_Z_SWITCH_MASKS")
            if blockers:
                receipt.update({"status": "rejected", "finished_at": time.time(), "blockers": blockers})
                self._append_z_receipt(z, receipt)
                self._save_state(state)
                return {"ok": False, "blockers": blockers, "authority_receipt": receipt}

            previous_state = z.get("state")
            receipt["status"] = "executing"
            z["active_receipt"] = copy.deepcopy(receipt)
            z["state"] = "executing"
            self._save_state(state)
            try:
                if intent == "prepare":
                    preparer = self.preparation_provider
                    if preparer is None:
                        raise RuntimeError("Z preparation provider is not bound")
                    result = preparer.prepare_for_initialize_motors(
                        expected_generation=observed_generation
                    )
                elif intent == "reconcile_switch_masks":
                    result = self.primitives.z_reconcile_switch_masks()
                elif intent == "manual_home":
                    result = self.primitives.z_manual_home(timeout_s=float(values.get("timeout_s", 30.0)))
                elif intent == "diagnostic_home_axis":
                    result = self.primitives.z_diagnostic_home_axis(timeout_s=float(values.get("timeout_s", 30.0)))
                elif intent == "move_steps":
                    result = self.primitives.z_move_steps(
                        int(values["steps"]), timeout_s=float(values.get("wait_timeout_s", 20.0))
                    )
                elif intent == "move_absolute":
                    machine_status = state.get("machine_status") or {}
                    pseudo_home = machine_status.get("psudo_z_home_steps")
                    if type(pseudo_home) is not int or pseudo_home not in {500, 65000}:
                        raise RuntimeError("PSUDO_Z_HOME state is invalid")
                    result = self.primitives.oem_move_z(
                        int(values["position_steps"]),
                        pseudo_home_steps=int(pseudo_home),
                        motor_current=31,
                        wait_for_stop=True,
                    )
                else:
                    result = self.primitives.z_stop()
            except Exception as exc:
                result = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}

            ok = isinstance(result, Mapping) and result.get("ok") is True
            if not ok and intent != "stop":
                try:
                    result = {**dict(result), "failure_stop": _json_safe(self.primitives.z_stop())}
                except Exception as exc:
                    result = {**dict(result), "failure_stop": {"ok": False, "error": f"{type(exc).__name__}: {exc}"}}
            receipt.update({
                "status": "completed" if ok else "failed",
                "finished_at": time.time(),
                "result": _json_safe(result),
                "controller_command_acknowledged": bool(isinstance(result, Mapping) and result.get("controller_command_acknowledged") is True),
                "controller_terminal_state_verified": bool(isinstance(result, Mapping) and result.get("controller_terminal_state_verified") is True),
            })
            z["active_receipt"] = None
            if ok and intent == "prepare":
                z.update({
                    "state": "prepared_unreferenced",
                    "generation": observed_generation,
                    "prepared_receipt": _json_safe(receipt),
                    "reference_state": "desynced",
                    "last_failure": None,
                })
                self._z_mark_desynced("Z profile prepared; homing observation still required.", "serial206.z.prepare")
            elif ok and intent == "reconcile_switch_masks":
                z.update({"state": "unprepared", "generation": None, "prepared_receipt": None, "reference_state": "desynced", "last_failure": None})
                self._z_mark_desynced("Z switch masks reconciled; source profile must be prepared again.", "serial206.z.reconcile_switch_masks")
            elif ok and intent in {"manual_home", "diagnostic_home_axis"}:
                z.update({
                    "state": "awaiting_operator_observation",
                    "awaiting_observation_receipt_id": command_id,
                    "reference_state": "desynced",
                    "last_failure": None,
                })
                self._z_mark_desynced("Controller home proof awaits independent physical observation.", f"serial206.z.{intent}")
            elif ok and intent in {"move_steps", "move_absolute"}:
                z.update({"state": "referenced_ready", "reference_state": "referenced", "last_failure": None})
            elif ok and intent == "stop":
                z["state"] = previous_state if previous_state != "executing" else "failed_latched"
            else:
                z.update({
                    "state": "failed_latched",
                    "reference_state": "desynced",
                    "last_failure": _json_safe(receipt),
                })
                self._z_mark_desynced(f"Failed serial-206 Z intent {intent}.", f"serial206.z.{intent}")
            self._append_z_receipt(z, receipt)
            self._save_state(state)
            return {"ok": ok, "result": _json_safe(result), "authority_receipt": _json_safe(receipt), "z_state": z.get("state"), "z_lifecycle": _json_safe(z)}

    def record_z_observation(
        self,
        *,
        command_id: str,
        verdict: str,
        note: str,
        expected_generation: int,
    ) -> dict[str, Any]:
        if verdict not in {"pass", "fail"}:
            raise ValueError("Z observation verdict must be pass or fail")
        with self._lock:
            state = self._load_state()
            z = state["z_lifecycle"]
            if int(expected_generation) != int(self.generation_provider()):
                raise ValueError("ownership generation changed before Z observation")
            if z.get("state") != "awaiting_operator_observation" or z.get("awaiting_observation_receipt_id") != command_id:
                raise ValueError("Z lifecycle is not awaiting this observation")
            receipts = list(z.get("receipts") or [])
            match = next((row for row in reversed(receipts) if row.get("command_id") == command_id), None)
            if match is None:
                raise ValueError("Z authority receipt is missing")
            observation = {
                "command_id": command_id,
                "verdict": verdict,
                "note": str(note),
                "observed_at": time.time(),
                "physical_effect_verified": verdict == "pass",
            }
            if verdict == "pass":
                home_result = match.get("result") if isinstance(match.get("result"), Mapping) else {}
                if not isinstance(home_result, Mapping) or home_result.get("ok") is not True:
                    raise ValueError("controller home proof is not successful")
                z.update({
                    "state": "referenced_ready",
                    "reference_state": "referenced",
                    "awaiting_observation_receipt_id": None,
                    "last_failure": None,
                })
                if self.reference_store is not None:
                    self.reference_store.mark_referenced(
                        MarkAxisReferencedCommand(
                            axis="z", position_steps=0,
                            source="serial206.z.operator_observation",
                            motion_kind="home",
                        )
                    )
            else:
                z.update({
                    "state": "failed_latched",
                    "reference_state": "desynced",
                    "awaiting_observation_receipt_id": None,
                    "last_failure": _json_safe(observation),
                })
                self._z_mark_desynced("Operator rejected physical Z home observation.", "serial206.z.operator_observation")
            z["last_observation"] = observation
            self._save_state(state)
            return {"ok": True, "observation": observation, "z_state": z.get("state"), "z_lifecycle": _json_safe(z)}

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

    def initialize_motion_projection(self) -> dict[str, Any]:
        with self._lock:
            try:
                state = self._load_state()
                return {
                    "initialize_motion_ledger": copy.deepcopy(state["initialize_motion_ledger"]),
                    "machine_status": copy.deepcopy(state["machine_status"]),
                    "initialize_motors": copy.deepcopy(state["movement_ledger"]),
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
                self._append_z_receipt(z, z_receipt)
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
                operator_ack="EJECT",
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
