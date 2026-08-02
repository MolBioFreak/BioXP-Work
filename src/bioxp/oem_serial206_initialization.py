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
from .oem_movement_ledger import OemMovementLedger, OEM_INITIALIZE_MOTORS_STAGE_KEYS
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
        "motor_get_position",
        "motor_set_home",
        "motor_move_absolute",
        "motor_prepare_axis",
        "motor_oem_door_search_home",
        "motor_oem_open_thermal_door",
        "motor_thermal_door_status",
        "_motion_oem_axis_profile",
        "_machine_config_bundle",
        "_oem_no_motion_tmcl_with_readback",
    )

    def __init__(
        self,
        tester: Any,
        pipette_transport: Any,
        *,
        authority_provider: Callable[[], Any],
        generation_provider: Callable[[], int],
    ) -> None:
        self.tester = tester
        self.pipette_transport = pipette_transport
        self.authority_provider = authority_provider
        self.generation_provider = generation_provider
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
        raw = prepare_motion_without_motion(self.tester, self.authority_provider())
        ok = isinstance(raw, Mapping) and raw.get("ok") is True and raw.get("physical_motion") is False
        return {
            "ok": ok,
            "observed_generation": observed_generation,
            "board_preparation_verified": ok,
            "initialize_without_motion_verified": ok,
            "physical_motion": False,
            "receipt": _json_safe(raw),
        }

    def motor_oem_home_axis(self, *args: Any, **kwargs: Any) -> Any:
        return self.tester.motor_oem_home_axis(*args, **kwargs)

    def motor_set_axis_param(self, *args: Any, **kwargs: Any) -> Any:
        return self.tester.motor_set_axis_param(*args, **kwargs)

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

    def motor_set_home(self, *args: Any, **kwargs: Any) -> Any:
        return self.tester.motor_set_home(*args, **kwargs)

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
        return self.tester._oem_no_motion_tmcl_with_readback(**kwargs)

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
    ) -> dict[str, Any]:
        profile = self._axis_profile(axis)
        before = self.tester.motor_get_position(profile["board"], motor=profile.get("motor", 0))
        prepared = self._prepare_path_axis(axis, speed=speed, acc=acc)
        move = self.tester.motor_move_absolute(
            profile["board"],
            int(position),
            motor=profile.get("motor", 0),
        )
        wait = self.tester.motor_wait_stopped(
            profile["board"],
            motor=profile.get("motor", 0),
            timeout_s=float(timeout_s),
            require_seen_nonzero=True,
        )
        after = self.tester.motor_get_position(profile["board"], motor=profile.get("motor", 0))
        after_value = self._position_value(after)
        ok = bool(
            isinstance(move, Mapping)
            and move.get("ok") is True
            and isinstance(wait, Mapping)
            and wait.get("stopped") is True
            and after_value == int(position)
        )
        return {
            "ok": ok,
            "axis": str(axis).lower(),
            "target": int(position),
            "prepare": _json_safe(prepared),
            "move": _json_safe(move),
            "wait": _json_safe(wait),
            "position": {"before": _json_safe(before), "after": _json_safe(after)},
            "source_anchor": "ControlLib.initializeMotion:8815-8816; ClassControlInterface.moveZ/moveX",
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
        del acc
        profiles = {axis: self._axis_profile(axis) for axis in ("x", "y")}
        before = {axis: self._read_axis_position(axis) for axis in ("x", "y")}
        targets = {"x": int(x), "y": int(y)}
        distances = {axis: abs(targets[axis] - before[axis]) for axis in ("x", "y")}
        prepared = {
            "x": self._prepare_path_axis("x", speed=speed, acc=350),
            "y": self._prepare_path_axis("y", speed=speed, acc=400),
        }
        boosted: dict[str, Any] = {}
        commands: dict[str, Any] = {}
        launch_order: list[str] = []
        stagger_ms = 0
        if distances["x"] > distances["y"]:
            if distances["x"] > 10000:
                boosted["x"] = self.tester.motor_set_axis_param(
                    profiles["x"]["board"], 5, 750, motor=profiles["x"].get("motor", 0)
                )
                boosted["y"] = self.tester.motor_set_axis_param(
                    profiles["y"]["board"], 5, 1500, motor=profiles["y"].get("motor", 0)
                )
            if distances["x"] > 20:
                commands["x"] = self.tester.motor_move_absolute(
                    profiles["x"]["board"], targets["x"], motor=profiles["x"].get("motor", 0)
                )
                launch_order.append("x")
                if distances["y"] > 20:
                    stagger_ms = int(900 * distances["y"] / distances["x"])
                    time.sleep(float(stagger_ms) / 1000.0)
                    commands["y"] = self.tester.motor_move_absolute(
                        profiles["y"]["board"], targets["y"], motor=profiles["y"].get("motor", 0)
                    )
                    launch_order.append("y")
        else:
            if distances["y"] > 10000:
                boosted["x"] = self.tester.motor_set_axis_param(
                    profiles["x"]["board"], 5, 1500, motor=profiles["x"].get("motor", 0)
                )
                boosted["y"] = self.tester.motor_set_axis_param(
                    profiles["y"]["board"], 5, 750, motor=profiles["y"].get("motor", 0)
                )
            if distances["y"] > 20:
                commands["y"] = self.tester.motor_move_absolute(
                    profiles["y"]["board"], targets["y"], motor=profiles["y"].get("motor", 0)
                )
                launch_order.append("y")
                if distances["x"] > 20:
                    stagger_ms = int(900 * distances["x"] / distances["y"])
                    time.sleep(float(stagger_ms) / 1000.0)
                    commands["x"] = self.tester.motor_move_absolute(
                        profiles["x"]["board"], targets["x"], motor=profiles["x"].get("motor", 0)
                    )
                    launch_order.append("x")
        waits: dict[str, Any] = {}
        for axis in launch_order:
            profile = profiles[axis]
            waits[axis] = self.tester.motor_wait_stopped(
                profile["board"],
                motor=profile.get("motor", 0),
                timeout_s=float(wait_timeout_s),
                require_seen_nonzero=True,
            )
        restored_acc = {
            "x": self.tester.motor_set_axis_param(
                profiles["x"]["board"], 5, 350, motor=profiles["x"].get("motor", 0)
            ),
            "y": self.tester.motor_set_axis_param(
                profiles["y"]["board"], 5, 400, motor=profiles["y"].get("motor", 0)
            ),
        }
        after = {axis: self._read_axis_position(axis) for axis in ("x", "y")}
        expected_axes = {axis for axis in ("x", "y") if distances[axis] > 20}
        return {
            "ok": bool(
                set(launch_order) == expected_axes
                and all(
                    isinstance(commands[axis], Mapping)
                    and commands[axis].get("ok") is True
                    and isinstance(waits[axis], Mapping)
                    and waits[axis].get("stopped") is True
                    for axis in expected_axes
                )
                and all(abs(after[axis] - targets[axis]) <= 20 for axis in ("x", "y"))
                and all(isinstance(restored_acc[axis], Mapping) and restored_acc[axis].get("ok") is True for axis in ("x", "y"))
            ),
            "oem_move_xy_order": True,
            "targets": targets,
            "distances": distances,
            "before": before,
            "after": after,
            "prepare": _json_safe(prepared),
            "boosted_acceleration": _json_safe(boosted),
            "launch_order": launch_order,
            "stagger_ms": stagger_ms,
            "commands": _json_safe(commands),
            "waits": _json_safe(waits),
            "restored_acceleration": _json_safe(restored_acc),
            "source_anchor": "ClassControlInterface.moveXY:4285-4364",
        }

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
            pseudo_z_home=int(parity.values.get("ZPseudoHome", 65000)),
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
    def _new_state() -> dict[str, Any]:
        movement_ledger = OemMovementLedger._new()
        movement_ledger["stage_order"] = list(OEM_INITIALIZE_MOTORS_STAGE_KEYS)
        return {
            "schema_version": _STATE_SCHEMA,
            "movement_ledger": movement_ledger,
            "used_approvals": {},
            "used_motion_approvals": {},
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
        upgraded.setdefault("machine_status", copy.deepcopy(defaults["machine_status"]))
        machine = upgraded.get("machine_status")
        if isinstance(machine, dict):
            for key, value in defaults["machine_status"].items():
                machine.setdefault(key, copy.deepcopy(value))
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
        if not isinstance(ledger, dict) or ledger.get("schema_version") != OemMovementLedger.schema_version:
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
            "schema_version": OemMovementLedger.schema_version,
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
            blockers.extend(self._commissioning_blockers(dict(commissioning or {}), generation=generation))
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
                OemMovementLedger._advance_after_completed_stage(ledger, spec.key)
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
                OemMovementLedger._advance_after_completed_stage(ledger, selected)
                ok = True
                blockers = []
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
            return p.oem_initialize_motion_move_absolute("z", 80000, timeout_s=min(bounded, 45.0))
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
