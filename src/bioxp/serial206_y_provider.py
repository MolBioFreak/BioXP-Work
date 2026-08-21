"""Typed Serial-206 Y-axis provider.

The provider owns Y contract validation and authority projection. Transport remains
owned by BioXpTester. No method in this module claims independent physical proof.
"""
from __future__ import annotations

import hashlib
import json
import time
import uuid
from collections.abc import Mapping
from typing import Any, Callable


class Serial206YProvider:
    axis = "y"
    board = 4
    motor = 0
    min_steps = 0
    max_steps = 102956
    relative_min_target = 20
    relative_max_target = 102936
    normal_speed = 1800
    normal_acc = 400
    startup_home_speed = 250
    manual_home_speed = 500
    homexy_home_speed = 200
    location_home_speed = 1800
    board_test_home_speed = 200
    schema = "bioxp.serial206_y_provider.v2"

    _HOME_MODES = {
        "startup": (startup_home_speed, True),
        "diagnostic": (startup_home_speed, True),
        "manual_panel": (manual_home_speed, False),
        "homexy": (homexy_home_speed, False),
        "location_workflow": (location_home_speed, False),
        "board_test": (board_test_home_speed, False),
    }

    def __init__(
        self,
        tester: Any,
        *,
        state_store: Any | None,
        generation_provider: Callable[[], int],
        reference_store: Any | None = None,
    ) -> None:
        self.tester = tester
        self.state_store = state_store
        self.generation_provider = generation_provider
        self.reference_store = reference_store

    def profile(self, *, startup: bool = False) -> dict[str, Any]:
        reader = getattr(self.tester, "_motion_oem_axis_profile", None)
        if not callable(reader):
            raise RuntimeError("serial-206 Y profile primitive is not bound")
        raw = reader("y", startup=bool(startup))
        if not isinstance(raw, Mapping):
            raise RuntimeError("serial-206 Y profile is not a mapping")
        profile = dict(raw)
        if int(profile.get("board", -1)) != self.board or int(profile.get("motor", -1)) != self.motor:
            raise RuntimeError("serial-206 Y board/motor profile mismatch")
        if int(profile.get("axis_min_steps", -1)) != self.min_steps:
            raise RuntimeError("serial-206 Y minimum profile mismatch")
        if int(profile.get("axis_max_steps", -1)) != self.max_steps:
            raise RuntimeError("serial-206 Y selected-machine maximum is not authoritative")
        if int(profile.get("speed", -1)) != self.normal_speed:
            raise RuntimeError("serial-206 Y normal speed profile mismatch")
        if int(profile.get("acc", -1)) != self.normal_acc:
            raise RuntimeError("serial-206 Y normal acceleration profile mismatch")
        if int(profile.get("run_current", -1)) != 31:
            raise RuntimeError("serial-206 Y current profile mismatch")
        if int(profile.get("stall_guard", -1)) != 16:
            raise RuntimeError("serial-206 Y stall profile mismatch")
        if profile.get("disable_right") is not True:
            raise RuntimeError("serial-206 Y right-switch disable profile mismatch")
        profile["source_mode"] = "ClassControlInterface.initializeMotorsWithoutMotion"
        profile["selected_machine_max_authority"] = "102956"
        profile["home_speed"] = self.startup_home_speed if startup else self.manual_home_speed
        return profile

    def _profile_fingerprint(self, profile: Mapping[str, Any]) -> str:
        payload = {
            key: profile.get(key)
            for key in (
                "board", "motor", "axis_min_steps", "axis_max_steps", "speed", "acc",
                "run_current", "stall_guard", "disable_right", "home_speed",
            )
        }
        return hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        ).hexdigest()

    def _authority(self) -> dict[str, Any]:
        projection = getattr(self.state_store, "board4_authority_projection", None)
        if not callable(projection):
            return {}
        value = projection()
        return dict(value) if isinstance(value, Mapping) else {}

    def _current_authority(self, *, allow_unprepared: bool = False) -> dict[str, Any]:
        authority = self._authority()
        if not authority:
            return {"ok": True, "authority": authority}
        board = authority.get("board") if isinstance(authority.get("board"), Mapping) else {}
        axis = (authority.get("axes") or {}).get("y") if isinstance(authority.get("axes"), Mapping) else {}
        if board.get("state") != "active":
            return {"ok": False, "failure": "board4_not_active", "board": board, "axis": axis}
        epoch = board.get("active_board_epoch")
        if not allow_unprepared and (
            axis.get("prepared_board_epoch") != epoch
            or axis.get("lifecycle_state") not in {"prepared_unreferenced", "referenced_ready"}
        ):
            return {"ok": False, "failure": "y_axis_authority_not_current", "board": board, "axis": axis}
        return {"ok": True, "board": board, "axis": axis}

    def prepare(self, *, command_id: str | None = None) -> dict[str, Any]:
        command_id = command_id or f"y-prepare-{uuid.uuid4().hex}"
        profile = self.profile()
        current = self._current_authority(allow_unprepared=True)
        if not current.get("ok"):
            return {"ok": False, "axis": self.axis, "command_id": command_id, **current}
        primitive = getattr(self.tester, "motor_oem_require_no_motion_profile", None)
        raw = primitive("y") if callable(primitive) else {"ok": True, "skipped": True, "source": "profile-bound"}
        if not isinstance(raw, Mapping) or raw.get("ok") is not True:
            return {
                "ok": False,
                "axis": self.axis,
                "command_id": command_id,
                "physical_motion": False,
                "failure": "y_no_motion_profile_not_verified",
                "result": dict(raw) if isinstance(raw, Mapping) else raw,
            }
        prepared = getattr(self.state_store, "prepare_axis_authority", None)
        persisted = None
        if callable(prepared):
            persisted = prepared(
                self.axis,
                ownership_generation=int(self.generation_provider()),
                profile_fingerprint=self._profile_fingerprint(profile),
            )
            if not isinstance(persisted, Mapping) or persisted.get("ok") is not True:
                return {"ok": False, "axis": self.axis, "command_id": command_id, "failure": "y_authority_persist_failed", "result": persisted}
        return {
            "ok": True,
            "schema": self.schema,
            "axis": self.axis,
            "board": self.board,
            "motor": self.motor,
            "command_id": command_id,
            "physical_motion": False,
            "profile": profile,
            "result": dict(raw) if isinstance(raw, Mapping) else raw,
            "authority": persisted,
        }

    def _position(self) -> dict[str, Any]:
        reader = getattr(self.tester, "motor_get_position", None)
        if not callable(reader):
            return {"ok": False, "failure": "y_position_primitive_not_bound"}
        value = reader(self.board, motor=self.motor)
        return dict(value) if isinstance(value, Mapping) else {"ok": False, "failure": "y_position_result_not_mapping", "raw": value}

    @staticmethod
    def _position_value(value: Any) -> int | None:
        if isinstance(value, Mapping) and type(value.get("position")) is int:
            return int(value["position"])
        return None

    def _record_observation(
        self,
        *,
        command_id: str,
        target: int,
        observed: int | None,
    ) -> dict[str, Any] | None:
        if observed is None:
            return None
        recorder = getattr(self.state_store, "record_axis_observation", None)
        if not callable(recorder):
            return None
        value = recorder(
            self.axis,
            requested_position_steps=int(target),
            observed_position_steps=int(observed),
            receipt_id=command_id,
        )
        return dict(value) if isinstance(value, Mapping) else {"ok": False, "failure": "y_observation_result_not_mapping"}

    def move_steps(
        self,
        steps: int,
        *,
        wait_timeout_s: float = 20.0,
        command_id: str | None = None,
    ) -> dict[str, Any]:
        command_id = command_id or f"y-relative-{uuid.uuid4().hex}"
        profile = self.profile()
        authority = self._current_authority()
        if not authority.get("ok"):
            return {"ok": False, "axis": self.axis, "command_id": command_id, **authority}
        before = self._position()
        before_value = self._position_value(before)
        if before_value is None:
            return {"ok": False, "axis": self.axis, "command_id": command_id, "failure": "y_position_readback_invalid", "position_before": before}
        requested_steps = int(steps)
        target = before_value + requested_steps
        if target < self.relative_min_target or target > self.relative_max_target:
            return {
                "ok": False,
                "axis": self.axis,
                "command_id": command_id,
                "failure": "y_relative_target_out_of_range",
                "requested_steps": requested_steps,
                "position_before": before_value,
                "requested_target": target,
                "relative_min_target": self.relative_min_target,
                "relative_max_target": self.relative_max_target,
                "command_sent": False,
            }
        primitive = getattr(self.tester, "motor_y_move_relative_strict", None)
        if callable(primitive):
            result = primitive(requested_steps, timeout_s=float(wait_timeout_s))
        else:
            primitive = getattr(self.tester, "motor_move_relative", None)
            result = primitive(self.board, requested_steps, motor=self.motor) if callable(primitive) else {"ok": False, "failure": "y_relative_primitive_not_bound"}
        result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "y_relative_result_not_mapping", "raw": result}
        after = result.get("terminal_position") if isinstance(result.get("terminal_position"), Mapping) else self._position()
        after_value = self._position_value(after)
        observation = self._record_observation(command_id=command_id, target=target, observed=after_value)
        proof = result.get("proof") if isinstance(result.get("proof"), Mapping) else {}
        completion_class = str(result.get("completion_class") or ("event_128" if proof.get("addressed_event_128") else "unknown"))
        return {
            "ok": bool(result.get("ok") is True),
            "schema": self.schema,
            "axis": self.axis,
            "board": self.board,
            "motor": self.motor,
            "command_id": command_id,
            "requested_steps": requested_steps,
            "position_before": before_value,
            "requested_target": target,
            "position_after": after,
            "motor_command_delivery_count": 1,
            "motor_command_reply_valid": isinstance(result.get("ack"), Mapping),
            "motor_command_status_code": (result.get("ack") or {}).get("status") if isinstance(result.get("ack"), Mapping) else None,
            "motor_command_raw_return": result.get("source_return_code"),
            "board_wrapper_return": result.get("board_wrapper_return", result.get("raw_return")),
            "public_wrapper_return": after_value,
            "completion_class": completion_class,
            "controller_completion_verified": bool(proof.get("addressed_event_128") or result.get("completion_verified")),
            "terminal_speed_zero": bool(proof.get("speed_zero")),
            "coordinate_truth_available": after_value is not None,
            "physical_effect_verified": False,
            "discrepancy": observation,
            "result": result,
            "profile": profile,
        }

    def move_absolute(
        self,
        target_steps: int,
        *,
        wait_for_stop: bool = False,
        wait_timeout_s: float = 20.0,
        command_id: str | None = None,
    ) -> dict[str, Any]:
        command_id = command_id or f"y-absolute-{uuid.uuid4().hex}"
        profile = self.profile()
        authority = self._current_authority()
        if not authority.get("ok"):
            return {"ok": False, "axis": self.axis, "command_id": command_id, **authority}
        before = self._position()
        before_value = self._position_value(before)
        if before_value is None:
            return {"ok": False, "axis": self.axis, "command_id": command_id, "failure": "y_position_readback_invalid", "position_before": before}
        caller_target = int(target_steps)
        effective_target = max(self.min_steps, min(self.max_steps, caller_target))
        near_high_noop = bool(
            before_value <= self.max_steps
            and abs(self.max_steps - before_value) < 10
            and caller_target > before_value
        )
        if near_high_noop:
            return {
                "ok": True,
                "schema": self.schema,
                "axis": self.axis,
                "command_id": command_id,
                "state": "completed",
                "caller_requested_target": caller_target,
                "board_effective_target": before_value,
                "motor_effective_target": before_value,
                "near_high_noop": True,
                "command_sent": False,
                "completion_class": "oem_near_high_noop",
                "physical_effect_verified": False,
            }
        primitive = getattr(self.tester, "motor_oem_move_absolute", None)
        result = primitive(
            self.board,
            effective_target,
            motor=self.motor,
            wait_for_stop=bool(wait_for_stop),
            max_position=self.max_steps,
        ) if callable(primitive) else {"ok": False, "failure": "y_absolute_primitive_not_bound"}
        result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "y_absolute_result_not_mapping", "raw": result}
        state = "completed" if wait_for_stop else "issued_pending"
        after = result.get("terminal_position") if wait_for_stop and isinstance(result.get("terminal_position"), Mapping) else None
        after_value = self._position_value(after)
        observation = self._record_observation(command_id=command_id, target=effective_target, observed=after_value) if wait_for_stop else None
        proof = result.get("proof") if isinstance(result.get("proof"), Mapping) else {}
        return {
            "ok": bool(result.get("ok") is True),
            "schema": self.schema,
            "axis": self.axis,
            "board": self.board,
            "motor": self.motor,
            "command_id": command_id,
            "state": state,
            "caller_requested_target": caller_target,
            "board_effective_target": effective_target,
            "motor_effective_target": result.get("wire_position", effective_target),
            "near_high_noop": False,
            "motor_command_delivery_count": 1 if result.get("retry_ack") is None else 2,
            "motor_command_reply_valid": isinstance(result.get("ack"), Mapping),
            "motor_command_status_code": (result.get("ack") or {}).get("status") if isinstance(result.get("ack"), Mapping) else None,
            "motor_command_raw_return": result.get("source_return_code"),
            "board_wrapper_return": result.get("board_wrapper_return", result.get("raw_return")),
            "public_wrapper_return_kind": "void",
            "position_after": after,
            "completion_class": str(result.get("completion_class") or ("event_128" if proof.get("addressed_event_128") else state)),
            "controller_completion_verified": bool(proof.get("addressed_event_128") or result.get("completion_verified")),
            "terminal_speed_zero": bool(proof.get("speed_zero")),
            "coordinate_truth_available": after_value is not None,
            "physical_effect_verified": False,
            "discrepancy": observation,
            "result": result,
            "profile": profile,
        }

    @staticmethod
    def _mapping_at(value: Any, *keys: str) -> Mapping[str, Any] | None:
        current = value
        for key in keys:
            if not isinstance(current, Mapping):
                return None
            current = current.get(key)
        return current if isinstance(current, Mapping) else None

    def _home_proof(self, result: Mapping[str, Any]) -> dict[str, bool]:
        home = result.get("home") if isinstance(result.get("home"), Mapping) else result
        home_after = (
            self._mapping_at(home, "home_after")
            or self._mapping_at(home, "home_after_stop")
            or self._mapping_at(home, "home_after_sethome")
        )
        stop = self._mapping_at(home, "stop") or {}
        wait = self._mapping_at(home, "wait") or {}
        set_home = self._mapping_at(home, "set_home") or {}
        zero = self._mapping_at(home, "position_after_sethome") or self._mapping_at(home, "position_after_set_home") or {}
        return {
            "home_predicate_active": bool(home_after and home_after.get("value") == 1),
            "stop_complete": bool(stop.get("ok") is True and isinstance(stop.get("first_delivery"), Mapping) and isinstance(stop.get("second_delivery"), Mapping)),
            "speed_zero": bool(wait.get("stopped") is True and wait.get("last_speed") == 0),
            "set_home_valid": bool(set_home.get("ok") is True and isinstance(set_home.get("readback"), Mapping) and set_home["readback"].get("value") == 0),
            "zero_readback": bool(zero.get("position") == 0),
        }

    def home(self, mode: str, *, command_id: str | None = None, wait_timeout_s: float = 30.0) -> dict[str, Any]:
        mode = str(mode)
        if mode not in self._HOME_MODES:
            return {"ok": False, "axis": self.axis, "failure": "unsupported_y_home_source_mode", "source_mode": mode}
        command_id = command_id or f"y-home-{uuid.uuid4().hex}"
        speed, startup = self._HOME_MODES[mode]
        profile = self.profile(startup=startup)
        authority = self._current_authority(allow_unprepared=True)
        if not authority.get("ok"):
            return {"ok": False, "axis": self.axis, "command_id": command_id, "source_mode": mode, **authority}
        if authority.get("authority") and not authority.get("axis", {}).get("prepared_board_epoch"):
            prepared = self.prepare(command_id=f"{command_id}:prepare")
            if not prepared.get("ok"):
                return {"ok": False, "axis": self.axis, "command_id": command_id, "source_mode": mode, "failure": "y_home_prepare_failed", "prepare": prepared}
        primitive = getattr(self.tester, "motor_oem_home_axis", None)
        result = primitive(
            self.axis,
            speed=int(speed),
            startup=bool(startup),
            timeout_s=float(wait_timeout_s),
            require_switch_transition=False,
        ) if callable(primitive) else {"ok": False, "failure": "y_home_primitive_not_bound"}
        result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "y_home_result_not_mapping", "raw": result}
        proof = self._home_proof(result)
        proof_ok = all(proof.values())
        reference = None
        if proof_ok:
            publisher = getattr(self.state_store, "publish_axis_reference", None)
            if callable(publisher):
                reference = publisher(
                    self.axis,
                    position_steps=0,
                    ownership_generation=int(self.generation_provider()),
                    receipt_id=command_id,
                )
                proof_ok = bool(isinstance(reference, Mapping) and reference.get("ok") is True)
        return {
            "ok": bool(result.get("ok") is True and proof_ok),
            "schema": self.schema,
            "axis": self.axis,
            "board": self.board,
            "motor": self.motor,
            "command_id": command_id,
            "source_mode": mode,
            "source_speed": int(speed),
            "startup": bool(startup),
            "home_proof": proof,
            "reference_published": bool(proof_ok),
            "physical_effect_verified": False,
            "result": result,
            "reference": reference,
            "profile": profile,
        }

    def record_move_xy_observation(self, result: Mapping[str, Any], *, command_id: str) -> dict[str, Any]:
        targets = result.get("targets") if isinstance(result.get("targets"), Mapping) else result.get("requested")
        after = result.get("after") if isinstance(result.get("after"), Mapping) else {}
        target_value = targets.get("y") if isinstance(targets, Mapping) else None
        observed_value = after.get("y") if isinstance(after, Mapping) else None
        if isinstance(observed_value, Mapping):
            observed_value = observed_value.get("position")
        if type(target_value) is not int or type(observed_value) is not int:
            return {
                "ok": False,
                "axis": self.axis,
                "command_id": command_id,
                "failure": "move_xy_y_observation_unavailable",
                "reconciled_to_observed": False,
            }
        observation = self._record_observation(
            command_id=command_id,
            target=int(target_value),
            observed=int(observed_value),
        )
        return {
            "ok": bool(isinstance(observation, Mapping) and observation.get("ok") is True),
            "axis": self.axis,
            "command_id": command_id,
            "observed_position_steps": int(observed_value),
            "discrepancy_steps": int(observed_value) - int(target_value),
            "reconciled_to_observed": True,
            "observation": observation,
        }

    def publish_home_xy_reference(self, result: Mapping[str, Any], *, command_id: str) -> dict[str, Any]:
        child = result.get("home", {}).get("y") if isinstance(result.get("home"), Mapping) else None
        position = result.get("positions", {}).get("y") if isinstance(result.get("positions"), Mapping) else None
        proof_ok = bool(
            isinstance(child, Mapping)
            and child.get("ok") is True
            and child.get("controller_home_proof_verified") is True
            and isinstance(position, Mapping)
            and position.get("position") == 0
            and isinstance(position.get("ack"), Mapping)
            and position["ack"].get("status") == 100
        )
        if not proof_ok:
            return {
                "ok": False,
                "axis": self.axis,
                "command_id": command_id,
                "failure": "homexy_y_child_proof_incomplete",
                "reference_published": False,
            }
        authority = self._current_authority(allow_unprepared=True)
        if not authority.get("ok"):
            return {"ok": False, "axis": self.axis, "command_id": command_id, **authority, "reference_published": False}
        axis_state = authority.get("axis") if isinstance(authority.get("axis"), Mapping) else {}
        board = authority.get("board") if isinstance(authority.get("board"), Mapping) else {}
        if axis_state.get("prepared_board_epoch") != board.get("active_board_epoch"):
            prepared = getattr(self.state_store, "prepare_axis_authority", None)
            if callable(prepared):
                prepared_result = prepared(
                    self.axis,
                    ownership_generation=int(self.generation_provider()),
                    profile_fingerprint=self._profile_fingerprint(self.profile()),
                )
                if not isinstance(prepared_result, Mapping) or prepared_result.get("ok") is not True:
                    return {"ok": False, "axis": self.axis, "command_id": command_id, "failure": "homexy_y_prepare_persist_failed", "prepare": prepared_result, "reference_published": False}
        publisher = getattr(self.state_store, "publish_axis_reference", None)
        reference = publisher(
            self.axis,
            position_steps=0,
            ownership_generation=int(self.generation_provider()),
            receipt_id=command_id,
        ) if callable(publisher) else None
        published = bool(isinstance(reference, Mapping) and reference.get("ok") is True)
        return {
            "ok": published,
            "axis": self.axis,
            "command_id": command_id,
            "physical_motion": False,
            "reference_published": published,
            "reference": reference,
        }

    def set_home(self, operator_ack: str, *, command_id: str | None = None) -> dict[str, Any]:
        command_id = command_id or f"y-set-home-{uuid.uuid4().hex}"
        if operator_ack != "SET_HOME_CURRENT_POSITION":
            return {
                "ok": False,
                "axis": self.axis,
                "command_id": command_id,
                "failure": "y_set_home_ack_required",
                "physical_motion": False,
                "reference_published": False,
            }
        primitive = getattr(self.tester, "motor_set_home", None)
        result = primitive(self.board, motor=self.motor) if callable(primitive) else {"ok": False, "failure": "y_set_home_primitive_not_bound"}
        result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "y_set_home_result_not_mapping", "raw": result}
        readback = self._position()
        readback_value = self._position_value(readback)
        readback_ok = bool(readback_value == 0)
        return {
            "ok": bool(result.get("ok") is True and readback_ok),
            "schema": self.schema,
            "axis": self.axis,
            "board": self.board,
            "motor": self.motor,
            "command_id": command_id,
            "operator_ack": operator_ack,
            "physical_motion": False,
            "controller_set_home_acknowledged": bool(result.get("ok") is True),
            "zero_readback": readback_ok,
            "reference_published": False,
            "position_after": readback,
            "result": result,
        }

    def stop(self, *, command_id: str | None = None, timeout_s: float = 3.0) -> dict[str, Any]:
        command_id = command_id or f"y-stop-{uuid.uuid4().hex}"
        primitive = getattr(self.tester, "motor_oem_stop_exact", None) or getattr(self.tester, "motor_stop", None)
        result = primitive(self.board, motor=self.motor) if callable(primitive) else {"ok": False, "failure": "y_stop_primitive_not_bound"}
        result = dict(result) if isinstance(result, Mapping) else {"ok": False, "failure": "y_stop_result_not_mapping", "raw": result}
        return {
            "ok": bool(result.get("ok") is True),
            "schema": self.schema,
            "axis": self.axis,
            "board": self.board,
            "motor": self.motor,
            "command_id": command_id,
            "stop": result,
            "timeout_s": float(timeout_s),
            "physical_effect_verified": False,
        }

    def status(self) -> dict[str, Any]:
        profile = self.profile()
        status_reader = getattr(self.tester, "motor_axis_status", None)
        status = status_reader(self.board, motor=self.motor) if callable(status_reader) else {
            "position": self._position(),
        }
        return {
            "schema": self.schema,
            "axis": self.axis,
            "board": self.board,
            "motor": self.motor,
            "profile": profile,
            "status": dict(status) if isinstance(status, Mapping) else status,
            "authority": self._authority(),
            "physical_effect_verified": False,
        }

    def projection(self) -> dict[str, Any]:
        authority = self._authority()
        axis = authority.get("axes", {}).get("y") if isinstance(authority.get("axes"), Mapping) else None
        return {
            "schema": self.schema,
            "axis": self.axis,
            "board": self.board,
            "motor": self.motor,
            "profile": self.profile(),
            "authority": axis,
            "board_authority": authority.get("board"),
            "physical_effect_verified": False,
        }
