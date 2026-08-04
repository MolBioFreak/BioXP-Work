"""Canonical OEM startup and public operation-state authority.

This module owns state only.  Hardware mutation is supplied by explicit POST or
worker callbacks; importing or projecting this object can never open hardware.
"""
from __future__ import annotations

import copy
import threading
import time
import uuid
from datetime import datetime, timezone
from typing import Any, Callable, Mapping


PUBLIC_OPERATION_STATES = frozenset({"waiting", "running", "paused", "stopped", "emergency", "error"})
STARTUP_STAGES = ("constructor_pipette_stage", "initialization_without_motion", "initial_check")
_PREDECESSOR = {
    "constructor_pipette_stage": None,
    "initialization_without_motion": "constructor_pipette_stage",
    "initial_check": "initialization_without_motion",
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _stage_template(name: str) -> dict[str, Any]:
    return {
        "name": name,
        "state": "not_run" if _PREDECESSOR[name] is None else "blocked",
        "prerequisite": _PREDECESSOR[name],
        "repeatable": name == "initial_check",
        "attempt_id": None,
        "attempt_count": 0,
        "started_at": None,
        "completed_at": None,
        "evidence": None,
        "history": [],
        "error": None,
    }


class LifecycleStateError(RuntimeError):
    pass


class CanonicalLifecycleOwner:
    """Single owner for startup stages, door/POST events and public state."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._stage_lock = threading.Lock()
        self._revision = 0
        self._operation_state = "stopped"
        self._operation_reason = "process_created"
        self._start_mode: str | None = None
        self._board_test_mode = False
        self._check_camera = False
        self._can_ready: bool | None = None
        self._door: dict[str, Any] = {
            "door_closed": None,
            "latch_closed": None,
            "source": None,
            "observed_at": None,
        }
        self._camera_evidence: dict[str, Any] | None = None
        self._stages = {name: _stage_template(name) for name in STARTUP_STAGES}
        self._updated_at = _utc_now()

    def bind_configuration(self, *, start_mode: str, board_test_mode: bool, check_camera: bool) -> None:
        with self._lock:
            self._start_mode = str(start_mode)
            self._board_test_mode = bool(board_test_mode)
            self._check_camera = bool(check_camera)
            self._touch()

    def _touch(self) -> None:
        self._revision += 1
        self._updated_at = _utc_now()

    def _set_operation(self, state: str, reason: str) -> None:
        if state not in PUBLIC_OPERATION_STATES:
            raise LifecycleStateError(f"invalid public operation state: {state}")
        self._operation_state = state
        self._operation_reason = str(reason)
        self._touch()

    def transition(self, state: str, *, reason: str, evidence: Mapping[str, Any] | None = None) -> dict[str, Any]:
        with self._lock:
            self._set_operation(state, reason)
            if evidence and ("door_closed" in evidence or "latch_closed" in evidence):
                self._record_door_locked(
                    door_closed=evidence.get("door_closed"),
                    latch_closed=evidence.get("latch_closed"),
                    source=str(evidence.get("source") or reason),
                )
            return self.projection()

    def _record_door_locked(self, *, door_closed: Any, latch_closed: Any, source: str) -> None:
        if door_closed is not None:
            self._door["door_closed"] = bool(door_closed)
        if latch_closed is not None:
            self._door["latch_closed"] = bool(latch_closed)
        self._door["source"] = str(source)
        self._door["observed_at"] = _utc_now()
        self._touch()

    def record_door_event(self, *, door_closed: bool | None, latch_closed: bool | None, source: str) -> dict[str, Any]:
        with self._lock:
            self._record_door_locked(door_closed=door_closed, latch_closed=latch_closed, source=source)
            if door_closed is False:
                self._set_operation("waiting", "door_open")
            elif door_closed is True and latch_closed is True and self._operation_state == "waiting":
                self._set_operation("stopped", "door_closed_and_latched")
            return self.projection()

    def record_camera_evidence(self, evidence: Mapping[str, Any] | None) -> None:
        with self._lock:
            self._camera_evidence = None if evidence is None else copy.deepcopy(dict(evidence))
            self._touch()

    def transport_changed(self, can_ready: bool | None, *, reason: str) -> None:
        with self._lock:
            self._can_ready = None if can_ready is None else bool(can_ready)
            for name in STARTUP_STAGES:
                self._stages[name] = _stage_template(name)
            self._set_operation("stopped", reason)

    def _begin_stage(self, name: str) -> str:
        with self._lock:
            row = self._stages[name]
            if name == "constructor_pipette_stage" and self._can_ready is not True:
                raise LifecycleStateError("constructor_pipette_stage requires CAN_READY=true ownership evidence")
            predecessor = row["prerequisite"]
            if predecessor is not None and self._stages[predecessor]["state"] != "passed":
                raise LifecycleStateError(
                    f"{name} requires passed predecessor evidence from {predecessor}"
                )
            if row["state"] == "passed" and name != "initial_check":
                raise LifecycleStateError(f"{name} already passed in this ownership epoch")
            if name == "initial_check" and self._operation_state in {"running", "paused", "emergency"}:
                raise LifecycleStateError(
                    f"initial_check cannot run during active operation state {self._operation_state}"
                )
            if name == "initial_check" and row.get("evidence") is not None:
                row.setdefault("history", []).append({
                    "attempt_id": row.get("attempt_id"),
                    "started_at": row.get("started_at"),
                    "completed_at": row.get("completed_at"),
                    "state": row.get("state"),
                    "evidence": copy.deepcopy(row.get("evidence")),
                    "error": row.get("error"),
                })
            attempt_id = uuid.uuid4().hex
            row.update({
                "state": "running",
                "attempt_id": attempt_id,
                "attempt_count": int(row.get("attempt_count") or 0) + 1,
                "started_at": _utc_now(),
                "completed_at": None,
                "evidence": None,
                "error": None,
            })
            self._set_operation("running", name)
            return attempt_id

    def _finish_stage(self, name: str, attempt_id: str, result: Mapping[str, Any]) -> dict[str, Any]:
        evidence = copy.deepcopy(dict(result))
        ok = bool(evidence.get("ok"))
        with self._lock:
            row = self._stages[name]
            if row["attempt_id"] != attempt_id or row["state"] != "running":
                raise LifecycleStateError(f"stale completion evidence for {name}")
            row.update({
                "state": "passed" if ok else "failed",
                "completed_at": _utc_now(),
                "evidence": evidence,
                "error": None if ok else str(evidence.get("error") or evidence.get("outcome") or f"{name}_failed"),
            })
            successor_index = STARTUP_STAGES.index(name) + 1
            if ok and successor_index < len(STARTUP_STAGES):
                successor = self._stages[STARTUP_STAGES[successor_index]]
                if successor["state"] == "blocked":
                    successor["state"] = "not_run"
            self._set_operation("stopped" if ok else "error", f"{name}_{'passed' if ok else 'failed'}")
            return self.projection()

    def run_stage(self, name: str, action: Callable[[], Mapping[str, Any]]) -> dict[str, Any]:
        if name not in STARTUP_STAGES:
            raise LifecycleStateError(f"unknown startup stage: {name}")
        with self._stage_lock:
            attempt_id = self._begin_stage(name)
            try:
                result = action()
                if not isinstance(result, Mapping):
                    raise LifecycleStateError(f"{name} returned non-mapping evidence")
            except Exception as exc:
                result = {"ok": False, "error": str(exc), "exception_type": type(exc).__name__}
            return self._finish_stage(name, attempt_id, result)

    def run_initial_check(
        self,
        hardware: Any,
        *,
        can_ready: Callable[[], bool | None],
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ) -> dict[str, Any]:
        """Execute exact commissioned initialCheck semantics behind one stage gate."""

        def action() -> Mapping[str, Any]:
            started = clock()
            trace: list[dict[str, Any]] = []
            sleeps: list[int] = []

            def fail(error: str, **evidence: Any) -> dict[str, Any]:
                return {
                    "ok": False,
                    "error": error,
                    "can_ready_attempts": len([value for value in sleeps if value == 200]),
                    "sleep_count": len(sleeps),
                    "sleeps_ms": list(sleeps),
                    "elapsed_ms": int(round((clock() - started) * 1000.0)),
                    "trace": list(trace),
                    **evidence,
                }

            num = 0
            while can_ready() is not True:
                sleep(0.200)
                sleeps.append(200)
                trace.append({"step": "CAN_READY_wait", "counter_before_test": num, "sleep_ms": 200})
                if num > 10:
                    return {
                        "ok": False,
                        "error": "CAN_READY_timeout",
                        "can_ready_attempts": len(sleeps),
                        "sleep_count": len(sleeps),
                        "sleeps_ms": sleeps,
                        "elapsed_ms": int(round((clock() - started) * 1000.0)),
                        "trace": trace,
                    }
                num += 1

            if self._board_test_mode:
                # Exact ControlLib.initialCheck BoardTestMode branch:
                # result=true; activateBoard().  It does not run LED, door/latch,
                # or 24 V sampling and it does not synthesize a voltage value.
                activate = hardware.activate_boards()
                trace.append({"step": "BoardTestMode.activateBoard", "result": activate})
                can_waits = len([value for value in sleeps if value == 200])
                return {
                    "ok": _result_ok(activate),
                    "board_test_mode": True,
                    "can_ready_attempts": can_waits,
                    "sleep_count": len(sleeps),
                    "sleeps_ms": list(sleeps),
                    "power": None,
                    "led_write_performed": False,
                    "latch_write_performed": False,
                    "live_voltage_sample_performed": False,
                    "activate_boards": activate,
                    "elapsed_ms": int(round((clock() - started) * 1000.0)),
                    "trace": trace,
                }

            led = hardware.set_led_rgb(255, 255, 255)
            trace.append({"step": "LED_white", "rgb": [255, 255, 255], "result": led})
            if not _result_ok(led):
                return fail("LED_white_failed", led=led)
            sleep(0.050)
            sleeps.append(50)
            trace.append({"step": "LED_white_wait", "sleep_ms": 50})
            door = self._check_door_status(hardware, sleep=sleep, sleeps=sleeps, trace=trace)
            if not door["ok"]:
                return {
                    "ok": False,
                    "error": door.get("error") or "checkDoorStatus_failed",
                    "can_ready_attempts": len([value for value in sleeps if value == 200]),
                    "sleep_count": len(sleeps),
                    "sleeps_ms": sleeps,
                    "door_latch": door,
                    "elapsed_ms": int(round((clock() - started) * 1000.0)),
                    "trace": trace,
                }
            deactivate = hardware.deactivate_boards()
            trace.append({"step": "deactivate_boards", "result": deactivate})
            if not _result_ok(deactivate):
                return fail(
                    "deactivate_boards_failed",
                    door_latch=door,
                    deactivate_boards=deactivate,
                )
            activate = hardware.activate_boards()
            trace.append({"step": "activate_boards", "result": activate})
            if not _result_ok(activate):
                return fail(
                    "activate_boards_failed",
                    door_latch=door,
                    deactivate_boards=deactivate,
                    activate_boards=activate,
                )
            begin_generation = getattr(hardware, "oem_begin_board_lifecycle_generation", None)
            lifecycle_generation = (
                begin_generation(deactivation=deactivate, activation=activate)
                if callable(begin_generation)
                else {"ok": True, "legacy_harness_without_generation_binding": True}
            )
            trace.append({"step": "board_lifecycle_generation", "result": lifecycle_generation})
            if not _result_ok(lifecycle_generation):
                return fail(
                    "board_lifecycle_generation_failed",
                    door_latch=door,
                    deactivate_boards=deactivate,
                    activate_boards=activate,
                    board_lifecycle_generation=lifecycle_generation,
                )
            ok = True
            return {
                "ok": ok,
                "error": None if ok else "initialCheck_side_effect_failed",
                "board_test_mode": False,
                "can_ready_attempts": len([value for value in sleeps if value == 200]),
                "sleep_count": len(sleeps),
                "sleeps_ms": sleeps,
                "door_latch": door,
                "deactivate_boards": deactivate,
                "activate_boards": activate,
                "board_lifecycle_generation": lifecycle_generation,
                "elapsed_ms": int(round((clock() - started) * 1000.0)),
                "trace": trace,
            }

        return self.run_stage("initial_check", action)

    def initialize_system_camera_dependency(self) -> dict[str, Any]:
        """Evaluate the OEM camera gate at its initializeSystem boundary."""
        result = self._camera_dependency()
        return {
            **result,
            "stage": "initializeSystem_after_initializeMotion_before_inspectCover",
            "source_anchor": "BioXPMainWindow.initializeSystem lines 1172-1181",
        }

    def _camera_dependency(self) -> dict[str, Any]:
        if not self._check_camera:
            return {"required": False, "ok": True, "source": "OperationParameters.CheckCamera"}
        with self._lock:
            evidence = copy.deepcopy(self._camera_evidence)
        ok = bool(
            isinstance(evidence, dict)
            and evidence.get("available") is True
            and evidence.get("ok", True) is True
            and (evidence.get("probe_id") or evidence.get("session_id"))
        )
        return {
            "required": True,
            "ok": ok,
            "source": "explicit POST /camera/probe or POST /camera/stream/start evidence",
            "evidence": evidence,
            "lazy_camera_open_performed": False,
            "error": None if ok else "explicit_camera_probe_or_session_evidence_missing_or_failed",
        }

    def _check_door_status(
        self,
        hardware: Any,
        *,
        sleep: Callable[[float], None],
        sleeps: list[int],
        trace: list[dict[str, Any]],
    ) -> dict[str, Any]:
        sleep(0.500)
        sleeps.append(500)
        trace.append({"step": "checkDoorStatus_wait", "sleep_ms": 500})
        door = hardware.query_door()
        trace.append({"step": "query_door", "result": door})
        if not _result_ok(door):
            return {"ok": False, "error": "query_door_failed", "door": door}
        latch = hardware.query_latch()
        trace.append({"step": "query_latch", "result": latch})
        if not _result_ok(latch):
            return {"ok": False, "error": "query_latch_failed", "door": door, "latch": latch}
        latch_value = _observation_value(latch)
        latch_action = None
        door_requery = None
        latch_requery = None
        if latch_value == 1:
            latch_action = hardware.set_solenoid(1)
            trace.append({"step": "set_solenoid", "value": 1, "result": latch_action})
            if not _result_ok(latch_action):
                return {
                    "ok": False,
                    "error": "set_solenoid_latch_failed",
                    "door": door,
                    "latch": latch,
                    "latch_action": latch_action,
                }
            sleep(0.800)
            sleeps.append(800)
            trace.append({"step": "latch_wait", "sleep_ms": 800})
            door_requery = hardware.query_door()
            trace.append({"step": "requery_door", "result": door_requery})
            if not _result_ok(door_requery):
                return {
                    "ok": False,
                    "error": "requery_door_failed",
                    "door": door,
                    "latch": latch,
                    "latch_action": latch_action,
                    "door_requery": door_requery,
                }
            latch_requery = hardware.query_latch()
            trace.append({"step": "requery_latch", "result": latch_requery})
            if not _result_ok(latch_requery):
                return {
                    "ok": False,
                    "error": "requery_latch_failed",
                    "door": door_requery,
                    "latch": latch,
                    "latch_action": latch_action,
                    "door_requery": door_requery,
                    "latch_requery": latch_requery,
                }
            door = door_requery
            latch_value = _observation_value(latch_requery)
        voltage = hardware.query_voltage()
        power = self.voltage_observation_from_result(voltage)
        trace.append({"step": "query_24V", "result": voltage, "observation": power})
        if not _result_ok(voltage):
            return {
                "ok": False,
                "error": "query_24V_failed",
                "door": door,
                "latch": latch,
                "door_requery": door_requery,
                "latch_requery": latch_requery,
                "latch_action": latch_action,
                "power": power,
            }
        door_value = _observation_value(door)
        door_closed = None if door_value is None else bool(door_value)
        latch_closed = None if latch_value is None else bool(latch_value)
        with self._lock:
            self._record_door_locked(door_closed=door_closed, latch_closed=latch_closed, source="initialCheck.checkDoorStatus")
        release = None
        if power["oem_no24v"] is True:
            release = hardware.set_solenoid(0)
            trace.append({"step": "set_solenoid", "value": 0, "result": release})
            if not _result_ok(release):
                return {
                    "ok": False,
                    "error": "set_solenoid_release_failed",
                    "door": door,
                    "latch": latch,
                    "door_requery": door_requery,
                    "latch_requery": latch_requery,
                    "latch_action": latch_action,
                    "low_voltage_release": release,
                    "power": power,
                }
            with self._lock:
                self._record_door_locked(door_closed=False, latch_closed=latch_closed, source="initialCheck.low_24V")
            sleep(0.300)
            sleeps.append(300)
            trace.append({"step": "low_24V_wait", "sleep_ms": 300})
        # OEM checkDoorStatus returns true exactly on scalar zero.  Linux adds
        # the separate fail-closed reply-validity gate without changing that
        # scalar branch: null also maps to zero in the trace but cannot pass.
        ok = bool(power["oem_no24v"] is False and power["safety_valid"] is True)
        if latch_action is not None:
            ok = bool(ok and _result_ok(latch_action))
        return {
            "ok": ok,
            "error": None if ok else "OEM_24V_scalar_or_reply_validity_gate_failed",
            "door": door,
            "latch": latch,
            "door_requery": door_requery,
            "latch_requery": latch_requery,
            "door_closed": door_closed,
            "latch_closed": latch_closed,
            "latch_action": latch_action,
            "low_voltage_release": release,
            "power": power,
        }

    @staticmethod
    def voltage_observation(
        *,
        payload_raw: int | None,
        oem_status: int | None,
        reply_present: bool,
        transport_outcome: str,
        provenance: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        # Exact ClassIOControl.query24VSensor contract: null -> scalar 0;
        # non-status-100 -> scalar 1; status 100 -> payload byte 6.
        status = None if oem_status is None else int(oem_status)
        valid = bool(reply_present and status == 100 and payload_raw is not None)
        if not reply_present:
            scalar = 0
            scalar_source = "OEM_null_reply_fallback"
        elif status != 100:
            scalar = 1
            scalar_source = "OEM_non_success_status_default"
        elif payload_raw is not None:
            scalar = int(payload_raw)
            scalar_source = "OEM_status_100_payload_byte_6"
        else:
            # This shape is outside the OEM method's valid byte-array contract.
            # Preserve it as malformed/unknown rather than inventing a scalar.
            scalar = None
            scalar_source = "malformed_status_100_missing_payload"
        no24v = None if scalar is None else bool(scalar != 0)
        return {
            "payload_raw": None if payload_raw is None else int(payload_raw),
            "oem_status": status,
            "reply_present": bool(reply_present),
            "transport_outcome": str(transport_outcome),
            "oem_scalar": scalar,
            "oem_scalar_source": scalar_source,
            "oem_no24v": no24v,
            "sample_valid": valid,
            "reply_valid": valid,
            "safety_valid": bool(valid and not no24v),
            "zero_valid_sample": bool(valid and scalar == 0),
            "no_valid_sample": not valid,
            "provenance": copy.deepcopy(dict(provenance or {})),
        }

    @classmethod
    def voltage_observation_from_result(cls, result: Any) -> dict[str, Any]:
        row = result if isinstance(result, Mapping) else {}
        payload = row.get("payload_raw", row.get("value", row.get("raw")))
        reply_present = bool(row.get("reply_present", row.get("ack") is not None))
        status = row.get("oem_status")
        if status is None and isinstance(row.get("ack"), Mapping):
            status = row["ack"].get("status")
        return cls.voltage_observation(
            payload_raw=payload,
            oem_status=status,
            reply_present=reply_present,
            transport_outcome=str(row.get("transport_outcome") or ("reply" if reply_present else "no_reply")),
            provenance={"raw": copy.deepcopy(result)},
        )

    def projection(self) -> dict[str, Any]:
        with self._lock:
            return {
                "schema_version": "bioxp.canonical_lifecycle.v1",
                "revision": self._revision,
                "updated_at": self._updated_at,
                "operation_state": self._operation_state,
                "operation_reason": self._operation_reason,
                "start_mode": self._start_mode,
                "board_test_mode": self._board_test_mode,
                "check_camera": self._check_camera,
                "CAN_READY": self._can_ready,
                "door": copy.deepcopy(self._door),
                "startup": {
                    "state": _aggregate_startup_state(self._stages),
                    "active_stage": next((name for name in STARTUP_STAGES if self._stages[name]["state"] == "running"), None),
                    "stages": copy.deepcopy(self._stages),
                },
            }


def _aggregate_startup_state(stages: Mapping[str, Mapping[str, Any]]) -> str:
    states = [stages[name]["state"] for name in STARTUP_STAGES]
    if "running" in states:
        return "running"
    if "failed" in states:
        return "failed"
    if all(state == "passed" for state in states):
        return "passed"
    return "not_run"


def _observation_value(result: Any) -> Any:
    if isinstance(result, Mapping):
        for key in ("value", "raw", "state"):
            if key in result:
                return result[key]
    return result


def _result_ok(result: Any) -> bool:
    if not isinstance(result, Mapping):
        return bool(result)
    if "error" in result:
        return False
    explicit = result.get("ok")
    if isinstance(explicit, bool):
        return explicit
    ack = result.get("ack")
    if isinstance(ack, Mapping):
        return ack.get("status") == 100
    if "oem_status" in result or "reply_present" in result:
        return result.get("reply_present") is True and result.get("oem_status") == 100
    return False


lifecycle_state = CanonicalLifecycleOwner()
