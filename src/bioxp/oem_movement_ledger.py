"""Durable, robot-owned admission ledger for OEM initializeMotors stages.

This module has no transport dependency.  It can admit and record a source stage,
but cannot command hardware.  Hardware execution remains in the robot runtime
worker after admission succeeds.
"""
from __future__ import annotations

import copy
import threading
from typing import Any, Mapping


OEM_INITIALIZE_MOTORS_STAGES: tuple[dict[str, Any], ...] = (
    {
        "key": "z-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3348-3421; M01 MotorZ.axisSearchHome(speed=1791)",
    },
    {
        "key": "gripper-current-31",
        "source_anchor": "ClassControlInterface.initializeMotors:3354; M02 setGripperCurrent(31)",
    },
    {
        "key": "gripper-clear-10000",
        "source_anchor": "ClassControlInterface.initializeMotors:3355; M03 MotorGrip.moveSteps(+10000,wait=true)",
    },
    {
        "key": "gripper-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3348-3421; M04 MotorGrip.axisSearchHome(speed=200 for GripperVersion=1)",
    },
    {
        "key": "x-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3348-3421; M05 MotorX.axisSearchHome(speed=250)",
    },
    {
        "key": "x-park-6000",
        "source_anchor": "ClassControlInterface.initializeMotors:3348-3421; M06 sleep(20ms); M07 setHome(X); M08 setSpeed(X,1700); M09 sleep(40ms); M10 moveX(6000)",
    },
    {
        "key": "y-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3348-3421; M11 MotorY.axisSearchHome(speed=250)",
    },
    {
        "key": "door-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3380-3383; M12 ThermalDoor.doorSearchHome",
    },
    {
        "key": "door-closed-predicate",
        "source_anchor": "ClassControlInterface.initializeMotors:3384-3387; M13 SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated → openThermalDoor → throw",
    },
    {
        "key": "y-set-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3389-3392; M14 setHome(Y)",
    },
    {
        "key": "ui-zero-calibrated",
        "source_anchor": "ClassControlInterface.initializeMotors:3393-3413; M15 calibrated UI X/Y/Z=0 with duplicate Z write",
        "requires_operator_observation": False,
    },
    {
        "key": "chiller-oc-cool-rate",
        "source_anchor": "ClassControlInterface.initializeMotors:3414; M16 setChillerCoolRate(OC); usb_driver.py:4192-4194 GP8 bank=1 value=-25",
        "requires_operator_observation": False,
    },
    {
        "key": "chiller-rc-cool-rate",
        "source_anchor": "ClassControlInterface.initializeMotors:3415; M17 setChillerCoolRate(RC); usb_driver.py:4192-4194 GP8 bank=0 value=-25",
        "requires_operator_observation": False,
    },
    {
        "key": "system-status-initialized",
        "source_anchor": "ClassControlInterface.initializeMotors:3416; M18 system status=1 and ready=true",
        "requires_operator_observation": False,
    },
    {
        "key": "gripper-idle-current-10",
        "source_anchor": "ClassControlInterface.initializeMotors:3417-3420; M19 setGripperCurrent(10) iff GripperVersion==1",
        "requires_operator_observation": False,
    },
)
OEM_INITIALIZE_MOTORS_STAGE_KEYS = tuple(stage["key"] for stage in OEM_INITIALIZE_MOTORS_STAGES)


class OemMovementLedger:
    """One source-order ledger; no UI/client may choose the next physical stage."""

    schema_version = "bioxp.oem_initialize_motors_ledger.v1"

    def __init__(self, store: Any | None = None) -> None:
        self._store = store
        self._lock = threading.RLock()
        self._memory: dict[str, Any] | None = None

    @staticmethod
    def _new() -> dict[str, Any]:
        return {
            "schema_version": OemMovementLedger.schema_version,
            "source_sequence": "ClassControlInterface.initializeMotors:3348-3421",
            "expected_next_stage": OEM_INITIALIZE_MOTORS_STAGE_KEYS[0],
            "terminal_state": "not_started",
            "robot_state": {},
            "stages": {
                stage["key"]: {
                    "stage": stage["key"],
                    "source_anchor": stage["source_anchor"],
                    "requires_operator_observation": bool(stage.get("requires_operator_observation", True)),
                    "state": "pending",
                    "command_id": None,
                    "result": None,
                    "artifact_path": None,
                    "observation": None,
                }
                for stage in OEM_INITIALIZE_MOTORS_STAGES
            },
        }

    def _load(self) -> dict[str, Any]:
        if self._store is not None:
            stored = self._store.read_oem_movement_ledger()
            if stored is not None:
                return self._normalize(copy.deepcopy(stored))
        if self._memory is None:
            self._memory = self._new()
        return self._normalize(copy.deepcopy(self._memory))

    @staticmethod
    def _normalize(ledger: dict[str, Any]) -> dict[str, Any]:
        """Upgrade v1 ledger rows without silently inventing stage results."""
        ledger.setdefault("robot_state", {})
        rows = ledger.get("stages")
        if not isinstance(rows, dict):
            return ledger
        for stage in OEM_INITIALIZE_MOTORS_STAGES:
            row = rows.get(stage["key"])
            if isinstance(row, dict):
                row.setdefault("requires_operator_observation", bool(stage.get("requires_operator_observation", True)))
        return ledger

    def _save(self, ledger: Mapping[str, Any]) -> dict[str, Any]:
        payload = copy.deepcopy(dict(ledger))
        if self._store is not None:
            self._store.write_oem_movement_ledger(payload)
        self._memory = copy.deepcopy(payload)
        return payload

    def projection(self) -> dict[str, Any]:
        with self._lock:
            return self._load()

    def admit(self, *, stage: str, command_id: str) -> dict[str, Any]:
        with self._lock:
            ledger = self._load()
            selected = str(stage).strip().lower()
            expected = ledger["expected_next_stage"]
            if selected != expected:
                return {
                    "ok": False,
                    "blocker": f"oem_initializeMotors_expected_next_stage_{expected}",
                    "expected_next_stage": expected,
                    "ledger": self._save(ledger),
                }
            row = ledger["stages"][selected]
            if row["state"] != "pending":
                return {
                    "ok": False,
                    "blocker": f"oem_initializeMotors_stage_{selected}_not_pending",
                    "expected_next_stage": expected,
                    "ledger": ledger,
                }
            row.update({"state": "admitted", "command_id": str(command_id), "result": None, "artifact_path": None})
            ledger["terminal_state"] = "running"
            return {"ok": True, "ledger": self._save(ledger)}

    def record_result(self, *, stage: str, command_id: str, result: Mapping[str, Any], artifact_path: str | None) -> dict[str, Any]:
        with self._lock:
            ledger = self._load()
            selected = str(stage).strip().lower()
            row = ledger["stages"].get(selected)
            if row is None or row["state"] != "admitted" or row["command_id"] != str(command_id):
                raise ValueError(f"stale or unadmitted OEM movement result for {selected}")
            row["result"] = copy.deepcopy(dict(result))
            row["artifact_path"] = artifact_path
            if bool(result.get("ok")):
                if selected == "system-status-initialized":
                    durable = result.get("durable_robot_state")
                    if not (
                        isinstance(durable, Mapping)
                        and type(durable.get("system_status")) is int
                        and durable.get("system_status") == 1
                        and durable.get("ready") is True
                    ):
                        row["state"] = "failed"
                        ledger["terminal_state"] = "failed_closed"
                        row["result"] = {
                            **row["result"],
                            "ok": False,
                            "error": "system_status_initialized_requires_exact_durable_robot_state",
                        }
                        return self._save(ledger)
                    ledger["robot_state"] = {
                        "system_status": 1,
                        "ready": True,
                        "source_anchor": row["source_anchor"],
                    }
                if row["requires_operator_observation"]:
                    row["state"] = "acknowledged"
                    ledger["terminal_state"] = "awaiting_operator_observation"
                else:
                    row["state"] = "completed"
                    self._advance_after_completed_stage(ledger, selected)
            else:
                row["state"] = "failed"
                ledger["terminal_state"] = "failed_closed"
            return self._save(ledger)

    @staticmethod
    def _advance_after_completed_stage(ledger: dict[str, Any], selected: str) -> None:
        index = OEM_INITIALIZE_MOTORS_STAGE_KEYS.index(selected)
        if index + 1 == len(OEM_INITIALIZE_MOTORS_STAGE_KEYS):
            ledger["expected_next_stage"] = None
            ledger["terminal_state"] = "initializeMotors_complete"
        else:
            ledger["expected_next_stage"] = OEM_INITIALIZE_MOTORS_STAGE_KEYS[index + 1]
            ledger["terminal_state"] = "awaiting_next_stage"

    def record_observation(self, *, stage: str, observed_pass: bool, note: str | None, command_id: str) -> dict[str, Any]:
        with self._lock:
            ledger = self._load()
            selected = str(stage).strip().lower()
            expected = ledger["expected_next_stage"]
            if selected != expected:
                return {
                    "ok": False,
                    "blocker": f"oem_initializeMotors_expected_next_stage_{expected}",
                    "expected_next_stage": expected,
                    "ledger": self._save(ledger),
                }
            row = ledger["stages"][selected]
            if not row["requires_operator_observation"]:
                return {
                    "ok": False,
                    "blocker": f"oem_initializeMotors_stage_{selected}_does_not_accept_operator_observation",
                    "expected_next_stage": expected,
                    "ledger": ledger,
                }
            if row["state"] != "acknowledged":
                return {
                    "ok": False,
                    "blocker": f"oem_initializeMotors_stage_{selected}_requires_acknowledged_result_before_observation",
                    "expected_next_stage": expected,
                    "ledger": ledger,
                }
            row["observation"] = {"command_id": str(command_id), "observed_pass": bool(observed_pass), "note": note}
            if not bool(observed_pass):
                row["state"] = "failed"
                ledger["terminal_state"] = "failed_closed"
                return {"ok": False, "blocker": f"oem_initializeMotors_operator_observation_failed_{selected}", "ledger": self._save(ledger)}
            row["state"] = "operator_observed"
            self._advance_after_completed_stage(ledger, selected)
            return {"ok": True, "ledger": self._save(ledger)}
