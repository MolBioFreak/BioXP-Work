"""Pure schema helpers for the canonical serial-206 initialization state.

Persistence, admission, execution, and observation ownership live exclusively in
``Serial206OemInitializationProvider``. This module only defines immutable OEM
stage metadata and pure state construction/advancement helpers.
"""
from __future__ import annotations

from typing import Any


SERIAL206_INITIALIZE_MOTORS_LEDGER_SCHEMA = "bioxp.oem_initialize_motors_ledger.v1"

OEM_INITIALIZE_MOTORS_STAGES: tuple[dict[str, Any], ...] = (
    {
        "key": "z-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3348-3421; M01 MotorZ.axisSearchHome(speed=1791)",
    },
    {
        "key": "gripper-current-31",
        "source_anchor": "ClassControlInterface.initializeMotors:3354; M02 setGripperCurrent(31)",
        "requires_operator_observation": False,
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
        "key": "x-home-settle",
        "source_anchor": "ClassControlInterface.initializeMotors:3370; M06 sleep(20ms)",
        "requires_operator_observation": False,
    },
    {
        "key": "x-set-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3371; M07 setHome(X)",
        "requires_operator_observation": False,
    },
    {
        "key": "x-speed-1700",
        "source_anchor": "ClassControlInterface.initializeMotors:3372; M08 setSpeed(X,1700)",
        "requires_operator_observation": False,
    },
    {
        "key": "x-speed-settle",
        "source_anchor": "ClassControlInterface.initializeMotors:3373; M09 sleep(40ms)",
        "requires_operator_observation": False,
    },
    {
        "key": "x-park-6000",
        "source_anchor": "ClassControlInterface.initializeMotors:3374; M10 moveX(6000)",
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
        "requires_operator_observation": False,
    },
    {
        "key": "y-set-home",
        "source_anchor": "ClassControlInterface.initializeMotors:3389-3392; M14 setHome(Y)",
        "requires_operator_observation": False,
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
        "source_anchor": "ClassControlInterface.initializeMotors:3416; M18 system status=1 (initialization complete is not machine readiness)",
        "requires_operator_observation": False,
    },
    {
        "key": "gripper-idle-current-10",
        "source_anchor": "ClassControlInterface.initializeMotors:3417-3420; M19 setGripperCurrent(10) iff GripperVersion==1",
        "requires_operator_observation": False,
    },
)
OEM_INITIALIZE_MOTORS_STAGE_KEYS = tuple(stage["key"] for stage in OEM_INITIALIZE_MOTORS_STAGES)


def new_initialize_motors_ledger() -> dict[str, Any]:
    return {
        "schema_version": SERIAL206_INITIALIZE_MOTORS_LEDGER_SCHEMA,
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


def advance_initialize_motors_ledger(ledger: dict[str, Any], selected: str) -> None:
    index = OEM_INITIALIZE_MOTORS_STAGE_KEYS.index(selected)
    if index + 1 == len(OEM_INITIALIZE_MOTORS_STAGE_KEYS):
        ledger["expected_next_stage"] = None
        ledger["terminal_state"] = "initializeMotors_complete"
    else:
        ledger["expected_next_stage"] = OEM_INITIALIZE_MOTORS_STAGE_KEYS[index + 1]
        ledger["terminal_state"] = "awaiting_next_stage"
