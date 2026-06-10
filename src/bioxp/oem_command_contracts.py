
"""Source-derived OEM command contracts for BioXP motor/readback primitives.

These contracts are extracted from the decompiled OEM SSD backup and are meant
to prevent the fresh runtime from inventing command frames or reply semantics.
They are data-only; importing this module must not open USB or command motion.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class OemCommandContract:
    name: str
    source_file: str
    source_lines: str
    command_template: list[Any]
    success_reply_byte_1: int = 100
    reply_value_byte: int | None = None
    active_reply_value: int | None = None
    motion_commanded: bool = False
    null_reply_behavior: str = "log_and_return_0"
    error_reply_behavior: str = "log_error_return_nonzero"
    notes: tuple[str, ...] = ()


_CONTRACTS: tuple[OemCommandContract, ...] = (
    OemCommandContract(
        name="ClassMotor.MoveLeft",
        source_file="decompiled_src_can/ClassCanLib/ClassMotor.cs",
        source_lines="74-115",
        command_template=[2, 0, "axis", "speed_be[3]", "speed_be[2]", "speed_be[1]", "speed_be[0]"],
        motion_commanded=True,
        notes=("speed is encoded little-endian then emitted high byte first in the 7-byte frame",),
    ),
    OemCommandContract(
        name="ClassMotor.MoveRight",
        source_file="decompiled_src_can/ClassCanLib/ClassMotor.cs",
        source_lines="118-158",
        command_template=[1, 0, "axis", "speed_be[3]", "speed_be[2]", "speed_be[1]", "speed_be[0]"],
        motion_commanded=True,
    ),
    OemCommandContract(
        name="ClassMotor.StopMotor",
        source_file="decompiled_src_can/ClassCanLib/ClassMotor.cs",
        source_lines="161-168",
        command_template=[3, 0, "axis", 0, 0, 0, 0],
        motion_commanded=False,
        notes=("OEM sends StopMotor transmit twice",),
    ),
    OemCommandContract(
        name="ClassMotor.setHome",
        source_file="decompiled_src_can/ClassCanLib/ClassMotor.cs",
        source_lines="492-516",
        command_template=[5, 1, "axis", 0, 0, 0, 0],
        motion_commanded=False,
        notes=("also mutates local m_currentPosition=0, m_bHome=true, m_posClean=true",),
    ),
    OemCommandContract(
        name="ClassMotor.queryActualPosition",
        source_file="decompiled_src_can/ClassCanLib/ClassMotor.cs",
        source_lines="565-590",
        command_template=[6, 1, "axis", 0, 0, 0, 0],
        motion_commanded=False,
        reply_value_byte=3,
        notes=("reply bytes 3..6 converted from big-endian to int32",),
    ),
    OemCommandContract(
        name="ClassMotor.queryMotorSpeed",
        source_file="decompiled_src_can/ClassCanLib/ClassMotor.cs",
        source_lines="593-604+",
        command_template=[6, 3, "axis", 0, 0, 0, 0],
        motion_commanded=False,
        reply_value_byte=3,
        notes=("null reply is retried once in OEM",),
    ),
    OemCommandContract(
        name="ClassMotor.queryLeftSwitchStatus",
        source_file="decompiled_src_can/ClassCanLib/ClassMotor.cs",
        source_lines="641-663",
        command_template=[6, 9, "axis", 0, 0, 0, 0],
        motion_commanded=False,
        reply_value_byte=6,
        active_reply_value=1,
        notes=("OEM method returns 0 when reply[6] == 1, otherwise default 1",),
    ),
    OemCommandContract(
        name="ClassMotor.queryRightSwitchStatus",
        source_file="decompiled_src_can/ClassCanLib/ClassMotor.cs",
        source_lines="666-688",
        command_template=[6, 10, "axis", 0, 0, 0, 0],
        motion_commanded=False,
        reply_value_byte=6,
        active_reply_value=1,
        notes=("OEM method returns 0 when reply[6] == 1, otherwise default 1",),
    ),
    OemCommandContract(
        name="ClassHeadBoard.goHome",
        source_file="decompiled_src_can/ClassCanLib/ClassHeadBoard.cs",
        source_lines="60-119",
        command_template=["if rehome: moveToAbs(axis,10000)", "MoveLeft(axis,speed)", "wait stop <=30s", "poll queryHome", "stopMotor", "setHome"],
        motion_commanded=True,
        notes=("throws on lost 24V; returns 1 if uninitialized; gripper timeout recovery branch differs",),
    ),
)


def command_contracts() -> tuple[OemCommandContract, ...]:
    return _CONTRACTS


def get_command_contract(name: str) -> OemCommandContract:
    for contract in _CONTRACTS:
        if contract.name == name:
            return contract
    raise KeyError(name)
