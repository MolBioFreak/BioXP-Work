
"""Machine config/calibration binding for fresh OEM parity scaffold.

This module never talks to hardware. It labels whether constants are recovered
from a machine config or are only OEM source defaults.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

SOURCE_DEFAULTS: dict[str, Any] = {
    "X_MOTOR_MAX_POSITION": 91919,
    "Y_MOTOR_MAX_POSITION": 95247,
    "Z_MOTOR_MAX_POSITION": 160000,
    "G_MOTOR_MAX_POSITION": 15000,
    "X_MOTOR_SPEED": 1700,
    "X_MOTOR_ACCELERATION": 350,
    "Y_MOTOR_SPEED": 1800,
    "Y_MOTOR_ACCELERATION": 400,
    "Z_MOTOR_SPEED": 1791,
    "Z_MOTOR_ACCELERATION": 576,
    "G_GRIPPER_V0_SPEED": 600,
    "G_GRIPPER_V0_ACCELERATION": 5,
    "G_GRIPPER_V1_SPEED": 1500,
    "G_GRIPPER_V1_ACCELERATION": 20,
    "Z_MOTOR_MAX_CURRENT_UP": 31,
    "Z_MOTOR_MAX_CURRENT_DOWN": 25,
    "G_STARTUP_HOT_CURRENT": 31,
    "G_SAFE_IDLE_CURRENT": 10,
    "TCDoorStallGuardThreshold": 6,
    "TC_DOOR_VELOCITY": 50,
    "TC_DOOR_ACCELERATION": 20,
    "TC_DOOR_MAX_CURRENT": 31,
    "GripperVersion": None,
    "SerialNumber": None,
    "CameraCalibrated": None,
    "Calibrated": None,
}

KNOWN_KEYS = set(SOURCE_DEFAULTS)


@dataclass(frozen=True)
class OemParityConfig:
    calibration_source: str
    machine_calibrated: bool
    values: dict[str, Any]
    blockers: list[str] = field(default_factory=list)
    unknown_keys: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "calibration_source": self.calibration_source,
            "machine_calibrated": self.machine_calibrated,
            "values": dict(self.values),
            "blockers": list(self.blockers),
            "unknown_keys": list(self.unknown_keys),
        }


def _attr_int(attrs: dict[str, str], *names: str) -> int | None:
    for name in names:
        if name in attrs:
            try:
                return int(float(str(attrs[name]).strip()))
            except (TypeError, ValueError):
                return None
    return None


def _attr_value(attrs: dict[str, str], *names: str) -> Any:
    for name in names:
        if name in attrs:
            return _coerce(str(attrs[name]))
    return None


def _apply_oem_attribute_shape(root: ET.Element, values: dict[str, Any]) -> None:
    axis_key_map = {
        "x_limit": "X_MOTOR_MAX_POSITION",
        "y_limit": "Y_MOTOR_MAX_POSITION",
        "z_limit": "Z_MOTOR_MAX_POSITION",
        "g_limit": "G_MOTOR_MAX_POSITION",
    }
    offset_key_map = {
        "m_Z_MOTOR_MAX_CURRENT_UP": "Z_MOTOR_MAX_CURRENT_UP",
        "m_Z_MOTOR_MAX_CURRENT_DOWN": "Z_MOTOR_MAX_CURRENT_DOWN",
        "m_TCDoorStallGuardThreshold": "TCDoorStallGuardThreshold",
        "m_TC_DOOR_VELOCITY": "TC_DOOR_VELOCITY",
        "m_TC_DOOR_ACCELERATION": "TC_DOOR_ACCELERATION",
        "m_TC_DOOR_MAX_CURRENT": "TC_DOOR_MAX_CURRENT",
    }
    for elem in root.iter():
        tag = str(elem.tag).split("}")[-1]
        low = tag.lower()
        attrs = dict(elem.attrib)
        if low in axis_key_map:
            max_steps = _attr_int(attrs, "maxSteps", "maxsteps", "MaxSteps", "max")
            if max_steps is not None:
                values[axis_key_map[low]] = max_steps
        if tag == "Config":
            gv = _attr_value(attrs, "GripperVersion")
            if gv is not None:
                values["GripperVersion"] = gv
        if tag == "Calibration":
            cal = _attr_value(attrs, "Calibrated")
            if cal is not None:
                values["Calibrated"] = cal
        if tag == "CameraInstalled":
            cam = _attr_value(attrs, "Cameracalibrated", "CameraCalibrated")
            if cam is not None:
                values["CameraCalibrated"] = cam
        if tag == "Offsets":
            for attr, key in offset_key_map.items():
                val = _attr_value(attrs, attr)
                if val is not None:
                    values[key] = val


def _coerce(text: str) -> Any:
    s = text.strip()
    if s.lower() in {"true", "false"}:
        return s.lower() == "true"
    try:
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return s


def load_oem_parity_config(config_xml: str | Path | None) -> OemParityConfig:
    values = dict(SOURCE_DEFAULTS)
    if config_xml is None:
        return OemParityConfig(
            calibration_source="source_defaults",
            machine_calibrated=False,
            values=values,
            blockers=["config.xml_not_bound", "source_defaults_not_machine_calibration"],
        )
    path = Path(config_xml)
    root = ET.parse(path).getroot()
    unknown: list[str] = []
    _apply_oem_attribute_shape(root, values)
    for elem in root.iter():
        if elem is root or elem.text is None or not elem.text.strip():
            continue
        key = str(elem.tag).split("}")[-1]
        if key in KNOWN_KEYS:
            values[key] = _coerce(elem.text)
        else:
            unknown.append(key)
    return OemParityConfig(
        calibration_source=str(path),
        machine_calibrated=True,
        values=values,
        blockers=[],
        unknown_keys=sorted(set(unknown)),
    )
