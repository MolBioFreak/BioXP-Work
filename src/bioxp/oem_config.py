from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable

INTERESTING_FIELDS = {
    "StartMode",
    "GripperVersion",
    "Calibrated",
    "CameraCalibrated",
    "CameraInstalled",
    "CheckCamera",
    "XAxisMax",
    "YAxisMax",
    "ZAxisMax",
    "GAxisMax",
    "XAxisMin",
    "YAxisMin",
    "ZAxisMin",
    "GAxisMin",
    "ZMotorMaxCurrentUp",
    "ZMotorMaxCurrentDown",
    "ZMotorStallGuardThreshold",
    "ThermalDoorMaxVelocity",
    "TC_DOOR_VELOCITY",
    "TC_DOOR_ACCELERATION",
    "TC_DOOR_MAX_CURRENT",
    "ThermalDoorAcceleration",
    "ThermalDoorCurrent",
    "ThermalDoorStallGuardThreshold",
    "TCDoorStallGuardThreshold",
    "SerialNumber",
}

REQUIRED_FOR_LIVE = ["StartMode", "GripperVersion"]

DEFAULT_SEARCH_ROOTS = [
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/.deploy",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Users",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/ProgramData",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Program Files",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Program Files (x86)",
]


def _candidate_paths(root: Path) -> Iterable[Path]:
    if root.is_file() and root.name.lower() == "config.xml":
        yield root
        return
    if not root.exists():
        return
    direct = root / "config.xml"
    if direct.exists():
        yield direct
    for dirpath, _, filenames in os.walk(root):
        for fn in filenames:
            if fn.lower() == "config.xml":
                yield Path(dirpath) / fn


def _typed(value: str):
    low = value.strip().lower()
    if low in {"true", "false"}:
        return low == "true"
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


def _derived_requirements(fields: dict[str, str]) -> dict:
    def truthy(name: str) -> bool:
        val = str(fields.get(name, "")).strip().lower()
        return val in {"1", "true", "yes", "enabled"}
    return {
        "camera_check_required": truthy("CheckCamera") and truthy("CameraInstalled"),
        "camera_calibrated": truthy("CameraCalibrated"),
        "calibrated": truthy("Calibrated"),
        "start_mode": fields.get("StartMode"),
        "gripper_version": fields.get("GripperVersion"),
    }


def load_oem_config(path: str | Path) -> dict:
    p = Path(path)
    fields: dict[str, str] = {}
    try:
        tree = ET.parse(p)
        root = tree.getroot()
        for elem in root.iter():
            tag = str(elem.tag).split("}")[-1]
            text = (elem.text or "").strip()
            if text and (tag in INTERESTING_FIELDS or tag.endswith("Max") or "Motor" in tag or "ThermalDoor" in tag or tag.startswith("TC_DOOR")):
                fields[tag] = text
    except Exception as exc:
        return {"status": "error", "path": str(p), "error": str(exc), "fields": {}, "fields_typed": {}, "missing_fields": REQUIRED_FOR_LIVE, "derived_requirements": {}}
    missing = [name for name in REQUIRED_FOR_LIVE if name not in fields]
    return {
        "status": "loaded",
        "path": str(p),
        "fields": fields,
        "fields_raw": dict(fields),
        "fields_typed": {k: _typed(v) for k, v in fields.items()},
        "missing_fields": missing,
        "live_ready": not missing,
        "derived_requirements": _derived_requirements(fields),
    }


def find_oem_config(roots: Iterable[str | Path] | None = None) -> dict:
    searched = [str(Path(r)) for r in (roots or DEFAULT_SEARCH_ROOTS)]
    for root in searched:
        for candidate in _candidate_paths(Path(root)):
            loaded = load_oem_config(candidate)
            loaded["searched_roots"] = searched
            return loaded
    return {"status": "missing", "path": None, "searched_roots": searched, "fields": {}, "fields_raw": {}, "fields_typed": {}, "missing_fields": REQUIRED_FOR_LIVE, "live_ready": False, "derived_requirements": {}}
