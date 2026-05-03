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
    "ThermalDoorAcceleration",
    "ThermalDoorCurrent",
    "ThermalDoorStallGuardThreshold",
}

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


def load_oem_config(path: str | Path) -> dict:
    p = Path(path)
    fields: dict[str, str] = {}
    try:
        tree = ET.parse(p)
        root = tree.getroot()
        for elem in root.iter():
            tag = str(elem.tag).split("}")[-1]
            text = (elem.text or "").strip()
            if text and (tag in INTERESTING_FIELDS or tag.endswith("Max") or "Motor" in tag or "ThermalDoor" in tag):
                fields[tag] = text
    except Exception as exc:
        return {"status": "error", "path": str(p), "error": str(exc), "fields": {}}
    return {"status": "loaded", "path": str(p), "fields": fields}


def find_oem_config(roots: Iterable[str | Path] | None = None) -> dict:
    searched = [str(Path(r)) for r in (roots or DEFAULT_SEARCH_ROOTS)]
    for root in searched:
        for candidate in _candidate_paths(Path(root)):
            loaded = load_oem_config(candidate)
            loaded["searched_roots"] = searched
            return loaded
    return {"status": "missing", "path": None, "searched_roots": searched, "fields": {}}
