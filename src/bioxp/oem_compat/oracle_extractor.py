from __future__ import annotations

import hashlib
import json
import re
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Mapping

from src.bioxp.domain.oem_bindings import OEM_BINDING_SCHEMA

_LOCATION_ENTRY_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*(?:=\s*(-?\d+))?\s*,?")
_POSITION_BLOCK_RE = re.compile(
    r"\{\s*\(locationID\)\s*(?P<ordinal>\d+)\s*,\s*new\s+positionStruct\s*\{(?P<body>.*?)\}\s*\}",
    re.DOTALL,
)
_FIELD_RE = re.compile(r"(?P<key>name|x|y|zLow|zHigh|zDelta|inc_factor)\s*=\s*(?P<value>\"[^\"]*\"|-?\d+)")
_CONST_RE = re.compile(r"public\s+const\s+(?:int|double)\s+(?P<name>[A-Z0-9_]+)\s*=\s*(?P<value>-?\d+(?:\.\d+)?)\s*;")


def extract_location_id_enum(path: str | Path) -> dict[int, str]:
    """Extract CommonLib.locationID enum ordinals from decompiled C# source."""

    source = Path(path).read_text(encoding="utf-8", errors="replace")
    body_match = re.search(r"enum\s+locationID\s*\{(?P<body>.*?)\}", source, re.DOTALL)
    if body_match is None:
        raise ValueError(f"Could not find locationID enum in {path}")
    ordinal = 0
    values: dict[int, str] = {}
    for raw_line in body_match.group("body").splitlines():
        line = raw_line.split("//", 1)[0].strip()
        if not line:
            continue
        match = _LOCATION_ENTRY_RE.match(line)
        if match is None:
            continue
        name = match.group(1)
        explicit = match.group(2)
        if explicit is not None:
            ordinal = int(explicit)
        values[ordinal] = name
        ordinal += 1
    if not values:
        raise ValueError(f"No locationID enum values extracted from {path}")
    return values


def extract_position_table(
    class_bioxp_settings_path: str | Path,
    location_id_by_ordinal: Mapping[int, str],
) -> dict[str, dict[str, int | str]]:
    """Extract the default ClassBioXPSettings PositionTable from decompiled C# source."""

    source = Path(class_bioxp_settings_path).read_text(encoding="utf-8", errors="replace")
    table_start = source.find("m_positionTable = new Dictionary<locationID, positionStruct>")
    if table_start < 0:
        raise ValueError(f"Could not find m_positionTable initializer in {class_bioxp_settings_path}")
    table_end = source.find(";", table_start)
    if table_end < 0:
        raise ValueError(f"Could not find end of m_positionTable initializer in {class_bioxp_settings_path}")
    table_source = source[table_start:table_end]

    entries: dict[str, dict[str, int | str]] = {}
    for block in _POSITION_BLOCK_RE.finditer(table_source):
        ordinal = int(block.group("ordinal"))
        location_name = location_id_by_ordinal.get(ordinal, f"LOCATION_{ordinal}")
        fields: dict[str, int | str] = {"ordinal": ordinal}
        for field_match in _FIELD_RE.finditer(block.group("body")):
            key = field_match.group("key")
            value = field_match.group("value")
            if value.startswith('"') and value.endswith('"'):
                fields[key] = value[1:-1]
            else:
                fields[key] = int(value)
        for required in ("x", "y", "zLow", "zDelta", "inc_factor"):
            if required not in fields:
                raise ValueError(f"PositionTable entry {location_name} missing {required}")
        if "zHigh" not in fields:
            fields["zHigh"] = int(fields["zLow"]) - int(fields["zDelta"])
        entries[location_name] = fields
    if not entries:
        raise ValueError(f"No PositionTable entries extracted from {class_bioxp_settings_path}")
    return entries


def _coerce_xml_scalar(value: str) -> int | float | str:
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def extract_calibration_reference(path: str | Path) -> dict[str, Any]:
    """Extract deploy-time calreference.xml fluid/part-number offsets."""

    root = ET.parse(path).getroot()
    if root.tag != "CalibrationReferences":
        raise ValueError(f"Expected CalibrationReferences root in {path}, got {root.tag}")
    part_numbers: dict[str, dict[str, int | float | str]] = {}
    fluid_reference: dict[str, int | float | str] = {}
    for child in root:
        if child.tag.startswith("PartNumber_"):
            params = child.find("Parameters")
            if params is not None:
                part_numbers[child.tag] = {
                    key: _coerce_xml_scalar(value)
                    for key, value in params.attrib.items()
                }
        elif child.tag == "FluidReference":
            ref = child.find("ref_location")
            if ref is not None:
                fluid_reference = {
                    key: _coerce_xml_scalar(value)
                    for key, value in ref.attrib.items()
                }
    return {"part_numbers": part_numbers, "fluid_reference": fluid_reference}


def extract_process_time(path: str | Path) -> dict[str, float]:
    """Extract OEM ProcessTime.xml command timing estimates in seconds."""

    root = ET.parse(path).getroot()
    process_time = root.find("processTime")
    if process_time is None:
        raise ValueError(f"Could not find processTime element in {path}")
    timings: dict[str, float] = {}
    for child in process_time:
        if "process" in child.attrib:
            timings[child.tag] = float(child.attrib["process"])
    if not timings:
        raise ValueError(f"No process timings extracted from {path}")
    return timings


def extract_default_parameters(path: str | Path) -> dict[str, int | float]:
    """Extract numeric public const values from BioXPCommonLib.DefaultParameters."""

    source = Path(path).read_text(encoding="utf-8", errors="replace")
    constants: dict[str, int | float] = {}
    for match in _CONST_RE.finditer(source):
        raw_value = match.group("value")
        constants[match.group("name")] = float(raw_value) if "." in raw_value else int(raw_value)
    if not constants:
        raise ValueError(f"No DefaultParameters constants extracted from {path}")
    return constants


def extract_script_corpus(paths: Iterable[str | Path]) -> dict[str, Any]:
    """Count OEM XML cmd-attribute verbs across standalone script files."""

    total_verbs: Counter[str] = Counter()
    files: dict[str, Any] = {}
    command_count = 0
    for raw_path in paths:
        path = Path(raw_path)
        root = ET.parse(path).getroot()
        file_verbs: Counter[str] = Counter()
        file_commands = 0
        for elem in root.iter():
            raw_cmd = elem.attrib.get("cmd")
            if not raw_cmd:
                continue
            parts = raw_cmd.strip().split()
            if not parts:
                continue
            verb = parts[0].upper()
            file_verbs[verb] += 1
            total_verbs[verb] += 1
            file_commands += 1
            command_count += 1
        files[path.name] = {
            "source_path": str(path),
            "root_tag": root.tag,
            "command_count": file_commands,
            "verbs": dict(sorted(file_verbs.items())),
        }
    return {
        "file_count": len(files),
        "command_count": command_count,
        "verbs": dict(sorted(total_verbs.items())),
        "files": files,
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


def write_position_table_binding(
    *,
    class_bioxp_settings_path: str | Path,
    location_id_path: str | Path,
    output_path: str | Path,
    calreference_path: str | Path | None = None,
    process_time_path: str | Path | None = None,
    default_parameters_path: str | Path | None = None,
    script_paths: Iterable[str | Path] | None = None,
) -> None:
    """Write initial OEM binding JSON for source-backed OEM config data."""

    settings_path = Path(class_bioxp_settings_path)
    enum_path = Path(location_id_path)
    entries = extract_position_table(settings_path, extract_location_id_enum(enum_path))
    sections: dict[str, Any] = {
        "position_table": {
            "status": "available",
            "source_key": "ClassBioXPSettings.m_positionTable",
            "entries": entries,
        },
        "vision_calibration": {
            "status": "not_extracted",
            "reason": "InspectionSettings.xml was referenced by OEM code but not found as a standalone SSD-backup file in phase-0 inventory",
            "source_key": "InspectionSettings.xml",
        },
        "pipette_calibration": {
            "status": "not_extracted",
            "reason": "Pipette constants require ClassPipette/ClassPipetteCollection extraction in a later phase",
            "source_key": "ClassPipetteCollection/ClassPipette",
        },
    }
    if calreference_path is not None:
        sections["calibration_reference"] = {
            "status": "available",
            "source_key": "calreference.xml.deploy",
            **extract_calibration_reference(calreference_path),
        }
    if process_time_path is not None:
        sections["process_time"] = {
            "status": "available",
            "source_key": "ProcessTime.xml.deploy",
            "commands": extract_process_time(process_time_path),
        }
    if default_parameters_path is not None:
        sections["motion_constants"] = {
            "status": "available",
            "source_key": "DefaultParameters.cs",
            "constants": extract_default_parameters(default_parameters_path),
        }
    if script_paths is not None:
        sections["script_corpus"] = {
            "status": "available",
            "source_key": "Scripts/*.xml",
            **extract_script_corpus(script_paths),
        }
    payload = {
        "schema": OEM_BINDING_SCHEMA,
        "source": {
            "source_path": str(settings_path),
            "source_type": "decompiled_csharp",
            "source_key": "ClassBioXPSettings.m_positionTable",
            "source_hash": _sha256(settings_path),
        },
        "sections": sections,
    }
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
