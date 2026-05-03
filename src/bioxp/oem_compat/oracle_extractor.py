from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Mapping

from src.bioxp.domain.oem_bindings import OEM_BINDING_SCHEMA

_LOCATION_ENTRY_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*(?:=\s*(-?\d+))?\s*,?")
_POSITION_BLOCK_RE = re.compile(
    r"\{\s*\(locationID\)\s*(?P<ordinal>\d+)\s*,\s*new\s+positionStruct\s*\{(?P<body>.*?)\}\s*\}",
    re.DOTALL,
)
_FIELD_RE = re.compile(r"(?P<key>name|x|y|zLow|zHigh|zDelta|inc_factor)\s*=\s*(?P<value>\"[^\"]*\"|-?\d+)")


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
) -> None:
    """Write initial OEM binding JSON for the source-backed default PositionTable."""

    settings_path = Path(class_bioxp_settings_path)
    enum_path = Path(location_id_path)
    entries = extract_position_table(settings_path, extract_location_id_enum(enum_path))
    payload = {
        "schema": OEM_BINDING_SCHEMA,
        "source": {
            "source_path": str(settings_path),
            "source_type": "decompiled_csharp",
            "source_key": "ClassBioXPSettings.m_positionTable",
            "source_hash": _sha256(settings_path),
        },
        "sections": {
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
        },
    }
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
