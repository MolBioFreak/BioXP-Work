from __future__ import annotations

import hashlib
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Iterable

from .oem_machine_bundle import (
    OemMachineBundleError,
    OemMachineSnapshot,
    get_active_oem_machine_snapshot,
)

OEM_AXIS_LIMIT_SOURCE_EVIDENCE = {
    "defaults": "ClassBioXPSettings.cs lines 250-263: m_min*steps=0, m_maxXsteps=80000, m_maxYsteps=80000, m_maxZsteps=160000, m_maxGsteps=15000",
    "properties": "ClassBioXPSettings.cs lines 1277-1299: X/Y/Z/G HighLimit/LowLimit and SelfTest*Max properties",
    "config_loader": "ClassBioXPSettings.cs lines 2846-2854: config.xml load from CWD or app dir; lines 3310-3383 parse AxisLimits/*_limit minSteps/maxSteps",
    "config_writer": "ClassBioXPSettings.cs lines 3898-3931: writes AxisLimits/*_limit minSteps/maxSteps",
    "motor_binding": "ClassControlInterface.cs lines 389-422: setLimits applied to MotorX/MotorY/MotorZ during construction",
    "absolute_clamp": "ClassMotor.cs lines 290-302 and 768-772: moveToAbs clamps to stored low/high position limits",
    "field_calibration_limit_update": "ClassBioXPSettings.cs lines 4744-4754: Auto_XY/Auto_XY_New calibration rewrites m_maxXsteps=x-500 and m_maxYsteps=y-500",
    "deck_table_extents": "ClassBioXPSettings.cs default PositionTable includes X=91919/Y=93208 at locationID 6 and Y=95247 at Tip Tray 3/4, above the compiled 80000 fallback limits",
}

OEM_DEFAULT_DECK_COORDINATE_EXTENTS = {
    "x": {
        "max_steps": 91919,
        "source": "oem_default_position_table",
        "source_anchor": "ClassBioXPSettings.cs locationID 6 default x=91919 y=93208",
    },
    "y": {
        "max_steps": 95247,
        "source": "oem_default_position_table",
        "source_anchor": "ClassBioXPSettings.cs Tip Tray 3/4 default y=95247",
    },
}

OEM_DEFAULT_AXIS_LIMITS = {
    "x": {"min_steps": 0, "max_steps": 91919, "source": "oem_default_deck_extent_harmonized"},
    "y": {"min_steps": 0, "max_steps": 95247, "source": "oem_default_deck_extent_harmonized"},
    "z": {"min_steps": 0, "max_steps": 160000, "source": "oem_default_class_bioxpsettings"},
    "g": {"min_steps": 0, "max_steps": 15000, "source": "oem_default_class_bioxpsettings"},
}

OEM_THERMAL_DOOR_DEFAULTS_BY_SERIAL_CLASS = {
    "serial_lt_10": {
        "TCDoorOpen": 93000,
        "TC_DOOR_VELOCITY": 900,
        "TC_DOOR_ACCELERATION": 20,
        "TC_DOOR_MAX_CURRENT": 31,
        "TCDoorStallGuardThreshold": 6,
    },
    "serial_ge_10": {
        "TCDoorOpen": 16000,
        "TC_DOOR_VELOCITY": 50,
        "TC_DOOR_ACCELERATION": 20,
        "TC_DOOR_MAX_CURRENT": 31,
        "TCDoorStallGuardThreshold": 6,
    },
}


def oem_thermal_door_defaults(serial_number: int | str | None) -> dict[str, int]:
    """Return OEM thermal-door settings from ClassBioXPSettings serial branch.

    OEM source: serial <10 uses the legacy long-travel door profile; serial >=10
    uses the BioXP3200-era profile (TCDoorOpen=16000, velocity=50). Unknown
    serials fail closed to the >=10 profile used by this instrument class.
    """
    try:
        serial = int(serial_number) if serial_number is not None else 10
    except (TypeError, ValueError):
        serial = 10
    key = "serial_lt_10" if serial < 10 else "serial_ge_10"
    return dict(OEM_THERMAL_DOOR_DEFAULTS_BY_SERIAL_CLASS[key])


OEM_CRITICAL_SOURCE_DEFAULTS = {
    "Z_MOTOR_MAX_CURRENT_UP": 31,
    "Z_MOTOR_MAX_CURRENT_DOWN": 25,
    "Z_MOTOR_STALL_GUARD_THRESHOLD": 3,
    **OEM_THERMAL_DOOR_DEFAULTS_BY_SERIAL_CLASS["serial_ge_10"],
}

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

DIAGNOSTIC_NON_AUTHORITATIVE_SEARCH_ROOTS = [
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/.deploy",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Users",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/ProgramData",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Program Files",
    "/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Program Files (x86)",
]
# Compatibility-only alias. Accepted live mode never consults these roots.
DEFAULT_SEARCH_ROOTS = DIAGNOSTIC_NON_AUTHORITATIVE_SEARCH_ROOTS

OEM_MACHINE_CONFIG_ENV = "BIOXP_OEM_MACHINE_CONFIG_DIR"
OEM_MACHINE_CONFIG_PATH_ENV = "BIOXP_OEM_MACHINE_CONFIG_XML"
OEM_EXPECTED_MACHINE_FILES = {
    "config.xml": "config_xml",
    "Operation_parameters.xml": "operation_parameters_xml",
    "InspectionSettings.xml": "inspection_settings_xml",
    "processtime.xml": "processtime_xml",
    "ProcessTime.xml": "processtime_xml",
    "calreference.xml": "calreference_xml",
}
SENSITIVE_CONFIG_ATTRS = {"password", "host", "accesskey", "simpleServer".lower(), "jobinfolocation"}


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _redact_attrs(attrs: dict[str, str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in attrs.items():
        if key.lower() in SENSITIVE_CONFIG_ATTRS and value:
            out[key] = "[REDACTED]"
        elif key.lower() == "serialnumber" or key.lower() == "genbot":
            out[key] = "[REDACTED]" if value else value
        else:
            out[key] = _typed(value) if isinstance(value, str) else value
    return out


def _first_attrs(root: ET.Element, path: str) -> dict[str, Any]:
    elem = root.find(path)
    return _redact_attrs(dict(elem.attrib)) if elem is not None else {}


def _file_record(path: Path, role: str) -> dict[str, Any]:
    return {
        "role": role,
        "path": str(path),
        "size": path.stat().st_size,
        "sha256": _sha256_file(path),
        "source_type": "original_ssd_appdata_extracted",
    }


def _find_expected_file(root: Path, filename: str) -> Path | None:
    direct = root / filename
    if direct.exists():
        return direct
    lowered = filename.lower()
    for path in root.iterdir() if root.exists() and root.is_dir() else []:
        if path.is_file() and path.name.lower() == lowered:
            return path
    return None


def parse_oem_machine_config_bundle(root_dir: str | Path) -> dict[str, Any]:
    """Parse an extracted OEM appdata config bundle without hardware side effects.

    Expected input is a directory containing config.xml plus optional
    Operation_parameters.xml, InspectionSettings.xml, processtime.xml, and
    calreference.xml copied from the original Windows SSD appdata directory.
    Raw secrets remain in files; returned data redacts serial/server fields.
    """
    root_dir = Path(root_dir)
    files: dict[str, dict[str, Any]] = {}
    for filename, role in OEM_EXPECTED_MACHINE_FILES.items():
        found = _find_expected_file(root_dir, filename)
        if found is not None and role not in files:
            files[role] = _file_record(found, role)
    config_path = Path(files["config_xml"]["path"]) if "config_xml" in files else None
    if config_path is None:
        return {
            "ok": False,
            "machine_calibrated": False,
            "source_type": "original_ssd_appdata_extracted",
            "root_dir": str(root_dir),
            "files": files,
            "blockers": ["config.xml_missing"],
            "runtime_binding": "read_only",
        }
    root = ET.parse(config_path).getroot()
    axis_limits = _extract_axis_limits(root)
    positions = []
    table = root.find("./PositionTable")
    if table is not None:
        for elem in list(table):
            row = {"name": elem.tag}
            for k, v in elem.attrib.items():
                row[k] = _typed(v)
            if "zLow" in row and "zDelta" in row and "zHigh" not in row:
                try:
                    row["zHigh"] = int(row["zLow"]) - int(row["zDelta"])
                except (TypeError, ValueError):
                    pass
            positions.append(row)
    calibration = _first_attrs(root, "./GenBot/Calibration")
    config = _first_attrs(root, "./GenBot/Config")
    camera = _first_attrs(root, "./GenBot/CameraInstalled")
    offsets = _first_attrs(root, "./CalibrationFactors/Offsets")
    active = {
        "serial_present": bool(root.find("./GenBot/SerialNumber") is not None and root.find("./GenBot/SerialNumber").attrib),
        "serial_redacted": "[REDACTED]" if root.find("./GenBot/SerialNumber") is not None else None,
        "config": config,
        "calibration": calibration,
        "camera": camera,
        "server_redacted": _first_attrs(root, "./GenBot/Server"),
        "offsets": offsets,
        "seal_cut": _first_attrs(root, "./CalibrationFactors/SealCut"),
        "reagent_chiller": _first_attrs(root, "./CalibrationFactors/ReagentChiller"),
        "output_chiller": _first_attrs(root, "./CalibrationFactors/OutputChiller"),
        "axis_limits": axis_limits,
        "position_table_count": len(positions),
        "position_table": positions,
        "scale_port": _first_attrs(root, "./ScalePort/Port"),
    }
    machine_calibrated = str(calibration.get("Calibrated", "")).lower() in {"1", "true", "yes"}
    diffs = machine_config_diff(active)
    return {
        "ok": True,
        "machine_calibrated": machine_calibrated,
        "source_type": "original_ssd_appdata_extracted",
        "root_dir": str(root_dir),
        "runtime_binding": "read_only",
        "files": files,
        "config": active,
        "diff_vs_source_defaults": diffs,
        "blockers": [],
        "caveats": [
            "Read-only binding only; no motion or homing behavior consumes these values yet.",
            "Axis limits and position table are OEM software configuration, not live physical endpoint proof by themselves.",
        ],
    }


def machine_config_diff(parsed_config: dict[str, Any]) -> dict[str, Any]:
    axis_limits = parsed_config.get("axis_limits", {}) if isinstance(parsed_config, dict) else {}
    changed_axis_limits: dict[str, Any] = {}
    for axis, default in OEM_DEFAULT_AXIS_LIMITS.items():
        actual = axis_limits.get(axis, {}) if isinstance(axis_limits, dict) else {}
        if not isinstance(actual, dict):
            continue
        if actual.get("min_steps") != default.get("min_steps") or actual.get("max_steps") != default.get("max_steps"):
            changed_axis_limits[axis] = {"source_default": default, "machine_config": actual}
    offsets = parsed_config.get("offsets", {}) if isinstance(parsed_config, dict) else {}
    key_map = {
        "m_Z_MOTOR_MAX_CURRENT_UP": "Z_MOTOR_MAX_CURRENT_UP",
        "m_Z_MOTOR_MAX_CURRENT_DOWN": "Z_MOTOR_MAX_CURRENT_DOWN",
        "m_Z_MOTOR_STALL_GUARD_THRESHOLD": "Z_MOTOR_STALL_GUARD_THRESHOLD",
        "m_TCDoorStallGuardThreshold": "TCDoorStallGuardThreshold",
        "m_TC_DOOR_VELOCITY": "TC_DOOR_VELOCITY",
        "m_TC_DOOR_ACCELERATION": "TC_DOOR_ACCELERATION",
        "m_TC_DOOR_MAX_CURRENT": "TC_DOOR_MAX_CURRENT",
        "m_TCDoorOpen": "TCDoorOpen",
    }
    changed_constants: dict[str, Any] = {}
    if isinstance(offsets, dict):
        for machine_key, default_key in key_map.items():
            if machine_key not in offsets:
                continue
            machine_value = offsets[machine_key]
            default_value = OEM_CRITICAL_SOURCE_DEFAULTS.get(default_key)
            if machine_value != default_value:
                changed_constants[default_key] = {
                    "machine_key": machine_key,
                    "source_default": default_value,
                    "machine_config": machine_value,
                }
    config = parsed_config.get("config", {}) if isinstance(parsed_config, dict) else {}
    return {
        "axis_limits_changed_from_prior_defaults": changed_axis_limits,
        "critical_constants_changed_from_source_defaults": changed_constants,
        "gripper_version": (config or {}).get("GripperVersion") if isinstance(config, dict) else None,
        "position_table_count": parsed_config.get("position_table_count"),
        "summary": {
            "axis_limit_changes": len(changed_axis_limits),
            "critical_constant_changes": len(changed_constants),
        },
    }


def find_oem_machine_config_bundle(root_dir: str | Path | None = None) -> dict[str, Any]:
    """Project the process-bound snapshot, or an explicit diagnostic fixture.

    A directory argument is retained for offline/backward-compatible diagnostics
    only. It can never replace an already-bound accepted live snapshot.
    """
    try:
        snapshot = get_active_oem_machine_snapshot()
    except OemMachineBundleError:
        snapshot = None
    if snapshot is not None:
        if root_dir is not None:
            raise OemMachineBundleError("accepted live snapshot authority cannot be replaced by root_dir")
        return _snapshot_legacy_bundle(snapshot)
    if root_dir is None:
        return {
            "ok": False,
            "machine_calibrated": False,
            "source_type": "unbound",
            "runtime_binding": "read_only",
            "accepted_live_mode": False,
            "blockers": ["OEM_machine_snapshot_not_bound"],
        }
    diagnostic = parse_oem_machine_config_bundle(root_dir)
    diagnostic["accepted_live_mode"] = False
    diagnostic["non_authoritative_diagnostic_only"] = True
    diagnostic.setdefault("blockers", []).append("explicit_directory_is_not_accepted_live_authority")
    return diagnostic


def _snapshot_legacy_bundle(snapshot: OemMachineSnapshot) -> dict[str, Any]:
    sections = {name: dict(values) for name, values in snapshot.config_sections.items()}
    positions = [dict(row) for row in snapshot.position_table]
    operation_parameters = dict(snapshot.operation_parameters)
    try:
        from .runtime_state import get_active_oem_runtime_state_store

        store = get_active_oem_runtime_state_store()
        if store.snapshot is snapshot:
            operation_parameters = store.operation_parameters_projection()
    except Exception:
        pass
    active = {
        "serial_present": True,
        "serial_redacted": "[REDACTED]",
        "config": sections["config"],
        "calibration": sections["calibration"],
        "camera": sections["camera"],
        "server_redacted": {},
        "offsets": sections["offsets"],
        "seal_cut": sections["seal_cut"],
        "reagent_chiller": sections["reagent_chiller"],
        "output_chiller": sections["output_chiller"],
        "axis_limits": {axis: dict(row) for axis, row in snapshot.axis_limits.items()},
        "position_table_count": len(positions),
        "position_table": positions,
        "scale_port": sections["scale_port"],
        "operation_parameters": operation_parameters,
        "inspection_profile": snapshot.inspection_profile_name,
    }
    files = {
        "config_xml": snapshot.records["appdata/config.xml"].public_projection(),
        "operation_parameters_xml": snapshot.records["appdata/Operation_parameters.xml"].public_projection(),
        "inspection_settings_xml": snapshot.records["appdata/InspectionSettings.xml"].public_projection(),
        "processtime_xml": snapshot.records["appdata/processtime.xml"].public_projection(),
        "calreference_xml": snapshot.records["appdata/calreference.xml"].public_projection(),
    }
    for record in files.values():
        record["path"] = str(snapshot.bundle_root / record["bundle_relative_path"])
    return {
        "ok": True,
        "accepted_live_mode": True,
        "machine_calibrated": snapshot.machine_calibrated,
        "source_type": "immutable_oem_machine_snapshot",
        "root_dir": str(snapshot.bundle_root),
        "runtime_binding": "read_only_immutable_evidence",
        "files": files,
        "config": active,
        "diff_vs_source_defaults": machine_config_diff(active),
        "blockers": [],
        "snapshot_status": snapshot.config_status_projection(),
    }


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


def _attr_int(elem: ET.Element, *names: str) -> int | None:
    for name in names:
        if name in elem.attrib:
            try:
                return int(str(elem.attrib[name]).strip())
            except (TypeError, ValueError):
                return None
    return None


def _extract_axis_limits(root: ET.Element) -> dict[str, dict[str, Any]]:
    """Extract OEM AxisLimits/*_limit minSteps/maxSteps overrides.

    The decompiled OEM settings loader parses ``AxisLimits/X_limit`` etc. with
    minSteps/maxSteps attributes and falls back to ClassBioXPSettings defaults
    when config.xml is absent.  Preserve that same provenance instead of
    inventing live-tested limits.
    """
    limits: dict[str, dict[str, Any]] = {
        axis: dict(row) for axis, row in OEM_DEFAULT_AXIS_LIMITS.items()
    }
    for elem in root.iter():
        tag = str(elem.tag).split("}")[-1]
        low = tag.lower()
        if not low.endswith("_limit"):
            continue
        axis = low.split("_", 1)[0]
        if axis not in limits:
            continue
        min_steps = _attr_int(elem, "minSteps", "minsteps", "MinSteps", "min")
        max_steps = _attr_int(elem, "maxSteps", "maxsteps", "MaxSteps", "max")
        if min_steps is None and max_steps is None:
            continue
        row = dict(limits[axis])
        if min_steps is not None:
            row["min_steps"] = min_steps
        if max_steps is not None:
            row["max_steps"] = max_steps
        row["source"] = "config_xml_axislimits"
        row["config_tag"] = tag
        limits[axis] = row
    return limits


def _axis_limit_diagnostics(axis_limits: dict[str, dict[str, Any]], config_status: str | None) -> dict[str, dict[str, Any]]:
    diagnostics: dict[str, dict[str, Any]] = {}
    for axis in ("x", "y"):
        limit = axis_limits.get(axis, {}) if isinstance(axis_limits, dict) else {}
        extent = OEM_DEFAULT_DECK_COORDINATE_EXTENTS.get(axis, {})
        max_limit = limit.get("max_steps") if isinstance(limit, dict) else None
        deck_max = extent.get("max_steps") if isinstance(extent, dict) else None
        try:
            below_deck = max_limit is not None and deck_max is not None and int(max_limit) < int(deck_max)
            shortfall = None if not below_deck else int(deck_max) - int(max_limit)
        except (TypeError, ValueError):
            below_deck = False
            shortfall = None
        diagnostics[axis] = {
            "configured_max_steps": max_limit,
            "deck_extent_max_steps": deck_max,
            "default_limit_below_deck_extent": bool(below_deck),
            "shortfall_to_default_deck_extent_steps": shortfall,
            "recommended_status": "missing_field_calibration_or_config_override" if below_deck and config_status != "loaded" else "config_or_default_consistent_with_known_deck_extent",
            "interpretation": "Do not treat compiled 80000 fallback as authoritative physical/deck coverage when OEM deck coordinates exceed it and no field config.xml was loaded."
            if below_deck
            else "Configured limit is not below the known OEM default deck coordinate extent for this axis.",
        }
    return diagnostics


def harmonized_motion_config(config_result: dict | None = None) -> dict:
    """Return source-grounded motion config, preferring bound machine SSD config.

    This is still read-only metadata. It does not command motion; it only makes
    status/planning surfaces stop reporting generic source extents when the
    original SSD's machine config has been explicitly bound.
    """
    machine = None if config_result is not None else find_oem_machine_config_bundle()
    if isinstance(machine, dict) and machine.get("ok") is True:
        machine_config = machine.get("config", {}) if isinstance(machine.get("config"), dict) else {}
        axis_limits = machine_config.get("axis_limits") if isinstance(machine_config, dict) else None
        source = "immutable_oem_machine_snapshot"
        config_status: dict[str, Any] = {
            "status": "loaded",
            "path": (machine.get("files", {}).get("config_xml", {}) or {}).get("path"),
            "source_type": machine.get("source_type"),
            "machine_calibrated": machine.get("machine_calibrated"),
            "runtime_binding": machine.get("runtime_binding"),
            "files": machine.get("files", {}),
            "diff_vs_source_defaults": machine.get("diff_vs_source_defaults", {}),
        }
    elif config_result is not None:
        config = config_result
        axis_limits = config.get("axis_limits") if isinstance(config, dict) else None
        source = "non_authoritative_diagnostic_config_xml" if isinstance(config, dict) and config.get("status") == "loaded" and any(
            isinstance(row, dict) and row.get("source") == "config_xml_axislimits" for row in (axis_limits or {}).values()
        ) else "diagnostic_config_unavailable"
        config_status = config if isinstance(config, dict) else {"status": "missing"}
    else:
        axis_limits = {}
        source = "fail_closed_oem_machine_snapshot_not_bound"
        config_status = {"status": "unbound", "blockers": ["OEM_machine_snapshot_not_bound"]}
    live_bound = source == "immutable_oem_machine_snapshot"
    return {
        "ok": live_bound or config_result is not None,
        "schema_version": "bioxp.oem_motion_config.v1",
        "source": source,
        "config_status": config_status,
        "axis_limits": axis_limits,
        "deck_coordinate_extents": {axis: dict(row) for axis, row in OEM_DEFAULT_DECK_COORDINATE_EXTENTS.items()},
        "axis_limit_diagnostics": _axis_limit_diagnostics(axis_limits, config_status.get("status") if isinstance(config_status, dict) else None),
        "source_evidence": OEM_AXIS_LIMIT_SOURCE_EVIDENCE,
        "live_ready": live_bound,
        "caveats": [
            "These are software/configured OEM limits, not proof of physical endpoint travel.",
            "Accepted live mode exposes no axis limits unless the immutable serial-206 snapshot is bound.",
            "Compiled source defaults remain diagnostic metadata only and are not accepted live authority.",
            "Auto_XY/Auto_XY_New field calibration in OEM code rewrites X/Y high limits to measured travel minus 500 steps.",
            "Live Linux Z sign convention observed during commissioning uses negative values for upward/head-clear motion; OEM limits are source-space positive steps.",
        ],
    }


def load_oem_config(path: str | Path) -> dict:
    p = Path(path)
    fields: dict[str, str] = {}
    axis_limits: dict[str, dict[str, Any]] = {axis: dict(row) for axis, row in OEM_DEFAULT_AXIS_LIMITS.items()}
    try:
        tree = ET.parse(p)
        root = tree.getroot()
        axis_limits = _extract_axis_limits(root)
        for elem in root.iter():
            tag = str(elem.tag).split("}")[-1]
            text = (elem.text or "").strip()
            if text and (tag in INTERESTING_FIELDS or tag.endswith("Max") or "Motor" in tag or "ThermalDoor" in tag or tag.startswith("TC_DOOR")):
                fields[tag] = text
            for attr_key, attr_value in elem.attrib.items():
                if attr_key in INTERESTING_FIELDS or attr_key in {"GripperVersion", "Cameracalibrated", "Calibrated"} or "Motor" in attr_key or attr_key.startswith("m_Z_MOTOR") or attr_key.startswith("m_TC_DOOR") or attr_key.startswith("m_TCDoor"):
                    normalized = "CameraCalibrated" if attr_key == "Cameracalibrated" else attr_key
                    fields[normalized] = attr_value
    except Exception as exc:
        return {"status": "error", "path": str(p), "error": str(exc), "fields": {}, "fields_typed": {}, "axis_limits": axis_limits, "missing_fields": REQUIRED_FOR_LIVE, "derived_requirements": {}}
    missing = [name for name in REQUIRED_FOR_LIVE if name not in fields]
    return {
        "status": "loaded",
        "path": str(p),
        "fields": fields,
        "fields_raw": dict(fields),
        "fields_typed": {k: _typed(v) for k, v in fields.items()},
        "axis_limits": axis_limits,
        "missing_fields": missing,
        "live_ready": not missing,
        "derived_requirements": _derived_requirements(fields),
    }


def find_oem_config(roots: Iterable[str | Path] | None = None) -> dict:
    if roots is None:
        try:
            snapshot = get_active_oem_machine_snapshot()
        except OemMachineBundleError:
            return {"status": "unbound", "path": None, "searched_roots": [], "fields": {}, "fields_raw": {}, "fields_typed": {}, "axis_limits": {}, "missing_fields": ["OemMachineSnapshot"], "live_ready": False, "accepted_live_mode": False, "derived_requirements": {}}
        fields = {
            "StartMode": snapshot.startup_mode,
            "GripperVersion": str(snapshot.fields["machine.gripper_version"].raw_value),
            "Calibrated": str(snapshot.fields["machine.calibrated"].raw_value),
            "CameraCalibrated": str(snapshot.fields["machine.camera_calibrated"].raw_value),
            "CheckCamera": str(snapshot.operation_parameters["CheckCamera"]),
        }
        try:
            from .runtime_state import get_active_oem_runtime_state_store

            store = get_active_oem_runtime_state_store()
            if store.snapshot is snapshot:
                operation = store.operation_parameters_projection()
                fields["StartMode"] = str(operation["Mode"])
                fields["CheckCamera"] = str(operation["CheckCamera"])
        except Exception:
            pass
        return {
            "status": "loaded",
            "path": str(snapshot.bundle_root / "appdata/config.xml"),
            "searched_roots": [],
            "fields": fields,
            "fields_raw": dict(fields),
            "fields_typed": {key: _typed(value) for key, value in fields.items()},
            "axis_limits": {axis: dict(row) for axis, row in snapshot.axis_limits.items()},
            "missing_fields": [],
            "live_ready": True,
            "accepted_live_mode": True,
            "derived_requirements": {
                "camera_check_required": _typed(fields["CheckCamera"]) is True,
                "camera_calibrated": snapshot.camera_calibrated,
                "calibrated": snapshot.machine_calibrated,
                "start_mode": fields["StartMode"],
                "gripper_version": snapshot.fields["machine.gripper_version"].value,
            },
            "snapshot_status": snapshot.config_status_projection(),
        }
    searched = [str(Path(r)) for r in roots]
    for root in searched:
        for candidate in _candidate_paths(Path(root)):
            loaded = load_oem_config(candidate)
            loaded["searched_roots"] = searched
            loaded["accepted_live_mode"] = False
            loaded["non_authoritative_diagnostic_only"] = True
            return loaded
    return {"status": "missing", "path": None, "searched_roots": searched, "fields": {}, "fields_raw": {}, "fields_typed": {}, "axis_limits": {}, "missing_fields": REQUIRED_FOR_LIVE, "live_ready": False, "accepted_live_mode": False, "non_authoritative_diagnostic_only": True, "derived_requirements": {}}
