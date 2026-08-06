from __future__ import annotations

from copy import deepcopy


# Projection of the accepted serial-206 immutable machine snapshot. These values
# come from the hash-locked runtime bundle, not source defaults or a temporary
# diagnostic XML file.
_SERIAL_206_IMMUTABLE_MACHINE_BUNDLE = {
    "ok": True,
    "accepted_live_mode": True,
    "machine_calibrated": True,
    "source_type": "immutable_oem_machine_snapshot",
    "runtime_binding": "read_only_immutable_evidence",
    "config": {
        "config": {
            "Version": 3,
            "GripperVersion": 1,
            "TroughVersion": 1,
        },
        "calibration": {
            "Calibrated": 1,
        },
        "camera": {
            "Camera": 1,
            "Cameracalibrated": True,
        },
        "axis_limits": {
            "x": {"min_steps": 0, "max_steps": 90263, "source": "serial_206_oem_machine_snapshot"},
            "y": {"min_steps": 0, "max_steps": 102956, "source": "serial_206_oem_machine_snapshot"},
            "z": {"min_steps": 0, "max_steps": 160000, "source": "serial_206_oem_machine_snapshot"},
            "g": {"min_steps": 0, "max_steps": 15000, "source": "serial_206_oem_machine_snapshot"},
        },
        "offsets": {
            "m_TCDoorOpen": 18500,
            "m_TCDoorStallGuardThreshold": 6,
            "m_TC_DOOR_VELOCITY": 50,
            "m_TC_DOOR_ACCELERATION": 20,
            "m_TC_DOOR_MAX_CURRENT": 31,
            "m_Z_MOTOR_MAX_CURRENT_DOWN": 25,
            "m_Z_MOTOR_MAX_CURRENT_UP": 31,
            "m_Z_MOTOR_STALL_GUARD_THRESHOLD": 3,
        },
    },
    "blockers": [],
    "snapshot_status": {
        "serial": 206,
        "lock_sha256": "a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c",
        "operator_label_matched": True,
        "mutation_authorized": True,
        "validation_conflicts": [],
    },
}


def serial_206_immutable_machine_bundle() -> dict:
    """Return an isolated projection of the accepted immutable test authority."""
    return deepcopy(_SERIAL_206_IMMUTABLE_MACHINE_BUNDLE)
