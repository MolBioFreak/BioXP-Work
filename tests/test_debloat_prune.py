"""Offline regression coverage for removal of unselected compatibility seams."""
from __future__ import annotations

import importlib

import pytest


@pytest.mark.parametrize("module", [
    "can_driver", "command_exchange_observer", "domain.capabilities",
    "interrupt_journal", "motion_safety", "novo_router", "oem_command_contracts",
    "oem_compat.control_lib", "oem_compat.frames", "oem_compat.machine_state",
    "oem_compat.position_table", "oem_compat.state", "oem_deck_catalog",
    "oem_gripper", "oem_homing_routes", "oem_initialization",
    "oem_startup_program", "oem_switch_audit", "operator_receipt_store",
    "operator_reports", "pipette.receipts", "pipette.transport",
    "runtime_audit_store", "runtime_state", "serial206_y_provider", "api",
])
def test_surviving_modules_import(module):
    assert importlib.import_module(f"bioxp.{module}") is not None


def test_compatibility_motion_retains_dry_run_and_addressed_stop():
    from bioxp.oem_compat.boards import BioXPBoards

    boards = BioXPBoards.dry_run()
    motor = boards.axis("x")
    motor.move_absolute(321)
    motor.stop()
    frames = boards.transport.frames
    assert [(f.command, f.motor, f.value) for f in frames] == [
        (4, 0, 321), (3, 0, 0), (3, 0, 0),
    ]
    assert boards.transport.opened_usb is False


def test_real_app_keeps_status_and_protocol_routes():
    from bioxp.api import app

    paths = set(app.openapi()["paths"])
    assert "/status" in paths
    assert "/protocol/compile" in paths


def test_retired_runtime_router_is_not_an_app_dependency():
    import ast
    from pathlib import Path

    retired = {"oem_runtime_api", "oem_runtime_commands", "oem_runtime_events",
               "oem_runtime_status", "oem_runtime_worker"}
    source = Path(__file__).resolve().parents[1] / "src" / "bioxp"
    for name in retired:
        assert not (source / f"{name}.py").exists()
    for path in source.rglob("*.py"):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                assert (node.module or "").split(".")[-1] not in retired, path
            elif isinstance(node, ast.Import):
                assert not any(alias.name.split(".")[-1] in retired for alias in node.names), path


def test_live_registry_identity_remains_byte_derived():
    import hashlib
    from pathlib import Path
    from bioxp.oem_full_lifecycle import current_registry_sha256

    path = Path(__file__).resolve().parents[1] / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json"
    assert current_registry_sha256() == hashlib.sha256(path.read_bytes()).hexdigest()


def test_both_source_thermal_door_profiles_remain():
    from bioxp.oem_config import OEM_THERMAL_DOOR_DEFAULTS_BY_SERIAL_CLASS

    assert OEM_THERMAL_DOOR_DEFAULTS_BY_SERIAL_CLASS["serial_lt_10"]["TCDoorOpen"] == 93000
    assert OEM_THERMAL_DOOR_DEFAULTS_BY_SERIAL_CLASS["serial_ge_10"]["TCDoorOpen"] == 16000


def test_homing_program_modes_survive_historical_model_retirement():
    from bioxp.oem_homing_spec import program_names

    assert {
        "initialize_motors_without_motion", "initialize_motors", "home_axis",
        "home_xy", "rehome", "initialize_motion", "manual_home_x",
        "manual_home_y", "manual_home_z", "manual_home_g", "manual_home_door",
    } <= set(program_names())


def test_standalone_source_api_retains_no_live_import_isolation():
    import os
    import subprocess
    import sys

    # A fresh interpreter is essential: the parent imports the real primary app.
    code = """
import asyncio
import sys

def offline(event, args):
    if event in ('socket.connect', 'socket.getaddrinfo'):
        raise AssertionError('unexpected network')
    if event == 'open' and args and isinstance(args[0], str) and args[0].startswith(('/dev/bus/usb/', '/dev/video', '/var/lib/bioxp-oem-runtime/')):
        raise AssertionError('unexpected hardware/live state')
sys.addaudithook(offline)
from bioxp.oem_source_only_api import list_programs, dry_run_program
programs = asyncio.run(list_programs())
assert programs['opened_usb'] is False
assert programs['programs']
result = asyncio.run(dry_run_program('initialize_motors_without_motion'))
assert result['opened_usb'] is False
assert result['physical_motion'] is False
assert not any(name in sys.modules for name in ('bioxp.api', 'bioxp.usb_driver', 'bioxp.camera_provider', 'src.bioxp.usb_driver'))
"""
    subprocess.run([sys.executable, "-c", code], check=True, env=os.environ.copy(), capture_output=True, text=True)
