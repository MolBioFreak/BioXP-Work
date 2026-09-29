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
