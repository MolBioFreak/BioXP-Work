from __future__ import annotations

from typing import Callable

import pytest

from src.bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
from src.bioxp.usb_driver import BioXpTester


def _bound_oem_machine_config():
    return {
        "ok": True,
        "config": {
            "axis_limits": {
                "x": {"max_steps": 91919},
                "y": {"max_steps": 95247},
                "z": {"max_steps": 160000},
                "g": {"max_steps": 15000},
            },
            "offsets": {
                "m_Z_MOTOR_MAX_CURRENT_DOWN": 25,
                "m_Z_MOTOR_MAX_CURRENT_UP": 31,
                "m_Z_MOTOR_STALL_GUARD_THRESHOLD": 3,
            },
        },
    }


def test_z_profile_is_source_positive_board_four_motor_one(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "_machine_config_bundle", _bound_oem_machine_config)

    profile = tester._motion_oem_axis_profile("z", startup=True)

    assert profile["board"] == 4
    assert profile["motor"] == 1
    assert profile["coordinate_contract"] == "oem_source_nonnegative_z"
    assert profile["axis_min_steps"] == 0
    assert profile["axis_max_steps"] == 160000
    assert profile["home_speed"] == 1791
    assert profile["oem_home_step"] == "MotorZ.axisSearchHome(speed=1791)"
    assert "disable_right" not in profile
    assert "disable_left" not in profile


def test_signed_z_reference_authority_is_retired():
    tester = BioXpTester.__new__(BioXpTester)

    with pytest.raises(RuntimeError, match="retired signed-Z reference authority"):
        tester.motor_oem_move_z_to_reference(target_position=0, timeout_s=10)


class _ZPrimitiveSpy:
    def __init__(self):
        self.calls: list[tuple[str, dict]] = []
        self.board_generation = 1
        self.observer: Callable[..., object] | None = None

    def prepare_for_initialize_motors(self, *, expected_generation: int):
        self.calls.append(("prepare", {"generation": expected_generation}))
        if self.observer is not None:
            self.observer(4, {"status": 100}, active=False)
            self.observer(4, {"status": 100}, active=True)
        return {
            "ok": True,
            "observed_generation": expected_generation,
            "board_lifecycle_generation": self.board_generation,
            "board_preparation_verified": True,
            "initialize_without_motion_verified": True,
            "physical_motion": False,
        }

    def z_manual_home(self, *, timeout_s: float):
        self.calls.append(("manual_home", {"timeout_s": timeout_s}))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }

    def current_board_lifecycle_generation(self):
        return self.board_generation

    def z_move_steps(self, *, steps: int, wait_timeout_s: float):
        self.calls.append(("move_steps", {"steps": steps, "timeout_s": wait_timeout_s}))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }

    def z_move_absolute(
        self,
        *,
        requested_position_steps: int,
        pseudo_home_steps: int,
        wait_timeout_s: float,
    ):
        self.calls.append(("move_absolute", {
            "position": requested_position_steps,
            "pseudo_home_steps": pseudo_home_steps,
            "wait_timeout_s": wait_timeout_s,
        }))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }

    def z_stop(self):
        self.calls.append(("stop", {}))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
        }


class _ReferenceStore:
    def mark_desynced(self, command):
        return {"ok": True, "state": "desynced", "persisted": True}

    def mark_referenced(self, command):
        return {
            "ok": True,
            "state": "referenced",
            "persisted": True,
            "verified": True,
            "durable_clean": True,
        }


def test_provider_owns_z_lifecycle_and_routes_only_source_positive_intents():
    spy = _ZPrimitiveSpy()
    provider = Serial206OemInitializationProvider(
        spy,
        generation_provider=lambda: 1,
        reference_store=_ReferenceStore(),
    )
    spy.observer = provider.notify_board_activation

    prepared = provider.execute_z_intent(
        "prepare", expected_generation=1, idempotency_key="prepare-12345678"
    )
    assert prepared["ok"] is True
    assert prepared["z_state"] == "prepared_unreferenced"

    homed = provider.execute_z_intent(
        "manual_home", expected_generation=1, idempotency_key="home-12345678"
    )
    assert homed["ok"] is True
    command_id = homed["authority_receipt"]["command_id"]
    assert homed["z_state"] == "awaiting_operator_observation"

    observed = provider.record_z_observation(
        command_id=command_id,
        verdict="pass",
        physical_motion_observed=True,
        expected_direction_observed=True,
        home_endpoint_observed=True,
        stopped_observed=True,
        note="Independent physical observation recorded.",
        expected_generation=1,
    )
    assert observed["z_state"] == "referenced_ready"

    moved = provider.execute_z_intent(
        "move_steps",
        inputs={"steps": 250},
        expected_generation=1,
        idempotency_key="move-steps-12345678",
    )
    assert moved["ok"] is True
    assert moved["z_state"] == "referenced_ready"

    absolute = provider.execute_z_intent(
        "move_absolute",
        inputs={"position_steps": 70000},
        expected_generation=1,
        idempotency_key="move-abs-12345678",
    )
    assert absolute["ok"] is True
    assert ("move_steps", {"steps": 250, "timeout_s": 20.0}) in spy.calls
    assert ("move_absolute", {
        "position": 70000,
        "pseudo_home_steps": 65000,
        "wait_timeout_s": 20.0,
    }) in spy.calls
    assert [name for name, _ in spy.calls] == ["prepare", "manual_home", "move_steps", "move_absolute"]
