from types import SimpleNamespace

import pytest

from src.bioxp import oem_serial206_initialization as subject


class _PositionTable:
    source = "test-bound-position-table"

    def rows(self):
        return [{"z_low": 120_000}]


def _adapter(monkeypatch, *, source_return_code: int, move_ok: bool = True):
    calls = []
    adapter = subject.Serial206ProductionPrimitiveAdapter.__new__(
        subject.Serial206ProductionPrimitiveAdapter
    )
    adapter._z_profile_overrides = {}
    adapter._z_profile = lambda: calls.append("profile") or {"board": 4, "motor": 1}
    adapter.z_move_z_home = lambda **kwargs: calls.append("MoveZHome") or {
        "ok": True,
        "controller_command_acknowledged": True,
        "controller_terminal_state_verified": True,
    }
    adapter.oem_move_z = lambda target, **kwargs: calls.append(("moveZ", target, kwargs["motor_current"])) or {
        "ok": move_ok,
        "controller_command_acknowledged": move_ok,
        "controller_terminal_state_verified": move_ok,
    }
    adapter.z_diagnostic_home_axis = lambda **kwargs: calls.append("HomeAxis(z,597)") or {
        "ok": True,
        "controller_command_acknowledged": True,
        "controller_terminal_state_verified": True,
        "home": {
            "source_return_code": source_return_code,
            "controller_home_proof_verified": True,
            "short_circuit": None,
        },
    }
    monkeypatch.setattr(subject, "load_bound_oem_position_table", lambda: _PositionTable())
    monkeypatch.setattr(
        subject,
        "load_oem_parity_config",
        lambda _path: SimpleNamespace(
            blockers=[],
            values={"Z_MOTOR_MAX_POSITION": 160000, "Z_MOTOR_MAX_CURRENT_UP": 31},
        ),
    )
    return adapter, calls


@pytest.mark.parametrize(
    ("travel_error", "expected_pass"),
    [(0, True), (100, True), (101, False)],
)
def test_z_self_test_uses_frozen_target_and_exact_tolerance(monkeypatch, travel_error, expected_pass):
    target = subject.SERIAL206_Z_SELF_TEST_MAX_STEPS
    adapter, calls = _adapter(monkeypatch, source_return_code=target + travel_error)

    result = adapter.z_self_test(pseudo_z_home_steps=500, wait_timeout_s=30.0)

    assert result["self_test_z_max"] == 92049
    assert result["self_test_tolerance_steps"] == 100
    assert result["travel_error_steps"] == travel_error
    assert result["ok"] is expected_pass
    assert calls == ["profile", "MoveZHome", ("moveZ", 92049, 31), "HomeAxis(z,597)"]


def test_z_self_test_executes_final_home_after_failed_outbound_move(monkeypatch):
    adapter, calls = _adapter(
        monkeypatch,
        source_return_code=subject.SERIAL206_Z_SELF_TEST_MAX_STEPS,
        move_ok=False,
    )

    result = adapter.z_self_test(pseudo_z_home_steps=500, wait_timeout_s=30.0)

    assert result["ok"] is False
    assert calls[-1] == "HomeAxis(z,597)"
