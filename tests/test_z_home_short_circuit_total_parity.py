from types import SimpleNamespace

from src.bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter


def test_move_z_home_source_short_circuit_is_acknowledged_by_controller_home_proof():
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter._z_profile = lambda: {"board": 4, "motor": 1}  # type: ignore[method-assign]
    adapter._z_profile_overrides = {}
    adapter.tester = SimpleNamespace(
        motor_oem_verify_motion_interlock=lambda: {"ok": True},
        motor_oem_move_z_home=lambda **kwargs: {
            "ok": True,
            "home": {
                "ok": True,
                "short_circuit": "MotorHome_and_CurrentPosition_zero",
                "controller_home_proof_verified": True,
                "controller_terminal_state_verified": True,
                "position_after": {"value": 0},
                "home_after": {"value": 1},
            },
        },
    )

    result = adapter.z_move_z_home()

    assert result["ok"] is True
    assert result["controller_command_acknowledged"] is True
    assert result["controller_terminal_state_verified"] is True
