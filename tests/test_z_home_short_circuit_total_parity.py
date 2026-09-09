from types import SimpleNamespace

from src.bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter


def test_move_z_home_source_cached_short_circuit_does_not_invent_ack_or_home_proof():
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
                "source_noop": True,
                "source_return_code": 0,
                "controller_command_acknowledged": False,
                "controller_home_proof_verified": False,
                "controller_terminal_state_verified": False,
                "position_after": {"position": 0, "authority": "oem_cached_CurrentPosition"},
                "home_after": None,
            },
        },
    )

    result = adapter.z_move_z_home()

    assert result["ok"] is True
    assert result["controller_command_acknowledged"] is False
    assert result["controller_terminal_state_verified"] is False
    assert result["physical_effect_verified"] is False


def test_resume_after_abort_reads_nested_axis_search_home_evidence():
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter._z_profile = lambda: {"home_search_max_abs_delta": 160000}  # type: ignore[method-assign]
    adapter.tester = SimpleNamespace(
        motor_query_24v_sensor=lambda: {
            "reply_valid": True,
            "sample_valid": True,
            "oem_no24v": False,
            "oem_scalar": 0,
        },
        motor_oem_axis_search_home=lambda *args, **kwargs: {
            "ok": True,
            "source_return_code": 10000,
            "go_home": {
                "controller_command_acknowledged": True,
                "controller_terminal_state_verified": True,
                "controller_home_proof_verified": True,
            },
        },
    )

    result = adapter.z_resume_after_abort()

    assert result["ok"] is True
    assert result["controller_command_acknowledged"] is True
    assert result["controller_terminal_state_verified"] is True
