from types import SimpleNamespace

from src.bioxp import oem_serial206_initialization as subject


ACK = {"status": 100}


def test_event_128_is_authoritative_when_final_counter_differs_from_target():
    adapter = subject.Serial206ProductionPrimitiveAdapter.__new__(
        subject.Serial206ProductionPrimitiveAdapter
    )
    adapter.tester = SimpleNamespace(
        motor_wait_stopped=lambda *args, **kwargs: {
            "stopped": True,
            "last_speed": 0,
            "last_ack": ACK,
        },
        collect_bus_events=lambda **kwargs: [
            {
                "board": 4,
                "motor": 1,
                "status": 128,
                "event_sequence": 11,
            }
        ],
        motor_get_position=lambda *args, **kwargs: {
            "ok": True,
            "ack": ACK,
            "position": 10_006,
        },
    )
    adapter.z_stop = lambda **kwargs: (_ for _ in ()).throw(
        AssertionError("successful event-128 completion must not issue failure stop")
    )

    result = adapter._z_finalize_position_move(
        profile={"board": 4, "motor": 1},
        before={"ok": True, "ack": ACK, "position": 0},
        target=10_000,
        move={"ok": True, "ack": ACK},
        wait_timeout_s=20.0,
        pre_command_event_window={"after_sequence": 9},
        event_window={"after_sequence": 10},
    )

    assert result["ok"] is True
    assert result["failure"] is None
    assert result["controller_command_acknowledged"] is True
    assert result["controller_terminal_state_verified"] is True
    assert result["target_position_steps"] == 10_000
    assert result["after_position_steps"] == 10_006
    assert result["target_events"][0]["status"] == 128
    assert result["failure_stop"] is None
