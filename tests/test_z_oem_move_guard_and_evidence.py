import threading
import time
from types import SimpleNamespace

from bioxp import oem_serial206_initialization as subject


ACK = {"status": 100}


def _adapter(position: int):
    calls: list[tuple[str, object]] = []
    tester = SimpleNamespace(
        motor_get_position=lambda *args, **kwargs: {"ok": True, "ack": ACK, "position": position},
        begin_bus_event_window=lambda: calls.append(("event_window", None)) or {"after_sequence": 41},
        motor_move_relative=lambda *args, **kwargs: calls.append(("move_relative", args)) or {"ok": True, "ack": ACK},
    )
    adapter = subject.Serial206ProductionPrimitiveAdapter.__new__(subject.Serial206ProductionPrimitiveAdapter)
    adapter.tester = tester
    adapter._z_profile = lambda: {"board": 4, "motor": 1}
    adapter._z_finalize_position_move = lambda **kwargs: calls.append(("finalize", kwargs)) or {"ok": True}
    return adapter, calls


def test_critical_receipt_evidence_survives_deep_result_compaction():
    receipt = {
        "command_id": "operator-test",
        "intent": "move_steps",
        "status": "failed",
        "started_at": 120.0,
        "finished_at": 123.0,
        "controller_command_acknowledged": True,
        "controller_terminal_state_verified": True,
        "result_summary": {"ok": False, "failure": "z_target_event_128_missing_or_stale"},
        "result": {
            "failure": "z_target_event_128_missing_or_stale",
            "before_position_steps": 10_000,
            "target_position_steps": 20,
            "after_position_steps": 13,
            "target_events": [],
            "controller_error_events": [{"status": 130, "event_sequence": 77}],
            "wait": {"stopped": True, "last_speed": 0},
            "noise": [{str(index): list(range(100))} for index in range(100)],
        },
    }
    z = {"receipts": []}

    provider = subject.Serial206OemInitializationProvider(object())
    provider._append_z_receipt(z, receipt)

    stored = z["receipts"][0]
    assert stored["status"] == "failed"
    assert stored["started_at"] == 120.0
    assert stored["finished_at"] == 123.0
    assert stored["controller_command_acknowledged"] is True
    assert stored["controller_terminal_state_verified"] is True
    assert stored["critical_evidence"] == {
        "failure": "z_target_event_128_missing_or_stale",
        "before_position_steps": 10_000,
        "target_position_steps": 20,
        "after_position_steps": 13,
        "terminal_speed_steps_s": 0,
        "terminal_stopped": True,
        "target_events": [],
        "controller_error_events": [{"status": 130, "event_sequence": 77}],
    }


def test_partial_compacted_terminal_state_cannot_overwrite_live_register_projection():
    result = {
        "terminal_z_state": {
            "ok": True,
            "position_steps": 0,
            "speed_steps_s": {"omitted": "item_limit"},
        },
    }

    assert subject.Serial206OemInitializationProvider._z_terminal_state_from_result(
        result,
        command_id="operator-home",
        observed_at=124.0,
    ) is None


def test_terminal_projection_converts_compaction_markers_to_unknown_values():
    terminal = subject.Serial206OemInitializationProvider._sanitize_z_terminal_state({
        "authority": "serial206_terminal_register_readback",
        "position_steps": 0,
        "speed_steps_s": {"omitted": "item_limit"},
        "left_switch_state": 1,
        "right_switch_state": {"omitted": "item_limit"},
        "left_switch_disabled": False,
        "right_switch_disabled": {"omitted": "item_limit"},
    })

    assert terminal["position_steps"] == 0
    assert terminal["speed_steps_s"] is None
    assert terminal["left_switch_state"] == 1
    assert terminal["right_switch_state"] is None
    assert terminal["left_switch_disabled"] is False
    assert terminal["right_switch_disabled"] is None


def test_mutation_priority_lock_admits_waiting_command_before_later_reader():
    lock = subject._MutationPriorityRLock()
    first_reader_entered = threading.Event()
    release_first_reader = threading.Event()
    order: list[str] = []

    def first_reader():
        with lock:
            first_reader_entered.set()
            release_first_reader.wait(1.0)
            order.append("first_reader")

    def mutation():
        with lock.mutation():
            with lock:
                order.append("mutation")

    def later_reader():
        with lock:
            order.append("later_reader")

    first = threading.Thread(target=first_reader)
    command = threading.Thread(target=mutation)
    later = threading.Thread(target=later_reader)
    first.start()
    assert first_reader_entered.wait(1.0)
    command.start()
    deadline = time.monotonic() + 1.0
    while lock._mutation_waiters != 1 and time.monotonic() < deadline:
        time.sleep(0.001)
    assert lock._mutation_waiters == 1
    later.start()
    release_first_reader.set()
    for thread in (first, command, later):
        thread.join(1.0)
        assert thread.is_alive() is False

    assert order == ["first_reader", "mutation", "later_reader"]


def test_command_capability_check_does_not_build_full_provider_status(monkeypatch):
    from bioxp import api

    class Provider:
        def capability_status(self):
            return {"initialize_motors_live_available": True}

        def z_projection(self):
            raise AssertionError("command admission must not build full provider status")

    provider = Provider()
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)

    assert api._require_serial206_oem_initialization_provider() is provider