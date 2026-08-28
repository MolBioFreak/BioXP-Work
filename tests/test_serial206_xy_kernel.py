from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.oem_runtime_store import OEMRuntimeStore


class FakeReferenceStore:
    def __init__(self):
        self.motion_many = []
        self.motion_single = []

    def snapshot(self, axes):
        return {"ok": True, "durable_clean": True, "rows": {axis: {"state": "referenced"} for axis in axes}}

    def record_motion_many(self, motions):
        rows = tuple(motions)
        self.motion_many.append(rows)
        return {"ok": True, "durable_clean": True, "rows": {axis: {"state": "referenced"} for axis, _motion_kind in rows}}

    def record_motion(self, axis, motion_kind):
        self.motion_single.append((axis, motion_kind))
        return {"ok": True, "durable_clean": True, "axis": axis, "state": "referenced"}

    def mark_desynced_many(self, commands):
        return {"ok": True, "durable_clean": True, "commands": list(commands)}

    def mark_referenced_many(self, commands):
        return {"ok": True, "durable_clean": True, "rows": {command.axis: {"state": "referenced"} for command in commands}}


class FakeTester:
    def __init__(self, *, fail_acceleration: bool = False):
        self.positions = {5: 0, 4: 0}
        self.acceleration_writes = []
        self.moves = []
        self.windows = []
        self.wait_windows = []
        self.fail_acceleration = fail_acceleration
        self.events = [
            {"event_sequence": 1, "status": 128, "board": 5, "motor": 0},
            {"event_sequence": 1, "status": 128, "board": 4, "motor": 0},
        ]

    def _motion_oem_axis_profile(self, axis, startup=True):
        return {
            "x": {"board": 5, "motor": 0, "axis_min_steps": 0, "axis_max_steps": 90263},
            "y": {"board": 4, "motor": 0, "axis_min_steps": 0, "axis_max_steps": 102956},
        }[axis]

    def motor_get_position(self, board, motor=0):
        return {"ok": True, "ack": {"status": 100}, "position": self.positions[int(board)]}

    def motor_get_speed(self, board, motor=0):
        return {"ok": True, "ack": {"status": 100}, "speed": 0}

    def oem_no24v_state(self):
        return False

    def _oem_board_state(self):
        return {4: True, 5: True}

    def motor_get_axis_param(self, board, param, motor=0):
        values = {(5, 9): 0, (5, 10): 0, (5, 12): 0, (5, 13): 0}
        return {"ok": True, "ack": {"status": 100}, "value": values[(int(board), int(param))]}

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.acceleration_writes.append((int(board), int(param), int(value), int(motor)))
        readback = int(value) + 1 if self.fail_acceleration and int(board) == 5 and int(value) in {350, 400} else int(value)
        return {"ok": True, "ack": {"status": 100}, "readback": {"ok": True, "ack": {"status": 100}, "value": readback}}

    def begin_bus_event_window(self):
        token = {"after_sequence": 0}
        self.windows.append(token)
        return token

    def motor_oem_move_absolute(self, board, position, motor=0, wait_for_stop=False, max_position=None):
        self.moves.append((int(board), int(position), bool(wait_for_stop)))
        self.positions[int(board)] = int(position)
        cursor = 10.0 if int(board) == 5 else 20.0
        return {"ok": True, "ack": {"status": 100, "provenance": {"tx_write_completed_at": cursor}}, "event_window": {"after_sequence": 0, "dispatch_cursors": {f"{int(board)}:{int(motor)}": cursor}}, "board": int(board), "motor": int(motor), "position": int(position)}

    @staticmethod
    def _bind_event_dispatch_cursor(event_window, board, motor, ack):
        bound = dict(event_window)
        cursors = dict(bound.get("dispatch_cursors") or {})
        cursor = ((ack or {}).get("provenance") or {}).get("tx_write_completed_at")
        if cursor is not None:
            cursors[f"{int(board)}:{int(motor)}"] = float(cursor)
        bound["dispatch_cursors"] = cursors
        return bound

    def collect_bus_events(self, **_kwargs):
        return list(self.events)

    def motor_wait_target_reached_many(self, targets, event_window=None, timeout_s=5.0, sta_sequential=False):
        self.wait_windows.append(dict(event_window or {}))
        return {
            "ok": True,
            "per_axis": {
                "x": {"ok": True, "target_reached": True},
                "y": {"ok": True, "target_reached": True},
            },
        }

    def motor_wait_target_reached(self, board, motor=0, timeout_s=5.0, event_window=None):
        return {"ok": True, "target_reached": True}

    def motor_oem_stop_exact(self, board, motor=0):
        return {"ok": True, "board": int(board), "motor": int(motor)}


def adapter(*, fail_acceleration=False, reference=True):
    primitive = Serial206ProductionPrimitiveAdapter.__new__(Serial206ProductionPrimitiveAdapter)
    primitive.tester = FakeTester(fail_acceleration=fail_acceleration)
    primitive.reference_store = FakeReferenceStore() if reference else None
    primitive._x_profile_overrides = {}

    def move_y_absolute(**kwargs):
        primitive.tester.moves.append((4, int(kwargs["target_steps"]), False))
        primitive.tester.positions[4] = int(kwargs["target_steps"])
        return {
            "ok": True,
            "command_issued": True,
            "physical_motion_commanded": True,
            "controller_command_acknowledged": True,
        }

    primitive.y_provider = SimpleNamespace(move_absolute=move_y_absolute)
    return primitive


class LifecyclePrimitives:
    @staticmethod
    def current_board_lifecycle_generation():
        return 9

    def x_move_absolute(self, **kwargs):
        return {"ok": True, "pending_motion": True, "command_issued": True, "target_position_steps": kwargs["position_steps"]}

    def x_wait_for_motor(self, **kwargs):
        return {"ok": True, "target_position_verified": True}


class ImmediateXPrimitives:
    def __init__(self):
        self.dispatches = 0

    @staticmethod
    def current_board_lifecycle_generation():
        return 9

    @staticmethod
    def _x_require_motion_preflight():
        return {"profile": {"axis": "x"}, "switch_masks": {12: 1, 13: 0}}

    def x_move_absolute(self, **_kwargs):
        self.dispatches += 1
        return {"ok": True, "command_issued": True, "target_position_verified": True, "controller_command_acknowledged": True, "controller_terminal_state_verified": True}

    def x_stop(self, **_kwargs):
        self.dispatches += 1
        return {"ok": True, "command_issued": True, "stopped": True}

    def home_xy(self, **_kwargs):
        self.dispatches += 1
        return {"ok": True, "command_issued": True, "homed": True, "controller_terminal_state_verified": True, "reference_publication_required": True}

    def move_xy(self, _x, _y, **_kwargs):
        self.dispatches += 1
        return {"ok": True, "command_issued": True, "target_position_verified": True, "controller_terminal_state_verified": True, "acceleration_restore_verified": True}


def seed_ready(provider, *, referenced=True):
    state = provider._load_state()
    state["x_lifecycle"].update({
        "state": "referenced_ready" if referenced else "prepared_unreferenced",
        "generation": 17,
        "board_lifecycle_generation": 9,
        "reference_state": "referenced" if referenced else "desynced",
        "prepared_receipt": {"ok": True, "board_lifecycle_generation": 9},
    })
    provider._save_state(state)
    return provider


class BlockingXPrimitives(ImmediateXPrimitives):
    def __init__(self):
        super().__init__()
        self.move_entered = threading.Event()
        self.release_move = threading.Event()
        self.stop_dispatched = threading.Event()

    def x_move_absolute(self, **_kwargs):
        self.dispatches += 1
        self.move_entered.set()
        assert self.release_move.wait(timeout=2.0)
        return {"ok": True, "command_issued": True}

    def x_stop(self, **_kwargs):
        self.dispatches += 1
        self.stop_dispatched.set()
        return {"ok": True, "command_issued": True, "stopped": True}

    def move_xy(self, _x, _y, **_kwargs):
        self.dispatches += 1
        self.move_entered.set()
        assert self.release_move.wait(timeout=2.0)
        return {"ok": True, "command_issued": True}

    def home_xy(self, **_kwargs):
        self.dispatches += 1
        self.move_entered.set()
        assert self.release_move.wait(timeout=2.0)
        return {"ok": True, "command_issued": True, "homed": True}


def test_x_lifecycle_persists_pending_ticket_and_latches_reentry():
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    provider = seed_ready(Serial206OemInitializationProvider(LifecyclePrimitives(), generation_provider=lambda: 17))
    pending = provider.execute_x_intent("move_absolute", {"position_steps": 1000, "wait_for_stop": False, "command_id": "x-pending-001"})
    assert pending["ok"] is True
    assert pending["state"] == "executing"
    assert provider.x_projection()["lifecycle"]["pending_ticket"]["pending_motion"] is True

    reentry = provider.execute_x_intent("move_absolute", {"position_steps": 1200, "command_id": "x-reentry-002"})
    assert reentry["ok"] is False
    assert reentry["failure"] == "x_executing_outcome_ambiguous"
    assert provider.x_projection()["lifecycle"]["state"] == "failed_latched"


def test_x_terminal_authority_is_saved_before_sql_receipt(tmp_path, monkeypatch):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    events: list[tuple[str, str]] = []
    real_write = store.write_oem_serial206_initialization_state
    real_append = store.append_serial206_receipt

    def write(state):
        events.append(("authority", str(state["x_lifecycle"]["state"])))
        return real_write(state)

    def append(stream, row):
        events.append(("sql", str(row["status"])))
        return real_append(stream, row)

    monkeypatch.setattr(store, "write_oem_serial206_initialization_state", write)
    monkeypatch.setattr(store, "append_serial206_receipt", append)
    provider = Serial206OemInitializationProvider(
        ImmediateXPrimitives(),
        state_store=store,
        generation_provider=lambda: 17,
    )
    seed_ready(provider)

    result = provider.execute_x_intent(
        "move_absolute",
        {"position_steps": 1000, "command_id": "x-order"},
    )

    assert result["ok"] is True
    assert events[-2:] == [("authority", "referenced_ready"), ("sql", "completed")]


def test_x_sql_failure_after_authority_save_replays_without_redispatch(tmp_path, monkeypatch):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    primitives = ImmediateXPrimitives()
    provider = seed_ready(Serial206OemInitializationProvider(primitives, state_store=store, generation_provider=lambda: 17))
    real_append = store.append_serial206_receipt
    monkeypatch.setattr(store, "append_serial206_receipt", lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("sql failed")))
    values = {"position_steps": 1000, "command_id": "x-sql-failure", "idempotency_key": "x-sql-key"}
    with pytest.raises(OSError, match="sql failed"):
        provider.execute_x_intent("move_absolute", values)
    assert primitives.dispatches == 1

    monkeypatch.setattr(store, "append_serial206_receipt", real_append)
    restarted = Serial206OemInitializationProvider(primitives, state_store=store, generation_provider=lambda: 17)
    replay = restarted.execute_x_intent("move_absolute", values)
    assert replay["ok"] is True
    assert replay["replayed"] is True
    assert primitives.dispatches == 1


def test_homexy_terminal_receipt_replays_without_physical_redispatch(tmp_path):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    primitives = ImmediateXPrimitives()
    provider = seed_ready(Serial206OemInitializationProvider(primitives, state_store=store, generation_provider=lambda: 17))
    values = {"command_id": "homexy-once", "idempotency_key": "homexy-key"}
    assert provider.execute_homexy_intent(values)["ok"] is True
    replay = provider.execute_homexy_intent(values)
    assert replay["ok"] is True
    assert replay["replayed"] is True
    assert primitives.dispatches == 1


def test_x_stop_uses_interrupt_lane_and_reused_command_id_still_dispatches(tmp_path, monkeypatch):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    primitives = ImmediateXPrimitives()
    calls: list[str] = []
    real_interrupt = store.append_serial206_interrupt_receipt

    def append_interrupt(stream, row):
        calls.append(str(row["receipt_id"]))
        return real_interrupt(stream, row)

    monkeypatch.setattr(store, "append_serial206_interrupt_receipt", append_interrupt)
    provider = seed_ready(Serial206OemInitializationProvider(primitives, state_store=store, generation_provider=lambda: 17))
    values = {"command_id": "reused-x-stop", "idempotency_key": "reused-x-stop-key"}
    assert provider.execute_x_intent("stop", values)["ok"] is True
    assert provider.execute_x_intent("stop", values)["ok"] is True
    assert primitives.dispatches == 2
    assert len(calls) == 2
    assert calls[0] != calls[1]
    assert len([row for row in store.list_serial206_receipts("x", 20) if row["command_id"] == "reused-x-stop"]) == 2


def test_x_stop_dispatches_while_move_holds_lifecycle_lock(tmp_path):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    primitives = BlockingXPrimitives()
    provider = seed_ready(Serial206OemInitializationProvider(primitives, state_store=store, generation_provider=lambda: 17))
    outcomes = {}

    move_thread = threading.Thread(
        target=lambda: outcomes.setdefault(
            "move",
            provider.execute_x_intent("move_absolute", {"position_steps": 2000, "command_id": "blocking-x-move"}),
        )
    )
    stop_thread = threading.Thread(
        target=lambda: outcomes.setdefault(
            "stop",
            provider.execute_x_intent("stop", {"command_id": "preemptive-x-stop"}),
        )
    )
    move_thread.start()
    assert primitives.move_entered.wait(timeout=1.0)
    stop_thread.start()
    assert primitives.stop_dispatched.wait(timeout=1.0)
    assert move_thread.is_alive()
    primitives.release_move.set()
    move_thread.join(timeout=2.0)
    stop_thread.join(timeout=2.0)

    assert outcomes["move"]["ok"] is False
    assert outcomes["move"]["result"]["failure"] == "x_intent_interrupted_by_safety_command"
    assert outcomes["stop"]["ok"] is True


def test_x_generation_change_during_dispatch_cannot_publish_success(tmp_path):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    primitives = BlockingXPrimitives()
    generation = {"value": 17}
    provider = Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        generation_provider=lambda: generation["value"],
    )
    seed_ready(provider)
    outcome = {}
    worker = threading.Thread(
        target=lambda: outcome.setdefault(
            "result",
            provider.execute_x_intent("move_absolute", {"command_id": "x-generation-drift", "position_steps": 42}),
        )
    )

    worker.start()
    assert primitives.move_entered.wait(timeout=1.0)
    generation["value"] = 18
    primitives.release_move.set()
    worker.join(timeout=2.0)

    assert outcome["result"]["ok"] is False
    assert outcome["result"]["result"]["failure"] == "x_generation_changed_during_command"
    assert provider._load_state()["x_lifecycle"]["state"] == "failed_latched"


def test_x_projection_invalidates_stale_generation_authority(tmp_path):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    primitives = ImmediateXPrimitives()
    generation = {"value": 17}
    provider = Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        generation_provider=lambda: generation["value"],
    )
    seed_ready(provider)
    assert provider.execute_x_intent("move_absolute", {"command_id": "x-projection-drift", "position_steps": 42})["ok"] is True

    generation["value"] = 18
    projection = provider.x_projection()

    assert projection["lifecycle"]["state"] == "unprepared"
    assert projection["lifecycle"]["last_failure"]["failure"] == "x_generation_changed"
    assert provider._load_state()["x_lifecycle"]["state"] == "unprepared"


@pytest.mark.parametrize("operation", ["xy", "homexy"])
def test_xy_and_homexy_exceptions_persist_failed_latched_authority(tmp_path, operation):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    primitives = ImmediateXPrimitives()
    if operation == "xy":
        primitives.move_xy = lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("injected xy failure"))
    else:
        primitives.home_xy = lambda **_kwargs: (_ for _ in ()).throw(RuntimeError("injected homexy failure"))
    provider = seed_ready(Serial206OemInitializationProvider(primitives, state_store=store, generation_provider=lambda: 17))

    result = (
        provider.execute_xy_intent(10, 20, {"command_id": "xy-exception"})
        if operation == "xy"
        else provider.execute_homexy_intent({"command_id": "homexy-exception"})
    )

    assert result["ok"] is False
    assert result["state"] == "failed_latched"
    durable = provider._load_state()["x_lifecycle"]
    assert durable["state"] == "failed_latched"
    assert durable["reference_state"] == "desynced"
    assert durable["active_receipt"] is None


def test_xy_reused_command_id_cannot_replay_different_target(tmp_path):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    store = OEMRuntimeStore(tmp_path)
    primitives = ImmediateXPrimitives()
    provider = seed_ready(Serial206OemInitializationProvider(primitives, state_store=store, generation_provider=lambda: 17))
    values = {"command_id": "xy-bound-target", "speed": 100}

    assert provider.execute_xy_intent(10, 20, values)["ok"] is True
    replay = provider.execute_xy_intent(11, 20, values)

    assert replay["ok"] is False
    assert replay["failure"] == "xy_replay_current_authority_or_request_invalid"
    assert primitives.dispatches == 1