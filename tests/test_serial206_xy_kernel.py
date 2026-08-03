from __future__ import annotations

from types import SimpleNamespace

from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter


class FakeReferenceStore:
    def __init__(self):
        self.motion_many = []
        self.motion_single = []

    def snapshot(self, axes):
        return {"ok": True, "durable_clean": True, "rows": {axis: {"state": "referenced"} for axis in axes}}

    def record_motion_many(self, axes, motion_kind):
        self.motion_many.append((tuple(axes), motion_kind))
        return {"ok": True, "durable_clean": True, "rows": {axis: {"state": "referenced"} for axis in axes}}

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
        self.fail_acceleration = fail_acceleration

    def _motion_oem_axis_profile(self, axis, startup=True):
        return {
            "x": {"board": 5, "motor": 0, "axis_min_steps": 0, "axis_max_steps": 90263},
            "y": {"board": 4, "motor": 0, "axis_min_steps": 0, "axis_max_steps": 102956},
        }[axis]

    def motor_get_position(self, board, motor=0):
        return {"ok": True, "position": self.positions[int(board)]}

    def motor_get_speed(self, board, motor=0):
        return {"ok": True, "speed": 0}

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.acceleration_writes.append((int(board), int(param), int(value), int(motor)))
        readback = int(value) + 1 if self.fail_acceleration and int(board) == 5 and int(value) in {350, 400} else int(value)
        return {"ok": True, "readback": {"value": readback}}

    def begin_bus_event_window(self):
        token = {"after_sequence": len(self.windows)}
        self.windows.append(token)
        return token

    def motor_oem_move_absolute(self, board, position, motor=0, wait_for_stop=False, max_position=None):
        self.moves.append((int(board), int(position), bool(wait_for_stop)))
        self.positions[int(board)] = int(position)
        return {"ok": True, "board": int(board), "motor": int(motor), "position": int(position)}

    def motor_wait_target_reached_many(self, targets, event_window=None, timeout_s=5.0, sta_sequential=False):
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
    return primitive


def test_move_xy_uses_oem_pair_acceleration_order_and_atomic_metadata():
    primitive = adapter()
    receipt = primitive.move_xy(12000, 6000, wait_timeout_s=5.0)

    assert receipt["ok"] is True
    assert receipt["branch"] == "parallel"
    assert receipt["launch_order"] == ["x", "y"]
    assert receipt["acceleration_selected"] == {"x": 400, "y": 400}
    assert receipt["stagger_ms"] == 100
    assert receipt["pre_wait_sleep_ms"] == 5
    assert primitive.tester.moves == [(5, 12000, False), (4, 6000, False)]
    assert primitive.reference_store.motion_many == [(("x", "y"), "move_xy")]
    assert primitive.reference_store.motion_single == []
    assert primitive.tester.acceleration_writes[-2:] == [(5, 5, 350, 0), (4, 5, 400, 0)]


def test_move_xy_near_axis_keeps_both_targets_live_without_boost():
    primitive = adapter()
    receipt = primitive.move_xy(100, 10, wait_timeout_s=5.0)

    assert receipt["ok"] is True
    assert receipt["branch"] == "near_axis_sequential"
    assert receipt["launch_order"] == ["x", "y"]
    assert primitive.tester.moves == [(5, 100, False), (4, 10, False)]
    assert (5, 5, 400, 0) not in primitive.tester.acceleration_writes
    assert (4, 5, 750, 0) not in primitive.tester.acceleration_writes


def test_move_xy_acceleration_readback_failure_delivers_no_move():
    primitive = adapter(fail_acceleration=True)
    receipt = primitive.move_xy(12000, 6000, wait_timeout_s=5.0)

    assert receipt["ok"] is False
    assert receipt["failure"] == "moveXY_acceleration_setup_not_verified"
    assert receipt["command_issued"] is False
    assert primitive.tester.moves == []
    assert primitive.reference_store.motion_many == []


def test_move_xy_longer_y_axis_launches_y_first_with_integer_stagger():
    primitive = adapter()
    receipt = primitive.move_xy(6000, 12000, wait_timeout_s=5.0)

    assert receipt["ok"] is True
    assert receipt["launch_order"] == ["y", "x"]
    assert receipt["stagger_ms"] == 100
    assert primitive.tester.moves == [(4, 12000, False), (5, 6000, False)]


class LifecyclePrimitives:
    def x_move_absolute(self, **kwargs):
        return {"ok": True, "pending_motion": True, "command_issued": True, "target_position_steps": kwargs["position_steps"]}

    def x_wait_for_motor(self, **kwargs):
        return {"ok": True, "target_position_verified": True}


def test_x_lifecycle_persists_pending_ticket_and_latches_reentry():
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    provider = Serial206OemInitializationProvider(LifecyclePrimitives(), generation_provider=lambda: 17)
    pending = provider.execute_x_intent("move_absolute", {"position_steps": 1000, "wait_for_stop": False, "command_id": "x-pending-001"})
    assert pending["ok"] is True
    assert pending["state"] == "executing"
    assert provider.x_projection()["lifecycle"]["pending_ticket"]["pending_motion"] is True

    reentry = provider.execute_x_intent("move_absolute", {"position_steps": 1200, "command_id": "x-reentry-002"})
    assert reentry["ok"] is False
    assert reentry["failure"] == "x_executing_outcome_ambiguous"
    assert provider.x_projection()["lifecycle"]["state"] == "failed_latched"
