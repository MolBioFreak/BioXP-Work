"""Arrival: real moveTo/driver -> existing API binding -> SQLite updates.

Only controller leaves are replaced; reuse the existing offline pose/board rigs.
"""
import threading

import pytest

from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.usb_driver import BioXpTester
from tests.test_deck_complete_effective import BoardLeaf
from tests.test_native_pose_observations import connected, no_hardware


class ArrivalBoard(BoardLeaf):
    query_only_tmcl = BioXpTester.query_only_tmcl
    motor_get_position = BioXpTester.motor_get_position
    set_axis_position_observer = BioXpTester.set_axis_position_observer
    _publish_axis_position = BioXpTester._publish_axis_position
    MOTOR_AXIS_PRESETS = BioXpTester.MOTOR_AXIS_PRESETS
    BOARD_CHILLER = BioXpTester.BOARD_CHILLER

    def __init__(self, before, fault=None):
        super().__init__(before, fault)
        self.queries = []
        self.writes = []
        self.arrival_reply = None
        self.at_arrival = None

    def motor_get_axis_param(self, board, param, motor=0):
        assert param == 1
        value = self.positions[board, motor]
        return {"ack": {"status": 100, "value": value}, "value": value}

    def motor_set_axis_param(self, *args, **kwargs):
        self.writes.append((args, kwargs))
        return super().motor_set_axis_param(*args, **kwargs)

    def _send_motor(self, board, command, typ, motor, value, **kwargs):
        if (command, typ) == (6, 1):
            self.queries.append((board, motor, kwargs, threading.get_ident()))
            if self.at_arrival:
                self.at_arrival(board, motor)
            if self.arrival_reply:
                return self.arrival_reply(board, motor)
            return {"status": 100, "value": self.positions[board, motor]}
        return super()._send_motor(board, command, typ, motor, value, **kwargs)


def rig(connected, monkeypatch, before=(90213, 93211), fault=None):
    _, store, hardware, bind, namespace = connected
    leaf = ArrivalBoard(before, fault)
    namespace["_tester"] = leaf
    bind(epoch=hardware.ownership_epoch, owned=True)
    adapter = Serial206ProductionPrimitiveAdapter(
        leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: hardware.ownership_epoch)
    monkeypatch.setattr("bioxp.oem_serial206_initialization.time.sleep", lambda _: None)
    return leaf, adapter, store


def move(adapter, target=(26213, 42413), **kwargs):
    return adapter.oem_move_to(*target, 0, pseudo_home_steps=500,
        gripper_confirmed=kwargs.pop("gripper_confirmed", False),
        tip_loaded=False, source_context="ClassControlInterface.btnLOC1_Click", **kwargs)


def axes(store):
    return {row["axis"]: row for row in store._updates_snapshot(None, None)["pose"]["axes"]}


@pytest.mark.parametrize("before,target,options,branch", [
    ((90213, 93211), (26213, 42413), {}, "descending_y_parallel_x_first"),
    ((90213, 93211), (26213, 42413), {"run_in_parallel": False}, "descending_y_sequential_x_first"),
    ((26213, 42413), (90213, 93211), {}, "parallel_y_first"),
    ((26213, 42413), (90213, 93211), {"run_in_parallel": False}, "sequential_y_first"),
    ((90213, 93211), (26213, 42413), {"plate_on_gantry": 4, "location19_y": 50000}, "descending_y_loaded_plate"),
    ((90213, 93211), (26213, 42413), {"gripper_confirmed": True}, "confirmed_gripper_no_tip_moveXY"),
    ((26213, 42413), (26213, 42413), {}, "descending_y_parallel_x_first"),
    ((26213, 93211), (26213, 42413), {}, "descending_y_parallel_x_first"),
    ((90213, 42413), (26213, 42413), {}, "descending_y_parallel_x_first"),
])
def test_arrival_once_after_children_on_command_owner(connected, monkeypatch, before, target, options, branch):
    leaf, adapter, store = rig(connected, monkeypatch, before)
    owner = threading.get_ident()
    def settled(board, motor):
        assert leaf.positions == {(5, 0): target[0], (4, 0): target[1], (4, 1): 0}
    leaf.at_arrival = settled
    result = move(adapter, target, **options)
    assert result["ok"] and result["source_return_code"] == 0
    assert result["controller_completion_verified"]
    assert result["branch"] == branch
    assert [(b, m) for b, m, _, _ in leaf.queries] == [(5, 0), (4, 0), (4, 1)]
    for _, _, bounds, thread in leaf.queries:
        assert thread == owner
        assert bounds == dict(attempts=1, wait_reply=True, write_timeout_ms=60,
            read_timeout_ms=90, max_reads=28, strict_match=True, allow_recover=False)
    assert {a: row["position_steps"] for a, row in axes(store).items()} == dict(x=target[0], y=target[1], z=0)
    assert connected[2].completed_snapshot() is None
    assert sorted(leaf.moves) == sorted((b, m, t) for b, m, t, start in
        [(5, 0, target[0], before[0]), (4, 0, target[1], before[1])] if t != start)


@pytest.mark.parametrize("failure", ["query", "callback", "null_cached", "invalid", "malformed"])
def test_observation_failure_keeps_outcome_and_next_move(connected, monkeypatch, failure):
    leaf, adapter, store = rig(connected, monkeypatch)
    original_publish = store.publish_axis_observation
    snapshot = []
    def fail(*args, **kwargs):
        raise RuntimeError("display observation failed")
    def arrival(board, motor):
        if not snapshot:
            snapshot.append(store._updates_snapshot(None, None))
            if failure == "callback":
                monkeypatch.setattr(store, "publish_axis_observation", fail)
    leaf.at_arrival = arrival
    if failure == "query":
        leaf.arrival_reply = fail
    elif failure == "null_cached":
        leaf._oem_position_cache = {(5, 0): 999, (4, 0): 888, (4, 1): 777}
        leaf.arrival_reply = lambda *args: None
    elif failure == "invalid":
        leaf.arrival_reply = lambda *args: {"status": 2, "value": 999}
    elif failure == "malformed":
        leaf.arrival_reply = lambda *args: {"status": 100, "value": True}
    first = move(adapter)
    assert first["ok"] and first["controller_completion_verified"]
    assert first["source_return_code"] == 0
    assert store._updates_snapshot(None, None) == snapshot[0]
    assert len(leaf.queries) == 3
    # Restore just the display sink/replies, not movement or controller state.
    leaf.arrival_reply = leaf.at_arrival = None
    monkeypatch.setattr(store, "publish_axis_observation", original_publish)
    connected[3](epoch=connected[2].ownership_epoch, owned=True)
    second = move(adapter, (90213, 93211))
    assert second["ok"] and second["controller_completion_verified"]
    assert len(leaf.queries) == 6
    assert sorted(leaf.moves) == sorted([(5, 0, 26213), (4, 0, 42413), (5, 0, 90213), (4, 0, 93211)])
    assert {a: row["position_steps"] for a, row in axes(store).items()} == dict(x=90213, y=93211, z=0)


def test_one_axis_failure_does_not_refresh_it_or_hide_other_axes(connected, monkeypatch):
    leaf, adapter, store = rig(connected, monkeypatch)
    prior = []
    def reply(board, motor):
        if not prior:
            prior.append(axes(store))
        return None if board == 5 else {"status": 100, "value": leaf.positions[board, motor]}
    leaf.arrival_reply = reply
    assert move(adapter)["ok"]
    current = axes(store)
    assert current["x"] == prior[0]["x"]
    assert current["y"]["position_steps"] == 42413
    assert current["y"]["observed_at"] >= prior[0]["y"]["observed_at"]


def test_epoch_replacement_during_arrival_drops_old_reply(connected, monkeypatch):
    leaf, adapter, store = rig(connected, monkeypatch)
    def replace(board, motor):
        epoch = connected[2].change_ownership(reason="offline replacement")
        connected[3](epoch=epoch, owned=False)
    leaf.at_arrival = replace
    assert move(adapter)["ok"]
    assert store._updates_snapshot(None, None)["pose"] is None


@pytest.mark.parametrize("fault", ["missing_ack", "missing_event"])
def test_failed_motion_unchanged_no_arrival_queries(connected, monkeypatch, fault):
    leaf, adapter, _ = rig(connected, monkeypatch, fault=fault)
    try:
        result = move(adapter)
    except Exception as exc:
        assert getattr(exc, "motion_evidence", None) is not None
    else:
        assert not result["controller_completion_verified"]
    assert not leaf.queries


def test_stop_at_completion_keeps_existing_result_without_queries(connected, monkeypatch):
    leaf, adapter, _ = rig(connected, monkeypatch)
    result = move(adapter, interrupt_reason=lambda: "operator_stop" if len(leaf.moves) == 2 else None)
    assert result["failure"] == "operator_stop"
    assert result["ok"] is False and result["source_return_code"] == 1
    assert not leaf.queries


def test_final_z_arrival_is_not_pseudo_home(connected, monkeypatch):
    leaf, adapter, store = rig(connected, monkeypatch)
    leaf.positions[4, 1] = 1000
    result = adapter.oem_move_to(26213, 42413, 2000, pseudo_home_steps=500,
        gripper_confirmed=False, tip_loaded=False)
    assert result["ok"] and result["controller_completion_verified"]
    assert [(b, m) for b, m, _, _ in leaf.queries] == [(5, 0), (4, 0), (4, 1)]
    assert axes(store)["z"]["position_steps"] == 2000
    assert [t for b, m, t in leaf.moves if (b, m) == (4, 1)] == [500, 2000]


@pytest.mark.parametrize("confirmed", [False, True])
def test_pre_repair_method_same_outcome_and_nonquery_writes(connected, monkeypatch, confirmed):
    # Targeted-method replay, not a full historical checkout.
    import ast
    import subprocess
    from pathlib import Path
    from types import MethodType
    import bioxp.oem_serial206_initialization as native

    try:
        source = subprocess.check_output([
            "git", "show", "fafbb1341583e5a660ff2bbc941c0226f8239ccc:src/bioxp/oem_serial206_initialization.py"],
            cwd=Path(__file__).parents[1], text=True, stderr=subprocess.PIPE)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pytest.skip("historical method replay needs local Git history; arrival tests do not")
    cls = next(n for n in ast.parse(source).body
        if isinstance(n, ast.ClassDef) and n.name == "Serial206ProductionPrimitiveAdapter")
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "oem_move_to")
    namespace = dict(vars(native))
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "baseline-moveTo", "exec"), namespace)
    baseline_leaf, baseline_adapter, store = rig(connected, monkeypatch)
    baseline_adapter.oem_move_to = MethodType(namespace["oem_move_to"], baseline_adapter)
    before = move(baseline_adapter, gripper_confirmed=confirmed)
    assert not baseline_leaf.queries
    if not confirmed:
        assert axes(store)["x"]["position_steps"] == 90213
        assert axes(store)["y"]["position_steps"] == 93211
    leaf, adapter, store = rig(connected, monkeypatch)
    after = move(adapter, gripper_confirmed=confirmed)

    def stable(value):
        if isinstance(value, dict):
            return {k: stable(v) for k, v in value.items()
                    if k not in {"elapsed_ms", "elapsed_s"}}
        if isinstance(value, list):
            return [stable(v) for v in value]
        return value

    assert stable(after) == stable(before)
    assert leaf.moves == baseline_leaf.moves
    assert leaf.writes == baseline_leaf.writes
    assert len(leaf.queries) == 3
    assert axes(store)["x"]["position_steps"] == 26213
    assert axes(store)["y"]["position_steps"] == 42413
