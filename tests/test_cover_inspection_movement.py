"""Offline connected inspection -> moveTo -> moveXY -> native-axis seams.

SSD BioXPControlLib:7726-7744,8355-8504,22215-22312. No production tester,
transport, adapter constructor or runtime writer is instantiated. Only state
readers and the native tester boundary are fixtures; routing, X/Y issue,
completion aggregation, offset math and gripper observation remain real.
"""
from types import SimpleNamespace

import pytest

import bioxp.oem_serial206_initialization as mod
from bioxp.oem_compat.position_table import PositionTarget


class OfflineNative:
    addresses = {"x": (5, 0), "y": (4, 0), "z": (4, 1)}

    def __init__(self):
        self.positions = {(5, 0): 1000, (4, 0): 2000, (4, 1): 500, (4, 2): 100}
        self.calls = []
        self.moves = []
        self.waits = []
        self.home = True
        self.failure = None
        self.exception = None
        self.window = {"after_sequence": 0, "receive_owner": "offline", "owner_generation": 1}

    def _motion_oem_axis_profile(self, axis, *, startup):
        board, motor = self.addresses[axis]
        return {"board": board, "motor": motor, "axis_min_steps": 0,
                "axis_max_steps": 90263 if axis == "x" else 102956}

    def _machine_config_axis_max(self, axis):
        assert axis == "x"
        return 90263, "offline"

    def _oem_board_present(self, board):
        assert board in (4, 5)
        return True

    def motor_oem_axis_board_present(self, axis):
        assert axis in ("x", "y")
        return True

    def oem_no24v_state(self):
        return False

    def _oem_board_state(self):
        return {4: True, 5: True}

    def motor_query_home_switch(self, board, *, motor):
        assert (board, motor) == (4, 2)
        self.calls.append(("gripper",))
        return {"reply_valid": True, "home": self.home}

    def motor_get_position(self, board, *, motor):
        value = self.positions[board, motor]
        self.calls.append(("position", board, motor))
        return {"ok": True, "position": value, "ack": {"status": 100, "value": value}}

    def motor_get_speed(self, board, *, motor):
        return {"ok": True, "speed": 0, "ack": {"status": 100}}

    def motor_set_axis_param(self, board, param, value, *, motor):
        self.calls.append(("parameter", board, motor, param, value))
        return {"ok": True, "ack": {"status": 100}}

    def begin_bus_event_window(self):
        self.calls.append(("event_window",))
        return dict(self.window)

    def motor_oem_move_absolute(self, board, target, *, motor, wait_for_stop, max_position):
        axis = next(axis for axis, address in self.addresses.items() if address == (board, motor))
        self.moves.append((axis, target, wait_for_stop))
        if self.exception == axis:
            raise RuntimeError("offline_native_" + axis)
        before = self.motor_get_position(board, motor=motor)
        ok = self.failure != axis
        if ok:
            self.positions[board, motor] = target
        return {"ok": ok, "command_sent": ok, "ack": {"status": 100 if ok else 2},
                "before": before, "event_window": {**self.window, "dispatch_cursors": {f"{board}:{motor}": float(board)}},
                "oem_wait_for_stop": wait_for_stop, "wait": self._wait(board, motor)}

    def _wait(self, board, motor):
        return {"ok": True, "target_reached": True, "event": {
            "board": board, "motor": motor, "status": 128, "event_sequence": 1,
            "source": "novo_router_async", "latch_disposition": "consumed",
            "receive_owner": "offline", "owner_generation": 1}}

    def motor_wait_target_reached_many(self, addresses, **kwargs):
        self.waits.append((addresses, kwargs))
        if self.exception == "wait":
            raise RuntimeError("offline_native_wait")
        return {"ok": True, "per_axis": {"x": self._wait(5, 0), "y": self._wait(4, 0)}}

    def collect_bus_events(self, **kwargs):
        return []


@pytest.fixture
def rig(monkeypatch):
    # Any accidental network/serial opening must fail, not touch hardware.
    import socket

    def closed(*args, **kwargs):
        raise AssertionError("hardware transport must remain closed")

    monkeypatch.setattr(socket, "socket", closed)
    try:
        import serial
    except ImportError:
        pass  # No serial driver installed: there is nothing to open.
    else:
        monkeypatch.setattr(serial, "Serial", closed)
    monkeypatch.setattr(mod.time, "sleep", lambda seconds: sleeps.append(seconds))
    sleeps = []
    rows = {"LOC_OC_COVER": (1324, 42129), "LOC_RC_COVER": (42788, 44972),
            "LOC_OC_COVER_STORAGE": (84252, 6057), "LOC_RC_COVER_STORAGE": (84252, 36267),
            "LOC_RC": (42788, 44972)}
    table = SimpleNamespace(resolve=lambda *, location_id: PositionTarget(
        location_id, base_coordinates=dict(zip(("x", "y"), rows[location_id]))))
    monkeypatch.setattr(mod, "load_bound_oem_position_table", lambda: table)
    native = OfflineNative()
    adapter = object.__new__(mod.Serial206ProductionPrimitiveAdapter)
    adapter.tester = native
    adapter.y_provider = None
    adapter.reference_store = None
    adapter._reference_snapshot = lambda axes, context: {"offline": True}
    provider = object.__new__(mod.Serial206OemInitializationProvider)
    provider.primitives = adapter
    state = {"pseudo_z_home": 500, "tip_loaded": False, "plate_on_gantry": None}
    provider.mov_execution_machine_state = lambda: dict(state)
    raw_moves = []
    real_move_to = adapter.oem_move_to

    def observe_move_to(*args, **kwargs):
        # Observe, never fake, the real result before bounded JSON diagnostics.
        result = real_move_to(*args, **kwargs)
        raw_moves.append(result)
        return result

    adapter.oem_move_to = observe_move_to
    return SimpleNamespace(provider=provider, adapter=adapter, native=native, state=state,
                           sleeps=sleeps, raw_moves=raw_moves)


@pytest.mark.parametrize("location,offsets,target", [
    (17, (20021, 0), (21345, 42129)),
    (19, (20021, 0), (62809, 44972)),
    (20, (5923, 0), (90175, 36267)),
    (18, (5923, 0), (90175, 6057)),
    (3, (-20431, -162), (22357, 44810)),
    (18, (20000, 110000), (90213, 102906)),
])
def test_connected_inspection_uses_both_native_axes_and_one_mta_wait(rig, location, offsets, target):
    result = rig.provider._cover_inspection_move(location, *offsets)
    assert (result["target"]["x"], result["target"]["y"]) == target
    move = rig.raw_moves[-1]
    assert move["branch"] == "confirmed_gripper_no_tip_moveXY"
    assert move["target"] == {"x": target[0], "y": target[1], "z": 500}
    xy = move["operations"][0]
    assert xy["source_context"] == "ControlLib.inspectCover"
    assert xy["source_context_sealed"] is True
    assert xy["wait_schedule"] == "MTA_WaitAll"
    assert xy["branch"] == "parallel"
    assert move["controller_completion_verified"] is True
    assert sorted(rig.native.moves) == [("x", target[0], False), ("y", target[1], False)]
    assert rig.native.waits == [(((5, 0), (4, 0)), {
        "event_window": {**rig.native.window, "dispatch_cursors": {"5:0": 5.0, "4:0": 4.0}},
        "timeout_s": 5.0, "sta_sequential": False})]
    assert rig.native.calls[0] == ("gripper",)
    assert not any(call[0] == "parameter" and call[2] != 0 for call in rig.native.calls)


@pytest.mark.parametrize("current_z,pseudo,expected", [(499, 500, []), (500, 500, []),
                                                        (501, 500, [("z", 500, True)]),
                                                        (500, 1500, []), (1600, 1500, [("z", 1500, True)])])
def test_conditional_clearance_uses_dynamic_pseudo_home(rig, current_z, pseudo, expected):
    rig.native.positions[4, 1] = current_z
    rig.state["pseudo_z_home"] = pseudo
    result = rig.provider._cover_inspection_move(17, 20021, 0)
    assert rig.raw_moves[-1]["target"]["z"] == pseudo
    assert [move for move in rig.native.moves if move[0] == "z"] == expected
    if expected:
        assert rig.native.moves[0] == expected[0]
    assert len(rig.native.moves) == 2 + len(expected)


@pytest.mark.parametrize("gripper,tip,cover,branch,expected", [
    (True, True, None, "descending_y_parallel_x_first", [("x", 90175, True), ("y", 6057, True)]),
    (False, False, None, "descending_y_parallel_x_first", [("x", 90175, True), ("y", 6057, True)]),
    (False, False, 4, "descending_y_loaded_plate", [("y", 44972, True), ("x", 90175, True), ("y", 6057, True)]),
    (False, False, 5, "descending_y_loaded_plate", [("y", 44972, True), ("x", 90175, True), ("y", 6057, True)]),
    (True, True, 5, "descending_y_loaded_plate", [("y", 44972, True), ("x", 90175, True), ("y", 6057, True)]),
    # The source tests gripper/no-tip first, even when custody names a cover.
    (True, False, 4, "confirmed_gripper_no_tip_moveXY", [("x", 90175, False), ("y", 6057, False)]),
])
def test_real_state_selects_loaded_routing_and_safe_waypoint(rig, gripper, tip, cover, branch, expected):
    rig.native.home = gripper
    rig.state.update(tip_loaded=tip, plate_on_gantry=cover)
    result = rig.provider._cover_inspection_move(18, 5923, 0)
    assert result["move"]["branch"] == branch
    assert rig.native.moves == expected
    assert len(rig.native.waits) == (1 if branch == "confirmed_gripper_no_tip_moveXY" else 0)
    if branch == "descending_y_parallel_x_first":
        assert 0.600 in rig.sleeps


@pytest.mark.parametrize("current_x,current_y", [(1000, 50000), (85000, 2000)])
def test_loaded_cover_does_not_add_unneeded_clearance_waypoint(rig, current_x, current_y):
    rig.native.home = False
    rig.native.positions[5, 0] = current_x
    rig.native.positions[4, 0] = current_y
    rig.state["plate_on_gantry"] = 5
    result = rig.provider._cover_inspection_move(18, 5923, 0)
    assert result["move"]["branch"] == "descending_y_loaded_plate"
    assert rig.native.moves == [("x", 90175, True), ("y", 6057, True)]
    assert rig.native.waits == []


@pytest.mark.parametrize("failure", ["false", "exception"])
def test_inspection_failure_never_publishes_captures_or_classifies(rig, failure):
    def unexpected(*args, **kwargs):
        pytest.fail("inspection continued after failed movement")

    if failure == "false":
        rig.native.failure = "x"
        message = "cover_inspection_move_failed"
    else:
        rig.native.exception = "wait"
        message = "offline_native_wait"
    rig.provider.bind_oem_cover_inspection_callbacks(
        settings=lambda: {"CameraXOffset": 3499, "CameraYOffset": -7744, "CameraZOffset": 3145,
                          "InspectionSettings": {"CoverInspection": {"Exposure": 1000}}},
        capture=unexpected, save=unexpected, barcode=unexpected,
        led=lambda **kwargs: None, rgb=lambda *args: None)
    rig.provider.wp8_update_location = unexpected
    with pytest.raises(RuntimeError, match=message):
        rig.provider.wp8_inspect_cover_at(
            "inspectCoverAt", {"destination": 17, "screen_resolution_high": False},
            command_id="offline", child_order=2, plan_digest="offline")
    assert not getattr(rig.provider, "_oem_cover_inspection_findings", {})


def test_semantic_authority_refusal_never_queries_or_moves_axes(rig):
    def refused():
        raise RuntimeError("deck_semantic_state_not_authoritative:offline")

    rig.provider.mov_execution_machine_state = refused
    with pytest.raises(RuntimeError, match="deck_semantic_state_not_authoritative"):
        rig.provider._cover_inspection_move(17, 20021, 0)
    assert rig.native.calls == []
    assert rig.native.moves == []


def test_loaded_ascending_route_starts_y_without_xy_wait(rig):
    rig.state["tip_loaded"] = True
    result = rig.provider._cover_inspection_move(19, 0, 10000)
    assert result["move"]["branch"] == "parallel_y_first"
    assert rig.native.moves == [("y", 54972, True), ("x", 42788, True)]
    assert rig.native.waits == []
    assert 0.300 in rig.sleeps


@pytest.mark.parametrize("axis", ["x", "y", "z"])
def test_native_false_propagates_without_inspection_success(rig, axis):
    rig.native.failure = axis
    rig.native.positions[4, 1] = 600
    with pytest.raises(RuntimeError, match="^cover_inspection_move_failed$"):
        rig.provider._cover_inspection_move(17, 20021, 0)
    # Existing moveTo aggregates source return values; do not add a new guard.
    assert rig.native.moves == [("z", 500, True), ("y", 42129, False), ("x", 21345, False)]


@pytest.mark.parametrize("failure,expected", [
    ("z", [("z", 500, True)]),
    ("y", [("z", 500, True), ("y", 42129, False)]),
    ("x", [("z", 500, True), ("y", 42129, False), ("x", 21345, False)]),
    ("wait", [("z", 500, True), ("y", 42129, False), ("x", 21345, False)]),
])
def test_native_exception_retains_evidence_and_stops_remaining_calls(rig, failure, expected):
    rig.native.exception = failure
    rig.native.positions[4, 1] = 600
    with pytest.raises(RuntimeError, match="offline_native_" + failure) as error:
        rig.provider._cover_inspection_move(17, 20021, 0)
    assert error.value.motion_evidence["physical_effect_verified"] is False
    assert rig.native.moves == expected
    assert len(rig.native.waits) == (1 if failure == "wait" else 0)


def test_gripper_authority_refusal_never_issues_motion(rig):
    rig.native.motor_query_home_switch = lambda *args, **kwargs: {"reply_valid": False}
    with pytest.raises(RuntimeError, match="deck_gripper_observation_not_authoritative"):
        rig.provider._cover_inspection_move(17, 20021, 0)
    assert rig.native.moves == []
    assert rig.native.waits == []


def test_unknown_xy_context_remains_rejected(rig):
    with pytest.raises(ValueError, match="unsealed_moveXY_source_context"):
        rig.adapter.move_xy(100, 200, wait_timeout_s=5, source_context="inspectCover")
    assert rig.native.calls == []


def test_manual_ui_context_keeps_sequential_wait_contract(rig):
    result = rig.adapter.oem_move_to(21345, 42129, 500, pseudo_home_steps=500,
        gripper_confirmed=True, tip_loaded=False, source_context="ClassControlInterface.btnLOC1_Click")
    assert result["operations"][0]["wait_schedule"] == "STA_WaitAny_X_then_Y"
    assert rig.native.waits[0][1]["sta_sequential"] is True
    assert rig.native.waits[0][1]["timeout_s"] == 5.0
