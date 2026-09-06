from __future__ import annotations

from collections.abc import Mapping
import ast
import asyncio
import hashlib
import inspect
import json
from pathlib import Path
import sqlite3
import textwrap
import threading
import time
from typing import Any

import pytest
from fastapi import FastAPI

from bioxp.oem_serial206_initialization import (
    SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS,
    Serial206OemInitializationProvider,
    Serial206ProductionPrimitiveAdapter,
)
from bioxp.operator_receipt_store import OperatorHistoryReader
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.serial206_y_provider import Serial206YProvider
from bioxp.services.reference_service import MarkAxisReferencedCommand, ReferenceStateStore
from bioxp.usb_driver import BioXpTester


ACK = {"status": 100, "value": 0}


class OperatorCommandStore:
    def __new__(cls, *_args, **_kwargs):
        pytest.skip("retired duplicate operator mutation/scheduler authority")


def _absolute_driver(monkeypatch, *, board: int, positions: list[int], wait_result: dict[str, Any]):
    driver = object.__new__(BioXpTester)
    sent: list[tuple[Any, ...]] = []
    reads = iter(positions)
    monkeypatch.setattr(driver, "oem_no24v_state", lambda: False)
    monkeypatch.setattr(driver, "_oem_board_state", lambda: {board: True})
    monkeypatch.setattr(
        driver,
        "_motion_oem_axis_profile",
        lambda axis: {
            "board": board,
            "motor": 0,
            "axis_min_steps": 0,
            "axis_max_steps": 1000,
        },
    )
    monkeypatch.setattr(driver, "_oem_board_present", lambda selected: int(selected) == board)
    monkeypatch.setattr(driver, "motor_axis_key_for_channel", lambda selected, motor=0: "x" if int(selected) == 5 else "y")
    monkeypatch.setattr(
        driver,
        "motor_get_position",
        lambda selected, motor=0: {"ok": True, "ack": ACK, "position": next(reads)},
    )
    monkeypatch.setattr(
        driver,
        "begin_bus_event_window",
        lambda *, reset_wait_latch=True: {
            "after_sequence": 0 if reset_wait_latch else None,
            "oem_wait_latch_reset": bool(reset_wait_latch),
        },
    )
    monkeypatch.setattr(driver, "motor_query_motor_stop", lambda selected, motor=0: {"ok": True, "ack": ACK})

    monkeypatch.setattr(
        driver,
        "_send_motor",
        lambda *args, **kwargs: sent.append(tuple(args)) or dict(ACK),
    )
    monkeypatch.setattr(driver, "motor_oem_wait_target_reached", lambda *args, **kwargs: dict(wait_result))
    monkeypatch.setattr(
        driver,
        "motor_get_speed",
        lambda selected, motor=0: {"ok": True, "ack": ACK, "speed": 0},
    )
    return driver, sent


def test_deck_move_to_abs_keeps_direct_zero_and_returns_board_wrapper_zero(monkeypatch):
    driver, sent = _absolute_driver(
        monkeypatch,
        board=BioXpTester.BOARD_DECK,
        positions=[500, 0],
        wait_result={"ok": True, "event": {"status": 128}},
    )

    result = driver.motor_oem_move_absolute(
        BioXpTester.BOARD_DECK,
        0,
        motor=0,
        wait_for_stop=True,
        max_position=1000,
    )

    assert result["ok"] is True
    assert result["wire_position"] == 0
    assert result["source_return_code"] == 0
    assert sent[0][4] == 0


def test_deck_move_to_abs_does_not_suppress_high_target_until_already_near_limit(monkeypatch):
    driver, sent = _absolute_driver(
        monkeypatch,
        board=BioXpTester.BOARD_DECK,
        positions=[500, 1000],
        wait_result={"ok": True, "event": {"status": 128}},
    )

    result = driver.motor_oem_move_absolute(
        BioXpTester.BOARD_DECK,
        1000,
        motor=0,
        wait_for_stop=True,
        max_position=1000,
    )

    assert result["ok"] is True
    assert result.get("source_noop", False) is False
    assert len(sent) == 1


def test_deck_move_to_abs_timeout_raises_source_exception(monkeypatch):
    driver, _ = _absolute_driver(
        monkeypatch,
        board=BioXpTester.BOARD_DECK,
        positions=[500, 800],
        wait_result={"ok": False, "failure": "oem_moveToAbs_target_event_timeout"},
    )

    with pytest.raises(RuntimeError, match="Reach position time out"):
        driver.motor_oem_move_absolute(
            BioXpTester.BOARD_DECK,
            800,
            motor=0,
            wait_for_stop=True,
            max_position=1000,
        )


def test_move_steps_uses_cached_position_before_limit_decision_and_zero_sends_no_frame(monkeypatch):
    driver = object.__new__(BioXpTester)
    driver._oem_position_cache = {(BioXpTester.BOARD_HEAD, 0): 500}
    calls: list[str] = []
    monkeypatch.setattr(driver, "oem_no24v_state", lambda: False)
    monkeypatch.setattr(driver, "_oem_board_state", lambda: {BioXpTester.BOARD_HEAD: True})
    monkeypatch.setattr(
        driver,
        "_motion_oem_axis_profile",
        lambda axis: {
            "board": BioXpTester.BOARD_HEAD,
            "motor": 0,
            "axis_min_steps": 0,
            "axis_max_steps": 1000,
        },
    )
    monkeypatch.setattr(driver, "_oem_board_present", lambda board: True)
    monkeypatch.setattr(
        driver,
        "motor_get_position",
        lambda board, motor=0: calls.append("position") or {"ok": True, "ack": ACK, "position": 500},
    )
    monkeypatch.setattr(
        driver,
        "motor_move_relative",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("zero-step OEM call must not send a move frame")),
    )

    result = driver.motor_y_move_relative_strict(steps=0, timeout_s=1.0)

    assert result["ok"] is True
    assert result["source_noop"] is True
    assert result["source_return_code"] == 500
    assert calls == ["position", "position"]


def test_nonzero_move_steps_dispatches_from_cached_position_before_public_read(monkeypatch):
    driver = object.__new__(BioXpTester)
    driver._oem_position_cache = {(BioXpTester.BOARD_HEAD, 0): 500}
    calls: list[str] = []
    positions = iter((600, 600))

    monkeypatch.setattr(driver, "oem_no24v_state", lambda: False)
    monkeypatch.setattr(driver, "_oem_board_state", lambda: {BioXpTester.BOARD_HEAD: True})
    monkeypatch.setattr(
        driver,
        "_motion_oem_axis_profile",
        lambda *_args, **_kwargs: {"axis_min_steps": 0, "axis_max_steps": 102956},
    )
    monkeypatch.setattr(driver, "motor_query_motor_stop", lambda *_args, **_kwargs: {"ok": True})
    monkeypatch.setattr(driver, "begin_bus_event_window", lambda **_kwargs: {"started": True})
    monkeypatch.setattr(
        driver,
        "_send_motor",
        lambda *_args, **_kwargs: calls.append("move")
        or {"status": 100, "provenance": {"tx_attempt_count": 1}},
    )

    monkeypatch.setattr(
        driver,
        "motor_oem_wait_target_reached",
        lambda *_args, **_kwargs: {
            "ok": True,
            "target_reached": True,
            "event": {"status": 128, "board": 4, "motor": 0},
        },
    )
    monkeypatch.setattr(
        driver,
        "motor_get_position",
        lambda *_args, **_kwargs: calls.append("position") or {"ok": True, "position": next(positions)},
    )

    result = driver.motor_y_move_relative_strict(100, timeout_s=1.0)

    assert result["ok"] is True
    assert calls == ["move", "position", "position"]
    assert result["board_wrapper_return"] == 600
    assert result["source_return_code"] == 600


class _HomeXYFailureTester(BioXpTester):
    def __init__(self):
        self.set_home_calls: list[tuple[int, int]] = []
        self.parameter_writes: list[tuple[int, int, int, int]] = []
        self.go_home_timeouts: list[float] = []

    def _motion_oem_axis_profile(self, axis, startup=True):
        return {
            "x": {"board": 5, "motor": 0, "speed": 1700, "acc": 350},
            "y": {"board": 4, "motor": 0, "speed": 1800, "acc": 400},
        }[axis]

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.parameter_writes.append((int(board), int(motor), int(param), int(value)))
        return {"ok": True, "readback": None}

    def motor_oem_go_home(self, axis, **kwargs):
        self.go_home_timeouts.append(float(kwargs["timeout_s"]))
        return {"ok": axis == "y", "axis": axis, "source_return_code": 0 if axis == "y" else -1}

    def motor_axis_status(self, board, motor=0):
        return {"speed": {"speed": 0}, "switches": {"left_raw_active": True}}

    def motor_set_home(self, board, motor=0):
        self.set_home_calls.append((int(board), int(motor)))
        return {"ok": True}

    def motor_get_position(self, board, motor=0):
        return {"position": 0}


def test_low_level_home_xy_propagates_child_exception_without_profile_restore():
    class Tester(_HomeXYFailureTester):
        def motor_oem_go_home(self, axis, **kwargs):
            if axis == "x":
                raise RuntimeError("x home failed")
            return super().motor_oem_go_home(axis, **kwargs)

    driver = Tester()
    with pytest.raises(RuntimeError, match="x home failed"):
        driver.motor_oem_home_xy(timeout_s=1.0)
    assert [value for *_prefix, value in driver.parameter_writes] == [200, 200, 200, 200]


def test_low_level_home_xy_restores_profiles_after_normal_child_return_code():
    driver = _HomeXYFailureTester()

    result = driver.motor_oem_home_xy(timeout_s=120.0)

    assert result["ok"] is True
    assert driver.set_home_calls == []
    assert "home_rebase" not in result
    assert driver.go_home_timeouts == [30.0, 30.0]
    restored = [value for _board, _motor, _parameter, value in driver.parameter_writes]
    for expected in (1700, 350, 1800, 400):
        assert expected in restored


class _MoveXYTester:
    def __init__(self):
        self.calls: list[tuple[Any, ...]] = []

    def motor_get_position(self, board, motor=0):
        return {"ok": False, "ack": None, "position": 0}

    def _motion_oem_axis_profile(self, axis, startup=False):
        return {"board": 5 if axis == "x" else 4, "motor": 0, "axis_max_steps": 1000}

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.calls.append(("sap", board, motor, param, value))
        return {"ok": True, "readback": {"value": value}}

    def motor_oem_move_absolute(self, board, position, motor=0, wait_for_stop=False, max_position=None):
        self.calls.append(("move_abs", board, motor, position, wait_for_stop))
        return {"ok": True, "source_return_code": 0, "position": position}


class _MoveXYYProvider:
    def __init__(self, tester: _MoveXYTester):
        self.tester = tester

    def move_absolute(self, *, target_steps, wait_for_stop, wait_timeout_s):
        self.tester.calls.append(("y_abs", target_steps, wait_for_stop))
        return {"ok": True, "source_return_code": 0, "position": target_steps}


def test_move_xy_uses_integer_getter_results_and_near_branch_waits_x_then_y():
    tester = _MoveXYTester()
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = tester
    adapter.y_provider = _MoveXYYProvider(tester)
    adapter.reference_store = None
    adapter.x_move_absolute = lambda **kwargs: tester.calls.append(
        ("move_abs", 5, 0, kwargs["position_steps"], kwargs["wait_for_stop"])
    ) or {"ok": True, "source_return_code": 0, "position": kwargs["position_steps"]}

    result = adapter.move_xy(x=100, y=10, wait_timeout_s=2.0)

    assert result["ok"] is True
    moves = [row for row in tester.calls if row[0] in {"move_abs", "y_abs"}]
    assert moves == [
        ("move_abs", 5, 0, 100, True),
        ("y_abs", 10, True),
    ]


class _Timer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


def test_force_abort_does_not_cancel_unrelated_thermal_fan_service_timer():
    driver = object.__new__(BioXpTester)
    timer = _Timer()
    driver._oem_fan_service_state = {(BioXpTester.BOARD_THERMAL, 0): {"timer": timer}}
    driver._oem_board_presence = {
        BioXpTester.BOARD_HEAD: False,
        BioXpTester.BOARD_DECK: False,
        BioXpTester.BOARD_THERMAL: True,
    }

    result = driver.motor_oem_force_abort_motion(reason="test")

    assert result["ok"] is True
    assert timer.cancelled is False


def test_initialize_motors_live_runs_the_whole_oem_sequence_without_stage_approvals(monkeypatch):
    provider = Serial206OemInitializationProvider(object(), generation_provider=lambda: 7, sleep=lambda _s: None)
    executed: list[str] = []

    monkeypatch.setattr(
        provider,
        "_execute_stage",
        lambda spec, timeout_s: executed.append(spec.key) or {"ok": True, "stage": spec.key},
    )

    result = provider.initialize_motors(mode="live", timeout_s=30.0)

    assert result["ok"] is True
    assert result["ready"] is True
    assert executed == [spec.key for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS]
    assert result["movement_ledger"]["expected_next_stage"] is None
    assert all(row["state"] == "completed" for row in result["movement_ledger"]["stages"].values())


@pytest.mark.parametrize("failing_stage", ["x-park-6000", "door-closed-predicate"])
def test_initialize_motors_propagates_source_exception_and_stops_sequence(monkeypatch, failing_stage):
    provider = Serial206OemInitializationProvider(object(), generation_provider=lambda: 7, sleep=lambda _s: None)
    executed: list[str] = []

    def execute(spec, timeout_s):
        del timeout_s
        executed.append(spec.key)
        if spec.key == failing_stage:
            raise RuntimeError(f"source failure at {failing_stage}")
        return {"ok": True}

    monkeypatch.setattr(provider, "_execute_stage", execute)

    with pytest.raises(RuntimeError, match=f"source failure at {failing_stage}"):
        provider.initialize_motors(mode="live", timeout_s=30.0)
    expected = [spec.key for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS]
    assert executed == expected[: expected.index(failing_stage) + 1]


def test_door_closed_source_branch_opens_door_then_throws():
    calls: list[str] = []

    class Primitives:
        def motor_oem_door_search_home(self, **_kwargs):
            return {"status_after": {"oem_predicates": {"tcDoorClosed": False}}}

        def oem_initialize_motors_branch_binding(self):
            return {"serial_number": 206, "camera_calibrated": True}

        def motor_oem_open_thermal_door(self, **_kwargs):
            calls.append("open")
            return {"ok": True}

    provider = Serial206OemInitializationProvider(Primitives(), generation_provider=lambda: 7, sleep=lambda _s: None)
    spec = next(row for row in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS if row.key == "door-home")

    with pytest.raises(RuntimeError, match="Cannot close thermal cycler door"):
        provider._execute_stage(spec, timeout_s=30.0)
    assert calls == ["open"]


def test_initialize_motion_runs_source_branch_without_stage_approvals(monkeypatch):
    provider = Serial206OemInitializationProvider(
        object(),
        generation_provider=lambda: 7,
        sleep=lambda _seconds: None,
    )
    seen: list[str] = []

    def initialize_motors(*, mode, timeout_s):
        assert mode == "live"
        assert timeout_s == 30.0
        seen.append("initializeMotion.initializeMotors")
        return {"ok": True, "ready": True}

    def execute_motion(state, spec, *, timeout_s):
        del state, timeout_s
        seen.append(spec.key)
        if spec.key == "initializeMotion.queryTipStatus.initial":
            return {"ok": True, "tip_exists": False, "tip_channels": []}
        return {"ok": True}

    monkeypatch.setattr(provider, "initialize_motors", initialize_motors)
    monkeypatch.setattr(provider, "_execute_initialize_motion_stage", execute_motion)

    result = provider.initialize_motion(mode="live", timeout_s=30.0)

    assert result["ok"] is True
    assert seen == [
        "initializeMotion.stop_scripts",
        "initializeMotion.clear_forceabort",
        "initializeMotion.initializeMotors",
        "initializeMotion.thermal_door_closed",
        "initializeMotion.queryTipStatus.initial",
        "initializeMotion.sleep.after_tip_query",
        "initializeMotion.tip_loaded_false.no_tip",
    ]


def test_xy_intent_uses_source_completion_without_extra_terminal_proof_gates():
    class Primitives:
        @staticmethod
        def current_board_lifecycle_generation():
            return 9

        def move_xy(self, *_args, **_kwargs):
            return {
                "ok": True,
                "source_calls_completed": True,
                "controller_terminal_state_verified": False,
                "target_position_verified": False,
                "acceleration_restore_verified": False,
            }

    provider = Serial206OemInitializationProvider(Primitives(), generation_provider=lambda: 3)
    state = provider._load_state()
    state["x_lifecycle"].update({
        "state": "referenced_ready",
        "generation": 3,
        "board_lifecycle_generation": 9,
        "reference_state": "referenced",
    })
    provider._save_state(state)

    class YAuthority:
        @staticmethod
        def _authority():
            return {
                "board": {"state": "active", "active_board_epoch": 7},
                "axes": {"y": {
                    "ownership_generation": 3,
                    "prepared_board_epoch": 7,
                    "lifecycle_state": "referenced_ready",
                    "reference_state": "referenced",
                    "interrupt_epoch": 0,
                }},
            }

    provider.y_provider = YAuthority()

    result = provider.execute_xy_intent(
        100,
        200,
        {"idempotency_key": "xy-source-complete"},
    )

    assert result["ok"] is True
    assert result["state"] == "referenced_ready"
    assert result["result"]["ok"] is True
    assert result["result"]["source_calls_completed"] is True


def _operator_state(generation: int = 4) -> dict[str, Any]:
    return {
        "ownership_generation": generation,
        "serial206_initialization_provider": {
            "x_authority": {},
            "y_authority": {"live_status": {"position_steps": 1000}},
            "z_authority": {"state": "referenced_ready", "reference_state": "referenced"},
        },
    }


def _action_request(action_id: str, inputs: dict[str, Any], key: str, *, expected_generation: int = 4):
    return {
        "schema_version": "bioxp.operator_action_request.v2",
        "action_id": action_id,
        "inputs": inputs,
        "expected_ownership_generation": expected_generation,
        "expected_board_epoch_by_board": {},
        "idempotency_key": key,
    }


def test_disjoint_y_and_z_commands_can_be_claimed_concurrently(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        y = store.admit_command(
            _action_request("oem.y.move_steps", {"steps": 10}, "parallel-y"),
            state=_operator_state(),
        )
        z = store.admit_command(
            _action_request("oem.z.move_steps", {"steps": 10}, "parallel-z"),
            state=_operator_state(),
        )
        first = store.claim_next()
        second = store.claim_next()
        assert first is not None and first["command_id"] == y["command_id"]
        assert second is not None and second["command_id"] == z["command_id"]
    finally:
        store.stop()


def test_request_generation_is_observed_metadata_not_an_admission_blocker(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        receipt = store.admit_command(
            _action_request(
                "oem.y.move_steps",
                {"steps": 10},
                "stale-generation-observation",
                expected_generation=3,
            ),
            state=_operator_state(generation=4),
        )
        assert receipt["status"] == "queued"
        assert receipt["ownership_generation"] == 3
    finally:
        store.stop()


def test_axis_stop_does_not_raise_the_global_dispatch_fence(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        store.begin_interrupt(
            "oem.y.stop",
            state=_operator_state(),
            request={"idempotency_key": "axis-stop-local-fence", "expected_generation": 4},
        )
        assert store._priority_fence.is_set() is False
    finally:
        store.stop()


def test_axis_stop_finalization_does_not_clear_an_aggregate_interrupt_fence(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        receipt = store.begin_interrupt(
            "oem.y.stop",
            state=_operator_state(),
            request={
                "schema_version": "bioxp.operator_interrupt_request.v1",
                "idempotency_key": "stop-y-fence-12345678",
                "reason": "test",
                "expected_generation": 4,
            },
        )
        store._priority_fence.set()
        attempted = store.mark_interrupt_attempted(
            idempotency_key="stop-y-fence-12345678"
        )
        store.finalize_interrupt(
            idempotency_key="stop-y-fence-12345678",
            receipt=attempted,
            attempted=True,
            acknowledged=True,
            response={
                "ok": True,
                "source_call_completed": True,
                "source_return_ok": True,
            },
        )
        assert store._priority_fence.is_set() is True
    finally:
        store.stop()




def test_legacy_json_runtime_plane_is_not_mounted(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp.api import app

    assert all(
        not path.startswith("/oem/runtime")
        for path in app.openapi()["paths"]
    )


def test_reference_service_production_path_is_sqlite_not_json(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp import api

    assert api._default_reference_state_path() == str(tmp_path / "bioxp_runtime.db")


def test_initialization_state_never_falls_back_to_runtime_json(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        (tmp_path / "serial206_oem_initialization_state.json").write_text(
            json.dumps({"schema_version": "legacy-json"}),
            encoding="utf-8",
        )
        assert store.read_oem_serial206_initialization_state() is None
    finally:
        store.close()


def test_board4_transition_attempts_append_to_sqlite_history(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        store.record_board4_transition(
            active=True,
            ack={"status": 100},
            transition_id="activate-1",
            ownership_generation=1,
        )
        store.record_board4_transition(
            active=False,
            ack={"status": 100},
            transition_id="deactivate-1",
            ownership_generation=1,
        )
        table = store._db.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='serial206_board_transitions'"
        ).fetchone()
        assert table is not None
        rows = store._db.execute(
            "SELECT transition_id,requested_active,status_code FROM serial206_board_transitions ORDER BY sequence"
        ).fetchall()
        assert [tuple(row) for row in rows] == [
            ("activate-1", 1, 100),
            ("deactivate-1", 0, 100),
        ]
    finally:
        store.close()


def test_board4_transition_rebinds_invalidated_axes_to_current_ownership_generation(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        store.record_board4_transition(
            active=True,
            ack={"status": 100},
            transition_id="generation-rebind",
            ownership_generation=9,
        )
        rows = store._db.execute(
            "SELECT axis,ownership_generation,lifecycle_state FROM serial206_axis_authority ORDER BY axis"
        ).fetchall()
        assert [(row["axis"], row["ownership_generation"]) for row in rows] == [
            ("gripper", 9),
            ("y", 9),
            ("z", 9),
        ]
    finally:
        store.close()




def test_v2_detail_projects_nested_provider_readbacks_as_scalars(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        admitted = store.admit_command(
            _action_request("oem.y.move_steps", {"steps": 10}, "scalar-detail"),
            state=_operator_state(),
        )
        claimed = store.claim_next()
        assert claimed is not None
        store.finish(
            admitted["command_id"],
            status="completed",
            payload={
                "response": {
                    "position_after": {"position": 123},
                    "terminal_position": {"value": 123},
                    "terminal_speed": {"speed": 0},
                    "discrepancy": {"value": 2},
                }
            },
            claimed=claimed,
        )

        detail = store.command_detail_v2(admitted["command_id"])
        assert detail is not None
        assert detail["observed_values"] == {
            "position_after": 123,
            "terminal_position": 123,
            "terminal_speed": 0,
            "discrepancy": 2,
        }
    finally:
        store.stop()


def test_z_terminal_projection_preserves_observed_only_switch_registers():
    projected = Serial206OemInitializationProvider._sanitize_z_terminal_state({
        "authority": "serial206_terminal_register_readback",
        "position_steps": 10,
        "speed_steps_s": 0,
        "switch_mask_tuple": {12: 0, 13: 1},
        "switch_mask_policy": "observed_only_oem_source_omits_z_writes",
    })

    assert projected is not None
    assert projected["switch_mask_tuple"] == {12: 0, 13: 1}
    assert projected["switch_mask_policy"] == "observed_only_oem_source_omits_z_writes"
    from_result = Serial206OemInitializationProvider._z_terminal_state_from_result(
        {
            "terminal_z_state": {
                "ok": True,
                "position_steps": 10,
                "speed_steps_s": 0,
                "switch_mask_tuple": {12: 0, 13: 1},
                "switch_mask_policy": "observed_only_oem_source_omits_z_writes",
            }
        },
        command_id="z-terminal",
        observed_at=1.0,
    )
    assert from_result is not None
    assert from_result["switch_mask_tuple"] == {12: 0, 13: 1}


def test_deck_x_absolute_timeout_is_not_converted_to_success_at_target(monkeypatch):
    tester = object.__new__(BioXpTester)
    monkeypatch.setattr(tester, "motor_oem_wait_target_reached", lambda *_args, **_kwargs: {"ok": False, "failure": "timeout"})
    monkeypatch.setattr(tester, "collect_bus_events", lambda **_kwargs: [])
    monkeypatch.setattr(tester, "motor_get_position", lambda *_args, **_kwargs: {"position": 100, "ok": True})
    monkeypatch.setattr(tester, "motor_get_speed", lambda *_args, **_kwargs: {"speed": 0, "ok": True})
    monkeypatch.setattr(tester, "_tmcl_success", lambda value: isinstance(value, Mapping) and value.get("status") == 100)
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = tester
    adapter.reference_store = None

    result = adapter._x_finalize(
        {
            "ok": True,
            "command_issued": True,
            "ack": {"status": 100},
            "target_position_steps": 100,
            "event_window": {"after_sequence": 0},
        },
        timeout_s=1.0,
        motion_kind="absolute",
    )

    assert result["ok"] is False
    assert result["failure"] == "timeout"


def test_x_move_steps_delegates_to_cached_currentposition_source_wrapper(monkeypatch):
    class Tester:
        def oem_no24v_state(self):
            return False

        def _oem_board_state(self):
            return {5: True}

        def motor_get_position(self, *_args, **_kwargs):
            raise AssertionError("adapter must not replace cached CurrentPosition with a live pre-read")

        def motor_x_move_relative_strict(self, steps, *, timeout_s):
            assert steps == 125
            assert timeout_s == 3.0
            return {
                "ok": True,
                "command_sent": True,
                "source_return_code": 1125,
                "board_wrapper_return": 1125,
            }

    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = Tester()
    adapter.reference_store = None
    monkeypatch.setattr(adapter, "_x_oem_move_preflight", lambda: {"ok": True})
    monkeypatch.setattr(adapter, "_reference_snapshot", lambda *_args: {"ok": True})

    result = adapter.x_move_steps(steps=125, wait_timeout_s=3.0)

    assert result["ok"] is True
    assert result["source_return_code"] == 1125


def test_protocol_motion_handler_uses_canonical_durable_deck_executor(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp import api

    source = inspect.getsource(api._protocol_live_move_handler)
    assert "_execute_absolute_move" not in source
    assert "_execute_relative_move" not in source
    assert "execute_x_intent" not in source
    assert "execute_z_intent" not in source
    assert "ClassMoveToIntent" in source
    assert "oem_mov_execution_admitter" in source
    assert "_wait_protocol_deck_command" in source


def test_v2_catalog_and_receipt_source_cover_all_interrupt_states():
    import bioxp.operator_controls as controls

    source = inspect.getsource(controls.install_operator_control_plane)
    assert 'str(action["action_id"]) == "oem.y.stop"' not in source
    assert '"stop_requested": "interrupting"' in source
    assert '"abort_requested": "interrupting"' in source




def test_axis_stop_preserves_other_axis_method_siblings(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        admitted = store.admit_method(
            {
                "idempotency_key": "mixed-axis-method-stop",
                "expected_ownership_generation": 4,
                "expected_board_epoch_by_board": {},
                "name": "mixed axis",
                "metadata": {},
                "steps": [
                    {"action_id": "oem.x.move_steps", "inputs": {"steps": 10}},
                    {"action_id": "oem.y.move_steps", "inputs": {"steps": 10}},
                ],
            },
            state=_operator_state(),
        )
        store.begin_interrupt(
            "oem.y.stop",
            state=_operator_state(),
            request={"idempotency_key": "method-y-stop", "observed_ownership_generation": 4},
        )
        rows = store.connection.execute(
            "SELECT action_id,status FROM operator_plane_commands WHERE method_id=? ORDER BY method_sequence",
            (admitted["method_id"],),
        ).fetchall()
        assert [tuple(row) for row in rows] == [
            ("oem.x.move_steps", "queued"),
            ("oem.y.move_steps", "cleared"),
        ]
        method = store.connection.execute(
            "SELECT status FROM operator_plane_methods WHERE method_id=?",
            (admitted["method_id"],),
        ).fetchone()
        assert method[0] == "queued"
    finally:
        store.stop()


def test_interrupt_response_has_exact_sqlite_evidence(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        receipt = store.begin_interrupt(
            "oem.x.stop",
            state=_operator_state(),
            request={"idempotency_key": "large-interrupt-response", "observed_ownership_generation": 4},
            interrupt_attempt_id="attempt-large-response",
        )
        large_value = "x" * 200_000
        final = store.finalize_interrupt(
            idempotency_key="large-interrupt-response",
            receipt=receipt,
            attempted=True,
            acknowledged=True,
            response={"ok": True, "controller_command_acknowledged": True, "blob": large_value},
        )
        assert final["interrupt_attempt_id"] == "attempt-large-response"
        assert final["controller_response_evidence"]["payload_bytes"] > 200_000
        row = store.connection.execute(
            "SELECT payload_json FROM operator_plane_interrupt_evidence WHERE interrupt_attempt_id=?",
            ("attempt-large-response",),
        ).fetchone()
        assert row is not None
        assert json.loads(row[0])["blob"] == large_value
    finally:
        store.stop()


def test_initialization_run_is_durable_and_idempotent(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp import api

    store = OEMRuntimeStore(tmp_path)
    calls: list[str] = []

    class Provider:
        state_store = store
        generation_provider = staticmethod(lambda: 7)

        @staticmethod
        def initialize_motors(*, mode, timeout_s):
            calls.append(f"{mode}:{timeout_s}")
            return {"ok": True, "state": "complete", "physical_motion_commanded": True}

    try:
        first = api._run_idempotent_serial206_initialization(
            Provider(),
            initialization_kind="initialize_motors",
            idempotency_key="initialize-run-key",
            timeout_s=12.0,
        )
        replay = api._run_idempotent_serial206_initialization(
            Provider(),
            initialization_kind="initialize_motors",
            idempotency_key="initialize-run-key",
            timeout_s=12.0,
        )
        assert first["ok"] is True and first["idempotent_replay"] is False
        assert replay["ok"] is True and replay["idempotent_replay"] is True
        assert replay["run_id"] == first["run_id"]
        conflict = api._run_idempotent_serial206_initialization(
            Provider(),
            initialization_kind="initialize_motors",
            idempotency_key="initialize-run-key",
            timeout_s=13.0,
        )
        assert conflict["ok"] is False
        assert conflict["failure"] == "initialization_idempotency_conflict"
        assert calls == ["live:12.0"]
        row = store._db.execute(
            "SELECT status,receipt_json FROM serial206_receipts WHERE stream='initialize_motors' AND idempotency_key=?",
            ("initialize-run-key",),
        ).fetchone()
        assert row is not None and row["status"] == "completed"
        saved = json.loads(row["receipt_json"])
        assert saved["response"]["ok"] is True
        with pytest.raises(Exception):
            store.append_serial206_receipt("initialize_motors", {**saved, "status": "failed"})
    finally:
        store.close()


def test_emergency_stop_alias_is_removed_in_favor_of_one_oem_abort_identity():
    from bioxp import api

    paths = api.app.openapi()["paths"]
    assert not hasattr(api, "motion_emergency_stop")
    assert "/motion/emergency_stop" not in paths
    assert "/motion/oem/x/abort" in paths




class _ProviderHomeXYFailureTester:
    def __init__(self):
        self.parameter_writes: list[tuple[int, int, int, int]] = []

    def _oem_board_present(self, board):
        return int(board) in {4, 5}

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.parameter_writes.append((int(board), int(motor), int(param), int(value)))
        return {"ok": True, "readback": None}

    def motor_oem_go_home(self, axis, **_kwargs):
        return {
            "ok": axis == "y",
            "axis": axis,
            "source_return_code": 0 if axis == "y" else -1,
            "controller_command_acknowledged": axis == "y",
            "controller_terminal_state_verified": axis == "y",
            "controller_home_proof_verified": axis == "y",
        }

    def motor_get_position(self, board, motor=0):
        return {"ok": True, "position": 0}


def test_provider_home_xy_propagates_child_exception_without_profile_restore():
    class Tester(_ProviderHomeXYFailureTester):
        def motor_oem_go_home(self, axis, **kwargs):
            if axis == "x":
                raise RuntimeError("provider x home failed")
            return super().motor_oem_go_home(axis, **kwargs)

    tester = Tester()
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = tester
    adapter.reference_store = None
    adapter.y_provider = None

    with pytest.raises(RuntimeError, match="provider x home failed"):
        adapter.home_xy()
    assert [value for *_prefix, value in tester.parameter_writes] == [200, 200, 200, 200]


def test_provider_home_xy_restores_profiles_after_normal_child_return_code():
    tester = _ProviderHomeXYFailureTester()
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = tester
    adapter.reference_store = None
    adapter.y_provider = None

    result = adapter.home_xy()

    assert result["ok"] is True
    assert result["setup_verified"] is True
    assert result["restore_verified"] is True
    assert result["restore"]["x_speed"]["ok"] is True
    assert result["restore"]["y_acc"]["ok"] is True
    restored = [value for *_prefix, value in tester.parameter_writes]
    for expected in (1700, 350, 1800, 400):
        assert expected in restored


def test_x_acceleration_overload_restores_default_after_normal_failure_return():
    writes: list[tuple[int, int, int, int]] = []
    delegated: dict[str, Any] = {}

    class Tester:
        def motor_set_axis_param(self, board, param, value, motor=0):
            writes.append((int(board), int(motor), int(param), int(value)))
            return {"ok": True, "ack": ACK, "readback": {"value": int(value)}}

    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = Tester()
    adapter.reference_store = None
    adapter._reference_snapshot = lambda *_args, **_kwargs: {"ok": True}
    def issue(*_args, **kwargs):
        delegated.update(kwargs)
        return {
            "ok": False,
            "command_issued": False,
            "failure": "normal_source_non_success_return",
        }

    adapter._x_issue_absolute = issue
    adapter._x_finalize = lambda ticket, **_kwargs: dict(ticket)

    result = adapter.x_move_absolute(position_steps=100, acceleration=123)

    assert result["ok"] is False
    assert delegated["acceleration"] == 123
    assert writes == [(5, 0, 5, 350)]
    assert result["acceleration_restore_acknowledged"] is True


def test_y_provider_defers_cached_first_limit_decision_to_source_primitive():
    calls: list[tuple[Any, ...]] = []

    class Tester:
        def _motion_oem_axis_profile(self, axis, startup=False):
            assert axis == "y"
            return {
                "board": 4,
                "motor": 0,
                "axis_min_steps": 0,
                "axis_max_steps": 102956,
                "speed": 1800,
                "acc": 400,
                "run_current": 31,
                "stall_guard": 16,
                "disable_right": True,
            }

        def _machine_config_axis_max(self, axis):
            assert axis == "y"
            return 102956, "test_selected_machine_configuration"

        def motor_get_position(self, board, motor=0):
            calls.append(("live_position", int(board), int(motor)))
            return {"ok": True, "position": 102950}

        def motor_y_move_relative_strict(self, steps, timeout_s):
            calls.append(("source_move", int(steps), float(timeout_s)))
            return {
                "ok": True,
                "before": {"position": 500},
                "target_position": 600,
                "terminal_position": {"position": 600},
                "source_return_code": 600,
                "proof": {"addressed_event_128": True, "speed_zero": True},
            }

    provider = Serial206YProvider(
        Tester(),
        state_store=None,
        generation_provider=lambda: 1,
        reference_store=None,
    )

    result = provider.move_steps(100, wait_timeout_s=1.0)

    assert result["ok"] is True
    assert calls[0] == ("source_move", 100, 1.0)
    assert result["position_before"] == 500
    assert result["requested_target"] == 600


def test_initialize_motors_continues_after_ignored_normal_non_success_return(monkeypatch):
    provider = Serial206OemInitializationProvider(
        object(),
        generation_provider=lambda: 7,
        sleep=lambda _seconds: None,
    )
    executed: list[str] = []

    def execute(spec, timeout_s):
        del timeout_s
        executed.append(spec.key)
        return {"ok": False, "failure": "ignored_integer_status"} if len(executed) == 1 else {"ok": True}

    monkeypatch.setattr(provider, "_execute_stage", execute)

    result = provider.initialize_motors(mode="live", timeout_s=30.0)

    assert executed == [spec.key for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS]
    assert result["ok"] is True
    assert result["stage_receipts"][0]["source_return_ok"] is True
    assert result["stage_receipts"][0]["raw_source_return_ok"] is False


def test_xz_interrupts_deliver_before_any_runtime_store_read_and_preserve_result_on_persistence_failure():
    x_source = inspect.getsource(Serial206OemInitializationProvider.execute_x_stop_interrupt)
    z_source = inspect.getsource(Serial206OemInitializationProvider.execute_z_stop_interrupt)

    assert "read_oem_serial206_initialization_state" not in x_source.split("raw_result =", 1)[0]
    assert "read_oem_serial206_initialization_state" not in z_source.split("raw_result =", 1)[0]
    assert "interrupt_persistence_failed" in x_source
    assert "interrupt_persistence_failed" in z_source
    assert '"source_return_ok": result.get("ok") is True' in x_source
    assert '"source_return_ok": result.get("ok") is True' in z_source



def test_axis_stop_clears_composite_command_by_resource(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        xy = store.admit_command(
            _action_request("oem.xy.move_absolute", {"x": 100, "y": 200}, "queued-xy-command"),
            state=_operator_state(),
        )
        store.begin_interrupt(
            "oem.y.stop",
            state=_operator_state(),
            request={
                "idempotency_key": "stop-y-composite",
                "observed_ownership_generation": 4,
                "observed_board_epoch_by_board": {},
            },
        )
        assert store.get_command(xy["command_id"])["status"] == "cleared"
    finally:
        store.stop()


def test_axis_stop_terminalizes_only_source_dependent_method_tail(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        request = {
            "schema_version": "bioxp.operator_method_request.v1",
            "name": "Y then X",
            "idempotency_key": "method-stop-dependency",
            "expected_ownership_generation": 4,
            "failure_policy": "fail_fast",
            "steps": [
                {"action_id": "oem.y.move_steps", "inputs": {"steps": 10}, "repeat": 1},
                {"action_id": "oem.x.move_steps", "inputs": {"steps": 10}, "repeat": 1},
            ],
            "metadata": {},
        }
        method = store.admit_method(request, state=_operator_state())
        active = store.claim_next()
        assert active is not None and active["action_id"] == "oem.y.move_steps"
        admission = store.begin_interrupt(
            "oem.y.stop",
            state=_operator_state(),
            request={
                "idempotency_key": "method-y-stop",
                "observed_ownership_generation": 4,
                "observed_board_epoch_by_board": {},
            },
        )
        store.finalize_interrupt(
            idempotency_key="method-y-stop",
            receipt=admission,
            attempted=True,
            acknowledged=True,
            response={"ok": True, "source_call_completed": True, "controller_command_acknowledged": True},
        )
        rows = store.connection.execute(
            "SELECT action_id,status FROM operator_plane_commands WHERE method_id=? ORDER BY method_sequence",
            (method["method_id"],),
        ).fetchall()
        assert [(row["action_id"], row["status"]) for row in rows] == [
            ("oem.y.move_steps", "stopped"),
            ("oem.x.move_steps", "cleared"),
        ]
        assert store.get_method(method["method_id"])["status"] == "interrupted"
    finally:
        store.stop()




def test_normal_command_and_method_replays_project_current_terminal_state(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        command_request = _action_request("oem.x.move_steps", {"steps": 10}, "replay-current-command")
        command = store.admit_command(command_request, state=_operator_state())
        claimed = store.claim_next()
        assert claimed is not None
        store.finish(command["command_id"], status="completed", payload={"response": {"ok": True}}, claimed=claimed)
        command_replay = store.admit_command(command_request, state=_operator_state())
        assert command_replay["status"] == "completed"
        assert command_replay["idempotent_replay"] is True

        method_request = {
            "schema_version": "bioxp.operator_method_request.v1",
            "name": "One X",
            "idempotency_key": "replay-current-method",
            "expected_ownership_generation": 4,
            "failure_policy": "fail_fast",
            "steps": [{"action_id": "oem.x.move_steps", "inputs": {"steps": 5}, "repeat": 1}],
            "metadata": {},
        }
        method = store.admit_method(method_request, state=_operator_state())
        method_claim = store.claim_next()
        assert method_claim is not None
        store.finish(method_claim["command_id"], status="completed", payload={"response": {"ok": True}}, claimed=method_claim)
        method_replay = store.admit_method(method_request, state=_operator_state())
        assert method_replay["status"] == "completed"
        assert method_replay["idempotent_replay"] is True
    finally:
        store.stop()


def test_sqlite_lock_interrupt_receipt_preserves_observed_authority(tmp_path):
    store = OperatorCommandStore(tmp_path)
    blocker = sqlite3.connect(store.path, isolation_level=None)
    try:
        blocker.execute("BEGIN EXCLUSIVE")
        receipt = store.begin_interrupt(
            "oem.x.stop",
            state=_operator_state(generation=9),
            request={
                "idempotency_key": "locked-stop-observation",
                "observed_ownership_generation": 4,
                "observed_board_epoch_by_board": {"4": 7},
            },
        )
        assert receipt["persistence_state"] == "lock_timeout"
        assert receipt["observed_ownership_generation"] == 4
        assert receipt["observed_board_epoch_by_board"] == {"4": 7}
    finally:
        try:
            blocker.execute("ROLLBACK")
        except sqlite3.Error:
            pass
        blocker.close()
        store.stop()


def test_z_interrupt_controller_call_is_outside_snapshot_exception_handler():
    source = textwrap.dedent(inspect.getsource(Serial206OemInitializationProvider.execute_z_stop_interrupt))
    tree = ast.parse(source)
    parents: dict[ast.AST, ast.AST] = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            parents[child] = node
    calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in {"z_stop", "z_abort"}
    ]
    assert len(calls) == 2
    for call in calls:
        parent = parents.get(call)
        while parent is not None:
            assert not isinstance(parent, ast.ExceptHandler)
            parent = parents.get(parent)




def test_move_to_abs_retries_once_when_first_transport_reply_is_null(monkeypatch):
    driver, _sent = _absolute_driver(
        monkeypatch,
        board=BioXpTester.BOARD_DECK,
        positions=[100],
        wait_result={"ok": True, "event": {"status": 128}},
    )
    replies = iter([None, dict(ACK)])
    calls: list[tuple[Any, ...]] = []

    def send(*args, **_kwargs):
        calls.append(tuple(args))
        return next(replies)

    monkeypatch.setattr(driver, "_send_motor", send)
    result = driver.motor_oem_move_absolute(
        BioXpTester.BOARD_DECK,
        500,
        motor=0,
        wait_for_stop=False,
    )
    assert len(calls) == 2
    assert result["ack"] is None
    assert result["retry_ack"] == ACK
    assert result["low_level_source_return_code"] == 0


def test_move_to_abs_first_null_returns_low_level_zero_even_when_retry_reply_is_error(monkeypatch):
    driver, _sent = _absolute_driver(
        monkeypatch,
        board=BioXpTester.BOARD_DECK,
        positions=[100],
        wait_result={"ok": True, "event": {"status": 128}},
    )
    replies = iter([None, {"status": 13}])
    monkeypatch.setattr(driver, "_send_motor", lambda *_args, **_kwargs: next(replies))

    result = driver.motor_oem_move_absolute(
        BioXpTester.BOARD_DECK, 500, motor=0, wait_for_stop=False
    )

    assert result["ack"] is None
    assert result["retry_ack"] == {"status": 13}
    assert result["low_level_source_return_code"] == 0


def test_initialization_running_receipt_cannot_rebind_request_fingerprint(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    store = OEMRuntimeStore(tmp_path)
    try:
        receipt = {
            "schema_version": "bioxp.serial206_initialization_run.v1",
            "receipt_id": "init-running-fingerprint",
            "command_id": "init-running-fingerprint",
            "run_id": "init-running-fingerprint",
            "idempotency_key": "init-fingerprint-key",
            "idempotency_replay_enabled": True,
            "intent": "initialize_motors",
            "status": "running",
            "started_at": 1.0,
            "finished_at": None,
            "request": {"initialization_kind": "initialize_motors", "timeout_s": 12.0},
            "request_sha256": "a" * 64,
            "response": None,
        }
        store.append_serial206_receipt("initialize_motors", receipt)
        with pytest.raises(sqlite3.IntegrityError):
            store._audit_database.connection.execute(
                "UPDATE serial206_receipts SET status='completed', receipt_json=json_set(receipt_json,'$.status','completed','$.request_sha256',?) WHERE stream='initialize_motors' AND receipt_id=?",
                ("b" * 64, "init-running-fingerprint"),
            )
    finally:
        store._audit_database.connection.close()




def test_oem_query_motor_stop_null_reply_returns_source_zero(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    monkeypatch.setattr(driver, "_send_motor", lambda *_args, **_kwargs: None)

    result = BioXpTester.motor_query_motor_stop(driver, BioXpTester.BOARD_DECK, motor=0)

    assert result["source_return_code"] == 0
    assert result["ok"] is False


def test_oem_query_motor_stop_nonnull_error_returns_source_one(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    monkeypatch.setattr(driver, "_send_motor", lambda *_args, **_kwargs: {"status": 13})

    result = BioXpTester.motor_query_motor_stop(driver, BioXpTester.BOARD_DECK, motor=0)

    assert result["source_return_code"] == 1
    assert result["ok"] is False


def test_oem_reached_position_null_reply_returns_source_zero(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver.motor_get_axis_param = lambda *_args, **_kwargs: {"ack": None, "value": None, "ok": False}

    result = driver.motor_get_reached_position(BioXpTester.BOARD_DECK, motor=0)

    assert result["oem_query_reached_position_return"] == 0
    assert result["ack_success"] is False
    assert result["ok"] is False


def test_oem_reached_position_nonnull_error_returns_source_one(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver.motor_get_axis_param = lambda *_args, **_kwargs: {"ack": {"status": 13}, "value": 0, "ok": False}

    result = driver.motor_get_reached_position(BioXpTester.BOARD_DECK, motor=0)

    assert result["oem_query_reached_position_return"] == 1
    assert result["ack_success"] is False
    assert result["ok"] is False


def test_oem_position_query_null_reply_returns_cached_position(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver._oem_position_cache = {(BioXpTester.BOARD_DECK, 0): 321}
    driver.motor_get_axis_param = lambda *_args, **_kwargs: {"ack": None, "value": None, "ok": False}

    result = BioXpTester.motor_get_position(driver, BioXpTester.BOARD_DECK, motor=0)

    assert result["position"] == 321
    assert result["position_source"] == "oem_cached"
    assert result["position_reply_valid"] is False


def test_oem_position_query_nonnull_error_returns_source_sentinel(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver._oem_position_cache = {(BioXpTester.BOARD_DECK, 0): 321}
    driver.motor_get_axis_param = lambda *_args, **_kwargs: {"ack": {"status": 13}, "value": 999, "ok": False}

    result = BioXpTester.motor_get_position(driver, BioXpTester.BOARD_DECK, motor=0)

    assert result["position"] == 1
    assert result["position_source"] == "oem_error_sentinel"
    assert result["position_reply_valid"] is False


def test_oem_speed_query_retries_once_then_returns_cached_speed(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver._oem_speed_cache = {(BioXpTester.BOARD_DECK, 0): 77}
    calls = []

    def null_then_valid_speed(board, parameter, motor=0):
        calls.append((board, parameter, motor))
        if len(calls) == 1:
            return {"ack": None, "value": None, "ok": False}
        return {"ack": dict(ACK), "value": 999, "ok": True}

    driver.motor_get_axis_param = null_then_valid_speed
    result = BioXpTester.motor_get_speed(driver, BioXpTester.BOARD_DECK, motor=0)

    assert len(calls) == 2
    assert result["speed"] == 77
    assert result["speed_source"] == "oem_cached"
    assert result["speed_reply_valid"] is False


def test_oem_speed_query_returns_error_sentinel_for_nonnull_error_reply(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver._oem_speed_cache = {(BioXpTester.BOARD_DECK, 0): 27}
    calls = []

    def error_speed(board, parameter, motor=0):
        calls.append((board, parameter, motor))
        return {"ack": {"status": 13}, "value": 999, "ok": False}

    driver.motor_get_axis_param = error_speed
    result = BioXpTester.motor_get_speed(driver, BioXpTester.BOARD_DECK, motor=0)

    assert len(calls) == 1
    assert result["speed"] == 100000
    assert result["speed_source"] == "oem_error_sentinel"
    assert result["speed_reply_valid"] is False


def test_oem_home_switch_null_reply_returns_source_scalar_zero_active(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver._oem_motor_home_cache = {(BioXpTester.BOARD_DECK, 0): False}
    driver.motor_get_axis_param = lambda *_args, **_kwargs: {"ack": None, "value": None, "ok": False}

    result = driver.motor_query_home_switch(BioXpTester.BOARD_DECK, motor=0)

    assert result["value"] == 0
    assert result["home"] is True
    assert result["ok"] is False
    assert result["reply_valid"] is False
    assert result["home_source"] == "oem_null_reply_fallback"


def test_oem_home_switch_nonnull_error_reply_is_not_home(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver._oem_motor_home_cache = {(BioXpTester.BOARD_DECK, 0): True}
    driver.motor_get_axis_param = lambda *_args, **_kwargs: {"ack": {"status": 13}, "value": 1, "ok": False}

    result = driver.motor_query_home_switch(BioXpTester.BOARD_DECK, motor=0)

    assert result["value"] == 1
    assert result["home"] is False
    assert result["ok"] is False
    assert result["reply_valid"] is False
    assert result["home_source"] == "oem_error_sentinel"

def test_no24v_go_home_raises_source_exception(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver.oem_no24v_state = lambda: True

    with pytest.raises(RuntimeError, match="Lost 24V power go home"):
        driver.motor_oem_go_home("x", speed=250, rehome=True)


def test_no24v_move_absolute_raises_source_exception(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver.oem_no24v_state = lambda: True

    with pytest.raises(RuntimeError, match="Lost 24V power move abs1"):
        driver.motor_oem_move_absolute(BioXpTester.BOARD_DECK, 500, motor=0)


def test_no24v_move_steps_raises_source_exception(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[0], wait_result={})
    driver.oem_no24v_state = lambda: True

    with pytest.raises(RuntimeError, match="Lost 24V power moveSteps1"):
        driver._motor_oem_move_steps_source(
            board=BioXpTester.BOARD_DECK,
            motor=0,
            axis="x",
            steps=10,
            timeout_s=1.0,
        )


def test_no24v_during_move_absolute_wait_raises_source_exception(monkeypatch):
    driver, _sent = _absolute_driver(
        monkeypatch,
        board=BioXpTester.BOARD_DECK,
        positions=[0],
        wait_result={"ok": False, "failure": "No24V", "no24v": True},
    )

    with pytest.raises(RuntimeError, match="Lost 24V power move abs2"):
        driver.motor_oem_move_absolute(BioXpTester.BOARD_DECK, 500, motor=0)


def test_no24v_during_move_steps_wait_raises_source_exception(monkeypatch):
    driver, _sent = _absolute_driver(
        monkeypatch,
        board=BioXpTester.BOARD_DECK,
        positions=[500, 500],
        wait_result={"ok": False, "failure": "No24V", "no24v": True},
    )
    driver._oem_position_cache = {(BioXpTester.BOARD_DECK, 0): 500}

    with pytest.raises(RuntimeError, match="Lost 24V power moveSteps3"):
        driver._motor_oem_move_steps_source(
            board=BioXpTester.BOARD_DECK,
            motor=0,
            axis="x",
            steps=10,
            timeout_s=1.0,
        )


def test_board_stop_preserves_oem_no24v_and_uninitialized_board_semantics():
    driver = object.__new__(BioXpTester)
    calls: list[str] = []
    driver._oem_board_present = lambda _board: False
    driver.oem_no24v_state = lambda: False
    driver.motor_oem_stop_exact = lambda *_args, **_kwargs: calls.append("leaf") or {}

    absent = driver.motor_oem_board_stop(BioXpTester.BOARD_DECK, motor=0, axis_name="x")
    assert absent["ok"] is True
    assert absent["source_noop"] is True
    assert calls == []

    driver.oem_no24v_state = lambda: True
    with pytest.raises(RuntimeError, match="Lost 24V power stopMotor 1"):
        driver.motor_oem_board_stop(BioXpTester.BOARD_DECK, motor=0, axis_name="x")
    assert calls == []

    states = iter([False, True])
    driver._oem_board_present = lambda _board: True
    driver.oem_no24v_state = lambda: next(states)
    driver.motor_oem_stop_exact = lambda *_args, **_kwargs: calls.append("leaf") or {"ok": True, "source_call_completed": True, "source_return_code": 0}
    with pytest.raises(RuntimeError, match="Lost 24V power stopMotor2"):
        driver.motor_oem_board_stop(BioXpTester.BOARD_HEAD, motor=1, axis_name="z")
    assert calls == ["leaf"]

    calls.clear()
    states = iter([False, False])
    driver.oem_no24v_state = lambda: next(states)
    driver.motor_oem_stop_exact = lambda *_args, **_kwargs: calls.append("leaf") or {
        "ok": True,
        "source_call_completed": True,
        "source_return_code": 0,
    }
    stopped = driver.motor_oem_board_stop(BioXpTester.BOARD_HEAD, motor=1, axis_name="z")
    assert stopped["ok"] is True
    assert stopped["source_return_code"] == 0
    assert calls == ["leaf"]


def test_xyz_stop_providers_use_board_wrapper_not_motor_leaf():
    assert "motor_oem_board_stop" in inspect.getsource(Serial206ProductionPrimitiveAdapter.x_stop)
    assert "motor_oem_board_stop" in inspect.getsource(Serial206ProductionPrimitiveAdapter.z_stop)
    assert "motor_oem_board_stop" in inspect.getsource(Serial206YProvider.stop)


def test_x_stop_void_wrapper_completes_even_when_leaf_return_is_nonzero():
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)

    class Tester:
        @staticmethod
        def _oem_board_present(_board):
            return True

        @staticmethod
        def oem_no24v_state():
            return False

        @staticmethod
        def _oem_board_state():
            return {BioXpTester.BOARD_DECK: True}

        @staticmethod
        def motor_oem_board_stop(*_args, **_kwargs):
            return {
                "ok": False,
                "source_call_completed": True,
                "source_return_code": 1,
                "first_delivery": {"status": 100},
                "second_delivery": {"status": 13},
            }

    adapter.tester = Tester()
    adapter.reference_store = None

    result = adapter.x_stop()

    assert result["ok"] is True
    assert result["source_call_completed"] is True
    assert result["source_return_code"] == 1
    assert result["controller_command_acknowledged"] is False


def test_motion_and_stop_leaves_request_oem_transmit_retry():
    driver = object.__new__(BioXpTester)
    attempts: list[int] = []
    driver.motor_query_motor_stop = lambda *_args, **_kwargs: {"ok": False, "source_return_code": 1}
    driver._send_motor = lambda *_args, **kwargs: attempts.append(int(kwargs["attempts"])) or dict(ACK)

    driver.motor_move_relative(BioXpTester.BOARD_DECK, 10, motor=0)
    driver.motor_move_left(BioXpTester.BOARD_DECK, speed=200, motor=0)
    driver.motor_oem_stop_exact(BioXpTester.BOARD_DECK, motor=0)

    assert attempts == [1, 1, 1, 1]


def test_motion_leaf_timeout_budgets_match_transmit_message():
    driver = object.__new__(BioXpTester)
    timeout_ms: list[int] = []
    driver.motor_query_motor_stop = lambda *_args, **_kwargs: {"ok": False, "source_return_code": 1}
    driver._send_motor = lambda *_args, **kwargs: timeout_ms.append(int(kwargs["read_timeout_ms"]) * int(kwargs["max_reads"])) or dict(ACK)

    driver.motor_move_relative(BioXpTester.BOARD_DECK, 10, motor=0)
    driver.motor_move_left(BioXpTester.BOARD_DECK, speed=200, motor=0)
    driver.motor_oem_stop_exact(BioXpTester.BOARD_DECK, motor=0)

    assert timeout_ms == [60000, 1000, 60000, 60000]


def test_move_to_abs_repeats_full_transmit_after_null(monkeypatch):
    driver, _sent = _absolute_driver(
        monkeypatch,
        board=BioXpTester.BOARD_DECK,
        positions=[500],
        wait_result={"ok": True},
    )
    calls: list[tuple[int, int]] = []
    replies = iter([None, dict(ACK)])
    monkeypatch.setattr(
        driver,
        "_send_motor",
        lambda *_args, **kwargs: calls.append((int(kwargs["attempts"]), int(kwargs["read_timeout_ms"]) * int(kwargs["max_reads"]))) or next(replies),
    )

    result = driver.motor_oem_move_absolute(BioXpTester.BOARD_DECK, 800, motor=0)

    assert result["low_level_source_return_code"] == 0
    assert calls == [(1, 60000), (1, 60000)]


def test_motor_queries_and_profile_writes_use_oem_transmit_budgets_without_readback():
    driver = object.__new__(BioXpTester)
    calls: list[tuple[int, int, int]] = []
    driver._send_motor = lambda _board, command, *_args, **kwargs: calls.append(
        (int(command), int(kwargs["attempts"]), int(kwargs["read_timeout_ms"]) * int(kwargs["max_reads"]))
    ) or dict(ACK)

    driver.motor_query_motor_stop(BioXpTester.BOARD_DECK, motor=0)
    result = driver.motor_set_axis_param(BioXpTester.BOARD_DECK, 4, 100, motor=0)
    driver.motor_set_axis_param(BioXpTester.BOARD_DECK, 5, 100, motor=0)
    driver.motor_set_axis_param(BioXpTester.BOARD_DECK, 6, 100, motor=0)
    driver.motor_get_axis_param(BioXpTester.BOARD_DECK, 4, motor=0)
    driver.motor_query_24v_sensor()
    driver.motor_set_home(BioXpTester.BOARD_DECK, motor=0)

    assert result["readback"] is None
    assert calls == [
        (138, 1, 60000),
        (5, 1, 60000),
        (5, 1, 1000),
        (5, 1, 1000),
        (6, 1, 60000),
        (15, 2, 1000),
        (5, 1, 60000),
    ]


def test_x_leaf_routes_do_not_expose_caller_selected_oem_wait_timeout():
    source = (Path(__file__).parents[1] / "src" / "bioxp" / "api.py").read_text(encoding="utf-8")
    move_steps_model = source[source.index("class OemXMoveStepsRequest"):source.index("class OemXMoveAbsoluteRequest")]
    move_absolute_model = source[source.index("class OemXMoveAbsoluteRequest"):source.index("class OemXReconcileRequest")]
    move_steps_route = source[source.index('async def motion_oem_x_move_steps'):source.index('@app.post("/motion/oem/x/move_absolute")')]
    move_absolute_route = source[source.index('async def motion_oem_x_move_absolute'):source.index('@app.post("/motion/oem/x/manual_home")')]
    assert "wait_timeout_s" not in move_steps_model
    assert "wait_timeout_s" not in move_absolute_model
    assert "wait_timeout_s" not in move_steps_route
    assert "wait_timeout_s" not in move_absolute_route


def test_axis_search_home_checks_no24v_before_controller_io():
    driver = object.__new__(BioXpTester)
    calls: list[str] = []
    driver._motion_oem_axis_profile = lambda *_args, **_kwargs: {"board": 5, "motor": 0}
    driver._oem_board_state = lambda: {5: True}
    driver.oem_no24v_state = lambda: True
    driver.motor_set_home = lambda *_args, **_kwargs: calls.append("set_home") or {"ok": True}
    driver.motor_query_home_switch = lambda *_args, **_kwargs: calls.append("query_home") or {"value": 0}
    driver.motor_get_switch_activity = lambda *_args, **_kwargs: calls.append("switches") or {}

    with pytest.raises(RuntimeError, match="Lost 24V power axisSearchHome"):
        driver.motor_oem_axis_search_home("x", speed=250)

    assert calls == []


def test_axis_search_home_ignores_normal_sethome_and_query_return_codes():
    driver = object.__new__(BioXpTester)
    calls: list[str] = []
    driver._motion_oem_axis_profile = lambda *_args, **_kwargs: {"board": 5, "motor": 0}
    driver._oem_board_state = lambda: {5: True}
    driver.oem_no24v_state = lambda: False
    driver.motor_set_home = lambda *_args, **_kwargs: calls.append("set_home") or {"ok": False, "ack": None, "source_return_code": 0}
    driver.motor_query_home_switch = lambda *_args, **_kwargs: calls.append("query_home") or {"ok": False, "ack": None, "value": 0, "home": True}
    driver.motor_get_switch_activity = lambda *_args, **_kwargs: {}
    driver.motor_oem_move_absolute = lambda *_args, **_kwargs: calls.append("move_absolute") or {"ok": False, "source_return_code": 1}
    driver.motor_wait_stopped = lambda *_args, **_kwargs: calls.append("wait") or {"stopped": False}
    driver.motor_oem_go_home = lambda *_args, **_kwargs: calls.append("go_home") or {"ok": True, "source_return_code": 0}

    result = driver.motor_oem_axis_search_home("x", speed=250)

    assert result["ok"] is True
    assert calls == ["set_home", "query_home", "move_absolute", "go_home"]


def test_initialize_motors_gripper_home_calls_axis_search_without_profile_prepare():
    class Primitives:
        def __init__(self):
            self.calls: list[tuple[Any, ...]] = []

        def motor_oem_axis_search_home(self, axis, **kwargs):
            self.calls.append((axis, kwargs))
            return {"ok": True, "source_return_code": 0}

        def motor_oem_home_axis(self, *_args, **_kwargs):
            raise AssertionError("initializeMotors must not add motor_prepare_axis before gripper axisSearchHome")

    primitives = Primitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 1, sleep=lambda _seconds: None)
    spec = next(row for row in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS if row.key == "gripper-home")

    result = provider._execute_stage(spec, timeout_s=30.0)

    assert result["ok"] is True
    assert primitives.calls == [("g", {"speed": 200, "timeout_s": 30.0, "max_search_abs_delta": None})]


def test_x_rehome_uses_deck_post_move_position_as_tail_return(monkeypatch):
    driver = object.__new__(BioXpTester)
    positions = iter([111, 9999, 8888])
    driver._oem_position_cache = {}
    driver._oem_home_cache = {}
    monkeypatch.setattr(driver, "oem_no24v_state", lambda: False)
    monkeypatch.setattr(driver, "_motion_oem_axis_profile", lambda _axis: {"board": 5, "motor": 0, "axis_max_steps": 100000})
    monkeypatch.setattr(driver, "_oem_board_state", lambda: {5: True})
    monkeypatch.setattr(driver, "motor_get_position", lambda *_args, **_kwargs: {"position": next(positions)})
    monkeypatch.setattr(driver, "motor_query_home_switch", lambda *_args, **_kwargs: {"ack": dict(ACK), "value": 0, "home": False})
    monkeypatch.setattr(driver, "motor_get_speed", lambda *_args, **_kwargs: {"speed": 0})
    monkeypatch.setattr(driver, "motor_oem_move_absolute", lambda *_args, **_kwargs: {"ok": True, "source_return_code": 123})
    monkeypatch.setattr(driver, "begin_bus_event_window", lambda **_kwargs: {})
    monkeypatch.setattr(driver, "motor_move_left", lambda *_args, **_kwargs: {"ack": dict(ACK)})
    monkeypatch.setattr(driver, "motor_wait_stopped", lambda *_args, **_kwargs: {"stopped": True, "last_speed": 0})
    times = iter([0.0, 31.0])
    monkeypatch.setattr("bioxp.usb_driver.time.monotonic", lambda: next(times))

    result = driver.motor_oem_go_home("x", speed=200, rehome=True)

    assert result["source_return_code"] == 9999
    assert result["deck_rehome_position"]["position"] == 9999


def test_go_home_timeout_raises_source_exception(monkeypatch):
    driver, _sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[500], wait_result={})
    driver._oem_motor_home_cache = {(BioXpTester.BOARD_DECK, 0): False}
    driver._oem_position_cache = {(BioXpTester.BOARD_DECK, 0): 500}
    driver.motor_query_home_switch = lambda *_args, **_kwargs: {"ack": ACK, "value": 1, "home": False}
    driver.motor_get_speed = lambda *_args, **_kwargs: {"ack": ACK, "speed": 0, "ok": True}
    driver.motor_move_left = lambda *_args, **_kwargs: {"ack": ACK, "ok": True}
    driver.motor_wait_stopped = lambda *_args, **_kwargs: {"stopped": False}
    driver.motor_oem_stop_exact = lambda *_args, **_kwargs: {"ok": True}

    with pytest.raises(RuntimeError, match="Move to left time out"):
        driver.motor_oem_go_home("x", speed=200, rehome=False)


def test_move_steps_uninitialized_board_returns_public_position_tail(monkeypatch):
    driver, sent = _absolute_driver(monkeypatch, board=BioXpTester.BOARD_DECK, positions=[321], wait_result={})
    driver._oem_board_state = lambda: {BioXpTester.BOARD_DECK: False}
    driver._oem_board_present = lambda selected: int(selected) == BioXpTester.BOARD_DECK

    result = driver._motor_oem_move_steps_source(
        board=BioXpTester.BOARD_DECK,
        motor=0,
        axis="x",
        steps=10,
        timeout_s=1.0,
    )

    assert result["board_wrapper_return"] == 1
    assert result["public_wrapper_return"] == 321
    assert result["source_return_code"] == 321
    assert sent == []


def test_set_home_null_reply_updates_position_only_and_performs_no_gap_readback():
    driver = object.__new__(BioXpTester)
    sent: list[tuple[int, int]] = []
    driver._oem_position_cache = {(BioXpTester.BOARD_DECK, 0): 44}
    driver._oem_motor_home_cache = {(BioXpTester.BOARD_DECK, 0): False}
    driver._send_motor = lambda _board, command, cmd_type, *_args, **_kwargs: sent.append((int(command), int(cmd_type))) or None

    result = driver.motor_set_home(BioXpTester.BOARD_DECK, motor=0)

    assert sent == [(5, 1)]
    assert result["source_return_code"] == 0
    assert driver._oem_position_cache[(BioXpTester.BOARD_DECK, 0)] == 0
    assert driver._oem_motor_home_cache[(BioXpTester.BOARD_DECK, 0)] is False


def test_initialize_motors_x_park_uses_board_event_wait_and_ignores_stop_query_result():
    class Primitives:
        def __init__(self):
            self.calls: list[tuple[Any, ...]] = []

        def motor_oem_move_absolute(self, board, position, **kwargs):
            self.calls.append((board, position, kwargs))
            return {"ok": True, "source_return_code": 0}

        def motor_move_absolute(self, *_args, **_kwargs):
            raise AssertionError("x-park must use board moveToAbs event wait")

        def motor_wait_stopped(self, *_args, **_kwargs):
            raise AssertionError("x-park must not add speed polling")

    primitives = Primitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 1, sleep=lambda _seconds: None)
    spec = next(row for row in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS if row.key == "x-park-6000")

    result = provider._execute_stage(spec, timeout_s=45.0)

    assert result["ok"] is True
    assert primitives.calls == [(5, 6000, {"motor": 0, "wait_for_stop": True})]


def test_ambiguous_terminal_rows_are_immutable_and_recovery_is_append_only(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        admitted = store.admit_command(
            _action_request("oem.x.move_steps", {"steps": 10}, "ambiguous-terminal-immutable"),
            state=_operator_state(),
        )
        command_id = admitted["command_id"]
        with store._authority_write():
            store.connection.execute(
                "UPDATE operator_plane_commands SET status='ambiguous',version=version+1,finished_at=1,updated_at=1,terminal_json='{}' WHERE command_id=?",
                (command_id,),
            )
            store.connection.execute(
                "UPDATE serial206_movement_commands SET state='ambiguous',state_version=state_version+1,finished_at=1,terminal_receipt_id='receipt-1' WHERE command_id=?",
                (command_id,),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE operator_plane_commands SET terminal_json='{""forged"":true}' WHERE command_id=?",
                (command_id,),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE serial206_movement_commands SET terminal_receipt_id='forged' WHERE command_id=?",
                (command_id,),
            )

        store.connection.execute(
            "UPDATE operator_plane_safety SET recovery_epoch=1,recovery_hold=1 WHERE singleton=1"
        )
        result = store.resolve_recovery(
            1,
            {
                "idempotency_key": "recovery-append-only-ack",
                "expected_version": 1,
                "expected_safety_epoch": 0,
                "acknowledge_command_ids": [command_id],
                "operation": "cancel_pending",
            },
        )
        assert result["acknowledged_command_ids"] == [command_id]
        assert store.connection.execute(
            "SELECT status FROM operator_plane_commands WHERE command_id=?", (command_id,)
        ).fetchone()[0] == "ambiguous"
        assert store.connection.execute(
            "SELECT state FROM serial206_movement_commands WHERE command_id=?", (command_id,)
        ).fetchone()[0] == "ambiguous"
        assert store.connection.execute(
            "SELECT COUNT(*) FROM operator_plane_recovery_acknowledgements WHERE command_id=?",
            (command_id,),
        ).fetchone()[0] == 1
    finally:
        store.stop()


def test_xyz_provider_receipts_are_append_only_and_conflicting_replay_is_rejected(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        receipt = {
            "receipt_id": "x-terminal-1",
            "command_id": "x-command-1",
            "idempotency_key": "x-receipt-key-1",
            "intent": "move_steps",
            "status": "completed",
            "controller_command_acknowledged": True,
        }
        store.append_serial206_receipt("x", receipt)
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "UPDATE serial206_receipts SET receipt_json='{}' WHERE stream='x' AND receipt_id='x-terminal-1'"
            )
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "DELETE FROM serial206_receipts WHERE stream='x' AND receipt_id='x-terminal-1'"
            )
        assert store.append_serial206_receipt("x", dict(receipt)) == {**receipt, "idempotency_replay_enabled": True}
        with pytest.raises(ValueError, match="conflicts"):
            store.append_serial206_receipt("x", {**receipt, "status": "failed"})
        forged = json.dumps(
            {"receipt_id": "forged-x", "command_id": "forged-command", "status": "completed"},
            sort_keys=True,
            separators=(",", ":"),
        )
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "INSERT INTO serial206_receipts(stream,receipt_id,command_id,idempotency_key,idempotency_replay_enabled,status,observed_at,receipt_json) VALUES(?,?,?,?,?,?,?,?)",
                ("x", "forged-x", "forged-command", "forged-key", 1, "completed", 2.0, forged),
            )
    finally:
        store.close()


def test_evidence_inserts_bind_exact_bytes_identity_and_interrupt_lineage(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        admitted = store.admit_command(
            _action_request("oem.x.move_steps", {"steps": 10}, "evidence-coherence-command"),
            state=_operator_state(),
        )
        command_id = admitted["command_id"]
        payload = "{}"
        digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "INSERT INTO operator_plane_evidence(evidence_id,command_id,evidence_kind,content_sha256,payload_json,payload_bytes,created_at) VALUES(?,?,?,?,?,?,?)",
                ("forged", command_id, "controller_response", "0" * 64, payload, 2, 1.0),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "INSERT INTO operator_plane_evidence(evidence_id,command_id,evidence_kind,content_sha256,payload_json,payload_bytes,created_at) VALUES(?,?,?,?,?,?,?)",
                ("wrong-identity", command_id, "controller_response", digest, payload, 2, 1.0),
            )
        forged_payload = json.dumps({"forged": True}, sort_keys=True, separators=(",", ":"))
        forged_digest = hashlib.sha256(forged_payload.encode("utf-8")).hexdigest()
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "INSERT INTO operator_plane_evidence(evidence_id,command_id,evidence_kind,content_sha256,payload_json,payload_bytes,created_at) VALUES(?,?,?,?,?,?,?)",
                (
                    f"{command_id}:controller_response:{forged_digest}",
                    command_id,
                    "controller_response",
                    forged_digest,
                    forged_payload,
                    len(forged_payload.encode("utf-8")),
                    1.0,
                ),
            )
        stored = store._store_evidence(
            store.connection,
            command_id=command_id,
            evidence_kind="controller_response",
            payload={},
        )
        assert stored["content_sha256"] == digest

        attempt_id = "standalone-terminal-attempt"
        receipt = json.dumps(
            {"action_id": "oem.x.stop", "interrupt_attempt_id": attempt_id},
            sort_keys=True,
            separators=(",", ":"),
        )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "INSERT INTO operator_plane_interrupt_attempts(interrupt_attempt_id,idempotency_key,fingerprint,action_id,phase,receipt_json,created_at) VALUES(?,?,?,?,?,?,?)",
                (attempt_id, "standalone-key", "0" * 64, "oem.x.stop", "terminal", receipt, 1.0),
            )
    finally:
        store.stop()




def test_direct_sql_cannot_forge_runtime_authority_snapshots_or_journal(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        state = {"schema_version": "bioxp.oem_serial206_initialization_state.v4", "movement_ledger": {}, "used_approvals": {}, "initialize_motion_ledger": {}, "initialization_runs": {}, "z_lifecycle": {"receipts": []}, "x_lifecycle": {"receipts": []}}
        stored = store.write_oem_serial206_initialization_state(state)
        state_json = json.dumps(stored, sort_keys=True, separators=(",", ":"), allow_nan=False)
        state_sha = hashlib.sha256(state_json.encode()).hexdigest()
        with store._lock:
            receipt_json, receipt_sha = store._serial206_receipt_set_locked()
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "INSERT INTO serial206_authority_snapshots(state_json,state_sha256,receipt_set_json,receipt_set_sha256,created_at) VALUES(?,?,?,?,?)",
                (state_json, state_sha, receipt_json, receipt_sha, time.time()),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "INSERT INTO runtime_state_snapshots(sequence,state_json,state_sha256,created_at) VALUES(?,?,?,?)",
                (999999, state_json, state_sha, time.time()),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "INSERT INTO runtime_journal(sequence,stream,payload_json,payload_sha256,created_at) VALUES(?,?,?,?,?)",
                (999999, "forged", state_json, state_sha, time.time()),
            )
    finally:
        store.close()


def test_direct_sql_cannot_forge_board_axis_or_transition_authority(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "UPDATE serial206_board_authority SET state='active',active_board_epoch=99 WHERE board_id=4"
            )
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "UPDATE serial206_axis_authority SET lifecycle_state='referenced_ready',reference_state='referenced' WHERE axis='y'"
            )
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "INSERT INTO serial206_board_transitions(transition_id,requested_active,ownership_generation,delivery_attempted,reply_valid,status_code,continuity_proven,accepted,state_before,state_after,created_at) VALUES('forged',1,0,1,1,100,1,1,'inactive','active',?)",
                (time.time(),),
            )
    finally:
        store.close()


def test_reference_authority_is_canonical_hash_bound_and_direct_sql_guarded(tmp_path):
    database = tmp_path / "bioxp_runtime.db"
    runtime = OEMRuntimeStore(tmp_path)
    runtime.close()
    store = ReferenceStateStore(state_path=database)
    result = store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0))
    assert result["persisted"] is True
    with sqlite3.connect(database) as connection:
        with pytest.raises(sqlite3.OperationalError):
            connection.execute(
                "UPDATE reference_state_authority SET payload_json='{}',payload_sha256=? WHERE authority_key='reference_state'",
                (hashlib.sha256(b"{}").hexdigest(),),
            )
    reloaded = ReferenceStateStore(state_path=database)
    snapshot = reloaded.snapshot(["x"])
    assert snapshot["ok"] is True
    assert snapshot["rows"]["x"]["state"] == "referenced"


def test_serial206_current_state_bounds_oversized_mapping_keys_and_total_bytes(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        huge_key = "k" * 500_000
        state = {
            "movement_ledger": {},
            "used_approvals": {},
            "initialize_motion_ledger": {},
            huge_key: {f"item-{index}": "v" * 10_000 for index in range(100)},
        }
        store.write_oem_serial206_initialization_state(state)
        row = store._db.execute(
            "SELECT state_json FROM serial206_authority_snapshots ORDER BY sequence DESC LIMIT 1"
        ).fetchone()
        assert len(str(row[0]).encode("utf-8")) <= 262_144
        assert huge_key not in str(row[0])
    finally:
        store.close()


def test_post_start_unknown_triggers_block_authorized_runtime_and_operator_writes(tmp_path):
    runtime = OEMRuntimeStore(tmp_path)
    external = sqlite3.connect(runtime.root / "bioxp_runtime.db", isolation_level=None)
    try:
        external.execute(
            "CREATE TRIGGER counterfeit_runtime_piggyback AFTER INSERT ON runtime_metadata BEGIN SELECT 1; END"
        )
        with pytest.raises(RuntimeError, match="physical schema fingerprint mismatch"):
            with runtime._authority_write():
                pass
    finally:
        external.close()
        runtime.close()

    command_store = OperatorCommandStore(tmp_path / "operator")
    external = sqlite3.connect(command_store.path, isolation_level=None)
    try:
        external.execute(
            "CREATE TRIGGER counterfeit_operator_piggyback AFTER INSERT ON operator_plane_metadata BEGIN SELECT 1; END"
        )
        with pytest.raises(RuntimeError, match="physical schema fingerprint mismatch"):
            with command_store._authority_write():
                pass
    finally:
        external.execute("DROP TRIGGER IF EXISTS counterfeit_operator_piggyback")
        external.close()
        command_store.stop()


def test_runtime_restore_rejects_missing_authority_triggers_before_hash_reads(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    store.write_state({"worker": {"state": "idle"}})
    store.write_oem_full_lifecycle_run({"run_id": "hash-run", "idempotency_key": "hash-run-key", "request": {}, "run_state": "completed"})
    store.append_journal("hash-journal", {"event": "ok"})
    database = store._audit_database.path
    store.close()
    with sqlite3.connect(database) as connection:
        for trigger in (
            "runtime_state_snapshots_no_update_v1",
            "runtime_movement_runs_coherence_update_v1",
            "runtime_movement_runs_authorized_update_v2",
            "runtime_journal_no_update_v1",
        ):
            connection.execute(f'DROP TRIGGER "{trigger}"')
        connection.execute("UPDATE runtime_state_snapshots SET state_sha256=?", ("0" * 64,))
        connection.execute("UPDATE runtime_movement_runs SET run_sha256=? WHERE run_id='hash-run'", ("0" * 64,))
        connection.execute("UPDATE runtime_journal SET payload_sha256=? WHERE stream='hash-journal'", ("0" * 64,))
        connection.commit()
    with pytest.raises(RuntimeError, match="runtime authority triggers missing"):
        OEMRuntimeStore(tmp_path)


def test_direct_sql_cannot_forge_method_replay_or_delete_nonterminal_command(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        request = {
            "schema_version": "bioxp.operator_method_request.v1",
            "name": "oem.xy.home",
            "idempotency_key": "method-writer-gate",
            "expected_ownership_generation": 1,
            "expected_board_epoch_by_board": {},
            "failure_policy": "fail_fast",
            "steps": [{"action_id": "oem.xy.home", "inputs": {}, "repeat": 1}],
            "metadata": {"method_action_id": "oem.xy.home"},
        }
        admitted = store.admit_method(request, state={"ownership_generation": 1})
        child = store.connection.execute(
            "SELECT command_id FROM operator_plane_commands WHERE method_id=?",
            (admitted["method_id"],),
        ).fetchone()[0]
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE operator_plane_methods SET status='completed' WHERE method_id=?",
                (admitted["method_id"],),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE serial206_movement_methods SET state='completed' WHERE method_id=?",
                (admitted["method_id"],),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "INSERT INTO operator_plane_idempotency(operation_kind,idempotency_key,fingerprint,command_id,method_id,response_json,created_at) VALUES(?,?,?,?,?,?,?)",
                ("action", "forged-replay", "f" * 64, child, None, "{}", time.time()),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute("DELETE FROM operator_plane_commands WHERE command_id=?", (child,))
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute("DELETE FROM serial206_movement_commands WHERE command_id=?", (child,))
    finally:
        store.stop()


def test_direct_sql_cannot_mutate_command_state_or_publish_incoherent_transition(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        admitted = store.admit_command(
            _action_request("oem.x.move_steps", {"steps": 10}, "state-coherence-command"),
            state=_operator_state(),
        )
        command_id = admitted["command_id"]
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE operator_plane_commands SET status='forged' WHERE command_id=?",
                (command_id,),
            )
        with store._authority_write():
            store.connection.execute(
                "UPDATE operator_plane_commands SET status='dispatched',version=version+1 WHERE command_id=?",
                (command_id,),
            )
            with pytest.raises(sqlite3.IntegrityError):
                store._insert_transition(
                    store.connection,
                    event_kind="forged_transition",
                    command_id=command_id,
                    state="dispatched",
                    payload={},
                )
    finally:
        store.stop()


def test_existing_v2_database_rejects_replaced_authority_trigger(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    db_path = store._audit_database.path
    store.close()

    connection = sqlite3.connect(db_path)
    try:
        connection.execute("DROP TRIGGER serial206_authority_snapshots_coherence_v1")
        connection.execute(
            "CREATE TRIGGER serial206_authority_snapshots_coherence_v1 BEFORE INSERT ON serial206_authority_snapshots WHEN 0 BEGIN SELECT RAISE(ABORT,'weak'); END"
        )
        connection.commit()
    finally:
        connection.close()

    with pytest.raises(RuntimeError, match="runtime authority trigger definition mismatch"):
        OEMRuntimeStore(tmp_path)


def test_axis_reference_publication_treats_generation_as_observational(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        store.record_board4_transition(
            active=True,
            ack={"status": 100},
            transition_id="generation-observational",
            ownership_generation=8,
        )
        prepared = store.prepare_axis_authority(
            "y", ownership_generation=8, profile_fingerprint="generation-observational"
        )
        assert prepared["ok"] is True
        published = store.publish_axis_reference(
            "y",
            position_steps=0,
            ownership_generation=7,
            receipt_id="reference-generation-observational",
        )
        assert published["ok"] is True
        assert published["axis"]["ownership_generation"] == 8
    finally:
        store.close()




def test_normal_command_idempotency_receipt_is_immutable_by_direct_sql(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        store.admit_command(
            _action_request("oem.x.move_steps", {"steps": 10}, "immutable-command-idempotency"),
            state=_operator_state(),
        )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE operator_plane_idempotency SET response_json='{}' WHERE operation_kind='command' AND idempotency_key=?",
                ("immutable-command-idempotency",),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "DELETE FROM operator_plane_idempotency WHERE operation_kind='command' AND idempotency_key=?",
                ("immutable-command-idempotency",),
            )
    finally:
        store.stop()


def test_terminal_command_and_canonical_receipt_are_immutable_by_direct_sql(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        admitted = store.admit_command(
            _action_request("oem.x.move_steps", {"steps": 10}, "immutable-terminal-command"),
            state=_operator_state(),
        )
        command_id = admitted["command_id"]
        with store._authority_write():
            store.connection.execute(
                "UPDATE operator_plane_commands SET status='completed',version=version+1,finished_at=1,updated_at=1,terminal_json='{}' WHERE command_id=?",
                (command_id,),
            )
            store.connection.execute(
                "UPDATE serial206_movement_commands SET state='completed',state_version=state_version+1,finished_at=1,terminal_receipt_id='receipt-1' WHERE command_id=?",
                (command_id,),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE operator_plane_commands SET terminal_json='{""forged"":true}' WHERE command_id=?",
                (command_id,),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE serial206_movement_commands SET terminal_receipt_id='forged' WHERE command_id=?",
                (command_id,),
            )
    finally:
        store.stop()


def test_method_resource_and_dependency_authority_is_immutable_by_direct_sql(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        admitted = store.admit_method(
            {
                "idempotency_key": "immutable-method-graph",
                "expected_ownership_generation": 4,
                "expected_board_epoch_by_board": {},
                "name": "immutable method graph",
                "metadata": {},
                "steps": [
                    {"action_id": "oem.y.move_steps", "inputs": {"steps": 10}},
                    {"action_id": "oem.x.move_steps", "inputs": {"steps": 10}},
                ],
            },
            state=_operator_state(),
        )
        method_id = admitted["method_id"]
        command_ids = [
            str(row[0])
            for row in store.connection.execute(
                "SELECT command_id FROM operator_plane_commands WHERE method_id=? ORDER BY method_sequence",
                (method_id,),
            ).fetchall()
        ]
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "INSERT INTO serial206_command_resources(command_id,resource_key) VALUES(?,?)",
                (command_ids[0], "axis:z"),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "INSERT INTO serial206_command_dependencies(command_id,depends_on_command_id,required_terminal) VALUES(?,?,'completed')",
                (command_ids[0], command_ids[1]),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "DELETE FROM serial206_command_resources WHERE command_id=?",
                (command_ids[0],),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "DELETE FROM serial206_command_dependencies WHERE command_id=?",
                (command_ids[1],),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE operator_plane_methods SET status='completed',version=version+1,updated_at=1 WHERE method_id=?",
                (method_id,),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE serial206_movement_methods SET state='completed',state_version=state_version+1,finished_at=1 WHERE method_id=?",
                (method_id,),
            )
    finally:
        store.stop()


def test_initialization_status_and_receipt_status_must_remain_coherent(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    admission = {
        "schema_version": "bioxp.serial206_initialization_run.v1",
        "receipt_id": "init-coherence-run",
        "command_id": "init-coherence-run",
        "run_id": "init-coherence-run",
        "idempotency_key": "init-coherence-key",
        "idempotency_replay_enabled": True,
        "intent": "initialize_motors",
        "status": "running",
        "started_at": 1.0,
        "request_sha256": "1" * 64,
        "request": {"initialization_kind": "initialize_motors", "timeout_s": 12.0},
        "response": None,
    }
    try:
        store.append_serial206_receipt("initialize_motors", admission)
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "UPDATE serial206_receipts SET status='completed' WHERE stream='initialize_motors' AND receipt_id=?",
                ("init-coherence-run",),
            )
        store.append_serial206_receipt(
            "initialize_motors",
            {**admission, "status": "completed", "finished_at": 2.0, "response": {"ok": True}},
        )
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "UPDATE serial206_receipts SET receipt_json=json_set(receipt_json,'$.response.ok',0) WHERE stream='initialize_motors' AND receipt_id=?",
                ("init-coherence-run",),
            )
    finally:
        store.close()


def test_counterfeit_interrupt_spool_schema_is_rejected(tmp_path):
    path = tmp_path / "operator_interrupt_reconciliation.db"
    connection = sqlite3.connect(path)
    connection.execute(
        "CREATE TABLE interrupt_reconciliation_events(event_sequence INTEGER PRIMARY KEY, interrupt_attempt_id TEXT, phase TEXT CHECK(phase IN ('pending','delivered','reconciled')), payload_json TEXT, content_sha256 TEXT, created_at REAL)"
    )
    connection.close()
    with pytest.raises(RuntimeError, match="interrupt reconciliation schema"):
        OperatorCommandStore(tmp_path)


def test_legacy_interrupt_spool_schema_upgrades_without_losing_pending_identity(tmp_path):
    path = tmp_path / "operator_interrupt_reconciliation.db"
    connection = sqlite3.connect(path)
    connection.executescript(
        """
        CREATE TABLE interrupt_reconciliation_events (
            sequence INTEGER PRIMARY KEY AUTOINCREMENT,
            interrupt_attempt_id TEXT NOT NULL,
            phase TEXT NOT NULL CHECK(phase IN ('pending','reconciled')),
            payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
            content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
            created_at REAL NOT NULL,
            UNIQUE(interrupt_attempt_id,phase)
        );
        """
    )
    payload = json.dumps(
        {
            "action_id": "oem.x.stop",
            "state": _operator_state(),
            "request": {"idempotency_key": "legacy-spool-key"},
            "interrupt_attempt_id": "legacy-spool-attempt",
        },
        sort_keys=True,
        separators=(",", ":"),
    )
    connection.execute(
        "INSERT INTO interrupt_reconciliation_events(interrupt_attempt_id,phase,payload_json,content_sha256,created_at) VALUES(?,?,?,?,1)",
        ("legacy-spool-attempt", "pending", payload, hashlib.sha256(payload.encode("utf-8")).hexdigest()),
    )
    connection.commit()
    connection.close()

    store = OperatorCommandStore(tmp_path)
    try:
        with sqlite3.connect(store._interrupt_spool_path) as spool:
            sql = spool.execute(
                "SELECT sql FROM sqlite_master WHERE type='table' AND name='interrupt_reconciliation_events'"
            ).fetchone()[0]
        assert "'delivered'" in sql
        assert store._pending_interrupt_spool_rows()[0]["interrupt_attempt_id"] == "legacy-spool-attempt"
    finally:
        store.stop()


def test_owner_specific_y_interrupt_fallback_imports_before_startup_recovery(tmp_path):
    fallback = tmp_path / "operator_y_interrupt_fallback.v2.jsonl"
    wrapper = {
        "stream": "y",
        "receipt": {
            "interrupt_attempt_id": "legacy-y-attempt",
            "status": "stopped",
            "persistence_state": "fsync_fallback",
        },
    }
    fallback.write_text(json.dumps(wrapper, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8")

    store = OperatorCommandStore(tmp_path)
    try:
        assert fallback.exists() is False
        row = store.connection.execute(
            "SELECT record_sha256,source_wrapper_json FROM operator_plane_interrupt_history WHERE interrupt_attempt_id='legacy-y-attempt'"
        ).fetchone()
        expected_wrapper = json.dumps(wrapper, sort_keys=True, separators=(",", ":"))
        assert row is not None
        assert row[1] == expected_wrapper
        assert row[0] == hashlib.sha256(expected_wrapper.encode()).hexdigest()
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE operator_plane_interrupt_history SET source_wrapper_json='{}' WHERE interrupt_attempt_id='legacy-y-attempt'"
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "DELETE FROM operator_plane_interrupt_history WHERE interrupt_attempt_id='legacy-y-attempt'"
            )
        assert list(tmp_path.glob("operator_y_interrupt_fallback.v2.imported.*.jsonl"))
    finally:
        store.stop()




def test_interrupt_spool_rejects_forged_reconciliation_without_authoritative_writer(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        payload = json.dumps(
            {
                "interrupt_attempt_id": "forged-spool-attempt",
                "terminal_receipt": {"status": "stopped"},
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        connection = store._interrupt_spool_connection()
        try:
            with pytest.raises(sqlite3.Error):
                connection.execute(
                    "INSERT INTO interrupt_reconciliation_events(interrupt_attempt_id,phase,payload_json,content_sha256,created_at) VALUES(?,?,?,?,?)",
                    (
                        "forged-spool-attempt",
                        "reconciled",
                        payload,
                        hashlib.sha256(payload.encode("utf-8")).hexdigest(),
                        1.0,
                    ),
                )
        finally:
            connection.close()
    finally:
        store.stop()




def test_delivered_interrupt_reconciliation_survives_process_restart(tmp_path):
    first = OperatorCommandStore(tmp_path)
    attempt_id = "durable-reconciliation-attempt"
    request = {
        "idempotency_key": "durable-reconciliation-key",
        "observed_ownership_generation": 4,
        "observed_board_epoch_by_board": {},
    }
    first.queue_pending_interrupt_reconciliation(
        {
            "action_id": "oem.x.stop",
            "state": _operator_state(),
            "request": request,
            "interrupt_attempt_id": attempt_id,
            "attempted": True,
            "acknowledged": True,
            "response": {
                "ok": True,
                "source_call_completed": True,
                "controller_command_acknowledged": True,
            },
            "error": None,
        }
    )
    first.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1"
    )
    first.connection.close()

    second = OperatorCommandStore(tmp_path)
    try:
        assert second.reconcile_pending_interrupts() == 1
        phases = second.connection.execute(
            "SELECT phase FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=? ORDER BY attempt_sequence",
            (attempt_id,),
        ).fetchall()
        assert [row[0] for row in phases] == ["admitted", "terminal"]
        with sqlite3.connect(second._interrupt_spool_path) as spool:
            assert spool.execute(
                "SELECT COUNT(*) FROM interrupt_reconciliation_events WHERE interrupt_attempt_id=? AND phase='reconciled'",
                (attempt_id,),
            ).fetchone()[0] == 1
    finally:
        second.stop()


def test_serial206_admission_state_is_append_only_and_receipt_bound(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    state = {
        "movement_ledger": {},
        "used_approvals": {},
        "initialize_motion_ledger": {},
        "marker": "authoritative",
    }
    try:
        store.write_oem_serial206_initialization_state(state)
        store.append_serial206_receipt(
            "x",
            {
                "receipt_id": "state-bound-receipt",
                "command_id": "state-bound-command",
                "idempotency_key": "state-bound-key",
                "intent": "move_steps",
                "status": "completed",
            },
        )
        assert store.read_oem_serial206_initialization_state()["marker"] == "authoritative"
        rows = store._db.execute(
            "SELECT sequence,receipt_set_json FROM serial206_authority_snapshots ORDER BY sequence"
        ).fetchall()
        assert len(rows) == 2
        assert json.loads(rows[-1]["receipt_set_json"])[0][0:2] == ["x", "state-bound-receipt"]
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "UPDATE serial206_authority_snapshots SET state_json='{}' WHERE sequence=?",
                (rows[-1]["sequence"],),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store._db.execute(
                "INSERT OR REPLACE INTO runtime_metadata(key,value,updated_at) VALUES('serial206_oem_initialization_state','{}',1)"
            )
    finally:
        store.close()


def test_serial206_state_compaction_bounds_arbitrary_scalars_depth_and_cardinality(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        state = {
            "schema_version": "bioxp.oem_serial206_initialization_state.v4",
            "movement_ledger": {
                "large_scalar": "x" * 10000,
                "large_list": list(range(1000)),
                "deep": {"a": {"b": {"c": {"d": {"e": {"f": {"g": {"h": {"i": {"j": {"k": {"l": {"m": 1}}}}}}}}}}}}},
            },
            "used_approvals": {},
            "initialize_motion_ledger": {},
            "z_lifecycle": {"receipts": []},
            "x_lifecycle": {"receipts": []},
        }
        store.write_oem_serial206_initialization_state(state)
        encoded = str(store._db.execute(
            "SELECT state_json FROM serial206_authority_snapshots ORDER BY sequence DESC LIMIT 1"
        ).fetchone()[0])
        assert len(encoded.encode()) < 131072
        assert "x" * 1000 not in encoded
        assert "value_omitted_from_current_state" in encoded
    finally:
        store.close()


def test_serial206_current_state_compacts_raw_controller_packets_but_provider_receipt_keeps_them(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    raw = {"wait": {"ok": False, "events": [{"raw_packet": [1, 2, 3, 4]}]}}
    try:
        store.append_serial206_receipt(
            "x",
            {
                "receipt_id": "raw-provider-receipt",
                "command_id": "raw-provider-command",
                "idempotency_key": "raw-provider-key",
                "intent": "move_steps",
                "status": "failed",
                "response": raw,
            },
        )
        store.write_oem_serial206_initialization_state(
            {
                "movement_ledger": {"stage": raw},
                "used_approvals": {},
                "initialize_motion_ledger": {},
            }
        )
        row = store._db.execute(
            "SELECT state_json FROM serial206_authority_snapshots ORDER BY sequence DESC LIMIT 1"
        ).fetchone()
        assert "raw_packet" not in str(row["state_json"])
        assert "controller_payload_omitted_to_provider_receipt" in str(row["state_json"])
        receipt = store.read_serial206_receipt("x", "raw-provider-command")
        assert receipt["response"] == raw
    finally:
        store.close()


def test_retired_runtime_json_files_are_imported_as_immutable_sqlite_evidence(tmp_path):
    legacy_names = (
        "operator_command_store.json",
        "sequence_state.json",
        "oem_runtime_state.json",
        "oem_full_lifecycle_runs.json",
        "oem_serial206_interrupt_journal.json",
        "oem_initialization_state.json",
    )
    expected: dict[str, bytes] = {}
    for index, name in enumerate(legacy_names):
        content = json.dumps({"legacy": index}, sort_keys=True).encode("utf-8")
        (tmp_path / name).write_bytes(content)
        expected[name] = content

    store = OEMRuntimeStore(tmp_path)
    try:
        rows = store._db.execute(
            "SELECT source_name,content_blob FROM runtime_retired_json_artifacts ORDER BY source_name"
        ).fetchall()
        assert {row["source_name"]: bytes(row["content_blob"]) for row in rows} == expected
        for name in legacy_names:
            assert (tmp_path / name).exists() is False
            assert list(tmp_path.glob(f"{name}.retired.*"))
    finally:
        store.close()


def test_serial206_state_restore_revalidates_state_bytes_and_hash(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        store.write_oem_serial206_initialization_state(
            {
                "movement_ledger": {},
                "used_approvals": {},
                "initialize_motion_ledger": {},
                "marker": "original",
            }
        )
        store._db.execute("DROP TRIGGER serial206_authority_snapshots_no_update_v1")
        store._db.execute("DROP TRIGGER serial206_authority_snapshots_coherence_v1")
        store._db.execute(
            "UPDATE serial206_authority_snapshots SET state_json=? WHERE sequence=(SELECT MAX(sequence) FROM serial206_authority_snapshots)",
            (
                json.dumps(
                    {
                        "movement_ledger": {},
                        "used_approvals": {},
                        "initialize_motion_ledger": {},
                        "marker": "forged",
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                ),
            ),
        )
        with pytest.raises(RuntimeError, match="state bytes"):
            store.read_oem_serial206_initialization_state()
    finally:
        store.close()


def test_runtime_state_runs_and_journals_use_sqlite_without_active_json_files(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    try:
        state = store.write_state({"worker": {"state": "idle", "active_command": None}})
        run = store.write_oem_full_lifecycle_run(
            {"run_id": "sqlite-run-1", "run_state": "planned", "request": {}}
        )
        event = store.append_event({"kind": "sqlite-event"})
        assert not (tmp_path / "runtime_state.json").exists()
        assert not (tmp_path / "movement_runs" / "sqlite-run-1.json").exists()
        assert not (tmp_path / "event_journal.jsonl").exists()
        assert store.read_state()["sequence"] == state["sequence"]
        assert store.read_oem_full_lifecycle_run("sqlite-run-1")["sequence"] == run["sequence"]
        assert store.read_journal("event_journal.jsonl") == [event]
    finally:
        store.close()

    reopened = OEMRuntimeStore(tmp_path)
    try:
        assert reopened.read_state()["sequence"] == state["sequence"]
        assert reopened.list_oem_full_lifecycle_runs()[0]["run_id"] == "sqlite-run-1"
        assert reopened.read_journal("event_journal.jsonl")[0]["kind"] == "sqlite-event"
    finally:
        reopened.close()


def test_interrupt_attempts_are_append_only_and_reused_keys_create_fresh_attempts(tmp_path):
    store = OperatorCommandStore(tmp_path)
    try:
        request = {
            "idempotency_key": "append-only-interrupt-key",
            "observed_ownership_generation": 4,
            "observed_board_epoch_by_board": {},
        }
        first = store.begin_interrupt(
            "oem.x.stop",
            state=_operator_state(),
            request=request,
            interrupt_attempt_id="append-only-attempt-1",
        )
        store.finalize_interrupt(
            idempotency_key=request["idempotency_key"],
            receipt=first,
            attempted=True,
            acknowledged=True,
            response={"ok": True, "source_call_completed": True, "controller_command_acknowledged": True},
        )
        second = store.begin_interrupt(
            "oem.x.stop",
            state=_operator_state(),
            request=request,
            interrupt_attempt_id="append-only-attempt-2",
        )
        assert second["interrupt_attempt_id"] == "append-only-attempt-2"
        assert second["idempotent_replay"] is True
        attempts = store.connection.execute(
            "SELECT interrupt_attempt_id,phase FROM operator_plane_interrupt_attempts WHERE idempotency_key=? ORDER BY attempt_sequence",
            (request["idempotency_key"],),
        ).fetchall()
        assert [tuple(row) for row in attempts] == [
            ("append-only-attempt-1", "admitted"),
            ("append-only-attempt-1", "terminal"),
            ("append-only-attempt-2", "admitted"),
        ]
        assert store.connection.execute(
            "SELECT 1 FROM operator_plane_idempotency WHERE operation_kind='interrupt' AND idempotency_key=?",
            (request["idempotency_key"],),
        ).fetchone() is None
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "UPDATE operator_plane_interrupt_attempts SET receipt_json='{}' WHERE interrupt_attempt_id=?",
                ("append-only-attempt-1",),
            )
        with pytest.raises(sqlite3.IntegrityError):
            store.connection.execute(
                "DELETE FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=?",
                ("append-only-attempt-1",),
            )
    finally:
        store.stop()


def test_operator_history_reader_is_read_only(tmp_path):
    database = tmp_path / "bioxp_runtime.db"
    connection = sqlite3.connect(database)
    connection.execute("CREATE TABLE canary(value TEXT NOT NULL)")
    connection.execute("INSERT INTO canary(value) VALUES('preserved')")
    connection.commit()
    connection.close()
    before = database.read_bytes()

    reader = OperatorHistoryReader(tmp_path)
    try:
        assert reader.get_command("missing-command") is None
        assert reader.get_method("missing-method") is None
        assert reader.list_method_commands("missing-method") == []
    finally:
        reader.close()

    assert database.read_bytes() == before
