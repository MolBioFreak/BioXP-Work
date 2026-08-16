"""Executable software-only contract for serial-206 X OEM parity.

The fakes in this module are deliberately in-memory.  They expose controller-shaped
ACK/readback/event envelopes but never import, construct, or call USB, CAN, serial,
HTTP, DNS, or socket clients.
"""

from __future__ import annotations

import asyncio
import copy
import json
import threading
from collections.abc import Mapping
from typing import Any

import pytest

from bioxp import oem_serial206_initialization as subject
from bioxp.oem_serial206_initialization import (
    Serial206OemInitializationProvider,
    Serial206ProductionPrimitiveAdapter,
)


ACK = {"status": 100, "value": 0}
X_PROFILE = {
    "board": 5,
    "motor": 0,
    "axis_min_steps": 0,
    "axis_max_steps": 90263,
    "speed": 1700,
    "acc": 350,
    "run_current": 31,
    "stall_guard": 16,
}
X_MASK_TUPLE = {12: 1, 13: 0}
TERMINAL_GAPS = (1, 3, 4, 5, 6, 9, 10, 12, 13, 205)
ERROR_EVENTS = (13, 14, 130)


class HashableMapping(Mapping):
    def __init__(self, values):
        self._data = dict(values)

    def __getitem__(self, key):
        return self._data[key]

    def __iter__(self):
        return iter(self._data)

    def __len__(self):
        return len(self._data)

    def __hash__(self):
        return hash(tuple(sorted(self._data.items())))


@pytest.fixture(autouse=True, scope="module")
def isolated_oem_runtime_root(tmp_path_factory):
    root = tmp_path_factory.mktemp("x-oem-contract-runtime")
    with pytest.MonkeyPatch.context() as monkeypatch:
        monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(root))
        yield


class InMemoryReferenceStore:
    def __init__(self, state: str = "referenced") -> None:
        self.state = state
        self.events: list[tuple[str, Any]] = []

    def snapshot(self, axes):
        return {
            "ok": True,
            "durable_clean": True,
            "authority_untrusted": False,
            "rows": {str(axis): {"state": self.state} for axis in axes},
        }

    def record_motion(self, axis, motion_kind):
        self.events.append(("motion", (axis, motion_kind)))
        return {"ok": True, "durable_clean": True, "state": self.state}

    def record_motion_many(self, motions):
        rows = tuple(motions)
        self.events.append(("motion_many", rows))
        return {"ok": True, "durable_clean": True}

    def mark_referenced(self, command):
        self.events.append(("referenced", command))
        self.state = "referenced"
        return {"ok": True, "durable_clean": True, "state": "referenced"}

    def mark_referenced_many(self, commands):
        rows = list(commands)
        self.events.append(("referenced_many", rows))
        self.state = "referenced"
        return {"ok": True, "durable_clean": True}

    def mark_desynced(self, command):
        self.events.append(("desynced", command))
        self.state = "desynced"
        return {"ok": True, "durable_clean": True, "state": "desynced"}

    def mark_desynced_many(self, commands):
        rows = list(commands)
        self.events.append(("desynced_many", rows))
        self.state = "desynced"
        return {"ok": True, "durable_clean": True}

    def recover_untrusted_authority(self, reason):
        self.events.append(("recover", reason))
        return {"ok": True}


class InMemoryXTester:
    """Controller-envelope fake; no hardware or network implementation exists."""

    def __init__(self, *, position: int = 1000) -> None:
        self.positions = {5: int(position), 4: 2000}
        self.params = {
            5: {1: int(position), 3: 0, 4: 1700, 5: 350, 6: 31, 9: 1, 10: 1, 12: 1, 13: 0, 205: 16},
            4: {1: 2000, 3: 0, 4: 1800, 5: 400, 6: 31, 9: 1, 10: 1, 12: 0, 13: 0, 205: 16},
        }
        self.calls: list[tuple[Any, ...]] = []
        self.events: list[dict[str, Any]] = []
        self.sequence = 10
        self.move_ack_status = 100
        self.speed_value: Any = 0
        self.fail_acceleration_setup = False
        self.fail_home_axis: str | None = None
        self.present = {"x": True, "y": True}
        self._oem_board_presence = {4: True, 5: True}

    def _oem_board_present(self, board):
        return bool(self._oem_board_presence.get(int(board), False))

    def _motion_oem_axis_profile(self, axis, startup=True):
        del startup
        if str(axis) == "x":
            return dict(X_PROFILE)
        if str(axis) == "y":
            return {
                "board": 4,
                "motor": 0,
                "axis_min_steps": 0,
                "axis_max_steps": 102956,
                "speed": 1800,
                "acc": 400,
                "run_current": 31,
                "stall_guard": 16,
            }
        raise AssertionError(f"unexpected profile axis {axis}")

    @staticmethod
    def _tmcl_success(row):
        return isinstance(row, Mapping) and row.get("status") == 100

    def motor_get_axis_param(self, board, param, motor=0):
        self.calls.append(("gap", int(board), int(motor), int(param)))
        value = self.params[int(board)][int(param)]
        return {"ok": True, "ack": dict(ACK), "param": int(param), "value": value}

    def motor_set_axis_param(self, board, param, value, motor=0):
        board, param, value, motor = int(board), int(param), int(value), int(motor)
        self.calls.append(("sap", board, motor, param, value))
        readback = value
        if self.fail_acceleration_setup and board == 5 and param == 5 and value != 350:
            readback = value + 1
        else:
            self.params[board][param] = value
        return {
            "ok": True,
            "ack": dict(ACK),
            "set_value": value,
            "readback": {"ok": True, "ack": dict(ACK), "value": readback},
        }

    def motor_get_position(self, board, motor=0):
        self.calls.append(("gap", int(board), int(motor), 1))
        return {"ok": True, "ack": dict(ACK), "position": self.positions[int(board)]}

    def motor_get_speed(self, board, motor=0):
        self.calls.append(("gap", int(board), int(motor), 3))
        return {"ok": True, "ack": dict(ACK), "speed": self.speed_value}

    def begin_bus_event_window(self):
        self.calls.append(("event_window", self.sequence))
        return {"after_sequence": self.sequence}

    def motor_oem_axis_board_present(self, axis):
        return self.present[str(axis)]

    def collect_bus_events(self, **_kwargs):
        self.calls.append(("collect_events",))
        return copy.deepcopy(self.events)

    def motor_oem_move_absolute(self, board, position, motor=0, **_kwargs):
        board, position, motor = int(board), int(position), int(motor)
        self.calls.append(("move_abs", board, motor, position))
        self.positions[board] = position
        self.params[board][1] = position
        return {
            "ok": True,
            "ack": {"status": self.move_ack_status, "value": 0},
            "board": board,
            "motor": motor,
            "position": position,
        }

    def motor_move_relative(self, board, steps, motor=0, **_kwargs):
        board, steps, motor = int(board), int(steps), int(motor)
        self.calls.append(("move_rel", board, motor, steps))
        self.positions[board] += steps
        self.params[board][1] = self.positions[board]
        return {"ok": True, "ack": dict(ACK), "board": board, "motor": motor}

    def motor_wait_target_reached(self, board, motor=0, **_kwargs):
        self.calls.append(("wait_event_128", int(board), int(motor)))
        return {"ok": True, "target_reached": True}

    motor_oem_wait_target_reached = motor_wait_target_reached

    def motor_wait_stopped(self, board, motor=0, **_kwargs):
        self.calls.append(("wait_speed_zero", int(board), int(motor)))
        return {"stopped": True, "last_speed": self.speed_value, "last_ack": dict(ACK)}

    def motor_oem_stop_exact(self, board, motor=0):
        self.calls.append(("stop", int(board), int(motor)))
        return {
            "ok": True,
            "first_delivery": dict(ACK),
            "second_delivery": dict(ACK),
        }

    def motor_oem_force_abort_motion(self, **_kwargs):
        self.calls.append(("abort_all",))
        return {
            "ok": True,
            "physical_scope": "aggregate_oem_all_present_boards",
            "affected_boards": [4, 5, 6],
        }

    def motor_oem_go_home(self, axis, **kwargs):
        self.calls.append(("go_home", str(axis), HashableMapping(kwargs)))
        if self.fail_home_axis == str(axis):
            return {"ok": False, "source_return_code": -77}
        self.positions[5 if str(axis) == "x" else 4] = 0
        return {
            "ok": True,
            "source_return_code": -123 if str(axis) == "x" else -456,
            "home_predicate_confirmed": True,
            "controller_terminal_state_verified": True,
            "controller_home_proof_verified": True,
            "controller_command_acknowledged": True,
        }

    def motor_oem_axis_search_home(self, axis, **kwargs):
        self.calls.append(("axis_search_home", str(axis), HashableMapping(kwargs)))
        self.positions[5 if str(axis) == "x" else 4] = 0
        self.params[5 if str(axis) == "x" else 4][1] = 0
        return {
            "ok": True,
            "source_return_code": -234,
            "home_predicate_confirmed": True,
            "controller_terminal_state_verified": True,
            "controller_home_proof_verified": True,
            "controller_command_acknowledged": True,
        }

    motor_oem_home_axis = motor_oem_axis_search_home

    def motor_oem_set_xy_current_mode(self, enabled):
        self.calls.append(("enable_xy_current", bool(enabled)))
        return {"ok": True, "physical_motion_commanded": False}

    def motor_oem_set_xyz_current_mode(self, enabled, *, z_current_up=31):
        self.calls.append(("enable_xyz_current", bool(enabled), int(z_current_up)))
        return {"ok": True, "physical_motion_commanded": False}

    def motor_set_home(self, board, motor=0):
        self.calls.append(("set_home", int(board), int(motor)))
        self.positions[int(board)] = 0
        self.params[int(board)][1] = 0
        return {"ok": True, "ack": dict(ACK), "readback": {"value": 0}}


def make_adapter(*, tester=None, reference_state="referenced"):
    adapter = Serial206ProductionPrimitiveAdapter.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = tester or InMemoryXTester()
    adapter.reference_store = InMemoryReferenceStore(reference_state)
    adapter.authority_provider = lambda: object()
    adapter.generation_provider = lambda: 17
    adapter._z_profile_overrides = {}
    adapter._x_profile_overrides = {}
    adapter._last_tip_channels = None
    adapter._pending_move_position_before = None
    return adapter


def valid_x_event(sequence=11, status=128, board=5, motor=0, received_at=None):
    row: dict[str, Any] = {"event_sequence": sequence, "status": status, "board": board, "motor": motor}
    if received_at is not None:
        row["received_at"] = float(received_at)
    return row


def seed_x_lifecycle(provider, *, state, reference_state, generation=17, seed_z=False):
    payload = provider._new_state()
    payload["x_lifecycle"].update(
        {
            "state": state,
            "generation": generation,
            "board_lifecycle_generation": 9,
            "reference_state": reference_state,
            "prepared_receipt": {"ok": True, "board_lifecycle_generation": 9},
        }
    )
    if seed_z:
        payload["z_lifecycle"].update({
            "state": "referenced_ready",
            "generation": generation,
            "board_lifecycle_generation": 9,
            "reference_state": "referenced",
            "prepared_receipt": {"ok": True, "board_lifecycle_generation": 9},
            "active_receipt": None,
            "awaiting_observation_receipt_id": None,
        })
    provider._save_state(payload)


# --- Preparation, D1 recovery, and typed terminal status --------------------


def test_x_prepare_selects_x_component_and_exact_oem_profile_without_mask_writes(monkeypatch):
    tester = InMemoryXTester()
    adapter = make_adapter(tester=tester)
    observed = []

    def prepare(_tester, _authority, *, components):
        observed.append(tuple(components))
        return {
            "ok": True,
            "physical_motion": False,
            "board_lifecycle_generation": 9,
            "components": {
                "x": {
                    "board": 5,
                    "motor": 0,
                    "writes": [(4, 1700), (5, 350), (6, 31), (205, 16)],
                    "readbacks": {4: 1700, 5: 350, 6: 31, 205: 16},
                }
            },
        }

    monkeypatch.setattr(subject, "prepare_motion_without_motion", prepare)
    result = adapter.prepare_x(expected_generation=17)

    assert result["ok"] is True
    assert observed == [("x",)]
    receipt = result["receipt"]
    assert receipt["components"]["x"]["writes"] == [[4, 1700], [5, 350], [6, 31], [205, 16]]
    assert not any(call[:4] in {("sap", 5, 0, 12), ("sap", 5, 0, 13)} for call in tester.calls)


def test_d1_recovery_writes_only_sap12_and_verifies_shared_12_13_tuple_then_invalidates():
    tester = InMemoryXTester()
    tester.params[5][12] = 0
    adapter = make_adapter(tester=tester)

    result = adapter.x_reconcile_switch_masks()

    assert result["ok"] is True
    assert result["classification"] == "serial206_machine_safety_adaptation"
    assert result["switch_mask_tuple"] == X_MASK_TUPLE
    assert [call for call in tester.calls if call[0] == "sap"] == [("sap", 5, 0, 12, 1)]
    assert {call for call in tester.calls if call[0] == "gap" and call[-1] in (12, 13)} == {
        ("gap", 5, 0, 12),
        ("gap", 5, 0, 13),
    }
    assert result["preparation_invalidated"] is True
    assert result["reference_invalidated"] is True


def test_x_terminal_status_uses_ack_backed_typed_gap_reads_for_complete_contract():
    tester = InMemoryXTester(position=4321)
    adapter = make_adapter(tester=tester)

    result = adapter.x_terminal_status()

    assert result["ok"] is True
    assert result["position_steps"] == 4321
    assert type(result["speed_steps_s"]) is int and result["speed_steps_s"] == 0
    assert result["switch_mask_tuple"] == X_MASK_TUPLE
    assert set(result["readbacks"]) == set(TERMINAL_GAPS)
    assert all(row["ack"]["status"] == 100 for row in result["readbacks"].values())


def test_x_terminal_status_rejects_boolean_speed_even_though_false_equals_zero():
    tester = InMemoryXTester()
    tester.params[5][3] = False
    adapter = make_adapter(tester=tester)

    result = adapter.x_terminal_status()

    assert result["ok"] is False
    assert result["failure"] == "x_terminal_speed_not_typed_integer_zero"


# --- Relative/absolute bounds, no-op truth, and movement evidence -----------


@pytest.mark.parametrize(
    ("before", "steps", "accepted", "target"),
    [
        (20, -1, False, 19),
        (20, 0, True, 20),
        (20, 1, True, 21),
        (90243, 0, True, 90243),
        (90243, 1, False, 90244),
    ],
)
def test_x_relative_uses_source_inner_margin_20_before_dispatch(before, steps, accepted, target):
    tester = InMemoryXTester(position=before)
    tester.events = [valid_x_event()]
    adapter = make_adapter(tester=tester)

    result = adapter.x_move_steps(steps=steps, wait_timeout_s=20.0)

    assert result["target_position_steps"] == target
    assert result["ok"] is accepted
    moves = [call for call in tester.calls if call[0] == "move_rel"]
    assert bool(moves) is (accepted and steps != 0)
    if not accepted:
        assert result["command_issued"] is False


@pytest.mark.parametrize(("requested", "effective"), [(-1, 60), (0, 60), (59, 60), (60, 60), (90263, 90263), (92049, 90263)])
def test_x_absolute_applies_source_minimum_60_and_release_maximum_90263(requested, effective):
    tester = InMemoryXTester(position=1000)
    tester.events = [valid_x_event()]
    adapter = make_adapter(tester=tester)

    result = adapter.x_move_absolute(position_steps=requested)

    assert result["target_position_steps"] == effective
    assert 60 <= result["target_position_steps"] <= 90263


def test_same_effective_target_is_source_noop_with_terminal_readback_and_no_dispatch_or_event_claim():
    tester = InMemoryXTester(position=60)
    adapter = make_adapter(tester=tester)

    result = adapter.x_move_absolute(position_steps=0)

    assert result["ok"] is True
    assert result["source_noop"] is True
    assert result["physical_motion_commanded"] is False
    assert result["controller_command_acknowledged"] is False
    assert result["target_event_128_observed"] is False
    assert result["after_position_steps"] == 60
    assert type(result["terminal_speed"]["speed"]) is int and result["terminal_speed"]["speed"] == 0
    assert not any(call[0] in {"event_window", "move_abs", "collect_events"} for call in tester.calls)


def test_transport_high_limit_guard_remains_truthful_source_noop_without_false_ack_failure(monkeypatch):
    tester = InMemoryXTester(position=90260)
    adapter = make_adapter(tester=tester)
    monkeypatch.setattr(tester, "motor_oem_move_absolute", lambda *args, **kwargs: {
        "ok": True,
        "source_noop": True,
        "short_circuit": "high_limit_guard",
        "command_sent": False,
        "ack": None,
    })

    result = adapter.x_move_absolute(position_steps=90263)

    assert result["ok"] is True
    assert result["source_noop"] is True
    assert result["noop_reason"] == "high_limit_guard"
    assert result["command_issued"] is False
    assert result["physical_motion_commanded"] is False
    assert result["controller_command_acknowledged"] is False
    assert result["target_event_128_observed"] is False


@pytest.mark.parametrize(
    ("events", "move_ack_status", "speed_value", "expected_failure"),
    [
        ([valid_x_event()], 101, 0, "x_move_command_not_acknowledged"),
        ([valid_x_event(sequence=10)], 100, 0, "x_target_event_128_missing_or_stale"),
        ([valid_x_event(board=4)], 100, 0, "x_target_event_128_missing_or_stale"),
        ([valid_x_event(), valid_x_event(status=13, sequence=12)], 100, 0, "x_controller_error_event"),
        ([valid_x_event(), valid_x_event(status=14, sequence=12)], 100, 0, "x_controller_error_event"),
        ([valid_x_event(), valid_x_event(status=130, sequence=12)], 100, 0, "x_controller_error_event"),
        ([valid_x_event()], 100, False, "x_terminal_zero_speed_not_verified"),
    ],
)
def test_x_move_requires_direct_ack_fresh_addressed_128_no_error_and_typed_speed_zero(
    events, move_ack_status, speed_value, expected_failure
):
    tester = InMemoryXTester(position=1000)
    tester.events = events
    tester.move_ack_status = move_ack_status
    tester.speed_value = speed_value
    adapter = make_adapter(tester=tester)

    result = adapter.x_move_absolute(position_steps=2000)

    assert result["ok"] is False
    assert result["failure"] == expected_failure
    assert len([call for call in tester.calls if call[0] == "move_abs"]) == 1
    assert len([call for call in tester.calls if call[0] == "stop"]) == 1


def test_x_move_accepts_only_full_ack_event_target_speed_contract_and_preserves_reference():
    tester = InMemoryXTester(position=1000)
    tester.events = [valid_x_event()]
    references = InMemoryReferenceStore("referenced")
    adapter = make_adapter(tester=tester)
    adapter.reference_store = references

    result = adapter.x_move_absolute(position_steps=2000)

    assert result["ok"] is True
    assert result["controller_command_acknowledged"] is True
    assert result["target_event_128_observed"] is True
    assert result["controller_error_events"] == []
    assert result["after_position_steps"] == 2000
    assert result["controller_terminal_state_verified"] is True
    assert references.state == "referenced"
    assert not any(kind == "referenced" for kind, _ in references.events)


@pytest.mark.parametrize("exit_case", ["noop", "setup_failure", "command_failure", "event_failure"])
def test_custom_acceleration_restores_x_350_on_every_exit(exit_case):
    before = 60 if exit_case == "noop" else 1000
    tester = InMemoryXTester(position=before)
    tester.events = [valid_x_event()]
    if exit_case == "setup_failure":
        tester.fail_acceleration_setup = True
    elif exit_case == "command_failure":
        tester.move_ack_status = 101
    elif exit_case == "event_failure":
        tester.events = []
    adapter = make_adapter(tester=tester)

    result = adapter.x_move_absolute(position_steps=60 if exit_case == "noop" else 2000, acceleration=444)

    assert ("sap", 5, 0, 5, 350) in tester.calls
    assert result["acceleration_restore"]["readback"]["value"] == 350


# --- Reference authority and distinct home identities ----------------------


class ProviderPrimitives:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, Any]]] = []
        self.reference_store = InMemoryReferenceStore("desynced")
        self.board_generation: int | None = 9

    def current_board_lifecycle_generation(self):
        return self.board_generation

    def prepare_x(self, **kwargs):
        self.calls.append(("prepare", dict(kwargs)))
        return {"ok": True, "physical_motion": False, "board_lifecycle_generation": 9}

    def x_move_absolute(self, **kwargs):
        self.calls.append(("move_absolute", dict(kwargs)))
        return {
            "ok": True,
            "command_issued": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "target_event_128_observed": True,
        }

    def x_move_steps(self, **kwargs):
        self.calls.append(("move_steps", dict(kwargs)))
        return {"ok": True, "command_issued": True, "controller_terminal_state_verified": True}

    def x_manual_panel_home(self, **kwargs):
        self.calls.append(("manual_panel_home", dict(kwargs)))
        return {
            "ok": True,
            "command_issued": True,
            "home_predicate_confirmed": kwargs.get("proved", True),
            "controller_terminal_state_verified": kwargs.get("proved", True),
            "source_return": -123,
            "reference_publication_required": kwargs.get("proved", True),
        }

    def x_stop(self, **kwargs):
        self.calls.append(("stop", dict(kwargs)))
        return {"ok": True, "controller_terminal_state_verified": True}

    def x_abort(self, **kwargs):
        self.calls.append(("abort", dict(kwargs)))
        return {"ok": True, "physical_scope": "aggregate_oem_all_present_boards"}

    def x_set_max_speed(self, **kwargs):
        self.calls.append(("set_max_speed", dict(kwargs)))
        return {"ok": True, "physical_motion": False}

    def x_set_max_acc(self, **kwargs):
        self.calls.append(("set_max_acc", dict(kwargs)))
        return {"ok": True, "physical_motion": False}

    def x_restore_original_speed(self, **kwargs):
        self.calls.append(("restore_original_speed", dict(kwargs)))
        return {"ok": True, "physical_motion": False}

    def x_set_stall_guard(self, **kwargs):
        self.calls.append(("set_stall_guard", dict(kwargs)))
        return {"ok": True, "physical_motion": False}

    def _x_require_motion_preflight(self):
        self.calls.append(("x_live_preflight", {}))
        return {"profile": {"axis": "x"}, "switch_masks": {12: 1, 13: 0}}

    def home_xy(self, **kwargs):
        self.calls.append(("home_xy", dict(kwargs)))
        return {
            "ok": True,
            "controller_terminal_state_verified": True,
            "reference_publication_required": True,
        }

    def x_enable_xy_current_mode(self, **kwargs):
        self.calls.append(("enable_xy_current", dict(kwargs)))
        return {"ok": True, "physical_motion_commanded": False}

    def x_enable_xyz_current_mode(self, **kwargs):
        self.calls.append(("enable_xyz_current", dict(kwargs)))
        return {"ok": True, "physical_motion_commanded": False}


def execute_x_intent(provider, intent, values=None):
    bound_values = dict(values or {})
    bound_values.setdefault("expected_generation", int(provider.generation_provider()))
    return Serial206OemInitializationProvider.execute_x_intent(provider, intent, bound_values)


def test_manual_panel_home_automatically_prepares_unprepared_x_without_motion():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)

    result = execute_x_intent(provider, "manual_panel_home", {"command_id": "x-home-auto-prepare"})

    assert result["ok"] is True
    assert [name for name, _inputs in primitives.calls] == ["prepare", "manual_panel_home"]
    assert result["result"]["automatic_prerequisites"][0]["stage"] == "auto_prepare"
    assert result["result"]["automatic_prerequisites"][0]["physical_motion_commanded"] is False
    assert result["authority_receipt"]["command_id"] == "x-home-auto-prepare"
    assert result["authority_receipt"]["status"] == "completed"
    assert provider.x_projection()["lifecycle"]["state"] == "awaiting_operator_observation"


def test_manual_panel_home_admission_stays_open_while_x_is_automatically_preparable():
    from bioxp import operator_controls

    action = {
        "action_id": "oem.x.manual_panel_home",
        "provider_available": True,
        "required_provider_capability": "initialize_motors",
        "informational_method": "POST",
        "informational_path": "/motion/oem/x/manual_home",
        "safety_class": "motion",
    }
    machine_state = {
        "lifecycle": {"operation_state": "idle"},
        "ownership": {
            "transport": "owned",
            "usb": "service",
            "router": "running",
            "CAN_READY": True,
        },
        "maintenance": {"motion_blocked": False, "recovery_required": False},
        "serial206_initialization_provider": {
            "bound": True,
            "initialize_motors_live_available": True,
            "x_authority": {
                "lifecycle": {
                    "state": "unprepared",
                    "board_lifecycle_generation_fresh": False,
                },
            },
        },
    }

    assessment = operator_controls._assess_action(action, machine_state, {})

    assert assessment["enabled"] is True
    assert assessment["disabled_reason"] is None


def test_x_observation_admission_requires_pending_lifecycle_but_not_motion_readiness():
    from bioxp import operator_controls

    action = {
        "action_id": "oem.x.observe",
        "provider_available": True,
        "required_provider_capability": "initialize_motors",
        "informational_method": "POST",
        "informational_path": "/motion/oem/x/observation",
        "safety_class": "motion",
    }
    machine_state = {
        "lifecycle": {"operation_state": "idle"},
        "ownership": {
            "transport": "owned",
            "usb": "service",
            "router": "running",
            "CAN_READY": True,
        },
        "maintenance": {"motion_blocked": True, "recovery_required": True},
        "serial206_initialization_provider": {
            "bound": True,
            "initialize_motors_live_available": True,
            "x_authority": {
                "lifecycle": {"state": "awaiting_operator_observation"},
            },
        },
    }

    assessment = operator_controls._assess_action(action, machine_state, {})

    assert assessment["enabled"] is True
    assert assessment["disabled_reason"] is None
    assert {row["key"] for row in assessment["dependencies"]} == {
        "provider_available",
        "serial206_x_lifecycle",
        "transport_live",
        "operation_allows_motion",
    }


def test_x_projection_preserves_complete_typed_terminal_status_without_omission_markers():
    class RichTerminalStatus(ProviderPrimitives):
        def x_terminal_status(self):
            readbacks = {}
            for parameter in (1, 3, 4, 5, 6, 9, 10, 12, 13, 205):
                readbacks[parameter] = {
                    "board": 5,
                    "param": parameter,
                    "motor": 0,
                    "ack": {
                        "status": 100,
                        "status_str": "success",
                        "board": 5,
                        "cmd": 6,
                        "value": parameter,
                        "raw": [0] * 9,
                        "provenance": {
                            "transaction_id": f"read-{parameter}",
                            "owner_generation": 17,
                            "generation_changed": False,
                            "ok": True,
                            "outcome": "acknowledged",
                            "matcher": "tmcl_reply",
                            "registration_timestamp": 1.0,
                            "tx_timestamp": 1.0,
                            "tx_write_completed_at": 1.0,
                            "timeout_ms": 1000,
                            "tx_raw": [0] * 9,
                            "tx_frame_count": 1,
                            "tx_frames": [[0] * 9],
                            "tx_write_timestamps": [1.0],
                            "frames": [],
                            "skipped_frames": [],
                        },
                    },
                    "value": parameter,
                }
            return {
                "ok": True,
                "axis": "x",
                "board": 5,
                "motor": 0,
                "position_steps": 100,
                "speed_steps_s": 0,
                "max_speed": 1700,
                "max_acceleration": 350,
                "max_current": 31,
                "left_switch_state": 0,
                "right_switch_state": 0,
                "right_switch_disabled": True,
                "left_switch_disabled": False,
                "stall_guard": 16,
                "profile_verified": True,
                "expected_profile": {4: 1700, 5: 350, 6: 31, 205: 16},
                "switch_mask_verified": True,
                "switch_mask_tuple": {12: 1, 13: 0},
                "expected_switch_masks": {12: 1, 13: 0},
                "readbacks": readbacks,
                "authority": "serial206_x_terminal_register_readback",
                "failure": None,
            }

    projection = Serial206OemInitializationProvider(
        RichTerminalStatus(), generation_provider=lambda: 17
    ).x_projection()

    encoded = json.dumps(projection["live_status"], sort_keys=True)
    assert '"omitted"' not in encoded
    assert {int(key) for key in projection["live_status"]["readbacks"]} == {1, 3, 4, 5, 6, 9, 10, 12, 13, 205}
    assert all(
        row["ack"]["provenance"]["transaction_id"] == f"read-{parameter}"
        for parameter, row in projection["live_status"]["readbacks"].items()
    )


def test_successful_move_cannot_create_reference_from_prepared_unreferenced_state():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")

    result = execute_x_intent(provider, "move_absolute", {"position_steps": 2000, "command_id": "x-no-ref"})

    assert result["ok"] is False
    assert result["failure"] == "x_reference_required_before_move"
    assert primitives.calls == []
    lifecycle = provider.x_projection()["lifecycle"]
    assert lifecycle["state"] == "prepared_unreferenced"
    assert lifecycle["reference_state"] == "desynced"


def test_verified_move_preserves_existing_reference_without_reestablishing_it():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced")

    result = execute_x_intent(provider, "move_absolute", {"position_steps": 2000, "command_id": "x-with-ref"})

    assert result["ok"] is True
    lifecycle = provider.x_projection()["lifecycle"]
    assert lifecycle["state"] == "referenced_ready"
    assert lifecycle["reference_state"] == "referenced"


def test_home_cannot_establish_reference_without_home_proof_and_operator_observation():
    class Unproved(ProviderPrimitives):
        def x_manual_panel_home(self, **kwargs):
            self.calls.append(("manual_panel_home", dict(kwargs)))
            return {
                "ok": True,
                "command_issued": True,
                "home_predicate_confirmed": False,
                "controller_terminal_state_verified": False,
                "source_return": -123,
            }

    primitives = Unproved()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")

    result = execute_x_intent(provider, "manual_panel_home", {"command_id": "x-home-unproved"})

    assert result["ok"] is False
    assert provider.x_projection()["lifecycle"]["reference_state"] == "desynced"


def test_proved_home_enters_awaiting_observation_then_exact_observation_establishes_reference():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")

    home = execute_x_intent(provider, "manual_panel_home", {"command_id": "x-home-proof"})
    assert home["ok"] is True
    assert provider.x_projection()["lifecycle"]["state"] == "awaiting_operator_observation"

    observed = provider.record_x_observation(
        command_id="x-home-proof",
        observation_command_id="x-observe-proof",
        verdict="pass",
        physical_motion_observed=True,
        expected_direction_observed=True,
        home_endpoint_observed=True,
        stopped_observed=True,
        note="X reached its observed home endpoint and stopped.",
        expected_generation=17,
    )
    assert observed["ok"] is True
    assert observed["observation_receipt"]["command_id"] == "x-observe-proof"
    assert observed["observation_receipt"]["observes_command_id"] == "x-home-proof"
    assert observed["observation_receipt"]["status"] == "completed"
    assert provider.x_projection()["lifecycle"]["state"] == "referenced_ready"


@pytest.mark.parametrize(
    ("method", "leaf", "speed", "rehome"),
    [
        ("x_manual_panel_home", "go_home", 500, True),
        ("x_move_to_origin_home", "go_home", 1700, True),
        ("x_caught_plate_recovery_home", "go_home", 1700, False),
        ("x_home_axis", "axis_search_home", 250, False),
    ],
)
def test_x_home_modes_retain_distinct_source_leaf_speed_rehome_and_signed_return(method, leaf, speed, rehome):
    tester = InMemoryXTester(position=1000)
    adapter = make_adapter(tester=tester)

    result = getattr(adapter, method)(timeout_s=30.0)

    matching = [call for call in tester.calls if call[0] == leaf]
    assert len(matching) == 1
    kwargs = matching[0][2]
    assert kwargs["speed"] == speed
    if leaf == "go_home":
        assert kwargs["rehome"] is rehome
    assert type(result["source_return"]) is int
    assert result["source_return"] < 0


def test_compatibility_home_is_retired_instead_of_aliasing_an_oem_mode():
    assert not hasattr(Serial206ProductionPrimitiveAdapter, "x_compatibility_home")


def test_x_startup_home_preserves_250_search_waits_sethome_speed1700_and_park6000():
    tester = InMemoryXTester(position=1000)
    adapter = make_adapter(tester=tester)
    sleeps: list[float] = []
    adapter.sleep = sleeps.append
    tester.events = [valid_x_event()]

    result = adapter.x_startup_home(timeout_s=30.0)

    assert result["ok"] is True
    significant = [call for call in tester.calls if call[0] in {"axis_search_home", "set_home", "sap", "move_abs"}]
    assert significant[0][0] == "axis_search_home"
    assert significant[0][2]["speed"] == 250
    assert significant[1] == ("set_home", 5, 0)
    assert ("sap", 5, 0, 4, 1700) in significant
    assert significant[-1] == ("move_abs", 5, 0, 6000)
    assert sleeps == [0.020, 0.040]
    assert result["controller_position_steps"] == 6000
    assert result["oem_display_position_steps"] == 0


# --- Composite HomeXY and durable lifecycle/replay behavior -----------------


def test_home_xy_uses_200_200_nonrehome_parallel_homes_signed_returns_and_full_restore():
    tester = InMemoryXTester(position=1000)
    adapter = make_adapter(tester=tester)
    adapter.reference_store = None

    result = adapter.home_xy(timeout_s=30.0)

    assert result["ok"] is True
    setup = [call for call in tester.calls if call[0] == "sap" and call[-1] == 200]
    assert set(setup) == {
        ("sap", 5, 0, 4, 200),
        ("sap", 5, 0, 5, 200),
        ("sap", 4, 0, 4, 200),
        ("sap", 4, 0, 5, 200),
    }
    homes = [call for call in tester.calls if call[0] == "go_home"]
    assert {call[1] for call in homes} == {"x", "y"}
    assert all(call[2]["speed"] == 200 and call[2]["rehome"] is False for call in homes)
    assert result["source_return"] == {"x": -123, "y": -456}
    assert not any(call[0] == "set_home" for call in tester.calls), "goHome owns its source setHome"
    assert {
        ("sap", 5, 0, 4, 1700),
        ("sap", 5, 0, 5, 350),
        ("sap", 4, 0, 4, 1800),
        ("sap", 4, 0, 5, 400),
    } <= set(tester.calls)


def test_home_xy_launches_x_and_y_home_leaves_concurrently():
    tester = InMemoryXTester(position=1000)
    adapter = make_adapter(tester=tester)
    adapter.reference_store = None
    entered = threading.Barrier(2, timeout=2.0)
    completed: list[str] = []
    original = tester.motor_oem_go_home

    def overlapping_home(axis, **kwargs):
        entered.wait()
        row = original(axis, **kwargs)
        completed.append(str(axis))
        return row

    tester.motor_oem_go_home = overlapping_home

    result = adapter.home_xy(timeout_s=30.0)

    assert result["ok"] is True
    assert set(completed) == {"x", "y"}


@pytest.mark.parametrize("missing_board", [4, 5])
def test_home_xy_returns_source_null_noop_before_io_when_either_board_absent(missing_board):
    tester = InMemoryXTester(position=1000)
    tester._oem_board_presence[missing_board] = False
    adapter = make_adapter(tester=tester)

    result = adapter.home_xy(timeout_s=5.0)

    assert result["ok"] is True
    assert result["source_noop"] is True
    assert result["source_return"] is None
    assert result["source_return_semantics"] == "null_when_either_board_absent"
    assert result["command_issued"] is False
    assert result["physical_motion_commanded"] is False
    assert result["controller_command_acknowledged"] is False
    assert tester.calls == []


def test_home_xy_restores_both_profiles_and_desyncs_pair_on_partial_failure():
    tester = InMemoryXTester(position=1000)
    tester.fail_home_axis = "y"
    references = InMemoryReferenceStore("referenced")
    adapter = make_adapter(tester=tester)
    adapter.reference_store = references

    result = adapter.home_xy(timeout_s=30.0)

    assert result["ok"] is False
    assert {
        ("sap", 5, 0, 4, 1700),
        ("sap", 5, 0, 5, 350),
        ("sap", 4, 0, 4, 1800),
        ("sap", 4, 0, 5, 400),
    } <= set(tester.calls)
    assert any(kind == "desynced_many" for kind, _ in references.events)
    assert {("stop", 5, 0), ("stop", 4, 0)} <= set(tester.calls)


def test_move_xy_pair_noop_has_fresh_typed_terminal_truth_and_no_motion_metadata():
    tester = InMemoryXTester(position=1000)
    adapter = make_adapter(tester=tester)

    result = adapter.move_xy(1000, 2000, wait_timeout_s=5.0)

    assert result["ok"] is True
    assert result["branch"] == "source_noop"
    assert result["source_noop"] is True
    assert result["command_issued"] is False
    assert result["physical_motion_commanded"] is False
    assert result["controller_command_acknowledged"] is False
    assert result["target_event_128_observed"] is False
    assert result["motion_metadata_recorded"] is False
    assert result["after"] == {"x": 1000, "y": 2000}
    assert result["terminal_speed_steps_s"] == {"x": 0, "y": 0}
    assert not any(call[0] in {"event_window", "move_abs", "collect_events"} for call in tester.calls)


def test_move_xy_missing_x_preserves_literal_source_move_x_y_fallback():
    tester = InMemoryXTester(position=1000)
    tester.present["x"] = False
    adapter = make_adapter(tester=tester)

    result = adapter.move_xy(3000, 4000, wait_timeout_s=5.0)

    assert result["branch"] == "missing_x_calls_moveX_y"
    assert result["fallback"]["requested_position_steps"] == 4000
    assert result["fallback"]["source_mode"] == "moveXY.missing_x.moveX_y"
    assert result["ok"] is False
    assert ("move_abs", 5, 0, 4000) in tester.calls


def test_move_xy_one_axis_records_only_moved_axis_metadata():
    tester = InMemoryXTester(position=1000)
    tester.events = [valid_x_event()]
    references = InMemoryReferenceStore("referenced")
    adapter = make_adapter(tester=tester)
    adapter.reference_store = references

    result = adapter.move_xy(3000, 2000, wait_timeout_s=5.0)

    assert result["ok"] is True
    assert result["moved_axes"] == ["x"]
    assert references.events == [("motion", ("x", "move_xy"))]


class LocalReceiptStateStore:
    def __init__(self) -> None:
        self.state = None
        self.receipts: list[tuple[str, dict[str, Any]]] = []

    def read_oem_serial206_initialization_state(self):
        return copy.deepcopy(self.state)

    def write_oem_serial206_initialization_state(self, state):
        self.state = copy.deepcopy(state)

    def read_serial206_receipt(self, stream, command_id):
        return next((copy.deepcopy(row) for item_stream, row in reversed(self.receipts) if item_stream == stream and row.get("command_id") == command_id), None)

    def read_serial206_receipt_by_idempotency(self, stream, key):
        return next((copy.deepcopy(row) for item_stream, row in reversed(self.receipts) if item_stream == stream and row.get("idempotency_key") == key), None)

    def append_serial206_receipt(self, stream, row):
        self.receipts.append((stream, copy.deepcopy(row)))

    def append_serial206_receipts_atomic(self, receipts):
        stored = []
        for stream, row in receipts:
            payload = copy.deepcopy(row)
            self.receipts.append((stream, payload))
            stored.append(payload)
        return stored


@pytest.mark.parametrize("intent", ["enable_xy_current", "enable_xyz_current"])
def test_provider_current_modes_require_current_prepared_authority_and_preserve_lifecycle(intent):
    primitives = ProviderPrimitives()
    primitives.x_enable_xy_current_mode = lambda **kwargs: primitives.calls.append((intent, dict(kwargs))) or {"ok": True, "physical_motion_commanded": False}
    primitives.x_enable_xyz_current_mode = lambda **kwargs: primitives.calls.append((intent, dict(kwargs))) or {"ok": True, "physical_motion_commanded": False}
    state_store = LocalReceiptStateStore()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17, state_store=state_store)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced", seed_z=intent == "enable_xyz_current")

    if intent == "enable_xyz_current":
        seeded = provider._load_state()
        seeded["z_lifecycle"].update({"state": "prepared_unreferenced", "reference_state": "desynced"})
        provider._save_state(seeded)
    values = {"command_id": f"x-{intent}", "enabled": True}
    if intent == "enable_xyz_current":
        values["z_current_up"] = 29
    result = execute_x_intent(provider, intent, values)

    assert result["ok"] is True
    lifecycle = provider.x_projection()["lifecycle"]
    assert lifecycle["state"] == "referenced_ready"
    assert lifecycle["reference_state"] == "referenced"
    if intent == "enable_xyz_current":
        stored = provider._load_state()
        assert stored["z_lifecycle"]["state"] == "prepared_unreferenced"
        assert stored["z_lifecycle"]["reference_state"] == "desynced"
        assert {stream for stream, row in state_store.receipts if row.get("command_id") == values["command_id"]} == {"x", "z"}


def test_enable_xyz_replay_rejects_z_authority_drift():
    primitives = ProviderPrimitives()
    state_store = LocalReceiptStateStore()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17, state_store=state_store)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced", seed_z=True)
    values = {"command_id": "xyz-replay", "idempotency_key": "xyz-replay", "enabled": True, "z_current_up": 29}
    first = execute_x_intent(provider, "enable_xyz_current", values)
    assert first["ok"] is True

    state = provider._load_state()
    state["z_lifecycle"]["reference_state"] = "desynced"
    provider._save_state(state)
    replay = execute_x_intent(provider, "enable_xyz_current", values)

    assert replay["ok"] is False
    assert replay["failure"] == "x_replay_current_authority_invalid"


def test_enable_xyz_rejects_stale_z_authority_before_dispatch():
    primitives = ProviderPrimitives()
    primitives.x_enable_xyz_current_mode = lambda **_kwargs: (_ for _ in ()).throw(AssertionError("dispatched"))
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced", seed_z=False)

    result = execute_x_intent(provider, "enable_xyz_current", {"command_id": "xyz-stale-z", "enabled": True})

    assert result["ok"] is False
    assert result["failure"] == "xyz_current_requires_current_x_and_z_authority"


def test_provider_diagnostic_home_axis_maps_to_x_home_axis_and_awaits_observation():
    primitives = ProviderPrimitives()
    primitives.x_home_axis = lambda **kwargs: primitives.calls.append(("home_axis", dict(kwargs))) or {
        "ok": True,
        "home_predicate_confirmed": True,
        "controller_terminal_state_verified": True,
        "reference_publication_required": True,
    }
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")

    result = execute_x_intent(provider, "diagnostic_home_axis", {"command_id": "x-diagnostic-home"})

    assert result["ok"] is True
    assert primitives.calls[-1][0] == "home_axis"
    assert provider.x_projection()["lifecycle"]["state"] == "awaiting_operator_observation"


def test_provider_xy_requires_fresh_referenced_authority_before_dispatch():
    primitives = ProviderPrimitives()
    primitives.move_xy = lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("dispatched"))
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")

    result = provider.execute_xy_intent(10, 20, {"command_id": "xy-no-reference"})

    assert result["ok"] is False
    assert result["failure"] == "xy_current_referenced_authority_required"


def test_provider_homexy_requires_current_prepared_authority_and_success_awaits_observation():
    primitives = ProviderPrimitives()
    def successful_homexy(**_kwargs):
        return {
            "ok": True,
            "home_predicate_confirmed": True,
            "controller_terminal_state_verified": True,
            "reference_publication_required": True,
        }
    primitives.home_xy = successful_homexy
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")

    result = provider.execute_homexy_intent({"command_id": "homexy-current"})

    assert result["ok"] is True
    assert result["state"] == "awaiting_operator_observation"
    lifecycle = provider.x_projection()["lifecycle"]
    assert lifecycle["reference_state"] == "desynced"
    receipt = provider._load_state()["x_lifecycle"]["receipts"][-1]
    assert receipt["board_lifecycle_generation"] == 9
    assert primitives.calls[0][0] == "x_live_preflight"


def test_provider_homexy_source_noop_preserves_prior_lifecycle_and_reference():
    primitives = ProviderPrimitives()
    def noop_homexy(**_kwargs):
        return {
            "ok": True,
            "source_noop": True,
            "source_return": None,
            "command_issued": False,
            "physical_motion_commanded": False,
        }
    primitives.home_xy = noop_homexy
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced")

    result = provider.execute_homexy_intent({"command_id": "homexy-null"})

    assert result["ok"] is True
    assert result["state"] == "referenced_ready"
    lifecycle = provider.x_projection()["lifecycle"]
    assert lifecycle["reference_state"] == "referenced"
    assert lifecycle["awaiting_observation_receipt_id"] is None


def test_provider_homexy_preflight_failure_blocks_before_primitive_dispatch():
    primitives = ProviderPrimitives()
    setattr(primitives, "_x_require_motion_preflight", lambda: (_ for _ in ()).throw(RuntimeError("mask drift")))
    setattr(primitives, "home_xy", lambda **_kwargs: (_ for _ in ()).throw(AssertionError("home dispatched")))
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")

    result = provider.execute_homexy_intent({"command_id": "homexy-preflight-fail"})

    assert result["ok"] is False
    assert result["failure"].startswith("homexy_live_preflight_failed")


def test_homexy_observation_atomically_publishes_x_and_y_reference():
    primitives = ProviderPrimitives()
    def observable_homexy(**_kwargs):
        return {
            "ok": True,
            "controller_terminal_state_verified": True,
            "reference_publication_required": True,
        }
    primitives.home_xy = observable_homexy
    references = InMemoryReferenceStore("desynced")
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17, reference_store=references)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")
    home = provider.execute_homexy_intent({"command_id": "homexy-observed"})
    assert home["ok"] is True

    observed = provider.record_x_observation(
        command_id="homexy-observed",
        observation_command_id="homexy-observation",
        verdict="pass",
        physical_motion_observed=True,
        expected_direction_observed=True,
        home_endpoint_observed=True,
        stopped_observed=True,
        note="Both X and Y reached observed home endpoints and stopped.",
        expected_generation=17,
    )

    assert observed["ok"] is True
    kind, commands = references.events[-1]
    assert kind == "referenced_many"
    assert {command.axis for command in commands} == {"x", "y"}


class MemoryStateStore:
    def __init__(self, state=None) -> None:
        self.state = copy.deepcopy(state)
        self.writes = 0
        self.receipts: list[tuple[str, dict[str, Any]]] = []

    def read_oem_serial206_initialization_state(self):
        return copy.deepcopy(self.state)

    def write_oem_serial206_initialization_state(self, state):
        self.writes += 1
        self.state = copy.deepcopy(state)

    def read_serial206_receipt(self, _stream, _command_id):
        return None

    def read_serial206_receipt_by_idempotency(self, _stream, _key):
        return None

    def append_serial206_receipt(self, stream, row):
        self.receipts.append((stream, copy.deepcopy(row)))

    def append_serial206_receipts_atomic(self, receipts):
        stored = []
        for stream, row in receipts:
            payload = copy.deepcopy(row)
            self.receipts.append((stream, payload))
            stored.append(payload)
        return stored

    def append_serial206_interrupt_receipt(self, stream, row):
        self.receipts.append((stream, copy.deepcopy(row)))

    def list_serial206_receipts(self, stream, _limit):
        return [copy.deepcopy(row) for item_stream, row in self.receipts if item_stream == stream]



def test_board5_external_activation_invalidates_x_preparation_reference_and_active_authority():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced")

    result = provider.notify_board_activation(5, {"status": 100}, active=False)

    assert result["x_affected"] is True
    lifecycle = provider.x_projection()["lifecycle"]
    assert lifecycle["state"] == "unprepared"
    assert lifecycle["generation"] is None
    assert lifecycle["prepared_receipt"] is None
    assert lifecycle["reference_state"] == "desynced"


def test_passive_projection_invalidates_stale_board_generation_without_dispatch():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced")
    state = provider._load_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 9
    provider._save_state(state)
    primitives.calls.clear()

    primitives.board_generation = 10
    projection = provider.x_projection()

    assert projection["lifecycle"]["state"] == "unprepared"
    assert projection["lifecycle"]["reference_state"] == "desynced"
    assert projection["lifecycle"]["last_failure"]["failure"] == "x_board_lifecycle_generation_changed"
    assert primitives.calls == []


def test_restart_without_process_local_board_generation_preserves_pending_home_observation():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")
    home = execute_x_intent(provider, "manual_panel_home", {"command_id": "x-home-before-restart"})
    assert home["ok"] is True

    primitives.board_generation = None
    primitives.calls.clear()
    projection = provider.x_projection()

    assert projection["lifecycle"]["state"] == "awaiting_operator_observation"
    assert projection["lifecycle"]["awaiting_observation_receipt_id"] == "x-home-before-restart"
    assert projection["lifecycle"]["board_lifecycle_generation"] == 9
    assert projection["lifecycle"]["last_failure"] is None
    assert primitives.calls == []


def test_restart_recovers_pending_home_observation_after_prior_projection_invalidation():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="prepared_unreferenced", reference_state="desynced")
    home = execute_x_intent(provider, "manual_panel_home", {"command_id": "x-home-recoverable"})
    assert home["ok"] is True
    state = provider._load_state()
    state["x_lifecycle"]["receipts"][-1]["result"] = {
        "ok": {"omitted": "item_limit"},
        "controller_command_acknowledged": True,
        "controller_terminal_state_verified": True,
        "reference_publication_required": {"omitted": "item_limit"},
        "home": {
            "controller_home_proof_verified": True,
            "controller_terminal_state_verified": True,
        },
    }
    state["x_lifecycle"].update(
        {
            "state": "unprepared",
            "generation": None,
            "board_lifecycle_generation": None,
            "prepared_receipt": None,
            "reference_state": "desynced",
            "awaiting_observation_receipt_id": None,
            "last_failure": {
                "failure": "x_board_lifecycle_generation_changed",
                "recorded_generation": 17,
                "current_generation": 17,
                "recorded_board_lifecycle_generation": 9,
                "current_board_lifecycle_generation": None,
            },
        }
    )
    provider._save_state(state)
    primitives.board_generation = None
    primitives.calls.clear()

    projection = provider.x_projection()

    assert projection["lifecycle"]["state"] == "awaiting_operator_observation"
    assert projection["lifecycle"]["generation"] == 17
    assert projection["lifecycle"]["board_lifecycle_generation"] == 9
    assert projection["lifecycle"]["awaiting_observation_receipt_id"] == "x-home-recoverable"
    assert projection["lifecycle"]["last_failure"] is None
    assert primitives.calls == []


def test_restart_without_board_generation_still_invalidates_referenced_x():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced")
    primitives.board_generation = None

    projection = provider.x_projection()

    assert projection["lifecycle"]["state"] == "unprepared"
    assert projection["lifecycle"]["reference_state"] == "desynced"
    assert projection["lifecycle"]["last_failure"]["failure"] == "x_board_lifecycle_generation_changed"
    assert primitives.calls == []


def test_restart_pending_home_without_controller_evidence_is_invalidated():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="awaiting_operator_observation", reference_state="desynced")
    state = provider._load_state()
    state["x_lifecycle"].update(
        {
            "awaiting_observation_receipt_id": "x-home-incomplete",
            "receipts": [
                {
                    "command_id": "x-home-incomplete",
                    "receipt_id": "x-home-incomplete",
                    "intent": "manual_panel_home",
                    "motion_kind": "home",
                    "status": "completed",
                    "generation": 17,
                    "board_lifecycle_generation": 9,
                    "result": {"ok": True},
                }
            ],
        }
    )
    provider._save_state(state)
    primitives.board_generation = None

    projection = provider.x_projection()

    assert projection["lifecycle"]["state"] == "unprepared"
    assert projection["lifecycle"]["awaiting_observation_receipt_id"] is None
    assert projection["lifecycle"]["last_failure"]["failure"] == "x_board_lifecycle_generation_changed"
    assert primitives.calls == []


def test_interrupted_x_transaction_reconciles_during_load_without_dispatch_or_replay():
    primitives = ProviderPrimitives()
    seed = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    state = seed._new_state()
    state["x_lifecycle"].update(
        {
            "state": "executing",
            "generation": 17,
            "reference_state": "referenced",
            "active_receipt": {"command_id": "x-interrupted", "intent": "move_absolute", "status": "executing"},
        }
    )
    store = MemoryStateStore(state)
    provider = Serial206OemInitializationProvider(primitives, state_store=store, generation_provider=lambda: 17)

    loaded = provider._load_state()

    assert loaded["x_lifecycle"]["state"] in {"reconciliation_required", "failed_latched"}
    assert loaded["x_lifecycle"]["reference_state"] == "desynced"
    assert loaded["x_lifecycle"]["active_receipt"] is None
    assert store.writes >= 1
    assert primitives.calls == []


def test_completed_x_replay_is_fenced_after_generation_change_without_redispatch():
    primitives = ProviderPrimitives()
    generation = {"value": 17}
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: generation["value"])
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced")
    values = {"position_steps": 2000, "command_id": "x-once", "idempotency_key": "x-once-key"}
    assert execute_x_intent(provider, "move_absolute", values)["ok"] is True
    dispatches = len(primitives.calls)

    generation["value"] = 18
    replay = execute_x_intent(provider, "move_absolute", values)

    assert replay["ok"] is False
    assert replay["replayed"] is True
    assert len(primitives.calls) == dispatches


def test_completed_x_replay_is_fenced_after_reference_invalidation_without_redispatch():
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state="referenced_ready", reference_state="referenced")
    values = {
        "position_steps": 2000,
        "command_id": "x-ref-once",
        "idempotency_key": "x-ref-once-key",
    }
    assert execute_x_intent(provider, "move_absolute", values)["ok"] is True
    dispatches = len(primitives.calls)
    state = provider._load_state()
    state["x_lifecycle"].update(
        {"state": "prepared_unreferenced", "reference_state": "desynced"}
    )
    provider._save_state(state)

    replay = execute_x_intent(provider, "move_absolute", values)

    assert replay["ok"] is False
    assert replay["replayed"] is True
    assert replay["failure"] == "x_replay_current_authority_invalid"
    assert len(primitives.calls) == dispatches


@pytest.mark.parametrize("intent", ["stop", "abort"])
def test_stop_and_aggregate_abort_are_nonreplay_interrupts(intent):
    primitives = ProviderPrimitives()
    store = MemoryStateStore()
    provider = Serial206OemInitializationProvider(
        primitives,
        state_store=store,
        generation_provider=lambda: 17,
    )
    values = {"command_id": f"x-{intent}-same", "idempotency_key": f"x-{intent}-same-key"}

    first = execute_x_intent(provider, intent, values)
    second = execute_x_intent(provider, intent, values)

    assert first["ok"] is True and second["ok"] is True
    calls = [call for call in primitives.calls if call[0] == intent]
    assert len(calls) == 2
    rows = [row for stream, row in store.receipts if stream == "x" and row["intent"] == intent]
    assert len(rows) == 2
    assert all(row["idempotency_replay_enabled"] is False for row in rows)
    if intent == "abort":
        assert first["result"]["physical_scope"] == "aggregate_oem_all_present_boards"


@pytest.mark.parametrize(
    ("intent", "value", "initial_state", "initial_reference"),
    [
        (intent, value, state, reference)
        for intent, value in (
            ("set_max_speed", 0),
            ("set_max_acc", 0),
            ("restore_original_speed", None),
            ("set_stall_guard", 0),
        )
        for state, reference in (
            ("referenced_ready", "referenced"),
            ("prepared_unreferenced", "desynced"),
        )
    ],
)
def test_x_profile_controls_preserve_reference_and_never_establish_it(
    intent, value, initial_state, initial_reference
):
    primitives = ProviderPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 17)
    seed_x_lifecycle(provider, state=initial_state, reference_state=initial_reference)
    values = {"command_id": f"x-{intent}"}
    if value is not None:
        values["value"] = value

    result = execute_x_intent(provider, intent, values)

    assert result["ok"] is True
    lifecycle = provider.x_projection()["lifecycle"]
    assert lifecycle["state"] == initial_state
    assert lifecycle["reference_state"] == initial_reference


# --- Typed robot/API/operator ownership and retirement of generic X writes --


EXPECTED_X_PATHS = {
    "/motion/oem/x/status",
    "/motion/oem/x/prepare",
    "/motion/oem/x/reconcile_switch_masks",
    "/motion/oem/x/move_steps",
    "/motion/oem/x/move_absolute",
    "/motion/oem/x/manual_home",
    "/motion/oem/x/diagnostic_home_axis",
    "/motion/oem/x/set_home",
    "/motion/oem/x/set_max_speed",
    "/motion/oem/x/set_max_acc",
    "/motion/oem/x/restore_original_speed",
    "/motion/oem/x/set_stall_guard",
    "/motion/oem/x/stop",
    "/motion/oem/x/abort",
    "/motion/oem/x/observation",
    "/motion/oem/x/internal/enable_xy",
    "/motion/oem/x/internal/enable_xyz",
    "/motion/oem/home_xy",
}
EXPECTED_X_ACTIONS = {
    "oem.x.status",
    "oem.x.prepare",
    "oem.x.reconcile_switch_masks",
    "oem.x.move_steps",
    "oem.x.move_absolute",
    "oem.x.manual_panel_home",
    "oem.x.diagnostic_home_axis",
    "oem.x.set_home",
    "oem.x.set_max_speed",
    "oem.x.set_max_acc",
    "oem.x.restore_original_speed",
    "oem.x.set_stall_guard",
    "oem.x.stop",
    "oem.abort_all",
    "oem.x.observe",
    "oem.xy.enable",
    "oem.xyz.enable",
}


def test_openapi_exposes_complete_typed_x_route_family():
    from bioxp import api

    paths = set(api.app.openapi()["paths"])
    assert EXPECTED_X_PATHS <= paths


def test_operator_catalog_exposes_only_typed_x_family_and_aggregate_abort_label():
    from bioxp import api, operator_controls

    actions, _dispatch = operator_controls._build_catalog(api.app)
    by_id = {row["action_id"]: row for row in actions}

    assert EXPECTED_X_ACTIONS <= set(by_id)
    assert "aggregate" in by_id["oem.abort_all"]["label"].lower()
    assert "x-only" not in by_id["oem.abort_all"]["description"].lower()
    assert by_id["oem.abort_all"].get("physical_scope") == "aggregate_oem_all_present_boards"
    assert by_id["oem.x.manual_panel_home"]["requires_confirmation"] is False
    assert all(by_id[action_id]["category"] == "x-axis" for action_id in EXPECTED_X_ACTIONS if action_id.startswith("oem.x."))
    assert all(by_id[action_id]["category"] == "x-composite" for action_id in {"oem.xy.enable", "oem.xyz.enable"})


@pytest.mark.parametrize(
    ("handler_name", "request_name", "payload", "expected_intent"),
    [
        ("motion_oem_manual_relative", "OemManualRelativeRequest", {"axis": "x", "steps": 10}, "move_steps"),
        ("motion_oem_manual_absolute", "OemManualAbsoluteRequest", {"axis": "x", "position_steps": 1000}, "move_absolute"),
        ("motion_oem_manual_home", "OemManualHomeRequest", {"axis": "x"}, "manual_panel_home"),
    ],
)
def test_generic_x_mutation_routes_delegate_to_provider_before_any_tester_access(
    monkeypatch, handler_name, request_name, payload, expected_intent
):
    from bioxp import api

    execute = getattr(api, "_execute_provider_x_intent", None)
    assert callable(execute), "generic X mutation cannot retire until the typed provider dispatcher exists"
    calls = []

    def provider_dispatch(intent, inputs=None):
        calls.append((intent, dict(inputs or {})))
        return {"ok": True, "intent": intent}

    async def inline(_label, fn, **_kwargs):
        return fn()

    monkeypatch.setattr(api, "_execute_provider_x_intent", provider_dispatch)
    monkeypatch.setattr(api, "_run_blocking", inline)
    monkeypatch.setattr(api, "_get_tester", lambda: (_ for _ in ()).throw(AssertionError("generic tester accessed")))

    request = getattr(api, request_name)(**payload)
    result = asyncio.run(getattr(api, handler_name)(request))

    assert result["ok"] is True
    assert calls and calls[0][0] == expected_intent


def test_aggregate_abort_primitive_receipt_never_claims_x_only_scope():
    tester = InMemoryXTester()
    adapter = make_adapter(tester=tester)

    result = adapter.x_abort(reason="operator aggregate OEM abort")

    assert result["ok"] is True
    assert result["intent"] == "aggregate_oem_abort"
    assert result["physical_scope"] == "aggregate_oem_all_present_boards"
    assert result.get("axis") not in {"x", "X"}
