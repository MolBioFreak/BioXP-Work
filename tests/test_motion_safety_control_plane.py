from __future__ import annotations

from dataclasses import replace

from fastapi import FastAPI

from bioxp.motion_safety import (
    Serial206MotionAuthority,
    physical_aggregate_stop,
    prepare_motion_without_motion,
)
from bioxp.operator_controls import BoundedReceiptStore, _assess_action, _build_catalog, _dashboard_payload


class FakeMotionDriver:
    MOTOR_FUNCTION_PRESETS = {
        "x": {"board": 5, "motor": 0},
        "y": {"board": 4, "motor": 0},
        "z": {"board": 4, "motor": 1},
        "g": {"board": 4, "motor": 2},
        "door": {"board": 6, "motor": 0},
    }

    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.activation_status = {4: 100, 5: 100, 6: 100}
        self.stop_status = {axis: 100 for axis in self.MOTOR_FUNCTION_PRESETS}
        self.speeds = {axis: 0 for axis in self.MOTOR_FUNCTION_PRESETS}
        self.parameter_readbacks = {
            (5, 0, 4): 1700,
            (4, 0, 4): 1800,
        }
        self.rail = {"ack": {"status": 100}, "reply_valid": True, "sample_valid": True, "safety_valid": True, "oem_scalar": 0}
        self.io = {
            1: {"ack": {"status": 100}, "value": 1, "ok": True},
            3: {"ack": {"status": 100}, "value": 1, "ok": True},
        }

    def board_activate(self, board_id: int):
        self.calls.append(("activate", board_id))
        return {"board": board_id, "ack": {"status": self.activation_status[board_id]}}

    def oem_initialize_without_motion_test_case(self, *, board_wait):
        self.calls.append(("initialize_without_motion", board_wait))
        return {
            "ok": True,
            "physical_motion": False,
            "homing_performed": False,
            "source_anchor": "ClassControlInterface.initializeMotorsWithoutMotion lines 3181-3265",
            "transcript": [
                {"label": "x.setMaxSpeed", "board": 5, "motor": 0, "command": 5, "type": 4, "value": 1700, "ack": {"status": 100}, "ok": True},
                {"label": "y.setMaxSpeed", "board": 4, "motor": 0, "command": 5, "type": 4, "value": 1800, "ack": {"status": 100}, "ok": True},
            ],
            "failures": [],
            "no_replies": [],
        }

    def motor_get_axis_param(self, board_id: int, param: int, *, motor: int):
        self.calls.append(("read_param", board_id, motor, param))
        value = self.parameter_readbacks.get((board_id, motor, param))
        return {"board": board_id, "motor": motor, "param": param, "ack": {"status": 100}, "value": value}

    def motor_query_24v_sensor(self):
        self.calls.append(("query_24v",))
        return self.rail

    def deck_io_query_type(self, io_type: int):
        self.calls.append(("query_io", io_type))
        return self.io[io_type]

    def motor_stop(self, board_id: int, *, motor: int):
        axis = next(name for name, row in self.MOTOR_FUNCTION_PRESETS.items() if row == {"board": board_id, "motor": motor})
        self.calls.append(("stop", axis, board_id, motor))
        status = self.stop_status[axis]
        return {"board": board_id, "motor": motor, "ack": None if status is None else {"status": status}, "ok": status == 100}

    def motor_wait_stopped(self, board_id: int, *, motor: int, timeout_s: float, require_seen_nonzero: bool):
        axis = next(name for name, row in self.MOTOR_FUNCTION_PRESETS.items() if row == {"board": board_id, "motor": motor})
        self.calls.append(("wait_stopped", axis, board_id, motor, timeout_s, require_seen_nonzero))
        speed = self.speeds[axis]
        return {"board": board_id, "motor": motor, "terminal_speed": speed, "ok": speed == 0}


def authority() -> Serial206MotionAuthority:
    return Serial206MotionAuthority.from_snapshot_projection(
        {
            "serial": 206,
            "mutation_authorized": True,
            "lock_sha256": "a" * 64,
            "acquisition_id": "serial-206-test",
        }
    )


def ready_state() -> dict:
    return {
        "ownership_generation": 9,
        "ownership": {"transport": "owned", "usb": "service", "router": "running", "CAN_READY": True},
        "snapshot_id": "snapshot-9",
        "freshness": {"state": "fresh", "age_s": 1.0, "fresh_for_s": 30.0},
        "maintenance": {"motion_blocked": False, "recovery_required": False, "block_reason": None},
        "lifecycle": {"operation_state": "stopped", "door": {"door_closed": True, "latch_closed": True}},
        "references": {"rows": {axis: {"state": "referenced"} for axis in ("x", "y", "z", "g", "door")}},
        "domains": {
            "power": {"status": "observed", "observation": {"safety_valid": True}},
            "interlock": {"status": "observed", "observation": {"motion_arm": {"armed": True}}},
            "latch": {"status": "observed", "observation": {"door_sensor": 1, "latch_sensor": 1}},
        },
    }


def motion_action(path: str = "/motion/axis/move") -> dict:
    return {
        "action_id": "route.motion",
        "kind": "primitive",
        "informational_method": "POST",
        "informational_path": path,
        "safety_class": "motion",
        "provider_available": True,
    }


def test_prepare_without_motion_uses_only_authoritative_motor_boards_and_exact_readback():
    driver = FakeMotionDriver()

    result = prepare_motion_without_motion(driver, authority())

    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert result["homing_performed"] is False
    assert [call for call in driver.calls if call[0] == "activate"] == [("activate", 4), ("activate", 5), ("activate", 6)]
    assert all(call[0] not in {"move", "home", "enable_motor_power"} for call in driver.calls)
    board7 = next(row for row in result["stage_ledger"] if row["stage_id"] == "board_7_resolution")
    assert board7["status"] == "not_applicable"
    assert board7["controller_evidence"]["invalid_command_status"] == 2
    assert board7["controller_evidence"]["command_sent"] is False
    assert [row["stage_id"] for row in result["stage_ledger"]] == [
        "authority",
        "activate_board_4",
        "activate_board_5",
        "activate_board_6",
        "board_7_resolution",
        "initializeMotorsWithoutMotion",
        "parameter_readback",
        "rail_24v_readback",
        "door_readback",
        "latch_readback",
    ]
    assert all(row["status"] in {"passed", "not_applicable"} for row in result["stage_ledger"])


def test_prepare_without_motion_fails_closed_on_parameter_readback_mismatch():
    driver = FakeMotionDriver()
    driver.parameter_readbacks[(5, 0, 4)] = 1699

    result = prepare_motion_without_motion(driver, authority())

    assert result["ok"] is False
    readback = next(row for row in result["stage_ledger"] if row["stage_id"] == "parameter_readback")
    assert readback["status"] == "failed"
    assert readback["controller_evidence"]["mismatches"][0]["expected"] == 1700
    assert result["physical_motion"] is False


def test_prepare_without_motion_fails_fast_on_missing_board_ack_without_initialization():
    driver = FakeMotionDriver()
    driver.activation_status[5] = None

    result = prepare_motion_without_motion(driver, authority())

    assert result["ok"] is False
    assert not any(call[0] == "initialize_without_motion" for call in driver.calls)
    failed = next(row for row in result["stage_ledger"] if row["stage_id"] == "activate_board_5")
    assert failed["status"] == "failed"


def test_physical_aggregate_stop_calls_every_component_and_verifies_ack_and_zero_speed():
    driver = FakeMotionDriver()

    result = physical_aggregate_stop(driver, authority())

    assert result["ok"] is True
    assert result["controller_terminal_state_verified"] is True
    assert result["physical_effect_verified"] is False
    assert result["physical_effect_verification_required"] is True
    assert [row["component"] for row in result["components"]] == ["x", "y", "z", "g", "door"]
    assert [call[1] for call in driver.calls if call[0] == "stop"] == ["x", "y", "z", "g", "door"]
    first_wait = next(index for index, call in enumerate(driver.calls) if call[0] == "wait_stopped")
    assert all(call[0] == "stop" for call in driver.calls[:first_wait])
    assert all(row["stop_acknowledged"] is True and row["zero_speed_verified"] is True for row in result["components"])


def test_physical_aggregate_stop_attempts_all_components_and_fails_on_partial_failure_or_missing_ack():
    for axis, status in (("y", 7), ("g", None)):
        driver = FakeMotionDriver()
        driver.stop_status[axis] = status

        result = physical_aggregate_stop(driver, authority())

        assert result["ok"] is False
        assert result["physical_effect_verified"] is False
        assert [call[1] for call in driver.calls if call[0] == "stop"] == ["x", "y", "z", "g", "door"]
        row = next(item for item in result["components"] if item["component"] == axis)
        assert row["stop_acknowledged"] is False
        assert row["status"] == "failed"


def test_physical_aggregate_stop_is_idempotent_but_redelivers_stop_for_safety():
    driver = FakeMotionDriver()

    first = physical_aggregate_stop(driver, authority())
    second = physical_aggregate_stop(driver, authority())

    assert first["ok"] is True and second["ok"] is True
    assert [call[1] for call in driver.calls if call[0] == "stop"] == ["x", "y", "z", "g", "door"] * 2


def test_catalog_binds_source_grounded_prepare_and_physical_emergency_meta_actions():
    app = FastAPI()

    @app.post("/motion/oem/prepare_without_motion")
    async def prepare():
        return {"ok": True, "physical_motion": False}

    @app.post("/motion/emergency_stop")
    async def emergency():
        return {"ok": True, "physical_effect_verified": True}

    @app.post("/motion/power/enable")
    async def compatibility_enable():
        return {"ok": False}

    actions, dispatch = _build_catalog(app)
    by_id = {row["action_id"]: row for row in actions}

    assert by_id["meta.activate_motion"]["informational_path"] == "/motion/oem/prepare_without_motion"
    assert by_id["meta.activate_motion"]["provider_available"] is True
    assert dispatch["meta.activate_motion"]["path"] == "/motion/oem/prepare_without_motion"
    assert by_id["meta.emergency_stop"]["informational_path"] == "/motion/emergency_stop"
    assert by_id["meta.emergency_stop"]["provider_available"] is True
    assert dispatch["meta.emergency_stop"]["path"] == "/motion/emergency_stop"
    compatibility = next(row for row in actions if row.get("informational_path") == "/motion/power/enable" and row["kind"] == "primitive")
    assert compatibility["enabled"] is False
    assert "global 24 V" in compatibility["disabled_reason"]


def test_dashboard_and_action_admission_share_complete_motion_readiness_predicate():
    state = ready_state()
    assert _assess_action(motion_action(), state, {"axis": "x"})["enabled"] is True
    assert _dashboard_payload(state)["motion"]["enabled"] is True

    mutations = (
        ("transport", lambda row: row["ownership"].update(transport="released"), "Robot transport is unavailable."),
        ("can", lambda row: row["ownership"].update(CAN_READY=None), "Same-epoch CAN readiness has not been established."),
        ("snapshot", lambda row: row.update(snapshot_id=None), "Fresh canonical hardware snapshot is unavailable."),
        ("freshness", lambda row: row["freshness"].update(state="stale"), "Canonical hardware snapshot is stale."),
        ("rail", lambda row: row["domains"]["power"]["observation"].update(safety_valid=False), "24 V rail sensor is not confirmed ready."),
        ("door", lambda row: row["lifecycle"]["door"].update(door_closed=None), "Robot door is not confirmed closed and latched."),
        ("arm", lambda row: row["domains"]["interlock"]["observation"]["motion_arm"].update(armed=False), "Motion arm is not confirmed."),
        ("maintenance", lambda row: row["maintenance"].update(motion_blocked=True), "Motion is inactive. Activate motion before moving this motor."),
        ("reference", lambda row: row["references"]["rows"]["x"].update(state="unknown"), "X axis is not homed."),
    )
    import copy

    for label, mutate, reason in mutations:
        candidate = copy.deepcopy(state)
        mutate(candidate)
        admission = _assess_action(motion_action(), candidate, {"axis": "x"})
        dashboard = _dashboard_payload(candidate)
        assert admission["enabled"] is False, label
        assert dashboard["motion"]["enabled"] is False, label
        assert admission["disabled_reason"] == reason, label
        assert dashboard["motion"]["reason"] == reason, label


def test_reconnect_invalidation_blocks_dashboard_and_admission_with_specific_reason():
    state = ready_state()
    state["ownership"]["CAN_READY"] = None
    state["snapshot_id"] = None
    state["freshness"] = {"state": "missing", "age_s": None, "fresh_for_s": 30.0}

    admission = _assess_action(motion_action(), state, {"axis": "x"})
    dashboard = _dashboard_payload(state)

    assert admission["enabled"] is False
    assert admission["disabled_reason"] == "Same-epoch CAN readiness has not been established."
    assert dashboard["motion"] == {"enabled": False, "reason": "Same-epoch CAN readiness has not been established."}


def test_operator_store_persists_safety_receipts_with_required_evidence(tmp_path, monkeypatch):
    monkeypatch.setattr("bioxp.operator_controls._MAX_RECEIPTS", 2)
    store = BoundedReceiptStore(tmp_path)
    for index in range(3):
        store.put({
            "command_id": f"command-{index}",
            "exact_route": "/motion/emergency_stop",
            "ownership_generation": 9,
            "controller_evidence": {"component_count": 5, "sequence": index},
            "physical_effect_verified": False,
            "controller_terminal_state_verified": index == 2,
            "response": {"ok": index == 2},
        })

    rows = list(reversed(store.list()))
    assert [row["command_id"] for row in rows] == ["command-1", "command-2"]
    receipt = rows[-1]
    assert receipt["exact_route"] == "/motion/emergency_stop"
    assert receipt["ownership_generation"] == 9
    assert receipt["controller_evidence"] == {"component_count": 5, "sequence": 2}
    assert receipt["physical_effect_verified"] is False
    assert receipt["controller_terminal_state_verified"] is True
