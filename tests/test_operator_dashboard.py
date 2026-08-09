from __future__ import annotations

from fastapi import FastAPI

from bioxp.oem_startup_types import OemDoorEventRequest
from bioxp.operator_controls import _assess_action, _build_catalog, _dashboard_payload, _motor_motion_action, _safety


def machine_state(
    *,
    motion_enabled: bool = False,
    x: str = "unknown",
    y: str = "unknown",
    z: str = "unknown",
    can_ready: bool | None = True,
    power_ready: bool = True,
):
    return {
        "ownership_generation": 12,
        "ownership": {"transport": "owned", "usb": "service", "router": "running", "CAN_READY": can_ready},
        "maintenance": {
            "motion_blocked": not motion_enabled,
            "recovery_required": not motion_enabled,
            "block_reason": None if motion_enabled else "Motion is inactive.",
        },
        "lifecycle": {"operation_state": "stopped", "door": {"door_closed": True, "latch_closed": True}},
        "references": {"rows": {
            "x": {"state": x}, "y": {"state": y}, "z": {"state": z},
            "g": {"state": "referenced"}, "door": {"state": "referenced"},
        }},
        "domains": {
            "power": {"status": "observed", "observation": {"safety_valid": power_ready}},
            "axes": {"status": "observed", "observation": {"rows": {
                "x": {
                    "status": {
                        "position": {"value": 123},
                        "speed": {"value": 0},
                        "max_current": {"value": 31},
                        "standby_current": {"value": 8},
                    },
                    "switch_activity": {
                        "left_state": 0,
                        "right_state": 1,
                        "left_raw_active": False,
                        "right_raw_active": True,
                        "left_disabled": False,
                        "right_disabled": True,
                        "left_effective_active": False,
                        "right_effective_active": False,
                    },
                },
            }}},
            "thermal": {"status": "observed", "observation": {"temps": {"tc_temp_c": {"value": 37000, "ok": True}, "lid_temp_c": {"temp_c": 42.125, "ok": True}, "ped_temp_c": {"value": True, "ok": True}}}},
            "chiller": {"status": "observed", "observation": {"temps": {"rc_temp_c": {"value": 12000, "ok": True}}}},
            "pipette": {"status": "observed", "observation": {"ok": True, "channels": [{"channel": index, "available": True, "initialized": index < 2, "tip_loaded": index == 0} for index in range(4)]}},
            "latch": {"status": "observed", "observation": {"door_sensor": 1, "latch_sensor": 1}},
            "interlock": {"status": "observed", "observation": {"motion_arm": {"armed": True}}},
        },
        "freshness": {"state": "fresh", "age_s": 1.25, "fresh_for_s": 30.0},
        "snapshot_id": "snapshot-1",
    }


def action(path: str, *, method="POST", safety="motion", provider=True, action_id="route.test"):
    return {
        "action_id": action_id,
        "kind": "primitive",
        "informational_method": method,
        "informational_path": path,
        "safety_class": safety,
        "provider_available": provider,
        "provider_unavailable_reason": None if provider else "provider not bound",
    }


def test_provider_owned_z_motion_does_not_depend_on_expiring_global_snapshot():
    state = machine_state(motion_enabled=True, z="referenced")
    state["snapshot_id"] = None
    state["freshness"] = {"state": "missing", "age_s": None, "fresh_for_s": 30.0}
    state["domains"] = {}
    state["serial206_initialization_provider"] = {
        "bound": True,
        "initialize_motors_live_available": True,
        "z_authority": {
            "state": "referenced_ready",
            "reference_state": "referenced",
            "current_board_lifecycle_generation": 4,
            "board_lifecycle_generation": 4,
            "board_lifecycle_generation_fresh": True,
            "terminal_state": {"position_steps": 0, "speed_steps_s": 0},
        },
    }

    home = _assess_action(
        action("/motion/oem/manual/home", action_id="oem.z.manual_home"),
        state,
        {"axis": "z"},
    )
    absolute = _assess_action(
        action("/motion/oem/manual/absolute", action_id="oem.z.move_absolute"),
        state,
        {"axis": "z", "position_steps": 92049},
    )
    generic = _assess_action(action("/motion/axis/move"), state, {"axis": "x"})

    assert home["enabled"] is True
    assert absolute["enabled"] is True
    assert all(row["key"] != "canonical_snapshot" for row in absolute["dependencies"])
    assert generic["enabled"] is False
    assert generic["disabled_reason"] == "Fresh canonical hardware snapshot is unavailable."


def test_provider_owned_z_motion_still_requires_live_transport_can_motion_and_lifecycle():
    state = machine_state(motion_enabled=True, z="referenced")
    state["snapshot_id"] = None
    state["freshness"] = {"state": "missing"}
    state["domains"] = {}
    state["serial206_initialization_provider"] = {
        "bound": True,
        "initialize_motors_live_available": True,
        "z_authority": {"state": "referenced_ready", "reference_state": "referenced"},
    }
    move = action("/motion/oem/manual/relative", action_id="oem.z.move_steps")

    state["ownership"]["CAN_READY"] = None
    assert _assess_action(move, state, {"axis": "z", "steps": 10})["disabled_reason"] == "Same-epoch CAN readiness has not been established."
    state["ownership"]["CAN_READY"] = True
    state["maintenance"]["motion_blocked"] = True
    assert _assess_action(move, state, {"axis": "z", "steps": 10})["disabled_reason"] == "Motion is inactive. Activate motion before moving this motor."
    state["maintenance"] = {"motion_blocked": False, "recovery_required": False}
    state["serial206_initialization_provider"]["z_authority"]["state"] = "unprepared"
    assert "Current Z lifecycle state" in _assess_action(move, state, {"axis": "z", "steps": 10})["disabled_reason"]


def test_emergency_lifecycle_blocks_dashboard_and_normal_motion_but_keeps_recovery_available():
    state = machine_state(motion_enabled=True, x="referenced", y="referenced", z="referenced")
    state["lifecycle"].update(
        operation_state="emergency",
        operation_reason="emergency_stop:operator_request",
    )
    state["serial206_initialization_provider"] = {
        "bound": True,
        "initialize_motors_live_available": True,
        "z_authority": {
            "state": "referenced_ready",
            "reference_state": "referenced",
            "board_lifecycle_generation_fresh": True,
        },
    }
    generic_move = action("/motion/axis/move")
    provider_clear = action(
        "/motion/oem/z/clear",
        action_id="oem.z.clear",
    )
    emergency_stop = action(
        "/motion/emergency_stop",
        safety="emergency",
        action_id="meta.emergency_stop",
    )
    activate = action(
        "/motion/oem/prepare_without_motion",
        safety="service",
        action_id="meta.activate_motion",
    )

    reason = "Motion is blocked while operation state is emergency."
    assert _dashboard_payload(state)["motion"] == {"enabled": False, "reason": reason}
    for candidate in (generic_move, provider_clear):
        assessed = _assess_action(candidate, state, {})
        assert assessed["enabled"] is False
        assert assessed["disabled_reason"] == reason
        assert next(row for row in assessed["dependencies"] if row["key"] == "operation_allows_motion")["met"] is False
    assert _assess_action(emergency_stop, state, {})["enabled"] is True
    assert _assess_action(activate, state, {})["enabled"] is True


def test_activate_motion_is_disabled_when_recovery_is_already_complete():
    state = machine_state(motion_enabled=True)
    activate = action(
        "/motion/oem/prepare_without_motion",
        safety="service",
        action_id="meta.activate_motion",
    )

    assessed = _assess_action(activate, state, {})

    assert assessed["enabled"] is False
    assert assessed["disabled_reason"] == "Motion is already active."
    state["maintenance"] = {"motion_blocked": True, "recovery_required": True}
    assert _assess_action(activate, state, {})["enabled"] is True


def test_local_only_maintenance_routes_are_visible_but_never_dispatchable():
    app = FastAPI()

    @app.post("/maintenance/usb/reconnect")
    async def local_reconnect():
        return {"ok": True}

    @app.post("/reconnect")
    async def public_reconnect():
        return {"ok": True}

    actions, dispatch = _build_catalog(app)
    by_path = {action["informational_path"]: action for action in actions if action["kind"] == "primitive"}

    local = by_path["/maintenance/usb/reconnect"]
    assert local["provider_available"] is False
    assert local["available"] is False
    assert local["enabled"] is False
    assert local["provider_unavailable_reason"] == "Local-only maintenance route is not callable through the operator relay."
    assert local["action_id"] not in dispatch

    public = by_path["/reconnect"]
    assert public["provider_available"] is True
    assert public["action_id"] in dispatch


def test_operator_catalog_removes_non_oem_session_id_requirement_but_keeps_latest_status():
    app = FastAPI()

    @app.get("/oem/startup/status/latest")
    async def latest_status():
        return {"ok": True}

    @app.get("/oem/startup/status/{session_id}")
    async def compatibility_status(session_id: str):
        return {"session_id": session_id}

    @app.post("/oem/startup/door_event")
    async def door_event(request: OemDoorEventRequest):
        return request.model_dump()

    actions, dispatch = _build_catalog(app)
    primitives = [row for row in actions if row["kind"] == "primitive"]
    paths = {row["informational_path"] for row in primitives}

    assert "/oem/startup/status/latest" in paths
    assert "/oem/startup/status/{session_id}" not in paths
    assert all(input_row["name"] != "session_id" for row in primitives for input_row in row["inputs"])
    assert all(route["path"] != "/oem/startup/status/{session_id}" for route in dispatch.values())


def test_motion_inactive_blocks_motor_axis_gripper_door_and_pipette_motion_with_obvious_reason():
    state = machine_state(motion_enabled=False, x="referenced", y="referenced", z="referenced")
    for path in (
        "/motion/axis/move",
        "/motion/gripper/open",
        "/motion/thermal-door/close",
        "/pipette/aspirate",
        "/liquid/aspirate",
    ):
        assessed = _assess_action(action(path), state, {"axis": "x"})
        assert assessed["enabled"] is False
        assert assessed["disabled_reason"] == "Motion is inactive. Activate motion before moving this motor."
        assert next(row for row in assessed["dependencies"] if row["key"] == "motion_enabled")["met"] is False


def test_incomplete_maintenance_state_fails_closed_for_every_motion_class():
    paths = (
        "/motion/axis/move",
        "/motion/gripper/open",
        "/pipette/aspirate",
        "/latch/lock",
        "/protocol/execute",
        "/oem/startup/initialize_environment",
        "/oem/initial_check",
    )
    for missing_value in (None, "missing"):
        state = machine_state(motion_enabled=True, x="referenced", y="referenced", z="referenced")
        if missing_value == "missing":
            del state["maintenance"]["recovery_required"]
        else:
            state["maintenance"]["recovery_required"] = None
        for path in paths:
            row = action(path, safety=_safety("POST", path))
            assessed = _assess_action(row, state, {"axis": "x"})
            assert assessed["enabled"] is False, (missing_value, path)
            assert assessed["disabled_reason"] == "Motion is inactive. Activate motion before moving this motor.", (
                missing_value,
                path,
            )


def test_full_production_catalog_motion_primitives_fail_closed_on_incomplete_maintenance():
    from bioxp import api

    actions, _dispatch = _build_catalog(api.app)
    motion_actions = [
        row
        for row in actions
        if row["kind"] == "primitive" and row["safety_class"] == "motion" and row["provider_available"] is True
    ]
    assert len(motion_actions) >= 48

    for missing_value in (None, "missing"):
        state = machine_state(motion_enabled=True, x="referenced", y="referenced", z="referenced")
        if missing_value == "missing":
            del state["maintenance"]["recovery_required"]
        else:
            state["maintenance"]["recovery_required"] = None
        admitted = []
        for row in motion_actions:
            assessed = _assess_action(row, state, {})
            if assessed["enabled"]:
                admitted.append(row["informational_path"])
        assert admitted == [], (missing_value, admitted)
        dashboard = _dashboard_payload(state)
        assert dashboard["motion"]["enabled"] is False, missing_value
        assert dashboard["motion"]["reason"] == "Motion is inactive. Activate motion before moving this motor.", missing_value


def test_non_motion_thermal_and_chiller_controls_are_service_classed():
    for path in ("/thermal/set_temp", "/thermal/rates", "/chiller/set_temp", "/chiller/fan"):
        row = action(path, safety=_safety("POST", path))
        assert row["safety_class"] == "service", path
        assert _motor_motion_action(row) is False, path


def test_read_only_and_stop_controls_do_not_require_motion_enabled():
    state = machine_state(motion_enabled=False)
    read = _assess_action(action("/motion/axis/x/status", method="GET", safety="read_only"), state, {})
    stop = _assess_action(action("/motion/axis/x/stop", safety="stop"), state, {})
    assert read["enabled"] is True
    assert stop["enabled"] is True
    state["ownership"]["transport"] = "released"
    stop_without_transport = _assess_action(action("/motion/axis/x/stop", safety="stop"), state, {})
    assert stop_without_transport["enabled"] is False
    assert stop_without_transport["disabled_reason"] == "Robot transport is unavailable."


def test_bootstrap_controls_do_not_require_the_state_they_establish():
    state = machine_state(
        motion_enabled=True,
        x="referenced",
        y="referenced",
        z="referenced",
        can_ready=None,
        power_ready=False,
    )

    stop = _assess_action(action("/motion/diagnostics/stop", safety="stop"), state, {})
    snapshot = _assess_action(action("/hardware/snapshot/collect", safety="service"), state, {})
    normal_move = _assess_action(action("/motion/axis/move"), state, {"axis": "x"})

    assert stop["enabled"] is True
    assert snapshot["enabled"] is True
    assert normal_move["enabled"] is False
    assert any(row["key"] == "power_ready" and row["met"] is False for row in normal_move["dependencies"])


def test_reconnect_does_not_require_existing_usb_ownership():
    state = machine_state()
    state["ownership"] = {"transport": "released", "usb": "released", "router": "stopped", "CAN_READY": None}

    reconnect = _assess_action(action("/reconnect", safety="service"), state, {})

    assert reconnect["enabled"] is True
    assert all(row["key"] != "transport_live" for row in reconnect["dependencies"])


def test_constructor_pipettes_stage_does_not_require_motion_enabled():
    state = machine_state(motion_enabled=False)
    constructor = _assess_action(action("/oem/startup/constructor_pipettes", safety="service"), state, {})
    assert constructor["enabled"] is True
    assert all(row["key"] != "motion_enabled" for row in constructor["dependencies"])


def test_move_requires_selected_axis_reference_but_home_establishes_it():
    state = machine_state(motion_enabled=True, x="unknown")
    move = _assess_action(action("/motion/axis/move"), state, {"axis": "x"})
    home = _assess_action(action("/motion/axis/home"), state, {"axis": "x"})
    assert move["enabled"] is False
    assert move["disabled_reason"] == "X axis is not homed."
    assert home["enabled"] is True


def test_pipette_liquid_motion_requires_xyz_references():
    state = machine_state(motion_enabled=True, x="referenced", y="unknown", z="referenced")
    assessed = _assess_action(action("/pipette/aspirate"), state, {"channel": 0})
    assert assessed["enabled"] is False
    assert assessed["disabled_reason"] == "Y axis is not homed."


def test_provider_gate_precedes_runtime_dependencies():
    assessed = _assess_action(action("/operator/meta/full", provider=False), machine_state(), {})
    assert assessed["enabled"] is False
    assert assessed["disabled_reason"] == "provider not bound"


def test_catalog_exposes_only_stable_robot_owned_z_semantic_actions():
    from bioxp import api

    actions, dispatch = _build_catalog(api.app)
    by_id = {row["action_id"]: row for row in actions}
    required = {
        "oem.z.manual_home": [],
        "oem.z.clear": [],
        "oem.z.move_z_home": ["wait_timeout_s"],
        "oem.z.set_clean_path": ["enabled"],
        "oem.z.move_steps": ["steps"],
        "oem.z.move_absolute": ["position_steps"],
        "oem.z.prepare": [],
        "oem.z.reconcile_switch_masks": ["confirm"],
        "oem.z.set_home": [],
        "oem.z.diagnostic_home_axis": [],
        "oem.z.stop": [],
        "oem.z.observe": [
            "command_id",
            "verdict",
            "physical_motion_observed",
            "expected_direction_observed",
            "home_endpoint_observed",
            "stopped_observed",
            "note",
        ],
    }
    for action_id, input_names in required.items():
        assert action_id in by_id
        assert [row["name"] for row in by_id[action_id]["inputs"]] == input_names
        assert by_id[action_id]["requires_confirmation"] is (
            action_id in {"oem.z.reconcile_switch_masks", "oem.z.set_home"}
        )
        assert by_id[action_id]["required_provider_capability"] == "initialize_motors"
        if action_id == "oem.z.move_z_home":
            assert dispatch[action_id]["fixed_inputs"]["rehome"] is True
        else:
            assert dispatch[action_id]["fixed_inputs"].get("axis") == "z"
    assert "z_pseudo_home" not in {row["name"] for row in by_id["oem.z.move_absolute"]["inputs"]}
    assert any(row["informational_path"] == "/motion/oem/z/move_z_home" for row in actions)
    assert not any(row["informational_path"] == "/motion/oem/z/live_right_reference" for row in actions)
    assert {
        "oem.z.set_max_speed",
        "oem.z.set_max_acc",
        "oem.z.set_vmax",
        "oem.z.set_current_max",
        "oem.z.restore_original_speed",
        "oem.z.scriptmove_to",
        "oem.z.move_gz",
        "oem.z.home_gz",
        "oem.z.lower_pipette",
        "oem.z.lift_pipette",
        "oem.z.self_test",
        "oem.z.abort",
        "oem.z.resume_after_abort",
    } <= set(by_id)
    assert by_id["oem.z.abort"]["inputs"] == []
    assert by_id["oem.z.diagnostic_home_axis"]["inputs"] == []
    assert by_id["oem.z.abort"]["requires_confirmation"] is False


def test_dashboard_normalizes_cache_only_axis_temperature_and_pipette_analytics():
    dashboard = _dashboard_payload(machine_state(motion_enabled=True, x="referenced", y="referenced", z="referenced"))
    assert dashboard["schema_version"] == "bioxp.operator_dashboard.v1"
    assert dashboard["motion"]["enabled"] is True
    assert dashboard["connection"]["live"] is True
    axis = dashboard["axes"][0]
    assert {
        key: axis[key]
        for key in (
            "axis", "reference", "position_steps", "speed_steps_s",
            "run_current", "standby_current", "left_switch_active",
            "right_switch_active", "motor_temperature_c",
            "motor_temperature_available",
        )
    } == {
        "axis": "x", "reference": "referenced", "position_steps": 123, "speed_steps_s": 0,
        "run_current": 31, "standby_current": 8, "left_switch_active": False,
        "right_switch_active": False, "motor_temperature_c": None,
        "motor_temperature_available": False,
    }
    assert axis["right_switch_raw_active"] is True
    assert {"left_switch_state", "right_switch_state", "left_switch_raw_active", "right_switch_raw_active", "left_switch_disabled", "right_switch_disabled", "coordinate_contract", "min_steps", "max_steps"} <= set(axis)
    assert dashboard["z_axis"]["authority"] == "Serial206OemInitializationProvider"
    assert dashboard["temperatures"] == [
        {"sensor": "tc_temp_c", "label": "Thermal cycler block", "unit": "°C", "temperature_c": 37.0, "available": True},
        {"sensor": "lid_temp_c", "label": "Thermal cycler lid", "unit": "°C", "temperature_c": 42.125, "available": True},
        {"sensor": "ped_temp_c", "label": "Thermal cycler pedestal", "unit": "°C", "temperature_c": None, "available": False},
        {"sensor": "rc_temp_c", "label": "Reagent chiller", "unit": "°C", "temperature_c": 12.0, "available": True},
    ]
    assert len(dashboard["pipettes"]["channels"]) == 4
    assert dashboard["pipettes"]["channels"][0]["tip_loaded"] is True
    assert dashboard["snapshot"]["collection_triggered"] is False


def test_production_axis_snapshot_reports_numeric_raw_and_effective_switch_state():
    from bioxp import api

    values = {1: 123, 3: 0, 6: 31, 7: 8, 9: 0, 10: 1, 12: 1, 13: 0}

    class Tester:
        MOTOR_SWITCH_ACTIVE_VALUE = 1

        @staticmethod
        def _motion_oem_axis_profile(axis):
            return {
                "board": 4,
                "motor": 1,
                "axis_min_steps": 0,
                "axis_max_steps": 160000,
                "coordinate_contract": "oem_source_nonnegative_z",
            }

        @staticmethod
        def query_only_tmcl(board, command, cmd_type, motor, value):
            return {"status": 100, "value": values[cmd_type]}

    row = api._query_axis_for_snapshot(Tester(), api.AxisName.Z)

    assert row["switch_activity"] == {
        "left_state": 0,
        "right_state": 1,
        "left_raw_active": False,
        "right_raw_active": True,
        "left_disabled": False,
        "right_disabled": True,
        "left_effective_active": False,
        "right_effective_active": False,
    }
    state = machine_state(motion_enabled=True, z="referenced")
    state["domains"]["axes"]["observation"]["rows"] = {"z": row}
    z = _dashboard_payload(state)["axes"][0]
    assert z["right_switch_state"] == 1
    assert z["right_switch_raw_active"] is True
    assert z["right_switch_active"] is False


def test_dashboard_reports_active_motion_independently_from_homing_and_uses_live_latch_evidence():
    state = machine_state(motion_enabled=True, x="desynced", y="referenced", z="desynced")
    state["references"]["rows"]["g"]["state"] = "desynced"
    state["references"]["rows"]["door"]["state"] = "desynced"
    state["lifecycle"]["door"] = {"door_closed": None, "latch_closed": None}

    dashboard = _dashboard_payload(state)

    assert dashboard["motion"] == {"enabled": True, "reason": None}
    assert dashboard["enclosure"] == {"door_closed": True, "latch_closed": True}


def test_dashboard_connection_live_does_not_fabricate_can_readiness():
    state = machine_state(can_ready=None)
    dashboard = _dashboard_payload(state)

    assert dashboard["connection"]["live"] is True
    assert dashboard["connection"]["ownership"]["CAN_READY"] is None
    assert dashboard["snapshot"]["collection_triggered"] is False
