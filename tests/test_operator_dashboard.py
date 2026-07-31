from __future__ import annotations

from bioxp.operator_controls import _assess_action, _build_catalog, _dashboard_payload, _motor_motion_action, _safety


def machine_state(*, motion_enabled=False, x="unknown", y="unknown", z="unknown"):
    return {
        "ownership_generation": 12,
        "ownership": {"transport": "owned", "usb": "service", "router": "running", "CAN_READY": True},
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
            "power": {"status": "observed", "observation": {"safety_valid": True}},
            "axes": {"status": "observed", "observation": {"rows": {
                "x": {"status": {"position": {"value": 123}, "speed": {"value": 0}, "max_current": {"value": 31}, "standby_current": {"value": 8}}, "switch_activity": {"left_raw_active": False, "right_raw_active": True}},
            }}},
            "thermal": {"status": "observed", "observation": {"temps": {"tc_temp_c": {"value": 37000, "ok": True}, "lid_temp_c": {"temp_c": 42.125, "ok": True}, "ped_temp_c": {"value": True, "ok": True}}}},
            "chiller": {"status": "observed", "observation": {"temps": {"rc_temp_c": {"value": 12000, "ok": True}}}},
            "pipette": {"status": "observed", "observation": {"ok": True, "channels": [{"channel": index, "available": True, "initialized": index < 2, "tip_loaded": index == 0} for index in range(4)]}},
            "latch": {"status": "observed", "observation": {"door_sensor": 1, "latch_sensor": 1}},
        },
        "freshness": {"state": "fresh", "age_s": 1.25, "fresh_for_s": 30.0},
        "snapshot_id": "snapshot-1",
    }


def action(path: str, *, method="POST", safety="motion", provider=True):
    return {
        "action_id": "route.test",
        "kind": "primitive",
        "informational_method": method,
        "informational_path": path,
        "safety_class": safety,
        "provider_available": provider,
        "provider_unavailable_reason": None if provider else "provider not bound",
    }


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


def test_indirect_live_motion_routes_are_classified_and_gated_as_motion():
    state = machine_state(motion_enabled=False, x="referenced", y="referenced", z="referenced")
    for path in (
        "/latch/lock",
        "/latch/unlock",
        "/protocol/execute",
        "/oem/runtime/commands/enqueue",
        "/oem/runtime/commands/initializeSystem",
        "/oem/runtime/commands/PrepareToRunJob",
        "/oem/runtime/events/resume",
        "/oem/startup/initialize_environment",
        "/oem/initial_check",
    ):
        row = action(path, safety=_safety("POST", path))
        assert row["safety_class"] == "motion", path
        assert _motor_motion_action(row) is True, path
        assessed = _assess_action(row, state, {})
        assert assessed["enabled"] is False, path
        assert assessed["disabled_reason"] == "Motion is inactive. Activate motion before moving this motor.", path


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
        if row["kind"] == "primitive" and row["safety_class"] == "motion"
    ]
    assert len(motion_actions) >= 52

    for missing_value in (None, "missing"):
        state = machine_state(motion_enabled=True, x="referenced", y="referenced", z="referenced")
        if missing_value == "missing":
            del state["maintenance"]["recovery_required"]
        else:
            state["maintenance"]["recovery_required"] = None
        admitted = []
        wrong_reason = []
        for row in motion_actions:
            assessed = _assess_action(row, state, {})
            if assessed["enabled"]:
                admitted.append(row["informational_path"])
            elif assessed["disabled_reason"] != "Motion is inactive. Activate motion before moving this motor.":
                wrong_reason.append((row["informational_path"], assessed["disabled_reason"]))
        assert admitted == [], (missing_value, admitted)
        assert wrong_reason == [], (missing_value, wrong_reason)
        dashboard = _dashboard_payload(state)
        assert dashboard["motion"]["enabled"] is False, missing_value
        assert dashboard["motion"]["reason"] == "Motion is inactive.", missing_value


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


def test_dashboard_normalizes_cache_only_axis_temperature_and_pipette_analytics():
    dashboard = _dashboard_payload(machine_state(motion_enabled=True, x="referenced", y="referenced", z="referenced"))
    assert dashboard["schema_version"] == "bioxp.operator_dashboard.v1"
    assert dashboard["motion"]["enabled"] is True
    assert dashboard["connection"]["live"] is True
    assert dashboard["axes"][0] == {
        "axis": "x", "reference": "referenced", "position_steps": 123, "speed_steps_s": 0,
        "run_current": 31, "standby_current": 8, "left_switch_active": False,
        "right_switch_active": True, "motor_temperature_c": None,
        "motor_temperature_available": False,
    }
    assert dashboard["temperatures"] == [
        {"sensor": "tc_temp_c", "label": "Thermal cycler block", "unit": "°C", "temperature_c": 37.0, "available": True},
        {"sensor": "lid_temp_c", "label": "Thermal cycler lid", "unit": "°C", "temperature_c": 42.125, "available": True},
        {"sensor": "ped_temp_c", "label": "Thermal cycler pedestal", "unit": "°C", "temperature_c": None, "available": False},
        {"sensor": "rc_temp_c", "label": "Reagent chiller", "unit": "°C", "temperature_c": 12.0, "available": True},
    ]
    assert len(dashboard["pipettes"]["channels"]) == 4
    assert dashboard["pipettes"]["channels"][0]["tip_loaded"] is True
    assert dashboard["snapshot"]["collection_triggered"] is False
