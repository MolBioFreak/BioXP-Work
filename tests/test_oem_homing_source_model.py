import json

from src.bioxp.oem_homing_model import (
    BOARD_PRIMITIVE_ANCHORS,
    ROUTE_MAPPINGS,
    RAW_FASTAPI_ROUTE_TABLE,
    LIVE_TARGET_MAPPINGS,
    operation_names,
    source_matrix,
    trace_for,
)


def test_initialize_motors_trace_is_exact_oem_order():
    assert operation_names("initializeMotors") == [
        "z.axisSearchHome",
        "g.setMaxCurrent.before_clear",
        "g.clear.moveSteps",
        "g.axisSearchHome",
        "x.axisSearchHome",
        "x.setHome",
        "x.setSpeed.restore",
        "x.park_6000",
        "y.axisSearchHome",
        "door.doorSearchHome",
        "y.setHome.final",
        "g.restore_current.version1",
    ]

    trace = trace_for("initializeMotors")
    assert trace[0].source.file.endswith("ClassControlInterface.cs")
    assert trace[0].source.lines == "3350-3353"
    assert trace[0].axis == "z"
    assert trace[0].params == {"speed": 1791}
    assert trace[7].params == {"position": 6000}


def test_manual_home_is_separate_from_startup_axis_search():
    assert operation_names("manual_home", axis="x") == ["manual.x.goHome"]
    manual_x = trace_for("manual_home", axis="x")[0]
    startup_x = [step for step in trace_for("initializeMotors") if step.axis == "x"]

    assert manual_x.operation == "goHome"
    assert manual_x.params == {"rehome": True, "speed": 500, "waitforstop": True}
    assert startup_x[0].operation == "axisSearchHome"
    assert startup_x[0].params == {"speed": 250}
    assert startup_x[1].operation == "setHome"
    assert startup_x[2].params == {"speed": 1700}
    assert startup_x[3].operation == "moveX"


def test_homeaxis_homexy_rehome_initialize_motion_are_not_collapsed():
    assert operation_names("HomeAxis") == [
        "HomeAxis.x.axisSearchHome",
        "HomeAxis.y.axisSearchHome",
        "HomeAxis.z.current_axisSearchHome",
        "HomeAxis.g.current_stall_axisSearchHome",
        "HomeAxis.door.preclear_doorSearchHome",
    ]
    assert operation_names("HomeXY") == [
        "HomeXY.set_xy_speedacc_200",
        "HomeXY.parallel_x_goHome",
        "HomeXY.parallel_y_goHome",
        "HomeXY.restore_xy_speedacc",
    ]
    assert operation_names("rehome") == [
        "rehome.save_door_state",
        "rehome.initializeMotors",
        "rehome.sleep_restore_door",
    ]
    assert operation_names("initializeMotion") == [
        "initializeMotion.flags",
        "initializeMotion.initializeMotors",
        "initializeMotion.tip_pipette_cleanup",
    ]


def test_board_primitives_capture_queryhome_switch_polarity_and_can_payloads():
    assert BOARD_PRIMITIVE_ANCHORS["queryHome.head"].lines == "389-401"
    assert "left switch status 0 means home true" in BOARD_PRIMITIVE_ANCHORS["queryHome.head"].note
    assert BOARD_PRIMITIVE_ANCHORS["queryLeftSwitchStatus.motor"].lines == "641-664"
    assert "{6,9,axis" in BOARD_PRIMITIVE_ANCHORS["queryLeftSwitchStatus.motor"].note
    assert BOARD_PRIMITIVE_ANCHORS["queryRightSwitchStatus.motor"].lines == "666-689"
    assert "{6,10,axis" in BOARD_PRIMITIVE_ANCHORS["queryRightSwitchStatus.motor"].note
    assert BOARD_PRIMITIVE_ANCHORS["setHome.motor"].lines == "492-517"


def test_source_matrix_is_json_serializable_and_no_motion_scoped():
    matrix = source_matrix()
    assert matrix["truth_level"] == "source_model_only_no_motion_no_usb"
    assert matrix["axis_to_board"]["x"] == {"board": "deck/CAN5", "motor": 0, "oem_designator": "MotorX"}
    assert matrix["axis_to_board"]["z"] == {"board": "head/CAN4", "motor": 1, "oem_designator": "MotorZ"}
    assert matrix["deviations_to_live_linux"] == [
        "Source-shaped software contracts are present; controller and physical acceptance remain pending.",
        "BMS /api/bioxp/* is a proxy/linkage layer and may not expose every raw FastAPI route.",
    ]
    json.dumps(matrix, sort_keys=True)



def test_live_target_mapping_uses_current_provider_owned_source_positive_contracts():
    by_mode = {m.source_mode: m for m in LIVE_TARGET_MAPPINGS}

    assert "initializeMotorsWithoutMotion" not in by_mode
    assert by_mode["initializeMotors/initializeMotion"].target_line == 3529
    assert by_mode["startup axisSearchHome"].target_line == 5576
    assert by_mode["startup axisSearchHome"].target_status == "source_shaped_provider_authority"
    assert all("GAP10/controller-zero" not in d for d in by_mode["startup axisSearchHome"].deviations)
    assert by_mode["manual button goHome(true)"].target_line == 5663
    assert by_mode["manual button goHome(true)"].target_status == "provider_owned_leaf_direct_route_retired"
    assert by_mode["doorSearchHome"].target_line == 6557
    assert by_mode["HomeAxis"].target_line == 7055
    assert by_mode["HomeAxis"].target_status == "provider_owned_leaf_direct_route_retired"
    assert by_mode["HomeXY"].target_line == 7146
    assert by_mode["HomeXY"].target_status == "direct_oem_parallel_task_run_waitall"
    assert any("concurrently" in d or "Task.Run" in d for d in by_mode["HomeXY"].deviations)
    assert by_mode["initializeMotors/initializeMotion"].target_status == "canonical_atomic_serial206_authority"
    assert "ControlLib.rehome" not in by_mode
    assert "ControlLib.initializeMotion" not in by_mode
