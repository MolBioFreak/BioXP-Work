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


def test_route_mapping_marks_raw_fastapi_bms_proxy_and_linux_deviations_separately():
    by_route = {(m.surface, m.route): m for m in ROUTE_MAPPINGS}

    raw_startup = by_route[("raw-fastapi", "/motion/oem/startup_step")]
    raw_home = by_route[("raw-fastapi", "/motion/axis/home")]
    bms_startup = by_route[("bms-proxy", "/api/bioxp/motion/oem/startup_step")]

    assert raw_startup.maps_to == "stepwise initializeMotors subset"
    assert raw_home.maps_to == "manual button goHome-style route"
    assert raw_startup.equivalence != raw_home.equivalence
    assert bms_startup.equivalence == "proxy-not-authority"
    assert any("inside robot/container network" in note for note in raw_startup.notes)
    assert any("subset" in note or "status shape" in note for note in bms_startup.notes)


def test_source_matrix_is_json_serializable_and_no_motion_scoped():
    matrix = source_matrix()
    assert matrix["truth_level"] == "source_model_only_no_motion_no_usb"
    assert matrix["axis_to_board"]["x"] == {"board": "deck/CAN5", "motor": 0, "oem_designator": "MotorX"}
    assert matrix["axis_to_board"]["z"] == {"board": "head/CAN4", "motor": 1, "oem_designator": "MotorZ"}
    assert "Current live Linux homing is a guarded reconstruction" in matrix["deviations_to_live_linux"][0]
    json.dumps(matrix, sort_keys=True)



def test_live_target_mapping_labels_missing_or_partial_ports_explicitly():
    by_mode = {m.source_mode: m for m in LIVE_TARGET_MAPPINGS}

    assert by_mode["initializeMotorsWithoutMotion"].target_symbol == "BioXpTester.motor_oem_initialize_without_motion"
    assert by_mode["startup axisSearchHome"].target_status == "partial_guarded_reconstruction"
    assert any("GAP10/controller-zero" in d for d in by_mode["startup axisSearchHome"].deviations)
    assert by_mode["manual button goHome(true)"].target_status == "unsafe_until_predicate_matrix_fixed"
    assert by_mode["HomeXY"].target_status == "direct_oem_parallel_task_run_waitall"
    assert any("concurrently" in d or "Task.Run" in d for d in by_mode["HomeXY"].deviations)
    assert by_mode["ControlLib.rehome"].target_status == "direct_wrapper_with_door_restore_gap"
    assert by_mode["ControlLib.initializeMotion"].target_status == "direct_wrapper_no_homing_diagnostic_by_default"


def test_raw_fastapi_route_table_is_enumerated_as_raw_not_bms():
    by_path = {row["path"]: row for row in RAW_FASTAPI_ROUTE_TABLE}
    assert by_path["/motion/oem/startup_step"]["classification"] == "stepwise initializeMotors subset"
    assert by_path["/motion/oem/home_xy"]["classification"].startswith("direct HomeXY mode surface")
    assert by_path["/motion/oem/rehome"]["classification"].startswith("direct ControlLib.rehome wrapper")
    assert by_path["/motion/oem/initialize_motion"]["classification"].startswith("direct ControlLib.initializeMotion wrapper")
    assert by_path["/motion/axis/home"]["classification"] == "manual/goHome-style route"
    assert by_path["/motion/axis/zero"]["classification"].startswith("Linux absolute controller-zero")
    assert "/api/bioxp/motion/oem/startup_step" not in by_path
