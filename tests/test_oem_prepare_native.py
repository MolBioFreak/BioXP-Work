"""Pure native preparation/source capture acceptance; external bwrap required.

No device, camera, network or service is opened. Only snapshot capture callbacks
are doubled. Real immutable snapshot parser, logical model, provider snapshot
adapter, and CV kernels execute. Canonical parent-motion integration is a
separate parent-owned acceptance layer, not claimed by these tests.
"""
from __future__ import annotations
import copy
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from bioxp.oem_job_preparation import (
    build_prepare_handler, build_preparation_inspection_reader,
    capture_preparation_input, capture_preparation_settings,
    construct_new_machine_source_model, reset_loaded_job_tip_inventory,
)
from bioxp.oem_machine_bundle import load_oem_machine_snapshot
from bioxp.protocols.runtime_state import ProtocolSourceModel
from bioxp.oem_deck_movement import compile_finite_plate_operation, execute_finite_plate_operation


class NativeLeaves:
    """Finite compiler/executor exercised, physical children recorded only.

    Not a substitute for the parent's SQLite/resource-lineage acceptance.
    """
    def __init__(self, fail=None):
        self.children = []
        self.operations = []
        self.fail = fail

    def execute(self, operation, arguments, state):
        self.operations.append((operation, dict(arguments)))
        plan = compile_finite_plate_operation(operation, source_leaf_available=True, **arguments)
        def leaf(child):
            self.children.append(child)
            if child["operation"] == self.fail:
                raise IOError("physical-leaf failure")
            return {"ok": True, "source_return": True} if child["operation"] == "sourceConfirmGripper" else {"ok": True}
        return execute_finite_plate_operation(plan, leaf)

    def control(self, operation, arguments, state):
        self.operations.append((operation, dict(arguments)))
        return {"ok": True}


def test_actual_low_source_sequence_real_cv_and_no_tip_scan(snapshot):
    source = capture_preparation_settings(snapshot)
    model = construct_new_machine_source_model()
    # Labeled physical-image double: one dark 60x50 handle inside source ROI.
    handle = np.full((480, 640, 3), 200, np.uint8)
    handle[210:260, 300:360] = 0
    captures = []
    def frame(method, state):
        captures.append(method)
        return handle if method == "findStripHandle" else np.zeros((480, 640, 3), np.uint8)
    reader = build_preparation_inspection_reader(snapshot=snapshot, settings=source["settings"], source_model=model, capture_image=frame)
    captured = capture_preparation_input(settings=source["settings"], source_model=model,
        requirements={"JobName": None, "OutputPlateRequired": False, "TroughRequired": False}, source_identity=source["source_identity"])
    leaves = NativeLeaves()
    sleeps = []
    result = build_prepare_handler(captured=captured, execute_native=leaves.execute,
        execute_control=leaves.control, inspect_image=reader, sleep=sleeps.append)(SimpleNamespace(source_model=model))
    assert result["source_return"] == "OK" and result["inspection_issues"] == []
    assert captures == ["checkPurificationStation", "findStripHandle", "findStripHandle"]
    assert sleeps == [.7, 1.5, 1.5]
    names = [name for name, args in leaves.operations]
    assert names[0] == "confirm_gripper" and "home_gripper" not in names
    assert names.count("preparation_force_high_home") == 5
    assert names[-1] == "park_gantry"
    assert "pipette_tip_transition" not in names
    moves = [args for name, args in leaves.operations if name == "pipette_waste"]
    assert [(a["location"], a["offset_x"], a["offset_y"]) for a in moves] == [(0, 5868, -7744), (11, 277, -10942), (11, 277, 9931)]
    assert all(t.wells[0].empty for t in model.tip_trays[4:])  # inspection never resets hotel


@pytest.mark.parametrize("log_only", [False, True])
def test_actual_high_source_predicate_precedence_and_logonly(snapshot, log_only):
    import cv2
    source = capture_preparation_settings(snapshot, operation_parameters={**snapshot.operation_parameters,
        "ScreenResolutionHigh": True, "InspectionLogOnly": log_only})
    model = construct_new_machine_source_model()
    blank = np.zeros((480, 640, 3), np.uint8)
    def placed(name, x):
        image = cv2.imdecode(np.frombuffer(snapshot.records["appdata/" + name].raw_bytes, np.uint8), 1)
        h, w = image.shape[:2]
        canvas = np.zeros((max(480, h + 20), max(640, w + x + 20), 3), np.uint8)
        canvas[10:10 + h, x:x + w] = image
        return canvas
    # Each source call reads a fresh physical frame. Four all-zero output scores
    # exercise source's literal output==max branch WITHOUT an added >0.7 test.
    frames = [blank, blank, blank, blank, blank, placed("EmptyTrough.jpg", 0),
              placed("LowerHandle.jpg", 239), placed("UpperHandle.jpg", 236)]
    reader = build_preparation_inspection_reader(snapshot=snapshot, settings=source["settings"], source_model=model,
        capture_image=lambda method, state: frames.pop(0))
    captured = capture_preparation_input(settings=source["settings"], source_model=model,
        requirements={"JobName": None, "OutputPlateRequired": True, "TroughRequired": True}, source_identity=source["source_identity"])
    leaves = NativeLeaves()
    result = build_prepare_handler(captured=captured, execute_native=leaves.execute,
        execute_control=leaves.control, inspect_image=reader, sleep=lambda _: None)(SimpleNamespace(source_model=model))
    assert frames == []
    assert result["inspection_issues"] == ([] if log_only else ["trough"])
    assert result["source_return"] == "OK"
    assert [r for r in result["inspections"] if r["stage"] == "recovery"][0]["source_status"] == "OK"
    assert leaves.operations[-1][0] == "park_gantry"


@pytest.mark.parametrize("log_only", [False, True])
def test_strip_five_real_frames_source_error_and_logonly(snapshot, log_only):
    source = capture_preparation_settings(snapshot, operation_parameters={**snapshot.operation_parameters, "InspectionLogOnly": log_only})
    model = construct_new_machine_source_model()
    model.strips[0].strip_color = "A"
    handle = np.full((480, 640, 3), 200, np.uint8)
    handle[210:260, 300:360] = 0
    captures = []
    def frame(method, state):
        captures.append(method)
        return handle if method == "findStripHandle" else np.zeros((480, 640, 3), np.uint8)
    reader = build_preparation_inspection_reader(snapshot=snapshot, settings=source["settings"], source_model=model, capture_image=frame)
    captured = capture_preparation_input(settings=source["settings"], source_model=model,
        requirements={"JobName": None, "OutputPlateRequired": False, "TroughRequired": False}, source_identity=source["source_identity"])
    leaves = NativeLeaves()
    result = build_prepare_handler(captured=captured, execute_native=leaves.execute, execute_control=leaves.control,
        inspect_image=reader, sleep=lambda _: None)(SimpleNamespace(source_model=model))
    assert captures.count("inspectStrip") == 5
    assert result["inspection_issues"] == ([] if log_only else ["wells"])
    snapshots = [args["name"] for name, args in leaves.operations if name == "pipette_snapshot"]
    assert snapshots == ["strip_well_missing"] + (["strip_well_image_log"] if log_only else [])
    assert captured["source_model"]["strips"][0]["location"] == model.strips[0].location


def test_explicit_job_load_reset_is_distinct_from_constructor_and_inspection():
    model = construct_new_machine_source_model()
    model.logical_tip_present = None
    model.tip_trays[0].wells[0].content = "old_label"
    model.tip_trays[0].wells[0].zone_index = 7
    leaves = NativeLeaves()
    original = model.to_payload()
    rows = reset_loaded_job_tip_inventory(SimpleNamespace(source_model=model), execute_native=leaves.execute)
    assert len(rows) == 5
    assert [args["transition"] for name, args in leaves.operations] == ["reset"] * 5
    assert all(not w.empty and w.content is None and w.zone_index is None for t in model.tip_trays for w in t.wells)
    assert model.logical_tip_present is None
    assert original["tip_trays"][4]["wells"][0]["empty"] is True
    assert [t.tip_type for t in model.tip_trays] == [50, 50, 50, 200, 50]


def test_source_failure_propagates_without_motion_retry_or_finally_park(snapshot):
    source = capture_preparation_settings(snapshot)
    model = construct_new_machine_source_model()
    captured = capture_preparation_input(settings=source["settings"], source_model=model,
        requirements={"JobName": None, "OutputPlateRequired": False, "TroughRequired": False}, source_identity=source["source_identity"])
    leaves = NativeLeaves(fail="sourceMoveTo")
    with pytest.raises(OSError) as caught:
        build_prepare_handler(captured=captured, execute_native=leaves.execute,
            execute_control=leaves.control, inspect_image=forbidden, sleep=forbidden)(SimpleNamespace(source_model=model))
    assert [c["operation"] for c in leaves.children].count("sourceMoveTo") == 1
    assert "park_gantry" not in [name for name, args in leaves.operations]
    assert caught.value.preparation_evidence[-1]["operation"] == "pipette_waste"


def test_source_handles_capture_exception_is_visible_not_fabricated_cv(snapshot):
    source = capture_preparation_settings(snapshot)
    model = construct_new_machine_source_model()
    def frame(method, state):
        if method == "findStripHandle":
            raise IOError("physical camera disconnected")
        return np.zeros((480, 640, 3), np.uint8)
    reader = build_preparation_inspection_reader(snapshot=snapshot, settings=source["settings"], source_model=model, capture_image=frame)
    captured = capture_preparation_input(settings=source["settings"], source_model=model,
        requirements={"JobName": None, "OutputPlateRequired": False, "TroughRequired": False}, source_identity=source["source_identity"])
    leaves = NativeLeaves()
    result = build_prepare_handler(captured=captured, execute_native=leaves.execute,
        execute_control=leaves.control, inspect_image=reader, sleep=lambda _: None)(SimpleNamespace(source_model=model))
    assert result["source_return"] == "OK"
    assert any(e.get("source_catch") == "ControlLib.inspectStripHandle" for e in result["preparation_evidence"])
    assert not any(e.get("inspection_method") == "findStripHandle" for e in result["preparation_evidence"])
    assert leaves.operations[-1][0] == "park_gantry"


@pytest.fixture(scope="module")
def snapshot():
    return load_oem_machine_snapshot(Path(os.environ["OEM_NATIVE_BUNDLE_LOCK"]), require_operator_label=False)


def forbidden(*args, **kwargs):
    raise AssertionError("No native/camera leaf permitted in this test")


def test_actual_snapshot_settings_are_exact_and_constructor_defaults_are_labeled(snapshot):
    export = capture_preparation_settings(snapshot)
    settings = export["settings"]
    assert settings["CameraXOffset"] == 3499
    assert settings["CameraYOffset"] == -7744
    assert settings["CameraZOffset"] == 3145
    assert settings["ScreenResolutionHigh"] is False
    assert settings["DeckInspection"] is True
    assert settings["InspectionSettings"]["PurificationInspection"]["Parameters"] is None
    assert settings["InspectionSettings"]["StripInspection"]["Parameters"] == {"threLow": 100, "threHigh": 200, "equalize": False}
    assert export["source_identity"]["inspection_sha256"] == hashlib.sha256(snapshot.records["appdata/InspectionSettings.xml"].raw_bytes).hexdigest()
    out = Path(os.environ["OEM_NATIVE_RESULTS"])
    out.mkdir(exist_ok=True)
    (out / "captured-preparation-settings.json").write_text(json.dumps(export, indent=2))


def test_partial_current_host_policy_preserves_proven_seed_properties(snapshot):
    export = capture_preparation_settings(snapshot, operation_parameters={"DeckInspection": False})
    assert export["settings"]["DeckInspection"] is False
    assert export["settings"]["ScreenResolutionHigh"] is False
    assert export["settings"]["InspectionLogOnly"] is False
    assert export["settings"]["ThermalFault"] is False
    origins = export["source_identity"]["operation_parameter_origins"]
    assert origins["DeckInspection"] == "captured_current_host_policy"
    assert all(origins[name] == "immutable_operation_parameters" for name in ("ScreenResolutionHigh", "InspectionLogOnly", "ThermalFault"))


def test_source_constructor_defaults_when_operation_property_has_no_setter(snapshot):
    from dataclasses import replace
    # Deliberately model the typed pre-setter projection, not a new source file.
    initial = replace(snapshot, operation_parameters={})
    export = capture_preparation_settings(initial, operation_parameters={"DeckInspection": True})
    assert export["settings"]["ScreenResolutionHigh"] is False
    assert export["settings"]["InspectionLogOnly"] is True
    assert export["settings"]["ThermalFault"] is False
    assert all(export["source_identity"]["operation_parameter_origins"][name] == "ClassBioXPSettings_constructor1649-1661" for name in ("ScreenResolutionHigh", "InspectionLogOnly", "ThermalFault"))


def test_high_profile_selection_uses_captured_policy_not_hardcoded_serial(snapshot):
    policy = dict(snapshot.operation_parameters)
    policy["ScreenResolutionHigh"] = True
    export = capture_preparation_settings(snapshot, operation_parameters=policy)
    assert export["source_identity"]["inspection_profile"] == "Settings3250"
    assert export["settings"]["InspectionSettings"]["PurificationInspection"]["Parameters"]["threshold"] == .8
    assert snapshot.operation_parameters["ScreenResolutionHigh"] is False


def test_new_constructor_is_not_emptytip_or_historical_physical_default():
    model = construct_new_machine_source_model()
    assert [t.tip_type for t in model.tip_trays] == [50, 50, 50, 200, 50]
    assert [t.location for t in model.tip_trays] == [7, 8, 9, 10, 15]
    assert all(not w.empty for t in model.tip_trays[:4] for w in t.wells)
    assert all(w.empty for w in model.tip_trays[4].wells)
    assert all(t.tray_empty is False for t in model.tip_trays)
    assert model.logical_tip_present is model.carried_plate_present is model.allow_to_stop is None
    assert all(w.capacity == 200 and w.volume == 0 and w.zone_index is None and w.content is None for t in model.tip_trays for w in t.wells)
    assert len(model.strips) == 4 and all(len(t.wells) == 8 and t.strip_color is None for t in model.strips)
    assert ProtocolSourceModel.from_payload(model.to_payload()).to_payload() == model.to_payload()


def test_explicit_capture_keeps_null_job_name_and_detaches_model_settings(snapshot):
    source = capture_preparation_settings(snapshot)
    model = construct_new_machine_source_model()
    requirements = {"JobName": None, "OutputPlateRequired": False, "TroughRequired": False}
    captured = capture_preparation_input(settings=source["settings"], source_model=model,
                requirements=requirements, source_identity=source["source_identity"])
    original = copy.deepcopy(captured)
    model.tip_trays[0].wells[0].empty = True
    source["settings"]["DeckInspection"] = False
    requirements["JobName"] = "not-the-source-name"
    assert captured == original
    assert captured["requirements"]["JobName"] is None
    out = Path(os.environ["OEM_NATIVE_RESULTS"])
    out.mkdir(exist_ok=True)
    (out / "new-constructor-capture-not-historical-state.json").write_text(json.dumps(captured, indent=2))


@pytest.mark.parametrize("requirements", [{}, {"JobName": "diagnostic", "OutputPlateRequired": 0, "TroughRequired": False}])
def test_no_missing_job_requirement_defaults(requirements):
    with pytest.raises(ValueError):
        capture_preparation_input(settings={}, source_model=construct_new_machine_source_model(), requirements=requirements, source_identity={})


def test_source_no_inspection_branch_never_resets_or_moves(snapshot):
    source = capture_preparation_settings(snapshot)
    source["settings"]["DeckInspection"] = False
    model = ProtocolSourceModel()  # retained unknown must remain unknown
    captured = capture_preparation_input(settings=source["settings"], source_model=model,
        requirements={"JobName": None, "OutputPlateRequired": False, "TroughRequired": False}, source_identity=source["source_identity"])
    state = SimpleNamespace(source_model=model)
    prepare = build_prepare_handler(captured=captured, execute_native=forbidden,
        execute_control=forbidden, inspect_image=forbidden, sleep=forbidden)
    assert prepare(state)["source_noop"] == "DeckInspection=false"
    assert state.source_model.tip_trays == []
    assert captured["source_model"]["tip_trays"] == []


def test_changed_captured_input_rejected_before_any_motion(snapshot):
    source = capture_preparation_settings(snapshot)
    captured = capture_preparation_input(settings=source["settings"], source_model=construct_new_machine_source_model(),
        requirements={"JobName": None, "OutputPlateRequired": False, "TroughRequired": False}, source_identity=source["source_identity"])
    captured["source_settings"]["DeckInspection"] = False
    with pytest.raises(ValueError, match="digest mismatch"):
        build_prepare_handler(captured=captured, execute_native=forbidden, execute_control=forbidden, inspect_image=forbidden, sleep=forbidden)


def test_unknown_retained_model_not_retrofitted_by_load_prefix():
    state = SimpleNamespace(source_model=ProtocolSourceModel())
    with pytest.raises(ValueError, match="selected_five_tray_metadata"):
        reset_loaded_job_tip_inventory(state, execute_native=forbidden)
    assert state.source_model.tip_trays == []


def test_real_cv_reader_captures_fresh_each_call_and_uses_exact_bundle(snapshot):
    source = capture_preparation_settings(snapshot, operation_parameters={**snapshot.operation_parameters, "ScreenResolutionHigh": True})
    images = [np.zeros((480, 640, 3), dtype=np.uint8), np.full((480, 640, 3), 255, dtype=np.uint8)]
    calls = []
    def capture(method, state):
        calls.append(method)
        return images.pop(0)
    reader = build_preparation_inspection_reader(snapshot=snapshot, settings=source["settings"],
        source_model=construct_new_machine_source_model(), capture_image=capture)
    assert reader("checkPurificationStation", {}, None) is False
    assert reader("checkPurificationStation", {}, None) is True
    assert calls == ["checkPurificationStation", "checkPurificationStation"]
    template = snapshot.records["appdata/LowerHandle.jpg"].raw_bytes
    reader = build_preparation_inspection_reader(snapshot=snapshot, settings=source["settings"],
        source_model=construct_new_machine_source_model(), capture_image=lambda method, state: template)
    assert reader("matchPattern", {"template": "LowerHandle.jpg", "method": 5}, None)[0] > .99


def test_snapshot_provider_binds_shared_camera_and_retains_source_suppression():
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
    provider = object.__new__(Serial206OemInitializationProvider)
    provider.primitives = SimpleNamespace()
    calls = []
    def physical_camera(**kwargs):
        calls.append(kwargs)
        return {"snapshot_id": "physical-leaf-double"}
    provider.bind_oem_snapshot_image(physical_camera)
    result = provider.wp8_snapshot_image("SnapshotImage", {"name": "MS_plate"}, command_id="parent-child", child_order=0, plan_digest="f" * 64)
    assert result["delivery_attempted"] is True and result["result"]["snapshot_id"] == "physical-leaf-double"
    assert calls[0]["condition"] == "MS_plate" and "parent-child" in calls[0]["artifact_id"]
    def camera_failure(**kwargs):
        raise IOError("physical capture failed")
    provider.bind_oem_snapshot_image(camera_failure)
    result = provider.wp8_snapshot_image("SnapshotImage", {"name": "MS_plate"}, command_id="parent-child", child_order=1, plan_digest="f" * 64)
    assert result["ok"] and result["exception_suppressed"] and result["exception_type"] == "OSError"
