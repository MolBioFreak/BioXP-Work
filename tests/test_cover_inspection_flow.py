"""OEM inspectCover port: compile contracts, decision parity, relocation order.

Source: ControlLib.inspectCover:3663-3768 and the branch methods
(checkChillerCover:3891-3942, InspectOutputLocation:3778-3823,
checkRCCover:3825-3848, checkCoverStorage:3850-3888). These tests pin the
source child graph, the index-paired relocation semantics (crossed when both
storages are empty), the refusals that must never move, and the receipt shape
the acceptance evaluator requires.
"""
from __future__ import annotations

import importlib
import json
import sys
import types
from itertools import product

import cv2
import numpy as np
import pytest

from bioxp.oem_deck_movement import (
    FINITE_PLATE_OPERATIONS,
    WP8_COMPILED_CHILD_OPERATIONS,
    WP8_OPERATION_INTENT_KEYS,
    compile_finite_plate_operation,
)
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider, Serial206ProductionPrimitiveAdapter
from bioxp.oem_vision_acceptance import (
    OemVisionReceiptError,
    assemble_inspect_cover_receipt,
    evaluate_inspect_cover_receipt,
    plan_cover_relocations,
)


def _ops(plan):
    return [child["operation"] for child in plan["children"]]


def _jpeg(image: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return bytes(encoded.tobytes())


class TestBarcodeDecoder:
    def test_oem_gray_and_one_symbol_rule(self, monkeypatch):
        scan_barcode = importlib.import_module("bioxp.vision.oem_inspection").scan_barcode
        seen = []
        symbols = [types.SimpleNamespace(data=b"  ABC123  ")]
        scanner = types.ModuleType("pyzbar.pyzbar")
        setattr(scanner, "decode", lambda gray: (seen.append(gray.copy()), list(symbols))[1])
        monkeypatch.setitem(sys.modules, "pyzbar", types.ModuleType("pyzbar"))
        monkeypatch.setitem(sys.modules, "pyzbar.pyzbar", scanner)
        image = np.zeros((2, 2, 3), np.uint8)
        image[:] = (10, 60, 180)
        assert scan_barcode(_jpeg(image)) == "ABC123"
        assert np.array_equal(seen[0], cv2.cvtColor(cv2.imdecode(np.frombuffer(_jpeg(image), np.uint8), 1), 7))
        symbols.append(types.SimpleNamespace(data=b"SECOND"))
        assert scan_barcode(_jpeg(image)) == ""


class TestSourceThreadContext:
    def test_inspect_cover_uses_oem_motion_thread_mta_wait(self):
        adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
        adapter.tester = types.SimpleNamespace(
            motor_oem_axis_board_present=lambda axis: axis != "y",
        )
        adapter.x_move_absolute = lambda **kwargs: {"ok": True}
        result = adapter.move_xy(100, 200, wait_timeout_s=5, source_context="ControlLib.inspectCover")
        assert result["ok"] is True
        assert result["source_context_sealed"] is True
        assert result["wait_schedule"] == "MTA_WaitAll"
        with pytest.raises(ValueError, match="unsealed_moveXY_source_context"):
            adapter.move_xy(100, 200, wait_timeout_s=5, source_context="unknown")


class TestCameraPreflight:
    def test_initialize_illumination_before_deck_admission(self, monkeypatch):
        from bioxp import api
        from fastapi import HTTPException

        class Camera:
            initialized = False

            def initialize_illumination(self):
                self.initialized = True
                return {"ok": True}

        camera = Camera()
        calls = []
        monkeypatch.setattr(api, "_camera_provider", camera)
        monkeypatch.setattr(api, "_serial206_oem_initialization_provider", object())
        monkeypatch.setattr(api, "_bind_deck_cover_inspection", lambda _: None)
        monkeypatch.setattr(api, "_deck_inspection_settings", lambda: {
            "DeckInspection": True, "ScreenResolutionHigh": False, "InspectionLogOnly": False,
        })

        class Admitted(Exception):
            pass

        def admit(operation, **kwargs):
            assert camera.initialized and operation == "cover_inspection"
            calls.append(operation)
            raise Admitted

        monkeypatch.setattr(api.app.state, "oem_wp8_operation_admitter", admit, raising=False)
        action = types.SimpleNamespace(action_id="inspect-covers")
        state = types.SimpleNamespace(job_id="test-cover-preflight")
        with pytest.raises(Admitted):
            api._protocol_live_inspect_cover_handler(action, state)
        assert calls == ["cover_inspection"]
        camera.initialize_illumination = lambda: {"ok": False}
        with pytest.raises(HTTPException) as error:
            api._protocol_live_inspect_cover_handler(action, state)
        assert error.value.status_code == 503 and calls == ["cover_inspection"]


class TestRetainedTrayCleanPath:
    def test_oem_tip_available_is_independent_of_motor_epoch(self):
        provider = object.__new__(Serial206OemInitializationProvider)
        provider._tip_tray_state_reader = lambda tray_id: {
            "tip_available": True, "revision": 3,
            "operation_id": "reset:tray0", "command_id": "reset",
            "ownership_generation": 1, "board_epoch_4": 76, "board_epoch_5": 2,
        }
        provider.deck_owner_authority_stamps = lambda: {
            "ownership_generation": 1, "board_epoch_4": 107, "board_epoch_5": 1,
        }
        assert provider._derived_clean_path_from_tray_zero(expected_clean_path=False) is False
        with pytest.raises(ValueError, match="does not match"):
            provider._derived_clean_path_from_tray_zero(expected_clean_path=True)

    def test_missing_source_tray_still_refuses(self):
        provider = object.__new__(Serial206OemInitializationProvider)
        provider._tip_tray_state_reader = lambda tray_id: {"tip_available": True}
        with pytest.raises(RuntimeError, match="tray_0_tip_availability_unavailable"):
            provider._derived_clean_path_from_tray_zero(expected_clean_path=False)

    def test_gripper_version_comes_from_machine_binding_when_legacy_state_is_empty(self, monkeypatch):
        from contextlib import nullcontext
        module = importlib.import_module("bioxp.oem_serial206_initialization")
        provider = object.__new__(Serial206OemInitializationProvider)
        provider._lock = nullcontext()
        provider._load_state = lambda: {"machine_status": {"thermal_door_open": False}}
        provider._canonical_deck_semantic_state = lambda: {
            "current_location": "LOC_RC_COVER_STORAGE", "semantic_state_revision": 581,
            "ownership_generation": 1, "board_epoch_4": 108, "board_epoch_5": 1,
            "current_well": 0, "tip_loaded": False, "tip_dirty": False,
            "tip_location": -1, "clean_path": False, "pseudo_z_home": 500,
        }
        monkeypatch.setattr(module, "load_bound_oem_position_table",
                            lambda: types.SimpleNamespace(rows=lambda: [], digest="position-revision"))
        monkeypatch.setattr(module, "load_oem_parity_config",
                            lambda unused: types.SimpleNamespace(values={"GripperVersion": 1}))
        assert provider.mov_execution_machine_state()["gripper_version"] == 1
        monkeypatch.setattr(module, "load_oem_parity_config",
                            lambda unused: types.SimpleNamespace(values={"GripperVersion": None}))
        with pytest.raises(RuntimeError, match="source_authority_missing:GripperVersion"):
            provider.mov_execution_machine_state()


class TestCompileContract:
    def test_membership_and_intent_keys(self):
        assert "cover_inspection" in FINITE_PLATE_OPERATIONS
        assert WP8_OPERATION_INTENT_KEYS["cover_inspection"] == frozenset(
            {"deck_inspection", "screen_resolution_high", "inspection_log_only"}
        )
        assert {"inspectCoverAt", "coverInspectionRelocate"} <= WP8_COMPILED_CHILD_OPERATIONS

    def test_disabled_returns_after_force_high_home(self):
        plan = compile_finite_plate_operation(
            "cover_inspection", source_leaf_available=True, deck_inspection=False,
        )
        assert _ops(plan) == ["sourceForceToHighHome"]

    def test_enabled_child_graph_matches_source_order(self):
        plan = compile_finite_plate_operation(
            "cover_inspection", source_leaf_available=True,
            deck_inspection=True, screen_resolution_high=False, inspection_log_only=False,
        )
        assert _ops(plan) == [
            "sourceForceToHighHome", "doorOpen",
            "inspectCoverAt", "updatePlateLocation",
            "inspectCoverAt", "updatePlateLocation",
            "inspectCoverAt", "inspectCoverAt",
            "coverInspectionRelocate",
        ]
        children = plan["children"]
        assert children[1]["arguments"] == {"open": False}
        assert children[2]["arguments"] == {"destination": 17, "screen_resolution_high": False}
        assert children[3]["arguments"] == {"plate": 4, "location": 17}
        assert children[3]["source_condition"] == {
            "child_order": 2, "result_field": "cover_detected", "equals": True,
        }
        assert children[4]["arguments"]["destination"] == 19
        assert children[5]["arguments"] == {"plate": 5, "location": 19}
        assert children[5]["source_condition"]["child_order"] == 4
        assert children[6]["arguments"]["destination"] == 20
        assert children[7]["arguments"]["destination"] == 18
        assert children[8]["arguments"] == {"inspection_log_only": False}

    def test_invalid_inputs_refuse(self):
        with pytest.raises(RuntimeError):
            compile_finite_plate_operation("cover_inspection", source_leaf_available=True)
        with pytest.raises(RuntimeError):
            compile_finite_plate_operation(
                "cover_inspection", source_leaf_available=True, deck_inspection=True,
            )
        with pytest.raises(RuntimeError):
            compile_finite_plate_operation(
                "cover_inspection", source_leaf_available=True, deck_inspection=True,
                screen_resolution_high=False, inspection_log_only=1,
            )


class TestRelocationPlanner:
    def test_crossed_pairing_when_both_storages_empty(self):
        plan = plan_cover_relocations({17: True, 19: True, 20: False, 18: False})
        assert plan["cover_count"] == 2 and plan["error_status"] is None
        # OEM 3732-3744 pairs list[] with list2[] by index: with both storages
        # empty the source order crosses the covers (17->20, 19->18).
        assert plan["relocations"] == [
            {"cover": "output", "from": 17, "to": 20},
            {"cover": "reagent", "from": 19, "to": 18},
        ]

    def test_single_storage_pairs_by_index(self):
        assert plan_cover_relocations({17: True, 19: False, 20: True, 18: False})["relocations"] == [
            {"cover": "output", "from": 17, "to": 18},
        ]
        assert plan_cover_relocations({17: False, 19: True, 20: False, 18: True})["relocations"] == [
            {"cover": "reagent", "from": 19, "to": 20},
        ]

    def test_short_and_over_error_statuses(self):
        short = plan_cover_relocations({17: True, 19: False, 20: False, 18: False})
        assert short["error_status"] == "SHORT_CHILLER_COVER" and short["relocations"] == []
        over = plan_cover_relocations({17: True, 19: True, 20: True, 18: False})
        assert over["error_status"] == "OVER_CHILLER_COVER" and over["relocations"] == []


def _receipt(plan, *, log_only=False):
    success = plan["cover_count"] == 2 and plan["error_status"] is None
    observed = [17, 19, 20, 18]
    if plan["error_status"] == "OVER_CHILLER_COVER" and not log_only:
        # The evaluator mirrors the source early return: the receipt must end
        # at the location where the over-count was detected.
        count = 0
        for location in (17, 19, 20, 18):
            present = bool(plan["detected"][location])
            if location in (17, 19):
                if present:
                    count += 1
                continue
            if present and count >= 2:
                observed = [17, 19, 20, 18][: (17, 19, 20, 18).index(location) + 1]
                break
            if present:
                count += 1
    return {
        "force_to_high_home": {
            "provider_id": "test", "command_id": "c", "attempted": True,
            "controller_acknowledged": True, "postcondition_verified": True,
            "attempted_at_ms": 1, "acknowledged_at_ms": 2, "postcondition_verified_at_ms": 3,
        },
        "deck_inspection": True,
        "screen_resolution_high": False,
        "observed_locations": observed,
        "inspection_methods": {str(location): "checkChillerCover" for location in (17, 19, 20, 18)},
        "cover_detected": {str(location): bool(plan["detected"][location]) for location in (17, 19, 20, 18)},
        "relocations": list(plan["relocations"]),
        "final_cover_locations": {"output": 18, "reagent": 20} if success else None,
        "door_closed_verified": True,
        "door_open_verified": bool(success),
        "inspection_log_only": log_only,
    }


class TestEvaluatorAgreement:
    def test_planner_and_evaluator_agree_across_all_combinations(self):
        for bits in product((False, True), repeat=4):
            detected = dict(zip((17, 19, 20, 18), bits))
            try:
                plan = plan_cover_relocations(detected)
            except OemVisionReceiptError:
                continue
            receipt = _receipt(plan)
            observed = receipt["observed_locations"]
            receipt["inspection_methods"] = {str(l): "checkChillerCover" for l in observed}
            receipt["cover_detected"] = {str(l): bool(plan["detected"][l]) for l in observed}
            evaluation = evaluate_inspect_cover_receipt(receipt)
            assert evaluation["expected_relocations"] == plan["relocations"]
            success = plan["cover_count"] == 2 and plan["error_status"] is None
            assert evaluation["receipt_validation_pass"] is success


class _FakePrimitives:
    def __init__(self):
        self.calls = []

    def _axis_profile(self, axis):
        return {"axis_max_steps": 90263 if axis == "x" else 102956}

    def oem_move_z(self, position, *, pseudo_home_steps, motor_current=31, wait_for_stop=True):
        self.calls.append(("z", int(position)))
        return {"ok": True}

    def oem_move_to(self, x, y, z, *, source_context=None, **state):
        # Camera/decision tests only; connected movement coverage is in
        # test_cover_inspection_movement.py (real moveTo, XY and axis adapters).
        self.calls.append(("move_to", int(x), int(y), int(z)))
        self.source_context = source_context
        return {"ok": True}


class _FakeRow:
    def __init__(self, x, y):
        self.base_coordinates = {"x": x, "y": y}

    def oem_offset_move_coordinates(self, *, offset_x=0, offset_y=0, x_high_limit=None, y_high_limit=None):
        x = self.base_coordinates["x"] + offset_x
        y = self.base_coordinates["y"] + offset_y
        if x_high_limit is not None and x > x_high_limit:
            x = x_high_limit - 50
        if y_high_limit is not None and y > y_high_limit:
            y = y_high_limit - 50
        return {"x": x, "y": y, "z": 500}


class _FakeTable:
    def __init__(self, rows):
        self._rows = rows

    def resolve(self, *, location_id, well_id=None, plate_name=None):
        return _FakeRow(*self._rows[location_id])


_BASE = {
    "LOC_OC": (26213, 42413),
    "LOC_OC_COVER": (1324, 42129),
    "LOC_RC_COVER": (42788, 44972),
    "LOC_RC": (42788, 44972),
    "LOC_OC_COVER_STORAGE": (84252, 6057),
    "LOC_RC_COVER_STORAGE": (84252, 36267),
}

_SETTINGS = {
    "CameraXOffset": 3499, "CameraYOffset": -7744, "CameraZOffset": 3145,
    "InspectionSettings": {
        "CoverInspection": {"Exposure": 1000, "Gain": 1000, "LED1": False, "LED2": True, "LED3": False},
        "ScanBarCode": {"Exposure": 1000, "Gain": 1000, "LED1": False, "LED2": False, "LED3": False},
        "OutputPlateInspection": {"Exposure": 1000, "Gain": 1000, "LED1": False, "LED2": False, "LED3": False},
        "CoverStorageInspection": {"Exposure": 1000, "Gain": 1000, "LED1": True, "LED2": True, "LED3": True},
    },
    "VisionTemplates": {name: name.encode() for name in (
        "cover.jpg", "output.jpg", "outputw_foil.jpg", "output_empty.jpg",
        "reagentTray.jpg", "reagentEmpty.jpg", "EmptyStorage.jpg")},
}


def _make_provider(*, frames, scores=None, monkeypatch=None, barcode_reads=()):
    provider = object.__new__(Serial206OemInitializationProvider)
    primitives = _FakePrimitives()
    calls = {"led": [], "rgb": [], "save": [], "capture": 0, "update": [], "barcode": []}
    frame_iter = list(frames)
    barcode_iter = iter(barcode_reads)

    def barcode(frame):
        calls["barcode"].append(frame)
        return next(barcode_iter, "")

    def capture(*, condition, artifact_id):
        calls["capture"] += 1
        return {"frame": frame_iter.pop(0) if frame_iter else frames[-1], "capture_evidence": {}}

    def save(*, frame, condition, artifact_id):
        calls["save"].append(condition)
        return {"ok": True}

    provider.primitives = primitives
    provider.bind_oem_cover_inspection_callbacks(
        settings=lambda: _SETTINGS,
        capture=capture,
        save=save,
        led=lambda *, channel, on: calls["led"].append((channel, on)),
        rgb=lambda r, g, b: calls["rgb"].append((r, g, b)),
        barcode=barcode,
    )
    provider.mov_execution_machine_state = lambda: {"pseudo_z_home": 500, "tip_loaded": False}
    provider._deck_gripper_confirmed = lambda: True
    provider.wp8_update_location = lambda operation, arguments, **kwargs: calls["update"].append(
        (operation, dict(arguments))
    )
    provider._test_calls = calls
    return provider


@pytest.fixture(autouse=True)
def _fake_table(monkeypatch):
    import bioxp.oem_serial206_initialization as mod

    monkeypatch.setattr(mod, "load_bound_oem_position_table", lambda: _FakeTable(_BASE))


_GRAY_WITH_COVER = np.full((480, 640), 70, dtype=np.uint8)   # locate_cover TRUE (lowest=70>35)
_GRAY_EMPTY = np.zeros((480, 640), dtype=np.uint8)           # locate_cover FALSE


class TestInspectCoverAt:
    def test_low_path_detected_records_findings_and_evidence(self):
        provider = _make_provider(frames=[_jpeg(_GRAY_WITH_COVER)])
        result = provider.wp8_inspect_cover_at(
            "inspectCoverAt", {"destination": 17, "screen_resolution_high": False},
            command_id="cmd-1", child_order=2, plan_digest="d",
        )
        assert result["cover_detected"] is True
        assert result["method"] == "checkChillerCover"
        calls = provider._test_calls
        assert calls["rgb"] == [(255, 255, 255)]
        assert (2, True) in calls["led"] and (1, False) in calls["led"]
        assert calls["led"][-3:] == [(1, False), (2, False), (3, False)]
        assert ("move_to", 1324 + 20021, 42129, 500) in provider.primitives.calls
        assert provider.primitives.source_context == "ControlLib.inspectCover"
        assert not any(call[0] == "z" for call in provider.primitives.calls)
        assert calls["update"] == [("updateLocation", {"destination": 17, "well": 0})]
        assert calls["save"] == ["check_chiller_cover_LOC_OC_COVERfound"]
        assert provider._oem_cover_inspection_findings["cmd-1"] == {17: True}

    @pytest.mark.parametrize("location,offset", [(19, 20021), (20, 5923), (18, 5923)])
    def test_low_path_uses_oem_station_vs_storage_offset(self, location, offset):
        provider = _make_provider(frames=[_jpeg(_GRAY_EMPTY)])
        result = provider.wp8_inspect_cover_at(
            "inspectCoverAt", {"destination": location, "screen_resolution_high": False},
            command_id="cmd-offset", child_order=2, plan_digest="d",
        )
        name = {19: "LOC_RC_COVER", 20: "LOC_RC_COVER_STORAGE", 18: "LOC_OC_COVER_STORAGE"}[location]
        x, y = _BASE[name]
        assert ("move_to", x + offset, y, 500) in provider.primitives.calls
        assert result["cover_detected"] is False

    def test_low_path_missing_names_snapshot_missing(self):
        provider = _make_provider(frames=[_jpeg(_GRAY_EMPTY)])
        result = provider.wp8_inspect_cover_at(
            "inspectCoverAt", {"destination": 19, "screen_resolution_high": False},
            command_id="cmd-2", child_order=2, plan_digest="d",
        )
        assert result["cover_detected"] is False
        assert provider._test_calls["save"] == ["check_chiller_cover_LOC_RC_COVERmissing"]

    def test_low_reagent_barcode_flips_positive_after_oem_camera_move(self):
        provider = _make_provider(
            frames=[_jpeg(_GRAY_WITH_COVER), _jpeg(_GRAY_EMPTY)], barcode_reads=("REAGENT123",),
        )
        result = provider.wp8_inspect_cover_at(
            "inspectCoverAt", {"destination": 19, "screen_resolution_high": False},
            command_id="cmd-barcode", child_order=4, plan_digest="d",
        )
        assert result["cover_detected"] is False
        assert result["details"]["barcode_flip_applied"] is True
        assert provider._test_calls["capture"] == 2
        assert ("move_to", 42788 - 23930 + 3499, 44972 + 7582 - 7744, 500) in provider.primitives.calls
        assert ("z", 3145) in provider.primitives.calls
        assert provider._test_calls["update"][-1] == ("updateLocation", {"destination": 3, "well": 0})

    def test_low_reagent_empty_barcode_retries_second_oem_pose(self):
        provider = _make_provider(
            frames=[_jpeg(_GRAY_WITH_COVER), _jpeg(_GRAY_EMPTY)], barcode_reads=("", ""),
        )
        result = provider.wp8_inspect_cover_at(
            "inspectCoverAt", {"destination": 19, "screen_resolution_high": False},
            command_id="cmd-empty-barcode", child_order=4, plan_digest="d",
        )
        assert result["cover_detected"] is True
        assert provider._test_calls["capture"] == 3
        assert ("move_to", 42788 - 23930 + 3499 + 2000, 44972 + 7582 - 7744 - 4000, 500) in provider.primitives.calls
        assert len(result["details"]["reagent_barcode_attempts"]) == 2

    def test_high_output_location_selection(self, monkeypatch):
        import bioxp.vision.oem_inspection as cvmod

        for cover_score, expected in ((0.9, True), (0.2, False)):
            scores = {"cover.jpg": cover_score, "output.jpg": 0.1, "outputw_foil.jpg": 0.1, "output_empty.jpg": 0.1}

            class _Score:
                def __init__(self, maximum):
                    self.maximum = maximum

            monkeypatch.setattr(cvmod, "match_pattern", lambda image, template, method=None: _Score(scores[template.decode() if isinstance(template, bytes) else template]))
            provider = _make_provider(frames=[_jpeg(_GRAY_WITH_COVER), _jpeg(_GRAY_WITH_COVER)])
            result = provider.wp8_inspect_cover_at(
                "inspectCoverAt", {"destination": 17, "screen_resolution_high": True},
                command_id="cmd-3", child_order=6, plan_digest="d",
            )
            assert result["method"] == "InspectOutputLocation(1)"
            assert result["cover_detected"] is expected
            if not expected:
                assert provider._test_calls["save"] == ["check_chiller_cover_LOC_OC_COVER missing"]
            assert ("z", 15000 + 3145) in provider.primitives.calls


class _RelocateProvider:
    def __init__(self):
        self.nested = []
        self.finalize = []
        self._oem_cover_inspection_findings = {}

    def _wp8_compile_and_execute(self, *, operation, inputs, command_id, owner_identity):
        self.nested.append((operation, dict(inputs)))
        return {"ok": True, "operation": operation}

    def _wp8_execute_nested_plan(self, *, plan, command_id, owner_identity):
        self.finalize.append(plan)
        return {"ok": True}

    def wp8_read_door_sensors(self, operation, arguments, **kwargs):
        return {"door_open": True, "door_closed": False}


class TestRelocate:
    def _run(self, findings, *, log_only=False):
        provider = _RelocateProvider()
        provider._oem_cover_inspection_findings = {"cmd": dict(findings)}
        result = Serial206OemInitializationProvider.wp8_cover_inspection_relocate(
            provider, "coverInspectionRelocate", {"inspection_log_only": log_only},
            command_id="cmd", child_order=8, plan_digest="d", owner_identity={"work_identity": "w"},
        )
        return provider, result

    def test_canonical_moves_in_source_order(self):
        provider, result = self._run({17: True, 19: True, 20: False, 18: False})
        assert provider.nested == [
            ("catch_plate", {"plate": 4, "run_in_parallel": True}),
            ("release_plate", {"destination": 20, "press_plate": False, "run_in_parallel": True}),
            ("catch_plate", {"plate": 5, "run_in_parallel": True}),
            ("release_plate", {"destination": 18, "press_plate": False, "run_in_parallel": True}),
            ("thermal_door", {"open": True}),
        ]
        finalize_children = provider.finalize[0]["children"]
        assert [c["arguments"] for c in finalize_children] == [
            {"locations": [{"plate": 4, "location": 18}, {"plate": 5, "location": 20}]},
        ]
        assert result["relocations"] == [
            {"cover": "output", "from": 17, "to": 20},
            {"cover": "reagent", "from": 19, "to": 18},
        ]
        assert result["final_cover_locations"] == {"output": 18, "reagent": 20}
        assert result["door_open_verified"] is True
        assert provider._oem_cover_inspection_findings == {}

    def test_failed_path_moves_nothing(self):
        provider, result = self._run({17: True, 19: False, 20: False, 18: False})
        assert provider.nested == [] and provider.finalize == []
        assert result["cover_count"] == 1
        assert result["error_status"] == "SHORT_CHILLER_COVER"
        assert result["relocations"] == [] and result["final_cover_locations"] is None
        assert result["door_open_verified"] is False

    def test_missing_findings_refuse(self):
        provider = _RelocateProvider()
        provider._oem_cover_inspection_findings = {}
        with pytest.raises(RuntimeError):
            Serial206OemInitializationProvider.wp8_cover_inspection_relocate(
                provider, "coverInspectionRelocate", {"inspection_log_only": False},
                command_id="cmd", child_order=8, plan_digest="d", owner_identity={},
            )


class TestReceiptAssembly:
    def _evidence(self, detected, *, relocations, final, door_open, door_closed=True):
        children = [
            {"child_order": 0, "operation": "sourceForceToHighHome", "terminal_state": "completed",
             "terminal_evidence_json": json.dumps({"result": {"ok": True}})},
            {"child_order": 1, "operation": "doorOpen", "terminal_state": "completed",
             "terminal_evidence_json": json.dumps({"completed_children": [
                 {"order": 4, "operation": "readDoorSensors",
                  "result": {"door_open": not door_closed, "door_closed": door_closed}}]})},
        ]
        methods = {str(location): "checkChillerCover" for location in (17, 19, 20, 18)}
        for index, location in enumerate((17, 19, 20, 18)):
            children.append({
                "child_order": 2 + index, "operation": "inspectCoverAt", "terminal_state": "completed",
                "terminal_evidence_json": json.dumps({"result": {
                    "cover_detected": bool(detected[location]), "location": location,
                    "method": methods[str(location)], "ok": True}}),
            })
        children.append({
            "child_order": 8, "operation": "coverInspectionRelocate", "terminal_state": "completed",
            "terminal_evidence_json": json.dumps({"result": {
                "relocations": relocations, "final_cover_locations": final,
                "door_open_verified": door_open, "ok": True}}),
        })
        return {
            "operation": {"command_id": "cmd-1"},
            "children": children,
            "state_transitions": [
                {"child_order": 0, "created_at": 1000.0},
                {"child_order": 8, "created_at": 1005.0},
            ],
        }

    def test_canonical_receipt_passes_evaluator(self):
        evidence = self._evidence(
            {17: True, 19: True, 20: False, 18: False},
            relocations=[{"cover": "output", "from": 17, "to": 20},
                         {"cover": "reagent", "from": 19, "to": 18}],
            final={"output": 18, "reagent": 20}, door_open=True,
        )
        receipt = assemble_inspect_cover_receipt(
            evidence, settings={"deck_inspection": True, "screen_resolution_high": False,
                                "inspection_log_only": False},
        )
        evaluation = evaluate_inspect_cover_receipt(receipt)
        assert evaluation["receipt_validation_pass"] is True
        assert evaluation["outcome"] == "covers_canonicalized"
        assert receipt["cover_detected"] == {"17": True, "19": True, "20": False, "18": False}
        assert receipt["door_closed_verified"] is True and receipt["door_open_verified"] is True

    def test_short_receipt_records_failure_without_claims(self):
        evidence = self._evidence(
            {17: True, 19: False, 20: False, 18: False},
            relocations=[], final=None, door_open=False,
        )
        receipt = assemble_inspect_cover_receipt(
            evidence, settings={"deck_inspection": True, "screen_resolution_high": False,
                                "inspection_log_only": False},
        )
        evaluation = evaluate_inspect_cover_receipt(receipt)
        assert evaluation["receipt_validation_pass"] is False
        assert evaluation["error_status"] == "SHORT_CHILLER_COVER"
