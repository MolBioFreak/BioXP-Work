from __future__ import annotations

import sqlite3

import pytest
from pydantic import ValidationError

from bioxp import oem_machine_bundle as bundle
from bioxp.oem_calibration_settings import (
    CalibrationSettingsPatch, CalibrationSettingsService, load_saved_calibration,
)
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.oem_config import _snapshot_legacy_bundle
from bioxp.oem_runtime_store import OEMRuntimeStore
from oem_machine_bundle_test_support import bind_serial206_oem_snapshot


@pytest.fixture
def baseline(monkeypatch):
    return bind_serial206_oem_snapshot(monkeypatch)


@pytest.fixture
def store(tmp_path):
    result = OEMRuntimeStore(tmp_path)
    yield result
    result.close()


def patch(**values):
    return CalibrationSettingsPatch.model_validate({"positions": [{"name": "TECANRACK1", **values}]})


def test_absent_overlay_is_identity_and_no_write(baseline, store):
    before = store._db.total_changes
    assert load_saved_calibration(baseline, store) is baseline
    result = CalibrationSettingsService(store, baseline).read()
    assert result["saved_revision"] is None
    assert not result["pending_restart"]
    assert result["active_positions"] == result["saved_positions"]
    assert store._db.total_changes == before
    assert "active_calibration_revision" not in baseline.config_status_projection()


def test_save_reload_consumes_calibration_and_preserves_baseline(baseline, store, monkeypatch):
    original = next(row for row in baseline.position_table if row["name"] == "TECANRACK1")
    config_record = baseline.records["appdata/config.xml"]
    original_bytes = (baseline.bundle_root / config_record.bundle_relative_path).read_bytes()
    service = CalibrationSettingsService(store, baseline)
    result = service.save(patch(x=12345, y=23456, zLow=34000, zDelta=1000, inc_factor=2))
    assert not result["pending_restart"] and result["active_revision_id"] == result["saved_revision_id"]
    assert not result["motion_commanded"]
    assert next(row for row in result["active_positions"] if row["name"] == "TECANRACK1")["x"] == 12345
    assert bundle.get_active_oem_machine_snapshot() is baseline
    effective = load_saved_calibration(baseline, store)
    assert effective.records is baseline.records
    assert effective.fields is baseline.fields
    assert effective.axis_limits is baseline.axis_limits
    assert effective.operation_parameters is baseline.operation_parameters
    assert (baseline.bundle_root / config_record.bundle_relative_path).read_bytes() == original_bytes
    row = next(row for row in effective.position_table if row["name"] == "TECANRACK1")
    assert row["zHigh"] == 33000
    assert row["raw_attributes"] == original["raw_attributes"]
    assert row["source"].startswith("user_authored_calibration:")
    # True shared consumer path, not a bespoke settings-only projection.
    table = load_bound_oem_position_table(effective)
    target = table.resolve(location_id="TECANRACK1")
    # OEM loader always substitutes TECAN zDelta=53000 and clamps zHigh<5000.
    assert target.oem_move_to_coordinates(column=1, row=1) == {"x": 8081, "y": 27720, "z": 0}
    assert target.z_low == 34000
    assert target.z_delta == 53000
    projected = next(row for row in result["saved_motion_positions"] if row["location_id"] == "TECANRACK1")
    assert projected["z_high"] == 0
    assert result["saved_loader_adjustments"]
    assert "user_authored_calibration:" in target.source_anchor
    assert _snapshot_legacy_bundle(effective)["source_type"] == "sealed_baseline_plus_user_calibration"
    assert not CalibrationSettingsService(store, effective).read()["pending_restart"]
    with pytest.raises(bundle.OemMachineBundleError):
        bundle.set_active_oem_machine_snapshot(effective)
    monkeypatch.setattr(bundle, "_active_snapshot", None)
    monkeypatch.setattr(bundle, "load_oem_machine_snapshot", lambda *args, **kwargs: baseline)
    monkeypatch.setenv(bundle.OEM_MACHINE_BUNDLE_LOCK_ENV, "/not-read-test-lock")
    bound = bundle.configure_oem_machine_snapshot_from_env(require_operator_label=False, runtime_store=store)
    assert bound == effective
    assert load_bound_oem_position_table().resolve(location_id="TECANRACK1").base_coordinates["x"] == 12345


def test_plate_well_and_camera_offsets_share_startup_projection(baseline, store):
    service = CalibrationSettingsService(store, baseline)
    service.save(CalibrationSettingsPatch.model_validate({"positions": [
        {"name": "LOC_RC", "x": 30000, "y": 10000, "zLow": 80000, "zDelta": 20000, "inc_factor": 1},
        {"name": "CAMERA_OFFSET", "x": -1000, "y": 2000, "zLow": 45000},
    ]}))
    effective = load_saved_calibration(baseline, store)
    table = load_bound_oem_position_table(effective)
    target = table.resolve(location_id="LOC_RC")
    assert target.oem_move_to_coordinates(column=2, row=3, high_pos=False) == {"x": 25736, "y": 16396, "z": 80000}
    assert target.oem_move_to_coordinates(column=2, row=3, high_pos=True)["z"] == 60000
    camera = next(row for row in effective.position_table if row["name"] == "CAMERA_OFFSET")
    assert (camera["x"], camera["y"], camera["zLow"]) == (-1000, 2000, 45000)
    with pytest.raises(TypeError):
        camera["x"] = 0


def test_partial_saves_merge_and_persist_across_store_reopen(baseline, tmp_path):
    store = OEMRuntimeStore(tmp_path)
    service = CalibrationSettingsService(store, baseline)
    first = service.save(patch(x=111))
    second = service.save(patch(y=222))
    assert first["saved_revision_id"] != second["saved_revision_id"]
    store.close()
    store = OEMRuntimeStore(tmp_path)
    try:
        row = next(row for row in load_saved_calibration(baseline, store).position_table if row["name"] == "TECANRACK1")
        assert (row["x"], row["y"]) == (111, 222)
    finally:
        store.close()


def test_failed_save_does_not_promote_or_replace_saved_revision(baseline, store):
    service = CalibrationSettingsService(store, baseline)
    before = service.save(patch(x=111))
    store._db.execute("CREATE TEMP TRIGGER reject_calibration BEFORE UPDATE ON runtime_metadata BEGIN SELECT RAISE(ABORT, 'injected disk write failure'); END")
    with pytest.raises(sqlite3.IntegrityError):
        service.save(patch(x=999))
    after = service.read()
    assert before["saved_revision"] == after["saved_revision"]
    assert service.active_snapshot.calibration_revision["revision_id"] == before["active_revision_id"]


@pytest.mark.parametrize("values", [{"x": True}, {"x": 1.5}, {"x": "1"}, {"x": None}, {"x": 2147483648}, {"x": -2147483649}, {"zHigh": 1}, {}])
def test_source_types_not_coercion_or_arbitrary_fields(values):
    with pytest.raises(ValidationError):
        patch(**values)


def test_no_invented_travel_or_increment_bounds(baseline, store):
    result = CalibrationSettingsService(store, baseline).save(patch(x=-2147483648, inc_factor=-2))
    assert not result["pending_restart"]


def test_unknown_or_absent_rows_and_duplicates_do_not_save(baseline, store):
    service = CalibrationSettingsService(store, baseline)
    with pytest.raises(ValidationError):
        CalibrationSettingsPatch.model_validate({"positions": [{"name": "arbitrary", "x": 1}]})
    with pytest.raises(ValidationError):
        CalibrationSettingsPatch.model_validate({"positions": [{"name": "LOC_MS", "x": 1}, {"name": "LOC_MS", "y": 2}]})
    with pytest.raises(ValueError, match="absent"):
        service.save(CalibrationSettingsPatch.model_validate({"positions": [{"name": "LOC_GANTRY", "x": 1}]}))
    assert service.read()["saved_revision"] is None
