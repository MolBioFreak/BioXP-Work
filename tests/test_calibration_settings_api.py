"""Calibration REST/save/startup consumption; real SQLite, no hardware."""
import pytest
from fastapi.testclient import TestClient

from bioxp import api, oem_machine_bundle
from bioxp.oem_calibration_settings import CalibrationSettingsPatch, CalibrationSettingsService
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_controls import _build_catalog, _motor_motion_action, _safety
from oem_machine_bundle_test_support import bind_serial206_oem_snapshot


PATH = "/motion/oem/calibration_settings"


@pytest.fixture
def calibration(monkeypatch, tmp_path):
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    store = OEMRuntimeStore(tmp_path)
    service = CalibrationSettingsService(store, snapshot, publish=oem_machine_bundle.apply_owned_calibration_snapshot)
    monkeypatch.setattr(api.app.state, "calibration_settings", service, raising=False)

    def no_hardware(*args, **kwargs):
        raise AssertionError("calibration settings must not acquire hardware or require motion readiness")

    monkeypatch.setattr(api, "_get_tester", no_hardware)
    monkeypatch.setattr(api, "_require_motion_route_ready", no_hardware)
    client = TestClient(api.app, raise_server_exceptions=False)
    yield client, snapshot, store, service
    client.close()
    store.close()


def test_settings_http_save_readback_applies_to_active_consumers(calibration):
    client, snapshot, store, service = calibration
    response = client.get(PATH)
    assert response.status_code == 200
    before = response.json()
    assert not before["pending_restart"]
    values = {"x": 0, "y": -20, "zLow": 78000, "zDelta": 19000, "inc_factor": 0}
    response = client.patch(PATH, json={"positions": [{"name": "LOC_RC", **values}]})
    assert response.status_code == 200, response.text
    saved = response.json()
    readback = client.get(PATH).json()
    assert saved["committed_revision_id"] == readback["saved_revision_id"]
    assert readback["application_status"] == "bound_configuration"
    assert readback["active_positions"] == readback["saved_positions"]
    row = next(row for row in readback["saved_positions"] if row["name"] == "LOC_RC")
    assert all(row[key] == value for key, value in values.items())
    assert row["zHigh"] == 59000
    assert not readback["motion_commanded"]
    assert not readback["physical_calibration_verified"]
    assert oem_machine_bundle.get_active_oem_machine_snapshot() is service.active_snapshot
    assert service.active_snapshot.records is snapshot.records
    assert load_bound_oem_position_table().resolve(location_id="LOC_RC").z_low == 78000


@pytest.mark.parametrize("positions", [
    [{"name": "LOC_RC", "x": None}],
    [{"name": "LOC_RC", "x": True}],
    [{"name": "LOC_RC", "x": 1.5}],
    [{"name": "LOC_RC", "x": "1"}],
    [{"name": "LOC_RC", "zHigh": 3}],
    [{"name": "LOC_RC", "x": 1}, {"name": "LOC_RC", "y": 2}],
    [{"name": "nonexistent", "x": 1}],
])
def test_invalid_input_is_a_request_error_not_a_motion_gate(calibration, positions):
    client, _, _, service = calibration
    response = client.patch(PATH, json={"positions": positions})
    assert response.status_code == 422
    assert service.read()["saved_revision"] is None


def test_save_storage_failure_does_not_report_persisted_or_applied(calibration):
    client, _, store, _ = calibration
    assert client.patch(PATH, json={"positions": [{"name": "LOC_RC", "x": 111}]}).status_code == 200
    before = client.get(PATH).json()
    store._db.execute("CREATE TEMP TRIGGER reject_calibration BEFORE UPDATE ON runtime_metadata BEGIN SELECT RAISE(ABORT, 'injected write failure'); END")
    failed = client.patch(PATH, json={"positions": [{"name": "LOC_RC", "x": 222}]})
    assert failed.status_code == 500
    assert client.get(PATH).json() == before


def test_startup_binds_saved_settings_before_provider_consumption(calibration, monkeypatch, tmp_path):
    client, baseline, _, _ = calibration
    response = client.patch(PATH, json={"positions": [{"name": "LOC_RC", "x": 30000}]})
    assert response.status_code == 200
    saved_revision = response.json()["saved_revision_id"]
    monkeypatch.setattr(oem_machine_bundle, "_active_snapshot", None)
    monkeypatch.setattr(oem_machine_bundle, "load_oem_machine_snapshot", lambda *args, **kwargs: baseline)
    monkeypatch.setenv(oem_machine_bundle.OEM_MACHINE_BUNDLE_LOCK_ENV, "/not-read-test-lock")
    monkeypatch.setattr(api, "configure_oem_runtime_state_from_env", lambda snapshot: None)
    snapshot, reopened_store, service = api._configure_machine_calibration(tmp_path)
    try:
        assert snapshot.calibration_revision["revision_id"] == saved_revision
        assert service.read()["active_revision_id"] == saved_revision
        assert not service.read()["pending_restart"]
        assert load_bound_oem_position_table().resolve(location_id="LOC_RC").base_coordinates["x"] == 30000
    finally:
        reopened_store.close()


def test_no_configured_settings_does_not_attempt_hardware(monkeypatch):
    monkeypatch.setattr(api.app.state, "calibration_settings", None, raising=False)
    client = TestClient(api.app)
    try:
        assert client.get(PATH).status_code == 503
    finally:
        client.close()


def test_catalog_settings_read_and_save_are_not_motion():
    assert _safety("GET", PATH) == "read_only"
    assert _safety("PATCH", PATH) == "service"
    _, actions = _build_catalog(api.app)
    matches = [a for a in actions.values() if a.get("path") == PATH]
    assert {a["method"] for a in matches} == {"GET", "PATCH"}
    assert all(not _motor_motion_action(a) for a in matches)


def test_native_schema_does_not_offer_rejected_null_defaults():
    schema = CalibrationSettingsPatch.model_json_schema()["$defs"]["PositionCalibrationPatch"]
    assert schema["required"] == ["name"]
    assert schema["minProperties"] == 2
    for name in ("x", "y", "zLow", "zDelta", "inc_factor"):
        field = schema["properties"][name]
        assert field["type"] == "integer"
        assert (field["minimum"], field["maximum"]) == (-2147483648, 2147483647)
        assert "anyOf" not in field and "default" not in field
