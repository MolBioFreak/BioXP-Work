"""Actual HTTP/provider/SQLite and runtime-state writer, hardware I/O replaced."""
from dataclasses import replace
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from bioxp import api, oem_machine_bundle, runtime_state
from bioxp.oem_calibration_settings import CalibrationSettingsService
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.runtime_state import OemRuntimeStateStore
from bioxp.operator_controls import _build_catalog, _motor_motion_action, _safety
from oem_machine_bundle_test_support import bind_serial206_oem_snapshot

SET = "/motion/oem/pipette/tip_tray_set"
SETTINGS = "/liquid/pipette/settings"


@pytest.fixture
def mounted(monkeypatch, tmp_path):
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    store = OEMRuntimeStore(tmp_path / "calibration")
    service = CalibrationSettingsService(store, snapshot, publish=oem_machine_bundle.apply_owned_calibration_snapshot)
    monkeypatch.setattr(api.app.state, "calibration_settings", service, raising=False)
    runtime = OemRuntimeStateStore(tmp_path / "mutable", replace(snapshot, operator_label_matched=True))
    monkeypatch.setattr(api, "get_active_oem_runtime_state_store", lambda: runtime)
    positions = []
    provider = SimpleNamespace(primitives=SimpleNamespace(_read_axis_position=lambda axis: (positions.append(axis), 32123)[1]))
    monkeypatch.setattr(api, "_require_serial206_oem_initialization_provider", lambda _: provider)
    monkeypatch.setattr(api, "_get_tester", lambda: pytest.fail("hardware transport must not be acquired"))
    client = TestClient(api.app, raise_server_exceptions=False)
    yield client, store, snapshot, runtime, provider, positions
    client.close()
    store.close()


@pytest.mark.parametrize("tray,pair", [(1, ("TECANRACK1", "TECANRACK2")), (2, ("TECANRACK1", "TECANRACK2")),
                                        (3, ("TECANRACK3", "TECANRACK4")), (4, ("TECANRACK3", "TECANRACK4"))])
def test_manual_set_paired_saved_readback_and_active_geometry(mounted, tray, pair):
    client, store, snapshot, _, _, positions = mounted
    before = client.get("/motion/oem/calibration_settings").json()
    response = client.post(SET, json={"tray": tray})
    assert response.status_code == 200, response.text
    data = response.json()
    assert positions == ["z"]
    assert data["measured_z_steps"] == 32123
    assert data["paired_positions"] == list(pair)
    assert not data["pending_restart"] and not data["motion_commanded"]
    assert {r["name"]: r["zLow"] for r in data["saved_positions"]} == dict.fromkeys(pair, 32123)
    readback = client.get("/motion/oem/calibration_settings").json()
    assert readback["saved_revision_id"] == data["committed_revision_id"]
    assert readback["active_positions"] == readback["saved_positions"]
    assert all(load_bound_oem_position_table().resolve(location_id=name).z_low == 32123 for name in pair)
    assert oem_machine_bundle.get_active_oem_machine_snapshot().records is snapshot.records


def test_manual_set_failures_do_not_commit(mounted):
    client, store, _, _, provider, positions = mounted
    for value in (0, 5, True, "1"):
        assert client.post(SET, json={"tray": value}).status_code == 422
    assert positions == []
    provider.primitives._read_axis_position = lambda _: (_ for _ in ()).throw(RuntimeError("controller unavailable"))
    assert client.post(SET, json={"tray": 1}).status_code == 500
    assert client.get("/motion/oem/calibration_settings").json()["saved_revision"] is None
    provider.primitives._read_axis_position = lambda _: 123
    assert client.post(SET, json={"tray": 1}).status_code == 200
    before = client.get("/motion/oem/calibration_settings").json()
    store._db.execute("CREATE TEMP TRIGGER reject_set BEFORE UPDATE ON runtime_metadata BEGIN SELECT RAISE(ABORT, 'write failure'); END")
    assert client.post(SET, json={"tray": 1}).status_code == 500
    assert client.get("/motion/oem/calibration_settings").json() == before


def test_tip_tray_set_applies_to_both_rows_after_ordinary_startup(mounted, monkeypatch):
    client, store, baseline, _, _, _ = mounted
    assert client.post(SET, json={"tray": 4}).status_code == 200
    revision = client.get("/motion/oem/calibration_settings").json()["saved_revision_id"]
    monkeypatch.setattr(oem_machine_bundle, "_active_snapshot", None)
    monkeypatch.setattr(oem_machine_bundle, "load_oem_machine_snapshot", lambda *args, **kwargs: baseline)
    monkeypatch.setenv(oem_machine_bundle.OEM_MACHINE_BUNDLE_LOCK_ENV, "/not-read-test-lock")
    monkeypatch.setattr(api, "configure_oem_runtime_state_from_env", lambda snapshot: None)
    reopened, reopened_store, service = api._configure_machine_calibration(store.root)
    try:
        assert service.read()["active_revision_id"] == revision
        assert not service.read()["pending_restart"]
        assert all(load_bound_oem_position_table().resolve(location_id=name).z_low == 32123
                   for name in ("TECANRACK3", "TECANRACK4"))
    finally:
        reopened_store.close()


def test_consumed_flags_http_writer_and_readback(mounted):
    client, _, snapshot, runtime, _, _ = mounted
    before = runtime.read_operation_parameters()
    response = client.patch(SETTINGS, json={"LogPressure": True, "CheckForStaticTipLoss": True})
    assert response.status_code == 200, response.text
    readback = client.get(SETTINGS).json()
    assert readback["runtime_values"] == {"LogPressure": True, "CheckForStaticTipLoss": True}
    assert runtime.read_operation_parameters() != before
    assert runtime.operation_parameters_projection()["LogPressure"] is True
    assert snapshot.operation_parameters["LogPressure"] is False
    assert OemRuntimeStateStore(runtime.base_root, runtime.snapshot).operation_parameters_projection()["LogPressure"] is True
    assert client.patch(SETTINGS, json={"LogPressure": False}).status_code == 200
    assert client.get(SETTINGS).json()["runtime_values"] == {"LogPressure": False, "CheckForStaticTipLoss": True}


@pytest.mark.parametrize("body", [{}, {"LogPressure": None}, {"LogPressure": 1}, {"CheckSnapTips": True}, {"LogPressure": "true"}])
def test_invalid_settings_never_mutate(mounted, body):
    client, _, _, runtime, _, _ = mounted
    before = runtime.read_operation_parameters()
    assert client.patch(SETTINGS, json=body).status_code in (400, 422)
    assert runtime.read_operation_parameters() == before


def test_settings_failed_atomic_write_does_not_publish(mounted, monkeypatch):
    client, _, _, runtime, _, _ = mounted
    before = client.get(SETTINGS).json()
    original = runtime_state._atomic_replace

    def fail_operation(path, data):
        if path.name == "Operation_parameters.xml":
            raise OSError("injected storage failure")
        return original(path, data)

    monkeypatch.setattr(runtime_state, "_atomic_replace", fail_operation)
    assert client.patch(SETTINGS, json={"LogPressure": True}).status_code == 500
    assert client.get(SETTINGS).json() == before


def test_catalog_and_native_typed_schema():
    _, actions = _build_catalog(api.app)
    for path, methods in ((SET, {"POST"}), (SETTINGS, {"GET", "PATCH"})):
        selected = [row for row in actions.values() if row.get("path") == path]
        assert {row["method"] for row in selected} == methods
        assert all(not _motor_motion_action(row) for row in selected)
        assert _safety("PATCH", path) == "service"
    schema = api.PipetteOperationSettingsPatch.model_json_schema()
    assert set(schema["properties"]) == {"LogPressure", "CheckForStaticTipLoss"}
    assert schema["minProperties"] == 1
    assert all(field["type"] == "boolean" and "anyOf" not in field and "default" not in field
               for field in schema["properties"].values())
