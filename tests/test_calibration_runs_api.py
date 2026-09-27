"""Durable OEM comparison through actual REST/SQLite and shared geometry."""
from dataclasses import replace

import pytest
from fastapi.testclient import TestClient

from bioxp import api, oem_machine_bundle as bundle
from bioxp.oem_calibration_settings import CalibrationSettingsService, load_saved_calibration
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.oem_fluid_callers import calwith_fluid
from bioxp.operator_controls import _build_catalog, _motor_motion_action
from tests.test_oem_fluid_callers import setup, REF
from oem_machine_bundle_test_support import bind_serial206_oem_snapshot

PATH = "/motion/oem/calibration_settings"


@pytest.fixture
def rig(monkeypatch, tmp_path):
    baseline = bind_serial206_oem_snapshot(monkeypatch)
    store = OEMRuntimeStore(tmp_path)
    service = CalibrationSettingsService(store, baseline, publish=bundle.apply_owned_calibration_snapshot)
    monkeypatch.setattr(api.app.state, "calibration_settings", service, raising=False)
    monkeypatch.setattr(api, "_get_tester", lambda *a, **k: pytest.fail("hardware acquisition"))
    client = TestClient(api.app)
    yield client, baseline, store, service
    client.close()
    store.close()


@pytest.mark.parametrize("prior", [False, True])
@pytest.mark.parametrize("failed_station", [None, "TC", "MS", "OC", "RC", "STRIP"])
def test_run_recovery_restore_replaces_later_edits(rig, monkeypatch, tmp_path, prior, failed_station):
    client, baseline, store, service = rig
    if prior:
        assert client.patch(PATH, json={"positions": [{"name": "LOC_TC", "zLow": 5555}]}).status_code == 200
    before = service.read()
    bindings, events = setup(service, choice=None)
    scan = bindings.scan
    def failing(plate, *args):
        if plate == failed_station:
            raise RuntimeError("scan failed: " + plate)
        return scan(plate, *args)
    result = calwith_fluid(replace(bindings, scan=failing), service, REF, machine_calibrated=True)
    path = PATH + "/runs/" + result["run_id"]
    run = client.get(path).json()
    assert run["run_id"] == result["run_id"]
    assert run["decision"] is None and run["decision_status"] == "pending"
    assert run["body_completed"] is (failed_station is None)
    assert set(run["before"]) == {"positions", "liquid_calibration", "revision_id"}
    assert set(run["before"]["positions"][0]) == {"name", "x", "y", "zLow", "zDelta", "inc_factor"}
    assert run["before"]["revision_id"] == before["saved_revision_id"]
    assert run["saved_revision_id"] == run["active_revision_id"]
    assert all("scan" not in m for m in run["measurements"])
    assert events[-2:] == [("ui", None), ("acc", 576)]
    accepted = client.post(path + "/decision", json={"decision": "accept"}).json()
    assert accepted["decision"] == "accept"
    assert accepted["accepted_history"] == run["after"]
    assert client.patch(PATH, json={"positions": [{"name": "LOC_RC", "x": 123456}]}).status_code == 200
    # Recover by run identity from a newly opened owner, not callback memory.
    reopened = OEMRuntimeStore(tmp_path)
    recovered = CalibrationSettingsService(reopened, load_saved_calibration(baseline, reopened),
                                          publish=bundle.apply_owned_calibration_snapshot)
    monkeypatch.setattr(api.app.state, "calibration_settings", recovered)
    try:
        assert client.get(path).json()["after"] == run["after"]
        restored = client.post(path + "/decision", json={"decision": "restore"})
        assert restored.status_code == 200, restored.text
        restored = restored.json()
        assert restored["decision"] == "restore" and restored["decision_status"] == "restored"
        assert restored["saved_revision_id"] == before["saved_revision_id"]
        assert restored["active_revision_id"] == before["active_revision_id"]
        assert recovered.read()["active_positions"] == before["active_positions"]
        assert recovered.read()["active_liquid_calibration"] == before["active_liquid_calibration"]
        expected = load_bound_oem_position_table(load_saved_calibration(baseline, reopened))
        assert load_bound_oem_position_table().rows() == expected.rows()
        assert client.get(path).json()["decision"] == "restore"
    finally:
        reopened.close()


@pytest.mark.parametrize("failure", ["compare", "history", "restore"])
def test_internal_comparison_errors_do_not_skip_ui_acceleration(rig, failure):
    _, _, _, service = rig
    bindings, events = setup(service, choice=failure != "restore", failure=failure)
    result = calwith_fluid(bindings, service, REF, machine_calibrated=True)
    assert result["comparison_error"] == failure
    assert events[-2:] == [("ui", None), ("acc", 576)]
    assert not result.get("finalization_error")
    assert service.read_run(result["run_id"])["comparison_error"] == failure


@pytest.mark.parametrize("failure", ["waste", "eject", "park", "completed", "ui", "acc"])
def test_actual_finally_failure_stops_subsequent_source_calls(rig, failure):
    _, _, _, service = rig
    bindings, events = setup(service, choice=None, failure=failure)
    result = calwith_fluid(bindings, service, REF, machine_calibrated=True)
    assert result["finalization_error"] == failure
    assert events[-1][0] == failure
    assert service.read_run(result["run_id"])["finalization_error"] == failure


def test_no_backup_partial_run_has_source_result_not_operator_consent(rig):
    client, _, _, service = rig
    bindings, _ = setup(service, failure="scan")
    result = calwith_fluid(bindings, service, REF, machine_calibrated=False)
    path = PATH + "/runs/" + result["run_id"]
    run = client.post(path + "/decision", json={"decision": "accept"}).json()
    assert run["decision"] is None
    assert run["comparison_choice"] is True and run["comparison_source"] == "no_previous_values"
    assert not run["body_completed"] and run["error"]


def test_restoration_failure_is_reported_and_retryable(rig, monkeypatch):
    client, _, _, service = rig
    bindings, _ = setup(service, choice=None)
    result = calwith_fluid(bindings, service, REF, machine_calibrated=True)
    path = PATH + "/runs/" + result["run_id"]
    def broken(*args):
        raise RuntimeError("restore storage failure")
    monkeypatch.setattr(service, "restore", broken)
    run = client.post(path + "/decision", json={"decision": "restore"}).json()
    assert run["decision"] is None and run["comparison_error"] == "restore storage failure"
    assert client.get(path).json()["comparison_error"] == run["comparison_error"]


def test_station_revision_and_checkpoint_commit_atomically(rig, monkeypatch):
    _, _, store, service = rig
    from bioxp.oem_calibration_settings import CalibrationSettingsPatch
    run = service.begin_run(machine_calibrated=True)
    before = service.read()
    original = store.write_calibration_run
    def broken(payload):
        raise RuntimeError("checkpoint failed")
    monkeypatch.setattr(store, "write_calibration_run", broken)
    with pytest.raises(RuntimeError, match="checkpoint failed"):
        service.save(CalibrationSettingsPatch.model_validate({"positions": [{"name": "LOC_TC", "zLow": 4444}]}),
                     run_id=run["run_id"], measurement={"plate": "TC", "measured_raw_z": 3333})
    assert service.read() == before
    assert service.read_run(run["run_id"])["measurements"] == []
    monkeypatch.setattr(store, "write_calibration_run", original)


def test_run_contract_and_catalog(rig):
    client, _, _, _ = rig
    path = PATH + "/runs/missing"
    assert client.get(path).status_code == 404
    assert client.post(path + "/decision", json={"decision": "restore"}).status_code == 404
    assert client.post(path + "/decision", json={"decision": "reject"}).status_code == 422
    _, actions = _build_catalog(api.app)
    matches = [a for a in actions.values() if a.get("path", "").startswith(PATH + "/runs/")]
    assert {a["method"] for a in matches} == {"GET", "POST"}
    assert all(not _motor_motion_action(a) for a in matches)
