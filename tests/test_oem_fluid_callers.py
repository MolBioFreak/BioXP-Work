import pytest

from bioxp.oem_calibration_settings import CalibrationSettingsPatch, CalibrationSettingsService
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.oem_fluid_callers import FluidCallerBindings, calwith_fluid, detect_fluid
from oem_machine_bundle_test_support import bind_serial206_oem_snapshot


@pytest.fixture
def settings(monkeypatch, tmp_path):
    baseline = bind_serial206_oem_snapshot(monkeypatch)
    store = OEMRuntimeStore(tmp_path)
    yield CalibrationSettingsService(store, baseline)
    store.close()


def setup(settings=None, *, choice=True, failure=None):
    events = []
    def event(name, value=None):
        events.append((name, value))
        if name == failure:
            raise RuntimeError(failure)
    def scan(plate, speed, transfer, skip):
        event("scan", (plate, speed, transfer, skip))
        return {"source_return": {"TC": 2000, "MS": 2100, "OC": 2200, "RC": 2300, "STRIP": 2400}[plate]}
    def restore(previous):
        event("restore", None if previous is None else previous["revision_id"])
        if previous is None:
            # Owner of the canonical store is responsible for an atomic removal.
            settings.store._db.execute("DELETE FROM runtime_metadata WHERE key=?",
                ("machine_calibration_v1:" + settings.active_snapshot.lock_sha256,))
        else:
            settings.store.update_machine_calibration_revision(settings.active_snapshot.lock_sha256, lambda _: previous)
    b = FluidCallerBindings(
        log_file=lambda: event("log"), initiate_group=lambda: event("initiate"),
        catch_plate=lambda n: event("catch", n),
        release_plate=lambda n, press: event("release", (n, press)),
        press_plates=lambda p: event("press", p), scan=scan,
        mark_strip=lambda: event("strip"), park=lambda: event("park"),
        error=lambda name, exc: event("error", name),
        reset_status=lambda: event("reset"), tip_exists=lambda: event("tip_exists") or True,
        move_waste=lambda: event("waste"), sleep_ms=lambda ms: event("sleep", ms),
        eject_tips=lambda: event("eject"), completed=lambda: event("completed"),
        finish_ui=lambda: event("ui"), set_z_acceleration=lambda n: event("acc", n),
        compare=lambda before, after: event("compare") or choice,
        save_history=lambda: event("history"), restore=restore)
    return b, events


REF = {"REVISION": 17, "FLUID_TC_OFFSET": 100, "FLUID_RC_OFFSET": 200, "FLUID_STRIP_OFFSET": 300}


def test_diagnostic_order_and_early_catch():
    b, events = setup()
    r = detect_fluid(b)
    assert r["completed"] and len(r["scans"]) == 5
    assert events == [("log", None), ("catch", 0), ("release", (23, True)), ("initiate", None),
        ("scan", ("TC", 300, False, 1)), ("catch", 0), ("release", (25, True)),
        ("scan", ("MS", 300, False, 1)), ("press", (1,)), ("scan", ("OC", 300, False, 1)),
        ("press", (2,)), ("scan", ("RC", 300, False, 1)), ("strip", None),
        ("scan", ("STRIP", 300, False, 1)), ("park", None)]
    b, events = setup(failure="scan")
    assert not detect_fluid(b)["completed"]
    assert events[-1] == ("error", "btnDetectFluid_Click")
    assert ("park", None) not in events


def test_calibration_saves_each_adjustment_and_next_startup_only(settings):
    b, events = setup(settings)
    original = {r["name"]: r["zLow"] for r in settings.read()["active_positions"]}
    r = calwith_fluid(b, settings, REF, machine_calibrated=True)
    assert r["outcome"] == "accepted" and r["body_completed"]
    assert [m["plate"] for m in r["measurements"]] == ["TC", "MS", "OC", "RC", "STRIP"]
    assert len({m["saved_revision_id"] for m in r["measurements"]}) == 5
    assert r["measurements"][-1]["calculated_z_lows"]["LOC_STRIP2"] == 1200
    assert r["fluid_reference_revision"] == "17"
    assert r["pending_restart"] and r["active_revision_id"] is None
    state = settings.read()
    saved = {p["name"]: p["zLow"] for p in state["saved_positions"]}
    assert {p["name"]: p["zLow"] for p in state["active_positions"]} == original
    assert [saved[n] for n in ("LOC_TC", "LOC_MS", "LOC_OC", "LOC_RC")] == [2100, 2200, 2300, 2500]
    assert saved["LOC_STRIP1"] - original["LOC_STRIP1"] == 1200 - original["LOC_STRIP2"]
    assert events[:5] == [("log", None), ("reset", None), ("press", (0,)), ("scan", ("TC", 300, True, 4)), ("catch", 0)]
    assert events[-10:] == [("waste", None), ("sleep", 20), ("eject", None), ("sleep", 20),
        ("park", None), ("completed", None), ("compare", None), ("history", None), ("ui", None), ("acc", 576)]


def test_reject_restores_prior_revision_and_failure_still_finalizes(settings):
    prior = settings.save(CalibrationSettingsPatch.model_validate({"positions": [{"name": "LOC_TC", "zLow": 7777}]}))
    b, events = setup(settings, choice=False)
    r = calwith_fluid(b, settings, REF, machine_calibrated=True)
    assert r["outcome"] == "rejected_restored"
    assert r["saved_revision_id"] == prior["saved_revision_id"]
    assert settings.read()["saved_revision_id"] == prior["saved_revision_id"]
    assert ("history", None) not in events and ("restore", prior["saved_revision_id"]) in events
    b, events = setup(settings, failure="scan")
    r = calwith_fluid(b, settings, REF, machine_calibrated=True)
    assert r["outcome"] == "incomplete" and not r["body_completed"]
    assert events[-3:] == [("compare", None), ("ui", None), ("acc", 576)]


def test_no_explicit_acceptance_is_incomplete(settings):
    b, events = setup(settings, choice=None)
    r = calwith_fluid(b, settings, REF, machine_calibrated=True)
    assert r["outcome"] == "incomplete" and r["pending_restart"]
    assert ("history", None) not in events


def test_reject_without_previous_revision_restores_absence(settings):
    b, events = setup(settings, choice=False)
    r = calwith_fluid(b, settings, REF, machine_calibrated=True)
    assert r["outcome"] == "rejected_restored"
    assert r["saved_revision_id"] is None and not r["pending_restart"]
    assert settings.read()["saved_revision"] is None


def test_absent_backup_does_not_claim_user_acceptance(settings):
    b, events = setup(settings, choice=True)
    r = calwith_fluid(b, settings, REF, machine_calibrated=False)
    assert r["body_completed"] and r["outcome"] == "incomplete"
    assert ("compare", None) not in events and ("history", None) not in events
