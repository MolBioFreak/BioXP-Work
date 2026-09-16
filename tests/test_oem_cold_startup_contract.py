"""Cold API aggregate + ordinary adapter + native cycle/profile owner.

Reuse the isolated native rig unchanged; only physical exchanges and pipette
construction are doubled. No generation/profile/preparation flags are seeded.
"""
import pytest
from bioxp import api
from bioxp.lifecycle_state import CanonicalLifecycleOwner, LifecycleStateError
from tests.test_wake_setup_debloat import rig, cycle


def startup(monkeypatch, driver):
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason="isolated cold transport")
    monkeypatch.setattr(api, "lifecycle_state", owner)
    monkeypatch.setattr(api, "_get_tester", lambda: driver)
    monkeypatch.setattr(api, "_can_ready_observation", lambda: True)
    monkeypatch.setattr(api, "_constructor_pipette_action", lambda: {"ok": True})
    monkeypatch.setattr(driver, "strip_set_rgb", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(driver, "query_only_tmcl", lambda b, c, t, m, v: {
        "status": 100, "value": 0 if t == 0 else 1})
    monkeypatch.setattr(driver, "deck_io_set_type", lambda *a: {"ok": True})
    # Ordinary transport board activation is not a completed initialCheck cycle.
    driver.activate_boards()
    assert driver.oem_current_board_lifecycle_generation() is None
    return owner


def run():
    return api._run_oem_non_motion_startup_sequence(sleep=lambda _: None)


def test_cold_aggregate_then_native_initialization(monkeypatch, rig):
    driver, provider, frames, fault, root = rig
    owner = startup(monkeypatch, driver)
    result = run()
    assert result["ok"] is True, result
    stages = result["lifecycle"]["startup"]["stages"]
    assert result["sequence"] == list(stages) == [
        "constructor_pipette_stage", "initialization_without_motion", "initial_check"]
    config = stages["initialization_without_motion"]["evidence"]["motor_current_verification"]
    assert config["ok"] is True and config["generation_bound"] is False
    assert config["board_lifecycle_generation"] is None
    assert config["failures"] == config["no_replies"] == []
    generation = stages["initial_check"]["evidence"]["board_lifecycle_generation"]
    assert generation["board_lifecycle_generation"] == 1
    assert "legacy_harness_without_generation_binding" not in generation
    assert driver._oem_no_motion_profile_generations == {}
    assert driver._oem_no_motion_profiles_ready == set()
    assert driver._oem_no_motion_profile_ready is False
    first_configuration = next(i for i, row in enumerate(frames) if row[1] == 5)
    first_deactivation = next(i for i, row in enumerate(frames) if row[1] == 64 and row[4] == 0)
    assert first_configuration < first_deactivation
    assert not any(row[1] in (2, 3, 4, 30) for row in frames)
    # Actual generation-scoped native current readbacks, not preparation rebasing.
    before = len(frames)
    current = driver.motor_oem_require_no_motion_profile()
    assert current["ok"] is True
    assert current["board_lifecycle_generation"] == 1
    assert all(row[1] == 6 for row in frames[before:])
    assert driver._oem_no_motion_profile_generations == {}
    initialized = provider.initialize_motors(mode="live")
    assert initialized["ok"] is True
    assert all(row["published"] for row in initialized["reference_publications"].values())
    assert provider.state_store.read_oem_serial206_initialization_state()["preparation"]["state"] == "not_started"
    with pytest.raises(LifecycleStateError, match="fresh ownership epoch"):
        run()


@pytest.mark.parametrize("bad_board", [4, 5, 6, 7])
def test_ordinary_adapter_rejects_partial_cycle(monkeypatch, rig, bad_board):
    driver, provider, frames, fault, root = rig
    startup(monkeypatch, driver)
    fault["reject"] = (bad_board, 64, 0, 0)
    result = run()
    assert result["ok"] is False and result["failed_stage"] == "initial_check"
    assert driver.oem_current_board_lifecycle_generation() is None
    assert not any(row[1] in (2, 3, 4, 30) for row in frames)


def test_constructor_write_failure_stops_before_cycle(monkeypatch, rig):
    driver, provider, frames, fault, root = rig
    startup(monkeypatch, driver)
    fault["reject"] = (5, 5, 4, 0)
    result = run()
    assert result["failed_stage"] == "initialization_without_motion"
    assert not any(row[1] == 64 and row[4] == 0 for row in frames)
    assert driver.oem_current_board_lifecycle_generation() is None


def test_ordinary_adapter_cannot_use_legacy_generation_fallback(monkeypatch, rig):
    driver, provider, frames, fault, root = rig
    startup(monkeypatch, driver)
    monkeypatch.setattr(driver, "oem_begin_board_lifecycle_generation", None)
    result = run()
    assert result["failed_stage"] == "initial_check"
    assert "initial_check_generation_unavailable" in result["lifecycle"]["startup"]["stages"]["initial_check"]["error"]
    assert driver.oem_current_board_lifecycle_generation() is None


def test_repeat_initial_check_preserves_constructor_history(monkeypatch, rig):
    driver, provider, frames, fault, root = rig
    owner = startup(monkeypatch, driver)
    assert run()["ok"] is True
    original = owner.projection()["startup"]["stages"]["initialization_without_motion"]
    before = len(frames)
    result = owner.run_initial_check(api._LifecycleHardware(driver), can_ready=lambda: True, sleep=lambda _: None)
    assert result["startup"]["state"] == "passed"
    assert result["startup"]["stages"]["initialization_without_motion"] == original
    assert driver.oem_current_board_lifecycle_generation() == 2
    assert not any(row[1] == 5 for row in frames[before:])
    assert driver._oem_no_motion_profile_generations == {}


@pytest.mark.parametrize("fault_kind", ["mismatch", "rejected", "missing", "generation_drift"])
def test_current_profile_requires_fresh_successful_stable_reads(monkeypatch, rig, fault_kind):
    driver, provider, frames, fault, root = rig
    startup(monkeypatch, driver)
    assert run()["ok"] is True
    original = driver.motor_get_axis_param
    def read(board, param, **kwargs):
        row = original(board, param, **kwargs)
        if param == 4:
            if fault_kind == "mismatch":
                row["value"] += 1
            elif fault_kind == "rejected":
                row["ack"]["status"] = 1
            elif fault_kind == "missing":
                row["ack"] = None
            else:
                cycle(driver)
        return row
    monkeypatch.setattr(driver, "motor_get_axis_param", read)
    with pytest.raises(RuntimeError, match="profile readback mismatch|generation changed"):
        driver.motor_oem_require_no_motion_profile("z")
    assert driver._oem_no_motion_profile_generations == {}


def test_current_profile_without_cycle_refuses_before_reads(rig):
    driver, provider, frames, fault, root = rig
    before = len(frames)
    with pytest.raises(RuntimeError, match="active board lifecycle generation"):
        driver.motor_oem_require_no_motion_profile("z")
    assert frames[before:] == []
