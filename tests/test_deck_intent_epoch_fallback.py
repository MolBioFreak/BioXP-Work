"""Intent deck contract must hold through transient authority-blank windows.

The catalog's expected board epochs are read from the live initialization
projection; during an authority-busy window or a mid-reset projection that
envelope is unbound, which used to disable the deck lane with
``deck_board_epochs_not_authoritative`` and an empty epoch map even though the
durable deck authority still knew both epochs. The contract now fills missing
epochs from the cache-local last deck-authority snapshot; honestly
unavailable authority (no live epochs and no cached snapshot) still disables.
"""
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import FastAPI

from bioxp import operator_controls as controls
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.oem_runtime_store import OEMRuntimeStore
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot

BUSY = ["provider_authority_busy"]


class DeckProvider:
    deck_scoped_authority_version = 1

    def __init__(self, cached):
        self._cached = cached
        self.cached_calls = []

    def movement_lease(self, **kwargs):
        return {}

    def force_to_high_home(self, **kwargs):
        return {"ok": True}

    def deck_authority_snapshot(self, **kwargs):
        return {}

    def deck_authority_cached_snapshot(self, *, expected_generation, target=None):
        self.cached_calls.append((expected_generation, target))
        if isinstance(self._cached, Exception):
            raise self._cached
        return self._cached

    def moveTo(self, **kwargs):
        return {"ok": True}

    def moveZCamera(self, **kwargs):
        return {"ok": True}

    def parkGantry(self, **kwargs):
        return {"ok": True}


@pytest.fixture
def assessment_rig(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    OEMRuntimeStore(tmp_path).close()
    bind_serial206_oem_snapshot(monkeypatch)
    monkeypatch.setattr(controls, "current_release_identity", lambda: {
        "verified": True, "release_id": "offline-intent",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64}})
    monkeypatch.setattr(controls, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True, "evidence_lock_sha256": "3" * 64})
    monkeypatch.setattr(controls, "current_registry_sha256", lambda: "4" * 64)

    class Hardware:
        ownership_epoch = 7

        def ownership_projection(self):
            return {"ownership_epoch": 7, "ownership": {
                "transport": "owned", "usb": "service", "router": "running", "CAN_READY": True}}

        def project(self, *domains, **kw):
            return {"snapshot_id": "offline", "domains": {},
                    "freshness": {"state": "fresh", "age_s": 0.0, "fresh_for_s": 30.0}}

    monkeypatch.setattr(controls, "hardware_state", Hardware())

    def build(cached, request):
        provider = DeckProvider(cached)
        app = FastAPI()
        controls.install_operator_control_plane(
            app,
            maintenance_state_provider=lambda: {"motion_blocked": False, "recovery_required": False},
            reference_state_provider=lambda: {"rows": {}},
            lifecycle_state_provider=lambda: {"operation_state": "stopped"},
            serial206_initialization_state_provider=lambda: {"bound": True},
            oem_deck_provider=lambda: provider,
            oem_deck_position_table_provider=load_bound_oem_position_table)
        plane = getattr(app.state, "operator_command_plane", None)
        if plane is not None:
            request.addfinalizer(plane.stop)
        return provider, app

    yield build


def machine_state(provider_envelope):
    return {
        "maintenance": {"motion_blocked": False, "recovery_required": False},
        "lifecycle": {"operation_state": "stopped"},
        "ownership_generation": 7,
        "references": {"rows": {}},
        "serial206_initialization_provider": provider_envelope,
    }


def busy_envelope():
    return {
        "bound": True,
        "authority_busy": True,
        "x_authority": {"available": False, "state": "unbound", "blockers": BUSY},
        "y_authority": {"available": False, "state": "unbound", "blockers": BUSY},
    }


def live_envelope():
    return {
        "bound": True,
        "x_authority": {"current_board_lifecycle_generation": 44},
        "board4_authority": {"active_board_epoch": 1059},
    }


def test_busy_envelope_falls_back_to_cached_epochs(assessment_rig, request):
    provider, app = assessment_rig({"board_epoch_4": 1059, "board_epoch_5": 44}, request)
    assessment = app.state.oem_deck_command_assessment(machine_state(busy_envelope()))
    assert assessment["enabled"] is True, assessment
    assert assessment["disabled_reason"] is None
    assert assessment["expected_board_epoch_by_board"] == {"4": 1059, "5": 44}
    assert provider.cached_calls == [(7, "LOC_MS")]
    assert assessment["destination_options"], "destination options must stay selectable"


def test_missing_cache_still_disables_with_named_reason(assessment_rig, request):
    _, app = assessment_rig(RuntimeError("deck_authority_cache_unavailable"), request)
    assessment = app.state.oem_deck_command_assessment(machine_state(busy_envelope()))
    assert assessment["enabled"] is False
    assert assessment["disabled_reason"] == "deck_board_epochs_not_authoritative"
    assert not [row for row in assessment["destination_options"] if row.get("enabled")]


def test_complete_live_envelope_skips_the_fallback(assessment_rig, request):
    provider, app = assessment_rig(RuntimeError("must not be read"), request)
    assessment = app.state.oem_deck_command_assessment(machine_state(live_envelope()))
    assert assessment["enabled"] is True, assessment
    assert assessment["expected_board_epoch_by_board"] == {"4": 1059, "5": 44}
    assert provider.cached_calls == []
