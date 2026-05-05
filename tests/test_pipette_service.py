import asyncio
import importlib
import sys
import types

import pytest
from fastapi import HTTPException
from pydantic import ValidationError

from src.bioxp.pipette.models import (
    LiquidLocation,
    PipetteAspirateCommand,
    PipetteDispenseCommand,
    PipetteInitCommand,
    PipetteMixCommand,
    PipettePreflightError,
    PipetteTipAction,
    PipetteTipCommand,
    PipetteTipStateError,
)
from src.bioxp.pipette.transport import CanPipetteTransport
from src.bioxp.services.pipette_service import (
    run_pipette_aspirate_command,
    run_pipette_dispense_command,
    run_pipette_init_command,
    run_pipette_mix_command,
    run_pipette_status,
    run_pipette_tip_command,
)


class FakeTransport:
    def __init__(self):
        self.calls = []
        self.initialized = False
        self.tip_loaded = False

    def get_status(self):
        self.calls.append(("status", None))
        return {
            "ok": True,
            "initialized": self.initialized,
            "tip_loaded": self.tip_loaded,
            "transport": "fake",
        }

    def initialize(self, command):
        self.calls.append(("initialize", command))
        self.initialized = True
        return {
            "ok": True,
            "initialized": True,
            "tip_loaded": self.tip_loaded,
            "pressure_profile": command.pressure_profile,
        }

    def set_tip(self, command):
        self.calls.append(("set_tip", command))
        self.tip_loaded = command.action is PipetteTipAction.LOAD
        return {
            "ok": True,
            "initialized": self.initialized,
            "tip_loaded": self.tip_loaded,
            "action": command.action.value,
        }

    def aspirate(self, command):
        self.calls.append(("aspirate", command))
        if not self.tip_loaded:
            raise PipetteTipStateError("Tip must be loaded before aspirating.")
        return {
            "ok": True,
            "volume_ul": command.volume_ul,
            "pressure_profile": command.pressure_profile,
            "tip_loaded": self.tip_loaded,
        }

    def dispense(self, command):
        self.calls.append(("dispense", command))
        if not self.tip_loaded:
            raise PipetteTipStateError("Tip must be loaded before dispensing.")
        return {
            "ok": True,
            "volume_ul": command.volume_ul,
            "pressure_profile": command.pressure_profile,
            "blow_out": command.blow_out,
            "tip_loaded": self.tip_loaded,
        }

    def mix(self, command):
        self.calls.append(("mix", command))
        if not self.tip_loaded:
            raise PipetteTipStateError("Tip must be loaded before mixing.")
        return {
            "ok": True,
            "cycles": command.cycles,
            "volume_ul": command.volume_ul,
            "pressure_profile": command.pressure_profile,
            "tip_loaded": self.tip_loaded,
        }


async def _fake_run_blocking(label, func, timeout_s=30.0):
    del label, timeout_s
    return func()


def load_api(monkeypatch):
    usb_pkg = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_pkg.core = usb_core
    usb_pkg.util = usb_util
    monkeypatch.setitem(sys.modules, "usb", usb_pkg)
    monkeypatch.setitem(sys.modules, "usb.core", usb_core)
    monkeypatch.setitem(sys.modules, "usb.util", usb_util)
    for name in [
        "src.bioxp.api",
        "src.bioxp.usb_driver",
        "src.bioxp.pipette.models",
        "src.bioxp.pipette.transport",
        "src.bioxp.pipette",
        "src.bioxp.services.pipette_service",
        "src.bioxp.services",
        "src.bioxp",
    ]:
        sys.modules.pop(name, None)
    return importlib.import_module("src.bioxp.api")


def test_run_pipette_status_uses_transport_snapshot():
    transport = FakeTransport()

    result = asyncio.run(
        run_pipette_status(
            get_transport=lambda: transport,
            run_blocking=_fake_run_blocking,
        )
    )

    assert result["transport"] == "fake"
    assert transport.calls == [("status", None)]


def test_run_pipette_aspirate_command_delegates_to_transport():
    transport = FakeTransport()
    transport.initialized = True
    transport.tip_loaded = True
    command = PipetteAspirateCommand(volume_ul=25.0, pressure_profile="1R")

    result = asyncio.run(
        run_pipette_aspirate_command(
            command,
            get_transport=lambda: transport,
            run_blocking=_fake_run_blocking,
        )
    )

    assert result["ok"] is True
    assert result["volume_ul"] == 25.0
    assert transport.calls[-1][0] == "aspirate"


def test_run_pipette_tip_error_maps_to_http_exception():
    transport = FakeTransport()
    command = PipetteAspirateCommand(volume_ul=10.0, pressure_profile="1R")

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            run_pipette_aspirate_command(
                command,
                get_transport=lambda: transport,
                run_blocking=_fake_run_blocking,
            )
        )

    assert exc_info.value.status_code == 409
    assert "Tip must be loaded" in str(exc_info.value.detail)


def test_run_pipette_mix_command_delegates_to_transport():
    transport = FakeTransport()
    transport.initialized = True
    transport.tip_loaded = True
    command = PipetteMixCommand(volume_ul=15.0, cycles=3, pressure_profile="1R")

    result = asyncio.run(
        run_pipette_mix_command(
            command,
            get_transport=lambda: transport,
            run_blocking=_fake_run_blocking,
        )
    )

    assert result["cycles"] == 3
    assert transport.calls[-1][0] == "mix"


def test_liquid_routes_delegate_to_pipette_service(monkeypatch):
    api = load_api(monkeypatch)
    calls = []

    async def fake_status(**kwargs):
        calls.append(("status", kwargs))
        return {"ok": True, "transport": "fake"}

    async def fake_init(command, **kwargs):
        calls.append(("init", command))
        return {"ok": True, "initialized": True}

    async def fake_tip(command, **kwargs):
        calls.append(("tip", command))
        return {"ok": True, "action": command.action.value}

    async def fake_aspirate(command, **kwargs):
        calls.append(("aspirate", command))
        return {"ok": True, "volume_ul": command.volume_ul}

    async def fake_dispense(command, **kwargs):
        calls.append(("dispense", command))
        return {"ok": True, "volume_ul": command.volume_ul}

    async def fake_mix(command, **kwargs):
        calls.append(("mix", command))
        return {"ok": True, "cycles": command.cycles}

    monkeypatch.setattr(api, "run_pipette_status", fake_status)
    monkeypatch.setattr(api, "run_pipette_init_command", fake_init)
    monkeypatch.setattr(api, "run_pipette_tip_command", fake_tip)
    monkeypatch.setattr(api, "run_pipette_aspirate_command", fake_aspirate)
    monkeypatch.setattr(api, "run_pipette_dispense_command", fake_dispense)
    monkeypatch.setattr(api, "run_pipette_mix_command", fake_mix)

    status = asyncio.run(api.liquid_status())
    init = asyncio.run(api.liquid_init(api.PipetteInitRequest(pressure_profile="1R")))
    tip = asyncio.run(api.liquid_tip(api.PipetteTipRequest(action=api.PipetteTipAction.LOAD)))
    aspirate = asyncio.run(api.liquid_aspirate(api.PipetteAspirateRequest(volume_ul=20.0, pressure_profile="1R")))
    dispense = asyncio.run(api.liquid_dispense(api.PipetteDispenseRequest(volume_ul=20.0, pressure_profile="1R", blow_out=True)))
    mix = asyncio.run(api.liquid_mix(api.PipetteMixRequest(volume_ul=10.0, cycles=2, pressure_profile="1R")))

    assert status["transport"] == "fake"
    assert init["initialized"] is True
    assert tip["action"] == "load"
    assert aspirate["volume_ul"] == 20.0
    assert dispense["volume_ul"] == 20.0
    assert mix["cycles"] == 2
    assert [name for name, _ in calls] == ["status", "init", "tip", "aspirate", "dispense", "mix"]


def test_liquid_aspirate_request_validates_positive_volume(monkeypatch):
    api = load_api(monkeypatch)

    with pytest.raises(ValidationError):
        api.PipetteAspirateRequest(volume_ul=0.0, pressure_profile="1R")


def test_liquid_dispense_request_validates_positive_volume(monkeypatch):
    api = load_api(monkeypatch)

    with pytest.raises(ValidationError):
        api.PipetteDispenseRequest(volume_ul=-1.0, pressure_profile="1R")


def test_pipette_command_payload_preserves_semantic_location_context():
    command = PipetteAspirateCommand(
        volume_ul=12.5,
        pressure_profile="1r",
        source={
            "location_id": "reagent_rack",
            "well_id": "b2",
            "plate_name": "Plate A",
            "z_offset_steps": 42,
        },
        liquid_class="glycerol_mix",
        tip_id="tip-001",
        air_gap_ul=2.0,
        operator="Christian",
        metadata={"run": "smoke"},
    )

    payload = command.to_payload()

    assert payload["pressure_profile"] == "1R"
    assert payload["source"] == {
        "location_id": "reagent_rack",
        "well_id": "B2",
        "plate_name": "Plate A",
        "z_offset_steps": 42,
    }
    assert payload["liquid_class"] == "glycerol_mix"
    assert payload["tip_id"] == "tip-001"
    assert payload["air_gap_ul"] == 2.0
    assert payload["operator"] == "Christian"
    assert payload["metadata"] == {"run": "smoke"}


def test_run_pipette_command_blocks_transport_when_preflight_fails():
    transport = FakeTransport()
    transport.initialized = True
    transport.tip_loaded = True
    command = PipetteAspirateCommand(
        volume_ul=25.0,
        pressure_profile="1R",
        source=LiquidLocation(location_id="reagent_rack", well_id="A1"),
    )

    def failing_preflight(operation, cmd):
        assert operation == "aspirate"
        assert cmd is command
        raise PipettePreflightError(
            "Liquid operation requires referenced x/y/z axes before pipetting.",
            details={"missing_reference_axes": ["x", "y", "z"]},
        )

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            run_pipette_aspirate_command(
                command,
                get_transport=lambda: transport,
                run_blocking=_fake_run_blocking,
                preflight=failing_preflight,
            )
        )

    assert exc_info.value.status_code == 409
    assert exc_info.value.detail["error"] == "preflight_failed"
    assert transport.calls == []


def test_can_transport_does_not_shadow_success_when_init_ack_fails():
    class AckFailingDriver:
        def __init__(self):
            self.calls = []

        def pipette_initialize(self, pressure_profile="1R"):
            self.calls.append(("init", pressure_profile))
            return {
                "ok": False,
                "error": "ack_timeout",
                "ack": {"received": False},
                "ascii_command": "WR",
            }

        def query_tip_status(self):
            return {"ok": True, "tip_loaded": False, "hardware_truth_level": "hardware_query"}

        def query_pressure(self):
            return {"ok": True, "pressure": 1.5, "hardware_truth_level": "hardware_query"}

    driver = AckFailingDriver()
    transport = CanPipetteTransport(driver_factory=lambda: driver)

    with pytest.raises(Exception) as exc_info:
        transport.initialize(PipetteInitCommand(pressure_profile="1R"))

    assert "ack_timeout" in str(exc_info.value)
    status = transport.get_status()
    assert status["software_initialized"] is False
    assert status["initialized"] is False
    assert status["hardware_tip_status"]["tip_loaded"] is False
    assert status["hardware_pressure"]["pressure"] == 1.5


def test_liquid_route_attaches_reference_preflight_and_location_context(monkeypatch):
    api = load_api(monkeypatch)
    calls = []

    class ReferenceStore:
        def snapshot(self, axes):
            axis_keys = [axis.value if hasattr(axis, "value") else str(axis) for axis in axes]
            return {
                "axes": axis_keys,
                "rows": {
                    axis: {"axis": axis, "state": "referenced", "origin_position_steps": 0}
                    for axis in axis_keys
                },
            }

    async def fake_aspirate(command, **kwargs):
        preflight = kwargs["preflight"]("aspirate", command)
        calls.append((command, preflight))
        return {"ok": True, **command.to_payload(), "preflight": preflight}

    monkeypatch.setattr(api, "_reference_state_store", ReferenceStore())
    monkeypatch.setattr(api, "run_pipette_aspirate_command", fake_aspirate)

    result = asyncio.run(
        api.liquid_aspirate(
            api.PipetteAspirateRequest(
                volume_ul=20.0,
                pressure_profile="1R",
                source={"location_id": "rxn_plate", "well_id": "c3"},
                liquid_class="aqueous",
                operator="Christian",
            )
        )
    )

    assert result["source"] == {"location_id": "rxn_plate", "well_id": "C3", "plate_name": None, "z_offset_steps": None}
    assert result["liquid_class"] == "aqueous"
    assert result["operator"] == "Christian"
    assert result["preflight"]["ok"] is True
    assert result["preflight"]["required_reference_axes"] == ["x", "y", "z"]
    assert calls[0][0].source.location_id == "rxn_plate"
