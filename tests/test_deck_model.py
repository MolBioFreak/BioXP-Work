import importlib
import sys

import pytest


def load_domain_module():
    for name in ["src.bioxp.domain", "src.bioxp.usb_driver", "src.bioxp"]:
        sys.modules.pop(name, None)

    return importlib.import_module("src.bioxp.domain")


DOMAIN = load_domain_module()

CapabilityName = DOMAIN.CapabilityName
Coordinate3D = DOMAIN.Coordinate3D
DEFAULT_DECK_LAYOUT_PATH = DOMAIN.DEFAULT_DECK_LAYOUT_PATH
MissingCapabilityError = DOMAIN.MissingCapabilityError
UnknownLocationError = DOMAIN.UnknownLocationError
UnknownWellError = DOMAIN.UnknownWellError
DeckLayout = DOMAIN.DeckLayout
load_deck_layout = DOMAIN.load_deck_layout
load_layout_data = DOMAIN.load_layout_data


def test_default_layout_parses_slots_locations_and_capabilities():
    payload = load_layout_data(DEFAULT_DECK_LAYOUT_PATH)
    layout = load_deck_layout(DEFAULT_DECK_LAYOUT_PATH)

    assert payload["deck_id"] == "bioxp-default"
    assert layout.deck_id == "bioxp-default"
    assert layout.version == 1
    assert set(layout.slots) == {"reagent_plate", "mag_plate", "waste_station"}
    assert set(layout.locations) == {
        "reagent_rack",
        "reagent_pickup",
        "magnetic_station",
        "waste_drop",
    }
    assert layout.capabilities.is_enabled(CapabilityName.MOTION) is True
    assert layout.capabilities.is_enabled("pipette") is True
    assert layout.capabilities.is_enabled(CapabilityName.BARCODE) is False


def test_resolve_semantic_location_aliases_to_slot_space():
    layout = load_deck_layout()

    reagent_rack = layout.resolve("reagents")
    reagent_pickup = layout.resolve("reagent")
    waste = layout.resolve("waste")

    assert reagent_rack.location_id == "reagent_rack"
    assert reagent_rack.slot_id == "reagent_plate"
    assert reagent_rack.coordinate == Coordinate3D(120.0, 40.0, 5.0)

    assert reagent_pickup.location_id == "reagent_pickup"
    assert reagent_pickup.coordinate == Coordinate3D(123.0, 44.0, 17.5)

    assert waste.location_id == "waste_drop"
    assert waste.coordinate == Coordinate3D(415.0, 105.0, 40.0)


def test_resolve_well_address_from_semantic_location_reference():
    layout = load_deck_layout()

    reagent_b2 = layout.resolve("reagent_rack:B2")
    magnet_a2 = layout.resolve("magnetic_station", well_id="A2")

    assert reagent_b2.location_id == "reagent_rack"
    assert reagent_b2.address == "reagent_rack:B2"
    assert reagent_b2.labware_id == "plate_96_generic"
    assert reagent_b2.well_id == "B2"
    assert reagent_b2.coordinate == Coordinate3D(129.0, 49.0, 5.0)

    assert magnet_a2.location_id == "magnetic_station"
    assert magnet_a2.address == "magnetic_station:A2"
    assert magnet_a2.coordinate == Coordinate3D(269.0, 40.0, 5.0)


def test_resolve_physical_slot_without_alias_collision():
    layout = load_deck_layout()

    reagent_slot = layout.resolve("reagent_plate")
    magnetic_slot = layout.resolve("mag_plate")

    assert reagent_slot.location_id == "reagent_plate"
    assert reagent_slot.slot_id == "reagent_plate"
    assert reagent_slot.coordinate == Coordinate3D(120.0, 40.0, 5.0)

    assert magnetic_slot.location_id == "mag_plate"
    assert magnetic_slot.slot_id == "mag_plate"
    assert magnetic_slot.coordinate == Coordinate3D(260.0, 40.0, 5.0)


def test_location_resolution_rejects_unknown_locations_and_wells():
    layout = load_deck_layout()

    with pytest.raises(UnknownLocationError):
        layout.resolve("does_not_exist")

    with pytest.raises(UnknownWellError):
        layout.resolve("waste", well_id="A1")

    with pytest.raises(UnknownWellError):
        layout.resolve("reagent_rack:Z99")


def test_capability_registry_require_reports_disabled_capabilities():
    layout = load_deck_layout()

    motion = layout.capabilities.require("motion")
    assert motion.name is CapabilityName.MOTION
    assert motion.enabled is True

    with pytest.raises(MissingCapabilityError) as exc_info:
        layout.capabilities.require("barcode")

    assert "barcode" in str(exc_info.value)
    assert "scanner_not_integrated" in str(exc_info.value)


def test_rejects_ambiguous_slot_and_location_references():
    with pytest.raises(ValueError, match="Deck references must be unique"):
        DeckLayout.from_mapping(
            {
                "deck_id": "ambiguous",
                "version": 1,
                "slots": [
                    {
                        "slot_id": "plate_a",
                        "origin": {"x": 0, "y": 0, "z": 0},
                    }
                ],
                "locations": [
                    {
                        "location_id": "reagent_rack",
                        "slot": "plate_a",
                        "aliases": ["plate_a"],
                    }
                ],
            }
        )
