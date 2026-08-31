from __future__ import annotations

import dataclasses
import hashlib
import json
from pathlib import Path

from bioxp.oem_deck_catalog import DeckCatalog, configured_location_names
from bioxp.oem_deck_movement import DeckAuthoritySnapshot, NamedLocationIntent, compile_named_location
from bioxp.oem_compat.position_table import PositionTable


REPO = Path(__file__).resolve().parents[1]
REGISTRY_JSON = REPO / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json"
REGISTRY_MD = REPO / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.md"


def _table() -> PositionTable:
    rows = [
        {"name": name, "x": index * 100, "y": index * 100 + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ]
    rows.append({"name": "CAMERA_OFFSET", "x": 3499, "y": -7744, "zLow": 3145, "zDelta": 6842, "inc_factor": 0})
    return PositionTable.from_rows(rows, source_sha256="3" * 64)


def _authority(table: PositionTable) -> DeckAuthoritySnapshot:
    return DeckAuthoritySnapshot(
        ownership_generation=4, provider_owner_id="owner", board_epoch_4=8, board_epoch_5=9,
        position_table_sha256=table.digest, machine_state_revision=2,
        reference_versions={"x": 1, "y": 1, "z": 1, "g": 1},
        safety_epochs={"global": 0, "x": 0, "y": 0, "z": 0},
        latch_observation_id="latch", controller_position_observation_id="position", captured_at=1.0,
        current_x=0, current_y=0, current_z=65000, current_location_id="LOC_MS", current_well_id=0,
        tip_loaded=False, tip_dirty=False, tip_location=-1, clean_path=True, plate_on_gantry=None,
        pseudo_z_home=65000, device_type="BIOXP", latch_status=True, machine_latch_closed=True,
    )


def test_registry_hashes_named_deck_methods_and_finite_source_denominator_are_current() -> None:
    registry = json.loads(REGISTRY_JSON.read_text(encoding="utf-8"))
    required_methods = {
        "ClassControlInterface.btnLOC1_Click@1770",
        "ClassControlInterface.moveTo@3663",
        "ClassControlInterface.moveTo@3691",
        "ClassControlInterface.scriptmoveTo@3718",
        "ClassControlInterface.scriptmoveTo@3734",
        "ClassControlInterface.moveTo@4463",
        "ClassControlInterface.getMidPoint@5254",
        "ControlLib.movExecution@6706",
        "ControlLib.parkGantry@7071",
        "ControlLib.pressPlates@5154",
        "ControlLib.releasePlate@8116",
        "ControlLib.catchPlate@8262",
        "ClassMachineStatus.updateLocation@494",
        "ClassMachineStatus.updatePlateLocation@691",
        "ClassMachineStatus.LoadGantry@957",
    }
    methods = {row["method_id"]: row for row in registry["methods"]}
    assert required_methods <= methods.keys()
    sources = {row["source_id"]: row for row in registry["sources"]}
    for source_id in {methods[method_id]["source_id"] for method_id in required_methods}:
        source = sources[source_id]
        source_path = Path(source["absolute_path"])
        assert hashlib.sha256(source_path.read_bytes()).hexdigest() == source["sha256"]
        assert methods[next(method_id for method_id in required_methods if methods[method_id]["source_id"] == source_id)]["binary_id"] == source["binary_id"]
    denominator = registry["finite_destination_denominator"]
    assert denominator["panel_label_count"] == 26
    assert denominator["distinct_location_id_count"] == 24
    assert denominator["configured_position_table_row_count"] == 29
    assert {row["canonical_request_key"] for row in denominator["entries"]} == {
        row["target"] for row in DeckCatalog.from_position_table(_table()).rows()
    }
    assert "Canonical named deck action source closure" in REGISTRY_MD.read_text(encoding="utf-8")


def test_registry_records_pinned_raw_il_barcode_and_park_resolution_without_closing_vision() -> None:
    registry = json.loads(REGISTRY_JSON.read_text(encoding="utf-8"))
    resolution = registry["raw_il_resolutions"]["barcode_and_park"]
    assert resolution["binary_id"] == "BioXPControlLib.dll"
    assert resolution["binary_sha256"] == "163db8f7835cecbc87da4d14734a8224d79ea1e2ccc77bbb299998fa31bf14ed"
    assert resolution["method_tokens"] == {
        "btnLOC1_Click": "0x060000CB",
        "moveTo_offset": "0x0600011E",
        "moveZ": "0x0600012E",
        "parkGantry": "0x06000351",
    }
    assert resolution["source_confirmation"]["tc"] == {
        "location_id": 2, "x": "-11847 + CameraXOffset",
        "y": "7582 + CameraYOffset",
        "z": "trunc_i4(-1350.5511600000034 + CameraZOffset)",
        "child_order": ["moveTo", "moveZ"],
    }
    assert resolution["source_confirmation"]["rc"] == {
        "location_id": 3, "x": "-23930 + CameraXOffset",
        "y": "7582 + CameraYOffset", "z": "CameraZOffset",
        "child_order": ["moveTo", "moveZ"],
    }
    assert resolution["linux_naming"] == {
        "oem_source_method": "moveZ",
        "linux_provider_method": "moveZCamera",
        "disposition": "provider_name_only_not_oem_source_name",
    }
    hazards = {row["hazard_id"]: row for row in registry["known_hazards"]}
    assert "VISION_INVALID_IL_REGIONS" not in hazards
    assert hazards["BARCODE_PARK_INVALID_IL_RESOLVED_FROM_PINNED_BINARY"]["disposition"] == "resolved_for_barcode_offsets_and_park_only"
    unresolved = hazards["VISION_INVALID_IL_REGIONS_UNRELATED"]
    assert unresolved["disposition"] == "unresolved_outside_governing_barcode_offset_scope"
    assert "7073" not in unresolved["source_ref"]
    assert "vision algorithms remain unresolved" in REGISTRY_MD.read_text(encoding="utf-8")


def test_named_location_plans_have_branch_hazards_bound_into_digest() -> None:
    table = _table()
    catalog = DeckCatalog.from_position_table(table)
    authority = _authority(table)
    plans = {
        branch: compile_named_location(NamedLocationIntent(target=target), catalog, table, authority)
        for branch, target in {
            "ordinary": "LOC_OC", "barcode": "LOC_TC_BARCODE", "park": "LOC_PARK",
        }.items()
    }
    for branch, plan in plans.items():
        assert plan.source_hazards
        assert all(item.startswith(f"{branch}:") for item in plan.source_hazards)
        without_hazards = dataclasses.replace(plan, source_hazards=())
        assert plan.plan_digest != without_hazards.plan_digest


def test_preview_only_ordinal_19_uses_loc_rc_cover_and_remains_zero_io() -> None:
    source = (REPO / "src/bioxp/oem_homing_routes.py").read_text(encoding="utf-8")
    assert 'resolve(location_id="LOC19")' not in source
    assert 'resolve(location_id="LOC_RC_COVER")' in source
    assert "live_enabled = False" in source
