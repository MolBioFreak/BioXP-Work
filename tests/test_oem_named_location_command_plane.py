from __future__ import annotations

from dataclasses import replace

import pytest

from bioxp.oem_deck_catalog import DeckCatalog, configured_location_names
from bioxp.oem_deck_movement import DeckAuthoritySnapshot, NamedLocationIntent, compile_named_location
from bioxp.oem_compat.position_table import PositionTable


def _table() -> PositionTable:
    return PositionTable.from_rows([
        {"name": name, "x": i * 100, "y": i * 200, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for i, name in enumerate(configured_location_names())
    ])


def _authority(table: PositionTable) -> DeckAuthoritySnapshot:
    return DeckAuthoritySnapshot(
        ownership_generation=4, provider_owner_id="owner", board_epoch_4=8, board_epoch_5=9,
        position_table_sha256=table.digest, machine_state_revision=2,
        reference_versions={"x": 1, "y": 1, "z": 1, "g": 1}, safety_epochs={"global": 0, "x": 0, "y": 0, "z": 0},
        latch_observation_id="latch-1", controller_position_observation_id="pos-1", captured_at=1.0,
        current_x=0, current_y=0, current_z=65000, current_location_id="LOC_MS", current_well_id=0,
        tip_loaded=False, tip_dirty=False, tip_location=-1, clean_path=True, plate_on_gantry=None,
        pseudo_z_home=65000, device_type="BIOXP", latch_status=True, machine_latch_closed=True,
    )


def test_named_move_compiles_force_home_before_both_latches_and_success_transition() -> None:
    table = _table(); authority = _authority(table)
    plan = compile_named_location(NamedLocationIntent(target="LOC_OC"), DeckCatalog.from_position_table(table), table, authority)
    assert [step.operation for step in plan.steps[:3]] == ["ForceToHighHome", "check_latch_status", "check_machine_latch_closed"]
    assert plan.source_branch == "ordinary"
    assert plan.semantic_transition["current_location_id"] == 1
    assert plan.semantic_transition["current_well_id"] == 0
    assert plan.semantic_transition["ownership_generation"] == authority.ownership_generation
    assert plan.semantic_transition["board_epoch_4"] == authority.board_epoch_4
    assert plan.semantic_transition["board_epoch_5"] == authority.board_epoch_5
    assert plan.semantic_transition["authority_snapshot_digest"] == authority.digest
    assert len(plan.plan_digest) == 64 and plan.authority_digest == authority.digest


def test_barcode_park_and_contradictory_input_are_finite() -> None:
    table = _table(); catalog = DeckCatalog.from_position_table(table); authority = _authority(table)
    assert compile_named_location(NamedLocationIntent(target="LOC_TC_BARCODE"), catalog, table, authority).source_branch == "barcode"
    assert compile_named_location(NamedLocationIntent(target="LOC_PARK"), catalog, table, authority).source_branch == "park"
    with pytest.raises(ValueError, match="contradictory_input"):
        compile_named_location(NamedLocationIntent(target="LOC_TC_BARCODE", camera_offset=True), catalog, table, authority)


def test_unsafe_latch_still_compiles_force_home_then_blocks_physical_stages() -> None:
    table = _table(); authority = replace(_authority(table), latch_status=False)
    plan = compile_named_location(NamedLocationIntent(target="LOC_OC"), DeckCatalog.from_position_table(table), table, authority)
    assert [step.operation for step in plan.steps] == ["ForceToHighHome", "check_latch_status", "check_machine_latch_closed"]
    assert plan.blocked_reason == "latch_not_closed"
