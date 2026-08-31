from __future__ import annotations

import pytest

from bioxp import oem_deck_movement


def test_finite_plate_park_waste_families_are_success_only() -> None:
    for operation in ("catch_plate", "release_plate", "press_plate", "park_gantry", "waste_sequence"):
        plan = oem_deck_movement.compile_finite_plate_operation(operation, source_leaf_available=True)
        assert plan["state_update"] == "success_only"


def test_absent_controller_leaf_fails_closed_explicitly() -> None:
    with pytest.raises(RuntimeError, match="source_authority_missing:press_plate"):
        oem_deck_movement.compile_finite_plate_operation("press_plate", source_leaf_available=False)
