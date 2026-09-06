from __future__ import annotations

import pytest

from bioxp import oem_deck_movement


def test_mov_execution_translates_plate_station_and_well() -> None:
    plan = oem_deck_movement.compile_mov_execution(current_plate_location=23, destination=23, well="H12", plate_name="POOL_PLATE")
    assert plan["destination"] == 2
    assert plan["well_id"] == 95
    assert plan["source_operation"] == "movExecution->scriptmoveTo"


def test_mov_execution_fails_closed_without_plate_location_authority() -> None:
    with pytest.raises(ValueError, match="source_authority_missing:plate_location"):
        oem_deck_movement.compile_mov_execution(current_plate_location=-1, destination=23, well=0, plate_name="POOL_PLATE")
