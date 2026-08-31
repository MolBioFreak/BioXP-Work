from __future__ import annotations

import pytest

from bioxp.oem_deck_catalog import DeckCatalog
from bioxp.oem_compat.position_table import PositionTable


def _table() -> PositionTable:
    from bioxp.oem_deck_catalog import configured_location_names
    return PositionTable.from_rows([
        {"name": name, "x": i, "y": i + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for i, name in enumerate(configured_location_names())
    ])


def test_finite_catalog_has_26_labels_24_destinations_and_no_internal_points() -> None:
    catalog = DeckCatalog.from_position_table(_table())
    rows = catalog.rows()
    assert len(rows) == 26
    assert len({row["location_id"] for row in rows}) == 24
    assert {row["branch"] for row in rows} == {"ordinary", "barcode", "park"}
    public = {row["target"] for row in rows}
    assert not public & {"LOC_P_OC_PRESS", "LOC_P_TC_PRESS", "LOC_P_MS_PRESS", "LOC_P_RC_PRESS", "CAMERA_OFFSET"}
    assert catalog.resolve_alias("OC chiller").target == "LOC_OC"
    assert all(row["position_table_sha256"] == _table().digest for row in DeckCatalog.from_position_table(_table()).rows())


def test_catalog_rejects_absent_machine_destination_and_unknown_alias() -> None:
    table = PositionTable.from_rows([{"name": "LOC_OC", "x": 1, "y": 2, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}])
    with pytest.raises(ValueError, match="absent"):
        DeckCatalog.from_position_table(table)
    with pytest.raises(KeyError):
        DeckCatalog.from_position_table(_table()).resolve_alias("nearest thing")
