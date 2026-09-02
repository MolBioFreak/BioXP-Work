from __future__ import annotations

import pytest

from bioxp.oem_compat import position_table

PositionTable = position_table.PositionTable


def test_source_normalization_tecan_low_high_adjustments_and_digest() -> None:
    table = PositionTable.from_rows([
        {"name": "TECANRACK1", "x": 1, "y": 2, "zLow": 60000, "zHigh": 59000, "zDelta": 1000, "inc_factor": 1},
        {"name": "LOC_OC", "x": 3, "y": 4, "zLow": 4000, "zHigh": 3000, "zDelta": 1000, "inc_factor": 0},
    ], source="fixture", source_sha256="a" * 64)

    tecan = table.resolve(location_id="TECANRACK1")
    low = table.resolve(location_id="LOC_OC")
    assert (tecan.z_delta, tecan.z_high) == (53000, 7000)
    assert low.z_high == 0
    assert table.adjustment_ledger
    assert len(table.digest) == 64
    assert table.to_snapshot()["position_table_sha256"] == table.digest


def test_wp8_provider_rejects_absent_target_before_handler_call(monkeypatch) -> None:
    from bioxp import oem_serial206_initialization
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

    table = PositionTable.from_rows([
        {"location_id": "LOC_MS", "x": 1, "y": 2, "zLow": 60000, "zDelta": 10000},
    ])
    monkeypatch.setattr(
        oem_serial206_initialization, "load_bound_oem_position_table", lambda: table,
    )
    provider = object.__new__(Serial206OemInitializationProvider)
    calls = []
    provider.wp8_scriptmove_to = lambda *args, **kwargs: calls.append((args, kwargs))
    child = {
        "order": 0, "operation": "scriptmoveTo",
        "arguments": {"destination": 32},
    }
    with pytest.raises(
        RuntimeError,
        match="machine_target_absent_from_serial206_position_table:32",
    ):
        provider.execute_wp8_child(
            child, command_id="absent-32", child_order=0, plan_digest="digest",
        )
    assert calls == []


def test_duplicate_and_unknown_rows_fail_closed() -> None:
    row = {"name": "LOC_OC", "x": 1, "y": 2, "zLow": 60000, "zDelta": 1000, "inc_factor": 0}
    with pytest.raises(ValueError, match="duplicate"):
        PositionTable.from_rows([row, row])
    with pytest.raises(ValueError, match="unknown"):
        PositionTable.from_rows([{**row, "name": "LOC_NOT_REAL"}])


def test_well_conversion_round_trips_and_rejects_96() -> None:
    assert position_table.well_id_from_label("A1") == 0
    assert position_table.well_id_from_label("H12") == 95
    assert position_table.well_label_from_id(0) == "A1"
    assert position_table.well_label_from_id(95) == "H12"
    with pytest.raises(ValueError):
        position_table.well_id_from_label(96)


def test_source_coordinate_repairs_apply_in_exact_loader_order() -> None:
    rows = [
        {"name": "LOC_P_OC", "x": 10, "y": 21, "zLow": 60000, "zDelta": 10000},
        {"name": "LOC_P_OC_PRESS", "x": 0, "y": 0, "zLow": 60000, "zDelta": 10000},
        {"name": "LOC_P_TC", "x": 23, "y": 24, "zLow": 60000, "zDelta": 10000},
        {"name": "LOC_P_TC_PRESS", "x": 0, "y": 0, "zLow": 60000, "zDelta": 10000},
        {"name": "LOC_P_MS", "x": 25, "y": 26, "zLow": 60000, "zDelta": 10000},
        {"name": "LOC_P_MS_PRESS", "x": 0, "y": 0, "zLow": 60000, "zDelta": 10000},
    ]
    table = PositionTable.from_rows(rows, source_sha256="3" * 64)
    assert table.resolve(location_id="LOC_P_OC").base_coordinates["x"] == 25
    assert table.resolve(location_id="LOC_P_MS_PRESS").base_coordinates == {"x": 25, "y": 26, "z": 50000}
    assert table.resolve(location_id="LOC_P_OC_PRESS").base_coordinates == {"x": 25, "y": 21, "z": 50000}
    assert table.resolve(location_id="LOC_P_TC_PRESS").base_coordinates == {"x": 23, "y": 24, "z": 50000}
    assert [row["kind"] for row in table.adjustment_ledger[-4:]] == [
        "align_output_plate_x", "repair_ms_press_xy", "repair_oc_press_xy", "repair_tc_press_xy"
    ]
