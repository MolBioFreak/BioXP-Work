from __future__ import annotations

import copy

from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider


class MemoryStateStore:
    def __init__(self, state: dict):
        self.state = copy.deepcopy(state)

    def read_oem_serial206_initialization_state(self):
        return copy.deepcopy(self.state)

    def write_oem_serial206_initialization_state(self, state):
        self.state = copy.deepcopy(state)
        return copy.deepcopy(state)


def test_default_initialize_motion_projection_omits_full_stage_evidence():
    provider = Serial206OemInitializationProvider(object())
    state = provider._new_state()
    state["initialize_motion_ledger"]["stage_receipts"] = [
        {
            "stage": "initializeMotion.initializeMotors",
            "command_id": "initialize-1",
            "status": "completed",
            "controller_acknowledged": True,
            "receipt": {"payload": "x" * 50_000},
            "raw_result": {"trace": list(range(1000))},
        }
    ]
    first_stage = state["movement_ledger"]["stage_order"][0]
    state["movement_ledger"]["stages"][first_stage].update(
        {
            "status": "completed",
            "command_id": "movement-1",
            "receipt": {"payload": "y" * 50_000},
            "raw_result": {"trace": list(range(1000))},
        }
    )
    provider = Serial206OemInitializationProvider(
        object(), state_store=MemoryStateStore(state)
    )

    projection = provider.initialize_motion_projection()

    initialize_ledger = projection["initialize_motion_ledger"]
    assert initialize_ledger["stage_receipt_count"] == 1
    assert initialize_ledger["latest_stage_receipt"] == {
        "stage": "initializeMotion.initializeMotors",
        "status": "completed",
        "command_id": "initialize-1",
    }
    movement_row = projection["initialize_motors"]["stages"][first_stage]
    assert movement_row["status"] == "completed"
    assert movement_row["command_id"] == "movement-1"
    assert "receipt" not in movement_row
    assert "raw_result" not in movement_row
    assert "payload" not in repr(projection)


def test_x_and_z_default_projections_keep_only_compact_current_authority():
    provider = Serial206OemInitializationProvider(object())
    state = provider._new_state()
    state["x_lifecycle"]["receipts"] = [
        {"command_id": "x-old", "status": "completed", "result": {"payload": "x" * 20_000}}
    ]
    state["z_lifecycle"]["receipts"] = [
        {"command_id": "z-old", "status": "completed", "result": {"payload": "z" * 20_000}}
    ]
    provider = Serial206OemInitializationProvider(
        object(), state_store=MemoryStateStore(state)
    )

    x_projection = provider.x_projection()
    z_projection = provider.z_projection()

    assert "receipts" not in x_projection["lifecycle"]
    assert "receipts" not in z_projection
    assert "payload" not in repr(x_projection)
    assert "payload" not in repr(z_projection)
