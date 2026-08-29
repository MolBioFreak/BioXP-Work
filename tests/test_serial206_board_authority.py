from src.bioxp.oem_runtime_store import OEMRuntimeStore


def test_board4_transition_records_epoch_and_invalidates_member_axes(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    first = store.record_board4_transition(
        active=True,
        ack={"status": 100, "raw": "activate"},
        transition_id="t-1",
        ownership_generation=7,
        invalidate_axes=False,
    )
    assert first["board"]["state"] == "active"
    epoch = first["board"]["active_board_epoch"]
    assert epoch == 1

    assert store.prepare_axis_authority(
        "y", ownership_generation=7, profile_fingerprint="y-profile"
    )["ok"] is True
    assert store.prepare_axis_authority(
        "z", ownership_generation=7, profile_fingerprint="z-profile"
    )["ok"] is True
    assert store.publish_axis_reference("y", position_steps=0, ownership_generation=7)["ok"] is True

    second = store.record_board4_transition(
        active=False,
        ack={"status": 100, "raw": "deactivate"},
        transition_id="t-2",
        ownership_generation=7,
        invalidate_axes=True,
    )
    assert second["board"]["state"] == "inactive"
    assert second["board"]["prior_board_epoch"] == epoch
    assert second["board"]["active_board_epoch"] is None
    axes = second["axes"]
    assert axes["y"]["lifecycle_state"] == "generation_stale"
    assert axes["y"]["reference_state"] == "generation_stale"
    assert axes["z"]["lifecycle_state"] == "generation_stale"
    assert axes["gripper"]["lifecycle_state"] == "unprepared"
    store.close()


def test_board4_activation_creates_new_epoch_without_clearing_unprepared_axes(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    first = store.record_board4_transition(
        active=True,
        ack={"status": 100},
        transition_id="t-1",
        ownership_generation=2,
        invalidate_axes=False,
    )
    store.record_board4_transition(
        active=False,
        ack={"status": 100},
        transition_id="t-2",
        ownership_generation=2,
        invalidate_axes=False,
    )
    second = store.record_board4_transition(
        active=True,
        ack={"status": 100},
        transition_id="t-3",
        ownership_generation=3,
        invalidate_axes=True,
    )
    assert second["board"]["active_board_epoch"] == first["board"]["active_board_epoch"] + 1
    assert second["axes"]["y"]["lifecycle_state"] == "unprepared"
    assert second["axes"]["y"]["reference_state"] == "unreferenced"
    store.close()


def test_axis_observation_records_discrepancy_without_desynchronizing(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    store.record_board4_transition(
        active=True,
        ack={"status": 100},
        transition_id="t-1",
        ownership_generation=1,
        invalidate_axes=False,
    )
    store.prepare_axis_authority("y", ownership_generation=1, profile_fingerprint="p")
    store.publish_axis_reference("y", position_steps=100, ownership_generation=1)
    observed = store.record_axis_observation(
        "y", requested_position_steps=120, observed_position_steps=117, receipt_id="r-1"
    )
    assert observed["ok"] is True
    assert observed["discrepancy_steps"] == -3
    assert observed["axis"]["reference_state"] == "referenced"
    assert observed["axis"]["lifecycle_state"] == "referenced_ready"
    store.close()
