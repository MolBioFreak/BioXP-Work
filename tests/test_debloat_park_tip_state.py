"""Park consumes the OEM TipExist value instead of refusing an unknown mix."""
import pytest

from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider


def _provider(reader):
    p = object.__new__(Provider)
    p.bind_pipette_collection_state_reader(reader)
    return p


def test_unknown_channels_read_as_oem_constructor_false_with_evidence():
    p = _provider(lambda: {"tip_exists": None, "hardware_tip_exists": None, "command_id": "q"})
    state = p._park_collection_state()
    assert state["tip_exists"] is False
    assert state["tip_exists_unknown_channels"] is True
    assert state["hardware_tip_exists"] is None and state["command_id"] == "q"
    # Deterministic, so the snapshot/dispatch equality fences still hold.
    assert p._park_collection_state() == state


@pytest.mark.parametrize("value", [True, False])
def test_known_tip_state_is_unchanged(value):
    p = _provider(lambda: {"tip_exists": value, "hardware_tip_exists": value})
    assert p._park_collection_state() == {"tip_exists": value, "hardware_tip_exists": value}


def test_positive_channel_still_takes_cleanup_branch_value():
    p = _provider(lambda: {"tip_exists": True, "hardware_tip_exists": None})
    assert p._park_collection_state()["tip_exists"] is True


def test_unbound_or_malformed_reader_is_still_a_contract_error():
    p = object.__new__(Provider)
    with pytest.raises(RuntimeError, match="pipette_collection_owner_not_bound"):
        p._park_collection_state()
    with pytest.raises(RuntimeError, match="pipette_collection_state_not_authoritative"):
        _provider(lambda: None)._park_collection_state()
