from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.oem_runtime_types import OEMRuntimeSnapshot


def test_runtime_store_writes_state_and_journals_with_sequence(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    state = store.write_state(OEMRuntimeSnapshot())
    event = store.append_event({"event_type": "door"})
    hist = store.append_command_history({"command": {"name": "initializeSystem"}})
    assert state["sequence"] < event["sequence"] < hist["sequence"]
    assert store.read_state()["runtime_state"] == "idle_not_ready"
    assert store.read_journal("event_journal.jsonl")[0]["event_type"] == "door"


def test_runtime_recovery_flags_active_command(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    payload = OEMRuntimeSnapshot().to_dict()
    payload["worker"]["state"] = "running"
    payload["worker"]["active_command"] = {"name": "initializeSystem"}
    store.write_state(payload)
    recovered = store.recover_state()
    assert recovered["recovery_required"] is True
    assert recovered["recovery"] == "active_command"
