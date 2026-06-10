import pytest

from src.bioxp.oem_runtime_types import OEMCommandName, OEMRuntimeCommand


def test_oem_command_vocabulary_is_exact():
    assert [c.value for c in OEMCommandName] == [
        "initializeSystem",
        "unlockProcess",
        "PrepareToRunJob",
        "abortjob",
        "validateJob",
        "wakefrompause",
    ]


def test_unknown_command_rejects_before_queue():
    with pytest.raises(ValueError):
        OEMRuntimeCommand(name="homeEverything")


def test_live_command_requires_ack_and_artifact_root():
    with pytest.raises(ValueError):
        OEMRuntimeCommand(name="initializeSystem", mode="live", artifact_root="/tmp/x")
    with pytest.raises(ValueError):
        OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE")
    cmd = OEMRuntimeCommand(name="initializeSystem", mode="live", operator_ack="INITIALIZE", artifact_root="/tmp/bioxp-live-runs/test")
    assert cmd.name == "initializeSystem"
