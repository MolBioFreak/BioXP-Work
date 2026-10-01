"""Pipettes are constructed when CAN becomes ready, as the OEM app does at start."""
from bioxp import api


class _SyncThread:
    def __init__(self, target, name=None, daemon=None):
        self.target = target

    def start(self):
        self.target()


def test_constructor_runs_through_the_existing_owner_path(monkeypatch):
    calls = []
    monkeypatch.setattr(api.threading, "Thread", _SyncThread)
    monkeypatch.setattr(api, "_pipette_collection_state", lambda **kw: calls.append(kw))
    api._start_oem_constructor_pipettes("test")
    assert calls == [{"ensure_constructor": True}]


def test_constructor_failure_is_logged_not_raised(monkeypatch):
    monkeypatch.setattr(api.threading, "Thread", _SyncThread)

    def fail(**kw):
        raise RuntimeError("no pipettes")
    monkeypatch.setattr(api, "_pipette_collection_state", fail)
    api._start_oem_constructor_pipettes("test")
