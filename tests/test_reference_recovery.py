"""Unit tests for the guarded 24V power-up reference recovery monitor."""

import json
from pathlib import Path

from bioxp.reference_recovery import ReferenceRecoveryMonitor


def _rail(no24v):
    return {"no24v": no24v, "sample_valid": True, "safety_valid": no24v is False}


class Harness:
    def __init__(self, tmp_path: Path, *, rails, refs=True, idle=(True, None)):
        self.now = [1000.0]
        self.rails = list(rails)
        self.refs = refs
        self.idle = idle
        self.fires = []
        self.state_path = tmp_path / "reference_recovery_state.json"

    def rail_reader(self):
        if len(self.rails) > 1:
            return self.rails.pop(0)
        return self.rails[0] if self.rails else None

    def rereference(self):
        self.fires.append(self.now[0])
        return {"ok": True, "state": "initializeMotors_complete"}

    def monitor(self, **kwargs):
        return ReferenceRecoveryMonitor(
            rail_reader=self.rail_reader,
            references_ready=lambda: self.refs,
            idle_check=lambda: self.idle,
            rereference=self.rereference,
            state_path=self.state_path,
            clock=lambda: self.now[0],
            **kwargs,
        )


def test_explicit_down_then_up_fires_once(tmp_path):
    h = Harness(tmp_path, rails=[_rail(True), _rail(False)], refs=False)
    m = h.monitor()
    m.tick()  # explicit down -> armed
    assert m.status()["armed_down"] is True
    m.tick()  # power-up edge -> references invalid + idle -> one fire
    assert len(h.fires) == 1
    assert m.status()["last_outcome"] == "completed"
    assert m.status()["attempts"] == 1
    # further powered ticks: one attempt per edge, no repeats
    h.now[0] += 10
    m.tick()
    m.tick()
    assert len(h.fires) == 1


def test_no_reply_streak_counts_as_power_down(tmp_path):
    h = Harness(tmp_path, rails=[None, None, None, _rail(False)], refs=False)
    m = h.monitor()
    m.tick()
    m.tick()
    assert m.status()["armed_down"] is False  # single/two hiccups stay unknown
    m.tick()
    assert m.status()["armed_down"] is True  # sustained no-reply = machine off
    m.tick()
    assert len(h.fires) == 1


def test_no_edge_never_fires(tmp_path):
    h = Harness(tmp_path, rails=[_rail(False)], refs=False)
    m = h.monitor()
    for _ in range(5):
        m.tick()
    assert h.fires == []
    assert m.status()["last_edge_at"] is None


def test_references_valid_at_edge_skips(tmp_path):
    h = Harness(tmp_path, rails=[_rail(True), _rail(False)], refs=True)
    m = h.monitor()
    m.tick()
    m.tick()
    assert h.fires == []
    assert m.status()["last_outcome"] == "references_already_valid"


def test_busy_machine_defers_then_fires(tmp_path):
    h = Harness(tmp_path, rails=[_rail(True), _rail(False)], refs=False, idle=(False, "operator_action_pending"))
    m = h.monitor()
    m.tick()
    m.tick()
    assert h.fires == []  # busy: pending retained
    assert m.status()["pending_since"] is not None
    assert m.status()["last_skip_reason"] == "operator_action_pending"
    h.idle = (True, None)
    h.now[0] += 30
    m.tick()
    assert len(h.fires) == 1


def test_busy_beyond_pending_window_abandons(tmp_path):
    h = Harness(tmp_path, rails=[_rail(True), _rail(False)], refs=False, idle=(False, "operation_state:running"))
    m = h.monitor()
    m.tick()
    m.tick()
    h.now[0] += 700
    m.tick()
    assert h.fires == []
    assert m.status()["last_outcome"] == "abandoned_machine_busy"
    assert m.status()["pending_since"] is None
    h.idle = (True, None)
    h.now[0] += 10
    m.tick()
    assert h.fires == []  # no new edge, no late fire


def test_malformed_sample_keeps_state(tmp_path):
    h = Harness(tmp_path, rails=[_rail(True), {"no24v": None}, _rail(False)], refs=False)
    m = h.monitor()
    m.tick()  # down -> armed
    m.tick()  # malformed: no state change, no edge
    assert h.fires == []
    m.tick()  # powered -> edge
    assert len(h.fires) == 1


def test_state_persists_across_restart_without_inventing_edge(tmp_path):
    h = Harness(tmp_path, rails=[_rail(True), _rail(False)], refs=False)
    m = h.monitor()
    m.tick()
    m.tick()
    assert len(h.fires) == 1
    persisted = json.loads(h.state_path.read_text())
    assert persisted["last_powered"] is True
    assert persisted["armed_down"] is False
    # A service restart while the machine stays powered must not invent an edge.
    h2 = Harness(tmp_path, rails=[_rail(False)], refs=False)
    m2 = h2.monitor()
    m2.tick()
    m2.tick()
    assert h2.fires == []


def test_failed_attempt_recorded(tmp_path):
    h = Harness(tmp_path, rails=[_rail(True), _rail(False)], refs=False)

    def boom():
        h.fires.append(h.now[0])
        raise RuntimeError("provider unavailable")

    m = ReferenceRecoveryMonitor(
        rail_reader=h.rail_reader,
        references_ready=lambda: False,
        idle_check=lambda: (True, None),
        rereference=boom,
        state_path=h.state_path,
        clock=lambda: h.now[0],
    )
    m.tick()
    m.tick()
    assert len(h.fires) == 1
    assert m.status()["last_outcome"].startswith("failed:RuntimeError")
