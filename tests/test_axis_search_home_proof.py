"""axisSearchHome carries goHome's controller proof up, as OEM returns goHome's result."""
import pytest

from bioxp.usb_driver import BioXpTester


def _tester(monkeypatch, go_home):
    t = object.__new__(BioXpTester)
    monkeypatch.setattr(t, "_motion_oem_axis_profile", lambda *a, **k: {"board": 5, "motor": 0}, raising=False)
    monkeypatch.setattr(t, "oem_no24v_state", lambda: False, raising=False)
    monkeypatch.setattr(t, "_oem_board_state", lambda: {5: True}, raising=False)
    monkeypatch.setattr(t, "motor_set_home", lambda *a, **k: {"ok": True}, raising=False)
    monkeypatch.setattr(t, "_oem_store_search_speed", lambda *a, **k: None, raising=False)
    monkeypatch.setattr(t, "motor_query_home_switch", lambda *a, **k: {"home": False}, raising=False)
    monkeypatch.setattr(t, "motor_oem_go_home", lambda *a, **k: go_home, raising=False)
    return t


@pytest.mark.parametrize("proof", [True, False])
def test_search_home_propagates_inner_go_home_proof(monkeypatch, proof):
    inner = {
        "ok": True, "source_return_code": 0,
        "controller_command_acknowledged": proof,
        "controller_terminal_state_verified": proof,
        "controller_home_proof_verified": proof,
    }
    out = _tester(monkeypatch, inner).motor_oem_axis_search_home("x", speed=250)
    assert out["go_home"] is inner
    assert out["controller_command_acknowledged"] is proof
    assert out["controller_terminal_state_verified"] is proof
    assert out["controller_home_proof_verified"] is proof


def test_cached_noop_go_home_is_not_promoted_to_proof(monkeypatch):
    inner = {"ok": True, "source_noop": True, "completion_class": "source_cached_noop",
             "controller_home_proof_verified": False}
    out = _tester(monkeypatch, inner).motor_oem_axis_search_home("x", speed=250)
    assert out["controller_home_proof_verified"] is False
