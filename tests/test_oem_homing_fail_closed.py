from collections import deque

import pytest

from src.bioxp.usb_driver import BioXpTester


class _HomingRig:
    def __init__(
        self,
        *,
        home_values=(0, 0, 0, 1, 1),
        speed_values=(100, 100),
        position_values=(100, 100, 110, 120, 120, 0),
        move_ok=True,
        stop_ok=True,
        wait_stopped=True,
        wait_last_speed=0,
        sethome_ok=True,
        sethome_readback=0,
    ):
        self.home_values = deque(home_values)
        self.speed_values = deque(speed_values)
        self.position_values = deque(position_values)
        self.last_home = home_values[-1]
        self.last_speed = speed_values[-1]
        self.last_position = position_values[-1]
        self.move_ok = move_ok
        self.stop_ok = stop_ok
        self.wait_stopped = wait_stopped
        self.wait_last_speed = wait_last_speed
        self.sethome_ok = sethome_ok
        self.sethome_readback = sethome_readback
        self.calls = []

    @staticmethod
    def _next(values, fallback):
        return values.popleft() if values else fallback

    def position(self, board, motor=0):
        value = self._next(self.position_values, self.last_position)
        self.last_position = value
        return {"ok": True, "board": board, "motor": motor, "position": value}

    def home(self, board, motor=0):
        value = self._next(self.home_values, self.last_home)
        self.last_home = value
        return {"ok": True, "board": board, "motor": motor, "value": value}

    def speed(self, board, motor=0):
        value = self._next(self.speed_values, self.last_speed)
        self.last_speed = value
        return {"ok": True, "board": board, "motor": motor, "speed": value}


def _run(monkeypatch, rig, *, require_switch_transition=True):
    tester = BioXpTester.__new__(BioXpTester)
    tester.MOTOR_SWITCH_ACTIVE_VALUE = 1
    monkeypatch.setattr(
        tester,
        "_motion_oem_axis_profile",
        lambda axis: {
            "board": tester.BOARD_DECK,
            "motor": 0,
            "axis_min_steps": 0,
            "axis_max_steps": 1000,
        },
    )
    monkeypatch.setattr(tester, "motor_get_position", rig.position)
    monkeypatch.setattr(tester, "motor_query_home_switch", rig.home)
    monkeypatch.setattr(
        tester,
        "motor_get_switch_activity",
        lambda board, motor=0: {"left_state": 0, "right_state": 0},
    )
    monkeypatch.setattr(
        tester,
        "motor_get_switches",
        lambda board, motor=0: {"left_state": rig.last_home, "right_state": 0},
    )
    monkeypatch.setattr(tester, "motor_get_speed", rig.speed)

    def move_left(board, speed=250, motor=0):
        rig.calls.append("move_left")
        return {
            "ok": rig.move_ok,
            "ack": {"status": 100 if rig.move_ok else 4},
            "board": board,
            "motor": motor,
        }

    def stop(board, motor=0):
        rig.calls.append("stop")
        return {
            "ok": rig.stop_ok,
            "ack": {"status": 100 if rig.stop_ok else 4},
            "board": board,
            "motor": motor,
        }

    def wait_stopped(board, motor=0, **kwargs):
        rig.calls.append("wait_stopped")
        return {
            "stopped": rig.wait_stopped,
            "last_speed": rig.wait_last_speed,
            "seen_nonzero": True,
        }

    def set_home(board, motor=0):
        rig.calls.append("set_home")
        return {
            "ok": rig.sethome_ok,
            "ack": {"status": 100 if rig.sethome_ok else 4},
            "readback": {"value": rig.sethome_readback},
            "board": board,
            "motor": motor,
        }

    monkeypatch.setattr(tester, "motor_move_left", move_left)
    monkeypatch.setattr(tester, "motor_stop", stop)
    monkeypatch.setattr(tester, "motor_wait_stopped", wait_stopped)
    monkeypatch.setattr(tester, "motor_set_home", set_home)
    monkeypatch.setattr("src.bioxp.usb_driver.time.sleep", lambda seconds: None)

    return tester.motor_oem_go_home(
        "x",
        speed=250,
        rehome=False,
        timeout_s=0.2,
        require_switch_transition=require_switch_transition,
        max_search_abs_delta=500,
    )


def test_moving_home_requires_complete_evidence_and_zero_readback(monkeypatch):
    result = _run(monkeypatch, _HomingRig())

    assert result["ok"] is True
    assert result["seen_motion"] is True
    assert result["switch_transition"] is True
    assert result["stop"]["ok"] is True
    assert result["wait"]["stopped"] is True
    assert result["wait"]["last_speed"] == 0
    assert result["set_home"]["readback"]["value"] == 0
    assert result["position_after_sethome"]["position"] == 0


def test_stale_active_switch_and_failed_move_ack_cannot_publish_home(monkeypatch):
    rig = _HomingRig(
        home_values=(1, 1, 1, 1),
        speed_values=(0,),
        position_values=(100, 100, 100, 100, 100),
        move_ok=False,
    )

    result = _run(monkeypatch, rig)

    assert result["ok"] is False
    assert "set_home" not in rig.calls
    assert result["false_home_guard"] in {
        "home_switch_active_before_search",
        "move_home_ack_failed",
        "home_switch_transition_not_observed",
    }


@pytest.mark.parametrize(
    ("rig", "expected_failure"),
    [
        (
            _HomingRig(
                home_values=(0, 0, 1, 1),
                speed_values=(0,),
                position_values=(100, 100, 100, 100, 0),
            ),
            "motion_not_observed",
        ),
        (_HomingRig(stop_ok=False), "stop_ack_failed"),
        (_HomingRig(wait_stopped=False), "motor_not_stopped"),
        (_HomingRig(wait_last_speed=5), "post_stop_speed_nonzero_or_unknown"),
        (_HomingRig(sethome_ok=False), "set_home_failed"),
        (_HomingRig(sethome_readback=7), "set_home_readback_not_zero"),
        (
            _HomingRig(position_values=(100, 100, 110, 120, 120, 9)),
            "position_after_sethome_not_zero",
        ),
    ],
)
def test_moving_home_rejects_each_missing_authoritative_evidence(monkeypatch, rig, expected_failure):
    result = _run(monkeypatch, rig)

    assert result["ok"] is False
    assert result["false_home_guard"] == expected_failure
