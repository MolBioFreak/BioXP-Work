from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.serial206_y_provider import Serial206YProvider


class FakeYTester:
    def __init__(self):
        self.position = 1000
        self.calls = []

    def _motion_oem_axis_profile(self, axis, *, startup=False):
        assert axis == "y"
        return {
            "board": 4,
            "motor": 0,
            "speed": 1800,
            "acc": 400,
            "home_speed": 250 if startup else 500,
            "run_current": 31,
            "stall_guard": 16,
            "disable_right": True,
            "axis_min_steps": 0,
            "axis_max_steps": 102956,
        }

    def motor_oem_require_no_motion_profile(self, axis):
        self.calls.append(("prepare", axis))
        return {"ok": True, "profile": {"axis": axis}}

    def motor_get_position(self, board, *, motor=0):
        assert (board, motor) == (4, 0)
        return {"ok": True, "position": self.position, "ack": {"status": 100}}

    def motor_y_move_relative_strict(self, steps, *, timeout_s=20.0):
        self.calls.append(("relative", steps, timeout_s))
        self.position += int(steps)
        return {
            "ok": True, "board": 4, "motor": 0, "steps": int(steps),
            "target_position": self.position,
            "terminal_position": {"ok": True, "position": self.position, "ack": {"status": 100}},
            "terminal_speed": {"ok": True, "speed": 0, "ack": {"status": 100}},
            "proof": {"direct_ack": True, "addressed_event_128": True, "target_position": True, "speed_zero": True},
            "completion_class": "event_128",
        }

    def motor_oem_move_absolute(self, board, position, *, motor=0, wait_for_stop=True, max_position=None):
        self.calls.append(("absolute", board, position, wait_for_stop, max_position))
        self.position = int(position)
        return {
            "ok": True, "board": board, "motor": motor, "requested_position": int(position),
            "wire_position": int(position), "command_sent": True,
            "pending_motion": not wait_for_stop, "completion_verified": bool(wait_for_stop),
            "terminal_position": {"ok": True, "position": self.position, "ack": {"status": 100}} if wait_for_stop else None,
            "terminal_speed": {"ok": True, "speed": 0, "ack": {"status": 100}} if wait_for_stop else None,
            "proof": {"direct_ack": True, "addressed_event_128": bool(wait_for_stop), "target_position": bool(wait_for_stop), "speed_zero": bool(wait_for_stop)},
            "completion_class": "event_128" if wait_for_stop else "issued_pending",
        }

    def motor_oem_home_axis(self, axis, *, speed, startup=False, require_switch_transition=True, **kwargs):
        self.calls.append(("home", axis, speed, startup, require_switch_transition))
        return {
            "ok": True, "axis": axis,
            "home": {
                "ok": True,
                "home_after": {"ok": True, "value": 1, "ack": {"status": 100}},
                "stop": {"ok": True, "first_delivery": {"status": 100}, "second_delivery": {"status": 100}},
                "wait": {"stopped": True, "last_speed": 0, "last_ack": {"status": 100}},
                "set_home": {"ok": True, "ack": {"status": 100}, "readback": {"value": 0, "ack": {"status": 100}}},
                "position_after_sethome": {"ok": True, "position": 0, "ack": {"status": 100}},
                "switch_transition": False,
            },
        }

    def motor_oem_stop_exact(self, board, *, motor=0):
        self.calls.append(("stop", board, motor))
        return {
            "ok": True, "first_delivery": {"status": 100}, "second_delivery": {"status": 100},
            "terminal_speed": {"ok": True, "speed": 0, "ack": {"status": 100}},
        }

    def motor_set_home(self, board, *, motor=0):
        self.calls.append(("set_home", board, motor))
        self.position = 0
        return {
            "ok": True,
            "ack": {"status": 100},
            "readback": {"ok": True, "value": 0, "ack": {"status": 100}},
        }


def make_provider(tmp_path):
    store = OEMRuntimeStore(tmp_path / "runtime")
    store.record_board4_transition(
        active=True, ack={"status": 100}, transition_id="activate", ownership_generation=4, invalidate_axes=False
    )
    provider = Serial206YProvider(FakeYTester(), state_store=store, generation_provider=lambda: 4)
    assert provider.prepare()["ok"] is True
    return provider


def test_y_profile_is_selected_machine_source_shape(tmp_path):
    provider = Serial206YProvider(FakeYTester(), state_store=None, generation_provider=lambda: 1)
    profile = provider.profile()
    assert profile["board"] == 4
    assert profile["motor"] == 0
    assert profile["axis_max_steps"] == 102956
    assert profile["speed"] == 1800
    assert profile["acc"] == 400
    assert profile["home_speed"] == 500


def test_relative_guard_rejects_both_twenty_step_end_bands(tmp_path):
    provider = make_provider(tmp_path)
    tester = provider.tester
    tester.position = 0
    low = provider.move_steps(19)
    assert low["ok"] is False
    assert low["failure"] == "y_relative_target_out_of_range"
    tester.position = 102956
    high = provider.move_steps(-19)
    assert high["ok"] is False
    assert high["failure"] == "y_relative_target_out_of_range"
    assert not [call for call in tester.calls if call[0] == "relative"]


def test_relative_move_reconciles_observed_discrepancy_without_latch(tmp_path):
    provider = make_provider(tmp_path)
    result = provider.move_steps(100, command_id="y-relative-1")
    assert result["ok"] is True
    assert result["completion_class"] == "event_128"
    assert result["physical_effect_verified"] is False
    axis = provider.state_store.board4_authority_projection()["axes"]["y"]
    assert axis["lifecycle_state"] == "prepared_unreferenced"
    assert axis["reference_state"] == "unreferenced"


def test_absolute_move_clamps_controller_target_and_starts_pending_by_default(tmp_path):
    provider = make_provider(tmp_path)
    result = provider.move_absolute(-5, command_id="y-absolute-1")
    assert result["ok"] is True
    assert result["state"] == "issued_pending"
    assert result["caller_requested_target"] == -5
    assert result["board_effective_target"] == 0
    assert provider.tester.calls[-1] == ("absolute", 4, 0, False, 102956)


def test_home_modes_use_distinct_source_speed_and_publish_reference_without_transition(tmp_path):
    provider = make_provider(tmp_path)
    result = provider.home("diagnostic", command_id="y-home-1")
    assert result["ok"] is True
    assert result["source_mode"] == "diagnostic"
    assert result["source_speed"] == 250
    assert result["reference_published"] is True
    assert provider.tester.calls[-1] == ("home", "y", 250, True, False)

    result = provider.home("manual_panel", command_id="y-home-2")
    assert result["source_speed"] == 500
    assert provider.tester.calls[-1] == ("home", "y", 500, False, False)


def test_stop_is_y_scoped_and_preserves_z_projection(tmp_path):
    provider = make_provider(tmp_path)
    result = provider.stop(command_id="y-stop-1")
    assert result["ok"] is True
    assert result["axis"] == "y"
    assert result["stop"]["first_delivery"]["status"] == 100
    axes = provider.state_store.board4_authority_projection()["axes"]
    assert axes["z"]["lifecycle_state"] == "unprepared"


def test_set_home_is_explicit_no_motion_and_does_not_claim_home_reference(tmp_path):
    provider = make_provider(tmp_path)
    result = provider.set_home("SET_HOME_CURRENT_POSITION", command_id="y-set-home-1")
    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert result["reference_published"] is False
    assert provider.tester.calls[-1] == ("set_home", 4, 0)


def test_home_xy_publishes_only_the_y_child_reference_from_complete_proof(tmp_path):
    provider = make_provider(tmp_path)
    result = provider.publish_home_xy_reference(
        {
            "ok": True,
            "home": {"y": {"ok": True, "controller_home_proof_verified": True}},
            "positions": {"y": {"ok": True, "position": 0, "ack": {"status": 100}}},
        },
        command_id="homexy-1",
    )
    assert result["ok"] is True
    assert result["reference_published"] is True
    axes = provider.state_store.board4_authority_projection()["axes"]
    assert axes["y"]["reference_state"] == "referenced"
    assert axes["z"]["reference_state"] == "unreferenced"


def test_move_xy_reconciles_y_observation_without_touching_z_authority(tmp_path):
    provider = make_provider(tmp_path)
    result = provider.record_move_xy_observation(
        {"ok": True, "targets": {"x": 200, "y": 120}, "after": {"x": 200, "y": 117}},
        command_id="move-xy-1",
    )
    assert result["ok"] is True
    assert result["discrepancy_steps"] == -3
    axes = provider.state_store.board4_authority_projection()["axes"]
    assert axes["y"]["observed_position_steps"] == 117
    assert axes["z"]["observed_position_steps"] is None
