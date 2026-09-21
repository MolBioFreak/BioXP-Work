"""Actual initial-check body; only physical leaves/time and authority doubled."""
import pytest
from bioxp.lifecycle_state import CanonicalLifecycleOwner, LifecycleStateError


class Hardware:
    def __init__(self, *, voltage=0, fail=None):
        self.calls = []
        self.voltage = voltage
        self.fail = fail
        self.epoch = 0

    def leaf(self, name, **result):
        self.calls.append(name)
        if self.fail == name:
            return {"ok": False, "error": "native_reply_failed"}
        if self.fail == name + "_exception":
            raise OSError("native transport failed")
        return {"ok": True, **result}

    def set_led_rgb(self, *rgb):
        assert rgb == (255, 255, 255)
        return self.leaf("led")

    def query_door(self):
        return self.leaf("door", value=1)

    def query_latch(self):
        return self.leaf("latch", value=1)

    def query_voltage(self):
        return self.leaf("voltage", payload_raw=self.voltage,
                         reply_present=True, oem_status=100)

    def set_solenoid(self, value):
        return self.leaf("solenoid" + str(value))

    def deactivate_boards(self):
        return self.leaf("deactivate")

    def activate_boards(self):
        return self.leaf("activate")

    def oem_begin_board_lifecycle_generation(self, *, deactivation, activation):
        assert deactivation == activation == {"ok": True}
        result = self.leaf("generation")
        if result.get("ok"):
            self.epoch += 1
        return result


def owner():
    state = CanonicalLifecycleOwner()
    state.transport_changed(True, reason="fixture transport")
    state.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    state.run_stage("initialization_without_motion", lambda: {"ok": True})
    state.transition("paused", reason="fixture deferred gate")
    return state


def wake(state, hardware, validate=lambda phase: True, ready=lambda: True):
    return state.run_workflow_wake_initial_check(
        hardware, validate_current=validate, can_ready=ready,
        sleep=lambda seconds: None, clock=lambda: 0.0,
    )


def test_legitimate_wake_does_not_replay_startup_or_publish_stopped():
    state, hardware = owner(), Hardware()
    before = state.projection()
    phases = []
    result = wake(state, hardware, lambda phase: phases.append(phase) or True)
    assert result["ok"] is True and result["source_return"] is True
    assert hardware.calls == ["led", "door", "latch", "solenoid1", "door", "latch",
                              "voltage", "deactivate", "activate", "generation"]
    assert phases[-2:] == ["oem_begin_board_lifecycle_generation", "completion"]
    assert result["initial_check"]["sleeps_ms"] == [50, 500, 800]
    assert state.projection()["startup"] == before["startup"]
    assert state.projection()["operation_state"] == "paused"
    assert hardware.epoch == 1


@pytest.mark.parametrize("operation", ["running", "paused", "emergency"])
def test_public_startup_active_refusal_unchanged(operation):
    state, hardware = owner(), Hardware()
    state.transition(operation, reason="test")
    with pytest.raises(LifecycleStateError, match="cannot run"):
        state.run_initial_check(hardware, can_ready=lambda: True)
    assert hardware.calls == []


@pytest.mark.parametrize("phase", ["admission", "can_ready", "query_door", "activate_boards", "completion"])
@pytest.mark.parametrize("drift", ["owner", "gate"])
def test_callback_rejects_stale_owner_or_gate(phase, drift):
    state, hardware = owner(), Hardware()
    expected = {"owner": "parent-attempt", "gate": "reached-gate"}
    current = dict(expected)
    def validate(where):
        if where == phase:
            current[drift] = "replacement"
        return current == expected
    with pytest.raises(LifecycleStateError, match="authority rejected"):
        wake(state, hardware, validate)
    if phase in {"admission", "can_ready"}:
        assert not hardware.calls
    if phase == "query_door":
        assert hardware.calls == ["led"]
    if phase == "activate_boards":
        assert hardware.calls[-1] == "deactivate"
    assert state.projection()["operation_state"] == "paused"


def test_true_source_false_can_timeout():
    hardware = Hardware()
    result = wake(owner(), hardware, ready=lambda: False)
    assert result["ok"] is True and result["source_return"] is False
    assert result["initial_check"]["sleeps_ms"] == [200] * 12
    assert hardware.calls == []


def test_true_source_false_nonzero_valid_voltage():
    hardware = Hardware(voltage=1)
    result = wake(owner(), hardware)
    assert result["ok"] is True and result["source_return"] is False
    assert hardware.calls[-1] == "solenoid0"
    assert "deactivate" not in hardware.calls
    assert result["initial_check"]["sleeps_ms"] == [50, 500, 800, 300]


@pytest.mark.parametrize("failure", ["led", "door", "latch", "solenoid1", "voltage", "deactivate", "activate", "generation", "solenoid0"])
def test_native_failed_result_not_ignored_source_false(failure):
    hardware = Hardware(fail=failure, voltage=1 if failure == "solenoid0" else 0)
    result = wake(owner(), hardware)
    assert result["ok"] is False and result["source_return"] is None
    assert hardware.calls[-1] == failure


def test_invalid_voltage_reply_not_source_false():
    hardware = Hardware()
    hardware.query_voltage = lambda: {"ok": True, "reply_present": False}
    result = wake(owner(), hardware)
    assert result["ok"] is False and result["source_return"] is None
    assert "deactivate" not in hardware.calls


@pytest.mark.parametrize("failure", ["led_exception", "activate_exception"])
def test_native_exception_propagates(failure):
    with pytest.raises(OSError, match="native transport"):
        wake(owner(), Hardware(fail=failure))


def test_missing_generation_producer_refused_before_hardware():
    hardware = Hardware()
    hardware.oem_begin_board_lifecycle_generation = None
    with pytest.raises(LifecycleStateError, match="generation producer"):
        wake(owner(), hardware)
    assert not hardware.calls


@pytest.mark.parametrize("answer", [None, False, 1, {"ok": True}])
def test_validation_requires_literal_true(answer):
    hardware = Hardware()
    with pytest.raises(LifecycleStateError, match="authority rejected"):
        wake(owner(), hardware, lambda phase: answer)
    assert not hardware.calls


def test_emergency_not_authorized_by_callback():
    state, hardware = owner(), Hardware()
    state.transition("emergency", reason="test")
    with pytest.raises(LifecycleStateError, match="emergency"):
        wake(state, hardware)
    assert not hardware.calls


@pytest.mark.parametrize("external_drift", [False, True])
def test_generation_completion_uses_exact_producer_handoff(external_drift):
    state, hardware = owner(), Hardware()
    admitted = {"epoch": 0}
    producer = hardware.oem_begin_board_lifecycle_generation
    def publish(**kwargs):
        result = producer(**kwargs)
        # Test double for canonical producer publication, not epoch rebasing.
        assert admitted["epoch"] == 0 and hardware.epoch == 1
        admitted["epoch"] = 1
        if external_drift:
            hardware.epoch = 2
        return result
    hardware.oem_begin_board_lifecycle_generation = publish
    validate = lambda phase: hardware.epoch == admitted["epoch"]
    if external_drift:
        with pytest.raises(LifecycleStateError, match="completion"):
            wake(state, hardware, validate)
    else:
        assert wake(state, hardware, validate)["ok"] is True


def test_board_test_source_branch_remains_activation_only():
    state, hardware = owner(), Hardware()
    state.bind_configuration(start_mode="test", board_test_mode=True, check_camera=False)
    result = wake(state, hardware)
    assert result["source_return"] is True and result["ok"] is True
    assert hardware.calls == ["activate"]
    assert hardware.epoch == 0


def test_public_startup_still_publishes_stage_and_preserves_constructor_configuration():
    state, hardware = owner(), Hardware()
    state.transition("stopped", reason="test")
    configured = state.projection()["startup"]["stages"]["initialization_without_motion"]
    def initial():
        return state.run_initial_check(hardware, can_ready=lambda: True,
                                      sleep=lambda _: None, clock=lambda: 0.0)
    assert initial()["startup"]["stages"]["initial_check"]["state"] == "passed"
    result = initial()
    assert result["operation_state"] == "stopped"
    assert result["startup"]["stages"]["initialization_without_motion"] == configured
    repeated = result["startup"]["stages"]["initial_check"]
    assert repeated["state"] == "passed"
    assert len(repeated["history"]) == 1
    assert repeated["history"][0]["attempt_id"] != repeated["attempt_id"]
