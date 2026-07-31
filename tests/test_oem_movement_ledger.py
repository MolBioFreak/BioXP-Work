from src.bioxp.oem_movement_ledger import OemMovementLedger


class _Store:
    def __init__(self, payload):
        self.payload = payload

    def read_oem_movement_ledger(self):
        return self.payload

    def write_oem_movement_ledger(self, payload):
        self.payload = payload


def test_legacy_ledger_missing_inserted_x_stages_fails_closed_without_skipping():
    legacy = OemMovementLedger._new()
    for key in ("x-home-settle", "x-set-home", "x-speed-1700", "x-speed-settle"):
        del legacy["stages"][key]
    legacy["expected_next_stage"] = "x-park-6000"
    store = _Store(legacy)
    ledger = OemMovementLedger(store)

    projection = ledger.projection()
    admission = ledger.admit(stage="x-park-6000", command_id="must-not-run")

    assert projection["terminal_state"] == "failed_closed"
    assert projection["compatibility_blocker"] == "oem_initializeMotors_ledger_stage_schema_incompatible"
    assert admission["ok"] is False
    assert admission["blocker"] == "oem_initializeMotors_ledger_stage_schema_incompatible"
    assert store.payload["stages"]["x-park-6000"]["state"] == "pending"


def test_ledger_rejects_non_boolean_verdict_and_blank_note_without_advancing():
    store = _Store(None)
    ledger = OemMovementLedger(store)
    assert ledger.admit(stage="z-home", command_id="z-home-command")["ok"] is True
    ledger.record_result(
        stage="z-home",
        command_id="z-home-command",
        result={"ok": True, "physical_effect_verified": False},
        artifact_path=None,
    )

    coercible = ledger.record_observation(
        stage="z-home",
        observed_pass="false",  # type: ignore[arg-type]
        note="Observed failure.",
        command_id="observe-coercible",
    )
    blank = ledger.record_observation(
        stage="z-home",
        observed_pass=True,
        note="   ",
        command_id="observe-blank",
    )

    assert coercible["ok"] is False
    assert coercible["blocker"] == "oem_initializeMotors_observed_pass_must_be_boolean"
    assert blank["ok"] is False
    assert blank["blocker"] == "oem_initializeMotors_operator_note_required"
    assert store.payload is not None
    assert store.payload["stages"]["z-home"]["state"] == "acknowledged"
    assert store.payload["stages"]["z-home"]["observation"] is None
    assert store.payload["expected_next_stage"] == "z-home"


def test_controller_verifiable_gripper_current_advances_without_human_observation():
    store = _Store(None)
    ledger = OemMovementLedger(store)
    assert ledger.admit(stage="z-home", command_id="z")["ok"] is True
    ledger.record_result(stage="z-home", command_id="z", result={"ok": True}, artifact_path=None)
    assert ledger.record_observation(
        stage="z-home",
        observed_pass=True,
        note="Z reached the upper reference.",
        command_id="observe-z",
    )["ok"] is True

    assert ledger.admit(stage="gripper-current-31", command_id="current")["ok"] is True
    result = ledger.record_result(
        stage="gripper-current-31",
        command_id="current",
        result={"ok": True, "controller_readback": 31},
        artifact_path=None,
    )

    assert result["stages"]["gripper-current-31"]["state"] == "completed"
    assert result["stages"]["gripper-current-31"]["requires_operator_observation"] is False
    assert result["expected_next_stage"] == "gripper-clear-10000"


def test_legacy_acknowledged_gripper_current_is_migrated_to_completed():
    legacy = OemMovementLedger._new()
    legacy["stages"]["z-home"]["state"] = "operator_observed"
    legacy["stages"]["gripper-current-31"].update(
        {
            "state": "acknowledged",
            "requires_operator_observation": True,
            "command_id": "legacy-current",
            "result": {"ok": True, "controller_readback": 31},
        }
    )
    legacy["expected_next_stage"] = "gripper-current-31"
    legacy["terminal_state"] = "awaiting_operator_observation"

    store = _Store(legacy)
    projection = OemMovementLedger(store).projection()

    assert projection["stages"]["gripper-current-31"]["state"] == "completed"
    assert projection["stages"]["gripper-current-31"]["requires_operator_observation"] is False
    assert projection["expected_next_stage"] == "gripper-clear-10000"
    assert projection["terminal_state"] == "awaiting_next_stage"
    assert store.payload["expected_next_stage"] == "gripper-clear-10000"
    assert store.payload["stages"]["gripper-current-31"]["state"] == "completed"