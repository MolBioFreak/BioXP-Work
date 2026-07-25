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