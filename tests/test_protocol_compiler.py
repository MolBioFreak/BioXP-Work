from src.bioxp.domain.capabilities import CapabilityName
from src.bioxp.protocols.compiler import compile_native_protocol
from src.bioxp.protocols.models import ProtocolActionKind


def test_compile_native_protocol_builds_normalized_action_schema():
    document = compile_native_protocol(
        {
            "protocol_id": "tiny-native",
            "version": 1,
            "stages": [
                {
                    "stage_id": "load",
                    "title": "Load Plate",
                    "actions": [
                        {"kind": "move", "target": "reagent_rack"},
                        {"kind": "pause_review", "message": "Confirm reagent plate is seated"},
                    ],
                },
                {
                    "stage_id": "transfer",
                    "actions": [
                        {"kind": "pipette_aspirate", "volume_ul": 25.0, "source": "reagent_rack:A1"},
                        {"kind": "pipette_dispense", "volume_ul": 25.0, "dest": "magnetic_station:A1"},
                    ],
                },
            ],
        }
    )

    assert document.protocol_id == "tiny-native"
    assert len(document.stages) == 2
    assert document.stages[0].actions[0].kind is ProtocolActionKind.MOVE
    assert document.stages[0].actions[0].required_capability is CapabilityName.MOTION
    assert document.stages[0].actions[1].review_required is True
    assert document.stages[1].actions[0].required_capability is CapabilityName.PIPETTE
    assert document.stages[1].actions[0].params["volume_ul"] == 25.0


def test_compile_native_protocol_rejects_duplicate_stage_ids():
    try:
        compile_native_protocol(
            {
                "protocol_id": "dup-stage",
                "stages": [
                    {"stage_id": "stage_a", "actions": [{"kind": "note", "message": "first"}]},
                    {"stage_id": "stage_a", "actions": [{"kind": "note", "message": "second"}]},
                ],
            }
        )
    except ValueError as exc:
        assert "duplicate" in str(exc).lower()
    else:
        raise AssertionError("expected duplicate stage validation to fail")
