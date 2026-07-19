from pathlib import Path

from src.bioxp.protocols.models import ProtocolActionKind
from src.bioxp.protocols.oem_xml_import import generate_oem_fixture_coverage_report, import_oem_xml_protocol

OEM_SCRIPT_ROOT = Path(__file__).resolve().parents[1] / "scripts"
OEM_SCRIPT_PATHS = sorted(OEM_SCRIPT_ROOT.glob("*.xml"))


def _actions_by_verb(imported, verb: str):
    verb = verb.upper()
    return [
        action
        for stage in imported.document.stages
        for action in stage.actions
        if action.metadata.get("oem_verb") == verb
    ]


def test_full_oem_script_corpus_has_zero_unsupported_commands() -> None:
    assert len(OEM_SCRIPT_PATHS) == 5

    report = generate_oem_fixture_coverage_report(OEM_SCRIPT_PATHS)

    assert report["file_count"] == 5
    assert report["aggregate"]["command_nodes_total"] == 2933
    assert report["aggregate"]["supported_command_count"] == 2933
    assert report["aggregate"]["unsupported_command_count"] == 0
    assert report["aggregate"]["unsupported_verbs"] == {}


def test_lifetest_purification_macros_are_structured_semantic_actions() -> None:
    imported = import_oem_xml_protocol(OEM_SCRIPT_ROOT / "lifetest.xml")

    assert imported.coverage.unsupported_command_count == 0
    for verb in ("RT", "SA", "ST", "SW", "TT", "ZW"):
        assert imported.coverage.supported_verbs[verb] > 0
        action = _actions_by_verb(imported, verb)[0]
        assert action.kind in {
            ProtocolActionKind.PIPETTE_ASPIRATE,
            ProtocolActionKind.PIPETTE_DISPENSE,
            ProtocolActionKind.PIPETTE_MIX,
        }
        assert action.params["semantic_action"]
        assert action.params["macro_verb"] == verb
        assert action.params["raw_tokens"]
        assert action.params["requires_virtual_bioxp_state"] is True
        assert action.params["requires_ack_readback"] is True


def test_observed_wash_and_reelution_macro_metadata_is_lossless_enough_for_virtual_state() -> None:
    imported = import_oem_xml_protocol(OEM_SCRIPT_ROOT / "lifetest.xml")

    sa = _actions_by_verb(imported, "SA")[0]
    assert sa.params["material_id"] == "AMP"
    assert sa.params["tip_type"] == "T200"

    st = _actions_by_verb(imported, "ST")[0]
    assert st.params["material_id"] == "AMP"
    assert st.params["dest_location_id"] == "PL_POOL"
    assert st.params["dest_zone"] == "Z5"
    assert st.params["volume_ul"] == 14.0
    assert st.params["tip_type"] == "T50"
    assert st.params["tip_option"] == "NTY"
    assert st.params["option_groups"]["AO"]["A"] == 10.0
    assert st.params["option_groups"]["DAO"]["S"] == 10.0
    assert st.params["option_groups"]["MO"]["RC"] == 1
    assert st.params["option_groups"]["DBO"]["RC"] == 5

    tt = _actions_by_verb(imported, "TT")[0]
    assert tt.params["material_id"] == "ETOH"
    assert tt.params["volume_ul"] == 4320.0
    assert tt.params["tip_type"] == "T200"
    assert tt.params["scalar_options"]["AM"] == 0
    assert tt.params["scalar_options"]["DM"] == 0

    zw = _actions_by_verb(imported, "ZW")[0]
    assert zw.params["target_location_id"] == "PL_POOL"
    assert zw.params["target_zone"] == "Z5"
    assert zw.params["source_location_id"] == "PL_OUTPUT"
    assert zw.params["source_zone"] == "Z9"
    assert zw.params["volume_ul"] == 120.0
    assert zw.params["wash_count"] == 10
    assert zw.params["mix_repeat"] == 5
    assert zw.params["material_id"] == "AMP"

    sw = _actions_by_verb(imported, "SW")[0]
    assert sw.params["wash_material_id"] == "ETOH"
    assert sw.params["target_location_id"] == "PL_POOL"
    assert sw.params["target_zone"] == "Z5"
    assert sw.params["volume_ul"] == 120.0
    assert sw.params["wash_count"] == 10
    assert sw.params["mix_repeat"] == 5
    assert sw.params["material_id"] == "AMP"

    rt = _actions_by_verb(imported, "RT")[0]
    assert rt.params["material_id"] == "EB"
    assert rt.params["target_location_id"] == "PL_POOL"
    assert rt.params["target_zone"] == "Z5"
    assert rt.params["volume_ul"] == 50.0
    assert rt.params["tip_type"] == "T50"
    assert rt.params["tip_option"] == "NTY"
    assert rt.params["option_groups"]["MO"]["RC"] == 25
    assert rt.params["option_groups"]["DBO"]["RC"] == 5
