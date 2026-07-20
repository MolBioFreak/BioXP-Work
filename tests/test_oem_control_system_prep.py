from pathlib import Path

from src.bioxp.protocols.models import ProtocolActionKind


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "testdata" / "oem_xml"
LIFETEST_XML = FIXTURE_ROOT / "lifetest.xml"


def test_axis_profiles_expose_source_anchored_testing_matrix():
    from src.bioxp.oem_compat.boards import axis_profile_matrix

    matrix = axis_profile_matrix()

    assert set(matrix) == {"x", "y", "z", "g", "door"}
    assert matrix["x"]["board_id"] == 0x05
    assert matrix["x"]["startup_home_speed"] == 250
    assert matrix["x"]["manual_home_speed"] == 500
    assert matrix["z"]["manual_home_speed"] == 1791
    assert matrix["g"]["gripper_version_1_home_speed"] == 200
    assert matrix["door"]["home_method"] == "doorSearchHome"
    assert all(row["source_anchor"] for row in matrix.values())


def test_position_table_resolves_oem_location_well_plate_targets():
    from src.bioxp.oem_compat.position_table import PositionTable

    table = PositionTable.from_rows(
        [
            {
                "locationID": "PL_POOL",
                "wellID": "WLA9",
                "plateName": "pool",
                "x": 1000,
                "y": 2000,
                "zHigh": -300,
                "xOffset": 10,
                "yOffset": -20,
                "zOffset": 5,
                "source": "fixture-position-table.xml",
            }
        ]
    )

    target = table.resolve(location_id="PL_POOL", well_id="WLA9", plate_name="pool")
    assert target.location_id == "PL_POOL"
    assert target.coordinates == {"x": 1010, "y": 1980, "z": -295}
    assert target.source_anchor == "fixture-position-table.xml"

    move = table.compile_move_to("PL_POOL", well_id="WLA9", plate_name="pool")
    assert move["semantic_action"] == "moveTo"
    assert move["target"]["coordinates"]["x"] == 1010
    assert move["requires_reference_axes"] == ["x", "y", "z"]


def test_oem_script_translator_supports_testing_critical_mt_fp_semantics():
    from src.bioxp.oem_compat.scripts import OemScript, OemScriptTranslator

    script = OemScript.from_text(
        """
        <WpfGenBotCommonLib><script>
          <line1 cmd="FP PL_REAGENT ML /MLPCR FP /R T200 /NTY" />
          <line2 cmd="MT PL_REAGENT ML /MLPCR VL /VL26 AP /AA0 PL_POOL ML /WLA9 DP /DC0 T50 /NTN" />
        </script></WpfGenBotCommonLib>
        """
    )
    translated = OemScriptTranslator().translate(script)

    assert [c.command for c in translated.commands] == ["fluid_prep", "liquid_transfer"]
    assert all(c.supported for c in translated.commands)
    assert translated.commands[1].metadata["source_location_id"] == "PL_REAGENT"
    assert translated.commands[1].metadata["dest_location_id"] == "PL_POOL"
    assert translated.commands[1].metadata["volume_ul"] == 26.0
    assert translated.commands[1].metadata["requires_ack_readback"] is True


def test_oem_xml_import_now_compiles_lifetest_mt_fp_for_dry_run_testing():
    from src.bioxp.protocols.oem_xml_import import import_oem_xml_protocol

    imported = import_oem_xml_protocol(LIFETEST_XML)

    assert imported.coverage.unsupported_verbs.get("MT", 0) == 0
    assert imported.coverage.unsupported_verbs.get("FP", 0) == 0
    assert imported.coverage.supported_verbs["MT"] == 62
    assert imported.coverage.supported_verbs["FP"] == 3
    assert imported.coverage.supported_command_count == 178
    assert imported.coverage.unsupported_command_count == 0
    assert imported.coverage.unsupported_verbs == {}
    mt_action = next(
        action
        for stage in imported.document.stages
        for action in stage.actions
        if action.metadata.get("oem_verb") == "MT"
    )
    assert mt_action.kind is ProtocolActionKind.PIPETTE_ASPIRATE
    assert mt_action.params["semantic_action"] == "liquid_transfer"
    assert mt_action.params["source_location_id"] == "PL_REAGENT"
    assert mt_action.params["dest_location_id"] == "PL_POOL"
    assert mt_action.params["requires_ack_readback"] is True


def test_pipette_plans_are_fail_closed_ack_readback_contracts():
    from src.bioxp.oem_compat.pipette import PipetteController

    controller = PipetteController.dry_run()
    plan = controller.aspirate(26, source={"location_id": "PL_REAGENT", "well_id": "MLPCR"})

    assert plan.command_ascii == "P26,1R"
    assert plan.requires_readback is True
    assert plan.ack_policy == "ack_and_status_readback_required"
    assert plan.preflight["requires_reference_axes"] == ["x", "y", "z"]
    assert plan.preflight["requires_tip_state"] is True
    assert plan.executed is False


def test_vision_results_have_artifacted_failure_contracts_for_testing():
    from src.bioxp.oem_compat.vision import VisionFacade

    result = VisionFacade.dry_run().scan_barcode(location_id="PL_OUTPUT")

    assert result.status == "unavailable"
    assert result.artifact_contract["image_required"] is True
    assert result.artifact_contract["failure_is_semantic"] is True
    assert result.location_id == "PL_OUTPUT"
