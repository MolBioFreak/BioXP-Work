from pathlib import Path

from src.bioxp.protocols.models import ProtocolActionKind
from src.bioxp.protocols.oem_xml_import import generate_oem_fixture_coverage_report, import_oem_xml_protocol


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "testdata" / "oem_xml"
DEMO_XML = FIXTURE_ROOT / "demo.xml"
LIFETEST_XML = FIXTURE_ROOT / "lifetest.xml"


def test_import_demo_xml_maps_supported_subset_with_source_traceability() -> None:
    imported = import_oem_xml_protocol(DEMO_XML)

    assert imported.document.protocol_id == "oem-system-demo-v4"
    assert [stage.stage_id for stage in imported.document.stages] == ["step-01", "step-02", "step-03", "step-04"]
    assert imported.coverage.command_nodes_total == 68
    assert imported.coverage.supported_command_count == 68
    assert imported.coverage.unsupported_command_count == 0
    assert imported.coverage.coverage_ratio == 1.0
    assert imported.coverage.supported_verbs["MP"] == 4

    first_action = imported.document.stages[0].actions[0]
    assert first_action.kind is ProtocolActionKind.LED
    assert first_action.metadata["source_node"]["tag"] == "line2"
    assert first_action.metadata["source_node"]["script_position"] == 2
    assert first_action.metadata["source_node"]["raw_cmd"] == "LED 255 0 0"

    delaypoint = next(
        action
        for stage in imported.document.stages
        for action in stage.actions
        if action.kind is ProtocolActionKind.PAUSE_REVIEW
    )
    assert delaypoint.review_required is True
    assert delaypoint.pause_message == "OEM delay point reached"


def test_import_lifetest_xml_reports_unsupported_verbs_and_preserves_step_traceability() -> None:
    imported = import_oem_xml_protocol(LIFETEST_XML)

    assert imported.document.protocol_id == "oem-assembly-lifetest-vx"
    assert [stage.stage_id for stage in imported.document.stages] == ["step-01", "step-02", "step-03", "step-04"]
    assert imported.coverage.command_nodes_total == 178
    assert imported.coverage.supported_command_count == 95
    assert imported.coverage.unsupported_command_count == 83
    assert imported.coverage.unsupported_verbs["MT"] == 62
    assert imported.coverage.unsupported_verbs["FP"] == 3
    assert 0.5 < imported.coverage.coverage_ratio < 0.6

    thermal_action = next(
        action
        for stage in imported.document.stages
        for action in stage.actions
        if action.kind is ProtocolActionKind.THERMAL_SETPOINT
    )
    assert thermal_action.params["target_c"] == 98.0
    assert thermal_action.params["duration_s"] == 60.0
    assert thermal_action.metadata["source_node"]["step"] == "2"

    unsupported = imported.coverage.unsupported_commands[0]
    assert unsupported.reason == "unsupported_oem_verb"
    assert unsupported.source_node["source_file"] == "lifetest.xml"
    assert unsupported.verb in imported.coverage.unsupported_verbs


def test_generate_oem_fixture_coverage_report_aggregates_fixture_counts() -> None:
    report = generate_oem_fixture_coverage_report([DEMO_XML, LIFETEST_XML])

    assert report["file_count"] == 2
    assert report["aggregate"]["command_nodes_total"] == 246
    assert report["aggregate"]["supported_command_count"] == 163
    assert report["aggregate"]["unsupported_command_count"] == 83
    assert report["aggregate"]["unsupported_verbs"]["MT"] == 62
    assert report["aggregate"]["supported_verbs"]["LED"] == 34
    assert report["files"][0]["coverage"]["coverage_ratio"] == 1.0
