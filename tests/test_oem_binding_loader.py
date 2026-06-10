import json
from pathlib import Path

import pytest

from src.bioxp.domain import (
    DEFAULT_DECK_LAYOUT_PATH,
    OemBindingStatus,
    bind_oem_metadata_to_deck_layout,
    load_deck_layout,
    load_oem_binding_data,
)


def _write_binding(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "schema": "bioxp-oem-binding-v1",
                "source": {
                    "source_path": "/oem/decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs",
                    "source_type": "decompiled_csharp",
                    "source_key": "ClassBioXPSettings.PositionTable",
                    "source_hash": "sha256:test-fixture",
                },
                "sections": {
                    "position_table": {
                        "status": "available",
                        "source_key": "PositionTable",
                        "entries": {
                            "LOC_RC": {
                                "name": "Reagent Tray",
                                "x": 12000,
                                "y": 4000,
                                "zLow": 18000,
                                "zHigh": 5000,
                                "zDelta": 13000,
                                "inc_factor": 1,
                            }
                        },
                    },
                    "vision_calibration": {
                        "status": "not_extracted",
                        "reason": "InspectionSettings.xml absent from SSD backup",
                        "source_key": "InspectionSettings.xml",
                    },
                },
            }
        ),
        encoding="utf-8",
    )


def test_loads_source_backed_oem_binding_without_hardware(tmp_path):
    binding_path = tmp_path / "binding.json"
    _write_binding(binding_path)

    binding = load_oem_binding_data(binding_path)

    assert binding.schema == "bioxp-oem-binding-v1"
    assert binding.source.source_type == "decompiled_csharp"
    assert binding.sections["position_table"].status is OemBindingStatus.AVAILABLE
    assert binding.sections["vision_calibration"].status is OemBindingStatus.NOT_EXTRACTED
    assert binding.sections["vision_calibration"].reason == "InspectionSettings.xml absent from SSD backup"
    assert binding.sections["position_table"].payload["entries"]["LOC_RC"]["zDelta"] == 13000


def test_binds_oem_provenance_to_existing_deck_layout_metadata(tmp_path):
    binding_path = tmp_path / "binding.json"
    _write_binding(binding_path)

    layout = bind_oem_metadata_to_deck_layout(DEFAULT_DECK_LAYOUT_PATH, binding_path)

    assert layout.deck_id == "bioxp-default"
    assert layout.metadata["oem_binding"]["schema"] == "bioxp-oem-binding-v1"
    assert layout.metadata["oem_binding"]["source"]["source_key"] == "ClassBioXPSettings.PositionTable"
    assert layout.metadata["oem_binding"]["sections"]["position_table"]["status"] == "available"
    assert layout.metadata["oem_binding"]["sections"]["vision_calibration"]["status"] == "not_extracted"
    assert layout.metadata["oem_binding"]["sections"]["vision_calibration"]["reason"] == "InspectionSettings.xml absent from SSD backup"


@pytest.mark.parametrize(
    "payload, match",
    [
        ({"schema": "wrong", "source": {}, "sections": {}}, "schema"),
        ({"schema": "bioxp-oem-binding-v1", "source": {}, "sections": {}}, "source_path"),
        (
            {
                "schema": "bioxp-oem-binding-v1",
                "source": {"source_path": "x", "source_type": "y", "source_key": "z"},
                "sections": {"position_table": {"status": "available"}},
            },
            "entries",
        ),
        (
            {
                "schema": "bioxp-oem-binding-v1",
                "source": {"source_path": "x", "source_type": "y", "source_key": "z"},
                "sections": {"mystery": {"status": "available", "entries": {}}},
            },
            "Unsupported OEM binding section",
        ),
    ],
)
def test_rejects_malformed_or_unknown_critical_oem_binding_sections(tmp_path, payload, match):
    binding_path = tmp_path / "binding.json"
    binding_path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match=match):
        load_oem_binding_data(binding_path)


def test_default_deck_layout_remains_unchanged_after_loading_binding(tmp_path):
    binding_path = tmp_path / "binding.json"
    _write_binding(binding_path)

    before = load_deck_layout(DEFAULT_DECK_LAYOUT_PATH)
    bound = bind_oem_metadata_to_deck_layout(DEFAULT_DECK_LAYOUT_PATH, binding_path)
    after = load_deck_layout(DEFAULT_DECK_LAYOUT_PATH)

    assert "oem_binding" not in before.metadata
    assert "oem_binding" in bound.metadata
    assert "oem_binding" not in after.metadata
    assert before.deck_id == after.deck_id
    assert before.version == after.version
    assert before.slots == after.slots
    assert before.locations == after.locations
