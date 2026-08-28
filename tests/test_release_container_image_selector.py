from pathlib import Path


RUNNER = Path(__file__).parents[1] / "scripts" / "bioxp_release_container_run.sh"


def test_release_runner_uses_only_immutable_image_and_verified_manifest_contract():
    source = RUNNER.read_text()
    assert "CONTAINER_NAME=07252f6e0fbdc226:20260821-4374bdc" not in source
    assert 'image_id_pattern = re.compile(r"^sha256:[0-9a-f]{64}$")' in source
    assert '"$IMAGE_ID"' in source
    assert '"$UDOCKER_BIN" run \\\n  --repo="$UDOCKER_ROOT/store" \\' in source
    assert "--pull=never" in source
    assert "SOURCE_MANIFEST_FILE=/etc/bioxp/source-manifest.json" in source
    assert "IMAGE_INSPECTION_FILE=/etc/bioxp/image-inspection.json" in source
    assert "source_bytes_verified" in source
    assert "source_manifest_sha256" in source
    assert "source_aggregate_sha256" in source
    assert "image_inspection_receipt_sha256" in source
    assert '"declared_listener": {"host": "0.0.0.0", "port": 8123}' in source
    assert '"observed_listener": None' in source
