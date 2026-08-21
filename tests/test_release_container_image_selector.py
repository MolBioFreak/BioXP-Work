from pathlib import Path


RUNNER = Path(__file__).parents[1] / "scripts" / "bioxp_release_container_run.sh"


def test_release_runner_accepts_release_selected_container_image():
    source = RUNNER.read_text()
    assert "CONTAINER_NAME=07252f6e0fbdc226:20260821-4374bdc" in source
    assert "--pull=never" in source
