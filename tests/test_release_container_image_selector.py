from pathlib import Path


RUNNER = Path(__file__).parents[1] / "scripts" / "bioxp_release_container_run.sh"


def test_release_runner_accepts_release_selected_container_image():
    source = RUNNER.read_text()
    assert 'CONTAINER_NAME="${BIOXP_CONTAINER_NAME:-bioxp-robot-handler-prod}"' in source
    assert "--pull=never" in source
