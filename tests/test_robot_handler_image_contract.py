from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_robot_handler_image_installs_the_frozen_application_requirements():
    dockerfile = (ROOT / "Dockerfile.robot-handler").read_text(encoding="utf-8")
    requirements = {
        line.strip().lower()
        for line in (ROOT / "requirements.txt").read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    }

    assert "COPY requirements.txt /app/requirements.txt" in dockerfile
    assert "pip install --no-cache-dir --requirement /app/requirements.txt" in dockerfile
    assert "pillow" in requirements
    assert "RUN pip install --no-cache-dir \\\n      fastapi" not in dockerfile
