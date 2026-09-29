"""Static reference and retained input tests; never run operational scripts."""
from pathlib import Path
import ast
import hashlib
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
RETIRED = (
    "bioxp_robot_container_run.sh", "bioxp_robot_container_watchdog.sh",
    "bioxp_robot_container_smoke.sh", "bioxp_supervised_home_axis.sh",
    "bioxp_supervised_relative_move.sh", "bioxp_supervised_strict_startup.sh",
    "bioxp_supervised_oem_app_startup.sh", "bioxp_motion_readiness_snapshot.sh",
    "generate_bioxp_runtime_audit_entrypoint_denominator.py", "led_cycle.py",
    "github_bootstrap.sh", "bioxp_latency_microbench.py",
    "oem_compat_workstation_readiness.py",
)


def test_retired_entrypoints_have_no_surviving_executable_callers():
    for name in RETIRED:
        assert not (ROOT / "scripts" / name).exists()
    for directory in (ROOT / "scripts", ROOT / "release", ROOT / ".github"):
        if not directory.exists():
            continue
        for path in directory.rglob("*"):
            if path.is_file() and path.suffix in {".py", ".sh", ".yml", ".yaml"}:
                text = path.read_text()
                assert not any(name in text for name in RETIRED), path


@pytest.mark.parametrize("name,digest", [
    ("demo.xml", "b5f7c159245cbd6516c89587795a4021d4a3e1a5f71e8e4eead46a243a38b763"),
    ("lifetest.xml", "454a235d932b79cd01ce81bb4bbb8a4ba061c3c2e41c1e1bbb743f1adbfbaba8"),
])
def test_xml_fixture_bytes_survive_duplicate_removal(name, digest):
    assert not (ROOT / "scripts" / name).exists()
    assert hashlib.sha256((ROOT / "testdata/oem_xml" / name).read_bytes()).hexdigest() == digest


def test_surviving_scripts_parse_without_execution():
    for path in (ROOT / "scripts").glob("*.py"):
        ast.parse(path.read_text(), filename=str(path))
    for path in (ROOT / "scripts").glob("*.sh"):
        subprocess.run(["bash", "-n", str(path)], check=True, capture_output=True)


def test_manual_recovery_and_observation_routes_still_exist():
    from bioxp.api import app

    paths = app.openapi()["paths"]
    for path in ("/motion/axis/home", "/motion/axis/relative", "/motion/arm/strict_startup"):
        assert "post" in paths[path]
    for path in ("/status", "/motion/power/status", "/latch/status", "/motion/axes/status"):
        assert "get" in paths[path]
