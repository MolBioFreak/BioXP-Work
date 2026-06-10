import asyncio

from src.bioxp.oem_compat.movement_readiness import build_movement_readiness_comparison
from src.bioxp.oem_homing_routes import get_oem_movement_readiness_comparison


def test_movement_readiness_comparison_is_no_motion_and_reports_executor_available():
    payload = build_movement_readiness_comparison()
    assert payload["opened_usb"] is False
    assert payload["physical_motion"] is False
    assert payload["motion_commanded"] is False
    assert payload["summary"]["live_oem_path_execution"] == "guarded_live_executor_available"
    assert payload["summary"]["hard_gap_count"] == 0
    adapter = next(row for row in payload["matrix"] if row["layer"] == "Live execution adapter")
    assert adapter["gap"] is None
    assert adapter["new_system_status"] == "implemented_guarded_live_executor"
    assert any("operator or camera observation" in item for item in payload["movement_test_start_policy"])


def test_movement_readiness_route_is_read_only():
    payload = asyncio.run(get_oem_movement_readiness_comparison())
    assert payload["schema_version"] == "bioxp.oem_movement_readiness_comparison.v1"
    assert payload["motion_commanded"] is False
    assert "GET /motion/power/status" in payload["required_pre_move_live_checks"]
