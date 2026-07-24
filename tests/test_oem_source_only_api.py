from __future__ import annotations

import asyncio


def test_source_only_app_exposes_only_fresh_program_discovery_and_dry_run():
    from src.bioxp.oem_source_only_api import create_oem_source_only_app

    app = create_oem_source_only_app()
    routes = {(route.path, tuple(sorted(route.methods or ()))) for route in app.routes}

    assert ("/motion/oem/programs", ("GET",)) in routes
    assert ("/motion/oem/programs/{program_name}", ("GET",)) in routes
    assert ("/motion/oem/{program_name}/dry_run", ("POST",)) in routes
    assert not any(path.startswith("/motion/axis/") for path, _methods in routes)
    assert not any(path.startswith("/oem/runtime/") for path, _methods in routes)
    assert not any("scriptmove_execute" in path for path, _methods in routes)


def test_source_only_app_dry_run_executes_fake_trace_and_never_opens_usb():
    from src.bioxp.oem_source_only_api import dry_run_program

    payload = asyncio.run(
        dry_run_program(
            "initialize_motors",
            {"simulation": {"serial_number": 206, "camera_calibrated": True, "tc_door_closed": False}},
        )
    )

    assert payload["ok"] is False
    assert payload["failed_closed"] is True
    assert payload["failure"]["source_anchor"].endswith(":3387")
    assert payload["opened_usb"] is False
    assert payload["physical_motion"] is False
