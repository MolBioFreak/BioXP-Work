
import asyncio
from pathlib import Path

from bioxp.api import app


def _route(path, method):
    for route in app.routes:
        if getattr(route, "path", None) == path and method in getattr(route, "methods", set()):
            return route
    raise AssertionError(f"missing route {method} {path}")


def test_program_listing_routes_are_registered_and_dry_run_only():
    assert _route("/motion/oem/programs", "GET")
    assert _route("/motion/oem/programs/{program_name}", "GET")
    assert _route("/motion/oem/{program_name}/dry_run", "POST")
    assert not any(getattr(r, "path", "") == "/motion/oem/{program_name}/live" for r in app.routes)


def test_program_listing_endpoint_returns_no_motion_payload():
    endpoint = _route("/motion/oem/programs", "GET").endpoint
    body = asyncio.run(endpoint())
    assert body["ok"] is True
    names = {p["name"] for p in body["programs"]}
    assert "initialize_motors" in names
    assert "home_gz" in names
    assert all(p["live_allowed_default"] is False for p in body["programs"])
    assert body["opened_usb"] is False
    assert body["physical_motion"] is False


def test_dry_run_endpoint_returns_artifact_shape(tmp_path):
    endpoint = _route("/motion/oem/{program_name}/dry_run", "POST").endpoint
    body = asyncio.run(endpoint("initialize_motors", {"artifact_root": str(tmp_path)}))
    assert body["program"] == "initialize_motors"
    assert body["mode"] == "dry_run"
    assert body["opened_usb"] is False
    assert body["physical_motion"] is False
    assert body["artifact_path"]
    assert Path(body["artifact_path"]).exists()
