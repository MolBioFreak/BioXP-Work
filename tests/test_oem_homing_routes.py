
import asyncio
from pathlib import Path

from bioxp.api import app
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


def _route(path: str, method: str):
    for route in app.routes:
        candidates = [route]
        effective_route_contexts = getattr(route, "effective_route_contexts", None)
        if callable(effective_route_contexts):
            candidates += list(effective_route_contexts())  # type: ignore[misc]
        for candidate in candidates:
            if getattr(candidate, "path", None) == path and method in (getattr(candidate, "methods", None) or set()):
                return candidate
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



def test_machine_config_route_is_registered_and_read_only(monkeypatch):
    bind_serial206_oem_snapshot(monkeypatch)
    endpoint = _route("/motion/oem/machine_config", "GET").endpoint

    body = asyncio.run(endpoint())

    assert body["ok"] is True
    assert body["accepted_live_mode"] is True
    assert body["runtime_binding"] == "read_only_immutable_evidence"
    assert body["opened_usb"] is False
    assert body["physical_motion"] is False
    assert body["motion_commanded"] is False
    assert body["config"]["server_redacted"] == {}
    assert body["config"]["serial_redacted"] == "[REDACTED]"
    assert body["config"]["axis_limits"]["x"]["max_steps"] == 90263
    assert body["snapshot_status"]["serial"] == 206
    assert body["snapshot_status"]["mutation_authorized"] is False
