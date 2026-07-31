from __future__ import annotations

import json
import os
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "bioxp_supervised_initialize_motion_diagnostic.sh"
PASS_RESPONSE = {
    "ok": True,
    "result": {"physical_motion_commanded": False},
}


def _run_launcher(
    tmp_path: Path,
    *,
    response: dict = PASS_RESPONSE,
    post_status: int = 200,
    operator_ack: str | None = "INITIALIZE",
) -> tuple[subprocess.CompletedProcess[str], list[tuple[str, str, dict | None]], list[Path]]:
    requests: list[tuple[str, str, dict | None]] = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: object) -> None:
            return

        def _send(self, payload: dict, *, status: int = 200) -> None:
            body = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            requests.append(("GET", self.path, None))
            self._send({"ok": True, "path": self.path})

        def do_POST(self) -> None:  # noqa: N802
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            requests.append(("POST", self.path, body))
            self._send(response, status=post_status)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        env = {
            **os.environ,
            "BIOXP_BASE_URL": f"http://127.0.0.1:{server.server_port}",
            "BIOXP_LOG_ROOT": str(tmp_path),
        }
        if operator_ack is not None:
            env["BIOXP_OPERATOR_ACK"] = operator_ack
        result = subprocess.run(
            ["bash", str(SCRIPT)],
            text=True,
            input="\n" if operator_ack is None else None,
            capture_output=True,
            check=False,
            env=env,
            timeout=20,
        )
    finally:
        server.shutdown()
        thread.join(timeout=5)
    return result, requests, sorted(tmp_path.iterdir())


def _artifact_root(entries: list[Path]) -> Path:
    assert len(entries) == 1
    return entries[0]


def test_diagnostic_launcher_artifacts_real_api_envelope_and_forces_no_homing(tmp_path: Path) -> None:
    result, requests, entries = _run_launcher(tmp_path)

    assert result.returncode == 0, result.stderr
    assert "PASS: no-homing diagnostic artifacted" in result.stdout
    assert [path for method, path, _ in requests if method == "POST"] == ["/motion/oem/initialize_motion"]
    assert [method for method, _, _ in requests].count("POST") == 1
    assert [path for method, path, _ in requests if method == "GET"] == [
        "/status",
        "/motion/power/status",
        "/latch/status",
        "/motion/axes/status?axes=x,y,z,g,door",
    ] * 2
    payload = [body for method, _, body in requests if method == "POST"][0]
    assert payload == {
        "operator_ack": "INITIALIZE",
        "run_homing": False,
        "include_tip_pipette_cleanup": False,
        "timeout_s": 90,
    }

    root = _artifact_root(entries)
    assert json.loads((root / "initialize_motion_response.json").read_text()) == PASS_RESPONSE
    assert json.loads((root / "summary.json").read_text()) == {
        "mode": "supervised_no_homing_diagnostic",
        "ok": True,
        "physical_motion_commanded": False,
        "request": "initialize_motion_request.json",
        "response": "initialize_motion_response.json",
    }


@pytest.mark.parametrize(
    "response",
    [
        {"ok": False, "result": {"physical_motion_commanded": False}},
        {"ok": True},
        {"ok": True, "result": {"physical_motion_commanded": "false"}},
        {"ok": True, "result": {"physical_motion_commanded": None}},
    ],
)
def test_diagnostic_launcher_refuses_invalid_or_ambiguous_motion_evidence(tmp_path: Path, response: dict) -> None:
    result, requests, entries = _run_launcher(tmp_path, response=response)

    assert result.returncode != 0
    assert "PASS:" not in result.stdout
    assert [path for method, path, _ in requests if method == "POST"] == ["/motion/oem/initialize_motion"]
    assert len([request for request in requests if request[0] == "GET"]) == 8
    root = _artifact_root(entries)
    assert (root / "initialize_motion_response.json").exists()
    assert not (root / "summary.json").exists()


def test_diagnostic_launcher_preserves_post_snapshot_when_api_returns_http_failure(tmp_path: Path) -> None:
    result, requests, entries = _run_launcher(
        tmp_path,
        response={"detail": "forced failure"},
        post_status=503,
    )

    assert result.returncode != 0
    assert "PASS:" not in result.stdout
    assert [path for method, path, _ in requests if method == "POST"] == ["/motion/oem/initialize_motion"]
    assert len([request for request in requests if request[0] == "GET"]) == 8
    root = _artifact_root(entries)
    assert json.loads((root / "initialize_motion_response.json").read_text()) == {"detail": "forced failure"}
    assert (root / "post_status.json").exists()


@pytest.mark.parametrize("operator_ack", ["NOPE", None])
def test_diagnostic_launcher_refuses_absent_or_wrong_ack_before_any_request(tmp_path: Path, operator_ack: str | None) -> None:
    result, requests, entries = _run_launcher(tmp_path, operator_ack=operator_ack)

    assert result.returncode != 0
    assert "PASS:" not in result.stdout
    assert requests == []
    assert entries == []
