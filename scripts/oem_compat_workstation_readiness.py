#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from fastapi.testclient import TestClient

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from src.bioxp.oem_compat.api import router


def _run(cmd: list[str], cwd: Path) -> dict:
    proc = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)
    return {"cmd": cmd, "returncode": proc.returncode, "stdout": proc.stdout[-4000:], "stderr": proc.stderr[-4000:]}


def main() -> int:
    parser = argparse.ArgumentParser(description="BioXP OEM compatibility workstation dry-run readiness check")
    parser.add_argument("--output", required=True, help="Path to JSON readiness report")
    parser.add_argument("--skip-pytest", action="store_true", help="Skip pytest inside this script; caller may run tests separately")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    out = Path(args.output).expanduser().resolve()
    artifact_dir = out.parent / "oem_compat_artifacts"
    artifact_dir.mkdir(parents=True, exist_ok=True)
    startup_trace = artifact_dir / "startup_trace.json"

    from fastapi import FastAPI

    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)
    startup_resp = client.post("/oem-compat/startup/dry-run", json={"run_homing": True, "artifact_path": str(startup_trace)})
    script_resp = client.post(
        "/oem-compat/scripts/translate/dry-run",
        json={
            "xml": """<WpfGenBotCommonLib><script><line1 cmd=\"LED 0 0 0\"/><line2 cmd=\"WAIT 1\"/><line3 cmd=\"TCD OPEN\"/></script></WpfGenBotCommonLib>"""
        },
    )

    checks = {
        "py_compile": _run([sys.executable, "-m", "py_compile", *[str(p) for p in Path("src/bioxp/oem_compat").glob("*.py")], "src/bioxp/api.py"], repo),
        "pytest": None,
    }
    if not args.skip_pytest:
        checks["pytest"] = _run(
            [
                "uv",
                "run",
                "--with",
                "fastapi",
                "--with",
                "httpx",
                "--with",
                "pydantic",
                "--with",
                "starlette",
                "--with",
                "pyusb",
                "--with",
                "pytest",
                "python",
                "-m",
                "pytest",
                "tests/test_oem_compat_phases.py",
                "tests/test_oem_compat_api.py",
                "tests/test_oem_compat_end_to_end.py",
                "tests/test_oem_compat_transport.py",
                "tests/test_oem_compat_workstation_finish.py",
                "-q",
            ],
            repo,
        )

    startup_body = startup_resp.json()
    script_body = script_resp.json()
    ok = (
        startup_resp.status_code == 200
        and script_resp.status_code == 200
        and startup_body.get("replay_ok") is True
        and startup_body.get("physical_motion") is False
        and script_body.get("executed") is False
        and checks["py_compile"]["returncode"] == 0
        and (args.skip_pytest or (checks["pytest"] and checks["pytest"]["returncode"] == 0))
    )
    report = {
        "ok": ok,
        "mode": "workstation_dry_run",
        "startup_trace": {
            "artifact_path": startup_body.get("artifact_path"),
            "replay_ok": startup_body.get("replay_ok"),
            "frame_count": startup_body.get("frame_count"),
            "physical_motion": startup_body.get("physical_motion"),
            "opened_usb": startup_body.get("opened_usb"),
        },
        "script_translate": {"status_code": script_resp.status_code, "executed": script_body.get("executed"), "action_count": len(script_body.get("actions", []))},
        "checks": checks,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
