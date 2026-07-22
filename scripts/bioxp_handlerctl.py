#!/usr/bin/env python3
"""Safe lifecycle controller for the canonical BioXP root systemd handler."""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

UNIT = "bioxp-api.service"
RECOVERY_UNIT = "bioxp-api-recovery.service"
BASE_URL = "http://127.0.0.1:8123"
RECOVERY_COMMAND = [
    "systemd-run",
    "--user",
    f"--unit={RECOVERY_UNIT.removesuffix('.service')}",
    "--property=WorkingDirectory=/home/molbiofreak/bioxp_re",
    "--setenv=PYTHONPATH=src",
    "/home/molbiofreak/bioxp_re/.venv/bin/uvicorn",
    "bioxp.api:app",
    "--host",
    "0.0.0.0",
    "--port",
    "8123",
]


def run(*args: str, check: bool = True, timeout: float = 45.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, text=True, capture_output=True, check=check, timeout=timeout)


def systemctl(*args: str, check: bool = True, timeout: float = 45.0) -> subprocess.CompletedProcess[str]:
    return run("systemctl", *args, check=check, timeout=timeout)


def user_systemctl(*args: str, check: bool = True, timeout: float = 45.0) -> subprocess.CompletedProcess[str]:
    return run("systemctl", "--user", *args, check=check, timeout=timeout)


def request(path: str, *, method: str = "GET", timeout: float = 15.0) -> Any:
    payload = b"{}" if method == "POST" else None
    req = urllib.request.Request(
        BASE_URL + path,
        data=payload,
        method=method,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=timeout) as response:
        return json.load(response)


def get_status(required: bool = False) -> dict[str, Any] | None:
    try:
        value = request("/status", timeout=5.0)
        return value if isinstance(value, dict) else None
    except (OSError, urllib.error.URLError, json.JSONDecodeError):
        if required:
            raise RuntimeError("BioXP status endpoint is not reachable")
        return None


def running_stages(status: dict[str, Any] | None) -> list[str]:
    if not status:
        return []
    startup = status.get("startup")
    stages = startup.get("stages") if isinstance(startup, dict) else None
    if not isinstance(stages, dict):
        return []
    return [str(name) for name, stage in stages.items() if isinstance(stage, dict) and stage.get("state") == "running"]


def refuse_running(status: dict[str, Any] | None, *, force: bool) -> None:
    stages = running_stages(status)
    if stages and not force:
        raise RuntimeError(f"refusing lifecycle mutation while stages are running: {stages}")


def port_open() -> bool:
    try:
        with socket.create_connection(("127.0.0.1", 8123), timeout=0.5):
            return True
    except OSError:
        return False


def wait_port(expected: bool, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if port_open() is expected:
            return
        time.sleep(0.25)
    raise RuntimeError(f"port 8123 did not become {'open' if expected else 'free'} within {timeout}s")


def root_unit_details() -> dict[str, Any]:
    fields = ["ActiveState", "SubState", "MainPID", "ExecMainStartTimestamp", "Restart", "FragmentPath", "DropInPaths"]
    cp = systemctl("show", UNIT, *[f"-p{x}" for x in fields])
    details: dict[str, Any] = {}
    for line in cp.stdout.splitlines():
        key, _, value = line.partition("=")
        details[key] = int(value) if key == "MainPID" and value.isdigit() else value
    pid = int(details.get("MainPID") or 0)
    if pid:
        cgroup = Path(f"/proc/{pid}/cgroup")
        details["main_pid_cgroup"] = cgroup.read_text().strip() if cgroup.exists() else None
    return details


def verify_root_owner() -> dict[str, Any]:
    details = root_unit_details()
    if details.get("ActiveState") != "active" or details.get("SubState") != "running":
        raise RuntimeError(f"canonical unit is not active/running: {details}")
    if "bioxp-api.service" not in str(details.get("main_pid_cgroup") or ""):
        raise RuntimeError(f"MainPID is not in the canonical unit cgroup: {details}")
    wait_port(True, 45.0)
    return details


def reclaim_and_snapshot() -> dict[str, Any]:
    reconnect = request("/maintenance/usb/reconnect", method="POST", timeout=60.0)
    snapshot = request("/hardware/snapshot/collect", method="POST", timeout=240.0)
    status = get_status(required=True)
    ownership = status.get("ownership") if isinstance(status, dict) else None
    if not isinstance(ownership, dict) or ownership.get("usb") != "service" or ownership.get("router") != "running" or ownership.get("CAN_READY") is not True:
        raise RuntimeError(f"handler started but hardware ownership is not ready: {ownership}")
    return {"reconnect": reconnect, "snapshot": snapshot, "status": status}


def rollback_recovery() -> dict[str, Any]:
    user_systemctl("reset-failed", RECOVERY_UNIT, check=False)
    cp = run(*RECOVERY_COMMAND, check=False, timeout=30.0)
    if cp.returncode != 0:
        return {"ok": False, "stderr": cp.stderr.strip(), "stdout": cp.stdout.strip()}
    wait_port(True, 45.0)
    return {"ok": True, "stdout": cp.stdout.strip()}


def handoff(*, force: bool) -> dict[str, Any]:
    before_status = get_status()
    refuse_running(before_status, force=force)
    # Harmless authorization preflight before surrendering the working listener.
    systemctl("reset-failed", UNIT)
    recovery_was_active = user_systemctl("is-active", RECOVERY_UNIT, check=False).stdout.strip() == "active"
    if recovery_was_active:
        user_systemctl("stop", RECOVERY_UNIT, timeout=30.0)
        wait_port(False, 30.0)
    try:
        systemctl("start", UNIT, timeout=45.0)
        details = verify_root_owner()
        hardware = reclaim_and_snapshot()
    except Exception:
        systemctl("stop", UNIT, check=False, timeout=30.0)
        if recovery_was_active:
            rollback = rollback_recovery()
            print(json.dumps({"handoff": "failed", "recovery_rollback": rollback}, indent=2), file=sys.stderr)
        raise
    return {"action": "handoff", "root_unit": details, "hardware": hardware}


def start_or_restart(action: str, *, force: bool) -> dict[str, Any]:
    before = get_status()
    refuse_running(before, force=force)
    if user_systemctl("is-active", RECOVERY_UNIT, check=False).stdout.strip() == "active":
        raise RuntimeError("temporary recovery unit is active; use 'handoff' to preserve single ownership")
    systemctl(action, UNIT, timeout=45.0)
    details = verify_root_owner()
    hardware = reclaim_and_snapshot()
    return {"action": action, "root_unit": details, "hardware": hardware}


def stop(*, force: bool) -> dict[str, Any]:
    status = get_status()
    refuse_running(status, force=force)
    systemctl("stop", UNIT, timeout=45.0)
    wait_port(False, 30.0)
    return {"action": "stop", "root_unit": root_unit_details()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("status", "handoff", "start", "restart", "stop", "reset-failed"))
    parser.add_argument("--recover-stuck", action="store_true", help="permit termination despite a running lifecycle stage")
    args = parser.parse_args()
    if os.geteuid() == 0:
        raise RuntimeError("run handlerctl as molbiofreak, never as root")
    if args.action == "status":
        result = {"root_unit": root_unit_details(), "recovery_active": user_systemctl("is-active", RECOVERY_UNIT, check=False).stdout.strip(), "api": get_status()}
    elif args.action == "handoff":
        result = handoff(force=args.recover_stuck)
    elif args.action in ("start", "restart"):
        result = start_or_restart(args.action, force=args.recover_stuck)
    elif args.action == "stop":
        result = stop(force=args.recover_stuck)
    else:
        systemctl("reset-failed", UNIT)
        result = {"action": "reset-failed", "root_unit": root_unit_details()}
    print(json.dumps(result, indent=2, default=str))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, indent=2), file=sys.stderr)
        raise SystemExit(1)
