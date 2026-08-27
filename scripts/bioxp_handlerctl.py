#!/usr/bin/env python3
"""Safe lifecycle controller for the canonical BioXP root systemd handler."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import socket
import sqlite3
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

UNIT = "bioxp-api.service"
BASE_URL = "http://127.0.0.1:8123"
UNIT_PATH = Path("/etc/systemd/system/bioxp-api.service")
LAUNCHER_PATH = Path("/usr/local/libexec/bioxp-release-container-run")
RECEIPT_PATH = Path("/etc/bioxp/release-identity.json")
RUNTIME_BINDING_PATH = Path("/run/bioxp-release/runtime-binding.json")
RUNTIME_DATABASE = Path("/var/lib/bioxp-oem-runtime/bioxp_runtime.db")


class HandlerControlError(RuntimeError):
    def __init__(self, message: str, receipt: dict[str, Any] | None = None):
        super().__init__(message)
        self.receipt = dict(receipt or {})


def run(*args: str, check: bool = True, timeout: float = 45.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, text=True, capture_output=True, check=check, timeout=timeout)


def systemctl(*args: str, check: bool = True, timeout: float = 45.0) -> subprocess.CompletedProcess[str]:
    return run("systemctl", *args, check=check, timeout=timeout)


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


def _unit_active_or_transitioning(details: dict[str, Any]) -> bool:
    return str(details.get("ActiveState") or "") in {
        "active", "activating", "deactivating", "reloading",
    } or str(details.get("SubState") or "") in {
        "start-pre", "start", "start-post", "running", "reload", "stop", "stop-sigterm",
        "stop-sigkill", "auto-restart",
    }


def refuse_lifecycle_mutation(
    status: dict[str, Any] | None,
    details: dict[str, Any],
    *,
    listener_open: bool,
    recover_stuck: bool,
) -> None:
    if recover_stuck:
        return
    operation_state = status.get("operation_state") if isinstance(status, dict) else None
    stages = running_stages(status)
    if operation_state in {"waiting", "running", "paused", "emergency"}:
        raise RuntimeError(f"refusing lifecycle mutation while operation state is {operation_state}")
    if stages:
        raise RuntimeError(f"refusing lifecycle mutation while stages are running: {stages}")
    status_complete = isinstance(operation_state, str) and isinstance(status.get("startup"), dict) if isinstance(status, dict) else False
    if not status_complete and (_unit_active_or_transitioning(details) or listener_open):
        raise RuntimeError(
            "refusing lifecycle mutation because /status is unavailable or incomplete while the unit/listener is active"
        )


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


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_loaded_authority(details: dict[str, Any]) -> dict[str, Any]:
    receipt = json.loads(RECEIPT_PATH.read_bytes())
    binding = receipt.get("binding") if isinstance(receipt, dict) else None
    if not isinstance(receipt, dict) or receipt.get("schema") != "bioxp.release.identity.v1" or receipt.get("status") != "verified" or not isinstance(binding, dict):
        raise RuntimeError("canonical verified release receipt is unavailable")
    if details.get("FragmentPath") != str(UNIT_PATH):
        raise RuntimeError(f"loaded FragmentPath is not canonical: {details.get('FragmentPath')}")
    if str(details.get("DropInPaths") or "").strip():
        raise RuntimeError(f"loaded unit has forbidden DropInPaths: {details.get('DropInPaths')}")
    exec_fields: dict[str, str] = {}
    for part in str(details.get("ExecStart") or "").strip().strip("{}").split(";"):
        key, separator, value = part.strip().partition("=")
        if separator:
            exec_fields[key.strip()] = value.strip()
    if exec_fields.get("path") != str(LAUNCHER_PATH) or exec_fields.get("argv[]") != str(LAUNCHER_PATH):
        raise RuntimeError(f"loaded ExecStart is not the exact canonical launcher: {details.get('ExecStart')}")
    if binding.get("unit_path") != str(UNIT_PATH) or binding.get("launcher_path") != str(LAUNCHER_PATH):
        raise RuntimeError("release receipt loaded unit/launcher paths are contradictory")
    if _sha256_file(UNIT_PATH) != binding.get("unit_sha256"):
        raise RuntimeError("loaded canonical unit digest contradicts release receipt")
    if _sha256_file(LAUNCHER_PATH) != binding.get("launcher_sha256"):
        raise RuntimeError("loaded canonical launcher digest contradicts release receipt")
    return receipt


def root_unit_details() -> dict[str, Any]:
    fields = ["ActiveState", "SubState", "MainPID", "ExecMainStartTimestamp", "InvocationID", "Restart", "FragmentPath", "DropInPaths", "ExecStart"]
    cp = systemctl("show", UNIT, *[f"-p{x}" for x in fields])
    details: dict[str, Any] = {}
    for line in cp.stdout.splitlines():
        key, _, value = line.partition("=")
        details[key] = int(value) if key == "MainPID" and value.isdigit() else value
    pid = int(details.get("MainPID") or 0)
    if pid:
        cgroup = Path(f"/proc/{pid}/cgroup")
        details["main_pid_cgroup"] = cgroup.read_text().strip() if cgroup.exists() else None
    try:
        details["loaded_release_receipt"] = verify_loaded_authority(details)
    except Exception as exc:
        raise HandlerControlError(
            str(exc),
            {"root_unit": details, "authority_stage": "loaded_unit_release_verification"},
        ) from exc
    return details


def _live_listener_for_pid(pid: int, expected_host: str, expected_port: int) -> dict[str, Any]:
    socket_inodes: set[str] = set()
    for descriptor in Path(f"/proc/{pid}/fd").iterdir():
        try:
            target = os.readlink(descriptor)
        except OSError:
            continue
        if target.startswith("socket:[") and target.endswith("]"):
            socket_inodes.add(target[8:-1])
    matches: list[dict[str, Any]] = []
    for table_path in (Path("/proc/net/tcp"), Path("/proc/net/tcp6")):
        for line in table_path.read_text(encoding="utf-8").splitlines()[1:]:
            fields = line.split()
            if len(fields) < 10 or fields[3] != "0A":
                continue
            address, separator, port_hex = fields[1].rpartition(":")
            inode = fields[9]
            if not separator or inode not in socket_inodes or int(port_hex, 16) != expected_port:
                continue
            raw = bytes.fromhex(address)
            if table_path.name == "tcp":
                host = socket.inet_ntop(socket.AF_INET, raw[::-1])
            else:
                host = socket.inet_ntop(
                    socket.AF_INET6,
                    b"".join(raw[index : index + 4][::-1] for index in range(0, 16, 4)),
                )
            if host == expected_host:
                matches.append({"host": host, "port": expected_port, "socket_inode": int(inode), "owner_pid": pid})
    if len(matches) != 1:
        raise RuntimeError(f"canonical listener ownership is not unique for MainPID {pid}: {matches}")
    return matches[0]


def _verify_process_identity(pid: int, release_receipt: dict[str, Any]) -> dict[str, Any]:
    binding = release_receipt.get("binding")
    configuration = binding.get("configuration") if isinstance(binding, dict) else None
    expected_argv = configuration.get("argv") if isinstance(configuration, dict) else None
    if not isinstance(expected_argv, list) or not expected_argv or not all(isinstance(item, str) for item in expected_argv):
        raise RuntimeError("canonical release receipt has no exact process argv")
    executable = os.readlink(f"/proc/{pid}/exe")
    cmdline = [part.decode("utf-8") for part in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0") if part]
    if (
        not Path(executable).name.startswith("python")
        or not cmdline
        or not Path(cmdline[0]).name.startswith("python")
        or cmdline[1:] != expected_argv[1:]
    ):
        raise RuntimeError(
            f"MainPID executable/cmdline contradicts the release configuration: executable={executable!r}, cmdline={cmdline!r}"
        )
    try:
        runtime_binding = json.loads(RUNTIME_BINDING_PATH.read_bytes())
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError("launch-time runtime binding is unavailable or malformed") from exc
    source = release_receipt.get("source")
    image = release_receipt.get("image")
    expected_environment = {
        "BIOXP_RELEASE_ID": release_receipt.get("release_id"),
        "BIOXP_RELEASE_IMAGE_ID": image.get("id") if isinstance(image, dict) else None,
        "BIOXP_RELEASE_SOURCE_COMMIT": source.get("commit") if isinstance(source, dict) else None,
        "BIOXP_RELEASE_UDOCKER_SHA256": runtime_binding.get("udocker_sha256") if isinstance(runtime_binding, dict) else None,
        "BIOXP_RELEASE_UDOCKER_TREE_SHA256": runtime_binding.get("udocker_tree_sha256") if isinstance(runtime_binding, dict) else None,
    }
    if (
        not isinstance(runtime_binding, dict)
        or runtime_binding.get("schema") != "bioxp.release.runtime_binding.v1"
        or runtime_binding.get("status") != "verified"
        or runtime_binding.get("release_id") != expected_environment["BIOXP_RELEASE_ID"]
        or runtime_binding.get("image_id") != expected_environment["BIOXP_RELEASE_IMAGE_ID"]
        or runtime_binding.get("udocker_path") != "/opt/bioxp/udocker-runtime/venv/bin/udocker"
    ):
        raise RuntimeError("launch-time runtime binding contradicts the installed release identity")
    environment = {}
    for entry in Path(f"/proc/{pid}/environ").read_bytes().split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        environment[key.decode("utf-8")] = value.decode("utf-8")
    observed_release_environment = {
        key: environment.get(key)
        for key in expected_environment
    }
    if observed_release_environment != expected_environment:
        raise RuntimeError(
            "MainPID release environment does not match the launch-time image/runtime binding"
        )
    return {
        "executable": executable,
        "cmdline": cmdline,
        "release_environment": observed_release_environment,
        "runtime_binding_sha256": hashlib.sha256(RUNTIME_BINDING_PATH.read_bytes()).hexdigest(),
    }


def _verify_status_runtime_identity(
    status: dict[str, Any],
    receipt: dict[str, Any],
) -> None:
    identity = status.get("runtime_identity")
    deployment = identity.get("deployment") if isinstance(identity, dict) else None
    binding = identity.get("binding") if isinstance(identity, dict) else None
    observed = receipt.get("observed_listener")
    public_listener = (
        {"host": observed.get("host"), "port": observed.get("port")}
        if isinstance(observed, dict)
        else None
    )
    if (
        not isinstance(identity, dict)
        or identity.get("verified") is not True
        or identity.get("release_id") != receipt.get("release_id")
        or not isinstance(deployment, dict)
        or deployment.get("receipt_id") != receipt.get("deployment_receipt_id")
        or not isinstance(binding, dict)
        or binding.get("observed_listener") != public_listener
        or binding.get("udocker_sha256") != receipt.get("udocker_sha256")
        or binding.get("udocker_tree_sha256") != receipt.get("udocker_tree_sha256")
    ):
        raise RuntimeError("/status public runtime identity contradicts the verified private release-start receipt")


def verify_runtime_receipt(details: dict[str, Any], status: dict[str, Any]) -> dict[str, Any]:
    pid = int(details.get("MainPID") or 0)
    invocation_id = str(details.get("InvocationID") or "")
    if pid <= 1 or not invocation_id:
        raise RuntimeError("canonical systemd runtime identity is incomplete")
    connection = sqlite3.connect(f"file:{RUNTIME_DATABASE}?mode=ro", uri=True)
    connection.row_factory = sqlite3.Row
    try:
        row = connection.execute(
            "SELECT receipt_json,receipt_sha256 FROM runtime_release_receipts "
            "ORDER BY recorded_at DESC,receipt_id DESC LIMIT 1"
        ).fetchone()
    finally:
        connection.close()
    if row is None:
        raise RuntimeError("durable runtime release-start receipt is unavailable")
    receipt = json.loads(str(row["receipt_json"]))
    observed = receipt.get("observed_listener") if isinstance(receipt, dict) else None
    unsigned_receipt = dict(receipt) if isinstance(receipt, dict) else {}
    embedded_sha256 = unsigned_receipt.pop("receipt_sha256", None)
    calculated_sha256 = hashlib.sha256(
        json.dumps(unsigned_receipt, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    if (
        receipt.get("schema") != "bioxp.runtime.release_start.v1"
        or embedded_sha256 != str(row["receipt_sha256"])
        or calculated_sha256 != embedded_sha256
        or receipt.get("application_pid") != pid
        or receipt.get("systemd_invocation_id") != invocation_id
        or not isinstance(observed, dict)
        or observed.get("host") != "0.0.0.0"
        or observed.get("port") != 8123
        or observed.get("owner_pid") != pid
    ):
        raise RuntimeError("durable runtime receipt contradicts the loaded service owner")
    process_stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    close_paren = process_stat.rfind(")")
    if (
        close_paren < 0
        or int(process_stat[close_paren + 2 :].split()[19])
        != receipt.get("application_start_time_ticks")
    ):
        raise RuntimeError("durable runtime receipt process-start identity is stale")
    cgroup = str(details.get("main_pid_cgroup") or "").strip()
    cgroup_sha256 = hashlib.sha256((cgroup + "\n").encode("utf-8")).hexdigest()
    if receipt.get("application_cgroup_sha256") != cgroup_sha256:
        raise RuntimeError("durable runtime receipt cgroup identity is stale")
    if receipt.get("canonical_receipt_sha256") != _sha256_file(RECEIPT_PATH):
        raise RuntimeError("durable runtime receipt does not bind the installed release receipt")
    release_receipt = json.loads(RECEIPT_PATH.read_bytes())
    process_identity = _verify_process_identity(pid, release_receipt)
    live_listener = _live_listener_for_pid(pid, str(observed["host"]), int(observed["port"]))
    if live_listener.get("socket_inode") != observed.get("socket_inode"):
        raise RuntimeError("durable runtime receipt socket inode is not owned by the live MainPID listener")
    _verify_status_runtime_identity(status, receipt)
    return {**receipt, "verified_process": process_identity, "verified_live_listener": live_listener}


def verify_root_owner() -> dict[str, Any]:
    details = root_unit_details()
    if details.get("ActiveState") != "active" or details.get("SubState") != "running":
        raise RuntimeError(f"canonical unit is not active/running: {details}")
    if "bioxp-api.service" not in str(details.get("main_pid_cgroup") or ""):
        raise RuntimeError(f"MainPID is not in the canonical unit cgroup: {details}")
    wait_port(True, 45.0)
    status = get_status(required=True)
    if not isinstance(status, dict):
        raise RuntimeError("BioXP status endpoint did not return an object")
    details["runtime_release_receipt"] = verify_runtime_receipt(details, status)
    details["runtime_status_identity"] = status.get("runtime_identity")
    return details


def pre_stop_proof(*, recover_stuck: bool, recovery_reason: str | None) -> dict[str, Any]:
    """Capture one authoritative pre-stop proof for stop and restart."""
    details = root_unit_details()
    listener_open = port_open()
    status = get_status()
    proof = {
        "loaded_root_unit": details,
        "listener_open": listener_open,
        "api_status": status,
        "live_runtime_identity": None,
        "live_runtime_identity_error": None,
        "recovery": {"recover_stuck": recover_stuck, "reason": recovery_reason},
    }
    try:
        refuse_lifecycle_mutation(
            status,
            details,
            listener_open=listener_open,
            recover_stuck=recover_stuck,
        )
    except Exception as exc:
        raise HandlerControlError(
            str(exc),
            {"pre_stop_proof": proof, "failure_stage": "lifecycle_refusal"},
        ) from exc
    live_identity: dict[str, Any] | None = None
    live_identity_error: str | None = None
    if _unit_active_or_transitioning(details) or listener_open:
        try:
            live_identity = verify_root_owner()
        except Exception as exc:
            live_identity_error = f"{type(exc).__name__}: {exc}"
    proof["live_runtime_identity"] = live_identity
    proof["live_runtime_identity_error"] = live_identity_error
    if live_identity_error is not None and (not recover_stuck or not recovery_reason):
        raise HandlerControlError(
            "refusing lifecycle mutation because active-runtime identity proof failed",
            {"pre_stop_proof": proof, "failure_stage": "active_runtime_identity"},
        )
    return proof


def reclaim_and_snapshot() -> dict[str, Any]:
    receipt: dict[str, Any] = {}
    try:
        receipt["reconnect"] = request("/maintenance/usb/reconnect", method="POST", timeout=60.0)
        receipt["snapshot"] = request("/hardware/snapshot/collect", method="POST", timeout=240.0)
        snapshot_response = receipt["snapshot"]
        snapshot = snapshot_response.get("snapshot") if isinstance(snapshot_response, dict) else None
        requested_domains = snapshot.get("requested_domains") if isinstance(snapshot, dict) else None
        domains = snapshot.get("domains") if isinstance(snapshot, dict) else None
        if (
            not isinstance(snapshot_response, dict)
            or snapshot_response.get("ok") is not True
            or snapshot_response.get("published") is not True
            or not isinstance(requested_domains, list)
            or not requested_domains
            or not isinstance(domains, dict)
        ):
            raise RuntimeError("hardware snapshot was not atomically published")
        failed_domains = {
            str(domain): domains.get(domain)
            for domain in requested_domains
            if not isinstance(domains.get(domain), dict)
            or domains[domain].get("status") != "observed"
            or domains[domain].get("error") is not None
        }
        if failed_domains:
            raise RuntimeError(f"hardware snapshot contains failed requested domains: {sorted(failed_domains)}")
        status = get_status(required=True)
        receipt["status"] = status
        ownership = status.get("ownership") if isinstance(status, dict) else None
        if not isinstance(ownership, dict) or ownership.get("usb") != "service" or ownership.get("router") != "running" or ownership.get("CAN_READY") is not True:
            raise RuntimeError(f"handler started but hardware ownership is not ready: {ownership}")
        return receipt
    except Exception as exc:
        raise HandlerControlError(str(exc), {"hardware": receipt}) from exc


def handoff(*, force: bool) -> dict[str, Any]:
    del force
    raise RuntimeError(
        "host/user recovery handoff is permanently disabled; bioxp-api.service is the sole runtime owner"
    )


def _raise_operation_failure(
    receipt: dict[str, Any],
    stage: str,
    exc: Exception,
) -> None:
    failed = dict(receipt)
    failed["failure_stage"] = stage
    failed["error"] = f"{type(exc).__name__}: {exc}"
    if isinstance(exc, subprocess.CalledProcessError):
        failed["command_failure"] = {
            "argv": list(exc.cmd) if isinstance(exc.cmd, (list, tuple)) else [str(exc.cmd)],
            "returncode": int(exc.returncode),
            "stdout": str(exc.stdout or ""),
            "stderr": str(exc.stderr or ""),
        }
    if isinstance(exc, HandlerControlError):
        failed.update(exc.receipt)
    try:
        details = root_unit_details()
        listener_open = port_open()
        api_status = get_status() if listener_open else None
        live_runtime_identity = None
        live_runtime_identity_error = None
        if _unit_active_or_transitioning(details) or listener_open:
            try:
                live_runtime_identity = verify_root_owner()
            except Exception as identity_exc:
                live_runtime_identity_error = f"{type(identity_exc).__name__}: {identity_exc}"
        failed["post_failure_identity"] = {
            "root_unit": details,
            "api": api_status,
            "listener_open": listener_open,
            "live_runtime_identity": live_runtime_identity,
            "live_runtime_identity_error": live_runtime_identity_error,
        }
    except Exception as snapshot_exc:
        failed["post_failure_identity"] = {
            "capture_error": f"{type(snapshot_exc).__name__}: {snapshot_exc}",
        }
    raise HandlerControlError(str(exc), failed) from exc


def start_or_restart(action: str, *, recover_stuck: bool, recovery_reason: str | None) -> dict[str, Any]:
    receipt: dict[str, Any] = {
        "action": action,
        "recovery": {"recover_stuck": recover_stuck, "reason": recovery_reason},
    }
    stage = "pre_stop_proof" if action == "restart" else "pre_start_proof"
    try:
        if action == "restart":
            receipt["pre_stop_proof"] = pre_stop_proof(
                recover_stuck=recover_stuck,
                recovery_reason=recovery_reason,
            )
        else:
            before_details = root_unit_details()
            pre_start_status = get_status()
            pre_start_listener = port_open()
            receipt["pre_start_proof"] = {
                "loaded_root_unit": before_details,
                "api_status": pre_start_status,
                "listener_open": pre_start_listener,
            }
            refuse_lifecycle_mutation(
                pre_start_status,
                before_details,
                listener_open=pre_start_listener,
                recover_stuck=recover_stuck,
            )
        stage = "systemd_lifecycle_mutation"
        mutation = systemctl(action, UNIT, timeout=45.0)
        receipt["systemd_lifecycle"] = {
            "returncode": int(mutation.returncode),
            "stdout": mutation.stdout,
            "stderr": mutation.stderr,
        }
        stage = "pre_reclaim_root_owner"
        receipt["pre_reclaim_root_unit"] = verify_root_owner()
        stage = "hardware_reclaim_snapshot"
        receipt["hardware"] = reclaim_and_snapshot()
        stage = "post_reclaim_root_owner"
        receipt["root_unit"] = verify_root_owner()
        return receipt
    except Exception as exc:
        _raise_operation_failure(receipt, stage, exc)
        raise AssertionError("unreachable")


def stop(*, recover_stuck: bool, recovery_reason: str | None) -> dict[str, Any]:
    receipt: dict[str, Any] = {
        "action": "stop",
        "recovery": {"recover_stuck": recover_stuck, "reason": recovery_reason},
    }
    stage = "pre_stop_proof"
    try:
        receipt["pre_stop_proof"] = pre_stop_proof(
            recover_stuck=recover_stuck,
            recovery_reason=recovery_reason,
        )
        stage = "systemd_stop"
        systemctl("stop", UNIT, timeout=45.0)
        stage = "listener_shutdown"
        wait_port(False, 30.0)
        stage = "post_stop_loaded_unit"
        receipt["post_stop"] = {
            "root_unit": root_unit_details(),
            "listener_open": port_open(),
            "api_status": get_status(),
        }
        return receipt
    except Exception as exc:
        _raise_operation_failure(receipt, stage, exc)
        raise AssertionError("unreachable")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("status", "handoff", "start", "restart", "stop", "reset-failed"))
    parser.add_argument("--recover-stuck", action="store_true", help="permit an explicit reason-bound stuck-runtime recovery")
    parser.add_argument("--recovery-reason", help="nonempty operator reason required with --recover-stuck")
    args = parser.parse_args()
    recovery_reason = str(args.recovery_reason or "").strip() or None
    if args.recover_stuck and recovery_reason is None:
        raise RuntimeError("--recover-stuck requires a nonempty --recovery-reason")
    if recovery_reason is not None and not args.recover_stuck:
        raise RuntimeError("--recovery-reason requires --recover-stuck")
    if args.recover_stuck and args.action not in {"start", "restart", "stop"}:
        raise RuntimeError("stuck-runtime recovery options apply only to start, restart, or stop")
    if os.geteuid() == 0:
        raise RuntimeError("run handlerctl as molbiofreak, never as root")
    if args.action == "status":
        loaded = root_unit_details()
        listener_open = port_open()
        if _unit_active_or_transitioning(loaded) or listener_open:
            api_status = get_status()
            try:
                authoritative = verify_root_owner()
            except Exception as exc:
                raise HandlerControlError(
                    str(exc),
                    {
                        "action": "status",
                        "failure_stage": "active_runtime_identity",
                        "root_unit": loaded,
                        "api": api_status,
                        "listener_open": listener_open,
                        "error": f"{type(exc).__name__}: {exc}",
                    },
                ) from exc
            result = {
                "root_unit": authoritative,
                "api": get_status(required=True),
                "listener_open": listener_open,
                "authority": {
                    "state": "active",
                    "loaded_unit_verified": True,
                    "installed_release_identity": loaded["loaded_release_receipt"],
                    "live_runtime_identity": authoritative,
                },
            }
        else:
            failed_unit = str(loaded.get("ActiveState") or "") == "failed" or str(loaded.get("SubState") or "") == "failed"
            result = {
                "root_unit": loaded,
                "api": None,
                "listener_open": False,
                "authority": {
                    "state": "degraded" if failed_unit else "inactive",
                    "loaded_unit_verified": True,
                    "installed_release_identity": loaded["loaded_release_receipt"],
                    "live_runtime_identity": None,
                },
            }
    elif args.action == "handoff":
        result = handoff(force=False)
    elif args.action in ("start", "restart"):
        result = start_or_restart(
            args.action,
            recover_stuck=args.recover_stuck,
            recovery_reason=recovery_reason,
        )
    elif args.action == "stop":
        result = stop(recover_stuck=args.recover_stuck, recovery_reason=recovery_reason)
    else:
        systemctl("reset-failed", UNIT)
        result = {"action": "reset-failed", "root_unit": root_unit_details()}
    print(json.dumps(result, indent=2, default=str))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except HandlerControlError as exc:
        print(
            json.dumps({"ok": False, "error": str(exc), **exc.receipt}, indent=2, default=str),
            file=sys.stderr,
        )
        raise SystemExit(1)
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, indent=2), file=sys.stderr)
        raise SystemExit(1)
