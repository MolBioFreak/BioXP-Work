#!/usr/bin/env python3
"""Live Z-axis OEM homing proof for BioXP.

No dry-run/demo mode: without --execute this script exits before opening USB.
The only successful outcome is a raw GAP9 inactive->active transition during
OEM MoveLeft travel followed by setHome() at the physical switch.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
import traceback
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from bioxp.usb_driver import BioXpTester  # noqa: E402

AXIS = "z"
LIVE_PROOF_SPEED = 250
OEM_SOURCE_Z_HOME_SPEED = 1791
DEFAULT_Z_SEARCH_LIMIT = 250000
_ACTIVE_DRIVER: BioXpTester | None = None


def utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def json_default(obj: Any) -> Any:
    if isinstance(obj, Path):
        return str(obj)
    return repr(obj)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, sort_keys=True, indent=2, default=json_default) + "\n")


def ack_success(driver: BioXpTester, row: Any) -> bool:
    return isinstance(row, dict) and (bool(row.get("ok")) or driver._tmcl_success(row.get("ack")))


def stop_all(driver: BioXpTester) -> None:
    for axis in ("z", "x", "y", "g", "door"):
        try:
            preset = driver._motion_oem_axis_profile(axis, startup=True)
            driver.motor_stop(int(preset["board"]), motor=int(preset["motor"]))
        except Exception:
            pass


def emergency_signal_handler(signum: int, _frame: Any) -> None:
    if _ACTIVE_DRIVER is not None:
        stop_all(_ACTIVE_DRIVER)
    raise RuntimeError(f"interrupted_by_signal_{int(signum)}")


def axis_snapshot(driver: BioXpTester, axis: str = AXIS) -> dict[str, Any]:
    preset = driver._motion_oem_axis_profile(axis, startup=True)
    board = int(preset["board"])
    motor = int(preset["motor"])
    params: dict[str, Any] = {}
    for param in (1, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 205):
        try:
            params[str(param)] = driver.motor_get_axis_param(board, param, motor=motor)
        except Exception as exc:
            params[str(param)] = {"ok": False, "error": str(exc)}
    try:
        activity = driver.motor_get_switch_activity(board, motor=motor)
    except Exception as exc:
        activity = {"ok": False, "error": str(exc)}
    return {
        "axis": axis,
        "board": board,
        "motor": motor,
        "active_value": int(driver.MOTOR_SWITCH_ACTIVE_VALUE),
        "position": params.get("1"),
        "speed": params.get("3"),
        "home_gap9": params.get("9"),
        "right_gap10": params.get("10"),
        "switch_activity": activity,
        "params": params,
    }


def z_home_ok(driver: BioXpTester, row: Any) -> bool:
    if not isinstance(row, dict):
        return False
    set_home = row.get("set_home")
    home_after = row.get("home_after")
    transition_ok = bool(
        row.get("ok") is True
        and row.get("switch_transition") is True
        and row.get("false_home_guard") is None
        and ack_success(driver, set_home)
        and isinstance(home_after, dict)
        and home_after.get("value") == int(driver.MOTOR_SWITCH_ACTIVE_VALUE)
    )
    if transition_ok:
        return True
    predicate = row.get("live_z_reference_predicate") if isinstance(row.get("live_z_reference_predicate"), dict) else {}
    no_motion_reference_ok = bool(
        row.get("ok") is True
        and row.get("already_at_z_reference") is True
        and row.get("physical_motion_commanded") is False
        and row.get("false_home_guard") is None
        and set_home is None
        and predicate.get("channel") == "right/GAP10"
        and predicate.get("observed_value") == int(driver.MOTOR_SWITCH_ACTIVE_VALUE)
        and predicate.get("controller_position") == 0
    )
    return no_motion_reference_ok


def main() -> int:
    global _ACTIVE_DRIVER
    signal.signal(signal.SIGINT, emergency_signal_handler)
    signal.signal(signal.SIGTERM, emergency_signal_handler)

    parser = argparse.ArgumentParser(description="Live Z OEM homing proof; no dry-run output.")
    parser.add_argument("--execute", action="store_true", help="Open USB and command live robot motion.")
    parser.add_argument("--artifact-root", default=os.environ.get("BIOXP_LOG_ROOT", "/tmp/bioxp-live-runs"))
    parser.add_argument("--home-timeout-s", type=float, default=45.0)
    parser.add_argument("--z-search-limit", type=int, default=DEFAULT_Z_SEARCH_LIMIT)
    args = parser.parse_args()

    if not args.execute:
        print("REFUSING: this script has no dry-run/demo mode. Re-run with --execute for live hardware.", file=sys.stderr)
        return 2

    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_dir = Path(args.artifact_root) / f"{stamp}_Z_OEM_HOME_PROOF"
    evidence: dict[str, Any] = {
        "schema_version": "bioxp.z_oem_home_proof.v1",
        "started_at": utc_now(),
        "run_dir": str(run_dir),
        "repo_root": str(REPO_ROOT),
        "axis": AXIS,
        "speed": LIVE_PROOF_SPEED,
        "oem_source_z_home_speed": OEM_SOURCE_Z_HOME_SPEED,
        "z_search_limit": int(args.z_search_limit),
        "home_timeout_s": float(args.home_timeout_s),
        "ok": False,
        "steps": {},
        "snapshots": {},
    }
    run_dir.mkdir(parents=True, exist_ok=True)
    write_json(run_dir / "manifest.json", evidence)

    driver: BioXpTester | None = None
    try:
        driver = BioXpTester()
        _ACTIVE_DRIVER = driver
        evidence["snapshots"]["before"] = axis_snapshot(driver)
        write_json(run_dir / "snapshot_before.json", evidence["snapshots"]["before"])

        strict = driver.motion_arm_strict_startup(run_homing=False)
        evidence["steps"]["strict_startup_no_homing"] = strict
        write_json(run_dir / "strict_startup_no_homing.json", strict)
        if not bool(strict.get("ok")):
            raise RuntimeError("strict_startup_no_homing_failed")

        prep = driver.motor_oem_initialize_without_motion()
        evidence["steps"]["oem_initialize_without_motion"] = prep
        write_json(run_dir / "oem_initialize_without_motion.json", prep)
        evidence["snapshots"]["after_prepare"] = axis_snapshot(driver)
        write_json(run_dir / "snapshot_after_prepare.json", evidence["snapshots"]["after_prepare"])

        z_home = driver.motor_oem_home_axis(
            AXIS,
            speed=LIVE_PROOF_SPEED,
            timeout_s=float(args.home_timeout_s),
            startup=True,
        )
        home_payload = z_home.get("home") if isinstance(z_home, dict) else z_home
        evidence["steps"]["z_oem_home_axis"] = z_home
        write_json(run_dir / "z_oem_home_axis.json", z_home)

        evidence["snapshots"]["after_home"] = axis_snapshot(driver)
        write_json(run_dir / "snapshot_after_home.json", evidence["snapshots"]["after_home"])
        evidence["ok"] = z_home_ok(driver, home_payload)
        evidence["proof_mode"] = (
            "transition_switch_home" if isinstance(home_payload, dict) and home_payload.get("switch_transition") is True
            else "already_at_live_z_reference_no_motion" if isinstance(home_payload, dict) and home_payload.get("already_at_z_reference") is True
            else "failed"
        )
        if not evidence["ok"]:
            raise RuntimeError("z_oem_home_axis_failed")
        evidence["finished_at"] = utc_now()
        write_json(run_dir / "final_evidence.json", evidence)
        print(json.dumps({"ok": True, "run_dir": str(run_dir), "proof_mode": evidence.get("proof_mode"), "home": home_payload}, sort_keys=True))
        return 0
    except Exception as exc:
        if driver is not None:
            try:
                stop_all(driver)
            except Exception:
                pass
        evidence["ok"] = False
        evidence["error"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
        evidence["finished_at"] = utc_now()
        write_json(run_dir / "final_evidence.json", evidence)
        print(json.dumps({"ok": False, "run_dir": str(run_dir), "error": evidence["error"]}, sort_keys=True), file=sys.stderr)
        return 1
    finally:
        _ACTIVE_DRIVER = None
        if driver is not None:
            try:
                driver._disconnect(hard_reset=False)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
