#!/usr/bin/env python3
"""Live BioXP OEM reference challenge.

Hard-gated hardware validation for the requested sequence:
  1. strict startup gate without homing
  2. OEM-like physical switch reference pass for X/Y/Z
  3. challenge move: Z=-70000, X=50000, Y=50000
  4. from that challenge pose, physical switch search back to reference
  5. final setHome() at the switch and persisted reference/sensor artifacts

This script deliberately has no dry-run mode.  Without --execute it exits before
opening the USB device so it cannot produce fake/demo motion evidence.
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
from bioxp.services.reference_service import (  # noqa: E402
    MarkAxisDesyncedCommand,
    MarkAxisReferencedCommand,
    ReferenceStateStore,
)

AXES = ("z", "x", "y")
CHALLENGE_TARGETS = {"z": -70000, "x": 50000, "y": 50000}
STARTUP_HOME_SPEEDS = {"z": 1791, "x": 250, "y": 250}
DEFAULT_SEARCH_LIMITS = {"z": 250000, "x": 75000, "y": 75000}
PRECLEAR_STEPS = {"z": 10000, "x": 10000, "y": 10000}
_ACTIVE_DRIVER: BioXpTester | None = None


def emergency_signal_handler(signum: int, _frame: Any) -> None:
    if _ACTIVE_DRIVER is not None:
        try:
            stop_all(_ACTIVE_DRIVER)
        except Exception:
            pass
    raise RuntimeError(f"interrupted_by_signal_{int(signum)}")


def utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def json_default(obj: Any) -> Any:
    if isinstance(obj, Path):
        return str(obj)
    return repr(obj)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, sort_keys=True, indent=2, default=json_default) + "\n")


def append_jsonl(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, sort_keys=True, default=json_default) + "\n")


def ack_ok(driver: BioXpTester, row: Any) -> bool:
    if isinstance(row, dict) and "ack" in row:
        return bool(row.get("ok")) or driver._tmcl_success(row.get("ack"))
    return bool(row)


def unwrap_home(row: Any) -> dict[str, Any] | None:
    if not isinstance(row, dict):
        return None
    inner = row.get("home")
    return inner if isinstance(inner, dict) else row


def home_ok(row: Any) -> bool:
    h = unwrap_home(row)
    if not isinstance(h, dict):
        return False
    set_home = h.get("set_home")
    set_home_ok = isinstance(set_home, dict) and bool(set_home.get("ok"))
    return bool(
        h.get("ok")
        and h.get("switch_transition")
        and set_home_ok
        and h.get("false_home_guard") is None
    )


def axis_profile(driver: BioXpTester, axis: str, *, startup: bool = True) -> dict[str, Any]:
    return driver._motion_oem_axis_profile(axis, startup=startup)


def axis_snapshot(driver: BioXpTester, axis: str) -> dict[str, Any]:
    preset = axis_profile(driver, axis, startup=True)
    board = int(preset["board"])
    motor = int(preset["motor"])
    params = {}
    for param in (1, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 205):
        try:
            params[str(param)] = driver.motor_get_axis_param(board, param, motor=motor)
        except Exception as exc:  # keep artifact complete even on one read failure
            params[str(param)] = {"ok": False, "error": str(exc)}
    return {
        "axis": axis,
        "board": board,
        "motor": motor,
        "position": driver.motor_get_position(board, motor=motor),
        "speed": driver.motor_get_speed(board, motor=motor),
        "home_switch": driver.motor_query_home_switch(board, motor=motor),
        "switch_activity": driver.motor_get_switch_activity(board, motor=motor),
        "axis_params": params,
    }


def machine_snapshot(driver: BioXpTester, label: str) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "label": label,
        "captured_at": utc_now(),
        "motion_gate": None,
        "rail_24v": None,
        "axes": {},
    }
    try:
        payload["motion_gate"] = driver.motion_gate_live_snapshot()
    except Exception as exc:
        payload["motion_gate"] = {"ok": False, "error": str(exc)}
    try:
        payload["rail_24v"] = driver.motor_query_24v_sensor()
    except Exception as exc:
        payload["rail_24v"] = {"ok": False, "error": str(exc)}
    for axis in ("z", "x", "y", "g", "door"):
        try:
            payload["axes"][axis] = axis_snapshot(driver, axis)
        except Exception as exc:
            payload["axes"][axis] = {"ok": False, "error": str(exc)}
    return payload


def stop_all(driver: BioXpTester) -> dict[str, Any]:
    out: dict[str, Any] = {"stopped_at": utc_now(), "axes": {}}
    for axis in ("z", "x", "y", "g", "door"):
        try:
            preset = axis_profile(driver, axis, startup=True)
            out["axes"][axis] = driver.motor_stop(int(preset["board"]), motor=int(preset["motor"]))
        except Exception as exc:
            out["axes"][axis] = {"ok": False, "error": str(exc)}
    return out


def require(condition: bool, code: str, detail: Any, evidence: dict[str, Any]) -> None:
    if condition:
        return
    raise RuntimeError(json.dumps({"code": code, "detail": detail}, sort_keys=True, default=json_default))


def wait_move(driver: BioXpTester, axis: str, target: int, timeout_s: float) -> dict[str, Any]:
    preset = axis_profile(driver, axis, startup=True)
    board = int(preset["board"])
    motor = int(preset["motor"])
    move = driver.motor_move_absolute(board, int(target), motor=motor)
    wait = driver.motor_wait_stopped(board, motor=motor, timeout_s=float(timeout_s), poll_s=0.08, require_seen_nonzero=False, min_polls=4)
    post = axis_snapshot(driver, axis)
    return {"axis": axis, "target": int(target), "move": move, "wait": wait, "post": post}


def position_value(snapshot_row: dict[str, Any]) -> int | None:
    try:
        return int(snapshot_row["position"]["position"])
    except Exception:
        return None


def position_within(row: dict[str, Any], target: int, tolerance: int) -> bool:
    value = position_value(row["post"] if "post" in row else row)
    return value is not None and abs(value - int(target)) <= int(tolerance)


def prepare_axis_for_reference(driver: BioXpTester, axis: str) -> dict[str, Any]:
    preset = axis_profile(driver, axis, startup=True)
    return driver.motor_prepare_axis(
        preset["board"],
        motor=preset["motor"],
        run_current=preset.get("run_current"),
        standby_current=preset.get("standby_current"),
        speed=preset.get("speed"),
        acc=preset.get("acc"),
        stall_guard=preset.get("stall_guard"),
        ramp_mode=preset.get("ramp_mode"),
        disable_right=bool(preset.get("disable_right", False)),
        disable_left=bool(preset.get("disable_left", False)),
        rdiv=preset.get("rdiv"),
        pdiv=preset.get("pdiv"),
        warm_enable=bool(preset.get("warm_enable", False)),
    )


def preclear_home_switch_until_inactive(
    driver: BioXpTester,
    axis: str,
    *,
    step: int,
    max_abs_delta: int,
    timeout_s: float,
) -> dict[str, Any]:
    preset = axis_profile(driver, axis, startup=True)
    board = int(preset["board"])
    motor = int(preset["motor"])
    active_value = int(driver.MOTOR_SWITCH_ACTIVE_VALUE)
    start_position = driver.motor_get_position(board, motor=motor)
    start_home = driver.motor_query_home_switch(board, motor=motor)
    start_switches = driver.motor_get_switch_activity(board, motor=motor)
    trace = []
    total_delta = 0
    if start_home.get("value") != active_value:
        return {
            "ok": True,
            "axis": axis,
            "needed": False,
            "active_value": active_value,
            "start_position": start_position,
            "start_home": start_home,
            "start_switches": start_switches,
            "trace": trace,
        }
    while abs(total_delta) < int(max_abs_delta):
        remaining = int(max_abs_delta) - abs(total_delta)
        this_step = min(abs(int(step)), remaining)
        if this_step <= 0:
            break
        move = driver.motor_move_relative(board, this_step, motor=motor)
        wait = driver.motor_wait_stopped(board, motor=motor, timeout_s=min(float(timeout_s), 12.0), poll_s=0.08, require_seen_nonzero=False, min_polls=4)
        total_delta += this_step
        time.sleep(0.12)
        home = driver.motor_query_home_switch(board, motor=motor)
        switches = driver.motor_get_switch_activity(board, motor=motor)
        position = driver.motor_get_position(board, motor=motor)
        row = {
            "delta_from_start": total_delta,
            "move": move,
            "wait": wait,
            "home": home,
            "switches": switches,
            "position": position,
        }
        trace.append(row)
        if home.get("value") != active_value:
            return {
                "ok": True,
                "axis": axis,
                "needed": True,
                "active_value": active_value,
                "start_position": start_position,
                "start_home": start_home,
                "start_switches": start_switches,
                "deassert_delta": total_delta,
                "trace": trace,
                "final_home": home,
                "final_switches": switches,
                "final_position": position,
            }
    stop = driver.motor_stop(board, motor=motor)
    return {
        "ok": False,
        "axis": axis,
        "needed": True,
        "active_value": active_value,
        "start_position": start_position,
        "start_home": start_home,
        "start_switches": start_switches,
        "max_abs_delta": int(max_abs_delta),
        "delta_from_start": total_delta,
        "trace": trace,
        "stop": stop,
        "false_home_guard": "home_switch_did_not_deassert_after_incremental_preclear",
    }


def physical_reference_axis(
    driver: BioXpTester,
    axis: str,
    *,
    speed: int,
    home_timeout_s: float,
    preclear_step: int,
    max_search_abs_delta: int,
) -> dict[str, Any]:
    prepare = prepare_axis_for_reference(driver, axis)
    # Use the OEM startup/reference primitive directly:
    # Class*Board.axisSearchHome(axis, speed) == setHome(); optional moveToAbs(10000)
    # if queryHome is already active; then goHome(false) and final setHome() only
    # after the switch predicate is observed during travel.  preclear_step remains
    # in the signature only to keep the CLI/artifact schema stable for old bundles.
    home = driver.motor_oem_axis_search_home(
        axis,
        speed=int(speed),
        timeout_s=float(home_timeout_s),
        max_search_abs_delta=int(max_search_abs_delta),
    )
    return {
        "axis": axis,
        "oem_method": "ClassBoard.axisSearchHome_setHome_goHome_false_setHome",
        "prepare": prepare,
        "preclear": home.get("preclear_move") if isinstance(home, dict) else None,
        "home": home,
        "ok": home_ok(home),
        "switch_transition": home.get("switch_transition") if isinstance(home, dict) else None,
        "set_home": home.get("set_home") if isinstance(home, dict) else None,
        "false_home_guard": home.get("false_home_guard") if isinstance(home, dict) else None,
    }


def default_reference_state_path() -> Path:
    env = os.environ.get("BIOXP_REFERENCE_STATE_PATH")
    if env:
        return Path(env)
    xdg = os.environ.get("XDG_STATE_HOME")
    if xdg:
        return Path(xdg) / "bioxp" / "reference-state.json"
    return Path.home() / ".local" / "state" / "bioxp" / "reference-state.json"


def main() -> int:
    global _ACTIVE_DRIVER
    signal.signal(signal.SIGINT, emergency_signal_handler)
    signal.signal(signal.SIGTERM, emergency_signal_handler)

    parser = argparse.ArgumentParser(description="Run live BioXP OEM reference challenge; no dry-run output.")
    parser.add_argument("--execute", action="store_true", help="Actually open USB and command live motion.")
    parser.add_argument("--artifact-root", default=os.environ.get("BIOXP_LOG_ROOT", "/tmp/bioxp-live-runs"))
    parser.add_argument("--reference-state-path", default=str(default_reference_state_path()))
    parser.add_argument("--position-tolerance", type=int, default=250)
    parser.add_argument("--move-timeout-s", type=float, default=45.0)
    parser.add_argument("--home-timeout-s", type=float, default=35.0)
    parser.add_argument("--z-search-limit", type=int, default=DEFAULT_SEARCH_LIMITS["z"])
    parser.add_argument("--x-search-limit", type=int, default=DEFAULT_SEARCH_LIMITS["x"])
    parser.add_argument("--y-search-limit", type=int, default=DEFAULT_SEARCH_LIMITS["y"])
    parser.add_argument("--skip-initial-reference", action="store_true", help="Do not do the initial reference pass before challenge moves.")
    args = parser.parse_args()

    if not args.execute:
        print("REFUSING: this script has no dry-run/demo mode. Re-run with --execute for live hardware.", file=sys.stderr)
        return 2

    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_dir = Path(args.artifact_root) / f"{stamp}_OEM_REFERENCE_CHALLENGE"
    events_path = run_dir / "events.jsonl"
    evidence: dict[str, Any] = {
        "schema_version": "bioxp.oem_reference_challenge.v1",
        "started_at": utc_now(),
        "repo_root": str(REPO_ROOT),
        "run_dir": str(run_dir),
        "challenge_targets": CHALLENGE_TARGETS,
        "position_tolerance": int(args.position_tolerance),
        "reference_state_path": str(args.reference_state_path),
        "steps": {},
        "snapshots": {},
        "ok": False,
    }
    run_dir.mkdir(parents=True, exist_ok=True)
    write_json(run_dir / "manifest.json", evidence)
    append_jsonl(events_path, {"event": "start", "at": utc_now(), "run_dir": str(run_dir)})

    driver: BioXpTester | None = None
    store = ReferenceStateStore(args.reference_state_path)

    try:
        driver = BioXpTester()
        _ACTIVE_DRIVER = driver
        append_jsonl(events_path, {"event": "usb_opened", "at": utc_now()})

        evidence["snapshots"]["before"] = machine_snapshot(driver, "before")
        write_json(run_dir / "snapshot_before.json", evidence["snapshots"]["before"])

        strict = driver.motion_arm_strict_startup(run_homing=False)
        evidence["steps"]["strict_startup_no_homing"] = strict
        write_json(run_dir / "strict_startup_no_homing.json", strict)
        require(bool(strict.get("ok")), "strict_startup_no_homing_failed", strict.get("checks"), evidence)

        prep = driver.motor_oem_initialize_without_motion()
        evidence["steps"]["oem_initialize_without_motion"] = prep
        write_json(run_dir / "oem_initialize_without_motion.json", prep)

        limit_by_axis = {"z": args.z_search_limit, "x": args.x_search_limit, "y": args.y_search_limit}

        if not args.skip_initial_reference:
            initial_ref: dict[str, Any] = {}
            for axis in AXES:
                row = physical_reference_axis(
                    driver,
                    axis,
                    speed=STARTUP_HOME_SPEEDS[axis],
                    home_timeout_s=float(args.home_timeout_s),
                    preclear_step=PRECLEAR_STEPS[axis],
                    max_search_abs_delta=int(limit_by_axis[axis]),
                )
                initial_ref[axis] = row
                write_json(run_dir / f"initial_reference_{axis}.json", row)
                append_jsonl(events_path, {"event": "initial_reference", "axis": axis, "ok": home_ok(row), "at": utc_now()})
                require(home_ok(row), f"initial_reference_{axis}_failed", row, evidence)
            evidence["steps"]["initial_reference"] = initial_ref

        evidence["snapshots"]["after_initial_reference"] = machine_snapshot(driver, "after_initial_reference")
        write_json(run_dir / "snapshot_after_initial_reference.json", evidence["snapshots"]["after_initial_reference"])

        challenge_moves: dict[str, Any] = {}
        # Z first to the requested high/negative coordinate, then X/Y to 50000.
        z_move = driver.motor_oem_move_z_to_reference(target_position=CHALLENGE_TARGETS["z"], timeout_s=float(args.move_timeout_s))
        challenge_moves["z"] = z_move
        write_json(run_dir / "challenge_move_z.json", z_move)
        require(bool(z_move.get("ok")), "challenge_move_z_failed", z_move, evidence)
        require(position_within({"post": z_move.get("post", {})}, CHALLENGE_TARGETS["z"], args.position_tolerance), "challenge_move_z_position_mismatch", z_move.get("post"), evidence)

        for axis in ("x", "y"):
            row = wait_move(driver, axis, CHALLENGE_TARGETS[axis], timeout_s=float(args.move_timeout_s))
            challenge_moves[axis] = row
            write_json(run_dir / f"challenge_move_{axis}.json", row)
            require(ack_ok(driver, row.get("move")) and row.get("wait", {}).get("stopped") is True, f"challenge_move_{axis}_failed", row, evidence)
            require(position_within(row, CHALLENGE_TARGETS[axis], args.position_tolerance), f"challenge_move_{axis}_position_mismatch", row, evidence)

        evidence["steps"]["challenge_moves"] = challenge_moves
        evidence["snapshots"]["challenge_pose"] = machine_snapshot(driver, "challenge_pose_z-70000_x50000_y50000")
        write_json(run_dir / "snapshot_challenge_pose.json", evidence["snapshots"]["challenge_pose"])

        challenge_position_proof = {
            axis: {
                "target": CHALLENGE_TARGETS[axis],
                "observed": position_value(evidence["snapshots"]["challenge_pose"]["axes"][axis]),
                "within_tolerance": position_within(evidence["snapshots"]["challenge_pose"]["axes"][axis], CHALLENGE_TARGETS[axis], args.position_tolerance),
            }
            for axis in AXES
        }
        evidence["challenge_position_proof"] = challenge_position_proof
        write_json(run_dir / "challenge_position_proof.json", challenge_position_proof)
        require(all(row["within_tolerance"] for row in challenge_position_proof.values()), "challenge_position_proof_failed", challenge_position_proof, evidence)

        final_ref: dict[str, Any] = {}
        for axis in AXES:
            row = physical_reference_axis(
                driver,
                axis,
                speed=STARTUP_HOME_SPEEDS[axis],
                home_timeout_s=float(args.home_timeout_s),
                preclear_step=PRECLEAR_STEPS[axis],
                max_search_abs_delta=int(limit_by_axis[axis]),
            )
            final_ref[axis] = row
            write_json(run_dir / f"final_reference_{axis}.json", row)
            append_jsonl(events_path, {"event": "final_reference", "axis": axis, "ok": home_ok(row), "at": utc_now()})
            require(home_ok(row), f"final_reference_{axis}_failed", row, evidence)

        evidence["steps"]["final_reference"] = final_ref
        evidence["snapshots"]["after_final_reference"] = machine_snapshot(driver, "after_final_reference")
        write_json(run_dir / "snapshot_after_final_reference.json", evidence["snapshots"]["after_final_reference"])

        reference_updates: dict[str, Any] = {}
        for axis in AXES:
            reference_updates[axis] = store.mark_referenced(
                MarkAxisReferencedCommand(
                    axis=axis,
                    position_steps=0,
                    source="oem_reference_challenge",
                    note="Physical switch inactive-to-active transition observed; final setHome() executed at switch after Z=-70000/X=50000/Y=50000 challenge pose.",
                    motion_kind="oem_reference_challenge_final_home",
                )
            )
        evidence["reference_state_updates"] = reference_updates
        evidence["reference_state_snapshot"] = store.snapshot(AXES)
        write_json(run_dir / "reference_state_updates.json", reference_updates)
        write_json(run_dir / "reference_state_snapshot.json", evidence["reference_state_snapshot"])

        sensor_reference_artifacts = {
            "created_at": utc_now(),
            "basis": "post-final-reference live controller reads",
            "motion_gate": evidence["snapshots"]["after_final_reference"].get("motion_gate"),
            "rail_24v": evidence["snapshots"]["after_final_reference"].get("rail_24v"),
            "axis_home_switches": {
                axis: evidence["snapshots"]["after_final_reference"]["axes"][axis].get("home_switch")
                for axis in AXES
            },
            "axis_switch_activity": {
                axis: evidence["snapshots"]["after_final_reference"]["axes"][axis].get("switch_activity")
                for axis in AXES
            },
            "axis_positions": {
                axis: evidence["snapshots"]["after_final_reference"]["axes"][axis].get("position")
                for axis in AXES
            },
        }
        evidence["sensor_reference_artifacts"] = sensor_reference_artifacts
        write_json(run_dir / "sensor_reference_artifacts.json", sensor_reference_artifacts)

        final_position_proof = {
            axis: {
                "observed": position_value(evidence["snapshots"]["after_final_reference"]["axes"][axis]),
                "home_switch": evidence["snapshots"]["after_final_reference"]["axes"][axis].get("home_switch"),
                "reference_state": evidence["reference_state_snapshot"]["rows"].get(axis),
                "switch_transition": final_ref[axis].get("switch_transition"),
                "set_home": final_ref[axis].get("set_home"),
                "false_home_guard": final_ref[axis].get("false_home_guard"),
            }
            for axis in AXES
        }
        evidence["final_position_proof"] = final_position_proof
        write_json(run_dir / "final_position_proof.json", final_position_proof)

        evidence["ok"] = True
        evidence["completed_at"] = utc_now()
        write_json(run_dir / "final_evidence.json", evidence)
        append_jsonl(events_path, {"event": "complete", "ok": True, "at": utc_now(), "final_evidence": str(run_dir / "final_evidence.json")})
        print(json.dumps({"ok": True, "run_dir": str(run_dir), "final_evidence": str(run_dir / "final_evidence.json")}, sort_keys=True))
        return 0

    except Exception as exc:
        evidence["ok"] = False
        evidence["failed_at"] = utc_now()
        evidence["error"] = str(exc)
        evidence["traceback"] = traceback.format_exc()
        if driver is not None:
            try:
                evidence["emergency_stop"] = stop_all(driver)
            except Exception as stop_exc:
                evidence["emergency_stop"] = {"ok": False, "error": str(stop_exc)}
            try:
                evidence["snapshots"]["failure"] = machine_snapshot(driver, "failure")
            except Exception as snap_exc:
                evidence["snapshots"]["failure"] = {"ok": False, "error": str(snap_exc)}
        try:
            for axis in AXES:
                store.mark_desynced(
                    MarkAxisDesyncedCommand(
                        axis=axis,
                        reason=f"oem_reference_challenge_failed: {exc}",
                        source="oem_reference_challenge",
                        motion_kind="oem_reference_challenge_failed",
                    )
                )
        except Exception:
            pass
        write_json(run_dir / "final_evidence.json", evidence)
        append_jsonl(events_path, {"event": "failed", "ok": False, "at": utc_now(), "error": str(exc), "final_evidence": str(run_dir / "final_evidence.json")})
        print(json.dumps({"ok": False, "run_dir": str(run_dir), "final_evidence": str(run_dir / "final_evidence.json"), "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
