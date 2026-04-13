#!/usr/bin/env python3
"""Cycle the BioXP LED strip through colors via the local BMS BioXP proxy.

This version is intentionally paced and less hacky than the first pass:
- frame timing is based on a steady monotonic schedule rather than request time + sleep
- rainbow speed can be specified as a full cycle duration in seconds
- reconnects are time-based instead of forced every few frames by default
- retries are bounded and explicit instead of being mixed into the main loop
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import List, Sequence, Tuple

RGB = Tuple[int, int, int]


@dataclass
class StopController:
    stop_requested: bool = False

    def handle_signal(self, signum, _frame) -> None:
        self.stop_requested = True
        print(
            json.dumps(
                {
                    "event": "signal",
                    "signal": int(signum),
                    "message": "stopping LED cycle",
                }
            ),
            flush=True,
        )


STOP = StopController()
for _sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(_sig, STOP.handle_signal)


@dataclass(frozen=True)
class RuntimeConfig:
    base_url: str
    timeout: float


@dataclass
class SendResult:
    ok: bool
    reconnect_first: bool
    rgb: RGB
    response: dict | None
    ack_statuses: list[str]
    elapsed_ms: int
    error: str | None = None


_COLOR_ALIASES = {
    "red": (255, 0, 0),
    "orange": (255, 64, 0),
    "yellow": (255, 255, 0),
    "green": (0, 255, 0),
    "cyan": (0, 255, 255),
    "blue": (0, 0, 255),
    "magenta": (255, 0, 255),
    "purple": (180, 0, 255),
    "white": (255, 255, 255),
    "off": (0, 0, 0),
}


def clamp_u8(value: int) -> int:
    return max(0, min(255, int(value)))


def parse_rgb_triplet(text: str) -> RGB:
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise ValueError(f"Expected r,g,b triplet, got {text!r}")
    try:
        return tuple(clamp_u8(int(part)) for part in parts)  # type: ignore[return-value]
    except ValueError as exc:
        raise ValueError(f"Invalid RGB triplet {text!r}") from exc


def scale_rgb(rgb: RGB, brightness: int) -> RGB:
    brightness = clamp_u8(brightness)
    if brightness >= 255:
        return rgb
    return tuple((int(c) * brightness) // 255 for c in rgb)


def wheel_rgb(pos: int) -> RGB:
    p = int(pos) % 256
    if p < 85:
        return 255 - (p * 3), p * 3, 0
    if p < 170:
        p -= 85
        return 0, 255 - (p * 3), p * 3
    p -= 170
    return p * 3, 0, 255 - (p * 3)


def request_json(url: str, *, method: str = "GET", payload: dict | None = None, timeout: float = 20.0) -> dict:
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code} for {url}: {body}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Request failed for {url}: {exc}") from exc
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Non-JSON response from {url}: {raw[:300]!r}") from exc


def parse_sequence(spec: str, brightness: int) -> List[RGB]:
    colors: List[RGB] = []
    for item in spec.split(";"):
        token = item.strip().lower()
        if not token:
            continue
        if token in _COLOR_ALIASES:
            rgb = _COLOR_ALIASES[token]
        else:
            rgb = parse_rgb_triplet(token)
        colors.append(scale_rgb(rgb, brightness))
    if not colors:
        raise ValueError("Sequence is empty")
    return colors


def resolve_final_color(spec: str, brightness: int) -> RGB | None:
    token = (spec or "keep").strip().lower()
    if token == "keep":
        return None
    if token in _COLOR_ALIASES:
        return scale_rgb(_COLOR_ALIASES[token], brightness)
    return scale_rgb(parse_rgb_triplet(token), brightness)


def rainbow_rgb(elapsed_s: float, frame: int, brightness: int, cycle_seconds: float, stride: int) -> RGB:
    if cycle_seconds > 0:
        pos = int(((elapsed_s % cycle_seconds) / cycle_seconds) * 256.0)
    else:
        pos = (int(frame) * max(1, int(stride))) % 256
    return scale_rgb(wheel_rgb(pos), brightness)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Cycle the BioXP LED strip through colors via the local BMS proxy.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8000/api/bioxp", help="BioXP proxy base URL")
    parser.add_argument("--mode", choices=("sequence", "rainbow"), default="sequence")
    parser.add_argument("--sequence", default="red;orange;yellow;green;cyan;blue;magenta", help="Semicolon-separated list of color names or r,g,b triplets")
    parser.add_argument("--brightness", type=int, default=160, help="0-255 brightness cap applied before sending")
    parser.add_argument("--step-delay", type=float, default=1.0, help="Target seconds per frame, including request time")
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after this many seconds; 0 means run until interrupted")
    parser.add_argument("--cycle-seconds", type=float, default=90.0, help="Rainbow cycle duration in seconds when > 0; smaller is faster")
    parser.add_argument("--rainbow-stride", type=int, default=4, help="Legacy rainbow hue step used only when cycle-seconds <= 0")
    parser.add_argument("--reconnect-seconds", type=float, default=30.0, help="Force reconnect_first on the first frame and then every N seconds; 0 disables periodic reconnects")
    parser.add_argument("--reconnect-every", type=int, default=0, help="Legacy reconnect cadence in frames, used only when reconnect-seconds <= 0")
    parser.add_argument("--max-consecutive-failures", type=int, default=5, help="Abort after this many failed frame sends in a row")
    parser.add_argument("--timeout", type=float, default=20.0, help="Per-request timeout in seconds")
    parser.add_argument("--final", default="keep", help="Final color on exit: keep, off, a color alias, or r,g,b")
    parser.add_argument("--verbose", action="store_true", help="Print one JSON line per frame")
    return parser


def extract_ack_statuses(response: dict | None) -> list[str]:
    if not isinstance(response, dict):
        return []
    acks = response.get("acks") or {}
    return [
        str((ack or {}).get("status_str"))
        for ack in acks.values()
        if isinstance(ack, dict) and (ack or {}).get("status_str") is not None
    ]


def response_ok(response: dict | None) -> tuple[bool, list[str], str | None]:
    statuses = extract_ack_statuses(response)
    if not statuses:
        return False, [], "missing ack statuses"
    if any(status != "Success" for status in statuses):
        return False, statuses, f"non-success ack(s): {statuses}"
    return True, statuses, None


def send_rgb(runtime: RuntimeConfig, rgb: RGB, *, reconnect_first: bool) -> SendResult:
    payload = {
        "r": int(rgb[0]),
        "g": int(rgb[1]),
        "b": int(rgb[2]),
        "reconnect_first": bool(reconnect_first),
    }
    t0 = time.monotonic()
    try:
        response = request_json(
            f"{runtime.base_url}/led/rgb",
            method="POST",
            payload=payload,
            timeout=runtime.timeout,
        )
        ok, statuses, error = response_ok(response)
        elapsed_ms = int(round((time.monotonic() - t0) * 1000.0))
        return SendResult(
            ok=ok,
            reconnect_first=reconnect_first,
            rgb=rgb,
            response=response,
            ack_statuses=statuses,
            elapsed_ms=elapsed_ms,
            error=error,
        )
    except Exception as exc:
        elapsed_ms = int(round((time.monotonic() - t0) * 1000.0))
        return SendResult(
            ok=False,
            reconnect_first=reconnect_first,
            rgb=rgb,
            response=None,
            ack_statuses=[],
            elapsed_ms=elapsed_ms,
            error=str(exc),
        )


def send_rgb_with_recovery(runtime: RuntimeConfig, rgb: RGB, *, reconnect_first: bool) -> tuple[SendResult, list[SendResult]]:
    attempts: list[SendResult] = []
    first = send_rgb(runtime, rgb, reconnect_first=reconnect_first)
    attempts.append(first)
    if first.ok:
        return first, attempts

    second = send_rgb(runtime, rgb, reconnect_first=True)
    attempts.append(second)
    return second, attempts


def should_reconnect(frame: int, now_mono: float, next_reconnect_deadline: float | None, reconnect_seconds: float, reconnect_every: int) -> bool:
    if frame == 0:
        return True
    if reconnect_seconds > 0:
        return next_reconnect_deadline is not None and now_mono >= next_reconnect_deadline
    return reconnect_every > 0 and frame % reconnect_every == 0


def next_rgb(args: argparse.Namespace, elapsed_s: float, frame: int, sequence_colors: Sequence[RGB] | None) -> RGB:
    if args.mode == "sequence":
        assert sequence_colors is not None
        return sequence_colors[frame % len(sequence_colors)]
    return rainbow_rgb(
        elapsed_s=elapsed_s,
        frame=frame,
        brightness=args.brightness,
        cycle_seconds=max(0.0, float(args.cycle_seconds)),
        stride=max(1, int(args.rainbow_stride)),
    )


def sleep_until(deadline: float) -> None:
    while not STOP.stop_requested:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(remaining, 0.2))


def main() -> int:
    args = build_arg_parser().parse_args()
    runtime = RuntimeConfig(base_url=args.base_url.rstrip("/"), timeout=float(args.timeout))

    linkage = request_json(f"{runtime.base_url}/linkage", timeout=runtime.timeout)
    status = request_json(f"{runtime.base_url}/status", timeout=runtime.timeout)
    if not status.get("hardware_connected"):
        print(
            json.dumps(
                {
                    "event": "startup_error",
                    "message": "BioXP hardware is not connected according to /status",
                    "status": status,
                    "linkage": linkage,
                }
            ),
            file=sys.stderr,
            flush=True,
        )
        return 2

    sequence_colors = parse_sequence(args.sequence, args.brightness) if args.mode == "sequence" else None
    final_color = resolve_final_color(args.final, args.brightness)
    frame_period_s = max(0.05, float(args.step_delay))
    reconnect_seconds = max(0.0, float(args.reconnect_seconds))
    max_consecutive_failures = max(1, int(args.max_consecutive_failures))

    start_mono = time.monotonic()
    next_deadline = start_mono
    next_reconnect_deadline: float | None = None
    frame = 0
    last_rgb: RGB | None = None
    consecutive_failures = 0
    exit_reason = "completed"

    print(
        json.dumps(
            {
                "event": "startup",
                "base_url": runtime.base_url,
                "mode": args.mode,
                "brightness": clamp_u8(args.brightness),
                "step_delay": frame_period_s,
                "duration": float(args.duration),
                "cycle_seconds": float(args.cycle_seconds),
                "rainbow_stride": int(args.rainbow_stride),
                "reconnect_seconds": reconnect_seconds,
                "reconnect_every": int(args.reconnect_every),
                "max_consecutive_failures": max_consecutive_failures,
                "linkage": linkage,
                "status_summary": {
                    "status": status.get("status"),
                    "transport": status.get("transport"),
                    "runtime_available": status.get("runtime_available"),
                    "hardware_connected": status.get("hardware_connected"),
                },
            }
        ),
        flush=True,
    )

    try:
        while not STOP.stop_requested:
            now_mono = time.monotonic()
            elapsed_s = now_mono - start_mono
            if args.duration > 0 and elapsed_s >= float(args.duration):
                exit_reason = "duration_elapsed"
                break

            rgb = next_rgb(args, elapsed_s, frame, sequence_colors)
            reconnect_first = should_reconnect(
                frame=frame,
                now_mono=now_mono,
                next_reconnect_deadline=next_reconnect_deadline,
                reconnect_seconds=reconnect_seconds,
                reconnect_every=int(args.reconnect_every),
            )
            result, attempts = send_rgb_with_recovery(runtime, rgb, reconnect_first=reconnect_first)

            if result.ok:
                consecutive_failures = 0
                last_rgb = rgb
                if reconnect_seconds > 0 and any(item.reconnect_first for item in attempts):
                    next_reconnect_deadline = time.monotonic() + reconnect_seconds
            else:
                consecutive_failures += 1
                print(
                    json.dumps(
                        {
                            "event": "frame_error",
                            "frame": frame,
                            "rgb": list(rgb),
                            "attempts": [
                                {
                                    "reconnect_first": item.reconnect_first,
                                    "elapsed_ms": item.elapsed_ms,
                                    "ack_statuses": item.ack_statuses,
                                    "error": item.error,
                                }
                                for item in attempts
                            ],
                            "consecutive_failures": consecutive_failures,
                        }
                    ),
                    file=sys.stderr,
                    flush=True,
                )
                if consecutive_failures >= max_consecutive_failures:
                    exit_reason = "too_many_failures"
                    return 3

            if args.verbose:
                print(
                    json.dumps(
                        {
                            "event": "frame",
                            "frame": frame,
                            "rgb": list(rgb),
                            "reconnect_requested": reconnect_first,
                            "attempt_count": len(attempts),
                            "ok": result.ok,
                            "elapsed_ms": result.elapsed_ms,
                            "ack_statuses": result.ack_statuses,
                            "error": result.error,
                        }
                    ),
                    flush=True,
                )

            frame += 1
            next_deadline += frame_period_s
            if next_deadline < time.monotonic():
                next_deadline = time.monotonic()
            sleep_until(next_deadline)

        if STOP.stop_requested and exit_reason == "completed":
            exit_reason = "signal"
    finally:
        if final_color is not None:
            final_result, _ = send_rgb_with_recovery(runtime, final_color, reconnect_first=True)
            if final_result.ok:
                last_rgb = final_color
            else:
                print(
                    json.dumps(
                        {
                            "event": "final_color_error",
                            "rgb": list(final_color),
                            "error": final_result.error,
                            "ack_statuses": final_result.ack_statuses,
                        }
                    ),
                    file=sys.stderr,
                    flush=True,
                )
        print(
            json.dumps(
                {
                    "event": "shutdown",
                    "reason": exit_reason,
                    "frames": frame,
                    "last_rgb": list(last_rgb) if last_rgb is not None else None,
                    "uptime_s": round(time.monotonic() - start_mono, 3),
                }
            ),
            flush=True,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
