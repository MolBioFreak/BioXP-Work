#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))



def load_tester_class():
    from src.bioxp.usb_driver import BioXpTester

    return BioXpTester



def fmt_hex(frame: list[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in frame)



def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")



def frame_record(direction: str, frame: list[int], **extra: Any) -> dict[str, Any]:
    return {
        "timestamp": utc_timestamp(),
        "timestamp_ns": time.time_ns(),
        "direction": direction,
        "frame": list(frame),
        "hex": fmt_hex(list(frame)),
        "length": len(frame),
        **extra,
    }



def write_records(records: list[dict[str, Any]], output_path: str | None) -> None:
    lines = [json.dumps(record, sort_keys=True) for record in records]
    if output_path:
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        return
    for line in lines:
        print(line)



def capture_command(args: argparse.Namespace) -> dict[str, Any]:
    tester = load_tester_class()(alt=args.alt)
    records: list[dict[str, Any]] = []

    request_frame = list(tester._build_frame(args.board, args.command, args.type, args.motor, args.value))
    records.append(
        frame_record(
            "tx",
            request_frame,
            board=args.board,
            command=args.command,
            type=args.type,
            motor=args.motor,
            value=args.value,
        )
    )

    reply = tester.send_tmcl(
        board_id=args.board,
        command=args.command,
        cmd_type=args.type,
        motor=args.motor,
        value=args.value,
        wait_reply=not args.no_wait_reply,
        write_timeout_ms=args.write_timeout_ms,
        read_timeout_ms=args.read_timeout_ms,
        max_reads=args.max_reads,
        strict_match=args.strict_match,
    )

    if reply and reply.get("raw"):
        records.append(
            frame_record(
                "rx",
                list(reply["raw"]),
                board=reply.get("board"),
                command=reply.get("cmd"),
                status=reply.get("status"),
                status_str=reply.get("status_str"),
                value=reply.get("value"),
            )
        )

    if args.events_after > 0:
        events = tester.collect_bus_events(
            duration_s=args.events_after,
            timeout_ms=args.event_timeout_ms,
            max_events=args.max_events,
        )
        for event in events:
            records.append(
                frame_record(
                    "event",
                    list(event["raw"]),
                    board=event.get("board"),
                    command=event.get("cmd"),
                    status=event.get("status"),
                    status_str=event.get("status_str"),
                    value=event.get("value"),
                )
            )

    write_records(records, args.output)
    return {
        "mode": "command",
        "records_written": len(records),
        "output": args.output,
        "reply_received": bool(reply),
    }



def capture_listen(args: argparse.Namespace) -> dict[str, Any]:
    tester = load_tester_class()(alt=args.alt)
    events = tester.collect_bus_events(
        duration_s=args.duration,
        timeout_ms=args.timeout_ms,
        max_events=args.max_events,
    )
    records = [
        frame_record(
            "event",
            list(event["raw"]),
            board=event.get("board"),
            command=event.get("cmd"),
            status=event.get("status"),
            status_str=event.get("status_str"),
            value=event.get("value"),
        )
        for event in events
    ]
    write_records(records, args.output)
    return {
        "mode": "listen",
        "records_written": len(records),
        "output": args.output,
        "duration_s": args.duration,
    }



def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture BioXP TMCL session traffic as JSONL.")
    parser.add_argument("--alt", type=int, default=1, help="USB interface alternate setting (default: 1).")
    subparsers = parser.add_subparsers(dest="mode", required=True)

    cmd = subparsers.add_parser("command", help="Send one TMCL request and capture the request/reply pair.")
    cmd.add_argument("--board", type=lambda text: int(text, 0), required=True)
    cmd.add_argument("--command", type=lambda text: int(text, 0), required=True)
    cmd.add_argument("--type", type=lambda text: int(text, 0), default=0)
    cmd.add_argument("--motor", type=lambda text: int(text, 0), default=0)
    cmd.add_argument("--value", type=lambda text: int(text, 0), default=0)
    cmd.add_argument("--output", help="Write JSONL records to this path. Defaults to stdout.")
    cmd.add_argument("--no-wait-reply", action="store_true", help="Send without waiting for a reply.")
    cmd.add_argument("--write-timeout-ms", type=int, default=80)
    cmd.add_argument("--read-timeout-ms", type=int, default=30)
    cmd.add_argument("--max-reads", type=int, default=12)
    cmd.add_argument("--strict-match", dest="strict_match", action="store_true", default=True)
    cmd.add_argument("--no-strict-match", dest="strict_match", action="store_false")
    cmd.add_argument("--events-after", type=float, default=0.0, help="Collect async bus events after the command for this many seconds.")
    cmd.add_argument("--event-timeout-ms", type=int, default=20)
    cmd.add_argument("--max-events", type=int, default=96)
    cmd.set_defaults(func=capture_command)

    listen = subparsers.add_parser("listen", help="Collect asynchronous TMCL events without sending a command.")
    listen.add_argument("--duration", type=float, default=0.5)
    listen.add_argument("--timeout-ms", type=int, default=20)
    listen.add_argument("--max-events", type=int, default=96)
    listen.add_argument("--output", help="Write JSONL records to this path. Defaults to stdout.")
    listen.set_defaults(func=capture_listen)

    return parser



def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        summary = args.func(args)
    except Exception as exc:  # pragma: no cover - hardware path
        print(json.dumps({"ok": False, "error": str(exc), "mode": args.mode}, sort_keys=True), file=sys.stderr)
        return 1

    print(json.dumps({"ok": True, **summary}, sort_keys=True), file=sys.stderr if args.output is None else sys.stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
