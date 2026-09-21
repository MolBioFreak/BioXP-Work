#!/usr/bin/env python3
"""Light GET-only latency sampling. Never submits moves or snapshot collections.

Use an already authorized API endpoint. No full receipt/state dumps are saved;
existing command IDs can be polled instead of repeatedly exporting stage evidence.
This measures client-observed HTTP time, not physical motion or deployment safety.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from typing import Callable
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlsplit
from urllib.request import Request, urlopen

MAX_RESPONSE_BYTES = 2 * 1024 * 1024


def sample(url: str, *, timeout: float, opener: Callable = urlopen) -> dict:
    start = time.perf_counter()
    try:
        try:
            response = opener(Request(url, method='GET'), timeout=timeout)
        except HTTPError as exc:
            response = exc
        with response:
            data = response.read(MAX_RESPONSE_BYTES + 1)
            code = response.code
        elapsed = time.perf_counter() - start
        row = {'elapsed_s': elapsed, 'http_status': code, 'response_bytes': len(data)}
        if len(data) > MAX_RESPONSE_BYTES:
            return row | {'error': 'response_too_large'}
        try:
            payload = json.loads(data)
        except (ValueError, UnicodeError):
            return row | {'error': 'invalid_json'}
        if isinstance(payload, dict):
            # Safe scalar summary; never retain provider evidence/paths/secrets.
            for key in ('command_id', 'status', 'terminal', 'state_version', 'completion_class'):
                value = payload.get(key)
                if value is None or type(value) in (bool, int) or (isinstance(value, str) and len(value) <= 160):
                    row[key] = value
        return row
    except (OSError, URLError, TimeoutError):
        return {'elapsed_s': time.perf_counter() - start, 'error': 'request_failed'}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-url', required=True, help='Authorized robot API URL (no embedded credentials)')
    parser.add_argument('--command', action='append', default=[], help='Existing command ID; may repeat')
    parser.add_argument('--count', type=int, default=10)
    parser.add_argument('--interval', type=float, default=.5)
    parser.add_argument('--timeout', type=float, default=10.)
    args = parser.parse_args(argv)
    parsed = urlsplit(args.base_url)
    if (parsed.scheme not in {'http', 'https'} or not parsed.netloc or parsed.username or parsed.password
            or parsed.query or parsed.fragment):
        parser.error('base URL must be http(s) without credentials, query or fragment')
    if not (1 <= args.count <= 1000 and math.isfinite(args.interval) and .05 <= args.interval <= 60
            and math.isfinite(args.timeout) and .1 <= args.timeout <= 120):
        parser.error('count 1..1000, interval .05..60 seconds, timeout .1..120 seconds')
    if any(not cid or len(cid) > 160 for cid in args.command):
        parser.error('command IDs must contain 1..160 characters')
    paths = ([f'/operator/v2/actions/receipts/{quote(cid, safe="")}?detail=false' for cid in args.command]
             or ['/status'])
    samples = []
    for index in range(args.count):
        for path in paths:
            row = sample(args.base_url.rstrip('/') + path, timeout=args.timeout)
            samples.append({'sample': index + 1, 'route': path, **row})
        if index + 1 < args.count:
            time.sleep(args.interval)
    durations = sorted(r['elapsed_s'] for r in samples)
    print(json.dumps({'scope': 'client_http_only_get_requests_no_motion_or_collection',
                      'samples': samples, 'median_s': statistics.median(durations),
                      'p95_s': durations[max(0, math.ceil(.95 * len(durations)) - 1)]}, indent=2))
    return int(any('error' in row or row.get('http_status', 500) >= 400 for row in samples))


if __name__ == '__main__':
    raise SystemExit(main())
