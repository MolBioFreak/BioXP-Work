"""Replay an actual captured /protocol/jobs response into scratch SQLite.

Usage: PYTHONPATH=src:tests python tests/benchmark_workflow_summary.py CAPTURE OUTPUT_DIR
Fixture columns only; replay is not a runtime database backup. Timing reads use
mode=ro and the unchanged production readers; no robot connection is made.
"""
import json
from pathlib import Path
import sqlite3
import statistics
import sys
import time
from test_workflow_list_summary import reader, insert, project
from bioxp import operator_command_plane as plane

capture, output = Path(sys.argv[1]), Path(sys.argv[2])
output.mkdir(parents=True, exist_ok=True)
rows = json.loads(capture.read_text())["rows"]
store = reader(output / "jobs.db")
for sequence, row in enumerate(reversed(rows), 1):
    insert(store, row, sequence)
store.connection.close()
store.connection = sqlite3.connect(f"file:{output / 'jobs.db'}?mode=ro", uri=True)
store.connection.row_factory = sqlite3.Row
original = plane._json_load
metrics = {}
for name, call in (("full", store.list_workflows), ("summary", store.list_workflow_summaries)):
    queries, decoded = [], []
    store.connection.set_trace_callback(queries.append)
    def counted(value, *args):
        decoded.append(len(value.encode()) if value else 0)
        return original(value, *args)
    plane._json_load = counted
    result = call(limit=len(rows))
    plane._json_load = original
    store.connection.set_trace_callback(None)
    times = []
    for _ in range(100):
        start = time.perf_counter()
        call(limit=len(rows))
        times.append((time.perf_counter() - start) * 1000)
    serialization = []
    wire = ""
    for _ in range(100):
        start = time.perf_counter()
        wire = json.dumps({"rows": result}, separators=(",", ":"), ensure_ascii=False)
        serialization.append((time.perf_counter() - start) * 1000)
    (output / f"{name}.json").write_text(wire)
    metrics[name] = dict(rows=len(result), queries=len(queries), decoded_bytes=sum(decoded),
        wire_bytes=len(wire.encode()), median_read_ms=statistics.median(times),
        median_serialization_ms=statistics.median(serialization))
full = json.loads((output / "full.json").read_text())["rows"]
summary = json.loads((output / "summary.json").read_text())["rows"]
assert full == rows
assert summary == [project(row) for row in rows]
for row in summary:
    assert store.get_workflow(row["job_id"]) == next(x for x in rows if x["job_id"] == row["job_id"])
metrics["saving_percent"] = 100 * (1 - metrics["summary"]["wire_bytes"] / metrics["full"]["wire_bytes"])
(output / "metrics.json").write_text(json.dumps(metrics, indent=2))
print(json.dumps(metrics, indent=2))
