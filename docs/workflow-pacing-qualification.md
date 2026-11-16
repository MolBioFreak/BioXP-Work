# Workflow publication pacing (offline)

The receipt-update trigger was appending a report snapshot for every workflow
publication, although those snapshots do not contain receipt_json. They repeated
unchanged multi-MB requested inputs. V16 skips only those workflow updates whose
report fields (apart from the publication timestamp) are unchanged. It preserves
all existing history and still snapshots status, authority and input changes.
Receipt durability, plane state-version increments and control/recovery publication
remain synchronous. No coalescing, extra ledger, deferred flush or hardware change.
The child-ID projection now copies only its modified dictionary path, not the
whole scientific bundle through JSON twice. Full receipt reads do not also decode
the admission bundle unless no receipt exists.

## Qualification

Host: BigBoy. Runtime: bioxp-oem:2048334c8209, candidate src mounted read-only,
network disabled. The source store was mounted read-only and SQLite-backed up to
scratch; no robot connection, service restart or deployment.

Source: workflow-child-handoff/live-store/bioxp_runtime.db; retained job
protocol-live-fe245b5cb1606a7a2563d241347983c4a2da37a87a8e053dc7e94a42b8d86181.
Baseline methods extracted from 2048334, baseline trigger unchanged. Candidate
methods and V16 trigger applied to an independent identical scratch copy.
Twelve real publish_workflow transactions per case, each with a different runtime
held_reason, using actual authority checks, WAL/FULL synchronization and normal
256-page auto-checkpointing. Only the copied lane lease was renewed; no dispatcher
or native worker was started. This is owner publication timing, not robot action
gap acceptance. The production constructor/recovery lifecycle was intentionally
not invoked in the timing harness, to retain the captured active parent.

| Measurement | Before | After |
| --- | ---: | ---: |
| Median publication | 104.477 ms | 48.006 ms |
| Total for 12 publications | 1.381184 s | 0.611885 s |
| Added report versions | 12 | 0 |
| Added report snapshot bytes | 35,890,592 | 0 |

The full current receipt remains durable at every boundary. These workstation
measurements do not establish elimination of the observed 10–20 s robot gaps.
Raw timings and executable harness remain in BigBoy scratch/workflow-pacing/
(benchmark.json, benchmark.py, before.py).

Focused command inside the production image:

```
python -B -m pytest tests/test_workflow_publication_pacing.py \
  tests/test_protocol_workflow_connected.py tests/test_workflow_park_rehome.py \
  -o addopts= -p no:cacheprovider -q
```

`WORKFLOW_PACING_BASELINE` points to the read-only source DB;
`DECK_RETAINED_BASELINE` points to a quiesced scratch copy of that DB.
Result: **19 passed, 1 warning in 49.52s** (Starlette httpx deprecation).
The regression covers publication/readback, plane versions, no duplicate report
snapshots, stale-owner refusal, terminal snapshot retention, active-migration
refusal, full V15-to-V16 backed-up migration, schema attestation and reopen.
Connected tests cover idempotency, review, execution, custody and Park/rehome.
