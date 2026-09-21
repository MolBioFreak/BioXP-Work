# Live-source latency repairs — candidate review branch

**Status:** offline-qualified candidate changes, not deployment or hardware acceptance.
**Branch:** `test-latency-20260920`.
**Base:** `207aac9cc40130241c85bf86516a223f893d8254` on `test`.
**Base tree:** `6a309b056325d1369eea085fbe607ca44d0c99b7`, identical to reported live
`9eeb6a147a856e080635796755e280155c01c35d` (V13).

No change to `test`, `main`, installed release, robot database, OEM evidence lock,
motor commands, speeds, clamps, native waits, USB reply matching, or physical
Stop/Abort delivery is part of this series. No new schema version or migration.
All 13 canonical migration identities were computed on base and candidate and
matched exactly. A synthetic V13 database created by the base was opened by the
candidate with the same schema version and unchanged migration ledger. This is
not a rehearsal against the robot's retained 615 MB database.

## Evidence supplied by Hermes (not independently measured here)

The 2026-09-20 RCA measured 26 deck moves at 7.1–13.3 seconds, median 9.2 seconds;
manual cockpit moves with a collector active took 18.3/20.1/27.2 seconds. Actual
motion was about 1–1.5 seconds. `/status` was 0.55–1.48 seconds and a full collection
about 7.3 seconds. Sustained idle CPU was about 65–70% of one thread for Python,
plus about 8% for PRoot. The 25-second profile attributed substantial worker time
to deepcopy, JSON and reference-database opening; dispatcher samples concentrated
in SQL execute. The reported 22 rows/12 tables and 31 KB provider result remain
important, but are not by themselves a measurement of serialization/commit cost.
The historical DB sizes and wire/anatomy measurements remain Hermes's observations.

Source review confirmed these mechanisms. It also established that named-deck
admission is separate from the direct-action full-refresh fallback; not every
named deck move performs that fallback. Two recorded latch predicates are not
proof of two duplicate bus reads. The ordinary offset path already retains two
active samples and avoids a third post-pseudo-home sample. These distinctions are
preserved; no motion or latch check was removed to improve timing.

## Implemented changes and boundaries

### K1 — dispatcher idle work

`OperatorCommandStore.claim_next()` first performs an indexed, read-only queued
existence check. An empty queue no longer enters a writer transaction or walks
historical recovery rows. A real claim retains all existing authoritative checks
inside the original transaction.

The loop consumes its Event BEFORE scanning durable/memory predicates, not after
waiting. Admissions, finishes, controls and interrupts use existing notifications;
worker exit now also notifies after capacity is released. A one-second fallback
covers external work and lease renewal. Normal same-process requests wake
immediately; direct external DB writers without notification can wait for that
fallback. Five-second lease semantics and fail-closed ownership loss are unchanged.
The recovery query itself is not rewritten and no index is added to the frozen
schema. Its cost during genuine admission remains a profiling follow-up.

### K6 — reference connection churn

`ReferenceStateStore` retains one handle serialized by its existing object lock,
uses explicit cross-thread support, and never retains an open transaction between
operations. It checks process/file identity and SQLite schema version before
reuse, revalidates table/trigger structure on schema change, and fails closed on
file replacement. Every read still selects and validates the current reference
payload; there is no reference-readiness cache. Failed writes discard unfinished
connections; explicit recovery remains required for untrusted authority.
The API closes this handle after physical producers have quiesced at shutdown.

### K3 — repeated state reconstruction

The runtime store selects the current authority row for each unscoped read and
memoizes integrity verification only for an exact match of sequence, state bytes,
state digest, receipt-set bytes and receipt-set digest. A changed byte string is
not accepted merely because its stored digest/sequence is unchanged. Every call
still gets one fresh JSON decode and a private mutable result.

The production reader explicitly declares that detached, verified-JSON contract.
The provider avoids deepcopying that already-private result and avoids a redundant
JSON validation dump after the store has verified it. Structural validation and
restart/reference/owner reconciliation still execute. Memory-only and other
readers keep defensive copies and ordinary validation. Migration defaults are
built once; fully current state bypasses no-op upgrade construction. Legacy
missing trays/construction IDs are still not fabricated.

This is not a TTL cache of "ready", a shared mutable state object, or permission
to skip command authority checks. Repeated guard calls still occur and still
read current authority; their repeated decode/copy cost is reduced rather than
replacing them with stale stamps.

### K4 — receipt expansion and host-only persistence

V2 compact fallback reads use an internal scalar summary rather than fetching
request plans, full terminal payloads and all stage evidence before discarding
them. The existing compact serializer and externally visible field semantics are
retained, including recovery-required completion-class correction. Full detail
and recovery readers are unchanged. A detail request decodes each stage evidence
once instead of twice.

The two host-only latch predicate stage publications share one transaction,
retaining the existing per-stage writers and checks under provider-before-writer
lock order. No motor or sensor operation is moved inside that transaction. The
records either both commit before physical delivery, or both roll back. After a
crash before commit, neither predicate is durably terminalized; no move was
allowed past this boundary. ForceToHighHome still records its command identity
and semantic transition even when the numeric pseudo-home is already 500.

The source return, delivery records, full provider evidence, terminal disposition,
semantic history, versions and transactional outbox remain durable. This series
does NOT make terminal evidence asynchronous, announce early completion, drop
history, or hold a database transaction across motor execution. The 31 KB payload
and most row writes are deliberately retained; a deeper normalization is a
separate reader/schema/recovery migration, not claimed fixed here.

### K5 — immutable planning work

Static catalog entries and catalog revisions are reused by exact immutable input
keys with bounded caches. Required table destinations are still checked on each
call; each catalog object and response list remains independent. Current board
identity, references, observations, position tables and executable command plans
are NOT reused as authority. Initial/final pre-TX sampling and both latch
predicates remain unchanged. Dynamic plan rebuilding is not eliminated.

### K2 — collection competition

At most one automatic HTTP collection may be queued/running per process. A second
automatic request returns `ok=false`, `published=false`, with
`hardware_collection_in_progress` (or the existing action-pending response when
that check wins), instead of accumulating a sweep backlog. The slot is held until
the actual retained worker finishes, even after HTTP timeout/disconnection.
Existing yield/preemption and failed-observation invalidation rules remain.

Direct-action motion prerequisite refresh uses transport/boards/power/interlock/
latch/axes/gripper, not thermal/chiller/pipette/camera diagnostics, and does not
warm the all-target deck catalog. It remains command-owned, not automatic, so it
does not preempt itself. Named deck commands keep their separate seam. Explicit
full diagnostic requests and workflow thermal monitoring are unchanged.

Callers that omit `automatic=true` are still explicit requests; this series does
not guess that they are background polling. Inventory the deployed cockpit/BMS
callers. Thermal/chiller reads are not globally disabled and stale timestamps are
never refreshed without observations.

### K7 — shared interpreter contention

The series removes CPU/SQL amplification in the shared interpreter. It does not
add API workers, change the GIL, split hardware ownership, or claim elimination
of every collector/command lock wait. Re-measure with collectors active.

### K8 — harness overhead

`scripts/bioxp_latency_probe.py` provides light GET-only timing of `/status` or
existing compact receipts. It never submits moves, posts a collection, reads the
robot DB, or saves large provider payloads. It is an alternative observation tool;
the external 26-location sweep harness was not present in the repo and is not
modified. Do not count its evidence-capture time as robot command latency.

## Tests and repeatable commands

Run the portable suite with the development interpreter from the repository root:

```bash
PYTHONPATH=src:. python -m pytest -q \
  tests/test_latency_dispatch_idle.py \
  tests/test_latency_reference_connection.py \
  tests/test_latency_verified_state.py \
  tests/test_latency_compact_receipts.py \
  tests/test_latency_collection_admission.py \
  tests/test_latency_deck_bookkeeping.py \
  tests/test_latency_light_probe.py \
  tests/test_serial206_state_copy_latency.py \
  tests/test_review_direct_board_epochs.py \
  tests/test_review_z_reference_publication.py \
  tests/test_review_command_axis_fences.py \
  tests/test_z_oem_api_contract.py \
  tests/test_protocol_workflow_api.py \
  tests/test_protocol_executor_finalization.py \
  tests/test_protocol_source_children.py \
  tests/test_protocol_source_lifetime.py
```

The new tests cover idle scan counts, notification races, external fallback and
owner loss; reference thread handoff, schema/inode replacement, failed writes and
external desync; byte/digest tampering and detached state; compact-field parity;
automatic-refresh overlap and retained-worker cancellation/timeout; catalog digest
identity and output independence; predicate transaction rollback; and bounded,
GET-only harness behavior. Tests requiring a captured native bundle/retained DB
remain separate. Do not fabricate them or disable their integrity checks.

The final portable suite passed **179 cases locally** (44 new regression cases
plus 135 existing cases). The local log and JUnit file are retained in the review
packet. Hosted qualification is a separate publishing gate; its result is not
physical or OEM-parity acceptance. An attempt to run `test_deck_command_latency`
stopped at fixture setup because the locked native bundle was unavailable; that
suite is not counted as passed.

Local interpreter: Python 3.13.5, pytest 9.0.2, FastAPI 0.128.2, Pydantic 2.13.4,
httpx 0.28.1, rfc8785 0.1.4, pyusb 1.3.1, python-can 4.6.1. The local environment's
wrapt is 2.2.1, outside python-can's declared ~=1.10 dependency; local portable
checks did not exercise CAN. Hosted validation installs the repo requirements
on Python 3.12. Neither substitutes for the robot's locked interpreter.

## Microbenchmark evidence — NOT robot timings

A generated 100,406-byte state, 200 calls per measurement, on the authoring host:

| Operation | Base 207aac9 | Candidate |
|---|---:|---:|
| Provider state load, mean | 4.239 ms | 0.485 ms |
| Reference snapshot, mean | 3.295 ms | 0.049 ms |

These compare the real methods in separate base/candidate interpreters, with
synthetic data and no hardware. They are indicative mechanism measurements, not
promised whole-move speedups. Reproduce on either source tree with the candidate
script path and that tree on PYTHONPATH:

```bash
PYTHONPATH=src:. python /path/to/candidate/scripts/bioxp_latency_microbench.py
# Light observation of an existing command (never creates a move):
python scripts/bioxp_latency_probe.py --base-url http://ROBOT_API \
  --command EXISTING_COMMAND_ID --count 10 --interval 0.5
```

## Host-side acceptance before adoption

Build a new candidate release from this branch through the existing source and
runtime admission process. First qualify on a consistent copy of the live V13 DB,
with the locked interpreter and native evidence. Re-run especially:
`test_deck_command_latency`, post-move reference/finalization/Stop tests,
`test_deck_automatic_refresh_owner`, readiness-cost and poll-lock-order tests,
`test_homexy_reference_publication`, `test_critical_images_current_reconciliation`,
and the worker's recovery/parity selection. Existing source-dependent fixtures
are unavailable in the authoring environment.

Verify: stale epochs reject before TX; changed-key binding conflicts; same-intent
replay never transmits; final pre-TX latch/reference/owner drift still rejects;
interrupt delivery is independent of SQL/collector backlog; host predicate batch
failure rolls back before movement; and V13 decision/history semantics remain.
Retained receipt equivalence is required both before and after governed recovery.

Then repeat the existing approved supervised timing workflow, with light capture,
collector idle/active and cockpit open/closed. Record idle CPU, `/status` p50/p95,
admission-to-first-TX, wire motion time, motion-complete-to-durable-terminal time,
reference opens, state decode/dump/copy counts, and collection caller/domain/
automatic/preemption information. Do not disable safety or run new movement solely
because this branch exists. BMS 502 response-contract repair remains separate.

## Deliberately excluded and rollback

No journal pruning, retention migration, fsync weakening, removal of delivery or
recovery evidence, unchecked position/reference caching, removal of latch reads,
new hardware-owning process, or replacement of the OEM motion implementation.
The 615 MB journal still needs a governed retention/archive design that preserves
all incoming references and replay/reconciliation readers. Deleting append-only
rows is not a performance shortcut in this patch series.

These commits add no persistent fields or formats and keep the V13 registry
unchanged. Roll back code through the normal quiesced release process; do not roll
back the runtime DB or reissue uncertain physical commands. The pre-existing
board-epoch receipt compatibility restriction from the earlier fixes still applies
when considering a downgrade past those fixes.
