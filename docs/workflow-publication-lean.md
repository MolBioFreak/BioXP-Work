# Lean workflow publication — offline qualification

Base: `685f60ebeacf1b425988c2758cdd95a6dfa1571e`.
Worktree: `/home/dalab/robot/wt-workflow-publication-lean` on BigBoy.

## OEM source inspected before implementation

Source root: `/home/dalab/Migration-Recovery/old-app-state/.hermes/profiles/fresh/robot-audit/rea-core-mapping-20261016/oem-cs`.

- `BioXPControlLib.dll/BioXPControlLib/ControlLib.cs:4928–4947`: the decompiled execution entry is `executeScript`; it constructs its in-memory statement list once. This source does not name that entry `RunJob`.
- `ControlLib.cs:5164–5247`: execute the statement, update line/time/stage progress, invoke `updateTime`/`updateDBCStatue`, and update the displayed finish time. This inspected loop does not serialize the authored script/job bundle each statement.
- `ControlLib.cs:5111–5118, 5139–5161, 5262–5275`: actual pause/stop branches and component waits remain separate from progress display.
- `ControlLib.cs:5370–5381`: `saveProcessTime`, status reporting and pressure collection occur in `finally`.
- `CommonLib.dll/CommonLib/ClassTimeEstimate.cs:209–224`: progress updates mutate time/index fields. `:241–258` saves timing entries to `processtime.xml`, not the authored job.

Contrast: the native Python baseline encoded the whole multi-MB bundle, updated its receipt, advanced the plane version, then read/decoded the whole bundle back on every callback. V16 only suppressed duplicate report snapshots. The dispatcher also immediately repeated the authority check already performed by `workflow_context`.

## Repair

- Store new authored bundles once using the **existing** immutable `operator_plane_evidence` owner (`workflow_input`). Admission/report rows contain a small reference and initial mutable receipt, not another document. The plane request carries only dispatch/authority metadata and the reference.
- Keep mutable execution/control state in the existing `operator_commands.receipt_json`. No new table, replay log, background flusher, progress worker, or resumable checkpoint.
- Exclude protocol/document, live contract, and idempotency binding from progress/terminal receipt encodes. Reconstruct those fields for explicit full reads.
- Compare meaningful mutable state, excluding publication timestamps. Identical publication calls do not UPDATE either command table and do not advance the version.
- Update changed mutable subtrees in SQLite; append new history-list entries without Python re-encoding the preceding event/action-result history. No replay semantics are attached to these lists.
- Return publication metadata/child IDs without `get_workflow` and its full input decode. Control and wake inspection use compact observations. Remove the immediately duplicated assertion inside already-validated `workflow_context` wrappers.
- Preserve the final callback's owner/lease check. An attempted deletion exposed an existing expired-lease exception contract; the callback remains, but its unchanged write is eliminated.
- Compact unknown-outcome payloads before terminal copying. Preserve actual worker exception details and all existing pause/abort/owner/epoch/unknown-outcome behavior.

No DDL or migration identity changed. The existing evidence and JSON receipt owners accept these records. New records have an explicit internal receipt marker; unmarked historical full receipts retain whole-receipt precedence. There is no bulk migration or rewrite of past records. Older application binaries do not understand the new input references, so a code rollback must not assume new records are readable by an older binary.

## Consumers

- Full job GET/list: hydrate authored input plus current mutable execution, retaining the existing public bundle shape.
- Compact observation/list summaries: select small mutable fields and admission summary; never hydrate authored input. Tests replace the hydration method with a failure to verify this.
- Shared read-only `CommandReceiptReader`, canonical command detail, and `OperatorHistoryReader`: hydrate public requested inputs and terminal evidence using the same reader.
- Control/wake: retain gate/version/ownership semantics using compact reads.
- Startup recovery: compact receipts retain the same runtime/workflow paths, so actual killed-worker recovery preserves evidence and refuses replay.
- Worker exceptions: retain the last durable runtime and exception fields, then publish ambiguous state without serializing authored input.
- Export: the reconstructed full bundle is saved through the real `ProtocolOperatorBundleStore`; exported protocol and absence of internal markers are asserted.
- BMS context: full job and compact public contracts remain unchanged; no BMS source changed. This qualification does **not** claim a fresh BMS/browser run.

## Measured complete execution

Production image `bioxp-oem:2048334c8209`, network disabled, source/OEM/retained fixtures mounted read-only, private scratch SQLite, real store constructor/admission/claim/dispatcher/ProtocolExecutor/terminalization. Only LED physical leaves are inert. The actual scratch owner lease is renewed to 120 seconds before synchronous execution. No live database or robot is opened.

Both versions use identical authored documents containing approximately 3 MB of metadata and either 2 or 64 actions. Every original publication is immediately repeated to test no-change behavior. Thus complete-run timings and write totals include intentional duplicate probes; these are not claimed as production robot inter-action latency. The publication median measures original calls, not the duplicate probes. Admission and export are outside the execution timer. Report counters include dispatch and terminal snapshots.

| Measurement | Baseline, 2 actions | Candidate, 2 actions | Baseline, 64 actions | Candidate, 64 actions |
|---|---:|---:|---:|---:|
| Execution plus duplicate probes (s) | 0.823845 | 0.065625 | 13.304339 | 1.187927 |
| Median original publication (ms) | 45.626 | 5.081 | 45.278 | 5.655 |
| Receipt UPDATE count | 23 | 11 | 395 | 197 |
| Receipt JSON bytes written | 69,053,562 | 17,499 | 1,195,563,368 | 1,447,075 |
| Largest receipt bytes | 3,002,548 | 1,820 | 3,032,710 | 13,328 |
| Report snapshot count | 2 | 2 | 2 | 2 |
| Report JSON bytes written | 6,005,943 | 4,521 | 6,048,706 | 4,518 |
| Terminal-plane JSON bytes written | 3,002,548 | 1,820 | 3,032,710 | 13,328 |
| Python bytes encoded during original publications | 33,025,507 | 4,345 | 596,265,329 | 458,781 |
| Publications encoding authored input | 11 | 0 | 197 | 0 |
| Duplicate probes changing rows | 11 | 0 | 197 | 0 |

The candidate stores exactly one authored evidence record, no authored marker in admission/plane/report rows, and performs zero input-evidence updates during execution. Counters are actual SQLite-trigger observations of logical UTF-8 JSON values, **not physical filesystem/WAL/device byte counts**. The existing SQLite pager still writes mutable-row pages.

Raw per-publication timings/counts: `workflow-publication-lean-measurements.json`.
Exact production-image invocations: `workflow-publication-lean-commands.txt`.

## Regression

Final combined roster: **127 passed, 0 failed, 0 errors, 0 skipped**, parsed JUnit time **240.433 s**. This is the prior 124-case tip/Z, pickup, eject, door, child waiter, native bindings, connected workflow, Park/rehome and V16 control/crash roster plus three new large-document/legacy-read cases. One existing Starlette/httpx deprecation warning.

The V16 crash test now compares the recovered document against its independent immutable evidence record, rather than requiring a duplicate document inside the progress receipt. Actual SIGKILL, lease expiry, cold reopen, unknown outcome, control refusal, and no-second-leaf assertions remain.

Evidence on BigBoy: `/home/dalab/.hermes/profiles/fresh/cache/scratch/workflow-publication-lean/`:
`baseline-red.log`, `baseline-final.log`, `candidate-qualified.log`, `acceptance-final-readers.log`, `acceptance-final-readers.xml`. Initial baseline bind-mount failure and the expired-lease regression failure are retained separately, not counted as successful acceptance.

## Remaining costs and limits

Mutable receipts still grow with real execution history. Python reads/compares the current mutable receipt; SQLite JSON updates rewrite that mutable value internally. Changed stage/source-model subtrees are encoded as units, and canonical child IDs are queried in admission order. Terminal mutable evidence is still stored once in each pre-existing terminal owner; full explicit GET/export reconstructs the document and export writes a full bundle once. This is not an O(1) total-history-storage claim.

No deployment, push, service restart, robot access, live DB writes, physical acceptance, or BMS browser acceptance occurred.
