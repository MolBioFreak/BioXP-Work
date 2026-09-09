# Operator responsiveness changes

## Scope and boundaries

These changes address the identified pre-dispatch copying/receipt verification cost,
blocking status handlers, oversized polling payloads, completion-blocking action
responses, automatic snapshot contention, stale board-member authority, Y failure
reporting, and receiver-audit commit contention. They do not change OEM move
coordinates, homing sequences, completion waits, Stops, Abort, No24V behavior,
controller acknowledgement semantics, or physical verification requirements.

Software cancellation and addressed controller Stops remain distinct. No motion
retry or automatic recovery has been added.

## API behavior

- Canonical non-deck v2 actions return the existing receipt after durable admission,
  not after physical completion. Legacy X-panel action admission uses the same
  retained execution path. The initial receipt can be `queued` and nonterminal.
- Follow the returned `status_path` to terminal disposition. A queued receipt is
  not evidence of controller delivery or physical completion. Dispatch is persisted
  before the provider call. Ambiguous execution is not retried.
- Same-key/same-input replay returns the same receipt; conflicting key reuse fails.
  A second distinct normal action receives `409 operator_action_busy`, rather than
  silently waiting behind a long action. Interrupt routes retain their own path.
- Compact catalog/dashboard/history receipts omit wire exchanges. The explicit
  receipt-detail endpoint still loads their retained exchanges and evidence.
  History selects ordering metadata before hydrating the selected receipt page;
  it retains legacy/direct ordering, identity deduplication and cursor behavior.
  This reduces payload hydration; it does not claim an indexed O(1) archive search.
- `POST /hardware/snapshot/collect` accepts `{"automatic": true}` for background
  observation. It can return `published: false, reason: operator_action_pending`.
  It declines to queue behind an active tester/normal action and yields at safe
  observation boundaries. An in-flight controller query is not forcibly cancelled.
  Partial collection is not published over the previous snapshot. Explicit
  collections retain their existing behavior when `automatic` is absent/false.
- A thermal/chiller board-presence query reads firmware only. Full parameter-bank
  diagnostics remain in their explicit collectors, not duplicated in board lookup.

## Authority and persistence

- Status projection is passive. Live X terminal readback remains available through
  its explicit status endpoint, not repeated by each general status projection.
- Serial-206 state/receipt integrity verification can be reused only inside one
  coordinated projection scope. Local writes and external-connection data-version
  changes invalidate the scoped result. Results do not survive the scope.
- Hardware projections copy selected rows once, under the snapshot lock. Independent
  domain collection preserves an available domain when a sibling is missing;
  ownership invalidation still removes its authority.
- Board-4 transitions invalidate stale Y/gripper members even when the Z transition
  is provider-owned. Stale prepared epochs are not projected as referenced-ready.
  XY admission refuses non-current Y authority before provider/controller entry.
- Y completion exceptions retain move submission/acknowledgement, target/readback,
  failed stage, and whether the actual homing sweep started. Acknowledgement is
  not completion. Without positive evidence, motion submission is unknown, not false.
- Receiver audit records are committed in bounded batches through the existing
  SQLite writer/coordinator. FULL/WAL durability remains; there is no parallel log
  authority, per-row dropping policy, or receiver-side database waiting.
- Idle reconciliation uses a monotonic interval. An interrupt wake bypasses the
  idle interval; it must not be consumed by clearing the wake event first.

## Verification and limits

`tests/test_operator_responsiveness.py` exercises the new hot-path contracts with
isolated SQLite and fake I/O, including early durable admission, no duplicate
provider invocation, active-action rejection, projection isolation, verification
cache invalidation, passive X reads, batch commits, preemption without publication,
event-loop responsiveness, Y evidence, and prompt interrupt wake-up.

Existing operator, persistence, receiver, Y/XY, board-authority, snapshot, and deck
integration regressions remain required. The paired BMS candidate must retain
last-known display/history without using stale state for motion admission.

Offline benchmarks and test counts are not live acceptance. The historical Y
failure to move from 10,767 to its 10,000 preliminary target remains mechanically
unresolved. Its OEM twenty-second completion wait has deliberately not been
shortened. Live latency, loaded audit throughput, Y limit/power/controller state,
and motion require separate fresh controller/camera evidence and authorization.
