# F12: Z Stop lifecycle bypass and retained reply serialization

## Approved boundary

Z Stop/abort is classified before the API's ordinary Z command lease. It uses
existing interrupt dispatch, then existing lifecycle reconciliation. Ordinary
intents still require their normal lease. This bypasses only the additional
motion-lifecycle lock; it does **not** preempt an outstanding synchronous reply.
There is no immediate Stop or physical-effect guarantee.

Christian's approved F12 boundary preserves OEM transport ownership, timeouts,
and source-defined retry placement. No transport lock, timeout, retry policy,
Stop send count, ignored-return treatment, or terminal verification policy is
changed here. The two source Stop sends remain; a source return of zero does
not mean physical Stop was verified. Existing terminal policy can return HTTP
409 after dispatch.

## Source and lock boundaries

- OEM `ClassCanLib/ClassNovo.cs:194–226`: `m_sendingLock` surrounds
  `sendCommand`, with the source sleeps and exception handling retained inside
  that ownership interval.
- OEM `NovoCANUSBLib/ClassNovoCANUSB.cs:525–559`: ordinary reply wait, and only
  on its timeout, retransmit and second reply wait. Thus an outstanding exchange
  and its source-defined retry can delay a later Stop. These are decompiled
  source anchors, not a new certification of missing OEM library binaries.
- Operator controls already choose a separate interrupt lock. API Stop uses
  its reserved interrupt executor and tester transition lock; concurrent
  interrupts and connection transitions can still serialize execution.
- Provider interrupt dispatch uses its interrupt-dispatch lock and brief state
  lock. It sends through the existing adapter before acquiring the lifecycle
  lock for durable reconciliation.
- `BioXpTester.send_tmcl` holds the device transport guard across the real
  `NovoRouter.transact`; the router transaction lock covers write and reply
  wait. F12 retains both. The nominated synchronous exchange uses 60000 ms,
  unchanged. This is not a maximum end-to-end Stop latency: source retry,
  other queued ownership, scheduling, and Stop's own exchanges also matter.
- Asynchronous target-event waiting holds the transport guard only during
  individual collection windows, not the entire motion lifecycle wait. This
  allows Stop dispatch with transport available while reconciliation still
  waits for the lifecycle lease.
- Router pending/completion locks and motor pacing are unchanged.

The external `wp0-source-reconciliation.md` (F03, ordinary transport and
`TransmitMessage` rows) supplies the governing family-specific retry and
ignored-return reconciliation. F12 does not implement or certify other work
packages' retry changes.

## Offline evidence

`tests/test_z_stop_outer_lease.py` exercises real API function/reserved executor,
provider, primitive adapter, tester motor methods, router transactions and
SQLite reconciliation with synthetic endpoints/configuration and seeded ready
state. It is not an HTTP operator-router or hardware acceptance test.

- `async-wait`: withhold the asynchronous target event, observe both Stop USB
  writes while the ordinary move still holds its lifecycle lease, and confirm
  no Stop result/receipt yet. Release the target event, then inspect the Stop
  receipt and reopen durable storage.
- `sync-reply-delay`: withhold the first ordinary synchronous response. Observe
  an actual failed Stop transport-lock acquisition, an unset pending reply and
  a held router transaction lock, with zero Stop writes. Supply the accepted
  synthetic reply (no shortened timeout or fake move/wait return), verify the
  actual transaction's completion and unchanged 60000-ms timeout, then observe
  both Stop writes before releasing the asynchronous target event. Reconcile
  afterward using the same durable-state assertions as the first case.

Both retain source return zero, two Stop writes, physical-effect false, the
interrupted ordinary command ID, failed-latched/desynced state and no active
receipt after reconciliation. The synchronous case characterizes approved
existing behavior; it is not a newly repaired defect or invented RED.

Test coordination budgets are 2 seconds for phase/event arrival, 1 second for
Stop delivery observation, an 8-second requested asynchronous wait, and
10-second thread joins; the synthetic router read window is 10 ms. These are
offline harness bounds, not measured hardware latency or physical guarantees.
The accepted-response case does not measure the full timeout/retry path.
Hardware latency requires a separately approved user-only run.
