# Door handoff and worker-exit regression

Baseline: `2048334c8209cf7cf3e3019888092e1a4ce4f441`.
Scope: offline BigBoy only; no robot access, motion, deploy, restart or live DB writes.

## Proven

- The retained read-only DB has parent
  `protocol-live-fe245b5cb1606a7a2563d241347983c4a2da37a87a8e053dc7e94a42b8d86181`
  dispatched, with its first thermal-door action pending. Child
  `673a970c-b5dd-448d-96cd-5ef3d0f894ef` is completed in operator_commands,
  operator_plane_commands and serial206_movement_commands. The later stack's
  absent workflow worker is supplied incident evidence, not a new live probe.
- The actual DC handler -> finite door compiler/executor/provider -> SQLite ->
  ProtocolExecutor/service path completes the already-closed no-op and advances
  to the next action, including a deliberately stale first receipt read that
  exercises completion-before-subscription. No controller frames are emitted.
- Expiring the isolated store's lease immediately after the real child commits
  reproduces the dead-worker/dispatched-parent failure on the baseline. The next
  workflow publication raises workflow_authority_lost; exception closeout also
  demands that expired lease, then falls into the unsuitable ordinary-command
  fallback. Its failure is swallowed. The worker exits without terminal parent
  truth. This is a demonstrated failure mechanism, not historical attribution.

## Repair

Only unknown-outcome closeout may record after its own lease expires. The same
transaction still checks owner ID, dispatcher epoch and current workflow lane.
Native entry and all known-outcome closeouts retain the original lease check.
Worker exception closeout preserves the durable bundle/children and records the
exception type/message. A pre-executor setup exception handles workflow=null.
No polling, timeout extension, retry, scheduler, or admission gate was added.

## Verification

Run inside `bioxp-oem:2048334c8209`, network disabled, no devices, source and
OEM evidence read-only; pytest scratch is container tmpfs. Tests use real
provider/adapter/store owners and inert wire replies. Child scheduling is
controlled by the test admission adapter, not the live dispatch loop. The
post-door LED is a host witness, not a physical LED test.

- Final baseline owner-module overlay (unchanged baseline module, final tests):
  `tests/test_door_handoff.py tests/test_debloat_child_waiter.py`
  -> **2 failed, 12 passed in 15.31s**. Failures: expired-lease parent remains
  dispatched; setup exception discards original exception/bundle information.
- Candidate focused: same files -> **14 passed in 15.39s**.
- Candidate extended: those files plus `tests/test_protocol_oem_provider_bindings.py`
  and `tests/test_workflow_park_rehome.py` -> **48 passed in 39.43s**.
- All final runs use `-q --tb=short -W error::pytest.PytestUnhandledThreadExceptionWarning
  -p no:cacheprovider`.

Exact Docker command and logs are retained at
`/home/dalab/.hermes/profiles/fresh/cache/scratch/door-handoff-qualification/`
on BigBoy. Baseline comparison overlays the sole changed production module,
not a separately claimed pristine full-tree run.

## Unresolved

The historical worker exception/stack at the moment of exit was not retained.
The later lane lease is renewed, which neither proves nor excludes transient
expiry. Neither the old subscription repair nor this injected lease-expiry
repair is proven to explain/cure that exact incident. Hardware acceptance is
not performed. Owner replacement and persistent SQLite write failure still
require the existing recovery owner; this repair does not override them.

Production overlap for integration: `src/bioxp/operator_command_plane.py` only
(_workflow_current, finish_workflow, _dispatch_worker). No service/executor/API
or pacing files changed. New regression: `tests/test_door_handoff.py`.
