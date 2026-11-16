# Integrated native workflow repairs — offline qualification

Base: 2048334c8209cf7cf3e3019888092e1a4ce4f441.
Isolated BigBoy branch: fix/workflow-repair-integrated.
Worktree: /home/dalab/robot/wt-workflow-repair-integrated.

Reviewed and cherry-picked 2e0124ea7ab0a421e9df5f79a6e0ea809780b6f6,
4b9ee84e1f517c4adeeb4c505088d991093c8f46 and
167d0fab778207c46bbbba113cbc0ba0b21649f3, without conflicts. Shared
operator_command_plane.py edits affect distinct functions. No further production
changes were made. Integration qualification additionally corrects a test's
full-rack assumption to compare every well against retained pre-operation
occupancy, and asserts loaded=true / pseudo home=500 through the actual receipt
handler and cold canonical store.

## Combined execution

Production image bioxp-oem:2048334c8209, source and OEM bundle read-only,
network none, no devices, only private scratch writable. Retained source DBs
mounted read-only; all test migrations/writes target copies. Final JUnit:
**111 passed, 0 failed, 0 errors, 0 skipped**, 176.262 seconds. One Starlette
httpx deprecation warning. Not a full suite or hardware qualification.

Exact commands, JUnit and logs:
/home/dalab/.hermes/profiles/fresh/cache/scratch/workflow-repair-integrated/
(command-final.txt, combined-final.xml, combined-final.log).

Roster counts:
- test_tip_travel_height: 5
- test_partial_pickup_accounting: 35
- test_tip_length_z_offset: 6
- test_oem_load_tips: 7
- test_workflow_publication_pacing: 1
- test_door_handoff: 10
- test_debloat_child_waiter: 4
- test_protocol_oem_provider_bindings: 19
- test_protocol_workflow_connected: 3
- test_workflow_park_rehome: 15
- test_pipette_tip_type_selection: 6

The earlier tip lane's six optional retained-fixture skips are exercised here:
active-receipt pickup with provider reopen true/false, normal completion, failed
home and release error. No skips remain in this roster.

Initial attempts remain preserved, not counted as successful qualification:
combined.log: 96 passed, 9 setup errors (directory env was supplied a DB file).
combined-corrected.log: 83 passed, 6 failures (test assumed retained rack full),
16 errors (default tmpfs exhausted). Corrected directory binding, task-private
disk scratch and exact occupancy assertion yielded the full clean run.

## Proven and remaining acceptance

- Actual manual pickup handler/finite provider/receipt/store path, partial tips,
  failed tail operations, cold readback and loaded->500 pass. Named travel emits
  Z=500 before later XY; rack pickup ignores loaded-tip length offset.
- V15->V16 quiescence refusal, backed-up migration, exact schema verification,
  reopen, synchronous publication/plane versions and terminal snapshot pass.
- Already-closed door native no-op advances; stale initial child read advances;
  injected lease expiry produces ambiguous parent with child evidence and actual
  exception; setup failure retains bundle. Replacement owner/epoch and known
  outcomes remain fenced.
- The two remaining acceptance gaps recorded below were closed by the follow-up
  qualification in **Acceptance closure**. The initial 111-case run did not
  establish complete eject publication or explicit pause/abort/crash recovery.
- Historical dead-worker cause remains unknown; injected lease expiry is one
  reproducible mechanism. Persistent SQLite failure/owner replacement recovery
  is not overridden by this repair.
- Original pacing benchmark (104.477 to 48.006 ms median on workstation) is prior
  lane evidence, not rerun here, and does not explain observed 10–20 s live gaps.

## BMS compatibility (read-only)

Inspected commit 458336511638392b7924bd73416ba6bea0eb6edd and
/home/dalab/.cache/bioxp-live-feedback/REPORT.md. Status vocabulary includes native
uncertain/control states; compact observation/list and one immutable full-context
read use existing native contracts preserved by these repairs. No source-level
incompatibility identified. BMS tests/browser not rerun; API/fixture counts in its
report are separate evidence, not part of the 111. BMS worktree not edited.

No deployment, push, service restart, robot access or live DB write performed.

## Acceptance closure

Implementation/test commit: **5dc55c19c343b24b22d6ab9df1aefd2185a612f9**,
following 629b8666dda1c856aae15da9e44b8b767fece373. This section is a
documentation-only follow-up; the tested implementation and tests are unchanged.

### Eject publication

The real workflow TIP_EJECT handler and direct eject service had no canonical
publication after transport verification; diagnostic eject updated channel flags
only. The caller now observes the existing all-channel post-eject query and
publishes through the existing provider tip setter while the real receipt is
reserved, carrying that receipt's command ID. Diagnostic completion publishes
through the same provider owner and preserves completed channel releases before
a later channel failure. No extra query, eject, retry, motion, or admission gate
was introduced; the transport's existing OEM retry behavior is unchanged.

Nine cases exercise actual workflow admission/dispatcher/executor ->
api._protocol_live_pipette_handler -> run_pipette_tip_command ->
FourPipetteTransport.set_tip/eject_all_tips, direct eject-all, and
run_diagnostic_inline/run_collection_diagnostic with real receipt stores and
inert pump drivers. For each caller: selected partial ejection leaves tips and
pseudo-home 500; full ejection publishes unloaded and 65000; injected canonical
publication failure is retained as observation without failing/repeating the
completed ejection. Cold canonical/provider reads are asserted. The workflow
retains its real pipette child identity and reaches completed. Workflow/direct
key replay after a later pickup neither sends again nor republishes old tip
state. Direct/workflow pipette receipts retain the exact publication and
all-channel observation across cold SQLite reads. Existing plate distinctions
and pickup tests remain in the combined roster.

The same complete-eject cases against the original three production files at
629b8666dda1c856aae15da9e44b8b767fece373 fail **3/3** on missing
`deck_state_publication`; the candidate passes. This intentional regression
check is retained separately in eject-baseline-red.xml/.log and
command-baseline-red.txt, not counted as passing qualification.

### V16 controls and crash recovery

Four new cases use real migrated/private V16 stores and native workflow owners:

- Actual HTTP pause during an entered native leaf, reached ordinary_pause,
  independent SQLite readback, then explicit Continue; only the second leaf
  executes, and fresh-process readback matches the completed job.
- The same explicit pause followed by HTTP Abort; the second leaf never runs,
  the job becomes interrupted, and control-key replay/fresh-process readback
  preserve the exact result. Receipt-only pause publications add no full report
  snapshots; terminal status changes do.
- SIGKILL a separate process during an entered action, then allow its real lease
  to expire and reopen through OperatorCommandStore's actual startup recovery.
- SIGKILL a separate process at a reached explicit pause and reopen identically.

Both crash cases preserve action evidence and document, durably publish
ambiguous/reconciling/workflow_owner_lost, refuse new Continue and Abort without
an eligible owner, and retain the old pause acceptance when applicable. No
lease/status row is fabricated to trigger recovery. A native-entry file proves
only the first leaf entered; claim_next and a second cold reopen never replay
it or enter the second leaf. Leaves are inert test callbacks, not physical
hardware qualification. No V16 production-code change was needed.

### Final combined evidence

BigBoy only; image `bioxp-oem:2048334c8209`, image ID
`sha256:0daee7b395b2a3eeb4026a7bee259bf12aea029e194f00805efd65e94090be0c`.
Read-only committed source/OEM/retained fixture mounts, no network or devices,
private writable scratch, and unchanged `tests.z_stop_offline_guard` imported
before pytest. The unchanged prior roster plus the nine eject and four control
cases yielded **124 passed, 0 failed, 0 errors, 0 skipped**, **227.805 s** by
parsed JUnit (124 named testcases). One Starlette/httpx deprecation warning.

Evidence directory remains:
`/home/dalab/.hermes/profiles/fresh/cache/scratch/workflow-repair-integrated/`

- `command-acceptance.txt`: exact production-image command and complete roster.
- `acceptance-combined.xml`, `acceptance-combined.log`: final combined results.
- `new-focused.xml/.log`: preceding guarded new-case run, 13 passed.
- `eject-first`/`v16-first`: initial fixture failures (missing stable receipt key,
  missing action stage ID); corrected without changing product admission.
- `eject-fifth`: an invalid attempt to replay a workflow leaf outside its original
  workflow context correctly conflicted. The final test re-admits the original
  parent instead; no idempotency contract was relaxed.

Remaining limits: not a full suite, BMS/browser run, deployment or physical
acceptance. The historical dead-worker cause remains unknown; actual killed
process recovery is now qualified, not claimed as that historical diagnosis.
No robot connection, live DB mutation, restart, deployment, or push occurred.
