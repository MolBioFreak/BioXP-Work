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
- **Not fully qualified:** actual complete eject action through its production
  caller into unloaded->65000 publication. Existing test calls clearTipLoaded
  directly, and partial-release coverage proves retained-tip behavior, not that
  complete eject invokes the publication. Do not treat setter tests as closure.
- **Not fully qualified:** explicit pause/abort/process-crash readback after V16
  through real control/recovery owners. Review holds, park pauses, cold receipt
  reads and injected worker exceptions are narrower coverage, not substitutes.
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
