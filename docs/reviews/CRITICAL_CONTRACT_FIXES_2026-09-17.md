# BioXP 3200 critical contract fixes — review branch

Status: **candidate changes for manual review; not a deployment approval**.

Base: `MolBioFreak/BioXP-Work` commit
`aa644727b9540029a61adc0635eaa739d701f60c`.
The base tree was verified as `98752787623e63ec0210946507e26aa2a03c7bdc`.
Prepared on September 17, 2026. No robot commands were executed during this work.

## Scope and evidence boundary

These patches address Python contract inconsistencies identified in the preceding
repository review. The native Windows image, complete IL/decompilation corpus,
live machine state, and physical robot were not accessible for this work.
The immutable OEM record remains the specification; Linux code and Linux tests
are implementations under correction, not replacement evidence of OEM behavior.

The changes do not modify motor command sequences, coordinates, calibration,
homing parameters, source-shaped path predicates, physical STOP delivery,
release configuration, or files under `.oem_lock`. No merge into main or
installation on the robot is part of this branch.

## Commit 1 — retain and enforce supplied V2 board epoch bindings

**Finding.** `invoke_action_v2` and the legacy route's V2 conversion discarded
`expected_board_epoch_by_board` when creating `InvokeRequest`. The direct
request's durable claim also did not include these expectations in its identity.

**Change.** Preserve supplied epochs in the internal request, retained-owner
binding, canonical request digest, initial durable reservation, subsequent
receipts, and dispatch context. Compare nonempty expectations with the existing
composite board-authority projection at admission and again through the existing
worker precheck. A changed board epoch cannot be silently replaced by the current
one merely because the process ownership generation has stayed constant.

Same-key, same-intent receipt retrieval remains read-only after board drift.
Same-key requests with changed epoch expectations conflict. This distinction
prevents a recovery read from turning into a new hardware command.

**Compatibility limit.** Empty maps and legacy requests with no epoch expectations
retain their previous unspecified-epoch behavior. This patch does **not** make a
complete epoch map mandatory for every action. That would be a separate client/API
migration with action-specific requirements. Unknown or unavailable *supplied*
board expectations fail closed. The independent interrupt request path does not
acquire this new admission requirement.

Files: `operator_controls.py`, `operator_receipt_store.py`,
`runtime_audit_store.py`, and `tests/test_review_direct_board_epochs.py`.

**Manual/OEM-host checks.** Confirm that the existing composite projection's
board 4 authority and board 5 current X lifecycle generation match the deployed
provider contract. With the same process ownership generation, change a board
lifecycle generation and verify that an old nonempty expectation is rejected
before a motor call. Confirm that receipt retrieval after the change does not
reissue the operation. Review the client treatment of empty maps explicitly.

## Commit 2 — withhold Z readiness after reference-publication failure

**Finding.** `execute_z_intent` caught `_z_mark_referenced` failures but left `ok`
true, allowing a completed receipt and a `referenced_ready` lifecycle despite a
failed durable reference publication.

**Change.** Treat reference-publication failure as failure of the overall
reference-establishing operation, while retaining its original successful
controller evidence and source return value. Publish `failed_latched` and
`reference_state=desynced`, not `referenced_ready`. Attempt the existing reference
invalidation. If that also fails, retain the secondary error in the failed
receipt and attempt to save the failed lifecycle through the existing state owner.

The patch does not repeat homing or add motor calls. Same-key replay returns the
failed receipt. A failure of the final lifecycle/receipt storage still propagates;
this is not a claim that a fully unavailable database can provide durable custody.
When invalidation fails, its receipt explicitly says so: the lifecycle's
conservative `desynced` state is not proof that the reference store was repaired.

Files: `oem_serial206_initialization.py` and
`tests/test_review_z_reference_publication.py`.

**Manual/OEM-host checks.** Review the distinction between controller homing success
and usable Python reference authority. Inject publication and compensation
failures in a copy of runtime storage, not the live robot database. Verify all
operator/readiness consumers respect the failed lifecycle even if the separate
reference store retains an older row. Validate recovery deliberately; do not
blindly retry a physical home because persistence failed.

## Commit 3 — correct shared action footprints and method epoch checks

**Finding.** The shared command plane omitted normal Y actions from
`AXIS_BY_ACTION`. Its `_axes_for_action` treated the multi-axis
`oem.z.scriptmove_to` operation as Z only. Shared method-control epoch checks also
omitted Y and the other axes of this composite.

**Change.** Add normal Y home/relative/absolute actions to the Y mapping, give
`scriptmove_to` the existing XYZ fence footprint, and include Y/composite axes in
method-control epoch validation. Retain the existing method API's scalar maximum
epoch representation; replacing that representation with a full epoch vector is
outside this patch.

Files: `operator_command_plane.py` and
`tests/test_review_command_axis_fences.py`.

**Reachability limit.** This is a shared/durable/legacy plane correction. The
primary V2 direct-action path is a distinct path. Passing these tests is not
proof of real STOP latency, electrical stop delivery, or complete cancellation
coverage across every route.

**Manual/OEM-host checks.** Inventory deployed routes that still use this plane.
Verify that source `scriptmove_to` may affect X/Y/Z and that each relevant stop
fence blocks subsequent dispatch. Check method pause/control clients' expected
epoch fields. Preserve existing source task joins and controller stop semantics.

## Validation performed

The portable suite completed with **129 passed**, comprising 55 new regression
cases plus 74 existing API/protocol regression cases. The 55 new cases run on the
unmodified base produced **35 failed, 20 passed**. The failures include dropped
board expectations, false Z reference readiness, and missing action footprints.
This is evidence that the new tests detect the intended Python defects; it is
not proof of OEM equivalence or physical motion safety.

Validation environment: Python 3.13.5, pytest 9.0.2, FastAPI 0.128.2,
Pydantic 2.13.4, pyusb 1.3.1, python-can 4.6.1, rfc8785 0.1.4.
The portable tests use offline hardware/readiness doubles and temporary runtime
storage. The epoch suite deliberately isolates unrelated readiness rules while
exercising the real retained owner, ASGI routing, request digest, and SQLite store.
The reference suite uses simulated controller evidence and real runtime storage.

Run the portable review suite from the repository root in an isolated checkout:

```bash
PYTHONPATH=src:. python -m pytest -q \
  tests/test_review_direct_board_epochs.py \
  tests/test_review_z_reference_publication.py \
  tests/test_review_command_axis_fences.py \
  tests/test_z_oem_api_contract.py \
  tests/test_protocol_workflow_api.py \
  tests/test_protocol_executor_finalization.py \
  tests/test_protocol_source_children.py \
  tests/test_protocol_source_lifetime.py
```

Install the repository requirements plus pytest and httpx into a development
virtual environment first. Never substitute fake data for the locked OEM bundle
to make native-parity tests pass. The existing host-dependent integration tests
require the separately held immutable evidence and must be run by that reviewer.
The full repository suite was attempted separately; it is **not a release gate
passed by this review**, and missing host evidence was not disabled or rewritten.

## Deliberately not changed

* **Barcode/already-at-target completion.** The earlier report identified a
  candidate wrapper/receipt mismatch, not a proven native-equivalence defect.
  No acknowledgement is fabricated and no terminal-proof predicate is relaxed.
  Reproduce through the installed `make_deck_command_executor` and the real
  `moveZCamera` producer before changing acceptance rules.
* **Named deck steps in general methods.** Existing rejection is preserved. Do
  not remove it without equivalent robot-owned ordering, evidence, interruption,
  and partially completed workflow recovery.
* **OEM-looking oddities and documentation authority.** No path predicate or
  native behavior was normalized because it looked unusual. The broader README
  and traceability reconciliation remains a separate task against the evidence
  lock's authority hierarchy.

## Acceptance and rollback

Review each implementation commit independently. Confirm the diff against the
pinned base, run the portable tests, then cross-check the relevant native sources,
configuration, retained receipts, and installed route bindings. Record the
reviewed binary/config hashes and any disagreement with these assumptions.
Only after that review should supervised robot checks be considered under the
existing commissioning/runbook procedures. Do not deploy this branch automatically.

The implementation changes are separate commits and add no database columns or
schema migrations. Revert them independently as needed. Be aware that the first
patch intentionally binds nonempty epoch expectations into new command identities;
receipts created by patched code should not be reinterpreted by older code or
reissued after a downgrade. Existing runtime reconciliation rules still apply.
