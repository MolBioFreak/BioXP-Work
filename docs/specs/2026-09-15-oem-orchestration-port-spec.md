# OEM orchestration port — draft for Christian's review

**Status: DRAFT. Port implementation is not authorized until this specification is reviewed.**

## Review in one minute

- **Port the actual OEM queue and workflow engine**, including its command/lifecycle semantics—not just a similar queue of our own.
- **One canonical runtime owner and store:** integrate the port with existing command/receipt and native device owners; remove competing paths instead of adding another scheduler.
- **Preserve source operations:** manual/script Park remain different; thermal nonwaiting modes, explicit error holds, pause, Abort and native completion keep their own meanings.
- **Correct host-mechanics defects explicitly:** owned/observed children, atomic resources, truthful results, no silent unknown operations or uncertain-motion replay. These proposed deviations are listed in §6 for review.
- **Apply the approved Stop boundary:** affected older named intents cannot become new motion by arriving late, including from other clients; native Stop remains independent.
- **Delete redundant reads separately:** the completed candidate removed admission/worker duplication and passed 551 offline tests. Further planning/Park consolidation is being qualified, not claimed complete.

**Review focus:** §3 lifecycle/control ownership, §6 intentional corrections, and §8 the keep/port/remove ledger. No deployment or physical motion is included in this approval.

## 1. Decision and scope

Port the recovered OEM **application queue, workflow representation/interpreter, operation dependencies and lifecycle semantics into the Python robot runtime**. This supersedes the earlier pattern-only recommendation. Do not host the Windows GUI or call an entire OEM job merely to execute one manual move.

The target is a source-faithful semantic port with explicit, reviewed corrections to unsafe task/error handling—not a literal translation of WPF events, `async void`, logging and accidental races.

Two separately tracked activities:
- **Already authorized:** delete proven duplicate hardware acquisition and qualify the existing repairs offline.
- **Review required:** implement the queue/workflow port and its connected API/UI migration. The selected full pre-Stop intent fence is an approved behavior requirement, incorporated here rather than implemented as a separate competing architecture.

No deployment, new physical movement, scientific-default changes or PyLabRobot integration is authorized by this draft.

## 2. What is actually being ported

| OEM owner | Target responsibility | Preserve / deliberately exclude |
|---|---|---|
| `BioXPMainWindow.motion_thread_process`, `BlockingCollection<motionCommands>` | Ongoing application-message admission and dispatch | Preserve named messages and source call chains; replace name + mutable `object[]` with typed immutable inputs. |
| Application handlers | `initializeSystem`, `unlockProcess`, `PrepareToRunJob`, `abortjob`, `validateJob`, `wakefrompause` | Map each to its original operation/lifecycle meaning. Do not silently substitute a similarly named existing endpoint. |
| `ClassBioXPScriptHandler` / `ClassScriptGenerator` / operation DTOs | OEM script ingestion, expansion and typed operation representation | Preserve supported syntax, source order, parameters, units and source-defined branch/expansion behavior; no new general-purpose workflow language. |
| `ControlLib.executeScript` / `scriptInterpretor` | Cursor/stage progression, operation dispatch, job completion | Port all recognized operation branches, not just manual `mov` and `park`. |
| `ComponentLocks` and nested tasks | Operation resource dependencies and source-permitted overlap | Preserve real exclusion/overlap. Replace non-atomic event reservation and unobserved child tasks with structured ownership. |
| Manual callbacks / existing mapped native operations | Manual moves, native operation execution and actual completion | Keep manual versus workflow variants distinct. Existing native/CAN/receiver owners remain the sole device path. |
| Pause/safe-stop/Abort/job cleanup owners | Explicit lifecycle transitions and cleanup policy | Keep distinct from addressed motor Stop. Cleanup that moves hardware is never inferred from an HTTP error. |

The companion operation matrix accounts for every explicit interpreter label and the default branch, including explicit `catchPlate → catch` and `releasePlate → release` inheritance, branch-specific behavior, cancellation and source-return distinctions. Coverage is a port-verification denominator, **not a runtime move-count gate**.

**Park mappings are pinned:** manual Park uses `rehome=false`; the script `park` operation uses `rehome=true`. Preserve their separate source paths and parameter contracts. Never route a manual Park through the script variant merely to reuse the interpreter.

### Port completeness

Each branch must have: source reference, typed input binding, original lower calls, resource dependencies, completion/error contract, lifecycle behavior, test evidence and a real runtime adapter. An unsupported adapter must reject the job **before execution**, identifying the missing operation. A stub returning success is prohibited. Partial milestone support must not be advertised as a complete OEM workflow engine.

## 3. Runtime architecture

```text
Manual named requests / OEM job inputs / application lifecycle messages
                         |
             strict validation + immutable identity
                         |
       existing canonical command store and admission transaction
                         |
            ported OEM application dispatch owner
                  /                     \
        manual native operation     structured OEM workflow task
                                         |
                              typed OEM interpreter + dependencies
                                         |
                         existing mapped native operation owners
                                         |
                           existing transport / receive / completion
```

**One canonical command/receipt store. No second motion scheduler or persistence service.** SQLite supplies the durability and request/outcome identity that the desktop OEM queue did not supply. Keep that responsibility in the existing store; do not defend every inherited wrapper/table simply because it is there.

Proposed code ownership:
- `operator_command_plane.py`: canonical admission, FIFO, worker/resource ownership, receipt identity and existing safety epochs.
- `oem_script.py` (new): OEM syntax/typed-input lowering and source mapping.
- `oem_workflow.py` (new): source-derived interpreter, cursor, child lifetimes, lifecycle and result aggregation.
- Existing movement/pipette/thermal/vision adapters: actual device operations. No alternate driver stack.
- Existing API/BMS owners: typed submission, read-only status/receipt projection and operator intent capture.

The application dispatcher and an active workflow are not collapsed into one blocking call that prevents Abort/Pause handling. A started workflow is a real owned asynchronous operation; its children remain joined to it. Ordinary synchronous native moves must not be relabelled as asynchronous tickets.

### Ordering and concurrency
- Deliberate named moves retain FIFO order under continuing entry, with no fixed batch size.
- Browser admission order and canonical commit order must be connected explicitly; browser state never authorizes motion.
- Normal manual moves captured during a running job remain pending rather than interleaving into its scientific sequence. This is a proposed reviewable ownership rule, not a claim every diagnostic OEM UI enforced it.
- Inside a workflow, preserve source-defined overlap, including independent thermal work. Do not serialize everything behind a global mutex, or create concurrency merely because an axis appears idle.
- Source resource acquisition becomes atomic and consistently ordered. Release reservations in `finally` after child settlement. Resource release is **not permission to advance**: source-defined error/decision holds remain explicit workflow states, rather than leaked locks.
- Control signals—addressed Stop, workflow Abort and pause/resume handling—must remain serviceable while normal work is occupied. They are not ordinary moves waiting behind the FIFO.

### Active-workflow and control contract (proposed for review)

Every control targets the current canonical workflow identity and its execution generation. A stale control cannot affect a successor job. Control delivery uses the existing control/interrupt owner, not another normal-work queue; acknowledgements and terminal outcomes stay in the same canonical store.

| Message/path | Eligibility and targeting | Resource/lifetime and completion boundary |
|---|---|---|
| `initializeSystem`, `unlockProcess` | Normal application FIFO; no competing active/preparing workflow | Run the exact mapped handler and owned children. Complete on that operation's source boundary, not on enqueue. No bypass into an occupied machine. |
| `validateJob` | Pure validation may run without hardware; its result binds the particular immutable input | No motion/resource permission is granted by validation. Source validation that needs hardware remains a separately owned execution step. |
| `PrepareToRunJob` | Normal FIFO, validated input and exclusive preparation eligibility | Own source preparation/prologue calls and their children. Prepared/startable is not whole-job completion. |
| Direct OEM start path / script-test start using the core interpreter | Explicit start against the prepared/validated identity; reject competing start | Register one active owned workflow before starting children. The application dispatcher remains responsive; the job receipt remains active until its defined terminal boundary. |
| Ordinary/deferred pause | Control signal to that active workflow; never trapped behind pending manual moves | Preserve their different source boundaries. A pause request is not proof that native children or all resources are quiescent. Publish reached-pause separately from requested-pause. |
| `wakefrompause` | Explicit operator resume of the same paused workflow, not an automatic reconnect action | Preserve OEM initial-check/rehome **before Continue** where that path requires it. Enter only after conflicting owned motion has settled, under the same job's resource ownership. Retain genuinely independent thermal children. No silent substitution with a bare Continue signal. |
| `abortjob` / software Abort | Control request to that workflow; close permission to dispatch its future physical nodes immediately | Signal source Abort and wake cancellable host wait/delay gates; retain and settle native children through supported native behavior. Cleanup enters only after conflicting children settle and under the same ownership. Do not pretend signalling, a released event or elapsed host timeout proves cancellation. |
| Source safe-state stop | Separate mapped lifecycle request, not an alias for Abort/addressed Stop | Preserve its source continuation/cleanup behavior and identify any hardware actions explicitly. Do not relabel it as an immediate no-motion cancellation. |
| Addressed motor Stop | Existing immediate interrupt lane, independent of workflow/FIFO occupancy | Deliver native Stop without waiting for admission/storage. Apply the approved named-intent fence; preserve exact native/canonical evidence and existing scope. It does not automatically initiate rehome, eject, safe-state cleanup or resume. |

For Abort, pure host waits/delays become cancellation-aware so a blocked host gate cannot strand the control request; cancellation does not claim the wait's physical objective completed. An already-entered native call is not killed or declared settled by that host signal. A native child whose outcome cannot be established keeps the workflow interrupted/uncertain and forbids further physical nodes until the existing recovery owner resolves it. This is an explicit D1/D5 host-lifetime correction, not a new native timeout or retry policy.

The `dopen` barcode-error/retry branch deliberately holds progression in the OEM source. The port releases settled resources but records **held-for-source-error/decision** and does not advance its cursor. Only the mapped source recovery decision can resolve that hold; no generic retry, skip or implicit next operation is added.

## 4. Identity, capture and admission

Capture immutable operation arguments, request identity, applicable owner/board generations and the named-intent safety token at the actual operator submission boundary. Runtime machine state remains with its current owner; do not deep-copy live services or freeze evolving device state into a job input.

- Existing same-key/same-input recovery returns the canonical current receipt before new-admission checks.
- Same key with different input remains an error.
- A lost response is reconciled by the original key. No blind POST replay and no new-key retry of uncertain physical work.
- Normal request IDs must remain distinct across documents/reloads. Addressed Stop must not acquire a new browser-entropy prerequisite. The exceptional legacy Stop-key collision must be closed in the request-identity design without introducing a separate ID service or blocking native Stop delivery.
- Existing finite capacity/argument bounds may remain explicit; they must not become an arbitrary small move-count policy.

## 5. Stop boundary — approved option 1

**All affected pre-Stop named intents must be fenced, including requests still in transit, waiting for admission, or originating from another client. They must not become new work merely because they arrive after Stop.**

Use the existing canonical global and X/Y/Z safety epochs, not a fake board/connection-generation bump or another store:
1. Expose a coherent read-only safety token with named-intent metadata.
2. Capture that token with the immutable intent. Never refresh it on an already captured request.
3. Compare it inside the existing admission transaction before allocating new work, after original-key recovery.
4. Admission before the interrupt cutoff is handled by the existing queued/active interruption logic. Admission after an epoch change rejects the stale intent.
5. At local Stop invocation, retire only genuinely unsent affected entries synchronously. Sent/uncertain/accepted identities remain available for truthful reconciliation. Failed or late callbacks must not restart retired entries.
6. A fresh deliberate action uses fresh authority. No automatic re-entry, retokening or replay of prior work.

Preserve existing addressed X/Y/Z resource scope and the existing aggregate host barrier for G/Abort. Named deck work already claims whole-deck resources; do not invent destination-specific axis independence. Native Stop delivery remains **ahead of SQLite reconciliation**, with the existing immediate RAM fence and storage-failure recovery. A remote browser click cannot take effect before it reaches the robot, and no promise is made that a native operation already entered is retroactively undone.

This approval concerns captured named-deck intent. Whole-job safe-stop/Abort and resume follow the separately specified OEM lifecycle; do not silently expand it into a new workflow replay/cancellation policy.

## 6. Completion and failure: explicit port corrections

These are intentional differences from problematic OEM host mechanics and require review with the port:

| ID | Port rule | Why |
|---|---|---|
| D1 | Retain and observe all operation children; async entrypoints return owned awaitable results. | `async void`/unretained `Task.Run` must not turn dispatch into claimed completion. |
| D2 | Atomic resource reservation and guaranteed release after child settlement; progression holds are explicit and independent. | Wait-then-reset races and accidental skipped releases are not behavior to reproduce. Deliberate error/decision holds must not disappear with lock cleanup. |
| D3 | Reject unknown/unbound operations before job execution. | Do not preserve silent unknown-command no-ops as successful work. |
| D4 | Normal job success requires every owned child to reach its **operation-specific source completion boundary**. | Cursor advancement/unlocked events alone are insufficient. Preserve nonwaiting variants such as `splid(..., wait=false)`: completion of its setter is not a newly imposed wait for temperature attainment. |
| D5 | Preserve partial native results, exact causes and uncertainty; consume correlated source error/hold/state outcomes even when a helper returns normally. | No swallowed fault, manufactured success or automatic physical retry. Preserve native return semantics separately from host classification; arbitrary logged diagnostics or unrelated sticky flags are not automatically operation failures. |
| D6 | Keep source-defined ordinary/deferred pause, safe-state stop, software Abort and addressed Stop distinct. | None is a universal synonym for another; cleanup may itself move hardware. |

Preserve operation-specific OEM geometry, calibration, clamping, units, biological settings, retries and native completion rules. Any newly discovered scientific/default/retry change must be called out for approval, not smuggled into an infrastructure port. Known ambiguous/decompiler-sensitive branches require retained-binary/source validation before implementation claims.

A restart must not replay uncertain physical operations. Persisted cursor/job state does not itself authorize resume. Use the existing recovery owner and explicit operator decisions; keep failed history immutable.

## 7. Redundant checks: delete at their owner

The following deletions are separate from approval of the port and already present in the isolated repair candidate:
- Physical authority sampling at named POST admission.
- The worker's duplicate physical assessment before the native executor.
- Repeated XYZ/latch acquisition for compatible ordinary/Park readiness scopes; retain independently evaluated scope outcomes.
- Dead mixed-snapshot detection and repeated freshness reductions over one already-coherent projection.

Also preserve subset observations and query-refresh yielding, so background collection does not create another submission/readiness stall.

**Additional source-proven consolidation, under isolated qualification:** remove the executor's physical planning acquisition and Park's blanket post-Force reacquisition. Build the finite stage skeleton from immutable inputs; preserve and verify the host-only Force/canonical transition; acquire final physical authority once afterward to make both latch decisions and bind execution evidence. Current successful established paths use two wrapper acquisitions for ordinary/barcode and three for Park. The proposed consolidation is one final wrapper acquisition, not removal of native queries. It requires correctly moving the consumers—not simply deleting calls or inventing a ready snapshot. This follow-up is not yet covered by the completed regression result.

**Do not make “two samples” a permanent rule merely because a test counted two.** For every remaining read, document the fact consumed, the operation/state change that makes an older observation insufficient, and the actual consumer. Delete reads with no distinct responsibility. Consolidate planning/validation acquisition where source/state ordering permits; do not replace a needed post-change observation with stale cache, extend TTLs, relax leases or remove a genuine Stop/owner/board/latch/reference check.

Keep source-required native position/completion reads distinct from host preflight repetition. Read-only UI/status polling must not initiate hidden hardware acquisition or repeatedly rebuild full diagnostics.

## 8. Deletion and migration plan

Before implementation, enumerate all direct and indirect consumers: manual named ingress, the six application messages, raw/typed scripts, generic method ingress, internal `mov`/WP8 callers, BMS models/relay/hooks, receipt readers, tools and tests.

### Current-owner disposition ledger — part of this approval

| Current owner/consumer | Disposition in the port | Replacement/retained responsibility |
|---|---|---|
| `OperatorCommandPlane` / `OperatorCommandStore`; `admit_command`, `claim_next`, `_dispatch_one` | **Retain and adapt**, not a second parallel dispatcher | Canonical custody, ordering, request/receipt identity, current-resource and safety-epoch authority. Port the OEM application-message semantics here. |
| `operator_controls` named V2 ingress and BMS relay | **Migrate together** | Typed immutable manual intent, original-key recovery and captured safety token; retain fresh execution checks at their actual consumer, not at every ingress layer. |
| Current manual `make_deck_command_executor` and mapped provider operations | **Retain native bodies; simplify/port wrapper ownership** | Manual semantics and actual completion remain. The new workflow invokes the distinct script bindings; it does not feed manual Park through script Park. |
| Current resource claims, movement lease and addressed-interrupt lane | **Retain authority; map OEM dependencies onto it** | No second `ComponentLocks` authority. Atomic reservations/child settlement and explicit workflow progression holds replace accidental event behavior. Native Stop keeps its existing independent path. |
| Canonical receipts, source/board/reference stores and partial-result finalization | **Retain** | No parallel outcome database, RAM authority substitute or history rewrite. Extend the existing schema only for a demonstrated missing workflow fact, through its migration owner. |
| Browser named-admission custody and receipt hooks | **Migrate, not promote to a motion scheduler** | Track captured unsent versus sent/uncertain identities, preserve admission order, retire genuinely unsent entries on Stop, never retoken or replay old intent. Canonical store owns accepted work. |
| Legacy single-flight/latest-receipt assumptions and `successive_move_queue` fallback for named work | **Remove from the migrated named path** | Current canonical queue and independent receipt identities replace them. A legacy display fallback must not silently become another scheduling owner. |
| Generic grouped-method deck ingress | **Replace with typed OEM job ingress; no parallel OEM execution path** | The existing unsupported deck shortcut remains rejected until mapped. Unrelated non-deck methods are not silently redefined; they cannot serve as an alternate OEM workflow scheduler. |
| Six application messages plus direct start/pause/resume paths | **Port into the single lifecycle contract in §3** | Preserve each mapped source action, active-workflow targeting, source-specific control boundary and asynchronous lifetime. No independent application scheduler alongside the canonical dispatcher. |
| Core-interpreter raw/typed script inputs and OEM script-test UI calls using that interpreter | **Port their interpreter input contract** | One OEM compiler/runtime; exposing a production script-test UI is not required. All accepted inputs receive the same validation, ownership and safety boundaries. |
| W6 independent board-test runner | **Exclude from this production-workflow port** | Do not import its separate diagnostic scheduler or let it bypass production ownership. Any future exposure is a separately reviewed diagnostic capability. |
| W6 calibration/setup entrypoints | **Exclude new calibration execution/UI** | Preserve consumed calibration data and native parameter meaning; do not change calibration, firmware, scientific defaults or setup sequencing under this port. |
| Windows/WPF dispatcher/events, UI log machinery and unused SDK command-queue class | **Do not port as infrastructure** | Replace only their necessary typed callback/control semantics. Keep existing native transport/receive authority; no second wire-command queue. |

These dispositions are the review baseline, not a claim that an unrelated legacy route or table is safe to delete. Before editing, enumerate exact direct/aliased callers and test consumers for these named owners. Any newly found conflicting owner, unsupported state/schema need or changed disposition returns to review rather than becoming an implicit compatibility framework. The implementation PR then records the actual deleted/replaced callers and evidence against this ledger.

Cut over the affected consumers together to one canonical owner. Preserve native mappings and required operational data. Use the existing migration authority where a schema change is genuinely necessary; no parallel database, blanket history rewrite or emergency downgrade that loses uncertain receipts. Remove old paths after verified cutover; do not leave permanent parallel implementations.

## 9. Acceptance required before release

- **Source port:** every application message and interpreter branch traced through real adapters; alias, input form, cursor/progress, prologue/epilogue, child/resource and failure behavior covered. No supported workflow contains a success stub.
- **Queue:** ongoing entry during execution; immutable inputs/identities; canonical order; explicit capacity refusal; no duplicate execution or hidden browser motion owner.
- **Workflow:** representative real parsed jobs exercise serial and permitted overlapping resources, actual child completion/error propagation, ordinary/deferred pause, Abort/safe-stop and terminal aggregation. Native-recorder comparisons preserve operation order/parameters, not invented fixture outputs.
- **Stop:** force both sides of the admission/cutoff race, other-client late requests, BMS waiters, response loss, late callbacks, failed reconciliation and fresh deliberate re-entry. Verify native Stop is never delayed by queue/DB admission or identity prerequisites.
- **Safety/uncertainty:** owner/board/reference/latch drift, partial native delivery, unknown completion, child failure and restart cannot yield a false success or replay. Include fresh-process canonical receipt reads.
- **Latency:** count actual owner acquisitions across POST → claim → executor → first transport write; attribute every retained read. Later live authorization must measure command/readiness latency on the real machine. Audit markers are not motor onset.
- **Integration:** frozen source, actual current producer payloads through strict API and mounted UI, all direct caller regressions, pinned runtime/package qualification, then deployment identity checks and separately authorized physical acceptance.

Test counts establish coverage, not physical permission or completion rules.

## 10. Review order and deliverables

1. Review this architecture, scope and D1–D6 deviations. **No port implementation before that review.**
2. After approval: land the typed OEM compiler/interpreter and queue/resource ownership together in isolated code, with a source/consumer migration ledger and executable tests—not a stub scaffolding deliverable.
3. Complete native adapter coverage and lifecycle/Stop/uncertainty qualification, eliminate replaced paths, and verify the pinned runtime.
4. Seek separate deployment/motion authorization and measure the actual latency improvement.

PyLabRobot can later submit typed protocols/resources through this owner. It must not become another motion scheduler or direct driver bypass. Its integration is not a prerequisite for this port.

## Source basis and evidence limits

Reviewed corpus: recovered OEM application/control/common/device/vision types, with per-file read-range/hash ledger. Key sources: `BioXPMainWindow.cs:2030–2101`; `ClassBioXPScriptHandler.cs`; `ClassScriptGenerator.cs`; `ControlLib.cs:5227–5688,5779–6704,10398–10670`; `ComponentLocks.cs`; complete `ClassControlInterface.cs` and `ClassPipetteCollection.cs`. Original retained GenBotApp IL independently confirms the application FIFO.

Companion: `2026-09-15-oem-orchestration-operation-matrix.csv`. Detailed source findings remain in the external `robot-audit/deck-command-audit/oem-orchestration/` evidence directory, not runtime modules or UI logs. Native firmware and generated mixed-mode vision internals remain explicit qualification boundaries. Source review is not proof the port is implemented or physically accepted.
