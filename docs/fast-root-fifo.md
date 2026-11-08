# Fast-root FIFO: native backend/API adaptation

## Scope and custody

Starting revision: `90e1314eb7337b46ba00278243c5626a138d2ac8`, including the sampler and bounded compact-XY-receipt fixes. Task branch `fix/fast-root-fifo`, isolated worktree `/home/dalab/robot/wt-fast-root-fifo` on BigBoy. Installed baseline remains `37f487c185e77c1bead60d8856f219babf552647`. Neither the source checkout nor the sibling candidate is edited.

Canonical V2 `oem.xy.move_sequence` now uses `OperatorCommandStore.admit_command`, its existing SQLite identity/stream/resource owners and existing finite command worker. Each root dispatches the existing `/motion/oem/move_xy_sequence` once; that route runs the existing provider `execute_xy_sequence`. It does not compile a replacement WP8 plan, enqueue each target, create another scheduler, coalesce targets, retry, introduce a new limit or redefine completion. Retarget stays on its separate direct owner. Browser click custody is not repaired here.

## Original .NET evidence, read before editing

Original retained source root:
`/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/`.

- `ControlLib.cs:6398-6412`: `mov` acquires `ComponentLocks.LockGeneral`, observes cancellation and runs `movExecution` on `Task.Run`, releasing the same general lock after its source call. This is the source serialization/worker mechanism, not an HTTP queue.
- `ClassControlInterface.cs:4285-4311`: actual `moveXY` missing-axis and near-axis branches.
- Same file `4313-4348`: source distance-dependent acceleration, source-dependent X/Y issue ordering/stagger and 5ms sleep.
- Same file `4349-4367`: 5000ms source wait; STA individual waits versus worker `WaitAll`; timeout logs and restores acceleration, rather than adding a failure, physical-arrival proof gate or retry.
- `WindowBoardTest.cs:1010-1034`: one Script thread executes input lines in order and observes Stop between them. Its `1073-1082` `mxy` caller is distinct from `moveXY`; Retarget remains that separate source contract.

The OEM did not have a short HTTP acknowledgment or durable HTTP idempotency key. These are necessary API adaptations for deliberate independent requests. The substantive source behavior remains in the same driver/adapter/provider. AST comparisons against the exact starting revision confirm unchanged `execute_xy_intent`, `oem_move_xy`, `_raise_to_travel_height`, `await_pending_xy_retarget` and `execute_xy_retarget` bodies. No native driver code changes.

## Admission, source work and receipts

1. V2 routes named/well/Fast intents to the existing durable command owner. Unique roots get committed IDs/stream sequence before source work finishes. Same-key requests use the original immutable fingerprint. Changed inputs, generation or requested board observations cannot change an existing key.
2. Prior direct Fast receipts retain their old owner: the V2 route recovers the original receipt with the existing direct replay validators instead of attempting a second SQL claim. A concurrent durable commit between identity reads is rechecked through the durable owner. No automatic replay after a failed or uncertain outcome.
3. A real-owner reproduction revealed a second admission wait hidden by the static movement fixture: full provider-status collection and public named admission's provider-before-writer scope waited on a Fast root's retained motion mutex. Public intent admission now reads a small generation/capability projection from the existing generation owners, without taking the movement mutex, and commits observed bindings. It does not use cached passive/display status as an admission token. Internal finite-plan admission keeps its original provider-before-writer boundary. Actual dispatch, coherent planning, provider locks and pre-TX source checks remain in the existing executor. Generation changes after admission can still refuse dispatch; committed acceptance is not physical readiness or completion.
4. Fast retains its provider mutex and source worker context for the whole ordered body. Its existing one-height preparation is unchanged. Because a named/well Fast target can raise Z, the durable root uses existing XYZ resources/interrupt scope, not XY alone. Normal named/well roots share that same durable stream and exclude conflicting work.
5. The existing durable root Stop/cutoff fence is connected to the provider's existing per-handoff interrupt callback. This is essential for addressed Y Stop, whose source-void body need not mutate the sequence provider's local software-interrupt epoch. Clearing Stop cannot revive the terminal root or cleared queued roots. Physical Stop still emits addressed controller Stop writes; aggregate software abort does not acquire a fabricated physical-Stop meaning.
6. Operational `xy_sequence` results remain outside diagnostic truncation in the existing receipt owner. Full children remain on detail, bounded counts/failure/index summaries remain on compact/history/dashboard. If Stop wins terminal CAS before the source worker returns, the same-attempt source ledger is appended through the existing evidence/transition owners, without rewriting the canonical interrupt outcome or claiming arrival. The old source suffix remains ended.
7. Failed/ambiguous history remains record-only. Resource exclusion considers genuine live holders, not old receipt uncertainty. Root source-call completion remains distinct from physical proof.

## Executed offline qualification

Image: existing `bioxp-oem:37f487c185e7`. Disposable Docker containers use `--network none`, no devices, source and OEM-evidence mounts read-only, task-private SQLite, existing task-only pytest/httpx dependencies. No robot API calls or hardware calls. Production V2 routes, SQLite owners, dispatcher, provider, production adapter and native driver execute; electrical replies are inert. Pass-through instrumentation pauses/observes native waits and identity-read scheduling without replacing movement/provider outcomes. Source-logical predecessors in the reused named fixture are explicitly offline fixture state, not hardware evidence.

Final results (both XML rosters parsed, no failures/errors/skips):

- Existing required roster: **49 passed, 1 warning in 183.51s** (JUnit time 183.488).
  `test_xy_compact_receipts.py`, `test_retarget_sampler_stability.py`, `test_movement_modes.py`, `test_z_target_receipt.py`, `test_latency_receipt_fallback.py`.
- Focused changed-owner roster: **33 passed, 1 warning in 154.85s** (JUnit time 154.855).
  `test_fast_root_fifo.py` (13 cases), `test_xy_move_sequence.py` (15 cases), and existing `test_debloat_core.py` warm ordinary (2), consecutive/fresh-process (1), real Stop/owner dispatch fences (2).
- Total: **82 passing cases**, no skips. Warning is existing Starlette/httpx deprecation.

Cases include three separately committed Fast roots while the first native wait is held; real bound-provider admission; Fast/named/well/Fast FIFO; one source call per committed root; ordered distinct child receipts and native MTA WaitAll; existing one-height preparation; no missing-notification proof gate; success/failure replay; changed-payload rejection; old direct-key recovery; concurrent identity-read/commit race; X/Y/Z addressed Stop and aggregate software abort; cleared old queue, ended active suffix and retained late source ledger; failed and explicitly labelled ambiguous history; cold receipt readback and all compact/detail/history/dashboard surfaces.

Actual final inert-wire admission samples with the first root still held:

- Three Fast roots: 0.027074033s, 0.010441565s, 0.008961911s; all committed unique IDs and dispatched in exactly that order; all three source roots completed.
- Fast/named/well/Fast: 0.027121573s, 0.010496295s, 0.010195706s, 0.009030291s; all four committed before release, dispatched FIFO and completed.

These are workstation/inert-wire API times, not robot latency or physical-arrival measurements.

## Reproduction and red evidence

External evidence directory: `/root/workstation-migration/bioxp-stability-backend/`.

`sh fast-fifo-test.sh <nodeids/files>` records the exact offline Docker invocation. `python3 fast-fifo-final-check.py 49` and `... focused` run the final rosters, save logs and parse JUnit. Final logs: `fast-fifo-final-49.log`, `fast-fifo-final-focused.log`; XML under `scratch/fast-fifo-final-{49,focused}.xml`; actual producer handles/receipts/wire/timings under `scratch/fast-fifo-wire/{fast,mixed}.json`.

Retained red evidence:
- `fast-fifo-red.log`: second independently submitted root receives the original busy409.
- `fast-fifo-mixed-first.log`: real provider binding exposes admission waiting on the retained root; after its fixture wait expires, an unintended later dispatch invalidates the held-root assertion. Corrected at the existing admission boundary, not by weakening the source lock.
- `fast-fifo-interrupt-green.log`'s earlier failure is separately retained in `fast-fifo-interrupt-first.log`/`fast-fifo-x-stop-first.log`; the initial artificial X wait needed to wake on actual double-Stop delivery, matching native wait behavior. Y's source-void Stop also exposed the missing durable-root handoff fence; corrected without changing Y Stop.
- `fast-fifo-replay-red.log`: prior direct key caused the canonical SQL unique-key rejection; recovery now uses the existing receipt owner.
- `fast-fifo-stop-ledger-red.log`: Stop won root terminal CAS and discarded later source ledger; additive same-attempt evidence now survives.
- Initial history fixture cleanup used nonexistent `OperatorReceiptStore.close`; corrected to its actual connection close. No product workaround or dependency installation into the live environment.

No deployment, push, live GET/POST, robot DB read, reconnect, restart, Home, motion, UI edit or physical verification was performed. Browser click loss, Retarget pending-target policy and Park action routing remain separate owners.
