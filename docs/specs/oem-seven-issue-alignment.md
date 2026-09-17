# BioXP Serial-206: seven-issue OEM alignment specification

Status: proposed specification for Christian's approval. Documentation only; neither approval of this text nor permission to implement, run tests, conduct reviews, deploy or actuate is implied by its creation.

## 1. Authority and hard scope boundary

This document formalizes ONLY the seven issues in the immediately preceding line-by-line explanation. It does not replace the overall BioXP specification, add a new architecture, or reopen unrelated repaired behavior. Christian's approved specification and explicit amendments govern; this document cannot authorize its own deviations.

The requested AGENTS changes were read from `/home/dalab/biomodstack/wt-bridge-closure-design/AGENTS.md`, including the change in commit `ef443c4a5fb25c126de071a459ff701b1b0f266d`. No AGENTS.md was found in the robot candidate or its checked ancestors. The applicable user-directed rules are: specification fidelity; use existing mechanisms; preserve semantics; no silent scope reduction; distinguish implementation from acceptance; primary agent only; no subagents, test runs or review passes until explicitly reauthorized. BMS-specific worker architecture and branch/deployment rules are not transplanted into BioXP.

Target implementation, if subsequently authorized: `/home/dalab/robot/rectify-oem-candidate`.

The seven fixed scope IDs are:

1. S1 — OEM machine-state construction/defaults.
2. S2 — OEM tray construction connected to existing durable state.
3. S3 — Exact no-tip startup behavior, relying on S1 rather than invented initialization operations.
4. S4 — Startup G +10000 through the OEM moveSteps primitive and enclosing current lifetime.
5. S5 — One source-ordered thermal-door predicate evaluation in initializeMotors.
6. S6 — Atomic mixed-signal WaitAll semantics in the no-router fallback.
7. S7 — Align the existing legacy startup-G wrapper with its actual source role; do not misidentify it as the initializer's current call path.

S3 is an integration requirement of S1, not a claim that OEM omitted an additional startup assignment. S7 is a wrapper-contract issue; the direct initializer currently bypasses that wrapper. These qualifications are part of the scope, not optional commentary.

### Explicit exclusions

No wider motion audit or repair campaign; no other Home/Stop/Abort redesign; no new board initialization policy; no general reference-service rewrite; no extra tip acquisition/ejection/reset/inspection workflows; no coordinate/pathing/camera-offset changes; no catalog or frontend redesign; no BMS consumer repair; no historical endpoint investigation; no camera, logging-performance, liquid-operation, CV, calibration, firmware or job-automation work. No installs, network/device/live-DB/API operations, OS changes, commits, deployment or physical acceptance under this specification-writing request.

The previously fixed recovery generations, stale-state publication, interrupted observation receipts, deck collection/cache expiry, target dispatch and existing safety guards are preservation constraints, not new work packages.

## 2. Source authority and interpretation

Available OEM evidence root:
`/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup`

Aliases:
- CI: `decompiled_src/BioXPControlLib/ClassControlInterface.cs`
- CL: `decompiled_src/BioXPControlLib/ControlLib.cs`
- MS: `decompiled_src_bioxpcommon/BioXPCommonLib/ClassMachineStatus.cs`
- TT: `decompiled_src_bioxpcommon/BioXPCommonLib/ClassTipTray.cs`
- P: candidate `src/bioxp/oem_serial206_initialization.py`
- D: candidate `src/bioxp/usb_driver.py`
- CP: candidate `src/bioxp/operator_command_plane.py`

Line anchors below identify the source read for this specification, not a guarantee that line numbers remain fixed after editing. Installed binary/locked IL is authoritative over a misleading decompiler projection. Existing source locks and exact methods must be used for any ambiguous enum value, well layout, board moveSteps branch or event behavior. No new SSD acquisition requirement has been demonstrated.

OEM constructor defaults are software semantics. Persisting them as such is not fabrication of measured physical location or tip presence. Conversely, logging a constructor or source return must never label it as a controller observation, reference measurement or physical-success proof.

Existing SQLite is the persistence mechanism. No competing store, bootstrap scheduler, external prerequisite service, fabricated generation, hard-coded command proof or independent state authority shall be added. An internal persistence check that prevents an OEM-valid fresh construction must be reconciled with the constructor path, not retained as a permanent excuse to disable that path. Existing safety conflicts require a precise user decision; this document grants no blanket safety bypass.

## 3. Required corrections

### S1 — Reproduce machine-state construction

Source: MS:31–35, 53, 63–65 and constructor MS:400–454.
Current mismatch: P:4781–4782 and 4789–4790 initialize tip-loaded, tip-dirty, location and well to None; P:4585–4586 and 4601–4602 then reject incomplete bootstrap. Tip-location -1 at P:4783 already matches its source initializer.

Requirements:
- S1.1 For a genuinely new machine-status object, reproduce OEM enum/default initialization for location and well, tip-loaded=false, tip-dirty=false and tip-location=-1. Resolve enum values through the existing source-bound enum mapping; do not equate a numeric default with Park or infer a nearest location.
- S1.2 Preserve all already-correct constructor fields and source semantics. This is not authority to replace the complete state model or rewrite unrelated defaults.
- S1.3 Connect construction to the existing durable machine/deck state through its existing publication/transaction interfaces. Its provenance must identify an actual host-construction operation, not a made-up controller query or historical motion command.
- S1.4 Construct once per actual new object/state lifecycle, not on every GET, reconnect, provider rebinding, ownership change or process restart. Preserve valid retained state, recovery holds and completed transitions. Unknown retained state must not be silently overwritten with new-object defaults.
- S1.5 Internal revision/provenance requirements must be satisfied through the real constructor publication. Do not fabricate board epochs, reference completion, latch observations or physical coordinates to pass unrelated checks.

Completion evidence: fresh construction yields the exact source software values in the real owner and durable reader; reopening/rebinding preserves later values; fresh construction alone produces no motion, controller query or measured-position/reference claim.

### S2 — Execute tray construction, not merely define publication methods

Source: MS:403–417 constructs trays 0–4, assigns types and locations; TT:183–196 constructs ordinary versus hotel wells; TT:119 defines TipAvailable as !m_trayempty.
Current gap: P:4340 and CP:3282 define publication methods; the inspected production source has no explicit call to publish_tip_tray_transition(...) beyond definitions and no matching explicit construct invocation.

Requirements:
- S2.1 At the corresponding new-machine construction boundary, construct all five trays using existing tray representation/persistence. Preserve OEM IDs, tray types, locations, ordinary/hotel well construction and source availability-latch semantics.
- S2.2 Determine initial well contents and m_trayempty from their actual source initializers/constructors. Do not infer availability from an image, any-one-occupied heuristic, or an invented physical inventory.
- S2.3 Wire the existing construction transition to the actual constructor path. Merely exposing a callable API, binding a callback, or manually seeding a test does not complete this requirement.
- S2.4 Do not reconstruct/reset trays during polling, restart or retry, or destroy retained depletion/inspection/reset state. Partial durable construction must either remain atomic under the existing transaction mechanism or resume idempotently without overwriting completed transitions.
- S2.5 Construction must not invent active ownership epochs. Reuse the existing identity/lifecycle mechanism; do not relabel stale physical observations as current constructor evidence.

Completion evidence: an actual new-machine construction initializes all five durable trays, including the hotel distinction; the deck consumer receives the resulting source latch; repeat binding/restart leaves modified trays unchanged. No new physical tray inspection is required to reproduce a software constructor.

### S3 — Keep no-tip initializeMotion literal

Source: CL:8843–8845 only assigns TipLoaded=false in the no-tip branch. The tip-present branch already includes location publication at CL:8813 and dirty/loaded clearing at CL:8829–8830.
Current code: P:11932–11936 implements the no-tip branch. P:11877–11878 and 11900–11903 already implement the cited tip-present assignments.

Requirements:
- S3.1 Keep the no-tip branch's source assignment; obtain initial location/well/dirty values from S1. Do not add a move, Home, tip ejection, location measurement or fabricated Park assignment to compensate for wrong construction.
- S3.2 Preserve source setter effects through the existing mechanisms, including existing pseudo-home handling where source-bound; do not add independent setter logic in a second store.
- S3.3 A no-tip startup on an existing machine state must not reset location, well or dirty state merely because a newly constructed object's defaults differ.
- S3.4 Preserve the existing tip-present updates and error branches. Only their integration with the same state owner may change if necessary for S1; they are not missing implementations to rebuild.

Completion evidence: source no-tip sequence is unchanged; a new object's constructor values survive it; retained nondefault state is not reset; the supported constructor-plus-startup path reaches the existing consumer without a manual fake state seed.

### S4 — Preserve the OEM G clear primitive and enclosing lifetime

Source: CI:3354 current31; CI:3355 board moveSteps(axis,10000,true); CI:3360/3364 version-selected axisSearchHome; CI:3417–3419 final current10 only for gripper version1.
Current code: P:11964–11965 sets31, P:11966–11969 substitutes motor_move_relative plus motor_wait_stopped(require_seen_nonzero=True), P:11971–11977 calls axisSearchHome, P:12058–12059 restores10.

Requirements:
- S4.1 Replace the substituted raw move/poll composition with the existing source-shaped board moveSteps implementation, preserving addressed board/axis, +10000 input, wait=true and the exact relevant OEM board branch, return and exception semantics.
- S4.2 Do not impose an extra seen-nonzero-speed criterion, readback-delta requirement, independent timeout/retry or duplicate Stop not present in that source primitive.
- S4.3 Preserve the enclosing current31 -> clear -> G home -> remaining initialization -> final current10 order for Serial-206/version1. Source exceptions must not execute a newly invented unconditional current restoration. Reuse the existing setting binding; do not add new machine-version support outside this task.
- S4.4 Preserve already-correct Z, X, Y, door, calibrated UI and chiller stages, except S5's exact predicate correction. No wholesale initializeMotors rewrite.
- S4.5 The source's board-presence checks and their order, including whether the clear call precedes a null check, must not be replaced with generic per-stage suppression. Physical/controller evidence remains distinct from nonthrowing source completion.

Completion evidence: exact addressed operation sequence and current lifetime, including normal return, board-specific no-op/error behavior, moveSteps failure/exception and G-home exception. No extra polling composition or dispatch is introduced.

### S5 — Evaluate the enclosing door condition once, in source order

Source: CI:3380–3383 conditionally calls doorSearchHome; CI:3384 evaluates `SerialNumber > 9 && !confirmAxis(tcDoorClosed) && CameraCalibrated`; CI:3386–3387 opens then throws; CI:3391 subsequently sets Y home.
Current duplication: P:11999–12009 evaluates a door-home return predicate and can open/throw; P:12011–12037 separately acquires status and evaluates the condition again.

Requirements:
- S5.1 The door-home stage shall perform its source operation, not a second copy of the enclosing condition. The enclosing condition has one owner after that operation.
- S5.2 Evaluate operands in the source's actual short-circuit order. For example, source SerialNumber<=9 does not query confirmAxis for this expression; CameraCalibrated being false does not move that operand ahead of confirmAxis.
- S5.3 Use the exact confirmAxis(tcDoorClosed) source operation rather than reusing unrelated cached status or adding a broad diagnostic status sweep. Queries intrinsic to doorSearchHome remain intrinsic; do not remove them merely to force a global one-query count.
- S5.4 If the expression is true, invoke openThermalDoor once and throw the source failure before Y setHome or subsequent stages. Do not convert malformed evidence into closed=true.
- S5.5 Preserve the existing stage/receipt ownership where possible. A stage may remain for truthful reporting; it must not cause duplicated hardware operations or altered evaluation order.

Completion evidence: exact calls/order for true/false condition, source short-circuit branches, door-home failure and open failure; no subsequent Y setHome after the source throw.

### S6 — Atomic no-router fallback WaitAll

Source: CI:4325–4329 uses the two motor handles; CI:4350–4359 uses sequential WaitAny in STA; CI:4361 uses WaitAll otherwise. These schedules must remain distinct.
Current fallback: D:3967–3978 treats all-software signals atomically, but D:4004–4005 immediately consumes individual wire completions into a local reached set. D:3952–3953 uses the router path when available; that is not the identified defect.

Requirements:
- S6.1 Represent retained fallback signals in the existing fallback signal/transport owner, covering wire, initial-state and software-Abort signals together. Do not add a second reader or independent completion service.
- S6.2 WaitAll success requires all requested handles signaled at one atomic consumption point. Consume all requested signals together only on success; timeout consumes none of those signals.
- S6.3 A partial wire signal must remain available after timeout for a subsequent legitimate wait, subject to the same source Reset/Set/ownership rules. A later attempt must not depend on receiving a duplicate wire event to reconstruct a signal that should not have been consumed.
- S6.4 Preserve AutoReset-style coalescing, explicit source resets, accepted frame/axis identity, abort semantics and generation invalidation. Do not claim physical completion when a handle was signaled by initialization or software Abort.
- S6.5 Do not change the router path, STA sequential consumption, source timeout, public caller return handling or existing No24V/Stop policy to conceal the fallback bug. Shared helpers may change only as directly necessary, preserving other behavior.
- S6.6 Do not choose STA versus non-STA from the Linux worker thread or invent an OEM caller context. Existing source-bound caller selection remains authoritative.

Completion evidence: all-wire, all-software and both mixed orders; timeout after a partial signal followed by a later successful wait; reset between waits; repeated/coalesced signals; wrong-axis/rejected events; concurrent wait consumption; Abort and generation change. No redispatch or extra controller queries.

### S7 — Align the legacy startup-G wrapper without confusing callers

Source role: CI:3354–3364 and CI:3417–3419 distinguish outer current/clear lifetime from the inner G axisSearchHome.
Current wrapper: D:7113–7127 adds preparation; D:7144–7149 reads retained current; D:7150–7164 can restore in finally. P:11972 directly calls axisSearchHome for initialization and bypasses this wrapper.

Requirements:
- S7.1 Identify actual in-repository callers of motor_oem_home_axis(g,startup=True) as prerequisite implementation reading, not a new broad review. Do not claim the initializer uses the wrapper when it does not.
- S7.2 A retained startup-G home-only entrypoint shall delegate to the source-shaped axisSearchHome and leave outer current31, +10000 and final current10 to their existing enclosing owner. It must not repeat preparation, perform a second clear, add proof-only current readback or restore current in finally contrary to that role.
- S7.3 Do not reroute the initializer through a combined manual-home routine. Preserve the separately repaired standalone/manual G path and its own source-specific current behavior.
- S7.4 If the wrapper is genuinely unused and has no supported compatibility role, remove only that proven redundant startup branch and its obsolete internal expectations instead of retaining an invented source contract. Do not remove public controls or retire a supported caller as a shortcut to alignment.
- S7.5 Existing arguments that become meaningless must be reconciled at their callers; they shall not silently select non-OEM preparation/restoration in a path labelled startup-OEM.

Completion evidence: each retained startup caller maps to its exact role; direct initializeMotors still dispatches once with its outer lifetime; standalone manual behavior remains unchanged; no startup-only extra writes/readback/finally restoration remain without source authority.

## 4. Allowed integration surface

Expected production surface: P and D; existing machine/tray persistence and binding code in CP, operator_controls or oem_runtime_store only where directly necessary for S1–S3 or the existing fallback owner for S6. Existing stage-contract metadata may be adjusted only to describe the corrected calls. No new queue, store, generic scheduler, polling collector or public control is authorized.

Every changed function must be attributable to S1–S7. Necessary adjacent changes require that traceable dependency, not an opportunistic cleanup. An unexpected unrelated defect shall be reported separately and shall not expand this specification automatically.

Preserve tracked and untracked work. Never restore an old full provider file over current repairs. Retain current recovery generation ownership, observation failure receipts, software interruption before persistence, physical Stop guards/addressing, single dispatch and independent latch predicates. No live-state migration is authorized; fresh-object construction must not reset real retained state.

## 5. Verification contract and current prohibition

The scenarios above are the acceptance definition, NOT permission to execute tests now. No tests, review passes or delegation are authorized by this document. No acceptance evidence is fabricated while those activities are prohibited.

If Christian later explicitly authorizes validation:
- Use independent OEM operation/state expectations and actual owner integration, not mocks that manually prefill the very constructors being implemented.
- Exercise each S1–S7 success, source no-op, failure/exception and relevant repeated-call branch; reproduce changed behavior on preserved input before claiming a fix.
- Use only the existing approved isolated venv/bwrap method, read-only source and disposable SQLite; run sequentially. No live devices, services, network, historical endpoints or external repository imports.
- Retain the existing affected regression/node denominator; no deleted or weakened assertions solely to obtain green results. Source-obsolete assertions require a source-backed replacement.
- Verify input preservation, full input-relative patch replay and unchanged source during final execution. Counts supplement, not replace, named behavior evidence.

Status per issue must distinguish: specified, implemented/connected, offline-verified (only when authorized), and any unresolved exact conflict. A blocked control, helper definition, passing fixture or commit is not completed OEM behavior. Physical verification and deployment are separate and outside these seven code corrections.

## 6. Completion and change control

The implementation deliverable, if authorized, is OEM-aligned connected code for all S1–S7 in the existing mechanisms, with every requirement mapped to changed or already-correct code. S3 and S7 must retain the qualifications stated above. An already-correct portion is preserved and identified, not rewritten for activity.

No assertion of all-motion, full BioXP, physical or production acceptance follows from completion of this bounded specification. Conversely, unrelated unverified areas must not be used to postpone completion of S1–S7.

Any alteration to this fixed issue set, source behavior or architecture requires Christian's explicit amendment BEFORE implementation. This document is a proposed specification only; the present turn changes documentation and nothing else.
