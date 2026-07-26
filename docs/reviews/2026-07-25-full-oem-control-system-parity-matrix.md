# BioXP 3200 serial-206 OEM control-system parity gap matrix

Date: 2026-07-25
Implementation parent: `928a7d310621fbf6e90c4f5e03d5b67f2cf5fa19`
Working branch: `hermes/full-oem-control-parity-20260725`
Machine authority: serial `206`, acquisition `20260719T024740Z-4a7fe6783205846c`, `OEM_EVIDENCE_LOCK.json`
Current canonical lock SHA-256: `a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c` (the earlier `148b2248...` value identifies historical lock bytes, not the registry-selected current lock)
Movement registry: `docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json`

## Status vocabulary

- **accepted-foundation** — already independently accepted at parent `928a7d3`.
- **rejected-checkpoint** — independently reviewed exact candidate failed; no acceptance or deployment claim is permitted.
- **implemented-this-tranche** — implemented and locally tested on this branch, not independently accepted or deployed.
- **implemented-not-integrated** — working primitive exists but is not yet bound into the full OEM lifecycle.
- **source-present-unline-locked** — recovered source exists; exact production contract is not yet sealed.
- **scaffold-only** — dry-run/diagnostic shape exists and must not be represented as live parity.
- **not-commissioned** — software may exist, but no authorized physical proof has been run.

## Lifecycle matrix

| Order / branch | OEM authority | Required behavior | Current Linux truth | This tranche / remaining gate |
|---|---|---|---|---|
| ControlLib construction | `BioXPMainWindow:642-675`; `ControlLib:963-984` | construct `ControlLib`; conditionally initialize/status pipettes and call `initializeMotorsWithoutMotion` only when `CAN_READY` | accepted lower-level primitives; complete constructor owner absent | selected-path planner now places this before `initializeEnvironment`; live provider remains unbound |
| Application admission | `BioXPMainWindow.initializeEnvironment:973-1026` | branch on `CAN_READY`, then run hardware-affecting `initialCheck`; branch on door/latch before enqueue | accepted initial-check foundation plus typed planner | exact CAN/door/latch predicates are required; unresolved state makes `plan_available=false` |
| initializeSystem claim/reentry | `BioXPMainWindow:989-997,1046-1341`; worker `2030-2051` | enqueue only from the closed door/latch branch; worker `UpdateCheck` can suppress initialization; preserve reentry and finally cleanup | accepted queue/worker foundation | planner records the worker gate; live update-check receipt and full executor remain absent |
| ShipMode PARK | `BioXPMainWindow:1127-1134,1335-1340` | close door and require its Boolean success before warning/shutdown; otherwise terminal `DOOR_LATCH_ERROR`; return | source present; live path not integrated | Planner now stops at a typed door-close handoff with explicit success/failure terminals; it does not assume shutdown success |
| Initial check | `BioXPMainWindow:976-979,1140-1144`; `ControlLib:8728-8795` | board wait, LED, door/latch/24V checks; OEM callers ignore its Boolean return | **accepted-foundation** | both calls are marked hardware-affecting and result-ignored; no readiness claim is derived from return truthiness |
| SavedStatus 3/4 recovery | `BioXPMainWindow:1144-1155` | warning, initializeMotion, inspectCover, unlock/return; no normal fall-through | previously not a canonical full-run branch | exact early-return graph **implemented-this-tranche**; providers not commissioned |
| `initializeMotorsWithoutMotion` | `ControlLib:963-984`; `ClassControlInterface:3181-3265` | constructor-time, `CAN_READY`-conditional board wait; heater/chiller/profile/deck writes before `initializeEnvironment` | exact byte/order executor exists in `usb_driver.py:4060-4221`; covered by accepted startup tests | selected-path planner ordering corrected; live constructor binding and physical proof remain absent |
| initializeMotion flags | `ControlLib:8797-8804` | reset stop/pause/motion flags before motion init | scaffold/legacy state only | typed stage **implemented-this-tranche**; canonical state adapter remains |
| M01–M19 initializeMotors | `ClassControlInterface:3348-3421`; `oem_movement_ledger.py` | exact Z/G/X/Y/door homing, X=6000, door predicate/failure, Y home, chiller GP8 writes, initialized status, G idle current | **accepted-foundation**, persistent ordered ledger and queued executor | all 19 child stages now projected individually in full ledger; no hardware run |
| Tip query | `ControlLib:8806-8813`; `ClassPipette.QueryTipStatus:571-589` | query actual tip state, exact four-channel result | transport/projection primitives exist; full integration absent | typed stage **implemented-this-tranche**; robot-owned exact predicate required before API create |
| No-tip branch | `ControlLib:8854-8856` | skip remediation and continue | dry-run shape only | graph **implemented-this-tranche** |
| Stale-tip remediation | `ControlLib:8814-8856`; `ClassPipetteCollection:677-748,1176-1358` | open TC door; route park→waste; update location; eject all; Z 80000; X 79000; verify empty; initiate/retry once | four-channel `?31`/`E1R` provider is implemented and locally tested, but not lifecycle-bound | transaction lock, strict hardware provenance, and partial-mutation/post-attempt evidence added; lifecycle binding and commissioning remain absent |
| Self-test due gate | `BioXPMainWindow:1163-1171` | date/policy decision; do not silently run if not due | no canonical full lifecycle | typed branch **implemented-this-tranche** |
| TC/RC/OC + motion self-test | `ControlLib:10688-10999` | launch TC/RC/OC tasks, run motion checks while they are active, then join/evaluate and restore outputs | strict receipt evaluators exist; no live orchestrator | planner records launch→overlap→join semantics; provider remains unbound and uncommissioned |
| Camera gate | `BioXPMainWindow:1172-1180`; selected `CheckCamera=true`, `CameraInstalled=true`, calibrated | exact policy/config branch | machine snapshot and explicit camera ownership routes exist | exact policy projection corrected **implemented-this-tranche** |
| CheckCamera | `ControlLib:1929-1960` | LEDs, location/motion, frame processing, result, LEDs off, park; does **not** release the frame grabber | explicit probe/stream/snapshot/control APIs exist; strict source-cited receipt evaluator only | **implemented-not-integrated**; evaluator does not prove a live CVision provider and does not invent camera-session disposal |
| inspectCover | `ControlLib:3663-3768`; native CVision assets/settings | always force the high-resolution home move first; if deck inspection is disabled return true; otherwise process `17→19→20→18`, including non-log-only early returns | source/assets and sequential source-cited receipt evaluator exist; no bound provider | evaluator permits only an observed source-order prefix on early return and does not invent LED/session cleanup; provider remains unbound and uncommissioned |
| parkGantry | `ControlLib:7071-7122` | source branch over current location/rehome, route/position table, safe terminal location | route/motion primitives present; no sealed full provider | **source-present-unline-locked**, not commissioned |
| StartMode DevMode (enum 0) | `ControlLib:3173-3176`; `BioXPMainWindow:1203-1212` | development-ready terminal | no canonical typed terminal | exact enum name and development terminal represented in planner |
| StartMode WebMode/LocalMode/TradeShowMode (enum 1/2/3) | `ControlLib:3163-3210`; `BioXPMainWindow:1213-1295` | distinct web job-admission, local job-admission, and trade-show terminals | BMS command plane exists only for accepted stagewise path | exact names/terminals represented; BMS full-run controls remain blocked |
| Normal shutdown | `BioXPMainWindow:3919-3934`; `ControlLib.Shutdown:4773-4780` | call ControlLib shutdown; dispose/null frame grabber; app shutdown | camera lifecycle has owned stop paths; no exact full-run shutdown receipt | **source-present-unline-locked** |
| Failure/restart | source exceptions plus Linux safety contract | no auto-resume after unresolved hardware stage; preserve last evidence | **accepted-foundation** failed-closed command worker | full-run restart block **implemented-this-tranche** |

## Subsystem evidence summary

### Movement/deck

- Serial-206 axis/door profiles, position table, route graph, deck IO, LED, solenoid/latch, and M01–M19 source are present in the locked corpus.
- M01–M19 execution is accepted only within its reviewed scope.
- The new full ledger links each M-stage by its existing `movement_ledger_stage`; it does not duplicate execution or allow callers to choose the next stage.

### Thermal/chillers

- Startup baseline is already encoded in the exact `initializeMotorsWithoutMotion` frame order.
- Diagnostic `/thermal/baseline` and `/chiller/baseline` use different values and must not be substituted for OEM startup.
- M16/M17 repeat OC/RC cool-rate writes at `-0.025 C/s` after homing.
- TC/RC/OC self-test parity remains a separate source-derived orchestration task.

### Camera/cover vision

- Linux explicit camera probe/session ownership is real, but generic camera availability is not equivalent to OEM `CheckCamera` or `inspectCover`.
- CVision source/assets/settings are present; this is **unreviewed/unline-locked**, not absent.
- Camera/CVision operations remain disabled from the full-run live API until image fixtures, exact call order, deterministic fake results, failure classification, and supervised proof are accepted.

## API safety boundary implemented in this tranche

`POST /oem/runtime/movement-runs` accepts only:

- fixed command `initialize_oem_movement_lifecycle`;
- `operator_ack=INITIALIZE`;
- expected machine serial;
- expected movement-registry SHA-256;
- idempotency key;
- `mode=dry_run`.

Axis, stage, position, speed, current, timeout, raw frame, and branch predicates are rejected. Branch inputs are derived by the robot. Live mode is blocked until commissioned providers are bound.

## Honest parity verdict at this point

**Not full OEM parity.** Exact checkpoints `59c255c2acb1b30ea7f4b6f09e65006b49b24f56` and `a098f034742cbcf6538d40527451717176357a93` are both **rejected** by independent correctness/safety and source-parity reviews. The current unsealed working tree repairs the second review's lock identity, source-anchor, constructor-nesting, producer-inventory, ShipMode, M-stage linkage, launched-task-result, and camera/cover receipt defects and has local no-hardware tests only. It is a source-cited selected-path planner, not an executable full lifecycle and not accepted source parity. Complete robot-owned providers for pipette lifecycle binding, TC/RC/OC/motion self-test, CheckCamera/CVision, inspectCover, parkGantry, update-check/finally behavior, shutdown disposal, and BMS operator controls remain absent or uncommissioned. No physical subsystem was actuated while producing this matrix.
