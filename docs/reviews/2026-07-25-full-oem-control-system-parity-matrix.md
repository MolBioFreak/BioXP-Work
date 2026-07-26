# BioXP 3200 serial-206 full OEM control-system parity matrix

Date: 2026-07-25
Implementation parent: `928a7d310621fbf6e90c4f5e03d5b67f2cf5fa19`
Working branch: `hermes/full-oem-control-parity-20260725`
Machine authority: serial `206`, acquisition `bioxp_runtime_acquisition_20260718T012321Z`, `OEM_EVIDENCE_LOCK.json`
Movement registry: `docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json`

## Status vocabulary

- **accepted-foundation** — already independently accepted at parent `928a7d3`.
- **implemented-this-tranche** — implemented and locally tested on this branch, not independently accepted or deployed.
- **implemented-not-integrated** — working primitive exists but is not yet bound into the full OEM lifecycle.
- **source-present-unline-locked** — recovered source exists; exact production contract is not yet sealed.
- **scaffold-only** — dry-run/diagnostic shape exists and must not be represented as live parity.
- **not-commissioned** — software may exist, but no authorized physical proof has been run.

## Lifecycle matrix

| Order / branch | OEM authority | Required behavior | Current Linux truth | This tranche / remaining gate |
|---|---|---|---|---|
| Application environment | `BioXPMainWindow.initializeEnvironment:973-1027` | Load resources, construct/queue startup, preserve app state | **accepted-foundation** plus typed full-lifecycle stage | Persistent run creation is **implemented-this-tranche** |
| initializeSystem claim/reentry | `BioXPMainWindow.initializeSystem:1046-1058`, motion worker `2030-2100` | single owner, reentry guard, queued execution | **accepted-foundation** queue/worker | Full-run durable state and restart block are **implemented-this-tranche** |
| ShipMode PARK | `BioXPMainWindow:1127-1134` | door action, shutdown-ready warning, OS shutdown, return | source present; live path not integrated | Typed early terminal is **implemented-this-tranche**; door/shutdown provider remains **source-present-unline-locked** |
| Initial check | `ControlLib.initialCheck:8728-8759`, `queryDoorStatus:8762-8770` | board wait, LED, door/latch/24V checks, fail closed | **accepted-foundation** | Reuse existing canonical initial-check receipt |
| SavedStatus 3/4 recovery | `BioXPMainWindow:1144-1155` | warning, initializeMotion, inspectCover, unlock/return; no normal fall-through | previously not a canonical full-run branch | exact early-return graph **implemented-this-tranche**; providers not commissioned |
| `initializeMotorsWithoutMotion` | `ClassControlInterface:3181-3265` | board wait; heater off twice; chiller PWM=0 OC/RC; X/Y/Z/G/door profiles; chiller GP8=-25 OC/RC; TC GP7=2500 GP8=-2000; deck white | exact byte/order executor exists in `usb_driver.py:4060-4221`; covered by accepted startup tests | projected as explicit full-lifecycle stage **implemented-this-tranche**; physical controller proof remains **not-commissioned** |
| initializeMotion flags | `ControlLib:8797-8804` | reset stop/pause/motion flags before motion init | scaffold/legacy state only | typed stage **implemented-this-tranche**; canonical state adapter remains |
| M01–M19 initializeMotors | `ClassControlInterface:3348-3421`; `oem_movement_ledger.py` | exact Z/G/X/Y/door homing, X=6000, door predicate/failure, Y home, chiller GP8 writes, initialized status, G idle current | **accepted-foundation**, persistent ordered ledger and queued executor | all 19 child stages now projected individually in full ledger; no hardware run |
| Tip query | `ControlLib:8806-8813`; `ClassPipette.QueryTipStatus:571-589` | query actual tip state, exact four-channel result | transport/projection primitives exist; full integration absent | typed stage **implemented-this-tranche**; robot-owned exact predicate required before API create |
| No-tip branch | `ControlLib:8854-8856` | skip remediation and continue | dry-run shape only | graph **implemented-this-tranche** |
| Stale-tip remediation | `ControlLib:8814-8856`; `ClassPipetteCollection:677-748,1176-1358` | open TC door; route park→waste; update location; eject all; Z 80000; X 79000; verify empty; initiate/retry once | `oem_pipette_collection.py` is **scaffold-only** | exact 7-stage branch **implemented-this-tranche**; live provider/ACK/readback remains **source-present-unline-locked** |
| Self-test due gate | `BioXPMainWindow:1163-1171` | date/policy decision; do not silently run if not due | no canonical full lifecycle | typed branch **implemented-this-tranche** |
| TC/RC/OC self-tests | `ControlLib:10688-10999` | threaded TC, RC, OC checks, door close, result/error handling | thermal/chiller primitives exist, but full self-test orchestration is absent | **source-present-unline-locked**; not commissioned |
| Motion self-test remainder | `ControlLib:10688-10785` | XY/Z/G/door/location/plate checks and failures | motion primitives exist; no canonical self-test provider | **source-present-unline-locked**; not commissioned |
| Camera gate | `BioXPMainWindow:1172-1180`; selected `CheckCamera=true`, `CameraInstalled=true`, calibrated | exact policy/config branch | machine snapshot and explicit camera ownership routes exist | exact policy projection corrected **implemented-this-tranche** |
| CheckCamera | `ControlLib:1929-1960` | LEDs, location/motion, frame processing, result, LEDs off, park | explicit probe/stream/snapshot/control APIs exist; not OEM `CheckCamera` parity | **implemented-not-integrated** / CVision call graph **source-present-unline-locked** |
| inspectCover | `ControlLib:3663-3768`; native CVision assets/settings | selected deck inspection; deck route/vision; exact ErrorStatus | source/assets are present; `oem_pipette_collection` currently emits only a payload shape | **scaffold-only**, provider not commissioned |
| parkGantry | `ControlLib:7071-7122` | source branch over current location/rehome, route/position table, safe terminal location | route/motion primitives present; no sealed full provider | **source-present-unline-locked**, not commissioned |
| StartMode Dev | `BioXPMainWindow:1203-1212` | manual-ready terminal | no canonical typed terminal | typed terminal **implemented-this-tranche** |
| StartMode TradeShow | `BioXPMainWindow:1213-1221` | trade-show-ready terminal | no canonical typed terminal | typed terminal **implemented-this-tranche** |
| StartMode Web/Local | `BioXPMainWindow:1222-1295` | job-admission terminal, not generic ready | BMS command plane exists for accepted stagewise path | typed terminal **implemented-this-tranche**; BMS full-run controls pending |
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

**Not full OEM parity yet.** The full source-shaped lifecycle graph, persistent run contract, exact M01–M19 linkage, authority checks, restart blocking, and fixed API are implemented locally. Pipette remediation, TC/RC/OC self-test, CheckCamera/CVision, inspectCover, parkGantry, shutdown disposal, and BMS full-run controls still require implementation/acceptance. No physical subsystem was actuated while producing this matrix.
