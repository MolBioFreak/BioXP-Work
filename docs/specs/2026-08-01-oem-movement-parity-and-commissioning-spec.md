# BioXP 3200 OEM Movement Parity and Commissioning Specification

**Status:** Major-gap audit and acceptance specification

**Date:** 2026-08-01

**Machine:** BioXP 3200 serial 206

**Scope:** 24 V/motion preparation, axis movement, homing, stopping, gripper, thermal door, `initializeMotors`, and `initializeMotion`

**Out of scope:** unrelated robot controls, protocol/job execution, deployment implementation

## 1. Frozen basis

| Surface | Exact identity |
|---|---|
| OEM authority | `OEM_EVIDENCE_LOCK` SHA-256 `a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c` |
| Movement registry | SHA-256 `171a99b3da0a6efacc6a105cb2ff7f1e2c7ee0dc79f3bba6da2470c397329e35` |
| Robot candidate | `798311ba77d72ebbe96c8b89b2b11c8e77216487` |
| Robot deployed process | `/home/molbiofreak/bioxp_release_76c55db` / source revision `76c55dbaed911188e9d94c93a2cb42875af1d667` |
| BMS BioXP source audited | `6b3ca5fe8304ba4a8fbb2191ef10fc6d6b8a6362` |
| BMS final source/API identity | `b7e69b812b99e61e7f7687ff1d3ce3c67159ad67`; the intervening commit changes only MD queue files, with no BioXP file changes |

The deployed robot is not the accepted candidate. No parity claim may cross that revision mismatch.

## 2. Major gaps

| Priority | Gap | Current evidence and impact | Required parity outcome |
|---|---|---|---|
| P0 | No source-grounded global **24 V On/Off** operation | `/motion/power/enable` calls `motor_enable_sequence()` (`src/bioxp/api.py:5219-5225`). Its underlying `enable_motor_power()` sends unacknowledged reverse-engineered `cmd14/type0` and `cmd14/type1` writes (`src/bioxp/usb_driver.py:1007-1018`). OEM source exposes a read-only `query24VSensor()` (`ClassIOControl.cs:92-110`) and per-board activation (`ClassControlInterface.cs:3474-3493`), not a proven BioXP global power toggle. Candidate `798311b` correctly quarantines this route (`src/bioxp/operator_controls.py:257-260`). The last preserved non-homing startup receipt saw boards 4–6 activate successfully, board 7 reject command 64, and a valid 24 V sample with `no24v=false`; that is evidence the rail was present then, not evidence the named UI power button works. | Remove the false global power command from operator use. Bind an exact OEM **board activation + no-motion motor preparation** transaction with per-board ACKs, axis-parameter readback, 24 V sensor readback, and truthful failure. Resolve the board-7 applicability/rejection explicitly. A global power toggle remains unsupported unless immutable OEM/trace evidence proves it. |
| P0 | Deployed runtime is stale | Live service CWD is `bioxp_release_76c55db`; accepted candidate is `798311b`. The newer candidate adds CAN admission and quarantines fake power/emergency controls. | Deploy one reviewed exact revision, restart only with Christian's approval, and prove process CWD/hash/OpenAPI/registry all match before any physical test. |
| P0 | Readiness state is contradictory after reconnect | Live canonical state was invalidated by `runtime_reconnect`: snapshot missing, `rail_24v=null`, `motion_arm=null`, `CAN_READY=null`, and startup `not_run`. The operator dashboard still reported motion enabled because it derives that field only from maintenance flags (`operator_controls.py:415-421`). | A movement action is enabled only when the same-epoch transport, CAN readiness, fresh snapshot, door/latch, 24 V sensor, maintenance state, and required axis reference all pass. Dashboard and admission must use the same predicate. |
| P0 | No accepted physical aggregate abort | BMS explicitly reports “Physical Emergency Abort Unavailable”; candidate quarantines `/oem/runtime/emergency_stop` because it records lifecycle state without dispatching the OEM physical aggregate abort (`operator_controls.py:257-260`). OEM `ControlLib.forceAbortMotion()` is source-visible at `ControlLib.cs:10564-10606` and fans into `ClassControlInterface.forceAbortMotion()` at `ClassControlInterface.cs:5095-5104`. | Implement and live-verify that OEM physical abort/stop fan-out. Until then, no full homing/full-travel sequence. Only explicitly approved bounded commissioning moves may run with an on-machine observer and an already verified component-stop path. |
| P0 | Current movement commands are proven failing | Preserved BMS receipts contained six `run_axis_diagnostic` failures: four Z requests rejected HTTP 409 because Z is desynced; Z home rejected HTTP 409 because the GAP9/GAP10 predicate is uncommissioned; gripper open commanded movement but received no move ACK, position delta remained `0`, and returned HTTP 409 ambiguous/failure. | Each movement primitive must return command ACK, terminal stop, position/switch evidence, and operator physical observation. Any absent ACK, zero unexpected delta, stale state, or ambiguous completion is `FAIL`, never success. |
| P1 | Full OEM homing lifecycle is not executable | `/motion/oem/initialization/run` and `/motion/oem/initialize_motion` refuse `run_homing=true` with `literal_direct_oem_stage_rewrite_pending` (`api.py:5962-6004`). `meta.activate_motion` and `meta.full_initialization` are unavailable. | Implement the literal serial-206 sequence and branch outcomes from `ClassControlInterface.initializeMotors()` (`ClassControlInterface.cs:3348-3421`), then `ControlLib.initializeMotion()` including stale-tip remediation (`ControlLib.cs:8797-8856`). No monolithic route is enabled before each stage passes independently. |
| P1 | Axis reference predicates are not commissioned | Z is persisted `desynced`; Z home is explicitly blocked pending live GAP9/GAP10 truth. No current accepted physical result exists for X, Y, gripper, or thermal-door homing on the deployed revision. | Commission switch polarity, direction, bounds, stop behavior, home search, set-home, and post-home position independently for X, Y, Z, gripper, and thermal door. Persist reference state only after controller and physical evidence agree. |
| P1 | Two BMS movement control planes disagree | Generic operator actions use route-specific admission and disable blocked actions. `BioXpCockpit` separately submits `run_axis_diagnostic` (`BioXpCockpit.tsx:164-180,284-315`); its registry explicitly sets `requires_hardware_ready=False` (`command_registry.py:112-122`), so failures can arrive only after robot HTTP dispatch. | One authoritative admission predicate must govern both surfaces. The manual cockpit must query the exact action admission and show the specific blocker before enabling a button. |
| P1 | Failure evidence is not durable | BMS command history initially exposed the six failures above, then became empty after the API process changed/restarted. The current store is process-local. | Persist bounded command receipts durably by command ID, revision, generation, route, request, HTTP status, machine reason, controller evidence, and operator assessment. UI must display the nested machine reason, not only “HTTP 409”. |

## 3. Required OEM movement contract

The movement-ready transaction must preserve this source-derived order and stop on the first failed stage:

1. Establish one transport owner and same-epoch CAN readiness.
2. Collect a fresh query-only hardware snapshot.
3. Evaluate door, latch, and 24 V sensor polarity from source/IL-resolved semantics.
4. Run exact per-board `activateBoard()` and require an ACK for every configured board.
5. Run `initializeMotorsWithoutMotion()` with serial-206 parameters and read each written parameter back (`ClassControlInterface.cs:3181-3265`).
6. Commission component stop before component movement.
7. Execute the literal `initializeMotors()` stages: Z home; gripper current/clear/home; X home/set-home/+6000; Y home/set-home; thermal-door home/close check; post-current/cooling state (`ClassControlInterface.cs:3348-3421`).
8. Execute `initializeMotion()` stale-tip/no-tip branches exactly, including verified pipette status and terminal state (`ControlLib.cs:8797-8856`).

Every stage returns a durable receipt. A transport ACK alone does not prove physical movement.

## 4. Christian-approval gate for live tests

No live row below may execute from this document alone.

Before each row, the operator must present Christian with:

- the exact test ID;
- exact deployed revision and ownership generation;
- exact HTTP method, path, and JSON body;
- expected component, direction, maximum distance/time, and terminal state;
- the stop/abort action and the on-machine observer;
- the evidence path that will receive pre-state, controller frames, post-state, and physical observation.

Execution requires an explicit approval naming that one test ID. Approval is single-use. A `FAIL` or unexpected observation stops the matrix and invalidates approval for later rows.

## 5. Approval-gated physical test matrix

| ID | Test | Prerequisite | Required PASS evidence | Current result |
|---|---|---|---|---|
| T00 | Exact deployed-build proof | None; no hardware I/O | Service CWD/hash, OpenAPI, registry, and BMS target all identify the same accepted revision | **FAIL** — deployed `76c55db`, candidate `798311b` |
| T01 | Query-only canonical snapshot | T00 | Same-epoch transport/CAN; fresh door/latch/24 V/axis observations; no recovery or activation side effect | **BLOCKED / NOT RUN** |
| T02 | OEM board activation + no-motion setup | T01; exact replacement for false power route implemented | ACK from every configured board; `IsInitialized` truth; serial-206 current/speed/acceleration/stall/switch parameters read back; 24 V sensor interpreted; no axis movement | **PARTIAL HISTORICAL EVIDENCE, NOT ACCEPTED T02** — boards 4–6 succeeded, board 7 rejected command 64, and `no24v=false`; no test-specific approval/receipt |
| T03 | Per-component stop on idle X/Y/Z/gripper/door | T02 | Each stop gets controller ACK and terminal speed `0`; no ownership loss | **NOT RUN** |
| T04 | Small bounded X and Y direction jogs | T03; operator at robot | One approved direction at a time; ACK; nonzero expected position delta; stop; switch state safe; observed physical direction matches specification | **NOT RUN** |
| T05 | Small bounded Z direction jog away from physical limit | T03; camera/on-machine view; GAP9/GAP10 raw state known | ACK; expected bounded delta; immediate stop; no limit violation; physical direction recorded | **BLOCKED — Z desynced/predicate uncommissioned** |
| T06 | Gripper bounded clear/open/close primitives | T03; gripper path clear | ACK plus expected position delta and terminal state for each separately approved primitive | **FAIL — latest open had no move ACK and delta `0`** |
| T07 | Thermal-door home/open/close | T03; door path clear | Controller ACK, terminal stop, switch/sensor-confirmed state, and physical observation for each operation | **NOT RUN** |
| T08 | Individual X/Y/Z/gripper homing | T04-T07 relevant component passes | Source-correct search direction/polarity; stop at predicate; `setHome` ACK; position near zero; physical home confirmed | **BLOCKED / NOT RUN** |
| T09 | OEM concurrent `HomeXY` | Individual X and Y home pass; abort accepted | Both tasks complete within bounds; both home predicates and positions pass; no partial-success claim | **NOT RUN** |
| T10 | Literal `initializeMotors()` | T02-T09 all pass | Exact ordered stage ledger matches OEM source; every stage ACK/postcondition passes; safe terminal currents and door state | **BLOCKED — implementation refuses live homing** |
| T11 | Literal `initializeMotion()` no-tip branch | T10; pipette query commissioned | Exact branch trace; no stale-tip cleanup when no tip; terminal lifecycle ready | **BLOCKED — full provider not bound/commissioned** |
| T12 | Literal `initializeMotion()` stale-tip branch | T11; explicit separate approval and disposable setup | Thermal door, route move, tip eject, Z/X moves, tip re-query, and pipette group checks all match OEM source with physical evidence | **BLOCKED — full provider not bound/commissioned** |

## 6. Acceptance definition

Movement parity is **100%** only when:

- T00-T12 are all `PASS` on the exact accepted and deployed revision;
- no listed action is scaffolded, quarantined, dry-run-only, or physically unverified;
- the BMS button uses the same admission and route exercised by the accepted test;
- every HTTP failure is specific, durable, and visible to the operator;
- controller ACK, terminal state, sensor/position postcondition, and Christian/operator physical observation agree;
- a failed independent test blocks the sequence regardless of unit-test results.

## 7. Audit safety record

This gap audit used source inspection and passive status/history reads only. It issued **zero** reconnect, snapshot-collection, board activation, 24 V, recovery, initialization, stop, homing, movement, gripper, thermal-door, or emergency commands.
