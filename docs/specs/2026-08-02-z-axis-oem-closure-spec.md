# BioXP serial-206 Z-axis OEM closure specification

**Contract status:** source and Linux contract closed for the provider-owned Z surface. Deployment identity, controller state, and physical commissioning remain separate evidence ledgers and must never be inferred from this document.

## 1. Authority and physical identity

- Machine: BioXP serial `206`.
- Z transport identity: head board/CAN `4`, motor `1`.
- Coordinate contract: OEM source-positive Z, inclusive controller range `0..160000` steps.
- Canonical Linux owner: `Serial206OemInitializationProvider` backed by `Serial206ProductionPrimitiveAdapter`.
- A direct/raw route, generic composite, stale process, legacy signed-Z helper, or unit-test fake cannot grant Z preparation or reference authority.

### OEM C# anchors

- `ClassControlInterface.cs:2350-2368` — diagnostic-panel Z Home: `goHome(true, Z, 1791, true)`.
- `ClassControlInterface.cs:4165-4204` — `moveSteps` relative motion.
- `ClassControlInterface.cs:4254-4265` — `moveZ(z, current=31)`: clamp to dynamic `PSUDO_Z_HOME`, set Z max current, then absolute move.
- `ClassControlInterface.cs:4463-4506` — `moveTo`; the all-zero branch is a composite home, not a generic move.
- `ClassControlInterface.cs:4623-4632` — `MoveZHome(rehome=true)`: set/read max current `31`, then `goHome(rehome, Z, 1791, true)`.
- `ClassControlInterface.cs:4657-4687` — `homeGZ`: move Z to `PSUDO_Z_HOME`, home gripper, restore Z; it is not a Z-home alias.
- `ClassControlInterface.cs:4997-5052` — `HomeAxis("z")`: current prelude then `axisSearchHome(Z, 597)`.
- `ClassControlInterface.cs:3348-3353` — `initializeMotors` M01: `axisSearchHome(Z, 1791)`.
- `ClassHeadBoard.cs:60-124` — `goHome`: optional `moveToAbs(10000)` for rehome, `moveLeft(speed)`, home-switch polling, STOP, then `setHome`.
- `ClassHeadBoard.cs:368-400` — `axisSearchHome`: initial `setHome`, optional preclear from an active home switch, then `goHome(false, speed)`.
- `ClassMotor.cs:492-516` — `setHome`: controller coordinate write `SAP1=0`; no movement/search prelude.
- `ClassMotor.cs:641-663` — raw GAP9 `1` is the active Linux home-switch value.

## 2. Required separate Z semantics

| OEM mode | Linux semantic | Live status |
|---|---|---|
| `initializeMotors` M01 | `axisSearchHome(Z,1791)` | Provider initialization ledger only |
| panel/manual Z Home | `goHome(true,Z,1791,true)` | `oem.z.manual_home`; exact controller home proof publishes reference |
| board-test `HomeAxis("z")` | current `31`, then `axisSearchHome(Z,597)` | Distinct diagnostic intent; exact controller home proof publishes reference |
| `MoveZHome` | current `31`, then `goHome(rehome,Z,1791,true)` | Separate `oem.z.move_z_home` intent and source identity |
| no-motion `setHome` | `SAP1=0` plus exact zero readback and durable reference publication | `oem.z.set_home`; never motion proof |
| `moveSteps` | bounded relative movement from an ACKed current position | `oem.z.move_steps` after `referenced_ready` |
| `moveZ` | `max(PSUDO_Z_HOME, requested)`, current `31`, absolute movement | `oem.z.move_absolute` after `referenced_ready` |
| `homeGZ` | coupled Z/gripper source transaction | Distinct provider-owned live intent with caught-plate recovery evidence |
| `moveTo(0,0,0)` | composite home in OEM source | Provider-owned compound home, not a generic zero-distance move |
| `scriptmoveTo` | source path planner/executor | Strict provider-backed live route preserving serial order and parallel X/Y leaves |
| Z profile controls | `setMaxSpeed`, `setMaxAcc`, `setZaxisVmax`, `setZaxisCurrentmax`, restore original speed | Distinct provider controls with exact readback |
| coupled pipette methods | `moveGZ`, `lowerPipette`, `liftPipette` | Distinct provider intents and receipts |
| Z self-test projection | `MoveZHome -> moveZ(92049,31,false) -> HomeAxis(Z,597)`; tolerance `±100` | Live Z-only projection; non-Z self-test stages are not claimed |
| Z abort/recovery projection | full-machine `forceAbortMotion`; Z recovery `initialCheck -> Z rehome` | Abort is explicitly machine-wide; Z recovery is explicit and does not claim or release full `wakefrompause` job readiness |

For live `scriptmoveTo`, destination/options remain operator inputs, but current X/Y/Z positions, current location/well, tip state, gripper reference, plate state, pseudo-home, and LOC19 Y are provider/controller/durable-state authority. Caller-supplied preview machine-state fields are ignored in live mode.

The generic runtime `wakefrompause` command stays paused and reports
`full_oem_wakefrompause_not_implemented`. Door-close events do not queue the
Z-only recovery projection under that full-system name. Operators may invoke
`/motion/oem/z/resume_after_abort` to recover Z reference authority after the
machine-wide abort, but this does not restore thermal state or resume a job.

## 3. Canonical lifecycle and durable authority

The Z lifecycle schema is `bioxp.serial206_z_lifecycle.v2`.

States:

1. `unprepared`
2. `prepared_unreferenced`
3. `executing`
4. `awaiting_operator_observation`
5. `referenced_ready`
6. `failed_latched`

`awaiting_operator_observation` remains readable for legacy/interrupted receipts,
but successful controller-proven home, self-test, composite home, and resume paths
publish reference immediately. Physical observation is optional commissioning
evidence, not a normal-operation admission gate.

Authority is valid only when all applicable identities agree:

- current process ownership generation;
- provider receipt generation;
- current board-lifecycle generation minted after a complete acknowledged command-64 `0 -> 1` cycle;
- durable Z lifecycle generation;
- immutable board `4` / motor `1` binding;
- exact controller ACK/readback evidence;
- durable robot-local reference persistence.

Any non-provider board-4 command-64 activation or deactivation clears preparation-derived Z authority. Ownership or board-generation drift fails closed. Legacy `v1` records without board-generation evidence are rewritten to `v2`, downgraded to `unprepared`, and marked `desynced`; a `v1` record that already carries exact board-generation evidence is schema-migrated without inventing new authority.

## 4. Movement success contract

A motion HTTP response or dispatch is not movement success. Relative and absolute Z moves require all of the following:

1. provider state is `referenced_ready`;
2. ownership and board-lifecycle generations are fresh;
3. fresh motion interlock and source profile verification pass;
4. pre-position is a genuine integer from an ACK `100` controller readback;
5. requested/effective target is within `0..160000`;
6. source current write/readback is ACKed where required;
7. physical command ACK is `100`;
8. controller terminal readback is ACK `100` with genuine integer speed `0`;
9. target-reached status `128` is newer than the event-window cursor;
10. no fresh controller error event `13`, `14`, or `130` exists;
11. final ACKed controller position equals the exact target.

Failure invokes provider STOP and latches/desynchronizes reference authority. The receipt still distinguishes command ACK, terminal controller state, and independent physical observation.

## 5. Homing and reference contract

### Manual `MoveZHome`

- Requires the provider-owned prepared profile and live interlock.
- Uses literal `goHome(true,1791)` without a Linux coordinate-distance cutoff.
- Requires a fresh event window, ACKed move-left command, ACKed double STOP, ACKed speed-zero terminal readback, GAP9 home predicate, and ACKed `SAP1=0`/position-zero proof.
- Preserves a compact `home_summary` with the exact `false_home_guard`, before/after positions, switch values, and controller evidence even when the bounded raw trace is omitted.
- A successful controller home publishes durable reference authority immediately from GAP9, position-zero, speed-zero, ACK, and terminal evidence. Independent observation remains optional commissioning evidence.

### Diagnostic `HomeAxis("z")`

- Remains separate at speed `597`.
- Uses the distinct diagnostic route and source identity without an additional confirmation phrase.
- Exact GAP9/position/speed terminal proof publishes reference and does not create an observation wait.

### No-motion `setHome`

- Writes only controller `SAP1=0` at board `4`, motor `1`.
- Requires ACKed write/readback, ACKed position-zero readback, a live board-lifecycle generation, and durable reference-store success.
- May repair a physically confirmed home position without further movement.
- Does not prove that physical movement, switch transition, torque, or motor output occurred.

### GAP10

GAP10/right-switch evidence is diagnostic only. `/motion/oem/z/live_right_reference` is retired and cannot establish reference authority.

## 6. Preparation and enclosure semantics

Source `initialCheck` reads door and latch as observations before the command-64 cycle; it does not invent an equality-to-closed gate. Each observation must be a mapping with ACK `100` and a genuine integer value in `{0,1}`. Strings and booleans are rejected.

Actual motion admission is stricter: fresh canonical state must report genuine integer `door_sensor=1` and `latch_sensor=1`, a valid 24 V observation, an armed motion interlock, live transport/CAN ownership, and all required references.

Preparation performs no homing and no movement. Its acknowledged order is:

1. read rail/door/latch;
2. command-64 deactivate all four source boards;
3. command-64 activate all four source boards;
4. mint board-lifecycle generation;
5. wait for board state;
6. initialize exact source profiles without motion;
7. verify serial-206 persistent Z wiring state (`GAP12=1`, `GAP13=0`) and exact parameter readbacks; OEM source initialization leaves these Z parameters unchanged.

## 7. STOP contract

Z STOP is the source double-delivery command. STOP and abort are the Z semantic actions admitted on the independent interrupt lane: the cockpit must not put either behind normal-command pending state or catalog fetch, and the robot operator plane must not queue controller delivery behind normal invocation/provider lifecycle locks. HTTP completion may still wait for durable lifecycle/reference reconciliation after delivery.

The provider increments a Z interrupt epoch before controller delivery. A normal Z intent that was waiting before that epoch is rejected; an intent already executing is STOPped and its lifecycle/reference authority is durably failed-latched and desynchronized after controller delivery.

Success requires:

- both STOP deliveries ACK `100`; and
- a later speed readback with ACK `100` and genuine integer value `0`.

`stopped=true` without the ACKed zero value is not accepted. STOP controller proof is not independent physical proof. If interrupted-command or reference-state reconciliation cannot be persisted, the response remains failed even when controller STOP delivery itself was acknowledged.

## 8. Twenty-one-item closure ledger

| # | Denominator requirement | Linux closure | Status |
|---:|---|---|---|
| 1 | No fakes in live path | Production provider binds only to managed tester/pipette owner; tests use fakes only as contracts | Closed |
| 2 | Six source home contexts stay distinct | Startup, manual, diagnostic, `MoveZHome`, `homeGZ`, and composite `moveTo` remain separately named | Closed |
| 3 | Manual `goHome(true,1791)` literal | No coordinate cutoff; compact false-guard evidence retained | Closed |
| 4 | Diagnostic `axisSearchHome(597)` separate | Distinct source identity; exact controller home proof publishes reference | Implemented; final acceptance pending |
| 5 | Current semantics | Z source current `31`; no invented standby write in manual/diagnostic paths | Closed |
| 6 | GAP9 exact home predicate | Raw active value `1`; ACKed readback and transition evidence | Closed |
| 7 | GAP10 diagnostic only | Live-right-reference route retired; its simultaneous assertion at GAP9 top is not opposite-end travel authority | Closed |
| 8 | Switch-mask truth | Machine-bound `GAP12/right-disable=1`; `GAP13/GAP9-left-disable=0`; retained OEM path is read-only and performs no SAP12/SAP13 repair or lifecycle invalidation | Closed |
| 9 | Relative `moveSteps` | Production keyword binding, live current-position bounds, ACK/terminal/event/final-position proof | Closed |
| 10 | Absolute `moveZ` | Dynamic pseudo-home clamp, current write/readback, guarded production primitive | Closed |
| 11 | Coordinate bounds | `0..160000`; unreferenced/pre-home values are never motion authority | Closed |
| 12 | STOP/abort interruption | STOP double ACK and zero speed; abort shares the independent lane and invalidates reference | Implemented; active-motion acceptance pending |
| 13 | Async event freshness | Monotonic window cursor; stale `128/130` ignored, fresh `130` fails | Closed |
| 14 | Startup ordering | Full acknowledged board cycle precedes profile generation | Closed |
| 15 | Composite source semantics | `homeGZ`, nonzero pathing, all-zero compound home, `moveGZ`, and pipette methods remain distinct provider-owned operations | Implemented; final acceptance pending |
| 16 | `PSUDO_Z_HOME` state | Durable machine status uses exact `500` or `65000`; absolute move consumes it | Closed |
| 17 | Persistent lifecycle and evidence boundary | Schema `v2`, migration, idempotent receipts; observation is evidence rather than an operational gate | Implemented; final acceptance pending |
| 18 | Snapshot/interlock authority | Fresh canonical snapshot, typed enclosure, rail, arm, references | Closed |
| 19 | Operator API | Semantic Z actions, admission, history, command-ID linkage, generation binding, independent STOP/abort dispatch | Implemented; BMS/cockpit acceptance pending |
| 20 | Tests and hardware separation | Executable source contracts and live evidence stay separate; includes profile/path/coupled/self-test coverage | Acceptance pending |
| 21 | Explicit reporting | This ledger plus release/controller/physical ledgers in the deployment report | Acceptance pending |

## 9. Physical evidence boundary

The supervised manual-home run `20260804T0150Z-z-manual-home` physically moved the Z mechanism. Controller evidence recorded position `-1808468 -> -1969141`, GAP9 `0 -> 1`, and terminal speed `0`; Christian independently confirmed the visible physical movement worked.

That run nevertheless returned `controller_async_error_130`, did not complete `setHome`, and left the durable lifecycle `failed_latched/desynced`. Therefore:

- physical movement: operator-confirmed pass;
- controller home/reference: not established by that run;
- STOP terminal state: ACKed and speed zero;
- release source identity for that physical run: not proven by the observation alone.

A later no-motion `setHome` may establish controller/reference state only after the final release is deployed, current board-generation preparation is proven, the physical position is still accepted as home, the SAP1 write and zero readback are ACKed, and durable reference persistence succeeds. No additional movement is implied or authorized by this specification.

## 10. Verification anchors

Primary contract tests:

- `tests/test_z_axis_oem_contract.py`
- `tests/test_oem_serial206_initialization_provider.py`
- `tests/test_oem_serial206_provider_api_binding.py`
- `tests/test_z_oem_api_contract.py`
- `tests/test_motion_safety_control_plane.py`
- `tests/test_bioxp_oem_initialize_motors_live_parity.py`
- `tests/test_oem_homing_fail_closed.py`
- `tests/test_bioxp_oem_homing.py`
- `tests/test_oem_tranche_safety_regressions.py`
- `tests/test_operator_controls.py`

Release acceptance must report exact committed tree, pushed branch, deployed tree, process identity, live provider projection, controller state, and physical observation as separate facts.
