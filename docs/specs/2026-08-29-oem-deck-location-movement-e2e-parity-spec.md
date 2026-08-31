# BioXP 3200 OEM Deck and Location Movement E2E Parity Specification

> **For Hermes:** Use `subagent-driven-development` to implement one approved work package at a time. Require specification review before code-quality review. Do not infer approval for tests, deployment, controller access, or physical motion from this document.

**Status:** Implementation specification and phased execution plan

**Date:** 2026-08-29

**Machine:** BioXP 3200 serial 206

**Goal:** Reproduce the source-defined OEM deck/location movement system in the native Linux robot runtime so a finite semantic request such as `move to OC chiller` resolves through the exact Serial-206 PositionTable, uses source-shaped movement and machine-state branches, enters the durable robot-owned command plane, updates state only after terminal controller success, and reaches the same typed BMS and agent surface.

**Architecture:** The robot remains the sole movement authority. A finite robot-owned destination catalog resolves operator and workflow intent into OEM `locationID`, `wellID`, and `plateName` semantics. The durable SQLite command plane obtains one coherent provider snapshot, compiles the appropriate `btnLOC1_Click`, `movExecution`, `scriptmoveTo`, `moveTo`, or plate-handling sequence, persists its plan identity, and executes it through the Serial-206 provider. BMS and agents submit typed semantic intent and display robot receipts. They never own coordinates, path selection, interlocks, or machine state.

**Technology:** Python, FastAPI, Pydantic, SQLite, the Serial-206 OEM provider, React, TypeScript, TanStack Query, and the existing BMS BioXP operator-control relay.

---

## 1. Controlling verdict

The current candidate contains the correct architectural skeleton and much of the central path planner. It is not execution-closed for named location movement.

Current status for this scope:

| Layer | Current state |
|---|---|
| Exact Serial-206 PositionTable source | Present and hash-identified |
| Configured PositionTable row coverage | 29 of 29 rows parsed |
| OEM diagnostic destination coverage | 24 distinct configured destination IDs found in source |
| `scriptmoveTo` branch planner | Major source branches present |
| Serial-206 X/Y/Z execution sinks | Present |
| SQLite command/runtime stores | Present |
| Canonical named-location command | Missing |
| Current live `scriptmoveTo` route | Fails on literal `LOC19` lookup |
| Canonical queue admission for `scriptmoveTo` | Missing |
| Source-correct machine-state authority | Incomplete |
| Controller-accepted named destinations through this path | 0 of 24 accepted |
| Physically accepted named destinations through this path | 0 of 24 accepted |

This specification closes the named diagnostic path, normal workflow path, source state transitions, durable command lifecycle, BMS relay, and acceptance matrix as one controlled program.

## 2. Authority and frozen basis

### 2.1 Authority order

The authority order is:

1. captured OEM binary bytes and raw IL;
2. the hash-locked Serial-206 machine corpus;
3. selected OEM runtime configuration and calibration bytes;
4. source projections tied to captured assemblies;
5. current Linux implementation and tests, used only to locate deviations.

Replacement behavior, generic deck YAML, current comments, tests, and operator UI cannot redefine OEM semantics.

### 2.2 OEM evidence

| Evidence | Identity |
|---|---|
| Canonical evidence lock | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/derived/oem_runtime_parity_spec_20260719/OEM_EVIDENCE_LOCK.json` |
| Evidence-lock SHA-256 | `a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c` |
| Serial-206 machine config | `.../oem_machine_runtime_bundle_serial206/appdata/config.xml` |
| Serial-206 config SHA-256 | `33aadf87f631cf33f2e0b4c86948c92be3b21412ca5477ea8fa8bc7848cbf475` |
| `ClassControlInterface.cs` SHA-256 | `86093e5270c82ea2e45cb4de449076372ca79d9485ba6de9565d5eb255811e6e` |
| `ClassBioXPSettings.cs` SHA-256 | `08155dc24602bc12cf25af745c74cc478f33e0f2675fc0d4b6f2e1ba917d8d41` |
| `locationID.cs` SHA-256 | `e99a88c5cf114526c2a08fb1f57fc6ab8860cdda17f838c3f13b48f868f59031` |
| `DefaultParameters.cs` SHA-256 | `04f53c129317f8ae508d6971d2cf9fb534e1bc8307b192f8db395cbbcdf64fd2` |
| `ClassMachineStatus.cs` SHA-256 | `c3f400f9563caf3497ae00202228eab19018466efde9ea45efc92c7bbfb9c32b` |

### 2.3 Implementation baselines

| Surface | Baseline |
|---|---|
| Robot worktree | `/home/dalab/worktrees/bioxp-strict-oem-parity-20260823` |
| Robot baseline commit | `4843712bdd2d98f6da83988213bfc7a0fd9885fa` |
| Robot baseline branch | `work/serial206-strict-oem-parity-20260823` |
| Robot status before this document | Clean |
| BMS reference worktree | `/home/dalab/worktrees/bms-bioxp-strict-oem-parity-20260823` |
| BMS reference commit | `1d41fb50300a4e574670f6a02a0345e597ac9ed4` |
| BMS reference branch | `fix/bioxp-strict-oem-parity-20260823` |

These are planning baselines. Implementation must re-freeze current bytes before each approved work package. A later commit or dirty change requires claim-by-claim reconciliation.

### 2.4 Source parity and Linux governance

This program separates physical OEM semantics from Linux control governance.

| Class | Ownership | Rule |
|---|---|---|
| source-equivalent movement | OEM source and Serial-206 configuration | Coordinates, branch predicates, movement order, caller-specific preparation, and semantic updates must match the selected source |
| Linux admission and durability | robot command plane | Generations, idempotency, SQLite lifecycle, replay fences, provider exclusion, and independent STOP/Abort can deny or defer execution |
| BMS and agent presentation | robot catalog and receipt contract | Consumers can select finite intent and display evidence; they cannot change source movement meaning |

Every compiled plan must identify which checks reproduce OEM behavior and which checks are Linux governance. A governance check can block execution. It cannot change coordinates, select another branch, reorder source movement, or publish a different semantic result while claiming source parity.

## 3. Scope

### 3.1 Included

This program includes:

- exact Serial-206 PositionTable loading, normalization, repair, provenance, and revision identity;
- finite named destination catalog for the OEM diagnostic location panel;
- exact `btnLOC1_Click` movement semantics;
- normal `ClassMoveTo`/`movExecution` destination, plate, material, and well translation;
- exact `scriptmoveTo` coordinate calculation and branch selection;
- exact primitive `moveTo` composite behavior consumed by location movement;
- `CurrentLocation`, `CurrentWell`, `CurrentTray`, tip, clean-path, plate, gantry-load, and pseudo-Z state;
- durable canonical command and method admission;
- one global source-ordered movement stream with independent STOP/Abort delivery;
- plan/snapshot identity and execution-time revalidation;
- controller completion, failure, ambiguity, and success-only state publication;
- restart recovery without automatic redispatch of an ambiguous physical command;
- plate, cover, press, waste, parking, and tip-aware movement wrappers required by the critical gap set;
- robot control catalog, BMS proxy/models/client, cockpit rendering, command-ID polling, and AI use of the same typed contract;
- no-hardware, controller, supervised physical, and UI acceptance gates.

### 3.2 Excluded

The following remain separate programs unless a direct dependency is named in a work package:

- redesign of raw axis controls;
- changes to OEM homing or startup semantics;
- controller access, physical motion, STOP, homing, or service restart during specification work;
- pipette fluid mechanics, pressure control, or chemistry behavior beyond movement-state publication;
- vision algorithms beyond source-defined barcode location offsets;
- thermal and chiller process control;
- generic deck-layout redesign;
- extended `locationID` values absent from the selected 29-row Serial-206 table;
- arbitrary coordinate, path, speed, acceleration, current, or raw-frame operator input;
- deployment, robot synchronization, or physical commissioning without later explicit authority.

### 3.3 Serial-206 denominator

The `locationID` enum has 47 values. The selected Serial-206 configuration defines 29 PositionTable rows. The 18 enum values absent from this machine configuration do not become movement targets through inference or compiled defaults.

Every request for an absent target must fail before command dispatch.

## 4. OEM source model

### 4.1 Typed identities

The OEM movement model uses:

- `locationID` for a semantic deck location;
- `wellID` for A1 through H12, stored row-major as `0..95`;
- `plateName` for movable plates, covers, strips, trough, and related objects;
- `positionStruct` with `name`, `x`, `y`, `zLow`, `zHigh`, `zDelta`, and `inc_factor`;
- `ClassMachineStatus` for current location, current well, current tray, tip state, plate state, and gantry load;
- `DefaultParameters.PSUDO_Z_HOME` for the dynamic clearance reference.

A display label or YAML slot is not a movement identity.

### 4.2 Well conversion

The exact well conversion is:

```text
row = wellID / 12
column = wellID % 12
```

Movement accepts `A1..H12` or canonical integer `0..95` at the semantic boundary. Unknown well `96` cannot become a movement target.

Grid increments are:

```text
xIncrement = -2132
yIncrement = +2132
```

### 4.3 PositionTable normalization

The resolved PositionTable must apply the source loader in this order:

1. parse `x`, `y`, `zLow`, `zHigh`, `zDelta`, `inc_factor`, and optional display name;
2. for TECAN entries, force `zDelta=53000` and derive `zHigh=zLow-zDelta`;
3. when `zDelta==0`, derive `zDelta=zLow-zHigh`;
4. otherwise, when `zHigh==0`, derive `zHigh=zLow-zDelta`;
5. if derived or supplied `zHigh<5000`, set `zHigh=0`;
6. replace or add only enum-recognized location rows;
7. apply source-defined coordinate repairs, including output-plate X alignment and absent press-point X/Y repair;
8. retain an ordered adjustment ledger with source anchors;
9. compute one resolved-table digest used by planning and receipts.

The current Linux loader omits step 5. This must be corrected before live named movement.

### 4.4 PositionTable ownership

The selected Serial-206 config and source transformations own movement coordinates.

These surfaces cannot own movement coordinates:

- `config/deck/default_layout.yaml`;
- `src/bioxp/domain/deck.py`;
- UI geometry;
- stale compiled/decompiled default coordinates in `config/oem/bioxp_oem_binding_initial.json`;
- caller-supplied X/Y/Z;
- a nearest-location heuristic.

The generic deck model may display metadata after binding to a canonical OEM target. It cannot create or modify a target.

## 5. Finite destination catalog

### 5.1 Public diagnostic destination keys

The source diagnostic panel contains 26 labels that resolve to 24 distinct configured destination IDs.

| Canonical request key | OEM panel label | `locationID` | Source behavior |
|---|---:|---:|---|
| `LOC_MS` | `LOC_MS` | 0 | ordinary direct location move |
| `LOC_OC` | `LOC_OC` | 1 | ordinary direct location move |
| `TECANRACK2` | `TIP TRAY 2` | 8 | ordinary direct location move |
| `TECANRACK4` | `TIP TRAY_4` | 10 | ordinary direct location move |
| `LOC_P_MS` | `LOC_MS_PLATE` | 25 | ordinary direct location move |
| `LOC_P_OC` | `LOC_OC_PLATE` | 21 | ordinary direct location move |
| `LOC_OC_COVER` | `LOC_OC_COVER` | 17 | ordinary direct location move |
| `LOC_TC` | `LOC_TC` | 2 | ordinary direct location move |
| `LOC_TC_BARCODE` | `LOC_TC_BARCODE` | 2 | source camera-offset branch |
| `LOC_RC` | `LOC_RC` | 3 | ordinary direct location move |
| `LOC_RC_BARCODE` | `LOC_RC_BARCODE` | 3 | source camera-offset branch |
| `TECANRACK1` | `TIP TRAY 1` | 7 | ordinary direct location move |
| `TECANRACK3` | `TIP TRAY_3` | 9 | ordinary direct location move |
| `LOC_P_TC` | `LOC_TC_PLATE` | 23 | ordinary direct location move |
| `LOC_BSC` | `LOC_TC_COVER` | 5 | ordinary direct location move |
| `LOC_RC_COVER` | `LOC_RC_COVER` | 19 | ordinary direct location move |
| `LOC_STRIP1` | `STRIP 1` | 11 | ordinary direct location move |
| `LOC_STRIP2` | `STRIP 2` | 12 | ordinary direct location move |
| `LOC_STRIP3` | `STRIP 3` | 13 | ordinary direct location move |
| `LOC_STRIP4` | `STRIP 4` | 14 | ordinary direct location move |
| `LOC_OC_COVER_STORAGE` | `LOC_OC_COVER_S` | 18 | ordinary direct location move |
| `LOC_RC_COVER_STORAGE` | `LOC_RC_COVER_S` | 20 | ordinary direct location move |
| `LOC_TROUGH` | `LOC_TROUGH1` | 16 | ordinary direct location move |
| `LOC_BSCS` | `LOC_BSCS` | 4 | ordinary direct location move |
| `WASTE_BIN` | `Waste Bin` | 6 | ordinary direct location move |
| `LOC_PARK` | `Park` | 28 | source `parkGantry` branch |

`LOC_TC_BARCODE` and `LOC_RC_BARCODE` share base destination IDs with `LOC_TC` and `LOC_RC`. They remain distinct source operations because they add camera offsets and a separate Z movement.

### 5.2 Human aliases

The robot catalog publishes finite human aliases. Aliases are metadata and must map to one canonical request key.

Required initial aliases:

| Alias | Canonical request key |
|---|---|
| `OC chiller` | `LOC_OC` |
| `Output Chiller` | `LOC_OC` |
| `Output Tray` | `LOC_OC` |
| `Reagent Chiller` | `LOC_RC` |
| `Thermal Cycler` | `LOC_TC` |
| `Magnetic Station` | `LOC_MS` |
| `Waste Bin` | `WASTE_BIN` |
| `Park` | `LOC_PARK` |

The robot accepts the canonical key. BMS, an AI agent, or another client can use catalog aliases for selection. No free-text resolver runs inside the movement provider.

### 5.3 Internal-only targets

These rows are internal operation points and cannot appear as ordinary named destination buttons:

- `LOC_P_OC_PRESS` (22);
- `LOC_P_TC_PRESS` (24);
- `LOC_P_MS_PRESS` (26);
- `LOC_P_RC_PRESS` (27);
- `CAMERA_OFFSET` (31).

Only source-shaped press, camera, or plate operation compilers can select them.

## 6. Required movement entry families

### 6.1 Diagnostic named movement

The exact ordinary diagnostic contract is:

```text
ForceToHighHome
→ evaluate both latch predicates
→ translate finite label/key to locationID
→ choose Park, barcode, or ordinary branch
→ execute source-shaped movement
→ updateLocation(destination, well 0) after successful return
```

Binding details:

- `ForceToHighHome()` sets pseudo-Z home to `500` before latch evaluation.
- Both `m_latchStatus` and `MachineStatus.LatchClosed` are evaluated.
- A failed latch gate blocks physical dispatch.
- The pseudo-home mutation has already occurred when the latch gate fails. The durable replacement must preserve this source ordering.
- Ordinary targets call `moveTo(locationID, offsetX, offsetY)`.
- Normal ordinary movement uses zero offsets unless the finite camera-offset option is explicitly selected.
- The caller cannot supply numeric offsets.
- Barcode targets use source-owned camera offsets and a separate source Z move.
- Park calls `parkGantry()`.
- `updateLocation` occurs only after the selected movement branch returns without failure.

### 6.2 Normal workflow movement

The normal script/job contract is:

```text
ClassMoveTo intent
→ movExecution
→ resolve old-well, explicit well, or material-selected well
→ resolve plateName or allowed direct location
→ get current plate location
→ translate LOC_P_TC→LOC_TC, LOC_P_OC→LOC_OC, LOC_P_MS→LOC_MS when applicable
→ scriptmoveTo(current location, current well, resolved destination, well)
→ updateLocation
→ updatePlateLocation
→ optional source-shaped pierce/wash/hokey-pokey/circle-punch operation
```

`movExecution` remains an internal robot compiler. BMS and agents cannot supply its resolved coordinate or path fields.

### 6.3 Plate, cover, and press movement

The program must cover the source wrappers that combine movement with:

- `catchPlate`;
- `releasePlate`;
- `pressPlates`;
- output and reagent cover placement/storage;
- thermal-cycler plate placement;
- gripper open/close/home;
- Z current changes and restoration;
- `LoadGantry` and `PlateOnGantry` updates;
- `updatePlateLocation` and `updateLocation`;
- door state where the source operation requires it.

These are finite robot-owned operations. They cannot be compiled from arbitrary client step lists.

## 7. `scriptmoveTo` normative contract

### 7.1 Target calculation

For a resolved destination:

```text
x = base.x + inc_factor × -2132 × column
```

For ordinary locations:

```text
y = base.y + inc_factor × 2132 × (row - 2 × effective TipLocation)
```

Tip offset is suppressed for IDs 6, 7, 8, 9, 10, and 16.

Z selection is:

| `positionflag` | Z target |
|---:|---|
| 0 | current pseudo-Z home |
| 1 | resolved `zHigh` |
| any other value | resolved `zLow` |

### 7.2 Required branch inputs

One coherent provider snapshot must contain:

- ownership generation;
- active board epochs for boards 4 and 5;
- resolved PositionTable digest;
- machine-state revision;
- current controller X, Y, and Z;
- current semantic location and well;
- X, Y, Z, and gripper reference states and versions;
- tip loaded;
- tip dirty;
- active tip location;
- clean-path state;
- plate on gantry;
- pseudo-Z home;
- device type;
- both latch predicates and their observation identity;
- STOP/Abort safety epochs;
- provider ownership identity.

Unknown, stale, malformed, contradictory, or generation-mismatched required fields block planning or dispatch.

### 7.3 Branch families

The compiler must preserve these source families:

1. same X/Y, move Z only;
2. gripper confirmed with no tip, direct composite `moveTo`;
3. tip loaded, including pseudo-home lift;
4. tip-rack or tip-hotel departure;
5. midpoint path with clean-path branch;
6. waste-bin midpoint and waste sequence;
7. near-axis parallel or sequential movement;
8. TECAN-rack departure staging;
9. ordinary tip-loaded fallback;
10. no-tip clean fallback;
11. dirty-tip waste sequence;
12. dirty-tip ordinary fallback.

Every compiled stage records its source anchor, source operation, axis resources, expected evidence, and dependency order.

### 7.4 Known source hazard

The decompiled expressions at the cover-storage checks use:

```text
destination != 20 || destination != 18
```

This expression is always true. Before implementation changes its behavior, raw IL or binary behavior must disposition the hazard. Until then, the replacement must retain the decompiled behavior and label the hazard in the plan receipt.

## 8. Primitive `moveTo` contract

The location compiler ultimately reaches the provider-owned composite `moveTo(x,y,z,runInParallel)` implementation.

The implementation must preserve:

- the all-zero home branch;
- current-Z versus pseudo-home ordering;
- source acceleration decisions for long X/Y travel;
- gripper-reference and tip-loaded predicates;
- direction-dependent sequencing;
- plate-on-gantry cover clearance through ordinal 19 / `LOC_RC_COVER` Y;
- parallel and delayed-axis behavior;
- target-Y-zero homing behavior where reached from source callers;
- Z descent only under the source predicate;
- provider controller completion for each child movement.

The public named-location request never exposes raw X/Y/Z, `runInParallel`, speed, acceleration, current, timeout, or internal branch predicates.

## 9. Machine-state authority

### 9.1 One canonical state

SQLite-backed robot state is authoritative for semantic machine state. Controller reads are authoritative for current axis coordinates and controller completion. Reference state comes from the reference authority. Pipette state comes from the pipette owner after source-equivalent query or successful source operation.

No filesystem JSON file owns active machine or command state.

Typed SQLite columns must own fields used for admission and recovery. Canonical JSON stored inside SQLite can serve as bounded serialization or evidence. It cannot be the only authority for a critical predicate.

### 9.2 Required semantic fields

The durable machine state must include:

- `current_location_id`;
- `current_well_id`;
- `current_tray`;
- `tip_loaded`;
- `tip_dirty`;
- `tip_location`;
- `clean_path`;
- `plate_on_gantry`;
- movable plate and cover locations;
- pseudo-Z home;
- state revision;
- source operation and command ID for each transition;
- ownership and board epochs under which each value was produced.

### 9.3 State producers

Only these producer classes may update the related fields:

| Field | Permitted producer |
|---|---|
| current location/well/tray | successful semantic movement transition |
| tip loaded/location/dirty | pipette owner or successful source tip operation |
| clean path | source tip-tray availability calculation |
| plate on gantry | successful catch/release/gantry-load transition |
| movable plate/cover locations | successful source plate/cover operation |
| pseudo-Z home | `ForceToHighHome`, `GantryLoad`, `LoadGantry`, or exact source reset |

Nearest-position inference cannot write authoritative semantic location. It can remain a diagnostic suggestion labeled `inferred_nearest_location`, with no admission role.

### 9.4 Success-only transitions

For movement-owned state:

1. reserve the intended transition with the command;
2. execute provider child stages;
3. classify controller completion;
4. commit command terminal state and semantic state transition in one SQLite writer transaction;
5. publish the new state revision;
6. retain `physical_effect_verified=false` until separate observation exists.

Request acceptance, HTTP success from an outer relay, or planned target coordinates cannot update current location.

### 9.5 Ambiguous outcomes

When physical I/O may have occurred and terminal controller evidence or the final SQLite commit is missing:

- command state becomes `ambiguous` or `recovery_required`;
- semantic current location retains its last confirmed value, while a separate blocking ambiguity record marks current physical location untrusted;
- automatic redispatch is forbidden;
- later movement remains blocked until a source-compatible recovery decision is recorded;
- STOP and aggregate Abort remain available.

## 10. Coherent plan and execution authority

### 10.1 Snapshot identity

The provider emits `bioxp.oem_deck_authority.v1` with these minimum identity fields:

```text
ownership_generation
provider_owner_id
board_epoch_4
board_epoch_5
position_table_sha256
machine_state_revision
reference_versions{x,y,z,g}
safety_epochs{global,x,y,z}
latch_observation_id
controller_position_observation_id
captured_at
```

The complete canonical snapshot receives a digest.

### 10.2 Planning transaction

The global movement worker must:

1. claim the durable method;
2. obtain the provider movement lease;
3. read the complete authority snapshot;
4. validate the finite semantic intent;
5. compile source stages;
6. persist snapshot digest, plan digest, source branch, and child rows;
7. re-read or revalidate every dispatch-sensitive version immediately before the first physical write;
8. execute under the same movement owner;
9. release only after terminalization or ambiguity publication.

STOP and Abort use their independent lane and can preempt the normal stream.

### 10.3 No caller state

Live movement ignores caller-supplied values for:

- current X/Y/Z;
- current location/well;
- tip state/location;
- clean path;
- plate on gantry;
- pseudo-Z home;
- latch state;
- reference state;
- board epochs other than exact expected-generation fences;
- source speed, acceleration, path, offsets, or waypoints.

Such fields are forbidden by strict request models rather than silently ignored.

## 11. Canonical command-plane contract

### 11.1 Public action

Add one finite normal action:

```text
action_id: oem.deck.move_to_location
request schema: bioxp.operator_action_request.v2
```

Allowed inputs:

```json
{
  "target": "LOC_OC",
  "camera_offset": false
}
```

Rules:

- `target` must be a key published by the robot catalog.
- `camera_offset` applies only to source-supported ordinary diagnostic targets.
- Barcode keys compile their source camera branch and reject `camera_offset=true` with `contradictory_input`.
- Park compiles `parkGantry`.
- No additional fields are accepted.

The V2 envelope retains:

- idempotency key;
- expected ownership generation;
- expected board epochs for boards 4 and 5.

### 11.2 Internal source operations

The robot command plane also requires finite internal method identities:

- `oem.deck.script_move_to`;
- `oem.deck.mov_execution`;
- `oem.deck.park_gantry`;
- `oem.deck.catch_plate`;
- `oem.deck.release_plate`;
- `oem.deck.press_plate`.

These are not arbitrary public method builders. Only the robot’s source-shaped workflow or finite operator catalog can submit them.

### 11.3 Command lifecycle

The existing durable lifecycle remains controlling:

```text
queued
→ dispatched
→ issued_pending
→ completed | failed | ambiguous | stopped | aborted | cancelled | cleared | interrupted
```

A location method also owns ordered child commands. Parent completion requires every required child to reach a compatible terminal state.

The HTTP submission path returns after the SQLite admission transaction commits and supplies the durable command ID. It does not wait for planning or physical completion. Planning, dispatch, evidence capture, and terminalization run in the robot worker.

### 11.4 Global ordering

Location movement uses the single global OEM movement stream. Per-axis queues cannot reorder a source sequence such as:

```text
Z clearance → X/Y travel → Z descent
```

The command plane can index X/Y/Z/G resources for conflict reporting. Ordering comes from the global sequence and method dependencies.

### 11.5 Idempotency

The command plane reserves the idempotency key before planning or provider I/O.

- same key, same canonical intent, same current ownership generation: return or join the existing command;
- same key with different intent: conflict;
- key from an old ownership generation: reject stale replay;
- terminal result replay: no controller contact;
- process restart with issued work: publish recovery/ambiguity; do not issue the movement again.

### 11.6 Persistence

Extend the current SQLite command plane rather than creating a second store.

Required durable records:

- outer method/action identity;
- canonical semantic input;
- destination catalog revision;
- PositionTable digest;
- authority snapshot digest;
- compiled plan digest;
- source branch and source anchors;
- ordered child commands and dependencies;
- provider command IDs and terminal evidence;
- controller completion classification;
- state transition revision;
- operator assessment and independent observation linkage;
- restart/recovery state.

No movement run data may depend on a filesystem JSON receipt.

### 11.7 Admission latency and capacity

Admission must keep its SQLite transaction compact. It stores canonical intent, order, generations, idempotency identity, timestamps, and initial lifecycle state. Large provider evidence and verbose diagnostics stay outside the enqueue latency path.

The release record must report admission p50, p95, and maximum latency, queue-capacity behavior, and rapid sequential submission behavior against the accepted baseline. The implementation cannot claim a speed improvement without those measurements.

## 12. Completion and evidence contract

### 12.1 Evidence layers

Every receipt keeps these facts separate:

- `admitted`;
- `delivery_attempted`;
- `controller_command_acknowledged`;
- `controller_completion_verified`;
- `hardware_postcondition_verified`;
- `semantic_state_committed`;
- `physical_effect_verified`.

The first six can be established by the robot runtime where evidence exists. `physical_effect_verified` remains false until an independent operator, camera, or external sensor observation is linked.

### 12.2 Required child evidence

Each axis movement child records:

- before and after controller position;
- speed observations;
- target-reached evidence when applicable;
- raw and interpreted switch state when relevant;
- source operation parameters;
- board and motor identity;
- board epoch and ownership generation;
- async target reached event 128;
- stall/error events 130, 14, and 13 when seen;
- STOP/Abort interruption lineage;
- terminal classification.

Controller values are controller evidence. They do not establish physical movement by themselves.

### 12.3 Failure classification

| Condition | Required outcome |
|---|---|
| invalid target or input | reject before queue or provider I/O |
| absent PositionTable row | reject before planning |
| stale ownership/board epoch | reject or clear before dispatch |
| unknown required state | block before planning |
| latch predicate unsafe | block before physical write |
| reference predicate unsafe | block before physical write |
| authority changed before first TX | return to blocked/recovery state; do not dispatch old plan |
| child provider failure before later children | fail fast; later normal stages do not run |
| STOP/Abort during method | terminalize affected children and parent with lineage |
| controller outcome unknown after possible I/O | `ambiguous`; no automatic replay |
| semantic SQLite commit failure after controller completion | `ambiguous/recovery_required`; do not issue movement again |
| outer BMS parse failure after robot dispatch | reconcile by command ID; do not blind retry |

## 13. Robot API and catalog

### 13.1 Required robot surfaces

Extend the existing V2 operator plane with:

- catalog row for `oem.deck.move_to_location`;
- V2 action admission and enqueue through the existing route;
- command receipt lookup by command ID;
- method/child status projection;
- read-only finite destination catalog projection;
- read-only resolved PositionTable identity and destination metadata;
- recovery/ambiguity projection;
- existing independent STOP and aggregate Abort.

The robot should not add a generic arbitrary-path proxy.

### 13.2 Catalog row

The catalog row must publish:

- action ID and schema versions;
- finite destination options;
- human label and aliases;
- source method and source anchors;
- required board epochs;
- required references;
- current enabled state;
- deterministic disabled reason;
- PositionTable and catalog revisions;
- no raw coordinate inputs.

### 13.3 Legacy route disposition

The current direct live `scriptmove_execute` compatibility route is retired from live use and retained as preview-only. Preview returns a source plan with no provider I/O and no semantic state mutation.

The raw `/motion/oem/move_to` HTTP route is also removed from live movement authority. It can remain as preview-only diagnostics with zero provider I/O. The internal provider primitive remains available only to canonical robot-owned method execution.

## 14. BMS and agent contract

### 14.1 BMS role

BMS remains a thin generation-bound relay and typed operator surface.

BMS may:

- validate the robot response schema;
- bind the current connection generation;
- forward the exact canonical target key;
- poll receipts by command ID;
- render robot-owned admission and evidence.

BMS may not:

- store or calculate movement coordinates;
- choose waypoints;
- infer current machine state;
- relax robot disabled reasons;
- issue raw route/path/coordinate requests;
- treat HTTP success as physical success.

### 14.2 BMS API files

Expected BMS modifications:

- `platform/api/services/bioxp/operator_models.py`;
- `platform/api/services/bioxp/robot_client.py`;
- `platform/api/routers/bioxp/operator_controls.py`;
- `platform/api/tests/test_serial206_bioxp_v2_models.py`;
- `platform/api/tests/test_bioxp_operator_controls.py`;
- `platform/api/tests/test_bioxp_robot_client.py`.

### 14.3 Frontend files

Expected frontend modifications:

- `platform/frontend/src/lib/bioxpClient.ts`;
- `platform/frontend/src/components/BioXpCockpit.tsx`;
- `platform/frontend/tests/vitest/bioxpOperatorGenerationPayload.test.ts`;
- `platform/frontend/tests/vitest/bioxpOperatorCriticalControlsMounted.test.tsx`;
- a new mounted test dedicated to deck/location movement.

### 14.4 Cockpit behavior

Add one **OEM Deck Movement** panel with:

- a finite robot-provided destination selector;
- canonical target key and operator label;
- current semantic location and well;
- PositionTable/catalog revision identity;
- exact disabled reason;
- one submission button;
- current command ID and lifecycle;
- source branch selected by the robot;
- controller completion and semantic-state status;
- a separate physical-observation status.

The UI has no raw X/Y/Z, waypoint, speed, acceleration, current, offset, or branch controls.

### 14.5 AI contract

An AI agent uses the same destination catalog and action request as the cockpit.

For the user statement `move to OC chiller`, the agent resolves the catalog alias to `LOC_OC`, shows or states the canonical target, and submits the typed action only after the robot catalog says it is enabled and the applicable user authority exists.

The AI cannot convert natural language directly into coordinates.

## 15. Security and physical-safety boundary

This specification preserves the existing operator trust model. It does not add credentials or confirmation prose.

Machine safety remains enforced through:

- finite action IDs;
- strict request schemas;
- target/catalog membership;
- ownership and board generations;
- provider exclusion;
- latch and reference predicates;
- state authority;
- idempotency;
- durable command lifecycle;
- independent STOP and Abort;
- controller and physical evidence separation.

A replacement safety intercept may inhibit an OEM action. It cannot alter source coordinates, ordering, branch meaning, or state semantics while claiming parity.

## 16. Data and migration contract

### 16.1 SQLite changes

Use additive migrations in `src/bioxp/oem_runtime_store.py` and the existing `OperatorCommandStore` schema path.

Schema additions must support:

- finite catalog revision;
- semantic target key and resolved location ID;
- authority and plan digests;
- source branch;
- machine-state before/after revisions;
- child stage identity and dependency order;
- transition and ambiguity records;
- operator observation linkage.

Critical lifecycle and recovery fields require typed columns. A bounded canonical JSON evidence column may retain complete source/provider payloads.

### 16.2 Migration requirements

Future authorized migration verification must cover:

- fresh database;
- current baseline database shape;
- repeated migration application;
- interrupted migration rollback;
- retained command/method rows;
- foreign-key integrity;
- trigger and index attestation;
- restart recovery of queued and issued location methods.

No production database may be opened or migrated during specification work.

## 17. Exact production files

### 17.1 Robot files to modify

- `src/bioxp/oem_machine_bundle.py`
- `src/bioxp/oem_compat/position_table.py`
- `src/bioxp/oem_compat/pathing.py`
- `src/bioxp/oem_compat/machine_state.py`
- `src/bioxp/oem_homing_routes.py`
- `src/bioxp/oem_serial206_initialization.py`
- `src/bioxp/oem_runtime_store.py`
- `src/bioxp/operator_command_plane.py`
- `src/bioxp/operator_controls.py`
- `src/bioxp/api.py`

### 17.2 Robot files to create

- `src/bioxp/oem_deck_catalog.py`
- `src/bioxp/oem_deck_movement.py`

`oem_deck_catalog.py` owns the finite source catalog and aliases. `oem_deck_movement.py` owns source-shaped semantic intents, compilation, authority validation, and success-state transitions. It does not own transport.

### 17.3 Robot tests to modify or create during authorized implementation

Existing:

- `tests/test_oem_pathing_planner.py`
- `tests/test_oem_pathing_routes.py`
- `tests/test_oem_pathing_executor.py`
- `tests/test_oem_machine_state_pathing.py`
- `tests/test_strict_oem_parity_regressions.py`
- `tests/test_serial206_y_command_plane.py`
- `tests/test_serial206_v2_migration.py`

New:

- `tests/test_oem_position_table_serial206_parity.py`
- `tests/test_oem_deck_catalog.py`
- `tests/test_oem_named_location_command_plane.py`
- `tests/test_oem_deck_command_queue.py`
- `tests/test_oem_deck_authority_atomicity.py`
- `tests/test_oem_mov_execution.py`
- `tests/test_oem_plate_cover_press_movement.py`
- `tests/test_oem_deck_restart_recovery.py`
- `tests/test_oem_deck_persistence_integrity.py`

## 18. Phased implementation plan

Each work package requires separate approval for its code edits and tests. Deployment and hardware work remain separate approvals.

### WP0: Freeze authority and implementation denominator

**Objective:** Bind implementation to the exact OEM corpus, selected Serial-206 config, current robot tree, and this specification.

**Files:**

- modify `docs/specs/2026-08-29-oem-deck-location-movement-e2e-parity-spec.md` only if the operator approves corrections;
- modify `docs/specs/2026-07-23-oem-movement-method-source-binary-registry.md`;
- modify `docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json`.

**Tasks:**

1. Recompute all evidence and source hashes.
2. Verify each cited source file maps to a captured binary in the evidence lock.
3. Add exact method ranges for `btnLOC1_Click`, both location `moveTo` overloads, both `scriptmoveTo` overloads, primitive `moveTo`, `getMidPoint`, `movExecution`, `parkGantry`, catch/release/press helpers, and state transitions.
4. Add the finite destination denominator and known-hazard ledger.
5. Resolve or formally defer the cover-storage Boolean expression through raw IL.
6. Freeze the current robot target tree and relevant external evidence hashes.

**Exit gate:** Exact-source registry and this specification have a current-hash review. No runtime behavior claim is made.

### WP1: Correct resolved PositionTable semantics

**Objective:** Produce one source-normalized, provenance-bearing PositionTable snapshot.

**Files:**

- modify `src/bioxp/oem_machine_bundle.py`;
- modify `src/bioxp/oem_compat/position_table.py`;
- create `tests/test_oem_position_table_serial206_parity.py`.

**Tasks:**

1. Add OEM TECAN derivation and `zHigh<5000→0` normalization.
2. Port source coordinate repairs in exact order.
3. Reject enum-unknown config tags and duplicate canonical rows.
4. Emit normalized rows plus an adjustment ledger.
5. Compute resolved PositionTable digest from canonical rows and source config identity.
6. Remove live planner reliance on stale initial-binding coordinates.
7. Prove all 29 Serial-206 rows match an independently generated source oracle.
8. Prove the 14 materially changed Z-high values become OEM zero.

**Exit gate:** 29-row normalized table, exact digest, adjustment ledger, no generic-deck authority.

### WP2: Build the finite source destination catalog

**Objective:** Give every public semantic destination one canonical robot-owned identity.

**Files:**

- create `src/bioxp/oem_deck_catalog.py`;
- modify `src/bioxp/operator_controls.py`;
- create `tests/test_oem_deck_catalog.py`.

**Tasks:**

1. Encode the 26 diagnostic labels and 24 distinct target IDs.
2. Separate ordinary, barcode, and Park branch kinds.
3. Encode approved human aliases as metadata.
4. Keep four press points and `CAMERA_OFFSET` internal.
5. Bind every catalog row to the resolved PositionTable digest.
6. Reject absent Serial-206 enum targets.
7. Publish a typed read-only catalog through the existing operator catalog.

**Exit gate:** Complete finite catalog with no raw coordinates or arbitrary aliases.

### WP3: Repair the planner and semantic translators

**Objective:** Make the central planner match source formulas and remove the current unsatisfiable route.

**Files:**

- modify `src/bioxp/oem_compat/pathing.py`;
- modify `src/bioxp/oem_homing_routes.py`;
- modify `src/bioxp/oem_compat/position_table.py`;
- modify existing pathing tests.

**Tasks:**

1. Replace literal `LOC19` with ordinal 19 resolved as `LOC_RC_COVER`.
2. Add strict `wellID` A1-H12 conversion and range validation.
3. Connect source `plateName` and destination translations through explicit semantic compilers.
4. Keep `scriptmoveTo` live inputs provider-owned.
5. Bind plan output to the PositionTable and authority digest.
6. Expand planner fixtures to the exact 29-row Serial-206 table.
7. Cover every source branch family and the source hazard disposition.

**Exit gate:** Every branch compiles against exact machine rows, and live planning has no impossible key lookup.

### WP4: Close machine-state producers

**Objective:** Make every branch-changing field authoritative and durable.

**Files:**

- modify `src/bioxp/oem_compat/machine_state.py`;
- modify `src/bioxp/oem_serial206_initialization.py`;
- modify `src/bioxp/oem_runtime_store.py`;
- create or update machine-state tests.

**Tasks:**

1. Add typed SQLite fields and revisions for semantic state.
2. Retire nearest-position inference as an authority writer.
3. Wire pipette owner publication for tip loaded/location/dirty state.
4. Wire source clean-path calculation.
5. Wire `LoadGantry`, plate-on-gantry, and movable plate/cover state.
6. Persist pseudo-home transitions from source operations.
7. Add transition provenance and command linkage.
8. Reject loaded-tip state without valid tip location.
9. Reject plate-on-gantry values outside the source enum/domain.

**Exit gate:** `path_planning_authority()` can only return `ok=true` from complete current-generation state.

### WP5: Add coherent plan/execution authority

**Objective:** Prevent state drift between planning and the first physical write.

**Files:**

- create `src/bioxp/oem_deck_movement.py`;
- modify `src/bioxp/oem_serial206_initialization.py`;
- modify `src/bioxp/operator_command_plane.py`;
- create `tests/test_oem_deck_authority_atomicity.py`.

**Tasks:**

1. Define `bioxp.oem_deck_authority.v1`.
2. Compute canonical authority and plan digests.
3. Add one provider movement lease shared by plan and execution.
4. Revalidate ownership, board, reference, safety, latch, PositionTable, and machine-state versions before first TX.
5. Persist plan and child commands before first TX.
6. Keep STOP and Abort independent of the normal lease.
7. Inject generation/state changes at every planning boundary and prove zero physical dispatch.

**Exit gate:** No live plan can execute under a different authority snapshot.

### WP6: Implement the canonical diagnostic named move

**Objective:** Make `move to OC chiller` and every finite diagnostic destination execute through one durable source-shaped command.

**Files:**

- modify `src/bioxp/oem_deck_movement.py`;
- modify `src/bioxp/operator_command_plane.py`;
- modify `src/bioxp/operator_controls.py`;
- modify `src/bioxp/api.py` only for router/model registration that cannot live in the owning modules;
- create `tests/test_oem_named_location_command_plane.py`.

**Tasks:**

1. Add `oem.deck.move_to_location` to canonical allowed actions.
2. Add strict target and camera-offset input validation.
3. Require exact board epochs for boards 4 and 5.
4. Compile `ForceToHighHome` before both latch predicates.
5. Compile ordinary, barcode, and Park branches.
6. Execute through existing provider primitives.
7. Commit current location/well only after controller completion.
8. Persist source branch, target identity, plan digest, and evidence.
9. Make old `scriptmove_execute` and raw `/motion/oem/move_to` HTTP execution preview-only with zero provider I/O.
10. Prove no raw `/motion/oem/move_to` bypass exists.
11. Prove concurrent idempotency, conflicting-key rejection, rapid sequential enqueue, global FIFO order, and queue-capacity rejection without provider I/O in the HTTP handler.

**Exit gate:** All 26 labels compile; all 24 distinct destinations have canonical queued commands; no hardware is required for this software gate.

### WP7: Implement normal `movExecution` and `scriptmoveTo` semantics

**Objective:** Close normal workflow movement used for pipetting, material transfer, and well targeting.

**Files:**

- modify `src/bioxp/oem_deck_movement.py`;
- modify `src/bioxp/oem_compat/pathing.py`;
- modify `src/bioxp/operator_command_plane.py`;
- create `tests/test_oem_mov_execution.py`.

**Tasks:**

1. Define typed `ClassMoveTo`-equivalent intent.
2. Resolve old-well, explicit well, and material-selected well branches.
3. Resolve `plateName` through current plate state.
4. Apply station translations for pool, output, and magnetic plates.
5. Compile the exact `scriptmoveTo` branch.
6. Persist child stage ordering.
7. Commit location and plate state after successful movement.
8. Add finite optional pierce/wash operation continuations with source ordering.
9. Reject unexpected direct plate/location combinations exactly.

**Exit gate:** Workflow movement no longer depends on caller raw rows, coordinates, or stale metadata aliases.

### WP8: Close plate, cover, press, Park, and waste operation families

**Objective:** Complete the major movement wrappers that make deck motion useful to robot workflows.

**Files:**

- modify `src/bioxp/oem_deck_movement.py`;
- modify `src/bioxp/operator_command_plane.py`;
- modify `src/bioxp/oem_serial206_initialization.py` as needed for provider primitives;
- create `tests/test_oem_plate_cover_press_movement.py`.

**Tasks:**

1. Implement finite catch/release operations.
2. Implement output/reagent/biosecurity cover movement and storage.
3. Implement press-point operations using internal-only targets.
4. Preserve gripper/Z current and restoration sequences.
5. Preserve door and park prerequisites where source-owned.
6. Apply `LoadGantry` and pseudo-home transitions in source order.
7. Persist plate location only after terminal success.
8. Fail closed on partial gripper/Z/plate movement with exact residual state.

**Exit gate:** All critical plate and cover movement families have source-shaped durable methods.

### WP9: Restart, interruption, and ambiguity closure

**Objective:** Make location methods truthful across STOP, Abort, timeout, persistence failure, and restart.

**Files:**

- modify `src/bioxp/operator_command_plane.py`;
- modify `src/bioxp/oem_runtime_store.py`;
- create `tests/test_oem_deck_restart_recovery.py`.
- create `tests/test_oem_deck_persistence_integrity.py`.

**Tasks:**

1. Preserve global FIFO and child dependencies.
2. Test STOP at each child boundary.
3. Test aggregate Abort while normal execution holds the movement lease.
4. Fence stale target-reached events after interruption.
5. Reconcile queued, dispatched, and issued-pending rows after restart.
6. Prevent automatic physical redispatch after ambiguity.
7. Link recovery decisions to command and plan identity.
8. Keep semantic state at last confirmed revision when terminal state is uncertain.
9. Add direct-SQL rejection tests for command, plan, generation, child-dependency, and terminal-evidence rebinding.
10. Attest exact columns, constraints, foreign keys, ordered indexes, triggers, schema fingerprint, and migration ledger.

**Exit gate:** Every restart/interruption state has deterministic durable behavior.

### WP10: BMS relay, typed models, and cockpit

**Objective:** Expose the canonical robot action without duplicating movement authority.

**Robot prerequisite:** WP6 software gate passed.

**Files:**

- modify `platform/api/services/bioxp/operator_models.py`;
- modify `platform/api/services/bioxp/robot_client.py`;
- modify `platform/api/routers/bioxp/operator_controls.py`;
- modify `platform/frontend/src/lib/bioxpClient.ts`;
- modify `platform/frontend/src/components/BioXpCockpit.tsx`;
- modify or add the tests named in sections 14 and 17.

**Tasks:**

1. Add strict BMS models for destination options and action receipts.
2. Forward the V2 canonical action with connection generation.
3. Preserve nested robot 409/422/503 evidence.
4. Add command-ID polling and no-blind-retry reconciliation.
5. Add the finite deck selector and one action button.
6. Render robot-owned disabled reason and revisions.
7. Display controller, semantic-state, and physical-observation truth separately.
8. Prove one click produces one robot enqueue.
9. Prove stale catalog/dashboard state disables normal movement.
10. Expose the same typed action for agents.

**Exit gate:** BMS and agents use one robot-owned contract and cannot supply coordinates.

### WP11: No-hardware verification and exact candidate review

**Objective:** Prove source, schema, queue, state, migration, and UI behavior without controller access.

**Permission required:** Test execution requires explicit approval.

**Planned robot commands:**

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider \
  tests/test_oem_position_table_serial206_parity.py \
  tests/test_oem_deck_catalog.py \
  tests/test_oem_pathing_planner.py \
  tests/test_oem_pathing_routes.py \
  tests/test_oem_pathing_executor.py \
  tests/test_oem_machine_state_pathing.py \
  tests/test_oem_named_location_command_plane.py \
  tests/test_oem_deck_command_queue.py \
  tests/test_oem_deck_authority_atomicity.py \
  tests/test_oem_mov_execution.py \
  tests/test_oem_plate_cover_press_movement.py \
  tests/test_oem_deck_restart_recovery.py \
  tests/test_oem_deck_persistence_integrity.py
```

**Planned BMS API commands:**

```bash
uv run --frozen --group dev python -m pytest \
  tests/test_serial206_bioxp_v2_models.py \
  tests/test_bioxp_operator_controls.py \
  tests/test_bioxp_robot_client.py
```

**Planned frontend commands:**

```bash
pnpm run test
pnpm run build
```

**Required review:**

- exact robot tree review;
- exact BMS tree review;
- evidence-lock/registry check;
- route and public-bypass census;
- SQLite schema/trigger/index attestation;
- source-branch denominator review;
- final `git diff --check` and candidate hash freeze.

**Exit gate:** Current exact bytes pass the authorized no-hardware gates and independent review. This does not establish controller or physical acceptance.

### WP12: Managed Development integration

**Objective:** Put the exact accepted robot and BMS bytes into their managed Development environments.

**Permission required:** Commit, push, deployment, and service restart require separate explicit authority.

**Required evidence:**

- exact accepted source SHAs;
- remote branch identities;
- managed process owner, CWD, listener, and runtime database;
- robot and BMS OpenAPI/catalog identity;
- live producer payload accepted by BMS Pydantic models;
- cockpit reachable through Christian’s real entry path;
- zero physical command during deployment proof.

**Exit gate:** Source, process, API, catalog, database, and UI identities agree.

### WP13: Controller-only commissioning

**Objective:** Prove the command reaches the correct boards/motors and produces source-compatible terminal controller evidence.

**Permission required:** Every controller or motion action requires explicit test-ID authority. Deployment authority does not authorize this phase.

**Order:**

1. query-only authority snapshot;
2. stop/abort availability;
3. no-motion command admission and plan inspection;
4. one bounded target selected by Christian;
5. controller command and event evidence;
6. failure-path and interruption checks;
7. later destination groups only after prior PASS.

**Exit gate:** Controller evidence is complete for each approved row. `physical_effect_verified` remains false without independent observation.

### WP14: Supervised physical and UI acceptance

**Objective:** Accept real named movement through the operator path.

**Permission required:** Each physical row requires an explicit single-use approval and an on-machine observer.

**First proposed row:** `LOC_OC` only after Christian selects it and confirms clearance.

For each row record:

- exact deployed robot and BMS SHAs;
- command ID, ownership generation, board epochs, PositionTable digest, and plan digest;
- pre-state and post-state;
- controller events and terminal evidence;
- observed physical destination;
- UI receipt and disabled/enabled behavior;
- independent PASS/FAIL observation;
- state publication after success;
- STOP/Abort readiness.

**Exit gate:** All required destination and operation-family rows pass. Automation is not Christian’s final scientific/robot acceptance unless he inspects it through the live operator path.

## 19. Acceptance matrix

### 19.1 Software and contract gates

| ID | Requirement | PASS condition |
|---|---|---|
| S01 | Exact authority | Current hashes and source-to-binary maps pass |
| S02 | PositionTable | All 29 rows match source-normalized Serial-206 oracle |
| S03 | Destination catalog | 26 labels, 24 distinct targets, finite aliases, internal points excluded |
| S04 | `LOC19` repair | Ordinal 19 resolves only as `LOC_RC_COVER` |
| S05 | Well semantics | A1-H12 and 0-95 round-trip; 96 rejected |
| S06 | Planner branches | Every source branch family has positive and negative coverage |
| S07 | Machine state | Every branch predicate has a typed authoritative producer |
| S08 | Plan binding | Any authority drift before first TX yields zero dispatch |
| S09 | Canonical queue | Named move uses SQLite global movement stream |
| S10 | No bypass | Legacy live path and raw move route cannot bypass canonical admission |
| S11 | State transitions | Location/plate state changes only after controller completion |
| S12 | Restart | Issued ambiguity never auto-replays physical movement |
| S13 | BMS proxy | Strict models preserve robot truth and command ID |
| S14 | UI | Finite selector, one submit, receipt polling, no coordinate controls |
| S15 | Agent parity | Agent uses the same catalog/action schema as UI |
| S16 | Admission performance | Durable enqueue returns before physical execution; p50/p95/max and queue-capacity evidence are recorded |
| S17 | SQLite integrity | Direct SQL cannot rebind immutable command, plan, generation, or terminal evidence fields |

### 19.2 Controller and physical gates

| ID | Requirement | PASS condition |
|---|---|---|
| C01 | Stop/Abort | Independent delivery and terminal evidence available before normal movement |
| C02 | Board ownership | Boards 4 and 5 match expected epochs and owner |
| C03 | References | X/Y/Z and required gripper state are authoritative |
| C04 | Latches | Both source predicates are fresh and safe |
| C05 | OC controller path | `LOC_OC` command stages and terminal events match the accepted plan |
| P01 | OC physical path | Observer confirms expected physical destination and no unexpected movement |
| P02 | State publication | Current location/well update follows the accepted physical/controller result |
| P03 | Remaining destinations | Every approved catalog row passes its required source branch |
| P04 | Plate/cover operations | Catch/release/press state and physical results agree |
| P05 | Live UI path | Christian reaches and inspects the accepted control through the actual cockpit entry path |

### 19.3 Strict completion rule

This program is complete only when:

- every applicable S, C, and P row is PASS on the exact deployed revisions;
- all 24 distinct configured destination IDs are execution-closed;
- barcode and Park branches are accepted separately;
- plate, cover, press, waste, and tip-aware movement families have their required accepted rows;
- no public live route bypasses the canonical authority;
- state, controller evidence, physical observation, BMS receipt, and UI agree;
- no severity-1 or severity-2 parity defect remains;
- Christian completes the required live inspection.

## 20. Physical destination campaign template

This document does not authorize the campaign.

Each destination campaign row must contain:

```text
test_id
target_key
source_branch
exact deployed robot SHA
exact deployed BMS SHA
ownership generation
board epochs 4 and 5
position table digest
catalog revision
machine-state before revision
expected source stages
maximum source-defined movement envelope
STOP action
Abort action
observer
pre-state evidence
controller terminal evidence
post-state evidence
physical observation
machine-state after revision
verdict
```

A failure or unexpected observation stops later rows until Christian approves the disposition.

## 21. Rollback boundaries

Each implementation work package has its own commit and rollback boundary.

- WP1-WP3 can roll back without changing durable command tables if no live migration is deployed.
- WP4-WP5 share the machine-state/authority schema boundary and must roll back with their migration compatibility preserved.
- WP6-WP9 share the canonical command-plane boundary. Rollback must disable the named action before changing persisted semantics.
- WP10 BMS rollback must leave the robot action available only through accepted robot clients; the cockpit control must disappear or fail closed.
- Deployment rollback cannot reinterpret commands issued by a newer schema. Existing receipts remain readable.
- Physical acceptance artifacts remain immutable historical evidence after rollback.

## 22. Stop conditions

Implementation or acceptance stops immediately when:

- current source/evidence hashes differ from the frozen work-package baseline;
- an OEM source dependency is absent or contradictory;
- the exact source branch cannot be resolved;
- a test requires invented coordinates or state;
- a plan can dispatch after authority drift;
- STOP or Abort is unavailable for an approved physical row;
- controller evidence is ambiguous;
- physical observation disagrees with controller or semantic state;
- a later child executes after a fail-fast failure;
- BMS loses the command ID or nested robot reason;
- a public compatibility route bypasses canonical admission;
- database migration or recovery cannot preserve truthful command state.

## 23. Deliverables

The completed program produces:

1. source and machine registry updates;
2. normalized 29-row Serial-206 PositionTable projection and digest;
3. finite 26-label/24-destination catalog;
4. canonical named-location command;
5. source-shaped workflow and plate movement compilers;
6. coherent provider authority and plan binding;
7. typed SQLite machine-state and command evidence;
8. restart and interruption recovery;
9. BMS models, relay, cockpit panel, and agent contract;
10. no-hardware verification records;
11. exact-tree reviews and deployment identity proof;
12. controller and physical acceptance records;
13. final destination/operation parity matrix signed off by Christian.

## 24. Current specification-work safety record

This specification pass performed source and repository reads and created this Markdown file only.

It did not:

- change robot production code;
- change BMS production code;
- run tests or builds;
- open or migrate a runtime database;
- access a controller;
- issue motion, homing, STOP, Abort, or raw motor commands;
- restart a service;
- deploy, commit, or push anything;
- claim controller or physical acceptance.
