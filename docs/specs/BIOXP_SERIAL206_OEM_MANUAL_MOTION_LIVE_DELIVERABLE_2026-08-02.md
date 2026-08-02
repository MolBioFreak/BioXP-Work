# BioXP serial-206 exact-OEM manual motion and homing live-deliverable specification

Date: 2026-08-02  
Scope: BioXP 3200 serial 206, current July/August 2026 tranches only  
Execution constraint for this tranche: **tests, builds, compile/import probes, simulations, and physical motion commands are prohibited by the operator.** Production code and deployment may proceed; physical commissioning remains operator-run.

## 1. Deliverable

The next major deliverable is one live robot/BMS operator path in which:

1. X, Y, Z, and gripper relative step moves call the literal OEM `ClassControlInterface.moveSteps(axis, steps)` mechanism.
2. X, Y, Z, and gripper absolute moves call the axis-specific literal OEM `moveX`, `moveY`, `moveZ`, and `moveG` mechanisms, including their source-defined clamps/current behavior.
3. X, Y, Z, gripper, and thermal-door manual home call the literal OEM `HomeAxis` branch for that component.
4. Gripper Open, Open Wide, and Close call the complete OEM semantic methods, not a generic axis delta.
5. Thermal-door Open and Close call the complete OEM semantic methods, not a generic axis move.
6. Stop is an interrupt path. Per-component stop and exact OEM aggregate `forceAbortMotion` are not admitted through ordinary move readiness.
7. The selected-machine axis/designator table and the complete 29-row position table are visible in a compact operator workbench.
8. Position-table movement uses the selected serial-206 `PositionTable` and the exact source overload/context; it is not converted into an approximate jog.
9. BMS is a thin typed relay. It does not ask for acknowledgement, reason, operator, artifact, snapshot, debug-reuse, speed, acceleration, timeout, board, motor, or raw protocol values for the normal manual controls unless that value is a literal caller input of the selected OEM method.
10. Every non-success response preserves route, action ID, submitted BMS envelope, translated robot body, HTTP status, and complete robot response detail.
11. The main UI is a compact handler/motion workbench. The complete 150-action robot catalog remains available behind subsystem navigation/search and does not mask handler state or manual controls.
12. Candidate source, remote branch, deployed robot service, BMS Development API, BMS Development frontend, and actual browser origin identify the same current tranche before it is called live.

## 2. Frozen OEM authority

### 2.1 Primary source

Absolute path:

`/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

SHA-256:

`86093e5270c82ea2e45cb4de449076372ca79d9485ba6de9565d5eb255811e6e`

Relevant source anchors:

- axis/designator table: lines 29-111
- board construction and selected settings binding: lines 170 onward; `PositionTable = m_settingsWindow.PositionTable` at line 386
- thermal-door open: lines 2651-2676
- thermal-door close: lines 2678 onward
- gripper open: lines 3536-3567
- gripper close: lines 3569 onward
- position-table `moveTo(location,column,row,Tip10,highPos)`: lines 3663-3688
- position-table `moveTo(location,offsetX,offsetY)`: lines 3691-3715
- `moveSteps`: lines 4165-4204
- absolute X/Y/Z/G: lines 4206-4273
- manual `HomeAxis`: lines 4997-5052
- exact aggregate `forceAbortMotion`: lines 5095-5105

### 2.2 Controller board authority

Absolute path:

`/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_can/ClassCanLib/ClassBaseBoard.cs`

SHA-256:

`2622aee1810b9a8a52bed54a49196d50e50751465608c8420dd1e5077d95dd5e`

### 2.3 Selected serial-206 machine authority

The robot's immutable selected-machine snapshot is authoritative. The live position-table endpoint currently publishes 29 rows from:

`serial_206_oem_machine_snapshot:appdata/config.xml`

The source must consume this bound snapshot. No generic fallback, nearby machine profile, remembered limit, or caller-supplied board/motor is permitted.

## 3. Exact axis/designator mapping

The OEM `m_AxisIODesignater` maps:

| OEM designator | OEM board slot | OEM axis | selected CAN/controller identity |
|---|---:|---:|---|
| `MotorX` | 1 | 0 | deck / CAN 5 / motor 0 |
| `MotorY` | 0 | 0 | head / CAN 4 / motor 0 |
| `MotorZ` | 0 | 1 | head / CAN 4 / motor 1 |
| `MotorGrip` | 0 | 2 | head / CAN 4 / motor 2 |
| `MotorLid` | 2 | 0 | thermal board alias |
| `ThermalCycler` | 2 | 1 | thermal board function |
| `ThermalDoor` | 2 | 0 | thermal / CAN 6 / motor 0 |
| `IO` | 1 | 0 | deck IO function |
| `OutputChiller` | 3 | 1 | chiller board function |
| `ReagentChiller` | 3 | 0 | chiller board function |

`MotorLid` and `ThermalDoor` alias the same OEM board slot/axis. The UI must not misrepresent those aliases as two independently movable motors. ThermalCycler/chillers are not generic stepper-jog controls.

## 4. Manual relative movement contract

### 4.1 Supported components

Literal OEM arbitrary relative steps exist for X, Y, Z, and gripper only. Thermal door is not included in the OEM `moveSteps` switch and must not be added by inference.

### 4.2 Request

`POST /motion/oem/manual/relative`

```json
{"axis":"x|y|z|g","steps":10000}
```

No additional browser inputs are allowed.

### 4.3 Required implementation

For the selected component, dispatch exactly:

1. resolve `m_AxisIODesignater` from the immutable serial-206 table;
2. read current controller position;
3. compute target = current + steps and reject before dispatch if outside selected-machine bounds;
4. call board `moveSteps(mapped_axis, steps, true)`;
5. call `getCurrentPosition(mapped_axis)`;
6. return requested delta, before position, computed target, controller reply, and after position.

Do not set caller-selected speed/acceleration, convert to absolute movement, call a generic diagnostic recipe, zero the axis, alter reference state, or synthesize success from HTTP acceptance.

Relative movement may be used to establish controlled clearance before homing; therefore it must not require the referenced state it may help establish. It still requires the current owned transport, valid 24 V/interlock/motion-arm state, readable current position, and selected-machine target bounds.

## 5. Manual absolute movement contract

`POST /motion/oem/manual/absolute`

```json
{"axis":"x|y|z|g","position_steps":10000}
```

Axis-specific source behavior is mandatory:

- X: `moveX`; clamp requested target to a minimum of 60, then `moveToAbs(..., wait=true, false, false)`.
- Y: `moveY`; `moveToAbs(..., wait=true, false, false)`.
- Z: `moveZ`; clamp to `DefaultParameters.PSUDO_Z_HOME`, set Z maximum current using the selected OEM value/default method contract, then `moveToAbs(..., wait=true, false, false)`.
- G: `moveG`; `moveToAbs(..., wait=true, false, false)`.

A target outside the selected serial-206 envelope is rejected before dispatch. The response must identify any OEM clamp separately as `requested_target` and `effective_target`.

Thermal-door arbitrary absolute positioning is excluded because the OEM operator semantics are Home/Open/Close, not generic position entry.

## 6. Manual home contract

`POST /motion/oem/manual/home`

```json
{"axis":"x|y|z|g|door"}
```

No browser acknowledgment, reason, speed, timeout, predicate override, or implementation-mapped-predicate checkbox is permitted.

Literal OEM branches:

- X: `axisSearchHome(MotorX.axis, 250)`.
- Y: `axisSearchHome(MotorY.axis, 250)`.
- Z: set `Z_MOTOR_MAX_CURRENT_UP`, then `axisSearchHome(MotorZ.axis, 597)`.
- Gripper: set current 31; set stall threshold 5; for serial-206 `GripperVersion == 1`, `axisSearchHome(..., 200)`; restore current 10 after the call.
- Door: if OEM `confirmAxis("tcDoorClosed")` is true, set stall threshold to `TCDoorStallGuardThreshold + 2` and move absolute to 1000; then restore `TCDoorStallGuardThreshold` and call `doorSearchHome(axis, TC_DOOR_VELOCITY, TCDoorStallGuardThreshold)`.

A manual-home route is distinct from the `initializeMotors` startup sequence. It must not silently execute the X park, UI-zero, chiller, or system-status stages.

## 7. Semantic gripper and thermal-door controls

The normal operator workbench exposes:

- Gripper Home
- Gripper Open
- Gripper Open Wide
- Gripper Close
- Thermal Door Home
- Thermal Door Open
- Thermal Door Close

Each route has no free-form operator fields and invokes its complete source method. Generic relative movement is not used as a substitute for Open/Close. Temporary gripper current/stall/speed changes and their source-defined restoration remain internal to the semantic transaction.

## 8. Stop contract

### 8.1 Per-component stop

`POST /motion/oem/manual/stop`

```json
{"axis":"x|y|z|g|door"}
```

This bypasses ordinary move busy/admission state and dispatches the selected board/motor stop.

### 8.2 Aggregate OEM abort

`POST /motion/oem/manual/abort_all`

No body.

Exact source order:

1. set OEM `ClassBaseBoard.No24V = true` host state;
2. sleep 1 ms;
3. iterate OEM board slots 0 through 3;
4. for each non-null board call `forceAbortMotion()`.

It bypasses ordinary motion admission. The UI must not call an aggregate stop physically verified until operator commissioning proves delivery/effect, but the implementation must be exact and available as the emergency interrupt lane.

## 9. Initialization/homing integration

1. One robot-owned authoritative serial-206 initialization ledger must serve `/status`, provider status, `initializeMotors`, and `initializeMotion`.
2. The current disagreement—provider says M01 pending while `/status` says M01/M02 completed—must fail closed as `reconciliation_required`; it must never replay M01 automatically.
3. `initializeMotors` resumes only from the single authoritative expected stage.
4. Internal controller-verifiable stages advance from exact readback. Physical stages retain explicit single-use commissioning observation but do not expose generic `operator_ack` or `operator_reason` fields.
5. The current startup-step UI must use the robot-advertised stage enum; it must never permit numeric `"5"` for a named-stage field.
6. Constructor pipette stage and no-motion startup predecessor state must be represented as a guided sequence, not exposed as unrelated raw commands that merely return HTTP 409.

## 10. HTTP contract

### 10.1 Remove accidental 422s

- Normal manual controls use closed typed bodies shown above.
- The frontend renders enum/select inputs from the exact live schema rather than free text.
- Optional object/array fields are omitted when absent; they are not submitted as `{}` or `[]`.
- The BMS relay serializes only fields in the selected operation's discriminated union.
- No browser route submits hidden `operator_ack`, `operator_reason`, `reason`, `operator`, `capture_bundle`, `dry_run_bundle`, `snapshot_refs`, `reuse_prepared`, or arbitrary timeout/profile fields for normal manual actions.

### 10.2 Preserve legitimate refusal truth

A source/state refusal may remain 409, but the UI must display the exact blocker before invocation and preserve the full response after invocation. Required receipt fields:

- BMS action ID and semantic operation
- exact robot route
- submitted BMS envelope
- translated robot body
- HTTP status
- complete robot response JSON/detail
- robot command/transaction ID when created
- controller command attempted boolean
- physical motion commanded boolean
- controller acknowledged boolean
- terminal state

### 10.3 Current live failures to close

- `initialize_without_motion`: HTTP 409 because `constructor_pipette_stage` has not passed.
- startup step submitted `"5"`: HTTP 422 because the robot requires a named stage enum.
- `initialize_motors`: HTTP 422 because the generic form submitted empty `stage_approval`/`commissioning` objects with required children absent.
- aggregate initialization run: same empty-object HTTP 422 class.
- `/motion/oem/machine_config`: HTTP 500 because FastAPI/Pydantic cannot serialize a `mappingproxy`.

These are live current-tranche defects, not historical May items.

## 11. Compact BMS operator workbench

The BioXP landing surface order is fixed:

1. Connection and active robot identity.
2. Motion power/interlock state and Activate 24 V / Prepare Motion.
3. Current operation/command result with full error detail.
4. Five component cards: X, Y, Z, Gripper, Thermal Door.
5. Each component card: OEM designator, CAN board, motor index, position, reference state, selected bounds, switch/home state, current/speed/acc readback if the handler actually publishes it, exact supported controls.
6. Gripper semantic state/current controls.
7. Door/latch/solenoid and semantic door controls.
8. OEM initialization progress: authoritative M01-M19 and initializeMotion stage status.
9. Axis/designator table.
10. Searchable/filterable 29-row PositionTable with `Move To` only after selecting an exact OEM overload/context.
11. Pipette/thermal/chiller handler parameters in compact subsystem panels.
12. Primitive catalog, Meta Actions, and Logs in separate secondary navigation.

The 150-action catalog is never rendered as one default wall. It is searchable and grouped by subsystem/category, with advanced/read-only/dry-run actions hidden behind explicit expansion. Camera remains in the compact workbench but does not imply motion readiness.

## 12. Live identity and release boundary

Current pre-implementation identities observed on 2026-08-02:

- robot candidate/deployed source: `9f1fdf412553a6d10bf0c5714bd8a270a4b0aa92`
- robot `origin/test`: same SHA
- BMS canonical Development and `origin/test`: `864b23ce7bc073a6271f954be5a64c5f024d94db`
- BMS current BioXP changes include `e185624`, `86e3aa1`, `17b9a21`, and earlier operator-control commits in that ancestry
- live BMS catalog count: 150

Completion requires:

1. robot changes committed and pushed to `origin/test`;
2. exact robot SHA deployed through `bioxp-api.service` on port 8123;
3. BMS changes committed/pushed to `origin/test` from an isolated current worktree, not direct canonical edits;
4. canonical Development fast-forwards through its managed 60-second sync;
5. API and frontend report/use the new SHA;
6. actual operator browser origin displays the compact workbench and exact controls;
7. passive route/schema/status checks report no accidental 422/500 contract defects;
8. **tests run: 0** under the operator prohibition;
9. **physical motion commands issued by the assistant: 0**;
10. Christian performs subsequent physical homing/movement commissioning.

No production (`main`) promotion is included.