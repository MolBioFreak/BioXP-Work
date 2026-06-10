# BioXP Movement Testing Readiness: OEM vs New System Gap Assessment

Date: 2026-06-10

## Scope

Prepare to begin movement testing by comparing the OEM motion/pathing stack against the current Linux/BMS implementation, identifying gaps before motion, and defining the exact gates required before any supervised movement command.

This report is **no-motion**. It does not authorize homing, absolute moves, or full OEM path execution by itself.

## Evidence collected

Passive live artifact directory:

```text
/tmp/bioxp_movement_readiness_20260610T051243Z
```

Collected robot-local read-only/passive endpoints:

```text
/openapi.json
/status
/motion/power/status
/latch/status
/motion/axes/status?axes=x,y,z,g,door
/motion/reference/status?axes=x,y,z,g,door
/motion/range/status?axes=x,y,z,g
/motion/oem/machine_config
/motion/oem/position_table
/motion/oem/pathing/scriptmove_plan?...sample...
```

New comparison surface:

```text
GET /motion/oem/movement_readiness/comparison
GET /api/bioxp/motion/oem/movement_readiness/comparison
```

## OEM source baseline

The relevant OEM movement stack is not just a coordinate table. It consists of:

1. `config.xml` / original SSD AppData:
   - axis limits
   - calibration flags
   - instrument-specific offsets
   - `PositionTable`
2. `DefaultParameters.cs`:
   - `PSUDO_Z_HOME_HEIGH = 500`
   - `PSUDO_Z_HOME_LOW = 65000`
   - `GantryLoad(...)`
   - `ForceToHighHome()`
3. `ClassControlInterface.cs`:
   - `moveTo(locationID,column,row,Tip10,highPos)`
   - `moveTo(locationID,offsetX,offsetY)`
   - `scriptmoveTo(...)`
   - `getMidPoint(x,y)`
   - clean-path / waste-bin / tip-loaded branches
4. Runtime `MachineStatus`:
   - current location/well
   - tip loaded/dirty/location
   - current position
   - `confirmAxis("gripper")`
   - device type
5. Live execution:
   - controller prep / current / interlock
   - `moveX`, `moveY`, `moveXY`, `moveZ`, `moveSteps`
   - waits and status events
6. Physical truth:
   - observed/camera/fiducial movement, not only controller counters.

## Current implemented parity

### Present / source-backed

- Original SSD machine config is bound read-only.
- Axis limits are sourced from `config.xml`:
  - X `0..90263`
  - Y `0..102956`
  - Z `0..160000`
  - G `0..15000`
- `PositionTable` is parsed from original SSD config.
- `PositionTable` row count live: `29`.
- Read-only coordinate planning is available.
- Read-only pathing planning is available.
- `DefaultParameters` pseudo-Z transitions are modeled.
- `scriptmoveTo` dry-run planner includes source-line branch labels and emits step sequences.
- BMS proxies the read-only truth/planning routes as thin proxy only.

### Verified live sample

Robot-local sample `scriptmove_plan`:

```json
{
  "branch": "gripper_confirmed_no_tip_direct_moveTo",
  "target_coordinates": {"x": 21949, "y": 15637, "z": 46007},
  "step_count": 1,
  "opened_usb": false,
  "physical_motion": false,
  "motion_commanded": false
}
```

OpenAPI live contains:

```text
/motion/oem/machine_config
/motion/oem/position_table
/motion/oem/pathing/default_parameters
/motion/oem/pathing/scriptmove_plan
/motion/oem/movement_readiness/comparison
/motion/arm/strict_startup
/motion/axis/relative
/motion/axis/absolute
```

## Live no-motion readiness snapshot

From `/tmp/bioxp_movement_readiness_20260610T051243Z/SUMMARY.json` plus inspected raw endpoint payloads:

- `/status`:
  - `status = degraded`
  - `runtime_available = true`
  - `hardware_connected = false`
  - `transport = usb`
- `/motion/power/status`:
  - `motion_arm.armed = false`
  - `motion_arm.reason = startup`
  - `motion_arm.note = strict init not yet run`
  - `rail_24v.no24v = false`
- `/latch/status`:
  - `door_sensor = 1`
  - `solenoid_state = 0`
  - `latch_sensor = 1`
  - `rail_24v = 0`
- `/motion/axes/status`:
  - X/Y/Z/G/door speeds all `0`
  - X/Y/Z/G/door positions all `0` in the current controller frame
- `/motion/reference/status`:
  - X `desynced`, source `operator_physical_truth`
  - Y `desynced`, source `operator_physical_truth`
  - Z `desynced`, source `motion_hard_reset`
  - G `desynced`, source `motion_hard_reset`
  - door `desynced`, source `motion_hard_reset`

## Gap matrix

### 1. Config and position table

Status: **ready for pre-move review**.

Evidence:

```text
/motion/oem/machine_config
/motion/oem/position_table
/motion/range/status
```

Gap: none for read-only planning.

### 2. Path planning

Status: **ready for dry-run comparison**.

Evidence:

```text
/motion/oem/pathing/default_parameters
/motion/oem/pathing/scriptmove_plan
```

Gap: dry-run planner is not yet the live executor. It emits OEM-equivalent planned steps, but live movement routes do not consume the step plan as their sole source of truth.

### 3. Live movement execution adapter

Status: **not complete for full OEM path execution**.

Current live movement routes are still generic axis routes:

```text
POST /motion/axis/relative
POST /motion/axis/absolute
```

Gap: no committed adapter yet that consumes `scriptmove_plan.steps` and executes `moveXY`/`moveZ`/`moveX`/`moveY`/`moveSteps` with one-step-at-a-time evidence, waits, interlocks, and abort semantics.

Implication: do **not** begin with full OEM path execution. Begin only with a supervised micro-move/proof move after live gates pass.

### 4. Movement observation / operator truth

Status: **not an OEM-parity blocker**.

Correction: this is a commissioning observation discipline, not a separate parity gap. The OEM stack also does not provide independent automatic physical-motion proof; it commands controller motion and relies on instrument/operator context. For our tests, we still record operator/camera observation so the result is auditable.

Gap: none for OEM parity. Observation remains required for commissioning evidence quality.

### 5. Arm/interlock/live state

Status: **currently not armed**.

Current passive state:

```text
motion_arm.armed = false
reason = startup
strict init not yet run
rail_24v.no24v = false
all speeds = 0
```

Gap: before any movement test, the robot must pass non-homing strict startup or equivalent explicit no-homing activation, then re-check latch/rail/axis speeds/reference.

### 6. Reference state

Status: **not ready for blind absolute/path motion**.

All motion axes are currently `desynced`.

Gap: blind absolute coordinates and full PositionTable/path moves are not appropriate until reference/physical frame is reconciled or the movement class is deliberately chosen as a small supervised relative proof move.

### 7. Gripper/current safety

Status: **must be checked live immediately before motion**.

Known invariant:

```text
if G speed == 0: G param6 <= 10 and G param7 <= 10 unless active gripper operation is in progress
```

Gap: readiness report did not yet include a fresh extracted G param6/param7 summary; check `/motion/axis/g/status` or `/motion/axes/status?axes=g` immediately before movement.

## Go / no-go classification

### Ready now

Ready to begin the **pre-movement no-motion gate**:

1. Use `/motion/oem/movement_readiness/comparison` to confirm the gap matrix is visible.
2. Plan the intended movement through `/motion/oem/pathing/scriptmove_plan` if it is an OEM semantic target, or through explicit relative-move spec if it is a proof micro-move.
3. Capture passive state: status, power, latch, axes, reference, range, G current safety.
4. If operator authorizes, run strict startup with `run_homing:false` only.
5. Re-capture passive state.

### Not ready now

Not ready for:

- blind absolute moves
- switch-search homing
- full OEM `scriptmoveTo` execution
- movement reported without commissioning observation
- any unattended move

### Eligible first movement class after gates pass

Only after arm/latch/current/speed gates pass:

- one supervised relative micro-move on a selected axis
- operator/camera before/after observation recorded for commissioning evidence
- controller telemetry classified as controller-only until physical proof confirms motion

## Required immediate pre-move checklist

Before the first movement command:

```text
GET /status
GET /motion/power/status
GET /latch/status
GET /motion/axes/status?axes=x,y,z,g,door
GET /motion/reference/status?axes=x,y,z,g,door
GET /motion/range/status?axes=x,y,z,g
GET /motion/axis/g/status
GET /motion/oem/machine_config
GET /motion/oem/position_table
GET /motion/oem/pathing/scriptmove_plan?...intended scenario...
```

Pass criteria:

- all axis speeds `0`
- 24V rail good (`no24v=false`)
- latch/door raw truth acceptable for intended activation
- override state visible; override OFF unless explicitly required
- G idle current safe (`10/10` target)
- arm state explicitly `strict_init_pass` after non-homing activation if it was startup/unarmed
- reference/desync state acknowledged in the movement choice
- physical proof method ready before command is sent

## Bottom line

The new system is now good enough to **compare and plan** OEM-equivalent motion paths. It is **not yet good enough to execute full OEM paths blindly**.

Given the current live state (`motion_arm.armed=false`, all axes `desynced`), the correct next action is not a move. The correct next action is the no-motion activation/readiness gate, followed by exactly one supervised proof micro-move only if those gates pass and Christian/operator is watching.
