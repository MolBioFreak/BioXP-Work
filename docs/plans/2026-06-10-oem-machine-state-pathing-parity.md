# BioXP OEM Machine-State Pathing Parity Implementation Plan

> **For Hermes:** Execute phase-by-phase with focused tests and commits at every phase boundary. Do not run live motion/homing commands while implementing this tranche.

**Goal:** Recreate the OEM machine-state/pathing layer needed for script/move planning parity before any live movement consumer uses OEM PositionTable coordinates.

**Architecture:** Add a pure, no-USB OEM parity planner beside the existing PositionTable binding. It models OEM `DefaultParameters`, the relevant `MachineStatus` fields, `confirmAxis("gripper")`, `getMidPoint(x,y)`, and `scriptmoveTo(...)` branch selection as an auditable dry-run step list. Existing live motion remains unchanged until a later explicit activation phase.

**Tech Stack:** Python 3, FastAPI route module, pytest, existing robot repo `/home/molbiofreak/bioxp_re`, BMS thin proxy `/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py`.

---

## Non-negotiable safety constraints

- No homing commands.
- No axis movement commands.
- No USB/motor controller writes from the planner.
- All new endpoints are read-only/dry-run and must return `opened_usb=false`, `physical_motion=false`, `motion_commanded=false`.
- Any live execution of these plans is out of scope until explicitly requested and separately gated.

## OEM source anchors

- `DefaultParameters.cs:47-59`: `PSUDO_Z_HOME_HEIGH=500`, `PSUDO_Z_HOME_LOW=65000`, default current pseudo-home `65000`.
- `DefaultParameters.cs:61-84`: `GantryLoad(...)` and `ForceToHighHome()` pseudo-home transitions.
- `ClassControlInterface.cs:3663-3688`: `moveTo(locationID,column,row,Tip10,highPos)` coordinate formula.
- `ClassControlInterface.cs:3691-3715`: `moveTo(locationID,offsetX,offsetY)` clamp/pseudo-home formula.
- `ClassControlInterface.cs:3734-4014`: `scriptmoveTo(...)` route-selection/pathing logic.
- `ClassControlInterface.cs:3858`: `confirmAxis("gripper") && !TipLoaded` direct move branch.
- `ClassControlInterface.cs:3862-3988`: tip-loaded path/midpoint/bin-special branches.
- `ClassControlInterface.cs:3990-4012`: dirty-tip/fallback branches.
- `ClassControlInterface.cs:5254-5366`: `getMidPoint(x,y)` midpoint generator.

## Phase 1 — OEM state model

**Objective:** Implement pure OEM state helpers with exact pseudo-Z transitions and relevant MachineStatus fields.

**Files:**
- Create: `src/bioxp/oem_compat/machine_state.py`
- Test: `tests/test_oem_machine_state_pathing.py`

**Acceptance:**
- Default pseudo-home is 65000.
- `force_to_high_home()` sets 500.
- `gantry_load(tiploaded=known)` sets 500.
- `gantry_load(tiploaded=None, plateloaded=None)` sets 65000.
- `gantry_load(plateloaded="BIO_SECURITY_COVER")` sets 65000.
- Other plate-loaded states set 500.
- `confirm_axis("gripper")` is modeled from `axis_confirmed["gripper"]` and defaults false.

**Commit:** `feat: add OEM machine state model`

## Phase 2 — exact path planner

**Objective:** Implement a no-USB `OemPathPlanner` that consumes the bound SSD PositionTable and emits the sequence of OEM-equivalent planning steps for `scriptmoveTo`.

**Files:**
- Create: `src/bioxp/oem_compat/pathing.py`
- Modify: `src/bioxp/oem_compat/position_table.py` only if small helper hooks are needed.
- Test: `tests/test_oem_pathing_planner.py`

**Acceptance:**
- `getMidPoint(x,y)` reproduces `ClassControlInterface.cs:5254-5366` using the SSD PositionTable anchors for location IDs 1,2,3,11,14.
- `scriptmoveTo` target coordinate computation matches existing PositionTable formula.
- Direct branch when `confirmAxis("gripper") && !TipLoaded` emits one `moveTo(x,y,z)` step.
- Tip-loaded branch emits `moveZ(pseudo)` first when current Z is greater than pseudo-home.
- Midpoint branch emits midpoint `moveXY` steps followed by `moveZ(target_z)`.
- Special `locationid == 6` branch emits the OEM `82450`, `145000`, `XHighLimit-500`, `moveSteps x -1000`, pseudo-home sequence.
- Dirty-tip/non-dirty branches match the OEM branch labels and planned command shape.
- Every output includes `source_formula`/`source_lines`, `opened_usb=false`, `physical_motion=false`, `motion_commanded=false`.

**Commit:** `feat: add OEM script path planner`

## Phase 3 — read-only API/BMS surfaces

**Objective:** Expose the planner through read-only robot and BMS endpoints.

**Files:**
- Modify: `src/bioxp/oem_homing_routes.py`
- Test: `tests/test_oem_pathing_routes.py`
- Modify BMS: `platform/api/routers/bioxp.py`
- Test BMS: `platform/api/tests/test_bioxp_oem_shadow_proxy.py`

**Robot endpoints:**
- `GET /motion/oem/pathing/default_parameters`
- `GET /motion/oem/pathing/scriptmove_plan`

**BMS proxy endpoints:**
- `GET /api/bioxp/motion/oem/pathing/default_parameters`
- `GET /api/bioxp/motion/oem/pathing/scriptmove_plan`

**Acceptance:**
- Endpoints are read-only and report no USB/motion.
- BMS remains thin-proxy only; no path semantics in BMS.
- Query parameters make machine state explicit: current location, target location, current position, tip state, tip location, cleanPath, gripper confirmed, device type, positionflag, column, row.

**Commit robot:** `feat: expose OEM pathing dry-run routes`
**Commit BMS:** `feat: proxy BioXP OEM pathing planner`

## Phase 4 — verification/review

**Objective:** Run focused test/compile, reload APIs if needed, verify live read-only responses, and review implementation against this spec.

**Acceptance commands:**
- Robot: `python3 -m py_compile src/bioxp/oem_compat/machine_state.py src/bioxp/oem_compat/pathing.py src/bioxp/oem_homing_routes.py`
- Robot: focused pytest for all OEM config/pathing tests.
- BMS: `pytest platform/api/tests/test_bioxp_oem_shadow_proxy.py -q` and `py_compile routers/bioxp.py`.
- Live robot and BMS endpoint calls show `motion_commanded=false` and expected path steps.

**Commit:** no extra commit unless verification uncovers a fix.

## Explicit non-goals for this tranche

- Do not connect this dry-run planner to live move execution.
- Do not mutate persisted MachineStatus from these read-only endpoints.
- Do not infer camera/barcode physical validation; only encode source-derived formulas and config-derived offsets where source is explicit.
- Do not treat controller counters as physical motion proof.
