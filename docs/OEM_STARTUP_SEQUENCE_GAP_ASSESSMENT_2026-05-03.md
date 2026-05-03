# BioXP3200 OEM startup-sequence gap assessment — 2026-05-03

## Purpose

Define the gap between the current Linux BioXP runtime and the OEM Windows application startup/door-close initialization sequence, with the specific goal of reaching a robot-local command that can mirror the previous Windows app’s full startup readiness sequence.

This is scoped to the BioXP robot-local/OEM-compat runtime, not BMS UI polish. BMS should remain a thin operator/proxy layer after the robot-local sequence is correct.

## Bottom line

The current Linux runtime has several useful low-level pieces, but it does not yet replicate the OEM application startup program.

The OEM behavior is not just “home axes.” It is an application-level initialization state machine:

1. Windows auto-launches GenBotApp.
2. BioXP settings load from `config.xml`.
3. ControlLib starts CAN/pipette/camera-support surfaces.
4. ControlLib performs configure-only motor initialization: `initializeMotorsWithoutMotion()`.
5. Main app runs `initializeEnvironment()`.
6. `initialCheck()` validates CAN/door/latch and cycles board activation.
7. If door/latch are not closed, UI prompts the operator to close the door for initialization.
8. Door/latch close event re-runs `initialCheck()`.
9. If door and latch are closed, the app enqueues `initializeSystem` into a dedicated motion command queue.
10. The motion worker consumes `initializeSystem`.
11. `BioXPMainWindow.initializeSystem()` calls `ControlLib.initializeMotion()`.
12. `initializeMotion()` calls `ClassControlInterface.initializeMotors()`.
13. Only then does full physical Z/G/X/Y/door homing and readiness happen.
14. After homing, the app handles tip cleanup, cover inspection, park gantry, door open, ready screen/job flow depending `StartMode`.

Linux currently has pieces of this but not the sequence as a first-class, source-shaped startup program.

## Evidence anchors from OEM Windows decompile

### Application auto-launch

`decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

- lines 570-578: creates ClickOnce/appref-ms shortcut in Windows Startup folder when not a development machine.
- This is why the Windows computer auto-loads the app.
- This is not a homing XML script.

### ControlLib constructor / configure-only initialization

`decompiled_src/BioXPControlLib/ControlLib.cs`

- line 983: `m_ControlInterface.initializeMotorsWithoutMotion();`

`ClassControlInterface.cs`

- line 3181: `initializeMotorsWithoutMotion()`
- line 3348: `initializeMotors()`

Important distinction:

- `initializeMotorsWithoutMotion()` configures the hardware and motor parameters.
- `initializeMotors()` performs the physical startup homing sequence.

### Door-close gate / environment initialization

`BioXPMainWindow.cs`

- line 821: `initializeEnvironment();`
- line 973: `private void initializeEnvironment()`
- line 978: `m_control.initialCheck();`
- lines 979-988: door/latch not closed leads to warning / wait-initialization prompt.
- lines 989-996: door and latch closed logs “Start system initialization” and enqueues `initializeSystem`.

### Door/latch event path

`BioXPMainWindow.cs`

- line 715: subscribes to `handleEnclosureDoorEvent`.
- line 716: subscribes to `handleLatchEvent`.
- line 2428: `m_canControl_handleEnclosureDoorEventProcess(bool dooropen)`.
- lines 2487-2495: on door close, re-run `initialCheck()`, then enqueue `initializeSystem` if `EnclosureDoorClosed && LatchClosed`.

### Motion command queue

`BioXPMainWindow.cs`

- line 228: `BlockingCollection<motionCommands> m_commandQueue`.
- line 2030: `motion_thread_process()`.
- line 2041: consumes `m_commandQueue.Take()`.
- lines 2045-2049: case `initializeSystem` calls `initializeSystem()` then sets ready status.

This matters because OEM does not run startup homing as a blocking HTTP request. It serializes motion work through a dedicated motion worker.

### initializeSystem → initializeMotion

`BioXPMainWindow.cs`

- line 1046: `initializeSystem(bool skipInitializeMotion = false)`.
- line 1143: `m_control.initialCheck();`
- line 1159: `m_control.initializeMotion();`

### initializeMotion → physical homing

`ControlLib.cs`

- line 8797: `initializeMotion()`.
- line 8803: `m_ControlInterface.initializeMotors();`
- lines 8805-8846: tip-status and tip-cleanup behavior after motor initialization.

### initialCheck

`ControlLib.cs`

- line 8728: `initialCheck()`.
- lines 8732-8740: waits for CAN readiness.
- line 8748: sets LED white.
- line 8751: calls `checkDoorStatus()`.
- lines 8756-8757: deactivates then activates boards.

### Door/latch sensor status

`ControlLib.cs`

- lines 8762-8770: `queryDoorStatus()` reads door sensor and latch sensor; true only if both sum to 2.

### config.xml dependency

`ClassBioXPSettings.cs`

- line 169: `CONFIG_FILE = "config.xml"`.
- lines 2847-2854: loads `./config.xml` or `m_appDir + "\\config.xml"`.
- line 2857: logs warning if missing.
- lines 3835-3857: saves offsets including gripper positions, TC door constants, Z motor max current/stall constants, output buffer, outlier factor.
- lines 3900-3923: saves X/Y/Z/G axis limits.

The actual `config.xml` was not found in the current extracted backup tree by file search. The `.deploy` app config found is only .NET runtime binding redirects, not machine calibration/config.

## Current Linux runtime inventory

Primary files inspected:

- `src/bioxp/api.py`
- `src/bioxp/usb_driver.py`
- `src/bioxp/oem_compat/*`
- `scripts/bioxp_supervised_oem_startup_homing_stepwise.sh`

### What exists now

1. Basic robot-local API service.
2. Board activation and strict startup arm gate.
3. Latch/door/24V query and lock/unlock surfaces.
4. `motion_arm_strict_startup(run_homing=False)`.
5. Monolithic homing disabled at `/motion/arm/strict_startup` when `run_homing=true`.
6. Stepwise OEM startup endpoint: `POST /motion/oem/startup_step`.
7. Source-shaped rough startup step order:
   - `z-home`
   - `gripper-clear`
   - `gripper-home`
   - `x-home`
   - `x-park-6000`
   - `y-home`
   - `door-home`
   - `y-set-home`
8. `motor_oem_initialize_without_motion()`.
9. `motor_startup_homing_mimic()` with rough OEM order.
10. Emergency kill hardening outside the API path.
11. OEM-compat dry-run/scaffold routes and artifacts.

### Current Linux code anchors

`src/bioxp/api.py`

- lines 1902-1918: `/motion/arm/strict_startup`, with live `run_homing` blocked.
- lines 2020-2099: `/motion/oem/startup_step` step runner.

`src/bioxp/usb_driver.py`

- line 2245: `motion_arm_strict_startup()`.
- line 3176: `motor_oem_initialize_without_motion()`.
- line 3197: `motor_oem_go_home()`.
- line 3280: `motor_oem_door_search_home()`.
- line 3342: `motor_oem_home_axis()`.
- line 3375: `motor_startup_homing_mimic()`.

## Critical gaps preventing OEM startup mirroring

### Gap 1 — No first-class OEM startup state machine

OEM has:

`app_launch → configure-only → initializeEnvironment → initialCheck → wait for door/latch → door-close event → enqueue initializeSystem → motion worker → initializeSystem → initializeMotion → initializeMotors → inspect/park/door/open/ready`

Linux currently has:

- direct strict startup endpoint
- direct stepwise startup-step endpoint
- scripts that call endpoints
- no durable app-level startup state machine equivalent

Needed:

Implement a robot-local `OEMStartupProgram` or equivalent service that models the OEM application sequence explicitly.

Target source files:

- new: `src/bioxp/oem_startup_program.py` or `src/bioxp/oem_compat/startup_program.py`
- API: `src/bioxp/api.py`
- tests: `tests/test_oem_startup_program.py`

Required states:

- `created`
- `app_started`
- `settings_loaded`
- `control_lib_constructed`
- `can_started`
- `pipette_initialized_or_checked`
- `motors_configured_without_motion`
- `environment_check_started`
- `initial_check_passed`
- `waiting_for_door_close`
- `door_close_observed`
- `initialize_system_queued`
- `initialize_system_running`
- `initialize_motion_running`
- `initialize_motors_running`
- `homing_complete`
- `post_home_inspection_running`
- `parked`
- `ready`
- `failed_closed`

The startup program should be callable on command, but internally source-shaped like OEM.

### Gap 2 — No OEM-equivalent motion command queue/worker

OEM uses `BlockingCollection<motionCommands>` and a dedicated `motion_thread_process()`.

Linux currently uses FastAPI blocking endpoint calls and shell scripts. The earlier monolithic homing path wedged because the entire physical sequence lived inside one API request and held USB ownership.

Needed:

Add a single robot-local motion worker / command queue:

- queue commands like `initializeSystem`, `abortjob`, `validateJob`, `wakefrompause`
- serialize motion operations
- expose queue state/status
- write stage artifacts continuously
- emergency kill must remain out-of-band and able to preempt USB ownership

Target files:

- new: `src/bioxp/motion_worker.py` or `src/bioxp/oem_compat/motion_queue.py`
- API route group in `src/bioxp/api.py`

Endpoints:

- `POST /oem/startup/request` — enqueue startup sequence
- `GET /oem/startup/status` — latest startup program state
- `POST /oem/startup/door_event` — inject/handle door close/open state for command-mode testing
- `POST /oem/motion_queue/abort` — fail-closed queue abort
- `GET /oem/motion_queue/status`

### Gap 3 — Missing real machine config.xml import

OEM depends on `config.xml` for machine-specific constants. Current backup search did not find it.

This is probably not optional. It likely controls values directly relevant to whether motion occurs correctly:

- `StartMode`
- `GripperVersion`
- `Calibrated`
- `CameraCalibrated`
- X/Y/Z/G axis limits
- Z motor max current up/down
- Z motor stall guard threshold
- thermal door velocity/acc/current/stall threshold
- gripper open/close/wide positions
- origin offsets
- position table
- camera/inspection config flags

Needed:

1. Search the original/restored Windows SSD more broadly, outside the current extracted tree:
   - ClickOnce app data cache
   - installed app working directory
   - desktop shortcut working directory
   - `%LOCALAPPDATA%/Apps/2.0/...`
   - `%PROGRAMDATA%`
   - OEM install directories
   - any restored user profile paths
2. Add a parser/importer for `config.xml`.
3. Make startup program refuse live homing if config source is missing or values are default/unproven.

Target files:

- new: `src/bioxp/oem_config.py`
- new: `tests/test_oem_config_import.py`
- maybe `config/oem/*.json` binding outputs after parsing

### Gap 4 — Current homing primitive is not OEM-safe enough

Current Linux `motor_oem_go_home()` still has a symptom-patch in it:

- Z-only direction reversal from OEM source (`move_right` instead of `move_left`) exists in `usb_driver.py` lines 3210-3218.
- It still validates using a simple home switch predicate.
- It does not yet implement a robust source-derived switch-polling home search.

OEM behavior uses source-level home/search commands and sensor predicates. For Linux parity we need to prove which raw switch values match OEM predicates, not assume.

Needed:

1. Raw switch audit for every axis:
   - GAP9 raw
   - GAP10 raw
   - position
   - speed
   - switch masks
   - stall/current params
2. Define per-axis OEM home predicate:
   - which switch
   - polarity
   - whether active is 0 or 1
   - whether startup vs manual home differs
3. Replace `wait_stopped_then_check_home` with `move_while_polling_switches`:
   - poll speed/position/GAP9/GAP10 during motion
   - stop immediately on correct switch predicate
   - stop immediately on wrong/opposite limit predicate
   - stop on no-motion/runaway
   - never set home unless predicate verified
4. Remove the Z direction hack and replace it with config/source/trace-backed direction semantics.

Target files:

- `src/bioxp/usb_driver.py`
- new tests for switch predicate mapping
- new diagnostic endpoint/script for raw switch audit

### Gap 5 — initialCheck is not replicated exactly as a named startup stage

OEM `initialCheck()`:

- waits for CAN ready
- sets LED white
- checks door/latch
- deactivates boards
- activates boards

Linux has components of this, but not as a named, tested OEM stage with source-shaped output.

Needed:

Add `oem_initial_check()` with explicit output:

- CAN/backend readiness
- LED white command result
- door/latch/24V snapshot
- deactivate board results
- activate board results
- pass/fail reason
- source anchor

Target:

- `src/bioxp/usb_driver.py` or new startup program wrapper
- API `POST /oem/initial_check`

### Gap 6 — Door/latch event behavior is not mirrored

OEM waits for door/latch and responds to door events.

Linux currently has direct scripts and direct endpoints; it can query latch status, but it does not expose an OEM-like door-close state transition.

Needed:

- `waiting_for_door_close` state
- `door_event(open/closed)` handler
- re-run initialCheck on close
- queue initializeSystem only if `EnclosureDoorClosed && LatchClosed`
- enforce retry counters / failure state similar to OEM `m_doorcloseretry`
- do not conflate thermal door with enclosure door/latch

Target:

- startup program state machine
- API door-event/status endpoints

### Gap 7 — Post-homing readiness sequence is incomplete

OEM `initializeSystem()` does more than homing:

- shows status steps
- handles unexpected shutdown saved status
- runs `initializeMotion()`
- optionally self-test
- camera check
- inspect covers
- park gantry
- open door
- prepare ready/job flow depending `StartMode`
- post logs/config/images depending host/mode

Linux currently focuses on motor startup/homing only.

Needed minimum for “ready for use” mirror:

- tip status query and tip cleanup branch
- cover inspection placeholder must fail closed if unavailable, not silently succeed
- park gantry equivalent
- open door / unlock door equivalent if OEM does it for selected mode
- ready-state output
- StartMode-aware branch, at least for current machine mode

Target files:

- `src/bioxp/oem_compat/control_lib.py`
- `src/bioxp/api.py`
- pipette/CAN subsystem files
- vision/camera facade files

### Gap 8 — Pipette initialization/readback is not at OEM startup parity

ControlLib constructor and initializeMotion query pipette state and handle tip cleanup. OEM uses `ClassPipetteCollection` behavior, not just motion board control.

Linux has liquid endpoints and CAN surface, but the confidence level is lower than motion:

- likely partial ACK/readback semantics
- not fully bound to OEM startup readiness
- tip loaded/exists status must be hardware truth, not just shadow state

Needed:

- source-shaped `ClassPipetteCollection` startup/check path
- `queryTipStatus(-1)` equivalent
- `checkedPipetteCondition()` / `checkedPipetteStatus()` equivalent
- startup program must include pipette init/check stage before or during readiness as OEM does
- fail closed if pipette status cannot be proven and the OEM mode requires it

### Gap 9 — Vision/inspection readiness is not at OEM startup parity

OEM startup can include camera initialization and cover/deck inspection depending settings:

- `CheckCamera`
- `CameraInstalled`
- `inspectCover()`
- `CheckCamera()`
- cover/pool/deck vision semantics

Linux has camera devices/snapshot/control surfaces, but not a faithful `CVisionLib` startup inspection layer.

Needed:

- config-driven camera/inspection gate
- explicit unavailable/fail-closed status for any vision stage not implemented
- artifact capture for each image/inspection result
- no fake pass

### Gap 10 — Status semantics do not match OEM readiness semantics

OEM updates `ClassStatusLog` and `ClassMachineStatus` through the startup flow.

Linux currently has multiple status planes that can disagree:

- `/status`
- `/motion/power/status`
- reference state
- motion arm state
- latch status
- OEM-compat dry-run artifacts

Needed:

Create one OEM startup-readiness status surface:

- current stage
- last successful stage
- door/latch state
- CAN/USB state
- board activation state
- config source
- axis reference/home state
- pipette state
- camera/inspection state
- thermal door state
- ready/not-ready reason
- source anchor per stage

Endpoint:

- `GET /oem/startup/status`

### Gap 11 — Artifact capture is incomplete for live startup parity

OEM logs statuses and errors; for Linux parity we need stronger artifacts:

- per-stage JSON
- raw command/reply traces
- sensor snapshots before/after
- switch-poll traces during homing
- physical-motion truth fields
- emergency-stop provenance if invoked

Needed:

Every command-mode startup run must create an artifact root like:

`/tmp/bioxp-live-runs/<timestamp>_OEM_APP_STARTUP_SEQUENCE/`

Minimum files:

- `startup_request.json`
- `config_binding.json`
- `initial_check.json`
- `door_wait.json`
- `door_close_event.json`
- `queue_events.jsonl`
- `initialize_system.json`
- `initialize_motion.json`
- `initialize_motors_trace.jsonl`
- `axis_switch_trace_<axis>.jsonl`
- `pipette_startup.json`
- `vision_inspection.json`
- `final_readiness.json`
- `failure.json` if any

### Gap 12 — BMS should not own the hardware truth

For this specific goal, BMS should not implement the startup sequence. BMS should call the robot-local OEM startup program and display its state/artifacts.

Needed later after robot-local implementation:

- BMS proxy routes:
  - `/api/bioxp/oem/startup/request`
  - `/api/bioxp/oem/startup/status`
  - `/api/bioxp/oem/startup/artifacts`
- UI panel showing stage, gate, ready reason, artifacts.

But the sequence itself must live robot-local.

## Proposed target command behavior

We want a command that mirrors Windows app startup on demand without relying on the Windows UI:

`POST /oem/startup/request`

Example request:

```json
{
  "mode": "live",
  "operator_ack": "INITIALIZE",
  "require_config": true,
  "door_policy": "wait_for_closed",
  "run_homing": true,
  "artifact_root": "/tmp/bioxp-live-runs/20260503_oem_startup"
}
```

Expected internal behavior:

1. Create startup session/artifact root.
2. Load OEM config binding.
3. Start/verify backend/CAN/USB.
4. Initialize pipette group/status if configured.
5. Run `initializeMotorsWithoutMotion` configure-only stage.
6. Run `initialCheck`.
7. If door/latch not closed, enter `waiting_for_door_close` and return status, not failure.
8. On door/latch close event or polling transition, re-run `initialCheck`.
9. Queue `initializeSystem` on the motion worker.
10. Motion worker runs `initializeSystem`.
11. `initializeSystem` runs `initializeMotion`.
12. `initializeMotion` runs `initializeMotors`.
13. `initializeMotors` performs source-shaped homing with switch-poll traces.
14. Run post-homing tip cleanup/inspection/park/door/ready stages according to config.
15. Publish final ready state or fail-closed reason.

## Phase plan to close the gap

### Phase 0 — Freeze live actuation assumptions

Goal: stop guessing while preserving operator control.

Tasks:

- Remove live symptom hacks from being treated as parity fixes.
- Keep scripts runnable, but route them through safer diagnostics with explicit operator prompts.
- Require artifact root for live startup program.
- Keep emergency kill out-of-band.

Deliverable:

- no new guessed homing direction changes
- current scripts left available
- all new live code emits source anchors/artifacts

### Phase 1 — Config recovery/import

Goal: get the missing OEM `config.xml` or prove where defaults are coming from.

Tasks:

- Search actual Windows SSD/user/app dirs for `config.xml` and `InspectionSettings.xml`.
- Parse config into structured JSON.
- Bind constants into Linux startup program.
- Refuse live startup if config is missing unless explicitly overridden for diagnostic mode.

Files:

- `src/bioxp/oem_config.py`
- `tests/test_oem_config_import.py`
- `config/oem/bioxp_oem_config_binding.json`

### Phase 2 — OEM startup state machine and initialCheck

Goal: mirror app-level door/latch initialization flow without physical homing first.

Tasks:

- Implement `OEMStartupProgram`.
- Implement `oem_initial_check()`.
- Implement door/latch wait state.
- Implement door-close event/polling transition.
- Implement queueing of `initializeSystem` but initially with homing dry-run/blocked.

Tests:

- door open leads to `waiting_for_door_close`
- door close queues `initializeSystem`
- failed door/latch does not run homing
- initialCheck outputs LED/door/latch/deactivate/activate stage records

### Phase 3 — Motion command queue/worker

Goal: replace monolithic request-owned homing with OEM-shaped serialized worker.

Tasks:

- Add motion command queue.
- Add worker status endpoint.
- Add abort/fail-closed behavior.
- Persist queue events.
- Ensure emergency kill can preempt the worker.

Tests:

- only one motion command active
- abort disarms and writes artifact
- startup request returns queued/running status instead of wedging HTTP

### Phase 4 — Switch predicate and homing primitive rewrite

Goal: make homing sensor-driven and source/config backed.

Tasks:

- Add raw switch audit endpoint.
- Log GAP9/GAP10 for all axes without interpreting active state.
- Decode OEM `queryHome` / `queryRightSensor` polarity per board/axis.
- Implement switch-polling home search.
- Remove Z direction hack after replacing it with predicate matrix evidence.

Tests:

- fake transport proves stop-on-correct-switch
- wrong switch stops/fails
- no-motion stops/fails
- setHome only after verified predicate

### Phase 5 — Source-shaped initializeMotors

Goal: faithfully execute `ClassControlInterface.initializeMotors()` with artifacts.

Tasks:

- Implement startup Z/G/X/Y/door sequence in worker.
- Include exact config-driven currents/stall/speed.
- Include gripper version branch.
- Include X setHome/speed/move6000.
- Include Y setHome after door.
- Include door validation conditions.

### Phase 6 — initializeMotion post-homing behavior

Goal: reach OEM “ready for use,” not just homed axes.

Tasks:

- Integrate pipette tip status query.
- Implement tip cleanup branch or fail-closed if not supported.
- Integrate cover inspection/camera gates or explicit unavailable fail-closed.
- Park gantry.
- Door open/unlock/ready depending `StartMode`.

### Phase 7 — BMS thin operator surface

Goal: expose robot-local startup state to BMS after robot-local truth exists.

Tasks:

- Add BMS proxy endpoints.
- Add frontend stage/status/artifact view.
- Do not duplicate startup logic in BMS.

## Readiness estimate

For the specific goal “mirror Windows app full startup sequence on command”:

- Low-level component availability: ~45-60%.
- Source understanding of OEM startup path: ~75-85%.
- Current Linux implementation of full OEM startup program: ~25-35%.
- Safe live readiness to run the full mirrored startup sequence unattended: not ready.

Why not ready:

- no first-class OEM startup state machine
- no OEM-equivalent motion worker queue
- missing config.xml import
- homing switch/direction predicates not proven
- pipette and vision startup branches not integrated
- artifacts/status are not yet one coherent OEM readiness record

## Minimum acceptance criteria

Before we claim “Linux can mirror Windows startup on command,” all of these must be true:

1. `config.xml` or equivalent OEM config binding loaded and reported.
2. Startup program state machine exists and is tested.
3. Door/latch close gate behaves like OEM.
4. Motion command queue serializes `initializeSystem`.
5. `initialCheck` source-shaped output exists.
6. `initializeMotorsWithoutMotion` source-shaped output exists.
7. `initializeMotors` executes with switch-polling traces and correct per-axis predicates.
8. No direction hacks remain unanchored.
9. Pipette startup status path is represented.
10. Vision/inspection readiness path is represented or explicitly blocks.
11. Final ready/not-ready reason is a single coherent OEM startup status.
12. Emergency stop remains out-of-band and tested.
13. Full run writes durable artifacts.
14. BMS only proxies/displays, not owns the sequence.

## Immediate next engineering slice

Do this next, before live homing attempts:

1. Add `OEMStartupProgram` skeleton with states and artifacts.
2. Add `oem_initial_check()` source-shaped stage.
3. Add config import/search scaffolding and explicit “config missing” status.
4. Add raw switch audit endpoint.
5. Add tests proving door/latch waiting and queue transition.

This gives us the OEM-shaped control shell first. Then homing becomes one stage inside a proper startup program instead of a free-floating endpoint/script.
