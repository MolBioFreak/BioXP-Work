# BioXP3200 OEM Runtime-App Parity Hard Requirement Spec

> For Hermes: Use subagent-driven-development and test-driven-development when implementing this spec. The target is source-shaped OEM runtime parity, not merely HTTP endpoint coverage.

Date: 2026-05-03
Repo: `/home/dalab/Desktop/BioXP 3200 Development Work/bioxp_re`
OEM oracle root: `/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup`

## 0. Non-negotiable requirement

The Linux robot-local runtime must implement parity with the OEM Windows application runtime model. It is not sufficient to expose one-shot FastAPI endpoints that call hardware directly. The OEM application is a continuously running, stateful runtime with:

- a blocking command queue and always-on motion thread;
- a single ownership boundary around motion/USB/CAN hardware;
- door/latch/board/pipette/control event handlers;
- status logs and machine state transitions;
- initialization, pause/resume, abort, job validation, and job-prep commands;
- start-mode, saved-status, self-test, camera, cover inspection, gantry parking, and door-ready branches;
- explicit UI/operator-state semantics, even if Linux exposes them as JSON rather than WPF pages.

This spec makes runtime-app parity a hard requirement. Linux can expose a modern API, but the API must be a thin surface over an OEM-shaped runtime, not the runtime itself.

## 1. OEM source anchors

The following source anchors are normative. If code behavior conflicts with these anchors, the implementation must either match the OEM behavior or record an explicit, reviewed safety deviation.

### 1.1 `BioXPMainWindow.initializeEnvironment()`

File:
`/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

Lines: 973-1004

Observed behavior:

- If `m_control.m_canControl.CAN_READY`:
  - call `m_control.initialCheck()`;
  - if enclosure and latch are open, show warning;
  - if enclosure door open, unlock door, increment `m_doorcloseretry`, show wait warning;
  - if enclosure door closed and latch closed, log `Start system initialization`, enqueue `motionCommands { name = "initializeSystem" }` into `m_commandQueue`;
  - otherwise unlock door, show ready state, log ready.
- If CAN is not ready:
  - StartMode gates whether to return;
  - navigate to main menu page;
  - set `m_control.GantryAvailable = true`.

Linux implication:

- startup request must not execute initialization inline;
- startup request must evaluate CAN/backend readiness and door/latch state;
- closed door/latch must enqueue `initializeSystem` into the runtime queue;
- open door/latch must enter an operator-action state, not fake ready;
- no direct homing endpoint may bypass this state machine.

### 1.2 `BioXPMainWindow.initializeSystem(bool skipInitializeMotion = false)`

File:
`.../decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

Lines: 1046-1342

Observed behavior:

- sets `m_abortJob = false`;
- returns immediately if `m_systemInmotion` is already true;
- sets `m_systemInmotion = true` and clears it in `finally`;
- selects inspection/status message sequence based on `StartMode` and `SelfTest`;
- handles `ClassStatusLog.ShipMode == "PARK"`;
- calls `m_control.initialCheck()`;
- if saved status indicates unexpected shutdown (`SavedStatus == 3 || SavedStatus == 4`), takes a recovery branch: warning, `initializeMotion()`, `inspectCover()`, then door warning/return;
- otherwise shows initializing screen and calls `m_control.initializeMotion()`;
- attaches pipette error handler;
- optionally runs self-test;
- optionally checks camera with `CheckCamera && CameraInstalled && !IsDevelopmentMachine()`;
- calls `m_control.inspectCover()`;
- handles inspection errors by parking gantry, unlocking door, showing specific errors;
- calls `m_control.parkGantry(false)`;
- branches by `StartMode`:
  - `0`: door open, show main menu/trade surface, ready;
  - `3`: door open, show trade show page;
  - `1`/`2`: door open, network wait/check, prepare/load job path;
- posts images/logs/config files to AWS in some modes;
- maps exceptions to error situations and logs stack traces;
- always clears `m_systemInmotion` in `finally`.

Linux implication:

- `initializeSystem` must be a worker command with a non-reentrant `system_in_motion` lock;
- it must own the full startup-to-ready branch, not only homing;
- each OEM substage must be represented and either implemented or explicitly fail-closed;
- exception handling must map to durable error state and operator-action state;
- worker must reset `system_in_motion` even on failure.

### 1.3 `BioXPMainWindow.motion_thread_process()`

File:
`.../decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

Lines: 2030-2100

Observed behavior:

```csharp
while (true) {
    motionCommands motionCommands = m_commandQueue.Take();
    m_control.GantryAvailable = false;
    switch (motionCommands.name) {
        case "initializeSystem": ... initializeSystem(); ClassStatusLog.setStatus(1, true); break;
        case "unlockProcess": unlockProcess(); ClassStatusLog.setStatus(1, true); break;
        case "PrepareToRunJob": ... PrepareToRunJob(); break;
        case "abortjob": abortjob(...); ClassStatusLog.setStatus(1, true); break;
        case "validateJob": validateJob(); break;
        case "wakefrompause": wakefrompause(); ClassStatusLog.setStatus(3, true); break;
    }
    m_control.GantryAvailable = true;
}
```

Linux implication:

- the runtime must have an always-on queue consumer;
- every motion-affecting command must go through this worker;
- `GantryAvailable=false` must bracket command execution;
- command vocabulary must include all OEM command names above;
- queue state, active command, and history must be durable/artifacted.

### 1.4 `BioXPMainWindow.m_canControl_handleEnclosureDoorEventProcess(bool dooropen)`

File:
`.../decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

Lines: 2428-2512

Observed behavior:

- locks `m_dooropencloselock`;
- if `UserPaused`:
  - on close: change UI to Resuming and enqueue `wakefrompause`;
  - on open: change UI to Close Door;
  - return;
- if door opens:
  - call `initialCheck()`;
  - call `forceAbortMotion(false)`;
  - branch on `SetDoorOpen`, `m_odeevent`, `LatestStatus`, and `queryDoorStatus()`;
  - may unlock door and show ready/warning;
- if door closes:
  - call `initialCheck()`;
  - if door/latch closed: reset retry, enqueue `initializeSystem`;
  - if enclosure not closed: unlock door, increment retry, log latch failure, show wait warning;
  - else show ready.

Linux implication:

- door events are asynchronous runtime events, not only request bodies;
- open and close events have distinct semantics;
- door open during active operation must force abort motion;
- door close while paused must queue `wakefrompause`, not `initializeSystem`;
- door close while waiting initialization must queue `initializeSystem`;
- latch retries and operator-action states must be tracked.

### 1.5 `ControlLib.initialCheck()` and `queryDoorStatus()`

File:
`/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ControlLib.cs`

Lines: 8728-8770

Observed behavior:

- waits for `CAN_READY` in 200 ms increments up to `num > 10`;
- in board-test mode: activate board and return true;
- otherwise:
  - set LED white;
  - sleep 50 ms;
  - call `checkDoorStatus()`;
  - if false, return false;
  - deactivate board;
  - activate board;
  - return true;
- `queryDoorStatus()` returns true only if door sensor + latch sensor sum to 2.

Linux implication:

- `initialCheck` is not a read-only status call in live mode;
- shadow mode may be query-only, but live mode must perform the LED/board cycle or explicitly block;
- API must distinguish `initial_check_shadow` from live `initialCheck`.

### 1.6 `ControlLib.initializeMotion()`

File:
`.../decompiled_src/BioXPControlLib/ControlLib.cs`

Lines: 8797-8856

Observed behavior:

- set `m_stopScripts = true`;
- set `forceabort = false`;
- call `m_ControlInterface.initializeMotors()`;
- set thermal door status false;
- query pipette tip status;
- if tip exists:
  - open thermal door;
  - set thermal door open and tip loaded;
  - script move to tip/eject location;
  - update machine location;
  - eject all tips;
  - move Z and X;
  - query tip status again;
  - if tip still exists, pause scripts, emit error, throw;
  - otherwise mark tip clean/unloaded;
  - initiate group;
  - check pipette status, retry initiate/check, error if still false;
- if no tip exists, set TipLoaded false;
- catch errors and route through `errorEvent`.

Linux implication:

- `initializeMotion` is not just motor homing;
- post-home pipette cleanup/readback is a hard readiness gate;
- `initializeMotors` cannot mark startup ready without pipette/tip status and error handling.

## 2. Hard acceptance criteria

The runtime-app parity layer is not complete until all criteria below pass.

### 2.1 Runtime ownership criteria

- All motion-affecting live commands are routed through one runtime worker.
- Direct live motion endpoints either enqueue worker commands or reject while the worker owns motion.
- Worker has a durable queue journal.
- Worker has active command state with command id, source, parameters, start time, timeout, and artifact root.
- Worker brackets each command with `gantry_available=false` then `gantry_available=true`, including failure paths.
- Worker has restart recovery semantics.

### 2.2 OEM command vocabulary criteria

The runtime queue must support these OEM command names exactly, plus typed parameters:

- `initializeSystem`
- `unlockProcess`
- `PrepareToRunJob`
- `abortjob`
- `validateJob`
- `wakefrompause`

Command support must be explicit. Unknown command names must fail validation before queue insertion.

### 2.3 Event parity criteria

The runtime must ingest and journal at least these events:

- door opened;
- door closed;
- latch state changed;
- board error;
- control error;
- pipette error;
- CAN/backend ready/unready;
- USB disconnect/reconnect;
- emergency stop requested;
- user pause requested;
- user resume requested;
- worker command timeout.

Each event must have:

- event id;
- timestamp;
- source;
- raw payload when available;
- state before;
- state after;
- actions taken;
- artifact path.

### 2.4 Machine state criteria

Linux must expose a single authoritative OEM-shaped runtime state object, not multiple contradictory truth planes.

Required state fields:

- `machine_status.enclosure_door_closed`
- `machine_status.latch_closed`
- `machine_status.thermal_door_open`
- `machine_status.tip_loaded`
- `machine_status.tip_dirty`
- `machine_status.location`
- `gantry_available`
- `running_job`
- `user_paused`
- `system_in_motion`
- `abort_job`
- `stop_scripts`
- `pause_scripts`
- `force_abort`
- `latest_status`
- `saved_status`
- `door_close_retry`
- `start_mode`
- `ship_mode`
- `self_test_due`
- `camera_required`
- `camera_ready`
- `cover_inspection_status`
- `worker.state`
- `worker.active_command`
- `worker.queue_depth`
- `hardware.backend_ready`
- `hardware.usb_connected`
- `hardware.can_ready`
- `hardware.board_status`
- `motion_reference.axes`

`/status`, `/motion/power/status`, and OEM runtime status must be reconciled or explicitly identify their source and confidence.

### 2.5 Startup criteria

A live startup request is complete only after:

1. config/calibration binding loaded or explicit diagnostic override recorded;
2. backend/CAN ready confirmed;
3. live `initialCheck` passed;
4. door/latch closed or entered waiting-for-door state;
5. `initializeSystem` queued;
6. worker consumes `initializeSystem`;
7. `initializeMotion` completes;
8. `initializeMotors` completes with source/live-proven switch predicates;
9. pipette tip cleanup/readback gate completes;
10. camera check completes if configured;
11. cover inspection completes;
12. gantry park completes;
13. door open/ready transition completes for the active StartMode;
14. final state is ready with artifacts.

If any required substage is missing or unproven, final state must be `failed_closed` or `diagnostic_complete ready=false`, not ready.

### 2.6 Door event criteria

Door handling must match OEM branches:

- door open while paused: update operator state to close-door-required;
- door close while paused: enqueue `wakefrompause`;
- door open while not paused: run `initialCheck`, force abort motion, then branch on door/query/status state;
- door close while initialization is waiting: run `initialCheck`, enqueue `initializeSystem` if closed/latch closed;
- failed latch close increments retry and logs error;
- door/latch inconclusive states fail closed or enter operator-action required.

### 2.7 Error handling criteria

Every runtime error must map to an OEM-like error category and safe action.

Required error categories:

- board error;
- motion initialization error;
- door malfunction;
- initialization failure;
- job load error;
- server communication error;
- pipette error;
- camera initialization failure;
- cover inspection failure;
- emergency stop;
- command timeout;
- USB/CAN transport loss.

Required fields:

- `error_situation`;
- `message_id` if known;
- `operator_action_required`;
- `safe_action_taken`;
- `force_abort_motion_called` when applicable;
- `unlock_door_called` when applicable;
- `park_gantry_called` when applicable;
- `post_files_artifact_bundle` local equivalent.

### 2.8 Persistence criteria

The runtime must survive API restart without losing safety truth.

Required persistence:

- runtime state snapshot;
- command queue journal;
- active command record;
- command history;
- event journal;
- latest/saved status;
- startup session index;
- artifact roots;
- last emergency stop;
- last USB/CAN reconnect event.

On restart, the runtime must enter `recovery_required` unless it can prove no command was active and hardware is idle.

## 3. Target architecture

### 3.1 New/updated modules

Create or extend these robot-local modules.

#### `src/bioxp/oem_runtime_types.py`

Defines typed models/enums:

- `OEMRuntimeState`
- `OEMMachineStatus`
- `OEMWorkerState`
- `OEMCommandName`
- `OEMRuntimeCommand`
- `OEMRuntimeEvent`
- `OEMErrorSituation`
- `OEMOperatorAction`
- `OEMStartMode`
- `OEMSystemStatus`
- `OEMRuntimeSnapshot`

Command names must include:

```python
class OEMCommandName(str, Enum):
    INITIALIZE_SYSTEM = "initializeSystem"
    UNLOCK_PROCESS = "unlockProcess"
    PREPARE_TO_RUN_JOB = "PrepareToRunJob"
    ABORT_JOB = "abortjob"
    VALIDATE_JOB = "validateJob"
    WAKE_FROM_PAUSE = "wakefrompause"
```

#### `src/bioxp/oem_runtime_store.py`

Durable JSON/JSONL store:

- `runtime_state.json`
- `command_queue.jsonl`
- `command_history.jsonl`
- `event_journal.jsonl`
- `latest_status.json`
- `sessions/<session_id>/...`

Requirements:

- atomic JSON writes;
- append-only journals;
- startup recovery scan;
- no secrets/hostnames/keys in artifacts;
- root validated for live mode.

#### `src/bioxp/oem_runtime_worker.py`

Always-on command consumer.

Responsibilities:

- starts with API startup lifecycle;
- owns command queue;
- sets `gantry_available=false` before execution;
- dispatches command handlers;
- handles timeout/cancel/abort;
- sets `gantry_available=true` in finally;
- writes state and history;
- exposes worker heartbeat;
- refuses direct concurrent live motion.

#### `src/bioxp/oem_runtime_events.py`

Event ingestion and routing.

Responsibilities:

- door events;
- board errors;
- pipette errors;
- control errors;
- USB/CAN disconnect/reconnect;
- emergency stop;
- pause/resume;
- command timeout;
- worker lifecycle events.

#### `src/bioxp/oem_runtime_commands.py`

Command handlers:

- `handle_initialize_system`
- `handle_unlock_process`
- `handle_prepare_to_run_job`
- `handle_abortjob`
- `handle_validate_job`
- `handle_wakefrompause`

These handlers should call lower layers such as `oem_startup_program`, `usb_driver`, `oem_config`, pipette, vision, script/job engine, but own the OEM app-runtime branching.

#### `src/bioxp/oem_runtime_status.py`

Unifies status surfaces:

- machine status;
- worker status;
- hardware status;
- motion reference status;
- startup status;
- operator status;
- error status;
- confidence/source of each field.

#### `src/bioxp/oem_runtime_api.py`

FastAPI router exposing runtime controls.

Required routes:

- `GET /oem/runtime/status`
- `GET /oem/runtime/state`
- `GET /oem/runtime/events/latest`
- `GET /oem/runtime/commands/history`
- `GET /oem/runtime/worker/status`
- `POST /oem/runtime/commands/enqueue`
- `POST /oem/runtime/commands/initializeSystem`
- `POST /oem/runtime/commands/unlockProcess`
- `POST /oem/runtime/commands/PrepareToRunJob`
- `POST /oem/runtime/commands/abortjob`
- `POST /oem/runtime/commands/validateJob`
- `POST /oem/runtime/commands/wakefrompause`
- `POST /oem/runtime/events/door`
- `POST /oem/runtime/events/pause`
- `POST /oem/runtime/events/resume`
- `POST /oem/runtime/emergency_stop`
- `POST /oem/runtime/recover`

Existing `/oem/startup/*` routes may remain but must become thin wrappers over `/oem/runtime/commands/initializeSystem` and runtime event/state surfaces.

### 3.2 Updated existing modules

#### `src/bioxp/api.py`

- include `oem_runtime_api.router`;
- initialize background runtime worker at FastAPI startup;
- stop worker cleanly at shutdown;
- route live motion-affecting endpoints through runtime guard;
- make direct endpoint motion reject while runtime owns hardware.

#### `src/bioxp/oem_startup_program.py`

- become a command-helper layer, not owner of runtime;
- preserve current no-fake-ready gates;
- expose substage methods callable from `handle_initialize_system`;
- remove/manual-run-next as primary runtime path once background worker exists.

#### `src/bioxp/usb_driver.py`

- expose safe functions for runtime worker:
  - `oem_initial_check_live()`;
  - `oem_initialize_motion()`;
  - `oem_force_abort_motion()`;
  - `oem_unlock_door()`;
  - `oem_door_open()`;
  - `oem_park_gantry()`;
  - `oem_query_door_status()`;
- ensure no function silently claims OEM success without hardware/readback proof.

#### `scripts/bioxp_supervised_oem_app_startup.sh`

- call runtime enqueue/status endpoints;
- show worker/event state;
- never call direct homing endpoints;
- include timeout and emergency stop instructions.

## 4. Exact runtime state machine

### 4.1 Top-level runtime states

- `booting`
- `backend_unavailable`
- `idle_not_ready`
- `waiting_for_door_close`
- `initializing`
- `initialization_recovery_required`
- `ready_for_job`
- `job_validation_running`
- `job_preparation_running`
- `running_job`
- `user_paused`
- `resuming_from_pause`
- `aborting_job`
- `error_operator_action_required`
- `emergency_stopped`
- `recovery_required`
- `shutdown`

### 4.2 Worker states

- `not_started`
- `idle`
- `queued`
- `running`
- `timeout`
- `aborting`
- `failed`
- `recovering`
- `stopped`

### 4.3 Startup sub-states

- `config_loading`
- `config_loaded`
- `config_missing`
- `initial_check_before_door`
- `waiting_for_door_close`
- `door_close_observed`
- `initialize_system_queued`
- `initialize_system_running`
- `initial_check_inside_initialize_system`
- `unexpected_shutdown_recovery`
- `initialize_motion_running`
- `initialize_motors_running`
- `homing_z`
- `homing_gripper_clear`
- `homing_gripper`
- `homing_x`
- `parking_x_6000`
- `homing_y`
- `homing_door`
- `post_home_pipette_tip_query`
- `post_home_pipette_tip_eject`
- `post_home_pipette_status_check`
- `self_test_running`
- `camera_check_running`
- `cover_inspection_running`
- `gantry_parking`
- `door_opening_for_ready`
- `ready_for_job`

## 5. Command semantics

### 5.1 `initializeSystem`

Inputs:

- `mode`: `dry_run|shadow|live`
- `operator_ack`
- `artifact_root`
- `skipInitializeMotion`
- `start_mode`
- `require_config`
- `run_self_test`
- `allow_network_job_branches`

Preconditions:

- worker idle or command queued;
- live mode has operator ack and valid artifact root;
- if live, runtime is not emergency stopped;
- if live, no direct motion command active;
- config loaded or diagnostic override recorded;
- door/latch handling follows `initializeEnvironment`/door event state.

Execution sequence:

1. set `abort_job=false`;
2. if `system_in_motion=true`, return no-op/reject per OEM semantics;
3. set `system_in_motion=true`;
4. choose message sequence from StartMode/SelfTest;
5. handle ShipMode PARK;
6. if not `skipInitializeMotion`:
   - run `initialCheck`;
   - handle SavedStatus recovery branch;
   - run `initializeMotion`;
   - attach pipette error handler equivalent;
7. optional self-test;
8. optional camera check;
9. inspect cover;
10. park gantry;
11. StartMode branch;
12. log finished initialization;
13. post local artifact bundle equivalent;
14. finally set `system_in_motion=false`.

Acceptance:

- every substage writes artifact;
- live readiness is false unless all required substages pass;
- exceptions map to error state and clear `system_in_motion`.

### 5.2 `unlockProcess`

OEM source: handled by motion thread, implementation to be traced further before live.

Required Linux behavior:

- must be queued command;
- must set gantry unavailable while running;
- must call source-anchored unlock process or fail closed;
- must update status to ready if successful.

### 5.3 `PrepareToRunJob`

Required Linux behavior:

- queued command;
- validates reagent barcode when StartMode requires;
- calls `PrepareToRunJob` equivalent when source-anchored;
- parks gantry on appropriate branches;
- updates job/material state;
- no fake success without script/deck/pipette readiness.

### 5.4 `abortjob`

Required Linux behavior:

- queued command unless emergency stop path is used;
- maps warning situation parameter;
- calls force abort/stop-script semantics;
- clears or preserves state per OEM branch;
- logs operator-action state.

### 5.5 `validateJob`

Required Linux behavior:

- queued command;
- waits/checks network state if StartMode requires;
- validates script/job material availability;
- on failure unlocks door and sets operator-action status;
- no live execution without deck/script/pipette proof.

### 5.6 `wakefrompause`

OEM source: lines 2103-2129.

Observed behavior:

- `m_control.initialCheck()`;
- `m_control.rehome()`;
- UI changes pause button to Continue and enabled;
- ClassStatusLog set to status 3 in motion thread.

Required Linux behavior:

- queued command;
- live requires source/live-proven rehome semantics;
- if predicates unknown, fail closed and keep paused/recovery-required;
- update pause/resume operator state.

## 6. Door/latch event semantics

### 6.1 Required event API

`POST /oem/runtime/events/door`

Payload:

```json
{
  "door_open": false,
  "door_closed": true,
  "latch_closed": true,
  "source": "hardware|operator|test",
  "raw": {},
  "artifact_root": "/tmp/bioxp-live-runs/..."
}
```

### 6.2 Door close while waiting initialization

Expected:

- lock door-event mutex;
- call `initialCheck`;
- if door/latch closed: reset retry and enqueue `initializeSystem`;
- if door not closed: unlock door, increment retry, warning state;
- else ready state.

### 6.3 Door open while not paused

Expected:

- call `initialCheck`;
- call `forceAbortMotion(false)`;
- if not set-door-open or ODE event, show warning/operator action;
- if latest status is running/initializing, show initialization warning;
- if queryDoorStatus true, unlock door;
- show ready only after safe branch.

### 6.4 Door events while paused

Expected:

- door close queues `wakefrompause`;
- door open sets operator state close-door-required;
- no initializeSystem queue while paused.

## 7. Status/log parity

### 7.1 Runtime status endpoint

`GET /oem/runtime/status` returns:

```json
{
  "ok": true,
  "runtime_state": "ready_for_job",
  "worker": {
    "state": "idle",
    "gantry_available": true,
    "queue_depth": 0,
    "active_command": null
  },
  "machine_status": {
    "enclosure_door_closed": true,
    "latch_closed": true,
    "thermal_door_open": false,
    "tip_loaded": false,
    "tip_dirty": false,
    "location": null
  },
  "status_log": {
    "latest_status": "ready",
    "saved_status": "ready",
    "source": "oem_runtime_store"
  },
  "operator": {
    "action_required": false,
    "message_id": null,
    "error_situation": null
  },
  "hardware": {
    "backend_ready": true,
    "can_ready": true,
    "usb_connected": true,
    "board_status": {}
  },
  "confidence": {
    "door_latch": "hardware_readback",
    "motion_reference": "live_verified|unknown",
    "pipette": "live_readback|unproven",
    "vision": "live_artifact|unproven"
  }
}
```

### 7.2 Logs

Required journals:

- `runtime_events.jsonl`
- `runtime_commands.jsonl`
- `runtime_errors.jsonl`
- `status_transitions.jsonl`
- `hardware_snapshots.jsonl`

Each line must include a monotonic sequence number.

## 8. API contract

### 8.1 Runtime routes

Add `src/bioxp/oem_runtime_api.py` and include it under `/oem/runtime`.

Required routes and behavior:

| Route | Behavior |
|---|---|
| `GET /oem/runtime/status` | authoritative runtime summary |
| `GET /oem/runtime/state` | full state snapshot |
| `GET /oem/runtime/events/latest?limit=50` | latest event journal rows |
| `GET /oem/runtime/commands/history?limit=50` | latest command history rows |
| `GET /oem/runtime/worker/status` | worker queue/active/heartbeat |
| `POST /oem/runtime/commands/enqueue` | validate and enqueue typed command |
| `POST /oem/runtime/commands/initializeSystem` | convenience enqueue for initializeSystem |
| `POST /oem/runtime/commands/unlockProcess` | convenience enqueue |
| `POST /oem/runtime/commands/PrepareToRunJob` | convenience enqueue |
| `POST /oem/runtime/commands/abortjob` | convenience enqueue |
| `POST /oem/runtime/commands/validateJob` | convenience enqueue |
| `POST /oem/runtime/commands/wakefrompause` | convenience enqueue |
| `POST /oem/runtime/events/door` | handle door event branch |
| `POST /oem/runtime/events/pause` | set pause/request pause |
| `POST /oem/runtime/events/resume` | close-door/resume semantics |
| `POST /oem/runtime/emergency_stop` | out-of-band emergency stop path |
| `POST /oem/runtime/recover` | restart recovery procedure |

### 8.2 Existing route migration

Existing route | Required future behavior
---|---
`POST /oem/startup/request` | thin wrapper over enqueue `initializeSystem`/startup state, not direct executor
`POST /oem/startup/door_event` | thin wrapper over `/oem/runtime/events/door`
`GET /oem/startup/status/*` | derived from runtime state/session journal
`POST /oem/motion_worker/run_next` | dev/test only; not live/operator path once background worker exists
`POST /motion/*` live routes | must reject or enqueue if runtime worker owns motion

## 9. Testing plan

### 9.1 New test files

Create:

- `tests/test_oem_runtime_types.py`
- `tests/test_oem_runtime_store.py`
- `tests/test_oem_runtime_worker.py`
- `tests/test_oem_runtime_events.py`
- `tests/test_oem_runtime_commands.py`
- `tests/test_oem_runtime_status.py`
- `tests/test_oem_runtime_api.py`
- `tests/test_oem_runtime_recovery.py`
- `tests/test_oem_runtime_door_parity.py`
- `tests/test_oem_runtime_initialize_system_parity.py`

### 9.2 Required tests

#### Command vocabulary

- unknown command rejects before queue insertion;
- all OEM command names validate;
- command parameters persist to journal.

#### Worker ownership

- worker sets `gantry_available=false` during command;
- worker restores `gantry_available=true` after success;
- worker restores `gantry_available=true` after exception;
- direct live motion is rejected while worker active;
- command timeout records error and safe state.

#### Startup parity

- `initializeEnvironment` with closed door/latch enqueues `initializeSystem`;
- open door enters waiting/operator state;
- `initializeSystem` respects `system_in_motion` guard;
- exception clears `system_in_motion`;
- SavedStatus 3/4 enters recovery branch;
- ShipMode PARK branch does not proceed to homing;
- StartMode 0 opens door and ready state after successful gates;
- StartMode 1/2 requires network/job branch readiness or fails closed.

#### Door event parity

- door close while waiting queues `initializeSystem`;
- door close while paused queues `wakefrompause`;
- door open while active calls force abort;
- failed latch close increments retry;
- door event outside valid states records warning/error state.

#### initializeMotion parity

- initializeMotion calls initializeMotors before pipette cleanup;
- tip-exists branch requires eject/readback;
- failed eject maps pipette error and pauses/fails;
- no-tip branch sets TipLoaded false;
- camera required branch blocks without camera proof;
- inspectCover failure parks gantry/unlocks door/fails.

#### Persistence/recovery

- active command before simulated restart causes `recovery_required`;
- idle worker before restart recovers idle state;
- queue journal replays pending commands only when safe;
- artifacts are not lost across restart.

#### API tests

- `/oem/runtime/status` returns unified status;
- `/oem/runtime/commands/initializeSystem` enqueues command;
- `/oem/runtime/events/door` triggers branch-specific queueing;
- live command requires ack/artifact root;
- emergency_stop can be called even while worker active.

### 9.3 Validation commands

Run for each implementation slice:

```bash
python3 -m py_compile src/bioxp/oem_runtime_types.py src/bioxp/oem_runtime_store.py src/bioxp/oem_runtime_worker.py src/bioxp/oem_runtime_events.py src/bioxp/oem_runtime_commands.py src/bioxp/oem_runtime_status.py src/bioxp/oem_runtime_api.py src/bioxp/api.py
python3 -m pytest tests/test_oem_runtime_types.py tests/test_oem_runtime_store.py tests/test_oem_runtime_worker.py tests/test_oem_runtime_events.py tests/test_oem_runtime_commands.py tests/test_oem_runtime_status.py tests/test_oem_runtime_api.py tests/test_oem_runtime_recovery.py tests/test_oem_runtime_door_parity.py tests/test_oem_runtime_initialize_system_parity.py -q
git diff --check
python3 -m pytest tests -q
```

Expected final suite after implementation: all tests pass. Existing 196-test baseline must not regress.

## 10. Implementation roadmap

### Phase 1: Runtime model and durable store

Objective:
Create the state/command/event models and durable store with no hardware access.

Files:

- Create `src/bioxp/oem_runtime_types.py`
- Create `src/bioxp/oem_runtime_store.py`
- Create `tests/test_oem_runtime_types.py`
- Create `tests/test_oem_runtime_store.py`

Acceptance:

- command names validate exactly;
- state snapshots write atomically;
- journals append with sequence numbers;
- recovery scan distinguishes idle vs active command.

### Phase 2: Background worker shell

Objective:
Implement an always-on worker that consumes OEM commands and owns gantry availability.

Files:

- Create `src/bioxp/oem_runtime_worker.py`
- Create `tests/test_oem_runtime_worker.py`

Acceptance:

- worker runs in background;
- queue insert wakes worker;
- active command state is persisted;
- gantry availability bracketing is guaranteed;
- timeout and exception paths fail closed.

### Phase 3: Runtime API

Objective:
Expose runtime as API while keeping API thin.

Files:

- Create `src/bioxp/oem_runtime_api.py`
- Modify `src/bioxp/api.py`
- Create `tests/test_oem_runtime_api.py`

Acceptance:

- runtime routes exist;
- commands enqueue rather than execute inline;
- live commands require ack/artifact root;
- status route returns authoritative runtime state.

### Phase 4: Door/latch event parity

Objective:
Implement OEM door-open/door-close event branches.

Files:

- Create/modify `src/bioxp/oem_runtime_events.py`
- Modify `src/bioxp/oem_runtime_commands.py`
- Create `tests/test_oem_runtime_door_parity.py`

Acceptance:

- paused close queues `wakefrompause`;
- active open force-aborts;
- waiting close queues `initializeSystem`;
- latch failure increments retry;
- every branch writes event and state artifacts.

### Phase 5: initializeSystem command parity shell

Objective:
Move current startup shell into the runtime worker and implement the OEM branches as first-class states.

Files:

- Create/modify `src/bioxp/oem_runtime_commands.py`
- Modify `src/bioxp/oem_startup_program.py`
- Create `tests/test_oem_runtime_initialize_system_parity.py`

Acceptance:

- `system_in_motion` guard works;
- `finally` clears it;
- StartMode/SelfTest/ShipMode/SavedStatus branches are represented;
- `initializeMotion` is called as substage;
- missing substage proof fails closed.

### Phase 6: initializeMotion live gates

Objective:
Implement `ControlLib.initializeMotion()` semantics through live/shadow gates.

Files:

- Modify `src/bioxp/usb_driver.py`
- Modify `src/bioxp/oem_runtime_commands.py`
- Extend pipette and vision modules as needed
- Add tests for tip cleanup/camera/cover branches

Acceptance:

- initializeMotors called through worker only;
- pipette tip query/eject/readback required;
- failed pipette cleanup maps to pipette error;
- camera check required when config says so;
- inspectCover failure parks/unlocks/fails.

### Phase 7: Pause/resume/abort/job command parity

Objective:
Implement the remaining OEM queue command vocabulary.

Files:

- Modify `src/bioxp/oem_runtime_commands.py`
- Add tests per command

Acceptance:

- `wakefrompause` runs initialCheck + rehome gate;
- `abortjob` differs from emergency stop and maps warning situation;
- `validateJob` and `PrepareToRunJob` fail closed until script/deck/job proof exists;
- `unlockProcess` is source-anchored or blocked.

### Phase 8: State/status unification

Objective:
Make `/status`, `/motion/power/status`, and OEM runtime status non-contradictory.

Files:

- Create/modify `src/bioxp/oem_runtime_status.py`
- Modify `src/bioxp/api.py`
- Add status tests

Acceptance:

- runtime status identifies source/confidence;
- contradictory hardware connection fields are explained or eliminated;
- operator can see one authoritative ready/not-ready truth.

### Phase 9: Restart recovery and emergency intervention

Objective:
Fix the class of bug where a hung request owns the USB handle and emergency kill cannot attach.

Files:

- Modify worker/store/API/emergency scripts
- Add recovery tests

Acceptance:

- command timeout triggers abort/recovery state;
- emergency stop has out-of-band priority;
- restart after active command enters recovery_required;
- recovery requires safe hardware snapshot before re-enabling commands.

### Phase 10: Live supervised proof ladder

Objective:
Prove runtime parity safely on robot.

Acceptance:

- dry-run worker startup trace passes;
- shadow startup reads statuses only;
- live `initialCheck` passes with artifacts;
- live initializeSystem stops before homing if predicates unproven;
- once predicates proven, stepwise homing runs via worker only;
- pipette/vision/park gates proven or remain fail-closed;
- final ready state only after all gates pass.

## 11. Definition of done

The API/runtime parity requirement is done only when:

- Linux has an always-on OEM runtime worker;
- every live motion path is worker-owned;
- all OEM command names from `motion_thread_process` are represented;
- door/latch events match OEM branches;
- `initializeSystem` and `initializeMotion` branches are implemented or explicitly fail-closed;
- status is unified and restart-durable;
- errors map to OEM-like error situations and safe actions;
- no route can return ready while pipette/vision/motion/door gates are unproven;
- full tests pass;
- live supervised proof artifacts exist for each claimed live behavior.

## 12. Explicit non-goals

These are not required for this parity layer unless later promoted:

- WPF visual pixel parity;
- cloud/AWS production connectivity with real credentials;
- bypassing safety interlocks;
- generic movement from locked/rest pose;
- fake pipette/camera success for convenience;
- replacing the OEM-shaped runtime with a simpler stateless REST controller.

## 13. Current gap summary at spec creation

Current Linux has:

- startup shell;
- manual motion worker/run-next surface;
- initialCheck wrapper;
- config parser/search;
- switch audit and predicate gating;
- fail-closed post-home gates;
- API endpoints and tests.

Current Linux lacks, relative to this spec:

- background always-on worker;
- full command vocabulary;
- durable runtime store/recovery;
- true asynchronous event bus;
- door-open and paused-door branches;
- status unification;
- full initializeSystem body;
- full initializeMotion live gates;
- StartMode/SelfTest/ShipMode/SavedStatus handling;
- pause/resume/abort/job command parity;
- live-proven pipette/vision/park/readiness.

Therefore API/runtime is not complete until this spec is implemented.
