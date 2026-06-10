# BioXP3200 OEM Startup Native Compliance Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Build a robot-local Linux-native startup program that mirrors the OEM Windows BioXP app’s full startup/door-close/initializeSystem sequence on command, with source-anchored checks, durable artifacts, fail-closed safety, and no fake hardware success.

**Architecture:** Add an OEM-shaped startup state machine and serialized motion command worker above the existing low-level `BioXpTester` primitives. The program must model `GenBotApp -> ControlLib -> ClassControlInterface` semantics explicitly: config load, configure-without-motion, initialCheck, wait for enclosure door/latch close, queue `initializeSystem`, run `initializeMotion`, run `initializeMotors`, then perform post-homing pipette/inspection/park/ready stages. BMS remains a thin proxy/operator surface after robot-local truth exists.

**Tech Stack:** Python/FastAPI robot-local runtime in `/home/dalab/Desktop/BioXP 3200 Development Work/bioxp_re`; pytest; existing `src/bioxp/api.py`, `src/bioxp/usb_driver.py`, and `src/bioxp/oem_compat/*`; JSON artifact logs under `/tmp/bioxp-live-runs` or caller-supplied artifact roots.

---

## 0. Compliance definition

Native OEM startup compliance means Linux can execute the OEM Windows application’s startup sequence on command with equivalent semantics and stronger proof artifacts. It does not mean reproducing the WPF UI.

A compliant startup run must show this logical path:

```text
request/session created
-> OEM config binding loaded
-> ControlLib construction checks represented
-> CAN/USB backend ready
-> pipette init/status checked or explicitly gated
-> initializeMotorsWithoutMotion source-shaped configure-only stage
-> initializeEnvironment source-shaped gate
-> initialCheck source-shaped stage
-> if door/latch not closed: waiting_for_door_close, no motion
-> door/latch close observed
-> initialCheck repeated
-> initializeSystem queued on motion worker
-> initializeSystem running
-> initializeMotion running
-> initializeMotors running
-> Z/G/X/Y/door startup homing with switch traces
-> post-home tip cleanup or fail-closed unavailable
-> cover/camera inspection or fail-closed unavailable
-> gantry park / door-open or unlock behavior per StartMode/config
-> final ready or final failed_closed reason
```

Compliance is source/trace anchored to these OEM methods:

- `BioXPMainWindow.initializeEnvironment()`
- `BioXPMainWindow.m_canControl_handleEnclosureDoorEventProcess(...)`
- `BioXPMainWindow.motion_thread_process()`
- `BioXPMainWindow.initializeSystem(...)`
- `ControlLib.initialCheck()`
- `ControlLib.initializeMotion()`
- `ClassControlInterface.initializeMotorsWithoutMotion()`
- `ClassControlInterface.initializeMotors()`
- `ClassBioXPSettings` `config.xml` load/save paths

## 1. Non-negotiable safety and honesty requirements

1. Default mode is dry-run/status-only unless `mode=live`, `operator_ack="INITIALIZE"`, and `artifact_root` are provided.
2. A live run must fail closed if the artifact root is invalid, missing, or not writable.
3. A live run must never report `ready=true` unless every required source-shaped stage passes.
4. Door/latch not closed is not a software error; it is a `waiting_for_door_close` state with no motion.
5. Motion/homing must run only through a single serialized motion worker, not a long blocking HTTP request that owns USB invisibly.
6. Emergency kill must stay out-of-band and must not require the motion worker to cooperate.
7. No software bypass of enclosure/latch/interlock checks for live motion.
8. No unanchored direction hacks. The current Z reversal experiment must be quarantined as diagnostic until switch/direction predicates are proven.
9. Physical observation and switch traces supersede TMCL ACK-only success.
10. Vision/pipette unavailable states must block readiness if the OEM config/mode requires them; they must not silently pass.
11. BMS must not own startup truth; it may only proxy and display robot-local results.

## 2. New modules and files

Create these primary modules:

```text
src/bioxp/oem_config.py
src/bioxp/oem_startup_types.py
src/bioxp/oem_startup_program.py
src/bioxp/oem_motion_worker.py
src/bioxp/oem_switch_audit.py
```

Create these tests:

```text
tests/test_oem_config_import.py
tests/test_oem_startup_program.py
tests/test_oem_motion_worker.py
tests/test_oem_initial_check.py
tests/test_oem_switch_audit.py
tests/test_oem_startup_api.py
```

Modify these existing files:

```text
src/bioxp/api.py
src/bioxp/usb_driver.py
scripts/bioxp_supervised_oem_startup_homing_stepwise.sh
```

Optional later BMS files, only after robot-local truth exists:

```text
/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py
/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts
/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx
```

## 3. Data contracts

### 3.1 Startup request

Endpoint:

```text
POST /oem/startup/request
```

Request model:

```python
class OemStartupRequest(BaseModel):
    mode: Literal["dry_run", "shadow", "live"] = "dry_run"
    operator_ack: Optional[str] = None
    artifact_root: Optional[str] = None
    require_config: bool = True
    door_policy: Literal["wait_for_closed", "fail_if_open", "already_closed"] = "wait_for_closed"
    run_homing: bool = True
    run_post_home: bool = True
    timeout_s: float = 300.0
```

Live validation:

- `mode == "live"` requires `operator_ack == "INITIALIZE"`.
- `mode == "live"` requires `artifact_root`.
- `artifact_root` must be an absolute path under `/tmp/bioxp-live-runs` or another explicitly configured allow-list root.
- `run_homing=true` requires startup state `initial_check_passed_after_door_close`.

### 3.2 Startup response

Immediate response should not block until the entire run completes. It should enqueue/start a session and return status:

```python
class OemStartupRequestResponse(BaseModel):
    ok: bool
    session_id: str
    status: str
    mode: str
    queued: bool
    artifact_root: str
    startup_status_url: str = "/oem/startup/status/{session_id}"
    source_anchors: list[str]
```

### 3.3 Startup status

Endpoint:

```text
GET /oem/startup/status/{session_id}
GET /oem/startup/status/latest
```

Status model:

```python
class OemStartupStatus(BaseModel):
    session_id: str
    mode: str
    state: str
    ready: bool
    failed: bool
    failed_closed: bool
    failure_reason: Optional[str]
    active_stage: Optional[str]
    completed_stages: list[str]
    pending_stages: list[str]
    door_latch: dict
    config: dict
    backend: dict
    motion_worker: dict
    axis_reference: dict
    pipette: dict
    vision: dict
    artifacts: dict
    source_anchors: dict[str, str]
```

### 3.4 Door event endpoint

Endpoint:

```text
POST /oem/startup/door_event
```

Purpose:

- In live mode, this records observed door/latch transition and re-runs `initialCheck`.
- In dry-run tests, this simulates OEM event flow.
- It must not directly perform homing; it only moves the state machine from `waiting_for_door_close` to `initializeSystem_queued` if gates pass.

### 3.5 Motion worker endpoints

```text
GET  /oem/motion_worker/status
POST /oem/motion_worker/abort
```

Worker status must expose:

- idle/busy/aborting/failed
- active command name
- active session_id
- active stage
- queue depth
- last command result
- emergency stop provenance if present

### 3.6 Initial check endpoint

```text
POST /oem/initial_check
```

This is both a reusable stage and standalone diagnostic. It must perform/query, source-shaped:

- backend/CAN ready wait
- LED white if live and allowed
- door/latch/24V snapshot
- board deactivate
- board activate
- final door/latch truth

### 3.7 Switch audit endpoint

```text
POST /oem/switch_audit
```

Request:

```python
class OemSwitchAuditRequest(BaseModel):
    axes: list[Literal["x", "y", "z", "g", "door"]] = ["x", "y", "z", "g", "door"]
    mode: Literal["status", "live_probe"] = "status"
    artifact_root: Optional[str] = None
```

Default `mode=status` must not move anything. It queries raw GAP/switch/position/speed/current state.

## 4. Artifact contract

Every startup session creates a root:

```text
/tmp/bioxp-live-runs/<timestamp>_OEM_APP_STARTUP_SEQUENCE_<session_id>/
```

Required files:

```text
startup_request.json
source_anchors.json
config_search.json
config_binding.json
backend_ready.json
control_lib_constructed.json
pipette_startup_check.json
initialize_motors_without_motion.json
initialize_environment.json
initial_check_before_door.json
door_wait.json
door_event.json
initial_check_after_door.json
motion_queue_events.jsonl
initialize_system.json
initialize_motion.json
initialize_motors_trace.jsonl
axis_switch_trace_z.jsonl
axis_switch_trace_g.jsonl
axis_switch_trace_x.jsonl
axis_switch_trace_y.jsonl
axis_switch_trace_door.jsonl
post_home_pipette_cleanup.json
vision_inspection.json
gantry_park.json
door_ready_state.json
final_readiness.json
failure.json
```

Notes:

- Files for skipped stages must exist with `skipped=true` and a reason.
- `failure.json` must exist for any failed-closed run.
- `final_readiness.json` must always exist unless process death occurs.
- Trace JSONL must be flush-written during live motion, not only at the end.

## 5. State machine

### 5.1 States

Define enum in `src/bioxp/oem_startup_types.py`:

```python
class OemStartupState(str, Enum):
    CREATED = "created"
    CONFIG_LOADING = "config_loading"
    CONFIG_LOADED = "config_loaded"
    CONFIG_MISSING = "config_missing"
    BACKEND_READY = "backend_ready"
    CONTROL_LIB_CONSTRUCTED = "control_lib_constructed"
    PIPETTE_CHECKED = "pipette_checked"
    MOTORS_CONFIGURED_WITHOUT_MOTION = "motors_configured_without_motion"
    INITIALIZE_ENVIRONMENT = "initialize_environment"
    INITIAL_CHECK_BEFORE_DOOR = "initial_check_before_door"
    WAITING_FOR_DOOR_CLOSE = "waiting_for_door_close"
    DOOR_CLOSE_OBSERVED = "door_close_observed"
    INITIAL_CHECK_AFTER_DOOR = "initial_check_after_door"
    INITIALIZE_SYSTEM_QUEUED = "initialize_system_queued"
    INITIALIZE_SYSTEM_RUNNING = "initialize_system_running"
    INITIALIZE_MOTION_RUNNING = "initialize_motion_running"
    INITIALIZE_MOTORS_RUNNING = "initialize_motors_running"
    HOMING_Z = "homing_z"
    HOMING_GRIPPER_CLEAR = "homing_gripper_clear"
    HOMING_GRIPPER = "homing_gripper"
    HOMING_X = "homing_x"
    PARKING_X_6000 = "parking_x_6000"
    HOMING_Y = "homing_y"
    HOMING_DOOR = "homing_door"
    SETTING_Y_HOME = "setting_y_home"
    POST_HOME_PIPETTE = "post_home_pipette"
    VISION_INSPECTION = "vision_inspection"
    PARKING_GANTRY = "parking_gantry"
    DOOR_READY = "door_ready"
    READY = "ready"
    FAILED_CLOSED = "failed_closed"
    ABORTED = "aborted"
```

### 5.2 Transitions

Allowed transitions:

```text
CREATED -> CONFIG_LOADING
CONFIG_LOADING -> CONFIG_LOADED | CONFIG_MISSING
CONFIG_MISSING -> FAILED_CLOSED unless require_config=false diagnostic mode
CONFIG_LOADED -> BACKEND_READY
BACKEND_READY -> CONTROL_LIB_CONSTRUCTED
CONTROL_LIB_CONSTRUCTED -> PIPETTE_CHECKED
PIPETTE_CHECKED -> MOTORS_CONFIGURED_WITHOUT_MOTION
MOTORS_CONFIGURED_WITHOUT_MOTION -> INITIALIZE_ENVIRONMENT
INITIALIZE_ENVIRONMENT -> INITIAL_CHECK_BEFORE_DOOR
INITIAL_CHECK_BEFORE_DOOR -> WAITING_FOR_DOOR_CLOSE | DOOR_CLOSE_OBSERVED | FAILED_CLOSED
WAITING_FOR_DOOR_CLOSE -> DOOR_CLOSE_OBSERVED on door/latch true
DOOR_CLOSE_OBSERVED -> INITIAL_CHECK_AFTER_DOOR
INITIAL_CHECK_AFTER_DOOR -> INITIALIZE_SYSTEM_QUEUED | FAILED_CLOSED
INITIALIZE_SYSTEM_QUEUED -> INITIALIZE_SYSTEM_RUNNING via worker
INITIALIZE_SYSTEM_RUNNING -> INITIALIZE_MOTION_RUNNING
INITIALIZE_MOTION_RUNNING -> INITIALIZE_MOTORS_RUNNING
INITIALIZE_MOTORS_RUNNING -> HOMING_Z -> HOMING_GRIPPER_CLEAR -> HOMING_GRIPPER -> HOMING_X -> PARKING_X_6000 -> HOMING_Y -> HOMING_DOOR -> SETTING_Y_HOME
SETTING_Y_HOME -> POST_HOME_PIPETTE
POST_HOME_PIPETTE -> VISION_INSPECTION
VISION_INSPECTION -> PARKING_GANTRY
PARKING_GANTRY -> DOOR_READY
DOOR_READY -> READY
any running state -> FAILED_CLOSED on failed gate
any running state -> ABORTED on emergency abort
```

## 6. Phase implementation plan

### Phase 1: Types, artifacts, and no-motion state machine skeleton

**Objective:** Create the startup session/status/artifact model with dry-run-only transitions through door wait and queue state.

**Files:**

- Create: `src/bioxp/oem_startup_types.py`
- Create: `src/bioxp/oem_startup_program.py`
- Test: `tests/test_oem_startup_program.py`

**TDD tasks:**

1. Test startup session creation writes `startup_request.json` and starts in `CREATED`.
2. Test dry-run with `require_config=false` reaches `WAITING_FOR_DOOR_CLOSE` when door is open.
3. Test dry-run door event transitions to `INITIALIZE_SYSTEM_QUEUED`.
4. Test missing artifact root in live mode returns validation failure.
5. Test live mode without `operator_ack="INITIALIZE"` fails before hardware calls.

**Implementation notes:**

- Inject a fake hardware provider into `OEMStartupProgram`; do not import or instantiate `BioXpTester` in unit tests.
- Artifact writes should be simple JSON helpers with atomic write pattern: write `file.tmp`, then rename.
- Use `time.strftime` plus short random suffix for session IDs.

**Validation:**

```bash
cd "/home/dalab/Desktop/BioXP 3200 Development Work/bioxp_re"
python3 -m pytest tests/test_oem_startup_program.py -q
```

Expected: new tests pass; no USB opened.

### Phase 2: OEM config search/import

**Objective:** Represent `ClassBioXPSettings` `config.xml` dependency explicitly and block live compliance when real config is missing.

**Files:**

- Create: `src/bioxp/oem_config.py`
- Test: `tests/test_oem_config_import.py`
- Modify: `src/bioxp/oem_startup_program.py`

**TDD tasks:**

1. Test `find_oem_config()` searches provided roots for `config.xml`.
2. Test parser extracts at least these fields if present:
   - `StartMode`
   - `GripperVersion`
   - axis limit fields for X/Y/Z/G
   - Z motor current/stall fields
   - thermal door constants
3. Test missing config returns `status="missing"` with searched roots.
4. Test `require_config=true` makes startup fail closed before hardware.
5. Test `require_config=false` allows diagnostic dry-run but records `config_missing=true`.

**Required search roots:**

```text
/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup
/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/.deploy
/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Users
/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/ProgramData
/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Program Files
/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/Program Files (x86)
```

**Validation:**

```bash
python3 -m pytest tests/test_oem_config_import.py tests/test_oem_startup_program.py -q
```

### Phase 3: Source-shaped `initialCheck()` wrapper

**Objective:** Add a named Linux implementation of OEM `ControlLib.initialCheck()`.

**Files:**

- Modify: `src/bioxp/usb_driver.py`
- Modify: `src/bioxp/oem_startup_program.py`
- Test: `tests/test_oem_initial_check.py`

**Required function:**

```python
def oem_initial_check(self, *, mode="shadow") -> dict:
    """Source-shaped ControlLib.initialCheck() equivalent.

    OEM anchors:
    - ControlLib.cs initialCheck lines 8728-8759
    - ControlLib.cs queryDoorStatus lines 8762-8770
    """
```

**Required output:**

```python
{
  "ok": True,
  "source_anchor": "ControlLib.initialCheck lines 8728-8759",
  "backend_ready": {...},
  "led_white": {...},
  "door_latch": {...},
  "deactivate_boards": {...},
  "activate_boards": {...},
  "checks": [...],
}
```

**TDD tasks:**

1. Fake tester proves order: backend wait -> LED white -> door snapshot -> deactivate -> activate.
2. Door/latch false gives `ok=false` but no motion.
3. Board activation failure gives `ok=false` and source-anchored detail.
4. Startup program uses this wrapper rather than ad-hoc latch/status checks.

**Live behavior:**

- `mode=shadow` may query but not write LED/boards unless explicitly allowed.
- `mode=live` may perform LED/deactivate/activate after operator-ack startup gate.

### Phase 4: FastAPI startup API skeleton

**Objective:** Expose command/status endpoints without live homing yet.

**Files:**

- Modify: `src/bioxp/api.py`
- Test: `tests/test_oem_startup_api.py`

**Endpoints:**

```text
POST /oem/startup/request
GET  /oem/startup/status/latest
GET  /oem/startup/status/{session_id}
POST /oem/startup/door_event
POST /oem/initial_check
```

**TDD tasks:**

1. Test live request without ack returns HTTP 409/422 and no hardware call.
2. Test dry-run request returns session_id and artifact_root.
3. Test status latest returns the same session.
4. Test door_event moves dry-run session from waiting to queued.
5. Test initial_check endpoint returns source_anchor.

**Validation:**

Use existing test pattern that avoids hardware import failures. If pyusb is missing, API import must still be safe for dry-run tests.

### Phase 5: Motion worker / OEM command queue

**Objective:** Replace long request-owned startup homing with OEM-shaped serialized motion command execution.

**Files:**

- Create: `src/bioxp/oem_motion_worker.py`
- Modify: `src/bioxp/oem_startup_program.py`
- Modify: `src/bioxp/api.py`
- Test: `tests/test_oem_motion_worker.py`

**Required command model:**

```python
class OemMotionCommand(BaseModel):
    command_id: str
    session_id: str
    name: Literal["initializeSystem", "abort", "diagnostic"]
    created_ms: int
    payload: dict = {}
```

**Worker rules:**

1. One active command at a time.
2. `initializeSystem` is the only route to `initializeMotion` and live `initializeMotors`.
3. HTTP request should return queued/running; it must not hide progress for minutes.
4. Worker writes `motion_queue_events.jsonl` continuously.
5. Abort moves active session to `ABORTED` or `FAILED_CLOSED` and disarms motion.
6. Emergency kill script remains separate and can kill/restart API if necessary.

**TDD tasks:**

1. Enqueue two commands: second waits until first completes.
2. Worker emits JSONL queue events.
3. Abort marks active command aborted.
4. Startup program reaches `INITIALIZE_SYSTEM_RUNNING` only through worker.

### Phase 6: Raw switch audit and predicate matrix

**Objective:** Stop guessing home polarity/direction. Build the diagnostic surface needed before live homing compliance.

**Files:**

- Create: `src/bioxp/oem_switch_audit.py`
- Modify: `src/bioxp/usb_driver.py`
- Modify: `src/bioxp/api.py`
- Test: `tests/test_oem_switch_audit.py`

**Endpoint:**

```text
POST /oem/switch_audit
```

**Status-only output per axis:**

```python
{
  "axis": "z",
  "board": 4,
  "motor": 1,
  "position": {...},
  "speed": {...},
  "gap9_left": {...},
  "gap10_right": {...},
  "home_query": {...},
  "switch_masks": {...},
  "current_params": {...},
  "oem_profile": {...},
  "interpreted": {
    "home_switch_candidate": null,
    "active_value_candidate": null,
    "confidence": "unknown"
  }
}
```

**TDD tasks:**

1. Status mode never calls move methods.
2. Output includes raw values without forcing interpretation.
3. Predicate matrix cannot be marked confident without explicit evidence.
4. Startup live homing refuses to run if required axis predicate confidence is unknown.

### Phase 7: Switch-polling homing primitive

**Objective:** Replace ACK/wait-stopped home logic with sensor-driven source-shaped homing.

**Files:**

- Modify: `src/bioxp/usb_driver.py`
- Test: `tests/test_oem_homing_predicates.py`

**New function:**

```python
def motor_oem_home_axis_switch_driven(
    self,
    axis_key: str,
    *,
    predicate: OemHomePredicate,
    startup: bool,
    timeout_s: float,
    artifact_writer: Optional[Callable[[dict], None]] = None,
) -> dict:
```

**Required behavior:**

1. Query before state.
2. If already on home switch and OEM rehome requires preclear, preclear away from switch.
3. Start movement in configured OEM direction.
4. Poll speed, position, GAP9, GAP10, home query.
5. Stop immediately on correct home predicate.
6. Stop/fail immediately on wrong limit predicate.
7. Stop/fail on ambiguous no-motion.
8. Stop/fail on timeout/runaway.
9. Set home only after verified predicate.
10. Write JSONL trace rows during movement.

**TDD tasks:**

1. Correct switch hit stops and sets home.
2. Wrong switch hit stops and fails closed.
3. Timeout stops and fails closed.
4. No-motion fails closed.
5. `set_home` is not called before predicate verified.
6. Z direction hack is not used when predicate matrix supplies direction.

### Phase 8: Source-shaped `initializeMotors()` stage

**Objective:** Execute OEM startup homing order as one worker-controlled stage with per-axis artifacts.

**Files:**

- Modify: `src/bioxp/oem_startup_program.py`
- Modify: `src/bioxp/usb_driver.py`
- Test: `tests/test_oem_initialize_motors_stage.py`

**Order:**

```text
initializeMotorsWithoutMotion
Z home
G current 31
G move +10000 clear
G home
X home
X setHome
X speed 1700
X move absolute 6000
Y home
door home/search home
Y setHome
```

**Rules:**

- Abort sequence on first ambiguous/failed physical-motion step.
- Do not continue to later axes because controller ACKs look OK.
- Each axis writes `axis_switch_trace_<axis>.jsonl`.
- Final stage writes `initialize_motors_trace.jsonl` and summary JSON.

### Phase 9: `initializeMotion()` post-homing parity shell

**Objective:** Mirror the OEM `ControlLib.initializeMotion()` post-homing behavior sufficiently to block/allow readiness honestly.

**Files:**

- Modify: `src/bioxp/oem_startup_program.py`
- Modify: pipette subsystem module(s), depending current layout
- Test: `tests/test_oem_initialize_motion_post_home.py`

**Required represented stages:**

1. `queryTipStatus(-1)` equivalent.
2. If tips loaded/exist, execute or fail-closed the OEM cleanup branch:
   - open thermal door
   - move to waste
   - eject all tips
   - move Z safe
   - move X safe
   - recheck pipettes
3. If no tips loaded, mark `TipLoaded=false` equivalent.
4. Any unimplemented pipette cleanup must block final `ready=true` unless request explicitly sets diagnostic/no-post-home mode.

### Phase 10: Vision/inspection and ready-state parity shell

**Objective:** Represent OEM startup inspection/park/door/ready flow honestly.

**Files:**

- Modify: `src/bioxp/oem_startup_program.py`
- Modify: `src/bioxp/oem_compat/vision.py` or camera facade
- Test: `tests/test_oem_startup_ready_state.py`

**Required behavior:**

- If config says camera/inspection required, run inspection implementation or fail closed with `vision_unavailable`.
- If camera/inspection not required by config/mode, record skipped with source/config reason.
- Gantry park and door-ready operations must be separate named stages.
- Final `READY` only after all required stages complete.

### Phase 11: Operator script migration

**Objective:** Preserve operator control but route scripts through the new startup program.

**Files:**

- Modify: `scripts/bioxp_supervised_oem_startup_homing_stepwise.sh`
- Maybe create: `scripts/bioxp_supervised_oem_app_startup.sh`

**Script behavior:**

- Prompt `INITIALIZE` before live startup request.
- Submit `POST /oem/startup/request`.
- Poll `GET /oem/startup/status/{session_id}`.
- If state is `waiting_for_door_close`, print that state and wait/operator prompt.
- Never call old monolithic `/motion/arm/strict_startup` with homing.
- On timeout, call emergency kill helper and print artifact root.

### Phase 12: BMS proxy/display, only after robot-local pass

**Objective:** Add BMS operator visibility without moving logic into BMS.

**Files:**

- Modify: `/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py`
- Modify: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts`
- Modify: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`

**Routes:**

```text
/api/bioxp/oem/startup/request -> robot /oem/startup/request
/api/bioxp/oem/startup/status/latest -> robot /oem/startup/status/latest
/api/bioxp/oem/startup/status/{session_id} -> robot /oem/startup/status/{session_id}
/api/bioxp/oem/switch_audit -> robot /oem/switch_audit
```

**UI:**

- Stage timeline
- Door/latch wait state
- Ready/not-ready reason
- Artifact root link/path
- “BMS is display/proxy only; robot-local runtime owns hardware truth” label

## 7. Validation ladder

### Workstation dry-run validation

```bash
cd "/home/dalab/Desktop/BioXP 3200 Development Work/bioxp_re"
python3 -m pytest \
  tests/test_oem_config_import.py \
  tests/test_oem_startup_program.py \
  tests/test_oem_motion_worker.py \
  tests/test_oem_initial_check.py \
  tests/test_oem_switch_audit.py \
  tests/test_oem_startup_api.py \
  -q
```

Expected:

- all tests pass
- no USB opened
- artifacts written only under temp dirs

### Robot status-only validation

Run only safe endpoints:

```bash
curl -sS http://127.0.0.1:8123/oem/startup/status/latest
curl -sS -X POST http://127.0.0.1:8123/oem/switch_audit \
  -H 'Content-Type: application/json' \
  -d '{"mode":"status"}'
```

Expected:

- no motion
- raw switch/position/speed values captured
- config status explicit

### Robot dry-run startup validation

```bash
curl -sS -X POST http://127.0.0.1:8123/oem/startup/request \
  -H 'Content-Type: application/json' \
  -d '{"mode":"dry_run","require_config":false,"door_policy":"wait_for_closed","run_homing":true}'
```

Expected:

- session created
- no USB physical motion
- status reaches waiting/queued/runnable dry-run state
- artifacts exist

### Robot shadow validation

Shadow may query hardware but must not move axes:

```bash
curl -sS -X POST http://127.0.0.1:8123/oem/startup/request \
  -H 'Content-Type: application/json' \
  -d '{"mode":"shadow","require_config":false,"door_policy":"wait_for_closed","run_homing":false}'
```

Expected:

- config/backend/door/latch/raw switch evidence captured
- no homing/motion
- not ready if homing/post-home required

### Robot live supervised first pass

Preconditions:

- emergency kill helper verified
- operator physically present
- robot assembled enough for interlocks and OEM startup geometry
- config binding present or explicit diagnostic override chosen
- switch predicate matrix for Z/G/X/Y/door has confidence sufficient for live
- artifact root chosen

Command:

```bash
curl -sS -X POST http://127.0.0.1:8123/oem/startup/request \
  -H 'Content-Type: application/json' \
  -d '{
    "mode":"live",
    "operator_ack":"INITIALIZE",
    "artifact_root":"/tmp/bioxp-live-runs/manual_oem_startup_YYYYMMDD_HHMMSS",
    "require_config":true,
    "door_policy":"wait_for_closed",
    "run_homing":true,
    "run_post_home":true,
    "timeout_s":300
  }'
```

Expected:

- request returns quickly with session/status, not a hidden long block
- status shows `waiting_for_door_close` if door/latch are not closed
- after close, worker runs source-shaped stages
- switch traces are written during each axis motion
- on any failure, motors stop/disarm and final status is failed_closed
- final `ready=true` only if post-home pipette/vision/park/door gates pass

## 8. Git/commit strategy

Use small commits:

1. `docs: add OEM startup native compliance spec`
2. `feat: add OEM startup state model and artifact writer`
3. `feat: add OEM config search and binding loader`
4. `feat: add source-shaped OEM initial check`
5. `feat: expose OEM startup status API`
6. `feat: add OEM motion command worker`
7. `feat: add raw switch audit surface`
8. `feat: add switch-driven OEM homing primitive`
9. `feat: run initializeMotors via OEM startup worker`
10. `feat: add initializeMotion post-home readiness gates`
11. `feat: add supervised OEM app startup script`
12. `feat: proxy OEM startup status through BMS` only after robot-local validation

Do not mix unrelated BMS/Fold-CP/runtime dirty files into these commits.

## 9. Acceptance checklist

A run can be called “OEM startup compliant” only when:

- [ ] `config.xml` or equivalent binding is loaded and reported, or missing config blocks live readiness.
- [ ] Startup state machine exists and is tested.
- [ ] Door/latch close wait/event behavior is represented.
- [ ] `initialCheck()` is source-shaped and stage-logged.
- [ ] Motion worker serializes `initializeSystem`.
- [ ] HTTP request does not hide a long homing operation.
- [ ] `initializeMotorsWithoutMotion()` runs as configure-only stage.
- [ ] `initializeMotors()` runs in OEM order.
- [ ] Per-axis switch predicates are proven and recorded.
- [ ] No unanchored Z-direction hack remains.
- [ ] Homing writes switch traces during motion.
- [ ] Any ambiguous/no-motion/wrong-limit event fails closed.
- [ ] Pipette/tip startup path is represented.
- [ ] Vision/inspection path is represented or blocks readiness.
- [ ] Park/door/ready flow is represented.
- [ ] Final status is one coherent OEM startup truth record.
- [ ] Emergency kill remains out-of-band and verified.
- [ ] BMS only proxies/displays robot-local startup truth.

## 10. Immediate first implementation slice

Start with a no-motion PR:

1. Add `oem_startup_types.py`.
2. Add `oem_startup_program.py` with fake hardware provider support.
3. Add artifact writer.
4. Add config missing status stub.
5. Add dry-run door/latch wait transition.
6. Add API skeleton for request/status/door_event.
7. Add tests proving live requests cannot run without ack/artifact root.

This first slice produces the OEM-shaped shell without touching live homing. After that, add initialCheck and motion worker, then switch audit, then switch-driven homing.
