# BioXP 3200 Complete OEM Initialization Contract — Source-Only

**Date:** 2026-07-23
**Target:** BioXP 3200 serial 206
**Status:** specification / source-extraction contract only — **no runtime implementation authorization**

## 1. Governing rule and claim boundary

This document defines the entire OEM Windows initialization sequence that must be reconstructed before any claim of full BioXP initialization parity.

```text
Direct OEM binary/decode/configuration
        ↓
this contract
        ↓
Linux replacement implementation
        ↓
separately approved commissioning / physical proof
```

The current Linux handler, its tests, BMS controls, Linux observations, and generic motion practice are not behavioral authority. They may identify implementation defects, but may not fill an OEM-source gap.

**This document does not authorize:** deployment, handler restart, USB activation, `initializeSystem`, homing, movement, camera access, board cycle, reference write, solenoid action, or physical test.

## 2. Sealed direct-OEM authority set

> **Corpus-closure correction (2026-07-23):** The lower pipette/CAN, route-table,
> camera/cover, chiller, and selected serial-206 configuration artifacts were already
> present in the frozen SSD decode and the hash-locked live machine corpus. Earlier
> wording in this document that described those artifacts as missing or still requiring
> acquisition was wrong. The artifacts are available; the remaining work is exact
> line-level extraction/mapping, Linux implementation, and validation.

| OEM role | Source file | SHA-256 / status |
|---|---|---|
| Application startup / queue / terminal workflow | `decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs` | pinned source authority (`b288a45e…e904c`) |
| Control lifecycle / tip remediation / inspection / parking | `decompiled_src/BioXPControlLib/ControlLib.cs` | pinned source authority (`f69b3529…f6c2`) |
| Axis mapping, non-motion preparation, motor orchestration | `decompiled_src/BioXPControlLib/ClassControlInterface.cs` | pinned source authority (`86093e52…6e6e`) |
| Pipette collection status and tip ejection | `decompiled_src/BioXPControlLib/ClassPipetteCollection.cs` | `ffe3729fa35642d04ef6fe45501e52200dd7c2977a70902aa20e46fb26d4011e` |
| Per-pipette command bodies | `decompiled_src_can/BioXPControlLib/ClassPipette.cs` | `681f959cf527b060cece17b3cf7ff59c1ba1f5ead99fea53520d09486ac0c957` |
| CAN interface and Novo routing | `decompiled_src_can/ClassCanLib/InterfaceCAN.cs`, `ClassNovo.cs`; `decompiled_src_novo/NovoCANUSBLib/ClassNovoCANUSB.cs` | `aed90411d2966ae45142f8a988a2c6757011845c3ab2c5823c98090783419da1`; `11293074caec278076723666e69022b547c43f32b5fa886c99f75d5b60043d06`; `e4cf1c311ed5ae79e9490564a48947bced5f46af36890cf4a77120a3b50ffb06` |
| Machine-setting reader and field definitions | `decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs` | `08155dc24602bc12cf25af745c74cc478f33e0f2675fc0d4b6f2e1ba917d8d41`; reader anchors `2516-2709`, `2843-3524` |
| Head-axis board / Z-Y-gripper home primitive | `decompiled_src_can/ClassCanLib/ClassHeadBoard.cs` | `342a9b2f09731002194b67e37f1d4e866ecbfb3c25effd85b3cd609e8cbdd1ea` |
| Thermal door board / dedicated door-home primitive | `decompiled_src_can/ClassCanLib/ClassThermalBoard.cs` | `23d50725da200044422fde56b00611df708b514cef0f3637b2ad8d19e0b23f26` |
| Motor TMCL operations / switch semantics | `decompiled_src_can/ClassCanLib/ClassMotor.cs` | `9fb1b4bec771165053a82b4fe95510615d6ed9beda1a041280584ceb4ab7fe99` |
| Deck and chiller board activation/commands | `decompiled_src_can/ClassCanLib/ClassDeckBoard.cs`, `ClassChillerBoard.cs`, `ClassThermalControl.cs` | chiller sources sealed at `c048e2cdfcecc58c97f857e2c2bad85eb9766c2196981fa8fde67df7334981fd` and `fdefde9b38c70b9fec47e6ffdd4929ad8f25bc90c57c43baf03722addbe6cf25` |
| Camera and pattern-matching implementation | `decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs`; captured `CVisionLib.dll` | source `6eec22f02eae5b4738a6d857b57691009dacd5b1b9bb615d41e9d8a8e4d28501`; binary `84ab0c851f1bb418289035efd9fa84420e9ff82ea0a69ec0c00bb5d401e750f2` |
| Exact live serial-206 machine corpus | `OEM_EVIDENCE_LOCK.json`; runtime projection in `src/bioxp/oem_machine_bundle.py` | acquisition `20260719T024740Z-4a7fe6783205846c`; machine serial `206`; registry-selected current lock `a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c`; historical acceptance-package lock bytes were `148b224828fc2a0437897b63b352efa7cb80715df8045d8aa8e19d6d8e7cb1fa` |

A decompiled method is authoritative only through its mapped OEM binary. Any ambiguity in decompilation must be resolved against the captured DLL/IL before implementation.

## 3. OEM topology — direct code mapping, not replacement inference

`ClassControlInterface.m_AxisIODesignater` establishes the OEM logical routing:

| Logical key | OEM board-array slot | Axis | Concrete constructor / CAN board |
|---|---:|---:|---|
| `MotorX` | 1 | 0 | `ClassDeckBoard(..., 5, ...)` |
| `MotorY` | 0 | 0 | `ClassHeadBoard(..., 4, ...)` |
| `MotorZ` | 0 | 1 | `ClassHeadBoard(..., 4, ...)` |
| `MotorGrip` | 0 | 2 | `ClassHeadBoard(..., 4, ...)` |
| `ThermalDoor` | 2 | 0 | `ClassThermalBoard(..., 6, ...)` |
| `ThermalCycler` | 2 | 1 | `ClassThermalBoard(..., 6, ...)` |
| `OutputChiller` | 3 | 1 | `ClassChillerBoard(..., 7, ...)` |
| `ReagentChiller` | 3 | 0 | `ClassChillerBoard(..., 7, ...)` |

**Anchors:** `ClassControlInterface.cs:29-111,383-447`.

The runtime must bind this mapping from the OEM configuration/constructor record. It may not derive it from a Linux profile, generic axis naming, or a UI selection.

## 4. Complete application-level OEM call graph

```text
BioXPMainWindow startup
└─ initializeEnvironment()
   ├─ CAN_READY gate
   ├─ ControlLib.initialCheck()
   ├─ enclosure/latch branches
   └─ queue motionCommands { name = "initializeSystem" }
      └─ motion_thread_process()
         ├─ commandQueue.Take()
         ├─ GantryAvailable = false
         ├─ if name == "initializeSystem" and !UpdateCheck()
         │  └─ initializeSystem(skipInitializeMotion=false)
         └─ GantryAvailable = true

initializeSystem()
├─ re-entry guard m_systemInmotion
├─ ShipMode == "PARK" branch
├─ ControlLib.initialCheck()
├─ unexpected-shutdown SavedStatus branch
│  ├─ ControlLib.initializeMotion()
│  └─ ControlLib.inspectCover()
├─ normal ControlLib.initializeMotion()
├─ optional SelfTest() → ControlLib.selftest()
├─ conditional ControlLib.CheckCamera()
├─ ControlLib.inspectCover()
├─ ControlLib.parkGantry(false)
└─ StartMode-specific completion / job acquisition branches
```

### 4.1 `BioXPMainWindow.initializeEnvironment()` — pre-motion admission

**Anchor:** `BioXPMainWindow.cs:973-1027`.

1. Requires `m_control.m_canControl.CAN_READY`.
2. Calls `m_control.initialCheck()`.
3. Branches on enclosure and latch state:
   - enclosure open and latch open: warning;
   - enclosure open: `unlockDoor()`, increment close retry, wait-initialization warning;
   - enclosure closed **and** latch closed: enqueue exact command name `"initializeSystem"`;
   - otherwise: `unlockDoor()`, show ready state.
4. If CAN is not ready and `StartMode != 0`, returns. Otherwise selects main page and sets `GantryAvailable = true`.

**Implementation rule:** the already-completed Linux non-motion action ends before the queue submission. It must never imply that `initializeSystem` ran.

### 4.2 `BioXPMainWindow.motion_thread_process()` — sole OEM application dispatch boundary

**Anchor:** `BioXPMainWindow.cs:2030-2100`.

1. Blocks on `m_commandQueue.Take()`.
2. Sets `m_control.GantryAvailable = false` around dispatch.
3. On exact `motionCommands.name == "initializeSystem"`, calls `initializeSystem()` only when `!UpdateCheck()`.
4. Calls `ClassStatusLog.setStatus(system_status=1, true)` after that branch.
5. Restores `GantryAvailable = true` after switch dispatch.

**Implementation rule:** any replacement full-startup executor must be a single serialized robot-local worker. BMS cannot directly sequence axes or advance lifecycle state.

### 4.3 `BioXPMainWindow.initializeSystem(bool skipInitializeMotion=false)` — full top-level lifecycle

**Anchor:** `BioXPMainWindow.cs:1046-1341`.

| Order | OEM action / code name | Exact branch / consequence |
|---:|---|---|
| 1 | `m_systemInmotion` | Return immediately on re-entry; otherwise set true. Cleared in `finally`. |
| 2 | `ClassStatusLog.ShipMode == "PARK"` | `doorOpen(false,false)`, operator shutdown prompt, Windows shutdown, return. This is not normal initialization. |
| 3 | `m_control.initialCheck()` | Runs again before motion. |
| 4 | unexpected shutdown | If `SavedStatus` is 3 or 4: warning → `initializeMotion()` → `inspectCover()` → unlock/warning/return. It does **not** continue normal initialization. |
| 5 | normal motion | `showScreen`; `m_control.initializeMotion()`; rebind pipette error event. |
| 6 | optional self test | Only if `SelfTest == true` and elapsed since `SelfTestDate` exceeds 1 day; `SelfTest()` delegates to `m_control.selftest()`; false throws. |
| 7 | conditional camera | Only if `CheckCamera && CameraInstalled && !IsDevelopmentMachine()`; failure unlocks door, shows initialization failure, posts artifacts, returns. |
| 8 | cover inspection | `m_control.inspectCover()`; non-OK parks gantry, unlocks door, reports code-specific job-load error, returns. |
| 9 | park | `m_control.parkGantry(false)`. |
| 10 | StartMode 0 | `doorOpen(true,false)` then main page. |
| 11 | StartMode 3 | `doorOpen(true,false)` then trade-show page. |
| 12 | StartMode 1/2 | `doorOpen(true,false)`; StartMode 1 waits communication event up to 5000 ms; network failure unlocks/error; success calls `PrepareToRunJob`, parks, acquires job using local/AWS/Google branch. |
| 13 | terminal | Logs `Finished initialization`; AWS post-image/log/config branch under its stated host/start/job conditions. |
| 14 | exception | unlocks door; chooses motion-initialization error display by phase/message; logs stack trace. |

**OEM code names to implement line-for-line in this tranche:**
`initializeEnvironment`, `motion_thread_process`, `UpdateCheck`, `initializeSystem`, `initialCheck`, `initializeMotion`, `SelfTest`, `selftest`, `CheckCamera`, `inspectCover`, `parkGantry`, `doorOpen`, `unlockDoor`, `PrepareToRunJob`.

## 5. OEM non-motion prerequisite — preserve as a distinct stage

### 5.1 `ClassControlInterface.initializeMotorsWithoutMotion()`

**Anchor:** `ClassControlInterface.cs:3181-3265`.

This is not a generic passive status check. It performs board setup and writes settings before full motion:

1. `waitForBoard()`; `turnOffHeater()`; `setChillerPWM()`; sleep 1 ms.
2. X: speed/acc `1700/350`; max current 31; stall guard 16; 2 ms between writes.
3. Y: `1800/400`; current 31; stall guard 16; disable right switch; 2 ms boundaries.
4. Z: `1791/576`; `Z_MOTOR_MAX_CURRENT_UP`; read max current; `Z_MOTOR_STALL_GUARD_THRESHOLD`; 2 ms boundaries.
5. Gripper: version 0 → `600/5`, current 31, guard 5; version 1 → `1500/20`, current 10, guard 20; then `setRdivPdiv(6,2)`.
6. Thermal door: `TC_DOOR_VELOCITY/TC_DOOR_ACCELERATION`; `TC_DOOR_MAX_CURRENT`; `TCDoorStallGuardThreshold`; disable right then left switch; 2 ms boundaries.
7. `setChillerCoolRate("OC")`; `setChillerCoolRate("RC")`; thermal board heat rate 2.5 and cool rate -2.0; `setColor(255,255,255)`.

### 5.2 `ControlLib.initialCheck()`

**Anchor:** `ControlLib.cs:8728-8760`; door details at `8669-8726`.

1. Polls `CAN_READY` with 200 ms sleep, returns false after counter >10.
2. Board-test mode: result true and `activateBoard()` only.
3. Normal mode: set white LED; sleep 50 ms; call `checkDoorStatus()`; on failure return false; then `deactivateBoard()` → `activateBoard()`.

## 6. OEM physical initialization — `ControlLib.initializeMotion()`

**Anchor:** `ControlLib.cs:8797-8856`.

```text
m_stopScripts = true
forceabort = false
try:
  ClassControlInterface.initializeMotors()
  MachineStatus.ThermalDoorOpen = false
  PipetteControl.queryTipStatus(-1)
  sleep 500 ms
  if TipExist:
     openThermalDoor()
     ThermalDoorOpen = true
     TipLoaded = true
     scriptmoveTo(current Location=28/current Well=0 → target Location=6/Well=0)
     updateLocation(6,0)
     ejectAllTips(false,true)
     moveZ(80000)
     moveX(79000)
     queryTipStatus(-1)
     sleep 100 ms
     if TipExist: pause + errorEvent + throw
     TipDirty=false; TipLoaded=false
     sleep 2 ms
     initiateGroup(); checkedPipetteStatus(); retry once; error/throw if still bad
  else:
     TipLoaded=false
catch Exception:
  errorEvent(message); rethrow if event registered
```

**Critical consequence:** `initializeMotion()` includes physical homing and a physical stale-tip remediation sequence. It is not a diagnostic that can truthfully run with `run_homing=false`.

### Required direct OEM pipette bodies

| Code name | Direct OEM contract now sourced |
|---|---|
| `ClassPipetteCollection.queryTipStatus(int)` | `ClassPipetteCollection.cs:90-101,1336-1358`: aggregate query/TIP predicate; lower `ClassPipette.QueryTipStatus()` is present in `decompiled_src_can/BioXPControlLib/ClassPipette.cs`. |
| `ClassPipetteCollection.ejectAllTips(bool checkMissingTip=true, bool wait=true)` | `ClassPipetteCollection.cs:1176-1235`: selects active pipettes, queries tips, issues per-pipette `ejectTip`, waits up to 8000 ms when requested, checks missing-tip branch, calls `verifyEjectTip`, and resets `TipLocation=-1`. |
| `ClassPipetteCollection.initiateGroup()` | `ClassPipetteCollection.cs:677-693`: reinitializes completed channels, waits `Reinitialize pipette` up to 10000 ms, enables pressure stream 1000 ms, then calculates pressure offset. |
| `ClassPipetteCollection.checkedPipetteStatus()` | `ClassPipetteCollection.cs:726-748`: queries all four channels with 30 ms inter-command delays, sleeps 1 ms, then requires each error code to be zero. |
| `ClassControlInterface.openThermalDoor()` | `ClassControlInterface.cs:2651+`; exact body is direct source and must be carried into the thermal action tranche. |
| `ClassControlInterface.scriptmoveTo(...)` | Direct source exists; complete body and location-table dependencies must be line-locked before tip remediation/park implementation. |
| `ClassControlInterface.moveZ(int)` / `moveX(int)` | `ClassControlInterface.cs:4206+`, `4254+`; direct source exists; preserve current, limits, application adjustment, and wait behavior. |

## 7. OEM literal homing and closeout — `ClassControlInterface.initializeMotors()`

**Anchor:** `ClassControlInterface.cs:3348-3421`.

This is the authoritative ordered physical-homing sequence. No generic Home All, individual manual home, reference-return, or additional clearance belongs in this path unless separately documented as a Linux safety intercept.

| # | OEM direct call | Exact parameters / branch | Source continuation |
|---:|---|---|---|
| 1 | `MotorZ.axisSearchHome` | axis Z, speed **1791**, only if Z board non-null | next action is gripper current; no added Z reference move |
| 2 | `setGripperCurrent` | **31** | unconditional |
| 3 | `MotorGrip.moveSteps` | **+10000**, `waitforstop=true` | required before gripper home |
| 4 | `MotorGrip.axisSearchHome` | GV0: **600**; otherwise **200** | config field `GripperVersion` is authoritative |
| 5 | `MotorX.axisSearchHome` | **250** | sleep 20 ms |
| 6 | `MotorX.setHome` | X axis | `setSpeed(1700)` |
| 7 | `MotorX.setSpeed` | **1700** | sleep 40 ms |
| 8 | `moveX` | **6000** | next Y home |
| 9 | `MotorY.axisSearchHome` | **250** | next dedicated thermal-door home |
| 10 | `ThermalDoor.doorSearchHome` | `TC_DOOR_VELOCITY`, `TCDoorStallGuardThreshold` | next door predicate branch |
| 11 | `confirmAxis("tcDoorClosed")` | if `SerialNumber > 9 && !closed && CameraCalibrated`: `openThermalDoor(); throw "Cannot close thermal cycler door!"` | otherwise Y set home |
| 12 | `MotorY.setHome` | Y axis | calibration UI zeroing branch only |
| 13 | calibration status | if `Calibrated`, update diagnostic text boxes to zero | chiller rates |
| 14 | `setChillerCoolRate` | `"OC"`, then `"RC"`, default rate -0.025 | set status |
| 15 | `ClassStatusLog.setStatus` | `system_status=1`, true | version-1 current restoration |
| 16 | `setGripperCurrent` | **10 only if `GripperVersion == 1`** | return |

## 8. Direct OEM primitive contracts required by the homing sequence

### 8.1 Head axes (Y, Z, gripper): `ClassHeadBoard.axisSearchHome()`

**Anchor:** `ClassHeadBoard.cs:368-386`.

1. Throws on `ClassBaseBoard.m_24Vdropped`.
2. Returns 1 if board not initialized.
3. Calls motor `setHome()`.
4. Stores source search speed.
5. If `queryHome(axis)`: `moveToAbs(axis,10000)` then sleep **500 ms**.
6. Sets `MotorHome=false`.
7. Calls `goHome(rehome:false, axis, speed)`.

### 8.2 Head-axis `goHome()`

**Anchor:** `ClassHeadBoard.cs:60-119`.

- Throws on 24-V loss; returns 1 if uninitialized.
- Short-circuits when OEM host state says `MotorHome && CurrentPosition==0`.
- If `rehome`, first `moveToAbs(10000)`.
- Calls `moveLeft(axis,speed)`.
- With `waitforstop=true`: waits controller stop up to 30 seconds.
  - non-gripper timeout: `stopMotor(false)` then throw;
  - gripper timeout: recovery `moveToAbs(m_gripperposition[GripperStatus 4], ..., gripperRecover:true)`.
- Polls `queryHome` for up to the same 30-second window; when true: `stopMotor(false)`, records negative actual position, `setHome(axis)`.

### 8.3 Dedicated thermal-door home — not generic axis home

**Anchor:** `ClassThermalBoard.cs:320-366`.

1. 24-V loss throws; uninitialized board returns with no action.
2. Stores search speed.
3. If `queryHome(axis) || SerialNumber < 10`:
   - `setStallGuardThreshold(stallThread + 2)`;
   - `moveSteps(axis,+2000)`.
4. Restore nominal `setStallGuardThreshold(stallThread)`.
5. `moveLeft(axis,speed)`.
6. Counter starts 300. Each cycle evaluates home; if home and counter >81, sets counter 80; sleeps **50 ms**; decrements and logs timeout below zero.
7. Waits until `checkMotorStopped(axis)`.
8. Calls `stopMotor(axis)`.
9. Final branch:
   - home true → `setHome`;
   - `SerialNumber < 10` → `setHome`;
   - otherwise if `CameraCalibrated` → throw `"Failed to find door home"`;
   - otherwise return without throw.

### 8.4 Motor primitives

| OEM method | Anchor | Required source behavior |
|---|---|---|
| `ClassMotor.MoveLeft(speed)` | `ClassMotor.cs:74-115` | command 2/type 0/axis/speed; one transmit; host state resets; status 100 success. |
| `ClassMotor.StopMotor()` | `ClassMotor.cs:161-182` | command 3 emitted **twice**; second reply decides success and sets host stopped state. |
| `ClassMotor.MovetoRelPosition(steps)` | `ClassMotor.cs:231+` | relative command 4/type 1/signed steps. |
| `ClassHeadBoard.moveSteps` | `ClassHeadBoard.cs:231-286` | 24-V checks, initialized gate, limit check, controller-stop query, relative move, default wait/event timeout logic. |
| `ClassMotor.setHome()` | `ClassMotor.cs:492-516` | SAP parameter 1 / coordinate zero and OEM host-state update. |
| `ClassMotor.queryLeftSwitchStatus()` | `ClassMotor.cs:641-664` | GAP parameter 9; raw asserted mapping used by board `queryHome`. |
| `ClassMotor.queryRightSwitchStatus()` | `ClassMotor.cs:666-688` | GAP parameter 10; used by board `queryRightSensor`. |
| `ClassHeadBoard.queryHome/queryRightSensor` | `ClassHeadBoard.cs:389-414` | OEM board predicate semantics, including uninitialized behavior. |
| `ClassHeadBoard.checkMotorStopped` | `ClassHeadBoard.cs:417-433` | controller speed equals zero predicate. |

**Source-versus-safety rule:** a future Linux travel limit, reply-validity check, observed-motion check, or bounded-timeout interception may inhibit an OEM action, but must report `safety_intercepted=true` and may not be represented as an OEM branch or alter the OEM sequence.

## 9. Post-home OEM functions required for full initialization

These are inside the normal `initializeSystem` path and are mandatory parts of a complete spec, even though they are not homing.

| Function | Direct anchor | Required specification scope |
|---|---|---|
| `ControlLib.selftest()` | `ControlLib.cs:10688-10785`; runs TC/RC/OC self-tests concurrently, closes thermal door, homes Z/X/Y, drives X/Y to `SelfTestXMax/YMax` then `HomeXY`, drives Z to `SelfTestZMax`, checks gripper against `m_gripperposition[2]`, parks/unlocks, waits up to 100000 ms for subsystem completion, restores chiller PWM, and updates SelfTestDate only on pass. The direct `TCSelfTest`/`RCSelfTest`/`OCSelfTest` bodies and their `ClassChillerBoard`/`ClassThermalControl` dependencies are present in the frozen corpus. |
| `ControlLib.CheckCamera()` | `ControlLib.cs:1929-1960` | white LED; camera settings; close door; move to location 23 at offsets 4738 then 1895; label checks/snapshots; LEDs off; `parkGantry`. |
| `ControlLib.inspectCover()` | `ControlLib.cs:3663-3768` plus helper inspection methods | `ForceToHighHome`; `DeckInspection` branch; door close; cover recognition/rearrangement; `ErrorStatus`; physical camera/gantry motion. |
| `ControlLib.parkGantry(bool)` | `ControlLib.cs:7071-7122` | stale-tip remediation; optionally `HomeXY` with ±100 lost-step branches; `scriptmoveTo(... locationID 28 ...)`; update location. |
| `ControlLib.unlockDoor()` | `ControlLib.cs:10451-10459` | `doorOpen(true)`, solenoid control 0, latch/status/script state changes. |
| `BioXPMainWindow.PrepareToRunJob` | `BioXPMainWindow.cs:1567+` | full job/reagent admission and StartMode continuation; separate from physical homing but part of full terminal state. |

## 10. OEM configuration fields that must be bound for serial 206

No runtime default may be promoted to serial-206 OEM truth. The selected configuration
is already proven by the live-machine evidence lock and `oem_machine_bundle.py`:

```text
OEM acquisition: 20260719T024740Z-4a7fe6783205846c
OEM app root:     %LOCALAPPDATA%\Synthetic Genomics\GenBotApp
config.xml:       33aadf87f631cf33f2e0b4c86948c92be3b21412ca5477ea8fa8bc7848cbf475
Operation params: d032d58c08312706892a2d5c7a9a319f817a6d9ef73e75d30dd933ae110c9685
Inspection data:  d38220177e7e01b3d6d50892e0ffbbe27b1eb46087c4623cd6ca4757cc80b2d7
```

`BioXPMainWindow.GetAppDir()` derives this root from `AssemblyCompany("Synthetic
Genomics")` and assembly name `GenBotApp`. `ClassBioXPSettings.cs:2843-3524`
loads the machine configuration and inspection settings; `2516-2709` loads
`Operation_parameters.xml` using current-directory-first/AppData-second precedence.
The acquisition contains no current-directory operation-policy override.

The selected serial-206 values include:

```text
SerialNumber=206; ConfigVersion=3; GripperVersion=1; TroughVersion=1
Calibrated=1; CameraInstalled=1; CameraCalibrated=True
Operation Mode=WebMode; SelfTest=True; DeckInspection=True; CheckCamera=True
TCDoorOpen=18500; TCDoorStallGuardThreshold=6
TC_DOOR_VELOCITY=50; TC_DOOR_ACCELERATION=20; TC_DOOR_MAX_CURRENT=31
Z_MOTOR_MAX_CURRENT_DOWN=25; Z_MOTOR_MAX_CURRENT_UP=31
Z_MOTOR_STALL_GUARD_THRESHOLD=3
X limits=0..90263; Y limits=0..102956; Z limits=0..160000; G limits=0..15000
PositionTable=29 selected entries
CAMERA_OFFSET: x=3499, y=-7744, zLow=3145, zDelta=6842
```

The runtime implementation must consume the immutable selected snapshot for at least:

```text
SerialNumber
GripperVersion
Calibrated
CameraCalibrated
CameraInstalled
CheckCamera
SelfTest
StartMode
TC_DOOR_VELOCITY
TC_DOOR_ACCELERATION
TC_DOOR_MAX_CURRENT
TCDoorStallGuardThreshold
TCDoorOpen
XLowLimit / XHighLimit
YLowLimit / YHighLimit
ZLowLimit / ZHighLimit
Z_MOTOR_MAX_CURRENT_UP
Z_MOTOR_STALL_GUARD_THRESHOLD
m_gripperposition
PositionTable
InspectionSettings
CameraXOffset / CameraYOffset / CameraZOffset
DeckInspection / InspectionLogOnly
```

**Configuration source:** selection and values are proven. The implementation gate is
now consumption of the locked snapshot without silent fallback, not further acquisition.

## 11. Complete OEM code-name acquisition ledger

### 11.1 Source bodies already located and ready for line-level implementation review

```text
BioXPMainWindow.initializeEnvironment
BioXPMainWindow.motion_thread_process
BioXPMainWindow.initializeSystem
BioXPMainWindow.SelfTest
BioXPMainWindow.PrepareToRunJob
BioXPMainWindow.UpdateCheck
ControlLib.initialCheck
ControlLib.checkDoorStatus
ControlLib.initializeMotion
ControlLib.CheckCamera
ControlLib.inspectCover
ControlLib.parkGantry
ControlLib.unlockDoor
ControlLib.selftest / TCSelfTest / RCSelfTest / OCSelfTest
ClassControlInterface.initializeMotorsWithoutMotion
ClassControlInterface.initializeMotors
ClassControlInterface.setChillerCoolRate
ClassControlInterface.confirmAxis
ClassControlInterface.moveX / moveZ / setGripperCurrent
ClassHeadBoard.goHome / axisSearchHome / moveSteps / doorSearchHome
ClassThermalBoard.doorSearchHome / axisSearchHome / queryHome
ClassMotor.MoveLeft / StopMotor / MovetoRelPosition / setHome /
  queryLeftSwitchStatus / queryRightSwitchStatus / queryMotorSpeed /
  setMaxSpeed / setStallGuardThreshold
ClassPipetteCollection.initiateGroup / checkedPipetteStatus / ejectAllTips
ClassBioXPSettings field definitions
```

### 11.2 Recovered artifacts that must be line-locked into the implementation registry

| OEM code name / artifact | Why it is still required |
|---|---|
| `ClassPipette` command bodies plus `InterfaceCAN`/`ClassNovo` routing | All named sources are present and hash-sealed in §2. Extract exact command bytes, waits, completion/error ownership, and reply predicates into the tranche registry; this is no longer an acquisition blocker. |
| `ClassControlInterface.scriptmoveTo`, `moveTo`, `moveX`, `moveZ`, `HomeXY`, `HomeAxis` | Bodies are present in `ClassControlInterface.cs`; the selected 29-entry serial-206 `PositionTable` and axis limits are sealed in `config.xml`. Line-lock each physical route before implementation. |
| `ControlLib.doorOpen`, `checkDoorStatus`, `inspectCover` helpers | `checkChillerCover`, `InspectOutputLocation`, `checkRCCover`, `checkCoverStorage`, capture/release, `CheckCamera`, `AdjustCamera`, and `SnapshotImage` are present in `ControlLib.cs`; `ClassFrameGrabber.cs`, `CVisionLib.dll`, `InspectionSettings.xml`, and all required case-sensitive templates are captured and sealed. |
| `ControlLib.selftest`, `TCSelfTest`, `RCSelfTest`, `OCSelfTest` dependencies | Named bodies are present in `ControlLib.cs`; lower chiller/thermal bodies are present in `ClassChillerBoard.cs` and `ClassThermalControl.cs`. Extract exact command/reply paths into the implementation registry. |
| `ControlLib.parkGantry` dependencies | `parkGantry`, `scriptmoveTo`, `HomeXY`, and the selected location table are present. Line-locking remains; source acquisition does not. |
| Board activation/transport methods | `ClassDeckBoard`, `ClassChillerBoard`, `ClassThermalBoard`, `ClassThermalControl`, `InterfaceCAN`, `ClassNovo`, and `ClassNovoCANUSB` are present. Map called bodies to binaries and runtime transport. |
| `ClassBioXPSettings` and serial-206 corpus | Reader, selected paths, exact 19-file corpus, field provenance, values, hashes, and closed-world runtime projection are already established by `OEM_EVIDENCE_LOCK.json` and `oem_machine_bundle.py`. |
| exact OEM DLL/IL map for every cited decompile | Required if any decompiler ambiguity affects a behavioral branch or literal. |

## 12. Source-shaped implementation tranches — no broad rewrite

Implementation follows this order only after the ledger entries needed by each tranche are sealed.

1. **Tranche A — configuration and source-oracle foundation**
   - immutable serial-206 OEM configuration snapshot;
   - direct source-to-binary map and code-name citation registry;
   - no hardware action.

2. **Tranche B — application lifecycle worker**
   - exact queued `initializeSystem` ownership, re-entry state, terminal source state, and branch ledger;
   - no physical motion enabled.

3. **Tranche C — primitive source oracles**
   - fake-transport production-executor tests for `axisSearchHome`, `goHome`, door home, stop, switch, and set-home command rounds;
   - separate Linux safety-intercept metadata; no parity relabeling.

4. **Tranche D — literal `initializeMotors` state machine**
   - all 16 ordered actions in §7, config-bound and machine-guarded;
   - every action produces source stage, raw controller evidence, safety evidence, and terminal branch.

5. **Tranche E — full `initializeMotion`**
   - includes tip-query/remediation/retry branch exactly; not a homing-only substitute.

6. **Tranche F — full `initializeSystem` terminal branches**
   - unexpected shutdown, self-test, camera, cover inspection, parking, door, StartMode, job acquisition, and error exits.

7. **Tranche G — supervised BMS projection**
   - only after robot-local lifecycle is authoritative;
   - BMS is a typed proxy and display, never a stage-order authority.

## 13. Acceptance gates

A tranche may not be called OEM-parity complete until it has:

1. direct source/binary/configuration anchors for every called method and field;
2. a production call ledger that proves exact order, literals, waits, and branches using fake transport/clock only;
3. no replacement-only step presented as OEM behavior;
4. a separate list of safety intercepts, each inhibiting but never rewriting the OEM action;
5. a robot-side persistent lifecycle/artifact ledger;
6. independent review of the exact candidate bytes;
7. explicit Christian authorization before any physical stage.

## 14. Current stop point

The current live proof remains only:

```text
constructor_pipette
→ initializeMotorsWithoutMotion-equivalent
→ initialCheck
→ STOP before initializeSystem
```

The next physical OEM boundary is the queued `initializeSystem` worker path. It is intentionally not implemented or run by this specification work.

## 15. Independent direct-source reconciliation — exact additions

This section reconciles an independent three-way direct-C# audit performed against the same pinned OEM corpus. It supersedes less-specific wording elsewhere in this contract.

### 15.1 Application and lifecycle corrections

- `OperationMode` is directly defined by `OperationMode.cs:3-9` as:
  `0=DevMode`, `1=WebMode`, `2=LocalMode`, `3=TradeShowMode`.
- `initializeEnvironment()` **calls but does not consume** the Boolean returned by `initialCheck()` (`BioXPMainWindow.cs:976-979`). Its queue admission is based on the subsequent enclosure/latch branches, not that return value.
- `initialCheck()` polls a persistently unready CAN service for **12 × 200 ms = 2400 ms** before returning false (`ControlLib.cs:8728-8759`).
- `checkDoorStatus()` is source-significant: it sleeps 500 ms, may actuate the solenoid for 800 ms on latch value 1, and its `query24voltage()` branch changes returned truth/status fields (`ControlLib.cs:8670-8726`). A replacement must reproduce or separately label this behavior; it must not reduce it to a generic door-read predicate.
- `motion_thread_process()` has no dispatch-level `try/finally`; it sets `GantryAvailable=false` before dispatch and `true` after its switch (`BioXPMainWindow.cs:2039-2100`). `UpdateCheck()==true` skips queued `initializeSystem` execution (`4264-4309`).
- The `ShipMode == "PARK"` branch returns before `initializeSystem()` reaches its `finally`; the direct source therefore leaves `m_systemInmotion=true` on that return (`1127-1134`). This is OEM behavior to preserve or explicitly intercept, not normalize silently.
- There is a separate `WarningSituation.ENCLOSURE_OPEN` recovery path in `BioXPMainWindow.cs:2627-2661` that calls `initializeMotion()` directly after enclosure recovery. It is not the normal full `initializeSystem` continuation and must not be exposed as a parity-equivalent full-startup command.

### 15.2 Full `initializeMotion()` pipette contract now source-anchored

The collection and per-pipette methods are present in `ClassPipetteCollection.cs` and `decompiled_src_can/BioXPControlLib/ClassPipette.cs`; CAN/Novo transport ownership is present in `InterfaceCAN.cs`, `ClassNovo.cs`, and `ClassNovoCANUSB.cs`. This corrects the earlier acquisition ledger: exact implementation extraction remains, but source acquisition does not.

| OEM code name | Exact direct behavior / anchor |
|---|---|
| `queryTipStatus(-1)` / `TipExist` | `TipExist` is true when any of four underlying `ClassPipette.TipLoaded` values exceeds zero; the integer query result is not the `initializeMotion()` branch predicate. `ClassPipetteCollection.cs:90-101,1336-1358`. |
| `ejectAllTips(false,true)` | `1176-1235`: selects one/all channels, calls each selected `ejectTip`, waits up to 8000 ms, disables the missing-tip callback because `checkMissingTip=false`, calls `verifyEjectTip`, then sets `TipLocation=-1`. |
| `verifyEjectTip()` | `1265-1323`: if any tip remains, ejects all four, waits 6000 ms, re-queries, then unlocks door and throws on persistent tips. |
| `initiateGroup()` | `677-693`: reinitializes eligible channels, waits 10000 ms, enables pressure streaming for 1000 ms, then calculates pressure offset. |
| `checkedPipetteStatus()` | `726-748`: queries all four with 30 ms inter-command delay, then requires each error code to be zero. |

`initializeMotion()` calls raw `ClassControlInterface.openThermalDoor()` and ignores its Boolean result before setting `ThermalDoorOpen=true` (`ControlLib.cs:8809-8814`). `openThermalDoor()` itself uses `TCDoorStallGuardThreshold+2`, `TC_DOOR_MAX_CURRENT`, `TCDoorOpen`, and for serial >9 returns true only when `tcDoorOpened && !tcDoorClosed` (`ClassControlInterface.cs:2651-2676`).

### 15.3 Full optional self-test contract

`ControlLib.selftest()` is not a superficial health check. It queues `TCSelfTest`, `RCSelfTest`, and `OCSelfTest` in parallel; closes the thermal door; homes/moves X/Y/Z/gripper; applies lost-step thresholds (X/Y/Z: 100; gripper: 500); parks/unlocks; then waits for subtest completion (`ControlLib.cs:10688-10785`).

- `TCSelfTest`: source thresholds/timeouts include 150-second high/low/lid criteria (`10788-10865`).
- `RCSelfTest` / `OCSelfTest`: 180-second cooldown criteria (`10867-10999`).
- Aggregate completion is called as `waitforcompletion(100000)` and status-date update occurs only on pass.

No self-test control may exist until all three direct bodies and their thermal/chiller dependencies are implementation-complete.

### 15.4 Corpus closure correction: no missing-data blocker

The recovered corpus contains every category previously mislabeled as absent:

1. Per-pipette commands, CAN routing, Novo USB transport, completion and reply bodies.
2. Route functions, the selected 29-entry serial-206 position table, axis limits, and camera offsets.
3. Camera/cover helpers, `ClassFrameGrabber`, `CVisionLib.dll`, inspection profiles, and case-sensitive pattern assets.
4. `ClassChillerBoard` and `ClassThermalControl` command/reply bodies used by chiller rates and TC/RC/OC self-tests.
5. The exact `ClassBioXPSettings` reader path and immutable serial-206 live corpus with closed-world hashes and runtime projection.

The true remaining source-phase work is mechanical but mandatory: populate the
source-to-binary/code-name registry with exact line ranges and resolve only concrete
decompiler ambiguities against the already captured DLL/IL. It is **not** additional
OEM data acquisition. Runtime semantic implementation, fake-transport validation,
live-controller validation, and physical authorization remain separate later gates.
