# BioXP3200 serial-206 finished OEM movement-parity integration work tranche

**Date:** 2026-07-23
**Status:** implementation-ready design
**Target:** BioXP3200 serial `206`
**Controlling rule:** recovered OEM authority only
**Physical action performed by this design cycle:** none

## 1. Outcome

Implement and accept one complete robot-owned OEM startup movement lifecycle:

```text
initializeEnvironment
→ initialCheck
→ enclosure/latch admission
→ queue initializeSystem
→ motion_thread_process
→ initializeSystem
→ initializeMotion
→ initializeMotors
→ stale-tip remediation when required
→ optional daily self-test
→ conditional camera check
→ cover inspection/remediation
→ gantry park
→ door / StartMode / job-admission terminal state
```

The finished system must preserve the recovered OEM method order, selected serial-206 configuration, board/axis routing, waits, branch predicates, reply semantics, faults, and postconditions. Every physical stage must be traceable to the exact method/source/binary registry and to a persistent robot-side command ledger.

This tranche ends only when the implementation has passed frozen-source review, deterministic fake-transport tests, real-controller non-motion checks, separately authorized physical stage commissioning, complete startup replay, API/BMS proof, and remote Git verification.

## 2. Controlling source registry

The implementation authority is:

```text
docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json
docs/specs/2026-07-23-oem-movement-method-source-binary-registry.md
scripts/verify_oem_movement_registry.py
```

Accepted registry baseline:

```text
binaries=8
sources=32
methods_and_members=392
live_machine_records=19
selected_configuration_records=44
required_call_graph_anchors=124
required_exact_call_edges=47
ordered_source_sequences=3
named_hazards=17
OEM_MOVEMENT_REGISTRY=PASS
```

Canonical upstream evidence remains:

```text
OEM_EVIDENCE_LOCK.json
acquisition=20260719T024740Z-4a7fe6783205846c
machine=BIOXP206
serial=206
```

No implementation change may introduce a behavior attributed to OEM authority unless it names:

```text
source_id
source SHA-256
inclusive source lines
captured binary and SHA-256
selected configuration artifact/hash/selector when applicable
known decompiler-warning boundary
named source-hazard disposition
```

The source closure includes the route and policy data types consumed by the methods, not just executable bodies:

```text
CameraSettings.cs
InspectionItems.cs
locationID.cs
positionStruct.cs
wellID.cs
plateName.cs
OperationMode.cs
```

`Not implemented`, `not line-extracted into a fixture`, and `not physically validated` must never be reported as source absence.

## 3. Scope

### 3.1 Included

- exact serial-206 machine bundle selection;
- one-writer Novo USB/CAN ownership;
- CAN service readiness and board construction;
- board 4/5/6/7 activation and acknowledgement policy;
- enclosure/latch/24-V/solenoid admission behavior;
- `initializeMotorsWithoutMotion` configuration writes;
- literal `initializeMotors` ordering;
- X/Y/Z/gripper/thermal-door movement primitives;
- chiller rate initialization used by startup;
- stale-tip query, route, eject, verification, and pipette reinitialization;
- optional OEM daily self-test movement and thermal/chiller workers;
- camera startup check;
- cover inspection and cover relocation movements;
- gantry parking;
- terminal door and selected StartMode branches;
- persistent stage, frame, reply, state, provenance, and authorization ledger;
- fixed robot API commands and BMS presentation;
- fake-transport, controller, physical-stage, and full lifecycle acceptance.

### 3.2 Excluded

- arbitrary diagnostic motion exposed to BMS;
- general job-script execution after startup/job admission;
- rewriting native CVision algorithms from decompiler projections;
- treating diagnostic XML/defaults as live machine authority;
- automatic restart, automatic physical retry, or automatic resume after process death;
- unrelated robot, BioModStack, deployment, or fixture changes.

## 4. Immutable serial-206 authority

### 4.1 Exact machine selection

Production must load only the closed-world snapshot already implemented by:

```text
src/bioxp/oem_machine_bundle.py
```

Required identity:

```text
acquisition_id=20260719T024740Z-4a7fe6783205846c
machine_serial=206
live_machine_records=19
```

Required principal hashes:

```text
config.xml
33aadf87f631cf33f2e0b4c86948c92be3b21412ca5477ea8fa8bc7848cbf475

Operation_parameters.xml
d032d58c08312706892a2d5c7a9a319f817a6d9ef73e75d30dd933ae110c9685

InspectionSettings.xml
d38220177e7e01b3d6d50892e0ffbbe27b1eb46087c4623cd6ca4757cc80b2d7
```

Missing, extra, wrong-case, wrong-size, wrong-hash, or wrong-serial records must fail before transport ownership or movement admission.

### 4.2 Selected movement data

The implementation must consume, not duplicate as unbound literals:

```text
SerialNumber=206
ConfigVersion=3
GripperVersion=1
Calibrated=1
CameraInstalled=1
CameraCalibrated=True
Mode=WebMode
SelfTest=True
DeckInspection=True
InspectionLogOnly=False
CheckCamera=True
ScreenResolutionHigh=False

SelfTestTravel raw OEM projection:
  X=92049
  Y=92049
  Z=92049
SelfTestXMax=90263
SelfTestYMax=92049
SelfTestZMax=92049

OriginOffsetG=4450
GripperClosePOS=27350
GripperOpenPOS=31400
GripperOpenWide=32400
TCDoorOpen=18500
TCDoorStallGuardThreshold=6
TC_DOOR_VELOCITY=50
TC_DOOR_ACCELERATION=20
TC_DOOR_MAX_CURRENT=31
Z_MOTOR_MAX_CURRENT_DOWN=25
Z_MOTOR_MAX_CURRENT_UP=31
Z_MOTOR_STALL_GUARD_THRESHOLD=3
OutPutBufferatMS_Zlow=0
OutlierRangeFactor=4

X limits=0..90263
Y limits=0..102956
Z limits=0..160000
G limits=0..15000
PositionTable=29 exact entries
CAMERA_OFFSET=(3499,-7744,3145,6842)
```

The self-test values above are not inferred motor limits. They are mechanically replayed from the selected 29-entry `PositionTable` through the literal `ClassBioXPSettings.loadConfig` projection at lines `3387-3450`: raw `X=Y=Z=92049`, including the cross-axis assignments at `3449-3450`, followed by `SelfTestX/Y/ZMax = Math.Min(raw, axis-high-limit)` at `1277-1281`. The resulting selected serial-206 maxima are exactly `90263/92049/92049`. This records the OEM behavior while keeping `SETTINGS_SELFTEST_AXIS_CROSS_ASSIGNMENT` as a mandatory binary-resolution and safety-review hazard.

Chiller calibration:

```text
ReagentChiller: max heat 5, max cool -5, proportional 1.5, feed-forward 400
OutputChiller:  max heat 5, max cool -5, proportional 1.5, feed-forward 400
```

### 4.3 Board and axis routing

Recovered board construction:

```text
head board=4
 deck board=5
thermal board=6
chiller board=7
```

Recovered logical routing:

```text
MotorX         → board-array 1 / physical board 5 / axis 0
MotorY         → board-array 0 / physical board 4 / axis 0
MotorZ         → board-array 0 / physical board 4 / axis 1
MotorGrip      → board-array 0 / physical board 4 / axis 2
ThermalDoor    → board-array 2 / physical board 6 / axis 0
ThermalCycler  → board-array 2 / physical board 6 / axis 1
OutputChiller  → board-array 3 / physical board 7 / axis 1
ReagentChiller → board-array 3 / physical board 7 / axis 0
IO             → board-array 1 / physical board 5
```

Authority:

```text
ClassControlInterface.cs:29-111
ClassControlInterface.cs:170-556
```

No generic or inferred axis map may override this selection.

## 5. Runtime architecture

### 5.1 One writer

Exactly one robot service owns the Novo USB/CAN device during the lifecycle. Ownership must continue through:

```text
POST /oem/runtime/activate_service
```

The legacy maintenance reconnect route is not an ownership transition and must not be used for this tranche.

No test, diagnostic process, background monitor, or BMS request may open the device concurrently. Ownership generation must be captured at run creation and checked before every stage.

### 5.2 Robot-owned state machine

Implement a persistent robot-side state machine, not a sequence assembled in the UI.

Recommended focused modules:

```text
src/bioxp/oem_movement_registry.py
src/bioxp/oem_movement_ledger.py
src/bioxp/oem_movement_state_machine.py
src/bioxp/oem_transport_trace.py
```

Existing modules remain the integration points:

```text
src/bioxp/oem_machine_bundle.py
src/bioxp/oem_runtime_commands.py
src/bioxp/oem_initialization.py
src/bioxp/oem_startup_program.py
src/bioxp/usb_driver.py
src/bioxp/api.py
```

Keep the implementation small enough for a five-person team. Do not introduce a distributed workflow system, generic plugin framework, or arbitrary command language.

### 5.3 Persistent ledger

Every run and stage record must contain:

```text
run_id
command_id
stage_id
attempt
parent_stage_id
status
expected_predecessor
ownership_generation
registry_schema
registry_sha256
evidence_lock_schema
acquisition_id
machine_serial
source_id
source_sha256
source_lines
binary_id
binary_sha256
configuration artifact/hash/selector/value
operator acknowledgement
stage authorization identifier
precondition snapshot
outbound frame bytes and timestamp
inbound reply bytes and timestamp
normalized acknowledgement
readback/postcondition
state transition
physical_motion_commanded
physical_effect_verified
blocked_reason/error
started_at/completed_at
```

Stage status is monotonic:

```text
pending
→ admitted
→ running
→ acknowledged
→ postcondition_verified
→ completed
```

Terminal alternatives:

```text
blocked
failed
cancelled
aborted
```

A restart must reconstruct the last durable state but must not automatically resume a physical stage. An interrupted `running` physical stage becomes `blocked` pending operator inspection.

### 5.4 Source registry enforcement

At process startup and before run admission:

```text
python3 scripts/verify_oem_movement_registry.py
```

Equivalent in-process validation must reject:

- source/binary/config hash drift;
- missing exact registry method/member IDs or overload identities;
- missing or mismatched caller→callee edge lines;
- altered ordered source sequence lines/tokens;
- source-to-binary mapping not proven by the canonical evidence lock or exact project `AssemblyName`;
- XML selector/attribute/profile identity drift or self-test derivation drift;
- a machine serial other than 206;
- an unknown or non-accepted source mode;
- a physical stage without a registry binding;
- a decompiler-warning method lacking its required binary-resolution disposition.

## 6. Transport parity

### 6.1 Authority chain

```text
InterfaceCAN.TransmitMessage
→ ClassNovo.TransmitMessage
→ ClassNovoCANUSB.sendCommand
→ ClassNovoCANUSB.transmitCommand
→ Novo.Devices.CanPacket
→ CanInterfaceBoard.WritePacket
→ NovoEncoding.Encode
→ WinUsbCommunications.SendDataViaBulkTransfer
→ USB-CAN device
→ WinUsbCommunications.ReceiveDataViaBulkTransfer
→ NovoEncoding.Decode
→ CanInterfaceBoard.RxThread / ReplyPacketAvailable
→ ClassNovoCANUSB.ProcessReceivedTrafficPacket
→ IsAMatch / IsAMatchPipette / IsAMatchPipetteUIM
→ ClassNovo.GotMessageProcess
→ board/pipette handler
```

Each movement primitive must produce a golden request/reply vector extracted from its registered source method and pinned captured binary. No command bytes may be supplied from generic TMCL/CAN convention alone.

### 6.2 Exact Novo.Devices wire framing

The captured `Novo.Devices.dll` and recovered sources are line-registered through the USB boundary:

```text
CanPacket.DataLength = payload count
CanInterfaceBoard.WritePacket:
  allocate payload_length + 5
  bytes 0..3 = ModuleId in big-endian order
  byte 4     = DLC / DataLength
  bytes 5..  = CAN payload
NovoEncoding.Encode:
  checksum = modulo-256 sum of unescaped source bytes
  frame delimiter = 0x7e at both ends
  escape byte     = 0x7d
  escaped 0x7e/0x7d byte = 0x7d followed by byte XOR 0x20
  checksum is escaped by the same rule
WinUsbCommunications.SendDataViaBulkTransfer:
  transfer exactly the encoded frame length
```

Receive processing must bind the raw USB bytes, decoded bytes, reconstructed module ID, DLC/payload, reply matcher, and controller acknowledgement to one command ledger entry. Golden fixtures must include normal bytes, escaped delimiter, escaped escape byte, checksum wraparound, malformed framing, malformed checksum, short payload, wrong DLC, and wrong module ID.

`NovoEncoding.Decode:49-75` visibly removes framing/checksum bytes, unescapes the body, and returns `true` without checking delimiters or checksum. The Linux port must therefore **fail closed by validating both framing and checksum before reply admission**. This is a reviewed safety deviation under `NOVO_DECODE_NO_CHECKSUM_VALIDATION`, not a false claim that the OEM projection performed the check.

### 6.3 Acknowledgement policy

Canonical startup policy:

```text
boards 4–6: status 100 only
board 7 command-64 activation/deactivation: status 100 or 2
```

All other status values are failures unless the exact registered method and binary prove a narrower exception. Empty replies, mismatched IDs, wrong DLC, stale replies, duplicate unmatched replies, and timeouts fail closed.

### 6.4 Timing and serialization

Preserve:

- literal source sleeps;
- per-command timeout values;
- request/reply correlation;
- single in-flight ownership where the OEM transport serializes;
- pipette-specific matching behavior;
- board reply handler state updates;
- the OEM `StopMotor` behavior, including duplicate stop transmission where shown by authority;
- explicit deviations only when the OEM behavior is unsafe or ambiguous, with a named safety disposition and tests.

A safer Linux deviation must not be labeled literal OEM parity.

### 6.5 Named source-hazard dispositions

The independent source audits established the following mandatory design constraints. These are present-source hazards—not acquisition gaps:

| Hazard | Required implementation disposition |
|---|---|
| `ClassNovoCANUSB.cs:481-484` projects `(ID & 7) == 259/260`, while line 691 coherently uses `ID & 0x107` | Resolve raw IL before creating the pipette transport fixture or enabling the path. Do not guess the mask. |
| `NovoEncoding.Decode:49-75` projects no delimiter/checksum validation before returning `true` | Preserve OEM receive vectors, but require explicit fail-closed framing/checksum validation as the reviewed Linux safety deviation `NOVO_DECODE_NO_CHECKSUM_VALIDATION`. |
| `ClassMotor` and board home methods conflate null/uninitialized replies with success-like values and can update host home state after failed transport | Require a matched controller reply, expected acknowledgement, switch/readback evidence, and physical postcondition before declaring home. |
| `ClassThermalControl` can return cached/default temperatures, success-like zero on null, dereference a null lid reply, and discard PWM status through wrappers | Preserve raw replies/status; fail closed on null, stale, or unacknowledged values. |
| `scriptmoveTo` projected location exclusions at `3869` and `3877` are always true | Resolve against raw IL or register an explicit reviewed safety deviation before porting those predicates. |
| `moveXY` projected null-X-board branch at `4292-4295` calls `moveX(y)` | Resolve against raw IL or block the branch; never reproduce it as an inferred valid route. |
| Thermal-door methods can return success with a null board and cache the requested state despite failed confirmation | Require board presence and both door-sensor postconditions. Homing without a confirmed retry does not prove closure. |
| `loadConfig` bypasses the serial setter; serial-206 XML `TCDoorOpen=18500` differs from setter default `16000` | Pin the selected XML value and loader path; do not reconstruct it from the setter default. |
| Self-test Y/Z travel projection uses X travel in existing-value branches at `3449-3450` | Preserve the projected fixture for comparison, but require an explicit safety disposition before physical self-test. |
| OEM settings readers continue or suppress errors for missing config/master position/offset files | Linux production remains closed-world and fail-closed; no permissive fallback becomes authority. |
| `motion_thread_process` can bypass `GantryAvailable=true` on exception | The persistent state machine must always terminalize or enter blocked recovery in a finalizer. |
| Gripper `+10000` occurs before the later null guard in `initializeMotors` | Admission must prove the head/gripper board before issuing the command; record this as a safer explicit deviation. |
| `turnOffHeater` sends identical `setTCorLidPWM(0,0)` calls twice | Golden fixture must preserve both calls; both need transport evidence. Do not relabel them as separate inferred selectors. |
| Self-test emits “Finished” messages after queue submission and has STA/non-STA timeout differences | Ledger records submission separately from completion and pins one tested timeout model. |
| Vision/cover routines contain explicit invalid-IL/unsafe projection regions | Resolve affected control flow against the pinned binary or retain a blocker; do not compile apparent decompiler output as authority. |
| Selected camera profiles contain `Gain`, but `AdjustCamera` does not apply it | Pin the selected gain in provenance but do not claim it was applied; acceptance must verify only exposure/LED effects that the caller actually performs. |
| `checkDoorStatus` projects nonzero `query24voltage()` as false/door-open and only zero as success | Resolve signal polarity against raw IL and controller evidence before physical admission; do not infer that nonzero means healthy or unhealthy from naming alone. |

Every enabled stage must name its resolved hazard IDs in the program manifest. An unresolved applicable hazard is an admission blocker.

## 7. Admission and non-motion baseline

### 7.1 Existing accepted prerequisite

Preserve the verified sequence:

```text
ownership activation
→ runtime_ready
→ verified camera-free hardware snapshot
→ CAN_READY=true
→ hardware_connected=true
→ hardware_fresh=true
→ hardware_ready=true
```

Preserve the successful non-motion lifecycle:

```text
constructor_pipette_stage=passed
initialization_without_motion=passed
initial_check=passed
next_oem_boundary=initializeSystem
```

Do not rerun this merely to collect duplicate evidence.

### 7.2 `initializeMotorsWithoutMotion`

Implement and fixture the exact registered body at `ClassControlInterface.cs:3181-3265`:

1. `waitForBoard`;
2. heater off;
3. chiller PWM setup;
4. sleep 1 ms;
5. X speed/acceleration `1700/350`, max current `31`, stall guard `16`, with 2-ms gaps;
6. Y speed/acceleration `1800/400`, max current `31`, stall guard `16`, disable right switch, with 2-ms gaps;
7. Z speed/acceleration `1791/576`, selected max current `31`, read current, selected stall guard `3`, with 2-ms gaps;
8. gripper-v1 speed/acceleration `1500/20`, max current `10`, stall guard `20`, R/P divisors `6/2`, with 2-ms gaps;
9. thermal-door speed/acceleration `50/20`, max current `31`, stall guard `6`, disable both limit switches, with 2-ms gaps;
10. selected chiller cool rates;
11. thermal cycler heat/cool rates `2.5/-2.0`;
12. LED white.

This stage writes configuration but commands no axis travel. Tests must prove no movement opcode/frame appears.

### 7.3 Door admission

Source anchors:

```text
ControlLib.checkDoorStatus:8670-8726
ControlLib.initialCheck:8728-8760
BioXPMainWindow.initializeEnvironment:973-1027
```

Preserve the exact read/solenoid/wait branches, including 500-ms, 800-ms, 300-ms, 50-ms, and CAN-ready 200-ms polling. The unusual OEM 24-V branch that clears `EnclosureDoorClosed` after solenoid release must be resolved against the pinned binary and hardware meaning before physical acceptance. If Linux intentionally uses a safer interpretation, record `safety_deviation`, source lines, reason, replacement predicate, and acceptance evidence.

Movement admission requires fresh robot-local proof of:

```text
CAN ready
correct ownership generation
serial 206 bundle
24-V state accepted
enclosure closed
latch closed
no active board fault
no unresolved prior physical stage
```

## 8. Literal startup movement state machine

### 8.1 Application dispatch

Source anchors:

```text
BioXPMainWindow.initializeEnvironment:973-1027
BioXPMainWindow.motion_thread_process:2030-2101
BioXPMainWindow.initializeSystem:1046-1342
```

Required dispatch:

```text
initializeEnvironment
→ initialCheck
→ exact enclosure/latch branch
→ enqueue fixed initializeSystem command
→ worker takes command
→ GantryAvailable=false
→ UpdateCheck gate
→ initializeSystem
→ terminal status update
→ GantryAvailable=true
```

Only one `initializeSystem` run may be active. Duplicate requests return the existing run identifier and do not enqueue another physical sequence.

The implementation must preserve the `initializeSystem(bool skipInitializeMotion=false)` branch matrix rather than flattening it into one happy path:

```text
re-entry:
  if m_systemInmotion: do not start a second lifecycle

skipInitializeMotion=false:
  run a fresh ControlLib.initialCheck at initializeSystem line 1143
  read ClassStatusLog.SavedStatus after that fresh check
  if SavedStatus is neither 3 nor 4: run initializeMotion once at line 1159

skipInitializeMotion=true:
  bypass the line-1143 fresh initialCheck and initializeMotion branch
  continue into the later self-test/camera/cover stages exactly as OEM

saved machine status 3 or 4, only inside skipInitializeMotion=false:
  run initializeMotion at line 1148
  run inspectCover at line 1149
  unlock/report the recovered branch and return
  do not run the normal line-1159 initializeMotion or fall through to normal completion

normal branch:
  optional daily self-test only when SelfTest=true and elapsed days >1
  optional camera check only when CheckCamera && CameraInstalled && !development
  invoke inspectCover; DeckInspection=false can make its body return OK
  on cover failure: park, unlock, persist blocked terminal state
  on success: park before StartMode handling

PARK/ship branch:
  close thermal door using the registered movement path
  publish a typed park/shutdown-ready terminal state
  actual OS shutdown requires a separate reviewed deployment authorization

communication/job-admission branch:
  preserve the 5000-ms communication wait
  end at typed job-admission readiness; general job execution remains excluded

all exits:
  clear running ownership in a finalizer
  persist exact terminal state and blocker
```

Startup state authority is `BioXPMainWindow.MainWindowInitialize:375-822 → ClassStatusLog.loadStatus:249-391`. `loadStatus` reads `c:\\logfile\\instrument_status.xml`, copies `LatestStatus` into `SavedStatus` at lines `303-312`, and loads `SelfTestDate` at `327-335`. `ShipMode` is **not** loaded from that file: it defaults to an empty string at `ClassStatusLog.cs:44` and changes only through `updateShipMode:125-128`. Linux must therefore model `SavedStatus`, `SelfTestDate`, and runtime `ShipMode` as distinct typed inputs; it must not fabricate a selected ShipMode configuration value.

The recovered application’s saved-status and PARK branches are OEM authority. The Linux finalizer and separate OS-shutdown authorization are explicit safety/reliability deviations and must be labeled as such.

### 8.2 `initializeMotors` exact order

Source: `ClassControlInterface.cs:3348-3421`.

Implement these fixed stages in order:

```text
M01 Z axisSearchHome(speed=1791)
M02 gripper current=31
M03 gripper moveSteps(+10000, wait=true)
M04 gripper-v1 axisSearchHome(speed=200)
M05 X axisSearchHome(speed=250)
M06 sleep 20 ms
M07 X setHome
M08 X setSpeed(1700)
M09 sleep 40 ms
M10 X moveX(6000)
M11 Y axisSearchHome(speed=250)
M12 thermal doorSearchHome(speed=50, stall=6)
M13 if serial>9 and !confirmAxis(tcDoorClosed) and CameraCalibrated:
    openThermalDoor; fail "Cannot close thermal cycler door!"
M14 Y setHome
M15 selected calibrated-position state update
M16 output chiller cool rate=-0.025
M17 reagent chiller cool rate=-0.025
M18 system status initialization update
M19 gripper-v1 idle current=10
```

No reference-return, right-limit correction, generic rehome, or adapted post-home move may be inserted.

### 8.3 Primitive postconditions

Each physical primitive must have an OEM-derived completion predicate and a separately recorded safety predicate.

At minimum:

```text
axisSearchHome: correlated acknowledgement + home/readback semantics
moveSteps/moveToAbs: correlated acknowledgement + stopped/target readback
setHome: acknowledgement + zero/reference readback where source supports it
doorSearchHome: acknowledgement + door-closed predicate for serial 206
setSpeed/current/stall/divisors: acknowledgement; readback where supported
stopMotor: required duplicate transmit semantics + stopped readback
```

The known OEM null/uninitialized home-query behavior must not be silently converted into proof of home. Linux may fail closed, but the ledger must classify this as an explicit safety deviation rather than literal behavior.

### 8.4 Stale-tip remediation

Source: `ControlLib.initializeMotion:8797-8856` plus registered route/pipette methods.

Exact branch:

```text
initializeMotors
→ mark thermal door closed
→ queryTipStatus(all)
→ sleep 500 ms
→ if no tip: TipLoaded=false; continue
→ if tip:
   open thermal door
   mark door open / tip loaded
   scriptmoveTo(current=LOC_PARK, destination=WASTE_BIN)
   update location=WASTE_BIN
   ejectAllTips(checkMissingTip=false, wait=true)
   moveZ(80000)
   moveX(79000)
   queryTipStatus(all)
   sleep 100 ms
   if tip remains: pause/error/fail
   clear TipDirty/TipLoaded
   sleep 2 ms
   initiateGroup
   if checkedPipetteStatus fails:
      initiateGroup once more
      if still bad: error/fail
```

`WASTE_BIN`, `LOC_PARK`, route clearances, and coordinates must come from the selected 29-entry `PositionTable`. BMS must not supply these coordinates.

Pipette transport authority:

```text
ClassPipetteCollection
ClassPipette
InterfaceCAN
ClassNovo
ClassNovoCANUSB
```

Collection-level semantics from `ClassPipetteCollection.cs:1176-1323` are mandatory inside that upper-level branch:

```text
ejectAllTips:
  query current tip count
  target the recorded TipLocation or all four channels
  clear completion/reset wait events for commanded channels
  issue per-channel ejectTip
  waitforcompletion("Eject tip", 8000) when wait=true
  otherwise sleep 500 ms
  verifyEjectTip
  set TipLocation=-1 only after verification

verifyEjectTip(all channels):
  query status
  if any tip remains:
    issue one additional eject on all four channels
    waitforcompletion("Eject tip", 6000)
    query again
    if nonzero: unlock/error/fail

verifyEjectTip(keep-one-tip path):
  reject zero tips as lost-tip failure
  if more than one tip remains, eject all non-kept channels
  query and reject a count greater than one
```

The `KeepTip` path has its own 6000-ms wait and must remain distinct from startup's all-tip ejection. These waits are collection-level authority; they must not be replaced by the upper-level 500-ms/100-ms observation sleeps.

Every pipette request/reply, timeout, completion flag, and `TipLoaded` transition must appear in the robot ledger.

## 9. Optional self-test movement

Source: `ControlLib.selftest:10688-10786` and registered `TCSelfTest`, `RCSelfTest`, `OCSelfTest`.

Admission predicate:

```text
SelfTest=True
and elapsed since ClassStatusLog.SelfTestDate > 1 day
```

Required behavior:

1. reset three completion events;
2. queue TC, reagent-chiller, and output-chiller self-test workers;
3. close thermal door;
4. home Z, X, and Y;
5. sleep 100 ms and record position;
6. move X and Y concurrently to selected self-test maxima;
7. read positions;
8. sleep 100 ms;
9. `HomeXY` and reject absolute error above 100 steps per axis;
10. move Z to selected self-test maximum;
11. `HomeAxis("z")` and reject error above 100;
12. close gripper and `HomeAxis("g")`; reject error above 500;
13. park gantry and apply door behavior;
14. wait up to 100000 ms for thermal/chiller workers;
15. combine all worker results;
16. set chiller PWM;
17. update self-test date only on complete success.

The registered thermal/chiller worker semantics are also fixed:

```text
TC worker:
  lid target = current lid + 5°C
  open thermal door
  thermal target = current thermal + 5°C
  reject high-temperature attainment over 150 s
  reject nest A/B delta over 2°C
  require pedestal temperature in 5..40°C
  low target = prior lid-derived target - 10°C
  reject low-temperature attainment over 150 s
  require lid to exceed target-1°C within 150 s
  turn heater off in self-test finalizer

RC worker:
  cool rate=-0.05
  if current temperature <=15°C: pass and restore default rate
  otherwise target=int(current)-4°C
  reject set-temperature command failure
  poll until within 1°C, 1-ms sleeps, hard limit 180 s
  restore default RC rate in finalizer

OC worker:
  same contract as RC using output-chiller routing
  hard limit 180 s
  restore default OC rate in finalizer
```

`InspectionLogOnly` can cause the recovered worker catch blocks to mark a failed worker as accepted. The implementation must preserve this branch as source-visible behavior, but the ledger must still retain the underlying measurement/fault; `InspectionLogOnly` must never erase evidence.

The selected self-test maxima are derived by `ClassBioXPSettings` from the loaded position table and axis limits; they must be projected by the machine snapshot loader and pinned in the run record before this stage is enabled.

Concurrency must preserve OEM ordering but use one physical-command serialization point per shared transport. Worker completion events cannot mark success before all required reply/readback assertions pass.

## 10. Camera and cover movement

### 10.1 Camera gate

Source branch:

```text
BioXPMainWindow.initializeSystem:1172-1180
```

Selected predicate:

```text
CheckCamera
and CameraInstalled
and not development machine
```

Then `ControlLib.CheckCamera:1929-1960` must pass. Camera failure unlocks the door, records the OEM error branch, and prevents cover inspection.

Native authority:

```text
CVisionLib.dll
84ab0c851f1bb418289035efd9fa84420e9ff82ea0a69ec0c00bb5d401e750f2
```

The camera adapter must select the commissioned profile exactly through `CameraSettings.GetCameraSettings(bool Is3250, InspectionItems)` at `CameraSettings.cs:318-325`. For the selected serial-206 corpus, `Operation_parameters.xml:19` is exactly `ScreenResolutionHigh="False"`; therefore `Is3250=false`, the active profile is `Settings3200`, and the selected initial-check ordinal is `CamereInitialCheck=4`. `Settings3250` remains registered as non-selected branch authority only:

```text
Settings3200: exposure=1000, gain=1000, LED1/2/3=false
Settings3250: exposure=-3,   gain=1000, LED1/2/3=false
```

`AdjustCamera` applies exposure and LEDs but does not apply `Gain`; provenance must not be misreported as an applied camera setting.

Do not rewrite `ClassFrameGrabber.checkLabel` or `matchPattern` from decompiler output. Use the captured binary in an isolated adapter, or preserve a fail-closed blocker until binary execution is accepted.

### 10.2 Cover inspection

Source: `ControlLib.inspectCover:3663-3768` and registered helpers.

Required branch:

```text
if DeckInspection=false: return OK
close enclosure door
inspect output cover
inspect reagent cover
inspect output-cover storage
inspect reagent-cover storage
apply exact too-many/missing predicates
if InspectionLogOnly=false: fail on non-OK condition
if exactly two covers and misplaced:
  catchPlate
  releasePlate into selected storage
update machine plate locations
doors as specified
```

The stage must select the exact commissioned algorithm/profile branch:

```text
ScreenResolutionHigh=false / Settings3200:
  locations 17,19,20,18 use checkChillerCover → locateCover("")
  CoverInspection: exposure=1000, gain=1000, LED2=true
  OutputPlateInspection: exposure=1000, LEDs=false, threshold=60, pixelCount=1000

ScreenResolutionHigh=true / Settings3250:
  location 17 uses InspectOutputLocation
  location 19 uses checkRCCover
  locations 20 and 18 use checkCoverStorage
  CoverInspection: exposure=-2, gain=1000, LED2=true
  CoverStorageInspection: exposure=-1, all LEDs=true
  OutputPlateInspection: exposure=-2, all LEDs=true, threshold=30, pixelCount=1000
```

Both branches must use the hash-locked assets appropriate to their registered callers:

```text
InspectionSettings.xml
cover.jpg
output.jpg
outputw_foil.jpg
output_empty.jpg
reagentTray.jpg
reagentEmpty.jpg
EmptyStorage.jpg
CAMERA_OFFSET from config.xml
selected cover/storage PositionTable entries
```

Serial `206` must execute the `ScreenResolutionHigh=false` / `Settings3200` branch. Selecting `Settings3250` is prohibited unless a different operation-parameter artifact is separately hash-locked, line-registered, approved, and bound to that run. If the runtime cannot prove the registered `False` value, camera/cover execution blocks rather than defaulting to either profile.

All image/template hashes and resulting measurements must be bound to the run. Generated snapshots are runtime outputs and must not mutate captured templates.

### 10.3 Gantry park

`ControlLib.parkGantry:7071-7122` is the only startup parking authority. A generic `move to zero` is not equivalent. Implement its exact route, home/rehome predicate, thermal-door handling, and final state from the registered body.

On cover-inspection error, park before unlock as in `initializeSystem:1184-1201`. On success, park before terminal StartMode handling as in `initializeSystem:1203`.

## 11. Terminal application states

After successful movement, self-test, camera, cover, and park:

```text
StartMode=Manual/WebMode equivalent branch:
  increment progress
  doorOpen(true)
  publish ready UI/state

TradeShow:
  doorOpen(true)
  publish trade-show ready

Web/local job modes:
  doorOpen(true)
  perform bounded communication/job-admission branch
  park again after admission handling
```

General job execution is outside this tranche. The movement integration ends in a typed terminal state:

```text
oem_movement_ready_manual
oem_movement_ready_trade_show
oem_movement_ready_job_admission
oem_movement_blocked
```

`Finished initialization` may be emitted only after all mandatory stages and their postconditions are durable.

## 12. API contract

### 12.1 Fixed commands

Expose only fixed lifecycle actions:

```text
POST /oem/runtime/activate_service
POST /oem/runtime/movement-runs
GET  /oem/runtime/movement-runs/{run_id}
POST /oem/runtime/movement-runs/{run_id}/cancel
POST /oem/runtime/movement-runs/{run_id}/authorize-next-stage
GET  /oem/runtime/movement-runs/{run_id}/ledger
```

Creation request:

```json
{
  "command": "initialize_oem_movement_lifecycle",
  "operator_ack": "INITIALIZE",
  "expected_machine_serial": 206,
  "expected_registry_sha256": "8aa94083af6c4facd8f543dccad8760e7a0e49f292a21021f7a8df0d2839d0a3"
}
```

Do not accept:

```text
arbitrary stage name
axis
board
position
speed
acceleration
current
stall threshold
timeout
raw frame
source mode
```

### 12.2 Physical authorization

A run can be created and statically admitted without physical authorization. Each first-time physical commissioning stage requires a robot-side authorization bound to:

```text
run_id
next exact stage_id
expected predecessor
machine serial
ownership generation
fresh interlock snapshot
bounded expiration
operator identity/acknowledgement
```

Authorization is single-use and cannot be transferred to a different stage or retry.

Production operation after final commissioning may use an approved lifecycle policy, but must still require click-time native confirmation and fresh interlocks. It must never expose raw parameters.

### 12.3 Status response

Status must distinguish:

```text
source_authority_verified
configuration_verified
transport_owner_verified
controller_acknowledged
postcondition_verified
physical_motion_commanded
physical_effect_verified
current_stage
expected_next_stage
blocked_reason
safety_deviation
```

HTTP 200/health is not movement proof.

## 13. BMS contract

BMS is a viewer/initiator over robot-owned truth.

Required UI:

- exact machine/serial, registry, and acquisition identity;
- ownership and hardware readiness;
- fixed `Initialize OEM movement lifecycle` action;
- click-time native confirmation supplying backend `operator_ack="INITIALIZE"`;
- current, completed, expected-next, blocked, and terminal state;
- source/config references per stage;
- controller acknowledgement and physical verification separately;
- cancel only where robot reports a safe cancellation boundary;
- no arbitrary movement fields or generic “Run full startup” bypass.

BMS replay generation must equal the robot ownership generation. Stale UI data cannot authorize a stage.

Visible completion requires proof of the actual browser origin and serving worktree/API, live robot ledger rendering, and zero browser-console errors.

## 14. Implementation sequence and gates

### Gate A — authority freeze

Deliver:

- current registry JSON/Markdown/verifier committed;
- canonical evidence lock extended with registry-extension sources and `CVisionLib.dll`;
- all semantic lock tests passing;
- registry hash surfaced by runtime.

Exit:

```text
all exact hashes pass
no source-discovery placeholders
no unclassified decompiler-warning method used by enabled stages
```

### Gate B — deterministic config and transport

Deliver:

- only serial-206 closed-world snapshot accepted;
- golden request/reply fixtures for every startup movement primitive;
- exact acknowledgement normalization;
- one-writer enforcement;
- full frame/reply ledger.

Exit:

```text
fake transport proves byte order, waits, timeout, status, retries, and state changes
all negative transport cases fail closed
```

### Gate C — robot state machine in dry-run mode

Deliver:

- persistent run/stage ledger;
- exact predecessor rules;
- complete application-to-terminal graph;
- dry-run planner emits the expected full stage sequence and no physical frames;
- restart/recovery and cancellation tests.

Exit:

```text
dry-run stage graph matches the OEM call graph
no BMS-supplied movement parameters
no physical command on dry-run paths
```

### Gate D — controller non-motion acceptance

Deliver:

- ownership activation and verified snapshot;
- board identity/firmware and read-only status checks;
- exact `initializeMotorsWithoutMotion` configuration writes;
- post-write readbacks where supported;
- no movement opcode assertion.

Exit:

```text
controller acknowledgements accepted
interlocks fresh
no physical travel
ledger complete
```

### Gate E — supervised primitive commissioning

Commission one stage per authorization, in source order:

```text
Z home
gripper clearance
gripper home
X home/setHome/6000
Y home
thermal-door home/closed predicate
chiller rate writes
gripper idle current
```

For each stage:

1. frozen source review;
2. exact frame review;
3. dry-run trace;
4. fresh physical inspection/interlocks;
5. single-use authorization;
6. execute once;
7. controller acknowledgement;
8. independent readback/physical observation;
9. ledger sealing;
10. stop and investigate any discrepancy.

Exit only after every primitive has an accepted record. Do not batch first execution.

### Gate F — stale-tip and route acceptance

Deliver:

- selected route projection and collision/clearance semantics;
- tip query/eject/requery fixtures;
- clean no-tip branch acceptance;
- controlled stale-tip fixture/commissioning plan;
- pipette reinitialization/retry semantics;
- failed-eject hard stop.

Exit:

```text
no-tip and stale-tip branches both accepted
no unbounded retry
all route coordinates source-bound
```

### Gate G — self-test acceptance

Deliver:

- derived self-test maxima bound to snapshot;
- thermal/chiller worker fixtures;
- serialized shared transport under concurrent worker semantics;
- X/Y/Z/gripper threshold checks;
- 100000-ms completion timeout;
- success-date update only after complete pass.

Exit only after authorized physical self-test proof or an explicit policy that disables self-test while reporting startup as partial—not complete.

### Gate H — camera, cover, and park acceptance

Deliver:

- isolated captured CVision adapter;
- exact settings/template binding;
- camera gate and failure branch;
- cover classification fixtures;
- supervised catch/release movement;
- park on success and error;
- runtime-output/template immutability.

Exit:

```text
real camera result recorded
cover state/remediation physically verified
park postcondition verified
```

### Gate I — full lifecycle and BMS acceptance

Deliver:

- one complete authorized startup from fixed API command;
- terminal state matching selected StartMode;
- BMS live ledger rendering;
- cancellation/restart failure drills;
- frozen candidate review;
- local and deployed robot/API proof;
- browser origin/worktree proof and zero console errors;
- commit, push, and remote hash verification.

Exit:

```text
complete lifecycle passes without manual raw commands
all mandatory stages are postcondition_verified
physical_effect_verified=true for all movement stages
no hidden fallback or inferred authority path
```

## 15. Test matrix

### 15.1 Static authority

```text
registry verifier
canonical evidence-lock verifier
source/binary/config hash drift
required anchor inventory
inclusive line/declaration validation
machine serial and acquisition identity
case-sensitive camera template inventory
```

### 15.2 Unit and golden transport

For every movement primitive:

```text
exact outbound bytes
exact address/board/axis
exact request order
exact literal sleep order
reply correlation
accepted/rejected status
empty/wrong/stale reply
boundary timeout
duplicate reply
postcondition/readback
ledger record
```

### 15.3 State machine

```text
happy path
no-tip path
stale-tip path
failed tip ejection
self-test due/not due
camera enabled/disabled/failure
cover OK/missing/too many/misplaced
StartMode branches
restart during admitted/running/completed
cancel at safe/unsafe boundary
duplicate creation/idempotency
ownership-generation mismatch
interlock drift after authorization
```

### 15.4 Safety and configuration

```text
wrong serial
wrong acquisition
missing/extra/wrong-case/wrong-hash artifact
out-of-range route target
axis/board mismatch
non-fresh hardware snapshot
open enclosure
open latch
bad 24-V state
board fault
unresolved prior physical stage
unauthorized physical stage
expired/replayed authorization
```

### 15.5 Real acceptance

```text
controller acknowledgements
independent sensor/readback
operator-observed physical effect
stage-by-stage commissioning
full lifecycle replay
API state/ledger
BMS live render
browser console=0 errors
managed service deployment proof
remote Git proof
```

Passing unit tests alone is not controller or physical proof.

## 16. Failure and rollback

On any stage failure:

1. persist outbound/reply/readback/error evidence;
2. stop issuing subsequent movement frames;
3. issue only the source-authorized abort/stop sequence appropriate to the active primitive;
4. preserve one-writer ownership unless releasing it is proven safer;
5. mark run blocked;
6. require physical inspection and new authorization before any retry;
7. never auto-home or reset references as generic recovery;
8. never discard the failed ledger or overwrite it with a clean retry.

Software rollback restores the prior reviewed runtime and API/BMS surface. It does not claim to restore physical state. Physical state remains blocked until inspected.

The existing blockers remain until their exact replacements pass their gates:

```text
motor_oem_initialize_motors_full_sequence
ControlLib.rehome adapted wrapper
blocked_reason=literal_direct_oem_stage_rewrite_pending
```

Do not simply remove these blockers; replace them with the accepted state-machine stages.

## 17. Definition of done

Movement OEM parity is finished only when all are true:

- registry and canonical lock pass from the frozen candidate;
- all registered startup source families are consumed or explicitly out of scope;
- serial-206 snapshot is the only machine authority;
- one writer owns transport;
- every enabled stage has exact source/binary/config bindings;
- every primitive has golden frame/reply fixtures;
- full persistent stage ledger survives restart;
- BMS exposes only fixed lifecycle controls;
- all fake-transport and negative tests pass;
- controller non-motion acceptance passes;
- each physical primitive is separately commissioned and verified;
- stale-tip, self-test, camera, cover, park, and terminal branches are accepted;
- one complete startup reaches the correct terminal state;
- physical and controller proof is present, not inferred from HTTP success;
- deployed API/BMS and browser origin are verified;
- zero browser-console errors;
- frozen review passes;
- commit is pushed and local/remote hashes match.

Until then, status must name the highest completed gate and exact blocker. It must not report a made-up global completion percentage.

## 18. Immediate implementation order

The implementation team executes this exact order:

```text
1. Extend canonical evidence lock from the completed registry.
2. Add registry/runtime hash enforcement.
3. Add persistent run/stage/frame ledger.
4. Extract golden transport vectors from all enabled registered primitives.
5. Implement deterministic transport and board reply semantics.
6. Implement full state machine in dry-run mode.
7. Accept non-motion controller configuration.
8. Commission literal initializeMotors stages one at a time.
9. Accept stale-tip route/eject/reinitialize behavior.
10. Accept optional self-test.
11. Accept camera/cover/park.
12. Wire fixed API and BMS state presentation.
13. Run full lifecycle acceptance.
14. Freeze, review, deploy, verify, commit, and push.
```

This is one governed delivery tranche. Gates prevent unsafe promotion; they do not create separate speculative projects or reopen source acquisition.