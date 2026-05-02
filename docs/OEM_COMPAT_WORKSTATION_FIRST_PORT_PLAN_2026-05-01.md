# BioXP3200 OEM Compatibility Runtime — Workstation-First Port Plan

Date: 2026-05-01
Scope: direct Linux compatibility port of the critical OEM/native control surface on the workstation first; deploy/post to robot only after dry-run/parity tests pass. No live robot actuation in this phase.

## Direction

Christian wants the Windows/native BioXP app's existing control surface converted to Linux compatibility, not a Windows VM, not Wine-as-runtime, and not a new BMS-first redesign.

The implementation target is an OEM-compatible Linux runtime that preserves the native API semantics:

- transport/reply matching from `NovoCANUSBLib`
- board/motor APIs from `ClassCanLib`
- machine motion API from `ClassControlInterface`
- high-level orchestration from `ControlLib`
- OEM script grammar from `ClassBioXPScriptHandler`
- semantic state from `ClassVirtualBioXP` / `ClassMachineStatus`
- pipette/liquid API from `ClassPipette` / `ClassPipetteCollection`

The robot is not the development target until workstation dry-run and frame-sequence tests are green.

## Current repo state at plan creation

Workstation repo:

`/home/dalab/Desktop/BioXP 3200 Development Work/bioxp_re`

Branch:

`test`

Last commit observed:

`f7dd6f1 fix: harden BioXP status polling and OEM homing`

Important: the working tree is already dirty with active changes in:

- `README.md`
- `docs/README.md`
- `src/bioxp/api.py`
- `src/bioxp/usb_driver.py`
- `src/bioxp/can_driver.py`
- `src/bioxp/pipette/*`
- `src/bioxp/services/pipette_service.py`
- `src/bioxp/services/protocol_service.py`
- multiple tests

So the port must be additive and carefully staged. Do not bulldoze existing code.

## OEM source anchors

OEM backup:

`/home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup`

Critical source anchors:

- `decompiled_src_novo/NovoCANUSBLib/ClassNovoCANUSB.cs`
- `decompiled_src_can/ClassCanLib/ClassNovo.cs`
- `decompiled_src_can/ClassCanLib/ClassMotor.cs`
- `decompiled_src_can/ClassCanLib/ClassBaseBoard.cs`
- `decompiled_src_can/ClassCanLib/ClassDeckBoard.cs`
- `decompiled_src_can/ClassCanLib/ClassHeadBoard.cs`
- `decompiled_src_can/ClassCanLib/ClassThermalBoard.cs`
- `decompiled_src_can/ClassCanLib/ClassChillerBoard.cs`
- `decompiled_src/BioXPControlLib/ClassControlInterface.cs`
- `decompiled_src/BioXPControlLib/ControlLib.cs`
- `decompiled_src/BioXPControlLib/ClassPipetteCollection.cs`
- `decompiled_src_can/BioXPControlLib/ClassPipette.cs`
- `decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPScriptHandler.cs`
- `decompiled_src_bioxpcommon/BioXPCommonLib/ClassVirtualBioXP.cs`
- `decompiled_src_bioxpcommon/BioXPCommonLib/ClassMachineStatus.cs`
- `decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs`

Supporting reports already written:

- `/home/dalab/Desktop/BioXP 3200 Development Work/reports/BIOXP3200_OEM_CODEX_DEAD_SCRATCH_REVIEW_2026-05-01.md`
- `/home/dalab/Desktop/BioXP 3200 Development Work/reports/BIOXP3200_NATIVE_APP_CONTROL_SURFACE_LINUX_COMPAT_REVIEW_2026-05-01.md`

## Implementation shape

Create a new additive compatibility package inside the current repo:

```text
src/bioxp/oem_compat/
  __init__.py
  transport.py
  frames.py
  motor.py
  boards.py
  control_interface.py
  control_lib.py
  scripts.py
  state.py
  pipette.py
  vision.py
  modes.py
```

This package should not immediately replace `src/bioxp/usb_driver.py` or `src/bioxp/api.py`. It should first become a tested compatibility core that existing API code can wrap later.

Runtime modes:

- `dry_run`: default; emits frames and semantic actions only; no USB/CAN device access
- `shadow`: safe status/query only; records hardware traffic; no motion/liquid actuation
- `live`: explicit hardware access; operator acknowledgement required; artifact recording mandatory

## Critical definition of “worked out on workstation”

A port slice is workstation-complete when:

1. It has tests written first.
2. Tests run without robot hardware.
3. It can emit an OEM-compatible semantic trace and/or low-level frame trace.
4. The trace is grounded in decompiled OEM methods.
5. It does not claim physical success from controller counters.
6. It has no live USB/CAN side effects by default.

Only after that do we post/sync to the robot for supervised physical validation.

## Phase 1 — Dry-run transport + frame/event model

Goal: create the compatibility skeleton that can represent OEM command traffic without touching hardware.

Files to add:

- `src/bioxp/oem_compat/frames.py`
- `src/bioxp/oem_compat/transport.py`
- `src/bioxp/oem_compat/modes.py`
- `tests/test_oem_compat_transport.py`

Port concepts:

- `InterfaceCAN.TransmitMessage`
- `ClassNovoCANUSB.sendCommand`
- `ClassNovoCANUSB.transmitCommand`
- reply matching categories:
  - motion/CAN ID 4-9
  - CANOpen/UIM ID 0 or >512
  - pipette IDs otherwise
- events:
  - motion message
  - pipette message
  - enclosure door
  - latch
  - board error
  - raw CAN traffic

Acceptance tests:

- dry-run transport records outgoing SIDH/SIDL/CMD/message/timeout
- recording transport preserves event category
- no transport opens USB in dry-run mode
- transmit returns deterministic synthetic ACK only when configured

## Phase 2 — ClassMotor command compatibility

Goal: port low-level motor command builders before moving any axis.

Files:

- `src/bioxp/oem_compat/motor.py`
- `tests/test_oem_compat_motor.py`

Port first:

- `setMaxSpeed`
- `setMaxAcc`
- `setMaxCurrent`
- `readMaxCurrent`
- `setHome`
- `MoveHome`
- `MovetoRelPosition`
- `moveToAbs`
- `StopMotor`
- `queryActualPosition`
- `queryMotorSpeed`
- `queryReachedPosition`
- `queryLeftSwitchStatus`
- `queryRightSwitchStatus`
- `setStallGuardThreshold`
- `setLimits`

Acceptance tests:

- each method emits the same logical OEM command tuple: board ID, axis, command, parameter, value
- signed steps/positions are encoded consistently
- run-current/standby-current parameter numbers match the current `usb_driver.py` constants and OEM source comments

Note: if exact bytes are ambiguous from decompiled code, test named command tuples first, then add byte-exact tests after source extraction.

## Phase 3 — Board classes

Goal: map OEM board classes onto Linux-compatible board objects.

Files:

- `src/bioxp/oem_compat/boards.py`
- `tests/test_oem_compat_boards.py`

Board IDs:

- head: `0x04`
- deck: `0x05`
- thermal: `0x06`
- chiller: `0x07`

Axis mapping:

- X: deck, motor 0
- Y: head, motor 0
- Z: head, motor 1
- gripper: head, motor 2
- door: thermal, motor 0

Port first:

- board construction
- axis name mapping
- activate/deactivate board commands
- board status query wrappers
- stop/abort routing
- door/latch/solenoid query wrappers

Acceptance tests:

- axis lookup is OEM-compatible
- board ID and motor index cannot drift
- thermal door is not treated as a normal XYZ axis

## Phase 4 — ClassControlInterface startup/homing

Goal: port the most critical motor-control semantics that likely explain Linux vs OEM behavior.

Files:

- `src/bioxp/oem_compat/control_interface.py`
- `tests/test_oem_compat_control_interface.py`

Port first:

- `initializeMotorsWithoutMotion`
- `initializeMotors`
- `activateBoard`
- `deactivateBoard`
- `waitForBoard`
- `HomeAxis`
- `HomeXY`
- `enableXYZ`
- `enableXY`
- `resetXYLimits`
- `forceAbortMotion`

Startup/homing requirements:

- preserve OEM current/speed/acc/stall/switch-mask order
- preserve X startup special sequence: search-home, setHome, restore speed, moveX(6000)
- preserve Z/gripper/door special behavior
- preserve door `doorSearchHome` semantics separately from generic XYZ
- emit a trace that can be diffed before motion

Acceptance tests:

- `initialize_motors_without_motion()` emits expected ordered semantic operations
- `startup_homing()` emits expected ordered operations: init, gripper pre-move, Z, G, X, X setHome/speed/move, Y, door
- no call uses live transport in default mode

## Phase 5 — Script grammar and semantic state

Goal: make original OEM XML scripts parse/translate on Linux unchanged.

Files:

- `src/bioxp/oem_compat/scripts.py`
- `src/bioxp/oem_compat/state.py`
- `tests/test_oem_compat_scripts.py`
- `tests/test_oem_compat_state.py`

Port first:

- XML root/sections: `WpfGenBotCommonLib`, `experiment`, `tips`, `reagents`, `oligos`, `script`
- script line `cmd` attributes
- verbs: `LED`, `WAIT`, `TCD`, `PP`, `MP`, `MC`, `MT`, `LA`, `FP`, `SP`, `CC`, `LOOP`, `DWELL`, `ET`, `DELAYPOINT`
- location/plate/well/token enums enough to dry-run shipped scripts

Acceptance tests:

- parse `Scripts/demo.xml`
- parse `Scripts/tp506.xml`
- return line-numbered commands
- preserve unknown commands as explicit unsupported records, not silent no-ops

## Phase 6 — ControlLib compatibility facade

Goal: expose the native app’s high-level API as a Linux runtime facade.

Files:

- `src/bioxp/oem_compat/control_lib.py`
- `tests/test_oem_compat_control_lib.py`

Port first:

- `initial_check`
- `initialize_motion`
- `startup`
- `selftest`
- `run_job` dry-run only
- `execute_script` dry-run only
- `script_interpreter` dry-run only
- `pause_script`
- `resume_job`
- `stop_script`
- `cleanup`

Acceptance tests:

- startup calls `BioXPControlInterface` in OEM order
- script dry-run produces semantic actions only
- stop/abort produce safe event/state changes

## Phase 7 — Pipette critical compatibility

Goal: port the pipette surface enough to avoid TX-only fake success.

Files:

- `src/bioxp/oem_compat/pipette.py`
- `tests/test_oem_compat_pipette.py`

Port first:

- `initiate_group`
- `query_tip_status`
- `query_pressure`
- `query_error_log`
- `aspirate`
- `dispense`
- `mix_all`
- `eject_all_tips`
- `detect_fluid`

Acceptance tests:

- commands route through pipette transport category, not motor category
- operations require ACK/readback models
- dry-run reports “planned” not “done”

## Phase 8 — Vision/barcode facade

Goal: provide compatible call surfaces without pretending full CV parity exists.

Files:

- `src/bioxp/oem_compat/vision.py`
- `tests/test_oem_compat_vision.py`

Port first:

- `scan_barcode`
- `snapshot_image`
- `check_camera`
- inspection method stubs that return explicit `UNAVAILABLE` in dry-run unless a real implementation is configured

Acceptance tests:

- unsupported CV operations are explicit and typed
- script/job flow can call vision methods without crashing or faking success

## Phase 9 — API adapter, still workstation-first

Only after compatibility core tests pass, add a thin adapter in existing FastAPI surface:

- `src/bioxp/api.py`
- possibly `src/bioxp/services/*`

Add endpoints under a clearly marked compatibility namespace first:

- `/oem-compat/status`
- `/oem-compat/startup/dry-run`
- `/oem-compat/scripts/parse`
- `/oem-compat/scripts/translate`
- `/oem-compat/motion/initialize/dry-run`
- `/oem-compat/motion/home/dry-run`

Do not bind these to live hardware initially.

## Phase 10 — Post/sync to robot later

Robot deployment only after workstation criteria:

- all oem_compat tests pass
- existing motion/pipette/protocol tests still pass or failures are documented dependency/env failures
- startup/homing dry-run trace is saved as artifact
- no live mode default
- operator ack required for live

Robot post/deploy checklist later:

1. rsync/git sync to robot-local repo.
2. Restart robot-local service only if needed.
3. Query safe GETs:
   - `/openapi.json`
   - `/status`
   - `/motion/power/status`
   - `/motion/reference/status`
   - `/liquid/status`
4. Exercise only dry-run compatibility endpoints first.
5. Save artifacts under `/mnt/BioModStack/bms_results/bioxp_validation/<timestamp>/`.
6. Only then plan supervised motor-control test.

## Immediate next code slice

Start with Phase 1 + a small piece of Phase 2 using strict TDD:

1. Add failing tests for `DryRunTransport` and `OemCommandFrame`.
2. Verify tests fail because modules do not exist.
3. Add minimal modules to pass.
4. Add failing tests for one `Motor.set_max_speed()` and one `Motor.set_max_current()` command tuple.
5. Implement minimal motor methods.
6. Run focused tests.
7. Run existing safe tests:
   - `tests/test_bioxp_oem_homing.py`
   - `tests/test_motion_phase1.py`
   - `tests/test_usb_reconnect.py`

## Non-goals for workstation phase

- no live robot motion
- no Windows VM
- no Wine control path
- no pretending a dry-run trace proves physical success
- no BMS UI work until compatibility core exists
- no broad rewrite of existing `usb_driver.py` before parity scaffold is tested

## Success definition before first robot motor test

Before first post-port robot motor-control test, we need:

- OEM-compatible dry-run transport and trace recorder
- motor command compatibility tests
- board/axis map tests
- `initializeMotorsWithoutMotion` trace test
- `startup_homing` trace test
- explicit live-mode gate
- artifact output path
- supervised test script that performs safe status snapshot before any movement

Then the robot test can be a controlled validation, not experimentation.
