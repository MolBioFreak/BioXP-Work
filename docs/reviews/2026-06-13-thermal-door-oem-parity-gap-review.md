# BioXP Thermal Door OEM-Parity Code Review / Gap Analysis

Date: 2026-06-13
Scope: thermal-door motion/homing/open/close parity against the OEM SSD/decompiled source.

## Summary

Current Linux thermal-door open/close behavior is **not OEM-spec**. Homing is partially OEM-shaped (`doorSearchHome`), but open/close currently rely on generic absolute moves or diagnostic-menu constants. The prior `Door=10750` move was **not OEM spec**; it came from a Linux diagnostic helper and should be treated as partial diagnostic movement only.

## OEM source evidence

### Settings / ranges

Source: `decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs`

- `m_TCDoorOpen = 16000` (line 266)
- `m_TCDoorStallGuardThreshold = 6` (line 308)
- `m_TC_DOOR_VELOCITY = 50` (line 310)
- `m_TC_DOOR_ACCELERATION = 20` (line 312)
- `m_TC_DOOR_MAX_CURRENT = 31` (line 314)
- Serial branch:
  - serial `<10`: velocity `900`, `TCDoorOpen=93000` (lines 779-780)
  - serial `>=10`: velocity `50`, `TCDoorOpen=16000` (lines 784-785)

For this BioXP3200 class, source default is therefore **TCDoorOpen=16000**, not 10750.

### Initialization/setup

Source: `decompiled_src/BioXPControlLib/ClassControlInterface.cs`

- thermal door setup calls `setSpeedAcc(TC_DOOR_VELOCITY, TC_DOOR_ACCELERATION)` (line 3246)
- `setMaxCurrent(TC_DOOR_MAX_CURRENT)` (line 3248)
- `setStallGuardThreshold(TCDoorStallGuardThreshold)` (line 3250)
- `disableRightSwitch` and `disableLeftSwitch` (lines 3252, 3254)

Door masks are explicit OEM writes; this is different from X/Z/G no-write mask preservation.

### Homing/close reference

Source: `ClassControlInterface.cs`

- `initializeMotors()` calls `doorSearchHome(ThermalDoor, TC_DOOR_VELOCITY, TCDoorStallGuardThreshold)` (line 3382)
- manual D-home also calls `doorSearchHome(...)` (line 1231)

Source: `decompiled_src_can/ClassCanLib/ClassThermalBoard.cs`

- if already home or serial `<10`, raise stallguard and `moveSteps(axis, 2000)` preclear (lines 375-379)
- restore threshold and `moveLeft(axis, speed)` (lines 380-381)
- wait for motor stop, stop motor (lines 382-397)
- if `queryHome(axis)`, `setHome(axis)` (lines 398-401)
- if calibrated and home not found, throw `Failed to find door home` (lines 406-408)

### Open/close

Source: `ClassControlInterface.cs`

`openThermalDoor()`:

- set stallguard `TCDoorStallGuardThreshold + 2` (line 2655)
- set max current `TC_DOOR_MAX_CURRENT` (line 2656)
- `moveToAbs(axis, m_settingsWindow.TCDoorOpen, ...)` (line 2657)
- then evaluate `tcDoorClosed` and `tcDoorOpened` (lines 2660-2661)

`closeThermalDoor()`:

- if `tcDoorOpened`, set stallguard/current (lines 2681-2686)
- `moveToAbs(axis, 0, ...)` (line 2687)
- then evaluate closed/open predicates (lines 2690-2691)

Predicates:

- `tcDoorOpened` -> `queryRightSensor(ThermalDoor)` (lines 2748-2751)
- `tcDoorClosed` -> `queryHome(ThermalDoor)` (lines 2754-2757)

## Current Linux implementation review

### Current positives

- Door board/motor mapping exists: thermal/CAN6 motor 0.
- Door switch masks are explicit both-side writes in `MOTOR_FUNCTION_PRESETS["door"]`.
- `motor_oem_door_search_home()` models a doorSearchHome-like flow.
- `/motion/oem/startup_step {"step":"door-home"}` exists.
- `oem_homing_model.py` records `door.setup` and `door.doorSearchHome` traces.

### Critical gaps

1. **Door preset values are wrong.** Current Linux door preset uses `speed=1500`, `acc=200`, `run_current=20`; OEM serial >=10 defaults are `speed=50`, `acc=20`, `run_current=31`.

2. **`10750` and `-7000` are not OEM.** They are diagnostic constants in `run_thermal_door_menu()`, not found in OEM source as open/close specs.

3. **No first-class OEM open route.** Linux lacks `openThermalDoor()` parity: threshold+2, current 31, move to `TCDoorOpen`, verify `tcDoorOpened`.

4. **No first-class OEM close route.** Linux lacks `closeThermalDoor()` parity: only close if open predicate is true, move to 0, verify `tcDoorClosed`.

5. **`TCDoorOpen` not modeled in config.** `oem_config.py` captures door velocity/current/stallguard but not `TCDoorOpen` nor serial-class branch.

6. **Predicate semantics not enforced for open/close success.** Generic absolute route can report coordinate movement without confirming `tcDoorOpened`/`tcDoorClosed`.

7. **Rehome door-state restore gap is now actionable.** `oem_homing_model.py` labels door-state restore as missing. With `openThermalDoor`/`closeThermalDoor` modeled, save/restore can be implemented after approval.

## Risk review

- Do not repeat `door=10750` as “open.”
- Do not discover door range with arbitrary large absolute moves.
- Do not restart the API without explicit approval.
- Live door validation must be staged: passive state -> door home -> OEM open target -> OEM close, with camera/operator observation.

## Review verdict

Current code is **not approved for full thermal-door OEM operation**. The next step should be an implementation PR with source-backed settings, explicit door open/close/home operations, predicate verification, and tests before any live runtime reload.
